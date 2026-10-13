"""stream_node: an image topic as low-latency H.264 / MPEG-TS over plain TCP.

streamer.py from upd-rasp-stream-tcp, fed from ROS instead of picamera2's
encoder. By default it serves /vision/debug_image, so a viewer sees the
camera with the QR outline and readout burnt in, exactly as before:

    image topic -> libx264 (PyAV, slice threads, zerolatency)
                -> one MPEG-TS muxer per viewer -> TCP 0.0.0.0:8554

    ffplay -fflags nobuffer -flags low_delay -framedrop tcp://<IP_Pi>:8554

Nothing is encoded while no one is connected. A viewer that joins starts at
the next keyframe (at most `keyframe_interval` seconds). Optional: inside
ROS, rqt_image_view on the same topic needs none of this.
"""

import collections
from fractions import Fraction
import socket
import threading

import av
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from roboarm_nodes.vision import pixels, rgb

MICROSECONDS = Fraction(1, 1_000_000)


class Encoder:
    """libx264 set up the way LowLatencyH264Encoder set it up: slice threads.

    Frame threading holds one frame per thread before the first packet
    (measured on a Pi 5 at 720p: 4 threads = 100 ms); slice threading emits
    each frame's packet as soon as it is encoded.
    """

    def __init__(self, width, height, fps, bitrate, threads, keyframe_interval):
        self.context = av.CodecContext.create('libx264', 'w')
        self.context.width, self.context.height = width, height
        self.context.pix_fmt = 'yuv420p'
        self.context.time_base = MICROSECONDS
        self.context.framerate = Fraction(fps).limit_denominator(1000)
        self.context.bit_rate = bitrate
        self.context.gop_size = max(1, round(fps * keyframe_interval))
        self.context.options = {'preset': 'ultrafast', 'tune': 'zerolatency'}
        self.context.thread_type = 'SLICE'
        self.context.thread_count = threads

    def encode(self, image, pts):
        frame = av.VideoFrame.from_ndarray(image, format='rgb24').reformat(format='yuv420p')
        frame.pts = pts
        frame.time_base = MICROSECONDS
        return self.context.encode(frame)


class _Sink:
    """File-like object a PyAV container writes into; collects the bytes."""

    def __init__(self):
        self._buffer = bytearray()

    def write(self, data):
        self._buffer += data
        return len(data)

    def take(self):
        data = bytes(self._buffer)
        self._buffer.clear()
        return data


class _Viewer:
    """One connected client: its socket, its own muxer and its send queue.

    Each viewer gets a muxer of its own rather than a share of one common
    byte stream, so that every viewer's stream starts cleanly - tables
    first, then a keyframe - no matter when it connected.
    """

    def __init__(self, sock, addr):
        self.sock = sock
        self.addr = addr
        self.queue = collections.deque()
        self.cond = threading.Condition()
        self.dead = False
        self.dropping = False
        # Touched only on the executor thread (and in stop(), after it is done).
        self.container = None
        self.stream = None
        self.sink = None


class TcpTsServer:
    """Serves encoded H.264 packets as MPEG-TS over TCP, any number of viewers.

    The image callback only muxes (microseconds, all in memory) and appends
    to each viewer's short queue; a sender thread per viewer does the
    blocking socket write. A viewer that cannot keep up loses frames, never
    the node: once its queue is full we drop, and once we have dropped
    anything we keep dropping until the next keyframe, so it resyncs cleanly
    on an IDR instead of decoding into a mess of missing references.
    """

    def __init__(self, host, port, log, depth=5):
        # depth is deliberately shallow - five frames. Whatever is sitting in
        # a queue is stale video delivered before anything live.
        self.host = host
        self.port = port
        self.log = log
        self.depth = depth
        self.rate = None
        self.size = None
        self.running = False
        self._viewers = []
        self._viewers_lock = threading.Lock()
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((host, port))
        self._listener.listen(8)
        self.running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True, name='tcp-accept')
        self._accept_thread.start()

    @property
    def viewers(self):
        with self._viewers_lock:
            return sum(1 for viewer in self._viewers if not viewer.dead)

    def reset(self, size, rate):
        """New stream parameters: every viewer restarts its muxer on the next keyframe."""
        self.size, self.rate = size, rate
        with self._viewers_lock:
            viewers = list(self._viewers)
        for viewer in viewers:
            if viewer.container is not None:
                viewer.container.close()
                viewer.container = None

    def stop(self):
        self.running = False
        try:
            # Closing the listener is what wakes the accept loop out of accept().
            self._listener.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._listener.close()
        self._accept_thread.join(timeout=2.0)
        with self._viewers_lock:
            viewers, self._viewers = self._viewers, []
        for viewer in viewers:
            self._drop(viewer)

    def send(self, data, keyframe, timestamp):
        with self._viewers_lock:
            viewers = list(self._viewers)
        for viewer in viewers:
            if viewer.dead:
                with self._viewers_lock:
                    if viewer in self._viewers:
                        self._viewers.remove(viewer)
                self._drop(viewer)
                continue
            self._feed(viewer, data, keyframe, timestamp)

    # ---- executor thread ------------------------------------------------

    def _feed(self, viewer, data, keyframe, timestamp):
        if viewer.container is None:
            if not keyframe:
                return          # a stream has to start on a keyframe
            self._open(viewer)

        with viewer.cond:
            if viewer.dropping and not keyframe:
                return
            if len(viewer.queue) >= self.depth:
                if not keyframe:
                    viewer.dropping = True
                    return
                # A keyframe arrived with the queue still full: throw the
                # backlog away and restart the stream from here.
                viewer.queue.clear()
            viewer.dropping = False

        out = av.Packet(data)
        out.pts = out.dts = timestamp
        out.time_base = MICROSECONDS
        out.is_keyframe = keyframe
        out.stream = viewer.stream
        try:
            viewer.container.mux(out)
        except Exception as error:  # noqa: BLE001 - one bad viewer must not stop the rest
            self.log(f'viewer {viewer.addr[0]}: mux failed: {error}')
            viewer.dead = True
            return
        # flush_packets makes the muxer hand over every packet at once, so
        # this is exactly one frame's worth of whole 188-byte TS packets.
        chunk = viewer.sink.take()
        if chunk:
            with viewer.cond:
                viewer.queue.append(chunk)
                viewer.cond.notify()

    def _open(self, viewer):
        viewer.sink = _Sink()
        viewer.container = av.open(
            viewer.sink, 'w', format='mpegts',
            # flush_packets: write every TS packet out straight away, rather
            # than waiting for libav's IO buffer to fill. max_delay=0: no
            # muxing delay added before the first packet.
            container_options={'flush_packets': '1', 'max_delay': '0'})
        viewer.stream = viewer.container.add_stream('h264', rate=self.rate)
        viewer.stream.width, viewer.stream.height = self.size

    def _drop(self, viewer):
        viewer.dead = True
        with viewer.cond:
            viewer.cond.notify()
        try:
            viewer.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        if viewer.container is not None:
            try:
                viewer.container.close()
            except Exception:  # noqa: BLE001
                pass
            viewer.container = None

    # ---- network threads ------------------------------------------------

    def _accept_loop(self):
        while self.running:
            try:
                conn, addr = self._listener.accept()
            except OSError:
                return          # listener closed by stop()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            viewer = _Viewer(conn, addr)
            with self._viewers_lock:
                self._viewers.append(viewer)
            threading.Thread(target=self._send_loop, args=(viewer,), daemon=True, name='tcp-viewer').start()
            self.log(f'viewer connected from {addr[0]}')

    def _send_loop(self, viewer):
        try:
            while True:
                with viewer.cond:
                    while not viewer.queue and not viewer.dead:
                        viewer.cond.wait()
                    if viewer.dead:
                        return
                    chunk = viewer.queue.popleft()
                try:
                    viewer.sock.sendall(chunk)
                except OSError:
                    return
        finally:
            # The muxer belongs to the executor thread; it is closed there,
            # the next time send() sees this viewer is dead.
            viewer.dead = True
            try:
                viewer.sock.close()
            except OSError:
                pass
            self.log(f'viewer from {viewer.addr[0]} disconnected')


class Stream(Node):
    def __init__(self):
        super().__init__('stream_node')
        defaults = {'image_topic': '/vision/debug_image', 'host': '0.0.0.0', 'port': 8554,
                    'fps': 15.0, 'bitrate': 2_000_000, 'threads': 4, 'keyframe_interval': 1.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        if not 0 < param('fps') <= 60 or param('bitrate') <= 0 or param('threads') <= 0:
            raise ValueError('fps, bitrate and threads must be positive')
        self.encoder = None
        self.size = None
        self.start_ns = None
        self.server = TcpTsServer(param('host'), param('port'), self.get_logger().info)
        topic = param('image_topic')
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'Serving {topic} on tcp://{param("host")}:{param("port")}\n  '
                               f'ffplay -fflags nobuffer -flags low_delay -framedrop tcp://<IP_Pi>:{param("port")}')

    def on_image(self, msg):
        if not self.server.viewers:
            # Nobody watching: drop the encoder, and the next viewer gets a
            # fresh one whose first frame is a keyframe.
            self.encoder = None
            return
        try:
            image = rgb(msg, pixels(msg))
        except ValueError as exc:
            self.get_logger().warning(f'Frame skipped: {exc}', throttle_duration_sec=5.0)
            return
        # yuv420p needs even dimensions.
        image = image[:msg.height & ~1, :msg.width & ~1]
        size = (image.shape[1], image.shape[0])
        if self.encoder is None or size != self.size:
            fps = self.get_parameter('fps').value
            self.encoder = Encoder(*size, fps, self.get_parameter('bitrate').value,
                                   self.get_parameter('threads').value,
                                   self.get_parameter('keyframe_interval').value)
            self.size = size
            self.start_ns = None
            self.server.reset(size, Fraction(fps).limit_denominator(1000))
        now_ns = self.get_clock().now().nanoseconds
        if self.start_ns is None:
            self.start_ns = now_ns
        try:
            packets = self.encoder.encode(np.ascontiguousarray(image), (now_ns - self.start_ns) // 1000)
        except av.error.FFmpegError as exc:
            self.get_logger().error(f'Encoder failed: {exc}', throttle_duration_sec=5.0)
            self.encoder = None
            return
        for packet in packets:
            self.server.send(bytes(packet), packet.is_keyframe, packet.pts)

    def destroy_node(self):
        self.server.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stream()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
