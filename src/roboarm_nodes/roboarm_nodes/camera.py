"""camera_node: owns the camera and publishes /camera/image_raw + /camera/camera_info.

In upd-rasp-stream-tcp the QR program opened the camera itself. In ROS only
this node opens it; vision_node and anything else (rqt_image_view, the TCP
stream) take frames from the topic. Backends, chosen with `backend`:

    libcamera  libcamerasrc -> videoconvert -> appsink (GStreamer). The CSI
               IMX219 on the Yocto image, where meta-vva-robot provides
               libcamera with the Pi 5 PiSP pipeline.          -> rgb8
    picamera2  Raspberry Pi OS / the old Docker image.          -> bgr8
    v4l2       USB camera through OpenCV VideoCapture.         -> bgr8
    images     A file or a folder of PNG/JPEG, looped at `fps`: tests the
               whole perception path with photos of labels, no camera.

`demo:=true` overrides all of them with a labelled synthetic frame. No
cv_bridge: messages are filled from numpy directly, which keeps the
dependency list short for a Yocto image.
"""

from fractions import Fraction
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

IMAGE_SUFFIXES = ('.png', '.jpg', '.jpeg', '.bmp')


class LibcameraSource:
    """libcamerasrc through GStreamer; the same pipeline vva-camera uses."""

    encoding = 'rgb8'

    def __init__(self, width, height, fps, camera_name='', pipeline=''):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        self.Gst = Gst
        Gst.init(None)
        if not pipeline:
            rate = Fraction(fps).limit_denominator(1000)
            source = 'libcamerasrc' + (f' camera-name="{camera_name}"' if camera_name else '')
            pipeline = (f'{source} ! video/x-raw,width={width},height={height},'
                        f'framerate={rate.numerator}/{rate.denominator} ! videoconvert ! '
                        'video/x-raw,format=RGB ! appsink name=sink max-buffers=1 drop=true sync=false')
        self.pipeline = Gst.parse_launch(pipeline)
        self.sink = self.pipeline.get_by_name('sink')
        if self.sink is None:
            raise ValueError('gst_pipeline must end in an appsink named "sink" producing RGB')
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(f'Cannot start camera pipeline: {pipeline}')

    def read(self):
        Gst = self.Gst
        message = self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
        if message is not None:
            error, debug = message.parse_error()
            raise RuntimeError(f'Camera pipeline error: {error.message} ({debug})')
        sample = self.sink.emit('try-pull-sample', Gst.SECOND)
        if sample is None:
            return None
        structure = sample.get_caps().get_structure(0)
        width, height = structure.get_value('width'), structure.get_value('height')
        buffer = sample.get_buffer()
        ok, info = buffer.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            # Rows are padded to the stride (4-byte aligned for RGB), so crop
            # each row to its real width, as the old gray_plane() did for lores.
            rows = np.frombuffer(info.data, np.uint8)
            stride = rows.size // height
            return rows[:stride * height].reshape(height, stride)[:, :width * 3] \
                .reshape(height, width, 3).copy()
        finally:
            buffer.unmap(info)

    def close(self):
        self.pipeline.set_state(self.Gst.State.NULL)


class Picamera2Source:
    """picamera2, as in the old qr_detector.py; only the main stream is needed now."""

    # picamera2's "RGB888" is laid out B, G, R in memory.
    encoding = 'bgr8'

    def __init__(self, width, height, fps):
        from picamera2 import Picamera2
        self.width, self.height = width, height
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_video_configuration(
            main={'size': (width, height), 'format': 'RGB888'},
            controls={'FrameRate': float(fps)}))
        self.camera.start()

    def read(self):
        return np.ascontiguousarray(self.camera.capture_array('main')[:self.height, :self.width, :3])

    def close(self):
        self.camera.stop()
        self.camera.close()


class V4l2Source:
    encoding = 'bgr8'

    def __init__(self, device, width, height):
        import cv2
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError('Cannot open camera; no synthetic fallback')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class PacedSource:
    """Base for sources that have no camera clock: hands out frames at `fps`."""

    def __init__(self, fps):
        self.period = 1.0 / fps
        self.next = time.monotonic()

    def read(self):
        self.next = max(self.next + self.period, time.monotonic())
        time.sleep(max(0.0, self.next - time.monotonic()))
        return self.frame()

    def close(self):
        pass


class ImagesSource(PacedSource):
    encoding = 'rgb8'

    def __init__(self, path, fps):
        from PIL import Image as PilImage
        super().__init__(fps)
        path = Path(path).expanduser()
        files = sorted(p for p in path.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES) \
            if path.is_dir() else [path]
        if not files or not files[0].is_file():
            raise RuntimeError(f'No images found at {path}')
        self.frames = [np.asarray(PilImage.open(f).convert('RGB')) for f in files]
        self.index = 0

    def frame(self):
        frame = self.frames[self.index]
        self.index = (self.index + 1) % len(self.frames)
        return frame


class DemoSource(PacedSource):
    encoding = 'rgb8'

    def __init__(self, width, height, fps):
        from PIL import Image as PilImage, ImageDraw
        super().__init__(fps)
        image = PilImage.new('RGB', (width, height))
        ImageDraw.Draw(image).text((20, 40), 'SIMULATION - NOT CAMERA DATA', fill=(255, 255, 0))
        self.image = np.asarray(image)

    def frame(self):
        return self.image


class Camera(Node):
    def __init__(self):
        super().__init__('camera_node')
        defaults = {'demo': False, 'backend': 'libcamera', 'device': 0, 'width': 640, 'height': 480,
                    'fps': 15.0, 'camera_name': '', 'gst_pipeline': '', 'images_path': '',
                    'frame_id': 'camera_optical_frame',
                    'camera_matrix': [0.0] * 9, 'distortion': [0.0] * 5}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.demo = param('demo')
        width, height, fps = param('width'), param('height'), param('fps')
        if width <= 0 or height <= 0 or not 0 < fps <= 60:
            raise ValueError('Invalid image size or fps')
        if len(param('camera_matrix')) != 9:
            raise ValueError('camera_matrix needs nine row-major values')
        backend = 'demo' if self.demo else param('backend')
        if backend == 'libcamera':
            self.source = LibcameraSource(width, height, fps, param('camera_name'), param('gst_pipeline'))
        elif backend == 'picamera2':
            self.source = Picamera2Source(width, height, fps)
        elif backend == 'v4l2':
            self.source = V4l2Source(param('device'), width, height)
        elif backend == 'images':
            self.source = ImagesSource(param('images_path'), fps)
        elif backend == 'demo':
            self.source = DemoSource(width, height, fps)
        else:
            raise ValueError('backend must be libcamera, picamera2, v4l2 or images')
        # vision_node's demo branch recognises synthetic frames by this frame id.
        self.frame_id = 'demo_camera' if self.demo else param('frame_id')
        self.images = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.infos = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.get_logger().info(f'Publishing {backend} frames on /camera/image_raw')
        # Camera reads block until the next frame, so they get a thread of
        # their own instead of a timer that would stall the executor.
        self.running = True
        self.thread = threading.Thread(target=self.capture_loop, daemon=True, name='camera')
        self.thread.start()

    def capture_loop(self):
        while self.running and rclpy.ok():
            try:
                frame = self.source.read()
            except Exception as exc:  # noqa: BLE001 - a dead camera must stop the node
                self.get_logger().fatal(f'{exc}')
                self.running = False
                rclpy.try_shutdown()  # exit non-zero below, so systemd can restart it
                return
            if frame is None:
                self.get_logger().warning('Camera frame unavailable', throttle_duration_sec=5.0)
                continue
            if not (self.running and rclpy.ok()):
                return  # Ctrl+C arrived while this thread was waiting for the frame
            try:
                self.publish(frame)
            except RuntimeError:
                if rclpy.ok():
                    raise
                return  # the context was shut down between the check and publish

    def publish(self, frame):
        msg = Image()
        # Reception time, not exposure time, as in vva-camera.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = self.source.encoding
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
        self.images.publish(msg)
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width = msg.height, msg.width
        info.distortion_model = 'plumb_bob'
        k = [float(v) for v in self.get_parameter('camera_matrix').value]
        info.k = k
        info.d = [float(v) for v in self.get_parameter('distortion').value]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]
        self.infos.publish(info)

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive() and self.thread is not threading.current_thread():
            self.thread.join(timeout=2.0)
        self.source.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        failed = not node.running
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if failed else 0
