"""vision_node: QR labels in /camera/image_raw -> part records and a target pose.

The scanning half of upd-rasp-stream-tcp, as a ROS node. It no longer opens
the camera: frames come from camera_node's topic. Per frame:

    zbar (qr.Scanner)  -> every code in view
      new read         -> /vision/qr (JSON) and, for part links,
                          iFurn -> parts.csv -> /vision/part (JSON), off-thread
      allowed code     -> solvePnP on its four corners (code side `qr_size`)
                          -> TF into `target_frame` -> /vision/target_pose
    overlay (Osd)      -> /vision/debug_image, only while someone subscribes

A code becomes a target only if its payload matches `target_pattern`: the
old handle_code() acted on anything in frame, Wikipedia links included, and
a moving arm should not. The pose needs OpenCV and a calibrated
/camera/camera_info; without them codes are still read and recorded.
"""

import json
import math
import os
import re
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped

from roboarm_nodes.overlay import Osd
from roboarm_nodes.parts import PartRecorder
from roboarm_nodes.qr import Scanner

# Integer luma weights (BT.601, /256): the cost of turning an RGB frame into
# the Y800 plane zbar reads, without a float pass over every pixel.
LUMA = {'rgb8': (77, 150, 29), 'rgba8': (77, 150, 29), 'bgr8': (29, 150, 77), 'bgra8': (29, 150, 77)}
CHANNELS = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}


def pixels(msg):
    """Image message -> height x width x channels uint8 array, row padding removed."""
    channels = CHANNELS.get(msg.encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding {msg.encoding}')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    return rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)


def gray(msg, frame):
    if msg.encoding == 'mono8':
        return np.ascontiguousarray(frame[..., 0])
    r, g, b = LUMA[msg.encoding]
    # Widen first: NumPy 1.x keeps uint8 * small-scalar in uint8 and wraps.
    channel = lambda i: frame[..., i].astype(np.uint16)
    plane = channel(0) * r + channel(1) * g + channel(2) * b
    return (plane >> 8).astype(np.uint8)


def rgb(msg, frame):
    if msg.encoding == 'mono8':
        return np.repeat(frame, 3, axis=2)
    if msg.encoding.startswith('bgr'):
        return np.ascontiguousarray(frame[..., 2::-1])
    return np.ascontiguousarray(frame[..., :3])


def clockwise(corners):
    """The four corners in on-screen clockwise order, top-left-most first.

    zbar's polygon order is its own; solvePnP needs the order of the model
    points below. A cyclic shift only turns the pose about the code's normal,
    which leaves the centre - the only part used - where it is.
    """
    points = np.asarray(corners, dtype=np.float64)
    centre = points.mean(axis=0)
    # Image y points down, so increasing atan2 runs clockwise on screen.
    points = points[np.argsort(np.arctan2(points[:, 1] - centre[1], points[:, 0] - centre[0]))]
    return np.roll(points, -int(np.argmin(points.sum(axis=1))), axis=0)


def area(corners):
    x, y = np.asarray(corners, dtype=np.float64).T
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        defaults = {
            'demo': False, 'target_frame': 'base_link', 'marker_to_tool_z': 0.0,
            'demo_radius': 0.3, 'demo_height': 0.1,
            'qr_size': 0.04, 'target_pattern': r'^https?://(2qi\.us|[\w.-]*ifurn\.pro)/',
            'max_reprojection_px': 3.0, 'repeat_after': 5.0,
            'record_parts': True, 'parts_csv': '~/.local/share/roboarm/parts.csv', 'fetch_timeout': 10.0,
            'osd_hold': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.demo = param('demo')
        if not 0 < param('qr_size') < 1:
            raise ValueError('qr_size must be positive, in metres')
        self.target_pattern = re.compile(param('target_pattern'))
        self.info = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(PoseStamped, '/vision/target_pose', 10)
        self.qr_publisher = self.create_publisher(String, '/vision/qr', 10)
        self.part_publisher = self.create_publisher(String, '/vision/part', 10)
        self.debug_publisher = self.create_publisher(Image, '/vision/debug_image', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.scanner = Scanner()
        self.last_seen = {}
        self.osd = None
        self.cv2 = None
        if not self.demo:
            try:
                import cv2
                self.cv2 = cv2
            except ImportError:
                self.get_logger().error('OpenCV is missing: QR codes are read and recorded, '
                                        'but no /vision/target_pose is published')
        self.recorder = None
        if not self.demo and param('record_parts'):
            path = os.path.expanduser(param('parts_csv'))
            self.recorder = PartRecorder(path, timeout=param('fetch_timeout'),
                                         on_row=self.on_part, log=self.log).start()
            self.get_logger().info(f'Part links are recorded in {path}')

    def log(self, level, text):
        getattr(self.get_logger(), level)(f'[part] {text}')

    def on_part(self, row):
        # Runs on the recorder thread; publishing from there is safe in rclpy.
        self.part_publisher.publish(String(data=json.dumps(row, ensure_ascii=False)))

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        if self.demo:
            if msg.header.frame_id != 'demo_camera':
                return
            target = PoseStamped()
            target.header.stamp = msg.header.stamp
            target.header.frame_id = 'base_link'
            target.pose.position.x = self.get_parameter('demo_radius').value
            target.pose.position.z = self.get_parameter('demo_height').value + 0.05
            target.pose.orientation.w = 1.0
            self.publisher.publish(target)
            return
        try:
            frame = pixels(msg)
            codes = self.scanner.scan_gray(gray(msg, frame), msg.width, msg.height)
        except ValueError as exc:
            self.get_logger().warning(f'Frame skipped: {exc}', throttle_duration_sec=5.0)
            return
        for code in codes:
            if self.is_new(code):
                self.report(code, msg)
        targets = [c for c in codes if len(c.corners) == 4 and self.target_pattern.search(c.text)]
        if targets:
            # The largest code in view is the nearest one: that is the part to pick.
            self.publish_target(max(targets, key=lambda c: area(c.corners)), msg)
        if self.debug_publisher.get_subscription_count():
            self.publish_debug(msg, frame, codes)

    def is_new(self, code):
        """True once per payload, then again only after it has been out of
        frame for `repeat_after` seconds."""
        now = time.monotonic()
        previous = self.last_seen.get(code.raw)
        self.last_seen[code.raw] = now
        return previous is None or now - previous >= self.get_parameter('repeat_after').value

    def report(self, code, msg):
        where = ' at {},{} {}x{}px'.format(*code.bbox) if code.bbox else ''
        self.get_logger().info(f'{code.kind} q={code.quality}{where} -> {code.text}')
        self.qr_publisher.publish(String(data=json.dumps({
            'text': code.text, 'kind': code.kind, 'quality': code.quality,
            'corners': code.corners, 'frame_id': msg.header.frame_id,
            'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
            'target': bool(self.target_pattern.search(code.text)),
        }, ensure_ascii=False)))
        if self.recorder is not None:
            self.recorder.submit(code.text)

    def publish_target(self, code, msg):
        cv2 = self.cv2
        if cv2 is None:
            return
        info = self.info
        if info is None or info.k[0] <= 0 or info.k[4] <= 0:
            self.get_logger().warning('QR target seen, waiting for calibrated /camera/camera_info',
                                      throttle_duration_sec=10.0)
            return
        if info.width != msg.width or info.height != msg.height or info.header.frame_id != msg.header.frame_id:
            return
        try:
            half = self.get_parameter('qr_size').value / 2
            points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
                              dtype=np.float64)
            corners = clockwise(code.corners)
            k = np.array(info.k, dtype=np.float64).reshape(3, 3)
            d = np.array(info.d, dtype=np.float64)
            ok, rotation, translation = cv2.solvePnP(points, corners, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or not np.isfinite(translation).all() or translation[2, 0] <= 0:
                return
            projected, _ = cv2.projectPoints(points, rotation, translation, k, d)
            error = np.linalg.norm(projected.reshape(4, 2) - corners, axis=1).mean()
            if error > self.get_parameter('max_reprojection_px').value:
                self.get_logger().warning(f'QR pose rejected, reprojection error {error:.1f}px',
                                          throttle_duration_sec=5.0)
                return
            marker = PoseStamped()
            marker.header = msg.header
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, translation.flatten())
            marker.pose.orientation.w = 1.0  # Only the code's centre is used, not the part's orientation.
            transform = self.tf.lookup_transform(self.get_parameter('target_frame').value,
                                                 msg.header.frame_id, Time.from_msg(msg.header.stamp))
            target = do_transform_pose_stamped(marker, transform)
            target.header.stamp = msg.header.stamp
            target.pose.position.z += self.get_parameter('marker_to_tool_z').value
            yaw = math.atan2(target.pose.position.y, target.pose.position.x)
            target.pose.orientation.x = target.pose.orientation.y = 0.0
            target.pose.orientation.z, target.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.publisher.publish(target)
        except (TransformException, cv2.error, ValueError) as exc:
            self.get_logger().warning(f'Target unavailable: {exc}', throttle_duration_sec=5.0)

    def publish_debug(self, msg, frame, codes):
        image = rgb(msg, frame)
        if self.osd is None or (self.osd.frame_w, self.osd.frame_h) != (msg.width, msg.height):
            size = (msg.width, msg.height)
            self.osd = Osd(size, size, hold=self.get_parameter('osd_hold').value,
                           font_size=max(12, msg.height // 22))
        self.osd.show(codes)
        self.osd.draw(image)
        out = Image(header=msg.header, height=msg.height, width=msg.width,
                    encoding='rgb8', step=msg.width * 3)
        out.data = image.tobytes()
        self.debug_publisher.publish(out)

    def destroy_node(self):
        if self.recorder is not None:
            self.recorder.stop()
        self.scanner.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Vision()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
