#!/usr/bin/env python3
"""Perception path without a camera: images -> camera_node -> vision_node -> pose, + TCP stream.

Run after a Jazzy colcon build, in its own domain:

    ROS_DOMAIN_ID=84 python3 tests/perception_smoke.py

Renders a QR label of known size at a known pose, plays it through
camera_node's `images` backend with matching intrinsics, publishes a static
base_link -> camera_optical_frame transform, and checks /vision/qr,
/vision/target_pose and, if PyAV is installed, a frame decoded from
stream_node's TCP port.
"""
from pathlib import Path
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/roboarm_nodes/test'))
from test_qr_vision import K, code_image, render  # noqa: E402

PAYLOAD = 'https://2qi.us/perception-smoke'
SIDE = 0.05
TVEC = np.array([0.03, -0.02, 0.35])
CAMERA_IN_BASE = (0.2, 0.0, 0.5)   # camera translation in base_link, no rotation
PORT = 18554


def start(log, *args):
    return subprocess.Popen(['ros2', 'run', 'roboarm_nodes', *args], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)


def main():
    from PIL import Image
    root = Path(__file__).resolve().parents[1]
    log_path = root / 'perception_smoke.log'
    processes = []
    rclpy.init()
    node = rclpy.create_node('perception_smoke_test')
    got = {'qr': [], 'pose': []}
    node.create_subscription(String, '/vision/qr', lambda m: got['qr'].append(json.loads(m.data)), 10)
    node.create_subscription(PoseStamped, '/vision/target_pose', lambda m: got['pose'].append(m), 10)
    try:
        with tempfile.TemporaryDirectory() as temporary, log_path.open('w') as log:
            frame = render(code_image(PAYLOAD), SIDE, np.array([0.2, -0.1, 0.4]), TVEC)
            Image.fromarray(frame).save(Path(temporary) / 'label.png')
            k = ','.join(f'{v:.1f}' for v in K.flatten())
            processes.append(subprocess.Popen(
                ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '--x', str(CAMERA_IN_BASE[0]), '--y', str(CAMERA_IN_BASE[1]), '--z', str(CAMERA_IN_BASE[2]),
                 '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            processes.append(start(log, 'camera_node', '--ros-args', '-p', 'backend:=images',
                                   '-p', f'images_path:={temporary}', '-p', 'fps:=10.0',
                                   '-p', f'camera_matrix:=[{k}]'))
            processes.append(start(log, 'vision_node', '--ros-args', '-p', 'record_parts:=false',
                                   '-p', f'qr_size:={SIDE}'))
            try:
                import av
            except ImportError:
                av = None
                print('PyAV missing: stream_node not tested')
            if av is not None:
                processes.append(start(log, 'stream_node', '--ros-args', '-p', f'port:={PORT}'))
            deadline = time.monotonic() + 30
            while not (got['qr'] and len(got['pose']) >= 3):
                assert all(p.poll() is None for p in processes), f'A node exited; inspect {log_path}'
                assert time.monotonic() < deadline, f'No QR/pose within 30 s; inspect {log_path}'
                rclpy.spin_once(node, timeout_sec=0.1)
            qr = got['qr'][0]
            assert qr['text'] == PAYLOAD and qr['target'], qr
            pose = got['pose'][-1]
            assert pose.header.frame_id == 'base_link'
            found = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            expected = TVEC + np.array(CAMERA_IN_BASE)
            error = np.linalg.norm(found - expected)
            assert error < 0.01, f'pose {found} != {expected}'
            yaw = 2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
            assert abs(yaw - math.atan2(found[1], found[0])) < 1e-6
            print(f'QR read: {qr["text"]}; target {found.round(4)} (error {error * 1000:.1f} mm)')
            if av is not None:
                deadline = time.monotonic() + 15
                while True:
                    try:
                        with av.open(f'tcp://127.0.0.1:{PORT}', options={'timeout': '5000000'}) as stream:
                            decoded = next(stream.decode(video=0)).to_ndarray(format='rgb24')
                        break
                    except (av.error.FFmpegError, OSError):
                        assert time.monotonic() < deadline, f'No stream on port {PORT}; inspect {log_path}'
                        rclpy.spin_once(node, timeout_sec=0.5)
                assert decoded.shape == (480, 640, 3), decoded.shape
                out = root / 'perception_smoke_stream.png'
                Image.fromarray(decoded).save(out)
                print(f'Stream frame decoded from tcp://127.0.0.1:{PORT} -> {out}')
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
        node.destroy_node()
        rclpy.try_shutdown()
    print('Perception smoke test passed')


if __name__ == '__main__':
    main()
