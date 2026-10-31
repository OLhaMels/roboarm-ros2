#!/usr/bin/env python3
"""Label OCR through ROS: a label photo -> camera_node -> vision_node -> /vision/part + parts.csv.

Run after a Jazzy colcon build, with tesseract installed:

    ROS_DOMAIN_ID=85 python3 tests/label_smoke.py [photo.jpg]

Defaults to the 1877 x 497 x 18 label in tests/labels (not committed); skips if
there is no photo to use.
"""
from pathlib import Path
import csv
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / "tests/labels/photo_2026-10-30_20-35-46.jpg"
EXPECTED = {"length_mm": "1877", "width_mm": "497", "thickness_mm": "18", "part_id": "13", "quantity": "3"}


def main():
    photo = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT
    if not photo.exists():
        print(f"SKIP: no label photo at {photo}")
        return
    log_path = ROOT / "label_smoke.log"
    processes = []
    rclpy.init()
    node = rclpy.create_node("label_smoke_test")
    parts = []
    node.create_subscription(String, "/vision/part", lambda m: parts.append(json.loads(m.data)), 10)
    try:
        with tempfile.TemporaryDirectory() as temporary, log_path.open("w") as log:
            shutil.copy(photo, Path(temporary) / photo.name)
            csv_path = Path(temporary) / "parts.csv"
            for args in (["camera_node", "--ros-args", "-p", "backend:=images", "-p", f"images_path:={temporary}",
                          "-p", "fps:=2.0"],
                         ["vision_node", "--ros-args", "-p", f"parts_csv:={csv_path}", "-p", "ocr_quiet:=0.5"]):
                processes.append(subprocess.Popen(["ros2", "run", "roboarm_nodes", *args], stdout=log,
                                                  stderr=subprocess.STDOUT, start_new_session=True))
            started = time.monotonic()
            while not parts:
                assert all(p.poll() is None for p in processes), f"A node exited; inspect {log_path}"
                assert time.monotonic() - started < 180, f"No /vision/part within 180 s; inspect {log_path}"
                rclpy.spin_once(node, timeout_sec=0.5)
            took = time.monotonic() - started
            part = parts[0]
            assert part["source"] == "label", part
            got = {k: part.get(k) for k in EXPECTED}
            print(f"/vision/part after {took:.0f} s: {got}")
            if photo == DEFAULT:
                assert got == EXPECTED, got
            with csv_path.open(encoding="utf-8-sig") as handle:
                rows = list(csv.DictReader(handle))
            assert len(rows) == 1 and rows[0]["source"] == "label" and rows[0]["length_mm"] == part["length_mm"]
            print(f"parts.csv row: {rows[0]['length_mm']}x{rows[0]['width_mm']}x{rows[0]['thickness_mm']}, "
                  f"#{rows[0]['part_id']}, x{rows[0]['quantity']}, {rows[0]['order_name']}")
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
    print("Label smoke test passed")


if __name__ == "__main__":
    main()
