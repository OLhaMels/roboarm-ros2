"""Offline checks for the QR path: zbar binding, frame conversion, corner order, pose.

Needs libzbar, Pillow, NumPy, OpenCV and the qrcode package (python3-qrcode),
but no running ROS graph: vision helpers are pure functions of a message.
"""
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
cv2 = pytest.importorskip('cv2')
qrcode = pytest.importorskip('qrcode')
from roboarm_nodes.qr import Scanner  # noqa: E402
from roboarm_nodes.parts import PART_URL, parse_drawing  # noqa: E402
from roboarm_nodes import vision  # noqa: E402

PAYLOAD = 'https://2qi.us/test-part-37'
K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])


def code_image(payload=PAYLOAD):
    """QR symbol with no quiet zone, so its corners are the image corners."""
    image = qrcode.make(payload, border=0, box_size=10).convert('L')
    return np.asarray(image, dtype=np.uint8)


def render(symbol, side, rvec, tvec, size=(640, 480)):
    """Project a `side`-metre code with pose (rvec, tvec) into a white frame."""
    half = side / 2
    model = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    image_pts, _ = cv2.projectPoints(model, rvec, tvec, K, np.zeros(5))
    n = symbol.shape[0]
    source = np.float32([[0, 0], [n, 0], [n, n], [0, n]])
    # A white margin around the symbol, as printed labels have.
    canvas = np.full((n + 40, n + 40), 255, np.uint8)
    canvas[20:20 + n, 20:20 + n] = symbol
    warp = cv2.getPerspectiveTransform(source + 20, image_pts.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(canvas, warp, size, borderValue=255)


def message(frame, encoding, pad=0):
    channels = 1 if frame.ndim == 2 else frame.shape[2]
    height, width = frame.shape[:2]
    rows = frame.reshape(height, width * channels)
    padded = np.zeros((height, width * channels + pad), np.uint8)
    padded[:, :width * channels] = rows
    return SimpleNamespace(height=height, width=width, step=width * channels + pad,
                           encoding=encoding, data=padded.tobytes())


def test_zbar_reads_a_flat_code():
    frame = render(code_image(), 0.05, np.zeros(3), np.array([0.0, 0.0, 0.3]))
    with Scanner() as scanner:
        codes = scanner.scan_gray(np.ascontiguousarray(frame), 640, 480)
    assert [c.text for c in codes] == [PAYLOAD]
    assert len(codes[0].corners) == 4


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8', 'rgba8', 'mono8'])
def test_gray_and_rgb_handle_encodings_and_row_padding(encoding):
    plane = np.arange(48, dtype=np.uint8).reshape(4, 12)
    channels = vision.CHANNELS[encoding]
    frame = np.repeat(plane[..., None], channels, axis=2) if channels > 1 else plane
    msg = message(frame, encoding, pad=4)
    pixels = vision.pixels(msg)
    # Equal channels must give back the same gray level (integer weights sum to 256).
    assert np.array_equal(vision.gray(msg, pixels), plane)
    assert vision.rgb(msg, pixels).shape == (4, 12, 3)


def test_bgr_is_swapped_to_rgb():
    frame = np.zeros((2, 2, 3), np.uint8)
    frame[..., 0] = 200  # blue in bgr8
    msg = message(frame, 'bgr8')
    out = vision.rgb(msg, vision.pixels(msg))
    assert out[0, 0].tolist() == [0, 0, 200]


@pytest.mark.parametrize('start', range(4))
@pytest.mark.parametrize('mirror', [False, True])
def test_clockwise_order_is_independent_of_zbar_order(start, mirror):
    square = [(10, 10), (50, 12), (48, 52), (8, 50)]  # clockwise on screen
    given = square[::-1] if mirror else square
    given = given[start:] + given[:start]
    assert vision.clockwise(given).tolist() == [list(map(float, p)) for p in square]


@pytest.mark.parametrize('rvec', [[0.0, 0.0, 0.0], [0.3, -0.2, 0.0], [0.0, 0.0, 1.2], [-0.25, 0.1, 2.6]])
def test_pose_from_zbar_corners_recovers_centre(rvec):
    side = 0.05
    tvec = np.array([0.03, -0.02, 0.35])
    frame = render(code_image(), side, np.array(rvec), tvec)
    with Scanner() as scanner:
        (code,) = scanner.scan_gray(np.ascontiguousarray(frame), 640, 480)
    half = side / 2
    model = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    corners = vision.clockwise(code.corners)
    ok, _, found = cv2.solvePnP(model, corners, K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
    assert ok
    # zbar corners are whole pixels: a few millimetres at 35 cm is expected.
    assert np.linalg.norm(found.flatten() - tvec) < 0.006


def test_target_pattern_default_accepts_part_links_only():
    import re
    pattern = re.compile(r'^https?://(2qi\.us|[\w.-]*ifurn\.pro)/')
    assert pattern.search('https://2qi.us/abc')
    assert pattern.search('https://de-q.ifurn.pro/v/p/mAGLvubp/37/1')
    assert not pattern.search('http://en.m.wikipedia.org')
    assert PART_URL.match('https://de-q.ifurn.pro/v/p/mAGLvubp/37/1').group('part') == '37'


def test_drawing_parser_reads_material_and_edges():
    svg = ('<svg xmlns="http://www.w3.org/2000/svg">'
           '<g id="front"><text><tspan>8685 BS білий сніг</tspan></text></g>'
           '<g id="t_info"><text><tspan>top</tspan><tspan>Товщина: 0.8</tspan>'
           '<tspan>Прифугівка: так</tspan></text></g></svg>')
    row = parse_drawing(svg)
    assert row['material'] == '8685 BS білий сніг'
    assert row['edge_sides'] == 'top'
    assert row['edge_thickness_mm'] == '0.8'
    assert row['edge_jointing'] == 'yes'
