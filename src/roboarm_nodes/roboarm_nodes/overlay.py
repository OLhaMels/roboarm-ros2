"""On-screen overlay: the zbar outline and a readout band burnt into a frame.

Carried over from upd-rasp-stream-tcp/qr_detector.py. There it drew into
the camera's main stream in picamera2's pre_callback; here vision_node draws
into a copy of the frame it scanned and publishes that as
/vision/debug_image. The only change is a fallback font, for an OS image
that ships without DejaVu.
"""

import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
BOX_COLOUR = (0, 255, 0)
BOX_THICKNESS = 3
BOX_HOLD = 0.35   # an outline outlives its scan just long enough not to flicker
MAX_CHARS = 52


class Osd:
    """Overlay state, owned entirely by the camera thread's pre_callback:
    each frame it is first told about any new detection, then asked to draw.
    Because both calls happen back to back on the same thread, the state
    here needs no locking of its own - the only cross-thread handoff in this
    pipeline is QRDetector's mailbox, upstream of this class.

    Colours are deliberately channel-order agnostic - green, white, black -
    because the main stream is XBGR8888 and only the middle channel is
    unambiguous.
    """

    def __init__(self, frame_size, scan_size, hold=5.0, font_size=22,
                 font_path=FONT_PATH):
        self.frame_w, self.frame_h = frame_size
        self.scale_x = self.frame_w / scan_size[0]
        self.scale_y = self.frame_h / scan_size[1]
        self.hold = hold
        try:
            self.font = ImageFont.truetype(font_path, font_size)
        except OSError:
            # No DejaVu on the image: PIL's built-in bitmap font is small and
            # Latin-only, but still shows that a code was read.
            self.font = ImageFont.load_default()
        self._text_cache = (None, None)   # (key, mask)
        self._boxes = None                # (polygons, until)
        self._readout = None              # (mask, until)

    # ---- called once per frame, before draw() ---------------------------

    def show(self, codes):
        """Register the codes seen in the most recent scan."""
        if not codes:
            return
        now = time.monotonic()

        polygons = []
        for code in codes:
            if code.corners:
                polygons.append(tuple(
                    (int(x * self.scale_x), int(y * self.scale_y))
                    for x, y in code.corners))
        if polygons:
            self._boxes = (tuple(polygons), now + BOX_HOLD)

        primary = codes[0]
        text = primary.text.replace("\n", " ")
        if len(text) > MAX_CHARS:
            text = text[:MAX_CHARS - 1] + "…"
        detail = "{}  q={}".format(time.strftime("%H:%M:%S"), primary.quality)
        if len(codes) > 1:
            detail += "  (+{} more)".format(len(codes) - 1)
        self._readout = (self._mask(text, detail), now + self.hold)

    # ---- called once per frame, right after show() ----------------------

    def draw(self, frame):
        """Draw the current overlay onto one main-stream frame, in place."""
        now = time.monotonic()
        height, width = frame.shape[0], min(frame.shape[1], self.frame_w)

        boxes = self._boxes
        if boxes and boxes[1] > now:
            for polygon in boxes[0]:
                self._polygon(frame, polygon, height, width)

        readout = self._readout
        if readout and readout[1] > now:
            self._band(frame, readout[0], height, width)

    # ---- internals ------------------------------------------------------

    def _mask(self, text, detail):
        """Render the two readout lines once, as a boolean pixel mask."""
        key = (text, detail)
        if self._text_cache[0] == key:
            return self._text_cache[1]

        pad = 10
        line_gap = 4
        top = self.font.getbbox(text)
        bottom = self.font.getbbox(detail)
        text_w = max(top[2], bottom[2])
        top_h = top[3]
        image = Image.new("L", (text_w + 2 * pad,
                                top_h + bottom[3] + line_gap + 2 * pad), 0)
        draw = ImageDraw.Draw(image)
        draw.text((pad, pad - top[1]), text, fill=255, font=self.font)
        draw.text((pad, pad + top_h + line_gap - bottom[1]), detail,
                  fill=255, font=self.font)
        mask = np.asarray(image) > 96
        self._text_cache = (key, mask)
        return mask

    def _polygon(self, frame, points, height, width):
        """Trace a closed outline through `points` (already main-frame pixels).

        This is the polygon zbar found, not its bounding box, so a code held
        at an angle gets an outline that actually follows it. QR codes only
        occupy a small part of the frame and this only runs while one is in
        view, so a plain per-pixel Bresenham stamp is cheap enough - no need
        to pull in a drawing library for four short lines.
        """
        n = len(points)
        if n < 2:
            return
        for i in range(n):
            self._line(frame, points[i], points[(i + 1) % n], height, width)

    def _line(self, frame, p0, p1, height, width):
        x0, y0 = p0
        x1, y1 = p1
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        t = BOX_THICKNESS // 2 + 1
        while True:
            xa, ya = max(0, x0 - t), max(0, y0 - t)
            xb, yb = min(width, x0 + t), min(height, y0 + t)
            if xb > xa and yb > ya:
                frame[ya:yb, xa:xb, :3] = BOX_COLOUR
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def _band(self, frame, mask, height, width):
        mask_h, mask_w = mask.shape
        margin = 16
        x0 = margin
        y0 = height - mask_h - margin
        if y0 < 0 or x0 + mask_w > width:
            # Frame too small for the readout as rendered; skip it rather
            # than crop it into something unreadable.
            return
        band = frame[y0:y0 + mask_h, x0:x0 + mask_w]
        band[..., :3] //= 4          # darken, so white text reads over anything
        band[..., :3][mask] = 255
