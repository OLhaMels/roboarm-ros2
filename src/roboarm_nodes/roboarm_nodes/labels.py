"""Printed part labels without a usable QR code: photo -> text (OCR) -> one CSV row.

Not every sticker has a QR link to an iFurn page. All of them, though, print
the sizes and most of the order details as text, in a few layouts seen on
the boards so far:

    Коваленко Петро 30.09 / ДСП Сатин 18мм В/Ф / 1877 x 497 x 18 / №:13 K:3
    П:Іван Бондар / М:ДСП ... / Р:788.4 x 430.4 x 18 / №:21 (1:21) K:1
    505365 Реставрац ЛВ-10073387. / #142-1 г/р 686 x 422x18
    Реставрація дверей / 2800-2070-18-swiss-krono-... / 60844 / 350_200_18

plus edge banding drawn round a rectangle as "(B) 1" / "(A) 0.8" per side.
Labels lie at any angle, often turned 90 or 180 degrees. prepare() keeps
only the black print, crops to the label and squares its text rows up;
read_label() then reads it in the orientation Tesseract's OSD suggests
first and the others only if needed, keeping the one whose text parses
into the most fields. No sizes, no row: a missed label is better than a
wrong size going to the arm.

OCR is Tesseract, called as a program (tesseract-ocr + the ukr, rus, eng and
osd data): available on Ubuntu, Debian and in meta-oe for Yocto, with no
Python binding to add. Imports nothing from ROS.
"""

import re
import shutil
import subprocess
import time

import numpy as np

NUMBER = r"\d{1,4}(?:[.,]\d{1,2})?"
X = r"\s*[xхXХ×*]\s*"
# "1877 x 497 x 18", "Р:788.4 x 430.4 x 18", "г/р 686 x 422x18"
# Not inside a longer number: no digit, and no "3." / "3," just before. A
# bare "." is fine - "Р.788,4" is how OCR often reads "Р:788.4".
DIMS_RE = re.compile(r"(?<!\d)(?<!\d[.,])({n}){x}({n}){x}(\d{{1,2}}(?:[.,]\d)?)(?![\d])".format(n=NUMBER, x=X))
# "350_200_18"; OCR often drops the underscores, so a line of exactly three
# numbers counts too (a date line has four).
UNDERSCORE_DIMS_RE = re.compile(r"(?<![\d_])(\d{2,4})_(\d{2,4})_(\d{1,2})(?![\d_])"
                                r"|^\s*(\d{2,4})\s+(\d{2,4})\s+(\d{1,2})\s*$", re.M)
QUANTITY_RE = re.compile(r"(?<![A-Za-zА-Яа-яІіЇїЄє])[KК]\s*[:.;]\s*(\d{1,3})\b")
# "№:13", "Nº 21", "No:9" - OCR reads № many ways. "№:" with a colon is the
# part number; "Н:№118" on the same label is the cabinet, so it ranks lower.
NUMBER_SIGN_RE = re.compile(r"(?:№|N[oO°º2]|Ne|М[oо](?=\s+\d))\s*([:.]?)\s*(\d{1,5})\b(?:\s*\(\s*\d+\s*[:;]\s*(\d+)\s*\))?")
# "#142-1": part 142, piece 1
HASH_RE = re.compile(r"#\s*(\d{1,5})\s*[-–]\s*(\d{1,3})\b")
ORDER_CODE_RE = re.compile(r"\b([ЛL][ВB])\s*[-–]?\s*(\d{6,9})\b")
# "488940 Шевчук ЛВ-10070933", "505365 Реставрац ЛВ-..."
ORDER_LINE_RE = re.compile(r"\b(\d{5,6})\s+([А-ЯІЇЄҐA-Z][\w'’ -]{2,40}?)\s+[ЛL][ВB]")
PERSON_RE = re.compile(r"^\s*П\s*[:.]\s*(.+)$", re.M)
MATERIAL_LINE_RE = re.compile(r"^\s*М\s*[:.]\s*(.+)$", re.M)
MATERIAL_WORDS_RE = re.compile(
    r"(ДСП|ДВП|МДФ|ЛДСП|egger|krono|kronospan|swiss|cleaf|сатин)", re.I)
DATE_RE = re.compile(r"\b(20\d{2})[.\- ](\d{2})[.\- ](\d{2})(?:\s+(\d{2})[: ](\d{2}))?\b")
SHORT_DATE_RE = re.compile(r"\b(\d{2})\.(\d{2})(?:\.(\d{2,4}))?\b")
EDGE_RE = re.compile(r"\(\s*([ABАВ])\s*\)\s*(\d(?:[.,]\d)?)")
NAME_DATE_RE = re.compile(r"^\s*([А-ЯІЇЄҐ][а-яіїєґ'’]+(?:\s+[А-ЯІЇЄҐ][а-яіїєґ'’]+)?)\s+\d{2}\.\d{2}", re.M)


def _mm(value):
    return value.replace(",", ".")


def _plausible(length, width, thickness):
    try:
        length, width, thickness = float(length), float(width), float(thickness)
    except ValueError:
        return False
    # Boards come from 2800 x 2070 sheets, 3 to 50 mm thick.
    return 10 <= length <= 3000 and 10 <= width <= 3000 and 3 <= thickness <= 50


def parse_label(text):
    """Fields of one label's OCR text, keyed like parts.csv. Empty dict if no sizes."""
    row = {}
    for match in DIMS_RE.finditer(text):
        if _plausible(*(_mm(g) for g in match.groups())):
            row.update(length_mm=_mm(match.group(1)), width_mm=_mm(match.group(2)),
                       thickness_mm=_mm(match.group(3)))
            break
    else:
        for match in UNDERSCORE_DIMS_RE.finditer(text):
            sizes = [g for g in match.groups() if g is not None]
            if _plausible(*sizes):
                row.update(length_mm=sizes[0], width_mm=sizes[1], thickness_mm=sizes[2])
                break
    if not row:
        return {}

    match = QUANTITY_RE.search(text)
    if match:
        row["quantity"] = match.group(1)
    match = HASH_RE.search(text)
    if match:
        row.update(part_id=match.group(1), piece=match.group(2))
    else:
        # Best: "№:" with a colon, then one followed by "(1:21)", then the
        # last on the label (the cabinet's "Н:№118" is printed above it).
        matches = list(NUMBER_SIGN_RE.finditer(text))
        if matches:
            chosen = max(enumerate(matches), key=lambda im: (bool(im[1].group(1)), bool(im[1].group(3)), im[0]))[1]
            row["part_id"] = chosen.group(2)
            if chosen.group(3):
                row["piece"] = chosen.group(3)

    match = ORDER_CODE_RE.search(text)
    if match:
        row["order_code"] = "ЛВ-" + match.group(2)
    match = ORDER_LINE_RE.search(text)
    if match:
        row.update(order_id=match.group(1), order_name=match.group(2).strip())
    else:
        match = PERSON_RE.search(text)
        if match:
            person = match.group(1).strip()
            number = re.match(r"(\d{4,6})[_\s]+(.*)", person)
            if number:
                row.update(order_id=number.group(1), order_name=number.group(2).replace("_", " ").strip())
            else:
                row["order_name"] = person
        else:
            match = NAME_DATE_RE.search(text)
            if match:
                row["order_name"] = match.group(1)

    match = MATERIAL_LINE_RE.search(text)
    if match:
        row["material"] = match.group(1).strip()
    else:
        for line in text.splitlines():
            if MATERIAL_WORDS_RE.search(line):
                row["material"] = line.strip()
                break

    match = DATE_RE.search(text)
    if match:
        year, month, day, hour, minute = match.groups()
        row["order_date"] = "{}-{}-{}".format(year, month, day) + (" {}:{}".format(hour, minute) if hour else "")
    else:
        match = SHORT_DATE_RE.search(text)
        if match and 1 <= int(match.group(1)) <= 31 and 1 <= int(match.group(2)) <= 12:
            row["order_date"] = match.group(0)

    edges = [_mm(value) for _, value in EDGE_RE.findall(text)]
    if edges:
        row["edge_count"] = str(min(len(edges), 4))
        row["edge_thickness_mm"] = ";".join(dict.fromkeys(edges))
    if re.search(r"\bPUR\b", text):
        row["edge_processing"] = "PUR"
    if re.search(r"\bDrill\b", text, re.I):
        row["operations"] = "Drill"
    return row


def score(row):
    """How much of a label a parse recovered: sizes count most."""
    if not row:
        return 0
    return 5 + sum(1 for key in ("quantity", "part_id", "order_name", "order_id", "order_code",
                                 "material", "order_date", "edge_count") if row.get(key))


# ---- image side -----------------------------------------------------------

def ink(gray):
    """The black print of a frame as a 0/1 mask, without the board under it,
    and the frame with its lighting flattened.

    Lighting is flattened by dividing by a blurred background, then only
    clearly dark pixels are kept: chipboard texture and shadows are much
    lighter than thermal print. Connected blobs that cannot be letters go:
    specks, long lines (label border, drawing) and solid blocks (QR and
    barcode modules), which otherwise turn into pages of OCR noise.
    """
    import cv2
    background = cv2.GaussianBlur(cv2.dilate(gray, np.ones((15, 15), np.uint8)), (0, 0), 15)
    flat = cv2.divide(gray, background, scale=255)
    mask = (flat < 150).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    height, width = gray.shape
    x, y, w, h, area = (stats[:, i] for i in range(5))
    keep = (area >= 12) & (h <= height * 0.06) & (w <= width * 0.25)
    keep &= ~((area > 0.85 * w * h) & (w > 6) & (h > 6))
    keep[0] = False
    return keep[labels].astype(np.uint8), flat


def skew_angle(mask):
    """Angle in [-45, 45) degrees that lines the print up with the image axes.

    Rows of text, turned straight, make the row (or column) sums of the
    mask most uneven; which of the four right-angle orientations is upright
    is left to OCR.
    """
    import cv2
    scale = 600.0 / max(mask.shape)
    small = cv2.resize(mask * 255, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    centre = (small.shape[1] / 2, small.shape[0] / 2)
    best = (-1.0, 0.0)
    for angle in np.arange(-45.0, 45.0, 1.0):
        turned = cv2.warpAffine(small, cv2.getRotationMatrix2D(centre, angle, 1.0), small.shape[::-1])
        spread = max(np.var(turned.sum(axis=1)), np.var(turned.sum(axis=0)))
        if spread > best[0]:
            best = (spread, float(angle))
    return best[1]


def straighten(image, angle, fill=0, smooth=False):
    """Rotate by `angle` degrees about the centre, growing the canvas to keep the corners."""
    import cv2
    height, width = image.shape
    matrix = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1.0)
    cos, sin = abs(matrix[0, 0]), abs(matrix[0, 1])
    new_w, new_h = int(height * sin + width * cos), int(height * cos + width * sin)
    matrix[0, 2] += new_w / 2 - width / 2
    matrix[1, 2] += new_h / 2 - height / 2
    flags = cv2.INTER_LINEAR if smooth else cv2.INTER_NEAREST
    return cv2.warpAffine(image, matrix, (new_w, new_h), flags=flags, borderValue=fill)


def _crop(mask, box, pad=20):
    x, y, w, h = box
    return mask[max(0, y - pad):y + h + pad, max(0, x - pad):x + w + pad]


def densest_print(mask):
    """Bounding box (x, y, w, h) of the largest cluster of print: the label.

    Letters a few character widths apart merge into one blob; specks of
    board texture elsewhere in the frame stay separate and are dropped, so
    they neither shrink the text in the crop nor skew the angle estimate.
    A label's text block and its drawing can still come apart, so any other
    sizeable blob right next to the largest one joins it.
    """
    import cv2
    reach = max(9, int(max(mask.shape) * 0.045)) | 1
    blobs = cv2.dilate(mask, np.ones((reach, reach), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(blobs, 8)
    if count < 2:
        return None
    ink_per_blob = np.bincount(labels[mask > 0], minlength=count)
    ink_per_blob[0] = 0
    main = int(np.argmax(ink_per_blob))
    x0, y0, w, h = stats[main, :4]
    x1, y1 = x0 + w, y0 + h
    near = 2 * reach
    for other in np.argsort(-ink_per_blob):
        if other == main or ink_per_blob[other] < 0.2 * ink_per_blob[main]:
            continue
        ox, oy, ow, oh = stats[other, :4]
        if ox - near <= x1 and ox + ow + near >= x0 and oy - near <= y1 and oy + oh + near >= y0:
            x0, y0, x1, y1 = min(x0, ox), min(y0, oy), max(x1, ox + ow), max(y1, oy + oh)
    return int(x0), int(y0), int(x1 - x0), int(y1 - y0)


def letter_height(mask):
    import cv2
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    heights = stats[1:, cv2.CC_STAT_HEIGHT]
    heights = heights[heights >= 5]
    return float(np.median(heights)) if heights.size else 0.0


def prepare(gray, upscale=1.5, letter_px=30):
    """Grayscale frame -> dark print on white, squared up and cropped to the label.

    What Tesseract gets is the lighting-flattened frame itself, not the
    0/1 mask: anti-aliased strokes survive its own binarisation better (a
    thin "1" or "x" drops out of a hard mask). The mask decides where the
    print is - everything more than a couple of pixels from it is blanked.

    Scaled so that a typical character is about `letter_px` pixels tall,
    Tesseract's comfort zone; `upscale` caps how far a frame is enlarged.
    """
    import cv2
    mask, flat = ink(gray)
    box = densest_print(mask) if mask.any() else None
    if box is None:
        return None
    near = cv2.dilate(mask, np.ones((5, 5), np.uint8)).astype(bool)
    shown = np.where(near, flat, 255).astype(np.uint8)
    mask, shown = _crop(mask, box), _crop(shown, box)
    angle = skew_angle(mask)
    mask, shown = straighten(mask, angle), straighten(shown, angle, fill=255, smooth=True)
    box = densest_print(mask)
    mask, shown = _crop(mask, box), _crop(shown, box)
    height = letter_height(mask)
    scale = min(max(upscale, 1.0) * 2, max(1.0, letter_px / height)) if height else upscale
    if scale != 1.0:
        shown = cv2.resize(shown, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    return np.ascontiguousarray(shown)


class Tesseract:
    """The tesseract program, fed PNGs on stdin."""

    def __init__(self, languages="ukr+rus+eng", psm=(6, 11), timeout=60.0, binary="tesseract"):
        self.binary = shutil.which(binary)
        if self.binary is None:
            raise RuntimeError("tesseract not found: install tesseract-ocr with the ukr, rus and eng data")
        self.languages = languages
        # Page segmentation modes tried in turn: 6 reads a label as one
        # block of text, 11 hunts for sparse words; each rescues labels the
        # other garbles.
        self.psm = (psm,) if isinstance(psm, int) else tuple(psm)
        self.timeout = timeout

    def _run(self, image, *args):
        import cv2
        ok, png = cv2.imencode(".png", image)
        if not ok:
            raise ValueError("cannot encode frame")
        return subprocess.run([self.binary, "stdin", "stdout", *args], input=png.tobytes(),
                              capture_output=True, timeout=self.timeout, check=False)

    def read(self, image, psm=None):
        done = self._run(image, "-l", self.languages, "--psm", str(psm or self.psm[0]))
        if done.returncode != 0:
            raise RuntimeError("tesseract failed: " + done.stderr.decode(errors="replace").strip())
        return done.stdout.decode("utf-8", "replace")

    def upright_turns(self, image):
        """Quarter turns counter-clockwise that OSD thinks make the text upright, or None.

        Needs the "osd" traineddata; a guess, not a verdict: OSD is wrong
        often enough on sparse labels that it only orders the attempts.
        """
        done = self._run(image, "--psm", "0")
        match = re.search(r"Rotate: (\d+)", done.stdout.decode(errors="replace"))
        # "Rotate: N" is clockwise degrees; np.rot90 turns counter-clockwise.
        return (4 - int(match.group(1)) // 90) % 4 if match else None


def read_label(gray, tesseract, upscale=1.5, good_enough=8, budget=45.0):
    """OCR one grayscale frame holding a label at any angle.

    Returns (row, text, orientation) for the best of the four orientations,
    row being {} when none of them yielded the part's sizes. OSD's guess is
    read first; the other turns only if that guess does not parse well, and
    only within `budget` seconds - a frame that will not read (glare, blur)
    otherwise holds the reader for minutes.
    """
    deadline = time.monotonic() + budget
    image = prepare(gray, upscale)
    if image is None:
        return {}, "", 0
    guess = tesseract.upright_turns(image)
    order = [0, 1, 2, 3] if guess is None else [guess] + [q for q in (0, 1, 2, 3) if q != guess]
    best = None
    for quarter in order:
        turned = np.ascontiguousarray(np.rot90(image, quarter))
        for psm in tesseract.psm:
            if best is not None and time.monotonic() > deadline:
                return best
            text = tesseract.read(turned, psm)
            row = parse_label(text)
            # The first read stays as the answer if nothing parses, for the log.
            if best is None or score(row) > score(best[0]):
                best = (row, text, 90 * quarter)
            if row:
                break  # sizes found in this turn: the other mode adds nothing
        if score(best[0]) >= good_enough or (best[0] and best[2] == 90 * (guess or 0) and guess is not None):
            break  # a confident read, or sizes where OSD said upright: skip the other turns
    return best


def label_row(row, text):
    """A parse_label() result as a complete parts.csv row."""
    full = {"scanned_at": time.strftime("%Y-%m-%d %H:%M:%S"), "source": "label",
            "status": "ok, from label text (OCR) - check before use",
            "label_text": " | ".join(line.strip() for line in text.splitlines() if line.strip())}
    full.update(row)
    return full


class LabelReader:
    """Reads labels off the image callback: OCR takes seconds, frames do not wait.

    offer() hands over a frame and returns at once. A single-slot mailbox
    holds it - while one frame is being read, a newer one replaces any
    still waiting, so the worker always reads the freshest view. Each label
    is reported once per run, keyed by its sizes, part number and order.

    `on_label(row, text, orientation)` gets the complete CSV row, the raw
    OCR text and the turn (degrees) the label was read at; `log(level,
    text)` as in PartRecorder.
    """

    def __init__(self, tesseract, on_label, log=None, upscale=1.5, budget=45.0):
        import queue
        import threading
        self.tesseract = tesseract
        self.upscale = upscale
        self.budget = budget
        self._on_label = on_label
        self._log = log or (lambda level, text: print(text, flush=True))
        self._mailbox = queue.Queue(maxsize=1)
        self._seen = set()
        self._busy = False
        self._thread = threading.Thread(target=self._run, daemon=True, name="label-reader")

    @property
    def busy(self):
        return self._busy or not self._mailbox.empty()

    def start(self):
        self._thread.start()
        return self

    def stop(self, timeout=2.0):
        self.offer(None)
        self._thread.join(timeout=timeout)

    def offer(self, gray):
        import queue
        try:
            self._mailbox.get_nowait()
        except queue.Empty:
            pass
        try:
            self._mailbox.put_nowait(gray)
        except queue.Full:
            pass

    def _run(self):
        while True:
            gray = self._mailbox.get()
            if gray is None:
                return
            self._busy = True
            try:
                started = time.monotonic()
                row, text, orientation = read_label(gray, self.tesseract, self.upscale, budget=self.budget)
                took = time.monotonic() - started
            except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
                self._log("warning", "OCR failed: {}".format(error))
                continue
            finally:
                self._busy = False
            if not row:
                self._log("debug", "no label sizes found ({:.1f} s)".format(took))
                continue
            key = tuple(row.get(k, "") for k in ("length_mm", "width_mm", "thickness_mm",
                                                 "part_id", "piece", "order_id", "order_code"))
            if key in self._seen:
                continue
            self._seen.add(key)
            self._log("info", "label at {} deg, {:.1f} s: {}x{}x{} mm, #{}, x{}, {}".format(
                orientation, took, row.get("length_mm"), row.get("width_mm"), row.get("thickness_mm"),
                row.get("part_id", "?"), row.get("quantity", "?"), row.get("order_name", "")))
            self._on_label(label_row(row, text), text, orientation)
