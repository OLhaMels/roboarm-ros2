"""Part pages: a scanned QR link -> its iFurn page -> one row of parts.csv.

Carried over from upd-rasp-stream-tcp/qr_detector.py. The parsing is
unchanged; PartRecorder now reports through callbacks instead of print(), so
vision_node can log through ROS and publish each recorded part on a topic.
Only the Python standard library is used, so nothing has to be added to the
OS image for this.
"""

import csv
import html
import os
import queue
import re
import threading
import time
import urllib.request
import xml.etree.ElementTree as ET

# ---- part pages: QR link -> iFurn page -> one CSV row --------------------
#
# The QR codes on the part labels are short links (https://2qi.us/...)
# that redirect to an iFurn page about that part, e.g.
# https://de-q.ifurn.pro/v/p/mAGLvubp/37/1 (order code, part number, and a
# trailing number that does not change the page - kept as `piece`). Any
# http(s) link is therefore followed, and recorded only if it ends up on
# such a page. The page is plain server-rendered HTML; no browser needed:
#
#   page    order id/code/name/date, sizes "(564/564*400/399.7*18) [txt 0] - 7",
#           edge banding per side ("top | Кромкование 201-G PVC ... MAAG 100m
#           22 1 Кромка"), the route after cutting, and a link to:
#   drawing an SVG blueprint: <g id="front"> holds the board material,
#           <g id="t_info"/"b_info"/"l_info"/"r_info"> one per banded side
#           with the edge's thickness, width, jointing and curved flags.
#
# Prices and the full material names are hidden unless logged in to the
# order's firm, so the material comes from the drawing, where iFurn itself
# truncates it ("8685 BS білий сніг KRONOSPA...").
#
# Only the Python standard library is used, so nothing has to be added to
# the OS image for this.

PART_URL = re.compile(r"^https?://[\w.-]*ifurn\.pro/v/p/(?P<order>[\w-]+)/"
                      r"(?P<part>\d+)(?:/(?P<piece>\d+))?/?$")
LINK = re.compile(r"^https?://\S+$")

CSV_FIELDS = [
    "scanned_at", "qr_code", "url", "order_id", "order_code", "order_name", "order_date",
    "part_id", "piece", "material",
    "length_mm", "width_mm", "thickness_mm", "cut_length_mm", "cut_width_mm",
    "quantity", "texture",
    "edge_sides", "edge_material", "edge_thickness_mm", "edge_width_mm",
    "edge_jointing", "edge_curved", "edge_processing",
    "operations", "status",
]

SIDES = {"t": "top", "b": "bottom", "l": "left", "r": "right"}

# Drawing labels come in the site's language; these cover UA, RU and EN.
EDGE_KEYS = {
    "edge_thickness_mm": ("товщина", "толщина", "thickness"),
    "edge_width_mm": ("ширина", "width"),
    "edge_jointing": ("прифугівка", "прифуговка", "jointing"),
    "edge_curved": ("криволінійка", "криволинейка", "curved"),
}
YES = ("так", "да", "yes")
NO = ("ні", "нет", "no")

# "( - 462917 / mAGLvubp) Матула-І-6 2026-07-13 11:08:16"
ORDER_RE = re.compile(r"\([^()]*?(\d+)\s*/\s*([\w-]+)\s*\)\s*(.*?)\s*"
                      r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})")
# "(564/564*400/399.7*18) [txt 0] - 7": length final/cut * width final/cut
# * thickness, texture flag, quantity in the order. The cut size is the
# smaller one, before the edge banding is added.
SIZES_RE = re.compile(r"\(\s*([\d.]+)\s*/\s*([\d.]+)\s*\*\s*([\d.]+)\s*/\s*"
                      r"([\d.]+)\s*\*\s*([\d.]+)\s*\)\s*\[txt\s*(\w+)\]\s*-\s*(\d+)")
EDGE_ROW_RE = re.compile(r"<tr>\s*<td>\s*(top|bottom|left|right)\s*</td>\s*"
                         r"<td>(.*?)</td>\s*</tr>", re.S)
STEP_RE = re.compile(r'<h5 class="card-title">(.*?)</h5>', re.S)
DRAWING_RE = re.compile(r'<iframe[^>]*src="([^"]*api/images/[^"]*)"')
SVG = "{http://www.w3.org/2000/svg}"


def _text(fragment):
    """HTML fragment -> plain text on one line."""
    return " ".join(html.unescape(re.sub(r"<[^>]+>", " ", fragment)).split())


class NotAPart(ValueError):
    """The link works but does not lead to an iFurn part page."""


def fetch(url, timeout=10.0):
    """(final URL after redirects, page text)."""
    request = urllib.request.Request(
        url, headers={"User-Agent": "Mozilla/5.0 (qr_detector)"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        charset = response.headers.get_content_charset() or "utf-8"
        return response.geturl(), response.read().decode(charset, "replace")


def parse_part_page(page):
    """Fields from the part page itself, plus the drawing's URL (or None)."""
    start = page.find("blank-wrapper")
    if start < 0:
        raise ValueError("no part on this page")
    body = page[start:]
    text = _text(body[:body.find("<table") if "<table" in body else 4000])
    row = {}

    match = ORDER_RE.search(text)
    if match:
        row.update(order_id=match.group(1), order_code=match.group(2),
                   order_name=match.group(3), order_date=match.group(4))

    match = SIZES_RE.search(text)
    if not match:
        raise ValueError("part sizes not found on the page")
    row.update(length_mm=match.group(1), cut_length_mm=match.group(2),
               width_mm=match.group(3), cut_width_mm=match.group(4),
               thickness_mm=match.group(5), texture=match.group(6),
               quantity=match.group(7))

    # The edge table follows the "Обработка торцов [pur] (с прифуговкой)"
    # heading; parts with no banding have neither.
    edges = [(side, _text(desc)) for side, desc in EDGE_ROW_RE.findall(body)]
    if edges:
        heading = re.findall(r"<h4>(.*?)</h4>", body[:body.find("<table")], re.S)
        row["edge_processing"] = _text(heading[-1]) if heading else ""
        row["edge_sides"] = ";".join(side for side, _ in edges)
        row["edge_material"] = ";".join(dict.fromkeys(desc for _, desc in edges))

    steps = [_text(step) for step in STEP_RE.findall(body)]
    # "A1 Порезка ... до 21 мм - Количество: 6.88" -> the operation alone.
    steps = [re.sub(r"\s+-\s+[^-:]+:\s*[\d.]+$", "", step) for step in steps]
    row["operations"] = ";".join(dict.fromkeys(steps))

    match = DRAWING_RE.search(body)
    return row, html.unescape(match.group(1)) if match else None


def _group_lines(group):
    """Text lines of one SVG <g>, one per <tspan> (or per <text>)."""
    lines = []
    for text in group.iter(SVG + "text"):
        spans = [span.text.strip() for span in text.iter(SVG + "tspan")
                 if span.text and span.text.strip()]
        lines += spans or ["".join(text.itertext()).strip()]
    return [line for line in lines if line]


def parse_drawing(svg):
    """Material and per-side edge parameters from the part's SVG drawing."""
    root = ET.fromstring(svg)
    groups = {}
    for group in root.iter(SVG + "g"):
        # The blueprint is drawn twice (screen and print); keep the first.
        groups.setdefault(group.get("id"), group)

    row = {}
    if "front" in groups:
        lines = _group_lines(groups["front"])
        if lines:
            row["material"] = lines[0]

    per_side = {}
    for key, side in SIDES.items():
        group = groups.get(key + "_info")
        if group is None:
            continue
        values = {}
        for line in _group_lines(group)[1:]:
            label, _, value = line.partition(":")
            label, value = label.strip().lower(), value.strip()
            for field, names in EDGE_KEYS.items():
                if label in names:
                    if value.lower() in YES:
                        value = "yes"
                    elif value.lower() in NO:
                        value = "no"
                    values[field] = value
        per_side[side] = values

    if per_side:
        row["edge_sides"] = ";".join(per_side)
        for field in EDGE_KEYS:
            row[field] = ";".join(dict.fromkeys(
                values[field] for values in per_side.values() if field in values))
    return row


def read_part(link, timeout=10.0):
    """Everything known about the part a QR link points at, as a CSV row.

    `link` is what the QR code holds: a part page URL, or a short link that
    redirects to one. Raises NotAPart if it leads anywhere else.
    """
    scanned_at = time.strftime("%Y-%m-%d %H:%M:%S")
    url, page = fetch(link, timeout)
    match = PART_URL.match(url)
    if not match:
        raise NotAPart("leads to {}, not to a part page".format(url))
    row = {"scanned_at": scanned_at, "qr_code": link, "url": url,
           "order_code": match.group("order"), "part_id": match.group("part"),
           "piece": match.group("piece") or ""}
    page_row, drawing_url = parse_part_page(page)
    row.update(page_row)
    status = "ok"
    if drawing_url:
        try:
            drawing = parse_drawing(fetch(drawing_url, timeout)[1])
            # The page's own edge sides win if both have them; the drawing
            # adds the parameters the page does not show.
            for field, value in drawing.items():
                if value and not row.get(field):
                    row[field] = value
        except (OSError, ET.ParseError) as error:
            status = "ok, no drawing: {}".format(error)
    else:
        status = "ok, no drawing"
    row["status"] = status
    return row


def append_row(path, row):
    """Append one row to the CSV, writing the header first if it is new.

    An existing file's own header decides the columns and their order, so
    the CSV can be prepared in advance with only the columns wanted: fields
    it does not name are left out, and columns it names that are unknown
    here stay empty.
    """
    fields = CSV_FIELDS
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    if not new:
        with open(path, newline="", encoding="utf-8-sig") as handle:
            header = next(csv.reader(handle), None)
        if header:
            fields = header
    # utf-8-sig only on a new file: the BOM lets Excel show Cyrillic right.
    with open(path, "a", newline="", encoding="utf-8-sig" if new else "utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, restval="",
                                extrasaction="ignore")
        if new:
            writer.writeheader()
        writer.writerow(row)


class PartRecorder:
    """Turns newly-read part links into CSV rows, off the image callback.

    Fetching a page takes as long as the network says, so submit() only
    queues the link and returns; a worker thread fetches, parses and
    appends. Each scanned link is recorded once per run - two different
    labels for the same part make two rows. A link whose fetch failed is
    retried the next time it is scanned; one that leads somewhere other
    than a part page is not looked up again.

    `on_row(row)` is called on the worker thread after a row is written;
    `log(level, text)` receives what the original printed, with level one of
    "info" or "warning".
    """

    def __init__(self, csv_path, timeout=10.0, on_row=None, log=None):
        self.csv_path = csv_path
        self.timeout = timeout
        self._on_row = on_row
        self._log = log or (lambda level, text: print(text, flush=True))
        self._queue = queue.Queue()
        self._done = set()
        self._done_lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="part-recorder")

    def start(self):
        directory = os.path.dirname(self.csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self._thread.start()
        return self

    def stop(self, timeout=2.0):
        self._queue.put(None)
        self._thread.join(timeout=timeout)

    def submit(self, text):
        url = text.strip()
        if not LINK.match(url):
            self._log("info", "{!r}: not a link, not recorded".format(url))
            return
        with self._done_lock:
            if url in self._done:
                self._log("info", "{}: part already recorded this run".format(url))
                return
            self._done.add(url)
        self._queue.put(url)

    def _run(self):
        while True:
            url = self._queue.get()
            if url is None:
                return
            try:
                row = read_part(url, self.timeout)
            except NotAPart as error:
                self._log("info", "{}: not recorded, {}".format(url, error))
                continue
            except (OSError, ValueError) as error:
                with self._done_lock:
                    self._done.discard(url)
                self._log("warning", "{}: FAILED, not recorded: {}".format(url, error))
                continue
            try:
                append_row(self.csv_path, row)
            except OSError as error:
                with self._done_lock:
                    self._done.discard(url)
                self._log("warning", "cannot write {}: {}".format(self.csv_path, error))
                continue
            self._log("info", "order {} part #{}: {}x{}x{} mm x{}, {}, edges: {} -> {}".format(
                row.get("order_id") or row.get("order_code"), row.get("part_id"),
                row.get("length_mm"), row.get("width_mm"), row.get("thickness_mm"),
                row.get("quantity"), row.get("material", "?"),
                row.get("edge_sides") or "none", self.csv_path))
            if self._on_row:
                self._on_row(row)
