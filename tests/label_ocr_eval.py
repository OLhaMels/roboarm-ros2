#!/usr/bin/env python3
"""Accuracy of label OCR on real photos: which fields come out right, and how long it takes.

    python3 tests/label_ocr_eval.py [tests/labels] [--upscale 1.5] [--languages ukr+rus+eng]

Expected values below were typed in by hand from the photos in
tests/labels (not committed: the labels carry customers' names). A photo
missing from the folder is skipped. Needs OpenCV and tesseract with the
ukr, rus, eng and osd data. Not a pass/fail test: OCR on phone photos is
not perfect, and this shows how far from perfect.
"""
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/roboarm_nodes"))
from roboarm_nodes.labels import Tesseract, read_label  # noqa: E402

EXPECTED = {
    "photo_2026-10-09_19-34-13.jpg": dict(length_mm="350", width_mm="200", thickness_mm="18"),
    "photo_2026-10-30_20-35-29.jpg": dict(length_mm="60", width_mm="482", thickness_mm="18",
                                          part_id="97", piece="1", order_id="488940"),
    "photo_2026-10-30_20-35-46.jpg": dict(length_mm="1877", width_mm="497", thickness_mm="18",
                                          part_id="13", quantity="3"),
    "photo_2026-10-30_20-35-50.jpg": dict(length_mm="1157", width_mm="340", thickness_mm="18",
                                          part_id="35", quantity="2"),
    "photo_2026-10-30_20-35-56.jpg": dict(length_mm="686", width_mm="422", thickness_mm="18",
                                          part_id="142", piece="1", order_id="505365"),
    "photo_2026-10-30_20-36-00.jpg": dict(length_mm="788.4", width_mm="430.4", thickness_mm="18",
                                          part_id="21", quantity="1"),
    "photo_2026-10-30_20-36-03.jpg": dict(length_mm="577", width_mm="1076"),
    "photo_2026-10-30_20-36-08.jpg": dict(length_mm="350", width_mm="200", thickness_mm="18"),
    "photo_2026-10-30_20-36-12.jpg": dict(length_mm="350", width_mm="200", thickness_mm="18"),
    "photo_2026-10-30_20-36-23.jpg": dict(length_mm="500.4", width_mm="200.4", thickness_mm="18",
                                          part_id="9", quantity="4"),
    "photo_2026-10-30_20-36-27.jpg": dict(length_mm="2664", width_mm="260", thickness_mm="18",
                                          part_id="23", piece="1", order_id="509804"),
}


def main():
    import cv2
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?", default=str(Path(__file__).parent / "labels"))
    parser.add_argument("--upscale", type=float, default=1.5)
    parser.add_argument("--languages", default="ukr+rus+eng")
    parser.add_argument("--verbose", action="store_true", help="show the text of labels read wrong")
    args = parser.parse_args()
    tesseract = Tesseract(args.languages)
    sizes_ok = fields_ok = fields_all = 0
    total_time = 0.0
    tried = 0
    for name, expected in EXPECTED.items():
        path = Path(args.folder) / name
        if not path.exists():
            continue
        tried += 1
        gray = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2GRAY)
        started = time.monotonic()
        row, text, orientation = read_label(gray, tesseract, args.upscale)
        took = time.monotonic() - started
        total_time += took
        right = [k for k, v in expected.items() if row.get(k) == v]
        wrong = {k: row.get(k) for k, v in expected.items() if row.get(k) not in (None, v)}
        sizes = all(row.get(k) == expected.get(k) for k in ("length_mm", "width_mm", "thickness_mm")
                    if k in expected)
        sizes_ok += sizes
        fields_ok += len(right)
        fields_all += len(expected)
        print("{:32} {:5.1f}s rot={:3} sizes={} fields {}/{}{}".format(
            name, took, orientation, "OK  " if sizes else "MISS", len(right), len(expected),
            "  wrong: {}".format(wrong) if wrong else ""))
        if not sizes and args.verbose:
            print("    read: " + " | ".join(line.strip() for line in text.splitlines() if line.strip())[:300])
    if tried:
        print("\nsizes right: {}/{}   fields right: {}/{}   mean time: {:.1f}s".format(
            sizes_ok, tried, fields_ok, fields_all, total_time / tried))


if __name__ == "__main__":
    main()
