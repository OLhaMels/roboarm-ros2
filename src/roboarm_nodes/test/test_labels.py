"""Label text -> CSV fields, on text Tesseract actually produced from photos of labels.

No OCR runs here: these lines are what tesseract 5.3 printed for the photos
in tests/labels, typos included, so the parser is checked against real
reading errors. Accuracy of the whole photo -> fields path is measured by
tests/label_ocr_eval.py instead.
"""
from pathlib import Path
import sys
import threading

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from roboarm_nodes.labels import LabelReader, label_row, parse_label, score  # noqa: E402
from roboarm_nodes.parts import CSV_FIELDS  # noqa: E402

# One label each of the layouts seen so far, as OCR read them.
VIYAR = "Коваленко Петро 30.09\nДСП Сатин 18мм В/Ф\n1877 х 497 х 18\n№:13 K:3\n(B)1\n1877\n"
VIYAR_ROTATED = "Савчук 22.09.26\nДСП Сатин 18мм В/Ф\n1157 х 340 x 18\n№:35\nK:2\n(B) 0.6\n1157\n"
KITCHEN = ("П іяви Бондар\nМ ДСП Caran 481200002070 7044 SU KronoSpan\nИ.Кухия\nН' №118 мийкл\n"
           "Р.788,4 x 430.4 x 18\nМо 21\n1:21\n) K:1\n")
IFURN = "505365 Реставраці ЛВ-10073387. 2\n#142 -1 гір 686 x 422x18\npd_f_50_664400\n"
RESTORATION = ("Реставрація дверей\n2800-2070-18-swiss-krono-u570-vl-білий-фарфор\n60844\n"
               "350 200 18\n2026 10 07 1056\n")


def test_viyar_layout():
    row = parse_label(VIYAR)
    assert (row["length_mm"], row["width_mm"], row["thickness_mm"]) == ("1877", "497", "18")
    assert row["part_id"] == "13" and row["quantity"] == "3"
    assert row["order_name"] == "Коваленко Петро"
    assert row["material"] == "ДСП Сатин 18мм В/Ф"
    assert row["order_date"] == "30.09"
    assert row["edge_count"] == "1" and row["edge_thickness_mm"] == "1"


def test_second_viyar_label():
    row = parse_label(VIYAR_ROTATED)
    assert (row["length_mm"], row["width_mm"], row["thickness_mm"]) == ("1157", "340", "18")
    assert row["part_id"] == "35" and row["quantity"] == "2"
    assert row["edge_thickness_mm"] == "0.6"


def test_kitchen_layout_with_decimal_sizes_and_cabinet_number():
    row = parse_label(KITCHEN)
    # "Р.788,4": a dot before the number and a decimal comma, as OCR reads "Р:788.4".
    assert (row["length_mm"], row["width_mm"], row["thickness_mm"]) == ("788.4", "430.4", "18")
    # "Н' №118" is the cabinet; the part is "Мо 21" (OCR's "№:21").
    assert row["part_id"] == "21"
    assert row["quantity"] == "1"


def test_ifurn_printed_label():
    row = parse_label(IFURN)
    assert (row["length_mm"], row["width_mm"], row["thickness_mm"]) == ("686", "422", "18")
    assert (row["part_id"], row["piece"]) == ("142", "1")
    assert (row["order_id"], row["order_code"]) == ("505365", "ЛВ-10073387")


def test_underscore_sizes_with_underscores_lost():
    row = parse_label(RESTORATION)
    assert (row["length_mm"], row["width_mm"], row["thickness_mm"]) == ("350", "200", "18")
    assert row["order_date"] == "2026-10-07"
    assert "swiss-krono" in row["material"]
    row = parse_label("60844\n350_200_18\n")
    assert row["length_mm"] == "350"


@pytest.mark.parametrize("text", [
    "",
    "Слава Україні Героям Слава!!!",
    "2026 10 07 1056",           # a date line is not three sizes
    "686 x 42318",               # OCR merged "422x18": no thickness, no guess
    "#142 -1 гір 686 x 4228",
    "9.74 Kr. 1221-18 мм",
])
def test_no_sizes_means_no_row(text):
    # Nothing is better than wrong sizes going to the arm.
    assert parse_label(text) == {}
    assert score(parse_label(text)) == 0


def test_implausible_sizes_are_skipped():
    assert parse_label("5000 x 200 x 18") == {}
    assert parse_label("350 x 200 x 90") == {}


def test_label_row_fills_csv_columns():
    row = label_row(parse_label(VIYAR), VIYAR)
    assert row["source"] == "label"
    assert "OCR" in row["status"]
    assert "1877 х 497 х 18" in row["label_text"]
    assert set(row) <= set(CSV_FIELDS)


class FakeTesseract:
    """Stands in for the program: returns one label's text, whatever the image."""
    psm = (6,)

    def __init__(self, text):
        self.text = text

    def upright_turns(self, image):
        return 0

    def read(self, image, psm=None):
        return self.text


def test_reader_reports_each_label_once():
    seen = []
    done = threading.Event()

    def on_label(row, text, orientation):
        seen.append(row)
        done.set()

    reader = LabelReader(FakeTesseract(VIYAR), on_label, log=lambda level, text: None).start()
    frame = np.full((200, 300), 255, np.uint8)
    for x in range(40, 260, 8):
        frame[96:104, x:x + 3] = 0  # letter-sized strokes for prepare() to find
    reader.offer(frame)
    assert done.wait(10)
    done.clear()
    reader.offer(frame)
    assert not done.wait(1.5)  # same label again: not reported twice
    reader.stop()
    assert len(seen) == 1 and seen[0]["length_mm"] == "1877"
