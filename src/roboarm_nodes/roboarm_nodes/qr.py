"""ctypes binding to libzbar: an 8-bit grayscale frame in, decoded QR codes out.

Carried over unchanged from upd-rasp-stream-tcp/qr_detector.py. Neither
Debian trixie nor the Yocto image has python3-pyzbar, but both have the
libzbar shared library itself, so the handful of calls actually needed are
bound here by hand. Imports nothing from ROS, so it can be tested on its own.
"""

import ctypes
import ctypes.util

ZBAR_NONE = 0
ZBAR_QRCODE = 64
ZBAR_CFG_ENABLE = 0


def _fourcc(code):
    return ord(code[0]) | ord(code[1]) << 8 | ord(code[2]) << 16 | ord(code[3]) << 24


GRAY_FORMAT = _fourcc("Y800")


def _load():
    path = ctypes.util.find_library("zbar") or "libzbar.so.0"
    lib = ctypes.CDLL(path)

    lib.zbar_version.argtypes = [ctypes.POINTER(ctypes.c_uint)] * 3
    lib.zbar_image_scanner_create.restype = ctypes.c_void_p
    lib.zbar_image_scanner_destroy.argtypes = [ctypes.c_void_p]
    lib.zbar_image_scanner_set_config.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.zbar_image_scanner_set_config.restype = ctypes.c_int

    lib.zbar_image_create.restype = ctypes.c_void_p
    lib.zbar_image_destroy.argtypes = [ctypes.c_void_p]
    lib.zbar_image_set_format.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    lib.zbar_image_set_size.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]
    lib.zbar_image_set_data.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p]

    lib.zbar_scan_image.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.zbar_scan_image.restype = ctypes.c_int

    lib.zbar_image_first_symbol.argtypes = [ctypes.c_void_p]
    lib.zbar_image_first_symbol.restype = ctypes.c_void_p
    lib.zbar_symbol_next.argtypes = [ctypes.c_void_p]
    lib.zbar_symbol_next.restype = ctypes.c_void_p
    lib.zbar_symbol_get_type.argtypes = [ctypes.c_void_p]
    lib.zbar_symbol_get_type.restype = ctypes.c_int
    lib.zbar_symbol_get_data.argtypes = [ctypes.c_void_p]
    lib.zbar_symbol_get_data.restype = ctypes.c_void_p
    lib.zbar_symbol_get_data_length.argtypes = [ctypes.c_void_p]
    lib.zbar_symbol_get_data_length.restype = ctypes.c_uint
    lib.zbar_symbol_get_quality.argtypes = [ctypes.c_void_p]
    lib.zbar_symbol_get_quality.restype = ctypes.c_int
    lib.zbar_symbol_get_loc_size.argtypes = [ctypes.c_void_p]
    lib.zbar_symbol_get_loc_size.restype = ctypes.c_uint
    lib.zbar_symbol_get_loc_x.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.zbar_symbol_get_loc_x.restype = ctypes.c_int
    lib.zbar_symbol_get_loc_y.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.zbar_symbol_get_loc_y.restype = ctypes.c_int
    lib.zbar_get_symbol_name.argtypes = [ctypes.c_int]
    lib.zbar_get_symbol_name.restype = ctypes.c_char_p
    return lib


_lib = _load()


def version():
    major, minor, patch = ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint()
    _lib.zbar_version(ctypes.byref(major), ctypes.byref(minor), ctypes.byref(patch))
    return "{}.{}.{}".format(major.value, minor.value, patch.value)


def _bbox_of(corners):
    if not corners:
        return None
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    return (min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))


class Code:
    """One decoded symbol: payload, quality, and where it sat in the frame.

    `corners` is the polygon zbar traced around the symbol (usually the four
    corners of the QR finder pattern, in perimeter order) in the pixel space
    of the frame that was scanned. `bbox` is the axis-aligned box around
    those corners, kept for callers that just want a quick rectangle.
    """

    __slots__ = ("kind", "raw", "quality", "corners", "bbox")

    def __init__(self, kind, raw, quality, corners):
        self.kind = kind
        self.raw = raw
        self.quality = quality
        self.corners = corners
        self.bbox = _bbox_of(corners)

    @property
    def text(self):
        """Payload as text; undecodable bytes are kept visible, not dropped."""
        return self.raw.decode("utf-8", "replace")

    def __repr__(self):
        return "Code({} {!r})".format(self.kind, self.text)


class Scanner:
    """Thin wrapper around one zbar_image_scanner_t.

    Not thread-safe - QRDetector keeps exactly one, used only from its own
    scan thread.
    """

    def __init__(self, qr_only=True):
        self._scanner = _lib.zbar_image_scanner_create()
        if not self._scanner:
            raise RuntimeError("zbar_image_scanner_create failed")
        if qr_only:
            # Everything off, then QR back on: skips the 1D barcode passes,
            # which are pure overhead here and an occasional source of false
            # reads.
            _lib.zbar_image_scanner_set_config(
                self._scanner, ZBAR_NONE, ZBAR_CFG_ENABLE, 0)
            _lib.zbar_image_scanner_set_config(
                self._scanner, ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1)

    def scan_gray(self, data, width, height):
        """Scan one 8-bit grayscale frame. `data` must be width*height bytes."""
        if self._scanner is None:
            raise RuntimeError("scanner is closed")
        expected = width * height
        view = memoryview(data)
        if not view.contiguous:
            raise ValueError("frame must be contiguous")
        # A 2D view slices by rows, so flatten to bytes before taking the plane.
        if view.ndim != 1 or view.format != "B":
            view = view.cast("B")
        if len(view) < expected:
            raise ValueError("expected {} bytes, got {}".format(expected, len(view)))
        buf = bytes(view[:expected])

        image = _lib.zbar_image_create()
        if not image:
            raise RuntimeError("zbar_image_create failed")
        try:
            _lib.zbar_image_set_format(image, GRAY_FORMAT)
            _lib.zbar_image_set_size(image, width, height)
            _lib.zbar_image_set_data(image, buf, len(buf), None)

            if _lib.zbar_scan_image(self._scanner, image) <= 0:
                return []

            codes = []
            symbol = _lib.zbar_image_first_symbol(image)
            while symbol:
                length = _lib.zbar_symbol_get_data_length(symbol)
                pointer = _lib.zbar_symbol_get_data(symbol)
                raw = ctypes.string_at(pointer, length) if pointer else b""
                kind = _lib.zbar_get_symbol_name(
                    _lib.zbar_symbol_get_type(symbol)).decode()
                codes.append(Code(kind, raw,
                                  _lib.zbar_symbol_get_quality(symbol),
                                  _corners(symbol)))
                symbol = _lib.zbar_symbol_next(symbol)
            return codes
        finally:
            _lib.zbar_image_destroy(image)

    def close(self):
        if self._scanner is not None:
            _lib.zbar_image_scanner_destroy(self._scanner)
            self._scanner = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def _corners(symbol):
    points = _lib.zbar_symbol_get_loc_size(symbol)
    return [(_lib.zbar_symbol_get_loc_x(symbol, i),
             _lib.zbar_symbol_get_loc_y(symbol, i))
            for i in range(points)]
