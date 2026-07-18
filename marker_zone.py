"""
ArUco-based dynamic detection zone — Apples & Oranges sorter.

Print zone_markers.png and place one marker at each corner of the platform:
    Marker ID 0 -> Top-Left      Marker ID 1 -> Top-Right
    Marker ID 3 -> Bot-Left      Marker ID 2 -> Bot-Right

The camera auto-detects the markers every few frames and computes the zone
polygon + homography, correcting for camera angle/position automatically.
The platform is divided into a 2x2 grid (S1 S2 / S3 S4); each segment's
center in canonical space corresponds to the physical white dot, so pick
points stay aligned even when the camera moves.

Generate printable markers (run once):
    python marker_zone.py
"""
import cv2
import numpy as np

ZONE_MARKER_IDS = [0, 1, 2, 3]
_DICT_ID = cv2.aruco.DICT_4X4_50

# Canonical rectangle for perspective mapping. 2x2 grid -> 150x150 cells.
_CANON_W, _CANON_H = 300, 300
_GRID_COLS, _GRID_ROWS = 2, 2

# S1 | S2
# S3 | S4
SEGMENT_LABELS = [f"S{r * _GRID_COLS + c + 1}"
                  for r in range(_GRID_ROWS) for c in range(_GRID_COLS)]


def _check_aruco():
    if not hasattr(cv2, "aruco"):
        raise ImportError(
            "OpenCV ArUco module not found.\n"
            "Install: pip install opencv-contrib-python\n"
            "(uninstall opencv-python first if installed)")


def _get_dict():
    _check_aruco()
    try:
        return cv2.aruco.getPredefinedDictionary(_DICT_ID)
    except AttributeError:
        return cv2.aruco.Dictionary_get(_DICT_ID)  # old API


def _tuned_params(params):
    """Settings that help with dark, noisy, low-res, or tightly-cut markers."""
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 45
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.015   # allow small markers in frame
    params.maxErroneousBitsInBorderRate = 0.6
    params.errorCorrectionRate = 0.8
    try:
        params.detectInvertedMarker = True  # tolerate inverted prints
    except AttributeError:
        pass
    try:
        # subpixel corners stabilize the homography on noisy feeds
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    except AttributeError:
        pass
    return params


def _make_detector():
    d = _get_dict()
    try:
        params = _tuned_params(cv2.aruco.DetectorParameters())
        return cv2.aruco.ArucoDetector(d, params)
    except AttributeError:
        return None  # old API — standalone detectMarkers


_CLAHE = None


def _detect_markers(frame, detector):
    """Return {marker_id: center(x, y)} for all zone markers found.
    Tries the raw gray frame first, then a CLAHE contrast-boosted version
    (rescues dark / noisy / low-light feeds)."""
    global _CLAHE
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if _CLAHE is None:
        _CLAHE = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

    def run(img):
        if detector is not None:
            corners, ids, _ = detector.detectMarkers(img)
        else:
            d = _get_dict()
            params = _tuned_params(cv2.aruco.DetectorParameters_create())
            corners, ids, _ = cv2.aruco.detectMarkers(img, d, parameters=params)
        out = {}
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if int(mid) in ZONE_MARKER_IDS:
                    out[int(mid)] = corners[i][0].mean(axis=0)
        return out

    found = run(gray)
    if len(found) < 4:                        # retry on contrast-boosted frame
        boosted = _CLAHE.apply(cv2.GaussianBlur(gray, (3, 3), 0))
        for mid, c in run(boosted).items():
            found.setdefault(mid, c)
    return found


class MarkerZone:
    """
    Tracks 4 ArUco markers, maintains zone polygon + homography, and maps
    points to 2x2 grid segments.

    Usage:
        zone = MarkerZone()
        zone.update(frame)                 # every frame (throttled inside)
        seg = zone.segment_of_point(x, y)  # 'S1'..'S4' or None
        cx, cy = zone.segment_center('S1') # pixel coords of the white dot
        zone.draw_overlay(frame)
    """

    def __init__(self, update_every: int = 15):
        self._detector = _make_detector()
        self.zone_poly = None      # np.float32 (4,2): TL TR BR BL
        self._H = None             # frame -> canonical
        self._H_inv = None         # canonical -> frame
        self.update_every = update_every
        self._tick = 0
        self._markers = {}
        self.status = "PLACE MARKERS"

    # ------------------------------------------------------------------ core
    def update(self, frame):
        self._tick += 1
        if self._tick % self.update_every != 0 and self.zone_poly is not None:
            return self.zone_poly
        found = _detect_markers(frame, self._detector)
        self._markers = found
        if all(mid in found for mid in ZONE_MARKER_IDS):
            self.zone_poly = np.array(
                [found[0], found[1], found[2], found[3]], dtype=np.float32)
            dst = np.array([[0, 0], [_CANON_W, 0],
                            [_CANON_W, _CANON_H], [0, _CANON_H]],
                           dtype=np.float32)
            self._H = cv2.getPerspectiveTransform(self.zone_poly, dst)
            self._H_inv = np.linalg.inv(self._H)
            self.status = "ZONE LOCKED"
        else:
            n = len(found)
            if self.zone_poly is not None:
                self.status = f"ZONE LOCKED (re-scanning {n}/4)"
            else:
                self.status = f"CALIBRATING ({n}/4 markers)" if n else "PLACE MARKERS"
        return self.zone_poly

    @property
    def locked(self) -> bool:
        return self.zone_poly is not None

    def point_in_zone(self, x, y) -> bool:
        if self.zone_poly is None:
            return False
        return cv2.pointPolygonTest(self.zone_poly, (float(x), float(y)), False) >= 0

    def _to_canonical(self, x, y):
        if self._H is None:
            return None
        p = cv2.perspectiveTransform(
            np.array([[[float(x), float(y)]]], dtype=np.float32), self._H)[0][0]
        return float(p[0]), float(p[1])

    def _to_frame(self, u, v):
        if self._H_inv is None:
            return None
        p = cv2.perspectiveTransform(
            np.array([[[float(u), float(v)]]], dtype=np.float32),
            self._H_inv.astype(np.float32))[0][0]
        return int(p[0]), int(p[1])

    def segment_of_point(self, x, y):
        """'S1'..'S4' for a pixel point, or None if outside the zone."""
        if not self.point_in_zone(x, y):
            return None
        c = self._to_canonical(x, y)
        if c is None:
            return None
        u, v = c
        col = min(_GRID_COLS - 1, max(0, int(u // (_CANON_W / _GRID_COLS))))
        row = min(_GRID_ROWS - 1, max(0, int(v // (_CANON_H / _GRID_ROWS))))
        return SEGMENT_LABELS[row * _GRID_COLS + col]

    def segment_center(self, label):
        """Pixel coords of a segment's canonical center (= the white dot)."""
        if label not in SEGMENT_LABELS or self._H_inv is None:
            return None
        idx = SEGMENT_LABELS.index(label)
        row, col = divmod(idx, _GRID_COLS)
        u = (col + 0.5) * (_CANON_W / _GRID_COLS)
        v = (row + 0.5) * (_CANON_H / _GRID_ROWS)
        return self._to_frame(u, v)

    # -------------------------------------------------------------- overlay
    def draw_overlay(self, frame, segment_states=None):
        """
        Draws zone edges, 2x2 grid lines, segment labels and dot centers.
        segment_states: optional {label: dict(occupied=bool, cls=str)} to
        tint occupied segments.
        """
        if self.zone_poly is None:
            cv2.putText(frame, self.status, (16, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
            return frame

        poly = self.zone_poly.astype(int)
        cv2.polylines(frame, [poly], True, (90, 200, 255), 2, cv2.LINE_AA)

        # grid lines (canonical midlines projected to frame)
        for (a, b) in [((_CANON_W / 2, 0), (_CANON_W / 2, _CANON_H)),
                       ((0, _CANON_H / 2), (_CANON_W, _CANON_H / 2))]:
            p1, p2 = self._to_frame(*a), self._to_frame(*b)
            if p1 and p2:
                cv2.line(frame, p1, p2, (90, 200, 255), 1, cv2.LINE_AA)

        for label in SEGMENT_LABELS:
            c = self.segment_center(label)
            if not c:
                continue
            occupied = bool(segment_states and
                            segment_states.get(label, {}).get("occupied"))
            color = (60, 120, 255) if occupied else (160, 160, 160)
            cv2.circle(frame, c, 6, color, -1, cv2.LINE_AA)
            cv2.circle(frame, c, 10, color, 1, cv2.LINE_AA)
            cv2.putText(frame, label, (c[0] - 16, c[1] - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        return frame


# ---------------------------------------------------------------- generator
def _generate_marker_sheet(path="zone_markers.png", px=300, pad=60,
                           inverted=False):
    """
    inverted=False: standard black-square markers on a white sheet
                    (white quiet zone; best on light platforms).
    inverted=True:  color-flipped markers on a BLACK sheet — the whole
                    printed square blends into a black platform and only
                    the white cells show. Keep the black margin when
                    cutting: it IS the quiet zone for inverted markers.
    """
    d = _get_dict()
    labels = ["ID 0  [Top-Left]", "ID 1  [Top-Right]",
              "ID 2  [Bot-Right]", "ID 3  [Bot-Left]"]
    bg = 0 if inverted else 255
    fg = 255 if inverted else 0
    sheet = np.full((2 * (px + 2 * pad) + 80, 2 * (px + 2 * pad), ), bg, np.uint8)
    order = [0, 1, 3, 2]  # visual layout TL TR / BL BR
    for i, mid in enumerate(order):
        try:
            img = cv2.aruco.generateImageMarker(d, mid, px)
        except AttributeError:
            img = cv2.aruco.drawMarker(d, mid, px)
        if inverted:
            img = 255 - img
        r, c = divmod(i, 2)
        y = 80 + r * (px + 2 * pad) + pad
        x = c * (px + 2 * pad) + pad
        sheet[y:y + px, x:x + px] = img
        cv2.putText(sheet, labels[mid], (x, y + px + 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, fg, 2, cv2.LINE_AA)
    title = ("INVERTED zone markers - for BLACK platforms - keep the black margin when cutting"
             if inverted else
             "Apples & Oranges zone markers - print, cut, tape at platform corners")
    cv2.putText(sheet, title, (20, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, fg, 2, cv2.LINE_AA)
    cv2.imwrite(path, sheet)
    print(f"Wrote {path}")


if __name__ == "__main__":
    import sys
    if "--inverted" in sys.argv or "-i" in sys.argv:
        _generate_marker_sheet("zone_markers_inverted.png", inverted=True)
    else:
        _generate_marker_sheet()
        _generate_marker_sheet("zone_markers_inverted.png", inverted=True)
