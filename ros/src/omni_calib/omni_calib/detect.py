"""Target and tag detections from real images (design.md 10). The solvers take what these return; the sim's tests skip
this and project the targets themselves.

  checkerboard_corners(gray, cols, rows)    sub-pixel inner corners (OpenCV findChessboardCornersSB), or None
  apriltags(gray)                           [(id, corners (4, 2) counter-clockwise from the bottom left, margin)]
  aprilgrid_view(gray, ids, pts)            (obj, uv) of the AprilGrid corners seen (any subset of the tags)
  apriltag_backend()                        "apriltag", "opencv" or None: what apriltags() uses here

AprilTags are found by the C apriltag library (pupil_apriltags, ros/requirements-robot.txt) when it is installed, else
by OpenCV's aruco module with apriltag corner refinement (4.6 on the Pi and in WSL: the older API; 4.7+ the
ArucoDetector class). The C library is the robot's detector since Nov 12 (design.md 4.6): on the Pi 4 tag bench it was
~3x faster at decimate 2, its tilt scatter 3-7x smaller, and it reports the decoder's margin, which tag_slam's
min_margin gate needs (OpenCV gives none: margin None). characterization/apriltag/README.md.
"""

from __future__ import annotations

import threading

import numpy as np


def _cv():
    import cv2                                   # robot / WSL only
    return cv2


def checkerboard_corners(gray, cols: int, rows: int):
    cv2 = _cv()
    ok, c = cv2.findChessboardCornersSB(gray, (cols, rows), flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return c.reshape(-1, 2).astype(float) if ok else None


def apriltag_backend() -> str | None:
    """The detector apriltags() uses by default: "apriltag" (the C library), "opencv" (aruco), or None (neither)."""
    try:
        import pupil_apriltags  # noqa: F401
        return "apriltag"
    except ImportError:
        pass
    try:
        return "opencv" if hasattr(_cv(), "aruco") else None
    except ImportError:
        return None


def _family(family: str) -> str:
    """"36h11" or "tag36h11" -> "36h11"."""
    return family[3:] if family.startswith("tag") else family


_c_detectors: dict[tuple, tuple] = {}            # (family, decimate, threads) -> (detector, its lock)
_c_lock = threading.Lock()


def _c_detector(family: str, decimate: float, threads: int):
    """A cached C detector: building one costs more than a detection. Never freed: on the Pi (aarch64,
    pupil-apriltags 1.0.4.post11, Nov 4) destroying a tag25h9 detector corrupts the heap (omni_console/apriltag_view)."""
    key = (family, float(decimate), int(threads))
    with _c_lock:
        if key not in _c_detectors:
            import pupil_apriltags

            class Kept(pupil_apriltags.Detector):
                def __del__(self):
                    pass

            det = Kept(families=f"tag{family}", nthreads=int(threads), quad_decimate=float(decimate), refine_edges=1,
                       decode_sharpening=0.25)
            _c_detectors[key] = (det, threading.Lock())
        return _c_detectors[key]


def apriltags(gray, family: str = "36h11", decimate: float = 1.0, threads: int = 2, backend: str | None = None):
    """Every tag of `family` in a grey picture: [(id, corners (4, 2) px counter-clockwise from the tag's bottom left
    (apriltag's order), margin)]. margin is the C decoder's decision margin, None from OpenCV. decimate > 1 finds the
    quads on a picture that much smaller (faster, shorter reach; the corners are refined at full resolution).
    backend: "apriltag" or "opencv" to force one; None picks apriltag_backend()."""
    fam = _family(family)
    use = backend or apriltag_backend()
    gray = np.ascontiguousarray(gray, dtype=np.uint8)
    if use == "apriltag":
        det, lock = _c_detector(fam, decimate, threads)
        with lock:                               # a detector is not re-entrant
            found = det.detect(gray)
        return [(int(r.tag_id), np.asarray(r.corners, float), float(r.decision_margin)) for r in found]
    if use != "opencv":
        raise RuntimeError("no AprilTag detector here: pip install pupil-apriltags (or OpenCV with aruco)")
    cv2 = _cv()
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_APRILTAG_{fam}"))
    if hasattr(cv2.aruco, "ArucoDetector"):
        p = cv2.aruco.DetectorParameters()
    else:
        p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    if decimate > 1.0:
        p.aprilTagQuadDecimate = float(decimate)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(d, p).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
    out = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel()):
        out.append((int(i), c.reshape(4, 2).astype(float)[OPENCV_TO_APRILTAG], None))
    return out


# OpenCV's AprilTag dictionaries are the official tags turned 180 deg (every id of 16h5, 25h9, 36h11; OpenCV 4.6 and
# 5.0, checked Nov 12 against the C library's own family tables). Its corners run clockwise on screen from what it
# takes for the top left, which on a real (official) tag is the bottom right: raw [BR, BL, TL, TR] -> apriltag's
# [BL, BR, TR, TL]. Before Nov 12 this was [3, 2, 1, 0], right only for OpenCV's own marker images: on a printed
# official tag the pose came out rolled 180 deg about the tag's normal (position and normal unaffected).
OPENCV_TO_APRILTAG = [1, 0, 3, 2]


def aprilgrid_view(gray, ids: np.ndarray, pts: np.ndarray, family: str = "36h11"):
    index = {int(i): k for k, i in enumerate(ids)}
    obj, uv = [], []
    for tid, c, _ in apriltags(gray, family):
        for k in range(4):
            j = index.get(4 * tid + k)
            if j is not None:
                obj.append(pts[j])
                uv.append(c[k])
    return (np.array(obj), np.array(uv)) if obj else None
