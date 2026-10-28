"""Target and tag detections from real images (design.md 10): OpenCV on the robot and in WSL (4.6 there: the aruco
module's older API; 4.7+ the ArucoDetector class). The solvers take what these return; the sim's tests skip this and
project the targets themselves.

  checkerboard_corners(gray, cols, rows)    sub-pixel inner corners (findChessboardCornersSB), or None
  apriltags(gray)                           [(id, corners (4, 2) counter-clockwise from the bottom left, margin)]
  aprilgrid_view(gray, ids, pts)            (obj, uv) of the AprilGrid corners seen (any subset of the tags)
"""

from __future__ import annotations

import numpy as np


def _cv():
    import cv2                                   # robot / WSL only
    return cv2


def checkerboard_corners(gray, cols: int, rows: int):
    cv2 = _cv()
    ok, c = cv2.findChessboardCornersSB(gray, (cols, rows), flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return c.reshape(-1, 2).astype(float) if ok else None


def apriltags(gray, family: str = "36h11"):
    cv2 = _cv()
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_APRILTAG_{family}"))
    if hasattr(cv2.aruco, "ArucoDetector"):
        p = cv2.aruco.DetectorParameters()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        corners, ids, _ = cv2.aruco.ArucoDetector(d, p).detectMarkers(gray)
    else:
        p = cv2.aruco.DetectorParameters_create()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
    out = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel()):
        c = c.reshape(4, 2).astype(float)
        # OpenCV gives clockwise from the top left; apriltag's convention (ours) is counter-clockwise from bottom left
        out.append((int(i), c[[3, 2, 1, 0]], None))
    return out


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
