"""AprilTags in a camera picture, for the console's Cameras tab overlay and the tag bench (characterization/apriltag).
ROS-free: runs inside camera_snap on the computer the cameras are plugged into.

    finder = TagFinder("tag25h9", decimate=2)       # the C apriltag library (pupil_apriltags), else OpenCV's aruco
    dets = finder.detect(gray)                      # [Det(id, corners (4, 2) px, margin, hamming)]
    info = describe(det, Pinhole(...), size_m)      # the pose as the SLAM sees it (omni_slam.tags.tag_pose), as JSON

Corners are counter-clockwise from the tag's bottom left (apriltag's order, omni_slam.tags.tag_corners). The pose and
its gates (small, oblique, bad fit, low margin, ambiguous) are omni_slam's own, so what the overlay calls usable is
what tag_slam would take. Pixels are not undistorted: the D435's streams report no distortion, and the Arducam has no
calibration yet (its focal length comes from the lens: camera_snap.PiCam).
"""
from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    from omni_slam.tags import Pinhole, TagDetection, TagGate, tag_pose
except ImportError:
    try:                               # on the Pi: cam_snap.sh / the bench copy omni_slam/tags.py next to this file
        from tags import Pinhole, TagDetection, TagGate, tag_pose      # type: ignore
    except ImportError:                # a repo checkout without ROS: the source tree
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "omni_slam"))
        from omni_slam.tags import Pinhole, TagDetection, TagGate, tag_pose

FAMILIES = ("tag25h9", "tag36h11", "tag16h5")


@dataclass
class Det:
    tag_id: int
    corners: np.ndarray                # (4, 2) px, counter-clockwise from the tag's bottom left
    margin: float | None               # the decoder's decision margin (None: OpenCV doesn't give one)
    hamming: int = 0                   # bits corrected


def backend() -> str | None:
    try:
        import pupil_apriltags  # noqa: F401
        return "apriltag"
    except ImportError:
        pass
    try:
        import cv2
        return "opencv" if hasattr(cv2, "aruco") else None
    except ImportError:
        return None


class TagFinder:
    """One detector (family, decimation): the C apriltag library when installed, else OpenCV's aruco (what
    omni_calib.detect and today's tag_detector node use). Not re-entrant: a lock serialises callers."""

    def __init__(self, family: str = "tag25h9", decimate: float = 2.0, threads: int = 2, use: str | None = None):
        self.family, self.decimate, self.use = family, float(decimate), use or backend()
        self.lock = threading.Lock()
        if self.use == "apriltag":
            import pupil_apriltags

            class Kept(pupil_apriltags.Detector):
                # Nov 4, pupil-apriltags 1.0.4.post11 on the Pi (aarch64): destroying a tag25h9 detector corrupts the
                # heap (glibc aborts later: "malloc(): mismatching next->prev_size"); tag36h11 doesn't. Never free one:
                # they're cached (finder()), a few MB each
                def __del__(self):
                    pass

            self.det = Kept(families=family, nthreads=threads, quad_decimate=self.decimate, refine_edges=1,
                            decode_sharpening=0.25)
        elif self.use == "opencv":
            import cv2
            self.cv2 = cv2
            d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_APRILTAG_{family[3:]}"))
            if hasattr(cv2.aruco, "ArucoDetector"):
                p = cv2.aruco.DetectorParameters()
                p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
                p.aprilTagQuadDecimate = self.decimate
                det = cv2.aruco.ArucoDetector(d, p)
                self.det = lambda g: det.detectMarkers(g)
            else:                                            # OpenCV 4.6 (the Pi, WSL): the older API
                p = cv2.aruco.DetectorParameters_create()
                p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
                p.aprilTagQuadDecimate = self.decimate
                self.det = lambda g: cv2.aruco.detectMarkers(g, d, parameters=p)
        else:
            raise RuntimeError("no AprilTag detector here: pip install pupil-apriltags (or OpenCV with aruco)")

    def detect(self, gray: np.ndarray) -> list[Det]:
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        with self.lock:
            if self.use == "apriltag":
                return [Det(int(r.tag_id), np.asarray(r.corners, float), float(r.decision_margin), int(r.hamming))
                        for r in self.det.detect(gray)]
            corners, ids, _ = self.det(gray)
        if ids is None:
            return []
        # OpenCV: clockwise from the top left; apriltag's (ours): counter-clockwise from the bottom left
        return [Det(int(i), c.reshape(4, 2).astype(float)[[3, 2, 1, 0]], None) for c, i in zip(corners, ids.ravel())]


_finders: dict[tuple, TagFinder] = {}
_finders_lock = threading.Lock()


def finder(family: str = "tag25h9", decimate: float = 2.0) -> TagFinder:
    """A shared detector per (family, decimation): building one costs more than a detection."""
    key = (family, float(decimate))
    with _finders_lock:
        if key not in _finders:
            _finders[key] = TagFinder(family, decimate)
        return _finders[key]


def gray_of(img: np.ndarray) -> np.ndarray:
    if img.ndim == 2:
        return img
    try:
        import cv2
        return cv2.cvtColor(np.ascontiguousarray(img[..., :3]), cv2.COLOR_RGB2GRAY)
    except ImportError:
        return (img[..., :3] @ np.array([0.299, 0.587, 0.114], np.float32)).astype(np.uint8)


def describe(det: Det, cam: Pinhole | None, size: float, gate: TagGate = TagGate()) -> dict:
    """One detection as JSON for the page: corners in 0..1 of the picture (so any display size can draw them) and,
    with a camera model, the pose omni_slam would compute: position (m, camera frame: x right, y down, z ahead),
    range, the angle off the tag's normal, reprojection fit and the mirror solution's, and the gates' verdict."""
    w = cam.width if cam else 1
    h = cam.height if cam else 1
    c = det.corners
    out = dict(id=det.tag_id, corners=[[round(x / w, 5), round(y / h, 5)] for x, y in c],
               margin=None if det.margin is None else round(det.margin, 1), hamming=det.hamming,
               side_px=round(float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)])), 1),
               corners_px=np.round(c, 2).tolist())
    if cam is None:
        return out
    p = tag_pose(cam, TagDetection(det.tag_id, c, 0.0, size, det.margin if det.margin is not None else 100.0), gate)
    t = p.t
    s = size / 2
    axes = cam.project(np.array([[0, 0, 0], [s, 0, 0], [0, s, 0], [0, 0, s]]) @ p.R.T + t)
    n = p.R[:, 2]                                         # the face's normal (out of the tag) in the camera frame
    out.update(xyz=[round(float(v), 4) for v in t], range=round(p.range, 4),
               bearing_deg=round(math.degrees(math.atan2(t[0], t[2])), 2),
               elevation_deg=round(math.degrees(math.atan2(-t[1], math.hypot(t[0], t[2]))), 2),
               angle_deg=round(p.angle_deg, 2),
               normal_yaw_deg=round(math.degrees(math.atan2(-n[0], -n[2])), 2),
               normal_pitch_deg=round(math.degrees(math.atan2(n[1], math.hypot(n[0], n[2]))), 2),
               rms_px=round(p.rms_px, 3), alt_rms_px=None if math.isinf(p.alt_rms_px) else round(p.alt_rms_px, 3),
               ok=p.ok, ambiguous=p.ambiguous, reasons=p.reasons,
               axes=[[round(float(u) / w, 5), round(float(v) / h, 5)] for u, v in axes], size_m=size)
    out["R"] = np.round(p.R, 6).tolist()
    return out


def find(img: np.ndarray, cam: Pinhole | None, family: str = "tag25h9", size: float = 0.2,
         decimate: float = 2.0) -> tuple[list[dict], float]:
    """Detect and describe every tag in a picture; returns (tags, detection ms)."""
    f = finder(family, decimate)
    t0 = time.perf_counter()
    dets = f.detect(gray_of(img))
    ms = (time.perf_counter() - t0) * 1e3
    return [describe(d, cam, size) for d in dets], ms
