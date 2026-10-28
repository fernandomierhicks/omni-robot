"""AprilTag observations: a detection's pose and whether to trust it (design.md 10).

A detection (apriltag's, or OpenCV's 36h11: omni_calib.detect) is a tag id and its four corners in the image. Its pose
in the camera comes from those corners (the homography, refined on the reprojection). What makes one poor, so it is
kept out of the graph or kept only as a weak corner factor:

  small         the tag spans few pixels (far, or a small tag): corner noise dominates; < min_side_px refused
  oblique       seen nearly edge-on: the pose's angle is poorly determined; > max_angle_deg refused
  ambiguous     a planar target's pose has a mirror solution (the tag tipped the other way about the view ray) that
                fits nearly as well when the tag is small or far: such a view doesn't start a landmark (it can still
                add its corners to one that exists, where the graph's estimate picks the right branch)
  bad fit       the refined pose's reprojection RMS > max_rms_px: a misdecode, a bent print, motion blur
  low margin    the decoder's decision margin < min_margin (apriltag's own confidence)

Pixels are undistorted first (the detector node does it with the camera's calibration): the graph uses a pinhole.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares


def rodrigues(r) -> np.ndarray:
    r = np.asarray(r, float)
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        x, y, z = r
        return np.eye(3) + np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def log_rot(R) -> np.ndarray:
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return v / 2
    if th > math.pi - 1e-6:
        A = (R + np.eye(3)) / 2
        k = A[:, int(np.argmax(np.diag(A)))]
        return th * k / np.linalg.norm(k)
    return th / (2 * math.sin(th)) * v


def tag_corners(size: float) -> np.ndarray:
    """The four corners in the tag's frame (z out of its face, x right, y up), counter-clockwise from bottom left."""
    h = size / 2
    return np.array([(-h, -h, 0.0), (h, -h, 0.0), (h, h, 0.0), (-h, h, 0.0)])


@dataclass
class Pinhole:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 1280
    height: int = 800

    @classmethod
    def from_fovy(cls, fovy_deg: float, width: int = 1280, height: int = 800) -> "Pinhole":
        f = height / 2 / math.tan(math.radians(fovy_deg) / 2)
        return cls(f, f, width / 2, height / 2, width, height)

    def project(self, P) -> np.ndarray:
        P = np.atleast_2d(P)
        z = np.where(P[..., 2] > 1e-6, P[..., 2], np.nan)
        return np.stack([self.fx * P[..., 0] / z + self.cx, self.fy * P[..., 1] / z + self.cy], axis=-1)


@dataclass
class TagDetection:
    tag_id: int
    corners: np.ndarray            # (4, 2) undistorted pixels, counter-clockwise from the tag's bottom left
    t: float                       # the frame's exposure time (camera_time.py), Jetson clock
    size: float = 0.16             # m, the tag's black square (from the tag map / a default per family)
    margin: float = 100.0          # the decoder's decision margin
    camera: str = "cam_left_optical"


@dataclass
class TagPose:
    R: np.ndarray                  # tag -> camera
    t: np.ndarray
    rms_px: float
    alt_rms_px: float              # the mirror solution's fit
    side_px: float
    angle_deg: float               # between the tag's normal and the view ray
    ok: bool = True
    ambiguous: bool = False
    reasons: list = field(default_factory=list)

    @property
    def range(self) -> float:
        return float(np.linalg.norm(self.t))


def _homography(src, dst):
    A = []
    for (x, y), (u, v) in zip(src, dst):
        A.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        A.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, Vt = np.linalg.svd(np.array(A))
    H = Vt[-1].reshape(3, 3)
    return H / H[2, 2]


def _refine(cam: Pinhole, obj, uv, R, t):
    def f(x):
        return (cam.project(obj @ rodrigues(x[:3]).T + x[3:]) - uv).ravel()
    sol = least_squares(f, np.concatenate([log_rot(R), t]), method="lm")
    return rodrigues(sol.x[:3]), sol.x[3:], float(np.sqrt(np.mean(sol.fun.reshape(-1, 2) ** 2) * 2))


@dataclass
class TagGate:
    min_side_px: float = 18.0
    max_angle_deg: float = 65.0
    max_rms_px: float = 1.5
    min_margin: float = 30.0
    max_range: float = 6.0
    ambiguity_ratio: float = 2.0   # the mirror solution must fit this much worse to start a landmark ...
    noise_px: float = 0.5          # ... than the corners' noise at least: a lucky best fit (0.01 px) doesn't count


def tag_pose(cam: Pinhole, det: TagDetection, gate: TagGate = TagGate()) -> TagPose:
    obj = tag_corners(det.size)
    uv = np.asarray(det.corners, float)
    xy = np.column_stack([(uv[:, 0] - cam.cx) / cam.fx, (uv[:, 1] - cam.cy) / cam.fy])
    H = _homography(obj[:, :2], xy)
    h1, h2, h3 = H[:, 0], H[:, 1], H[:, 2]
    s = 2.0 / (np.linalg.norm(h1) + np.linalg.norm(h2))
    if h3[2] * s < 0:
        s = -s
    R0 = np.column_stack([s * h1, s * h2, np.cross(s * h1, s * h2)])
    U, _, Vt = np.linalg.svd(R0)
    R0 = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    R, t, rms = _refine(cam, obj, uv, R0, s * h3)
    # the mirror solution: the tag tipped the other way about the view ray, refined from there. The homography can
    # start in either basin, so the better of the two is the pose and the other says how ambiguous it is
    v = t / np.linalg.norm(t)
    n = R[:, 2]
    n_alt = 2 * (n @ v) * v - n
    ax = np.cross(n, n_alt)
    ang = math.atan2(np.linalg.norm(ax), float(n @ n_alt))
    R_flip = rodrigues(ax / max(np.linalg.norm(ax), 1e-12) * ang) @ R if np.linalg.norm(ax) > 1e-9 else R
    R_alt, t_alt, rms_alt = _refine(cam, obj, uv, R_flip, t)
    if np.linalg.norm(log_rot(R.T @ R_alt)) < math.radians(2.0):
        rms_alt = math.inf                          # it fell back into the same minimum: there is no second one
    elif rms_alt < rms:
        R, t, rms, R_alt, t_alt, rms_alt = R_alt, t_alt, rms_alt, R, t, rms
    side = float(np.mean([np.linalg.norm(uv[k] - uv[(k + 1) % 4]) for k in range(4)]))
    angle = math.degrees(math.acos(min(1.0, abs(float(R[:, 2] @ v)))))
    p = TagPose(R, t, rms, rms_alt, side, angle)
    if side < gate.min_side_px:
        p.reasons.append("small")
    if angle > gate.max_angle_deg:
        p.reasons.append("oblique")
    if rms > gate.max_rms_px:
        p.reasons.append("bad fit")
    if det.margin < gate.min_margin:
        p.reasons.append("low margin")
    if p.range > gate.max_range:
        p.reasons.append("far")
    p.ok = not p.reasons
    p.ambiguous = rms_alt < gate.ambiguity_ratio * max(rms, gate.noise_px)
    return p
