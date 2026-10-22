"""The catch path's perception (design.md 5, 6, 30): camera frames -> ball detector -> tracking filter, in one process.

Every stereo frame pair: find the ball in each image (detector.py), place the cameras where the robot was when the
frames were exposed (the MCU's odometry for x, y, yaw, the BNO085 for roll and pitch, the calibrated extrinsics from
the robot config), and update the filter (tracker.ImageTracker) at the exposure time. The behavior reads the track
as a TrackView (tracker.py).

No ROS here: omni_catch.perception_node wraps it on the robot, the sim's in-process harness calls it directly.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from omni_catch.detector import Detector, DetectorConfig
from omni_catch.flight import BallFlight
from omni_catch.robot_config import RobotConfig
from omni_catch.tracker import Blob, ImageTracker, TrackerConfig, TrackView, sphere_image


@dataclass
class CameraModel:
    pos_b: np.ndarray             # in base_link
    R_b: np.ndarray               # columns: the camera's axes in base_link (it looks along -z)
    f: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_config(cls, c: dict) -> "CameraModel":
        f = (c["height"] / 2) / math.tan(math.radians(c["fovy_deg"]) / 2)
        return cls(np.array(c["pos"]), np.array(c["R"]), f, c["width"] / 2, c["height"] / 2, c["width"], c["height"])


def rot_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Perception:
    def __init__(self, robot: RobotConfig | None = None, tcfg: TrackerConfig | None = None,
                 dcfg: DetectorConfig | None = None):
        self.robot = robot or RobotConfig()
        b = self.robot.ball
        self.flight = BallFlight(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])
        self.cams = [CameraModel.from_config(c) for c in self.robot.cameras]
        self.tcfg = tcfg or TrackerConfig()
        self.tracker = ImageTracker(self.flight, self.tcfg, self.cams[0].f)
        self.det = Detector(dcfg)
        self.R_ball = b["radius"]
        self.poses: deque = deque(maxlen=600)          # (t, x, y, yaw, roll, pitch), odom frame
        self.t_last_det = -math.inf
        self.frames = self.detections = self.stereo = 0

    def set_play_area(self, center, half, z_max: float = 4.0, margin: float = 0.5):
        """Where a throw can start, in the odom frame (the behaviour's play area from the map): acquisition among
        several candidates ignores ball-coloured points outside it."""
        self.tracker.acq_box = ((float(center[0]), float(center[1])),
                                (float(half[0]) + margin, float(half[1]) + margin), float(z_max))

    # -- where the robot was --------------------------------------------------------------------------------------
    def add_pose(self, t: float, x: float, y: float, yaw: float, roll: float = 0.0, pitch: float = 0.0):
        if self.poses and t <= self.poses[-1][0]:
            return
        self.poses.append((t, x, y, yaw, roll, pitch))

    def pose_at(self, t: float):
        P = self.poses
        if not P:
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        if t <= P[0][0]:
            return P[0][1:]
        if t >= P[-1][0]:
            return P[-1][1:]
        lo, hi = 0, len(P) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if P[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = P[lo], P[hi]
        f = (t - a[0]) / (b[0] - a[0])
        dyaw = (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw,
                a[4] + f * (b[4] - a[4]), a[5] + f * (b[5] - a[5]))

    def cams_at(self, t: float):
        x, y, yaw, roll, pitch = self.pose_at(t)
        Rb = rot_zyx(yaw, pitch, roll)
        pb = np.array([x, y, self.robot.base_link_height])
        return [(pb + Rb @ c.pos_b, Rb @ c.R_b, c.f, c.cx, c.cy, self.R_ball) for c in self.cams]

    # -- frames ----------------------------------------------------------------------------------------------------
    def _predict_px(self, k: int, t: float, cams):
        x, _ = self.tracker.state_at(t)
        c, Rc, f, cx, cy, Rb = cams[k]
        img = sphere_image(Rc.T @ (x[:3] - c), Rb, f)
        if img is None:
            return None
        return cx + img[0], cy + img[1], img[2]

    def process(self, t_cap: float, frames) -> list:
        """One exposure: frames[k] is camera k's frame (or None if it delivered nothing)."""
        self.frames += 1
        cams = self.cams_at(t_cap)
        tracking = self.tracker.x is not None
        blobs = []
        n_acq = self.det.cfg.acquire_candidates
        if not tracking and n_acq > 1:
            # acquisition among several candidates: the tracker starts on the one that moves (design.md 31)
            cands = [[Blob(k, *c, t_cap) for c in self.det.candidates(fr, n_acq)] if fr is not None else []
                     for k, fr in enumerate(frames)]
            blobs = self.tracker.init_candidates(t_cap, cands, cams)
            if blobs:
                self.detections += 1
                self.stereo += len(blobs) == 2
                self.t_last_det = t_cap
            return blobs
        for k, fr in enumerate(frames):
            if fr is None:
                continue
            pred = self._predict_px(k, t_cap, cams) if tracking else None
            det = self.det.detect(fr, pred)
            if det is not None:
                blobs.append(Blob(k, det[0], det[1], det[2], det[3], t_cap))
        if blobs:
            self.detections += 1
            self.stereo += len(blobs) == 2
            self.t_last_det = t_cap
            self.tracker.update_blobs(t_cap, blobs, cams)
        tr = self.tracker
        if tr.x is not None:
            # a track the ball no longer feeds (caught, landed, out of view), or one that stopped: drop it
            if t_cap - self.t_last_det > self.tcfg.max_gap or float(np.linalg.norm(tr.x[3:])) < 0.8:
                tr.reset()
        return blobs

    # -- outputs -------------------------------------------------------------------------------------------------
    def ready(self) -> bool:
        return self.tracker.x is not None and self.tracker.n >= self.tcfg.min_frames

    def view(self) -> TrackView | None:
        tr = self.tracker
        if tr.x is None:
            return None
        return TrackView(self.flight, self.tcfg.accel_sigma, tr.t, tr.x.copy(), tr.P.copy(), tr.n, self.ready())

    def reset(self):
        self.tracker.reset()
