"""The Goalie's ball, ROS-free: a ball rolling on the floor, from the D435's colour image registered to its depth (the
same pixels: color_vision's frames), tracked in the odom frame with its velocity. ball_track_node runs it while the
console's Goalie plays; simulation/v11 sil/studies/goalie.py runs it on rendered frames.

  detect   the pixels of the ball's colour (color_vision's hue ranges), their depth points (base_link); the ball's
           points are the ones within its radius of their median (a red rug or a toy elsewhere in the picture falls out);
           enough of them, as wide as a ball and up off the floor as a ball's front is: the ball's centre is their median
           pushed back half a radius along the view (the camera sees its front), at the ball's height
  track    a constant-velocity Kalman filter on the floor (x, y, vx, vy, odom); a detection farther than gate from the
           prediction starts a new track; nothing seen for lost s: no ball
The thresholds are estimates for a 15-25 cm play ball in a home (VERIFY on the real D435 and a real ball).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from omni_arm import color_vision as cv


@dataclass
class BallParams:
    color: str = "red"                  # color_vision's names, or {h: [lo, hi], s_min, v_min}
    diameter: float = 0.22              # m
    min_px: int = 40                    # fewer ball-coloured pixels with depth: not seen
    max_range: float = 4.5              # m from the robot
    robot_r: float = 0.25               # m: nearer the base's centre is the robot itself (its bumper)
    accel: float = 4.0                  # m/s^2: the filter's process noise (a kick, a bounce off a wall)
    meas: float = 0.03                  # m: a detection's error (est: the D435's depth noise at 2-3 m on a ball)
    gate: float = 0.6                   # m: farther from the prediction is a new ball (or a new throw)
    lost: float = 0.5                   # s


class BallTracker:
    def __init__(self, p: BallParams | None = None):
        self.p = p or BallParams()
        self.spec = cv.color_spec(self.p.color)
        self.x = None                    # [x, y, vx, vy] odom
        self.P = None
        self.t = None
        self.t_seen = -math.inf
        self.n = 0                       # detections in this track

    # -- one frame ---------------------------------------------------------------------------------------------------
    def _points(self, frame):
        """(H x W x 3 points in base_link, H x W valid) out to max_range plus a margin (color_vision's stop at the
        floor detector's range: too short for a ball rolled from across the room)."""
        z = frame.depth
        Hh, Ww = z.shape
        fx, fy, cx, cy = frame.K
        v, u = np.mgrid[0:Hh, 0:Ww]
        ok = np.isfinite(z) & (z > 0.1) & (z < self.p.max_range + 0.5)
        zz = np.where(ok, z, 0.0)
        pc = np.stack([(u - cx) / fx * zz, (v - cy) / fy * zz, zz], axis=-1)
        T = frame.T_base_cam
        return pc @ T[:3, :3].T + T[:3, 3], ok

    def detect(self, depth_frame, rgb: np.ndarray) -> np.ndarray | None:
        """The ball's centre in base_link (x, y, z), or None. depth_frame: floor_vision.DepthFrame; rgb: the colour
        image registered to it (resized to the depth's size if it differs)."""
        P, ok = self._points(depth_frame)
        img = cv._resize_nearest(rgb, ok.shape)
        h, s, v = cv.hsv(img)
        mask = cv.in_color(h, s, v, self.spec) & ok
        if mask.sum() < self.p.min_px:
            return None
        Q = P[mask]
        r = np.hypot(Q[:, 0], Q[:, 1])
        Q = Q[(r > self.p.robot_r) & (r < self.p.max_range)]
        if len(Q) < self.p.min_px:
            return None
        rad = self.p.diameter / 2
        m = np.median(Q, axis=0)
        near = Q[np.linalg.norm(Q - m, axis=1) < 1.2 * rad]
        if len(near) < self.p.min_px:
            return None
        m = np.median(near, axis=0)
        width = np.ptp(near[:, :2] @ np.array([-m[1], m[0]]) / max(1e-6, math.hypot(m[0], m[1])))
        z_top = np.percentile(near[:, 2], 95)
        if width < 0.4 * self.p.diameter or width > 1.6 * self.p.diameter:
            return None                  # a sliver (an edge of something) or something bigger than the ball
        if z_top < 0.5 * self.p.diameter or np.percentile(near[:, 2], 5) > rad:
            return None                  # flat on the floor (a rug), or not touching it (up on furniture)
        cam = depth_frame.T_base_cam[:3, 3]
        ray = m - cam
        ray[2] = 0.0
        n = np.linalg.norm(ray)
        c = m + (ray / n * rad * 0.5 if n > 1e-6 else 0.0)
        c[2] = rad
        return c

    # -- the track ---------------------------------------------------------------------------------------------------
    def update(self, t: float, xy_odom) -> None:
        """A frame at t: the ball's odom (x, y), or None when not seen."""
        if self.x is not None:
            self._predict(t)
        if xy_odom is None:
            if t - self.t_seen > self.p.lost:
                self.x, self.P, self.n = None, None, 0
            return
        z = np.asarray(xy_odom, float)
        if self.x is None or np.linalg.norm(z - self.x[:2]) > self.p.gate:
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            self.P = np.diag([self.p.meas ** 2, self.p.meas ** 2, 4.0, 4.0])
            self.t, self.t_seen, self.n = t, t, 1
            return
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        R = np.eye(2) * self.p.meas ** 2
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.t_seen = t
        self.n += 1

    def _predict(self, t: float):
        dt = max(0.0, t - self.t)
        self.t = t
        if dt == 0.0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.p.accel ** 2
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * q

    def state(self, t: float) -> dict:
        """The ball now (odom), extrapolated from the last update; seen False without one."""
        if self.x is None:
            return dict(seen=False, t=t)
        dt = max(0.0, t - self.t)
        x, y, vx, vy = self.x
        return dict(seen=True, t=t, x=round(float(x + vx * dt), 4), y=round(float(y + vy * dt), 4),
                    vx=round(float(vx), 3) if self.n >= 3 else 0.0, vy=round(float(vy), 3) if self.n >= 3 else 0.0,
                    age=round(t - self.t_seen, 3), n=self.n)

    def step(self, t: float, depth_frame, rgb, pose) -> dict:
        """One registered frame pair taken at t with the robot at pose (x, y, yaw odom): detect, update, the state."""
        c = self.detect(depth_frame, rgb)
        xy = None
        if c is not None:
            co, si = math.cos(pose[2]), math.sin(pose[2])
            xy = (pose[0] + co * c[0] - si * c[1], pose[1] + si * c[0] + co * c[1])
        self.update(t, xy)
        return self.state(t)
