"""Stereo ball perception model and drag-aware tracking filter.

Replaces the "ground truth after 0.25 s" shortcut. Every camera frame (120 fps):
  - the ball must be inside both cameras' fields of view and not hidden by the
    robot, the funnel net or the thrower (ray cast, as studies/camera_view.py);
  - a frame is dropped with some probability (missed detection);
  - the triangulated position gets stereo noise: lateral error grows with
    distance, depth error with distance squared (disparity noise / (f * B));
  - the camera pose used to put it in the world frame has a small ego-motion error;
  - it is delivered after a jittered pipeline latency (capture -> detection ->
    triangulation -> filter), stamped with its capture time.

A Kalman filter (state: position, velocity) with the same quadratic + linear drag
law as ball_flight.py fuses the detections. The landing prediction and its
uncertainty come from propagating the filter state to the plane of interest;
the covariance is mapped through a finite-difference Jacobian of that crossing.

All noise figures are estimates (est) until the OV9782 pair is on the bench.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import mujoco
import numpy as np

from ball_flight import BallFlight

H_PX, W_PX = 800, 1280                 # OV9782


@dataclass
class PerceptionConfig:
    fps: float = 120.0
    pixel_sigma: float = 0.5           # px, centroid noise of the color-blob detector (est)
    disparity_sigma: float = 0.3       # px, stereo disparity noise (est)
    latency_mean: float = 0.050        # s, capture to filter (est; requirement is < 0.1 s end to end)
    latency_jitter: float = 0.015      # s, 1 sigma
    drop_prob: float = 0.05            # missed detections (est)
    ego_sigma: float = 0.003           # m, camera-pose error from odometry/IMU over a throw (est)
    accel_sigma: float = 0.5           # m/s^2 process noise: spin (Magnus lift ~0.6 m/s^2 at 10 rad/s), drag-coefficient
                                       # error (est). 1.5 made high lobs report 27 cm 1-sigma vs ~3 cm actual error
    min_frames: int = 8                # detections before the filter reports an estimate


def visible(m, d, cam_id: int, ball_pos, ball_gid: int, fovy: float, groups) -> bool:
    """Is the ball inside this camera's field of view with nothing in the way?"""
    pos = d.cam_xpos[cam_id]
    R = d.cam_xmat[cam_id].reshape(3, 3)
    v = ball_pos - pos
    vc = R.T @ v
    if vc[2] >= 0:
        return False                   # behind the camera (it looks along -z)
    ty = math.tan(fovy / 2)
    tx = ty * W_PX / H_PX
    if abs(vc[1] / -vc[2]) > ty or abs(vc[0] / -vc[2]) > tx:
        return False
    gid = np.array([-1], dtype=np.int32)
    mujoco.mj_ray(m, d, pos, v / np.linalg.norm(v), groups, 1, -1, gid)
    return int(gid[0]) == ball_gid


class BallTracker:
    """Constant-drag ballistic Kalman filter on (x, y, z, vx, vy, vz)."""

    def __init__(self, flight: BallFlight, accel_sigma: float):
        self.f = flight
        self.q = accel_sigma ** 2
        self.reset()

    def reset(self):
        self.x = None
        self.P = None
        self.t = None
        self.n = 0

    def _jac_acc(self, v):
        s = float(np.linalg.norm(v))
        k = self.f.k_lin + self.f.k_quad * s
        J = -k * np.eye(3)
        if s > 1e-6:
            J -= self.f.k_quad * np.outer(v, v) / s
        return J

    def _propagate(self, x, P, dt):
        """RK4 on the state, first-order on the covariance, in steps of at most 10 ms."""
        steps = max(1, int(math.ceil(abs(dt) / 0.01)))
        h = dt / steps
        I3 = np.eye(3)
        Q = self.q * np.block([[h ** 3 / 3 * I3, h ** 2 / 2 * I3], [h ** 2 / 2 * I3, h * I3]])
        F = np.eye(6)
        F[:3, 3:] = h * I3
        for _ in range(steps):
            p, v = x[:3], x[3:]
            a1 = self.f.accel(v)
            a2 = self.f.accel(v + h / 2 * a1)
            a3 = self.f.accel(v + h / 2 * a2)
            a4 = self.f.accel(v + h * a3)
            v_new = v + h / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
            p_new = p + h / 6 * (v + 2 * (v + h / 2 * a1) + 2 * (v + h / 2 * a2) + (v + h * a3))
            F[3:, 3:] = I3 + h * self._jac_acc(v)
            P = F @ P @ F.T + Q
            x = np.concatenate([p_new, v_new])
        return x, P

    def update(self, t: float, z, R):
        if self.x is None:
            self.x = np.concatenate([z, np.zeros(3)])
            self.P = np.block([[R, np.zeros((3, 3))], [np.zeros((3, 3)), 25.0 * np.eye(3)]])
            self.t, self.n = t, 1
            return
        if t < self.t:
            return                     # out-of-order detection (latency jitter): skip it
        self.x, self.P = self._propagate(self.x, self.P, t - self.t)
        self.t = t
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P
        self.n += 1

    def state_at(self, t: float):
        return self._propagate(self.x.copy(), self.P.copy(), t - self.t)


class StereoPerception:
    """Steps alongside the physics; deliver estimates to the behavior code."""

    def __init__(self, model, data, flight: BallFlight, p, cfg: PerceptionConfig, rng: np.random.Generator):
        self.m, self.d, self.cfg, self.rng = model, data, cfg, rng
        self.cams = [model.camera(f"stereo_{s}").id for s in ("left", "right")]
        self.ball_gid = model.geom("ball").id
        self.groups = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
        self.fovy = math.radians(p.camera_fovy_deg)
        self.f_px = (H_PX / 2) / math.tan(self.fovy / 2)
        self.tracker = BallTracker(flight, cfg.accel_sigma)
        self.flight = flight
        self.reset()

    def reset(self):
        self.tracker.reset()
        self.queue = deque()
        self.t_next = 0.0
        self.frames = self.detections = 0

    def baseline(self) -> float:
        return float(np.linalg.norm(self.d.cam_xpos[self.cams[0]] - self.d.cam_xpos[self.cams[1]]))

    def step(self, t: float):
        cfg = self.cfg
        if t >= self.t_next:
            self.t_next = t + 1.0 / cfg.fps
            self.frames += 1
            bp = self.d.sensor("ball_pos").data.copy()
            if all(visible(self.m, self.d, c, bp, self.ball_gid, self.fovy, self.groups) for c in self.cams) \
                    and self.rng.uniform() > cfg.drop_prob:
                z, R = self._measure(bp)
                lat = max(0.005, self.rng.normal(cfg.latency_mean, cfg.latency_jitter))
                self.queue.append((t + lat, t, z, R))
        # deliver whatever has made it through the pipeline, oldest capture first
        ready = sorted([q for q in self.queue if q[0] <= t], key=lambda q: q[1])
        if ready:
            self.queue = deque(q for q in self.queue if q[0] > t)
            for _, tc, z, R in ready:
                self.tracker.update(tc, z, R)
                self.detections += 1

    def _measure(self, ball_pos):
        """Noisy triangulated position in the world frame, and its covariance."""
        cfg = self.cfg
        cam = self.cams[0]
        pos = self.d.cam_xpos[cam]
        Rc = self.d.cam_xmat[cam].reshape(3, 3)
        vc = Rc.T @ (ball_pos - pos)
        depth = -vc[2]
        s_lat = depth * cfg.pixel_sigma / self.f_px
        s_dep = depth ** 2 * cfg.disparity_sigma / (self.f_px * self.baseline())
        Rcam = np.diag([s_lat ** 2, s_lat ** 2, s_dep ** 2])
        noise_c = self.rng.normal(0.0, [s_lat, s_lat, s_dep])
        z = ball_pos + Rc @ noise_c + self.rng.normal(0.0, cfg.ego_sigma, 3)
        R = Rc @ Rcam @ Rc.T + cfg.ego_sigma ** 2 * np.eye(3)
        return z, R

    # -- outputs -----------------------------------------------------------------
    def ready(self) -> bool:
        return self.tracker.x is not None and self.tracker.n >= self.cfg.min_frames

    def estimate(self, t: float):
        """(position, velocity, covariance) propagated to time t."""
        x, P = self.tracker.state_at(t)
        return x[:3], x[3:], P

    def crossing(self, t: float, z_plane: float):
        """Predicted crossing of z_plane: (xyz, velocity, time to go, 2x2 xy covariance) or None."""
        p, v, P = self.estimate(t)
        hit = self.flight.crossing_state(p, v, z_plane)
        if hit is None:
            return None
        J = np.zeros((2, 6))
        base = hit[0][:2]
        for k in range(6):
            h = 1e-3 if k < 3 else 1e-2
            dx = np.zeros(6)
            dx[k] = h
            hk = self.flight.crossing_state(p + dx[:3], v + dx[3:], z_plane)
            if hk is not None:
                J[:, k] = (hk[0][:2] - base) / h
        return hit[0], hit[1], hit[2], J @ P @ J.T
