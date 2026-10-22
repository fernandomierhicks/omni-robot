"""The ball tracker (design.md 6, 20-22): a drag-aware ballistic Kalman filter whose measurements are blobs in pixels.

Moved from the sim (perception.BallTracker, vision.ImageTracker, vision.sphere_image) into the robot's code for v11
(design.md 30). Unchanged except: the cameras' poses now come from the MCU's odometry and the BNO085's attitude (the
caller passes them in), a track only starts on a ball moving faster than min_init_speed (a ball held in a hand or
lying on the floor isn't a throw), and TrackView carries a track across a process boundary (ROS) and propagates it.

Cameras are (c, Rc, f, cx, cy, R_ball): position and rotation (columns = the camera's axes in the world; it looks
along -z, MuJoCo's convention), focal length and principal point in pixels, the ball radius the filter assumes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from omni_catch.flight import BallFlight

CHI2_999 = {1: 10.8, 2: 13.8, 3: 16.3, 4: 18.5, 5: 20.5, 6: 22.5}


@dataclass
class TrackerConfig:
    """What the filter is told about its cameras (vision.VisionConfig's filter half; values: camera_estimate.EXPECTED)."""
    mode: str = "fused"                # "fused" (stereo + size), "stereo", "mono"
    accel_sigma: float = 0.5           # m/s^2 process noise (spin, drag error)
    sigma_c: tuple = (None, 0.4)       # centroid noise c0 + c1 / sqrt(r_px); c0 None = from the calibration below
    sigma_b: tuple = (0.05, 0.45)
    sigma_size_rel: float | None = None
    calib_rot_deg: float = 0.10        # how well the cameras are calibrated (EXPECTED)
    calib_f_rel: float = 0.002
    ball_size_rel: float = 0.015
    ego_sigma: float = 0.001           # m per frame, camera-pose noise
    min_frames: int = 6
    gate: bool = True
    honest_init: bool = True
    min_init_speed: float = 1.5        # m/s: v11, a track starts only on a moving ball
    max_gap: float = 0.5               # s without a detection: the track is dropped
    acq_still_px: float = 1.0          # init_candidates: a blob within this of where a point of the last frame
                                       # reprojects is a still object (est: centroid jitter of a still blob ~0.3 px)
    acq_still_ego: float = 1.0         # ... plus this times the image motion the cameras' own motion causes between
                                       # the frames (a still blob moves by about that much in the raw image, and
                                       # odometry + attitude leave ~1-2 px of it after compensation; est)


def sphere_image(s, R: float, f: float):
    """Image of a sphere (center s in the camera frame, looking along -z) on a pinhole camera.
    Returns (du, dv, b): ellipse-center offset from the principal point (px, v down) and minor
    semi-axis (px), or None if the camera is inside or the sphere is behind it."""
    d = math.sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2])
    if d <= R * 1.001 or s[2] >= 0:
        return None
    cth = -s[2] / d
    sa2 = (R / d) ** 2
    den = cth * cth - sa2
    if den <= 1e-6:
        return None
    rxy = math.hypot(s[0], s[1])
    sth = rxy / d
    rho = f * sth * cth / den
    b = f * math.sqrt(sa2) / math.sqrt(den)
    if rxy < 1e-12:
        return 0.0, 0.0, b
    return rho * s[0] / rxy, -rho * s[1] / rxy, b


@dataclass
class Blob:
    cam: int
    u: float
    v: float
    b: float | None          # minor semi-axis (px), None = no usable size
    n_px: int
    t: float                 # capture time


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
        return propagate(self.f, self.q, x, P, dt)

    def state_at(self, t: float):
        return self._propagate(self.x.copy(), self.P.copy(), t - self.t)


def propagate(flight: BallFlight, q: float, x, P, dt):
    steps = max(1, int(math.ceil(abs(dt) / 0.01)))
    h = dt / steps
    I3 = np.eye(3)
    Q = q * np.block([[abs(h) ** 3 / 3 * I3, h ** 2 / 2 * I3], [h ** 2 / 2 * I3, abs(h) * I3]])
    F = np.eye(6)
    F[:3, 3:] = h * I3
    for _ in range(steps):
        p, v = x[:3], x[3:]
        a1 = flight.accel(v)
        a2 = flight.accel(v + h / 2 * a1)
        a3 = flight.accel(v + h / 2 * a2)
        a4 = flight.accel(v + h * a3)
        v_new = v + h / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        p_new = p + h / 6 * (v + 2 * (v + h / 2 * a1) + 2 * (v + h / 2 * a2) + (v + h * a3))
        s = float(np.linalg.norm(v))
        k = flight.k_lin + flight.k_quad * s
        J = -k * I3
        if s > 1e-6:
            J -= flight.k_quad * np.outer(v, v) / s
        F[3:, 3:] = I3 + h * J
        P = F @ P @ F.T + Q
        x = np.concatenate([p_new, v_new])
    return x, P


class ImageTracker(BallTracker):
    """BallTracker whose update takes blob measurements in pixels."""

    def __init__(self, flight: BallFlight, cfg: TrackerConfig, f_px: float = 571.0):
        super().__init__(flight, cfg.accel_sigma)
        self.cfg = cfg
        self.acq_box = None                # ((cx, cy), (hx, hy), z_max): where init_candidates may start a track
        self.rejected = 0
        # floor 0.15 px: per-frame camera-pose noise and edge noise the per-blob term misses
        self.c0 = cfg.sigma_c[0] if cfg.sigma_c[0] is not None else \
            math.hypot(0.15, f_px * math.radians(cfg.calib_rot_deg) * 1.2 + f_px * cfg.calib_f_rel * 0.3)
        self.miss_streak = 0
        self.size_rel = cfg.sigma_size_rel if cfg.sigma_size_rel is not None else \
            1.2 * math.hypot(cfg.ball_size_rel, cfg.calib_f_rel) + 0.002

    def _h(self, pos, cams, use_size):
        out = []
        for (c, Rc, f, cx, cy, Rb), size in zip(cams, use_size):
            img = sphere_image(Rc.T @ (pos - c), Rb, f)
            if img is None:
                return None
            out += [cx + img[0], cy + img[1]] + ([img[2]] if size else [])
        return np.array(out)

    def _noise(self, blobs, use_size):
        cfg = self.cfg
        out = []
        for bl, size in zip(blobs, use_size):
            r = max(bl.b or math.sqrt(bl.n_px / math.pi), 1.0)
            sc = math.hypot(self.c0 + cfg.sigma_c[1] / math.sqrt(r), cfg.ego_sigma * r / 0.037)
            out += [sc * sc, sc * sc]
            if size:
                sb = cfg.sigma_b[0] + cfg.sigma_b[1] / math.sqrt(r)
                sb = math.hypot(sb, self.size_rel * bl.b)
                out.append(sb * sb)
        return np.diag(out)

    def _point(self, blobs, cams):
        """A position from one frame's blobs, or None if they don't agree."""
        rays = []
        for bl in blobs:
            c, Rc, f, cx, cy, Rb = cams[bl.cam]
            d = Rc @ np.array([(bl.u - cx) / f, -(bl.v - cy) / f, -1.0])
            d = d / np.linalg.norm(d)
            rng = Rb * math.sqrt(f * f + bl.b * bl.b) / bl.b if bl.b else None
            rays.append((c, d, rng))
        use_size = self.cfg.mode != "stereo"
        if len(rays) >= 2 and self.cfg.mode in ("fused", "stereo"):
            (c1, d1, r1), (c2, d2, r2) = rays[:2]
            w0 = c1 - c2
            a, bb, cc, dd, ee = d1 @ d1, d1 @ d2, d2 @ d2, d1 @ w0, d2 @ w0
            den = a * cc - bb * bb
            if den > 1e-9:
                s1, s2 = (bb * ee - cc * dd) / den, (a * ee - bb * dd) / den
                q1, q2 = c1 + s1 * d1, c2 + s2 * d2
                if s1 > 0.1 and s2 > 0.1 and np.linalg.norm(q1 - q2) < 0.03:
                    if not use_size or r1 is None or abs(s1 - r1) < 0.25 * s1:
                        return 0.5 * (q1 + q2)
            return None
        if use_size and len(rays) == 1 and rays[0][2] is not None:
            c, d, rng = rays[0]
            return c + rng * d
        return None

    def _point_cov(self, pos, blobs, cams):
        c_all = [cams[bl.cam][0] for bl in blobs]
        c_mid = np.mean(c_all, axis=0)
        los = pos - c_mid
        rng = float(np.linalg.norm(los))
        e = los / rng
        f = cams[blobs[0].cam][2]
        r_px = max(1.0, min(bl.b or 1.0 for bl in blobs))
        s_px = self.c0 + self.cfg.sigma_c[1] / math.sqrt(r_px)
        s_lat = rng * s_px / f
        inv = 0.0
        if len(blobs) >= 2 and self.cfg.mode in ("fused", "stereo"):
            base = float(np.linalg.norm(c_all[0] - c_all[1]))
            inv += 1.0 / (rng * rng / (f * base) * math.sqrt(2) * s_px) ** 2
        if self.cfg.mode != "stereo" and blobs[0].b:
            sb = math.hypot(self.cfg.sigma_b[0] + self.cfg.sigma_b[1] / math.sqrt(r_px), self.size_rel * r_px)
            inv += 1.0 / (rng * sb / r_px) ** 2
        s_dep = 1.0 / math.sqrt(inv) if inv > 0 else rng
        s_dep = max(s_dep, s_lat)
        return s_lat ** 2 * np.eye(3) + (s_dep ** 2 - s_lat ** 2) * np.outer(e, e)

    def init_from(self, blobs, cams):
        """Start the track from two consecutive consistent frames of a moving ball."""
        pos = self._point(blobs, cams)
        t = blobs[0].t
        if pos is None:
            return False
        prev = getattr(self, "_cand", None)
        self._cand = (t, pos)
        if prev is None or not (0.0 < t - prev[0] < 0.05) or np.linalg.norm(pos - prev[1]) > 15.0 * (t - prev[0]) + 0.05:
            return False
        dt = t - prev[0]
        v0 = (pos - prev[1]) / dt
        if np.linalg.norm(v0) < self.cfg.min_init_speed:
            return False
        self.x = np.concatenate([pos, v0])
        if self.cfg.honest_init:
            Pp = self._point_cov(pos, blobs, cams)
            self.P = np.block([[Pp, Pp / dt], [Pp / dt, 2 * Pp / dt ** 2]])
        else:
            self.P = np.diag([0.05 ** 2] * 3 + [4.0] * 3)
        self.t, self.n = t, 1
        self.miss_streak = 0
        return True

    @staticmethod
    def _reproj_px(q, bl, cams) -> float:
        """Pixels between point q's image in blob bl's camera and the blob."""
        c, Rc, f, cx, cy, Rb = cams[bl.cam]
        img = sphere_image(Rc.T @ (q - c), Rb, f)
        return math.inf if img is None else math.hypot(cx + img[0] - bl.u, cy + img[1] - bl.v)

    @staticmethod
    def _ego_px(cams0, cams1, d_ref: float = 3.0) -> float:
        """Image motion (px) the cameras' own motion between two frames causes: their turn, plus their shift seen at
        d_ref m."""
        out = 0.0
        for (c0, R0, f, *_), (c1, R1, *_) in zip(cams0, cams1):
            ang = math.acos(max(-1.0, min(1.0, (float(np.trace(R0.T @ R1)) - 1.0) / 2.0)))
            out = max(out, f * (ang + float(np.linalg.norm(c1 - c0)) / d_ref))
        return out

    def _still(self, bls, prev_pts, prev_blobs, cams, tol: float) -> bool:
        """Was this object already there in the previous frame (init_candidates)?"""
        # in either image: a thrown ball moves a few pixels in both, while noise can split or merge a still object's
        # blob differently in one camera
        if any(any(math.hypot(pb.u - bl.u, pb.v - bl.v) <= tol for pb in prev_blobs[bl.cam]) for bl in bls):
            return True
        return any(all(self._reproj_px(q, bl, cams) <= tol for bl in bls) for q in prev_pts)

    @staticmethod
    def _rays(blobs, cam):
        c, Rc, f, cx, cy, _ = cam
        D = np.array([((bl.u - cx) / f, -(bl.v - cy) / f, -1.0) for bl in blobs]) @ Rc.T
        return c, D / np.linalg.norm(D, axis=1, keepdims=True)

    def init_candidates(self, t: float, cands, cams) -> list:
        """Start the track among several candidate blobs per camera (design.md 31). Returns the blobs the track
        started from, or []. cands[k]: camera k's Blobs, largest first.

          points    every left x right pair that triangulates (init_from's tests), inside the play volume if one is
                    set (acq_box: the robot's map knows where a throw can be; a bookcase or a garden can't)
          still     in the image a still object stays put to a fraction of a pixel, while its stereo depth jitters
                    by tenths of a metre at 7-11 m (two frames' points 15 cm apart are not motion). A point is still
                    if, in either camera, a blob of the previous frame lies within acq_still_px of its blob, or if
                    a point of the previous frame reprojects onto its blobs in every camera (that one also holds
                    while the robot turns). Pairs must look alike in both cameras (radius, pixel count). The
                    tolerance grows with the cameras' own motion between the frames (acq_still_ego): the
                    odometry and attitude that place the cameras are off by a pixel or two while the robot drives
          moving    a previous point 1.5-15 m/s away (as init_from) that reprojects more than acq_still_px from its
                    blob in every camera: it moved in both images (a point on the same ray of one camera, a
                    mismatched pair, is not where the ball came from)
        The biggest moving point wins."""
        L = cands[0] if cands else []
        R = cands[1] if len(cands) > 1 else []
        pts = []
        if L and R:
            # every left x right pair at once: keep those whose rays pass within 3 cm in front of both cameras,
            # then triangulate them exactly (_point: the same test plus the size-range check)
            (c1, D1), (c2, D2) = self._rays(L, cams[0]), self._rays(R, cams[1])
            w0 = c1 - c2
            b = D1 @ D2.T
            d, e = D1 @ w0, D2 @ w0
            den = np.maximum(1.0 - b * b, 1e-9)
            s1 = (b * e[None, :] - d[:, None]) / den
            s2 = (e[None, :] - b * d[:, None]) / den
            gap = np.linalg.norm(w0[None, None, :] + s1[..., None] * D1[:, None, :] - s2[..., None] * D2[None, :, :],
                                 axis=-1)
            # the same ball 19 cm apart looks the same size to both cameras: radius and pixel count
            bL = np.array([bl.b or 0.0 for bl in L])
            bR = np.array([bl.b or 0.0 for bl in R])
            nL = np.array([bl.n_px for bl in L], dtype=float)
            nR = np.array([bl.n_px for bl in R], dtype=float)
            same = ((np.abs(bL[:, None] - bR[None, :]) <= 0.3 * np.maximum(bL[:, None], bR[None, :]) + 0.5)
                    & (np.maximum(nL[:, None], nR[None, :]) <= 2.0 * np.minimum(nL[:, None], nR[None, :]) + 8))
            for i, j in zip(*np.nonzero((s1 > 0.1) & (s2 > 0.1) & (gap < 0.03) & same)):
                p = self._point([L[i], R[j]], cams)
                if p is not None:
                    pts.append((p, [L[i], R[j]]))
        else:
            for bl in L or R:
                if bl.b:
                    p = self._point([bl], cams)
                    if p is not None:
                        pts.append((p, [bl]))
        box = self.acq_box
        if box is not None:
            (cx, cy), (hx, hy), z_max = box
            pts = [(p, bls) for p, bls in pts if abs(p[0] - cx) <= hx and abs(p[1] - cy) <= hy and 0.0 < p[2] < z_max]
        prev = getattr(self, "_cand_pts", None)
        self._cand_pts = (t, pts, (L, R), cams)
        if prev is None or not prev[1] or not pts or not (0.0 < t - prev[0] < 0.05):
            return []
        dt = t - prev[0]
        prev_pts = [x[0] for x in prev[1]]
        tol = self.cfg.acq_still_px + self.cfg.acq_still_ego * self._ego_px(prev[3], cams)
        best = None
        for p, bls in pts:
            if self._still(bls, prev_pts, prev[2], cams, tol):
                continue
            src = None
            for q in prev_pts:
                step = float(np.linalg.norm(p - q))
                if step > 15.0 * dt + 0.05 or step / dt < self.cfg.min_init_speed:
                    continue
                if any(self._reproj_px(q, bl, cams) <= self.cfg.acq_still_px for bl in bls):
                    continue
                if src is None or step < src[0]:
                    src = (step, q)
            if src is None:
                continue
            size = sum(bl.n_px for bl in bls)
            if best is None or size > best[0]:
                best = (size, p, (p - src[1]) / dt, bls)
        if best is None:
            return []
        _, pos, v0, bls = best
        self.x = np.concatenate([pos, v0])
        if self.cfg.honest_init:
            Pp = self._point_cov(pos, bls, cams)
            self.P = np.block([[Pp, Pp / dt], [Pp / dt, 2 * Pp / dt ** 2]])
        else:
            self.P = np.diag([0.05 ** 2] * 3 + [4.0] * 3)
        self.t, self.n = t, 1
        self.miss_streak = 0
        return bls

    def reset(self):
        super().reset()
        self._cand = None
        self._cand_pts = None
        self.miss_streak = 0

    def update_blobs(self, t: float, blobs, cams):
        cfg = self.cfg
        if self.x is None:
            self.init_from(blobs, cams)
            return
        if t < self.t:
            return
        self.x, self.P = self._propagate(self.x, self.P, t - self.t)
        self.t = t
        sel, cams_sel, size = [], [], []
        for bl in blobs:
            use_b = bl.b is not None and cfg.mode in ("fused", "mono")
            sel.append(bl)
            cams_sel.append(cams[bl.cam])
            size.append(use_b)
        if not sel:
            return
        if cfg.gate:
            keep = []
            for k, bl in enumerate(sel):
                for use_b in ((size[k], False) if size[k] else (False,)):
                    if self._gate_one(bl, cams_sel[k], use_b):
                        keep.append((bl, cams_sel[k], use_b))
                        break
                else:
                    self.rejected += 1
            if not keep:
                self.miss_streak += 1
                if self.miss_streak >= 6:
                    self.x = None
                    self._cand = None
                return
            self.miss_streak = 0
            sel, cams_sel, size = [k[0] for k in keep], [k[1] for k in keep], [k[2] for k in keep]
        z = np.concatenate([[bl.u, bl.v] + ([bl.b] if s else []) for bl, s in zip(sel, size)])
        h0 = self._h(self.x[:3], cams_sel, size)
        if h0 is None:
            return
        H = np.zeros((len(z), 6))
        for i in range(3):
            dp = np.zeros(3)
            dp[i] = 1e-4
            hi = self._h(self.x[:3] + dp, cams_sel, size)
            if hi is None:
                return
            H[:, i] = (hi - h0) / 1e-4
        R = self._noise(sel, size)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - h0)
        self.P = (np.eye(6) - K @ H) @ self.P
        self.n += 1

    def _gate_one(self, bl, cam, use_b) -> bool:
        z = np.array([bl.u, bl.v] + ([bl.b] if use_b else []))
        h0 = self._h(self.x[:3], [cam], [use_b])
        if h0 is None:
            return False
        H = np.zeros((len(z), 3))
        for i in range(3):
            dp = np.zeros(3)
            dp[i] = 1e-4
            hi = self._h(self.x[:3] + dp, [cam], [use_b])
            if hi is None:
                return False
            H[:, i] = (hi - h0) / 1e-4
        S = H @ self.P[:3, :3] @ H.T + self._noise([bl], [use_b])
        y = z - h0
        return float(y @ np.linalg.solve(S, y)) < CHI2_999[len(z)]


class TrackView:
    """A snapshot of a track (state x at time t, covariance P, frames n), on either side of a process boundary:
    estimate() and crossing() propagate it, as VisionPerception did."""

    def __init__(self, flight: BallFlight, accel_sigma: float, t: float, x, P, n: int, ready: bool):
        self.f, self.q = flight, accel_sigma ** 2
        self.t, self.x, self.P, self.n, self.ready = t, np.asarray(x, float), np.asarray(P, float), n, ready

    def estimate(self, t: float):
        x, P = propagate(self.f, self.q, self.x.copy(), self.P.copy(), t - self.t)
        return x[:3], x[3:], P

    def crossing(self, t: float, z_plane: float):
        """(xyz, velocity, time to go from t, 2x2 xy covariance) where the ball crosses z_plane going down, or None."""
        p, v, P = self.estimate(t)
        hit = self.f.crossing_state(p, v, z_plane)
        if hit is None:
            return None
        J = np.zeros((2, 6))
        base = hit[0][:2]
        for k in range(6):
            h = 1e-3 if k < 3 else 1e-2
            dx = np.zeros(6)
            dx[k] = h
            hk = self.f.crossing_state(p + dx[:3], v + dx[3:], z_plane)
            if hk is not None:
                J[:, k] = (hk[0][:2] - base) / h
        return hit[0], hit[1], hit[2], J @ P @ J.T
