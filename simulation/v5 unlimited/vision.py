"""Detailed camera / vision model: per-pixel color-blob detection, stereo + known-size ranging.

perception.py (v3) adds Gaussian noise to the true ball position. This model goes
one level down, to the pixels each camera sees, so camera and pipeline choices can
be compared by what they do to the landing prediction:

Image formation (per camera, per frame; numpy, no rendering)
  - pinhole camera at the stereo camera's real pose, W x H pixels over its FOV;
  - the ball is rasterized on the pixel grid: a pixel is "ball colored" if the ball
    covers its center for at least half of the exposure (motion blur, from the ball's
    velocity relative to the moving camera); optional rolling shutter (each row is
    exposed at its own time);
  - segmentation noise: the color threshold at the blob's edge jitters per pixel
    (edge_sigma, in px);
  - occlusion by the robot, the thrower and the funnel net, ray-cast per pixel
    (mj_multiRay; the net is a fabric mesh that lets net_transmission of the ball
    through); pixels outside the image are lost.
Detector
  - a blob needs min_blob_px pixels; frames also drop at random (drop_prob) and the
    detector can lock onto clutter instead (false_prob);
  - "moments": centroid + minor-axis radius from the blob's second moments (motion
    blur stretches the major axis only). "circle": the same for a whole blob, but a
    blob whose edge looks partly straight (an occluder) gets a least-squares circle
    on its own edge instead, so a partly hidden ball still gives its true center and
    size;
  - a blob touching the image border is dropped (biased centroid; fitting the
    remaining arc was worse).
Errors the estimator does not know about
  - camera extrinsics after calibration (rotation, translation), focal-length scale;
  - stereo sync offset between the two exposures;
  - the real ball diameter vs the assumed 74 mm (ball-to-ball spread);
  - camera pose from odometry: a random walk growing with the distance driven since
    release, plus small per-frame noise.
Estimator (image-space EKF)
  - state: ball position and velocity, the drag + gravity model of ball_flight.py;
  - measurements in pixels: each camera's blob center (u, v) and, if known-size
    ranging is on, its minor-axis radius b. The exact projection of a sphere (an
    ellipse; its center is not the projected ball center off-axis) is the
    measurement model, so stereo, mono with size, or both are handled the same way,
    whichever cameras see the ball;
  - modes: "fused" (stereo + size; one camera alone still ranges by size), "stereo"
    (centers only, both cameras needed), "mono" (left camera only, center + size);
  - Mahalanobis gate per camera (a clutter blob or an occlusion-biased size is
    rejected), delivered after a jittered pipeline latency, stamped with the capture
    time (and the row time with a rolling shutter).

Every noise figure is an estimate (est) until the OV9782 pair is on the bench.
Interface as perception.StereoPerception: reset(), step(t), ready(), estimate(t), crossing(t, z).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import mujoco
import numpy as np

from ball_flight import BallFlight
from perception import BallTracker

CHI2_999 = {1: 10.8, 2: 13.8, 3: 16.3, 4: 18.5, 5: 20.5, 6: 22.5}


@dataclass
class VisionConfig:
    mode: str = "fused"                # "fused", "stereo", "mono"
    detector: str = "moments"          # "moments", or "circle": moments unless the blob looks partly hidden, then a
                                       # circle fit to its own edge
    width: int = 1280                  # OV9782 full resolution
    height: int = 800
    fovy_deg: float | None = None      # None: the camera's (params.camera_fovy_deg)
    fps: float = 120.0
    exposure: float = 0.001            # s (est: < 1 ms in the design)
    rolling_readout: float = 0.0       # s top to bottom row; 0 = global shutter (OV9782). e.g. 0.02 for an OV5647
    rolling_compensate: bool = True    # stamp each blob with its rows' capture time
    blur_samples: int = 5
    edge_sigma: float = 0.4            # px, per-pixel jitter of the color threshold at the blob edge (est)
    min_blob_px: int = 8
    drop_prob: float = 0.02            # missed detections not explained by size/occlusion (est)
    false_prob: float = 0.0            # chance per frame and camera that the detector picks a clutter blob
    latency_mean: float = 0.050        # s capture -> filter (est)
    latency_jitter: float = 0.015
    sync_error: float = 0.0            # s, right exposure starts this much later (hardware sync ~0; software ~1-5 ms)
    calib_rot_deg: float = 0.02        # 1-sigma per axis, camera rotation error after calibration (fixed per run, est;
                                       # 0.02 deg = 0.2 px at 1280x800 / 70 deg, a good checkerboard calibration)
    calib_trans: float = 0.0005        # m, camera position error after calibration
    calib_f_rel: float = 0.002         # focal length error (scale)
    ball_size_rel: float = 0.01        # 1-sigma: real diameter vs the assumed one (fixed per run)
    ego_sigma: float = 0.001           # m per frame, camera-pose noise
    odom_drift: float = 0.01           # m per m driven since release (random walk), est for wheels + IMU + flow
    yaw_drift_deg: float = 0.3         # deg per m driven
    net_transmission: float = 0.5      # fraction of the ball visible through the funnel net (est)
    # filter
    accel_sigma: float = 0.5           # m/s^2 process noise (spin, drag error), as perception.py
    # assumed measurement noise, c0 + c1 / sqrt(r_px) px. The detector itself gives ~0.28 / sqrt(r) per axis (center)
    # and ~0.32 / sqrt(r) (radius) at 0.4 px edge noise (vision_sweep.py part 1); c0 None = the calibration's
    # pointing error, f x calib_rot (the filter is told how well its cameras are calibrated)
    sigma_c: tuple = (None, 0.4)
    sigma_b: tuple = (0.05, 0.45)
    sigma_size_rel: float | None = None     # size-ranging bias the filter allows for; None = ball spread + f error
    min_frames: int = 6
    gate: bool = True


# ---------------------------------------------------------------------------
# camera geometry

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


def _rotvec(w):
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(w) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


@dataclass
class Blob:
    cam: int
    u: float
    v: float
    b: float | None          # minor semi-axis (px), None = no usable size
    n_px: int
    t: float                 # capture-time stamp the estimator uses


class ImageTracker(BallTracker):
    """BallTracker whose update takes blob measurements in pixels."""

    def __init__(self, flight: BallFlight, cfg: VisionConfig, f_px: float = 571.0):
        super().__init__(flight, cfg.accel_sigma)
        self.cfg = cfg
        self.rejected = 0
        # floor 0.15 px: per-frame camera-pose noise and edge noise the per-blob term misses (with a 0.05 floor and
        # perfect calibration the filter got overconfident: 4.2 cm landing error vs 2.4)
        self.c0 = cfg.sigma_c[0] if cfg.sigma_c[0] is not None else \
            math.hypot(0.15, f_px * math.radians(cfg.calib_rot_deg) * 1.2 + f_px * cfg.calib_f_rel * 0.3)
        self.miss_streak = 0
        self.size_rel = cfg.sigma_size_rel if cfg.sigma_size_rel is not None else \
            1.2 * math.hypot(cfg.ball_size_rel, cfg.calib_f_rel) + 0.002

    def _h(self, pos, cams, use_size):
        """Predicted measurement vector for the given cameras: (u, v[, b]) each."""
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
            # + the camera-pose noise seen from this range (f x ego / range = ego x b / R): 0.6 px at 1 m
            sc = math.hypot(self.c0 + cfg.sigma_c[1] / math.sqrt(r), cfg.ego_sigma * r / 0.037)
            out += [sc * sc, sc * sc]
            if size:
                sb = cfg.sigma_b[0] + cfg.sigma_b[1] / math.sqrt(r)
                sb = math.hypot(sb, self.size_rel * bl.b)
                out.append(sb * sb)
        return np.diag(out)

    def _point(self, blobs, cams):
        """A position from one frame's blobs, or None if they don't agree: stereo rays must pass within
        3 cm of each other (and, with size ranging, at a range within 25% of the size's); a single blob
        is ranged by its size."""
        rays = []
        for bl in blobs:
            c, Rc, f, cx, cy, Rb = cams[bl.cam]
            d = Rc @ np.array([(bl.u - cx) / f, -(bl.v - cy) / f, -1.0])
            d = d / np.linalg.norm(d)
            rng = Rb * math.sqrt(f * f + bl.b * bl.b) / bl.b if bl.b else None     # ~ R / sin(alpha), on-axis
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

    def init_from(self, blobs, cams):
        """Start the track from two consecutive consistent frames (one frame can be a clutter blob)."""
        pos = self._point(blobs, cams)
        t = blobs[0].t
        if pos is None:
            return False
        prev = getattr(self, "_cand", None)
        self._cand = (t, pos)
        if prev is None or not (0.0 < t - prev[0] < 0.05) or np.linalg.norm(pos - prev[1]) > 15.0 * (t - prev[0]) + 0.05:
            return False
        v0 = (pos - prev[1]) / (t - prev[0])
        self.x = np.concatenate([pos, v0])
        self.P = np.diag([0.05 ** 2] * 3 + [4.0] * 3)
        self.t, self.n = t, 1
        self.miss_streak = 0
        return True

    def reset(self):
        super().reset()
        self._cand = None
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
        # per-camera gate: drop a camera's size, then the camera, if it disagrees with the prediction
        if cfg.gate:
            keep = []
            for k, bl in enumerate(sel):
                for use_b in ((size[k], False) if size[k] else (False,)):
                    ok = self._gate_one(bl, cams_sel[k], use_b)
                    if ok:
                        keep.append((bl, cams_sel[k], use_b))
                        break
                else:
                    self.rejected += 1
            if not keep:
                # a run of rejected frames means the track itself is wrong (started on clutter): start over
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


# ---------------------------------------------------------------------------

class VisionPerception:
    """Steps alongside the physics; same outputs as perception.StereoPerception."""

    def __init__(self, model, data, flight: BallFlight, p, cfg: VisionConfig, rng: np.random.Generator):
        self.m, self.d, self.cfg, self.rng, self.p = model, data, cfg, rng, p
        self.cams = [model.camera(f"stereo_{s}").id for s in ("left", "right")]
        self.ball_body = model.body("ball").id
        self.base_body = model.body("base").id
        self.R_ball = p.ball_spec.radius
        fovy = math.radians(cfg.fovy_deg if cfg.fovy_deg is not None else p.camera_fovy_deg)
        self.W, self.H = cfg.width, cfg.height
        self.f = (self.H / 2) / math.tan(fovy / 2)
        self.cx, self.cy = self.W / 2, self.H / 2
        self.g_opaque = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)
        self.g_net = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)
        # fixed per run: calibration errors and the real ball size
        sd = math.radians(cfg.calib_rot_deg)
        self.cal_R = [_rotvec(rng.normal(0, sd, 3)) for _ in self.cams]
        self.cal_t = [rng.normal(0, cfg.calib_trans, 3) for _ in self.cams]
        self.cal_f = [1 + rng.normal(0, cfg.calib_f_rel) for _ in self.cams]
        self.R_assumed = self.R_ball * (1 + rng.normal(0, cfg.ball_size_rel))
        self.tracker = ImageTracker(flight, cfg, self.f)
        self.flight = flight
        self.stats = []                       # per detection: (cam, n_px, center error px, size error px, visible frac)
        self.keep_stats = False
        self.reset()

    def reset(self):
        self.tracker.reset()
        self.tracker.rejected = 0
        self.queue = deque()
        self.t_next = 0.0
        self.frames = self.detections = 0
        self.mono_frames = self.stereo_frames = 0
        self.e_xy = np.zeros(2)
        self.e_yaw = 0.0
        self.last_base = None

    # -- the robot's belief of where its cameras are --------------------------
    def _odometry(self):
        base = self.d.xpos[self.base_body][:2].copy()
        if self.last_base is not None:
            ds = float(np.linalg.norm(base - self.last_base))
            if ds > 0:
                self.e_xy += self.rng.normal(0, self.cfg.odom_drift * math.sqrt(ds * 0.05), 2)
                self.e_yaw += self.rng.normal(0, math.radians(self.cfg.yaw_drift_deg) * math.sqrt(ds * 0.05))
        self.last_base = base

    def _believed_cam(self, k):
        """(position, rotation, f, cx, cy, assumed ball radius) the estimator uses for camera k."""
        cid = self.cams[k]
        c = self.d.cam_xpos[cid].copy()
        Rc = self.d.cam_xmat[cid].reshape(3, 3).copy()
        base = self.d.xpos[self.base_body]
        cy, sy = math.cos(self.e_yaw), math.sin(self.e_yaw)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        c = base + Rz @ (c - base) + np.array([*self.e_xy, 0.0]) + self.rng.normal(0, self.cfg.ego_sigma, 3)
        Rc = Rz @ Rc @ self.cal_R[k]
        c = c + Rc @ self.cal_t[k]
        return c, Rc, self.f * self.cal_f[k], self.cx, self.cy, self.R_assumed

    # -- image formation + detector ---------------------------------------------
    def _render_blob(self, k, t_frame):
        cfg, d, m = self.cfg, self.d, self.m
        cid = self.cams[k]
        c = d.cam_xpos[cid].copy()
        Rc = d.cam_xmat[cid].reshape(3, 3)
        bpos = d.xpos[self.ball_body].copy()
        bvel = d.sensor("ball_vel").data.copy()
        w_base = d.sensor("base_angvel").data.copy()
        v_cam = d.sensor("base_linvel").data.copy() + np.cross(w_base, c - d.xpos[self.base_body])
        s0 = Rc.T @ (bpos - c)
        v_rel = Rc.T @ (bvel - v_cam) - np.cross(Rc.T @ w_base, s0)
        dt0 = cfg.sync_error if k == 1 else 0.0
        R, f = self.R_ball, self.f
        # pixel window around the ball over the exposure and readout
        span = cfg.exposure / 2 + cfg.rolling_readout / 2 + abs(dt0)
        pts = []
        for tt in (-span, span):
            img = sphere_image(s0 + v_rel * (tt + dt0), R, f)
            if img is not None:
                pts.append((self.cx + img[0], self.cy + img[1], img[2]))
        if not pts:
            return None
        r_px = max(p[2] for p in pts) * 1.3 + 3
        u0 = int(max(0, math.floor(min(p[0] for p in pts) - r_px)))
        u1 = int(min(self.W, math.ceil(max(p[0] for p in pts) + r_px)))
        v0 = int(max(0, math.floor(min(p[1] for p in pts) - r_px)))
        v1 = int(min(self.H, math.ceil(max(p[1] for p in pts) + r_px)))
        if u1 <= u0 or v1 <= v0:
            return None
        uu, vv = np.meshgrid(np.arange(u0, u1) + 0.5, np.arange(v0, v1) + 0.5)
        dirs = np.stack([(uu - self.cx) / f, -(vv - self.cy) / f, -np.ones_like(uu)], axis=-1)
        dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
        K = max(1, cfg.blur_samples)
        t_row = cfg.rolling_readout * (vv / self.H - 0.5) if cfg.rolling_readout > 0 else 0.0
        noise = self.rng.normal(0, cfg.edge_sigma, uu.shape) if cfg.edge_sigma > 0 else 0.0
        cover = np.zeros(uu.shape)
        for j in range(K):
            te = cfg.exposure * ((j + 0.5) / K - 0.5) + dt0 + t_row
            sc = s0[None, None, :] + v_rel[None, None, :] * (te[..., None] if np.ndim(te) else te)
            along = np.sum(sc * dirs, axis=-1)
            perp = np.sqrt(np.maximum(np.sum(sc * sc, axis=-1) - along ** 2, 0.0))
            edge = (perp - R) / np.maximum(along, 1e-3) * f            # px outside the ball's silhouette
            cover += (edge + noise < 0) & (along > 0)
        mask = cover >= 0.5 * K
        if mask.sum() < cfg.min_blob_px:
            return None
        # occlusion: rays from the camera through the blob pixels (subsampled for big blobs)
        iy, ix = np.nonzero(mask)
        n = len(iy)
        stride = max(1, int(math.ceil(math.sqrt(n / 300))))
        key = (iy // stride) * 100000 + (ix // stride)
        uniq, inv = np.unique(key, return_inverse=True)
        rep = np.zeros(len(uniq), dtype=int)
        rep[inv] = np.arange(n)
        ry, rx = iy[rep], ix[rep]
        dw = (Rc @ dirs[ry, rx].T).T
        t_ball = np.linalg.norm(s0) - R
        vis = np.ones(len(uniq), dtype=bool)
        for groups, transmit in ((self.g_opaque, 0.0), (self.g_net, cfg.net_transmission)):
            gid = np.full(len(uniq), -1, dtype=np.int32)
            dist = np.full(len(uniq), -1.0)
            mujoco.mj_multiRay(m, d, c, dw.reshape(-1).copy(), groups, 1, self.ball_body, gid, dist, None,
                               len(uniq), 50.0)
            hit = (gid >= 0) & (dist < t_ball)
            if transmit > 0:
                hit &= self.rng.uniform(size=len(uniq)) > transmit
            vis &= ~hit
        keep = vis[inv]
        iy, ix = iy[keep], ix[keep]
        n = len(iy)
        vis_frac = n / max(1, mask.sum())
        if n < cfg.min_blob_px:
            return None
        u = ix + u0 + 0.5
        v = iy + v0 + 0.5
        on_edge = (u < 1) | (v < 1) | (u > self.W - 1) | (v > self.H - 1)
        res = self._detect(u, v, iy, ix, on_edge)
        if res is None:
            return None
        uc, vc, b = res
        # capture time of this blob: the frame time, or its rows' time with a compensated rolling shutter
        t_stamp = t_frame
        if cfg.rolling_readout > 0 and cfg.rolling_compensate:
            t_stamp += cfg.rolling_readout * (vc / self.H - 0.5)
        if self.keep_stats:
            img = sphere_image(s0, R, f)
            if img is not None:
                ce = math.hypot(uc - self.cx - img[0], vc - self.cy - img[1])
                self.stats.append((k, n, ce, (b - img[2]) if b is not None else float("nan"), vis_frac, img[2],
                                   float(on_edge.any())))
        return Blob(k, uc, vc, b, n, t_stamp)

    def _detect(self, u, v, iy, ix, on_edge):
        """(u, v, b) of the blob, b None if no usable size; None if the blob is unusable."""
        if on_edge.any():
            # cut by the image edge: the centroid is biased, and a circle fit to the remaining arc was worse
            # still (median 3 px, 95% 11 px off in the game), so the blob is dropped
            return None
        border = False
        uc, vc = float(u.mean()), float(v.mean())
        du, dv = u - uc, v - vc
        cov = np.array([[du @ du, du @ dv], [du @ dv, dv @ dv]]) / len(u)
        lam = np.linalg.eigvalsh(cov)
        b = 2 * math.sqrt(max(lam[0] - 1 / 12, 1e-6))
        if self.cfg.detector != "circle" or len(u) < 20:
            return None if border else (uc, vc, b)      # a blob cut by the image edge has a biased centroid
        # boundary pixels (a 4-neighbor outside the blob), then an algebraic circle fit that drops
        # boundary points well inside the circle: those are edges of an occluder, not of the ball. Points
        # on the image edge are not the ball's edge either.
        occ = np.zeros((iy.max() + 3, ix.max() + 3), dtype=bool)
        occ[iy + 1, ix + 1] = True
        inner = occ[iy, ix + 1] & occ[iy + 2, ix + 1] & occ[iy + 1, ix] & occ[iy + 1, ix + 2]
        sel = ~inner & ~on_edge
        bu, bv = u[sel], v[sel]
        if len(bu) < 8:
            return None if border else (uc, vc, b)
        keep = np.ones(len(bu), dtype=bool)
        cu, cv, r = uc, vc, b
        for _ in range(3):
            A = np.column_stack([bu[keep], bv[keep], np.ones(keep.sum())])
            rhs = bu[keep] ** 2 + bv[keep] ** 2
            try:
                sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
            except np.linalg.LinAlgError:
                return None if border else (uc, vc, b)
            cu, cv = sol[0] / 2, sol[1] / 2
            r = math.sqrt(max(sol[2] + cu * cu + cv * cv, 1e-6))
            res = np.hypot(bu - cu, bv - cv) - r
            new = res > -1.5
            if new.sum() < 8 or np.array_equal(new, keep):
                break
            keep = new
        # boundary pixel centers sit about half a pixel inside the true edge
        r += 0.5
        # a whole, unhidden blob: the moments are more precise than any edge fit (they use every pixel)
        if not border and keep.mean() > 0.85:
            return uc, vc, b
        if not (1.0 < r < 400):
            return None
        if border:
            # only an arc is visible: keep the center, trust the size only if most of the edge is in view
            arc = keep.sum() / max(1.0, 2 * math.pi * r)
            return cu, cv, (r if arc > 0.6 else None)
        return cu, cv, r                               # partly occluded: the fit to the ball's own edge

    def _false_blob(self, k, t):
        return Blob(k, self.rng.uniform(0, self.W), self.rng.uniform(0, self.H), self.rng.uniform(3, 25), 50, t)

    # -- per physics step --------------------------------------------------------
    def step(self, t: float):
        cfg = self.cfg
        if t >= self.t_next:
            self.t_next = t + 1.0 / cfg.fps
            self.frames += 1
            self._odometry()
            use = (0,) if cfg.mode == "mono" else (0, 1)
            blobs = []
            for k in use:
                if self.rng.uniform() < cfg.drop_prob:
                    continue
                if cfg.false_prob > 0 and self.rng.uniform() < cfg.false_prob:
                    blobs.append(self._false_blob(k, t))
                    continue
                bl = self._render_blob(k, t)
                if bl is not None:
                    blobs.append(bl)
            if blobs:
                if len(blobs) == 2:
                    self.stereo_frames += 1
                else:
                    self.mono_frames += 1
                cams = [self._believed_cam(k) for k in range(2)]
                lat = max(0.005, self.rng.normal(cfg.latency_mean, cfg.latency_jitter))
                v_base = self.d.sensor("base_linvel").data.copy()
                self.queue.append((t + lat, t, blobs, cams, v_base))
        ready = sorted([q for q in self.queue if q[0] <= t], key=lambda q: q[1])
        if ready:
            self.queue = deque(q for q in self.queue if q[0] > t)
            for _, tc, blobs, cams, v_base in ready:
                if len(blobs) == 2 and abs(blobs[0].t - blobs[1].t) > 2e-4:
                    # rolling shutter: the two blobs were exposed at different times; stereo pairing at one
                    # time would be wrong, so each camera updates at its own time
                    if self.tracker.x is None:      # starting the track: a pair is close enough
                        self.tracker.update_blobs(float(np.mean([b.t for b in blobs])), blobs, cams)
                        continue
                    for bl in sorted(blobs, key=lambda b: b.t):
                        # and the camera pose at that row's time (odometry interpolated)
                        c, Rc, *rest = cams[bl.cam]
                        moved = list(cams)
                        moved[bl.cam] = (c + v_base * (bl.t - tc), Rc, *rest)
                        self.tracker.update_blobs(bl.t, [bl], moved)
                else:
                    self.tracker.update_blobs(float(np.mean([b.t for b in blobs])), blobs, cams)
                self.detections += 1

    # -- outputs -----------------------------------------------------------------
    def ready(self) -> bool:
        return self.tracker.x is not None and self.tracker.n >= self.cfg.min_frames

    def estimate(self, t: float):
        x, P = self.tracker.state_at(t)
        return x[:3], x[3:], P

    def crossing(self, t: float, z_plane: float):
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
