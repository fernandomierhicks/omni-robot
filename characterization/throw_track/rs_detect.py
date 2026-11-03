"""Ball detectors for a RealSense throw, one per imager, and the robust arc through their detections (offline: numpy + scipy).

  depth   foreground against the empty scene's depth (the first frames of the throw, or the session's setup still), ball-sized
          blobs, range from the median depth (+ 2R/3: the mean surface of a sphere sits that far in front of its centre).
  ir      brightness against the empty scene's IR image, ball-sized blobs in each imager, matched across the pair on the same
          row: the disparity gives the range (the robot's blob stereo: z = f B / d), with no depth map involved.
  colour  the robot's own colour detector (omni_catch): blobs, the thrown one picked by its arc in the image (arc2d.flight_track: the
          scene's static orange things and a carried ball are dropped), range from the depth map at its pixel (depth_at_pixel) or
          from its apparent size; the 3D arc fitted to the pixels with the ranges as a looser term (fit_ray_arc).

Every knob is in `DetectCfg`: change it and re-run `rs_analyze.py` on the same recording to compare filters. Camera frame
throughout: the RealSense optical frame of the stream's own imager (x right, y down, z forward); depth and IR1 are the same
viewpoint, IR2 is `baseline` to the side, the colour camera is offset by the factory extrinsics (`depth_to_color`).
"""
from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage as ndi

from arcfit import Ball, G, integrate


@dataclass
class DetectCfg:
    ball_radius: float = 0.037
    # depth
    zmin: float = 0.25
    zmax: float = 9.0
    depth_gap: float = 0.25            # m in front of the empty scene to count as foreground
    min_area: int = 5                  # px
    ratio_lo: float = 0.3              # blob area / the ball's model area at its range
    ratio_hi: float = 3.0
    elong_max: float = 3.0             # bounding box aspect
    # ir
    ir_thr: float = 35.0               # grey levels brighter than the empty scene
    ir_min_area: int = 4
    ir_max_r: float = 100.0            # px: bigger is not the ball (a 160 mm test ball is 68 px at 0.5 m on the 424 px IR lens)
    row_tol: float = 1.5               # px between a blob's rows in the two imagers
    d_min: float = 1.0                 # disparity range, px (z = f B / d: 1 px at f 447, B 0.05 = 22 m)
    d_max: float = 80.0
    r_tol: float = 0.5                 # relative radius difference between the pair's blobs
    # the arc
    a_lo: float = 6.0                  # |a| window of the quadratic of a ball with drag, m/s^2 (gravity 9.81; synthetic throws read +3%)
    a_hi: float = 14.0
    inlier_m: float = 0.10
    ransac_iters: int = 400
    min_inliers: int = 8               # distinct frames on the arc
    min_span_s: float = 0.15           # an arc seen for less than this is not a throw
    min_density: float = 0.35          # detected frames / frames spanned (a loose fit through scene noise has ~0.2)
    min_speed: float = 1.0             # m/s: the design's honest track start (design.md 4.1); slower is a hand or a head
    drag: bool = True                  # refine the quadratic with the drag-law arc (gravity direction fitted)
    sigma0: float = 0.004              # m: the stereo noise model sigma = sigma0 + sigma_z2 z^2 weights the fit (est)
    sigma_z2: float = 0.001
    ball: Ball = field(default_factory=Ball)
    bg_s: float = 0.25                 # the empty scene = the throw's first bg_s seconds
    seed: int = 1


@dataclass
class Intr:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float

    @classmethod
    def of(cls, d: dict) -> "Intr":
        return cls(int(d["width"]), int(d["height"]), float(d["fx"]), float(d["fy"]), float(d["ppx"]), float(d["ppy"]))

    def deproject(self, u, v, z):
        u, v, z = np.asarray(u, float), np.asarray(v, float), np.asarray(z, float)
        return np.stack([(u - self.ppx) / self.fx * z, (v - self.ppy) / self.fy * z, z], axis=-1)

    def project(self, p):
        p = np.asarray(p, float)
        return np.stack([p[..., 0] / p[..., 2] * self.fx + self.ppx, p[..., 1] / p[..., 2] * self.fy + self.ppy], axis=-1)


@dataclass
class Cand:
    u: float
    v: float
    z: float                           # m (nan: none from this imager alone)
    r: float                           # px, equivalent radius
    area: int
    score: float                       # lower = more ball-like


def background(frames, scale: float | None = None) -> np.ndarray:
    """Median of the frames (zeros = no depth are ignored for depth); the empty scene."""
    a = np.asarray(frames)
    if scale is None:
        return np.median(a.astype(np.float32), axis=0)
    z = a.astype(np.float32) * scale
    z[z <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)           # pixels that never had depth are all-NaN: they stay 0
        m = np.nanmedian(z, axis=0)
    return np.nan_to_num(m, nan=0.0).astype(np.float32)


# ---------------------------------------------------------------------------------------------------- candidates
def depth_candidates(depth: np.ndarray, scale: float, bg_m: np.ndarray, intr: Intr, cfg: DetectCfg) -> list[Cand]:
    z = depth.astype(np.float32) * scale
    fg = (z > cfg.zmin) & (z < cfg.zmax) & ((bg_m <= 0) | (z < bg_m - cfg.depth_gap))
    if not fg.any():
        return []
    lab, n = ndi.label(fg)
    if n == 0:
        return []
    sizes = np.bincount(lab.ravel(), minlength=n + 1)
    objs = ndi.find_objects(lab)
    out = []
    f, R = 0.5 * (intr.fx + intr.fy), cfg.ball_radius
    for k in np.nonzero(sizes >= cfg.min_area)[0]:
        if k == 0:
            continue
        sl = objs[k - 1]
        m = lab[sl] == k
        zs = float(np.median(z[sl][m]))
        zc = zs + 2 * R / 3
        model = math.pi * (f * R / zc) ** 2
        area = int(m.sum())
        ratio = area / max(model, 1.0)
        h, w = m.shape
        elong = max(h, w) / max(1, min(h, w))
        if not (cfg.ratio_lo <= ratio <= cfg.ratio_hi) or elong > cfg.elong_max:
            continue
        ys, xs = np.nonzero(m)
        out.append(Cand(sl[1].start + xs.mean() + 0.5, sl[0].start + ys.mean() + 0.5, zc, math.sqrt(area / math.pi), area,
                        abs(math.log(ratio)) + 0.2 * (elong - 1)))
    return out


def ir_candidates(ir: np.ndarray, bg: np.ndarray, cfg: DetectCfg) -> list[Cand]:
    d = ir.astype(np.int16) - bg.astype(np.int16)
    fg = d > cfg.ir_thr
    if not fg.any():
        return []
    lab, n = ndi.label(fg)
    sizes = np.bincount(lab.ravel(), minlength=n + 1)
    objs = ndi.find_objects(lab)
    out = []
    for k in np.nonzero(sizes >= cfg.ir_min_area)[0]:
        if k == 0:
            continue
        sl = objs[k - 1]
        m = lab[sl] == k
        h, w = m.shape
        area = int(m.sum())
        r = math.sqrt(area / math.pi)
        if r > cfg.ir_max_r or max(h, w) / max(1, min(h, w)) > cfg.elong_max:
            continue
        wgt = np.where(m, d[sl], 0).astype(np.float64)
        ys, xs = np.mgrid[sl[0], sl[1]]
        s = wgt.sum()
        circ = area / (math.pi * (max(h, w) / 2) ** 2)
        out.append(Cand((xs * wgt).sum() / s + 0.5, (ys * wgt).sum() / s + 0.5, float("nan"), r, area, abs(math.log(max(circ, 1e-3)))))
    return out


def stereo_match(c1: list[Cand], c2: list[Cand], intr: Intr, baseline: float, cfg: DetectCfg, ppx2: float | None = None) -> list[Cand]:
    """Blobs of the left imager with a partner on the same row in the right: range from the disparity."""
    f = 0.5 * (intr.fx + intr.fy)
    p2 = intr.ppx if ppx2 is None else ppx2
    out = []
    for a in c1:
        best = None
        for b in c2:
            if abs(a.v - b.v) > cfg.row_tol:
                continue
            d = (a.u - intr.ppx) - (b.u - p2)
            if not (cfg.d_min <= d <= cfg.d_max) or abs(a.r - b.r) > cfg.r_tol * max(a.r, b.r):
                continue
            key = abs(a.v - b.v) + abs(a.r - b.r)
            if best is None or key < best[0]:
                best = (key, d, b)
        if best is not None:
            d = best[1]
            out.append(Cand(a.u, a.v, f * baseline / d, a.r, a.area, a.score + 0.1 * best[0]))
    return out


# ---------------------------------------------------------------------------------------------------- the arc
@dataclass
class ArcFit:
    """A ballistic arc through 3D points, in the stream's camera frame. `a` is the constant-acceleration (quadratic) fit; when
    `ball` is set the arc is the drag-law one (arcfit.integrate, the robot's drag) with gravity `g` fitted in direction:
    a foam ball's drag is 2-3 m/s^2 at 7 m/s, which a quadratic can't follow (cm-scale bias)."""
    t_ref: float
    p0: np.ndarray
    v0: np.ndarray
    a: np.ndarray
    rms: float
    inlier: np.ndarray                 # per point
    n: int
    g: np.ndarray | None = None        # drag-law arc: the gravity vector (|g| = 9.81, direction fitted)
    ball: object | None = None
    a_quad: float = float("nan")       # |a| of the quadratic stage (a check on clocks and scale)

    def at(self, t) -> np.ndarray:
        t = np.asarray(t, float)
        tau = t - self.t_ref
        quad = self.p0 + self.v0 * tau[..., None] + 0.5 * (self.a if self.g is None else self.g) * (tau[..., None] ** 2)
        if self.g is None:
            return quad
        flat = t.ravel()
        late = flat > self.t_ref
        out = quad.reshape(-1, 3).copy()                 # before the first detection: no drag (a short extrapolation)
        if late.any():
            order = np.argsort(flat[late])
            ts = np.concatenate([[self.t_ref], flat[late][order]])
            pos = integrate(self.p0, self.v0, self.g, ts, self.ball)[1:]
            tmp = np.empty_like(pos)
            tmp[order] = pos
            out[late] = tmp
        return out.reshape(quad.shape)

    def speed(self) -> float:
        return float(np.linalg.norm(self.v0))

    def up(self) -> np.ndarray | None:
        """World up in this camera frame, from the fitted gravity (the floor's normal if the throw fell freely)."""
        return None if self.g is None else -self.g / np.linalg.norm(self.g)


def _quad(t, P):
    A = np.column_stack((np.ones_like(t), t, 0.5 * t ** 2))
    coef, *_ = np.linalg.lstsq(A, P, rcond=None)
    return coef                       # rows: p0, v0, a


def fit_arc(t: np.ndarray, P: np.ndarray, cfg: DetectCfg, a_window: bool = True) -> ArcFit | None:
    """RANSAC quadratic through the points (t in s, P in m): every candidate point of every frame goes in, so a wrong blob in
    a frame doesn't hide the ball in it. A candidate arc must have |a| near gravity (cfg.a_lo .. a_hi): that rejects the random
    chains. Refit on the inliers; the frame's detection is whichever inlier is nearest the fit."""
    t_all, P_all = np.asarray(t, float), np.asarray(P, float)
    ok = np.isfinite(P_all).all(axis=1)
    t, P = t_all[ok], P_all[ok]
    if len(t) < cfg.min_inliers:
        return None
    t0 = t.min()
    tt = t - t0
    rng = np.random.default_rng(cfg.seed)
    uniq = np.unique(np.round(tt, 6))
    best, best_n = None, 0
    for _ in range(cfg.ransac_iters):
        pick = np.sort(rng.choice(len(uniq), 3, replace=False)) if len(uniq) >= 3 else None
        if pick is None:
            break
        idx = [int(np.argmin(np.abs(tt - uniq[i]))) for i in pick]
        A = np.column_stack((np.ones(3), tt[idx], 0.5 * tt[idx] ** 2))
        try:
            coef = np.linalg.solve(A, P[idx])
        except np.linalg.LinAlgError:
            continue
        if a_window and not (cfg.a_lo <= np.linalg.norm(coef[2]) <= cfg.a_hi):
            continue
        res = np.linalg.norm(P - (coef[0] + np.outer(tt, coef[1]) + 0.5 * np.outer(tt ** 2, coef[2])), axis=1)
        n_in = len(np.unique(np.round(tt[res < cfg.inlier_m], 6)))
        if n_in > best_n:
            best, best_n = coef, n_in
    if best is None or best_n < cfg.min_inliers:
        return None
    coef, thr = best, cfg.inlier_m

    def residual(c):
        return np.linalg.norm(P - (c[0] + np.outer(tt, c[1]) + 0.5 * np.outer(tt ** 2, c[2])), axis=1)

    for _ in range(4):                          # refit on the inliers, tighten the gate to the fit's own scatter
        inl = residual(coef) < thr
        if inl.sum() < cfg.min_inliers:
            return None
        coef = _quad(tt[inl], P[inl])
        thr = float(np.clip(3.5 * 1.4826 * np.median(residual(coef)[inl]), 0.03, cfg.inlier_m))
    res = residual(coef)
    inl = res < thr
    full = np.zeros(len(ok), bool)
    full[np.nonzero(ok)[0][inl]] = True
    n_in = int(len(np.unique(np.round(tt[inl], 6))))
    span = float(tt[inl].max() - tt[inl].min())
    a_n = float(np.linalg.norm(coef[2]))
    frame_dt = float(np.median(np.diff(uniq))) if len(uniq) > 1 else 1.0
    density = n_in / (span / frame_dt + 1)                 # a ball is detected in most of the frames along its path
    # the refined arc must still look like a thrown ball: gravity-sized acceleration, long enough, fast enough, enough frames.
    # (Real scenes are full of small moving blobs that a loose fit threads together; measured on the D435's first empty-scene capture.)
    if (a_window and not (cfg.a_lo <= a_n <= cfg.a_hi)) or n_in < cfg.min_inliers or span < cfg.min_span_s or density < cfg.min_density \
            or float(np.linalg.norm(coef[1] + coef[2] * span / 2)) < cfg.min_speed:
        return None
    fit = ArcFit(float(t0), coef[0], coef[1], coef[2], float(np.sqrt(np.mean(res[inl] ** 2))), full, n_in, a_quad=a_n)
    return refine_drag(fit, t_all, P_all, cfg) if cfg.drag else fit


def refine_drag(fit: ArcFit, t: np.ndarray, P: np.ndarray, cfg: DetectCfg) -> ArcFit:
    """The drag-law arc through the quadratic's inliers: p0, v0 and gravity's direction free (|g| = 9.81), points weighted by the
    stereo noise model sigma = sigma0 + sigma_z2 z^2, soft-L1 against the odd outlier. Falls back to the quadratic if the
    optimiser fails or ends worse."""
    from scipy.optimize import least_squares
    idx = np.nonzero(fit.inlier)[0]                   # indexes into the arrays fit_arc was given
    tt, PP = np.asarray(t, float)[idx], np.asarray(P, float)[idx]
    order = np.argsort(tt)
    tt, PP = tt[order], PP[order]
    sig = cfg.sigma0 + cfg.sigma_z2 * np.maximum(PP[:, 2], 0.3) ** 2
    ball = cfg.ball
    t_ref = float(tt[0])
    tq = fit.at(np.array([t_ref]))[0]
    a0 = fit.a / max(np.linalg.norm(fit.a), 1e-9)
    # the quadratic's p0, v0 are at fit.t_ref; move them to the first inlier's time
    tau = t_ref - fit.t_ref
    v_ref = fit.v0 + fit.a * tau
    x0 = np.concatenate([tq, v_ref, [a0[0] / a0[1] if abs(a0[1]) > 1e-3 else 0.0, a0[2] / a0[1] if abs(a0[1]) > 1e-3 else 0.0]])

    def gvec(x):
        g = np.array([x[6], 1.0, x[7]])
        return G * g / np.linalg.norm(g)

    def resid(x):
        pos = integrate(x[0:3], x[3:6], gvec(x), tt, ball)
        return ((pos - PP) / sig[:, None]).ravel()

    try:
        sol = least_squares(resid, x0, loss="soft_l1", f_scale=2.0, max_nfev=60)
    except Exception:
        return fit
    pos = integrate(sol.x[0:3], sol.x[3:6], gvec(sol.x), tt, ball)
    rms = float(np.sqrt(np.mean(np.sum((pos - PP) ** 2, axis=1))))
    if not np.isfinite(rms) or rms > 1.5 * fit.rms + 0.01:
        return fit
    return ArcFit(t_ref, sol.x[0:3], sol.x[3:6], fit.a, rms, fit.inlier, fit.n, g=gvec(sol.x), ball=ball, a_quad=fit.a_quad)


# ---------------------------------------------------------------------------------------------------- colour + range (monocular arc)
def depth_in_front(u: float, v: float, r: float, depth: np.ndarray, scale: float, ic: Intr, idp: Intr, R: np.ndarray, t: np.ndarray,
                   z_guess: float, near: float = 0.6, share: float = 0.5) -> bool:
    """arc2d.depth_in_front with Intr: a colour blob that sits on a nearer, bigger object (a hand, an arm), not a free ball."""
    import arc2d
    return arc2d.depth_in_front(u, v, r, depth, scale, (ic.fx, ic.fy, ic.ppx, ic.ppy), (idp.fx, idp.fy, idp.ppx, idp.ppy), R, t, z_guess,
                                near, share)


def depth_at_pixel(u: float, v: float, r: float, depth: np.ndarray, scale: float, ic: Intr, idp: Intr, R: np.ndarray, t: np.ndarray,
                   ball_radius: float, z_guess: float) -> float:
    """The ball's range (the depth frame's z of its centre) from the depth map at a colour blob: the blob's ray at the size range z_guess
    mapped into the depth image (15 mm baseline: a few px), the median depth in a disc of 0.6 x the ball's radius there, kept within
    +-35 % of z_guess (the background behind a small blob is not the ball), + 2R/3. NaN when under 30 % of the disc has such depth."""
    pc = ic.deproject(u, v, z_guess)
    pd = R.T @ (pc - t)
    if pd[2] <= 0.1:
        return float("nan")
    ud, vd = idp.project(pd)
    rd = max(1.5, 0.6 * r * idp.fx / ic.fx)
    x0, x1 = int(max(0, ud - rd)), int(min(depth.shape[1], ud + rd + 1))
    y0, y1 = int(max(0, vd - rd)), int(min(depth.shape[0], vd + rd + 1))
    if x1 <= x0 or y1 <= y0:
        return float("nan")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    inside = (xx + 0.5 - ud) ** 2 + (yy + 0.5 - vd) ** 2 <= rd * rd
    z = depth[y0:y1, x0:x1][inside].astype(np.float32) * scale
    ok = (z > 0.65 * z_guess) & (z < 1.35 * z_guess)
    if ok.sum() < max(3, 0.3 * inside.sum()):
        return float("nan")
    return float(np.median(z[ok])) + 2 * ball_radius / 3


def fit_ray_arc(t, uv, z, sig_z, ic: Intr, R: np.ndarray, tr: np.ndarray, ball, up=None, x0=None, sig_px=None):
    """The drag-law arc (depth / IR1 frame) through colour pixels and ranges: the pixel residual (the arc projected into the colour camera
    through depth_to_color R, tr) carries the direction, the range residual (z: the depth frame's z, sigma sig_z per point) the distance,
    gravity's known size the scale. up given: gravity = -g up (6 numbers); else its direction is fitted too (8). Returns (ArcFit, x) or
    (None, None)."""
    from scipy.optimize import least_squares
    t, uv, z, sig_z = (np.asarray(a, float) for a in (t, uv, z, sig_z))
    sig_px = np.full(len(t), 1.5) if sig_px is None else np.asarray(sig_px, float)
    t_ref = float(t[0])
    if x0 is None:
        P = ic.deproject(uv[:, 0], uv[:, 1], z)                          # colour frame
        P = (P - tr) @ R                                                 # -> depth frame (R.T @ (p - t), row-wise)
        dt = max(t[-1] - t[0], 1e-3)
        g0 = -G * (np.asarray(up, float) if up is not None else np.array([0.0, -1.0, 0.0]))
        v0 = (P[-1] - P[0]) / dt - 0.5 * g0 * dt
        x0 = np.concatenate([P[0], v0] + ([] if up is not None else [[0.0, 0.0]]))

    def gvec(x):
        if up is not None:
            return -G * np.asarray(up, float) / np.linalg.norm(up)
        g = np.array([x[6], 1.0, x[7]])                                   # y down in the optical frame: mostly +y
        return G * g / np.linalg.norm(g)

    def resid(x):
        pos = integrate(x[0:3], x[3:6], gvec(x), t, ball, step=0.01)
        pc = pos @ R.T + tr
        pix = ic.project(pc)
        return np.concatenate([((pix - uv) / sig_px[:, None]).ravel(), (pos[:, 2] - z) / sig_z])
    try:
        sol = least_squares(resid, np.asarray(x0, float), loss="soft_l1", f_scale=2.0, max_nfev=60)
    except Exception:
        return None, None
    pos = integrate(sol.x[0:3], sol.x[3:6], gvec(sol.x), t, ball, step=0.01)
    pix = ic.project(pos @ R.T + tr)
    rms_px = float(np.sqrt(np.mean(np.sum((pix - uv) ** 2, axis=1))))
    g = gvec(sol.x)
    fit = ArcFit(t_ref, sol.x[0:3], sol.x[3:6], g, rms_px * float(np.median(z)) / ic.fx, np.ones(len(t), bool), len(t), g=g, ball=ball)
    fit.rms_px = rms_px
    return fit, sol.x


def extrinsics_matrix(e: dict) -> tuple[np.ndarray, np.ndarray]:
    """(R, t) of a session's extrinsics entry: librealsense's rotation is column-major."""
    return np.array(e["R"], float).reshape(3, 3, order="F"), np.array(e["t"], float)


# ---------------------------------------------------------------------------------------------------- one throw
def collect(times: np.ndarray, cand_lists: list[list[Cand]], intr: Intr, need_z: bool = True):
    """Flatten per-frame candidates to (frame index, time, 3D point, candidate) arrays."""
    fi, tt, P, C = [], [], [], []
    for i, (t, cs) in enumerate(zip(times, cand_lists)):
        for c in cs:
            if need_z and not np.isfinite(c.z):
                continue
            fi.append(i)
            tt.append(t)
            P.append(intr.deproject(c.u, c.v, c.z))
            C.append(c)
    return np.array(fi, int), np.array(tt, float), np.array(P, float).reshape(-1, 3), C


def pick_per_frame(fit: ArcFit, fi, tt, P, C, n_frames: int, max_dist: float):
    """For each frame the candidate nearest the fitted arc (within max_dist), else None."""
    best = [None] * n_frames
    dist = np.linalg.norm(P - fit.at(tt), axis=1) if len(tt) else np.zeros(0)
    for j in range(len(fi)):
        if dist[j] <= max_dist and (best[fi[j]] is None or dist[j] < best[fi[j]][0]):
            best[fi[j]] = (float(dist[j]), C[j], P[j])
    return best
