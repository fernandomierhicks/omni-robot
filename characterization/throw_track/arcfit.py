"""Fit a ballistic arc with air drag to a thrown ball's detections: the "truth" for the throw (design.md 8, phase 1C).

Input: per-frame detections of ONE camera, (t, u, v, b) = time, centroid (px), minor semi-axis (px). Output: the
3D arc in the CAMERA frame (x right, y up, the camera looks along -z: omni_catch's convention, `sphere_image`), the
direction of gravity in that frame, and the residuals. The ball's size (74 mm, from the robot config) fixes the
range, gravity's 9.81 m/s^2 fixes the scale, so one camera is enough for a whole arc even if a single frame is not.

What the residuals mean, honestly: the arc uses the same frames as the detections, so its centroid residual is the
detector's per-frame noise (plus any real deviation from drag-only flight, i.e. spin) and the fitted arc is a bound
on the tracker's error, not an absolute truth. The basket (tag) gives the independent check. See README.md.

Fitting uses scipy (the PC; imported inside fit_arc); the geometry and integrator are numpy only, so synth.py runs on the Pi.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81


@dataclass
class Ball:
    mass: float = 0.022            # kg (the robot config's ball; robot_v11.json "ball"; weigh the real one)
    radius: float = 0.037
    drag_cd: float = 0.5
    air_density: float = 1.204
    air_viscosity: float = 1.8e-5

    def k_quad(self) -> float:
        return self.air_density * math.pi * self.radius ** 2 * self.drag_cd / 2 / self.mass

    def k_lin(self) -> float:
        return 6 * math.pi * self.air_viscosity * self.radius / self.mass


@dataclass
class Camera:
    f: float                       # px
    cx: float
    cy: float
    width: int
    height: int


def up_in_camera(phi: float, psi: float) -> np.ndarray:
    """World 'up' in the camera frame for a camera pitched up by phi and rolled by psi (radians); level = (0, 1, 0)."""
    return np.array([math.sin(psi), math.cos(psi) * math.cos(phi), -math.cos(psi) * math.sin(phi)])


def angles_from_up(up) -> tuple[float, float]:
    up = np.asarray(up, float) / np.linalg.norm(up)
    psi = math.asin(float(np.clip(up[0], -1, 1)))
    phi = math.atan2(-up[2], up[1])
    return phi, psi


def integrate(p0, v0, g, ts, ball: Ball, step: float = 0.004, velocity: bool = False):
    """Positions at the times ts (sorted, ts[0] = start time of p0 / v0) under gravity vector g (m/s^2) and drag;
    with velocity=True (positions, velocities)."""
    kq, kl = ball.k_quad(), ball.k_lin()
    gx, gy, gz = g
    x, y, z = p0
    vx, vy, vz = v0
    out = np.empty((len(ts), 3))
    vout = np.empty((len(ts), 3)) if velocity else None
    t = ts[0]
    for i, tn in enumerate(ts):
        n = max(1, int(math.ceil((tn - t) / step - 1e-9))) if tn > t else 0
        h = (tn - t) / n if n else 0.0
        for _ in range(n):
            def acc(vx_, vy_, vz_):
                k = kl + kq * math.sqrt(vx_ * vx_ + vy_ * vy_ + vz_ * vz_)
                return gx - k * vx_, gy - k * vy_, gz - k * vz_
            a1 = acc(vx, vy, vz)
            v2 = (vx + h / 2 * a1[0], vy + h / 2 * a1[1], vz + h / 2 * a1[2])
            a2 = acc(*v2)
            v3 = (vx + h / 2 * a2[0], vy + h / 2 * a2[1], vz + h / 2 * a2[2])
            a3 = acc(*v3)
            v4 = (vx + h * a3[0], vy + h * a3[1], vz + h * a3[2])
            a4 = acc(*v4)
            x += h / 6 * (vx + 2 * v2[0] + 2 * v3[0] + v4[0])
            y += h / 6 * (vy + 2 * v2[1] + 2 * v3[1] + v4[1])
            z += h / 6 * (vz + 2 * v2[2] + 2 * v3[2] + v4[2])
            vx += h / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
            vy += h / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
            vz += h / 6 * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2])
        t = tn
        out[i] = (x, y, z)
        if velocity:
            vout[i] = (vx, vy, vz)
    return (out, vout) if velocity else out


def sphere_px(pos, cam: Camera, radius: float):
    """(u, v, b) of a sphere at camera-frame positions pos (N x 3): omni_catch's sphere_image, vectorised."""
    pos = np.atleast_2d(pos)
    d = np.linalg.norm(pos, axis=1)
    ok = (pos[:, 2] < 0) & (d > radius * 1.001)
    cth = np.where(ok, -pos[:, 2] / np.where(d > 0, d, 1), 0.0)
    sa2 = (radius / np.where(d > 0, d, 1)) ** 2
    den = cth * cth - sa2
    ok &= den > 1e-6
    rxy = np.hypot(pos[:, 0], pos[:, 1])
    sth = rxy / np.where(d > 0, d, 1)
    den_s = np.where(ok, den, 1.0)
    rho = cam.f * sth * cth / den_s
    b = cam.f * np.sqrt(sa2) / np.sqrt(den_s)
    rxy_s = np.where(rxy > 1e-12, rxy, 1.0)
    du = np.where(rxy > 1e-12, rho * pos[:, 0] / rxy_s, 0.0)
    dv = np.where(rxy > 1e-12, -rho * pos[:, 1] / rxy_s, 0.0)
    u, v = cam.cx + du, cam.cy + dv
    nan = np.full(len(pos), np.nan)
    return np.where(ok, u, nan), np.where(ok, v, nan), np.where(ok, b, nan)


@dataclass
class Detections:
    t: np.ndarray                  # s, sorted
    u: np.ndarray
    v: np.ndarray
    b: np.ndarray                  # minor semi-axis, px (nan = no usable size)


@dataclass
class Arc:
    t0: float
    p0: np.ndarray                 # camera frame, at t0
    v0: np.ndarray
    phi: float
    psi: float
    ball: Ball
    cost: float = 0.0
    rms_uv: float = float("nan")   # px, centroid residual of the inliers
    rms_b: float = float("nan")    # px
    rms_b_rel: float = float("nan")
    n: int = 0
    n_inliers: int = 0
    inlier: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    resid: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))   # per detection: du, dv, db
    ok: bool = False

    def g(self) -> np.ndarray:
        return -G * up_in_camera(self.phi, self.psi)

    def states(self, ts):
        """(positions, velocities) at the times ts (>= t0, any order), camera frame."""
        ts = np.asarray(ts, float)
        order = np.argsort(ts)
        tt = np.concatenate([[self.t0], ts[order]])
        pos, vel = integrate(self.p0, self.v0, self.g(), tt, self.ball, velocity=True)
        p, v = np.empty((len(ts), 3)), np.empty((len(ts), 3))
        p[order], v[order] = pos[1:], vel[1:]
        return p, v

    def positions(self, ts) -> np.ndarray:
        return self.states(ts)[0]

    def state_at(self, t: float):
        p, v = self.states([t])
        return p[0], v[0]

    def crossing_plane(self, point, normal, t_max: float = 3.0, dt: float = 0.002):
        """(position, time) where the arc next crosses the plane through `point` with unit `normal`, going against
        the normal (down onto a horizontal plane when normal = up), or None. Camera frame."""
        ts = np.arange(self.t0, self.t0 + t_max, dt)
        pos = integrate(self.p0, self.v0, self.g(), ts, self.ball)
        d = (pos - np.asarray(point)) @ np.asarray(normal)
        for i in range(1, len(ts)):
            if d[i - 1] > 0 >= d[i]:
                f = d[i - 1] / (d[i - 1] - d[i])
                return pos[i - 1] + f * (pos[i] - pos[i - 1]), ts[i - 1] + f * dt
        return None


def _residuals(x, det: Detections, cam: Camera, ball: Ball, t0: float, sig_uv: float, sig_b: float, use_b: np.ndarray,
               fixed_angles, prior=None):
    p0, v0 = x[0:3], x[3:6]
    phi, psi = fixed_angles if fixed_angles is not None else (x[6], x[7])
    g = -G * up_in_camera(phi, psi)
    pos = integrate(p0, v0, g, det.t, ball)
    u, v, b = sphere_px(pos, cam, ball.radius)
    bad = ~np.isfinite(u)
    ru = np.where(bad, 1e3, (det.u - u)) / sig_uv
    rv = np.where(bad, 1e3, (det.v - v)) / sig_uv
    rb = np.where(use_b & ~bad, (det.b - b) / sig_b, 0.0)
    out = np.column_stack([ru, rv, rb]).ravel()
    if prior is not None and fixed_angles is None:          # gravity's direction near the session's: (phi, psi, sigma)
        out = np.concatenate([out, [(phi - prior[0]) / prior[2], (psi - prior[1]) / prior[2]]])
    return out


def initial_guess(det: Detections, cam: Camera, ball: Ball, up=(0.0, 1.0, 0.0)):
    """Range from the ball's size at the ends, direction from the pixels, velocity from a straight-line fit."""
    n = len(det.t)
    k = max(3, n // 4)
    d = np.where(np.isfinite(det.b), cam.f * ball.radius / np.maximum(det.b, 0.5), np.nan)
    d_med = np.nanmedian(d) if np.isfinite(d).any() else 3.0
    rng = np.where(np.isfinite(d), d, d_med)
    # camera-frame position from the pixel and the range: ray = (du, -dv, -f) normalised
    ray = np.column_stack([det.u - cam.cx, -(det.v - cam.cy), -np.full(n, cam.f)])
    ray /= np.linalg.norm(ray, axis=1)[:, None]
    pts = ray * rng[:, None]
    # smooth the range (size is noisy): a quadratic in time
    c = np.polyfit(det.t - det.t[0], rng, 2)
    rng_s = np.polyval(c, det.t - det.t[0])
    pts = ray * np.clip(rng_s, 0.5, 30.0)[:, None]
    t = det.t - det.t[0]
    g = -G * np.asarray(up, float) / np.linalg.norm(up)
    pts_flat = pts - 0.5 * g[None, :] * t[:, None] ** 2            # remove gravity's curve: the rest is nearly a line
    A = np.column_stack([np.ones(n), t])
    coef, *_ = np.linalg.lstsq(A, pts_flat, rcond=None)
    return coef[0], coef[1]


def _stats(arc: Arc, best_x, det, full, mask, use_b, sig_uv, sig_b, phi, psi):
    n = len(det.t)
    resid = np.column_stack([full[:, 0] * sig_uv, full[:, 1] * sig_uv, full[:, 2] * sig_b])
    arc.p0, arc.v0, arc.phi, arc.psi = best_x[0:3].copy(), best_x[3:6].copy(), float(phi), float(psi)
    arc.inlier, arc.resid, arc.n_inliers = mask, resid, int(mask.sum())
    arc.rms_uv = float(math.sqrt(np.mean(resid[mask, 0] ** 2 + resid[mask, 1] ** 2) / 2)) if mask.any() else float("nan")
    ok_b = mask & use_b
    arc.rms_b = float(np.sqrt(np.mean(resid[ok_b, 2] ** 2))) if ok_b.any() else float("nan")
    b_fit = det.b - resid[:, 2]
    arc.rms_b_rel = float(np.sqrt(np.mean((resid[ok_b, 2] / np.maximum(b_fit[ok_b], 1)) ** 2))) if ok_b.any() else float("nan")


def fit_arc(det: Detections, cam: Camera, ball: Ball | None = None, sig_uv: float = 0.7, sig_b: float = 0.7,
            up=None, fit_gravity: bool = True, loss: str = "soft_l1", reject_sigma: float = 4.0,
            up_sigma: float | None = None) -> Arc:
    """Fit one throw. `up` = world up in the camera frame if known (a session fit, the tag) and the start of the
    search if not. Gravity's direction is free (2 angles) with fit_gravity True; with `up_sigma` (rad) it is free but
    pulled toward `up` (a prior from the session's other throws: one monocular arc fixes the tilt to a fraction of
    a degree only, and 0.3 deg is 4 px at 4 m); with fit_gravity False it is held at `up`. The fit is repeated once
    on the inliers."""
    from scipy.optimize import least_squares
    ball = ball or Ball()
    n = len(det.t)
    arc = Arc(float(det.t[0]), np.zeros(3), np.zeros(3), 0.0, 0.0, ball, n=n)
    if n < 8:
        return arc
    up0 = np.asarray(up if up is not None else (0.0, 1.0, 0.0), float)
    phi0, psi0 = angles_from_up(up0)
    p0, v0 = initial_guess(det, cam, ball, up0)
    use_b = np.isfinite(det.b)
    prior = (phi0, psi0, up_sigma) if (up is not None and up_sigma) else None
    free = fit_gravity or prior is not None
    x0 = np.concatenate([p0, v0, [phi0, psi0]]) if free else np.concatenate([p0, v0])
    fixed = None if free else (phi0, psi0)
    xs = np.concatenate([[1, 1, 1, 3, 3, 3], [0.2, 0.2]])[:len(x0)]
    t0 = float(det.t[0])

    def run(x0, fixed, mask):
        sub = Detections(det.t[mask], det.u[mask], det.v[mask], det.b[mask])
        return least_squares(_residuals, x0, args=(sub, cam, ball, t0, sig_uv, sig_b, use_b[mask], fixed, prior),
                             loss=loss, f_scale=2.0, x_scale=xs[:len(x0)], max_nfev=60)

    def inliers(x, fixed, mask):
        full = _residuals(x, det, cam, ball, t0, sig_uv, sig_b, use_b, fixed)[:3 * n].reshape(n, 3)
        res_uv = np.hypot(full[:, 0], full[:, 1]) * sig_uv
        mad = 1.4826 * np.median(res_uv[mask]) + 0.3
        return full, res_uv < max(reject_sigma * mad, 2.0)

    mask = np.ones(n, bool)
    best = None
    for _ in range(2):
        best = run(x0, fixed, mask)
        x0 = best.x
        full, new = inliers(best.x, fixed, mask)
        if new.sum() < 8:
            break
        mask = new
    full, _ = inliers(best.x, fixed, mask)
    phi, psi = fixed if fixed is not None else (best.x[6], best.x[7])
    arc.cost = float(best.cost)
    _stats(arc, best.x, det, full, mask, use_b, sig_uv, sig_b, phi, psi)
    arc.ok = bool(best.success and arc.n_inliers >= 8 and np.linalg.norm(arc.v0) > 0.5 and arc.rms_uv < 5.0)
    if not arc.ok and fixed is not None:                    # a bad start with gravity fixed: free it first, then fix it
        fr = fit_arc(det, cam, ball, sig_uv, sig_b, up=up, fit_gravity=True, loss=loss, reject_sigma=reject_sigma)
        if fr.ok:
            r = least_squares(_residuals, np.concatenate([fr.p0, fr.v0]), args=(det, cam, ball, t0, sig_uv, sig_b, use_b, fixed, None),
                              loss=loss, f_scale=2.0, x_scale=np.array([1, 1, 1, 3, 3, 3.0]), max_nfev=60)
            full, mask = inliers(r.x, fixed, np.ones(n, bool))
            arc.cost = float(r.cost)
            _stats(arc, r.x, det, full, mask, use_b, sig_uv, sig_b, phi0, psi0)
            arc.ok = bool(arc.n_inliers >= 8 and arc.rms_uv < 5.0)
    return arc


def session_gravity(arcs: list[Arc]) -> np.ndarray | None:
    """World up in the camera frame, from the throws that fit (the median of their free gravity directions)."""
    ups = [up_in_camera(a.phi, a.psi) for a in arcs if a.ok]
    if len(ups) < 3:
        return None
    u = np.median(np.array(ups), axis=0)
    return u / np.linalg.norm(u)


def scale_scan(dets: list[Detections], cam: Camera, ups, scales=(0.94, 0.97, 1.0, 1.03, 1.06), ball: Ball | None = None):
    """The ball's size is the weakly determined thing: refit every throw with the radius scaled and return
    (scale, total robust cost) per scale. The minimum is the session's apparent ball size / assumed (a bias in
    ball_size_rel, or a focal-length error). With `ups` fixed to the session's gravity direction."""
    ball = ball or Ball()
    out = []
    for s in scales:
        b2 = Ball(ball.mass, ball.radius * s, ball.drag_cd, ball.air_density, ball.air_viscosity)
        c = 0.0
        for det in dets:
            a = fit_arc(det, cam, b2, up=ups, fit_gravity=False)
            c += a.cost if a.ok else 0.0
        out.append((s, c))
    return out
