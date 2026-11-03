"""Synthetic RealSense-style streams of a thrown ball, with known truth: colour (RGB, 60 fps, from synth.py), depth (z16, mm) and
the IR pair (y8) at 90 fps, all on one clock, the ball's disc in each. For tests and for dry-running record_rs.py without the
camera (`record_rs.py --synthetic`). numpy only.

Simplifications (they are the synthetic world's, not the D435's): one pinhole for every stream and no extrinsics; a flat back wall
with a gradient; depth noise sigma = k z^2 with random holes; the IR images carry the ball as a bright disc on a speckled grey wall
(IR2 shifted by the disparity f B / z); the streams are sampled at different phases so the replay's alignment has something to do.
Frames follow omni_catch's camera convention inside (x right, y up, looking along -z) and are RealSense images (u right, v down).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

import synth as Y
from arcfit import Arc, Ball, sphere_px


@dataclass
class RSSynthConfig:
    width: int = 848
    height: int = 480
    fovy_deg: float = 58.0                 # the D435's depth FOV (colour is narrower: 42; one pinhole here)
    color_fps: float = 60.0
    depth_fps: float = 90.0
    exposure: float = 0.003                # colour, s
    ir_exposure: float = 0.001
    baseline: float = 0.050                # IR pair, m
    depth_noise_k: float = 0.0015          # sigma_z = k z^2 (m)
    depth_holes: float = 0.02
    pixel_noise: float = 3.0
    ir_speckle: float = 5.0
    wall_z: float = 6.5                    # back wall range, m
    depth_phase: float = 0.0037            # IR / depth sampling offset from the colour frames, s
    pitch_deg: float = 0.0
    seed: int = 1
    ball: Ball = field(default_factory=Ball)       # the thrown ball (radius sets the blobs, mass / radius the drag)

    def color_cfg(self) -> Y.SynthConfig:
        return Y.SynthConfig(width=self.width, height=self.height, fovy_deg=self.fovy_deg, fps=self.color_fps,
                             exposure=self.exposure, pixel_noise=self.pixel_noise, pitch_deg=self.pitch_deg,
                             centroid_jitter=0.25, clutter=True, seed=self.seed, ball=self.ball)

    def camera(self):
        return self.color_cfg().camera()


def _disc(shape, u, v, b):
    """(slice, coverage 0..1) of a disc of radius b at (u, v), anti-aliased."""
    H, W = shape
    x0, x1 = int(max(0, np.floor(u - b - 2))), int(min(W, np.ceil(u + b + 3)))
    y0, y1 = int(max(0, np.floor(v - b - 2))), int(min(H, np.ceil(v + b + 3)))
    if x1 <= x0 or y1 <= y0:
        return None, None, None
    gy, gx = np.mgrid[y0:y1, x0:x1]
    d = np.hypot(gx + 0.5 - u, gy + 0.5 - v)
    return (slice(y0, y1), slice(x0, x1)), np.clip(b - d + 0.5, 0, 1), d


def render_streams(cfg: RSSynthConfig, arc: Arc, T: float, pre: float = 1.0, seed: int | None = None):
    """All streams over [t0 - pre, t0 + T]: {name: (frames, stamps_s)} with stamps = mid-exposure, seconds, t0 = 0, and the
    truth {t, pos (N x 3, camera frame x right, y up, -z forward), u, v, b} on a dense 1 kHz grid."""
    rng = np.random.default_rng(cfg.seed if seed is None else seed)
    cam = cfg.camera()
    H, W = cfg.height, cfg.width
    ccfg = cfg.color_cfg()
    out = {}
    frames, t, _ = Y.render_throw(ccfg, arc, T, pre=pre, seed=int(rng.integers(1 << 30)))
    out["color"] = (frames, t + cfg.exposure / 2)

    f = cam.f
    yy, xx = np.mgrid[0:H, 0:W]
    wall = (cfg.wall_z + 0.6 * xx / W + 0.2 * yy / H).astype(np.float32)
    ir_bg = (52 + 14 * yy / H + 6 * xx / W).astype(np.float32)
    n = int(round((T + pre) * cfg.depth_fps))
    td = arc.t0 - pre + cfg.depth_phase + np.arange(n) / cfg.depth_fps
    tmid = td + cfg.ir_exposure / 2
    pos = arc.positions(np.maximum(tmid, arc.t0))                    # before the throw there is no ball
    u, v, b = sphere_px(pos, cam, ccfg.ball.radius)
    ok = (tmid >= arc.t0 - 1e-9) & np.isfinite(u)
    z = -pos[:, 2]
    depth = np.empty((n, H, W), np.uint16)
    ir1 = np.empty((n, H, W), np.uint8)
    ir2 = np.empty((n, H, W), np.uint8)
    R = ccfg.ball.radius
    for i in range(n):
        zf = wall + rng.normal(0, 1, wall.shape).astype(np.float32) * (cfg.depth_noise_k * wall ** 2)
        a1, a2 = ir_bg.copy(), ir_bg.copy()
        a1 += rng.normal(0, cfg.ir_speckle, a1.shape).astype(np.float32)
        a2 += rng.normal(0, cfg.ir_speckle, a2.shape).astype(np.float32)
        if ok[i]:
            sl, cov, d = _disc((H, W), u[i], v[i], b[i])
            if sl is not None:
                surf = z[i] - R * np.sqrt(np.clip(1 - (d / max(b[i], 1e-3)) ** 2, 0, 1))
                m = cov > 0.5
                zf[sl][m] = surf[m] + rng.normal(0, cfg.depth_noise_k * z[i] ** 2, m.sum())
                a1[sl] = a1[sl] * (1 - cov) + 195 * cov
            dsp = f * cfg.baseline / max(z[i], 0.2)
            sl2, cov2, _ = _disc((H, W), u[i] - dsp, v[i], b[i])
            if sl2 is not None:
                a2[sl2] = a2[sl2] * (1 - cov2) + 195 * cov2
        holes = rng.random(zf.shape) < cfg.depth_holes
        zf[holes] = 0
        depth[i] = np.clip(zf * 1000, 0, 65535).astype(np.uint16)
        ir1[i] = np.clip(a1, 0, 255).astype(np.uint8)
        ir2[i] = np.clip(a2, 0, 255).astype(np.uint8)
    out["depth"], out["ir1"], out["ir2"] = (depth, tmid), (ir1, tmid), (ir2, tmid)

    tt = np.arange(arc.t0, arc.t0 + T, 0.001)
    pp = arc.positions(tt)
    uu, vv, bb = sphere_px(pp, cam, R)
    return out, dict(t=tt, pos=pp, u=uu, v=vv, b=bb)
