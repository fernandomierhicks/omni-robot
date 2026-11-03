"""Synthetic throws with known truth: a rendered optic-yellow ball on a noisy grey scene, an arc with drag, the camera
at a given tilt. For tests (the analysis must recover what is injected) and for dry-running the recorder and the
analysis without a camera (`record_throws.py --synthetic`).

numpy only (no scipy): the Pi can run it. Frames are R G B uint8, like Picamera2's BGR888 buffers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from arcfit import Arc, Ball, Camera, integrate, sphere_px, up_in_camera, G

BALL_RGB = np.array((209, 242, 31), np.float32)         # omni_catch.frames.BALL_RGB
BACKGROUND_RGB = np.array((104, 104, 98), np.float32)


@dataclass
class SynthConfig:
    width: int = 768
    height: int = 432
    fovy_deg: float = 28.6                  # the bench camera's (est)
    fps: float = 120.0
    exposure: float = 0.004                 # s: motion blur span
    blur_samples: int = 5
    pixel_noise: float = 3.0                # grey levels, per pixel and channel
    drop_prob: float = 0.0                  # the ball missing from a frame (the detector would see nothing)
    clutter: bool = True                    # a yellow box (not the ball) in a corner
    latency: float = 0.0                    # stamps run this much later than the exposure (only the host stamp)
    centroid_jitter: float = 0.0            # px, 1-sigma: the rendered ball is displaced by this (truth stays), a known
                                            # centroid noise for the tests
    ball: Ball = field(default_factory=Ball)
    pitch_deg: float = 0.0                  # the camera looks up by this much
    roll_deg: float = 0.0
    seed: int = 1

    def camera(self) -> Camera:
        f = (self.height / 2) / math.tan(math.radians(self.fovy_deg) / 2)
        return Camera(f, self.width / 2, self.height / 2, self.width, self.height)

    def up(self) -> np.ndarray:
        return up_in_camera(math.radians(self.pitch_deg), math.radians(self.roll_deg))


def aimed_arc(cfg: SynthConfig, start, aim, flight_time: float) -> Arc:
    """An arc in the camera frame from `start` that passes through `aim` after flight_time (drag included: Newton
    on the release velocity). t0 = 0."""
    g = -G * cfg.up()
    start, aim = np.asarray(start, float), np.asarray(aim, float)
    v = (aim - start - 0.5 * g * flight_time ** 2) / flight_time
    for _ in range(6):
        p = integrate(start, v, g, np.array([0.0, flight_time]), cfg.ball)[1]
        err = aim - p
        v = v + err / flight_time
        if np.linalg.norm(err) < 1e-4:
            break
    phi = math.radians(cfg.pitch_deg)
    psi = math.radians(cfg.roll_deg)
    return Arc(0.0, start, v, phi, psi, cfg.ball, ok=True)


def render_throw(cfg: SynthConfig, arc: Arc, duration: float, pre: float = 0.0, seed: int | None = None):
    """Frames over [t0 - pre, t0 + duration]. Returns frames (N,H,W,3) uint8, t (s), truth dict(u, v, b, visible)."""
    rng = np.random.default_rng(cfg.seed if seed is None else seed)
    cam = cfg.camera()
    n = int(round((duration + pre) * cfg.fps))
    t = arc.t0 - pre + np.arange(n) / cfg.fps
    H, W = cfg.height, cfg.width
    bg = np.empty((H, W, 3), np.float32)
    yy, xx = np.mgrid[0:H, 0:W]
    bg[...] = BACKGROUND_RGB + np.stack([8 * yy / H, 6 * xx / W, 0 * xx], -1)
    if cfg.clutter:
        bg[H - 90:H - 20, 30:110] = (222, 196, 40)             # a yellow box: hue ~48, the ball's neighbour
    frames = np.empty((n, H, W, 3), np.uint8)
    tu, tv, tb, vis = (np.full(n, np.nan) for _ in range(4))
    # sub-sample times across the exposure, centred on the frame's stamp + half the exposure (mid-exposure)
    sub = (np.arange(cfg.blur_samples) + 0.5) / cfg.blur_samples * cfg.exposure
    for i, ti in enumerate(t):
        img = bg.copy()
        if ti >= arc.t0 - 1e-9:
            ts = ti + sub
            pos = arc.positions(ts)
            u, v, b = sphere_px(pos, cam, cfg.ball.radius)
            mid = arc.positions([ti + cfg.exposure / 2])
            mu, mv, mb = (a[0] for a in sphere_px(mid, cam, cfg.ball.radius))
            tu[i], tv[i], tb[i] = mu, mv, mb
            if cfg.centroid_jitter > 0:
                ju, jv = rng.normal(0, cfg.centroid_jitter, 2)
                u, v, mu, mv = u + ju, v + jv, mu + ju, mv + jv
            ok = np.isfinite(u)
            drop = rng.random() < cfg.drop_prob
            vis[i] = float(ok.any() and not drop)
            if ok.any() and not drop:
                x0 = int(max(0, np.nanmin(u) - np.nanmax(b) - 3))
                x1 = int(min(W, np.nanmax(u) + np.nanmax(b) + 4))
                y0 = int(max(0, np.nanmin(v) - np.nanmax(b) - 3))
                y1 = int(min(H, np.nanmax(v) + np.nanmax(b) + 4))
                if x1 > x0 and y1 > y0:
                    gy, gx = np.mgrid[y0:y1, x0:x1]
                    cov = np.zeros(gy.shape, np.float32)
                    for uu, vv, bb in zip(u[ok], v[ok], b[ok]):
                        d = np.hypot(gx + 0.5 - uu, gy + 0.5 - vv)
                        cov += np.clip(bb - d + 0.5, 0.0, 1.0)
                    cov /= len(u)
                    r = np.clip(1 - np.hypot(gx + 0.5 - mu, gy + 0.5 - mv) / max(mb, 1e-3), 0, 1)
                    shade = (0.72 + 0.28 * np.sqrt(r))[..., None]
                    col = BALL_RGB * shade
                    reg = img[y0:y1, x0:x1]
                    reg[...] = reg * (1 - cov[..., None]) + col * cov[..., None]
        if cfg.pixel_noise > 0:
            img += rng.normal(0, cfg.pixel_noise, img.shape).astype(np.float32)
        frames[i] = np.clip(img, 0, 255).astype(np.uint8)
    return frames, t, dict(u=tu, v=tv, b=tb, visible=vis)


def default_throw(cfg: SynthConfig, distance: float = 4.5, seed: int = 0):
    """A throw toward the camera from `distance` m away, arriving ~0.3 m above the lens, ~1.1 s in the air (a lob)."""
    rng = np.random.default_rng(seed)
    start = np.array([rng.uniform(-0.4, 0.4), -0.6, -distance])
    aim = np.array([rng.uniform(-0.15, 0.15), 0.25, -0.8])
    T = rng.uniform(0.95, 1.25) * distance / 4.5
    return aimed_arc(cfg, start, aim, T), T


def write_session(root, cfg: SynthConfig, n_throws: int = 6, distance: float = 4.5, name: str = "synthetic",
                  rim_radius: float = 0.165, outcomes: bool = True, detector: dict | None = None):
    """A session on disk, in the recorder's format, from synthetic throws. The basket's rim sits where throw k's arc
    crosses a plane 0.30 m above the camera (every other throw is aimed off the rim); labels y / n say which."""
    import json
    from pathlib import Path
    import session as S
    d = Path(root) / name
    d.mkdir(parents=True, exist_ok=True)
    cam = cfg.camera()
    M_up = cfg.up()
    info = dict(camera=dict(name="synthetic", width=cfg.width, height=cfg.height, fps=cfg.fps, fovy_deg=cfg.fovy_deg,
                            exposure_us=int(cfg.exposure * 1e6), shutter="global"),
                detector=detector or {}, distance_m=distance, ball=dict(radius=cfg.ball.radius, mass=cfg.ball.mass),
                basket=dict(rim_radius=rim_radius, rim_center_world=None), notes="synthetic")
    truth = []
    base_ns = 5_000_000_000
    for k in range(n_throws):
        rng = np.random.default_rng(100 + k)
        start = np.array([rng.uniform(-0.4, 0.4), -0.6, -distance])
        inside = (k % 2 == 0)
        off = rng.uniform(0.0, 0.10) if inside else rng.uniform(0.30, 0.45)
        aim = np.array([off * math.cos(k), 0.25 + off * math.sin(k) * 0.0, -0.8])
        T = rng.uniform(0.95, 1.25) * distance / 4.5
        arc = aimed_arc(cfg, start, aim, T)
        frames, t, tr = render_throw(cfg, arc, T * 1.02, pre=0.15, seed=1000 + k)
        stamps = np.column_stack([base_ns + (t * 1e9).astype(np.int64), base_ns + (t * 1e9).astype(np.int64) + 12_000_000])
        meta = dict(trigger=int(0.15 * cfg.fps), outcome="y" if inside else "n", exposure_us=int(cfg.exposure * 1e6))
        S.save_throw(d, k + 1, frames, stamps, meta)
        truth.append(dict(k=k + 1, inside=inside, aim=aim.tolist(), T=T, arc=dict(p0=arc.p0.tolist(), v0=arc.v0.tolist())))
    # the rim: the aim point's plane. Aim points are at z=-0.8 (camera frame) and y=0.25 above the lens; in the world frame
    # (camera level when pitch = 0) the rim is the horizontal plane 0.25 m above the camera, centred on the lens axis.
    from analyze_throws import world_from_camera
    Mw = world_from_camera(M_up)
    info["basket"]["rim_center_world"] = (Mw @ np.array([0.0, 0.25, -0.8])).tolist()
    S.save_session(d, info)
    (d / "truth.json").write_text(json.dumps(truth, indent=1))
    return d
