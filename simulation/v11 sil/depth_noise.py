"""The RealSense D435's depth errors on a rendered depth image (design.md 4.11). No MuJoCo, no ROS: sil_pick.PickCameras
renders the true depth (and, for the dark-surface term, each pixel's albedo) and calls d435(); tests feed it synthetic
scenes.

A D435 is an active stereo pair: the depth comes from matching the left and right IR images (the projector's dots add
texture), so its errors are a stereo matcher's. What d435() adds to the true depth z (m along the optical axis), each
an estimate until characterization/realsense/rs_depth_plane.py has measured the owned unit:

  axial      disparity noise sigma_d px at the matching resolution, so depth noise grows as z^2 (sigma_z = z^2
             sigma_d / (f b)). Intel's typical subpixel RMS is 0.08 px and the datasheet bounds the RMS at <= 2% at 2 m
             (~0.22 px here): 0.1 px. Half of its variance is spatially smooth over ~corr_px pixels (the matcher's
             census/aggregation window), so neighbouring pixels err together and plane fits don't average it away
  scale      a fixed range scale error per camera (its calibration): N(0, scale_sd) per seed, 0.3% (est)
  grazing    surfaces seen edge-on match badly: past graze_deg between the ray and the surface's normal the noise
             grows and pixels drop out, all of them by 88 deg (a floor 2 m ahead of a camera 0.11 m up is seen at 87)
  edges      flying pixels: within edge_px of a depth step (> edge_rel of the range) a pixel takes a depth between the
             two sides with probability edge_p
  occlusion  the left imager sees background the right one doesn't, left of a nearer object's edge: no match there
             (a band as wide as the two disparities differ)
  dark       IR-dark surfaces (black plastic, dark fabric) return little projector light: with albedo under dark_albedo
             a pixel drops out with probability up to dark_p, more with range
  dropout    plus dropout of the pixels at random, and nothing nearer than z_min or past z_max; 1 mm steps (16UC1)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class D435Noise:
    disp_sigma: float = 0.10          # px, axial (est: Intel typical subpixel 0.08; <= 2% at 2 m bounds it)
    smooth_share: float = 0.5         # of the disparity variance in a smooth field
    corr_px: int = 5                  # px: that field's correlation length (est: a matcher window)
    scale_sd: float = 0.003           # range scale error per camera (est)
    graze_deg: float = 75.0           # past this the match degrades (est)
    graze_drop_deg: float = 88.0      # all dropped by this (est)
    graze_noise: float = 3.0          # disparity noise x (1 + graze_noise * s), s 0..1 over the grazing band (est)
    edge_rel: float = 0.05            # a depth step: neighbours differ by more than this share of the range
    edge_px: int = 1                  # px each side of a step
    edge_p: float = 0.5               # chance a pixel there flies (est)
    dark_albedo: float = 0.12         # albedo (0..1) below which IR returns weaken (est)
    dark_p: float = 0.8               # dropout at albedo 0, 3 m (est)
    dropout: float = 0.02             # at random
    z_min: float = 0.11               # m: the datasheet's 0.195 at 848 px, scaled to 480 (VERIFY)
    z_max: float = 10.0


IDEAL = D435Noise(smooth_share=0.0, scale_sd=0.0, graze_deg=90.0, graze_drop_deg=91.0, graze_noise=0.0,
                  edge_p=0.0, dark_p=0.0)                         # the Nov 4 model: iid disparity noise and dropout


def _smooth_field(shape, corr_px: int, rng: np.random.Generator) -> np.ndarray:
    """Unit-variance noise smooth over ~corr_px pixels (block noise upsampled bilinearly, then renormalized)."""
    h, w = shape
    k = max(1, int(corr_px))
    gh, gw = h // k + 2, w // k + 2
    g = rng.normal(size=(gh, gw))
    y = np.linspace(0, gh - 1.001, h)
    x = np.linspace(0, gw - 1.001, w)
    y0, x0 = y.astype(int), x.astype(int)
    fy, fx = (y - y0)[:, None], (x - x0)[None, :]
    f = (g[y0][:, x0] * (1 - fy) * (1 - fx) + g[y0 + 1][:, x0] * fy * (1 - fx)
         + g[y0][:, x0 + 1] * (1 - fy) * fx + g[y0 + 1][:, x0 + 1] * fy * fx)
    return f / max(f.std(), 1e-9)


def incidence_deg(z: np.ndarray, K: tuple) -> np.ndarray:
    """The angle (deg) between each pixel's ray and the surface's normal there, from the depth's neighbours (90 where
    a neighbour is missing)."""
    h, w = z.shape
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:h, 0:w]
    P = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=-1)
    du = np.zeros_like(P)
    dv = np.zeros_like(P)
    du[:, 1:-1] = P[:, 2:] - P[:, :-2]
    dv[1:-1] = P[2:] - P[:-2]
    n = np.cross(du, dv)
    nn = np.linalg.norm(n, axis=-1)
    ray = P / np.maximum(np.linalg.norm(P, axis=-1, keepdims=True), 1e-9)
    c = np.abs(np.sum(n * ray, axis=-1)) / np.maximum(nn, 1e-12)
    ang = np.degrees(np.arccos(np.clip(c, 0.0, 1.0)))
    bad = ~np.isfinite(ang) | (nn < 1e-12)
    bad[:, [0, -1]] = bad[[0, -1], :] = True
    return np.where(bad, 90.0, ang)


def d435(z_true: np.ndarray, K: tuple, f_match: float, baseline: float, rng: np.random.Generator,
         n: D435Noise = D435Noise(), albedo: np.ndarray | None = None, scale: float | None = None,
         portrait: bool = False) -> np.ndarray:
    """z_true (m, H x W, the optical axis) as the D435 would report it: float32 m, nan = no depth (module notes).
    scale: this camera's range scale error (one per camera; drawn from n.scale_sd when None). portrait: the camera
    rolled 90 deg, its baseline along the image's columns (the occlusion band then lies above or below an edge)."""
    z = np.asarray(z_true, np.float64)
    fb = f_match * baseline
    valid = np.isfinite(z) & (z > 1e-3)
    zt = np.where(valid, z, np.inf)
    d_true = np.where(valid, fb / np.maximum(zt, 1e-3), 0.0)
    ang = incidence_deg(np.where(valid, z, np.nan), K)
    s = np.clip((ang - n.graze_deg) / max(n.graze_drop_deg - n.graze_deg, 1e-6), 0.0, 1.0)
    sig = n.disp_sigma * (1.0 + n.graze_noise * s)
    iid = rng.normal(size=z.shape)
    if n.smooth_share > 0:
        noise = np.sqrt(1 - n.smooth_share) * iid + np.sqrt(n.smooth_share) * _smooth_field(z.shape, n.corr_px, rng)
    else:
        noise = iid
    d = d_true + sig * noise
    out = fb / np.maximum(d, 1e-3)
    if scale is None:
        scale = rng.normal(0.0, n.scale_sd) if n.scale_sd > 0 else 0.0
    out *= 1.0 + scale
    drop = ~valid | (d <= 0.5)
    # grazing: the chance of no match rises from 0 at graze_deg to 1 at graze_drop_deg
    drop |= rng.random(z.shape) < s ** 2
    # flying pixels at depth steps: a mix of the two sides
    if n.edge_p > 0:
        zz = np.where(valid, z, np.nan)
        for axis in (0, 1):
            a = np.roll(zz, 1, axis=axis)
            b = np.roll(zz, -1, axis=axis)
            for nb in (a, b):
                step = np.abs(nb - zz) > n.edge_rel * np.fmin(nb, zz)
                fly = step & (rng.random(z.shape) < n.edge_p / 2) & np.isfinite(nb)
                mix = rng.random(z.shape)
                out = np.where(fly, mix * zz + (1 - mix) * nb, out)
    # stereo occlusion: background left of a nearer object's left edge isn't seen by the right imager
    occl = np.zeros(z.shape, bool)
    dd = d_true.T if portrait else d_true
    occl = occl.T if portrait else occl
    for k in range(1, int(min(np.nanmax(dd) if dd.size else 0, 64)) + 1):
        right = np.zeros_like(dd)
        right[:, :-k] = dd[:, k:]
        occl |= (right - dd) >= k
    drop |= (occl.T if portrait else occl) & valid
    # IR-dark surfaces
    if albedo is not None and n.dark_p > 0:
        weak = np.clip((n.dark_albedo - albedo) / n.dark_albedo, 0.0, 1.0)
        drop |= rng.random(z.shape) < n.dark_p * weak * np.clip(zt / 3.0, 0.2, 1.0)
    drop |= rng.random(z.shape) < n.dropout
    out = np.where(drop, np.nan, out)
    out = np.where((out >= n.z_min) & (out <= n.z_max), np.round(out * 1000.0) / 1000.0, np.nan)
    return out.astype(np.float32)
