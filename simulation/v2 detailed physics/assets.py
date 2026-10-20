"""Procedural textures (PNG) and meshes (inline MJCF) for the v2 scene."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image

ASSET_DIR = Path(__file__).with_name("assets")


def _f(*v) -> str:
    return " ".join(f"{x:.5g}" for x in v)


# ---------------------------------------------------------------------------
# textures

def _smooth_noise(rng, h, w, scale):
    small = rng.random((max(2, h // scale), max(2, w // scale)))
    img = Image.fromarray((small * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)
    return np.asarray(img, dtype=np.float32) / 255.0


def wood_floor(path: Path, size=1024, planks=14, seed=3):
    """Light maple gym-floor strips: long low-contrast grain, subtle tone per board."""
    rng = np.random.default_rng(seed)
    h = w = size
    pw = w // planks
    base = np.array([214, 182, 136], np.float32) / 255
    img = np.zeros((h, w, 3), np.float32)
    for k in range(planks):
        x0, x1 = k * pw, (k + 1) * pw
        # grain: noise across the board, stretched along it
        across = rng.random(pw // 2 + 2).astype(np.float32)
        across = np.interp(np.linspace(0, len(across) - 1, x1 - x0), np.arange(len(across)), across)
        along = _smooth_noise(rng, h, 4, 1)[:, :1] * 0.5
        grain = 0.94 + 0.06 * np.clip(across[None, :] + along - 0.25, 0, 1)
        cuts = sorted(rng.uniform(0, h, 2).astype(int))
        for y0, y1 in ((0, cuts[0]), (cuts[0], cuts[1]), (cuts[1], h)):
            img[y0:y1, x0:x1] = base * rng.normal(1.0, 0.035) * grain[y0:y1, :, None]
            img[max(0, y1 - 1):y1, x0:x1] *= 0.82          # butt joints
        img[:, x0:x0 + 1] *= 0.8                             # board seams
    fine = 0.985 + 0.015 * rng.random((h, w)).astype(np.float32)
    img *= fine[..., None]
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(path)


def court_lines(path: Path, size=1024):
    """Transparent overlay is not supported per-material, so this is a plain mat texture."""
    img = np.full((size, size, 3), (52, 92, 140), np.uint8)
    rng = np.random.default_rng(1)
    noise = (rng.random((size, size)) * 10).astype(np.int16)
    img = np.clip(img.astype(np.int16) + noise[..., None] - 5, 0, 255).astype(np.uint8)
    Image.fromarray(img).save(path)


def netting(path: Path, size=256, cells=8, line=3):
    """Diamond knotted-net pattern on a dark background (used with a translucent material)."""
    y, x = np.mgrid[0:size, 0:size].astype(np.float32)
    period = size / cells
    a = np.abs(((x + y) % period) - period / 2)
    b = np.abs(((x - y) % period) - period / 2)
    lines = (np.minimum(a, b) > period / 2 - line).astype(np.float32)
    img = np.zeros((size, size, 3), np.float32)
    img += lines[..., None] * np.array([0.95, 0.95, 0.97])
    img += (1 - lines[..., None]) * np.array([0.12, 0.13, 0.15])
    Image.fromarray((img * 255).astype(np.uint8)).save(path)


def skybox(path: Path, w=1024, h=1536):
    """Soft studio gradient laid out as a MuJoCo 'gradient-like' cube texture strip."""
    t = np.linspace(0, 1, h, dtype=np.float32)[:, None]
    top = np.array([0.55, 0.68, 0.84])
    mid = np.array([0.86, 0.89, 0.93])
    bot = np.array([0.42, 0.44, 0.47])
    col = np.where(t < 0.5, top + (mid - top) * (t / 0.5), mid + (bot - mid) * ((t - 0.5) / 0.5))
    img = np.repeat(col[:, None, :], w, axis=1).reshape(h, w, 3)
    Image.fromarray((img * 255).astype(np.uint8)).save(path)


def ensure_textures() -> Path:
    ASSET_DIR.mkdir(exist_ok=True)
    makers = {"floor_wood.png": wood_floor, "netting.png": netting, "mat.png": court_lines}
    for name, fn in makers.items():
        p = ASSET_DIR / name
        if not p.exists():
            fn(p)
    return ASSET_DIR


# ---------------------------------------------------------------------------
# meshes (inline MJCF <mesh> elements)

def cone_shell(name: str, r0: float, z0: float, r1: float, z1: float, n: int = 48, wraps: float = 6.0) -> str:
    """Open truncated-cone surface, both faces, with texture coordinates."""
    verts, uvs, faces = [], [], []
    slant = math.hypot(r1 - r0, z1 - z0)
    cell = 2 * math.pi * 0.5 * (r0 + r1) / wraps          # keep net cells roughly square
    for k in range(n + 1):
        a = 2 * math.pi * k / n
        for r, z, v in ((r0, z0, 0.0), (r1, z1, 1.0)):
            verts += [r * math.cos(a), r * math.sin(a), z]
            uvs += [wraps * k / n, v * slant / cell]
    for k in range(n):
        i0, i1, i2, i3 = 2 * k, 2 * k + 1, 2 * k + 2, 2 * k + 3
        faces += [i0, i2, i1, i1, i2, i3]          # outward
        faces += [i0, i1, i2, i1, i3, i2]          # inward
    return (f'<mesh name="{name}" inertia="shell" vertex="{_f(*verts)}" texcoord="{_f(*uvs)}" '
            f'face="{" ".join(map(str, faces))}"/>')


def barrel(name: str, wheel_r: float, roller_r: float, half_len: float, ns: int = 9, nt: int = 14) -> str:
    """Omni-wheel roller: a barrel whose outer profile lies on the wheel circle.

    Roller axis is local x, centered at the origin; the roller sits wheel_r -
    roller_r from the wheel axis, so its profile radius is sqrt(R^2 - s^2) - d.
    """
    d = wheel_r - roller_r
    verts = []
    for i in range(ns):
        s = -half_len + 2 * half_len * i / (ns - 1)
        rho = max(0.25 * roller_r, math.sqrt(max(wheel_r ** 2 - s * s, 0.0)) - d)
        for j in range(nt):
            t = 2 * math.pi * j / nt
            verts += [s, rho * math.cos(t), rho * math.sin(t)]
    return f'<mesh name="{name}" vertex="{_f(*verts)}"/>'      # convex hull is the collision shape
