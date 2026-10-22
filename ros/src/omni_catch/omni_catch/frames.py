"""Camera frames as the ball detector sees them (design.md 30). Two kinds, one interface:

  ArrayFrame   a real image (H x W x 3, uint8 RGB): from the cameras on the Jetson, a rosbag, or MuJoCo's renderer
  SparseFrame  a synthetic image from the sim, sent as what differs from a plain background (the ball's pixels and
               any clutter blobs: a few hundred bytes instead of 3 MB per frame); the pixels are drawn on demand

The detector only asks for windows of a frame (roi) and a decimated view of all of it (coarse), as a real-time
detector at 120 fps would (search the whole frame coarsely, then measure the ball at full resolution in a window).
"""

from __future__ import annotations

import numpy as np

BALL_RGB = (209, 242, 31)          # the optic-yellow foam ball (the sim's material: 0.82 0.95 0.12)
BACKGROUND_RGB = (104, 104, 98)


class ArrayFrame:
    def __init__(self, img: np.ndarray):
        self.img = img
        self.height, self.width = img.shape[:2]

    def roi(self, u0: int, v0: int, u1: int, v1: int) -> np.ndarray:
        return self.img[v0:v1, u0:u1]

    def coarse(self, s: int) -> np.ndarray:
        return self.img[::s, ::s]


class SparseFrame:
    """idx: flat pixel indices (y * width + x) of the ball's pixels; clutter: more ball-colored pixel indices (a
    yellow toy, a shirt) the detector must not take for the ball."""

    _coarse_cache: dict = {}

    def __init__(self, width: int, height: int, idx, clutter=None, seed: int = 0, noise: int = 6):
        self.width, self.height = width, height
        self.idx = np.asarray(idx, dtype=np.int64)
        self.clutter = np.asarray(clutter if clutter is not None else [], dtype=np.int64)
        self.seed, self.noise = seed, noise

    def _paint(self, out: np.ndarray, ys: np.ndarray, xs: np.ndarray, u0: int, v0: int, s: int, rng):
        for ids in (self.idx, self.clutter):
            if len(ids) == 0:
                continue
            y, x = ids // self.width, ids % self.width
            if s > 1:
                keep = (y % s == 0) & (x % s == 0)
                y, x = y[keep] // s, x[keep] // s
            sel = (y >= v0) & (y < v0 + out.shape[0]) & (x >= u0) & (x < u0 + out.shape[1])
            y, x = y[sel] - v0, x[sel] - u0
            if len(y):
                shade = rng.uniform(0.8, 1.0, (len(y), 1)) if rng is not None else 0.9
                out[y, x] = np.clip(np.array(BALL_RGB) * shade, 0, 255).astype(np.uint8)

    def roi(self, u0: int, v0: int, u1: int, v1: int) -> np.ndarray:
        h, w = max(0, v1 - v0), max(0, u1 - u0)
        rng = np.random.default_rng(self.seed + 7919 * u0 + 104729 * v0)
        out = (np.array(BACKGROUND_RGB, dtype=np.int16) +
               rng.integers(-self.noise, self.noise + 1, (h, w, 3), dtype=np.int16)).astype(np.uint8)
        self._paint(out, None, None, u0, v0, 1, rng)
        return out

    def coarse(self, s: int) -> np.ndarray:
        key = (self.width, self.height, s)
        base = SparseFrame._coarse_cache.get(key)
        if base is None:
            base = np.empty(((self.height + s - 1) // s, (self.width + s - 1) // s, 3), dtype=np.uint8)
            base[:] = BACKGROUND_RGB
            SparseFrame._coarse_cache[key] = base
        out = base.copy()
        self._paint(out, None, None, 0, 0, s, None)
        return out

    def nbytes(self) -> int:
        return 8 * (len(self.idx) + len(self.clutter)) + 16
