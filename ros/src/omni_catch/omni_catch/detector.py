"""Color-blob ball detector (design.md 6: pipeline step 1), on pixels (v11, design.md 30).

  1. tracking: look in a window around where the filter expects the ball (a few ms per frame at most);
  2. otherwise, or if the window is empty: threshold a decimated view of the whole frame, then measure each candidate
     at full resolution in a window around it;
  3. measure the blob with its moments, as the sim's vision model did (vision.py "moments"): the centroid, and the
     minor semi-axis from the second moments (motion blur stretches only the major axis). A blob touching the image
     border is dropped (its centroid is biased).

The color test is a hue window with minimum saturation and value, so it works on the sim's sparse frames and on
rendered or real images alike. With several candidates the one nearest the prediction wins (tracking), else the
largest.

Acquisition (no track yet): with acquire_candidates = 1 the largest blob is the answer, so a bigger ball-coloured
object that doesn't move (a yellow shirt, foliage through a window, a ball on the floor) hides the thrown ball until
the track starts. With acquire_candidates > 1 (design.md 31) candidates() hands the tracker the n largest and the
tracker starts on the one that moves (tracker.ImageTracker.init_candidates).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

try:
    from scipy.ndimage import find_objects as _find_objects, label as _label
except ImportError:                       # pragma: no cover - the Jetson image has scipy; this keeps it optional
    _label = _find_objects = None


@dataclass
class DetectorConfig:
    hue_deg: tuple = (45.0, 100.0)     # the optic-yellow ball is ~70 deg
    min_sat: float = 0.45
    min_val: float = 0.35
    min_blob_px: int = 8
    coarse_stride: int = 4
    window_px: float = 25.0            # tracking window: half size = window_scale x radius + window_px
    window_scale: float = 2.5
    max_candidates: int = 3
    acquire_candidates: int = 40       # blobs per camera handed to the tracker while it has no track; 1: the largest
                                       # only (before design.md 31: rendered house 11% caught, with 40: ~80%)
    size_gain: float = 1.0             # measured minor semi-axis x this: the threshold drops the ball's dim rim, so a
                                       # real blob reads small; calibrate on the bench (design.md 31)
    backend: str = "python"            # "native": the pixel loops in C++ (fast_detector.py), the same blobs; the robot's
                                       # catch_perception uses it, the sim's studies keep numpy unless asked


def ball_mask(rgb: np.ndarray, cfg: DetectorConfig) -> np.ndarray:
    c = rgb.astype(np.int16)
    r, g, b = c[..., 0], c[..., 1], c[..., 2]
    mx = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    chroma = mx - mn
    ok = (mx >= cfg.min_val * 255) & (chroma >= cfg.min_sat * np.maximum(mx, 1))
    # hue (deg) of each pixel, only where it matters
    hue = np.zeros(r.shape, dtype=np.float32)
    cz = np.maximum(chroma, 1).astype(np.float32)
    is_r = ok & (mx == r)
    is_g = ok & (mx == g) & ~is_r
    is_b = ok & ~is_r & ~is_g
    hue[is_r] = (60.0 * ((g - b)[is_r] / cz[is_r])) % 360.0
    hue[is_g] = 60.0 * ((b - r)[is_g] / cz[is_g] + 2.0)
    hue[is_b] = 60.0 * ((r - g)[is_b] / cz[is_b] + 4.0)
    return ok & (hue >= cfg.hue_deg[0]) & (hue <= cfg.hue_deg[1])


def label(mask: np.ndarray):
    if _label is not None:
        return _label(mask)
    # 4-connected flood fill (fallback)
    lab = np.zeros(mask.shape, dtype=np.int32)
    n = 0
    for y0, x0 in zip(*np.nonzero(mask)):
        if lab[y0, x0]:
            continue
        n += 1
        stack = [(y0, x0)]
        lab[y0, x0] = n
        while stack:
            y, x = stack.pop()
            for yy, xx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                if 0 <= yy < mask.shape[0] and 0 <= xx < mask.shape[1] and mask[yy, xx] and not lab[yy, xx]:
                    lab[yy, xx] = n
                    stack.append((yy, xx))
    return lab, n


class Detector:
    def __init__(self, cfg: DetectorConfig | None = None):
        self.cfg = cfg or DetectorConfig()
        self.n_coarse = 0

    def _measure(self, ys, xs, u0, v0, W, H):
        """(u, v, b, n) from a component's pixels (window coordinates), or None."""
        u = xs + u0 + 0.5
        v = ys + v0 + 0.5
        if len(u) < self.cfg.min_blob_px:
            return None
        if (u < 1).any() or (v < 1).any() or (u > W - 1).any() or (v > H - 1).any():
            return None                                  # cut by the image border: biased centroid
        uc, vc = float(u.mean()), float(v.mean())
        du, dv = u - uc, v - vc
        cov = np.array([[du @ du, du @ dv], [du @ dv, dv @ dv]]) / len(u)
        lam = np.linalg.eigvalsh(cov)
        b = 2 * math.sqrt(max(lam[0] - 1 / 12, 1e-6))
        if self.cfg.size_gain != 1.0:
            b *= self.cfg.size_gain
        return uc, vc, b, len(u)

    def _components(self, fr, u0, v0, u1, v1):
        u0, v0 = max(0, int(u0)), max(0, int(v0))
        u1, v1 = min(fr.width, int(math.ceil(u1))), min(fr.height, int(math.ceil(v1)))
        if u1 <= u0 or v1 <= v0:
            return []
        m = ball_mask(fr.roi(u0, v0, u1, v1), self.cfg)
        if not m.any():
            return []
        lab, n = label(m)
        out = []
        for k in range(1, n + 1):
            ys, xs = np.nonzero(lab == k)
            res = self._measure(ys, xs, u0, v0, fr.width, fr.height)
            if res is not None:
                out.append(res)
        return out

    def _coarse(self, fr) -> list:
        """The decimated view's components in label order: (k, size, x_min, x_max, y_min, y_max), coarse pixels.
        (fast_detector.FastDetector does this in C++.)"""
        cm = ball_mask(fr.coarse(self.cfg.coarse_stride), self.cfg)
        if not cm.any():
            return []
        lab, n = label(cm)
        sizes = np.bincount(lab.ravel(), minlength=n + 1)
        if _find_objects is not None:
            return [(k, int(sizes[k]), sl[1].start, sl[1].stop - 1, sl[0].start, sl[0].stop - 1)
                    for k, sl in enumerate(_find_objects(lab), start=1)]
        out = []
        for k in range(1, n + 1):
            ys, xs = np.nonzero(lab == k)
            out.append((k, int(sizes[k]), int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())))
        return out

    def _window(self, fr, comp):
        """A coarse component's window at full resolution (a stride's margin around it)."""
        s = self.cfg.coarse_stride
        _, _, x0, x1, y0, y1 = comp
        return self._components(fr, x0 * s - s - 2, y0 * s - s - 2, (x1 + 1) * s + s + 2, (y1 + 1) * s + s + 2)

    def detect(self, fr, pred=None):
        """(u, v, b, n_px) of the ball, or None. pred: (u, v, radius px) where the tracker expects it."""
        cfg = self.cfg
        if pred is not None:
            pu, pv, pr = pred
            half = cfg.window_scale * max(pr, 1.0) + cfg.window_px
            comps = self._components(fr, pu - half, pv - half, pu + half, pv + half)
            if comps:
                return min(comps, key=lambda c: (c[0] - pu) ** 2 + (c[1] - pv) ** 2)
        # whole frame, decimated
        self.n_coarse += 1
        coarse = self._coarse(fr)
        if not coarse:
            return None
        best = None
        for _, k in sorted([(c[1], c[0]) for c in coarse], reverse=True)[:cfg.max_candidates]:
            for c in self._window(fr, coarse[k - 1]):
                if best is None or c[3] > best[3]:
                    best = c
        if best is None and pred is None:
            return None
        return best

    def candidates(self, fr, n: int) -> list:
        """Up to n blobs (u, v, b, n_px), largest first, from the whole frame: the coarse view's n largest
        components, each measured at full resolution (acquisition, no prediction)."""
        cfg = self.cfg
        self.n_coarse += 1
        coarse = self._coarse(fr)
        if not coarse:
            return []
        sizes = np.array([c[1] for c in coarse])
        out = []
        for k in np.argsort(-sizes, kind="stable")[:max(n, cfg.max_candidates)]:
            for c in self._window(fr, coarse[k]):
                if all((c[0] - o[0]) ** 2 + (c[1] - o[1]) ** 2 > 4.0 for o in out):    # neighbours' windows overlap
                    out.append(c)
        out.sort(key=lambda c: -c[3])
        return out[:n]
