"""Small things on the floor, from the stereo pair's depth (design.md 11). No ROS.

The nose pods look 35 deg down at the floor ahead; the Jetson's stereo matcher (VPI's SGM on the rectified pair, or
OpenCV's SGBM: VERIFY its rate on the Orin) gives a depth image in the left camera's optical frame. detect():
  1  every valid pixel to base_link (the camera's extrinsics from robot_arm.json, the base's roll and pitch from the IMU)
  2  the floor: a plane fitted (RANSAC) to the points within 3 cm of z = 0, so the base's tilt doesn't count as
     height; then up to two raised levels (a rug, a mat: big flat patches 4-25 mm up), each floor where it is seen
     (its cells, 2 cm grown); each point's height above the level it stands on
  3  a top-down grid (1 cm cells) of the points 6 mm - 10 cm above the floor; connected cells are a candidate
  4  a candidate touching a cell with something taller than 10 cm nearby is part of furniture, a wall or a person:
     not an object. One whose footprint is bigger than MAX_SIZE is an obstacle, not something to pick up. A thing on
     the floor has floor round it: the ring of cells 1-2 cm out must be mostly seen floor (within 3 mm of the plane)
     with little raised in it; a rug's edge (8-16 mm, broken into patches by the depth noise), a threshold or a thing
     cut off by the image's edge fails that
  5  each object: the centroid of its points, its height (95th percentile), its footprint's long and short sides and
     yaw (PCA of the top-down points), the number of points behind it
FloorMap keeps them in the odom frame over many looks: a detection within 6 cm of a known object refines it, a known
object the camera should see (in view, in range) but doesn't is dropped after misses, picked ones are kept out.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

H_MIN, H_MAX, TALL = 0.006, 0.10, 0.10       # m: an object's points, and what counts as tall
CELL = 0.01
MIN_SIZE, MAX_SIZE = 0.010, 0.15             # m: longer than MAX_SIZE is an obstacle (a marker pen, 13 cm, isn't)
RANGE = 2.0                                   # m from the camera
MIN_POINTS = 12
RING_SEEN, RING_FLOOR, RING_RAISED = 0.4, 0.5, 0.35   # the ring round a thing: this much of it seen, and of what's
                                                      # seen at least this much floor, at most this raised
RING_NEAR, RING_SEEN_FAR = 1.0, 0.15          # m: beyond this the floor is sampled cm apart along the view: a far
                                              # thing's ring is seen less (its sighting only brings the robot closer)
RUGS, RUG_POINTS, RUG_CELLS = 2, 400, 300     # up to two raised floor levels (rugs, mats): this many points over at
                                              # least this many cells (0.03 m^2)


@dataclass
class DepthFrame:
    t: float
    depth: np.ndarray            # H x W, m along the optical axis (z); 0 or nan = no match
    K: tuple                     # fx, fy, cx, cy (px)
    T_base_cam: np.ndarray       # 4 x 4: the optical frame (z forward, x right, y down) in base_link


@dataclass
class Detection:
    x: float                     # base_link (or odom, after to_odom), the footprint's centre
    y: float
    top: float                   # height of its top above the floor
    length: float                # footprint, long side
    width: float                 # short side
    yaw: float                   # of the long side
    n: int                       # points
    big: bool = False            # bigger than the gripper takes: an obstacle


def _floor_plane(P: np.ndarray, rng: np.random.Generator, iters: int = 60, tol: float = 0.004):
    """(n, d) with n . p + d = height above the plane (n up), from the points near z = 0."""
    near = P[np.abs(P[:, 2]) < 0.03]
    if len(near) < 50:
        return np.array([0.0, 0.0, 1.0]), 0.0
    n, d, _ = _fit_plane(near, rng, tol, iters)
    return n, d


def _fit_plane(near: np.ndarray, rng: np.random.Generator, tol: float, iters: int = 60):
    """RANSAC: (n, d, inliers) of the level-ish plane (within 14 deg) most of near lie on."""
    if len(near) < 3:
        return np.array([0.0, 0.0, 1.0]), 0.0, 0
    best, best_n = (np.array([0.0, 0.0, 1.0]), 0.0), -1
    for _ in range(iters):
        a, b, c = near[rng.integers(0, len(near), 3)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if n[2] < 0:
            n = -n
        if n[2] < 0.97:                           # within 14 deg of level
            continue
        d = -n @ a
        k = int(np.sum(np.abs(near @ n + d) < tol))
        if k > best_n:
            best, best_n = (n, d), k
    n, d = best
    inl = near[np.abs(near @ n + d) < tol]
    if len(inl) >= 3:                              # refine on the inliers (least squares z = ax + by + c)
        A = np.c_[inl[:, 0], inl[:, 1], np.ones(len(inl))]
        a, b, c = np.linalg.lstsq(A, inl[:, 2], rcond=None)[0]
        n = np.array([-a, -b, 1.0])
        s = np.linalg.norm(n)
        n, d = n / s, -c / s
    return n, d, int(np.sum(np.abs(near @ n + d) < tol))


def points(frame: DepthFrame, stride: int = 1) -> np.ndarray:
    z = frame.depth[::stride, ::stride]
    h, w = z.shape
    fx, fy, cx, cy = frame.K
    v, u = np.mgrid[0:h, 0:w]
    u = u * stride + (stride - 1) / 2
    v = v * stride + (stride - 1) / 2
    ok = np.isfinite(z) & (z > 0.05) & (z < RANGE)
    zc = z[ok]
    pc = np.stack([(u[ok] - cx) / fx * zc, (v[ok] - cy) / fy * zc, zc], axis=1)
    T = frame.T_base_cam
    return pc @ T[:3, :3].T + T[:3, 3]


def _label(occ: np.ndarray):
    """8-connected components of a boolean grid (labels, n)."""
    try:
        from scipy.ndimage import label
        return label(occ, structure=np.ones((3, 3), int))
    except ImportError:                            # a small flood fill
        lab = np.zeros(occ.shape, int)
        n = 0
        for i, j in zip(*np.nonzero(occ)):
            if lab[i, j]:
                continue
            n += 1
            stack = [(i, j)]
            lab[i, j] = n
            while stack:
                a, b = stack.pop()
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        x, y = a + da, b + db
                        if 0 <= x < occ.shape[0] and 0 <= y < occ.shape[1] and occ[x, y] and not lab[x, y]:
                            lab[x, y] = n
                            stack.append((x, y))
        return lab, n


def _dilate(m: np.ndarray, r: int) -> np.ndarray:
    out = m.copy()
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx or dy:
                s = np.zeros_like(m)
                xs = slice(max(0, dx), m.shape[0] + min(0, dx))
                xd = slice(max(0, -dx), m.shape[0] + min(0, -dx))
                ys = slice(max(0, dy), m.shape[1] + min(0, dy))
                yd = slice(max(0, -dy), m.shape[1] + min(0, -dy))
                s[xs, ys] = m[xd, yd]
                out |= s
    return out


def detect(frame: DepthFrame, robot_radius: float = 0.23, rng: np.random.Generator | None = None,
           stride: int = 1, why: list | None = None) -> list:
    """Objects on the floor in base_link (see the module notes). why: a list that collects each candidate turned
    down, (x, y, n points, reason)."""
    def reject(sel, reason):
        if why is not None and sel.any():
            q = obj[sel]
            why.append((round(float(q[:, 0].mean()), 3), round(float(q[:, 1].mean()), 3), int(sel.sum()), reason))
    rng = rng or np.random.default_rng(0)
    P = points(frame, stride)
    P = P[np.hypot(P[:, 0], P[:, 1]) > robot_radius]
    if len(P) < 50:
        return []
    # every point's cell on a top-down grid over the view
    x0, y0 = P[:, 0].min() - 3 * CELL, P[:, 1].min() - 3 * CELL
    cx_, cy_ = ((P[:, 0] - x0) / CELL).astype(int), ((P[:, 1] - y0) / CELL).astype(int)
    shape = (cx_.max() + 4, cy_.max() + 4)

    def grid_of(sel):
        g = np.zeros(shape, int)
        np.add.at(g, (cx_[sel], cy_[sel]), 1)
        return g
    n, d = _floor_plane(P, rng)
    h = P @ n + d
    on_floor = np.abs(h) < 0.003
    # rugs, mats: big flat patches 4-25 mm over the floor are floor too (a thing on a rug stands on the rug)
    for _ in range(RUGS):
        cand = (h > 0.004) & (h < 0.025)
        if cand.sum() < RUG_POINTS:
            break
        nb, db, k = _fit_plane(P[cand], rng, 0.003)
        if k < RUG_POINTS:
            break
        hb = P @ nb + db
        rug_cells = grid_of(cand & (np.abs(hb) < 0.003)) > 0
        if rug_cells.sum() < RUG_CELLS:              # a toy's top face isn't a rug: a rug covers ground
            break
        in_rug = _dilate(rug_cells, 2)[cx_, cy_] & (hb > -0.004)
        h = np.where(in_rug, hb, h)
        on_floor |= in_rug & (np.abs(hb) < 0.003)
    sel_obj = (h > H_MIN) & (h < H_MAX)
    obj, hob = P[sel_obj], h[sel_obj]
    if len(obj) < MIN_POINTS:
        return []
    ix, iy = cx_[sel_obj], cy_[sel_obj]
    cnt = grid_of(sel_obj)
    occ = cnt >= 2
    tall_g = _dilate(grid_of(h > TALL) > 0, 2)         # 2 cells of margin around anything tall
    floor_g = grid_of(on_floor)
    seen_g = grid_of(np.ones(len(P), bool))
    lab, nl = _label(occ)
    out = []
    pl = lab[ix, iy]
    for c in range(1, nl + 1):
        sel = pl == c
        comp = lab == c
        if sel.sum() < MIN_POINTS:
            continue
        if np.any(tall_g[comp]):
            reject(sel, "next to something tall")
            continue
        ring = _dilate(comp, 2) & ~comp
        seen = seen_g[ring] > 0                       # the floor right behind a thing is hidden from the camera
        far = math.dist(obj[sel][:, :2].mean(axis=0), frame.T_base_cam[:2, 3]) > RING_NEAR
        if seen.mean() < (RING_SEEN_FAR if far else RING_SEEN):
            reject(sel, f"ring {seen.mean():.2f} seen")         # cut off by the image's edge or the range
            continue
        fl, ra = (floor_g[ring][seen] > 0).mean(), (cnt[ring][seen] > 0).mean()
        if fl < RING_FLOOR or ra > RING_RAISED:
            reject(sel, f"ring {fl:.2f} floor, {ra:.2f} raised")
            continue
        q, hq = obj[sel], hob[sel]
        xy = q[:, :2]
        m = xy.mean(axis=0)
        C = np.cov((xy - m).T) if len(xy) > 2 else np.eye(2) * 1e-6
        w, V = np.linalg.eigh(C)
        ax = V[:, 1]
        along, across = (xy - m) @ ax, (xy - m) @ np.array([-ax[1], ax[0]])
        L = float(np.percentile(along, 97) - np.percentile(along, 3)) + CELL / 2
        W = float(np.percentile(across, 97) - np.percentile(across, 3)) + CELL / 2
        ctr = m + ax * (np.percentile(along, 97) + np.percentile(along, 3)) / 2 \
            + np.array([-ax[1], ax[0]]) * (np.percentile(across, 97) + np.percentile(across, 3)) / 2
        if max(L, W) < MIN_SIZE:
            continue
        out.append(Detection(float(ctr[0]), float(ctr[1]), float(np.percentile(hq, 95)), L, W,
                             float(math.atan2(ax[1], ax[0])), int(sel.sum()), big=max(L, W) > MAX_SIZE))
    return out


def to_frame(dets, pose) -> list:
    """Detections from base_link into the odom frame with the base at pose (x, y, yaw)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    out = []
    for o in dets:
        out.append(Detection(pose[0] + c * o.x - s * o.y, pose[1] + s * o.x + c * o.y, o.top, o.length, o.width,
                             math.remainder(o.yaw + pose[2], math.pi), o.n, o.big))
    return out


@dataclass
class Tracked:
    id: int
    det: Detection
    seen: int = 1
    missed: int = 0
    t_seen: float = 0.0
    attempts: int = 0
    status: str = "new"           # new | picked | failed | gone


@dataclass
class FloorMap:
    """Objects in the odom frame over many looks (see the module notes)."""
    merge: float = 0.06
    tracked: list = field(default_factory=list)
    _next: int = 1

    def update(self, t: float, dets_odom: list, in_view=None):
        """in_view(x, y) -> bool: whether this look should have seen a known object (for dropping it)."""
        used = set()
        for d in dets_odom:
            best, bd = None, self.merge
            for o in self.tracked:
                if o.status in ("picked", "gone"):
                    continue
                dist = math.hypot(o.det.x - d.x, o.det.y - d.y)
                if dist < bd:
                    best, bd = o, dist
            if best is None:
                if any(o.status == "picked" and math.hypot(o.det.x - d.x, o.det.y - d.y) < self.merge
                       for o in self.tracked if o.status == "picked"):
                    continue
                self.tracked.append(Tracked(self._next, d, t_seen=t))
                self._next += 1
            else:
                k = min(best.seen, 4)                 # a running mean that follows a nudged object
                b = best.det
                best.det = Detection((k * b.x + d.x) / (k + 1), (k * b.y + d.y) / (k + 1), max(b.top, d.top),
                                     (k * b.length + d.length) / (k + 1), (k * b.width + d.width) / (k + 1),
                                     d.yaw if d.n > b.n else b.yaw, max(b.n, d.n), d.big)
                best.seen += 1
                best.missed = 0
                best.t_seen = t
                used.add(best.id)
        if in_view is not None:
            for o in self.tracked:
                if o.status in ("new",) and o.id not in used and in_view(o.det.x, o.det.y):
                    o.missed += 1
                    if o.missed >= 3:
                        o.status = "gone"

    def candidates(self) -> list:
        return [o for o in self.tracked if o.status == "new" and not o.det.big]

    def replace(self, o: Tracked, det: Detection):
        o.det = det
        o.missed = 0
