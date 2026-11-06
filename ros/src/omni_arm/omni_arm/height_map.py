"""What the depth camera adds to the lidar for navigation: a 2.5D height map of the floor fused over many frames
(design.md 4.5, 4.11). No ROS.

The lidar scans one plane 163 mm up. The D435 sees below and above it: a shoe, a box edge, a table top, a drop. A
single frame at driving speed is a poor witness: the floor 1-2 m ahead is seen edge-on, sparse and noisy, and a rug's
edge or a sill looks like a small object (the per-frame layer of Nov 5 closed doors that way; floor_obstacles.py).
Here every frame adds evidence to the map's own cells, and a cell is judged on what many frames saw:

  per frame  the points against the floor plane that frame sees (floor_vision's fit: the base's pitch drops out),
             then per map cell with >= MIN_PTS points: the median height, the 80th percentile, the lowest point
  fused      per cell a running mean of those (recent frames weigh more, capped at N_MAX; a look CHANGE away
             restarts it: a moved thing clears in a couple of frames), the evidence count, when it was last seen. A cell keeps its state while out of view (under the camera's 0.34 m blind zone, behind
             or beside the robot)
  classes    UNKNOWN   too little evidence, an obstacle not seen again for OBS_TTL (it may have gone), or a drop not
                       yet confirmed (deep under the floor but seen from far or briefly: never known floor)
             FLOOR     its 90th percentile under STEP_MIN over the local floor (BASE_R round it: relief)
             STEP      raised STEP_MIN..OBS_H over a big region (STEP_AREA): a rug, a sill, a raised floor. Drivable
             LOW       raised LOW_H..OBS_H, compact: a toy, a flat thing. Drivable over but slowed for; a catalog item
             OBSTACLE  OBS_H .. TOP (the robot's top: a table top counts) seen in >= OBS_N frames
             DROP      its median DROP_H or more under the robot's floor, seen near (DROP_RANGE) in DROP_N
                       frames over DROP_CELLS cells (stairs, a step down), and the unseen cells next to it (the
                       lower floor just past an edge is hidden from above)
  costmap    OBSTACLE and DROP only (Nav2's StaticLayer on /floor_obstacles/grid). Nothing low ever closes a door
  speed      speed_limit(): along the way the robot is going, the nearest UNKNOWN cell sets a speed it can stop
             in before reaching it (BRAKE m/s^2, never under V_MIN); a LOW thing near the path caps V_LOW; a known
             floor or step leaves the speed alone. Driving sideways or backwards into ground the forward camera
             hasn't seen is therefore slow by itself
  labels     an AI's (or a person's) word on a cell overrides the class (floor, step, low, obstacle) until the cell
             is seen to change by LABEL_CHANGE

Every threshold is an estimate (sim, Nov 5), to tune against the real D435 (characterization/realsense).
"""

from __future__ import annotations

import math

import numpy as np

from omni_arm import floor_vision as fv

UNKNOWN, FLOOR, STEP, LOW, OBSTACLE, DROP = 0, 1, 2, 3, 4, 5
NAMES = ("unknown", "floor", "step", "low", "obstacle", "drop")

RANGE = 2.5             # m from the camera (along its axis)
MIN_PTS = 3             # points a cell needs in a frame to count as seen
N_MAX = 8.0             # evidence cap: the newest ~N_MAX frames dominate (a moved thing clears in a few)
N_MIN = 1.5             # evidence to judge a cell at all
STEP_MIN = 0.004        # m: raised less than this is floor
LOW_H = 0.008           # m: a compact thing this tall (90th percentile: a 12 mm crayon reads ~10 in a 5 cm cell) is
LOW_N = 3.0             # LOW, seen in this many frames; a false one only slows the robot (floor p90 ~6 mm at 2 m)
OBS_H = 0.030           # m: an obstacle (above every level step in a house: rugs 8-16 mm, a raised tile floor ~12)
OBS_N = 2.0             # evidence an obstacle needs
DROP_H = 0.05           # m under the floor: a drop
TOP = 0.60              # m: the robot's top; higher things are out of its way
STEP_AREA = 0.10        # m^2: a raised region this big (or STEP_LEN long) is a level step, not a thing
STEP_LEN = 0.40
OBS_TTL = 120.0         # s: an obstacle not seen again goes UNKNOWN (slow, not blocked)
CHANGE = 0.008          # m: a look this far from the cell's fused height restarts its evidence (a thing moved,
                        # or a 12 mm one arrived; an obstacle still needs OBS_H in OBS_N frames)
DROP_SHADOW = 0.6       # m: unseen cells next to a drop are the drop's shadow (the lower floor just past an edge
                        # is hidden from above), marked DROP this far
DROP_N, DROP_RANGE, DROP_CELLS = 3.0, 1.5, 6   # a drop: seen in this many frames, within this range of the camera
                        # (a 1 deg error in the frame's floor plane is 4 cm at 2.5 m), in a region this many cells
BASE_R = 0.5            # m: the local floor level round a cell (seen floor cells within ~this): heights against it
                        # (relief) cancel the frame planes' small tilt errors, which grow with range
ROBOT_RADIUS = 0.24
BRAKE = 0.8             # m/s^2: the deceleration the speed limit budgets for (est; MPPI's limit is 2)
V_MIN, V_LOW, V_BLIND = 0.25, 0.4, 0.3      # m/s: creeping into the unknown, near a low thing, the camera silent
LOOK = 2.5              # m: how far along the path the speed limit looks
LABEL_CHANGE = 0.006    # m: a labelled cell whose height changes this much forgets the label
LABELS = {"floor": FLOOR, "rug": STEP, "step": STEP, "low": LOW, "toy": LOW, "obstacle": OBSTACLE, "drop": DROP}


def frame_points(frame: fv.DepthFrame, stride: int = 1, rng=None, robot_radius: float = ROBOT_RADIUS):
    """(x, y, h) per point in base_link, h over the floor plane the frame sees, or None (too little floor)."""
    z = frame.depth[::stride, ::stride]
    hh, ww = z.shape
    fx, fy, cx, cy = frame.K
    v, u = np.mgrid[0:hh, 0:ww]
    u = u * stride + (stride - 1) / 2
    v = v * stride + (stride - 1) / 2
    ok = np.isfinite(z) & (z > 0.05) & (z < RANGE)
    zc = z[ok]
    pc = np.stack([(u[ok] - cx) / fx * zc, (v[ok] - cy) / fy * zc, zc], axis=1)
    T = frame.T_base_cam
    P = pc @ T[:3, :3].T + T[:3, 3]
    P = P[np.hypot(P[:, 0], P[:, 1]) > robot_radius]
    if len(P) < 100 or np.sum(np.abs(P[:, 2]) < 0.03) < 50:
        return None                                       # no floor to measure against
    n, d = fv._floor_plane(P, rng or np.random.default_rng(0))
    return np.c_[P[:, 0], P[:, 1], P @ n + d]


class HeightMap:
    """The fused map on the map's own cells (module notes)."""

    def __init__(self):
        self.res = None
        self.origin = (0.0, 0.0)
        self.n = np.zeros((0, 0), np.float32)          # evidence
        self.med = np.zeros((0, 0), np.float32)        # fused median height
        self.p80 = np.zeros((0, 0), np.float32)
        self.low = np.zeros((0, 0), np.float32)        # fused lowest point
        self.t = np.zeros((0, 0))                      # last seen
        self.near = np.zeros((0, 0), np.float32)       # fused distance from the camera it was seen at
        self.label = np.zeros((0, 0), np.int8)         # 0 = none, else a class
        self.label_h = np.zeros((0, 0), np.float32)    # p80 when labelled
        self.t_frame = -math.inf                       # the last frame's time (camera health)

    _FIELDS = ("n", "med", "p80", "low", "t", "near", "label", "label_h")

    @property
    def shape(self):
        return self.n.shape

    def geometry(self, res: float, ox: float, oy: float, width: int, height: int) -> bool:
        """Take the map's geometry, keeping what overlaps. True if it changed."""
        if (self.res, self.origin, self.n.shape) == (res, (ox, oy), (height, width)):
            return False
        new = {k: np.zeros((height, width), getattr(self, k).dtype) for k in self._FIELDS}
        if self.res == res and self.n.size:
            di, dj = round((self.origin[1] - oy) / res), round((self.origin[0] - ox) / res)
            h0, w0 = self.n.shape
            i0, i1, j0, j1 = max(0, di), min(height, di + h0), max(0, dj), min(width, dj + w0)
            if i0 < i1 and j0 < j1:
                for k in self._FIELDS:
                    new[k][i0:i1, j0:j1] = getattr(self, k)[i0 - di:i1 - di, j0 - dj:j1 - dj]
        self.res, self.origin = res, (ox, oy)
        for k in self._FIELDS:
            setattr(self, k, new[k])
        return True

    def ij(self, xy: np.ndarray):
        """Cell indices (i row = y, j col = x) of map points, and which are on the map."""
        j = np.floor((xy[:, 0] - self.origin[0]) / self.res).astype(int)
        i = np.floor((xy[:, 1] - self.origin[1]) / self.res).astype(int)
        ok = (i >= 0) & (i < self.n.shape[0]) & (j >= 0) & (j < self.n.shape[1])
        return i, j, ok

    def xy(self, i, j):
        return self.origin[0] + (np.asarray(j) + 0.5) * self.res, self.origin[1] + (np.asarray(i) + 0.5) * self.res

    def update(self, t: float, pts: np.ndarray, cam_xy=None):
        """One frame: (x, y, h) points in the map frame; cam_xy: the camera's place in it (each cell's range)."""
        self.t_frame = t
        if self.res is None or pts is None or not len(pts):
            return
        i, j, ok = self.ij(pts[:, :2])
        i, j, h = i[ok], j[ok], pts[ok, 2]
        dist = (np.hypot(pts[ok, 0] - cam_xy[0], pts[ok, 1] - cam_xy[1]) if cam_xy is not None
                else np.zeros(len(h)))
        W = self.n.shape[1]
        cell = i * W + j
        order = np.lexsort((h, cell))
        cell, h, dist = cell[order], h[order], dist[order]
        starts = np.r_[0, np.nonzero(np.diff(cell))[0] + 1]
        counts = np.diff(np.r_[starts, len(cell)])
        keep = counts >= MIN_PTS
        starts, counts = starts[keep], counts[keep]
        ids = cell[starts]
        med = h[starts + (counts - 1) // 2]
        p80 = h[starts + ((counts - 1) * 9) // 10]          # the 90th percentile (field name kept: p80)
        lo = h[starts + (counts - 1) // 10]               # the 10th percentile: one flying pixel isn't a drop
        rng_ = dist[starts]
        ci, cj = ids // W, ids % W
        n0 = self.n[ci, cj]
        n0 = np.where((n0 > 0) & (np.abs(p80 - self.p80[ci, cj]) > CHANGE), np.minimum(n0, 1.0), n0)
        w = 1.0 / (n0 + 1.0)
        self.med[ci, cj] = (1 - w) * self.med[ci, cj] + w * med
        self.p80[ci, cj] = (1 - w) * self.p80[ci, cj] + w * p80
        self.low[ci, cj] = (1 - w) * self.low[ci, cj] + w * lo
        self.near[ci, cj] = (1 - w) * self.near[ci, cj] + w * rng_
        self.n[ci, cj] = np.minimum(n0 + 1.0, N_MAX)
        self.t[ci, cj] = t
        moved = (self.label[ci, cj] != 0) & (np.abs(self.p80[ci, cj] - self.label_h[ci, cj]) > LABEL_CHANGE)
        self.label[ci[moved], cj[moved]] = 0

    def set_label(self, x: float, y: float, r: float, label: str) -> int:
        """An AI's or a person's word on the cells within r of (x, y): one of LABELS. The number of cells labelled.
        A label can always add caution (low, obstacle, drop); one that relaxes (floor, rug, step) applies only to
        cells under OBS_H and above any drop: no word can clear an obstacle or a drop the camera measures."""
        c = LABELS[label]
        i, j = self._disc(x, y, r)
        if c in (FLOOR, STEP):
            ok = (self.p80[i, j] < OBS_H) & (self.med[i, j] > -DROP_H)
            i, j = i[ok], j[ok]
        self.label[i, j] = c
        self.label_h[i, j] = self.p80[i, j]
        return len(i)

    def things(self, classes: np.ndarray, x: float, y: float, r: float = 2.0, max_size: float = 0.6,
               walls: np.ndarray | None = None) -> list:
        """The LOW and OBSTACLE regions within r of (x, y) that are no bigger than max_size across (a thing, not a wall
        or a sofa's edge), unlabelled, not within 15 cm of walls (the map's occupied cells, when given): dicts of x, y
        (their middle), size (m), top (m, over the floor), class."""
        if self.res is None:
            return []
        sel = ((classes == LOW) | (classes == OBSTACLE)) & (self.label == 0)
        if walls is not None and walls.shape == sel.shape:
            sel &= ~fv._dilate(walls, int(math.ceil(0.15 / self.res)))
        lab, nl = fv._label(sel)
        out = []
        for k in range(1, nl + 1):
            ii, jj = np.nonzero(lab == k)
            cx, cy = self.xy(ii, jj)
            mx, my = float(cx.mean()), float(cy.mean())
            size = max(float(np.ptp(cx)), float(np.ptp(cy))) + self.res
            if size > max_size or math.hypot(mx - x, my - y) > r:
                continue
            cls_ = int(np.max(classes[ii, jj]))
            out.append(dict(x=round(mx, 3), y=round(my, 3), size=round(size, 3),
                            top=round(float(self.p80[ii, jj].max()), 3), cls=NAMES[cls_]))
        return out

    def forget(self, x: float, y: float, r: float):
        """Back to UNKNOWN within r of (x, y): the thing the arm just picked (re-seen when the camera looks again)."""
        i, j = self._disc(x, y, r)
        for k in self._FIELDS:
            getattr(self, k)[i, j] = 0

    def _disc(self, x, y, r):
        if self.res is None:
            return np.zeros(0, int), np.zeros(0, int)
        H, W = self.n.shape
        k = int(math.ceil(r / self.res)) + 1
        ci, cj = int((y - self.origin[1]) / self.res), int((x - self.origin[0]) / self.res)
        ii, jj = np.mgrid[max(0, ci - k):min(H, ci + k + 1), max(0, cj - k):min(W, cj + k + 1)]
        cx, cy = self.xy(ii, jj)
        near = np.hypot(cx - x, cy - y) < r
        return ii[near], jj[near]

    def classes(self, t: float) -> np.ndarray:
        """Each cell's class now (module notes)."""
        H, W = self.n.shape
        c = np.full((H, W), UNKNOWN, np.int8)
        seen = self.n >= N_MIN
        c[seen] = FLOOR
        rel = self.p80 - self.base(seen)                  # relief over the local floor: low things, obstacles
        # a level step (a rug) has no relief inside it: raised regions against the robot's own floor; a tilt error
        # at range at most makes a false step, which is drivable like the floor
        raised = seen & (self.p80 >= STEP_MIN) & (self.p80 < OBS_H)
        if raised.any():
            lab, nl = fv._label(raised)
            if nl:
                area = np.bincount(lab.ravel(), minlength=nl + 1)[1:] * self.res ** 2
                ii, jj = np.nonzero(lab)
                ll = lab[ii, jj]
                ext = np.zeros(nl)
                for a in (ii, jj):                        # the longer side of each region's bounding box
                    lo = np.full(nl, np.iinfo(np.int64).max)
                    hi = np.full(nl, np.iinfo(np.int64).min)
                    np.minimum.at(lo, ll - 1, a)
                    np.maximum.at(hi, ll - 1, a)
                    ext = np.maximum(ext, (hi - lo + 1) * self.res)
                big = (area >= STEP_AREA) | (ext >= STEP_LEN)
                is_step = np.r_[False, big][lab]
                c[raised & is_step] = STEP
                small = raised & ~is_step
                c[small & (rel >= LOW_H) & (rel < OBS_H) & (self.n >= LOW_N)] = LOW
        c[seen & (rel >= OBS_H) & (self.p80 < TOP) & (self.n >= OBS_N)] = OBSTACLE
        deep = seen & (self.med <= -DROP_H)
        c[deep] = UNKNOWN                                 # a drop until shown otherwise: never known floor
        drop = deep & (self.n >= DROP_N) & (self.near <= DROP_RANGE)
        if drop.any():
            lab, nl = fv._label(drop)
            size = np.bincount(lab.ravel(), minlength=nl + 1)
            drop &= (size >= DROP_CELLS)[lab]
        c[drop] = DROP
        if drop.any():                                    # into the unseen shadow next to it
            unseen = c == UNKNOWN
            for _ in range(int(DROP_SHADOW / self.res)):
                grown = fv._dilate(drop, 1) & unseen & ~drop
                if not grown.any():
                    break
                drop |= grown
            c[drop] = DROP
        stale = (c == OBSTACLE) & (self.t < t - OBS_TTL)
        c[stale] = UNKNOWN
        lab = self.label != 0
        c[lab] = self.label[lab]
        return c

    def base(self, seen: np.ndarray) -> np.ndarray:
        """The local floor level round each cell: the mean median height of the seen cells within ~BASE_R that aren't
        raised much themselves (a box-filter mean; 0 where none)."""
        from scipy.ndimage import uniform_filter
        floorish = seen & (self.p80 < OBS_H)
        k = 2 * int(round(BASE_R / self.res)) + 1
        num = uniform_filter(np.where(floorish, self.med, 0.0).astype(np.float64), k, mode="constant")
        den = uniform_filter(floorish.astype(np.float64), k, mode="constant")
        return np.where(den > 1e-6, num / np.maximum(den, 1e-6), 0.0).astype(np.float32)

    def occupancy(self, classes: np.ndarray) -> np.ndarray:
        """nav_msgs/OccupancyGrid data for Nav2: 100 where an obstacle or a drop is, -1 elsewhere (row-major)."""
        return np.where((classes == OBSTACLE) | (classes == DROP), 100, -1).astype(np.int8).ravel()

    def speed_limit(self, classes: np.ndarray, t: float, x: float, y: float, heading: float,
                    v_max: float) -> tuple:
        """(m/s, why) for the robot at (x, y) going toward heading (rad, map frame): module notes."""
        if t - self.t_frame > 1.0:
            return min(V_BLIND, v_max), "camera silent"
        if self.res is None:
            return min(V_BLIND, v_max), "no map"
        c, s = math.cos(heading), math.sin(heading)
        half = ROBOT_RADIUS + 0.05
        H, W = classes.shape
        k = int(math.ceil((LOOK + half) / self.res)) + 1
        ci, cj = int((y - self.origin[1]) / self.res), int((x - self.origin[0]) / self.res)
        ii, jj = np.mgrid[max(0, ci - k):min(H, ci + k + 1), max(0, cj - k):min(W, cj + k + 1)]
        cx, cy = self.xy(ii, jj)
        along = (cx - x) * c + (cy - y) * s
        across = -(cx - x) * s + (cy - y) * c
        cl = classes[ii, jj]
        path = (along > ROBOT_RADIUS) & (along < LOOK) & (np.abs(across) < half)
        v, why = v_max, "clear"
        unk = path & (cl == UNKNOWN)
        if unk.any():
            d = max(float(along[unk].min()) - ROBOT_RADIUS - 0.10, 0.0)
            vu = max(math.sqrt(2 * BRAKE * d), V_MIN)
            if vu < v:
                v, why = vu, f"unknown floor {d:.2f} m ahead"
        near = (along > 0) & (along < LOOK) & (np.abs(across) < half + 0.3) & (cl == LOW)
        if near.any() and V_LOW < v:
            v, why = V_LOW, "a low thing near the path"
        return min(v, v_max), why


class Catalog:
    """Things the height map found that an AI (or a person) can name: id, place (map frame), size, height, the
    class the map gave it, the label given, a note. A thing found again within MATCH of a known one is the same."""

    MATCH = 0.2

    def __init__(self, items: list | None = None):
        self.items = list(items or [])
        self.next_id = 1 + max((it["id"] for it in self.items), default=0)

    def add(self, thing: dict, t: float) -> dict | None:
        """A sighting: the new entry, or None (known: its last sighting updated)."""
        for it in self.items:
            if math.hypot(it["x"] - thing["x"], it["y"] - thing["y"]) < self.MATCH:
                it["t_last"] = round(t, 1)
                return None
        it = dict(thing, id=self.next_id, label=None, note="", t_first=round(t, 1), t_last=round(t, 1))
        self.next_id += 1
        self.items.append(it)
        return it

    def unlabelled(self) -> list:
        return [it for it in self.items if it["label"] is None]

    def label(self, id_: int, label: str, note: str = "") -> dict | None:
        if label not in LABELS:
            raise ValueError(f"label: one of {', '.join(LABELS)}")
        for it in self.items:
            if it["id"] == int(id_):
                it["label"], it["note"] = label, str(note)[:200]
                return it
        return None
