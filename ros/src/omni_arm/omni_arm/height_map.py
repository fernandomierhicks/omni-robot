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
                       frames over DROP_CELLS cells (stairs, a step down), its edge seen from within EDGE_SEE, and
                       the unseen cells next to it (the lower floor just past an edge is hidden from above). Until
                       the edge is seen that near it is a suspected drop: UNKNOWN, and the robot slows to V_EDGE
                       near it (from far a ramp down hides past its edge just like a step)
  levels     a floor at another height (OBS_H or more up or down, big) joined to the robot's floor by a walkable
             surface (cells spanning under SMOOTH, neighbours under JUMP apart: a ramp) is floor (raised: STEP),
             never a drop or an obstacle; things on it are judged against it (Nov 25)
  datum      heights are stored against one level for the whole map, not each frame's floor: every frame is
             registered on the map (the median height difference over the smooth cells both see, REG_MIN of them
             agreeing within REG_TOL), which gives the robot's floor (z) in it; classes judge against z. Stored
             against each frame's own floor, the family room seen from on top read level with the hallway seen from
             below, and its ramp's top, glimpsed from the hallway, became a 68 mm ridge across the doorway: no path
             home (Nov 25, world:=home, hm12). A frame overlapping the map without agreeing is dropped
  footprint  where the robot stands (within FOOTPRINT_CLEAR) nothing is an obstacle or a drop
  costmap    OBSTACLE and DROP only (Nav2's StaticLayer on /floor_obstacles/grid). Nothing low ever closes a door
  speed      speed_limit(): along the way the robot is going, the nearest UNKNOWN cell sets a speed it can stop
             in before reaching it (BRAKE m/s^2, never under V_MIN); a LOW thing on the path (or just beside it,
             near) likewise, down to V_LOW at it (Nov 26; before, any LOW within 2.5 m capped V_LOW flat); a known
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
# Ramps against drops (Nov 25, the home's family room: a 13 mm lip and a 20 deg ramp, 68 mm, at each entry; the den a
# 165 mm step): a floor at another height that a walkable surface joins to the robot's is a floor, wherever it is
SMOOTH = 0.035          # m: a cell whose points span less than this (90th - 10th percentile) is a surface: a floor or
                        # a ramp (20 deg is 18 mm over a 5 cm cell, plus the lip), not an edge or a thing's side
JUMP = 0.035            # m: neighbouring surface cells this close in height are one surface
LEVEL_AREA, LEVEL_LEN = 0.25, 0.6   # m^2, m: a surface region off the robot's level this big is a floor (smaller: a
                        # thing's top, judged as before)
EDGE_SEE = 0.40         # m: a drop is confirmed only once the camera has seen its edge from this near (horizontal).
                        # Past an edge, from far, a ramp hides under the grazing rays exactly as the floor past a step
                        # does; from within H / tan(slope) (0.51 m for 20 deg at the riser's 0.19 m) the ramp is in
                        # view and joins its floor. Until then the low floor is a suspected drop: unknown, approached
FIT_COS, FIT_D = math.cos(math.radians(4.0)), 0.025   # a frame whose floor fit is further than this from the
                        # robot's own floor (tilt, offset) is skipped: near a ramp's top the floor left in view is a
                        # sliver and the fit takes the ramp or the floor past it (sim, Nov 25: 10-19 deg, 80-220 mm
                        # off, heights 100 mm wrong); on the ramp the base is pitched. Driving level: within 1.5 deg, 15 mm
V_EDGE = 0.2            # m/s: at this speed, within EDGE_LOOK of a suspected drop, so the camera gets frames between
EDGE_LOOK = 1.0         # seeing a ramp (0.51 m) and confirming a drop (EDGE_SEE), and stops short of a real one
BASE_R = 0.5            # m: the local floor level round a cell (seen floor cells within ~this): heights against it
                        # (relief) cancel the frame planes' small tilt errors, which grow with range
ROBOT_RADIUS = 0.24
BRAKE = 0.8             # m/s^2: the deceleration the speed limit budgets for (est; MPPI's limit is 2)
V_MIN, V_LOW, V_BLIND = 0.25, 0.4, 0.3      # m/s: creeping into the unknown, near a low thing, the camera silent
LOOK = 2.5              # m: how far along the path the speed limit looks
LOW_SIDE, LOW_SIDE_NEAR = 0.3, 0.5   # m: a LOW thing this far beside the path slows the robot once this near ahead
LABEL_CHANGE = 0.006    # m: a labelled cell whose height changes this much forgets the label
LEVEL_EDGE = 0.15       # m from a floor up or down a ramp: relief there is also judged against the higher floor
                        # round it (base_hi), never only the two floors' mean
FOOTPRINT_CLEAR = ROBOT_RADIUS + 0.05   # m round the robot: no obstacle or drop. A mark just inside the robot's
                        # inscribed radius put its own cell in collision: Nav2 planned nowhere and its backup and spin
                        # refused to move (Nov 25, world:=home, hm15: by the family room's sofa). The robot is there,
                        # so nothing else is; a real thing that close is the lidar's and the collision monitor's
REG_MIN = 30           # overlapping smooth cells a frame needs to register its floor on the map (fewer: new ground,
                        # stored against the last z)
REG_TOL, REG_AGREE = 0.012, 0.5   # m, share: that many of them within this of the median, else the frame is dropped
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
    if n[2] < FIT_COS or abs(d) > FIT_D:
        return None                                       # not the robot's own floor: skip the frame
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
        self.close = np.zeros((0, 0), np.float32)      # the nearest it was ever seen from (0 = never)
        self.label = np.zeros((0, 0), np.int8)         # 0 = none, else a class
        self.label_h = np.zeros((0, 0), np.float32)    # p80 when labelled
        self.t_frame = -math.inf                       # the last frame's time (camera health)
        self.suspect = np.zeros((0, 0), bool)          # the last classes()'s suspected drops (the speed limit's)
        self.z = 0.0                                   # the robot's floor against the map's datum (registration)
        self.n_reg, self.n_unreg = 0, 0                # frames registered on the map / dropped as not agreeing

    _FIELDS = ("n", "med", "p80", "low", "t", "near", "close", "label", "label_h")

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
        # the frame's floor against the map's datum: the smooth cells both see (module notes: datum)
        both = ((self.n[ci, cj] >= N_MIN) & (self.p80[ci, cj] - self.low[ci, cj] < SMOOTH) & (p80 - lo < SMOOTH)
                & (self.label[ci, cj] == 0))
        if both.sum() >= REG_MIN:
            diff = self.med[ci, cj][both] - med[both]
            z = float(np.median(diff))
            if np.mean(np.abs(diff - z) < REG_TOL) < REG_AGREE:
                self.n_unreg += 1
                return                                    # it overlaps the map without agreeing: not stored
            self.z = z
            self.n_reg += 1
        med, p80, lo = med + self.z, p80 + self.z, lo + self.z
        n0 = self.n[ci, cj]
        n0 = np.where((n0 > 0) & (np.abs(p80 - self.p80[ci, cj]) > CHANGE), np.minimum(n0, 1.0), n0)
        w = 1.0 / (n0 + 1.0)
        self.med[ci, cj] = (1 - w) * self.med[ci, cj] + w * med
        self.p80[ci, cj] = (1 - w) * self.p80[ci, cj] + w * p80
        self.low[ci, cj] = (1 - w) * self.low[ci, cj] + w * lo
        self.near[ci, cj] = (1 - w) * self.near[ci, cj] + w * rng_
        cl0 = self.close[ci, cj]
        self.close[ci, cj] = np.where(cl0 > 0, np.minimum(cl0, rng_ + 1e-3), rng_ + 1e-3)
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
            ok = (self.p80[i, j] - self.z < OBS_H) & (self.med[i, j] - self.z > -DROP_H)
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
                            top=round(float(self.p80[ii, jj].max()) - self.z, 3), cls=NAMES[cls_]))
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

    def classes(self, t: float, robot_xy=None) -> np.ndarray:
        """Each cell's class now (module notes). robot_xy: where the robot stands (map frame): within FOOTPRINT_CLEAR
        of it no obstacle or drop (module notes: footprint)."""
        H, W = self.n.shape
        c = np.full((H, W), UNKNOWN, np.int8)
        seen = self.n >= N_MIN
        c[seen] = FLOOR
        med, p80 = self.med - self.z, self.p80 - self.z   # against the robot's floor (module notes: datum)
        surf, level, flat = self.levels(seen, med)        # floors at other heights a walkable surface joins on
        deep = seen & (med <= -DROP_H) & ~level
        rel = p80 - self.base(seen & ~deep, level, med, p80)   # relief over the local floor: low things, obstacles
        if level.any():                                   # at a second floor's edge: against the higher one round it
            edge = fv._dilate(level, int(round(LEVEL_EDGE / self.res)))
            rel = np.where(edge, np.minimum(rel, p80 - self.base_hi(seen & ~deep, level, med, p80)), rel)
        # a level step (a rug) has no relief inside it: raised regions against the robot's own floor; a tilt error
        # at range at most makes a false step, which is drivable like the floor
        raised = seen & (p80 >= STEP_MIN) & (p80 < OBS_H)
        if raised.any():
            is_step = self._big(raised, STEP_AREA, STEP_LEN)
            c[raised & is_step] = STEP
            small = raised & ~is_step
            c[small & (rel >= LOW_H) & (rel < OBS_H) & (self.n >= LOW_N)] = LOW
        # the robot's own floor (flat) is never an obstacle, even beside a lower floor that pulls its relief up
        c[seen & (rel >= OBS_H) & (p80 < TOP) & (self.n >= OBS_N) & ~level & ~flat] = OBSTACLE
        c[level & (med >= STEP_MIN)] = STEP          # a raised floor up a ramp; a lower one stays FLOOR
        c[deep] = UNKNOWN                                 # a drop until shown otherwise: never known floor
        drop = deep & (self.n >= DROP_N) & (self.near <= DROP_RANGE)
        if drop.any():
            lab, nl = fv._label(drop)
            size = np.bincount(lab.ravel(), minlength=nl + 1)
            drop &= (size >= DROP_CELLS)[lab]
        k = int(DROP_SHADOW / self.res)
        unseen = ~seen
        if drop.any():                                    # confirmed only where its edge was seen from near
            gap = unseen & fv._dilate(drop, k)            # the hidden strip between the floor and the low one
            edge = surf & ~deep & fv._dilate(gap, 1) & (self.close > 0) & (self.close <= EDGE_SEE)
            near_edge = drop & fv._dilate(edge, k)
            if near_edge.any():
                lab, nl = fv._label(drop)
                drop &= np.r_[False, np.bincount(lab[near_edge], minlength=nl + 1)[1:] > 0][lab]
            else:
                drop[:] = False
        drop = self._grow(drop, unseen | deep, k)         # into the unseen shadow next to it
        c[drop] = DROP
        self.suspect = self._grow(deep & ~drop, unseen & ~drop, k)   # a suspected drop and its shadow: slow there
        stale = (c == OBSTACLE) & (self.t < t - OBS_TTL)
        c[stale] = UNKNOWN
        lab = self.label != 0
        c[lab] = self.label[lab]
        if robot_xy is not None:                          # the robot stands there: nothing can be (footprint)
            i, j = self._disc(robot_xy[0], robot_xy[1], FOOTPRINT_CLEAR)
            under = (c[i, j] == OBSTACLE) | (c[i, j] == DROP)
            c[i[under], j[under]] = FLOOR
        return c

    @staticmethod
    def _grow(mask: np.ndarray, into: np.ndarray, steps: int) -> np.ndarray:
        """mask grown cell by cell into the cells of into, at most steps."""
        mask = mask.copy()
        if mask.any():
            for _ in range(steps):
                grown = fv._dilate(mask, 1) & into & ~mask
                if not grown.any():
                    break
                mask |= grown
        return mask

    def _big(self, mask: np.ndarray, area: float, length: float) -> np.ndarray:
        """The cells of mask in regions at least area big or length long (their bounding box's longer side)."""
        lab, nl = fv._label(mask)
        if not nl:
            return np.zeros_like(mask)
        a = np.bincount(lab.ravel(), minlength=nl + 1)[1:] * self.res ** 2
        ii, jj = np.nonzero(lab)
        ll = lab[ii, jj]
        ext = np.zeros(nl)
        for v in (ii, jj):
            lo = np.full(nl, np.iinfo(np.int64).max)
            hi = np.full(nl, np.iinfo(np.int64).min)
            np.minimum.at(lo, ll - 1, v)
            np.maximum.at(hi, ll - 1, v)
            ext = np.maximum(ext, (hi - lo + 1) * self.res)
        return np.r_[False, (a >= area) | (ext >= length)][lab]

    def levels(self, seen: np.ndarray, med: np.ndarray | None = None):
        """(surf, level, flat): surf the seen cells whose points lie on a surface (span under SMOOTH); level the big
        regions (LEVEL_AREA / LEVEL_LEN) of surface off the robot's level (OBS_H or more up or down) that a chain of
        surface cells, neighbours under JUMP apart, joins to the robot's level: a floor up or down a ramp; flat the
        surface cells at the robot's level on that chain."""
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        surf = seen & (self.p80 - self.low < SMOOTH)
        H, W = surf.shape
        med = self.med - self.z if med is None else med       # against the robot's floor
        idx = np.arange(H * W).reshape(H, W)
        eh = surf[:, :-1] & surf[:, 1:] & (np.abs(med[:, :-1] - med[:, 1:]) < JUMP)
        ev = surf[:-1, :] & surf[1:, :] & (np.abs(med[:-1, :] - med[1:, :]) < JUMP)
        r = np.r_[idx[:, :-1][eh], idx[:-1, :][ev]]
        q = np.r_[idx[:, 1:][eh], idx[1:, :][ev]]
        _, lab = connected_components(coo_matrix((np.ones(len(r), np.int8), (r, q)), shape=(H * W, H * W)),
                                      directed=False)
        lab = lab.reshape(H, W)
        home = surf & (np.abs(med) < OBS_H / 2)          # the robot's own level (heights are against its floor)
        walk = surf & np.isin(lab, np.unique(lab[home]))
        off = walk & (np.abs(med) >= OBS_H)
        return surf, (off & self._big(off, LEVEL_AREA, LEVEL_LEN)) if off.any() else off, home

    def base(self, seen: np.ndarray, level: np.ndarray | None = None, med: np.ndarray | None = None,
             p80: np.ndarray | None = None) -> np.ndarray:
        """The local floor level round each cell: the mean median height of the seen cells within ~BASE_R that aren't
        raised much themselves, or are a floor up a ramp (level; a box-filter mean; 0 where none)."""
        from scipy.ndimage import uniform_filter
        med = self.med - self.z if med is None else med       # against the robot's floor
        p80 = self.p80 - self.z if p80 is None else p80
        floorish = seen & (p80 < OBS_H)
        if level is not None:
            floorish |= level
        k = 2 * int(round(BASE_R / self.res)) + 1
        num = uniform_filter(np.where(floorish, med, 0.0).astype(np.float64), k, mode="constant")
        den = uniform_filter(floorish.astype(np.float64), k, mode="constant")
        return np.where(den > 1e-6, num / np.maximum(den, 1e-6), 0.0).astype(np.float32)

    def base_hi(self, seen: np.ndarray, level: np.ndarray, med: np.ndarray, p80: np.ndarray) -> np.ndarray:
        """The highest floor round each cell (within ~BASE_R; -inf where none). Next to a floor up or down a ramp
        base()'s mean mixes the two, and a raised floor's edge cell that isn't a smooth surface (the ramp's top, a
        cell half on each) stood ~34 mm over it: an obstacle. Inflated, a few of those closed the hallway's doorway
        with the robot on its ramp (Nov 25, world:=home, hm14). A thing on the lower floor still stands OBS_H over
        both floors only if taller than the step; that holds for the robot's real obstacles (est)."""
        from scipy.ndimage import maximum_filter
        floorish = seen & ((p80 < OBS_H) | level)
        k = 2 * int(round(BASE_R / self.res)) + 1
        return maximum_filter(np.where(floorish, med, -np.inf).astype(np.float64), size=k, mode="constant",
                              cval=-np.inf).astype(np.float32)

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
        # a LOW thing: brake to V_LOW before reaching it (as for unknown floor), on the path or within LOW_SIDE beside
        # it once near. Nov 26 (world:=course): a flat V_LOW for any LOW cell 2.5 m ahead and 0.59 m either side held
        # Go To at 0.40 m/s down a 1.2 m hall (cells at the walls' feet, floor noise over LOW_H) under a 2.0 cap
        low = (cl == LOW) & (along > 0) & (along < LOOK) & (
            (np.abs(across) < half) | ((np.abs(across) < half + LOW_SIDE) & (along < ROBOT_RADIUS + LOW_SIDE_NEAR)))
        if low.any():
            d = max(float(along[low].min()) - ROBOT_RADIUS - 0.10, 0.0)
            vl = max(math.sqrt(2 * BRAKE * d), V_LOW)
            if vl < v:
                v, why = vl, f"a low thing {d:.2f} m ahead"
        if self.suspect.shape == classes.shape:
            sus = (along > 0) & (along < ROBOT_RADIUS + EDGE_LOOK) & (np.abs(across) < half) & self.suspect[ii, jj]
            if sus.any() and V_EDGE < v:
                v, why = V_EDGE, "a suspected drop ahead (a step or a ramp)"
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
