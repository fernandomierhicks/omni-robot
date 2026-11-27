"""Frontier exploration for the console's Explore mode, ROS-free (numpy + scipy on slam_toolbox's occupancy grid).

A frontier is free space next to unknown space. Each pass: the free cells far enough from walls that the robot fits
(clearance), the part of them the robot can reach, the frontier cells grouped into clusters, and for every cluster big
enough and inside the allowed radius a goal on reachable space next to it. The cheapest goal wins: distance minus a
bonus for size, so a large opening across the room beats a crack nearby. Spots that failed, and spots already reached
(their opening can't be cleared from there), are not goals again (blacklist), but the opening's other spots are: whole
openings were skipped once their middle's spot had been tried, and the kitchen, back hall, pantry and bathroom 3 stayed
unexplored behind 9 openings (Nov 25, world:=home, hm16). Nav2 drives there; when the cluster has been seen
(no unknown left around the goal) the mission picks the next one. The height map's drops and floor obstacles count as
walls (with_blocked): the lidar maps the floor past a drop as free.

A cluster's goal is reached from one of its own cells (the one nearest its middle), never its centroid, and lies at
least min_goal from the robot. A new map's first scan is a small starburst ringed by one frontier (slam_toolbox marks a
cell free only after two rays pass it, min_pass_through), and that ring's centroid is the robot itself: the goal fell
inside Nav2's goal tolerance, "succeeded" at once and was picked again, the robot never moved, slam_toolbox added no
scans (it needs minimum_travel_distance) and the map stayed at ~2 m² (Nov 24, arm step, A2M8).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

FREE_MAX = 25                       # occupancy <= this is free (slam_toolbox writes 0 / 100 / -1)
OCC_MIN = 65
SIZE_BONUS = 1.0                    # m of travel a metre of frontier is worth
EIGHT = np.ones((3, 3), bool)


@dataclass
class Grid:
    """An occupancy grid (row 0 at the origin, rows going +y), origin yaw 0 as slam_toolbox publishes it."""
    data: np.ndarray                # (rows, cols) int8: -1 unknown, 0..100
    res: float
    ox: float
    oy: float

    @classmethod
    def from_msg(cls, m) -> "Grid":
        i = m.info
        d = np.asarray(m.data, dtype=np.int8).reshape(i.height, i.width)
        return cls(d, float(i.resolution), float(i.origin.position.x), float(i.origin.position.y))

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def xy(self, r: int, c: int) -> tuple[float, float]:
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    def inside(self, r: int, c: int) -> bool:
        return 0 <= r < self.data.shape[0] and 0 <= c < self.data.shape[1]

    def known_area(self) -> float:
        return float(np.count_nonzero(self.data >= 0)) * self.res ** 2


@dataclass
class ExploreParams:
    radius: float = 15.0            # m from the start: frontiers farther out are left alone (console explore.radius)
    min_frontier: float = 0.5       # m: smaller openings are ignored
    clearance: float = 0.35         # m from walls for a goal: robot radius 0.24 + margin, toward Nav2's 0.40 inflation
    pass_clearance: float = 0.30    # m from walls on the way there: on 5 cm cells, openings from 0.55 m. At 0.35 every
                                    # doorway under 0.7 m, narrower still with the floor camera's marks on its jambs, read
                                    # as shut (hm16; 0.33 shuts the kitchen past the island's stools). Bathroom 3's 24 in
                                    # door (~0.57 m clear) still passes: the robot got in and MPPI dithered in it on the way
                                    # out until the jam check (Nov 26, hm32); 0.31 shut it but also bedroom 3 (past its
                                    # bed). Where Nav2 (0.24) takes the robot past this, frontier_goal also starts from the
                                    # start (hm31)
    blacklist_r: float = 0.6        # m around a failed or reached goal
    min_goal: float = 0.5           # m from the robot: nearer goals are inside Nav2's tolerance or move it too little
                                    # for slam_toolbox to add a scan (minimum_travel_distance 0.25)
    max_reach: float = 1.0          # m from a frontier cell to the reachable floor its goal stands on
    near: float = 1.0               # m over free floor from the robot: the passable areas it starts from (_near_labels)


def _near_labels(g: Grid, free, passable, lab, rr: int, rc: int, near: float) -> list:
    """The passable components (labels of lab) the robot reaches over free floor within near of its cell. Not only the
    one under it: in a 0.55 m passage beside the coat closet's open door the passable strip (0.28 m from walls) was a
    15-cell island, every frontier looked unreachable from it and Explore went home with bedroom 3 unseen (Nov 26,
    hm26); Nav2 (robot_radius 0.24) drives out of there."""
    k = int(math.ceil(near / g.res))
    r0, r1 = max(0, rr - k), min(g.data.shape[0], rr + k + 1)
    c0, c1 = max(0, rc - k), min(g.data.shape[1], rc + k + 1)
    yy, xx = np.mgrid[r0:r1, c0:c1]
    win = free[r0:r1, c0:c1] & ((yy - rr) ** 2 + (xx - rc) ** 2 <= k * k)
    if not win.any():
        return []
    wl, _ = ndimage.label(win, structure=EIGHT)
    lr, lc = rr - r0, rc - c0
    if not win[lr, lc]:                                   # the robot's own cell isn't free (a wall's noise): the nearest
        _, idx = ndimage.distance_transform_edt(~win, return_indices=True)
        lr, lc = int(idx[0][lr, lc]), int(idx[1][lr, lc])
    own = (wl == wl[lr, lc]) & passable[r0:r1, c0:c1]
    return [int(v) for v in np.unique(lab[r0:r1, c0:c1][own]) if v]


def frontier_goal(g: Grid, robot: tuple, start: tuple, p: ExploreParams, blacklist=()) -> tuple:
    """The next goal (x, y, yaw) or None, and a dict of counts for the operator. blacklist: (x, y) of goals that failed
    or were reached; no goal within blacklist_r of one (a cluster with no other usable spot is left alone)."""
    d = g.data
    unknown = d < 0
    occ = d >= OCC_MIN
    free = (d >= 0) & (d <= FREE_MAX)
    info = dict(frontiers=0, known_m2=round(g.known_area(), 1))
    if not free.any():
        return None, info
    wall_dist = ndimage.distance_transform_edt(~occ) * g.res
    passable = free & (wall_dist >= min(p.pass_clearance, p.clearance))
    safe = free & (wall_dist >= p.clearance)
    rr, rc = g.cell(*robot[:2])
    if not g.inside(rr, rc) or not passable.any():
        return None, info
    lab, _ = ndimage.label(passable, structure=EIGHT)
    ids = _near_labels(g, free, passable, lab, rr, rc, p.near)
    if not ids:                                           # nothing near over free floor: the nearest passable cell's
        _, idx = ndimage.distance_transform_edt(~passable, return_indices=True)
        ids = [int(lab[idx[0][rr, rc], idx[1][rr, rc]])]
    sr, sc = g.cell(*start[:2])                           # and what the start reaches: the robot can always get back
    if g.inside(sr, sc):                                  # there (Nav2 drove it into bathroom 3 through a door Explore
        ids = ids + _near_labels(g, free, passable, lab, sr, sc, p.near)    # couldn't see a way out of, hm31)
    reach = np.isin(lab, ids) & safe                      # goals: the reachable spots clear of walls
    cross = ndimage.generate_binary_structure(2, 1)
    frontier = free & ndimage.binary_dilation(unknown, structure=cross)
    flab, n = ndimage.label(frontier, structure=EIGHT)
    if n == 0:
        return None, info
    reach_dist, ridx = ndimage.distance_transform_edt(~reach, return_indices=True)
    sizes = ndimage.sum_labels(frontier, flab, index=np.arange(1, n + 1)) * g.res
    cen = np.array(ndimage.center_of_mass(frontier, flab, index=np.arange(1, n + 1)), float).reshape(n, 2)
    # every frontier cell's goal: the nearest reachable cell; usable if near enough the cell and far enough from the robot
    fr, fc = np.nonzero(frontier)
    lab_f = flab[fr, fc] - 1
    gr_f, gc_f = ridx[0][fr, fc], ridx[1][fr, fc]
    gx_f, gy_f = g.ox + (gc_f + 0.5) * g.res, g.oy + (gr_f + 0.5) * g.res
    ok = ((reach_dist[fr, fc] * g.res <= p.max_reach)
          & (np.hypot(gx_f - robot[0], gy_f - robot[1]) >= p.min_goal))
    for b in blacklist:                                   # a spot tried (reached or failed) is not tried again; the
        ok &= np.hypot(gx_f - b[0], gy_f - b[1]) >= p.blacklist_r     # opening's other spots still are
    d_mid = (fr - cen[lab_f, 0]) ** 2 + (fc - cen[lab_f, 1]) ** 2
    d_mid[~ok] = np.inf
    order = np.lexsort((d_mid, lab_f))                     # by cluster, then nearest its middle first
    labs, first = np.unique(lab_f[order], return_index=True)
    pick = dict(zip(labs.tolist(), order[first].tolist()))
    best, best_cost, count = None, math.inf, 0
    for k in range(n):
        size = float(sizes[k])
        if size < p.min_frontier:
            continue
        i = pick.get(k)
        if i is None or not math.isfinite(d_mid[i]):      # no usable goal: no floor near it, or only at the robot
            continue
        fx, fy = g.xy(int(fr[i]), int(fc[i]))
        if math.hypot(fx - start[0], fy - start[1]) > p.radius:
            continue
        gx, gy = float(gx_f[i]), float(gy_f[i])
        count += 1
        cost = math.hypot(gx - robot[0], gy - robot[1]) - SIZE_BONUS * size
        if cost < best_cost:
            yaw = math.atan2(fy - gy, fx - gx) if math.hypot(fx - gx, fy - gy) > 0.05 else robot[2]
            best, best_cost = (gx, gy, yaw), cost
    info["frontiers"] = count
    return best, info


def with_blocked(g: Grid, b: Grid | None) -> Grid:
    """g with b's occupied cells (both map frame, yaw 0; any resolutions) as walls: the height map's drops and floor
    obstacles (/floor_obstacles/grid). The lidar sees over a drop and maps the floor beyond it free: in the home world
    Explore sent the robot for the den's openings, 165 mm down, and the floor camera's drop held it at the doors until
    the jam check gave up (Nov 25, hm11). As walls, what lies past a drop is unreachable and its openings are skipped."""
    if b is None or not (b.data >= OCC_MIN).any():
        return g
    rows, cols = g.data.shape
    br = np.floor((g.oy + (np.arange(rows) + 0.5) * g.res - b.oy) / b.res).astype(int)
    bc = np.floor((g.ox + (np.arange(cols) + 0.5) * g.res - b.ox) / b.res).astype(int)
    ri, ci = (br >= 0) & (br < b.data.shape[0]), (bc >= 0) & (bc < b.data.shape[1])
    blocked = np.zeros(g.data.shape, bool)
    blocked[np.ix_(ri, ci)] = b.data[np.ix_(br[ri], bc[ci])] >= OCC_MIN
    if not blocked.any():
        return g
    d = g.data.copy()
    d[blocked] = 100
    return Grid(d, g.res, g.ox, g.oy)


def connected(g: Grid, a: tuple, b: tuple, discs=(), r: float = 0.0, clearance: float = 0.24) -> bool:
    """Can the robot get from a to b (x, y) on g's free space clear of walls, with discs (x, y) of radius r shut?
    (Explore's blocks must never cut the robot off from where it started.)"""
    occ = g.data >= OCC_MIN
    if discs:
        rows, cols = g.data.shape
        ys = g.oy + (np.arange(rows) + 0.5) * g.res
        xs = g.ox + (np.arange(cols) + 0.5) * g.res
        for dx, dy in discs:
            occ = occ | (((xs[None, :] - dx) ** 2 + (ys[:, None] - dy) ** 2) <= r * r)
    free = (g.data >= 0) & (g.data <= FREE_MAX) & ~occ
    ok = free & (ndimage.distance_transform_edt(~occ) * g.res >= clearance)
    cells = []
    for x, y in (a, b):
        rr, cc = g.cell(x, y)
        if not g.inside(rr, cc):
            return False
        if not ok[rr, cc]:                                # next to a wall: from the nearest clear cell
            if not ok.any():
                return False
            _, idx = ndimage.distance_transform_edt(~ok, return_indices=True)
            rr, cc = int(idx[0][rr, cc]), int(idx[1][rr, cc])
        cells.append((rr, cc))
    lab, _ = ndimage.label(ok, structure=EIGHT)
    return bool(lab[cells[0]] and lab[cells[0]] == lab[cells[1]])


def clearance_at(g: Grid, xy: tuple, reach: float = 1.0) -> float:
    """m from (x, y) to the nearest wall on g, looked for within reach (beyond: reach)."""
    r, c = g.cell(xy[0], xy[1])
    k = int(math.ceil(reach / g.res))
    r0, r1 = max(0, r - k), min(g.data.shape[0], r + k + 1)
    c0, c1 = max(0, c - k), min(g.data.shape[1], c + k + 1)
    if r0 >= r1 or c0 >= c1:
        return reach
    occ = np.argwhere(g.data[r0:r1, c0:c1] >= OCC_MIN)
    if occ.size == 0:
        return reach
    cy, cx = g.oy + (occ[:, 0] + r0 + 0.5) * g.res, g.ox + (occ[:, 1] + c0 + 0.5) * g.res
    # to the wall cell's near edge (the costmap's inflation counts from cells)
    return float(max(0.0, min(reach, np.min(np.hypot(cx - xy[0], cy - xy[1])) - 0.5 * g.res)))


def backout_path(g: Grid, trail: list, clear: float, max_len: float, min_len: float = 0.0) -> list:
    """The way back out of a tight spot: the robot's own trail (oldest first, its pose last), newest back, to the first
    point at least clear from walls and min_len of travel back, within max_len. The points to drive through, or []
    (none clear enough). The trail is where the robot's body has just been, so driving it backwards is free even past
    what the lidar can't see (a stool's legs: the floor camera marked them only once the robot stood beside them, Nov
    26, hm23), and with the pose off it still undoes the last motion (pinned on a ramp's jamb, hm30)."""
    path, run = [], 0.0
    for k in range(len(trail) - 2, -1, -1):
        run += math.hypot(trail[k][0] - trail[k + 1][0], trail[k][1] - trail[k + 1][1])
        if run > max_len:
            return []
        path.append(trail[k])
        if run >= min_len and clearance_at(g, trail[k]) >= clear:
            return path
    return []


def still_open(g: Grid, goal: tuple, radius: float = 0.8) -> bool:
    """Is there unknown space within radius of the goal (else it has been seen: pick the next frontier)?"""
    r, c = g.cell(goal[0], goal[1])
    k = int(math.ceil(radius / g.res))
    r0, r1 = max(0, r - k), min(g.data.shape[0], r + k + 1)
    c0, c1 = max(0, c - k), min(g.data.shape[1], c + k + 1)
    if r0 >= r1 or c0 >= c1:
        return False
    return bool((g.data[r0:r1, c0:c1] < 0).any())
