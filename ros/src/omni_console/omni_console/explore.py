"""Frontier exploration for the console's Explore mode, ROS-free (numpy + scipy on slam_toolbox's occupancy grid).

A frontier is free space next to unknown space. Each pass: the free cells far enough from walls that the robot fits
(clearance), the part of them the robot can reach, the frontier cells grouped into clusters, and for every cluster big
enough and inside the allowed radius a goal on reachable space next to it. The cheapest goal wins: distance minus a
bonus for size, so a large opening across the room beats a crack nearby. Goals that failed are skipped (blacklist).
Nav2 drives there; when the cluster has been seen (no unknown left around the goal) the mission picks the next one.
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
    radius: float = 10.0            # m from the start: frontiers farther out are left alone
    min_frontier: float = 0.5       # m: smaller openings are ignored
    clearance: float = 0.35         # m from walls for a goal: robot radius 0.24 + margin, toward Nav2's 0.40 inflation
    blacklist_r: float = 0.6        # m around a failed goal


def frontier_goal(g: Grid, robot: tuple, start: tuple, p: ExploreParams, blacklist=()) -> tuple:
    """The next goal (x, y, yaw) or None, and a dict of counts for the operator."""
    d = g.data
    unknown = d < 0
    occ = d >= OCC_MIN
    free = (d >= 0) & (d <= FREE_MAX)
    info = dict(frontiers=0, known_m2=round(g.known_area(), 1))
    if not free.any():
        return None, info
    wall_dist = ndimage.distance_transform_edt(~occ) * g.res
    safe = free & (wall_dist >= p.clearance)
    rr, rc = g.cell(*robot[:2])
    if not g.inside(rr, rc):
        return None, info
    if not safe[rr, rc]:                                  # the robot sits close to a wall: start from the nearest
        if not safe.any():
            return None, info
        _, idx = ndimage.distance_transform_edt(~safe, return_indices=True)
        rr, rc = int(idx[0][rr, rc]), int(idx[1][rr, rc])
    lab, _ = ndimage.label(safe, structure=EIGHT)
    reach = lab == lab[rr, rc]
    cross = ndimage.generate_binary_structure(2, 1)
    frontier = free & ndimage.binary_dilation(unknown, structure=cross)
    flab, n = ndimage.label(frontier, structure=EIGHT)
    if n == 0:
        return None, info
    reach_dist, ridx = ndimage.distance_transform_edt(~reach, return_indices=True)
    sizes = ndimage.sum_labels(frontier, flab, index=np.arange(1, n + 1)) * g.res
    centroids = ndimage.center_of_mass(frontier, flab, index=np.arange(1, n + 1))
    best, best_cost, count = None, math.inf, 0
    for k in range(n):
        size = float(sizes[k])
        if size < p.min_frontier:
            continue
        cy, cx = centroids[k]
        r0, c0 = int(round(cy)), int(round(cx))
        fx, fy = g.xy(r0, c0)
        if math.hypot(fx - start[0], fy - start[1]) > p.radius:
            continue
        if reach_dist[r0, c0] * g.res > 1.0:              # no reachable floor within 1 m of the opening
            continue
        gr, gc = int(ridx[0][r0, c0]), int(ridx[1][r0, c0])
        gx, gy = g.xy(gr, gc)
        if any(math.hypot(gx - b[0], gy - b[1]) < p.blacklist_r for b in blacklist):
            continue
        count += 1
        cost = math.hypot(gx - robot[0], gy - robot[1]) - SIZE_BONUS * size
        if cost < best_cost:
            yaw = math.atan2(fy - gy, fx - gx) if math.hypot(fx - gx, fy - gy) > 0.05 else robot[2]
            best, best_cost = (gx, gy, yaw), cost
    info["frontiers"] = count
    return best, info


def still_open(g: Grid, goal: tuple, radius: float = 0.8) -> bool:
    """Is there unknown space within radius of the goal (else it has been seen: pick the next frontier)?"""
    r, c = g.cell(goal[0], goal[1])
    k = int(math.ceil(radius / g.res))
    r0, r1 = max(0, r - k), min(g.data.shape[0], r + k + 1)
    c0, c1 = max(0, c - k), min(g.data.shape[1], c + k + 1)
    if r0 >= r1 or c0 >= c1:
        return False
    return bool((g.data[r0:r1, c0:c1] < 0).any())
