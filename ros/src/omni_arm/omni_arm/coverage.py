"""Where the tidy has looked (design.md 11): the floor of its region it has seen well enough to know nothing is left to
pick up there, what the base can't drive through, and where to look from next. No ROS.

A grid of 5 cm cells over the region (a circle round where the tidy started, odom) and a margin round it:
  checked   at least min_points depth points in the cell, each within check_range of the camera: near enough that
            floor_vision would have found a small thing there (a toy farther off is only a hint); and no thing in it
  thing     raised points of a small thing's height (floor_vision's 6 mm - 10 cm) that floor_vision didn't take for one
            (cut off by the image's edge, its ring not seen): not checked until a look sees bare floor there, or finds
            the thing; still floor to look at even when it blocks the base
  blocked   points higher than obs_h (furniture, a wall, a shoe, a toy: the base would push it); cleared when the cell
            is seen again as bare floor
  free      seen as floor and not blocked, or under the base where it has been
The base goes only where every cell within its radius + clear is free: through floor it has seen, never through
floor it hasn't. plan() finds the reachable spot whose look round would check the most of the region's unchecked
floor per second (driving there plus the look), counting only floor in sight from there (no blocked cell between):
floor behind a sofa is looked for from beside it. A cell a look round should have checked (in its ring and in sight)
but didn't counts a try; after `tries` it is left unchecked. The tidy is done when no reachable look checks min_gain
more.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

CELL = 0.05
THING_H = (0.006, 0.10)                 # m: floor_vision's H_MIN, H_MAX: the heights of a thing to pick up


@dataclass
class CoverParams:
    check_range: float = 1.2            # m from the camera: floor seen nearer than this counts as checked
    min_points: int = 4                 # depth points in a cell (25 cm^2) to count it seen
    thing_points: int = 3               # raised points in a cell (and thing_share of its points) for a thing there
    thing_share: float = 0.15
    thing_merge: float = 0.08           # m: raised points this near a detection are that detection's
    obs_h: float = 0.03                 # m: higher than this the base doesn't drive over
    robot_radius: float = 0.23          # the bumper
    clear: float = 0.05                 # m between the bumper and anything not seen as free floor
    escape: float = 0.15                # m: the base may leave its own spot through cells it can't plan through
    look_s: float = 14.0                # s: a look round (six turns and looks)
    v: float = 0.25                     # m/s on the way
    min_gain: float = 0.02              # m^2: a look that would check less than this isn't worth going for
    tries: int = 2                      # look rounds that should have seen a cell before it is left as unseen


def _disk(r_cells: float) -> list:
    k = int(math.ceil(r_cells))
    return [(a, b) for a in range(-k, k + 1) for b in range(-k, k + 1) if a * a + b * b <= r_cells * r_cells]


def _dilate(m: np.ndarray, offsets) -> np.ndarray:
    out = np.zeros_like(m)
    n0, n1 = m.shape
    for a, b in offsets:
        out[max(0, a):n0 + min(0, a), max(0, b):n1 + min(0, b)] |= m[max(0, -a):n0 + min(0, -a), max(0, -b):n1 + min(0, -b)]
    return out


def _conv(a: np.ndarray, k: np.ndarray) -> np.ndarray:
    """a convolved with the odd square kernel k, the same size as a (FFT)."""
    s = (a.shape[0] + k.shape[0] - 1, a.shape[1] + k.shape[1] - 1)
    f = np.fft.irfft2(np.fft.rfft2(a, s) * np.fft.rfft2(k, s), s)
    r = k.shape[0] // 2
    return np.rint(f[r:r + a.shape[0], r:r + a.shape[1]])


@dataclass
class Plan:
    goal: tuple                         # (x, y) odom: where to look from
    path: list                          # [(x, y), ...] odom waypoints to it, the last the goal
    gain: float                         # m^2 of unchecked floor in its look ring
    dist: float                         # m along the path


class Coverage:
    def __init__(self, centre, radius: float, p: CoverParams | None = None, margin: float = 0.8):
        self.p = p or CoverParams()
        self.centre, self.radius = (float(centre[0]), float(centre[1])), float(radius)
        k = int(math.ceil((radius + margin) / CELL))
        self.n = 2 * k + 1
        self.x0, self.y0 = self.centre[0] - k * CELL, self.centre[1] - k * CELL
        ii, jj = np.mgrid[0:self.n, 0:self.n]
        self.X, self.Y = self.x0 + ii * CELL, self.y0 + jj * CELL
        self.inside = np.hypot(self.X - self.centre[0], self.Y - self.centre[1]) <= radius
        self.checked = np.zeros((self.n, self.n), bool)
        self.free = np.zeros((self.n, self.n), bool)
        self.blocked = np.zeros((self.n, self.n), bool)
        self.thing = np.zeros((self.n, self.n), bool)
        self.tries = np.zeros((self.n, self.n), int)
        self._robot = _disk((self.p.robot_radius + self.p.clear) / CELL)
        self._last = None

    # -- cells --------------------------------------------------------------------------------------------------------
    def cell(self, x: float, y: float):
        i, j = int(round((x - self.x0) / CELL)), int(round((y - self.y0) / CELL))
        return (i, j) if 0 <= i < self.n and 0 <= j < self.n else None

    def _ring(self, x, y, near, far) -> np.ndarray:
        r = np.hypot(self.X - x, self.Y - y)
        return (r >= near) & (r <= far)

    def floor(self) -> np.ndarray:
        """The region's floor to look over: not blocked, or a thing not yet made out."""
        return self.inside & (~self.blocked | self.thing)

    def todo(self) -> np.ndarray:
        """The region's cells still to check: floor not checked, not given up on."""
        return self.floor() & ~self.checked & (self.tries < self.p.tries)

    # -- what the robot sees and where it is ---------------------------------------------------------------------------
    def update(self, pose, P: np.ndarray, h: np.ndarray | None, cam_xy, dets=()):
        """A depth look: P (n x 3) base_link points, h their heights over the floor (floor_vision.detect_points; None:
        their z), cam_xy the camera's x, y in base_link, dets floor_vision's detections in it (base_link), the base at
        pose (x, y, yaw) odom."""
        if len(P) == 0:
            return
        h = P[:, 2] if h is None else h
        c, s = math.cos(pose[2]), math.sin(pose[2])
        x = pose[0] + c * P[:, 0] - s * P[:, 1]
        y = pose[1] + s * P[:, 0] + c * P[:, 1]
        i, j = np.rint((x - self.x0) / CELL).astype(int), np.rint((y - self.y0) / CELL).astype(int)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        idx = i[ok] * self.n + j[ok]
        hh = h[ok]
        near = np.hypot(P[ok, 0] - cam_xy[0], P[ok, 1] - cam_xy[1]) < self.p.check_range
        raised = (hh > THING_H[0]) & (hh < THING_H[1])
        for d in dets:                                # a detection's own points are made out
            raised &= np.hypot(P[ok, 0] - d.x, P[ok, 1] - d.y) > self.p.thing_merge

        def count(sel):
            return np.bincount(idx[sel], minlength=self.n * self.n).reshape(self.n, self.n)
        high = count((hh >= self.p.obs_h) & (hh < 2.0))
        low = count(hh < self.p.obs_h)
        n_raised, n_all = count(raised), count(np.ones(len(hh), bool))
        thing = (n_raised >= self.p.thing_points) & (n_raised >= self.p.thing_share * n_all)
        floor = (low >= self.p.min_points) & (high == 0) & ~thing
        self.thing = (self.thing & ~floor) | thing
        self.checked = (self.checked & ~thing) | ((count(near) >= self.p.min_points) & ~self.thing)
        self.blocked = (self.blocked & ~floor) | (high >= 2)
        self.free = (self.free | floor) & ~self.blocked

    def occupy(self, x: float, y: float, r: float | None = None):
        """The base is here: the floor under it is free and checked (nothing to pick up under the robot); r: free this
        far round it (where the tidy starts: it got there)."""
        ij = self.cell(x, y)
        if ij is None or (r is None and ij == self._last):
            return
        self._last = ij
        m = np.zeros((self.n, self.n), bool)
        m[ij] = True
        under = _dilate(m, _disk(self.p.robot_radius / CELL))
        self.checked |= under
        self.free |= (under if r is None else _dilate(m, _disk(r / CELL))) & ~self.blocked

    def end_look(self, x: float, y: float, near: float, far: float):
        """A look round from (x, y) is over: the cells in its ring and in sight it didn't check count a try."""
        self.tries[self._visible(x, y, self._ring(x, y, near, far) & self.inside & ~self.checked)] += 1

    def _visible(self, x: float, y: float, mask: np.ndarray) -> np.ndarray:
        """The cells of mask with no blocked cell on the straight line from (x, y) to them (cells never seen don't
        hide anything: they are hoped to be floor)."""
        out = np.zeros_like(mask)
        ii, jj = np.nonzero(mask)
        if not len(ii):
            return out
        ci, cj = (x - self.x0) / CELL, (y - self.y0) / CELL
        di, dj = ii - ci, jj - cj
        stop = 1.0 - 0.7 / np.maximum(np.hypot(di, dj), 1.0)          # short of the cell itself
        t = np.linspace(0.0, 1.0, 40)[None, :] * stop[:, None]
        pi = np.clip(np.rint(ci + di[:, None] * t).astype(int), 0, self.n - 1)
        pj = np.clip(np.rint(cj + dj[:, None] * t).astype(int), 0, self.n - 1)
        out[ii, jj] = ~self.blocked[pi, pj].any(axis=1)
        return out

    # -- the region ------------------------------------------------------------------------------------------------------
    def todo_near(self, x: float, y: float, near: float, far: float) -> bool:
        return bool(np.any(self._visible(x, y, self.todo() & self._ring(x, y, near, far))))

    def coverage(self) -> dict:
        """m^2 of the region: checked, not checked (when done: hidden or out of reach), of that still to look for,
        and blocked; the checked share of the floor that isn't blocked."""
        a = CELL * CELL
        floor = self.floor()
        return dict(checked=round(float((floor & self.checked).sum()) * a, 2),
                    unchecked=round(float((floor & ~self.checked).sum()) * a, 2), todo=round(float(self.todo().sum()) * a, 2),
                    blocked=round(float((self.inside & self.blocked).sum()) * a, 2),
                    share=round(float((floor & self.checked).sum()) / max(1, int(floor.sum())), 3))

    def traversable(self) -> np.ndarray:
        """Cells the base's centre may be on: all of its radius + clear seen as free floor."""
        return ~_dilate(~self.free | self.blocked, self._robot)

    def plan(self, pose, near: float, far: float) -> Plan | None:
        """The best next look (see the module notes), or None: nothing reachable left worth a look."""
        start = self.cell(pose[0], pose[1])
        if start is None:
            return None
        trav = self.traversable()
        esc = np.hypot(self.X - pose[0], self.Y - pose[1]) <= self.p.escape
        ok = trav | (esc & ~self.blocked)
        dist, parent = self._dijkstra(start, ok)
        k = int(math.ceil(far / CELL))
        kk = np.mgrid[-k:k + 1, -k:k + 1] * CELL
        rr = np.hypot(kk[0], kk[1])
        todo = self.todo()
        gain = _conv(todo.astype(float), ((rr >= near) & (rr <= far)).astype(float)) * CELL * CELL
        reach = np.isfinite(dist) & trav & (np.hypot(self.X - self.centre[0], self.Y - self.centre[1]) <= self.radius)
        # the ring's floor in sight from each spot, best first: the ring's whole floor bounds it (branch and bound)
        bound = np.where(reach & (gain >= self.p.min_gain), gain / (dist / self.p.v + self.p.look_s), -1.0)
        best, best_u, best_g = None, 0.0, 0.0
        for k, idx in enumerate(np.argsort(-bound, axis=None)):
            i, j = divmod(int(idx), self.n)
            if bound[i, j] <= best_u or k >= 400:
                break
            g = float(self._visible(self.X[i, j], self.Y[i, j], todo & self._ring(self.X[i, j], self.Y[i, j], near, far))
                      .sum()) * CELL * CELL
            u = g / (dist[i, j] / self.p.v + self.p.look_s)
            if g >= self.p.min_gain and u > best_u:
                best, best_u, best_g = (i, j), u, g
        if best is None:
            return None
        cells = [best]
        while cells[-1] != start:
            cells.append(parent[cells[-1]])
        cells.reverse()
        path = [(self.X[c], self.Y[c]) for c in self._smooth(cells, ok)]
        return Plan((float(self.X[best]), float(self.Y[best])), [(float(a), float(b)) for a, b in path[1:]] or
                    [(float(self.X[best]), float(self.Y[best]))], best_g, float(dist[best]))

    # -- search ----------------------------------------------------------------------------------------------------------
    def _dijkstra(self, start, ok):
        dist = np.full((self.n, self.n), np.inf)
        parent = {}
        dist[start] = 0.0
        heap = [(0.0, start)]
        steps = [(a, b, CELL * math.hypot(a, b)) for a in (-1, 0, 1) for b in (-1, 0, 1) if a or b]
        while heap:
            d, (i, j) = heapq.heappop(heap)
            if d > dist[i, j]:
                continue
            for a, b, w in steps:
                u, v = i + a, j + b
                if 0 <= u < self.n and 0 <= v < self.n and ok[u, v] and d + w < dist[u, v]:
                    dist[u, v] = d + w
                    parent[(u, v)] = (i, j)
                    heapq.heappush(heap, (d + w, (u, v)))
        return dist, parent

    def _line(self, a, b, ok) -> bool:
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
        for t in np.linspace(0.0, 1.0, 2 * n + 1):
            if not ok[int(round(a[0] + (b[0] - a[0]) * t)), int(round(a[1] + (b[1] - a[1]) * t))]:
                return False
        return True

    def _smooth(self, cells, ok) -> list:
        """The path's cells cut to the fewest straight legs through ok cells."""
        out, k = [cells[0]], 0
        while k < len(cells) - 1:
            m = len(cells) - 1
            while m > k + 1 and not self._line(cells[k], cells[m], ok):
                m -= 1
            out.append(cells[m])
            k = m
        return out
