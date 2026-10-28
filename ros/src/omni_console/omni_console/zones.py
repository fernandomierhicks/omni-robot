"""No-go zones and floor edges drawn on the map (the console's Map edit tab), ROS-free.

  zone  a polygon the robot must not enter: Nav2's keepout filter (its costmaps treat it as a wall), so Go To,
        Explore, Patrol and Program's goto plan around it
  edge  a rug edge or a threshold, as a line with its height: a slow band either side of it (design.md 2: <= 1 m/s
        over mapped thresholds). Nav2's speed filter limits its controller there, and the console caps the stick in
        Manual (and Follow / Program) to the edge's speed while the robot is inside the band. Kept with the height for
        the terrain assist (design.md 7: "where the edge map comes from: app markup")

Both are kept per map in <map>.places.json, in the map's frame. masks() turns them into the two OccupancyGrids Nav2's
filters read (the map's own geometry): keepout 100 inside a zone, else 0; speed in cm/s inside a band (absolute
speed filter: limit = 0.01 * value m/s), 0 = no limit.
"""

from __future__ import annotations

import math

import numpy as np

EDGE_BAND = 0.35                 # m either side of an edge line where it's slow (the robot's radius + a margin)
EDGE_SPEED = 1.0                 # m/s by default (design.md 2)
KINDS = ("rug", "threshold", "step", "other")


def check_zone(points) -> list:
    pts = [[float(p[0]), float(p[1])] for p in points]
    if len(pts) < 3 or not all(math.isfinite(v) for p in pts for v in p):
        raise ValueError("a no-go zone needs at least three corners")
    return pts


def check_edge(points, height_mm=10.0, speed=EDGE_SPEED, kind="rug") -> dict:
    pts = [[float(p[0]), float(p[1])] for p in points]
    if len(pts) < 2 or not all(math.isfinite(v) for p in pts for v in p):
        raise ValueError("an edge needs at least two points")
    h, v = float(height_mm), float(speed)
    if not 0 <= h <= 60:
        raise ValueError("edge height: 0 to 60 mm")
    if not 0.1 <= v <= 2.0:
        raise ValueError("edge speed: 0.1 to 2 m/s")
    if kind not in KINDS:
        raise ValueError(f"edge kind: one of {', '.join(KINDS)}")
    return dict(points=pts, height_mm=h, speed=v, kind=kind)


def inside(poly, x: float, y: float) -> bool:
    """Even-odd rule."""
    n, c = len(poly), False
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            c = not c
    return c


def dist_to_line(pts, x: float, y: float) -> float:
    best = math.inf
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
        best = min(best, math.hypot(x - (x0 + t * dx), y - (y0 + t * dy)))
    return best


def speed_cap(edges, x: float, y: float):
    """The lowest edge speed at (x, y) (map frame), with its edge, or (None, None) outside every band."""
    cap, which = None, None
    for e in edges:
        if dist_to_line(e["points"], x, y) <= EDGE_BAND and (cap is None or e["speed"] < cap):
            cap, which = e["speed"], e
    return cap, which


def covers(mask_geom, map_geom) -> bool:
    """Whether masks made on one grid (width, height, resolution, origin x, origin y) still serve a map on another:
    the same resolution and the map's extent inside the masks' (a cell's slack). A Nav2 filter mask is its own grid in
    the map frame; it needn't match the map's cell for cell."""
    if mask_geom is None or map_geom is None:
        return False
    w0, h0, r0, x0, y0 = mask_geom
    w1, h1, r1, x1, y1 = map_geom
    if abs(r0 - r1) > 1e-6:
        return False
    return (x1 >= x0 - r0 and y1 >= y0 - r0 and x1 + w1 * r1 <= x0 + w0 * r0 + r0
            and y1 + h1 * r1 <= y0 + h0 * r0 + r0)


def masks(zones: dict, edges: dict, width: int, height: int, res: float, ox: float, oy: float):
    """(keepout int8[h, w], speed int8[h, w]) on the map's grid (row 0 at the origin, as OccupancyGrid.data)."""
    xs = ox + (np.arange(width) + 0.5) * res
    ys = oy + (np.arange(height) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    keep = np.zeros((height, width), dtype=np.int8)
    for poly in zones.values():
        p = np.asarray(poly)
        x0, y0 = p.min(axis=0)
        x1, y1 = p.max(axis=0)
        c0, c1 = max(0, int((x0 - ox) / res)), min(width, int((x1 - ox) / res) + 1)
        r0, r1 = max(0, int((y0 - oy) / res)), min(height, int((y1 - oy) / res) + 1)
        if c1 <= c0 or r1 <= r0:
            continue
        sx, sy = X[r0:r1, c0:c1], Y[r0:r1, c0:c1]
        ins = np.zeros(sx.shape, dtype=bool)
        n = len(p)
        for i in range(n):
            (ax, ay), (bx, by) = p[i], p[(i + 1) % n]
            if ay == by:
                continue
            cross = ((ay > sy) != (by > sy)) & (sx < ax + (sy - ay) * (bx - ax) / (by - ay))
            ins ^= cross
        keep[r0:r1, c0:c1][ins] = 100
    speed = np.zeros((height, width), dtype=np.int16)
    for e in edges.values():
        pts = np.asarray(e["points"])
        v = int(round(100 * e["speed"]))
        x0, y0 = pts.min(axis=0) - EDGE_BAND
        x1, y1 = pts.max(axis=0) + EDGE_BAND
        c0, c1 = max(0, int((x0 - ox) / res)), min(width, int((x1 - ox) / res) + 1)
        r0, r1 = max(0, int((y0 - oy) / res)), min(height, int((y1 - oy) / res) + 1)
        if c1 <= c0 or r1 <= r0:
            continue
        sx, sy = X[r0:r1, c0:c1], Y[r0:r1, c0:c1]
        d = np.full(sx.shape, np.inf)
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = np.zeros(sx.shape) if L2 == 0 else np.clip(((sx - ax) * dx + (sy - ay) * dy) / L2, 0.0, 1.0)
            d = np.minimum(d, np.hypot(sx - (ax + t * dx), sy - (ay + t * dy)))
        band = d <= EDGE_BAND
        cur = speed[r0:r1, c0:c1]
        cur[band] = np.where(cur[band] == 0, v, np.minimum(cur[band], v))
    return keep, np.clip(speed, 0, 100).astype(np.int8)
