"""cad/v11 fasteners as records, so that every screw is counted, checked and placed from one table (Nov 4 2026).

A record: dict(kind, size, L, p, d, name): p = the head's bearing point (where the head sits on the part), d = unit vector along the shank
(into the parts), L = shank length (mm). kind: 'pan' (hex-drive shallow pan head; M3 6.5 x 2.5, M2.5 5 x 2), 'flat' (90 deg countersunk,
the head below p), 'dowel' (p = one end, along d), 'nut' (M3 nyloc, p = its bearing face, d out of the part).
solid(rec) is the build123d body (for the checks and the STEPs); pose(rec) the 4 x 4 that puts the library part (origin at p, its shank
along -z, the head above z 0) where the record says; the library part per (kind, size, L) is part_solid().
Also: from_solid() reads a head + shank solid built elsewhere (chassis.py / pod_lean.py hardware) back into a record.
"""
import math

import numpy as np
import build123d as b

MAGENTA = (0.85, 0.0, 0.85)
PAN_HEAD = {3.0: (6.5, 2.5), 2.5: (5.0, 2.0), 4.0: (7.6, 2.2)}       # M4: ISO 7380 button head (the spring plungers)
FLAT_HEAD = {3.0: 6.0}


def _plane(p, d):
    return b.Plane(origin=tuple(float(v) for v in p), z_dir=tuple(float(v) for v in d))


def solid(rec):
    p, d = np.asarray(rec['p'], float), np.asarray(rec['d'], float)
    d = d / np.linalg.norm(d)
    k, s, L = rec['kind'], rec['size'], rec['L']
    if k == 'pan':
        hd, hh = PAN_HEAD[s]
        head = b.Solid.make_cylinder(hd / 2, hh, _plane(p - hh * d, d))
        return head + b.Solid.make_cylinder(s / 2, L, _plane(p, d))
    if k == 'flat':
        cd = FLAT_HEAD[s]
        cone = b.Solid.make_cone(cd / 2, s / 2, (cd - s) / 2, _plane(p, d))
        return cone + b.Solid.make_cylinder(s / 2, L, _plane(p, d))
    if k == 'dowel':
        return b.Solid.make_cylinder(s / 2, L, _plane(p, d))
    if k == 'nut':
        return b.Solid.make_cylinder(3.15, 4.0, _plane(p, d)) - b.Solid.make_cylinder(1.5, 4.2, _plane(p - 0.1 * d, d))
    raise ValueError(k)


def part_name(rec):
    k, s, L = rec['kind'], rec['size'], rec['L']
    sz = f'M{s:g}'.replace('.', 'p')
    return {'pan': f'{sz}x{L:g}_pan', 'flat': f'{sz}x{L:g}_flat', 'dowel': f'dowel_{s:g}x{L:g}', 'nut': f'{sz}_nyloc'}[k]


def part_solid(rec):
    """the library part: origin at the bearing point, shank along -z (head above z 0)"""
    r = dict(rec, p=(0, 0, 0), d=(0, 0, -1))
    s = solid(r)
    if rec['kind'] == 'pan':                                      # hex socket, 2 mm across flats (2.5 for M4), 1.3 deep
        af = 2.5 if rec['size'] == 4.0 else 2.0
        hh = PAN_HEAD[rec['size']][1]
        hexa = b.extrude(b.Plane.XY.offset(hh - 1.3) * b.RegularPolygon(af / math.sqrt(3), 6), amount=1.4)
        s = s - hexa
    return s


def pose(rec):
    """4 x 4 placing the library part (shank -z) at the record"""
    d = np.asarray(rec['d'], float)
    z = -d / np.linalg.norm(d)
    a = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(a, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, rec['p']
    if rec['kind'] == 'dowel':                                    # the library dowel runs from z 0 down: p is its top end
        T[:3, 3] = np.asarray(rec['p'], float)
    return T


def pan(p, d, L, name, size=3.0):
    return dict(kind='pan', size=size, L=float(L), p=tuple(float(v) for v in p), d=tuple(float(v) for v in d), name=name)


def from_solid(s, name):
    """read a pan head (or flat head) + shank, or a plain dowel, built elsewhere as one solid: its axis from the cylinders, the head from the
    widest one. Returns a record or None"""
    cyls = []
    for f in s.faces():
        if f.geom_type.name == 'CYLINDER':
            ax = f.axis_of_rotation if hasattr(f, 'axis_of_rotation') else None
            try:
                r = f.radius
            except Exception:                                         # noqa: BLE001
                continue
            if ax is None:
                continue
            cyls.append((r, ax, f))
    cones = [f for f in s.faces() if f.geom_type.name == 'CONE']
    if not cyls:
        return None
    d = np.array(tuple(cyls[0][1].direction))
    d /= np.linalg.norm(d)
    pts = np.array([tuple(v) for v in s.vertices()]) if s.vertices() else None
    bb = s.bounding_box()
    cen = np.array(tuple(bb.center()))
    o = np.array(tuple(cyls[0][1].position))
    o = o + d * ((cen - o) @ d)                                       # a point on the axis near the middle
    # extent along the axis
    tv = []
    for f in s.faces():
        for v in f.vertices():
            tv.append((np.array(tuple(v)) - o) @ d)
    t0, t1 = min(tv), max(tv)
    rs = sorted({round(r, 2) for r, _, _ in cyls})
    shank_r = rs[0]
    if len(rs) == 1 and not cones:                                # a dowel
        return dict(kind='dowel', size=round(2 * shank_r, 2), L=round(t1 - t0, 2), p=tuple(o + d * t1), d=tuple(-d), name=name)
    # the head's end: where the widest cylinder (or the cone) is
    if cones:
        tc = np.mean([(np.array(tuple(v)) - o) @ d for f in cones for v in f.vertices()])
        head_hi = tc > (t0 + t1) / 2
        kind, hh = 'flat', 0.0
    else:
        hf = [f for r, _, f in cyls if abs(r - rs[-1]) < 1e-2]
        th = np.mean([(np.array(tuple(v)) - o) @ d for f in hf for v in f.vertices()])
        head_hi = th > (t0 + t1) / 2
        kind = 'pan'
        hh = PAN_HEAD.get(round(2 * shank_r, 1), (0, 2.5))[1]
    size = round(2 * shank_r, 1)
    if head_hi:                                                   # head at t1: the shank runs toward -d
        p = o + d * (t1 - hh) if kind == 'pan' else o + d * t1
        sd = -d
        L = (t1 - hh) - t0 if kind == 'pan' else t1 - t0
    else:
        p = o + d * (t0 + hh) if kind == 'pan' else o + d * t0
        sd = d
        L = t1 - (t0 + hh) if kind == 'pan' else t1 - t0
    return dict(kind=kind, size=size, L=round(float(L), 1), p=tuple(float(v) for v in p), d=tuple(float(v) for v in sd), name=name)
