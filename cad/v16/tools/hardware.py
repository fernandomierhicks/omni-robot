"""cad/v11 fasteners as records, so that every screw is counted, checked and placed from one table (Nov 4 2026).

A record: dict(kind, size, L, p, d, name): p = the head's bearing point (where the head sits on the part), d = unit vector along the shank
(into the parts), L = shank length (mm). kind: 'pan' (hex-drive shallow pan head; M3 6.5 x 2.5, M2.5 5 x 2, M4 the ISO 7380 button head),
'flat' (90 deg countersunk, hex drive, the head below p), 'dowel' (p = one end, along d), 'nut' (M3 nyloc, p = its bearing face, d out of the
part), 'hexnut' (plain M3 hex nut, 5.5 AF x 2.4, as 'nut').
solid(rec) is the build123d body (for the checks and the STEPs); pose(rec) the 4 x 4 that puts the library part (origin at p, its shank
along -z, the head above z 0) where the record says; the library part per (kind, size, L) is part_solid(): every head has its hex socket
(round 13: the flat heads had none, so their heads read as plain discs in SolidWorks).
Also: from_solid() reads a head + shank solid built elsewhere (chassis.py / pod_lean.py hardware) back into a record; audit() (round 13) checks
every screw's holes: a hole that is not a full circle in a part (it straddles a split or runs off an edge), and a thread-forming pilot whose
axis lies in the part's layer plane (the screw wedges the layers apart).
"""
import math

import numpy as np
import build123d as b

MAGENTA = (0.85, 0.0, 0.85)
PAN_HEAD = {3.0: (6.5, 2.5), 2.5: (5.0, 2.0), 4.0: (7.6, 2.2)}       # M4: ISO 7380 button head (the spring plungers)
FLAT_HEAD = {3.0: 6.0, 4.0: 8.0}


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
    if k == 'hexnut':
        nut = b.extrude(_plane(p, d) * b.RegularPolygon(HEXNUT[0] / math.sqrt(3), 6), amount=HEXNUT[1])
        return nut - b.Solid.make_cylinder(1.5, HEXNUT[1] + 0.2, _plane(p - 0.1 * d, d))
    raise ValueError(k)


HEXNUT = (5.5, 2.4)                                               # M3 hex nut (ISO 4032): across flats, thickness


# ---- STEP models of bought hardware (Nov 13 evening, Fernando: assembly STEPs show bearings and fasteners, magenta; cad/solidworks.md 15) ---------
def bearing_model(d, D, B, recess=0.25):
    """a ZZ deep-groove ball bearing for the STEPs (axis z, centred): inner and outer rings full width, the shields between them recessed `recess`
    from each face. The rings are 0.3 of the section each (nominal; the checks use the plain envelope)"""
    t = (D - d) / 2
    ri, ro = d / 2 + 0.3 * t, D / 2 - 0.3 * t

    def ring(r0, r1, h):
        pl = b.Plane.XY.offset(-h / 2)
        return b.Solid.make_cylinder(r1, h, pl) - b.Solid.make_cylinder(r0, h + 0.2, b.Plane.XY.offset(-h / 2 - 0.1))
    return ring(d / 2, ri, B) + ring(ro, D / 2, B) + ring(ri - 0.01, ro + 0.01, B - 2 * recess)


def detail_bearing(env, recess=0.25):
    """the bearing_model() in the pose of a plain bearing envelope (an annulus: two coaxial cylinders); returns env unchanged if it is not one"""
    cyls = []
    for f in env.faces():
        if f.geom_type.name == 'CYLINDER':
            try:
                cyls.append((round(f.radius, 4), f.axis_of_rotation))
            except Exception:                                         # noqa: BLE001
                return env
    rs = sorted({r for r, _ in cyls})
    if len(rs) != 2:
        return env
    a = np.array(tuple(cyls[0][1].direction), float)
    a /= np.linalg.norm(a)
    c = np.array(tuple(env.center()), float)                          # the annulus' centre of mass: on its axis, mid-width
    t = (np.array([tuple(v) for v in env.vertices()]) - c) @ a
    return bearing_model(2 * rs[0], 2 * rs[1], float(t.max() - t.min()), recess).moved(b.Location(b.Plane(origin=tuple(c), z_dir=tuple(a))))


def step_model(name, shape):
    """the shape a STEP assembly shows for a part name: a screw (bolt_*) as its library part with the hex socket (read back with from_solid), a plain
    bearing envelope (bearing_*) as bearing_model(); anything else unchanged"""
    if name.startswith('bolt_'):
        if len(shape.faces()) > 8:                                    # already a real model (e.g. the wheel's reference/bolt_M3_50_PAN.step)
            return shape
        rec = from_solid(shape, name)
        if rec and ((rec['kind'] == 'pan' and rec['size'] in PAN_HEAD) or (rec['kind'] == 'flat' and rec['size'] in FLAT_HEAD)):
            return part_solid(rec).moved(b.Location(b.Plane(origin=tuple(float(v) for v in rec['p']), x_dir=tuple(pose(rec)[:3, 0]),
                                                            z_dir=tuple(pose(rec)[:3, 2]))))
        return shape
    if name.startswith('bearing_'):
        return detail_bearing(shape)
    return shape


STEP_HARDWARE = ('bolt_', 'bearing_', 'washer', 'spring_plunger', 'nut_')   # magenta in the STEP assemblies
NUT_RECESS = (5.5, 2.5)                                           # Nov 13 (Fernando): the hex recess that holds a nut on a part's outer face


def hex_recess(face_pt, into, depth=NUT_RECESS[1], af=NUT_RECESS[0]):
    """Nov 13 (Fernando: no captive nuts): the cut for a hex recess on a part's face. face_pt: the hole's centre on the face; into: unit vector
    into the part. On a vertical face a corner points up (+z), so the recess roof slopes at 60 deg and prints without support. 0.1 proud of the
    face so the cut is clean. The nut's bearing face is then at face_pt + into * depth"""
    d = np.asarray(into, float)
    d = d / np.linalg.norm(d)
    up = np.array([0.0, 0, 1.0]) - d * d[2]
    xd = up / np.linalg.norm(up) if np.linalg.norm(up) > 1e-6 else np.array([1.0, 0, 0])
    pl = b.Plane(origin=tuple(np.asarray(face_pt, float) - 0.1 * d), x_dir=tuple(xd), z_dir=tuple(d))
    return b.extrude(pl * b.RegularPolygon(af / math.sqrt(3), 6), amount=depth + 0.1)


def part_name(rec):
    k, s, L = rec['kind'], rec['size'], rec['L']
    sz = f'M{s:g}'.replace('.', 'p')
    return {'pan': f'{sz}x{L:g}_pan', 'flat': f'{sz}x{L:g}_flat', 'dowel': f'dowel_{s:g}x{L:g}', 'nut': f'{sz}_nyloc', 'hexnut': f'{sz}_hexnut'}[k]


def part_solid(rec):
    """the library part: origin at the bearing point, shank along -z (head above z 0); pan and flat heads get a hex socket, 2 mm across
    flats (2.5 for M4), 1.3 deep from the head's top"""
    r = dict(rec, p=(0, 0, 0), d=(0, 0, -1))
    s = solid(r)
    if rec['kind'] in ('pan', 'flat'):
        af = 2.5 if rec['size'] == 4.0 else 2.0
        top = PAN_HEAD[rec['size']][1] if rec['kind'] == 'pan' else 0.0
        hexa = b.extrude(b.Plane.XY.offset(top - 1.3) * b.RegularPolygon(af / math.sqrt(3), 6), amount=1.4)
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


# ---- hole audit (round 13) -----------------------------------------------------------------------------------------------------------
HOLE_R = {3.0: (1.42, 1.58, 1.7),   # M3 probe to r 1.58 (Nov 13 evening; was 1.68): plastic a snug Ø3.2 hole (r 1.6) leaves is not thread
 2.5: (1.05, 1.32, 1.45), 4.0: (1.7, 2.15, 2.25)}   # size: thread probe r in / out, clearance hole r
STATION, PARTIAL = 1.0, (0.15, 0.9)                              # ring slices every mm; a slice between these fractions of a full ring is
                                                                 # partial (under 0.15: a nut slot or a pocket round the hole, by design)


def _bbox_near(a, c, gap=0.0):
    p, q = a.bounding_box(), c.bounding_box()
    return all(tuple(p.min)[i] - gap <= tuple(q.max)[i] and tuple(q.min)[i] - gap <= tuple(p.max)[i] for i in range(3))


def audit(recs, parts, up_of=lambda name: None, layer_limit_deg=45.0):
    """every pan / flat screw against every part its shank passes:
     * thread: engagement in the part (shank length with plastic between the pilot and clearance radii);
     * partial: the wall round the hole (a ring 0.3..1.3 mm outside the clearance radius) sliced every mm along the shank; a slice that is
       neither empty (a counterbore, a gap between parts) nor full means the hole is a half circle: it straddles a split between two parts or
       runs off an edge;
     * across_layers: a thread-forming pilot (engagement > 1 mm) whose axis is more than layer_limit_deg from the part's print-up axis
       (up_of(name): unit vector in the part's frame, None for bought / vendor parts): the screw's wedging opens the layers. Round 14: an M3
       in a Ø2.9 pilot goes to soft_across_layers instead (allowed: Fernando, gentle enough not to delaminate);
     * head: the head's volume inside a part (a head buried in a wall, or no counterbore where it sits flush);
     * no_hole: the shank's core (r 0.8 x the thread) inside a part: a part the screw passes with no hole cut for it.
    Returns dict(thread={screw: {part: mm}}, partial={screw / part: min fraction}, across_layers={screw / part: deg off the up axis},
    head={screw / part: mm3})"""
    out = dict(thread={}, partial={}, across_layers={}, soft_across_layers={}, head={}, no_hole={})
    for n, r in enumerate(recs):
        if r['kind'] not in ('pan', 'flat') or r['size'] not in HOLE_R:
            continue
        p, d = np.asarray(r['p'], float), np.asarray(r['d'], float)
        d = d / np.linalg.norm(d)
        ri, ro, rc = HOLE_R[r['size']]
        L = r['L']
        key = f"{r['name']}#{n}"
        probe = b.Solid.make_cylinder(ro, L, _plane(p, d)) - b.Solid.make_cylinder(ri, L + 0.2, _plane(p - 0.1 * d, d))
        wall = b.Solid.make_cylinder(rc + 1.3, L, _plane(p, d)) - b.Solid.make_cylinder(rc + 0.3, L + 0.2, _plane(p - 0.1 * d, d))
        full = math.pi * ((rc + 1.3) ** 2 - (rc + 0.3) ** 2) * 0.4
        skip = 2.0 if r['kind'] == 'flat' else 0.0                   # the countersink's cone
        head = None
        if r['kind'] == 'pan':                                         # its head, 0.05 inside its real size (a head sitting on a face is fine)
            hd, hh = PAN_HEAD[r['size']]
            head = b.Solid.make_cylinder(hd / 2 - 0.05, hh - 0.05, _plane(p - (hh - 0.05) * d, d))
        core = b.Solid.make_cylinder(0.4 * r['size'], L - 0.5, _plane(p + 0.25 * d, d))
        reach = wall if head is None else b.Compound([wall, head])
        for name, s in parts.items():
            if not _bbox_near(reach, s):
                continue
            x = probe & s
            eng = 0.0 if x is None else x.volume / (math.pi * (ro ** 2 - ri ** 2))
            if eng > 0.05:
                out['thread'].setdefault(key, {})[name] = round(eng, 1)
            cx = core & s
            if cx is not None and cx.volume > 0.5:
                out['no_hole'][f'{key} / {name}'] = round(cx.volume, 1)
            if head is not None and _bbox_near(head, s):
                hx = head & s
                if hx is not None and hx.volume > 0.5:
                    out['head'][f'{key} / {name}'] = round(hx.volume, 1)
            up = up_of(name)
            if eng > 1.0 and up is not None:
                ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(d, up)) / np.linalg.norm(up)))))
                if ang > layer_limit_deg:
                    soft = _soft(p, d, L, s, eng) if r['size'] == 3.0 else False
                    out['soft_across_layers' if soft else 'across_layers'][f'{key} / {name}'] = round(ang, 1)
            w = wall & s if up is not None else None                  # the half-hole check on our printed parts only (vendor holes are theirs)
            if w is None or w.volume < 1e-3:
                continue
            fr = []
            for t in np.arange(skip + 0.5, L - 0.3, STATION):
                sl = w & b.Solid.make_cylinder(rc + 2.0, 0.4, _plane(p + d * (t - 0.2), d))
                if sl is None or sl.volume < 1e-6:
                    continue
                ts = [float((np.array(tuple(v)) - p) @ d) for v in sl.vertices()]
                ext = max(ts) - min(ts) if ts else 0.4                 # a slice through a part's end face is thinner, not partial
                f = sl.volume / (full * max(ext, 0.05) / 0.4)
                if PARTIAL[0] < f < PARTIAL[1]:
                    fr.append(f)
            if len(fr) >= 2:
                out['partial'][f'{key} / {name}'] = round(min(fr), 2)
    return out


def _inside(s, pt):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN
    return BRepClass3d_SolidClassifier(s.wrapped, gp_Pnt(*[float(v) for v in pt]), 1e-6).State() == TopAbs_IN


def blind_holes(recs, parts, up_of=lambda name: None, reach=80.0, step=0.25):
    """Nov 13 (Fernando: every hole through the whole part, so support can be pushed out). For every pan / flat screw and every printed part
    (up_of(name) not None) its thread ends in: walk on along the screw's axis from its tip. Where the axis meets plastic again before it
    leaves the part, the hole is blind: returns {screw#n / part: dict(bottom=mm past the tip, solid=mm of plastic on the axis to the far
    face or a cavity, exit=(x, y, z) where it comes out, d=axis, start=(x, y, z) the hole's floor)}. solid None: no exit within reach"""
    out = {}
    for n, r in enumerate(recs):
        if r['kind'] not in ('pan', 'flat') or r['size'] not in HOLE_R:
            continue
        p, d = np.asarray(r['p'], float), np.asarray(r['d'], float)
        d = d / np.linalg.norm(d)
        tip = p + d * r['L']
        for name, s in parts.items():
            if up_of(name) is None:
                continue
            bb = s.bounding_box()
            if not all(tuple(bb.min)[i] - 0.5 <= tip[i] <= tuple(bb.max)[i] + 0.5 for i in range(3)):
                continue
            # the screw must thread (or end) in this part: plastic round its last 2 mm
            ring = b.Solid.make_cylinder(1.68, 2.0, _plane(tip - 2.0 * d, d)) - b.Solid.make_cylinder(1.3, 2.2, _plane(tip - 2.1 * d, d))
            x = ring & s
            if x is None or x.volume < 0.5:
                continue
            t, bottom, exit_ = 0.0, None, None
            while t < reach:
                q = tip + d * t
                ins = _inside(s, q)
                if bottom is None and ins:
                    bottom = t
                elif bottom is not None and not ins:
                    exit_ = t
                    break
                if bottom is None and not all(tuple(bb.min)[i] - 0.1 <= q[i] <= tuple(bb.max)[i] + 0.1 for i in range(3)):
                    break                                              # out of the part with no floor: through
                t += step
            if bottom is not None:
                out[f"{r['name']}#{n} / {name}"] = dict(bottom=round(bottom, 2), solid=None if exit_ is None else round(exit_ - bottom, 2),
                                                        start=tuple(float(v) for v in tip + d * bottom),     # exact (on the screw's own axis:
                                                        exit=None if exit_ is None else tuple(float(v) for v in tip + d * exit_),   # a rounded
                                                        d=tuple(float(v) for v in d))   # start left 0.001 mm crescents that OCC cannot mesh)
    return out


THROUGH_MAX = 20.0                                                # drill_through: deepest run of plastic it drills on its own; deeper: listed


def bed_up_of():
    """up_of(name) from print_beds (None for parts with no bed: bought, vendor)"""
    import print_beds as PB
    return lambda n: (up_axis(PB.bed(n.split('/')[-1])[0]) if PB.bed(n.split('/')[-1]) else None)


def make_through(parts, recs, up_of=None, max_solid=THROUGH_MAX):
    """Nov 13: every screw's hole through its printed part: find the blind ones and drill those with at most max_solid mm of plastic left on
    their axis; returns (parts, dict(drilled=[...], left={key: mm of plastic}))"""
    up_of = up_of or bed_up_of()
    bl = blind_holes(recs, parts, up_of)
    parts, drilled, left = drill_through(parts, bl, max_solid)
    return parts, dict(drilled=drilled, left={k: bl[k]['solid'] for k in left})


def drill_through(parts, blind, max_solid=THROUGH_MAX, d_hole=2.8):
    """run each blind hole found by blind_holes() on out of the part as a Ø d_hole hole, where at most max_solid mm of plastic is left on its
    axis; returns (parts, drilled keys, left keys: too deep to drill, for the exceptions list)"""
    drilled, left = [], []
    for key, h in blind.items():
        name = key.split(' / ', 1)[1]
        if h['solid'] is None or h['solid'] > max_solid:
            left.append(key)
            continue
        d = np.asarray(h['d'])
        a = np.asarray(h['start']) - 0.3 * d
        L = h['solid'] + 0.8
        parts[name] = parts[name] - b.Solid.make_cylinder(d_hole / 2, L, _plane(a, d))
        drilled.append(key)
    return parts, drilled, left


def _soft(p, d, L, s, eng):
    """round 14: is the M3's pilot in s a Ø2.9 one (Fernando: allowed across the layers)? No plastic at r 1.405..1.44 along the engagement
    (a Ø2.8 pilot has plastic there)"""
    ring = b.Solid.make_cylinder(1.44, L, _plane(p, d)) - b.Solid.make_cylinder(1.405, L + 0.2, _plane(p - 0.1 * d, d))
    x = ring & s
    return (0.0 if x is None else x.volume) < 0.1 * eng * math.pi * (1.44 ** 2 - 1.405 ** 2)


def up_axis(axis):
    """'+z' / '-x' ... (print_beds.py) -> unit vector; the up direction's sign does not matter for the layer check"""
    v = np.zeros(3)
    v['xyz'.index(axis[1])] = 1.0
    return v


def pan_screw(p, axis, length):
    """an M3 hex-drive pan head (6.5 x 2.5) + shank as one solid: head on the surface at p, shank along the unit `axis` (into the parts)"""
    ax = np.asarray(axis, float)
    head = b.Solid.make_cylinder(3.25, 2.5, b.Plane(origin=tuple(np.asarray(p) - 2.5 * ax), z_dir=tuple(ax)))
    shank = b.Solid.make_cylinder(1.5, length, b.Plane(origin=tuple(p), z_dir=tuple(ax)))
    return head + shank
