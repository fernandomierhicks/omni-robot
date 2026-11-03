"""Nov 2 2026: compact wheel + motor, STEP first (build123d), for Fernando's review before anything goes into SolidWorks.

    python -B 70_wheel_motor_v2.py [build|check|render]          (omni-robot-sim interpreter)

What changes against the v10 wheel (pod frame, x = wheel axis, rows at x = -18 / +18, roller axes at r 52 -- all unchanged):
 * ONE printed hub replaces hubs A/B/C: drum + two rows of six spokes with their axle blocks. Rollers go in after printing.
 * Each roller axle is an M3 x 50 into a blind 2.8 mm pilot (thread-forming, no nut). Head side: Ø3.4 hole + Ø6.4 head pocket. Odd rollers take the
   bolt from their +tangent end, even rollers from -tangent, so the two bolts that share a spoke come in through the same open "B-B" spoke and a
   Ø6.4 tunnel from each head pocket outward is the whole insertion path.
 * Width: the removable cartridge is now a flush plate with a boss that carries the left 6804 inside the hub; the right 6804 sits inside hub end.
   End faces x -39.5 .. +37 (was -51 .. +44).
 * Motor cable: stator groove + cup slot + a groove in the cup stub, with the cable modelled (Ø4 assumed, VERIFY) so clearance can be read.
 * M2.5 clearance holes 2.9 -> 2.7 (hub flange, cup rear).
Everything is nominal CAD; nothing has been printed.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from OCP.gp import gp_Trsf

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'step_export' / 'wheel_motor_v2'
VAL = ROOT / 'validation' / 'wheel_motor_v2'
MAN = {r['key']: r for r in json.loads((ROOT / 'reference' / 'assembly_manifest.json').read_text())['parts']}

# ---- parameters ------------------------------------------------------------------------------------------------------
X_LEFT, X_RIGHT = -39.5, 37.0            # hub end faces
DRUM_IN, DRUM_OUT = 30.0, 33.0
PLATE_T = 3.0                            # cartridge plate, flush with the left end face
X_PLATE_FLOOR = X_LEFT + PLATE_T         # -36.5: counterbore floor in the hub
ACCESS_R = 18.0                          # left access bore (cartridge boss + driver access to the rotor screws)
FLANGE_FRONT, FLANGE_BACK = -26.5, -23.0 # hub flange plate that the rotor screws clamp (unchanged)
R_BRG = 16.0                             # 6804 outside radius
BRG_FIT = 0.05
ROW_X = (-18.0, 18.0)
R_AXIS = 52.0
BLOCK_R = 6.0
Z_FACE = 21.3                            # roller thrust faces, +-z about the roller centre
B_OUT = 25.8                             # head-side block outer face
A_OUT = 29.0                             # pilot-side block outer face
HEAD_POCKET_D, HEAD_POCKET_DEPTH = 6.4, 3.0
PILOT_D, PILOT_DEPTH = 2.8, 6.5
CLR_D = 3.4
POCKET_CLEAR = 0.95
TUNNEL_D = 6.4
WEB_T = 12.0
M25_CLEAR = 2.7                          # was 2.9
FLANGE_R, FLANGE_ANGLES = 13.5, (30, 150, 270)
CART_R = 26.0
CABLE_D = 4.0
CABLE_ANG = 45.0


def loc(M):
    t = gp_Trsf()
    t.SetValues(*[float(v) for r in np.asarray(M)[:3] for v in r])
    return b.Location(t)


def placed(step_stem, key, folder='validation/human'):
    s = b.import_step(ROOT / folder / (step_stem + '.step'))
    return s.moved(loc(np.array(MAN[key]['matrix'])))


def cyl(r, length, centre):
    """cylinder along local z, centred at `centre`"""
    return b.Cylinder(r, length).moved(b.Location(centre))


def along_x(r, x0, x1, y=0.0, z=0.0):
    return b.Cylinder(r, x1 - x0).rotate(b.Axis.Y, 90).moved(b.Location(((x0 + x1) / 2, y, z)))


def at_angle(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)             # (y, z)


def hull2d(points):
    pts = sorted(set((round(p[0], 6), round(p[1], 6)) for p in points))
    def cross(o, a, c):
        return (a[0] - o[0]) * (c[1] - o[1]) - (a[1] - o[1]) * (c[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


# ---- the single-piece hub ------------------------------------------------------------------------------------------------
def roller_frames():
    """[(row, k, 4x4 local->pod, flipped)] -- local z = roller axis (tangent), local y = radial, local x = pod x. Even rollers are turned 180 deg about local y."""
    out = []
    for row in (1, 2):
        for k in range(1, 7):
            M = np.array(MAN[f'Roller_{row}_{k}']['matrix'])
            flipped = (k % 2 == 0)
            if flipped:
                M = M @ np.diag([-1.0, 1.0, -1.0, 1.0])
            out.append((row, k, M, flipped))
    return out


def web(zc):
    """prism along local x: hull of the drum arc under the block and the block circle, centred on the row plane"""
    pts = [(BLOCK_R * math.cos(t) , zc + BLOCK_R * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 40, endpoint=False)]
    pts = [(y, z) for y, z in pts]
    for dz in (-10.0, -5.0, 0.0, 5.0, 10.0):
        z = max(-DRUM_OUT + 1, min(DRUM_OUT - 1, zc + dz))
        pts.append((-R_AXIS + math.sqrt(DRUM_OUT ** 2 - z ** 2) - 1.0, z))      # a little inside the drum so the union fuses
    poly = hull2d(pts)
    face = b.Plane.YZ * b.Polygon(*poly, align=None)
    return b.extrude(face, amount=WEB_T / 2, both=True)


def support_unit():
    """in the roller's local frame: both blocks and their webs (the head block is +z)"""
    zb = (Z_FACE + B_OUT) / 2
    za = -(Z_FACE + A_OUT) / 2
    s = cyl(BLOCK_R, B_OUT - Z_FACE, (0, 0, zb)) + cyl(BLOCK_R, A_OUT - Z_FACE, (0, 0, za))
    s = s + web(zb) + web(za)
    # nothing of the webs may reach into the drum bore: keep only what lies outside r 32.5 (the wall is r 30..33, so the union still fuses)
    return s - b.Cylinder(DRUM_OUT - 0.5, 120.0).rotate(b.Axis.Y, 90).moved(b.Location((0, -R_AXIS, 0)))


def pocket_tool():
    """barrel clearance around the roller, only between the thrust faces"""
    zs = np.linspace(-Z_FACE, Z_FACE, 61)
    rr = [math.sqrt(70.0 ** 2 - z ** 2) - R_AXIS + POCKET_CLEAR for z in zs]
    pts = [(0.0, zs[0])] + [(r, z) for r, z in zip(rr, zs)] + [(0.0, zs[-1])]
    face = b.Plane.XZ * b.Polygon(*pts, align=None)
    return b.revolve(face, axis=b.Axis.Z)


def build_hub():
    # drum, end walls, flange plate, bearing seat: one revolve about the pod x axis
    prof = [(X_LEFT, DRUM_IN + 0.2), (X_LEFT, DRUM_OUT), (X_RIGHT, DRUM_OUT), (X_RIGHT, R_BRG + 0.5), (X_RIGHT - 0.5, R_BRG + BRG_FIT),
            (30.0, R_BRG + BRG_FIT), (30.0, 13.5), (27.5, 13.5), (27.5, DRUM_IN), (FLANGE_BACK, DRUM_IN), (FLANGE_BACK, 0.0),
            (FLANGE_FRONT, 0.0), (FLANGE_FRONT, ACCESS_R), (X_PLATE_FLOOR, ACCESS_R), (X_PLATE_FLOOR, DRUM_IN + 0.2)]
    body = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    units = []
    unit = support_unit()
    pocket = pocket_tool()
    frames = roller_frames()
    for row, k, M, flipped in frames:
        units.append(unit.moved(loc(M)))
    hub = body
    for u in units:
        hub = hub + u
    for row, k, M, flipped in frames:                      # roller clearance
        hub = hub - pocket.moved(loc(M))
    for row, k, M, flipped in frames:
        # head side: through hole + head pocket; pilot side: blind pilot
        thru = cyl(CLR_D / 2, B_OUT - Z_FACE + 1.0, (0, 0, (Z_FACE + B_OUT) / 2 - 0.0))
        head = cyl(HEAD_POCKET_D / 2, HEAD_POCKET_DEPTH + 0.01, (0, 0, B_OUT - HEAD_POCKET_DEPTH / 2 + 0.005))
        tunnel = cyl(TUNNEL_D / 2, 60.0, (0, 0, B_OUT + 30.0))
        pilot = cyl(PILOT_D / 2, PILOT_DEPTH, (0, 0, -(Z_FACE + PILOT_DEPTH / 2) + 0.0))
        hub = hub - (thru + head + tunnel + pilot).moved(loc(M))
    # rotor flange screw holes (M2.5 clearance) and cartridge pilots
    for a in FLANGE_ANGLES:
        y, z = at_angle(FLANGE_R, a)
        hub = hub - along_x(M25_CLEAR / 2, FLANGE_FRONT - 0.1, FLANGE_BACK + 0.1, y, z)
        y, z = at_angle(CART_R, a)
        hub = hub - along_x(1.4, X_PLATE_FLOOR - 0.1, X_PLATE_FLOOR + 10.0, y, z)
    return hub


# ---- cartridge (flush plate + boss carrying the left 6804) -------------------------------------------------------------------
def build_cartridge():
    x0, x1 = X_LEFT, X_PLATE_FLOOR
    brg_front, brg_back = x1, x1 + 7.0
    floor_back = brg_back + 1.2
    prof = [(x0, 13.0), (x0, DRUM_IN - 0.3), (x1, DRUM_IN - 0.3), (x1, 17.6), (floor_back, 17.6), (floor_back, 13.5), (brg_back, 13.5),
            (brg_back, R_BRG + BRG_FIT), (brg_front, R_BRG + BRG_FIT), (brg_front, 13.0)]
    # plate lip overlaps the outer race (r 13 .. 16); the bearing sits in the boss behind it
    # (profile: plate face, plate outer, plate back, boss outer, boss floor outer, floor inner, bearing seat back, seat wall, seat front, lip inner)
    c = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    for a in FLANGE_ANGLES:
        y, z = at_angle(CART_R, a)
        c = c - along_x(1.7, x0 - 0.1, x1 + 0.1, y, z)
        cone = b.Cone(2.75, 1.7, 1.7).rotate(b.Axis.Y, 90).moved(b.Location((x0 + 0.85 - 0.0, y, z)))
        c = c - cone
    return c, brg_front, brg_back


# ---- motor edits ---------------------------------------------------------------------------------------------------------
def radial_frame(deg):
    a = math.radians(deg)
    u = np.array([0, math.cos(a), math.sin(a)])
    w = np.array([0, -math.sin(a), math.cos(a)])
    return u, w


def slab(x0, x1, r0, r1, half_w, deg):
    """box between x0..x1, radial r0..r1 and +-half_w tangential, about azimuth deg (pod frame, y = r cos a, z = r sin a)"""
    u, w = radial_frame(deg)
    M = np.eye(4)
    M[:3, 0] = [1, 0, 0]
    M[:3, 1] = -w                 # (x, -w, u) is right-handed
    M[:3, 2] = u
    centre = np.array([(x0 + x1) / 2, 0, 0]) + u * (r0 + r1) / 2
    M[:3, 3] = centre
    return b.Box(x1 - x0, 2 * half_w, r1 - r0).moved(loc(M))


def wedge(x0, x1, r0, r1, a0, a1):
    pts = []
    n = 24
    for r, order in ((r1, 1), (r0, -1)):
        angs = np.linspace(a0, a1, n)[::order]
        for a in angs:
            pts.append(at_angle(r, a))
    face = b.Plane.YZ * b.Polygon(*pts, align=None)
    return b.extrude(face, amount=x1 - x0).moved(b.Location((x0, 0, 0)))


def refill_hole(part, y, z, x0, x1, r_old, r_new):
    return part + along_x(r_old + 0.05, x0, x1, y, z) - along_x(r_new, x0 - 0.1, x1 + 0.1, y, z)


def build_motor():
    stator = placed('dummy_motor_stator', 'Motor_stator')
    # turn the stator 90 deg about the axis so its plug flat lines up with the cup slot at 45 deg (the rear holes are 4-fold)
    stator = stator.rotate(b.Axis.X, -90)
    # cable groove along the rear half of the stator: floor r 23.8, 8 wide, from the plug (x 4) to the rear face
    stator = stator - slab(4.0, 17.3, 23.8, 28.0, 4.0, CABLE_ANG)
    rotor = placed('dummy_motor_rotor', 'Motor_rotor')
    cup = placed('motor_cup', 'Motor_cup')
    # bigger cable slot: 60 deg wide (was 34) from x 2 (was 6), through the tube wall and the end wall
    cup = cup - wedge(2.0, 21.4, 10.5, 29.5, CABLE_ANG - 30, CABLE_ANG + 30)
    # groove in the stub (replaces the shallow flat): 5.6 wide, floor r 4.6, from the end wall to the stub end
    cup = cup - slab(17.0, 37.1, 4.6, 11.0, 2.8, CABLE_ANG)
    # M2.5 clearance holes in the cup's end wall: 2.9 -> 2.7
    for a in (0, 90, 180, 270):
        y, z = at_angle(23.5, a)
        cup = refill_hole(cup, y, z, 17.2, 21.2, 1.45, M25_CLEAR / 2)
    return stator, rotor, cup


def build_cable():
    """one Ø4 lead (assumed, VERIFY) in the plane of the axis and the 45 deg radial: along the stator groove, down through the cup's end-wall slot, along the stub groove"""
    u, w = radial_frame(CABLE_ANG)
    def P(x, v):
        return b.Vector(*(np.array([x, 0, 0]) + u * v))
    rb = 6.0
    segs = []
    p0, p1 = P(6.0, 26.4), P(14.0, 26.4)
    segs.append(b.Line(p0, p1))
    c1 = (14.0, 26.4 - rb)
    a = b.Edge.make_three_point_arc(P(14.0, 26.4), P(c1[0] + rb * math.sin(math.pi / 4), c1[1] + rb * math.cos(math.pi / 4)), P(c1[0] + rb, c1[1]))
    segs.append(a)
    segs.append(b.Line(P(c1[0] + rb, c1[1]), P(20.0, 13.0)))
    c2 = (26.0, 13.0)
    a2 = b.Edge.make_three_point_arc(P(20.0, 13.0), P(c2[0] - rb * math.cos(math.pi / 4), c2[1] - rb * math.sin(math.pi / 4)), P(26.0, 7.0))
    segs.append(a2)
    segs.append(b.Line(P(26.0, 7.0), P(46.0, 7.0)))
    path = b.Wire(segs)
    start = path.position_at(0)
    tan = path.tangent_at(0)
    prof = b.Plane(origin=start, z_dir=tan) * b.Circle(CABLE_D / 2)
    return b.sweep(prof, path)


# ---- hardware ----------------------------------------------------------------------------------------------------------------
def hardware(hub_frames):
    parts = {}
    bolt = b.import_step(ROOT / 'reference' / 'bolt_M3_50_PAN.step')
    roller = b.import_step(ROOT / 'reference' / 'roller_core.step')
    tread = b.import_step(ROOT / 'reference' / 'roller_tread.step')
    seat = B_OUT - HEAD_POCKET_DEPTH                 # head underside z in the roller frame (22.8)
    for row, k, M, flipped in hub_frames:
        Mb = M @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, seat], [0, 0, 0, 1.0]])    # head underside on the pocket floor, shank toward the pilot block
        parts[f'bolt_{row}_{k}'] = bolt.moved(loc(Mb))
        Mr = np.array(MAN[f'Roller_{row}_{k}']['matrix'])
        parts[f'roller_core_{row}_{k}'] = roller.moved(loc(Mr))
        parts[f'roller_tread_{row}_{k}'] = tread.moved(loc(Mr))
    return parts


def main(stage):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'parts').mkdir(exist_ok=True)
    VAL.mkdir(parents=True, exist_ok=True)
    shapes = {}
    hub = build_hub()
    shapes['wheel_hub'] = hub
    cart, bf, bb = build_cartridge()
    shapes['wheel_cartridge'] = cart
    stator, rotor, cup = build_motor()
    shapes['dummy_motor_stator'] = stator
    shapes['dummy_motor_rotor'] = rotor
    shapes['motor_cup'] = cup
    shapes['motor_cable'] = build_cable()
    frames = roller_frames()
    hw = hardware(frames)
    shapes.update(hw)
    # bearings and small hardware
    brg = b.import_step(ROOT / 'reference' / 'bearing_6804.step')
    def axial(shape, x, flip=False):
        """shape with local z along pod x, centred at x"""
        M = np.array([[0, 0, 1, x], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1.0]])
        return shape.moved(loc(M))
    shapes['bearing_6804_left'] = axial(brg, (bf + bb) / 2)
    shapes['bearing_6804_right'] = axial(brg, 33.5)
    for key in ('bolt_M2p5_10_PAN_1', 'bolt_M2p5_10_PAN_2', 'bolt_M2p5_10_PAN_3'):
        shapes[key] = placed('bolt_M2p5_10_PAN', key, 'reference')
    flat = b.import_step(ROOT / 'reference' / 'bolt_M3_12_FLAT.step')
    for i, a in enumerate(FLANGE_ANGLES, 1):
        y, z = at_angle(CART_R, a)
        shapes[f'cartridge_screw_{i}'] = flat.moved(loc(np.array([[0, 0, -1, X_LEFT], [0, 1, 0, y], [1, 0, 0, z], [0, 0, 0, 1.0]])))
    for key, stem in (('Motor_bearing', 'bearing_687'), ('Motor_screw', 'bolt_M3_8_PAN')):
        shapes[key] = placed(stem, key, 'reference')
    shapes = {k: v for k, v in shapes.items() if v is not None}
    for k, s in shapes.items():
        s.label = k
    return shapes


def overlap(a, c):
    x = a & c
    return 0.0 if x is None else float(x.volume)


def near(a, c, gap=0.5):
    p, q = a.bounding_box(), c.bounding_box()
    return all(tuple(p.min)[i] - gap <= tuple(q.max)[i] and tuple(q.min)[i] - gap <= tuple(p.max)[i] for i in range(3))


def min_dist(a, c):
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    d = BRepExtrema_DistShapeShape(a.wrapped, c.wrapped)
    d.Perform()
    return d.Value()


def check(shapes):
    rep = {}
    hub = shapes['wheel_hub']
    rep['hub'] = dict(solids=len(hub.solids()), valid=bool(hub.is_valid), volume_mm3=round(hub.volume), bbox=[[round(v, 2) for v in tuple(hub.bounding_box().min)], [round(v, 2) for v in tuple(hub.bounding_box().max)]])
    pairs = {}
    names = [k for k in shapes if k != 'wheel_hub']
    for k in names:
        if near(hub, shapes[k]):
            v = overlap(hub, shapes[k])
            if v > 0.01:
                pairs[k] = round(v, 2)
    rep['hub_overlaps_mm3'] = pairs                    # bolts overlap the pilots by design (thread major diameter 3.0 in a 2.8 hole)
    cab = shapes['motor_cable']
    cl = {}
    for k in names:
        if k == 'motor_cable' or k.startswith(('bolt_', 'roller_')):
            continue
        d = min_dist(cab, shapes[k])
        cl[k] = round(d, 3)
    cl['wheel_hub'] = round(min_dist(cab, hub), 3)
    rep['cable_clearance_mm'] = cl
    # insertion tunnels: a Ø6.4 cylinder from the head pocket floor outward must meet nothing but the (sacrificial) pocket air
    tun = {}
    for row, k, M, flipped in roller_frames():
        tool = cyl(TUNNEL_D / 2 - 0.05, 70.0, (0, 0, B_OUT - HEAD_POCKET_DEPTH + 35.0)).moved(loc(M))
        hits = {}
        for kk in names:
            if kk.startswith(('roller_', 'bolt_')) and kk.endswith(f'_{row}_{k}') :
                continue
            if kk.startswith(('roller_', 'bolt_', 'wheel_hub', 'wheel_cartridge')) and near(tool, shapes[kk], 0):
                v = overlap(tool, shapes[kk])
                if v > 0.05:
                    hits[kk] = round(v, 2)
        tun[f'{row}_{k}'] = hits
    rep['tunnel_hits'] = tun
    # rollers drop in radially after printing: slide two (an odd and an even one) outward and see whether they touch the hub on the way
    drop = {}
    for key in ((1, 1), (1, 2), (2, 1), (2, 2)):
        M = dict(((r, k), m_) for r, k, m_, f in roller_frames())[key]
        for d in (0.0, 8.0, 16.0, 24.0, 32.0):
            T = np.eye(4)
            T[:3, 3] = M[:3, 1] * d                   # local y = radial outward
            hit = overlap(hub, shapes[f'roller_core_{key[0]}_{key[1]}'].moved(loc(T))) + overlap(hub, shapes[f'roller_tread_{key[0]}_{key[1]}'].moved(loc(T)))
            drop[f'{key[0]}_{key[1]} +{d:g} mm'] = round(hit, 2)
    rep['roller_drop_in_overlap_mm3'] = drop
    # screw fits: rotor flange screws must sit in the rotor pilots (small thread overlap) and clear the hub holes; cartridge screws likewise
    fit = {}
    for i in (1, 2, 3):
        sc = shapes[f'bolt_M2p5_10_PAN_{i}']
        fit[f'M2.5 flange screw {i}'] = dict(rotor_pilot_overlap=round(overlap(sc, shapes['dummy_motor_rotor']), 2), hub_overlap=round(overlap(sc, hub), 2),
                                           gap_to_cartridge=round(min_dist(sc, shapes['wheel_cartridge']), 2))
        cs = shapes[f'cartridge_screw_{i}']
        fit[f'cartridge screw {i}'] = dict(hub_thread_overlap=round(overlap(cs, hub), 2), cartridge_overlap=round(overlap(cs, shapes['wheel_cartridge']), 2))
    rep['screw_fits'] = fit
    return rep


def export(shapes):
    from build123d import Compound
    for k, s in shapes.items():
        if k.startswith(('bolt_', 'roller_')):
            continue
        b.export_step(s, OUT / 'parts' / (k + '.step'))
    asm = Compound(children=list(shapes.values()), label='wheel_motor_v2')
    b.export_step(asm, OUT / 'wheel_motor_v2_assembly.step')


COLOURS = {'wheel_hub': '#d9826b', 'wheel_cartridge': '#e6c86b', 'dummy_motor_stator': '#8a8a8a', 'dummy_motor_rotor': '#c986c9', 'motor_cup': '#6fc2c2',
           'motor_cable': '#e03030', 'bearing': '#333333', 'roller_core': '#6aa86a', 'roller_tread': '#9ab6ff', 'bolt': '#ff00ff', 'cartridge_screw': '#ff00ff',
           'bolt_M2p5': '#ff00ff', 'Motor_': '#444444'}


def colour(name):
    for k, v in COLOURS.items():
        if name.startswith(k):
            return v
    return '#aaaaaa'


def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.colors import to_rgb

    def sliced(rot_x_deg, half_thickness=0.3):
        slab_ = b.Box(300, 300, 2 * half_thickness)
        polys = []
        for k, s in shapes.items():
            if k.startswith('roller_tread') or (k.startswith('roller_core') and False):
                continue
            sh = s.rotate(b.Axis.X, rot_x_deg) if rot_x_deg else s
            x = sh & slab_
            if x is None or not x.solids() and not x.faces():
                continue
            try:
                v, t = x.tessellate(0.12, 0.2)
            except ValueError:
                continue
            V = np.array([[q.X, q.Y] for q in v])
            for tri in np.array(t):
                polys.append((V[tri], colour(k)))
        return polys

    for name, rot, xlim, ylim in (('section_axis_plane_z0', 0, (-48, 48), (-68, 68)), ('section_cable_plane_45deg', -CABLE_ANG, (-30, 50), (-5, 40))):
        polys = sliced(rot)
        fig, ax = plt.subplots(figsize=(14, 9) if rot == 0 else (14, 6))
        ax.add_collection(PolyCollection([p[0] for p in polys], facecolors=[to_rgb(p[1]) for p in polys], edgecolors='none'))
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_xlabel('pod x (wheel axis) mm')
        ax.set_ylabel('radial mm')
        ax.set_title(name)
        fig.savefig(VAL / (name + '.png'), dpi=100)
        plt.close(fig)

    def iso(items, out, az, el, title):
        a, e = np.radians(az), np.radians(el)
        Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        Rx = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]])
        Rm = Rx @ Rz
        polys = []
        for k, s in items.items():
            v, t = s.tessellate(0.35, 0.3)
            V = np.array([[q.X, q.Y, q.Z] for q in v]) @ Rm.T
            col = np.array(to_rgb(colour(k)))
            for tri in np.array(t):
                P = V[tri]
                n = np.cross(P[1] - P[0], P[2] - P[0])
                nn = np.linalg.norm(n)
                if nn == 0:
                    continue
                n = n / nn
                if n[1] > 0:
                    continue
                polys.append((P[:, 1].mean(), P[:, [0, 2]], tuple(col * (0.45 + 0.55 * abs(n[1])))))
        polys.sort(key=lambda p: -p[0])
        fig, ax = plt.subplots(figsize=(11, 9))
        ax.add_collection(PolyCollection([p[1] for p in polys], facecolors=[p[2] for p in polys], edgecolors='none'))
        ax.autoscale()
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title(title)
        fig.savefig(out, dpi=95)
        plt.close(fig)

    nonroll = {k: v for k, v in shapes.items() if not k.startswith(('roller_tread',))}
    iso(nonroll, VAL / 'iso_whole.png', 215, 25, 'wheel + motor (treads hidden), from the motor side')
    hubonly = {k: shapes[k] for k in ('wheel_hub', 'wheel_cartridge')}
    iso(hubonly, VAL / 'iso_hub_outside.png', 150, 28, 'single-piece hub + cartridge')
    iso(hubonly, VAL / 'iso_hub_other.png', 330, 28, 'single-piece hub, motor side')
    motor = {k: shapes[k] for k in ('dummy_motor_stator', 'motor_cup', 'motor_cable')}
    iso(motor, VAL / 'iso_cable.png', 60, 35, 'stator, cup and cable')


if __name__ == '__main__':
    stage = sys.argv[1] if sys.argv[1:] else 'build'
    sh = main(stage)
    if stage in ('check', 'all'):
        rep = check(sh)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1))
        print(json.dumps({k: v for k, v in rep.items() if k != 'tunnel_hits'}, indent=1))
        print('tunnel hits:', {k: v for k, v in rep['tunnel_hits'].items() if v})
    if stage in ('export', 'all'):
        export(sh)
        print('exported to', OUT)
    if stage in ('render', 'all'):
        render(sh)
        print('rendered to', VAL)
