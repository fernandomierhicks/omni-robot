"""cad/v11 first floor for layout F (PROPOSAL, Nov 3 2026): deck petals, centre spider, lidar pedestal, D435 nose bracket, second-floor post
pads and the bumper quarters, on four lean pods (pod_lean.py). build123d, STEP review only; printed parts PETG unless noted.

    python -B first_floor.py [build|check|export|render|all]        (omni-robot-sim interpreter)

Layout F (validation/layout/LAYOUT.md): the sim's hand pattern, rolling lines tangent to r 154, each wheel slid 20 mm along its rolling line
away from its pivot (kinematics unchanged); robot Ø462. Robot frame: origin on the floor under the centre, x forward, y left, z up.
 * Deck: one 6 mm plate (top z 62, 8 mm under the axles) cut round the pods' swept volumes (+3 mm) and their drop-in path (+1 mm), printed as
   four petals split on the diagonals. Each split jogs to cross its pod between the inner frame's two foot screws, so every pod ties two petals.
   An 8 mm X spider under the centre ties the petals' inner ends and carries the lidar pedestal; the bumper quarters tie them at the rim.
 * Pods: inner frame 2 x M3 x 14 and outer keel 2 x M3 x 14 straight down into the plate (6 mm engagement); a pod lifts out from above.
 * Bumper: four PETG quarters (6 mm wall, z 40..90), split at the midlines where the plate reaches the rim; each held by four radial M3 x 12 pan
   heads from outside (counterbored) into rim bosses under the plate edge, and hung on short lips over the plate top. Off: four screws from
   the side, lift 4 mm, pull out. Across the nose the bumper is lowered to z 52 so it does not cut the D435's view of the near floor.
 * Second floor: eight posts as radial blades (5 mm across, 25 mm along the radius) so the centre lidar loses only ~1.4-2 deg behind each:
   two under the arm (in front of the lidar, behind the D435), two each side and two at the back. The deck has their foot pilots; the blades
   come with the second floor.
Device sizes and hole patterns marked VERIFY are not measured. Nothing has been printed.
"""
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import LineString, MultiPoint, Point, Polygon, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import R_WHEEL, M3_PILOT, M3_CLR                             # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

OUT = W.ROOT / 'step_export' / 'first_floor'
VAL = W.ROOT / 'validation' / 'first_floor'
B.D_WHEEL, B.SLIDE = 154.0, -20.0          # layout F
cyl, box = W.cyl, P.box

PLATE_Z = (R_WHEEL + L.PLATE_BOTTOM_L, R_WHEEL + L.PLATE_TOP_L)        # 56..62
BED = 250.0
CLR_CHECK = 2.5                            # wanted clearance to anything that moves
# bumper
BUMPER_T, BUMPER_Z, BUMPER_GAP = 6.0, (40.0, 90.0), 3.0
NOSE_DIP_DEG, NOSE_DIP_TOP = 22.0, 52.0    # across the nose the bumper stops at z 52: the D435's lowest ray (59 deg down) passes over it
MOUNTS_Q0 = (10.0, 28.0, 72.0, 84.0)       # bumper screws in the first quarter (deg); the others are its mirror images; all where the plate reaches the rim
SCREW_Z = 48.0                             # radial bumper screws (robot z)
BOSS_Z, BOSS_DEPTH, BOSS_W = (42.0, PLATE_Z[0]), 12.0, 20.0             # rim boss under the plate edge: z, radial depth, tangential width
LIP_Z, LIP_REACH, LIP_W = (PLATE_Z[1], PLATE_Z[1] + 3.0), 8.0, 24.0     # bumper lip resting on the plate top
CB_D, CB_DEPTH = 7.2, 2.9                  # counterbore for the 6.5 x 2.5 hex-drive pan head
# spider and lidar pedestal
SPIDER_T, SPIDER_W, SPIDER_HUB_R = 8.0, 20.0, 42.0
LIDAR = dict(d=76.0, h=41.0, plane=163.0, plane_above_base=31.3)        # A2M12; Nov 4: 31.3 from the vendor RPLIDAR model's Laser_link (was 30, assumed); VERIFY on the unit
LIDAR_BASE_Z = LIDAR['plane'] - LIDAR['plane_above_base']
PED_R, PED_WALL, PED_FLANGE_R, PED_TOP_R = 20.0, 3.0, 36.0, 40.0
PED_SCREW_R = 29.0
LIDAR_HOLES_R = 26.0                        # A2M12 base mounting holes: 4 x M2.5 on this circle (PLACEHOLDER, VERIFY on the unit)
# D435 at the nose (sim ARM_D435)
D435_C, D435_TILT, D435_SIZE = (190.0, 0.0, 111.5), 30.0, (25.0, 90.0, 25.0)   # centre, deg down, (depth along view, width, height)
D435_HOLES = ((0.0, -22.5), (0.0, 22.5))   # two M3 in its back face, 45 mm apart (VERIFY)
BRACKET_HALF_W = 22.0                       # the nose narrows to the front wheels behind the camera
BRACKET_BASE_BACK = 22.0                    # cradle base reaches this far behind the camera's rear face (chassis.py: 16, for the lidar pedestal)
# second-floor posts: radial blades (deg, outermost r), each on a T foot with two M3 into the plate
BLADES = ((7.0, 140.0), (-7.0, 140.0), (76.0, 208.0), (104.0, 208.0), (256.0, 208.0), (284.0, 208.0), (170.0, 208.0), (190.0, 208.0))
BLADE_T, BLADE_L, BLADE_FOOT = 5.0, 25.0, (31.0, 17.0)                  # blade thickness / radial length; foot radial x tangential
BLADE_PILOTS = ((9.0, 5.5), (-9.0, -5.5))                               # (radial, tangential) from the blade centre
COLOURS = {'plate': (0.80, 0.80, 0.78), 'spider': (0.55, 0.57, 0.60), 'pedestal': (0.45, 0.47, 0.52), 'bracket': (0.35, 0.55, 0.85),
           'bumper': (0.90, 0.45, 0.15), 'bolt': (1.0, 0.0, 1.0), 'env': (0.12, 0.12, 0.14)}


def colour(name):
    for k, v in COLOURS.items():
        if name.split('/')[-1].startswith(k):
            return v
    return L.colour(name.split('/')[-1])


def U(a):
    return np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])


def pan_screw(p, axis, length):
    """M3 hex-drive pan head (6.5 x 2.5) + shank: head on the surface at p, shank along unit `axis` (into the parts)"""
    ax = np.asarray(axis, float)
    head = b.Solid.make_cylinder(3.25, 2.5, b.Plane(origin=tuple(np.asarray(p) - 2.5 * ax), z_dir=tuple(ax)))
    shank = b.Solid.make_cylinder(1.5, length, b.Plane(origin=tuple(p), z_dir=tuple(ax)))
    return head + shank


# ---- pod footprints ---------------------------------------------------------------------------------------------------------------------
def pod_geometry():
    cache = VAL / 'pod_fp.pkl'
    if cache.exists():
        return pickle.loads(cache.read_bytes())
    parts, _ = L.build_static()
    unit = P.wheel_unit()
    fp = B.footprints(parts, unit)
    bands = {}                                      # moving parts' plan sections per robot-z band, over the poses (pod frame)
    for nm, (z0, z1) in dict(below=(BOSS_Z[0] - 2, PLATE_Z[0]), plate=PLATE_Z, above=(PLATE_Z[1], PLATE_Z[1] + 6), bumper=BUMPER_Z,
                              nose=(95.0, 135.0)).items():
        gs = []
        for a in L.POSES.values():
            sh, _ = L.posed(parts, unit, a)
            for k, s in sh.items():
                if k.startswith(B.MOVING_SKIP):
                    continue
                g = B.proj(s, k.startswith(B.EXACT), z0 - R_WHEEL, z1 - R_WHEEL)
                if g is not None:
                    gs.append(g)
        bands[nm] = union_all(gs)
    st = []
    sh, _ = L.posed(parts, unit, 0.0)
    for k in ('pod_frame_inner', 'pod_keel_outer'):
        g = B.proj(sh[k], True, BOSS_Z[0] - R_WHEEL, PLATE_Z[1] - 0.05 - R_WHEEL)
        if g is not None:
            st.append(g)
    fp['bands'] = bands
    fp['static_below_deck'] = union_all(st)
    VAL.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(pickle.dumps(fp))
    return fp


def placed(g):
    return union_all([B.place2d(g, i) for i in range(4)])


# ---- build ------------------------------------------------------------------------------------------------------------------------------
def mount_angles():
    out = []
    for a in MOUNTS_Q0:
        out += [a, 180.0 - a, 180.0 + a, 360.0 - a]
    return sorted(round(v % 360, 3) for v in out)


def blade_foot(a, r, grow=0.0):
    return affinity.rotate(sbox(r - BLADE_FOOT[0] / 2 - grow, -BLADE_FOOT[1] / 2 - grow, r + BLADE_FOOT[0] / 2 + grow, BLADE_FOOT[1] / 2 + grow), a, origin=(0, 0))


def post_points(free, blocked):
    pts = []
    for a, rmax in BLADES:
        for r in np.arange(rmax, 60, -1.0):
            f = blade_foot(a, r, 2.0)
            if free.contains(f) and not f.intersects(blocked):
                pts.append((a % 360, float(r)))
                break
    return pts


def build(fp):
    rb = B.r_bumper(fp)
    rp = rb - 0.5
    cut = placed(fp['cutout'])
    disc = Point(0, 0).buffer(rp, 720)
    plate2d = max(getattr(disc.difference(cut), 'geoms', [disc.difference(cut)]), key=lambda g: g.area)   # drops slivers inside the cutouts
    free = disc.difference(placed(fp['all']).buffer(3.0))
    info = dict(r_bumper_inside=round(rb, 1), r_plate=round(rp, 1), robot_diameter=round(2 * (rb + BUMPER_T), 1))
    # spider and splits (as base.py: along each diagonal, jogged past the spider to cross the pod between its frame's foot screws)
    split_y = sum(L.MOUNT_Y_L) / 2
    arms, r_sp, split = [], {}, {}
    for i, a in enumerate(B.ANGLES):
        u = U(a)
        r = next(r for r in np.arange(SPIDER_HUB_R, rp, 0.5) if cut.distance(Point(*(u * r))) < SPIDER_W / 2 + 6.0)
        r_sp[a] = float(r)
        arms.append(affinity.rotate(sbox(0, -SPIDER_W / 2, r, SPIDER_W / 2), a, origin=(0, 0)))
        ypod = np.asarray(B.place2d(Point(0, 1), i).coords[0]) - np.asarray(B.place2d(Point(0, 0), i).coords[0])
        off = (-B.SLIDE - split_y) * -ypod
        t = r - 2.0
        split[a] = [np.zeros(2), u * t, u * t + off, u * 1.5 * rb + off]
    spider2d = union_all(arms + [Point(0, 0).buffer(SPIDER_HUB_R, 128)])
    # holes in the plate: pod pilots, spider / pedestal clearances, D435 bracket pilots, post pads
    holes = dict(pod=[], spider=[], pedestal=[], bracket=[], post=[])
    for i in range(4):
        for p in B.pod_screws():
            holes['pod'].append(tuple(np.asarray(B.place2d(Point(*p), i).coords[0])))
    for a in B.ANGLES:
        u, n = U(a), U(a + 90)
        for r in (SPIDER_HUB_R + 6.0, r_sp[a] - 9.0):
            for s in (1, -1):
                holes['spider'].append(tuple(u * r + s * 5.5 * n))
    for a in (0, 90, 180, 270):
        holes['pedestal'].append(tuple(U(a) * PED_SCREW_R))
    BR = bracket_geometry()
    holes['bracket'] = BR['deck_holes']
    blocked = union_all([B.proj(BR['bracket'], False), B.proj(BR['camera'], False), B.proj(BR['plug'], False), Point(0, 0).buffer(LIDAR['d'] / 2 + 3)]).buffer(3.0)
    posts = post_points(free.intersection(plate2d), blocked)
    for a, r in posts:
        for dr, dt in BLADE_PILOTS:
            holes['post'].append(tuple(U(a) * (r + dr) + U(a + 90) * dt))
    # petals: 2D cut by the splits, then 3D with rim bosses, holes and the bumper screw pilots
    F = 1.5 * rb
    mounts = mount_angles()
    petals, petal2d = {}, {}
    for k, nm in enumerate(('front', 'left', 'back', 'right')):
        lo, hi = split[(90.0 * k - 45) % 360], split[(90.0 * k + 45) % 360]
        arc_ = [tuple(F * U(t)) for t in np.linspace(90 * k - 40, 90 * k + 40, 14)]
        wedge = Polygon([tuple(p) for p in lo] + arc_ + [tuple(p) for p in hi[::-1]])
        g = plate2d.intersection(wedge)
        g = max(getattr(g, 'geoms', [g]), key=lambda q: q.area)
        petal2d[nm] = g
        s = B.solid_of(g, *PLATE_Z)
        for a in mounts:
            if not g.contains(Point(*(U(a) * (rp - BOSS_DEPTH / 2)))):
                continue
            boss = affinity.rotate(sbox(rp - BOSS_DEPTH, -BOSS_W / 2, rp, BOSS_W / 2), a, origin=(0, 0))
            s = s + B.solid_of(boss, *BOSS_Z)
            pil = b.Solid.make_cylinder(M3_PILOT / 2, 10.5, b.Plane(origin=(*(U(a) * (rp + 0.1)), SCREW_Z), z_dir=(*(-U(a)), 0)))
            s = s - pil
        for kind, d in (('pod', M3_PILOT), ('spider', M3_CLR), ('pedestal', M3_CLR), ('bracket', M3_PILOT), ('post', M3_PILOT)):
            for x, y in holes[kind]:
                if g.buffer(1).contains(Point(x, y)):
                    s = s - cyl(d / 2, PLATE_Z[1] - PLATE_Z[0] + 0.2, (x, y, sum(PLATE_Z) / 2))
        petals[f'plate_{nm}'] = s
    spider = B.solid_of(spider2d, PLATE_Z[0] - SPIDER_T, PLATE_Z[0])
    for x, y in holes['spider'] + holes['pedestal']:
        spider = spider - cyl(M3_PILOT / 2, SPIDER_T + 0.2, (x, y, PLATE_Z[0] - SPIDER_T / 2))
    parts = dict(petals, spider=spider, pedestal=build_pedestal(), bracket_d435=BR['bracket'])
    parts.update(build_bumpers(rb, rp, petal2d))
    # hardware and devices (not printed; not in the printed-parts STEP)
    hw = {}
    n = 0
    for x, y in holes['pod']:
        hw[f'bolt_pod_{n}'] = pan_screw((x, y, PLATE_Z[1] + 8.0), (0, 0, -1), 14.0)       # 8 mm feet (keel foot 7: the head sits 1 lower, same screw)
        n += 1
    for n, (x, y) in enumerate(holes['spider']):
        hw[f'bolt_spider_{n}'] = pan_screw((x, y, PLATE_Z[1]), (0, 0, -1), 14.0)
    for n, (x, y) in enumerate(holes['pedestal']):
        hw[f'bolt_pedestal_{n}'] = pan_screw((x, y, PLATE_Z[1] + 4.0), (0, 0, -1), 18.0)
    for n, a in enumerate(mounts):
        hw[f'bolt_bumper_{n}'] = pan_screw((*(U(a) * (rb + BUMPER_T - CB_DEPTH)), SCREW_Z), (*(-U(a)), 0), 12.0)
    hw.update(BR['hardware'])
    env = dict(env_d435=BR['camera'], env_d435_plug=BR['plug'],
               env_lidar=cyl(LIDAR['d'] / 2, LIDAR['h'], (0, 0, LIDAR_BASE_Z + LIDAR['h'] / 2)))
    info.update(spider_reach=r_sp, posts=[[a, round(r, 1)] for a, r in posts], bumper_mounts_deg=mounts,
                holes={k: len(v) for k, v in holes.items()}, plate_area_cm2=round(plate2d.area / 100, 1))
    geo = dict(plate2d=plate2d, petal2d=petal2d, spider2d=spider2d, cut=cut, free=free, holes=holes, posts=posts, rb=rb, rp=rp, split=split)
    return parts, hw, env, info, geo


def build_pedestal():
    z0, z1 = PLATE_Z[1], LIDAR_BASE_Z
    p = cyl(PED_FLANGE_R, 4.0, (0, 0, z0 + 2.0)) + cyl(PED_R, z1 - z0, (0, 0, (z0 + z1) / 2)) + cyl(PED_TOP_R, 5.0, (0, 0, z1 - 2.5))
    p = p - cyl(PED_R - PED_WALL, z1 - z0 - 5.0 + 0.1, (0, 0, (z0 + z1 - 5.0) / 2))
    for a in (0, 90, 180, 270):
        x, y = U(a) * PED_SCREW_R
        p = p - cyl(M3_CLR / 2, 4.2, (x, y, z0 + 2.0))
    for a in (45, 135, 225, 315):
        x, y = U(a) * LIDAR_HOLES_R
        p = p - cyl(1.35, 5.2, (x, y, z1 - 2.5))
    p = p - box(-6, 6, PED_R - PED_WALL - 1, PED_R + 1, z1 - 30, z1 - 5)                 # slot for the lidar's lead into the tube
    p = p - cyl(8.0, 5.2, (0, 0, z1 - 2.5))                                              # lead up / through the top plate
    return p


def bracket_geometry():
    """D435 cradle on the front petal: a base on the deck and a back plate tilted with the camera, two cheeks between; the camera's two rear M3s
    go through the back plate. Camera frame: x along its view (forward-down), y across, z its up."""
    cx, cy, cz = D435_C
    t = math.radians(D435_TILT)
    view = np.array([math.cos(t), 0, -math.sin(t)])
    up = np.array([math.sin(t), 0, math.cos(t)])
    dpt, wid, hgt = D435_SIZE

    def cam_loc():
        return b.Location((cx, cy, cz), (0, 1, 0), D435_TILT)                               # rotate about +y by +tilt: x forward-down

    camera = box(-dpt / 2, dpt / 2, -wid / 2, wid / 2, -hgt / 2, hgt / 2).moved(cam_loc())
    back = box(-dpt / 2 - 4.0, -dpt / 2, -wid / 2 + 5, wid / 2 - 5, -hgt / 2 - 3, hgt / 2 + 3).moved(cam_loc())
    rear_c = np.array(D435_C) - view * (dpt / 2 + 4.0)
    base_x = (rear_c[0] - BRACKET_BASE_BACK, rear_c[0] + 8.0)
    base = box(base_x[0], base_x[1], -BRACKET_HALF_W, BRACKET_HALF_W, PLATE_Z[1], PLATE_Z[1] + 4.0)
    cheeks = None
    for s in (1, -1):
        y0, y1 = sorted((s * (BRACKET_HALF_W - 6.0), s * BRACKET_HALF_W))
        pts = [(base_x[0], PLATE_Z[1]), (base_x[1], PLATE_Z[1])]
        bb = back.bounding_box()
        pts += [(bb.max.X - 2.0, bb.min.Z + 6.0), (bb.min.X, bb.max.Z - 2.0), (base_x[0], PLATE_Z[1] + 4.0)]
        poly = b.extrude(b.Face(b.Wire.make_polygon([b.Vector(x, 0, z) for x, z in pts], close=True)), amount=y1 - y0).moved(b.Location((0, y1, 0)))
        cheeks = poly if cheeks is None else cheeks + poly
    br = base + back + cheeks
    hw = {}
    for n, (cz_, cy_) in enumerate(D435_HOLES):
        p = np.array(D435_C) - view * dpt / 2 + up * cz_ + np.array([0, cy_, 0])
        br = br - b.Solid.make_cylinder(M3_CLR / 2, 12.0, b.Plane(origin=tuple(p - view * 10.0), z_dir=tuple(view)))
        hw[f'bolt_d435_{n}'] = pan_screw(tuple(p - view * 4.0), tuple(view), 8.0)
    deck_holes = [(base_x[0] + 6.0, s * (BRACKET_HALF_W - 12.0)) for s in (1, -1)]
    for n, (x, y) in enumerate(deck_holes):
        br = br - cyl(M3_CLR / 2, 4.2, (x, y, PLATE_Z[1] + 2.0))
        hw[f'bolt_bracket_{n}'] = pan_screw((x, y, PLATE_Z[1] + 4.0), (0, 0, -1), 10.0)
    # its USB-C (on the camera's -y end, VERIFY side): a straight plug (25 out + an R25 bend) reaches the front wheel, so a right-angle
    # USB-C plug (12 out, its overmold turning up 22) with the lead rising toward the PC on the second floor (VERIFY the cable)
    plug = box(0, 12.0, -6.0, 6.0, -3.5, 22.0)
    tail = b.Solid.make_cylinder(2.5, 20.0, b.Plane(origin=(6.0, 0, 22.0), z_dir=(0, 0, 1)))
    pl = (plug + tail).rotate(b.Axis.Z, -90)                                               # out along -y (camera frame), up its +z
    pl = pl.moved(b.Location((0, -wid / 2, 0))).moved(cam_loc())
    return dict(bracket=br, camera=camera, plug=pl, deck_holes=deck_holes, hardware=hw)


def build_bumpers(rb, rp, petal2d):
    ro = rb + BUMPER_T
    ring = B.solid_of(Point(0, 0).buffer(ro, 720).difference(Point(0, 0).buffer(rb, 720)), *BUMPER_Z)
    dip = B.solid_of(Polygon([(0, 0)] + [tuple(1.5 * ro * U(t)) for t in np.linspace(-NOSE_DIP_DEG, NOSE_DIP_DEG, 9)]), NOSE_DIP_TOP, BUMPER_Z[1] + 1)
    ring = ring - dip
    mounts = mount_angles()
    plate_all = union_all(list(petal2d.values()))
    out = {}
    for q in range(4):
        wedge = B.solid_of(Polygon([(0, 0)] + [tuple(1.5 * ro * U(t)) for t in np.linspace(90 * q, 90 * q + 90, 12)]), 0, 200)
        s = ring & wedge
        for a in mounts:
            if not (90 * q < a < 90 * q + 90):
                continue
            in_dip = abs(((a + 180) % 360) - 180) < NOSE_DIP_DEG
            if not in_dip:                                                                   # lip over the plate top
                lip = affinity.rotate(sbox(rp - LIP_REACH, -LIP_W / 2, rb + 0.5, LIP_W / 2), a, origin=(0, 0))
                if plate_all.buffer(0.5).contains(affinity.rotate(sbox(rp - LIP_REACH, -LIP_W / 2, rp, LIP_W / 2), a, origin=(0, 0))):
                    s = s + B.solid_of(lip, *LIP_Z)
            org = U(a) * (ro + 0.1)
            s = s - b.Solid.make_cylinder(M3_CLR / 2, BUMPER_T + 1.0, b.Plane(origin=(*org, SCREW_Z), z_dir=(*(-U(a)), 0)))
            s = s - b.Solid.make_cylinder(CB_D / 2, CB_DEPTH + 0.1, b.Plane(origin=(*org, SCREW_Z), z_dir=(*(-U(a)), 0)))
        out[f'bumper_q{q}'] = s
    return out


# ---- checks ---------------------------------------------------------------------------------------------------------------------------------
def check(parts, hw, env, info, geo, fp):
    rep = dict(layout=dict(D=B.D_WHEEL, slide=B.SLIDE, wheels={B.NAMES[i]: [round(v, 1) for v in B.frame(i)[0]] for i in range(4)},
                           wheel_circle_r=round(float(np.hypot(*B.frame(0)[0])), 1),
                           wheel_angles_deg=[round(math.degrees(math.atan2(*B.frame(i)[0][::-1])), 1) for i in range(4)]), info=info)
    pcs = {}
    for k, s in parts.items():
        g = B.proj(s, False)
        fits = []
        for ang in range(0, 90, 2):
            x0, y0, x1, y1 = affinity.rotate(g, ang, origin=(0, 0)).bounds
            fits.append((max(x1 - x0, y1 - y0), sorted((x1 - x0, y1 - y0))))
        e = min(fits)[1]
        bb = s.bounding_box()
        pcs[k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_cm3=round(s.volume / 1000, 1), mass_g_petg=round(s.volume / 1000 * 1.27 * 0.6),
                      footprint_on_bed=[round(e[1], 1), round(e[0], 1)], height=round(bb.size.Z, 1), bed_fit=bool(e[1] <= BED))
    rep['printed_parts'] = pcs
    # clearances to the pods' moving parts (swept droop..bump), plan sections in robot-z bands
    bands = {k: placed(v) for k, v in fp['bands'].items()}
    deck2d = union_all(list(geo['petal2d'].values()))
    rp, rb = geo['rp'], geo['rb']
    bosses = union_all([affinity.rotate(sbox(rp - BOSS_DEPTH, -BOSS_W / 2, rp, BOSS_W / 2), a, origin=(0, 0)) for a in info['bumper_mounts_deg']
                        if deck2d.contains(Point(*(U(a) * (rp - BOSS_DEPTH / 2))))])
    spider2d = geo['spider2d']
    ring = Point(0, 0).buffer(rb + BUMPER_T, 720).difference(Point(0, 0).buffer(rb, 720))
    lips = union_all([affinity.rotate(sbox(rp - LIP_REACH, -LIP_W / 2, rb, LIP_W / 2), a, origin=(0, 0)) for a in info['bumper_mounts_deg']])
    br2d = B.proj(parts['bracket_d435'], False)
    cl = dict(plate_to_moving=deck2d.distance(bands['plate']), bosses_spider_to_moving=union_all([bosses, spider2d]).distance(bands['below']),
              lips_to_moving=lips.distance(bands['above']), bracket_to_moving_plan=br2d.distance(bands['above']),
              bumper_to_moving=ring.distance(bands['bumper']))
    rep['clearance_to_moving_mm'] = {k: round(float(v), 2) for k, v in cl.items()}
    rep['static_pod_parts_below_deck_hit_plate_mm2'] = round(placed(fp['static_below_deck']).intersection(deck2d).area, 2)
    # D435, its plug and its bracket against the front pods (3D, every pose)
    parts_pod, _ = L.build_static()
    unit = P.wheel_unit()
    near_items = dict(camera=env['env_d435'], plug=env['env_d435_plug'], bracket=parts['bracket_d435'])
    dmin = {k: 1e9 for k in near_items}
    for a in L.POSES.values():
        sh, _ = L.posed(parts_pod, unit, a)
        for i in (0, 3):                                                                     # FL, FR
            for k, s in sh.items():
                if k.startswith(('spring_plunger', 'die_spring')):
                    continue
                ps = B.place3d(s, i)
                for nm, it in near_items.items():
                    if W.near(ps, it, 15):
                        dmin[nm] = min(dmin[nm], W.min_dist(ps, it))
    rep['d435_to_front_pods_mm'] = {k: round(v, 2) for k, v in dmin.items()}
    # the D435's view of the floor past the bumper dip
    cx, _, cz = D435_C
    ang = math.degrees(math.atan2(cz - NOSE_DIP_TOP, rb + BUMPER_T - cx))
    lowest = min(ang, D435_TILT + 29.0)
    rep['d435_floor_view'] = dict(lowest_ray_deg_below_horizontal=round(lowest, 1), nearest_floor_x=round(cx + cz / math.tan(math.radians(lowest)), 1),
                                  camera_own_limit_deg=D435_TILT + 29.0, note='arm floor reach 260..360 mm from the centre (sim)')
    # lidar
    lx = ly = 0.0
    sh_ = [dict(bearing=a, r=r, width_deg=round(2 * math.degrees(math.asin((BLADE_T / 2 + 0.5) / (r - BLADE_L / 2))), 1)) for a, r in geo['posts']]
    rep['lidar'] = dict(at=[0, 0], scan_z=LIDAR['plane'], base_z=LIDAR_BASE_Z, blade_shadows=sh_, blind_total_deg=round(sum(s['width_deg'] for s in sh_), 1),
                        note=f'radial blades {BLADE_T:g} mm across (+0.5 mm a side for the beam); round Ø16 posts at r 200 would be ~4.6 deg each')
    # screws: grip and engagement as built
    rep['screws'] = [
        dict(where='pod inner frame foot -> plate', n=8, size='M3 x 14 pan', grip=8.0, engagement=6.0, into='plate Ø2.8 pilot (through)'),
        dict(where='pod outer keel foot -> plate', n=8, size='M3 x 14 pan', grip=7.0, engagement=6.0, into='plate Ø2.8 pilot (through; 1 mm proud below)'),
        dict(where='plate -> spider', n=info['holes']['spider'], size='M3 x 14 pan', grip=6.0, engagement=8.0, into='spider Ø2.8'),
        dict(where='lidar pedestal -> plate -> spider', n=4, size='M3 x 18 pan', grip=10.0, engagement=8.0, into='spider Ø2.8'),
        dict(where='bumper -> rim boss (radial, from outside)', n=len(info['bumper_mounts_deg']), size='M3 x 12 pan', grip=round(BUMPER_T - CB_DEPTH + 0.5, 1),
             engagement=round(12 - (BUMPER_T - CB_DEPTH) - 0.5, 1), into='rim boss Ø2.8 x 10.5'),
        dict(where='D435 bracket -> plate', n=2, size='M3 x 10 pan', grip=4.0, engagement=6.0, into='plate Ø2.8'),
        dict(where='D435 -> bracket', n=2, size='M3 x 8 pan', grip=4.0, engagement=4.0, into="camera's rear M3 (vendor thread, depth VERIFY)"),
        dict(where='lidar -> pedestal top', n=4, size='M2.5 x 8', grip=5.0, engagement=3.0, into='lidar base (pattern PLACEHOLDER, VERIFY)')]
    return rep


# ---- export / render -------------------------------------------------------------------------------------------------------------------------
def pod_items():
    parts, _ = L.build_static()
    unit = P.wheel_unit()
    return B.pod_items(parts, unit)


def export(parts, env):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    for k, s in parts.items():
        write_part(s, colour(k), OUT / 'parts' / f'{k}.step', k)
    items = {k: (s, colour(k)) for k, s in parts.items()}
    items.update({k: (s, COLOURS['env']) for k, s in env.items()})
    for i in range(4):
        for k, s in pod_items().items():
            items[f'{B.NAMES[i]}/{k}'] = (B.place3d(s, i), L.colour(k))
    write_step(items, OUT / 'first_floor_assembly.step', 'first_floor')


def render(parts, hw, env, info, geo, fp):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MP, Rectangle
    for f in VAL.glob('*.png'):
        f.unlink()

    def fill(ax, g, **kw):
        for q in getattr(g, 'geoms', [g]):
            if q.is_empty:
                continue
            ax.add_patch(MP(np.asarray(q.exterior.coords), closed=True, **kw))
            for h in q.interiors:
                ax.add_patch(MP(np.asarray(h.coords), closed=True, facecolor='white', edgecolor=kw.get('edgecolor', 'k'), lw=0.4))

    pcol = dict(front=(0.86, 0.84, 0.74), left=(0.78, 0.84, 0.86), back=(0.84, 0.78, 0.84), right=(0.80, 0.86, 0.78))
    fig, ax = plt.subplots(figsize=(13, 13))
    rb = geo['rb']
    for q in range(4):
        seg = Point(0, 0).buffer(rb + BUMPER_T, 720).difference(Point(0, 0).buffer(rb, 720)).intersection(
            Polygon([(0, 0)] + [tuple(1.5 * rb * U(t)) for t in np.linspace(90 * q + 0.4, 90 * q + 89.6, 12)]))
        fill(ax, seg, facecolor=(0.9, 0.45, 0.15) if q % 2 else (0.95, 0.6, 0.3), edgecolor='k', lw=0.4)
    for nm, g in geo['petal2d'].items():
        fill(ax, g, facecolor=pcol[nm], edgecolor='0.3', lw=0.7)
        c = g.representative_point()
        ax.text(c.x * 1.0, c.y * 1.0, f'petal {nm}', fontsize=9, ha='center', color='0.3')
    fill(ax, geo['spider2d'], facecolor='none', edgecolor='0.2', lw=0.9, ls='--')
    for i in range(4):
        fill(ax, B.place2d(fp['moving'], i), facecolor=(0.95, 0.75, 0.15, 0.35), edgecolor=(0.6, 0.45, 0), lw=0.4)
        fill(ax, B.place2d(fp['static'], i), facecolor=(0.25, 0.68, 0.74, 0.8), edgecolor=(0.1, 0.35, 0.4), lw=0.4)
        A = B.frame(i)[0]
        ax.plot(*A, 'k+', ms=12)
        ax.text(*(A * 1.13), B.NAMES[i], fontsize=11, weight='bold', ha='center')
    ax.add_patch(Circle((0, 0), float(np.hypot(*B.frame(0)[0])), facecolor='none', edgecolor='0.5', ls=':', lw=0.7))
    ax.add_patch(Circle((0, 0), PED_FLANGE_R, facecolor=COLOURS['pedestal'], edgecolor='k'))
    ax.add_patch(Circle((0, 0), LIDAR['d'] / 2, facecolor='none', edgecolor='k', ls='--'))
    fill(ax, B.proj(parts['bracket_d435'], False), facecolor=COLOURS['bracket'], edgecolor='k')
    fill(ax, B.proj(env['env_d435'], False), facecolor=(0.15, 0.15, 0.3, 0.6), edgecolor='k')
    fill(ax, B.proj(env['env_d435_plug'], False), facecolor=(1, 0, 1, 0.7), edgecolor='m')
    for a, r in geo['posts']:
        fill(ax, blade_foot(a, r), facecolor='none', edgecolor='k', lw=0.8)
        fill(ax, affinity.rotate(sbox(r - BLADE_L / 2, -BLADE_T / 2, r + BLADE_L / 2, BLADE_T / 2), a, origin=(0, 0)), facecolor='k', edgecolor='k')
        for k in (-1, 1):
            w = math.asin((BLADE_T / 2 + 0.5) / (r - BLADE_L / 2)) * k
            ax.plot([0, 250 * math.cos(math.radians(a) + w)], [0, 250 * math.sin(math.radians(a) + w)], color='0.6', lw=0.4)
    for k, s in hw.items():
        bb = s.bounding_box()
        ax.plot((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, 'm.', ms=5)
    ax.set_xlim(-245, 245)
    ax.set_ylim(-245, 245)
    ax.set_aspect('equal')
    ax.grid(True, lw=0.3)
    ax.set_title(f'first floor, layout F (plan, front = +x): Ø{info["robot_diameter"]:.0f}; four petals (split on the diagonals, jogged between each pod frame\'s screws),\n'
                 'spider dashed, lidar pedestal at the centre, D435 bracket at the nose, 8 second-floor blade posts (black, T feet; grey: their lidar shadows), bumper quarters (orange), screws magenta;\n'
                 f'wheels on one circle r {np.hypot(*B.frame(0)[0]):.1f} (dotted), at ±{abs(math.degrees(math.atan2(B.frame(0)[0][1], B.frame(0)[0][0]))):.1f} deg from the x axis', fontsize=10)
    fig.tight_layout()
    fig.savefig(VAL / 'plan.png', dpi=90)
    plt.close(fig)
    # bumper mount section (r-z) at a lip mount and at a nose-dip mount
    fig, axs = plt.subplots(1, 2, figsize=(14, 6))
    rp = geo['rp']
    for ax, (a, title) in zip(axs, ((72.0, 'side mount (72 deg): lip on the plate + radial screw'), (10.0, 'nose mount (10 deg): bumper dipped to z 52, screw only'))):
        ax.add_patch(Rectangle((rp - 60, PLATE_Z[0]), 60, PLATE_Z[1] - PLATE_Z[0], facecolor=COLOURS['plate'], edgecolor='k'))
        ax.add_patch(Rectangle((rp - BOSS_DEPTH, BOSS_Z[0]), BOSS_DEPTH, BOSS_Z[1] - BOSS_Z[0], facecolor=COLOURS['plate'], edgecolor='k'))
        top = NOSE_DIP_TOP if a == 10.0 else BUMPER_Z[1]
        ax.add_patch(Rectangle((rb, BUMPER_Z[0]), BUMPER_T, top - BUMPER_Z[0], facecolor=(0.9, 0.45, 0.15), edgecolor='k'))
        if a != 10.0:
            ax.add_patch(Rectangle((rp - LIP_REACH, LIP_Z[0]), rb - rp + LIP_REACH, LIP_Z[1] - LIP_Z[0], facecolor=(0.9, 0.45, 0.15), edgecolor='k'))
        ax.add_patch(Rectangle((rb + BUMPER_T - CB_DEPTH - 2.5, SCREW_Z - 3.25), 2.5, 6.5, facecolor='m'))
        ax.add_patch(Rectangle((rb + BUMPER_T - CB_DEPTH - 12, SCREW_Z - 1.5), 12, 3.0, facecolor='m'))
        ax.annotate('plate (petal)', (rp - 55, PLATE_Z[1] + 2), fontsize=9)
        ax.annotate('rim boss (part of the petal)', (rp - 45, BOSS_Z[0] - 6), fontsize=9)
        ax.annotate('bumper quarter', (rb + BUMPER_T + 2, BUMPER_Z[0] + 5), fontsize=9)
        ax.annotate('M3 x 12 pan, from outside', (rb + BUMPER_T + 2, SCREW_Z + 2), fontsize=9, color='m')
        ax.axhline(0, color='k')
        ax.set_xlim(rp - 65, rb + 45)
        ax.set_ylim(30, 100)
        ax.set_aspect('equal')
        ax.grid(True, lw=0.3)
        ax.set_title(title + '  (r-z section; off: 4 screws, lift 4 mm, pull out)', fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'bumper_mount_sections.png', dpi=90)
    plt.close(fig)
    # iso views: whole floor with the pods; the floor parts alone, one bumper quarter pulled off
    items = {k: v for k, v in parts.items()}
    items.update(env)
    items.update(hw)
    pod = {k: v for k, v in pod_items().items() if not k.startswith(('roller_core', 'die_spring'))}
    full = dict(items)
    for i in range(4):
        for k, s in pod.items():
            full[f'{B.NAMES[i]}/{k}'] = B.place3d(s, i)
    tiles = {}
    for k, s in list(full.items()):
        if k.startswith('plate_'):
            for ix, x in enumerate(range(-240, 240, 40)):
                for iy, y in enumerate(range(-240, 240, 40)):
                    t = s & box(x, x + 40, y, y + 40, 30, 80)
                    if t is not None and t.solids():
                        tiles[f'{k}_{ix}_{iy}'] = t
            del full[k]
    full.update(tiles)
    L.iso(full, VAL / 'iso_front_left.png', 225, 28, 'first floor with the pods (front-left)', colour, tol=0.8)
    floor_only = {k: v for k, v in full.items() if not ('/' in k)}
    exp = dict(floor_only)
    for k in [k for k in exp if k.startswith('bumper_q0')]:
        exp[k] = exp[k].moved(b.Location((*(U(45) * 45), 8)))
    for k in [k for k in exp if k.startswith('bolt_bumper')]:
        bb = exp[k].bounding_box()
        c = math.degrees(math.atan2((bb.min.Y + bb.max.Y) / 2, (bb.min.X + bb.max.X) / 2))
        if 0 < c < 90:
            exp[k] = exp[k].moved(b.Location((*(U(45) * 70), 8)))
    L.iso(exp, VAL / 'iso_floor_parts.png', 210, 35, 'first-floor parts only (pods hidden); front-left bumper quarter pulled off', colour, tol=0.8)
    L.iso(floor_only, VAL / 'iso_from_below.png', 200, -30, 'first floor from below: spider, rim bosses, bumper', colour, tol=0.8)


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    fp = pod_geometry()
    parts, hw, env, info, geo = build(fp)
    print(json.dumps(info), flush=True)
    for k, s in parts.items():
        print(k, len(s.solids()), s.is_valid, round(s.volume / 1000, 1), flush=True)
    if stage in ('check', 'all'):
        rep = check(parts, hw, env, info, geo, fp)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps({k: v for k, v in rep.items() if k not in ('printed_parts', 'screws')}, indent=1), flush=True)
        print(json.dumps(rep['printed_parts'], indent=None), flush=True)
    if stage in ('export', 'all'):
        export(parts, env)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(parts, hw, env, info, geo, fp)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
