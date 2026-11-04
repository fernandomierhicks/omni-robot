"""cad/v11 chassis for layout F (PROPOSAL, Nov 3 2026): the first floor as a frame instead of a plate. build123d, STEP review only.
Replaces first_floor.py's petals and spider (kept there for comparison); reuses its D435 cradle and pod footprints.

    python -B chassis.py [build|check|export|render|all]        (omni-robot-sim interpreter)

Why a frame: the 6 mm plate tied neighbouring pods through 19-25 mm wide necks (about as stiff as a ruler); its petals were really held
together by the pods. The first floor's jobs are to tie the four pods into one rigid chassis and hold the lidar, the D435 and the bumper; the
second floor stands on the pods themselves (their spring columns), so its weight goes straight to the wheels. Robot frame: x forward, z up.
 * RING: solid square-section members, 22 mm deep (z 40..62; print them with few walls and low infill), on |x| + |y| = 100..121 under the pods'
   inner frames; CROSS on the square's diagonals triangulates it; NODES at its corners run out to the rim.
 * Each pod is held, not just screwed: its inner frame stands in a pocket (0.2 mm clearance) whose 14 mm walls hug its centre-side face and its
   front end; its outer keel stands in a pocket on the side node (walls behind it and at its inboard end). Its pivot block bears on the ring.
   Round 12: all its screws come UP from under the members (3 into the frame's foot, 2 into the keel), and members and walls are clipped 1 mm
   off the path every pod part sweeps going out radially, so a pod comes out sideways with the second floor on (bumper quarter off).
 * Lidar (A2M12) on a pedestal on the front node at x 120, scan z 163: forward, where the scans matter, just behind the D435 cradle (the sim's
   x 140 would put its top plate into the cradle). Nothing of the pods reaches its height band.
 * Second-floor supports: three printed fins, each standing in a vertical plane through the lidar's axis, so at the scan plane the lidar sees
   only a 5 mm web edge-on however long the fin is. Each fin uses that length: a flange the whole length under the floor, two legs, an elliptical
   arch between them and coves out to the tips; legs flare toward the deck. The spine runs down the centreline behind the lidar (feet on the cross
   and the back node, under the battery and computer). The two side fins lie on the rays through the front pods' column-top pilots, which also
   pass under the arm riser's rear corners: nose under the corner, seat on the column top (dowels in its two pilots), foot on the ring by the side
   stem. That is the arm's front support; nothing stands in the lidar's front half. M3 x 30 up through the members into the feet; the floor
   screws down into the flanges. Printed upside down on the flange without supports.
 * Bumper: just for bumps (not structural): 3 mm PETG quarters, countersunk flat-head M3 from outside into the node arcs, lips on the arcs.
 * Nine frame parts (hub/cross, four ring edges, four nodes) joined by half-laps (11 + 11 mm, 2 x M3 each). Every part fits a 256 bed.
Clipped to stay 3 mm off everything that moves (droop..bump) and off each pod's drop-in path. Nothing printed or load-tested.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import LineString, Point, Polygon, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import first_floor as FF                                               # noqa: E402
import hardware as H                                                   # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import R_WHEEL, M3_PILOT, M3_CLR                             # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

OUT = W.ROOT / 'step_export' / 'chassis'
VAL = W.ROOT / 'validation' / 'chassis'
U, cyl, box, pan_screw = FF.U, W.cyl, P.box, FF.pan_screw
Z0, Z1 = 40.0, FF.PLATE_Z[1]               # members: bottom (ground clearance) and top (the pods' feet)
ZL = (Z0 + Z1) / 2                         # half-lap line
RING_IN, RING_OUT = 100.0, 121.0           # |x| + |y| band of the ring (its outer edge is the inner arms' cutout)
CROSS_W, HUB_R = 16.0, 20.0
STEM_W = dict(front=50.0, back=36.0, side=30.0)
ARC_IN = 201.0
ARCS = dict(front=(-30.0, 30.0), left=(62.0, 118.0), back=(150.0, 210.0), right=(242.0, 298.0))   # rim arcs; front/back +-30 deg: 225 mm chords print
CORNER = 26.0                              # ring edges end this far from the axes; the nodes own the ring corners
LAP = 12.0                                 # half-lap length either side of each split
NODES = ('front', 'left', 'back', 'right')
NODE_DEG = dict(front=0.0, left=90.0, back=180.0, right=270.0)
# pod pockets (pod frame; robot z = pod z + R_WHEEL)
POCKET_CLR, POCKET_WALL, POCKET_TOP = 0.2, 5.0, 6.0                    # wall top at pod z 6 = robot 76 (inner frame); keel walls to its top
# round 11 (Fernando): a brace per pod from the keel's pocket on the side node to the ring by the inner frame, parallel to the pivot axis
# behind the fork's base bar: it closes a triangle with the side stem and the rim arc, and ties the two pivot supports directly
BRACE = dict(x=(-43.5, 84.0), y=(96.0, 110.0))                          # pod frame; clipped 3 mm off everything that moves like every member
# round 12: each pod comes out radially (its -x) with the second floor on: its screws come up from under the members (heads 1.4 mm up in
# counterbores), and the members and pocket walls are clipped 1 mm off the path every pod part (all poses) sweeps going out
BELOW_CB = 3.9                             # counterbore from the members' underside for those heads (pan 6.5 x 2.5)
SLIDE_L, SLIDE_CLR = 260.0, 1.0
# lidar
LIDAR_X = 120.0
PED_FLANGE_R, PED_R, PED_WALL, PED_TOP_R, PED_SCREW_R = 24.0, 20.0, 3.0, 40.0, 17.0
# second-floor supports: printed fins, each in a vertical plane through the lidar's axis. u: distance from that axis along the fin's ray
FLOOR2_UNDER = 176.5                       # second floor underside (layout F; arm riser 62..205 x +-56 on top of it)
COLUMN_TOP = P.Z_TT + R_WHEEL + 0.02       # the pods' spring-column top (inner frame), 126.9; the side fin bears on it (0.02 keeps the booleans clean)
SUPPORT_PLANE_BAND = (148.0, 172.0)        # only the web crosses this band (robot z); the scan plane is 163
FIN_WEB, FIN_FLANGE = 5.0, 16.0           # web in the scan band; top flange under the floor
FIN_LEG = dict(spine=(4.0, 7.0), side=(4.0, 4.5))   # leg half width from z 120 up / at the deck (the side foot stands between the pods' forks)
SPINE = dict(bearing=180.0, top=(140.0, 336.0), feet=((155.0, 185.0), (300.0, 336.0)), apex=125.0, nose=166.0,
             screws=(163.0, 177.0, 310.0, 326.0), floor=(148.0, 195.0, 245.0, 290.0, 328.0))
SIDE = dict(bearing=136.7, top=(78.0, 225.0), column=(105.5, 133.5), arch=136.0, foot=(165.0, 175.0), leg_shift=-2.5, nose=166.0, tail=168.0,
            screws=(170.0,), floor=(86.0, 119.0, 160.0, 205.0))   # bearing: the ray through both column pilots; its tip is under the riser's corner
FIN_SCREW_L, FIN_PILOT_DEPTH, FLOOR_PILOT_DEPTH = 30.0, 12.0, 7.0      # deck feet: M3 x 30 up through the member; floor: M3 x 12 down
PIN_D, PIN_L, PIN_PROUD = 3.0, 10.0, 4.0  # steel dowels pressed into the column's two support pilots, locating the side fin's foot
# bumper: impact only
BUMPER_T = 3.0
CSK_D, CSK_DEPTH = 6.3, 1.65               # 90 deg countersink for an M3 flat head
FF.MOUNTS_Q0 = (8.0, 17.0, 72.0, 84.0)    # round 12: 25 -> 17, the front / back arcs now end at 20 deg (clear of the pods' way out)
FF.BRACKET_BASE_BACK = 16.0                # the cradle's base stops short of the lidar pedestal
COLOURS = dict(FF.COLOURS, hub=(0.50, 0.52, 0.56), edge=(0.70, 0.72, 0.62), node=(0.62, 0.68, 0.78), support=(0.85, 0.85, 0.88))


def colour(name):
    n = name.split('/')[-1]
    for k in ('hub', 'edge', 'node', 'support'):
        if n.startswith(k):
            return COLOURS[k]
    return FF.colour(name)


def diamond(r):
    return Polygon([(r, 0), (0, r), (-r, 0), (0, -r)])


def sector(a0, a1, r=400.0):
    return Polygon([(0, 0)] + [tuple(r * U(t)) for t in np.linspace(a0, a1, 24)])


def prism(geom, z0, z1):
    """round 12: opened by 0.05 first; the side nodes' outline (valid in shapely) made an invalid solid after the slide-path clip"""
    return B.solid_of(geom.buffer(-0.05, join_style=2).buffer(0.05, join_style=2), z0, z1)


# ---- pod pockets (pod frame) ------------------------------------------------------------------------------------------------------------------
def frame_walls():
    """walls hugging the inner frame: along its centre-side face and across its front end (round 12: no horizontal screws through them)"""
    x1 = L.FI_X[1] + POCKET_CLR
    z0, z1 = L.PLATE_TOP_L, POCKET_TOP
    side = box(x1, x1 + POCKET_WALL, L.FOOT_Y_L[0] - POCKET_CLR - POCKET_WALL, L.FOOT_Y_L[1], z0, z1)
    front = box(L.FI_X[0], x1 + POCKET_WALL, L.FOOT_Y_L[0] - POCKET_CLR - POCKET_WALL, L.FOOT_Y_L[0] - POCKET_CLR, z0, z1)
    return side + front, []


def keel_walls():
    kx, ky = L.KEEL_FOOT['x'], L.KEEL_FOOT['y']
    z0, z1 = L.PLATE_TOP_L, L.KEEL_TOP_Z                                # round 11: up to the keel's top (was its foot's, 7)
    back = box(kx[0], kx[1] + POCKET_CLR + POCKET_WALL, ky[1] + POCKET_CLR, ky[1] + POCKET_CLR + POCKET_WALL, z0, z1)
    end = box(kx[1] + POCKET_CLR, kx[1] + POCKET_CLR + POCKET_WALL, ky[0] + 2.0, ky[1] + POCKET_CLR + POCKET_WALL, z0, z1)   # 2 mm back: the base bar is 1.5 off the keel
    return back + end


def slide_path():
    """round 12, pod frame plan: what every pod part (droop, nominal, bump) sweeps going out along -x by SLIDE_L, + SLIDE_CLR, in the members'
    band (robot z Z0..Z1) and the pocket walls' (Z1..Z1 + 30). Cached; rebuilt when the pod's sources change"""
    import pickle
    srcs = [Path(__file__).with_name(n) for n in ('pod.py', 'pod_lean.py', 'dims.py', 'wheel_motor.py')]
    key = [round(s.stat().st_mtime) for s in srcs]
    cache = VAL / 'slide_path.pkl'
    if cache.exists():
        k, out = pickle.loads(cache.read_bytes())
        if k == key:
            return out
    sp, _ = L.build_static()
    unit = P.wheel_unit()
    bands = dict(members=(Z0, Z1 - 0.05), walls=(Z1 + 0.05, Z1 + 30.0))
    secs = {nm: [] for nm in bands}
    for a in L.POSES.values():
        sh, _ = L.posed(sp, unit, a)
        for k, s in sh.items():
            for nm, (z0, z1) in bands.items():
                g = B.proj(s, True, z0 - R_WHEEL, z1 - R_WHEEL)
                if g is not None:
                    secs[nm].append(g)
    out = {}
    for nm, gs in secs.items():
        g = union_all(gs)
        quads = [g, affinity.translate(g, -SLIDE_L, 0)]
        for q in getattr(g, 'geoms', [g]):
            for ring in [q.exterior] + list(q.interiors):
                c = list(ring.coords)
                quads += [Polygon([a, b_, (b_[0] - SLIDE_L, b_[1]), (a[0] - SLIDE_L, a[1])]).buffer(0) for a, b_ in zip(c[:-1], c[1:])]
        out[nm] = union_all(quads).buffer(SLIDE_CLR, 16)
    VAL.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(pickle.dumps((key, out)))
    return out


# ---- frame geometry (2D) ---------------------------------------------------------------------------------------------------------------------
def keel_foot():
    """the outer keel's foot in plan (pod frame), from 2.8 outboard of the keel (round 9's pad) to the foot's end"""
    (x0, x1), (y0, y1) = L.KEEL_FOOT['x'], L.KEEL_FOOT['y']
    return sbox(x0 - 2.8, y0, x1, y1)


def skeleton(fp, BR):
    rb = B.r_bumper(fp)
    rp = rb - 0.5
    cut = FF.placed(fp['cutout'])
    below = FF.placed(fp['bands']['below']).buffer(3.0)                 # moving parts in z 40..56, +3
    allowed = Point(0, 0).buffer(rp, 720).difference(cut).difference(below).difference(FF.placed(slide_path()['members']))
    ring = diamond(RING_OUT).difference(diamond(RING_IN))
    cross = union_all([sbox(-RING_IN - 2, -CROSS_W / 2, RING_IN + 2, CROSS_W / 2), sbox(-CROSS_W / 2, -RING_IN - 2, CROSS_W / 2, RING_IN + 2)])
    hub = Point(0, 0).buffer(HUB_R, 64)
    stems = [affinity.rotate(sbox(RING_IN, -STEM_W[k if k in ('front', 'back') else 'side'] / 2, rp, STEM_W[k if k in ('front', 'back') else 'side'] / 2),
                             NODE_DEG[k], origin=(0, 0)) for k in NODES]
    annulus = Point(0, 0).buffer(rp, 720).difference(Point(0, 0).buffer(ARC_IN, 720))
    arcs = [annulus.intersection(sector(*ARCS[k])) for k in NODES]
    keel_pad = FF.placed(keel_foot()).buffer(POCKET_CLR + POCKET_WALL + 1.0)
    brace = FF.placed(sbox(BRACE['x'][0], BRACE['y'][0], BRACE['x'][1], BRACE['y'][1]))
    cradle = sbox(*B.proj(BR['bracket'], False).buffer(3).bounds)
    ped = Point(LIDAR_X, 0).buffer(PED_FLANGE_R + 3.0, 64)
    sk = union_all([ring, cross, hub] + stems + arcs + [keel_pad, brace] + [cradle, ped]).intersection(allowed)
    sk = sk.buffer(-0.3, 16, join_style=2).buffer(0.3, 16, join_style=2)   # round 12: drop the slide path's slivers (< 0.6 mm)
    pieces = sorted(getattr(sk, 'geoms', [sk]), key=lambda g: -g.area)
    sk = union_all([g for g in pieces if g.area > 400])
    return sk, dict(rb=rb, rp=rp, cut=cut, allowed=allowed, pieces=[round(g.area) for g in pieces])


def split(sk):
    """the nine parts' regions (2D) and the half-lap zones between them: (zone, lower part, upper part)"""
    regions = {'hub': sk.intersection(diamond(RING_IN))}
    rest = sk.difference(diamond(RING_IN))
    quads = {'edge_FL': sbox(CORNER, CORNER, 400, 400), 'edge_RL': sbox(-400, CORNER, -CORNER, 400),
             'edge_RR': sbox(-400, -400, -CORNER, -CORNER), 'edge_FR': sbox(CORNER, -400, 400, -CORNER)}
    band = diamond(RING_OUT + 2).difference(diamond(RING_IN))
    for k, q in quads.items():
        regions[k] = rest.intersection(band).intersection(q)
    used = union_all([regions[k] for k in quads])
    for n in NODES:
        a = NODE_DEG[n]
        regions[f'node_{n}'] = rest.difference(used).intersection(sector(a - 45, a + 45))
    laps = []
    for n in NODES:
        a = NODE_DEG[n]
        z = diamond(RING_IN + LAP).difference(diamond(RING_IN - LAP)).intersection(sector(a - 20, a + 20)).intersection(sk)
        laps.append((z, 'hub', f'node_{n}'))
    for e, (sx, sy) in dict(edge_FL=(1, 1), edge_RL=(-1, 1), edge_RR=(-1, -1), edge_FR=(1, -1)).items():
        zy = affinity.scale(sbox(0 if sx > 0 else -400, CORNER - LAP, 400 if sx > 0 else 0, CORNER + LAP), 1, sy, origin=(0, 0)).intersection(band).intersection(sk)
        laps.append((zy, e, 'node_front' if sx > 0 else 'node_back'))
        zx = affinity.scale(sbox(CORNER - LAP, 0 if sy > 0 else -400, CORNER + LAP, 400 if sy > 0 else 0), sx, 1, origin=(0, 0)).intersection(band).intersection(sk)
        laps.append((zx, e, 'node_left' if sy > 0 else 'node_right'))
    return regions, laps


def lap_screws(laps):
    pts = []
    for z, lo, hi in laps:
        if z.is_empty:
            continue
        r = z.minimum_rotated_rectangle
        c = np.array(z.centroid.coords[0])
        xy = np.array(r.exterior.coords[:4])
        e = [xy[1] - xy[0], xy[2] - xy[1]]
        long_ = e[0] if np.linalg.norm(e[0]) > np.linalg.norm(e[1]) else e[1]
        u = long_ / np.linalg.norm(long_)
        for s in (-1, 1):
            p = c + s * u * min(0.25 * np.linalg.norm(long_), 9.0)
            if z.buffer(-2.5).contains(Point(*p)):
                pts.append((tuple(p), lo, hi))
    return pts


# ---- parts ----------------------------------------------------------------------------------------------------------------------------------------
def build_pedestal():
    z0, z1 = Z1, FF.LIDAR_BASE_Z
    c = (LIDAR_X, 0.0)
    p = cyl(PED_FLANGE_R, 4.0, (*c, z0 + 2.0)) + cyl(PED_R, z1 - z0, (*c, (z0 + z1) / 2)) + cyl(PED_TOP_R, 5.0, (*c, z1 - 2.5))
    p = p - cyl(PED_R - PED_WALL, z1 - z0 - 5.0 + 0.1, (*c, (z0 + z1 - 5.0) / 2))
    for a in (45, 135, 225, 315):
        x, y = np.array(c) + U(a) * PED_SCREW_R
        p = p - cyl(M3_CLR / 2, 4.2, (x, y, z0 + 2.0))
        x, y = np.array(c) + U(a) * FF.LIDAR_HOLES_R
        p = p - cyl(1.35, 5.2, (x, y, z1 - 2.5))
    p = p - box(c[0] - PED_R - 1, c[0] - PED_R + PED_WALL + 1, -6, 6, z1 - 30, z1 - 5)        # the lidar's lead out of the back of the tube
    p = p - cyl(8.0, 5.2, (*c, z1 - 2.5))
    return p


def build_bumpers(rb, rp, sk):
    ro = rb + BUMPER_T
    ring = prism(Point(0, 0).buffer(ro, 720).difference(Point(0, 0).buffer(rb, 720)), *FF.BUMPER_Z)
    ring = ring - prism(Polygon([(0, 0)] + [tuple(1.5 * ro * U(t)) for t in np.linspace(-FF.NOSE_DIP_DEG, FF.NOSE_DIP_DEG, 9)]), FF.NOSE_DIP_TOP, FF.BUMPER_Z[1] + 1)
    out, hw = {}, {}
    mounts = FF.mount_angles()
    for q in range(4):
        s = ring & prism(Polygon([(0, 0)] + [tuple(1.5 * ro * U(t)) for t in np.linspace(90 * q, 90 * q + 90, 12)]), 0, 200)
        for a in mounts:
            if not (90 * q < a < 90 * q + 90):
                continue
            if abs(((a + 180) % 360) - 180) >= FF.NOSE_DIP_DEG:
                foot = affinity.rotate(sbox(rp - FF.LIP_REACH, -FF.LIP_W / 2, rp, FF.LIP_W / 2), a, origin=(0, 0))
                if sk.buffer(0.5).contains(foot):
                    s = s + prism(affinity.rotate(sbox(rp - FF.LIP_REACH, -FF.LIP_W / 2, rb + 0.5, FF.LIP_W / 2), a, origin=(0, 0)), *FF.LIP_Z)
            org = U(a) * (ro + 0.01)
            pl = b.Plane(origin=(*org, FF.SCREW_Z), z_dir=(*(-U(a)), 0))
            s = s - b.Solid.make_cylinder(M3_CLR / 2, BUMPER_T + 1.0, pl) - b.Solid.make_cone(CSK_D / 2, M3_CLR / 2, CSK_DEPTH, pl)
            hw[f'bolt_bumper_{a:g}'] = b.Solid.make_cone(CSK_D / 2 - 0.15, 1.5, CSK_DEPTH, pl) + b.Solid.make_cylinder(1.5, 10.0, pl)
        out[f'bumper_q{q}'] = s
    return out, hw


def ellipse(c, a, bz, t0, t1, n=24):
    return [(c[0] + a * math.cos(math.radians(t)), c[1] + bz * math.sin(math.radians(t))) for t in np.linspace(t0, t1, n)]


def fin_profile(kind):
    """the fin's outline in its own plane (u, robot z): a top flange the whole length under the floor, legs on the chassis, elliptical arches
    between them and elliptical coves out to the tips; corners rounded"""
    zf, zd, zc = FLOOR2_UNDER, Z1, COLUMN_TOP
    if kind == 'spine':                    # two feet on the cross and the back node, arched between; the nose a cove out toward the hub
        S = SPINE
        (u0, u1), ((a0, a1), (b0, _)) = S['top'], S['feet']
        pts = [(u0, zf), (u1, zf), (u1, zd), (b0, zd)]
        pts += ellipse(((a1 + b0) / 2, zd), (b0 - a1) / 2, S['apex'] - zd, 0, 180)
        pts += [(a0, zd)] + ellipse((u0, zd), a0 - u0, S['nose'] - zd, 0, 90)
    else:                                  # side: nose under the arm riser's corner, bearing on the column top, an arch down to a foot on the ring
        S = SIDE                           # by the side stem, a cove out to the tail
        (u0, u1), (c0, c1), (f0, f1), ua = S['top'], S['column'], S['foot'], S['arch']   # the seat runs on past the column's end, then the arch
        pts = [(u0, zf), (u1, zf), (u1, S['tail'])]
        pts += ellipse((u1, zd), u1 - f1, S['tail'] - zd, 90, 180)
        pts += [(f0, zd)] + ellipse((ua, zd), f0 - ua, zc - zd, 0, 90)
        pts += [(c0, zc)] + ellipse((c0, S['nose']), c0 - u0, S['nose'] - zc, 270, 180)
    g = Polygon(pts).buffer(0)
    return g.buffer(-1.0, 32).buffer(1.0, 32).buffer(3.0, 32).buffer(-3.0, 32)


def fin_cross_section(kind, side=1):
    """across the fin against z: legs flaring out toward the deck (the side fin's leg also shifted off the rear pod's fork), only the web
    through the scan band, the flange under the floor"""
    (w_up, w_deck), w, f, (b0, b1) = FIN_LEG[kind], FIN_WEB / 2, FIN_FLANGE / 2, SUPPORT_PLANE_BAND
    half = [(w_deck, Z1 - 2.0), (w_up, 120.0), (w_up, b0 - 8.0), (w, b0), (w, b1), (f, b1 + 2.5), (f, FLOOR2_UNDER + 1.0)]
    shift = side * (SIDE['leg_shift'] if kind == 'side' else 0.0)
    dz = lambda z: shift * max(0.0, 120.0 - z) / (120.0 - Z1 + 2.0)                    # noqa: E731
    return Polygon([(x + dz(z), z) for x, z in half] + [(-x + dz(z), z) for x, z in reversed(half)])


def fin_axis(kind, side=1):
    bearing = SPINE['bearing'] if kind == 'spine' else side * SIDE['bearing']
    return np.array([LIDAR_X, 0.0]), U(bearing), bearing


def fin_xy(kind, u, side=1):
    lc, d, _ = fin_axis(kind, side)
    return lc + d * u


def build_fin(kind, side=1):
    """the fin solid in robot coordinates, its holes, hardware and its plan section at the scan plane"""
    prof = fin_profile(kind)
    S = SPINE if kind == 'spine' else SIDE
    w = FIN_FLANGE / 2 + 1.0
    s = B.solid_of(prof, -w, w).rotate(b.Axis.X, 90)                                  # (u, z, across) -> (u, -across, z)
    u0, u1 = S['top']
    s = s & B.solid_of(fin_cross_section(kind, side), u0 - 2.0, u1 + 2.0).rotate(b.Axis.X, 90).rotate(b.Axis.Z, 90)
    lc, d, bearing = fin_axis(kind, side)
    s = s.rotate(b.Axis.Z, bearing).moved(b.Location((*lc, 0)))
    hw = {}
    for n, u in enumerate(S['screws']):                                                 # deck feet: M3 x 30 up through the member into the leg
        x, y = fin_xy(kind, u, side)
        s = s - cyl(M3_PILOT / 2, FIN_PILOT_DEPTH, (x, y, Z1 + FIN_PILOT_DEPTH / 2 - 0.05))
    for u in S['floor']:                                                                # the floor: M3 x 12 down into the flange
        x, y = fin_xy(kind, u, side)
        s = s - cyl(M3_PILOT / 2, FLOOR_PILOT_DEPTH, (x, y, FLOOR2_UNDER - FLOOR_PILOT_DEPTH / 2 + 0.05))
    if kind == 'side':                                                                  # dowels in the column's two support pilots
        i = 0 if side > 0 else 3
        out = -B.frame(i)[1]                                                            # round 12: slots open toward the pod's way out
        for n, p in enumerate(L.SUPPORT_PILOTS):
            x, y = B.place2d(Point(*p), i).coords[0]
            slot = LineString([(x, y), (x + 15.0 * out[0], y + 15.0 * out[1])]).buffer((PIN_D + 0.1) / 2, 16, cap_style=2)
            s = s - B.solid_of(union_all([Point(x, y).buffer((PIN_D + 0.1) / 2, 32), slot]), COLUMN_TOP - 0.05, COLUMN_TOP + PIN_PROUD + 0.5)
            hw[f'bolt_pin_{B.NAMES[i]}_{n}'] = cyl(PIN_D / 2, PIN_L, (x, y, COLUMN_TOP + PIN_PROUD - PIN_L / 2))
    zp = FF.LIDAR['plane']
    band = prof.intersection(sbox(-1e3, zp - 0.5, 1e3, zp + 0.5)).bounds
    sec = affinity.translate(affinity.rotate(sbox(band[0], -FIN_WEB / 2, band[2], FIN_WEB / 2), bearing, origin=(0, 0)), *lc)
    return s, hw, sec


def build_supports():
    parts, hw, sections, screws = {}, {}, [], []
    for name, kind, side in (('support_spine', 'spine', 1), ('support_side_L', 'side', 1), ('support_side_R', 'side', -1)):
        s, h, sec = build_fin(kind, side)
        parts[name] = s
        hw.update(h)
        sections.append((name, sec))
        S = SPINE if kind == 'spine' else SIDE
        screws += [tuple(fin_xy(kind, u, side)) for u in S['screws']]
    return parts, hw, sections, screws


def build(fp):
    BR = FF.bracket_geometry()
    sk, g = skeleton(fp, BR)
    regions, laps = split(sk)
    lap_all = union_all([z for z, _, _ in laps if not z.is_empty])
    parts = {}
    for k, reg in regions.items():
        body = reg.difference(lap_all)
        s = prism(body, Z0, Z1) if not body.is_empty else None
        for z, lo, hi in laps:
            if z.is_empty or k not in (lo, hi):
                continue
            half = prism(z, Z0, ZL) if k == lo else prism(z, ZL, Z1)
            s = half if s is None else s + half
        parts[k] = s
    # pod pockets: walls placed per pod, clipped to the members under them and given to the part each piece stands on
    fw, fhw = frame_walls()
    kw = keel_walls()
    hw = {}
    tops = {}                                                           # where each part owns the top surface (laps: the upper half's part)
    for k, reg in regions.items():
        t = reg.difference(union_all([z for z, lo, _ in laps if lo == k and not z.is_empty] or [Point(9e3, 9e3).buffer(1)]))
        ups = [z for z, _, hi in laps if hi == k and not z.is_empty]
        tops[k] = union_all([t] + ups) if ups else t
    walls_out = FF.placed(slide_path()['walls'])                        # round 12: off every pod's way out
    for i in range(4):
        for nm, w in (('frame', fw), ('keel', kw)):
            pw = B.place3d(w, i) - prism(walls_out, Z1 - 2, Z1 + 32)
            for k, reg in tops.items():
                if reg.is_empty:
                    continue
                piece = pw & prism(reg, Z1 - 1, Z1 + 30)
                if piece is not None and piece.solids() and piece.volume > 1.0:
                    parts[k] = parts[k] + piece
    holes = dict(pod=[], keel=[], pedestal=[], bracket=[])
    for i in range(4):
        for p in [((L.FI_X[0] + L.FI_X[1]) / 2, y) for y in L.FOOT_SCREW_Y]:
            holes['pod'].append(tuple(np.asarray(B.place2d(Point(*p), i).coords[0])))
        for p in L.KEEL_SCREWS:
            holes['keel'].append(tuple(np.asarray(B.place2d(Point(*p), i).coords[0])))
    holes['pedestal'] = [tuple(np.array([LIDAR_X, 0.0]) + U(a) * PED_SCREW_R) for a in (45, 135, 225, 315)]
    holes['bracket'] = BR['deck_holes']
    lapped = lap_screws(laps)
    owner = lambda x, y: next((k for k, reg in regions.items() if reg.buffer(0.5).contains(Point(x, y))), None)   # noqa: E731
    for kind in ('pedestal', 'bracket'):
        for x, y in holes[kind]:
            k = owner(x, y)
            if k:
                parts[k] = parts[k] - cyl(M3_PILOT / 2, 16.0, (x, y, Z1 - 8.0))
    frame = [k for k in parts if k.startswith(('hub', 'edge', 'node'))]
    for kind, Ls in (('pod', L.FOOT_SCREW_L), ('keel', L.KEEL_SCREW_L)):   # round 12: up from under the members (laps: both halves)
        for n, (x, y) in enumerate(holes[kind]):
            hole = cyl(M3_CLR / 2, Z1 - Z0 + 0.2, (x, y, (Z0 + Z1) / 2)) + cyl(FF.CB_D / 2, BELOW_CB + 0.1, (x, y, Z0 + BELOW_CB / 2 - 0.05))
            for k in frame:
                if W.near(parts[k], hole, 0.0) and (parts[k] & hole).volume > 0.01:
                    parts[k] = parts[k] - hole
            hw[f'bolt_{kind}_{n}'] = pan_screw((x, y, Z0 + BELOW_CB), (0, 0, 1), Ls)
    for n, ((x, y), lo, hi) in enumerate(lapped):
        parts[hi] = parts[hi] - cyl(M3_CLR / 2, Z1 - ZL + 0.2, (x, y, (Z1 + ZL) / 2)) - cyl(FF.CB_D / 2, FF.CB_DEPTH + 0.1, (x, y, Z1 - FF.CB_DEPTH / 2 + 0.05))
        parts[lo] = parts[lo] - cyl(M3_PILOT / 2, ZL - Z0 - 1.0, (x, y, (ZL + Z0 + 1.0) / 2))
        hw[f'bolt_lap_{n}'] = pan_screw((x, y, Z1 - FF.CB_DEPTH), (0, 0, -1), 20.0)
    mounts = FF.mount_angles()
    for a in mounts:
        x, y = U(a) * (g['rp'] - 5)
        k = owner(x, y)
        if k:
            parts[k] = parts[k] - b.Solid.make_cylinder(M3_PILOT / 2, 10.5, b.Plane(origin=(*(U(a) * (g['rp'] + 0.1)), FF.SCREW_Z), z_dir=(*(-U(a)), 0)))
    for n, (x, y) in enumerate(holes['pedestal']):
        hw[f'bolt_pedestal_{n}'] = pan_screw((x, y, Z1 + 4.0), (0, 0, -1), 14.0)
    for n, a in enumerate((45, 135, 225, 315)):                         # round 12: the lidar's 4 x M2.5 x 8 (v10's) up through the top plate
        x, y = np.array([LIDAR_X, 0.0]) + U(a) * FF.LIDAR_HOLES_R      # into its base (thread depth VERIFY); driven on the bench before
        hw[f'bolt_lidar_{n}'] = H.solid(H.pan((x, y, FF.LIDAR_BASE_Z - 5.0), (0, 0, 1), 8.0, 'bolt_lidar', size=2.5))   # the pedestal goes on
    hw.update(BR['hardware'])
    parts['pedestal'] = build_pedestal()
    parts['bracket_d435'] = BR['bracket']
    bump, bhw = build_bumpers(g['rb'], g['rp'], sk)
    parts.update(bump)
    hw.update(bhw)
    sup, shw, sections, fin_screws = build_supports()
    for n, (x, y) in enumerate(fin_screws):                                             # up through the member (laps: both halves) into the fin's foot
        hole = cyl(M3_CLR / 2, Z1 - Z0 + 0.2, (x, y, (Z0 + Z1) / 2)) + cyl(FF.CB_D / 2, FF.CB_DEPTH + 0.1, (x, y, Z0 + FF.CB_DEPTH / 2 - 0.05))
        for k in frame:
            if W.near(parts[k], hole, 0.0) and (parts[k] & hole).volume > 0.01:
                parts[k] = parts[k] - hole
        hw[f'bolt_fin_{n}'] = pan_screw((x, y, Z0 + FF.CB_DEPTH), (0, 0, 1), FIN_SCREW_L)
    parts.update(sup)
    hw.update(shw)
    holes['fin'] = fin_screws
    env = dict(env_d435=BR['camera'], env_d435_plug=BR['plug'],
               env_lidar=cyl(FF.LIDAR['d'] / 2, FF.LIDAR['h'], (LIDAR_X, 0, FF.LIDAR_BASE_Z + FF.LIDAR['h'] / 2)),
               env_arm_riser=box(62.0, 205.0, -56.0, 56.0, FLOOR2_UNDER + 5.0, 218.5))                   # layout F: on the 5 mm floor (not drawn)
    g.update(sk=sk, regions=regions, laps=laps, holes=holes, lapped=lapped, mounts=mounts, sections=sections)
    return parts, hw, env, g


# ---- checks ----------------------------------------------------------------------------------------------------------------------------------
def check(parts, hw, env, g, fp):
    rep = dict(layout=dict(D=B.D_WHEEL, slide=B.SLIDE, robot_diameter=round(2 * (g['rb'] + BUMPER_T), 1)),
               frame=dict(member='solid square section, 22 mm deep (print with few walls, low infill)', ring_band=[RING_IN, RING_OUT], cross_w=CROSS_W))
    pcs = {}
    for k, s in parts.items():
        gp = B.proj(s, False)
        fits = []
        for ang in range(0, 90, 2):
            x0, y0, x1, y1 = affinity.rotate(gp, ang, origin=(0, 0)).bounds
            fits.append((max(x1 - x0, y1 - y0), sorted((x1 - x0, y1 - y0))))
        e = min(fits)[1]
        pcs[k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_cm3=round(s.volume / 1000, 1),
                      footprint_on_bed=[round(e[1], 1), round(e[0], 1)], height=round(s.bounding_box().size.Z, 1), bed_fit=bool(e[1] <= FF.BED))
    rep['printed_parts'] = pcs
    rep['all_fit_256_bed'] = all(v['bed_fit'] for v in pcs.values())
    frame = [k for k in parts if k.startswith(('hub', 'edge', 'node'))]
    rep['frame_volume_cm3'] = round(sum(parts[k].volume for k in frame) / 1000, 1)
    bands = {k: FF.placed(v) for k, v in fp['bands'].items()}
    cl = dict(frame_to_moving_z40_56=g['sk'].distance(bands['below']), frame_to_moving_z56_62=g['sk'].distance(bands['plate']),
              bumper_to_moving=Point(0, 0).buffer(g['rb'] + BUMPER_T, 720).difference(Point(0, 0).buffer(g['rb'], 720)).distance(bands['bumper']))
    # the pocket walls (z 62..76) against the pods' moving parts, in 3D, every pose (they sit beside the static frames)
    parts_pod, _ = L.build_static()
    unit = P.wheel_unit()
    walls = {}
    for i in range(4):
        walls[f'frame{i}'] = B.place3d(frame_walls()[0], i)
        walls[f'keel{i}'] = B.place3d(keel_walls(), i)
    near_items = dict(walls=None, camera=env['env_d435'], plug=env['env_d435_plug'], bracket=parts['bracket_d435'], lidar=env['env_lidar'],
                      pedestal=parts['pedestal'], supports=None)
    sups = {k: v for k, v in parts.items() if k.startswith('support')}
    static_pod = ('pod_frame', 'pod_keel', 'pod_sleeve', 'bolt_pivot', 'dowel', 'washer')
    dmin = {k: 1e9 for k in near_items}
    for a in L.POSES.values():
        sh, _ = L.posed(parts_pod, unit, a)
        for i in range(4):
            for k, s in sh.items():
                if k.startswith(static_pod):
                    continue
                plunger = k.startswith(('spring_plunger', 'die_spring'))     # they sit beside the column the side fins stand on
                ps = B.place3d(s, i)
                for nm, it in near_items.items():
                    if plunger and nm != 'supports':
                        continue
                    items = list(walls.values()) if nm == 'walls' else (list(sups.values()) if nm == 'supports' else [it])
                    for x in items:
                        if W.near(ps, x, 12):
                            dmin[nm] = min(dmin[nm], W.min_dist(ps, x))
    cl.update({f'{k}_to_moving': v for k, v in dmin.items()})
    # the fins against everything static: the pods' frames (the side fins sit on the inner frames' column tops), the chassis, lidar, cradle
    sh, _ = L.posed(parts_pod, unit, 0.0)
    pod_static = {f'{B.NAMES[i]}/{k}': B.place3d(s, i) for i in range(4) for k, s in sh.items() if k.startswith(static_pod)}
    others = {k: v for k, v in parts.items() if not k.startswith('support')}
    others.update(lidar=env['env_lidar'], d435=env['env_d435'], d435_plug=env['env_d435_plug'])
    ov, gap = {}, {}
    for nm, f in sups.items():
        for k, x in {**pod_static, **others}.items():
            if not W.near(f, x, 6):
                continue
            d = W.min_dist(f, x)
            gap[f'{nm} / {k}'] = round(d, 2)
            if d < 1e-3:
                v = f & x
                ov[f'{nm} / {k}'] = round(v.volume, 3) if v is not None else 0.0
    rep['supports_static'] = dict(overlap_mm3=ov, gaps_mm=gap,
                                  note='0 overlap everywhere; the side fins sit 0.02 above the column top (bearing there), the feet stand on the members')
    rep['clearance_to_moving_mm'] = {k: round(float(v), 2) for k, v in cl.items()}
    rep['lidar_to_d435_cradle_mm'] = round(W.min_dist(env['env_lidar'], parts['bracket_d435']), 2)
    rep['lidar_to_camera_mm'] = round(W.min_dist(env['env_lidar'], env['env_d435']), 2)
    rep['pedestal_to_cradle_mm'] = round(W.min_dist(parts['pedestal'], parts['bracket_d435']), 2)
    land = {k: f"{sum(g['sk'].buffer(-1.5).contains(Point(*p)) for p in g['holes'][k])}/{len(g['holes'][k])}" for k in g['holes']}
    land['bumper'] = f"{sum(g['sk'].contains(Point(*(U(a) * (g['rp'] - 6)))) for a in g['mounts'])}/{len(g['mounts'])}"
    land['laps'] = f"{len(g['lapped'])} screws in {sum(1 for z, _, _ in g['laps'] if not z.is_empty)} laps"
    rep['fasteners_landing'] = land
    rep['per_pod'] = dict(inner_frame=f'3 x M3 x {L.FOOT_SCREW_L:g} up from under the member into its foot; pocket walls on its centre-side face and front end (0.2 clearance, 14 tall)',
                          keel=f'2 x M3 x {L.KEEL_SCREW_L:g} up from under the member; pocket walls behind it and at its inboard end (clipped off the pod\'s way out)',
                          pivot_block='bears on the ring',
                          supports='front pods: the side fin bears on the column top, located by 2 dowels in its 2 pilots (slots in the fin open toward the '
                                   'pod\'s way out); rear pods: pilots unused',
                          removal='screws out from below, bumper quarter off, unplug the motor; the pod slides out radially (its -x) under the second floor')
    foot = Polygon([(66.6, 18.0), (78.6, 18.0), (78.6, 92.0), (66.6, 92.0)])
    rep['inner_frame_foot_supported'] = [round(B.place2d(foot, i).intersection(g['sk']).area / foot.area, 2) for i in range(4)]
    keel = keel_foot()
    rep['keel_foot_supported'] = [round(B.place2d(keel, i).intersection(g['sk']).area / keel.area, 2) for i in range(4)]
    lc = np.array([LIDAR_X, 0.0])
    sh_ = []
    for nm, sec in g['sections']:
        c = np.array(sec.centroid.coords[0])
        a0 = math.degrees(math.atan2(c[1] - lc[1], c[0] - lc[0]))
        angs = [math.degrees(math.atan2(y - lc[1], x - lc[0])) for x, y in sec.exterior.coords]
        w = max(abs(((v - a0) + 180) % 360 - 180) for v in angs) * 2
        sh_.append(dict(support=nm, nearest=round(Point(*lc).distance(sec), 1), bearing=round(a0, 1), shadow_deg=round(w, 1)))
    rep['lidar'] = dict(at=[LIDAR_X, 0], scan_z=FF.LIDAR['plane'], supports=sh_, blind_total_deg=round(sum(s['shadow_deg'] for s in sh_), 1),
                        note="only the fins' 5 mm webs cross z 148..172, each in the plane of a ray; plus the arm while it reaches down in front")
    rep['supports'] = dict(
        spine=dict(bearing=SPINE['bearing'], length=SPINE['top'][1] - SPINE['top'][0], feet='cross by the hub + back node, 2 x M3 x 30 up through each',
                   arch_apex_z=SPINE['apex']),
        side=dict(bearing=[SIDE['bearing'], -SIDE['bearing']], length=SIDE['top'][1] - SIDE['top'][0],
                  feet="front pod's column top (bears, 2 dowels) + the ring by the side stem (1 x M3 x 30 up through it)",
                  nose_xy=[round(float(v), 1) for v in fin_xy('side', SIDE['top'][0])], arm_riser_rear_corner=[62.0, 56.0]),
        floor=f'M3 x 12 down through the 5 mm floor into Ø{M3_PILOT} x {FLOOR_PILOT_DEPTH:g} pilots in the flanges ({len(SPINE["floor"]) + 2 * len(SIDE["floor"])})',
        section=f'web {FIN_WEB:g} through z {SUPPORT_PLANE_BAND[0]:g}..{SUPPORT_PLANE_BAND[1]:g}, flange {FIN_FLANGE:g} under the floor, legs flaring toward the deck',
        printing='upside down on the top flange: arches, coves and leg flares all lean inward, no supports')
    rep['section_I_mm4'] = dict(plate_neck_19x6=round(19 * 6 ** 3 / 12), plate_neck_25x6=round(25 * 6 ** 3 / 12),
                                ring_21x22_solid=round(21 / math.sqrt(2) * 22 ** 3 / 12 * math.sqrt(2)), cross_16x22_solid=round(16 * 22 ** 3 / 12),
                                note='printed with 3-4 walls and low infill a solid square section is a box section: close to these numbers, stiffer than the U-channel')
    return rep


# ---- export / render ----------------------------------------------------------------------------------------------------------------------------
def export(parts, env):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    for k, s in parts.items():
        write_part(s, colour(k), OUT / 'parts' / f'{k}.step', k)
    items = {k: (s, colour(k)) for k, s in parts.items()}
    items.update({k: (s, COLOURS['support'] if k.startswith('support') else FF.COLOURS['env']) for k, s in env.items()})
    pod = FF.pod_items()
    for i in range(4):
        for k, s in pod.items():
            items[f'{B.NAMES[i]}/{k}'] = (B.place3d(s, i), L.colour(k))
    write_step(items, OUT / 'chassis_assembly.step', 'chassis')


def render(parts, hw, env, g, fp, rep=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MP
    for f in VAL.glob('*.png'):
        f.unlink()

    def fill(ax, geom, **kw):
        for q in getattr(geom, 'geoms', [geom]):
            if q.is_empty or q.geom_type != 'Polygon':
                continue
            ax.add_patch(MP(np.asarray(q.exterior.coords), closed=True, **kw))
            for h in q.interiors:
                ax.add_patch(MP(np.asarray(h.coords), closed=True, facecolor='white', edgecolor=kw.get('edgecolor', 'k'), lw=0.4))

    fig, axs = plt.subplots(1, 2, figsize=(22, 11))
    lc = np.array([LIDAR_X, 0.0])
    for ax, with_pods in zip(axs, (False, True)):
        rb = g['rb']
        ax.add_patch(Circle((0, 0), rb + BUMPER_T, facecolor=(0.95, 0.6, 0.3), edgecolor='none'))
        ax.add_patch(Circle((0, 0), rb, facecolor='white', edgecolor='none'))
        if with_pods:
            for i in range(4):
                fill(ax, B.place2d(fp['moving'], i), facecolor=(0.95, 0.75, 0.15, 0.35), edgecolor=(0.6, 0.45, 0), lw=0.4)
        for k, reg in g['regions'].items():
            fill(ax, reg, facecolor=colour(k), edgecolor='k', lw=0.6)
        for z, _, _ in g['laps']:
            fill(ax, z, facecolor='none', edgecolor='r', lw=0.8, hatch='////')
        for i in range(4):
            fill(ax, B.proj(B.place3d(frame_walls()[0], i), False), facecolor=(0.3, 0.3, 0.35), edgecolor='k', lw=0.3)
            fill(ax, B.proj(B.place3d(keel_walls(), i), False), facecolor=(0.3, 0.3, 0.35), edgecolor='k', lw=0.3)
        if with_pods:
            for i in range(4):
                fill(ax, B.place2d(fp['static'], i), facecolor=(0.25, 0.68, 0.74, 0.75), edgecolor=(0.1, 0.35, 0.4), lw=0.4)
                A = B.frame(i)[0]
                ax.text(*(A * 1.13), B.NAMES[i], fontsize=11, weight='bold', ha='center')
        fill(ax, B.proj(parts['bracket_d435'], False), facecolor=FF.COLOURS['bracket'], edgecolor='k')
        ax.add_patch(Circle(tuple(lc), FF.LIDAR['d'] / 2, facecolor=(0.1, 0.1, 0.1, 0.25), edgecolor='k', lw=1.0))
        for nm, sec in g['sections']:
            fill(ax, sec, facecolor='k', edgecolor='k')
            c = sec.centroid.coords[0]
            angs = [math.atan2(y, x - LIDAR_X) for x, y in sec.exterior.coords]
            a0 = math.atan2(c[1], c[0] - LIDAR_X)
            dev = [((v - a0) + math.pi) % (2 * math.pi) - math.pi for v in angs]
            for a in (a0 + min(dev), a0 + max(dev)):
                ax.plot([LIDAR_X, LIDAR_X + 380 * math.cos(a)], [0, 380 * math.sin(a)], color='0.55', lw=0.4)
        for k, s in hw.items():
            bb = s.bounding_box()
            ax.plot((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, 'm.', ms=4)
        if not with_pods:
            for k, reg in g['regions'].items():
                c = reg.representative_point()
                ax.text(c.x, c.y, k.replace('_', '\n'), fontsize=8, ha='center', va='center', color='0.15')
        ax.set_xlim(-240, 240)
        ax.set_ylim(-240, 240)
        ax.set_aspect('equal')
        ax.grid(True, lw=0.3)
        blind = rep['lidar']['blind_total_deg'] if rep else ''
        ax.set_title(('chassis (front = +x): hub + cross, 4 ring edges, 4 nodes; red hatch: half-laps; dark: pod pocket walls;\n'
                      f'lidar at x {LIDAR_X:g} (circle); black: the second-floor fins at the scan plane, each along a ray (blind {blind} deg)')
                     if not with_pods else 'with the pods: teal = static pod parts in their pockets, yellow = moving (swept)', fontsize=10)
    fig.tight_layout()
    fig.savefig(VAL / 'plan.png', dpi=85)
    plt.close(fig)
    fig, axs = plt.subplots(3, 1, figsize=(13, 13))
    for ax, (kind, title) in zip(axs, (('spine', 'spine (bearing 180, on the centreline behind the lidar): feet on the cross by the hub and on the back node'),
                                        ('side', f'side fins (bearing +-{SIDE["bearing"]:g}): nose under the arm riser corner, seat on the front pod column top, '
                                                 'foot on the ring by the side stem'),
                                        (None, 'cross-sections: spine | side (leg shifted off the rear pod fork)'))):
        if kind is None:
            for n, (k, sd) in enumerate((('spine', 1), ('side', 1))):
                fill(ax, affinity.translate(fin_cross_section(k, sd), 40 * n, 0), facecolor=COLOURS['support'], edgecolor='k')
            ax.set_xlim(-20, 60)
        else:
            fill(ax, fin_profile(kind), facecolor=COLOURS['support'], edgecolor='k')
            S = SPINE if kind == 'spine' else SIDE
            for u in S['screws']:
                ax.plot([u, u], [Z0, Z1 + FIN_PILOT_DEPTH], 'm-', lw=2)
            for u in S['floor']:
                ax.plot([u, u], [FLOOR2_UNDER + 5, FLOOR2_UNDER - FLOOR_PILOT_DEPTH], 'm-', lw=1.5)
            if kind == 'side':
                ax.add_patch(plt.Rectangle((SIDE['column'][0] - 1, COLUMN_TOP - 25), SIDE['column'][1] - SIDE['column'][0] + 2, 25, facecolor=(0.25, 0.68, 0.74)))
                ax.text(sum(SIDE['column']) / 2, COLUMN_TOP - 14, 'column top', ha='center', fontsize=8)
            ax.set_xlim(S['top'][0] - 15, S['top'][1] + 15)
        ax.axhspan(*SUPPORT_PLANE_BAND, color=(0.2, 0.7, 0.2, 0.15))
        ax.axhline(FF.LIDAR['plane'], color='g', lw=0.6)
        ax.axhline(Z1, color='0.3', lw=0.8)
        ax.axhline(FLOOR2_UNDER, color='0.3', lw=0.8, ls='--')
        ax.set_aspect('equal')
        ax.grid(True, lw=0.3)
        ax.set_title(title + ('' if kind is None else '; u = distance from the lidar axis; green: scan band (web only), magenta: screws'), fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'fin_profiles.png', dpi=85)
    plt.close(fig)
    items = dict(parts)
    items.update({k: v for k, v in env.items() if k != 'env_arm_riser'})
    items.update(hw)
    L.iso(items, VAL / 'iso_chassis.png', 215, 35, 'chassis parts (pods hidden)', colour, tol=0.6)
    L.iso({k: v for k, v in items.items() if not k.startswith(('bumper', 'bolt_bumper'))}, VAL / 'iso_chassis_below.png', 200, -35,
          'chassis from below (bumper hidden): solid 22 mm members, half-laps', colour, tol=0.6)
    full = dict(items)
    full['env_arm_riser'] = env['env_arm_riser']
    pod = {k: v for k, v in FF.pod_items().items() if not k.startswith(('roller_core', 'die_spring'))}
    for i in range(4):
        for k, s in pod.items():
            full[f'{B.NAMES[i]}/{k}'] = B.place3d(s, i)
    L.iso(full, VAL / 'iso_with_pods.png', 225, 28, 'chassis with the pods, lidar forward, second-floor fins (floor not drawn; dark box: the arm riser on it)',
          colour, tol=0.8)
    side = {k: v for k, v in full.items() if not k.startswith(('bumper', 'bolt_bumper'))}
    L.iso(side, VAL / 'iso_supports.png', 150, 10, 'second-floor fins from the rear left, bumper hidden (floor not drawn; dark box: the arm riser on it)',
          colour, tol=0.8)
    pocket = {k: v for k, v in full.items() if k.startswith(('edge_FL', 'node_front', 'node_left', 'FL/pod_frame', 'FL/pod_keel', 'bolt_side_FL', 'bolt_pod', 'bolt_keel'))}
    L.iso(pocket, VAL / 'iso_pod_pocket_FL.png', 250, 30, 'FL pod static parts in their pockets (moving parts hidden)', colour, tol=0.4)


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    fp = FF.pod_geometry()
    parts, hw, env, g = build(fp)
    for k, s in parts.items():
        print(k, len(s.solids()), s.is_valid, round(s.volume / 1000, 1), flush=True)
    rep = None
    if stage in ('check', 'all'):
        rep = check(parts, hw, env, g, fp)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps({k: v for k, v in rep.items() if k != 'printed_parts'}, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(parts, env)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(parts, hw, env, g, fp, rep)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
