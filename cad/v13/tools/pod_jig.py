"""cad/v13 bench stand for one wheel pod (Nov 11 2026, Fernando). build123d, one printed part and its mirror image; not part of the robot.

    python -B pod_jig.py [build|check|export|render|all]        (omni-robot-sim interpreter)

A pod has two static parts, and on the robot the chassis holds both: the inner frame (beside the motor) stands in a pocket on the ring, the outer
keel (inside the outer arm) in a pocket on the side node, and a brace behind the fork's base bar ties the two pivot supports (chassis.py). The jig
is that piece of chassis on its own, standing on the bench:
 * SEAT: the chassis' frame pad, keel pad and brace in plan (chassis.frame_outline / keel_outline / keel_foot / BRACE), inside corners rounded,
   clipped POD_JIG_CLR off everything that moves (droop, nominal, bump), with the chassis' pocket walls (0.2 clearance). Its top is the members'
   top (pod z -8). Six screws come up from the underside: 3 into the inner frame's foot nuts, the column bolt into its nut, 2 into the keel's
   Ø2.9 pilots. Nov 11 (Fernando: long screws are fine here): each tip lands where the robot's does, but the screws are longer
   (POD_JIG_SCREW_L: M3 x 70 / 100 / 80), so the heads seat 6-11 mm up a Ø7.2 counterbore from the bench (the first draft used the robot's
   lengths, heads 51 mm up a tunnel).
 * LEGS: the seat runs straight down to the bench (one vertical prism, so the jig prints upright on its base without support); print it with low
   infill.
 * BASE: one plate on the bench, POD_JIG_RING_W round the moving parts' plan with that plan cut out, fused to the seat along every edge they share
   (Nov 11, Fernando: the first draft's ring left a slit beside the keel's block). The wheel and motor sit ahead of the seat (it starts at pod
   y 13), so without it the stand tips forward.
The wheel hangs free (POD_JIG_WHEEL_LIFT under it at droop, the unloaded pod's pose): it spins, and pushed up from below the fork runs through
droop..bump against its own stops. The pod drops in from above. FL / RR pods (the mirror-image hand, pod_m) take jig_pod_m.
Everything is nominal CAD; nothing has been printed. Dimensions in dims.py (POD_JIG_*).
"""
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import union_all
from shapely.geometry import Point, MultiPoint, Polygon, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import chassis as C                                                    # noqa: E402
import hardware as H                                                   # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part, write_step                         # noqa: E402

cyl, box, overlap, min_dist, near = W.cyl, P.box, W.overlap, W.min_dist, W.near
OUT = W.ROOT / 'step_export' / 'pod_jig'
VAL = W.ROOT / 'validation' / 'pod_jig'
JIG_COLOUR = (0.95, 0.72, 0.18)
Z_BENCH = -R_WHEEL - DROOP - POD_JIG_WHEEL_LIFT      # pod frame: the bench the jig stands on
Z_SLAB = C.Z0 - R_WHEEL                              # -30: the chassis members' underside
Z_TOP = L.PLATE_TOP_L                                # -8: the members' top, where the pod's feet stand
Z_HEAD = Z_SLAB + C.BELOW_CB                         # where each screw head seats (as on the robot)
WALL_TOP = L.KEEL_TOP_Z + 2.0                        # the walls' keep-out band reaches this high (keel walls end at its top, 16)
BED = 250.0


def colour(name):
    if name.startswith('jig_pod'):
        return JIG_COLOUR
    if name.startswith('bolt_jig'):
        return H.MAGENTA
    return L.colour(name)


# ---- the pod -------------------------------------------------------------------------------------------------------------------------------
def pod():
    parts, _ = L.build_static()
    return parts, P.wheel_unit()


def keepout(parts, unit):
    """pod-frame plan of every moving part over the three poses, in the jig body's band (bench .. member top + 0.5: the outer arm hangs 0.07 under
    it at droop) and the pocket walls' band, + POD_JIG_CLR. Cached; rebuilt when the pod's sources or the bands change"""
    srcs = [Path(__file__).with_name(n) for n in ('pod.py', 'pod_lean.py', 'dims.py', 'wheel_motor.py')]
    key = [round(s.stat().st_mtime) for s in srcs] + [Z_BENCH, Z_TOP, WALL_TOP, POD_JIG_CLR]
    cache = VAL / 'keepout.pkl'
    if cache.exists():
        k, out = pickle.loads(cache.read_bytes())
        if k == key:
            return out
    bands = dict(body=(Z_BENCH - 1.0, Z_TOP + 0.5), walls=(Z_TOP + 0.05, WALL_TOP))
    secs = {nm: [] for nm in bands}
    for a in L.POSES.values():
        sh, _ = L.posed(parts, unit, a)
        for k, s in sh.items():
            if k.startswith(B.MOVING_SKIP):
                continue
            for nm, (z0, z1) in bands.items():
                g = B.proj(s, k.startswith(B.EXACT), z0, z1)
                if g is not None:
                    secs[nm].append(g)
    out = {nm: union_all(gs).buffer(POD_JIG_CLR, 16) for nm, gs in secs.items()}
    VAL.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(pickle.dumps((key, out)))
    return out


def screws():
    """the six screws as hardware records: 3 foot screws + the column bolt into the inner frame's nuts, 2 into the keel's pilots. Nov 11 (Fernando: long
    screws are fine here): each tip lands where the robot's screw tip does (its head at Z_HEAD, chassis.build), but the screw is POD_JIG_SCREW_L long, so
    its head seats near the bench"""
    xm = sum(L.FI_X) / 2
    rows = [((xm, y), L.FOOT_SCREW_L, 'foot') for y in L.FOOT_SCREW_Y] + [(p, L.FRAME_BOLT_L, 'column') for p in L.FRAME_BOLTS] \
        + [(p, L.KEEL_SCREW_L, 'keel') for p in L.KEEL_SCREWS]
    recs = []
    for (x, y), robot_l, kind in rows:
        jl = POD_JIG_SCREW_L[kind]
        recs.append(H.pan((x, y, Z_HEAD + robot_l - jl), (0, 0, 1), jl, f'bolt_jig_{kind}'))
    return recs


# ---- the jig -------------------------------------------------------------------------------------------------------------------------------
def seat_plan(ko):
    """the chassis' pads under the two static parts and the brace between them, inside corners rounded, off the moving parts, no thin pieces"""
    brace = sbox(C.BRACE['x'][0], C.BRACE['y'][0], C.BRACE['x'][1], C.BRACE['y'][1])
    pads = [C.frame_outline(), C.keel_outline(), C.keel_foot()]
    g = union_all(pads + [brace]).buffer(C.FRAME_FILLET, 32).buffer(-C.FRAME_FILLET, 32)
    g = union_all([g] + pads).difference(ko['body'])
    g = C.no_thin(g)
    pieces = sorted(getattr(g, 'geoms', [g]), key=lambda q: -q.area)
    return pieces[0], [round(q.area) for q in pieces]


def ring_plan(ko, seat):
    """the base on the bench: a rounded rectangle POD_JIG_RING_W outside the moving parts' plan and the seat, with that plan cut out of it, so the base is
    one plate fused to the seat along every edge they share. Nov 11 (Fernando: a gap between the base square and the bulk): the first base was a ring
    POD_JIG_RING_W wide round the plan's bounding box; on the keel side its bar ran outside the outer arm (x -55.5) and the keel's block starts at -45.3,
    so a 10 mm slit separated them everywhere but along the rear bar"""
    kx0, ky0, kx1, _ = ko['body'].bounds
    sx0, _, sx1, sy1 = seat.bounds
    w = POD_JIG_RING_W
    outer = C.rounded(sbox(min(kx0 - w, sx0), ky0 - w, max(kx1 + w, sx1), sy1), POD_JIG_CORNER_R)
    base = C.no_thin(outer.difference(ko['body']))
    base = union_all([base, seat])
    pieces = sorted(getattr(base, 'geoms', [base]), key=lambda q: -q.area)
    return pieces[0], [round(q.area) for q in pieces]


def chamfer_bottom(s):
    try:
        return b.chamfer([e for e in s.edges() if all(abs(e.position_at(t).Z - Z_BENCH) < 1e-3 for t in (0, 0.5, 1))], POD_JIG_EDGE_C)
    except Exception:                                                  # noqa: BLE001
        return s


def build_jig(ko):
    seat, pieces = seat_plan(ko)
    base, base_pieces = ring_plan(ko, seat)
    j = B.solid_of(seat, Z_BENCH, Z_TOP) + B.solid_of(base, Z_BENCH, Z_BENCH + POD_JIG_RING_T)
    j = chamfer_bottom(j)
    cav = Polygon()
    if POD_JIG_SHELL > 0:                                              # Nov 12: closed pockets in the legs, base plate under, slab over
        holes = union_all([Point(r['p'][0], r['p'][1]).buffer(POD_JIG_TUNNEL_D / 2 + POD_JIG_HOLE_WALL, 32) for r in screws()])
        cav = C.no_thin(seat.buffer(-POD_JIG_SHELL, 32, join_style=2).difference(holes))
        cav = union_all([q for q in getattr(cav, 'geoms', [cav]) if q.area > 30.0] or [Polygon()])
        if not cav.is_empty:
            j = j - B.solid_of(cav, Z_BENCH + POD_JIG_RING_T, Z_SLAB)
    walls = C.frame_walls()[0] + C.keel_walls()
    walls = walls - B.solid_of(ko['walls'], Z_TOP - 1.0, WALL_TOP + 5.0)
    j = j + walls
    c = POD_JIG_EDGE_C
    for r in screws():
        x, y, zh = r['p']
        j = j - cyl(M3_CLR / 2, Z_TOP - zh + 0.2, (x, y, (Z_TOP + zh) / 2))
        j = j - cyl(POD_JIG_TUNNEL_D / 2, zh - Z_BENCH + 0.1, (x, y, (zh + Z_BENCH) / 2 - 0.05))
        j = j - b.Cone(POD_JIG_TUNNEL_D / 2 + c, POD_JIG_TUNNEL_D / 2 - 0.01, c).moved(b.Location((x, y, Z_BENCH + c / 2 - 0.001)))
    slits = base.buffer(6.0, 16).buffer(-6.0, 16).difference(base).difference(ko['body'])   # gaps under 12 mm wide between base and seat, outside the keep-out
    widest = 0.0                                                       # widest bridge over a pocket: twice its largest inscribed radius
    for q in getattr(cav, 'geoms', [cav]):
        if not q.is_empty:
            from shapely import maximum_inscribed_circle
            widest = max(widest, 2 * maximum_inscribed_circle(q, 0.1).length)
    return j, dict(pockets=len(getattr(cav, 'geoms', [cav])) if not cav.is_empty else 0, pocket_area_mm2=round(cav.area),
                   pocket_volume_cm3=round(cav.area * (Z_SLAB - Z_BENCH - POD_JIG_RING_T) / 1000, 1), widest_pocket_roof_mm=round(widest, 1),
                   seat_pieces_mm2=pieces, seat_area_mm2=round(seat.area), base_area_mm2=round(base.area), base_pieces_mm2=base_pieces,
                   base_slits_mm2=round(slits.area, 1), base_and_seat_overlap_mm2=round(base.intersection(seat).area))


def build_all():
    VAL.mkdir(parents=True, exist_ok=True)
    parts, unit = pod()
    ko = keepout(parts, unit)
    jig, info = build_jig(ko)
    return parts, unit, ko, jig, info


# ---- checks --------------------------------------------------------------------------------------------------------------------------------
def check(parts, unit, ko, jig, info):
    rep = dict(build=info, z=dict(bench=Z_BENCH, member_underside=Z_SLAB, top=Z_TOP, screw_heads=round(Z_HEAD, 2)))
    bb = jig.bounding_box()
    rep['jig'] = dict(solids=len(jig.solids()), valid=bool(jig.is_valid), volume_cm3=round(jig.volume / 1000, 1),
                      bbox=[round(v, 1) for v in tuple(bb.size)], fits_bed=bool(max(bb.size.X, bb.size.Y) <= BED),
                      overhang_mm2_as_printed=L.overhang_mm2(jig.rotate(b.Axis.Y, 90), 1))   # printed on its base, +z up (z -> x for the helper)
    # the static parts stand on it; nothing overlaps
    sh0, _ = L.posed(parts, unit, 0.0)
    rep['static_parts'] = {k: dict(overlap_mm3=round(overlap(jig, sh0[k]), 3), gap_mm=round(min_dist(jig, sh0[k]), 3))
                           for k in ('pod_frame_inner', 'pod_keel_outer', 'pod_sleeve', 'dowel_inner', 'washer_pivot_outer', 'bolt_pivot_outer')}
    # moving parts, every pose
    poses = {}
    for name, a in L.POSES.items():
        sh, _ = L.posed(parts, unit, a)
        mov = {k: s for k, s in sh.items() if not k.startswith(L.STATIC) and not k.startswith(('spring_plunger', 'die_spring'))}
        ov = {k: round(overlap(jig, s), 3) for k, s in mov.items() if near(jig, s)}
        ds = {k: min_dist(jig, s) for k, s in mov.items() if near(jig, s, 10.0)}
        k_min = min(ds, key=ds.get) if ds else None
        lowest = min(s.bounding_box().min.Z for k, s in mov.items())
        poses[name] = dict(overlap_mm3={k: v for k, v in ov.items() if v > 0.0}, nearest_moving=k_min,
                           nearest_mm=round(ds[k_min], 2) if k_min else None, lowest_moving_z=round(lowest, 2),
                           to_bench_mm=round(lowest - Z_BENCH, 2))
    rep['moving_parts'] = poses
    # the pod drops in from above (droop: the unloaded pose) with its feet into the pockets
    shd, _ = L.posed(parts, unit, L.POSES['droop'])
    allp = {k: s for k, s in shd.items() if not k.startswith(('spring_plunger', 'die_spring'))}
    rep['drop_in_from_above_overlap_mm3'] = {f'{d:g}': round(sum(overlap(jig, s.moved(b.Location((0, 0, d)))) for s in allp.values()
                                                                if near(jig, s.moved(b.Location((0, 0, d))))), 3)
                                             for d in (0.5, 2, 5, 10, 20, 40, 80, 120)}
    # screws: holes, thread, driver access from below
    recs = screws()
    aud = H.audit(recs, dict(jig_pod=jig, pod_frame_inner=sh0['pod_frame_inner'], pod_keel_outer=sh0['pod_keel_outer']),
                  up_of=lambda n: {'jig_pod': (0, 0, 1), 'pod_frame_inner': (1, 0, 0), 'pod_keel_outer': (1, 0, 0)}.get(n))
    rep['screw_audit'] = aud
    nuts = L.nuts()
    tips = []
    for r in recs:
        tip = r['p'][2] + r['L']
        row = dict(name=r['name'], xy=[round(v, 1) for v in r['p'][:2]], L=r['L'], tip_z=round(tip, 2))
        nut = next((n for n in nuts if math.hypot(n['p'][0] - r['p'][0], n['p'][1] - r['p'][1]) < 0.1), None)
        if nut:
            row['past_nut_top_mm'] = round(tip - (nut['p'][2] + 2.4), 2)
        tips.append(row)
    rep['screws'] = tips
    driver = {}
    for r in recs:
        x, y, z = r['p']
        bit = cyl(3.0, z - 2.5 - Z_BENCH + 20.0, (x, y, (z - 2.5 + Z_BENCH - 20.0) / 2))   # Ø6 driver from the head to 20 below the bench
        driver[f"{r['name']} ({x:.1f}, {y:.1f})"] = round(overlap(jig, bit), 3)
    rep['driver_from_below_overlap_mm3 (Ø6)'] = driver
    rep['driver_reach_mm (bench to the head)'] = {r['name']: round(r['p'][2] - 2.5 - Z_BENCH, 1) for r in recs}
    rep['screw_lengths'] = {k: f'M3 x {v:g}' for k, v in POD_JIG_SCREW_L.items()}
    # stability: the plan points that carry the weight inside the base's footprint
    foot = B.proj(jig, True, Z_BENCH - 0.1, Z_BENCH + 0.5, tol=0.2)
    hull = MultiPoint(np.array(foot.convex_hull.exterior.coords)).convex_hull
    motor = sh0['ak40_stator'].bounding_box().center()
    rep['stability (distance inside the base footprint, mm)'] = dict(axle=round(hull.exterior.distance(Point(0, 0)) * (1 if hull.contains(Point(0, 0)) else -1), 1),
                                                                     motor=round(hull.exterior.distance(Point(motor.X, motor.Y)) * (1 if hull.contains(Point(motor.X, motor.Y)) else -1), 1),
                                                                     footprint_bbox=[round(v, 1) for v in hull.bounds])
    return rep


# ---- export / render -----------------------------------------------------------------------------------------------------------------------
STEP_SKIP = ('bolt_tie', 'bearing_', 'spring_plunger', 'dowel', 'washer', 'die_spring')


def export(parts, unit, jig):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    write_part(jig, JIG_COLOUR, OUT / 'parts' / 'jig_pod.step', 'jig_pod')
    write_part(jig.mirror(b.Plane.XZ), JIG_COLOUR, OUT / 'parts' / 'jig_pod_m.step', 'jig_pod_m')
    sh, _ = L.posed(parts, unit, L.POSES['droop'])
    items = {k: (v, colour(k)) for k, v in sh.items() if not k.startswith(STEP_SKIP)}
    items['jig_pod'] = (jig, JIG_COLOUR)
    for n, r in enumerate(screws()):
        items[f'bolt_jig_{n}'] = (H.solid(r), H.MAGENTA)
    write_step(items, OUT / 'pod_jig_assembly.step', 'pod_jig')


def render(parts, unit, jig):
    for f in VAL.glob('*.png'):
        f.unlink()
    sh, _ = L.posed(parts, unit, L.POSES['droop'])
    keep = {k: v for k, v in sh.items() if not k.startswith(('bolt_tie', 'roller_core'))}
    keep['jig_pod'] = jig
    for n, r in enumerate(screws()):
        keep[f'bolt_jig_{n}'] = H.solid(r)
    L.iso(keep, VAL / 'iso_motor_side.png', 235, 22, 'pod in the bench stand (droop: unloaded), motor side', colour_fn=colour)
    L.iso(keep, VAL / 'iso_outer_side.png', 55, 22, 'pod in the bench stand (droop: unloaded), outer side', colour_fn=colour)
    L.iso({'jig_pod': jig}, VAL / 'iso_jig.png', 215, 30, 'bench stand alone: inner frame pocket (right), keel pocket (left), brace, base plate',
          colour_fn=colour, tol=0.2)
    L.iso({'jig_pod': jig, **{f'bolt_jig_{n}': H.solid(r) for n, r in enumerate(screws())}}, VAL / 'iso_jig_below.png', 200, -40,
          'from below: six counterbores for the long M3s (magenta)', colour_fn=colour, tol=0.2)


def main(stage):
    for d in (OUT, VAL):
        d.mkdir(parents=True, exist_ok=True)
    parts, unit, ko, jig, info = build_all()
    print('built', len(jig.solids()), jig.is_valid, round(jig.volume), json.dumps(info), flush=True)
    if stage in ('check', 'all'):
        rep = check(parts, unit, ko, jig, info)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(parts, unit, jig)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(parts, unit, jig)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
