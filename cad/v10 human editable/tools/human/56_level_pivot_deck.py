"""Nov 2 2026 level fork pivot: the robot's deck and corner bumpers follow the pod (Fernando chose to cut the deck).

    python -B 56_level_pivot_deck.py          (omni-robot-sim interpreter; build123d)

At each wheel the pivot side of the deck's wheel slot opens where the pod now passes through the deck plane (pod frame,
unmirrored; y -> -y on the mirrored FL / RR pods): the outer fork plate's 627 lobe, the shaft and the guard's ears, the inner
fork plate (from its shock eye to its lobe). The guard's pivot-side feet move beyond the slot (y 94.5, x -31.5 / -14 / 16 / 30):
new 3.4 mm holes; the old ones fall inside the slot. The TPU corner bumper's side screws move from 23.36 to 17 degrees, where
the guard's bosses now are (clear of the outer fork); its old holes are filled with the bumper's own hole-free profile.
Writes step_parts/REC_deck_{left,right}_pivot.step and REC_bumper_corner_pivot.step, updates reference/robot_sim_instances.json
(products and the moved screws) and validation/level_pivot_robot_geometry.json. v9 and the original STEP parts are untouched.
"""
import copy
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

ROOT = Path(__file__).resolve().parents[2]
STEP = ROOT / 'step_parts'
TABLE = ROOT / 'reference' / 'robot_sim_instances.json'
PODS = {'FL': (45.0, True), 'RL': (135.0, False), 'RR': (225.0, True), 'FR': (315.0, False)}
L_WHEEL, WHEEL_R, DECK_CX = 146.0, 70.0, 146.0
# pod-frame (x0, x1, y0, y1) of the slot additions on the pivot side
SLOT = [(-49.3, -37.0, 79.0, 95.1),      # outer fork plate + outer spacer: the 627 lobe (r 13.5 about y 80.6) and 1 mm; clear of the bumper lobe
        (-37.0, 37.0, 79.0, 89.5),       # shaft and the guard's ears (bosses r 7 about the shaft)
        (37.0, 60.5, 30.0, 66.0),        # inner fork plate from its shock boss (which dips to z 2.9 at droop)
        (37.0, 56.3, 66.0, 95.6)]        # ... to its lobe; stops short of the payload standoff's flange (x 56.8)
FEET_OLD = [(-31.5, 87.5), (-14.0, 87.5), (16.0, 87.5), (40.25, 87.5)]
FEET_NEW = [(-31.5, 94.5), (-14.0, 94.5), (16.0, 94.5), (30.0, 94.5)]
HOLE = 3.4
SIDE_OLD, SIDE_NEW = 23.35835493834063, 17.0       # side bumper screw angle from the pod's -x about the deck centre
SIDE_Z = (10.0, 18.0)


def pod_to_robot(deg):
    th = math.radians(deg)
    P = np.eye(4)
    c, s = math.cos(th + math.pi), math.sin(th + math.pi)
    P[:2, :2] = [[c, -s], [s, c]]
    P[:3, 3] = [L_WHEEL * math.cos(th), L_WHEEL * math.sin(th), WHEEL_R]
    return P


def loc(M):
    t = gp_Trsf()
    t.SetValues(*[v for r in np.asarray(M)[:3] for v in r])
    return b.Location(t)


def rot_about_deck_centre(deg):
    """pod frame: rotation about the vertical through the deck centre (x 146)"""
    a = math.radians(deg)
    R = np.eye(4)
    R[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    T, Ti = np.eye(4), np.eye(4)
    T[0, 3], Ti[0, 3] = DECK_CX, -DECK_CX
    return T @ R @ Ti


def box(x0, x1, y0, y1, z0=-10.0, z1=10.0):
    return b.Box(x1 - x0, y1 - y0, z1 - z0, align=(b.Align.MIN,) * 3).moved(b.Location((x0, y0, z0)))


def info(s):
    bb = s.bounding_box()
    return dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_mm3=round(float(s.volume), 3),
                min=[round(v, 3) for v in bb.min], max=[round(v, 3) for v in bb.max])


def side(mirrored):
    return -1.0 if mirrored else 1.0


def export(shape, name, like):
    solids = shape.solids()
    assert len(solids) == 1, (name, len(solids))
    # one solid with its placement baked into the geometry: a compound, or a shape carrying a location, exports as a STEP
    # assembly (an instance with a transform), which the SolidWorks import refuses
    raw = solids[0].wrapped
    shape = b.Solid(BRepBuilderAPI_Transform(raw.Located(TopLoc_Location()), raw.Location().Transformation(), True).Shape())
    shape.label = name
    if getattr(like, 'color', None) is not None:
        shape.color = like.color
    path = STEP / (name + '.step')
    b.export_step(shape, path)
    back = b.import_step(path)
    i, j = info(shape), info(back)
    assert i['valid'] and i['solids'] == 1 and j['valid'] and j['solids'] == 1 and abs(i['volume_mm3'] - j['volume_mm3']) < .05, (name, i, j)
    return dict(original=None, new=i, reimport=j)


# The deck bridge (robot frame, left strip; symmetric about x = 0; the right strip's is turned 180 deg about z). Both pods'
# inner fork lobes meet at the strip's neck (y 126-138) and their ears and pivot walls stand to z 127 beside it, so the
# bridge is a portal: a leg on the inner piece short of the payload standoff (0, 112; flange r 10), a beam at z 130-140
# (3 mm over the guards; a 15 mm hole round the standoff column), a leg down into the V between the two pivot walls onto
# the outer piece. M3 x 14 from under the deck into 2.8 mm pilots, as the guard feet.
DECK_TOP, BEAM = 73.0, (130.0, 140.0)
INNER_LEG = dict(x=12.0, y=(88.0, 100.0))
OUTER_LEG = dict(x=9.0, y=(158.0, 170.0))
BEAM_HALF = 13.0
STANDOFF = (0.0, 112.0, 7.5)
PILOT, PILOT_DEPTH, GUSSET = 2.8, 10.0, (20.0, 8.0)     # gussets: inner leg; outer leg (short: the pivot walls' tops are near)


def bridge_part():
    yb0, yb1 = INNER_LEG['y'][0], OUTER_LEG['y'][1]
    beam = box(-BEAM_HALF, BEAM_HALF, yb0, yb1, *BEAM)
    leg_i = box(-INNER_LEG['x'], INNER_LEG['x'], *INNER_LEG['y'], DECK_TOP, BEAM[0])
    leg_o = box(-OUTER_LEG['x'], OUTER_LEG['x'], *OUTER_LEG['y'], DECK_TOP, BEAM[0])
    # gussets under the beam at both legs' inner faces: right triangles in the y-z plane, as wide as the outer leg
    def gusset(y_face, toward, size):
        tri = b.Plane.YZ * b.Polygon((y_face, BEAM[0]), (y_face + toward * size, BEAM[0]), (y_face, BEAM[0] - size), align=None)
        return b.extrude(tri, amount=OUTER_LEG['x'], both=True)
    part = beam + leg_i + leg_o + gusset(INNER_LEG['y'][1], 1, GUSSET[0]) + gusset(OUTER_LEG['y'][0], -1, GUSSET[1])
    part = part - b.Cylinder(STANDOFF[2], 100).moved(b.Location((STANDOFF[0], STANDOFF[1], 135.0)))
    screws = [(-6.0, 94.0), (6.0, 94.0), (0.0, 161.0), (0.0, 167.0)]
    for x, y in screws:
        part = part - b.Cylinder(PILOT / 2, PILOT_DEPTH, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN)).moved(b.Location((x, y, DECK_TOP)))
    assert len(part.solids()) == 1 and part.is_valid
    return part, screws


def find(rows, product, point, tol=0.05):
    hits = [r for r in rows if r['product'] == product and np.linalg.norm(np.array(r['matrix'])[:3, 3] - point[:3]) < tol]
    if len(hits) != 1:
        raise RuntimeError(f'{product} at {np.round(point[:3], 3)}: {len(hits)} matches')
    return hits[0]


def main():
    rows = json.loads(TABLE.read_text())
    if any(r['product'].startswith(('REC_deck_left_pivot', 'REC_deck_right_pivot', 'REC_bumper_corner_pivot')) for r in rows):
        raise SystemExit('instance table already has the level-pivot parts')
    report = dict(date='2026-11-02', slot_pod_frame=SLOT, feet_old=FEET_OLD, feet_new=FEET_NEW, hole=HOLE,
                  side_bumper_deg=[SIDE_OLD, SIDE_NEW], parts={}, moved=[])
    # ---- deck parts: each pod's slot addition and new foot holes, applied to whichever deck part they cross
    cutters = {}
    for name, (deg, mir) in PODS.items():
        P, s = pod_to_robot(deg), side(mir)
        c = None
        for x0, x1, y0, y1 in SLOT:
            ya, yb = sorted((s * y0, s * y1))
            c = box(x0, x1, ya, yb) if c is None else c + box(x0, x1, ya, yb)
        for x, y in FEET_NEW:
            c = c + b.Cylinder(HOLE / 2, 20).moved(b.Location((x, s * y, 0)))
        cutters[name] = c.moved(loc(P))
    # The pods' hands are opposite on the diagonals (Fernando kept them, Nov 2: hands_quick.py), so both pivots of a side point
    # into that side's deck strip and their slots meet at its neck: the side deck part becomes two pieces (outer, inner), joined
    # by a printed bridge over the neck. A few slivers between the cuts carry nothing and are dropped.
    bridge, bridge_screws = bridge_part()
    report['bridge'] = dict(info=info(bridge), screws_left=bridge_screws, note='PROPOSED: a new structural part, strength untested')
    # the deck parts sit in the robot frame (identity); the second splice is turned 180 deg and must stay clear of the cuts
    for r in rows:
        if r['product'].startswith('deck_') and not np.allclose(np.array(r['matrix']), np.eye(4)):
            placed = b.import_step(STEP / (r['product'] + '.step')).moved(loc(np.array(r['matrix'])))
            if any((placed & c) is not None and (placed & c).volume > 1e-3 for c in cutters.values()):
                raise RuntimeError(r['name'] + ' crosses a pivot slot')
    decks = {r['product']: r for r in rows if r['product'].startswith('deck_') and np.allclose(np.array(r['matrix']), np.eye(4))}
    for product, r in decks.items():
        orig = b.import_step(STEP / (product + '.step'))
        new, hit = orig, []
        for name, c in cutters.items():
            x = orig & c
            if x is not None and x.volume > 1:
                new, hit = new - c, hit + [name]
        if not hit:
            continue
        side_sign = 1.0 if orig.bounding_box().center().Y > 0 else -1.0     # deck_left (+y) / deck_right (-y)
        for x, y in bridge_screws:
            new = new - b.Cylinder(HOLE / 2, 20).moved(b.Location((side_sign * x, side_sign * y, 70.0)))
        solids = sorted(new.solids(), key=lambda s: -s.volume)
        keep, drop = solids[:2], solids[2:]
        if len(keep) != 2 or keep[1].volume < 5000 or any(s.volume > 500 for s in drop):
            raise RuntimeError(f'{product}: unexpected pieces {[round(s.volume) for s in solids]}')
        keep.sort(key=lambda s: -abs(s.bounding_box().center().Y))           # outer (toward the bumper) first
        for label, piece in zip(('outer', 'inner'), keep):
            rec = f'REC_{product}_{label}_pivot'
            rep = export(piece, rec, orig)
            rep['pods'] = hit
            report['parts'][rec] = rep
            print(rec, hit, rep['new']['volume_mm3'], 'mm3', flush=True)
        report['parts'][product + ' split'] = dict(original=info(orig), dropped_slivers_mm3=[round(s.volume, 1) for s in drop],
                                                   removed_mm3=round(orig.volume - sum(s.volume for s in keep), 1))
        r['product'] = f'REC_{product}_outer_pivot'
        rows.append(dict(copy.deepcopy(r), name=f'{product}_inner#1', product=f'REC_{product}_inner_pivot'))
    export(bridge, 'REC_deck_bridge', bridge)
    foot = next(r for r in rows if r['product'] == 'M3_x_14_PAN' and abs(r['matrix'][2][3] - 67.0) < 1e-6)
    for k, (sgn, nm) in enumerate(((1.0, 'left'), (-1.0, 'right'))):
        Rz = np.diag([sgn, sgn, 1.0, 1.0])                                    # the right strip is the left one turned 180 deg
        rows.append(dict(group='base', fastener=False, name=f'deck_bridge#{k + 1}', product='REC_deck_bridge', matrix=Rz.tolist()))
        for j, (x, y) in enumerate(bridge_screws):
            M = np.array(foot['matrix'])
            M[:3, 3] = [sgn * x, sgn * y, 67.0]
            rows.append(dict(copy.deepcopy(foot), name=f'deck bridge screw {nm} {j + 1}', matrix=M.tolist()))
            report['moved'].append(dict(screw='deck bridge M3 x 14 (new)', side=nm, at=[sgn * x, sgn * y]))
    # ---- the corner bumper (one part at all four corners): move its side screw holes from +-23.36 to +-17 degrees
    corners = [r for r in rows if r['product'] == 'bumper_corner']
    assert len(corners) == 4
    # work in the FR pod's frame with its own corner (the part is symmetric about the pod's x axis, so one frame does all four)
    P = pod_to_robot(315.0)
    orig = b.import_step(STEP / 'bumper_corner.step')
    here = (P @ np.array([-60.0, 0.0, 0.0, 1.0]))[:3]                       # the FR corner, outboard of its wheel

    def centre(r):
        c = orig.moved(loc(np.array(r['matrix']))).bounding_box().center()
        return np.linalg.norm(np.array([c.X, c.Y]) - here[:2])
    own = min(corners, key=centre)
    Mp = np.linalg.inv(P) @ np.array(own['matrix'])                       # product frame -> pod frame
    bc = orig.moved(loc(Mp))
    d = SIDE_OLD - SIDE_NEW

    def radial(a_deg, z, radius, r0, r1):
        """a cylinder on the screw axis at angle a_deg (from the pod's -x about the deck centre), between radii r0 and r1"""
        a = math.radians(a_deg)
        c = b.Cylinder(radius, r1 - r0).rotate(b.Axis.Y, 90)
        rm = 0.5 * (r0 + r1)
        return c.moved(b.Location((DECK_CX - rm * math.cos(a), rm * math.sin(a), z), (0, 0, -a_deg)))

    # A point at angle a has polar angle 180 - a about the deck centre, so taking it from a to a' is a rotation of (a - a').
    # Fill: the bumper's own hole-free profile from +-17 deg, rotated onto +-23.36 and cut to r 4.5 about each old hole (it
    # overlaps the walls round the 3.6 mm hole, so the union fuses). Drill: the original hole, 3.4 mm through the inner
    # wall (r 217.2-219.5) and 7.2 mm from r 219.5 (a 0.2 mm seat in the inner wall, then the outer wall).
    def turn(a_from, a_to):
        return loc(rot_about_deck_centre(a_from - a_to))
    # (the old holes are ~5 mm from the bumper's end: each plug is confined to the old bumper plus its hole, so it adds no
    # material beyond the end face)
    def hole(a_deg, z):
        return radial(a_deg, z, 1.7, 214.0, 219.5) + radial(a_deg, z, 3.6, 219.5, 228.0)
    filled = bc
    for sgn in (1.0, -1.0):
        plain = bc.moved(turn(sgn * SIDE_NEW, sgn * SIDE_OLD))
        for z in SIDE_Z:
            filled = filled + (plain & radial(sgn * SIDE_OLD, z, 4.5, 214.0, 228.0) & (bc + hole(sgn * SIDE_OLD, z)))
    new_pod = filled
    for sgn in (1.0, -1.0):
        for z in SIDE_Z:
            new_pod = new_pod - hole(sgn * SIDE_NEW, z)
    new = new_pod.moved(loc(np.linalg.inv(Mp)))
    rep = export(new, 'REC_bumper_corner_pivot', orig)
    rep['original'] = info(orig)
    rep['volume_change_mm3'] = round(rep['new']['volume_mm3'] - rep['original']['volume_mm3'], 3)
    report['parts']['REC_bumper_corner_pivot'] = rep
    for r in corners:
        r['product'] = 'REC_bumper_corner_pivot'
    print('REC_bumper_corner_pivot volume change', rep['volume_change_mm3'], 'mm3 (filled and drilled: ~0 expected)', flush=True)
    # ---- the base's screws: the guard's pivot-side feet and the bumper's side screws, at every corner
    for name, (deg, mir) in PODS.items():
        P, s = pod_to_robot(deg), side(mir)
        for (xo, yo), (xn, yn) in zip(FEET_OLD, FEET_NEW):
            r = find(rows, 'M3_x_14_PAN', P @ np.array([xo, s * yo, -3.0, 1.0]))
            M = np.array(r['matrix'])
            M[:3, 3] = (P @ np.array([xn, s * yn, -3.0, 1.0]))[:3]
            r['matrix'] = M.tolist()
            report['moved'].append(dict(pod=name, screw='guard foot M3 x 14', old=[xo, s * yo], new=[xn, s * yn]))
        for sgn in (1.0, -1.0):
            a = math.radians(sgn * SIDE_OLD)
            for z in SIDE_Z:
                r_head = 219.8
                p_old = np.array([DECK_CX - r_head * math.cos(a), r_head * math.sin(a), z, 1.0])
                r = find(rows, 'M3_x_10_PAN', P @ p_old, tol=0.2)
                Mpod = np.linalg.inv(P) @ np.array(r['matrix'])
                Mpod = rot_about_deck_centre(sgn * d) @ Mpod                  # a -> a' is a rotation of (a - a')
                r['matrix'] = (P @ Mpod).tolist()
                report['moved'].append(dict(pod=name, screw='bumper side M3 x 10', old_deg=sgn * SIDE_OLD, new_deg=sgn * SIDE_NEW, z=z))
    TABLE.write_text(json.dumps(rows, indent=1))
    (ROOT / 'validation' / 'level_pivot_robot_geometry.json').write_text(json.dumps(report, indent=2))
    print('instance table updated;', len(report['moved']), 'screws moved', flush=True)


if __name__ == '__main__':
    main()
