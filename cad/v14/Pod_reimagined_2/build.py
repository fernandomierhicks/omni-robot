"""Pod_reimagined_2 (MEDIUM): the static block of proposal 1 holds both pivots, two C-frames over the arms carry today's front springs.
Blue-sky proposal, Nov 13 2026; build123d, STEP review only (cad/v14/tools/pod_rx.py has what the three proposals share).

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B build.py [build|check|export|render|all]

 * Fork = the same two arm plates (identical outline, 12 thick, the same pivot housing) joined by the lower bar only: an open U. Each arm's top
   is a flat pad under its spring pair, exactly where pod_lean's pads are (26 mm ahead of the pivot).
 * Static = a BLOCK between the arms behind the wheel (both pivots coaxial on its end faces, the droop slots on a chin behind the bar, four
   chassis screws) and two C-FRAMES, one per arm: a column behind the arm's pivot lobe, bolted to the block's end face, and a neck over the arm
   to the spring tower. Nothing outboard of the arms; each C-frame is as wide as its arm (+0.9 a side).
 * Springs: pod.py's four bed springs and plungers, unchanged (today's lever, rate, plunger, bump rails), two over each arm.
 * Droop: an M3 pan head on each arm's wheel face, below and behind the pivot, in an arc slot in the block's chin (lever 23.6).
 * Load path: the springs' force goes up into each tower and back along the neck and column (a 50 mm cantilever, the longest of the three
   proposals), into the block and the chassis; the pivots see 2.1 x the wheel load (proposal 1: 4.1 x).
 * Footprint: the width of proposal 1 (x -50.4 .. 55.7), today's height (z 56.9 + nuts), y to 120 (17 behind pod_lean). See README.md.
"""
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'tools'))
import pod_rx as R                                                     # noqa: E402
from pod_rx import box, along_x, cyl, min_dist, PIV, PIVOT_AXIS, POSES, A_DROOP, A_BUMP   # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
import hardware as H                                                   # noqa: E402
import pod_lean as L                                                   # noqa: E402

NAME = 'pod_reimagined_2'
P = R.P
BAR = dict(y=(74.0, 87.0), z=(-36.0, -20.0))
BAR_CIRCLES = [(78.0, -32.0, 4.0), (83.0, -32.0, 4.0), (78.0, BAR['z'][1] - 4.0, 4.0), (83.0, BAR['z'][1] - 4.0, 4.0)]
INSET = 3.0
R.PAD_CIRCLES[:] = [P.Y_S - 14.0, P.Y_S + 14.0]                       # each arm's flat top (the pad) from y 40.6 to 68.6
DROOP_YZ = (95.0, -19.0)                                               # droop screw on each arm's wheel face (lever 23.6), clear of the ring
ARM_EXTRA = [(y, z, r + INSET) for y, z, r in BAR_CIRCLES[:2]] + R.pad_circles() + [(93.0, -20.0, 6.5)]   # + the droop screw's boss
R.WAISTS[:] = [(38.0, -78.0, 50.0)]
R.POLY_CUTS[:] = [[(P.Y_S - 38.0, 29.8), (P.Y_S - 22.0, PAD_Z), (P.Y_S + 18.0, PAD_Z), (P.Y_S + 18.0, 90.0), (P.Y_S - 38.0, 90.0)]]   # the arm top
                                   # ramps down (20 deg) onto a flat pad (the domes slide on it), corners rounded; the flat starts 4 mm
                                                                       # ahead of the tower so nothing rises into it at bump
TIE_LINES = [(80.5, -28.0, 45, 50)]
ARM_C = (sum(R.OUTER_ARM) / 2, sum(R.INNER_ARM) / 2)                 # spring axes (x -43, 48.7)
BLOCK = dict(x=(R.static_end_x(R.OUTER_ARM), R.static_end_x(R.INNER_ARM)), y=(73.6, 120.0), z=(-8.0, 12.0))
CHIN = dict(y=(89.5, 100.0), z=(-27.0, -8.0))                        # under the block, behind the bar: the droop slots in its end faces
CF_T_HALF = (SPRING_HOLE_D + 0.6) / 2 + 2.5                            # C-frame half width about the spring axis (6.8: 2.5 wall)
CF = dict(y_front=P.Y_S - 12.0, y_col=102.6, y_back=120.0, z_bot=-8.0)  # C-frame: tower from y 39.6, column from y 102.6 (1.6 off the lobe)
CF_BOLTS = [(110.0, 5.0), (114.5, -3.0)]                               # 2 x M3 x 30 from each C-frame's outer face into the block's end
X_MID = 2.85
CHASSIS_SCREWS = [(X_MID + dx, 104.0) for dx in (-27.0, -9.0, 9.0, 27.0)]   # a row behind the chin, in front of the C-frame bolts
CHASSIS_PLATE_T = 6.0


# ---- fork --------------------------------------------------------------------------------------------------------------------------------
def build_fork(info):
    oc, ic, recs = R.tie_lines_holes(TIE_LINES)
    outer = R.outer_arm(ARM_EXTRA, info, members=(R.member(BAR_CIRCLES),)) + R.tenon(*BAR['y'], *BAR['z'])
    inner = R.motor_plate(ARM_EXTRA, info) - R.tenon(*BAR['y'], *BAR['z'], grow=R.TENON_CLR)
    for c in oc:
        outer = outer - c
    for c in ic:
        inner = inner - c
    for arm, tag in ((R.OUTER_ARM, 'o'), (R.INNER_ARM, 'i')):
        xo, xw, s = R.frame_x(arm)
        rec, pilot = R.droop_screw(xw, -s, DROOP_YZ, f'bolt_droop_{tag}')
        recs.append(rec)
        if tag == 'o':
            outer = outer - pilot
        else:
            inner = inner - pilot
    return outer, inner, recs


# ---- static block (pivots, droop chin, chassis screws) ---------------------------------------------------------------------------------
def build_block(info):
    x0, x1 = BLOCK['x']
    k = box(x0, x1, *BLOCK['y'], *BLOCK['z']) + box(x0, x1, *CHIN['y'], CHIN['z'][0], BLOCK['z'][0] + 0.5)
    k = k.clean()
    zt = BLOCK['z'][1]                                                 # the top face prints on the bed: chamfered; the other long edges rounded
    on_top = lambda e: all(abs(e.position_at(t).Z - zt) < 1e-3 for t in (0, 0.5, 1))   # noqa: E731
    k = R.fillet_try(k, [e for e in k.edges() if abs(e.position_at(1).X - e.position_at(0).X) > 20 and not on_top(e)], (3.0, 2.0), info, 'block_long_edges')
    try:
        k = b.chamfer([e for e in k.edges() if on_top(e)], 0.6)
    except Exception:                                                  # noqa: BLE001
        info['block_bed_chamfer'] = 'refused'
    for arm in (R.OUTER_ARM, R.INNER_ARM):
        k = k + R.static_boss(arm)
    k = k - along_x(M3_SOFT_PILOT / 2, x0 - 8, x1 + 8, *PIV)
    for x, y in CHASSIS_SCREWS:
        k = k - cyl(M3_SOFT_PILOT / 2, 40, (x, y, BLOCK['z'][0] + 10))
    for y, z in CF_BOLTS:
        k = k - along_x(M3_SOFT_PILOT / 2, x0 - 0.1, x1 + 0.1, y, z)
    for arm in (R.OUTER_ARM, R.INNER_ARM):
        xo, xw, s = R.frame_x(arm)
        k = k - R.droop_slot(R.static_end_x(arm), -s, DROOP_YZ)   # into the static part (away from the arm)
    return k


# ---- C-frames (the spring towers) ------------------------------------------------------------------------------------------------------
def cframe_face():
    """the C-frame's side profile: tower + neck from the front of the tower to the column, column down to the chassis; corners rounded"""
    yf, yc, yb, zb = CF['y_front'], CF['y_col'], CF['y_back'], CF['z_bot']
    top = b.Face(b.Wire.make_polygon([b.Vector(0, yf, P.Z_TBOT), b.Vector(0, yb, P.Z_TBOT), b.Vector(0, yb, P.Z_TT), b.Vector(0, yf, P.Z_TT)], close=True))
    col = b.Face(b.Wire.make_polygon([b.Vector(0, yc, zb), b.Vector(0, yb, zb), b.Vector(0, yb, P.Z_TBOT + 1), b.Vector(0, yc, P.Z_TBOT + 1)], close=True))
    f = (top + col).clean()
    f = f.faces()[0] if hasattr(f, 'faces') else f
    inner = [v for v in f.vertices() if abs(v.Y - yc) < 1e-3 and abs(v.Z - P.Z_TBOT) < 1e-3]
    outer = [v for v in f.vertices() if v not in inner and not (abs(v.Z - zb) < 1e-3)]
    try:
        f = f.fillet_2d(8.0, inner)
    except Exception:                                                  # noqa: BLE001
        pass
    try:
        f = f.fillet_2d(4.0, [v for v in f.vertices() if any((v - o).length < 1e-3 for o in outer)])
    except Exception:                                                  # noqa: BLE001
        pass
    return f


def build_cframe(side, info):
    arm = R.OUTER_ARM if side == 'o' else R.INNER_ARM
    xc = ARM_C[0] if side == 'o' else ARM_C[1]
    xe = R.static_end_x(arm)                                           # the block's end face: the C-frame's inner face sits on it
    xo_, xw_, s = R.frame_x(arm)
    xout = xc + s * CF_T_HALF                                          # its outer face (0.8 past the arm's outer face)
    x0, x1 = R.xs(xe, xout)
    f = b.extrude(cframe_face().moved(b.Location((x0, 0, 0))), amount=x1 - x0, dir=(1, 0, 0))
    f = R.round_bed(f, xout, xe, r_top=1.4, info=info, key=f'cframe_{side}')
    f = f - box(*R.xs(xe - s * 0.1, xw_), CF['y_front'] - 1, CF['y_col'] - 0.5, CF['z_bot'] - 1, P.Z_TT + 1)   # the head and neck stop at the arm's
                                                                       # wheel face (2 mm off the treads); the column sits on the block's end
    # bump rails under the tower over the arm (pod.py style: their underside on the pad's top at bump)
    line_b = R.rail_line(P.Y_S - 14.0, P.Y_S + 14.0)
    pad_front_at_bump = float(P.rot_pt((P.Y_S - 14.0, PAD_Z), A_BUMP)[0]) + 0.5
    wy = WASHER_OD / 2 + RAIL_CLEAR
    ax = R.xs(*arm)
    for ry in ((max(CF['y_front'], pad_front_at_bump), P.Y_PAIR[0] - wy), (P.Y_PAIR[1] + wy, CF['y_front'] + 30.0)):
        if ry[1] - ry[0] < 1.0:
            continue
        f = f + R.P.yz_poly([(ry[0], line_b(ry[0])), (ry[1], line_b(ry[1])), (ry[1], P.Z_TBOT + 0.5), (ry[0], P.Z_TBOT + 0.5)], *ax)
    for y in P.Y_PAIR:
        f = f - cyl((SPRING_HOLE_D + 0.6) / 2, P.Z_CEIL - P.Z_TBOT + 0.1, (xc, y, (P.Z_CEIL + P.Z_TBOT) / 2 - 0.05))
        f = f - cyl(GUIDE_R, GUIDE_LEN + 0.2, (xc, y, P.Z_CEIL + GUIDE_LEN / 2))
    for y, z in CF_BOLTS:
        f = f - along_x(M3_CLR / 2, *R.xs(xout + s * 0.1, xe - s * 0.1), y, z) - along_x(R.CB_D / 2, *R.xs(xout + s * 0.1, xout - s * R.CB_DEPTH), y, z)
    return f


def static_recs():
    out = [R.pivot_screw_rec(R.OUTER_ARM, 'bolt_pivot_o'), R.pivot_screw_rec(R.INNER_ARM, 'bolt_pivot_i')]
    zb = BLOCK['z'][0] - CHASSIS_PLATE_T
    out += [dict(kind='pan', size=3.0, L=25.0, p=(x, y, zb), d=(0, 0, 1.0), name=f'bolt_chassis_{i + 1}') for i, (x, y) in enumerate(CHASSIS_SCREWS)]
    for side, arm, xc in (('o', R.OUTER_ARM, ARM_C[0]), ('i', R.INNER_ARM, ARM_C[1])):
        xo_, xw_, s = R.frame_x(arm)
        xout = xc + s * CF_T_HALF
        out += [dict(kind='pan', size=3.0, L=30.0, p=(xout - s * R.CB_DEPTH, y, z), d=(-s, 0, 0), name=f'bolt_cframe_{side}{k + 1}') for k, (y, z) in enumerate(CF_BOLTS)]
    return out


def build(info=None):
    info = {} if info is None else info
    outer, inner, recs = build_fork(info)
    parts = dict(pod_fork_outer=outer, pod_fork_inner=inner, rx_block=build_block(info), rx_tower_outer=build_cframe('o', info),
                 rx_tower_inner=build_cframe('i', info), rx_ring_outer=R.build_ring(R.OUTER_ARM), rx_ring_inner=R.build_ring(R.INNER_ARM),
                 rx_cap_outer=R.build_cap(R.OUTER_ARM), rx_cap_inner=R.build_cap(R.INNER_ARM),
                 rx_shim_outer=R.build_shim(R.OUTER_ARM), rx_shim_inner=R.build_shim(R.INNER_ARM), pod_retainer=L.build_retainer())
    recs_moving = recs + R.ring_screw_recs(R.OUTER_ARM, 'o') + R.ring_screw_recs(R.INNER_ARM, 'i') + R.motor_screw_recs() + [R.retainer_screw_rec()]
    return parts, recs_moving, static_recs(), info


# the block prints top face down (its chin up), the C-frames on their outer faces
PRINTED_UP = dict(pod_fork_outer=(1, 0, 0), pod_fork_inner=(-1, 0, 0), rx_block=(0, 0, -1), rx_tower_outer=(1, 0, 0), rx_tower_inner=(-1, 0, 0),
                  rx_ring_outer=(1, 0, 0), rx_ring_inner=(-1, 0, 0), rx_cap_outer=(1, 0, 0), rx_cap_inner=(-1, 0, 0),
                  rx_shim_outer=(1, 0, 0), rx_shim_inner=(-1, 0, 0), pod_retainer=(1, 0, 0))
STATIC = ('rx_block', 'rx_tower_outer', 'rx_tower_inner', 'rx_cap_outer', 'rx_cap_inner', 'rx_shim_outer', 'rx_shim_inner')
STATIC_PRINTED = ('rx_block', 'rx_tower_outer', 'rx_tower_inner')


def assembly(parts, recs_moving, recs_static, unit, a):
    moving = dict(unit)
    moving.update({k: parts[k] for k in ('pod_fork_outer', 'pod_fork_inner', 'rx_ring_outer', 'rx_ring_inner', 'pod_retainer')})
    moving['bearing_627_outer'] = R.bearing_627(R.OUTER_ARM)
    moving['bearing_627_inner'] = R.bearing_627(R.INNER_ARM)
    moving['tie_lead'] = L.lead_tie()
    for r in recs_moving:
        moving[r['name']] = H.solid(r)
    out = {k: (v.rotate(PIVOT_AXIS, a) if a else v) for k, v in moving.items()}
    for k in STATIC:
        out[k] = parts[k]
    for r in recs_static:
        out[r['name']] = H.solid(r)
    lifts = {}
    for side, xc in (('o', ARM_C[0]), ('i', ARM_C[1])):
        for j, y in enumerate(P.Y_PAIR):
            lift = P.plunger_lift(a, y)[0]
            lifts[f'{side}{j + 1}'] = lift
            out[f'plunger_{side}{j + 1}'] = P.plunger(xc, y).moved(b.Location((0, 0, lift)))
            out[f'spring_{side}{j + 1}'] = P.die_spring(xc, y, P.Z_CEIL - P.Z_SB - lift).moved(b.Location((0, 0, lift)))
    return out, set(moving), lifts


def pose_gaps(sh, a):
    return dict(droop_heads_to_block=min(min_dist(sh[f'bolt_droop_{t}'], sh['rx_block']) for t in 'oi'),
                rails_to_pads=min(min_dist(sh[t], sh[f]) for t, f in (('rx_tower_outer', 'pod_fork_outer'), ('rx_tower_inner', 'pod_fork_inner'))),
                caps_to_fork=min(min_dist(sh[c], sh[f]) for c in ('rx_cap_outer', 'rx_cap_inner') for f in R.FORK + ('rx_ring_outer', 'rx_ring_inner')),
                plunger_nut_gap=NUT_GAP + min(P.plunger_lift(a, y)[0] for y in P.Y_PAIR))


def check(parts, rm, rs, unit, info):
    span = abs(sum(R.brg_x(R.INNER_ARM)) / 2 - sum(R.brg_x(R.OUTER_ARM)) / 2)
    return R.run_checks(NAME, parts, rm, rs, unit, info, assembly, STATIC_PRINTED, PRINTED_UP, pose_gaps, R.front_springs_report(),
                        R.hand_loads(SPRING_ARM, False, 4, bearing_span=span))


HIDE = ('roller_core', 'roller_sleeve', 'bearing_687', 'bolt_1', 'bolt_2', 'bolt_cartridge')


def render(parts, rm, rs, unit):
    for f in HERE.glob('*.png'):
        f.unlink()
    sh, moving, _ = assembly(parts, rm, rs, unit, 0.0)
    view = {k: v for k, v in sh.items() if not k.startswith(HIDE)}
    R.iso(view, HERE / 'iso_motor_side.png', 235, 22, 'Pod_reimagined_2 (medium): motor (robot-centre) side, nominal')
    R.iso(view, HERE / 'iso_outer_side.png', 55, 22, 'Pod_reimagined_2: outer (perimeter) side, nominal')
    nowheel = {k: v for k, v in sh.items() if k.startswith(('pod_', 'rx_', 'plunger', 'spring', 'bearing_627', 'bolt_')) and not k.startswith(HIDE)}
    R.iso(nowheel, HERE / 'iso_back.png', 145, 28, 'block, C-frames and fork from behind (wheel hidden)')
    R.iso(nowheel, HERE / 'iso_front.png', 300, 20, 'C-frames over the arms, from the wheel side (wheel hidden)')
    R.iso(nowheel, HERE / 'iso_below.png', 200, -35, 'from below: the chin, chassis screws, the bar')
    apart = {'pod_fork_outer': parts['pod_fork_outer'], 'pod_fork_inner': parts['pod_fork_inner'].moved(b.Location((40, 0, 0))),
             'rx_block': parts['rx_block'].moved(b.Location((0, 40, 0))), 'rx_tower_outer': parts['rx_tower_outer'].moved(b.Location((-40, 40, 0))),
             'rx_tower_inner': parts['rx_tower_inner'].moved(b.Location((80, 40, 0)))}
    R.iso(apart, HERE / 'iso_exploded.png', 215, 25, 'exploded: block and C-frames back, motor plate out', tol=0.25, size=(14, 8))
    sec = {k: v for k, v in sh.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
    R.section(sec, HERE / 'section_pivot_plane.png', 'z0', ((-60, 65), (55, 125)), 'section z 0 (the pivot plane)')
    R.section(sec, HERE / 'section_cframe_outer.png', ('x', ARM_C[0]), ((-40, 125), (-45, 70)), f'section x {ARM_C[0]}: C-frame, springs, arm')
    for nm, a in (('droop', A_DROOP), ('bump', A_BUMP)):
        s2, _, _ = assembly(parts, rm, rs, unit, a)
        s2 = {k: v for k, v in s2.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
        R.section(s2, HERE / f'section_cframe_{nm}.png', ('x', ARM_C[0]), ((-80, 125), (-75, 90)), f'{nm}: section x {ARM_C[0]}')


def main(stage):
    parts, rm, rs, info = build()
    for k, v in parts.items():
        print(k, len(v.solids()), v.is_valid, round(v.volume), flush=True)
    print(info, flush=True)
    if stage == 'build':
        return
    unit = R.wheel_unit()
    R.wheel_envelope(unit)
    if stage in ('check', 'all'):
        R.jdump(HERE / 'checks.json', check(parts, rm, rs, unit, info))
        print('checks written', flush=True)
    if stage in ('export', 'all'):
        sh, _, _ = assembly(parts, rm, rs, unit, 0.0)
        names = {r['name'] for r in rm + rs}
        print('exported', R.export(HERE, parts, {k: v for k, v in sh.items() if k not in names}, rm + rs, NAME), flush=True)
    if stage in ('render', 'all'):
        render(parts, rm, rs, unit)
        print('rendered', flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
