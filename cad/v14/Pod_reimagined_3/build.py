"""Pod_reimagined_3 (RELAXED): the classic swing arm done cleanly: two outboard side frames tied behind the fork, springs over both arms.
Blue-sky proposal, Nov 13 2026; build123d, STEP review only (cad/v14/tools/pod_rx.py has what the three proposals share).

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B build.py [build|check|export|render|all]

 * Fork = the same two arm plates as proposals 1-2 (identical outline, 12 thick, the same pivot housing turned to face the frames) joined by
   the lower bar only (an open U). Each arm's top is a flat pad under its spring pair.
 * Static = two mirror-image SIDE FRAMES, one outboard of each arm: a single convex plate (foot, pivot, tower) with the spring tower straight
   over its arm, so the spring force goes down the plate to its own foot, and the pivot pin is in the same plate a hand's width away. A printed
   TIE behind the fork joins the two frames into one closed static U (the pivots stay coaxial without the chassis' help).
 * Springs: pod.py's four bed springs and plungers, 26 mm ahead of the pivot, two over each arm (today's lever, rate and plunger), bump rails
   under each tower (today's), droop: an M3 pan head on each arm's outer face in an arc slot in its frame (lever 26).
 * Footprint: 10 mm more outboard than pod_lean at the perimeter (x -62.4), 11 mm less toward the robot centre (x 67.7), today's height,
   11 mm further behind the pivot (the tie). The most direct load paths and the simplest prints of the three; the widest at the perimeter.
"""
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'tools'))
import pod_rx as R                                                     # noqa: E402
from pod_rx import box, along_x, cyl, near, overlap, min_dist, PIV, PIVOT_AXIS, POSES, A_DROOP, A_BUMP   # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
import hardware as H                                                   # noqa: E402
import pod_lean as L                                                   # noqa: E402

NAME = 'pod_reimagined_3'
P = R.P
FRAME_T = 12.0
OUT_FRAME = (R.OUTER_ARM[0] - R.STATIC_GAP - FRAME_T, R.OUTER_ARM[0] - R.STATIC_GAP)    # (outer face, inner face) = (-62, -50)
IN_FRAME = (R.INNER_ARM[0] + R.STATIC_GAP + FRAME_T, R.INNER_ARM[0] + R.STATIC_GAP)     # (67.7, 55.7)
ARM_C = (sum(R.OUTER_ARM) / 2, sum(R.INNER_ARM) / 2)                 # spring axes: the arms' centre planes (x -43, 48.7)
TOWER_HALF_Y = 15.0                                                    # tower y = Y_S +- 15 (4.2 mm rails in front of and behind the pair)
TOWER_WALL = 2.5                                                       # tower wall past the spring bores toward the wheel
BAR = dict(y=(74.0, 87.0), z=(-36.0, -20.0))        # under the pivot, below the ring's recess (r 18.2 round the pivot, on the arm's wheel face)
BAR_CIRCLES = [(78.0, -32.0, 4.0), (83.0, -32.0, 4.0), (78.0, BAR['z'][1] - 4.0, 4.0), (83.0, BAR['z'][1] - 4.0, 4.0)]
INSET = 3.0
R.PAD_CIRCLES[:] = [P.Y_S - 14.0, P.Y_S + 14.0]                       # each arm's flat top (the pad) from y 40.6 to 68.6
ARM_EXTRA = [(y, z, r + INSET) for y, z, r in BAR_CIRCLES[:2]] + R.pad_circles()
R.WAISTS[:] = [(38.0, -78.0, 50.0)]                                   # bottom waist (as proposal 1); the top is the pad
R.POLY_CUTS[:] = [[(P.Y_S - 38.0, 29.8), (P.Y_S - 22.0, PAD_Z), (P.Y_S + 18.0, PAD_Z), (P.Y_S + 18.0, 90.0), (P.Y_S - 38.0, 90.0)]]   # the arm top
                                   # ramps down (20 deg) onto a flat pad (the domes slide on it), corners rounded; the flat starts 4 mm
                                                                       # ahead of the tower so nothing rises into it at bump
TIE_LINES = [(80.5, -28.0, 45, 50)]                                   # one line, centred in the bar and its tenon
DROOP_YZ = (56.0, 8.0)                                                 # droop screw on each arm's outer face (lever 25.9 from the pivot)
FRAME_CIRCLES = [(40.0, -2.0, 6.0), (108.0, -2.0, 6.0), (P.Y_S - TOWER_HALF_Y + 6.0, P.Z_TBOT + 5.0, 6.0),   # the plate runs up into the
                 (P.Y_S + TOWER_HALF_Y - 6.0, P.Z_TBOT + 5.0, 6.0), (PIVOT_Y, PIVOT_Z, 8.0)]                     # tower block (its top at z 45);
                                   # the pivot circle no lower than the foot (z -8, the chassis face)
FRAME_WAIST = (112.0, 62.0, 38.0)
TIE = dict(y=(103.0, 113.0), z=(-8.0, 6.0))                            # the static tie behind the fork (>= 1.9 from the arms' lobes, r 20.5)
TIE_SCREWS = [(108.0, -1.0)]                                           # one M3 x 20 from each frame's outer face into the tie's end; the
TIE_KEY = 2.0                                                          # tie's ends sit 2 mm deep in pockets in the frames' inner faces
CHASSIS_Y = (44.0, 70.0, 96.0)                                         # 3 x M3 x 20 up into each frame's foot (pilots through the plate)
CHASSIS_PLATE_T = 6.0
PIVOT_SCREW_L3 = 25.0
CAP_HEX_AF, CAP_HEX_L = 8.0, 6.0                                       # the threaded cap's hex boss past the arm's wheel face (hold it with a spanner)


# ---- fork --------------------------------------------------------------------------------------------------------------------------------
def build_fork(info):
    oc, ic, tie_recs = R.tie_lines_holes(TIE_LINES)
    outer = R.outer_arm(ARM_EXTRA, info, members=(R.member(BAR_CIRCLES),))
    outer = outer + R.tenon(*BAR['y'], *BAR['z'])
    for c in oc:
        outer = outer - c
    inner = R.motor_plate(ARM_EXTRA, info) - R.tenon(*BAR['y'], *BAR['z'], grow=R.TENON_CLR)
    for c in ic:
        inner = inner - c
    recs = list(tie_recs)
    for arm, tag in ((R.OUTER_ARM, 'o'), (R.INNER_ARM, 'i')):
        xo, xw, s = R.frame_x(arm)
        rec, pilot = R.droop_screw(xo, s, DROOP_YZ, f'bolt_droop_{tag}')
        recs.append(rec)
        if tag == 'o':
            outer = outer - pilot
        else:
            inner = inner - pilot
    return outer, inner, recs


# ---- side frames -------------------------------------------------------------------------------------------------------------------------
def frame_face():
    f = b.Face(b.Wire(R.hull_edges(FRAME_CIRCLES)))
    y, z, r = FRAME_WAIST
    f = f - b.Face(b.Wire([b.Edge.make_circle(r, b.Plane(origin=(0, y, z), z_dir=(1, 0, 0)))]))
    f = f.faces()[0] if hasattr(f, 'faces') else f
    try:
        f = f.fillet_2d(6.0, [v for v in f.vertices() if abs(math.hypot(v.Y - y, v.Z - z) - r) < 1e-3])
    except Exception:                                                  # noqa: BLE001
        pass
    return f


def build_frame(side, info):
    """one side frame: the plate (outer face on the bed), its tower over the arm, bump rails, the pivot boss, the droop slot, the pivot screw's
    counterbore and snug hole, chassis and tie screw holes"""
    fr, arm = (OUT_FRAME, R.OUTER_ARM) if side == 'o' else (IN_FRAME, R.INNER_ARM)
    xo, xi = fr
    s = 1.0 if xi > xo else -1.0                                       # toward the arm
    xc = ARM_C[0] if side == 'o' else ARM_C[1]
    x0, x1 = R.xs(xo, xi)
    f = b.extrude(frame_face().moved(b.Location((x0, 0, 0))), amount=x1 - x0, dir=(1, 0, 0))
    f = R.round_bed(f, xo, xi, r_top=1.4, info=info, key=f'frame_{side}')     # the plate rounded alone (with the tower OCC crashed)
    ty0, ty1 = P.Y_S - 12.0, P.Y_S + TOWER_HALF_Y                       # the front starts 2.4 mm ahead of the front bore
    tx = R.xs(xo, arm[1])                                              # the tower block: over the plate and the arm to its wheel face (not past
                                                                       # it: 2 mm off the treads); every edge rounded but
    tower = box(*tx, ty0, ty1, P.Z_TBOT, P.Z_TT)                           # those on the bed face
    tower = R.fillet_try(tower, [e for e in tower.edges() if not R.on_x(e, xo)], (3.0, 2.0), info, f'tower_{side}_edges')
    try:
        tower = b.chamfer([e for e in tower.edges() if R.on_x(e, xo)], R.BED_CHAMFER)
    except Exception:                                                  # noqa: BLE001
        pass
    # the cove under the tower's overhang, where it leaves the plate's inner face (R4, a solid: the load path of the spring force)
    cr = 4.0
    cy0, cy1 = ty0 + 3.5, ty1 - 3.5
    cove = box(*R.xs(xi - s * 0.5, xi + s * cr), cy0, cy1, P.Z_TBOT - cr, P.Z_TBOT + 0.5) - R.P.cyl(cr, cy1 - cy0 + 2, (0, 0, 0)).rotate(b.Axis.X, 90).moved(
        b.Location((xi + s * cr, (cy0 + cy1) / 2, P.Z_TBOT - cr)))
    f = (f + tower + cove).clean()
    # bump rails: under the tower over the arm, their underside on the pad's top at bump; in front of / behind the spring pair
    line_b = R.rail_line(P.Y_S - 14.0, P.Y_S + 14.0)
    pad_front_at_bump = float(P.rot_pt((P.Y_S - 14.0, PAD_Z), A_BUMP)[0]) + 0.5
    wy = WASHER_OD / 2 + RAIL_CLEAR
    ax = R.xs(arm[0], arm[1])
    for ry in ((max(ty0, pad_front_at_bump), P.Y_PAIR[0] - wy), (P.Y_PAIR[1] + wy, ty1)):
        if ry[1] - ry[0] < 1.0:
            continue
        rail = R.P.yz_poly([(ry[0], line_b(ry[0])), (ry[1], line_b(ry[1])), (ry[1], P.Z_TBOT + 0.5), (ry[0], P.Z_TBOT + 0.5)], *ax)
        f = f + rail
    f = f + R.static_boss(arm, 'outer')
    for y in P.Y_PAIR:                                                 # spring bores and plunger guides (pod.py)
        f = f - cyl((SPRING_HOLE_D + 0.6) / 2, P.Z_CEIL - P.Z_TBOT + 0.1, (xc, y, (P.Z_CEIL + P.Z_TBOT) / 2 - 0.05))
        f = f - cyl(GUIDE_R, GUIDE_LEN + 0.2, (xc, y, P.Z_CEIL + GUIDE_LEN / 2))
    # pivot screw: counterbore in the outer face, Ø3.2 snug through the plate, boss and spigot (the pin's steel core)
    f = f - along_x(R.CB_D / 2, *R.xs(xo - s * 0.1, xo + s * R.CB_DEPTH), *PIV) - along_x(M3_SNUG / 2, *R.xs(xo - s * 0.1, xi + s * 12), *PIV)
    f = f - R.droop_slot(xi, -s, DROOP_YZ)
    xm = (xo + xi) / 2
    for y in CHASSIS_Y:
        f = f - cyl(M3_SOFT_PILOT / 2, 80.0, (xm, y, 32.0))
    f = f - tie_pocket(fr)
    for y, z in TIE_SCREWS:
        f = f - along_x(M3_CLR / 2, *R.xs(xo - s * 0.1, xi + s * 0.1), y, z) - along_x(R.CB_D / 2, *R.xs(xo - s * 0.1, xo + s * R.CB_DEPTH), y, z)
    return f


def tie_pocket(fr, grow=0.1):
    """the pocket for the tie's end in a frame's inner face (the interlock)"""
    xo, xi = fr
    s = 1.0 if xi > xo else -1.0
    return box(*R.xs(xi + s * 0.1, xi - s * TIE_KEY), TIE['y'][0] - grow, TIE['y'][1] + grow, TIE['z'][0] - grow, TIE['z'][1] + grow)


def build_tie(info):
    x0, x1 = IN_FRAME[1], OUT_FRAME[1]
    x0, x1 = min(x0, x1) - TIE_KEY, max(x0, x1) + TIE_KEY
    t = box(x0, x1, *TIE['y'], *TIE['z'])
    t = R.fillet_try(t, [e for e in t.edges() if abs(e.position_at(1).X - e.position_at(0).X) > 20 and e.center().Z > TIE['z'][0] + 0.1], (3.0, 2.0), info, 'tie_edges')
    for y, z in TIE_SCREWS:
        t = t - along_x(M3_SOFT_PILOT / 2, x0 - 0.1, x1 + 0.1, y, z)
    return t


def build_cap3(arm):
    """the inner race's cap on the wheel side, in the ring's bore: Ø10 flange from the race to the arm's wheel face, Ø7.2 spigot 3 into the bore
    and an 8 AF hex boss past the wheel face; Ø2.8 through: the pivot screw from the frame threads into it (hold the hex while tightening)"""
    xo, xw, s = R.frame_x(arm)
    b_ring, _ = R.brg_x(arm)
    c = along_x(R.CAP_D / 2, *R.xs(b_ring, xw), *PIV) + along_x(R.SPIGOT_D / 2, *R.xs(b_ring - s * 0.01, b_ring + s * R.SPIGOT_L), *PIV)
    hexa = b.extrude(b.Plane(origin=(xw + s * 0.01, *PIV), x_dir=(0, 0, 1), z_dir=(-s, 0, 0)) * b.RegularPolygon(CAP_HEX_AF / math.sqrt(3), 6), amount=CAP_HEX_L)
    c = c + hexa
    return c - along_x(M3_PILOT / 2, *R.xs(b_ring + s * (R.SPIGOT_L + 0.1), xw - s * (CAP_HEX_L + 0.1)), *PIV)


def pivot_screw3(side):
    fr = OUT_FRAME if side == 'o' else IN_FRAME
    xo, xi = fr
    s = 1.0 if xi > xo else -1.0
    return dict(kind='pan', size=3.0, L=PIVOT_SCREW_L3, p=(xo + s * R.CB_DEPTH, *PIV), d=(s, 0.0, 0.0), name=f'bolt_pivot_{side}')


def static_recs():
    out = [pivot_screw3('o'), pivot_screw3('i')]
    zb = -8.0 - CHASSIS_PLATE_T
    for side, fr in (('o', OUT_FRAME), ('i', IN_FRAME)):
        xm = sum(fr) / 2
        out += [dict(kind='pan', size=3.0, L=20.0, p=(xm, y, zb), d=(0, 0, 1.0), name=f'bolt_chassis_{side}{k + 1}') for k, y in enumerate(CHASSIS_Y)]
        xo, xi = fr
        s = 1.0 if xi > xo else -1.0
        out += [dict(kind='pan', size=3.0, L=20.0, p=(xo + s * R.CB_DEPTH, y, z), d=(s, 0, 0), name=f'bolt_tie_frame_{side}{k + 1}') for k, (y, z) in enumerate(TIE_SCREWS)]
    return out


def build(info=None):
    info = {} if info is None else info
    outer, inner, recs_moving = build_fork(info)
    parts = dict(pod_fork_outer=outer, pod_fork_inner=inner, rx_frame_outer=build_frame('o', info), rx_frame_inner=build_frame('i', info),
                 rx_tie=build_tie(info), rx_ring_outer=R.build_ring(R.OUTER_ARM), rx_ring_inner=R.build_ring(R.INNER_ARM),
                 rx_cap_outer=build_cap3(R.OUTER_ARM), rx_cap_inner=build_cap3(R.INNER_ARM),
                 rx_shim_outer=R.build_shim(R.OUTER_ARM, 'outer'), rx_shim_inner=R.build_shim(R.INNER_ARM, 'outer'), pod_retainer=L.build_retainer())
    recs_moving = (recs_moving + R.ring_screw_recs(R.OUTER_ARM, 'o') + R.ring_screw_recs(R.INNER_ARM, 'i') + R.motor_screw_recs()
                   + [R.retainer_screw_rec()])
    return parts, recs_moving, static_recs(), info


PRINTED_UP = dict(pod_fork_outer=(1, 0, 0), pod_fork_inner=(-1, 0, 0), rx_frame_outer=(1, 0, 0), rx_frame_inner=(-1, 0, 0), rx_tie=(0, 0, 1),
                  rx_ring_outer=(1, 0, 0), rx_ring_inner=(-1, 0, 0), rx_cap_outer=(-1, 0, 0), rx_cap_inner=(1, 0, 0),
                  rx_shim_outer=(1, 0, 0), rx_shim_inner=(-1, 0, 0), pod_retainer=(1, 0, 0))
STATIC = ('rx_frame_outer', 'rx_frame_inner', 'rx_tie', 'rx_cap_outer', 'rx_cap_inner', 'rx_shim_outer', 'rx_shim_inner')
STATIC_PRINTED = ('rx_frame_outer', 'rx_frame_inner', 'rx_tie')


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
    return dict(droop_heads_to_frames=min(min_dist(sh['bolt_droop_o'], sh['rx_frame_outer']), min_dist(sh['bolt_droop_i'], sh['rx_frame_inner'])),
                rails_to_pads=min(min_dist(sh[f], sh[k]) for f, k in (('rx_frame_outer', 'pod_fork_outer'), ('rx_frame_inner', 'pod_fork_inner'))),
                caps_to_fork=min(min_dist(sh[c], sh[f]) for c in ('rx_cap_outer', 'rx_cap_inner') for f in R.FORK + ('rx_ring_outer', 'rx_ring_inner')),
                tie_to_fork=min(min_dist(sh['rx_tie'], sh[f]) for f in R.FORK),
                plunger_nut_gap=NUT_GAP + min(P.plunger_lift(a, y)[0] for y in P.Y_PAIR),
                caps_to_wheel_envelope=min(R.wheel_clearance(sh[c], R.wheel_envelope(None), a) for c in ('rx_cap_outer', 'rx_cap_inner')))


def check(parts, rm, rs, unit, info):
    span = abs(sum(R.brg_x(R.INNER_ARM)) / 2 - sum(R.brg_x(R.OUTER_ARM)) / 2)
    return R.run_checks(NAME, parts, rm, rs, unit, info, assembly, STATIC_PRINTED, PRINTED_UP, pose_gaps, R.front_springs_report(),
                        R.hand_loads(SPRING_ARM, False, 4, bearing_span=span))


def render(parts, rm, rs, unit):
    for f in HERE.glob('*.png'):
        f.unlink()
    sh, moving, _ = assembly(parts, rm, rs, unit, 0.0)
    view = {k: v for k, v in sh.items() if not k.startswith(('roller_core', 'roller_sleeve', 'bearing_687', 'bolt_1', 'bolt_2', 'bolt_cartridge'))}
    R.iso(view, HERE / 'iso_motor_side.png', 235, 22, 'Pod_reimagined_3 (relaxed): motor (robot-centre) side, nominal')
    R.iso(view, HERE / 'iso_outer_side.png', 55, 22, 'Pod_reimagined_3: outer (perimeter) side, nominal')
    nowheel = {k: v for k, v in sh.items() if k.startswith(('pod_', 'rx_', 'plunger', 'spring', 'bearing_627', 'bolt_'))
               and not k.startswith(('bolt_1', 'bolt_2', 'bolt_cartridge'))}
    R.iso(nowheel, HERE / 'iso_frames_back.png', 145, 28, 'side frames and tie round the fork, from behind (wheel hidden)')
    R.iso(nowheel, HERE / 'iso_frames_front.png', 300, 20, 'side frames, fork and pads from the front (wheel hidden)')
    R.iso(nowheel, HERE / 'iso_below.png', 200, -35, 'from below')
    apart = {'pod_fork_outer': parts['pod_fork_outer'], 'pod_fork_inner': parts['pod_fork_inner'].moved(b.Location((40, 0, 0))),
             'rx_frame_outer': parts['rx_frame_outer'].moved(b.Location((-40, 0, 0))), 'rx_frame_inner': parts['rx_frame_inner'].moved(b.Location((80, 0, 0))),
             'rx_tie': parts['rx_tie'].moved(b.Location((0, 40, 0)))}
    R.iso(apart, HERE / 'iso_exploded.png', 215, 25, 'exploded: frames out along x, tie back', tol=0.25, size=(14, 8))
    sec = {k: v for k, v in sh.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
    R.section(sec, HERE / 'section_pivot_plane.png', 'z0', ((-70, 75), (55, 120)), 'section z 0 (the pivot plane): frames, pivots, arms, tie')
    R.section(sec, HERE / 'section_spring_outer.png', ('x', ARM_C[0]), ((-40, 125), (-45, 70)), f'section x {ARM_C[0]}: tower, springs, arm, frame, tie')
    for nm, a in (('droop', A_DROOP), ('bump', A_BUMP)):
        s2, _, _ = assembly(parts, rm, rs, unit, a)
        s2 = {k: v for k, v in s2.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
        R.section(s2, HERE / f'section_spring_{nm}.png', ('x', ARM_C[0]), ((-80, 125), (-75, 90)), f'{nm}: section x {ARM_C[0]}')


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
