"""Pod_reimagined_1 (COMPACT): a closed fork frame round one static block, springs behind the pivot. Blue-sky proposal, Nov 13 2026; build123d,
STEP review only (cad/v14/tools/pod_rx.py has what the three proposals share).

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B build.py [build|check|export|render|all]

 * Fork = two identical arm plates (the motor plate and the outer arm: one outline each, 12 thick) joined by a LOWER BAR under the pivot and a
   TOP-BACK BRIDGE behind it: a closed rectangular frame, stiff in torsion (side hits; the hub's uneven load split between the motor and the
   6804). Two printed halves (the outer arm with both cross members; the motor plate), joined by square tenons and one M3 tie line per member
   that also pre-compresses the member's layers.
 * Static = ONE block between the arms, behind the wheel: both pivots on its end faces (coaxial in one print), the four springs, the bump pads,
   the droop slots and the four chassis screws. It sits inside the fork frame: bar below, bridge above, arms either side.
 * Springs: the same four 8 x 25 bed springs on M4 plungers, 26 mm behind the pivot (today's lever, so today's wheel rate), standing in the
   block and pushing UP on the bridge, two per side, symmetric about the plane between the arms.
 * Stops: bump = the bridge landing on two pads on the block (at the springs' lever); droop = an M3 pan head on each arm's wheel face in an arc
   slot in the block's end face (lever 28).
 * Footprint: nothing outboard of the arms (x -50.4 .. 55.7 with the caps: 106 mm, pod_lean 131), top at z 38.9 (pod_lean 57 + nuts), 16 mm
   further behind the pivot (y 119, swept 125 at bump; pod_lean 103), where the paired pods have 106 mm between them. See README.md.
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

NAME = 'pod_reimagined_1'
BAR = dict(y=(74.0, 87.0), z=(-36.0, -20.0))        # lower bar under the pivot, below the ring's recess (r 18.2 round the pivot, wheel face)
BRIDGE_Z0 = 3.0 + SPRING_L0 - R.P.C_NOM             # bridge underside = the plunger domes' apex at nominal (the spring floors at z 0)
BRIDGE = dict(y=(97.0, 116.0), z=(BRIDGE_Z0, BRIDGE_Z0 + 11.0))     # the bridge over the springs
INSET = 3.0                        # the arms' outline stands this far proud of the cross members all round: their joins are interior loops,
                                   # rounded with the outline in one pass (R1.4; flush faces gave tangent joins OCC cannot round)
BAR_CIRCLES = [(78.0, -32.0, 4.0), (83.0, -32.0, 4.0), (78.0, BAR['z'][1] - 4.0, 4.0), (83.0, BAR['z'][1] - 4.0, 4.0)]
BRIDGE_CIRCLES = [(100.0, BRIDGE['z'][1] - 3.0, 3.0), (BRIDGE['y'][1] - 3.0, BRIDGE['z'][1] - 3.0, 3.0), (100.0, BRIDGE['z'][0] + 3.0, 3.0),
                  (BRIDGE['y'][1] - 3.0, BRIDGE['z'][0] + 3.0, 3.0)]
ARM_EXTRA = [(y, z, r + INSET) for y, z, r in BAR_CIRCLES[:2] + BRIDGE_CIRCLES[:2]] + [(BRIDGE['y'][1] - 3.0, 8.0, 3.0 + INSET)]   # + the ear
R.WAISTS[:] = [(46.0, 83.0, 60.0), (38.0, -78.0, 50.0)]   # the arms' waists (top, bottom): >= 35 from the axle, clear of the bar and bridge
BLOCK = dict(x=(R.static_end_x(R.OUTER_ARM), R.static_end_x(R.INNER_ARM)), y=(73.6, 117.0), z=(-8.0, 15.5))
BLOCK_R_PLAN, BLOCK_R_TOP, BLOCK_CHAMFER = 4.0, 3.0, 0.5
X_MID = 2.85                       # the plane half way between the two arms' centres (x -43 and 48.7)
SPRING_Y = PIVOT_Y + SPRING_ARM    # 106.6: 26 behind the pivot
SPRING_XS = (-29.6, -13.6, 2 * X_MID + 13.6, 2 * X_MID + 29.6)   # two per side, symmetric about X_MID; 2.1 wall to the block's end faces
PAIR_X = (-21.6, 2 * X_MID + 21.6)
PAD = dict(dx=3.0, y=(109.0, 116.0))  # bump pads on the block's top at each pair's middle
DROOP_YZ = (108.0, 6.0)            # droop screw: M3 x 8 pan head on each arm's wheel face, its head in an arc slot in the block's end face
TIE_LINES = [(80.5, -28.0, 45, 50), (106.5, BRIDGE['z'][0] + 5.5, 50, 45)]   # one line per member, centred in it and its tenon: (y, z, L out, L in)
CHASSIS_SCREWS = [(X_MID - 30.0, 95.0), (X_MID + 30.0, 95.0), (X_MID - 8.0, 113.0), (X_MID + 8.0, 113.0)]
CHASSIS_PLATE_T = 6.0              # the chassis member under the block (screws from below: M3 x 25, 19 mm of Ø2.9 thread)
# plunger: dome up under the bridge, shank down through the block's guide, nyloc 1.5 under the block
PL_NUT_Z = BRIDGE_Z0 - (BLOCK['z'][0] - NUT_GAP)                    # nut face from the apex, along the shank
PL_BOLT = math.ceil((PL_NUT_Z + NUT_T + 1.0 - BOLT_HEAD_K) / 5.0) * 5.0
R.pl_setup(PL_BOLT, PL_NUT_Z)


# ---- fork ------------------------------------------------------------------------------------------------------------------------------
def build_fork(info):
    oc, ic, recs = R.tie_lines_holes(TIE_LINES)
    outer = R.outer_arm(ARM_EXTRA, info, members=(R.member(BAR_CIRCLES), R.member(BRIDGE_CIRCLES)))
    outer = outer + R.tenon(*BAR['y'], *BAR['z']) + R.tenon(*BRIDGE['y'], *BRIDGE['z'])
    inner = R.motor_plate(ARM_EXTRA, info)
    inner = inner - R.tenon(*BAR['y'], *BAR['z'], grow=R.TENON_CLR) - R.tenon(*BRIDGE['y'], *BRIDGE['z'], grow=R.TENON_CLR)
    for c in oc:
        outer = outer - c
    for c in ic:
        inner = inner - c
    for arm, tag in ((R.OUTER_ARM, 'o'), (R.INNER_ARM, 'i')):          # droop screws on the wheel faces, heads toward the block
        xo, xw, s = R.frame_x(arm)
        rec, pilot = R.droop_screw(xw, -s, DROOP_YZ, f'bolt_droop_{tag}')
        recs.append(rec)
        if tag == 'o':
            outer = outer - pilot
        else:
            inner = inner - pilot
    return outer, inner, recs


# ---- static block ------------------------------------------------------------------------------------------------------------------------
def bump_plane_z(y):
    """the bridge's underside at bump, as z at y (a line in y-z)"""
    p0 = R.P.rot_pt((BRIDGE['y'][0], BRIDGE['z'][0]), A_BUMP)
    p1 = R.P.rot_pt((BRIDGE['y'][1], BRIDGE['z'][0]), A_BUMP)
    return p0[1] + (p1[1] - p0[1]) * (y - p0[0]) / (p1[0] - p0[0])


def build_block(info):
    x0, x1 = BLOCK['x']
    y0, y1 = BLOCK['y']
    z0, z1 = BLOCK['z']
    k = box(x0, x1, y0, y1, z0, z1)
    k = R.fillet_try(k, [e for e in k.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 1], (BLOCK_R_PLAN,), info, 'block_plan_corners')
    k = R.fillet_try(k, [e for e in k.edges() if all(abs(e.position_at(t).Z - z1) < 1e-3 for t in (0, 0.5, 1))], (BLOCK_R_TOP, 2.0), info, 'block_top_edges')
    try:
        k = b.chamfer([e for e in k.edges() if all(abs(e.position_at(t).Z - z0) < 1e-3 for t in (0, 0.5, 1))], BLOCK_CHAMFER)
    except Exception:                                                  # noqa: BLE001
        info['block_bed_chamfer'] = 'refused'
    for xp in PAIR_X:                                                  # bump pads, their tops on the bridge's underside at bump
        pad = R.P.yz_poly([(PAD['y'][0], z1 - 0.5), (PAD['y'][1], z1 - 0.5), (PAD['y'][1], bump_plane_z(PAD['y'][1])), (PAD['y'][0], bump_plane_z(PAD['y'][0]))],
                          xp - PAD['dx'], xp + PAD['dx'])
        k = k + R.fillet_try(pad, [e for e in pad.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 0.5], (1.5, 1.0))
    for arm in (R.OUTER_ARM, R.INNER_ARM):
        k = k + R.static_boss(arm)
    k = k.clean()
    k = R.fillet_try(k, [e for e in k.edges() if e.geom_type.name == 'CIRCLE' and abs(e.radius - R.BOSS_D / 2) < 1e-3
                         and any(abs(e.center().X - R.static_end_x(a)) < 1e-3 for a in (R.OUTER_ARM, R.INNER_ARM))], (1.0, 0.6), info, 'boss_roots')
    k = k - along_x(M3_SOFT_PILOT / 2, x0 - 8, x1 + 8, *PIV)       # the two pivot screws' Ø2.9 pilot: one line through the block and spigots
    for xs_ in SPRING_XS:
        k = k - cyl((SPRING_HOLE_D + 0.6) / 2, z1 + 5, (xs_, SPRING_Y, (z1 + 5) / 2))
        k = k - cyl(GUIDE_R, -z0 + 0.2, (xs_, SPRING_Y, z0 / 2))
    for x, y in CHASSIS_SCREWS:
        k = k - cyl(M3_SOFT_PILOT / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2))
    for arm in (R.OUTER_ARM, R.INNER_ARM):
        xo, xw, s = R.frame_x(arm)
        k = k - R.droop_slot(R.static_end_x(arm), -s, DROOP_YZ)   # into the static part (away from the arm)
    return k


def chassis_screw_recs():
    zb = BLOCK['z'][0] - CHASSIS_PLATE_T
    return [dict(kind='pan', size=3.0, L=25.0, p=(x, y, zb), d=(0, 0, 1.0), name=f'bolt_chassis_{i + 1}') for i, (x, y) in enumerate(CHASSIS_SCREWS)]


# ---- the assembly ------------------------------------------------------------------------------------------------------------------------
def build(info=None):
    info = {} if info is None else info
    outer, inner, recs = build_fork(info)
    parts = dict(pod_fork_outer=outer, pod_fork_inner=inner, rx_block=build_block(info),
                 rx_ring_outer=R.build_ring(R.OUTER_ARM), rx_ring_inner=R.build_ring(R.INNER_ARM),
                 rx_cap_outer=R.build_cap(R.OUTER_ARM), rx_cap_inner=R.build_cap(R.INNER_ARM),
                 rx_shim_outer=R.build_shim(R.OUTER_ARM), rx_shim_inner=R.build_shim(R.INNER_ARM), pod_retainer=L.build_retainer())
    recs_moving = recs + R.ring_screw_recs(R.OUTER_ARM, 'o') + R.ring_screw_recs(R.INNER_ARM, 'i') + R.motor_screw_recs() + [R.retainer_screw_rec()]
    recs_static = [R.pivot_screw_rec(R.OUTER_ARM, 'bolt_pivot_o'), R.pivot_screw_rec(R.INNER_ARM, 'bolt_pivot_i')] + chassis_screw_recs()
    return parts, recs_moving, recs_static, info


# print orientation (unit vector up): fork halves on their outer faces, the block on its bottom, rings with the countersinks up, caps on the flange
PRINTED_UP = dict(pod_fork_outer=(1, 0, 0), pod_fork_inner=(-1, 0, 0), rx_block=(0, 0, 1), rx_ring_outer=(1, 0, 0), rx_ring_inner=(-1, 0, 0),
                  rx_cap_outer=(1, 0, 0), rx_cap_inner=(-1, 0, 0), rx_shim_outer=(1, 0, 0), rx_shim_inner=(-1, 0, 0), pod_retainer=(1, 0, 0))
STATIC = ('rx_block', 'rx_cap_outer', 'rx_cap_inner', 'rx_shim_outer', 'rx_shim_inner')
STATIC_PRINTED = ('rx_block',)


def assembly(parts, recs_moving, recs_static, unit, a):
    """everything at fork angle a: name -> shape; the set of moving names; the plungers' lifts"""
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
    for i, x in enumerate(SPRING_XS):
        lift = R.plunger_lift(a, SPRING_Y, BRIDGE['z'][0], -1)
        lifts[f'spring_{i + 1}'] = lift
        out[f'plunger_{i + 1}'] = R.plunger_solid(x, SPRING_Y, BRIDGE['z'][0] + lift, up=True)
        out[f'spring_{i + 1}'] = R.spring_solid(x, SPRING_Y, 0.0, BRIDGE['z'][0] - BOLT_HEAD_K - WASHER_T + lift)
    return out, set(moving), lifts


def pose_gaps(sh, a):
    pads = sh['rx_block'] & box(-40, 42, PAD['y'][0] - 1, PAD['y'][1] + 1, BLOCK['z'][1] - 0.6, 30)
    rest = sh['rx_block'] & box(-50, 60, 60, 130, -20, BLOCK['z'][1] - 0.6)
    return dict(bridge_to_pads=min_dist(sh['pod_fork_outer'], pads),
                block_except_pads_to_fork=min(min_dist(rest, sh[f]) for f in R.FORK),
                droop_heads_to_block=min(min_dist(sh[f'bolt_droop_{t}'], sh['rx_block']) for t in 'oi'),
                caps_to_fork=min(min_dist(sh[c], sh[f]) for c in ('rx_cap_outer', 'rx_cap_inner') for f in R.FORK + ('rx_ring_outer', 'rx_ring_inner')),
                plunger_heads_to_bridge=max(min_dist(sh[f'plunger_{i + 1}'], sh['pod_fork_outer']) for i in range(len(SPRING_XS))),
                plunger_nut_gap=NUT_GAP - max(R.plunger_lift(a, SPRING_Y, BRIDGE['z'][0], -1) for _ in SPRING_XS))


def springs_report():
    comp = {n: round(SPRING_L0 - (BRIDGE['z'][0] - BOLT_HEAD_K - WASHER_T + R.plunger_lift(a, SPRING_Y, BRIDGE['z'][0], -1)), 2) for n, a in POSES.items()}
    return dict(count=len(SPRING_XS), lever_mm=SPRING_ARM, where='behind the pivot, under the bridge',
                wheel_rate_N_per_mm=round(len(SPRING_XS) * R.SPRING_K * (SPRING_ARM / PIVOT_Y) ** 2, 2), compression_mm=comp,
                bump_percent_of_free=round(100 * comp['bump'] / SPRING_L0, 1), long_life_limit_mm=SPRINGS[SPRING_DESIGN][1],
                plunger=f'M4 x {PL_BOLT:g} button head, dome up, nyloc {NUT_GAP} under the block')


def check(parts, rm, rs, unit, info):
    span = abs(sum(R.brg_x(R.INNER_ARM)) / 2 - sum(R.brg_x(R.OUTER_ARM)) / 2)
    return R.run_checks(NAME, parts, rm, rs, unit, info, assembly, STATIC_PRINTED, PRINTED_UP, pose_gaps, springs_report(),
                        R.hand_loads(SPRING_ARM, True, len(SPRING_XS), bearing_span=span))


# ---- outputs -------------------------------------------------------------------------------------------------------------------------------
HIDE = ('roller_core', 'roller_sleeve', 'bearing_687', 'bolt_1', 'bolt_2', 'bolt_cartridge')


def render(parts, rm, rs, unit):
    for f in HERE.glob('*.png'):
        f.unlink()
    sh, moving, _ = assembly(parts, rm, rs, unit, 0.0)
    view = {k: v for k, v in sh.items() if not k.startswith(HIDE)}
    R.iso(view, HERE / 'iso_motor_side.png', 235, 22, 'Pod_reimagined_1 (compact): motor (robot-centre) side, nominal')
    R.iso(view, HERE / 'iso_outer_side.png', 55, 22, 'Pod_reimagined_1: outer (perimeter) side, nominal')
    nowheel = {k: v for k, v in sh.items() if k.startswith(('pod_', 'rx_', 'plunger', 'spring', 'bearing_627', 'bolt_')) and not k.startswith(HIDE)}
    R.iso(nowheel, HERE / 'iso_frame_back.png', 145, 28, 'fork frame round the static block, from behind (wheel hidden)')
    R.iso(nowheel, HERE / 'iso_frame_front.png', 300, 20, 'fork frame and block from the wheel side (wheel hidden)')
    R.iso(nowheel, HERE / 'iso_below.png', 200, -35, 'from below: chassis screws, plunger nuts, the lower bar')
    st = {k: sh[k] for k in sh if k in STATIC or k.startswith(('plunger', 'spring', 'bolt_pivot', 'bolt_chassis'))}
    R.iso(st, HERE / 'iso_static_block.png', 215, 30, 'static: the block with both pivots, springs and plungers, caps, chassis screws', tol=0.15)
    apart = {'pod_fork_outer': parts['pod_fork_outer'].moved(b.Location((-60, 0, 0))), 'pod_fork_inner': parts['pod_fork_inner'].moved(b.Location((60, 0, 0))),
             'rx_block': parts['rx_block'], 'rx_ring_outer': parts['rx_ring_outer'].moved(b.Location((-30, 0, 0))),
             'rx_ring_inner': parts['rx_ring_inner'].moved(b.Location((30, 0, 0))), 'rx_cap_outer': parts['rx_cap_outer'].moved(b.Location((-95, 0, 0))),
             'rx_cap_inner': parts['rx_cap_inner'].moved(b.Location((95, 0, 0)))}
    R.iso(apart, HERE / 'iso_exploded.png', 215, 25, 'exploded along the pivot axis: caps, outer half, rings, block, motor plate', tol=0.25, size=(14, 8))
    sec = {k: v for k, v in sh.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
    R.section(sec, HERE / 'section_pivot_plane.png', 'z0', ((-60, 65), (55, 125)), 'section z 0 (the pivot plane): both pivots, the block, the arms')
    R.section(sec, HERE / 'section_spring_outer.png', ('x', SPRING_XS[0]), ((60, 125), (-45, 45)), f'section x {SPRING_XS[0]}: spring, plunger, bridge, block, bar')
    for nm, a in (('droop', A_DROOP), ('bump', A_BUMP)):
        s2, _, _ = assembly(parts, rm, rs, unit, a)
        s2 = {k: v for k, v in s2.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
        R.section(s2, HERE / f'section_spring_{nm}.png', ('x', SPRING_XS[0]), ((-80, 125), (-75, 90)), f'{nm}: section x {SPRING_XS[0]}')


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
