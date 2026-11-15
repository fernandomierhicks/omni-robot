"""Pod_reimagined_4: proposal 3's outboard side frames + an inboard BEAM, so each 627's inner race is held from both sides, on a stand-in
FLOOR DECK that replaces proposal 3's tie. Blue-sky proposal, Nov 14 2026 (Fernando); build123d, STEP review only (shared: ../tools/pod_rx.py).

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B build.py [build|check|export|render|all]

 * Fork = proposal 3's open U (two 12 mm arm plates with the same pivot housing, the lower bar, a flat pad on each arm under its springs).
   DROOP STOP: the bar's back face (28 below the pivot) lands on a lug that hangs from the beam, on the centre plane, so the droop screws and
   their arc slots (the "cavities") are gone. (A heel on the bar under the beam was tried first: it ran into the pivot housing's ring pocket
   and ring screws.)
 * Each pivot is supported on BOTH sides: the side frame's boss on the inner race's outer face, the beam's boss on its wheel-side face (with the
   0.6 shim). One M3 x 30 pan head from the frame's outer face, through the frame, its boss and the race, threads into the beam: it clamps
   frame - race - shim - beam. No threaded printed caps (proposal 3's weak point).
 * Side frames 8 thick (proposal 3: 12): the pod is 122 mm wide (proposal 3: 130, pod_lean: 131). Each carries its spring tower straight over its
   arm (pod.py's springs and plungers, today's lever and rate) and stands on the deck: 4 x M3 up from below, two ahead of the pivot and two
   behind it.
 * rx_deck: a stand-in for the robot's first floor under this pod (today's chassis members are 22 mm deep, z -30..-8 here): a U of two rails
   under the frames and a cross member behind the fork. The beam sits on the cross member (2 x M3 up from below). Frames + beam + deck close a
   box round the fork, so the pod no longer needs its own tie.
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

NAME = 'pod_reimagined_4'
P = R.P
FRAME_T = 8.0                                                          # proposal 3: 12
OUT_FRAME = (R.OUTER_ARM[0] - R.STATIC_GAP - FRAME_T, R.OUTER_ARM[0] - R.STATIC_GAP)    # (outer face, inner face) = (-58.4, -50.4)
IN_FRAME = (R.INNER_ARM[0] + R.STATIC_GAP + FRAME_T, R.INNER_ARM[0] + R.STATIC_GAP)     # (63.7, 55.7)
ARM_C = (sum(R.OUTER_ARM) / 2, sum(R.INNER_ARM) / 2)                 # spring axes: the arms' centre planes
TOWER_HALF_Y = 15.0
X_MID = 2.85                                                           # half way between the arms' centres
# fork: proposal 3's bar (its back face is the droop stop's moving half: 28 below the pivot, it moves back ~1 mm from nominal to droop)
BAR = dict(y=(74.0, 87.0), z=(-36.0, -20.0))
BAR_CIRCLES = [(78.0, -32.0, 4.0), (83.0, -32.0, 4.0), (78.0, BAR['z'][1] - 4.0, 4.0), (83.0, BAR['z'][1] - 4.0, 4.0)]
INSET = 3.0
R.PAD_CIRCLES[:] = [P.Y_S - 14.0, P.Y_S + 14.0]
ARM_EXTRA = [(y, z, r + INSET) for y, z, r in BAR_CIRCLES[:2]] + R.pad_circles()
R.WAISTS[:] = [(38.0, -78.0, 50.0)]
R.POLY_CUTS[:] = [[(P.Y_S - 38.0, 29.8), (P.Y_S - 22.0, PAD_Z), (P.Y_S + 18.0, PAD_Z), (P.Y_S + 18.0, 90.0), (P.Y_S - 38.0, 90.0)]]
TIE_LINES = [(80.5, -28.0, 45, 50)]
# static
FRAME_CIRCLES = [(40.0, -2.0, 6.0), (108.0, -2.0, 6.0), (P.Y_S - TOWER_HALF_Y + 6.0, P.Z_TBOT + 5.0, 6.0),
                 (P.Y_S + TOWER_HALF_Y - 6.0, P.Z_TBOT + 5.0, 6.0), (PIVOT_Y, PIVOT_Z, 8.0)]
FRAME_WAIST = (112.0, 62.0, 38.0)
BEAM = dict(x=(R.static_end_x(R.OUTER_ARM), R.static_end_x(R.INNER_ARM)), y=(73.6, 118.0), z=(-8.0, 10.0))
BEAM_R_FRONT, BEAM_R_BACK = 0.8, 4.0  # plan corners: the bosses (Ø10 at y 80.6) come within 2 mm of the front face, so its corners stay small
LUG = dict(y=(BAR['y'][1], BAR['y'][1] + 9.0), z=(-34.0, -4.0))      # the droop stop: hangs from the beam behind the bar; its front face is the
                                   # bar's back face as it stands at droop (the box turned by the droop angle about the pivot)
DECK = dict(z=(-30.0, -8.0), y=(28.0, 122.0), cross_y=106.0, rail_w=14.0)
DECK_SEAT = 6.0                                                        # deck under a screw head (counterbored from below)
DECK_SCREW_L = 16.0                                                    # M3 x 16: 6 in the deck, 10 of thread above it
FRAME_SCREW_Y = (44.0, 68.0, 94.0, 106.0)                              # two ahead of the pivot (y 80.6), two behind it
BEAM_SCREWS = [(X_MID - 22.0, 114.0), (X_MID + 22.0, 114.0)]
PIVOT_SCREW_L4 = 30.0


# ---- fork --------------------------------------------------------------------------------------------------------------------------------
def build_fork(info):
    oc, ic, recs = R.tie_lines_holes(TIE_LINES)
    outer = R.outer_arm(ARM_EXTRA, info, members=(R.member(BAR_CIRCLES),))
    outer = outer + R.tenon(*BAR['y'], *BAR['z'])
    for c in oc:
        outer = outer - c
    inner = R.motor_plate(ARM_EXTRA, info) - R.tenon(*BAR['y'], *BAR['z'], grow=R.TENON_CLR)
    for c in ic:
        inner = inner - c
    return outer, inner, list(recs)


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


def frame_sx(side):
    fr, arm = (OUT_FRAME, R.OUTER_ARM) if side == 'o' else (IN_FRAME, R.INNER_ARM)
    xo, xi = fr
    return fr, arm, xo, xi, (1.0 if xi > xo else -1.0)


def build_frame(side, info):
    """one side frame (8 thick): the plate (outer face on the bed), its tower over the arm with the R4 cove, bump rails, the pivot boss on the
    inner race (no shim this side), the pivot screw's counterbore and snug hole, the deck screws' pilots"""
    fr, arm, xo, xi, s = frame_sx(side)
    xc = ARM_C[0] if side == 'o' else ARM_C[1]
    x0, x1 = R.xs(xo, xi)
    f = b.extrude(frame_face().moved(b.Location((x0, 0, 0))), amount=x1 - x0, dir=(1, 0, 0))
    f = R.round_bed(f, xo, xi, r_top=1.4, info=info, key=f'frame_{side}')
    ty0, ty1 = P.Y_S - 12.0, P.Y_S + TOWER_HALF_Y
    tower = box(*R.xs(xo, arm[1]), ty0, ty1, P.Z_TBOT, P.Z_TT)
    tower = R.fillet_try(tower, [e for e in tower.edges() if not R.on_x(e, xo)], (3.0, 2.0), info, f'tower_{side}_edges')
    try:
        tower = b.chamfer([e for e in tower.edges() if R.on_x(e, xo)], R.BED_CHAMFER)
    except Exception:                                                  # noqa: BLE001
        pass
    cr = 4.0
    cy0, cy1 = ty0 + 3.5, ty1 - 3.5
    cove = box(*R.xs(xi - s * 0.5, xi + s * cr), cy0, cy1, P.Z_TBOT - cr, P.Z_TBOT + 0.5) - R.P.cyl(cr, cy1 - cy0 + 2, (0, 0, 0)).rotate(b.Axis.X, 90).moved(
        b.Location((xi + s * cr, (cy0 + cy1) / 2, P.Z_TBOT - cr)))
    f = (f + tower + cove).clean()
    line_b = R.rail_line(P.Y_S - 14.0, P.Y_S + 14.0)
    pad_front_at_bump = float(P.rot_pt((P.Y_S - 14.0, PAD_Z), A_BUMP)[0]) + 0.5
    wy = WASHER_OD / 2 + RAIL_CLEAR
    ax = R.xs(arm[0], arm[1])
    for ry in ((max(ty0, pad_front_at_bump), P.Y_PAIR[0] - wy), (P.Y_PAIR[1] + wy, ty1)):
        if ry[1] - ry[0] < 1.0:
            continue
        f = f + R.P.yz_poly([(ry[0], line_b(ry[0])), (ry[1], line_b(ry[1])), (ry[1], P.Z_TBOT + 0.5), (ry[0], P.Z_TBOT + 0.5)], *ax)
    f = f + R.static_boss(arm, 'outer', shim=False)
    for y in P.Y_PAIR:
        f = f - cyl((SPRING_HOLE_D + 0.6) / 2, P.Z_CEIL - P.Z_TBOT + 0.1, (xc, y, (P.Z_CEIL + P.Z_TBOT) / 2 - 0.05))
        f = f - cyl(GUIDE_R, GUIDE_LEN + 0.2, (xc, y, P.Z_CEIL + GUIDE_LEN / 2))
    b_ring, b_lip = R.brg_x(arm)
    f = f - along_x(R.CB_D / 2, *R.xs(xo - s * 0.1, xo + s * R.CB_DEPTH), *PIV) - along_x(M3_SNUG / 2, *R.xs(xo - s * 0.1, b_lip + s * (R.SPIGOT_L + 0.1)), *PIV)
    xm = (xo + xi) / 2
    for y in FRAME_SCREW_Y:
        f = f - cyl(M3_SOFT_PILOT / 2, 80.0, (xm, y, 32.0))           # up through the plate (holes through; in the layer plane: Ø2.9)
    return f


# ---- the beam: the inboard support of both pivots ------------------------------------------------------------------------------------------
def build_beam(info):
    """between the arms, behind the wheel: a boss on each 627's wheel-side face (through the ring's bore, 0.6 short of the race for the shim),
    one Ø2.9 line for both pivot screws, the droop stop's lug hanging behind the fork's bar, its back on the deck's cross member. Prints top
    face down (the lug standing up)"""
    x0, x1 = BEAM['x']
    y0, y1 = BEAM['y']
    z0, z1 = BEAM['z']
    at_z = lambda e, z: all(abs(e.position_at(t).Z - z) < 1e-3 for t in (0, 0.5, 1))   # noqa: E731
    k = box(x0, x1, y0, y1, z0, z1)
    vert = [e for e in k.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 1]
    k = R.fillet_try(k, [e for e in vert if e.center().Y > (y0 + y1) / 2], (BEAM_R_BACK,), info, 'beam_back_corners')
    vert = [e for e in k.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 1 and e.center().Y < y0 + 1]
    k = R.fillet_try(k, vert, (BEAM_R_FRONT,), info, 'beam_front_corners')
    try:                                                               # the top prints on the bed: chamfered
        k = b.chamfer([e for e in k.edges() if at_z(e, z1)], 0.5)
    except Exception:                                                  # noqa: BLE001
        info['beam_bed_chamfer'] = 'refused'
    # the droop stop's lug, 3 mm in from the beam's ends (so its joins are interior loops, clear of the bosses and the ring screws)
    lx0, lx1 = x0 + 3.0, x1 - 3.0
    lug = box(lx0, lx1, *LUG['y'], *LUG['z']).rotate(PIVOT_AXIS, A_DROOP)
    k = (k + lug).clean()
    in_lug = lambda e, g=0.0: lx0 - g < e.center().X < lx1 + g and LUG['y'][0] - 1 - g < e.center().Y < LUG['y'][1] + 2 + g   # noqa: E731
    k = R.fillet_try(k, [e for e in k.edges() if at_z(e, z0) and in_lug(e, 0.1)], (2.0, 1.5, 1.0), info, 'lug_root')
    k = R.fillet_try(k, [e for e in k.edges() if e.center().Z < z0 - 3.0 and in_lug(e, 0.5)], (1.5, 1.0), info, 'lug_edges')
    k = R.fillet_try(k, [e for e in k.edges() if at_z(e, z0) and not in_lug(e, 3.0)], (1.0, 0.6), info, 'beam_bottom_edges')
    for arm in (R.OUTER_ARM, R.INNER_ARM):
        k = k + R.static_boss(arm, 'wheel')
    k = k.clean()
    k = R.fillet_try(k, [e for e in k.edges() if e.geom_type.name == 'CIRCLE' and abs(e.radius - R.BOSS_D / 2) < 1e-3
                         and any(abs(e.center().X - R.static_end_x(a)) < 1e-3 for a in (R.OUTER_ARM, R.INNER_ARM))], (1.0, 0.6), info, 'beam_boss_roots')
    k = k - along_x(M3_SOFT_PILOT / 2, x0 - 8, x1 + 8, *PIV)
    for x, y in BEAM_SCREWS:
        k = k - cyl(M3_PILOT / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2))   # along the print axis: Ø2.8
    return k


# ---- the floor deck (a stand-in for the chassis under this pod) ------------------------------------------------------------------------
def deck_rails():
    """(x0, x1) of the outer and inner rails: from 1 mm off each arm outward, under the frames"""
    xo_in, xi_in = OUT_FRAME[1], IN_FRAME[1]
    return (xo_in - DECK['rail_w'], xo_in), (xi_in, xi_in + DECK['rail_w'])


def build_deck(info):
    """a U in plan, 22 deep (z -30..-8, as today's chassis members): a rail under each frame and a cross member behind the fork; corners R6
    in plan, top edges R2, the screws counterbored from below (6 mm of deck under each head)"""
    (ox0, ox1), (ix0, ix1) = deck_rails()
    y0, y1 = DECK['y']
    z0, z1 = DECK['z']
    d = box(ox0, ix1, y0, y1, z0, z1) - box(ox1, ix0, y0 - 1, DECK['cross_y'], z0 - 1, z1 + 1)
    d = R.fillet_try(d, [e for e in d.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 1], (6.0, 4.0), info, 'deck_plan_corners')
    d = R.fillet_try(d, [e for e in d.edges() if all(abs(e.position_at(t).Z - z1) < 1e-3 for t in (0, 0.5, 1))], (2.0, 1.4), info, 'deck_top_edges')
    for x, y in deck_screw_xy():
        d = d - cyl(R.CB_D / 2, z1 - DECK_SEAT - z0 + 0.1, (x, y, (z0 + z1 - DECK_SEAT) / 2 - 0.05))
        d = d - cyl(M3_CLR / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2))
    return d


def deck_screw_xy():
    out = []
    for fr in (OUT_FRAME, IN_FRAME):
        xm = sum(fr) / 2
        out += [(xm, y) for y in FRAME_SCREW_Y]
    return out + list(BEAM_SCREWS)


def pivot_screw4(side):
    fr, arm, xo, xi, s = frame_sx(side)
    return dict(kind='pan', size=3.0, L=PIVOT_SCREW_L4, p=(xo + s * R.CB_DEPTH, *PIV), d=(s, 0.0, 0.0), name=f'bolt_pivot_{side}')


def static_recs():
    zh = DECK['z'][1] - DECK_SEAT
    out = [pivot_screw4('o'), pivot_screw4('i')]
    out += [dict(kind='pan', size=3.0, L=DECK_SCREW_L, p=(x, y, zh), d=(0, 0, 1.0), name=f'bolt_deck_{k + 1}') for k, (x, y) in enumerate(deck_screw_xy())]
    return out


def build(info=None):
    info = {} if info is None else info
    outer, inner, recs_moving = build_fork(info)
    parts = dict(pod_fork_outer=outer, pod_fork_inner=inner, rx_frame_outer=build_frame('o', info), rx_frame_inner=build_frame('i', info),
                 rx_beam=build_beam(info), rx_deck=build_deck(info), rx_ring_outer=R.build_ring(R.OUTER_ARM), rx_ring_inner=R.build_ring(R.INNER_ARM),
                 rx_shim_outer=R.build_shim(R.OUTER_ARM, 'wheel'), rx_shim_inner=R.build_shim(R.INNER_ARM, 'wheel'), pod_retainer=L.build_retainer())
    recs_moving = (recs_moving + R.ring_screw_recs(R.OUTER_ARM, 'o') + R.ring_screw_recs(R.INNER_ARM, 'i') + R.motor_screw_recs()
                   + [R.retainer_screw_rec()])
    return parts, recs_moving, static_recs(), info


PRINTED_UP = dict(pod_fork_outer=(1, 0, 0), pod_fork_inner=(-1, 0, 0), rx_frame_outer=(1, 0, 0), rx_frame_inner=(-1, 0, 0), rx_beam=(0, 0, -1),
                  rx_deck=(0, 0, 1), rx_ring_outer=(1, 0, 0), rx_ring_inner=(-1, 0, 0), rx_shim_outer=(1, 0, 0), rx_shim_inner=(-1, 0, 0),
                  pod_retainer=(1, 0, 0))
STATIC = ('rx_frame_outer', 'rx_frame_inner', 'rx_beam', 'rx_deck', 'rx_shim_outer', 'rx_shim_inner')
STATIC_PRINTED = ('rx_frame_outer', 'rx_frame_inner', 'rx_beam', 'rx_deck')


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
    fork = R.FORK + ('rx_ring_outer', 'rx_ring_inner')
    return dict(bar_to_lug_droop_stop=min_dist(sh['pod_fork_outer'], sh['rx_beam']),
                beam_to_motor_plate=min_dist(sh['pod_fork_inner'], sh['rx_beam']),
                rails_to_pads=min(min_dist(sh[f], sh[k]) for f, k in (('rx_frame_outer', 'pod_fork_outer'), ('rx_frame_inner', 'pod_fork_inner'))),
                deck_to_fork_and_rings=min(min_dist(sh['rx_deck'], sh[f]) for f in fork),
                frames_to_rings=min(min_dist(sh[f], sh[r]) for f in ('rx_frame_outer', 'rx_frame_inner') for r in ('rx_ring_outer', 'rx_ring_inner')),
                plunger_nut_gap=NUT_GAP + min(P.plunger_lift(a, y)[0] for y in P.Y_PAIR))


def droop_stop_report():
    """where the droop stop is and how it is drawn"""
    return dict(where='the fork bar\'s back face (28 below the pivot) lands on the face of a lug hanging from the beam, on the centre plane',
                drawn='the lug is a box turned by the droop angle about the pivot, so its face is the bar\'s back face at droop',
                travel_mm='nominal -> droop: the bar\'s back face moves back about 0.75 (top) to 1.3 (bottom)',
                replaces='the droop screws and their arc slots')


def check(parts, rm, rs, unit, info):
    span = abs(sum(R.brg_x(R.INNER_ARM)) / 2 - sum(R.brg_x(R.OUTER_ARM)) / 2)
    rep = R.run_checks(NAME, parts, rm, rs, unit, info, assembly, STATIC_PRINTED, PRINTED_UP, pose_gaps, R.front_springs_report(),
                       R.hand_loads(SPRING_ARM, False, 4, bearing_span=span))
    rep['droop_stop'] = droop_stop_report()
    return rep


def render(parts, rm, rs, unit):
    for f in HERE.glob('*.png'):
        f.unlink()
    sh, moving, _ = assembly(parts, rm, rs, unit, 0.0)
    hide = ('roller_core', 'roller_sleeve', 'bearing_687', 'bolt_1', 'bolt_2', 'bolt_cartridge')
    view = {k: v for k, v in sh.items() if not k.startswith(hide)}
    R.iso(view, HERE / 'iso_motor_side.png', 235, 22, 'Pod_reimagined_4: motor (robot-centre) side, on the deck stand-in, nominal')
    R.iso(view, HERE / 'iso_outer_side.png', 55, 22, 'Pod_reimagined_4: outer (perimeter) side, nominal')
    nowheel = {k: v for k, v in sh.items() if k.startswith(('pod_', 'rx_', 'plunger', 'spring', 'bearing_627', 'bolt_')) and not k.startswith(hide)}
    R.iso(nowheel, HERE / 'iso_back.png', 145, 28, 'frames, beam and deck round the fork, from behind (wheel hidden)')
    R.iso(nowheel, HERE / 'iso_front.png', 300, 20, 'from the front (wheel hidden)')
    R.iso(nowheel, HERE / 'iso_below.png', 200, -35, 'from below: the deck and its screws')
    nodeck = {k: v for k, v in nowheel.items() if not k.startswith(('rx_deck', 'bolt_deck'))}
    R.iso(nodeck, HERE / 'iso_below_nodeck.png', 200, -35, 'from below, deck hidden: the heel under the beam, the bar')
    apart = {'pod_fork_outer': parts['pod_fork_outer'], 'pod_fork_inner': parts['pod_fork_inner'].moved(b.Location((40, 0, 0))),
             'rx_frame_outer': parts['rx_frame_outer'].moved(b.Location((-40, 0, 0))), 'rx_frame_inner': parts['rx_frame_inner'].moved(b.Location((80, 0, 0))),
             'rx_beam': parts['rx_beam'].moved(b.Location((0, 45, 0))), 'rx_deck': parts['rx_deck'].moved(b.Location((0, 0, -40)))}
    R.iso(apart, HERE / 'iso_exploded.png', 215, 25, 'exploded: frames out along x, beam back, deck down', tol=0.25, size=(14, 9))
    sec = {k: v for k, v in sh.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
    R.section(sec, HERE / 'section_pivot_plane.png', 'z0', ((-70, 75), (55, 125)), 'section z 0 (the pivot plane): frame - 627 - beam - 627 - frame')
    R.section(sec, HERE / 'section_spring_outer.png', ('x', ARM_C[0]), ((-40, 125), (-45, 70)), f'section x {ARM_C[0]}: tower, springs, arm, frame, deck')
    for nm, a in (('droop', A_DROOP), ('nominal', 0.0), ('bump', A_BUMP)):
        s2, _, _ = assembly(parts, rm, rs, unit, a)
        s2 = {k: v for k, v in s2.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
        R.section(s2, HERE / f'section_mid_{nm}.png', ('x', X_MID), ((40, 130), (-45, 20)), f'{nm}: section x {X_MID} (centre plane): heel, beam, deck')


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
