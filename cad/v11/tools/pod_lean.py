"""cad/v11 lean pod (round 8 PROPOSAL, Nov 3 2026): the round-7 pod (pod.py) made narrow enough for the compact robot base (base.py).
build123d, STEP review only. Round 7's pod.py, its STEPs and the SolidWorks files are not changed by this file.

    python -B pod_lean.py [build|check|export|render|all]        (omni-robot-sim interpreter)

Why: on the X4 base the opposite-hand pods face each other in pairs, pivot end to pivot end, at the left and right side midlines, and
round 7's outer frame + outer spring tower stand 34 mm outboard of the wheel at the pod's back corner. That corner set the robot at
r 243 (base.py layout study). Here nothing static stands outboard of the wheel:
 * all four springs sit on the inner arm (2 x 2, the same distances from the pivot, so the same wheel rate and travel);
 * the plate is lowered: its top 8 mm BELOW the axle (62 mm above the floor; round 7: 3 mm above, 73). The pivot (level with the axle) and
   both static parts then stand on the plate: no keel notches, the inner frame bears on the plate along its whole foot;
 * the inner frame (foot, 2 x 2 spring tower, pivot block, droop hook, bump rails) carries the inner pivot on a Ø7 x 20 steel dowel: with the
   springs on one side the inner pivot carries about 2.7x the wheel load (round 7's printed pin with an M3 core would not);
 * the outer pivot is held from INSIDE the outer arm by a small outer keel behind the wheel, between the rollers and the arm; it stands on
   the plate behind the fork's base bar. An M3 x 14 pan from outside runs through a printed flanged sleeve in the arm's 627 into the keel;
 * the fork joins its arms with a base bar under the pivot instead of the Ø30 beam, so the keel can reach the pivot; the plate has an
   opening for the bar behind the wheel (the pod still drops in from above);
 * the outer arm is 11 mm (was 17).
As in round 7 the two static parts are tied by the robot plate. Everything is nominal CAD; nothing has been printed or tested.

Round 9 (Nov 3, after Fernando's review of the SolidWorks robot):
 * the fork is two printed halves that meet on the motor-face plane, each printed flat on a big face without support:
   the OUTER half (outer arm, the axle stub as part of it, the whole base bar) on its outer face, the INNER half (motor plate, spring pads,
   inner 627 seat) on its motor face. A gabled tenon on the bar's end locates it in a socket in the inner half (45 deg roof, no bridge);
 * M3 x 50 tie bolts run along the bar on two lines, one from each end per line (tips at x -2 / +5.5): the inner ones clamp the halves and
   squeeze the bar (printed upright, layers across it) from the joint to x 15.5, the outer ones squeeze it from the arm to x -12;
 * the bar is deeper (z -29.5 .. -8, 14 wide); past the keel it ramps up at 45 deg to the pivot height (z 0) and stays there to the joint;
 * the outer keel prints on its flat outer face: an M4 washer replaces its printed boss on the 627's inner race. It gets a gusset on its
   foot (an L in plan, against side load) and a boss round the pivot screw (M3 x 16, 7 mm engagement); edges rounded;
 * rounded edges and root fillets on the fork halves.

Round 10 (Nov 3, Fernando's review of round 9):
 * outer arm 3 mm closer to the wheel (2 mm from the hub's left face); the keel moves with it (it stands outside the wheel's radius);
 * tie bolts thread into a Ø2.9 pilot their whole length (the inner ones pass the inner half in Ø3.4 to clamp it), and the two lines are
   staggered so some bolt is threaded in at every x along the bar except under the outer heads;
 * (a 6.7 mm motor plate was tried and put back to 13: the inner half carries all four springs);
 * keel foot screws moved off the keel's side plate, heads on the foot top in counterbores down through the gusset;
 * inner frame edges rounded (R1, 36 of 48).

Round 11 (Nov 4, Fernando's review of the SolidWorks robot):
 * the outer keel is bulkier: its side plate 9.2 thick (was 6.2) and 16 tall above the pivot behind it (was 7); the gusset becomes a full
   block from the side plate to the foot's end, as tall as the side plate. Its two M3s now run from the block's top (flush counterbores)
   down through the whole keel into the chassis (M3 x 30), so they clamp all of it; the pivot screw is M3 x 20 into a longer pin boss;
 * one M3 x 70 through-bolt down the inner frame from the spring column's top (flush counterbore) through the column and the foot into the
   chassis ring, so the whole frame is clamped to the plate.

Round 12 (Nov 4, review: a pod must come out without lifting the second floor): every pod-to-chassis screw is driven from UNDER the
chassis, and the pod slides out radially (its -x) once they are out (chassis.py clips its members and pocket walls off that path):
 * inner frame: three M3 x 25 up through the member into Ø2.8 pilots in the foot (y 23, 34.5 and 88 under the pivot block); the three
   horizontal side screws are gone (their heads faced the robot's centre, under the floor);
 * keel: its two M3 x 30 come up from below into blind pilots (its top is plain);
 * the column through-bolt stays (Fernando, round 11) but is the pod's own: M3 x 60 from the column top into a pilot in the foot, tightened
   on the bench before the pod goes in.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part, write_step                         # noqa: E402

box, prism, hull2c, rot_pt, yz_poly = P.box, P.prism, P.hull2c, P.rot_pt, P.yz_poly
along_x, cyl, line, arc, V = W.along_x, W.cyl, W.line, W.arc, W.V
overlap, near, min_dist = W.overlap, W.near, W.min_dist
PIVOT_AXIS, POSES, A_DROOP, A_BUMP = P.PIVOT_AXIS, P.POSES, P.A_DROOP, P.A_BUMP
OUT = W.ROOT / 'step_export' / 'pod_lean'
VAL = W.ROOT / 'validation' / 'pod_lean'
PIV = (PIVOT_Y, PIVOT_Z)

# ---- lean-pod dimensions (pod frame as dims.py: x = wheel axis, + toward the motor = toward the robot centre; y toward the pivot) ------
OA_X = (-52.5, -41.5)              # outer arm, 11 mm; round 10: 2 mm from the cartridge / hub left face (round 8-9: -55.5..-44.5, 5 mm)
OA_LOBE_R = 13.5                   # arm lobes round the 627s (Ø22): 2.5 mm wall
PIVOT_WASHER_T = 0.8               # round 9: M4 washer (ISO 7089, Ø9) on the outer 627's inner race (round 8: a printed boss on the keel)
KEEL_X = (OA_X[1] - BRG_RECESS + PIVOT_WASHER_T + 0.1, OA_X[1] - BRG_RECESS + PIVOT_WASHER_T + 9.3)   # outer keel, 9.2 thick (round 11; was
                                   # 6.2), 0.1 past the washer; it stands outside the wheel's radius (>= 73.6 from the axle), so it may overlap
                                   # the rollers' x reach
KEEL_TOP_Z = 16.0                  # round 11: keel top behind the pivot (side plate and block; round 10: 7, the boss's top)
KEEL_TOP_Y = 86.0                  # the side plate rises from the boss's top (y 80.6, z 7) to KEEL_TOP_Z here (>= 9.6 from the tread at bump)
PLATE_TOP_L, PLATE_T_L = -8.0, 6.0   # robot plate top (pod frame: 8 mm below the axle, 62 above the floor) and thickness
PLATE_BOTTOM_L = PLATE_TOP_L - PLATE_T_L
KEEL_BOSS_R = 7.0                  # keel round the pivot
KEEL_FOOT = dict(x=(KEEL_X[0], -12.0), y=(88.5, 103.0), t=7.0)  # outer keel's foot on the plate, behind the fork's base bar. Round 12: to
                                   # y 103 (was 97): the inner fork's pivot lobe dips 3 mm under the plate top up to y 91.5, so going out it
                                   # sweeps the chassis under the keel's front; the keel's screws and its support sit behind that path
KEEL_GUSSET_Y = (89.0, 103.0)      # round 11: a full block on the foot (round 9-10: a wedge from the keel's top down to the foot at x -15)
KEEL_PIN_BOSS_R, KEEL_PIN_BOSS_X = 5.0, KEEL_X[1] + 3.5   # boss round the pivot pilot on the keel's wheel side (>= 10 mm from the rollers radially)
KEEL_EDGE_R = 0.8                  # all keel edges (OCC refuses larger at its corners); R3 in the L's inside corner
KEEL_COVE_R = 3.0
FRAME_EDGE_R = 1.0                 # round 10: inner frame edges
KEEL_SCREWS = ((-27.0, 98.0), (-17.0, 98.0))     # vertical M3 from under the chassis into the keel (round 11: y 92.75)
CBORE_D, CBORE_DEPTH = 7.2, 2.9    # flush counterbore for an M3 hex-drive pan head (6.5 x 2.5)
KEEL_SCREW_L = 30.0                # round 12: M3 x 30 up from under the chassis (chassis.BELOW_CB), 18.1 through the member, 11.9 into the keel
KEEL_PILOT_TOP = 5.0               # round 12: the keel's pilots end here (blind; its top is plain). Round 11 drove them from its top
FRAME_BOLTS = ((72.6, 54.6),)      # round 11: through-bolt down the inner frame, between the two support pilots (y 44 / 65)
FRAME_BOLT_L = 60.0                # round 12: M3 x 60 from the column top (flush) into a pilot in the foot (z -7..2): it clamps the column to the
FRAME_BOLT_PILOT = (-7.0, 2.0)     # foot inside the pod (round 11: M3 x 70 on into the ring, which needed the column top reachable in the robot)
FOOT_SCREW_L = 25.0                # round 12: M3 x 25 up from under the chassis into Ø2.8 pilots in the inner frame's foot (6.9 engagement)
FOOT_SCREW_Y = (23.0, 34.5, 88.0)  # MOUNT_Y_L and one under the pivot block
SLEEVE_D, SLEEVE_FLANGE_D, SLEEVE_FLANGE_T = 7.0, 10.0, 1.0   # printed sleeve in the outer 627's inner race (Ø3.4 bore for the M3)
HEAD_HOLE_D = 14.0                 # through the outer arm round the sleeve flange and the screw head (static parts in the turning arm)
OUTER_SCREW_L = 20.0               # round 11 (round 9-10: 16): 11.1 mm into the keel and its pin boss
INNER_W_X = (INNER_ARM_X[0], 65.6)  # inner arm widened behind the motor: spring pads (2 x 2) and the inner 627 seat
MOTOR_PLATE_X = INNER_ARM_X       # the plate under the motor's rear face, 13 thick: it carries all 4 springs (round 10 tried 6.7, Fernando: back to 13)
MOTOR_SCREW_L = REAR_SCREW_L      # 4 x M3 x 12 pan heads, 3.8 into the motor's Ø2.8 x 4.3 holes
SPRING_X = (50.5, 61.1)            # the two spring columns (x); the tower's wall is 7.3 mm from the rollers
PAD_HALF_L, TOWER_HALF_L = 15.0, 15.0   # pad / tower half length (y): 4.2 mm bump rails in front of and behind the spring pairs
FI_X = (INNER_W_X[1] + FRAME_GAP, INNER_W_X[1] + FRAME_GAP + FRAME_T)   # inner frame
FOOT_Y_L = (18.0, 92.0)            # foot on the plate along the whole frame (round 7 ran on to y 104, over a keel notch)
PIVOT_BLOCK_R = 9.0                # inner frame's block round the dowel, down to the foot
GUSSET_END_L = 84.0                # gusset from the column back down to the foot top (round 7: 90)
MOUNT_Y_L = (23.0, 34.5)           # vertical M3 x 14 through the inner frame's foot into the plate, in front of the spring column (clear of its fillet)
SIDE_SCREW_Y, SIDE_SCREW_Z = (47.0, 60.0, 72.0), PLATE_TOP_L + 7.0   # horizontal M3 pilots in the inner frame's outer face (chassis wall screws);
SIDE_PILOT_DEPTH = 9.5             # y 72 is in the pivot block, 3.7 mm clear of the dowel bore
SUPPORT_PILOTS, SUPPORT_PILOT_DEPTH = ((72.6, 44.0), (72.6, 65.0)), 12.0   # spring-column top: the second-floor support's two M3
BASE_Y, BASE_Z = (73.0, 87.0), (-29.5, -8.0)     # fork base bar under the pivot, behind the wheel (through an opening in the plate; round 8: y 74.5, z -26)
BASE_TOP = 0.0                     # round 9: from BASE_RAISE_X to the inner half the bar rises to the pivot height
BASE_KEEL_CUT = 10.0               # under the outer keel the bar's top is cut down to z -BASE_KEEL_CUT (and r 9 round the pivot)
BASE_RAISE_X = KEEL_PIN_BOSS_X + 1.5
# round 9: fork halves and tie bolts
SPLIT_X = INNER_ARM_X[0]           # the halves meet on the motor-face plane (44.7)
TIE_LINES = ((78.5, -16.5), (82.0, -24.5))   # (y, z) of the two tie-bolt lines; each head counterbore stays >= 2 mm off the 627 seats
TIE_L, TIE_CBORE_D, TIE_HEAD = 50.0, 6.8, (6.5, 2.5)   # M3 x 50 pan heads
TIE_SEATS = ((OA_X[0] + 3.5, 55.5), (OA_X[0] + 10.5, 62.0))   # round 10, per line (outer, inner) head seats, staggered so that along the bar
                                   # some bolt is threaded in at every x: line 1 from x -49 to 1 and 5.5 to 55.5, line 2 from -42 to 8 and 12 to 62.
                                   # The inner heads sit >= 3 mm above the socket's ridge
TIE_PILOT_D = 2.9                  # round 10: the bolts thread into a Ø2.9 pilot along their whole length (soft thread through every layer); the
                                   # inner ones pass the inner half in Ø3.4 so they clamp it to the bar
TENON_L, TENON_INSET, TENON_CLR = 3.0, 2.5, 0.15   # tenon on the bar's end into a socket in the inner half
FORK_EDGE_R, FORK_ROOT_R = 2.0, 3.0
DOWEL_D, DOWEL_L = 7.0, 20.0       # inner pivot: Ø7 x 20 steel dowel (ISO 8734) pressed through the inner keel (fit VERIFY)
Y_S = P.Y_S
Y_PAIR = P.Y_PAIR
PLUNGERS = {f'inner_{"ab"[j]}{i + 1}': (xs, y) for i, xs in enumerate(SPRING_X) for j, y in enumerate(Y_PAIR)}   # a nearer the axle

EXTRA = {'pod_keel': (0.25, 0.68, 0.74), 'pod_sleeve': (0.66, 0.46, 0.86), 'dowel': (0.75, 0.75, 0.78)}


def colour(name):
    for k, v in EXTRA.items():
        if name.startswith(k):
            return v
    return P.colour(name)


# ---- fork (round 9: two halves) -----------------------------------------------------------------------------------------------------
def on_x(e, xs):
    """edge lying in one of the planes x = xs"""
    return any(all(abs(e.position_at(t).X - x) < 1e-3 for t in (0, 0.5, 1)) for x in xs)


def round_edges(s, edges, radii):
    """fillet `edges`, trying the radii in turn; returns (shape, radius used or None)"""
    for r in radii:
        try:
            out = b.fillet(edges, r)
            if out.is_valid and len(out.solids()) == 1:
                return out, r
        except Exception:                   # noqa: BLE001
            pass
    return s, None


def round_edges_each(s, edges, r):
    """fillet all `edges` at r if OCC can; else drop those it refuses alone (tangent seams, crowded corners) and fillet the rest in groups,
    halving a group OCC refuses (edges re-found by midpoint; corners where rounded edges meet need them filleted together).
    Returns (shape, 'done/total')"""
    out, used = round_edges(s, edges, (r,))
    if used:
        return out, f'{len(edges)}/{len(edges)}'
    ok = [e.position_at(0.5) for e in edges if round_edges(s, [e], (r,))[1]]

    def go(s, mids):
        es = [e for e in s.edges() if any((e.position_at(0.5) - m).length < 1e-3 for m in mids)]
        if not es:
            return s, 0
        out, used = round_edges(s, es, (r,))
        if used:
            return out, len(mids)
        if len(mids) == 1:
            return s, 0
        s, n1 = go(s, mids[:len(mids) // 2])
        s, n2 = go(s, mids[len(mids) // 2:])
        return s, n1 + n2
    s, done = go(s, ok)
    return s, f'{done}/{len(edges)}'


def arm(outline, x0, x1, bed, info, name):
    """an arm: y-z outline prismed over x0..x1, its outline rounded on the face away from the print bed (x = bed) and chamfered 45 deg on the
    bed face (a round there would overhang)"""
    a = prism(outline, x0, x1)
    top = x1 if abs(bed - x0) < 1e-6 else x0
    a, info[name + '_edges'] = round_edges(a, [e for e in a.edges() if on_x(e, (top,))], (FORK_EDGE_R, 1.0))
    try:
        a = b.chamfer([e for e in a.edges() if on_x(e, (bed,))], 0.8)
    except Exception:                       # noqa: BLE001
        info[name + '_bed_chamfer'] = None
    return a


def tenon(grow=0.0):
    """tenon on the bar's end, gabled at 45 deg so the socket in the inner half (printed motor face down) has a self-supporting roof"""
    y0, y1 = BASE_Y[0] + TENON_INSET - grow, BASE_Y[1] - TENON_INSET + grow
    z0, z1 = BASE_Z[0] + TENON_INSET - grow, BASE_TOP - TENON_INSET + grow
    xe = SPLIT_X + TENON_L + grow
    gable = b.extrude(b.Plane.XY.offset(z0) * b.Polygon((SPLIT_X - 0.01, y0), (xe, y0), (xe + (y1 - y0) / 2, (y0 + y1) / 2), (xe, y1), (SPLIT_X - 0.01, y1),
                                                       align=None), amount=z1 - z0)
    return gable


def tie_holes():
    """the tie bolts' holes: (cut from the outer half, cut from the inner half). Each bolt threads into a Ø2.9 pilot from its head seat to 0.8
    past its tip (round 10; round 9 had Ø3.4 then 10 mm of Ø2.8); the inner bolts pass the inner half in Ø3.4, so they clamp it to the bar"""
    outer, inner = [], []
    for (y, z), seats in zip(TIE_LINES, TIE_SEATS):
        for side, seat in enumerate(seats):
            s = (1, -1)[side]
            tip = seat + s * TIE_L
            face = (OA_X[0], INNER_W_X[1])[side]
            cbore = along_x(TIE_CBORE_D / 2, *sorted((face - s * 0.1, seat)), y, z)
            pilot = along_x(TIE_PILOT_D / 2, *sorted((seat, tip + s * 0.8)), y, z)
            if side == 0:
                outer += [cbore, pilot]
            else:
                outer.append(pilot)
                inner += [cbore, along_x(M3_CLR / 2, SPLIT_X - 1.0, seat + 0.01, y, z)]
    return outer, inner


def tie_bolts():
    """M3 x 50 pan heads, one from each end on each line (moving with the fork)"""
    out = {}
    for side, nm in ((0, 'outer'), (1, 'inner')):
        s = (1, -1)[side]
        for i, ((y, z), seats) in enumerate(zip(TIE_LINES, TIE_SEATS)):
            seat = seats[side]
            out[f'bolt_tie_{nm}_{i + 1}'] = (along_x(TIE_HEAD[0] / 2, *sorted((seat - s * TIE_HEAD[1], seat)), y, z)
                                             + along_x(1.5, *sorted((seat, seat + s * TIE_L)), y, z))
    return out


def build_fork():
    """returns the outer half, the inner half and what the rounding managed"""
    info = {}
    # outer half: outer arm, axle stub, the base bar up to the split plane and its tenon
    outer = arm(hull2c((0, 0), OUTER_BOSS_R, PIV, OA_LOBE_R), *OA_X, OA_X[0], info, 'outer_arm')
    # bar side profile (x-z): top z -8 at the arm, down to -10 under the keel, up at 45 deg (prints upright without support) to the pivot height
    xk = KEEL_X[0] - 1.5
    bar = xz_poly([(OA_X[0], BASE_Z[0]), (SPLIT_X, BASE_Z[0]), (SPLIT_X, BASE_TOP), (BASE_RAISE_X + BASE_TOP + BASE_KEEL_CUT, BASE_TOP),
                   (BASE_RAISE_X, -BASE_KEEL_CUT), (xk, -BASE_KEEL_CUT), (xk, BASE_Z[1]), (OA_X[0], BASE_Z[1])], *BASE_Y)
    bar, info['bar_long_edges'] = round_edges(bar, [e for e in bar.edges() if abs(e.position_at(1).X - e.position_at(0).X) > 20], (FORK_EDGE_R + 0.5, FORK_EDGE_R))
    outer = outer + (bar - along_x(WHEEL_CLEAR_R, OA_X[1], SPLIT_X))
    roots = [e for e in outer.edges() if on_x(e, (OA_X[1],)) and e.geom_type.name == 'LINE' and abs(e.center().Z + BASE_KEEL_CUT) < 0.5]
    outer, info['bar_root'] = round_edges_each(outer, roots, FORK_ROOT_R)
    xs0 = X_PLATE_FLOOR - STUB_SHOULDER_GAP                              # axle stub (round 8: a separate part bolted in from outside)
    stub = along_x(STUB_SHOULDER_D / 2, OA_X[1] - 1.0, xs0) + along_x(STUB_D / 2, xs0 - 0.01, X_PLATE_FLOOR + 7.0)
    outer = outer + stub
    outer, info['stub_root'] = round_edges(outer, [e for e in outer.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - OA_X[1]) < 1e-3
                                                  and abs(e.radius - STUB_SHOULDER_D / 2) < 1e-3], (1.5, 1.0))
    outer = outer + tenon()
    xb0 = OA_X[1] - BRG_RECESS                                            # outer 627 from the arm's inner face; Ø14 hole for the sleeve flange + head
    outer = outer - along_x(B627_OD / 2, xb0 - B627_W, xb0 + 0.2, *PIV) - along_x(HEAD_HOLE_D / 2, OA_X[0] - 0.1, xb0 - B627_W + 0.1, *PIV)
    for h in tie_holes()[0]:
        outer = outer - h
    # inner half: motor plate, widened arm (spring pads, inner 627), the bar's last 21 mm, socket for the tenon
    inner = arm(hull2c((0, 0), INNER_PLATE_R, PIV, OA_LOBE_R), *MOTOR_PLATE_X, MOTOR_PLATE_X[0], info, 'motor_plate')
    yp0, yp1 = Y_S - PAD_HALF_L, Y_S + PAD_HALF_L
    wide = (box(INNER_W_X[0], INNER_W_X[1], yp0, yp1, 5.0, PAD_Z)
            + prism(hull2c((yp1 - 6.0, PAD_Z - 6.0), 6.0, PIV, OA_LOBE_R), INNER_ARM_X[0], INNER_W_X[1])
            + prism(hull2c((yp0 + 6.0, PAD_Z - 6.0), 6.0, PIV, OA_LOBE_R), INNER_ARM_X[0], INNER_W_X[1])
            + box(SPLIT_X, INNER_W_X[1], *BASE_Y, BASE_Z[0], BASE_TOP))
    wide, info['wide_arm_edges'] = round_edges(wide, [e for e in wide.edges() if on_x(e, (INNER_W_X[1],))], (FORK_EDGE_R, 1.0))
    inner = inner + wide
    inner, info['motor_plate_root'] = round_edges(inner, [e for e in inner.edges() if on_x(e, (MOTOR_PLATE_X[1],)) and e.center().Y > 30], (FORK_ROOT_R, 2.0, 1.0))
    y, z = W.at_angle(LEAD_NOTCH_R + 12.0, P.lead_azimuth())             # notch where the leads pass (they move with the fork)
    inner = inner - along_x(12.0, MOTOR_PLATE_X[0] - 0.1, MOTOR_PLATE_X[1] + 0.1, y, z)
    xi = INNER_W_X[1] - BRG_RECESS                                         # inner 627: seat in the outer face, relief beyond it
    inner = inner - along_x(B627_OD / 2, xi - B627_W, INNER_W_X[1] + 0.1, *PIV) - along_x(SEAT_RELIEF_D / 2, xi - B627_W - SEAT_RELIEF_T, xi - B627_W + 0.1, *PIV)
    head_x = MOTOR_PLATE_X[0] + MOTOR_SCREW_L - REAR_ENGAGE                 # motor rear face: 4 x M3 x 8 in counterbores
    for a in REAR_HOLES_AZ:
        y, z = W.at_angle(REAR_HOLES_R, a)
        inner = inner - along_x(M3_CLR / 2, MOTOR_PLATE_X[0] - 0.1, MOTOR_PLATE_X[1] + 0.1, y, z) - along_x(REAR_CBORE_D / 2, head_x, MOTOR_PLATE_X[1] + 0.1, y, z)
    inner = inner - tenon(TENON_CLR)
    for h in tie_holes()[1]:
        inner = inner - h
    return outer, inner, info


def build_sleeve():
    xb0 = OA_X[1] - BRG_RECESS
    x1 = xb0 - B627_W
    s = along_x(SLEEVE_D / 2 - 0.02, x1, xb0 - 0.05, *PIV) + along_x(SLEEVE_FLANGE_D / 2, x1 - SLEEVE_FLANGE_T, x1, *PIV)
    return s - along_x(M3_CLR / 2, x1 - SLEEVE_FLANGE_T - 0.1, xb0 + 0.1, *PIV)


def outer_screw():
    x_head = OA_X[1] - BRG_RECESS - B627_W - SLEEVE_FLANGE_T
    return along_x(3.25, x_head - 2.5, x_head, *PIV) + along_x(1.5, x_head, x_head + OUTER_SCREW_L, *PIV)


def bearing(x_face, s):
    """627 at the pivot, from x_face toward s"""
    a, c = sorted((x_face, x_face + s * B627_W))
    return along_x(B627_OD / 2, a, c, *PIV) - along_x(B627_BORE / 2, a - 1, c + 1, *PIV)


def dowel():
    x0 = INNER_W_X[1] - BRG_RECESS - B627_W + 0.2
    return along_x(DOWEL_D / 2, x0, x0 + DOWEL_L, *PIV)


# ---- static parts --------------------------------------------------------------------------------------------------------------------
def stops(fork):
    """droop hook top (the widened inner arm's underside at droop) and the bump rail underside (the pad's top at bump), as y-z lines"""
    ya, yb = Y_S - P.STOP_HALF, Y_S + P.STOP_HALF
    lx = (FI_X[0] - LEDGE_W, FI_X[0])
    down = fork.rotate(PIVOT_AXIS, A_DROOP)
    za, zb = ((down & box(lx[0], lx[1], y - 0.1, y + 0.1, -60, 20)).bounding_box().min.Z for y in (ya, yb))
    pf = rot_pt((Y_S - PAD_HALF_L, PAD_Z), A_BUMP)
    pb = rot_pt((Y_S + PAD_HALF_L, PAD_Z), A_BUMP)
    m = (pb[1] - pf[1]) / (pb[0] - pf[0])
    y0, y1 = float(pf[0]) + 0.5, Y_S + TOWER_HALF_L
    return dict(hook=((ya, float(za)), (yb, float(zb))), rail=((y0, float(pf[1] + m * 0.5)), (y1, float(pf[1] + m * (y1 - pf[0])))))


def build_inner_frame(st):
    x0, x1 = FI_X
    top_foot = PLATE_TOP_L + FOOT_T
    (ha, hb), (ra, rb) = st['hook'], st['rail']
    z_bot = min(ha[1], hb[1]) - HOOK_T
    yc0, yc1 = Y_S - TOWER_HALF_L, Y_S + TOWER_HALF_L
    pts = [(FOOT_Y_L[0], PLATE_TOP_L), (FOOT_Y_L[1], PLATE_TOP_L), (FOOT_Y_L[1], top_foot),
           (GUSSET_END_L, top_foot), (yc1, GUSSET_TOP_Z), (yc1, P.Z_TT), (yc0, P.Z_TT), (yc0, top_foot), (FOOT_Y_L[0], top_foot)]
    f = yz_poly(pts, x0, x1)
    for r, at in ((FILLET_COL, [(yc0, top_foot)]), (FILLET_GUSSET, [(yc1, GUSSET_TOP_Z), (GUSSET_END_L, top_foot)])):
        f = f.fillet(r, [e for e in f.edges() if abs(e.length - FRAME_T) < 1e-3 and any(abs(e.center().Y - y) < 1e-3 and abs(e.center().Z - z) < 1e-3 for y, z in at)])
    tx = (SPRING_X[0] - (SPRING_HOLE_D + 0.6) / 2 - 3.4, x0)
    f = f + box(tx[0], tx[1], yc0, yc1, P.Z_TBOT, P.Z_TT)                                   # 2 x 2 spring tower over the widened arm
    wy = WASHER_OD / 2 + RAIL_CLEAR
    for ry in ((yc0, Y_PAIR[0] - wy), (Y_PAIR[-1] + wy, yc1)):                              # bump rails in front of / behind the springs
        rail = yz_poly([ra, rb, (rb[0], P.Z_TBOT + 0.5), (ra[0], P.Z_TBOT + 0.5)], *tx)
        f = f + (rail & box(tx[0] - 1, tx[1] + 1, ry[0], ry[1], -50, 100))
    f = f + P.yz_quad(ha, hb, -HOOK_T, x0 - LEDGE_W, x0)                                    # droop hook under the widened arm
    f = f + along_x(PIVOT_BLOCK_R, x0, x1, *PIV) + box(x0, x1, PIVOT_Y - PIVOT_BLOCK_R, PIVOT_Y + PIVOT_BLOCK_R, PLATE_TOP_L, PIVOT_Z)   # pivot block
    # round 10: round every edge but those on the print face (outer, x1) and the plate face
    f, rounded = round_edges_each(f, [e for e in f.edges() if not on_x(e, (x1,)) and not all(abs(e.position_at(t).Z - PLATE_TOP_L) < 1e-3 for t in (0, 0.5, 1))],
                                  FRAME_EDGE_R)
    for xs in SPRING_X:
        for y in Y_PAIR:
            f = f - cyl((SPRING_HOLE_D + 0.6) / 2, P.Z_CEIL - P.Z_TBOT + 0.1, (xs, y, (P.Z_CEIL + P.Z_TBOT) / 2 - 0.05))
            f = f - cyl(GUIDE_R, GUIDE_LEN + 0.2, (xs, y, P.Z_CEIL + GUIDE_LEN / 2))
    f = f + along_x(PIVOT_BOSS_D / 2, x0 - FRAME_GAP, x0 + 0.5, *PIV)                       # boss onto the 627's inner race
    f = f - along_x(DOWEL_D / 2 - 0.05, x0 - FRAME_GAP - 0.1, x1 + 0.1, *PIV)               # dowel press bore, through (VERIFY fit)
    for y in FOOT_SCREW_Y:                                                                   # round 12: pilots for M3s from under the chassis
        f = f - cyl(M3_PILOT / 2, FOOT_T + 0.1, ((x0 + x1) / 2, y, PLATE_TOP_L + FOOT_T / 2 - 0.05))   # (round 11: Ø3.4, heads on top;
    for xs, ys in SUPPORT_PILOTS:                                                            # its horizontal side pilots are gone)
        f = f - cyl(M3_PILOT / 2, SUPPORT_PILOT_DEPTH + 0.1, (xs, ys, P.Z_TT - SUPPORT_PILOT_DEPTH / 2 + 0.05))
    for xs, ys in FRAME_BOLTS:                                                               # through-bolt: column top into the foot
        z0, z1 = FRAME_BOLT_PILOT
        f = f - cyl(M3_CLR / 2, P.Z_TT - z1 + 0.1, (xs, ys, (P.Z_TT + z1) / 2))
        f = f - cyl(M3_PILOT / 2, z1 - z0 + 0.1, (xs, ys, (z0 + z1) / 2))
        f = f - cyl(CBORE_D / 2, CBORE_DEPTH + 0.1, (xs, ys, P.Z_TT - CBORE_DEPTH / 2 + 0.05))
    return f, dict(x=list(FI_X), edge_round=rounded, z_bottom=round(z_bot, 2), z_top=round(P.Z_TT, 2), droop_hook_top=[[round(v, 2) for v in p] for p in (ha, hb)],
                   bump_rail_underside=[[round(v, 2) for v in p] for p in (ra, rb)])


def xz_poly(pts, y0, y1):
    """closed x-z polygon extruded over y0..y1"""
    return b.extrude(b.Plane.XZ.offset(-y1) * b.Polygon(*pts, align=None), amount=y1 - y0)


def build_keel():
    """outer keel: round the pivot inside the outer arm, over the plate's opening for the fork's base bar, standing on the plate behind it.
    Round 9: prints on its flat outer face (an M4 washer on the 627's inner race instead of the printed boss); a gusset on the foot makes it
    an L in plan against side load at the pivot; a boss round the pivot pilot on the wheel side (the rollers are >= 10 mm away radially there).
    Round 11: side plate 9.2 thick and up to z 16 behind the pivot; the gusset is a full block (side plate to the foot's end, z -8..16), so
    in plan the keel is a solid L; its two M3 x 30 run from the block's top down through it all into the chassis"""
    top = KEEL_TOP_Z
    (y0, y1), rb = KEEL_FOOT['y'], KEEL_BOSS_R
    # side plate (y-z): round the pivot at the front, rising behind it to the top, back to the foot's end, down to the plate, under the plate's
    # opening at z -7
    side = [arc(V(PIVOT_Y, -rb), V(PIVOT_Y - rb, PIVOT_Z), V(PIVOT_Y, rb)), line(V(PIVOT_Y, rb), V(KEEL_TOP_Y, top)), line(V(KEEL_TOP_Y, top), V(y1, top)),
            line(V(y1, top), V(y1, PLATE_TOP_L)), line(V(y1, PLATE_TOP_L), V(y0, PLATE_TOP_L)), line(V(y0, PLATE_TOP_L), V(y0, -rb)), line(V(y0, -rb), V(PIVOT_Y, -rb))]
    k = prism(side, *KEEL_X)
    # the foot and the block (behind the fork's base bar): an L in plan with the side plate
    k = k + box(KEEL_X[1] - 0.01, KEEL_FOOT['x'][1], y0, y1, PLATE_TOP_L, PLATE_TOP_L + KEEL_FOOT['t'])
    k = k + box(KEEL_X[1] - 0.01, KEEL_FOOT['x'][1], *KEEL_GUSSET_Y, PLATE_TOP_L, top)
    k = k + along_x(KEEL_PIN_BOSS_R, KEEL_X[1] - 0.5, KEEL_PIN_BOSS_X, *PIV)
    # cove (R KEEL_COVE_R) in the inside corner of the L, side plate into the block: a solid, since OCC will not fillet it with the rest.
    # Then round every edge but those on the print face and the plate face
    c, yb = KEEL_COVE_R, KEEL_GUSSET_Y[0]
    cove = (box(KEEL_X[1] - 0.01, KEEL_X[1] + c, yb - c, yb + 0.01, -rb, top) - cyl(c, 2 * top + 2 * rb + 2, (KEEL_X[1] + c, yb - c, 0))) \
        & prism([line(V(yb - c - 1, -rb), V(yb + 1, -rb)), line(V(yb + 1, -rb), V(yb + 1, top)), line(V(yb + 1, top), V(KEEL_TOP_Y, top)),
                 line(V(KEEL_TOP_Y, top), V(yb - c - 1, top - (KEEL_TOP_Y - yb + c + 1) * (top - rb) / (KEEL_TOP_Y - PIVOT_Y))),
                 line(V(yb - c - 1, top - (KEEL_TOP_Y - yb + c + 1) * (top - rb) / (KEEL_TOP_Y - PIVOT_Y)), V(yb - c - 1, -rb))], KEEL_X[1] - 1, KEEL_X[1] + c + 1)
    k = k + cove
    k, r = round_edges_each(k, [e for e in k.edges() if not on_x(e, (KEEL_X[0],)) and not all(abs(e.position_at(t).Z - PLATE_TOP_L) < 1e-3 for t in (0, 0.5, 1))],
                            KEEL_EDGE_R)
    k = k - along_x(M3_PILOT / 2, KEEL_X[0] - 0.1, KEEL_PIN_BOSS_X + 0.1, *PIV)          # M3 x 20 from outside, through the sleeve and the washer
    for x, y in KEEL_SCREWS:                                                                 # round holes (Fernando: no teardrop points)
        k = k - cyl(M3_PILOT / 2, KEEL_PILOT_TOP - PLATE_TOP_L + 0.1, (x, y, (KEEL_PILOT_TOP + PLATE_TOP_L) / 2 - 0.05))   # round 12: blind
    x_tip = OA_X[1] - BRG_RECESS - B627_W - SLEEVE_FLANGE_T + OUTER_SCREW_L                 # pilots from the foot
    return k, dict(top=top, edge_round=r, l_cove_r=KEEL_COVE_R, pivot_screw_engagement=round(x_tip - KEEL_X[0], 2), pivot_screw_tip_to_boss_end=round(KEEL_PIN_BOSS_X - x_tip, 2),
                   foot_screw=f'M3 x {KEEL_SCREW_L:g} up from under the chassis into a blind pilot to z {KEEL_PILOT_TOP:g}')


def frame_bolt():
    """round 12: the inner frame's through-bolt record (pod frame): column top (flush) down into the foot"""
    import hardware as H
    xs, ys = FRAME_BOLTS[0]
    return H.pan((xs, ys, P.Z_TT - CBORE_DEPTH), (0, 0, -1), FRAME_BOLT_L, 'bolt_frame')


def build_static():
    fork_outer, fork_inner, info = build_fork()
    st = stops(fork_inner)
    parts = dict(fork_outer=fork_outer, fork_inner=fork_inner, sleeve=build_sleeve())
    parts['frame_inner'], finfo = build_inner_frame(st)
    parts['keel'], kinfo = build_keel()
    return parts, dict(inner_frame=finfo, keel=kinfo, fork_rounds=info)


def washer():
    """M4 washer between the outer 627's inner race and the keel"""
    xb0 = OA_X[1] - BRG_RECESS
    return along_x(WASHER_OD / 2, xb0, xb0 + PIVOT_WASHER_T, *PIV) - along_x(GUIDE_R, xb0 - 0.1, xb0 + PIVOT_WASHER_T + 0.1, *PIV)


def posed(parts, unit, a):
    xb0 = OA_X[1] - BRG_RECESS
    moving = dict(unit, pod_fork_outer=parts['fork_outer'], pod_fork_inner=parts['fork_inner'],
                  bearing_627_outer=bearing(xb0, -1), bearing_627_inner=bearing(INNER_W_X[1] - BRG_RECESS, -1), **tie_bolts())
    out = {k: (v.rotate(PIVOT_AXIS, a) if a else v) for k, v in moving.items()}
    out['pod_frame_inner'], out['pod_keel_outer'], out['pod_sleeve'] = parts['frame_inner'], parts['keel'], parts['sleeve']
    out['bolt_pivot_outer'], out['dowel_inner'], out['washer_pivot_outer'] = outer_screw(), dowel(), washer()
    lifts = {}
    for k, (xs, y) in PLUNGERS.items():
        lifts[k] = lift = P.plunger_lift(a, y)[0]
        out[f'spring_plunger_{k}'] = P.plunger(xs, y).moved(b.Location((0, 0, lift)))
        out[f'die_spring_{k}'] = P.die_spring(xs, y, P.Z_CEIL - P.Z_SB - lift).moved(b.Location((0, 0, lift)))
    return out, lifts


STATIC = ('pod_frame_inner', 'pod_keel_outer', 'pod_sleeve', 'bolt_pivot_outer', 'dowel_inner', 'washer_pivot_outer')
PRINTED = (('pod_frame_inner', 'frame_inner'), ('pod_keel_outer', 'keel'), ('pod_fork_outer', 'fork_outer'), ('pod_fork_inner', 'fork_inner'),
           ('pod_sleeve', 'sleeve'))
FORK = ('pod_fork_outer', 'pod_fork_inner')
PRINT_UP = {'frame_inner': -1, 'keel': 1, 'fork_outer': 1, 'fork_inner': 1}   # print direction along x (each part lies on its flat x face)


def overhang_mm2(shape, sign, lim=0.7072, tol=0.1):
    """area of faces pointing down more than 45 deg when the part prints with +x (sign 1) or -x up, off the bed: bridges and overhangs"""
    v, t = shape.tessellate(tol, 0.2)
    Pv, T = np.array([[q.X, q.Y, q.Z] for q in v]), np.array(t)
    A, B_, C = Pv[T[:, 0]], Pv[T[:, 1]], Pv[T[:, 2]]
    n = np.cross(B_ - A, C - A)
    ar = np.linalg.norm(n, axis=1) / 2
    nx = sign * n[:, 0] / np.maximum(2 * ar, 1e-12)
    h = sign * (A[:, 0] + B_[:, 0] + C[:, 0]) / 3
    m = (nx < -lim) & (h > (sign * Pv[:, 0]).min() + 0.3)
    return round(float(ar[m].sum()), 1)


def check(parts, unit):
    rep = dict(poses_deg={k: round(v, 3) for k, v in POSES.items()})
    rotating = [k for k in unit if k.startswith(('wheel_', 'roller_', 'bolt_', 'bearing_6804'))]
    harness = ('xt30_housing', 'lead_pos', 'lead_neg')
    for name, a in POSES.items():
        sh, lifts = posed(parts, unit, a)
        ov = {}
        moving = [k for k in sh if k not in STATIC and not k.startswith(('spring_plunger', 'die_spring'))]
        for st in STATIC:
            for k in moving:
                if near(sh[st], sh[k]):
                    v = overlap(sh[st], sh[k])
                    if v > 0.01:
                        ov[f'{st} / {k}'] = round(v, 2)
        for pk in PLUNGERS:
            for k in sh:
                if k.endswith(pk) or not near(sh[f'spring_plunger_{pk}'], sh[k]):
                    continue
                v = overlap(sh[f'spring_plunger_{pk}'], sh[k])
                if v > 0.05:
                    ov[f'spring_plunger_{pk} / {k}'] = round(v, 2)
        r = dict(axle=[round(float(v), 2) for v in rot_pt((0, 0), a)], plunger_lift={k: round(v, 2) for k, v in lifts.items()},
                 interference_mm3=ov)
        g = dict(inner_frame_to_rotating=min(min_dist(sh['pod_frame_inner'], sh[k]) for k in rotating if near(sh['pod_frame_inner'], sh[k], 10)),
                 keel_to_rotating=min(min_dist(sh['pod_keel_outer'], sh[k]) for k in rotating if near(sh['pod_keel_outer'], sh[k], 10)),
                 inner_frame_to_fork=min(min_dist(sh['pod_frame_inner'], sh[f]) for f in FORK),
                 keel_to_fork=min(min_dist(sh['pod_keel_outer'], sh[f]) for f in FORK),
                 keel_to_tie_bolts=min(min_dist(sh['pod_keel_outer'], sh[k]) for k in sh if k.startswith('bolt_tie')),
                 static_to_harness=min(min_dist(sh[s], sh[k]) for s in STATIC[:2] for k in harness))
        r['gaps_mm'] = {k: round(float(v), 2) for k, v in g.items()}
        if name == 'nominal':
            r['fork_to_rotating (stub: not its 6804)'] = {f: round(min(min_dist(sh[f], sh[k]) for k in rotating if k != 'bearing_6804_left' and near(sh[f], sh[k], 6)), 2)
                                                         for f in FORK}
            r['fork_to_harness'] = round(min(min_dist(sh[f], sh[k]) for f in FORK for k in harness), 2)
            r['fork_halves_overlap_mm3'] = round(overlap(sh[FORK[0]], sh[FORK[1]]), 3)
            r['fork_halves_gap_mm'] = round(min_dist(sh[FORK[0]], sh[FORK[1]]), 3)
        bb = (sh['pod_fork_outer'] & box(KEEL_X[1] + 2, INNER_ARM_X[0] - 2, 70, 100, -40, 5)).bounding_box()
        r['fork_base_z'] = [round(float(bb.min.Z), 2), round(float(bb.max.Z), 2)]
        rep[name] = r
    rep['printed_parts'] = {k: dict(solids=len(parts[p].solids()), valid=bool(parts[p].is_valid), volume_cm3=round(parts[p].volume / 1000, 2),
                                    bbox=[round(v, 1) for v in tuple(parts[p].bounding_box().size)],
                                    **({'overhang_mm2_as_printed': overhang_mm2(parts[p], PRINT_UP[p])} if p in PRINT_UP else {})) for k, p in PRINTED}
    # tie bolts: hole walls and what each bolt clamps / threads into
    tb = tie_bolts()
    rep['tie_bolts'] = dict(lines_yz=[list(t) for t in TIE_LINES], bolt=f'4 x M3 x {TIE_L:g} pan head, Ø{TIE_PILOT_D} pilot their whole length',
                            seats_x_outer_inner=[list(t) for t in TIE_SEATS],
                            threaded_x=[[sorted((se[0], se[0] + TIE_L)), sorted((se[1] - TIE_L, SPLIT_X))] for se in TIE_SEATS],
                            bar_x_without_a_bolt=[round(x, 1) for x in np.arange(OA_X[0], SPLIT_X, 0.5)
                                                  if not any(se[0] <= x <= se[0] + TIE_L or se[1] - TIE_L <= x <= se[1] for se in TIE_SEATS)],
                            bolt_overlap_with_fork_mm3={k: round(sum(overlap(v, parts[p]) for p in ('fork_outer', 'fork_inner')), 1) for k, v in tb.items()},
                            min_wall_to_627_seats_mm=round(min(math.hypot(y - PIVOT_Y, z - PIVOT_Z) for y, z in TIE_LINES) - TIE_CBORE_D / 2 - B627_OD / 2, 2))
    sh, _ = posed(parts, unit, 0.0)
    bb = b.Compound([v for k, v in sh.items() if not k.startswith(('spring_plunger', 'die_spring'))]).bounding_box()
    rep['envelope_nominal'] = dict(x=[round(bb.min.X, 1), round(bb.max.X, 1)], y=[round(bb.min.Y, 1), round(bb.max.Y, 1)], z=[round(bb.min.Z, 1), round(bb.max.Z, 1)])
    return rep


# ---- export / render -----------------------------------------------------------------------------------------------------------------
STEP_SKIP = ('bolt_', 'bearing_', 'spring_plunger', 'dowel', 'washer')


def export(parts, unit):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    for n, p in PRINTED:
        write_part(parts[p], colour(n), OUT / 'parts' / (n + '.step'), n)
    sh, _ = posed(parts, unit, 0.0)
    write_step({k: (v, colour(k)) for k, v in sh.items() if not k.startswith(STEP_SKIP)}, OUT / 'pod_lean_assembly.step', 'pod_lean')


def iso(items, out, az_deg, el, title, colour_fn=colour, tol=0.4):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    a, e = np.radians(az_deg), np.radians(el)
    Rm = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]]) @ np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    polys = []
    for k, s in items.items():
        v, t = s.tessellate(tol, 0.4)
        P3 = np.array([[q.X, q.Y, q.Z] for q in v]) @ Rm.T
        col = np.array(colour_fn(k))
        for tri in np.array(t):
            Q = P3[tri]
            n = np.cross(Q[1] - Q[0], Q[2] - Q[0])
            nn = np.linalg.norm(n)
            if nn == 0 or n[1] > 0:
                continue
            polys.append((Q[:, 1].mean(), Q[:, [0, 2]], tuple(col * (0.45 + 0.55 * abs(n[1] / nn)))))
    polys.sort(key=lambda p: -p[0])
    fig, ax = plt.subplots(figsize=(11, 9))
    ax.add_collection(PolyCollection([p[1] for p in polys], facecolors=[p[2] for p in polys], edgecolors='none'))
    ax.autoscale()
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title(title)
    fig.savefig(out, dpi=95)
    plt.close(fig)


def render(parts, unit):
    for f in VAL.glob('*.png'):
        f.unlink()
    sh, _ = posed(parts, unit, 0.0)
    keep = {k: v for k, v in sh.items() if not k.startswith(('bolt_', 'roller_core'))}
    keep['bolt_pivot_outer'] = sh['bolt_pivot_outer']
    iso(keep, VAL / 'iso_motor_side.png', 235, 22, 'lean pod, motor (robot-centre) side, nominal')
    iso(keep, VAL / 'iso_outer_side.png', 55, 22, 'lean pod, outer (perimeter) side, nominal')
    iso({k: sh[k] for k in sh if k.startswith(('pod_', 'bearing_627', 'spring_plunger', 'die_spring', 'bolt_pivot', 'bolt_tie', 'dowel', 'washer'))},
        VAL / 'iso_static_fork.png', 215, 30, 'inner frame, outer keel, fork halves, 627s, plungers, tie bolts (wheel hidden)')
    iso({k: sh[k] for k in sh if k.startswith(('pod_', 'bearing_627', 'bolt_pivot', 'bolt_tie', 'dowel', 'washer'))},
        VAL / 'iso_from_below.png', 200, -35, 'from below: fork base bar under the pivot, outer keel inside the arm')
    # round 9: the fork halves pulled apart along x, with the tie bolts, and the keel alone
    apart = {'pod_fork_outer': parts['fork_outer'].moved(b.Location((-40, 0, 0))), 'pod_fork_inner': parts['fork_inner'].moved(b.Location((40, 0, 0)))}
    for k, v in tie_bolts().items():
        apart[k] = v.moved(b.Location((-90 if '_outer_' in k else 90, 0, 0)))
    iso(apart, VAL / 'iso_fork_halves.png', 215, 25, 'fork halves pulled apart (outer: arm + stub + bar + tenon; inner: motor plate, pads, socket), tie bolts out')
    iso(apart, VAL / 'iso_fork_halves_back.png', 35, 25, 'fork halves from behind: tie-bolt counterbores, tenon / socket', tol=0.2)
    iso({'pod_keel_outer': parts['keel'], 'washer_pivot_outer': washer(), 'bolt_pivot_outer': outer_screw(), 'pod_sleeve': parts['sleeve']},
        VAL / 'iso_keel.png', 120, 25, 'outer keel (wheel side), round 11: 9.2 side plate, full block, flush M3 x 30 heads; sleeve, washer, M3 x 20', tol=0.15)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    fig, axs = plt.subplots(1, 2, figsize=(18, 6))
    for ax, (zc, title) in zip(axs, ((TIE_LINES[0][1], 'tie line 1'), (TIE_LINES[1][1], 'tie line 2'))):
        sl = box(-80, 80, -10, 120, zc - 0.25, zc + 0.25)
        for k, s in dict(pod_fork_outer=parts['fork_outer'], pod_fork_inner=parts['fork_inner'], pod_keel_outer=parts['keel'], **tie_bolts()).items():
            x = s & sl
            if x is None or not x.faces():
                continue
            v, t = x.tessellate(0.1, 0.2)
            Pv = np.array([[q.X, q.Y] for q in v])
            ax.add_collection(PolyCollection([Pv[tri] for tri in np.array(t)], facecolors=[colour(k)], edgecolors='none'))
        ax.set_xlim(-60, 70)
        ax.set_ylim(60, 100)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_title(f'{title}: section z = {zc:g} (x along the bar, y toward the back); split plane x {SPLIT_X:g}', fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'section_tie_bolts.png', dpi=90)
    plt.close(fig)


def main(stage):
    for d in (OUT, VAL):
        d.mkdir(parents=True, exist_ok=True)
    parts, info = build_static()
    for k in ('fork_outer', 'fork_inner', 'sleeve', 'frame_inner', 'keel'):
        print(k, len(parts[k].solids()), parts[k].is_valid, round(parts[k].volume), flush=True)
    print(json.dumps(info), flush=True)
    if stage == 'build':
        return
    unit = P.wheel_unit()
    if stage in ('check', 'all'):
        rep = check(parts, unit)
        rep['static'] = info
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(parts, unit)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(parts, unit)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
