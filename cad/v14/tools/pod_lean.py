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

Round 13 (Nov 4, Fernando's review of v12): no thread-forming pilot in a pod part runs across its layers (the inner frame and the keel print
on their sides, so a vertical pilot is in their layer plane and the screw's wedging splits them):
 * every screw from the chassis goes into a plain M3 hex nut in a slot instead of a pilot: the inner frame's three M3 x 25 (slots open to
   its outer face, which the chassis pocket wall covers), the keel's two M3 x 25 (one slot from the foot's inboard end, covered by the
   pocket's end wall). The nuts go in on the bench and stay in the pod;
 * the column bolt comes up from under the chassis (Fernando: removable with the pod in place from below): M3 x 60 through the member, the
   foot and the column into a nut in a slot 30 mm up the column; the column top is closed.

Round 14 (Nov 4, Fernando's review of round 13):
 * the keel's two screws thread into a Ø2.9 pilot through the whole keel (M3 x 35, 16.9 mm engaged) instead of hex nuts in a slot: a 2.9
   pilot is gentle enough not to split the layers it lies in;
 * the inner frame's pivot block no longer dips 1 mm under its foot (its round, r 9 about the pivot, went to z -9): the chassis' slide-out
   clip took that as part of the pod's way out and cut the ring from under the foot (y 76..86, the whole foot width) on every pod.

Nov 13 (cad/v14, Fernando's review; rules in cad/solidworks.md 18: no captive nuts, holes through, countersinks 0.5 deeper, no dowels):
 * the tenon fits its socket with 0.05 a face (was 0.1, printed too loose);
 * the inner pivot's Ø7 x 20 steel dowel is gone: a printed Ø7 pin on the inner frame (one part) in the inner 627, an M3 x 16 from the frame's
   outer face threaded along its axis as its core;
 * the inner frame's four chassis screws thread into Ø2.9 pilots (no nuts); every hole runs through; the side fin's support pilots Ø2.9 through;
 * a printed retainer on the 6804's inner race (build_retainer), screwed from the outer arm's outer face through the stub: the bearing cannot
   slide in and the stub's shoulder (gap 0) clamps it;
 * an "up" arrow engraved on each fork half; the motor leads tied to the inner half by a zip tie through a tunnel under the plate's rim.

Nov 13 evening (Fernando):
 * inner frame: the short threads (the three foot screws, 6.2 mm; the side fin's M3 x 8 studs) in Ø2.8 pilots (were Ø2.9); the column bolt (15.9 mm)
   keeps Ø2.9. A 5.5 AF x 2.5 hex recess in a Ø9.8 x 2.5 pad on the foot's top round the y 23 and 34.5 foot screws, for an optional nut (with
   an M3 x 30); the pad keeps the foot's 8 mm under the nut (6.2 mm of thread without it);
 * inner frame reinforced round the bearing: the pivot block R 13.4 (was 9), its back on the foot's end, the gusset tangent to it, all one solid;
 * outer keel: the pin boss 13.5 long (was 3.5), the pivot screw M3 x 30 (was x 20) in a Ø2.9 pilot, 21.3 mm of thread (was 11.3); the fork bar's
   cut under the keel runs 10 mm further toward the motor (BASE_RAISE_X follows the boss). Behind the pivot the keel is solid out to the boss's
   end (the side plate's profile run on along x), so the boss is one piece with the block that carries the two mounting screws, not a cantilever;
 * snug Ø3.2 where an M3 is a shaft or core in a printed part (cad/solidworks.md 18): the fork's axle stub, the 6804 retainer's spigot and flange,
   the outer pivot's sleeve (all were Ø3.4, the retainer Ø2.8 thread its whole length). The retainer's screw is an M3 x 30 threading 7.9 mm into
   a 9.4 mm boss (was M3 x 25, 6.2 mm, boss 4.5);
 * the pivots' screws clamp each 627's inner race to its static part through printed retainer caps (no washers), and the fork is held axially both
   ways through the bearings (it was held one way only by the outer races' friction in their seats): the outer 627's seat opens on the arm's outer
   face with a 45 deg lip on the keel side, the M3 x 30 bears on a retainer cap and runs through the sleeve (collar onto the keel); the inner 627's
   race is clamped onto the frame's boss by a cap keyed on the pin's hex tip with an M3 nut in its outer face and an M3 x 25 from the frame's outer
   face (cap and nut fitted at the bench before the fork's inner half goes on). See OUTER_BRG_INSET above.
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
import palette as PAL                                                  # noqa: E402
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
PIVOT_WASHER_T = 0.8               # round 9 - Nov 13: an M4 washer between the outer 627's inner race and the keel. Evening: no washer; the 0.8 stays in the
                                   # keel's position (the outer sleeve's collar now spans the arm-to-keel gap)
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
KEEL_PIN_BOSS_R, KEEL_PIN_BOSS_X = 5.0, KEEL_X[1] + 13.5  # boss round the pivot pilot on the keel's wheel side (>= 5.6 mm from the R70 tread
                                   # circle radially in every pose: it is coaxial with the pivot). Nov 13 evening (Fernando): 13.5 long (was 3.5)
                                   # for the M3 x 30 (was x 20): 21.3 mm of thread. The fork bar's cut under the keel follows (BASE_RAISE_X)
KEEL_EDGE_R = 0.8                  # all keel edges (OCC refuses larger at its corners); R3 in the L's inside corner
KEEL_COVE_R = 3.0
FRAME_EDGE_R = 1.0                 # round 10: inner frame edges
KEEL_SCREWS = ((-27.0, 98.0), (-17.0, 98.0))     # vertical M3 from under the chassis into the keel (round 11: y 92.75)
CBORE_D, CBORE_DEPTH = 7.2, 2.9    # flush counterbore for an M3 hex-drive pan head (6.5 x 2.5)
KEEL_SCREW_L = 35.0                # round 14: M3 x 35 up from under the chassis (chassis.BELOW_CB) into a Ø2.9 pilot through the whole keel
                                   # (16.9 mm engaged; Fernando: gentle enough across the layers not to split them). Round 13: x 25 into a nut
FRAME_BOLTS = ((72.6, 54.6),)      # round 11: through-bolt in the inner frame, between the two support pilots (y 44 / 65)
FRAME_BOLT_L = 60.0                # round 13: M3 x 60 up from under the chassis through the member, foot and column into a nut 30 mm up the column
FOOT_SCREW_L = 25.0                # round 13: M3 x 25 up from under the chassis into a hex nut in the inner frame's foot
FOOT_SCREW_Y = (23.0, 34.5, 65.0)  # MOUNT_Y_L and one behind the column. Nov 13 evening: y 65 (was 89.5, under the pivot block, where the frame's new
                                   # 627 seat is now); it shares the side fin's support pilot (x 72.6, y 65: one Ø2.8 hole the whole height)
NUT_SLOT_W, NUT_SLOT_T = 5.7, 2.6  # round 13: slot for an M3 hex nut (5.5 AF x 2.4): width across its flats, height along the screw
FOOT_NUT_Z = -5.0                  # nut's bearing face (its underside) in the inner frame's and the keel's feet (foot z -8..0 / -8..-1)
COLUMN_NUT_Z = 30.0                # round 13: the column bolt's nut (pod z); unused since Nov 13
COLUMN_PILOT_Z = 18.0              # Nov 13: the column bolt's Ø2.9 pilot from here up (its tip at z 33.9: 15.9 mm of thread), clearance below
SLEEVE_D, SLEEVE_FLANGE_D, SLEEVE_FLANGE_T = 7.0, 10.0, 1.0   # printed sleeve in the outer 627's inner race (until Nov 13 evening: its flange on the
                                   # race's outer face under the screw head)
# Nov 13 evening (Fernando: the fork was held to its bearings only by friction one way; the screw must clamp the bearing to the static part). Both static
# parts sit on the bearings' +x side, and both seats stopped the fork only moving +x. Now:
#  * OUTER 627: the seat opens on the arm's OUTER face, a lip on the keel side (a 45 deg cone onto the outer race, then Ø18). The M3 x 30 from outside
#    bears on a printed retainer cap (a Ø10 x 2 flange on the inner race and a spigot into its bore; no washer: a printed one would be weak) and runs
#    through the sleeve, whose spigot enters the race from the other side and whose Ø10 collar passes the lip's hole onto the keel: the inner race is
#    clamped to the keel, and the fork pulled away from the keel (-x) is held by the lip -> outer race -> balls -> inner race -> screw -> keel;
#  * INNER 627 (Fernando, the third try): the bearing lives in the FRAME. Its outer race sits in a seat in the frame's fork-side face (the seat floor,
#    printed facing up, on the outer race only) and stays still; its inner race turns with the fork: the fork's inner half carries a printed Ø7 pin
#    in the race and a Ø10 boss on the race's motor-side face, and an M3 x 20 from the frame's outer face, through a Ø12 bore, bears on a printed
#    Ø10 x 2 cap on the race's other face and threads into the pin: the fork is pressed onto the bearing. Fork pushed +x: boss -> inner race ->
#    balls -> outer race -> the frame's seat floor. (Tries one and two kept the bearing in the fork and clamped its inner race to the frame: a screw
#    head inside the fork, then a keyed cap and nut on the frame's pin; neither could be tightened, Fernando.) The 627 reaches z -11, 3 mm under the
#    frame's foot: the chassis clips round it along the pod's way out, as round the fork's lobe. Fernando: the frame must be compressed too, so a
#    printed clamp ring (upper arc, in a recess in the frame's fork-side face, 2 x M3 x 8 into the frame, fitted at the bench before the fork) clamps
#    the outer race onto the seat floor: frame + bearing clamped by the ring, fork + inner race by the pivot screw (a flat head in the cap).
OUTER_BRG_INSET = 1.5              # outer 627's outer face this far inside the arm's outer face (the cap's flange and head then stand 3.0 proud of it)
XO = OA_X[0] + OUTER_BRG_INSET     # -51.0: outer 627 x XO .. XO + 7, its inner race clamped to the keel
LIP_HOLE_D, LIP_CONE = SEAT_RELIEF_D, 2.0   # the lip's hole (Ø18: on the outer race only) and its 45 deg cone from Ø22 (prints without support)
COLLAR_D = PIVOT_BOSS_D            # the sleeve's collar through the lip's hole onto the keel (Ø10: on the inner race only)
OCAP_D, OCAP_T, SPIGOT_L = PIVOT_BOSS_D, 2.0, 3.0   # outer retainer cap: flange Ø10 x 2 on the inner race; cap and sleeve spigots 3 into the race's bore
OUTER_HEAD_X = XO - OCAP_T         # the outer pivot screw's head bearing point (on the cap)
HEAD_HOLE_D = 14.0                 # (round 9 - Nov 13: the Ø14 hole round the old sleeve flange and head; gone)
OUTER_SCREW_L = 30.0               # Nov 13 evening (Fernando: a longer thread in the keel): M3 x 30, ~19 mm into the keel and its pin boss in a
                                   # Ø2.9 pilot (a long thread: cad/solidworks.md 18). Round 11: x 20, 11.3 mm (round 9-10: 16)
KEEL_PIVOT_PILOT = M3_SOFT_PILOT
INNER_FACE_X = INNER_ARM_X[0] - MOTOR_POCKET_D   # Nov 10: the inner half's motor-side face, MOTOR_POCKET_D in front of the motor's rear face (44.7)
INNER_W_X = (INNER_FACE_X, 65.6)  # inner arm widened behind the motor: spring pads (2 x 2) and the inner 627 seat
MOTOR_PLATE_X = (INNER_FACE_X, INNER_ARM_X[1])   # the plate under the motor's rear face, 13 thick behind the pocket: it carries all 4 springs (round 10 tried 6.7, Fernando: back to 13)
MOTOR_SCREW_L = REAR_SCREW_L      # 4 x M2.5 x 14 pan heads, 3.8 into the motor's 4.3 deep holes
SPRING_X = (50.5, 61.1)            # the two spring columns (x); the tower's wall is 7.3 mm from the rollers
PAD_HALF_L, TOWER_HALF_L = 15.0, 15.0   # pad / tower half length (y): 4.2 mm bump rails in front of and behind the spring pairs
FI_X = (INNER_W_X[1] + FRAME_GAP, INNER_W_X[1] + FRAME_GAP + FRAME_T)   # inner frame
FOOT_Y_L = (18.0, 94.0)            # foot on the plate along the whole frame (round 7 ran on to y 104, over a keel notch; round 13: 92 -> 94,
                                   # wall behind the y 89.5 screw's nut slot)
PIVOT_BLOCK_R = FOOT_Y_L[1] - PIVOT_Y   # inner frame's solid block round the pivot pin, down to the foot. Nov 13 evening (Fernando: reinforce
                                   # round the bearing, mass no concern): R 13.4 (was 9), its back flush with the foot's end, the gusset tangent to it
FOOT_PILOT = M3_PILOT              # Nov 13 evening (Fernando): the inner frame's short threads (the foot screws' 6.2 mm, the fin studs') in Ø2.8
                                   # (were Ø2.9); the column bolt (15.9 mm of thread) keeps Ø2.9
FOOT_NUT_Y = (23.0, 34.5)          # Nov 13 evening (Fernando: nuts for extra grip, optional): a 5.5 AF x 2.5 hex recess on the foot's top round these
                                   # foot screws (a nut there needs an M3 x 30 from below). Not at y 89.5: the pivot block stands over it.
NUT_PAD_D, NUT_PAD_H = 9.8, 2.5    # the recess sits in a round pad on the foot's top, so the screw keeps the foot's 8 mm (6.2 mm of thread; a
                                   # recess cut into the foot left 5.5, under the 6 mm rule). Ø9.8: 0.1 inside the foot's front face at y 23
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
SPLIT_X = INNER_FACE_X             # the halves meet on the inner half's motor-side face (Nov 10: 42.7; was the motor-face plane 44.7)
TIE_LINES = ((78.5, -16.5), (82.0, -24.5))   # (y, z) of the two tie-bolt lines; each head counterbore stays >= 2 mm off the 627 seats
TIE_L, TIE_CBORE_D, TIE_HEAD = 50.0, 6.8, (6.5, 2.5)   # M3 x 50 pan heads
TIE_SEATS = ((OA_X[0] + 3.5, 55.5), (OA_X[0] + 10.5, 62.0))   # round 10, per line (outer, inner) head seats, staggered so that along the bar
                                   # some bolt is threaded in at every x: line 1 from x -49 to 1 and 5.5 to 55.5, line 2 from -42 to 8 and 12 to 62.
                                   # The inner heads sit >= 3 mm above the socket's floor
TIE_PILOT_D = 3.0                  # round 10: the bolts thread into a pilot along their whole length (soft thread through every layer); Nov 10 (Fernando): Ø3.0, was Ø2.9, easier
                                   # to drive 50 mm of screw. The inner ones pass the inner half in Ø3.4 so they clamp it to the bar
TENON_L, TENON_INSET, TENON_CLR = 7.0, 2.5, 0.05  # Nov 10 (Fernando): square tenon on the bar's end, a tight fit in a square socket in the inner half (was 45 deg gabled, 3 + 4.5 long, 0.15 clear).
                                   # Nov 13 (Fernando: the printed fit was too loose): 0.05 a face, was 0.1
FORK_EDGE_R, FORK_ROOT_R = 2.0, 3.0
# Nov 13 (Fernando): a small "up" arrow engraved on each fork half, on its top face as printed (seen when the halves go together): (y, z) of the
# arrow's centre, its length (pointing +z), head width, depth
ARROW_OUTER, ARROW_INNER = (45.0, 0.0), (0.0, 0.0)   # outer half: the outer arm's wheel-side face; inner half: the motor plate's outer face
ARROW_L, ARROW_W, ARROW_STEM, ARROW_DEPTH = 9.0, 5.0, 1.6, 0.4
# Nov 13 (Fernando): a zip tie holds the motor's two leads to the fork, away from the turning wheel. It runs through a tangential tunnel in the inner
# half's motor plate just inside its rim (the plate under the leads is the bridge) and round both leads, which move with the fork. Sized for a
# 3.6 x 1.2 mm tie (as the harness ties, VERIFY on the ties bought); its head drawn 5.0 x 4.5 x 5.6 on top of the leads
LEAD_TIE_X = 52.0                  # tie centre plane (pod x; the motor plate is x 42.7..57.7, its can pocket to 44.7)
LEAD_TIE_W, LEAD_TIE_T = 3.6, 1.2  # strap width (along x), thickness
LEAD_TIE_TUNNEL_R = 26.5           # tunnel centre radius (rim r 29.5: a 2 mm bridge over its 2.0 tall slot)
LEAD_TIE_TUNNEL = (4.6, 2.0)       # tunnel cross-section: along x, radial (the strap + 0.5 / 0.4 a side)
LEAD_TIE_HEAD = (5.0, 4.5, 5.6)    # tie head: tangential, radial, along x
LEAD_TIE_TUNNEL_T = 14.5           # the strap's corners this far either side of the leads' middle (tangential), past where the tunnel leaves the rim
FORK_CORNER_R = 10.0               # Nov 10 (Fernando): where each arm's bottom edge meets the front of the bar's leg: the corner looks down and is in tension under wheel load
# Nov 13 evening (third try, Fernando): the inner 627 in the frame, the fork's inner half pressed onto its inner race. (Nov 13: a printed Ø7 pin on the
# frame with an M3 x 16 core in the 627 in the fork; before that a Ø7 x 20 dowel)
PIN_D = B627_PIN_D                 # the fork's printed pin in the 627's 7.0 bore: Ø7.20, coupon_627 cell 4 (Nov 13 evening, Fernando)
PIN_TIP_CLR = 0.2                  # pin end this far short of the 627's far face (the cap bears on the race, not the pin)
RING_T, RING_TAB_T = 2.5, 4.0       # the clamp ring: 2.5 thick over the outer race, 4.0 at its two screw tabs (pan heads in counterbores)
RING_R = (9.5, 12.5)               # the ring's band: on the outer race only (its inner ring ~Ø18.6, VERIFY) and 1.5 into the frame past the seat
RING_ARC = (-30.0, 210.0)          # its arc (deg from +y toward +z): over the top and sides, not below z -6.25 (the foot ends at z -8)
RING_SCREWS = ((15.0, 90.0), (15.0, 135.0))   # (r, deg) of its two M3 x 8: where the frame is solid behind them (top, front-top; the y 65 pilot clear)
RING_TAB_R, RING_CLR, RING_SCREW_L = 4.5, 0.2, 8.0
FBRG_X0 = FI_X[0] + RING_T         # 69.1: the 627's motor-side face, under the ring
FBRG_X1 = FBRG_X0 + B627_W         # 76.1: its outer face, on the seat floor
FRAME_BORE_D = 12.0                # through the frame's outer part round the cap (Ø10) and head, which turn with the fork
ICAP_D, ICAP_T = PIVOT_BOSS_D, 2.4   # the inner cap: Ø10 x 2.4 on the inner race only, the M3 flat head countersunk in it (0.5 deeper, 0.4 under the cone)
FORK_BOSS_D = PIVOT_BOSS_D         # the fork's boss on the race's motor-side face (Ø10: the inner race only), through the ring's hole
INNER_HEAD_X = FBRG_X1 + ICAP_T - CSK_EXTRA   # 78.0: the flat head's top, 0.6 under the frame's outer face
INNER_PIVOT_SCREW_L = 20.0         # M3 x 20 flat head from the frame's outer face along -x: ~18 mm of thread in the fork's pin and body (Ø2.9, long)
Y_S = P.Y_S
Y_PAIR = P.Y_PAIR
PLUNGERS = {f'inner_{"ab"[j]}{i + 1}': (xs, y) for i, xs in enumerate(SPRING_X) for j, y in enumerate(Y_PAIR)}   # a nearer the axle

EXTRA = {'pod_keel': (0.25, 0.68, 0.74), 'pod_sleeve': (0.66, 0.46, 0.86), 'tie_': (0.12, 0.12, 0.12)}


def colour(name):
    if PAL.colour(name):                                               # printed parts: their filament (palette.py, Nov 13)
        return PAL.colour(name)
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
    """square tenon on the bar's end (grow 0), or the socket it fits in the inner half (grow TENON_CLR, which also leaves the tenon's end clear of the socket floor, so the halves
    seat on the split plane). Nov 10 (Fernando): was gabled at 45 deg for a self-supporting socket roof; the gable did nothing for rigidity, and the square socket's roof is a
    9.2 mm bridge (printed motor face down)"""
    y0, y1 = BASE_Y[0] + TENON_INSET - grow, BASE_Y[1] - TENON_INSET + grow
    z0, z1 = BASE_Z[0] + TENON_INSET - grow, BASE_TOP - TENON_INSET + grow
    xe = SPLIT_X + TENON_L + grow
    t = box(SPLIT_X - 0.01, xe, y0, y1, z0, z1)
    return t if grow else b.chamfer([e for e in t.edges() if on_x(e, (xe,))], 0.5)      # lead-in for the tight fit


def arm_outline(r_axle):
    """y-z outline of an arm: the hull of its axle circle and pivot lobe, with the bottom edge blended into the front of the bar's leg by a FORK_CORNER_R concave arc (the outline
    then runs inside the bar back up to the lobe, so one closed profile takes the arm's edge rounds round the corner too)"""
    a_top, lobe, bottom, axle_end = hull2c((0, 0), r_axle, PIV, OA_LOBE_R)
    B2, B1 = np.array(tuple(bottom.position_at(0)))[1:], np.array(tuple(bottom.position_at(1)))[1:]
    u = (B1 - B2) / np.linalg.norm(B1 - B2)                              # along the bottom edge, toward the axle
    v = np.array([0.0, -1.0])                                            # down the bar's front face
    Pc = B2 + u * (BASE_Y[0] - B2[0]) / u[0]                             # the sharp corner
    half = math.acos(float(u @ v)) / 2
    t = FORK_CORNER_R / math.tan(half)
    T1, T2 = Pc + u * t, Pc + v * t
    w = (u + v) / np.linalg.norm(u + v)
    M = Pc + w * (FORK_CORNER_R / math.sin(half) - FORK_CORNER_R)
    G = np.array([B2[0], T2[1]])                                         # inside the bar
    return [a_top, lobe, line(V(*B2), V(*G)), line(V(*G), V(*T2)), arc(V(*T2), V(*M), V(*T1)), line(V(*T1), V(*B1)), axle_end]


def arrow_cut(x_face, inward, yc, zc):
    """an up (+z) arrow engraved ARROW_DEPTH into the face x = x_face (inward: +1 if the part lies toward +x of the face)"""
    hl, hw, sw = ARROW_L / 2, ARROW_W / 2, ARROW_STEM / 2
    zh = zc + hl - ARROW_W * 0.8                                            # head base
    pts = [(yc - sw, zc - hl), (yc + sw, zc - hl), (yc + sw, zh), (yc + hw, zh), (yc, zc + hl), (yc - hw, zh), (yc - sw, zh)]
    xa, xb = sorted((x_face - inward * 0.1, x_face + inward * ARROW_DEPTH))
    return yz_poly(pts, xa, xb)


_LEADS = {}


def lead_centres():
    """(y, z) of the two motor leads' centrelines where they run along x past the motor plate (straight there), and their radius"""
    if not _LEADS:
        _, _, pins = W.motor_and_plug()
        for i, l in enumerate(W.build_leads(pins)):
            c = (l & along_x(80.0, LEAD_TIE_X - 0.3, LEAD_TIE_X + 0.3)).center()
            _LEADS[i] = (c.Y, c.Z)
    return [_LEADS[0], _LEADS[1]]


def lead_tie_frame():
    """the tie's (t, r) frame in the pod's y-z plane: (unit radial e_r at the leads' middle azimuth, unit tangential e_t)"""
    a = [math.atan2(z, y) for y, z in lead_centres()]
    th = (a[0] + a[1]) / 2
    return np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])


def _tr_face(pts):
    """a face in the plane x = LEAD_TIE_X from (t, r) points"""
    er, et = lead_tie_frame()
    P3 = [b.Vector(LEAD_TIE_X, *(r * er + t * et)) for t, r in pts]
    return b.Face(b.Wire.make_polygon(P3, close=True))


def lead_tie_cuts():
    """the tunnel through the motor plate for the leads' tie (tangential, through both sides of the rim)"""
    w, h = LEAD_TIE_TUNNEL
    r0, r1, t = LEAD_TIE_TUNNEL_R - h / 2, LEAD_TIE_TUNNEL_R + h / 2, 20.0
    return b.extrude(_tr_face([(-t, r0), (t, r0), (t, r1), (-t, r1)]), amount=w / 2, both=True)


def lead_tie():
    """the zip tie as drawn (one solid): a 1.2 thick band round the tunnel's bridge and both leads (drawn round the leads where they are
    drawn: pulled tight it brings them onto the rim, not modelled), 3.6 wide along x, and its head on top of the leads"""
    er, et = lead_tie_frame()
    lc = [(float(np.dot(c, et)), float(np.dot(c, er))) for c in map(np.array, lead_centres())]
    r_top = max(r for _, r in lc) + LEAD_D / 2 + LEAD_TIE_T / 2 + 0.1
    tl = max(abs(t) for t, _ in lc) + LEAD_D / 2 + LEAD_TIE_T / 2 + 0.6
    T = LEAD_TIE_TUNNEL_T
    path = [(-T, LEAD_TIE_TUNNEL_R), (T, LEAD_TIE_TUNNEL_R), (tl, r_top), (-tl, r_top)]
    f = _tr_face(path)
    band = b.offset(f, LEAD_TIE_T / 2, kind=b.Kind.ARC) - b.offset(f, -LEAD_TIE_T / 2, kind=b.Kind.ARC)
    strap = b.extrude(band, amount=LEAD_TIE_W / 2, both=True)
    ht, hr, hx = LEAD_TIE_HEAD
    r0 = r_top + LEAD_TIE_T / 2 - 0.01
    head = b.extrude(_tr_face([(-ht / 2, r0), (ht / 2, r0), (ht / 2, r0 + hr), (-ht / 2, r0 + hr)]), amount=hx / 2, both=True)
    return strap + head


def tie_and_marks(parts):
    """the moving extras drawn with the fork: the motor leads' zip tie"""
    return {'tie_lead': lead_tie()}


def tie_holes():
    """the tie bolts' holes: (cut from the outer half, cut from the inner half). Both bolts on a line thread into one pilot that runs straight
    through the outer half (Nov 10, Fernando: each bolt's blind pilot stopped 0.8 past its tip, leaving a 2.4-2.9 mm plug of PLA between the two tips;
    now only the head seats need support); the inner bolts pass the inner half in Ø3.4, so they clamp it to the bar"""
    outer, inner = [], []
    for (y, z), (seat_o, seat_i) in zip(TIE_LINES, TIE_SEATS):
        outer += [along_x(TIE_CBORE_D / 2, OA_X[0] - 0.1, seat_o, y, z), along_x(TIE_PILOT_D / 2, seat_o, seat_i, y, z)]
        inner += [along_x(TIE_CBORE_D / 2, seat_i, INNER_W_X[1] + 0.1, y, z), along_x(M3_CLR / 2, SPLIT_X - 1.0, seat_i + 0.01, y, z)]
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
    outer = arm(arm_outline(OUTER_BOSS_R), *OA_X, OA_X[0], info, 'outer_arm')
    # bar side profile (x-z): top z -8 at the arm, down to -10 under the keel, up at 45 deg (prints upright without support) to the pivot height
    xk = KEEL_X[0] - 1.5
    bar = xz_poly([(OA_X[0], BASE_Z[0]), (SPLIT_X, BASE_Z[0]), (SPLIT_X, BASE_TOP), (BASE_RAISE_X + BASE_TOP + BASE_KEEL_CUT, BASE_TOP),
                   (BASE_RAISE_X, -BASE_KEEL_CUT), (xk, -BASE_KEEL_CUT), (xk, BASE_Z[1]), (OA_X[0], BASE_Z[1])], *BASE_Y)
    bar, info['bar_long_edges'] = round_edges(bar, [e for e in bar.edges() if abs(e.position_at(1).X - e.position_at(0).X) > 20], (FORK_EDGE_R + 0.5, FORK_EDGE_R))
    outer = outer + (bar - along_x(WHEEL_CLEAR_R, OA_X[1], SPLIT_X))
    roots = [e for e in outer.edges() if on_x(e, (OA_X[1],)) and e.geom_type.name == 'LINE' and abs(e.center().Z + BASE_KEEL_CUT) < 0.5]
    outer, info['bar_root'] = round_edges_each(outer, roots, FORK_ROOT_R)
    xs0 = X_PLATE_FLOOR - STUB_SHOULDER_GAP                              # axle stub (round 8: a separate part bolted in from outside)
    stub = along_x(STUB_SHOULDER_D / 2, OA_X[1] - 1.0, xs0) + along_x(STUB_D / 2, xs0 - 0.01, STUB_END)   # Nov 13: stops short of the retainer's spigot
    outer = outer + stub
    outer, info['stub_root'] = round_edges(outer, [e for e in outer.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - OA_X[1]) < 1e-3
                                                  and abs(e.radius - STUB_SHOULDER_D / 2) < 1e-3], (1.5, 1.0))
    outer = outer + tenon()
    # Nov 13: the 6804 retainer's M3 from the outer face along the axle through the stub (counterbore in the outer face; the hole is through)
    # Nov 13 evening (Fernando: snug Ø3.2 where an M3 is a shaft in a printed part, cad/solidworks.md 18): the screw is the stub's steel core (was Ø3.4)
    outer = outer - along_x(M3_SNUG / 2, OA_X[0] - 0.1, STUB_END + 0.1) - along_x(CBORE_D / 2, OA_X[0] - 0.1, OA_X[0] + CBORE_DEPTH)
    # Nov 13 evening: the outer 627's seat opens on the arm's outer face; a lip on the keel side (45 deg cone onto the outer race, then the Ø18 hole the
    # sleeve's collar passes). Was: the seat open on the inner face, the bearing slid in along a trough in the bar's top, a Ø14 hole for the head
    lip = b.Cone(B627_SEAT_D / 2, LIP_HOLE_D / 2, LIP_CONE, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN)).rotate(b.Axis.Y, 90)
    outer = (outer - along_x(B627_SEAT_D / 2, OA_X[0] - 0.1, XO + B627_W, *PIV) - lip.moved(b.Location((XO + B627_W - 0.001, PIVOT_Y, PIVOT_Z)))
             - along_x(LIP_HOLE_D / 2, XO + B627_W, OA_X[1] + 0.1, *PIV))
    # (Nov 9 - Nov 13: a trough in the bar's top, BRG_PATH_LEN past the arm's inner face, let the bearing slide in from that side; gone with the seat's
    # flip: the bearing goes in from the arm's outer face)
    for h in tie_holes()[0]:
        outer = outer - h
    outer = outer - arrow_cut(OA_X[1], -1, *ARROW_OUTER)                  # Nov 13: "up" mark, on the face that prints on top
    # inner half: motor plate, widened arm (spring pads, inner 627), the bar's last 21 mm, socket for the tenon
    inner = arm(arm_outline(INNER_PLATE_R), *MOTOR_PLATE_X, MOTOR_PLATE_X[0], info, 'motor_plate')
    yp0, yp1 = Y_S - PAD_HALF_L, Y_S + PAD_HALF_L
    wide = (box(INNER_W_X[0], INNER_W_X[1], yp0, yp1, 5.0, PAD_Z)
            + prism(hull2c((yp1 - 6.0, PAD_Z - 6.0), 6.0, PIV, OA_LOBE_R), INNER_W_X[0], INNER_W_X[1])
            + prism(hull2c((yp0 + 6.0, PAD_Z - 6.0), 6.0, PIV, OA_LOBE_R), INNER_W_X[0], INNER_W_X[1])
            + box(SPLIT_X, INNER_W_X[1], *BASE_Y, BASE_Z[0], BASE_TOP))
    wide, info['wide_arm_edges'] = round_edges(wide, [e for e in wide.edges() if on_x(e, (INNER_W_X[1],))], (FORK_EDGE_R, 1.0))
    inner = inner + wide
    inner, info['motor_plate_root'] = round_edges(inner, [e for e in inner.edges() if on_x(e, (MOTOR_PLATE_X[1],)) and e.center().Y > 30], (FORK_ROOT_R, 2.0, 1.0))
    # Nov 10 (Fernando): shallow tight-fit pocket for the can, so the stator drops onto the hole pattern; 0.5 x 45 deg lead-in at its mouth
    xm, rp = MOTOR_PLATE_X[0], MOTOR_R + MOTOR_POCKET_CLR
    if MOTOR_POCKET_D > 0:
        inner = inner - along_x(rp, xm - 0.1, INNER_ARM_X[0]) - b.Cone(rp + 0.6, rp - 0.01, 0.6).rotate(b.Axis.Y, 90).moved(b.Location((xm + 0.2, 0, 0)))
    # Nov 13: no lead notch any more (the leads pass 2.7 mm outside the plate's r 29.5 rim; the notch would have thinned the tie's bridge)
    # Nov 13 evening (third try): no 627 seat in the inner half any more; it carries the Ø10 boss onto the frame's 627's inner race and the Ø7 pin in its
    # bore (printed on top, +x up), and the M3 x 20 from the frame's outer face threads into a Ø2.9 pilot along the pin, through the half to the split face
    pin = along_x(PIN_D / 2, FBRG_X0, FBRG_X1 - PIN_TIP_CLR, *PIV)
    pin = b.chamfer([e for e in pin.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - (FBRG_X1 - PIN_TIP_CLR)) < 1e-3], 0.3)
    inner = inner + along_x(FORK_BOSS_D / 2, INNER_W_X[1] - 0.01, FBRG_X0 + 0.01, *PIV) + pin
    inner = inner - along_x(M3_SOFT_PILOT / 2, SPLIT_X - 0.1, FBRG_X1 + 0.1, *PIV)
    head_x = INNER_ARM_X[0] + MOTOR_SCREW_L - REAR_ENGAGE                   # from the motor's rear face (the pocket floor): 4 x M2.5 x 14 in counterbores
    for a in REAR_HOLES_AZ:
        y, z = W.at_angle(REAR_HOLES_R, a)
        inner = inner - along_x(REAR_CLR_D / 2, MOTOR_PLATE_X[0] - 0.1, MOTOR_PLATE_X[1] + 0.1, y, z) - along_x(REAR_CBORE_D / 2, head_x, MOTOR_PLATE_X[1] + 0.1, y, z)
    inner = inner - tenon(TENON_CLR)
    for h in tie_holes()[1]:
        inner = inner - h
    inner = inner - arrow_cut(MOTOR_PLATE_X[1], -1, *ARROW_INNER)         # Nov 13: "up" mark on the motor plate's outer face (prints on top)
    inner = inner - lead_tie_cuts()                                        # Nov 13: the motor leads' zip-tie slots
    return outer, inner, info


BRG6804_X = (X_PLATE_FLOOR, X_PLATE_FLOOR + 7.0)                      # the left 6804 in the cartridge (its inner face toward the motor at [1])
STUB_END = BRG6804_X[1] - RET_SPIGOT_L - RET_STUB_GAP


def build_retainer():
    """Nov 13 (Fernando): the 6804's positive retainer. A spigot centres it in the bearing's bore, its flange bears on the inner race's inner face,
    a boss behind carries the screw's thread; the M3 x 25 from the outer arm clamps the inner race between the flange and the stub's shoulder, so
    the bearing cannot slide in and the fork bears on it. Pressed into the bearing from inside the hub before the cartridge goes on. Prints
    spigot down (+x up): the flange overhangs the spigot by 1.6 mm"""
    x0 = BRG6804_X[1]
    r = along_x(STUB_D / 2 - RET_SPIGOT_CLR, x0 - RET_SPIGOT_L, x0 + 0.01) + along_x(RET_FLANGE_D / 2, x0, x0 + RET_FLANGE_T)         + along_x(RET_BOSS_D / 2, x0 + RET_FLANGE_T - 0.01, x0 + RET_FLANGE_T + RET_BOSS_L)
    try:
        r = b.chamfer([e for e in r.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - (x0 + RET_FLANGE_T + RET_BOSS_L)) < 1e-3], 0.5)
    except Exception:                       # noqa: BLE001
        pass
    # Nov 13 evening (Fernando: snug Ø3.2 where an M3 is a shaft in a printed part): Ø3.2 through the spigot and flange (the screw is their steel core
    # in the 6804's bore), the Ø2.8 thread only in the boss (7.9 mm with the M3 x 30; was Ø2.8 its whole length, 6.2 mm with an M3 x 25)
    return (r - along_x(M3_SNUG / 2, x0 - RET_SPIGOT_L - 0.1, x0 + RET_FLANGE_T)
            - along_x(M3_PILOT / 2, x0 + RET_FLANGE_T - 0.01, x0 + RET_FLANGE_T + RET_BOSS_L + 0.1))


def retainer_screw():
    xs = OA_X[0] + CBORE_DEPTH
    return along_x(3.25, xs - 2.5, xs) + along_x(1.5, xs, xs + RET_SCREW_L)


def hex_x(af, x0, x1):
    """a hexagonal prism along x over x0..x1 on the pivot axis, across flats af, a corner toward +z (the pin's key, the cap's socket and nut recess)"""
    return b.extrude(b.Plane(origin=(x0, PIVOT_Y, PIVOT_Z), x_dir=(0, 0, 1), z_dir=(1, 0, 0)) * b.RegularPolygon(af / math.sqrt(3), 6), amount=x1 - x0)


def build_sleeve():
    """Nov 13 evening: the outer pivot's sleeve, from the keel side: a Ø7 spigot SPIGOT_L into the 627's inner race and a Ø10 collar from the race's
    inner face through the lip's hole onto the keel; Ø3.2 snug on the M3 x 30. Prints on its collar's end. Was: a flange under the head and the body
    through the race, the M4 washer between the race and the keel"""
    x1 = XO + B627_W
    s = along_x(B627_PIN_D / 2, x1 - SPIGOT_L, x1 + 0.01, *PIV) + along_x(COLLAR_D / 2, x1, KEEL_X[0], *PIV)
    return s - along_x(M3_SNUG / 2, x1 - SPIGOT_L - 0.1, KEEL_X[0] + 0.1, *PIV)


def build_pivot_cap():
    """Nov 13 evening (Fernando: no printed washer, a retaining part instead): the outer pivot's retainer cap under the screw head, a Ø10 x 2 flange on
    the 627's inner race (only) and a Ø7 spigot SPIGOT_L into its bore; Ø3.2 snug. Prints flange down"""
    c = along_x(OCAP_D / 2, OUTER_HEAD_X, XO + 0.01, *PIV) + along_x(B627_PIN_D / 2, XO, XO + SPIGOT_L, *PIV)
    return c - along_x(M3_SNUG / 2, OUTER_HEAD_X - 0.1, XO + SPIGOT_L + 0.1, *PIV)


def build_pin_cap():
    """Nov 13 evening (third try): the inner pivot's cap under the M3 x 20 flat head: Ø10 x 2.4 on the frame 627's inner race (only), the head countersunk
    in it 0.5 deeper than the head (cad/solidworks.md 18), Ø3.2 snug below. It turns with the fork. Prints with the countersink up"""
    x_top = FBRG_X1 + ICAP_T
    c = along_x(ICAP_D / 2, FBRG_X1, x_top, *PIV) - along_x(M3_SNUG / 2, FBRG_X1 - 0.1, x_top + 0.1, *PIV)
    c = c - along_x(FLAT_HEAD_D / 2, INNER_HEAD_X, x_top + 0.1, *PIV)
    csk = b.Cone(FLAT_HEAD_D / 2, M3_SNUG / 2, (FLAT_HEAD_D - M3_SNUG) / 2, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN)).rotate(b.Axis.Y, -90)
    return c - csk.moved(b.Location((INNER_HEAD_X + 0.001, PIVOT_Y, PIVOT_Z)))   # Ø6 at the head's top, narrowing toward -x


FLAT_HEAD_D = 6.0                                                          # M3 flat head (hardware.FLAT_HEAD)


def ring_screw_yz():
    return [(PIVOT_Y + r * math.cos(math.radians(a)), PIVOT_Z + r * math.sin(math.radians(a))) for r, a in RING_SCREWS]


def ring_shape(x0, grow=0.0):
    """the clamp ring's outline from x0 (its band RING_T deep, its tabs RING_TAB_T), grown by `grow` (the frame's recess); an upper arc of the annulus
    RING_R about the pivot, the bottom wedge between RING_ARC[1] and RING_ARC[0] + 360 left out"""
    a0, a1 = RING_ARC[0] - grow * 5, RING_ARC[1] + grow * 5
    band = along_x(RING_R[1] + grow, x0, x0 + RING_T + 0.1 * (grow > 0), *PIV) - along_x(RING_R[0] - grow, x0 - 1, x0 + RING_T + 1, *PIV)
    P = lambda a, r=40.0: (PIVOT_Y + r * math.cos(math.radians(a)), PIVOT_Z + r * math.sin(math.radians(a)))   # noqa: E731
    band = band - yz_poly([(PIVOT_Y, PIVOT_Z), P(a1), P(270.0), P(a0 + 360.0)], x0 - 1, x0 + RING_T + 1)
    for y, z in ring_screw_yz():
        band = band + along_x(RING_TAB_R + grow, x0, x0 + RING_TAB_T + 0.1 * (grow > 0), y, z)
    return band


def build_brg_ring():
    """Nov 13 evening (Fernando: compress the frame too): the clamp ring on the frame 627's outer race (its motor-side face), in a recess in the frame's
    fork-side face, flush with it; 2 x M3 x 8 pan heads in its tabs (counterbores) into the frame. Fitted at the bench before the fork's inner half goes
    on; the fork's Ø10 boss passes its Ø19 hole. Prints on its flat face"""
    x0 = FI_X[0]
    r = ring_shape(x0)
    for y, z in ring_screw_yz():
        r = r - along_x(M3_CLR / 2, x0 - 0.1, x0 + RING_TAB_T + 0.1, y, z) - along_x(CBORE_D / 2, x0 - 0.1, x0 + CBORE_DEPTH, y, z)
    return r


def ring_screws():
    """the clamp ring's two M3 x 8 pan heads (records), head in the tab's counterbore, along +x into the frame"""
    import hardware as H
    return [H.pan((FI_X[0] + CBORE_DEPTH, y, z), (1, 0, 0), RING_SCREW_L, f'bolt_ring_{i + 1}') for i, (y, z) in enumerate(ring_screw_yz())]


def inner_pivot_rec():
    import hardware as H
    return dict(kind='flat', size=3.0, L=INNER_PIVOT_SCREW_L, p=(INNER_HEAD_X, PIVOT_Y, PIVOT_Z), d=(-1.0, 0.0, 0.0), name='bolt_pivot_inner')


def outer_screw():
    """M3 x 30 pan head on the outer retainer cap, from outside the arm through the cap and sleeve into the keel"""
    return along_x(3.25, OUTER_HEAD_X - 2.5, OUTER_HEAD_X, *PIV) + along_x(1.5, OUTER_HEAD_X, OUTER_HEAD_X + OUTER_SCREW_L, *PIV)


def bearing(x_face, s):
    """627 at the pivot, from x_face toward s"""
    a, c = sorted((x_face, x_face + s * B627_W))
    return along_x(B627_OD / 2, a, c, *PIV) - along_x(B627_BORE / 2, a - 1, c + 1, *PIV)




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


def block_tangent():
    """(y, z) where the gusset's line from the column's back face (y Y_S + TOWER_HALF_L, z GUSSET_TOP_Z) touches the pivot block's circle, on its back"""
    gy, gz = Y_S + TOWER_HALF_L - PIVOT_Y, GUSSET_TOP_Z - PIVOT_Z
    d = math.hypot(gy, gz)
    th = math.atan2(gz, gy) - math.acos(PIVOT_BLOCK_R / d)
    return (PIVOT_Y + PIVOT_BLOCK_R * math.cos(th), PIVOT_Z + PIVOT_BLOCK_R * math.sin(th))


def build_inner_frame(st):
    import hardware as H
    x0, x1 = FI_X
    top_foot = PLATE_TOP_L + FOOT_T
    (ha, hb), (ra, rb) = st['hook'], st['rail']
    z_bot = min(ha[1], hb[1]) - HOOK_T
    yc0, yc1 = Y_S - TOWER_HALF_L, Y_S + TOWER_HALF_L
    # Nov 13 evening (Fernando: reinforce round the bearing): the side profile runs from the foot's end up round the pivot block (R PIVOT_BLOCK_R about
    # the pivot, its back on the foot's end) and on along its tangent to the column's back face: the gusset, block and foot are one solid
    # (was: the gusset down to the foot top at y 84 and an R9 block round the pivot)
    T = block_tangent()
    rblk, ht = PIVOT_BLOCK_R, math.atan2(T[1] - PIVOT_Z, T[0] - PIVOT_Y) / 2
    am = (PIVOT_Y + rblk * math.cos(ht), PIVOT_Z + rblk * math.sin(ht))
    yb = PIVOT_Y + rblk
    pts = [(yb, top_foot), (yb, PLATE_TOP_L), (FOOT_Y_L[0], PLATE_TOP_L), (FOOT_Y_L[0], top_foot), (yc0, top_foot), (yc0, P.Z_TT), (yc1, P.Z_TT),
           (yc1, GUSSET_TOP_Z), T]
    f = prism([line(V(*p), V(*q)) for p, q in zip(pts[:-1], pts[1:])] + [arc(V(*T), V(*am), V(*pts[0]))], x0, x1)
    for r, at in ((FILLET_COL, [(yc0, top_foot)]), (FILLET_GUSSET, [(yc1, GUSSET_TOP_Z)])):
        f = f.fillet(r, [e for e in f.edges() if abs(e.length - FRAME_T) < 1e-3 and any(abs(e.center().Y - y) < 1e-3 and abs(e.center().Z - z) < 1e-3 for y, z in at)])
    tx = (SPRING_X[0] - (SPRING_HOLE_D + 0.6) / 2 - 3.4, x0)
    f = f + box(tx[0], tx[1], yc0, yc1, P.Z_TBOT, P.Z_TT)                                   # 2 x 2 spring tower over the widened arm
    wy = WASHER_OD / 2 + RAIL_CLEAR
    for ry in ((yc0, Y_PAIR[0] - wy), (Y_PAIR[-1] + wy, yc1)):                              # bump rails in front of / behind the springs
        rail = yz_poly([ra, rb, (rb[0], P.Z_TBOT + 0.5), (ra[0], P.Z_TBOT + 0.5)], *tx)
        f = f + (rail & box(tx[0] - 1, tx[1] + 1, ry[0], ry[1], -50, 100))
    f = f + P.yz_quad(ha, hb, -HOOK_T, x0 - LEDGE_W, x0)                                    # droop hook under the widened arm
    # (the pivot block is in the side profile since Nov 13 evening; round 14: it must not dip under the foot, or the slide-out clip cuts the
    # chassis ring from under it)
    # round 10: round every edge but those on the print face (outer, x1) and the plate face
    f, rounded = round_edges_each(f, [e for e in f.edges() if not on_x(e, (x1,)) and not all(abs(e.position_at(t).Z - PLATE_TOP_L) < 1e-3 for t in (0, 0.5, 1))],
                                  FRAME_EDGE_R)
    for y in FOOT_NUT_Y:                                                                     # Nov 13 evening: the nut pads
        f = f + cyl(NUT_PAD_D / 2, NUT_PAD_H + 0.01, ((x0 + x1) / 2, y, top_foot + NUT_PAD_H / 2 - 0.005))
    for xs in SPRING_X:
        for y in Y_PAIR:
            f = f - cyl((SPRING_HOLE_D + 0.6) / 2, P.Z_CEIL - P.Z_TBOT + 0.1, (xs, y, (P.Z_CEIL + P.Z_TBOT) / 2 - 0.05))
            f = f - cyl(GUIDE_R, GUIDE_LEN + 0.2, (xs, y, P.Z_CEIL + GUIDE_LEN / 2))
    # Nov 13 evening (third try): the inner 627 sits in the frame: a seat from the fork-side face (its floor faces up as the frame prints, on the outer race
    # only past a Ø18 relief), then a Ø12 bore out of the outer face round the cap and the screw head (they turn with the fork)
    f = (f - along_x(B627_SEAT_D / 2, x0 - 0.1, FBRG_X1, *PIV) - along_x(SEAT_RELIEF_D / 2, FBRG_X1 - 0.01, FBRG_X1 + SEAT_RELIEF_T, *PIV)
         - along_x(FRAME_BORE_D / 2, FBRG_X1, x1 + 0.1, *PIV))
    f = f - ring_shape(x0 - 0.1, RING_CLR)                                                   # the clamp ring's recess (band 2.5, tabs 4.0 deep)
    for y, z in ring_screw_yz():                                                             # its two M3 x 8: Ø2.8 pilots on through the frame
        f = f - along_x(M3_PILOT / 2, x0 + RING_TAB_T - 0.01, x1 + 0.1, y, z)
    xm = (x0 + x1) / 2
    for y in FOOT_SCREW_Y:                                                                   # Nov 13 (Fernando: no captive nuts): the M3s
        f = f - cyl(FOOT_PILOT / 2, 20.0 - PLATE_TOP_L, (xm, y, (20.0 + PLATE_TOP_L) / 2 - 0.1))   # from under the chassis thread into a pilot
                                                                                             # through the foot (round 13-14: hex nuts in slots;
                                                                                             # Nov 13 Ø2.9, evening Ø2.8: a short thread)
    for y in FOOT_NUT_Y:                                                                     # Nov 13 evening: optional nuts on the foot's top, in
        f = f - H.hex_recess((xm, y, top_foot + NUT_PAD_H), (0, 0, -1))                     # a pad (so the foot's 8 mm stays under the nut)
    for xs, ys in SUPPORT_PILOTS:                                                            # the side fin's M3 x 8 studs: Ø2.8 (evening; Nov 13
        f = f - cyl(FOOT_PILOT / 2, P.Z_TT - PLATE_TOP_L + 0.4, (xs, ys, (P.Z_TT + PLATE_TOP_L) / 2))   # Ø2.9) through the whole column and foot
                                                                                             # (was Ø2.8 x 12 blind)
    for xs, ys in FRAME_BOLTS:                                                               # Nov 13: the column bolt from below threads into a
        z2 = COLUMN_PILOT_Z                                                                  # Ø2.9 pilot up the column (was a nut in a slot), the
        f = f - cyl(M3_CLR / 2, z2 - PLATE_TOP_L + 0.1, (xs, ys, (z2 + PLATE_TOP_L) / 2 - 0.05))   # hole on through the column top
        f = f - cyl(M3_SOFT_PILOT / 2, P.Z_TT - z2 + 0.3, (xs, ys, (P.Z_TT + z2) / 2 + 0.05))
    return f, dict(x=list(FI_X), edge_round=rounded, z_bottom=round(z_bot, 2), z_top=round(P.Z_TT, 2), droop_hook_top=[[round(v, 2) for v in p] for p in (ha, hb)],
                   bump_rail_underside=[[round(v, 2) for v in p] for p in (ra, rb)])


def pin_x():
    """the fork's pin in the frame 627's bore (x range)"""
    return FBRG_X0, FBRG_X1 - PIN_TIP_CLR


def inner_pivot_screw():
    """Nov 13 evening (third try): M3 x 20 flat head countersunk in the inner cap, reached through the frame's outer face, threaded into the fork's pin: it
    presses the fork's inner half onto the frame 627's inner race (and turns with it)"""
    import hardware as H
    return H.solid(inner_pivot_rec())


def nut_slot(x, y, z, x_open):
    """round 13: pocket for an M3 hex nut on a vertical screw at (x, y), its bearing face (underside) at z, flats across y, open along x to the
    face x = x_open (the nut slides in from there)"""
    hexa = b.extrude(b.Plane.XY.offset(z) * b.RegularPolygon(NUT_SLOT_W / math.sqrt(3), 6), amount=NUT_SLOT_T).moved(b.Location((x, y, 0)))
    xa, xb = sorted((x, x_open + math.copysign(0.1, x_open - x)))
    return hexa + box(xa, xb, y - NUT_SLOT_W / 2, y + NUT_SLOT_W / 2, z, z + NUT_SLOT_T)


def nuts():
    """the pod's hex nut records: none (Nov 13, Fernando: no captive nuts; the inner frame's four screws thread into pilots; the evening's keyed cap and
    nut on the frame's pin went with the third try)"""
    return []


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
    # Nov 13 evening (Fernando: the boss must join the main piece where the mounting screws are, not stand as a cantilevered cylinder: the screw
    # that holds the bearing must be stiff against the keel's mounting screws): behind the pivot the side plate's profile runs on out to the
    # boss's end, so boss, side plate, block and foot (with the two M3 x 35) are one solid. Its underside stays the side plate's (z -7 over the
    # fork bar, which is cut to z -10 there)
    k = k + (prism(side, KEEL_X[1] - 0.01, KEEL_PIN_BOSS_X) & box(KEEL_X[1] - 1, KEEL_PIN_BOSS_X + 1, PIVOT_Y, y1 + 1, PLATE_TOP_L - 1, top + 1))
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
    k = k - along_x(KEEL_PIVOT_PILOT / 2, KEEL_X[0] - 0.1, KEEL_PIN_BOSS_X + 0.1, *PIV)   # M3 x 30 from outside, through the sleeve and the washer
    for x, y in KEEL_SCREWS:                                                                 # round 14: a Ø2.9 pilot through the whole
        k = k - cyl(M3_SOFT_PILOT / 2, top - PLATE_TOP_L + 0.2, (x, y, (top + PLATE_TOP_L) / 2))   # keel (round 13: hex nuts in a slot)
    x_tip = OUTER_HEAD_X + OUTER_SCREW_L
    return k, dict(top=top, edge_round=r, l_cove_r=KEEL_COVE_R, pivot_screw_engagement=round(x_tip - KEEL_X[0], 2), pivot_screw_tip_to_boss_end=round(KEEL_PIN_BOSS_X - x_tip, 2),
                   foot_screw=f'M3 x {KEEL_SCREW_L:g} up from under the chassis into a Ø{M3_SOFT_PILOT} pilot through the whole keel')


def build_static():
    fork_outer, fork_inner, info = build_fork()
    st = stops(fork_inner)
    parts = dict(fork_outer=fork_outer, fork_inner=fork_inner, sleeve=build_sleeve(), retainer=build_retainer(), pivot_cap=build_pivot_cap(),
                 pin_cap=build_pin_cap(), brg_ring=build_brg_ring())
    parts['frame_inner'], finfo = build_inner_frame(st)
    parts['keel'], kinfo = build_keel()
    return parts, dict(inner_frame=finfo, keel=kinfo, fork_rounds=info)


def posed(parts, unit, a):
    moving = dict(unit, pod_fork_outer=parts['fork_outer'], pod_fork_inner=parts['fork_inner'],
                  bearing_627_outer=bearing(XO + B627_W, -1), **tie_bolts(), bolt_pivot_inner=inner_pivot_screw(), pod_pin_cap=parts['pin_cap'],
                  pod_retainer=parts['retainer'], bolt_retainer=retainer_screw(), **tie_and_marks(parts))
    out = {k: (v.rotate(PIVOT_AXIS, a) if a else v) for k, v in moving.items()}
    out['pod_frame_inner'], out['pod_keel_outer'], out['pod_sleeve'] = parts['frame_inner'], parts['keel'], parts['sleeve']
    out['bolt_pivot_outer'], out['pod_pivot_cap'] = outer_screw(), parts['pivot_cap']   # evening: no washers
    out['bearing_627_inner'] = bearing(FBRG_X1, -1)                             # evening (third try): in the frame; its inner race turns with the fork
    out['pod_brg_ring'] = parts['brg_ring']                                     # its clamp ring and the ring's two screws (static)
    import hardware as H
    for r in ring_screws():
        out[r['name']] = H.solid(r)
    lifts = {}
    for k, (xs, y) in PLUNGERS.items():
        lifts[k] = lift = P.plunger_lift(a, y)[0]
        out[f'spring_plunger_{k}'] = P.plunger(xs, y).moved(b.Location((0, 0, lift)))
        out[f'die_spring_{k}'] = P.die_spring(xs, y, P.Z_CEIL - P.Z_SB - lift).moved(b.Location((0, 0, lift)))
    return out, lifts


STATIC = ('pod_frame_inner', 'pod_keel_outer', 'pod_sleeve', 'bolt_pivot_outer', 'pod_pivot_cap', 'bearing_627_inner', 'pod_brg_ring', 'bolt_ring_1',
          'bolt_ring_2')
PRINTED = (('pod_frame_inner', 'frame_inner'), ('pod_keel_outer', 'keel'), ('pod_fork_outer', 'fork_outer'), ('pod_fork_inner', 'fork_inner'),
           ('pod_sleeve', 'sleeve'), ('pod_retainer', 'retainer'), ('pod_pivot_cap', 'pivot_cap'), ('pod_pin_cap', 'pin_cap'), ('pod_brg_ring', 'brg_ring'))
FORK = ('pod_fork_outer', 'pod_fork_inner')
PRINT_UP = {'frame_inner': -1, 'keel': 1, 'fork_outer': 1, 'fork_inner': 1, 'retainer': 1, 'pivot_cap': 1, 'pin_cap': 1, 'brg_ring': 1}   # print direction along x (each part lies on its flat x face)


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
        ov, press = {}, {}
        moving = [k for k in sh if k not in STATIC and not k.startswith(('spring_plunger', 'die_spring'))]
        for st in STATIC:
            for k in moving:
                if near(sh[st], sh[k]):
                    v = overlap(sh[st], sh[k])
                    if v > 0.01:                                         # Nov 13 evening: the 627s' designed press fits (Ø21.95 seats, Ø7.20
                        (press if 'bearing_627' in st + k else ov)[f'{st} / {k}'] = round(v, 2)   # pins and spigots) apart
        for pk in PLUNGERS:
            for k in sh:
                if k.endswith(pk) or not near(sh[f'spring_plunger_{pk}'], sh[k]):
                    continue
                v = overlap(sh[f'spring_plunger_{pk}'], sh[k])
                if v > 0.05:
                    ov[f'spring_plunger_{pk} / {k}'] = round(v, 2)
        r = dict(axle=[round(float(v), 2) for v in rot_pt((0, 0), a)], plunger_lift={k: round(v, 2) for k, v in lifts.items()},
                 interference_mm3=ov, press_fits_627_mm3=press)
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
    rep['nov13'] = nov13_checks(parts, unit)
    sh, _ = posed(parts, unit, 0.0)
    bb = b.Compound([v for k, v in sh.items() if not k.startswith(('spring_plunger', 'die_spring'))]).bounding_box()
    rep['envelope_nominal'] = dict(x=[round(bb.min.X, 1), round(bb.max.X, 1)], y=[round(bb.min.Y, 1), round(bb.max.Y, 1)], z=[round(bb.min.Z, 1), round(bb.max.Z, 1)])
    return rep


def fastener_records():
    """the pod's screws as hardware records (pod frame, nominal), for the hole audits: the pivot screws, the 6804 retainer's screw, the tie
    bolts, and the chassis screws that end in the inner frame and the keel (chassis.py: heads in the members' underside counterbores)"""
    import hardware as H
    import chassis as C
    recs = [H.from_solid(outer_screw(), 'bolt_pivot_outer'), inner_pivot_rec()] + ring_screws() + [
            H.from_solid(retainer_screw(), 'bolt_retainer')] + [H.from_solid(v, k) for k, v in tie_bolts().items()]
    z_head = C.Z0 + C.BELOW_CB - (C.Z1 - PLATE_TOP_L)                     # robot z -> pod z
    xm = sum(FI_X) / 2
    recs += [H.pan((xm, y, z_head), (0, 0, 1), FOOT_SCREW_L, 'bolt_pod_foot') for y in FOOT_SCREW_Y]
    recs += [H.pan((x, y, z_head), (0, 0, 1), FRAME_BOLT_L, 'bolt_pod_column') for x, y in FRAME_BOLTS]
    recs += [H.pan((x, y, z_head), (0, 0, 1), KEEL_SCREW_L, 'bolt_keel') for x, y in KEEL_SCREWS]
    return [r for r in recs if r]


def pivot_clamp_checks(parts):
    """Nov 13 evening: each 627's inner race clamped to its static part by its screw through printed retainer caps (no washers), the fork held axially
    both ways through the bearings"""
    bo, bi = bearing(XO + B627_W, -1), bearing(FBRG_X1, -1)
    oc, ic, sl, so, si = parts['pivot_cap'], parts['pin_cap'], parts['sleeve'], outer_screw(), inner_pivot_screw()
    fr = parts['frame_inner']
    gaps = {}
    for nm, a in POSES.items():
        fo, fi = parts['fork_outer'].rotate(PIVOT_AXIS, a), parts['fork_inner'].rotate(PIVOT_AXIS, a)
        ic_a, si_a = ic.rotate(PIVOT_AXIS, a), si.rotate(PIVOT_AXIS, a)
        gaps[nm] = dict(outer_cap_head_sleeve_to_fork=round(min(min_dist(s, fo) for s in (oc, so, sl)), 2),
                        inner_cap_head_to_frame=round(min(min_dist(s, fr) for s in (ic_a, si_a)), 2),
                        fork_inner_to_frame=round(min_dist(fi, fr), 2),
                        overlap_mm3=round(sum(overlap(s, f) for s in (oc, so, sl) for f in (fo, fi)) + sum(overlap(s, fr) for s in (ic_a, si_a, fi)), 3))
    import hardware as H
    foot65 = H.solid(H.pan((sum(FI_X) / 2, 65.0, PLATE_TOP_L - 1.0), (0, 0, 1), FOOT_T + 1.0, 'probe'))
    return dict(
        outer=dict(seat="opens on the arm's outer face; lip on the keel side", bearing_x=[round(XO, 2), round(XO + B627_W, 2)],
                   lip_to_outer_race=round(min_dist(parts['fork_outer'], bo), 3), lip_overlap_mm3=round(overlap(parts['fork_outer'], bo), 3),
                   cap_to_inner_race=round(min_dist(oc, bo), 3), cap_overlap_bearing_mm3=round(overlap(oc, bo), 3),
                   sleeve_to_inner_race=round(min_dist(sl, bo), 3), sleeve_to_keel=round(min_dist(sl, parts['keel']), 3),
                   sleeve_overlap_keel_mm3=round(overlap(sl, parts['keel']), 3), spigots_gap_in_bore=round(B627_W - 2 * SPIGOT_L, 2),
                   cap_flange=f'Ø{OCAP_D:g} x {OCAP_T:g}', collar_radial_clear_in_lip=round((LIP_HOLE_D - COLLAR_D) / 2, 2),
                   head_proud_of_arm_face=round(OA_X[0] - (OUTER_HEAD_X - 2.5), 2),
                   thread_in_keel=round(OUTER_HEAD_X + OUTER_SCREW_L - KEEL_X[0], 2),
                   holds='fork pulled -x (away from the keel): lip -> outer race -> balls -> inner race -> cap -> screw -> keel'),
        inner=dict(bearing='in the frame', bearing_x=[round(FBRG_X0, 2), round(FBRG_X1, 2)], bearing_bottom_z=round(PIVOT_Z - B627_OD / 2, 2),
                   below_frame_foot_by=round(PLATE_TOP_L - (PIVOT_Z - B627_OD / 2), 2),
                   seat_floor_to_outer_race=round(min_dist(fr, bi), 3), seat_overlap_mm3=round(overlap(fr, bi), 3),
                   fork_boss_to_inner_race=round(min_dist(parts['fork_inner'], bi), 3), fork_pin_overlap_bearing_mm3=round(overlap(parts['fork_inner'], bi), 3),
                   cap_to_inner_race=round(min_dist(ic, bi), 3), cap_overlap_bearing_mm3=round(overlap(ic, bi), 3),
                   screw=f'M3 x {INNER_PIVOT_SCREW_L:g} from the frame\'s outer face into the fork\'s pin',
                   thread_in_fork_mm=round(FBRG_X1 - PIN_TIP_CLR - (INNER_HEAD_X - INNER_PIVOT_SCREW_L), 1),
                   flat_head_top_under_frame_outer_face=round(FI_X[1] - INNER_HEAD_X, 2), bore_radial_clear_to_cap=round((FRAME_BORE_D - ICAP_D) / 2, 2),
                   foot_screw_y65_to_bearing=round(min_dist(foot65, bi), 2),
                   holds='fork pushed +x: boss -> inner race -> balls -> outer race -> frame seat floor; the screw presses the fork onto the inner race'),
        poses=gaps)


def nov13_checks(parts, unit):
    """Nov 13 (Fernando's review): tenon fit, inner pivot pin (no dowel), screws into pilots (no captive nuts), the 6804 retainer, the up arrows,
    the leads' zip tie, and every pod screw's hole through its part"""
    import hardware as H
    out = dict(tenon_clearance_per_face=TENON_CLR)
    xp0, xp1 = pin_x()
    out['inner_pivot'] = dict(pin_d=PIN_D, pin_on='the fork inner half', pin_x=[round(xp0, 2), round(xp1, 2)],
                              screw=f'M3 x {INNER_PIVOT_SCREW_L:g} from the frame\'s outer face', screw_tip_x=round(INNER_HEAD_X - INNER_PIVOT_SCREW_L, 2),
                              pin_wall=round((PIN_D - M3_SOFT_PILOT) / 2, 2))
    out['pivot_clamps (evening)'] = pivot_clamp_checks(parts)
    ret, rs = parts['retainer'], retainer_screw()
    b6804 = unit['bearing_6804_left']
    rotating = {k: v for k, v in unit.items() if k.startswith(('wheel_', 'roller_', 'bolt_', 'ak40_rotor'))}
    out['retainer'] = dict(volume_cm3=round(ret.volume / 1000, 2), solids=len(ret.solids()), valid=bool(ret.is_valid),
                           overlap_6804_mm3=round(overlap(ret, b6804), 3), gap_to_6804_mm=round(min_dist(ret, b6804), 3),
                           stub_shoulder_gap_to_6804=STUB_SHOULDER_GAP, stub_overlap_6804_mm3=round(overlap(parts['fork_outer'], b6804), 3),
                           nearest_turning_part=min((round(min_dist(ret, v), 2), k) for k, v in rotating.items() if near(ret, v, 8)),
                           screw=f'M3 x {RET_SCREW_L:g}', thread_in_retainer_mm=round(OA_X[0] + CBORE_DEPTH + RET_SCREW_L - (BRG6804_X[1] + RET_FLANGE_T), 2),
                           snug_d=M3_SNUG, snug_through='fork stub, retainer spigot + flange, pivot sleeve',
                           screw_tip_to_boss_end=round(BRG6804_X[1] + RET_FLANGE_T + RET_BOSS_L - (OA_X[0] + CBORE_DEPTH + RET_SCREW_L), 2))
    tie = lead_tie()
    ys = {k: tie_vs(parts, unit, a, tie) for k, a in POSES.items()}
    er, et = lead_tie_frame()
    out['lead_tie'] = dict(strap=f'{LEAD_TIE_W} x {LEAD_TIE_T}', x=LEAD_TIE_X, tunnel=list(LEAD_TIE_TUNNEL), tunnel_r=LEAD_TIE_TUNNEL_R,
                           bridge_over_tunnel=round(INNER_PLATE_R - LEAD_TIE_TUNNEL_R - LEAD_TIE_TUNNEL[1] / 2, 2),
                           tie_overlap_fork_inner_mm3=round(overlap(tie, parts['fork_inner']), 3),
                           tie_gap_to_fork_inner=round(min_dist(tie, parts['fork_inner']), 3),
                           poses=ys, lead_centres_yz=[[round(v, 2) for v in c] for c in lead_centres()],
                           tie_outer_radius=round(max(math.hypot(q[1], q[2]) for q in W.points(tie, 0.1)), 2))
    out['up_arrows_engraved'] = dict(  # the arrow's middle is open 0.2 under the face, the face 8 mm below it is solid
        outer=not W._inside_pt(parts['fork_outer'], (OA_X[1] - 0.2, ARROW_OUTER[0], ARROW_OUTER[1] - 1.0)) and W._inside_pt(parts['fork_outer'], (OA_X[1] - 0.2, ARROW_OUTER[0], ARROW_OUTER[1] - 8.0)),
        inner=not W._inside_pt(parts['fork_inner'], (MOTOR_PLATE_X[1] - 0.2, ARROW_INNER[0], ARROW_INNER[1] - 1.0)) and W._inside_pt(parts['fork_inner'], (MOTOR_PLATE_X[1] - 0.2, ARROW_INNER[0], ARROW_INNER[1] - 8.0)),
        size=f'{ARROW_L:g} long, {ARROW_W:g} wide head, {ARROW_DEPTH} deep', faces='outer arm wheel side (y 45); motor plate outer face (axle centre)')
    recs = fastener_records()
    printed = {'pod_frame_inner': parts['frame_inner'], 'pod_keel_outer': parts['keel'], 'pod_fork_outer': parts['fork_outer'],
               'pod_fork_inner': parts['fork_inner'], 'pod_retainer': parts['retainer']}
    bl = H.blind_holes(recs, printed, up_of=lambda n: (1, 0, 0))
    out['blind_holes (want none; Nov 13: every hole through)'] = {k: dict(bottom_past_tip=v['bottom'], solid_to_exit=v['solid']) for k, v in bl.items()}
    au = H.audit(recs, printed, up_of=lambda n: (1.0, 0, 0))
    out['hole_audit'] = dict(thread_mm={k: v for k, v in au['thread'].items()}, partial=au['partial'], across_layers=au['across_layers'],
                             soft_across_layers=au['soft_across_layers'], head_buried=au['head'], no_hole=au['no_hole'])
    out['nuts_in_pod'] = len(nuts())
    # Nov 13 evening: optional foot nuts, the reinforced pivot block, the keel's longer pin boss
    fr, xm, kl = parts['frame_inner'], sum(FI_X) / 2, parts['keel']
    nut = lambda y: b.extrude(b.Plane.XY.offset(PLATE_TOP_L + FOOT_T + NUT_PAD_H - 2.4) * b.RegularPolygon(5.5 / math.sqrt(3), 6), amount=2.4).moved(b.Location((xm, y, 0)))
    x_tip = OUTER_HEAD_X + OUTER_SCREW_L
    out['evening'] = dict(
        foot_pilot_d=FOOT_PILOT, support_pilot_d=FOOT_PILOT, column_bolt_pilot_d=M3_SOFT_PILOT,
        foot_nut_recesses={y: dict(nut_overlap_frame_mm3=round(overlap(nut(y), fr), 3), floor_solid=W._inside_pt(fr, (xm + 2.0, y, PLATE_TOP_L + FOOT_T - 0.1)),
                                   pad=f'Ø{NUT_PAD_D} x {NUT_PAD_H}', plastic_under_recess_mm=round(FOOT_T, 2),
                                   pad_wall_flats=round((NUT_PAD_D - 5.5) / 2, 2), pad_wall_corners=round(NUT_PAD_D / 2 - 5.5 / math.sqrt(3), 2),
                                   pad_inside_foot_front_by=round(y - NUT_PAD_D / 2 - FOOT_Y_L[0], 2))
                           for y in FOOT_NUT_Y},
        pivot_block=dict(r=round(PIVOT_BLOCK_R, 2), tangent_yz=[round(v, 2) for v in block_tangent()],
                         solid_above_pin_mm=round(PIVOT_BLOCK_R - M3_PILOT / 2, 2),
                         solid_at=[W._inside_pt(fr, (xm, PIVOT_Y + dy, PIVOT_Z + dz)) for dy, dz in ((0, 12.5), (9, 9), (12.8, 2), (-6, 12), (6, 4))]),
        keel_pin_boss=dict(length=round(KEEL_PIN_BOSS_X - KEEL_X[1], 2), x=[round(KEEL_X[1], 2), round(KEEL_PIN_BOSS_X, 2)],
                           screw=f'M3 x {OUTER_SCREW_L:g}', pilot_d=KEEL_PIVOT_PILOT, thread_in_keel=round(x_tip - KEEL_X[0], 2),
                           tip_to_boss_end=round(KEEL_PIN_BOSS_X - x_tip, 2), fork_bar_cut_to_x=round(BASE_RAISE_X, 2)))
    return out


def prism_vol(x):
    return 0.0 if x is None else float(x.volume)


def tie_vs(parts, unit, a, tie):
    """the zip tie (moves with the fork) at pose angle a: nearest turning part, nearest static pod part, overlap with the leads"""
    t = tie.rotate(PIVOT_AXIS, a) if a else tie
    rot = {k: (v.rotate(PIVOT_AXIS, a) if a else v) for k, v in unit.items() if k.startswith(('wheel_', 'roller_', 'bolt_', 'bearing_6804', 'ak40_rotor'))}
    st = {'pod_frame_inner': parts['frame_inner'], 'pod_keel_outer': parts['keel']}
    leads = {k: (unit[k].rotate(PIVOT_AXIS, a) if a else unit[k]) for k in ('lead_pos', 'lead_neg', 'xt30_housing')}
    near_rot = [(min_dist(t, v), k) for k, v in rot.items() if near(t, v, 15)]
    return dict(nearest_turning=(round(min(near_rot)[0], 2), min(near_rot)[1]) if near_rot else 'none within 15 mm',
                nearest_static=min((round(min_dist(t, v), 2), k) for k, v in st.items()),
                overlap_leads_mm3=round(sum(overlap(t, v) for v in leads.values()), 3), gap_to_leads=round(min(min_dist(t, v) for v in leads.values()), 3))


# ---- export / render -----------------------------------------------------------------------------------------------------------------
STEP_SKIP = ()                     # Nov 13 evening (Fernando, cad/solidworks.md 15): the assembly STEP shows every bearing and fastener, magenta
                                   # (was: bolts, bearings, plungers, washer and the lead tie left out). parts/ stays printed parts only


def export(parts, unit):
    import hardware as H
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    for n, p in PRINTED:
        write_part(parts[p], colour(n), OUT / 'parts' / (n + '.step'), n)
    sh, _ = posed(parts, unit, 0.0)
    items = {k: (H.step_model(k, v), H.MAGENTA if k.startswith(H.STEP_HARDWARE) else colour(k)) for k, v in sh.items() if not k.startswith(STEP_SKIP)}
    for i, r in enumerate(fastener_records()):                            # the pod's mounting screws, from under the chassis, as installed
        if r['name'].startswith(('bolt_pod_', 'bolt_keel')):
            items[f"bolt_mount_{i}_{r['name']}"] = (H.part_solid(r).moved(b.Location(b.Plane(origin=tuple(float(v) for v in r['p']),
                                                    x_dir=tuple(H.pose(r)[:3, 0]), z_dir=tuple(H.pose(r)[:3, 2])))), H.MAGENTA)
    write_step(items, OUT / 'pod_lean_assembly.step', 'pod_lean')
    return {k: len(v[0].solids()) for k, v in items.items() if k.startswith(H.STEP_HARDWARE) or k.startswith('bolt_mount')}


def iso(items, out, az_deg, el, title, colour_fn=colour, tol=0.4, overlay=None):
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
    if overlay is not None:                                              # v13: lines drawn over the picture (ax, the view rotation)
        overlay(ax, Rm)
    fig.savefig(out, dpi=95)
    plt.close(fig)


def render(parts, unit):
    for f in VAL.glob('*.png'):
        f.unlink()
    sh, _ = posed(parts, unit, 0.0)
    keep = {k: v for k, v in sh.items() if not k.startswith(('bolt_', 'roller_core', 'roller_sleeve', 'bearing_687'))}
    keep['bolt_pivot_outer'] = sh['bolt_pivot_outer']
    iso(keep, VAL / 'iso_motor_side.png', 235, 22, 'lean pod, motor (robot-centre) side, nominal')
    iso(keep, VAL / 'iso_outer_side.png', 55, 22, 'lean pod, outer (perimeter) side, nominal')
    iso({k: sh[k] for k in sh if k.startswith(('pod_', 'bearing_627', 'spring_plunger', 'die_spring', 'bolt_pivot', 'bolt_tie', 'washer'))},
        VAL / 'iso_static_fork.png', 215, 30, 'inner frame, outer keel, fork halves, 627s, plungers, tie bolts (wheel hidden)')
    iso({k: sh[k] for k in sh if k.startswith(('pod_', 'bearing_627', 'bolt_pivot', 'bolt_tie', 'washer'))},
        VAL / 'iso_from_below.png', 200, -35, 'from below: fork base bar under the pivot, outer keel inside the arm')
    # round 9: the fork halves pulled apart along x, with the tie bolts, and the keel alone
    apart = {'pod_fork_outer': parts['fork_outer'].moved(b.Location((-40, 0, 0))), 'pod_fork_inner': parts['fork_inner'].moved(b.Location((40, 0, 0)))}
    for k, v in tie_bolts().items():
        apart[k] = v.moved(b.Location((-90 if '_outer_' in k else 90, 0, 0)))
    iso(apart, VAL / 'iso_fork_halves.png', 215, 25, 'fork halves pulled apart (outer: arm + stub + bar + tenon; inner: motor plate, pads, socket), tie bolts out')
    iso(apart, VAL / 'iso_fork_halves_back.png', 35, 25, 'fork halves from behind: tie-bolt counterbores, tenon / socket', tol=0.2)
    iso({'pod_keel_outer': parts['keel'], 'pod_pivot_cap': parts['pivot_cap'], 'bolt_pivot_outer': outer_screw(), 'pod_sleeve': parts['sleeve']},
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
    # Nov 13: the motor leads' zip tie (inner half, leads, plug, tie) and the 6804 retainer / inner pivot pin in the axis plane
    iso({k: sh[k] for k in ('pod_fork_inner', 'lead_pos', 'lead_neg', 'xt30_housing', 'tie_lead', 'ak40_stator', 'wheel_hub') if k in sh},
        VAL / 'iso_lead_tie.png', 330, 35, 'Nov 13: the motor leads tied to the inner fork half (tie through the tunnel under the rim, round both leads)', tol=0.15)
    fig, axs = plt.subplots(1, 2, figsize=(18, 7))
    for ax, (zc, yc, xl, yl, title) in zip(axs, ((0.0, None, (-58, -18), (-22, 22), 'axis plane z 0: 6804 retainer, stub, cartridge, hub'),
                                                 (PIVOT_Z, PIVOT_Y, (50, 82), (PIVOT_Y - 16, PIVOT_Y + 16), 'pivot plane z 0 near y 80.6: inner pin (printed) + M3 x 16'))):
        sl = box(-120, 120, -50, 150, zc - 0.2, zc + 0.2)
        for k, v in sh.items():
            if k.startswith(('roller_', 'bearing_687', 'spring', 'die_')):
                continue
            x = v & sl
            if x is None or not x.faces():
                continue
            vv, t = x.tessellate(0.05, 0.2)
            Pv = np.array([[q.X, q.Y] for q in vv])
            ax.add_collection(PolyCollection([Pv[tri] for tri in np.array(t)], facecolors=[colour(k) if not k.startswith('bolt') else (0.85, 0, 0.85)], edgecolors='none'))
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'section_retainer_pivot.png', dpi=90)
    plt.close(fig)


def main(stage):
    for d in (OUT, VAL):
        d.mkdir(parents=True, exist_ok=True)
    parts, info = build_static()
    for k in ('fork_outer', 'fork_inner', 'sleeve', 'frame_inner', 'keel', 'retainer', 'pivot_cap', 'pin_cap', 'brg_ring'):
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
