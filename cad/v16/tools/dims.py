"""Dimensions of the cad/v14 robot's wheel + motor, suspension pod and wiring harness, mm.

Pod frame: x = wheel axis (motor output flange toward -x, cup stub toward +x); y, z radial. Roller frames: local z = roller axis (tangent),
local y = radial outward, local x = pod x. Everything is nominal CAD; nothing has been printed.
"""
import math

# ---- wheel envelope (same as the v10 / v9 wheel and the sim baseline: Ø140, roller axes at r 52) ---------------------------
R_WHEEL = 70.0                     # wheel outer radius; the roller tread profile is this circle about the wheel axis
R_AXIS = 52.0                      # roller axis radius
D_ROW = 17.25                      # row planes at x = -D_ROW (row 1) and +D_ROW (row 2); rows are 30 deg apart in azimuth
N_PER_ROW = 6

# ---- rollers (v14 baseline: the bearing roller, cad/v13 wheel_quiet round 5 + wheel_quiet_2's tread tip relief) ---------------------------
# Each roller: a PLA core on two 687 ball bearings (7 x 14 x 5, Fernando's kit) round a stationary printed sleeve on the M3 x 50 axle, and a TPU
# tread. The sleeve is clamped between the hub's two thrust faces, so the roller has no play on its axle beyond the 687s' own.
HT = 19.5                          # half length of core and tread (2.35 deg overlap between neighbouring rollers)
TREAD_T = 2.5                      # TPU tread thickness at the ends (5.3 in the middle)
FILLET_TREAD = 2.0                 # TPU end-edge fillet
KEY_W, KEY_DEPTH = 2.4, 1.5        # three tread keys in core keyways open at both ends (the tread slides off): width, depth into the core
CORE_R = 12.7                      # straight core / tread bore
CORE_CHAMFER = 0.5                 # lead-in on the core ends and the tread bore ends
DETENT_DEPTH, DETENT_RIB = 0.5, 0.4   # the tread snaps into place: 45 deg V groove round the core's middle, a matching V rib 0.4 high inside the
                                   # tread bore (0.1 clear at its tip). Fitting stretches the bore 0.4 mm in radius (3 %), by hand
TREAD_RELIEF_LEN = 7.0             # tread tip relief (Fernando, Nov 12): the last 7 mm of each tread end leave the R_WHEEL circle on a tangent arc ...
TREAD_RELIEF_DEPTH = 0.2           # ... 0.2 under it by the end face (0.14 at the upright row handover), so the incoming roller takes up the load over a
                                   # few mm of travel, not in a step (sim: wheel_wobble_2026-11-12.md). Near TPU print precision: VERIFY on a print
END_GAP = 0.9                      # core end to the block thrust face (where v13's 0.75 journal + 0.3 thrust gap were). Nov 13 evening: 0.9
                                   # (was 1.05), 0.3 more of the M3 x 50 in the pilot; the tread stays 0.8 off the hub
# 687 roller cartridge: core + two 687 + a stepped printed sleeve (Ø7 ends through the bearings, Ø8.6 middle on the inner races)
BRG_D, BRG_DO, BRG_B = 7.0, 14.0, 5.0      # 687 bore, OD, width
BRG_IR_LAND = 8.6                  # sleeve shoulder Ø: on the inner race only (687 inner-race land ~Ø8.8-9.1: VERIFY)
BRG_POCKET_FIT = 0.025             # core pocket radius over the bearing OD: Ø14.05, coupon_687 cell 2 (Fernando printed it, Nov 13: the best OD fit;
                                   # was 0.05 = cell 1). A friction fit; it holds the bearing (Fernando: no snap features needed)
SLEEVE_BRG_D = 7.25                # sleeve Ø through the 687s: coupon_687 cell 4 (Nov 13: the best ID fit; was 7.0); Nov 18 (Fernando: the printed
                                   # rollers' sleeves are too loose in the 687s) 0.1 over it (was 7.15). Outside the bearings the sleeve
SLEEVE_STEP_L = 0.15               # steps back to BRG_D (7.0) for the hub's click seats over a 45 deg cone this long, just outside each bearing face
LAND_GAP = 0.25                    # Nov 13 (Fernando: a thrust washer / journal to keep the roller centred): a land on each hub thrust face, Ø LAND_ID ..
                                   # (evening: 0.25, was 0.2: the land face 0.05 outside the core's end plane, which the core passes in the radial drop-in)
LAND_ID = BRG_D + 0.3              # .. BRG_IR_LAND (= SEAT_CHANNEL_W, so the opening is a clean keyhole) round the click seat, standing to LAND_GAP from the 687's face: it meets the static inner race
                                   # only (the outer race and the core stay 1.15 / 1.05 off the face), so a bearing or the roller cannot walk more than
                                   # LAND_GAP out; cut open on the drop-in side by the channel (SEAT_CHANNEL_W), so the cartridge still presses in radially
# Nov 13 evening (Fernando: hard to get both bearings flush or a little inside). The core prints upright, B end (+z) up: pocket B's floor faces up
# and prints true, pocket A's floor is a downward ring printed over air (it sags, so A, which used to stop on it, sat high), and B was placed by A
# plus the sleeve's shoulder with only 0.1 of margin. Now B stops on its (true) floor and A, pressed in last, on the sleeve's shoulder; both
# 0.2 inside the core ends (0.3 was tried: the thrust lands, 0.2-0.25 off the bearings, then stood inside the core's end plane and the core hit
# them as the cartridge drops in radially)
B_RECESS = 0.2                     # bearing B's outer face this far below the core end: B sits on pocket B's floor (was 0.1, on the sleeve's shoulder)
A_RECESS = 0.2                     # bearing A's outer face this far below the core end: A stops on the sleeve's shoulder (was 0.2, on pocket A's floor)
POCKET_A_EXTRA = 0.5               # pocket A is this much deeper than A's place: A never reaches its sagging floor
CORE_BORE_D = BRG_IR_LAND + 2.0    # core between the pockets: 1.0 radial clearance round the sleeve's middle
SLEEVE_ID = 3.2                    # on the M3 (prints ~3.05-3.1: snug; drill 3.2 if it binds)
SLEEVE_END_GAP = 0.0               # each sleeve end on its seat floor. Nov 13 evening (Fernando: the M3 x 50 barely reached its thread): was -0.1
                                   # (0.1 past each floor, the spokes sprung 0.2 apart to pinch it); now 0.1 shorter at each end, the M3 clamps it.
                                   # The cartridge still clicks: the neck is SEAT_STEP shallower than the round
SLEEVE_LEAD = 0.2                  # 45 deg lead-in on the sleeve ends' outer edges, to start the press-in
SEAT_SLOT_D = 0.5                  # each thrust face has a click seat for the sleeve end, this deep: a Ø SEAT_SLOT_W round about the axle ...
SEAT_SLOT_W = BRG_D + 0.1          # ... (0.05 a side), which closes to SEAT_NECK_W just past the sleeve's centre, then opens at 45 deg into a channel
SEAT_NECK_W = BRG_D - 0.1          # SEAT_CHANNEL_W wide out to the block's edge (the drop-in path). Neck and channel are SEAT_STEP shallower than the
SEAT_STEP = 0.1                    # round, so the sleeve rides in pinched SEAT_STEP a side and clicks down into the round. The seat keeps 6.9
SEAT_CHANNEL_W = BRG_D + 0.3       # of M3 x 50 thread in the pilot and HEAD_WALL (1.3) under the head seat (Nov 13 evening; was 6.0 / 1.9)
HUB_EDGE_R = 0.8                   # round on the spoke webs' x-end edges (each spoke filleted before the union) and the drum's left end

# ---- hub ----------------------------------------------------------------------------------------------------------------------
MOTOR_SHIFT_X = 27.5              # option 1 (Nov 2): motor moved outboard (+x) so its XT30 plug and leads sit outside the hub with >= 4 mm to anything rotating
X_LEFT, X_RIGHT = -39.5, 29.0      # hub end faces; the right end is the drum only: no right bearing, the hub rides on the motor output bearing + the left 6804
DRUM_IN, DRUM_OUT = 30.0, 33.0     # drum bore and outside radius (the front of the motor can lives in the bore)
WALL_BACK = -26.5                  # left end wall (r ACCESS_R .. DRUM_IN) ends here at the access bore and runs out on a 45 deg cone to the drum
FLANGE_BACK = -23.0 + MOTOR_SHIFT_X        # flange plate clamped to the motor output face by three M2.5 screws
FLANGE_FRONT = FLANGE_BACK - 3.5           # a 45 deg cone joins its front to the drum (printable cavity roof)
MOTOR_ROT = 74.0                   # motor turned about the axis from its STEP pose. v15 Nov 14 (Fernando): the XT30 faces UP (plug centroid az ~90,
                                   # was 143: az ~159, back and slightly up) so its leads route straight up through the deck. The rear holes, the
                                   # hub's flange holes, the motor plate's screws and the leads' zip-tie tunnel all follow MOTOR_ROT
FLANGE_R = 13.5
FLANGE_ANGLES = tuple(a + MOTOR_ROT for a in (30, 150, 270))   # follow the motor's M2.5 output holes
CART_ANGLES = (30, 150, 270)       # cartridge screws
M25_CLEAR = 2.7
PLATE_T = 3.0                      # cartridge plate, flush with the left end face
X_PLATE_FLOOR = X_LEFT + PLATE_T
ACCESS_R = 18.0                    # left access bore (cartridge boss, driver access to the flange screws)
CART_R = 26.0                      # cartridge screws (M3 x 12 flat head into Ø2.8 pilots)
R_BRG, BRG_FIT = 16.0, 0.05        # 6804 outside radius and seat allowance
# v15 Nov 15 (Fernando: the 6804 could slide in and out; positive retention, the wheel's axis must not move): the cartridge is two parts. The CUP
# (wheel_cartridge) is a flange in the hub's counterbore, the bearing seat and a lip on the outer race's motor-side face; the CAP
# (wheel_cartridge_cap, the old flush plate) covers it and holds the outer race's other face. The cartridge's 3 flat heads clamp cap + cup flange to
# the hub, so the outer race is clamped between the cap and the cup's lip (the inner race already is: stub shoulder + retainer)
CUP_FLANGE_T = 2.0                 # the cup's flange under the cap: the hub's counterbore is this much deeper (its end wall 10 -> 8 mm)
X_CUP_FLOOR = X_PLATE_FLOOR + CUP_FLANGE_T   # the hub's counterbore floor
CUP_LIP_T, CUP_LIP_R = 1.5, 14.5   # the cup's lip on the outer race's motor-side face: thickness, inner radius (6804 outer ring land ~Ø28.5,
                                   # the shields inside it: VERIFY on the bearing). Clear of the static retainer's flange (r 11.5) by 3.0
CSK_EXTRA = 0.5                    # Nov 13 (Fernando, a general rule): every flat-head countersink 0.5 deeper (a straight bore of the head's top Ø over
                                   # the cone), so the printed head ends flush
# Nov 13 (Fernando): a positive retainer on the 6804's inner race, so the bearing cannot slide in along the stub and the fork's stub shoulder clamps
# against it. A printed plug (pod_lean.build_retainer: a spigot in the bearing's bore, a flange on the inner race's inner face, a boss) and one M3 from
# the outer arm's outer face along the axle through the stub into the plug's pilot
RET_SPIGOT_L, RET_SPIGOT_CLR = 1.3, 0.05   # spigot into the 6804 bore (Ø20): this long, this much under the bore radius (it only centres the plug)
RET_FLANGE_D, RET_FLANGE_T = 23.0, 2.0     # flange on the inner race only (6804 inner ring ~Ø23.6 at its face: VERIFY on the bearing)
RET_BOSS_D, RET_BOSS_L = 10.0, 9.4         # boss behind the flange for the screw's thread (inside the hub's Ø36 access bore, clear of the turning wall).
                                           # Nov 13 evening: 9.4 (was 4.5): the screw threads only into the boss now (spigot and flange Ø3.2 snug on it),
                                           # 7.9 mm of thread, tip 1.5 inside the boss's end
RET_STUB_GAP = 0.2                         # stub end to the spigot (so the screw clamps the inner race, not stub against plug)
RET_SCREW_L = 30.0                         # M3 x 30 pan head from the outer arm's outer face (Nov 13 evening; was x 25): snug Ø3.2 through the fork's stub
                                           # and the retainer's spigot and flange (the screw is their steel core), Ø2.8 thread in the boss

ZF = HT + END_GAP                  # block thrust faces (+-ZF about the roller centre); the flat cut runs across the whole roller end
BAND_C = 0.95                      # radial clearance around the tread
BLEND_R = 3.0                      # blend between the thrust face and the tread clearance: the fillet at the thin spoke neck
BLOCK_R = 6.0                      # axle block radius (v13 8; Fernando, Nov 11: smaller round petals). Seen along its axle each block is this round
                                   # about the axle with 45 deg tangent flanks down to both x faces (prints without support), R_FLANK_FACE into them
R_FLANK_FACE = 2.5
WEB_OUT = 12.0                     # spoke web reaches this far outboard of its row plane (x; row 2 stops at the drum end X_RIGHT)
WEB_GAP = 6.0                      # inboard the webs run to x = +-WEB_GAP / 2 (the other row's rollers reach x +-1.7 with clearance)
                                   # round 6: the roller cut is the roller's revolved envelope plus its radial drop-in channel, not a
                                   # prism through x, so the web outside the roller end stays and wraps it (v10 had 24 mm deep spokes)
WEB_REACH = 12.0                   # web foot half width on the drum, either side of the block centre (local z)
CHANNEL_CHAMFER = 11.0             # v13 Nov 10 (Fernando: supports deep in the channel between the spoke rows were hard to pull out): a 45 deg
                                   # collar round the drum, from r DRUM_OUT at x WEB_GAP/2 - this to r DRUM_OUT + this at x WEB_GAP/2 (row 2's inboard
                                   # web faces). Printed cartridge end down, row 2's web roots then grow out of the drum instead of hanging over the
                                   # channel. Its cone stays (D_ROW + R_AXIS - DRUM_OUT + WEB_GAP/2 - this) / sqrt 2 - (R_WHEEL + BAND_C - R_AXIS)
                                   # = 1.0 mm off row 1's roller clearance envelope (2 mm off the tread); 0 = no collar (v11/v12 hub)

HEAD_WALL = 1.3                    # Nov 13 evening: plastic between the head seat and the click seat (was 1.9): the head presses it straight onto the
                                   # sleeve end (Ø7 ring under the Ø6.5 head), so it only takes compression
SEAT = ZF + SEAT_SLOT_D + HEAD_WALL   # axle head underside (local z); the shank runs 50 mm toward -z
ENGAGE = 50.0 - ZF - SEAT          # M3 x 50 reach past the pilot-side thrust face (7.4; its thread starts at the seat floor, SEAT_SLOT_D in: 6.9,
                                   # was 6.0)
HEAD_POCKET_D, HEAD_DEPTH = 7.0, 3.0   # round 8: round counterbore for the M3 pan head (Ø6.5 x 2.5), Ø3.4 shank hole to the thrust face,
FLARE_L = 5.0                      # then a 45 deg cone opening outward for FLARE_L (to r 8.5, past the block), then Ø7 on for the head and driver.
                                   # Longer cuts into the neighbouring roller's head seat (the two axles in a spoke cross 4 mm past their heads):
                                   # 5: 99 % of its seat left, 7: 61 %, 9: 30 %
AXLE_HEAD_D = 6.4                  # axle head envelope for the insertion graze check
B_OUT = 25.95                      # head-side block outer face (Nov 13 evening: kept where it was, the counterbore 3.75 deep, so the head path past it,
                                   # which crosses the neighbouring roller's block in the spoke, cuts as before; SEAT + HEAD_DEPTH moved it 0.75 in and
                                   # cut two thrust lands loose)
HEAD_DEPTH = B_OUT - SEAT
PILOT_D = 2.7                      # roller axle thread pilot (v15 Nov 14, Fernando: the hub prints the blocks sideways, so the pilots come out
                                   # a little large: 2.7. Nov 11: 2.8 as every M3 pilot; v13 2.9)
PILOT_DEPTH = ENGAGE + 0.8
PILOT_OUT = 37.7                   # Nov 17 (Fernando: through, so the support can be pushed out): the pilot's far end (local z). The pilot ended
                                   # at z 31.7 inside the neighbouring roller's block in the pilot-side spoke (plastic on the axis to z 36.2); 37.7
                                   # clears the oblique exit fully (hub volume = 38-42's). 38-42 leave OCC's hub invalid, 37.3 a 0.2 mm3 lip per hole.
                                   # The closest pass to that roller's thread stays 2.01 mm between axes at z 31.7 (unchanged)
A_OUT = 29.05                      # pilot-side block outer face (kept where it was: the tip now ends 1.25 inside it, as before; the pilot is through)
CLR_D = 3.4                        # M3 shank hole from the thrust face to the head seat

# ---- motor and harness (the pod's fork bolts straight to the motor's rear face; no motor mount) -------------------------------
# components/ak40-10/AK40-10 with connector.STEP is the round-2 pod-frame motor, moved; this puts it back, then MOTOR_SHIFT_X outboard,
# then turns it MOTOR_ROT about the axis
MOTOR_STEP_SHIFT = (34.462 + MOTOR_SHIFT_X, 41.0002, -49.0161)
MOTOR_R = 26.5                     # can radius
MOTOR_X = (-23.0 + MOTOR_SHIFT_X, 17.2 + MOTOR_SHIFT_X)   # output face, rear face
REAR_HOLES_AZ = tuple(a + MOTOR_ROT for a in (-120, -30, 60, 150))   # motor rear face: 4 holes x 4.3 deep at r 23.5. The maker's STEP draws them Ø2.8 (read as M3
                                   # until Nov 10); Fernando, with the motor in hand, screws M2.5 into them: tapped M2.5 here, drawn at the thread's minor Ø like the output holes
REAR_HOLES_R, REAR_HOLE_DEPTH, REAR_HOLE_D = 23.5, 4.3, 2.05
LEAD_D = 3.0                       # ASSUMED power lead OD (VERIFY)
LEAD_STRAIGHT = 0.5                # straight lead beyond the pin ends before the bend
LEAD_BEND_R = LEAD_D               # sharp 90 deg bend (centreline radius = 1 x OD) toward +x
LEAD_RUN_TO = MOTOR_X[1] + 12.0    # leads modelled to here (x) after the bend, inside the plate cutout (route on to the robot not drawn)
HARNESS_CLEAR = 4.0                # wanted clearance all round the plug and leads

# ---- robot wiring harness (harness.py, harness_route.py; Nov 11, Fernando: slots behind the fins, zip ties straight through the PLA) ---------
# Floor-2 cable slots: a lead that crosses the lidar's scan plane (z 163) where the lidar sees it is a blind angle. The spine and front fins run
# to the rim, so the only open shadow is past each side fin's tail (u 238 from the lidar's axis, along the fin's ray at bearing +-136.7): one
# stadium slot each, centred on the ray. The side fin's shadow is 3.7 deg wide (web 5 at u 78.1), half width 8.0 mm at u 245, so a slot of
# half width SLOT_R 6.5 stays inside it with 1.5 mm to spare and a bundle up to Ø8 in it is hidden (bundles lie one behind another along the ray)
HARNESS_SLOT_U = (245.0, 272.0)    # slot centreline from / to (u along the fin's ray); its round ends reach u 238.5 (the fin's last floor screw at
                                   # u 232: 3.2 mm of floor between its head and the slot) and 278.5 (r 208.1; the rim lip starts at r 212)
HARNESS_SLOT_R = 6.5               # slot half width (the shadow's half width at u 238.5 is 7.7)
# zip-tie anchors: two slots through a printed plate either side of the cable; the tie goes down one, under the plate's bridge, up the other
# and round the cable (no clip, no insert). Sized for 3.6 mm wide x 1.2 mm ties (VERIFY on the ties bought); fits bundles up to ~Ø10
TIE_SLOT_L, TIE_SLOT_W = 4.6, 2.0  # each slot: along the cable, across it
TIE_PITCH = 13.0                   # slot centre to centre across the cable (an 11 mm PLA bridge under the cable)
TIE_KEEP = 2.0                     # each tie pair's footprint this far from any hole, boss, lip, rib, flange or device underneath / on top
# Nov 11 (Fernando): besides the tie pairs, M3 pilots (Ø2.8, the floor's boss under each: upper.BOSS_*) on a square grid, to attach things later.
# The grid is the tie grid's (30 mm, on multiples of 30) shifted half a pitch, so each hole sits in the middle of four tie points
GRID_PITCH, GRID_OFFSET = 30.0, 15.0
GRID_KEEP = 2.0                    # each hole's boss (Ø upper.BOSS_D) this far from anything under the floor (lip, rib, flange, boss, lead) or any hole
                                   # or edge in the plate; nothing on top within GRID_TOP_R of the hole, GRID_TOP_H above the floor
GRID_TOP_R, GRID_TOP_H = 5.0, 8.0
# the three potted DC-DC modules (Fernando's picks, Nov 11; harness.CONVERTER_PARTS): body x, y, z as drawn (leads extra). ASSUMED (VERIFY):
# the 19 V size is a comparable 24-19 V 10 A unit's, the 12 V and 5 V the common potted 8 A / 5 A class
CONV_19V = (74.0, 74.0, 32.0)      # B09KY6H4FZ 24 -> 19 V 10 A buck (22-40 V in: bench-test its dropout at 20-22 V before trusting it)
CONV_12V = (64.0, 60.0, 22.0)      # B0F3HQ6ND8 15-90 -> 12 V 8 A buck (the arm)
CONV_5V = (60.0, 66.0, 22.0)       # B0CM37DX4Q 8-40 -> 5 V 5 A buck


def rt(z):
    """tread outer radius about the roller axis at local z"""
    return math.sqrt(R_WHEEL ** 2 - z * z) - R_AXIS


def rc(z):
    """core outer radius"""
    return rt(z) - TREAD_T


# ---- suspension pod (pod.py) ------------------------------------------------------------------------------------------------
# Pod frame = wheel frame at nominal ride height (axle at the origin, floor at z = -R_WHEEL). The robot plate (designed later) is flat with its top
# at PLATE_TOP_Z, as v10's deck (top 3 mm above the axle): the hinge sits at plate height. Two side frames stand on the plate beside a cutout, each
# with a keel down through a notch to the shaft, and are held by vertical M3 screws from the top; the shaft ties them together.
PLATE_TOP_Z = 3.0
PIVOT_Y, PIVOT_Z = 80.6, 0.0       # hinge level with the axle, 80.6 mm arm (design.md 2, sim POD_PIVOT): no torque jacking
DROOP, BUMP = 3.0, 15.0            # axle travel below / above nominal (sim pod_droop, pod_travel - droop)
# pivot (round 7): no shaft. Each 627 (owned) presses into a seat in a fork arm's outer face. Each frame's keel carries a printed Ø7 pin in the
# 627's inner race on a Ø10 boss that bears on the inner race (spacer across the 1 mm frame gap), and one M3 x 16 pan head from the keel's
# outer face threads through the keel, boss and pin: a steel core for the printed pin, held in the static frame. The bosses locate the fork
# axially (0.1 mm each side).
B627_OD, B627_W, B627_BORE = 22.0, 7.0, 7.0
BRG_RECESS = 0.1                   # 627 face under the fork arm's outer face
BRG_PATH_CLEAR = 0.5               # Nov 9 (Fernando): the outer 627 slides into its seat along its axis from the arm's inner face, so the bar is cut away
                                   # around its path, a trough this much larger than the bearing's radius (the seat itself stays the press fit)
BRG_PATH_LEN = 10.0                # Nov 9 (Fernando): ... and only this far (along x) past the bearing's exposed face; the full-length trough was too deep
SEAT_RELIEF_D, SEAT_RELIEF_T = 18.0, 1.0   # beyond each seat, so the seat bottom bears on the outer race only
PIVOT_BOSS_D = 10.0                # frame boss onto the inner race only (627 inner ring about Ø10.5-11)
PIVOT_HEAD_D, PIVOT_HEAD_RECESS = 7.0, 2.6   # counterbore for the M3 pan head in the keel's outer face (flush: the keel sits in the plate notch)
PIVOT_SCREW_L = 16.0               # M3 pan head: all of it threaded into the keel (9.4), boss (1) and pin (5.6 of 6.9)
LOBE_R = 15.0                      # fork pivot lobe radius
M3_PILOT, M3_CLR = 2.8, 3.4
M3_SNUG = 3.2                      # Nov 13 (Fernando, cad/solidworks.md 18): an M3 snug in a printed part it does not thread into (the screw as a shaft or
                                   # steel core in a PLA sleeve, stub or spigot); prints ~3.05-3.1
M3_SOFT_PILOT = 2.9                # round 14 (Fernando): a gentle thread-forming pilot, through the whole hole, allowed across print layers
DOVETAIL_CLR = 0.1                 # round 14 (Fernando): every dovetail socket this much bigger than its tail on each face (first try)
MIN_FEATURE = 4.0                  # round 14 (Fernando): no structural part narrower than this in plan (no thin slivers to match faces)
# springs (pod round 4, Nov 3): the yellow 8 x 25 mm 3D-printer bed-levelling springs (Ender 3 / CR-10 upgrade, sold in 10-12 packs). They
# copy the JIS extra-light die spring for an Ø8 hole / Ø4 rod x 25 free (MW Components DJ-8X25Y: 6.1 N/mm, 10 mm long-life = 40 %,
# 12.5 mm max = 50 %). The Amazon packs give no rate: 6.1 N/mm is ASSUMED until a spring is measured (VERIFY). One spring alone would reach
# 46 % at bump, so each side has TWO side by side, each on an M4 button-head bolt used as a plunger: its dome presses on the fork pad, its
# shank slides in the frame's guide hole, an M4 nyloc on top keeps it captive (it never bottoms: the fork stops first); an M4 washer seats
# the spring. No damper. Replaced the ISO 10243 Ø10 x 38 die spring (McMaster 1804N112) of rounds 2-3.
SPRING_HOLE_D, SPRING_ROD_D, SPRING_L0 = 8.0, 4.0, 20.2   # Nov 20 (Fernando): 7.85 OD x 4.25 ID x 20.2 free (v15 Nov 14: 7.7 x 18.5;
                                   # the same springs, measured in more detail)
SPRING_SOLID = 12.0                # Nov 20 (Fernando, measured): the coils touch at 12 mm (v15 Nov 14's figure: 9.23)
SPRING_OD, SPRING_ID = 7.85, 4.25  # Nov 20 (Fernando)
SPRINGS = {'yellow 8x25': (6.1, 10.0),   # N/mm, long-life working deflection mm (max 12.5)
           '7.7x18.5 (Nov 14)': (10.4, 18.5 - 9.23 - 1.0),
           '7.85x4.25x20.2 (Nov 20)': (7.6, SPRING_L0 - SPRING_SOLID - 0.5)}   # Nov 21: 7.6 N/mm ESTIMATED (was 10.4 kept from Nov 14): a JIS
                                   # yellow (extra light) 8 x 20 die spring, which these match, is listed at 7.6 (Century DJ-8X20YCS 7.62, Raymond
                                   # ASF 8x20 0.78 kgf/mm, Dingli DTF8-20 7.8). VERIFY with the kitchen-scale test (5 mm: ~38 N, 3.9 kg).
                                   # Reports only: the towers come from STACK_PRELOAD_AT_DROOP. sim studies/springs_v16.py
SPRING_DESIGN = '7.85x4.25x20.2 (Nov 20)'   # its rate gives C_NOM (reports, the old M4 pod layers); the towers are set by STACK_PRELOAD_AT_DROOP
SPRINGS_IN_SERIES = 1              # Nov 20 (Fernando: "double bore per tower but assume we will only use one spring per bore, so shorter towers"):
                                   # one spring per bore, two bores per tower (v15 Nov 15 - Nov 20: two springs stacked in series in each bore)
STACK_PRELOAD_AT_DROOP = 0.1       # Nov 20: the spring ceiling in the tower (pod.Z_CEIL) is set so the far bore's spring (the one that droops most) is
                                   # this much under its free length at droop: it just touches (v12's springs were slack there); the margin to solid at
                                   # bump follows (pod.STACK_MARGIN_BUMP: 0.52, the 8.2 mm stroke less the far plunger's 7.58). Earlier on Nov 20, two
                                   # springs per bore: 1.0 at droop (a soft suspension, but an M4 x 60 and the tip at the lidar's plane), 1.0 off solid at
                                   # bump (7.8 of preload: the fork likely sat on its droop stop); Nov 15: STACK_PRELOAD 5.0 (the far pair bound at +9)
PLUNGER_SEAT_OVER_PAD = 2.7        # the plunger (plunger.py, which asserts it): its seat washer's top over the pad. Nov 20 M4 set: head 2.2 + DIN 433
                                   # washer 0.5 (the M3 set: cap 1.2 + head 1.8 + washer 0.6 = 3.6)
PLUNGER_DOME_R = ((7.6 / 2) ** 2 + 2.2 ** 2) / (2 * 2.2)   # what touches the pad: the M4 button head's dome, R 4.38 (the M3 set: its cap's R8)
                                   # (STACK_PRELOAD, Nov 15 evening: the spring seat 5 mm lower than the design-mass geometry asked, so the two
                                   # springs were always compressed; the Nov 20 rule keeps them compressed at droop too)
SPRINGS_PER_SIDE, SPRING_PITCH = 2, 10.6   # two plungers per side, this far apart in y (2 mm wall between the Ø8.6 bores)
DESIGN_MASS = 10.0                 # kg robot (all on the 4 wheels: conservative)
SPRING_ARM = 26.0                  # centre of the spring pair this far from the pivot toward the axle (springs at 31.3 / 20.7): wheel rate 2.6 N/mm
BOLT_HEAD_D, BOLT_HEAD_K = 7.6, 2.2    # ISO 7380 M4 button head
SHANK_R, GUIDE_R = 2.0, 2.2        # M4 shank; its guide hole in the tower cap
WASHER_OD, WASHER_T = 9.0, 0.8     # M4 washer (ISO 7089)
NUT_T, NUT_AF, GUIDE_LEN, NUT_GAP = 5.0, 7.0, 16.0, 1.5  # M4 nyloc (DIN 985). GUIDE_LEN: the plunger's guide (bushing) in the tower's top; Nov 20
                                   # (Fernando: "increase the length of the bushing for smoother M4 motion") 16, was 8 (v12 to Nov 20); the
                                   # M4 x 50 (his longest) would allow up to ~21 with one spring per bore
PAD_Z = 24.0                       # top of the fork pads under the plungers (both sides)
PAD_HALF = 13.0                    # pad half length (y): each dome slides about 6 mm along it over the travel, 5.3 mm either side of the centre
TOWER_CLEAR = 10.0                 # tower underside above the pad top (bump travel + rotation; the far plunger lifts 6.4 mm)
# fork (one printed part: two arms + a beam behind the wheel)
OUTER_ARM_X = (-60.5, -43.5)       # 4 mm from the cartridge face (x -39.5)
INNER_ARM_X = (MOTOR_X[1], MOTOR_X[1] + 13.0)   # clamped to the motor's rear face by 4 x M2.5 x 14 pan heads in Ø5.6 counterbores
OUTER_BOSS_R, INNER_PLATE_R = 17.0, 29.5
LEAD_NOTCH_R = 28.0                # inner plate cut back to this radius where the leads pass
REAR_CLR_D, REAR_CBORE_D = 2.9, 5.6   # Nov 10 (Fernando: long M2.5s, heads closer to the plate's outer face): M2.5 clearance; counterbore for the Ø5 x 2 pan head
REAR_SCREW_L, REAR_ENGAGE = 14.0, 3.8   # head seat 2.8 under the plate's outer face (head top 0.8 under it; was 4.8 deep with M3 x 12). 14 is the longest that fits:
                                   # 15 would leave the head proud or bottom in the 4.3 deep hole
MOTOR_POCKET_D, MOTOR_POCKET_CLR = 2.0, 0.1   # Nov 10 (Fernando): shallow tight-fit pocket for the can in the fork's motor face, so the stator drops onto the hole pattern.
                                   # The inner half's motor face moves this far toward the wheel (the motor does not move); the pocket's floor is the old face. 0.5 x 45 deg
                                   # lead-in at its mouth (and against elephant's foot); the can's rear edge has its own 0.5 chamfer, so 1.0 of straight wall grips. Prints motor
                                   # face down: support only inside the pocket. 0.1 radial is a first try for a tight fit (VERIFY on a print); 0 = no pocket
WHEEL_CLEAR_R = 74.0               # nothing of the fork between the arms inside this radius about the axle; the beam is the pivot lobe
                                   # (r LOBE_R) across, cut to this radius: no tail behind the pivot since the stops moved to the springs (round 5)
# axle stub (printed, bolted into the outer arm from outside; carries the left 6804's inner race)
STUB_D, STUB_SHOULDER_D, STUB_FLANGE_D, STUB_FLANGE_T = 20.0, 23.0, 34.0, 3.0
STUB_SHOULDER_GAP = 0.0            # to the inner race (Nov 13: 0, was 0.3; the retainer's screw clamps the inner race against the shoulder)
STUB_SCREW_R = 13.5                # 3 x M3 x 12 into the arm
# side frames (two printed parts, each printed lying on its flat outer face)
FRAME_GAP, FRAME_T = 1.0, 12.0     # fork arm to frame; frame thickness (x) = foot width
FOOT_T = 8.0                       # foot on the plate top
FOOT_Y = (20.0, 104.0)
KEEL_Y = (41.6, 92.0)              # keel below the plate, from the spring column's front to 11.4 mm behind the pivot: needs a notch in the plate
# hard stops at the springs (round 5): bump = two rails under each tower, either side of the spring washers, that land flat on the fork pad
# at bump; droop = a hook from the keel under the fork arm, below the springs. Their faces are the pad / arm faces at bump / droop.
STOP_HALF = 8.0                    # droop hook half length (y), centred on the spring pair
HOOK_T = 6.0                       # droop hook thickness under the arm
LEDGE_W = 7.0                      # droop hook reach inward under the arm (x; 6 mm under it past the 1 mm gap)
RAIL_CLEAR = 1.0                   # bump rails to the spring washers (x)
# frame strength (pod round 3, Nov 3): the spring pushes the tower up and the shaft pulls the keel down, so the frame bends between them;
# a full-height web joins the column to the keel (no notch at the column root) and a gusset braces the column back onto the keel
TOWER_HALF = 13.0                  # spring column and tower half width (y): 3.4 mm wall past the two Ø8.6 spring bores
TOWER_REACH = 8.5                  # tower inner face this far past the spring axes (x): 4.2 mm wall
GUSSET_TOP_Z, GUSSET_END_Y = 50.0, 90.0   # gusset from the column's back face at this height down to the foot top at this y (over the shaft)
FILLET_COL, FILLET_GUSSET = 2.0, 5.0   # concave fillets: column front / foot (clear of the y 35 screw head), gusset to column and foot
MOUNT_Y = (25.0, 35.0, 98.0)       # vertical M3 through each foot into the plate (clear of the spring column, the gusset and the keel notch)
PLATE_T_ASSUMED = 6.0              # for the cutout check only

# ---- 687 fit coupon (coupon_687.py; Nov 13, Fernando: a very compact, very tight fit test of the 687 in the roller core and on the sleeve) ----
# Four cells in a 2 x 2 block; cell n has a core pocket open on the bed side and a sleeve stub standing on top, both one step tighter than cell n-1.
# Cell 1 is the drawn roller (pocket 2 x (BRG_DO/2 + BRG_POCKET_FIT) = 14.10, sleeve BRG_D = 7.00).
C687_POCKET = (14.10, 14.05, 14.00, 13.95)   # pocket diameters, cells 1-4
C687_PIN = (7.00, 7.05, 7.10, 7.15)          # sleeve stub diameters, cells 1-4
C687_PITCH = 18.0                  # cell pitch: 4.0 between pockets, the core's 4.15 wall from pocket to key slot
C687_WALL = 4.0                    # pocket to the block's outer faces
C687_POCKET_D = 5.6                # pocket depth: the bearing (5.0) pressed from the bed side stops flush with the face (press it on a flat bench)
C687_TOP = 12.0                    # block height: 45 deg cone roof over each pocket (prints without support) up to the sleeve bore
C687_SHOULDER = 0.8                # sleeve stub: Ø BRG_IR_LAND shoulder this high (the bearing's inner race lands on it), then the Ø pin ...
C687_PIN_LEN = 6.0                 # ... this long (bearing + 1.0), SLEEVE_LEAD chamfer, SLEEVE_ID bore through (the real sleeve's 1.9 wall)
C687_PUSH_D, C687_PUSH_R = 1.8, 6.05  # two push-out holes per pocket onto the outer ring (r 6-7; a 1.5 mm hex key), inside the tightest pocket (r 6.95)
C687_LABEL = (4.0, 0.4)            # engraved cell number: font size, depth
# Printed Nov 13 (Fernando): the best pocket is cell 2 (14.05), the best pin cell 4 (7.15); both are in the roller now (BRG_POCKET_FIT, SLEEVE_BRG_D)

# ---- 627 fit coupon (coupon_627.py; Nov 13, Fernando: the fork's 627 seats, Ø22.0 as drawn, are too loose) -------------------------------------
# As coupon_687: four cells in a 2 x 2 block, a 627 pocket open on the bed side (as the fork seats print, axis vertical) under a 45 deg cone roof,
# and a pin on top for the inner race (the inner pivot's printed pin, Ø7 with an M3 threaded through it: a Ø2.8 bore). Both get tighter cell by cell.
# Nov 13 evening (Fernando: compact): a 7.6 base plate with the pockets, and over each pocket only a 45 deg cone shell (outside and in) up to the pin,
# instead of a solid 18.2 block (35 cm3 -> about 18)
C627_POCKET = (21.95, 21.90, 21.85, 21.80)   # pocket Ø, cells 1-4 (the fork's seats are 22.00: loose)
C627_PIN = (7.05, 7.10, 7.15, 7.20)          # pin Ø, cells 1-4 (the 687's sleeve took 7.15 on its 7.0 bore)
C627_WALL = 2.5                    # pocket to pocket and to the block's faces: the fork's lobes leave 2.5 round their 627s (evening; was 3.0)
C627_POCKET_D = 7.6                # pocket depth: the bearing (7.0) pressed from the bed side stops flush with the face; the base plate's height
C627_CAP = 0.6                     # solid over the inner cone's apex, under the shoulder (the outer cone ends at the shoulder's Ø there)
C627_SHOULDER_D, C627_SHOULDER = 10.0, 0.8   # pin's shoulder on the inner race only (627 inner ring ~Ø10.5-11), its height
C627_PIN_LEN = 8.0                 # pin length (bearing + 1.0)
C627_BORE = M3_PILOT               # through the pin and the roof, as the inner pivot pin's M3 pilot
C627_PUSH_D, C627_PUSH_R = 1.6, 10.05   # two push-out holes per pocket onto the outer ring (r 9.25-10.85), inside the tightest pocket (r 10.9)
# Printed Nov 13 evening (Fernando): the best pocket is cell 1 (21.95), the best pin cell 4 (7.20, the largest offered: a larger one not tried). The pod's
# 627 seats and the pins / spigots in its bore take them (pod_lean: the outer arm's and the frame's seats; the fork's pin, the outer cap's and sleeve's spigots)
B627_SEAT_D = C627_POCKET[0]       # 21.95: a 627 seat (the bearing's OD 22.0)

# ---- dovetail fit coupon (coupon_dovetail.py; Nov 15 evening, Fernando: the deck laps' 0.1 clearance is far too loose; try 0.05 and 0) -----------
CDT_CLR = (0.05, 0.0)              # socket over the tab, per face, the socket bar's two cells (Fernando: one male tab, two sockets 0.05 and 0;
                                   # Nov 17, printed (Fernando): 0.05 fits best, now the deck's chassis_mid.TAB / KEY clr (was 0.1)
CDT_T = 10.0                       # both bars as thick as the deck's half-lap layer (chassis_mid: the tab is the lower piece's bottom half)
CDT_WALL, CDT_BACK = 6.0, 8.0      # bar wall between / beside the cells; bar behind the tab roots / behind the sockets' far ends
CDT_LABEL = (5.0, 0.6)             # engraved clearance labels on the socket bar: font size, depth
B627_PIN_D = C627_PIN[3]           # 7.20: a printed pin or spigot in a 627's 7.0 bore
