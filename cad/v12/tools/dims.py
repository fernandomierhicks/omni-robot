"""Dimensions of the cad/v11 compact wheel + motor, mm.

Pod frame: x = wheel axis (motor output flange toward -x, cup stub toward +x); y, z radial. Roller frames: local z = roller axis (tangent),
local y = radial outward, local x = pod x. Everything is nominal CAD; nothing has been printed.
"""
import math

# ---- wheel envelope (same as the v10 / v9 wheel and the sim baseline: Ø140, roller axes at r 52) ---------------------------
R_WHEEL = 70.0                     # wheel outer radius; the roller tread profile is this circle about the wheel axis
R_AXIS = 52.0                      # roller axis radius
D_ROW = 17.25                      # row planes at x = -D_ROW (row 1) and +D_ROW (row 2); rows are 30 deg apart in azimuth
N_PER_ROW = 6

# ---- rollers (PLA core + TPU tread, flush flat ends) -----------------------------------------------------------------------
HT = 19.5                          # half length of core and tread (2.35 deg overlap between neighbouring rollers)
TREAD_T = 2.5                      # TPU tread thickness
FILLET_TREAD = 2.0                 # TPU end-edge fillet
BORE_D = 3.4                       # roller bore on the M3 axle (was 3.6: too much slop)
JOURNAL, JOURNAL_R = 0.75, 4.0     # thrust journal at each end: length, radius
KEY_W, KEY_DEPTH = 2.4, 1.5        # three tread keys in core keyways open at both ends (the tread slides off): width, depth into the core
CORE_R = 12.7                      # straight core / tread bore (round 6): the barrel interface needed the TPU stretched 2.8 mm over the core's
                                   # middle, so a printed tread could not be taken off; the tread is now 2.5 mm thick at the ends, 5.3 in the middle
CORE_CHAMFER = 0.5                 # lead-in on the core ends and the tread bore ends
DETENT_DEPTH, DETENT_RIB = 0.5, 0.4   # round 7: the tread snaps into place: 45 deg V groove round the core's middle, a matching V rib 0.4 high
                                   # inside the tread bore (0.1 clear at its tip). Fitting stretches the bore 0.4 mm in radius (3 %), by hand
HUB_EDGE_R = 0.8                   # round 7: round on the spoke webs' x-end edges (each spoke filleted before the union) and the drum's left end

# ---- hub ----------------------------------------------------------------------------------------------------------------------
MOTOR_SHIFT_X = 27.5              # option 1 (Nov 2): motor moved outboard (+x) so its XT30 plug and leads sit outside the hub with >= 4 mm to anything rotating
X_LEFT, X_RIGHT = -39.5, 29.0      # hub end faces; the right end is the drum only: no right bearing, the hub rides on the motor output bearing + the left 6804
DRUM_IN, DRUM_OUT = 30.0, 33.0     # drum bore and outside radius (the front of the motor can lives in the bore)
WALL_BACK = -26.5                  # left end wall (r ACCESS_R .. DRUM_IN) ends here at the access bore and runs out on a 45 deg cone to the drum
FLANGE_BACK = -23.0 + MOTOR_SHIFT_X        # flange plate clamped to the motor output face by three M2.5 screws
FLANGE_FRONT = FLANGE_BACK - 3.5           # a 45 deg cone joins its front to the drum (printable cavity roof)
MOTOR_ROT = 143.0                  # motor turned about the axis from its STEP pose: XT30 at az ~160 (back, slightly up: away from the pivot and shocks)
FLANGE_R = 13.5
FLANGE_ANGLES = tuple(a + MOTOR_ROT for a in (30, 150, 270))   # follow the motor's M2.5 output holes
CART_ANGLES = (30, 150, 270)       # cartridge screws
M25_CLEAR = 2.7
PLATE_T = 3.0                      # cartridge plate, flush with the left end face
X_PLATE_FLOOR = X_LEFT + PLATE_T
ACCESS_R = 18.0                    # left access bore (cartridge boss, driver access to the flange screws)
CART_R = 26.0                      # cartridge screws (M3 x 12 flat head into Ø2.8 pilots)
R_BRG, BRG_FIT = 16.0, 0.05        # 6804 outside radius and seat allowance

THRUST_GAP = 0.3                   # journal face to block face
ZF = HT + JOURNAL + THRUST_GAP     # block thrust faces (+-ZF about the roller centre); the flat cut runs across the whole roller end
BAND_C = 0.95                      # radial clearance around the tread
BLEND_R = 3.0                      # blend between the thrust face and the tread clearance: the fillet at the thin spoke neck
BLOCK_R = 8.0                      # axle block radius
WEB_OUT = 12.0                     # spoke web reaches this far outboard of its row plane (x; row 2 stops at the drum end X_RIGHT)
WEB_GAP = 6.0                      # inboard the webs run to x = +-WEB_GAP / 2 (the other row's rollers reach x +-1.7 with clearance)
                                   # round 6: the roller cut is the roller's revolved envelope plus its radial drop-in channel, not a
                                   # prism through x, so the web outside the roller end stays and wraps it (v10 had 24 mm deep spokes)
WEB_REACH = 12.0                   # web foot half width on the drum, either side of the block centre (local z)

ENGAGE = 6.5                       # M3 x 50 axle thread engagement in the pilot block
SEAT = 50.0 - ZF - ENGAGE          # axle head underside (local z); the shank runs 50 mm toward -z
TUNNEL_D = 8.0                     # round 6-7 head pocket + driver tunnel (double teardrop); now only the teardrop helper's default size
HEAD_POCKET_D, HEAD_DEPTH = 7.0, 3.0   # round 8: round counterbore for the M3 pan head (Ø6.5 x 2.5), Ø3.4 shank hole to the thrust face,
FLARE_L = 5.0                      # then a 45 deg cone opening outward for FLARE_L (to r 8.5, past the block), then Ø7 on for the head and driver.
                                   # Longer cuts into the neighbouring roller's head seat (the two axles in a spoke cross 4 mm past their heads):
                                   # 5: 99 % of its seat left, 7: 61 %, 9: 30 %
AXLE_HEAD_D = 6.4                  # axle head envelope for the insertion graze check
B_OUT = SEAT + HEAD_DEPTH          # head-side block outer face
PILOT_D = 2.9                      # roller axle thread pilot (printed across the layers; every other M3 pilot is 2.8)
PILOT_DEPTH = ENGAGE + 0.8
A_OUT = ZF + PILOT_DEPTH + 1.2     # pilot-side block outer face
CLR_D = 3.4

# ---- motor and harness (the pod's fork bolts straight to the motor's rear face; no motor mount) -------------------------------
# components/ak40-10/AK40-10 with connector.STEP is the round-2 pod-frame motor, moved; this puts it back, then MOTOR_SHIFT_X outboard,
# then turns it MOTOR_ROT about the axis
MOTOR_STEP_SHIFT = (34.462 + MOTOR_SHIFT_X, 41.0002, -49.0161)
MOTOR_R = 26.5                     # can radius
MOTOR_X = (-23.0 + MOTOR_SHIFT_X, 17.2 + MOTOR_SHIFT_X)   # output face, rear face
REAR_HOLES_AZ = tuple(a + MOTOR_ROT for a in (-120, -30, 60, 150))   # motor rear face: 4 x Ø2.8 x 4.3 deep at r 23.5 in the STEP (taken as M3, VERIFY)
REAR_HOLES_R, REAR_HOLE_DEPTH = 23.5, 4.3
LEAD_D = 3.0                       # ASSUMED power lead OD (VERIFY)
LEAD_STRAIGHT = 0.5                # straight lead beyond the pin ends before the bend
LEAD_BEND_R = LEAD_D               # sharp 90 deg bend (centreline radius = 1 x OD) toward +x
LEAD_RUN_TO = MOTOR_X[1] + 12.0    # leads modelled to here (x) after the bend, inside the plate cutout (route on to the robot not drawn)
HARNESS_CLEAR = 4.0                # wanted clearance all round the plug and leads


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
SEAT_RELIEF_D, SEAT_RELIEF_T = 18.0, 1.0   # beyond each seat, so the seat bottom bears on the outer race only
PIVOT_BOSS_D = 10.0                # frame boss onto the inner race only (627 inner ring about Ø10.5-11)
PIVOT_HEAD_D, PIVOT_HEAD_RECESS = 7.0, 2.6   # counterbore for the M3 pan head in the keel's outer face (flush: the keel sits in the plate notch)
PIVOT_SCREW_L = 16.0               # M3 pan head: all of it threaded into the keel (9.4), boss (1) and pin (5.6 of 6.9)
LOBE_R = 15.0                      # fork pivot lobe radius
M3_PILOT, M3_CLR = 2.8, 3.4
M3_SOFT_PILOT = 2.9                # round 14 (Fernando): a gentle thread-forming pilot, through the whole hole, allowed across print layers
DOVETAIL_CLR = 0.1                 # round 14 (Fernando): every dovetail socket this much bigger than its tail on each face (first try)
MIN_FEATURE = 4.0                  # round 14 (Fernando): no structural part narrower than this in plan (no thin slivers to match faces)
# springs (pod round 4, Nov 3): the yellow 8 x 25 mm 3D-printer bed-levelling springs (Ender 3 / CR-10 upgrade, sold in 10-12 packs). They
# copy the JIS extra-light die spring for an Ø8 hole / Ø4 rod x 25 free (MW Components DJ-8X25Y: 6.1 N/mm, 10 mm long-life = 40 %,
# 12.5 mm max = 50 %). The Amazon packs give no rate: 6.1 N/mm is ASSUMED until a spring is measured (VERIFY). One spring alone would reach
# 46 % at bump, so each side has TWO side by side, each on an M4 button-head bolt used as a plunger: its dome presses on the fork pad, its
# shank slides in the frame's guide hole, an M4 nyloc on top keeps it captive (it never bottoms: the fork stops first); an M4 washer seats
# the spring. No damper. Replaced the ISO 10243 Ø10 x 38 die spring (McMaster 1804N112) of rounds 2-3.
SPRING_HOLE_D, SPRING_ROD_D, SPRING_L0 = 8.0, 4.0, 25.0
SPRING_OD, SPRING_ID = 7.7, 4.3    # modelled envelope (VERIFY on the part)
SPRINGS = {'yellow 8x25': (6.1, 10.0)}   # N/mm, long-life working deflection mm (max 12.5)
SPRING_DESIGN = 'yellow 8x25'      # the nominal geometry is for this spring at DESIGN_MASS; other masses: printed shims (spring table)
SPRINGS_PER_SIDE, SPRING_PITCH = 2, 10.6   # two plungers per side, this far apart in y (2 mm wall between the Ø8.6 bores)
DESIGN_MASS = 10.0                 # kg robot (all on the 4 wheels: conservative)
SPRING_ARM = 26.0                  # centre of the spring pair this far from the pivot toward the axle (springs at 31.3 / 20.7): wheel rate 2.6 N/mm
BOLT_HEAD_D, BOLT_HEAD_K = 7.6, 2.2    # ISO 7380 M4 button head
SHANK_R, GUIDE_R = 2.0, 2.2        # M4 shank; its guide hole in the tower cap
WASHER_OD, WASHER_T = 9.0, 0.8     # M4 washer (ISO 7089)
NUT_T, NUT_AF, GUIDE_LEN, NUT_GAP = 5.0, 7.0, 8.0, 1.5   # M4 nyloc (DIN 985)
PAD_Z = 24.0                       # top of the fork pads under the plungers (both sides)
PAD_HALF = 13.0                    # pad half length (y): each dome slides about 6 mm along it over the travel, 5.3 mm either side of the centre
TOWER_CLEAR = 10.0                 # tower underside above the pad top (bump travel + rotation; the far plunger lifts 6.4 mm)
# fork (one printed part: two arms + a beam behind the wheel)
OUTER_ARM_X = (-60.5, -43.5)       # 4 mm from the cartridge face (x -39.5)
INNER_ARM_X = (MOTOR_X[1], MOTOR_X[1] + 13.0)   # clamped to the motor's rear face by 4 x M3 x 12 (VERIFY thread) in Ø6.8 counterbores
OUTER_BOSS_R, INNER_PLATE_R = 17.0, 29.5
LEAD_NOTCH_R = 28.0                # inner plate cut back to this radius where the leads pass
REAR_CBORE_D = 6.8
REAR_SCREW_L, REAR_ENGAGE = 12.0, 3.8
WHEEL_CLEAR_R = 74.0               # nothing of the fork between the arms inside this radius about the axle; the beam is the pivot lobe
                                   # (r LOBE_R) across, cut to this radius: no tail behind the pivot since the stops moved to the springs (round 5)
# axle stub (printed, bolted into the outer arm from outside; carries the left 6804's inner race)
STUB_D, STUB_SHOULDER_D, STUB_FLANGE_D, STUB_FLANGE_T = 20.0, 23.0, 34.0, 3.0
STUB_SHOULDER_GAP = 0.3            # to the inner race
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
