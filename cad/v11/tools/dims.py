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
KEY_W, KEY_HALF = 2.4, 15.0        # three tread keys in core keyways: width, half length

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
WEB_T = 16.0                       # spoke web thickness (x)
WEB_REACH = 12.0                   # web foot half width on the drum, either side of the block centre (local z)

ENGAGE = 6.5                       # M3 x 50 axle thread engagement in the pilot block
SEAT = 50.0 - ZF - ENGAGE          # axle head underside (local z); the shank runs 50 mm toward -z
HEAD_POCKET_D, HEAD_DEPTH = 6.4, 3.0
B_OUT = SEAT + HEAD_DEPTH          # head-side block outer face
PILOT_D = 2.9                      # roller axle thread pilot (printed across the layers; every other M3 pilot is 2.8)
PILOT_DEPTH = ENGAGE + 0.8
A_OUT = ZF + PILOT_DEPTH + 1.2     # pilot-side block outer face
CLR_D = 3.4
TUNNEL_D = 6.4                     # insertion tunnel from each head pocket outward along the axle

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
SHAFT_D, SHAFT_CLR = 7.0, 8.0      # steel pivot shaft, fixed in the frames; fork bore clearance
B627_OD, B627_W = 22.0, 7.0        # 2 x 627 (owned) pressed into the fork's pivot lobes
LOBE_R = 15.0                      # fork pivot lobe radius
M3_PILOT, M3_CLR = 2.8, 3.4
# springs: ISO 10243 die springs, Ø10 hole / Ø5 rod x 38 free (HPC R203106 green 6.8 N/mm, max working 13.3 mm; R204106 blue 11.9 N/mm,
# 11.4 mm; same size at McMaster / Misumi). One per side on an M5 x 50 button-head bolt as a plunger: its dome presses on a pad on the fork,
# its shank slides in the frame's guide hole, an M5 nyloc on top keeps it captive; an M5 washer seats the spring. No damper.
SPRING_HOLE_D, SPRING_ROD_D, SPRING_L0 = 10.0, 5.0, 38.0
SPRING_OD, SPRING_ID = 9.6, 5.4    # modelled envelope (VERIFY on the part)
SPRINGS = {'green': (6.8, 13.3), 'blue': (11.9, 11.4)}   # N/mm, max working deflection mm
SPRING_DESIGN = 'green'            # the nominal geometry is for this spring at DESIGN_MASS; heavier robots: shims or the blue spring
DESIGN_MASS = 10.0                 # kg robot (all on the 4 wheels: conservative)
SPRING_ARM = 30.0                  # spring line this far from the pivot toward the axle: spring travel = 0.372 x axle travel
BOLT_HEAD_D, BOLT_HEAD_K = 9.5, 2.75   # ISO 7380 M5 button head
WASHER_OD, WASHER_T = 10.0, 1.0
NUT_T, GUIDE_LEN, NUT_GAP = 4.0, 8.0, 1.5
PAD_Z = 24.0                       # top of the fork pads under the plungers (both sides)
PAD_HALF = 9.0                     # pad half length (y): the dome slides 6 mm along it over the travel
TOWER_CLEAR = 9.0                  # tower underside above the pad top (bump travel + rotation)
# fork (one printed part: two arms + a beam behind the wheel)
OUTER_ARM_X = (-60.5, -43.5)       # 4 mm from the cartridge face (x -39.5)
INNER_ARM_X = (MOTOR_X[1], MOTOR_X[1] + 13.0)   # clamped to the motor's rear face by 4 x M3 x 12 (VERIFY thread) in Ø6.8 counterbores
OUTER_BOSS_R, INNER_PLATE_R = 17.0, 29.5
LEAD_NOTCH_R = 28.0                # inner plate cut back to this radius where the leads pass
REAR_CBORE_D = 6.8
REAR_SCREW_L, REAR_ENGAGE = 12.0, 3.8
BEAM_TAIL = (96.0, 10.0)           # beam: hull of the pivot lobe and a tail circle (y, r), cut clear of the wheel; its end meets the stops
WHEEL_CLEAR_R = 74.0               # nothing of the fork between the arms inside this radius about the axle
# axle stub (printed, bolted into the outer arm from outside; carries the left 6804's inner race)
STUB_D, STUB_SHOULDER_D, STUB_FLANGE_D, STUB_FLANGE_T = 20.0, 23.0, 34.0, 3.0
STUB_SHOULDER_GAP = 0.3            # to the inner race
STUB_SCREW_R = 13.5                # 3 x M3 x 12 into the arm
# side frames (two printed parts, each printed lying on its flat outer face)
FRAME_GAP, FRAME_T = 1.0, 12.0     # fork arm to frame; frame thickness (x) = foot width
FOOT_T = 8.0                       # foot on the plate top
FOOT_Y = (20.0, 122.0)
KEEL_Y = (64.0, 112.0)             # keel below the plate (shaft, stops): needs a notch in the plate
STOP_Y = (92.0, 106.0)             # droop and bump ledges over / under the fork's tail
LEDGE_W = 6.0                      # ledge reach inward over the tail (x)
MOUNT_Y = (25.0, 35.0, 117.0)      # vertical M3 through each foot into the plate (clear of the spring column and the keel notch)
PLATE_T_ASSUMED = 6.0              # for the cutout check only
