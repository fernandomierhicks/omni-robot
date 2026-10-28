"""v8 CAD parameters (mm): v7 plus a fork round each wheel, hex-drive heads, labels and stronger cheeks (design.md 14).
The layout comes from the v7 simulation so the CAD and the sim can't drift.

Robot built here (design.md 25 + 22-24): the v7 home-floor base (X4, printed 127 mm omni wheels with 2 x 5 rollers of
36 mm, sprung pods) carrying the v6 round-3 tilting catch / launch module (chute pocket, two Pololu 25D feed wheels,
J-bend, two F90 flywheels, STS3250 tilt servo), on section 24's Damiao DM-2325 drive and a 7S1P pack.

Coordinate-frame contracts (every part is modeled in its own frame, then placed):
  ROBOT    origin on the floor under the base centre; x forward (launcher side), y left, z up.
  POD k    origin at wheel k's centre (static ride height); x along the axle pointing INWARD (toward the robot
           centre), z up, y = z cross x. Wheel k sits at WHEEL_ANGLES[k]; its pod +y points clockwise (seen from above),
           so all four pods are identical parts (rotational symmetry).
  MODULE   the catch module (as the sim's robot_model.geometry): x forward, z up the funnel axis, origin at the module
           base on the centreline. At tilt phi it rotates about the pivot (y axis); MODULE -> ROBOT is module_to_robot().
  PAYLOAD  origin at the payload-plate top centre; axes parallel to ROBOT.

Fasteners (Fernando, Oct 21): M3 wherever possible, no heat-set inserts. Screws form their own threads in undersized
printed holes: TAP_M3 = 2.8 mm, at least 6 mm (2 x d) of engagement. Through holes are 3.4 mm. Nuts only where a
joint is a pivot or must be clamped through.
Print materials: PETG for structure, PLA for low-load parts, TPU 95A for tyres / treads / bumpers / skirt.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SIM_DIR = ROOT / "simulation" / "v7 home floor"
sys.path.insert(0, str(SIM_DIR))

from params import SimParams  # noqa: E402
from robot_model import geometry  # noqa: E402

# the v7 proposal (house.BASES["v7"]) on section 24's drive and feed choice. plate_gap (CAD, Oct 21): the payload
# plate rides on six standoffs high enough for the lidar's scan plane to pass over the wheels at their bump stop and
# under the plate (deck top 66.5, scan plane 150, lidar top 160.5, plate underside 162.5); the module and its pivot
# rise with the plate (the sim's 0.065 put the scan plane through the wheel houses: design.md 29, finding 6)
PLATE_GAP = 0.096
SIM = SimParams(wheel="Printed-127-2x5-36", suspension="pod", drive_motor="DM-2325",
                feed_motor="Pololu-25D-RoboClaw", feed_motor_reach=0.080, plate_gap=PLATE_GAP)
GEO = geometry(SIM)
MM = 1000.0

# --- printer -------------------------------------------------------------------------------------------------------
BED = 250.0                     # usable bed (Bambu 256 mm, keep a margin)

# --- holes (Fernando: undersized holes, no inserts) ----------------------------------------------------------------
TAP_M3 = 2.8                    # M3 thread-forming hole in PETG / PLA
CLR_M3 = 3.4                    # M3 through hole
CB_M3 = (7.2, 2.9)              # pan-head counterbore (dia, depth): the 6.5 x 2.5 hex-drive head sits 0.4 under the face
CSK_M3 = 6.6                    # 90 deg countersink dia (ISO 7046 head 5.5)
TAP_CSK = 0.5                   # entry countersink on tapped holes with thread to spare (hw.tap(entry=...))
FILLET = 3.0                    # inside fillet on load paths (printed parts crack at sharp inside corners)
ENGAGE_MIN = {"M2": 4.0, "M2.5": 5.0, "M3": 6.0}   # thread in printed parts (2 x d)
ENGAGE_METAL = {"M2": 1.6, "M2.5": 2.0, "M3": 2.4}  # thread in nuts and metal parts (0.8 x d, a nut's height)
NUT_M3 = (5.8, 2.6)             # hex nut trap (across flats, depth)
NYLOC_M3 = (5.8, 4.2)
TAP_M25, CLR_M25 = 2.2, 2.9     # vendor boards that only take M2.5 (Jetson, BNO085)
TAP_M2, CLR_M2 = 1.7, 2.4

# --- base layout (from the sim) -------------------------------------------------------------------------------------
WHEEL_R = SIM.wheel_radius * MM                  # 63.5
WHEEL_W = SIM.omni.width * MM                    # 80
ROLLER_D = SIM.omni.roller_diameter * MM         # 36
ROLLERS_PER_ROW = SIM.omni.rollers_per_row       # 5
L_WHEEL = SIM.wheel_base_radius * MM             # 150: robot centre to wheel centre plane
WHEEL_ANGLES = SIM.wheel_angles_deg              # 45, 135, 225, 315
DECK_Z = GEO.deck_z * MM                         # deck underside, 60.5 (deck in the axle plane)
DECK_T = SIM.deck_thickness * MM                 # 6
DECK_TOP = DECK_Z + DECK_T                       # 66.5
PLATE_TOP = GEO.plate_top * MM                   # 137.5
PLATE_T = 6.0
PLATE_BOT = PLATE_TOP - PLATE_T                  # 131.5
PLATE_R = SIM.plate_radius * MM                  # 120
SIM_DECK_R = SIM.deck_radius * MM                # 185 (the sim's deck; the CAD's must be bigger, see DECK_R)
SLOT_HW = SIM.deck_slot_half_width * MM          # 70.5: wheel slot half width in the deck
POD_TRAVEL = SIM.pod_travel * MM                 # 15 mm wheel travel, droop stop to bump stop
POD_RATE = SIM.pod_rate / 1000.0                 # 3.0 N/mm at the wheel
POD_PRELOAD = SIM.pod_preload                    # 2 N

# CAD choices for the base (not in the sim)
DECK_R = 223.0                  # deck / wheel-guard outer radius (v7: 205, v8: 217). The wheel's outer face is at r 190;
                                # round 2's fork pivots on a shaft held on both sides of the wheel, and the outer plate's
                                # pivot boss beside the slot (r 215) and the outer shock set the guard's panel (r 218-223)
SLOT_R0 = 85.0                  # wheel slots start this far out along each diagonal (the inner plate and its shock
                                # boss at r 86.5-101; at 88 the boss met the deck at full droop)
BUMPER_T = 8.0                  # TPU bumper depth outside DECK_R (v8: 12; round 2: thinner, to give back what the fork took)
LIDAR_SCAN_Z = 150.0            # A2M12 scan plane: 8 mm over the wheels at their bump stop (142), 12.5 under the plate
STANDOFF_D, STANDOFF_FLANGE = 12.0, (20.0, 3.0)   # printed payload standoffs (dia; end flanges dia, thickness)
# Six standoffs, deck to plate. Side pair under the yokes; the front pair on the lidar's rays through the side pair, so
# their shadows overlap; rear pair far from the lidar (thin shadows). The front pair clears the inner wheel slot by 2 mm.
STANDOFF_XY = ((97.0, 34.5), (97.0, -34.5), (0.0, 112.0), (0.0, -112.0), (-104.0, 34.0), (-104.0, -34.0))
# The front pair, 55 mm from the lidar, threw the widest shadows (12.5 deg each at Ø12): 6061 rods Ø8 instead (8.3 deg),
# M3 tapped at both ends, located in 1 mm recesses in the deck and the plate (design.md 12.1, Oct 24). The other four
# stay printed: they carry the sway (a rod's ends are near pins), and their shadows lie inside the rods' wedges or far.
ROD_XY = STANDOFF_XY[:2]
ROD_D, ROD_RECESS = 8.0, 1.0

# --- wheel (POD frame, x inward) -------------------------------------------------------------------------------------
ROW_X = (-WHEEL_W / 4, WHEEL_W / 4)              # outer row plane x = -20, inner row x = +20
ROLLER_AXIS_R = WHEEL_R - ROLLER_D / 2           # 45.5: wheel axis to each roller axis
ROLLER_HALF = 21.5                               # roller half length (24.1 would overlap its neighbours, design.md 25.3)
ROLLER_CONE = 4.5                                # coned roller ends: axial length of the cone
TREAD_T = 2.5                                    # TPU tread over the PETG core
PIN_D, PIN_L = 3.0, 58.0                         # 3 mm hardened steel pin, cut to 58 mm (one per roller)

# --- Damiao DM-2325 (DM-S2325-1EC drawing, Damiao Sept 2025; vendor/DM-S2325-1EC_drawing.pdf) ------------------------
DM_D = 28.0                     # body
DM_BODY = 60.6                  # face to rear (incl. the 2.5 mm rear cap)
DM_PILOT = (17.0, 2.0)          # pilot boss on the face (dia, height)
DM_SHAFT = (6.0, 5.0, 17.0)     # D shaft: dia, across the flat, length from the face (incl. the pilot)
DM_FACE_PCD = 22.0              # 4 x M3, 6.5 max depth, at 45 deg to the connector
DM_PLUG = (35.8, 39.8, 9.0, 5.5)  # connector housing: from / to (mm behind the face), width, height above the body
DM_RADIAL_N = 395.0             # max radial load on the shaft (reseller listing)

# --- POD stack along the axle (POD x, inward +) -----------------------------------------------------------------------
X_WHEEL_OUT = -WHEEL_W / 2      # -40 outer face of the rollers' envelope
X_FACE = -23.0                  # motor output face: the 17 mm shaft ends at the wheel's outer face
X_REAR = X_FACE + DM_BODY       # 37.6 motor rear
X_CUP_OUT = X_FACE - 3.0        # -26 outer face of the motor cup's end wall (countersunk face screws)
X_HUB = X_CUP_OUT - 2.0         # -28 Pololu #1999 hub, boss toward the motor
X_DRUM_OUT = X_HUB - 9.2 - 5.8  # -43 outer face of the wheel's outer end plate (4 mm under the #1999 hub's screw
                                # heads, which carry the drive torque; 2 mm before)
# v8 fork (Fernando, Oct 26: the wheel supported on both sides, not hung off one printed arm). Outboard of hub A's end
# plate: a boss on hub A carries a second 61808 (inner race), held by the fork's outer plate (outer race); the outer
# plate is joined to the arm by a 6061 bar under the deck on the pivot's side (+y), where bump moves it down, not up.
X_BOSS_OUT = (X_DRUM_OUT, -52.0)          # hub A's bearing boss (r 20, bore r 15.5: the #1999's screws inside it)
X_BRG_OUT = (-51.5, -44.5)                # the outer 61808, 1.5 mm off the end plate (a shoulder ring on the boss)
X_OUTER = (-54.5, -44.0)                  # the outer plate's housing (bore 52, a 3 mm lip outboard)
X_OPLATE = (-50.0, -41.0)                 # the outer plate's web beyond the end plate (rollers end at x -38.6): 9 mm
# Round 2 (Fernando, Oct 27: "a cantilever wheel assembly is going to break"; bearings and shocks on both sides, thick
# parts). The fork (inner plate, outer plate, the bar under the deck, the wheel between them on a 61808 at each end)
# pivots on one 8 mm steel shaft beside the wheel, held by an ear on each side of the wheel (both in the wheel guard),
# with a 608 in each plate; a shock on each side.
PIVOT_YZ = (68.2, 30.0)         # the shaft: 74.5 from the axle (the outer ear's 10 mm round it clears the rollers, r 65.5)
X_EAR_IN = (40.5, 48.0)         # inner ear, beside hub C's end plate (the rollers end at x 38.6); the shaft pinned in it
X_EAR_OUT = (-40.5, -30.5)      # outer ear, beside the wheel (cut back to r 65.5 from the axle round the rollers)
X_PIV_IN = (48.5, 55.5)         # the inner plate's 608 (shim to the inner ear at 48)
X_PIV_OUT = (-48.0, -41.0)      # the outer plate's 608 (shim to the outer ear at -40.5)
X_PBOSS_OUT = (-49.0, -40.5)    # the outer plate's pivot boss (a 1 mm lip outboard of its 608)
SHAFT = (-48.0, 56.0)           # 8 mm steel shaft, x ends
PIN_X = sum(X_EAR_IN) / 2       # the M3 through the inner ear and the shaft (a 3.4 mm cross hole)
FORK_BAR = dict(y=(66.0, 76.0), z=(-28.0, -8.0), x=(-49.0, 61.0))   # 6061 flat bar 10 x 20 on edge (v8: 6 x 20)
FORK_LUG_Y = (58.0, 66.0)                 # the inner plate's leg and the outer plate's lug, under the bar's inner face
FORK_SCREW_Z = (-12.0, -24.0)             # two M3 along y through the bar into each lug
X_CUP_IN = 41.0                 # inner end of the motor cup = outer face of the arm's bearing boss
X_BRG = (41.0, 48.0)            # 61808-2RS (40 x 52 x 7) on the arm boss: wheel's inner support
X_ARM = (49.0, 61.0)            # inner plate (v8: 49-57; round 2: 12 mm)
CUP_OD = 44.0
CUP_BORE = 28.6

# --- pod suspension (POD y-z plane) ------------------------------------------------------------------------------------
POD_Z_DECK = DECK_TOP - WHEEL_R  # 3.0: deck top in the POD frame
# The fork's shaft sits beside the wheel on the pivot side (+y). Mirrored pods put both pivots of a side on that side's
# lobe; with the shaft 68 mm out, the two plates' pivot bosses on a lobe come within ~10 mm of each other (L 150).
SLOT_R1 = 110.0                 # the wheel slot is 141 mm wide only beyond r 110 (the rollers start at r 112) ...
SLOT_HW_IN = 36.0               # ... and 72 mm wide from SLOT_R0 to SLOT_R1 (arm, cup, bearing housing: |y| < 30)
POD_MIRROR = {45.0: True, 135.0: False, 225.0: True, 315.0: False}   # mirrored pods put both left mounts on the
                                # left lobe and both right ones on the right lobe: the front lobe stays free for the
                                # lidar and camera posts, the rear for the electronics bay (mirror-symmetric robot)
# Two shocks, one each side of the wheel, both upright (POD y, z of the eyes): the inner one from the inner plate to the
# pod mount's tower, the outer one from the outer plate to a boss on the wheel guard's front panel. One spring for both
# (pod.shock_travel() gives each one's motion ratio and the spring rate).
SHOCK_IN = ((30.0, 10.0), (30.0, 58.3))
SHOCK_OUT = ((36.0, 10.0), (36.0, 58.0))
SHOCK_X = (64.0, 70.0)          # inner shock's eyes between these x (its 11 mm body clears the plate's inner face, 61)
SHOCK_X_OUT = (-59.5, -53.5)    # outer shock's eyes (the outer plate's web at -50 + a 3.5 mm boss)
X_TOWER = (73.5, 79.5)          # the pod mount's tower web (the body clears it too)
SHOCK_LO_YZ, SHOCK_HI_YZ = SHOCK_IN
HOUSE_R_IN = WHEEL_R + POD_TRAVEL + 4.0  # 82.5: wheel-house roof, 4 mm over the wheel at its bump stop
HOUSE_X_IN = L_WHEEL - PLATE_R - 2.0     # 28: the house stops outside the payload plate
HOUSE_Y = 75.0                  # house legs (on the lobes, outside the 70.5 slot)
# Rigid parts end 48 mm above the floor and the TPU skirt at 34 (round 2, Fernando: the bumper would catch a rug; v8: 42
# and 20). The skirt still meets a 74 mm ball below its centre (37 mm up), so a missed ball is pushed, not run over.
PANEL_Z0 = 48.0 - WHEEL_R
SHOCK_L, SHOCK_STROKE = 50.0, 10.0     # 1/16-scale RC oil shock, eye to eye extended (est, VERIFY)
BRG_608 = (8.0, 22.0, 7.0)

# --- catch module (MODULE frame, from the sim) ---------------------------------------------------------------------------
RB = GEO.rb * MM                                  # 37 ball radius
BALL_D = 2 * RB
W_IN = GEO.w * MM                                 # 39: padded half width of the pocket / chute / J
PAD_T = 6.0                                       # catch-pad foam (sim: lined walls); printed wall inner face at W_IN + PAD_T
WALL_T = 3.0
THROAT_Z = GEO.throat_z * MM                      # 136.9
RIM_Z = GEO.rim_z * MM                            # 223.0
FUNNEL_RIM_R = SIM.funnel_rim_radius * MM         # 165
FUNNEL_THROAT_R = SIM.funnel_throat_radius * MM   # 42
FUNNEL_ANGLE = math.radians(SIM.funnel_wall_angle_deg)   # 55 deg from the axis
HOLD = (GEO.hold[0] * MM, GEO.hold[2] * MM)       # resting ball centre (x, z)
CHUTE = GEO.chute_angle                           # -40 deg: floor direction
THETA = GEO.theta                                 # 30 deg launch line in the module
U = GEO.u                                         # launch direction (x, z)
NRM = GEO.n
FEED_C = (GEO.E[0] * MM, GEO.E[1] * MM)           # feed-wheel centres (x, z) at y = +-FEED_Y
FEED_Y = GEO.feed_y * MM                          # 53
FEED_WHEEL = (SIM.feed_wheel_diameter * MM, SIM.feed_wheel_width * MM)   # 40 x 12
J_C = (GEO.j_center[0] * MM, GEO.j_center[1] * MM)
J_RHO = GEO.rho * MM                              # 40: ball-centre radius of the bend
J_END = (GEO.j_end[0] * MM, GEO.j_end[1] * MM)
NIP = (GEO.N[0] * MM, GEO.N[1] * MM)              # flywheel centres (x, z) at y = +-FW_Y
FW_R = GEO.fw_r * MM                              # 35
FW_W = SIM.flywheel_width * MM                    # 28
FW_Y = GEO.fw_y * MM                              # 67
FRONT_WALL_Z = GEO.front_wall_z * MM
LAUNCH_RUN = GEO.floor_len * MM
PIVOT_M = (GEO.pivot_m[0] * MM, GEO.pivot_m[1] * MM)     # (-10, 150) module frame
PIVOT_R = (GEO.pivot_r[0] * MM, GEO.pivot_r[1] * MM)     # (-10, 329.3) robot frame
TILT_RANGE = SIM.tilt_range_deg                    # (-30, 40)
YOKE_Y = 105.0                                      # the sim's pivot yokes
CAM_BASELINE = SIM.camera_baseline * MM            # 190 (the cameras themselves: head.py)

# --- vendor parts with drawings ------------------------------------------------------------------------------------------
POLOLU_25D = dict(d=25.0, gearbox=17.0, motor=30.8, encoder=14.7, shaft=(4.0, 3.5, 12.5), bushing=(7.0, 2.5),
                  holes=8.5)    # 9.7:1 HP 12 V + 48 CPR encoder (#4842): 2 x M3 at +-8.5 mm, 6 mm deep
HUB_1999 = dict(d=25.4, t=5.0, boss=(12.0, 4.2), pcd=19.0, n=6)   # Pololu 6 mm hub, 6 x M3
HUB_1997 = dict(d=19.0, t=5.0, pcd=12.7, n=4)                      # Pololu 4 mm hub, 4 x M3
F90 = dict(d=34.5, h=26.0, base_d=30.0, mount=19.0, shaft=5.0)    # T-Motor F90 2806.5 (VERIFY: 19 x 19 M3 assumed;
                                                                   # the base must reach past the holes: 30 est)
STS3250 = dict(l=45.2, w=24.7, h=35.0, horn_pcd=14.0)               # Feetech STS3250 (STS3215 case, VERIFY)
A2M12 = dict(d=76.0, h=41.0, base_h=24.0, head_d=70.0, scan_z=30.5, holes=(4, 2.5, 50.0))  # (VERIFY: hole circle, scan height)
JETSON = dict(l=103.0, w=90.5, h=34.8, holes=(86.0, 58.0))          # Orin Nano Super dev kit (VERIFY hole pattern)
BATTERY = dict(l=153.0, w=72.0, h=23.0)                             # 7S1P P50B flat row, wrapped (est)
BMS = (65.0, 45.0, 9.0)
BRG_61808 = (40.0, 52.0, 7.0)
BRG_623 = (3.0, 10.0, 4.0)


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


def module_to_robot_xz(x, z, tilt_deg=0.0):
    """MODULE (x, z) -> ROBOT (x, z) at a tilt (positive tips the funnel toward the thrower, as the sim)."""
    c, s = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
    dx, dz = x - PIVOT_M[0], z - PIVOT_M[1]
    return PIVOT_R[0] + c * dx + s * dz, PIVOT_R[1] - s * dx + c * dz
