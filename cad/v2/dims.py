"""CAD parameters (mm). Robot layout comes from the simulation so the two cannot drift.

Coordinate-frame contracts (every part is modeled in its own frame, then placed):
  ROBOT      origin on the floor under the base center; x forward (launcher side), y left, z up.
  PAYLOAD    origin at the payload-plate top surface center; axes parallel to ROBOT.
             Everything in the catch module is placed in this frame.
  POD i      origin on the wheel axle at the wheel center plane; x along the axle pointing
             inward (toward the robot center), z up. Wheel i sits at WHEEL_ANGLES[i].
  MOTOR      AK40-10 frame from the CubeMars STEP: axis = +Y, output face at y = -14.0,
             rear face at y = +26.2.

Fasteners: M3 heat-set inserts in printed parts unless noted (M3 x 5.7 mm OD x 4.6 mm).
Print materials: PETG for structure, PLA for low-load parts, TPU 95A for tires/bumpers.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation" / "v2 detailed physics"))

from params import SimParams  # noqa: E402
from robot_model import geometry  # noqa: E402

SIM = SimParams()
GEO = geometry(SIM)
MM = 1000.0

# --- printer -------------------------------------------------------------------
BED = 250.0                       # usable bed (Bambu X1/P1/A1 256 mm, keep margin)

# --- base layout (from the sim) -------------------------------------------------
WHEEL_R = SIM.wheel_radius * MM               # 48
WHEEL_W = SIM.omni.width * MM                 # 38 (goBILDA 96 mm omni, VERIFY width)
WHEEL_BASE_R = SIM.wheel_base_radius * MM     # 180: robot center to wheel center plane
WHEEL_ANGLES = SIM.wheel_angles_deg           # 60, 180, 300
DECK_Z = GEO.deck_z * MM                      # deck underside height (108)
DECK_T = 6.0
DECK_R = SIM.deck_radius * MM                 # 225
PLATE_TOP = GEO.plate_top * MM                # payload plate top (185)
PLATE_T = 6.0
PLATE_R = 120.0                               # payload plate radius (prints in one piece)
CENTER_R = 120.0                              # central chassis plate radius
STANDOFF_R = 100.0
STANDOFF_ANGLES = (0.0, 120.0, 240.0)         # between the wheel pods (pods at 60/180/300)

# --- AK40-10 V3.0 (CubeMars drawing 2026-06-12) -----------------------------------
AK_D = 53.0
AK_L = 40.2
AK_OUT_D = 37.0                  # rotating output flange, protrudes 1 mm
AK_OUT_PCD = 27.0                # 3 x M2.5 x 3 deep, at 30/150/270 deg
AK_OUT_SPIGOT = 15.0             # centering boss
AK_REAR_PCD = 47.0               # 4 x M2.5 x 5 deep at 0/90/180/270
AK_FRONT_PCD = 47.5              # 3 x M2.5 x 5 deep at 0/120/240
AK_GAP = 4.0                     # wheel inner face to motor output face

# --- goBILDA 96 mm omni (3624-0014-0096) ------------------------------------------
GB_BORE = 14.0
GB_PCD_OUTER = 32.0              # 8 x M4 clearance; we use 4 of them
GB_PCD_INNER = 16.0

# --- catch module (from the sim geometry, module frame) ----------------------------
RIM_Z = GEO.rim_z * MM
THROAT_Z = GEO.throat_z * MM
GATE_Z = GEO.gate_z * MM
HOLD_Z = GEO.hold[2] * MM
TUBE_W = GEO.w * MM              # inner half width of pocket / tube / J (39)
WALL = 3.0                       # printed wall thickness around the ball path
J_C = (GEO.C[0] * MM, GEO.C[1] * MM)
J_R = GEO.R_wall * MM            # J outer-wall inner radius (87)
J_RHO = GEO.rho * MM
THETA = GEO.theta                # launch angle (rad)
U = GEO.u                        # launch direction (x, z)
NRM = GEO.n                      # launch-line normal, up (x, z)
J_E = (GEO.E[0] * MM, GEO.E[1] * MM)
NIP = (GEO.N[0] * MM, GEO.N[1] * MM)
FW_R = GEO.fw_r * MM             # flywheel radius (35)
FW_W = SIM.flywheel_width * MM   # 28
FW_Y = GEO.fw_y * MM             # flywheel center |y| (67)
FUNNEL_R = SIM.funnel_rim_radius * MM
FUNNEL_THROAT_R = SIM.funnel_throat_radius * MM
FUNNEL_ANGLE = math.radians(SIM.funnel_wall_angle_deg)
BALL_R = GEO.rb * MM

# --- flywheel motor: T-Motor F90 2806.5 1300KV ------------------------------------
FM_D = 34.0                      # bell diameter (VERIFY)
FM_H = 26.0                      # height without shaft (VERIFY)
FM_MOUNT = 19.0                  # 4 x M3 on a 19 x 19 mm square (VERIFY; some batches 16 x 16)
FM_SHAFT = 5.0                   # M5 prop shaft

# --- gate servo (drives the trapdoor directly on its hinge axis) --------------------
SERVO = (40.7, 20.3, 38.6)       # standard-size servo body (e.g. DS3225MG, 25 kg cm)
SERVO_TAB = (54.5, 2.6)          # tab span, tab thickness
SERVO_HOLES = (49.5, 10.0)       # mounting-hole spacing (VERIFY for the chosen servo)


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)
