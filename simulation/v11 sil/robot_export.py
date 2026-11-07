"""v11 (design.md 30): the sim's robot, exported to the code that will run on it. One source, like cad/v7/dims.py:

  firmware/lib/omni_fw/fw_params.h            the MCU firmware's constants (kinematics, limits, flywheel loop, feed,
                                               tilt range, drive-motor CAN ids and ranges)
  ros/src/omni_catch/omni_catch/config/robot_v11.json
                                               what the robot computer needs: the funnel's rim and the launch exit against
                                               tilt, the launcher calibration, the ball, the cameras' extrinsics and
                                               intrinsics, the base's planning limits
  ros/src/omni_arm/omni_arm/config/robot_arm.json
                                               the arm robot (design.md 11, ARM below): the SO-101's chain as the sim
                                               builds it, its servos, finger pads, wrist camera, the stereo pods, the
                                               bin, the robot's keep-out volumes for the arm, the arm-out base caps
  ros/src/omni_arm/omni_arm/config/robot_arm_d435.json
                                               the same robot with a RealSense D435 on the arm riser's front as its
                                               depth camera (ARM_D435, design.md 4.11; the nose: ARM_D435_NOSE)

The robot is V11 below: the v7 base (design.md 25, adopted: 127 mm printed omni wheels, sprung pods), the v6 round 3
catch / launch module (sections 22-23: chute feed, padded pocket, 35 deg catch tilt) and section 24's actuators
(DM-2325 drive, Pololu 25D feed on a RoboClaw). Firmware adult limits as every catch study (2 m/s).

Run it after a change to params.py, catalog.py or the model:  python robot_export.py   (test_v11.py checks the
committed files are current).
"""

from __future__ import annotations

import json
import math
import os
import sys
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import arm_so101 as A  # noqa: E402
from omni_control import OmniKinematics  # noqa: E402
from params import SimParams  # noqa: E402
from robot import LauncherCal, cal_file  # noqa: E402
from robot_model import build_model, geometry  # noqa: E402
from vision import VisionConfig  # noqa: E402

# (ros/omni.sh mirrors both into WSL's ~/omni_ws and says where)
FW_PARAMS = Path(os.environ.get("OMNI_FIRMWARE_DIR", REPO / "firmware")) / "lib" / "omni_fw" / "fw_params.h"
# the arm robot's (cad/v10) wheel, base and motor: a second constants header, compiled into its own SIL library
# (firmware/sil/build_sil.py variant "arm"; sil_robot picks it by firmware_variant(p)). fw_params.h stays the catch robot's
FW_PARAMS_ARM = FW_PARAMS.with_name("fw_params_arm.h")
ROBOT_JSON = Path(os.environ.get("OMNI_ROS_SRC", REPO / "ros" / "src")) / "omni_catch" / "omni_catch" / "config" / \
    "robot_v11.json"
ARM_JSON = Path(os.environ.get("OMNI_ROS_SRC", REPO / "ros" / "src")) / "omni_arm" / "omni_arm" / "config" / \
    "robot_arm.json"

# the v11 robot: house.BASES["v7"] + shallow_sweep.R3 (v6 round 3 module) + design.md 24's feed motor
V11 = dict(wheel="Printed-127-2x5-36", suspension="pod", drive_motor="DM-2325",
           funnel="cone55", feed="chute", pocket_pad=True, feed_motor="Pololu-25D-RoboClaw",
           limits="firmware", base_max_speed=2.0, wheel_model="rollers",
           # the baseline as drawn (cad/v7, design.md 12.2; Fernando, Oct 25): the plate 96 mm over the deck, the
           # cameras on the head's bar at its catch index
           plate_gap=0.096, camera_mount="head")
# the flattest launch the firmware fires (design.md 12.2): below 24 deg the launch line (ball + 5 mm) hits the 5 in face
# between the cameras (cad/v7 checks.json "head"). The game's flattest return is 30 deg; this binds the Thrower bench mode
FIRE_MIN_ELEVATION_DEG = 25.0

# kid mode (design.md 14): hard caps the firmware applies whatever the robot computer asks (est, to be set in an adult-only
# session): 0.8 m/s, 60% of the acceleration table, half the jerk and yaw rate, gentle lobs only
KID = dict(max_speed=0.8, accel_scale=0.6, jerk_scale=0.5, yaw_scale=0.5, fly_rpm_max=2600.0)
ADULT_FLY_RPM_MAX = 6000.0      # est: above every calibrated shot (design.md 9: 10.1 m/s needs ~3950 rpm)

# Damiao DM-2325 on CAN (dm_motor.h). VERIFY all of these against the units flashed with Damiao's tool
DM = dict(ids=(1, 2, 3, 4), dirs=(1, 1, 1, 1), master_base=0x10, p_max=12.5, v_max=60.0, t_max=5.0)
FLY_POLE_PAIRS = 7              # F90 2806.5: 12N14P (est from the stator class; VERIFY)
# the arm robot's CubeMars AK40-10 V3.0 KV170 in servo mode (ak_motor.h): estimates and VERIFY items, see the header
AK = dict(pole_pairs=14.0, gear=10.0, max_erpm=60000.0, kt_out=0.48, sample_to_rx_us=150)
def wheel_speed_max(p) -> float:
    """The firmware's wheel-speed ceiling (rad/s): wheel_speed_frac of the motor's no-load speed, and for a CubeMars
    drive no more than its own ERPM limit allows (the motor clamps there: AK40-10, 60000 ERPM = 44.9 rad/s)."""
    w = p.wheel_speed_frac * p.motor.noload_rpm * 2 * math.pi / 60
    if p.drive_motor.startswith("AK"):
        w = min(w, AK["max_erpm"] / (AK["gear"] * AK["pole_pairs"]) * 2 * math.pi / 60)
    return w


DM_SAMPLE_TO_RX_US = 140        # design.md 10: reply frame ~125 us + turnaround (est); sil_robot.DmBus models it


def v11_params(**kw) -> SimParams:
    return SimParams(**{**V11, **kw})


# the drawn robot's mass (cad/v7/out/BOM.md "Mass": build.py's sum of every part, the module at tilt 0, ROBOT frame;
# test_v11 checks these against the BOM)
CAD_MASS_KG = 8.19
CAD_CG = (0.0180, -0.0014, 0.1246)


def _ballasted(p: SimParams, kg: float, cg) -> SimParams:
    """p with one point mass on the base that brings the robot's total and CG to the CAD's: what the CAD has and the sim
    doesn't (the head and face, heavier printed parts, fasteners, cables). The payload at its default pose."""
    from dataclasses import replace
    from robot_model import mass_properties
    mass, now = mass_properties(build_model(p), p, 0.0)
    dm = kg - mass
    if dm <= 0.0:
        raise ValueError(f"the sim's robot ({mass:.2f} kg) is already heavier than the CAD's")
    pos = (kg * np.array(cg) - mass * np.asarray(now)) / dm
    return replace(p, extra_masses=tuple(p.extra_masses) + ((round(dm, 4), *(round(float(x), 4) for x in pos)),))


def drawn_params(**kw) -> SimParams:
    """The v11 robot with the CAD's masses (design.md 7, step 1): mass_model "cad" (the 515 g printed wheels, no front
    rocker, the bumper ring at r 205-217 mm) and one point mass on the base that brings the total and the CG to the
    BOM's: what the CAD has and the sim doesn't (the head and face, heavier printed parts, fasteners, cables)."""
    return _ballasted(v11_params(mass_model="cad", **kw), CAD_MASS_KG, CAD_CG)


# the Oct 25 arm robot as cad/v7 drew it (design.md 12.3; cad/v7 arm_kit.py, studies/layout_trade.py A6): the baseline base
# and head with the arm kit in place of the catch module: the SO-101 on a 50 mm riser at the plate's front edge (its
# base over the camera bar), the bar at its arm index (40 deg down), the lidar at the front, the bin on the plate's
# rear, the face and the audio box over it. Kept for the layout studies and the earlier results; since Nov 1 the arm
# robot is ARM (cad/v10's coarse model, below)
ARM_V7 = dict(payload="arm", plate_gap=0.096, arm_mount=(0.14, 0.0, 0.0), arm_riser=0.05, arm_pick=True,
              arm_bin=(-0.07, 0.0, 0.12, 0.12, 0.07), arm_display="face", lidar_mount="front", camera_mount="head",
              camera_head_xz=(0.18266, 0.16573), camera_tilt_deg=-40.0)

# The arm robot as cad/v10 human editable draws it (Nov 1; the base is cad/v9's), the baseline since Nov 1 (Fernando):
# the sim's coarse 3D model of the CAD. The 140 mm 2 x 6 x 36 wheel (rows 72 mm apart) on r 146, the AK40-10 inside it,
# each wheel on a fork that pivots on a shaft beside it (80.6 mm from the axle, level with it: y 80.6, z 0), the axle -3 to
# +15 mm (18 mm travel, 12.9 deg), the deck and wheel guards out to r 217 with the TPU bumper 8 mm outside (450 x 453 mm), wheel
# slots 158 mm wide from r 85, the plate at 181.5 mm (cad/v9's upper parts 13 mm up: 6.5 mm of wheel radius and 6.5 mm
# of plate gap) and the lidar scan plane at 163, 140 mm from the centre, and Fernando's D435 as a box on the nose where
# design.md 4.11 puts it (an idealized spot, rides the deck: 6.5 mm up). Three things differ from the drawn CAD, for the
# arm to reach the floor (design.md 4.11, Nov 1): the arm kit is NOT lifted (riser 37 mm: the arm base stays at the v7
# robot's 218.5 mm; at the drawn 231.5 mm the SO-101 has no fingers-down reach to the floor at all) and the head is only
# HEAD_LIFT higher, not 13 mm (the window between the lidar's top and the adapter over the bar: 4 mm each side).
# Nov 2 (Fernando, from the motor study): the fork's pivot moves level with the axle, the same 80.6 mm arm. With the AK40
# on the fork, a pivot 43 mm up turned drive torque into hinge torque (~0.6 tau: torque jacking), so the wheels bounced
# and fought (2 m/s cruise 55 -> 41 W at 10 kg, 102 -> 62 W at 20 kg; forward start 2.6 -> 6.2 m/s^2 at 10 kg;
# studies/pivot_quick.py). Nov 2: cad/v10's fork is redrawn to match (CAD_POD_PIVOT, the drawn fork: the deck slotted at
# the pivots, a bridge over each side strip); its earlier pivot was (0.0682, 0.043). The pods stay opposite hands on opposite
# diagonals: one hand everywhere would lose forward start 6.2 -> 4.1 m/s^2 and slope start 28 -> 16-18 deg at 10 kg
# (studies/hands_quick.py, pod_hands).
CAD_POD_PIVOT = (0.0806, 0.0)
POD_PIVOT = (0.0806, 0.0)
V10 = dict(wheel="Printed-140-2x6-36-cad", drive_motor="AK40-10", wheel_base_radius=0.146,
           pod_travel=0.018, pod_droop=0.003, pod_pivot=POD_PIVOT,
           deck_radius=0.217, deck_cut_radius=0.085, deck_slot_half_width=0.079, bumper_radius=0.211,
           plate_gap=0.1025, lidar_x=0.14, lidar_scan_z=0.163, mass_model="cad",
           # Nov 1 (motor study): the v10 rollers are PLA cores turning on their M3 axle bolts, no bearings (est mu)
           roller_pin_mu=0.3, roller_pin_radius=0.0015,
           # and a smooth roller hull: 14 flats around hammered the floor (wheel loads 0-120 N, base 7% slow at 1 m/s)
           roller_mesh=(13, 64))
HEAD_LIFT = 0.005                # m: the camera bar and cheeks over the v7 robot's (cad/v9 has 0.013; see above)
# design.md 4.11 (Nov 4, Fernando): the D435 7.5 deg down, not 30, so it sees the room ahead (wall tags at 0.9-1.3 m,
# furniture, people) as well as the floor; its depth then sees no floor nearer than ~0.34 m, so the tidy's close look
# is taken D435_LOOK_BACK short of the sweet spot and the base creeps in (studies/d435_tilt.py: 7.5 cm needed, + 2.5)
D435_NOSE = dict(d435_pos=(0.19, 0.0, 0.1115), d435_tilt_deg=-7.5, arm_riser_pocket=0.0)
# design.md 4.11 (Nov 5, Fernando baselined cad/v13's mount): the D435 moved from the nose to a pocket in the arm riser's
# front, landscape, 30 deg down, under the adapter, its lower edge in a notch in the floor (portrait fits nowhere: clear of
# the arm it reaches below the lidar's plane). cad/v13: 11-18 mm from the arm in every pose and move, 15 mm over the scan
# plane, the floor seen from 0.31 m. The sim's riser block is cut back 10 mm at its front for it (3.7 mm behind the
# camera; the CAD's pocket runs back to x 0.153). The close look still steps back D435_LOOK_BACK (its floor starts at
# ~0.31 m, the sweet spot's distance)
D435_RISER = dict(d435_pos=(0.1908, 0.0, 0.193), d435_tilt_deg=-30.0, arm_riser_pocket=0.010)
D435_TILT = D435_RISER["d435_tilt_deg"]
D435_LOOK_BACK = 0.10
ARM = dict(ARM_V7, **V10, arm_riser=0.037, camera_head_xz=(ARM_V7["camera_head_xz"][0], ARM_V7["camera_head_xz"][1] + HEAD_LIFT),
           d435_box=True, **D435_RISER)
# cad/v9/out/ARM_KIT.md (the arm robot's CAD estimate, the SO-101 stowed; v10 keeps v9's base) plus the D435's 72 g
CAD_ARM_MASS_KG = 9.56 + 0.072
CAD_ARM_CG = (0.0066, 0.0, 0.1264)
# its fixed poses (rad, JOINTS order): a drop 10 deg off vertical over the bin's middle (layout_trade.py A6); a stow
# folded sideways over the plate's right half, 0.41 m up; high, the arm upright (the way between the stow, the work
# ahead and the bin); predrop, folded back low before the drop (the face over the bin is in the way of a straight
# swing back). Each straight joint path stow-high, high-ready, high-predrop, predrop-drop clears the drawn robot by
# >= 9.8 mm with the SO-101's own meshes (and omni_arm's keep-outs)
ARM_POSES = dict(stow=A.POSES["kit_stow"],
                 high=(0.0, -0.218, -1.268, -0.207, 0.0, -0.17),
                 predrop=(0.0, -1.745, 0.634, -1.658, 0.0, -0.17),
                 drop=(0.0, -1.3265, -1.5548, -1.6581, 0.0, -0.17))
ARM_SWEET = (0.31, 0.0)          # m, base_link: the middle of the arm kit's fingers-down floor reach (0.26-0.36 m)
# the head and the arm kit as the arm's planner sees them (cad/v7 at the arm index, boxes round each part, m)
ARM_HEAD_KEEP_OUT = (("the camera bar", (0.1409, -0.114, 0.1809), (0.1691, 0.114, 0.2091)),
                     ("the left camera and its holder", (0.138, 0.0645, 0.1555), (0.193, 0.117, 0.212)),
                     ("the right camera and its holder", (0.138, -0.117, 0.1555), (0.193, -0.0645, 0.212)),
                     ("the left cheek", (0.084, 0.064, 0.158), (0.1957, 0.072, 0.2492)),
                     ("the right cheek", (0.084, -0.072, 0.158), (0.1957, -0.064, 0.2492)),
                     ("the arm's adapter", (0.062, -0.056, 0.1685), (0.205, 0.056, 0.2221)))
# the robot computer's "arm out" request (design.md 11): while the tool is outside the bumper or above ARM_OUT_Z, the base runs
# at half the acceleration table's straight-ahead value and at most 1 m/s (CMD a_cap / speed_cap)
ARM_OUT = dict(speed_cap=1.0, a_cap_frac=0.5, out_radius=0.217, out_z=0.40)
# the six STS3215s on the robot computer's USB bus board (Waveshare bus servo adapter, 1 Mbaud): ids as LeRobot's SO-101
ARM_SERVO_IDS = (1, 2, 3, 4, 5, 6)


def sprung_mass(p: SimParams, total: float) -> float:
    """kg the pods carry: the robot's total less each wheel, its motor and its pod (fork, shaft, shocks)."""
    unsprung = (p.wheel_mass if p.wheel_mass is not None else p.omni.mass) + p.motor.mass + p.m_pod + p.m_pod_suspension
    return total - p.n_wheels * unsprung


@lru_cache(maxsize=None)
def arm_ballast(depth_camera: str = "none", kg: float = CAD_ARM_MASS_KG, cg_dz: float = 0.0,
                drive_motor: str | None = None) -> tuple:
    """The point mass that brings the arm robot to the CAD's total and CG (one entry for params.extra_masses), or to
    kg with the CG cg_dz higher (the motor study's heavier robots: the extra mass is payload on the plate)."""
    kw = {**ARM, "depth_camera": depth_camera}
    if drive_motor is not None:
        kw["drive_motor"] = drive_motor
    cg = (CAD_ARM_CG[0], CAD_ARM_CG[1], CAD_ARM_CG[2] + cg_dz)
    return _ballasted(v11_params(**kw), kg, cg).extra_masses


def arm_robot(camera: str = "stereo", mass_kg: float | None = None, cg_dz: float = 0.0,
              drive_motor: str | None = None) -> dict:
    """ARM (camera "stereo": the head's pair gives the depth) or ARM_D435 (the D435 on the riser does) as SimParams
    overrides, with the CAD's mass and CG (or mass_kg, CG cg_dz higher) and the pods' damping set for that mass.
    drive_motor swaps the AK40-10s (the ballast keeps the total)."""
    base = dict(ARM_D435 if camera == "d435" else ARM)
    if drive_motor is not None:
        base["drive_motor"] = drive_motor
    kg = CAD_ARM_MASS_KG if mass_kg is None else float(mass_kg)
    extra = arm_ballast(base.get("depth_camera", "none"), kg, cg_dz, drive_motor)
    return dict(base, extra_masses=extra, pod_sprung_mass=round(sprung_mass(v11_params(**base), kg), 3))


def arm_params(mass_kg: float | None = None, cg_dz: float = 0.0, **kw) -> SimParams:
    """The arm robot (ARM), with the CAD's total mass and CG unless extra_masses is given; mass_kg / cg_dz / a
    drive_motor in kw give the motor study's variants."""
    return v11_params(**{**arm_robot("stereo", mass_kg, cg_dz, kw.get("drive_motor")), **kw})


# design.md 4.11 (Nov 1; the riser since Nov 5): the arm robot with Fernando's RealSense D435 as its floor camera
# (and Nav2's near-field depth), the stereo pair not used: robot_arm_d435.json; ROS: omni.sh arm arm_camera:=d435
ARM_D435 = dict(ARM, depth_camera="d435")
ARM_D435_NOSE = dict(ARM, depth_camera="d435", **D435_NOSE)            # Nov 4-5: on the nose, 7.5 deg down (earlier results)
ARM_D435_JSON = ARM_JSON.with_name("robot_arm_d435.json")


def arm_d435_params(**kw) -> SimParams:
    return v11_params(**{**arm_robot("d435", drive_motor=kw.get("drive_motor")), **kw})


def _c(v) -> str:
    s = f"{float(v):.9g}"
    return (s if any(ch in s for ch in ".e") else s + ".0") + "f"


def firmware_variant(p: SimParams) -> str:
    """Which compiled-in constants header the firmware in the loop needs for p: "arm" (fw_params_arm.h) when p has the arm
    robot's wheels, base radius and motor (cad/v10), else "" (fw_params.h: the catch robot's). The constants (wheel radius,
    the kinematics matrix, the wheel speed limit) are compile-time in the firmware, as on the Teensy."""
    a = ARM
    same = (p.wheel == a["wheel"] and abs(p.wheel_base_radius - a["wheel_base_radius"]) < 1e-9
            and p.drive_motor == a["drive_motor"])
    return "arm" if same else ""


def study_firmware(p: SimParams, tag: str, robot: dict | None = None) -> str:
    """A simulation study's firmware variant (Nov 1 motor study): p's constants (motor, wheel, limits) in a header under
    firmware/sil/build/headers (never lib/omni_fw: not for hardware), built as the SIL library variant `tag`. Returns
    tag, for SimParams.fw_variant."""
    sys.path.insert(0, str(REPO / "firmware" / "sil"))
    import build_sil
    path = build_sil.STUDY_HEADERS / f"fw_params_{tag}.h"
    text = firmware_header(p, robot if robot is not None else {"study": tag})
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text() != text:
        path.write_text(text)
    build_sil.build(variant=tag)
    return tag


def study_limits(p: SimParams, speed: float, accel_scale: float = 1.0) -> SimParams:
    """p with the firmware's ADULT speed cap at `speed` and its acceleration table, acceleration, jerk and yaw limits x
    accel_scale (the Nov 1 motor study's profiles: where the firmware stops being the limit). Not adopted defaults."""
    dirs, vals = p.accel_table
    return p.with_(base_max_speed=speed, accel_table=(tuple(dirs), tuple(v * accel_scale for v in vals)),
                   base_max_accel=p.base_max_accel * accel_scale, base_max_jerk=p.base_max_jerk * accel_scale,
                   base_max_yaw_rate=p.base_max_yaw_rate * accel_scale,
                   base_max_yaw_accel=p.base_max_yaw_accel * accel_scale)


def study_from_env(p: SimParams, camera: str = "stereo") -> SimParams:
    """The ROS sim bridge's study hook (Nov 1 motor study, phase 6): OMNI_SIM_STUDY='{"motor": "AK40-10", "mass": 10,
    "speed": 3.0, "accel_scale": 1.5}' swaps the arm robot's drive motor and mass and builds the firmware with those
    limits (study_firmware). Unset: p unchanged. Not a launch option: a study's environment only."""
    raw = os.environ.get("OMNI_SIM_STUDY")
    if not raw or p.payload != "arm":
        return p
    st = json.loads(raw)
    motor = st.get("motor", p.drive_motor)
    kg = float(st.get("mass", CAD_ARM_MASS_KG))
    k = sprung_mass(p, kg) / sprung_mass(p, 10.0)
    p = p.with_(**arm_robot(camera, kg, float(st.get("cg_dz", 0.0)), motor))
    p = p.with_(pod_rate=p.pod_rate * k, pod_preload=p.pod_preload * k)
    p = study_limits(p, float(st.get("speed", p.base_max_speed)), float(st.get("accel_scale", 1.0)))
    import re
    tag = "ros_" + re.sub(r"[^A-Za-z0-9]", "_", f"{motor}_{st.get('speed')}_{st.get('accel_scale', 1.0)}")
    return p.with_(fw_variant=study_firmware(p, tag))


def firmware_header(p: SimParams, robot: dict | None = None) -> str:
    kin = OmniKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius)
    w_max = wheel_speed_max(p)
    dirs, vals = p.accel_table if p.accel_table is not None else ((), ())
    adult = (p.base_max_speed, p.base_max_accel, p.base_max_jerk, p.base_max_yaw_rate, p.base_max_yaw_accel,
             p.brake_margin, 1.0, ADULT_FLY_RPM_MAX)
    kid = (KID["max_speed"], p.base_max_accel * KID["accel_scale"], p.base_max_jerk * KID["jerk_scale"],
           p.base_max_yaw_rate * KID["yaw_scale"], p.base_max_yaw_accel * KID["yaw_scale"], p.brake_margin,
           KID["accel_scale"], KID["fly_rpm_max"])
    fm = p.fly_motor
    ke = 1 / (fm.kv * 2 * math.pi / 60)
    rows = lambda M: ", ".join("{" + ", ".join(_c(x) for x in r) + "}" for r in M)      # noqa: E731
    L = [
        "// GENERATED by simulation/v11 sil/robot_export.py from the sim's SimParams: do not edit by hand.",
        f"// Robot: {json.dumps(V11 if robot is None else robot)}",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        "namespace fw {",
        "namespace prm {",
        "",
        "constexpr float TICK_S = 0.001f;                 // the firmware's 1 kHz tick (design.md 4)",
        "constexpr uint64_t STATE_PERIOD_US = 5000;       // STATE to the robot computer at 200 Hz",
        "constexpr float CMD_TIMEOUT = 0.5f;              // s without a CMD -> ramped stop (est)",
        "constexpr float DM_FEEDBACK_TIMEOUT = 0.05f;     // s without a drive motor reply -> fault",
        f"constexpr float BATT_LOW = {_c(p.pack.v_low_cell * p.pack.series)};          // V: {p.pack.v_low_cell:g} V per cell (catalog pack setting)",
        "constexpr float MANUAL_SPEED = 1.0f;             // m/s cap on the RC sticks (est)",
        f"constexpr bool HAS_BUMPERS = {'false' if p.payload == 'arm' else 'true'};{' ' * (19 if p.payload == 'arm' else 20)}// bumper switches wired (the arm robot has none yet: todo.md); without them the inputs are ignored",
        f"constexpr bool DRIVE_DAMIAO = {'true' if p.drive_motor.startswith('DM') else 'false'};{' ' * (18 if p.drive_motor.startswith('DM') else 17)}// {p.drive_motor}: {'Damiao velocity mode (dm_motor.h)' if p.drive_motor.startswith('DM') else 'CubeMars servo mode (ak_motor.h)'}",
        "",
        "// time references (design.md 10): SYNC_OUT to the robot computer's GPIO, CAM_TRIG to both cameras' FSIN",
        "constexpr uint64_t SYNC_PERIOD_US = 1000000;     // one pulse a second, on whole seconds of MCU time",
        f"constexpr float CAM_TRIGGER_HZ = {_c(VisionConfig().fps)};        // the stereo pair's frame rate",
        "// a DM-2325 reply's interrupt minus this = when the motor sampled its position: the 8-byte reply on the wire",
        "// at 1 Mbit/s (~125 us with stuffing) plus the motor's turnaround (est 15 us; VERIFY with a scope on CAN H)",
        f"constexpr uint64_t DM_SAMPLE_TO_RX_US = {DM_SAMPLE_TO_RX_US};",
        "",
        "// kinematics: X4 omni, wheel speed (rad/s) = J (vx, vy, wz); odometry by its pseudo-inverse",
        f"constexpr int N_WHEELS = {p.n_wheels};",
        f"constexpr float WHEEL_RADIUS = {_c(p.wheel_radius)};",
        f"constexpr float J[{p.n_wheels}][3] = {{{rows(kin.J)}}};",
        f"constexpr float J_PINV[3][{p.n_wheels}] = {{{rows(kin.J_inv)}}};",
        f"constexpr float MAX_WHEEL_SPEED = {_c(w_max)};   // rad/s: {p.wheel_speed_frac} x the {p.drive_motor}'s no-load speed"
        + (f", capped by its {AK['max_erpm']:g} ERPM limit" if p.drive_motor.startswith("AK") else ""),
        "",
        "// limits by profile: max_speed, max_accel, max_jerk, max_yaw_rate, max_yaw_accel, brake_margin, accel_scale,",
        "// fly_rpm_max. The robot computer can only ask for less (CMD speed_cap)",
        f"constexpr float ADULT_LIMITS[8] = {{{', '.join(_c(x) for x in adult)}}};",
        f"constexpr float KID_LIMITS[8] = {{{', '.join(_c(x) for x in kid)}}};   // est (design.md 14)",
        "// direction-aware acceleration limit, body frame, 0 = forward, mirrored left / right (m/s^2)",
        f"constexpr int ACCEL_N = {len(dirs)};",
        f"constexpr float ACCEL_DIRS[{max(1, len(dirs))}] = {{{', '.join(_c(x) for x in dirs) or '0'}}};",
        f"constexpr float ACCEL_VALS[{max(1, len(vals))}] = {{{', '.join(_c(x) for x in vals) or '0'}}};",
        "",
        f"// flywheels: {fm.name} on the {p.esc_spec.name}; rpm PI with back-EMF feed-forward (models.FlywheelDrive)",
        f"constexpr float FLY_KE = {_c(ke)};",
        f"constexpr float FLY_RM = {_c(fm.r_m)};",
        f"constexpr float FLY_KT = {_c(fm.kt)};",
        f"constexpr float FLY_I0 = {_c(fm.i0)};",
        f"constexpr float FLY_W10V = {_c(10 / ke)};",
        "constexpr float FLY_KP = 0.004f;",
        "constexpr float FLY_KI = 0.08f;",
        f"constexpr int FLY_POLE_PAIRS = {FLY_POLE_PAIRS};             // VERIFY",
        f"constexpr bool FLY_BRAKE = {'true' if p.feed == 'kicker' else 'false'};",
        "constexpr float FLY_READY_TOL = 0.03f;          // the fire interlock (the robot computer asks for 2%)",
        "",
        f"// feed: {p.feed} pocket, {p.feed_spec.name}",
        f"constexpr bool FEED_CHUTE = {'true' if p.feed == 'chute' else 'false'};",
        f"constexpr float FEED_SPEED = {_c(p.gate_open_target)};       // rad/s at the feed wheels ({p.feed_speed} m/s)",
        f"constexpr float FEED_LINEAR_SPEED = {_c(p.feed_speed)};",
        f"constexpr float FEED_SLEW = {_c(p.gate_slew)};",
        f"constexpr float FEED_TIME = {_c(p.feed_time)};",
        f"constexpr float FEED_SEAT_TIME = {_c(p.feed_seat_time)};",
        f"constexpr float FEED_CLEAR_TRAVEL = {_c(p.feed_clear_travel)};",
        "",
        "// tilt servo range (rad)",
        f"constexpr float TILT_MIN = {_c(p.tilt_limits[0])};",
        f"constexpr float TILT_MAX = {_c(p.tilt_limits[1])};",
        f"// the fire interlock's tilt: elevation = {p.launch_angle_deg:g} deg - tilt, the flattest fired "
        f"{FIRE_MIN_ELEVATION_DEG:g} deg (the face)",
        f"constexpr float FIRE_TILT_MAX = {_c(math.radians(p.launch_angle_deg - FIRE_MIN_ELEVATION_DEG))};",
        "",
        "// Damiao drive motors on CAN1 (dm_motor.h): ids, mounting direction, feedback id = DM_MASTER_BASE + id,",
        "// the feedback's ranges (VERIFY: must match what is flashed into each motor)",
        f"constexpr uint8_t DM_IDS[{p.n_wheels}] = {{{', '.join(str(i) for i in DM['ids'])}}};",
        f"constexpr float DM_DIR[{p.n_wheels}] = {{{', '.join(_c(d) for d in DM['dirs'])}}};",
        f"constexpr uint32_t DM_MASTER_BASE = 0x{DM['master_base']:X};",
        f"constexpr float DM_PMAX = {_c(DM['p_max'])};",
        f"constexpr float DM_VMAX = {_c(DM['v_max'])};",
        f"constexpr float DM_TMAX = {_c(DM['t_max'])};",
        "// CubeMars drive motors (ak_motor.h, servo mode; the ids and directions above): ERPM = output rpm x gear x pole",
        "// pairs (14 pole pairs: third-party listings of the AK40-10 V3.0, as the AK45-10's 14 measured on the wheeled-leg",
        "// robot: VERIFY), the motor's own ERPM limit (l_max_erpm, 60000 at the factory: VERIFY), output torque per amp",
        "// (rated 1.3 N m at 2.7 A, est) and a status frame's interrupt less when it was sampled (an extended 8-byte frame",
        "// ~150 us at 1 Mbit/s, est)",
        f"constexpr float AK_POLE_PAIRS = {_c(AK['pole_pairs'])};",
        f"constexpr float AK_GEAR = {_c(AK['gear'])};",
        f"constexpr float AK_MAX_ERPM = {_c(AK['max_erpm'])};",
        f"constexpr float AK_KT = {_c(AK['kt_out'])};",
        f"constexpr uint64_t AK_SAMPLE_TO_RX_US = {AK['sample_to_rx_us']};",
        "",
        "}  // namespace prm",
        "}  // namespace fw",
        "",
    ]
    return "\n".join(L)


def _quat_xyzw(R) -> list:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, dtype=float).reshape(-1))
    if q[0] < 0:
        q = -q
    return [float(q[1]), float(q[2]), float(q[3]), float(q[0])]


# REP-103: an optical frame has z forward (the view), x right, y down; MuJoCo's camera looks along -z with y up
MJCAM_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])
# the camera's body frame (x forward, y left, z up) -> its optical frame
BODY_TO_OPTICAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]).T


def frames(m, d, p) -> list:
    """The robot's fixed frames (design.md 10), each {parent, child, xyz, quat (x y z w)}: what robot_frames publishes
    on /tf_static. base_link is the base body: the floor under the wheel circle's centre, x forward, z up (REP-103).
    The CAD's positions replace the sim's when the sim takes them (design.md 7, next step 1); calibration results
    (omni_calib) override each within its bounds."""
    base = m.body("base").id
    Rb, pb = d.xmat[base].reshape(3, 3), d.xpos[base]
    out = []

    def add(parent, child, pos, R, note):
        out.append(dict(parent=parent, child=child, xyz=[float(x) for x in pos], quat=_quat_xyzw(R), note=note))

    for child, site, note in (("laser", "lidar", "RPLIDAR A2M12: scan plane origin, x = its 0 deg"),
                              ("imu_link", "imu", "BNO085 on the MCU board, axes as mounted")):
        sid = m.site(site).id
        add("base_link", child, Rb.T @ (d.site_xpos[sid] - pb), Rb.T @ d.site_xmat[sid].reshape(3, 3), note)
    for name in ("left", "right"):
        cid = m.camera(f"stereo_{name}").id
        R_opt = Rb.T @ d.cam_xmat[cid].reshape(3, 3) @ MJCAM_TO_OPTICAL
        R_body = R_opt @ BODY_TO_OPTICAL.T
        add("base_link", f"cam_{name}", Rb.T @ (d.cam_xpos[cid] - pb), R_body,
            f"OV9782 {name}: camera body frame (x along the view, REP-103)")
        add(f"cam_{name}", f"cam_{name}_optical", (0.0, 0.0, 0.0), BODY_TO_OPTICAL,
            "optical frame (z forward, x right, y down): images and intrinsics")
    if p.depth_camera == "d435":
        # realsense2_camera's names (camera_name d435): d435_link at the depth origin (the left imager), x forward;
        # on the robot base_link -> d435_link comes from here and the driver publishes the rest
        cid = m.camera("d435_depth").id
        R_body = Rb.T @ d.cam_xmat[cid].reshape(3, 3) @ MJCAM_TO_OPTICAL @ BODY_TO_OPTICAL.T
        add("base_link", "d435_link", Rb.T @ (d.cam_xpos[cid] - pb), R_body,
            "RealSense D435 (design.md 4.11): the depth origin (left imager), x along the view")
        add("d435_link", "d435_depth_optical_frame", (0.0, 0.0, 0.0), BODY_TO_OPTICAL,
            "its depth image's optical frame (z forward, x right, y down)")
        cc = m.camera("d435_color").id                 # (parallel to the depth's: the factory extrinsic replaces it)
        add("d435_link", "d435_color_optical_frame", R_body.T @ Rb.T @ (d.cam_xpos[cc] - d.cam_xpos[cid]),
            BODY_TO_OPTICAL, "its colour image's optical frame (the RGB imager, 15 mm left of the depth origin): "
                             "the wall tags' camera")
    if getattr(p, "front_camera", "none") == "imx662":
        cid = m.camera("imx662").id
        R_body = Rb.T @ d.cam_xmat[cid].reshape(3, 3) @ MJCAM_TO_OPTICAL @ BODY_TO_OPTICAL.T
        add("base_link", "front_cam", Rb.T @ (d.cam_xpos[cid] - pb), R_body,
            "forward IMX662 (design.md 4.12, proposal): its entrance pupil, x along the view (20 deg up)")
        add("front_cam", "front_cam_optical", (0.0, 0.0, 0.0), BODY_TO_OPTICAL,
            "its image's optical frame (z forward, x right, y down): people, tags")
    jid = m.joint("tilt").id if any(m.joint(i).name == "tilt" for i in range(m.njnt)) else None
    if jid is not None:
        add("base_link", "module_pivot", Rb.T @ (d.xanchor[jid] - pb), np.eye(3),
            "the module's tilt axis at tilt 0 (y axis); the tilting module hangs off it (the tilt from /mcu/state)")
    g = geometry(p)
    if p.payload == "arm":
        b = m.body("arm_mount").id
        add("base_link", "arm_mount", Rb.T @ (d.xpos[b] - pb), Rb.T @ d.xmat[b].reshape(3, 3),
            "the SO-101's adapter: its base frame hangs off it (omni_arm publishes the arm's links from here)")
    else:
        add("base_link", "arm_mount", (0.0, 0.0, float(g.plate_top)), np.eye(3),
            "payload plate top centre: where a top module or an arm bolts on (its own frames hang off it)")
    return out


def robot_json(p: SimParams) -> dict:
    g = geometry(p)
    lo, hi = p.tilt_limits
    tilts = np.radians(np.arange(math.degrees(lo), math.degrees(hi) + 0.01, 1.0))
    exit_m = (g.N[0] + 0.06 * g.u[0], g.N[1] + 0.06 * g.u[1])        # as catch_game.CatchGame.exit_m
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    base = m.body("base").id
    Rb = d.xmat[base].reshape(3, 3)
    cams = []
    for name in ("left", "right"):
        cid = m.camera(f"stereo_{name}").id
        pos = Rb.T @ (d.cam_xpos[cid] - d.xpos[base])
        R = Rb.T @ d.cam_xmat[cid].reshape(3, 3)
        cams.append(dict(name=name, pos=[float(x) for x in pos], R=[[float(x) for x in r] for r in R],
                         fovy_deg=p.camera_fovy_deg, width=1280, height=800))
    cal = LauncherCal(p)
    kin = OmniKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius)
    b = p.ball_spec
    return dict(
        generated_by="simulation/v11 sil/robot_export.py", robot=V11,
        base=dict(base_link_height=float(d.xpos[base][2]), robot_radius=0.21, wheel_radius=p.wheel_radius,
                  J=kin.J.tolist(), max_wheel_speed=wheel_speed_max(p),
                  max_speed=p.base_max_speed, brake_margin=p.brake_margin,
                  accel_table=[list(p.accel_table[0]), list(p.accel_table[1])] if p.accel_table else None,
                  max_accel=p.base_max_accel, kid=KID),
        module=dict(tilt_limits=[lo, hi], tilt_deg=[float(math.degrees(t)) for t in tilts],
                    rim=[[float(x) for x in g.rim_center_robot(t)] for t in tilts],
                    exit=[[float(x) for x in g.module_to_robot(exit_m, t)] for t in tilts],
                    funnel_rim_radius=p.funnel_rim_radius, launch_angle_deg=p.launch_angle_deg,
                    fire_min_elevation_deg=FIRE_MIN_ELEVATION_DEG),
        launcher=dict(source=cal_file(p).name, calibrated=bool(cal.calibrated),
                      tilt_rad=[float(t) for t in cal.tilts], w=[list(map(float, w)) for w in cal.w],
                      v=[list(map(float, v)) for v in cal.v], d_elev=[list(map(float, e)) for e in cal.d_elev]),
        ball=dict(mass=b.mass, radius=b.radius, drag_cd=b.drag_cd, air_density=p.air_density,
                  air_viscosity=p.air_viscosity),
        cameras=cams,
        frames=frames(m, d, p),
    )


def _wxyz(q) -> list:
    return [float(v) for v in q]


def arm_json(p: SimParams, robot: dict = ARM) -> dict:
    """The arm robot for omni_arm (kinematics.ArmConfig): every transform as the sim's model has it (MuJoCo's body
    frames: pos + quaternion w x y z in the parent body; each joint turns its body about its axis through the body's
    origin), so the robot computer's kinematics are the sim's to the bit. cameras[0] is the depth image's camera (the tidy's
    looks): the stereo pair's left, or the D435 when the robot has one (design.md 4.11)."""
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    g = geometry(p)
    chain = []
    for b in ["arm_mount", "arm_base", "arm_shoulder", "arm_upper_arm", "arm_lower_arm", "arm_wrist", "arm_gripper"]:
        bid = m.body(b).id
        jn = [j for j in range(m.njnt) if m.jnt_bodyid[j] == bid]
        joint = None
        if jn:
            j = jn[0]
            joint = dict(name=A.JOINTS[[A.name(n) for n in A.JOINTS].index(m.joint(j).name)],
                         axis=[float(v) for v in m.jnt_axis[j]], pos=[float(v) for v in m.jnt_pos[j]],
                         range=[float(v) for v in m.jnt_range[j]])
        chain.append(dict(body=b, pos=[float(v) for v in m.body_pos[bid]], quat=_wxyz(m.body_quat[bid]), joint=joint))
    jaw = m.body("arm_jaw").id
    jj = m.joint("arm_gripper").id
    tool = m.site("arm_tool").id
    cam_body = m.body("arm_wrist_cam").id
    cid = m.camera("arm_wrist").id
    caps = []
    for gi in range(m.ngeom):
        nm = m.geom(gi).name
        # (the base's own capsule sits on the adapter: fixed to the robot, not checked)
        if nm.startswith("arm_") and ("_link" in nm or nm.startswith("arm_tip")) and not nm.startswith("arm_base_"):
            body = m.body(m.geom_bodyid[gi]).name
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.geom_quat[gi])
            ax = R.reshape(3, 3)[:, 2]
            c, h = m.geom_pos[gi], float(m.geom_size[gi][1]) if m.geom_type[gi] == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.0
            caps.append(dict(name=nm, body=body, a=[float(v) for v in c - h * ax], b=[float(v) for v in c + h * ax],
                             r=float(m.geom_size[gi][0])))
    cams = []
    base = m.body("base").id
    for name in ("left", "right"):
        c = m.camera(f"stereo_{name}").id
        cams.append(dict(name=name, pos=[float(v) for v in d.cam_xpos[c] - d.xpos[base]],
                         R=[[float(v) for v in r] for r in d.cam_xmat[c].reshape(3, 3)], fovy_deg=p.camera_fovy_deg,
                         width=1280, height=800, baseline=p.camera_baseline))
    if p.depth_camera == "d435":
        from robot_model import D435, d435_view
        c = m.camera("d435_depth").id
        w, h, fovy, _ = d435_view(p)
        cams.insert(0, dict(name="d435", pos=[float(v) for v in d.cam_xpos[c] - d.xpos[base]],
                            R=[[float(v) for v in r] for r in d.cam_xmat[c].reshape(3, 3)],
                            fovy_deg=fovy, width=w, height=h, baseline=D435["baseline"],
                            min_z=round(D435["min_z_848"] * max(w, h) / 848, 4), frame="d435_depth_optical_frame",
                            note="RealSense D435 depth (design.md 4.11); min_z scaled from the datasheet's 848 px (VERIFY)"))
    mx, my, _ = p.arm_mount
    mz = A.mount_z(p, g.plate_top)
    bx, by, L, W, H = p.arm_bin
    keep_out = [
        dict(kind="floor", z=0.002, note="every link's capsule stays this high (the fingertips' 4 mm spheres included)"),
        dict(kind="ring", r0=0.19, r1=0.235, z1=0.085, note="the bumper ring and the wheel guards (r 0.205-0.217 in CAD)"),
        dict(kind="cylinder", r=0.20, z1=float(g.plate_top + 0.005), x_max=float(mx - 0.07),
             note="the plate and what is under it, behind the nose bracket"),
        # ahead of that: under the nose bracket's adapter; on the plate (the arm kit's riser) the plate's top, as the
        # riser, the adapter and the head are boxes of their own
        dict(kind="cylinder", r=0.19, z1=float((mz - 0.01) if p.arm_mount_z is not None else g.plate_top + 0.005),
             x_min=float(mx - 0.07), note="the deck and the bracket under the arm's adapter"),
        dict(kind="box", min=[bx - L / 2, by - W / 2, float(g.plate_top)], max=[bx + L / 2, by + W / 2, float(g.plate_top + H + 0.005)],
             note="the bin: from above only"),
    ]
    for nm, (dx0, dx1), (dy0, dy1), (dz0, dz1) in A.rear_boxes(p, g.plate_top):
        keep_out.append(dict(kind="box", min=[dx0 - 0.005, dy0 - 0.005, dz0], max=[dx1 + 0.005, dy1 + 0.005, dz1 + 0.005],
                             note=nm.replace("arm_", "the ").replace("_", " ")))
    if p.camera_mount == "head":                      # the head and the kit's riser (cad/v7)
        up = p.camera_head_xz[1] - ARM_V7["camera_head_xz"][1]    # the head's lift over cad/v7 (the arm kit's adapter stays)
        for note, lo, hi in ARM_HEAD_KEEP_OUT:
            u = 0.0 if "adapter" in note else up
            keep_out.append(dict(kind="box", min=[lo[0], lo[1], lo[2] + u], max=[hi[0], hi[1], hi[2] + u], note=note))
    else:
        for c in cams:                                # each camera pod, a 4 cm block
            x, y, z = c["pos"]
            keep_out.append(dict(kind="box", min=[x - 0.03, y - 0.02, 0.0], max=[x + 0.01, y + 0.02, z + 0.02],
                                 note=f"the {c['name']} camera pod"))
    if p.depth_camera == "d435":                      # its body and bracket, a box round it on the deck's nose
        gid = m.geom("d435").id
        R, c0, hs = d.geom_xmat[gid].reshape(3, 3), d.geom_xpos[gid] - d.xpos[base], m.geom_size[gid]
        signs = np.array([(sx, sy, sz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        corners = c0 + (signs * hs) @ R.T
        keep_out.append(dict(kind="box", min=[float(v) for v in corners.min(axis=0) - 0.005],
                             max=[float(v) for v in corners.max(axis=0) + 0.005], note="the D435"))
    # the depth camera's nearest floor straight ahead (its field's lower edge), + 5 cm for a thing's ring of floor: the
    # tidy expects to see a known thing only beyond it (and never nearer than the old 0.32 m)
    c0 = cams[0]
    fwd = -np.array(c0["R"])[:, 2]
    low = math.asin(-fwd[2]) + math.radians(c0["fovy_deg"]) / 2
    floor_near = c0["pos"][0] + (c0["pos"][2] + float(d.xpos[base][2])) / math.tan(low)
    tbl = p.accel_table
    a_ahead = float(tbl[1][0]) if tbl else float(p.base_max_accel)
    return dict(
        generated_by="simulation/v11 sil/robot_export.py", robot={k: (list(v) if isinstance(v, tuple) else v)
                                                                  for k, v in robot.items()},
        base=dict(base_link_height=float(d.xpos[base][2]), robot_radius=0.217, max_speed=p.base_max_speed,
                  a_ahead=a_ahead, J=OmniKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius).J.tolist()),
        arm_out=dict(speed_cap=ARM_OUT["speed_cap"], a_cap=ARM_OUT["a_cap_frac"] * a_ahead, out_radius=ARM_OUT["out_radius"],
                     out_z=ARM_OUT["out_z"]),
        arm=dict(
            joints=list(A.JOINTS), chain=chain,
            tool=dict(body="arm_gripper", pos=[float(v) for v in m.site_pos[tool]], quat=_wxyz(m.site_quat[tool]),
                      note="the fixed finger's inner face, 6 mm above its tip; x = along the fingers, z = toward the moving jaw"),
            jaw=dict(body="arm_gripper", pos=[float(v) for v in m.body_pos[jaw]], quat=_wxyz(m.body_quat[jaw]),
                     axis=[float(v) for v in m.jnt_axis[jj]], range=[float(v) for v in m.jnt_range[jj]]),
            pads=dict(fixed=[list(A.FIXED_PAD[0]), list(A.FIXED_PAD[1])], moving=[list(A.MOVING_PAD[0]), list(A.MOVING_PAD[1])],
                      fingertip_below_tool=0.006, gripper_closed=A.GRIPPER_CLOSED, gripper_open=A.GRIPPER_OPEN),
            wrist_cam=dict(body="arm_gripper", pos=[float(v) for v in m.body_pos[cam_body]],
                           cam_pos=[float(v) for v in m.cam_pos[cid]], cam_quat=_wxyz(m.cam_quat[cid]),
                           fovy_deg=A.WRIST_CAM["fovy_deg"], width=A.WRIST_CAM["width"], height=A.WRIST_CAM["height"],
                           note="MuJoCo camera convention: looks along its -z, image up = its +y",
                           # the gripper as the camera sees it: boxes (min xyz, max xyz) in the gripper's / jaw's frame
                           self_gripper=[[[b[2], b[4], b[0]], [b[3], b[5], b[1]]] for b in A.SELF_BOXES],
                           self_jaw=[[[b[2], b[0], b[4]], [b[3], b[1], b[5]]] for b in A.JAW_BOXES]),
            capsules=caps,
            servos=dict(model="STS3215", ids=list(ARM_SERVO_IDS), stall_torque=A.SERVO_STALL, speed=A.SERVO_SPEED,
                        ticks_per_turn=4096, baud=1000000),
        ),
        bin=dict(centre=[bx, by, float(g.plate_top)], size=[L, W, H]),
        poses={k: list(v) for k, v in ARM_POSES.items()},
        reach=dict(sweet=list(ARM_SWEET), look_back=D435_LOOK_BACK if p.depth_camera == "d435" else 0.0,
                   view_near=max(0.32, round(floor_near + 0.05, 3)),
                   note="sweet: where the base brings a thing before the reach, the middle of the fingers-down reach "
                        "(0.26-0.36 m ahead on the arm kit); look_back: the close look this far short of it (a depth "
                        "camera that can't see it), then a creep in; view_near: the depth camera's floor + 5 cm"),
        keep_out=keep_out,
        cameras=cams,
        frames=frames(m, d, p),
    )


def _rounded(x):
    """10 significant digits: the same file from Windows and WSL (their numpy / MuJoCo builds differ in the last bit).
    Anything under 1e-12 is rounding noise around zero (it differed between the two in its leading digits): 0."""
    if isinstance(x, float):
        return 0.0 if abs(x) < 1e-12 else float(f"{x:.10g}")
    if isinstance(x, dict):
        return {k: _rounded(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_rounded(v) for v in x]
    return x


def main(check: bool = False) -> bool:
    p = v11_params()
    hdr = firmware_header(p)
    hdr_arm = firmware_header(arm_params(), ARM)
    js = json.dumps(_rounded(robot_json(p)), indent=1) + "\n"
    arm = json.dumps(_rounded(arm_json(arm_params())), indent=1) + "\n"
    arm_d435 = json.dumps(_rounded(arm_json(arm_d435_params(), ARM_D435)), indent=1) + "\n"
    files = ((FW_PARAMS, hdr), (FW_PARAMS_ARM, hdr_arm), (ROBOT_JSON, js), (ARM_JSON, arm), (ARM_D435_JSON, arm_d435))
    same = all(f.exists() and f.read_text() == text for f, text in files)
    if check:
        return same
    for f, text in files:
        f.parent.mkdir(parents=True, exist_ok=True)
        if not f.exists() or f.read_text() != text:       # unchanged files keep their time (the SIL library's build)
            f.write_text(text)
    print(f"wrote {', '.join(str(f) for f, _ in files)}" + ("" if not same else " (unchanged)"))
    return True


if __name__ == "__main__":
    main()
