"""v11 (design.md 30): the sim's robot, exported to the code that will run on it. One source, like cad/v7/dims.py:

  firmware/lib/omni_fw/fw_params.h            the MCU firmware's constants (kinematics, limits, flywheel loop, feed,
                                               tilt range, drive-motor CAN ids and ranges)
  ros/src/omni_catch/omni_catch/config/robot_v11.json
                                               what the Jetson needs: the funnel's rim and the launch exit against
                                               tilt, the launcher calibration, the ball, the cameras' extrinsics and
                                               intrinsics, the base's planning limits
  ros/src/omni_arm/omni_arm/config/robot_arm.json
                                               the arm robot (design.md 11, ARM below): the SO-101's chain as the sim
                                               builds it, its servos, finger pads, wrist camera, the stereo pods, the
                                               bin, the robot's keep-out volumes for the arm, the arm-out base caps

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

# kid mode (design.md 14): hard caps the firmware applies whatever the Jetson asks (est, to be set in an adult-only
# session): 0.8 m/s, 60% of the acceleration table, half the jerk and yaw rate, gentle lobs only
KID = dict(max_speed=0.8, accel_scale=0.6, jerk_scale=0.5, yaw_scale=0.5, fly_rpm_max=2600.0)
ADULT_FLY_RPM_MAX = 6000.0      # est: above every calibrated shot (design.md 9: 10.1 m/s needs ~3950 rpm)

# Damiao DM-2325 on CAN (dm_motor.h). VERIFY all of these against the units flashed with Damiao's tool
DM = dict(ids=(1, 2, 3, 4), dirs=(1, 1, 1, 1), master_base=0x10, p_max=12.5, v_max=60.0, t_max=5.0)
FLY_POLE_PAIRS = 7              # F90 2806.5: 12N14P (est from the stator class; VERIFY)
DM_SAMPLE_TO_RX_US = 140        # design.md 10: reply frame ~125 us + turnaround (est); sil_robot.DmBus models it


def v11_params(**kw) -> SimParams:
    return SimParams(**{**V11, **kw})


# the drawn robot's mass (cad/v7/out/BOM.md "Mass": build.py's sum of every part, the module at tilt 0, ROBOT frame;
# test_v11 checks these against the BOM)
CAD_MASS_KG = 8.19
CAD_CG = (0.0180, -0.0014, 0.1246)


def drawn_params(**kw) -> SimParams:
    """The v11 robot with the CAD's masses (design.md 7, step 1): mass_model "cad" (the 515 g printed wheels, no front
    rocker, the bumper ring at r 205-217 mm) and one point mass on the base that brings the total and the CG to the
    BOM's: what the CAD has and the sim doesn't (the head and face, heavier printed parts, fasteners, cables)."""
    from dataclasses import replace
    from robot_model import mass_properties
    p = v11_params(mass_model="cad", **kw)
    mass, cg = mass_properties(build_model(p), p, 0.0)
    dm = CAD_MASS_KG - mass
    if dm <= 0.0:
        raise ValueError(f"the sim's robot ({mass:.2f} kg) is already heavier than the CAD's")
    pos = (CAD_MASS_KG * np.array(CAD_CG) - mass * np.asarray(cg)) / dm
    return replace(p, extra_masses=tuple(p.extra_masses) + ((round(dm, 4), *(round(float(x), 4) for x in pos)),))


# the arm robot as drawn (design.md 12.3, Oct 25; cad/v7 arm_kit.py, studies/layout_trade.py A6): the baseline base
# and head with the arm kit in place of the catch module: the SO-101 on a 50 mm riser at the plate's front edge (its
# base over the camera bar), the bar at its arm index (40 deg down), the lidar at the front, the bin on the plate's
# rear, the face and the audio box over it
ARM = dict(payload="arm", plate_gap=0.096, arm_mount=(0.14, 0.0, 0.0), arm_riser=0.05, arm_pick=True,
           arm_bin=(-0.07, 0.0, 0.12, 0.12, 0.07), arm_display="face", lidar_mount="front", camera_mount="head",
           camera_head_xz=(0.18266, 0.16573), camera_tilt_deg=-40.0)
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
# the Jetson's "arm out" request (design.md 11): while the tool is outside the bumper or above ARM_OUT_Z, the base runs
# at half the acceleration table's straight-ahead value and at most 1 m/s (CMD a_cap / speed_cap)
ARM_OUT = dict(speed_cap=1.0, a_cap_frac=0.5, out_radius=0.217, out_z=0.40)
# the six STS3215s on the Jetson's USB bus board (Waveshare bus servo adapter, 1 Mbaud): ids as LeRobot's SO-101
ARM_SERVO_IDS = (1, 2, 3, 4, 5, 6)


def arm_params(**kw) -> SimParams:
    return v11_params(**{**ARM, **kw})


def _c(v) -> str:
    s = f"{float(v):.9g}"
    return (s if any(ch in s for ch in ".e") else s + ".0") + "f"


def firmware_header(p: SimParams) -> str:
    kin = OmniKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius)
    w_max = p.wheel_speed_frac * p.motor.noload_rpm * 2 * math.pi / 60
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
        f"// Robot: {json.dumps(V11)}",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        "namespace fw {",
        "namespace prm {",
        "",
        "constexpr float TICK_S = 0.001f;                 // the firmware's 1 kHz tick (design.md 4)",
        "constexpr uint64_t STATE_PERIOD_US = 5000;       // STATE to the Jetson at 200 Hz",
        "constexpr float CMD_TIMEOUT = 0.5f;              // s without a CMD -> ramped stop (est)",
        "constexpr float DM_FEEDBACK_TIMEOUT = 0.05f;     // s without a drive motor reply -> fault",
        f"constexpr float BATT_LOW = {_c(3.2 * p.pack.series)};          // V: 3.2 V per cell",
        "constexpr float MANUAL_SPEED = 1.0f;             // m/s cap on the RC sticks (est)",
        "",
        "// time references (design.md 10): SYNC_OUT to the Jetson's GPIO, CAM_TRIG to both cameras' FSIN",
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
        f"constexpr float MAX_WHEEL_SPEED = {_c(w_max)};   // rad/s: {p.wheel_speed_frac} x the {p.drive_motor}'s no-load speed",
        "",
        "// limits by profile: max_speed, max_accel, max_jerk, max_yaw_rate, max_yaw_accel, brake_margin, accel_scale,",
        "// fly_rpm_max. The Jetson can only ask for less (CMD speed_cap)",
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
        "constexpr float FLY_READY_TOL = 0.03f;          // the fire interlock (the Jetson asks for 2%)",
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
                  J=kin.J.tolist(), max_wheel_speed=p.wheel_speed_frac * p.motor.noload_rpm * 2 * math.pi / 60,
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


def arm_json(p: SimParams) -> dict:
    """The arm robot for omni_arm (kinematics.ArmConfig): every transform as the sim's model has it (MuJoCo's body
    frames: pos + quaternion w x y z in the parent body; each joint turns its body about its axis through the body's
    origin), so the Jetson's kinematics are the sim's to the bit."""
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
        for note, lo, hi in ARM_HEAD_KEEP_OUT:
            keep_out.append(dict(kind="box", min=list(lo), max=list(hi), note=note))
    else:
        for c in cams:                                # each camera pod, a 4 cm block
            x, y, z = c["pos"]
            keep_out.append(dict(kind="box", min=[x - 0.03, y - 0.02, 0.0], max=[x + 0.01, y + 0.02, z + 0.02],
                                 note=f"the {c['name']} camera pod"))
    tbl = p.accel_table
    a_ahead = float(tbl[1][0]) if tbl else float(p.base_max_accel)
    return dict(
        generated_by="simulation/v11 sil/robot_export.py", robot={k: (list(v) if isinstance(v, tuple) else v)
                                                                  for k, v in ARM.items()},
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
        reach=dict(sweet=list(ARM_SWEET), note="where the base brings a thing before the reach: the middle of the "
                                               "fingers-down reach (0.26-0.36 m ahead on the arm kit)"),
        keep_out=keep_out,
        cameras=cams,
        frames=frames(m, d, p),
    )


def _rounded(x):
    """10 significant digits: the same file from Windows and WSL (their numpy / MuJoCo builds differ in the last bit)."""
    if isinstance(x, float):
        return float(f"{x:.10g}")
    if isinstance(x, dict):
        return {k: _rounded(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_rounded(v) for v in x]
    return x


def main(check: bool = False) -> bool:
    p = v11_params()
    hdr = firmware_header(p)
    js = json.dumps(_rounded(robot_json(p)), indent=1) + "\n"
    arm = json.dumps(_rounded(arm_json(arm_params())), indent=1) + "\n"
    same = (FW_PARAMS.exists() and FW_PARAMS.read_text() == hdr and ROBOT_JSON.exists() and ROBOT_JSON.read_text() == js
            and ARM_JSON.exists() and ARM_JSON.read_text() == arm)
    if check:
        return same
    for f, text in ((FW_PARAMS, hdr), (ROBOT_JSON, js), (ARM_JSON, arm)):
        f.parent.mkdir(parents=True, exist_ok=True)
        if not f.exists() or f.read_text() != text:       # unchanged files keep their time (the SIL library's build)
            f.write_text(text)
    print(f"wrote {FW_PARAMS}, {ROBOT_JSON} and {ARM_JSON}" + ("" if not same else " (unchanged)"))
    return True


if __name__ == "__main__":
    main()
