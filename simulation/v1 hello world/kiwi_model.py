"""Parametric MuJoCo model (MJCF) of the v1 play-catch kiwi robot and its test area.

Everything is generated from SimParams so later spec sweeps can vary geometry and
limits. Run this file to write scene.xml, which opens in MuJoCo's stand-alone
`simulate` app (drag and drop).

Frames
  World: z up, floor at z = 0. The thrower stands on the -x side of the area.
  Robot ("kiwi" body): origin on the floor under the base center, x forward
    (launcher side), y left, z up.
  Catch module: origin at the center of the payload-interface plate's top face,
    axes parallel to the robot frame (the first coordinate-frame contract).
  Wheel i sits at angle wheel_angles_deg[i] from robot +x. A positive wheel speed
    pushes the robot counter-clockwise around its center (along the tangent).

Omni-wheel model (no individual rollers, per the design doc)
  Each wheel is a driven hub (hinge on the axle, carries the visual wheel and the
  drive inertia) plus a contact sphere on a ball joint. A stiff damper couples the
  sphere to the hub about the axle only, so the sphere is driven in the rolling
  direction and rolls freely along the axle, like the bottom roller of a real omni
  wheel. Friction is ordinary Coulomb friction (elliptic cone) at the sphere.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import mujoco


@dataclass
class SimParams:
    # --- Drive base ---------------------------------------------------------
    wheel_radius: float = 0.0625            # 125 mm double-row omni wheel
    wheel_width: float = 0.045
    wheel_base_radius: float = 0.18         # base center to wheel center
    wheel_angles_deg: tuple = (60.0, 180.0, 300.0)  # front-left, back, front-right
    wheel_mass: float = 0.25                # hub + rollers
    # Share of the wheel mass carried by the contact sphere. Its isotropic inertia
    # adds a little sideways rolling inertia (~0.06 kg equivalent per wheel), but a
    # lighter sphere makes the 2 ms step drift in yaw (tested down to 20 g).
    wheel_contact_mass: float = 0.15
    wheel_drive_inertia: float = 1.0e-3     # reflected AK40-10 rotor inertia at the wheel, kg m^2
    wheel_friction: float = 0.8             # rubber roller on floor
    deck_radius: float = 0.225              # 45 cm wide base
    deck_height: float = 0.135
    plate_height: float = 0.25              # payload-interface plate center

    # AK40-10 KV170 (CubeMars spec): internal velocity loop on the driver.
    motor_peak_torque: float = 4.1          # N m at the output
    motor_max_speed: float = 38.7           # rad/s (370 rpm rated)
    motor_velocity_gain: float = 2.0        # N m per rad/s of speed error

    # Base motion limits (firmware). Accelerating between the two front wheels is
    # pushed by those two wheels alone while load transfer unloads them, so with
    # mu = 0.8 and this CG the wheels start slipping a little above 3 m/s^2.
    base_max_accel: float = 3.0
    base_max_speed: float = 1.4

    # --- Catch module ------------------------------------------------------
    funnel_rim_height: float = 0.60         # above the floor
    funnel_rim_radius: float = 0.165        # 33 cm funnel
    funnel_throat_radius: float = 0.045     # ball radius + 8 mm
    funnel_wall_angle_deg: float = 35.0     # from vertical
    funnel_panels: int = 20
    funnel_wall_thickness: float = 0.004
    chamber_floor_height: float = 0.066     # above the payload plate top
    launch_angle_deg: float = 52.0
    launch_exit: tuple = (0.11, 0.0, 0.16)  # catch-module frame

    # --- Ball: pickleball-sized foam ball ------------------------------------
    ball_radius: float = 0.037
    ball_mass: float = 0.025
    ball_drag_cd: float = 0.5

    # --- Test area and play -------------------------------------------------
    area_size: tuple = (5.0, 4.0)           # x, y; netting walls on the edges
    wall_height: float = 2.0
    thrower_xy: tuple = (-2.0, 0.0)
    home_xy: tuple = (1.0, 0.0)

    timestep: float = 0.002
    air_density: float = 1.204
    air_viscosity: float = 1.8e-5

    @property
    def payload_plate_top(self) -> float:
        return self.plate_height + 0.004

    @property
    def funnel_depth(self) -> float:
        alpha = math.radians(self.funnel_wall_angle_deg)
        return (self.funnel_rim_radius - self.funnel_throat_radius) / math.tan(alpha)

    @property
    def home_yaw(self) -> float:
        """Heading that points the launcher at the thrower from home."""
        dx = self.thrower_xy[0] - self.home_xy[0]
        dy = self.thrower_xy[1] - self.home_xy[1]
        return math.atan2(dy, dx)


# ---------------------------------------------------------------------------
# small geometry helpers

def _f(*vals) -> str:
    return " ".join(f"{v:.6g}" for v in vals)


def quat_axis(axis, angle):
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def xyaxes_looking_at(pos, target):
    """MuJoCo camera xyaxes for a camera at pos looking at target, z up."""
    f = [t - p for p, t in zip(pos, target)]
    n = math.sqrt(sum(c * c for c in f))
    f = [c / n for c in f]
    x = [f[1], -f[0], 0.0]                       # f x z_up
    n = math.hypot(x[0], x[1])
    x = [c / n for c in x]
    y = [x[1] * f[2] - x[2] * f[1], x[2] * f[0] - x[0] * f[2], x[0] * f[1] - x[1] * f[0]]
    return _f(*x, *y)


# ---------------------------------------------------------------------------
# robot parts

def _wheel(p: SimParams, i: int) -> str:
    th = math.radians(p.wheel_angles_deg[i - 1])
    c, s = math.cos(th), math.sin(th)
    r = p.wheel_radius
    hw = p.wheel_width / 2
    rm0 = p.wheel_base_radius - hw - 0.004     # motor face, radial distance
    rm1 = rm0 - 0.035                          # motor back
    # Hub x axis = axle pointing inward, so a positive hinge speed drives the
    # contact point along the counter-clockwise tangent.
    q = quat_axis((0, 0, 1), th + math.pi)
    rollers = []
    n_rollers, roller_r = 8, 0.011
    ring = r - roller_r
    for row, off in enumerate((-hw / 2, hw / 2)):
        for j in range(n_rollers):
            g = 2 * math.pi * (j + 0.5 * row) / n_rollers
            cy, cz = ring * math.cos(g), ring * math.sin(g)
            ty, tz = -math.sin(g), math.cos(g)
            half = 0.6 * ring * math.pi / n_rollers
            rollers.append(
                f'<geom class="decor" type="capsule" size="{roller_r}" material="rubber" '
                f'fromto="{_f(off, cy - half * ty, cz - half * tz, off, cy + half * ty, cz + half * tz)}"/>')
    return f"""
      <body name="hub{i}" pos="{_f(p.wheel_base_radius * c, p.wheel_base_radius * s, r)}" quat="{_f(*q)}">
        <joint name="drive{i}" type="hinge" axis="1 0 0" armature="{p.wheel_drive_inertia}" damping="0.0005"/>
        <geom class="decor" type="cylinder" size="0.034" fromto="{_f(-hw * 0.85, 0, 0, hw * 0.85, 0, 0)}" material="hub" mass="{p.wheel_mass - p.wheel_contact_mass}"/>
        <geom class="decor" type="cylinder" size="{ring - 0.004}" fromto="{_f(-0.002, 0, 0, 0.002, 0, 0)}" material="hub"/>
        <geom class="decor" type="cylinder" size="0.012" fromto="{_f(-hw, 0, 0, hw, 0, 0)}" material="steel"/>
        {''.join(rollers)}
        <site name="hub{i}" group="4"/>
        <body name="wheel{i}_contact">
          <joint name="roller{i}" type="ball" damping="0.0002"/>
          <geom class="contact" type="sphere" size="{r}" mass="{p.wheel_contact_mass}" friction="{p.wheel_friction} 0.005 0.0001"/>
          <site name="wheel{i}_contact" group="4"/>
        </body>
      </body>
      <geom class="decor" type="cylinder" size="0.026" material="motor" mass="0.19"
            fromto="{_f(rm0 * c, rm0 * s, r, rm1 * c, rm1 * s, r)}"/>
      <geom class="decor" type="box" size="0.003 0.032 {_f((p.deck_height - r) / 2 + 0.01)}" material="aluminum" mass="0.06"
            pos="{_f((rm1 - 0.003) * c, (rm1 - 0.003) * s, (p.deck_height + r) / 2 - 0.005)}"
            quat="{_f(*quat_axis((0, 0, 1), th))}"/>"""


def _funnel(p: SimParams) -> tuple[str, float]:
    """Funnel wall panels and rim (catch-module frame); also returns the throat height."""
    alpha = math.radians(p.funnel_wall_angle_deg)
    top = p.funnel_rim_height - p.payload_plate_top           # rim, module frame
    z_throat = top - p.funnel_depth
    R, rb, n = p.funnel_rim_radius, p.funnel_throat_radius, p.funnel_panels
    slant = (R - rb) / math.sin(alpha)
    r_mid, z_mid = (R + rb) / 2, (top + z_throat) / 2
    ht = p.funnel_wall_thickness / 2
    hw = R * math.tan(math.pi / n) * 1.08
    out = []
    for k in range(n):
        b = 2 * math.pi * k / n
        q = quat_mul(quat_axis((0, 0, 1), b), quat_axis((0, 1, 0), alpha))
        rc, zc = r_mid + ht * math.cos(alpha), z_mid - ht * math.sin(alpha)
        out.append(f'<geom class="funnel" type="box" size="{_f(ht, hw, slant / 2)}" '
                   f'pos="{_f(rc * math.cos(b), rc * math.sin(b), zc)}" quat="{_f(*q)}"/>')
    # rim ring
    rr = R + 0.004
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        out.append(f'<geom class="decor" type="capsule" size="0.006" material="rim" mass="0.005" '
                   f'fromto="{_f(rr * math.cos(a0), rr * math.sin(a0), top, rr * math.cos(a1), rr * math.sin(a1), top)}"/>')
    return "\n        ".join(out), z_throat


def _catch_module(p: SimParams) -> str:
    funnel, z_throat = _funnel(p)
    w, t = p.funnel_throat_radius, 0.004
    zf = p.chamber_floor_height
    hz = (z_throat - zf) / 2
    zc = zf + hz
    z_ball = zf + p.ball_radius
    la = math.radians(p.launch_angle_deg)
    ex, ey, ez = p.launch_exit
    # flywheel axes are perpendicular to the launch direction and to y
    fw_axis = (-math.sin(la), 0.0, math.cos(la))
    fw_c = (ex - 0.03 * math.cos(la), 0.0, ez - 0.03 * math.sin(la))
    fw_y = p.ball_radius - 0.006 + 0.03
    flywheels = []
    for sgn in (-1, 1):
        cx, cy, cz = fw_c[0], sgn * fw_y, fw_c[2]
        flywheels.append(
            f'<geom class="decor" type="cylinder" size="0.03" material="flywheel" mass="0.03" '
            f'fromto="{_f(cx - 0.01 * fw_axis[0], cy, cz - 0.01 * fw_axis[2], cx + 0.01 * fw_axis[0], cy, cz + 0.01 * fw_axis[2])}"/>'
            f'<geom class="decor" type="cylinder" size="0.014" material="motor" mass="0.03" '
            f'fromto="{_f(cx - 0.012 * fw_axis[0], cy, cz - 0.012 * fw_axis[2], cx - 0.04 * fw_axis[0], cy, cz - 0.04 * fw_axis[2])}"/>')
    rim_z = p.funnel_rim_height - p.payload_plate_top
    cam_axes = xyaxes_looking_at((0, 0, 0), (math.cos(math.radians(30)), 0, math.sin(math.radians(30))))
    cams = []
    for name, y in (("left", 0.1), ("right", -0.1)):
        pos = (0.15, y, rim_z + 0.015)
        cams.append(f'<geom class="decor" type="box" size="0.012 0.018 0.012" pos="{_f(*pos)}" material="camera" mass="0.04"/>'
                    f'<camera name="stereo_{name}" pos="{_f(pos[0] + 0.013, pos[1], pos[2])}" xyaxes="{cam_axes}" fovy="70"/>')
    chute_len = 0.07
    chute_c = (ex - chute_len / 2 * math.cos(la), 0, ez - chute_len / 2 * math.sin(la))
    return f"""
      <body name="catch_module" pos="0 0 {_f(p.payload_plate_top)}">
        <geom class="decor" type="box" size="0.075 0.07 0.03" pos="0 0 0.03" material="launcher" mass="0.8"/>
        <geom class="structure" type="box" size="{_f(w + t, w + t, t / 2)}" pos="{_f(0, 0, zf - t / 2)}" material="chamber" mass="0.03"/>
        <geom class="structure" type="box" size="{_f(t / 2, w + t, hz)}" pos="{_f(w + t / 2, 0, zc)}" material="chamber" mass="0.03"/>
        <geom class="structure" type="box" size="{_f(t / 2, w + t, hz)}" pos="{_f(-w - t / 2, 0, zc)}" material="chamber" mass="0.03"/>
        <geom class="structure" type="box" size="{_f(w, t / 2, hz)}" pos="{_f(0, w + t / 2, zc)}" material="chamber" mass="0.03"/>
        <geom class="structure" type="box" size="{_f(w, t / 2, hz)}" pos="{_f(0, -w - t / 2, zc)}" material="chamber" mass="0.03"/>
        <geom class="decor" type="box" size="{_f(chute_len / 2, 0.03, 0.003)}" pos="{_f(*chute_c)}" quat="{_f(*quat_axis((0, 1, 0), -la))}" material="launcher" mass="0.02"/>
        {''.join(flywheels)}
        {funnel}
        {''.join(cams)}
        <site name="catch_beam" pos="{_f(0, -w + 0.002, z_ball)}" quat="{_f(*quat_axis((1, 0, 0), -math.pi / 2))}" group="4"/>
        <site name="launch_exit" pos="{_f(ex, ey, ez)}" group="4"/>
        <site name="funnel_center" pos="{_f(0, 0, rim_z)}" group="4"/>
      </body>"""


def _robot(p: SimParams) -> str:
    dz = p.deck_height
    standoffs = []
    for a in (60, 180, 300):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        standoffs.append(f'<geom class="decor" type="cylinder" size="0.008" material="steel" mass="0.03" '
                         f'fromto="{_f(0.15 * c, 0.15 * s, dz + 0.003, 0.15 * c, 0.15 * s, p.plate_height - 0.004)}"/>')
    bumper = []
    nb, rb = 18, p.deck_radius + 0.004
    for k in range(nb):
        a0, a1 = 2 * math.pi * k / nb, 2 * math.pi * (k + 1) / nb
        bumper.append(f'<geom class="decor" type="capsule" size="0.016" material="bumper" mass="0.015" '
                      f'fromto="{_f(rb * math.cos(a0), rb * math.sin(a0), dz - 0.02, rb * math.cos(a1), rb * math.sin(a1), dz - 0.02)}"/>')
    hx, hy = p.home_xy
    q = quat_axis((0, 0, 1), p.home_yaw)
    wheels = "".join(_wheel(p, i) for i in (1, 2, 3))
    return f"""
    <body name="kiwi" pos="{_f(hx, hy, 0)}" quat="{_f(*q)}">
      <freejoint name="kiwi"/>
      <site name="imu" pos="0 0 {_f(dz + 0.01)}" group="4"/>
      <geom class="structure" type="cylinder" size="{_f(p.deck_radius, 0.003)}" pos="0 0 {_f(dz)}" material="deck" mass="0.9"/>
      <geom class="decor" type="box" size="0.075 0.045 0.035" pos="0 0 {_f(dz - 0.05)}" material="battery" mass="0.85"/>
      <geom class="decor" type="box" size="0.06 0.06 0.004" pos="0 0 {_f(dz - 0.088)}" material="aluminum" mass="0.35"/>
      <geom class="decor" type="box" size="0.05 0.05 0.022" pos="{_f(-0.075, 0, dz + 0.025)}" material="jetson" mass="0.35"/>
      <geom class="decor" type="box" size="0.035 0.035 0.008" pos="{_f(-0.075, 0, dz + 0.055)}" material="heatsink"/>
      <geom class="decor" type="box" size="0.04 0.025 0.01" pos="{_f(0.0, 0.1, dz + 0.013)}" material="pcb" mass="0.25"/>
      <geom class="decor" type="cylinder" size="0.037 0.02" pos="{_f(0.155, 0, dz + 0.023)}" material="lidar" mass="0.11"/>
      <geom class="decor" type="cylinder" size="0.03 0.006" pos="{_f(0.155, 0, dz + 0.049)}" material="lidar_top"/>
      <geom class="decor" type="cylinder" size="0.022 0.012" pos="{_f(-0.175, 0, dz + 0.015)}" material="estop" mass="0.05"/>
      <geom class="decor" type="cylinder" size="0.016 0.006" pos="{_f(-0.175, 0.0, dz + 0.004)}" material="estop_base"/>
      {''.join(standoffs)}
      {''.join(bumper)}
      <geom class="structure" type="cylinder" size="0.17 0.004" pos="0 0 {_f(p.plate_height)}" material="plate" mass="0.35"/>
      {wheels}
      {_catch_module(p)}
    </body>"""


# ---------------------------------------------------------------------------
# scene

def build_xml(p: SimParams = SimParams()) -> str:
    ax, ay = p.area_size
    wh = p.wall_height
    # Netting: invisible collision walls plus posts and rails to show where they are.
    walls = []
    for name, pos, size in (("wall_px", (ax / 2, 0, wh / 2), (0.01, ay / 2, wh / 2)),
                            ("wall_nx", (-ax / 2, 0, wh / 2), (0.01, ay / 2, wh / 2)),
                            ("wall_py", (0, ay / 2, wh / 2), (ax / 2, 0.01, wh / 2)),
                            ("wall_ny", (0, -ay / 2, wh / 2), (ax / 2, 0.01, wh / 2))):
        walls.append(f'<geom name="{name}" type="box" pos="{_f(*pos)}" size="{_f(*size)}" group="3" rgba="0.2 0.2 0.2 0.1"/>')
    corners = [(-ax / 2, -ay / 2), (ax / 2, -ay / 2), (ax / 2, ay / 2), (-ax / 2, ay / 2)]
    for k, (x0, y0) in enumerate(corners):
        x1, y1 = corners[(k + 1) % 4]
        walls.append(f'<geom class="decor" type="cylinder" size="0.02" fromto="{_f(x0, y0, 0, x0, y0, wh)}" material="post"/>')
        for z in (0.03, wh):
            walls.append(f'<geom class="decor" type="capsule" size="0.008" fromto="{_f(x0, y0, z, x1, y1, z)}" material="post"/>')
    tx, ty = p.thrower_xy
    side_cam_pos = (-0.5, -3.9, 1.6)
    side_cam_axes = xyaxes_looking_at(side_cam_pos, (-0.5, 0.0, 0.95))
    actuators = []
    for i in (1, 2, 3):
        actuators.append(
            f'<velocity name="wheel{i}" joint="drive{i}" kv="{p.motor_velocity_gain}" '
            f'ctrlrange="{_f(-p.motor_max_speed, p.motor_max_speed)}" forcerange="{_f(-p.motor_peak_torque, p.motor_peak_torque)}"/>')
    for i in (1, 2, 3):
        # Stiff damper between hub and contact sphere about the axle only.
        actuators.append(
            f'<general name="wheel{i}_coupling" site="wheel{i}_contact" refsite="hub{i}" gear="0 0 0 1 0 0" '
            f'gainprm="0" biastype="affine" biasprm="0 0 -50"/>')
    # objtype "xbody" = body frame origin; "body" would be the inertial (CoM) frame
    sensors = [
        '<framepos name="base_pos" objtype="xbody" objname="kiwi"/>',
        '<framequat name="base_quat" objtype="xbody" objname="kiwi"/>',
        '<framelinvel name="base_linvel" objtype="xbody" objname="kiwi"/>',
        '<frameangvel name="base_angvel" objtype="xbody" objname="kiwi"/>',
        '<gyro name="imu_gyro" site="imu"/>',
        '<accelerometer name="imu_accel" site="imu"/>',
        '<rangefinder name="catch_beam" site="catch_beam" cutoff="0.5"/>',
        '<framepos name="ball_pos" objtype="xbody" objname="ball"/>',
        '<framelinvel name="ball_vel" objtype="xbody" objname="ball"/>',
    ]
    for i in (1, 2, 3):
        sensors.append(f'<jointvel name="wheel{i}_speed" joint="drive{i}"/>')
        sensors.append(f'<actuatorfrc name="wheel{i}_torque" actuator="wheel{i}"/>')

    return f"""<mujoco model="play_catch_v1_hello_world">
  <!-- Generated by kiwi_model.py; edit SimParams there, not this file. -->
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{p.timestep}" integrator="implicitfast" cone="elliptic" impratio="10"
          density="{p.air_density}" viscosity="{p.air_viscosity}"/>
  <statistic center="-0.4 0 0.7" extent="3.2"/>

  <visual>
    <global offwidth="1920" offheight="1080" azimuth="90" elevation="-18"/>
    <quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.35 0.35 0.38" diffuse="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
    <map znear="0.01" zfar="60"/>
  </visual>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.62 0.74 0.88" rgb2="0.18 0.22 0.3" width="512" height="512"/>
    <texture name="floor" type="2d" builtin="checker" rgb1="0.36 0.4 0.44" rgb2="0.31 0.35 0.39" width="512" height="512" mark="edge" markrgb="0.45 0.5 0.55"/>
    <material name="floor" texture="floor" texrepeat="10 10" reflectance="0.12"/>
    <material name="post" rgba="0.25 0.27 0.3 1"/>
    <material name="deck" rgba="0.22 0.24 0.27 1" specular="0.4" shininess="0.5"/>
    <material name="plate" rgba="0.55 0.57 0.6 1" specular="0.5" shininess="0.6"/>
    <material name="aluminum" rgba="0.7 0.72 0.75 1" specular="0.6" shininess="0.7"/>
    <material name="steel" rgba="0.8 0.8 0.82 1" specular="0.8" shininess="0.9"/>
    <material name="hub" rgba="0.15 0.16 0.18 1"/>
    <material name="rubber" rgba="0.08 0.08 0.08 1" specular="0.1"/>
    <material name="motor" rgba="0.12 0.12 0.13 1" specular="0.6" shininess="0.6"/>
    <material name="battery" rgba="0.15 0.35 0.75 1"/>
    <material name="jetson" rgba="0.1 0.45 0.2 1"/>
    <material name="heatsink" rgba="0.25 0.25 0.27 1" specular="0.7"/>
    <material name="pcb" rgba="0.05 0.3 0.15 1"/>
    <material name="lidar" rgba="0.1 0.1 0.11 1"/>
    <material name="lidar_top" rgba="0.75 0.12 0.1 1"/>
    <material name="estop" rgba="0.85 0.1 0.08 1"/>
    <material name="estop_base" rgba="0.95 0.8 0.1 1"/>
    <material name="bumper" rgba="0.95 0.55 0.1 1"/>
    <material name="launcher" rgba="0.2 0.21 0.23 1"/>
    <material name="flywheel" rgba="0.9 0.3 0.1 1"/>
    <material name="chamber" rgba="0.6 0.65 0.7 0.5"/>
    <material name="funnel" rgba="0.1 0.55 0.85 0.35"/>
    <material name="rim" rgba="0.95 0.55 0.1 1"/>
    <material name="camera" rgba="0.05 0.05 0.05 1"/>
    <material name="ball" rgba="0.85 0.95 0.15 1" specular="0.2"/>
    <material name="person" rgba="0.55 0.6 0.7 0.55"/>
    <material name="marker" rgba="0.1 0.9 0.3 0.35"/>
  </asset>

  <default>
    <default class="structure">
      <geom group="0" contype="1" conaffinity="1"/>
    </default>
    <default class="decor">
      <geom group="1" contype="0" conaffinity="0" density="0"/>
    </default>
    <default class="contact">
      <geom group="3" contype="1" conaffinity="1" rgba="0.9 0.2 0.2 0.3"/>
    </default>
    <default class="funnel">
      <geom group="0" contype="1" conaffinity="1" material="funnel" mass="0.015" friction="0.9 0.005 0.0001"/>
    </default>
  </default>

  <worldbody>
    <light name="sun" pos="1 -1 6" dir="-0.15 0.15 -1" directional="true" castshadow="true" diffuse="0.6 0.6 0.58"/>
    <light name="fill" pos="-3 3 4" dir="0.5 -0.5 -0.7" directional="true" castshadow="false" diffuse="0.2 0.2 0.22"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="floor" friction="0.8 0.005 0.0001"/>
    {''.join(walls)}
    <camera name="side" pos="{_f(*side_cam_pos)}" xyaxes="{side_cam_axes}" fovy="50"/>

    <body name="thrower" pos="{_f(tx, ty, 0)}">
      <geom class="decor" type="capsule" size="0.07" fromto="0 0.1 0.05 0 0.1 0.85" material="person"/>
      <geom class="decor" type="capsule" size="0.07" fromto="0 -0.1 0.05 0 -0.1 0.85" material="person"/>
      <geom class="decor" type="capsule" size="0.16" fromto="0 0 0.95 0 0 1.35" material="person"/>
      <geom class="decor" type="sphere" size="0.12" pos="0 0 1.65" material="person"/>
      <geom class="decor" type="capsule" size="0.045" fromto="0 -0.22 1.4 0.3 -0.12 1.0" material="person"/>
    </body>

    <body name="landing_marker" mocap="true" pos="{_f(*p.home_xy, 0.002)}">
      <geom class="decor" type="cylinder" size="{_f(p.funnel_rim_radius, 0.001)}" material="marker"/>
    </body>

    <body name="ball" pos="{_f(tx + 0.3, ty + 0.4, p.ball_radius + 0.001)}">
      <freejoint name="ball"/>
      <geom name="ball" type="sphere" size="{p.ball_radius}" mass="{p.ball_mass}" material="ball"
            condim="6" friction="0.8 0.005 0.002" fluidshape="ellipsoid" fluidcoef="{_f(p.ball_drag_cd / 2, 0.25, 1.5, 1.0, 1.0)}"/>
    </body>
    {_robot(p)}
  </worldbody>

  <actuator>
    {''.join(actuators)}
  </actuator>

  <sensor>
    {''.join(sensors)}
  </sensor>
</mujoco>
"""


def build_model(p: SimParams = SimParams()) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(build_xml(p))


def describe(model: mujoco.MjModel, p: SimParams) -> str:
    """One-paragraph mass / CG / tipping summary of the robot."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    kiwi = model.body("kiwi").id
    mass = model.body_subtreemass[kiwi]
    cg = data.subtree_com[kiwi] - data.xpos[kiwi]
    # Support polygon is the wheel-contact triangle; its inradius is half the
    # wheel circle radius, which is the worst-case tipping lever.
    edge = p.wheel_base_radius / 2
    a_edge = 9.81 * edge / cg[2]
    a_vertex = 9.81 * p.wheel_base_radius / cg[2]
    return (f"robot mass {mass:.2f} kg, CG height {cg[2] * 100:.1f} cm; quasi-static tip-over at "
            f"{a_edge:.1f} m/s^2 toward a wheel-triangle edge, {a_vertex:.1f} m/s^2 toward a wheel; "
            f"top speed {p.motor_max_speed * p.wheel_radius:.2f} m/s at the wheel")


if __name__ == "__main__":
    params = SimParams()
    out = Path(__file__).with_name("scene.xml")
    out.write_text(build_xml(params))
    print(f"wrote {out}")
    print(describe(build_model(params), params))
