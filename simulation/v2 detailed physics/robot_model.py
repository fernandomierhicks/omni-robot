"""MJCF builder for the v2 play-catch robot and its test area.

Frames (same as v1)
  World: z up, floor at z = 0, thrower on the -x side.
  Robot "kiwi": origin on the floor under the base center, x forward (launcher), y left.
  Catch module: origin at the payload-plate top surface, axes parallel to the robot.

Catch module, front view of the ball path (module frame, x forward, z up):
  funnel throat -> sliding gate (holds the caught ball; break-beam across it)
  -> vertical drop tube -> J-bend (ball rides the outer wall) -> straight run up
  the launch line -> nip between two left/right flywheels -> launch at
  launch_angle_deg. The flywheel axes are perpendicular to the launch line and to
  y, so equal wheel speeds launch without spin.

Omni wheels
  "sphere": driven hub (visual rollers spin with it) + contact sphere on a ball
            joint coupled to the hub about the axle only (see v1). Fast.
  "rollers": driven hub with explicit barrel rollers on passive hinges, two
            offset rows. Slower; used to validate the sphere model.

Collision groups (contype / conaffinity bits)
  1 = general (ball, structure, walls)   2 = wheel contacts (floor only)
  The flywheels collide with the ball only through explicit <pair> elements.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import mujoco

from assets import barrel, cone_shell, ensure_textures, _f
from params import SimParams


# ---------------------------------------------------------------------------
# geometry helpers

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
    f = [t - q for q, t in zip(pos, target)]
    n = math.sqrt(sum(c * c for c in f))
    f = [c / n for c in f]
    x = [f[1], -f[0], 0.0]
    n = math.hypot(x[0], x[1])
    x = [c / n for c in x]
    y = [x[1] * f[2] - x[2] * f[1], x[2] * f[0] - x[0] * f[2], x[0] * f[1] - x[1] * f[0]]
    return _f(*x, *y)


@dataclass
class Geometry:
    """Derived dimensions shared by the model builder and the robot code."""
    R: float              # wheel radius
    deck_z: float         # deck underside
    plate_top: float      # payload plate top (module origin height)
    rim_z: float          # funnel rim, module frame
    throat_z: float
    gate_z: float         # gate plate top surface
    hold: tuple           # held-ball center, module frame
    w: float              # tube inner half width
    t: float              # wall thickness
    rb: float             # ball radius
    rho: float            # J centerline radius
    C: tuple              # J center (x, z)
    R_wall: float         # J outer-wall inner radius
    theta: float          # launch angle
    u: tuple              # launch direction (x, z)
    n: tuple              # launch-line normal pointing up (x, z)
    E: tuple              # ball center at the end of the J
    N: tuple              # ball center at the nip
    fw_r: float
    fw_y: float           # flywheel center |y|
    side_wall_end: float  # x where the tube side walls stop (flywheels beyond)


def geometry(p: SimParams) -> Geometry:
    R = p.wheel_radius
    rb = p.ball_spec.radius
    t = 0.004
    deck_z = p.deck_height
    plate_top = p.plate_height
    rim_z = p.funnel_rim_height - plate_top
    throat_z = rim_z - p.funnel_depth
    # the caught ball sits in a vertical-walled pocket one diameter deep, so its
    # sideways momentum is stopped by walls instead of carrying it up the funnel
    gate_z = throat_z - p.pocket_depth
    w = p.tube_half_width
    rho = p.j_radius
    R_wall = rho + rb
    z_b = R_wall + t + 0.006
    C = (-w + R_wall, z_b)
    th = math.radians(p.launch_angle_deg)
    u = (math.cos(th), math.sin(th))
    n = (-math.sin(th), math.cos(th))
    a_end = math.pi / 2 + th
    E = (C[0] - rho * math.cos(a_end), C[1] - rho * math.sin(a_end))
    N = (E[0] + p.nip_distance * u[0], E[1] + p.nip_distance * u[1])
    fw_r = p.flywheel_diameter / 2
    fw_y = fw_r + rb - p.squeeze / 2
    fw_xmin = N[0] - fw_r * u[0] - p.flywheel_width / 2 * abs(n[0])
    return Geometry(R, deck_z, plate_top, rim_z, throat_z, gate_z, (C[0] - rho, 0.0, gate_z + rb + 0.001),
                    w, t, rb, rho, C, R_wall, th, u, n, E, N, fw_r, fw_y, fw_xmin - 0.003)


# ---------------------------------------------------------------------------
# robot parts

def _wheel(p: SimParams, g: Geometry, i: int) -> str:
    th = math.radians(p.wheel_angles_deg[i - 1])
    c, s = math.cos(th), math.sin(th)
    om, mot = p.omni, p.motor
    R, hw = g.R, om.width / 2
    q = quat_axis((0, 0, 1), th + math.pi)          # hub x = axle, pointing inward
    rr = om.roller_diameter / 2
    d = R - rr
    n_r = om.rollers_per_row
    half_len = R * math.sin(math.pi * 0.62 / n_r)
    rollers = []
    for row, off in enumerate((-hw / 2, hw / 2)):
        for j in range(n_r):
            gam = 2 * math.pi * (j + 0.5 * row) / n_r
            pos = (off, d * math.cos(gam), d * math.sin(gam))
            # roller axis (local x) along the rim tangent (0, -sin, cos): x -> z, then about the axle
            rq = quat_mul(quat_axis((1, 0, 0), gam), quat_axis((0, 1, 0), -math.pi / 2))
            if p.wheel_model == "rollers":
                rollers.append(
                    f'<body pos="{_f(*pos)}" quat="{_f(*rq)}">'
                    # same sideways drag as the sphere model: torque scales with r, viscous with r^2
                    f'<joint type="hinge" axis="1 0 0" damping="{p.roller_damping * (rr / R) ** 2:.3g}" '
                    f'frictionloss="{p.roller_friction_torque * rr / R:.3g}"/>'
                    f'<geom name="roller{i}_{row * n_r + j}" class="roller" mesh="roller{i}" mass="0.004"/></body>')
            else:
                rollers.append(f'<geom class="decor" type="mesh" mesh="roller{i}" material="roller" pos="{_f(*pos)}" quat="{_f(*rq)}"/>')
    contact_mass = 0.06
    hub_mass = max(0.02, om.mass - (contact_mass if p.wheel_model == "sphere" else 0.004 * 2 * n_r))
    if p.wheel_model == "sphere":
        contact = f"""
        <body name="wheel{i}_contact">
          <joint name="roller{i}" type="ball" damping="{p.roller_damping}" frictionloss="{p.roller_friction_torque}"/>
          <geom name="wheel{i}_contact" class="wheel_contact" type="sphere" size="{R}" mass="{contact_mass}"
                friction="{p.mu} 0.005 0.0001"/>
          <site name="wheel{i}_contact" group="5"/>
        </body>"""
    else:
        contact = ""
    rm0 = p.wheel_base_radius - hw - 0.004
    rm1 = rm0 - mot.length
    br = rm1 - 0.004
    return f"""
      <body name="hub{i}" pos="{_f(p.wheel_base_radius * c, p.wheel_base_radius * s, R)}" quat="{_f(*q)}">
        <joint name="drive{i}" type="hinge" axis="1 0 0" armature="{mot.reflected_inertia:.4g}"
               frictionloss="{mot.backdrive_torque}" damping="0.0005"/>
        <geom class="decor" type="cylinder" size="{_f(d - rr * 0.6)}" fromto="{_f(-hw * 0.9, 0, 0, hw * 0.9, 0, 0)}" material="wheel_hub" mass="{hub_mass}"/>
        <geom class="decor" type="cylinder" size="{_f(d + rr * 0.2)}" fromto="{_f(-0.0015, 0, 0, 0.0015, 0, 0)}" material="wheel_plate"/>
        <geom class="decor" type="cylinder" size="0.012" fromto="{_f(-hw - 0.002, 0, 0, hw + 0.002, 0, 0)}" material="steel"/>
        {''.join(rollers)}
        <site name="hub{i}" group="5"/>
        {contact}
      </body>
      <geom class="decor" type="cylinder" size="{_f(mot.diameter / 2)}" material="motor_silver" mass="{mot.mass}"
            fromto="{_f(rm0 * c, rm0 * s, R, (rm0 - 0.006) * c, (rm0 - 0.006) * s, R)}"/>
      <geom class="decor" type="cylinder" size="{_f(mot.diameter / 2 - 0.001)}" material="motor_black"
            fromto="{_f((rm0 - 0.006) * c, (rm0 - 0.006) * s, R, rm1 * c, rm1 * s, R)}"/>
      <geom class="decor" type="box" size="0.004 {_f(mot.diameter / 2 + 0.006)} {_f((g.deck_z - R) / 2 + mot.diameter / 4)}"
            material="petg_dark" mass="0.05"
            pos="{_f(br * c, br * s, (g.deck_z + R) / 2 - mot.diameter / 4)}" quat="{_f(*quat_axis((0, 0, 1), th))}"/>"""


def _funnel(p: SimParams, g: Geometry) -> str:
    alpha = math.radians(p.funnel_wall_angle_deg)
    R, rt, n = p.funnel_rim_radius, p.funnel_throat_radius, p.funnel_panels
    slant = (R - rt) / math.sin(alpha)
    r_mid, z_mid = (R + rt) / 2, (g.rim_z + g.throat_z) / 2
    ht = 0.005           # thick collision panels: no tunneling at 7 m/s
    hw = R * math.tan(math.pi / n) * 1.08
    out = []
    for k in range(n):
        b = 2 * math.pi * k / n
        q = quat_mul(quat_axis((0, 0, 1), b), quat_axis((0, 1, 0), alpha))
        rc, zc = r_mid + ht * math.cos(alpha), z_mid - ht * math.sin(alpha)
        out.append(f'<geom name="funnel{k}" class="funnel_collision" type="box" size="{_f(ht, hw, slant / 2)}" '
                   f'pos="{_f(rc * math.cos(b), rc * math.sin(b), zc)}" quat="{_f(*q)}"/>')
    out.append(f'<geom class="decor" type="mesh" mesh="funnel_net" material="netting" mass="{p.m_funnel * 0.4}"/>')
    rr = R + 0.006
    nr = 32
    for k in range(nr):
        a0, a1 = 2 * math.pi * k / nr, 2 * math.pi * (k + 1) / nr
        out.append(f'<geom class="decor" type="capsule" size="0.007" material="pla_orange" mass="{p.m_funnel * 0.5 / nr:.4g}" '
                   f'fromto="{_f(rr * math.cos(a0), rr * math.sin(a0), g.rim_z, rr * math.cos(a1), rr * math.sin(a1), g.rim_z)}"/>')
    # six printed struts from the rim down to the throat collar
    for k in range(6):
        a = 2 * math.pi * (k + 0.5) / 6
        out.append(f'<geom class="decor" type="capsule" size="0.004" material="petg_dark" '
                   f'fromto="{_f(rr * math.cos(a), rr * math.sin(a), g.rim_z, (rt + 0.01) * math.cos(a), (rt + 0.01) * math.sin(a), g.throat_z)}"/>')
    return "\n        ".join(out)


def _ball_path(p: SimParams, g: Geometry) -> str:
    """Gate, drop tube, J-bend, straight run, flywheels (catch-module frame)."""
    w, t, rb = g.w, g.t, g.rb
    out = []
    # funnel collar between throat and gate
    out.append(f'<geom class="decor" type="cylinder" size="{_f(w + 0.012, 0.006)}" pos="{_f(0, 0, g.throat_z - 0.002)}" material="petg_dark"/>')
    z_top = g.throat_z
    z_low = g.C[1]
    # back wall of the drop tube (continues into the J outer wall)
    hz = (z_top - z_low) / 2
    out.append(f'<geom class="chamber" type="box" size="{_f(t / 2, w + t, hz)}" pos="{_f(-w - t / 2, 0, z_low + hz)}"/>')
    # front wall stops above the J so the ball can turn forward under it
    zf0 = z_low + 0.001
    hzf = (z_top - zf0) / 2
    out.append(f'<geom class="chamber" type="box" size="{_f(t / 2, w + t, hzf)}" pos="{_f(w + t / 2, 0, zf0 + hzf)}"/>')
    # J outer wall: segments around C from straight down to the launch direction
    a_end = math.pi / 2 + g.theta
    nseg = 72            # fine facets: each facet joint costs the rolling ball a little energy
    Rw = g.R_wall + t / 2
    for k in range(nseg):
        a0, a1 = a_end * k / nseg, a_end * (k + 1) / nseg
        am = 0.5 * (a0 + a1)
        cx, cz = g.C[0] - Rw * math.cos(am), g.C[1] - Rw * math.sin(am)
        seg = 2 * Rw * math.sin((a1 - a0) / 2) * 1.08
        # wall tangent direction at am: (sin am, -cos am); box x along tangent
        ang = math.atan2(-math.cos(am), math.sin(am))
        q = quat_axis((0, 1, 0), -ang)
        out.append(f'<geom class="chamber" type="box" size="{_f(seg / 2, w + t, t / 2)}" pos="{_f(cx, 0, cz)}" quat="{_f(*q)}"/>')
    # straight floor after the J, stopping before the nip
    run = p.nip_distance - 0.032
    if run > 0.005:
        fx = g.E[0] - g.n[0] * (rb + t / 2) + g.u[0] * run / 2
        fz = g.E[1] - g.n[1] * (rb + t / 2) + g.u[1] * run / 2
        out.append(f'<geom class="chamber" type="box" size="{_f(run / 2, w + t, t / 2)}" pos="{_f(fx, 0, fz)}" '
                   f'quat="{_f(*quat_axis((0, 1, 0), -g.theta))}"/>')
    # side walls over the tube and J, up to where the flywheels start
    x0, x1 = -w - t, g.side_wall_end
    zs0 = g.C[1] - g.R_wall - t
    for sgn in (-1, 1):
        out.append(f'<geom class="chamber" type="box" size="{_f((x1 - x0) / 2, t / 2, (z_top - zs0) / 2)}" '
                   f'pos="{_f((x0 + x1) / 2, sgn * (w + t / 2), (z_top + zs0) / 2)}"/>')
    # break-beam across the held ball (rangefinder from the back wall, +x)
    out.append(f'<site name="catch_beam" pos="{_f(-w + 0.001, 0, g.hold[2])}" quat="{_f(*quat_axis((0, 1, 0), math.pi / 2))}" group="5"/>')
    # gate: trapdoor hinged along x just outside the -y wall; opens downward 95 deg
    # (negative angle) so the flap ends up flush in a window in that wall
    hy, hz = -w - 0.0015, g.gate_z - 0.0015
    flap_len = 2 * w - 0.0024
    out.append(f"""
        <body name="gate" pos="{_f(0, hy, hz)}">
          <joint name="gate" type="hinge" axis="1 0 0" range="{_f(-p.gate_open_angle - 0.05, 0)}" damping="0.02" armature="2e-4"/>
          <geom name="gate" class="structure" type="box" size="{_f(w - 0.0006, flap_len / 2, 0.0015)}"
                pos="{_f(0, 0.0035 + flap_len / 2, 0)}" material="pla_orange" mass="0.028" contype="0" conaffinity="0"/>
          <geom class="decor" type="cylinder" size="0.0035" fromto="{_f(-w + 0.0006, 0, 0, w - 0.0006, 0, 0)}" material="pla_orange"/>
        </body>
        <geom class="decor" type="box" size="0.018 0.01 0.02" pos="{_f(-w - t - 0.035, hy, hz - 0.01)}" material="servo" mass="0.065"/>""")
    # flywheels: hinge axis = launch-line normal; left spins +, right spins -
    q_fw = quat_axis((0, 1, 0), -g.theta)
    fm = p.fly_motor
    # The foam's squeeze compliance is a sprung slide on each wheel (MuJoCo's soft
    # contacts scale stiffness with effective mass, which is unbounded for a ball
    # pinched between two wheels), so the ball-wheel contact itself can be stiff.
    k_side = p.squeeze_force / (p.squeeze / 2)
    c_side = 2 * 0.3 * math.sqrt(k_side * p.flywheel_mass)
    for side, sgn in (("L", 1), ("R", -1)):
        cx, cz = g.N
        axis = "0 0 1" if sgn > 0 else "0 0 -1"
        hwid = p.flywheel_width / 2
        I = p.flywheel_inertia
        mx, mz = cx - g.n[0] * (hwid + 0.016), cz - g.n[1] * (hwid + 0.016)
        out.append(f"""
        <body name="flywheel_{side}" pos="{_f(cx, sgn * g.fw_y, cz)}" quat="{_f(*q_fw)}">
          <joint name="squeeze_{side}" type="slide" axis="0 {sgn} 0" stiffness="{k_side:.4g}" damping="{c_side:.3g}"
                 range="0 {p.squeeze / 2 + 0.006}"/>
          <joint name="flywheel_{side}" type="hinge" axis="{axis}" damping="0"/>
          <inertial pos="0 0 0" mass="{p.flywheel_mass}" diaginertia="{_f(0.6 * I, 0.6 * I, I)}"/>
          <geom name="flywheel_{side}" type="cylinder" size="{_f(g.fw_r, hwid)}" contype="0" conaffinity="0" group="1" material="tpu_black"/>
          <geom class="decor" type="cylinder" size="{_f(g.fw_r - 0.006, hwid + 0.001)}" material="petg_orange"/>
          <geom class="decor" type="box" size="{_f(g.fw_r - 0.012, 0.003, hwid + 0.0015)}" material="petg_dark"/>
          <geom class="decor" type="box" size="{_f(0.003, g.fw_r - 0.012, hwid + 0.0015)}" material="petg_dark"/>
        </body>
        <geom class="decor" type="cylinder" size="0.0175 0.0125" pos="{_f(mx, sgn * g.fw_y, mz)}" quat="{_f(*q_fw)}" material="motor_black" mass="{fm.mass}"/>
        <geom class="decor" type="cylinder" size="0.012 0.004" pos="{_f(mx - g.n[0] * 0.016, sgn * g.fw_y, mz - g.n[1] * 0.016)}" quat="{_f(*q_fw)}" material="petg_dark"/>""")
    # launcher frame: a printed pedestal under each motor mount
    for sgn in (-1, 1):
        mx = g.N[0] - g.n[0] * (p.flywheel_width / 2 + 0.032)
        mz = g.N[1] - g.n[1] * (p.flywheel_width / 2 + 0.032)
        out.append(f'<geom class="decor" type="capsule" size="0.009" fromto="{_f(mx, sgn * g.fw_y, mz, mx, sgn * g.fw_y, 0.004)}" '
                   f'material="petg_dark" mass="{p.m_launcher_frame / 2}"/>')
    out.append(f'<site name="launch_exit" pos="{_f(g.N[0] + 0.06 * g.u[0], 0, g.N[1] + 0.06 * g.u[1])}" group="5"/>')
    return "\n        ".join(out)


def _catch_module(p: SimParams, g: Geometry) -> str:
    cam_axes = xyaxes_looking_at((0, 0, 0), (math.cos(math.radians(30)), 0, math.sin(math.radians(30))))
    cams = []
    for name, y in (("left", 0.095), ("right", -0.095)):     # rim pads at +/-33.7 deg (cad/catch.py)
        pos = (0.142, y, g.rim_z + 0.04)
        cams.append(f'<geom class="decor" type="box" size="0.012 0.02 0.014" pos="{_f(*pos)}" material="camera_body" mass="{p.m_cameras / 2}"/>'
                    f'<geom class="decor" type="cylinder" size="0.006 0.003" pos="{_f(pos[0] + 0.013, pos[1], pos[2])}" quat="{_f(*quat_axis((0, 1, 0), math.pi / 2))}" material="lens"/>'
                    f'<camera name="stereo_{name}" pos="{_f(pos[0] + 0.017, pos[1], pos[2])}" xyaxes="{cam_axes}" fovy="70"/>')
    return f"""
      <body name="catch_module" pos="0 0 {_f(g.plate_top)}">
        {_funnel(p, g)}
        {_ball_path(p, g)}
        {''.join(cams)}
        <site name="funnel_center" pos="{_f(0, 0, g.rim_z)}" group="5"/>
      </body>"""


def _robot(p: SimParams, g: Geometry) -> str:
    dz = g.deck_z
    top = dz + 0.006
    standoffs = []
    for a in (0, 120, 240):                        # same as cad/dims.py STANDOFF_ANGLES / STANDOFF_R
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        standoffs.append(f'<geom class="decor" type="cylinder" size="0.004" material="aluminum" mass="0.02" '
                         f'fromto="{_f(0.1 * c, 0.1 * s, top, 0.1 * c, 0.1 * s, g.plate_top - 0.006)}"/>')
    bumper = []
    nb, rbump = 36, p.deck_radius + 0.008
    for k in range(nb):
        a0, a1 = 2 * math.pi * k / nb, 2 * math.pi * (k + 1) / nb
        bumper.append(f'<geom class="decor" type="capsule" size="0.014" material="tpu_orange" mass="{p.m_bumper / nb:.4g}" '
                      f'fromto="{_f(rbump * math.cos(a0), rbump * math.sin(a0), dz - 0.004, rbump * math.cos(a1), rbump * math.sin(a1), dz - 0.004)}"/>')
    hx, hy = p.home_xy
    q = quat_axis((0, 0, 1), p.home_yaw)
    pack = p.pack
    wheels = "".join(_wheel(p, g, i) for i in (1, 2, 3))
    return f"""
    <body name="kiwi" pos="{_f(hx, hy, 0)}" quat="{_f(*q)}">
      <freejoint name="kiwi"/>
      <site name="imu" pos="0 0 {_f(top + 0.01)}" group="5"/>
      <geom class="structure" type="cylinder" size="{_f(p.deck_radius, 0.003)}" pos="0 0 {_f(dz + 0.003)}" material="petg_dark" mass="{p.m_deck}"/>
      <geom class="decor" type="cylinder" size="{_f(p.deck_radius - 0.03, 0.0035)}" pos="0 0 {_f(dz + 0.0032)}" material="petg_grey"/>
      <geom class="decor" type="box" size="0.045 0.038 0.036" pos="0 0 {_f(dz - 0.04)}" material="battery" mass="{pack.mass}"/>
      <geom class="decor" type="box" size="0.046 0.039 0.002" pos="0 0 {_f(dz - 0.003)}" material="pla_black" mass="{p.m_lower_frame}"/>
      <geom class="decor" type="box" size="0.05 0.05 0.016" pos="{_f(-0.085, 0, top + 0.016)}" material="jetson" mass="{p.m_jetson}"/>
      <geom class="decor" type="box" size="0.045 0.045 0.007" pos="{_f(-0.085, 0, top + 0.039)}" material="heatsink"/>
      <geom class="decor" type="cylinder" size="0.014 0.006" pos="{_f(-0.085, 0, top + 0.052)}" material="fan"/>
      <geom class="decor" type="box" size="0.035 0.03 0.006" pos="{_f(0.0, 0.105, top + 0.006)}" material="pcb" mass="{p.m_electronics}"/>
      <geom class="decor" type="box" size="0.02 0.012 0.008" pos="{_f(0.0, -0.11, top + 0.008)}" material="pla_black"/>
      <geom class="decor" type="cylinder" size="0.028 0.0165" pos="{_f(0.15, 0, top + 0.0165)}" material="lidar" mass="{p.m_lidar}"/>
      <geom class="decor" type="cylinder" size="0.022 0.006" pos="{_f(0.15, 0, top + 0.039)}" material="lidar_top"/>
      <geom class="decor" type="cylinder" size="0.02 0.01" pos="{_f(-0.185, 0, top + 0.01)}" material="estop_base"/>
      <geom class="decor" type="cylinder" size="0.017 0.009" pos="{_f(-0.185, 0, top + 0.029)}" material="estop"/>
      <geom class="decor" type="box" size="0.01 0.01 0.01" pos="0 0 {_f(top)}" material="pla_black" mass="{p.m_misc}"/>
      {''.join(standoffs)}
      {''.join(bumper)}
      <geom class="structure" type="cylinder" size="0.12 0.003" pos="0 0 {_f(g.plate_top - 0.003)}" material="petg_grey" mass="{p.m_plate}"/>
      {wheels}
      {_catch_module(p, g)}
    </body>"""


def _thrower(p: SimParams) -> str:
    tx, ty = p.thrower_xy
    return f"""
    <body name="thrower" pos="{_f(tx, ty, 0)}">
      <geom class="decor" type="capsule" size="0.065" fromto="0 0.1 0.08 0.02 0.1 0.86" material="jeans"/>
      <geom class="decor" type="capsule" size="0.065" fromto="0 -0.1 0.08 0.02 -0.1 0.86" material="jeans"/>
      <geom class="decor" type="box" size="0.11 0.05 0.03" pos="0.04 0.1 0.03" material="shoe"/>
      <geom class="decor" type="box" size="0.11 0.05 0.03" pos="0.04 -0.1 0.03" material="shoe"/>
      <geom class="decor" type="capsule" size="0.17" fromto="0 0 0.98 0 0 1.32" material="shirt"/>
      <geom class="decor" type="capsule" size="0.05" fromto="0 0 1.48 0 0 1.55" material="skin"/>
      <geom class="decor" type="sphere" size="0.11" pos="0.01 0 1.66" material="skin"/>
      <geom class="decor" type="ellipsoid" size="0.115 0.115 0.06" pos="-0.01 0 1.73" material="hair"/>
      <geom class="decor" type="capsule" size="0.045" fromto="0 0.22 1.42 0.04 0.26 1.1" material="shirt"/>
      <geom class="decor" type="capsule" size="0.04" fromto="0.04 0.26 1.1 0.12 0.25 0.92" material="skin"/>
      <geom class="decor" type="capsule" size="0.045" fromto="0 -0.22 1.42 0.12 -0.2 1.15" material="shirt"/>
      <geom class="decor" type="capsule" size="0.04" fromto="0.12 -0.2 1.15 0.3 -0.12 1.0" material="skin"/>
    </body>"""


# ---------------------------------------------------------------------------
# scene

MATERIALS = """
    <material name="floor" texture="floor_wood" texrepeat="1 1" texuniform="true" reflectance="0.15" specular="0.3" shininess="0.4"/>
    <material name="mat" texture="mat" texrepeat="2 2" texuniform="true" specular="0.1"/>
    <material name="netting" texture="netting" texrepeat="1 1" rgba="1 1 1 0.55" specular="0.1"/>
    <material name="post" rgba="0.2 0.21 0.23 1" specular="0.6" shininess="0.6"/>
    <material name="rope" rgba="0.92 0.92 0.88 1"/>
    <material name="petg_dark" rgba="0.17 0.18 0.2 1" specular="0.35" shininess="0.45"/>
    <material name="petg_grey" rgba="0.42 0.44 0.47 1" specular="0.3" shininess="0.4"/>
    <material name="petg_orange" rgba="0.98 0.45 0.08 1" specular="0.4" shininess="0.5"/>
    <material name="pla_orange" rgba="1.0 0.5 0.1 1" specular="0.3" shininess="0.4"/>
    <material name="pla_black" rgba="0.08 0.08 0.09 1" specular="0.2"/>
    <material name="tpu_black" rgba="0.06 0.06 0.065 1" specular="0.1"/>
    <material name="tpu_orange" rgba="0.95 0.42 0.06 1" specular="0.15"/>
    <material name="aluminum" rgba="0.75 0.77 0.8 1" specular="0.8" shininess="0.8"/>
    <material name="steel" rgba="0.82 0.83 0.85 1" specular="0.9" shininess="0.9"/>
    <material name="motor_silver" rgba="0.72 0.74 0.77 1" specular="0.9" shininess="0.85"/>
    <material name="motor_black" rgba="0.1 0.1 0.11 1" specular="0.6" shininess="0.6"/>
    <material name="wheel_hub" rgba="0.12 0.12 0.13 1" specular="0.3"/>
    <material name="wheel_plate" rgba="0.95 0.78 0.1 1" specular="0.4" shininess="0.5"/>
    <material name="roller" rgba="0.3 0.31 0.33 1" specular="0.1"/>
    <material name="battery" rgba="0.12 0.3 0.7 1" specular="0.4"/>
    <material name="jetson" rgba="0.08 0.35 0.18 1"/>
    <material name="heatsink" rgba="0.15 0.15 0.16 1" specular="0.8" shininess="0.7"/>
    <material name="fan" rgba="0.05 0.05 0.05 1"/>
    <material name="pcb" rgba="0.05 0.32 0.17 1" specular="0.5"/>
    <material name="lidar" rgba="0.08 0.08 0.09 1" specular="0.7" shininess="0.7"/>
    <material name="lidar_top" rgba="0.75 0.12 0.1 1" specular="0.6"/>
    <material name="estop" rgba="0.85 0.08 0.06 1" specular="0.6" shininess="0.6"/>
    <material name="estop_base" rgba="0.95 0.8 0.08 1"/>
    <material name="chamber" rgba="0.75 0.85 0.95 0.28" specular="0.8" shininess="0.9"/>
    <material name="servo" rgba="0.15 0.2 0.6 1"/>
    <material name="camera_body" rgba="0.05 0.05 0.05 1" specular="0.5"/>
    <material name="lens" rgba="0.1 0.2 0.35 1" specular="1" shininess="1"/>
    <material name="ball" rgba="0.82 0.95 0.12 1" specular="0.15" shininess="0.2"/>
    <material name="marker" rgba="0.1 0.85 0.4 0.22" emission="0.4"/>
    <material name="trail" rgba="1 0.9 0.2 0.6"/>
    <material name="skin" rgba="0.87 0.7 0.58 1" specular="0.2"/>
    <material name="hair" rgba="0.2 0.13 0.08 1"/>
    <material name="shirt" rgba="0.2 0.45 0.75 1" specular="0.1"/>
    <material name="jeans" rgba="0.15 0.18 0.3 1"/>
    <material name="shoe" rgba="0.9 0.9 0.92 1"/>
"""


def _solref(stiffness: float, mass: float, restitution: float) -> str:
    """MuJoCo solref (timeconst, dampratio) for a spring-damper contact with this
    stiffness on this mass that bounces back with the given restitution."""
    wn = math.sqrt(stiffness / mass)
    lr = math.log(restitution)
    zeta = -lr / math.sqrt(math.pi ** 2 + lr ** 2)
    return f"{1 / (wn * zeta):.4g} {zeta:.3g}"


def build_xml(p: SimParams = SimParams()) -> str:
    tex = ensure_textures()
    g = geometry(p)
    ax, ay = p.area_size
    wh = p.wall_height
    walls = []
    for name, pos, size in (("wall_px", (ax / 2, 0, wh / 2), (0.01, ay / 2, wh / 2)),
                            ("wall_nx", (-ax / 2, 0, wh / 2), (0.01, ay / 2, wh / 2)),
                            ("wall_py", (0, ay / 2, wh / 2), (ax / 2, 0.01, wh / 2)),
                            ("wall_ny", (0, -ay / 2, wh / 2), (ax / 2, 0.01, wh / 2))):
        walls.append(f'<geom name="{name}" type="box" pos="{_f(*pos)}" size="{_f(*size)}" group="3" rgba="0.2 0.2 0.2 0.1"/>')
    corners = [(-ax / 2, -ay / 2), (ax / 2, -ay / 2), (ax / 2, ay / 2), (-ax / 2, ay / 2)]
    for k, (x0, y0) in enumerate(corners):
        x1, y1 = corners[(k + 1) % 4]
        walls.append(f'<geom class="decor" type="cylinder" size="0.025" fromto="{_f(x0, y0, 0, x0, y0, wh)}" material="post"/>')
        n_mid = int(round(math.hypot(x1 - x0, y1 - y0) / 1.25))
        for j in range(1, n_mid):
            xm, ym = x0 + (x1 - x0) * j / n_mid, y0 + (y1 - y0) * j / n_mid
            walls.append(f'<geom class="decor" type="cylinder" size="0.015" fromto="{_f(xm, ym, 0, xm, ym, wh)}" material="post"/>')
        for z in (0.05, 0.6, 1.2, wh):
            walls.append(f'<geom class="decor" type="capsule" size="0.006" fromto="{_f(x0, y0, z, x1, y1, z)}" material="rope"/>')
    side_cam = (-0.45, -1.92, 1.3)          # inside the netting, near the long side
    rb = g.rb
    ball = p.ball_spec
    ball_solref = _solref(ball.stiffness, ball.mass, ball.restitution)
    # stiff contacts where friction must be Coulomb-like (MuJoCo's soft friction
    # otherwise behaves like a weak damper)
    stiff = 'solref="0.002 1" solimp="0.95 0.99 0.001"'
    pairs = [f'<pair geom1="ball" geom2="flywheel_{s}" condim="3" friction="{p.flywheel_mu} {p.flywheel_mu} 0.005 0.0001 0.0001" '
             f'{stiff}/>' for s in ("L", "R")]
    pairs.append(f'<pair geom1="ball" geom2="gate" condim="3" friction="0.8 0.8 0.005 0.0001 0.0001" '
                 f'solref="{_solref(ball.stiffness, ball.mass, p.gate_pad_restitution)}" solimp="0.95 0.99 0.001"/>')
    # the net is grippy and lossy when the ball rolls over it (rolling-friction term)
    fm, fr = p.funnel_mu, p.funnel_roll
    pairs += [f'<pair geom1="ball" geom2="funnel{k}" condim="6" friction="{fm} {fm} 0.005 {fr} {fr}" solref="0.004 1.0"/>'
              for k in range(p.funnel_panels)]
    wheel_contacts = ([f"wheel{i}_contact" for i in (1, 2, 3)] if p.wheel_model == "sphere"
                      else [f"roller{i}_{j}" for i in (1, 2, 3) for j in range(2 * p.omni.rollers_per_row)])
    mu = p.mu
    pairs += [f'<pair geom1="floor" geom2="{gname}" condim="3" friction="{mu} {mu} 0.005 0.0001 0.0001" '
              f'solref="0.004 1" solreffriction="0.002 1" solimp="0.95 0.99 0.001"/>' for gname in wheel_contacts]
    actuators = []
    for i in (1, 2, 3):
        actuators.append(f'<general name="wheel{i}" joint="drive{i}" gainprm="{p.drive_kp}" biastype="affine" '
                         f'biasprm="0 0 {-p.drive_kp}" forcelimited="true" forcerange="-5 5"/>')
    if p.wheel_model == "sphere":
        for i in (1, 2, 3):
            actuators.append(f'<general name="wheel{i}_coupling" site="wheel{i}_contact" refsite="hub{i}" gear="0 0 0 1 0 0" '
                             f'gainprm="0" biastype="affine" biasprm="0 0 -50"/>')
    actuators += ['<motor name="flywheel_L" joint="flywheel_L" gear="1"/>',
                  '<motor name="flywheel_R" joint="flywheel_R" gear="1"/>',
                  f'<position name="gate" joint="gate" kp="25" kv="0.3" forcelimited="true" forcerange="{_f(-p.gate_torque, p.gate_torque)}"/>']
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
        '<frameangvel name="ball_angvel" objtype="xbody" objname="ball"/>',
        '<jointvel name="flywheel_L_speed" joint="flywheel_L"/>',
        '<jointvel name="flywheel_R_speed" joint="flywheel_R"/>',
        '<jointpos name="gate_pos" joint="gate"/>',
    ]
    for i in (1, 2, 3):
        sensors.append(f'<jointvel name="wheel{i}_speed" joint="drive{i}"/>')
    om = p.omni
    rr = om.roller_diameter / 2
    roller_mesh = "".join(barrel(f"roller{i}", g.R, rr, g.R * math.sin(math.pi * 0.62 / om.rollers_per_row)) for i in (1, 2, 3))
    alpha = math.radians(p.funnel_wall_angle_deg)
    funnel_mesh = cone_shell("funnel_net", p.funnel_throat_radius + 0.002, g.throat_z + 0.002 / math.tan(alpha),
                             p.funnel_rim_radius + 0.002, g.rim_z, n=60, wraps=10)

    return f"""<mujoco model="play_catch_v2">
  <!-- Generated by robot_model.py from params.py / catalog.py; do not edit by hand. -->
  <compiler angle="radian" autolimits="true" texturedir="{tex.as_posix()}"/>
  <option timestep="{p.timestep}" integrator="implicitfast" cone="elliptic" impratio="10"
          density="{p.air_density}" viscosity="{p.air_viscosity}"/>
  <size memory="20M"/>
  <statistic center="-0.4 0 0.7" extent="3.2"/>

  <visual>
    <global offwidth="1920" offheight="1080" azimuth="90" elevation="-15"/>
    <quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.32 0.32 0.34" diffuse="0.35 0.35 0.35" specular="0.08 0.08 0.08"/>
    <map znear="0.005" zfar="60" haze="0.25"/>
    <rgba haze="0.82 0.86 0.92 1"/>
  </visual>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.52 0.66 0.84" rgb2="0.9 0.92 0.95" width="512" height="3072"/>
    <texture name="floor_wood" type="2d" file="floor_wood.png"/>
    <texture name="mat" type="2d" file="mat.png"/>
    <texture name="netting" type="2d" file="netting.png"/>
    {MATERIALS}
    {roller_mesh}
    {funnel_mesh}
  </asset>

  <default>
    <geom solref="0.01 1"/>
    <default class="structure">
      <geom group="0" contype="1" conaffinity="1" material="petg_dark"/>
    </default>
    <default class="chamber">
      <geom group="0" contype="1" conaffinity="1" material="chamber" mass="0.004" friction="0.35 0.005 0.0001"/>
    </default>
    <default class="decor">
      <geom group="1" contype="0" conaffinity="0" density="0"/>
    </default>
    <default class="wheel_contact">
      <geom group="3" contype="0" conaffinity="0" rgba="0.9 0.2 0.2 0.3" condim="3"/>
    </default>
    <default class="roller">
      <geom type="mesh" group="1" contype="0" conaffinity="0" material="roller" condim="3"/>
    </default>
    <default class="funnel_collision">
      <geom group="3" contype="0" conaffinity="0" mass="{p.m_funnel * 0.1 / p.funnel_panels:.4g}" rgba="0.2 0.6 0.9 0.3"/>
    </default>
  </default>

  <worldbody>
    <light name="key" pos="2 -3 6" dir="-0.3 0.45 -1" directional="true" castshadow="true" diffuse="0.62 0.6 0.56" specular="0.25 0.25 0.25"/>
    <light name="fill" pos="-4 3 4" dir="0.6 -0.4 -0.6" directional="true" castshadow="false" diffuse="0.22 0.24 0.28"/>
    <light name="rim" pos="0 4 3" dir="0 -0.8 -0.5" directional="true" castshadow="false" diffuse="0.18 0.18 0.2"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="floor" contype="1" conaffinity="3" friction="{p.floor_mu} 0.005 0.0001"/>
    {''.join(walls)}
    <camera name="side" pos="{_f(*side_cam)}" xyaxes="{xyaxes_looking_at(side_cam, (-0.45, 0.0, 1.0))}" fovy="62"/>
    <camera name="robot_close" mode="targetbody" target="kiwi" pos="2.4 -1.3 1.0" fovy="40"/>
    <camera name="overhead" pos="-0.4 0 6.2" xyaxes="1 0 0 0 1 0" fovy="50"/>
    {_thrower(p)}
    <body name="landing_marker" mocap="true" pos="{_f(*p.home_xy, -0.05)}">
      <geom class="decor" type="cylinder" size="{_f(p.funnel_rim_radius, 0.0008)}" material="marker"/>
    </body>
    <body name="ball" pos="{_f(p.thrower_xy[0] + 0.3, p.thrower_xy[1] + 0.4, rb + 0.002)}">
      <freejoint name="ball"/>
      <geom name="ball" type="sphere" size="{rb}" mass="{ball.mass}" material="ball" priority="1"
            contype="1" conaffinity="1" condim="6" friction="{ball.mu} 0.005 0.002"
            solref="{ball_solref}" solimp="0.95 0.99 0.001"
            fluidshape="ellipsoid" fluidcoef="{_f(ball.drag_cd / 2, 0.25, 1.5, 1.0, 1.0)}"/>
    </body>
    {_robot(p, g)}
  </worldbody>

  <contact>
    {''.join(pairs)}
  </contact>

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
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    kiwi = model.body("kiwi").id
    mass = model.body_subtreemass[kiwi]
    cg = data.subtree_com[kiwi] - data.xpos[kiwi]
    edge = p.wheel_base_radius / 2
    return (f"robot {mass:.2f} kg, CG {cg[2] * 100:.1f} cm high; tip-over {9.81 * edge / cg[2]:.1f} m/s^2 "
            f"toward a wheel-triangle edge; {p.motor.name} on {p.omni.name}")


if __name__ == "__main__":
    params = SimParams()
    out = Path(__file__).with_name("scene.xml")
    out.write_text(build_xml(params))
    print(f"wrote {out}")
    print(describe(build_model(params), params))
