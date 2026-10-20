"""MJCF builder for the v3 compact play-catch robot and its test area.

Frames
  World: z up, floor at z = 0, thrower on the -x side.
  Robot "base": origin on the floor under the base center, x forward (launcher), y left.
  Tilt body: on the pivot axis (robot y), hinge "tilt"; + tips the funnel toward +x.
  Catch module: the v2 module frame (x forward, z up along the funnel axis, origin
  at the module base on the funnel axis), hung from the tilt body at -pivot.

Ball path in the module (as v2): funnel throat -> pocket -> trapdoor gate
(break-beam across the held ball) -> drop tube -> J-bend -> straight run up the
launch line -> nip between left/right flywheels. The launch line is at
launch_angle_deg in the module, so the world elevation is that minus the tilt.

Omni wheels: as v2 ("sphere" fast model or explicit "rollers"), plus a vertical
slide on each hub whose spring is the rubber rollers' compliance. With four
wheels the loads are statically indeterminate, so that compliance (and any floor
dip, wheel_z_offset) sets how they share the weight. The slide position reads
out each wheel's normal load. Optional rocker: the two front wheels on a beam
pivoting about x, with end stops at +/- rocker_deg.

Collision groups (contype / conaffinity bits)
  1 = general (ball, structure, walls)   2 = wheel contacts (floor only)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

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
    """Derived dimensions shared by the model builder and the robot code (module frame unless noted)."""
    R: float              # wheel radius
    deck_z: float         # deck underside (robot frame)
    plate_top: float      # payload plate top (robot frame)
    rim_z: float
    throat_z: float
    gate_z: float         # j_bend: gate plate top; kicker: floor height at the back corner
    hold: tuple           # held-ball center
    w: float
    t: float
    rb: float
    rho: float
    C: tuple              # j_bend: J center (x, z); kicker: resting ball center (x, z)
    R_wall: float
    theta: float          # launch angle in the module
    u: tuple
    n: tuple
    E: tuple              # j_bend: ball center at the end of the J; kicker: the back-bottom corner (paddle hinge)
    N: tuple
    fw_r: float
    fw_y: float
    side_wall_end: float
    pivot_m: tuple        # pivot (x, z) in the module frame
    pivot_r: tuple        # pivot (x, z) in the robot frame
    drop: float           # j_bend: held-ball center to J bottom (gravity-feed head); kicker: 0
    feed: str = "j_bend"
    floor_len: float = 0.0    # kicker: inclined floor length from the corner along u
    front_wall_z: float = 0.0  # kicker: bottom of the pocket's front wall (above the launch path)

    def rim_center_robot(self, tilt: float) -> np.ndarray:
        """Funnel rim center in the robot frame at a tilt angle (x, z)."""
        return self.module_to_robot((0.0, self.rim_z), tilt)

    def module_to_robot(self, xz, tilt: float) -> np.ndarray:
        dx, dz = xz[0] - self.pivot_m[0], xz[1] - self.pivot_m[1]
        c, s = math.cos(tilt), math.sin(tilt)
        return np.array([self.pivot_r[0] + c * dx + s * dz, self.pivot_r[1] - s * dx + c * dz])


def _flywheel_low_points(p: SimParams, u, n, N, fw_r) -> list:
    hw = p.flywheel_width / 2
    pts = []
    for k in range(12):
        s = 2 * math.pi * k / 12
        for side in (-1, 1):
            pts.append((N[0] + fw_r * math.cos(s) * u[0] + side * hw * n[0],
                        N[1] + fw_r * math.cos(s) * u[1] + side * hw * n[1]))
    m = (N[0] - n[0] * (hw + 0.016), N[1] - n[1] * (hw + 0.016))
    for a in (-1, 1):
        for b in (-1, 1):
            pts.append((m[0] + a * 0.0175 * u[0] + b * 0.0125 * n[0], m[1] + a * 0.0175 * u[1] + b * 0.0125 * n[1]))
    return pts


def _module_low_points(p: SimParams, C, R_wall, t, u, n, N, fw_r, side_end, gate_z, w) -> list:
    """(x, z) points on the J-bend module's outline that could hit the payload plate when it tilts."""
    th = math.radians(p.launch_angle_deg)
    a_end = math.pi / 2 + th
    pts = []
    Rw = R_wall + t
    for k in range(13):
        a = a_end * k / 12
        pts.append((C[0] - Rw * math.cos(a), C[1] - Rw * math.sin(a)))
    zs0 = C[1] - R_wall - t
    pts += [(-w - t, zs0), (side_end, zs0), (-w - t, gate_z + p.pocket_depth)]
    pts += _flywheel_low_points(p, u, n, N, fw_r)
    # gate servo
    pts += [(-w - t - 0.053, gate_z - 0.031), (-w - t - 0.017, gate_z - 0.031)]
    return pts


def _pivot(p: SimParams, pts, plate_top: float):
    """Pivot (module frame, robot frame) so the module clears the plate over the whole tilt range."""
    xp, zp = p.tilt_pivot
    lo, hi = p.tilt_limits
    z_min = 0.0
    for phi in np.linspace(lo, hi, 25):
        c, s = math.cos(phi), math.sin(phi)
        z_min = min(z_min, min(-s * (x - xp) + c * (z - zp) for x, z in pts))
    return (xp, zp), (xp, plate_top + p.tilt_clearance - z_min)    # funnel axis over the base center at tilt 0


def geometry(p: SimParams) -> Geometry:
    return _geometry_kicker(p) if p.feed == "kicker" else _geometry_j(p)


def _geometry_j(p: SimParams) -> Geometry:
    R = p.wheel_radius
    rb = p.ball_spec.radius
    t = 0.004
    deck_z = p.deck_height
    plate_top = p.plate_height
    w = p.tube_half_width
    rho = p.j_radius
    R_wall = rho + rb
    z_b = R_wall + t + 0.006
    C = (-w + R_wall, z_b)
    gate_z = z_b + p.drop_gap
    throat_z = gate_z + p.pocket_depth
    rim_z = throat_z + p.funnel_depth
    th = math.radians(p.launch_angle_deg)
    u = (math.cos(th), math.sin(th))
    n = (-math.sin(th), math.cos(th))
    a_end = math.pi / 2 + th
    E = (C[0] - rho * math.cos(a_end), C[1] - rho * math.sin(a_end))
    N = (E[0] + p.nip_distance * u[0], E[1] + p.nip_distance * u[1])
    fw_r = p.flywheel_diameter / 2
    fw_y = fw_r + rb - p.squeeze / 2
    fw_xmin = N[0] - fw_r * u[0] - p.flywheel_width / 2 * abs(n[0])
    side_end = fw_xmin - 0.003
    hold = (C[0] - rho, 0.0, gate_z + rb + 0.001)
    pts = _module_low_points(p, C, R_wall, t, u, n, N, fw_r, side_end, gate_z, w)
    pivot_m, pivot_r = _pivot(p, pts, plate_top)
    drop = hold[2] - (C[1] - rho)
    return Geometry(R, deck_z, plate_top, rim_z, throat_z, gate_z, hold, w, t, rb, rho, C, R_wall, th, u, n,
                    E, N, fw_r, fw_y, side_end, pivot_m, pivot_r, drop)


def _geometry_kicker(p: SimParams) -> Geometry:
    """Kicker feed, module frame (x forward, z up the funnel axis, origin at the module base).

    The pocket under the throat has a floor inclined at the launch angle (rising forward). The
    caught ball rolls back into the corner between that floor and a plunger whose face stands
    perpendicular to it (the floor slopes back at every catch tilt as long as the tilt stays below
    the launch angle). To launch, the plunger slides up the floor and pushes the ball into the
    flywheel nip N; the 90 deg corner can't wedge the ball. The launch line is shallower than the
    funnel wall, so the throat is raised until the ball's path clears the underside of the funnel.
    """
    R = p.wheel_radius
    rb = p.ball_spec.radius
    t = 0.004
    tp = 0.003                                     # plunger plate thickness
    deck_z = p.deck_height
    plate_top = p.plate_height
    w = p.tube_half_width
    th = math.radians(p.launch_angle_deg)
    u = (math.cos(th), math.sin(th))
    n = (-math.sin(th), math.cos(th))
    hp = p.plunger_height
    # corner F0 = floor x plunger face at rest; the plunger leans back by th and must stay inside the tube
    xF = -w + tp * u[0] + hp * math.sin(th) + 0.002
    z0 = t + 0.004 + (xF + w) * math.tan(th)       # the floor's low end (at the back wall) stays above the base
    F0 = (xF, z0)
    H = (xF + rb * u[0] + rb * n[0], z0 + rb * u[1] + rb * n[1])
    d = p.kicker_nip_distance
    N = (H[0] + d * u[0], H[1] + d * u[1])
    fw_r = p.flywheel_diameter / 2
    fw_y = fw_r + rb - p.squeeze / 2
    fw_xmin = N[0] - fw_r * u[0] - p.flywheel_width / 2 * abs(n[0])
    side_end = fw_xmin - 0.003
    s_N = (N[0] - F0[0]) * u[0] + (N[1] - F0[1]) * u[1]
    floor_len = s_N - 0.03
    # throat: above the resting ball, and high enough that the launched ball passes under the funnel
    a_w = math.radians(p.funnel_wall_angle_deg)
    rt, Rr = p.funnel_throat_radius, p.funnel_rim_radius
    throat_z = H[1] + rb + p.pocket_clearance
    for k in range(80):
        s = 0.4 * k / 79
        cx, cz = H[0] + s * u[0], H[1] + s * u[1]
        if cx < rt + 0.03:          # under the throat collar, which hangs ~15 mm below the throat
            throat_z = max(throat_z, cz + rb + 0.015)
        if rt - rb < cx < Rr + rb:  # under the cone: 8 mm of air plus the 10 mm funnel wall (net + frame)
            r = max(cx, rt)
            throat_z = max(throat_z, cz + (rb + 0.018) / math.sin(a_w) - (r - rt) / math.tan(a_w))
    rim_z = throat_z + p.funnel_depth
    # the pocket's front wall stops above the launch path where it crosses x = +w
    cz_w = H[1] + (w - H[0]) * math.tan(th)
    front_wall_z = cz_w + rb / math.cos(th) + 0.004
    hold = (H[0], 0.0, H[1])
    z_back = z0 - (xF + w) * math.tan(th)          # floor top at the back wall
    pts = [(-w - t, z_back - t - 0.004),
           (F0[0] + floor_len * u[0] - t * n[0], F0[1] + floor_len * u[1] - t * n[1]),
           (side_end, z_back - t - 0.004)]
    pts += _flywheel_low_points(p, u, n, N, fw_r)
    pts += [(-w - 0.01, z_back - 0.01), (-w + 0.03, z_back - 0.01)]   # plunger servo + link beside the pocket
    pivot_m, pivot_r = _pivot(p, pts, plate_top)
    return Geometry(R, deck_z, plate_top, rim_z, throat_z, z0, hold, w, t, rb, 0.0, H, rb, th, u, n,
                    F0, N, fw_r, fw_y, side_end, pivot_m, pivot_r, 0.0, feed="kicker",
                    floor_len=floor_len, front_wall_z=front_wall_z)


def _wheel(p: SimParams, g: Geometry, i: int, origin=(0.0, 0.0, 0.0)) -> str:
    """Wheel i (1-based) with its motor and pod; positions relative to origin (robot frame)."""
    th = math.radians(p.wheel_angles_deg[i - 1])
    c, s = math.cos(th), math.sin(th)
    om, mot = p.omni, p.motor
    R, hw = g.R, om.width / 2
    q = quat_axis((0, 0, 1), th + math.pi)          # hub x = axle, pointing inward
    rr = om.roller_diameter / 2
    d = R - rr
    n_r = om.rollers_per_row
    rollers = []
    for row, off in enumerate((-hw / 2, hw / 2)):
        for j in range(n_r):
            gam = 2 * math.pi * (j + 0.5 * row) / n_r
            pos = (off, d * math.cos(gam), d * math.sin(gam))
            rq = quat_mul(quat_axis((1, 0, 0), gam), quat_axis((0, 1, 0), -math.pi / 2))
            if p.wheel_model == "rollers":
                rollers.append(
                    f'<body pos="{_f(*pos)}" quat="{_f(*rq)}">'
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
    off = p.wheel_z_offset[i - 1] if len(p.wheel_z_offset) >= i else 0.0
    L = p.wheel_base_radius
    ox, oy, oz = origin
    # roller compliance: the hub rides a vertical slide; the spring is the loaded rubber
    k = p.roller_stiffness
    m_share = 6.0 / p.n_wheels
    c_susp = 2 * p.roller_damping_ratio * math.sqrt(k * m_share)
    rm0 = L - hw - 0.004
    rm1 = rm0 - mot.length
    br = rm1 - 0.004
    if p.deck_mode == "axle":        # bulkhead at axle height, bolted into the deck slot
        pod_hz, pod_z = mot.diameter / 2, R
    else:                            # pod hanging from the deck above the wheel
        pod_hz, pod_z = (g.deck_z - R) / 2 + mot.diameter / 4, (g.deck_z + R) / 2 - mot.diameter / 4
    return f"""
      <body name="hub{i}" pos="{_f(L * c - ox, L * s - oy, R - off - oz)}" quat="{_f(*q)}">
        <joint name="susp{i}" type="slide" axis="0 0 1" stiffness="{k:.4g}" damping="{c_susp:.4g}" range="0 0.006"/>
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
            fromto="{_f(rm0 * c - ox, rm0 * s - oy, R - oz, (rm0 - 0.006) * c - ox, (rm0 - 0.006) * s - oy, R - oz)}"/>
      <geom class="decor" type="cylinder" size="{_f(mot.diameter / 2 - 0.001)}" material="motor_black"
            fromto="{_f((rm0 - 0.006) * c - ox, (rm0 - 0.006) * s - oy, R - oz, rm1 * c - ox, rm1 * s - oy, R - oz)}"/>
      <geom class="decor" type="box" size="0.004 {_f(mot.diameter / 2 + 0.006)} {_f(pod_hz)}"
            material="petg_dark" mass="{p.m_pod}"
            pos="{_f(br * c - ox, br * s - oy, pod_z - oz)}" quat="{_f(*quat_axis((0, 0, 1), th))}"/>"""


def _wheels(p: SimParams, g: Geometry) -> str:
    n = p.n_wheels
    if p.rocker_deg <= 0 or n != 4:
        return "".join(_wheel(p, g, i) for i in range(1, n + 1))
    # rocker: the two front wheels (1 and 4 in the X layout) on a beam pivoting about x
    L = p.wheel_base_radius
    a0 = math.radians(p.wheel_angles_deg[0])
    piv = (L * math.cos(a0), 0.0, g.R)
    r = math.radians(p.rocker_deg)
    fixed = "".join(_wheel(p, g, i) for i in (2, 3))
    front = "".join(_wheel(p, g, i, piv) for i in (1, 4))
    return f"""{fixed}
      <body name="rocker" pos="{_f(*piv)}">
        <joint name="rocker" type="hinge" axis="1 0 0" range="{_f(-r, r)}" damping="0.05"/>
        <geom class="decor" type="box" size="0.012 {_f(L * math.sin(a0))} 0.006" pos="0 0 0.03" material="petg_dark" mass="0.06"/>
        {front}
      </body>"""


def _funnel(p: SimParams, g: Geometry) -> str:
    alpha = math.radians(p.funnel_wall_angle_deg)
    R, rt, n = p.funnel_rim_radius, p.funnel_throat_radius, p.funnel_panels
    slant = (R - rt) / math.sin(alpha)
    r_mid, z_mid = (R + rt) / 2, (g.rim_z + g.throat_z) / 2
    ht = 0.005
    hw = R * math.tan(math.pi / n) * 1.08
    out = []
    for k in range(n):
        b = 2 * math.pi * k / n
        q = quat_mul(quat_axis((0, 0, 1), b), quat_axis((0, 1, 0), alpha))
        rc, zc = r_mid + ht * math.cos(alpha), z_mid - ht * math.sin(alpha)
        out.append(f'<geom name="funnel{k}" class="funnel_collision" type="box" size="{_f(ht, hw, slant / 2)}" '
                   f'pos="{_f(rc * math.cos(b), rc * math.sin(b), zc)}" quat="{_f(*q)}"/>')
    # group 2: the vision model ray-casts the net separately (fabric: part of the ball shows through)
    out.append(f'<geom class="decor" type="mesh" mesh="funnel_net" material="netting" group="2" mass="{p.m_funnel * 0.4}"/>')
    rr = R + 0.006
    nr = 32
    for k in range(nr):
        a0, a1 = 2 * math.pi * k / nr, 2 * math.pi * (k + 1) / nr
        out.append(f'<geom class="decor" type="capsule" size="0.007" material="pla_orange" mass="{p.m_funnel * 0.5 / nr:.4g}" '
                   f'fromto="{_f(rr * math.cos(a0), rr * math.sin(a0), g.rim_z, rr * math.cos(a1), rr * math.sin(a1), g.rim_z)}"/>')
    for k in range(6):
        a = 2 * math.pi * (k + 0.5) / 6
        out.append(f'<geom class="decor" type="capsule" size="0.004" material="petg_dark" '
                   f'fromto="{_f(rr * math.cos(a), rr * math.sin(a), g.rim_z, (rt + 0.01) * math.cos(a), (rt + 0.01) * math.sin(a), g.throat_z)}"/>')
    return "\n        ".join(out)


def _hopper(p: SimParams, g: Geometry) -> str:
    """Multi-ball feed parts in the module frame (params.hopper)."""
    if p.hopper == "tube":
        # single-file magazine: a 12-sided tube, 4 mm radial clearance, standing on the throat
        n, ri, ht = 12, p.ball_spec.radius + 0.004, 0.002
        H = p.hopper_tube_height
        hw = ri * math.tan(math.pi / n) * 1.1
        out = []
        for k in range(n):
            b = 2 * math.pi * k / n
            rc = ri + ht
            out.append(f'<geom class="chamber" type="box" size="{_f(ht, hw, H / 2)}" '
                       f'pos="{_f(rc * math.cos(b), rc * math.sin(b), g.throat_z + H / 2)}" '
                       f'quat="{_f(*quat_axis((0, 0, 1), b))}" mass="{p.m_hopper / n:.4g}"/>')
        return "\n        ".join(out)
    if p.hopper == "sweeper":
        # a 6 mm wire arm lying 4 mm above the net, from the throat collar to the rim, on a hinge about the
        # funnel axis (in hardware: fixed to a rim ring turned by a gearmotor; parked, it is flush with the net)
        a = math.radians(p.funnel_wall_angle_deg)
        rw = 0.003
        off = rw + 0.004
        rt, R = p.funnel_throat_radius, p.funnel_rim_radius
        # cone surface point at radius r: z = throat_z + (r - rt) / tan(a); inward normal (-cos a, sin a)
        p0 = (rt + 0.012, g.throat_z + 0.012 / math.tan(a))
        p1 = (R - 0.012, g.throat_z + (R - 0.012 - rt) / math.tan(a))
        n_in = (-math.cos(a), math.sin(a))
        q0 = (p0[0] + off * n_in[0], p0[1] + off * n_in[1])
        q1 = (p1[0] + off * n_in[0], p1[1] + off * n_in[1])
        return f"""
        <body name="sweeper">
          <joint name="sweeper" type="hinge" axis="0 0 1" damping="0.002" armature="1e-4"/>
          <geom name="sweeper" type="capsule" size="{rw}" fromto="{_f(q0[0], 0, q0[1], q1[0], 0, q1[1])}"
                contype="1" conaffinity="1" material="pla_orange" mass="{p.m_hopper * 0.3:.4g}" friction="0.4 0.005 0.0001"/>
        </body>
        <geom class="decor" type="box" size="0.015 0.012 0.012" pos="{_f(R + 0.03, 0, g.rim_z - 0.01)}" material="servo"
              mass="{p.m_hopper * 0.7:.4g}"/>"""
    return ""


def _ball_path(p: SimParams, g: Geometry) -> str:
    """Pocket, gate, drop tube, J-bend, straight run, flywheels (module frame); as v2."""
    w, t, rb = g.w, g.t, g.rb
    out = []
    out.append(f'<geom class="decor" type="cylinder" size="{_f(w + 0.012, 0.006)}" pos="{_f(0, 0, g.throat_z - 0.002)}" material="petg_dark"/>')
    z_top = g.throat_z
    z_low = g.C[1]
    hz = (z_top - z_low) / 2
    out.append(f'<geom class="chamber" type="box" size="{_f(t / 2, w + t, hz)}" pos="{_f(-w - t / 2, 0, z_low + hz)}"/>')
    zf0 = z_low + 0.001
    hzf = (z_top - zf0) / 2
    out.append(f'<geom class="chamber" type="box" size="{_f(t / 2, w + t, hzf)}" pos="{_f(w + t / 2, 0, zf0 + hzf)}"/>')
    a_end = math.pi / 2 + g.theta
    nseg = 72
    Rw = g.R_wall + t / 2
    for k in range(nseg):
        a0, a1 = a_end * k / nseg, a_end * (k + 1) / nseg
        am = 0.5 * (a0 + a1)
        cx, cz = g.C[0] - Rw * math.cos(am), g.C[1] - Rw * math.sin(am)
        seg = 2 * Rw * math.sin((a1 - a0) / 2) * 1.08
        ang = math.atan2(-math.cos(am), math.sin(am))
        q = quat_axis((0, 1, 0), -ang)
        out.append(f'<geom class="chamber" type="box" size="{_f(seg / 2, w + t, t / 2)}" pos="{_f(cx, 0, cz)}" quat="{_f(*q)}"/>')
    run = p.nip_distance - 0.032
    if run > 0.005:
        fx = g.E[0] - g.n[0] * (rb + t / 2) + g.u[0] * run / 2
        fz = g.E[1] - g.n[1] * (rb + t / 2) + g.u[1] * run / 2
        out.append(f'<geom class="chamber" type="box" size="{_f(run / 2, w + t, t / 2)}" pos="{_f(fx, 0, fz)}" '
                   f'quat="{_f(*quat_axis((0, 1, 0), -g.theta))}"/>')
    x0, x1 = -w - t, g.side_wall_end
    zs0 = g.C[1] - g.R_wall - t
    for sgn in (-1, 1):
        out.append(f'<geom class="chamber" type="box" size="{_f((x1 - x0) / 2, t / 2, (z_top - zs0) / 2)}" '
                   f'pos="{_f((x0 + x1) / 2, sgn * (w + t / 2), (z_top + zs0) / 2)}"/>')
    out.append(f'<site name="catch_beam" pos="{_f(-w + 0.001, 0, g.hold[2])}" quat="{_f(*quat_axis((0, 1, 0), math.pi / 2))}" group="5"/>')
    if p.throat_flaps:
        # one-way trap: each flap hinges on a side wall at the top of the pocket, points inward and
        # slightly up, and can only fold down (joint range), so a rebounding ball meets a closed stop.
        # The hinge sits in the wall and the flap folds into a window in it, flush (the tube has only
        # 2 mm of side clearance, so a flap folding against a solid wall would wedge the ball).
        yh = w + 0.0025
        L = yh - 0.004
        beta = math.radians(p.flap_up_deg)
        for side, s in (("L", 1), ("R", -1)):
            out.append(f"""
        <body name="flap_{side}" pos="{_f(0, s * yh, g.throat_z - 0.004)}" quat="{_f(*quat_axis((1, 0, 0), -s * beta))}">
          <joint name="flap_{side}" type="hinge" axis="{s} 0 0" range="0 {_f(beta + math.pi / 2 + 0.03)}" stiffness="{p.flap_stiffness}"
                 springref="{-p.flap_preload}" damping="2e-5" armature="1e-6" solreflimit="0.002 1"/>
          <geom name="flap_{side}" type="box" size="{_f(w - 0.0015, L / 2, 0.00075)}" pos="{_f(0, -s * L / 2, 0)}"
                contype="0" conaffinity="0" group="1" material="tpu_orange" mass="0.002"/>
        </body>""")
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
    out += _launcher(p, g, bracket_z=g.C[1])
    return "\n        ".join(out)


def _ball_path_kicker(p: SimParams, g: Geometry) -> str:
    """Pocket with an inclined floor, plunger, short channel, flywheels (module frame)."""
    w, t, rb = g.w, g.t, g.rb
    u, n, th = g.u, g.n, g.theta
    F0 = g.E
    out = [f'<geom class="decor" type="cylinder" size="{_f(w + 0.012, 0.006)}" pos="{_f(0, 0, g.throat_z - 0.002)}" material="petg_dark"/>']
    z_top = g.throat_z
    # inclined floor from the back wall to just short of the flywheels, padded (pair in build_xml)
    s0 = -(F0[0] + w) / math.cos(th)
    s1 = g.floor_len
    sc = 0.5 * (s0 + s1)
    fc = (F0[0] + sc * u[0] - t / 2 * n[0], F0[1] + sc * u[1] - t / 2 * n[1])
    out.append(f'<geom name="pocket_floor" class="chamber" type="box" size="{_f((s1 - s0) / 2, w + t, t / 2)}" '
               f'pos="{_f(fc[0], 0, fc[1])}" quat="{_f(*quat_axis((0, 1, 0), -th))}" contype="0" conaffinity="0"/>')
    zb = F0[1] + s0 * u[1] - t
    out.append(f'<geom class="chamber" type="box" size="{_f(t / 2, w + t, (z_top - zb) / 2)}" pos="{_f(-w - t / 2, 0, (z_top + zb) / 2)}"/>')
    if p.pocket_detent > 0:
        # retaining ridge across the floor, touching the front of the resting ball: to leave, the ball must climb
        # it (~g tan(slope + climb angle)), which base accelerations can't do but the plunger easily does
        hd, rd = p.pocket_detent, 0.003
        s_d = rb + math.sqrt(rb * rb - (rb - hd) ** 2) + rd
        cxd = (F0[0] + s_d * u[0] + (hd - rd) * n[0], F0[1] + s_d * u[1] + (hd - rd) * n[1])
        out.append(f'<geom name="detent" class="chamber" type="capsule" size="{rd}" '
                   f'fromto="{_f(cxd[0], -w + 0.002, cxd[1], cxd[0], w - 0.002, cxd[1])}" material="pla_orange"/>')
    if g.front_wall_z < z_top - 0.005:
        out.append(f'<geom class="chamber" type="box" size="{_f(t / 2, w + t, (z_top - g.front_wall_z) / 2)}" '
                   f'pos="{_f(w + t / 2, 0, (z_top + g.front_wall_z) / 2)}"/>')
    x0, x1 = -w - t, g.side_wall_end
    zs0 = zb - 0.004
    for sgn in (-1, 1):
        out.append(f'<geom class="chamber" type="box" size="{_f((x1 - x0) / 2, t / 2, (z_top - zs0) / 2)}" '
                   f'pos="{_f((x0 + x1) / 2, sgn * (w + t / 2), (z_top + zs0) / 2)}"/>')
    # break-beam across the resting ball, side to side
    out.append(f'<site name="catch_beam" pos="{_f(g.hold[0], -w + 0.001, g.hold[2])}" '
               f'quat="{_f(*quat_axis((1, 0, 0), -math.pi / 2))}" group="5"/>')
    # plunger: a plate standing perpendicular to the floor, sliding up it (joint "gate", metres)
    tp, hp = 0.003, p.plunger_height
    pc = (-tp / 2 * u[0] + hp / 2 * n[0], -tp / 2 * u[1] + hp / 2 * n[1])
    out.append(f"""
        <body name="gate" pos="{_f(F0[0], 0, F0[1])}">
          <joint name="gate" type="slide" axis="{_f(u[0], 0, u[1])}" range="{_f(0, p.kick_stroke + 0.005)}" damping="2" armature="0.02"/>
          <geom name="gate" type="box" size="{_f(tp / 2, w - 0.002, hp / 2)}" pos="{_f(pc[0], 0, pc[1])}"
                quat="{_f(*quat_axis((0, 1, 0), -th))}" material="pla_orange" mass="0.02" contype="0" conaffinity="0" group="1"/>
        </body>
        <geom class="decor" type="box" size="0.02 0.01 0.018" pos="{_f(-w + 0.01, -w - t - 0.012, F0[1] - 0.004)}" material="servo" mass="0.065"/>""")
    out += _launcher(p, g, bracket_z=g.hold[2])
    return "\n        ".join(out)


def _launcher(p: SimParams, g: Geometry, bracket_z: float) -> list:
    """Flywheels on sprung slides, their motors, frame brackets and the launch-exit site."""
    w, t = g.w, g.t
    out = []
    q_fw = quat_axis((0, 1, 0), -g.theta)
    fm = p.fly_motor
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
        <geom class="decor" type="cylinder" size="0.0175 0.0125" pos="{_f(mx, sgn * g.fw_y, mz)}" quat="{_f(*q_fw)}" material="motor_black" mass="{fm.mass}"/>""")
    # launcher frame: one printed bracket per side from the motor mount back to the tube walls
    for sgn in (-1, 1):
        mx = g.N[0] - g.n[0] * (p.flywheel_width / 2 + 0.032)
        mz = g.N[1] - g.n[1] * (p.flywheel_width / 2 + 0.032)
        out.append(f'<geom class="decor" type="capsule" size="0.008" fromto="{_f(mx, sgn * g.fw_y, mz, -w - t, sgn * (w + t + 0.006), bracket_z)}" '
                   f'material="petg_dark" mass="{p.m_launcher_frame / 2}"/>')
    out.append(f'<site name="launch_exit" pos="{_f(g.N[0] + 0.06 * g.u[0], 0, g.N[1] + 0.06 * g.u[1])}" group="5"/>')
    return out


def _camera(name: str, pos, p: SimParams, axis_extra: str = "") -> str:
    ct = math.radians(p.camera_tilt_deg)
    axes = xyaxes_looking_at((0, 0, 0), (math.cos(ct), 0, math.sin(ct)))
    lens = (pos[0] + 0.013 * math.cos(ct), pos[1], pos[2] + 0.013 * math.sin(ct))
    return (f'<geom class="decor" type="box" size="0.012 0.02 0.014" pos="{_f(*pos)}" quat="{_f(*quat_axis((0, 1, 0), -ct))}" '
            f'material="camera_body" mass="{p.m_cameras / 2}"/>'
            f'<geom class="decor" type="cylinder" size="0.006 0.003" pos="{_f(*lens)}" quat="{_f(*quat_axis((0, 1, 0), math.pi / 2 - ct))}" material="lens"/>'
            f'<camera name="stereo_{name}" pos="{_f(pos[0] + 0.017 * math.cos(ct), pos[1], pos[2] + 0.017 * math.sin(ct))}" '
            f'xyaxes="{axes}" fovy="{_f(p.camera_fovy_deg)}"/>')


def camera_positions(p: SimParams, g: Geometry):
    """(parent, [(name, pos)]) for the chosen camera mount. parent is "module" or "base"."""
    hb = p.camera_baseline / 2
    if p.camera_mount == "rim":
        return "module", [(n, (0.142, y, g.rim_z + 0.04)) for n, y in (("left", hb), ("right", -hb))]
    if p.camera_mount == "plate_front":
        return "base", [(n, (p.plate_radius - 0.005, y, g.plate_top + 0.02)) for n, y in (("left", hb), ("right", -hb))]
    if p.camera_mount == "side_posts":
        z = g.rim_center_robot(0.0)[1]
        return "base", [(n, (0.03, y, z)) for n, y in (("left", 0.20), ("right", -0.20))]
    raise ValueError(p.camera_mount)


def _catch_module(p: SimParams, g: Geometry) -> str:
    lo, hi = p.tilt_limits
    parent, cams = camera_positions(p, g)
    cam_xml = "".join(_camera(n, pos, p) for n, pos in cams) if parent == "module" else ""
    px, pz = g.pivot_r
    yoke_y = 0.105
    yokes = "".join(
        f'<geom class="decor" type="box" size="0.012 0.005 {_f((pz - g.plate_top) / 2 + 0.01)}" '
        f'pos="{_f(px, s * yoke_y, (pz + g.plate_top) / 2 + 0.01)}" material="petg_dark" mass="{p.m_tilt_frame / 2}"/>'
        for s in (-1, 1))
    servo = (f'<geom class="decor" type="box" size="0.02 0.01 0.02" pos="{_f(px, -yoke_y - 0.016, pz)}" material="servo" '
             f'mass="{p.m_tilt_servo}"/>')
    # two stub shafts from the tube's side walls out to the yokes (the pocket stays clear)
    y_in = g.w + g.t + 0.002
    stubs = "".join(f'<geom class="decor" type="cylinder" size="0.005" fromto="{_f(0, s * y_in, 0, 0, s * (yoke_y + 0.006), 0)}" '
                    f'material="steel" mass="0.01"/>' for s in (-1, 1))
    return f"""
      {yokes}{servo}
      <body name="tilt" pos="{_f(px, 0, pz)}">
        <joint name="tilt" type="hinge" axis="0 1 0" range="{_f(lo - 0.02, hi + 0.02)}" damping="0.05" armature="0.002"/>
        {stubs}
        <body name="catch_module" pos="{_f(-g.pivot_m[0], 0, -g.pivot_m[1])}">
          {_funnel(p, g)}
          {_hopper(p, g)}
          {_ball_path_kicker(p, g) if p.feed == "kicker" else _ball_path(p, g)}
          {cam_xml}
          <site name="funnel_center" pos="{_f(0, 0, g.rim_z)}" group="5"/>
        </body>
      </body>"""


def _clip(poly, a, b):
    """Sutherland-Hodgman: keep the part of a convex polygon with a . p <= b."""
    out = []
    for k in range(len(poly)):
        p0, p1 = poly[k], poly[(k + 1) % len(poly)]
        d0, d1 = a[0] * p0[0] + a[1] * p0[1] - b, a[0] * p1[0] + a[1] * p1[1] - b
        if d0 <= 0:
            out.append(p0)
        if d0 * d1 < 0:
            f = d0 / (d0 - d1)
            out.append((p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1])))
    return out


def deck_pieces(p: SimParams) -> list:
    """Convex outlines (x, y) of the deck. "high": one disc. "axle": the disc minus a slot along each
    wheel's axle for the wheel and motor, built from a center piece plus one lobe between each pair of
    neighboring wheels (they overlap a little)."""
    circle = [(p.deck_radius * math.cos(a), p.deck_radius * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 64, endpoint=False)]
    if p.deck_mode != "axle":
        return [circle]
    th = sorted(math.radians(a) for a in p.wheel_angles_deg)
    h = p.deck_slot_half_width
    center = circle
    for a in th:
        center = _clip(center, (math.cos(a), math.sin(a)), p.deck_cut_radius)
    pieces = [center]
    for k, a in enumerate(th):
        b = th[(k + 1) % len(th)]
        lobe = _clip(circle, (math.sin(a), -math.cos(a)), -h)     # left of wheel a's slot: p . t_a >= h
        lobe = _clip(lobe, (-math.sin(b), math.cos(b)), -h)       # right of wheel b's slot: p . t_b <= -h
        if len(lobe) >= 3:
            pieces.append(lobe)
    return pieces


def _area(poly) -> float:
    return 0.5 * abs(sum(poly[k][0] * poly[(k + 1) % len(poly)][1] - poly[(k + 1) % len(poly)][0] * poly[k][1]
                         for k in range(len(poly))))


def _deck_assets(p: SimParams) -> str:
    ht = p.deck_thickness / 2
    out = []
    for k, poly in enumerate(deck_pieces(p)):
        v = [c for x, y in poly for c in (x, y, -ht)] + [c for x, y in poly for c in (x, y, ht)]
        out.append(f'<mesh name="deck{k}" vertex="{_f(*v)}"/>')
    return "".join(out)


def _deck(p: SimParams, g: Geometry) -> str:
    pieces = deck_pieces(p)
    areas = [_area(q) for q in pieces]
    zc = g.deck_z + p.deck_thickness / 2
    return "".join(f'<geom class="structure" type="mesh" mesh="deck{k}" pos="0 0 {_f(zc)}" material="petg_dark" '
                   f'mass="{p.m_deck * a / sum(areas):.4g}"/>' for k, a in enumerate(areas))


def _robot(p: SimParams, g: Geometry) -> str:
    dz = g.deck_z
    top = dz + 0.006
    axle = p.deck_mode == "axle"
    standoffs = []
    # payload-plate standoffs between the wheels (the X layout's diagonals hold wheels and motors)
    st_angles = (0, 90, 180, 270) if axle and p.n_wheels == 4 else (0, 120, 240)
    for a in st_angles:
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        standoffs.append(f'<geom class="decor" type="cylinder" size="0.004" material="aluminum" mass="0.02" '
                         f'fromto="{_f(0.1 * c, 0.1 * s, top, 0.1 * c, 0.1 * s, g.plate_top - 0.006)}"/>')
    bumper = []
    nb, rbump = 36, p.deck_radius + 0.008
    zbump = dz + 0.003 if axle else dz - 0.004
    for k in range(nb):
        a0, a1 = 2 * math.pi * k / nb, 2 * math.pi * (k + 1) / nb
        bumper.append(f'<geom class="decor" type="capsule" size="0.014" material="tpu_orange" mass="{p.m_bumper / nb:.4g}" '
                      f'fromto="{_f(rbump * math.cos(a0), rbump * math.sin(a0), zbump, rbump * math.cos(a1), rbump * math.sin(a1), zbump)}"/>')
    pack = p.pack
    if axle:
        # 7 x 21700 in one flat row under the deck (~150 x 72 x 23 mm with wrap), between the motors, on a
        # 1 mm strap: its underside matches the motors' (~21 mm ground clearance)
        battery = (f'<geom class="decor" type="box" size="0.075 0.036 0.0115" pos="0 0 {_f(dz - 0.0115)}" material="battery" mass="{pack.mass}"/>'
                   f'<geom class="decor" type="box" size="0.077 0.012 0.0005" pos="0 0 {_f(dz - 0.0235)}" material="pla_black" mass="{p.m_battery_tray}"/>')
        jet, pcb, estop = (0.0, 0.0), (0.0, 0.125), (-0.155, 0.0)
        lidar_base = max(top, 2 * g.R + 0.004 - 0.026)          # scan plane (~26 mm up) above the wheel tops
    else:
        battery = (f'<geom class="decor" type="box" size="0.045 0.038 0.036" pos="0 0 {_f(dz - 0.04)}" material="battery" mass="{pack.mass}"/>'
                   f'<geom class="decor" type="box" size="0.046 0.039 0.002" pos="0 0 {_f(dz - 0.003)}" material="pla_black" mass="{p.m_battery_tray}"/>')
        jet, pcb, estop = (-0.07, 0.0), (0.03, 0.1), (-0.13, -0.08)
        lidar_base = top
    deck_top_decor = "" if axle else \
        f'<geom class="decor" type="cylinder" size="{_f(p.deck_radius - 0.03, 0.0035)}" pos="0 0 {_f(dz + 0.0032)}" material="petg_grey"/>'
    riser = (f'<geom class="decor" type="cylinder" size="0.024 {_f((lidar_base - top) / 2)}" pos="{_f(p.deck_radius - 0.045, 0, (lidar_base + top) / 2)}" '
             f'material="pla_black"/>') if lidar_base > top + 0.002 else ""
    parent, cams = camera_positions(p, g)
    cam_xml = ""
    if parent == "base":
        cam_xml = "".join(_camera(n, pos, p) for n, pos in cams)
        if p.camera_mount == "side_posts":     # thin carbon posts from the deck edge
            cam_xml += "".join(f'<geom class="decor" type="cylinder" size="0.005" material="petg_dark" mass="0.025" '
                               f'fromto="{_f(pos[0], pos[1], top, pos[0], pos[1], pos[2] - 0.014)}"/>' for _, pos in cams)
    hx, hy = p.home_xy
    q = quat_axis((0, 0, 1), p.home_yaw)
    rl = p.deck_radius - 0.045                 # lidar near the front edge
    lb = lidar_base
    return f"""
    <body name="base" pos="{_f(hx, hy, 0)}" quat="{_f(*q)}">
      <freejoint name="base"/>
      <site name="imu" pos="0 0 {_f(top + 0.01)}" group="5"/>
      {_deck(p, g)}
      {deck_top_decor}
      {battery}
      <geom class="decor" type="box" size="0.05 0.05 0.016" pos="{_f(*jet, top + 0.016)}" material="jetson" mass="{p.m_jetson}"/>
      <geom class="decor" type="box" size="0.045 0.045 0.007" pos="{_f(*jet, top + 0.039)}" material="heatsink"/>
      <geom class="decor" type="cylinder" size="0.014 0.006" pos="{_f(*jet, top + 0.052)}" material="fan"/>
      <geom class="decor" type="box" size="0.03 0.03 0.006" pos="{_f(*pcb, top + 0.006)}" material="pcb" mass="{p.m_electronics}"/>
      {riser}
      <geom class="decor" type="cylinder" size="0.028 0.0165" pos="{_f(rl, 0, lb + 0.0165)}" material="lidar" mass="{p.m_lidar}"/>
      <geom class="decor" type="cylinder" size="0.022 0.006" pos="{_f(rl, 0, lb + 0.039)}" material="lidar_top"/>
      <geom class="decor" type="cylinder" size="0.02 0.01" pos="{_f(*estop, top + 0.01)}" material="estop_base"/>
      <geom class="decor" type="cylinder" size="0.017 0.009" pos="{_f(*estop, top + 0.029)}" material="estop"/>
      <geom class="decor" type="box" size="0.01 0.01 0.01" pos="0 0 {_f(top)}" material="pla_black" mass="{p.m_misc}"/>
      {''.join(standoffs)}
      {''.join(bumper)}
      <geom class="structure" type="cylinder" size="{_f(p.plate_radius, 0.003)}" pos="0 0 {_f(g.plate_top - 0.003)}" material="petg_grey" mass="{p.m_plate}"/>
      {cam_xml}
      {_wheels(p, g)}
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
    wn = math.sqrt(stiffness / mass)
    lr = math.log(restitution)
    zeta = -lr / math.sqrt(math.pi ** 2 + lr ** 2)
    return f"{1 / (wn * zeta):.4g} {zeta:.3g}"


def ball_names(p: SimParams) -> list:
    """"ball" (the tracked one) plus the hopper's extra balls "ball_1".. "ball_n"."""
    return ["ball"] + [f"ball_{k}" for k in range(1, p.n_balls + 1)]


def ball_parking(p: SimParams) -> list:
    """Starting spots: the main ball by the thrower's feet, the extras in a row along the far netting."""
    rb = p.ball_spec.radius
    ax, ay = p.area_size
    spots = [(p.thrower_xy[0] + 0.3, p.thrower_xy[1] + 0.4, rb + 0.002)]
    spots += [(-ax / 2 + 0.3 + 0.1 * k, ay / 2 - 0.15, rb + 0.002) for k in range(p.n_balls)]
    return spots


def _ball_body(name: str, pos, p: SimParams, solref: str) -> str:
    ball = p.ball_spec
    return f"""
    <body name="{name}" pos="{_f(*pos)}">
      <freejoint name="{name}"/>
      <geom name="{name}" type="sphere" size="{ball.radius}" mass="{ball.mass}" material="ball" priority="1"
            contype="1" conaffinity="1" condim="6" friction="{ball.mu} 0.005 0.002"
            solref="{solref}" solimp="0.95 0.99 0.001"
            fluidshape="ellipsoid" fluidcoef="{_f(ball.drag_cd / 2, 0.25, 1.5, 1.0, 1.0)}"/>
    </body>"""


def build_xml(p: SimParams = SimParams()) -> str:
    tex = ensure_textures()
    g = geometry(p)
    nw = p.n_wheels
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
    side_cam = (-0.45, -1.92, 1.3)
    rb = g.rb
    ball = p.ball_spec
    ball_solref = _solref(ball.stiffness, ball.mass, ball.restitution)
    stiff = 'solref="0.002 1" solimp="0.95 0.99 0.001"'
    pairs = []
    for bname in ball_names(p):      # every ball, including the hopper's extra ones, gets the same contacts
        pairs += [f'<pair geom1="{bname}" geom2="flywheel_{s}" condim="3" friction="{p.flywheel_mu} {p.flywheel_mu} 0.005 0.0001 0.0001" '
                  f'{stiff}/>' for s in ("L", "R")]
        pairs.append(f'<pair geom1="{bname}" geom2="gate" condim="3" friction="0.8 0.8 0.005 0.0001 0.0001" '
                     f'solref="{_solref(ball.stiffness, ball.mass, p.gate_pad_restitution)}" solimp="0.95 0.99 0.001"/>')
        if p.feed == "kicker":           # the caught ball lands on the padded floor
            pairs.append(f'<pair geom1="{bname}" geom2="pocket_floor" condim="3" friction="0.8 0.8 0.005 0.0001 0.0001" '
                         f'solref="{_solref(ball.stiffness, ball.mass, p.gate_pad_restitution)}" solimp="0.95 0.99 0.001"/>')
        if p.throat_flaps and p.feed == "j_bend":      # the flaps exist only in the J-bend pocket
            pairs += [f'<pair geom1="{bname}" geom2="flap_{s}" condim="3" friction="0.6 0.6 0.005 0.0001 0.0001" '
                      f'solref="0.003 1" solimp="0.95 0.99 0.001"/>' for s in ("L", "R")]
        fm, fr = p.funnel_mu, p.funnel_roll
        pairs += [f'<pair geom1="{bname}" geom2="funnel{k}" condim="6" friction="{fm} {fm} 0.005 {fr} {fr}" solref="0.004 1.0"/>'
                  for k in range(p.funnel_panels)]
    wheel_contacts = ([f"wheel{i}_contact" for i in range(1, nw + 1)] if p.wheel_model == "sphere"
                      else [f"roller{i}_{j}" for i in range(1, nw + 1) for j in range(2 * p.omni.rollers_per_row)])
    mu = p.mu
    pairs += [f'<pair geom1="floor" geom2="{gname}" condim="3" friction="{mu} {mu} 0.005 0.0001 0.0001" '
              f'solref="0.004 1" solreffriction="0.002 1" solimp="0.95 0.99 0.001"/>' for gname in wheel_contacts]
    actuators = []
    for i in range(1, nw + 1):
        actuators.append(f'<general name="wheel{i}" joint="drive{i}" gainprm="{p.drive_kp}" biastype="affine" '
                         f'biasprm="0 0 {-p.drive_kp}" forcelimited="true" forcerange="-5 5"/>')
    if p.wheel_model == "sphere":
        for i in range(1, nw + 1):
            actuators.append(f'<general name="wheel{i}_coupling" site="wheel{i}_contact" refsite="hub{i}" gear="0 0 0 1 0 0" '
                             f'gainprm="0" biastype="affine" biasprm="0 0 -50"/>')
    actuators += ['<motor name="flywheel_L" joint="flywheel_L" gear="1"/>',
                  '<motor name="flywheel_R" joint="flywheel_R" gear="1"/>',
                  (f'<position name="gate" joint="gate" kp="2000" kv="40" forcelimited="true" forcerange="{_f(-p.kick_force, p.kick_force)}"/>'
                   if p.feed == "kicker" else
                   f'<position name="gate" joint="gate" kp="25" kv="0.3" forcelimited="true" forcerange="{_f(-p.gate_torque, p.gate_torque)}"/>'),
                  f'<position name="tilt" joint="tilt" kp="60" kv="1.0" forcelimited="true" forcerange="{_f(-p.tilt_torque, p.tilt_torque)}"/>']
    if p.hopper == "sweeper":
        actuators.append(f'<velocity name="sweeper" joint="sweeper" kv="0.2" forcelimited="true" '
                         f'forcerange="{_f(-p.sweeper_torque, p.sweeper_torque)}"/>')
    sensors = [
        '<framepos name="base_pos" objtype="xbody" objname="base"/>',
        '<framequat name="base_quat" objtype="xbody" objname="base"/>',
        '<framelinvel name="base_linvel" objtype="xbody" objname="base"/>',
        '<frameangvel name="base_angvel" objtype="xbody" objname="base"/>',
        '<subtreelinvel name="base_cg_vel" body="base"/>',
        '<gyro name="imu_gyro" site="imu"/>',
        '<accelerometer name="imu_accel" site="imu"/>',
        '<rangefinder name="catch_beam" site="catch_beam" cutoff="0.5"/>',
        '<framepos name="ball_pos" objtype="xbody" objname="ball"/>',
        '<framelinvel name="ball_vel" objtype="xbody" objname="ball"/>',
        '<frameangvel name="ball_angvel" objtype="xbody" objname="ball"/>',
        '<jointvel name="flywheel_L_speed" joint="flywheel_L"/>',
        '<jointvel name="flywheel_R_speed" joint="flywheel_R"/>',
        '<jointpos name="gate_pos" joint="gate"/>',
        '<jointpos name="tilt_pos" joint="tilt"/>',
        '<actuatorfrc name="tilt_torque" actuator="tilt"/>',
    ]
    for i in range(1, nw + 1):
        sensors.append(f'<jointvel name="wheel{i}_speed" joint="drive{i}"/>')
        sensors.append(f'<jointpos name="susp{i}" joint="susp{i}"/>')
    om = p.omni
    rr = om.roller_diameter / 2
    roller_mesh = "".join(barrel(f"roller{i}", g.R, rr, g.R * math.sin(math.pi * 0.62 / om.rollers_per_row))
                          for i in range(1, nw + 1))
    alpha = math.radians(p.funnel_wall_angle_deg)
    funnel_mesh = cone_shell("funnel_net", p.funnel_throat_radius + 0.002, g.throat_z + 0.002 / math.tan(alpha),
                             p.funnel_rim_radius + 0.002, g.rim_z, n=60, wraps=10)

    return f"""<mujoco model="play_catch_v3">
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
    {_deck_assets(p)}
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
    <camera name="robot_close" mode="targetbody" target="base" pos="2.4 -1.3 1.0" fovy="40"/>
    <camera name="overhead" pos="-0.4 0 6.2" xyaxes="1 0 0 0 1 0" fovy="50"/>
    {_thrower(p)}
    <body name="landing_marker" mocap="true" pos="{_f(*p.home_xy, -0.05)}">
      <geom class="decor" type="cylinder" size="{_f(p.funnel_rim_radius, 0.0008)}" material="marker"/>
    </body>
    {''.join(_ball_body(bname, pos, p, ball_solref) for bname, pos in zip(ball_names(p), ball_parking(p)))}
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


def mass_properties(model: mujoco.MjModel, p: SimParams, tilt: float = 0.0):
    """(mass, CG xyz in the robot frame) with the robot upright at the origin and the module at tilt."""
    data = mujoco.MjData(model)
    adr = model.joint("base").qposadr[0]
    data.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
    data.qpos[model.joint("tilt").qposadr[0]] = tilt
    mujoco.mj_forward(model, data)
    b = model.body("base").id
    return float(model.body_subtreemass[b]), data.subtree_com[b] - data.xpos[b]


def describe(model: mujoco.MjModel, p: SimParams) -> str:
    from traction import Base
    g = geometry(p)
    mass, cg = mass_properties(model, p)
    b = Base(p.wheel_angles_deg, p.wheel_base_radius, cg[2], cg[0], cg[1], p.mu)
    tip = min(b.tip_accel((math.cos(a), math.sin(a))) for a in np.radians(np.arange(0, 360, 5)))
    rim = g.rim_center_robot(0.0)[1]
    return (f"robot {mass:.2f} kg, CG {cg[2] * 100:.1f} cm high; {p.n_wheels} wheels at r {p.wheel_base_radius * 1000:.0f} mm; "
            f"rim {rim * 1000:.0f} mm, pivot {g.pivot_r[1] * 1000:.0f} mm; worst-direction tip-over {tip:.1f} m/s^2")


if __name__ == "__main__":
    params = SimParams()
    out = Path(__file__).with_name("scene.xml")
    out.write_text(build_xml(params))
    print(f"wrote {out}")
    print(describe(build_model(params), params))
