"""An SO-101 arm on the v11 robot in place of the catch module (design.md 11; the payload interface: §15): what it
reaches, and whether the robot tips. Proposed, exploration: no CAD for the mount.

The robot: robot_export.v11_params(payload="arm") on the CAD's payload plate (plate_gap 0.096: plate top 168.5 mm,
cad/v7/dims.py), the arm on a printed adapter at MOUNTS ("centre" on the plate's centre, "front" 60 mm ahead),
pan 0 = robot +x. Two mass models:
  sim  the sim's parts: 5.17 kg without the catch module, + the arm 0.76 kg (arm_so101.py: 0.63 kg of links, adapter,
       bus board, cables, wrist camera)
  cad  + one point mass so that the robot without its catch module weighs what the CAD estimates (cad/v7/out/BOM.md,
       Oct 22: 8.37 kg, CG 127 mm, of which the module 1.27 kg; the module's CG taken from the sim's)
The X4's wheels stand at 45/135/225/315 deg, 150 mm out: the support polygon is a square whose edges are 106 mm from
the centre straight ahead, behind and to each side, and 150 mm toward each wheel.

  R  reach: the tool point (between the jaws) over the arm's joint space (pan 0 = ahead, pan 90 = to the left), outside
     the robot's outline (the bumper r 217 mm up to the plate, the stereo cameras at the plate's front edge, the display
     at its rear) and above the floor: how far it reaches on the floor (any approach / top-down within 30 deg), at a
     coffee table (0.45 m), a dining table (0.75 m), a counter (0.90 m), and how high; the lidar rays the arm cuts
     while it reaches for the floor ahead.
  S  static: along each reach line (the floor, a 0.45 m coffee table, the arm level at shoulder height) the payload the servos hold
     (stall 2.94 N m; rated 0.98 N m, the thermal limit) and the payload that tips the robot (straight ahead and over a
     wheel), the push and the pull at the tool that tip it.
  D  in motion (MuJoCo, explicit rollers, the firmware's limits): drive at 1 / 2 m/s, then the firmware's ramped stop
     (velocity_step to zero: the direction table, 4.26 m/s^2 straight ahead or back, 2.84 at 60 deg), for poses x
     payloads x the motion relative to the arm; the arm swung at full servo speed while parked.
  T  a 12 mm square rug edge and an 18 mm threshold at 1 m/s, the arm out.
  F  the worst stops through the real firmware (SilRobot + SimLink): Python controller vs C++.
  B  what fixes a tip: a lower stop limit with the arm out, or ballast.
  K  the arm kit as drawn (x 0.14 on a 50 mm riser, design.md 12.3): the stops again, stowed, upright, reaching.

  python arm_study.py              # all (D, T, F, B in parallel, ~5 min on 12 cores)
  python arm_study.py R S          # parts;  python arm_study.py report
Writes results/arm_study.json, results/arm_reach.png.
"""

from __future__ import annotations

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache

import mujoco
import numpy as np

from common import RESULTS, save, tilt_deg  # noqa: E402
import arm_so101 as A  # noqa: E402
import floor  # noqa: E402
from robot import Robot  # noqa: E402
from robot_export import v11_params  # noqa: E402
from robot_model import build_model, camera_positions, geometry, mass_properties  # noqa: E402

G = 9.81
PLATE_GAP = 0.096                  # the CAD's plate (cad/v7/dims.py PLATE_GAP)
CAD_TOTAL, CAD_CG, CAD_MODULE = 8.372, np.array([0.0073, -0.0011, 0.1274]), 1.269    # cad/v7/out/BOM.md "Mass"
BUMPER_R = 0.217                   # CAD: deck r 205 + 12 mm TPU bumper
CAM_RAISE = 0.0125                 # CAD: cameras 12.5 mm above the sim's spot (dims.py)
DISPLAY_BOX = ((-0.163, -0.068), (-0.098, 0.098), 0.302)     # x, y ranges, top z (cad/v7/display.py housing)
MOUNTS = {"centre": (0.0, 0.0, 0.0), "front": (0.06, 0.0, 0.0)}
RISERS = {"kit": 0.05}             # the arm kit as drawn (cad/v7 arm_kit.py, design.md 12.3): x 0.14 on a 50 mm riser;
                                   # in the stop and fix checks only (python arm_study.py K)
PAYLOADS = (0.0, 0.25, 0.5)
HEIGHTS = {"floor": (0.02, 0.05), "coffee table 0.45": (0.46, 0.49), "dining table 0.75": (0.76, 0.79),
           "counter 0.90": (0.91, 0.94)}
TILT_OVER = 30.0                   # deg: past this the robot is going over (a stop never pitches it more than ~3)
WHEEL_LIFT = 1.0                   # N: a wheel under this has left the floor


BALLAST_Z = 0.03                   # m: a steel plate under the battery, between the motors (est)


def params(base: str = "sim", mount: str = "centre", payload: float = 0.0, ballast: float = 0.0, **kw):
    """base "catch": today's robot with its catch module, the baseline for the stops."""
    world = dict(plate_gap=PLATE_GAP, area_size=(16.0, 16.0), home_xy=(0.0, 0.0), thrower_xy=(7.5, 0.0))
    if base == "catch":
        return v11_params(**world, **kw)
    mt = (0.14, 0.0, 0.0) if mount == "kit" else MOUNTS[mount]
    p = v11_params(payload="arm", arm_mount=mt, arm_riser=RISERS.get(mount, 0.0), arm_payload_kg=payload, **world, **kw)
    extra = ((cad_trim(mount),) if base == "cad" else ()) + (((ballast, 0.0, 0.0, BALLAST_Z),) if ballast else ())
    return p.with_(extra_masses=extra) if extra else p


@lru_cache(maxsize=None)
def cad_trim(mount: str = "centre"):
    """(kg, x, y, z): the point mass that makes the sim's robot without its catch module match the CAD's estimate."""
    pc = v11_params(plate_gap=PLATE_GAP)
    mc, cc = mass_properties(build_model(pc), pc)
    pa = v11_params(payload="arm", plate_gap=PLATE_GAP, arm_mount=(0.14, 0.0, 0.0) if mount == "kit" else MOUNTS[mount],
                    arm_riser=RISERS.get(mount, 0.0))
    ma_, ca = mass_properties(build_model(pa), pa)
    fk = ArmFK(pa)
    m_arm, c_arm = fk.arm_mass_cg(A.POSES["stow"], 0.0)
    ms, cs = ma_ - m_arm, (ma_ * ca - m_arm * c_arm) / (ma_ - m_arm)        # the base without catch module or arm
    c_mod = (mc * cc - ms * cs) / (mc - ms)                                 # the sim's catch module
    mb = CAD_TOTAL - CAD_MODULE
    cb = (CAD_TOTAL * CAD_CG - CAD_MODULE * c_mod) / mb
    mt = mb - ms
    ct = (mb * cb - ms * cs) / mt
    return (round(float(mt), 4), *(round(float(v), 4) for v in ct))


# ---------------------------------------------------------------------------
# kinematics, the robot's outline, mass

class ArmFK:
    """The arm on the robot, robot frame (origin on the floor under the centre), base held level."""

    def __init__(self, p):
        self.p = p
        self.m = build_model(p)
        self.d = mujoco.MjData(self.m)
        m = self.m
        adr = m.joint("base").qposadr[0]
        self.d.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
        self.qadr = np.array([m.joint(A.name(j)).qposadr[0] for j in A.JOINTS])
        self.vadr = np.array([m.joint(A.name(j)).dofadr[0] for j in A.JOINTS])
        self.tool = m.site("arm_tool").id
        caps = [g for g in range(m.ngeom) if m.geom(g).name.startswith("arm_") and "_link" in m.geom(g).name]
        self.caps = [g for g in caps if m.geom(g).name != "arm_base_link0"]       # the base sits on the plate
        self.cap_r = np.array([m.geom_size[g][0] for g in self.caps])
        self.cap_h = np.array([m.geom_size[g][1] for g in self.caps])
        root = m.body("arm_mount").id
        self.arm_bodies = [b for b in range(m.nbody) if self._under(b, root)]
        self.pay = m.body("arm_payload").id
        g = geometry(p)
        self.plate_top = float(g.plate_top)
        cams = [np.array(pos) for _, pos in camera_positions(p, g)[1]]
        cx, cz = cams[0][0], cams[0][2] + CAM_RAISE
        cy = max(abs(c[1]) for c in cams)
        if p.arm_mount[0] < 0.03:      # the stereo bar across the plate's front edge, as in the CAD
            self.cam_boxes = [((cx - 0.025, cx + 0.03), (-cy - 0.03, cy + 0.03), cz + 0.025)]
        else:                          # an arm on the front needs the bar split: two camera pods, the arm between
            self.cam_boxes = [((cx - 0.025, cx + 0.03), (s * cy - 0.03, s * cy + 0.03), cz + 0.025) for s in (-1, 1)]
        self.lidar = np.array([p.deck_radius - 0.045, 0.0, 0.150])          # the CAD's scan plane

    def _under(self, b, root):
        while b > 0:
            if b == root:
                return True
            b = self.m.body_parentid[b]
        return False

    def set(self, q):
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)

    def tool_pos(self) -> np.ndarray:
        return self.d.site_xpos[self.tool].copy()

    def approach(self) -> np.ndarray:
        """The gripper's pointing direction (the tool frame's x axis)."""
        return self.d.site_xmat[self.tool].reshape(3, 3)[:, 0].copy()

    def link_points(self, n: int = 5):
        """Points along each link capsule's axis (n per capsule) and their radii."""
        pts, rad = [], []
        s = np.linspace(-1.0, 1.0, n)
        for g, r, h in zip(self.caps, self.cap_r, self.cap_h):
            c = self.d.geom_xpos[g]
            ax = self.d.geom_xmat[g].reshape(3, 3)[:, 2]
            pts.append(c + np.outer(s * h, ax))
            rad.append(np.full(n, r))
        return np.vstack(pts), np.concatenate(rad)

    def clear(self) -> bool:
        """The arm keeps out of the robot (bumper ring up to the plate, cameras, display) and the floor."""
        P, r = self.link_points()
        x, y, z = P.T
        if np.any(z - r < -0.002) or self.tool_pos()[2] < 0.012:
            return False
        if np.any((np.hypot(x, y) < BUMPER_R + r) & (z - r < self.plate_top + 0.004)):
            return False
        for (x0, x1), (y0, y1), zt in (*self.cam_boxes, DISPLAY_BOX):
            if np.any((x > x0 - r) & (x < x1 + r) & (y > y0 - r) & (y < y1 + r) & (z - r < zt)):
                return False
        return True

    def arm_mass_cg(self, q, payload: float):
        """Mass and CG of the arm (with its added parts and the payload at the tool), robot frame."""
        self.set(q)
        m = self.m
        ms = np.array([m.body_mass[b] for b in self.arm_bodies])
        xs = np.array([self.d.xipos[b] for b in self.arm_bodies])
        k = self.arm_bodies.index(self.pay)
        ms[k] = payload
        xs[k] = self.tool_pos()
        M = float(ms.sum())
        return M, (ms[:, None] * xs).sum(0) / max(M, 1e-9)

    def hold_torques(self, q):
        """(gravity torque on each arm joint with no payload, d torque / d payload kg) at q, standing still."""
        self.set(q)
        m, d = self.m, self.d
        d.qvel[:] = 0
        mujoco.mj_comPos(m, d)
        out = np.zeros(m.nv)
        mujoco.mj_rne(m, d, 0, out)
        jp = np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jp, None, self.tool)
        return out[self.vadr].copy(), G * jp[2, self.vadr]

    def servo_payload(self, q, tau_lim: float) -> float:
        """The heaviest payload at the tool every joint holds within tau_lim (0: not even the arm itself)."""
        tb, k = self.hold_torques(q)
        lim = math.inf
        for t0, kk in zip(tb[:5], k[:5]):            # the gripper's jaw carries none of it
            if abs(t0) > tau_lim:
                return 0.0
            if kk > 1e-9:
                lim = min(lim, (tau_lim - t0) / kk)
            elif kk < -1e-9:
                lim = min(lim, (-tau_lim - t0) / kk)
        return max(0.0, lim)


@lru_cache(maxsize=None)
def base_mass_cg(base: str, mount: str):
    """The robot without the arm: (kg, CG)."""
    p = params(base, mount)
    m, c = mass_properties(build_model(p), p)
    fk = ArmFK(p)
    ma, ca = fk.arm_mass_cg(A.POSES["stow"], 0.0)
    return m - ma, (m * c - ma * ca) / (m - ma)


def support_edges(p):
    """(normals n, offsets e) of the support polygon's edges (n . x <= e inside): the wheels' contact points."""
    th = np.radians(p.wheel_angles_deg)
    pts = p.wheel_base_radius * np.column_stack([np.cos(th), np.sin(th)])
    order = np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))
    pts = pts[order]
    ns, es = [], []
    for k in range(len(pts)):
        a, b = pts[k], pts[(k + 1) % len(pts)]
        n = np.array([b[1] - a[1], a[0] - b[0]])
        n /= np.linalg.norm(n)
        ns.append(n)
        es.append(float(n @ a))
    return np.array(ns), np.array(es)


def tip_numbers(M, c, tool, p):
    """Static stability of a robot of mass M (CG c) with the tool at `tool`: the CG's margin inside the support
    polygon, the payload at the tool that tips it, and the horizontal push (outward, along the arm), pull (toward the
    robot) and downward press at the tool that tip it."""
    n, e = support_edges(p)
    W = M * G
    margin = float(np.min(e - n @ c[:2]))
    out = dict(margin=margin)
    over = n @ tool[:2] - e                       # > 0: the tool is outside that edge
    restore = W * (e - n @ c[:2])                 # each edge's restoring moment (N m)
    m_tip = [restore[k] / (G * over[k]) for k in range(len(e)) if over[k] > 1e-6]
    out["payload_tip"] = min(m_tip) if m_tip else math.inf
    u = tool[:2] / max(np.linalg.norm(tool[:2]), 1e-9)
    for key, f in (("push", np.array([*u, 0.0])), ("pull", np.array([*-u, 0.0])), ("press", np.array([0, 0, -1.0]))):
        lev = (n @ f[:2]) * tool[2] - f[2] * over         # tipping moment of a unit force about each edge
        fs = [restore[k] / lev[k] for k in range(len(e)) if lev[k] > 1e-9]
        out[key] = min(fs) if fs else math.inf
    return out


# ---------------------------------------------------------------------------
# R: reach

GRID = (73, 69, 35)          # lift, elbow, wrist flex samples


def sweep(p, pan: float):
    """Every grid pose at this pan that keeps clear: rows (lift, elbow, wrist, tool x, y, z, approach z)."""
    fk = ArmFK(p)
    lo, hi = A.joint_ranges().T
    rows = []
    for a in np.linspace(lo[1], hi[1], GRID[0]):
        for b in np.linspace(lo[2], hi[2], GRID[1]):
            for c in np.linspace(lo[3], hi[3], GRID[2]):
                q = (pan, a, b, c, 0.0, 0.0)
                fk.set(q)
                if fk.clear():
                    t = fk.tool_pos()
                    rows.append((a, b, c, *t, fk.approach()[2]))
    return np.array(rows)


def reach_job(args):
    mount, pan_deg = args
    p = params("sim", mount)
    rows = sweep(p, A.pan_for(pan_deg))
    u = np.array([math.cos(math.radians(pan_deg)), math.sin(math.radians(pan_deg))])
    s = rows[:, 3:5] @ u                          # distance from the robot's centre along the arm's direction
    z = rows[:, 5]
    out = dict(mount=mount, pan=pan_deg, n=len(rows), top=float(z.max()), s_max=float(s.max()))
    for name, (z0, z1) in HEIGHTS.items():
        k = (z >= z0) & (z <= z1)
        out[name] = (float(s[k].min()), float(s[k].max())) if np.any(k) else None
        if name == "floor":
            kd = k & (rows[:, 6] < -math.cos(math.radians(30)))
            out["floor top-down"] = (float(s[kd].min()), float(s[kd].max())) if np.any(kd) else None
    out["grid"] = rows[::7, :].round(4).tolist()  # for the plot
    out["_rows"] = rows
    return out


def poses_for(mount: str, rows=None):
    """The poses D and T run (pan 0, arm ahead), picked from the reach grid: stow (the most compact), carry (tool 0.15
    m above the plate and close), level (the farthest reach at shoulder height: the largest moment), floor (the
    farthest floor reach, any approach), pick (the farthest top-down floor pick), up (the highest)."""
    p = params("sim", mount)
    fk = ArmFK(p)
    if rows is None:
        rows = sweep(p, 0.0)
    q = lambda r: (0.0, float(r[0]), float(r[1]), float(r[2]), 0.0, 0.0)    # noqa: E731
    x, z, az = rows[:, 3], rows[:, 5], rows[:, 6]
    pt = fk.plate_top
    out = {}
    ext = []
    for r in rows[::3]:
        fk.set(q(r))
        P, _ = fk.link_points(3)
        ext.append(max(np.max(np.hypot(P[:, 0] - MOUNTS[mount][0], P[:, 1])), 0.0) + 0.3 * (P[:, 2].max() - pt))
    out["stow"] = q(rows[::3][int(np.argmin(ext))])
    k = np.where(np.abs(z - (pt + 0.15)) < 0.01)[0]
    out["carry"] = q(rows[k[np.argmin(np.abs(x[k] - (MOUNTS[mount][0] + 0.12)))]])
    sh = pt + 0.006 + 0.117                        # shoulder-lift axis height
    k = np.where(np.abs(z - sh) < 0.01)[0]
    out["level"] = q(rows[k[np.argmax(x[k])]])
    k = np.where((z > 0.02) & (z < 0.05))[0]
    out["floor"] = q(rows[k[np.argmax(x[k])]])
    k = np.where((z > 0.02) & (z < 0.05) & (az < -math.cos(math.radians(30))))[0]
    out["pick"] = q(rows[k[np.argmax(x[k])]]) if len(k) else out["floor"]
    out["up"] = q(rows[int(np.argmax(z))])
    return out


def lidar_shadow(mount: str, q) -> float:
    """Degrees of the lidar's scan (its plane 150 mm up) the arm blocks at pose q."""
    p = params("sim", mount)
    fk = ArmFK(p)
    fk.set(q)
    P, r = fk.link_points(25)
    k = np.abs(P[:, 2] - fk.lidar[2]) < r
    if not np.any(k):
        return 0.0
    rel = P[k, :2] - fk.lidar[:2]
    ang = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))
    half = np.degrees(np.arcsin(np.clip(r[k] / np.linalg.norm(rel, axis=1), 0, 1)))
    cover = np.zeros(3600, dtype=bool)
    for a, h in zip(ang, half):
        i0, i1 = int(math.floor((a - h) * 10)), int(math.ceil((a + h) * 10))
        cover[np.arange(i0, i1 + 1) % 3600] = True
    return float(cover.sum() / 10)


# ---------------------------------------------------------------------------
# S: static

def static_part(reach):
    """Along each reach line: the servo-held payload (stall / rated) and the tipping numbers, per base."""
    out = []
    for mount in MOUNTS:
        p = params("sim", mount)
        fk = ArmFK(p)
        rows = reach[(mount, 0.0)]["_rows"]
        sh = fk.plate_top + 0.006 + 0.117
        lines = {"floor": (0.02, 0.05), "level": (sh - 0.01, sh + 0.01), "coffee table 0.45": (0.46, 0.49)}
        for line, (z0, z1) in lines.items():
            k = np.where((rows[:, 5] >= z0) & (rows[:, 5] <= z1))[0]
            if not len(k):
                continue
            s = rows[k, 3]
            for s0 in np.arange(np.floor(s.min() * 50) / 50, s.max() + 1e-9, 0.02):
                kk = k[np.abs(rows[k, 3] - s0) < 0.01]
                if not len(kk):
                    continue
                best, bq = -1.0, None
                for i in kk[:: max(1, len(kk) // 25)]:
                    qq = (0.0, *rows[i, :3], 0.0, 0.0)
                    w = fk.servo_payload(qq, A.SERVO_STALL)
                    if w > best:
                        best, bq = w, qq
                rated = fk.servo_payload(bq, 0.9807)
                row = dict(mount=mount, line=line, s=round(float(s0), 3), stall_kg=best, rated_kg=rated)
                for base in ("sim", "cad"):
                    mb, cb = base_mass_cg(base, mount)
                    for phi in (0.0, 45.0):              # panned ahead (the edge 106 mm out) and toward a wheel
                        qq = (A.pan_for(phi), *bq[1:])
                        ma, ca = fk.arm_mass_cg(qq, 0.0)
                        M = mb + ma
                        row[f"{base}_{int(phi)}"] = tip_numbers(M, (mb * cb + ma * ca) / M, fk.tool_pos(), p)
                out.append(row)
    return out


# ---------------------------------------------------------------------------
# D / T / F: in motion

def make(p, q, settle: float = 0.6):
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    r = Robot(m, d, p)
    if r.arm is not None:
        r.arm.set_pose(q, snap=True)
    mujoco.mj_forward(m, d)
    for _ in range(int(settle / m.opt.timestep)):
        r.step()
    return m, d, r


class Watch:
    """At rest (when made): the tilt, each wheel's load, the CG's margin inside the support polygon and the tilt that
    would put the CG over the nearest edge (atan(margin / CG height)). Then: the peak tilt, the smallest load of the
    wheels that carried > 3 N at rest and how long one of them was off the floor (a wheel the sprung pods already
    leave unloaded at rest is not counted again), the tool's sag and the servos' peak use of stall torque."""

    def __init__(self, r):
        self.r = r
        d, m = r.d, r.m
        self.tilt0 = self.tilt = tilt_deg(d)
        self.loads0 = r.wheel_loads().copy()
        self.carry = self.loads0 > 3.0
        b = m.body("base").id
        c = d.xmat[b].reshape(3, 3).T @ (d.subtree_com[b] - d.xpos[b])
        n, e = support_edges(r.p)
        self.margin = float(np.min(e - n @ c[:2]))
        self.tip_angle = math.degrees(math.atan2(self.margin, c[2]))
        self.load = math.inf
        self.lift_t = 0.0
        self.sat = 0.0
        self.sag = 0.0
        self.tool0 = self._tool() if r.arm is not None else None

    def _tool(self):
        d = self.r.d
        b = self.r.m.body("base").id
        return d.xmat[b].reshape(3, 3).T @ (d.sensor("arm_tool_pos").data - d.xpos[b])

    def step(self, dt):
        r = self.r
        self.tilt = max(self.tilt, tilt_deg(r.d))
        w = r.wheel_loads()[self.carry]
        self.load = min(self.load, float(w.min()))
        if w.min() < WHEEL_LIFT:
            self.lift_t += dt
        if r.arm is not None:
            f = np.abs(r.d.actuator_force[r.arm.a[:5]])
            self.sat = max(self.sat, float(f.max() / A.SERVO_STALL))
            self.sag = max(self.sag, float(np.linalg.norm(self._tool() - self.tool0)))

    def result(self):
        return dict(tilt0=round(self.tilt0, 2), tilt=round(self.tilt, 2), tip_angle=round(self.tip_angle, 1),
                    margin_mm=round(1000 * self.margin, 1), rest_loads=[round(float(x), 1) for x in self.loads0],
                    min_load=round(self.load, 2), lift_ms=round(1000 * self.lift_t, 1), sat=round(self.sat, 3),
                    sag_mm=round(1000 * self.sag, 1), tipped=self.tilt > TILT_OVER)


def stop_job(args):
    """Drive at v along body direction theta (deg) with the arm at pose q panned to phi, then the firmware's stop."""
    base, mount, pose, q, payload, v, theta, phi, kw = args
    kw = dict(kw)
    fix, ballast = kw.pop("_fix", None), kw.pop("_ballast", 0.0)
    p = params(base, mount, payload, ballast=ballast, **kw)
    q = (A.pan_for(phi), *q[1:]) if q is not None else None
    m, d, r = make(p, q)
    dt = m.opt.timestep
    u = np.array([math.cos(math.radians(theta)), math.sin(math.radians(theta))])
    w = Watch(r)
    t, phase, t_next = 0.0, "go", None       # go -> cruise (0.5 s at speed) -> stop -> settle (1.5 s)
    while t < 8.0 and w.tilt <= TILT_OVER:
        r.drive_velocity(v * u if phase in ("go", "cruise") else np.zeros(2), None, r.pose()[2])
        r.step()
        w.step(dt)
        t += dt
        sp = float(np.linalg.norm(r.ctl.v_cmd))
        if phase == "go" and sp >= v - 1e-6:
            phase, t_next = "cruise", t + 0.5
        elif phase == "cruise" and t >= t_next:
            phase = "stop"
        elif phase == "stop" and sp < 1e-6:
            phase, t_next = "settle", t + 1.5
        elif phase == "settle" and t >= t_next:
            break
    return dict(base=base, mount=mount, pose=pose, payload=payload, v=v, theta=theta, phi=phi,
                fix=fix, **w.result())


def swing_job(args):
    """Parked: the arm swings its pan 90 deg each way at full speed (pose "level"), or lifts from the floor reach to
    straight up and back ("floor" -> "up")."""
    base, mount, kind, qa, qb, payload = args
    p = params(base, mount, payload)
    if kind == "pan":
        qa = (-math.pi / 2, *qa[1:])
        qb = (math.pi / 2, *qa[1:])
    m, d, r = make(p, qa)
    w = Watch(r)
    dt = m.opt.timestep
    for target in (qb, qa, qb):
        r.arm.set_pose(target)
        t0 = d.time
        while d.time - t0 < 1.2:
            r.drive_velocity(np.zeros(2), 0.0, r.pose()[2])
            r.step()
            w.step(dt)
    res = w.result()
    res.pop("sag_mm")
    return dict(base=base, mount=mount, kind=kind, payload=payload, **res)


def edge_job(args):
    """A floor edge at x = 0.8 m, crossed at 1 m/s with the arm at pose q (pan 0) along theta."""
    base, mount, pose, q, payload, feature, theta = args
    feat = {"rug 12 mm square": floor.Carpet(0.8, -3.0, 6.0, 3.0, thickness=0.012),
            "threshold 18 mm": floor.Threshold(0.8, 0.0, length=3.0, height=0.018)}[feature]
    p = params(base, mount, payload, floor_features=(feat,))
    m, d, r = make(p, q)
    dt = m.opt.timestep
    u = np.array([math.cos(math.radians(theta)), math.sin(math.radians(theta))])
    w = Watch(r)
    while d.time < 4.5 and r.pose()[0] < 2.2:
        r.drive_velocity(1.0 * u, None, r.pose()[2])
        r.step()
        w.step(dt)
    return dict(base=base, mount=mount, pose=pose, payload=payload, feature=feature, theta=theta, **w.result())


def fw_job(args):
    """The same stop through the real firmware: SilRobot ticks the C++ core, SimLink carries DRIVE_VEL_BODY."""
    base, mount, pose, q, payload, v, theta, phi = args
    import sil_link  # noqa: F401
    from omni_mcu import protocol as PR
    from sil_link import SimLink
    from sil_robot import SilRobot
    p = params(base, mount, payload)
    q = (A.pan_for(phi), *q[1:])
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = SilRobot(m, d, p, seed=1)
    r.arm.set_pose(q, snap=True)
    mujoco.mj_forward(m, d)
    link = SimLink(r)
    w = None
    th = math.radians(theta)
    go = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=v * math.cos(th), b=v * math.sin(th), flags=PR.CMD_HOLD_YAW)
    stop = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.0, b=0.0, flags=PR.CMD_HOLD_YAW)
    t_next, t_go, t_stop, v_peak = 0.0, 1.0, None, 0.0
    try:
        while d.time < 9.0:
            t = d.time
            if t >= t_next - 1e-9:
                t_next = t + 0.01
                link.send(stop if t < t_go or (t_stop is not None and t >= t_stop) else go, t)
            link.poll(t)
            r.step()
            if t >= t_go:
                w = w or Watch(r)
                w.step(m.opt.timestep)
                sp = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
                v_peak = max(v_peak, sp)
                if t_stop is None and sp > 0.98 * v:
                    t_stop = t + 0.5
                if t_stop is not None and t > t_stop + 2.0 or w.tilt > TILT_OVER:
                    break
    finally:
        r.close()
    return dict(base=base, mount=mount, pose=pose, payload=payload, v=v, theta=theta, phi=phi, v_peak=round(v_peak, 3),
                **w.result())


# ---------------------------------------------------------------------------

def run(parts):
    path = RESULTS / "arm_study.json"
    out = json.loads(path.read_text()) if path.exists() else {}
    workers = min(24, os.cpu_count() or 4)
    reach = {}
    if {"R", "S", "D", "T", "F", "B"} & set(parts):
        with ProcessPoolExecutor(workers) as pool:
            for res in pool.map(reach_job, [(mt, pan) for mt in MOUNTS for pan in (0.0, 90.0)]):
                reach[(res["mount"], res["pan"])] = res
    poses = {mt: poses_for(mt, reach[(mt, 0.0)]["_rows"]) for mt in MOUNTS} if reach else {}
    if "R" in parts:
        out["reach"] = [{k: v for k, v in r.items() if k != "_rows"} for r in reach.values()]
        out["poses"] = {mt: {k: [round(x, 4) for x in q] for k, q in ps.items()} for mt, ps in poses.items()}
        out["lidar_shadow"] = {mt: {k: lidar_shadow(mt, q) for k, q in ps.items()} for mt, ps in poses.items()}
        out["mass"] = {f"{b}/{mt}": dict(base_kg=base_mass_cg(b, mt)[0], base_cg=base_mass_cg(b, mt)[1].tolist())
                       for b in ("sim", "cad") for mt in MOUNTS}
        out["cad_trim"] = {mt: cad_trim(mt) for mt in MOUNTS}
        plot_reach(reach, poses)
    if "S" in parts:
        out["static"] = static_part(reach)
    jobs_d, jobs_s, jobs_t, jobs_f = [], [], [], []
    if "D" in parts:
        for v in (1.0, 2.0):                 # the baseline: today's robot with its catch module
            for theta in (0.0, 90.0, 45.0):
                jobs_d.append(("catch", "centre", "catch module", None, 0.0, v, theta, 0.0, {}))
        for base in ("sim", "cad"):
            for mt in MOUNTS:
                for pose in ("stow", "carry", "level", "floor", "up"):
                    for pl in PAYLOADS:
                        for v in (1.0, 2.0):
                            for theta, phi in ((0.0, 0.0), (90.0, 0.0), (45.0, 0.0), (45.0, 45.0)):
                                jobs_d.append((base, mt, pose, poses[mt][pose], pl, v, theta, phi, {}))
                for pl in PAYLOADS:
                    jobs_s.append((base, mt, "pan", poses[mt]["level"], None, pl))
                    jobs_s.append((base, mt, "lift", poses[mt]["floor"], poses[mt]["up"], pl))
    if "T" in parts:
        for mt in MOUNTS:
            for pose in ("stow", "level"):
                for pl in (0.0, 0.25):
                    for feat in ("rug 12 mm square", "threshold 18 mm"):
                        for theta in (0.0, 45.0):
                            jobs_t.append(("sim", mt, pose, poses[mt][pose], pl, feat, theta))
    with ProcessPoolExecutor(workers) as pool:
        if jobs_d:
            out["stops"] = list(pool.map(stop_job, jobs_d, chunksize=1))
            out["swings"] = list(pool.map(swing_job, jobs_s, chunksize=1))
        if jobs_t:
            out["edges"] = list(pool.map(edge_job, jobs_t, chunksize=1))
        if "F" in parts or "B" in parts:
            stops = [s for s in out.get("stops", []) if s["base"] != "catch"]
            worst = sorted([s for s in stops if s["v"] == 2.0], key=lambda s: (s["tipped"], s["tilt"]))[-4:]
            if "F" in parts:
                jf = [(s["base"], s["mount"], s["pose"], poses[s["mount"]][s["pose"]], s["payload"], s["v"],
                       s["theta"], s["phi"]) for s in worst]
                out["firmware"] = list(pool.map(fw_job, jf, chunksize=1))
            if "B" in parts:
                out["fixes"] = list(pool.map(stop_job, fix_jobs(stops, poses), chunksize=1))
    save("arm_study", out)
    report(out)


def fix_jobs(stops, poses):
    """The worst stops the servos can hold (payload <= 0.25 kg): any that tipped, else the four whose tilt rose most
    above rest; each again with the acceleration table scaled down (as kid mode's accel_scale), or with ballast under
    the deck (1 / 2 kg at the battery)."""
    jobs = []
    held = [s for s in stops if s["payload"] <= 0.25]
    bad = [s for s in held if s["tipped"]] or sorted(held, key=lambda s: s["tilt"] - s["tilt0"])[-4:]
    for s in bad:
        q = poses[s["mount"]][s["pose"]]
        for scale in (0.75, 0.5, 0.35):
            p = v11_params()
            dirs, vals = p.accel_table
            jobs.append((s["base"], s["mount"], s["pose"], q, s["payload"], s["v"], s["theta"], s["phi"],
                         dict(accel_table=(dirs, tuple(v * scale for v in vals)), _fix=f"accel x{scale}")))
        for kg in (1.0, 2.0):
            jobs.append((s["base"], s["mount"], s["pose"], q, s["payload"], s["v"], s["theta"], s["phi"],
                         dict(_ballast=kg, _fix=f"ballast {kg:.0f} kg")))
    return jobs


def plot_reach(reach, poses):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(MOUNTS), 2, figsize=(12, 5.2 * len(MOUNTS)), squeeze=False)
    for i, mt in enumerate(MOUNTS):
        p = params("sim", mt)
        fk = ArmFK(p)
        for j, pan in enumerate((0.0, 90.0)):
            ax = axs[i][j]
            rows = reach[(mt, pan)]["_rows"]
            u = np.array([math.cos(math.radians(pan)), math.sin(math.radians(pan))])
            s, z = rows[:, 3:5] @ u, rows[:, 5]
            h, xe, ze = np.histogram2d(s, z, bins=(np.arange(-0.45, 0.75, 0.01), np.arange(0.0, 1.05, 0.01)))
            ax.imshow((h.T > 0).astype(float), origin="lower", extent=(xe[0], xe[-1], ze[0], ze[-1]), cmap="Blues",
                      vmin=0, vmax=1.6, aspect="equal")
            ax.add_patch(plt.Rectangle((-BUMPER_R, 0), 2 * BUMPER_R, fk.plate_top, color="0.35", alpha=0.6))
            if pan == 0.0:
                (x0, x1), _, zt = fk.cam_boxes[0]
                ax.add_patch(plt.Rectangle((x0, fk.plate_top), x1 - x0, zt - fk.plate_top, color="0.15",
                                           alpha=1.0 if len(fk.cam_boxes) == 1 else 0.3))
                (x0, x1), _, zt = DISPLAY_BOX
                ax.add_patch(plt.Rectangle((x0, fk.plate_top), x1 - x0, zt - fk.plate_top, color="0.15"))
            for name, (z0, _) in HEIGHTS.items():
                if name != "floor":
                    ax.axhline(z0 - 0.01, color="0.6", lw=0.8, ls="--")
                    ax.text(-0.44, z0 - 0.005, name, fontsize=7, color="0.4")
            ax.axvline(0.106, color="tab:red", lw=0.8, ls=":")       # the X4's edge, ahead and to each side
            ax.text(0.11, 0.95, "support edge", fontsize=7, color="tab:red")
            if pan == 0.0:
                for name, q in poses[mt].items():
                    fk.set(q)
                    P, _ = fk.link_points(2)
                    t = fk.tool_pos()
                    ax.plot(*t[[0, 2]], "o", ms=4)
                    ax.annotate(name, t[[0, 2]], fontsize=7, xytext=(3, 3), textcoords="offset points")
            ax.set_title(f"mount {mt} ({MOUNTS[mt][0] * 1000:.0f} mm), arm {'ahead' if pan == 0 else 'to the side'}")
            ax.set_xlabel("m from the robot's centre along the arm")
            ax.set_ylabel("tool height, m")
            ax.set_xlim(-0.45, 0.75)
            ax.set_ylim(0, 1.0)
    fig.suptitle("SO-101 on the v11 robot: where the tool point reaches (blue), robot outline (grey)")
    fig.tight_layout()
    fig.savefig(RESULTS / "arm_reach.png", dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------------------

def _r(a):
    return "-" if a is None else f"{a[0]:.2f}-{a[1]:.2f}"


def report(o):
    if "mass" in o:
        print("Mass without the arm (the arm adds 0.76 kg):")
        for k, v in o["mass"].items():
            print(f"  {k:12s} {v['base_kg']:.2f} kg, CG {1000 * v['base_cg'][0]:+.0f} / {1000 * v['base_cg'][2]:.0f} mm "
                  "(x / height)")
        print(f"  cad trim: {o['cad_trim']}")
    if "reach" in o:
        print("\nReach of the tool point (m from the robot's centre along the arm; the bumper is at 0.217):")
        print(f"  {'mount':7s} {'arm':6s} {'floor':>10s} {'top-down':>10s} {'0.45 m':>10s} {'0.75 m':>10s} "
              f"{'0.90 m':>10s} {'highest':>8s}")
        for r in o["reach"]:
            print(f"  {r['mount']:7s} {'ahead' if r['pan'] == 0 else 'side':6s} {_r(r['floor']):>10s} "
                  f"{_r(r['floor top-down']):>10s} {_r(r['coffee table 0.45']):>10s} {_r(r['dining table 0.75']):>10s} "
                  f"{_r(r['counter 0.90']):>10s} {r['top']:8.2f}")
        print("  lidar shadow by pose (deg of the scan):", o["lidar_shadow"])
    if "static" in o:
        print("\nStatic, arm ahead (phi 0: over the support edge 106 mm out) / over a wheel (45: 150 mm). Servo-held "
              "payload at stall / rated torque; payload that tips (sim / cad base); push, pull at the tool that tip "
              "(sim base):")
        for r in o["static"]:
            if round(r["s"] * 100) % 4:
                continue
            a, b = r["sim_0"], r["cad_0"]
            c = r["sim_45"]
            print(f"  {r['mount']:6s} {r['line']:18s} s {r['s']:.2f}  holds {r['stall_kg']:5.2f} / {r['rated_kg']:4.2f}"
                  f"  tips at {a['payload_tip']:6.2f} / {b['payload_tip']:6.2f} kg (wheel: {c['payload_tip']:6.2f})"
                  f"  push {a['push']:5.1f} N pull {a['pull']:5.1f} N press {a['press']:6.1f} N")
    if "stops" in o:
        print("\nStops (the firmware's ramp from 1 and 2 m/s, 4 directions). At rest: tilt, the tilt that would put the "
              "CG over an edge, the lightest wheel. Worst run: peak tilt, a loaded wheel off the floor (ms), servo "
              "use of stall, tool sag (mm); runs tipped:")
        rows = {}
        for s in o["stops"]:
            k = (s["base"], s["mount"], s["pose"], s["payload"])
            rows.setdefault(k, []).append(s)
        for k, ss in rows.items():
            w = max(ss, key=lambda s: (s["tipped"], s["tilt"]))
            w1 = max((s for s in ss if s["v"] == 1.0), key=lambda s: s["lift_ms"])
            w2 = max((s for s in ss if s["v"] == 2.0), key=lambda s: s["lift_ms"])
            tip = sum(s["tipped"] for s in ss)
            print(f"  {k[0]:5s} {k[1]:6s} {k[2]:12s} {k[3]:4.2f} kg | rest {w['tilt0']:4.1f} deg, tips at "
                  f"{w['tip_angle']:4.1f}, lightest {min(w['rest_loads']):5.1f} N | peak {w['tilt']:5.1f} deg "
                  f"(v {w['v']:.0f} th {w['theta']:.0f} phi {w['phi']:.0f}), lift {w1['lift_ms']:4.0f} / "
                  f"{w2['lift_ms']:4.0f} ms at 1 / 2 m/s, sat {w['sat']:.2f}, sag {w['sag_mm']:4.1f} | {tip}/{len(ss)}")
        print("  parked swings (pan 90 deg each way at full speed with the arm level; floor reach to straight up):")
        for s in o["swings"]:
            print(f"    {s['base']}/{s['mount']:6s} {s['kind']:4s} {s['payload']:.2f} kg: rest {s['tilt0']:.1f}, peak "
                  f"{s['tilt']:.1f} deg (tips at {s['tip_angle']:.1f}), lift {s['lift_ms']:.0f} ms, tipped {s['tipped']}")
    if "edges" in o:
        print("\nFloor edges at 1 m/s (sim base):")
        for s in o["edges"]:
            print(f"  {s['mount']:6s} {s['pose']:5s} {s['payload']:.2f} kg {s['feature']:17s} th {s['theta']:2.0f}: "
                  f"rest {s['tilt0']:.1f}, peak {s['tilt']:5.1f} deg (tips at {s['tip_angle']:.1f}), lift "
                  f"{s['lift_ms']:4.0f} ms, tipped {s['tipped']}")
    if "firmware" in o:
        print("\nThe worst stops through the real firmware (SilRobot) vs the Python controller:")
        py = {(s["base"], s["mount"], s["pose"], s["payload"], s["v"], s["theta"], s["phi"]): s for s in o["stops"]}
        for s in o["firmware"]:
            q = py[(s["base"], s["mount"], s["pose"], s["payload"], s["v"], s["theta"], s["phi"])]
            print(f"  {s['base']}/{s['mount']} {s['pose']} {s['payload']} kg v {s['v']} th {s['theta']:.0f}: firmware tilt "
                  f"{s['tilt']:.1f} lift {s['lift_ms']:.0f} ms tipped {s['tipped']}  |  python tilt {q['tilt']:.1f} "
                  f"lift {q['lift_ms']:.0f} ms tipped {q['tipped']}")
    if "fixes" in o:
        print("\nThe worst stops the servos can hold (<= 0.25 kg), slower or ballasted:")
        py = {(s["base"], s["mount"], s["pose"], s["payload"], s["v"], s["theta"], s["phi"]): s for s in o["stops"]}
        last = None
        for s in o["fixes"]:
            k = (s["base"], s["mount"], s["pose"], s["payload"], s["v"], s["theta"], s["phi"])
            if k != last:
                q = py[k]
                print(f"  {s['base']}/{s['mount']} {s['pose']} {s['payload']} kg v {s['v']} th {s['theta']:.0f} phi "
                      f"{s['phi']:.0f}: as is: peak {q['tilt']:5.1f} deg, lift {q['lift_ms']:4.0f} ms")
                last = k
            print(f"      {s['fix']:12s}: rest {s['tilt0']:.1f}, peak {s['tilt']:5.1f} deg (tips at "
                  f"{s['tip_angle']:.1f}), lift {s['lift_ms']:4.0f} ms, tipped {s['tipped']}")


def kit_stops():
    """K: the firmware's ramped stops with the arm kit as drawn (design.md 12.3: x 0.14 on a 50 mm riser): stowed from
    1 and 2 m/s, upright (the "high" transit pose) and reaching for the floor ahead from 1 m/s (the "arm out" cap), with
    0 and 0.25 kg at the tool, both mass models, four directions."""
    from robot_export import ARM_POSES
    a6 = {r["key"]: r for r in json.loads((RESULTS / "layout_trade.json").read_text())["layouts"]}["A6"]
    poses = {"stow": ARM_POSES["stow"], "high": ARM_POSES["high"], "far grasp": a6["arm"]["grasps"]["far"][1]}
    jobs = []
    for base in ("sim", "cad"):
        for pose, q in poses.items():
            for payload in ((0.0,) if pose == "stow" else (0.0, 0.25)):
                for v in ((1.0, 2.0) if pose == "stow" else (1.0,)):
                    for theta in (0.0, 45.0, 90.0, 180.0):
                        jobs.append((base, "kit", pose, tuple(q), payload, v, theta, math.degrees(-q[0]), {}))
    with ProcessPoolExecutor(min(24, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(stop_job, jobs))
    path = RESULTS / "arm_study.json"
    out = json.loads(path.read_text()) if path.exists() else {}
    out["kit_stops"] = rows
    save("arm_study", out)
    print(f"{len(rows)} stops, tipped {sum(r['tipped'] for r in rows)}; peak tilt {max(r['tilt'] for r in rows):.1f} deg "
          f"(the CG over an edge at {min(r['tip_angle'] for r in rows):.1f} deg)")
    for pose in poses:
        rr = [r for r in rows if r["pose"] == pose]
        w = max(rr, key=lambda r: r["tilt"] / max(r["tip_angle"], 1e-3))
        print(f"  {pose:10s} peak {max(r['tilt'] for r in rr):5.1f} deg, worst {w['tilt']:.1f} of {w['tip_angle']:.1f} "
              f"({w['base']}, {w['v']} m/s at {w['theta']:.0f} deg, {w['payload']} kg), a wheel off the floor up to "
              f"{max(r['lift_ms'] for r in rr):.0f} ms")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:]]
    if args == ["report"]:
        report(json.loads((RESULTS / "arm_study.json").read_text()))
    elif args == ["K"]:
        kit_stops()
    else:
        run(args or ["R", "S", "D", "T", "F", "B"])
