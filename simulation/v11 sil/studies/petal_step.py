"""Hub "petals" against a step (Nov 11 2026, Fernando: when the plastic spoke end that holds two roller axles -- the
petal -- meets a step edge, the wheel climbs worse than when a roller meets it; would a chamfer or round let the
petal ride up until the rollers take over?). Sim only, a stand-alone single-wheel rig.

    python -B studies/petal_step.py [quick|all] [-j N]        (omni-robot-sim interpreter, from simulation/v11 sil)

The wheel is cad/v13's (dims.py, copied below): Ø140, two rows of six barrel rollers (axes at r 52, rows x -+17.25)
with the tread's R2 end fillet, each roller its own body on a free hinge; the hub's spokes as drawn by
wheel_motor.web(): per axle block, the convex hull of the R8 block circle and its two feet on the drum, run along the
wheel axis from the row's outboard face (x -29.25 / +29.0) to the channel (x -+3). The petals' R0.8 edge rounds and
the roller clearance cuts are left out (they do not change the outer shape a step meets).
A petal variant trims the spokes' outboard ends with a cone about the wheel axis: keep r <= r_face + (X_out - |x|)
tan(beta): at the outboard face the spokes stop at radius r_face and rise inboard at beta. "rollers only" removes
every spoke (the ideal).

Rig: the wheel on its pod spring (2.6 N/mm, cad/v13) under a quarter of the arm robot (2.4 kg in all), the body pushed
toward a square step at v by up to F_push (the other three wheels' help), the wheel's speed loop rolling it at the
matching speed with the AK40-10's 4 N m. The wheel meets the step edge at approach angle psi: 0 = rolling straight
at it, 90 = moving along its axle (the rollers turn), the cartridge-side row (row 1) leading. The contact phase is
swept through one spoke pitch (30 deg) by the wheel's start angle. Crossed: the wheel centre gets 40 mm past the edge
on top of the step within the time limit.
PLA spokes on the step: mu 0.35; TPU rollers: 0.9 (est). MuJoCo's soft contacts and convex hulls: a relative test of
the shapes, not a prediction of the real climb height.
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "petal_step"

# ---- cad/v13 dims.py (mm) -----------------------------------------------------------------------------------------------
R_WHEEL, R_AXIS, D_ROW, N_PER_ROW = 70.0, 52.0, 17.25, 6
HT, FILLET_TREAD = 19.5, 2.0
JOURNAL, THRUST_GAP = 0.75, 0.3
ZF = HT + JOURNAL + THRUST_GAP
ENGAGE, HEAD_DEPTH, PILOT_DEPTH_EXTRA = 6.5, 3.0, 0.8
SEAT = 50.0 - ZF - ENGAGE
B_OUT = SEAT + HEAD_DEPTH
PILOT_DEPTH = ENGAGE + PILOT_DEPTH_EXTRA
A_OUT = ZF + PILOT_DEPTH + 1.2
BLOCK_R, WEB_OUT, WEB_GAP, WEB_REACH = 8.0, 12.0, 6.0, 12.0
DRUM_OUT, X_LEFT, X_RIGHT = 33.0, -39.5, 29.0


@dataclass(frozen=True)
class Petal:
    name: str = "v13 petals"
    present: bool = True
    r_face: float = 99.0            # mm: spokes stop at this radius at the outboard face (99 = no trim)
    beta_deg: float = 45.0          # and rise inboard at this angle (from the wheel axis)
    web_out: float = WEB_OUT        # spoke reach outboard of the row plane
    mu: float = 0.35                # PLA on the step (est)
    r_max: float = 99.0             # spoke tips cut to this radius (cad/v13 wheel_quiet round 2: 62.5)
    block_r: float = BLOCK_R        # axle block radius (cad/v13 wheel_quiet round 3: 6)
    flanks: bool = False            # round 3: seen along its axle each block is the block_r round + 45 deg flanks to the faces


@dataclass(frozen=True)
class Case:
    petal: Petal = Petal()
    psi_deg: float = 45.0
    h: float = 0.012                # step height, m
    phase_deg: float = 0.0
    v: float = 0.3
    F_push: float = 30.0
    side: int = 1                   # 1: row 1 (cartridge side) leads when moving along the axle; -1: row 2
    t_extra: float = 2.5


def roller_frames():
    """(row, k, x_row, theta, flip) as wheel_motor.roller_frames()"""
    out = []
    for row in (1, 2):
        x = -D_ROW if row == 1 else D_ROW
        for k in range(1, N_PER_ROW + 1):
            th = math.radians(60 * (k - 1) + (0 if row == 1 else 30))
            out.append((row, k, x, th, k % 2 == 0))
    return out


def web_profile(zc, br=BLOCK_R):
    """wheel_motor.web(): hull of the block circle (local y 0, z zc) and two feet 1 mm inside the drum, local (y, z)"""
    pts = [(br * math.cos(a), zc + br * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 48, endpoint=False)]
    for dz in (-WEB_REACH, WEB_REACH):
        z = max(-DRUM_OUT + 1, min(DRUM_OUT - 1, zc + dz))
        pts.append((-R_AXIS + math.sqrt(DRUM_OUT ** 2 - z * z) - 1.0, z))
    return np.array(pts)


def spoke_pieces(p: Petal):
    """convex point sets (mm, hub frame: x = axle) of every spoke piece, trimmed by the petal cone"""
    if not p.present:
        return []
    zb, za = (ZF + B_OUT) / 2, -(ZF + A_OUT) / 2
    tb = math.tan(math.radians(p.beta_deg))
    pieces = []
    for row, k, xr, th, flip in roller_frames():
        x0, x1 = (-(D_ROW + p.web_out), -WEB_GAP / 2) if row == 1 else (WEB_GAP / 2, min(D_ROW + p.web_out, X_RIGHT))
        x_out = abs(x0) if row == 1 else x1
        c, s = math.cos(th), math.sin(th)
        for zc in (zb, za):
            prof = web_profile(zc, p.block_r)
            if p.flanks:      # densify so the per-point x limits below trace the flanks
                prof = np.vstack([prof] + [np.c_[np.full(25, yv), np.linspace(prof[:, 1].min(), prof[:, 1].max(), 25)]
                                           for yv in np.linspace(prof[:, 0].min(), prof[:, 0].max(), 25)])
                from scipy.spatial import Delaunay
                hull_ok = Delaunay(web_profile(zc, p.block_r)).find_simplex(prof) >= 0
                prof = prof[hull_ok]
            yl, zl = prof[:, 0], prof[:, 1] * (-1 if flip else 1)
            Y = (R_AXIS + yl) * c - zl * s
            Z = (R_AXIS + yl) * s + zl * c
            if p.r_max < 90:
                # the profile cut by the circle r_max (both convex): its points inside plus the arc points inside it
                from matplotlib.path import Path as MPath
                from scipy.spatial import ConvexHull
                P2 = np.c_[Y, Z]
                hull = MPath(P2[ConvexHull(P2).vertices])
                ang = np.linspace(0, 2 * math.pi, 1440, endpoint=False)
                arc = np.c_[p.r_max * np.cos(ang), p.r_max * np.sin(ang)]
                P2 = np.vstack([P2[np.hypot(Y, Z) <= p.r_max], arc[hull.contains_points(arc)]])
                if len(P2) < 3:
                    continue
                Y, Z = P2[:, 0], P2[:, 1]
            r = np.hypot(Y, Z)
            # each profile point (fixed r) stays where |x| <= X_out - (r - r_face) / tan(beta)
            xlim = np.minimum(x_out, x_out - (r - p.r_face) / tb)
            if p.flanks:      # |x - x_row| <= the side profile's half width at this block-local y
                yy = yl                                   # block-local y (flanks are only used without r_max)
                c45 = p.block_r / math.sqrt(2)
                halfw = np.where(yy > c45, np.sqrt(np.maximum(p.block_r ** 2 - yy ** 2, 0.0)), p.block_r * math.sqrt(2) - yy)
            pts = []
            for i, (Yi, Zi, xl) in enumerate(zip(Y, Z, xlim)):
                if row == 1:
                    lo, hi = -xl, x1
                else:
                    lo, hi = x0, xl
                if p.flanks:
                    if halfw[i] <= 0:
                        continue
                    lo, hi = max(lo, xr - halfw[i]), min(hi, xr + halfw[i])
                if hi - lo > 0.05:
                    pts += [(lo, Yi, Zi), (hi, Yi, Zi)]
            if len(pts) >= 8:
                pieces.append(np.array(pts))
    return pieces


def roller_points(nu=21, nt=24):
    """barrel roller (local z = axis), tread on the R70 circle, R2 end fillet; mm"""
    a = R_AXIS
    rp = R_WHEEL - FILLET_TREAD
    uc = HT - FILLET_TREAD
    wc = -a + math.sqrt(rp ** 2 - uc ** 2)               # fillet centre (u, w) in the roller frame
    phi_t = math.atan2(uc, wc + a)                      # tangency on the arc
    prof = []
    for ph in np.linspace(-phi_t, phi_t, nu):
        prof.append((R_WHEEL * math.sin(ph), R_WHEEL * math.cos(ph) - a))
    for sgn in (-1, 1):
        for q in np.linspace(phi_t, math.pi / 2, 6):
            prof.append((sgn * (uc + FILLET_TREAD * math.sin(q)), wc + FILLET_TREAD * math.cos(q)))
        prof.append((sgn * HT, 4.0))
    pts = []
    for u, w in prof:
        for t in np.linspace(0, 2 * math.pi, nt, endpoint=False):
            pts.append((w * math.cos(t), w * math.sin(t), u))
    return np.array(pts)


def _f(*v):
    return " ".join(f"{x:.6g}" for x in v)


def quat_from_mat(Rm):
    import mujoco
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(Rm, float).flatten())
    return q


def build_xml(c: Case) -> str:
    psi = math.radians(c.psi_deg)
    # hub frame: x = axle. Moving +X world; rolling direction d at psi from +X; axle a = (-sin, cos) so that at psi 90
    # the body moves along -x_hub (row 1, x < 0, leads); side -1 turns the wheel round
    d = np.array([math.cos(psi), math.sin(psi), 0.0])
    ax = np.array([-math.sin(psi), math.cos(psi), 0.0]) * c.side
    zz = np.array([0.0, 0.0, 1.0])
    yy = np.cross(zz, ax)
    Rh = np.stack([ax, yy, zz], 1)                       # hub axes in world (columns)
    qh = quat_from_mat(Rh)
    meshes, geoms = [], []
    rp = roller_points() / 1000
    meshes.append(f'<mesh name="roller" vertex="{_f(*rp.flatten())}"/>')
    rollers = []
    for i, (row, k, xr, th, flip) in enumerate(roller_frames()):
        cth, sth = math.cos(th), math.sin(th)
        pos = np.array([xr, R_AXIS * cth, R_AXIS * sth]) / 1000
        M = np.stack([[1, 0, 0], [0, cth, sth], [0, -sth, cth]], 1)    # local x, y (radial), z (tangent) in hub frame
        q = quat_from_mat(M)
        rollers.append(f'<body pos="{_f(*pos)}" quat="{_f(*q)}"><joint type="hinge" axis="0 0 1" damping="2e-6"/>'
                       f'<geom class="roller" mesh="roller" mass="0.026"/></body>')
    for j, pts in enumerate(spoke_pieces(c.petal)):
        meshes.append(f'<mesh name="spoke{j}" vertex="{_f(*(pts / 1000).flatten())}"/>')
        geoms.append(f'<geom class="petal" mesh="spoke{j}"/>')
    R = R_WHEEL / 1000
    m_s, m_h = 1.57, 0.55
    k_pod = 2600.0
    preload = m_s * 9.81
    h = c.h
    x_edge = 0.15
    return f"""
<mujoco model="petal_step">
  <compiler angle="radian"/>
  <option timestep="0.0005" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="5"/>
  <default>
    <geom contype="2" conaffinity="0" condim="3" solref="0.002 1" solimp="0.95 0.99 0.0005"/>
    <default class="roller"><geom type="mesh" friction="0.9 0.005 0.0001" rgba="0.2 0.24 0.32 1"/></default>
    <default class="petal"><geom type="mesh" friction="{c.petal.mu} 0.005 0.0001" rgba="0.93 0.52 0.2 1" mass="0.004"/></default>
  </default>
  <asset>{''.join(meshes)}</asset>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" contype="1" conaffinity="2" friction="0.3 0.005 0.0001"/>
    <geom name="step" type="box" pos="{_f(x_edge + 0.5, 0, h / 2)}" size="{_f(0.5, 0.5, h / 2)}" contype="1" conaffinity="2" friction="0.3 0.005 0.0001"/>
    <body name="carriage" pos="{_f(0, 0, R + 0.0)}">
      <joint name="cx" type="slide" axis="1 0 0" damping="5"/>
      <joint name="cz" type="slide" axis="0 0 1"/>
      <geom type="sphere" size="0.01" mass="{m_s}" contype="0" conaffinity="0"/>
      <body name="pod">
        <joint name="pod" type="slide" axis="0 0 1" stiffness="{k_pod}" springref="{-preload / k_pod:.5f}" damping="{2 * 0.3 * math.sqrt(k_pod * m_s):.3f}"
               range="-0.003 0.015" limited="true"/>
        <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
        <body name="hub" quat="{_f(*qh)}">
          <joint name="spin" type="hinge" axis="1 0 0" armature="9.7e-4" damping="1e-4"/>
          <geom type="cylinder" fromto="{_f(X_LEFT / 1000, 0, 0, X_RIGHT / 1000, 0, 0)}" size="{DRUM_OUT / 1000}" mass="{m_h}"
                contype="2" conaffinity="0" friction="0.35 0.005 0.0001" rgba="0.93 0.52 0.2 1"/>
          {''.join(geoms)}
          {''.join(rollers)}
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="push" joint="cx" kv="400" forcerange="{-c.F_push} {c.F_push}" forcelimited="true"/>
    <velocity name="motor" joint="spin" kv="2.0" forcerange="-4 4" forcelimited="true"/>
  </actuator>
</mujoco>"""


def run_case(c: Case):
    import mujoco
    m = mujoco.MjModel.from_xml_string(build_xml(c))
    d = mujoco.MjData(m)
    R = R_WHEEL / 1000
    psi = math.radians(c.psi_deg)
    jx, jz, jp, js = (m.joint(n).qposadr[0] for n in ("cx", "cz", "pod", "spin"))
    vx, vs = (m.joint(n).dofadr[0] for n in ("cx", "spin"))
    # spin sign: the contact point must not slip along the rolling direction
    hub = m.body("hub").id
    mujoco.mj_forward(m, d)
    ax_w = d.xmat[hub].reshape(3, 3)[:, 0]
    w_vec = np.cross(ax_w, [0, 0, -R])          # velocity of the contact point per unit spin rate about ax
    d_roll = np.array([math.cos(psi), math.sin(psi), 0.0])
    sgn = -1.0 if np.dot(w_vec, d_roll) > 0 else 1.0
    w_target = sgn * c.v * math.cos(psi) / R
    d.qpos[js] = math.radians(c.phase_deg) * (1 if c.side == 1 else -1)
    d.qpos[jx] = -0.0
    d.qvel[vx] = c.v
    d.qvel[vs] = w_target
    d.ctrl[0] = c.v
    d.ctrl[1] = w_target
    # settle on the floor
    x_edge = 0.15
    T = (x_edge + 0.05) / c.v + c.t_extra
    zmax, xmax = 0.0, 0.0
    t_cross = None
    stall_t = 0.0
    n = int(T / m.opt.timestep)
    hub_z0 = None
    peak_push = 0.0
    for i in range(n):
        mujoco.mj_step(m, d)
        hx, hz = d.xpos[hub][0], d.xpos[hub][2]
        if hub_z0 is None and i > 200:
            hub_z0 = hz
        xmax = max(xmax, hx)
        zmax = max(zmax, hz)
        peak_push = max(peak_push, abs(d.actuator_force[0]))
        if hx > x_edge + 0.04 and hz > R + c.h - 0.004:
            t_cross = d.time
            break
        if not np.isfinite(hx):
            break
    return dict(crossed=t_cross is not None, t=t_cross, x_reach=float(xmax - x_edge), z_rise=float(zmax - (hub_z0 or R)),
                peak_push=peak_push)


PETALS = {
    "v13 petals": Petal(),
    "chamfer 45 r58": Petal(name="chamfer 45 r58", r_face=58.0, beta_deg=45.0),
    "chamfer 30 r52": Petal(name="chamfer 30 r52", r_face=52.0, beta_deg=30.0),
    "petals 4 mm shorter": Petal(name="petals 4 mm shorter", web_out=8.0),
    "rollers only": Petal(name="rollers only", present=False),
    "wheel_quiet r2 trim": Petal(name="wheel_quiet r2 trim", r_face=50.0, beta_deg=math.degrees(math.atan(16.0 / 7.75)), r_max=62.5),
    "wheel_quiet r3 R6 flanks": Petal(name="wheel_quiet r3 R6 flanks", block_r=6.0, flanks=True),
}


def job(c: Case):
    try:
        return c, run_case(c)
    except Exception as ex:          # noqa: BLE001
        return c, dict(error=repr(ex))


def main():
    which = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "all"
    nj = int(sys.argv[sys.argv.index("-j") + 1]) if "-j" in sys.argv else 12
    OUT.mkdir(parents=True, exist_ok=True)
    phases = np.arange(0, 60, 3.0)
    if which == "sens":            # PLA friction on the step and a weak push, at the angles where petals mattered
        cases = [Case(petal=replace(PETALS[p], mu=mu), psi_deg=psi, h=0.020, phase_deg=ph, F_push=F)
                 for p in ("v13 petals", "chamfer 45 r58", "rollers only") for mu in ((0.15, 0.35, 0.6) if p != "rollers only" else (0.35,))
                 for psi in (60.0, 75.0, 82.0) for F in (12.0, 30.0) for ph in phases]
    elif which == "trim":          # cad/v13 wheel_quiet round 2 against the v13 petals and no petals
        cases = [Case(petal=PETALS[p], psi_deg=psi, h=h, phase_deg=ph, side=sd)
                 for p in ("v13 petals", "wheel_quiet r3 R6 flanks", "rollers only") for psi in (45.0, 60.0, 75.0, 82.0, 90.0)
                 for h in (0.012, 0.016, 0.020, 0.024) for ph in phases for sd in (1, -1)]
    elif which == "quick":
        cases = [Case(petal=PETALS[p], psi_deg=psi, h=0.016, phase_deg=ph) for p in ("v13 petals", "rollers only")
                 for psi in (45.0, 90.0) for ph in (0.0, 15.0)]
    else:
        cases = [Case(petal=PETALS[p], psi_deg=psi, h=h, phase_deg=ph, side=sd)
                 for p in PETALS for psi in (0.0, 30.0, 45.0, 60.0, 75.0, 90.0) for h in (0.012, 0.016, 0.020, 0.024)
                 for ph in phases for sd in ((1, -1) if psi >= 60 else (1,))]
    t0 = time.time()
    rows = []
    with Pool(nj) as pool:
        for c, r in pool.imap_unordered(job, cases, chunksize=4):
            rows.append(dict(petal=c.petal.name + (f" mu {c.petal.mu:g}" if which == "sens" else ""), F_push=c.F_push, psi=c.psi_deg, h_mm=round(c.h * 1000), phase=c.phase_deg, side=c.side, **r))
            if which == "quick":
                print(rows[-1], flush=True)
    (OUT / f"{which}.json").write_text(json.dumps(dict(petals={k: asdict(v) for k, v in PETALS.items()}, rows=rows), indent=1))
    # summary: crossing fraction over phases
    import collections
    tab = collections.defaultdict(list)
    for r in rows:
        if "error" not in r:
            tab[(r["petal"], r["psi"], r["side"], r["h_mm"], r["F_push"])].append(r["crossed"])
    print(f"{len(rows)} runs, {time.time() - t0:.0f} s")
    for key in sorted(tab):
        print(key, f"{np.mean(tab[key]):.2f}")


if __name__ == "__main__":
    main()
