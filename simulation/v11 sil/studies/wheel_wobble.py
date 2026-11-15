"""Omni-wheel "wobble" with the rollers locked (Nov 12 2026, Fernando: the printed v13 wheel, rolled by hand with every
roller fixed, still shakes a hollow hardwood floor; it seems to rock from three rollers to one or two). Sim only, a
stand-alone model; proposal CAD: cad/v13/tools/wheel_quiet_2.py.

    python -B studies/wheel_wobble.py [geom|dyn|all|roll|tune|refine|mj] [-j N]   (omni-robot-sim interpreter, from simulation/v11 sil)
      geom: the support geometry (fixed and free lean); dyn: the variants hand / pod; roll: the fork's roll stiffness;
      tune / refine: end-relief and 2 x 5 sweeps (refine adds load and roller radius error); mj: the robot sim itself.
      Report: results/wheel_wobble_2026-11-12.md

Geometry (exact for the drawn rollers): each roller is a solid of revolution about its axle; its profile is
cad/v13's (R70 arc about the wheel axis, R2 end fillet) with an optional end relief. The hub stands on the floor at the
highest of its rollers' support heights. For a solid of revolution the lowest point in any direction is found section by
section along the axle, so the support height is exact up to the axial sampling (0.02 mm at the ends).

Why it can rock: the two rows sit at x -+ D_ROW (17.25 mm). Leaning the wheel by beta lifts one row's axles by
D_ROW sin(beta) and drops the other's, whatever the roller shape (the roller turns about its own axis under a lean), so a
leaning wheel is no longer round: it climbs onto, or drops off, the other row every 30 deg. Held only at the hub (a hand),
nothing stops the lean: the wheel settles onto one roller of one row and the ends of the two rollers of the other row
either side (the "tripod") and rocks across each time the rows hand over.

Dynamics: rollers locked (no play: the wheel_noise.py clicks are another mechanism), the hub's height z and lean phi
free, Hertzian TPU contact (as wheel_noise.py), the wheel pushed along at constant spin. "hand": the bare wheel pressed
down at its hub by a soft hand with a weak roll hold; "pod": the robot's quarter on the pod spring, lean held by the
fork's roll stiffness, with an optional build/floor camber offset. Proxy for a booming floor: the floor force's ripple,
all of it (N rms) and its 5-500 Hz band (dB re 1 N^2), the hub's lean and height ripple. Only differences mean much.
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import asdict, dataclass, field, replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "wheel_wobble"
G = 9.81


@dataclass(frozen=True)
class Wheel:
    name: str = "v13"
    R: float = 0.070                  # wheel radius (tread profile = this circle about the wheel axis)
    R_axis: float = 0.052             # roller axis radius
    ht: float = 0.0195                # roller half length (cad/v13 HT)
    fillet: float = 0.002             # tread end-edge fillet
    d_row: float = 0.01725            # row planes at x -+ this
    n_row: int = 6
    relief_len: float = 0.0           # end relief: the profile drops relief_depth * (u / relief_len)^2 over the roller's
    relief_depth: float = 0.0         # last relief_len (u from where it starts), before the end fillet
    E_tread: float = 26e6             # TPU 95A (85A ~12 MPa)
    c_h: float = 3.0                  # s/m Hunt-Crossley damping, tread on floor
    relief_arc: bool = False          # True: the relief is the tangent arc cad/v13 wheel_quiet_2 draws (radius from
                                      # 1/Rr = 1/R + 2 depth / len^2, i.e. the same drop to second order), not the parabola
    runout: float = 0.0               # m, 1-sigma per-roller radius error (print), random per roller
    seed: int = 1


def relief_radius(w: Wheel) -> float:
    return 1.0 / (1.0 / w.R + 2.0 * w.relief_depth / w.relief_len ** 2)


def _relieved(w: Wheel, s):
    """the R arc with the end relief (no fillet), s >= 0"""
    rho = np.sqrt(np.maximum(w.R ** 2 - s ** 2, 0.0)) - w.R_axis
    if w.relief_len <= 0:
        return rho
    s0 = w.ht - w.relief_len
    if not w.relief_arc:
        return rho - w.relief_depth * (np.clip(s - s0, 0.0, None) / w.relief_len) ** 2
    Rr = relief_radius(w)
    q = math.sqrt(w.R ** 2 - s0 ** 2)
    cs, cr = s0 - Rr * s0 / w.R, (q - w.R_axis) - Rr * q / w.R
    arc = cr + np.sqrt(np.maximum(Rr ** 2 - (s - cs) ** 2, 0.0))
    return np.where(s > s0, arc, rho)


def profile(w: Wheel, s):
    """roller radius about its axle at axial position s (abs), with the relief and the end fillet"""
    s = np.abs(np.asarray(s, float))
    rho = _relieved(w, s)
    rf = w.fillet
    if rf > 0:
        # fillet tangent to the (relieved) profile and the end face s = ht: found numerically on the profile's offset
        sc = w.ht - rf
        ss = np.linspace(w.ht - 4 * rf, w.ht, 4001)
        r0 = _relieved(w, ss)
        # centre height yc: the circle (sc, yc) radius rf touches the profile from below: max over ss of
        # profile(ss) - sqrt(rf^2 - (ss - sc)^2) where |ss - sc| <= rf  -> yc = min over that of (r0 + ... ) (inside)
        m = np.abs(ss - sc) <= rf
        yc = np.min(r0[m] - np.sqrt(np.maximum(rf ** 2 - (ss[m] - sc) ** 2, 0.0)))
        # tangency where the profile first meets the circle
        f = np.where(s > sc, yc + np.sqrt(np.maximum(rf ** 2 - (s - sc) ** 2, 0.0)), np.inf)
        rho = np.minimum(rho, f)
    return np.where(s <= w.ht, rho, -1.0)


def frames(w: Wheel):
    """roller centres A (n,3) and axes t (n,3) in the hub frame (x = wheel axis), cad/v13 order: row 1 at -x on 0, 60 ..,
    row 2 at +x on 30, 90 .."""
    A, T, X = [], [], []
    for row in (1, 2):
        x = -w.d_row if row == 1 else w.d_row
        for k in range(w.n_row):
            th = 2 * math.pi * k / w.n_row + (0 if row == 1 else math.pi / w.n_row)
            c, s = math.cos(th), math.sin(th)
            A.append((x, w.R_axis * c, w.R_axis * s))
            T.append((0.0, -s, c))
    return np.array(A), np.array(T)


def s_grid(w: Wheel):
    a = np.linspace(-w.ht, w.ht, 401)
    e = np.linspace(w.ht - max(3 * w.fillet, w.relief_len, 0.004), w.ht, 300)
    return np.unique(np.concatenate([a, e, -e]))


class Support:
    """the hub-centre height each roller holds it at, for a hub turned theta about x then leaned phi about the rolling
    direction (world y); floor normal world z"""

    def __init__(self, w: Wheel):
        self.w = w
        self.A, self.T = frames(w)
        self.s = s_grid(w)
        self.rho = profile(w, self.s)
        n = len(self.A)
        self.dr = np.random.default_rng(w.seed).normal(0.0, w.runout, n) if w.runout > 0 else np.zeros(n)

    def rollers(self, theta, phi):
        """per roller: height of the hub centre above the floor it would hold (m) and the contact's world x offset from
        the hub (m)"""
        ct, st, cp, sp = math.cos(theta), math.sin(theta), math.cos(phi), math.sin(phi)
        Rx = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rm = Ry @ Rx
        A = self.A @ Rm.T
        T = self.T @ Rm.T
        # section centres c = A + s T; lowest point = c + rho * wdir, wdir = unit(-z + (z.T) T)
        cz = A[:, 2:3] + self.s[None, :] * T[:, 2:3]
        tz = T[:, 2]
        sn = np.sqrt(np.maximum(1 - tz ** 2, 0.0))[:, None]
        h = -cz + self.rho[None, :] * sn                            # hub height this section holds
        h = np.where(self.rho[None, :] > 0, h, -np.inf)
        j = np.argmax(h, axis=1)
        idx = np.arange(len(A))
        hz = h[idx, j] + self.dr
        # contact x: c.x + rho * wdir.x; wdir = (-z + tz T) / sn
        cx = A[:, 0] + self.s[j] * T[:, 0]
        wx = (tz * T[:, 0]) / np.maximum(sn[:, 0], 1e-9)
        xc = cx + self.rho[j] * wx
        yc = A[:, 1] + self.s[j] * T[:, 1] + self.rho[j] * (tz * T[:, 1]) / np.maximum(sn[:, 0], 1e-9)
        return hz, xc, yc

    def height(self, theta, phi):
        return float(np.max(self.rollers(theta, phi)[0]))


# ---- quasi-static geometry ------------------------------------------------------------------------------------------------
def geom(w: Wheel, betas_deg=(0.0, 0.1, 0.25, 0.5, 1.0, 2.0), dth_deg=0.01, v=0.3):
    """per fixed lean: hub-height ripple (um) over a 60 deg period and the largest vertical-velocity jump at v (m/s), i.e.
    the hub's slope change at a handover (an impact on a rigid wheel); and the free lean (a wheel held at its hub only)"""
    S = Support(w)
    th = np.radians(np.arange(-90.0, -90.0 + 360.0 / w.n_row, dth_deg))  # roller 1 of row 1 at the bottom at -90 deg
    out = dict(name=w.name)
    fixed = {}
    for b in betas_deg:
        ph = math.radians(b)
        h = np.array([S.height(t, ph) for t in th])
        dx = w.R * math.radians(dth_deg)
        slope = np.diff(h) / dx
        jumps = np.abs(np.diff(slope))
        fixed[f"{b:g}"] = dict(pp_um=float(np.ptp(h) * 1e6), dv_mm_s=float(jumps.max() * v * 1e3),
                               slope_max=float(np.abs(slope).max()))
    out["fixed_lean"] = fixed
    # free lean: the hub pushed straight down at x = 0 settles where its height is lowest (two supports either side)
    phis = np.radians(np.linspace(-4.5, 4.5, 181))
    th2 = th[::5]
    hmin, pmin = [], []
    for t in th2:
        hh = np.array([S.height(t, p) for p in phis])
        k = int(np.argmin(hh))
        lo, hi = max(k - 1, 0), min(k + 1, len(phis) - 1)
        pf = np.linspace(phis[lo], phis[hi], 41)
        hf = np.array([S.height(t, p) for p in pf])
        kk = int(np.argmin(hf))
        hmin.append(hf[kk])
        pmin.append(pf[kk])
    hmin, pmin = np.array(hmin), np.array(pmin)
    out["free_lean"] = dict(lean_pp_deg=float(np.degrees(np.ptp(pmin))), lean_max_deg=float(np.degrees(np.abs(pmin).max())),
                            hub_pp_um=float(np.ptp(hmin) * 1e6))
    out["free_trace"] = dict(theta_deg=np.degrees(th2).round(3).tolist(), lean_deg=np.degrees(pmin).round(4).tolist(),
                             hub_um=((hmin - hmin.max()) * 1e6).round(2).tolist())
    # how many rollers touch (within 20 um) in the free stance: the tripod
    return out


# ---- dynamics (rollers locked) ------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Run:
    wheel: Wheel = field(default_factory=Wheel)
    mode: str = "hand"                # "hand" or "pod"
    v: float = 0.3
    F_press: float = 15.0             # hand: N pressed down at the hub
    k_hand: float = 3000.0            # hand: N/m vertical (a soft push)
    k_roll_hand: float = 0.5          # hand: N m/rad roll hold (fingers in the bore: weak)
    m_wheel: float = 0.45             # hand: kg the bare wheel with locked rollers (+ a share of the hand)
    m_hub: float = 0.87               # pod: kg unsprung (hub, rollers, cartridge, 6804, AK40-10, fork share)
    m_sprung: float = 1.57            # pod: robot quarter on the pod spring
    k_pod: float = 2600.0
    zeta_pod: float = 0.3
    k_roll: float = 300.0             # pod: N m/rad fork + pivot roll stiffness at the wheel (est, as wheel_noise.py)
    camber_deg: float = 0.0           # pod: built-in / floor lean the fork holds the wheel at
    I_roll: float = 2.0e-3            # kg m2 about the rolling direction through the hub
    dt: float = 5e-6
    warmup: float = 0.15
    period_deg: float = 120.0         # record this much wheel turn
    fs: float = 20000.0


def simulate(run: Run):
    w = run.wheel
    S = Support(w)
    Re = math.sqrt(w.R * (w.R - w.R_axis))
    k_h = 4.0 / 3.0 * w.E_tread / (1 - 0.48 ** 2) * math.sqrt(Re)
    hand = run.mode == "hand"
    m = run.m_wheel if hand else run.m_hub
    load = (run.m_wheel * G + run.F_press) if hand else (run.m_hub + run.m_sprung) * G
    om = run.v / w.R
    T_total = run.warmup + math.radians(run.period_deg) / om
    n = int(T_total / run.dt)
    every = max(1, int(round(1 / (run.fs * run.dt))))
    phi0 = math.radians(run.camber_deg)
    th0 = -math.pi / 2 + 0.3
    z = S.height(th0, phi0) - (load / k_h) ** (2 / 3)
    vz = 0.0
    phi, phid = phi0, 0.0
    zs, vzs = z + 0.05, 0.0
    L0 = 0.05 - run.m_sprung * G / run.k_pod
    c_pod = 2 * run.zeta_pod * math.sqrt(run.k_pod * run.m_sprung)
    k_r = run.k_roll_hand if hand else run.k_roll
    c_roll = 2 * 0.2 * math.sqrt(max(k_r, 50.0) * run.I_roll)
    c_hand = 2 * 0.5 * math.sqrt(run.k_hand * m)
    zh = z                                   # hand's set point follows its own slow average
    rec = []
    for i in range(n):
        t = i * run.dt
        th = th0 + om * t
        hz, xc, _ = S.rollers(th, phi)
        pen = hz - z
        on = pen > 0
        if on.any():
            # contact point vertical speed: vz + phid * xc (lean about y moves a point at x down by x * dphi)
            vc = -(vz) - phid * xc                                   # penetration rate (approx)
            N = np.where(on, np.maximum(k_h * np.maximum(pen, 0) ** 1.5 * (1 + w.c_h * vc), 0.0), 0.0)
        else:
            N = np.zeros_like(pen)
        Fz = N.sum()
        Mroll = -np.sum(N * xc)                                      # floor pushes up at x: leans the wheel away
        if hand:
            az = (Fz - m * G - run.F_press + run.k_hand * (zh - z) - c_hand * vz) / m
            zh += (z - zh) * run.dt / 0.2
            Fch = 0.0
        else:
            Fch = run.k_pod * (zs - z - L0) + c_pod * (vzs - vz)
            az = (Fz - m * G + Fch) / m
            azs = (-Fch - run.m_sprung * G) / run.m_sprung
            vzs += azs * run.dt
            zs += vzs * run.dt
        phidd = (Mroll - k_r * (phi - phi0) - c_roll * phid) / run.I_roll
        vz += az * run.dt
        z += vz * run.dt
        phid += phidd * run.dt
        phi += phid * run.dt
        if t >= run.warmup and i % every == 0:
            rec.append((t, Fz, phi, z, Fch, int(on.sum())))
    return np.array(rec)


def band_db(x, fs, lo=5.0, hi=500.0):
    from scipy.signal import welch
    f, P = welch(x - x.mean(), fs=fs, nperseg=min(len(x), 8192))
    mk = (f >= lo) & (f <= hi)
    return 10 * math.log10(max(float(np.sum(P[mk]) * (f[1] - f[0])), 1e-30))


def dyn_metrics(run: Run):
    t0 = time.time()
    r = simulate(run)
    F = r[:, 1]
    return dict(floor_rms_N=float(F.std()), floor_pp_N=float(np.ptp(F)), floor_5_500_dB=band_db(F, run.fs),
                floor_500_5k_dB=band_db(F, run.fs, 500.0, 5000.0),
                lean_pp_deg=float(np.degrees(np.ptp(r[:, 2]))), hub_pp_um=float(np.ptp(r[:, 3]) * 1e6),
                chassis_rms_N=float(r[:, 4].std()) if run.mode == "pod" else float("nan"),
                lift_off_frac=float(np.mean(r[:, 5] == 0)), wall_s=time.time() - t0)


# ---- variants -------------------------------------------------------------------------------------------------------------
B = Wheel()
WHEELS = {
    "v13": B,
    "v13, 85A tread": replace(B, name="85A", E_tread=12e6, c_h=4.0),
    "no end fillet (sharp)": replace(B, name="sharp", fillet=0.0002),
    "rows -+14 (smaller rollers)": replace(B, name="d14", d_row=0.014),
    "rows -+10": replace(B, name="d10", d_row=0.010),
    "longer rollers ht 21": replace(B, name="ht21", ht=0.021),
    "longer rollers ht 22.5": replace(B, name="ht22.5", ht=0.0225),
    "ht 22.5 + relief 0.15/5": replace(B, name="ht22.5r", ht=0.0225, relief_len=0.005, relief_depth=0.00015),
    "ht 22.5 + relief 0.3/6": replace(B, name="ht22.5r3", ht=0.0225, relief_len=0.006, relief_depth=0.0003),
    "ht 19.5 + relief 0.15/4": replace(B, name="ht19.5r", relief_len=0.004, relief_depth=0.00015),
}
# round 2: what fits the hub. At 2 x 6 on r 52 the rollers cannot get longer (neighbouring M3 axles meet 9.5 mm past the
# thrust faces); relief within the v13 length; 2 x 5 has 11.6 mm there at a 6 deg overlap (half length 25.1)
TUNE = {"v13": B, "v13, 85A tread": WHEELS["v13, 85A tread"]}
for L in (3.0, 4.0, 5.0, 6.0):
    for dpt in (0.1, 0.2, 0.3):
        TUNE[f"v13 + relief {dpt:g}/{L:g}"] = replace(B, name=f"r{dpt}/{L}", relief_len=L * 1e-3, relief_depth=dpt * 1e-3)
for ov in (4.0, 6.0, 8.0):
    ht = 0.070 * math.sin(math.radians((36.0 + ov) / 2))
    TUNE[f"2x5 ov {ov:g}"] = replace(B, name=f"2x5 ov{ov}", n_row=5, ht=ht)
    for L, dpt in ((5.0, 0.2), (6.0, 0.3), (8.0, 0.3)):
        TUNE[f"2x5 ov {ov:g} + relief {dpt:g}/{L:g}"] = replace(B, name=f"2x5 ov{ov} r{dpt}/{L}", n_row=5, ht=ht,
                                                              relief_len=L * 1e-3, relief_depth=dpt * 1e-3)
WHEELS.update({k: v for k, v in TUNE.items() if k not in WHEELS})
REFINE = {"v13": B, "v13, runout 0.1": replace(B, runout=1e-4)}
for L in (6.0, 7.0, 8.0, 9.0):
    for dpt in (0.2, 0.3, 0.4):
        TUNE_W = replace(B, name=f"arc {dpt}/{L}", relief_len=L * 1e-3, relief_depth=dpt * 1e-3, relief_arc=True)
        REFINE[f"arc relief {dpt:g}/{L:g}"] = TUNE_W
        REFINE[f"arc relief {dpt:g}/{L:g}, runout 0.1"] = replace(TUNE_W, runout=1e-4)
REFINE["arc relief 0.3/8, 85A"] = replace(REFINE["arc relief 0.3/8"], E_tread=12e6, c_h=4.0)
WHEELS.update({k: v for k, v in REFINE.items() if k not in WHEELS})
RCASES = {
    "pod 0.3": dict(mode="pod", v=0.3), "pod 0.6": dict(mode="pod", v=0.6), "pod 1.0": dict(mode="pod", v=1.0),
    "pod 1.0 camber 0.5": dict(mode="pod", v=1.0, camber_deg=0.5),
    "pod 1.0 light (16 N)": dict(mode="pod", v=1.0, m_sprung=0.8),
    "pod 1.0 heavy (43 N, 18 kg robot)": dict(mode="pod", v=1.0, m_sprung=3.5),
    "pod 0.3 heavy": dict(mode="pod", v=0.3, m_sprung=3.5),
}
TCASES = {
    "pod 0.3": dict(mode="pod", v=0.3), "pod 0.6": dict(mode="pod", v=0.6), "pod 1.0": dict(mode="pod", v=1.0),
    "pod 1.0 camber 0.5": dict(mode="pod", v=1.0, camber_deg=0.5), "hand 0.3": dict(mode="hand", v=0.3),
}
CASES = {
    "hand 0.3 m/s": dict(mode="hand", v=0.3),
    "pod 0.3 m/s": dict(mode="pod", v=0.3),
    "pod 1 m/s": dict(mode="pod", v=1.0),
    "pod 0.3 m/s, camber 0.5": dict(mode="pod", v=0.3, camber_deg=0.5),
    "pod 1 m/s, camber 0.5": dict(mode="pod", v=1.0, camber_deg=0.5),
}
ROLL = {f"pod 0.3 m/s, k_roll {k:g}": dict(mode="pod", v=0.3, k_roll=k) for k in (30.0, 100.0, 300.0, 1000.0, 3000.0)}


def mj_job(args):
    """The robot sim itself (robot_export.arm_d435_params, explicit rollers vs the round 'sphere' wheel): drive straight
    at v along wheels 1 / 3's rolling direction (135 deg: they roll, wheels 2 / 4 slide on their rollers) and look for a
    wheel-load ripple at the roller-handover rate (12 per turn)."""
    model, v = args
    sys.path.insert(0, str(HERE.parent))
    from common import make_robot
    from robot_export import arm_d435_params
    p = arm_d435_params(wheel_model=model)
    m, d, r = make_robot(p, settle=0.5)
    u = np.array([math.cos(math.radians(135.0)), math.sin(math.radians(135.0))])
    dt = m.opt.timestep
    rec, t = [], 0.0
    while t < 2.5:
        vv = min(v, t * 1.0)
        r.wheel_cmd = r.kin.wheel_speeds([*(vv * u), 0.0])
        r.step()
        t += dt
        if t > 1.0 + v:
            q = d.sensor("base_quat").data
            roll = math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
            rec.append((t, *r.wheel_loads(), roll, d.sensor("base_linvel").data[2]))
    a = np.array(rec)
    fs = 1 / dt
    f_hand = 12 * v / (2 * math.pi * p.omni.diameter / 2)
    out = dict(model=model, v=v, f_handover_hz=f_hand, dt=dt)
    from scipy.signal import welch
    for i in range(4):
        x = a[:, 1 + i]
        f, P = welch(x - x.mean(), fs=fs, nperseg=min(len(x), 2 ** 14))
        k = int(np.argmin(np.abs(f - f_hand)))
        out[f"wheel{i + 1}"] = dict(mean_N=float(x.mean()), rms_N=float(x.std()), pp_N=float(np.ptp(x)),
                                    at_handover_N=float(math.sqrt(2 * np.sum(P[max(k - 2, 0):k + 3]) * (f[1] - f[0]))))
    out["body_roll_pp_deg"] = float(np.degrees(np.ptp(a[:, 5])))
    out["body_vz_rms_mm_s"] = float(a[:, 6].std() * 1e3)
    return out


def gjob(name):
    return name, geom(WHEELS[name])


def djob(args):
    wname, cname, kw = args
    try:
        return wname, cname, dyn_metrics(Run(wheel=WHEELS[wname], **kw))
    except Exception as ex:          # noqa: BLE001
        return wname, cname, dict(error=repr(ex))


def main():
    which = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "all"
    nj = int(sys.argv[sys.argv.index("-j") + 1]) if "-j" in sys.argv else 6
    only = sys.argv[sys.argv.index("-w") + 1].split(",") if "-w" in sys.argv else list(WHEELS)
    OUT.mkdir(parents=True, exist_ok=True)
    res = {}
    if which == "mj":
        with Pool(4) as pool:
            for o in pool.imap_unordered(mj_job, [(mo, v) for mo in ("rollers", "sphere") for v in (0.3, 1.0)]):
                res.setdefault("mj", []).append(o)
                print(o["model"], o["v"], " ".join(f"w{i}: {o[f'wheel{i}']['mean_N']:.1f} N rms {o[f'wheel{i}']['rms_N']:.2f} "
                                                    f"@hand {o[f'wheel{i}']['at_handover_N']:.2f}" for i in range(1, 5)),
                      f"roll pp {o['body_roll_pp_deg']:.3f} deg vz {o['body_vz_rms_mm_s']:.2f} mm/s", flush=True)
    if which in ("geom", "all"):
        with Pool(nj) as pool:
            for name, g in pool.imap_unordered(gjob, only):
                res.setdefault("geom", {})[name] = g
                fl = " ".join(f"{b}:{v['pp_um']:.0f}um/{v['dv_mm_s']:.0f}" for b, v in g["fixed_lean"].items())
                print(f"{name:28s} lean(deg):pp/dv(mm/s) {fl} | free lean pp {g['free_lean']['lean_pp_deg']:.2f} deg "
                      f"max {g['free_lean']['lean_max_deg']:.2f}, hub pp {g['free_lean']['hub_pp_um']:.0f} um", flush=True)
    if which == "refine":
        jobs = [(wn, cn, kw) for wn in REFINE for cn, kw in RCASES.items()
                if not ("runout" in wn and cn not in ("pod 0.3", "pod 1.0"))]
        with Pool(nj) as pool:
            for wn, cn, m in pool.imap_unordered(djob, jobs):
                res.setdefault("refine", {}).setdefault(wn, {})[cn] = m
                print(f"{wn:34s} {cn:34s} " + (f"floor rms {m['floor_rms_N']:6.2f} N  5-500 Hz {m['floor_5_500_dB']:6.1f} dB  "
                      f"lean pp {m['lean_pp_deg']:.3f}  hub pp {m['hub_pp_um']:5.0f} um" if "error" not in m else m["error"]), flush=True)
        # (no geometry pass here: it is serial and slow; `geom -w <names>` runs it in parallel when wanted)
    if which == "tune":
        jobs = [(wn, cn, kw) for wn in TUNE for cn, kw in TCASES.items()]
        with Pool(nj) as pool:
            for wn, cn, m in pool.imap_unordered(djob, jobs):
                res.setdefault("tune", {}).setdefault(wn, {})[cn] = m
                print(f"{wn:30s} {cn:20s} " + (f"floor rms {m['floor_rms_N']:6.2f} N  5-500 Hz {m['floor_5_500_dB']:6.1f} dB  "
                      f"lean pp {m['lean_pp_deg']:.3f}  hub pp {m['hub_pp_um']:5.0f} um" if "error" not in m else m["error"]), flush=True)
        pass
    if which in ("dyn", "all", "roll"):
        cases = ROLL if which == "roll" else CASES
        wl = ["v13"] if which == "roll" else only
        jobs = [(wn, cn, kw) for wn in wl for cn, kw in cases.items()]
        with Pool(nj) as pool:
            for wn, cn, m in pool.imap_unordered(djob, jobs):
                res.setdefault(which if which == "roll" else "dyn", {}).setdefault(wn, {})[cn] = m
                print(f"{wn:28s} {cn:26s} " + (f"floor rms {m['floor_rms_N']:6.2f} N pp {m['floor_pp_N']:6.1f}  5-500 Hz "
                      f"{m['floor_5_500_dB']:6.1f} dB  0.5-5k {m['floor_500_5k_dB']:6.1f}  lean pp {m['lean_pp_deg']:.3f} deg  "
                      f"hub pp {m['hub_pp_um']:5.0f} um  lift {m['lift_off_frac']:.2f}  ({m['wall_s']:.0f} s)"
                      if "error" not in m else m["error"]), flush=True)
    (OUT / f"{which}.json").write_text(json.dumps(dict(wheels={k: asdict(v) for k, v in WHEELS.items()}, results=res), indent=1))


if __name__ == "__main__":
    main()
