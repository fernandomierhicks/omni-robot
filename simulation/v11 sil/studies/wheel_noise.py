"""Omni-wheel roller "clicks" (Nov 11 2026, Fernando: a hand-rolled printed v13 wheel clicks as every roller meets a hard
floor). Sim only, a stand-alone model: what makes the click, and which wheel changes would quiet it.

    python -B studies/wheel_noise.py [quick|all] [-j N]        (omni-robot-sim interpreter, from simulation/v11 sil)

The model: one wheel in its own plane (y forward, z up), integrated at 1 us so the hard contacts resolve.
  * the hub: y, z, spin theta, and roll phi about y (the two rows sit at x -+17.25 mm, so a load jumping row to row
    rocks the wheel against the fork's roll stiffness);
  * every roller is a separate 2-D body (its own mass), held to its axle by contacts, not a joint: radial play g_r in
    its bore (the Ø3.4 bore on the M3 axle's ~Ø2.94 thread crests: 0.23 mm) and axial play g_a at its thrust faces
    (dims.THRUST_GAP 0.3 each side). Its in-plane orientation follows the hub. Its spin about its own (tangential) axis
    only moves its surface along x, so the in-plane model needs no roller spin;
  * the roller's floor silhouette is cad/v13's: an R70 arc about the hub centre (offset by the roller's play and its
    radius error), ending at the roller end in the tread's R2 fillet; floor contact is Hertzian (TPU tread, effective
    radius sqrt(70 x 18 mm)) with Hunt-Crossley damping and regularized Coulomb friction (mu 0.9);
  * "drive": the robot's quarter mass rides the pod spring above the hub and moves with it in y; the AK40-10's speed
    loop (reflected rotor inertia, 4 N m limit) holds the wheel speed; F_drag (N) on the hub stands for traction the
    wheel must give (pushing the rest of the robot, accelerating, a slope);
    "hand": the bare wheel (no motor) pushed along at v by a stiff hand pressing it down with F_press.
Clicks are impacts at hard contacts (PLA bore on steel axle, PLA journal on PLA thrust face). The noise proxy is the
A-weighted 0.4-10 kHz content of the force the rollers put into the hub (structure-borne: it is what shakes the hub,
fork and chassis that radiate), and of the floor force (a hard floor radiates too), in dB re the baseline. Only
differences between variants mean anything: the absolute level depends on contact stiffness and damping (est) and the
model has no structural modes or acoustic radiation.
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
OUT = HERE / "results" / "wheel_noise"
G = 9.81


@dataclass(frozen=True)
class Wheel:
    name: str = "baseline 2x6"
    n_row: int = 6                    # rollers per row (two rows, staggered half a pitch)
    R: float = 0.070                  # wheel radius (roller profile = this circle about the hub axis)
    rr: float = 0.018                 # roller radius at its middle (axis at R - rr)
    overlap_deg: float = 2.35         # angular overlap of neighbouring rollers (cad/v13: 2.35)
    fillet: float = 0.002             # tread end-edge fillet
    x_row: float = 0.01725            # rows at x -+ this
    g_r: float = 0.23e-3              # radial play of the roller on its axle (bore radius - axle radius)
    g_a: float = 0.30e-3              # axial play each side
    k_b: float = 5e6                  # N/m bore contact (PLA on steel thread crests; est)
    c_b: float = 2.0                  # s/m Hunt-Crossley damping, bore
    k_a: float = 5e6                  # N/m thrust face contact (PLA on PLA; est)
    c_a: float = 2.0
    mu_b: float = 0.25                # bore friction when a roller slides along its axle
    E_tread: float = 26e6             # Pa TPU 95A (85A ~12 MPa)
    c_h: float = 3.0                  # s/m Hunt-Crossley damping, tread on floor
    mu: float = 0.9
    runout: float = 0.0               # m, 1-sigma roller radius error (print tolerance), random per roller
    seed: int = 0
    roller_density: float = 750.0     # kg/m3 effective (PLA core infill + TPU tread)


@dataclass(frozen=True)
class Run:
    wheel: Wheel = field(default_factory=Wheel)
    mode: str = "drive"               # "drive" or "hand"
    v: float = 1.0                    # m/s
    F_drag: float = 0.0               # N traction the wheel must give (drive)
    F_press: float = 15.0             # N hand load (hand)
    m_hub: float = 0.55               # kg unsprung, without rollers (hub, cartridge, 6804, AK40-10, fork share)
    m_sprung: float = 1.57            # kg robot quarter on the pod (9.63 / 4 - unsprung - rollers)
    k_pod: float = 2600.0
    zeta_pod: float = 0.3
    J_hub: float = 3.0e-4             # kg m2 hub without rollers
    J_motor: float = 9.7e-4           # reflected rotor inertia (9.735e-6 x 10^2)
    I_roll: float = 2.0e-3
    k_roll: float = 300.0             # N m/rad fork + pivot roll stiffness at the wheel (est)
    dt: float = 1e-6
    warmup: float = 0.25
    revs: float = 1.0                 # record this many wheel turns
    fs: float = 40000.0


def geometry(w: Wheel):
    n = 2 * w.n_row
    pitch = 2 * math.pi / n
    alpha_h = (pitch + math.radians(w.overlap_deg)) / 2
    H = w.R * math.sin(alpha_h)
    a = w.R - w.rr
    gam0 = np.array([(k // 2) * 2 * pitch + (k % 2) * pitch for k in range(n)])     # alternate rows round the wheel
    xr = np.array([-w.x_row if k % 2 == 0 else w.x_row for k in range(n)])
    u = np.linspace(-H, H, 201)
    rt = np.sqrt(w.R ** 2 - u ** 2) - a
    vol = np.trapezoid(np.pi * (rt ** 2 - 0.0017 ** 2), u)
    m_r = w.roller_density * vol
    rng = np.random.default_rng(w.seed)
    Ri = w.R + (rng.normal(0.0, w.runout, n) if w.runout > 0 else np.zeros(n))
    return dict(n=n, H=H, a=a, gam0=gam0, xr=xr, m_r=m_r, Ri=Ri, alpha_h=alpha_h)


def a_weight(f):
    f2 = np.asarray(f, float) ** 2
    ra = (12194.0 ** 2 * f2 ** 2) / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194.0 ** 2))
    return ra / 0.7943           # 0 dB at 1 kHz


def band_level(x, fs, lo=400.0, hi=10000.0, weight=True):
    """dB re 1 N^2 of x's power between lo and hi Hz (A-weighted)"""
    from scipy.signal import welch
    f, P = welch(x - np.mean(x), fs=fs, nperseg=min(len(x), 4096))
    m = (f >= lo) & (f <= hi)
    wt = a_weight(f[m]) ** 2 if weight else 1.0
    return 10 * math.log10(max(np.sum(P[m] * wt) * (f[1] - f[0]), 1e-30))


def simulate(run: Run):
    w = run.wheel
    g = geometry(w)
    n, a, H, m_r, Ri, xr = g["n"], g["a"], g["H"], g["m_r"], g["Ri"], g["xr"]
    gam0 = g["gam0"]
    rf = w.fillet
    Rp = Ri - rf
    alp = np.arcsin((H - rf) / Rp)
    ca, sa = np.cos(alp), np.sin(alp)
    Re = math.sqrt(w.R * w.rr)
    k_h = 4.0 / 3.0 * w.E_tread / (1 - 0.48 ** 2) * math.sqrt(Re)
    drive = run.mode == "drive"
    m_h = run.m_hub if drive else 0.25
    m_y = m_h + (run.m_sprung if drive else 0.0)
    J = run.J_hub + (run.J_motor if drive else 0.0)
    load = (m_h + n * m_r + (run.m_sprung if drive else 0.0)) * G + (0.0 if drive else run.F_press)
    # start: one roller of row 0 at the bottom, hub at the static sink, rollers centred on their axles
    theta = -math.pi / 2 - gam0[0]
    delta0 = (load / k_h) ** (2 / 3)
    y, z = 0.0, w.R - delta0 - w.g_r
    vy, vz = run.v, 0.0
    om = -run.v / w.R
    phi = phid = 0.0
    zs = z + 0.05
    vzs = 0.0
    L0 = 0.05 - run.m_sprung * G / run.k_pod
    c_pod = 2 * run.zeta_pod * math.sqrt(run.k_pod * run.m_sprung)
    c_roll = 2 * 0.1 * math.sqrt(run.k_roll * run.I_roll)
    Kp = J * 2 * math.pi * 30
    Ki = Kp * 2 * math.pi * 5
    integ = 0.0
    gam = gam0 + theta
    r0 = -np.sin(gam) * w.g_r * 0.95             # unloaded rollers hang on their axles; the bottom one is pushed in
    r0[0] = -w.g_r
    P = np.stack([y + (a + r0) * np.cos(gam), z + (a + r0) * np.sin(gam)], 1)
    V = np.stack([vy - om * a * np.sin(gam), vz + om * a * np.cos(gam)], 1)
    k_hand = 2e4
    c_hand = 2 * math.sqrt(k_hand * m_y)
    T = run.warmup + run.revs * 2 * math.pi * w.R / run.v
    nsteps = int(T / run.dt)
    every = max(1, int(round(1.0 / (run.fs * run.dt))))
    nrec = nsteps // every + 1
    rec = np.zeros((nrec, 7))           # t, F_hub y, F_hub z, floor N, phi, hub z, force into the chassis (pod)
    Fpod = 0.0
    ri = 0
    in_b = np.zeros(n, bool)
    in_a = np.zeros(n, bool)
    impacts_b, impacts_a = [], []
    dt = run.dt
    vb_eps, v_eps = 2e-3, 1e-2
    t0_rec = run.warmup
    for k in range(nsteps):
        t = k * dt
        gam = gam0 + theta
        cg, sg = np.cos(gam), np.sin(gam)
        Ay, Az = y + a * cg, z + a * sg
        oy, oz = P[:, 0] - Ay, P[:, 1] - Az
        r = oy * cg + oz * sg
        tt = -oy * sg + oz * cg
        VAy, VAz = vy - om * a * sg, vz + om * a * cg
        dvy = V[:, 0] - VAy + om * oz
        dvz = V[:, 1] - VAz - om * oy
        rd = dvy * cg + dvz * sg
        td = -dvy * sg + dvz * cg
        # bore (radial) and thrust (axial) contacts: force on the roller along e_r / e_t
        do, di = r - w.g_r, -w.g_r - r
        fr = np.where(do > 0, -np.maximum(w.k_b * do * (1 + w.c_b * rd), 0.0), 0.0)
        fr = np.where(di > 0, np.maximum(w.k_b * di * (1 - w.c_b * rd), 0.0), fr)
        ao, ai = tt - w.g_a, -w.g_a - tt
        fa = np.where(ao > 0, -np.maximum(w.k_a * ao * (1 + w.c_a * td), 0.0), 0.0)
        fa = np.where(ai > 0, np.maximum(w.k_a * ai * (1 - w.c_a * td), 0.0), fa)
        # contact onsets: approach speed
        nb, na = (do > 0) | (di > 0), (ao > 0) | (ai > 0)
        if t >= t0_rec:
            new = nb & ~in_b
            if new.any():
                impacts_b.extend(np.abs(rd[new]).tolist())
            new = na & ~in_a
            if new.any():
                impacts_a.extend(np.abs(td[new]).tolist())
        in_b, in_a = nb, na
        fa = fa - w.mu_b * np.abs(fr) * np.tanh(td / vb_eps)
        # floor: lowest point of each roller's silhouette (direction -z)
        Oy, Oz = P[:, 0] - a * cg, P[:, 1] - a * sg
        ang = np.arctan2(-cg, -sg)                    # angle of -z from e_r toward e_t
        sgn = np.sign(ang)
        in_arc = np.abs(ang) <= alp
        cy = Oy + Rp * (ca * cg - sgn * sa * sg)
        cz = Oz + Rp * (ca * sg + sgn * sa * cg)
        ly = np.where(in_arc, Oy, cy)
        lz = np.where(in_arc, Oz - Ri, cz - rf)
        lz_eff = lz + xr * phi
        dpen = -lz_eff
        down = np.cos(ang) > 0
        c_on = (dpen > 0) & down
        N = np.zeros(n)
        Ff = np.zeros(n)
        if c_on.any():
            vcy = V[:, 0] - om * (lz - P[:, 1])
            vcz = V[:, 1] + om * (ly - P[:, 0]) + xr * phid
            dp = np.where(c_on, dpen, 0.0)
            N = np.where(c_on, np.maximum(k_h * dp ** 1.5 * (1 - w.c_h * vcz), 0.0), 0.0)
            Ff = -w.mu * N * np.tanh(vcy / v_eps)
        # roller forces
        Fy = Ff + fr * cg - fa * sg
        Fz = N - m_r * G + fr * sg + fa * cg
        # on the hub: reaction of the axle forces at the axle point, plus the floor force's moment about the roller
        Hy = -(fr * cg - fa * sg)
        Hz = -(fr * sg + fa * cg)
        tq = np.sum(a * (cg * Hz - sg * Hy)) + np.sum((ly - P[:, 0]) * N - (lz - P[:, 1]) * Ff)
        sHy, sHz = Hy.sum(), Hz.sum()
        mroll = np.sum(xr * Hz)
        # hub
        if drive:
            e = (-run.v / w.R) - om
            integ += e * dt
            tau = max(-4.0, min(4.0, Kp * e + Ki * integ))
            Fpod = run.k_pod * (zs - z - L0) + c_pod * (vzs - vz)
            ay = (sHy - run.F_drag) / m_y
            az = (sHz - m_h * G + Fpod) / m_h
            azs = (-Fpod - run.m_sprung * G) / run.m_sprung
            vzs += azs * dt
            zs += vzs * dt
        else:
            tau = -0.005 * math.tanh(om / 0.05)          # the two 6804s
            ay = (sHy + k_hand * (run.v * t - y) + c_hand * (run.v - vy)) / m_y
            az = (sHz - m_h * G - run.F_press) / m_h
        al = (tau + tq) / J
        phidd = (mroll - run.k_roll * phi - c_roll * phid) / run.I_roll
        vy += ay * dt
        vz += az * dt
        om += al * dt
        phid += phidd * dt
        y += vy * dt
        z += vz * dt
        theta += om * dt
        phi += phid * dt
        V[:, 0] += Fy / m_r * dt
        V[:, 1] += Fz / m_r * dt
        P += V * dt
        if k % every == 0:
            rec[ri] = (t, sHy, sHz, N.sum(), phi, z, Fpod)
            ri += 1
    rec = rec[:ri]
    return rec, np.array(impacts_b), np.array(impacts_a), g


def metrics(run: Run):
    t0 = time.time()
    rec, ib, ia, g = simulate(run)
    keep = rec[:, 0] >= run.warmup
    r = rec[keep]
    fs = run.fs
    Fhub = np.hypot(r[:, 1] - r[:, 1].mean(), r[:, 2] - r[:, 2].mean())
    turn = run.revs
    big = 0.005
    out = dict(
        hub_dB=10 * math.log10(10 ** (band_level(r[:, 1], fs) / 10) + 10 ** (band_level(r[:, 2], fs) / 10)),
        floor_dB=band_level(r[:, 3], fs),
        chassis_dB=band_level(r[:, 6], fs) if run.mode == "drive" else float("nan"),   # vertical force through the pod
        # the pod swings vertically only: horizontal hub forces reach the chassis through the pivot, the chassis's share
        # of them set by the masses (hub and chassis move together along y)
        chassis_y_dB=(band_level(r[:, 1] * run.m_sprung / (run.m_hub + run.m_sprung), fs) if run.mode == "drive" else float("nan")),
        hub_lo_rms=float(np.std(r[:, 2])),            # N, wheel-load ripple through the hub, all frequencies
        floor_N_pp=float(np.ptp(r[:, 3])),
        roll_pp_mrad=float(np.ptp(r[:, 4]) * 1e3),
        hub_z_pp_um=float(np.ptp(r[:, 5]) * 1e6),
        bore_hits_per_turn=float(np.sum(ib > big) / turn),
        bore_v_max=float(ib.max()) if len(ib) else 0.0,
        bore_v_p90=float(np.percentile(ib[ib > big], 90)) if np.any(ib > big) else 0.0,
        thrust_hits_per_turn=float(np.sum(ia > big) / turn),
        thrust_v_max=float(ia.max()) if len(ia) else 0.0,
        m_roller_g=g["m_r"] * 1e3,
        H_mm=g["H"] * 1e3,
        wall_s=time.time() - t0,
    )
    return out


# ---- variants --------------------------------------------------------------------------------------------------------------
B = Wheel()
WHEELS = {
    "baseline 2x6 (v13)": B,
    "bore 3.2 (play 0.13)": replace(B, g_r=0.13e-3),
    "bore 3.05 / smooth pin (0.05)": replace(B, g_r=0.05e-3),
    "623 bearings (0.01, axial 0.1)": replace(B, g_r=0.01e-3, g_a=0.10e-3, mu_b=0.05),
    "623 + axial preload (0, 0)": replace(B, g_r=0.005e-3, g_a=0.0, mu_b=0.05, k_a=1e6),
    "axial play 0.1 only": replace(B, g_a=0.10e-3),
    "TPU 85A tread": replace(B, E_tread=12e6, c_h=4.0),
    "2x8 rollers": replace(B, n_row=8, name="2x8"),
    "2x8 + 623": replace(B, n_row=8, g_r=0.01e-3, g_a=0.10e-3, mu_b=0.05),
    "2x5 rollers": replace(B, n_row=5),
    "overlap 6 deg": replace(B, overlap_deg=6.0),
    "fillet 0.5": replace(B, fillet=0.0005),
    "runout 0.1 mm": replace(B, runout=0.1e-3, seed=1),
    "623 + runout 0.1": replace(B, g_r=0.01e-3, g_a=0.10e-3, mu_b=0.05, runout=0.1e-3, seed=1),
    "623 + 85A": replace(B, g_r=0.01e-3, g_a=0.10e-3, mu_b=0.05, E_tread=12e6, c_h=4.0),
    # Fernando (Nov 11): no bearings, the original roller with tighter clearances (plain PLA bore on the axle)
    "plain: bore 0.05 + axial 0.1": replace(B, g_r=0.05e-3, g_a=0.10e-3, mu_b=0.3),
    "plain: bore 0.05 + axial 0.05": replace(B, g_r=0.05e-3, g_a=0.05e-3, mu_b=0.3),
    "plain: bore 0.02 + axial shimmed (0)": replace(B, g_r=0.02e-3, g_a=0.0, mu_b=0.3, k_a=1e6),
    "plain: bore 0.13 + axial shimmed (0)": replace(B, g_r=0.13e-3, g_a=0.0, mu_b=0.3, k_a=1e6),
}
CASES = {
    "hand 0.3 m/s": dict(mode="hand", v=0.3, F_press=15.0),
    "drive 1 m/s": dict(mode="drive", v=1.0),
    "drive 1 m/s, 8 N traction": dict(mode="drive", v=1.0, F_drag=8.0),
}

# Fernando (Nov 11): does the suspension, or having four wheels, help or hurt? Same wheel at 1 m/s on different mounts,
# and the per-wheel load of a 3-wheel (kiwi) base against the 4-wheel X (9.63 kg robot)
SUSP = {
    "pod 2.6 N/mm, damped (v13 sim)": dict(),
    "pod springs only (no damper)": dict(zeta_pod=0.02),
    "stiff rubber mount 200 N/mm": dict(k_pod=2e5, zeta_pod=0.1),
    "rigid mount 2000 N/mm": dict(k_pod=2e6, zeta_pod=0.05),
    "pod, 3-wheel load": dict(m_sprung=9.63 / 3 - 0.55 - 0.32),
    "rigid, 3-wheel load": dict(m_sprung=9.63 / 3 - 0.55 - 0.32, k_pod=2e6, zeta_pod=0.05),
}


def job(args):
    wname, cname, kw = args
    run = Run(wheel=WHEELS[wname], **kw)
    try:
        return wname, cname, metrics(run)
    except Exception as ex:          # noqa: BLE001
        return wname, cname, dict(error=repr(ex))


def main():
    which = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "all"
    nj = int(sys.argv[sys.argv.index("-j") + 1]) if "-j" in sys.argv else 6
    OUT.mkdir(parents=True, exist_ok=True)
    if which == "plain":
        jobs = [(w, c, CASES[c]) for w in WHEELS if w.startswith(("plain", "baseline", "623 + axial")) for c in ("hand 0.3 m/s", "drive 1 m/s")]
    elif which == "susp":
        jobs = [(w, c, dict(CASES["drive 1 m/s"], **kw)) for w in ("baseline 2x6 (v13)", "623 bearings (0.01, axial 0.1)")
                for c, kw in SUSP.items()]
    elif which == "quick":
        jobs = [(w, c, dict(CASES[c], revs=0.35)) for w in ("baseline 2x6 (v13)", "623 bearings (0.01, axial 0.1)") for c in ("drive 1 m/s",)]
    else:
        jobs = [(w, c, CASES[c]) for w in WHEELS for c in CASES]
    res = {}
    with Pool(nj) as pool:
        for wname, cname, m in pool.imap_unordered(job, jobs):
            res.setdefault(wname, {})[cname] = m
            print(f"{wname:34s} {cname:28s} " + (f"hub {m['hub_dB']:6.1f} dB  floor {m['floor_dB']:6.1f} dB  chassis z/y "
                  f"{m.get('chassis_dB', float('nan')):6.1f}/{m.get('chassis_y_dB', float('nan')):6.1f}  bore hits/turn "
                  f"{m['bore_hits_per_turn']:4.0f} v90 {m['bore_v_p90'] * 1e3:5.1f} mm/s  thrust hits {m['thrust_hits_per_turn']:4.0f} "
                  f"vmax {m['thrust_v_max'] * 1e3:5.1f}  roll {m['roll_pp_mrad']:.2f} mrad  ({m['wall_s']:.0f} s)"
                  if "error" not in m else m["error"]), flush=True)
    (OUT / f"{which}.json").write_text(json.dumps(dict(wheels={k: asdict(v) for k, v in WHEELS.items()}, cases=CASES, results=res), indent=1))


if __name__ == "__main__":
    main()
