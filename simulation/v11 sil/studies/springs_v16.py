"""cad/v16's pod springs (Nov 21, Fernando): what the drawn springs give at the wheel, and which arrangement to use.

Static part (no sim): cad/v16's fork and tower geometry (cad/v16/tools/dims.py, the lever maths of pod.py re-done here
so build123d is not imported) with the 7.85 x 4.25 x 20.2 springs, solid at 12 (measured). Their rate: a JIS B 5012
yellow (extra light) 8 x 20 die spring, which they match (OD, ID, free length, a ~41 % stroke to solid), is listed at
7.6 N/mm (Century Spring DJ-8X20YCS 7.62, Raymond ASF 8x20 0.78 kgf/mm, Dingli DTF8-20 7.8). ESTIMATE until the
kitchen-scale test. For each arrangement: the wheel force over the travel (virtual work through each plunger's lift),
where each spring goes solid, the sag at a robot mass.

Masses (Fernando, Nov 21): three deck quarters with their pods (wheel and motor in: 1 kg is what the CAD's volumes give
with them, ~0.5 kg without) weigh 3 kg, so 1.0 kg per quarter. Unsprung per corner (wheel, motor, fork, plungers) is
estimated from the CAD's volumes at the density that matches that weighing: 0.675 kg (the sim had 0.92).

Sim part: drive_char.py's suspension scenarios (the Nov 1 part P) on the arm robot at 8 kg (a bottom-up v16 estimate)
and 9.63 kg (the sim's CAD estimate), for the old sim pod (3 N/mm, zeta 0.6) and the v16 arrangements. The drawn pod
has no damper: only the plungers' friction (zeta ~0.1, est).

    python studies/springs_v16.py static        # tables, seconds
    python studies/springs_v16.py sim [--workers N]   # ~200 jobs in parallel, results/springs_v16/sim.jsonl
    python studies/springs_v16.py report
"""
from __future__ import annotations

import importlib.util
import json
import math
import multiprocessing as mp
import sys
import time
import traceback
from dataclasses import asdict, dataclass, replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

REPO = HERE.parents[2]
_spec = importlib.util.spec_from_file_location("v16_dims", REPO / "cad" / "v16" / "tools" / "dims.py")
DIMS = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(DIMS)

OUT = HERE / "results" / "springs_v16"
OUT.mkdir(parents=True, exist_ok=True)

K_SPRING = 7.6           # N/mm, JIS yellow 8 x 20 (est, see above)
L0, SOLID = DIMS.SPRING_L0, DIMS.SPRING_SOLID
G = 9.81
QUARTER_KG = 1.0         # measured: deck piece + pod (wheel, motor) per quarter
UNSPRUNG_KG = 0.675      # est per corner: wheel 0.37 (printed 0.28 + 24 x 687 + 6804 + screws), AK40-10 0.19, fork 0.065, plungers + 627s 0.05
MASSES = {"base only (now, 4 kg)": 4.0, "full, bottom-up est (8 kg)": 8.0, "full, sim CAD est (9.63 kg)": 9.63}


# ---- the fork's geometry (cad/v16 pod.py, re-done) ---------------------------------------------------------------------------
PY, PZ = DIMS.PIVOT_Y, DIMS.PIVOT_Z
PAD_Z = DIMS.PAD_Z
Y_S = PY - DIMS.SPRING_ARM
Y_FAR, Y_NEAR = Y_S - DIMS.SPRING_PITCH / 2, Y_S + DIMS.SPRING_PITCH / 2     # far from the pivot (toward the axle), near it
R_DOME = DIMS.PLUNGER_DOME_R
Z_SEAT = PAD_Z + DIMS.PLUNGER_SEAT_OVER_PAD


def rot_pt(p, a):
    y, z = p[0] - PY, p[1] - PZ
    c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
    return (PY + y * c - z * s, PZ + y * s + z * c)


def angle_for(dz):
    lo, hi = -40.0, 40.0
    for _ in range(80):
        m = (lo + hi) / 2
        if rot_pt((0.0, 0.0), m)[1] > dz:
            lo = m
        else:
            hi = m
    return (lo + hi) / 2


def lift(dz, y):
    """plunger on axis y: how far its dome rises when the axle is dz above nominal (pod.plunger_lift)"""
    a = angle_for(dz)
    p0 = rot_pt((y, PAD_Z), a)
    p1 = rot_pt((y, PAD_Z + 1.0), a)
    n = (p1[0] - p0[0], p1[1] - p0[1])
    zc = p0[1] + (R_DOME - n[0] * (y - p0[0])) / n[1]
    return zc - (PAD_Z + R_DOME)


@dataclass(frozen=True)
class Arr:
    name: str
    bores: tuple = ("far", "near")    # which bores of each tower hold springs (two towers per pod)
    series: int = 1                   # springs stacked in each used bore
    droop: float = 3.0                # mm below nominal (the droop stop)
    bump: float = 15.0                # mm above nominal (the towers' underside)
    pre: float = 0.1                  # mm the most-drooping used stack is compressed at full droop (sets the ceiling)


ARRS = {
    "as drawn: 2 per tower": Arr("as drawn: 2 per tower"),
    "1 per tower, far bore": Arr("1 per tower, far bore", bores=("far",)),
    "1 per tower, near bore": Arr("1 per tower, near bore", bores=("near",)),
    "2 per tower, bump 17": Arr("2 per tower, bump 17", bump=17.0),
    "2 in series x 2 per tower": Arr("2 in series x 2 per tower", series=2),
    "2 in series x 2, droop 6": Arr("2 in series x 2, droop 6", series=2, droop=6.0),
    "2 in series x 2, droop 6, bump 17": Arr("2 in series x 2, droop 6, bump 17", series=2, droop=6.0, bump=17.0),
    "2 in series, far bore": Arr("2 in series, far bore", bores=("far",), series=2),
}


def curve(arr: Arr, step=0.25):
    """wheel force (N, one wheel) over its travel, each stack's length, and the first z where a stack is solid"""
    ys = {"far": Y_FAR, "near": Y_NEAR}
    used = [ys[b] for b in arr.bores]
    ceil = Z_SEAT + min(lift(-arr.droop, y) for y in used) + arr.series * L0 - arr.pre
    k = K_SPRING / arr.series
    rows, solid_at = [], None
    z = -arr.droop
    while z <= arr.bump + 1e-9:
        f, lens = 0.0, {}
        for b in arr.bores:
            y = ys[b]
            L = ceil - (Z_SEAT + lift(z, y))
            c = max(0.0, arr.series * L0 - L)
            ratio = (lift(z + 0.01, y) - lift(z - 0.01, y)) / 0.02       # plunger mm per wheel mm
            f += 2 * k * c * ratio                                          # two towers
            lens[b] = L
            if L <= arr.series * SOLID + 1e-6 and solid_at is None:
                solid_at = z
        rows.append(dict(z=round(z, 2), F=f, lens=lens))
        z += step
    # beyond the bump stop: where would the first stack bind if the stop were moved
    zz = arr.bump
    while solid_at is None and zz < 40:
        zz += 0.1
        for b in arr.bores:
            if ceil - (Z_SEAT + lift(zz, ys[b])) <= arr.series * SOLID:
                solid_at = zz
                break
    return rows, solid_at, ceil


def summarize(arr: Arr):
    rows, solid_at, ceil = curve(arr)
    F0, F1 = rows[0]["F"], rows[-1]["F"]
    rate = (F1 - F0) / (arr.bump + arr.droop)
    out = dict(arr=asdict(arr), preload_N=round(F0, 2), force_at_bump_N=round(F1, 1), wheel_rate_N_per_mm=round(rate, 3),
               first_solid_at_mm=round(solid_at, 2), solid_before_bump=solid_at <= arr.bump,
               ceiling_over_pod_z=round(ceil - (Z_SEAT + lift(0, Y_FAR) + L0), 2),
               lengths_at_bump={b: round(v, 2) for b, v in rows[-1]["lens"].items()}, sag={})
    for lab, kg in MASSES.items():
        w = (kg - 4 * UNSPRUNG_KG) * G / 4
        if w <= F0:
            z_eq = -arr.droop
        elif w >= F1:
            z_eq = arr.bump
        else:
            z_eq = next(r0["z"] + (w - r0["F"]) / (r1["F"] - r0["F"]) * (r1["z"] - r0["z"])
                        for r0, r1 in zip(rows, rows[1:]) if r0["F"] <= w <= r1["F"])
        out["sag"][lab] = dict(wheel_load_N=round(w, 1), ride_z_mm=round(z_eq, 2), from_droop_mm=round(z_eq + arr.droop, 2),
                               left_to_bump_mm=round(arr.bump - z_eq, 2),
                               g_to_bump=round(F1 / w, 2) if w > 0 else None)
    return out


def static():
    res = {n: summarize(a) for n, a in ARRS.items()}
    print(f"spring {K_SPRING} N/mm (est), L0 {L0}, solid {SOLID}; levers: far {PY - Y_FAR:.1f} mm, near {PY - Y_NEAR:.1f} mm of {PY} "
          f"(ratio {(PY - Y_FAR) / PY:.3f} / {(PY - Y_NEAR) / PY:.3f})")
    print(f"{'arrangement':46s} {'rate':>6s} {'pre N':>6s} {'F bump':>7s} {'solid at':>8s} | ride z (mm, + up) / left to bump at 4 / 8 / 9.63 kg")
    for n, r in res.items():
        s = " ".join(f"{v['ride_z_mm']:6.1f}/{v['left_to_bump_mm']:5.1f}" for v in r["sag"].values())
        print(f"{n:46s} {r['wheel_rate_N_per_mm']:6.2f} {r['preload_N']:6.1f} {r['force_at_bump_N']:7.1f} {r['first_solid_at_mm']:8.2f} | {s}")
    (OUT / "static.json").write_text(json.dumps(res, indent=1))
    return res


# ---- sim ------------------------------------------------------------------------------------------------------------------------
SIM_ARRS = ("as drawn: 2 per tower", "1 per tower, far bore", "2 in series x 2, droop 6", "2 in series x 2, droop 6, bump 17")
SIM_MASSES = {"8": 8.0, "9.63": 9.63}
ZETA_NO_DAMPER = 0.1


def linear_pod(name: str, kg: float, fit: str = "sag") -> dict:
    """the sim's pod is a linear spring with a preload at its droop stop: keep the real preload and take the rate as the
    secant to the ride height at this mass (the sag right). The real springs stiffen toward bump (the pads sit 24 mm over
    the pivot's line and the springs compress more), so the sim's force near bump is lower than the CAD's: it bottoms
    sooner than the real pod would (conservative)."""
    a = ARRS[name]
    s = summarize(a)
    rows, solid_at, _ = curve(a)
    w = (kg - 4 * UNSPRUNG_KG) * G / 4
    pre = max(s["preload_N"], 0.2)
    z_eq = next((r0["z"] + (w - r0["F"]) / (r1["F"] - r0["F"]) * (r1["z"] - r0["z"])
                 for r0, r1 in zip(rows, rows[1:]) if r0["F"] <= w <= r1["F"]), None)
    rate = (w - pre) / (z_eq + a.droop) if z_eq is not None and z_eq + a.droop > 0.5 else s["wheel_rate_N_per_mm"]
    if fit == "bump":           # the other bracket: the CAD's force at bump (stiffer: rides lower, bottoms less)
        rate = s["wheel_rate_N_per_mm"]
    return dict(pod_rate=round(rate * 1000.0, 1), pod_preload=round(pre, 2), pod_droop=a.droop / 1000.0,
                pod_travel=(a.droop + min(a.bump, solid_at)) / 1000.0)


def sim_settings():
    """(label, arrangement or None, zeta): the old sim pod, then the v16 arrangements (linear_pod, per mass)"""
    out = [("old sim pod: 3 N/mm, 2 N, zeta 0.6, -3/+15", None, 0.6)]
    for n in SIM_ARRS:
        out.append((f"{n}, no damper", n, ZETA_NO_DAMPER))
        if n == "as drawn: 2 per tower":
            out.append((f"{n}, zeta 0.6 (a damper)", n, 0.6))
            out.append((f"{n}, no damper, rate to the bump force", n + "|bump", ZETA_NO_DAMPER))
    return out


def pod_for(arr, zeta, kg):
    if arr is None:
        return dict(pod_rate=3000.0, pod_preload=2.0, pod_damping_ratio=zeta, pod_droop=0.003, pod_travel=0.018)
    name, _, fit = arr.partition("|")
    return dict(linear_pod(name, kg, fit or "sag"), pod_damping_ratio=zeta)


def _patch():
    import drive_char as D
    for k, kg in SIM_MASSES.items():
        D.MASSES[k] = (kg, 0.0)
    if getattr(D, "_springs_v16", False):
        return D
    orig = D.params

    def params(cfg, features=(), accel=3.0, jerk=0.0, **kw):
        p = orig(cfg, features, accel, jerk, **kw)
        return p.with_(**POD_NOW) if POD_NOW else p
    D.params = params
    D._springs_v16 = True
    return D


POD_NOW: dict = {}


def run_one(job):
    global POD_NOW
    label, pod, mass, scen = job
    D = _patch()
    POD_NOW = dict(pod)
    cfg = D.Cfg(motor="AK40-10", mass=mass, springs="fixed")
    t0 = time.perf_counter()
    try:
        if scen == "settle":
            m, d, r = D.rig(cfg, settle=1.5)
            import numpy as np
            p = D.params(cfg)
            from robot_model import fork_kinematics
            zs = []
            for i in range(1, 5):
                q = float(d.qpos[m.joint(f"susp{i}").qposadr[0]])
                rho = math.hypot(*p.pod_pivot)
                f = fork_kinematics(p, i)
                b0 = math.asin((p.pod_pivot[1] + p.pod_droop) / rho)
                zs.append(1000 * (rho * (math.sin(b0) - math.sin(b0 - q)) - p.pod_droop))
            res = dict(ride_z_mm=[round(z, 2) for z in zs], loads=[round(float(x), 1) for x in r.wheel_loads()])
        else:
            res = D.job_susp(cfg, scen)
        err = None
    except Exception:
        res, err = None, traceback.format_exc()
    return dict(setting=label, pod=pod, mass=mass, scenario=scen, res=res, err=err, wall_s=time.perf_counter() - t0)


def sim(workers=24):
    import drive_char as D
    jobs = [(lab, pod_for(arr, z, SIM_MASSES[m]), m, sc) for lab, arr, z in sim_settings() for m in SIM_MASSES
            for sc in ("settle",) + D.SUSP_SCENARIOS]
    for lab, arr, z in sim_settings():
        print(lab, {m: pod_for(arr, z, kg) for m, kg in SIM_MASSES.items()})
    path = OUT / "sim.jsonl"
    done = set()
    if path.exists():
        for line in path.read_text().splitlines():
            j = json.loads(line)
            if j["err"] is None:
                done.add((j["setting"], j["mass"], j["scenario"]))
    jobs = [j for j in jobs if (j[0], j[2], j[3]) not in done]
    print(f"{len(jobs)} jobs, {workers} workers", flush=True)
    t0 = time.time()
    with mp.get_context("spawn").Pool(workers) as pool, path.open("a") as fh:
        for n, r in enumerate(pool.imap_unordered(run_one, jobs), 1):
            fh.write(json.dumps(r, default=float) + "\n")
            fh.flush()
            if n % 10 == 0 or n == len(jobs):
                print(f"{n}/{len(jobs)} {time.time() - t0:.0f} s", flush=True)


def report():
    rows = [json.loads(x) for x in (OUT / "sim.jsonl").read_text().splitlines()]
    by = {}
    for r in rows:
        by.setdefault((r["setting"], r["mass"]), {})[r["scenario"]] = r
    lines = ["| setting | kg | ride z mm (+ up) | passed | max tilt deg | longest wheel lift ms | on bump stop ms | start 0 / 90, brake m/s^2 | failed |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for (st, kg), sc in by.items():
        settle = sc.get("settle", {}).get("res") or {}
        ride = settle.get("ride_z_mm")
        ok = [n for n, r in sc.items() if n != "settle" and r["res"] and r["res"].get("ok") is not None]
        passed = [n for n in ok if sc[n]["res"]["ok"]]
        failed = [n for n in ok if not sc[n]["res"]["ok"]]
        errs = [n for n, r in sc.items() if r["err"]]
        res = [r["res"] for n, r in sc.items() if n != "settle" and r["res"]]
        tilt = max((x.get("tilt_max", 0) for x in res), default=0)
        lift = max((x.get("lift_ms", 0) for x in res), default=0)
        bottom = sum(x.get("bottom_ms", 0) for x in res)
        acc = "/".join(f"{sc[s]['res']['a_max']:.1f}" if sc.get(s) and sc[s]["res"] else "-"
                       for s in ("accel start 0", "accel start 90", "accel brake 0"))
        rz = f"{min(ride):.1f}..{max(ride):.1f}" if ride else "-"
        lines.append(f"| {st} | {kg} | {rz} | {len(passed)}/{len(ok)} | {tilt:.1f} | {lift:.0f} | {bottom:.0f} | {acc} | "
                     f"{', '.join(failed + [e + ' (error)' for e in errs]) or '-'} |")
    txt = "\n".join(lines)
    print(txt)
    (OUT / "sim_table.md").write_text(txt + "\n")
    # per-scenario detail
    det = ["| setting | kg | scenario | ok | tilt | lift ms | bottom ms |", "| --- | --- | --- | --- | --- | --- | --- |"]
    for (st, kg), sc in by.items():
        for n, r in sc.items():
            x = r["res"] or {}
            if n == "settle" or "tilt_max" not in x:
                continue
            det.append(f"| {st} | {kg} | {n} | {x.get('ok')} | {x.get('tilt_max', 0):.1f} | {x.get('lift_ms', 0):.0f} | {x.get('bottom_ms', 0):.0f} |")
    (OUT / "sim_detail.md").write_text("\n".join(det) + "\n")


if __name__ == "__main__":
    a = sys.argv[1:] or ["static"]
    if a[0] == "static":
        static()
    elif a[0] == "sim":
        w = int(a[a.index("--workers") + 1]) if "--workers" in a else 24
        sim(w)
    elif a[0] == "report":
        report()
