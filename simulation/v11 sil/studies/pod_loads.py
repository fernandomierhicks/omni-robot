"""Wheel loads for the pod strength check (Nov 12 2026, Fernando: "I have a feeling those PLA parts will break").

    python -B studies/pod_loads.py [run|report] [-j N]          (omni-robot-sim interpreter, from simulation/v11 sil)

The arm robot as baselined (robot_export.arm_d435_params: CAD mass and CG, the pods, the AK40-10s) on explicit rollers
(cad/v13's bearing-roller wheel), driven straight across door thresholds and rug edges at 0.5 / 1 / 2 m/s, head-on (0 deg)
and at 45 deg, plus a hard start and stop on the flat. Per wheel, at every step, the force the floor and the obstacle put on
its rollers (summed contact forces, world frame): the vertical and the horizontal part, the drive motor's torque and the
pod's travel (bump stop at pod_travel - pod_droop above nominal). Each wheel's peak is kept, with the other components at
that instant. These are the load cases cad/v13/tools/pod_fe_prep.py turns into forces on the fork.

Rigid-body contacts (MuJoCo soft constraints, the TPU's stiffness as solref) set the impact peaks: read them as the right
order, not to better than ~30 %. Results: results/pod_loads.json.
"""
from __future__ import annotations

import json
import math
import sys
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
OUT = HERE / "results" / "pod_loads.json"

SPEEDS = (0.5, 1.0, 2.0)
DIRS = (0.0, 45.0)


def obstacles():
    from floor import Carpet, Threshold
    return {"flat": None,
            "threshold 12 mm": Threshold(0.6, 0.0, height=0.012),
            "threshold 18 mm": Threshold(0.6, 0.0, height=0.018),
            "threshold 25 mm": Threshold(0.6, 0.0, height=0.025, top=0.04, ramp=0.025),
            "rug edge 12 mm (square)": Carpet(0.6, -1.5, 2.6, 1.5, thickness=0.012)}


def job(args):
    name, direction, v = args
    from common import make_robot
    from omni_control import wrap_angle
    from robot_export import arm_d435_params
    obst = obstacles()[name]
    p = arm_d435_params(wheel_model="rollers", floor_features=(obst,) if obst is not None else ())
    m, d, r = make_robot(p, settle=0.6)
    yaw = -math.radians(direction)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(m, d)
    for _ in range(int(0.4 / m.opt.timestep)):
        r.step()
    nw = p.n_wheels
    owner = {}
    for i in range(1, nw + 1):
        for k in range(2 * p.omni.rollers_per_row):
            owner[m.geom(f"roller{i}_{k}").id] = i - 1
    susp_adr = [m.joint(f"susp{i}").qposadr[0] for i in range(1, nw + 1)]
    # the pod frame (cad/v13's): x along the axle toward the motor (the sim's hub x points inboard, at the motor), y from
    # the axle toward the pivot, z up the pod. The carrier body sits on the pivot, the hub at (0, ay, az) in it
    carrier = [m.body(f"carrier{i}").id for i in range(1, nw + 1)]
    hub_in_c = [np.array(m.body_pos[m.body(f"hub{i}").id]) for i in range(1, nw + 1)]
    q0 = np.array([d.qpos[a] for a in susp_adr])
    a_wheel = r.a_wheel
    dt = m.opt.timestep
    f6 = np.zeros(6)
    arm = math.hypot(*p.pod_pivot)                           # the pod hinge's arm: susp (rad) x arm = axle travel (m)
    nav = max(1, int(round(0.005 / dt)))                     # 5 ms average: one solver step's spike is not the pulse
    hist = np.zeros((nav, nw, 3))
    peak = [dict(Fz=0.0, Fh=0.0, Fz_5ms=0.0, Fh_5ms=0.0, torque=0.0, susp_max=0.0, susp_min=0.0) for _ in range(nw)]
    n = 0
    hist_pod = np.zeros((nav, nw, 3))
    pod_peak = [dict(F=0.0, at=None, max_xyz=[0.0] * 3, min_xyz=[0.0] * 3) for _ in range(nw)]

    def pod_frame(i):
        Rc = d.xmat[carrier[i]].reshape(3, 3)              # carrier -> world
        ex = Rc[:, 0]
        to_piv = Rc @ (-hub_in_c[i])
        ey = to_piv - ex * (to_piv @ ex)
        ey /= np.linalg.norm(ey)
        ez = np.cross(ex, ey)
        if ez[2] < 0:
            ez = -ez
        return np.vstack([ex, ey, ez])                    # rows: pod axes in world
    at_fz = [None] * nw
    at_fh = [None] * nw
    t0 = d.time
    flat = obst is None
    while d.time - t0 < (3.0 if flat else 1.25 / v * 2.5 + 1.0):
        pose = r.pose()
        if flat:                                             # hard start, cruise, hard stop
            tt = d.time - t0
            vv = v if tt < 1.6 else 0.0
            a_cap = 6.0
        else:
            vv, a_cap = v, 2.0
        w = float(np.clip(4.0 * wrap_angle(yaw - pose[2]), -2.0, 2.0))
        r.drive_velocity((vv, 0.0), w, pose[2], a_cap=a_cap)
        r.step()
        F = np.zeros((nw, 3))
        for c in range(d.ncon):
            con = d.contact[c]
            i = owner.get(con.geom1, owner.get(con.geom2))
            if i is None:
                continue
            mujoco.mj_contactForce(m, d, c, f6)
            fw = np.asarray(con.frame).reshape(3, 3).T @ f6[:3]          # on geom2, world frame
            F[i] += fw if con.geom2 in owner else -fw                     # force ON the roller
        tq = np.abs(d.actuator_force[a_wheel])
        s = (np.array([d.qpos[a] for a in susp_adr]) - q0) * arm
        hist[n % nav] = F
        hist_pod[n % nav] = np.array([pod_frame(i) @ F[i] for i in range(nw)])
        n += 1
        Fa = hist.mean(axis=0) if n >= nav else F
        Fp = hist_pod.mean(axis=0) if n >= nav else hist_pod[(n - 1) % nav]
        for i in range(nw):
            q = pod_peak[i]
            mag = float(np.linalg.norm(Fp[i]))
            if mag > q["F"]:
                q["F"], q["at"] = mag, dict(t=round(d.time - t0, 4), F_pod=[round(float(v), 1) for v in Fp[i]],
                                            torque=round(float(np.abs(d.actuator_force[a_wheel])[i]), 3))
            q["max_xyz"] = [max(a, float(b)) for a, b in zip(q["max_xyz"], Fp[i])]
            q["min_xyz"] = [min(a, float(b)) for a, b in zip(q["min_xyz"], Fp[i])]
        for i in range(nw):
            fz, fh = float(F[i, 2]), float(math.hypot(F[i, 0], F[i, 1]))
            pk = peak[i]
            snap = dict(t=round(d.time - t0, 4), Fz=round(fz, 1), Fh=round(fh, 1), Fx_world=round(float(F[i, 0]), 1),
                        torque=round(float(tq[i]), 3), travel_mm=round(1000 * float(s[i]), 2))
            if fz > pk["Fz"]:
                pk["Fz"], at_fz[i] = fz, snap
            if fh > pk["Fh"]:
                pk["Fh"], at_fh[i] = fh, snap
            pk["Fz_5ms"] = max(pk["Fz_5ms"], float(Fa[i, 2]))
            pk["Fh_5ms"] = max(pk["Fh_5ms"], float(math.hypot(Fa[i, 0], Fa[i, 1])))
            pk["torque"] = max(pk["torque"], float(tq[i]))
            pk["susp_max"] = max(pk["susp_max"], float(s[i]))
            pk["susp_min"] = min(pk["susp_min"], float(s[i]))
        if not flat and r.pose()[0] > 1.25:
            break
    return dict(obstacle=name, direction=direction, speed=v, mass=round(float(sum(m.body_mass)), 3),
                wheels=[dict(**{k: round(vv, 4) for k, vv in peak[i].items()}, at_peak_Fz=at_fz[i], at_peak_Fh=at_fh[i],
                             pod_5ms=dict(peak_N=round(pod_peak[i]["F"], 1), at_peak=pod_peak[i]["at"],
                                          max_xyz=[round(v, 1) for v in pod_peak[i]["max_xyz"]],
                                          min_xyz=[round(v, 1) for v in pod_peak[i]["min_xyz"]]))
                        for i in range(nw)])


def run(jobs=6):
    cases = [(n, a, v) for n in obstacles() for a in DIRS for v in SPEEDS]
    with Pool(jobs) as pool:
        rows = pool.map(job, cases, chunksize=1)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(rows, indent=1))
    report(rows)


def report(rows=None):
    rows = rows if rows is not None else json.loads(OUT.read_text())
    print(f"{'case':42s} {'Fz max':>7s} {'(5 ms)':>7s} {'Fh max':>7s} {'(5 ms)':>7s} {'tq N m':>7s} {'travel mm':>11s}")
    for x in rows:
        w = x["wheels"]
        print(f"{x['obstacle'] + ' ' + str(int(x['direction'])) + ' deg ' + str(x['speed']) + ' m/s':42s} "
              f"{max(q['Fz'] for q in w):7.0f} {max(q['Fz_5ms'] for q in w):7.0f} {max(q['Fh'] for q in w):7.0f} "
              f"{max(q['Fh_5ms'] for q in w):7.0f} {max(q['torque'] for q in w):7.2f} "
              f"{1000 * max(q['susp_max'] for q in w):5.1f}/{1000 * min(q['susp_min'] for q in w):5.1f}  pod frame 5 ms "
              f"x(axle) {min(q['pod_5ms']['min_xyz'][0] for q in w):5.0f}..{max(q['pod_5ms']['max_xyz'][0] for q in w):4.0f} "
              f"y(pivot) {min(q['pod_5ms']['min_xyz'][1] for q in w):5.0f}..{max(q['pod_5ms']['max_xyz'][1] for q in w):4.0f} "
              f"z {max(q['pod_5ms']['max_xyz'][2] for q in w):4.0f}")


if __name__ == "__main__":
    a = sys.argv[1:]
    j = int(a[a.index("-j") + 1]) if "-j" in a else 6
    if not a or a[0] == "run":
        run(j)
    else:
        report()
