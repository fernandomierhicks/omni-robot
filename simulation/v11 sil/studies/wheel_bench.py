"""A real wheel motor on the bench against the sim's drive model (Nov 13, protocol 9).

The bench: one AK40-10 turning a bare 140 mm omni wheel (wheels up), driven by the robot's own stack: the console's
motor test (motors_run) -> mode_manager WHEELS -> the firmware's DRIVE_WHEELS ramp -> servo-mode ERPM; recorded at the
MCU's 200 Hz by ros/tools/wheel_capture.py (asked speed, measured speed, the motor's current).

The sim: the same asked speed (the firmware's command, as the motor gets it: every 4 ms, round robin) through the sim's
own wheel drive: models.DriveActuator (its speed PI, kp 2 / ki 40 N m s/rad at the output, the current and voltage
limits), the drive joint's inertia (MuJoCo's, armature = the rotor reflected, plus the wheel), its friction (Coulomb
backdrive_torque + viscous) on a 1-DOF wheel in the air. What the sim predicts for current: DriveMotor.current_for.

    python studies/wheel_bench.py capture.csv [more.csv ...] --wheel 2 [--j 1.2e-3] [--out results/wheel_bench_X]

Per capture: an overlay plot (speed, current) and per transition the measured / sim time to 90 %, peak acceleration,
peak current, overshoot; per steady hold the mean current (friction). Writes <out>.json and <out>_<name>.png.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from models import DriveActuator  # noqa: E402

V_BUS = 24.0              # the bench supply (the R4 bench reports a fixed 24 V)
DT = 0.0005               # the sim's timestep (params.timestep)
CMD_PERIOD = 0.004        # each AK40-10 gets a frame every 4 ms (the firmware's round robin over 4 wheels)


def load(path: Path, wheel: int) -> dict:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    k = str(wheel)
    t = np.array([float(r["t"]) for r in rows])
    return dict(name=path.stem, t=t, c=np.array([float(r["c" + k]) for r in rows]),
                w=np.array([float(r["w" + k]) for r in rows]), i=np.array([float(r["i" + k]) for r in rows]),
                e=np.array([int(r["e" + k]) for r in rows]), faults=sorted({r["faults"] for r in rows} - {""}),
                drive=np.array([int(r["drive_mode"]) for r in rows]))


def sim_params():
    """The arm robot's drive as the sim has it: motor spec, joint inertia (armature + wheel), friction."""
    import mujoco
    import robot_export
    import robot_model
    p = robot_export.arm_params()
    m = robot_model.build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, M)
    dof = m.jnt_dofadr[m.joint("drive1").id]
    return dict(p=p, J=float(M[dof, dof]), armature=float(m.dof_armature[dof]),
                fric=float(m.dof_frictionloss[dof]), visc=float(m.dof_damping[dof]))


def simulate(t_cmd: np.ndarray, c: np.ndarray, sp: dict, J: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """1-DOF wheel in the air under the sim's DriveActuator. The command: the firmware's (200 Hz samples,
    interpolated to its 1 kHz ticks), sampled every CMD_PERIOD as the motor receives it."""
    mot = sp["p"].motor
    drv = DriveActuator(mot, sp["p"].drive_kp, sp["p"].drive_ki)
    kp = drv.kp
    ts = np.arange(t_cmd[0], t_cmd[-1], DT)
    w, sp_w, t_next = 0.0, 0.0, ts[0]
    W, I = np.zeros_like(ts), np.zeros_like(ts)
    for n, t in enumerate(ts):
        if t >= t_next - 1e-12:
            sp_w = float(np.interp(math.floor(t * 1000) / 1000, t_cmd, c))
            t_next += CMD_PERIOD
        ctrl, (lo, hi) = drv.command(sp_w, w, V_BUS, DT)
        # MuJoCo's general actuator: force = kp (ctrl - qvel), clipped to its range (explicit here: kp dt / J ~ 0.5)
        f = min(max(kp * (ctrl - w), lo), hi)
        net = f - sp["visc"] * w
        fr = sp["fric"]
        if abs(w) < 1e-6 and abs(net) <= fr:            # stuck: friction holds it
            w_new = 0.0
        else:
            s = math.copysign(1.0, w if abs(w) > 1e-6 else net)
            w_new = w + DT * (net - fr * s) / J
            if w != 0.0 and w_new * w < 0.0 and abs(net) <= fr:
                w_new = 0.0
        w = w_new
        W[n] = w
        I[n] = math.copysign(mot.current_for(f), f)
    # the sim's speed loop is far stiffer than the motor's: each 4 ms command step is a torque pulse. Averaged over
    # 5 ms, the window one 200 Hz sample spans, its current compares with the motor's reported one
    k = int(round(0.005 / DT))
    I = np.convolve(I, np.ones(k) / k, mode="same")
    return ts, W, I


def segments(t, c, tol=0.05):
    """Hold intervals of the asked speed: (t0, t1, value)."""
    out, s = [], 0
    for k in range(1, len(c) + 1):
        if k == len(c) or abs(c[k] - c[s]) > tol:
            if t[k - 1] - t[s] >= 0.15:
                out.append((t[s], t[k - 1], float(c[s])))
            s = k
    return out


def deriv(t, x, win=0.010):
    """Speed -> acceleration: a centred difference over win s (2 samples either side at 200 Hz)."""
    a = np.zeros_like(x)
    h = max(1, int(round(win / np.median(np.diff(t)))))
    a[h:-h] = (x[2 * h:] - x[:-2 * h]) / (t[2 * h:] - t[:-2 * h])
    return a


def transitions(t, c, w, i, ws=None, ii=None, ts=None):
    """Each change of the asked speed between holds: the measured (and sim) response."""
    holds = segments(t, c)
    out = []
    for (a0, a1, v0), (b0, b1, v1) in zip(holds, holds[1:]):
        if abs(v1 - v0) < 0.5:
            continue
        sel = (t >= a1) & (t <= b1)
        tt, ww, ii_ = t[sel], w[sel], i[sel]
        target = v0 + 0.9 * (v1 - v0)
        hit = np.nonzero((ww - target) * np.sign(v1 - v0) >= 0)[0]
        r = dict(from_=v0, to=v1, t_start=round(float(a1), 3), ramp_cmd_s=round(float(b0 - a1), 4),
                 t90=round(float(tt[hit[0]] - a1), 4) if len(hit) else None,
                 acc_peak=round(float(np.max(np.abs(deriv(tt, ww)))), 1),
                 i_peak=round(float(ii_[np.argmax(np.abs(ii_))]), 2),
                 overshoot=round(float(np.max((ww - v1) * np.sign(v1 - v0))), 2))
        if ws is not None:
            sel = (ts >= a1) & (ts <= b1)
            tt, ww, ii_ = ts[sel], ws[sel], ii[sel]
            hit = np.nonzero((ww - target) * np.sign(v1 - v0) >= 0)[0]
            t2 = t[(t >= a1) & (t <= b1)]                   # the sim seen as the bench is: 200 Hz, same derivative
            r.update(sim_t90=round(float(tt[hit[0]] - a1), 4) if len(hit) else None,
                     sim_acc_peak=round(float(np.max(np.abs(deriv(t2, np.interp(t2, ts, ws))))), 1),
                     sim_i_peak=round(float(ii_[np.argmax(np.abs(ii_))]), 2),
                     sim_overshoot=round(float(np.max((ww - v1) * np.sign(v1 - v0))), 2))
        out.append(r)
    return out


def holds_report(t, c, w, i, ts=None, ws=None, ii=None):
    out = []
    for a0, a1, v in segments(t, c):
        lo = a0 + 0.5 * (a1 - a0)                       # the second half: settled
        sel = (t >= lo) & (t <= a1)
        r = dict(cmd=v, w_mean=round(float(np.mean(w[sel])), 3), w_std=round(float(np.std(w[sel])), 3),
                 i_mean=round(float(np.mean(i[sel])), 3), i_std=round(float(np.std(i[sel])), 3))
        if ts is not None:
            s2 = (ts >= lo) & (ts <= a1)
            r.update(sim_w_mean=round(float(np.mean(ws[s2])), 3), sim_i_mean=round(float(np.mean(ii[s2])), 3))
        out.append(r)
    return out


def plot(cap, ts, ws, ii, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(11, 6.5))
    ax[0].plot(cap["t"], cap["c"], "--", color="tab:orange", lw=1, label="asked (firmware ramp)")
    ax[0].plot(cap["t"], cap["w"], ".-", color="tab:blue", lw=1, ms=2.5, label="measured (200 Hz)")
    ax[0].plot(ts, ws, color="tab:green", lw=1, alpha=0.8, label="sim")
    ax[0].set_ylabel("wheel speed, rad/s")
    ax[0].legend(loc="upper right", fontsize=8)
    ax[1].plot(cap["t"], cap["i"], ".-", color="tab:blue", lw=1, ms=2.5, label="measured")
    ax[1].plot(ts, ii, color="tab:green", lw=1, alpha=0.8, label="sim")
    ax[1].set_ylabel("motor current, A")
    ax[1].set_xlabel("s")
    ax[1].legend(loc="upper right", fontsize=8)
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(cap["name"])
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--wheel", type=int, default=2)
    ap.add_argument("--j", type=float, help="kg m^2 at the output, in place of the sim's (armature + wheel)")
    ap.add_argument("--out", default=str(HERE / "results" / "wheel_bench"))
    a = ap.parse_args()
    sp = sim_params()
    J = a.j or sp["J"]
    mot = sp["p"].motor
    report = dict(sim=dict(J=J, J_sim=sp["J"], armature=sp["armature"], fric=sp["fric"], visc=sp["visc"],
                           kp=sp["p"].drive_kp, ki=sp["p"].drive_ki, k_out=mot.k_out, k_sat=mot.k_sat,
                           i_peak=mot.i_peak, r_eq=mot.r_eq, ke=mot.ke, v_bus=V_BUS), captures={})
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    for f in a.csv:
        cap = load(Path(f), a.wheel)
        ts, ws, ii = simulate(cap["t"], cap["c"], sp, J)
        plot(cap, ts, ws, ii, out.parent / f"{out.name}_{cap['name']}.png")
        report["captures"][cap["name"]] = dict(
            n=len(cap["t"]), span_s=round(float(cap["t"][-1]), 2), faults=cap["faults"],
            motor_errors=sorted({int(e) for e in cap["e"]} - {0, 1}),
            transitions=transitions(cap["t"], cap["c"], cap["w"], cap["i"], ws, ii, ts),
            holds=holds_report(cap["t"], cap["c"], cap["w"], cap["i"], ts, ws, ii))
    Path(str(out) + ".json").write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
