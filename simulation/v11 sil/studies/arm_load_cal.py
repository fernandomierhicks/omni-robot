"""The arm load calibration on the detailed arm (design.md 4.5; omni_calib arm_load, the procedure the robot runs from
the console's Calibrate tab): does it find the arm, and what does the robot gain?

The arm is params.arm_model "detailed" (arm_detailed.py: motors, gearboxes with friction and backlash, horn and link
flex, heat, supply sag; the servos' firmware and sensors in servo_emulator.py), its truth drawn per seed. The robot's
own C++ arm driver (omni_cpp ArmDriverCore through arm_sil) talks to the emulated servo bus byte for byte, and
omni_calib's ArmLoadCal runs the procedure on it as arm_brain does: moves, holds, sweeps; a simulated person
(sim_human.py: kitchen scale, calipers, ruler, luggage scale with their errors) weighs and hangs the weights, reads the
ruler and pulls. Gravity's direction comes from the base's attitude with 0.15 deg of IMU error (est).

Then the same arm (the same truth, new poses) is tested with the driver three ways: "legacy" (as before Nov 20: the
encoders' angles, no hold trim, no reflex), "nominal" (the datasheet's load model, the hold trim, the reflex allowed
on it) and "calibrated" (the procedure's model, the hold trim, the reflex):
  still     12 poses held, unloaded and with a 0.20 kg test weight on the finger: the outside torque the model sees
            (the band's middle) against the truth, and the weight's vertical force
  moves     25 pose-to-pose moves at the tidy's speed, guarded: false reflexes, the detector's peak against its threshold
  press     a 3 kg box (12 x 12 x 8 cm) 0.37 m ahead; the fingers, down, driven from 7 cm over it to 2 cm into it in
            1.5 s, guarded (the arm can't reach below the floor fingers-down, and a wrist dip only lifts the tips):
            the contact force when the reflex fires, the delay after first contact, the peak before it
  push      the arm settled and holding (guarded), a hand pushing the fingertip sideways, 2.5 N/s: the force when
            the model first sees an outside torque and when the reflex fires (the load or the deflection cue)
  reach     10 fingers-down points 2 cm over the floor, guarded moves with the reflex off (the moves test counts the
            false reflexes): where the tool really is against where the robot sends it

    python studies/arm_load_cal.py                     # 3 seeds, results/arm_load_cal.json + .md
    python studies/arm_load_cal.py --seeds 1 --quick   # a smoke run (the short procedure)
    python studies/arm_load_cal.py --records DIR       # fit again the records an earlier run kept (ARM_LOAD_RECORDS=DIR)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from sil_link import ROS_SRC  # noqa: E402  (puts ros/src on the path)

sys.path.insert(0, str(ROS_SRC / "omni_cpp" / "sil"))
sys.path.insert(0, str(ROS_SRC / "omni_calib"))

import arm_so101 as A  # noqa: E402
from arm_sil import MOVE, CppArmDriver  # noqa: E402
from robot_export import arm_params  # noqa: E402
from robot_model import build_model  # noqa: E402
from servo_emulator import ServoBus  # noqa: E402
from sim_human import ArmCalHuman  # noqa: E402
from sil_robot import SilRobot  # noqa: E402
from omni_arm import motion as mo  # noqa: E402
from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path  # noqa: E402
from omni_calib import arm_load as AL  # noqa: E402

OUT = Path(__file__).resolve().parent / "results"
CYCLE = 0.02                                   # s: the driver's 50 Hz
IMU_ERR_DEG = 0.15


class Bench:
    """The arm robot in the sim with the detailed arm, the emulated servo bus and the robot's C++ driver."""

    def __init__(self, seed: int, load: str = "", log=None, trim: bool = True, reflex_nominal: bool = False,
                 box: bool = False, datums: bool = False):
        kw = {}
        if datums:                               # the touch blocks beside the arm (arm_so101.DATUMS, studies/arm_datums.py)
            kw["arm_datums"] = A.DATUMS
        if box:                                  # the press test's box: 0.37 m ahead of the robot (it faces -x)
            kw["floor_objects"] = (("box", (0.12, 0.12, 0.08), 1.0 - 0.37, 0.0, 0.0, (0.6, 0.4, 0.2, 1.0), 3.0),)
        self.p = p = arm_params(arm_model="detailed", depth_camera="d435", arm_seed=seed, **kw)
        self.m = build_model(p)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        self.r = SilRobot(self.m, self.d, p, seed=seed)
        self.arm = self.r.arm
        self.bus = ServoBus(self.arm.servos)
        self.kin = ArmKinematics(ArmConfig(config_path("d435")))
        self.drv = CppArmDriver(self.bus, self.kin.cfg.path, load=load)
        self.missing = self.drv.connect()
        self.notes = self.drv.config_notes()
        self.drv.settings(reflex_nominal=reflex_nominal, hold_trim=trim)
        rng = np.random.default_rng(seed + 61)
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        a = math.radians(IMU_ERR_DEG)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        self.R_imu = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
        self.base = self.m.body("base").id
        self.t_next = 0.0
        self.until = -1.0
        self.hw = None
        self.ld = None
        self.log = log or (lambda s: None)
        self.speeds = mo.speeds(self.kin)
        self.via = [self.kin.cfg.poses[n] for n in ("high",) if n in self.kin.cfg.poses]
        self.run(1.0)                            # settle on the floor, holding

    def base_R(self) -> np.ndarray:
        return self.d.xmat[self.base].reshape(3, 3).copy()

    def gravity(self) -> np.ndarray:
        return self.R_imu @ (self.base_R().T @ np.array([0.0, 0.0, -9.81]))

    def tick(self):
        d = self.d
        if d.time >= self.t_next - 1e-9:
            self.t_next += CYCLE
            self.drv.set_gravity(self.gravity())
            self.hw = self.drv.cycle(d.time)
            self.ld = self.drv.load_state()
            self.on_cycle()
        self.r.step()

    def on_cycle(self):
        pass

    def run(self, dt: float, until=None):
        t_end = self.d.time + dt
        while self.d.time < t_end:
            self.tick()
            if until is not None and until():
                return True
        return False

    @property
    def moving(self) -> bool:
        return self.d.time < self.until

    def obs(self) -> dict:
        hw, ld = self.hw, self.ld
        return dict(t=self.d.time, q=hw.q, q_enc=ld.q_enc, duty=hw.load, volts=ld.volts,
                    temp=[float(v) for v in hw.temp_c], current=ld.current, w=hw.dq, goal_vel=ld.goal_vel,
                    goal_acc=ld.goal_acc, g=self.gravity(), moving=self.moving, ok=hw.ok)

    def move(self, q, frac: float = 0.5, guard: bool = True) -> bool:
        q0 = np.asarray(self.hw.goal, float)
        path, why = mo.plan_checked(self.kin, q0, np.asarray(q, float), via=self.via)
        if path is None:
            self.log(f"move refused: {why}")
            return False
        m = mo.Motion(path, self.speeds * frac / mo.SPEED_FRAC, self.d.time)
        self.drv.command(MOVE, m.t, m.q, guard=guard)
        self.until = m.t_end + 0.06
        return True

    def line(self, q0, q1, duration, guard: bool = True):
        t = self.d.time
        self.drv.command(MOVE, [t, t + duration], [list(q0), list(q1)], linear=True, guard=guard)
        self.until = t + duration + 0.06

    def tool_base(self) -> np.ndarray:
        """The tool point where it really is, in the base's frame (base_link)."""
        sid = self.m.site("arm_tool").id
        return self.base_R().T @ (self.d.site_xpos[sid] - self.d.xpos[self.base])

    def arm_contact_force(self) -> float:
        """Normal force on the arm's links from everything they touch (N)."""
        m, d = self.m, self.d
        arm_bodies = {m.body(n).id for n in ("arm_shoulder", "arm_upper_arm", "arm_lower_arm", "arm_wrist",
                                             "arm_gripper", "arm_jaw")}
        f = np.zeros(6)
        tot = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if m.geom_bodyid[c.geom1] in arm_bodies or m.geom_bodyid[c.geom2] in arm_bodies:
                mujoco.mj_contactForce(m, d, i, f)
                tot += abs(f[0])
        return tot


# -- the calibration -----------------------------------------------------------------------------------------------

def refit(seed: int, rec_path: Path, quick: bool = False) -> dict:
    """A calibration's kept records fitted again (the fit changed, the arm and the person didn't)."""
    import pickle
    d = pickle.loads(Path(rec_path).read_bytes())
    kin = ArmKinematics(ArmConfig(config_path("d435")))
    res = AL.fit(kin, d["records"], d["weights"])
    path = OUT / f"arm_load_seed{seed}{'_quick' if quick else ''}.json"
    path.write_text(json.dumps(res["value"], indent=1, default=float))
    return dict(seed=seed, quick=quick, refit=str(rec_path), records=len(d["records"]), sim_s=None, temps=None,
                bus=None, human=d.get("human"), truth=d.get("truth"), model_path=str(path), metrics=res["metrics"],
                summary=res["summary"])


def calibrate(seed: int, quick: bool = False, verbose: bool = False) -> dict:
    w0 = time.perf_counter()
    log = (lambda s: print(f"[seed {seed}] {s}", flush=True)) if verbose else (lambda s: None)
    b = Bench(seed, log=log)
    human = ArmCalHuman(b.arm, b.base_R, seed=seed)
    cal = AL.ArmLoadCal(b.kin, seed=seed, quick=quick, log=log)
    pending = None                                    # (t_due, prompt)
    state = dict(done=None)

    def on_cycle():
        nonlocal pending
        if state["done"] is not None:
            return
        t = b.d.time
        if pending is not None and t >= pending[0]:
            ok, why = cal.answer(human.answer(pending[1]))
            if not ok:
                log(f"answer refused: {why}")
            pending = None
        for a in cal.step(t, b.obs()):
            op = a["op"]
            if op == "move":
                if not b.move(a["q"], a.get("frac", 0.3), guard=False):
                    cal.refused("no clear path")
            elif op == "line":
                if b.kin.path_clear(a["q0"], a["q1"])[0]:
                    b.line(a["q0"], a["q1"], a["duration"], guard=False)
                else:
                    cal.refused("sweep not clear")
            elif op == "prompt":
                pending = (t + human.delay(a), a)
                log(f"prompt {a['kind']} {a.get('key', '')}")
            elif op == "done":
                state["done"] = a

    b.on_cycle = on_cycle
    while state["done"] is None and b.d.time < 1800:
        b.tick()
    res = state["done"]["result"] if state["done"] else None
    out = dict(seed=seed, quick=quick, sim_s=round(b.d.time, 1), wall_s=round(time.perf_counter() - w0, 1),
               records=len(cal.records), events=cal.events[-30:], why=cal.why, human=human.truth(),
               servo_notes=b.notes, truth=_truth(b), temps=[int(v) for v in b.hw.temp_c],
               bus=dict(packets=b.bus.stats.packets, errors=b.bus.stats.errors,
                        sync_read_us=round(b.bus.stats.busy_us / max(1, b.bus.stats.by_instr.get(0x82, 1)), 1)))
    rec_dir = os.environ.get("ARM_LOAD_RECORDS")              # keep the records to fit again (pickle)
    if rec_dir:
        import pickle
        Path(rec_dir).mkdir(parents=True, exist_ok=True)
        with open(Path(rec_dir) / f"records_seed{seed}{'_quick' if quick else ''}.pkl", "wb") as f:
            pickle.dump(dict(records=cal.records, weights=cal.weights, truth=out["truth"], human=human.truth()), f)
    if res is not None:
        path = OUT / f"arm_load_seed{seed}{'_quick' if quick else ''}.json"
        path.write_text(json.dumps(res["value"], indent=1))
        out.update(model_path=str(path), metrics=res["metrics"], summary=res["summary"])
        log(res["summary"])
    return out


def _truth(b: Bench) -> dict:
    """What the calibration should have found (for the report): masses, sag (rad/N m), the encoders' offsets."""
    a = b.arm
    return dict(masses={k: round(v["mass"], 4) for k, v in a.mass_truth.items()},
                compliance=[round(1.0 / s.k_flex, 4) for s in a.truth[:5]],
                cal_err_deg=[round(math.degrees(sv.s.cal_err), 2) for sv in a._sv[:5]])


# -- the tests -----------------------------------------------------------------------------------------------------

def _poses(kin, rng, n, z_min=0.15):
    out = []
    box = np.radians([[-50, 50], [-70, 30], [-60, 80], [-80, 80], [-80, 80]])
    while len(out) < n:
        q = np.r_[rng.uniform(box[:, 0], box[:, 1]), AL.CLOSED]
        if kin.clear(q)[0] and AL._tip_base(kin, q)[2] > z_min:
            out.append(q)
    return out


def evaluate(seed: int, load: str = "", label: str = "", legacy: bool = False) -> dict:
    """The five tests on the arm of this seed, the driver on load ("" = nominal); legacy: no hold trim, no reflex."""
    rng = np.random.default_rng(seed + 991)
    b = Bench(seed, load=load, trim=not legacy, reflex_nominal=not legacy)
    if legacy:
        b.drv.settings(reflex=False, hold_trim=False, model_angles=False)
    kin = b.kin
    model = AL.LoadModel.load(load, kin.cfg) if load else AL.LoadModel.nominal(kin.cfg)
    out = dict(label=label or ("calibrated" if load else "legacy" if legacy else "nominal"))

    def hold_avg(T=0.5):
        acc = []

        def grab():
            acc.append((np.array(b.hw.q), np.array(b.ld.motor), np.array(b.ld.gravity), np.array(b.ld.external)))
            return False
        b.on_cycle = grab
        b.run(T)
        b.on_cycle = lambda: None
        q, mt, gr, ex = (np.mean([a[k] for a in acc], axis=0) for k in range(4))
        return q, mt, gr, ex

    # still: unloaded, then a 0.20 kg test weight 10 mm up the finger
    errs, ferrs, trips_still = [], [], 0
    point = AL.hang_point(10.0)
    for k, q in enumerate(_poses(kin, rng, 12)):
        for kg in (0.0, 0.20):
            qa = q.copy()
            qa[:5] += rng.choice([-1, 1]) * math.radians(4)
            if not b.move(qa, 0.3, guard=False):
                continue
            b.run(4.0, until=lambda: not b.moving)
            b.move(q, 0.1, guard=False)
            if kg:
                b.arm.hang(kg, point)
            b.run(4.0, until=lambda: not b.moving)
            b.run(1.2 if kg else 0.8)
            qm, mt, gr, ex = hold_avg()
            centre = -gr[:5] - mt[:5]
            truth = kin.point_torque(qm, np.array([0, 0, -9.81 * kg]) if kg else np.zeros(3), point)[:5]
            errs.append(centre - truth)
            trips_still += int(kg == 0 and np.any(np.abs(ex[:5]) > 0))
            if kg:
                f = AL.tool_force(kin, qm, centre, point)
                ferrs.append(float(np.linalg.norm(f - [0, 0, -9.81 * kg])))
                b.arm.unhang()
    E = np.array(errs)
    out["still"] = dict(rms_nm=round(float(np.sqrt(np.mean(E ** 2))), 4),
                        per_joint_rms_nm=[round(float(v), 4) for v in np.sqrt(np.mean(E ** 2, axis=0))],
                        weight_force_err_n=round(float(np.sqrt(np.mean(np.square(ferrs)))), 3),
                        unloaded_outside_seen=trips_still, n=len(errs))
    # moves: tidy-speed motions, guarded
    reflex0 = b.ld.reflexes
    peak = np.zeros(5)
    thr = np.array([j.thr for j in model.joints])

    def watch():
        nonlocal peak
        peak = np.maximum(peak, np.abs(np.array(b.ld.filtered[:5])) / thr)
        return False
    b.on_cycle = watch
    poses = _poses(kin, rng, 25)
    moved = 0
    for q in poses:
        if b.move(q, 0.5, guard=True):
            moved += 1
            b.run(6.0, until=lambda: not b.moving)
            b.run(0.3)
            if b.ld.reflex_active:                   # a false reflex: carry on from where it stopped
                b.drv.command(1)
    b.on_cycle = lambda: None
    out["moves"] = dict(moves=moved, false_reflexes=b.ld.reflexes - reflex0,
                        peak_over_threshold=[round(float(v), 2) for v in peak])
    out["press"] = press(seed, load, legacy)
    # push: a hand on the fingertip, sideways, ramping
    q = kin.ik(np.array([0.34, 0.0, 0.20]), np.array([0.0, 0.0, -1.0]), yaw=0.0)
    ps = dict(fired=False)
    if q is not None:
        q[5] = AL.CLOSED
        b.move(q, 0.3, guard=False)
        b.run(5.0, until=lambda: not b.moving)
        b.drv.command(1)                             # hold, guarded
        b.run(1.5)                                   # settled: the hold trim has taken the droop out
        r0 = b.ld.reflexes
        t0 = b.d.time
        while b.d.time < t0 + 6.0:
            fy = 2.5 * (b.d.time - t0)
            b.arm.push(b.base_R() @ np.array([0.0, fy, 0.0]), AL.TIP)
            b.tick()
            if "seen_n" not in ps and np.abs(np.array(b.ld.external[:5])).max() > 0:
                ps["seen_n"] = round(fy, 2)                    # the model first sees an outside torque
            if b.ld.reflexes > r0:
                ps.update(fired=True, force_n=round(fy, 2), joints=[k for k in range(5) if b.ld.strained[k]])
                break
        b.arm.push(None, None)
        b.run(0.5)
    out["push"] = ps
    # reach: fingers-down points 2 cm over the floor; the reflex off (false reflexes are the moves test's)
    b.drv.settings(reflex=False, hold_trim=not legacy, model_angles=not legacy)
    rerr = []
    for _ in range(10):
        p = np.array([rng.uniform(0.30, 0.40), rng.uniform(-0.10, 0.10), 0.020 + 0.006])
        q = kin.ik(p, np.array([0.0, 0.0, -1.0]), yaw=rng.uniform(-0.8, 0.8))
        if q is None:
            continue
        q[5] = AL.CLOSED
        if not b.move(q, 0.3, guard=True):                # guarded, as the tidy's: the hold trim works
            continue
        b.run(5.0, until=lambda: not b.moving)
        if b.ld.reflex_active:
            b.drv.command(1)
        b.run(1.5)
        rerr.append((b.tool_base() - kin.tool(q)[:3, 3]) * 1000)
    R = np.array(rerr)
    out["reach"] = dict(rms_mm=round(float(np.sqrt(np.mean(np.sum(R ** 2, axis=1)))), 2),
                        vertical_rms_mm=round(float(np.sqrt(np.mean(R[:, 2] ** 2))), 2), n=len(rerr))
    out["temps"] = [int(v) for v in b.hw.temp_c]
    return out


def press(seed: int, load: str, legacy: bool) -> dict:
    """The fingers pressed down onto a heavy box, guarded."""
    b = Bench(seed, load=load, trim=not legacy, reflex_nominal=not legacy, box=True)
    if legacy:
        b.drv.settings(reflex=False, hold_trim=False, model_angles=False)
    kin = b.kin
    above = kin.ik(np.array([0.37, 0.0, 0.08 + 0.07 + 0.006]), np.array([0.0, 0.0, -1.0]), yaw=math.pi / 2)
    into = kin.ik(np.array([0.37, 0.0, 0.08 - 0.02 + 0.006]), np.array([0.0, 0.0, -1.0]), yaw=math.pi / 2)
    out = dict(fired=False)
    if above is None or into is None:
        return dict(out, why="no IK")
    above[5] = into[5] = AL.CLOSED
    b.move(above, 0.3, guard=False)
    b.run(5.0, until=lambda: not b.moving)
    b.drv.command(1)
    b.run(1.5)
    r0 = b.ld.reflexes
    b.drv.command(MOVE, [b.d.time, b.d.time + 1.5], [list(above), list(into)], linear=True, guard=True)
    t_contact, f_peak = None, 0.0
    t_end = b.d.time + 3.0
    while b.d.time < t_end:
        b.tick()
        f = b.arm_contact_force()
        if t_contact is None and f > 0.5:
            t_contact = b.d.time
        if b.ld.reflexes > r0:
            out.update(fired=True, delay_s=round(b.d.time - t_contact, 3) if t_contact else None,
                       force_at_reflex_n=round(f, 2), joints=[k for k in range(5) if b.ld.strained[k]])
            break
        f_peak = max(f_peak, f)
    out.update(contact=t_contact is not None, peak_force_before_reflex_n=round(f_peak, 2))
    return out


def run_seed(args) -> dict:
    seed, quick, verbose, rec_dir = args
    rec = Path(rec_dir) / f"records_seed{seed}{'_quick' if quick else ''}.pkl" if rec_dir else None
    c = refit(seed, rec, quick) if rec is not None and rec.exists() else calibrate(seed, quick, verbose)
    res = dict(seed=seed, calibration=c)
    res["legacy"] = evaluate(seed, "", legacy=True)
    res["nominal"] = evaluate(seed, "")
    if c.get("model_path"):
        res["calibrated"] = evaluate(seed, c["model_path"])
    return res


def report(rows) -> str:
    cfgs = ("legacy", "nominal", "calibrated")
    L = ["# Arm load calibration on the detailed arm (studies/arm_load_cal.py)", "",
         "Each seed: a detailed arm (its own truth), the procedure with the simulated person, then the same arm tested "
         "with the driver three ways: legacy (before Nov 20: no hold trim, no reflex) / nominal (datasheet load model, "
         "trim, reflex allowed) / calibrated. est throughout: no servo measured yet.", ""]

    def get(r, c, *ks):
        v = r.get(c, {})
        for k in ks:
            v = (v or {}).get(k)
        return v

    def fmt(v, f="{:.3f}"):
        return "-" if v is None else f.format(v) if isinstance(v, (int, float)) else str(v)

    rows_def = [
        ("still torque error, N m rms", ("still", "rms_nm"), "{:.3f}"),
        ("0.20 kg weight's force error, N", ("still", "weight_force_err_n"), "{:.2f}"),
        ("false reflexes in 25 tidy-speed moves", ("moves", "false_reflexes"), "{}"),
        ("box press: contact force at the reflex, N", ("press", "force_at_reflex_n"), "{:.1f}"),
        ("box press: delay after first contact, s", ("press", "delay_s"), "{:.2f}"),
        ("box press: peak force before the reflex (or all), N", ("press", "peak_force_before_reflex_n"), "{:.1f}"),
        ("sideways push: force at the reflex, N", ("push", "force_n"), "{:.1f}"),
        ("reach: tool error, mm rms", ("reach", "rms_mm"), "{:.1f}"),
        ("reach: vertical, mm rms", ("reach", "vertical_rms_mm"), "{:.1f}"),
    ]
    for r in rows:
        L += [f"## Seed {r['seed']}", "", "| | " + " | ".join(cfgs) + " |", "| --- | --- | --- | --- |"]
        for label, ks, f in rows_def:
            L.append(f"| {label} | " + " | ".join(fmt(get(r, c, *ks), f) for c in cfgs) + " |")
        cal = r["calibration"]
        src = f"refit of {Path(cal['refit']).name}" if cal.get("refit") else f"{cal['sim_s']} s of sim"
        tail = f"; servo temperatures at the end {cal['temps']}; bus {cal['bus']}" if cal.get("temps") else ""
        L += ["", f"Calibration: {cal.get('summary', cal.get('why'))}; {cal['records']} records, {src}{tail}", ""]
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--records", default="", help="fit again the records kept here (records_seed<N>.pkl)")
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    jobs = [(s, a.quick, a.verbose, a.records) for s in range(a.seeds)]
    if len(jobs) == 1:
        rows = [run_seed(jobs[0])]
    else:
        with ProcessPoolExecutor(max_workers=min(len(jobs), 4)) as ex:
            rows = list(ex.map(run_seed, jobs))
    tag = "_quick" if a.quick else ""
    (OUT / f"arm_load_cal{tag}.json").write_text(json.dumps(rows, indent=1, default=float))
    md = report(rows)
    (OUT / f"arm_load_cal{tag}.md").write_text(md)
    print(md)


if __name__ == "__main__":
    main()
