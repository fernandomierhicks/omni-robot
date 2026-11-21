"""Do touch blocks on the robot zero the arm better than the ruler? (design.md 4.5; omni_calib arm_datum; a proposal,
Nov 20, tried in simulation before any CAD.)

Each seed's detailed arm (arm_detailed.py, the same truth as studies/arm_load_cal.py's) gets two 20 x 20 x 70 mm blocks
on the plate beside the arm's adapter (arm_so101.DATUMS), each 0.3 mm rms off its drawing; the robot knows only the
drawing. Its driver on the calibrated load model (results/arm_load_seed<N>.json from arm_load_cal.py) touches each
block's top and sides with the fixed fingertip, slowly (3 mm/s) at 30 % torque, gripper open; a touch is where the
joints fell behind the goal. Then four models of the joints' zeros and sag, the torque model the same:
  none     no zeros, no sag (the encoders' angles; the hold trim still on)
  ruler    the arm load calibration's own (12 fingertip heights read with a ruler, 0.6 mm + parallax)
  datums   the touches alone
  both     the touches and the ruler's heights
  pan      the ruler's model with only pan's zero from the touches (the joint the ruler can't see)
scored on 12 fingers-down floor points (where the tool really is against where the robot sends it) and against the
truth's encoder offsets (each servo's hand calibration error) and horn / link sag.

    python studies/arm_datums.py              # seeds 0-2 (needs results/arm_load_seed<N>.json), results/arm_datums.md
    python studies/arm_datums.py --seeds 1
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import arm_load_cal as L  # noqa: E402  (its Bench, paths)

import arm_so101 as A  # noqa: E402
from arm_sil import MOVE  # noqa: E402
from omni_calib import arm_datum as AD  # noqa: E402
from omni_calib import arm_load as AL  # noqa: E402

OUT = HERE / "results"
SOFT_TORQUE = 300


def nominal_datums(p) -> list:
    from robot_model import geometry
    plate = geometry(p).plate_top
    return [AD.Datum(n, np.array([x, y, plate + h / 2]), np.array([sx / 2, sy / 2, h / 2]))
            for n, x, y, sx, sy, h in A.DATUMS]


def touch_off(seed: int, load: str, log=None) -> tuple:
    b = L.Bench(seed, load=load, datums=True, log=log)
    datums = nominal_datums(b.p)
    touches = AD.plan(b.kin, datums, seed=seed)
    proc = AD.TouchOff(touches, log=log)
    t0 = b.d.time
    soft = dict(on=False)

    def on_cycle():
        obs = dict(b.obs(), goal=b.hw.goal)
        for a in proc.step(b.d.time, obs):
            op = a["op"]
            if op == "move":
                if soft["on"]:
                    b.drv.settings(arm_torque=1000)
                    soft["on"] = False
                if not b.move(a["q"], a.get("frac", 0.3), guard=False):
                    proc.missed.append(touches[proc.k])
                    proc.phase = "back"
            elif op == "line":
                b.drv.settings(arm_torque=SOFT_TORQUE)          # soft: a touch presses lightly
                soft["on"] = True
                b.line(a["q0"], a["q1"], a["duration"], guard=False)
            elif op == "hold":
                b.drv.command(1, guard=False)
                b.until = -1.0

    b.on_cycle = on_cycle
    while not proc.done and b.d.time - t0 < 900:
        b.tick()
    from robot_model import geometry
    truth = {nm: c.tolist() for nm, c, hs in A.datum_truth(b.p, geometry(b.p).plate_top)}
    return proc, datums, truth, round(b.d.time - t0, 1)


def reach(seed: int, load: str, n: int = 12) -> dict:
    rng = np.random.default_rng(seed + 4711)
    b = L.Bench(seed, load=load)
    b.drv.settings(reflex=False, hold_trim=True)
    kin = b.kin
    err = []
    while len(err) < n:
        p = np.array([rng.uniform(0.30, 0.40), rng.uniform(-0.10, 0.10), 0.020 + 0.006])
        q = kin.ik(p, np.array([0.0, 0.0, -1.0]), yaw=rng.uniform(-0.8, 0.8))
        if q is None:
            continue
        q[5] = AL.CLOSED
        if not b.move(q, 0.3, guard=True):
            continue
        b.run(5.0, until=lambda: not b.moving)
        b.run(1.5)
        err.append((b.tool_base() - kin.tool(q)[:3, 3]) * 1000)
    E = np.array(err)
    return dict(rms_mm=round(float(np.sqrt(np.mean(np.sum(E ** 2, axis=1)))), 2),
                horizontal_rms_mm=round(float(np.sqrt(np.mean(np.sum(E[:, :2] ** 2, axis=1)))), 2),
                vertical_rms_mm=round(float(np.sqrt(np.mean(E[:, 2] ** 2))), 2))


def run_seed(args) -> dict:
    seed, rec_dir = args
    w0 = time.perf_counter()
    base_path = OUT / f"arm_load_seed{seed}.json"
    kin = L.ArmKinematics(L.ArmConfig(L.config_path("d435")))
    base = AL.LoadModel.load(str(base_path), kin.cfg)
    proc, datums, truth_blocks, t_touch = touch_off(seed, str(base_path))
    none = AL.LoadModel([AL.JointLoad(**{**AL.asdict(j), "zero": 0.0, "compliance": 0.0}) for j in base.joints],
                        base.links, base.use_current, base.source + " (no zeros)", 0.0)
    rec = pickle.loads((Path(rec_dir) / f"records_seed{seed}.pkl").read_bytes()) if rec_dir else None
    ruler = rec["records"] if rec else None
    weights = rec["weights"] if rec else None
    models = dict(none=none, ruler=base, datums=AD.fit(kin, none, proc.hits, datums))
    if ruler:
        models["both"] = AD.fit(kin, none, proc.hits, datums, ruler=ruler, weights=weights)
    models["pan"] = AD.fit(kin, base, proc.hits, datums, free=(0,))
    b = L.Bench(seed)                                    # the truth: the encoders' offsets and the horns' sag
    zero_true = np.array([sv.s.cal_err for sv in b.arm._sv[:5]])
    sag_true = np.array([1.0 / s.k_flex for s in b.arm.truth[:5]])
    out = dict(seed=seed, touches=len(proc.touches), hits=len(proc.hits), missed=len(proc.missed), touch_s=t_touch,
               blocks_truth=truth_blocks, zero_true_deg=np.round(np.degrees(zero_true), 2).tolist(),
               sag_true=np.round(sag_true, 4).tolist(), models={})
    for name, m in models.items():
        path = OUT / f"arm_datums_seed{seed}_{name}.json"
        path.write_text(json.dumps(m.to_json(kin.cfg), indent=1, default=float))
        z = np.array([j.zero for j in m.joints])
        c = np.array([j.compliance for j in m.joints])
        out["models"][name] = dict(
            zero_err_deg=np.round(np.degrees(z - zero_true), 2).tolist(),
            sag=np.round(c, 4).tolist(), reach=reach(seed, str(path)))
    out["hits_detail"] = [dict(datum=tc["datum"], normal=tc["normal"].tolist(), point=tc["point"].tolist(),
                               q_enc=q.tolist()) for tc, q in proc.hits]
    out["wall_s"] = round(time.perf_counter() - w0, 1)
    return out


def report(rows) -> str:
    names = ("none", "ruler", "datums", "both", "pan")
    L_ = ["# Touch blocks against the ruler (studies/arm_datums.py)", "",
          "Two 20 x 20 x 70 mm blocks beside the arm's adapter (0.3 mm off their drawing), touched by the fixed fingertip; "
          "the joints' zeros and sag from the touches, the ruler's heights, or both, on each seed's detailed arm. "
          "Reach: 12 fingers-down floor points (the hold trim on). Zero error: against each servo's true hand-calibration "
          "offset (pan, lift, elbow, wrist flex, roll). est throughout: no servo measured.", ""]
    for r in rows:
        L_ += [f"## Seed {r['seed']}: {r['hits']} of {r['touches']} touches in {r['touch_s']} s", "",
               "| | reach mm rms | horizontal | vertical | zero error deg (pan, lift, elbow, wrist, roll) |",
               "| --- | --- | --- | --- | --- |"]
        for n in names:
            m = r["models"].get(n)
            if m is None:
                continue
            R = m["reach"]
            L_.append(f"| {n} | {R['rms_mm']} | {R['horizontal_rms_mm']} | {R['vertical_rms_mm']} | "
                      f"{', '.join(f'{v:+.2f}' for v in m['zero_err_deg'])} |")
        L_.append("")
    return "\n".join(L_) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--records", default="", help="the arm load calibration's kept records (for 'both')")
    a = ap.parse_args()
    jobs = [(s, a.records) for s in range(a.seeds)]
    if len(jobs) == 1:
        rows = [run_seed(jobs[0])]
    else:
        with ProcessPoolExecutor(max_workers=min(4, len(jobs))) as ex:
            rows = list(ex.map(run_seed, jobs))
    (OUT / "arm_datums.json").write_text(json.dumps(rows, indent=1, default=float))
    md = report(rows)
    (OUT / "arm_datums.md").write_text(md)
    print(md)


if __name__ == "__main__":
    main()
