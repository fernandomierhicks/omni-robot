"""The sim's A2M8 against the bench (test 1): the same timing numbers from a still robot in the v11 house.

    C:\\Users\\ferna\\.venvs\\omni-robot-sim\\Scripts\\python.exe characterization/lidar/sim_check.py [--seconds 20]

For each mode: turn rate, samples a turn, the angle step and backward steps, a turn's first sample's angle, the
range quantization, the delay from a turn's last sample to its handover above the fastest delivery (the bench's
"turn ready"), and sllidar's stamp-to-stamp error. Prints them next to results/timing_2026-11-16.json's.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "..", "simulation", "v11 sil")
sys.path.insert(0, SIM)
os.chdir(SIM)

import house  # noqa: E402
from sensors import A2M8_MODES, Sensors, SensorConfig  # noqa: E402


def sim_mode(mode: str, seconds: float, spot: str = "9") -> dict:
    drv = house.Driver(house.house_params("v7", assist=False))
    drv.teleport(house.SPOTS[spot])
    sn = Sensors(drv.r, SensorConfig.parts("BNO085", "A2M8", lidar_mode=mode, seed=5, lidar_cast_dt=0.002,
                                           lidar_stamp="sllidar"))      # the bench measured what sllidar would stamp
    c = sn.cfg
    raw_ang, scans = [], []
    orig = sn._turn

    def keep(T, A, Rr, E):                    # the turn's samples in time order, before the re-binning
        raw_ang.append(np.degrees(-A))        # the device's angles are clockwise (the sim's laser frame's aren't)
        return orig(T, A, Rr, E)
    sn._turn = keep
    while drv.d.time < seconds:
        drv.step(np.zeros(2), 0.0, False)
        rd = sn.step()
        if rd.scan is not None:
            scans.append(rd.scan)
    scans, raw_ang = scans[1:], raw_ang[1:len(scans)]
    rate = c.lidar_rate
    tx = c.lidar_packet_bytes * 10 / c.lidar_baud
    ready = np.array([(s.t_ready - s.t_end - tx - c.lidar_usb[0]) * 1e3 for s in scans])
    stamps = np.array([s.stamp for s in scans])
    true_period = np.diff([s.t_end for s in scans])
    stamp_err = np.abs(np.diff(stamps) - true_period) * 1e3
    steps = np.concatenate([(np.diff(a) + 180) % 360 - 180 for a in raw_ang])
    first = np.array([((a[0] + 180) % 360) - 180 for a in raw_ang])
    r = np.concatenate([s.rng[np.isfinite(s.rng)] for s in scans])
    return dict(hz=round(rate / sn.n_scan, 3), samples_per_turn=sn.n_scan,
                step_mean_deg=round(float(steps.mean()), 4), step_p1_p99=np.round(np.percentile(steps, [1, 99]), 2).tolist(),
                backward_per_turn=round(float((steps < 0).sum() / len(raw_ang)), 1),
                first_deg=np.round(np.percentile(first, [0, 50, 100]), 2).tolist(),
                ready_ms=np.round(np.percentile(ready, [0, 50, 95, 100]), 2).tolist(),
                stamp_period_err_ms=np.round(np.percentile(stamp_err, [50, 95, 100]), 2).tolist(),
                valid=round(float(np.mean([np.isfinite(s.rng).mean() for s in scans])), 3),
                range_m_median=round(float(np.median(r)), 3))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--bench", default=os.path.join(HERE, "results", "timing_2026-11-16.json"))
    a = ap.parse_args()
    bench = {r["mode"]: r for r in json.load(open(a.bench))["runs"] if r["pwm"] == 660 and r["seconds"] <= 60}
    out = {}
    for mode in A2M8_MODES:
        s = sim_mode(mode, a.seconds)
        b = bench.get(mode, {})
        out[mode] = dict(sim=s, bench=dict(
            hz=b.get("hz"), samples_per_turn=b.get("samples_per_turn", {}).get("mean"),
            step_mean_deg=b.get("angle", {}).get("step_mean_deg"),
            step_p1_p99=b.get("angle", {}).get("step_p1_p99_deg"),
            backward_per_turn=b.get("angle", {}).get("backward_steps_per_turn"),
            first_deg=[b["angle"]["first_sample_deg"][k] for k in ("p0", "p50", "max")] if b else None,
            ready_ms=[b["turn_ready_ms"][k] for k in ("p0", "p50", "p95", "max")] if b else None,
            stamp_period_err_ms=[b["stamp_period_err_ms"][k] for k in ("p50", "p95", "max")] if b else None,
            valid=b.get("valid_share"), range_m_median=b.get("range_m_median")))
        print(f"\n{mode}")
        for k in out[mode]["sim"]:
            print(f"  {k:<22} sim {str(out[mode]['sim'][k]):<34} bench {out[mode]['bench'].get(k)}")
    path = os.path.join(HERE, "results", f"sim_check_{time.strftime('%Y-%m-%d')}.json")      # one per day: earlier ones kept
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    print(f"\n-> {path}")


if __name__ == "__main__":
    main()
