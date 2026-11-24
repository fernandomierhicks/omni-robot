"""The lidar drive guard's sim runs (ros/tools/guard_drive.py) scored on the truth: the house's static geometry (the
sim's own model) ray-cast from each logged pose along the run's way, less the robot's body ring (RIM).

    python studies/guard_drive_clear.py studies/results/guard_drive/rc_runs_2026-11-23.json [--plot out.png]

Per run: the top speed, acceleration from rest, where the speed started to fall (true gap), the speed at 50 / 20 /
10 / 5 cm from the wall, the peak deceleration, the final gap and any contact. Writes <runs>_summary.json beside.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import house  # noqa: E402
from robot_model import build_model  # noqa: E402

RIM = 0.225       # the sim robot's body ring (inward rays on robot_export.arm_params(): 0.225 all round, z 0.07);
                  # its stowed arm reaches ~0.24 ahead (an unguarded crash stopped its centre 0.24 m off the wall)
COLS = ("t", "x", "y", "yaw", "vx", "vy", "wz", "v_cmd", "v_meas", "limit", "free", "status")


class House:
    def __init__(self):
        self.m = build_model(house.house_params("v8"))
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        static = self.m.body_weldid[self.m.geom_bodyid] == 0
        for g in range(self.m.ngeom):
            if not static[g]:
                self.m.geom_group[g] = 5                   # the robot and loose things out of the rays
        self.mask = np.array([1, 1, 1, 1, 1, 0], np.uint8)
        self.gid = np.zeros(1, np.int32)

    def ray(self, x, y, z, ang):
        r = mujoco.mj_ray(self.m, self.d, np.array([x, y, z]), np.array([math.cos(ang), math.sin(ang), 0.0]),
                          self.mask, 1, -1, self.gid)
        return r if r >= 0 else 9.0

    def gap_ahead(self, x, y, ang, z=0.163):
        """Rim to the nearest thing in the way: rays from across the robot's width, each less the rim there."""
        best = 9.0
        for off in np.linspace(-RIM * 0.95, RIM * 0.95, 9):
            px, py = x - off * math.sin(ang), y + off * math.cos(ang)
            best = min(best, self.ray(px, py, z, ang) - math.sqrt(RIM * RIM - off * off))
        return best

    def clearance(self, x, y, z=0.163):
        return min(self.ray(x, y, z, a) for a in np.radians(np.arange(0, 360, 3))) - RIM

    def nearest(self, x, y, z=0.163):
        """Rim to the nearest thing all round and the world direction to it (the wall's normal, for a wall)."""
        a = np.radians(np.arange(0, 360, 2))
        r = np.array([self.ray(x, y, z, ang) for ang in a])
        i = int(np.argmin(r))
        return float(r[i]) - RIM, float(a[i])


def analyse_slide(h: House, r: dict) -> dict:
    """A slide run (guard_drive.py slide:speed:deg): the truth speed split along / into the nearest surface against
    the true gap all round."""
    T = np.array(r["trace"], float)
    c = {k: T[:, i] for i, k in enumerate(COLS)}
    t = c["t"] - c["t"][0]
    k = max(1, int(round(0.1 / np.median(np.diff(t)))))

    def smooth(y):
        yp = np.concatenate([np.full(k, y[0]), y, np.full(k, y[-1])])
        return np.convolve(yp, np.ones(k) / k, mode="same")[k:-k]
    vx, vy = np.gradient(smooth(c["x"]), t), np.gradient(smooth(c["y"]), t)
    near = [h.nearest(c["x"][i], c["y"][i]) for i in range(len(t))]
    gap = np.array([g for g, _ in near])
    into = np.array([vx[i] * math.cos(a) + vy[i] * math.sin(a) for i, (_, a) in enumerate(near)])
    speed = np.hypot(vx, vy)
    along = np.sqrt(np.maximum(speed ** 2 - into ** 2, 0.0))
    low = np.array([h.clearance(c["x"][i], c["y"][i], 0.03) for i in range(0, len(t), 5)])
    out = dict(label=r["label"], input=r.get("input"), slide=True, approach_deg=r.get("approach_deg"),
               top=round(float(speed.max()), 2), min_gap_scan_plane=round(float(gap.min()), 3),
               min_gap_3cm=round(float(low.min()), 3), final_gap=round(float(gap[-1]), 3))
    for d in (0.5, 0.2, 0.1, 0.05):
        i = np.where(gap <= d)[0]
        out[f"at_{int(d * 100)}cm"] = (None if not len(i) else
                                      dict(along=round(float(along[i[0]]), 2), into=round(float(into[i[0]]), 2)))
    w = gap <= 0.2                                          # sliding: within 20 cm of something
    if w.any():
        dt = np.gradient(t)
        out["slid_m_within_20cm"] = round(float((along * dt)[w].sum()), 2)
        out["along_within_20cm_mean"] = round(float(along[w].mean()), 2)
        out["into_within_10cm_max"] = round(float(into[gap <= 0.1].max()), 2) if (gap <= 0.1).any() else None
    touch = np.where(gap <= 0.003)[0]
    out["contact_speed_into"] = round(float(into[touch[0]]), 2) if len(touch) else None
    out["profile"] = [[round(float(gap[i]), 3), round(float(along[i]), 3), round(float(into[i]), 3),
                       round(float(t[i]), 3)] for i in range(0, len(t), 2)]
    return out


def analyse(h: House, r: dict) -> dict:
    T = np.array(r["trace"], float)
    if len(T) < 5:
        return dict(label=r["label"], error="no trace")
    if r.get("slide"):
        return analyse_slide(h, r)
    c = {k: T[:, i] for i, k in enumerate(COLS)}
    t = c["t"] - c["t"][0]
    wd = r["world_dir"]
    # truth speed along the way, smoothed over 0.1 s
    s = (c["x"] - c["x"][0]) * math.cos(wd) + (c["y"] - c["y"][0]) * math.sin(wd)
    k = max(1, int(round(0.1 / np.median(np.diff(t)))))

    def smooth(y):                                          # a 0.1 s mean, the ends padded with their values
        yp = np.concatenate([np.full(k, y[0]), y, np.full(k, y[-1])])
        return np.convolve(yp, np.ones(k) / k, mode="same")[k:-k]
    v = np.gradient(smooth(s), t)
    acc = np.gradient(smooth(v), t)
    out = dict(label=r["label"], input=r.get("input"), bypass=r.get("bypass"), top=round(float(v.max()), 2))
    ipk = int(np.argmax(v))
    out["accel_peak"] = round(float(acc[:max(ipk, 2)].max()), 2)
    reach = np.where(v >= 0.95 * v.max())[0]
    out["t_to_95pct"] = round(float(t[reach[0]]), 2) if len(reach) else None
    if r.get("accel"):
        out["profile"] = [[round(float(t[i]), 3), round(float(v[i]), 3)] for i in range(0, len(t), 2)]
        return out
    gap = np.array([h.gap_ahead(c["x"][i], c["y"][i], wd) for i in range(len(t))])
    low = np.array([h.gap_ahead(c["x"][i], c["y"][i], wd, 0.03) for i in range(0, len(t), 5)])
    out["min_gap_scan_plane"] = round(float(gap.min()), 3)
    out["min_gap_3cm"] = round(float(low.min()), 3)
    ib = ipk + int(np.argmax(v[ipk:] < 0.95 * v[ipk])) if ipk < len(v) - 1 else ipk
    out["brake_start_gap"] = round(float(gap[ib]), 2)
    out["decel_peak"] = round(float(-acc[ipk:].min()), 2)
    for d in (0.5, 0.2, 0.1, 0.05):
        i = np.where(gap <= d)[0]
        out[f"v_at_{int(d * 100)}cm"] = round(float(v[i[0]]), 3) if len(i) else None
    out["final_gap"] = round(float(gap[-1]), 3)
    touch = np.where(gap <= 0.003)[0]
    out["contact_speed"] = round(float(v[touch[0]]), 2) if len(touch) else None
    out["back_off"] = r.get("back_off")
    out["profile"] = [[round(float(gap[i]), 3), round(float(v[i]), 3), round(float(t[i]), 3)]
                      for i in range(0, len(t), 2) if gap[i] < 3.0]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs")
    a = ap.parse_args()
    h = House()
    R = json.load(open(a.runs))
    res = [analyse(h, r) for r in R]
    for x in res:
        print(json.dumps({k: v for k, v in x.items() if k != "profile"}))
    json.dump(res, open(a.runs.replace(".json", "_summary.json"), "w"))


if __name__ == "__main__":
    main()
