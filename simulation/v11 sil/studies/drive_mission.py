"""Realistic missions through the firmware in the loop (Nov 1 2026 motor study, phase 5; drive_char.py holds the physics
limits). The v10 arm robot (robot_export.arm_robot: any drive motor, 10 / 15 / 20 kg, pods' springs matched to the mass as
in drive_char) runs the C++ firmware (SilRobot) built with a study header per motor and profile (robot_export.
study_firmware, never lib/omni_fw); a Jetson-side follower sends body velocities over the emulated USB link every 10 ms.
The follower steers on the true pose (a stand-in for good lidar localization), stops at each waypoint (goal to goal),
and with "assist" caps its speed near mapped floor edges (params.edge_speeds) as a speed-filter layer would.

Profiles (firmware ADULT limits in the study header; not adopted defaults):
  adult    ARM's firmware: 2 m/s, the direction-aware acceleration table
  cap2.5 / cap3 / capmax   the same table, speed caps 2.5 / 3 / 4.5 m/s (4.5: above the wheels' ceiling: physics limits)
  fast     capmax with the acceleration table, accel limit, jerk and yaw limits x 1.5

Routes:
  tour       the house tour (bridge_core.TOUR, 19 stops, ~46 m through the doors, over rugs and thresholds)
  tour-arm   the same with the arm-out caps on every leg (ARM_OUT: 1 m/s, half the table's forward acceleration)
  corridor   a 15.2 m (50 ft) open straight, there and back, a 12 mm threshold half way
  shorts     0.5 / 1 / 2 m moves in four body directions (open floor)

  python drive_mission.py M [--motors ...] [--masses 10,20] [--profiles adult,cap3,fast] [--routes tour,corridor]
  python drive_mission.py E [--motors ...] [--masses 10,20] [--profiles adult,capmax]     endurance: the tour looped
                                                                                         from full to the 21 V stop
Results: results/motor_char/M.jsonl / E.jsonl (resumable, as drive_char).
"""

from __future__ import annotations

import argparse
import json
import math
import re
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed

import mujoco
import numpy as np

import drive_char as DC  # noqa: E402  (studies/ on the path; common puts the sim on it)
import robot_export as E  # noqa: E402
import floor as floor_mod  # noqa: E402
from floor import Threshold  # noqa: E402
from omni_control import wrap_angle, world_to_body  # noqa: E402
from robot_model import build_model  # noqa: E402
from sil_link import SimLink  # noqa: E402  (puts ros/src on the path)
from sil_robot import SilRobot  # noqa: E402
from omni_mcu import protocol as PR  # noqa: E402

OUT = DC.OUT
PROFILES = {"adult": (2.0, 1.0), "cap2.5": (2.5, 1.0), "cap3": (3.0, 1.0), "capmax": (4.5, 1.0), "fast": (4.5, 1.5)}
ROUTES = ("tour", "tour-arm", "corridor", "shorts")


def _tag(motor: str, profile: str) -> str:
    return "mc_" + re.sub(r"[^A-Za-z0-9]", "_", f"{motor}_{profile}")


def mission_params(motor: str, mass: str, profile: str, world: str):
    kg, dz = DC.MASSES[mass]
    arm = E.arm_robot("stereo", kg, dz, motor)
    if world == "house":
        import house
        p = house.house_params("v7", assist=False, **arm)
    else:
        feats = (Threshold(8.0, 0.0, yaw_deg=0.0, length=3.0, height=0.012, top=0.03, ramp=0.02),) \
            if world == "corridor" else ()
        p = E.v11_params(**arm, area_size=(24.0, 8.0) if world == "corridor" else (12.0, 12.0), home_xy=(0.0, 0.0),
                         floor_features=feats, wall_height=0.6)
    # the pods' springs matched to the mass (as drive_char's "matched")
    k = p.pod_sprung_mass / E.arm_params(10.0).pod_sprung_mass
    p = p.with_(pod_rate=p.pod_rate * k, pod_preload=p.pod_preload * k)
    cap, sc = PROFILES[profile]
    return E.study_limits(p, cap, sc)


def firmware_for(p, motor: str, profile: str) -> str:
    # always from the open-world 10 kg params: the header differs between worlds only in a feed-hardware comment, which
    # made every house job rewrite it and rebuild the shared variant concurrently (the Nov 2 phase 5 race)
    return E.study_firmware(mission_params(motor, "10", profile, "open"), _tag(motor, profile))


def route(name: str):
    """(world, start (x, y, yaw deg), waypoints [(x, y, yaw deg)], arm_out)."""
    if name in ("tour", "tour-arm"):
        import house
        from bridge_core import TOUR
        return "house", house.START, list(TOUR), name == "tour-arm"
    if name == "corridor":
        return "corridor", (-7.6, 0.0, 0.0), [(7.6, 0.0, 0.0), (-7.6, 0.0, 0.0)], False
    if name == "shorts":
        wps, x, y = [], 0.0, 0.0
        for L in (0.5, 1.0, 2.0):
            for ang in (0.0, 45.0, 90.0, 180.0):
                x += L * math.cos(math.radians(ang))
                y += L * math.sin(math.radians(ang))
                wps.append((round(x, 3), round(y, 3), 0.0))
            x, y = 0.0, 0.0
            wps.append((0.0, 0.0, 0.0))
        return "open", (0.0, 0.0, 0.0), wps, False
    raise ValueError(name)


class Follower:
    """Goal to goal: toward each waypoint at the most the cap and a braking curve allow, turning to its heading; done
    within 5 cm and 5 cm/s and 5 deg. assist: slow to params.edge_speeds within braking distance of a mapped edge."""

    def __init__(self, p, wps, arm_out: bool, assist: bool):
        self.p, self.wps, self.k = p, wps, 0
        self.arm_out, self.assist = arm_out, assist
        cap = E.ARM_OUT["speed_cap"] if arm_out else p.base_max_speed
        self.cap = cap
        self.a_brake = 0.8 * min(p.accel_table[1]) * (E.ARM_OUT["a_cap_frac"] if arm_out else 1.0)
        self.a_cap = E.ARM_OUT["a_cap_frac"] * max(p.accel_table[1]) if arm_out else 0.0
        self.edges = floor_mod.edges(p) if assist else []

    def edge_cap(self, pos, u, sp):
        cap = math.inf
        for e in self.edges:
            reach = max(0.0, (sp * sp - 0.25) / (2 * self.a_brake)) + 0.35
            pts = pos + np.outer(np.linspace(0.0, reach, 8), u)
            if float(np.min(floor_mod.seg_dist(pts, e))) < 0.35:
                h = e[4]
                v = next((vv for hh, vv in self.p.edge_speeds if h <= hh), 0.5)
                cap = min(cap, v)
        return cap

    def command(self, pose, v_now) -> tuple[PR.Command | None, bool]:
        """(the command, whether this waypoint was reached this step)."""
        if self.k >= len(self.wps):
            return PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.0, b=0.0, c=0.0), False
        x, y, yd = self.wps[self.k]
        e = np.array([x - pose[0], y - pose[1]])
        dist = float(np.linalg.norm(e))
        ey = wrap_angle(math.radians(yd) - pose[2])
        reached = dist < 0.05 and abs(ey) < math.radians(5) and float(np.linalg.norm(v_now)) < 0.05
        if reached:
            self.k += 1
            return PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.0, b=0.0, c=0.0), True
        sp = min(self.cap, math.sqrt(2 * self.a_brake * max(dist - 0.01, 0.0)), 2.5 * dist + 0.02)
        u = e / dist if dist > 1e-6 else np.zeros(2)
        if self.edges and dist > 0.05:
            sp = min(sp, self.edge_cap(np.asarray(pose[:2]), u, float(np.linalg.norm(v_now))))
        vb = world_to_body(u * sp, pose[2])
        wz = float(np.clip(3.0 * ey, -3.0, 3.0))
        return PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=float(vb[0]), b=float(vb[1]), c=wz,
                          speed_cap=self.cap if self.arm_out else 0.0, a_cap=self.a_cap), False


def run_mission(motor: str, mass: str, profile: str, route_name: str, assist: bool, loops: int = 1,
                until_stop: bool = False, t_max: float = 5400.0, seed: int = 0):
    world, start, wps, arm_out = route(route_name)
    p = mission_params(motor, mass, profile, world)
    p = p.with_(fw_variant=firmware_for(p, motor, profile))
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    yaw = math.radians(start[2])
    d.qpos[adr:adr + 2] = start[:2]
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(m, d)
    r = SilRobot(m, d, p, seed=seed)
    link = SimLink(r, seed=seed)
    mon = DC.Mon(r)
    all_wps = wps * (10_000 if until_stop else loops)
    fol = Follower(p, all_wps, arm_out, assist)
    t_cmd, t_leg0 = 0.0, 0.0
    legs, stuck, trace = [], 0, []
    v_peak = 0.0
    best = (math.inf, 0.0)               # (distance to the waypoint, when) for the stuck check
    stop_reason = "done"
    dt = m.opt.timestep
    while True:
        t = d.time
        for _ in link.poll(t):
            pass
        if t >= t_cmd - 1e-9:
            t_cmd = t + 0.01
            pose = r.pose()
            vw = d.sensor("base_linvel").data[:2].copy()
            cmd, reached = fol.command(pose, vw)
            if reached:
                legs.append(round(t - t_leg0, 3))
                t_leg0 = t
                best = (math.inf, t)
            elif fol.k < len(all_wps):
                dist = float(np.hypot(all_wps[fol.k][0] - pose[0], all_wps[fol.k][1] - pose[1]))
                if dist < best[0] - 0.02:
                    best = (dist, t)
                elif t - best[1] > 8.0:              # no progress for 8 s: stuck, skip the waypoint
                    stuck += 1
                    fol.k += 1
                    legs.append(None)
                    t_leg0 = t
                    best = (math.inf, t)
            link.send(cmd, t)
            v_peak = max(v_peak, float(np.linalg.norm(vw)))
            if int(t * 100) % 1000 == 0:
                trace.append(dict(t=round(t, 1), soc=round(r.battery.soc, 4), v=round(r.battery.v, 3),
                                  tw=round(max(dv.thermal.t_w for dv in r.drives), 2)))
        r.step()
        mon.step()
        if fol.k >= len(all_wps):
            break
        if until_stop and r.battery.v < DC.V_STOP:
            stop_reason = "21 V stop"
            break
        if d.time > t_max:
            stop_reason = "time limit"
            break
        if mon.flipped:
            stop_reason = "flipped"
            break
    s = mon.summary()
    dist_true = None
    done_legs = [x for x in legs if x is not None]
    return dict(time_s=d.time, legs=legs if not until_stop else None, n_legs=len(legs), n_done=len(done_legs),
                stuck=stuck, v_peak=v_peak, stop=stop_reason, laps=len(legs) / len(wps),
                soc_end=r.battery.soc, wh_total=s["e_total_J"] / 3600, wh_drive=s["e_drive_J"] / 3600,
                trace=trace if until_stop else None, cause=DC.classify(s, stuck == 0 and not s["flipped"]),
                odom_err=None, **s)


def jobs_for(part, a):
    J = []
    for mo in a.motors:
        for ms in a.masses:
            for pr in a.profiles:
                if part == "M":
                    for rt in a.routes:
                        J.append(("mission", (mo, ms, pr, rt, False)))
                        if rt == "tour":
                            J.append(("mission", (mo, ms, pr, rt, True)))
                else:
                    J.append(("endurance", (mo, ms, pr, "tour", True)))
    return J


def run_job(job):
    kind, args = job
    w0 = time.perf_counter()
    try:
        if kind == "mission":
            res = run_mission(*args)
        else:
            res = run_mission(*args, until_stop=True)
        err = None
    except Exception:
        res, err = None, traceback.format_exc()
    return dict(key=json.dumps([kind, list(args)]), kind=kind, args=list(args), res=res, err=err,
                wall_s=time.perf_counter() - w0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=("M", "E"))
    ap.add_argument("--motors", default="AK40-10")
    ap.add_argument("--masses", default="10,15,20")
    ap.add_argument("--profiles", default="adult,cap2.5,cap3,capmax,fast")
    ap.add_argument("--routes", default=",".join(ROUTES))
    ap.add_argument("--workers", type=int, default=DC.WORKERS)
    ap.add_argument("--smoke", action="store_true")
    a = ap.parse_args()
    a.motors, a.masses = a.motors.split(","), a.masses.split(",")
    a.profiles, a.routes = a.profiles.split(","), a.routes.split(",")
    jobs = jobs_for(a.part, a)
    # build every firmware variant once, here, before the workers race to build them
    for mo in a.motors:
        for pr in a.profiles:
            firmware_for(mission_params(mo, a.masses[0], pr, "open"), mo, pr)
    if a.smoke:
        jobs = jobs[:1]
    path = OUT / (f"{a.part}.jsonl" if not a.smoke else f"smoke_{a.part}.jsonl")
    done = set()
    if path.exists() and not a.smoke:
        done = {json.loads(ln)["key"] for ln in path.read_text().splitlines() if ln.strip()
                and json.loads(ln).get("err") is None}
    todo = [j for j in jobs if json.dumps([j[0], list(j[1])]) not in done]
    print(f"part {a.part}: {len(jobs)} jobs, {len(todo)} to run on {a.workers} workers", flush=True)
    w0 = time.perf_counter()
    with open(path, "a" if not a.smoke else "w") as f, ProcessPoolExecutor(a.workers) as ex:
        futs = [ex.submit(run_job, j) for j in todo]
        for k, fu in enumerate(as_completed(futs), 1):
            row = DC._clean(fu.result())
            if row["err"]:
                print(f"  ERROR {row['args']}: {row['err'].strip().splitlines()[-1]}", flush=True)
            f.write(json.dumps(row) + "\n")
            f.flush()
            el = time.perf_counter() - w0
            print(f"  {k}/{len(todo)}  {el / 60:.1f} min", flush=True)


if __name__ == "__main__":
    main()
