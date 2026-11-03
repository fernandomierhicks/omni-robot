"""The arm robot's modes one after the other, for a robot computer in the loop (design.md 4.9, 8 phase 1C step 2; rpi.md).

Run on the PC (WSL) beside `pi.sh split arm ...` / `pi.sh arm-load`: the robot's nodes run on the other computer, which
samples its own CPU (pi_keepup.py --raw); this script only drives the console and writes down when each phase ran and
what it achieved, so the robot computer's samples can be cut per phase afterwards (arm_load_report.py).

  python3 arm_load.py OUT.json [--phase 120] [--settle 15]

Phases: MANUAL (idle: localization, Nav2 waiting, the console, the arm holding), TIDY (the console's tidy: omni_arm
looks, drives, picks and drops into the bin), PATROL (Nav2 through the house's goal spots, soak.py's route). Each
phase: start / end wall times (this machine's clock), the console's status and /arm/state at both ends.
Scored phases (their own length, --phase does not apply): ROUTE (the eight spots one Go To at a time, --laps laps:
per goal reached or not, time, the true distance to the goal, the localization error, distance driven and top speed
from /ground_truth/odom) and NAVTIDY (--legs times: Go To 0.8 m short of the nearest floor thing still out of the bin,
/sim/objects' truth, then the console's tidy until it ends: arrival error, picks, failures, things in the bin, times).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mode_load import Driver    # noqa: E402
from soak import SPOTS, yaw_of         # noqa: E402


class ArmDriver(Driver):
    def __init__(self):
        super().__init__()
        self.arm = None
        self.objects = None
        self.truth = None              # (x, y, yaw, speed) world
        self.odo = {"dist": 0.0, "vmax": 0.0}
        self.create_subscription(String, "/arm/state", self.on_arm, 5)
        self.create_subscription(String, "/sim/objects", self.on_objects, 5)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 20)

    def on_arm(self, m):
        try:
            self.arm = json.loads(m.data)
        except ValueError:
            pass

    def on_objects(self, m):
        try:
            self.objects = json.loads(m.data)
        except ValueError:
            pass

    def on_truth(self, m):
        p, t = m.pose.pose.position, m.twist.twist.linear
        if self.truth is not None:
            self.odo["dist"] += math.hypot(p.x - self.truth[0], p.y - self.truth[1])
        v = math.hypot(t.x, t.y)
        self.odo["vmax"] = max(self.odo["vmax"], v)
        self.truth = (p.x, p.y, yaw_of(m.pose.pose.orientation), v)

    def to_world(self, x, y):
        ox, oy, oyaw = self.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        return ox + c * x - s * y, oy + s * x + c * y

    def loc_error(self):
        """The console's map pose against the truth, m (None without both)."""
        pose = (self.status or {}).get("pose") or {}
        if pose.get("frame") != "map" or self.truth is None:
            return None
        x, y = self.to_world(pose["x"], pose["y"])
        return round(math.hypot(x - self.truth[0], y - self.truth[1]), 3)

    def goto(self, wx, wy, yaw_deg, name, timeout=150.0):
        """One Go To (world goal) through the console, scored against the truth."""
        self.odo = {"dist": 0.0, "vmax": 0.0}
        t0 = time.monotonic()
        ok, msg, _ = self.cmd("goto", dict(zip(("x", "y", "yaw"), self.to_map(wx, wy, yaw_deg)), name=name))
        if ok:
            t_seen = time.monotonic() + 5.0
            while time.monotonic() < t_seen and (self.status or {}).get("mission") != "GOTO":
                self.spin_for(0.1)
            while time.monotonic() - t0 < timeout and (self.status or {}).get("mission") == "GOTO":
                self.spin_for(0.1)
        timed_out = (self.status or {}).get("mission") == "GOTO"
        if timed_out:
            self.cmd("cancel")
            self.spin_for(1.0)
        task = (self.status or {}).get("task") or msg
        tx, ty = self.truth[:2] if self.truth else (math.nan, math.nan)
        return {"name": name, "goal": [wx, wy], "reached": bool(ok and not timed_out and task.startswith("Arrived")),
                "timed_out": timed_out, "task": task, "time_s": round(time.monotonic() - t0, 1),
                "err_m": round(math.hypot(tx - wx, ty - wy), 3), "loc_err_m": self.loc_error(),
                "dist_m": round(self.odo["dist"], 2), "vmax": round(self.odo["vmax"], 2), "end_wall": time.time()}

    def snap(self):
        st = self.status or {}
        arm = self.arm or {}
        return {"t": time.time(), "mission": st.get("mission"), "pose": st.get("pose"), "patrol": st.get("patrol"),
                "arm": {k: arm.get(k) for k in ("connected", "tidy_on", "tidy", "source", "pose", "out")}}


def route_phase(n: ArmDriver, laps: int) -> dict:
    n.cmd("mission", {"name": "MANUAL"})
    n.spin_for(2.0)
    goals = []
    for lap in range(laps):
        for i, (x, y, yaw) in enumerate(SPOTS):
            g = n.goto(x, y, yaw, f"spot {i + 1}")
            g["lap"] = lap + 1
            goals.append(g)
            print(f"  lap {lap + 1} spot {i + 1}: {'ok ' if g['reached'] else 'NO '} {g['time_s']:5.1f} s, "
                  f"err {g['err_m']:.2f} m, loc {g['loc_err_m']}, {g['dist_m']:.1f} m, vmax {g['vmax']:.2f}  "
                  f"({g['task']})", flush=True)
            if len(goals) >= 3 and all(not g["reached"] and g["dist_m"] < 0.2 for g in goals[-3:]):
                print("  three goals without moving: Nav2 is down, route abandoned", flush=True)
                break
        else:
            continue
        break
    ok = [g for g in goals if g["reached"]]
    return {"goals": goals, "reached": len(ok), "n": len(goals),
            "mean_err_m": round(sum(g["err_m"] for g in ok) / len(ok), 3) if ok else None,
            "vmax": max((g["vmax"] for g in goals), default=0.0),
            "time_s": round(sum(g["time_s"] for g in goals), 1)}


def navtidy_phase(n: ArmDriver, legs: int, stand_off: float = 0.8, tidy_timeout: float = 600.0) -> dict:
    out = []
    for leg in range(legs):
        n.cmd("mission", {"name": "MANUAL"})
        n.spin_for(2.0)
        objs = n.objects or {}
        left = [p for p, inb in zip(objs.get("positions", []), objs.get("in_bin", [])) if not inb]
        if not left or n.truth is None:
            out.append({"leg": leg + 1, "skipped": "nothing left on the floor" if not left else "no truth"})
            break
        rx, ry = n.truth[:2]
        ox, oy = min(left, key=lambda p: math.hypot(p[0] - rx, p[1] - ry))[:2]
        d = math.hypot(ox - rx, oy - ry)
        ux, uy = (ox - rx) / max(d, 1e-6), (oy - ry) / max(d, 1e-6)
        gx, gy = ox - stand_off * ux, oy - stand_off * uy
        bin0 = objs.get("n_in_bin", 0)
        t_drive = time.time()
        g = n.goto(gx, gy, math.degrees(math.atan2(uy, ux)), f"cluster {leg + 1}")
        if not g["reached"]:
            out.append({"leg": leg + 1, "drive": g, "skipped": "navigation failed"})
            print(f"  leg {leg + 1}: navigation failed; tidy skipped ({g['task']})", flush=True)
            continue
        t_tidy = time.time()
        ok, msg, _ = n.cmd("tidy")
        t_seen = time.monotonic() + 5.0
        while ok and time.monotonic() < t_seen and not (n.arm or {}).get("tidy_on"):
            n.spin_for(0.1)
        started = bool((n.arm or {}).get("tidy_on"))
        t0 = time.monotonic()
        while time.monotonic() - t0 < tidy_timeout and (n.arm or {}).get("tidy_on"):
            n.spin_for(0.5)
        timed_out = bool((n.arm or {}).get("tidy_on"))
        if timed_out:
            n.cmd("mission", {"name": "MANUAL"})
            until = time.monotonic() + 30.0
            while (n.arm or {}).get("tidy_on") and time.monotonic() < until:
                n.spin_for(0.2)
        n.spin_for(3.0)
        tidy = (n.arm or {}).get("tidy") or {}
        rec = {"leg": leg + 1, "object": [ox, oy], "start_dist_m": round(d, 2), "drive": g,
               "tidy_cmd": [ok, msg], "tidy": tidy, "started": started, "timed_out": timed_out,
               "tidy_s": round(time.time() - t_tidy, 1),
               "bin_before": bin0, "bin_after": (n.objects or {}).get("n_in_bin"),
               "t_drive": t_drive, "t_tidy": t_tidy, "t_end": time.time()}
        out.append(rec)
        print(f"  leg {leg + 1}: drive {'ok' if g['reached'] else 'NO'} {g['time_s']} s err {g['err_m']} m; tidy "
              f"{tidy.get('picked')} picked {tidy.get('failed')} failed ({tidy.get('reason')}) in {rec['tidy_s']} s; "
              f"bin {bin0} -> {rec['bin_after']}", flush=True)
        if (n.arm or {}).get("tidy_on"):
            print("  arm has not finished stopping; no further driving phases", flush=True)
            return {"legs": out, "arm_still_active": True}
    return {"legs": out}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out")
    ap.add_argument("--phase", type=float, default=120.0, help="s in each mode, after the settle")
    ap.add_argument("--settle", type=float, default=15.0)
    ap.add_argument("--phases", default="MANUAL,TIDY,PATROL")
    ap.add_argument("--wait", type=float, default=300.0, help="s to wait for the console, Nav2 and the map pose")
    ap.add_argument("--laps", type=int, default=2, help="ROUTE: laps of the eight spots")
    ap.add_argument("--legs", type=int, default=2, help="NAVTIDY: drive-then-tidy legs")
    ap.add_argument("--tidy-timeout", type=float, default=600.0, help="NAVTIDY: wall seconds per tidy")
    a = ap.parse_args()
    rclpy.init()
    n = ArmDriver()
    route_name = f"arm load {time.time_ns()}"
    route_created = False
    print("waiting for the console, Nav2, the map pose and the arm ...", flush=True)
    t0 = time.monotonic()
    ready = lambda: (n.status and n.origin and n.cli.service_is_ready() and (n.status.get("caps") or {}).get("nav")  # noqa: E731
                     and (n.status.get("pose") or {}).get("frame") == "map" and (n.arm or {}).get("connected"))
    while time.monotonic() - t0 < a.wait and not ready():
        n.spin_for(1.0)
    res = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "ready_after_s": round(time.monotonic() - t0, 1),
           "ready": bool(ready()), "phase_s": a.phase, "settle_s": a.settle, "phases": {}}
    if not res["ready"]:
        print(f"not ready after {a.wait:g} s: status {bool(n.status)}, origin {bool(n.origin)}, arm {n.arm}", flush=True)
    for ph in a.phases.split(",") if res["ready"] else []:
        print(f"{ph} ...", flush=True)
        if ph in ("ROUTE", "NAVTIDY"):
            s0 = n.snap()
            r = route_phase(n, a.laps) if ph == "ROUTE" else navtidy_phase(n, a.legs, tidy_timeout=a.tidy_timeout)
            res["phases"][ph] = {"start": s0, "end": n.snap(), **r}
            if r.get("arm_still_active"):
                break
            continue
        if ph == "MANUAL":
            ok, msg, _ = n.cmd("mission", {"name": "MANUAL"})
        elif ph == "TIDY":
            n.cmd("mission", {"name": "MANUAL"})
            n.spin_for(2.0)
            ok, msg, _ = n.cmd("tidy")
        elif ph == "PATROL":
            n.cmd("mission", {"name": "MANUAL"})
            n.spin_for(2.0)
            for k, v in (("patrol.mode", "loop"), ("patrol.laps", 0), ("patrol.dwell", 1.0)):
                n.cmd("set", {"key": k, "value": v})
            route_created, msg, _ = n.cmd("route_save", {"name": route_name, "points": [n.to_map(*s) for s in SPOTS]})
            ok, msg, _ = n.cmd("patrol", {"route": route_name}) if route_created else (False, msg, {})
        else:
            raise SystemExit(f"phase: MANUAL, TIDY, PATROL, ROUTE or NAVTIDY, not {ph!r}")
        print(f"  {ok} {msg}", flush=True)
        n.spin_for(a.settle)
        s0 = n.snap()
        n.spin_for(a.phase)
        s1 = n.snap()
        res["phases"][ph] = {"command": [ok, msg], "start": s0, "end": s1}
        print(f"  {ph}: mission {s1['mission']}, arm tidy {s1['arm'].get('tidy')}, patrol {s1['patrol']}", flush=True)
    n.cmd("mission", {"name": "IDLE"})
    if route_created:
        n.cmd("route_delete", {"name": route_name})
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(res, indent=1, default=str))
    print("wrote", a.out, flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
