"""CPU per robot mode (design.md 4.9): against a running 'robot' step, the console's modes one after the other, every
process's CPU in each. The real peak is the busiest mode with everything else idling beside it, not the sum of the
scored steps.

  bash omni.sh py ../ros/tools/mode_load.py [--phase 120] [--out ~/omni_load]   (WSL, with the robot step running)

Phases (each --phase s after a settle): MANUAL (the stack idle: localization, Nav2 waiting, the console), PATROL (Nav2
through the house's eight goal spots, soak.py's route, at the console's Go To settings) and CATCH (the house's thrower,
the catch code playing). Every 10 s window: each process's % of one core and memory (cpu_watch.py's groups; "SIM" is
the sim). Writes <out>/mode_load_<time>.json: per phase, per group median / max, the robot's total (SIM excluded), and
what the phase achieved (patrol points reached, throws and catches). Leaves the robot in IDLE, its route deleted.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from omni_interfaces.srv import ConsoleCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpu_watch import HZ, snapshot          # noqa: E402
from soak import SPOTS, yaw_of              # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ROUTE = "mode load"
PLAYER = "Load test"


class Driver(Node):
    def __init__(self):
        super().__init__("mode_load")
        self.cli = self.create_client(ConsoleCommand, "/console/command")
        self.status, self.origin = None, None
        self.create_subscription(String, "/console/status", self.on_status, 5)
        self.create_subscription(PoseStamped, "/sim/origin", self.on_origin, LATCHED)

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def on_origin(self, m):
        self.origin = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def spin_for(self, s):
        t = time.monotonic() + s
        while time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def cmd(self, verb, args=None, timeout=10.0):
        f = self.cli.call_async(ConsoleCommand.Request(verb=verb, args=json.dumps(args or {})))
        t = time.monotonic() + timeout
        while not f.done() and time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        r = f.result() if f.done() else None
        return (r.ok, r.message, json.loads(r.data or "{}")) if r else (False, "no reply", {})

    def to_map(self, x, y, yaw_deg):
        ox, oy, oyaw = self.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        return [c * (x - ox) + s * (y - oy), -s * (x - ox) + c * (y - oy), math.radians(yaw_deg) - oyaw]

    def board(self):
        _, _, d = self.cmd("stats")
        return next((p for p in (d.get("board") or []) if p.get("player") == PLAYER), {})


def sample_phase(n: Driver, seconds: float, window: float = 10.0):
    """cpu_watch.py's windows for `seconds`, spinning the node between snapshots."""
    rows = []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        a, t0 = snapshot(), time.monotonic()
        n.spin_for(window)
        b, t1 = snapshot(), time.monotonic()
        use, mem = {}, {}
        for pid, (g, ticks, rss) in b.items():
            if pid in a and g != "python3":
                use[g] = use.get(g, 0.0) + (ticks - a[pid][1]) / HZ / (t1 - t0) * 100.0
                mem[g] = mem.get(g, 0) + rss
        use.pop("mode_load.py", None)
        mem.pop("mode_load.py", None)
        rows.append({"t": time.time(), "cpu": {k: round(v, 1) for k, v in use.items() if v >= 0.05},
                     "rss_mb": {k: round(v / 2**20) for k, v in mem.items()}})
    return rows


def summarize(rows):
    groups = sorted({g for r in rows for g in r["cpu"]})
    stats = {g: {"median": round(statistics.median([r["cpu"].get(g, 0.0) for r in rows]), 1),
                 "max": round(max(r["cpu"].get(g, 0.0) for r in rows), 1),
                 "rss_mb": max(r["rss_mb"].get(g, 0) for r in rows)} for g in groups}
    tot = [sum(v for g, v in r["cpu"].items() if g != "SIM") for r in rows]
    return {"windows": len(rows), "robot_total_median": round(statistics.median(tot), 1),
            "robot_total_max": round(max(tot), 1), "groups": stats}


def main():
    ap = argparse.ArgumentParser(description="CPU per robot mode against a running robot step")
    ap.add_argument("--phase", type=float, default=120.0, help="s sampled in each mode")
    ap.add_argument("--settle", type=float, default=20.0, help="s after a mode starts before sampling")
    ap.add_argument("--out", default=str(Path.home() / "omni_load"))
    ap.add_argument("--label", default="", help="what ran (e.g. glue:=cpp)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Driver()
    print("waiting for the console, the sim's origin, Nav2 and the map pose ...", flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 300 and not (n.status and n.origin and n.cli.service_is_ready() and
                                               (n.status.get("caps") or {}).get("nav") and
                                               (n.status.get("pose") or {}).get("frame") == "map"):
        n.spin_for(1.0)
    if not (n.status and n.origin):
        raise SystemExit("no console status or /sim/origin: is the 'robot' step running?")
    result = {"label": a.label, "phase_s": a.phase, "started": time.strftime("%Y-%m-%d %H:%M:%S"), "phases": {}}

    print("MANUAL (idle) ...", flush=True)
    n.cmd("mission", {"name": "MANUAL"})
    n.spin_for(a.settle)
    rows = sample_phase(n, a.phase)
    result["phases"]["MANUAL"] = {**summarize(rows), "raw": rows}

    print("PATROL ...", flush=True)
    for k, v in (("patrol.mode", "loop"), ("patrol.laps", 0), ("patrol.dwell", 1.0)):
        n.cmd("set", {"key": k, "value": v})
    ok, msg, _ = n.cmd("route_save", {"name": ROUTE, "points": [n.to_map(*s) for s in SPOTS]})
    ok, msg, _ = n.cmd("patrol", {"route": ROUTE})
    print(f"  patrol: {msg}", flush=True)
    n.spin_for(a.settle)
    p0 = dict((n.status or {}).get("patrol") or {})
    rows = sample_phase(n, a.phase)
    p1 = dict((n.status or {}).get("patrol") or {})
    result["phases"]["PATROL"] = {**summarize(rows), "patrol_start": p0, "patrol_end": p1,
                                  "mission_end": (n.status or {}).get("mission"), "raw": rows}

    print("CATCH ...", flush=True)
    n.cmd("mission", {"name": "MANUAL"})
    n.spin_for(2.0)
    n.cmd("player_set", {"name": PLAYER})
    b0 = n.board()
    ok, msg, _ = n.cmd("mission", {"name": "CATCH"})
    print(f"  catch: {msg}", flush=True)
    n.spin_for(a.settle)
    rows = sample_phase(n, a.phase)
    b1 = n.board()
    result["phases"]["CATCH"] = {**summarize(rows), "board_start": b0, "board_end": b1,
                                 "mission_end": (n.status or {}).get("mission"), "raw": rows}

    n.cmd("mission", {"name": "IDLE"})
    n.cmd("route_delete", {"name": ROUTE})
    path = out / f"mode_load_{time.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps(result, indent=1))
    for ph, r in result["phases"].items():
        print(f"== {ph}: robot total median {r['robot_total_median']}% max {r['robot_total_max']}% of one core")
        for g, s in sorted(r["groups"].items(), key=lambda kv: -kv[1]["median"])[:14]:
            print(f"   {g:34s} {s['median']:6.1f} {s['max']:6.1f}  {s['rss_mb']:5d} MB")
    print(f"patrol {result['phases']['PATROL']['patrol_start']} -> {result['phases']['PATROL']['patrol_end']}")
    print(f"catch board {result['phases']['CATCH']['board_start']} -> {result['phases']['CATCH']['board_end']}")
    print("wrote", path, flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
