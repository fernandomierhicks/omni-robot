"""Soak test: the robot patrols the house for hours; does anything crash, freeze, leak or drift? (WSL, rclpy)

Against a running 'robot' step (start it with the console: `bash omni.sh console --autostart robot`, or the Launcher's
Console button). It saves a route through the eight Nav2 goal spots of the house (omni_sim/nav_goals.py: doorways,
rugs, thresholds, every room), patrols it in a loop through the console's own verbs, and watches:

  every 5 s   the console's status (fresh?), the MCU (silent?), the mission (still PATROL? restarted if not), progress
              (a patrol point reached in the last stall_s?), the robot's true motion (/ground_truth/odom), the
              localization error (the console's map pose against the truth through /sim/origin)
  every 60 s  each ROS process's CPU and memory (/proc: a leak shows as a resident size that keeps growing), the nodes
              still in the graph
  always      every WARN / ERROR on /rosout, by node (the mission node's "feed messages waited" floods among them), the
              console's events (Nav2 skips, faults)

  bash omni.sh py ../ros/tools/soak.py --hours 2 [--out <dir>]      (or python3 with ROS and ~/omni_ws sourced)

Writes <out>/soak_<time>.jsonl (a sample per 5 s, a process line per minute, every problem) and soak_<time>.json
(the summary), prints a line every 5 min and the summary at the end. Ctrl-C ends it early with the summary. It
deletes its route afterwards and leaves the robot in IDLE.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import time
from collections import Counter, deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from omni_interfaces.srv import ConsoleCommand
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ROUTE = "soak test"
# omni_sim/nav_goals.py GOALS, world frame (x, y, yaw deg): every room, through each doorway and over each rug edge
SPOTS = [(-3.0, 3.3, 90), (0.45, 3.2, 90), (4.6, 3.25, 0), (2.5, -2.0, 0), (-3.1, -2.4, -135), (-0.8, -0.4, 60),
         (-5.0, -0.1, 90), (-3.0, 1.25, 0)]
PROCS = ("bridge", "mcu_link", "mode_manager", "mission_node", "rosbridge", "slam_toolbox", "controller_server",
         "planner_server", "bt_navigator", "behavior_server", "velocity_smoother", "collision_monitor", "ekf_node",
         "scan_prep", "tag_slam", "face_node", "supervisor", "smoother_server", "map_server", "costmap")
LEVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Procs:
    """CPU (% of a core since the last sample) and resident memory of the ROS processes, from /proc."""

    def __init__(self):
        self.last = {}
        self.hz = os.sysconf("SC_CLK_TCK")

    def sample(self) -> dict:
        now, out = time.monotonic(), {}
        for pid in (p for p in os.listdir("/proc") if p.isdigit()):
            try:
                cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
                name = next((n for n in PROCS if n in cmd), None)
                if name is None or "soak.py" in cmd:
                    continue
                st = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
                ticks = int(st[11]) + int(st[12])
                rss = int(Path(f"/proc/{pid}/statm").read_text().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 2 ** 20
            except (OSError, ValueError, IndexError):
                continue
            key = f"{name}:{pid}"
            t0, k0 = self.last.get(key, (None, None))
            cpu = None if t0 is None else 100.0 * (ticks - k0) / self.hz / max(now - t0, 1e-6)
            self.last[key] = (now, ticks)
            out[key] = dict(cpu=None if cpu is None else round(cpu, 1), rss_mb=round(rss, 1))
        return out


class Soak(Node):
    def __init__(self, a):
        super().__init__("soak_test")
        self.a = a
        self.cli = self.create_client(ConsoleCommand, "/console/command")
        self.status, self.t_status = None, 0.0
        self.origin = None
        self.truth, self.t_truth = None, 0.0
        self.events = deque(maxlen=50)
        self.logs = Counter()                 # (level, node, first words) -> count
        self.log_first = {}
        self.create_subscription(String, "/console/status", self.on_status, 5)
        self.create_subscription(String, "/console/events", self.on_event, 50)
        self.create_subscription(PoseStamped, "/sim/origin", self.on_origin, LATCHED)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 5)
        self.create_subscription(Log, "/rosout", self.on_log, 200)

    def on_status(self, m):
        try:
            self.status, self.t_status = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    def on_event(self, m):
        try:
            self.events.append(json.loads(m.data))
        except ValueError:
            pass

    def on_origin(self, m):
        self.origin = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def on_truth(self, m):
        p = m.pose.pose
        self.truth, self.t_truth = (p.position.x, p.position.y, yaw_of(p.orientation)), time.monotonic()

    def on_log(self, m: Log):
        if m.level < 30 or m.name == "soak_test":
            return
        key = (LEVEL.get(m.level, str(m.level)), m.name, " ".join(m.msg.split()[:6])[:80])
        self.logs[key] += 1
        self.log_first.setdefault(key, time.time())

    def spin_for(self, s):
        t = time.monotonic() + s
        while time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def cmd(self, verb, args=None, timeout=10.0):
        if not self.cli.service_is_ready():
            return False, "the console's service isn't up"
        f = self.cli.call_async(ConsoleCommand.Request(verb=verb, args=json.dumps(args or {})))
        t = time.monotonic() + timeout
        while not f.done() and time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        r = f.result() if f.done() else None
        return (r.ok, r.message) if r is not None else (False, "no reply")

    def to_map(self, x, y, yaw_deg):
        ox, oy, oyaw = self.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        dx, dy = x - ox, y - oy
        return [c * dx + s * dy, -s * dx + c * dy, math.radians(yaw_deg) - oyaw]

    def truth_in_map(self):
        if self.truth is None or self.origin is None:
            return None
        x, y, yaw = self.truth
        p = self.to_map(x, y, math.degrees(yaw))
        return p


def main():
    ap = argparse.ArgumentParser(description="patrol the house for hours and watch for crashes, freezes, leaks, drift")
    ap.add_argument("--hours", type=float, default=2.0)
    ap.add_argument("--out", default=str(Path.home() / "omni_soak"))
    ap.add_argument("--dwell", type=float, default=2.0, help="s at each point")
    ap.add_argument("--stall", type=float, default=180.0, help="s without reaching a point = stuck")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    jl = (out / f"soak_{stamp}.jsonl").open("w")
    rclpy.init()
    n = Soak(a)
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))

    def write(kind, **kw):
        jl.write(json.dumps(dict(kind=kind, t=round(time.time(), 2), **kw)) + "\n")
        jl.flush()

    problems = Counter()

    def problem(what, **kw):
        problems[what] += 1
        write("problem", what=what, **kw)
        print(f"[soak {time.strftime('%H:%M:%S')}] PROBLEM {what} {kw if kw else ''}", flush=True)

    print("waiting for the console, the sim's origin and the truth ...", flush=True)
    t0 = time.monotonic()
    while (n.status is None or n.origin is None or n.truth is None or not n.cli.service_is_ready()) and \
            time.monotonic() - t0 < 300:
        n.spin_for(0.5)
    if n.status is None or n.origin is None:
        raise SystemExit("no console status or /sim/origin: is the 'robot' step running?")
    while time.monotonic() - t0 < 300 and not ((n.status.get("caps") or {}).get("nav") and
                                                (n.status.get("pose") or {}).get("frame") == "map"):
        n.spin_for(1.0)                             # Nav2 active and the robot localized on the map
    pts = [n.to_map(*s) for s in SPOTS]
    for k, v in (("patrol.mode", "loop"), ("patrol.laps", 0), ("patrol.dwell", a.dwell)):
        n.cmd("set", {"key": k, "value": v})
    ok, msg = n.cmd("route_save", {"name": ROUTE, "points": pts})
    if not ok:
        raise SystemExit(f"route_save: {msg}")
    n.cmd("mission", {"name": "MANUAL"})
    n.spin_for(1.0)
    ok, msg = n.cmd("patrol", {"route": ROUTE})
    print(f"patrol: {msg}", flush=True)
    write("start", hours=a.hours, points=pts, msg=msg)

    t_start, t_end = time.monotonic(), time.monotonic() + 3600 * a.hours
    t_sample = t_proc = t_print = 0.0
    last_progress = (None, time.monotonic())       # ((lap, i), when it last changed)
    moves = deque()                                 # (t, x, y) of the truth, for "moving at all"
    procs = Procs()
    rss0 = {}
    loc_err, points_reached, restarts, laps = [], 0, 0, 0
    max_lap = 0
    nodes0 = None
    while not stop["now"] and time.monotonic() < t_end:
        n.spin_for(0.2)
        now = time.monotonic()
        if now - t_sample < 5.0:
            continue
        t_sample = now
        st = n.status or {}
        age = now - n.t_status
        mcu = st.get("mcu") or {}
        row = dict(elapsed=round(now - t_start, 1), status_age=round(age, 2), mission=st.get("mission"),
                   task=str(st.get("task", ""))[:80], mcu_age=mcu.get("age"), mcu_mode=mcu.get("mode"),
                   faults=[f.get("name") for f in mcu.get("faults") or []])
        if age > 3.0:
            problem("console status stale", age=round(age, 1))
        if mcu and (mcu.get("age") or 0) > 1.5:
            problem("MCU silent", age=mcu.get("age"))
        pa = st.get("patrol")
        if st.get("mission") != "PATROL" and age < 3.0:
            problem("patrol ended", mission=st.get("mission"), task=row["task"],
                    events=[e.get("text") for e in list(n.events)[-5:]])
            n.cmd("mission", {"name": "MANUAL"})
            n.spin_for(1.0)
            ok, msg = n.cmd("patrol", {"route": ROUTE})
            restarts += 1
            last_progress = (None, now)
        elif pa:
            key = (pa.get("lap"), pa.get("i"))
            if key != last_progress[0]:
                if last_progress[0] is not None:
                    points_reached += 1
                last_progress = (key, now)
                max_lap = max(max_lap, pa.get("lap") or 0)
            elif now - last_progress[1] > a.stall:
                problem("stuck: no patrol point reached", s=round(now - last_progress[1]), at=key, task=row["task"])
                last_progress = (key, now)
        if n.truth is not None:
            moves.append((now, n.truth[0], n.truth[1]))
            while moves and now - moves[0][0] > 120.0:
                moves.popleft()
            if now - t_start > 180 and moves and now - moves[0][0] > 100 and \
                    math.hypot(moves[-1][1] - moves[0][1], moves[-1][2] - moves[0][2]) < 0.1:
                problem("not moving for 2 min", task=row["task"])
                moves.clear()
            if now - n.t_truth > 5.0:
                problem("sim truth stale (the bridge froze?)", age=round(now - n.t_truth, 1))
        pose, tm = st.get("pose"), n.truth_in_map()
        if pose and pose.get("frame") == "map" and tm is not None:
            e = math.hypot(pose["x"] - tm[0], pose["y"] - tm[1])
            loc_err.append(e)
            row["loc_err_cm"] = round(100 * e, 1)
            row["yaw_err_deg"] = round(abs(math.degrees(wrap(pose["yaw"] - tm[2]))), 2)
            if e > 0.5:
                problem("localization off by > 0.5 m", err_m=round(e, 2))
        write("sample", **row)
        if now - t_proc > 60.0:
            t_proc = now
            ps = procs.sample()
            names = set(n.get_node_names())
            if nodes0 is None and now - t_start > 60:
                nodes0 = names
            elif nodes0 is not None:
                gone = sorted(nodes0 - names)
                if gone:
                    problem("nodes left the graph", nodes=gone)
                    nodes0 = names
            for k, v in ps.items():
                rss0.setdefault(k, v["rss_mb"])
            write("procs", procs=ps, nodes=len(names))
        if now - t_print > 300.0:
            t_print = now
            e = sorted(loc_err[-60:])
            print(f"[soak {time.strftime('%H:%M:%S')}] {(now - t_start) / 3600:.2f} h: {st.get('mission')} "
                  f"lap {max_lap}, {points_reached} points, {restarts} restarts, loc err median "
                  f"{100 * e[len(e) // 2] if e else float('nan'):.1f} cm, problems {sum(problems.values())}", flush=True)

    # the end: the summary, the route deleted, the robot in IDLE
    dur = time.monotonic() - t_start
    ps = procs.sample()
    growth = {k: round(v["rss_mb"] - rss0.get(k, v["rss_mb"]), 1) for k, v in ps.items()}
    e = sorted(loc_err)
    pct = lambda q: round(100 * e[min(len(e) - 1, int(q * len(e)))], 1) if e else None  # noqa: E731
    summary = dict(hours=round(dur / 3600, 2), laps=max_lap, points_reached=points_reached, patrol_restarts=restarts,
                   problems=dict(problems), loc_err_cm=dict(median=pct(0.5), p95=pct(0.95), max=pct(0.999)),
                   rss_growth_mb=dict(sorted(growth.items(), key=lambda kv: -abs(kv[1]))[:8]),
                   rss_now_mb={k: v["rss_mb"] for k, v in sorted(ps.items())},
                   warnings={f"{lv} [{nd}] {msg}": c for (lv, nd, msg), c in n.logs.most_common(25)})
    write("summary", **summary)
    n.cmd("mission", {"name": "IDLE"})
    n.cmd("route_delete", {"name": ROUTE})
    print(json.dumps(summary, indent=1), flush=True)
    (out / f"soak_{stamp}.json").write_text(json.dumps(summary, indent=1))
    jl.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
