"""The AI control path end to end in the sim (ai_robot_architecture.md 8): the robot's console, omni_arm and the sim
in WSL (omni.sh arm), omni_mcp here on Windows. No model: the checks call the tools themselves, through the real MCP
server (server.py as a child process, MCP JSON-RPC over its stdio) where it matters, else through its Robot class.

  spin    5 turns at "max": the sim's true yaw (/ground_truth/odom, unwrapped) 1800 +- 3 deg, the duration within 15%
          of the robot's estimate
  lease   the MCP process killed mid-spin: the robot stops (true yaw rate < 5 deg/s) within the lease TTL + 0.5 s
  dup     the same request_id twice: one task, one turn
  owner   the page takes control: a move is refused (owner_conflict)
  goto    Nav2 to a map pose and back, with its evidence; an unknown place refused
  blocked a goal Nav2 can't reach: the task FAILED with Nav2's reason, the robot still
  navlease the MCP process killed while Nav2 drives: the goal cancelled, the robot stops within the TTL + ~1 s
  pick    (objects:=colours) "go get the purple square object": find (look round, closer if the shape is a guess),
          pick; only cube40_purple goes into the bin (/sim/objects, the sim's truth); "the purple round one": none
  fetch   "bring me the red cube": find, pick (only cube30 into the bin), goto back to where it started
  restart (--restart-cmd, not by default) the console's mission node restarted mid-goto: the task INTERRUPTED,
          nothing resumes, the robot stops (ros/tools/restart_console.sh)

    wsl ... ros/omni.sh arm objects:=colours viewer:=false        (the robot side; separate ports beside Fernando's)
    & C:\\Users\\ferna\\.venvs\\omni-mcp\\Scripts\\python.exe ros/tools/ai_smoke.py [--only spin,lease,...]
      [--host 127.0.0.1 --rosbridge-port 9090 --video-port 8081] [--restart-cmd "..."] [--out result.json]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "omni_mcp"))

from lease import LeaseKeeper  # noqa: E402
from server import Robot  # noqa: E402

PY = sys.executable
SERVER = HERE / "omni_mcp" / "server.py"
TERMINAL = ("SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT", "INTERRUPTED")


class Truth:
    """The sim's true heading, unwrapped, sampled at 100 Hz from /ground_truth/odom (rosbridge keeps the newest)."""

    def __init__(self, rb):
        self.rb = rb
        rb.subscribe("/ground_truth/odom", "nav_msgs/msg/Odometry")
        self.yaw = None
        self.samples = []                   # (time.monotonic(), unwrapped yaw rad, sim time s: the message's stamp)
        self.last = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            m, age = self.rb.newest("/ground_truth/odom")
            if m is not None and m is not self.last:
                self.last = m
                q = m["pose"]["pose"]["orientation"]
                y = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
                if self.yaw is None:
                    self.yaw = y
                else:
                    self.yaw += math.remainder(y - (self.yaw % (2 * math.pi)), 2 * math.pi)
                st = m["header"]["stamp"]
                p = m["pose"]["pose"]["position"]
                self.samples.append((time.monotonic(), self.yaw, st["sec"] + st["nanosec"] * 1e-9, p["x"], p["y"]))
                self.samples = self.samples[-20000:]
            time.sleep(0.01)

    def rate(self, window: float = 0.3) -> float:
        """deg/s of sim time over the last window of wall time (the sim may run slower than real time)."""
        s = [x for x in self.samples if x[0] > time.monotonic() - window]
        if len(s) < 2:
            return float("nan")
        return math.degrees((s[-1][1] - s[0][1]) / max(1e-3, s[-1][2] - s[0][2]))

    def speed(self, window: float = 0.5) -> float:
        """m/s of sim time over the last window of wall time."""
        s = [x for x in self.samples if x[0] > time.monotonic() - window]
        if len(s) < 2:
            return float("nan")
        return math.hypot(s[-1][3] - s[0][3], s[-1][4] - s[0][4]) / max(1e-3, s[-1][2] - s[0][2])

    def max_rate(self, t0: float, t1: float, window: float = 0.2) -> float | None:
        """The fastest true turn (deg/s) over sim windows of this length between wall times t0 and t1."""
        s = [x for x in self.samples if t0 <= x[0] <= t1]
        best, j = None, 0
        for i in range(len(s)):
            while s[i][2] - s[j][2] > window:
                j += 1
            if s[i][2] - s[j][2] >= 0.5 * window:
                r = abs(math.degrees((s[i][1] - s[j][1]) / (s[i][2] - s[j][2])))
                best = r if best is None else max(best, r)
        return None if best is None else round(best, 1)

    def span(self, t0: float, t1: float, tol_deg: float = 1.0) -> float | None:
        """Sim seconds the turn took between wall times t0 and t1: from leaving the start by tol_deg to coming within
        tol_deg of where it ended."""
        s = [x for x in self.samples if t0 <= x[0] <= t1]
        if len(s) < 2:
            return None
        y0, y1, tol = s[0][1], s[-1][1], math.radians(tol_deg)
        a = next((x[2] for x in s if abs(x[1] - y0) > tol), None)
        b = next((x[2] for x in s if abs(x[1] - y1) < tol), None)
        return None if a is None or b is None else round(b - a, 2)


class McpChild:
    """server.py as Claude Code runs it: a child process speaking MCP JSON-RPC on its stdio."""

    def __init__(self, env: dict):
        self.p = subprocess.Popen([PY, str(SERVER)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, env={**os.environ, **env}, text=True, bufsize=1)
        self.n = 0
        self.rpc("initialize", dict(protocolVersion="2025-06-18", capabilities={},
                                    clientInfo=dict(name="ai_smoke", version="1")))
        self.p.stdin.write(json.dumps(dict(jsonrpc="2.0", method="notifications/initialized")) + "\n")

    def rpc(self, method: str, params: dict) -> dict:
        self.n += 1
        self.p.stdin.write(json.dumps(dict(jsonrpc="2.0", id=self.n, method=method, params=params)) + "\n")
        while True:
            line = self.p.stdout.readline()
            if not line:
                raise RuntimeError("the MCP server exited")
            m = json.loads(line)
            if m.get("id") == self.n:
                if "error" in m:
                    raise RuntimeError(m["error"])
                return m["result"]

    def tool(self, name: str, **args) -> dict:
        r = self.rpc("tools/call", dict(name=name, arguments=args))
        return json.loads(r["content"][0]["text"])

    def wait_lease(self, limit: float = 8.0) -> bool:
        """Until its lease thread holds the lease (another session's runs out within its 3 s TTL first)."""
        t_end = time.monotonic() + limit
        while time.monotonic() < t_end:
            d = self.tool("robot_status").get("data") or {}
            if (d.get("lease") or {}).get("held"):
                return True
            time.sleep(0.5)
        return False

    def kill(self):
        self.p.kill()
        self.p.wait(5)


def hold_lease(robot: Robot, on: bool):
    """One AI session holds the lease at a time: this process's for its own checks, else the MCP child's."""
    if on and robot.lease is None:
        robot.lease = LeaseKeeper(robot.rb, robot.session).start()
        t_end = time.monotonic() + 8.0                      # an MCP child's lease runs out first (3 s)
        while time.monotonic() < t_end and not robot.lease.state()["held"]:
            time.sleep(0.2)
    elif not on and robot.lease is not None:
        robot.lease.stop()
        robot.lease = None
        time.sleep(3.5)


def until_done(robot: Robot, tid: str, limit: float = 120.0) -> dict:
    t_end = time.monotonic() + limit
    d = {}
    while time.monotonic() < t_end:
        d = robot.task(tid, 10.0).get("data") or {}
        if d.get("state") in TERMINAL:
            return d
    return d


def settle(truth: Truth, limit: float = 10.0):
    t_end = time.monotonic() + limit
    while time.monotonic() < t_end and not abs(truth.rate()) < 2.0:
        time.sleep(0.1)


def free_robot(robot: Robot, limit: float = 20.0):
    """Wait until nothing holds the robot (a person's 10 s hold, a task ending): two free statuses in a row, the
    first after the status has caught up with the last command (it comes at 5 Hz)."""
    t_end = time.monotonic() + limit
    n = 0
    time.sleep(0.6)
    while time.monotonic() < t_end:
        s = robot.status().get("data") or {}
        n = n + 1 if (s.get("ai") or {}).get("owner") is None and s.get("mission") in ("MANUAL", "IDLE") else 0
        if n >= 2:
            return True
        time.sleep(0.5)
    return False


def check(name: str, ok: bool, **info) -> dict:
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {json.dumps(info, default=str)}")
    return dict(name=name, ok=bool(ok), **info)


# -- the checks ------------------------------------------------------------------------------------------------------
def spin(robot: Robot, truth: Truth, env: dict) -> list:
    hold_lease(robot, False)
    mcp = McpChild(env)
    try:
        mcp.wait_lease()
        settle(truth)
        y0, t0 = truth.yaw, time.monotonic()
        r = mcp.tool("robot_move", steps=[{"do": "spin", "turns": 5}], turn_rate="max")
        if not r["ok"]:
            return [check("spin accepted", False, reply=r)]
        tid, est = r["data"]["task_id"], r["data"]["result"]["estimated_s"]
        d = {}
        while True:
            d = mcp.tool("robot_task", task_id=tid, wait_s=10)["data"]
            if d["state"] in TERMINAL:
                break
        settle(truth, 3.0)
        t1 = time.monotonic()
        turned = math.degrees(truth.yaw - y0)
        dur = truth.span(t0, t1)                              # sim time: the console's own clock is the wall's
        return [check("spin 5 x 360 at max", d["state"] == "SUCCEEDED" and abs(turned - 1800) <= 3.0,
                      state=d["state"], true_turn_deg=round(turned, 1), odom_turn_deg=d["evidence"].get("turned_deg"),
                      limits=r["data"]["result"]["limits"]),
                check("spin duration vs estimate (sim time)", dur is not None and abs(dur - est) <= 0.15 * est,
                      sim_duration_s=dur, estimate_s=est, wall_duration_s=d["evidence"].get("duration_s"),
                      true_max_rate_deg_s=truth.max_rate(t0, t1),
                      odom_max_rate_deg_s=d["evidence"].get("max_turn_rate_deg_s"))]
    finally:
        mcp.kill()


def lease(robot: Robot, truth: Truth, env: dict) -> list:
    hold_lease(robot, False)
    free_robot(robot)
    mcp = McpChild(env)
    mcp.wait_lease()
    r = mcp.tool("robot_move", steps=[{"do": "spin", "turns": 5}], turn_rate="max")
    if not r["ok"]:
        mcp.kill()
        return [check("lease: spin accepted", False, reply=r)]
    time.sleep(2.5)
    spinning = truth.rate()
    mcp.kill()
    t_kill = time.monotonic()
    t_stop = None
    while time.monotonic() - t_kill < 8.0:
        if abs(truth.rate(0.2)) < 5.0:
            t_stop = time.monotonic()
            break
        time.sleep(0.02)
    d = robot.call("task", dict(task_id=r["data"]["task_id"]))["data"]
    dt = None if t_stop is None else round(t_stop - t_kill, 2)
    return [check("lease lapse stops the robot", t_stop is not None and dt <= 3.5 and d["state"] == "CANCELLED",
                  spinning_deg_s=round(spinning, 1), stopped_after_s=dt, task=d["state"], why=d.get("message"))]


def dup(robot: Robot, truth: Truth, env: dict) -> list:
    hold_lease(robot, True)
    free_robot(robot)
    settle(truth)
    y0 = truth.yaw
    rid = f"smoke-dup-{robot.boot}"                           # new each run: the robot keeps old ids over restarts
    r1 = robot.move([{"do": "spin", "turns": 1}], turn_rate=120, request_id=rid)
    r2 = robot.move([{"do": "spin", "turns": 1}], turn_rate=120, request_id=rid)
    d = until_done(robot, r1["data"]["task_id"], 60)
    r3 = robot.move([{"do": "spin", "turns": 1}], turn_rate=120, request_id=rid)    # after it ended too
    time.sleep(2.0)
    settle(truth)
    turned = math.degrees(truth.yaw - y0)
    same = r1["data"]["task_id"] == r2["data"]["task_id"] == r3["data"]["task_id"]
    return [check("duplicate request_id: one task, one turn", same and abs(turned - 360) < 5 and d["state"] ==
                  "SUCCEEDED", tasks=[r["data"].get("task_id") for r in (r1, r2, r3)], true_turn_deg=round(turned, 1))]


def owner(robot: Robot, truth: Truth, env: dict) -> list:
    hold_lease(robot, True)
    free_robot(robot)
    page = robot.call("mission", dict(name="MANUAL"))                 # the page's Take control (no "ai")
    r = robot.move([{"do": "spin", "turns": 1}])
    out = [check("page owns: move refused", page["ok"] and not r["ok"] and r["data"].get("error") == "owner_conflict",
                 reply=r["message"])]
    time.sleep(10.5)                                                  # the person's hold, then a running AI task
    r2 = robot.move([{"do": "spin", "turns": 2}], turn_rate=60)
    time.sleep(1.5)
    robot.call("mission", dict(name="MANUAL"))
    d = robot.call("task", dict(task_id=r2["data"].get("task_id", "")))["data"]
    out.append(check("page takes over a running task", r2["ok"] and d.get("state") == "CANCELLED",
                     task=d.get("state"), why=d.get("message")))
    return out


def goto(robot: Robot, truth: Truth, env: dict) -> list:
    hold_lease(robot, True)
    free_robot(robot)
    s = robot.status()["data"]
    p = s.get("pose") or {}
    if p.get("frame") != "map":
        return [check("goto: localized", False, pose=p)]
    bad = robot.goto("kitchen")
    out = [check("goto unknown place refused", bad["data"].get("error") == "unknown_place", reply=bad["message"])]
    x, y, yaw = p["x"], p["y"], p["yaw_deg"]
    gx, gy = x + 0.8 * math.cos(math.radians(yaw)), y + 0.8 * math.sin(math.radians(yaw))
    r = robot.goto(x=gx, y=gy, yaw_deg=yaw + 90)
    d = until_done(robot, r["data"]["task_id"], 120) if r["ok"] else {}
    out.append(check("goto a map pose", d.get("state") == "SUCCEEDED" and d["evidence"]["distance_to_goal_m"] < 0.25,
                     reply=r["message"], state=d.get("state"), evidence=d.get("evidence")))
    r = robot.goto(x=x, y=y, yaw_deg=yaw)
    d = until_done(robot, r["data"]["task_id"], 120) if r["ok"] else {}
    out.append(check("goto back", d.get("state") == "SUCCEEDED", state=d.get("state"), evidence=d.get("evidence")))
    return out


def seek(robot: Robot, color: str, shape: str, out: list):
    """As the model would: find; a guess -> face it and close in to ~0.6 m; the colour seen but nothing pickable ->
    face it and close in, once (a blob still nothing pickable when faced and near is something else red: look on);
    nothing -> turn 45 deg (eight times at most); find again. The best candidate (score >= 0.5) or None."""
    turned = 0
    for attempt in range(16):
        f, img = robot.find(color, shape)
        if not f["ok"]:
            out.append(check(f"find {color} {shape}", False, reply=f["message"]))
            return None
        cands = f["data"]["candidates"]
        blobs = [b for b in f["data"].get("image_only") or [] if not b.get("flat")         # a rug: not it
                 and (abs(b["bearing_deg"]) > 10 or (b.get("range_m") or 0) > 1.4)]   # faced and near: not it
        print(f"   find {attempt}: " + "; ".join(f"id {c['id']} {c['color']['name']} {c['shape']['class']}"
                                                 f"{'' if c['shape']['sure'] else '?'} {c['score']} at "
                                                 f"{c['range_m']} m {c['bearing_deg']} deg" for c in cands)
              + "".join(f"; image-only {b['pixels']} px at {b['bearing_deg']} deg, {b.get('range_m')} m, "
                        f"{b.get('height_m')} m up" for b in f["data"].get("image_only") or []))
        if cands and cands[0]["score"] >= 0.5:
            return cands[0]
        hint = f["data"].get("hint") or ""
        named = {s.strip() for s in hint.split(":")[0].split(",")} if hint else set()   # "6, 9: the right colour..."
        guess = [c for c in cands if str(c["id"]) in named]
        if guess:
            g = guess[0]
            steps = [{"do": "turn", "deg": g["bearing_deg"]}]
            if g["range_m"] > 0.7:
                steps.append({"do": "forward", "m": round(g["range_m"] - 0.6, 2)})
        elif blobs:
            steps = [{"do": "turn", "deg": blobs[0]["bearing_deg"]}]
            if (blobs[0].get("range_m") or 0) > 1.4:
                steps.append({"do": "forward", "m": round(min(1.0, blobs[0]["range_m"] - 0.8), 2)})
        elif turned < 8:
            turned += 1
            steps = [{"do": "turn", "deg": 45}]
        else:
            return None
        m = robot.move(steps, speed=0.3, turn_rate=60)
        if not m["ok"]:
            out.append(check("move closer", False, reply=m["message"]))
            return None
        until_done(robot, m["data"]["task_id"], 60)
        time.sleep(1.0)
    return None


def pick_one(robot: Robot, best: dict, want: str, out: list) -> dict:
    r = robot.pick(best["id"])
    if not r["ok"]:
        out.append(check("pick accepted", False, reply=r))
        return {}
    d = until_done(robot, r["data"]["task_id"], 200)
    truth_bin = d.get("sim_truth") or {}
    out.append(check(f"pick: only {want} into the bin",
                     d.get("state") == "SUCCEEDED" and truth_bin.get("newly_in_bin") == [want],
                     state=d.get("state"), evidence=d.get("evidence"), sim_truth=truth_bin, message=d.get("message")))
    return d


def pick(robot: Robot, truth: Truth, env: dict) -> list:
    hold_lease(robot, True)
    free_robot(robot)
    out = []
    look, jpeg = robot.look("front")
    out.append(check("look front", look["ok"] and jpeg is not None, info=look["message"]))
    f, img = robot.find("purple", "round")
    none_round = f["ok"] and all(c["score"] < 0.5 for c in f["data"]["candidates"])
    out.append(check("purple round: none matches", none_round,
                     candidates=[(c["id"], c["score"], c["shape"]["class"]) for c in f["data"].get("candidates") or []]))
    best = seek(robot, "purple", "square", out)
    out.append(check("find purple square", best is not None, best=best and dict(
        id=best["id"], score=best["score"], color=best["color"], shape=best["shape"], range_m=best["range_m"])))
    if best is not None:
        pick_one(robot, best, "cube40_purple", out)
    return out


def here(robot: Robot) -> dict:
    return (robot.status().get("data") or {}).get("pose") or {}


def ahead(p: dict, m: float) -> dict:
    yaw = math.radians(p.get("yaw_deg", 0.0))
    return dict(x=p["x"] + m * math.cos(yaw), y=p["y"] + m * math.sin(yaw), yaw_deg=p.get("yaw_deg", 0.0))


def fetch(robot: Robot, truth: Truth, env: dict) -> list:
    """'Bring me the red cube': from where the robot stands, find it, pick it, come back (goto)."""
    hold_lease(robot, True)
    free_robot(robot)
    out = []
    p0 = here(robot)
    if p0.get("frame") != "map":
        return [check("fetch: localized", False, pose=p0)]
    best = seek(robot, "red", "square", out)
    out.append(check("find red square", best is not None, best=best and dict(id=best["id"], score=best["score"])))
    if best is None:
        return out
    pick_one(robot, best, "cube30", out)
    r = robot.goto(x=p0["x"], y=p0["y"], yaw_deg=p0["yaw_deg"])
    d = until_done(robot, r["data"]["task_id"], 120) if r["ok"] else {}
    out.append(check("fetch: back where it started", d.get("state") == "SUCCEEDED" and
                     (d.get("evidence") or {}).get("distance_to_goal_m", 9) < 0.1, reply=r["message"],
                     evidence=d.get("evidence")))
    return out


def blocked(robot: Robot, truth: Truth, env: dict) -> list:
    """A goal Nav2 can't reach (far outside the house): the task fails with Nav2's reason, the robot stops."""
    hold_lease(robot, True)
    free_robot(robot)
    r = robot.goto(x=40.0, y=40.0)
    d = until_done(robot, r["data"]["task_id"], 150) if r["ok"] else {}
    time.sleep(1.0)
    s = robot.status().get("data") or {}
    v = truth.speed()
    return [check("unreachable goal fails, robot stops", d.get("state") == "FAILED" and s.get("mission") != "GOTO"
                  and v < 0.03, reply=r["message"], state=d.get("state"), result=d.get("result"),
                  why=d.get("message"), speed_m_s=round(v, 3))]


def navlease(robot: Robot, truth: Truth, env: dict) -> list:
    """The MCP process killed while Nav2 drives its goto: the console cancels the goal, the robot stops."""
    hold_lease(robot, False)
    free_robot(robot)
    p0 = here(robot)
    mcp = McpChild(env)
    mcp.wait_lease()
    r = mcp.tool("robot_goto", **ahead(p0, 1.5))
    if not r["ok"]:
        mcp.kill()
        return [check("navlease: goto accepted", False, reply=r)]
    t_end = time.monotonic() + 15
    while time.monotonic() < t_end and not truth.speed() > 0.1:      # until it really drives
        time.sleep(0.1)
    moving = truth.speed()
    mcp.kill()
    t_kill = time.monotonic()
    t_stop = None
    while time.monotonic() - t_kill < 10.0:
        if truth.speed(0.4) < 0.02:
            t_stop = time.monotonic()
            break
        time.sleep(0.05)
    d = robot.call("task", dict(task_id=r["data"]["task_id"])).get("data") or {}
    dt = None if t_stop is None else round(t_stop - t_kill, 2)
    out = [check("lease lapse during a goto stops the robot", t_stop is not None and dt <= 4.5 and
                 d.get("state") == "CANCELLED", driving_m_s=round(moving, 2), stopped_after_s=dt,
                 task=d.get("state"), why=d.get("message"))]
    hold_lease(robot, True)
    r = robot.goto(x=p0["x"], y=p0["y"], yaw_deg=p0.get("yaw_deg", 0.0))      # back for the next check
    if r["ok"]:
        until_done(robot, r["data"]["task_id"], 120)
    return out


def restart(robot: Robot, truth: Truth, env: dict) -> list:
    """The console's mission node restarted mid-goto (--restart-cmd): the task INTERRUPTED, nothing resumes, the
    robot stops (the new console cancels the goal Nav2 would otherwise keep driving)."""
    cmd = env.get("RESTART_CMD")
    if not cmd:
        return [check("restart", False, why="no --restart-cmd")]
    hold_lease(robot, True)
    free_robot(robot)
    p0 = here(robot)
    r = robot.goto(**ahead(p0, 1.5))
    if not r["ok"]:
        return [check("restart: goto accepted", False, reply=r)]
    t_end = time.monotonic() + 15
    while time.monotonic() < t_end and not truth.speed() > 0.1:
        time.sleep(0.1)
    rc = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=60)
    t_restart = time.monotonic()
    back = None
    while time.monotonic() - t_restart < 60:
        if robot.call("ping", timeout=2.0)["ok"]:
            back = round(time.monotonic() - t_restart, 1)
            break
        time.sleep(1.0)
    t_stop = None
    while time.monotonic() - t_restart < 45:
        if truth.speed(0.5) < 0.02:
            t_stop = round(time.monotonic() - t_restart, 1)
            break
        time.sleep(0.2)
    time.sleep(3.0)
    d = robot.call("task", dict(task_id=r["data"]["task_id"])).get("data") or {}
    s = robot.status().get("data") or {}
    v = truth.speed()
    return [check("restart: task INTERRUPTED, nothing resumed, robot stopped",
                  back is not None and d.get("state") == "INTERRUPTED" and s.get("mission") != "GOTO"
                  and t_stop is not None and v < 0.02, restart_output=(rc.stdout + rc.stderr)[-300:],
                  console_back_s=back, stopped_after_s=t_stop, task=d.get("state"), why=d.get("message"),
                  mission=s.get("mission"), speed_m_s=round(v, 3))]


CHECKS = dict(spin=spin, lease=lease, dup=dup, owner=owner, goto=goto, blocked=blocked, navlease=navlease,
              pick=pick, fetch=fetch, restart=restart)
DEFAULT = "spin,lease,dup,owner,goto,blocked,navlease,pick,fetch"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default=os.environ.get("OMNI_HOST", "127.0.0.1"))
    ap.add_argument("--rosbridge-port", type=int, default=int(os.environ.get("OMNI_ROSBRIDGE_PORT", 9090)))
    ap.add_argument("--video-port", type=int, default=int(os.environ.get("OMNI_VIDEO_PORT", 8081)))
    ap.add_argument("--only", default=DEFAULT, help="of " + ", ".join(CHECKS) + " (restart needs --restart-cmd)")
    ap.add_argument("--restart-cmd", help="a shell command that restarts the console's mission node (the restart check), e.g. wsl.exe ... ros/tools/restart_console.sh omni_ws_claude")
    ap.add_argument("--out", help="the results as JSON")
    a = ap.parse_args()
    env = dict(OMNI_HOST=a.host, OMNI_ROSBRIDGE_PORT=str(a.rosbridge_port), OMNI_VIDEO_PORT=str(a.video_port))
    if a.restart_cmd:
        env["RESTART_CMD"] = a.restart_cmd
    robot = Robot(a.host, a.rosbridge_port, a.video_port, session="ai-smoke", lease=False, log=lambda m: None)
    if not robot.rb.wait_up(15.0):
        sys.exit(f"no rosbridge at {a.host}:{a.rosbridge_port}: {robot.rb.error}")
    truth = Truth(robot.rb)
    t_end = time.monotonic() + 120
    while time.monotonic() < t_end and not (robot.status()["ok"] and truth.yaw is not None):
        time.sleep(1.0)
    st = robot.status()
    print(f"robot: {st['message']}")
    results = []
    for name in a.only.split(","):
        try:
            results += CHECKS[name](robot, truth, env)
        except Exception as e:                                  # a broken check is a failed check, not a crash
            results.append(check(name, False, exception=repr(e)))
    robot.stop()
    robot.close()
    n_ok = sum(r["ok"] for r in results)
    print(f"{n_ok}/{len(results)} passed")
    if a.out:
        Path(a.out).write_text(json.dumps(dict(results=results, status=st.get("data")), indent=1, default=str))
    sys.exit(0 if n_ok == len(results) else 1)


if __name__ == "__main__":
    main()
