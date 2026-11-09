"""Goalie on the full ROS stack, scored on the sim's truth: the console's own Goalie mission and omni_arm's ball tracker
on the sim's D435, and rolled shots from the bridge (sim_world.shoot: studies/goalie.py's 22 cm, 0.3 kg play ball from
3 m, 1.5-3 m/s, seeded). Sim only: the real-camera ball benches stay paused (AGENTS.md).

  wsl ... ros/omni.sh arm clutter:=off viewer:=false face:=false            (the robot side)
  & C:\\Users\\ferna\\.venvs\\omni-mcp\\Scripts\\python.exe ros/tools/goalie_smoke.py [--shots 10] [--seed 0] [--speed 2.5]
      [--host 127.0.0.1 --rosbridge-port 9090 --video-port 8081 --control-port 9873] [--out result.json]

Each shot's verdict is the sim's (/sim/goalie_shot): a save when the ball never crosses the goal line, a goal between
the posts, wide outside them. The robot's own count (/console/status goalie) is printed beside it. Exit 0 when at least
--min-rate of the on-target shots are saved and the tracker saw the ball on every shot.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "omni_mcp"))

from server import Robot  # noqa: E402


def world(host: str, port: int, req: dict) -> dict:
    """One request on the bridge's control port (sim_world.ControlPort)."""
    with socket.create_connection((host, port), timeout=5.0) as sock:
        sock.sendall((json.dumps(req) + "\n").encode())
        return json.loads(sock.makefile().readline())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default=os.environ.get("OMNI_HOST", "127.0.0.1"))
    ap.add_argument("--rosbridge-port", type=int, default=int(os.environ.get("OMNI_ROSBRIDGE_PORT", 9090)))
    ap.add_argument("--video-port", type=int, default=int(os.environ.get("OMNI_VIDEO_PORT", 8081)))
    ap.add_argument("--control-port", type=int, default=int(os.environ.get("OMNI_CONTROL_PORT", 9873)))
    ap.add_argument("--shots", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--speed", type=float, default=None, help="m/s, all shots (default 1.5-3, seeded)")
    ap.add_argument("--min-rate", type=float, default=0.7, help="share of on-target shots to save (est)")
    ap.add_argument("--out", help="the results as JSON")
    a = ap.parse_args()

    robot = Robot(a.host, a.rosbridge_port, a.video_port, session="goalie-smoke", lease=False, log=lambda m: None)
    if not robot.rb.wait_up(15.0):
        sys.exit(f"no rosbridge at {a.host}:{a.rosbridge_port}: {robot.rb.error}")
    rb = robot.rb
    rb.subscribe("/sim/goalie_shot", "std_msgs/msg/String")
    rb.subscribe("/goalie/ball", "std_msgs/msg/String")
    t_end = time.monotonic() + 120
    while time.monotonic() < t_end and not robot.status()["ok"]:
        time.sleep(1.0)
    print(f"robot: {robot.status()['message']}")
    world(a.host, a.control_port, {"clutter": "off"})
    rtf = world(a.host, a.control_port, {"get": 1}).get("rtf")
    r = robot.call("goalie", {"seconds": 0})
    if not r["ok"]:
        sys.exit(f"Goalie refused: {r['message']}")

    def raw() -> dict:
        return robot.status_raw()[0] or {}

    t_end = time.monotonic() + 30
    while time.monotonic() < t_end and raw().get("mission") != "GOALIE":
        time.sleep(0.2)
    if raw().get("mission") != "GOALIE":
        sys.exit(f"Goalie didn't start: {raw().get('task')}")
    time.sleep(8.0)                                     # the tracker wakes, the D435 streams settle

    def sim_t() -> float:
        return float(world(a.host, a.control_port, {"get": 1})["t"])

    rng = np.random.default_rng(a.seed)
    rows = []
    for k in range(a.shots):
        side, aim = float(rng.uniform(-0.4, 0.4)), float(rng.uniform(-0.55, 0.55))
        v = a.speed or float(rng.uniform(1.5, 3.0))
        shot = world(a.host, a.control_port, {"shoot": dict(side=side, aim=aim, speed=v)})
        if not shot.get("ok"):
            print(f"shot {k}: refused: {shot.get('message')}")
            rows.append(dict(shot=k, result="refused", why=shot.get("message")))
            continue
        res, seen, t_end = None, 0, time.monotonic() + 60
        while time.monotonic() < t_end:
            m, _ = rb.newest_json("/sim/goalie_shot")
            if m and m.get("id") == shot["id"]:
                res = m
                break
            b, age = rb.newest_json("/goalie/ball")
            seen = max(seen, int((b or {}).get("n", 0)) if age is not None and age < 0.5 and (b or {}).get("seen") else 0)
            time.sleep(0.02)
        row = dict(res or dict(result="no_result", speed=round(v, 2), aim=round(aim, 3), side=round(side, 2)), shot=k, tracked=seen)
        rows.append(row)
        t = sim_t() if res else 0.0                     # back to the middle of the goal: 2.5 sim-seconds, no ball
        t_end = time.monotonic() + 40
        ghosts, looks = 0, 0                            # the tracker's word while no ball is in the house
        while time.monotonic() < t_end and sim_t() < t + 2.5:
            b, age = rb.newest_json("/goalie/ball")
            looks += 1
            ghosts += bool(b and b.get("seen") and age is not None and age < 0.5)
            time.sleep(0.2)
        row["robot"] = {k2: (raw().get("goalie") or {}).get(k2) for k2 in ("shots", "saves", "goals")}
        row["ghost_share"] = round(ghosts / max(1, looks), 2)
        print(row, flush=True)

    s = raw()
    own = s.get("goalie") or {}
    robot.call("stop")
    robot.close()
    on = [x for x in rows if x["result"] in ("save", "goal")]
    saves = sum(x["result"] == "save" for x in on)
    summary = dict(shots=len(rows), on_target=len(on), saves=saves, goals=sum(x["result"] == "goal" for x in on),
                   wide=sum(x["result"] == "wide" for x in rows), robot_says=own, rtf_at_start=rtf, seed=a.seed,
                   untracked=sum(x.get("tracked", 0) == 0 for x in rows))
    print(summary)
    if a.out:
        Path(a.out).write_text(json.dumps(dict(summary, runs=rows), indent=1))
    ok = bool(rows) and summary["untracked"] == 0 and (not on or saves / len(on) >= a.min_rate)
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
