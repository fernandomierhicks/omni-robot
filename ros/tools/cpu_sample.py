"""CPU use of the robot-side processes, from /proc (Linux: WSL in the sim, the Jetson on the robot).

  python3 cpu_sample.py [seconds] [--json]

Groups the processes by what they are (the operator console's own processes first: what it costs the Jetson to serve
the page) and prints % of one core over the window. ros/tools/console_smoke.py --cpu runs it through wsl.exe.
"""

from __future__ import annotations

import json
import os
import sys
import time

# group: substrings of the command line (first match wins; the order matters)
GROUPS = (
    ("console: mission node", ("omni_console/mission_node", "lib/omni_console/mission_node")),
    ("console: page server", ("omni_console/console_web", "omni_console.supervisor", "omni_console/supervisor")),
    ("console: rosbridge", ("rosbridge_websocket",)),
    ("console: rosapi", ("rosapi_node",)),
    ("console: video", ("web_video_server", "image_preview")),
    ("mcu_link", ("omni_mcu/mcu_link", "lib/omni_mcu/mcu_link")),
    ("mode_manager", ("mode_manager",)),
    ("catch code", ("catch_perception", "catch_behavior")),
    ("slam_toolbox", ("slam_toolbox",)),
    ("nav2", ("controller_server", "planner_server", "bt_navigator", "behavior_server", "smoother_server",
              "velocity_smoother", "collision_monitor", "lifecycle_manager")),
    ("ekf + scan prep", ("ekf_node", "scan_prep")),
    ("foxglove", ("foxglove_bridge",)),
    ("sim (bridge)", ("omni_sim/bridge", "omni_sim/catch_bridge", "lib/omni_sim/")),
)
HZ = os.sysconf("SC_CLK_TCK")


def snapshot() -> dict:
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace")
            with open(f"/proc/{pid}/stat") as f:
                st = f.read().rsplit(")", 1)[1].split()
        except OSError:
            continue
        if "cpu_sample.py" in cmd:
            continue
        for name, keys in GROUPS:
            if any(k in cmd for k in keys):
                out[int(pid)] = (name, int(st[11]) + int(st[12]))      # utime + stime, clock ticks
                break
    return out


def sample(seconds: float) -> dict:
    a, t0 = snapshot(), time.monotonic()
    time.sleep(seconds)
    b, t1 = snapshot(), time.monotonic()
    use: dict = {}
    for pid, (name, ticks) in b.items():
        if pid in a:
            use[name] = use.get(name, 0.0) + (ticks - a[pid][1]) / HZ / (t1 - t0) * 100.0
    return {k: round(v, 1) for k, v in sorted(use.items(), key=lambda kv: [g[0] for g in GROUPS].index(kv[0]))}


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 10.0
    use = sample(secs)
    if "--json" in sys.argv:
        print(json.dumps(use))
        return
    console = sum(v for k, v in use.items() if k.startswith("console"))
    print(f"CPU over {secs:g} s, % of one core ({os.cpu_count()} cores):")
    for k, v in use.items():
        print(f"  {k:24s} {v:6.1f}")
    print(f"  {'console total':24s} {console:6.1f}")


if __name__ == "__main__":
    main()
