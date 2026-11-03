"""CPU of every process while a step runs, from /proc (Linux: WSL in the sim, the robot computer on the robot).

  python3 cpu_watch.py OUT.jsonl [--window 10] [--seconds 120 | --until FILE]

Unlike cpu_sample.py (fixed groups, one window), this records every process by its ROS executable, every window, as
JSON lines: {"t", "cpu": {group: % of one core}, "rss_mb": {group: MB}}. The sim (bridge, scorers) is the group "SIM"
so it can be left out. Run it beside a scored step (omni.sh goals, catch-score) in the sim, and beside the same stack
on a candidate computer: database/computers.csv's meas_stack_cpu_pct, simulation/v11 sil/studies/compute_choice.py.
"""

from __future__ import annotations

import json
import os
import sys
import time

HZ = os.sysconf("SC_CLK_TCK")
PAGE = os.sysconf("SC_PAGE_SIZE")
SKIP = ("bash", "sleep", "sshd", "systemd", "wsl", "init", "dbus", "rsync")
SIM = ("omni_sim/", "lib/omni_sim/", "mujoco_view", "catch_bridge", "bridge_node")


def group(cmd: str) -> str | None:
    if any(k in cmd for k in SIM):
        return "SIM"
    if "ros2 launch" in cmd or "_ros2_daemon" in cmd:
        return "ros2 launch/daemon"
    parts = cmd.split()
    base = os.path.basename(parts[0]) if parts else ""
    if base.startswith("python"):                      # a Python node: its script's name
        return next((os.path.basename(p) for p in parts[1:] if not p.startswith("-")), base)
    return None if base in SKIP or not base else base


def snapshot() -> dict:
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
            with open(f"/proc/{pid}/stat") as f:
                st = f.read().rsplit(")", 1)[1].split()
            with open(f"/proc/{pid}/statm") as f:
                rss = int(f.read().split()[1]) * PAGE
        except OSError:
            continue
        g = group(cmd) if cmd and "cpu_watch.py" not in cmd else None
        if g:
            out[int(pid)] = (g, int(st[11]) + int(st[12]), rss)        # utime + stime, clock ticks
    return out


def main(argv):
    if not argv or argv[0].startswith("-"):
        print(__doc__)
        return 2
    out, window, seconds, until = argv[0], 10.0, None, None
    for i, a in enumerate(argv):
        if a == "--window":
            window = float(argv[i + 1])
        elif a == "--seconds":
            seconds = float(argv[i + 1])
        elif a == "--until":
            until = argv[i + 1]
    t_end = time.monotonic() + seconds if seconds else None
    with open(out, "a") as f:
        while not (until and os.path.exists(until)) and not (t_end and time.monotonic() >= t_end):
            a, t0 = snapshot(), time.monotonic()
            time.sleep(window)
            b, t1 = snapshot(), time.monotonic()
            use: dict = {}
            mem: dict = {}
            for pid, (g, ticks, rss) in b.items():
                if pid in a:
                    use[g] = use.get(g, 0.0) + (ticks - a[pid][1]) / HZ / (t1 - t0) * 100.0
                    mem[g] = mem.get(g, 0) + rss
            f.write(json.dumps({"t": time.time(), "cpu": {k: round(v, 1) for k, v in use.items() if v >= 0.05},
                                "rss_mb": {k: round(v / 2**20) for k, v in mem.items()}}) + "\n")
            f.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
