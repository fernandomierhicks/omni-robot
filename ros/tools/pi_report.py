"""A table of the robot-computer runs (characterization/pi_camera/results/pi_robot_*.json, from pi.sh split / pi_keepup.py).

  python pi_report.py [DIR] [--md]

One row per run: the label, the Pi's CPU (median / max of all cores, busiest core), the robot's own processes in cores
(the monitor and ssh left out), the biggest processes, memory and swap, temperature, throttling flags, and the rate of the
slowest checked topic. Reads only the JSON, so it runs anywhere.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT = HERE.parents[1] / "characterization" / "pi_camera" / "results"
OTHER = {"pi_keepup.py", "sshd:", "avahi-daemon:", "systemd-journald", "polkitd", "fwupd", "ros2 launch/daemon", "record_throws.py"}


def row(f: Path) -> dict:
    d = json.loads(f.read_text())
    h, p = d["host_load"], d["processes"]
    top = sorted(((g, v["median_pct"]) for g, v in p.items() if isinstance(v, dict) and g not in OTHER), key=lambda x: -x[1])[:4]
    topics = {n: t for n, t in d.get("topics", {}).items() if "rate_hz" in t}
    return dict(label=d["label"], file=f.name, cores=d.get("cores"),
                cpu=f"{h['cpu_total_pct']['median']:.0f} / {h['cpu_total_pct']['max']:.0f} %", core=f"{h['busiest_core_pct']['max']:.0f} %",
                robot=f"{(p.get('_robot_total_pct_of_one_core') or 0) / 100:.2f}", top=", ".join(f"{g} {v:.0f}%" for g, v in top),
                mem=f"{h['mem_available_mb_min']:.0f} MB / swap {h['swap_used_mb_max']:.0f}", temp=h["temp_c"]["max"],
                thr=",".join(h["throttle_flags_seen"]) if isinstance(h["throttle_flags_seen"], list) and h["throttle_flags_seen"] else "none",
                rates="; ".join(f"{n} {t['rate_hz']:.0f} Hz" for n, t in topics.items() if n in ("/mcu/state_10hz", "/scan")))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dir", nargs="?", type=Path, default=DEFAULT)
    ap.add_argument("--md", action="store_true")
    a = ap.parse_args()
    rows = [row(f) for f in sorted(a.dir.glob("pi_robot_*.json"))]
    cols = ("label", "cpu", "core", "robot", "top", "mem", "temp", "thr", "rates")
    head = ("run", "CPU med / max", "busiest core", "robot cores", "biggest processes (% of a core)", "memory free", "temp C", "throttled", "rates")
    if a.md:
        print("| " + " | ".join(head) + " |\n|" + " --- |" * len(head))
        for r in rows:
            print("| " + " | ".join(str(r[c]) for c in cols) + " |")
    else:
        for r in rows:
            print(f"{r['label']}\n    CPU {r['cpu']} of {r['cores']} cores, busiest core {r['core']}; robot {r['robot']} cores: {r['top']}\n"
                  f"    memory {r['mem']}; temp {r['temp']} C; throttling {r['thr']}; {r['rates']}")


if __name__ == "__main__":
    main()
