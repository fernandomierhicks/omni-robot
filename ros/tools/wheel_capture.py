#!/usr/bin/env python3
"""Capture a motor-test profile from the robot's own stack (protocol 9): wheels up, attended.

Through the console like a person at the page: mission MOTORS, the wheel, motors_run (a preset or segments), so
mode_manager's WHEELS and the firmware's DRIVE_WHEELS turn it at the asked ramp. Meanwhile it records /mcu/state at
the MCU's 200 Hz: each wheel's asked and measured speed and its motor's current (wheel_cur). Ends with "stop" (motors
off, the wheels ramp to rest) unless --keep.

On the robot computer, against its running robot step (same ROS domain / discovery as the console):
    python3 wheel_capture.py --wheel 2 --preset steps --speed 30 --ramp 2000 --out steps.csv
    python3 wheel_capture.py --wheel 2 --segments '[[20, 500, 0.8], [0, 500, 0.8]]' --out r500.csv
Columns: t (MCU s from the first sample), then per wheel k: c<k> (asked rad/s), w<k> (measured), i<k> (A), e<k>
(dm_err), plus mode, drive_mode and the faults.
"""
import argparse
import csv
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from omni_interfaces.msg import McuState
from omni_interfaces.srv import ConsoleCommand


class Capture(Node):
    def __init__(self):
        super().__init__("wheel_capture")
        self.rows = []
        self.recording = False
        self.n_seen = 0
        self.create_subscription(McuState, "/mcu/state", self.on_state,
                                 QoSProfile(depth=400, reliability=ReliabilityPolicy.RELIABLE))
        self.cli = self.create_client(ConsoleCommand, "/console/command")

    def on_state(self, m: McuState):
        self.n_seen += 1
        if not self.recording:
            return
        row = [m.t_mcu_us * 1e-6]
        for k in range(4):
            row += [float(m.wheel_cmd[k]), float(m.wheel_vel[k]), float(m.wheel_cur[k]), int(m.dm_err[k])]
        row += [m.mode, int(m.drive_mode), "|".join(m.faults)]
        self.rows.append(row)

    def command(self, verb: str, args: dict | None = None, timeout: float = 5.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/console/command isn't there: is the robot step running?")
        req = ConsoleCommand.Request(verb=verb, args=json.dumps(args or {}))
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None:
            raise RuntimeError(f"{verb}: no reply")
        print(f"{verb} {json.dumps(args or {})}: {'ok' if r.ok else 'REFUSED'}: {r.message}", flush=True)
        return r

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wheel", type=int, default=1, help="1-4, the motor's CAN id")
    ap.add_argument("--preset", choices=("steps", "sweep"))
    ap.add_argument("--segments", help="JSON list of [rad/s, rad/s^2, hold s]")
    ap.add_argument("--speed", type=float)
    ap.add_argument("--ramp", type=float)
    ap.add_argument("--hold", type=float)
    ap.add_argument("--pre", type=float, default=0.5, help="s recorded before the profile")
    ap.add_argument("--post", type=float, default=1.0, help="s recorded after it")
    ap.add_argument("--keep", action="store_true", help="leave the motor test running after")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    run = {}
    if a.segments:
        run["segments"] = json.loads(a.segments)
    else:
        run["preset"] = a.preset or "steps"
    for k in ("speed", "ramp", "hold"):
        if getattr(a, k) is not None:
            run[k] = getattr(a, k)

    rclpy.init()
    n = Capture()
    # wait until STATE is flowing before asking for anything: a new node's subscription can take seconds to match on
    # the Pi (Nov 13: up to ~2.5 s), and a profile started before it loses its start
    end = time.monotonic() + 10.0
    while n.n_seen < 40 and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.n_seen < 40:
        print("no /mcu/state: is the robot step running?", file=sys.stderr)
        return 1
    try:
        if not n.command("mission", {"name": "MOTORS"}).ok:
            return 1
        if not n.command("motors", {"wheels": [a.wheel - 1]}).ok:
            return 1
        n.recording = True
        n.spin_for(a.pre)
        r = n.command("motors_run", run)
        if not r.ok:
            return 1
        total = float(json.loads(r.data or "{}").get("total", 30.0))
        n.spin_for(total + a.post)
        n.recording = False
    finally:
        if not a.keep:
            try:
                n.command("stop")
            except Exception as e:                         # noqa: BLE001  (the capture still gets written)
                print(f"stop: {e}", file=sys.stderr)
    rows = n.rows
    if not rows:
        print("no /mcu/state received", file=sys.stderr)
        return 1
    t0 = rows[0][0]
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"{c}{k}" for k in range(1, 5) for c in ("c", "w", "i", "e")] +
                   ["mode", "drive_mode", "faults"])
        for row in rows:
            w.writerow([f"{row[0] - t0:.6f}"] + row[1:])
    gaps = [b[0] - a_[0] for a_, b in zip(rows, rows[1:])]
    print(f"{len(rows)} samples over {rows[-1][0] - t0:.2f} s (largest gap {max(gaps) * 1e3:.1f} ms) -> {a.out}")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
