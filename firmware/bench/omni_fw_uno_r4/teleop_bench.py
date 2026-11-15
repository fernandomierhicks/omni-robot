"""Drive the bench MCU as the robot would: /set_mode TELEOP, then twists on /cmd_vel/manual (the console stick's topic)
through mode_manager -> /mcu/cmd -> mcu_link -> the firmware core -> CAN. Records /mcu/state and checks the wheel
odometry against the command.

Run by ros_bench.sh (mcu_link + mode_manager already up). The real AK40-10 is wheel 1 (CAN id 1); wheels 2-4 are the
bench firmware's virtual motors. Odometry is the pseudo-inverse over all four, so for a pure yaw command a stopped real
wheel would read 75% of the command, a wheel that tracks 100%: the measured/commanded ratio is the real motor's check.
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from omni_interfaces.msg import McuInfo, McuState
from omni_interfaces.srv import SetMode
from rclpy.node import Node

# (label, vx m/s, vy m/s, wz rad/s, seconds). Wheel 1's speed = J[0] . (vx, vy, wz) = -10.10 vx + 10.10 vy + 2.086 wz
PHASES = [
    ("rest", 0.0, 0.0, 0.0, 1.5),
    ("yaw +0.5", 0.0, 0.0, 0.5, 4.0),      # wheel 1 +1.04 rad/s (~10 rpm)
    ("rest", 0.0, 0.0, 0.0, 2.0),
    ("yaw -1.0", 0.0, 0.0, -1.0, 4.0),     # wheel 1 -2.09 rad/s (~20 rpm)
    ("rest", 0.0, 0.0, 0.0, 2.0),
    ("vx +0.15", 0.15, 0.0, 0.0, 4.0),     # wheel 1 -1.52 rad/s
    ("rest", 0.0, 0.0, 0.0, 2.0),
]


class Bench(Node):
    def __init__(self, out_csv):
        super().__init__("teleop_bench")
        self.pub = self.create_publisher(Twist, "/cmd_vel/manual", 10)
        self.cli = self.create_client(SetMode, "/set_mode")
        self.rows = []
        self.info = None
        self.phase = "start"
        self.create_subscription(McuState, "/mcu/state", self.on_state, 50)
        self.create_subscription(McuInfo, "/mcu/info", lambda m: setattr(self, "info", m), 5)
        self.out_csv = out_csv

    def on_state(self, m):
        self.rows.append((time.time(), self.phase, m.mode, "|".join(m.faults), list(m.dm_err), m.vx, m.vy, m.wz,
                          m.v_cmd_x, m.v_cmd_y, m.w_cmd, m.slip, m.yaw, m.v_bus))

    def set_mode(self, mode, kid=True):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/set_mode not available")
        f = self.cli.call_async(SetMode.Request(mode=mode, kid=kid))
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        r = f.result()
        print(f"set_mode {mode} kid={kid}: {r.success} {r.message}", flush=True)
        return r.success

    def spin_for(self, s, twist=None):
        end = time.time() + s
        while time.time() < end:
            if twist is not None:
                self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)


def summarize(rows, phases):
    print(f"{'phase':10} {'cmd':>18} {'odom (vx, vy, wz)':>24} {'ratio':>6} {'slip':>6}  mode / faults")
    for i, (label, vx, vy, wz, _) in enumerate(phases):
        rs = [r for r in rows if r[1] == f"{label}#{i}"]
        if not rs:
            continue
        tail = rs[len(rs) // 2:]                      # the second half: past the ramp
        mvx = sum(r[5] for r in tail) / len(tail)
        mvy = sum(r[6] for r in tail) / len(tail)
        mwz = sum(r[7] for r in tail) / len(tail)
        slip = max(r[11] for r in tail)
        big = max(abs(vx), abs(vy), abs(wz))
        ratio = "" if big == 0 else f"{(mwz / wz if wz else mvx / vx if vx else mvy / vy):6.2f}"
        modes = sorted({r[2] for r in rs})
        faults = sorted({f for r in rs for f in r[3].split("|") if f})
        print(f"{label:10} ({vx:5.2f},{vy:5.2f},{wz:5.2f}) ({mvx:6.3f},{mvy:6.3f},{mwz:6.3f}) {ratio:>6} {slip:6.3f}  "
              f"{','.join(modes)} {faults}")


def main():
    out_csv = sys.argv[1] if len(sys.argv) > 1 else "teleop_bench.csv"
    rclpy.init()
    n = Bench(out_csv)
    n.spin_for(2.0)                                   # discovery, first STATEs
    if not n.rows:
        print("no /mcu/state: is mcu_link up?")
        return 1
    ok = n.set_mode("TELEOP", kid=True)
    try:
        if ok:
            for i, (label, vx, vy, wz, s) in enumerate(PHASES):
                n.phase = f"{label}#{i}"
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
                print(f"-- {label}: vx {vx} vy {vy} wz {wz} for {s} s", flush=True)
                n.spin_for(s, t)
    finally:
        n.phase = "idle"
        n.set_mode("IDLE", kid=True)
        n.spin_for(1.0)
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "phase", "mode", "faults", "dm_err", "vx", "vy", "wz", "v_cmd_x", "v_cmd_y", "w_cmd", "slip",
                    "yaw", "v_bus"])
        w.writerows(n.rows)
    summarize(n.rows, PHASES)
    if n.info:
        print(f"INFO protocol {n.info.protocol} build {n.info.build} motor C {list(n.info.motor_temp_c)}")
    yaw_total = math.degrees(n.rows[-1][12]) if n.rows else float("nan")
    print(f"{len(n.rows)} STATEs recorded -> {out_csv}; final odom yaw {yaw_total:.1f} deg")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
