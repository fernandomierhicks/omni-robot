"""The lidar drive guard (mode_manager, Nov 23) on the full arm stack, driven like a person: the RC transmitter's
sticks on the sim's receiver (/sim/rc, protocol 10: through the robot computer and its guard), or the console's
stick (/console/cmd_vel at 20 Hz, as the page sends it). Sim only. Results: simulation/v11
sil/studies/results/guard_drive/README.md.

Each run: from where the robot is, take the world direction with the longest free run on the raw /scan, turn there
(the yaw stick, guard and all), then hold full stick in the run's body direction until the robot has rested 2 s (sim
time: the stack runs below real time) or timed out; optionally back straight off. Logs the truth pose, the MCU's
commanded and measured body speed (/mcu/state), the guard's state (/drive_guard/state). True clearance comes after,
on Windows, from the house model (studies/guard_drive_clear.py).

  ROS_DOMAIN_ID=42 OMNI_ROSBRIDGE_PORT=9190 OMNI_VIDEO_PORT=8181 OMNI_CONTROL_PORT=9973 \
      ros/omni.sh arm viewer:=false face:=false clutter:=off web:=false foxglove:=false mcu_port:=5771 servo_port:=5773
  ROS_DOMAIN_ID=42 (jazzy + ~/omni_ws/install sourced) ~/.venvs/omni-ros/bin/python ros/tools/guard_drive.py \
      [--input rc|console] [--out runs.json] 2:0 2:180:back 1:0:bypass accel:0
  run = speed:body_deg[:bypass][:back]  (bypass: hold the guard off, RC channel 6 / the console's bypass topic);
        accel:body_deg  (full stick from rest for 3 s in open space: the acceleration and top speed)
        slide:speed:approach_deg  (full stick forward at the nearest wall, approach_deg between the way and the
        wall: the slide)
"""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool, String

from omni_interfaces.msg import McuState
from omni_interfaces.srv import ConsoleCommand


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Driver(Node):
    def __init__(self, inp: str):
        super().__init__("guard_drive")
        self.inp = inp
        self.truth = None
        self.scan = None
        self.mcu = None
        self.guard = {}
        self.active = ""
        self.hist = None                            # a list while a run logs
        self.stick = (0.0, 0.0, 0.0)
        self.bypass = False
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 50)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), qos_profile_sensor_data)
        self.create_subscription(McuState, "/mcu/state", lambda m: setattr(self, "mcu", m), 50)
        self.create_subscription(String, "/drive_guard/state", self.on_guard, 10)
        self.create_subscription(String, "/cmd_vel/active", lambda m: setattr(self, "active", m.data), 5)
        self.p_rc = self.create_publisher(Joy, "/sim/rc", 10)
        self.p_stick = self.create_publisher(Twist, "/console/cmd_vel", 10)
        self.p_hb = self.create_publisher(String, "/console/heartbeat", 10)
        self.p_bypass = self.create_publisher(Bool, "/drive_guard/bypass", 10)
        self.cli = self.create_client(ConsoleCommand, "/console/command")
        self.create_timer(0.02, self.send)          # the RC receiver's frames / the page's stick
        self.create_timer(0.5, lambda: self.p_hb.publish(String(data=json.dumps({"id": "guard_drive"}))))

    def on_truth(self, m):
        p, v = m.pose.pose, m.twist.twist
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.truth = (t, p.position.x, p.position.y, yaw_of(p.orientation), v.linear.x, v.linear.y, v.angular.z)
        if self.hist is not None and (not self.hist or t > self.hist[-1][0] + 1e-6):
            mc = self.mcu
            g = self.guard
            self.hist.append(self.truth + (
                float("nan") if mc is None else math.hypot(mc.v_cmd_x, mc.v_cmd_y),
                float("nan") if mc is None else math.hypot(mc.vx, mc.vy),
                float(g.get("limit") or float("nan")), float(g.get("free") or float("nan")),
                {"clear": 0, "slowing": 1, "bypass": 2, "off": 3}.get(g.get("status"), -1)))

    def on_guard(self, m):
        try:
            self.guard = json.loads(m.data)
        except ValueError:
            pass

    def send(self):
        vx, vy, wz = self.stick
        if self.inp == "rc":
            # ch 1 right, ch 2 forward, ch 4 yaw right (the firmware's MANUAL mapping), ch 5 the drive switch on,
            # ch 6 the guard's bypass
            j = Joy()
            j.axes = [float(-vy), float(vx), 0.0, float(-wz), 1.0, 1.0 if self.bypass else -1.0, 0.0, 0.0]
            self.p_rc.publish(j)
        else:
            self.p_rc.publish(Joy())                    # the sim's RC link down: its drive switch would outrank us
            if any(abs(u) > 1e-6 for u in self.stick):
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
                self.p_stick.publish(t)
            if self.bypass:
                self.p_bypass.publish(Bool(data=True))

    def cmd(self, verb, **a):
        req = ConsoleCommand.Request(verb=verb, args=json.dumps(a))
        self.cli.wait_for_service(timeout_sec=10.0)
        for _ in range(4):
            f = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
            if f.result() is not None:
                return f.result().ok, f.result().message
        return False, "timeout"

    def spin_sim(self, dt):
        t0 = self.truth[0]
        while self.truth[0] - t0 < dt:
            rclpy.spin_once(self, timeout_sec=0.05)

    def free_dirs(self):
        """World directions (5 deg) and the free run there from the robot's centre, on the raw scan: the nearest
        return within the robot's swept width, less 0.24 m."""
        s, tr = self.scan, self.truth
        r = np.array(s.ranges)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > max(s.range_min, 0.35))   # nearer: the robot's own posts
        x, y, yaw = tr[1], tr[2], tr[3]
        lx, ly = x + 0.14 * math.cos(yaw), y + 0.14 * math.sin(yaw)
        px, py = lx + r[ok] * np.cos(a[ok] + yaw), ly + r[ok] * np.sin(a[ok] + yaw)
        out = []
        for d in np.radians(np.arange(0, 360, 5)):
            ux, uy = math.cos(d), math.sin(d)
            along = (px - x) * ux + (py - y) * uy
            across = np.abs(-(px - x) * uy + (py - y) * ux)
            k = (along > 0) & (across < 0.30)
            out.append((float(d), float(along[k].min()) - 0.24 if k.any() else 0.0))
        return out

    def turn_to(self, yaw_goal, timeout=20.0):
        t0 = self.truth[0]
        while self.truth[0] - t0 < timeout:
            e = math.atan2(math.sin(yaw_goal - self.truth[3]), math.cos(yaw_goal - self.truth[3]))
            if abs(e) < 0.03:
                break
            u = max(-1.0, min(1.0, 1.2 * e))
            self.stick = (0.0, 0.0, u if abs(u) > 0.08 else math.copysign(0.08, u))
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stick = (0.0, 0.0, 0.0)
        self.spin_sim(1.0)

    def run(self, speed, body_deg, bypass=False, accel=False):
        """Full stick (scaled to speed of the RC's 2 m/s full travel) in body direction body_deg."""
        world, free = max(self.free_dirs(), key=lambda d: d[1])
        self.turn_to(world - math.radians(body_deg))
        free = dict(self.free_dirs()).get(world, 0.0)
        b = math.radians(body_deg)
        k = min(1.0, speed / 2.0) if self.inp == "rc" else speed
        self.bypass = bypass
        self.hist = []
        self.stick = (k * math.cos(b), k * math.sin(b), 0.0)
        t0 = self.truth[0]
        timeout = 3.0 if accel else max(free, 1.0) / max(speed, 0.1) + 8.0
        moved, still = False, None
        while self.truth[0] - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            sp = math.hypot(self.truth[4], self.truth[5])
            moved = moved or sp > 0.2
            if moved and sp < 0.05:
                still = still or self.truth[0]
                if self.truth[0] - still > 2.0:
                    break
            else:
                still = None
        self.stick = (0.0, 0.0, 0.0)
        self.spin_sim(1.0)
        self.bypass = False
        H = np.array(self.hist, float)
        self.hist = None
        return dict(speed=speed, body_deg=body_deg, bypass=bypass, accel=accel, world_dir=round(world, 4),
                    free_run_m=round(free, 2), active=self.active, input=self.inp,
                    trace=[[round(float(v), 4) for v in row] for row in H])

    def run_slide(self, speed, approach_deg):
        """Full stick at a wall, the way approach_deg to it (the guard's slide, Nov 24): the wall is the nearest return
        straight out 0.6-4 m away with the longest run along it; the robot faces the way and holds forward until it
        has rested 2 s or timed out. Logs the wall's normal (world) and its start gap from the rim."""
        dirs = [(d, f if f != 0.0 else 8.0) for d, f in self.free_dirs()]   # 0: nothing in the way (open)

        def free_at(a):
            return min(dirs, key=lambda df: abs(math.atan2(math.sin(df[0] - a), math.cos(df[0] - a))))[1]

        best = None
        for d, f in dirs:
            step = math.radians(5)
            if not 0.6 <= f <= 4.0 or f > free_at(d - step) or f > free_at(d + step):
                continue                                # a wall's normal: the free run's local minimum
            for side in (1, -1):
                along = free_at(d + side * math.pi / 2)
                if best is None or along > best[2]:
                    best = (d, side, along, f)
        if best is None:
            raise RuntimeError("no wall 0.6-4 m away to slide along: " +
                               " ".join(f"{math.degrees(d):.0f}:{f:.2f}" for d, f in dirs))
        normal, side, along, gap0 = best
        way = normal + side * math.radians(90 - approach_deg)
        self.turn_to(way)
        x0, y0 = self.truth[1], self.truth[2]
        k = min(1.0, speed / 2.0) if self.inp == "rc" else speed
        self.hist = []
        self.stick = (k, 0.0, 0.0)
        t0 = self.truth[0]
        moved, still = False, None
        while self.truth[0] - t0 < (gap0 + along) / max(speed, 0.1) + 8.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            sp = math.hypot(self.truth[4], self.truth[5])
            moved = moved or sp > 0.2
            if moved and sp < 0.05:
                still = still or self.truth[0]
                if self.truth[0] - still > 2.0:
                    break
            else:
                still = None
        self.stick = (0.0, 0.0, 0.0)
        self.spin_sim(1.0)
        H = np.array(self.hist, float)
        self.hist = None
        b = normal + math.pi - self.truth[3]                # off the wall again, for the next run
        self.stick = (0.5 * math.cos(b), 0.5 * math.sin(b), 0.0)
        self.spin_sim(2.0)
        self.stick = (0.0, 0.0, 0.0)
        self.spin_sim(1.0)
        return dict(speed=speed, approach_deg=approach_deg, slide=True, normal=round(normal, 4), side=side,
                    gap0=round(gap0, 3), along_free=round(along, 2), x0=x0, y0=y0, active=self.active,
                    input=self.inp, trace=[[round(float(v), 4) for v in row] for row in H])

    def back_off(self, speed, body_deg):
        b = math.radians(body_deg + 180)
        k = min(1.0, speed / 2.0) if self.inp == "rc" else speed
        self.hist = []
        x0, y0 = self.truth[1], self.truth[2]
        self.stick = (k * math.cos(b), k * math.sin(b), 0.0)
        self.spin_sim(1.0)
        self.stick = (0.0, 0.0, 0.0)
        self.spin_sim(1.0)
        H = np.array(self.hist, float)
        self.hist = None
        return dict(peak=round(float(np.hypot(H[:, 4], H[:, 5]).max()), 2),
                    moved_m=round(math.hypot(self.truth[1] - x0, self.truth[2] - y0), 2))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--input", choices=("rc", "console"), default="rc")
    ap.add_argument("--out", default="guard_drive_runs.json")
    ap.add_argument("runs", nargs="*", default=["2:0"])
    a = ap.parse_args()
    rclpy.init()
    n = Driver(a.input)
    t_end = time.monotonic() + 300
    while time.monotonic() < t_end and (n.truth is None or n.scan is None or n.mcu is None or not n.guard
                                        or n.guard.get("status") == "no_lidar"):
        rclpy.spin_once(n, timeout_sec=0.2)
    print("ready:", n.guard, flush=True)
    if a.input == "console":
        print("mission", n.cmd("mission", name="MANUAL"), flush=True)
        n.spin_sim(1.0)
    out = []
    for spec in a.runs:
        p = spec.split(":")
        if p[0] == "accel":
            res = n.run(2.0, float(p[1]), accel=True)
        elif p[0] == "slide":
            try:
                res = n.run_slide(float(p[1]), float(p[2]))
            except RuntimeError as e:
                print(spec, "skipped:", e, flush=True)
                continue
        else:
            res = n.run(float(p[0]), float(p[1]), bypass="bypass" in p)
            if "back" in p:
                res["back_off"] = n.back_off(float(p[0]), float(p[1]))
        res["label"] = spec
        T = np.array(res["trace"]) if res["trace"] else np.zeros((1, 12))
        print(json.dumps(dict(label=spec, active=res["active"], free_run_m=res.get("free_run_m", res.get("gap0")),
                              peak=round(float(np.nanmax(np.hypot(T[:, 4], T[:, 5]))), 2),
                              back_off=res.get("back_off"))), flush=True)
        out.append(res)
        json.dump(out, open(a.out, "w"))                # each run kept as it lands
    if a.input == "console":
        n.cmd("mission", name="IDLE")
    json.dump(out, open(a.out, "w"))
    print("RESULTS ->", a.out, flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
