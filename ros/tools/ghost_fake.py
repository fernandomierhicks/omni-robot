"""A fake camera head for the ghost-ball path (omni_sim ghost_ball), to check it without a ball or a Pi.

Publishes /ghost/track like the Pi's camera head would for a lob thrown toward the robot (the head's robot frame, system
clock stamps, 120 Hz, a ready track after 8 frames), then listens to /catch/track (what ghost_ball hands the robot's
catch_behavior) and /ghost/episode, and prints what came out. In WSL, with `omni.sh catch ghost:=true` running:

  ROS_DOMAIN_ID=.. python3 ghost_fake.py [--throws 2]
"""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from omni_interfaces.msg import BallTrack
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def lob(t, x0=4.0, vx=-3.6, vz=4.9, z0=1.0, g=9.81):
    return np.array([x0 + vx * t, 0.1, z0 + vz * t - 0.5 * g * t * t, vx, 0.0, vz - g * t])


class Fake(Node):
    def __init__(self, throws: int):
        super().__init__("ghost_fake")
        be = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(BallTrack, "/ghost/track", be)
        self.got, self.episodes, self.throws = [], [], throws
        self.create_subscription(BallTrack, "/catch/track", lambda m: self.got.append((time.time(), m)), 50)
        self.create_subscription(String, "/ghost/episode", lambda m: self.episodes.append(json.loads(m.data)), 10)

    def fly(self, T=1.0, hz=120.0):
        t0 = time.time()
        k = 0
        while time.time() - t0 < T and rclpy.ok():
            t = time.time() - t0
            m = BallTrack()
            m.header.frame_id = "odom"
            now = time.time()
            m.header.stamp.sec, m.header.stamp.nanosec = int(now), int((now % 1) * 1e9)
            m.tracking, m.frames, m.ready = True, k, k >= 8
            m.x = [float(v) for v in lob(t)]
            m.covariance = [float(v) for v in (np.eye(6) * 0.01).reshape(-1)]
            m.crossing, m.t_go, m.crossing_sigma = [float(1.0 + 0.5 * t), 0.0, 0.0], max(0.0, 1.0 - t), 0.05
            self.pub.publish(m)
            k += 1
            rclpy.spin_once(self, timeout_sec=1.0 / hz)
        end = BallTrack()
        end.header.frame_id = "odom"
        self.pub.publish(end)             # tracking False: the throw is over


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--throws", type=int, default=2)
    a = ap.parse_args()
    rclpy.init()
    n = Fake(a.throws)
    time.sleep(3.0)                       # discovery
    for _ in range(a.throws):
        n.fly()
        t_end = time.time() + 2.0
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.05)
    tracking = [m for _, m in n.got if m.tracking]
    print(f"ghost_fake: {len(tracking)} tracking messages came back on /catch/track, {len(n.episodes)} episodes")
    if tracking:
        m = tracking[len(tracking) // 2]
        print(f"  a mid-flight sample: x = {[round(v, 2) for v in m.x]} ready={m.ready} frames={m.frames}")
    for e in n.episodes:
        print("  episode:", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items() if k != "crossing"})
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
