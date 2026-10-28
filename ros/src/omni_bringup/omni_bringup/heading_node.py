"""heading_node: which way the robot faces while Nav2 drives (the core and the why are in heading.py).

Sits between Nav2's velocity smoother and its collision monitor:
Subscribes cmd_vel_shape          geometry_msgs/Twist: the smoother's output (remapped there in stack.nav2_nodes)
           /odometry/filtered     nav_msgs/Odometry: the yaw and turn rate (odom frame)
           /plan                  nav_msgs/Path: the path left to the goal (Nav2 replans at 1 Hz)
Publishes  cmd_vel_smoothed       geometry_msgs/Twist: to the collision monitor; when agnostic each input unchanged,
                                  else the shaped command at `rate` Hz while Nav2's command is fresh
Parameters heading (agnostic | forward | spinning), spin_rate (rad/s), turn_max (rad/s), lead (s), rate (Hz); the console
           sets heading, spin_rate and turn_max at runtime (its Go To settings).
Wall time, not sim time: like the console's nodes it needs no stamps, and the sim's 1 kHz /clock would cost a Python
node most of a core.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from omni_bringup.heading import MODES, HeadingConfig, HeadingShaper


class HeadingNode(Node):
    def __init__(self):
        super().__init__("heading_node")
        cfg = HeadingConfig()
        P = self.declare_parameter
        cfg.mode = P("heading", cfg.mode).value
        cfg.spin_rate = P("spin_rate", cfg.spin_rate).value
        cfg.turn_max = P("turn_max", cfg.turn_max).value
        cfg.lead = P("lead", cfg.lead).value
        rate = P("rate", 50.0).value
        cfg.period = 1.0 / rate
        if cfg.mode not in MODES:
            raise ValueError(f"heading: one of {', '.join(MODES)}")
        self.core = HeadingShaper(cfg)
        self.add_on_set_parameters_callback(self.on_params)
        self.pub = self.create_publisher(Twist, "cmd_vel_smoothed", 10)
        self.create_subscription(Twist, "cmd_vel_shape", self.on_cmd, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 1)
        self.create_timer(cfg.period, self.tick)
        self.get_logger().info(f"heading {cfg.mode}")

    def on_params(self, params):
        cfg = self.core.cfg
        for p in params:
            if p.name == "heading":
                if p.value not in MODES:
                    return SetParametersResult(successful=False, reason=f"heading: one of {', '.join(MODES)}")
                if p.value != cfg.mode:
                    self.get_logger().info(f"heading {p.value}")
                cfg.mode = p.value
            elif p.name in ("spin_rate", "turn_max", "lead"):
                setattr(cfg, p.name, float(p.value))
        return SetParametersResult(successful=True)

    def publish(self, cmd):
        if cmd is None:
            return
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = cmd
        self.pub.publish(m)

    def on_cmd(self, m: Twist):
        self.publish(self.core.on_cmd(time.monotonic(), m.linear.x, m.linear.y, m.angular.z))

    def on_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.core.on_odom(time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                          m.twist.twist.angular.z)

    def on_plan(self, m: Path):
        p = m.poses
        length = sum(math.hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y)
                     for a, b in zip(p, p[1:]))
        self.core.on_plan(time.monotonic(), length)

    def tick(self):
        now = time.monotonic()
        if self.core.active(now) and now - self.core.t_out > 0.015:     # not right after an input's own publish
            self.publish(self.core.step(now))


def main():
    rclpy.init()
    node = HeadingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
