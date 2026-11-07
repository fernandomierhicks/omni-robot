"""ball_track: the Goalie's ball (ball_track.py) from the D435, only while the console's Goalie plays. Asleep it holds no
camera subscription and costs nothing; /goalie/enable true wakes it, false (or the console gone quiet) puts it back.

Subscribes /goalie/enable           std_msgs/Bool (latched by the console's last word): run or sleep
           color_topic (rgb8, registered to the depth: /d435/color/image_raw), depth_topic (+ camera_info): the D435's
           /d435/depth/image_rect_raw 16UC1 mm, the same pixels (the sim renders them so; the real D435 aligns its
           depth to its colour stream: design.md 2.1, AI control's phase 5 needs the same), /odom: where the robot was
Publishes  /goalie/ball             std_msgs/String JSON, each frame pair: {seen, t, x, y, vx, vy, age, n} in odom
Parameters config ("d435" | a robot_arm*.json path: the camera's pose in base_link), color_topic, depth_topic,
           depth_info_topic, odom_topic, the ball's color; live: diameter_m, min_pixels, max_range_m, stride (taken
           when it wakes)
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from omni_params import Params
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from omni_arm.ball_track import BallParams, BallTracker
from omni_arm.floor_vision import DepthFrame, decode_depth
from omni_arm.kinematics import ArmConfig, config_path
from omni_arm.pick import MJ_TO_OPTICAL


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class BallTrackNode(Node):
    def __init__(self):
        super().__init__("ball_track")
        p = Params(self)                                   # described parameters (omni_params; robot.yaml)
        config = p.text("config", "d435", "the robot_arm*.json with the depth camera's pose: d435 | a path")
        self.color_topic = p.text("color_topic", "/d435/color/image_raw", "rgb8 registered to the depth")
        self.depth_topic = p.text("depth_topic", "/d435/depth/image_rect_raw", "the depth image")
        self.info_topic = p.text("depth_info_topic", "", "its camera_info (\"\" = beside depth_topic)") or \
            self.depth_topic.rsplit("/", 1)[0] + "/camera_info"
        self.odom_topic = p.text("odom_topic", "/odom", "the robot's odometry")
        self.bp = BallParams()
        self.bp.color = p.text("color", "red", "the ball's colour: red | orange | yellow | green | blue | purple | pink")
        self.bp.diameter = p.number("diameter_m", 0.22, "m: the ball's diameter", (0.05, 0.4), live=self,
                                    attr="ball_diameter")
        self.bp.min_px = p.number("min_pixels", 40, "fewer ball-coloured pixels with depth: not seen", (5, 2000),
                                  live=self, attr="ball_min_px")
        self.bp.max_range = p.number("max_range_m", 4.5, "m: farther is not the ball", (1.0, 8.0), live=self,
                                     attr="ball_max_range")
        self.stride = p.number("stride", 2, "use every n-th pixel (CPU against range)", (1, 8), live=self)
        c = ArmConfig(config_path(config)).cameras[0]
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = np.array(c["R"]), c["pos"]
        self.T = T @ MJ_TO_OPTICAL
        self.K = None
        self.tracker = None
        self.on = False
        self.subs = []
        self.pose = None
        self.color = None                                  # (t, rgb) the newest colour frame
        self.pub = self.create_publisher(String, "/goalie/ball", 5)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/goalie/enable", self.on_enable, latched)
        self.get_logger().info(f"ball tracker asleep (wakes on /goalie/enable); camera {c['name']}")

    def on_enable(self, m: Bool):
        if bool(m.data) == self.on:
            return
        self.on = bool(m.data)
        if self.on:
            for k, v in (("diameter", "ball_diameter"), ("min_px", "ball_min_px"), ("max_range", "ball_max_range")):
                setattr(self.bp, k, getattr(self, v, getattr(self.bp, k)))
            self.bp.min_px = max(5, int(self.bp.min_px // max(1, int(self.stride)) ** 2))
            self.tracker = BallTracker(self.bp)
            ci = self.create_subscription
            info_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subs = [ci(Image, self.color_topic, self.on_color, 2), ci(Image, self.depth_topic, self.on_depth, 2),
                         ci(CameraInfo, self.info_topic, self.on_info, info_qos),
                         ci(Odometry, self.odom_topic, self.on_odom, 20)]
            self.get_logger().info(f"ball tracker on: a {self.bp.color} ball, {100 * self.bp.diameter:.0f} cm")
        else:
            for s in self.subs:
                self.destroy_subscription(s)
            self.subs, self.tracker, self.color = [], None, None
            self.get_logger().info("ball tracker asleep")

    def on_info(self, m: CameraInfo):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_odom(self, m: Odometry):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, _yaw(p.orientation))

    def on_color(self, m: Image):
        if m.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        self.color = img[..., ::-1] if m.encoding == "bgr8" else img

    def on_depth(self, m: Image):
        if self.tracker is None or self.K is None or self.color is None or self.pose is None:
            return
        z = decode_depth(m.encoding, bytes(m.data), m.height, m.width)
        if z is None:
            return
        k = max(1, int(self.stride))
        fx, fy, cx, cy = self.K
        K = (fx / k, fy / k, cx / k, cy / k)
        f = DepthFrame(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, z[::k, ::k], K, self.T)
        rgb = self.color[::max(1, self.color.shape[0] // f.depth.shape[0]), ::max(1, self.color.shape[1] // f.depth.shape[1])]
        st = self.tracker.step(self.get_clock().now().nanoseconds * 1e-9, f, rgb, self.pose)
        self.pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = BallTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
