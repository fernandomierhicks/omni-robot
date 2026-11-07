"""floor_obstacles: the depth camera's floor for Nav2 (floor_obstacles.py; design.md 4.5, 4.11).

Subscribes depth_topic                   sensor_msgs/Image, the depth camera's frames (+ its camera_info): the D435's
                                         /d435/depth/image_rect_raw 16UC1 mm with config:=d435
           /map                          nav_msgs/OccupancyGrid: the map's geometry (the grid uses its cells)
           /arm/out                      std_msgs/Bool: the arm is out (arm_node, or omni_cpp's arm_driver)
           /tf                           map -> base_link at each frame's stamp
Publishes  /floor_obstacles/grid         nav_msgs/OccupancyGrid (latched), the map's cells: 100 where a thing on the
                                         floor or something tall is, -1 elsewhere; Nav2's StaticLayer in both
                                         costmaps (stack.FLOOR_LAYER). At most publish_rate a second, when it changed

While the arm is out (a pick, a look) its own links are in the camera's view: frames are skipped. When it comes back
in, the grid forgets everything within forget_radius of the robot: the thing it just picked, whose spot the camera may
not see again (under its view at 7.5 deg), would otherwise block a later goal or stand-off. Every frame it also forgets
the marks under the robot's outline (Nav2's footprint clearing: a mark there puts every plan's start in collision).

Parameters config ("" | d435 | a json path: the camera's pose in base_link, robot_arm*.json), depth_topic,
depth_info_topic, rate (5 Hz), stride (1: every pixel; at 2 floor_vision's 1 cm grid breaks a 15 mm brick up),
forget_radius (0.5 m), publish_rate (2 Hz), map_frame (map). ~50 ms a D435 frame on the desktop PC (Python, one core).
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from omni_params import Params
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from omni_arm.floor_obstacles import ROBOT_RADIUS, ObstacleGrid, floor_obstacles, to_frame
from omni_arm.floor_vision import DepthFrame, decode_depth
from omni_arm.kinematics import ArmConfig, config_path
from omni_arm.pick import MJ_TO_OPTICAL


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class FloorObstacles(Node):
    def __init__(self):
        super().__init__("floor_obstacles")
        p = Params(self)                                   # described parameters (omni_params; robot.yaml)
        config = p.text("config", "d435", "the robot_arm*.json: d435 | \"\" (stereo) | a path")
        depth_topic = p.text("depth_topic", "/d435/depth/image_rect_raw" if config == "d435" else "/stereo/depth",
                             "the depth image")
        info_topic = p.text("depth_info_topic", "", "its camera_info (\"\" = beside depth_topic)") or \
            depth_topic.rsplit("/", 1)[0] + "/camera_info"
        hz_to_dt = lambda hz: 1.0 / max(float(hz), 0.1)   # noqa: E731
        self.dt = p.number("rate", 5.0, "Hz: depth frames used at most", (0.1, 30.0), live=self, attr="dt",
                           convert=hz_to_dt)
        self.stride = p.number("stride", 1, "use every n-th depth pixel (CPU against detail)", (1, 16), live=self)
        self.forget_r = p.number("forget_radius", 0.5, "m: marks this near the robot forgotten after a pick",
                                 (0.0, 3.0), live=self, attr="forget_r")
        self.pub_dt = p.number("publish_rate", 2.0, "Hz: the grid to Nav2 at most", (0.1, 10.0), live=self,
                               attr="pub_dt", convert=hz_to_dt)
        self.map_frame = p.text("map_frame", "map", "the grid's frame")
        c = ArmConfig(config_path(config)).cameras[0]
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = np.array(c["R"]), c["pos"]
        self.T = T @ MJ_TO_OPTICAL
        self.K = None
        self.out = False
        self.t_last = -1e9
        self.rng = np.random.default_rng(0)
        self.grid = ObstacleGrid()
        self.dirty, self.t_pub = False, -1e9
        self.ms = []
        self.skip = {"no map": 0, "no pose": 0}
        self.pending = None                # (stamp, t, depth): a frame waiting for TF at its stamp
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)       # on this node's executor (one thread: no blocking waits)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, "/floor_obstacles/grid", latched)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(CameraInfo, info_topic, self.on_info,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Image, depth_topic, self.on_depth, 2)
        self.create_subscription(Bool, "/arm/out", self.on_out, 10)
        self.create_timer(0.05, self.process)
        self.create_timer(0.1, self.publish)
        self.create_timer(30.0, self.report)
        self.get_logger().info(f"floor obstacles from {depth_topic} ({c['name']}) at up to {1 / self.dt:g} Hz "
                               f"-> /floor_obstacles/grid")

    def on_info(self, m: CameraInfo):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_map(self, m: OccupancyGrid):
        o = m.info.origin.position
        if self.grid.geometry(m.info.resolution, o.x, o.y, m.info.width, m.info.height):
            self.dirty = True

    def pose(self, stamp) -> tuple | None:
        """The base in the map frame at stamp (x, y, yaw; a zero stamp: the latest), or None (not there yet)."""
        try:
            tr = self.tf.lookup_transform(self.map_frame, "base_link", Time.from_msg(stamp))
        except Exception:                               # not localized yet, TF not at the stamp yet, or too old
            return None
        p, q = tr.transform.translation, tr.transform.rotation
        return p.x, p.y, _yaw(q)

    def on_out(self, m: Bool):
        if self.out and not m.data:
            pose = self.pose(Time().to_msg())
            if pose is not None and self.grid.forget(pose[0], pose[1], self.forget_r):
                self.dirty = True
            self.get_logger().info(f"arm in: marks within {self.forget_r:g} m of the robot forgotten")
        self.out = bool(m.data)

    def on_depth(self, m: Image):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.out or self.K is None or t - self.t_last < self.dt * 0.95:
            return
        if self.grid.res is None:
            self.skip["no map"] += 1
            return
        z = decode_depth(m.encoding, bytes(m.data), m.height, m.width)
        if z is not None:
            self.t_last = t
            if self.pending is not None:
                self.skip["no pose"] += 1                # the last one never got its pose
            self.pending = (m.header.stamp, t, z)
            self.process()

    def process(self):
        """The pending frame, once TF has the base at its stamp (dropped after 1 s: not localized)."""
        if self.pending is None:
            return
        stamp, t, z = self.pending
        pose = self.pose(stamp)
        if pose is None:
            if self.get_clock().now().nanoseconds * 1e-9 - t > 1.0:
                self.pending = None
                self.skip["no pose"] += 1
            return
        self.pending = None
        if self.out:
            return
        w0 = time.perf_counter()
        things, tall, floor, _ = floor_obstacles(DepthFrame(t, z, self.K, self.T), self.stride, self.rng)
        if self.grid.update(t, to_frame(things, pose), to_frame(tall, pose), to_frame(floor, pose)):
            self.dirty = True
        if self.grid.forget(pose[0], pose[1], ROBOT_RADIUS):         # under the robot: unseen, and in every plan's way
            self.dirty = True
        self.ms.append(1000 * (time.perf_counter() - w0))

    def publish(self):
        now = time.monotonic()
        if not self.dirty or now - self.t_pub < self.pub_dt or self.grid.res is None:
            return
        g = self.grid
        m = OccupancyGrid()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = float(g.res)
        m.info.height, m.info.width = g.kind.shape
        m.info.origin.position.x, m.info.origin.position.y = g.origin
        m.info.origin.orientation.w = 1.0
        m.data = g.occupancy().tolist()
        self.pub.publish(m)
        self.dirty, self.t_pub = False, now

    def report(self):
        skipped = ", ".join(f"{v} {k}" for k, v in self.skip.items() if v)
        if self.ms or skipped:
            ms = f", {np.median(self.ms):.0f} ms median, {max(self.ms):.0f} max" if self.ms else ""
            self.get_logger().info(f"{len(self.ms)} frames in 30 s{ms}{f' (skipped: {skipped})' if skipped else ''}; "
                                   f"{self.grid.marked()} cells marked")
        self.ms = []
        self.skip = dict.fromkeys(self.skip, 0)


def main():
    rclpy.init()
    n = FloorObstacles()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
