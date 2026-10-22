"""Save slam_toolbox's map (design.md 28): maps/<name>.pgm + .yaml (occupancy grid, for Nav2 and scoring) and
<name>.posegraph + .data (the serialized pose graph, for slam_toolbox's localization mode), plus <name>_origin.json
(the map frame's pose in the world, for scoring).

  ros2 run omni_bringup save_map --ros-args -p name:=house                 now
  ... -p wait_for_done:=true                                              after the auto-mapping tour ends
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String

from omni_bringup import maps_dir

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Saver(Node):
    def __init__(self):
        super().__init__("map_saver_omni")
        self.name = self.declare_parameter("name", "house").value
        self.wait = self.declare_parameter("wait_for_done", False).value
        self.settle = self.declare_parameter("settle", 3.0).value      # s of sim time after "done" (last scans in)
        self.origin, self.phase, self.t_done = None, None, None
        self.create_subscription(PoseStamped, "/sim/origin", self.on_origin, LATCHED)
        self.create_subscription(String, "/sim/phase", self.on_phase, LATCHED)
        self.c_save = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self.c_ser = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

    def on_origin(self, m):
        q = m.pose.orientation
        self.origin = (m.pose.position.x, m.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

    def on_phase(self, m):
        self.phase = m.data
        if m.data == "done":
            self.t_done = self.get_clock().now()

    def run(self):
        log = self.get_logger()
        if self.wait:
            log.info("waiting for the tour to finish")
            while rclpy.ok() and (self.t_done is None or
                                  (self.get_clock().now() - self.t_done).nanoseconds * 1e-9 < self.settle):
                rclpy.spin_once(self, timeout_sec=0.1)
        for c in (self.c_save, self.c_ser):
            while not c.wait_for_service(timeout_sec=1.0):
                log.info(f"waiting for {c.srv_name}")
        base = str(maps_dir() / self.name)
        r = SaveMap.Request()
        r.name = String(data=base)
        f = self.c_save.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30.0)
        log.info(f"save_map -> {base}.pgm/.yaml: {f.result()}")
        r2 = SerializePoseGraph.Request()
        r2.filename = base
        f2 = self.c_ser.call_async(r2)
        rclpy.spin_until_future_complete(self, f2, timeout_sec=60.0)
        log.info(f"serialize_map -> {base}.posegraph/.data: {f2.result()}")
        t_end = self.get_clock().now().nanoseconds * 1e-9 + 2.0
        while self.origin is None and self.get_clock().now().nanoseconds * 1e-9 < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        (maps_dir() / f"{self.name}_origin.json").write_text(json.dumps(dict(world_from_map=self.origin)))


def main():
    rclpy.init()
    node = Saver()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
