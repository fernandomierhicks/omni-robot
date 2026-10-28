"""robot_frames and frame_check (design.md 10). The cores are in frames.py.

robot_frames  /tf_static: base_link -> every sensor and mount (omni_catch's config/robot_v11.json "frames", generated
              from the sim; calibration results override within bounds), /tf: module_pivot -> module at the measured
              tilt (/mcu/state, at most 50 Hz). /frames/report (latched JSON): what was applied or refused.
              Parameters: config (default: omni_catch's robot_v11.json), calibration (a frames.json from omni_calib;
              "" = none), publish_static (false in the sim, where the bridge publishes the model's own frames).
frame_check   records /tf and /tf_static for `duration` s, then publishes /frames/check (latched JSON) and logs it:
              REP-105 problems (two parents, a broken map -> odom -> base_link, a sensor off base_link, an optical
              frame that isn't REP-103) and jumps in odom -> base_link (must be none) and map -> odom (corrections).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from omni_interfaces.msg import McuState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from omni_bringup.frames import R_to_quat, apply_calibration, check_motion, check_tree, load_frames, quat_to_R

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def default_config() -> str:
    from ament_index_python.packages import get_package_share_directory
    try:
        import omni_catch
        return str(Path(omni_catch.__file__).resolve().parent / "config" / "robot_v11.json")
    except ImportError:
        return str(Path(get_package_share_directory("omni_catch")) / "config" / "robot_v11.json")


def tf_msg(parent, child, xyz, quat, stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    r = t.transform.rotation
    r.x, r.y, r.z, r.w = map(float, quat)
    return t


class RobotFrames(Node):
    def __init__(self):
        super().__init__("robot_frames")
        P = self.declare_parameter
        frames = load_frames(P("config", "").value or default_config())
        cal_path = P("calibration", "").value
        report = []
        if cal_path and Path(cal_path).exists():
            frames, report = apply_calibration(frames, json.loads(Path(cal_path).read_text()))
            for r in report:
                (self.get_logger().info if r["applied"] else self.get_logger().warn)(
                    f"calibration of {r['frame']}: {r['moved_mm']} mm, {r['turned_deg']} deg from the drawing: "
                    + ("applied" if r["applied"] else "REFUSED (outside its bounds: check the mount, recalibrate)"))
        self.frames = frames
        now = self.get_clock().now().to_msg()
        if P("publish_static", True).value:
            self.static = StaticTransformBroadcaster(self)
            self.static.sendTransform([tf_msg(f.parent, f.child, f.xyz, R_to_quat(f.R), now) for f in frames])
        self.p_report = self.create_publisher(String, "/frames/report", LATCHED)
        self.p_report.publish(String(data=json.dumps(dict(frames=[f.as_dict() for f in frames], calibration=report))))
        self.tf = TransformBroadcaster(self)
        self.t_last = 0.0
        if any(f.child == "module_pivot" for f in frames):
            self.create_subscription(McuState, "/mcu/state", self.on_state, 10)

    def on_state(self, m: McuState):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - self.t_last < 0.02:
            return
        self.t_last = t
        c, s = math.cos(m.tilt / 2), math.sin(m.tilt / 2)        # about the pivot's y axis (+ = toward the thrower)
        self.tf.sendTransform(tf_msg("module_pivot", "module", (0.0, 0.0, 0.0), (0.0, s, 0.0, c), m.header.stamp))


class FrameCheck(Node):
    def __init__(self):
        super().__init__("frame_check")
        self.duration = self.declare_parameter("duration", 20.0).value
        self.edges: dict = {}
        self.motion: dict = {"odom->base_link": [], "map->odom": []}
        self.t0 = None
        self.p = self.create_publisher(String, "/frames/check", LATCHED)
        self.create_subscription(TFMessage, "/tf", lambda m: self.on_tf(m, False), 200)
        self.create_subscription(TFMessage, "/tf_static", lambda m: self.on_tf(m, True), LATCHED)
        self.create_timer(1.0, self.tick)

    def on_tf(self, msg: TFMessage, static: bool):
        for t in msg.transforms:
            q = t.transform.rotation
            R = quat_to_R((q.x, q.y, q.z, q.w))
            self.edges[(t.header.frame_id, t.child_frame_id)] = R
            key = f"{t.header.frame_id}->{t.child_frame_id}"
            if key in self.motion:
                ts = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                yaw = math.atan2(R[1, 0], R[0, 0])
                self.motion[key].append((ts, t.transform.translation.x, t.transform.translation.y, yaw))

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.t0 = self.t0 or now
        if now - self.t0 < self.duration:
            return
        problems = check_tree((p, c, R) for (p, c), R in self.edges.items())
        rep = dict(frames=sorted({c for _, c in self.edges} | {p for p, _ in self.edges}), problems=problems,
                   motion={k: check_motion(sorted(v)) for k, v in self.motion.items() if v})
        if rep["motion"].get("odom->base_link", {}).get("jumps"):
            rep["problems"].append("odom -> base_link jumped: local odometry must be continuous (REP-105)")
        self.p.publish(String(data=json.dumps(rep)))
        (self.get_logger().warn if rep["problems"] else self.get_logger().info)(json.dumps(rep))
        self.t0 = now
        for v in self.motion.values():
            v.clear()


def _spin(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    _spin(RobotFrames)


def check_main():
    _spin(FrameCheck)
