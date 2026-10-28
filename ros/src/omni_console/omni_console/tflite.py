"""The console's view of TF, ROS-free: just enough to say where the robot is on the map, read from /tf's raw bytes.

tf2's Python listener turns every /tf message (~100 Hz on the robot: the EKF, slam_toolbox, mcu_link) into Python
objects and then into its C++ buffer; on the Jetson that was most of the mission node's CPU. The console needs one
pose ten times a second, so it keeps the latest transform per child frame straight from the serialized message
(CDR, a few struct reads) and composes the chain only when asked. 2D: x, y and yaw (roll and pitch dropped).
"""

from __future__ import annotations

import math
import struct

from omni_console.cdr import CdrReader


def parse_tf(data: bytes) -> list:
    """tf2_msgs/TFMessage bytes -> [(parent, child, t, x, y, z, qx, qy, qz, qw)]."""
    r = CdrReader(data)
    out = []
    for _ in range(r.u32()):
        t = r.stamp()
        parent = r.string().lstrip("/")
        child = r.string().lstrip("/")
        x, y, z = r.f64s(3)
        qx, qy, qz, qw = r.f64s(4)
        out.append((parent, child, t, x, y, z, qx, qy, qz, qw))
    return out


def yaw_of(qx, qy, qz, qw) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class TfLite:
    """Latest transform per child frame (a tree: each child has one parent). lookup() composes child -> fixed."""

    def __init__(self):
        self.tf = {}                      # child -> (parent, t, x, y, yaw)
        self.n = 0

    def add(self, data: bytes, static: bool = False):
        """One /tf (or, static, /tf_static) message's bytes."""
        try:
            items = parse_tf(data)
        except (struct.error, IndexError):
            return
        self.n += 1
        for parent, child, t, x, y, _, qx, qy, qz, qw in items:
            self.tf[child] = (parent, 0.0 if static else t, x, y, yaw_of(qx, qy, qz, qw))

    def lookup(self, fixed: str, child: str):
        """(x, y, yaw, t) of child in fixed, t the oldest stamp on the chain (static ones count as 0), or None."""
        x = y = yaw = 0.0
        t = math.inf
        f = child
        for _ in range(16):
            if f == fixed:
                return x, y, yaw, (0.0 if t == math.inf else t)
            e = self.tf.get(f)
            if e is None:
                return None
            parent, te, px, py, pyaw = e
            c, s = math.cos(pyaw), math.sin(pyaw)
            x, y, yaw = px + c * x - s * y, py + s * x + c * y, pyaw + yaw
            if te > 0:
                t = min(t, te)
            f = parent
        return None

    def frame_pose(self, child: str, fixed=("map", "odom")):
        """(frame, x, y, yaw, t) of child in the first fixed frame that reaches it, or None."""
        for fr in fixed:
            p = self.lookup(fr, child)
            if p is not None:
                return (fr, p[0], p[1], math.atan2(math.sin(p[2]), math.cos(p[2])), p[3])
        return None
