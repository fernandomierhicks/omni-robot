"""Localization score against ground truth during the house tour (design.md 28), comparable with v9 (design.md 27.3).

Every 0.1 s of sim time while the tour runs (as studies/localization.py T samples), the true pose is compared with:
  ros_map    map -> base_link (slam_toolbox localization or mapping + robot_localization), put in the world with the
             map frame's pose there (/sim/origin: the map frame is the odom frame of the session that built it, which
             started where every session starts)
  ros_odom   odom -> base_link (robot_localization alone: wheels + BNO085), the same way
  v9:<k>     v9's own estimators on the same readings (/v9/<k>/pose, world frame)
Each lookup is made at the truth's own stamp, 1 s late, so the transforms covering it have arrived. At the end of the
tour and after the wall push the errors are noted as v9 does; the samples go to studies/results/<out>.json and the
node exits (which ends the launch).
"""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from omni_sim import results_dir

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """SE(2): a o b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


class Scorer(Node):
    def __init__(self):
        super().__init__("loc_scorer")
        self.out = self.declare_parameter("out", "ros_tour").value
        self.label = self.declare_parameter("label", "").value
        self.map_name = self.declare_parameter("map", "house").value
        self.period = self.declare_parameter("period", 0.1).value
        self.frames = dict(ros_map="map", ros_odom="odom")
        self.buf = Buffer(cache_time=Duration(seconds=60.0))
        self.tfl = TransformListener(self.buf, self)
        self.origin = None
        self.phase = "tour"
        self.pending = []                 # truth samples waiting for their transforms: (t, (x, y, yaw), phase)
        self.samples = []                 # (t, phase, truth, {name: est or None})
        self.v9 = {}                      # name -> list of (t, pose)
        self.marks = {}
        self.last_t = -1.0
        self.done_t = None
        self.create_subscription(PoseStamped, "/sim/origin", self.on_origin, LATCHED)
        self.create_subscription(String, "/sim/phase", self.on_phase, LATCHED)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 50)
        for k in ("wheels_imu", "wheels_imu_lidar"):
            self.create_subscription(PoseStamped, f"/v9/{k}/pose", lambda m, k=k: self.on_v9(k, m), 50)
        self.create_timer(0.2, self.process)

    def on_origin(self, m):
        self.origin = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def on_phase(self, m):
        if m.data != self.phase:
            self.get_logger().info(f"phase {m.data}")
            self.marks[m.data] = self.last_t
        self.phase = m.data

    def on_v9(self, k, m):
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.v9.setdefault(k, {})[round(t, 4)] = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def on_truth(self, m):
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        if self.phase == "done":
            if self.done_t is None:
                self.done_t = t
            return                                    # the run is over: take no more samples
        if t - self.last_t < self.period - 1e-6:
            return
        self.last_t = t
        p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        self.pending.append((t, p, self.phase, m.header.stamp))

    def lookup(self, frame, stamp):
        try:
            tr = self.buf.lookup_transform(frame, "base_link", Time.from_msg(stamp))
        except Exception:
            return None
        q = tr.transform.rotation
        return (tr.transform.translation.x, tr.transform.translation.y, yaw_of(q))

    def process(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - getattr(self, "_t_log", -1e9) > 10.0:
            self._t_log = now
            self.get_logger().info(f"t {now:.1f}: origin {'ok' if self.origin else 'MISSING'}, {len(self.pending)} "
                                   f"pending, {len(self.samples)} scored, phase {self.phase}, done {self.done_t}")
        if self.origin is None:
            return
        keep = []
        for t, p, ph, st in self.pending:
            if now - t < 1.0:
                keep.append((t, p, ph, st))
                continue
            est = {}
            for name, fr in self.frames.items():
                e = self.lookup(fr, st)
                est[name] = None if e is None else compose(self.origin, e)
            for k, d in self.v9.items():
                est[f"v9:{k}"] = d.get(round(t, 4))
            self.samples.append((t, ph, p, est))
        self.pending = keep
        if self.done_t is not None and now - self.done_t > 1.5 and not self.pending:
            self.finish()

    def finish(self):
        names = sorted({k for s in self.samples for k in s[3]})
        out = dict(label=self.label, map=self.map_name, origin=self.origin, marks=self.marks, names=names,
                   samples=[dict(t=t, phase=ph, truth=p, est=e) for t, ph, p, e in self.samples])
        stats = {}
        for k in names:
            pe = [math.hypot(e[k][0] - p[0], e[k][1] - p[1]) for t, ph, p, e in self.samples
                  if ph == "tour" and e.get(k) is not None]
            ye = [abs(math.degrees(wrap(e[k][2] - p[2]))) for t, ph, p, e in self.samples
                  if ph == "tour" and e.get(k) is not None]
            if pe:
                stats[k] = dict(n=len(pe), med=float(np.median(pe)), p95=float(np.percentile(pe, 95)),
                                max=float(np.max(pe)), yaw_p95=float(np.percentile(ye, 95)))
                self.get_logger().info(f"{k:22s} {100 * stats[k]['med']:6.1f} / {100 * stats[k]['p95']:6.1f} / "
                                       f"{100 * stats[k]['max']:6.1f} cm, yaw 95% {stats[k]['yaw_p95']:.2f} deg "
                                       f"({len(pe)} samples)")
        out["stats"] = stats
        path = results_dir() / f"{self.out}.json"
        path.write_text(json.dumps(out))
        self.get_logger().info(f"wrote {path}")
        raise SystemExit(0)


def main():
    rclpy.init()
    node = Scorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
