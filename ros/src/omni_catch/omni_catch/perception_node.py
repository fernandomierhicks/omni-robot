"""catch_perception: the catch path's camera frames -> ball track, in one process (design.md 5, 6, 30).

Subscribes /mcu/state                  omni_interfaces/McuState: where the robot was (odometry + BNO085 attitude)
           source "sparse" (the sim):  /cam/left/sparse, /cam/right/sparse   omni_interfaces/SparseImage
           source "image" (cameras, a rosbag, MuJoCo's renderer): /cam/left/image_raw, /cam/right/image_raw
                                       sensor_msgs/Image, rgb8 / bgr8
           A stereo pair is the two frames with the same exposure stamp (within 1 ms); a frame whose partner hasn't
           come 10 ms later is processed alone (the filter ranges it by the ball's size).
Publishes  /catch/track                omni_interfaces/BallTrack, after every frame pair while tracking
           /catch/ball                 visualization_msgs/Marker: the track and its rim crossing (odom frame)
           /catch/perception           std_msgs/String, JSON: frames, detections, processing time, 1 Hz
Parameters source, calib_rot_deg, ball_size_rel; arena_cx, arena_cy, arena_hx, arena_hy (odom frame, as
           catch_behavior): the play area, where acquisition may start a track (design.md 31)
"""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from omni_interfaces.msg import BallTrack, McuState, SparseImage
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from omni_catch.frames import ArrayFrame, SparseFrame
from omni_catch.perception import Perception
from omni_catch.tracker import TrackerConfig


def t_of(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_of(t: float):
    from builtin_interfaces.msg import Time
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("catch_perception")
        P = self.declare_parameter
        self.source = P("source", "sparse").value
        tc = TrackerConfig(calib_rot_deg=P("calib_rot_deg", 0.10).value, ball_size_rel=P("ball_size_rel", 0.015).value)
        self.perc = Perception(tcfg=tc)
        # the play area from the map, as catch_behavior has it: where a throw can start (acquisition, design.md 31)
        self.perc.set_play_area((P("arena_cx", 3.5).value, P("arena_cy", 0.0).value),
                                (P("arena_hx", 6.0).value, P("arena_hy", 3.0).value))
        self.z_rim = float(self.perc.robot.rim_center(0.0)[1])
        self.pending: dict = {}                      # stamp ns -> [frame left, frame right, wall time first seen]
        self.proc_ms: list = []
        self.n_pairs = 0
        self.was_tracking = False
        self.p_track = self.create_publisher(BallTrack, "/catch/track", 20)
        self.p_ball = self.create_publisher(Marker, "/catch/ball", 10)
        self.p_stats = self.create_publisher(String, "/catch/perception", 5)
        self.create_subscription(McuState, "/mcu/state", self.on_state, 50)
        if self.source == "sparse":
            for k, side in enumerate(("left", "right")):
                self.create_subscription(SparseImage, f"/cam/{side}/sparse", lambda m, k=k: self.on_sparse(k, m), 20)
        else:
            for k, side in enumerate(("left", "right")):
                self.create_subscription(Image, f"/cam/{side}/image_raw", lambda m, k=k: self.on_image(k, m), 5)
        self.create_timer(0.005, self.flush)
        self.create_timer(1.0, self.report)
        self.get_logger().info(f"perception on {self.source} frames, {len(self.perc.cams)} cameras")

    def on_state(self, m: McuState):
        self.perc.add_pose(t_of(m.header.stamp), m.x, m.y, m.yaw, m.roll, m.pitch)

    def on_sparse(self, k: int, m: SparseImage):
        fr = SparseFrame(m.width, m.height, np.asarray(m.ball, dtype=np.int64), np.asarray(m.clutter, dtype=np.int64),
                         m.seed)
        self._add(k, m.header.stamp, fr)

    def on_image(self, k: int, m: Image):
        img = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, -1)[..., :3]
        if m.encoding.lower().startswith("bgr"):
            img = img[..., ::-1]
        self._add(k, m.header.stamp, ArrayFrame(np.ascontiguousarray(img)))

    def _add(self, k, stamp, fr):
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        key = next((s for s in self.pending if abs(s - ns) < 1_000_000), ns)
        slot = self.pending.setdefault(key, [None, None, time.perf_counter()])
        slot[k] = fr
        if slot[0] is not None and slot[1] is not None:
            self._process(key)

    def flush(self):
        now = time.perf_counter()
        for key in sorted(self.pending):
            if now - self.pending[key][2] > 0.010:
                self._process(key)

    def _process(self, key):
        fl, fr, _ = self.pending.pop(key)
        w0 = time.perf_counter()
        self.perc.process(key * 1e-9, [fl, fr])
        self.proc_ms.append(1000 * (time.perf_counter() - w0))
        self.n_pairs += 1
        v = self.perc.view()
        if v is None and not self.was_tracking:
            return
        self.was_tracking = v is not None
        m = BallTrack()
        m.header.frame_id = "odom"
        m.header.stamp = stamp_of(key * 1e-9)
        if v is not None:
            m.header.stamp = stamp_of(v.t)
            m.tracking, m.ready, m.frames = True, bool(v.ready), int(v.n)
            m.x = [float(c) for c in v.x]
            m.covariance = [float(c) for c in v.P.reshape(-1)]
            cr = v.crossing(v.t, self.z_rim)
            if cr is not None:
                m.crossing = [float(c) for c in cr[0]]
                m.t_go = float(cr[2])
                m.crossing_sigma = float(math.sqrt(max(np.linalg.eigvalsh(cr[3]))))
            self._marker(v, cr)
        self.p_track.publish(m)

    def _marker(self, v, cr):
        mk = Marker()
        mk.header.frame_id = "odom"
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns, mk.id, mk.type, mk.action = "ball track", 0, Marker.SPHERE, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, v.x[:3])
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.074
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.85, 0.95, 0.1, 0.9
        self.p_ball.publish(mk)
        if cr is not None:
            mk2 = Marker()
            mk2.header = mk.header
            mk2.ns, mk2.id, mk2.type, mk2.action = "rim crossing", 1, Marker.CYLINDER, Marker.ADD
            mk2.pose.position.x, mk2.pose.position.y, mk2.pose.position.z = float(cr[0][0]), float(cr[0][1]), 0.01
            mk2.pose.orientation.w = 1.0
            mk2.scale.x = mk2.scale.y = 0.33
            mk2.scale.z = 0.01
            mk2.color.r, mk2.color.g, mk2.color.b, mk2.color.a = 1.0, 0.5, 0.0, 0.6
            self.p_ball.publish(mk2)

    def report(self):
        p = self.perc
        ms = np.array(self.proc_ms[-500:]) if self.proc_ms else np.zeros(1)
        d = dict(pairs=self.n_pairs, detections=p.detections, stereo=p.stereo, coarse=p.det.n_coarse,
                 tracking=p.tracker.x is not None, proc_ms_median=round(float(np.median(ms)), 2),
                 proc_ms_95=round(float(np.percentile(ms, 95)), 2), pending=len(self.pending))
        self.p_stats.publish(String(data=json.dumps(d)))


def main():
    rclpy.init()
    node = PerceptionNode()
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
