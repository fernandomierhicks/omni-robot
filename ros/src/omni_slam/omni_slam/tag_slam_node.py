"""tag_slam: AprilTags as landmarks next to the lidar's localization, and the localization's health (design.md 10).

Subscribes /tags/detections         omni_interfaces/TagDetections (tag_detector on the robot, the bridge in the sim)
           odom_topic               nav_msgs/Odometry: the smooth local odometry (the EKF's /odometry/filtered, or
                                    mcu_link's /odom without one)
           /imu                     sensor_msgs/Imu: roll and pitch at each exposure (every 8th sample used)
           TF map -> base_link      the lidar localizer's pose (slam_toolbox owns map -> odom), 10 Hz, as absolute
                                    factors with lidar_sigma (slam_toolbox gives no covariance)
Publishes  /tag_slam/pose           geometry_msgs/PoseWithCovarianceStamped (map): the graph's estimate now
           /tag_slam/tags           visualization_msgs/MarkerArray, latched-ish (1 Hz): the tag map
           /tf                      map -> tag_<id> (1 Hz) and map -> dock (dock_tag_id, with the calibrated tag ->
                                    contacts transform); map -> odom only with publish_map_odom (no lidar localizer)
           /localization/health     std_msgs/String JSON, 2 Hz (omni_slam.health): level, speed scale, reasons;
                                    the console slows its own driving with it
           /speed_limit             nav2_msgs/SpeedLimit (percentage) when nav2_speed_limit (off by default: the
                                    console's speed filter publishes there too)
           /initialpose             geometry_msgs/PoseWithCovarianceStamped: a relocalization from two agreeing tag
                                    views when the health asks for one (slam_toolbox's localization takes it: VERIFY)
Parameters map (house), maps_dir, odom_topic, lidar (true: anchor on the lidar's map), lidar_sigma_m / _deg,
           refine_camera_deg (0.5; 0 = off), publish_map_odom (false), dock_tag_id (-1), tag_size (0.16)
The tag map is <maps_dir>/<map>.tags.json (omni-tags/1), loaded at start and saved every minute and at exit.
"""

from __future__ import annotations

import json
import math
import os
import threading
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from omni_interfaces.msg import TagDetections
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from omni_slam.health import HealthMonitor
from omni_slam.slam import TagSlam
from omni_slam.tags import Pinhole, TagDetection


def t_of(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_of(t: float):
    from builtin_interfaces.msg import Time as T
    ns = int(round(t * 1e9))
    return T(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tf_matrix(tr) -> np.ndarray:
    q, p = tr.transform.rotation, tr.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = (p.x, p.y, p.z)
    return M


class TagSlamNode(Node):
    def __init__(self):
        super().__init__("tag_slam")
        P = self.declare_parameter
        self.map = P("map", "house").value
        self.maps_dir = Path(P("maps_dir", os.environ.get("OMNI_MAPS_DIR", str(Path.home() / "omni_ws" / "maps"))).value)
        self.odom_topic = P("odom_topic", "/odometry/filtered").value
        self.use_lidar = P("lidar", True).value
        self.lidar_sig = (P("lidar_sigma_m", 0.03).value, math.radians(P("lidar_sigma_deg", 1.0).value))
        refine = P("refine_camera_deg", 0.5).value
        self.own_map_odom = P("publish_map_odom", False).value
        self.dock_id = P("dock_tag_id", -1).value
        # the dock tag -> its charging contacts (omni_calib.extrinsics.dock_tag): x y z qx qy qz qw
        self.dock_tc = list(P("dock_tag_to_contacts", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]).value)
        self.tag_size = P("tag_size", 0.16).value
        self.want_speed_limit = P("nav2_speed_limit", False).value
        fovy = P("fovy_deg", 70.0).value
        self.cam = Pinhole.from_fovy(fovy, P("width", 1280).value, P("height", 800).value)
        self.cam_frame = P("camera_frame", "cam_left_optical").value
        self.refine = refine if refine > 0 else None
        self.tf_buf = Buffer()
        self.tfl = TransformListener(self.tf_buf, self)
        self.tfb = TransformBroadcaster(self)
        self.slam = None
        self.health = HealthMonitor()
        self.lock = threading.Lock()
        self.n_imu = 0
        self.t_last = None
        self.pending = []
        self.p_pose = self.create_publisher(PoseWithCovarianceStamped, "/tag_slam/pose", 10)
        self.p_tags = self.create_publisher(MarkerArray, "/tag_slam/tags", 2)
        self.p_health = self.create_publisher(String, "/localization/health", 5)
        self.p_init = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 2)
        self.p_speed = None
        if self.want_speed_limit:
            from nav2_msgs.msg import SpeedLimit
            self.SpeedLimit = SpeedLimit
            self.p_speed = self.create_publisher(SpeedLimit, "/speed_limit", 2)
        self.create_subscription(TagDetections, "/tags/detections", self.on_tags, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(Imu, "/imu", self.on_imu, 50)
        self.create_timer(0.1, self.on_lidar_tick)
        self.create_timer(0.5, self.on_health_tick)
        self.create_timer(1.0, self.publish_tags)
        self.create_timer(60.0, self.save)
        self.get_logger().info(f"tag SLAM on {self.odom_topic}, lidar fixes {'on' if self.use_lidar else 'off'}, "
                               f"tag map {self.map_file()}")

    def map_file(self) -> Path:
        return self.maps_dir / f"{self.map}.tags.json"

    def ensure(self) -> bool:
        """The graph needs the camera's mount: base_link -> camera (robot_frames / the bridge's static TF)."""
        if self.slam is not None:
            return True
        try:
            Tbc = tf_matrix(self.tf_buf.lookup_transform("base_link", self.cam_frame, Time()))
        except Exception:
            return False
        self.slam = TagSlam(self.cam, Tbc, anchor="lidar" if self.use_lidar else "first", refine_cam_deg=self.refine)
        if self.map_file().exists():
            self.slam.load_map(self.map_file())
            self.get_logger().info(f"{len(self.slam.g.lm_index)} tags from {self.map_file()}")
        return True

    # -- inputs ------------------------------------------------------------------------------------------------------
    def on_odom(self, m: Odometry):
        if not self.ensure():
            return
        t = t_of(m.header.stamp)
        self.t_last = t
        with self.lock:
            self.slam.on_odom(t, (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)))

    def on_imu(self, m: Imu):
        self.n_imu += 1
        if self.slam is None or self.n_imu % 8 or m.orientation_covariance[0] < 0:
            return
        q = m.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        with self.lock:
            self.slam.on_attitude(t_of(m.header.stamp), roll, pitch)

    def on_lidar_tick(self):
        if not self.use_lidar or self.slam is None or self.t_last is None:
            return
        # the lidar's pose at the odometry's own time, not "the latest": that one lags by the localizer's delay, and
        # a lagging fix reads as a heading bias the camera refinement would soak up (seen in the sim: 0.7 deg)
        try:
            tr = self.tf_buf.lookup_transform("map", "base_link", Time(seconds=self.t_last))
        except Exception:
            return
        t = t_of(tr.header.stamp)
        R = tf_matrix(tr)
        s, a = self.lidar_sig
        with self.lock:
            self.slam.on_lidar(t, (R[0, 3], R[1, 3], math.atan2(R[1, 0], R[0, 0])), np.diag([s * s, s * s, a * a]))
        self.health.on_fix(t)

    def on_tags(self, m: TagDetections):
        if not self.ensure():
            return
        t = t_of(m.header.stamp)
        dets = [TagDetection(d.id, np.array(d.corners).reshape(4, 2), t, d.size or self.tag_size, d.margin)
                for d in m.detections]
        with self.lock:
            out = self.slam.on_tags(t, dets)
            if any(o in ("used", "new tag") for o in out):
                self.health.on_fix(t)
            est, C = self.slam.pose(t), None
            try:
                C = self.slam.pose_cov()
            except Exception:
                pass
            for d in dets:                                   # tags that disagree with the estimate, and reloc votes
                r = self.slam.pose_from_tag(d)
                if r is not None and est is not None and C is not None:
                    self.health.on_tag_pose(est, C, r[0], r[1])
                h = self.health.last
                if h is not None and h.relocalize is not None:
                    v = self.slam.vote(t, d)
                    if v is not None:
                        self.relocalize(t, v[0], v[1])
        if est is not None:
            self.publish_pose(t, est, C)

    # -- outputs -----------------------------------------------------------------------------------------------------
    def publish_pose(self, t, est, C):
        p = PoseWithCovarianceStamped()
        p.header.stamp, p.header.frame_id = stamp_of(t), "map"
        p.pose.pose.position.x, p.pose.pose.position.y = float(est[0]), float(est[1])
        p.pose.pose.orientation.z, p.pose.pose.orientation.w = math.sin(est[2] / 2), math.cos(est[2] / 2)
        cov = [0.0] * 36
        if C is not None:
            for (i, j), (a, b) in zip(((0, 0), (0, 1), (1, 0), (1, 1), (0, 5), (5, 0), (1, 5), (5, 1), (5, 5)),
                                      ((0, 0), (0, 1), (1, 0), (1, 1), (0, 2), (2, 0), (1, 2), (2, 1), (2, 2))):
                cov[6 * i + j] = float(C[a, b])
        p.pose.covariance = cov
        self.p_pose.publish(p)
        if self.own_map_odom:
            mo = self.slam.map_from_odom()
            tr = TransformStamped()
            tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp_of(t), "map", "odom"
            tr.transform.translation.x, tr.transform.translation.y = float(mo[0]), float(mo[1])
            tr.transform.rotation.z, tr.transform.rotation.w = math.sin(mo[2] / 2), math.cos(mo[2] / 2)
            self.tfb.sendTransform(tr)

    def relocalize(self, t, pose, cov):
        m = PoseWithCovarianceStamped()
        m.header.stamp, m.header.frame_id = stamp_of(t), "map"
        m.pose.pose.position.x, m.pose.pose.position.y = float(pose[0]), float(pose[1])
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        c = [0.0] * 36
        c[0], c[7], c[35] = float(cov[0, 0]), float(cov[1, 1]), float(cov[2, 2])
        m.pose.covariance = c
        self.p_init.publish(m)
        self.health.relocalized(t)
        self.get_logger().warn(f"relocalized from the tags: ({pose[0]:.2f}, {pose[1]:.2f}, "
                               f"{math.degrees(pose[2]):.0f} deg)")

    def on_health_tick(self):
        if self.slam is None or self.t_last is None:
            return
        with self.lock:
            try:
                C = self.slam.pose_cov()
            except Exception:
                C = None
        h = self.health.assess(self.t_last, C)
        d = h.as_dict()
        d["tags"] = len(self.slam.g.lm_index)
        d["camera"] = self.slam.g.cam_correction()
        self.p_health.publish(String(data=json.dumps(d)))
        if self.p_speed is not None:
            self.p_speed.publish(self.SpeedLimit(percentage=True, speed_limit=100.0 * h.speed_scale))

    def publish_tags(self):
        if self.slam is None:
            return
        with self.lock:
            tags = self.slam.tags()
        arr, trs = MarkerArray(), []
        now = self.get_clock().now().to_msg()
        for tid, v in tags.items():
            q = _quat(v["R"])
            mk = Marker()
            mk.header.frame_id, mk.header.stamp = "map", now
            mk.ns, mk.id, mk.type, mk.action = "tags", int(tid), Marker.CUBE, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, v["t"])
            mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z, mk.pose.orientation.w = q
            mk.scale.x = mk.scale.y = float(v["size"])
            mk.scale.z = 0.005
            mk.color.r = mk.color.g = mk.color.b = 0.1
            mk.color.a = 0.9
            arr.markers.append(mk)
            tr = TransformStamped()
            tr.header.stamp, tr.header.frame_id = now, "map"
            tr.child_frame_id = "dock" if tid == self.dock_id else f"tag_{tid}"
            tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = map(float, v["t"])
            tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = q
            trs.append(tr)
            if tid == self.dock_id:                       # where the robot's contacts must go (docking's target)
                tc = TransformStamped()
                tc.header.stamp, tc.header.frame_id, tc.child_frame_id = now, "dock", "dock_contacts"
                tc.transform.translation.x, tc.transform.translation.y, tc.transform.translation.z = \
                    map(float, self.dock_tc[:3])
                r = tc.transform.rotation
                r.x, r.y, r.z, r.w = map(float, self.dock_tc[3:7])
                trs.append(tc)
        self.p_tags.publish(arr)
        if trs:
            self.tfb.sendTransform(trs)

    def save(self):
        if self.slam is None or not self.slam.g.lm_index:
            return
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        with self.lock:
            self.slam.save_map(self.map_file())


def _quat(R) -> tuple:
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i], q[j], q[k] = 0.25 * s, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return tuple(q)


def main():
    rclpy.init()
    node = TagSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
