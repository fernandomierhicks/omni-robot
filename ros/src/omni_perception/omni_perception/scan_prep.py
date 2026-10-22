"""Scan preparation for slam_toolbox and Nav2 (design.md 28): drop floor returns, then de-skew.  /scan -> /scan_filtered

Floor filter. The A2M12's scan plane is ~13 cm up and tilted by up to its 1.5 deg flatness spec; the sprung base pitches
~1.5 deg (95%) when it accelerates and up to ~7 deg on transitions. A ray tilted down meets the floor at h / slope: at
13 cm and 1.2 deg that is 6.4 m, inside the house. Those returns are phantom walls that move with the robot, and they
pull slam_toolbox's scan matching off (a 1.2 deg unit's map came out 35 cm off). For each ray the node predicts where it
meets the floor from the unit's measured plane tilt (/lidar/tilt_cal: a one-time bench calibration, (tilt, azimuth,
height)) and the BNO085's roll and pitch, and drops returns at or beyond that range less a margin.

De-skew. One A2M12 turn takes 0.1 s; at the tour's 2 m/s and 4 rad/s the robot moves 20 cm and turns 23 deg meanwhile.
slam_toolbox treats a scan as taken at its stamp (the first sample, ROS convention), so a moving robot's scans are both
smeared and late by half a turn. Each sample is moved into the laser frame at the stamp with odom -> base_link (the
EKF) at its own time (stamp + i * time_increment), then re-binned by angle (nearest return kept); time_increment is
set to 0. This is what v9's matcher does with its own pose history (estimator.ScanMatcher).
"""

from __future__ import annotations

import array
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ScanPrep(Node):
    def __init__(self):
        super().__init__("scan_prep")
        self.margin = self.declare_parameter("margin", 0.3).value          # m before the predicted floor hit
        self.floor = self.declare_parameter("floor", True).value
        self.deskew = self.declare_parameter("deskew", True).value
        self.n_key = self.declare_parameter("deskew_keys", 11).value        # odom poses per turn, interpolated
        self.cal = None                                                     # (tan tilt, azimuth, height)
        self.rp = (0.0, 0.0)
        self.n_in = self.n_drop = self.n_scans = self.n_skew_fail = 0
        self.buf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.buf, self)
        self.laser_off = None                                               # laser origin in base_link (x, y)
        self.pub = self.create_publisher(LaserScan, "/scan_filtered", 10)
        self.create_subscription(Vector3Stamped, "/lidar/tilt_cal", self.on_cal, LATCHED)
        self.create_subscription(Imu, "/imu", self.on_imu, 50)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.queue = []
        self.create_timer(0.005, self.drain)
        self.create_timer(10.0, self.report)

    def on_cal(self, m):
        self.cal = (math.tan(m.vector.x), m.vector.y, m.vector.z)

    def on_imu(self, m):
        q = m.orientation
        if m.orientation_covariance[0] < 0:
            return
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.rp = (roll, pitch)

    def floor_mask(self, a, r):
        tz, psi, h = self.cal
        d = np.vstack([np.cos(a), np.sin(a), tz * np.cos(a - psi)])          # laser frame (= base frame axes)
        roll, pitch = self.rp
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        Rrp = np.array([[cp, sp * sr, sp * cr], [0.0, cr, -sr], [-sp, cp * sr, cp * cr]])   # Ry(pitch) Rx(roll)
        w = Rrp @ d
        slope = w[2] / np.hypot(w[0], w[1])
        r_floor = np.where(slope < -1e-4, h / np.maximum(-slope, 1e-4), np.inf)
        return np.isfinite(r) & (r >= r_floor - self.margin)

    def odom_pose(self, t: float):
        tr = self.buf.lookup_transform("odom", "base_link", Time(seconds=t), timeout=Duration(seconds=0.0))
        q = tr.transform.rotation
        return tr.transform.translation.x, tr.transform.translation.y, _yaw(q)

    def deskew_scan(self, m, a, r):
        t0 = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        n = len(r)
        dt = m.time_increment
        if dt <= 0:
            return r
        if self.laser_off is None:
            tr = self.buf.lookup_transform("base_link", m.header.frame_id, Time())
            self.laser_off = (tr.transform.translation.x, tr.transform.translation.y)
        T = n * dt
        tk = t0 + np.linspace(0.0, T, self.n_key)
        try:
            K = np.array([self.odom_pose(float(t)) for t in tk])
        except Exception as e:
            if not self.n_skew_fail:
                self.get_logger().warn(f"de-skew: no odom -> base_link over the scan ({type(e).__name__}: {e})")
            self.n_skew_fail += 1
            return r
        K[:, 2] = np.unwrap(K[:, 2])
        ts = t0 + dt * np.arange(n)
        px, py, pyaw = (np.interp(ts, tk, K[:, j]) for j in range(3))
        ok = np.isfinite(r)
        lx, ly = self.laser_off
        bx = lx + r[ok] * np.cos(a[ok])                     # base frame at the sample's time
        by = ly + r[ok] * np.sin(a[ok])
        c, s = np.cos(pyaw[ok]), np.sin(pyaw[ok])
        wx, wy = px[ok] + c * bx - s * by, py[ok] + s * bx + c * by    # odom frame
        c0, s0 = math.cos(K[0, 2]), math.sin(K[0, 2])
        qx, qy = c0 * (wx - K[0, 0]) + s0 * (wy - K[0, 1]), -s0 * (wx - K[0, 0]) + c0 * (wy - K[0, 1])  # base at t0
        qx, qy = qx - lx, qy - ly                           # laser at t0
        rr = np.hypot(qx, qy)
        aa = np.mod(np.arctan2(qy, qx) - m.angle_min, 2 * math.pi)
        k = np.round(aa / m.angle_increment).astype(int) % n
        out = np.full(n, np.inf)
        np.minimum.at(out, k, rr)
        return out

    def on_scan(self, m: LaserScan):
        if not self.deskew:
            self.prepare(m)
            return
        self.queue.append(m)

    def drain(self):
        """De-skew needs odom -> base_link up to the scan's last sample; the EKF publishes it a little later. Scans
        wait here (never blocking the executor that receives /tf); one still uncovered after 0.3 s goes out as is."""
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.queue:
            m = self.queue[0]
            t_end = Time.from_msg(m.header.stamp).nanoseconds * 1e-9 + len(m.ranges) * m.time_increment
            ready = self.buf.can_transform("odom", "base_link", Time(seconds=t_end))
            if not ready and now - t_end < 0.3:
                return
            self.queue.pop(0)
            self.prepare(m)

    def prepare(self, m: LaserScan):
        r = np.frombuffer(array.array("f", m.ranges).tobytes(), dtype=np.float32).astype(float)
        a = m.angle_min + m.angle_increment * np.arange(len(r))
        self.n_scans += 1
        if self.floor and self.cal is not None:
            drop = self.floor_mask(a, r)
            self.n_in += int(np.isfinite(r).sum())
            self.n_drop += int(drop.sum())
            r[drop] = np.inf
        out = LaserScan()
        out.header, out.angle_min, out.angle_max = m.header, m.angle_min, m.angle_max
        out.angle_increment, out.time_increment, out.scan_time = m.angle_increment, m.time_increment, m.scan_time
        out.range_min, out.range_max = m.range_min, m.range_max
        if self.deskew:
            r = self.deskew_scan(m, a, r)
            out.time_increment = 0.0
        r = np.where((r < m.range_min) | (r > m.range_max), np.inf, r)
        arr = array.array("f")
        arr.frombytes(r.astype(np.float32).tobytes())
        out.ranges = arr
        self.pub.publish(out)

    def report(self):
        msg = f"{self.n_scans} scans"
        if self.n_in:
            msg += f", dropped {100 * self.n_drop / self.n_in:.1f}% of returns as floor"
        if self.floor and self.cal is None:
            msg += " (no /lidar/tilt_cal yet: no floor filter)"
        if self.deskew:
            msg += f", de-skew failed on {self.n_skew_fail}"
        self.get_logger().info(msg)


def main():
    rclpy.init()
    node = ScanPrep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
