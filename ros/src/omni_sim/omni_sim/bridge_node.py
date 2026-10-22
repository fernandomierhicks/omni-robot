"""The MuJoCo sim as a ROS 2 robot (design.md 28). It is the clock: everything else runs with use_sim_time.

Publishes  /clock                  rosgraph_msgs/Clock, every clock_period of sim time
           /scan                   sensor_msgs/LaserScan, one A2M12 turn (1,600 samples, 10 Hz): stamp = the first
                                   sample's time, time_increment = 1 / 16 kHz, so each sample has its own time
           /imu                    sensor_msgs/Imu, BNO085 at 400 Hz: game rotation vector, gyro, accelerometer
           /odom                   nav_msgs/Odometry, wheel odometry (50 Hz est) with covariance
           /tf                     odom -> base_link from the wheel odometry (publish_odom_tf; off when an EKF does it)
           /tf_static              base_link -> laser, base_link -> imu_link (from the model)
           /ground_truth/odom      nav_msgs/Odometry in "world" (the house frame): for scoring only
           /v9/<variant>/pose      v9's own estimators on the same readings (shadow), world frame
           /sim/origin             the odom (= SLAM map) frame's pose in the world, latched
           /sim/phase              tour phase ("tour", "wall", "back", "done"), latched
           /sim/house              MarkerArray: the house's true walls, rugs and thresholds (frame "world"), latched,
                                   with a static TF map (or odom) -> world, so Foxglove / RViz draw the real house under
                                   the SLAM map; /sim/robot: the robot's outline on base_link (where the stack thinks
                                   it is) and its true pose (frame "world")
And, with viewer:=true, MuJoCo's own 3D window of the running sim (WSLg), following the robot, the last scan in red.
Subscribes /cmd_vel                geometry_msgs/Twist (body frame) -> Robot.drive_velocity: the firmware limits apply;
                                   no command for 0.5 s -> a ramped stop

v11 (design.md 30), firmware:=true: the robot runs its C++ firmware (SIL). The bridge then serves the firmware's USB
port on localhost TCP (port, default 5761) instead: mcu_link publishes /odom, /imu and odom -> base_link from the
MCU, and /cmd_vel reaches the firmware through the mode manager. The bridge keeps /clock, /scan, the truth and the
markers.
"""

from __future__ import annotations

import array
import copy
import json
import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from omni_sim import sim_dir

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp(t: float) -> Time:
    ns = int(round(t * 1e9))                  # round once, then split (rounding the fraction can carry a second)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def yaw_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)        # x y z w


class Bridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")
        sim_dir()
        from bridge_core import CoreConfig, SimCore
        P = self.declare_parameter
        mode = P("mode", "cmd_vel").value
        shadow = P("shadow", mode == "tour").value
        flat = P("lidar_flatness_deg", -1.0).value
        kw = {} if flat < 0 else dict(lidar_flatness=math.radians(flat))
        fw = P("fw_div", 2).value                           # firmware every fw_div physics steps (2 = 1 kHz)
        hk = P("hook_div", 4).value                         # floor + electrical hooks (4 = 500 Hz, est: they vary slowly)
        sil = P("firmware", False).value and mode != "tour"  # v11: the C++ firmware in the loop
        self.cfg = CoreConfig(base=P("base", "v7").value, assist=P("assist", mode != "tour").value,
                              seed=P("seed", 0).value, mode=mode, tour_rounds=P("tour_rounds", 1).value,
                              shadow=("wheels+IMU", "wheels+IMU+lidar") if shadow else (), sensor_kw=kw,
                              fw_div=fw, floor_div=hk, post_div=hk, lidar_cast_dt=P("lidar_cast_dt", 0.002).value,
                              **(dict(firmware=True) if sil else {}))
        self.want_viewer = P("viewer", False).value
        self.viewer_mode = P("viewer_mode", "windows").value       # "windows" (native GL) or "wsl"
        self.want_markers = P("markers", True).value
        self.rtf = P("real_time_factor", 1.0).value         # 0 = as fast as it goes
        self.clock_period = P("clock_period", 0.004).value   # 250 Hz: plenty for 100 Hz EKF / 20 Hz Nav2; cheaper
        self.publish_odom_tf = P("publish_odom_tf", True).value
        self.exit_when_done = P("exit_when_done", False).value
        self.get_logger().info(f"building the v7 house ({self.cfg})")
        self.core = SimCore(self.cfg)
        sc = self.sc = self.core.sn.cfg
        self.n_scan = int(round(sc.lidar_rate / sc.lidar_scan_hz))
        self.get_logger().info(f"sensors: {sc.imu_part} at {sc.imu_rate:.0f} Hz, {sc.lidar_part} {sc.lidar_rate:.0f} "
                               f"samples/s at {sc.lidar_scan_hz:.0f} Hz, scan plane tilt "
                               f"{math.degrees(math.atan(self.core.sn.flat_tilt[0])):.2f} deg")
        self.p_clock = self.create_publisher(Clock, "/clock", 10)
        self.p_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.p_imu = self.create_publisher(Imu, "/imu", 50)
        self.p_odom = self.create_publisher(Odometry, "/odom", 20)
        self.p_truth = self.create_publisher(Odometry, "/ground_truth/odom", 20)
        self.p_origin = self.create_publisher(PoseStamped, "/sim/origin", LATCHED)
        self.p_phase = self.create_publisher(String, "/sim/phase", LATCHED)
        self.p_status = self.create_publisher(String, "/sim/status", 10)
        self.p_shadow = {k: self.create_publisher(PoseStamped, f"/v9/{k.lower().replace('+', '_')}/pose", 20)
                         for k in self.cfg.shadow}
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.port = None
        self.no_ekf = self.publish_odom_tf                  # odom -> base_link comes from the odometry, not an EKF
        if getattr(self.core, "firmware", False):
            from omni_sim.mcu_port import McuPort
            self.port = McuPort(P("port", 5761).value, log=self.get_logger().info)
            self.publish_odom_tf = False                    # mcu_link publishes the MCU's odometry
            self.get_logger().info(f"firmware in the loop: its USB port is tcp:127.0.0.1:{self.port.port}")
        else:
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self._static_frames()
        o = self.core.world_from_odom()
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = stamp(self.core.d.time)
        msg.pose.position.x, msg.pose.position.y = float(o[0]), float(o[1])
        q = yaw_quat(float(o[2]))
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.p_origin.publish(msg)
        if self.cfg.mode == "tour":
            self.p_phase.publish(String(data="tour"))
        # the lidar's scan-plane tilt as a one-time bench calibration would measure it: (tilt rad, azimuth rad, height
        # of the scan plane above the floor m), with a calibration error (est 0.2 deg, 10 deg in azimuth)
        tz, psi = self.core.sn.flat_tilt
        rng = np.random.default_rng(20_000 + self.cfg.seed)
        cal = Vector3Stamped()
        cal.header.frame_id = "laser"
        cal.vector.x = float(max(0.0, math.atan(tz) + rng.normal(0, math.radians(0.2))))
        cal.vector.y = float(psi + rng.normal(0, math.radians(10)))
        cal.vector.z = float(self.core.d.site_xpos[self.core.m.site("lidar").id][2])
        self.p_cal = self.create_publisher(Vector3Stamped, "/lidar/tilt_cal", LATCHED)
        self.p_cal.publish(cal)
        self.world_parent = "odom" if self.no_ekf else "map"
        self.p_house = self.create_publisher(MarkerArray, "/sim/house", LATCHED)
        self.p_robot = self.create_publisher(MarkerArray, "/sim/robot", 10)
        if self.want_markers:
            self._house_markers()
        self.viewer = None
        self.last_scan = None
        if self.want_viewer:
            self._open_viewer()

    # -- the MuJoCo viewer and the Foxglove / RViz markers ------------------------------------------------
    def _open_viewer(self):
        """MuJoCo's 3D view (omni_sim.viewer_proc): drawn on Windows (default), or in its own WSL process."""
        c = self.core
        self._scan_pts = None
        if self.viewer_mode == "wsl":
            from omni_sim.viewer_proc import ViewerLink
            self.viewer = ViewerLink(str(sim_dir()), self.cfg.base, self.cfg.wheel_model, c.m.nq, c.base_id)
            self.get_logger().info("MuJoCo viewer opening in WSL (its own process; slow: see viewer_proc.py)")
        else:
            from omni_sim.viewer_proc import WinViewer
            hello = dict(base=self.cfg.base, wheel_model=self.cfg.wheel_model, nq=int(c.m.nq))
            self.viewer = WinViewer(hello, log=self.get_logger().info)

    def _sync_viewer(self):
        v = self.viewer
        if not v.running():
            v.close()
            self.viewer = None
            self.get_logger().info("MuJoCo viewer closed; the sim keeps running")
            return
        pts = None
        s = self.last_scan
        if s is not None and s is not self._scan_pts:        # the last scan, where its rays really hit (truth)
            self._scan_pts = s
            d, m = self.core.d, self.core.m
            sid = m.site("lidar").id
            p0, R = d.site_xpos[sid], d.site_xmat[sid].reshape(3, 3)
            ok = np.isfinite(s.rng)
            a, r = s.ang[ok], s.rng[ok]
            pts = p0 + (R @ np.vstack([r * np.cos(a), r * np.sin(a), np.zeros(len(r))])).T
        v.push(self.core.d.qpos, pts)

    def _house_markers(self):
        """The static scene (walls, rugs, thresholds, the start mat ...) as boxes / cylinders in frame "world"."""
        import mujoco
        m, d = self.core.m, self.core.d
        static = m.body_weldid[m.geom_bodyid] == 0
        arr = MarkerArray()
        k = 0
        BOX, CYL = int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_CYLINDER)
        for g in range(m.ngeom):
            t = int(m.geom_type[g])                 # (a pybind enum == numpy int is False the other way round)
            if not static[g] or t not in (BOX, CYL):
                continue
            rgba = m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g] >= 0 else m.geom_rgba[g]
            if rgba[3] < 0.05 or m.geom_group[g] > 3:
                continue
            mk = Marker()
            mk.header.frame_id = "world"
            mk.ns, mk.id = "house", k
            k += 1
            mk.type = Marker.CUBE if t == BOX else Marker.CYLINDER
            mk.action = Marker.ADD
            p, q = d.geom_xpos[g], np.zeros(4)
            mujoco.mju_mat2Quat(q, d.geom_xmat[g])
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, p)
            mk.pose.orientation.w, mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z = map(float, q)
            s = m.geom_size[g]
            if t == BOX:
                mk.scale.x, mk.scale.y, mk.scale.z = (2 * float(s[0]), 2 * float(s[1]), 2 * float(s[2]))
            else:
                mk.scale.x = mk.scale.y = 2 * float(s[0])
                mk.scale.z = 2 * float(s[1])
            mk.color.r, mk.color.g, mk.color.b = map(float, rgba[:3])
            mk.color.a = float(min(1.0, max(0.3, rgba[3])))
            mk.frame_locked = True
            arr.markers.append(mk)
        self.p_house.publish(arr)
        # where the house is: the map (or, with no SLAM, odom) frame is the robot's pose at the start
        o = self.core.world_from_odom()
        c, s_ = math.cos(o[2]), math.sin(o[2])
        tr = TransformStamped()
        tr.header.stamp = stamp(self.core.d.time)
        tr.header.frame_id = self.world_parent
        tr.child_frame_id = "world"
        tr.transform.translation.x = float(-(c * o[0] + s_ * o[1]))
        tr.transform.translation.y = float(-(-s_ * o[0] + c * o[1]))
        r = tr.transform.rotation
        r.x, r.y, r.z, r.w = yaw_quat(-float(o[2]))
        self.tf_static.sendTransform(self._static + [tr])
        self.get_logger().info(f"/sim/house: {len(arr.markers)} markers of the true house (frame world)")

    def _robot_markers(self, tr):
        t, pos, quat, _ = tr
        arr = MarkerArray()
        for i, (frame, rgba) in enumerate((("base_link", (0.2, 0.5, 1.0, 0.45)), ("world", (0.1, 0.9, 0.3, 0.35)))):
            mk = Marker()
            mk.header.stamp = stamp(t) if frame == "world" else Time()
            mk.header.frame_id = frame
            mk.ns, mk.id, mk.type, mk.action = ("robot (estimate)", "robot (truth)")[i], 0, Marker.CYLINDER, Marker.ADD
            if frame == "world":
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(pos[0]), float(pos[1]), 0.15
                o = mk.pose.orientation
                o.w, o.x, o.y, o.z = map(float, quat)
            else:
                mk.pose.position.z, mk.pose.orientation.w = 0.15, 1.0
            mk.scale.x = mk.scale.y = 0.468                      # the v7 outline, 0.234 m radius
            mk.scale.z = 0.3
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = rgba
            mk.frame_locked = frame == "base_link"
            arr.markers.append(mk)
            ar = Marker()                                        # heading arrow
            ar.header = mk.header
            ar.ns, ar.id, ar.type, ar.action = mk.ns, 1, Marker.ARROW, Marker.ADD
            ar.pose = copy.deepcopy(mk.pose)
            ar.pose.position.z = 0.32
            ar.scale.x, ar.scale.y, ar.scale.z = 0.35, 0.05, 0.05
            ar.color = copy.deepcopy(mk.color)
            ar.color.a = 0.9
            ar.frame_locked = mk.frame_locked
            arr.markers.append(ar)
        self.p_robot.publish(arr)

    def _static_frames(self):
        out = []
        for child, site in (("laser", "lidar"), ("imu_link", "imu")):
            p, q = self.core.frame(site)
            t = TransformStamped()
            t.header.stamp = stamp(self.core.d.time)
            t.header.frame_id = "base_link"
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
            t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = map(float, q)
            out.append(t)
        self._static = out
        self.tf_static.sendTransform(out)

    def on_cmd(self, msg: Twist):
        self.core.set_cmd(msg.linear.x, msg.linear.y, msg.angular.z)

    # -- publishing ------------------------------------------------------------------------------
    def pub_imu(self, imu):
        t, q, g, a = imu
        m = Imu()
        m.header.stamp = stamp(t)
        m.header.frame_id = "imu_link"
        sc = self.sc
        if q is not None:
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = map(float, q)
            rp = (0.5 * math.pi / 180) ** 2               # roll / pitch (est)
            m.orientation_covariance = [rp, 0.0, 0.0, 0.0, rp, 0.0, 0.0, 0.0, (1.0 * math.pi / 180) ** 2]
        else:
            m.orientation_covariance[0] = -1.0                # REP-145: no orientation
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, g)
        sg = (sc.gyro_nd * math.sqrt(sc.imu_rate / 2)) ** 2 + sc.gyro_bias ** 2
        m.angular_velocity_covariance = [sg, 0.0, 0.0, 0.0, sg, 0.0, 0.0, 0.0, sg]
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, a)
        sa = (sc.accel_nd * math.sqrt(sc.imu_rate / 2)) ** 2 + 0.1 ** 2   # + tilt leaking gravity in (est)
        m.linear_acceleration_covariance = [sa, 0.0, 0.0, 0.0, sa, 0.0, 0.0, 0.0, sa]
        self.p_imu.publish(m)

    def pub_odom(self, od):
        t, pose, tw, pvar, tvar = od
        m = Odometry()
        m.header.stamp = stamp(t)
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y = float(pose[0]), float(pose[1])
        q = yaw_quat(pose[2])
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        big = 1e3
        m.pose.covariance = [0.0] * 36
        for i, v in zip((0, 7, 14, 21, 28, 35), (pvar[0], pvar[1], big, big, big, pvar[2])):
            m.pose.covariance[i] = float(v)
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z = map(float, tw)
        m.twist.covariance = [0.0] * 36
        for i, v in zip((0, 7, 14, 21, 28, 35), (tvar[0], tvar[1], big, big, big, tvar[2])):
            m.twist.covariance[i] = float(v)
        self.p_odom.publish(m)
        if self.publish_odom_tf:
            tr = TransformStamped()
            tr.header = m.header
            tr.child_frame_id = "base_link"
            tr.transform.translation.x, tr.transform.translation.y = float(pose[0]), float(pose[1])
            r = tr.transform.rotation
            r.x, r.y, r.z, r.w = q
            self.tf.sendTransform(tr)

    def pub_scan(self, s):
        m = LaserScan()
        m.header.stamp = stamp(float(s.t[0]))
        m.header.frame_id = "laser"
        inc = 2 * math.pi / self.n_scan
        m.angle_min = 0.0
        m.angle_increment = inc
        m.angle_max = inc * (len(s.rng) - 1)
        m.time_increment = 1.0 / self.sc.lidar_rate
        m.scan_time = 1.0 / self.sc.lidar_scan_hz
        m.range_min, m.range_max = map(float, self.sc.lidar_range)
        r = np.where(np.isfinite(s.rng), s.rng, np.inf).astype(np.float32)     # REP-117: no return = +inf
        a = array.array("f")
        a.frombytes(r.tobytes())
        m.ranges = a
        self.p_scan.publish(m)

    def pub_truth(self, tr, shadow):
        t, pos, quat, tw = tr
        m = Odometry()
        m.header.stamp = stamp(t)
        m.header.frame_id = "world"
        m.child_frame_id = "base_link_truth"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, pos)
        o = m.pose.pose.orientation
        o.w, o.x, o.y, o.z = map(float, quat)
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z = map(float, tw)
        self.p_truth.publish(m)
        for k, pose in shadow.items():
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y = float(pose[0]), float(pose[1])
            q = yaw_quat(float(pose[2]))
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
            self.p_shadow[k].publish(ps)

    # -- the loop ----------------------------------------------------------------------------------
    def run(self):
        core, d = self.core, self.core.d
        ex = SingleThreadedExecutor()
        ex.add_node(self)
        next_clock = d.time
        k = 0
        w0, s0 = time.perf_counter(), d.time
        w_rep, s_rep = w0, s0
        done_t = None
        n_truth, w_view = 0, 0.0
        fw_div = getattr(core.r, "fw_div", 1)
        while rclpy.ok():
            ev = core.step()
            if self.port is not None and k % fw_div == fw_div - 1:     # once per firmware tick: its USB port
                self.port.exchange(core.r.fw)
            if d.time >= next_clock - 1e-9:
                next_clock += self.clock_period
                self.p_clock.publish(Clock(clock=stamp(d.time)))
            if ev.imu is not None:
                self.pub_imu(ev.imu)
            if ev.odom is not None:
                self.pub_odom(ev.odom)
            if ev.scan is not None:
                self.pub_scan(ev.scan)
                self.last_scan = ev.scan
            if ev.truth is not None:
                self.pub_truth(ev.truth, ev.shadow)
                n_truth += 1
                if self.want_markers and n_truth % 5 == 0:
                    self._robot_markers(ev.truth)
            if self.viewer is not None and k % 20 == 0 and time.perf_counter() - w_view > 1 / 30:
                w_view = time.perf_counter()
                self._sync_viewer()
            if ev.phase is not None:
                self.p_phase.publish(String(data=ev.phase))
                self.get_logger().info(f"phase: {ev.phase} at t = {d.time:.1f} s, {core.dist:.1f} m driven")
                if ev.phase == "done":
                    done_t = d.time
            k += 1
            if k % 10 == 0:                                   # commands in every 5 ms of sim time
                ex.spin_once(timeout_sec=0.0)
            if self.rtf > 0 and k % 4 == 0:                   # pace to the wall clock
                ahead = (d.time - s0) / self.rtf - (time.perf_counter() - w0)
                if ahead > 0.002:
                    time.sleep(ahead)
                elif ahead < -0.25:                           # behind: don't try to catch up in a burst
                    w0, s0 = time.perf_counter(), d.time
            now = time.perf_counter()
            if now - w_rep > 5.0:
                rtf = (d.time - s_rep) / (now - w_rep)
                st = dict(t=round(d.time, 2), rtf=round(rtf, 3), cmds=core.n_cmd, dist=round(core.dist, 2),
                          phase=core.phase, pose=[round(float(x), 3) for x in core.r.pose()])
                self.p_status.publish(String(data=json.dumps(st)))
                self.get_logger().info(f"sim t {d.time:7.1f} s, x{rtf:.2f} real time, {core.n_cmd} cmd_vel, "
                                       f"{core.dist:.1f} m driven")
                w_rep, s_rep = now, d.time
            if done_t is not None and self.exit_when_done and d.time - done_t > 3.0:
                self.get_logger().info("tour done: exiting")
                break


def main():
    rclpy.init()
    node = Bridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.viewer is not None:
            node.viewer.close()
        if node.port is not None:
            node.port.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
