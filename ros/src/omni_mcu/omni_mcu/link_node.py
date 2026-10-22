"""mcu_link: the Jetson's end of the MCU link (design.md 5, 30). Runs on the robot (USB serial to the Teensy) and
against the sim (TCP to the bridge, which feeds the bytes to the SIL firmware); nothing else changes between them.

Publishes /mcu/state   omni_interfaces/McuState, every STATE (200 Hz), stamped on this clock (time sync)
          /odom        nav_msgs/Odometry (50 Hz): the MCU's wheel + BNO085 odometry, covariance from speed and slip
          /tf          odom -> base_link (publish_tf; off when an EKF publishes it)
          /imu         sensor_msgs/Imu: every BNO085 report the MCU forwards (400 Hz)
          /mcu/link    std_msgs/String, JSON link statistics, 1 Hz
Subscribes /mcu/cmd    omni_interfaces/McuCommand (the mode manager); sent at 100 Hz as the heartbeat. A request older
                       than 0.3 s is replaced by a ramped stop (the mode manager has died); the MCU's own watchdog
                       stops the robot if this node dies.
Parameters port: "tcp:127.0.0.1:5761" (sim) or "serial:/dev/ttyACM0"; publish_tf; rate (Hz).
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from omni_interfaces.msg import McuCommand, McuState
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from omni_mcu.protocol import (DRIVE_IDLE, FEED_STATES, REQ_AUTO, HostLink, Imu as ImuMsg, Request, State,
                               to_command)

DEG = math.pi / 180


def stamp_of(t: float):
    from builtin_interfaces.msg import Time
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class Transport:
    def __init__(self, spec: str, log):
        self.spec, self.log = spec, log
        self.sock = self.ser = None

    def open(self) -> bool:
        kind, _, rest = self.spec.partition(":")
        try:
            if kind == "tcp":
                host, port = rest.rsplit(":", 1)
                s = socket.create_connection((host, int(port)), timeout=1.0)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(0.05)
                self.sock = s
            elif kind == "serial":
                import serial                      # pyserial, on the robot
                self.ser = serial.Serial(rest, 2_000_000, timeout=0.05)
            else:
                raise ValueError(self.spec)
            return True
        except (OSError, ValueError) as e:
            self.log(f"MCU link {self.spec}: {e}")
            return False

    def read(self) -> bytes:
        try:
            if self.sock is not None:
                d = self.sock.recv(65536)
                if not d:
                    raise ConnectionError("closed")
                return d
            return self.ser.read(self.ser.in_waiting or 1)
        except socket.timeout:
            return b""

    def write(self, data: bytes):
        if self.sock is not None:
            self.sock.sendall(data)
        else:
            self.ser.write(data)

    def close(self):
        for h in (self.sock, self.ser):
            try:
                if h is not None:
                    h.close()
            except OSError:
                pass
        self.sock = self.ser = None


class McuLink(Node):
    def __init__(self):
        super().__init__("mcu_link")
        P = self.declare_parameter
        self.port = P("port", "tcp:127.0.0.1:5761").value
        self.publish_tf = P("publish_tf", True).value
        self.rate = P("rate", 100.0).value
        self.link = HostLink()
        self.lock = threading.Lock()
        self.tr = Transport(self.port, self.get_logger().warn)
        self.req = Request(req_mode=0, drive_mode=DRIVE_IDLE)
        self.t_req = -math.inf
        self.p_state = self.create_publisher(McuState, "/mcu/state", 50)
        self.p_odom = self.create_publisher(Odometry, "/odom", 20)
        self.p_imu = self.create_publisher(Imu, "/imu", 50)
        self.p_link = self.create_publisher(String, "/mcu/link", 5)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(McuCommand, "/mcu/cmd", self.on_cmd, 10)
        self.create_timer(1.0 / self.rate, self.send)
        self.create_timer(1.0, self.report)
        self.n_state = 0
        self.odom_var = [0.0, 0.0, 0.0]
        self.last_state = None
        self.last_mode = None
        self.connected = False
        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- commands out --------------------------------------------------------------------------------------------
    def on_cmd(self, m: McuCommand):
        self.req = Request(req_mode=m.req_mode, kid=m.kid, drive_mode=m.drive_mode, hold_yaw=m.hold_yaw, a=m.a, b=m.b,
                           c=m.c, v_end=m.v_end, a_cap=m.a_cap, speed_cap=m.speed_cap, t_brake=m.t_brake,
                           coast_s=m.coast_s, a_brake=m.a_brake, tilt=m.tilt, t_tilt=m.t_tilt, tilt_after=m.tilt_after,
                           fly_rpm=m.fly_rpm, fire_seq=m.fire_seq)
        self.t_req = self.now_s()

    def send(self):
        if not self.connected:
            return
        now = self.now_s()
        req = self.req
        if now - self.t_req > 0.3:                        # nobody is asking: a ramped stop, still armed
            req = Request(req_mode=REQ_AUTO if self.req.req_mode == REQ_AUTO else 0, kid=self.req.kid,
                          drive_mode=DRIVE_IDLE, fire_seq=self.req.fire_seq)
        with self.lock:
            data = self.link.encode(to_command(req, self.link.sync), self.get_clock().now().nanoseconds)
        try:
            self.tr.write(data)
        except OSError as e:
            self.get_logger().warn(f"MCU link write: {e}")
            self.connected = False

    # -- messages in ------------------------------------------------------------------------------------------------
    def read_loop(self):
        while self.running and rclpy.ok():
            if not self.connected:
                if self.tr.open():
                    self.connected = True
                    self.get_logger().info(f"MCU link up on {self.port}")
                else:
                    time.sleep(0.5)
                continue
            try:
                data = self.tr.read()
            except (OSError, ConnectionError) as e:
                self.get_logger().warn(f"MCU link lost: {e}")
                self.tr.close()
                self.connected = False
                continue
            if not data:
                continue
            with self.lock:
                msgs = self.link.feed(data, self.get_clock().now().nanoseconds)
            for m in msgs:
                if isinstance(m, State):
                    self.on_state(m)
                elif isinstance(m, ImuMsg):
                    self.on_imu(m)

    def on_state(self, s: State):
        if math.isnan(s.t_host):
            return
        self.n_state += 1
        self.last_state = s
        if s.mode_name != self.last_mode:
            self.get_logger().info(f"MCU mode {s.mode_name} (faults: {', '.join(s.fault_names()) or 'none'})")
            self.last_mode = s.mode_name
        m = McuState()
        m.header.stamp = stamp_of(s.t_host)
        m.header.frame_id = "odom"
        m.t_mcu_us = s.t_mcu_us
        m.mode = s.mode_name
        m.profile = "kid" if s.profile else "adult"
        m.drive_mode = s.drive_mode
        m.feed_state = FEED_STATES.get(s.feed_state, str(s.feed_state))
        m.faults = s.fault_names()
        m.dm_err = list(s.dm_err)
        m.x, m.y, m.yaw, m.vx, m.vy, m.wz, m.slip, m.roll, m.pitch = (s.x, s.y, s.yaw, s.vx, s.vy, s.wz, s.slip,
                                                                     s.roll, s.pitch)
        m.v_cmd_x, m.v_cmd_y, m.w_cmd = s.v_cmd_x, s.v_cmd_y, s.w_cmd
        m.fly_rpm = list(s.fly_rpm)
        m.fly_target_rpm = s.fly_target_rpm
        m.fly_ready = bool(s.fly_ready)
        m.tilt, m.tilt_cmd = s.tilt, s.tilt_cmd
        m.ball_held = s.ball_held
        m.feed_exit_beam = bool(s.beams & 2)
        m.fire_seq_done, m.fire_rejected = s.fire_seq_done, s.fire_rejected
        m.v_bus = s.v_bus
        m.sync_rtt_ms = (self.link.sync.rtt_ns or 0) * 1e-6
        self.p_state.publish(m)
        if self.n_state % 4:
            return
        # odometry at 50 Hz, with a covariance that grows with speed and slip (as v10's bridge, estimator.EKF)
        sp = math.hypot(s.vx, s.vy)
        sv = 0.01 + 0.02 * sp + 3.0 * s.slip
        sw = 0.02 + 0.05 * abs(s.wz) + 3.0 * s.slip
        T = 0.02
        self.odom_var = [self.odom_var[0] + (sv * T) ** 2, self.odom_var[1] + (sv * T) ** 2,
                         self.odom_var[2] + (sw * T) ** 2]
        o = Odometry()
        o.header = m.header
        o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = float(s.x), float(s.y)
        q = (0.0, 0.0, math.sin(s.yaw / 2), math.cos(s.yaw / 2))
        r = o.pose.pose.orientation
        r.x, r.y, r.z, r.w = q
        big = 1e3
        o.pose.covariance = [0.0] * 36
        for i, v in zip((0, 7, 14, 21, 28, 35), (*self.odom_var[:2], big, big, big, self.odom_var[2])):
            o.pose.covariance[i] = float(v)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = float(s.vx), float(s.vy), float(s.wz)
        o.twist.covariance = [0.0] * 36
        for i, v in zip((0, 7, 14, 21, 28, 35), (sv * sv, sv * sv, big, big, big, sw * sw)):
            o.twist.covariance[i] = float(v)
        self.p_odom.publish(o)
        if self.publish_tf:
            tr = TransformStamped()
            tr.header = m.header
            tr.child_frame_id = "base_link"
            tr.transform.translation.x, tr.transform.translation.y = float(s.x), float(s.y)
            rr = tr.transform.rotation
            rr.x, rr.y, rr.z, rr.w = q
            self.tf.sendTransform(tr)

    def on_imu(self, i: ImuMsg):
        if math.isnan(i.t_host):
            return
        m = Imu()
        m.header.stamp = stamp_of(i.t_host)
        m.header.frame_id = "imu_link"
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = map(float, i.quat)
        rp = (0.5 * DEG) ** 2                             # roll / pitch (est), yaw drifts (game rotation vector)
        m.orientation_covariance = [rp, 0.0, 0.0, 0.0, rp, 0.0, 0.0, 0.0, (1.0 * DEG) ** 2]
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, i.gyro)
        sg = (0.01 * DEG * math.sqrt(200)) ** 2 + (0.5 / 60 * DEG) ** 2     # BNO085 (est, as sensors.py)
        m.angular_velocity_covariance = [sg, 0.0, 0.0, 0.0, sg, 0.0, 0.0, 0.0, sg]
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, i.accel)
        sa = (150e-6 * 9.81 * math.sqrt(200)) ** 2 + 0.1 ** 2
        m.linear_acceleration_covariance = [sa, 0.0, 0.0, 0.0, sa, 0.0, 0.0, 0.0, sa]
        self.p_imu.publish(m)

    def report(self):
        st, s = self.link.stats, self.last_state
        d = dict(connected=self.connected, port=self.port, states=st.states, imus=st.imus, cmds=st.cmds,
                 errors=st.errors, rtt_ms=round((self.link.sync.rtt_ns or 0) * 1e-6, 3),
                 mode=s.mode_name if s else None, faults=s.fault_names() if s else None,
                 mcu_rx_errors=s.rx_errors if s else None)
        self.p_link.publish(String(data=json.dumps(d)))

    def destroy_node(self):
        self.running = False
        self.tr.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = McuLink()
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
