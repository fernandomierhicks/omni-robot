"""mode_manager: who drives (design.md 14, 15, 30). One writer to the MCU link: /mcu/cmd at 100 Hz.

Subscribes /cmd_vel        geometry_msgs/Twist (stick, Foxglove, Nav2): drives in TELEOP if fresher than 0.5 s
           /catch/cmd      omni_interfaces/McuCommand from the catch behavior: forwarded in CATCH
Service    /set_mode       omni_interfaces/SetMode: IDLE | TELEOP | CATCH, kid profile on / off
Publishes  /mcu/cmd        omni_interfaces/McuCommand
           /robot_mode     std_msgs/String (latched), e.g. "CATCH adult"
Parameter  mode (initial), kid.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from omni_interfaces.msg import McuCommand
from omni_interfaces.srv import SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from omni_catch.modes import ModeMux
from omni_mcu.protocol import Request

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FIELDS = ("req_mode", "kid", "drive_mode", "hold_yaw", "a", "b", "c", "v_end", "a_cap", "speed_cap", "t_brake",
          "coast_s", "a_brake", "tilt", "t_tilt", "tilt_after", "fly_rpm", "fire_seq")


def request_to_msg(r: Request, msg: McuCommand | None = None) -> McuCommand:
    m = msg or McuCommand()
    for f in FIELDS:
        v = getattr(r, f)
        setattr(m, f, type(getattr(m, f))(v))
    return m


def msg_to_request(m: McuCommand) -> Request:
    return Request(**{f: getattr(m, f) for f in FIELDS})


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")
        self.mux = ModeMux(self.declare_parameter("mode", "IDLE").value.upper(),
                           self.declare_parameter("kid", False).value)
        self.twist, self.t_twist = None, -1e9
        self.catch_req, self.t_catch = None, -1e9
        self.p_cmd = self.create_publisher(McuCommand, "/mcu/cmd", 10)
        self.p_mode = self.create_publisher(String, "/robot_mode", LATCHED)
        self.create_subscription(Twist, "/cmd_vel", self.on_twist, 10)
        self.create_subscription(McuCommand, "/catch/cmd", self.on_catch, 10)
        self.create_service(SetMode, "/set_mode", self.on_set_mode)
        self.create_timer(0.01, self.tick)
        self._announce()

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _announce(self):
        self.p_mode.publish(String(data=f"{self.mux.mode} {'kid' if self.mux.kid else 'adult'}"))
        self.get_logger().info(f"mode {self.mux.mode} ({'kid' if self.mux.kid else 'adult'} profile)")

    def on_twist(self, m: Twist):
        self.twist, self.t_twist = (m.linear.x, m.linear.y, m.angular.z), self.now()

    def on_catch(self, m: McuCommand):
        self.catch_req, self.t_catch = msg_to_request(m), self.now()

    def on_set_mode(self, req, resp):
        resp.success, resp.message = self.mux.set_mode(req.mode, req.kid)
        if resp.success:
            self._announce()
        return resp

    def tick(self):
        now = self.now()
        if self.mux.mode == "CATCH":
            if self.catch_req is not None and now - self.t_catch < 0.3:
                r = self.catch_req
                r.kid = self.mux.kid
            else:
                r = self.mux.request()                      # the behavior is silent: a ramped stop
        else:
            r = self.mux.request(twist=self.twist if now - self.t_twist < 0.5 else None)
        m = request_to_msg(r)
        m.header.stamp = self.get_clock().now().to_msg()
        self.p_cmd.publish(m)


def main():
    rclpy.init()
    node = ModeManager()
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
