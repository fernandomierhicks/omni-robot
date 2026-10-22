"""sensor_msgs/Joy from a joystick on Windows (design.md 28).

WSL2 doesn't see USB devices unless they are attached with usbipd-win (admin rights). This relay avoids that: the
Windows side (ros/joystick_sender.py, the Windows venv's pygame) reads the stick and streams its axes and buttons as
JSON lines over TCP to this node, which publishes /joy for teleop_twist_joy. Axes are sent in ROS joy_node's
convention (left / forward positive). The sender skips 3D mice (3Dconnexion, SpaceMouse, KMJ), which also show up as
joysticks. No packet for joy_timeout s -> an all-zero Joy (the dead-man button reads released).
"""

from __future__ import annotations

import json
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyRelay(Node):
    def __init__(self):
        super().__init__("joy_relay")
        self.port = self.declare_parameter("port", 9870).value
        self.timeout = self.declare_parameter("joy_timeout", 0.3).value
        self.pub = self.create_publisher(Joy, "/joy", 10)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", self.port))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn, self.buf, self.last = None, b"", None
        self.n_axes, self.n_buttons = 4, 12
        self.create_timer(0.01, self.poll)
        self.get_logger().info(f"waiting for ros/joystick_sender.py on Windows (TCP port {self.port})")

    def poll(self):
        if self.conn is None:
            try:
                self.conn, addr = self.srv.accept()
                self.conn.setblocking(False)
                self.get_logger().info(f"joystick sender connected from {addr[0]}")
            except BlockingIOError:
                pass
        if self.conn is not None:
            try:
                data = self.conn.recv(65536)
                if not data:
                    raise ConnectionResetError
                self.buf += data
            except BlockingIOError:
                pass
            except (ConnectionResetError, OSError):
                self.get_logger().warn("joystick sender disconnected")
                self.conn, self.buf = None, b""
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                try:
                    j = json.loads(line)
                except ValueError:
                    continue
                if "name" in j:
                    self.get_logger().info(f"controller: {j['name']}" + (f" (ignored: {', '.join(j['ignored'])})"
                                                                          if j.get("ignored") else ""))
                    continue
                self.n_axes, self.n_buttons = len(j["axes"]), len(j["buttons"])
                self.last = self.get_clock().now()
                self.publish(j["axes"], j["buttons"])
        if self.last is None or (self.get_clock().now() - self.last).nanoseconds * 1e-9 > self.timeout:
            self.publish([0.0] * self.n_axes, [0] * self.n_buttons)

    def publish(self, axes, buttons):
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "joy"
        m.axes = [float(a) for a in axes]
        m.buttons = [int(b) for b in buttons]
        self.pub.publish(m)


def main():
    rclpy.init()
    node = JoyRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
