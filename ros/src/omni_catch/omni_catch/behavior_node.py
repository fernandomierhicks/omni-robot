"""catch_behavior: the play-catch behavior (omni_catch.game.CatchBehavior) as a ROS node (design.md 30).

Subscribes /catch/track    omni_interfaces/BallTrack (catch_perception)
           /mcu/state      omni_interfaces/McuState (mcu_link)
           /catch/setup    std_msgs/String JSON, latched: where the thrower stands relative to the robot, when the
                           mode turns CATCH ({ok, seq, dist (m), bearing (rad from the robot's heading), half (the room's half
                           size), returns});
                           the game restarts there, its home the robot's pose then (the house's catch: one sim world
                           for every mode, design.md 7; a person detector would say this on the robot)
Publishes  /catch/cmd      omni_interfaces/McuCommand at 100 Hz (the mode manager forwards it in CATCH)
           /catch/status   omni_interfaces/CatchStatus, on every state change and at 2 Hz
           /catch/target   visualization_msgs/Marker, 10 Hz: where the base is going (odom frame)
           /catch/field    std_msgs/String JSON, latched: the field from /catch/setup in the odom frame (home, thrower,
                           the play area), for catch_perception
Parameters (odom frame, the robot's own start pose is the origin):
           thrower_x, thrower_y      the thrower's hand, a first guess (refined from each throw's release point)
           arena_cx, arena_cy, arena_hx, arena_hy   the play area (from the map)
           return_throws, kid, return_to ("thrower", or "other": the pass game between two people)
                                     (all three can change while the game runs: the operator console sets them)
/catch/status's `last` is the last throw's record: caught, the throw as tracked (release point, speed, apex, its path
at 20 Hz), the robot's pose; the console's throw radar and logbook use it.
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from omni_interfaces.msg import BallTrack, CatchStatus, McuCommand, McuState
from dataclasses import replace

from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from omni_catch.game import CatchBehavior, GameParams, RobotView
from omni_catch.modes import ModeMux
from omni_catch.mode_node import request_to_msg
from omni_catch.robot_config import RobotConfig
from omni_catch.tracker import TrackView


def t_of(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class BehaviorNode(Node):
    def __init__(self):
        super().__init__("catch_behavior")
        P = self.declare_parameter
        gp = GameParams(thrower=(P("thrower_x", 6.8).value, P("thrower_y", 0.0).value),
                        arena_center=(P("arena_cx", 3.5).value, P("arena_cy", 0.0).value),
                        arena_half=(P("arena_hx", 6.0).value, P("arena_hy", 3.0).value),
                        return_throws=P("return_throws", True).value, kid=P("kid", False).value,
                        return_to=P("return_to", "thrower").value)
        self.rc = RobotConfig()
        self.game = CatchBehavior(self.rc, gp, log=self.get_logger().info)
        self.flight = self.game.flight
        self.accel_sigma = 0.5
        self.mux = ModeMux("CATCH", gp.kid)
        self.track = None
        self.rv = None
        self.last_state = None
        self.p_cmd = self.create_publisher(McuCommand, "/catch/cmd", 10)
        self.p_status = self.create_publisher(CatchStatus, "/catch/status", 10)
        self.p_target = self.create_publisher(Marker, "/catch/target", 10)
        self.create_subscription(BallTrack, "/catch/track", self.on_track, 20)
        self.create_subscription(McuState, "/mcu/state", self.on_state, 50)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.p_field = self.create_publisher(String, "/catch/field", latched)
        self.create_subscription(String, "/catch/setup", self.on_setup, latched)
        self.setup_seq, self.pending_setup = None, None
        self.n_tick = 0
        self.create_timer(0.01, self.tick)
        self.create_timer(0.5, self.status)
        self.add_on_set_parameters_callback(self.on_params)

    def on_params(self, params):
        for p in params:
            if p.name == "return_throws":
                self.game.p.return_throws = bool(p.value)
            elif p.name == "kid":
                self.game.p.kid = self.mux.kid = bool(p.value)
            elif p.name == "return_to":
                if p.value not in ("thrower", "other"):
                    return SetParametersResult(successful=False, reason="return_to: thrower or other")
                self.game.p.return_to = str(p.value)
        return SetParametersResult(successful=True)

    def on_setup(self, m: String):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        if not s.get("ok") or s.get("seq") == self.setup_seq:
            return
        if self.rv is None:
            self.pending_setup = s                    # applied with the first MCU state
            return
        self._apply_setup(s)

    def _apply_setup(self, s: dict):
        """The field relative to the robot -> this game's odom frame: home where the robot is, the thrower dist m
        ahead, the play area the room around home. The console's settings (kid, returns, return_to) carry over."""
        self.setup_seq, self.pending_setup = s.get("seq"), None
        x, y, yaw = (float(c) for c in self.rv.pose)
        d, half = float(s["dist"]), float(s.get("half", 1.5))
        a = yaw + float(s.get("bearing", 0.0))            # where the thrower stands, from the robot's heading
        thrower = (x + d * math.cos(a), y + d * math.sin(a))
        gp = replace(self.game.p, home=(x, y), thrower=thrower, arena_center=(x, y), arena_half=(half, half),
                     return_throws=bool(s.get("returns", self.game.p.return_throws)))
        self.game = CatchBehavior(self.rc, gp, log=self.get_logger().info)
        self.flight = self.game.flight
        self.track = None
        mid = ((x + thrower[0]) / 2, (y + thrower[1]) / 2)
        self.p_field.publish(String(data=json.dumps(dict(
            seq=self.setup_seq, home=[x, y], thrower=list(thrower), arena_center=[x, y], arena_half=[half, half],
            play_center=list(mid), play_half=[d / 2 + 1.5, d / 2 + 1.5]))))
        self.get_logger().info(f"catch field: home ({x:.2f}, {y:.2f}), thrower {d:.1f} m away at "
                               f"{math.degrees(a - yaw):.0f} deg, room {half:.1f} m")

    def on_track(self, m: BallTrack):
        if not m.tracking:
            self.track = None
            return
        self.track = TrackView(self.flight, self.accel_sigma, t_of(m.header.stamp), np.array(m.x),
                               np.array(m.covariance).reshape(6, 6), m.frames, m.ready)

    def on_state(self, m: McuState):
        self.rv = RobotView(t=t_of(m.header.stamp), pose=np.array([m.x, m.y, m.yaw]),
                            vel_body=np.array([m.vx, m.vy, m.wz]), v_cmd=np.array([m.v_cmd_x, m.v_cmd_y]),
                            tilt=m.tilt, tilt_cmd=m.tilt_cmd, fly_rpm=tuple(m.fly_rpm),
                            fly_target_rpm=m.fly_target_rpm, fly_ready=m.fly_ready, ball_held=m.ball_held,
                            fire_seq_done=m.fire_seq_done, mode=m.mode)

    def tick(self):
        if self.rv is None:
            return
        if self.pending_setup is not None:
            self._apply_setup(self.pending_setup)
        t = self.get_clock().now().nanoseconds * 1e-9
        track = self.track
        if track is not None and t - track.t > 0.6:          # perception went quiet
            track = self.track = None
        cc = self.game.step(t, self.rv, track)
        m = request_to_msg(self.mux.request(catch=cc))
        m.header.stamp = self.get_clock().now().to_msg()
        self.p_cmd.publish(m)
        if self.game.state != self.last_state:
            self.last_state = self.game.state
            self.status()
        self.n_tick += 1
        if self.n_tick % 10:                       # the target marker at 10 Hz (at 100 Hz it cost the console's
            return                                 # mission node 7% of a core once Catch ran in the robot step)
        tgt = self.game.target
        mk = Marker()
        mk.header.frame_id = "odom"
        mk.header.stamp = m.header.stamp
        mk.ns, mk.id, mk.type, mk.action = "catch target", 0, Marker.ARROW, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(tgt[0]), float(tgt[1]), 0.02
        mk.pose.orientation.z, mk.pose.orientation.w = math.sin(tgt[2] / 2), math.cos(tgt[2] / 2)
        mk.scale.x, mk.scale.y, mk.scale.z = 0.3, 0.05, 0.05
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.2, 0.6, 1.0, 0.9
        self.p_target.publish(mk)

    def status(self):
        g = self.game
        s = CatchStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.state, s.tosses, s.catches = g.state, g.tosses, g.catches
        s.last = json.dumps(g.last)
        self.p_status.publish(s)


def main():
    rclpy.init()
    node = BehaviorNode()
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
