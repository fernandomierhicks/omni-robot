"""ghost_ball: a REAL thrown ball, tracked by the real camera head, drives the SIMULATED robot (design.md 8, phase 1C).

The cameras can't see the simulation, so the ball is real: the Pi's camera head (ros/src/omni_camera + omni_cpp's
catch_perception, `pi.sh ghost`) tracks your throw and publishes its BallTrack on /ghost/track (the head's /catch/track,
remapped, on the same ROS domain as the sim, unicast discovery). This node, in the sim's graph, turns that into the
/catch/track the robot's catch_behavior reads, with the sim's own perception off (stack.py ghost:=true): the sim robot
drives under the real ball with the sim's drive, tilt and funnel; real perception, real network, simulated actuation.

  frame   The head's track is in its robot frame as if the camera were on the robot at pose (0, 0, 0): the real thrower
          stands in front of the camera as the sim's thrower stands ahead of the robot at its home. So with the camera
          looking where the sim robot faces the transform is the identity; for another placement set cam_x / cam_y /
          cam_yaw (the head's frame in the sim's odom frame).
  time    The head stamps a frame with the Pi's system time at its capture; the sim runs on its own clock. Time is mapped
          by the shortest delay seen: sim_stamp = head_stamp + (min over messages of [sim time at receipt - head stamp]) -
          latency_min, latency_min being the capture -> publish -> here delay of the quickest message (the head's 15 ms
          sensor -> published and the network, est: `latency_min`). Valid at rtf 1 (the sim and the head's wall clock
          run at the same rate); clock_mode "wall" (stack.py clock:=wall, both computers' clocks synced) is the identity.
  log     Each track episode (a throw) is logged on /ghost/episode (JSON): when it started, was ready, ended, its last
          crossing prediction of the rim plane and where the robot's funnel was then; offset < the rim radius = the
          sim robot would have caught it. /ghost/ball is the ball for the 3D views.
"""
from __future__ import annotations

import json
import math

import rclpy
from omni_interfaces.msg import BallTrack, McuState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from omni_sim.ghost_core import ClockMap, transform_track


class GhostBall(Node):
    def __init__(self):
        super().__init__("ghost_ball")
        P = self.declare_parameter
        self.cam_xy = (P("cam_x", 0.0).value, P("cam_y", 0.0).value)
        self.cam_yaw = math.radians(P("cam_yaw_deg", 0.0).value)
        self.wall = P("clock_mode", "sim").value == "wall"
        self.rim_radius = P("rim_radius", 0.165).value
        self.map = ClockMap(P("latency_min", 0.017).value)
        be = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(BallTrack, "/catch/track", 20)
        self.p_ep = self.create_publisher(String, "/ghost/episode", 10)
        self.p_ball = self.create_publisher(Marker, "/ghost/ball", 5)
        self.create_subscription(BallTrack, "/ghost/track", self.on_track, be)
        self.create_subscription(McuState, "/mcu/state_10hz", self.on_state, 5)
        from omni_catch.robot_config import RobotConfig
        self.cfg = RobotConfig()
        self.robot = None
        self.ep = None
        self.n_episodes = 0
        self.last_rx = 0.0
        self.create_timer(0.5, self.watchdog)
        self.get_logger().info(f"ghost ball: /ghost/track -> /catch/track, camera frame at {self.cam_xy} yaw "
                               f"{math.degrees(self.cam_yaw):.0f} deg, time {'wall' if self.wall else 'mapped by the shortest delay'}")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, m: McuState):
        self.robot = (m.x, m.y, m.yaw, m.tilt)

    def on_track(self, m: BallTrack):
        now = self.now()
        self.last_rx = now
        head_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t = max(head_t if self.wall else self.map.update(head_t, now), 0.0)     # the first message can land before sim time 0.017
        out = BallTrack()
        out.header.frame_id = "odom"
        out.header.stamp.sec, out.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9))
        out.tracking, out.ready, out.frames = m.tracking, m.ready, m.frames
        if m.tracking:
            xs, Ps, cr = transform_track(m.x, m.covariance, m.crossing, self.cam_xy, self.cam_yaw)
            out.x, out.covariance = [float(v) for v in xs], [float(v) for v in Ps.reshape(-1)]
            out.crossing, out.t_go, out.crossing_sigma = [float(v) for v in cr], m.t_go, m.crossing_sigma
            self.marker(xs)
            self.episode(now, m, cr, t)
        else:
            self.end_episode(now)
        self.pub.publish(out)

    def marker(self, xs):
        mk = Marker()
        mk.header.frame_id = "odom"
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns, mk.id, mk.type, mk.action = "ghost ball", 0, Marker.SPHERE, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(xs[0]), float(xs[1]), float(xs[2])
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.074
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.9, 0.3, 0.9, 0.9
        self.p_ball.publish(mk)

    def episode(self, now, m, cr, t):
        if self.ep is None:
            self.ep = dict(t_start=now, t_ready=None, frames=0, last=None)
        e = self.ep
        e["frames"] = int(m.frames)
        if m.ready and e["t_ready"] is None:
            e["t_ready"] = now
        if m.ready and m.t_go > 0:
            e["last"] = dict(t=now, crossing=[float(v) for v in cr], t_go=float(m.t_go), sigma=float(m.crossing_sigma),
                             robot=None if self.robot is None else list(self.robot))

    def end_episode(self, now):
        e, self.ep = self.ep, None
        if e is None:
            return
        self.n_episodes += 1
        out = dict(n=self.n_episodes, duration=now - e["t_start"], frames=e["frames"],
                   ready_after=None if e["t_ready"] is None else e["t_ready"] - e["t_start"])
        last = e["last"]
        if last is not None and last["robot"] is not None:
            x, y, yaw, tilt = last["robot"]
            rim = float(self.cfg.rim_center(tilt)[0])               # the funnel's rim centre, ahead of the base
            x, y = x + rim * math.cos(yaw), y + rim * math.sin(yaw)
            out.update(crossing=last["crossing"], t_go_at_last=last["t_go"], sigma=last["sigma"],
                       offset_from_funnel=float(math.hypot(last["crossing"][0] - x, last["crossing"][1] - y)))
            out["would_catch"] = bool(out["offset_from_funnel"] < self.rim_radius)
        self.p_ep.publish(String(data=json.dumps(out)))
        self.get_logger().info(f"episode {out['n']}: {out.get('frames')} frames, ready after {out.get('ready_after')}, "
                               f"crossing {out.get('offset_from_funnel')} m from the funnel at the last prediction, "
                               f"{'would catch' if out.get('would_catch') else 'no catch'}")

    def watchdog(self):
        now = self.now()
        if self.ep is not None and now - self.last_rx > 0.6:         # the head went quiet: the track is over
            self.end_episode(now)
            self.pub.publish(BallTrack(header=self._hdr()))

    def _hdr(self):
        from std_msgs.msg import Header
        h = Header()
        h.frame_id = "odom"
        h.stamp = self.get_clock().now().to_msg()
        return h


def main():
    rclpy.init()
    node = GhostBall()
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
