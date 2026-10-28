"""catch_bridge: the catch game's world as a ROS 2 robot, with its firmware in the loop (design.md 30). PC only.

The sim (simulation/v11 sil: catch_world.CatchWorld) runs the field, the thrower, the physics and the robot with the
C++ firmware (SIL). The robot's software runs as on the Jetson: mcu_link connects to the firmware's USB port (TCP,
McuPort), catch_perception gets the cameras' frames, catch_behavior and mode_manager decide. This node is the clock.

Publishes /clock                     every clock_period of sim time (1 ms: the link's time sync and the catch need it)
          /cam/left/sparse, /cam/right/sparse   omni_interfaces/SparseImage: each frame when the capture pipeline
                                     delivers it (exposure + latency, catch_world), stamped with its exposure
          /tf_static                 base_link -> the robot's frames (robot_export.frames: cameras REP-103, frames
                                     cam_<side>_optical on the images), odom -> world
          /ground_truth/odom         the base's true pose, world frame
          /sim/ball                  visualization_msgs/Marker: the true ball (world frame)
          /sim/catch_result          std_msgs/String, JSON, one per throw (the truth: caught, funnel offset, ...)
          /sim/status                std_msgs/String, JSON, every 5 s wall
Parameters dist, apex_min, apex_max, seed, returns, tosses (0 = forever), floor, rtf, port, viewer, pass_game (two
           throwers taking turns: catch_world.WorldConfig.throwers).
"""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from omni_interfaces.msg import SparseImage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker

from omni_sim import sim_dir
from omni_sim.mcu_port import McuPort

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp(t: float) -> Time:
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class CatchBridge(Node):
    def __init__(self):
        super().__init__("catch_bridge")
        sim_dir()
        from catch_world import CatchWorld, WorldConfig
        from home_floor import GAME_FLOORS
        P = self.declare_parameter
        self.floor = P("floor", "flat").value
        self.dist = P("dist", 6.5).value
        cfg = WorldConfig(dist=self.dist, apex_range=(P("apex_min", 0.75).value, P("apex_max", 2.0).value),
                          seed=P("seed", 9000).value, return_throws=P("returns", True).value,
                          floor=GAME_FLOORS[self.floor], throwers=2 if P("pass_game", False).value else 1)
        self.tosses = P("tosses", 0).value
        self.rtf = P("rtf", 1.0).value
        self.clock_period = P("clock_period", 0.001).value
        self.want_viewer = P("viewer", False).value
        self.get_logger().info(f"building the catch field: thrower {self.dist} m out, {self.floor} floor, "
                               f"seed {cfg.seed}, returns {cfg.return_throws}")
        self.w = CatchWorld(cfg)
        self.r = self.w.robot
        self.port = McuPort(P("port", 5761).value, log=self.get_logger().info)
        from omni_sim.bridge_node import FaultTopic
        self.faults = FaultTopic(self, self.r)              # /sim/fault: the console's fault injection
        self.p_clock = self.create_publisher(Clock, "/clock", 10)
        self.p_cam = [self.create_publisher(SparseImage, f"/cam/{s}/sparse", 20) for s in ("left", "right")]
        self.p_truth = self.create_publisher(Odometry, "/ground_truth/odom", 20)
        self.p_ball = self.create_publisher(Marker, "/sim/ball", 10)
        self.p_result = self.create_publisher(String, "/sim/catch_result", 50)
        self.p_status = self.create_publisher(String, "/sim/status", 10)
        self.p_world = self.create_publisher(String, "/sim/world", LATCHED)
        self.tf_static = StaticTransformBroadcaster(self)
        self._static_frames()
        hand = self.w.odom_from_world(self.w.thrower.hand[:2])
        ac = self.w.odom_from_world((0.0, 0.0))
        self.p_world.publish(String(data=json.dumps(dict(
            thrower_odom=[float(hand[0]), float(hand[1])], arena_center_odom=[float(ac[0]), float(ac[1])],
            arena_half=[self.w.p.area_size[0] / 2, self.w.p.area_size[1] / 2], dist=self.dist, floor=self.floor))))
        self.viewer = None
        if self.want_viewer:
            from omni_sim.viewer_proc import WinViewer
            self.viewer = WinViewer(dict(scene="catch", dist=self.dist, floor=self.floor, nq=int(self.w.m.nq)),
                                    log=self.get_logger().info)

    def _static_frames(self):
        m, d = self.w.m, self.w.d
        out = []
        # design.md 10: the robot's frames as robot_export writes them (REP-103: cam_<side> x along the view,
        # cam_<side>_optical z forward), from this world's own model
        from robot_export import frames
        for f in frames(m, d, self.w.p):
            t = TransformStamped()
            t.header.stamp = stamp(d.time)
            t.header.frame_id, t.child_frame_id = f["parent"], f["child"]
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, f["xyz"])
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = \
                map(float, f["quat"])
            out.append(t)
        x0, y0, a = self.w.pose0                           # odom -> world: the world seen from the robot's start
        c, s = math.cos(a), math.sin(a)
        t = TransformStamped()
        t.header.stamp = stamp(d.time)
        t.header.frame_id = "odom"
        t.child_frame_id = "world"
        t.transform.translation.x = float(-(c * x0 + s * y0))
        t.transform.translation.y = float(-(-s * x0 + c * y0))
        t.transform.rotation.z, t.transform.rotation.w = math.sin(-a / 2), math.cos(-a / 2)
        out.append(t)
        self.tf_static.sendTransform(out)

    def _pub_frames(self, t: float):
        for pair in self.w.pop_frames(t):
            for f in pair:
                msg = SparseImage()
                msg.header.stamp = stamp(f.t_cap)
                msg.header.frame_id = f"cam_{('left', 'right')[f.cam]}_optical"
                msg.width, msg.height = f.width, f.height
                msg.ball = f.idx.astype(np.uint32).tolist()
                msg.clutter = f.clutter.astype(np.uint32).tolist()
                msg.seed = int(f.seed) & 0xFFFFFFFF
                self.p_cam[f.cam].publish(msg)

    def _pub_truth(self, t: float):
        d, m = self.w.d, self.w.m
        base = m.body("base").id
        o = Odometry()
        o.header.stamp = stamp(t)
        o.header.frame_id = "world"
        o.child_frame_id = "base_link_truth"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, d.xpos[base])
        q = d.xquat[base]
        o.pose.pose.orientation.w, o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z = \
            map(float, q)
        self.p_truth.publish(o)
        bp, _ = self.w.ball()
        mk = Marker()
        mk.header.stamp = stamp(t)
        mk.header.frame_id = "world"
        mk.ns, mk.id, mk.type, mk.action = "ball (truth)", 0, Marker.SPHERE, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, bp)
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.074
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.9, 0.3, 0.6
        self.p_ball.publish(mk)

    def run(self):
        from rclpy.executors import SingleThreadedExecutor
        w, d = self.w, self.w.d
        ex = SingleThreadedExecutor()
        ex.add_node(self)
        next_clock = d.time
        k = 0
        w0, s0 = time.perf_counter(), d.time
        w_rep, s_rep = w0, s0
        n_res = 0
        t_truth = 0.0
        w_view = 0.0
        while rclpy.ok():
            w.step()
            t = d.time
            if k % self.r.fw_div == self.r.fw_div - 1:      # once per firmware tick: the USB port
                self.faults.inj.step()
                self.port.exchange(self.r.fw, self.faults.inj.link_down())
            if t >= next_clock - 1e-9:
                next_clock += self.clock_period
                self.p_clock.publish(Clock(clock=stamp(t)))
            self._pub_frames(t)
            if t >= t_truth:
                t_truth = t + 0.02
                self._pub_truth(t)
            # a caught throw is published once its return throw is done (the world keeps filling it in until then)
            while n_res < len(w.results) - (1 if w.phase == "held" else 0):
                r = w.results[n_res]
                n_res += 1
                self.p_result.publish(String(data=json.dumps({k_: (None if isinstance(v, float) and math.isnan(v)
                                                                    else v) for k_, v in r.items()})))
                self.get_logger().info(f"throw {n_res}: {'CAUGHT' if r['caught'] else 'missed'}, funnel offset "
                                       f"{100 * r['pos_error']:.1f} cm ({sum(x['caught'] for x in w.results)}/{n_res})")
            if self.viewer is not None and k % 20 == 0 and time.perf_counter() - w_view > 1 / 30:
                w_view = time.perf_counter()
                self.viewer.push(d.qpos)
            k += 1
            if k % 10 == 0:
                ex.spin_once(timeout_sec=0.0)
            if self.rtf > 0 and k % 4 == 0:
                ahead = (t - s0) / self.rtf - (time.perf_counter() - w0)
                if ahead > 0.002:
                    time.sleep(ahead)
                elif ahead < -0.25:
                    w0, s0 = time.perf_counter(), t
            now = time.perf_counter()
            if now - w_rep > 5.0:
                rtf = (t - s_rep) / (now - w_rep)
                res = w.results
                st = dict(t=round(t, 2), rtf=round(rtf, 3), throws=len(res), caught=sum(x["caught"] for x in res),
                          mcu_mode=self.r.fw.mode, port_rx=self.port.rx_bytes, port_tx=self.port.tx_bytes)
                self.p_status.publish(String(data=json.dumps(st)))
                self.get_logger().info(f"sim t {t:7.1f} s, x{rtf:.2f} real time, {st['caught']}/{st['throws']} caught")
                w_rep, s_rep = now, t
            if self.tosses and n_res >= self.tosses and t - w.results[-1]["t_result"] > 1.0:
                self.get_logger().info(f"{self.tosses} throws done: exiting")
                break


def main():
    rclpy.init()
    node = CatchBridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.viewer is not None:
            node.viewer.close()
        node.port.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
