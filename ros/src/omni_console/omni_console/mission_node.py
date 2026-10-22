"""omni_console: the operator console's robot side. The web page talks only to this node (through rosbridge); it
switches missions, keeps the settings, and boils the robot down to a few small topics that cross Wi-Fi cheaply.

Service    /console/command   omni_interfaces/ConsoleCommand: verb + JSON args (missions.py VERBS)
Publishes  /console/status    std_msgs/String JSON, 10 Hz (wall clock): mission, task, MCU, battery, pose, catch ...
           /console/schema    std_msgs/String JSON (latched, and every 5 s): settings with values and hints, places
           /console/events    std_msgs/String JSON, one per event: mission changes and the stack's WARN / ERROR logs
           /console/scan      std_msgs/String JSON, 5 Hz: the lidar as points in the map (or odom) frame, decimated
           /console/plan      std_msgs/String JSON, 1 Hz: Nav2's path, decimated
           /cmd_vel           geometry_msgs/Twist: the console's stick in MANUAL (or /cmd_vel_smoothed, through Nav2's
                              collision monitor, when obstacle slow-down is on and it runs)
Subscribes /console/cmd_vel   geometry_msgs/Twist from the page, normalized -1..1 (linear.z > 0.5 = boost), 20 Hz while
                              the dead man is held; older than 0.3 s -> zero, then silence
           /console/heartbeat std_msgs/String JSON {"id": ...} from each open page (counts consoles)
           /mcu/state /mcu/link /robot_mode /catch/status /catch/track /catch/target <scan> /plan /rosout
Calls      /set_mode, Nav2's navigate_to_pose, parameters of velocity_smoother and catch_behavior
Parameters map (places file name), places_dir, state_dir (settings), scan_topic.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as PathMsg
from omni_interfaces.msg import BallTrack, CatchStatus, McuState
from omni_interfaces.srv import ConsoleCommand, SetMode
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from omni_console import status as st
from omni_console.missions import Console, load_json, save_json

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SCAN_POINTS = 360                  # at most, per scan sent to the page
PLAN_POINTS = 150
NAV_OUTCOME = {4: "succeeded", 5: "canceled", 6: "aborted"}    # action_msgs/GoalStatus


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Feed(Node):
    """The high-rate inputs (/mcu/state 200 Hz, /tf ~100 Hz) on a small node of their own, spun in its own thread:
    rclpy's cost per wake-up grows with a node's entities, and the mission node has ~40 (clients, timers, topics)."""

    def __init__(self, on_mcu):
        super().__init__("omni_console_feed")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(McuState, "/mcu/state", on_mcu, 5)


class MissionNode(Node):
    def __init__(self):
        super().__init__("omni_console")
        P = self.declare_parameter
        state_dir = Path(os.path.expanduser(P("state_dir", "~/.omni").value))
        self.settings_path = state_dir / "console.json"
        places_dir = Path(os.path.expanduser(P("places_dir", str(state_dir)).value))
        self.places_path = places_dir / f"{P('map', '').value or 'default'}.places.json"
        scan_topic = P("scan_topic", "/scan_filtered").value
        self.core = Console(load_json(self.settings_path).get("settings"), load_json(self.places_path))
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)

        self.mcu = self.mcu_t = None
        self.link = None
        self.catch = None
        self.ball = self.target = None
        self.consoles = {}                 # page id -> wall time last heard
        self.nav_handle = None
        self.t_scan = self.t_plan = 0.0
        self.recent_logs = {}
        self.wifi_t, self.wifi = -1e9, None

        self.p_status = self.create_publisher(String, "/console/status", 10)
        self.p_schema = self.create_publisher(String, "/console/schema", LATCHED)
        self.p_events = self.create_publisher(String, "/console/events", 50)
        self.p_scan = self.create_publisher(String, "/console/scan", 5)
        self.p_plan = self.create_publisher(String, "/console/plan", 5)
        self.p_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.p_cmd_safe = self.create_publisher(Twist, "/cmd_vel_smoothed", 10)

        self.feed = Feed(self.on_mcu)
        self.create_subscription(String, "/mcu/link", self.on_link, 5)
        self.create_subscription(String, "/robot_mode", self.on_robot_mode, LATCHED)
        self.create_subscription(CatchStatus, "/catch/status", self.on_catch, 10)
        self.create_subscription(BallTrack, "/catch/track", self.on_track, 10)
        self.create_subscription(Marker, "/catch/target", self.on_target, 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PathMsg, "/plan", self.on_plan, 5)
        self.create_subscription(Log, "/rosout", self.on_log, 50)
        self.create_subscription(Twist, "/console/cmd_vel", self.on_console_twist, 10)
        self.create_subscription(String, "/console/heartbeat", self.on_heartbeat, 10)

        self.create_service(ConsoleCommand, "/console/command", self.on_command)
        self.set_mode_cli = self.create_client(SetMode, "/set_mode")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.params = {n: AsyncParameterClient(self, n) for n in ("velocity_smoother", "catch_behavior")}
        self.tf = self.feed.tf

        # wall-clock timers: the page sees the robot even while the sim's clock is paused
        self.create_timer(0.1, self.publish_status, clock=self.wall)
        self.create_timer(0.1, self.manual_tick, clock=self.wall)     # the stick's timeout; input is sent at once
        self.create_timer(1.0, self.check_caps, clock=self.wall)
        self.create_timer(5.0, self.publish_schema, clock=self.wall)
        self.publish_schema()
        self.get_logger().info(f"console ready: settings {self.settings_path}, places {self.places_path}")

    # -- inputs -------------------------------------------------------------------------------------------------
    def on_mcu(self, m: McuState):
        # fixed-size arrays arrive as numpy: plain Python types, or json can't write them
        self.mcu = dict(mode=m.mode, profile=m.profile, faults=list(m.faults), v_bus=float(m.v_bus),
                        dm_err=[int(e) for e in m.dm_err], fly_rpm=[float(r) for r in m.fly_rpm], tilt=float(m.tilt),
                        ball_held=bool(m.ball_held), drive_mode=int(m.drive_mode))
        self.mcu_t = time.monotonic()

    def on_link(self, m: String):
        try:
            self.link = json.loads(m.data)
        except ValueError:
            pass

    def on_robot_mode(self, m: String):
        self.run_effects(self.core.on_robot_mode(m.data))
        self.publish_schema()

    def on_catch(self, m: CatchStatus):
        try:
            last = json.loads(m.last) if m.last else {}
        except ValueError:
            last = {}
        self.catch = dict(state=m.state, tosses=int(m.tosses), catches=int(m.catches), last=last)

    def on_track(self, m: BallTrack):
        self.ball = [round(float(v), 2) for v in m.x[:3]] if m.tracking else None

    def on_target(self, m: Marker):
        p = m.pose
        self.target = [round(p.position.x, 2), round(p.position.y, 2), round(yaw_of(p.orientation), 2)]

    def on_console_twist(self, m: Twist):
        self.core.manual_input(m.linear.x, m.linear.y, m.angular.z, m.linear.z > 0.5, time.monotonic())
        self.manual_tick()

    def on_heartbeat(self, m: String):
        try:
            cid = str(json.loads(m.data).get("id", "?"))
        except (ValueError, AttributeError):
            cid = "?"
        self.consoles[cid] = time.monotonic()

    def on_log(self, m: Log):
        """WARN and above from the stack, once per 10 s per message, into the console's event list."""
        if m.level < Log.WARN or m.name == self.get_name():
            return
        now = time.monotonic()
        key = (m.name, m.msg[:80])
        if now - self.recent_logs.get(key, -1e9) < 10.0:
            return
        self.recent_logs[key] = now
        if len(self.recent_logs) > 500:
            self.recent_logs = {k: t for k, t in self.recent_logs.items() if now - t < 10.0}
        self.emit("error" if m.level >= Log.ERROR else "warn", m.msg, source=m.name)

    # -- the fixed frame ---------------------------------------------------------------------------------------
    def frame_pose(self, child: str):
        """(frame, x, y, yaw) of child in map, else odom; None while TF has neither."""
        for fixed in ("map", "odom"):
            try:
                t = self.tf.lookup_transform(fixed, child, Time())
            except Exception:                         # tf2 raises several types; none is fatal here
                continue
            tr = t.transform
            return fixed, tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
        return None

    def on_scan(self, m: LaserScan):
        now = time.monotonic()
        if now - self.t_scan < 0.2 or self.p_scan.get_subscription_count() == 0:
            return
        self.t_scan = now
        fp = self.frame_pose(m.header.frame_id)
        if fp is None:
            return
        frame, x0, y0, th = fp
        c, s = math.cos(th), math.sin(th)
        n = len(m.ranges)
        step = max(1, math.ceil(n / SCAN_POINTS))
        pts = []
        for i in range(0, n, step):
            r = m.ranges[i]
            if not (m.range_min < r < m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment
            px, py = r * math.cos(a), r * math.sin(a)
            pts.append(round(x0 + c * px - s * py, 2))
            pts.append(round(y0 + s * px + c * py, 2))
        self.p_scan.publish(String(data=json.dumps(dict(frame=frame, xy=pts))))

    def on_plan(self, m: PathMsg):
        now = time.monotonic()
        if now - self.t_plan < 1.0 or self.p_plan.get_subscription_count() == 0:
            return
        self.t_plan = now
        poses = m.poses
        step = max(1, math.ceil(len(poses) / PLAN_POINTS))
        xy = []
        for p in poses[::step] + poses[-1:]:
            xy += [round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
        self.p_plan.publish(String(data=json.dumps(dict(frame=m.header.frame_id, xy=xy))))

    # -- commands and their effects ----------------------------------------------------------------------------
    def on_command(self, req, resp):
        try:
            args = json.loads(req.args) if req.args.strip() else {}
        except ValueError:
            args = None
        if not isinstance(args, dict):
            resp.ok, resp.message, resp.data = False, "args must be a JSON object", "{}"
            return resp
        r = self.core.command(req.verb, args)
        self.run_effects(r.effects)
        self.save()
        if req.verb in ("set", "place_save", "place_delete", "mission", "stop", "goto", "cancel"):
            self.publish_schema()
        self.flush_events()
        resp.ok, resp.message, resp.data = r.ok, r.message, json.dumps(r.data if r.data is not None else {})
        return resp

    def run_effects(self, effects):
        for fx in effects:
            kind = fx[0]
            if kind == "set_mode":
                self.call_set_mode(fx[1], fx[2])
            elif kind == "nav_goal":
                self.send_goal(*fx[1:])
            elif kind == "nav_cancel":
                self.cancel_goal()
            elif kind == "nav_limits":
                v, w = fx[1], fx[2]
                self.set_params("velocity_smoother", [
                    Parameter("max_velocity", Parameter.Type.DOUBLE_ARRAY, [v, v, w]),
                    Parameter("min_velocity", Parameter.Type.DOUBLE_ARRAY, [-v, -v, -w])])
            elif kind == "param":
                self.set_params(fx[1], [Parameter(fx[2], value=fx[3])])
        self.flush_events()

    def call_set_mode(self, mode: str, kid: bool):
        if not self.set_mode_cli.service_is_ready():
            self.core.on_set_mode_result(False, "the mode manager isn't running")
            return
        fut = self.set_mode_cli.call_async(SetMode.Request(mode=mode, kid=bool(kid)))

        def done(f):
            r = f.result()
            self.core.on_set_mode_result(r is not None and r.success, r.message if r else "no reply")
            self.flush_events()
        fut.add_done_callback(done)

    def set_params(self, node: str, params):
        cli = self.params[node]
        if not cli.services_are_ready():
            if node != "catch_behavior" or self.core.caps["catch"]:
                self.emit("warn", f"{node} isn't running: setting not applied there")
            return
        cli.set_parameters(params)

    def send_goal(self, x: float, y: float, yaw: float, seq: int):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"              # stamp 0: Nav2 takes the latest transform
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        fut = self.nav.send_goal_async(
            g, feedback_callback=lambda fb: self.core.on_nav_feedback(seq, fb.feedback.distance_remaining))

        def accepted(f):
            h = f.result()
            if h is None or not h.accepted:
                self.core.on_nav_done(seq, "rejected")
                self.flush_events()
                return
            self.nav_handle = h

            def result(rf):
                res = rf.result()
                self.core.on_nav_done(seq, NAV_OUTCOME.get(res.status if res else -1, "aborted"))
                self.flush_events()
            h.get_result_async().add_done_callback(result)
        fut.add_done_callback(accepted)

    def cancel_goal(self):
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()
            self.nav_handle = None

    def check_caps(self):
        names = set(self.get_node_names())
        self.core.set_caps(nav=self.nav.server_is_ready(), catch="catch_behavior" in names,
                           collision="collision_monitor" in names)
        self.flush_events()

    # -- outputs -----------------------------------------------------------------------------------------------
    def manual_tick(self):
        tw = self.core.manual_output(time.monotonic())
        if tw is None:
            return
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = tw
        safe = self.core.values["manual.obstacle_stop"] and self.core.caps["collision"]
        (self.p_cmd_safe if safe else self.p_cmd).publish(m)

    def emit(self, level: str, text: str, source: str = "console"):
        self.p_events.publish(String(data=json.dumps(dict(t=time.time(), level=level, text=text, source=source))))

    def flush_events(self):
        for level, text in self.core.pop_events():
            self.emit(level, text)

    def publish_schema(self):
        self.p_schema.publish(String(data=json.dumps(self.core.schema())))

    def save(self):
        if self.core.settings_changed:
            self.core.settings_changed = False
            save_json(self.settings_path, dict(settings=self.core.values))
        if self.core.places_changed:
            self.core.places_changed = False
            save_json(self.places_path, self.core.places)

    def read_wifi(self):
        now = time.monotonic()
        if now - self.wifi_t > 2.0:
            self.wifi_t = now
            try:
                self.wifi = st.parse_wireless(Path("/proc/net/wireless").read_text())
            except OSError:
                self.wifi = None
        return self.wifi

    def publish_status(self):
        now = time.monotonic()
        self.consoles = {k: t for k, t in self.consoles.items() if now - t < 3.0}
        s = self.core.summary()
        s["t"] = time.time()
        s["mcu"] = None if self.mcu is None else st.mcu_summary(self.mcu, now - self.mcu_t)
        s["link"] = self.link
        fp = self.frame_pose("base_link")
        s["pose"] = None if fp is None else dict(frame=fp[0], x=round(fp[1], 3), y=round(fp[2], 3),
                                                 yaw=round(fp[3], 3))
        s["catch"] = None if self.catch is None else st.catch_summary(dict(self.catch, ball=self.ball,
                                                                           target=self.target))
        s["wifi"] = self.read_wifi()
        s["consoles"] = len(self.consoles)
        tw = self.core.manual_output(now) if self.core.manual_active(now) else None
        s["manual"] = dict(active=tw is not None, cmd=None if tw is None else [round(v, 2) for v in tw])
        self.p_status.publish(String(data=json.dumps(s)))
        self.save()
        self.flush_events()


def main():
    rclpy.init()
    node = MissionNode()
    feed = SingleThreadedExecutor()
    feed.add_node(node.feed)
    threading.Thread(target=feed.spin, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        feed.shutdown()
        node.feed.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
