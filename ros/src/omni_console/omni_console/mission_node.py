"""omni_console: the operator console's robot side. The web page talks only to this node (through rosbridge); it
switches missions, keeps the settings, runs SLAM in the 'robot' step, and boils the robot down to a few small topics
that cross Wi-Fi cheaply.

The Jetson only sends data; the page does the drawing and the arithmetic for it. Three topics go to the page raw,
straight through rosbridge (cbor-raw: no conversion on the robot, web/js/cdr.js decodes them): the map, the lidar scan
(the page places it with the status's pose and the laser's offset from /console/version) and /mcu/samples (mcu_link,
the Base tab's 200 Hz samples). This node reads its own high-rate inputs as raw bytes too (/tf, /catch/track:
tflite.py, cdr.py) and decodes one message when it publishes, and takes the MCU's state at 10 Hz (/mcu/state_10hz).

Service    /console/command   omni_interfaces/ConsoleCommand: verb + JSON args (missions.py VERBS)
Also: the session logs (session_log.py: <state_dir>/logs/sessions), the throw log and leaderboard (throws.jsonl),
Health's counters (health.json), the kids' programs (programs.json), the calibration results (calibration.json), and
per map its places, routes, no-go zones and edges (<map>.places.json, which it turns into Nav2's filter masks).

Publishes  /console/status    std_msgs/String JSON, 10 Hz (wall clock): mission, task, MCU, battery, pose, SLAM, RC ...
           /console/schema    std_msgs/String JSON (latched, and every 5 s): settings with values and hints, places,
                              routes
           /console/version   std_msgs/String JSON, every 5 s: software, firmware (expected and running), packages, the
                              raw topics' names and the laser's offset on the robot
           /console/events    std_msgs/String JSON, one per event: mission changes and the stack's WARN / ERROR logs
           /console/plan      std_msgs/String JSON, 1 Hz: Nav2's path, decimated
           /console/camera    std_msgs/String JSON, 10 Hz while the Vision tab is open: the sim's sparse frames as blobs
           /console/throw     std_msgs/String JSON (latched): the last throw's record with its path (the throw radar)
           /keepout_filter_mask /speed_filter_mask (nav_msgs/OccupancyGrid) and /keepout_filter_info
           /speed_filter_info (nav2_msgs/CostmapFilterInfo), latched: the no-go zones and edges for Nav2's filters
           /cmd_vel           geometry_msgs/Twist: the console's stick in MANUAL (or /cmd_vel_smoothed, through Nav2's
                              collision monitor, when obstacle slow-down is on and it runs)
           /thrower/cmd       omni_interfaces/McuCommand, 20 Hz in THROWER: tilt, flywheel rpm, fire_seq
           /initialpose       geometry_msgs/PoseWithCovarianceStamped: where the robot is on a loaded map
           /face/say /face/hear std_msgs/String: the Say box's lines (JSON: text, mood, act) and typed voice
                              commands, for omni_face's face_node
Subscribes /console/cmd_vel   geometry_msgs/Twist from the page, normalized -1..1 (linear.z > 0.5 = boost), 20 Hz while
                              the dead man is held; older than 0.3 s -> zero, then silence
           /console/heartbeat std_msgs/String JSON {"id": ...} from each open page (counts consoles)
           /mcu/state_10hz /mcu/info /mcu/link /robot_mode /catch/status /catch/target /catch/perception /plan /map
           /rosout; raw: /tf /tf_static /catch/track; /cam/left/sparse (only while the Vision tab is open)
Calls      /set_mode, Nav2's navigate_to_pose, parameters of velocity_smoother and catch_behavior, slam_toolbox's
           serialize_map (the pose graph; the map image it writes itself), Nav2's lifecycle manager (is it active? and, when its bring-up failed because the
           map frame came late, a reset and a new start once the map is there)
Runs       slam_toolbox (its own launch files) when given slam_mapping_params: mapping or localization on a saved map
Parameters map, places_dir (maps and their places / routes), state_dir (settings), scan_topic, slam_mapping_params,
           slam_localization_params, slam_start (mapping | localization), slam_sim_time, firmware_dir, version_file.
"""

from __future__ import annotations

import array
import json
import math
import os
import platform
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from omni_interfaces.msg import BallTrack, CatchStatus, McuCommand, McuInfo, McuState, SparseImage
from omni_interfaces.srv import ConsoleCommand, SetMode
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

from omni_console import session_log
from omni_console import status as st
from omni_console.cdr import parse_ball_track, parse_laser_scan
from omni_console.explore import Grid
from omni_console.health import Health
from omni_console.missions import NEW_MAP, VERSION, Console, load_json, save_json
from omni_console.people import detect as detect_people
from omni_console.slam import localization_params, saved_maps, write_map_image
from omni_console.tflite import TfLite
from omni_console.throws import ThrowLog, kmh
from omni_console.zones import covers, masks as zone_masks

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
TF_STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
PLAN_POINTS = 150
CLUTTER_POINTS = 150
NAV_OUTCOME = {4: "succeeded", 5: "canceled", 6: "aborted"}    # action_msgs/GoalStatus
IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")
VIDEO_PORT = 8081                  # web_video_server (camera streams as MJPEG, never through rosbridge)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


SESSION_IDLE_END = 30.0            # s in IDLE that end a session log
# warnings that are routine, not news for the operator: Nav2's costmap filters say so (at WARN) each time the console
# sends new masks, and wait for the first one at start-up
QUIET_WARNINGS = ("filter mask arrived", "filter info arrived", "Filter mask was not received")
FILTER_KEEPOUT, FILTER_SPEED_ABS = 0, 2   # nav2_msgs CostmapFilterInfo types


class LoadMeter:
    """What the node's own callbacks cost: time per callback name over 10 s windows, logged (and kept for the Version
    tab) when they add up to more than 5% of a core. design.md 7: "profile the mission node on the Jetson"."""

    def __init__(self, log):
        self.log, self.acc, self.n, self.mx, self.t0, self.last = log, {}, {}, {}, time.monotonic(), {}

    def wrap(self, name, fn):
        def run(*a, **k):
            t = time.perf_counter()
            try:
                return fn(*a, **k)
            finally:
                dt = time.perf_counter() - t
                self.acc[name] = self.acc.get(name, 0.0) + dt
                self.n[name] = self.n.get(name, 0) + 1
                self.mx[name] = max(self.mx.get(name, 0.0), dt)
                now = time.monotonic()
                if now - self.t0 >= 10.0:
                    self.flush(now)
        return run

    def flush(self, now):
        span = now - self.t0
        total = sum(self.acc.values())
        self.last = {k: round(100 * v / span, 2) for k, v in sorted(self.acc.items(), key=lambda kv: -kv[1])}
        if total / span > 0.05:
            top = ", ".join(f"{k} {v}% ({self.n[k]} calls, max {1000 * self.mx[k]:.0f} ms)"
                            for k, v in list(self.last.items())[:5])
            self.log(f"busy: {100 * total / span:.1f}% of a core in 10 s (fast_tick includes drain, status, drive): "
                     f"{top}")
        self.acc, self.n, self.mx, self.t0 = {}, {}, {}, now


def load_map_extras(path) -> dict:
    """<map>.places.json: {"places", "routes", "zones", "edges"}; phase 1 wrote the places alone."""
    d = load_json(path)
    if not any(k in d for k in ("places", "routes", "zones", "edges")):
        d = dict(places=d)
    return {k: d.get(k) or {} for k in ("places", "routes", "zones", "edges")}


class Feed(Node):
    """The busy inputs, on a node that is never spun: /tf (~85 Hz) and /catch/track (camera rate) as raw bytes, the
    MCU's 10 Hz state and INFO wait in their queues and drain() takes them all at once, ten times a second, just before
    the status goes out. Woken by the executor per message, the /tf alone cost a few % of a core (rclpy's overhead per
    callback, not the decoding). use_global_arguments=False: the launch's __node remap would otherwise give it the
    mission node's name too."""

    def __init__(self, on_mcu, on_info, on_track, on_target):
        super().__init__("omni_console_feed", use_global_arguments=False)
        self.tf = TfLite()
        never = lambda m: None                                       # noqa: E731  (rclpy wants a callback)
        self.subs = [
            (self.create_subscription(TFMessage, "/tf", never, 100, raw=True), self.tf.add, False),
            (self.create_subscription(TFMessage, "/tf_static", never, TF_STATIC_QOS, raw=True),
             lambda b: self.tf.add(b, static=True), False),
            (self.create_subscription(McuState, "/mcu/state_10hz", never, 5), on_mcu, True),
            (self.create_subscription(McuInfo, "/mcu/info", never, 5), on_info, True),
            (self.create_subscription(BallTrack, "/catch/track", never, 5, raw=True), on_track, True),
            (self.create_subscription(Marker, "/catch/target", never, 5), on_target, True),       # 10 Hz
        ]
        self.scan_sub = None
        self.never = never

    def set_scan(self, topic: str | None, handler=None):
        """The lidar as raw bytes, only while a mode needs it (Follow): created and dropped as the mode changes."""
        if topic is None and self.scan_sub is not None:
            self.subs = [s for s in self.subs if s[0] is not self.scan_sub]
            self.destroy_subscription(self.scan_sub)
            self.scan_sub = None
        elif topic is not None and self.scan_sub is None:
            self.scan_sub = self.create_subscription(LaserScan, topic, self.never, qos_profile_sensor_data, raw=True)
            self.subs.append((self.scan_sub, handler, True))

    MAX_TAKE = 200                     # per subscription per drain: no queue here is deeper (/tf keeps 100)
    flood_info = ""
    n_repeats = 0                      # drains ended by a message handed back twice (see drain)

    def _describe_flood(self, topic: str, msgs: list) -> str:
        """What a capped drain held: how many distinct messages (the same bytes over and over = rmw handing one message
        out again; all distinct = a publisher in a burst), who publishes the topic, and for /tf the frames and the
        stamps' span."""
        distinct = len({bytes(m) for m in msgs})
        try:
            pubs = sorted({f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in self.get_publishers_info_by_topic(topic)})
        except Exception:
            pubs = []
        extra = ""
        if topic == "/tf":
            from omni_console.tflite import parse_tf
            frames, ts = {}, []
            for m in msgs:
                try:
                    for parent, child, t, *_ in parse_tf(bytes(m)):
                        frames[f"{parent}->{child}"] = frames.get(f"{parent}->{child}", 0) + 1
                        ts.append(t)
                except Exception:
                    pass
            if ts:
                extra = (f"; frames {dict(sorted(frames.items(), key=lambda kv: -kv[1])[:4])}, stamps over "
                         f"{max(ts) - min(ts):.3f} s")
        return f"{distinct} distinct of {len(msgs)}; publishers {pubs}{extra}"

    def drain(self):
        """Every waiting message to its handler (only the newest, for the ones marked latest-only). At most MAX_TAKE
        per subscription. The "/tf flood" (a million messages in one 4 s drain, LoadMeter found it; then the cap's 200
        every few minutes) was never a flood: the drain's own describe showed 200 takes holding 1-3 distinct messages
        while a normal /tf subscriber saw nothing unusual (Oct 25). rmw hands a raw take's message back again; the
        first exact repeat now ends the loop."""
        self.n_taken, self.flood = 0, None
        for sub, handler, latest_only in list(self.subs):
            msgs = []
            seen = set()
            try:
                with sub.handle:
                    while len(msgs) < self.MAX_TAKE:
                        r = sub.handle.take_message(sub.msg_type, sub.raw)
                        if r is None:
                            break
                        if sub.raw:
                            # rmw sometimes hands back messages this loop already took (the same bytes over and over:
                            # once a million of them in 4 s, the "/tf flood"): a real message never repeats exactly
                            # (its stamp moves), so the first repeat ends this subscription's drain
                            b = bytes(r[0])
                            if b in seen:
                                self.n_repeats += 1
                                break
                            seen.add(b)
                        msgs.append(r[0])
            except Exception:                         # shutting down, or a message rmw couldn't take
                pass
            if len(msgs) >= self.MAX_TAKE:
                self.flood = sub.topic_name
                self.flood_info = self._describe_flood(sub.topic_name, msgs)
            self.n_taken += len(msgs)
            for m in msgs[-1:] if latest_only else msgs:
                handler(m)


class SlamSupervisor:
    """slam_toolbox as the console runs it: slam_toolbox's own launch file (it configures and activates the lifecycle
    node) as a child process, mapping or localizing on a saved map, one at a time. Switching stops the old one first
    (SIGINT to its process group, SIGKILL after 8 s) in a thread, so the node never blocks."""

    def __init__(self, mapping_params: str, loc_template: str, maps_dir: Path, sim_time: bool, log_dir: Path, log):
        self.mapping_params, self.loc_template = mapping_params, Path(loc_template).read_text()
        self.maps_dir, self.sim_time, self.log = maps_dir, sim_time, log
        self.log_path = log_dir / "slam_toolbox.log"
        self.proc = None
        self.mode = self.map = None
        self.busy = False
        self.lock = threading.Lock()

    def start(self, mode: str, map_name: str | None, pose, done):
        self.busy = True
        threading.Thread(target=self._switch, args=(mode, map_name, pose, done), daemon=True).start()

    def _switch(self, mode, map_name, pose, done):
        with self.lock:
            self._stop()
            if mode == "mapping":
                params, launch = self.mapping_params, "online_async_launch.py"
            else:
                text = localization_params(self.loc_template, str(self.maps_dir / map_name), pose or (0, 0, 0))
                f = tempfile.NamedTemporaryFile("w", suffix="_console_slam_localization.yaml", delete=False)
                f.write(text)
                f.close()
                params, launch = f.name, "localization_launch.py"
            out = open(self.log_path, "ab")
            self.proc = subprocess.Popen(
                ["ros2", "launch", "slam_toolbox", launch, f"slam_params_file:={params}",
                 f"use_sim_time:={'true' if self.sim_time else 'false'}"],
                stdout=out, stderr=subprocess.STDOUT, start_new_session=True, cwd=str(self.log_path.parent))
            self.mode, self.map = mode, map_name
            self.log(f"slam_toolbox {mode}{' on ' + map_name if map_name else ''} (log {self.log_path})")
        done(mode, map_name)

    def _stop(self):
        p, self.proc = self.proc, None
        if p is None or p.poll() is not None:
            return
        try:
            os.killpg(p.pid, signal.SIGINT)
            p.wait(timeout=8.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def shutdown(self):
        with self.lock:
            self._stop()


class MissionNode(Node):
    def __init__(self):
        super().__init__("omni_console")
        P = self.declare_parameter
        state_dir = Path(os.path.expanduser(P("state_dir", "~/.omni").value))
        state_dir.mkdir(parents=True, exist_ok=True)
        self.settings_path = state_dir / "console.json"
        self.maps_dir = Path(os.path.expanduser(P("places_dir", str(state_dir)).value))
        self.scan_topic = P("scan_topic", "/scan_filtered").value
        map_name = P("map", "").value or None
        mapping_params = P("slam_mapping_params", "").value
        loc_params = P("slam_localization_params", "").value
        slam_start = P("slam_start", "localization").value
        sim_time = P("slam_sim_time", False).value
        self.firmware_dir = P("firmware_dir", os.environ.get("OMNI_FIRMWARE_DIR", "")).value
        self.version_file = P("version_file", os.environ.get("OMNI_VERSION_FILE",
                                                              str(Path.home() / "omni_ws" / "version.json"))).value
        self.log_source = P("log_source", "robot").value             # "sim" when the sim stands in for the robot
        self.state_dir = state_dir
        self.programs_path = state_dir / "programs.json"
        self.calib_path = state_dir / "calibration.json"
        self.health_path = state_dir / "health.json"
        self.sessions_dir = state_dir / "logs" / "sessions"
        self.core = Console(load_json(self.settings_path).get("settings"),
                            programs=load_json(self.programs_path).get("programs"),
                            calib=load_json(self.calib_path).get("results"),
                            throws=ThrowLog(state_dir / "throws.jsonl"), health=Health(load_json(self.health_path)))
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.slam = (SlamSupervisor(mapping_params, loc_params, self.maps_dir, sim_time, state_dir,
                                    self.get_logger().info) if mapping_params and loc_params else None)

        self.mcu = self.mcu_t = None
        self.info = self.info_t = None
        self.link = None
        self.catch = None
        self.track_raw = self.track = None
        self.t_track = -1e9
        self.perception = None
        self.target = None
        self.grid_msg, self.grid = None, None
        self.consoles = {}                 # page id -> wall time last heard
        self.nav_handle = None
        self.t_plan = self.t_cam = 0.0
        self.recent_logs = {}
        self.wifi_t, self.wifi = -1e9, None
        self.sys_t, self.sys = -1e9, None
        self.images, self.video = [], False
        self.ext_slam_mode, self.ext_slam_asked = None, False
        self.nav_active = False
        self.t_nav_retry = time.monotonic()           # first retry no sooner than 30 s after start (bring-up is slow)
        self.n_caps = 0
        self.cam_sub = None
        self.last_toss = None              # the catch code's last throw number seen (a new one is a new record)
        self.fires_seen = None             # the MCU's fire_seq_done last seen (counts balls fired)
        self.session = None                # session_log.SessionLog while a mission runs
        self.t_idle = None
        self.mask_geom = None              # the map geometry the filter masks were made for
        self.scan_laser = None             # the laser on the base (x, y, yaw), for people
        self.grid_np = None                # (the map message, its cells as numpy), for people's furniture test
        self.t_slam_up, self.t_slam_retry = None, -1e9    # SLAM's bring-up watch (check_slam_frame)

        self.p_status = self.create_publisher(String, "/console/status", 10)
        self.p_schema = self.create_publisher(String, "/console/schema", LATCHED)
        self.p_version = self.create_publisher(String, "/console/version", LATCHED)
        self.p_events = self.create_publisher(String, "/console/events", 50)
        self.p_plan = self.create_publisher(String, "/console/plan", 5)
        self.p_camera = self.create_publisher(String, "/console/camera", 5)
        self.p_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.p_cmd_safe = self.create_publisher(Twist, "/cmd_vel_smoothed", 10)
        self.p_thrower = self.create_publisher(McuCommand, "/thrower/cmd", 10)
        self.p_initial = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.p_throw = self.create_publisher(String, "/console/throw", LATCHED)      # the last throw, with its path
        self.p_arm = self.create_publisher(String, "/arm/request", 10)              # omni_arm (design.md 11)
        self.p_face_say = self.create_publisher(String, "/face/say", 10)            # omni_face: the Say box
        self.p_face_hear = self.create_publisher(String, "/face/hear", 10)          # a voice command, typed
        # Nav2's costmap filters (nav2.yaml): the Map edit tab's no-go zones and floor edges
        self.p_keep_mask = self.create_publisher(OccupancyGrid, "/keepout_filter_mask", MAP_QOS)
        self.p_speed_mask = self.create_publisher(OccupancyGrid, "/speed_filter_mask", MAP_QOS)
        self.p_keep_info = self.create_publisher(CostmapFilterInfo, "/keepout_filter_info", MAP_QOS)
        self.p_speed_info = self.create_publisher(CostmapFilterInfo, "/speed_filter_info", MAP_QOS)

        self.feed = Feed(self.on_mcu, self.on_info, self.on_track, self.on_target)
        lm = self.load = LoadMeter(self.get_logger().warn)
        self.create_subscription(String, "/mcu/link", self.on_link, 5)
        self.create_subscription(String, "/localization/health", self.on_loc_health, 5)     # omni_slam, 2 Hz
        self.create_subscription(String, "/robot_mode", self.on_robot_mode, LATCHED)
        self.create_subscription(CatchStatus, "/catch/status", lm.wrap("catch", self.on_catch), 10)
        self.create_subscription(String, "/catch/perception", self.on_perception, 5)
        self.create_subscription(PathMsg, "/plan", lm.wrap("plan", self.on_plan), 5)
        self.create_subscription(OccupancyGrid, "/map", lm.wrap("map", self.on_map), MAP_QOS)
        self.create_subscription(Log, "/rosout", lm.wrap("rosout", self.on_log), 50)
        self.create_subscription(Twist, "/console/cmd_vel", lm.wrap("stick", self.on_console_twist), 10)
        self.create_subscription(String, "/console/heartbeat", self.on_heartbeat, 10)
        self.create_subscription(String, "/arm/state", lm.wrap("arm", self.on_arm_state), 5)

        self.create_service(ConsoleCommand, "/console/command", lm.wrap("command", self.on_command))
        self.set_mode_cli = self.create_client(SetMode, "/set_mode")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.c_nav_active = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
        self.c_nav_manage = self.create_client(ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")
        self.params = {n: AsyncParameterClient(self, n) for n in ("velocity_smoother", "heading_node",
                                                                    "catch_behavior", "slam_toolbox")}
        from slam_toolbox.srv import SerializePoseGraph
        self.SerializePoseGraph = SerializePoseGraph
        self.c_serialize = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
        self.tf = self.feed.tf

        self.use_map(map_name)
        if self.slam is not None:
            have = map_name in saved_maps(self.maps_dir)
            mode = slam_start if (slam_start == "mapping" or have) else "mapping"
            if slam_start == "localization" and not have:
                self.get_logger().warn(f"no saved map '{map_name}': starting a new map")
            self.core.set_slam(managed=True, busy=True, maps=saved_maps(self.maps_dir))
            self.slam.start(mode, map_name if mode == "localization" else None, (0.0, 0.0, 0.0), self.on_slam_started)

        # wall-clock timers: the page sees the robot even while the sim's clock is paused
        self.n_fast = 0
        lm = self.load
        self._drain = lm.wrap("drain", self.feed.drain)              # fast_tick's parts, metered on their own
        self._status = lm.wrap("status", self.publish_status)
        self._drive = lm.wrap("drive", self.drive_tick)
        self.create_timer(0.05, lm.wrap("fast_tick", self.fast_tick), clock=self.wall)   # inputs, driving, status
        self.create_timer(0.5, lm.wrap("mission_tick", self.mission_tick), clock=self.wall)
        self.create_timer(1.0, lm.wrap("check_caps", self.check_caps), clock=self.wall)
        self.create_timer(5.0, lm.wrap("schema", self.publish_schema), clock=self.wall)
        self.create_timer(5.0, lm.wrap("version", self.publish_version), clock=self.wall)
        self.publish_schema()
        self.publish_version()
        self.get_logger().info(f"console ready: settings {self.settings_path}, maps {self.maps_dir}"
                               f"{', running SLAM' if self.slam else ''}")

    # -- maps, places, routes -------------------------------------------------------------------------------------
    def extras_path(self, name):
        return self.maps_dir / f"{name or 'default'}.places.json"

    def use_map(self, name):
        self.core.set_map(name, **load_map_extras(self.extras_path(name)))
        self.mask_geom = None                         # the next /map publishes the new map's masks

    def publish_masks(self):
        """The no-go zones and the edges' slow bands as Nav2's filter masks, on the current map's grid."""
        g = self.grid_msg
        if g is None:
            return
        i = g.info
        keep, speed = zone_masks(self.core.zones, self.core.edges, i.width, i.height, i.resolution,
                                 i.origin.position.x, i.origin.position.y)
        for pub, data, info_pub, kind, mult in ((self.p_keep_mask, keep, self.p_keep_info, FILTER_KEEPOUT, 1.0),
                                                (self.p_speed_mask, speed, self.p_speed_info, FILTER_SPEED_ABS, 0.01)):
            m = OccupancyGrid()
            m.header.frame_id = g.header.frame_id or "map"
            m.header.stamp = g.header.stamp
            m.info = g.info
            m.data = array.array("b", data.tobytes())
            pub.publish(m)
            fi = CostmapFilterInfo()
            fi.header.frame_id = m.header.frame_id
            fi.type, fi.base, fi.multiplier = kind, 0.0, mult
            fi.filter_mask_topic = pub.topic_name
            info_pub.publish(fi)
        self.mask_geom = (i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)
        self.core.zones_changed = False

    def on_slam_started(self, mode, map_name):
        """In the supervisor's thread: the new slam_toolbox is starting (check_caps sees it come up)."""
        self.use_map(map_name if mode == "localization" else NEW_MAP)
        self.grid_msg = self.grid = None
        self.core.set_slam(mode=None, map=map_name, busy=True, maps=saved_maps(self.maps_dir))
        self.publish_schema()

    # -- inputs -------------------------------------------------------------------------------------------------
    def on_mcu(self, m: McuState):
        """The newest /mcu/state_10hz (Feed.drain). Fixed-size arrays arrive as numpy: plain Python types, or json
        can't write them."""
        self.mcu = dict(mode=m.mode, profile=m.profile, faults=list(m.faults), v_bus=float(m.v_bus),
                        dm_err=[int(e) for e in m.dm_err], fly_rpm=[float(r) for r in m.fly_rpm], tilt=float(m.tilt),
                        ball_held=bool(m.ball_held), drive_mode=int(m.drive_mode),
                        fly_target_rpm=float(m.fly_target_rpm), fly_ready=bool(m.fly_ready),
                        tilt_cmd=float(m.tilt_cmd), feed_state=m.feed_state, feed_exit_beam=bool(m.feed_exit_beam),
                        fire_seq_done=int(m.fire_seq_done), fire_rejected=int(m.fire_rejected), vx=float(m.vx),
                        vy=float(m.vy), wz=float(m.wz), slip=float(m.slip), roll=float(m.roll), pitch=float(m.pitch),
                        sync_rtt_ms=float(m.sync_rtt_ms))
        self.mcu_t = time.monotonic()

    def on_info(self, m: McuInfo):
        self.info = dict(protocol=int(m.protocol), build=m.build, rc_ok=bool(m.rc_ok), rc_manual=bool(m.rc_manual),
                         rc=[round(float(c), 3) for c in m.rc], motor_temp=[int(c) for c in m.motor_temp_c],
                         driver_temp=[int(c) for c in m.driver_temp_c])
        self.info_t = time.monotonic()

    def on_link(self, m: String):
        try:
            self.link = json.loads(m.data)
        except ValueError:
            pass

    def on_loc_health(self, m: String):
        try:
            self.core.on_loc_health(json.loads(m.data), time.monotonic())
        except (ValueError, AttributeError):
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
        toss = last.get("toss")
        if toss is not None and toss != self.last_toss and "speed" in last:
            first = self.last_toss is None and m.tosses > 1          # the console started mid-game: not a new throw
            self.last_toss = toss
            if not first:
                self.on_throw(last)

    def on_throw(self, last: dict):
        """A throw the catch code finished with: the player's tally, the session log, the radar, Health's counters."""
        rec = self.core.throws.add(st.clean(last))
        self.core.health.count("throws")
        if rec.get("caught"):
            self.core.health.count("catches")
        self.p_throw.publish(String(data=json.dumps(rec)))
        if self.session is not None:
            self.session.write("throw", frame="odom", **{k: v for k, v in rec.items() if k != "time"})
        sp = kmh(rec.get("speed"))
        apex = rec.get("apex")
        self.emit("info", f"{rec['player']}: throw {rec.get('toss')}, {sp if sp is not None else '?'} km/h"
                          f"{f', {apex:.1f} m high' if apex is not None else ''}: "
                          f"{'caught!' if rec.get('caught') else 'missed'}")

    def on_track(self, raw: bytes):
        """The newest /catch/track, as bytes (Feed.drain); decoded when the status goes out."""
        self.track_raw, self.t_track = raw, time.monotonic()

    def latest_track(self):
        raw = self.track_raw
        if raw is not None:
            self.track_raw = None
            try:
                self.track = parse_ball_track(raw)
            except Exception:                         # a malformed message: keep the last one
                pass
        return self.track

    def on_target(self, m: Marker):
        p = m.pose
        self.target = [round(p.position.x, 2), round(p.position.y, 2), round(yaw_of(p.orientation), 2)]

    def on_perception(self, m: String):
        try:
            self.perception = json.loads(m.data)
        except ValueError:
            pass

    def on_map(self, m: OccupancyGrid):
        self.grid_msg, self.grid = m, None
        i = m.info
        geom = (i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)
        if not covers(self.mask_geom, geom):
            self.publish_masks()                      # the map is new or grew past the masks: they follow it (empty
                                                      # ones too: Nav2's filters wait for a mask). Not on every rebuild:
                                                      # slam_toolbox's bounds shift a little each time, and each new mask
                                                      # made both costmaps redo their filters (800 times in a 2 h soak)

    def on_console_twist(self, m: Twist):
        self.core.manual_input(m.linear.x, m.linear.y, m.angular.z, m.linear.z > 0.5, time.monotonic())
        if self.core.mission == "MANUAL":
            self.drive_tick()

    def on_scan_raw(self, raw: bytes):
        """The newest lidar scan while Follow runs: people in the base frame (people.py), furniture on the map left
        out."""
        try:
            t, frame, a0, inc, rmin, rmax, ranges = parse_laser_scan(raw)
        except Exception:                             # a malformed message
            return
        if self.scan_laser is None:
            lp = self.tf.lookup("base_link", frame or "laser")
            self.scan_laser = (lp[0], lp[1], lp[2]) if lp is not None else (0.0, 0.0, 0.0)
        occ = None
        g, pose = self.grid_msg, self.core.pose
        if g is not None and pose is not None and pose[0] == "map":
            i = g.info
            if self.grid_np is None or self.grid_np[0] is not g:
                self.grid_np = (g, np.frombuffer(bytes(g.data), dtype=np.int8).reshape(i.height, i.width))
            data = self.grid_np[1]
            c, s = math.cos(pose[3]), math.sin(pose[3])

            def occ(xb, yb):
                x, y = pose[1] + c * xb - s * yb, pose[2] + s * xb + c * yb
                col, row = int((x - i.origin.position.x) / i.resolution), int((y - i.origin.position.y) / i.resolution)
                # 5 cm around: a chair leg is on its mapped cell; a person walking by a wall isn't (10 cm lost them)
                win = data[max(0, row - 1):row + 2, max(0, col - 1):col + 2]
                return win.size > 0 and int(win.max()) >= 65
        r = np.where((ranges > rmin) & (ranges < rmax), ranges, np.inf)
        self.core.on_people(detect_people(r, a0, inc, self.scan_laser, occ), time.monotonic())

    def on_heartbeat(self, m: String):
        try:
            cid = str(json.loads(m.data).get("id", "?"))
        except (ValueError, AttributeError):
            cid = "?"
        self.consoles[cid] = time.monotonic()

    def on_log(self, m: Log):
        """WARN and above from the stack, once per 10 s per message, into the console's event list."""
        if m.level < Log.WARN or m.name == self.get_name() or any(n in m.msg for n in QUIET_WARNINGS):
            return
        now = time.monotonic()
        key = (m.name, m.msg[:80])
        if now - self.recent_logs.get(key, -1e9) < 10.0:
            return
        self.recent_logs[key] = now
        if len(self.recent_logs) > 500:
            self.recent_logs = {k: t for k, t in self.recent_logs.items() if now - t < 10.0}
        self.emit("error" if m.level >= Log.ERROR else "warn", m.msg, source=m.name)

    def on_sparse(self, m: SparseImage):
        """Camera rate, only while the Vision tab is open: the left camera's ball and clutter pixels as blobs, 10 Hz."""
        now = time.monotonic()
        if now - self.t_cam < 0.1:
            return
        self.t_cam = now
        w = int(m.width)
        ball = None
        if len(m.ball):
            b = np.asarray(m.ball, dtype=np.int64)
            xs, ys = b % w, b // w
            ball = [round(float(xs.mean()), 1), round(float(ys.mean()), 1),
                    round(float(math.sqrt(len(b) / math.pi)), 1), int(len(b))]
        cl = np.asarray(m.clutter, dtype=np.int64)
        if len(cl) > CLUTTER_POINTS:
            cl = cl[:: math.ceil(len(cl) / CLUTTER_POINTS)]
        clutter = [int(v) for p in zip(cl % w, cl // w) for v in p]
        self.p_camera.publish(String(data=json.dumps(dict(w=w, h=int(m.height), cam=m.header.frame_id, ball=ball,
                                                          clutter=clutter))))

    # -- the fixed frame ---------------------------------------------------------------------------------------
    def frame_pose(self, child: str):
        """(frame, x, y, yaw, t) of child in map, else odom; None while TF has neither."""
        return self.tf.frame_pose(child)

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
        self.core.now = time.monotonic()
        r = self.core.command(req.verb, args)
        self.run_effects(r.effects)
        self.save()
        if req.verb not in ("ping", "schema", "fire", "thrower"):
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
                v, w, facing, spin = fx[1:]
                self.set_params("velocity_smoother", [
                    Parameter("max_velocity", Parameter.Type.DOUBLE_ARRAY, [v, v, w]),
                    Parameter("min_velocity", Parameter.Type.DOUBLE_ARRAY, [-v, -v, -w])])
                self.set_params("heading_node", [Parameter("heading", value=facing),
                                                 Parameter("spin_rate", value=float(spin)),
                                                 Parameter("turn_max", value=float(w))])
            elif kind == "param":
                self.set_params(fx[1], [Parameter(fx[2], value=fx[3])])
            elif kind == "slam_start":
                if self.slam is not None:
                    self.slam.start(fx[1], fx[2], fx[3], self.on_slam_started)
            elif kind == "slam_save":
                self.save_map(fx[1])
            elif kind == "initial_pose":
                self.publish_initial_pose(*fx[1:])
            elif kind == "masks":
                self.publish_masks()
            elif kind == "arm":
                self.p_arm.publish(String(data=json.dumps(fx[1])))
            elif kind in ("face", "face_hear"):          # omni_face: a line to say (the Say box), a typed "heard"
                pub = self.p_face_say if kind == "face" else self.p_face_hear
                if pub.get_subscription_count() == 0:
                    self.emit("warn", "The face isn't running: nothing said (it runs with the console's nodes)")
                pub.publish(String(data=json.dumps(fx[1]) if kind == "face" else fx[1]))
        self.flush_events()

    def on_arm_state(self, m: String):
        """omni_arm's /arm/state: the Tidy tab's arm, and the end of a tidy (design.md 11)."""
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        now = time.monotonic()
        self.core.now = now
        self.core.caps["arm"] = bool(s.get("connected"))
        self.run_effects(self.core.on_arm_state(s, now))

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
                self.core.on_nav_done(seq, "rejected", time.monotonic())
                self.flush_events()
                return
            self.nav_handle = h

            def result(rf):
                res = rf.result()
                self.core.on_nav_done(seq, NAV_OUTCOME.get(res.status if res else -1, "aborted"), time.monotonic())
                self.flush_events()
            h.get_result_async().add_done_callback(result)
        fut.add_done_callback(accepted)

    def cancel_goal(self):
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()
            self.nav_handle = None

    def save_map(self, name: str):
        if not self.c_serialize.service_is_ready():
            self.emit("error", "Can't save: slam_toolbox isn't answering")
            return
        base = str(self.maps_dir / name)
        g = self.grid_msg                            # the image (.pgm / .yaml, map_server's format) from our /map
        if g is not None:
            o = g.info.origin
            write_map_image(base, g.info.width, g.info.height, g.info.resolution,
                            (o.position.x, o.position.y, yaw_of(o.orientation)), g.data)
        r2 = self.SerializePoseGraph.Request()
        r2.filename = base
        old = self.core.map_name

        def done(f):
            ok = f.result() is not None and getattr(f.result(), "result", 0) == 0
            if not ok:
                self.emit("error", f"Saving the map as '{name}' failed (slam_toolbox log)")
                return
            if old in (None, NEW_MAP) or old != name:               # its places, routes, zones and edges go with it
                save_json(self.extras_path(name), self.core.extras())
                self.core.map_name = name
            self.core.set_slam(map=name, maps=saved_maps(self.maps_dir))
            self.emit("info", f"Map saved as '{name}'")
            self.publish_schema()
        self.c_serialize.call_async(r2).add_done_callback(done)

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2
        cov[35] = math.radians(15) ** 2
        m.pose.covariance = cov
        self.p_initial.publish(m)

    def check_nav(self):
        """Nav2 counts only when its lifecycle manager says it is active (an inactive bt_navigator still has its action
        server, and rejects every goal). A failed bring-up (the map frame came late) is reset and started again."""
        if not self.c_nav_active.service_is_ready():
            self.nav_active = False
            return
        self.c_nav_active.call_async(Trigger.Request()).add_done_callback(
            lambda f: setattr(self, "nav_active", bool(f.result() and f.result().success)))
        now = time.monotonic()
        if (not self.nav_active and now - self.t_nav_retry > 30.0 and self.c_nav_manage.service_is_ready()
                and (self.frame_pose("base_link") or ("",))[0] == "map"):
            self.t_nav_retry = now
            self.emit("warn", "Navigation didn't come up (it started before the map): restarting it")

            def reset_done(f):
                self.c_nav_manage.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))
            self.c_nav_manage.call_async(ManageLifecycleNodes.Request(
                command=ManageLifecycleNodes.Request.RESET)).add_done_callback(reset_done)

    def check_slam_frame(self, up: bool):
        """slam_toolbox running for 40 s without a map frame: its lifecycle bring-up failed (seen in the sim after a
        restart under load: its change_state reply timed out, so it stayed inactive). Start it again, once a minute at
        most: a robot that boots unattended has to come up by itself."""
        now = time.monotonic()
        if not up or self.slam.busy:
            self.t_slam_up = None
            return
        self.t_slam_up = self.t_slam_up or now
        in_map = (self.frame_pose("base_link") or ("",))[0] == "map"
        if not in_map and now - self.t_slam_up > 40.0 and now - self.t_slam_retry > 60.0:
            self.t_slam_retry = now
            self.emit("warn", f"SLAM ({self.slam.mode}) gave no map frame in 40 s: starting it again")
            pose = self.core.pose[1:] if self.core.pose and self.core.pose[0] == "map" else (0.0, 0.0, 0.0)
            self.core.set_slam(busy=True)
            self.slam.start(self.slam.mode, self.slam.map, pose, self.on_slam_started)

    def check_caps(self):
        names = set(self.get_node_names())
        self.check_nav()
        nav = self.nav_active and self.nav.server_is_ready()
        self.core.set_caps(nav=nav, catch="catch_behavior" in names, collision=nav and "collision_monitor" in names)
        self.core.now = time.monotonic()
        self.core.caps["arm"] = self.core._arm_up()
        slam_up = "slam_toolbox" in names
        if self.slam is not None:
            up = slam_up and self.slam.alive()
            self.core.set_slam(mode=self.slam.mode if up else None, busy=self.slam.busy and not up)
            if up:
                self.slam.busy = False
            self.check_slam_frame(up)
        else:                                        # the launch runs slam_toolbox (or nothing): ask it its mode
            if slam_up and not self.ext_slam_asked and self.params["slam_toolbox"].services_are_ready():
                self.ext_slam_asked = True
                fut = self.params["slam_toolbox"].get_parameters(["mode"])
                fut.add_done_callback(lambda f: setattr(self, "ext_slam_mode",
                                                        f.result().values[0].string_value if f.result() else None))
            if not slam_up:
                self.ext_slam_asked, self.ext_slam_mode = False, None
            self.core.set_slam(mode=self.ext_slam_mode, managed=False, maps=saved_maps(self.maps_dir))
        self.n_caps += 1
        if self.n_caps % 3 == 1:                     # cameras: image topics, and the video server for them
            topics = self.get_topic_names_and_types()
            have = {n for n, _ in topics}
            # a preview counts only when its camera is there (image_preview advertises them all the same)
            self.images = sorted(n for n, types in topics if any(t in IMAGE_TYPES for t in types)
                                 and (not n.endswith("/preview/compressed")
                                      or n[:-len("/preview/compressed")] in have and self.count_publishers(
                                          n[:-len("/preview/compressed")]) > 0))
            self.video = "web_video_server" in names
        want_cam = self.p_camera.get_subscription_count() > 0          # the Vision tab is open somewhere
        if want_cam and self.cam_sub is None:
            self.cam_sub = self.create_subscription(SparseImage, "/cam/left/sparse", self.on_sparse,
                                                    qos_profile_sensor_data)
        elif not want_cam and self.cam_sub is not None:
            self.destroy_subscription(self.cam_sub)
            self.cam_sub = None
        self.health_tick()
        self.session_tick()
        self.flush_events()

    def health_tick(self):
        """1 Hz: the Health tab's history and counters (and balls fired, from the MCU's fire counter)."""
        h = self.core.health
        mcu = self.core.mcu
        if mcu is not None:
            done = mcu.get("fire_seq_done")
            if self.fires_seen is not None and done is not None and done != self.fires_seen:
                h.count("fires", (done - self.fires_seen) & 0xFFFF)
            self.fires_seen = done
        odom = self.tf.lookup("odom", "base_link")
        h.sample(time.time(), mcu, self.info, self.read_sys(), None if odom is None else odom[:2])
        if self.n_caps % 30 == 0 and h.changed:
            h.changed = False
            save_json(self.health_path, h.saved())

    def session_tick(self):
        """1 Hz: a session log (session_log.py) from the first mission after IDLE until 30 s back in IDLE."""
        c = self.core
        now = time.time()
        if c.mission != "IDLE":
            self.t_idle = None
            if self.session is None:
                i = self.info
                self.session = session_log.SessionLog(
                    self.sessions_dir, c.mission, source=self.log_source, start=now,
                    software=load_json(self.version_file) or None, console=VERSION,
                    firmware=None if i is None else dict(build=i["build"], protocol=i["protocol"]),
                    map=c.map_name, settings=dict(c.values))
        elif self.session is not None:
            self.t_idle = self.t_idle or now
            if now - self.t_idle > SESSION_IDLE_END:
                s = self.session.close(now)
                self.session = None
                self.emit("info", f"Session logged: {s['name']} ({s['duration_s'] / 60:.0f} min, {s['throws']} throws)")
                return
        if self.session is None:
            return
        log = self.session
        fp = self.frame_pose("base_link")
        if fp is not None:
            log.write("pose", frame=fp[0], x=round(fp[1], 3), y=round(fp[2], 3), yaw=round(fp[3], 3))
        m = c.mcu or {}
        log.write("status", mission=c.mission, task=c.task, v_bus=m.get("v_bus"), battery=m.get("battery_pct"),
                  mcu=m.get("mode"), faults=[f["name"] for f in m.get("faults", [])])
        if self.n_caps % 10 == 0:
            log.flush()

    # -- outputs -----------------------------------------------------------------------------------------------
    def drive_tick(self):
        """The driving missions' twist (missions.Console.drive_output): the stick in MANUAL, the follower, a program,
        a calibration drive. Through Nav2's collision monitor when it runs (Manual: if its setting says so)."""
        tw = self.core.drive_output(time.monotonic())
        self.run_effects(self.core.pop_fx())
        if tw is None:
            return
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = tw
        guard = self.core.values["manual.obstacle_stop"] if self.core.mission == "MANUAL" else True
        (self.p_cmd_safe if guard and self.core.caps["collision"] else self.p_cmd).publish(m)

    def fast_tick(self):
        """20 Hz, one timer for everything periodic (each wake-up costs): the inputs (drained, never woken per
        message), the driving missions' twist, the thrower's command, and every second tick the status."""
        self.n_fast += 1
        now = time.monotonic()
        self.tick_gap, self._t_fast = now - getattr(self, "_t_fast", now), now      # a stalled executor shows here
        self._drain()
        if self.feed.flood:                           # a backlog: something floods a feed topic, or a tick stalled
            self.get_logger().warn(f"{self.feed.n_taken} feed messages waited for one tick (capped; {self.feed.flood}: "
                                   f"{self.feed.flood_info}; tick gap {self.tick_gap:.2f} s)", throttle_duration_sec=10.0)
        if self.feed.n_repeats:
            self.get_logger().info(f"{self.feed.n_repeats} drains so far ended on a message rmw handed back twice "
                                   "(dropped)", throttle_duration_sec=300.0)
        odom = self.tf.lookup("odom", "base_link")
        self.core.odom_pose = None if odom is None else odom[:3]
        if self.mcu is not None:
            self.core.on_mcu(st.mcu_summary(self.mcu, time.monotonic() - self.mcu_t))
        follow = self.core.mission == "FOLLOW"
        if follow != (self.feed.scan_sub is not None):
            self.feed.set_scan(self.scan_topic if follow else None, self.on_scan_raw)
            self.scan_laser = None
        if self.n_fast % 2 == 0:
            self._status()
        self._drive()
        t = self.core.thrower_cmd()
        if t is not None:
            m = McuCommand()
            m.tilt, m.fly_rpm, m.fire_seq = float(t[0]), float(t[1]), int(t[2])
            m.header.stamp = self.get_clock().now().to_msg()
            self.p_thrower.publish(m)

    def mission_tick(self):
        grid = None
        if self.core.mission == "EXPLORE" and self.grid_msg is not None:
            if self.grid is None:
                self.grid = Grid.from_msg(self.grid_msg)
            grid = self.grid
        self.run_effects(self.core.tick(time.monotonic(), grid))

    def emit(self, level: str, text: str, source: str = "console"):
        self.p_events.publish(String(data=json.dumps(dict(t=time.time(), level=level, text=text, source=source))))
        if self.session is not None and (level != "info" or source == "console"):
            self.session.write("event", level=level, text=text, source=source)

    def flush_events(self):
        for level, text in self.core.pop_events():
            self.emit(level, text)

    def publish_schema(self):
        self.p_schema.publish(String(data=json.dumps(self.core.schema())))

    def publish_version(self):
        pk = {}
        try:
            from ament_index_python.packages import get_package_share_directory
            import xml.etree.ElementTree as ET
            for name in ("omni_bringup", "omni_catch", "omni_console", "omni_interfaces", "omni_mcu",
                         "omni_perception"):
                try:
                    root = ET.parse(Path(get_package_share_directory(name)) / "package.xml").getroot()
                    pk[name] = root.findtext("version")
                except Exception:                    # a package missing on this machine
                    pk[name] = None
        except ImportError:
            pass
        expected = st.firmware_build_id(Path(self.firmware_dir) / "lib" / "omni_fw") if self.firmware_dir else None
        from omni_mcu.protocol import PROTOCOL_VERSION
        i = self.info
        v = dict(console=VERSION, software=load_json(self.version_file) or None, packages=pk,
                 ros=os.environ.get("ROS_DISTRO"), host=platform.node(), os=platform.platform(),
                 python=platform.python_version(),
                 firmware=dict(expected=expected, running=i["build"] if i else None,
                               protocol=i["protocol"] if i else None, protocol_host=PROTOCOL_VERSION,
                               dir=self.firmware_dir or None),
                 slam_log=str(self.slam.log_path) if self.slam else None, video=self.video,
                 topics=dict(scan=self.scan_topic, map="/map", samples="/mcu/samples"),
                 load=self.load.last)                     # the node's own callbacks, % of a core (LoadMeter)
        lp = self.tf.lookup("base_link", "laser")         # where the page puts the raw scan's origin
        v["frames"] = dict(laser=None if lp is None else [round(lp[0], 4), round(lp[1], 4), round(lp[2], 5)])
        self.p_version.publish(String(data=json.dumps(v)))

    def save(self):
        if self.core.settings_changed:
            self.core.settings_changed = False
            save_json(self.settings_path, dict(settings=self.core.values))
        if self.core.places_changed:
            self.core.places_changed = False
            save_json(self.extras_path(self.core.map_name), self.core.extras())
        if self.core.programs_changed:
            self.core.programs_changed = False
            save_json(self.programs_path, dict(programs=self.core.programs))
        if self.core.calib_changed:
            self.core.calib_changed = False
            save_json(self.calib_path, dict(results=self.core.calib_results))

    def read_wifi(self):
        now = time.monotonic()
        if now - self.wifi_t > 2.0:
            self.wifi_t = now
            try:
                self.wifi = st.parse_wireless(Path("/proc/net/wireless").read_text())
            except OSError:
                self.wifi = None
        return self.wifi

    def read_sys(self):
        now = time.monotonic()
        if now - self.sys_t > 2.0:
            self.sys_t, self.sys = now, st.system()
        return self.sys

    def publish_status(self):
        now = time.monotonic()
        self.consoles = {k: t for k, t in self.consoles.items() if now - t < 3.0}
        fp = self.frame_pose("base_link")
        self.core.pose = None if fp is None else fp[:4]
        s = self.core.summary()
        s["t"] = time.time()
        s["map"] = self.core.map_name
        s["mcu"] = None if self.mcu is None else st.mcu_summary(self.mcu, now - self.mcu_t)
        s["link"] = self.link
        # t: the pose's stamp (robot clock; sim time in the sim), which the page matches to the raw scan's stamp
        s["pose"] = None if fp is None else dict(frame=fp[0], x=round(fp[1], 3), y=round(fp[2], 3),
                                                 yaw=round(fp[3], 4), t=round(fp[4], 4))
        if s["pose"] is not None and s["mcu"] is not None and s["mcu"]["age"] > 0.5:
            # no odometry from the MCU: the EKF keeps predicting with its last velocity, so the pose drifts away from
            # a robot that has stopped (found with the sim's fault injection)
            s["pose"]["stale"] = True
        tr = self.latest_track()
        fresh = tr is not None and now - self.t_track < 0.6
        ball = tr["x"][:3] if fresh and tr["tracking"] else None
        s["catch"] = None if self.catch is None else st.catch_summary(dict(self.catch, ball=ball,
                                                                           target=self.target))
        if tr is not None and not fresh:
            tr = dict(tr, tracking=False)
        s["vision"] = dict(perception=self.perception, track=tr, images=self.images,
                           video_port=VIDEO_PORT if self.video else None)
        i = self.info
        s["rc"] = None if i is None else dict(ok=i["rc_ok"], manual=i["rc_manual"], ch=i["rc"],
                                              age=round(now - self.info_t, 2))
        s["fw"] = None if i is None else dict(build=i["build"], protocol=i["protocol"])
        s["wifi"] = self.read_wifi()
        s["sys"] = self.read_sys()
        s["consoles"] = len(self.consoles)
        tw = self.core.last_cmd if self.core.manual_active(now) else None      # what was sent (slow bands applied)
        s["manual"] = dict(active=tw is not None, cmd=None if tw is None else [round(v, 2) for v in tw])
        self.p_status.publish(String(data=json.dumps(s)))
        self.save()
        self.flush_events()

    def shutdown(self):
        if self.session is not None:
            self.session.close()
            self.session = None
        save_json(self.health_path, self.core.health.saved())
        if self.slam is not None:
            self.slam.shutdown()


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)                              # node.feed is drained, never spun
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.feed.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
