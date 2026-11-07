"""arm_brain: the arm's planner and tidy task (design.md 11, 4.10), the other half of omni_cpp's arm_driver, which owns
the servo bus and does everything that repeats every cycle (the reads and writes, the joint states, TF, /arm/out). This
node sleeps between requests and wakes only to plan a motion, and to run the tidy task while it runs. (A one-process
Python arm_node did both until Nov 7, 2026; it was deleted with the other Python twins of C++ nodes.)

The brain is the one writer of /arm/motion. Each motion goes out whole (waypoints and times; the driver plays the
quintic segments itself), so nothing streams from here at 50 Hz and a brain that dies mid-move leaves the arm finishing
the move and holding. Odom, depth, wrist and joint-state subscriptions exist only while the tidy runs.

Subscribes /arm/request             std_msgs/String JSON {"verb": ...} from the console's mission node (or by hand):
                                      tidy {max_objects, radius, only}   start the tidy task (the mode manager must be
                                                                    in PICK); only: one object of the last find
                                      find {id, color, shape, max_range}   the floor's candidates by colour and shape
                                                                    (color_vision.py): the answer is /arm/state's
                                                                    "found", the picture /arm/find_image
                                      stop                          stop the task or motion; the arm stows
                                      pose {name: stow | ready | high | drop}   move there
                                      gripper {gap: m}              open / close to a gap
                                      jog {joint: name, delta: rad} one joint by delta (kept in range, refused if
                                                                    it would hit the robot, the bumper or the floor)
                                      hold | relax                  torque on, hold here | torque off (the arm drops:
                                                                    hold it; the SO-101 has no brakes)
           /arm/hw                  omni_interfaces/ArmHardware (5 Hz): connected, the measured joints
           the tidy's inputs, only while it runs: odom_topic (/odom), depth_topic (+ camera_info): /stereo/depth 32FC1,
           or with config:=d435 the D435's /d435/depth/image_rect_raw 16UC1 mm (design.md 4.11); /arm_cam/image_raw
           (+ camera_info), /arm/joint_states (the task steps on each message); a find's, only while it waits: odom,
           depth (+ camera_info), color_topic (rgb8 registered to the depth: /stereo/left/image_color or
           /d435/color/image_raw)
Publishes  /arm/motion              omni_interfaces/ArmMotion: the driver plays it
           /arm/state               std_msgs/String JSON: connected, source (who sets the goals: hold | pose | trajectory
                                    | tidy | relaxed), the task's status and its last events; at 1 Hz, and 5 Hz while
                                    tidying, and at once when a find answers ("found"). The servo fields (joints,
                                    loads, temperatures) are /arm/hw's: the console reads both
           /arm/find_image          sensor_msgs/Image rgb8: the last find's annotated picture, sent again (2 Hz) while
                                    anyone subscribes, for a minute after the find
           /pick/cmd_vel            geometry_msgs/Twist: the task's base command (the mode manager forwards it in PICK)
           /arm/camera_request      std_msgs/String "depth" | "wrist" when the task wants a frame
Action     /arm/follow_joint_trajectory   control_msgs/FollowJointTrajectory (played by the driver in straight lines)
Parameters odom_topic, config ("" | d435 | a json path), depth_topic, depth_info_topic, color_topic.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from omni_interfaces.msg import ArmHardware, ArmMotion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

from omni_arm import motion as mo
from omni_arm.color_vision import ColorFrame, Finder
from omni_arm.brain_core import HEALTH_GRACE, MotionSync, state_from_joints
from omni_arm.floor_vision import DepthFrame, decode_depth
from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path, q_for_gap
from omni_arm.pick import MJ_TO_OPTICAL, Inputs, TidyTask, camera_view, wrist_frame

HW_STALE = 2.0            # s without /arm/hw: the driver isn't running
ODOM_WAIT = 1.0           # s a starting tidy waits for the first odometry


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ArmBrain(Node):
    def __init__(self):
        super().__init__("arm_node")
        def P(name, default, text, rng=None, read_only=True):
            """A described parameter (omni_cpp's params.hpp in Python): range for numbers, read-only unless live."""
            d = ParameterDescriptor(description=text, read_only=read_only)
            if rng is not None:
                d.floating_point_range = [FloatingPointRange(from_value=float(rng[0]), to_value=float(rng[1]), step=0.0)]
            return self.declare_parameter(name, default, d).value

        self.odom_topic = P("odom_topic", "/odom", "the base's odometry for the tidy (the MCU's)")
        config = P("config", "", "the robot_arm*.json: \"\" | d435 | a path")
        self.depth_topic = P("depth_topic", "/d435/depth/image_rect_raw" if config == "d435" else "/stereo/depth",
                             "the tidy's depth image")
        self.info_topic = P("depth_info_topic", "", "its camera_info (\"\" = beside depth_topic)") or \
            self.depth_topic.rsplit("/", 1)[0] + "/camera_info"
        self.color_topic = P("color_topic", "/d435/color/image_raw" if config == "d435" else "/stereo/left/image_color",
                             "colour registered to the depth, for finds")
        self.health_grace = P("health_grace", HEALTH_GRACE, "s of unhealthy servo feedback a tidy rides through",
                              (0.05, 5.0), read_only=False)
        self.hw_stale = P("hw_stale", HW_STALE, "s without /arm/hw: the driver isn't running", (0.2, 30.0),
                          read_only=False)
        self.add_post_set_parameters_callback(self._on_params)
        self.kin = ArmKinematics(ArmConfig(config_path(config)))
        self.cfg = self.kin.cfg
        self.task = TidyTask(self.kin)
        self.tidy_on = False
        self.starting = False                  # the tidy is waiting for its first odometry
        self.motion: mo.Motion | None = None   # the last pose / jog / gripper motion sent (kept after it ends)
        self.goal = None                       # where the arm is commanded when nothing plays
        self.source = "hold"
        self.pose = None
        self.depth = self.wrist = None
        self.K_depth = self.K_wrist = None
        self.q_hist = deque(maxlen=200)
        self.want = {"depth": False, "wrist": False}
        self.events = deque(maxlen=12)
        self.n_ev = self.n_task_ev = 0
        self.traj_goal = None
        self.hw: ArmHardware | None = None
        self.t_hw = -math.inf
        self.t_healthy = -math.inf
        self.connected = False
        self.motion_id = 0
        self.sync = MotionSync()               # what the task's motion last sent to the driver (brain_core)
        self._args: dict = {}
        self._end_timer = None
        self._start_timer = None
        self.k = 0
        self.subs: list = []
        self.finder = Finder()                 # the AI's finds (color_vision.py); its map keeps the ids for a pick
        self.color = None
        self.find_subs: list = []              # a find's inputs while it waits (the tidy's own when it runs)
        self._find_timer = None
        self.t_found, self.t_find_pub = -math.inf, -math.inf
        self.p_motion = self.create_publisher(ArmMotion, "/arm/motion", 10)
        self.p_find = self.create_publisher(Image, "/arm/find_image", 2)
        self.p_state = self.create_publisher(String, "/arm/state", 10)
        self.p_twist = self.create_publisher(Twist, "/pick/cmd_vel", 10)
        self.p_camreq = self.create_publisher(String, "/arm/camera_request", 10)
        self.create_subscription(String, "/arm/request", self.on_request, 10)
        self.create_subscription(ArmHardware, "/arm/hw", self.on_hw, 10)
        self.create_timer(1.0, self.publish_state)
        self.T_depth = np.eye(4)
        c = self.cfg.cameras[0]
        self.T_depth[:3, :3], self.T_depth[:3, 3] = np.array(c["R"]), c["pos"]
        self.T_depth = self.T_depth @ MJ_TO_OPTICAL
        cb = ReentrantCallbackGroup()
        try:
            from control_msgs.action import FollowJointTrajectory
            self.FJT = FollowJointTrajectory
            ActionServer(self, FollowJointTrajectory, "/arm/follow_joint_trajectory", self.execute_traj,
                         goal_callback=self.accept_traj, cancel_callback=lambda g: CancelResponse.ACCEPT,
                         callback_group=cb)
        except ImportError:
            self.get_logger().warn("control_msgs not installed: no /arm/follow_joint_trajectory action")
        self.get_logger().info(f"arm brain: poses stow / ready / high / drop from {config_path(config).name}; "
                               f"depth from {self.depth_topic} ({c['name']}); the driver plays what it sends")

    def _on_params(self, params):
        for p in params:                          # ranges and read-only already checked by ROS
            if p.name in ("health_grace", "hw_stale"):
                setattr(self, p.name, float(p.value))

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, text: str):
        self.n_ev += 1
        self.events.append(dict(n=self.n_ev, t=round(self.now(), 2), text=text))
        self.get_logger().info(text)

    @staticmethod
    def _k(m: CameraInfo):
        return (m.k[0], m.k[4], m.k[2], m.k[5])

    # -- the driver ------------------------------------------------------------------------------------------------
    def on_hw(self, m: ArmHardware):
        was = self.connected
        self.hw, self.t_hw, self.connected = m, self.now(), bool(m.connected)
        if self.connected and not was:
            self.goal = np.array(m.position)
            self.motion, self.source = None, "hold"
            self.sync.reset()
            self.event(f"arm connected on {m.port}: holding where it is")
        elif was and not self.connected:
            self.event("servo bus lost")

    def q_now(self) -> np.ndarray:
        """The measured joints (/arm/hw's: up to 0.2 s old; the task has /arm/joint_states every cycle)."""
        return np.array(self.hw.position) if self.hw is not None else np.zeros(6)

    def cur_goal(self) -> np.ndarray:
        """Where the arm is commanded now: a motion sampled, the task's goal, or the held pose."""
        if self.tidy_on and self.task.goal is not None:
            return self.task.goal.copy()
        if self.motion is not None:
            return self.motion.sample(self.now())
        return self.goal.copy() if self.goal is not None else self.q_now()

    def send(self, command: int, times=(), waypoints=(), interpolation: int = ArmMotion.QUINTIC):
        m = ArmMotion()
        m.header.stamp = self.get_clock().now().to_msg()
        self.motion_id += 1
        m.id = self.motion_id
        m.command, m.interpolation = command, interpolation
        m.times = [float(t) for t in times]
        m.waypoints = [float(v) for w in waypoints for v in w]
        self.p_motion.publish(m)

    def play(self, motion: mo.Motion):
        """A pose / jog / gripper motion: sent whole; the source goes back to hold when it ends."""
        self.motion, self.source = motion, "pose"
        self.send(ArmMotion.MOVE, motion.t, motion.q)
        self._after(motion.t_end - self.now() + 0.05, lambda m=motion: self._motion_ended(m))

    def _after(self, delay: float, fn):
        if self._end_timer is not None:
            self.destroy_timer(self._end_timer)
        holder = []

        def once():
            self.destroy_timer(holder[0])
            if self._end_timer is holder[0]:
                self._end_timer = None
            fn()

        holder.append(self.create_timer(max(0.02, delay), once))
        self._end_timer = holder[0]

    def _motion_ended(self, m):
        if self.motion is m and self.traj_goal is None and not self.tidy_on:
            self.source = "hold"

    # -- the tidy --------------------------------------------------------------------------------------------------
    def _listen(self):
        """The tidy's inputs, only while it runs: nothing here wakes the brain when the arm is idle."""
        info_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ci = self.create_subscription
        self.subs = [
            ci(Odometry, self.odom_topic, self.on_odom, 20),
            ci(Image, self.depth_topic, self.on_depth, 2),
            ci(CameraInfo, self.info_topic, lambda m: setattr(self, "K_depth", self._k(m)), info_qos),
            ci(Image, "/arm_cam/image_raw", self.on_wrist, 2),
            ci(CameraInfo, "/arm_cam/camera_info", lambda m: setattr(self, "K_wrist", self._k(m)), info_qos),
            ci(JointState, "/arm/joint_states", self.on_joints, 10),
        ]

    def _stop_listening(self):
        for s in self.subs:
            self.destroy_subscription(s)
        self.subs = []
        self.want = {"depth": False, "wrist": False}
        self.q_hist.clear()
        self.wrist = None
        if not self.find_subs:                 # a find still waiting keeps the pose and the depth
            self.pose = self.depth = None

    def _begin_tidy(self, args: dict):
        self.motion = None
        self.task = TidyTask(self.kin)
        self.n_task_ev = 0
        self.sync.reset()
        self.starting = True
        self._args = args
        self._listen()
        self._start_timer = self.create_timer(ODOM_WAIT, self._start_task)    # no odometry: start without it

    def _start_task(self):
        if self._start_timer is not None:
            self.destroy_timer(self._start_timer)
            self._start_timer = None
        if not self.starting:
            return
        self.starting = False
        a, t = self._args, self.now()
        self.task.history.append((t, self.pose or (0.0, 0.0, 0.0)))
        only = a.get("only")
        self.task.start(t, int(a.get("max_objects", 0)) or None, float(a.get("radius", 0.0)) or None,
                        only=None if only is None else int(only), fmap=None if only is None else self.finder.map)
        self.tidy_on, self.source = True, "tidy"
        self.event("tidy: started" if only is None else f"tidy: started, object {only} only")

    def _end_tidy(self):
        self.tidy_on = self.starting = False
        if self._start_timer is not None:
            self.destroy_timer(self._start_timer)
            self._start_timer = None
        self.goal = self.task.goal.copy() if self.task.goal is not None else self.goal
        self._stop_listening()
        self.source = "hold"

    def on_odom(self, m: Odometry):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, _yaw(p.orientation))
        if self.starting:
            self._start_task()

    def on_depth(self, m: Image):
        z = None if self.K_depth is None else decode_depth(m.encoding, bytes(m.data), m.height, m.width)
        if z is None:
            self.get_logger().warn(f"depth frame dropped: {'no camera_info yet' if self.K_depth is None else m.encoding}",
                                   throttle_duration_sec=5.0)
            return
        self.depth = DepthFrame(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, z, self.K_depth, self.T_depth)

    def on_wrist(self, m: Image):
        if self.K_wrist is None or m.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == "bgr8":
            img = img[..., ::-1]
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.wrist = wrist_frame(self.kin, t, img.copy(), self.K_wrist, self.q_at(t))

    def q_at(self, t: float):
        best = None
        for tt, q in self.q_hist:
            if tt <= t + 1e-6:
                best = q
        return best if best is not None else (self.q_hist[-1][1] if self.q_hist else np.zeros(6))

    def on_joints(self, m: JointState):
        """The task's cycle: one step per joint-state message (the driver's rate), the goals sent as motions."""
        if not self.tidy_on:
            return
        t = self.now()
        hw = self.hw
        if hw is not None and hw.ok and t - self.t_hw < 0.5:            # /arm/hw's health, 5 Hz
            self.t_healthy = t
        if t - self.t_healthy > self.health_grace:
            # a servo silent or reporting an error (or no word from the driver): hold where commanded, stop the base
            self.task.state, self.task.done_reason = "DONE", "servo feedback"
            self._end_tidy()
            self.send(ArmMotion.HOLD)
            self.goal = self.q_now()
            self.p_twist.publish(Twist())
            self.event(f"tidy stopped: servo feedback unhealthy (missing {list(hw.missing) if hw else '?'}); holding")
            self.publish_state()
            return
        q = np.array(m.position)
        self.q_hist.append((t, q.copy()))
        st = state_from_joints(t, q, m.velocity, m.effort)
        out = self.task.step(t, Inputs(self.pose or (0.0, 0.0, 0.0), st, self.depth, self.wrist))
        self._sync_goal(t)
        for kind, want in (("depth", out.want_depth), ("wrist", out.want_wrist)):
            if want and not self.want[kind]:
                self.p_camreq.publish(String(data=kind))
            self.want[kind] = want
        tw = Twist()
        if out.twist is not None:
            tw.linear.x, tw.linear.y, tw.angular.z = map(float, out.twist)
        self.p_twist.publish(tw)
        for ev in self.task.events[self.n_task_ev:]:
            self.event(f"tidy: {ev[2]}")
        self.n_task_ev = len(self.task.events)
        if self.task.state == "DONE":
            self._end_tidy()
            self.event(f"tidy done: {self.task.picked} picked, {self.task.failed} not ({self.task.done_reason})")
            self.publish_state()
            return
        self.k += 1
        if self.k % 10 == 0:
            self.publish_state()

    def _sync_goal(self, t: float):
        """Tell the driver what the task now wants (brain_core.MotionSync: the sim's harness does the same)."""
        for cmd, times, wps, linear in self.sync.commands(self.task, t):
            self.send(cmd, times, wps, ArmMotion.LINEAR if linear else ArmMotion.QUINTIC)

    # -- the AI's finds (color_vision.py) ---------------------------------------------------------------------------
    def on_color(self, m: Image):
        if m.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        self.color = ColorFrame(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                (img[..., ::-1] if m.encoding == "bgr8" else img).copy())

    def _find(self, a: dict):
        why = self.finder.request(a, self.now())
        if why:
            self.event(f"find: {why}")
            self.publish_state()
            return
        if not self.find_subs:
            info_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            ci = self.create_subscription
            self.find_subs = [ci(Image, self.color_topic, self.on_color, 2)]
            if not self.subs:                  # the tidy's own subscriptions bring the pose and the depth otherwise
                self.find_subs += [
                    ci(Odometry, self.odom_topic, self.on_odom, 20),
                    ci(Image, self.depth_topic, self.on_depth, 2),
                    ci(CameraInfo, self.info_topic, lambda m: setattr(self, "K_depth", self._k(m)), info_qos)]
        if self._find_timer is None:
            self._find_timer = self.create_timer(0.1, self._find_step)
        self.event(f"find: looking for {a.get('color') or 'anything'} {a.get('shape') or ''}".rstrip())

    def _find_step(self):
        t = self.now()
        if self.finder.due(t):                 # frames from after the robot settled (the sim renders on request)
            for kind in ("depth", "color"):
                self.p_camreq.publish(String(data=kind))
        if self.finder.req is not None:
            if not self.finder.step(t, self.depth, self.color, self.pose):
                return
            r = self.finder.result
            self.event(f"find: {r['error']}" if r.get("error") else
                       f"find: {len(r['candidates'])} candidates, {len(r['image_only'])} image-only")
            for s in self.find_subs:
                self.destroy_subscription(s)
            self.find_subs, self.color = [], None
            if not self.tidy_on and not self.starting:
                self.pose = self.depth = None
            self.t_found, self.t_find_pub = time.monotonic(), -math.inf
            self.publish_state()
        img, now = self.finder.image, time.monotonic()
        if img is None or now - self.t_found > 60.0:
            if self._find_timer is not None:
                self.destroy_timer(self._find_timer)
                self._find_timer = None
            return
        if now - self.t_find_pub > 0.5 and (self.p_find.get_subscription_count() > 0 or self.t_find_pub == -math.inf):
            self.t_find_pub = now
            m = Image()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "arm_find"
            m.height, m.width = img.shape[:2]
            m.encoding, m.step = "rgb8", img.shape[1] * 3
            m.data = np.ascontiguousarray(img).tobytes()
            self.p_find.publish(m)

    # -- requests --------------------------------------------------------------------------------------------------
    def on_request(self, m: String):
        try:
            a = json.loads(m.data)
            verb = str(a.get("verb", ""))
        except (ValueError, AttributeError):
            self.get_logger().warn(f"/arm/request: not JSON: {m.data!r}")
            return
        if verb == "find":                     # the cameras alone: no servo bus needed
            self._find(a)
            return
        t = self.now()
        if not self.connected or t - self.t_hw > self.hw_stale:
            self.event(f"{verb}: the arm isn't connected")
            return
        q = self.q_now()
        if verb in ("tidy", "stop", "pose", "gripper", "jog", "hold", "relax") and self.traj_goal is not None:
            self.traj_goal = None            # any request takes over from a trajectory: the driver stops it here
            self.send(ArmMotion.HOLD)
            self.goal, q = self.q_now(), self.q_now()
        here = self.cur_goal()                           # planning starts from where the arm is commanded
        if verb == "tidy":
            if self.tidy_on or self.starting:
                return
            self._begin_tidy(a)
        elif verb == "stop":
            if self.tidy_on:
                self.task.stop(t)           # the task stows (and finishes a drop in progress), then ends
                self.event("tidy: stopping")
            elif self.starting:
                self._end_tidy()
            elif not self._move_to(here, self.task.poses["stow"], "stow"):
                self.send(ArmMotion.HOLD)                        # no checked way to stow: stop where it is
                self.goal, self.motion, self.source = q, None, "hold"
        elif verb == "pose":
            name = str(a.get("name", ""))
            if self.tidy_on or name not in self.task.poses:
                self.event(f"pose {name}: {'the tidy runs' if self.tidy_on else 'unknown'}")
                return
            self._move_to(here, self.task.poses[name], name)
        elif verb == "gripper":
            if self.tidy_on:
                return
            g = here.copy()
            g[5] = q_for_gap(self.cfg, float(a.get("gap", 0.0)))
            self.play(mo.Motion([here, g], mo.speeds(self.kin), t))
        elif verb == "jog":
            name = str(a.get("joint", ""))
            if self.tidy_on or name not in self.cfg.joints:
                self.event(f"jog {name}: {'the tidy runs' if self.tidy_on else 'unknown joint'}")
                return
            playing = self.motion is not None and not self.motion.done(t)
            base = self.motion.q[-1] if playing else here          # clicks while one plays add up
            target, why = mo.jog(self.kin, here, self.cfg.joints.index(name), float(a.get("delta", 0.0)), base)
            if target is None:
                self.event(f"jog {name.replace('_', ' ')}: {why}")
                return
            self.play(mo.Motion([here, target], mo.speeds(self.kin), t))
        elif verb == "hold":
            self._halt()
            self.send(ArmMotion.HOLD)
            self.goal, self.motion, self.source = q, None, "hold"
            self.event("holding")
        elif verb == "relax":
            self._halt()
            self.send(ArmMotion.RELAX)
            self.goal, self.motion, self.source = None, None, "relaxed"
            self.event("torque off: the arm is limp")
        else:
            self.event(f"/arm/request: unknown verb {verb!r}")

    def _halt(self):
        if self.tidy_on or self.starting:
            self._end_tidy()
        self.motion = None

    def _move_to(self, q, target, name):
        t = self.now()
        P = self.task.poses
        to_bin = name == "drop"                          # its fingers are in the bin's box, as the tidy's drop
        path, why = mo.plan_checked(self.kin, q, target, via=[P[n] for n in ("high", "ready", "predrop") if n in P],
                                    allow_bin=to_bin)
        if path is None:
            self.event(f"{name}: refused, {why} (jog it clear, or move it by hand)")
            return False
        if why:
            self.event(f"{name}: {why}")
        self.play(mo.Motion(path, mo.speeds(self.kin), t))
        self.event(f"moving to {name}")
        return True

    # -- the trajectory action ---------------------------------------------------------------------------------------
    def accept_traj(self, goal):
        if self.tidy_on or self.starting or not self.connected:
            return GoalResponse.REJECT
        names = list(goal.trajectory.joint_names)
        return GoalResponse.ACCEPT if set(names) <= set(self.cfg.joints) else GoalResponse.REJECT

    def execute_traj(self, gh):
        tr = gh.request.trajectory
        res = self.FJT.Result()
        times, wps = mo.check_trajectory(
            self.kin, self.cur_goal(), tr.joint_names,
            [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, list(p.positions)) for p in tr.points])
        if times is None:
            self.event(f"trajectory refused: {wps}")
            gh.abort()
            res.error_code, res.error_string = res.INVALID_GOAL, str(wps)
            return res
        t0 = self.now()
        pts = list(zip(times, wps))
        self.traj_goal, self.source, self.motion = gh, "trajectory", None
        self.send(ArmMotion.MOVE, [t0 + s for s, _ in pts], [q for _, q in pts], ArmMotion.LINEAR)
        self.goal = pts[-1][1]
        rate = self.create_rate(10)
        while True:
            if gh.is_cancel_requested or self.traj_goal is not gh or self.tidy_on:
                if self.traj_goal is gh:
                    self.send(ArmMotion.HOLD)                  # stop where it is
                    self.goal = self.q_now()
                gh.canceled()
                res.error_code = res.INVALID_GOAL
                break
            if self.now() - t0 >= pts[-1][0]:
                gh.succeed()
                res.error_code = res.SUCCESSFUL
                break
            rate.sleep()
        if self.traj_goal is gh:
            self.traj_goal = None
            self.source = "hold"
        return res

    # -- output ----------------------------------------------------------------------------------------------------
    def publish_state(self):
        up = self.connected and self.now() - self.t_hw < self.hw_stale
        s = dict(connected=bool(up), source=self.source, port=self.hw.port if self.hw is not None else "",
                 tidy=self.task.status() if (self.tidy_on or self.task.state != "IDLE") else None,
                 tidy_on=self.tidy_on, events=list(self.events), found=self.finder.result)
        if self.hw is not None:
            s["wrist_cam"] = camera_view(self.kin, np.array(self.hw.position))
        self.p_state.publish(String(data=json.dumps(s)))


def main():
    rclpy.init()
    node = ArmBrain()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
