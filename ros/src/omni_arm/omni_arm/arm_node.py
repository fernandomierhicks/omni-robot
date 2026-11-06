"""arm_node: the SO-101 on the robot (design.md 11). One process owns the servo bus and everything that moves the arm.

Every cycle (rate, 50 Hz): read the six servos (one SYNC_READ), publish the joints, then the one source in charge sets
the goals (one SYNC_WRITE): the tidy task, a joint trajectory (the action), a named pose, or holding still.

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
           odom_topic (/odom)       nav_msgs/Odometry: the base's pose for the task (the MCU's odometry)
           depth_topic              sensor_msgs/Image, the depth for the tidy's looks (+ its camera_info): /stereo/depth
                                    32FC1 (the stereo pair's left camera), or with config:=d435 the RealSense D435's
                                    /d435/depth/image_rect_raw 16UC1 mm (design.md 4.11)
           /arm_cam/image_raw       sensor_msgs/Image rgb8, the wrist camera (+ /arm_cam/camera_info)
           color_topic              sensor_msgs/Image rgb8 registered to the depth (the same K and pixels), only while a
                                    find runs: /stereo/left/image_color, or /d435/color/image_raw with config:=d435
Publishes  /arm/joint_states        sensor_msgs/JointState (rad; effort = load, fraction of the torque limit)
           /arm/state               std_msgs/String JSON at 5 Hz: connected, torque, joints (deg), loads, temperatures,
                                    volts, out, who drives, the task's status and its last events, the last find
           /arm/find_image          sensor_msgs/Image rgb8: the last find's annotated picture, sent again (2 Hz) while
                                    anyone subscribes (web_video_server's snapshot), for a minute after the find
           /arm/out                 std_msgs/Bool at 10 Hz: the tool is out (the mode manager caps the base)
           /pick/cmd_vel            geometry_msgs/Twist: the task's base command (the mode manager forwards it in PICK)
           /arm/camera_request      std_msgs/String "depth" | "wrist" when the task wants a frame (the sim renders on
                                    request; the robot's cameras stream and ignore it)
           /tf                      base_link -> each arm link (arm_shoulder .. arm_gripper, arm_jaw, arm_tool,
                                    arm_wrist_cam_optical), from the joints
Action     /arm/follow_joint_trajectory   control_msgs/FollowJointTrajectory (MoveIt's or LeRobot's driver can use it)
Parameters port (tcp:127.0.0.1:5763 in the sim, /dev/ttyACM0 or the bus board's device on the robot), calibration
           (a LeRobot calibration json; "" = the sim's nominal one), rate, odom_topic, config ("" | d435 | a json path:
           the robot it runs on), depth_topic, depth_info_topic, color_topic.
"""

from __future__ import annotations

import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from omni_arm import motion as mo
from omni_arm.color_vision import ColorFrame, Finder
from omni_arm.driver import ArmDriver
from omni_arm.feetech import Bus, load_calibration
from omni_arm.floor_vision import DepthFrame, decode_depth
from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path, q_for_gap, rot_quat as _quat
from omni_arm.pick import MJ_TO_OPTICAL, Inputs, TidyTask, camera_view, wrist_frame
from omni_arm.transport import open_port

LINKS = ("arm_shoulder", "arm_upper_arm", "arm_lower_arm", "arm_wrist", "arm_gripper", "arm_jaw", "tool")
VERBS = ("tidy", "stop", "pose", "gripper", "jog", "hold", "relax")
HEALTH_GRACE = 0.25        # s of unhealthy feedback (a servo silent or reporting an error) a tidy rides through


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node")
        P = self.declare_parameter
        self.port_spec = P("port", "tcp:127.0.0.1:5763").value
        cal = P("calibration", "").value
        self.rate = float(P("rate", 50.0).value)
        odom_topic = P("odom_topic", "/odom").value
        # the robot: "" = robot_arm.json (the stereo pair's depth), "d435" = robot_arm_d435.json (design.md 4.11), a path
        config = P("config", "").value
        # its depth image: /stereo/depth (32FC1) or the D435's /d435/depth/image_rect_raw (16UC1 mm); the camera_info
        # beside it ("" = the image topic's namespace + /camera_info)
        depth_topic = P("depth_topic", "/d435/depth/image_rect_raw" if config == "d435" else "/stereo/depth").value
        info_topic = P("depth_info_topic", "").value or depth_topic.rsplit("/", 1)[0] + "/camera_info"
        # the depth camera's colour, registered to its depth (ai_robot_architecture.md 5.2: find)
        self.color_topic = P("color_topic", "/d435/color/image_raw" if config == "d435"
                             else "/stereo/left/image_color").value
        self.kin = ArmKinematics(ArmConfig(config_path(config)))
        self.cfg = self.kin.cfg
        self.cal = load_calibration(cal, self.cfg.joints) if cal else None
        self.driver = None
        self.t_connect = -math.inf
        self.task = TidyTask(self.kin)                   # its events are relayed (tick), not logged twice
        self.tidy_on = False
        self.motion: mo.Motion | None = None
        self.goal = None
        self.source = "hold"                 # who sets the goals: hold | pose | trajectory | tidy
        self.pose = None
        self.depth = self.wrist = None
        self.K_depth = self.K_wrist = None
        self.q_hist = deque(maxlen=200)      # (t, q) for the wrist frames' camera pose
        self.want = {"depth": False, "wrist": False}
        self.events = deque(maxlen=12)
        self.n_ev = 0
        self.traj_goal = None
        self.t_healthy = -math.inf
        self.n_task_ev = 0
        self.finder = Finder()               # the AI's finds; its floor map keeps the ids for a pick of one
        self.color = None
        self.color_sub = None                # only while a find waits for its frames
        self.n_found, self.t_find_pub, self.t_found = 0, -math.inf, -math.inf
        cb = ReentrantCallbackGroup()
        self.p_js = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.p_state = self.create_publisher(String, "/arm/state", 10)
        self.p_out = self.create_publisher(Bool, "/arm/out", 10)
        self.p_twist = self.create_publisher(Twist, "/pick/cmd_vel", 10)
        self.p_camreq = self.create_publisher(String, "/arm/camera_request", 10)
        self.p_find = self.create_publisher(Image, "/arm/find_image", 2)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, "/arm/request", self.on_request, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        # camera_info latched (a driver may send it once) or with every frame: either way
        info_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.depth_topic = depth_topic
        self.create_subscription(Image, depth_topic, self.on_depth, 2)
        self.create_subscription(CameraInfo, info_topic, lambda m: setattr(self, "K_depth", self._k(m)), info_qos)
        self.create_subscription(Image, "/arm_cam/image_raw", self.on_wrist, 2)
        self.create_subscription(CameraInfo, "/arm_cam/camera_info", lambda m: setattr(self, "K_wrist", self._k(m)),
                                 info_qos)
        # one cycle at a time (a slow one, a vision step, delays the next; never two at once); the trajectory action's
        # loop runs beside it in its own group
        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(0.2, self.publish_state)
        self.k = 0
        self.T_depth = np.eye(4)
        c = self.cfg.cameras[0]
        self.T_depth[:3, :3], self.T_depth[:3, 3] = np.array(c["R"]), c["pos"]
        self.T_depth = self.T_depth @ MJ_TO_OPTICAL
        try:
            from control_msgs.action import FollowJointTrajectory
            self.FJT = FollowJointTrajectory
            ActionServer(self, FollowJointTrajectory, "/arm/follow_joint_trajectory", self.execute_traj,
                         goal_callback=self.accept_traj, cancel_callback=lambda g: CancelResponse.ACCEPT,
                         callback_group=cb)
        except ImportError:
            self.get_logger().warn("control_msgs not installed: no /arm/follow_joint_trajectory action")
        self.get_logger().info(f"arm on {self.port_spec}; poses stow / ready / high / drop from "
                               f"{config_path(config).name}; depth from {depth_topic} ({c['name']})")

    @staticmethod
    def _k(m: CameraInfo):
        return (m.k[0], m.k[4], m.k[2], m.k[5])

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, text: str):
        self.n_ev += 1
        self.events.append(dict(n=self.n_ev, t=round(self.now(), 2), text=text))
        self.get_logger().info(text)

    # -- the bus ---------------------------------------------------------------------------------------------------
    def connect(self):
        if self.driver is not None or time.monotonic() - self.t_connect < 2.0:
            return
        self.t_connect = time.monotonic()
        try:
            port = open_port(self.port_spec)
        except OSError as e:
            self.get_logger().warn(f"servo bus {self.port_spec}: {e} (retrying)", throttle_duration_sec=10.0)
            return
        d = ArmDriver(Bus(port, timeout=0.02), self.cfg, self.cal)
        missing = d.ping()
        if missing:
            self.get_logger().warn(f"servo bus: no answer from ids {missing} (retrying)")
            port.close()
            return
        d.enable(True)
        st = d.read(self.now())
        self.driver = d
        self.goal = st.q.copy()
        self.source = "hold"
        self.event(f"arm connected on {self.port_spec}: holding where it is")

    # -- the cycle -------------------------------------------------------------------------------------------------
    def tick(self):
        self.connect()
        d = self.driver
        if d is None:
            return
        t = self.now()
        try:
            st = d.read(t)
        except OSError as e:
            self.get_logger().error(f"servo bus lost: {e}")
            self.driver = None
            return
        self.q_hist.append((t, st.q.copy()))
        self.publish_joints(st)
        if st.ok:
            self.t_healthy = t
        if self.tidy_on and t - self.t_healthy > HEALTH_GRACE:
            # its joints are stale or a servo faulted: the task can't trust them. Hold where commanded, stop the base
            self.tidy_on, self.motion, self.source = False, None, "hold"
            self.task.state, self.task.done_reason = "DONE", "servo feedback"
            self.p_twist.publish(Twist())
            self.event(f"tidy stopped: servos {list(st.missing)} silent, {list(st.errors)} reporting errors; holding")
        if self.tidy_on:
            out = self.task.step(t, Inputs(self.pose or (0.0, 0.0, 0.0), st, self.depth, self.wrist))
            self.goal = out.arm_goal
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
                self.tidy_on = False
                self.source = "hold"
                self.event(f"tidy done: {self.task.picked} picked, {self.task.failed} not ({self.task.done_reason})")
        elif self.motion is not None:
            self.goal = self.motion.sample(t)
            if self.motion.done(t):
                self.motion = None
                if self.traj_goal is None:
                    self.source = "hold"
        if self.goal is not None and d.enabled:
            d.send(self.goal)
        self.k += 1
        if self.k % max(1, int(self.rate / 10)) == 0:
            self.p_out.publish(Bool(data=bool(self.kin.tool_out(st.q) or self.kin.tool_out(self.goal))))
        if self.k % max(1, int(self.rate / 20)) == 0:
            self.publish_tf(st.q)

    def q_at(self, t: float):
        best = None
        for tt, q in self.q_hist:
            if tt <= t + 1e-6:
                best = q
        return best if best is not None else (self.q_hist[-1][1] if self.q_hist else np.zeros(6))

    # -- inputs ----------------------------------------------------------------------------------------------------
    def on_odom(self, m: Odometry):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, _yaw(p.orientation))

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

    def on_color(self, m: Image):
        if m.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        self.color = ColorFrame(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                (img[..., ::-1] if m.encoding == "bgr8" else img).copy())

    def _find(self, a: dict):
        """An AI's find: the depth and colour frames are asked for; publish_state answers when they are in."""
        if self.color_sub is None:
            self.color_sub = self.create_subscription(Image, self.color_topic, self.on_color, 2)
        why = self.finder.request(a, self.now())
        if why:
            self.event(f"find: {why}")
            return
        self.event(f"find: looking for {a.get('color') or 'anything'} {a.get('shape') or ''}".rstrip())

    def _find_step(self):
        t = self.now()
        if self.finder.due(t):                   # frames from after the robot settled (the sim renders on request)
            for kind in ("depth", "color"):
                self.p_camreq.publish(String(data=kind))
        if self.finder.req is not None and self.finder.step(t, self.depth, self.color, self.pose):
            r = self.finder.result
            self.event(f"find: {r['error']}" if r.get("error") else
                       f"find: {len(r['candidates'])} candidates, {len(r['image_only'])} image-only")
            if self.color_sub is not None:
                self.destroy_subscription(self.color_sub)
                self.color_sub, self.color = None, None
            self.t_found, self.t_find_pub = time.monotonic(), -math.inf
        img = self.finder.image
        now = time.monotonic()
        if img is not None and now - self.t_found < 60.0 and now - self.t_find_pub > 0.5 and \
                (self.p_find.get_subscription_count() > 0 or self.t_find_pub == -math.inf):
            self.t_find_pub = now
            self.p_find.publish(self._rgb_msg(img))

    def _rgb_msg(self, img) -> Image:
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "arm_find"
        m.height, m.width = img.shape[:2]
        m.encoding, m.step = "rgb8", img.shape[1] * 3
        m.data = np.ascontiguousarray(img).tobytes()
        return m

    def on_request(self, m: String):
        try:
            a = json.loads(m.data)
            verb = str(a.get("verb", ""))
        except (ValueError, AttributeError):
            self.get_logger().warn(f"/arm/request: not JSON: {m.data!r}")
            return
        if verb == "find":                   # the cameras alone: no servo bus needed
            self._find(a)
            return
        d = self.driver
        t = self.now()
        if d is None:
            self.event(f"{verb}: the arm isn't connected")
            return
        q = d.state.q.copy()
        if verb in VERBS:
            self.traj_goal = None            # any request takes over from a trajectory (its loop sees it and ends)
        if verb == "tidy":
            if self.tidy_on:
                return
            self.motion = None
            self.task = TidyTask(self.kin)
            self.n_task_ev = 0
            self.task.history.append((t, self.pose or (0.0, 0.0, 0.0)))
            only = a.get("only")
            self.task.start(t, int(a.get("max_objects", 0)) or None, float(a.get("radius", 0.0)) or None,
                            only=None if only is None else int(only),
                            fmap=None if only is None else self.finder.map)
            self.tidy_on, self.source = True, "tidy"
            self.event("tidy: started" if only is None else f"tidy: started, object {only} only")
        elif verb == "stop":
            if self.tidy_on:
                self.task.stop(t)           # the task stows (and finishes a drop in progress), then ends
                self.event("tidy: stopping")
            elif not self._move_to(q, self.task.poses["stow"], "stow"):
                self.motion, self.source = None, "hold"          # no checked way to stow: stop where it is
                self.goal = (self.goal if self.goal is not None else q).copy()
        elif verb == "pose":
            name = str(a.get("name", ""))
            if self.tidy_on or name not in self.task.poses:
                self.event(f"pose {name}: {'the tidy runs' if self.tidy_on else 'unknown'}")
                return
            self._move_to(q, self.task.poses[name], name)
        elif verb == "gripper":
            if self.tidy_on:
                return
            g = (self.goal if self.goal is not None else q).copy()
            g[5] = q_for_gap(self.cfg, float(a.get("gap", 0.0)))
            self.motion = mo.Motion([self.goal if self.goal is not None else q, g], mo.speeds(self.kin), t)
            self.source = "pose"
        elif verb == "jog":
            name = str(a.get("joint", ""))
            if self.tidy_on or name not in self.cfg.joints:
                self.event(f"jog {name}: {'the tidy runs' if self.tidy_on else 'unknown joint'}")
                return
            start = self.goal if self.goal is not None else q
            base = self.motion.q[-1] if self.motion is not None else start      # clicks while one plays add up
            target, why = mo.jog(self.kin, start, self.cfg.joints.index(name), float(a.get("delta", 0.0)), base)
            if target is None:
                self.event(f"jog {name.replace('_', ' ')}: {why}")
                return
            if not d.enabled:
                d.enable(True)
            self.motion = mo.Motion([start, target], mo.speeds(self.kin), t)
            self.source = "pose"
        elif verb == "hold":
            self.tidy_on, self.motion = False, None
            d.enable(True)
            self.goal, self.source = q, "hold"
            self.event("holding")
        elif verb == "relax":
            self.tidy_on, self.motion = False, None
            d.enable(False)
            self.goal, self.source = None, "relaxed"
            self.event("torque off: the arm is limp")
        else:
            self.event(f"/arm/request: unknown verb {verb!r}")

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
        if not self.driver.enabled:
            self.driver.enable(True)
        self.motion = mo.Motion(path, mo.speeds(self.kin), t)
        self.source = "pose"
        self.event(f"moving to {name}")
        return True

    # -- the trajectory action -------------------------------------------------------------------------------------
    def accept_traj(self, goal):
        """Accepted only while nothing else runs it; its points are checked (mo.check_trajectory) when it starts."""
        if self.tidy_on or self.driver is None:
            return GoalResponse.REJECT
        names = list(goal.trajectory.joint_names)
        return GoalResponse.ACCEPT if set(names) <= set(self.cfg.joints) else GoalResponse.REJECT

    def execute_traj(self, gh):
        tr = gh.request.trajectory
        res = self.FJT.Result()
        d = self.driver
        q0 = (self.goal if self.goal is not None else d.state.q).copy() if d is not None else None
        times, wps = (None, "the arm isn't connected") if d is None else mo.check_trajectory(
            self.kin, q0, tr.joint_names,
            [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, list(p.positions)) for p in tr.points])
        if times is None:
            self.event(f"trajectory refused: {wps}")
            gh.abort()
            res.error_code, res.error_string = res.INVALID_GOAL, str(wps)
            return res
        self.traj_goal, self.motion = gh, None
        self.source = "trajectory"
        if not d.enabled:
            d.enable(True)
        t0 = self.now()
        play = mo.Linear([t0 + s for s in times], wps)
        rate = self.create_rate(self.rate)
        while True:
            if gh.is_cancel_requested or self.traj_goal is not gh or self.tidy_on:
                gh.canceled()                # the goal stays where it was sampled last: the arm holds there
                res.error_code = res.INVALID_GOAL
                break
            t = self.now()
            self.goal = play.sample(t)
            if play.done(t):
                gh.succeed()
                res.error_code = res.SUCCESSFUL
                break
            rate.sleep()
        if self.traj_goal is gh:
            self.traj_goal = None
            self.source = "hold"
        return res

    # -- outputs ---------------------------------------------------------------------------------------------------
    def publish_joints(self, st):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(self.cfg.joints)
        m.position = [float(v) for v in st.q]
        m.velocity = [float(v) for v in st.dq]
        m.effort = [float(v) for v in st.load]
        self.p_js.publish(m)

    def publish_tf(self, q):
        fk = self.kin.fk(q)
        stamp = self.get_clock().now().to_msg()
        out = []
        for name in LINKS + ("wrist_cam",):
            T = fk[name] @ MJ_TO_OPTICAL if name == "wrist_cam" else fk[name]
            tr = TransformStamped()
            tr.header.stamp, tr.header.frame_id = stamp, "base_link"
            tr.child_frame_id = {"tool": "arm_tool", "wrist_cam": "arm_wrist_cam_optical"}.get(name, name)
            tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = map(float, T[:3, 3])
            r = tr.transform.rotation
            r.x, r.y, r.z, r.w = map(float, _quat(T[:3, :3]))
            out.append(tr)
        self.tf.sendTransform(out)

    def publish_state(self):
        self._find_step()
        d = self.driver
        st = d.state if d is not None else None
        s = dict(connected=d is not None, torque=bool(d and d.enabled), source=self.source,
                 port=self.port_spec, tidy=self.task.status() if (self.tidy_on or self.task.state != "IDLE") else None,
                 tidy_on=self.tidy_on, events=list(self.events), found=self.finder.result)
        if st is not None:
            s.update(ok=st.ok, missing=list(st.missing), errors=list(st.errors), joints=[round(math.degrees(v), 1) for v in st.q],
                     loads=[round(float(v), 3) for v in st.load], temps=[int(v) for v in st.temp_c],
                     volts=round(float(st.volts), 1), out=bool(self.kin.tool_out(st.q)),
                     names=list(self.cfg.joints), wrist_cam=camera_view(self.kin, st.q))
        self.p_state.publish(String(data=json.dumps(s)))


def main():
    rclpy.init()
    node = ArmNode()
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    prof_out = os.environ.get("OMNI_PROFILE")           # a path: profile the node's spin (one thread) into it
    ex = SingleThreadedExecutor() if prof_out else MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    prof = None
    if prof_out:
        import cProfile
        prof = cProfile.Profile()
        prof.enable()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if prof is not None:
            prof.disable()
            prof.dump_stats(prof_out)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
