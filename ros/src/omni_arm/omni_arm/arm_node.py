"""arm_node: the SO-101 on the robot (design.md 11). One process owns the servo bus and everything that moves the arm.

Every cycle (rate, 50 Hz): read the six servos (one SYNC_READ), publish the joints, then the one source in charge sets
the goals (one SYNC_WRITE): the tidy task, a joint trajectory (the action), a named pose, or holding still.

Subscribes /arm/request             std_msgs/String JSON {"verb": ...} from the console's mission node (or by hand):
                                      tidy {max_objects, radius}   start the tidy task (the mode manager must be in PICK)
                                      stop                          stop the task or motion; the arm stows
                                      pose {name: stow | ready | high | drop}   move there
                                      gripper {gap: m}              open / close to a gap
                                      jog {joint: name, delta: rad} one joint by delta (kept in range, refused if
                                                                    it would hit the robot, the bumper or the floor)
                                      hold | relax                  torque on, hold here | torque off (the arm drops:
                                                                    hold it; the SO-101 has no brakes)
           odom_topic (/odom)       nav_msgs/Odometry: the base's pose for the task (the MCU's odometry)
           /stereo/depth            sensor_msgs/Image 32FC1, the left nose camera's depth (+ /stereo/camera_info)
           /arm_cam/image_raw       sensor_msgs/Image rgb8, the wrist camera (+ /arm_cam/camera_info)
Publishes  /arm/joint_states        sensor_msgs/JointState (rad; effort = load, fraction of the torque limit)
           /arm/state               std_msgs/String JSON at 5 Hz: connected, torque, joints (deg), loads, temperatures,
                                    volts, out, who drives, the task's status and its last events
           /arm/out                 std_msgs/Bool at 10 Hz: the tool is out (the mode manager caps the base)
           /pick/cmd_vel            geometry_msgs/Twist: the task's base command (the mode manager forwards it in PICK)
           /arm/camera_request      std_msgs/String "depth" | "wrist" when the task wants a frame (the sim renders on
                                    request; the robot's cameras stream and ignore it)
           /tf                      base_link -> each arm link (arm_shoulder .. arm_gripper, arm_jaw, arm_tool,
                                    arm_wrist_cam_optical), from the joints
Action     /arm/follow_joint_trajectory   control_msgs/FollowJointTrajectory (MoveIt's or LeRobot's driver can use it)
Parameters port (tcp:127.0.0.1:5763 in the sim, /dev/ttyACM0 or the bus board's device on the robot), calibration
           (a LeRobot calibration json; "" = the sim's nominal one), rate, odom_topic.
"""

from __future__ import annotations

import json
import math
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
from omni_arm.driver import ArmDriver
from omni_arm.feetech import Bus, load_calibration
from omni_arm.floor_vision import DepthFrame
from omni_arm.kinematics import ArmKinematics, q_for_gap
from omni_arm.pick import MJ_TO_OPTICAL, Inputs, TidyTask, wrist_frame
from omni_arm.transport import open_port

LINKS = ("arm_shoulder", "arm_upper_arm", "arm_lower_arm", "arm_wrist", "arm_gripper", "arm_jaw", "tool")


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _quat(R) -> tuple:
    """x y z w of a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node")
        P = self.declare_parameter
        self.port_spec = P("port", "tcp:127.0.0.1:5763").value
        cal = P("calibration", "").value
        self.rate = float(P("rate", 50.0).value)
        odom_topic = P("odom_topic", "/odom").value
        self.kin = ArmKinematics()
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
        self.n_task_ev = 0
        cb = ReentrantCallbackGroup()
        self.p_js = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.p_state = self.create_publisher(String, "/arm/state", 10)
        self.p_out = self.create_publisher(Bool, "/arm/out", 10)
        self.p_twist = self.create_publisher(Twist, "/pick/cmd_vel", 10)
        self.p_camreq = self.create_publisher(String, "/arm/camera_request", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, "/arm/request", self.on_request, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        # camera_info latched (a driver may send it once) or with every frame: either way
        info_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, "/stereo/depth", self.on_depth, 2)
        self.create_subscription(CameraInfo, "/stereo/camera_info", lambda m: setattr(self, "K_depth", self._k(m)),
                                 info_qos)
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
        self.get_logger().info(f"arm on {self.port_spec}; poses stow / ready / high / drop from robot_arm.json")

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
        if self.K_depth is None or m.encoding != "32FC1":
            self.get_logger().warn(f"depth frame dropped: {'no /stereo/camera_info yet' if self.K_depth is None else m.encoding}",
                                   throttle_duration_sec=5.0)
            return
        z = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)
        self.depth = DepthFrame(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, z.copy(), self.K_depth,
                                self.T_depth)

    def on_wrist(self, m: Image):
        if self.K_wrist is None or m.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == "bgr8":
            img = img[..., ::-1]
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.wrist = wrist_frame(self.kin, t, img.copy(), self.K_wrist, self.q_at(t))

    def on_request(self, m: String):
        try:
            a = json.loads(m.data)
            verb = str(a.get("verb", ""))
        except (ValueError, AttributeError):
            self.get_logger().warn(f"/arm/request: not JSON: {m.data!r}")
            return
        d = self.driver
        t = self.now()
        if d is None:
            self.event(f"{verb}: the arm isn't connected")
            return
        q = d.state.q.copy()
        if verb == "tidy":
            if self.tidy_on:
                return
            self.motion = None
            self.task = TidyTask(self.kin)
            self.n_task_ev = 0
            self.task.history.append((t, self.pose or (0.0, 0.0, 0.0)))
            self.task.start(t, int(a.get("max_objects", 0)) or None, float(a.get("radius", 0.0)) or None)
            self.tidy_on, self.source = True, "tidy"
            self.event("tidy: started")
        elif verb == "stop":
            if self.tidy_on:
                self.task.stop(t)           # the task stows (and finishes a drop in progress), then ends
                self.event("tidy: stopping")
            else:
                self._move_to(q, self.task.poses["stow"], "stow")
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
        path = mo.plan(self.kin, q, target, via=[P[n] for n in ("high", "ready", "predrop") if n in P],
                       allow_bin=to_bin)
        if path is None:
            path = [q, P["high"]] + ([P["predrop"]] if to_bin and "predrop" in P else []) + [target]
            self.event(f"{name}: no checked path from here, going through 'high'")
        if not self.driver.enabled:
            self.driver.enable(True)
        self.motion = mo.Motion(path, mo.speeds(self.kin), t)
        self.source = "pose"
        self.event(f"moving to {name}")

    # -- the trajectory action -------------------------------------------------------------------------------------
    def accept_traj(self, goal):
        if self.tidy_on or self.driver is None:
            return GoalResponse.REJECT
        names = list(goal.trajectory.joint_names)
        return GoalResponse.ACCEPT if set(names) <= set(self.cfg.joints) else GoalResponse.REJECT

    def execute_traj(self, gh):
        tr = gh.request.trajectory
        idx = [self.cfg.joints.index(n) for n in tr.joint_names]
        q0 = (self.goal if self.goal is not None else self.driver.state.q).copy()
        t0 = self.now()
        pts = [(0.0, q0.copy())]
        for p in tr.points:
            q = pts[-1][1].copy()
            for k, v in zip(idx, p.positions):
                q[k] = v
            pts.append((p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, np.clip(q, self.kin.lo, self.kin.hi)))
        self.traj_goal = gh
        self.source = "trajectory"
        res = self.FJT.Result()
        rate = self.create_rate(self.rate)
        while True:
            if gh.is_cancel_requested or self.traj_goal is not gh or self.tidy_on:
                gh.canceled()
                res.error_code = res.INVALID_GOAL
                break
            s = self.now() - t0
            if s >= pts[-1][0]:
                self.goal = pts[-1][1]
                gh.succeed()
                res.error_code = res.SUCCESSFUL
                break
            k = max(i for i in range(len(pts)) if pts[i][0] <= s)
            a, b = pts[k], pts[min(k + 1, len(pts) - 1)]
            f = (s - a[0]) / max(1e-6, b[0] - a[0])
            self.goal = a[1] + f * (b[1] - a[1])
            rate.sleep()
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
        d = self.driver
        st = d.state if d is not None else None
        s = dict(connected=d is not None, torque=bool(d and d.enabled), source=self.source,
                 port=self.port_spec, tidy=self.task.status() if (self.tidy_on or self.task.state != "IDLE") else None,
                 tidy_on=self.tidy_on, events=list(self.events))
        if st is not None:
            s.update(ok=st.ok, missing=list(st.missing), joints=[round(math.degrees(v), 1) for v in st.q],
                     loads=[round(float(v), 3) for v in st.load], temps=[int(v) for v in st.temp_c],
                     volts=round(float(st.volts), 1), out=bool(self.kin.tool_out(st.q)),
                     names=list(self.cfg.joints))
        self.p_state.publish(String(data=json.dumps(s)))


def main():
    rclpy.init()
    node = ArmNode()
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
