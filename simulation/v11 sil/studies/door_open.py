"""Opening a floor-level kitchen cabinet door (Nov 21; design.md 4.5): the arm holds a low bar pull, the base pulls.

The world: cabinet.py's base cabinet (a 457 x 762 mm full-overlay door on self-closing concealed hinges, a vertical bar
pull with its middle 0.38 m up), its face 0.9 m ahead of the robot. The robot: arm_params with the detailed arm
(arm_detailed.py: motors, gearboxes with friction and backlash, flex, heat; servo_emulator.py's STS3215s on the bus),
the robot's C++ arm driver (omni_cpp ArmDriverCore through arm_sil) with that arm's calibrated load model
(studies/results/arm_load_seed<N>.json, from arm_load_cal.py), the firmware SIL driving the AK40-10s, the mode mux in
PICK with the arm-out caps, the MCU link. The robot's code: omni_arm.door.DoorTask (the arm grips and holds, the base
drives the grip round the hinge, turning with the door, steered by the arm's give and slowed by the driver's force).

The robot finds the door by its AprilTag (Nov 21; the default): cabinet.py's 36h11 tag (id 7, 48 mm) at the door's
bottom edge under the pull, painted with its real code; TagEye renders the D435's left infrared at the robot's
480 x 270 stream (0.3 deg off its calibration) and the C apriltag library finds it, the depth (depth_noise.d435's
realistic noise) gives the face beside it; DoorTask's FIND and LOOK make the door from the door map's entry
(cabinet.tag_record, off by a ruler's 2 mm, the tag stuck on ~1 deg turned). Each look is scored against the truth.
The robot starts at home (its centre 0.88 m from the door's face, facing it) or off it (STARTS: turned away, it
must search). --given: the door told to the robot instead, off by a per-seed error (est: the hinge 10 mm per axis,
the heading 1 deg, the pull's height 5 mm, 1 sigma): the runs before the tag (results/door_open_given.*). The touch
at the grasp corrects the door's position, not its heading.

Each run reports: the outcome, the door's final angle (truth) 2 s after the robot let go, the time, the peak pull on
the hand (truth: the pull's contact forces on the arm, along the fingers) and what the driver's load model read, the
arm's peak give, the grip's slip along the fingers, any touch of the robot on the door or the cabinet, the servos'
temperatures, the door's angle the robot believed against the truth.

    python studies/door_open.py                       # the sweep (jobs(): 24 runs, ~10 min on 4 processes)
    python studies/door_open.py --given               # ... the door given, no tag (door_open_given.md)
    python studies/door_open.py --one --start 0.1 0.15 180   # starting turned away: it searches for the tag
    python studies/door_open.py --one --locked        # a child-locked door: the robot must let go
    python studies/door_open.py --one --seed 0        # one run, verbose
    python studies/door_open.py --one --record door.mp4   # ... and a video (door_view camera)
    python studies/door_open.py --one --view          # MuJoCo's window, real time
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from sil_link import ROS_SRC, SimLink  # noqa: E402  (puts ros/src on the path)

sys.path.insert(0, str(ROS_SRC / "omni_cpp" / "sil"))

import cabinet  # noqa: E402
from arm_sil import CppArmDriver  # noqa: E402
from robot_export import arm_params  # noqa: E402
from robot_model import build_xml  # noqa: E402
from servo_emulator import ServoBus  # noqa: E402
from sil_robot import SilRobot  # noqa: E402
from omni_arm.door import Door, DoorInputs, DoorParams, DoorTask  # noqa: E402
from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path  # noqa: E402
from omni_catch.modes import ModeMux  # noqa: E402
from omni_mcu.protocol import State  # noqa: E402

for _pkg in ("omni_calib",):
    sys.path.insert(0, str(ROS_SRC / _pkg))
from omni_arm.door import DoorCamera, DoorTag  # noqa: E402
from omni_calib.detect import apriltags  # noqa: E402

OUT = Path(__file__).resolve().parent / "results"
CYCLE = 0.02                       # s: the arm driver and the task, 50 Hz
IMU_ERR_DEG = 0.15
ARM_BODIES = ("arm_shoulder", "arm_upper_arm", "arm_lower_arm", "arm_wrist", "arm_gripper", "arm_jaw")
DOOR_ERR = dict(xy=0.010, heading_deg=1.0, z=0.005)     # 1 sigma (est): the door given without its tag
TAG_ERR = dict(roll_deg=1.0, ruler=0.002)                # 1 sigma (est): the tag stuck on turned; the door map's
                                                         # hinge distance and pull height measured with a ruler
MEANT = ("door_arm@PROBE", "door_arm@SEAT")              # the probe's fingertip on the door's face, till the seat
                                                         # backs it off
# the door's tag for each finder (cabinet.DoorSpec fields): the D435 sees only the door's bottom edge; the forward
# camera sees the whole door from afar, so its tag can be bigger and beside the pull (its print 0.065-0.165 m in from
# the free edge, clear of the bar's posts). Its centre 0.30 m up: at the pull's height (0.38) it was at the top edge
# of the camera's view from the standoff (46 of 49 deg up) and the close look lost it (2 of 24 runs, Nov 21)
FINDERS = dict(d435=dict(tag_id=7, tag_size=0.048, tag_z=0.142, tag_u=None),
               imx662=dict(tag_id=7, tag_size=0.080, tag_z=0.30, tag_u=0.457 - 0.115))


class TagEye:
    """The D435 as the door finder sees it, through sil_pick's PickCameras (the depth camera off its calibration by
    0.3 deg, depth_noise.d435's realistic depth): its left infrared imager (the depth's frame) at the stream the robot
    runs (480 x 270: the infrared comes at the depth's size), grey, 2 levels of noise, searched by the C apriltag
    library at decimate 2 (tag_detector's), 10 Hz; the depth of the same frame when asked. Infrared without the
    projector's dots (VERIFY: the robot turns the emitter off for tag frames, or the dots are faint enough at
    0.3-0.8 m; the depth needs it on)."""

    def __init__(self, m, d, p, seed: int, rate: float = 10.0):
        from sil_pick import PickCameras
        self.cams = PickCameras(m, d, p, seed=seed)
        self.period, self.t_next = 1.0 / rate, 0.0
        self.n_frames = self.n_dets = 0

    def due(self, t: float) -> bool:
        return t >= self.t_next - 1e-9

    def frame(self, t: float, depth: bool = False):
        """([(id, corners, margin)], depth image or None) of the frame at t."""
        self.t_next = t + self.period
        gray = self.cams.color().mean(axis=2).astype(np.uint8)
        dets = apriltags(gray, decimate=2.0)
        self.n_frames += 1
        self.n_dets += len(dets)
        return dets, (self.cams.depth() if depth else None)

    def close(self):
        self.cams.close()


class FrontEye(TagEye):
    """The forward camera finds the tags (design.md 4.12's IMX662 proposal: a generic Arducam-class USB module on the
    nose under the lidar, 20 deg up, 89.6 x 58.4 deg, 1920 x 1080 MJPEG): rendered still (no rolling shutter: the
    looks are made still; the search turns at 0.4 rad/s), grey, 2 levels of noise, JPEG at quality 85, 0.3 deg off
    its calibration, the C apriltag library at decimate 2; the D435's depth (PickCameras) for the face."""

    def __init__(self, m, d, p, seed: int, rate: float = 10.0, wh=(1920, 1080), cam_err_deg: float = 0.3):
        import copy
        super().__init__(m, d, p, seed, rate)
        self.d = d
        self.mr = copy.deepcopy(m)
        self.dr = mujoco.MjData(self.mr)
        self.W, self.H = wh
        self.cid = self.mr.camera("imx662").id
        rng = np.random.default_rng(seed + 931)
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        ang = math.radians(cam_err_deg) * rng.uniform(0.5, 1.0)
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, self.mr.cam_quat[self.cid].copy(),
                           np.array([math.cos(ang / 2), *(math.sin(ang / 2) * ax)]))
        self.mr.cam_quat[self.cid] = q
        self.mr.vis.global_.offwidth = max(self.mr.vis.global_.offwidth, self.W)
        self.mr.vis.global_.offheight = max(self.mr.vis.global_.offheight, self.H)
        self.ren = mujoco.Renderer(self.mr, self.H, self.W)
        self.rng = np.random.default_rng(seed + 932)

    @staticmethod
    def camera(m, wh=(1920, 1080)) -> DoorCamera:
        """The robot's model of it: the mount's pose (imx662_cam.Mount) and the lens as a pinhole (rectified)."""
        import imx662_cam
        mt = imx662_cam.Mount()
        fwd, right, up = mt.axes()
        T = np.eye(4)
        T[:3, :3] = np.column_stack([right, -up, fwd])            # optical: x right, y down, z along the view
        T[:3, 3] = mt.pupil
        W, H = wh
        f = (H / 2) / math.tan(math.radians(float(m.cam_fovy[m.camera("imx662").id])) / 2)
        return DoorCamera(f, f, (W - 1) / 2, (H - 1) / 2, T)

    def frame(self, t: float, depth: bool = False):
        import io
        from PIL import Image
        self.t_next = t + self.period
        self.dr.qpos[:] = self.d.qpos
        mujoco.mj_kinematics(self.mr, self.dr)
        mujoco.mj_camlight(self.mr, self.dr)
        self.ren.update_scene(self.dr, camera=self.cid)
        g = self.ren.render().astype(np.float32).mean(axis=2) + self.rng.normal(0, 2.0, (self.H, self.W))
        buf = io.BytesIO()
        Image.fromarray(np.clip(g, 0, 255).astype(np.uint8)).save(buf, "JPEG", quality=85)
        gray = np.asarray(Image.open(io.BytesIO(buf.getvalue())))
        dets = apriltags(gray, decimate=2.0)
        self.n_frames += 1
        self.n_dets += len(dets)
        return dets, (self.cams.depth() if depth else None)

    def close(self):
        self.ren.close()
        super().close()


class DoorRun:
    def __init__(self, seed: int = 0, torque_scale: float = 1.0, bar_mu: float = 0.7, door_err: bool = True,
                 load: str = "calibrated", params: DoorParams | None = None, verbose: bool = False,
                 door_seed: int | None = None, mirror: bool = False, locked: bool = False, find: bool = True,
                 start: tuple = (0.0, 0.0, 0.0), finder: str = "d435"):
        """seed: the detailed arm's truth (it needs that arm's calibration); door_seed: the door's error draw
        (default seed); mirror: the door hinged on the robot's right (it opens counter-clockwise); find: the robot
        finds the door by its tag (else it is given, off by DOOR_ERR: the runs before the tag); start: the robot's
        start off its home (m toward the door, m to its left, deg turned left; home: 0.88 m from the door's face,
        facing it). finder: the camera that finds the tag: "d435" (the arm robot's only camera: a 48 mm tag at the
        door's bottom edge, seen from ~0.9 m at most, there at the image's top edge) or "imx662" (FINDERS: the forward camera, a proposal; an 80 mm
        tag beside the pull at its height)."""
        self.seed, self.verbose = seed, verbose
        self.door_seed = seed if door_seed is None else door_seed
        self.mirror, self.find, self.start, self.finder = mirror, find, tuple(start), finder
        rng_t = np.random.default_rng(self.door_seed + 2401)
        kw = dict(hinge=(0.10, 0.25), closed_heading=-math.pi / 2, sense=1) if mirror else {}
        kw.update(FINDERS[finder] if find else dict(tag_id=-1))
        roll = float(rng_t.normal() * TAG_ERR["roll_deg"]) if find and door_err else 0.0
        self.spec = s = cabinet.DoorSpec(tau0=2.0 * torque_scale, bar_mu=bar_mu, locked=locked, tag_roll_deg=roll,
                                         **kw)
        p = arm_params(arm_model="detailed", depth_camera="d435", arm_seed=seed)
        if find and finder == "imx662":
            p = p.with_(front_camera="imx662")
        hx, hy = p.home_xy[0] - start[0], p.home_xy[1] - start[1]      # home faces -x: toward the door is -x
        yaw = math.pi + math.radians(start[2])
        self.p = p = p.with_(home_xy=(hx, hy), thrower_xy=(hx + 3 * math.cos(yaw), hy + 3 * math.sin(yaw)))
        self.m = m = mujoco.MjModel.from_xml_string(cabinet.with_cabinet(build_xml(p), s))
        cabinet.paint_tag(m, s)
        self.d = d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        self.r = SilRobot(m, d, p, seed=seed)
        self.hinge = cabinet.Hinge(m, d, s)
        self.bus = ServoBus(self.r.arm.servos)
        self.kin = ArmKinematics(ArmConfig(config_path("d435")))
        lp = OUT / f"arm_load_seed{seed}.json" if load == "calibrated" else ""
        if lp and not Path(lp).exists():
            raise FileNotFoundError(f"{lp}: run studies/arm_load_cal.py first")
        self.drv = CppArmDriver(self.bus, self.kin.cfg.path, load=str(lp))
        miss = self.drv.connect()
        if miss:
            raise RuntimeError(f"servos {miss} didn't answer")
        self.drv.settings()
        self.link = SimLink(self.r, seed=seed)
        ao = self.kin.cfg.arm_out
        self.mux = ModeMux("PICK", arm_out_caps=(ao["speed_cap"], ao["a_cap"]))
        rng = np.random.default_rng(seed + 401)
        ax = rng.normal(size=3)
        rng = np.random.default_rng(self.door_seed + 1401)
        ax /= np.linalg.norm(ax)
        a = math.radians(IMU_ERR_DEG)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        self.R_imu = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
        self.base = m.body("base").id
        self.pose0 = self.r.pose().copy()
        # the door as the robot is told it (odom frame), off by this seed's error
        e = rng.normal(size=4) if door_err else np.zeros(4)
        self.door_err = dict(dx_mm=round(e[0] * DOOR_ERR["xy"] * 1000, 1), dy_mm=round(e[1] * DOOR_ERR["xy"] * 1000, 1),
                             dheading_deg=round(e[2] * DOOR_ERR["heading_deg"], 2),
                             dz_mm=round(e[3] * DOOR_ERR["z"] * 1000, 1))
        hx, hy = self.to_odom(np.array(s.hinge))
        self.door_true = Door((hx, hy), s.closed_heading - self.pose0[2], s.sense, s.handle_r, s.handle_out,
                              s.handle_z, s.bar_d, math.radians(s.open_max_deg), s.handle_off)
        self.eye = None
        self.tag_rec = None
        if find:
            # the door map's entry for its tag, measured with a ruler (TAG_ERR); the camera from the robot's config
            rec = cabinet.tag_record(s)
            if door_err:
                rec["hinge_x"] += float(rng_t.normal() * TAG_ERR["ruler"])
                rec["pull_dy"] += float(rng_t.normal() * TAG_ERR["ruler"])
            self.tag_rec = DoorTag(**rec)
            d435 = DoorCamera.from_config(self.kin.cfg.cameras[0])
            if finder == "imx662":
                self.eye = FrontEye(m, d, p, seed)
                cam = FrontEye.camera(m)
            else:
                self.eye, cam = TagEye(m, d, p, seed), d435
            self.task = DoorTask(self.kin, None, params, log=self.log, tag=self.tag_rec, camera=cam,
                                 depth_camera=d435)
            self.door_err = dict(tag_roll_deg=round(roll, 2), ruler_mm=[round((self.tag_rec.hinge_x -
                                 cabinet.tag_record(s)["hinge_x"]) * 1000, 1), round((self.tag_rec.pull_dy -
                                 cabinet.tag_record(s)["pull_dy"]) * 1000, 1)])
        else:
            door = Door((hx + e[0] * DOOR_ERR["xy"], hy + e[1] * DOOR_ERR["xy"]),
                        s.closed_heading - self.pose0[2] + math.radians(e[2] * DOOR_ERR["heading_deg"]), s.sense,
                        s.handle_r, s.handle_out, s.handle_z + e[3] * DOOR_ERR["z"], s.bar_d,
                        math.radians(s.open_max_deg), s.handle_off)
            self.task = DoorTask(self.kin, door, params, log=self.log)
        self.found = []                 # per look: the door the robot made of its tag against the truth
        self.pose = (0.0, 0.0, 0.0)
        self.wheel_cur = np.zeros(4)
        self.J_base = np.array(self.kin.cfg.base["J"], float)      # wheel rad/s per body (vx, vy, wz)
        self.out = None
        self.hw = self.ld = None
        self.t_arm = self.t_cmd = 0.0
        self.geoms = self._geom_sets()
        self.rec = []                  # telemetry at 50 Hz
        self.touch: dict = {}           # "door_arm@PROBE": contact samples (50 Hz)
        self.touch_by: dict = {}        # ... which of the robot's geoms
        self.touch_max: dict = {}       # ... the largest normal force (N)
        self.slip0 = None
        self.slip = 0.0
        self.t_release = None
        self.wall = 0.0

    # -- frames ------------------------------------------------------------------------------------------------
    def to_odom(self, P) -> np.ndarray:
        x0, y0, a = self.pose0
        dx, dy = P[0] - x0, P[1] - y0
        c, s = math.cos(a), math.sin(a)
        return np.array([c * dx + s * dy, -s * dx + c * dy])

    def base_R(self) -> np.ndarray:
        return self.d.xmat[self.base].reshape(3, 3).copy()

    def gravity(self) -> np.ndarray:
        return self.R_imu @ (self.base_R().T @ np.array([0.0, 0.0, -9.81]))

    def log(self, msg: str):
        if self.verbose:
            print(f"[{self.d.time:7.2f}s] {msg}", flush=True)

    def _geom_sets(self):
        m = self.m
        arm = {m.body(n).id for n in ARM_BODIES}
        robot = set()
        root = self.base
        for b in range(m.nbody):                       # the robot's bodies: the base's subtree
            k = b
            while k > 0 and k != root:
                k = m.body_parentid[k]
            if k == root:
                robot.add(b)
        names = {i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "" for i in range(m.ngeom)}
        pull = {i for i, n in names.items() if n.startswith("pull_")}
        door = {m.geom("door").id}
        cab = {i for i, n in names.items() if n.startswith("cab") or n == "worktop"}
        return dict(arm=arm, robot=robot, pull=pull, door=door, cab=cab)

    # -- truth -------------------------------------------------------------------------------------------------
    def contacts(self, state: str = ""):
        """(force of the pull on the arm, world N), and the robot's touches of the door and the cabinet, counted
        per task stage (the probe's touch of the door's face is meant: door_arm@PROBE)."""
        m, d, G = self.m, self.d, self.geoms
        f6 = np.zeros(6)
        f_pull = np.zeros(3)
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
            mujoco.mj_contactForce(m, d, i, f6)
            fw = c.frame.reshape(3, 3).T @ f6[:3]        # on geom2, from geom1
            if (g1 in G["pull"] and b2 in G["arm"]) or (g2 in G["pull"] and b1 in G["arm"]):
                f_pull += fw if g1 in G["pull"] else -fw
                if state in ("APPROACH", "REACH", "CREEP", "PROBE"):      # bumping the pull, not holding it
                    k = f"pull_arm@{state}"
                    self.touch[k] = self.touch.get(k, 0) + 1
                    gb = g2 if g1 in G["pull"] else g1
                    self.touch_by.setdefault(k, set()).add(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gb) or "")
                    self.touch_max[k] = max(self.touch_max.get(k, 0.0), abs(float(f6[0])))
            else:
                for ga, gb, bb in ((g1, g2, b2), (g2, g1, b1)):
                    if bb in G["robot"] and (ga in G["door"] or ga in G["cab"]):
                        key = ("door" if ga in G["door"] else "cabinet") + ("_arm" if bb in G["arm"] else "_base")
                        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gb) or f"body {bb}"
                        k = f"{key}@{state}"
                        self.touch[k] = self.touch.get(k, 0) + 1
                        self.touch_by.setdefault(k, set()).add(name)
                        self.touch_max[k] = max(self.touch_max.get(k, 0.0), abs(float(f6[0])))
        return f_pull

    def bar_in_tool(self) -> float:
        """The bar's axis along the fingers, measured from the tool point (m)."""
        sid = self.m.site("arm_tool").id
        R = self.d.site_xmat[sid].reshape(3, 3)
        bar = self.spec.bar(self.hinge.angle)
        rel = np.array([bar[0], bar[1], self.spec.handle_z]) - self.d.site_xpos[sid]
        return float(rel @ R[:, 0])

    # -- the loop ----------------------------------------------------------------------------------------------
    def tick(self):
        d = self.d
        t = d.time
        for msg in self.link.poll(t):
            if isinstance(msg, State):
                self.pose = (msg.x, msg.y, msg.yaw)
                self.wheel_cur = np.array(msg.wheel_cur, float)
        if t >= self.t_arm - 1e-9:
            self.t_arm += CYCLE
            self.drv.set_gravity(self.gravity())
            self.hw = hw = self.drv.cycle(t)
            self.ld = ld = self.drv.load_state()
            tags = depth = None
            if self.eye is not None and self.task.wants_camera and self.eye.due(t):
                tags, depth = self.eye.frame(t, depth=self.task.wants_depth)
            inp = DoorInputs(self.pose, np.array(hw.q), np.array(hw.goal), np.array(ld.tool_force), ld.reflex_active,
                             hw.moving, np.array(ld.motor), np.array(ld.gravity), self.wheel_cur.copy(), tags, depth)
            st0 = self.task.state
            n_looks = len(self.task.looks)
            self.out = self.task.step(t, inp)
            if len(self.task.looks) > n_looks:
                self._score_look()
            for cmd, times, wps, linear, guard in self.out.arm:
                if not self.drv.command(cmd, times, wps, linear, guard):
                    self.log(f"arm refused: {self.drv.why()}")
            self.mux.arm_out = hw.tool_out
            self._record(t, st0)
        if t >= self.t_cmd - 1e-9:
            self.t_cmd = t + 0.01
            self.link.send(self.mux.command(self.link.host.sync, pick=self.out.twist if self.out else None), t)
        self.hinge.apply()
        self.r.step()

    def _score_look(self):
        """The door the robot made of its tag against the truth (odom; the odometry's drift since the start counts
        against it, as it would on the robot)."""
        dm, dt = self.task.door, self.door_true
        # the truth in the robot's odom frame now: the odometry's drift moves where the door is in odom
        x, y, yaw = self.r.pose()
        ox, oy, oyaw = self.pose
        c, s_ = math.cos(oyaw - yaw), math.sin(oyaw - yaw)
        Rd = np.array([[c, -s_], [s_, c]])
        def odom_now(P):                    # world -> the robot's odom frame as its odometry has it now
            pw = np.asarray(P, float) - np.array([x, y])
            return np.array([ox, oy]) + Rd @ pw
        hinge_w = np.array(self.spec.hinge)
        h_true = odom_now(hinge_w)
        head_true = self.spec.closed_heading + (oyaw - yaw)
        e_h = np.array(dm.hinge) - h_true
        n_true = np.array([math.cos(head_true + dm.sense * math.pi / 2), math.sin(head_true + dm.sense * math.pi / 2)])
        u_true = np.array([math.cos(head_true), math.sin(head_true)])
        look = dict(self.task.looks[-1])
        look.update(hinge_err_mm=round(float(np.linalg.norm(e_h)) * 1000, 1),
                    along_door_mm=round(float(e_h @ u_true) * 1000, 1),
                    along_normal_mm=round(float(e_h @ n_true) * 1000, 1),
                    heading_err_deg=round(math.degrees(math.remainder(dm.closed_heading - head_true, 2 * math.pi)), 2),
                    handle_z_err_mm=round((dm.handle_z - self.spec.handle_z) * 1000, 1),
                    sense_ok=dm.sense == self.spec.sense, t=round(self.d.time, 2))
        self.found.append(look)
        self.log(f"look: hinge {look['hinge_err_mm']} mm off (along the door {look['along_door_mm']}, normal "
                 f"{look['along_normal_mm']}), heading {look['heading_err_deg']} deg, pull height "
                 f"{look['handle_z_err_mm']} mm")

    def _record(self, t, st0):
        st = self.task.state
        f = self.contacts(st)
        fb = self.base_R().T @ f
        if st == "PULL" and self.slip0 is None:
            self.slip0 = self.bar_in_tool()
        if st == "PULL" and self.slip0 is not None:
            self.slip = max(self.slip, abs(self.bar_in_tool() - self.slip0))
        if st0 == "PULL" and st != "PULL":
            self.t_release = t
        x, y, yaw = self.r.pose()
        self.rec.append(dict(t=round(t, 3), state=st, door_deg=round(math.degrees(self.hinge.angle), 2),
                             est_deg=round(math.degrees(self.task.th_est), 2),
                             tag_deg=(round(math.degrees(self.task.th_tag), 2) if self.task.th_tag is not None
                                      else None),
                             pull_truth_n=round(float(fb[0]), 2), f_truth=[round(float(v), 2) for v in fb],
                             pull_est_n=round(self.task.f_pull, 2), give_mm=self.out.status.get("give_mm"),
                             pull_driver_n=round(max(0.0, float(self.ld.tool_force[0])), 2),
                             pull_arm_n=round(self.task.f_arm, 2), pull_base_n=round(self.task.f_base, 2),
                             base=[round(x, 4), round(y, 4), round(math.degrees(yaw), 2)],
                             temp=max(self.hw.temp_c[:5])))

    def run(self, t_max: float = 90.0, view: bool = False, record: str = "", after: float = 2.0) -> dict:
        w0 = time.perf_counter()
        viewer = writer = ren = None
        if view:
            from mujoco import viewer as mjv
            viewer = mjv.launch_passive(self.m, self.d, show_left_ui=False, show_right_ui=False)
        if record:
            import imageio
            self.m.vis.global_.offwidth, self.m.vis.global_.offheight = 1280, 720
            ren = mujoco.Renderer(self.m, 720, 1280)
            writer = imageio.get_writer(record, fps=30, quality=8, macro_block_size=8)
            cam_main, cam_top = self.m.camera("door_view").id, self.m.camera("door_top").id
        self.r.step()
        self.task.start(self.d.time)
        t_done, t_frame, s0 = None, 0.0, self.d.time
        while self.d.time < t_max:
            self.tick()
            if self.task.state == "DONE" and t_done is None:
                t_done = self.d.time
            if t_done is not None and self.d.time > t_done + after:
                break
            if writer is not None and self.d.time >= t_frame:
                t_frame += 1 / 30
                ren.update_scene(self.d, camera=cam_main)
                big = ren.render()
                ren.update_scene(self.d, camera=cam_top)
                small = ren.render()[::3, ::3]
                big[12:12 + small.shape[0], -12 - small.shape[1]:-12] = small
                writer.append_data(big)
            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
                ahead = (self.d.time - s0) - (time.perf_counter() - w0)
                if ahead > 0.002:
                    time.sleep(ahead)
        if writer is not None:
            writer.close()
        if viewer is not None:
            viewer.close()
        self.wall = time.perf_counter() - w0
        return self.summary(t_done)

    def summary(self, t_done) -> dict:
        R = self.rec
        pull = [r for r in R if r["state"] == "PULL"]
        tk = self.task
        def peak(k):
            return round(max((r[k] for r in pull), default=0.0), 2)
        # the driver's reading against the truth while pulling (the pull's component along the fingers)
        err = [r["pull_est_n"] - max(0.0, r["pull_truth_n"]) for r in pull]
        err_drv = [r["pull_driver_n"] - max(0.0, r["pull_truth_n"]) for r in pull]
        est_err = [r["est_deg"] - r["door_deg"] for r in pull]
        tag_err = [r["tag_deg"] - r["door_deg"] for r in pull if r.get("tag_deg") is not None]
        t_seen = next((e[0] for e in tk.events if e[1] == "LOOK"), None)
        return dict(seed=self.seed, door_seed=self.door_seed, locked=self.spec.locked, find=self.find,
                    finder=self.finder if self.find else None,
                    start=list(self.start), looks=self.found, t_seen=t_seen,
                    tag_frames=(self.eye.n_frames if self.eye else 0),
                    door_tag_err_deg=(round(float(np.sqrt(np.mean(np.square(tag_err)))), 2) if tag_err else None),
                    tag_seen_pulling=(round(len(tag_err) / max(1, len(pull)), 2) if self.find else None),
                    hinge=("right" if self.mirror else "left") + (", locked" if self.spec.locked else ""),
                    tau0=self.spec.tau0, bar_mu=self.spec.bar_mu, door_err=self.door_err,
                    outcome=tk.outcome, final_deg=round(math.degrees(self.hinge.angle), 1),
                    max_deg=round(max((r["door_deg"] for r in R), default=0.0), 1),
                    t_done=round(t_done, 1) if t_done else None,
                    t_pull=round(len(pull) * CYCLE, 1),
                    peak_pull_truth_n=peak("pull_truth_n"), peak_pull_est_n=peak("pull_est_n"),
                    pull_est_err_rms_n=round(float(np.sqrt(np.mean(np.square(err)))), 2) if err else None,
                    peak_pull_driver_n=peak("pull_driver_n"), peak_pull_arm_n=peak("pull_arm_n"),
                    peak_pull_base_n=peak("pull_base_n"),
                    pull_driver_err_rms_n=round(float(np.sqrt(np.mean(np.square(err_drv)))), 2) if err else None,
                    door_est_err_deg=(round(float(np.sqrt(np.mean(np.square(est_err)))), 2) if est_err else None),
                    peak_give_mm=round(tk.peak["give"] * 1000, 1), slip_mm=round(self.slip * 1000, 1),
                    touch=self.touch, touch_max_n={k: round(v, 1) for k, v in self.touch_max.items()},
                    touch_by={k: sorted(v) for k, v in self.touch_by.items()},
                    unmeant_touch=sum(v for k, v in self.touch.items() if k not in MEANT),
                    temps=[int(v) for v in self.hw.temp_c],
                    reflexes=self.ld.reflexes, events=tk.events, wall_s=round(self.wall, 1))


def run_one(kw) -> dict:
    r = DoorRun(**kw)
    s = r.run()
    if r.eye is not None:
        r.eye.close()                    # the worker runs the next job: free its renderers
    s["trace"] = r.rec[::5]
    s["probe_trace"] = r.task.probe_trace[::2]
    return s


# the robot's starts (m toward the door, m to its left, deg turned left of facing it), per door draw: facing it;
# off to the side and turned (the tag at the image's edge or out of it); turned well away; facing the other way
STARTS = ((0.0, 0.0, 0.0), (0.05, 0.25, 35.0), (0.0, -0.2, -100.0), (0.1, 0.15, 180.0))
# the forward camera finds the tag from across a kitchen: 1.4-2.1 m from the door's face, off to the side, turned
STARTS_FAR = ((-1.0, 0.0, 0.0), (-0.8, 0.6, 40.0), (-1.2, -0.4, -110.0), (-0.5, 0.5, 180.0))


def jobs(seeds: int = 3, find: bool = True, finder: str = "d435") -> list:
    """Nominal (2 N m at shut, mu 0.7): each arm x 4 door draws, the last two hinged on the right, each from its own
    start (STARTS; find); then each arm at half and double the hinges' torque, with a slippery pull (mu 0.4), and on
    a locked door, on draw 0 facing the door. find=False: the door given, off by DOOR_ERR (the runs before the tag)."""
    S = STARTS_FAR if finder == "imx662" else STARTS
    st = (lambda k: dict(start=S[k])) if find else (lambda k: {})
    f = dict(find=find, finder=finder)
    out = [dict(seed=s, door_seed=10 * s + k, mirror=k >= 2, **f, **st(k)) for s in range(seeds) for k in range(4)]
    out += [dict(seed=s, door_seed=10 * s, torque_scale=k, **f, **st(0)) for k in (0.5, 2.0) for s in range(seeds)]
    out += [dict(seed=s, door_seed=10 * s, bar_mu=0.4, **f, **st(0)) for s in range(seeds)]
    out += [dict(seed=s, door_seed=10 * s, locked=True, **f, **st(0)) for s in range(seeds)]   # a child lock: let go
    return out


def report(rows) -> str:
    L = ["# Opening a floor-level kitchen cabinet door (studies/door_open.py)", "",
         "The detailed arm (its own truth per seed) with its calibrated load model, the robot's C++ arm driver, the "
         "firmware SIL and omni_arm.door.DoorTask: the arm holds a low bar pull (its middle 0.38 m up), the base "
         "pulls the door round to 90 deg, turning with it. The door is told to the robot off by a per-seed error. "
         "Hinge torque, friction and the pull's grip are est; no hardware.", "",
         "| arm seed | door draw | hinged | hinges N m at shut | pull mu | door error (given: mm, mm, deg; or the tag's) "
         "| outcome | "
         "door after (deg) | pull time (s) | peak pull truth / arm / base / driver's (N) | read / driver's error rms (N) | "
         "peak give (mm) | "
         "slip (mm) | robot touched (besides the probe's touch) | servo max C |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        e = r["door_err"]
        e_txt = (f"tag {e['tag_roll_deg']:+.1f} deg, ruler {e['ruler_mm'][0]:+.0f}/{e['ruler_mm'][1]:+.0f} mm"
                 if "tag_roll_deg" in e else f"{e['dx_mm']:+.0f}, {e['dy_mm']:+.0f}, {e['dheading_deg']:+.1f}")
        tch = r["touch"]
        touched = ", ".join(f"{k} {v / 50:.2f} s, {r['touch_max_n'][k]:.0f} N" for k, v in tch.items()
                            if k not in MEANT) or "no"
        L.append(f"| {r['seed']} | {r['door_seed']} | {r['hinge']} | {r['tau0']:.1f} | {r['bar_mu']:.1f} | "
                 f"{e_txt} | {r['outcome']} | {r['final_deg']:.0f} | {r['t_pull']:.1f} | "
                 f"{r['peak_pull_truth_n']:.1f} / {r['peak_pull_arm_n']:.1f} / {r['peak_pull_base_n']:.1f} / "
                 f"{r['peak_pull_driver_n']:.1f} | "
                 f"{r['pull_est_err_rms_n']} / {r['pull_driver_err_rms_n']} | "
                 f"{r['peak_give_mm']:.0f} | {r['slip_mm']:.1f} | {touched} | {max(r['temps'][:5])} |")
    free = [r for r in rows if not r.get("locked")]
    ok = sum(r["outcome"] == "opened" and r["final_deg"] > 80 for r in free)
    lk = [r for r in rows if r.get("locked")]
    let_go = sum(r["outcome"] == "stalled" for r in lk)
    fr = [r for r in rows if r.get("find")]
    if fr:
        eye = ("The forward IMX662 (design.md 4.12, proposal) at 1920 x 1080, JPEG q85, 0.3 deg off its calibration; "
               "an 80 mm tag beside the pull, 0.30 m up; the face's depth from the D435" if fr[0].get("finder") == "imx662"
               else "The D435's left infrared at 480 x 270 (0.3 deg off its calibration); a 48 mm tag at the door's "
                    "bottom edge; the face's depth from the D435 too")
        L += ["", "## Finding the door by its tag", "",
              f"{eye}. The C apriltag library at decimate 2, "
              "the tag's pose fitted upright (5 unknowns: its normal level, its roll free) on the robot's camera model; 5 sightings averaged, the base "
              "still. The tag stuck on turned ~1 deg, the door map measured with a ruler (2 mm). Errors against the "
              "truth in the robot's odom frame (its drift counts).", "",
              "| arm seed | door draw | hinged | start (m, m, deg) | tag seen (s) | far look: hinge mm (along door / "
              "normal), heading deg, pull height mm | close look: the same | spread close (mm, deg) | the tag's "
              "door angle pulling: seen, rms error (deg) |",
              "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
        for r in fr:
            lk_ = r["looks"]
            def cell(k):
                if len(lk_) <= k:
                    return "-"
                x = lk_[k]
                return (f"{x['hinge_err_mm']:.0f} ({x['along_door_mm']:+.0f} / {x['along_normal_mm']:+.0f}), "
                        f"{x['heading_err_deg']:+.2f}, {x['handle_z_err_mm']:+.0f}")
            sp = f"{lk_[-1]['spread_mm']}, {lk_[-1]['spread_deg']}" if lk_ else "-"
            st = ", ".join(f"{v:g}" for v in r["start"])
            L.append(f"| {r['seed']} | {r['door_seed']} | {r['hinge']} | {st} | {r['t_seen']} | {cell(0)} | {cell(1)} | "
                     f"{sp} | {r['tag_seen_pulling']}, {r['door_tag_err_deg']} |")
    L += ["", f"Opened and stayed open (> 80 deg two seconds after letting go): {ok}/{len(free)}. Locked doors let go "
              f"(stalled, the grip kept): {let_go}/{len(lk)}, peak pull {max((r['peak_pull_truth_n'] for r in lk), default=0):.0f} N.",
          "", "Pull readings: arm = the driver's load model at the friction band's middle through the Jacobian; base = the "
              "drive currents through the wheels' Jacobian (0.48 N m/A, est); driver's = its tool force (0 inside the band); "
              "the task limits on the larger of arm and base.", ""]
    for r in rows:
        L.append(f"- arm {r['seed']}, door {r['door_seed']} ({r['hinge']}), {r['tau0']:.1f} N m, mu {r['bar_mu']}: " +
                 "; ".join(f"{t} {s}{' (' + n + ')' if n else ''}" for t, s, n in r["events"]))
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--one", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--torque", type=float, default=1.0, help="the hinges' closing torque, x nominal (2 N m at shut)")
    ap.add_argument("--mu", type=float, default=0.7, help="the pull's friction against the pads")
    ap.add_argument("--exact", action="store_true", help="the door told to the robot without error")
    ap.add_argument("--door-seed", type=int, default=None, help="the door error's draw (default the seed)")
    ap.add_argument("--mirror", action="store_true", help="the door hinged on the robot's right")
    ap.add_argument("--locked", action="store_true", help="a child lock: the door opens 1 deg")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--record", default="")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--given", action="store_true", help="the door given off by DOOR_ERR, no tag (the Nov 21 runs "
                                                         "before the tag: results/door_open_given.*)")
    ap.add_argument("--start", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("FWD", "LEFT", "DEG"),
                    help="the robot's start off its home: m toward the door, m to its left, deg turned left")
    ap.add_argument("--finder", default="d435", choices=tuple(FINDERS),
                    help="the camera that finds the tag: the D435 (the arm robot's) or the forward IMX662 (proposal: "
                         "results/door_open_front.*)")
    a = ap.parse_args()
    if a.one:
        r = DoorRun(a.seed, a.torque, a.mu, door_err=not a.exact, verbose=True, door_seed=a.door_seed,
                    mirror=a.mirror, locked=a.locked, find=not a.given, start=a.start, finder=a.finder)
        s = r.run(view=a.view, record=a.record)
        print(json.dumps({k: v for k, v in s.items() if k != "events"}, indent=1))
        return
    with ProcessPoolExecutor(max_workers=4) as ex:
        rows = list(ex.map(run_one, jobs(a.seeds, find=not a.given, finder=a.finder)))
    OUT.mkdir(exist_ok=True)
    name = "door_open_given" if a.given else ("door_open_front" if a.finder == "imx662" else "door_open")
    (OUT / f"{name}.json").write_text(json.dumps(rows, indent=1, default=float))
    md = report(rows)
    (OUT / f"{name}.md").write_text(md)
    print(md)


if __name__ == "__main__":
    main()
