"""The arm robot tidying up, in one process (design.md 11, Oct 24): the world with small things on the floor, the robot
with its C++ firmware (sil_robot.SilRobot) and its SO-101 on the servo bus as the servos behave (arm_bus), and the
Jetson's code as arm_node runs it (ros/src/omni_arm: driver, floor and wrist vision, the tidy task; omni_catch's mode
mux in PICK with the arm-out caps; omni_mcu's link):

  physics 0.5 ms -- CAN / sensors --> firmware (C++, 1 kHz) -- USB bytes --> mode mux (PICK) <-- twist --+
       |                                                                                                  |
       +-- servo bus bytes (STS3215 x 6) <---------------------- driver (50 Hz) <-- goals ---- tidy task --+
       +-- renders: the left camera's depth (a stereo matcher's error), the wrist camera's colour -------->+

The cameras (PickCameras): the nose pods' left camera rendered as depth at 640 x 400 with a stereo matcher's error
(disparity noise 0.25 px on the full 1280 x 800 pair, est; 3% of pixels unmatched) from a camera off its calibration
by cam_err_deg (the robot's code assumes the calibrated pose); the wrist camera rendered in colour at 320 x 240 with
2 levels of noise. The servos are off their calibration by joint_err_deg each (est: a hand calibration with LeRobot's
procedure; VERIFY). Frames are rendered when the task asks (sim time: rendering is not real time).

    python sil_pick.py                       4 toys around the robot, verbose
    python sil_pick.py --n 6 --set clutter --seed 3
    python sil_pick.py --view                MuJoCo's window, real time (slower while it renders a look)
"""

from __future__ import annotations

import argparse
import copy
import math
import time

import mujoco
import numpy as np

import arm_objects
from arm_bus import ServoBusSim
from robot_export import arm_params
from robot_model import build_model
from sil_link import SimLink  # noqa: F401  (puts ros/src on the path)
from sil_robot import SilRobot
from omni_arm.driver import ArmDriver  # noqa: E402
from omni_arm.feetech import Bus  # noqa: E402
from omni_arm.floor_vision import DepthFrame  # noqa: E402
from omni_arm.kinematics import ArmKinematics  # noqa: E402
from omni_arm.pick import Inputs, PickParams, TidyTask, wrist_frame  # noqa: E402
from omni_catch.modes import ModeMux  # noqa: E402
from omni_mcu.protocol import State  # noqa: E402

HOME = (1.0, 0.0)


def _quat_mat(q) -> np.ndarray:
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def _mat_quat(R) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(-1))
    return q


class PickCameras:
    """Renders on request: the left nose camera as a stereo matcher's depth, the wrist camera in colour."""

    DEPTH_WH, WRIST_WH = (640, 400), (320, 240)

    def __init__(self, m, d, p, seed: int = 0, cam_err_deg: float = 0.3, disp_sigma: float = 0.25,
                 dropout: float = 0.03):
        self.m, self.d, self.p = m, d, p
        self.rng = np.random.default_rng(seed + 31)
        self.mr = mr = copy.deepcopy(m)
        self.dr = mujoco.MjData(mr)
        self.cl = m.camera("stereo_left").id
        self.cw = m.camera("arm_wrist").id
        # the left camera off its calibration (a small rotation about a random axis)
        ax = self.rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        ang = math.radians(cam_err_deg) * self.rng.uniform(0.5, 1.0)
        Rerr = np.zeros(9)
        mujoco.mju_quat2Mat(Rerr, np.array([math.cos(ang / 2), *(math.sin(ang / 2) * ax)]))
        mr.cam_quat[self.cl] = _mat_quat(_quat_mat(m.cam_quat[self.cl]) @ Rerr.reshape(3, 3))
        W, H = self.DEPTH_WH
        Ww, Hw = self.WRIST_WH
        for wh in (self.DEPTH_WH, self.WRIST_WH):
            mr.vis.global_.offwidth = max(mr.vis.global_.offwidth, wh[0])
            mr.vis.global_.offheight = max(mr.vis.global_.offheight, wh[1])
        self.ren_d = mujoco.Renderer(mr, H, W)
        self.ren_d.enable_depth_rendering()
        self.ren_w = mujoco.Renderer(mr, Hw, Ww)
        fy = (H / 2) / math.tan(math.radians(m.cam_fovy[self.cl]) / 2)
        self.Kd = (fy, fy, W / 2, H / 2)
        fyw = (Hw / 2) / math.tan(math.radians(m.cam_fovy[self.cw]) / 2)
        self.Kw = (fyw, fyw, Ww / 2, Hw / 2)
        self.f_match = 2 * fy                           # the matcher runs on the full 1280 x 800 pair
        self.b = p.camera_baseline
        self.disp_sigma, self.dropout = disp_sigma, dropout
        self.n_depth = self.n_wrist = 0
        self.ms: list = []

    def _sync(self):
        self.dr.qpos[:] = self.d.qpos
        mujoco.mj_kinematics(self.mr, self.dr)
        mujoco.mj_camlight(self.mr, self.dr)

    def depth(self) -> np.ndarray:
        w0 = time.perf_counter()
        self._sync()
        self.ren_d.update_scene(self.dr, camera=self.cl)
        z = self.ren_d.render().astype(np.float64)
        fb = self.f_match * self.b
        disp = fb / np.maximum(z, 1e-3) + self.rng.normal(0, self.disp_sigma, z.shape)
        z = np.where(disp > 2.0, fb / np.maximum(disp, 1e-3), np.nan)
        z[self.rng.random(z.shape) < self.dropout] = np.nan
        self.n_depth += 1
        self.ms.append(1000 * (time.perf_counter() - w0))
        return z.astype(np.float32)

    def wrist(self) -> np.ndarray:
        w0 = time.perf_counter()
        self._sync()
        self.ren_w.update_scene(self.dr, camera=self.cw)
        img = self.ren_w.render().astype(np.float32)
        img += self.rng.normal(0, 2.0, img.shape)
        self.n_wrist += 1
        self.ms.append(1000 * (time.perf_counter() - w0))
        return np.clip(img, 0, 255).astype(np.uint8)

    def close(self):
        self.ren_d.close()
        self.ren_w.close()


class SilPick:
    def __init__(self, n: int = 4, objects: str = "toys", seed: int = 0, cam_err_deg: float = 0.3,
                 joint_err_deg: float = 1.0, verbose: bool = False, params: PickParams | None = None,
                 spread: float = 1.2, world: str = "arena", **param_kw):
        self.verbose = verbose
        names = list(arm_objects.SETS[objects]) * 3
        rng = np.random.default_rng(seed)
        names = [names[k] for k in rng.permutation(len(names))[:n]]
        if world == "house":
            # the ROS bridge's arm world (bridge_core.ARM_AREA): the v7 house, the robot at its start facing +x, the
            # things in the hallway and the living room ahead (rugs, a runner, the floors' edges)
            import house
            from bridge_core import ARM_AREA
            from robot_export import ARM
            objs = arm_objects.scatter(names, ARM_AREA[0], ARM_AREA[1], seed=seed, keep_out=(house.START[:2], 0.5))
            self.p = p = house.house_params("v7", **{**ARM, **param_kw}, floor_objects=objs)
            start = house.START
        else:
            # the robot at HOME facing -x (thrower_xy sets its heading); the things scattered ahead and around it
            ahead = (HOME[0] - 0.9, HOME[1])
            objs = arm_objects.scatter(names, ahead, spread, seed=seed, keep_out=(HOME, 0.45))
            # the arena's netting (its ropes 5 cm up look like things on the floor) well outside the tidy's 2 m
            self.p = p = arm_params(home_xy=HOME, thrower_xy=(-2.0, 0.0), area_size=(9.0, 9.0), floor_objects=objs,
                                    **param_kw)
            start = None
        self.m = build_model(p)
        self.d = mujoco.MjData(self.m)
        if start is not None:
            adr = self.m.joint("base").qposadr[0]
            yaw = math.radians(start[2])
            self.d.qpos[adr:adr + 7] = [start[0], start[1], 0.004, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(self.m, self.d)
        self.robot = r = SilRobot(self.m, self.d, p, seed=seed)
        self.pose0 = r.pose().copy()
        self.names = names
        self.link = SimLink(r, seed=seed)
        self.bus_sim = ServoBusSim(r.arm, joint_err_deg=joint_err_deg, seed=seed)
        self.kin = ArmKinematics()
        self.driver = ArmDriver(Bus(self.bus_sim, timeout=0.0, clock=lambda: 0.0), self.kin.cfg)
        self.cams = PickCameras(self.m, self.d, p, seed=seed, cam_err_deg=cam_err_deg)
        self.task = TidyTask(self.kin, params, log=self._log)
        self.mux = ModeMux("PICK")
        self.pose = (0.0, 0.0, 0.0)
        self.out = None
        self.t_cmd = self.t_task = 0.0
        self.depth = self.wrist = None
        self.n_objects = n
        self.wall = 0.0
        self.max_tilt = 0.0

    def _log(self, msg: str):
        if self.verbose:
            print(f"[{self.d.time:7.2f}s] {msg}")

    def step(self):
        d = self.d
        t = d.time
        for msg in self.link.poll(t):
            if isinstance(msg, State):
                self.pose = (msg.x, msg.y, msg.yaw)
        if t >= self.t_task - 1e-9:
            self.t_task = t + 0.02
            st = self.driver.read(t)
            if self.out is not None and self.out.want_depth:
                self.depth = DepthFrame(t, self.cams.depth(), self.cams.Kd, self.task.T_base_cam)
            if self.out is not None and self.out.want_wrist:
                self.wrist = wrist_frame(self.kin, t, self.cams.wrist(), self.cams.Kw, st.q)
            self.out = self.task.step(t, Inputs(self.pose, st, self.depth, self.wrist))
            self.driver.send(self.out.arm_goal)
            self.mux.arm_out = self.out.arm_out
        if t >= self.t_cmd - 1e-9:
            self.t_cmd = t + 0.01
            self.link.send(self.mux.command(self.link.host.sync, pick=self.out.twist if self.out else None), t)
        self.robot.step()
        if int(t * 2000) % 100 == 0:
            qb = d.sensor("base_quat").data
            self.max_tilt = max(self.max_tilt, math.degrees(math.acos(min(1.0, 1 - 2 * (qb[1] ** 2 + qb[2] ** 2)))))

    def odom_xy(self, P) -> np.ndarray:
        """World points in the odom frame (the robot's start)."""
        x0, y0, a = self.pose0
        d = np.asarray(P)[:, :2] - (x0, y0)
        c, s = math.cos(a), math.sin(a)
        return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)

    def in_bin(self) -> list:
        """Each object: in the bin (the truth, robot frame)?"""
        b = self.m.body("base").id
        R, pb = self.d.xmat[b].reshape(3, 3), self.d.xpos[b]
        cx, cy, cz = self.kin.cfg.bin["centre"]
        L, W, H = self.kin.cfg.bin["size"]
        out = []
        for P in arm_objects.positions(self.m, self.d):
            x, y, z = R.T @ (P - pb)
            out.append(bool(abs(x - cx) < L / 2 and abs(y - cy) < W / 2 and cz < z < cz + H + 0.03))
        return out

    def run(self, t_max: float = 400.0, view: bool = False):
        w0 = time.perf_counter()
        viewer = None
        if view:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(self.m, self.d, show_left_ui=False, show_right_ui=False)
        self.task.start(self.d.time)
        if self.verbose:
            od = self.odom_xy(arm_objects.positions(self.m, self.d))
            print("objects (odom, truth): " + ", ".join(f"{n} ({x:.2f}, {y:.2f})" for n, (x, y) in zip(self.names, od)))
        s0, w_view = self.d.time, 0.0
        while self.task.state != "DONE" and self.d.time < t_max:
            self.step()
            if viewer is not None:
                if not viewer.is_running():
                    break
                now = time.perf_counter()
                if now - w_view > 1 / 30:
                    w_view = now
                    viewer.sync()
                ahead = (self.d.time - s0) - (now - w0)
                if ahead > 0.002:
                    time.sleep(ahead)
        self.wall = time.perf_counter() - w0
        if viewer is not None:
            self.cams.close()
            viewer.close()
        return self.summary()

    def summary(self) -> dict:
        ib = self.in_bin()
        return dict(objects=self.n_objects, in_bin=int(sum(ib)), task_picked=self.task.picked, task_failed=self.task.failed,
                    task_skipped=self.task.skipped,
                    reason=self.task.done_reason, t=round(self.d.time, 1), wall=round(self.wall, 1),
                    depth_frames=self.cams.n_depth, wrist_frames=self.cams.n_wrist,
                    render_ms=round(float(np.median(self.cams.ms)), 1) if self.cams.ms else None,
                    max_tilt_deg=round(self.max_tilt, 2), bus_errors=self.bus_sim.n_errors, events=self.task.events)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--set", default="toys", choices=list(arm_objects.SETS))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--t-max", type=float, default=400.0)
    ap.add_argument("--world", default="arena", choices=("arena", "house"))
    a = ap.parse_args()
    g = SilPick(a.n, a.set, a.seed, verbose=True, world=a.world)
    s = g.run(a.t_max, a.view)
    print({k: v for k, v in s.items() if k != "events"})


if __name__ == "__main__":
    main()
