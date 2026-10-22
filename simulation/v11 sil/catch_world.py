"""v11 (design.md 30): the world around the robot in the catch game, with nothing of the robot's software in it.

  Thrower      shallow_sweep.HumanToss's throws, drawn from the same random streams (same seed, same throws as the
               sim's own catch game): landing points scattered around the waiting spot (7% of the distance along,
               5% across), apex height, spin
  CameraSim    the stereo cameras' images, per pixel (vision.VisionPerception's image formation, without its
               detector and filter, which now run on the Jetson side): motion blur over the exposure, the color
               threshold's edge noise, occlusion by the robot, the thrower and the funnel net, the calibration
               error of each camera, the real ball's size, sync error between the pair, dropped and clutter blobs.
               Each frame leaves as the ball's pixel list (a SparseFrame), delivered after the capture latency
  scoring      the truth: where the ball crossed the funnel, caught or not (the catch beam), return-throw error

The robot is sil_robot.SilRobot (the C++ firmware in the loop). The Jetson side (perception, behavior, the MCU link)
runs in sil_game.py (in-process) or as ROS nodes (the bridge).
"""

from __future__ import annotations

import math
import sys
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "studies"))

from ball_flight import BallFlight  # noqa: E402
from robot_export import v11_params  # noqa: E402
from robot_model import build_model  # noqa: E402
from sil_robot import SilRobot  # noqa: E402
from vision import VisionConfig, _rotvec, sphere_image  # noqa: E402

HOME = (3.5, 0.0)                       # shallow_sweep.HOME / AREA: the 6-7 m catch game's field
AREA = (12.0, 6.0)
SIG_RANGE, SIG_LAT = 0.07, 0.05


def expected_cameras() -> VisionConfig:
    """camera_estimate.EXPECTED: the OV9782 pair on the Argus ISP, hardware sync, a printed bracket (design.md 21.4)."""
    return replace(VisionConfig(), latency_mean=0.045, latency_jitter=0.010, calib_rot_deg=0.10, sync_error=0.0001,
                   edge_sigma=0.6, exposure=0.003, drop_prob=0.05, false_prob=0.01, ball_size_rel=0.015,
                   odom_drift=0.02, yaw_drift_deg=0.6)


@dataclass
class WorldConfig:
    dist: float = 6.5                   # thrower's hand to the waiting spot, m
    apex_range: tuple = (1.0, 1.0)
    toss_spin: float = 6.0              # rad/s 1-sigma per axis (camera_estimate.SPIN)
    release_height: float = 1.0
    return_height: float = 1.2
    wait_between: float = 1.2
    first_toss: float = 1.0
    return_throws: bool = False
    vision: VisionConfig = field(default_factory=expected_cameras)
    processing: float = 0.005           # s of latency_mean spent on the Jetson (detector + filter, est); the rest is
                                        # capture -> memory, which the sim adds before delivering the frame
    seed: int = 0
    floor: dict = field(default_factory=dict)   # SimParams floor kwargs (home_floor.GAME_FLOORS)
    scene: str = "arena"                # "arena" or "house" (house_scene.py: decoration only, the same physics)
    frames: str = "sparse"              # "sparse" (CameraSim's pixel lists) or "rendered" (rendered_camera.py)
    sensor: object = None               # rendered_camera.SensorConfig (None: its defaults)
    hold_time: float = 1.0              # house: the thrower shows the next ball in their hand this long before a throw


@dataclass
class SimFrame:
    """One camera frame as it leaves the sim: the ball's pixels (flat indices) and clutter."""
    cam: int
    t_cap: float
    width: int
    height: int
    idx: np.ndarray
    clutter: np.ndarray
    seed: int
    truth: tuple | None = None          # for scoring only (CameraSim.truth); the robot never sees it


class CameraSim:
    def __init__(self, m, d, p, cfg: VisionConfig, rng: np.random.Generator):
        self.m, self.d, self.p, self.cfg, self.rng = m, d, p, cfg, rng
        self.cams = [m.camera(f"stereo_{s}").id for s in ("left", "right")]
        self.ball_body = m.body("ball").id
        self.base_body = m.body("base").id
        self.W, self.H = cfg.width, cfg.height
        fovy = math.radians(cfg.fovy_deg if cfg.fovy_deg is not None else p.camera_fovy_deg)
        self.f = (self.H / 2) / math.tan(fovy / 2)
        self.cx, self.cy = self.W / 2, self.H / 2
        self.g_opaque = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)
        self.g_net = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)
        # fixed per run: how far each real camera is from its calibration, and the real ball's size
        sd = math.radians(cfg.calib_rot_deg)
        self.cal_R = [_rotvec(rng.normal(0, sd, 3)) for _ in self.cams]
        self.cal_t = [rng.normal(0, cfg.calib_trans, 3) for _ in self.cams]
        self.cal_f = [1 + rng.normal(0, cfg.calib_f_rel) for _ in self.cams]
        self.R_ball = p.ball_spec.radius * (1 + rng.normal(0, cfg.ball_size_rel))
        self.n_frames = 0
        self.g_view = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)

    def _real_cam(self, k: int):
        """Camera k as it really is (off its calibration): position, rotation, focal length (px)."""
        d, cid = self.d, self.cams[k]
        Rc = d.cam_xmat[cid].reshape(3, 3) @ self.cal_R[k]
        return d.cam_xpos[cid] + Rc @ self.cal_t[k], Rc, self.f * self.cal_f[k]

    def truth(self, k: int):
        """For scoring: (u, v, radius px, visible) of the real ball in the real camera k at mid-exposure, or None if
        it is behind the camera. visible: all of it inside the image and nothing on the ray to its centre."""
        d = self.d
        c, Rc, f = self._real_cam(k)
        dt0 = self.cfg.sync_error if k == 1 else 0.0
        bpos = d.xpos[self.ball_body] + d.sensor("ball_vel").data * dt0
        img = sphere_image(Rc.T @ (bpos - c), self.R_ball, f)
        if img is None:
            return None
        u, v, r = self.cx + img[0], self.cy + img[1], img[2]
        vis = r <= u <= self.W - r and r <= v <= self.H - r
        if vis:
            ray = bpos - c
            dist = float(np.linalg.norm(ray))
            gid = np.array([-1], dtype=np.int32)
            hit = mujoco.mj_ray(self.m, d, c, ray / dist, self.g_view, 1, self.ball_body, gid)
            vis = not (gid[0] >= 0 and 0 <= hit < dist - self.R_ball)
        return (float(u), float(v), float(r), bool(vis))

    def render(self, k: int, t_frame: float) -> np.ndarray:
        """Flat indices of the pixels the color threshold calls ball in camera k's frame (empty if none)."""
        cfg, d, m = self.cfg, self.d, self.m
        cid = self.cams[k]
        Rc = d.cam_xmat[cid].reshape(3, 3) @ self.cal_R[k]
        c = d.cam_xpos[cid] + Rc @ self.cal_t[k]
        f = self.f * self.cal_f[k]
        bpos = d.xpos[self.ball_body].copy()
        bvel = d.sensor("ball_vel").data.copy()
        w_base = d.sensor("base_angvel").data.copy()
        v_cam = d.sensor("base_linvel").data.copy() + np.cross(w_base, c - d.xpos[self.base_body])
        s0 = Rc.T @ (bpos - c)
        v_rel = Rc.T @ (bvel - v_cam) - np.cross(Rc.T @ w_base, s0)
        dt0 = cfg.sync_error if k == 1 else 0.0
        R = self.R_ball
        empty = np.zeros(0, dtype=np.int64)
        span = cfg.exposure / 2 + abs(dt0)
        pts = []
        for tt in (-span, span):
            img = sphere_image(s0 + v_rel * (tt + dt0), R, f)
            if img is not None:
                pts.append((self.cx + img[0], self.cy + img[1], img[2]))
        if not pts:
            return empty
        r_px = max(q[2] for q in pts) * 1.3 + 3
        u0 = int(max(0, math.floor(min(q[0] for q in pts) - r_px)))
        u1 = int(min(self.W, math.ceil(max(q[0] for q in pts) + r_px)))
        v0 = int(max(0, math.floor(min(q[1] for q in pts) - r_px)))
        v1 = int(min(self.H, math.ceil(max(q[1] for q in pts) + r_px)))
        if u1 <= u0 or v1 <= v0:
            return empty
        uu, vv = np.meshgrid(np.arange(u0, u1) + 0.5, np.arange(v0, v1) + 0.5)
        dirs = np.stack([(uu - self.cx) / f, -(vv - self.cy) / f, -np.ones_like(uu)], axis=-1)
        dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
        K = max(1, cfg.blur_samples)
        noise = self.rng.normal(0, cfg.edge_sigma, uu.shape) if cfg.edge_sigma > 0 else 0.0
        cover = np.zeros(uu.shape)
        for j in range(K):
            te = cfg.exposure * ((j + 0.5) / K - 0.5) + dt0
            sc = s0 + v_rel * te
            along = dirs @ sc
            perp = np.sqrt(np.maximum(sc @ sc - along ** 2, 0.0))
            edge = (perp - R) / np.maximum(along, 1e-3) * f
            cover += (edge + noise < 0) & (along > 0)
        mask = cover >= 0.5 * K
        if not mask.any():
            return empty
        # occlusion: rays from the camera through the ball's pixels (subsampled for big blobs)
        iy, ix = np.nonzero(mask)
        n = len(iy)
        stride = max(1, int(math.ceil(math.sqrt(n / 300))))
        key = (iy // stride) * 100000 + (ix // stride)
        uniq, inv = np.unique(key, return_inverse=True)
        rep = np.zeros(len(uniq), dtype=int)
        rep[inv] = np.arange(n)
        dw = (Rc @ dirs[iy[rep], ix[rep]].T).T
        t_ball = np.linalg.norm(s0) - R
        vis = np.ones(len(uniq), dtype=bool)
        for groups, transmit in ((self.g_opaque, 0.0), (self.g_net, cfg.net_transmission)):
            gid = np.full(len(uniq), -1, dtype=np.int32)
            dist = np.full(len(uniq), -1.0)
            mujoco.mj_multiRay(m, d, c, dw.reshape(-1).copy(), groups, 1, self.ball_body, gid, dist, None,
                               len(uniq), 50.0)
            hit = (gid >= 0) & (dist < t_ball)
            if transmit > 0:
                hit &= self.rng.uniform(size=len(uniq)) > transmit
            vis &= ~hit
        keep = vis[inv]
        return ((iy[keep] + v0) * self.W + (ix[keep] + u0)).astype(np.int64)

    def clutter(self) -> np.ndarray:
        """A ball-colored blob somewhere in the frame (a yellow toy, a shirt): vision.VisionConfig.false_prob."""
        u, v, r = self.rng.uniform(0, self.W), self.rng.uniform(0, self.H), self.rng.uniform(3, 25)
        uu, vv = np.meshgrid(np.arange(int(u - r), int(u + r) + 1), np.arange(int(v - r), int(v + r) + 1))
        ok = ((uu + 0.5 - u) ** 2 + (vv + 0.5 - v) ** 2 < r * r) & (uu >= 0) & (uu < self.W) & (vv >= 0) & (vv < self.H)
        return (vv[ok] * self.W + uu[ok]).astype(np.int64)

    def frame_pair(self, t: float):
        cfg = self.cfg
        out = []
        self.n_frames += 1
        for k in range(2):
            idx = np.zeros(0, dtype=np.int64)
            if self.rng.uniform() >= cfg.drop_prob:             # a missed detection (lighting, blur): no ball pixels
                idx = self.render(k, t)
            cl = self.clutter() if cfg.false_prob > 0 and self.rng.uniform() < cfg.false_prob else \
                np.zeros(0, dtype=np.int64)
            out.append(SimFrame(k, t, self.W, self.H, idx, cl, int(self.rng.integers(1 << 30)), self.truth(k)))
        return out


class Thrower:
    """HumanToss (shallow_sweep.py): the same random streams, so the same seed gives the same throws."""

    def __init__(self, p, cfg: WorldConfig, flight: BallFlight, z_aim: float):
        self.p, self.cfg, self.flight, self.z_aim = p, cfg, flight, z_aim
        self.rng = np.random.default_rng(cfg.seed)
        self.spin_rng = np.random.default_rng(cfg.seed + 99)
        self.home = np.array(p.home_xy)
        self.hand = np.array([p.thrower_xy[0] + 0.3, p.thrower_xy[1], cfg.release_height])

    def toss(self):
        """(position, velocity, spin, aim point, apex) of the next throw."""
        spin = self.spin_rng.normal(0, self.cfg.toss_spin, 3) if self.cfg.toss_spin > 0 else np.zeros(3)
        e = self.rng.normal(0, 1, 2) * np.array([SIG_RANGE, SIG_LAT]) * self.cfg.dist
        u = self.home - self.hand[:2]
        u /= np.linalg.norm(u)
        aim = self.home + e[0] * u + e[1] * np.array([-u[1], u[0]])
        apex = self.rng.uniform(*self.cfg.apex_range)
        v0 = self.flight.solve_toss(self.hand, [*aim, self.z_aim], apex)
        return self.hand.copy(), v0, spin, aim, apex


class CatchWorld:
    def __init__(self, cfg: WorldConfig | None = None, robot_kw: dict | None = None, **param_kw):
        self.cfg = c = cfg or WorldConfig()
        self.p = p = v11_params(home_xy=HOME, thrower_xy=(HOME[0] - c.dist - 0.3, 0.0), area_size=AREA,
                                scene=c.scene, **c.floor, **param_kw)
        self.m = build_model(p)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        self.robot = SilRobot(self.m, self.d, p, seed=c.seed, **(robot_kw or {}))
        b = p.ball_spec
        self.flight = BallFlight(b.mass, b.radius, b.drag_cd, p.air_density, p.air_viscosity)
        if c.frames == "rendered":
            from rendered_camera import RenderedCameraSim
            self.cams = RenderedCameraSim(self.m, self.d, p, c.vision, np.random.default_rng(c.seed + 7), c.sensor,
                                          seed=c.seed)
        else:
            self.cams = CameraSim(self.m, self.d, p, c.vision, np.random.default_rng(c.seed + 7))
        bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "held_ball")
        self.held_mocap = self.m.body_mocapid[bid] if bid >= 0 else -1
        self.lat_rng = np.random.default_rng(c.seed + 11)
        self.thrower = Thrower(p, c, self.flight, self.robot.g.rim_center_robot(0.0)[1])
        j = self.m.joint("ball")
        self.bq, self.bv = j.qposadr[0], j.dofadr[0]
        self.frames: deque = deque()          # (t due, [SimFrame, SimFrame])
        self.t_frame = 0.0
        self.results: list = []
        self.cur = None
        self.phase, self.t_next = "wait", c.first_toss
        self.pose0 = self.robot.pose().copy()  # the odom frame's origin, in the world

    # -- frames -------------------------------------------------------------------------------------------------
    def odom_from_world(self, xy):
        x0, y0, a = self.pose0
        dx, dy = xy[0] - x0, xy[1] - y0
        return np.array([math.cos(a) * dx + math.sin(a) * dy, -math.sin(a) * dx + math.cos(a) * dy])

    def pop_frames(self, t: float) -> list:
        out = []
        while self.frames and self.frames[0][0] <= t:
            out.append(self.frames.popleft()[1])
        return out

    # -- the step -------------------------------------------------------------------------------------------------
    def step(self):
        d, c = self.d, self.cfg
        self.robot.step()
        t = d.time
        if t >= self.t_frame - 1e-9:
            self.t_frame = t + 1.0 / c.vision.fps
            pair = self.cams.frame_pair(t)
            lat = max(0.005, self.lat_rng.normal(c.vision.latency_mean - c.processing, c.vision.latency_jitter))
            due = max(t + lat, self.frames[-1][0] if self.frames else 0.0)       # the pipeline keeps order
            self.frames.append((due, pair))
        self._game(t)

    def ball(self):
        return self.d.sensor("ball_pos").data.copy(), self.d.sensor("ball_vel").data.copy()

    def place_ball(self, pos, vel=(0, 0, 0), spin=(0, 0, 0)):
        self.d.qpos[self.bq:self.bq + 7] = [*pos, 1, 0, 0, 0]
        self.d.qvel[self.bv:self.bv + 6] = [*vel, *spin]

    def _hold(self, shown: bool):
        """House: the next ball in the thrower's hand (decoration; the thrown ball replaces it at release)."""
        if self.held_mocap >= 0:
            h = self.thrower.hand
            self.d.mocap_pos[self.held_mocap] = (h[0], h[1], h[2] if shown else -1.0)

    def _game(self, t: float):
        r, c = self.robot, self.cfg
        bp, bv = self.ball()
        if self.phase == "wait":
            self._hold(t >= self.t_next - c.hold_time)
            if t >= self.t_next:
                self._hold(False)
                pos, v0, spin, aim, apex = self.thrower.toss()
                self.place_ball(pos, v0, spin)
                self.cur = dict(t_release=t, apex=apex, zone_dist=float(np.linalg.norm(aim - np.array(HOME))),
                                release_speed=float(np.linalg.norm(v0)), caught=False, pos_error=math.nan,
                                arrival_deg=math.nan, catch_speed=math.nan, flight_time=math.nan, launched=False,
                                return_error=math.nan, t_held=None)
                self.phase = "flight"
            return
        cur = self.cur
        if self.phase == "flight":
            fc = r.funnel_center_world()
            if math.isnan(cur["pos_error"]) and bv[2] < 0 and bp[2] < fc[2]:
                cur["pos_error"] = float(np.hypot(*(bp[:2] - fc[:2])))
                cur["arrival_deg"] = math.degrees(math.atan2(math.hypot(bv[0], bv[1]), -bv[2]))
                cur["catch_speed"] = float(np.linalg.norm(self.d.sensor("base_linvel").data[:2]))
                cur["flight_time"] = t - cur["t_release"]
                cur["t_in"] = t
            held = r.ball_held()
            cur["t_held"] = (cur["t_held"] or t) if held else None
            t_in = cur.get("t_in", math.inf)
            if held and t - cur["t_held"] >= 0.5:
                self._resolve(t, True)
            elif not held and bp[2] < 0.15 and t - t_in > 0.3:
                self._resolve(t, False)
            elif t - cur["t_release"] > 5.0:
                self._resolve(t, r.ball_held())
        elif self.phase == "held":             # return throws: wait for the robot to launch it
            mod = self.d.xpos[r.module]
            if not cur["launched"] and np.linalg.norm(bp - mod) > 0.4 and not r.ball_held():
                cur["launched"] = True
                cur["t_launch"] = t
                cur["launch_speed"] = float(np.linalg.norm(bv))
            if cur["launched"] and bv[2] < 0 and bp[2] < c.return_height and math.isnan(cur["return_error"]):
                cur["return_error"] = float(np.hypot(*(bp[:2] - self.thrower.hand[:2])))
                self.phase, self.t_next = "wait", t + c.wait_between
            elif t - self.t_held_since > 8.0:
                self.phase, self.t_next = "wait", t + c.wait_between

    def _resolve(self, t: float, caught: bool):
        cur = self.cur
        cur["caught"] = bool(caught)
        cur["t_result"] = t
        self.results.append(cur)
        if caught and self.cfg.return_throws:
            self.phase, self.t_held_since = "held", t
        else:
            self.phase, self.t_next = "wait", t + self.cfg.wait_between

    def close(self):
        self.robot.close()
        if hasattr(self.cams, "close"):
            self.cams.close()
