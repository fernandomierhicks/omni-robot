"""v11 (design.md 31): the stereo cameras as images. MuJoCo renders each camera, a sensor model turns the render into
what the OV9782 + ISP would deliver, and the frame leaves as a full 1280 x 800 RGB image (omni_catch's ArrayFrame on
the robot side) instead of catch_world.CameraSim's pixel list.

Same per-run camera as CameraSim (it draws the same random numbers, so a seed gives the same cameras either way):
  calibration error  the render camera is moved off its calibrated pose (rotation, position) and its focal length
                     scaled, as CameraSim does to its rays; the robot's code still assumes the calibrated camera
  ball size          the rendered ball is the real ball's size (1.5% spread), the filter assumes the nominal one
  sync               the right camera's exposure starts sync_error later
What's new, per frame (SensorConfig, all est unless marked):
  motion blur        the exposure (3 ms) is the average of K renders spread over it, the whole scene moved to each
                     sub-time (mj_integratePos: the ball, the robot and so the cameras, the flywheels). K follows the
                     image motion during the exposure (one render per px, at most 6); K = 1 when nothing moves
  radiometry         the render is taken as the scene's display-referred colour: linearised (gamma 2.2), times the
                     relative illumination left after the ISP's lens-shading correction (vignetting, 20% at the
                     corners), as electrons (1500 e- at full scale: the analogue gain a 3 ms indoor exposure needs)
  noise              shot noise (Poisson, as Gaussian) and read noise (5 e-), then 10-bit quantisation (RAW10)
  ISP                gamma 2.2 back to 8-bit RGB. No demosaicing (each pixel has all three colours), no denoising,
                     no sharpening, no lens distortion (the real pipeline rectifies before the detector)
Not modelled: rolling shutter (the OV9782 is global shutter), auto exposure (the lights are set so the render is
well exposed), flicker, lens flare, bloom, Bayer colour artefacts, a lens PSF beyond MuJoCo's 8x multisampling.

Not real time: each frame pair costs tens of ms of render and numpy (render_ms), the game runs on sim time.
"""

from __future__ import annotations

import copy
import math
import time
from dataclasses import dataclass

import mujoco
import numpy as np

from catch_world import CameraSim


@dataclass
class SensorConfig:
    """The OV9782 pair through the Jetson's ISP, as images. The datasheet gives 1280 x 800, global shutter, RAW10 and
    120 fps at full resolution (OV9782 product brief, VERIFY against the datasheet); the rest is (est)."""
    full_scale_e: float = 1500.0       # electrons at the top code, at the analogue gain used indoors (est)
    read_noise_e: float = 5.0          # electrons rms (est)
    adc_bits: int = 10                 # RAW10
    gamma: float = 2.2                 # the ISP's output curve, and the render's assumed encoding (est)
    vignette: float = 0.2              # relative illumination lost at the corners after lens-shading correction (est)
    exposure_gain: float = 1.0         # scales the render's radiance (1 = the scene as lit)
    blur_px: float = 1.0               # one sub-exposure render per this much image motion during the exposure
    max_sub: int = 6
    noise: bool = True


@dataclass
class ImageFrame:
    """One rendered camera frame as it leaves the sim: the image and, for scoring only, where the ball really is."""
    cam: int
    t_cap: float
    img: np.ndarray                    # H x W x 3 uint8 RGB
    truth: tuple | None                # (u, v, radius px, visible) of the real ball in the real camera, or None


def _quat_to_mat(q) -> np.ndarray:
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def _mat_to_quat(R) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(-1))
    return q


class RenderedCameraSim(CameraSim):
    """frame_pair(t) -> [ImageFrame, ImageFrame]; the rest as CameraSim."""

    def __init__(self, m, d, p, cfg, rng: np.random.Generator, sensor: SensorConfig | None = None, seed: int = 0):
        super().__init__(m, d, p, cfg, rng)              # the per-run calibration error and ball size, as CameraSim
        self.sensor = s = sensor or SensorConfig()
        self.noise_rng = np.random.default_rng(seed + 13)
        W, H = self.W, self.H
        if m.vis.global_.offwidth < W or m.vis.global_.offheight < H:
            m.vis.global_.offwidth, m.vis.global_.offheight = max(W, m.vis.global_.offwidth), max(H, m.vis.global_.offheight)
        # the render model: the real cameras (off their calibration) and the real ball
        self.mr = mr = copy.deepcopy(m)
        for k, cid in enumerate(self.cams):
            R_bc = _quat_to_mat(m.cam_quat[cid])
            R_real = R_bc @ self.cal_R[k]
            mr.cam_pos[cid] = m.cam_pos[cid] + R_real @ self.cal_t[k]
            mr.cam_quat[cid] = _mat_to_quat(R_real)
            mr.cam_fovy[cid] = math.degrees(2 * math.atan((H / 2) / (self.f * self.cal_f[k])))
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == self.ball_body:
                mr.geom_size[g, 0] = self.R_ball
        self.dr = mujoco.MjData(mr)
        self.ren = mujoco.Renderer(mr, H, W)
        self.opt = mujoco.MjvOption()
        # sensor tables: render value -> electrons at full illumination; electrons -> code; code -> 8-bit output
        self.lin = (s.full_scale_e * s.exposure_gain * (np.arange(256) / 255.0) ** s.gamma).astype(np.float32)
        top = 2 ** s.adc_bits - 1
        self.code_scale = np.float32(top / s.full_scale_e)
        self.top = top
        self.gam = np.round(255 * (np.arange(top + 1) / top) ** (1 / s.gamma)).astype(np.uint8)
        yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
        r2 = ((xx + 0.5 - W / 2) ** 2 + (yy + 0.5 - H / 2) ** 2) / ((W / 2) ** 2 + (H / 2) ** 2)
        self.vig = (1 - s.vignette * r2)[..., None].astype(np.float32)
        n = H * W * 3
        self.bank = self.noise_rng.standard_normal(n + (1 << 20), dtype=np.float32)
        self.n_px = n
        self.rn2 = np.float32(s.read_noise_e ** 2)
        self.buf = np.empty((H, W, 3), np.float32)
        self.qtmp = np.zeros(m.nq)
        # bookkeeping
        self.ms_render: list = []          # wall ms per camera frame: renders + sensor model
        self.n_sub: list = []              # sub-exposure renders per camera frame

    # -- the image -----------------------------------------------------------------------------------------------------
    def _n_sub(self, k: int) -> int:
        """Sub-exposure renders for camera k: one per blur_px of image motion during the exposure."""
        s, cfg, d = self.sensor, self.cfg, self.d
        c, Rc, f = self._real_cam(k)
        w = d.sensor("base_angvel").data
        v_cam = d.sensor("base_linvel").data + np.cross(w, c - d.xpos[self.base_body])
        move = f * float(np.linalg.norm(w)) * cfg.exposure + f * float(np.linalg.norm(v_cam)) * cfg.exposure / 3.0
        s0 = Rc.T @ (d.xpos[self.ball_body] - c)
        depth = -s0[2]
        if depth > 0.1:
            v_rel = Rc.T @ (d.sensor("ball_vel").data - v_cam) - np.cross(Rc.T @ w, s0)
            lat = np.linalg.norm(v_rel - (v_rel @ s0) / (s0 @ s0) * s0)
            move = max(move, f * float(lat) * cfg.exposure / depth)
        return int(min(s.max_sub, max(1, math.ceil(move / s.blur_px))))

    def _render(self, k: int, taus) -> np.ndarray:
        """Electrons per pixel (before vignetting and noise), the mean over sub-exposures at times taus (s from the
        frame time)."""
        m, d, mr, dr = self.m, self.d, self.mr, self.dr
        cid = self.cams[k]
        dr.mocap_pos[:] = d.mocap_pos
        dr.mocap_quat[:] = d.mocap_quat
        acc = self.buf
        for j, tau in enumerate(taus):
            q = self.qtmp
            q[:] = d.qpos
            if tau != 0.0:
                mujoco.mj_integratePos(m, q, d.qvel, tau)
            dr.qpos[:] = q
            mujoco.mj_kinematics(mr, dr)
            mujoco.mj_camlight(mr, dr)
            self.ren.update_scene(dr, camera=cid, scene_option=self.opt)
            img = self.ren.render()
            if j == 0:
                np.take(self.lin, img, out=acc)
            else:
                acc += np.take(self.lin, img)
        if len(taus) > 1:
            acc *= np.float32(1.0 / len(taus))
        return acc

    def _sensor(self, e: np.ndarray) -> np.ndarray:
        """Electrons -> vignetting, shot and read noise, 10-bit code, gamma -> 8-bit RGB (in place on e)."""
        e *= self.vig
        if self.sensor.noise:
            o = int(self.noise_rng.integers(0, 1 << 20))
            sd = e + self.rn2
            np.sqrt(sd, out=sd)
            sd *= self.bank[o:o + self.n_px].reshape(e.shape)
            e += sd
        e *= self.code_scale
        e += np.float32(0.5)
        np.clip(e, 0, self.top, out=e)
        return np.take(self.gam, e.astype(np.uint16))

    def image(self, k: int) -> np.ndarray:
        cfg = self.cfg
        K = self._n_sub(k)
        dt0 = cfg.sync_error if k == 1 else 0.0
        taus = [cfg.exposure * ((j + 0.5) / K - 0.5) + dt0 for j in range(K)]
        w0 = time.perf_counter()
        img = self._sensor(self._render(k, taus))
        self.ms_render.append(1000 * (time.perf_counter() - w0))
        self.n_sub.append(K)
        return img

    def frame_pair(self, t: float):
        self.n_frames += 1
        return [ImageFrame(k, t, self.image(k), self.truth(k)) for k in range(2)]

    def close(self):
        if self.ren is not None:
            self.ren.close()
            self.ren = None
