"""A generic IMX662 USB 2 camera (Arducam-sized) on the arm robot's nose, under the lidar, tilted up: the forward-looking camera for
tags, people and the room (Nov 5 2026, study; cad/v13/tools/imx662_fit.py checks it fits). Not in robot_export / the bridge: the
camera is added to the model's XML here, at run time.

The sensor (Sony IMX662, STARVIS 2, 1/2.8", 1920 x 1080, 2.9 um, ROLLING shutter; the module and its UVC bridge are generic, VERIFY on
the one bought):
  lens           a 2.8 mm M12 lens, 89.6 x 58.4 deg, as a pinhole: the robot's code would rectify it first (real wide M12 lenses
                 barrel-distort strongly; not modelled)
  rolling shutter  each row is exposed over its own window [s_r - exposure, s_r], s_r = t0 + readout * r / H: the scene is rendered
                 at a grid of times dt apart and each row averages the renders inside its window (motion blur and the shutter's skew
                 together, exactly, to dt). readout (est): the time from the first row to the last; over USB 2 the bridge runs the
                 sensor at 30 fps, so between ~16 ms (a 60 fps sensor mode) and 33 ms (the whole frame time); 0 = a global shutter
  pose           the scene moves as the robot does: pose_fn(t) sets the robot's pose at time t (kinematic, mj_kinematics only), its
                 pitch/roll wobble included if the caller gives one
  radiometry     as rendered_camera: the render as display-referred colour, linearised (gamma 2.2), vignetting (30% at the
                 corners, a wide lens after shading correction, est), electrons (full scale 4000 e- at the gain used, est), shot
                 and read noise (1.5 e-, STARVIS 2 class, est), 12-bit, gamma back to 8 bit
  MJPEG          USB 2 cannot carry 1080p30 raw (~1 Gb/s YUY2): the bridge sends MJPEG; the frame is JPEG-encoded at quality
                 `jpeg_q` (est) and decoded again, so the detectors see its blocks
Not modelled: near-infrared (the IMX662 sees 850 nm well; with no IR-cut filter and an IR lamp it works in the dark, but the
renders are visible light), auto exposure, flicker (50/60 Hz lights banding a rolling shutter), lens flare, Bayer artefacts, the
USB latency (the frame's age when it arrives: ~1 frame + transfer, est).
"""

from __future__ import annotations

import io
import math
import re
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
CODES = HERE / "studies" / "data" / "tag36h11_codes.npz"      # cv2.aruco's 36h11 table, 8 x 8 cells with the black border


@dataclass
class IMX662Config:
    width: int = 1920
    height: int = 1080
    fovy_deg: float = 58.4                 # 2.8 mm lens on 1080 x 2.9 um (hfov 89.6)
    fps: float = 30.0                      # MJPEG over USB 2 (est for the module)
    readout_s: float = 0.016               # first row -> last row (est; 0.033 the whole frame time, 0 a global shutter)
    exposure_s: float = 0.004              # (est: a bright room; ~10-16 ms in dim light)
    dt_s: float = 0.0005                   # render grid: one render every dt across readout + exposure
    full_scale_e: float = 4000.0
    read_noise_e: float = 1.5
    adc_bits: int = 12
    gamma: float = 2.2
    vignette: float = 0.3
    exposure_gain: float = 1.0
    jpeg_q: int = 85                       # the bridge's MJPEG quality (est)
    noise: bool = True

    def fx(self) -> float:
        return self.height / 2 / math.tan(math.radians(self.fovy_deg) / 2)


@dataclass
class Mount:
    """The camera on the robot: its entrance pupil (robot frame, m) and the view's tilt above the horizontal (deg)."""
    pupil: tuple = (0.210, 0.0, 0.104)
    tilt_up_deg: float = 20.0
    body: tuple = (0.022, 0.038, 0.038)    # the module behind the pupil (depth along the view, width, height): drawn, no collision

    def axes(self):
        t = math.radians(self.tilt_up_deg)
        fwd = np.array([math.cos(t), 0.0, math.sin(t)])
        right = np.array([0.0, -1.0, 0.0])
        up = np.cross(right, fwd)
        return fwd, right, up


def _f(*v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def with_camera(xml: str, mount: Mount, name: str = "imx662") -> str:
    """xml with the IMX662 (a MuJoCo camera and its body) on the robot's base, beside the D435."""
    fwd, right, up = mount.axes()
    c = np.asarray(mount.pupil) - fwd * (0.004 + mount.body[0] / 2)        # the body sits behind the pupil, clear of the view
    t = math.radians(mount.tilt_up_deg)
    q = (math.cos(-t / 2), 0.0, math.sin(-t / 2), 0.0)                     # about +y by -tilt: x forward-up
    s = (f'<camera name="{name}" pos="{_f(*mount.pupil)}" xyaxes="{_f(*right, *up)}" fovy="58.4"/>'
         f'<geom name="{name}_body" class="decor" type="box" size="{_f(*(np.asarray(mount.body) / 2))}" pos="{_f(*c)}" '
         f'quat="{_f(*q)}" material="camera_body"/>')
    k = xml.find('<geom name="d435"')
    if k < 0:
        raise ValueError("no D435 in the model to put the IMX662 beside (arm_camera d435)")
    return xml[:k] + s + xml[k:]


def real_tags(m) -> list:
    """Paint the house's placeholder tag textures (house_props: 'not a decodable 36h11 code') with the real 36h11 code of each id, in
    the model only (the png files are untouched). Returns the ids painted."""
    codes = np.load(CODES)["codes"]
    out = []
    for i in range(m.ntex):
        nm = m.texture(i).name
        hit = re.fullmatch(r"prop_tag_(\d+)", nm or "")
        if not hit:
            continue
        tid = int(hit.group(1))
        w, h, ch = m.tex_width[i], m.tex_height[i], m.tex_nchannel[i]
        img = m.tex_data[m.tex_adr[i]:m.tex_adr[i] + w * h * ch].reshape(h, w, ch)
        c = w // 10                                                     # house_props._tag: the black square is 0.8 of the width
        cell = np.kron(np.where(codes[tid % len(codes)], 255, 0).astype(np.uint8), np.ones((c, c), np.uint8))
        img[c:c + 8 * c, c:c + 8 * c, :] = cell[:, :, None]
        out.append(tid)
    return out


class RollingShutterCamera:
    """frame(pose_fn, t0) -> (rgb uint8 after MJPEG, info); pose_fn(t) puts the robot (d.qpos) where it is at time t."""

    def __init__(self, m, d, cfg: IMX662Config | None = None, cam: str = "imx662", seed: int = 0):
        self.m, self.d, self.cfg = m, d, cfg or IMX662Config()
        c = self.cfg
        self.cam = m.camera(cam).id
        m.vis.global_.offwidth, m.vis.global_.offheight = max(c.width, m.vis.global_.offwidth), max(c.height, m.vis.global_.offheight)
        self.ren = mujoco.Renderer(m, c.height, c.width)
        self.rng = np.random.default_rng(seed + 662)
        self.lin = (c.full_scale_e * c.exposure_gain * (np.arange(256) / 255.0) ** c.gamma).astype(np.float32)
        self.top = 2 ** c.adc_bits - 1
        self.gam = np.round(255 * (np.arange(self.top + 1) / self.top) ** (1 / c.gamma)).astype(np.uint8)
        H, W = c.height, c.width
        yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
        r2 = ((xx + 0.5 - W / 2) ** 2 + (yy + 0.5 - H / 2) ** 2) / ((W / 2) ** 2 + (H / 2) ** 2)
        self.vig = (1 - c.vignette * r2)[..., None].astype(np.float32)
        self.n_renders: list = []

    def row_end(self, t0: float) -> np.ndarray:
        c = self.cfg
        return t0 + c.readout_s * (np.arange(c.height) + 0.5) / c.height

    def frame(self, pose_fn, t0: float):
        """The frame whose first row ends its exposure at t0 (the frame's time stamp is the middle row's: t0 + readout / 2)."""
        c, m, d = self.cfg, self.m, self.d
        H = c.height
        s = self.row_end(t0)
        lo, hi = s[0] - c.exposure_s, s[-1]
        n = max(1, int(math.ceil((hi - lo) / c.dt_s)))
        taus = lo + (np.arange(n) + 0.5) * (hi - lo) / n
        acc = np.zeros((H, c.width, 3), np.float32)
        cnt = np.zeros(H, np.float32)
        for tau in taus:
            rows = (s - c.exposure_s <= tau) & (tau < s)
            if not rows.any():
                continue
            pose_fn(tau)
            mujoco.mj_kinematics(m, d)
            mujoco.mj_camlight(m, d)
            self.ren.update_scene(d, camera=self.cam)
            img = self.ren.render()
            acc[rows] += np.take(self.lin, img[rows])
            cnt[rows] += 1
        empty = cnt == 0                                                  # exposure shorter than dt: the nearest render
        if empty.any():
            for r in np.flatnonzero(empty):
                tau = taus[int(np.argmin(np.abs(taus - (s[r] - c.exposure_s / 2))))]
                pose_fn(tau)
                mujoco.mj_kinematics(m, d)
                self.ren.update_scene(d, camera=self.cam)
                acc[r] = np.take(self.lin, self.ren.render()[r])
                cnt[r] = 1
        acc /= cnt[:, None, None]
        self.n_renders.append(n)
        return self._sensor(acc), dict(t0=t0, t_mid=t0 + c.readout_s / 2, renders=n)

    def _sensor(self, e: np.ndarray) -> np.ndarray:
        from PIL import Image
        c = self.cfg
        e *= self.vig
        if c.noise:
            e += np.sqrt(e + c.read_noise_e ** 2) * self.rng.standard_normal(e.shape, dtype=np.float32)
        e *= np.float32(self.top / c.full_scale_e)
        np.clip(e + 0.5, 0, self.top, out=e)
        rgb = np.take(self.gam, e.astype(np.uint16))
        if c.jpeg_q:
            buf = io.BytesIO()
            Image.fromarray(rgb).save(buf, format="JPEG", quality=c.jpeg_q)
            self.last_jpeg = buf.getvalue()
            rgb = np.asarray(Image.open(io.BytesIO(self.last_jpeg)).convert("RGB"))
        return rgb

    def close(self):
        if self.ren is not None:
            self.ren.close()
            self.ren = None


def camera_pose(m, d, cam: str = "imx662"):
    """(position, R optical: x right, y down, z forward) of the camera in the world, as d stands."""
    c = m.camera(cam).id
    return d.cam_xpos[c].copy(), d.cam_xmat[c].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
