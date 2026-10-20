"""Still renders: round 1 (v3a) next to v3b upright, tilted to catch, tilted back to launch.

  python render.py            -> renders/v3_poses.png
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from params import SimParams
from robot import Robot
from robot_model import build_model

OUT = Path(__file__).with_name("renders")


def pose_image(p: SimParams, tilt_deg: float, label: str, w=640, h=720) -> Image.Image:
    m = build_model(p.with_(home_xy=(0.0, 0.0)))
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    bq = m.joint("ball").qposadr[0]
    d.qpos[bq:bq + 3] = [-2.2, 1.7, 0.037]
    mujoco.mj_forward(m, d)
    r = Robot(m, d, p)
    r.tilt_target = math.radians(tilt_deg)
    for _ in range(int(0.8 / m.opt.timestep)):
        r.step()
    rnd = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.02, 0.0, 0.25]
    cam.distance, cam.azimuth, cam.elevation = 1.4, -62, -8
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    rnd.update_scene(d, camera=cam, scene_option=opt)
    img = Image.fromarray(rnd.render())
    draw = ImageDraw.Draw(img, "RGBA")
    try:
        font = ImageFont.truetype("segoeui.ttf", 24)
    except OSError:
        font = ImageFont.load_default()
    draw.rectangle((0, 0, w, 44), fill=(255, 255, 255, 200))
    draw.text((14, 8), label, fill=(20, 20, 20, 255), font=font)
    return img


def main():
    OUT.mkdir(exist_ok=True)
    p = SimParams()
    v3a = SimParams(deck_mode="high", feed="j_bend")
    shots = [(v3a, 0, "round 1 (v3a): rim 494 mm"), (p, 0, "v3b: deck at axle height, kicker"),
             (p, 15, "v3b +15 deg: meeting the ball"), (p, -20, "v3b -20 deg: 50 deg launch")]
    imgs = [pose_image(q, t, lab) for q, t, lab in shots]
    out = Image.new("RGB", (sum(i.width for i in imgs), imgs[0].height), "white")
    x = 0
    for i in imgs:
        out.paste(i, (x, 0))
        x += i.width
    out.save(OUT / "v3_poses.png")
    print(f"wrote {OUT / 'v3_poses.png'}")


if __name__ == "__main__":
    main()
