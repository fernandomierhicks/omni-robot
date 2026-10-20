"""Still renders of the v5 hopper options.

  python render.py            -> renders/v5_hopper.png
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
from PIL import Image, ImageDraw, ImageFont

from params import SimParams

sys.path.insert(0, str(Path(__file__).with_name("studies")))
import hopper_rig  # noqa: E402

OUT = Path(__file__).with_name("renders")


def pose_image(p: SimParams, n_balls: int, tilt_deg: float, label: str, w=640, h=720) -> Image.Image:
    m, d, r, bodies, qadr, vadr = hopper_rig.load(p, n_balls)
    r.tilt_target = math.radians(tilt_deg)
    for _ in range(int(0.8 / m.opt.timestep)):
        r.step()
    rnd = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.02, 0.0, 0.33]
    cam.distance, cam.azimuth, cam.elevation = 1.55, -62, -12
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
    shots = [(SimParams(n_balls=12), 4, 0, "bare funnel: 2 balls arch over the throat"),
             (SimParams(n_balls=12, hopper="tube"), 6, 0, "magazine tube: 6 balls single file"),
             (SimParams(n_balls=12, hopper="tube"), 6, -20, "tube at -20 deg: 50 deg launch")]
    imgs = [pose_image(*s) for s in shots]
    out = Image.new("RGB", (sum(i.width for i in imgs), imgs[0].height), "white")
    x = 0
    for i in imgs:
        out.paste(i, (x, 0))
        x += i.width
    out.save(OUT / "v5_hopper.png")
    print(f"wrote {OUT / 'v5_hopper.png'}")


if __name__ == "__main__":
    main()
