"""Render the CAD assembly with MuJoCo (meshes from build.py's out/render_stl).

  python render.py          -> out/renders/*.png
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

OUT = Path(__file__).with_name("out")
STL = OUT / "render_stl"

COLORS = {   # (prefix, rgba)
    "printed__bumper": (0.95, 0.45, 0.08, 1),
    "printed__flywheel_tire": (0.06, 0.06, 0.07, 1),
    "printed__flywheel_hub": (0.98, 0.45, 0.08, 1),
    "printed__funnel_rim": (0.98, 0.5, 0.1, 1),
    "printed__channel_half": (0.75, 0.85, 0.95, 0.45),
    "printed__gate_flap": (0.98, 0.5, 0.1, 1),
    "printed__payload_plate": (0.42, 0.44, 0.47, 1),
    "printed__wheel_adapter": (0.98, 0.5, 0.1, 1),
    "printed__": (0.18, 0.19, 0.21, 1),
    "purchased__AK40": (0.72, 0.74, 0.77, 1),
    "purchased__omni_wheel": (0.25, 0.26, 0.28, 1),
    "purchased__battery": (0.12, 0.3, 0.7, 1),
    "purchased__jetson": (0.08, 0.35, 0.18, 1),
    "purchased__lidar": (0.08, 0.08, 0.09, 1),
    "purchased__estop": (0.85, 0.08, 0.06, 1),
    "purchased__mcu": (0.05, 0.32, 0.17, 1),
    "purchased__power": (0.05, 0.32, 0.17, 1),
    "purchased__standoff": (0.8, 0.8, 0.82, 1),
    "purchased__flywheel_motor": (0.1, 0.1, 0.11, 1),
    "purchased__gate_servo": (0.15, 0.2, 0.6, 1),
    "purchased__esc": (0.1, 0.1, 0.1, 1),
    "purchased__strut": (0.05, 0.05, 0.05, 1),
    "purchased__funnel_net": (0.95, 0.95, 0.95, 0.35),
    "purchased__camera": (0.05, 0.05, 0.05, 1),
    "purchased__ball": (0.82, 0.95, 0.12, 1),
}


def color(name: str):
    best = max((k for k in COLORS if name.startswith(k)), key=len)
    return COLORS[best]


def build_xml() -> str:
    assets, geoms = [], []
    for k, f in enumerate(sorted(STL.glob("*.stl"))):
        name = f.stem
        assets.append(f'<mesh name="m{k}" file="{f.as_posix()}" scale="0.001 0.001 0.001" inertia="shell"/>')
        r, g, b, a = color(name)
        geoms.append(f'<geom type="mesh" mesh="m{k}" rgba="{r} {g} {b} {a}" contype="0" conaffinity="0" '
                     f'mass="0.001" group="1"/>')
    return f"""<mujoco>
  <visual><global offwidth="1920" offheight="1440"/><quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.35 0.35 0.37" diffuse="0.4 0.4 0.4"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.93 0.93 0.94" rgb2="0.88 0.88 0.9" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.1"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.98 0.98 1" rgb2="0.85 0.87 0.9" width="64" height="512"/>
    {''.join(assets)}
  </asset>
  <worldbody>
    <light pos="1 -1 3" dir="-0.3 0.3 -1" directional="true" castshadow="true" diffuse="0.6 0.6 0.6"/>
    <light pos="-2 2 2" dir="0.5 -0.5 -0.5" directional="true" castshadow="false" diffuse="0.25 0.25 0.25"/>
    <geom type="plane" size="2 2 0.01" material="grid"/>
    <body name="robot">{''.join(geoms)}</body>
  </worldbody>
</mujoco>"""


def turntable(model, data, path: Path, seconds: float = 12.0, fps: int = 30, width: int = 1280, height: int = 960):
    """One slow orbit of the assembly, rising a little so the funnel interior shows."""
    import imageio
    r = mujoco.Renderer(model, height, width)
    cam = mujoco.MjvCamera()
    writer = imageio.get_writer(str(path), fps=fps, quality=8, macro_block_size=8)
    n = int(seconds * fps)
    for k in range(n):
        f = k / n
        cam.lookat[:] = (0.0, 0.0, 0.27)
        cam.distance = 1.45 - 0.15 * math.sin(2 * math.pi * f)
        cam.azimuth = 145 + 360 * f
        cam.elevation = -18 - 10 * math.sin(math.pi * f)
        r.update_scene(data, camera=cam)
        writer.append_data(r.render())
    writer.close()


def overview(path: Path):
    """2 x 2 labeled sheet from the still renders."""
    from PIL import ImageDraw, ImageFont
    tiles = [("front_left", "Assembly, front-left"), ("catch_module", "Catch module: pocket, trapdoor, J-bend, flywheels"),
             ("launcher_side", "Side: gate servo, J-bend, 52 deg flywheel launcher"), ("top", "Top: funnel, struts, stereo cameras")]
    w, h = 720, 540
    sheet = Image.new("RGB", (2 * w, 2 * h), "white")
    try:
        font = ImageFont.truetype("segoeui.ttf", 22)
    except OSError:
        font = ImageFont.load_default()
    for i, (name, label) in enumerate(tiles):
        img = Image.open(OUT / "renders" / f"{name}.png").resize((w, h))
        d = ImageDraw.Draw(img, "RGBA")
        d.rectangle((0, 0, w, 36), fill=(0, 0, 0, 140))
        d.text((12, 6), label, fill="white", font=font)
        sheet.paste(img, ((i % 2) * w, (i // 2) * h))
    sheet.save(path)


def main():
    import sys
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    (OUT / "renders").mkdir(exist_ok=True)
    if "--turntable" in sys.argv:
        turntable(model, data, OUT / "renders" / "turntable.mp4")
        overview(OUT / "renders" / "overview.png")
        print("wrote turntable.mp4 and overview.png")
        return
    r = mujoco.Renderer(model, 1080, 1440)
    views = {
        "front_left": ((0.0, 0.0, 0.28), 1.35, 145, -20),
        "side": ((0.0, 0.0, 0.3), 1.3, 90, -8),
        "rear_right": ((0.0, 0.0, 0.28), 1.35, -40, -22),
        "top": ((0.0, 0.0, 0.3), 1.2, 90, -88),
        "catch_module": ((0.03, 0.0, 0.33), 0.62, 120, -12),
        "launcher_side": ((0.06, 0.0, 0.3), 0.5, 90, -2),
        "under": ((0.0, 0.0, 0.08), 0.9, 160, 18),
    }
    for name, (lookat, dist, az, el) in views.items():
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        r.update_scene(data, camera=cam)
        Image.fromarray(r.render()).save(OUT / "renders" / f"{name}.png")
        print("wrote", name)


if __name__ == "__main__":
    main()
