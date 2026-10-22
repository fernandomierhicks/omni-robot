"""Render the v7 CAD assembly with MuJoCo (meshes from build.py's out/render_stl, one per colour class).

  python render.py              -> out/renders/*.png and overview.png (meshes from `build.py --render-only`)
  python render.py --meshes     -> writes the meshes first, then renders
  python render.py --turntable  -> + out/renders/turntable.mp4
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
from PIL import Image, ImageDraw, ImageFont

from colors import COLORS

OUT = Path(__file__).with_name("out")
STL = OUT / "render_stl"


def build_xml(stl: Path = STL) -> str:
    assets, geoms = [], []
    for k, f in enumerate(sorted(stl.glob("*.stl"))):
        r, g, b, a = COLORS.get(f.stem.split("__")[0], (0.5, 0.5, 0.5, 1))
        assets.append(f'<mesh name="m{k}" file="{f.as_posix()}" scale="0.001 0.001 0.001" inertia="shell"/>')
        geoms.append(f'<geom type="mesh" mesh="m{k}" rgba="{r} {g} {b} {a}" contype="0" conaffinity="0" mass="0.001"/>')
    return f"""<mujoco>
  <visual><global offwidth="1920" offheight="1440"/><quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.38 0.38 0.4" diffuse="0.42 0.42 0.42"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.93 0.93 0.94" rgb2="0.88 0.88 0.9" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.08"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.98 0.98 1" rgb2="0.85 0.87 0.9" width="64" height="512"/>
    {''.join(assets)}
  </asset>
  <worldbody>
    <light pos="1 -1 3" dir="-0.3 0.3 -1" directional="true" castshadow="true" diffuse="0.55 0.55 0.55"/>
    <light pos="-2 2 2" dir="0.5 -0.5 -0.5" directional="true" castshadow="false" diffuse="0.25 0.25 0.25"/>
    <geom type="plane" size="2 2 0.01" material="grid"/>
    <body name="robot">{''.join(geoms)}</body>
  </worldbody>
</mujoco>"""


VIEWS = {   # name: (lookat, distance, azimuth, elevation, label)
    "front_left": ((0.0, 0.0, 0.2), 1.25, 145, -22, "Assembly, front-left"),
    "rear_right": ((0.0, 0.0, 0.2), 1.25, -35, -24, "Rear-right: e-stop, yokes, RoboClaw"),
    "side": ((0.0, 0.0, 0.22), 1.15, 90, -6, "Side (left)"),
    "top": ((0.0, 0.0, 0.25), 1.15, 90, -89, "Top"),
    "front": ((0.0, 0.0, 0.2), 1.15, 180, -10, "Front: stereo bar, lidar under the plate"),
    "module": ((0.04, 0.0, 0.3), 0.6, 125, -18, "Catch module: chute, feed wheels, flywheels"),
    "pod": ((0.11, 0.11, 0.07), 0.42, 205, -12, "Wheel pod: in-wheel DM-2325, trailing arm, shock"),
    "under": ((0.0, 0.0, 0.05), 0.95, 160, 22, "Underside: battery tray, skirt"),
}


def overview(path: Path, names):
    w, h = 720, 540
    sheet = Image.new("RGB", (2 * w, 2 * h), "white")
    try:
        font = ImageFont.truetype("segoeui.ttf", 22)
    except OSError:
        font = ImageFont.load_default()
    for i, name in enumerate(names):
        img = Image.open(OUT / "renders" / f"{name}.png").resize((w, h))
        d = ImageDraw.Draw(img, "RGBA")
        d.rectangle((0, 0, w, 36), fill=(0, 0, 0, 140))
        d.text((12, 6), VIEWS[name][4], fill="white", font=font)
        sheet.paste(img, ((i % 2) * w, (i // 2) * h))
    sheet.save(path)


def main():
    if "--meshes" in sys.argv:          # (re)write the meshes first (what `build.py --render-only` does)
        import assembly as A
        import build as B
        robot = A.build(0.0, 0.0)
        B.export_render(robot)
        B.export_render(robot, "render_stl_pod", lambda it: it.group in ("pod0", "pod0u") and not it.name.startswith("wheel_guard"))
        B.export_render(robot, "render_stl_module", lambda it: it.group in ("module", "mounts") and not it.name.startswith("funnel_net"))
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    (OUT / "renders").mkdir(exist_ok=True)
    r = mujoco.Renderer(model, 1080, 1440)
    for name, (lookat, dist, az, el, _) in VIEWS.items():
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        r.update_scene(data, camera=cam)
        Image.fromarray(r.render()).save(OUT / "renders" / f"{name}.png")
        print("wrote", name)
    r.close()
    # cutaways (build.py --render-only writes their meshes)
    for sub, shots in (("render_stl_pod", {"pod_cutaway": ((0.106, 0.106, 0.07), 0.5, 200, -15, "Pod (guard removed): in-wheel DM-2325, arm, shock"),
                                           "pod_inside": ((0.09, 0.09, 0.07), 0.42, 20, -20, "Pod from inside: arm, pivot, shock tower")}),
                       ("render_stl_module", {"module_cutaway": ((0.05, 0.0, 0.3), 0.75, 120, -15, "Module (net hidden): chute, feed wheels, flywheels, yokes"),
                                              "module_side": ((0.05, 0.0, 0.29), 0.7, 90, -4, "Module from the left: chute floor, J, launch run")})):
        if not (OUT / sub).exists():
            continue
        m2 = mujoco.MjModel.from_xml_string(build_xml(OUT / sub))
        d2 = mujoco.MjData(m2)
        mujoco.mj_forward(m2, d2)
        r2 = mujoco.Renderer(m2, 1080, 1440)
        for name, (lookat, dist, az, el, label) in shots.items():
            VIEWS[name] = (lookat, dist, az, el, label)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = lookat
            cam.distance, cam.azimuth, cam.elevation = dist, az, el
            r2.update_scene(d2, camera=cam)
            Image.fromarray(r2.render()).save(OUT / "renders" / f"{name}.png")
            print("wrote", name)
        r2.close()
    overview(OUT / "renders" / "overview.png", ["front_left", "module_cutaway", "pod_cutaway", "rear_right"])
    if "--turntable" in sys.argv:
        import imageio
        rr = mujoco.Renderer(model, 960, 1280)
        cam = mujoco.MjvCamera()
        writer = imageio.get_writer(str(OUT / "renders" / "turntable.mp4"), fps=30, quality=8, macro_block_size=8)
        n = 360
        for k in range(n):
            f = k / n
            cam.lookat[:] = (0.0, 0.0, 0.2)
            cam.distance = 1.3 - 0.12 * math.sin(2 * math.pi * f)
            cam.azimuth = 145 + 360 * f
            cam.elevation = -18 - 10 * math.sin(math.pi * f)
            rr.update_scene(data, camera=cam)
            writer.append_data(rr.render())
        writer.close()
        print("wrote turntable.mp4")


if __name__ == "__main__":
    main()
