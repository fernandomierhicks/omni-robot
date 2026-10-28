"""Interactive 3D viewer for the v8 CAD (MuJoCo's viewer, Windows), with a slider that tilts the module.

  python view.py            # meshes from out/view_stl (build.py writes them)
  python view.py --meshes   # rebuild the meshes from the CAD first (~30 s)
  python view.py --arm      # the arm robot (the arm kit and the SO-101 stowed; out/view_stl_arm, written on first use;
                            # key 2 toggles the arm)

Mouse: left-drag rotates, right-drag pans, scroll zooms, double-click a part to centre the view on it.
Keys 0-5 toggle groups: 0 base, 1 pods, 2 module, 3 yokes, head and face, 4 fasteners, 5 foam pads, net and ball.
Right panel, Control: "tilt" turns the module about its pivot (radians: -0.52 .. 0.70 is -30 .. +40 deg).
Tab / Shift+Tab hide the side panels.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

import colors
import dims as D

OUT = Path(__file__).with_name("out")
STL = OUT / ("view_stl_arm" if "--arm" in sys.argv else "view_stl")
GROUPS = {"base": 0, "pods": 1, "module": 2, "mounts": 3, "head": 3, "mast": 3, "arm": 2, "fasteners": 4, "soft": 5}


def category(it) -> tuple[str, int]:
    """(body, geom group) of an item: the module's parts ride the tilt joint."""
    body = "module" if it.group == "module" else "fixed"
    if it.kind == "fastener":
        return body, GROUPS["fasteners"]
    if it.kind in ("foam", "fabric", "ball"):
        return body, GROUPS["soft"]
    g = "pods" if it.group.startswith("pod") else it.group
    return body, GROUPS[g]


def export_meshes(robot=None):
    """out/view_stl/<body>__<group>__<colour>__<n>.stl; the module's meshes in its pivot frame (tilt 0)."""
    from collections import defaultdict

    from build123d import Compound, Pos, export_stl
    if robot is None:
        import assembly as A
        robot = A.build(0.0, 0.0, kit="arm" if "--arm" in sys.argv else "catch")
        if "--arm" in sys.argv:                        # the SO-101 stowed, its own STEP parts (build.so101_items)
            import build
            robot.items += build.so101_items()
    STL.mkdir(parents=True, exist_ok=True)
    for f in STL.glob("*.stl"):
        f.unlink()
    buckets = defaultdict(list)
    to_pivot = Pos(-D.PIVOT_R[0], 0, -D.PIVOT_R[1])
    for it in robot.items:
        body, grp = category(it)
        buckets[(body, grp, colors.color_class(it))].append(it.shape.moved(to_pivot) if body == "module" else it.shape)
    n = 0
    for (body, grp, col), shapes in buckets.items():
        for k in range(0, len(shapes), 6):           # MuJoCo takes at most 200 000 faces per mesh
            export_stl(Compound(shapes[k:k + 6]), str(STL / f"{body}__{grp}__{col}__{k // 6}.stl"),
                       tolerance=0.25, angular_tolerance=0.4)
            n += 1
    return n


def build_xml() -> str:
    assets, fixed, module = [], [], []
    for k, f in enumerate(sorted(STL.glob("*.stl"))):
        body, grp, col, _ = f.stem.split("__")
        r, g, b, a = colors.COLORS.get(col, (0.5, 0.5, 0.5, 1))
        assets.append(f'<mesh name="m{k}" file="{f.as_posix()}" scale="0.001 0.001 0.001" inertia="shell"/>')
        geom = f'<geom type="mesh" mesh="m{k}" group="{grp}" rgba="{r} {g} {b} {a}" contype="0" conaffinity="0" mass="0"/>'
        (module if body == "module" else fixed).append(geom)
    lo, hi = D.TILT_RANGE
    px, pz = D.PIVOT_R[0] / 1000, D.PIVOT_R[1] / 1000
    rad = 3.141592653589793 / 180
    return f"""<mujoco model="omni-robot v8 CAD">
  <compiler angle="radian" inertiafromgeom="false"/>
  <option gravity="0 0 0" timestep="0.002" integrator="implicitfast"/>
  <visual><global offwidth="1920" offheight="1080"/><quality shadowsize="4096"/>
    <headlight ambient="0.38 0.38 0.4" diffuse="0.42 0.42 0.42"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.93 0.93 0.94" rgb2="0.88 0.88 0.9" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.98 0.98 1" rgb2="0.85 0.87 0.9" width="64" height="512"/>
    {''.join(assets)}
  </asset>
  <worldbody>
    <light pos="1 -1 3" dir="-0.3 0.3 -1" directional="true" castshadow="true" diffuse="0.55 0.55 0.55"/>
    <light pos="-2 2 2" dir="0.5 -0.5 -0.5" directional="true" castshadow="false" diffuse="0.25 0.25 0.25"/>
    <geom type="plane" size="2 2 0.01" material="grid" contype="0" conaffinity="0"/>
    {''.join(fixed)}
    <body name="module" pos="{px} 0 {pz}">
      <inertial pos="0 0 0" mass="1.6" diaginertia="0.03 0.03 0.03"/>
      <joint name="tilt" type="hinge" axis="0 1 0" range="{lo * rad} {hi * rad}" damping="2" armature="0.01"/>
      {''.join(module)}
    </body>
  </worldbody>
  <actuator>
    <position name="tilt" joint="tilt" kp="40" ctrlrange="{lo * rad} {hi * rad}"/>
  </actuator>
</mujoco>"""


def main():
    if "--meshes" in sys.argv or not any(STL.glob("*.stl")):
        print("writing meshes ...", flush=True)
        print(export_meshes(), "meshes")
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as v:
        with v.lock():
            v.opt.geomgroup[:] = 1                    # all six groups on (MuJoCo shows 0-2 by default)
            v.cam.lookat[:] = (0.0, 0.0, 0.2)
            v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.3, 145.0, -22.0
        while v.is_running():
            t = time.time()
            with v.lock():
                mujoco.mj_step(model, data, nstep=8)
            v.sync()
            time.sleep(max(0.0, 1 / 60 - (time.time() - t)))


if __name__ == "__main__":
    main()
