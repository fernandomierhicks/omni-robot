"""The v11 robot with an SO-101 arm instead of the catch module (params.payload = "arm", design.md 11): watch it.

  python arm_view.py [--mount centre|front] [--payload 0.25] [--base sim|cad]
      MuJoCo's window at real time. Keys: 1-6 the study's poses (stow, carry, level, floor, pick, up), space drives
      2 m/s ahead and stops with the firmware's ramp (again: back), arrows pan the arm (left / right), R resets.
  python arm_view.py --png      renders/v11_arm_poses.png: every pose, both mounts

The poses come from studies/results/arm_study.json (python studies/arm_study.py R); without it, from a coarse sweep.
Meshes: python arm_so101.py fetch (else the arm draws as capsules).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "studies"))

import arm_so101 as A  # noqa: E402
import arm_study as S  # noqa: E402

ORDER = ("stow", "carry", "level", "floor", "pick", "up")


def poses(mount: str) -> dict:
    f = S.RESULTS / "arm_study.json"
    if f.exists():
        saved = json.loads(f.read_text()).get("poses", {}).get(mount)
        if saved:
            return {k: tuple(v) for k, v in saved.items()}
    S.GRID = (31, 31, 15)
    return S.poses_for(mount)


def view(args):
    import mujoco.viewer
    p = S.params(args.base, args.mount, args.payload)
    ps = poses(args.mount)
    m, d, r = S.make(p, ps["stow"], settle=0.3)
    state = dict(drive=0, target=None, pan=0.0, pose=ps["stow"])

    def key(k):
        if 49 <= k <= 54:
            state["pose"] = ps[ORDER[k - 49]]
            r.arm.set_pose((A.pan_for(state["pan"]), *state["pose"][1:]))
        elif k == 32:                      # space: 2 m/s ahead to x 2.5, then back to 0, each a firmware stop
            state["drive"] = 1 if r.pose()[0] < 1.0 else -1
        elif k in (262, 263):              # right / left arrow: pan 15 deg
            state["pan"] = float(np.clip(state["pan"] + (15.0 if k == 263 else -15.0), -105, 105))
            r.arm.set_pose((A.pan_for(state["pan"]), *state["pose"][1:]))
        elif k == 82:                      # R
            mujoco.mj_resetData(m, d)
            r.arm.set_pose(ps["stow"], snap=True)
            state.update(drive=0, pan=0.0, pose=ps["stow"])

    with mujoco.viewer.launch_passive(m, d, key_callback=key) as v:
        v.cam.lookat[:] = [0.4, 0.0, 0.3]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 2.2, 145.0, -18.0
        dt = m.opt.timestep
        t_wall = time.perf_counter()
        while v.is_running():
            for _ in range(int(round(1 / 60 / dt))):
                x = r.pose()[0]
                vx = 0.0
                if state["drive"] == 1:
                    vx = 2.0 if x < 2.5 - 1.0 else 0.0
                elif state["drive"] == -1:
                    vx = -2.0 if x > 1.0 else 0.0
                r.drive_velocity(np.array([vx, 0.0]), None, r.pose()[2])
                r.step()
            v.cam.lookat[:2] = [r.pose()[0] + 0.3, r.pose()[1]]
            v.sync()
            t_wall += 1 / 60
            time.sleep(max(0.0, t_wall - time.perf_counter()))


def pngs():
    from PIL import Image, ImageDraw
    tiles = []
    for mount in S.MOUNTS:
        ps = poses(mount)
        p = S.params("sim", mount, 0.25)
        for name in ORDER:
            m, d, r = S.make(p, ps[name], settle=0.4)
            rnd = mujoco.Renderer(m, 420, 520)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [0.15, 0.0, 0.28]
            cam.distance, cam.azimuth, cam.elevation = 1.45, 125.0, -12.0
            opt = mujoco.MjvOption()
            rnd.update_scene(d, cam, opt)
            img = Image.fromarray(rnd.render())
            ImageDraw.Draw(img).text((10, 8), f"{mount} mount: {name}", fill=(0, 0, 0))
            tiles.append(img)
            rnd.close()
    w, h = tiles[0].size
    sheet = Image.new("RGB", (w * len(ORDER), h * len(S.MOUNTS)), "white")
    for k, t in enumerate(tiles):
        sheet.paste(t, ((k % len(ORDER)) * w, (k // len(ORDER)) * h))
    out = HERE / "renders" / "v11_arm_poses.png"
    out.parent.mkdir(exist_ok=True)
    sheet.save(out)
    print(f"wrote {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mount", default="centre", choices=list(S.MOUNTS))
    ap.add_argument("--payload", type=float, default=0.0)
    ap.add_argument("--base", default="sim", choices=("sim", "cad"))
    ap.add_argument("--png", action="store_true")
    a = ap.parse_args()
    pngs() if a.png else view(a)
