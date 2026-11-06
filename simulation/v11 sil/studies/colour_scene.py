"""The AI's find-by-colour-and-shape scene, rendered (ai_robot_architecture.md 5.2): the arm robot (ARM, or ARM_D435
with --camera d435) in the arena, the "colours" set in view ahead of it, the depth camera's depth and colour (PickCameras:
the stereo matcher's or the D435's error, a 0.3 deg calibration error), and omni_arm's color_vision.find on them.

    python studies/colour_scene.py                     prints the find for each colour / shape asked
    python studies/colour_scene.py --save PATH.npz     also saves the frames (omni_arm's test fixture)
    python studies/colour_scene.py --camera d435 --png out.png
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import arm_objects  # noqa: E402
from robot_export import arm_params  # noqa: E402
from robot_model import build_model  # noqa: E402
from sil_link import SimLink  # noqa: E402,F401  (puts ros/src on the path)
from sil_pick import HOME, PickCameras  # noqa: E402
from omni_arm import color_vision as cv  # noqa: E402
from omni_arm import floor_vision as fv  # noqa: E402
from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path  # noqa: E402
from omni_arm.pick import TidyTask  # noqa: E402

# the scene in the robot's frame (forward, left m; yaw deg): every thing in the depth camera's view
LAYOUT = {"cube40_purple": (0.62, 0.10, 20), "block": (0.70, -0.16, 65), "cube30": (0.55, -0.02, 10),
          "cube40": (0.82, 0.22, 35), "ball40": (0.80, -0.30, 0), "crayon": (0.95, 0.02, 120)}
QUERIES = (("purple", "square"), ("purple", "round"), ("purple", "long"), ("red", "square"), ("orange", "round"),
           ("blue", "any"), ("green", "long"))


def render(camera: str = "stereo", seed: int = 0, layout=LAYOUT):
    names = list(layout)
    # the robot at HOME facing -x: its forward is world -x, its left world -y
    xy = [(HOME[0] - f, HOME[1] - l) for f, l, _ in layout.values()]
    yaw = [180.0 + a for _, _, a in layout.values()]
    p = arm_params(home_xy=HOME, thrower_xy=(-2.0, 0.0), area_size=(9.0, 9.0),
                   floor_objects=arm_objects.objects_for(names, xy, yaw),
                   **({"depth_camera": "d435"} if camera == "d435" else {}))
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(200):                                   # let the things settle on the floor
        mujoco.mj_step(m, d)
    cams = PickCameras(m, d, p, seed=seed)
    z, rgb = cams.depth(), cams.color()
    kin = ArmKinematics(ArmConfig(config_path("d435" if camera == "d435" else "")))
    T = TidyTask(kin).T_base_cam                           # the robot's own idea of its camera (calibrated pose)
    truth = {n: (f, l) for n, (f, l, _) in layout.items()}
    cams.close()
    return z, rgb, cams.Kd, T, truth


def run(z, rgb, K, T, queries=QUERIES, truth=None, verbose=True):
    frame = fv.DepthFrame(1.0, z, K, T)
    out = {}
    for color, shape in queries:
        res, img = cv.find(frame, rgb, (0.0, 0.0, 0.0), fv.FloorMap(), 1.0, color, shape)
        out[(color, shape)] = (res, img)
        if verbose:
            print(f"-- {color} {shape}: {len(res['candidates'])} candidates, {len(res['image_only'])} image-only")
            for c in res["candidates"]:
                who = ""
                if truth:
                    who = min(truth, key=lambda n: math.hypot(truth[n][0] - c["x"], truth[n][1] - c["y"]))
                print(f"   id {c['id']:2d} {who:14s} score {c['score']:.2f}  colour {c['color']['name']} "
                      f"({c['color']['match']:.2f} asked)  shape {c['shape']['class']} l/s {c['shape']['long_short']} "
                      f"profile {c['shape']['profile']} flat {c['shape']['flat_top']} l/h "
                      f"{c['shape']['long_height']}  {c['size_mm']} mm  {c['pixels']} px")
    return out


def shuffled(seed: int) -> dict:
    """The colours set at other places and yaws in the view (seed 0: LAYOUT)."""
    if seed == 0:
        return LAYOUT
    rng = np.random.default_rng(seed)
    out, pts = {}, []
    for n in LAYOUT:
        while True:
            f, l = rng.uniform(0.5, 1.05), rng.uniform(-0.3, 0.3)
            if abs(l) < 0.5 * f and all(math.hypot(f - a, l - b) > 0.14 for a, b in pts):
                break
        pts.append((f, l))
        out[n] = (f, l, float(rng.uniform(0, 180)))
    return out


def score(out, truth) -> tuple:
    """(right, closer, wrong): the queries whose top candidate (score >= 0.5) is the right thing (or, for purple round,
    none is); whose right thing only matched as a guess (the AI moves closer: not wrong); and the rest."""
    want = {("purple", "square"): "cube40_purple", ("purple", "long"): "block", ("red", "square"): "cube30",
            ("orange", "round"): "ball40", ("blue", "any"): "cube40", ("green", "long"): "crayon",
            ("purple", "round"): None}
    right = closer = 0
    near = lambda c: min(truth, key=lambda n: math.hypot(truth[n][0] - c["x"], truth[n][1] - c["y"]))  # noqa: E731
    for q, name in want.items():
        res = out[q][0]
        cands = res["candidates"]
        top = cands[0] if cands and cands[0]["score"] >= 0.5 else None
        who = None if top is None else near(top)
        if who == name:
            right += 1
        elif top is None and res["hint"] and any(near(c) == name for c in cands if str(c["id"]) in res["hint"]):
            closer += 1
            print(f"   CLOSER {q}: {res['hint']}")
        else:
            print(f"   MISS {q}: got {who} (score {top['score'] if top else '-'}), wanted {name}")
    return right, closer, len(want) - right - closer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="stereo", choices=("stereo", "d435"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seeds", type=int, default=0, help="score this many shuffled layouts instead")
    ap.add_argument("--save", help="the frames as an .npz (omni_arm's test fixture)")
    ap.add_argument("--png", help="the purple-square find's annotated image")
    a = ap.parse_args()
    if a.seeds:
        tot = [0, 0, 0]
        for s in range(a.seeds):
            z, rgb, K, T, truth = render(a.camera, s, shuffled(s))
            r = score(run(z, rgb, K, T, truth=truth, verbose=False), truth)
            print(f"seed {s}: right {r[0]}, closer {r[1]}, wrong {r[2]}")
            tot = [x + y for x, y in zip(tot, r)]
        print(f"{a.camera}: {tot[0]} right, {tot[1]} 'move closer', {tot[2]} wrong of {sum(tot)} queries over "
              f"{a.seeds} layouts")
        return
    z, rgb, K, T, truth = render(a.camera, a.seed, shuffled(a.seed))
    out = run(z, rgb, K, T, truth=truth)
    print("right %d, closer %d, wrong %d" % score(out, truth))
    if a.save:
        mm = np.where(np.isfinite(z), np.round(z * 1000), 0).astype(np.uint16)
        np.savez_compressed(a.save, depth_mm=mm, rgb=rgb, K=np.array(K), T=T,
                            names=np.array(list(truth)), truth=np.array(list(truth.values())))
        print(f"saved {a.save}")
    if a.png:
        from PIL import Image
        Image.fromarray(out[("purple", "square")][1]).save(a.png)
        print(f"wrote {a.png}")


if __name__ == "__main__":
    main()
