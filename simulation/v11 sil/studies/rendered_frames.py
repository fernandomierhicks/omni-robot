"""v11 (design.md 30): the robot's ball detector and filter on rendered camera images (MuJoCo's renderer), against the
sim's sparse frames and the truth. The first step toward feeding real or rendered images into the catch path.

For each throw (catch_world.Thrower, the robot standing at home, firmware disarmed), every 1/120 s:
  rendered  mujoco.Renderer draws each stereo camera at 1280 x 800 (the OV9782's resolution, fovy 70 deg): shading,
            lighting, the scene's colors, no motion blur, no sensor noise
  sparse    the frames the catch game uses (catch_world.CameraSim: blur, edge noise; calibration error off here)
Both go through omni_catch's Detector and Perception (the robot's code); each is compared with where the true ball
projects (sphere_image) and, at the end, where the filter predicts the ball crosses the rim against where it did.

    python rendered_frames.py [--throws 4] [--png]      (Windows: needs the GPU renderer)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import sil_link  # noqa: E402,F401  (puts ros/src on the path)
from catch_world import CatchWorld, WorldConfig, expected_cameras  # noqa: E402
from omni_catch.detector import Detector  # noqa: E402
from omni_catch.frames import ArrayFrame, SparseFrame  # noqa: E402
from omni_catch.perception import Perception  # noqa: E402
from omni_catch.tracker import TrackerConfig, sphere_image  # noqa: E402

RESULTS = HERE / "results"


def true_px(m, d, cam, ball, R, f, W, H):
    cid = m.camera(cam).id
    Rc = d.cam_xmat[cid].reshape(3, 3)
    img = sphere_image(Rc.T @ (d.xpos[ball] - d.cam_xpos[cid]), R, f)
    return None if img is None else (W / 2 + img[0], H / 2 + img[1], img[2])


def run_throw(seed: int, dist: float, apex: float, png: bool = False):
    vis = replace(expected_cameras(), calib_rot_deg=0.0, calib_trans=0.0, calib_f_rel=0.0, ball_size_rel=0.0,
                  drop_prob=0.0, false_prob=0.0)
    w = CatchWorld(WorldConfig(dist=dist, apex_range=(apex, apex), seed=seed, vision=vis))
    m, d = w.m, w.d
    W, H = 1280, 800
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    ren = mujoco.Renderer(m, H, W)
    f = (H / 2) / math.tan(math.radians(w.p.camera_fovy_deg) / 2)
    ball = m.body("ball").id
    R = w.p.ball_spec.radius
    # the robot stands still at home: its pose is the truth (this study is about the pixels, not the odometry)
    tc = TrackerConfig(calib_rot_deg=0.02)
    percs = {k: Perception(tcfg=tc) for k in ("rendered", "sparse")}
    det = Detector()
    base = m.body("base").id
    for _ in range(int(0.5 / m.opt.timestep)):
        w.robot.step()
    pose0 = w.pose0
    for pc in percs.values():
        pc.add_pose(0.0, 0.0, 0.0, 0.0)
    pos, v0, spin, aim, _ = w.thrower.toss()
    w.place_ball(pos, v0, spin)
    t0 = d.time
    errs = {"rendered": [], "sparse": []}
    seen = {"rendered": 0, "sparse": 0}
    frames = 0
    t_frame = t0
    ms_render, ms_detect = [], []
    crossing = None
    preds = {}
    z_rim = float(w.robot.g.rim_center_robot(0.0)[1])
    saved = False
    while d.time - t0 < 2.5:
        w.robot.step()
        bp = d.xpos[ball]
        bv = d.sensor("ball_vel").data
        if crossing is None and bv[2] < 0 and bp[2] < z_rim:
            crossing = w.odom_from_world(bp[:2])
            for k, pc in percs.items():
                v = pc.view()
                if v is not None and v.ready:
                    cr = v.crossing(v.t, z_rim)
                    if cr is not None:
                        preds[k] = float(np.linalg.norm(cr[0][:2] - crossing))
            break
        if d.time < t_frame - 1e-9:
            continue
        t_frame = d.time + 1 / 120
        frames += 1
        t_rel = d.time - t0
        pair = {"rendered": [], "sparse": []}
        sp = w.cams.frame_pair(d.time)
        for k, cam in enumerate(("stereo_left", "stereo_right")):
            w0 = time.perf_counter()
            ren.update_scene(d, camera=cam)
            img = ren.render()
            ms_render.append(1000 * (time.perf_counter() - w0))
            fr_r = ArrayFrame(img)
            fr_s = SparseFrame(sp[k].width, sp[k].height, sp[k].idx, sp[k].clutter, sp[k].seed)
            pair["rendered"].append(fr_r)
            pair["sparse"].append(fr_s)
            tp = true_px(m, d, cam, ball, R, f, W, H)
            for name, fr in (("rendered", fr_r), ("sparse", fr_s)):
                w0 = time.perf_counter()
                res = det.detect(fr, pred=tp)
                if name == "rendered":
                    ms_detect.append(1000 * (time.perf_counter() - w0))
                if res is not None and tp is not None:
                    seen[name] += 1
                    errs[name].append((res[0] - tp[0], res[1] - tp[1], res[2] - tp[2], tp[2]))
            if png and not saved and tp is not None and tp[2] > 4 and k == 0 and t_rel > 0.3:
                _save_png(img, tp, det.detect(fr_r, pred=tp))
                saved = True
        for name, pc in percs.items():
            pc.process(d.time, pair[name])
    out = dict(seed=seed, dist=dist, apex=apex, frames=frames, seen=seen, predicted_crossing_error=preds,
               render_ms=float(np.median(ms_render)), detect_ms=float(np.median(ms_detect)))
    for name, e in errs.items():
        e = np.array(e) if e else np.zeros((0, 4))
        out[name] = dict(n=len(e), center_px_med=float(np.median(np.hypot(e[:, 0], e[:, 1]))) if len(e) else None,
                         center_px_95=float(np.percentile(np.hypot(e[:, 0], e[:, 1]), 95)) if len(e) else None,
                         radius_px_med=float(np.median(e[:, 2])) if len(e) else None)
    ren.close()
    w.close()
    return out


def _save_png(img, tp, res):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return
    u, v, r = tp
    x0, y0 = int(max(0, u - 60)), int(max(0, v - 40))
    crop = Image.fromarray(img[y0:y0 + 80, x0:x0 + 120]).resize((480, 320), Image.NEAREST)
    dr = ImageDraw.Draw(crop)
    s = 4
    dr.ellipse([(u - x0 - r) * s, (v - y0 - r) * s, (u - x0 + r) * s, (v - y0 + r) * s], outline=(0, 120, 255), width=2)
    if res is not None:
        dr.ellipse([(res[0] - x0 - res[2]) * s, (res[1] - y0 - res[2]) * s, (res[0] - x0 + res[2]) * s,
                    (res[1] - y0 + res[2]) * s], outline=(255, 0, 0), width=2)
    path = HERE.parent / "renders" / "v11_rendered_ball.png"
    path.parent.mkdir(exist_ok=True)
    crop.save(path)
    print(f"wrote {path} (blue: true silhouette, red: the detector's blob)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--throws", type=int, default=4)
    ap.add_argument("--png", action="store_true")
    a = ap.parse_args()
    rows = []
    cells = [(6.5, 1.0), (6.0, 1.5), (7.0, 0.75), (6.5, 2.0), (6.0, 1.0), (7.0, 1.5)]
    for k in range(a.throws):
        dist, apex = cells[k % len(cells)]
        r = run_throw(9100 + k, dist, apex, png=a.png and k == 0)
        rows.append(r)
        print(f"throw {k + 1}: {dist} m, apex {apex} m: frames {r['frames']}, ball found rendered {r['seen']['rendered']}"
              f" / sparse {r['seen']['sparse']}; center error median rendered {r['rendered']['center_px_med']:.2f} px, "
              f"sparse {r['sparse']['center_px_med']:.2f} px; rim crossing predicted "
              + ", ".join(f"{k_} {100 * v:.1f} cm" for k_, v in r['predicted_crossing_error'].items())
              + f"; render {r['render_ms']:.1f} ms, detect {r['detect_ms']:.2f} ms per frame")
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "rendered_frames.json").write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
