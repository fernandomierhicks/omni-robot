"""Where can the stereo cameras go on the compact robot, and what cable run does each spot need?

Mounts (params.camera_mount):
  rim          on the funnel rim as in v2: moves with the tilt, so its cables must
               flex through the pivot and the extrinsics change with the tilt angle
  plate_front  fixed, low, at the front edge of the payload plate
  side_posts   fixed, on thin posts beside the funnel at rim height (0.40 m baseline)

For each mount, full catch games run headless. During each flight, the ball is
checked at 120 fps in both cameras: inside the field of view, and a ray from
the lens reaches it without hitting the robot, the funnel net or the thrower.
Reported per mount: stereo coverage of the first 0.5 s after release (what the
trajectory fit uses), the time before rim crossing when stereo is last
available, ball size in pixels, and the cable run to the Jetson.

Writes results/camera_view.json.
"""

from __future__ import annotations

import math
import sys

import mujoco
import numpy as np

from common import SimParams, save  # noqa: F401
from catch_game import CatchGame
from perception import visible
from robot_model import build_model, camera_positions, geometry

FPS = 120
H_PX, W_PX = 800, 1280                     # OV9782
JETSON = np.array([-0.07, 0.0, 0.135])     # robot frame, CSI connectors on the carrier (approx)


def cable_run(p: SimParams) -> float:
    """Rough cable route (m) from each camera to the Jetson: along the structure, plus slack."""
    g = geometry(p)
    parent, cams = camera_positions(p, g)
    runs = []
    for _, pos in cams:
        if parent == "module":           # down the module to the pivot, a service loop, then to the Jetson
            c = np.array([*g.module_to_robot((pos[0], pos[2]), 0.0)])
            piv = np.array(g.pivot_r)
            run = abs(c[0] - piv[0]) + abs(c[1] - piv[1]) + abs(pos[1]) + 0.10 + \
                abs(piv[0] - JETSON[0]) + abs(piv[1] - JETSON[2]) + 0.105
        else:
            run = abs(pos[0] - JETSON[0]) + abs(pos[1] - JETSON[1]) + abs(pos[2] - JETSON[2])
        runs.append(run + 0.05)          # connector slack
    return max(runs)


def run_mount(mount: str, tosses: int = 15, seed: int = 7):
    p = SimParams(camera_mount=mount)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, seed=seed, verbose=False)
    cams = [m.camera(f"stereo_{s}").id for s in ("left", "right")]
    ball_gid = m.geom("ball").id
    groups = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
    fovy = math.radians(p.camera_fovy_deg)
    f_px = (H_PX / 2) / math.tan(fovy / 2)
    per_toss = []
    track = None
    next_frame = 0.0
    while len(game.results) < tosses:
        game.step()
        if game.state == "flight":
            if track is None:
                track = dict(t0=game.t_release, t=[], vis=[], dist=[])
                next_frame = d.time
            if d.time >= next_frame:
                next_frame += 1.0 / FPS
                bp = d.sensor("ball_pos").data.copy()
                vis = [visible(m, d, c, bp, ball_gid, fovy, groups) for c in cams]
                track["t"].append(d.time - track["t0"])
                track["vis"].append(vis)
                track["dist"].append(float(np.linalg.norm(bp - d.cam_xpos[cams[0]])))
                fc = game.robot.funnel_center_world()
                if bp[2] < fc[2] and "t_cross" not in track:
                    track["t_cross"] = d.time - track["t0"]
        elif track is not None:
            t = np.array(track["t"])
            vis = np.array(track["vis"])
            both = vis.all(axis=1)
            t_cross = track.get("t_cross", t[-1])
            fit = t <= 0.5
            last = t[both & (t <= t_cross)].max() if (both & (t <= t_cross)).any() else float("nan")
            per_toss.append(dict(
                stereo_fit_window=float(both[fit].mean()) if fit.any() else 0.0,
                any_fit_window=float(vis[fit].any(axis=1).mean()) if fit.any() else 0.0,
                stereo_whole_flight=float(both[t <= t_cross].mean()),
                last_stereo_before_crossing=float(t_cross - last),
                min_px=float(f_px * 0.074 / max(track["dist"])), max_px=float(f_px * 0.074 / min(track["dist"]))))
            track = None
    res = dict(mount=mount, cable_run=cable_run(p), tosses=per_toss,
               caught=sum(r.caught for r in game.results), n=len(game.results))
    for k in ("stereo_fit_window", "stereo_whole_flight", "last_stereo_before_crossing", "min_px"):
        res[k + "_median"] = float(np.nanmedian([x[k] for x in per_toss]))
        res[k + "_min"] = float(np.nanmin([x[k] for x in per_toss]))
    return res


def main():
    from concurrent.futures import ProcessPoolExecutor
    mounts = sys.argv[1:] or ["side_posts", "plate_front", "rim"]
    out = []
    with ProcessPoolExecutor(len(mounts)) as pool:
        results = list(pool.map(run_mount, mounts))
    for mount, r in zip(mounts, results):
        out.append(r)
        print(f"{mount:12s} cable ~{r['cable_run'] * 100:.0f} cm | stereo in the first 0.5 s: median "
              f"{100 * r['stereo_fit_window_median']:.0f}% (worst {100 * r['stereo_fit_window_min']:.0f}%) | whole flight "
              f"{100 * r['stereo_whole_flight_median']:.0f}% | last stereo frame {1000 * r['last_stereo_before_crossing_median']:.0f} ms "
              f"before the rim (worst {1000 * max(x['last_stereo_before_crossing'] for x in r['tosses']):.0f}) | "
              f"ball >= {r['min_px_min']:.0f} px | caught {r['caught']}/{r['n']}", flush=True)
    save("camera_view", out)


if __name__ == "__main__":
    main()
