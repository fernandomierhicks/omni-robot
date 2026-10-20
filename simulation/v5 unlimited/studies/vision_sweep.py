"""How sensitive is catching to the camera and the vision pipeline? (vision.py)

1. Detector: blob center and size error vs range and ball speed (motion blur), for a
   ball in front of the left camera; the per-pixel noise model's own precision.
2. Game sweep: the full catch game (physics limits, moving catches) over the arena
   grid (thrower 1.5-3 m, apex 0.5-3 m, landing 0.3/0.6/1.0 m from the waiting spot,
   2 tosses per cell, the same throws for every variant). One setting changed at a
   time from the baseline (OV9782 1280x800 at 120 fps, 1 ms exposure, 50 ms latency,
   stereo + known-size ranging). For each variant:
     - catch rate;
     - landing-prediction error at the rim plane 0.5 s and 0.3 s before the ball gets
       there, and the time after release of the first usable estimate;
     - how honest the filter is: actual error / its own 1-sigma.

Writes results/vision_sweep.json and .png. ~6 min on 28 threads.
  python vision_sweep.py               # both parts
  python vision_sweep.py detector      # part 1 only
  python vision_sweep.py report        # reprint from the saved json
"""

from __future__ import annotations

import itertools
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from common import RESULTS, SimParams, save  # noqa: E402
from catch_game import CatchGame, GameConfig  # noqa: E402
from robot_model import build_model  # noqa: E402
from vision import VisionConfig, VisionPerception  # noqa: E402

DISTANCES = (1.5, 2.25, 3.0)
APEXES = (0.5, 1.0, 2.0, 3.0)
OFFSETS = (0.3, 0.6, 1.0)
TOSSES = 4
HOME = (1.0, 0.0)
T_GO = (0.5, 0.3)

B = VisionConfig()
# name: (perception, VisionConfig, SimParams overrides)
VARIANTS = {
    "baseline: stereo + size, 1280x800 @120, 1 ms": ("vision", B, {}),
    "v3 model (Gaussian noise on the true position)": ("model", None, {}),
    "exact state after 0.25 s (v3 'truth')": ("truth", None, {}),
    "stereo only (no size ranging)": ("vision", replace(B, mode="stereo"), {}),
    "mono: left camera, size ranging only": ("vision", replace(B, mode="mono"), {}),
    "60 fps": ("vision", replace(B, fps=60.0), {}),
    "90 fps": ("vision", replace(B, fps=90.0), {}),
    "200 fps": ("vision", replace(B, fps=200.0), {}),
    "exposure 0.5 ms": ("vision", replace(B, exposure=0.0005), {}),
    "exposure 4 ms": ("vision", replace(B, exposure=0.004), {}),
    "exposure 8 ms": ("vision", replace(B, exposure=0.008), {}),
    "latency 20 ms": ("vision", replace(B, latency_mean=0.020, latency_jitter=0.005), {}),
    "latency 100 ms": ("vision", replace(B, latency_mean=0.100, latency_jitter=0.025), {}),
    "latency 150 ms": ("vision", replace(B, latency_mean=0.150, latency_jitter=0.030), {}),
    "640x400 (2x2 binning)": ("vision", replace(B, width=640, height=400), {}),
    "1920x1200": ("vision", replace(B, width=1920, height=1200), {}),
    "baseline 10 cm": ("vision", B, dict(camera_baseline=0.10)),
    "baseline 30 cm": ("vision", B, dict(camera_baseline=0.30)),
    "FOV 90 deg": ("vision", B, dict(camera_fovy_deg=90.0)),
    "FOV 50 deg": ("vision", B, dict(camera_fovy_deg=50.0)),
    "edge noise 1.0 px": ("vision", replace(B, edge_sigma=1.0), {}),
    "perfect calibration": ("vision", replace(B, calib_rot_deg=0.0, calib_trans=0.0, calib_f_rel=0.0), {}),
    "calibration 0.2 deg": ("vision", replace(B, calib_rot_deg=0.2), {}),
    "calibration 0.5 deg": ("vision", replace(B, calib_rot_deg=0.5), {}),
    "stereo sync 2 ms (software)": ("vision", replace(B, sync_error=0.002), {}),
    "stereo sync 5 ms": ("vision", replace(B, sync_error=0.005), {}),
    "rolling shutter 20 ms, row-time stamped": ("vision", replace(B, rolling_readout=0.02), {}),
    "rolling shutter 20 ms, frame stamp only": ("vision", replace(B, rolling_readout=0.02, rolling_compensate=False), {}),
    "net opaque": ("vision", replace(B, net_transmission=0.0), {}),
    "ball size spread 3%": ("vision", replace(B, ball_size_rel=0.03), {}),
    "odometry drift 5%/m": ("vision", replace(B, odom_drift=0.05, yaw_drift_deg=1.5), {}),
    "5% clutter detections": ("vision", replace(B, false_prob=0.05), {}),
    "20% dropped frames": ("vision", replace(B, drop_prob=0.20), {}),
    "circle-fit detector": ("vision", replace(B, detector="circle"), {}),
    "filter process noise 0.2 m/s^2 (less conservative)": ("vision", replace(B, accel_sigma=0.2), {}),
    "fast pipeline: 200 fps, 20 ms, FOV 90": ("vision", replace(B, fps=200.0, latency_mean=0.020, latency_jitter=0.005),
                                              dict(camera_fovy_deg=90.0)),
    # thrown-ball spin (the toss model has none by default): does the less conservative filter still hold?
    "spin 10 rad/s, process noise 0.5 (baseline)": ("vision", B, dict(toss_spin=10.0)),
    "spin 10 rad/s, process noise 0.2": ("vision", replace(B, accel_sigma=0.2), dict(toss_spin=10.0)),
    "spin 10 rad/s, exact state after 0.25 s": ("truth", None, dict(toss_spin=10.0)),
}
SPIN_ONLY = [k for k in VARIANTS if k.startswith("spin")]


# --- 1. detector -------------------------------------------------------------
def detector_cell(args):
    rng_m, speed, seed = args
    p = SimParams(home_xy=(0.0, 0.0), area_size=(20.0, 20.0))     # no arena netting in the way
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    rng = np.random.default_rng(seed)
    from ball_flight import BallFlight
    vp = VisionPerception(m, d, BallFlight(0.022, 0.037, 0.5), p, replace(B, drop_prob=0.0), rng)
    vp.keep_stats = True
    cid = vp.cams[0]
    c = d.cam_xpos[cid].copy()
    Rc = d.cam_xmat[cid].reshape(3, 3).copy()
    bq, bv = m.joint("ball").qposadr[0], m.joint("ball").dofadr[0]
    for _ in range(60):
        # a point inside the central 60% of the image at this range
        x, y = rng.uniform(-0.3, 0.3, 2) * np.array([math.tan(math.radians(35)) * 1.6, math.tan(math.radians(35))])
        dirc = np.array([x, y, -1.0])
        pos = c + Rc @ (dirc / np.linalg.norm(dirc) * rng_m)
        vel = rng.normal(size=3)
        vel *= speed / np.linalg.norm(vel)
        d.qpos[bq:bq + 7] = [*pos, 1, 0, 0, 0]
        d.qvel[bv:bv + 6] = [*vel, 0, 0, 0]
        mujoco.mj_forward(m, d)
        vp._render_blob(0, 0.0)
    s = np.array(vp.stats) if vp.stats else np.zeros((0, 7))
    return dict(range=rng_m, speed=speed, r_px=float(np.median(s[:, 5])) if len(s) else float("nan"),
                n=len(s), center_rms=float(np.sqrt(np.mean(s[:, 2] ** 2) / 2)) if len(s) else float("nan"),
                size_bias=float(np.nanmean(s[:, 3])) if len(s) else float("nan"),
                size_rms=float(np.sqrt(np.nanmean(s[:, 3] ** 2))) if len(s) else float("nan"))


# --- 2. game sweep -------------------------------------------------------------
def game_cell(args):
    name, dist, apex, off, seed = args
    perc, vcfg, pkw = VARIANTS[name]
    pkw = dict(pkw)
    spin = pkw.pop("toss_spin", 0.0)
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(catch_zone_radius=off, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                     wait_between=0.8, catch_mode="moving", perception=perc, vision_cfg=vcfg, toss_spin=spin)
    game = CatchGame(m, d, p, cfg=cfg, seed=seed, verbose=False)
    z_plane = game.robot.g.rim_center_robot(0.0)[1]
    out = []
    traj, preds, first = [], [], None
    t_next = 0.0
    n_done = 0
    while len(game.results) < TOSSES:
        game.step()
        t = d.time
        if game.state == "flight":
            bp = d.sensor("ball_pos").data
            traj.append((t, bp[0], bp[1], bp[2]))
            ready = (game.perc.ready() if game.perc is not None else t - game.t_release >= cfg.perception_delay)
            if ready and first is None:
                first = t - game.t_release
            if ready and t >= t_next and game.cross_err is None:
                t_next = t + 0.025
                if game.perc is not None:
                    cr = game.perc.crossing(t, z_plane)
                    if cr is not None:
                        preds.append((t, cr[0][0], cr[0][1], math.sqrt(max(np.linalg.eigvalsh(cr[3])))))
                else:
                    hit = game.flight.crossing_state(bp.copy(), d.sensor("ball_vel").data.copy(), z_plane)
                    if hit is not None:
                        preds.append((t, hit[0][0], hit[0][1], 0.0))
        if len(game.results) > n_done:
            n_done = len(game.results)
            res = game.results[-1]
            out.append(_score(res, traj, preds, first, z_plane, p, game))
            traj, preds, first = [], [], None
    return dict(variant=name, distance=dist, apex=apex, offset=off, tosses=out)


def _score(res, traj, preds, first, z_plane, p, game):
    tr = np.array(traj)
    t_cross, xy_true = None, None
    for k in range(1, len(tr)):
        if tr[k - 1, 3] >= z_plane > tr[k, 3] and tr[k, 3] < tr[k - 1, 3]:
            f = (tr[k - 1, 3] - z_plane) / (tr[k - 1, 3] - tr[k, 3])
            t_cross = tr[k - 1, 0] + f * (tr[k, 0] - tr[k - 1, 0])
            xy_true = tr[k - 1, 1:3] + f * (tr[k, 1:3] - tr[k - 1, 1:3])
            break
    err, nees = {}, {}
    if t_cross is not None and preds:
        pr = np.array(preds)
        for tg in T_GO:
            k = int(np.argmin(np.abs((t_cross - pr[:, 0]) - tg)))
            if abs((t_cross - pr[k, 0]) - tg) < 0.03:
                e = float(np.hypot(*(pr[k, 1:3] - xy_true)))
                err[str(tg)] = e
                nees[str(tg)] = e / pr[k, 3] if pr[k, 3] > 0 else float("nan")
    rim_margin = p.funnel_rim_radius - p.ball_spec.radius
    why = "caught" if res.caught else ("reach" if not np.isfinite(res.pos_error) or res.pos_error > rim_margin else "bounce")
    perc = game.perc
    return dict(caught=bool(res.caught), why=why, pos_error=res.pos_error, first=first, err=err, nees=nees,
                stereo_frames=getattr(perc, "stereo_frames", None), mono_frames=getattr(perc, "mono_frames", None),
                rejected=getattr(getattr(perc, "tracker", None), "rejected", None))


# --- reporting -------------------------------------------------------------------
def summarize(rows):
    out = {}
    for name in VARIANTS:
        ts = [t for r in rows if r["variant"] == name for t in r["tosses"]]
        if not ts:
            continue
        s = dict(n=len(ts), caught=float(np.mean([t["caught"] for t in ts])),
                 reach=sum(t["why"] == "reach" for t in ts), bounce=sum(t["why"] == "bounce" for t in ts),
                 first=float(np.nanmedian([t["first"] if t["first"] is not None else np.nan for t in ts])),
                 offset=float(np.nanmedian([t["pos_error"] for t in ts if t["caught"]] or [np.nan])))
        for tg in T_GO:
            e = [t["err"].get(str(tg), np.nan) for t in ts]
            s[f"err{tg}_med"] = float(np.nanmedian(e))
            s[f"err{tg}_p90"] = float(np.nanpercentile(e, 90)) if np.isfinite(e).any() else float("nan")
            s[f"nees{tg}"] = float(np.nanmedian([t["nees"].get(str(tg), np.nan) for t in ts]))
        out[name] = s
    return out


def report(data):
    if "detector" in data:
        print("1. Detector (left camera, ball in the central 60% of the image): per-axis center rms, size bias / rms (px)")
        for c in data["detector"]:
            print(f"   {c['range']:4.1f} m, {c['speed']:4.1f} m/s: radius {c['r_px']:5.1f} px, n {c['n']:2d}: center "
                  f"{c['center_rms']:.3f}, size {c['size_bias']:+.3f} / {c['size_rms']:.3f}")
    if "rows" not in data:
        return
    s = summarize(data["rows"])
    print("\n2. Catch game, moving catches, physics limits, " +
          f"{s[next(iter(s))]['n']} tosses per variant (same throws). Landing error at the rim plane (cm), median / 90%:")
    print(f"   {'variant':52s} caught  reach/bounce  first est   err 0.5 s     err 0.3 s    err/sigma  offset")
    for name, v in s.items():
        print(f"   {name:52s} {100 * v['caught']:4.0f}%   {v['reach']:3d}/{v['bounce']:<3d}     {v['first']:5.3f} s  "
              f"{100 * v['err0.5_med']:5.1f}/{100 * v['err0.5_p90']:5.1f}  {100 * v['err0.3_med']:5.1f}/{100 * v['err0.3_p90']:5.1f}"
              f"   {v['nees0.3']:5.2f}    {100 * v['offset']:4.1f} cm")


def plot(data):
    s = summarize(data["rows"])
    names = list(s)
    fig, ax = plt.subplots(1, 2, figsize=(13, 0.32 * len(names) + 1.5), sharey=True)
    y = np.arange(len(names))
    ax[0].barh(y, [100 * s[n]["caught"] for n in names], color="#2a6fdb")
    ax[0].axvline(100 * s[names[0]]["caught"], color="0.3", lw=0.8, ls="--")
    ax[0].set_xlabel("caught (%)")
    ax[0].set_xlim(40, 100)
    ax[0].set_yticks(y, names, fontsize=7)
    ax[0].invert_yaxis()
    for tg, col in zip(T_GO, ("#e8743b", "#19a979")):
        ax[1].barh(y + (0.2 if tg == 0.3 else -0.2), [100 * s[n][f"err{tg}_med"] for n in names], height=0.4,
                   color=col, label=f"{tg} s before arrival")
    ax[1].set_xlabel("landing prediction error, median (cm)")
    ax[1].legend(fontsize=7)
    for a in ax:
        a.grid(axis="x", alpha=0.3)
    fig.suptitle("Vision sensitivity: one change at a time from the baseline (top row)", fontsize=10)
    fig.tight_layout()
    fig.savefig(RESULTS / "vision_sweep.png", dpi=120)


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "all"
    if arg == "report":
        data = json.loads((RESULTS / "vision_sweep.json").read_text())
        report(data)
        plot(data)
        return
    data = {}
    if arg == "spin":                  # only the spin variants, merged into the saved results
        data = json.loads((RESULTS / "vision_sweep.json").read_text())
        names = SPIN_ONLY
    else:
        names = list(VARIANTS)
    with ProcessPoolExecutor(min(26, os.cpu_count() or 4)) as pool:
        if arg != "spin":
            det = [(rg, sp, k) for k, (rg, sp) in enumerate(itertools.product((0.5, 1.0, 2.0, 3.0, 4.0, 6.0), (0.0, 6.0)))]
            data["detector"] = list(pool.map(detector_cell, det))
        if arg != "detector":
            cells = [(n, dd, a, o, j) for n in names
                     for j, (dd, a, o) in enumerate(itertools.product(DISTANCES, APEXES, OFFSETS))]
            rows = list(pool.map(game_cell, cells, chunksize=1))
            data["rows"] = [r for r in data.get("rows", []) if r["variant"] not in names] + rows
    save("vision_sweep", data)
    report(data)
    if "rows" in data:
        plot(data)


if __name__ == "__main__":
    main()
