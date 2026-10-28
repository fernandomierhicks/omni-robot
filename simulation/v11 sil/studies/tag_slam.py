"""design.md 10: AprilTags as unknown landmarks with the odometry and the lidar, on v9's house tour.

The robot drives the tour on the sim's truth (bridge_core mode "tour", as v9 / v10 scored localization) while four
estimators run on what its sensors give:
  odometry        v9's EKF on wheels + IMU (the odom frame): drifts
  tags+odometry   omni_slam.TagSlam on that odometry and the left camera's tag detections (tag_sim.py): the tags
                  are unknown at the start; the first keyframe anchors the map
  lidar           v9's EKF with its known-map scan matcher (slam_toolbox's localization stands for it on the robot)
  lidar+tags      TagSlam with the lidar's poses as absolute factors and the tags: the map frame is the lidar's
Reported: each one's position / heading error against the truth (median, 95%, max), how well the tags were mapped
(position error of each), one-tag relocalization from the final map, and how often detections were refused and why.

  python studies/tag_slam.py [--seconds 150] [--seed 0] [--calib-deg 0.3]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from bridge_core import CoreConfig, SimCore  # noqa: E402
from tag_sim import TagCamera, TagWorld  # noqa: E402
from omni_slam.graph import se2_between, se2_compose, wrap  # noqa: E402
from omni_slam.slam import TagSlam  # noqa: E402
from omni_slam.tags import Pinhole, log_rot, rodrigues  # noqa: E402
from omni_bringup.frames import load_frames  # noqa: E402

from robot_export import ROBOT_JSON  # noqa: E402  (the repo's, or WSL's mirror)


def T_base(child: str) -> np.ndarray:
    fr = {f.child: f for f in load_frames(ROBOT_JSON)}
    M, c = np.eye(4), child
    chain = []
    while c in fr:
        chain.append(fr[c])
        c = fr[c].parent
    for f in reversed(chain):
        A = np.eye(4)
        A[:3, :3], A[:3, 3] = f.R, f.xyz
        M = M @ A
    return M


def quat_T(pos, q):
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, pos
    return M


def err_stats(e) -> dict:
    e = np.asarray(e)
    if not len(e):
        return {}
    return dict(med=round(float(np.median(e)), 4), p95=round(float(np.percentile(e, 95)), 4),
                max=round(float(np.max(e)), 4))


def yaw_of(quat) -> float:
    return math.atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2] ** 2 + quat[3] ** 2))


def run(seconds: float = 150.0, seed: int = 0, calib_deg: float = 0.3, lidar_gap=(18.0, 33.0),
        verbose: bool = True) -> dict:
    """lidar_gap (s, s): no lidar fixes then (an open room, a crowd, a lidar fault): the "lidar" estimate dead-
    reckons from its last fix, the tag SLAMs get none; reported apart ("in the gap")."""
    core = SimCore(CoreConfig(base="v7", seed=seed, mode="tour", shadow=("wheels+IMU", "wheels+IMU+lidar")))
    world = TagWorld(core.m, core.d)
    cam = Pinhole.from_fovy(json.loads(ROBOT_JSON.read_text())["cameras"][0]["fovy_deg"])
    Tbc_true = T_base("cam_left_optical")
    tagcam = TagCamera(world, cam, Tbc_true, seed=seed)
    rng = np.random.default_rng(seed + 5)
    Tbc_est = Tbc_true.copy()                                # what the robot believes: its calibration's error
    Tbc_est[:3, :3] = rodrigues(rng.normal(0, math.radians(calib_deg) / math.sqrt(3), 3)) @ Tbc_true[:3, :3]
    Tbc_est[:3, 3] += rng.normal(0, 0.002, 3)
    ts_odom = TagSlam(cam, Tbc_est, anchor="first")
    ts_lidar = TagSlam(cam, Tbc_est, anchor="lidar", solve_every=2)
    ts_ref = TagSlam(cam, Tbc_est, anchor="lidar", solve_every=2, refine_cam_deg=0.5)
    slams = (("tags+odometry", ts_odom), ("lidar+tags", ts_lidar), ("lidar+tags+cam", ts_ref))
    names = ("odometry", "tags+odometry", "lidar", "lidar+tags", "lidar+tags+cam")
    errs = {p: {k: ([], []) for k in names} for p in ("all", "gap")}
    outcomes = Counter()
    single, voted = [], []
    t_vote = 10.0
    t_lidar_next = 0.0
    lidar_dr = None                                          # the lidar estimate dead-reckoning in the gap
    w0 = time.perf_counter()
    while core.d.time < seconds and core.phase != "done":
        ev = core.step()
        t = core.d.time
        if ev.imu is not None and ev.imu[1] is not None:      # the BNO085's game rotation vector: roll and pitch
            w, x, y, z = ev.imu[1]
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
            for _, ts in slams:
                ts.on_attitude(t, roll, pitch)
        if ev.truth is None:
            continue
        tt, pos, quat, tw = ev.truth
        truth = np.array([pos[0], pos[1], yaw_of(quat)])
        odo = np.asarray(ev.shadow["wheels+IMU"])
        lid = np.asarray(ev.shadow["wheels+IMU+lidar"])
        in_gap = lidar_gap is not None and lidar_gap[0] <= t < lidar_gap[1]
        if in_gap:
            if lidar_dr is None:
                lidar_dr = (lid.copy(), odo.copy())
            lid = se2_compose(lidar_dr[0], se2_between(lidar_dr[1], odo))
        for _, ts in slams:
            ts.on_odom(t, odo)
        if t >= t_lidar_next and not in_gap:
            t_lidar_next = t + 0.1
            P = core.loc.f["wheels+IMU+lidar"].P[:3, :3]
            for _, ts in slams[1:]:
                ts.on_lidar(t, lid, P + np.diag([0.01 ** 2, 0.01 ** 2, math.radians(0.5) ** 2]))
        if tagcam.due(t):
            dets = tagcam.observe(t, quat_T(pos, quat), tw[2])
            if dets:
                for o in ts_odom.on_tags(t, dets):
                    outcomes[o] += 1
                for _, ts in slams[1:]:
                    ts.on_tags(t, dets)
                if t >= t_vote:          # as after a kidnap: where the map's tags say the robot is
                    for dd in dets:
                        r = ts_ref.pose_from_tag(dd)
                        if r is not None:
                            single.append((float(np.hypot(r[0][0] - truth[0], r[0][1] - truth[1])),
                                           getattr(dd, "misdecode", False)))
                        v = ts_ref.vote(t, dd)
                        if v is not None:
                            voted.append((float(np.hypot(v[0][0] - truth[0], v[0][1] - truth[1])),
                                          abs(math.degrees(float(wrap(v[0][2] - truth[2]))))))
                            t_vote = t + 2.0
                            break
        for name, est in (("odometry", odo), ("tags+odometry", ts_odom.pose(t)), ("lidar", lid),
                          ("lidar+tags", ts_lidar.pose(t)), ("lidar+tags+cam", ts_ref.pose(t))):
            if est is None:
                continue
            e = (float(np.hypot(est[0] - truth[0], est[1] - truth[1])), abs(math.degrees(float(wrap(est[2] - truth[2])))))
            for p in ("all", "gap") if in_gap else ("all",):
                errs[p][name][0].append(e[0])
                errs[p][name][1].append(e[1])
    for _, ts in slams:
        ts.solve(20)
    out = dict(seconds=round(core.d.time, 1), wall_s=round(time.perf_counter() - w0, 1), distance_m=round(core.dist, 1),
               tags_in_house=len(world.tags), detections=tagcam.n_dets, frames=tagcam.n_frames,
               calib_error_deg=calib_deg, lidar_gap=lidar_gap, outcomes=dict(outcomes))
    for p in ("all", "gap"):
        out[f"position_m_{p}"] = {k: err_stats(v[0]) for k, v in errs[p].items()}
        out[f"heading_deg_{p}"] = {k: err_stats(v[1]) for k, v in errs[p].items()}
    truth_tags = {tg.tag_id: tg for tg in world.tags}
    for name, ts in slams:
        e = [float(np.linalg.norm(v["t"] - truth_tags[tid].t)) for tid, v in ts.tags().items()]
        out[f"tag_map_{name}"] = dict(mapped=len(e), **err_stats(e), keyframes=len(ts.g.X),
                                      tag_rms_px=round(ts.g.stats.tag_rms_px, 3), dropped_views=ts.n_dropped)
    # the camera mount: what the robot was given, how far off that was, and what the graph made of it (it also takes
    # up the IMU's roll / pitch reference error: the camera is seen through the IMU's attitude)
    true_corr = np.degrees(log_rot(Tbc_true[:3, :3] @ Tbc_est[:3, :3].T))
    bias = [math.degrees(b) for b in core.sn.tilt_bias]               # roll, pitch reference error of the IMU
    out["camera_rotation"] = dict(true_error_deg=[round(float(c), 3) for c in true_corr],
                                  imu_tilt_bias_deg=[round(b, 3) for b in bias],
                                  # what the camera is off by as seen through the IMU's attitude (small angles)
                                  expected_deg=[round(float(true_corr[0] - bias[0]), 3),
                                                round(float(true_corr[1] - bias[1]), 3), round(float(true_corr[2]), 3)],
                                  **ts_ref.g.cam_correction())
    good = [e for e, fake in single if not fake]
    out["relocalize"] = dict(single_views=len(single), single_err_m=err_stats(good),
                             single_misdecodes=sum(1 for _, fake in single if fake),
                             single_worst_m=round(max((e for e, _ in single), default=0.0), 3),
                             votes=len(voted), vote_err_m=err_stats([v[0] for v in voted]),
                             vote_err_deg=err_stats([v[1] for v in voted]))
    if verbose:
        print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=150.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--calib-deg", type=float, default=0.3)
    a = ap.parse_args()
    res = run(a.seconds, a.seed, a.calib_deg)
    p = HERE / "results" / "tag_slam.json"
    p.parent.mkdir(exist_ok=True)
    p.write_text(json.dumps(res, indent=1))
