"""omni_calib's solvers on synthetic data with a known truth, in the robot's own geometry (design.md 10): the cameras
and lidar where robot_v11.json puts them, the OV9782's 1280 x 800 at the sim's field of view.
Run: python -m pytest -q ros/src/omni_calib/test"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / "omni_calib"))

from omni_calib import cameras, extrinsics, online, results, targets, temporal  # noqa: E402
from omni_calib.geometry import Camera, T, inv, rodrigues, rot_deg  # noqa: E402

ROBOT = json.loads((SRC / "omni_catch" / "omni_catch" / "config" / "robot_v11.json").read_text())


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def frame(child):
    """base_link -> child, composed from the generated frames."""
    fr = {f["child"]: f for f in ROBOT["frames"]}
    M = np.eye(4)
    chain = []
    while child in fr:
        chain.append(fr[child])
        child = fr[child]["parent"]
    for f in reversed(chain):
        M = M @ T(quat_R(f["quat"]), f["xyz"])
    return M


def true_camera(dist=(-0.08, 0.02, 0.0005, -0.0003, 0.0)):
    c = ROBOT["cameras"][0]
    return Camera.from_fovy(c["fovy_deg"], c["width"], c["height"], dist)


def board_views(cam, obj, n, rng, noise=0.2, dist_range=(0.5, 1.4)):
    """n poses of a target in front of the camera, tilted up to 40 deg, spread over the image; pixels with noise."""
    views, poses = [], []
    c = obj[:, :2].mean(axis=0)
    while len(views) < n:
        R = rodrigues(rng.uniform(-0.7, 0.7, 3)) @ rodrigues([math.pi, 0, 0])      # facing the camera
        z = rng.uniform(*dist_range)
        off = np.array([rng.uniform(-0.45, 0.45) * z, rng.uniform(-0.3, 0.3) * z, z])
        t = off - R @ np.array([c[0], c[1], 0.0])
        uv = cam.project(obj @ R.T + t)
        ok = np.all(np.isfinite(uv), axis=1) & (uv[:, 0] > 0) & (uv[:, 0] < cam.width) & (uv[:, 1] > 0) & \
            (uv[:, 1] < cam.height)
        if ok.sum() < 0.7 * len(obj):
            continue
        views.append((obj[ok], uv[ok] + rng.normal(0, noise, (ok.sum(), 2))))
        poses.append((R, t, ok))
    return views, poses


def test_intrinsics_from_an_aprilgrid_recover_focal_length_centre_and_distortion():
    rng = np.random.default_rng(1)
    cam = true_camera()
    _, obj = targets.aprilgrid(6, 6, 0.04, 0.3)
    views, _ = board_views(cam, obj, 25, rng)
    r = cameras.calibrate_camera(views, cam.width, cam.height)
    c = r["camera"]
    assert abs(c.fx / cam.fx - 1) < 0.003 and abs(c.fy / cam.fy - 1) < 0.003
    assert abs(c.cx - cam.cx) < 2.0 and abs(c.cy - cam.cy) < 2.0
    assert c.dist[0] == pytest.approx(cam.dist[0], abs=0.01)
    assert r["rms_px"] == pytest.approx(0.2 * math.sqrt(2), rel=0.15)          # the noise: nothing left unmodelled
    assert r["sigma"]["fx"] < 2.0 and r["coverage"] > 0.6
    ok, _ = results.passes("intrinsics", dict(rms_px=r["rms_px"], coverage=r["coverage"]))
    assert ok


def test_stereo_extrinsics_recover_the_baseline_and_the_relative_rotation():
    rng = np.random.default_rng(2)
    cam = true_camera()
    T_l, T_r = frame("cam_left_optical"), frame("cam_right_optical")
    T_rl = inv(T_r) @ T_l                                        # left optical -> right optical
    bend = rodrigues(np.radians([0.05, -0.12, 0.08]))              # the real bar is not the drawing
    R_true, t_true = bend @ T_rl[:3, :3], T_rl[:3, 3]
    _, obj = targets.aprilgrid(6, 6, 0.04, 0.3)
    vl, vr = [], []
    while len(vl) < 20:
        (v, ), ((R, t, ok), ) = board_views(cam, obj, 1, rng, noise=0.0)
        Rr, tr = R_true @ R, R_true @ t + t_true
        uv_l, uv_r = cam.project(obj @ R.T + t), cam.project(obj @ Rr.T + tr)
        both = np.all(np.isfinite(uv_l), 1) & np.all(np.isfinite(uv_r), 1)
        both &= (uv_r[:, 0] > 0) & (uv_r[:, 0] < cam.width) & (uv_r[:, 1] > 0) & (uv_r[:, 1] < cam.height) & ok
        if both.sum() < 40:
            continue
        vl.append((obj[both], uv_l[both] + rng.normal(0, 0.2, (both.sum(), 2))))
        vr.append((obj[both], uv_r[both] + rng.normal(0, 0.2, (both.sum(), 2))))
    r = cameras.calibrate_stereo(vl, vr, cam, cam)
    assert r["baseline_m"] == pytest.approx(0.19, abs=0.0015)
    assert rot_deg(r["R"], R_true) < 0.02                           # the 0.04 deg the catch rate needs (design.md 3)
    assert r["epipolar_px"] < 0.3 and r["sigma_rot_deg"] < 0.02
    # the same pair with the drawing's rotation is 0.15 deg off: its epipolar error says so (past the 0.3 px limit)
    assert cameras.epipolar_px(vl, vr, cam, cam, T_rl[:3, :3], t_true) > 0.4


def _motion(t):
    """A wiggle: yaw back and forth with a roll and pitch rock (the base rocked by hand)."""
    w = np.column_stack([0.4 * np.sin(2 * np.pi * 1.7 * t), 0.3 * np.sin(2 * np.pi * 2.3 * t + 1),
                         2.0 * np.sin(2 * np.pi * 1.0 * t)])
    return w


def test_camera_imu_time_offset_and_rotation_from_a_wiggle():
    rng = np.random.default_rng(3)
    R_base_cam = frame("cam_left_optical")[:3, :3]
    R_base_imu = frame("imu_link")[:3, :3] @ rodrigues(np.radians([0.4, -0.3, 0.6]))   # the board isn't square
    R_imu_cam = R_base_imu.T @ R_base_cam
    lag = 0.0073                                                 # the camera's stamps run 7.3 ms late
    t_g = np.arange(0, 8, 1 / 400)
    w_base = _motion(t_g)
    gyro = (w_base @ R_base_imu) + rng.normal(0, 0.003, w_base.shape)        # each in its own frame
    t_c = np.arange(0.05, 7.95, 1 / 120)
    w_cam_true = _motion(t_c) @ R_base_cam + rng.normal(0, 0.01, (len(t_c), 3))
    stamps_c = t_c + lag
    r = temporal.time_offset(stamps_c, np.linalg.norm(w_cam_true, axis=1), t_g, np.linalg.norm(gyro, axis=1))
    assert r["observable"] and r["offset"] == pytest.approx(-lag, abs=0.2e-3) and r["sigma"] < 0.3e-3
    # with the offset out, the rates line up and give the camera's axes in the IMU's
    gi = np.column_stack([np.interp(stamps_c + r["offset"], t_g, gyro[:, k]) for k in range(3)])
    R, rms, cond = temporal.rotation_from_rates(w_cam_true, gi)
    assert rot_deg(R, R_imu_cam) < 0.3 and cond > 0.01          # (the rocking is smaller than the yaw)
    # a constant spin says nothing about the offset
    w_spin = np.full(len(t_c), 1.5) + rng.normal(0, 0.01, len(t_c))
    g_spin = np.full(len(t_g), 1.5) + rng.normal(0, 0.003, len(t_g))
    assert not temporal.time_offset(stamps_c, w_spin, t_g, g_spin)["observable"]


def _board_scan(R_lb, t_lb, wh, rng):
    """Lidar points (laser frame, z = 0) where the scan plane crosses a board (board -> laser pose R_lb, t_lb)."""
    n = R_lb[:, 2]
    pts = []
    for a in np.radians(np.arange(-60, 60, 0.225)):
        d = np.array([math.cos(a), math.sin(a), 0.0])
        den = n @ d
        if abs(den) < 1e-6:
            continue
        s = (n @ t_lb) / den
        if s <= 0.2:
            continue
        p = s * d
        q = R_lb.T @ (p - t_lb)
        if 0 <= q[0] <= wh[0] and 0 <= q[1] <= wh[1]:
            pts.append(p * (1 + rng.normal(0, 0.01 / 2.5)))
    return np.array(pts)


def test_lidar_to_camera_from_a_board_seen_by_both():
    rng = np.random.default_rng(4)
    cam = true_camera(dist=(0, 0, 0, 0, 0))
    T_cl = inv(frame("cam_left_optical")) @ frame("laser")         # laser -> camera
    wh = (0.8, 0.6)
    views, checks = [], []
    while len(views) < 14:
        # the board stands on the floor across the lidar's plane, 1-2.5 m ahead, turned and leaned
        yaw, lean = rng.uniform(-0.6, 0.6), rng.uniform(-0.5, 0.5)
        R_lb = rodrigues([0, 0, yaw]) @ rodrigues([0, lean, 0]) @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.0]])
        center = np.array([rng.uniform(1.0, 2.5), rng.uniform(-0.5, 0.5), 0.0])
        t_lb = center - R_lb @ np.array([wh[0] / 2, wh[1] / 2, 0.0])
        pts = _board_scan(R_lb, t_lb, wh, rng)
        if len(pts) < 20:
            continue
        R_cb = T_cl[:3, :3] @ R_lb
        t_cb = T_cl[:3, :3] @ t_lb + T_cl[:3, 3]
        uv = cam.project((np.array([(0, 0, 0), (wh[0], 0, 0), (0, wh[1], 0), wh + (0,)])) @ R_cb.T + t_cb)
        if not np.all(np.isfinite(uv)):
            continue
        n, d = extrinsics.board_plane(R_cb, t_cb)
        views.append((n, d, pts))
        checks.append((pts, R_cb, t_cb))
    R0 = rodrigues(np.radians([1.5, -2.0, 1.0])) @ T_cl[:3, :3]       # the drawing, a little off
    t0 = T_cl[:3, 3] + np.array([0.012, -0.01, 0.008])
    r = extrinsics.lidar_to_camera(views, R0, t0)
    assert np.linalg.norm(r["t"] - T_cl[:3, 3]) < 0.006 and rot_deg(r["R"], T_cl[:3, :3]) < 0.4
    assert r["rms_mm"] < 12
    good = np.mean([extrinsics.lidar_in_image(cam, r["R"], r["t"], *c, wh) for c in checks])
    bad = np.mean([extrinsics.lidar_in_image(cam, rodrigues([0.25, 0, 0]) @ R0, t0, *c, wh)     # 14 deg of pitch
                   for c in checks])
    assert good > 0.95 and bad < good - 0.1


def _se2(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def test_lidar_on_the_base_from_odometry_and_scan_matching_moves():
    rng = np.random.default_rng(5)
    X = _se2(0.14, -0.004, math.radians(0.8))                        # the lidar on base_link, as mounted
    sens, base = [], []
    for _ in range(40):
        B = _se2(rng.uniform(-0.4, 0.6), rng.uniform(-0.3, 0.3), rng.uniform(-1.5, 1.5))
        S = np.linalg.inv(X) @ B @ X
        sens.append((S[0, 2] + rng.normal(0, 0.002), S[1, 2] + rng.normal(0, 0.002),
                     math.atan2(S[1, 0], S[0, 0]) + rng.normal(0, 0.002)))
        base.append((B[0, 2], B[1, 2], math.atan2(B[1, 0], B[0, 0])))
    r = extrinsics.hand_eye_se2(sens, base)
    assert r["x"] == pytest.approx(0.14, abs=0.004) and r["y"] == pytest.approx(-0.004, abs=0.004)
    assert math.degrees(r["yaw"]) == pytest.approx(0.8, abs=0.3)


def test_the_dock_tag_against_its_contacts_from_repeat_dockings():
    rng = np.random.default_rng(6)
    T_base_cam = frame("cam_left_optical")
    T_base_contacts = T(np.eye(3), (-0.2, 0.0, 0.05))                 # the robot's charging pads (CAD, est)
    T_tag_contacts = T(rodrigues([0.1, -0.05, 0.2]), (0.03, -0.25, 0.4))
    obs = []
    for _ in range(8):
        # contacts closed: the base sits within a few mm of the same place each time (the dock's funnel)
        jitter = T(rodrigues([0, 0, rng.normal(0, 0.004)]), (rng.normal(0, 0.002), rng.normal(0, 0.002), 0))
        T_base_contacts_k = T_base_contacts @ jitter
        T_cam_tag = inv(T_base_cam) @ T_base_contacts_k @ inv(T_tag_contacts)
        noise = T(rodrigues(rng.normal(0, 0.003, 3)), rng.normal(0, 0.0015, 3))
        obs.append((T_base_cam, T_cam_tag @ noise))
    r = extrinsics.dock_tag(obs, T_base_contacts)
    mm, deg = extrinsics.pose_error(r["T_tag_contacts"], T_tag_contacts)
    assert mm < 4 and deg < 0.6 and r["spread_mm"] < 10
    assert results.passes("dock", dict(spread_mm=r["spread_mm"], spread_deg=r["spread_deg"]))[0] == \
        (r["spread_mm"] <= 5.0 and r["spread_deg"] <= 1.0)


def test_online_refinement_moves_only_when_observable_and_never_past_its_bound():
    p = online.BoundedParam("cam_time_offset", prior=0.0, prior_sigma=1e-3, bound=3e-3, q_per_sqrt_s=1e-5,
                            min_info=1e8, window_s=5.0)
    rng = np.random.default_rng(7)
    truth = 1.2e-3
    t = 0.0
    for k in range(200):                      # driving straight: the offset doesn't show (H ~ 0)
        t += 0.05
        assert p.update(t, rng.normal(0, 1e-3), 0.0, [[1e-6]], [[1e-6]]) == "held"
    assert p.x[0] == 0.0
    for k in range(400):                      # spinning back and forth: rate errors show it (H = the yaw accel)
        t += 0.05
        H = 3.0 * math.sin(k * 0.3)
        p.update(t, H * truth + rng.normal(0, 1e-3), H * p.x[0], [[H]], [[1e-6]])
    assert p.x[0] == pytest.approx(truth, abs=0.2e-3) and not p.at_bound
    for k in range(100):                      # a knock: the data suddenly want 6 ms; refused, and flagged
        t += 0.05
        H = 3.0 * math.sin(k * 0.3)
        p.update(t, H * 6e-3, H * p.x[0], [[H]], [[1e-6]])
    assert p.x[0] == pytest.approx(truth, abs=0.3e-3) and p.inconsistent
    # an offset that really is past the bound: the estimate stops there and says so
    q = online.BoundedParam("x", prior=0.0, prior_sigma=5e-3, bound=3e-3, q_per_sqrt_s=1e-5, min_info=1e3)
    for k in range(200):
        H = 3.0 * math.sin(k * 0.3)
        q.update(0.05 * k, H * 6e-3, H * q.x[0], [[H]], [[1e-6]])
    assert q.x[0] == pytest.approx(3e-3) and q.at_bound


def test_results_store_keeps_every_run_and_exports_only_passed_frames(tmp_path):
    st = results.Store(tmp_path)
    fr = dict(child="laser", xyz=[0.141, 0.0, 0.131], quat=[0, 0, 0.004, 1.0])
    st.add("laser_base", "base_extrinsics", dict(frame=fr), dict(rms_m=0.004, yaw_mismatch_deg=0.2), t=1.0)
    st.add("laser_base", "base_extrinsics", dict(frame={**fr, "xyz": [0.2, 0, 0.131]}),
           dict(rms_m=0.05, yaw_mismatch_deg=0.2), t=2.0)                     # a bad run: kept, not used
    assert len(st.runs("laser_base")) == 2 and not st.latest("laser_base", passed_only=False)["passed"]
    assert st.frames()["laser"]["xyz"] == [0.141, 0.0, 0.131]
    st.add("stereo", "stereo", dict(rot=[0.0, 0.0, 0.0]), dict(rms_px=0.3, epipolar_px=0.1, sigma_rot_deg=0.01))
    st.add("stereo", "stereo", dict(rot=[0.0, 0.002, 0.0]), dict(rms_px=0.3, epipolar_px=0.1, sigma_rot_deg=0.01))
    assert st.drift("stereo", "rot") == pytest.approx(0.002)
