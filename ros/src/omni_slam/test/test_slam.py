"""omni_slam on a synthetic room (design.md 10): tags on four walls, a robot driving a square loop, its odometry off by
2% in scale and drifting in heading. The sim's house tour is simulation/v11 sil/studies/tag_slam.py.
Run: python -m pytest -q ros/src/omni_slam/test"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_slam.graph import se2_between, se2_compose  # noqa: E402
from omni_slam.health import HealthMonitor  # noqa: E402
from omni_slam.slam import TagSlam  # noqa: E402
from omni_slam.tags import Pinhole, TagDetection, rodrigues, tag_corners, tag_pose  # noqa: E402

CAM = Pinhole.from_fovy(70.0)
# the left camera on base_link, 30 deg up (robot_v11.json, rounded): body x forward -> optical z forward
R_BODY_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
TBC = np.eye(4)
TBC[:3, :3] = rodrigues([0, -math.radians(30), 0]) @ R_BODY_OPT
TBC[:3, 3] = (0.13, 0.095, 0.166)


def room_tags(half=2.5, z=1.0, size=0.16):
    """Two tags per wall of a 5 x 5 m room, facing in."""
    tags = []
    k = 0
    for nx, ny in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        n = np.array([nx, ny, 0.0])
        for s in (-0.8, 0.8):
            c = -n * half + np.array([-ny, nx, 0.0]) * s + np.array([0, 0, z + 0.3 * (k % 2)])
            zt = n
            xt = np.array([-zt[1], zt[0], 0.0])
            R = np.column_stack([xt, np.cross(zt, xt), zt])
            tags.append((k, R, c, size))
            k += 1
    return tags


def observe(tags, pose, t, rng, Tbc=TBC, sigma=0.3):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    Tmb = np.array([[c, -s, 0, pose[0]], [s, c, 0, pose[1]], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    Tmc = Tmb @ Tbc
    out = []
    for tid, R, p, size in tags:
        Pc = ((p + tag_corners(size) @ R.T) - Tmc[:3, 3]) @ Tmc[:3, :3]
        if np.any(Pc[:, 2] < 0.2):
            continue
        uv = CAM.project(Pc)
        if np.any(uv < 5) or np.any(uv[:, 0] > CAM.width - 5) or np.any(uv[:, 1] > CAM.height - 5):
            continue
        out.append(TagDetection(tid, uv + rng.normal(0, sigma, uv.shape), t, size))
    return out


def square_drive(n_laps=2, dt=0.05, v=0.5):
    """(t, true pose) around a 2.4 m square, turning at the corners."""
    pts = [(-1.2, -1.2), (1.2, -1.2), (1.2, 1.2), (-1.2, 1.2)]
    path, t = [], 0.0
    pose = np.array([-1.2, -1.2, 0.0])
    for _ in range(n_laps):
        for k in range(4):
            a, b = np.array(pts[k]), np.array(pts[(k + 1) % 4])
            yaw = math.atan2(*(b - a)[::-1])
            for th in np.linspace(pose[2], pose[2] + math.atan2(math.sin(yaw - pose[2]), math.cos(yaw - pose[2])), 20):
                t += dt
                path.append((t, np.array([a[0], a[1], th])))
            L = float(np.linalg.norm(b - a))
            for f in np.linspace(0, 1, int(L / (v * dt))):
                t += dt
                path.append((t, np.array([*(a + f * (b - a)), yaw])))
            pose = np.array([b[0], b[1], yaw])
    return path


def odometry(path, rng, scale=1.03, yaw_drift=math.radians(2.0)):
    """The EKF's odom: 3% long and a heading that drifts 2 deg every 10 s."""
    out, o = [], path[0][1].copy()
    out.append((path[0][0], o.copy()))
    for (t0, a), (t1, b) in zip(path, path[1:]):
        d = se2_between(a, b)
        d[:2] *= scale
        d[2] += yaw_drift * (t1 - t0) / 10 + rng.normal(0, 0.0005)
        o = se2_compose(o, d)
        out.append((t1, o.copy()))
    return out


def run_slam(ts, path, odo, tags, rng, every=2, lidar=None, extra=None):
    """Returns the position errors along the way: (odometry, slam) per step."""
    errs = []
    for k, ((t, truth), (_, o)) in enumerate(zip(path, odo)):
        ts.on_odom(t, o)
        if lidar is not None and k % 2 == 0:
            ts.on_lidar(t, truth + rng.normal(0, [0.01, 0.01, 0.003]), np.diag([0.015 ** 2, 0.015 ** 2, 0.005 ** 2]))
        if k % every == 0:
            dets = observe(tags, truth, t, rng, **(extra or {}))
            if dets:
                ts.on_tags(t, dets)
        est = ts.pose(t)
        if est is not None:
            errs.append((math.hypot(*(o[:2] - truth[:2])), math.hypot(*(est[:2] - truth[:2]))))
    ts.solve(20)
    return np.array(errs)


def test_a_tag_pose_and_why_one_is_refused():
    rng = np.random.default_rng(0)
    (tid, R, p, size), = room_tags()[:1]
    near = observe([(tid, R, p, size)], np.array([-1.5, 0.3, math.pi]), 0.0, rng)     # the tag is on x = -2.5
    assert near
    tp = tag_pose(CAM, near[0])
    true_range = float(np.linalg.norm(p - (np.array([-1.5, 0.3, 0.0]) + np.array([-0.13, -0.095, 0.166]))))
    assert tp.ok and not tp.ambiguous and abs(tp.range - true_range) < 0.02
    far = observe([(tid, R, p, size)], np.array([1.5, 0.0, math.pi]), 0.0, rng)
    assert far and tag_pose(CAM, far[0]).ambiguous                # 4 m: its mirror pose fits within the noise
    tiny = TagDetection(1, np.array([(600, 400), (610, 400), (610, 390), (600, 390.0)]), 0.0, 0.16)
    assert "small" in tag_pose(CAM, tiny).reasons and "far" in tag_pose(CAM, tiny).reasons


def test_unknown_tags_and_robot_poses_together_beat_the_odometry():
    rng = np.random.default_rng(1)
    tags = room_tags()
    path = square_drive()
    odo = odometry(path, rng)
    ts = TagSlam(CAM, TBC, anchor="first")
    e = run_slam(ts, path, odo, tags, rng)
    assert np.max(e[:, 0]) > 0.15 and np.percentile(e[:, 1], 95) < 0.06 and np.max(e[:, 1]) < 0.5 * np.max(e[:, 0])
    true_tags = {tid: p for tid, _, p, _ in tags}
    errs = [np.linalg.norm(v["t"] - true_tags[tid]) for tid, v in ts.tags().items()]
    assert len(errs) >= 6 and max(errs) < 0.06
    C = ts.pose_cov()
    assert 1e-4 < math.sqrt(C[0, 0]) < 0.1                        # honest: neither zero nor useless


def test_a_bad_detection_is_dropped_not_averaged_in():
    rng = np.random.default_rng(2)
    tags = room_tags()
    path = square_drive(1)
    odo = odometry(path, rng)
    ts = TagSlam(CAM, TBC, anchor="first")
    corrupted = False
    for k, ((t, truth), (_, o)) in enumerate(zip(path, odo)):
        ts.on_odom(t, o)
        if k % 2 == 0:
            dets = observe(tags, truth, t, rng)
            if k >= 200 and dets and not corrupted and ts.g.has_tag(dets[0].tag_id):
                dets[0].corners = dets[0].corners + 60.0          # a known tag, reported 60 px off
                corrupted = True
            if dets:
                ts.on_tags(t, dets)
    ts.solve(20)
    assert corrupted and ts.n_dropped >= 1 and ts.g.stats.tag_rms_px < 0.8


def test_the_camera_rotation_is_refined_only_where_the_lidar_fixes_the_map():
    rng = np.random.default_rng(3)
    tags = room_tags()
    path = square_drive()
    odo = odometry(path, rng)
    Tbc_bad = TBC.copy()
    err = np.radians([0.0, 0.4, 0.3])                               # the bar is off: pitch and yaw
    Tbc_bad[:3, :3] = rodrigues(-err) @ TBC[:3, :3]
    ts = TagSlam(CAM, Tbc_bad, anchor="lidar", refine_cam_deg=0.5)
    run_slam(ts, path, odo, tags, rng, lidar=True)
    cc = ts.g.cam_correction()
    assert cc["observable"] and not cc["at_bound"]
    assert np.allclose(cc["deg"], np.degrees(err), atol=0.08)
    # with nothing but the camera to fix the map it stays off: a camera rotation and a map rotation look alike
    ts2 = TagSlam(CAM, Tbc_bad, anchor="first", refine_cam_deg=0.5)
    assert not ts2.g.refine_cam


def test_a_saved_tag_map_relocalizes_a_kidnapped_robot(tmp_path):
    rng = np.random.default_rng(4)
    tags = room_tags()
    path = square_drive(1)
    odo = odometry(path, rng)
    ts = TagSlam(CAM, TBC, anchor="first")
    run_slam(ts, path, odo, tags, rng)
    ts.save_map(tmp_path / "house.tags.json")
    # a new session: the robot wakes up somewhere else, its odometry starts at zero
    ts2 = TagSlam(CAM, TBC, anchor="first")
    ts2.load_map(tmp_path / "house.tags.json")
    truth = np.array([0.6, -0.4, math.radians(200)])
    got = None
    for k in range(40):
        t = 100 + 0.1 * k
        pose = truth + np.array([0.0, 0.0, math.radians(1.5) * k])     # turning slowly in place
        ts2.on_odom(t, (0.0, 0.0, math.radians(1.5) * k))
        for d in observe(tags, pose, t, rng):
            v = ts2.vote(t, d)
            if v is not None:
                got = (v[0], pose)
                break
        if got:
            break
    assert got is not None
    (est, pose) = got
    assert math.hypot(*(est[:2] - pose[:2])) < 0.06 and abs(math.degrees(est[2] - pose[2]) % 360) < 2.0 or \
        abs(math.degrees(est[2] - pose[2]) % 360) > 358.0


def test_health_slows_the_robot_then_stops_it_and_asks_to_relocalize():
    h = HealthMonitor()
    h.on_fix(0.0)
    ok = h.assess(1.0, np.diag([0.02 ** 2, 0.02 ** 2, 0.01]))
    assert ok.level == "OK" and ok.speed_scale == 1.0 and ok.catch_ok
    worse = h.assess(2.0, np.diag([0.09 ** 2, 0.09 ** 2, 0.01]))
    assert worse.level == "DEGRADED" and worse.speed_scale == 0.6
    assert h.assess(7.0, np.diag([0.03 ** 2] * 2 + [0.01]), moving=True).level == "DEGRADED"   # no fix for 7 s
    for k in range(3):                                   # three tag views that agree with each other, 0.5 m away
        h.on_tag_pose((0, 0, 0), np.diag([0.03 ** 2, 0.03 ** 2, 0.01]), (0.5, 0.0, 0.0), np.diag([0.02 ** 2] * 3))
    r = h.assess(8.0, np.diag([0.03 ** 2, 0.03 ** 2, 0.01]))
    assert r.relocalize == "tags" and r.speed_scale <= 0.3
    lost = h.assess(12.0, np.diag([0.2 ** 2, 0.2 ** 2, 0.01]))
    assert lost.level == "LOST" and lost.speed_scale == 0.0
    h.relocalized(12.5)
    assert h.assess(13.0, np.diag([0.02 ** 2] * 2 + [0.01])).level == "OK"
    assert not h.assess(13.5, np.diag([0.02 ** 2] * 2 + [0.01]), clock_mode="echo").catch_ok
