"""people_vision (design.md 4.12): the YOLOX decode, the camera's geometry, the lidar + camera fusion and the tracker,
ROS-free; the detector itself on a rendered sim frame when onnxruntime and a model are there
(ros/tools/fetch_models.py yolox_tiny).
    python -m pytest -q ros/src/omni_perception/test
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "omni_console"))

from omni_perception import people_vision as pv  # noqa: E402

PUPIL = (0.21, 0.0, 0.104)          # the IMX662 on the nose (simulation/v11 sil/imx662_cam.Mount), 20 deg up
TILT = 20.0
W, H = 960, 540
FOVY = 58.4


def camera(w=W, h=H, pupil=PUPIL, tilt=TILT) -> pv.Camera:
    t = math.radians(tilt)
    fwd = np.array([math.cos(t), 0.0, math.sin(t)])
    right = np.array([0.0, -1.0, 0.0])
    down = -np.cross(right, fwd)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([right, down, fwd])                  # optical x right, y down, z along the view
    T[:3, 3] = pupil
    f = h / 2 / math.tan(math.radians(FOVY) / 2)
    return pv.Camera((f, f, w / 2, h / 2), w, h, T)


def person_box(cam, x, y, height=1.7, half_w=0.22, score=0.9) -> pv.Box:
    """the box a detector would draw round an upright person at (x, y): the projected corners of their extent, clipped
    to the image"""
    us, vs = [], []
    a = math.atan2(y, x)
    side = np.array([-math.sin(a), math.cos(a)]) * half_w
    for z in (0.0, height):
        for s in (-1, 1):
            u, v = cam.project((x + s * side[0], y + s * side[1], z))
            us.append(u)
            vs.append(v)
    return pv.Box(max(0.0, min(us)), max(0.0, min(vs)), min(cam.width - 1.0, max(us)), min(cam.height - 1.0, max(vs)),
                  score)


# -- the detector's output -----------------------------------------------------------------------------------------------

def test_decode_turns_the_grid_output_into_person_boxes_and_suppresses_duplicates():
    size = 416
    g, st = pv.grids(size)
    out = np.zeros((len(g), 85), np.float32)
    out[:, 2:4] = math.log(1.0)                                      # 1 stride wide: tiny, but under the threshold
    k = 20 * 52 + 30                                                 # stride 8: cell (30, 20)
    out[k, :4] = (0.5, 0.25, math.log(40 / 8), math.log(80 / 8))
    out[k, 4], out[k, 5] = 0.9, 0.9                                  # objectness, person
    out[k + 1, :4] = (-0.3, 0.25, math.log(40 / 8), math.log(80 / 8))   # the same person from the next cell
    out[k + 1, 4], out[k + 1, 5] = 0.8, 0.8
    j = 5 * 52 + 5                                                   # a confident "bicycle" (class 1): not a person
    out[j, 4], out[j, 6] = 0.95, 0.95
    boxes = pv.decode(out, size, scale=0.5, min_score=0.35)
    assert len(boxes) == 1
    b = boxes[0]
    cx, cy = (30 + 0.5) * 8 / 0.5, (20 + 0.25) * 8 / 0.5            # back to the original image's pixels
    assert b.x0 == pytest.approx(cx - 40, abs=1e-3) and b.x1 == pytest.approx(cx + 40, abs=1e-3)
    assert b.y0 == pytest.approx(cy - 80, abs=1e-3) and b.y1 == pytest.approx(cy + 80, abs=1e-3)
    assert b.score == pytest.approx(0.81, abs=1e-4)
    assert pv.decode(np.zeros_like(out), size, 1.0) == []
    with pytest.raises(ValueError):
        pv.decode(out[:100], size, 1.0)


def test_letterbox_keeps_the_frame_at_the_top_left_in_bgr():
    pytest.importorskip("cv2")
    rgb = np.zeros((540, 960, 3), np.uint8)
    rgb[..., 0] = 200                                                # red
    img, r = pv.letterbox(rgb, 416)
    assert r == pytest.approx(416 / 960)
    assert img.shape == (416, 416, 3)
    assert img[10, 10, 2] == pytest.approx(200) and img[10, 10, 0] == 0   # BGR: red is the last channel
    assert img[400, 10, 0] == 114                                    # padding under the frame


# -- the camera's geometry -----------------------------------------------------------------------------------------------

def test_a_box_gives_the_persons_bearing_and_its_foot_point_gives_the_range():
    cam = camera()
    for x, y in ((2.5, -0.4), (4.0, 1.2), (1.6, 0.3)):
        b = person_box(cam, x, y)
        s = pv.sight(cam, b)
        a = math.atan2(y, x)
        assert s.az[1] <= a <= s.az[0]
        assert s.how == ("foot" if x < 1.7 else "both")              # the head leaves the frame's top under ~1.5 m
        r = math.hypot(x, y)
        assert math.hypot(s.xy[0] - x, s.xy[1] - y) < max(0.1, 2.0 * s.sigma)
        assert s.sigma < 0.4 * r                                     # the lidar's ~5 cm is far better
    p = cam.floor_point(*cam.project((3.0, 0.5, 0.0)))
    assert p == pytest.approx((3.0, 0.5), abs=1e-6)
    assert cam.floor_point(W / 2, 0) is None                         # above the horizon


def test_camera_ranges_get_rougher_with_distance_and_close_people_give_a_bearing_only():
    cam = camera()
    near, far = pv.sight(cam, person_box(cam, 1.8, 0.0)), pv.sight(cam, person_box(cam, 5.0, 0.0))
    assert near.sigma < 0.3 < far.sigma < 1.2                        # the lidar's range is better past ~2 m
    b = person_box(cam, 0.75, 0.0, height=1.6)                       # cut off at the top and the bottom
    assert b.y0 <= 0.0 and b.y1 >= H - 1.5
    s = pv.sight(cam, b)
    assert s.xy is None and s.az[1] <= 0.0 <= s.az[0]
    kid = pv.sight(cam, person_box(cam, 2.0, 0.0, height=1.1))       # a child: the head-top range reads too far
    assert math.hypot(*kid.xy) > 2.0


def test_sees_says_whether_a_person_sized_thing_is_in_view():
    cam = camera()
    assert cam.sees(2.5, 0.0)
    assert not cam.sees(0.0, 2.0)                                    # beside the robot
    assert not cam.sees(-2.0, 0.0)                                   # behind
    assert not cam.sees(0.5, 0.0)                                    # too close: its legs under the frame


# -- the lidar and the camera together -----------------------------------------------------------------------------------

def test_fuse_confirms_the_lidar_person_the_camera_sees_and_marks_the_chair_it_looked_at():
    cam = camera()
    person, chair, behind = (2.5, -0.4), (2.0, 0.8), (-1.0, 1.5)
    s = pv.sight(cam, person_box(cam, *person))
    obs = pv.fuse([s], [chair, person, behind], cam)
    by = {(round(o.x, 2), round(o.y, 2)): o for o in obs}
    p = by[person]
    assert p.lidar and p.camera and p.score == pytest.approx(0.9)
    assert by[chair].lidar and not by[chair].camera and by[chair].looked
    assert not by[behind].looked                                     # out of the camera's view: no evidence


def test_fuse_keeps_a_person_the_lidar_missed_and_pairs_by_range_along_one_bearing():
    cam = camera()
    s = pv.sight(cam, person_box(cam, 3.0, 0.0))
    obs = pv.fuse([s], [], cam)                                      # legs hidden (behind a sofa)
    assert len(obs) == 1 and obs[0].camera and not obs[0].lidar and obs[0].how == "both"
    assert math.hypot(obs[0].x - 3.0, obs[0].y) < 2.0 * obs[0].sigma
    obs = pv.fuse([s], [(1.2, 0.0), (3.05, 0.02)], cam)             # a table leg nearer on the same bearing
    p = [o for o in obs if o.camera][0]
    assert p.lidar and p.x == pytest.approx(3.05)


def test_move_points_follows_the_robot_between_the_frame_and_the_scan():
    src, dst = (1.0, 2.0, 0.0), (1.0, 2.0, math.radians(30))         # turned 30 deg left in between
    (x, y), = pv.move_points([(2.0, 0.0)], src, dst)
    assert math.atan2(y, x) == pytest.approx(math.radians(-30))
    assert math.hypot(x, y) == pytest.approx(2.0)
    back = pv.move_points([(x, y)], dst, src)
    assert back[0] == pytest.approx((2.0, 0.0))


# -- tracks --------------------------------------------------------------------------------------------------------------

def test_the_camera_makes_people_the_lidar_alone_never_does_and_a_looked_at_chair_drops_out():
    cam = camera()
    tr = pv.PeopleTracker()
    person, chair, side = (2.5, -0.4), (2.0, 0.8), (0.2, 2.0)
    pose = (0.0, 0.0, 0.0)
    for k in range(40):                                               # 4 s at 10 Hz, a camera frame every other scan
        t = 0.1 * k
        legs = [person, chair, side]
        if k % 2 == 0:
            obs = pv.fuse([pv.sight(cam, person_box(cam, *person))], legs, cam)
        else:
            obs = [pv.Observation(x, y, True, False) for x, y in legs]
        tr.update(t, obs, pose)
    rep = {(round(d["base_x"], 1), round(d["base_y"], 1)): d for d in tr.report(4.0, pose)}
    assert rep[(2.5, -0.4)]["status"] == "person" and rep[(2.5, -0.4)]["camera"]
    assert rep[(2.0, 0.8)]["status"] == "not_person"
    assert rep[(0.2, 2.0)]["status"] == "candidate"                  # beside the robot: never looked at, never confirmed
    assert rep[(0.2, 2.0)]["p"] <= 0.6 + 1e-6
    assert [d["status"] for d in tr.report(4.0, pose)][0] == "person"   # people first


def test_two_people_crossing_keep_their_ids():
    tr = pv.PeopleTracker()
    pose = (0.0, 0.0, 0.0)
    for k in range(31):
        t = 0.1 * k
        a = (2.0, -1.5 + 1.0 * t)                                    # walking left at 1 m/s
        b = (2.9, 1.5 - 1.0 * t)                                     # walking right, 0.9 m further out
        tracks = tr.update(t, [pv.Observation(*a, True, False), pv.Observation(*b, True, False)], pose)
        if k == 0:
            ida = min(tracks, key=lambda q: q.x).id
        ta = [q for q in tracks if q.id == ida]
        assert ta and math.hypot(ta[0].x - a[0], ta[0].y - a[1]) < 0.3, k
    assert len(tr.tracks) == 2


def test_a_track_ends_when_lost_and_a_confirmed_person_coasts_longer():
    tr = pv.PeopleTracker(lost_s=2.0, keep_s=4.0)
    tr.update(0.0, [pv.Observation(2.0, 0.0, True, True, 0.9, "lidar"), pv.Observation(0.0, 2.0, True, False)],
              (0.0, 0.0, 0.0))
    tr.update(2.5, [], (0.0, 0.0, 0.0))
    assert [tr.status(q) for q in tr.tracks] == ["person"]
    tr.update(4.5, [], (0.0, 0.0, 0.0))
    assert tr.tracks == []


def test_tracks_live_in_the_fixed_frame_while_the_robot_turns():
    tr = pv.PeopleTracker()
    for k in range(10):
        yaw = math.radians(9 * k)                                    # turning in place, the person still at (2, 0)
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c * 2.0, -s * 2.0                                   # (2, 0) in the turned base frame
        tr.update(0.1 * k, [pv.Observation(bx, by, True, False)], (0.0, 0.0, yaw))
    assert len(tr.tracks) == 1
    q = tr.tracks[0]
    assert math.hypot(q.x - 2.0, q.y) < 0.02
    d = tr.report(0.9, (0.0, 0.0, math.radians(81)))[0]
    assert d["bearing_deg"] == pytest.approx(-81, abs=1)


def test_follow_takes_confirmed_people_first_and_never_a_rejected_chair():
    from omni_console.people import Follower, from_tracks
    people = [dict(status="candidate", base_x=1.0, base_y=0.0, range=1.0, seen_s=0.1),
              dict(status="not_person", base_x=0.8, base_y=0.1, range=0.8, seen_s=0.0),
              dict(status="person", base_x=2.0, base_y=0.3, range=2.0, seen_s=0.1),
              dict(status="person", base_x=1.5, base_y=-0.2, range=1.5, seen_s=0.9)]   # stale
    pts = from_tracks(people)
    assert pts == [(2.0, 0.3), (1.0, 0.0)]
    f = Follower("follow", 1.0, 0.6)
    f.tick(0.0, pts, (0.0, 0.0, 0.0))
    assert f.target == pytest.approx((2.0, 0.3))                     # the camera's person, not the nearer candidate


# -- the detector on a rendered frame ------------------------------------------------------------------------------------

def _model(name="yolox_tiny"):
    pytest.importorskip("onnxruntime")
    pytest.importorskip("cv2")
    d = Path(os.environ.get("OMNI_MODEL_DIR") or Path.home() / ".omni_robot" / "models")
    p = d / f"{name}.onnx"
    if not p.exists():
        pytest.skip(f"no {p}: python ros/tools/fetch_models.py {name}")
    return p


@pytest.mark.parametrize("name", ["yolox_tiny", "yolox_nano"])
def test_the_detector_finds_the_sim_people_at_3_and_5_m_with_their_ranges(name):
    """data/sim_people_960.jpg: a v11 render from the IMX662's pose (house_scene people on a floor, MuJoCo, shrunk from
    1920 x 1080): people at (1.5, 0.6) (head cut off by the frame's top), (3.0, -0.8) and (5.0, 0.5) m."""
    from PIL import Image
    det = pv.PersonDetector(str(_model(name)))
    img = np.asarray(Image.open(HERE / "data" / "sim_people_960.jpg").convert("RGB"))
    cam = camera(img.shape[1], img.shape[0])
    sights = [pv.sight(cam, b) for b in det.detect(img)]
    for x, y in ((3.0, -0.8), (5.0, 0.5)):
        a, r = math.atan2(y, x), math.hypot(x, y)
        hit = [s for s in sights if s.az[1] - 0.03 <= a <= s.az[0] + 0.03]
        assert hit, (name, x, y)
        assert abs(math.hypot(*hit[0].xy) - r) < max(0.15 * r, 2.0 * hit[0].sigma), (name, hit[0])
    assert len(sights) <= 3                                          # no furniture called a person


def test_a_box_floating_above_the_floor_needs_a_higher_score():
    cam = camera()
    standing = person_box(cam, 3.0, 0.0, score=0.4)                  # feet on the floor in view
    assert pv.plausible(cam, standing)
    u0, v0 = cam.project((3.0, 0.3, 1.3))                            # a sign on a wall, 1.0-1.3 m up
    u1, v1 = cam.project((3.0, 0.0, 1.0))
    sign = pv.Box(min(u0, u1), min(v0, v1), max(u0, u1), max(v0, v1), 0.4)
    assert not pv.plausible(cam, sign) and pv.plausible(cam, pv.Box(sign.x0, sign.y0, sign.x1, sign.y1, 0.6))
    near = pv.Box(100, 50, 400, H - 1, 0.36)                         # cut off by the frame's bottom: kept
    assert pv.plausible(cam, near)
