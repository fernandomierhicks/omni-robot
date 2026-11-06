"""The AI's find (color_vision.py) and the pick of one object (pick.TidyTask's only), without the sim or ROS:
synthetic scenes (ray cast boxes and balls, painted), and a saved sim render of the "colours" set
(simulation/v11 sil/studies/colour_scene.py --save; data/colours_stereo.npz: the stereo pair's depth with its matcher's
error and a 0.3 deg calibration error, the depth camera's colour).

    python -m pytest -q ros/src/omni_arm/test
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_arm import color_vision as cv  # noqa: E402
from omni_arm import floor_vision as fv  # noqa: E402
from omni_arm.kinematics import ArmKinematics  # noqa: E402
from omni_arm.pick import Inputs, Outputs, TidyTask  # noqa: E402

DATA = Path(__file__).parent / "data" / "colours_stereo.npz"
K, WH = (285.6, 285.6, 320.0, 200.0), (640, 400)                 # the left camera's 70 deg at 640 x 400
WOOD = (205, 175, 130)
PURPLE, RED, ORANGE = (140, 51, 191), (217, 38, 31), (242, 115, 26)


@pytest.fixture(scope="module")
def T():
    return TidyTask(ArmKinematics()).T_base_cam


def scene(T, boxes=(), balls=(), shade=None):
    """Depth and colour of the floor with boxes (x0, x1, y0, y1, h, rgb) and balls (x, y, r, rgb), ray cast. shade:
    (x_split, factor): the parts of the boxes beyond x_split darker (a shadow)."""
    W, H = WH
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], axis=-1) @ T[:3, :3].T
    o = T[:3, 3]
    s_best = np.where(rays[..., 2] < 0, -o[2] / np.minimum(rays[..., 2], -1e-9), np.inf)
    col = np.zeros((H, W, 3)) + WOOD
    for x0, x1, y0, y1, h, rgb in boxes:
        for axis, planes in ((2, (h,)), (0, (x0, x1)), (1, (y0, y1))):     # top and sides
            for c in planes:
                s = (c - o[axis]) / np.where(np.abs(rays[..., axis]) > 1e-9, rays[..., axis], 1e-9)
                p = o + rays * s[..., None]
                hit = (s > 0) & (p[..., 0] >= x0 - 1e-6) & (p[..., 0] <= x1 + 1e-6) & (p[..., 1] >= y0 - 1e-6) & \
                      (p[..., 1] <= y1 + 1e-6) & (p[..., 2] >= -1e-6) & (p[..., 2] <= h + 1e-6) & (s < s_best)
                s_best = np.where(hit, s, s_best)
                c_ = np.array(rgb, float) * (0.75 if axis != 2 else 1.0)    # the sides a little darker
                if shade is not None:
                    c_ = np.where((p[..., 0] > shade[0])[..., None], c_ * shade[1], c_)
                col = np.where(hit[..., None], c_, col)
    for x, y, r, rgb in balls:
        c = np.array([x, y, r])
        oc = o - c
        b = np.einsum("hwk,k->hw", rays, oc)
        a = np.einsum("hwk,hwk->hw", rays, rays)
        disc = b * b - a * (oc @ oc - r * r)
        s = np.where(disc > 0, (-b - np.sqrt(np.maximum(disc, 0))) / a, np.inf)
        hit = (s > 0) & (s < s_best)
        s_best = np.where(hit, s, s_best)
        col = np.where(hit[..., None], np.array(rgb, float), col)
    z = np.where(np.isfinite(s_best), s_best, np.nan).astype(np.float32)   # ray z = 1: the optical z is s
    return z, np.clip(col, 0, 255).astype(np.uint8)


def box(cx_, cy_, L, W_, h, rgb):
    return (cx_ - L / 2, cx_ + L / 2, cy_ - W_ / 2, cy_ + W_ / 2, h, rgb)


def found(T, z, rgb, color, shape, fmap=None):
    res, img = cv.find(fv.DepthFrame(0.0, z, K, T), rgb, (0.0, 0.0, 0.0), fmap or fv.FloorMap(), 0.0, color, shape)
    return res, img


def near(res, x, y, tol=0.04):
    return [c for c in res["candidates"] if math.hypot(c["x"] - x, c["y"] - y) < tol]


# -- colour ----------------------------------------------------------------------------------------------------------
def test_hsv_follows_opencvs_scale():
    px = np.array([[PURPLE, RED, (200, 30, 40), (255, 255, 255), (0, 0, 0)]], np.uint8)
    h, s, v = cv.hsv(px)
    assert 126 <= h[0, 0] <= 160 and s[0, 0] > 150                  # purple
    assert h[0, 1] < 8 and h[0, 2] > 170                            # red either side of 0
    assert s[0, 3] == 0 and v[0, 3] == 255 and v[0, 4] == 0
    hh, ss, vv = cv.hsv(np.array([[RED] * 3 + [(200, 30, 40)] * 2], np.uint8))
    assert cv.dominant(hh.ravel(), ss.ravel(), vv.ravel())[0] == "red"


def test_colour_names_and_explicit_ranges():
    assert cv.color_spec("Violet")["name"] == "purple"
    assert cv.color_spec(None) is None and cv.color_spec("any") is None
    s = cv.color_spec({"h": [170, 5], "s_min": 50})                  # a range across 0 wraps
    assert len(s["ranges"]) == 2
    with pytest.raises(ValueError):
        cv.color_spec("chartreuse-ish")


# -- synthetic scenes ------------------------------------------------------------------------------------------------
def test_the_purple_square_beats_the_purple_block_and_the_red_cube(T):
    z, rgb = scene(T, boxes=[box(0.55, 0.10, 0.04, 0.04, 0.04, PURPLE), box(0.60, -0.15, 0.06, 0.03, 0.03, PURPLE),
                             box(0.50, -0.02, 0.03, 0.03, 0.03, RED)])
    res, img = found(T, z, rgb, "purple", "square")
    top = res["candidates"][0]
    assert math.hypot(top["x"] - 0.55, top["y"] - 0.10) < 0.02 and top["score"] > 0.9
    assert top["shape"]["class"] == "square" and top["color"]["name"] == "purple"
    blk, red = near(res, 0.60, -0.15)[0], near(res, 0.50, -0.02)[0]
    assert blk["shape"]["class"] == "long" and 0.1 < blk["score"] < 0.3   # a near miss: listed, ranked lower
    assert red["score"] < 0.05
    assert img.shape == (WH[1], WH[0], 3) and (img != rgb).any()      # the annotated picture
    r2, _ = found(T, z, rgb, "purple", "round")
    assert all(c["score"] < 0.5 for c in r2["candidates"])           # no purple round thing: nothing to pick


def test_a_ball_is_round_and_a_crayon_long(T):
    z, rgb = scene(T, boxes=[box(0.62, 0.12, 0.09, 0.012, 0.012, (50, 180, 60))], balls=[(0.55, -0.08, 0.02, ORANGE)])
    res, _ = found(T, z, rgb, "orange", "round")
    top = res["candidates"][0]
    assert math.hypot(top["x"] - 0.55, top["y"] + 0.08) < 0.03 and top["score"] > 0.9
    res, _ = found(T, z, rgb, "green", "long")
    assert res["candidates"][0]["shape"]["class"] == "long" and res["candidates"][0]["score"] > 0.9


def test_a_shadow_keeps_its_colour_but_grey_is_not_purple(T):
    z, rgb = scene(T, boxes=[box(0.55, 0.0, 0.04, 0.04, 0.04, PURPLE)], shade=(0.55, 0.45))
    res, _ = found(T, z, rgb, "purple", "square")
    assert res["candidates"][0]["color"]["match"] > 0.9
    z, rgb = scene(T, boxes=[box(0.55, 0.0, 0.04, 0.04, 0.04, (130, 115, 140))])   # greyish lilac: saturation 0.18
    res, _ = found(T, z, rgb, "purple", "square")
    assert res["candidates"][0]["color"]["match"] < 0.1 and res["candidates"][0]["score"] < 0.1


def test_colour_with_no_depth_candidate_is_an_image_only_blob(T):
    z, rgb = scene(T, boxes=[box(0.55, 0.0, 0.04, 0.04, 0.04, RED)])
    W, H = WH
    rgb[30:60, 500:540] = PURPLE                                       # purple far up the image: no depth there
    z[30:60, 500:540] = np.nan
    res, img = found(T, z, rgb, "purple", "square")
    assert res["image_only"] and res["image_only"][0]["pickable"] is False
    assert res["image_only"][0]["bearing_deg"] < 0                   # to the right
    assert all(c["score"] < 0.1 for c in res["candidates"])
    assert res["image_only"][0]["flat"] is None                      # no depth: can't tell


def test_a_rug_is_flat_colour_not_something_to_seek(T):
    """The D435 at 7.5 deg sees the red living-room rug: a find for the red cube chased it (Nov 5)."""
    z, rgb = scene(T, boxes=[box(0.9, 0.0, 0.5, 0.6, 0.012, RED)])    # a 12 mm rug
    res, _ = found(T, z, rgb, "red", "square")
    rug = [b for b in res["image_only"] if b.get("flat")]
    assert rug and all(c["score"] < 0.5 for c in res["candidates"])
    z, rgb = scene(T, boxes=[box(0.9, 0.0, 0.3, 0.3, 0.08, RED)])     # a red box too big to pick: not flat
    res, _ = found(T, z, rgb, "red", "square")
    assert res["image_only"] and not any(b.get("flat") for b in res["image_only"])


def test_ids_last_from_find_to_find(T):
    z, rgb = scene(T, boxes=[box(0.55, 0.10, 0.04, 0.04, 0.04, PURPLE), box(0.50, -0.1, 0.03, 0.03, 0.03, RED)])
    m = fv.FloorMap()
    a, _ = found(T, z, rgb, "purple", "square", m)
    b, _ = found(T, z, rgb, "red", "square", m)
    ida = {round(c["y"], 1): c["id"] for c in a["candidates"]}
    idb = {round(c["y"], 1): c["id"] for c in b["candidates"]}
    assert ida == idb and len(ida) == 2 and len(m.tracked) == 2


def test_the_finder_answers_once_with_fresh_frames_or_an_error(T):
    z, rgb = scene(T, boxes=[box(0.55, 0.10, 0.04, 0.04, 0.04, PURPLE)])
    f = cv.Finder()
    assert f.request(dict(id="t-0001", color="chartreuse-ish"), 0.0)
    assert f.result["id"] == "t-0001" and "colour" in f.result["error"]
    assert f.request(dict(id="t-0002", color="purple", shape="square"), 10.0) == ""
    assert not f.due(10.1) and f.due(10.3) and not f.due(10.5)      # ask the cameras once settled ...
    assert f.due(12.4)                                               # ... and again while nothing fresh came
    old = fv.DepthFrame(9.9, z, K, T)                                 # taken before the request: not used
    assert not f.step(10.1, old, cv.ColorFrame(9.9, rgb), (0, 0, 0))
    assert f.step(10.5, fv.DepthFrame(10.4, z, K, T), cv.ColorFrame(10.4, rgb), (0, 0, 0))
    assert f.result["id"] == "t-0002" and f.result["candidates"][0]["shape"]["class"] == "square"
    assert f.image is not None and f.req is None
    f.request(dict(id="t-0003", color="red"), 20.0)
    assert f.step(29.0, None, None, None) and "no depth frame" in f.result["error"]


def test_a_color_image_of_another_size_is_resampled_to_the_depth(T):
    z, rgb = scene(T, boxes=[box(0.55, 0.0, 0.04, 0.04, 0.04, PURPLE)])
    small = rgb[::2, ::2]
    res, img = found(T, z, small, "purple", "square")
    assert res["candidates"][0]["color"]["match"] > 0.8 and img.shape[:2] == z.shape


# -- the saved sim render ----------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def render():
    d = np.load(DATA)
    z = np.where(d["depth_mm"] > 0, d["depth_mm"] / 1000.0, np.nan).astype(np.float32)
    truth = {str(n): tuple(p) for n, p in zip(d["names"], d["truth"])}
    return fv.DepthFrame(0.0, z, tuple(d["K"]), d["T"]), d["rgb"], truth


@pytest.mark.parametrize("color,shape,want", [("purple", "square", "cube40_purple"), ("purple", "long", "block"),
                                              ("red", "square", "cube30"), ("orange", "round", "ball40"),
                                              ("blue", "any", "cube40"), ("purple", "round", None)])
def test_the_sim_render_answers_each_question(render, color, shape, want):
    frame, rgb, truth = render
    res, _ = cv.find(frame, rgb, (0.0, 0.0, 0.0), fv.FloorMap(), 0.0, color, shape)
    top = res["candidates"][0] if res["candidates"] and res["candidates"][0]["score"] >= 0.5 else None
    who = None if top is None else min(truth, key=lambda n: math.hypot(truth[n][0] - top["x"], truth[n][1] - top["y"]))
    assert who == want, res["candidates"]
    assert len(res["candidates"]) == 6                               # each thing found once


# -- the AI's look pose ------------------------------------------------------------------------------------------------
def test_the_look_pose_points_the_wrist_camera_ahead_and_up():
    from omni_arm import motion as mo
    from omni_arm.pick import camera_view
    kin = ArmKinematics()
    P = TidyTask(kin).poses
    q = P["look"]
    assert np.all(q >= kin.cfg.ranges[:, 0]) and np.all(q <= kin.cfg.ranges[:, 1])    # inside every joint's range
    assert kin.clear(q)[0]
    v = camera_view(kin, q)
    assert abs(v["azimuth_deg"]) < 3 and 15 < v["elevation_deg"] < 30 and v["height_m"] > 0.5
    assert v["upside_down"]                                         # robot_look turns its image round
    via = [P[n] for n in ("high", "ready", "predrop") if n in P]
    assert mo.plan_checked(kin, P["stow"], q, via=via)[0] is not None
    assert mo.plan_checked(kin, q, P["stow"], via=via)[0] is not None
    assert not camera_view(kin, P["ready"])["upside_down"] or camera_view(kin, P["ready"])["elevation_deg"] < -45


# -- the pick of one object ----------------------------------------------------------------------------------------
def test_a_pick_of_one_goes_for_it_alone_or_ends_not_found(T):
    kin = ArmKinematics()
    z, rgb = scene(T, boxes=[box(0.55, 0.10, 0.04, 0.04, 0.04, PURPLE), box(0.50, -0.1, 0.03, 0.03, 0.03, RED)])
    m = fv.FloorMap()
    res, _ = found(T, z, rgb, "purple", "square", m)
    want = res["candidates"][0]["id"]

    def look(task, z_):
        task.state, task.t_req, task.origin = "LOOK", 0.0, (0.0, 0.0, 0.0)
        task.history.append((0.0, (0.0, 0.0, 0.0)))
        task._s_look(1.0, Inputs((0.0, 0.0, 0.0), None, fv.DepthFrame(1.0, z_, K, T)), Outputs(), np.zeros(6))

    t = TidyTask(kin)
    t.start(0.0, 1, 2.0, only=want, fmap=m)
    look(t, z)
    assert t.state == "APPROACH" and t.target.id == want
    t2 = TidyTask(kin)
    t2.start(0.0, 1, 2.0, only=99, fmap=m)
    look(t2, z)
    assert t2.state == "DONE" and t2.outcome == "not_found" and t2.status()["outcome"] == "not_found"
    floor, _ = scene(T)
    m.tracked[[o.id for o in m.tracked].index(want)].status = "gone"
    t3 = TidyTask(kin)
    t3.start(0.0, 1, 2.0, only=want, fmap=m)
    look(t3, floor)
    assert t3.state == "DONE" and t3.outcome == "lost"
    assert t3.status()["stages"] == dict(attempted=False, grasped=False, released=False)
