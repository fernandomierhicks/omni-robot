"""Tests for the RealSense recording and replay (record_rs.py, rs_detect.py, rs_analyze.py, rs_view.py), no camera: the synthetic
rig (synth_rs.py) renders colour / depth / IR with known truth through the recorder's own code.

  & $py -m pytest -q test_rs.py          (in characterization/throw_track; ~1-2 min)
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import arcfit as F                # noqa: E402
import record_rs as R             # noqa: E402
import rs_analyze as A            # noqa: E402
import rs_detect as D             # noqa: E402
import rs_predict as P            # noqa: E402
import session as S               # noqa: E402
import synth as Y                 # noqa: E402
import synth_rs as X              # noqa: E402


@pytest.fixture(scope="module")
def recorded(tmp_path_factory):
    out = tmp_path_factory.mktemp("rs")
    R.main(["--synthetic", "--count", "2", "--name", "dry", "--out", str(out)])
    d = next(out.iterdir())
    return d, S.load_session(d)


def test_recorder_writes_every_stream_on_one_clock(recorded):
    d, sess = recorded
    dirs = S.throw_dirs(d)
    assert len(dirs) == 2                                                   # --count is honoured
    assert sess["rs"]["depth_scale"] == 0.001 and sess["camera"]["f_px"] > 0
    assert sess["rs"]["streams"]["color"]["intrinsics"]["fx"] == sess["camera"]["f_px"]
    for name in ("color", "depth", "ir1", "ir2"):
        assert (d / f"setup_{name}.npy").exists() and (d / f"end_{name}.npy").exists()
    for p in dirs:
        th = S.load_throw(p, sess)
        assert th.names == ["color", "depth", "ir1", "ir2"]
        assert th.frames.shape[1:] == (480, 848, 3) and th.stream("depth").frames.dtype == np.uint16
        for name, fps in (("color", 60), ("depth", 90), ("ir1", 90), ("ir2", 90)):
            st = th.stream(name).stamps if name != "color" else th.stamps
            assert st.shape[1] == 3 and np.all(np.diff(st[:, 0]) > 0)       # monotonic stamps
            s = S.stream_stats(st, th.t0_ns)
            assert abs(s["fps"] - fps) < 0.03 * fps and s["lost"] == 0
            assert abs(s["last_s"] - s["first_s"] - 2.0) < 0.12              # pre + post, all streams
            assert -0.02 < s["first_s"] < 0.03                                # the same time axis
            t = th.t_of(name)
            assert abs(t[0]) < 0.03 and 1.9 < t[-1] < 2.1
        assert th.meta["streams"]["depth"]["n"] == len(th.stream("depth").frames)
        assert 0.6 < th.meta["t_trigger_s"] < 0.8 and th.meta["live_blobs"]


def test_analysis_finds_the_ball_in_every_imager_and_they_agree(tmp_path):
    cfg = X.RSSynthConfig(seed=4)
    arc, T = Y.default_throw(cfg.color_cfg(), 4.5, seed=4)
    out, truth = X.render_streams(cfg, arc, T * 1.02, pre=0.8, seed=4)
    d = tmp_path / "s"
    d.mkdir()
    a = argparse.Namespace(size="848x480", fps=60.0, depth_fps=90.0, synthetic_pitch=0.0, distance=4.5, synthetic_gap=1.0)
    calib = R.SyntheticRig(a, ["color", "depth", "ir1", "ir2"], 2.0).start()
    f = calib["streams"]["color"]["intrinsics"]["fy"]
    sess = dict(camera=dict(name="synthetic", width=848, height=480, fps=60, fovy_deg=58.0, f_px=f), rs=calib,
                detector={"hue_deg": [40, 90]}, basket=dict(rim_radius=0.165))
    S.save_session(d, sess)
    base = 10_000_000_000
    streams = {}
    for name, (frames, ts) in out.items():
        streams[name] = dict(frames=frames, stamps=np.column_stack([base + (ts * 1e9).astype(np.int64), base + (ts * 1e9).astype(np.int64) + 8_000_000,
                                                                     np.arange(len(ts)) + 1]))
    p = S.save_throw(d, 1, None, None, dict(kind="realsense"), streams=streams)
    th = S.load_throw(p, sess)
    res = A.analyze_throw(th, sess, D.DetectCfg(), colour=True, session_dir=d)

    # the truth in the RealSense optical frame (x right, y down, z forward); synth's camera looks along -z with y up. Its time
    # axis is the render's (arc t0 = 0, stamps = mid-exposure); the throw's clock starts at the first colour stamp.
    tr_p = np.column_stack([truth["pos"][:, 0], -truth["pos"][:, 1], -truth["pos"][:, 2]])
    tr_t = truth["t"] + 0.8 - cfg.exposure / 2
    for src, key, tol in (("depth", "depth", 0.01), ("ir_stereo", "ir_stereo", 0.03)):
        s = res["sources"][src]
        assert s["ok"] and s["detections"] > 30, (src, s)
        assert abs(s["a_vs_gravity_pct"]) < 8 and s["rms_m"] < 0.03 and s["drag_arc"]
        assert 4.2 < s["first_range_m"] < 4.7                               # the throw starts 4.5 m out
        fit = res["_keep"][key][2]
        m = (tr_t >= s["first_t"]) & (tr_t <= s["last_t"])
        e = np.linalg.norm(fit.at(tr_t[m]) - tr_p[m], axis=1)
        assert e.mean() < tol and e.max() < 3 * tol, (src, e.mean(), e.max())      # against the injected truth
    assert res["depth_vs_ir_stereo"]["rms_m"] < 0.04
    s = res["sources"]["colour"]                                            # colour pixels + depth at the pixel (the scene's static box dropped)
    assert s["ok"] and s["flight_frames"] >= 18 and s["range_from_depth"] >= 0.8 * s["flight_frames"] and s["rms_px"] < 1.5
    assert abs(s["size_over_depth"] - 1) < 0.1
    fit = res["_keep"]["colour"][2]
    m = (tr_t >= s["first_t"]) & (tr_t <= s["last_t"])
    e = np.linalg.norm(fit.at(tr_t[m]) - tr_p[m], axis=1)
    assert e.mean() < 0.015 and e.max() < 0.04, (e.mean(), e.max())
    col = res["colour"]
    assert col["frames_with_a_blob"] > 100 and col["rms_px_at_zero_lag"] < 6.0
    assert abs(col["best_lag_ms"]) <= 8.0                                    # the colour and depth clocks agree to a few ms


def test_arc_fit_picks_the_ball_among_wrong_blobs():
    rng = np.random.default_rng(3)
    t = np.arange(0, 1.2, 1 / 90)
    p0, v0, g = np.array([0.2, -0.5, 4.5]), np.array([-0.1, -1.5, -5.0]), np.array([0.0, 9.81, 0.0])   # y down: gravity +y
    ball = p0 + np.outer(t, v0) + 0.5 * np.outer(t ** 2, g) + rng.normal(0, 0.01, (len(t), 3))
    junk_t = rng.choice(t, 150)
    junk = np.column_stack([rng.uniform(-2, 2, 150), rng.uniform(-1, 1, 150), rng.uniform(1, 6, 150)])
    fit = D.fit_arc(np.concatenate([t, junk_t]), np.vstack([ball, junk]), D.DetectCfg())
    assert fit is not None and abs(np.linalg.norm(fit.a) - 9.81) < 0.6
    assert fit.inlier[:len(t)].mean() > 0.9 and fit.inlier[len(t):].mean() < 0.05
    assert np.abs(fit.at(t) - ball).max() < 0.05


def test_stereo_range_from_disparity():
    intr = D.Intr(848, 480, 440.0, 440.0, 424.0, 240.0)
    z, B = 4.0, 0.05
    d = 440.0 * B / z
    c1 = [D.Cand(500.0, 200.0, float("nan"), 3.0, 28, 0.0)]
    c2 = [D.Cand(500.0 - d, 200.4, float("nan"), 3.1, 30, 0.0), D.Cand(100.0, 200.0, float("nan"), 3.0, 28, 0.0)]
    out = D.stereo_match(c1, c2, intr, B, D.DetectCfg())
    assert len(out) == 1 and abs(out[0].z - z) < 1e-6
    assert D.stereo_match(c1, [D.Cand(500 - d, 230.0, float("nan"), 3.0, 28, 0.0)], intr, B, D.DetectCfg()) == []      # a different row


def test_stream_stats_counts_lost_frames_and_ring_trims():
    st = np.column_stack([np.arange(10) * 11_111_111, np.arange(10), [1, 2, 3, 6, 7, 8, 9, 10, 11, 12]]).astype(np.int64)
    s = S.stream_stats(st)
    assert s["lost"] == 2 and abs(s["fps"] - 90) < 0.5 and s["n"] == 10
    ring = R.Ring(0.1)
    for i in range(50):
        ring.push(R.Item(i * 10_000_000, 0, i + 1, np.zeros(1), (0, 0, 0)))
    assert ring.q[0].t_ns >= 49 * 10_000_000 - 100_000_000 and ring.health()["lost"] == 0
    assert len(ring.window(400_000_000, 450_000_000)) == 6


def test_extrinsics_matrix_is_column_major():
    R_, t = D.extrinsics_matrix(dict(R=[1, 0, 0, 0, 0, 1, 0, -1, 0], t=[0.015, 0, 0]))   # column-major: columns (1,0,0), (0,0,1), (0,-1,0)
    assert np.allclose(R_ @ [0, 1, 0], [0, 0, 1]) and t[0] == 0.015


def test_bucket_pose_from_the_cad_numbers():
    b = P.Bucket.from_pose(0.0, 0.8, 0.25, 0.0)                                  # level camera, rim 0.8 m ahead and 0.25 m up
    assert np.allclose(b.rim_center, [0, -0.25, 0.8]) and np.allclose(b.up, [0, -1, 0])
    b = P.Bucket.from_pose(0.05, 0.5, 0.3, 30.0)                                 # looking up 30 deg
    assert np.allclose(b.rim_center, [0.05, 0.5 * 0.5 - 0.866025 * 0.3, 0.866025 * 0.5 + 0.5 * 0.3], atol=1e-5)
    assert np.allclose(b.up, [0, -0.866025, 0.5], atol=1e-5) and abs(np.linalg.norm(b.up) - 1) < 1e-9
    # a point straight above the rim centre is on the axis: no radial miss
    assert b.radial(b.rim_center + 0.4 * b.up) < 1e-9 and abs(b.radial(b.rim_center + np.array([0.1, 0, 0])) - 0.1) < 1e-9
    assert b.verdict(0.05) == "in" and b.verdict(0.165) == "rim" and b.verdict(0.30) == "out"


def _throw_on_disk(d, cfg, aim_x, seed, k):
    """A synthetic throw aimed at (aim_x, 0.25 up, 0.8 ahead) of the camera, on disk as record_rs.py writes it."""
    ccfg = cfg.color_cfg()
    rng = np.random.default_rng(seed)
    start = np.array([rng.uniform(-0.3, 0.3), -0.6, -4.5])
    arc = Y.aimed_arc(ccfg, start, np.array([aim_x, 0.25, -0.8]), 1.1)
    out, truth = X.render_streams(cfg, arc, 1.12, pre=0.8, seed=seed)
    base = 10_000_000_000
    streams = {n: dict(frames=f, stamps=np.column_stack([base + (ts * 1e9).astype(np.int64)] * 2 + [np.arange(len(ts)) + 1])) for n, (f, ts) in out.items()}
    return S.save_throw(d, k, None, None, dict(kind="realsense"), streams=streams), truth


def test_bucket_verdict_and_time_to_correct_on_synthetic_throws(tmp_path):
    cfg = X.RSSynthConfig(seed=2)
    d = tmp_path / "s"
    d.mkdir()
    a = argparse.Namespace(size="848x480", fps=60.0, depth_fps=90.0, synthetic_pitch=0.0, distance=4.5, synthetic_gap=1.0)
    calib = R.SyntheticRig(a, ["color", "depth", "ir1", "ir2"], 2.0).start()
    sess = dict(camera=dict(name="synthetic", width=848, height=480, fps=60, fovy_deg=58.0, f_px=calib["streams"]["color"]["intrinsics"]["fy"]), rs=calib,
                detector={"hue_deg": [40, 90]}, basket={})
    S.save_session(d, sess)
    bucket = P.Bucket.from_pose(0.0, 0.8, 0.25, 0.0)                              # the rim the throws are aimed through
    for k, (aim_x, want) in enumerate(((0.0, "in"), (0.34, "out")), 1):
        p, truth = _throw_on_disk(d, cfg, aim_x, 5 + int(aim_x * 100), k)
        th = S.load_throw(p, sess)
        res = A.analyze_throw(th, sess, D.DetectCfg(), colour=True, session_dir=d)
        A.add_predictions(res, bucket, D.DetectCfg())
        pc = res["prediction"].get("colour")                                         # the colour source's own verdict (pixels + depth at them)
        assert pc and pc["ok"] and pc["reference"]["verdict"] == want, pc
        pr = res["prediction"]["depth"]
        assert pr["ok"], pr
        ref = pr["reference"]
        # the truth: where the injected arc crosses the rim plane (height 0.25 above the lens, going down), radial miss from the axis
        pos = truth["pos"]
        i = int(np.nonzero((pos[:-1, 1] > 0.25) & (pos[1:, 1] <= 0.25))[0][0])
        d_true = float(np.hypot(pos[i, 0] - 0.0, pos[i, 2] + 0.8))
        assert abs(ref["radial_m"] - d_true) < 0.03, (ref["radial_m"], d_true)
        assert ref["verdict"] == want
        assert pr["settled"] and pr["time_to_correct_ttg"] is not None and pr["time_to_correct_ttg"] > 0.15, pr.get("time_to_correct_ttg")
        assert pr["first_prediction_ttg"] > 0.4                                      # a call is available well before the crossing
        for s in pr["series"][-3:]:                                                  # and it converges on the reference crossing
            assert s["error_m"] < 0.05
        assert "reference" in P.describe(pr)


def test_review_page_data_and_labels(tmp_path):
    import rs_review as V
    cfg = X.RSSynthConfig(seed=2)
    d = tmp_path / "s"
    d.mkdir()
    a = argparse.Namespace(size="848x480", fps=60.0, depth_fps=90.0, synthetic_pitch=0.0, distance=4.5, synthetic_gap=1.0)
    calib = R.SyntheticRig(a, ["color", "depth", "ir1", "ir2"], 2.0).start()
    S.save_session(d, dict(camera=dict(name="synthetic", f_px=calib["streams"]["color"]["intrinsics"]["fy"]), rs=calib, detector={"hue_deg": [40, 90]}))
    _throw_on_disk(d, cfg, 0.0, 5, 1)
    bj = tmp_path / "bucket.json"
    bj.write_text(json.dumps(dict(pose=dict(dx=0.0, dy=0.8, dz=0.25, pitch_deg=0.0), rim_radius_m=0.165)))
    rv = V.Review(d, bj, D.DetectCfg())
    data = rv.data(1)
    assert data["n"] == len(data["t"]) == len(data["blobs"])
    c = data["sources"]["colour"]
    assert sum(1 for p in c["picks"] if p) >= 18 and c["reference"]["verdict"] == "in" and c["arc"]["uv"]
    assert any(b[3] for fr in data["blobs"] for b in fr)                          # the scene's static box is marked static
    assert rv.jpeg(1, 10)[:2] == b"\xff\xd8"
    rv.save_review(1, dict(call="in", notes="swish", clicks=[dict(frame=3, u=1.0, v=2.0)], wrong=[4]))
    assert rv.data(1)["review"]["call"] == "in" and rv.index()[0]["call"] == "in"
    json.dumps(data, default=float)                                                # the page's JSON


def _pinhole_flight(n=50, fps=60.0):
    """A ballistic arc toward the camera (depth frame = colour frame here), its pixels and ranges: (t, uv, z, ic, up, ball)."""
    ball = F.Ball(radius=0.0803, mass=0.073, drag_cd=0.5)
    ic = D.Intr(848, 480, 616.0, 616.0, 424.0, 240.0)
    up = np.array([0.0, -np.cos(np.radians(30)), np.sin(np.radians(30))])
    t = np.arange(n) / fps
    pos = F.integrate(np.array([0.3, 0.6, 5.5]), np.array([-0.3, -3.5, -6.0]), -9.81 * up, t, ball)
    return t, ic.project(pos), pos[:, 2], ic, up, ball


def test_grow_flight_trims_the_hand_bridges_and_links():
    t, uv, z, ic, up, ball = _pinhole_flight()
    R, tr = np.eye(3), np.zeros(3)
    r_of = ic.fx * ball.radius / z

    t_all = np.concatenate([np.arange(-6, 0) / 60.0, t])                  # 6 frames of the ball still in the hand before the release
    zz = np.concatenate([np.full(6, z[0]), z])

    def observe(i, u, v, r, zg):
        return (u, v, r, zz[i], 0.02 + 0.01 * zz[i] ** 2, "depth")
    held = [(float(uv[0, 0]) - 30.0 + 0.5 * k, float(uv[0, 1]) + 12.0, float(r_of[0])) for k in range(6)]     # off the arc, nearly still
    cands = [[h] for h in held] + [[(float(u), float(v), float(r))] for (u, v), r in zip(uv, r_of)]
    core = [(i, *cands[i][0]) for i in range(0, 20)]                       # the 2D core swept up the hand frames with the first 14 of flight
    for k in range(30, 38):                                                # the lamp's glare: 8 frames with nothing
        cands[k] = []
    last_in_view = 6 + int(np.nonzero(uv[:, 1] + r_of < 480)[0].max())
    keys, pts, fit = A.grow_flight(t_all, cands, core, observe, ic, R, tr, ball, up=up, max_gap=12)
    assert min(keys) >= 6 and max(keys) >= last_in_view - 1                # the hand trimmed, the gap bridged, followed to the image's edge
    assert not set(range(30, 38)) & set(keys) and fit.rms_px < 1.0
    # the same with growing stopped by the gap: the later part comes back as a tracklet and is linked
    tl = dict(frames=list(range(38, len(t_all))), uv=[c[0][:2] for c in cands[38:]], r=[c[0][2] for c in cands[38:]])
    keys2, _, fit2 = A.grow_flight(t_all, cands, core, observe, ic, R, tr, ball, up=up, max_gap=2, tracklets=[tl])
    assert max(keys2) >= last_in_view - 1 and fit2.rms_px < 1.0
    keys3, _, _ = A.grow_flight(t_all, cands, core, observe, ic, R, tr, ball, up=up, max_gap=2)
    assert max(keys3) < 30                                                 # without the tracklet it stops at the gap


def test_a_blob_on_a_nearer_object_is_not_a_free_ball():
    ic = D.Intr(848, 480, 616.0, 616.0, 424.0, 240.0)
    depth = np.full((480, 848), 4000, np.uint16)                            # the room 4 m away (mm)
    depth[200:300, 380:470] = 1000                                          # an arm 1 m away
    R, tr = np.eye(3), np.zeros(3)
    assert D.depth_in_front(420.0, 250.0, 5.0, depth, 0.001, ic, ic, R, tr, z_guess=9.0)          # a 5 px orange patch on the arm: "9 m" by size
    assert not D.depth_in_front(200.0, 100.0, 12.0, depth, 0.001, ic, ic, R, tr, z_guess=3.0)     # a ball in front of the wall
    depth[90:110, 190:210] = 3000
    assert not D.depth_in_front(200.0, 100.0, 12.0, depth, 0.001, ic, ic, R, tr, z_guess=3.0)


def test_bucket_json_is_the_cad_geometry():
    cad = json.loads((HERE / "bucket.json").read_text())["cad"]
    b = P.Bucket.load(HERE / "bucket.json")
    th = np.radians(cad["pitch_deg"])
    s, c = np.sin(th), np.cos(th)
    o = cad["origin_from_back_face_m"]
    origin_y = cad["camera_back_horizontal_m"] + o["forward"] * c - o["up"] * s     # level frame, y toward the thrower: the depth origin from the axis
    origin_z = cad["camera_back_height_m"] + o["forward"] * s + o["up"] * c
    rim = np.array([-o["lateral"], -origin_y, cad["rim_height_m"] - origin_z])        # rim centre from the origin: right, forward, up
    want = np.array([rim[0], s * rim[1] - c * rim[2], c * rim[1] + s * rim[2]])
    assert np.allclose(b.rim_center, want, atol=1e-9) and abs(b.radius - cad["rim_diameter_m"] / 2) < 1e-9
    assert np.allclose(b.up, [0, -c, s])
    assert -want[1] > 0.25 and np.degrees(np.arctan2(np.hypot(want[0], want[1]), want[2])) > 60      # far above the optical axis: never in view


def _level_to_synth(p, pitch_deg):
    """A point in the camera's level frame (x right, y toward the thrower, z up) -> synth's camera frame (x right, y up, -z forward)."""
    s, c = np.sin(np.radians(pitch_deg)), np.cos(np.radians(pitch_deg))
    x, y, z = p
    return np.array([x, -(s * y - c * z), -(c * y + s * z)])


def test_bucket_on_the_cad_mount_extrapolates_the_crossing_behind_the_camera(tmp_path):
    b = P.Bucket.load(HERE / "bucket.json")
    pitch = json.loads((HERE / "bucket.json").read_text())["cad"]["pitch_deg"]
    cfg = X.RSSynthConfig(seed=3, pitch_deg=pitch)
    d = tmp_path / "s"
    d.mkdir()
    a = argparse.Namespace(size="848x480", fps=60.0, depth_fps=90.0, synthetic_pitch=pitch, distance=4.5, synthetic_gap=1.0)
    calib = R.SyntheticRig(a, ["color", "depth", "ir1", "ir2"], 2.0).start()
    sess = dict(camera=dict(name="synthetic", width=848, height=480, fps=60, fovy_deg=58.0, f_px=calib["streams"]["color"]["intrinsics"]["fy"]), rs=calib,
                detector={"hue_deg": [40, 90]}, basket={})
    S.save_session(d, sess)
    rim_synth = np.array([b.rim_center[0], -b.rim_center[1], -b.rim_center[2]])
    T = 1.0
    start = _level_to_synth((0.0, 4.5, 1.15), pitch)                  # released 4.5 m out, 1.2 m above the floor
    base = 10_000_000_000
    for k, dx in enumerate((0.0, 0.28), 1):         # 0.28 m is clear of the rim zone even for the 80 mm test ball (R + r = 0.222)
        arc = Y.aimed_arc(cfg.color_cfg(), start, rim_synth + np.array([dx, 0.0, 0.0]), T)   # through the rim plane at T, dx off the axis
        out, truth = X.render_streams(cfg, arc, T, pre=0.8, seed=k)
        streams = {n: dict(frames=f, stamps=np.column_stack([base + (ts * 1e9).astype(np.int64)] * 2 + [np.arange(len(ts)) + 1])) for n, (f, ts) in out.items()}
        th = S.load_throw(S.save_throw(d, k, None, None, dict(), streams=streams), sess)
        res = A.analyze_throw(th, sess, D.DetectCfg(), colour=False, session_dir=d)
        A.add_predictions(res, b, D.DetectCfg())
        pr = res["prediction"]["depth"]
        assert pr["ok"], pr
        ref = pr["reference"]
        assert abs(ref["radial_m"] - abs(dx)) < 0.04, (ref["radial_m"], dx)        # the miss, from an arc that stops short of the rim
        assert ref["verdict"] == ("in" if dx == 0 else "out")
        assert abs(ref["t_cross"] - (T + 0.8 - cfg.exposure / 2)) < 0.03             # and the time of the crossing
        assert ref["extrapolated_s"] > 0.05                                          # the ball left the view before it reached the rim
        assert pr["settled"] and pr["first_prediction_ttg"] > 0.5


def test_the_sessions_ball_sets_the_drag_law_and_the_blob_size(tmp_path):
    """The temporary test ball (orange foam, 19 7/8 in round = 160.7 mm, 73 g) is not the robot's 74 mm / 22 g one: the session carries it,
    the analysis must use it for both the drag law and the blob-size gate, and the robot config must stay as it is."""
    import dataclasses
    ball = F.Ball(mass=0.073, radius=19.875 * 0.0254 / (2 * np.pi))
    assert abs(2 * ball.radius - 0.1607) < 1e-3 and ball.k_quad() > 1.3 * F.Ball().k_quad()          # 42% more drag than the 22 g ball
    cfg = X.RSSynthConfig(seed=6, ball=ball)
    arc, T = Y.default_throw(cfg.color_cfg(), 4.5, seed=6)
    out, truth = X.render_streams(cfg, arc, T * 1.02, pre=0.8, seed=6)
    d = tmp_path / "s"
    d.mkdir()
    a = argparse.Namespace(size="848x480", fps=60.0, depth_fps=90.0, synthetic_pitch=0.0, distance=4.5, synthetic_gap=1.0)
    calib = R.SyntheticRig(a, ["color", "depth", "ir1", "ir2"], 2.0).start()
    sess = dict(camera=dict(name="synthetic", width=848, height=480, fps=60, fovy_deg=58.0, f_px=calib["streams"]["color"]["intrinsics"]["fy"]), rs=calib,
                detector={"hue_deg": [40, 90]}, basket={}, ball=dict(mass=ball.mass, radius=ball.radius, drag_cd=ball.drag_cd))
    S.save_session(d, sess)
    base = 10_000_000_000
    streams = {n: dict(frames=f, stamps=np.column_stack([base + (ts * 1e9).astype(np.int64)] * 2 + [np.arange(len(ts)) + 1])) for n, (f, ts) in out.items()}
    th = S.load_throw(S.save_throw(d, 1, None, None, dict(), streams=streams), sess)
    cfg_s = A.with_session_ball(D.DetectCfg(), sess)
    assert abs(cfg_s.ball.radius - ball.radius) < 1e-12 and abs(cfg_s.ball_radius - ball.radius) < 1e-12 and abs(cfg_s.ball.mass - 0.073) < 1e-12
    assert A.with_session_ball(D.DetectCfg(), dict()).ball.radius == 0.037                                 # no ball in the session: unchanged

    tr_p = np.column_stack([truth["pos"][:, 0], -truth["pos"][:, 1], -truth["pos"][:, 2]])
    tr_t = truth["t"] + 0.8 - cfg.exposure / 2

    def err(cfg_):
        res = A.analyze_throw(th, sess, cfg_, colour=False, session_dir=d, session_ball=False)
        s_ = res["sources"]["depth"]
        assert s_["ok"], s_
        m = (tr_t >= s_["first_t"]) & (tr_t <= s_["last_t"])
        return float(np.linalg.norm(res["_keep"]["depth"][2].at(tr_t[m]) - tr_p[m], axis=1).mean())
    right = err(cfg_s)                                                                  # the session's ball
    wrong = err(dataclasses.replace(D.DetectCfg(), ball_radius=ball.radius))            # the right blob size but the robot ball's drag
    assert right < 0.01, right
    assert wrong > 1.5 * right, (right, wrong)                                           # the drag law matters at this accuracy
    # and the recorder writes the ball it was given
    cfg_json = json.loads((HERE.parents[1] / "ros" / "src" / "omni_catch" / "omni_catch" / "config" / "robot_v11.json").read_text())
    assert cfg_json["ball"]["radius"] == 0.037 and cfg_json["ball"]["mass"] == 0.022                  # the robot's ball is unchanged
    ns = argparse.Namespace(synthetic=False, ball_circumference_in=19.875, ball_mass_g=73.0, ball_cd=0.5)
    bi = R.ball_info(ns)
    assert abs(bi["radius"] - ball.radius) < 1e-12 and bi["mass"] == 0.073 and bi["colour"] == "orange"


def test_throw_speed_gate_separates_a_hand_from_a_throw():
    ball_r = 0.0803

    def track(speed_mps, r_px, fps=60.0, n=5):
        step = speed_mps / ball_r * r_px / fps                         # pixels per frame of a ball of radius r_px moving sideways at that speed
        return [(i / fps, [(300.0 + i * step, 200.0, r_px)]) for i in range(n)]
    hand, throw = R.lateral_speeds(track(0.8, 9.0), ball_r), R.lateral_speeds(track(6.0, 9.0), ball_r)
    assert abs(hand[0] - 0.8) < 0.05 and abs(throw[0] - 6.0) < 0.3     # a carried ball is < 1.5 m/s, a thrown one is several
    far = R.lateral_speeds(track(6.0, 4.0), ball_r)                    # the same speed, the ball farther away (smaller, fewer px per frame)
    assert abs(far[0] - 6.0) < 0.3
    with_clutter = [(t, c + [(100.0, 50.0, 12.0)]) for t, c in track(6.0, 9.0)]            # a still blob elsewhere doesn't hide the ball
    assert max(R.lateral_speeds(with_clutter, ball_r)) > 5.0
    assert R.lateral_speeds(track(6.0, 9.0, n=2), ball_r) == [] and R.lateral_speeds([], ball_r) == []     # too short a chain: no verdict


def _ballistic_px(n=40, fps=60.0, z=3.5, f=616.0, R_=0.0803, v=(1.5, -5.0), t0=0.0):
    """Pixels of a ball moving across the view at range z (lateral m/s, image y down), gravity in the image: (t, [(u, v, r)]) per frame."""
    r = f * R_ / z
    out = []
    for i in range(n):
        tau = i / fps                                                   # the flight starts at t0
        x, y = v[0] * tau, v[1] * tau + 0.5 * 9.81 * tau * tau
        out.append((t0 + tau, (300 + f * x / z, 400 + f * y / z, r)))
    return out


def test_arc_trigger_fires_on_a_throw_not_on_decoys_or_a_carried_ball():
    import arc2d as Z
    cfg = Z.ArcCfg(ball_radius=0.0803, min_speed=2.0)
    rng = np.random.default_rng(3)
    decoys = [(640.0, 324.0, 9.0), (672.0, 258.0, 6.0), (20.0, 300.0, 15.0)]          # flowers, a red box, a pumpkin at the edge

    def run(frames):
        trk, hits = Z.ArcTracker(cfg), []
        for t, balls in frames:
            cs = [(u + rng.normal(0, 1.5), v + rng.normal(0, 1.5), r * rng.uniform(0.8, 1.2)) for u, v, r in decoys] + list(balls)
            cs.append((rng.uniform(0, 848), rng.uniform(0, 480), rng.uniform(4, 8)))   # a flicker somewhere new every frame
            h = trk.update(t, cs)
            if h:
                hits.append((t, h))
        return hits, trk

    still, _ = run([(i / 60, []) for i in range(180)])                                   # 3 s of the scene alone
    assert still == []
    carry, trk = run([(t, [(300 + 0.8 / 0.0803 * 14 * t, 380 + 6 * np.sin(5 * t), 14.0)]) for t in np.arange(0, 2, 1 / 60)])   # carried at 0.8 m/s
    assert carry == []
    throw, _ = run([(t, [c]) for t, c in _ballistic_px()])
    assert throw and throw[0][0] < 0.2                                                    # fires within ~0.2 s of the release
    assert abs(throw[0][1]["speed_mps"] - np.hypot(1.5, -5.0)) < 1.5 and 0.5 < throw[0][1]["acc_g"] < 1.5
    swing = [(t, [(300 + 40 * t * 60 * np.sin(30 * t), 400 - 3 * (t * 60) ** 2 * 0.2, 14.0)]) for t in np.arange(0, 0.3, 1 / 60)]   # an arm's swing: far over g
    assert run(swing)[0] == []


def test_offline_flight_track_drops_static_decoys_and_keeps_the_free_flight():
    import arc2d as Z
    cfg = Z.ArcCfg(ball_radius=0.0803)
    held = [(i / 60, (300 + 0.3 * i, 420 + 0.2 * i, 14.0)) for i in range(30)]           # held near still for 0.5 s
    fly = _ballistic_px(n=30, t0=0.5, v=(0.6, -4.0))                                     # up and over the apex (0.41 s in): slow there
    fly = [(t, (u + 9.0, v + 6.0, r)) for t, (u, v, r) in fly]
    times = [t for t, _ in held + fly]
    cands = [[c, (640.0, 324.0, 9.0)] for _, c in held + fly]                           # the flowers in every frame
    f = Z.flight_track(np.array(times), cands, cfg)
    assert f is not None
    assert all(abs(u - 640) > 5 for u, _ in f["uv"])                                      # never the flowers
    assert min(f["frames"]) >= 28 and len(f["frames"]) >= 25                               # the flight, not the held ball
