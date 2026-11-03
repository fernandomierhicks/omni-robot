"""ball_view.py without a camera: the maths on known inputs, and the whole pipeline on the synthetic source
(a rendered ball with known position), then the web server.   python -m pytest -q   (the omni-robot-sim venv)"""
import json
import threading
import urllib.request

import numpy as np
import pytest

import ball_view as B


def test_size_range_is_the_pinhole_inverse():
    f, R = 500.0, 0.037
    for z in (0.5, 1.0, 3.0, 6.5):
        assert B.size_range(f * R / z, f, 2 * R) == pytest.approx(z)


def test_depth_position_finds_the_centre_behind_the_front_surface():
    h, w, R, z = 120, 160, 0.037, 2.0
    depth = np.full((h, w), 3.5)
    yy, xx = np.mgrid[0:h, 0:w]
    f, u, v = 400.0, 80.0, 60.0
    b = f * R / z
    d2 = (xx + 0.5 - u) ** 2 + (yy + 0.5 - v) ** 2
    m = d2 <= b * b
    depth[m] = z - np.sqrt(np.clip(R * R - d2[m] / (b * b) * R * R, 0, None))
    got = B.depth_position(depth, u, v, b, 2 * R)
    assert got["z"] == pytest.approx(z, abs=0.004)           # the inner disc sits a little in front of the apex
    assert got["fill"] == 1.0


def test_depth_position_needs_depth():
    assert B.depth_position(np.zeros((50, 50)), 25, 25, 8, 0.074) is None


def test_depth_scale_is_applied():
    d = np.full((50, 50), 2000, np.uint16)
    assert B.depth_position(d, 25, 25, 8, 0.074, scale=0.001)["z_surface"] == pytest.approx(2.0)


def test_pick_window_centres_on_the_colour():
    patch = np.full((7, 7, 3), B.BALL_RGB, np.uint8)
    w = B.pick_window(patch)
    assert w["hue_min"] < w["picked"]["h"] < w["hue_max"]
    assert 60 < w["picked"]["h"] < 75


def test_second_difference_noise_recovers_sigma():
    rng = np.random.default_rng(0)
    t = np.arange(400) * 0.01
    u = 100 + 80 * t + 0.4 * rng.standard_normal(400)         # a smooth line plus 0.4 px noise
    assert B.second_difference_noise(list(u)) == pytest.approx(0.4, rel=0.15)
    assert B.second_difference_noise([1.0, 2.0]) is None


def make_pipe(**kw):
    src = B.SyntheticSource(pace=False)
    return src, B.Pipeline(src, B.Params(src), **kw)


@pytest.mark.parametrize("native", [True, False])
def test_pipeline_finds_the_ball_and_its_range(native):
    src, pipe = make_pipe(native=native)
    pipe.params.set("ignore_still", True)                      # the yellow box would win as the largest blob
    errs_px, errs_size, errs_depth, hits = [], [], [], 0
    for _ in range(90):
        fr = src.read()
        res = pipe.process(fr)
        t = fr.meta["truth"]
        if res.det is None:
            continue
        hits += 1
        errs_px.append(np.hypot(res.det[0] - t["u"], res.det[1] - t["v"]))
        errs_size.append(abs(res.size["z"] - t["z"]) / t["z"])
        assert res.depth is not None
        errs_depth.append(abs(res.depth["z"] - t["z"]) / t["z"])
    assert hits >= 75                                          # a few frames at the path's turning points are "still"
    assert np.median(errs_px) < 0.6
    assert np.median(errs_size) < 0.04                         # the threshold shaves the dim rim: a few %
    assert np.median(errs_depth) < 0.01


def test_tracking_window_follows_and_agrees_with_acquisition():
    src, pipe = make_pipe()
    pipe.params.set("ignore_still", True)
    p = pipe.params.get()
    p["track"] = True
    for _ in range(40):
        res = pipe.process(src.read(), p)
    t = res.frame.meta["truth"]
    assert res.window is not None
    assert np.hypot(res.det[0] - t["u"], res.det[1] - t["v"]) < 1.0


def test_default_acquisition_takes_the_largest_blob_not_the_ball():
    """The weakness detector.py documents: a bigger ball-coloured object wins until the track starts."""
    src, pipe = make_pipe()
    res = pipe.process(src.read())
    assert res.det is not None and abs(res.det[0] - 90) < 5 and abs(res.det[1] - 340) < 5     # the box
    assert len(res.cands) >= 2                                 # and the ball is among the candidates


def test_hue_window_off_the_ball_loses_it():
    src, pipe = make_pipe()
    pipe.params.set("hue_min", 200)
    pipe.params.set("hue_max", 260)
    res = pipe.process(src.read())
    assert res.det is None and res.size is None


def test_depth_suggests_the_size_gain_that_makes_them_agree():
    src, pipe = make_pipe()
    pipe.params.set("ignore_still", True)
    pipe.params.set("size_gain", 0.8)                         # a deliberately wrong gain
    res = None
    for _ in range(12):
        res = pipe.process(src.read())
        if res.depth is not None:
            break
    g = res.depth["suggest_size_gain"]
    assert 0.9 < g < 1.25                                     # back to ~1 (the dim rim shaves a little)
    pipe.params.set("size_gain", g)
    res = pipe.process(src.read())
    assert abs(res.depth["size_vs_depth_pct"]) < 2.0


def test_params_clamp_and_ints():
    src, pipe = make_pipe()
    pipe.params.set("hue_min", -50)
    pipe.params.set("min_blob_px", 12.7)
    pipe.params.set("nonsense", 3)
    v = pipe.params.get()
    assert v["hue_min"] == 0 and v["min_blob_px"] == 13 and "nonsense" not in v


def test_render_gives_jpegs_in_both_views():
    src, pipe = make_pipe()
    pipe.params.set("show_mask", True)
    res = pipe.process(src.read())
    assert B.render(res, pipe)[:2] == b"\xff\xd8"
    pipe.params.set("view", "depth")
    res = pipe.process(src.read())
    assert B.render(res, pipe)[:2] == b"\xff\xd8"


def test_server_status_set_pick_calibrate():
    src = B.SyntheticSource(pace=False)
    pipe = B.Pipeline(src, B.Params(src))
    threading.Thread(target=pipe.run, daemon=True).start()
    srv = B.serve(pipe, 0, host="127.0.0.1")
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{port}"
    try:
        def get(path):
            return urllib.request.urlopen(url + path, timeout=10).read()

        def post(path, body=None):
            req = urllib.request.Request(url + path, json.dumps(body or {}).encode(), {"Content-Type": "application/json"})
            return json.loads(urllib.request.urlopen(req, timeout=10).read())

        assert b"Ball view" in get("/")
        for _ in range(100):                                   # wait for a first result
            if pipe.latest is not None and pipe.latest.det is not None:
                break
            threading.Event().wait(0.05)
        st = json.loads(get("/status"))
        assert st["det"] is not None and st["has_depth"] and st["params"]["hue_min"] == 45
        post("/set", {"hue_min": 50, "size_gain": 1.1, "track": True})
        assert pipe.params.get()["hue_min"] == 50 and pipe.params.get()["track"] is True
        w = post("/pick", {"x": 0.5, "y": 0.5})
        assert "picked" in w
        post("/reset")
        assert pipe.params.get()["hue_min"] == 45
        post("/set", {"known_m": 5.0})
        post("/calibrate")
        assert pipe.params.get()["f_scale"] != 1.0
        assert get("/snapshot.jpg")[:2] == b"\xff\xd8"
        with urllib.request.urlopen(url + "/stream", timeout=10) as s:       # the first multipart frame
            head = s.read(300)
        assert b"--frame" in head and b"image/jpeg" in head
    finally:
        pipe.stop.set()
        srv.shutdown()


def test_changing_the_rule_drops_the_old_lock_and_a_lost_track_reacquires_by_the_rule():
    src, pipe = make_pipe()
    p = pipe.params.get()
    res = pipe.process(src.read(), p)                          # the default rule: the largest blob, the box
    assert abs(res.det[0] - 90) < 5
    p = dict(p, track=True, ignore_still=True)
    for _ in range(8):                                         # the new rule starts over and finds the ball
        res = pipe.process(src.read(), p)
    t = res.frame.meta["truth"]
    assert res.det is not None and np.hypot(res.det[0] - t["u"], res.det[1] - t["v"]) < 1.5
    res = pipe.process(src.read(), dict(p, hue_min=200, hue_max=260))      # the ball vanishes: the track is lost
    assert res.det is None and pipe.last is None
    for _ in range(8):                                         # and comes back as the ball, not the box
        res = pipe.process(src.read(), p)
        if res.det is not None:
            break
    t = res.frame.meta["truth"]
    assert res.det is not None and np.hypot(res.det[0] - t["u"], res.det[1] - t["v"]) < 1.5


# ---- colour swatches, size markers, auto-tune

def test_hsv_of_agrees_with_the_detectors_colour_test():
    from omni_catch.detector import DetectorConfig, ball_mask
    rng = np.random.default_rng(3)
    img = rng.integers(0, 256, (60, 80, 3), dtype=np.uint8)
    cfg = DetectorConfig(hue_deg=(40.0, 110.0), min_sat=0.3, min_val=0.25)
    h, s, v = B.hsv_of(img)
    mine = ((h >= 40) & (h <= 110) & (s >= 0.3) & (v >= 0.25)).reshape(60, 80)
    assert (mine != ball_mask(img, cfg)).mean() < 0.002        # float rounding on a few edge pixels at most
    h, s, v = B.hsv_of(np.array([[[209, 242, 31]]], np.uint8))
    assert abs(h[0] - 70.0) < 1.5 and s[0] > 0.85 and v[0] > 0.9


def ball_scene(box=True):
    """The synthetic scene's first frame (the ball dead centre at 2 m) and its pipeline."""
    src, pipe = make_pipe()
    return src.read(), pipe


def test_tune_colour_covers_the_ball_and_ignores_a_bit_of_background():
    fr, _ = ball_scene()
    t = fr.meta["truth"]
    r = t["b"] * 0.75
    px = B.disc_pixels(fr.rgb, t["u"], t["v"], r)
    tuned = B.tune_colour(px)
    h, s, v = B.hsv_of(px)
    assert tuned["hue_min"] <= np.percentile(h, 1) and tuned["hue_max"] >= np.percentile(h, 99)
    assert tuned["hue_max"] - tuned["hue_min"] < 40            # tight: the yellow box (48 deg) is outside it
    assert ((s >= tuned["sat_min"]) & (v >= tuned["val_min"])).mean() > 0.97
    mixed = np.concatenate([px, np.tile(np.array([[90, 100, 110]], np.uint8), (len(px) // 4, 1))])      # a grey wall in the disc
    assert B.tune_colour(mixed)["hue_min"] == tuned["hue_min"]
    assert B.tune_colour(np.tile(np.array([[90, 100, 110]], np.uint8), (200, 1))) is None


def test_autotune_on_the_ring_finds_the_ball_and_drops_the_box():
    src, pipe = make_pipe()
    pipe.params.set("hue_min", 200)                             # a window that sees nothing
    pipe.params.set("hue_max", 260)
    pipe.params.set("known_m", 2.0)                             # the ball is at 2 m, centred, in the first frame
    res = pipe.process(src.read())
    assert res.det is None
    out = B.autotune(res, "ring")
    assert out["ok"], out["message"]
    a = out["applied"]
    assert a["hue_min"] <= 68 <= a["hue_max"] and a["hue_min"] > 52         # the ball's hue, not the box's (~48)
    assert out["others"] == 0 and "Nothing else" in out["message"]
    area = np.pi * res.frame.meta["truth"]["b"] ** 2
    assert 3 <= a["min_blob_px"] <= 0.3 * area
    for k, v in a.items():
        pipe.params.set(k, v)
    res = pipe.process(src.read())                              # default acquisition: the largest blob is now the ball
    t = res.frame.meta["truth"]
    assert res.det is not None and np.hypot(res.det[0] - t["u"], res.det[1] - t["v"]) < 1.0


def test_autotune_reports_other_blobs_that_pass():
    src, pipe = make_pipe()
    pipe.params.set("known_m", 2.0)
    res = pipe.process(src.read())
    p = dict(res.params)
    img = res.frame.rgb.copy()
    img[300:380, 40:140] = B.BALL_RGB                           # a second ball-coloured patch
    import dataclasses
    out = B.autotune(dataclasses.replace(res, frame=dataclasses.replace(res.frame, rgb=img), params=p), "ring")
    assert out["ok"] and out["others"] == 1 and "other blob" in out["message"]


def test_autotune_says_why_it_cannot():
    src, pipe = make_pipe()
    pipe.params.set("hue_min", 200)
    pipe.params.set("hue_max", 260)
    res = pipe.process(src.read())
    out = B.autotune(res, "blob")                               # no blob to use
    assert not out["ok"] and "No blob" in out["message"]
    pipe.params.set("known_m", 9.0)                             # a ring of 2 px
    res = pipe.process(src.read())
    out = B.autotune(res, "ring")
    assert not out["ok"] and "too small" in out["message"]
    pipe.params.set("known_m", 2.0)
    src.k = 40                                                  # the ball is off centre now: the ring sees the background
    res = pipe.process(src.read())
    out = B.autotune(res, "ring")
    assert not out["ok"] and "single saturated colour" in out["message"]


def test_guides_small_blobs_and_status_extras():
    src, pipe = make_pipe()
    pipe.params.set("min_blob_px", 400)                         # the ball (~270 px) is under it
    pipe.params.set("show_small", True)
    pipe.params.set("ignore_still", False)
    res = pipe.process(src.read())
    assert any(abs(c[0] - res.frame.meta["truth"]["u"]) < 2 for c in res.small)
    assert B.render(res, pipe)[:2] == b"\xff\xd8"
    pipe.latest = res
    st = pipe.status()
    assert st["ring"]["r"] > 0 and st["limits"]["range_m"] > 0
    pipe.params.set("min_blob_px", 8)
    pipe.params.set("hue_min", 58)                              # the box is ~51 deg: leave it out, so the blob is the ball
    res = pipe.process(src.read())
    pipe.latest = res
    st = pipe.status()
    assert st["blob_hsv"] is not None and 55 < st["blob_hsv"]["h"][1] < 85
    assert st["limits"]["r_min_px"] == pytest.approx((8 / np.pi) ** 0.5)
    pipe.params.set("show_guides", False)
    assert B.render(pipe.process(src.read()), pipe)[:2] == b"\xff\xd8"


def test_server_autotune_and_undo():
    src = B.SyntheticSource(pace=False)
    pipe = B.Pipeline(src, B.Params(src))
    srv = B.serve(pipe, 0, host="127.0.0.1")
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{port}"

    def post(path, body=None):
        req = urllib.request.Request(url + path, json.dumps(body or {}).encode(), {"Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(req, timeout=10).read())
    try:
        pipe.params.set("hue_min", 200)
        pipe.params.set("hue_max", 260)
        pipe.params.set("known_m", 2.0)
        pipe.latest = pipe.process(src.read())
        assert post("/undo")["ok"] is False
        out = post("/autotune", {"target": "ring"})
        assert out["ok"], out
        v = pipe.params.get()
        assert v["hue_min"] == out["applied"]["hue_min"] and v["min_blob_px"] == out["applied"]["min_blob_px"]
        assert post("/undo")["ok"]
        v = pipe.params.get()
        assert v["hue_min"] == 200 and v["min_blob_px"] == 8
    finally:
        pipe.stop.set()
        srv.shutdown()


def test_effective_min_radius_is_the_larger_of_blob_size_and_half_the_stride():
    assert B.effective_min_radius(dict(min_blob_px=8, coarse_stride=4)) == pytest.approx(2.0)      # the stride limits
    assert B.effective_min_radius(dict(min_blob_px=100, coarse_stride=4)) == pytest.approx((100 / np.pi) ** 0.5)
    assert B.effective_min_radius(dict(min_blob_px=1, coarse_stride=1)) == pytest.approx(0.56, abs=0.01)


def test_save_and_load_settings_round_trip(tmp_path):
    src, pipe = make_pipe()
    pipe.settings_dir = tmp_path / "saved"
    pipe.latest = pipe.process(src.read())
    assert B.load_settings(pipe)["ok"] is False                 # nothing saved yet
    assert pipe.status()["saved_at"] is None
    for k, v in dict(hue_min=52, hue_max=81, sat_min=0.6, val_min=0.5, min_blob_px=30, size_gain=1.12, ball_mm=70,
                     f_scale=1.05, exposure_us=2000, gain=3.0, track=True).items():
        pipe.params.set(k, v)
    out = B.save_settings(pipe)
    assert out["ok"] and (tmp_path / "saved" / "synthetic.json").is_file()
    assert len(list((tmp_path / "saved").glob("synthetic_*.json"))) == 1
    s = json.loads((tmp_path / "saved" / "synthetic.json").read_text())
    d = s["detector"]
    assert d["hue_deg"] == [52, 81] and d["min_sat"] == 0.6 and d["min_val"] == 0.5 and d["min_blob_px"] == 30
    assert d["size_gain"] == 1.12 and s["ball_mm"] == 70 and s["focal_scale"] == 1.05 and s["camera"]["exposure_us"] == 2000
    assert s["rules"]["track"] is True and s["format"] == "ball_view/1" and s["image_px"] == [640, 480]
    from omni_catch.detector import DetectorConfig         # the block is a DetectorConfig's fields
    DetectorConfig(**{**d, "hue_deg": tuple(d["hue_deg"])})
    assert pipe.status()["saved_at"] is not None
    pipe.params.set("hue_min", 10)
    pipe.params.set("track", False)
    assert B.load_settings(pipe)["ok"]
    v = pipe.params.get()
    assert v["hue_min"] == 52 and v["min_blob_px"] == 30 and v["track"] is True and v["exposure_us"] == 2000


def test_server_save_and_load(tmp_path):
    src = B.SyntheticSource(pace=False)
    pipe = B.Pipeline(src, B.Params(src))
    pipe.settings_dir = tmp_path
    srv = B.serve(pipe, 0, host="127.0.0.1")
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def post(path, body=None):
        req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", json.dumps(body or {}).encode(),
                                     {"Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(req, timeout=10).read())
    try:
        pipe.latest = pipe.process(src.read())
        post("/set", {"hue_min": 61, "val_min": 0.5})
        out = post("/save")
        assert out["ok"] and out["settings"]["detector"]["hue_deg"][0] == 61
        post("/reset")
        assert pipe.params.get()["hue_min"] == 45
        assert post("/load")["ok"] and pipe.params.get()["hue_min"] == 61
    finally:
        srv.shutdown()
