"""Tests for the throw-tracking tools, no camera: synthetic throws with known truth (synth.py) go through the
recorder's format, arcfit.py and analyze_throws.py, and the analysis must recover what was injected.

  & $py -m pytest -q          (in characterization/throw_track; ~1 min)
"""
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import analyze_throws as A        # noqa: E402
import arcfit as F                # noqa: E402
import session as S               # noqa: E402
import synth as Y                 # noqa: E402


def test_arc_fit_recovers_the_arc_and_gravity():
    cfg = Y.SynthConfig(pitch_deg=12, roll_deg=3, fovy_deg=50)
    cam = cfg.camera()
    arc, T = Y.default_throw(cfg, 4.5, seed=3)
    ts = np.arange(0, T * 0.95, 1 / 120)
    pos = arc.positions(ts)
    u, v, b = F.sphere_px(pos, cam, cfg.ball.radius)
    rng = np.random.default_rng(0)
    det = F.Detections(ts, u + rng.normal(0, 0.5, len(ts)), v + rng.normal(0, 0.5, len(ts)), b + rng.normal(0, 0.4, len(ts)))
    fit = F.fit_arc(det, cam, cfg.ball)
    assert fit.ok
    assert np.mean(np.linalg.norm(fit.positions(ts) - pos, axis=1)) < 0.02                 # 2 cm over a 4.5 m throw
    assert abs(math.degrees(fit.phi) - 12) < 0.5 and abs(math.degrees(fit.psi) - 3) < 0.5   # gravity found
    assert 0.35 < fit.rms_uv < 0.65                                                          # the injected 0.5 px


def test_arc_fit_with_gravity_known_and_an_outlier():
    cfg = Y.SynthConfig(pitch_deg=8, fovy_deg=50)
    cam = cfg.camera()
    arc, T = Y.default_throw(cfg, 4.5, seed=5)
    ts = np.arange(0, T * 0.9, 1 / 120)
    u, v, b = F.sphere_px(arc.positions(ts), cam, cfg.ball.radius)
    u[20] += 80                                                                              # a wrong blob
    det = F.Detections(ts, u, v, b)
    fit = F.fit_arc(det, cam, cfg.ball, up=cfg.up(), fit_gravity=False)
    assert fit.ok and not fit.inlier[20] and fit.rms_uv < 0.2


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    cfg = Y.SynthConfig(pitch_deg=8, pixel_noise=3.0, fovy_deg=50, centroid_jitter=0.4, exposure=0.004)
    return Y.write_session(tmp_path_factory.mktemp("tt"), cfg, n_throws=4), cfg


def test_session_roundtrip(session):
    d, cfg = session
    sess = S.load_session(d)
    dirs = S.throw_dirs(d)
    assert len(dirs) == 4 and sess["camera"]["width"] == cfg.width
    th = S.load_throw(dirs[0], sess)
    assert th.frames.shape[1:] == (cfg.height, cfg.width, 3) and th.label == "y"
    assert abs(np.diff(th.t).mean() - 1 / cfg.fps) < 1e-6


@pytest.fixture(scope="module")
def analysis(session, tmp_path_factory):
    d, cfg = session
    out = tmp_path_factory.mktemp("res")
    r = subprocess.run([sys.executable, str(HERE / "analyze_throws.py"), str(d), "--out", str(out)], capture_output=True,
                       text=True, timeout=900)
    assert r.returncode == 0, r.stderr[-2000:]
    f = next(out.glob("analysis_*.json"))
    return f, json.loads(f.read_text())


def test_analysis_recovers_the_injected_values(session, analysis):
    d, cfg = session
    res = analysis[1]
    s = res["summary"]
    assert s["fitted"] == 4
    assert 0.30 < s["rms_uv_px"]["median"] < 0.55                       # centroid noise injected: 0.4 px
    assert 0.28 < s["second_diff_px"]["median"] < 0.55                  # the arc-free estimate agrees
    assert s["detect_given_arc"]["rate"] > 0.9 and s["detect_given_arc"]["wrong"] == 0
    up = np.array(res["up_in_camera"])
    assert np.degrees(np.arccos(up @ cfg.up())) < 0.5                    # gravity direction (camera pitched 8 deg)
    assert sum(abs(t["sensor_to_host_ms"]["median"] - 12) < 0.1 for t in [s]) == 1
    # the tracker: ready within the first fifth of a second, and its crossing 0.5 s ahead within a few cm
    live = s["live"]
    assert live["throws_tracked"] == 4
    assert live["crossing_error_by_ttg"]["0.5"]["median_m"] < 0.05
    # the arc's verdict on the rim agrees with the labels (even throws were aimed inside, odd outside)
    assert live["arc_vs_label"]["fp"] == 0 and live["arc_vs_label"]["fn"] == 0


def test_recorder_dry_run_writes_throws(tmp_path):
    r = subprocess.run([sys.executable, str(HERE / "record_throws.py"), "--synthetic", "--count", "1", "--no-prompt",
                        "--name", "dry", "--out", str(tmp_path)], capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stderr[-2000:]
    d = next(tmp_path.iterdir())
    dirs = S.throw_dirs(d)
    assert len(dirs) >= 1
    th = S.load_throw(dirs[0], S.load_session(d))
    assert len(th.frames) > 150 and th.meta["trigger"] > 20


def test_fit_vision_sets_the_sims_camera_from_the_analysis(analysis, tmp_path):
    out = tmp_path / "vision_measured.json"
    r = subprocess.run([sys.executable, str(HERE / "fit_vision.py"), str(analysis[0]), "--out", str(out)], capture_output=True,
                       text=True, timeout=900)
    assert r.returncode == 0, r.stderr[-2000:]
    v = json.loads(out.read_text())
    vis = v["vision"]
    # injected: 0.4 px centroid jitter, 4 ms exposure, 12 ms sensor -> host: the sim's per-pixel edge jitter that gives
    # 0.4 px of centroid noise is a few px (the centroid averages over the blob's edge pixels)
    assert 1.2 < vis["edge_sigma"] < 3.5
    assert abs(vis["exposure"] - 0.004) < 1e-6 and abs(vis["latency_mean"] - (0.012 + 0.002 + 0.005)) < 1e-3
    assert 0.0 <= vis["drop_prob"] < 0.15 and vis["false_prob"] < 0.02
    assert v["not_measured"].startswith("calib_rot_deg")
    from importlib import import_module
    sys.path.insert(0, str(HERE.parents[1] / "simulation" / "v11 sil"))
    cw = import_module("catch_world")
    assert cw.measured_cameras(out).edge_sigma == vis["edge_sigma"]


def test_gate_compares_the_sim_with_the_real_throws(analysis, tmp_path):
    vis = tmp_path / "vision_measured.json"
    subprocess.run([sys.executable, str(HERE / "fit_vision.py"), str(analysis[0]), "--out", str(vis)], check=True, capture_output=True,
                   timeout=900)
    r = subprocess.run([sys.executable, str(HERE / "gate_sim_vs_real.py"), str(analysis[0]), "--vision", str(vis)],
                       capture_output=True, text=True, timeout=900)
    assert r.returncode in (0, 1), r.stderr[-2000:]
    g = json.loads(next(analysis[0].parent.glob("gate_*.json")).read_text())
    rows = {x["metric"]: x for x in g["rows"]}
    assert rows["centroid noise rms (px)"]["ok"]                  # what fit_vision matched
    assert "GATE:" in r.stdout and g["throws"] == 4
