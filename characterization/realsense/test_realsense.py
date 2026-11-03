"""The characterization's maths on synthetic data with known answers (no camera needed).

    C:\\Users\\ferna\\.venvs\\realsense\\Scripts\\python.exe -m pytest -q
"""
import numpy as np
import pytest

import rs_common as C

W, H, F, PPX, PPY = 848, 480, 447.0, 424.0, 240.0


def render_sphere(c, R, bg_z=4.0):
    """Depth image (m) of a sphere at c in front of a wall at bg_z, pinhole camera W x H, f F."""
    v, u = np.mgrid[0:H, 0:W]
    d = np.stack(((u - PPX) / F, (v - PPY) / F, np.ones_like(u, float)), axis=-1)
    c = np.asarray(c, float)
    a = np.sum(d * d, axis=-1)
    b = -2 * d @ c
    cc = c @ c - R * R
    disc = b * b - 4 * a * cc
    t = np.where(disc >= 0, (-b - np.sqrt(np.clip(disc, 0, None))) / (2 * a), np.inf)
    z = np.where(np.isfinite(t), t, bg_z)          # d_z = 1: the ray parameter is the depth
    return np.minimum(z, bg_z).astype(np.float32)


def test_stats_and_intervals():
    s = C.stats([1, 2, 3, None, np.nan, 4])
    assert s["n"] == 4 and s["median"] == pytest.approx(2.5)
    t = np.arange(0, 1000, 1000 / 90.0)
    t = np.delete(t, [10, 11, 40])                  # three frames lost
    iv = C.interval_stats(t, 90)
    assert iv["lost_by_interval"] == 3
    assert iv["dt_median_ms"] == pytest.approx(1000 / 90, rel=1e-6)
    assert C.counter_gaps([1, 2, 3, 6, 7, 9]) == 3


def test_parse_mode():
    assert C.parse_mode("depth:848x480@90+ir:848x480@90") == [("depth", 848, 480, 90), ("ir", 848, 480, 90)]
    with pytest.raises(ValueError):
        C.parse_mode("thermal:1x1@1")


def test_plane_fit_and_noise():
    rng = np.random.default_rng(0)
    n_true = np.array([0.1, -0.2, -1.0])
    n_true /= np.linalg.norm(n_true)
    p0 = np.array([0.0, 0.0, 2.0])
    a = np.cross(n_true, [1, 0, 0]); a /= np.linalg.norm(a)
    b = np.cross(n_true, a)
    uv = rng.uniform(-0.5, 0.5, (5000, 2))
    pts = p0 + uv[:, :1] * a + uv[:, 1:] * b + rng.normal(0, 0.002, (5000, 1)) * n_true
    n, d, r = C.fit_plane(pts)
    assert C.angle_between_deg(n, n_true) < 0.1
    assert n @ p0 + d == pytest.approx(0, abs=1e-3)
    assert np.sqrt(np.mean(r ** 2)) == pytest.approx(0.002, rel=0.05)
    assert n @ (-p0) > 0                              # normal toward the camera
    assert C.plane_distance_on_axis(n, d) == pytest.approx(2.0, abs=1e-3)


def test_floor_pose_from_plane():
    """A camera 0.30 m above the floor, looking 40 deg down, rolled 2 deg."""
    pitch, roll, hgt = np.radians(40), np.radians(2), 0.30
    # world: X forward, Y left, Z up; camera optical frame x right, y down, z forward
    Rp = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rr = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # optical axes in a level camera's body frame
    R_wc = Rp @ Rr @ opt                                          # columns: optical x, y, z in the world
    rng = np.random.default_rng(1)
    g = np.column_stack((rng.uniform(0.3, 1.5, 4000), rng.uniform(-0.5, 0.5, 4000), np.zeros(4000)))
    cam = np.array([0, 0, hgt])
    pts_c = (g - cam) @ R_wc                                      # world -> camera
    n, d, _ = C.fit_plane(pts_c)
    P = C.plane_pose(n, d)
    assert P["height_m"] == pytest.approx(hgt, abs=1e-6)
    assert P["pitch_deg"] == pytest.approx(40, abs=1e-3)
    assert abs(P["roll_deg"]) == pytest.approx(2, abs=1e-3)


def test_deproject_roundtrip():
    z = np.full((H, W), 2.0, np.float32)
    z[0, 0] = 0
    pts = C.deproject(z, F, F, PPX, PPY)
    assert len(pts) == H * W - 1
    assert np.allclose(pts[:, 2], 2.0)
    i = (int(PPY) * W + int(PPX)) - 1                # the principal point, minus the dropped pixel
    assert np.allclose(pts[i, :2], 0.0, atol=1e-9)


def test_stereo_models():
    z, f, B, sd = 6.5, 447.0, 0.05, 0.08
    s = C.stereo_sigma_z(z, f, B, sd)
    assert s == pytest.approx(z * z * sd / (f * B))
    assert C.disparity_sigma(z, s, f, B) == pytest.approx(sd)
    # 1 mm at 2 m on a 50 mm baseline: B dz / z^2 rad
    assert C.depth_change_to_angle_deg(2.0, 0.001, 0.05) == pytest.approx(np.degrees(0.05 * 0.001 / 4))
    assert C.ball_radius_px(6.5, f) == pytest.approx(f * 0.037 / 6.5)


def test_flash_edges_and_matching():
    """A square wave seen 35 ms late at 90 fps, with sensor noise."""
    rng = np.random.default_rng(2)
    lag = 35.0
    cmd, t, state = [], 0.0, False
    while t < 20000:
        state = not state
        cmd.append((t, 1 if state else -1))
        t += rng.uniform(250, 450)
    frames = np.arange(0, 20000, 1000 / 90)

    def level(tf):
        on = [c for c in cmd if c[0] + lag <= tf]
        return 200.0 if (on and on[-1][1] == 1) else 20.0

    lv = np.array([level(tf) for tf in frames]) + rng.normal(0, 3, frames.size)
    edges = C.flash_edges(frames, lv)
    lags = C.match_flashes(cmd, edges)
    got = [x for x in lags if x is not None]
    assert len(got) >= len(cmd) - 1
    assert np.all(np.array(got) >= lag - 1e-6)
    assert np.all(np.array(got) <= lag + 1000 / 90 + 1e-6)     # quantized by one frame period


def test_ballistic_fit():
    t = np.arange(0, 0.8, 1 / 90)
    p0, v0, a = np.array([0.2, -0.5, 6.0]), np.array([0.1, -3.0, -6.0]), np.array([0, 9.81, 0])
    p = p0 + np.outer(t, v0) + 0.5 * np.outer(t ** 2, a)
    p += np.random.default_rng(3).normal(0, 0.01, p.shape)
    fit = C.fit_ballistic(t, p)
    assert fit["a_norm"] == pytest.approx(9.81, abs=0.5)
    assert fit["rms_m"] == pytest.approx(0.01 * np.sqrt(3), rel=0.15)
    assert fit["speed0"] == pytest.approx(np.linalg.norm(v0), rel=0.02)


def test_subpixel_peak():
    x0 = 0.3
    f = lambda x: -(x - x0) ** 2
    assert C.subpixel_peak(f(-1), f(0), f(1)) == pytest.approx(x0)


def test_ball_segmentation_and_centre_depth():
    import rs_ball
    R, c = 0.037, np.array([0.1, -0.05, 2.0])
    z = render_sphere(c, R)
    u, v = c[0] / c[2] * F + PPX, c[1] / c[2] * F + PPY
    seg = rs_ball.segment_ball(z, (int(round(u)), int(round(v))), R, F)
    assert seg is not None
    mask, cen, z_front = seg
    assert cen[0] == pytest.approx(u, abs=0.5) and cen[1] == pytest.approx(v, abs=0.5)
    r_model = float(C.ball_radius_px(c[2], F, 2 * R))
    assert np.sqrt(mask.sum() / np.pi) == pytest.approx(r_model, rel=0.1)
    fill, zc = rs_ball.ball_centre_depth(z, cen, r_model, R)
    assert fill == pytest.approx(1.0)
    assert zc == pytest.approx(np.linalg.norm(c), abs=0.004)   # centre along its ray, ~ c_z here


def test_blob_disparity_masked():
    """A textured ball 2 m away in front of a textured wall at 4 m: the ball's disparity, not the wall's."""
    import rs_ball
    B = 0.05
    r = F * 0.037 / 2.0
    cx, cy = 400.3, 250.6
    d_ball, d_bg = F * B / 2.0, F * B / 4.0
    v, u = np.mgrid[0:H, 0:W].astype(float)

    def ball_tex(x, y):
        return 120 + 60 * np.sin(0.9 * x + 0.3) * np.cos(0.7 * y)

    def wall_tex(x, y):
        return 60 + 40 * np.sin(0.21 * x) * np.sin(0.17 * y + 1.0)

    def image(shift_ball, shift_bg):
        x_b, y_b = u - (cx - shift_ball), v - cy
        inside = x_b ** 2 + y_b ** 2 <= r * r
        return np.where(inside, ball_tex(x_b, y_b), wall_tex(u + shift_bg, v)).astype(np.float32)

    left, right = image(0.0, 0.0), image(d_ball, d_bg)
    res = rs_ball.blob_disparity(left, right, (cx, cy), r, d_guess=d_ball * 1.1)
    assert res is not None
    disp, dy, peak, _ = res
    assert disp == pytest.approx(d_ball, abs=0.15)
    assert abs(dy) < 0.2 and peak > 0.9


def test_throw_detection_and_tracks():
    import rs_throw
    R = 0.037
    bg = np.full((H, W), 4.0, np.float32)
    dets = []
    for k, t in enumerate(np.arange(0, 0.5, 1 / 90)):
        c = np.array([0.3 - 0.5 * t, -0.4 + (-2.0 * t + 0.5 * 9.81 * t * t), 3.5 - 5.0 * t])
        z = render_sphere(c, R)
        hit = rs_throw.detect(z, bg, F, R)
        assert hit is not None, f"frame {k}"
        uu, vv, zc = hit[:3]
        xyz = ((uu - PPX) / F * zc, (vv - PPY) / F * zc, zc)
        assert np.linalg.norm(np.subtract(xyz, c)) < 0.03
        dets.append((t * 1000, xyz, hit[3], hit[4], hit[5], k))
    assert rs_throw.detect(bg.copy(), bg, F, R) is None
    tr = rs_throw.tracks_from(dets + [(2000.0, (0, 0, 1), 1, 1, 1, 999)])
    assert len(tr) == 2 and len(tr[0]) == len(dets)
    fit = C.fit_ballistic(np.array([d[0] for d in tr[0]]) / 1000, np.array([d[1] for d in tr[0]]))
    assert fit["a_norm"] == pytest.approx(9.81, abs=1.0)


def test_drift_summary():
    import rs_drift
    rows = [dict(t_s=60.0 * m, dist_m=2.0 + 0.0002 * min(m, 20) / 20, normal=[0, 0, 1],
                 temperatures=dict(asic_temperature=30 + m / 2, projector_temperature=None)) for m in range(0, 41, 2)]
    s = rs_drift.summarize(rows, 0.05)
    assert s["depth_drift_mm"]["all"] == pytest.approx(0.2, abs=1e-6)
    assert s["equiv_pair_yaw_deg"]["all"] == pytest.approx(np.degrees(0.05 * 0.0002 / 4), rel=1e-3)
    assert s["verdict"] == "inside the sim's 0.04 deg"
    assert s["asic_temperature_c"] == [30, 50]


def test_plane_analysis_on_a_noisy_wall():
    import rs_depth_plane
    rng = np.random.default_rng(4)
    z = 2.0 + rng.normal(0, 0.004, (20, H, W)).astype(np.float32)
    intr = dict(fx=F, fy=F, ppx=PPX, ppy=PPY, width=W, height=H)
    r = rs_depth_plane.analyze(z, intr, 0.4, 0.05, floor=False, min_height_mm=15, truth_m=2.0 - C.D435_DATASHEET["ground_zero_mm"] / 1000)
    assert r["perpendicular_m"] == pytest.approx(2.0, abs=1e-3)
    assert r["spatial_rms_mm"]["median"] == pytest.approx(4.0, rel=0.1)
    assert r["temporal_std_mm"]["median"] == pytest.approx(4.0, rel=0.1)
    assert abs(r["bias_mm"]) < 1.0
    assert r["disparity_sigma_px"] == pytest.approx(0.004 * F * 0.05 / 4, rel=0.1)


def test_report_runs_on_synthetic_results(tmp_path, monkeypatch):
    import rs_report
    monkeypatch.setattr(C, "RESULTS", tmp_path)
    C.save_result("plane", dict(intrinsics=dict(fx=F), baseline_m=0.05, captures=[
        dict(perpendicular_m=z, spatial_rms_mm=dict(median=1000 * C.stereo_sigma_z(z, F, 0.05, 0.08)),
             spatial_rms_pct=0.1, fill_rate=dict(median=0.99), temporal_std_mm=dict(median=1.0),
             disparity_sigma_px=0.08, normal_noise_deg=dict(median=0.01)) for z in (0.5, 1, 2, 3)]))
    rs_report.main()
    text = (tmp_path / "REPORT.md").read_text(encoding="utf-8")
    assert "Fitted subpixel noise 0.080 px" in text
    assert (tmp_path / "report_plane.png").exists()
