"""motion_analyze.py on a synthetic session with known truth: a still scene (tag, ball, texture) seen by a camera that
turns ever faster. A pure rotation maps the whole image by H = K R K^-1 whatever the depth (the robot turning), and
each frame is the average of sub-frame renders over its exposure (the blur)."""
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from motion_analyze import blur_length, colour_mask  # noqa: E402

W, H, F = 848, 480, 616.


def test_blur_length_of_a_smeared_disc():
    for L in (0., 6., 15., 30.):
        img = np.zeros((120, 160), np.float32)
        offsets = np.linspace(-L / 2, L / 2, 61)
        for s in offsets:                                    # the exposure's average of the moving disc
            disc = np.zeros_like(img)
            cv2.circle(disc, (80 * 16 + int(round(s * 16)), 60 * 16), 12 * 16, 1., -1, shift=4, lineType=cv2.LINE_AA)
            img += disc / len(offsets)
        L2, r = blur_length(img)
        assert abs(L2 - L) < max(2., .1 * L), (L, L2)
        assert abs(r - 12) < 1.5


def test_colour_mask_finds_a_dark_orange_ball():
    rgb = np.full((10, 10, 3), (8, 8, 10), np.uint8)
    rgb[3:7, 3:7] = (35, 18, 8)
    m = colour_mask(rgb, (5, 45), .35, .06)
    assert m[3:7, 3:7].all() and not m[0, 0]


def scene():
    """Base image at the reference camera pose + the ball's pixel; depth 2.0 m everywhere but the ball (1.6 m)."""
    rng = np.random.default_rng(3)
    grey = rng.integers(70, 190, (H // 16 + 1, W // 16 + 1, 1))                   # a room: mostly neutral, some tint
    img = np.clip(grey + rng.integers(-12, 13, (H // 16 + 1, W // 16 + 1, 3)), 0, 255).astype(np.uint8)
    img = cv2.resize(img, (W * 2, H * 2), interpolation=cv2.INTER_NEAREST)          # 2x canvas: room to rotate
    img = cv2.GaussianBlur(img, (0, 0), 3)
    tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9), 0, 80)
    side = 46                                                                         # px for 0.15 m at 2 m: 616*.15/2
    tag = cv2.resize(tag, (side * 7 // 5, side * 7 // 5), interpolation=cv2.INTER_NEAREST)   # 25h9: 5 data cells + border
    border = cv2.copyMakeBorder(tag, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=255)
    x0, y0 = W - 120, H - 60
    img[y0:y0 + border.shape[0], x0:x0 + border.shape[1]] = border[..., None]
    ball_uv, ball_r = (W + 140, H + 20), F * .0803 / 1.6
    cv2.circle(img, ball_uv, int(round(ball_r)), (220, 90, 15), -1, lineType=cv2.LINE_AA)
    depth = np.full((H * 2, W * 2), 2000, np.uint16)
    cv2.circle(depth, ball_uv, int(round(ball_r)) - 2, 1600 - 80, -1)
    return img, depth


def make_session(d: Path, seconds=5.5, fps=60):
    img, depth = scene()
    K = np.array([[F, 0, W / 2 + W / 2], [0, F, H / 2 + H / 2], [0, 0, 1]])      # canvas camera (centre of the 2x canvas)
    Kout = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])
    exposures, gains = [1000, 2000, 4000, 8000], [128, 128, 64, 64]
    (d / "colour").mkdir(parents=True)
    rows, drows, deps = [], [], []
    def rot(t):                                                     # still 2.2 s (every exposure), then a yaw sweep that speeds up
        if t < 2.2:
            return 0., 0.
        s = t - 2.2
        return .12 * math.sin(2 * math.pi * (.4 + .5 * s) * s), .03 * math.sin(2 * math.pi * .7 * s)
    def warp(src, yaw, pitch, interp):
        R = cv2.Rodrigues(np.array([pitch, yaw, 0.]))[0]
        Hm = Kout @ R @ np.linalg.inv(K)
        return cv2.warpPerspective(src, Hm, (W, H), flags=interp)
    n = int(seconds * fps)
    for i in range(n):
        t = i / fps
        k = int(t / .5) % 4
        since = t - int(t / .5) * .5
        e = exposures[k]
        subs = [rot(t + e * 1e-6 * (s / 8 - .5)) for s in range(9)]
        frame = np.mean([warp(img, y, p, cv2.INTER_LINEAR).astype(np.float32) for y, p in subs], axis=0)
        frame = np.clip(frame * min(1.5, e * gains[k] / (2000 * 128)), 0, 255).astype(np.uint8)
        name = f"colour/{i:05d}.jpg"
        cv2.imwrite(str(d / name), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
        rows.append(dict(file=name, number=i, t_ms=1000 * t, exposure_us=e, gain=gains[k], block=k,
                         since_switch_s=since, settling=since < .1))
        if i % 2 == 0:
            dep = warp(depth, *rot(t), cv2.INTER_NEAREST)[::2, ::2]
            deps.append(dep)
            drows.append(dict(number=i // 2, t_ms=1000 * t))
    (d / "colour.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    np.savez_compressed(d / "depth.npz", depth=np.stack(deps))
    (d / "depth.jsonl").write_text("".join(json.dumps(r) + "\n" for r in drows))
    calib = dict(color=dict(width=W, height=H, fx=F, fy=F, ppx=W / 2, ppy=H / 2),
                 depth=dict(width=W // 2, height=H // 2, fx=F / 2, fy=F / 2, ppx=W / 4, ppy=H / 4),
                 depth_scale=.001, depth_to_color=dict(R=list(np.eye(3).ravel()), t=[0, 0, 0]))
    (d / "session.json").write_text(json.dumps(dict(calib=calib, exposures_us=exposures, gains=gains)))


def test_analysis_recovers_truth_on_a_synthetic_turning_camera(tmp_path):
    d = tmp_path / "20261101_000000_synthetic"
    make_session(d)
    r = subprocess.run([sys.executable, str(HERE / "motion_analyze.py"), str(d), "--no-blob"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    res = json.loads((d / "motion_results.json").read_text())
    assert res["tag_seen"] > .8 * res["frames"]
    assert res["ball_spread_deg"] < .1
    slow = [x for x in res["frames_detail"] if x["speed"] < 100 and "colour_hit" in x]
    assert slow and np.mean([x["colour_hit"] for x in slow]) > .9
    fast = [x for x in res["frames_detail"] if x["speed"] > 600 and x.get("blur") == x.get("blur") and "blur" in x]
    assert fast, "the sweep must reach 600 px/s"
    ratio = np.median([x["blur"] / x["predicted_blur"] for x in fast if x["predicted_blur"] > 3])
    assert .6 < ratio < 1.4, ratio
