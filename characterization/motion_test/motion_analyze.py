"""Camera-motion test, analysis (PC, realsense venv: OpenCV): ball detection and blur against camera motion.

Truth comes from the AprilTag, not from the detectors under test: in frames where the camera is still (the tag's
corners move < `--still-px` between frames) the ball is found by colour and turned into a viewing direction of a
reference camera, using the camera's rotation recovered from the tag's corners. In every other frame that rotation
says where the ball must appear, and the change of that position between frames is the ball's image speed caused by
the camera's motion. Exact for a turning camera (what a robot's turns and rocking mostly are); a camera that also
shifts adds parallax between the tag and the ball (put the tag right next to the ball). Per exposure and image-speed
bin it reports how often each detector finds the ball (within `--gate` ball radii), the centroid error, the measured
blur length (the ball's mask is a stadium, 2r wide and L + 2r long: L = its length along the motion minus its width)
against speed x exposure, and the ball's brightness and saturation.

Detectors: `colour`, the robot's colour detector (omni_catch) with a colour window learned per exposure from the still
frames (what the lab's tuner would set); `blob`, the lab's colour-independent BlobProposer run over the frames in
order (its motion model sees the whole scene move when the camera moves).

  python motion_analyze.py <session dir> [--family 25h9] [--hue 5 45] [--no-blob]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (REPO / "ros/src/omni_calib", REPO / "ros/src/omni_catch", REPO / "characterization/ball_view",
          REPO / "characterization/throw_track", REPO / "characterization/realsense"):
    sys.path.insert(0, str(p))

SPEED_BINS = (0, 100, 300, 600, 1000, 2000, 1e9)          # px/s


STEP = .25                                                     # profile sampling, px
BAND = np.linspace(-1 / 3, 1 / 3, 7)                           # chords averaged, in ball radii across the line


def _profile(alpha, centre, e, half_len, r):
    """Coverage along direction e through centre, averaged over chords offset across by BAND * r."""
    import cv2
    ts = np.arange(-half_len, half_len + 1e-9, STEP)
    perp = (-e[1], e[0])
    offs = BAND * r
    xs = centre[0] + ts[None, :] * e[0] + offs[:, None] * perp[0]
    ys = centre[1] + ts[None, :] * e[1] + offs[:, None] * perp[1]
    vals = cv2.remap(alpha.astype(np.float32), xs.astype(np.float32), ys.astype(np.float32), cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return ts, vals.mean(axis=0)


def _models(ts, r, Ls, sigma):
    """Profiles of a disc (radius r, the BAND's chords) smeared by each L in Ls, softened by an optical sigma:
    each chord [-c, c] seen through a moving window [t - L/2, t + L/2] covers overlap / L."""
    import cv2
    c = r * np.sqrt(np.clip(1 - BAND ** 2, 0, None))                       # chord half-lengths
    L = np.maximum(np.asarray(Ls, float), 1e-3)[:, None, None]
    t = ts[None, None, :]
    lo, hi = np.maximum(t - L / 2, -c[None, :, None]), np.minimum(t + L / 2, c[None, :, None])
    m = (np.clip(hi - lo, 0, None) / L).mean(axis=1).astype(np.float32)   # (nL, nT)
    k = max(1, int(3 * sigma / STEP))
    g = np.exp(-.5 * (np.arange(-k, k + 1) * STEP / max(sigma, 1e-3)) ** 2).astype(np.float32)
    return cv2.sepFilter2D(m, -1, g / g.sum(), np.ones(1, np.float32), borderType=cv2.BORDER_CONSTANT)


def _fit(prof, models):
    """Best model index and its amplitude (coverage peak can be < 1 where colours mix), least squares."""
    num, den = models @ prof, (models * models).sum(axis=1)
    A = num / np.maximum(den, 1e-9)
    sse = (prof * prof).sum() - A * num
    k = int(np.argmin(sse))
    return k, float(A[k])


def blur_length(alpha: np.ndarray, direction=None, max_len=None) -> tuple[float, float]:
    """(L, r): smear length and disc radius from a ball-coverage map (0..1 per pixel), by fitting the smeared-disc
    profile: across the motion (no smear) for the radius and the optical softness, then along it for L. Moments
    (variance along minus across, r^2/4 + L^2/12 vs r^2/4) can't resolve a small smear (2 px on a 32 px ball is a
    0.13% change), and a thresholded mask can't either (at a 50% cut a disc blurred by L <= 2r stays ~2r long)."""
    ys, xs = np.nonzero(alpha > 0)
    w = alpha[ys, xs].astype(float)
    if w.sum() < 6:
        return float("nan"), float("nan")
    centre = ((xs * w).sum() / w.sum(), (ys * w).sum() / w.sum())
    r0 = math.sqrt(w.sum() / math.pi)
    n = float(np.hypot(*direction)) if direction is not None else 0.
    e = (direction[0] / n, direction[1] / n) if n > 1e-6 else (1., 0.)
    half = max(alpha.shape) * .75
    ts, across = _profile(alpha, centre, (-e[1], e[0]), half, r0)
    best = None
    for r in np.arange(.7, 1.31, .05) * r0:
        for sigma in (.4, .7, 1., 1.4, 2., 2.8):
            m = _models(ts, r, [0.], sigma)
            k, A = _fit(across, m)
            sse = float(((across - A * m[k]) ** 2).sum())
            if best is None or sse < best[0]:
                best = (sse, r, sigma)
    _, r, sigma = best
    ts, along = _profile(alpha, centre, e, half, r)
    Ls = np.arange(0, (max_len or 4 * r) + 1e-9, STEP)
    k, _ = _fit(along, _models(ts, r, Ls, sigma))
    return float(Ls[k]), float(r)


def coverage(sub: np.ndarray, ball_rgb, centre, half_width: float, direction=None, predicted=0.) -> np.ndarray:
    """Per-pixel ball coverage in a window: where each pixel's colour lies between the local background (the window's
    border, median) and the ball's colour, clipped to 0..1; only inside a strip along the image motion `direction`,
    `half_width` (the ball's radius + a margin) to each side and long enough for twice the `predicted` blur, so the
    background's texture stays out but a blur up to 2x the prediction can still be measured."""
    s = sub.astype(np.float32)
    border = np.concatenate((s[0], s[-1], s[:, 0], s[:, -1]))
    bg = np.median(border, axis=0)
    step = np.asarray(ball_rgb, np.float32) - bg
    n2 = float(step @ step)
    if n2 < 100:                                             # ball and background within ~10 levels: no contrast
        return np.zeros(s.shape[:2], np.float32)
    a = np.clip((s - bg) @ step / n2, 0, 1)
    a[a < .15] = 0                                           # background texture noise
    yy, xx = np.ogrid[:s.shape[0], :s.shape[1]]
    dx, dy = xx - centre[0], yy - centre[1]
    n = float(np.hypot(*direction)) if direction is not None else 0.
    if n < 1e-6:
        a[dx ** 2 + dy ** 2 > half_width ** 2] = 0
        return a
    ex, ey = direction[0] / n, direction[1] / n
    along, across = dx * ex + dy * ey, -dx * ey + dy * ex
    a[(np.abs(across) > half_width) | (np.abs(along) > half_width + predicted)] = 0
    return a


def colour_mask(rgb: np.ndarray, hue: tuple, min_sat: float, min_val: float) -> np.ndarray:
    a = rgb.astype(np.float32) / 255
    mx, mn = a.max(axis=2), a.min(axis=2)
    chroma = mx - mn
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    den = np.maximum(chroma, 1e-6)
    h = np.where(mx == r, (60 * (g - b) / den) % 360, np.where(mx == g, 60 * ((b - r) / den + 2), 60 * ((r - g) / den + 4)))
    inside = (h >= hue[0]) & (h <= hue[1]) if hue[0] <= hue[1] else (h >= hue[0]) | (h <= hue[1])
    return inside & (chroma / np.maximum(mx, 1e-6) >= min_sat) & (mx >= min_val)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--tag-id", type=int)
    ap.add_argument("--family", default="25h9", help="AprilTag family (Fernando's print, Nov 1: 25h9 id 0, ~140 mm black square)")
    ap.add_argument("--hue", type=float, nargs=2, default=(5, 45), help="ball hue window for the truth's still frames")
    ap.add_argument("--min-sat", type=float, default=.35)
    ap.add_argument("--min-val", type=float, default=.06)
    ap.add_argument("--still-px", type=float, default=.6, help="tag corner motion per frame below which the camera is still")
    ap.add_argument("--gate", type=float, default=.6, help="a detection within this many ball radii of the truth is a hit")
    ap.add_argument("--no-blob", action="store_true", help="skip the (slow) BlobProposer")
    a = ap.parse_args()

    import cv2
    from PIL import Image
    from bucket_vision import BlobProposer
    from omni_calib.detect import apriltags
    from bucket_workflow import colour_window
    from omni_catch.detector import Detector, DetectorConfig
    from omni_catch.frames import ArrayFrame

    d = a.session
    sess = json.loads((d / "session.json").read_text())
    cal = sess["calib"]
    f = cal["color"]["fx"]
    rows = [json.loads(s) for s in (d / "colour.jsonl").read_text().splitlines()]

    def load(i):
        return np.asarray(Image.open(d / rows[i]["file"]))

    # 1. The camera's rotation in every frame, from the tag's four corners. Not the tag's 3D pose (a small tag seen
    # nearly square-on has a tilt ambiguity of degrees) and not a homography fitted to the corners (8 parameters from
    # a ~50 px square, extrapolated to a ball a few tag-widths away: 5-15 px off in the synthetic test). A turning
    # camera is a rotation: 3 parameters from the corners' viewing directions (Wahba / SVD), exact for any depth.
    # A camera that also shifts adds parallax between the tag and the ball: put the tag next to the ball.
    Kinv = np.linalg.inv(np.array([[f, 0, cal["color"]["ppx"]], [0, cal["color"]["fy"], cal["color"]["ppy"]], [0, 0, 1.]]))
    K = np.linalg.inv(Kinv)

    def rays(px):
        v = (Kinv @ np.column_stack((px, np.ones(len(px)))).T).T
        return v / np.linalg.norm(v, axis=1, keepdims=True)

    def pixel(ray):
        q = K @ ray
        return q[0] / q[2], q[1] / q[2]

    corners, ref = [], None
    for i, r in enumerate(rows):
        gray = cv2.cvtColor(load(i), cv2.COLOR_RGB2GRAY)
        found = [t for t in apriltags(gray, a.family) if a.tag_id is None or t[0] == a.tag_id]
        if not found:
            corners.append(None)
            continue
        tid, c, _ = max(found, key=lambda t: cv2.contourArea(t[1].astype(np.float32)))
        a.tag_id = int(tid)
        corners.append(c.astype(float))
    seen = sum(c is not None for c in corners)
    print(f"{len(rows)} frames, tag {a.tag_id} seen in {seen}")
    if seen < 10:
        sys.exit("The tag is hardly seen: check the print, its size in the view and the light")
    ref = rays(next(c for c in corners if c is not None))
    rotation = []                                              # R_i: reference camera -> frame i
    for c in corners:
        if c is None:
            rotation.append(None)
            continue
        U, _, Vt = np.linalg.svd(rays(c).T @ ref)
        rotation.append(U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt)

    # 2. Still frames: the ball by colour, as a viewing direction in the reference camera.
    corner_motion = [None] + [float(np.abs(corners[i] - corners[i - 1]).max())
                              if corners[i] is not None and corners[i - 1] is not None else None
                              for i in range(1, len(rows))]
    seen_ball = []
    for i, r in enumerate(rows):
        if r["settling"] or corner_motion[i] is None or corner_motion[i] > a.still_px:
            continue
        m = colour_mask(load(i), a.hue, a.min_sat, a.min_val).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        if n < 2:
            continue
        k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        if st[k, cv2.CC_STAT_AREA] < 40:
            continue
        ray = rotation[i].T @ rays(np.array([cen[k]]))[0]
        seen_ball.append(np.r_[ray, math.sqrt(st[k, cv2.CC_STAT_AREA] / math.pi) / f])   # + angular radius
    if len(seen_ball) < 5:
        sys.exit(f"Only {len(seen_ball)} still frames with the ball found by colour: hold the camera still a moment, "
                 "ball and tag in view (or check --hue)")
    seen_ball = np.array(seen_ball)
    ball_ray = np.median(seen_ball[:, :3], axis=0)
    ball_ray /= np.linalg.norm(ball_ray)
    ball_ang_r = float(np.median(seen_ball[:, 3]))
    spread = float(np.degrees(np.median(np.arccos(np.clip(seen_ball[:, :3] @ ball_ray, -1, 1)))))
    print(f"ball direction from {len(seen_ball)} still frames, median spread {spread:.3f} deg "
          f"({spread * math.pi / 180 * f:.1f} px)")

    # 3. Truth per frame: pixel, radius, image speed (and the turn rate that would cause it, speed / f).
    truth = [None] * len(rows)
    for i, R in enumerate(rotation):
        if R is not None:
            u, v = pixel(R @ ball_ray)
            truth[i] = (float(u), float(v), ball_ang_r * f)
    speed, omega = [None] * len(rows), [None] * len(rows)
    for i in range(1, len(rows)):
        if truth[i] and truth[i - 1]:
            dt = (rows[i]["t_ms"] - rows[i - 1]["t_ms"]) / 1000
            if 0 < dt < .05:
                speed[i] = math.hypot(truth[i][0] - truth[i - 1][0], truth[i][1] - truth[i - 1][1]) / dt
                omega[i] = math.degrees(speed[i] / f)

    # 4. Colour windows per exposure, learned on still frames at the truth (what the lab's tuner would do).
    windows = {}
    for e in sess["exposures_us"]:
        px = []
        for i, r in enumerate(rows):
            if r["exposure_us"] == e and not r["settling"] and truth[i] and (speed[i] or 0) < 60:
                u, v, rr = truth[i]
                rgb = load(i)
                yy, xx = np.ogrid[:rgb.shape[0], :rgb.shape[1]]
                px.append(rgb[(xx - u) ** 2 + (yy - v) ** 2 <= (.55 * rr) ** 2])
            if len(px) >= 15:
                break
        w = colour_window(np.concatenate(px)) if px else None
        if w:
            w["ball_rgb"] = np.median(np.concatenate(px), axis=0).tolist()
        windows[e] = w
        print(f"exposure {e:5d} us: colour window " + (f"hue {w['hue_min']:.0f}-{w['hue_max']:.0f}, sat >= {w['sat_min']:.2f}, "
                                                   f"val >= {w['val_min']:.2f}, ball brightness {w['brightness']:.2f}" if w else "none (ball not found still)"))

    # 5. Detectors and blur per frame.
    blob = None if a.no_blob else BlobProposer()
    out = []
    for i, r in enumerate(rows):
        rgb = load(i)
        cands_blob = blob(rgb) if blob else []
        if r["settling"] or truth[i] is None or speed[i] is None:
            continue
        e = r["exposure_us"]
        u, v, rr = truth[i]
        rec = dict(i=i, exposure_us=e, gain=r["gain"], speed=speed[i], omega_dps=omega[i], truth=[u, v, rr],
                   predicted_blur=speed[i] * e * 1e-6)
        w = windows.get(e)
        if w and 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]:
            # A red-orange window can cross 0 deg (Nov 1: 355-28): the detector takes one interval, so run both halves,
            # as the bucket lab does.
            spans = [(w["hue_min"], w["hue_max"])] if w["hue_min"] <= w["hue_max"] else [(w["hue_min"], 360.), (0., w["hue_max"])]
            cs = []
            for span in spans:
                cfg = DetectorConfig(hue_deg=span, min_sat=w["sat_min"], min_val=w["val_min"], min_blob_px=8)
                cs += Detector(cfg).candidates(ArrayFrame(rgb), 16)
            near = [c for c in cs if math.hypot(c[0] - u, c[1] - v) < max(4., a.gate * rr) + .5 * rec["predicted_blur"]]
            rec["colour_hit"] = bool(near)
            rec["colour_err"] = math.hypot(near[0][0] - u, near[0][1] - v) if near else None
            reach = rr * 1.3 + rec["predicted_blur"] / 2 + 3
            h = int(reach + rr + 4)
            y0, x0 = max(0, int(v) - h), max(0, int(u) - h)
            sub = rgb[y0:int(v) + h, x0:int(u) + h]
            j = i - 1 if truth[i - 1] else i
            direction = np.array(truth[i][:2]) - np.array(truth[j][:2])
            alpha = coverage(sub, w["ball_rgb"], (u - x0, v - y0), rr + 2, direction, rec["predicted_blur"])
            if alpha.sum() > 6:
                L, rm = blur_length(alpha, direction, max_len=2 * rec["predicted_blur"] + 2 * rr)
                rec["blur"], rec["coverage_r"] = L, rm
                core = sub[alpha >= min(.8, float(alpha.max()))].astype(float) / 255   # the least mixed pixels
                mx = core.max(axis=1)
                rec["brightness"] = float(np.median(mx))
                rec["saturation"] = float(np.median((mx - core.min(axis=1)) / np.maximum(mx, 1e-6)))
                rec["peak_coverage"] = float(alpha.max())
        if blob:
            near = [c for c in cands_blob if math.hypot(c[0] - u, c[1] - v) < max(4., a.gate * rr) + .5 * rec["predicted_blur"]]
            rec["blob_hit"] = bool(near)
            rec["blob_err"] = math.hypot(near[0][0] - u, near[0][1] - v) if near else None
        out.append(rec)

    # 6. Table.
    def med(xs):
        xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
        return float(np.median(xs)) if xs else float("nan")
    table = []
    print("\nexposure  speed px/s   n  colour%  blob%  err px  blur px (pred)  brightness  sat   camera deg/s")
    for e in sess["exposures_us"]:
        for lo, hi in zip(SPEED_BINS[:-1], SPEED_BINS[1:]):
            g = [x for x in out if x["exposure_us"] == e and lo <= x["speed"] < hi]
            if not g:
                continue
            row = dict(exposure_us=e, speed_lo=lo, speed_hi=hi, n=len(g),
                       colour_rate=float(np.mean([x.get("colour_hit", False) for x in g])),
                       blob_rate=float(np.mean([x.get("blob_hit", False) for x in g])) if blob else None,
                       colour_err_px=med([x.get("colour_err") for x in g]), blur_px=med([x.get("blur") for x in g]),
                       predicted_blur_px=med([x["predicted_blur"] for x in g]), brightness=med([x.get("brightness") for x in g]),
                       saturation=med([x.get("saturation") for x in g]), camera_dps=med([x["omega_dps"] for x in g]))
            table.append(row)
            hi_s = f"{hi:.0f}" if hi < 1e8 else "+"
            print(f"{e:6d}    {lo:5.0f}-{hi_s:<5}  {row['n']:3d}  {100 * row['colour_rate']:5.0f}  "
                  f"{100 * row['blob_rate'] if blob else float('nan'):5.0f}  {row['colour_err_px']:6.1f}  "
                  f"{row['blur_px']:6.1f} ({row['predicted_blur_px']:5.1f})   {row['brightness']:6.2f}    {row['saturation']:.2f}   {row['camera_dps']:6.0f}")
    res = dict(session=str(d), tag_id=a.tag_id, tag_seen=seen, frames=len(rows),
               ball_ray=ball_ray.tolist(), ball_spread_deg=spread, still_frames=len(seen_ball),
               windows={str(k): v for k, v in windows.items()}, table=table, frames_detail=out)
    (d / "motion_results.json").write_text(json.dumps(res, indent=1, default=float))
    print(f"\nwritten {d / 'motion_results.json'}")


if __name__ == "__main__":
    main()
