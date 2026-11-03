"""Replay a RealSense throw session offline: every imager's ball detections, an arc through each, and how well they agree.

For each throw: depth blobs (foreground against the empty scene), IR blobs matched across the pair (blob stereo), and the robot's
colour detector; a robust arc through each source's 3D points (|a| should be near g: a check on clocks and scale); the sources
against each other (arc-to-arc distance), the colour pixels against the depth arc projected through the factory extrinsics, and
the lag between colour's clock and the depth's that best aligns them (rolling shutter, exposure and clock mapping all show up
there). No labels: this is the "path detection" half; the bucket and the rim verdict come on top (rs_predict, once the bucket's
numbers are in).

  python rs_analyze.py data/<session> [--throw 3] [--plot] [--set depth_gap=0.3 ir_thr=25] [--bg setup]

Run it again with other --set values on the same recording to compare filters. Results: results/rs_analysis_<session>.json.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, fields
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
for p in (HERE.parents[1] / "ros" / "src" / "omni_catch",):
    if p.is_dir():
        sys.path.insert(0, str(p))

import rs_detect as D           # noqa: E402
import session as S             # noqa: E402

RESULTS = HERE / "results"
G = 9.81


def calibration(sess: dict) -> dict:
    rs = sess["rs"]
    st = rs["streams"]
    intr = {n: D.Intr.of(v["intrinsics"]) for n, v in st.items()}
    for n in ("ir1", "ir2"):                      # the IR pair is rectified: depth's intrinsics are IR1's
        intr.setdefault(n, intr.get("depth"))
    ext = {k: D.extrinsics_matrix(v) for k, v in rs.get("extrinsics", {}).items()}
    return dict(intr=intr, scale=float(rs.get("depth_scale", 0.001)), baseline=float(rs.get("baseline_m", 0.05)), ext=ext)


def detector_cfg(sess: dict):
    from omni_catch.detector import DetectorConfig
    d = dict(sess.get("detector") or {})
    cfg = DetectorConfig(backend="native")
    for k, v in d.items():
        if hasattr(cfg, k) and k != "backend":
            setattr(cfg, k, tuple(v) if isinstance(v, list) else v)
    return cfg


def n_bg(t: np.ndarray, cfg: D.DetectCfg) -> int:
    return int(max(3, np.sum(t < t[0] + cfg.bg_s)))


def scene_still(d: Path, name: str):
    f = Path(d) / f"setup_{name}.npy"
    return np.load(f) if f.exists() else None


def nearest(t_src: np.ndarray, t_q: np.ndarray) -> np.ndarray:
    i = np.clip(np.searchsorted(t_src, t_q), 1, len(t_src) - 1)
    return np.where(np.abs(t_src[i - 1] - t_q) <= np.abs(t_src[i] - t_q), i - 1, i)


def source_summary(name: str, t_all: np.ndarray, fit: D.ArcFit | None, picks: list, n_cand: int) -> dict:
    out = dict(source=name, frames=len(t_all), candidates=n_cand)
    if fit is None:
        return dict(out, ok=False)
    got = [i for i, p in enumerate(picks) if p is not None]
    z = [picks[i][2][2] for i in got]
    out.update(ok=True, detections=len(got), detection_rate=len(got) / (got[-1] - got[0] + 1),       # of the frames from first to last seen
               first_t=float(t_all[got[0]]), last_t=float(t_all[got[-1]]), first_range_m=float(z[0]), last_range_m=float(z[-1]),
               rms_m=fit.rms, speed_mps=fit.speed(), a_norm=fit.a_quad if fit.g is not None else float(np.linalg.norm(fit.a)),
               a_vs_gravity_pct=float(100 * ((fit.a_quad if fit.g is not None else np.linalg.norm(fit.a)) - G) / G),
               drag_arc=fit.g is not None, up_in_camera=None if fit.g is None else fit.up().tolist())
    return out


def colour_check(throw: S.Throw, cal: dict, sess: dict, fit_d: D.ArcFit | None, lag_range_ms=40.0, max_px: float = 25.0) -> dict:
    """Colour detections (the robot's detector) against the depth arc projected into the colour camera."""
    from omni_catch.detector import Detector
    from omni_catch.frames import ArrayFrame
    try:
        from omni_catch.fast_detector import FastDetector, available
        det = FastDetector(detector_cfg(sess)) if available() else Detector(detector_cfg(sess))
    except Exception:
        det = Detector(detector_cfg(sess))
    t = throw.t_of("color")
    cands = []
    for i in range(len(t)):
        cs = det.candidates(ArrayFrame(np.asarray(throw.frames[i])), 4)
        cands.append([(float(c[0]), float(c[1]), float(c[2])) for c in cs])
    n_with = sum(1 for c in cands if c)
    out = dict(frames=len(t), frames_with_a_blob=n_with)
    if fit_d is None or "depth_to_color" not in cal["ext"] or "color" not in cal["intr"]:
        return dict(out, projected=False)
    R, tr = cal["ext"]["depth_to_color"]
    ic = cal["intr"]["color"]

    def predicted(tq, lag_s):
        p = fit_d.at(np.asarray(tq) + lag_s) @ R.T + tr
        return ic.project(p)

    def residuals(lag_s):
        pred = predicted(t, lag_s)
        res = []
        for i, cs in enumerate(cands):
            if not cs or not np.all(np.isfinite(pred[i])) or fit_d.at(t[i] + lag_s)[2] <= 0.1:
                continue
            d = [math.hypot(c[0] - pred[i][0], c[1] - pred[i][1]) for c in cs]
            if min(d) <= max_px:
                res.append(min(d))
        return res

    lags = np.arange(-lag_range_ms, lag_range_ms + 0.1, 2.0) * 1e-3
    rms = []
    for lg in lags:
        r = residuals(lg)
        rms.append(math.sqrt(np.mean(np.square(r))) if len(r) >= 5 else float("nan"))
    rms = np.array(rms)
    r0 = residuals(0.0)
    out.update(projected=True, matched_at_zero_lag=len(r0), rms_px_at_zero_lag=float(math.sqrt(np.mean(np.square(r0)))) if len(r0) >= 5 else None)
    if np.isfinite(rms).any():
        k = int(np.nanargmin(rms))
        out.update(best_lag_ms=float(lags[k] * 1e3), rms_px_at_best_lag=float(rms[k]), lag_curve_ms=[float(x) for x in lags * 1e3],
                   lag_curve_rms_px=[None if not np.isfinite(x) else float(x) for x in rms])
    return out


def colour_detector(sess: dict):
    from omni_catch.detector import Detector
    try:
        from omni_catch.fast_detector import FastDetector, available
        return FastDetector(detector_cfg(sess)) if available() else Detector(detector_cfg(sess))
    except Exception:
        return Detector(detector_cfg(sess))


SIG_Z_SIZE = 0.12          # relative range noise from the blob's size (a dim rim cut by the threshold, blur; est)


def sig_z_depth(z):
    """The D435's range noise on the ball, m (est: 1 cm + 1 % x z^2, about the RMS of 0.08 px subpixel disparity at f 430, B 50 mm, plus
    the foam's surface)."""
    return 0.01 + 0.01 * np.asarray(z, float) ** 2


def grow_flight(t, cands: list, core: list, observe, ic, R, tr, ball, up=None, trim_px: float = 4.0, trim_r: float = 0.3,
                gate_px: float = 4.0, gate_r: float = 0.6, r_cap: float = 12.0, refit_every: int = 3, max_gap: int = 12, min_pts: int = 8,
                tracklets: list | None = None, link_gap: int = 30) -> tuple:
    """The thrown ball's 3D arc from a clean 2D flight `core` [(frame, u, v, r)], by predict-and-search over every frame's blobs `cands`
    (fit_ray_arc: perspective is exact, so it keeps the ball as it nears the camera, where the 2D test fails).

      1. trim: drop the core's worse end while the arc misses it by more than trim_px + trim_r x radius, refit (a 2D window that passes
         takes a few frames of the ball still in the hand with it: on the first multi-throw session the arc was 12-56 px off them).
      2. grow forward, then backward: in each frame the blob nearest the arc's prediction within gate_px + gate_r x the PREDICTED radius
         (a blob cut by the lamp's glare is a fragment: its own size is wrong, its centroid off by up to ~r) joins; refit every few points;
         up to max_gap frames with no blob are bridged (the ball crossing the ceiling lamp: ~6 frames on Oct 30). The radius in both gates
         is capped at r_cap px (near the camera the ball is 30-50 px and an uncapped gate took any blob), and growing stops where the
         predicted ball touches the image's edge (the detector drops blobs cut by the border: what's left there are fragments).
      2b. link: when growing forward stops, a later 2D tracklet (starting within link_gap frames) joins if the arc refitted with it fits
         its points within 1.5 x the trim gate (median) and the old points no worse: after the ceiling lamp's glare the ball comes back
         35-43 px from a short arc's extrapolation, but one arc fits both sides to a few px (Oct 30, a close toss). Then growing resumes.
      3. clean: the worst point goes while it misses by more than the trim gate (or 3 robust sigmas, if larger), refit each time.

    observe(frame, u, v, r, z_guess) -> (u, v, r, z, sigma_z, source). Returns (frames, {frame: obs}, fit) or (frames, obs, None)."""
    pts = {i: observe(i, u, v, r, None) for i, u, v, r in core}

    def fitk(keys, x0=None):
        o = [pts[k] for k in keys]
        return D.fit_ray_arc(t[keys], [(a[0], a[1]) for a in o], [a[3] for a in o], [a[4] for a in o], ic, R, tr, ball, up=up,
                             sig_px=[1.0 + 0.05 * a[2] for a in o], x0=x0)

    def pred(i, f):
        p = f.at(np.array([t[i]]))[0]
        pc = p @ R.T + tr
        if pc[2] <= 0.1:
            return None
        u, v = ic.project(pc)
        return u, v, ic.fx * ball.radius / pc[2], p[2]

    keys = sorted(pts)
    fit, x = fitk(keys)
    if fit is None:
        return keys, pts, None

    def miss_px(k, f):
        pr = pred(k, f)
        return float("inf") if pr is None else math.hypot(pr[0] - pts[k][0], pr[1] - pts[k][1])

    def trim_gate(k, f):
        pr = pred(k, f)
        return trim_px + trim_r * min(pr[2], r_cap) if pr else 0.0

    def refit(ks):
        nonlocal fit, x
        f2, x2 = fitk(ks, x)
        if f2 is not None:
            fit, x = f2, x2

    # 1. trim, one point at a time, the worse end (a fit pulled by the hand's points misses the good end too)
    for _ in range(40):
        a_, b_ = (miss_px(k, fit) - trim_gate(k, fit) for k in (keys[0], keys[-1]))
        if len(keys) <= 6 or max(a_, b_) <= 0:
            break
        pts.pop(keys.pop(0 if a_ >= b_ else -1))
        refit(keys)

    # 2. grow
    def grow(step):
        nonlocal keys
        i, gap, added = (keys[-1] + 1 if step > 0 else keys[0] - 1), 0, 0
        while 0 <= i < len(t) and gap < max_gap:
            pr = pred(i, fit)
            if pr is None or pr[0] - pr[2] < 0 or pr[1] - pr[2] < 0 or pr[0] + pr[2] > ic.width or pr[1] + pr[2] > ic.height:
                break
            best = None
            for u, v, r in cands[i]:
                dd = math.hypot(u - pr[0], v - pr[1])
                if dd <= gate_px + gate_r * min(pr[2], r_cap) and (best is None or dd < best[0]):
                    best = (dd, u, v, r)
            if best is None:
                gap += 1
            else:
                pts[i] = observe(i, best[1], best[2], best[3], pr[3])
                keys = sorted(pts)
                gap, added = 0, added + 1
                if added % refit_every == 0:
                    refit(keys)
            i += step
        refit(keys)

    # 2b. link a later tracklet (the ball back after the lamp's glare)
    def link() -> bool:
        nonlocal keys, fit, x
        last = keys[-1]
        g = 1.5 * (trim_px + trim_r * r_cap)
        for tl in sorted(tracklets or [], key=lambda o: o["frames"][0]):
            new = [(i, u, v, r) for i, (u, v), r in zip(tl["frames"], tl["uv"], tl["r"]) if i > last]
            if len(new) < 4 or new[0][0] > last + link_gap:
                continue
            saved = dict(pts)
            for i, u, v, r in new:
                pr = pred(i, fit)
                pts[i] = observe(i, u, v, r, pr[3] if pr else None)
            ks = sorted(pts)
            f2, x2 = fitk(ks, x)
            if f2 is not None and np.median([miss_px(i, f2) for i, *_ in new]) <= g and np.median([miss_px(k, f2) for k in keys]) <= g:
                fit, x, keys = f2, x2, ks
                return True
            pts.clear()
            pts.update(saved)
        return False

    grow(1)
    for _ in range(3):
        if not link():
            break
        grow(1)
    grow(-1)

    # 3. clean: the worst point while it misses by more than its trim gate and 3 robust sigmas
    for _ in range(len(keys)):
        if len(keys) <= min_pts:
            break
        res = np.array([miss_px(k, fit) for k in keys])
        lim = np.maximum([trim_gate(k, fit) for k in keys], 3 * 1.4826 * float(np.median(res)))
        worst = int(np.argmax(res - lim))
        if res[worst] <= lim[worst]:
            break
        pts.pop(keys.pop(worst))
        refit(keys)
    return keys, pts, fit


def colour_source(throw: S.Throw, sess: dict, cal: dict, cfg: D.DetectCfg, n_cands: int = 8, up=None):
    """The colour imager as a 3D source: the session's detector (its colour window and size_gain) on every colour frame, the thrown ball's
    clean 2D flight (arc2d.flight_track), then its 3D arc by predict-and-search (grow_flight); each point's range from the depth map at its
    pixel (depth_at_pixel; nearest depth frame in time) or from its size, and none (pixels only) when the size disagrees with the arc (a
    glare fragment). Returns (t, picks, fit, obs, info)."""
    import arc2d
    from omni_catch.frames import ArrayFrame
    t = throw.t_of("color")
    det = colour_detector(sess)
    cands = [[(float(c[0]), float(c[1]), float(c[2])) for c in det.candidates(ArrayFrame(np.asarray(throw.frames[i])), n_cands)]
             for i in range(len(t))]
    picks = [None] * len(t)
    info = dict(frames=len(t), frames_with_a_blob=sum(1 for c in cands if c), cands=cands)        # cands: popped by analyze_throw (rs_review draws them)
    acfg = arc2d.ArcCfg(ball_radius=cfg.ball_radius)
    fl = arc2d.flight_track(t, cands, acfg)
    if fl is None or "color" not in cal["intr"]:
        return t, picks, None, None, dict(info, ok=False, reason="no blob track follows an arc")
    ic = cal["intr"]["color"]
    if "depth_to_color" in cal["ext"]:
        R, tr = cal["ext"]["depth_to_color"]
    else:
        R, tr = np.eye(3), np.zeros(3)
    have_depth = "depth" in throw.names and "depth" in cal["intr"]
    if have_depth:
        td, dst = throw.t_of("depth"), throw.stream("depth")

    def observe(i, u, v, r, z_guess):
        z_size = ic.fx * cfg.ball_radius / max(r, 0.5)
        g = z_size if z_guess is None else z_guess
        z_d = float("nan")
        if have_depth:
            j = int(nearest(td, np.array([t[i]]))[0])
            if abs(td[j] - t[i]) < 0.008:
                z_d = D.depth_at_pixel(u, v, max(r, 0.6 * ic.fx * cfg.ball_radius / g), dst.frames[j], cal["scale"], ic, cal["intr"]["depth"],
                                       R, tr, cfg.ball_radius, g)
        if np.isfinite(z_d):
            return (u, v, r, z_d, float(sig_z_depth(z_d)), "depth")
        if have_depth and z_guess is None and abs(td[j] - t[i]) < 0.008 and D.depth_in_front(u, v, r, dst.frames[j], cal["scale"], ic,
                                                                                            cal["intr"]["depth"], R, tr, z_size):
            return (u, v, r, z_size, SIG_Z_SIZE * z_size, "in_front")      # on a nearer object: counted against the track below
        if z_guess is not None and abs(z_size / z_guess - 1) > 0.3:
            return (u, v, r, z_guess, 1e3, "none")                    # a fragment's size is no range: pixels only
        return (u, v, r, z_size, SIG_Z_SIZE * z_size, "size")

    still = arc2d.static_mask(cands)
    live = [[c for c, s in zip(cs, ss) if not s and c[2] >= acfg.min_r_px * 0.5] for cs, ss in zip(cands, still)]
    core = [(i, u, v, r) for i, (u, v), r in zip(fl["frames"], fl["uv"], fl["r"])]
    n_front = 0
    for i, u, v, r in core:
        n_front += observe(i, u, v, r, None)[5] == "in_front"
    if n_front >= 0.5 * len(core):
        return t, picks, None, None, dict(info, ok=False, reason=f"the blob sits on a nearer object in {n_front}/{len(core)} frames (depth much "
                                                              "nearer than its size: a hand or an arm, not a free ball)")
    keys, pts, fit = grow_flight(t, live, core, observe, ic, R, tr, cfg.ball, up=up, tracklets=fl.get("tracklets"))
    o = [pts[k] for k in keys]
    tt = t[keys]
    uv, zs, sz = np.array([(a[0], a[1]) for a in o]), np.array([a[3] for a in o]), np.array([a[4] for a in o])
    rs_ = np.array([a[2] for a in o])
    src = [a[5] for a in o]
    obs = dict(t=tt, uv=uv, z=zs, sig_z=sz, sig_px=1.0 + 0.05 * rs_, R=R, tr=tr, ic=ic)
    if fit is None:
        return t, picks, None, obs, dict(info, ok=False, reason="the arc fit failed")
    P = fit.at(tt)
    for k, i in enumerate(keys):
        picks[i] = (0.0, D.Cand(uv[k, 0], uv[k, 1], zs[k], rs_[k], 0, 0.0), P[k])
    z_size_all = ic.fx * cfg.ball_radius / np.maximum(rs_, 0.5)
    with_d = np.array([s == "depth" for s in src])
    gaps = int(np.sum(np.diff(keys) - 1)) if len(keys) > 1 else 0
    info.update(ok=True, flight_frames=len(keys), core_frames=len(core), bridged_frames=gaps, range_from_depth=int(with_d.sum()),
                range_none=int(sum(s == "none" for s in src)), rms_px=fit.rms_px,
                size_over_depth=float(np.median(z_size_all[with_d] / zs[with_d])) if with_d.any() else None)
    return t, picks, fit, obs, info


def with_session_ball(cfg: D.DetectCfg, sess: dict) -> D.DetectCfg:
    """cfg with the session's ball (mass, radius, drag) for the drag law and the blob-size model: the robot config's ball updated by
    session.json's "ball" (analyze_throws.ball_of's rule). A session without one keeps cfg as it is."""
    if not sess.get("ball"):
        return cfg
    import dataclasses
    import analyze_throws as AT
    ball = AT.ball_of(sess)
    return dataclasses.replace(cfg, ball=ball, ball_radius=ball.radius)


def analyze_throw(throw: S.Throw, sess: dict, cfg: D.DetectCfg, bg_source: str = "throw", colour: bool = True, session_dir: Path | None = None,
                  session_ball: bool = True, up=None) -> dict:
    """up: world up in the camera frame (the mount's, bucket.json): the colour arc's gravity direction. A short head-on monocular track
    doesn't pin it (fitted free on the first real throw it came out 50 deg off and the arc missed the ball's last 20 frames)."""
    if session_ball:
        cfg = with_session_ball(cfg, sess)
    cal = calibration(sess)
    names = throw.names
    out = dict(path=str(throw.path), streams=names, sources={})
    keep = {}

    # --- depth
    fit_d = None
    if "depth" in names:
        st = throw.stream("depth")
        t = throw.t_of("depth")
        still = scene_still(session_dir, "depth") if (bg_source == "setup" and session_dir) else None
        bg = D.background(still[None], cal["scale"]) if still is not None else D.background(st.frames[:n_bg(t, cfg)], cal["scale"])
        cands = [D.depth_candidates(st.frames[i], cal["scale"], bg, cal["intr"]["depth"], cfg) for i in range(len(t))]
        fi, tt, P, C = D.collect(t, cands, cal["intr"]["depth"])
        fit_d = D.fit_arc(tt, P, cfg)
        picks = D.pick_per_frame(fit_d, fi, tt, P, C, len(t), cfg.inlier_m) if fit_d else [None] * len(t)
        out["sources"]["depth"] = source_summary("depth", t, fit_d, picks, len(C))
        keep["depth"] = (t, picks, fit_d)

    # --- IR blob stereo
    fit_i = None
    if "ir1" in names and "ir2" in names:
        s1, s2 = throw.stream("ir1"), throw.stream("ir2")
        t1, t2 = throw.t_of("ir1"), throw.t_of("ir2")
        j2 = nearest(t2, t1)
        nb = n_bg(t1, cfg)
        bg1, bg2 = D.background(s1.frames[:nb]), D.background(s2.frames[:nb])
        cands = []
        for i in range(len(t1)):
            c1 = D.ir_candidates(s1.frames[i], bg1, cfg)
            c2 = D.ir_candidates(s2.frames[j2[i]], bg2, cfg) if abs(t2[j2[i]] - t1[i]) < 2e-3 else []
            cands.append(D.stereo_match(c1, c2, cal["intr"]["ir1"], cal["baseline"], cfg, cal["intr"]["ir2"].ppx))
        fi, tt, P, C = D.collect(t1, cands, cal["intr"]["ir1"])
        fit_i = D.fit_arc(tt, P, cfg)
        picks = D.pick_per_frame(fit_i, fi, tt, P, C, len(t1), cfg.inlier_m) if fit_i else [None] * len(t1)
        out["sources"]["ir_stereo"] = source_summary("ir_stereo", t1, fit_i, picks, len(C))
        keep["ir_stereo"] = (t1, picks, fit_i)

    # --- the sources against each other
    if fit_d and fit_i:
        lo, hi = max(fit_d.t_ref, fit_i.t_ref), min(keep["depth"][0][-1], keep["ir_stereo"][0][-1])
        tq = np.linspace(lo, hi, 40)
        if hi > lo:
            d = np.linalg.norm(fit_d.at(tq) - fit_i.at(tq), axis=1)
            out["depth_vs_ir_stereo"] = dict(rms_m=float(np.sqrt(np.mean(d ** 2))), max_m=float(d.max()),
                                             mean_offset_m=(fit_i.at(tq) - fit_d.at(tq)).mean(axis=0).tolist(), t_span_s=float(hi - lo))

    # --- colour: the thrown ball's track, range from the depth map at its pixel (or its size), the arc through the pixels
    if colour and throw.frames is not None:
        t_c, picks, fit_c, obs, info = colour_source(throw, sess, cal, cfg, up=up)
        out["_cands"] = info.pop("cands")
        out["sources"]["colour"] = dict(source_summary("colour", t_c, fit_c, picks, info["frames_with_a_blob"]), **info)
        if fit_c is not None:
            keep["colour"] = (t_c, picks, fit_c)
            out["_obs"] = dict(colour=obs)
        out["colour"] = colour_check(throw, cal, sess, fit_d)
    out["_keep"] = keep
    return out


def plot_throw(res: dict, path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    keep = res["_keep"]
    pred = {k: v for k, v in (res.get("prediction") or {}).items() if v.get("ok") and v.get("series")}
    fig, ax = plt.subplots(1, 4 if pred else 3, figsize=(18 if pred else 14, 3.6))
    colours = dict(depth="tab:blue", ir_stereo="tab:red", colour="tab:orange")
    for name, (t, picks, fit) in keep.items():
        got = [(t[i], p[2]) for i, p in enumerate(picks) if p is not None]
        if got:
            tg = np.array([g[0] for g in got])
            P = np.array([g[1] for g in got])
            ax[0].plot(tg, P[:, 2], ".", ms=4, color=colours[name], label=f"{name} ({len(got)})")
            ax[1].plot(tg, P[:, 0], ".", ms=4, color=colours[name])
            ax[1].plot(tg, -P[:, 1], "x", ms=3, color=colours[name])
        if fit is not None:
            tq = np.linspace(tg[0], tg[-1], 100)
            pq = fit.at(tq)
            ax[0].plot(tq, pq[:, 2], "-", lw=0.8, color=colours[name])
    ax[0].set_title("range z (m)")
    ax[0].legend(fontsize=7)
    ax[1].set_title("x (dots), up = -y (x), m")
    c = res.get("colour", {})
    if c.get("lag_curve_ms"):
        ax[2].plot(c["lag_curve_ms"], [np.nan if v is None else v for v in c["lag_curve_rms_px"]], "-o", ms=3)
        ax[2].axvline(0, color="grey", lw=0.5)
        ax[2].set_title(f"colour vs depth arc: rms px by lag (best {c.get('best_lag_ms', float('nan')):.0f} ms)")
        ax[2].set_xlabel("colour clock shift, ms")
    for a_ in ax[:2]:
        a_.set_xlabel("s")
    if pred:
        for name, p in pred.items():
            ttg = [-s["ttg"] for s in p["series"]]
            ax[3].plot(ttg, [100 * s["radial_m"] for s in p["series"]], "-o", ms=3, color=colours[name], label=name)
            ax[3].axhline(100 * p["reference"]["radial_m"], color=colours[name], lw=0.6, ls=":")
        rim = res["_bucket_radius"]
        ax[3].axhline(100 * rim, color="k", lw=1.0)
        ax[3].set_title("predicted miss from the axis, cm (rim = black)", fontsize=9)
        ax[3].set_xlabel("s before the reference crossing")
    if not c.get("lag_curve_ms"):
        ax[2].axis("off")
    if pred:
        ax[3].legend(fontsize=7)
    fig.suptitle(Path(res["path"]).name)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def add_predictions(res: dict, bucket, cfg: D.DetectCfg, up: np.ndarray | None = None):
    """The bucket verdict and time to correct (rs_predict.py) from each source's detections that the offline arc accepted."""
    import rs_predict as P
    res["prediction"] = {}
    res["_bucket_radius"] = bucket.radius
    for name, (t_all, picks, fit) in res["_keep"].items():
        got = [(t_all[i], p[2]) for i, p in enumerate(picks) if p is not None]
        if fit is None or len(got) < 8:
            continue
        obs = (res.get("_obs") or {}).get(name)
        fitter = P.ray_fitter(obs, bucket.up if up is None else up, cfg.ball) if obs is not None else None
        res["prediction"][name] = P.predict_throw(np.array([g[0] for g in got]), np.array([g[1] for g in got]), bucket, cfg, up=up, fitter=fitter)


def pooled_up(results: list[dict]) -> np.ndarray | None:
    """World up in the camera frame, the median over the session's throws of the fitted gravity (the soft prior of arcfit.py's
    session_gravity): the mount's pitch isn't needed, and a floor-level check of the CAD."""
    ups = [r["_keep"][s][2].up() for r in results for s in r["_keep"] if r["_keep"][s][2] is not None and r["_keep"][s][2].up() is not None]
    if not ups:
        return None
    u = np.median(np.array(ups), axis=0)
    return u / np.linalg.norm(u)


def describe(res: dict) -> str:
    lines = [f"{Path(res['path']).name}: {', '.join(res['streams'])}"]
    for name, s in res["sources"].items():
        if not s.get("ok"):
            lines.append(f"  {name:10s} no arc ({s['candidates']} candidates" + (f"; {s['reason']}" if s.get("reason") else "") + ")")
            continue
        if name == "colour":
            so = s.get("size_over_depth")
            lines.append(f"  {name:10s} {s['detections']:3d}/{s['frames']} frames in flight, first seen {s['first_range_m']:.2f} m at {s['first_t']:.2f} s, "
                         f"last {s['last_range_m']:.2f} m at {s['last_t']:.2f} s, arc {s['rms_px']:.1f} px rms, speed {s['speed_mps']:.1f} m/s; "
                         f"range from depth on {s['range_from_depth']}/{s['flight_frames']}" + (f", size / depth range {so:.2f}" if so else ""))
            continue
        lines.append(f"  {name:10s} {s['detections']:3d}/{s['frames']} frames, first seen {s['first_range_m']:.2f} m at {s['first_t']:.2f} s, "
                     f"arc rms {1000 * s['rms_m']:.0f} mm, |a| {s['a_norm']:.1f} ({s['a_vs_gravity_pct']:+.0f}% vs g), speed {s['speed_mps']:.1f} m/s")
    if "depth_vs_ir_stereo" in res:
        d = res["depth_vs_ir_stereo"]
        lines.append(f"  depth vs IR stereo arcs: rms {100 * d['rms_m']:.1f} cm, max {100 * d['max_m']:.1f} cm over {d['t_span_s']:.2f} s")
    c = res.get("colour")
    if c:
        s = f"  colour: blob in {c['frames_with_a_blob']}/{c['frames']} frames"
        if c.get("rms_px_at_zero_lag") is not None:
            s += f"; vs depth arc {c['rms_px_at_zero_lag']:.1f} px rms ({c['matched_at_zero_lag']} matched)"
        if c.get("best_lag_ms") is not None:
            s += f"; best clock lag {c['best_lag_ms']:+.0f} ms ({c['rms_px_at_best_lag']:.1f} px)"
        lines.append(s)
    if res.get("prediction"):
        import rs_predict as P
        for name, p in res["prediction"].items():
            lines.append(f"{P.describe(p)}   [{name}]")
    return "\n".join(lines)


def parse_set(items: list[str]) -> D.DetectCfg:
    cfg = D.DetectCfg()
    types = {f.name: f.type for f in fields(D.DetectCfg)}
    for it in items or []:
        k, v = it.split("=")
        if k not in types:
            raise SystemExit(f"--set: no knob {k!r}; knobs: {', '.join(types)}")
        cur = getattr(cfg, k)
        setattr(cfg, k, type(cur)(v))
    return cfg


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", help="data/<session> (a RealSense session from record_rs.py)")
    ap.add_argument("--throw", type=int, action="append", help="only these throw numbers")
    ap.add_argument("--set", nargs="*", help="detector knobs, e.g. depth_gap=0.3 ir_thr=25 (rs_detect.DetectCfg)")
    ap.add_argument("--bg", choices=["throw", "setup"], default="throw", help="the empty scene: the throw's first frames, or the session's still")
    ap.add_argument("--no-colour", action="store_true")
    ap.add_argument("--bucket", help="bucket.json (rs_predict.Bucket.load): the verdict and the time to correct for each throw")
    ap.add_argument("--gravity", choices=["bucket", "pooled"], default="bucket",
                    help="the up direction of the causal fits: the mount's (bucket.json) or the median of the session's fitted gravity")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--out", default=str(RESULTS))
    a = ap.parse_args(argv)

    d = Path(a.session)
    sess = S.load_session(d)
    if "rs" not in sess:
        raise SystemExit("not a RealSense session (no 'rs' block): record it with record_rs.py")
    cfg = parse_set(a.set)
    if not any(s.split("=")[0] in ("ball_radius", "ball") for s in (a.set or [])):          # an explicit --set ball_radius=... wins
        cfg = with_session_ball(cfg, sess)                                                    # the session's (test) ball, not the robot's
    print(f"ball: radius {1000 * cfg.ball.radius:.1f} mm (diameter {2000 * cfg.ball.radius:.1f}), {1000 * cfg.ball.mass:.0f} g, "
          f"drag k = {cfg.ball.k_quad():.4f} /m, Cd {cfg.ball.drag_cd}" + ("" if sess.get("ball") else "  [the robot config's: this session recorded none]"))
    dirs = S.throw_dirs(d)
    if a.throw:
        dirs = [p for p in dirs if int(p.name[1:]) in a.throw]
    Path(a.out).mkdir(parents=True, exist_ok=True)
    results = []
    t0 = time.time()
    for p in dirs:
        th = S.load_throw(p, sess)
        res = analyze_throw(th, sess, cfg, a.bg, colour=not a.no_colour, session_dir=d, session_ball=False,       # cfg already carries the ball
                            up=__import__("rs_predict").Bucket.load(a.bucket).up if a.bucket else None)
        res["name"] = p.name
        results.append(res)
        if not a.bucket:
            print(describe(res))
    if a.bucket:
        import rs_predict as P
        bucket = P.Bucket.load(a.bucket)
        bucket.ball_radius = cfg.ball_radius                    # one ball for the whole analysis (bucket.json's is only a default)
        up =pooled_up(results) if a.gravity == "pooled" else None
        if up is not None:
            print(f"pooled gravity (world up in the camera frame): {np.round(up, 4).tolist()}  vs the mount's {np.round(bucket.up, 4).tolist()} "
                  f"({math.degrees(math.acos(np.clip(up @ bucket.up, -1, 1))):.2f} deg apart)")
        for res in results:
            add_predictions(res, bucket, cfg, up)
            print(describe(res))
    for res in results:
        if a.plot:
            png = Path(a.out) / f"rs_{d.name}_{res['name']}.png"
            plot_throw(res, png)
            res["plot"] = png.name
        res.pop("_keep", None)
        res.pop("_obs", None)
        res.pop("_cands", None)
        res.pop("_bucket_radius", None)
    f = Path(a.out) / f"rs_analysis_{d.name}.json"
    f.write_text(json.dumps(dict(session=str(d), cfg=asdict(cfg), bg=a.bg, bucket=a.bucket, throws=results), indent=1, default=float))
    print(f"{len(results)} throws in {time.time() - t0:.0f} s -> {f}")


if __name__ == "__main__":
    main()
