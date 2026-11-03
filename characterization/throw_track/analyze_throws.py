"""Analyse a recorded throw session (design.md 8, phase 1C): the robot's own perception over real frames, scored against
the throw's fitted arc and the basket.

For every throw:
  A  the robot's Perception (omni_catch: detector + image-space EKF, one camera) runs over the frames in order, as on
     the robot, and we keep what it does: when the track starts, its crossing of the basket's rim plane at each frame;
  B  an arc with drag is fitted to the detections (arcfit.py), then the detector is re-run with the arc's prediction
     as its search window, which finds the ball in frames A missed (far away, small): the detector's own rate;
  C  metrics: detection rates vs range, centroid / size noise (residuals to the arc, and the arc-free second
     differences), blob elongation (-> exposure), the live track's crossing error vs time to go, and the score
     prediction against what you typed.
Run on the PC (sim venv: numpy, scipy). Results go to results/analysis_<session>.json; fit_vision.py turns them into
the sim's VisionConfig.

  python analyze_throws.py data/<session> [--no-live] [--plot]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
for p in (HERE.parents[1] / "ros" / "src" / "omni_catch",):
    if p.is_dir():
        sys.path.insert(0, str(p))

from omni_catch.detector import DetectorConfig, ball_mask, label      # noqa: E402
from omni_catch.frames import ArrayFrame                              # noqa: E402
from omni_catch.perception import Perception                          # noqa: E402
from omni_catch.robot_config import RobotConfig                       # noqa: E402
from omni_catch.tracker import TrackerConfig                          # noqa: E402

import arcfit                                                          # noqa: E402
import session as S                                                    # noqa: E402

ROBOT_V11 = HERE.parents[1] / "ros" / "src" / "omni_catch" / "omni_catch" / "config" / "robot_v11.json"
TTG_BINS = (1.0, 0.8, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1)         # s before the ball crosses the rim plane


# -- geometry ---------------------------------------------------------------------------------------------------
def world_from_camera(up_c) -> np.ndarray:
    """Rows: the world axes (x right, y forward along the camera's heading, z up) in camera coordinates, so
    world = M @ cam. up_c: world up in the camera frame."""
    up = np.asarray(up_c, float) / np.linalg.norm(up_c)
    xc = np.array([1.0, 0.0, 0.0])
    xw = xc - (xc @ up) * up
    xw /= np.linalg.norm(xw)
    return np.vstack([xw, np.cross(up, xw), up])


def camera_of(sess: dict) -> arcfit.Camera:
    c = sess["camera"]
    f = c.get("f_px") or (c["height"] / 2) / math.tan(math.radians(c["fovy_deg"]) / 2)
    return arcfit.Camera(float(f), c["width"] / 2, c["height"] / 2, c["width"], c["height"])


def ball_of(sess: dict) -> arcfit.Ball:
    b = json.loads(ROBOT_V11.read_text())["ball"]
    b = {**b, **sess.get("ball", {})}
    return arcfit.Ball(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])


def robot_config_file(sess: dict, M: np.ndarray, ball: arcfit.Ball, tmp: Path) -> Path:
    """robot_v11.json with this session's one camera at the origin, looking where it looks, world z up."""
    cfg = json.loads(ROBOT_V11.read_text())
    c = sess["camera"]
    fovy = c["fovy_deg"] if not c.get("f_px") else math.degrees(2 * math.atan(c["height"] / 2 / c["f_px"]))
    cfg["cameras"] = [dict(name="left", pos=[0.0, 0.0, 0.0], R=M.tolist(), fovy_deg=fovy, width=c["width"], height=c["height"])]
    cfg["base"]["base_link_height"] = 0.0
    cfg["ball"].update(mass=ball.mass, radius=ball.radius, drag_cd=ball.drag_cd)
    f = tmp / "robot_session.json"
    f.write_text(json.dumps(cfg))
    return f


def rim_of(sess: dict, M: np.ndarray):
    """(centre in world coordinates, radius) of the basket's rim, or (None, r)."""
    b = sess.get("basket") or {}
    r = b.get("rim_radius", 0.165)
    if b.get("rim_center_world") is not None:
        return np.array(b["rim_center_world"], float), r
    if b.get("tag_cam") is not None:
        off = np.array(b.get("rim_center_from_tag", [0.0, r, 0.0]), float)
        return M @ np.array(b["tag_cam"], float) + off, r
    return None, r


# -- blob shape (elongation) ------------------------------------------------------------------------------------
def blob_axes(img, u, v, b, dcfg: DetectorConfig):
    """(major, minor) semi-axes of the ball-coloured blob near (u, v), same estimator as the detector's size."""
    h = int(3 * b + 12)
    x0, y0 = max(0, int(u) - h), max(0, int(v) - h)
    x1, y1 = min(img.shape[1], int(u) + h), min(img.shape[0], int(v) + h)
    m = ball_mask(np.asarray(img[y0:y1, x0:x1]), dcfg)
    if not m.any():
        return None
    lab, n = label(m)
    k = lab[min(max(int(v) - y0, 0), m.shape[0] - 1), min(max(int(u) - x0, 0), m.shape[1] - 1)]
    if not k:
        sizes = np.bincount(lab.ravel())[1:]
        k = 1 + int(np.argmax(sizes))
    ys, xs = np.nonzero(lab == k)
    if len(xs) < 8:
        return None
    cov = np.cov(np.vstack([xs, ys]), bias=True)
    lam = np.linalg.eigvalsh(cov)
    return 2 * math.sqrt(max(lam[1] - 1 / 12, 1e-6)), 2 * math.sqrt(max(lam[0] - 1 / 12, 1e-6))


def as_frame(x):
    """A recorded image (array) or a ready frame object (the sim's SparseFrame, in gate_sim_vs_real.py)."""
    return x if hasattr(x, "roi") else ArrayFrame(np.asarray(x))


# -- one throw --------------------------------------------------------------------------------------------------
def live_pass(throw: S.Throw, rc: RobotConfig, dcfg: DetectorConfig, tcfg: TrackerConfig, z_plane: float | None):
    """The robot's Perception over the frames. Per frame: the blob it used (or None) and, once the track is ready,
    its predicted crossing of z_plane."""
    perc = Perception(rc, tcfg, dcfg)
    t = throw.t
    rows = []
    for i in range(len(t)):
        blobs = perc.process(float(t[i]), [as_frame(throw.frames[i])])
        bl = blobs[0] if blobs else None
        row = dict(i=i, t=float(t[i]), blob=None if bl is None else (bl.u, bl.v, bl.b, bl.n_px),
                   tracking=perc.tracker.x is not None, ready=perc.ready())
        if z_plane is not None and perc.ready():
            v = perc.view()
            hit = v.crossing(float(t[i]), z_plane) if v is not None else None
            if hit is not None:
                row["cross"] = dict(xyz=hit[0].tolist(), ttg=float(hit[2]), cov=hit[3].tolist())
        rows.append(row)
    return rows, perc


def oracle_pass(throw: S.Throw, arc: arcfit.Arc, cam: arcfit.Camera, dcfg: DetectorConfig, ball: arcfit.Ball):
    """The detector with the fitted arc's prediction as its window, on every frame the arc puts the ball in view."""
    from omni_catch.fast_detector import FastDetector, available
    det = FastDetector(dcfg) if dcfg.backend == "native" and available() else None
    if det is None:
        from omni_catch.detector import Detector
        det = Detector(dcfg)
    t = throw.t
    sel = np.nonzero(t >= arc.t0 - 1e-9)[0]
    pos, _ = arc.states(t[sel])
    pu, pv, pb = arcfit.sphere_px(pos, cam, ball.radius)
    out = []
    for k, i in enumerate(sel):
        m = (pb[k] if np.isfinite(pb[k]) else 0.0) + 2.0         # a blob touching the border is dropped by the detector
        in_view = bool(np.isfinite(pu[k]) and m < pu[k] < cam.width - m and m < pv[k] < cam.height - m and pb[k] >= 1.5)
        rec = dict(i=int(i), t=float(t[i]), in_view=in_view, range=float(np.linalg.norm(pos[k])),
                   pred=None if not in_view else (float(pu[k]), float(pv[k]), float(pb[k])), blob=None, wrong=False)
        if in_view:
            half = dcfg.window_scale * max(pb[k], 1.0) + dcfg.window_px
            comps = det._components(as_frame(throw.frames[i]), pu[k] - half, pv[k] - half, pu[k] + half, pv[k] + half)
            if comps:
                c = min(comps, key=lambda c: (c[0] - pu[k]) ** 2 + (c[1] - pv[k]) ** 2)
                rec["blob"] = tuple(float(x) for x in c)
                rec["wrong"] = math.hypot(c[0] - pu[k], c[1] - pv[k]) > 2.5 * pb[k] + 4
        out.append(rec)
    return out


def detections_from(rows, key="blob", skip_wrong=True) -> arcfit.Detections | None:
    ts, us, vs, bs = [], [], [], []
    for r in rows:
        if r.get(key) is not None and not (skip_wrong and r.get("wrong")):
            u, v, b, _ = r[key]
            ts.append(r["t"]); us.append(u); vs.append(v); bs.append(b)
    if len(ts) < 8:
        return None
    return arcfit.Detections(np.array(ts), np.array(us), np.array(vs), np.array(bs))


def second_difference_noise(det: arcfit.Detections) -> float | None:
    """Arc-free centroid noise (px): the second difference of a smooth track is sqrt(6) sigma plus a little curvature."""
    if len(det.t) < 12:
        return None
    dt = np.diff(det.t)
    ok = (dt[:-1] > 0) & (np.abs(dt[1:] - dt[:-1]) < 0.002)       # consecutive frames, no gaps
    if ok.sum() < 8:
        return None
    d2u = det.u[2:] - 2 * det.u[1:-1] + det.u[:-2]
    d2v = det.v[2:] - 2 * det.v[1:-1] + det.v[:-2]
    d2 = np.concatenate([d2u[ok], d2v[ok]])
    mad = 1.4826 * np.median(np.abs(d2 - np.median(d2)))
    return float(mad / math.sqrt(6))


def analyse_throw(throw: S.Throw, sess: dict, ups, live: bool = True, up_sigma: float | None = None) -> dict:
    cam, ball = camera_of(sess), ball_of(sess)
    dcfg = DetectorConfig(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in sess.get("detector", {}).items()
                             if k in DetectorConfig.__dataclass_fields__})
    if dcfg.backend == "python":
        dcfg.backend = "native"
    res: dict = dict(path=str(throw.path), outcome=throw.label, n_frames=int(len(throw.frames)), ok=False)
    fit_up = ups if ups is not None else (0.0, 1.0, 0.0)
    # first pass of the arc: use a coarse live pass if we want the tracker, otherwise the oracle-free acquisition
    M = world_from_camera(fit_up)
    rim, rim_r = rim_of(sess, M)
    z_plane = None if rim is None else float(rim[2])
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        rc = RobotConfig(robot_config_file(sess, M, ball, Path(tmp)))
        tcfg = TrackerConfig(**{k: v for k, v in sess.get("tracker", {}).items() if k in TrackerConfig.__dataclass_fields__})
        if live:
            rows, perc = live_pass(throw, rc, dcfg, tcfg, z_plane)
    det = detections_from(rows) if rows else None
    if det is None:                                         # no live track: seed the arc from whole-frame detections
        det = seed_detections(throw, dcfg)
    if det is None:
        res["reason"] = "too few detections to fit an arc"
        return res
    arc = arcfit.fit_arc(det, cam, ball, up=ups, fit_gravity=ups is None, up_sigma=up_sigma)
    if not arc.ok:
        res["reason"] = "arc fit failed"
        res["n_live_blobs"] = int(len(det.t))
        return res
    # B: the detector with the arc's window, then refit on all of it
    orows = oracle_pass(throw, arc, cam, dcfg, ball)
    det2 = detections_from(orows)
    if det2 is not None and len(det2.t) >= len(det.t):
        arc2 = arcfit.fit_arc(det2, cam, ball, up=ups, fit_gravity=ups is None, up_sigma=up_sigma)
        if arc2.ok:
            arc, det = arc2, det2
    res.update(ok=True, arc=dict(t0=arc.t0, p0=arc.p0.tolist(), v0=arc.v0.tolist(), phi_deg=math.degrees(arc.phi),
                                 psi_deg=math.degrees(arc.psi), speed0=float(np.linalg.norm(arc.v0))),
               rms_uv=arc.rms_uv, rms_b=arc.rms_b, rms_b_rel=arc.rms_b_rel, n_fit=arc.n_inliers, n_det=int(len(det.t)),
               second_diff_px=second_difference_noise(det))
    # detection rates vs range (the oracle pass: the detector given the arc's prediction)
    inview = [r for r in orows if r["in_view"]]
    res["range_bins"] = range_bins(inview)
    res["in_view_frames"] = len(inview)
    res["detected_given_arc"] = sum(r["blob"] is not None and not r["wrong"] for r in inview)
    res["wrong_blobs"] = sum(r["wrong"] for r in inview)
    # elongation -> effective exposure
    el = []
    for r in inview:
        if not hasattr(throw.frames[0], "shape"):                 # the sim's sparse frames have no pixels to measure
            break
        if r["blob"] is not None and not r["wrong"] and r["blob"][2] >= 6:
            ax = blob_axes(throw.frames[r["i"]], r["blob"][0], r["blob"][1], r["blob"][2], dcfg)
            if ax is not None:
                # image speed from the arc (px / s) via neighbouring frames
                el.append((ax[0], ax[1], r["i"]))
    res["blur"] = blur_metrics(el, arc, cam, throw, orows)
    if rows:
        res["live"] = live_metrics(rows, arc, cam, M, ball, rim, rim_r, z_plane, throw)
    return res


def seed_detections(throw: S.Throw, dcfg: DetectorConfig) -> arcfit.Detections | None:
    """Without a live track, the largest blob of each frame, moving ones only (a still yellow box is dropped)."""
    from omni_catch.detector import Detector
    from omni_catch.fast_detector import FastDetector, available
    det = FastDetector(dcfg) if available() else Detector(dcfg)
    rows = []
    for i in range(len(throw.frames)):
        c = det.candidates(as_frame(throw.frames[i]), 8)
        rows.append([(x[0], x[1], x[2]) for x in c])
    # keep the blobs that move: a candidate that has no neighbour within 1 px in the previous or next frame's list
    keep = []
    for i, cs in enumerate(rows):
        best = None
        for c in cs:
            near = [d for j in (i - 1, i + 1) if 0 <= j < len(rows) for d in rows[j] if math.hypot(d[0] - c[0], d[1] - c[1]) < 1.0]
            if near:                                          # sat still across frames: not the ball
                continue
            if best is None or c[2] > best[2]:
                best = c
        keep.append(best)
    ts = [throw.t[i] for i, b in enumerate(keep) if b is not None]
    if len(ts) < 8:
        return None
    b = [x for x in keep if x is not None]
    return arcfit.Detections(np.array(ts), np.array([x[0] for x in b]), np.array([x[1] for x in b]), np.array([x[2] for x in b]))


def range_bins(inview, edges=(0, 1.5, 2.5, 3.5, 4.5, 6, 8, 12)):
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        s = [r for r in inview if lo <= r["range"] < hi]
        if s:
            out.append(dict(lo=lo, hi=hi, frames=len(s), detected=sum(r["blob"] is not None and not r["wrong"] for r in s),
                            wrong=sum(r["wrong"] for r in s)))
    return out


def blur_metrics(el, arc, cam, throw, orows):
    """Effective exposure (ms) from the blob's elongation: (major - minor) ~ image speed x exposure."""
    if not el:
        return None
    t = throw.t
    byi = {r["i"]: r for r in orows if r["blob"] is not None and not r["wrong"]}
    ex = []
    for major, minor, i in el:
        a, b = byi.get(i - 1), byi.get(i + 1)
        if a and b:
            sp = math.hypot(b["blob"][0] - a["blob"][0], b["blob"][1] - a["blob"][1]) / (t[i + 1] - t[i - 1])   # px/s
            if sp > 200:
                # a disc of radius r smeared over a streak of length L: the moment semi-axes are sqrt(r^2 + L^2/3)
                # along and r across, so L = sqrt(3 (major^2 - minor^2)); the exposure is L / image speed. The
                # threshold trims the streak's faint ends, so this reads low: calibrate it on synthetic blur (README)
                ex.append(math.sqrt(3 * max(major ** 2 - minor ** 2, 0.0)) / sp * 1e3)
    if not ex:
        return None
    return dict(effective_exposure_ms_median=float(np.median(ex)), n=len(ex), p90=float(np.percentile(ex, 90)))


def live_metrics(rows, arc, cam, M, ball, rim, rim_r, z_plane, throw) -> dict:
    t = throw.t
    out: dict = {}
    blobs = [r for r in rows if r["blob"] is not None]
    started = next((r for r in rows if r["tracking"]), None)
    ready = next((r for r in rows if r["ready"]), None)
    out["frames_with_blob"] = len(blobs)
    out["track_started_t"] = None if started is None else started["t"]
    out["track_ready_t"] = None if ready is None else ready["t"]
    if started is not None:
        p, _ = arc.states([started["t"]])
        out["track_started_range"] = float(np.linalg.norm(p[0]))
    if ready is not None:
        p, _ = arc.states([ready["t"]])
        out["track_ready_range"] = float(np.linalg.norm(p[0]))
    # the arc's crossing of the rim plane (world coordinates)
    if z_plane is None:
        return out
    n_w = np.array([0.0, 0.0, 1.0])                       # world up; crossing = going down through z_plane
    pt_c = M.T @ np.array([0.0, 0.0, z_plane])            # a point on the plane, in camera coordinates (x, y of it don't matter)
    hit = arc.crossing_plane(pt_c, M.T @ n_w)
    if hit is None:
        out["arc_crossing"] = None
        return out
    pos_c, t_c = hit
    pos_w = M @ pos_c
    out["arc_crossing"] = dict(xyz=pos_w.tolist(), t=float(t_c))
    out["arc_miss_m"] = float(np.hypot(*(pos_w[:2] - rim[:2])))
    out["arc_inside_rim"] = bool(out["arc_miss_m"] <= rim_r)
    errs = []
    for r in rows:
        if "cross" not in r:
            continue
        ttg = t_c - r["t"]
        if ttg <= 0.02:
            continue
        xy = np.array(r["cross"]["xyz"][:2])
        d = xy - pos_w[:2]
        cov = np.array(r["cross"]["cov"])
        try:
            m2 = float(d @ np.linalg.solve(cov + 1e-9 * np.eye(2), d))
        except np.linalg.LinAlgError:
            m2 = float("nan")
        errs.append(dict(ttg=float(ttg), err_m=float(np.hypot(*d)), ttg_err_s=float(r["cross"]["ttg"] - ttg), mahal2=m2,
                         miss_pred=float(np.hypot(*(xy - rim[:2])))))
    out["crossing_errors"] = errs
    out["bins"] = ttg_bins(errs, rim_r, out["arc_inside_rim"], throw.label)
    return out


def ttg_bins(errs, rim_r, arc_in, label):
    out = {}
    for tb in TTG_BINS:
        best = min(errs, key=lambda e: abs(e["ttg"] - tb), default=None)
        if best is None or abs(best["ttg"] - tb) > 0.03:
            continue
        out[f"{tb:g}"] = dict(err_m=best["err_m"], ttg_err_s=best["ttg_err_s"], mahal2=best["mahal2"],
                              pred_in=bool(best["miss_pred"] <= rim_r), miss_pred=best["miss_pred"])
    return out


# -- a whole session --------------------------------------------------------------------------------------------
def pool(results: list[dict], sess: dict) -> dict:
    ok = [r for r in results if r.get("ok")]
    out: dict = dict(throws=len(results), fitted=len(ok))
    if not ok:
        return out
    out["rms_uv_px"] = dict(median=float(np.median([r["rms_uv"] for r in ok])), p90=float(np.percentile([r["rms_uv"] for r in ok], 90)))
    sd = [r["second_diff_px"] for r in ok if r.get("second_diff_px")]
    if sd:
        out["second_diff_px"] = dict(median=float(np.median(sd)))
    out["rms_b_rel"] = dict(median=float(np.median([r["rms_b_rel"] for r in ok if np.isfinite(r["rms_b_rel"])])))
    inv = sum(r["in_view_frames"] for r in ok)
    det = sum(r["detected_given_arc"] for r in ok)
    out["detect_given_arc"] = dict(frames=inv, detected=det, rate=det / max(inv, 1), wrong=sum(r["wrong_blobs"] for r in ok))
    # by range
    bins: dict = {}
    for r in ok:
        for b in r["range_bins"]:
            k = (b["lo"], b["hi"])
            d = bins.setdefault(k, [0, 0])
            d[0] += b["frames"]; d[1] += b["detected"]
    out["detect_by_range"] = [dict(lo=k[0], hi=k[1], frames=v[0], rate=v[1] / v[0]) for k, v in sorted(bins.items())]
    ex = [r["blur"]["effective_exposure_ms_median"] for r in ok if r.get("blur")]
    if ex:
        out["effective_exposure_ms"] = float(np.median(ex))
    lives = [r["live"] for r in ok if "live" in r]
    if lives:
        out["live"] = pool_live(lives, ok)
    return out


def pool_live(lives, ok):
    out: dict = {}
    out["throws_tracked"] = sum(l["track_ready_t"] is not None for l in lives)
    rr = [l["track_ready_range"] for l in lives if l.get("track_ready_range") is not None]
    if rr:
        out["track_ready_range_m"] = dict(median=float(np.median(rr)), min=float(min(rr)), max=float(max(rr)))
    bins = {}
    conf = {}
    for r in ok:
        l = r.get("live")
        if not l or "bins" not in l:
            continue
        for k, v in l["bins"].items():
            bins.setdefault(k, []).append(v)
            if r["outcome"] in ("y", "n"):
                c = conf.setdefault(k, dict(tp=0, fp=0, fn=0, tn=0))
                pred, truth = v["pred_in"], r["outcome"] == "y"
                c["tp" if pred and truth else "fp" if pred else "fn" if truth else "tn"] += 1
    out["crossing_error_by_ttg"] = {
        k: dict(n=len(v), median_m=float(np.median([x["err_m"] for x in v])), p90_m=float(np.percentile([x["err_m"] for x in v], 90)),
                ttg_err_ms_median=float(np.median([abs(x["ttg_err_s"]) for x in v]) * 1e3),
                mahal2_median=float(np.nanmedian([x["mahal2"] for x in v])))
        for k, v in sorted(bins.items(), key=lambda kv: -float(kv[0]))}
    out["score_prediction_by_ttg"] = {k: dict(c, n=sum(c.values())) for k, c in sorted(conf.items(), key=lambda kv: -float(kv[0]))}
    arc_conf = dict(tp=0, fp=0, fn=0, tn=0)
    for r in ok:
        l = r.get("live")
        if l and "arc_inside_rim" in l and r["outcome"] in ("y", "n"):
            pred, truth = l["arc_inside_rim"], r["outcome"] == "y"
            arc_conf["tp" if pred and truth else "fp" if pred else "fn" if truth else "tn"] += 1
    out["arc_vs_label"] = arc_conf
    return out


def latency_from_stamps(throws: list[S.Throw]) -> dict:
    lat = []
    for t in throws:
        lat.extend(((t.stamps[:, 1] - t.stamps[:, 0]) * 1e-6).tolist())
    if not lat:
        return {}
    return dict(sensor_to_host_ms=dict(median=float(np.median(lat)), p95=float(np.percentile(lat, 95)), max=float(max(lat))))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--no-live", action="store_true", help="skip the tracker pass (arc fit and detection only)")
    ap.add_argument("--up", type=float, nargs=3, help="world up in the camera frame, if known (else fitted per throw, then the median)")
    ap.add_argument("--only", type=int, nargs="*", help="throw numbers")
    ap.add_argument("--out", type=Path, default=HERE / "results")
    a = ap.parse_args()
    sess = S.load_session(a.session)
    dirs = S.throw_dirs(a.session)
    if a.only:
        dirs = [d for d in dirs if int(d.name[1:]) in a.only]
    throws = [S.load_throw(d, sess) for d in dirs]
    print(f"{a.session.name}: {len(throws)} throws")
    t0 = time.time()
    ups = np.array(a.up) if a.up else None
    results = []
    up_sigma = None
    if ups is None:                                          # the session's gravity direction: median of the per-throw fits
        first = []
        for th in throws:
            r = analyse_throw(th, sess, None, live=False)
            if r["ok"]:
                first.append(r)
        if first:
            ups = np.median([arcfit.up_in_camera(math.radians(r["arc"]["phi_deg"]), math.radians(r["arc"]["psi_deg"])) for r in first], axis=0)
            ups /= np.linalg.norm(ups)
            # each throw's own tilt is good to ~0.4 deg; the median of N is better, and the later fits are pulled to it
            up_sigma = max(math.radians(0.05), math.radians(0.4) / math.sqrt(len(first)))
            print(f"world up in the camera frame (median of {len(first)} fits):", np.round(ups, 4),
                  f"(prior sigma {math.degrees(up_sigma):.2f} deg)")
    for th in throws:
        r = analyse_throw(th, sess, ups, live=not a.no_live, up_sigma=up_sigma)
        results.append(r)
        s = f"  {th.path.name}: " + (f"fit rms {r['rms_uv']:.2f} px, detected {r['detected_given_arc']}/{r['in_view_frames']} in view" if r["ok"] else r.get("reason", "failed"))
        l = r.get("live") or {}
        if l.get("track_ready_t") is not None:
            s += f", track ready at {l['track_ready_range']:.1f} m"
        if "arc_miss_m" in l:
            s += f", arc misses rim centre by {100 * l['arc_miss_m']:.0f} cm"
        print(s)
    summary = pool(results, sess)
    summary.update(latency_from_stamps(throws))
    out = dict(session=a.session.name, time=time.strftime("%Y-%m-%dT%H:%M:%S"), up_in_camera=None if ups is None else np.asarray(ups).tolist(),
               session_info=sess, summary=summary, throws=results, seconds=round(time.time() - t0, 1))
    a.out.mkdir(exist_ok=True)
    f = a.out / f"analysis_{a.session.name}.json"
    f.write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(summary, indent=1, default=float))
    print(f"-> {f}")


if __name__ == "__main__":
    main()
