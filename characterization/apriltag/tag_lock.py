"""How well the robot's cameras lock on to an AprilTag: a still tag, each camera mode and detector in turn (design.md
4.11 / 10: what the tag-augmented SLAM can expect). Runs on the computer the cameras are on (the Pi), no ROS; the
cameras through omni_console's camera_snap, detection and pose through apriltag_view (the C apriltag library, or
OpenCV's aruco: what today's tag_detector node uses) and omni_slam's own tag_pose and gates.

Per configuration, for --seconds: every frame the detector can keep up with, then

  rate        the share of frames processed with the tag found; the longest run without it
  corners     each corner's scatter about its mean (px RMS over x and y), against the sim's noise model
              (tag_sim.TagCamera: sigma = 0.25 + 4 / side px)
  pose        range mean and scatter, position scatter (mm), the normal's scatter (deg), fit RMS, ambiguous share,
              the SLAM gates' verdict
  covariance  what the SLAM's noise models need (omni_slam.graph takes each corner as independent, sigma_px 0.7, every
              frame independent): the corners' covariance split into a shift they share and noise of their own, the
              frame-to-frame correlation, the 6-DoF pose covariance (camera frame), and the pose covariance the
              corner noise predicts through the projection (at the measured sigma and at the graph's 0.7 px)
  truth       the D435's depth over the tag (projector on for a moment if it was off) and --tape, a rough measured
              range (a sanity check: the scatter is the point, not the bias)
  reach       the frames shrunk to emulate the tag farther away (fewer pixels, the same optics): detection rate and
              corner scatter against side length, and the range that side means for a 0.14 m and a 0.20 m tag.
              Shrinking averages the sensor noise away, so this reach is optimistic

  ~/.venvs/cam/bin/python tag_lock.py [--family tag25h9] [--size 0.14] [--tape 1.346] [--seconds 6] [--light WORDS]
                                       [--only NAME..]
  (characterization/apriltag/tl.sh deploys and runs it; results/<time>.json, and a table on the terminal)
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parents[1] / "ros" / "src" / "omni_console" / "omni_console"):
    sys.path.insert(0, str(p))

import apriltag_view as av                                   # noqa: E402
import camera_snap as cs                                     # noqa: E402

T = sys.modules.get("omni_slam.tags") or sys.modules["tags"]   # the SLAM's tag geometry, as apriltag_view loaded it
GRAPH_SIGMA_PX = 0.7                                          # omni_slam.graph.TagGraph's corner noise

# name: (camera, profile, stream, settings, decimate, backend)
CONFIGS = {
    "arducam_2304_d2": ("arducam", "wide_full", "color", {}, 2.0, "apriltag"),
    "arducam_1152_d2": ("arducam", "wide30", "color", {}, 2.0, "apriltag"),
    "arducam_1152_d1": ("arducam", "wide30", "color", {}, 1.0, "apriltag"),
    "arducam_1152_cv": ("arducam", "wide30", "color", {}, 2.0, "opencv"),
    "arducam_crop768_d1": ("arducam", "crop100", "color", {}, 1.0, "apriltag"),
    "d435_ir1280_d2": ("realsense", "hd30", "ir", {"emitter": 0}, 2.0, "apriltag"),
    "d435_ir1280_d1": ("realsense", "hd30", "ir", {"emitter": 0}, 1.0, "apriltag"),
    "d435_ir1280_cv": ("realsense", "hd30", "ir", {"emitter": 0}, 2.0, "opencv"),
    "d435_ir848_d1": ("realsense", "wvga60", "ir", {"emitter": 0}, 1.0, "apriltag"),
    "d435_ir640_d1": ("realsense", "vga30", "ir", {"emitter": 0}, 1.0, "apriltag"),
    "d435_ir1280_emitter_on": ("realsense", "hd30", "ir", {"emitter": 1}, 2.0, "apriltag"),
    "d435_color1280_d2": ("realsense", "hd30", "color", {"emitter": 1}, 2.0, "apriltag"),
}
SHRINK = (1.0, 0.7, 0.5, 0.35, 0.25, 0.18, 0.13, 0.1, 0.08, 0.065)


def sim_sigma(side: float) -> float:
    """The sim's corner noise (simulation/v11 sil/tag_sim.py TagCamera)."""
    return 0.25 + 4.0 / side


def open_source(cam: str, profile: str, settings: dict):
    return (cs.PiCam if cam == "arducam" else cs.RealSense)(profile, settings)


def collect(src, stream: str, finder: av.TagFinder, size: float, tag_id: int, seconds: float):
    """Detect on the newest frame, as fast as the detector goes. Returns (records, frames kept for the reach test,
    the last frame, frames the camera made)."""
    recs, keep, seq, f = [], [], 0, None
    seq0 = None
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        f = src.wait(seq, 3.0)
        seq0 = seq0 or f.seq
        seq = f.seq
        img = f.image(stream)
        gray = av.gray_of(img)
        t0 = time.perf_counter()
        dets = finder.detect(gray)
        ms = (time.perf_counter() - t0) * 1e3
        d = next((x for x in dets if x.tag_id == tag_id), None)
        # anything else found is a misread (one tag in view): what it looked like, and whether the SLAM's gates take it
        others = []
        for x in dets:
            if x.tag_id != tag_id:
                o = av.describe(x, f.cams.get(stream), size)
                others.append({k: o.get(k) for k in ("id", "margin", "hamming", "side_px", "rms_px", "ok", "ambiguous", "reasons")})
        r = dict(seq=f.seq, t=f.t, ms=ms, n=len(dets), others=others)
        if d is not None:
            info = av.describe(d, f.cams.get(stream), size)
            r.update(found=True, corners=d.corners, margin=d.margin, info=info, R=np.array(info["R"]),
                     xyz=np.array(info["xyz"]))
            if f.depth is not None:
                r["depth"] = f.depth.check(stream, r["R"], r["xyz"], size)
        else:
            r["found"] = False
        recs.append(r)
        if len(keep) < 24 and len(recs) % 3 == 0:
            keep.append(gray.copy())
    return recs, keep, f, (f.seq - seq0 + 1) if f else 0


def stats(recs: list, size: float, tape: float | None, cam=None) -> dict:
    found = [r for r in recs if r["found"]]
    n = len(recs)
    gap = run = 0
    for r in recs:
        run = 0 if r["found"] else run + 1
        gap = max(gap, run)
    out = dict(frames=n, found=len(found), rate=round(len(found) / max(n, 1), 3), longest_gap_frames=gap,
               detect_ms_median=round(float(np.median([r["ms"] for r in recs])), 1) if recs else None,
               detect_ms_p95=round(float(np.percentile([r["ms"] for r in recs], 95)), 1) if recs else None,
               misreads=sum(len(r["others"]) for r in recs),
               misread_list=[o for r in recs for o in r["others"]][:20],
               misreads_passing_gates=sum(1 for r in recs for o in r["others"] if o.get("ok") and not o.get("ambiguous")))
    if len(found) < 3:
        return out
    C = np.array([r["corners"] for r in found])                       # (k, 4, 2)
    corner_rms = float(np.sqrt(np.mean(np.var(C, axis=0))))             # per coordinate
    side = float(np.mean([r["info"]["side_px"] for r in found]))
    X = np.array([r["xyz"] for r in found])
    rng = np.linalg.norm(X, axis=1)
    N = np.array([r["R"][:, 2] for r in found])
    n_mean = N.mean(0) / np.linalg.norm(N.mean(0))
    ang = np.degrees(np.arccos(np.clip(N @ n_mean, -1, 1)))
    margins = [r["margin"] for r in found if r["margin"] is not None]
    reasons: dict = {}
    for r in found:
        for k in r["info"]["reasons"]:
            reasons[k] = reasons.get(k, 0) + 1
    out.update(side_px=round(side, 1), corner_rms_px=round(corner_rms, 3), sim_corner_sigma_px=round(sim_sigma(side), 3),
               margin_median=round(float(np.median(margins)), 1) if margins else None,
               range_mean_m=round(float(rng.mean()), 4), range_std_mm=round(float(rng.std() * 1e3), 2),
               xyz_std_mm=[round(float(v) * 1e3, 2) for v in X.std(0)],
               normal_std_deg=round(float(np.sqrt(np.mean(ang ** 2))), 3),
               angle_deg=round(float(np.mean([r["info"]["angle_deg"] for r in found])), 2),
               fit_rms_px=round(float(np.median([r["info"]["rms_px"] for r in found])), 3),
               ambiguous=round(float(np.mean([r["info"]["ambiguous"] for r in found])), 3),
               gate_ok=round(float(np.mean([r["info"]["ok"] for r in found])), 3), gate_reasons=reasons,
               xyz_mean_m=[round(float(v), 4) for v in X.mean(0)])
    if tape:
        out["range_vs_tape"] = round(float(rng.mean()) / tape, 4)
    dep = [r["depth"] for r in found if r.get("depth") and r["depth"].get("depth_z")]
    if dep:
        out["depth_z_m"] = round(float(np.median([d["depth_z"] for d in dep])), 4)
        out["depth_valid"] = round(float(np.median([d["valid"] for d in dep])), 3)
        out["pose_vs_depth"] = round(float(np.median([d["pose_z"] / d["depth_z"] for d in dep])), 4)
    if cam is not None and len(found) >= 10:
        out["cov"] = covariance(found, cam, size)
    return out


def _g(x, n: int = 3):
    """Rounded to n significant figures, nested lists too."""
    if isinstance(x, (list, tuple, np.ndarray)):
        return [_g(v, n) for v in x]
    return float(f"{float(x):.{n}g}")


def lag1(a: np.ndarray) -> float:
    """Lag-1 autocorrelation of each column of a (k, m) series, averaged over the columns."""
    a = a - a.mean(0)
    num, den = (a[1:] * a[:-1]).sum(0), (a * a).sum(0)
    ok = den > 0
    return float(np.mean(num[ok] / den[ok])) if ok.any() else math.nan


def corners_of(cam, R, t, size) -> np.ndarray:
    return cam.project(T.tag_corners(size) @ R.T + t).ravel()


def corner_jacobian(cam, R, t, size, eps: float = 1e-6) -> np.ndarray:
    """d(the 8 corner pixel coordinates) / d(translation m, rotation about the camera axes rad): (8, 6), the order of
    the pose vector covariance() uses."""
    c0, J = corners_of(cam, R, t, size), np.zeros((8, 6))
    for k in range(3):
        d = np.zeros(3)
        d[k] = eps
        J[:, k] = (corners_of(cam, R, t + d, size) - c0) / eps
        J[:, 3 + k] = (corners_of(cam, T.rodrigues(d) @ R, t, size) - c0) / eps
    return J


def covariance(found: list, cam, size: float) -> dict:
    """The noise of a still tag's detections, in the forms the SLAM's models take (see the docstring)."""
    C = np.array([r["corners"] for r in found])                       # (k, 4, 2) px
    D = C - C.mean(0)
    D8 = D.reshape(len(C), 8)
    common = D.mean(1)                                                # (k, 2): the shift all four corners share
    total = float(np.sum(D8.var(0)))
    own = D - common[:, None, :]
    corr = np.corrcoef(D8.T)
    same_axis = [corr[i, j] for ax in (0, 1) for i in range(ax, 8, 2) for j in range(ax, 8, 2) if i < j]
    # the pose: position (m) and rotation about the camera's axes (rad), camera frame (x right, y down, z ahead)
    X = np.array([r["xyz"] for r in found])
    R0 = found[0]["R"]
    W = np.array([T.log_rot(r["R"] @ R0.T) for r in found])
    P = np.column_stack([X, W])
    cov6 = np.cov(P.T)
    sd6 = np.sqrt(np.diag(cov6))
    rng = np.linalg.norm(X, axis=1)
    v = X.mean(0) / np.linalg.norm(X.mean(0))
    cx = cov6[:3, :3]
    along = float(np.sqrt(v @ cx @ v))
    across = float(np.sqrt(max(np.trace(cx) - along ** 2, 0) / 2))
    # what the corner noise predicts for the pose, at the mean pose: sigma^2 (J^T J)^-1
    Rm = T.rodrigues(W.mean(0)) @ R0
    J = corner_jacobian(cam, Rm, X.mean(0), size)
    JtJi = np.linalg.inv(J.T @ J)
    sig = float(np.sqrt(np.mean(D8.var(0))))
    pred = np.sqrt(np.diag(JtJi)) * sig
    pred_graph = np.sqrt(np.diag(JtJi)) * GRAPH_SIGMA_PX

    def six(a):
        return dict(x_mm=_g(a[0] * 1e3), y_mm=_g(a[1] * 1e3), z_mm=_g(a[2] * 1e3),
                    rx_deg=_g(math.degrees(a[3])), ry_deg=_g(math.degrees(a[4])), rz_deg=_g(math.degrees(a[5])))

    rho = lag1(D8)                                                    # the corners' frame-to-frame correlation
    dt = np.diff([r["t"] for r in found])
    return dict(
        samples=len(found), dt_ms=_g(float(np.median(dt)) * 1e3) if dt.size else None,
        corner_sigma_px=_g(sig), corner_sigma_u_px=_g(float(np.sqrt(D8[:, 0::2].var(0).mean()))),
        corner_sigma_v_px=_g(float(np.sqrt(D8[:, 1::2].var(0).mean()))),
        corner_common_share=_g(4 * float(common.var(0).sum()) / total if total else math.nan, 2),
        corner_own_sigma_px=_g(float(np.sqrt(np.mean(own.reshape(len(C), 8).var(0))))),
        corner_corr_same_axis=_g(float(np.mean(same_axis)), 2),
        corner_lag1=_g(rho, 2), range_lag1=_g(lag1(rng[:, None]), 2),
        independent_share=_g(min(1.0, max(0.0, (1 - rho) / (1 + rho))), 2),   # n_eff / n, a first-order correlated series
        pose_sigma=six(sd6), along_ray_mm=_g(along * 1e3), across_ray_mm=_g(across * 1e3),
        pose_cov_m_rad=_g(cov6.tolist()), pose_corr=_g(np.corrcoef(P.T).tolist(), 2),
        predicted_sigma=six(pred), measured_over_predicted=_g((sd6 / pred).tolist(), 2),
        predicted_sigma_graph=six(pred_graph))


def reach(frames: list, finder_for, tag_id: int, f_px: float) -> list:
    """Detection on the kept frames shrunk by each factor: emulates the tag farther away with the same optics."""
    import cv2
    out = []
    fin = finder_for(1.0)
    for k in SHRINK:
        C, n = [], 0
        for g in frames:
            gg = g if k == 1.0 else cv2.resize(g, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
            d = next((x for x in fin.detect(gg) if x.tag_id == tag_id), None)
            if d is not None:
                n += 1
                C.append(d.corners / k)                                 # back in full-size pixels
        row = dict(shrink=k, rate=round(n / max(len(frames), 1), 3))
        if len(C) >= 3:
            C = np.array(C)
            side_full = float(np.mean(np.linalg.norm(C - np.roll(C, -1, axis=1), axis=2)))
            side = side_full * k
            row.update(side_px=round(side, 1), corner_rms_px=round(float(np.sqrt(np.mean(np.var(C * k, axis=0)))), 3),
                       sim_sigma_px=round(sim_sigma(side), 3),
                       range_014_m=round(f_px * 0.14 / side, 2), range_020_m=round(f_px * 0.20 / side, 2))
        out.append(row)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--family", default="tag25h9")
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--size", type=float, default=0.14, help="the tag's black square, m")
    ap.add_argument("--tape", type=float, default=None, help="measured range to the tag from the D435, m")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--only", nargs="*", help="configuration names (default: all)")
    ap.add_argument("--light", help="the scene's light, in words (kept in the results: it changes exposure, margin, noise)")
    ap.add_argument("--out", type=Path, default=HERE / "results")
    ap.add_argument("--when", default=time.strftime("%Y%m%d_%H%M%S"),
                    help="the run's time stamp, YYYYmmdd_HHMMSS (tl.sh passes the PC's: the Pi's clock has no NTP)")
    a = ap.parse_args()
    names = a.only or list(CONFIGS)
    res = dict(when=a.when, light=a.light, family=a.family, tag_id=a.id, size_m=a.size, tape_m=a.tape,
               seconds=a.seconds, detector_default=av.backend(), configs={})
    src = None
    key = None
    try:
        for name in names:
            cam, profile, stream, settings, dec, use = CONFIGS[name]
            if (cam, profile) != key:                               # reopen only when the camera or mode changes
                if src is not None:
                    src.close()
                src, key = None, None
                src = open_source(cam, profile, settings)
                key = (cam, profile)
                time.sleep(1.5)                                     # auto exposure
            src.apply(settings)
            if settings:
                time.sleep(0.5)
            finder = av.TagFinder(a.family, dec, use=use)
            print(f"== {name}: {cam} {profile} {stream}, decimate {dec}, {use}", flush=True)
            recs, keep, last, made = collect(src, stream, finder, a.size, a.id, a.seconds)
            st = stats(recs, a.size, a.tape if cam == "realsense" else None, last.cams.get(stream) if last else None)
            st.update(camera=cam, profile=profile, stream=stream, settings=settings, decimate=dec, backend=use,
                      width=int(keep[0].shape[1]) if keep else None, height=int(keep[0].shape[0]) if keep else None,
                      camera_frames=made, processed_fps=round(len(recs) / a.seconds, 1),
                      focal_px=round(last.cams[stream].fx, 1) if last and stream in last.cams else None,
                      exposure=dict((k, v) for k, v in last.info.items() if "exposure" in k or k in ("gain", "lens_dioptres")) if last else {})
            if cam == "realsense" and settings.get("emitter") == 0 and st.get("found"):
                # depth with the projector on, at the pose found with it off (the tag is still): the truth for range
                src.apply({"emitter": 1})
                time.sleep(1.0)
                f = src.wait(0, 3.0)
                found = [r for r in recs if r["found"]]
                R, t = found[-1]["R"], np.mean([r["xyz"] for r in found], axis=0)
                chk = f.depth.check(stream, R, t, a.size)
                st["depth_emitter_on"] = chk
                src.apply({"emitter": 0})
                time.sleep(0.5)
            if keep and use == "apriltag" and stream != "depth" and st.get("found", 0) >= 3 and dec == 1.0:
                st["reach"] = reach(keep, lambda d: av.TagFinder(a.family, d), a.id, st["focal_px"])
            res["configs"][name] = st
            show(name, st)
    finally:
        if src is not None:
            src.close()
    a.out.mkdir(parents=True, exist_ok=True)
    p = a.out / f"tag_lock_{a.when}.json"
    p.write_text(json.dumps(res, indent=1))
    print(f"-> {p}")


def show(name: str, s: dict):
    if not s.get("found"):
        print(f"   {s['frames']} frames, tag never found (detect {s['detect_ms_median']} ms)", flush=True)
        return
    print(f"   {s['width']}x{s['height']} f {s['focal_px']} px: found {s['found']}/{s['frames']} ({100 * s['rate']:.0f}%), "
          f"longest gap {s['longest_gap_frames']}, detect {s['detect_ms_median']} ms (p95 {s['detect_ms_p95']}), "
          f"{s['processed_fps']} of {s['camera_frames'] / max(s['frames'], 1) * s['processed_fps']:.0f} fps", flush=True)
    print(f"   side {s['side_px']} px, margin {s['margin_median']}, corners {s['corner_rms_px']} px rms "
          f"(sim {s['sim_corner_sigma_px']}), fit {s['fit_rms_px']} px; range {s['range_mean_m']} m "
          f"± {s['range_std_mm']} mm, xyz ± {s['xyz_std_mm']} mm, normal ± {s['normal_std_deg']}°, "
          f"ambiguous {100 * s['ambiguous']:.0f}%, gates ok {100 * s['gate_ok']:.0f}%", flush=True)
    extra = [f"{k} {s[k]}" for k in ("range_vs_tape", "depth_z_m", "pose_vs_depth") if k in s]
    if s.get("misreads"):
        extra.append(f"misreads {s['misreads']} ({s['misreads_passing_gates']} pass the SLAM's gates): " + "; ".join(
            f"id {o['id']} margin {o['margin']} hamming {o['hamming']} side {o['side_px']} px {'ok' if o['ok'] else o['reasons']}"
            for o in s["misread_list"][:4]))
    if s.get("depth_emitter_on"):
        extra.append(f"depth (projector on) {s['depth_emitter_on']}")
    if extra:
        print("   " + ", ".join(extra), flush=True)
    c = s.get("cov")
    if c:
        ps, pp, pg = c["pose_sigma"], c["predicted_sigma"], c["predicted_sigma_graph"]
        print(f"   covariance ({c['samples']} frames, {c['dt_ms']} ms apart): corners {c['corner_sigma_px']} px "
              f"(u {c['corner_sigma_u_px']}, v {c['corner_sigma_v_px']}), {100 * c['corner_common_share']:.0f}% shared by all four "
              f"(own {c['corner_own_sigma_px']} px), frame-to-frame r {c['corner_lag1']} (range {c['range_lag1']}: "
              f"{100 * c['independent_share']:.0f}% independent)", flush=True)
        print(f"   pose sigma x y z {ps['x_mm']} {ps['y_mm']} {ps['z_mm']} mm (along the ray {c['along_ray_mm']}, across "
              f"{c['across_ray_mm']}), rot {ps['rx_deg']} {ps['ry_deg']} {ps['rz_deg']} deg; predicted from the corners "
              f"x y z {pp['x_mm']} {pp['y_mm']} {pp['z_mm']} mm, rot {pp['rx_deg']} {pp['ry_deg']} {pp['rz_deg']} deg "
              f"(measured/predicted {c['measured_over_predicted']}); the graph's 0.7 px assumes z {pg['z_mm']} mm, "
              f"rot {pg['rx_deg']} {pg['ry_deg']} deg", flush=True)
    for r in s.get("reach", []):
        print(f"     shrink {r['shrink']}: {100 * r['rate']:.0f}%" + (
            f", side {r['side_px']} px, corners {r['corner_rms_px']} px (sim {r['sim_sigma_px']}), "
            f"= {r['range_014_m']} m for 0.14, {r['range_020_m']} m for 0.20 m" if "side_px" in r else ""), flush=True)


if __name__ == "__main__":
    main()
