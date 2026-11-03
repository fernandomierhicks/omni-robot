"""Step 4: depth noise and accuracy on a flat target, and the camera's pose from the floor (README.md).

Wall mode (default): aim square at a flat, matte wall; capture at several distances (0.5, 1, 2, 3, 4, 6 m).
Per capture, over the central ROI: fill rate, spatial noise (RMS off a fitted plane: Intel's metric),
temporal noise (per-pixel std over the frames), the plane's distance against your tape measure (from the
front glass; the depth origin sits ~3.2 mm behind it), and the disparity noise that implies, which
carries over to other baselines and lenses: from it, the dense-depth noise at the 6-7 m catch range
and what the design's 190 mm bar would see.

Floor mode (--floor): the camera looking down at the floor as the camera bar does at -40 deg for the arm
(design.md 2, 4.4). The camera's height, pitch and roll from the floor plane and their frame-to-frame
noise (the design recovers the bar's pose from the floor plane, design.md 4.1), and the things standing
on the floor taller than --min-height-mm (the tidy's objects, low obstacles under the lidar's 150 mm
plane).

    python rs_depth_plane.py                         # preview; SPACE captures, then type the tape distance
    python rs_depth_plane.py --floor --min-height-mm 15
    python rs_depth_plane.py --no-preview --truth-m 2.0 --frames 90
"""
from __future__ import annotations

import argparse
import datetime

import cv2
import numpy as np

import rs_common as C
from rs_common import rs


def capture(pipe, n: int, scale: float):
    frames = []
    for _ in range(n):
        fs = pipe.wait_for_frames(3000)
        d = fs.get_depth_frame()
        frames.append(np.asanyarray(d.get_data()).astype(np.float32) * scale)
    return np.stack(frames)


def roi_mask(h, w, frac):
    m = np.zeros((h, w), bool)
    y0, y1 = int(h * (1 - frac) / 2), int(h * (1 + frac) / 2)
    x0, x1 = int(w * (1 - frac) / 2), int(w * (1 + frac) / 2)
    m[y0:y1, x0:x1] = True
    return m


def robust_plane(pts):
    n, d, r = C.fit_plane(pts)
    keep = np.abs(r) < max(3 * r.std(), 1e-4)
    if keep.sum() > 100 and keep.sum() < len(pts):
        n, d, r = C.fit_plane(pts[keep])
    return n, d, r


def analyze(stack: np.ndarray, intr: dict, roi_frac: float, baseline_m: float, floor: bool,
            min_height_mm: float, truth_m: float | None) -> dict:
    N, h, w = stack.shape
    fx, fy, ppx, ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]
    mask = np.ones((h, w), bool) if floor else roi_mask(h, w, roi_frac)
    fills, rms, normals, ds, dist_axis, poses = [], [], [], [], [], []
    for z in stack:
        valid = (z > 0) & mask
        fills.append(valid.sum() / mask.sum())
        pts = C.deproject(z, fx, fy, ppx, ppy, mask=mask, step=2)
        if len(pts) < 500:
            continue
        n, d, r = robust_plane(pts)
        rms.append(float(np.sqrt(np.mean(r ** 2))))
        normals.append(n)
        ds.append(d)
        dist_axis.append(C.plane_distance_on_axis(n, d) if abs(n[2]) > 0.2 else np.nan)
        if floor:
            poses.append(C.plane_pose(n, d))
    if not normals:
        return dict(error="too few valid depth pixels: is the target in range and lit?")
    normals = np.array(normals)
    n_mean = normals.mean(axis=0)
    n_mean /= np.linalg.norm(n_mean)
    ang = [C.angle_between_deg(n, n_mean) for n in normals]
    perp = float(np.median(ds))                     # perpendicular camera-plane distance
    z_axis = float(np.nanmedian(dist_axis))
    # temporal noise: pixels valid in >= 90% of the frames
    valid = stack > 0
    good = (valid.mean(axis=0) >= 0.9) & mask
    zt = np.where(valid, stack, np.nan)[:, good]
    tstd = np.nanstd(zt, axis=0) if zt.size else np.array([])
    z_ref = z_axis if np.isfinite(z_axis) else perp
    out = dict(frames=N, roi_frac=None if floor else roi_frac, width=w, height=h,
               fill_rate=C.stats(fills), spatial_rms_mm=C.stats(np.array(rms) * 1000),
               temporal_std_mm=dict(median=float(np.median(tstd) * 1000) if tstd.size else None,
                                    p95=float(np.percentile(tstd, 95) * 1000) if tstd.size else None),
               perpendicular_m=perp, axis_m=z_axis,
               tilt_to_axis_deg=C.angle_between_deg(n_mean, [0, 0, 1]),
               normal_noise_deg=C.stats(ang))
    sp = out["spatial_rms_mm"]["median"] / 1000
    out["spatial_rms_pct"] = 100 * sp / z_ref
    out["disparity_sigma_px"] = float(C.disparity_sigma(z_ref, sp, fx, baseline_m))
    sd = out["disparity_sigma_px"]
    zc = np.mean(C.THROW_RANGE_M)
    out["extrapolated"] = dict(
        z_m=zc, dense_depth_sigma_m=float(C.stereo_sigma_z(zc, fx, baseline_m, sd)),
        ball_radius_px=float(C.ball_radius_px(zc, fx)),
        design_bar_sigma_m=float(C.stereo_sigma_z(zc, C.SIM_F_PX, C.SIM_BASELINE_M, sd)),
        note="same disparity noise on the design's 190 mm bar and 70 deg vertical lens; a ball is a few pixels "
             "of disparity, not a plane, so take these as a floor (rs_ball.py measures the ball)")
    if truth_m is not None:
        t = truth_m + C.D435_DATASHEET["ground_zero_mm"] / 1000
        out["truth_m"] = truth_m
        out["bias_mm"] = (perp - t) * 1000
        out["bias_pct"] = 100 * (perp - t) / t
    if floor:
        P = {k: C.stats([p[k] for p in poses]) for k in ("height_m", "pitch_deg", "roll_deg")}
        out["floor_pose"] = P
        out["objects"] = floor_objects(stack, intr, n_mean, float(np.median(ds)), min_height_mm)
    return out


def floor_objects(stack, intr, n, d, min_height_mm):
    """Blobs standing above the floor plane in the median depth image."""
    z = np.median(np.where(stack > 0, stack, np.nan), axis=0)
    h, w = z.shape
    v, u = np.mgrid[0:h, 0:w]
    X = (u - intr["ppx"]) / intr["fx"] * z
    Y = (v - intr["ppy"]) / intr["fy"] * z
    height = n[0] * X + n[1] * Y + n[2] * z + d
    m = np.nan_to_num(height, nan=0) > min_height_mm / 1000
    m &= np.nan_to_num(height, nan=0) < 0.6
    k, lab, st, cen = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
    objs = []
    for i in range(1, k):
        if st[i, cv2.CC_STAT_AREA] < 40:
            continue
        sel = lab == i
        zi = float(np.nanmedian(z[sel]))
        objs.append(dict(pixels=int(st[i, cv2.CC_STAT_AREA]), range_m=zi,
                         max_height_mm=float(np.nanmax(height[sel]) * 1000),
                         size_m=float(np.sqrt(st[i, cv2.CC_STAT_AREA]) * zi / intr["fx"]),
                         centroid_px=[float(cen[i][0]), float(cen[i][1])]))
    return sorted(objs, key=lambda o: -o["pixels"])[:20]


def print_capture(r: dict):
    if "error" in r:
        print("  " + r["error"])
        return
    print(f"  plane {r['perpendicular_m']:.3f} m (tilt {r['tilt_to_axis_deg']:.1f} deg)  fill "
          f"{100 * r['fill_rate']['median']:.1f}%  spatial RMS {r['spatial_rms_mm']['median']:.2f} mm "
          f"({r['spatial_rms_pct']:.2f}%)  temporal {r['temporal_std_mm']['median'] or 0:.2f} mm  "
          f"subpixel {r['disparity_sigma_px']:.3f} px  normal noise {r['normal_noise_deg']['median']:.3f} deg")
    if "bias_mm" in r:
        print(f"  against the tape: {r['bias_mm']:+.1f} mm ({r['bias_pct']:+.2f}%)")
    e = r["extrapolated"]
    print(f"  at {e['z_m']:.1f} m: dense depth sigma {100 * e['dense_depth_sigma_m']:.1f} cm, ball radius "
          f"{e['ball_radius_px']:.1f} px; the design's bar {100 * e['design_bar_sigma_m']:.1f} cm")
    if "floor_pose" in r:
        P = r["floor_pose"]
        print(f"  floor: height {1000 * P['height_m']['median']:.1f} +- {1000 * P['height_m']['std']:.2f} mm, "
              f"pitch {P['pitch_deg']['median']:.2f} +- {P['pitch_deg']['std']:.3f} deg, "
              f"roll {P['roll_deg']['median']:.2f} +- {P['roll_deg']['std']:.3f} deg")
        for o in r["objects"][:8]:
            print(f"    object at {o['range_m']:.2f} m: {o['max_height_mm']:.0f} mm tall, ~{100 * o['size_m']:.0f} cm, "
                  f"{o['pixels']} px")


def save_png(stack, r, path, floor):
    z = np.median(np.where(stack > 0, stack, np.nan), axis=0)
    zc = np.nan_to_num(z, nan=0)
    vis = cv2.applyColorMap(cv2.convertScaleAbs(zc, alpha=255 / max(np.nanmax(zc), 0.1)), cv2.COLORMAP_TURBO)
    vis[zc == 0] = 0
    cv2.putText(vis, f"{r.get('perpendicular_m', 0):.3f} m  RMS {r.get('spatial_rms_mm', {}).get('median', 0):.2f} mm",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.imwrite(str(path), vis)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial")
    ap.add_argument("--mode", default="depth:848x480@30", help="depth mode (Intel's specs are at 848x480)")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--roi", type=float, default=0.4, help="central fraction of the image (Intel uses 0.8)")
    ap.add_argument("--preset", default="high_accuracy",
                    choices=["default", "high_accuracy", "high_density", "medium_density", "hand"])
    ap.add_argument("--emitter", choices=["on", "off"], default="on")
    ap.add_argument("--laser-power", type=float, default=None, help="mW (D435: 0..360)")
    ap.add_argument("--floor", action="store_true")
    ap.add_argument("--min-height-mm", type=float, default=15.0)
    ap.add_argument("--truth-m", type=float, default=None, help="tape distance, front glass to target")
    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--label", default="", help="free text kept with the result (target, lighting)")
    a = ap.parse_args()

    dev = C.find_device(a.serial)
    mode = C.parse_mode(a.mode)
    if not C.mode_supported(dev, mode):
        raise SystemExit(f"{a.mode} not offered on this link ({C.info(dev, 'usb_type_descriptor')})")
    cfg = rs.config()
    cfg.enable_device(C.info(dev, "serial_number"))
    C.enable_streams(cfg, mode)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    C.configure_stereo(dev, emitter=a.emitter == "on", laser_power=a.laser_power, preset=a.preset)
    scale = C.depth_sensor(dev).get_depth_scale()
    intr = C.intrinsics_dict(prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics())
    baseline = C.ir_baseline_m(dev)
    for _ in range(30):
        pipe.wait_for_frames(3000)                  # let auto-exposure settle

    captures = []
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    C.RESULTS.mkdir(exist_ok=True)

    def do_capture(truth):
        stack = capture(pipe, a.frames, scale)
        r = analyze(stack, intr, a.roi, baseline, a.floor, a.min_height_mm, truth)
        r["temperatures"] = C.temperatures(dev)
        r["label"] = a.label
        print_capture(r)
        png = C.RESULTS / f"plane_{stamp}_{len(captures)}.png"
        save_png(stack, r, png, a.floor)
        r["image"] = png.name
        captures.append(r)

    try:
        if a.no_preview:
            do_capture(a.truth_m)
        else:
            win = "depth (SPACE capture, q quit)"
            m = roi_mask(intr["height"], intr["width"], 1.0 if a.floor else a.roi)
            ys, xs = np.where(m)
            while True:
                fs = pipe.wait_for_frames(3000)
                z = np.asanyarray(fs.get_depth_frame().get_data()).astype(np.float32) * scale
                vis = cv2.applyColorMap(cv2.convertScaleAbs(z, alpha=255 / 6.0), cv2.COLORMAP_TURBO)
                vis[z == 0] = 0
                cv2.rectangle(vis, (int(xs.min()), int(ys.min())), (int(xs.max()), int(ys.max())), (255, 255, 255), 1)
                zr = z[m]
                zr = zr[zr > 0]
                txt = (f"median {np.median(zr):.3f} m  fill {100 * zr.size / m.sum():.0f}%" if zr.size else "no depth")
                cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.imshow(win, vis)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q") or k == 27:
                    break
                if k == ord(" "):
                    t = a.truth_m
                    if t is None:
                        s = input("tape distance, front glass to target (m; Enter to skip): ").strip()
                        t = float(s) if s else None
                    print(f"capturing {a.frames} frames ...")
                    do_capture(t)
            cv2.destroyAllWindows()
    finally:
        pipe.stop()
    if captures:
        C.save_result("floor" if a.floor else "plane",
                      dict(mode=a.mode, preset=a.preset, emitter=a.emitter, laser_power=a.laser_power,
                           intrinsics=intr, baseline_m=baseline, captures=captures), dev)


if __name__ == "__main__":
    main()
