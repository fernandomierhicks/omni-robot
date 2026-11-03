"""Step 6: a ball at known ranges, two ways (README.md).

The catch tracks a 74 mm ball thrown from 6-7 m (design.md 1). Put a ball (the design's foam ball when
it arrives; give --ball-mm for any other) on a stand at measured ranges (1, 2, 3, 4, 5, 6, 7 m) and click
it in the preview. Each capture measures:

  dense depth (emitter on): the ball's pixel radius against f R / z, how much of its disc has depth (fill),
    its centre's depth (each pixel corrected by the sphere's bulge), noise over the frames and the bias
    against your tape;
  blob stereo (emitter off): the robot's own method, the ball's patch in the left IR image matched along
    the same row in the right one (normalized cross-correlation, subpixel). Depth from the disparity, its
    noise, the disparity noise in px (it carries over to other cameras) and what the design's 190 mm bar
    with its 70 deg lens would get from the same pixel noise.

    python rs_ball.py                         # 848x480; click the ball, SPACE, type the tape distance
    python rs_ball.py --mode 1280x720 --ball-mm 74
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

import rs_common as C
from rs_common import rs


def segment_ball(z: np.ndarray, click: tuple[int, int], R: float, f: float):
    """The ball's pixels: the connected region around the click whose depth lies within the sphere's depth
    span of the nearest surface there. Returns (mask, centroid (x, y), front depth) or None."""
    h, w = z.shape
    x, y = click
    win = z[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
    win = win[win > 0]
    if win.size < 3:
        return None
    z_front = float(np.percentile(win, 20))
    r_px = f * R / (z_front + R)
    box = int(max(6, 2.5 * r_px))
    y0, y1, x0, x1 = max(0, y - box), min(h, y + box + 1), max(0, x - box), min(w, x + box + 1)
    sub = z[y0:y1, x0:x1]
    cand = ((sub > z_front - 0.03) & (sub < z_front + R + 0.03)).astype(np.uint8)
    k, lab = cv2.connectedComponents(cand, connectivity=8)
    lx, ly = x - x0, y - y0
    lbl = lab[ly, lx]
    if lbl == 0:                                   # the click hit a hole: take the biggest blob nearby
        if k <= 1:
            return None
        lbl = 1 + int(np.argmax([np.sum(lab == i) for i in range(1, k)]))
    m = np.zeros_like(z, bool)
    m[y0:y1, x0:x1] = lab == lbl
    ys, xs = np.nonzero(m)
    return m, (float(xs.mean()), float(ys.mean())), z_front


def ball_centre_depth(z: np.ndarray, centroid, r_model_px: float, R: float, disc: float = 0.8):
    """Fill of the ball's disc (radius disc * r_model) and its centre depth: each valid pixel at normalized
    radius rho is the sphere's surface, z_c - R sqrt(1 - rho^2)."""
    h, w = z.shape
    cx, cy = centroid
    r = max(1.0, disc * r_model_px)
    v, u = np.mgrid[0:h, 0:w]
    rho2 = ((u - cx) ** 2 + (v - cy) ** 2) / r_model_px ** 2
    inside = ((u - cx) ** 2 + (v - cy) ** 2) <= r ** 2
    vals = z[inside]
    ok = vals > 0
    fill = float(ok.mean()) if vals.size else 0.0
    if not ok.any():
        return fill, None
    zc = vals[ok] + R * np.sqrt(np.clip(1 - rho2[inside][ok], 0, 1))
    return fill, float(np.median(zc))


def blob_disparity(left: np.ndarray, right: np.ndarray, centroid, r_px: float, d_guess: float,
                   rows: int = 2, disc: float = 0.9):
    """The ball's disc in the left IR image matched in the right one along the same rows (+-rows), by
    normalized cross-correlation over the disc's pixels only: the background around the ball sits at
    another disparity and would pull a square template off it. Returns (disparity px, vertical offset px,
    NCC peak, disc contrast) or None."""
    h, w = left.shape
    cx, cy = centroid
    half = int(np.ceil(max(3.0, 1.1 * r_px)))
    xi, yi = int(round(cx)), int(round(cy))
    if xi - half < 0 or xi + half >= w or yi - half - rows < 0 or yi + half + rows >= h:
        return None
    yy, xx = np.mgrid[-half:half + 1, -half:half + 1]
    m = (xx + xi - cx) ** 2 + (yy + yi - cy) ** 2 <= (disc * r_px) ** 2
    if m.sum() < 9:
        return None
    t = left[yi - half:yi + half + 1, xi - half:xi + half + 1].astype(np.float32)[m]
    contrast = float(t.std() / max(t.mean(), 1e-3))
    t = t - t.mean()
    tn = float(np.linalg.norm(t))
    if tn < 1e-3:
        return None
    d_lo, d_hi = max(1, int(np.floor(0.6 * d_guess - 3))), int(np.ceil(1.5 * d_guess + 3))
    scores = np.full((2 * rows + 1, d_hi - d_lo + 1), -np.inf)
    for iy, dy in enumerate(range(-rows, rows + 1)):
        y0 = yi - half + dy
        band = right[y0:y0 + 2 * half + 1].astype(np.float32)
        for k, d in enumerate(range(d_lo, d_hi + 1)):
            x0 = xi - half - d                       # right(u) = left(u + d)
            if x0 < 0:
                break
            p = band[:, x0:x0 + 2 * half + 1][m]
            p = p - p.mean()
            pn = float(np.linalg.norm(p))
            if pn > 0:
                scores[iy, k] = float(t @ p) / (tn * pn)
    iy, k = np.unravel_index(int(np.argmax(scores)), scores.shape)
    peak = float(scores[iy, k])
    if not np.isfinite(peak):
        return None

    def sub(a, b, c):
        return C.subpixel_peak(a, b, c) if np.isfinite(a) and np.isfinite(c) else 0.0

    sx = sub(scores[iy, k - 1], peak, scores[iy, k + 1]) if 0 < k < scores.shape[1] - 1 else 0.0
    sy = sub(scores[iy - 1, k], peak, scores[iy + 1, k]) if 0 < iy < scores.shape[0] - 1 else 0.0
    return float(d_lo + k + sx), float(iy - rows + sy), peak, contrast


def capture_frames(pipe, n, scale):
    out = []
    for _ in range(n):
        fs = pipe.wait_for_frames(3000)
        out.append((np.asanyarray(fs.get_depth_frame().get_data()).astype(np.float32) * scale,
                    np.asanyarray(fs.get_infrared_frame(1).get_data()).copy(),
                    np.asanyarray(fs.get_infrared_frame(2).get_data()).copy()))
    return out


def settle(pipe, n=12):
    for _ in range(n):
        pipe.wait_for_frames(3000)


def measure(pipe, dev, click, intr, baseline, scale, R, n, truth_front_m):
    f = intr["fx"]
    C.configure_stereo(dev, emitter=True)
    settle(pipe)
    on = capture_frames(pipe, n, scale)
    zmed = np.median(np.stack([d for d, _, _ in on]), axis=0)
    seg = segment_ball(zmed, click, R, f)
    if seg is None:
        return dict(error="no depth at the click (out of range, or not on the ball)")
    mask, cen, z_front = seg
    z_c0 = z_front + R
    r_model = float(C.ball_radius_px(z_c0, f, 2 * R))
    dense = [ball_centre_depth(d, cen, r_model, R) for d, _, _ in on]
    fills = [x[0] for x in dense]
    zcs = [x[1] for x in dense if x[1] is not None]
    out = dict(centroid_px=list(cen), r_model_px=r_model, r_seen_px=float(np.sqrt(mask.sum() / np.pi)),
               dense=dict(fill=C.stats(fills), centre_m=C.stats(zcs)))

    C.configure_stereo(dev, emitter=False)
    settle(pipe)
    off = capture_frames(pipe, n, scale)
    d_guess = f * baseline / z_c0
    bl = [blob_disparity(l, r, cen, r_model, d_guess) for _, l, r in off]
    bl = [b for b in bl if b is not None and b[0] > 0]
    if bl:
        disp = np.array([b[0] for b in bl])
        zst = f * baseline / disp
        out["blob"] = dict(frames=len(bl), disparity_px=C.stats(disp), vertical_px=C.stats([b[1] for b in bl]),
                           ncc=C.stats([b[2] for b in bl]), contrast=C.stats([b[3] for b in bl]),
                           centre_m=C.stats(zst))
        sd = float(np.std(disp))
        zc = float(np.median(zst))
        out["blob"]["disparity_sigma_px"] = sd
        out["blob"]["design_bar_sigma_m"] = float(C.stereo_sigma_z(zc, C.SIM_F_PX, C.SIM_BASELINE_M, sd))
    # dense depth with the emitter off as well (ambient texture only)
    zs_off = [ball_centre_depth(d, cen, r_model, R) for d, _, _ in off]
    out["dense_emitter_off"] = dict(fill=C.stats([x[0] for x in zs_off]),
                                    centre_m=C.stats([x[1] for x in zs_off if x[1] is not None]))
    C.configure_stereo(dev, emitter=True)

    if truth_front_m is not None:
        t = truth_front_m + R + C.D435_DATASHEET["ground_zero_mm"] / 1000
        out["truth_centre_m"] = t
        out["r_truth_px"] = float(C.ball_radius_px(t, f, 2 * R))
        if zcs:
            out["dense"]["bias_mm"] = (float(np.median(zcs)) - t) * 1000
        if "blob" in out:
            out["blob"]["bias_mm"] = (out["blob"]["centre_m"]["median"] - t) * 1000
    return out


def print_result(r):
    if "error" in r:
        print("  " + r["error"])
        return
    d = r["dense"]
    zc = d["centre_m"]
    print(f"  ball radius {r['r_seen_px']:.1f} px seen, {r['r_model_px']:.1f} px model")
    print(f"  dense depth: fill {100 * d['fill']['median']:.0f}%"
          + (f", centre {zc['median']:.3f} m +- {1000 * zc['std']:.1f} mm" if zc.get("n") else ", no depth")
          + (f", bias {d['bias_mm']:+.0f} mm" if "bias_mm" in d else ""))
    if "blob" in r:
        b = r["blob"]
        print(f"  blob stereo: disparity {b['disparity_px']['median']:.2f} +- {b['disparity_sigma_px']:.3f} px, "
              f"centre {b['centre_m']['median']:.3f} m +- {1000 * b['centre_m']['std']:.1f} mm"
              + (f", bias {b['bias_mm']:+.0f} mm" if "bias_mm" in b else "")
              + f", NCC {b['ncc']['median']:.2f}, vertical {b['vertical_px']['median']:+.2f} px; on the design's bar "
              f"+- {1000 * b['design_bar_sigma_m']:.1f} mm")
    else:
        print("  blob stereo: no match (too little texture with the emitter off? more light)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial")
    ap.add_argument("--mode", default="848x480", help="WxH for depth and both IR streams (848x480, 1280x720)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--ball-mm", type=float, default=1000 * C.BALL_D_M)
    ap.add_argument("--preset", default="high_accuracy")
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    dev = C.find_device(a.serial)
    w, h = map(int, a.mode.split("x"))
    mode = [("depth", w, h, a.fps), ("stereo", w, h, a.fps)]
    if not C.mode_supported(dev, mode):
        raise SystemExit(f"depth + both IR at {a.mode}@{a.fps} not offered on this link")
    cfg = rs.config()
    cfg.enable_device(C.info(dev, "serial_number"))
    C.enable_streams(cfg, mode)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    C.configure_stereo(dev, emitter=True, preset=a.preset)
    scale = C.depth_sensor(dev).get_depth_scale()
    intr = C.intrinsics_dict(prof.get_stream(rs.stream.infrared, 1).as_video_stream_profile().get_intrinsics())
    baseline = C.ir_baseline_m(dev)
    R = a.ball_mm / 2000

    click = [None]
    win = "ball: click it, SPACE capture, q quit"

    def on_mouse(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN:
            click[0] = (x % w, y)

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    caps = []
    try:
        while True:
            fs = pipe.wait_for_frames(3000)
            z = np.asanyarray(fs.get_depth_frame().get_data()).astype(np.float32) * scale
            ir = cv2.cvtColor(np.asanyarray(fs.get_infrared_frame(1).get_data()), cv2.COLOR_GRAY2BGR)
            dv = cv2.applyColorMap(cv2.convertScaleAbs(z, alpha=255 / 8.0), cv2.COLORMAP_TURBO)
            dv[z == 0] = 0
            if click[0]:
                x, y = click[0]
                zz = z[y, x]
                for im in (ir, dv):
                    cv2.drawMarker(im, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 16, 1)
                cv2.putText(ir, f"{zz:.3f} m" if zz > 0 else "no depth", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 1)
            cv2.imshow(win, np.hstack((ir, dv)))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" ") and click[0]:
                s = input("tape distance, front glass to the ball's front surface (m; Enter to skip): ").strip()
                truth = float(s) if s else None
                print(f"capturing {a.frames} frames with the emitter on, then {a.frames} off ...")
                r = measure(pipe, dev, click[0], intr, baseline, scale, R, a.frames, truth)
                r.update(truth_front_m=truth, click=list(click[0]))
                print_result(r)
                caps.append(r)
            elif k == ord(" "):
                print("click the ball first")
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
    if caps:
        C.save_result("ball", dict(mode=f"{a.mode}@{a.fps}", ball_mm=a.ball_mm, preset=a.preset, label=a.label,
                                   intrinsics=intr, baseline_m=baseline, captures=caps), dev)


if __name__ == "__main__":
    main()
