"""Step 7: real throws, recorded and tracked (README.md).

record: the camera stands still at robot height (~0.3-0.5 m, tilted up ~30 deg like the camera bar in
catch, design.md 2) and someone throws the ball toward it from 2-7 m, a few throws per recording. Depth +
left IR at 90 fps go to a .bag in ~/realsense_recordings (~110 MB/s, kept out of Dropbox). Keep the
first second empty of people moving: it's the background.

analyze: background subtraction on depth, blobs the ball's size at their range (f R / z), one per frame,
joined into tracks; each track that moves faster than 1.5 m/s (the design's honest track start, design.md
4.1) gets a constant-acceleration fit: how far out the ball is first seen, how many of its frames have
it, the fit's residual (depth noise on a moving ball), the motion blur (blob elongation) and the
acceleration (|a| near g, a bit off with drag, and pointing down if the timestamps and depth scale are
right). A plot per track goes into results/.

    python rs_throw.py record --seconds 20 --exposure-us 1500
    python rs_throw.py analyze ~/realsense_recordings/throw_<time>.bag
    python rs_throw.py analyze --latest
"""
from __future__ import annotations

import argparse
import datetime
import pathlib
import time

import cv2
import numpy as np

import rs_common as C
from rs_common import rs


def record(a):
    dev = C.find_device(a.serial)
    w, h = map(int, a.size.split("x"))
    mode = [("depth", w, h, a.fps)] + ([] if a.no_ir else [("ir", w, h, a.fps)])
    if not C.mode_supported(dev, mode):
        raise SystemExit(f"{a.size}@{a.fps} not offered on this link ({C.info(dev, 'usb_type_descriptor')})")
    C.RECORDINGS.mkdir(parents=True, exist_ok=True)
    path = pathlib.Path(a.out) if a.out else C.RECORDINGS / f"throw_{datetime.datetime.now():%Y%m%d_%H%M%S}.bag"
    cfg = rs.config()
    cfg.enable_device(C.info(dev, "serial_number"))
    C.enable_streams(cfg, mode)
    pipe = rs.pipeline()
    # start once without recording to set the sensor, so the bag starts with the final settings
    pipe.start(cfg)
    C.configure_stereo(dev, emitter=True, exposure_us=a.exposure_us, preset=a.preset)
    if a.gain is not None:
        C.set_opt(C.depth_sensor(dev), "gain", a.gain)
    for _ in range(30):
        pipe.wait_for_frames(3000)
    pipe.stop()
    cfg.enable_record_to_file(str(path))
    pipe.start(cfg)
    print(f"recording {a.seconds:.0f} s to {path} - throw after the first second")
    t0 = time.time()
    n = 0
    try:
        while time.time() - t0 < a.seconds:
            fs = pipe.wait_for_frames(3000)
            n += 1
            if a.preview and n % 3 == 0:
                z = np.asanyarray(fs.get_depth_frame().get_data())
                cv2.imshow("recording (q stops)", cv2.applyColorMap(cv2.convertScaleAbs(z, alpha=255 / 8000),
                                                                   cv2.COLORMAP_TURBO))
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
    print(f"{n} framesets, {path.stat().st_size / 1e6:.0f} MB")
    return path


def detect(z: np.ndarray, bg: np.ndarray, f: float, R: float, min_gap_m: float = 0.25, zmax: float = 9.0):
    """The best ball-sized foreground blob in a depth frame (m), or None:
    (u, v, z_centre, area, model_area, elongation)."""
    fg = (z > 0.3) & (z < zmax) & ((bg == 0) | (z < bg - min_gap_m))
    fg = cv2.morphologyEx(fg.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    k, lab, st, cen = cv2.connectedComponentsWithStats(fg, connectivity=8)
    best, best_score = None, np.inf
    for i in range(1, k):
        area = st[i, cv2.CC_STAT_AREA]
        if area < 3:
            continue
        bw, bh = st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT]
        x0, y0 = st[i, cv2.CC_STAT_LEFT], st[i, cv2.CC_STAT_TOP]
        sub = z[y0:y0 + bh, x0:x0 + bw][lab[y0:y0 + bh, x0:x0 + bw] == i]
        zf = float(np.median(sub))
        r = f * R / (zf + R)
        model = np.pi * r * r
        ratio = area / max(model, 1.0)
        elong = max(bw, bh) / max(1, min(bw, bh))
        if not (0.25 <= ratio <= 4.0) or elong > 4:
            continue
        score = abs(np.log(ratio)) + 0.2 * (elong - 1)
        if score < best_score:
            best_score = score
            best = (float(cen[i][0]), float(cen[i][1]), zf + 2 * R / 3, int(area), float(model), float(elong))
    return best


def tracks_from(dets, max_gap_ms=60.0, max_speed=20.0):
    """Split time-ordered detections [(t_ms, xyz, ...)] into tracks by time gaps and jumps."""
    tracks, cur = [], []
    for d in dets:
        if cur:
            dt = (d[0] - cur[-1][0]) / 1000
            jump = np.linalg.norm(np.subtract(d[1], cur[-1][1]))
            if dt * 1000 > max_gap_ms or jump > max_speed * dt + 0.3:
                tracks.append(cur)
                cur = []
        cur.append(d)
    if cur:
        tracks.append(cur)
    return tracks


def analyze(a):
    path = pathlib.Path(a.bag) if a.bag else None
    if a.latest or path is None:
        bags = sorted(C.RECORDINGS.glob("*.bag"), key=lambda p: p.stat().st_mtime)
        if not bags:
            raise SystemExit(f"no .bag in {C.RECORDINGS}")
        path = bags[-1]
    C.need_rs()
    cfg = rs.config()
    cfg.enable_device_from_file(str(path), repeat_playback=False)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    prof.get_device().as_playback().set_real_time(False)
    dprof = prof.get_stream(rs.stream.depth).as_video_stream_profile()
    intr = C.intrinsics_dict(dprof.get_intrinsics())
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    f, R = intr["fx"], a.ball_mm / 2000
    frames, bg_stack, dets = 0, [], []
    t_first = None
    domain = None
    print(f"analyzing {path.name} ...")
    while True:
        ok, fs = pipe.try_wait_for_frames(2000)
        if not ok:
            break
        d = fs.get_depth_frame()
        if not d:
            continue
        z = np.asanyarray(d.get_data()).astype(np.float32) * scale
        t = d.get_timestamp()
        domain = domain or str(d.get_frame_timestamp_domain()).split(".")[-1]
        t_first = t if t_first is None else t_first
        frames += 1
        if len(bg_stack) < a.bg_frames:
            bg_stack.append(z)
            if len(bg_stack) == a.bg_frames:
                s = np.stack(bg_stack)
                bg = np.where((s > 0).sum(axis=0) > a.bg_frames // 2,
                              np.median(np.where(s > 0, s, np.nan), axis=0), 0).astype(np.float32)
                bg = np.nan_to_num(bg)
            continue
        hit = detect(z, bg, f, R)
        if hit is not None:
            u, v, zc, area, model, elong = hit
            xyz = ((u - intr["ppx"]) / f * zc, (v - intr["ppy"]) / intr["fy"] * zc, zc)
            dets.append((t, xyz, area, model, elong, d.get_frame_number()))
    pipe.stop()
    fps = dprof.fps()
    out_tracks = []
    for tr in tracks_from(dets):
        if len(tr) < a.min_points:
            continue
        t = np.array([x[0] for x in tr]) / 1000
        p = np.array([x[1] for x in tr])
        fit = C.fit_ballistic(t, p)
        if fit["speed0"] < 1.5:
            continue
        span_frames = max(1, int(round((t[-1] - t[0]) * fps)) + 1)
        a_vec = np.array(fit["a"])
        fit.update(t_start_s=float((tr[0][0] - t_first) / 1000), first_range_m=float(p[0, 2]),
                   last_range_m=float(p[-1, 2]), max_range_m=float(p[:, 2].max()), detections=len(tr),
                   detection_rate=len(tr) / span_frames, elongation=C.stats([x[4] for x in tr]),
                   area_ratio=C.stats([x[2] / x[3] for x in tr]),
                   a_angle_to_image_down_deg=C.angle_between_deg(a_vec, [0, 1, 0]))
        out_tracks.append(fit)
        if a.plot:
            png = C.RESULTS / f"throw_{path.stem}_{len(out_tracks)}.png"
            plot_track(t, p, fit, png)
            fit["plot"] = png.name
    print(f"{frames} frames ({domain}), {len(dets)} detections, {len(out_tracks)} throw track(s)")
    for i, tr in enumerate(out_tracks, 1):
        print(f"  {i}: t {tr['t_start_s']:.2f} s, {tr['detections']} pts over {tr['duration_s']:.2f} s "
              f"({100 * tr['detection_rate']:.0f}% of frames), range {tr['max_range_m']:.2f} -> {tr['last_range_m']:.2f} m, "
              f"speed {tr['speed0']:.1f} m/s, |a| {tr['a_norm']:.1f} m/s^2 ({tr['a_angle_to_image_down_deg']:.0f} deg off "
              f"image-down), residual {1000 * tr['rms_m']:.0f} mm, blur {tr['elongation']['median']:.1f}")
    if domain == "system_time":
        print("NOTE: host-arrival timestamps (metadata off): the fit's time base carries USB jitter")
    C.save_result("throw", dict(bag=str(path), frames=frames, domain=domain, intrinsics=intr, fps=fps,
                                ball_mm=a.ball_mm, detections=len(dets), tracks=out_tracks), None)


def plot_track(t, p, fit, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tt = t - t[0]
    A = np.column_stack((np.ones_like(tt), tt, 0.5 * tt ** 2))
    coef = np.array([fit["p0"], fit["v0"], fit["a"]])
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.2))
    for i, lab in enumerate(("x right", "y down", "z forward")):
        ax[i].plot(tt, p[:, i], ".", ms=4)
        ax[i].plot(tt, A @ coef[:, i], "-", lw=1)
        ax[i].set_title(f"{lab} (m)")
        ax[i].set_xlabel("s")
    fig.suptitle(f"|a| {fit['a_norm']:.1f} m/s^2, residual {1000 * fit['rms_m']:.0f} mm, {fit['n']} pts")
    fig.tight_layout()
    C.RESULTS.mkdir(exist_ok=True)
    fig.savefig(path, dpi=100)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--serial")
    r.add_argument("--size", default="848x480")
    r.add_argument("--fps", type=int, default=90)
    r.add_argument("--seconds", type=float, default=20)
    r.add_argument("--exposure-us", type=float, default=None, help="fixed exposure; ~1000-2000 freezes the ball")
    r.add_argument("--gain", type=float, default=None)
    r.add_argument("--preset", default="default")
    r.add_argument("--no-ir", action="store_true")
    r.add_argument("--preview", action="store_true")
    r.add_argument("--out")
    r.add_argument("--analyze", action="store_true", help="analyze right after recording")
    r.add_argument("--ball-mm", type=float, default=1000 * C.BALL_D_M)
    z = sub.add_parser("analyze")
    z.add_argument("bag", nargs="?")
    z.add_argument("--latest", action="store_true")
    z.add_argument("--ball-mm", type=float, default=1000 * C.BALL_D_M)
    z.add_argument("--bg-frames", type=int, default=30)
    z.add_argument("--min-points", type=int, default=6)
    z.add_argument("--no-plot", dest="plot", action="store_false")
    a = ap.parse_args()
    if a.cmd == "record":
        path = record(a)
        if a.analyze:
            analyze(argparse.Namespace(bag=str(path), latest=False, ball_mm=a.ball_mm, bg_frames=30, min_points=6,
                                       plot=True))
    else:
        analyze(a)


if __name__ == "__main__":
    main()
