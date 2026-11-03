"""Auto-tune the D435's colour detector on the ball itself: put the ball still in view (on a table, near the centre), run this, walk away.

The sequence (no clicks; ~30 s on the Pi):
  1. white balance: the value (2800-6000 K) that makes the room greyest (the lowest median saturation), then locked (a fixed colour
     window needs a fixed white balance; re-running this re-adapts to new light). The camera's own auto white balance left a warm-lit
     room as orange as the ball (Oct 30).
  2. the ball, found without assuming its colour: the most saturated round blob of the right size nearest the image centre, its range
     from the aligned depth map, so the expected radius in pixels is measured, not guessed (`--distance` is only a fallback).
  3. an exposure x gain sweep: at each setting the colour window is tuned from the ball's own pixels (ball_view.tune_colour's rule: the
     dominant hue's 2-98 % spread + a margin, saturation / value floors under the ball's dimmest pixels) and scored: the ball must come
     out whole (blob radius / expected), as few other blobs as possible may pass, the exposure short (motion blur: 6 m/s for 5 ms is 3 cm).
  4. tightening: if other blobs pass, narrower margins are tried while the ball stays whole.
  5. writes ~/omni_ws/ball_view_settings/realsense_auto.json (ball_view/1 format: record_rs.py --settings reads it) and the chosen frame
     (autotune_<time>.npz) for rs_autotune.py --show on the PC. Your own realsense.json is not touched.

  bash characterization/throw_track/tt.sh rsautotune [--distance 2.5]        (Pi)
  python rs_autotune.py --show <autotune .npz>                                (PC: the frame, the ball, what else passes)
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (HERE.parent / "realsense", HERE.parents[1] / "characterization" / "realsense", HERE.parent / "src" / "omni_catch",
          HERE.parents[1] / "ros" / "src" / "omni_catch"):
    if p.is_dir():
        sys.path.insert(0, str(p))
sys.path.insert(0, str(HERE))

from omni_catch.detector import Detector, DetectorConfig       # noqa: E402
from omni_catch.frames import ArrayFrame                       # noqa: E402

OUT = Path.home() / "omni_ws" / "ball_view_settings" / "realsense_auto.json"
BALL_D = 0.1607                          # the orange foam test ball, m
EXPOSURES_US = (2000, 3000, 5000, 8000, 12000)
GAINS = (64, 128)
WHITE_BALANCES = (2800, 3100, 3400, 3700, 4000, 4400, 4800, 5400, 6000)      # K, the D435 colour sensor's range is 2800-6500


# ----------------------------------------------------------------------------------------------- colour (ball_view's definitions)
def hsv_of(rgb: np.ndarray):
    """(hue deg, saturation, value) per pixel as the detector's ball_mask defines them: value = max / 255, saturation = chroma / max."""
    c = rgb.reshape(-1, 3).astype(np.float32)
    r, g, b = c[:, 0], c[:, 1], c[:, 2]
    mx, mn = c.max(axis=1), c.min(axis=1)
    ch = mx - mn
    cz = np.maximum(ch, 1e-6)
    h = np.where(mx == r, (60 * (g - b) / cz) % 360, np.where(mx == g, 60 * ((b - r) / cz + 2), 60 * ((r - g) / cz + 4)))
    return h, ch / np.maximum(mx, 1e-6), mx / 255.0


def disc(shape, u, v, r):
    yy, xx = np.mgrid[0:shape[0], 0:shape[1]]
    return (xx + 0.5 - u) ** 2 + (yy + 0.5 - v) ** 2 <= r * r


def tune_colour(px: np.ndarray, margin_deg: float = 8.0, slack: float = 0.85) -> dict | None:
    """ball_view.tune_colour's rule (that file needs PIL, the Pi's rs venv has none)."""
    h, s, v = hsv_of(px)
    ok = (s >= 0.2) & (v >= 0.08)
    if ok.sum() < max(12, 0.15 * h.size):
        return None
    hist = np.bincount(np.minimum(h[ok].astype(int), 359), minlength=360).astype(float)
    peak = float(np.argmax(sum(np.roll(hist, k) for k in range(-15, 16)))) + 0.5
    dh = (h - peak + 180) % 360 - 180
    keep = ok & (np.abs(dh) <= 30)
    if keep.sum() < 10:
        return None
    lo, hi = np.percentile(dh[keep], [2, 98])
    hmin, hmax = peak + lo - margin_deg, peak + hi + margin_deg
    return dict(hue_min=float(max(0, math.floor(hmin))), hue_max=float(min(360, math.ceil(hmax))),
                sat_min=round(float(np.clip(slack * np.percentile(s[keep], 5), 0.05, 0.95)), 3),
                val_min=round(float(np.clip(slack * np.percentile(v[keep], 5), 0.03, 0.95)), 3),
                wraps=bool(hmin < 0 or hmax > 360), peak=peak, v_p50=float(np.median(v[keep])), v_p95=float(np.percentile(v[keep], 95)),
                s_p50=float(np.median(s[keep])))


# ----------------------------------------------------------------------------------------------- finding the ball
def find_ball(rgb: np.ndarray, depth_m: np.ndarray | None, f_px: float, distance: float, near: tuple | None = None) -> dict | None:
    """The ball without a colour window: saturated pixels (s >= 0.35, v >= 0.06) in blobs, each blob's expected radius from its own
    depth (median of the aligned depth inside it, + 2R/3) or `distance`; kept if round and 0.4-2x the expected area; the one nearest
    `near` (default the image centre) wins. Works at half resolution with numpy only (the Pi's rs venv has no scipy).
    {u, v, r_blob, r_exp, range_m, range_from} in full-resolution pixels."""
    from omni_catch.detector import label
    k2 = 2
    small = rgb[::k2, ::k2]
    h, s, v = (a.reshape(small.shape[:2]) for a in hsv_of(small))
    m = (s >= 0.35) & (v >= 0.06)
    core = m.copy()                                      # a 3x3 opening: erode then dilate (drops 1-px specks and thin lines)
    core[1:-1, 1:-1] = m[1:-1, 1:-1] & m[:-2, 1:-1] & m[2:, 1:-1] & m[1:-1, :-2] & m[1:-1, 2:]
    m = core.copy()
    m[1:-1, 1:-1] |= core[:-2, 1:-1] | core[2:, 1:-1] | core[1:-1, :-2] | core[1:-1, 2:]
    lab, n = label(m)
    if n == 0:
        return None
    H, W = rgb.shape[:2]
    cu, cv = near if near else (W / 2, H / 2)
    ys, xs = np.nonzero(lab)
    ks = lab[ys, xs]
    order = np.argsort(ks, kind="stable")
    ys, xs, ks = ys[order], xs[order], ks[order]
    starts = np.r_[0, np.nonzero(np.diff(ks))[0] + 1]
    dsm = depth_m[::k2, ::k2] if depth_m is not None else None
    best = None
    for i0, i1 in zip(starts, np.r_[starts[1:], len(ks)]):
        area = (i1 - i0) * k2 * k2                       # full-resolution pixels
        if area < 30:
            continue
        yk, xk = ys[i0:i1], xs[i0:i1]
        hh, ww = (yk.max() - yk.min() + 1) * k2, (xk.max() - xk.min() + 1) * k2
        if max(hh, ww) / max(1, min(hh, ww)) > 1.6 or area / (math.pi * (max(hh, ww) / 2) ** 2) < 0.5:
            continue                                     # not round
        u, vv = (xk.mean() + 0.5) * k2, (yk.mean() + 0.5) * k2
        z, src = distance, "given"
        if dsm is not None:
            zs = dsm[yk, xk]
            zs = zs[(zs > 0.2) & (zs < 10)]
            if len(zs) >= 0.3 * (i1 - i0):
                z, src = float(np.median(zs)) + BALL_D / 3, "depth"
        r_exp = f_px * BALL_D / 2 / z
        ratio = area / (math.pi * r_exp ** 2)
        if not 0.4 <= ratio <= 2.0:
            continue
        d = math.hypot(u - cu, vv - cv)
        if best is None or d < best["dist_px"]:
            best = dict(u=float(u), v=float(vv), r_blob=math.sqrt(area / math.pi), r_exp=float(r_exp), range_m=float(z), range_from=src,
                        area_ratio=float(ratio), dist_px=float(d), hue=float(np.median(h[yk, xk])))
    return best


def manual_ball(rgb: np.ndarray, depth_m: np.ndarray | None, f_px: float, distance: float, at: tuple) -> dict:
    """The ball where the operator says (`--at U V`: its centre pixel), for a ball that touches something of its own colour and so isn't
    a round blob (find_ball skips it). The range is the median aligned depth on a small disc, else `distance`."""
    u, v = float(at[0]), float(at[1])
    z, src = distance, "given"
    if depth_m is not None:
        zs = depth_m[disc(depth_m.shape, u, v, 6)]
        zs = zs[(zs > 0.2) & (zs < 10)]
        if len(zs) >= 20:
            z, src = float(np.median(zs)) + BALL_D / 3, "depth"
    h, s, _ = hsv_of(rgb[disc(rgb.shape, u, v, 6)])
    return dict(u=u, v=v, r_blob=float("nan"), r_exp=float(f_px * BALL_D / 2 / z), range_m=z, range_from=src, area_ratio=1.0, dist_px=0.0,
                hue=float(np.median(h)))


def score(rgb: np.ndarray, ball: dict, margin: float, slack: float, coarse_stride: int = 4) -> dict | None:
    """Tune on the ball's disc and check the window on the whole frame: the ball's blob radius / expected, the other blobs that pass."""
    t = tune_colour(rgb[disc(rgb.shape, ball["u"], ball["v"], 0.75 * ball["r_exp"])], margin, slack)
    if t is None or t["wraps"] and t["hue_min"] <= 0 and t["hue_max"] >= 360:
        return None
    cfg = replace(DetectorConfig(), hue_deg=(t["hue_min"], t["hue_max"]), min_sat=t["sat_min"], min_val=t["val_min"],
                  min_blob_px=8, coarse_stride=coarse_stride, size_gain=1.0)
    cands = Detector(cfg).candidates(ArrayFrame(rgb), 12)
    near = [c for c in cands if math.hypot(c[0] - ball["u"], c[1] - ball["v"]) < 0.6 * ball["r_exp"] + 3]
    if not near:
        return dict(tune=t, cfg=cfg, found=False, fill=0.0, others=[(float(c[0]), float(c[1]), float(c[2])) for c in cands])
    b = max(near, key=lambda c: c[3])
    others = [(float(c[0]), float(c[1]), float(c[2])) for c in cands if c is not b and c[2] >= 4.0]
    return dict(tune=t, cfg=cfg, found=True, fill=float(b[2] / ball["r_exp"]), blob=(float(b[0]), float(b[1]), float(b[2]), int(b[3])),
                others=others)


def choose(trials: list) -> dict | None:
    """The shortest exposure (then the lower gain) whose ball comes out whole (blob radius 0.75-1.3 x expected) with the fewest other
    blobs (within one of the fewest), not clipped (v_p95 < 0.98). Else the best fill. (A floor on the ball's brightness rejected the only
    good setting in a warm-lit room: the ball's value is ~0.2 there.)"""
    good = [x for x in trials if x["s"] and x["s"]["found"] and 0.75 <= x["s"]["fill"] <= 1.3 and x["s"]["tune"]["v_p95"] < 0.98]
    if good:
        fewest = min(len(x["s"]["others"]) for x in good)
        good = [x for x in good if len(x["s"]["others"]) <= fewest + 1]
        return min(good, key=lambda x: (x["exposure_us"], x["gain"]))
    found = [x for x in trials if x["s"] and x["s"]["found"]]
    return max(found, key=lambda x: x["s"]["fill"]) if found else None


# ----------------------------------------------------------------------------------------------- the camera
class Cam:
    def __init__(self, fps=30):
        import rs_common as C
        self.C, self.rs = C, C.need_rs()
        rs = self.rs
        self.dev = C.find_device(None)
        cfg = rs.config()
        cfg.enable_device(C.info(self.dev, "serial_number"))
        cfg.enable_stream(rs.stream.color, 848, 480, rs.format.rgb8, fps)
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)
        self.pipe = rs.pipeline()
        prof = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.scale = self.dev.first_depth_sensor().get_depth_scale()
        self.intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.cs = C.color_sensor(self.dev)
        C.set_opt(self.cs, "auto_exposure_priority", 0)

    def grab(self, n: int, settle: int = 6):
        """Median of n frames (after `settle` dropped): colour uint8, aligned depth m."""
        cols, deps = [], []
        for i in range(settle + n):
            fs = self.align.process(self.pipe.wait_for_frames(2000))
            if i < settle:
                continue
            cols.append(np.asanyarray(fs.get_color_frame().get_data()).copy())
            d = fs.get_depth_frame()
            deps.append(np.asanyarray(d.get_data()).astype(np.float32) * self.scale)
        return np.median(np.stack(cols), axis=0).astype(np.uint8), np.median(np.stack(deps), axis=0), cols

    def set(self, exposure_us=None, gain=None, awb=None, wb=None):
        C = self.C
        if awb is not None:
            C.set_opt(self.cs, "enable_auto_white_balance", 1 if awb else 0)
        if wb is not None:
            C.set_opt(self.cs, "white_balance", wb)
        if exposure_us is not None:
            C.set_opt(self.cs, "enable_auto_exposure", 0)
            C.set_opt(self.cs, "exposure", max(1, round(exposure_us / 100)))         # 100 us units
        if gain is not None:
            C.set_opt(self.cs, "gain", gain)

    def get(self, name):
        return self.C.get_opt(self.cs, name)

    def close(self):
        self.pipe.stop()


def run(a):
    cam = Cam()
    try:
        f_px = 0.5 * (cam.intr.fx + cam.intr.fy)
        print(f"colour f {f_px:.1f} px; 1/2: white balance in the room's light ...")
        cam.set(exposure_us=8000, gain=64, awb=True)
        cam.grab(45, settle=0)
        wb_auto = cam.get("white_balance")
        # The camera's auto white balance left a warm-lit room orange (measured Oct 30: 4600 K, walls and ceiling at saturation ~0.55,
        # the ball's own): pick the value that makes the room greyest instead (grey world), so the ball is the saturated thing in view.
        cam.set(awb=False)
        greys = []
        for wb_try in WHITE_BALANCES:
            cam.set(wb=wb_try)
            rgb, _, _ = cam.grab(3, settle=4)
            _, s, v = hsv_of(rgb[::4, ::4])
            ok = (v > 0.08) & (v < 0.95)
            greys.append((float(np.median(s[ok])) if ok.any() else 1.0, wb_try))
        wb = min(greys)[1]
        cam.set(wb=wb)
        print(f"   auto said {wb_auto} K; the room is greyest at {wb} K (median saturation by K: "
              + ", ".join(f"{k:.0f} {m:.2f}" for m, k in greys) + ")")
        rgb, dep, _ = cam.grab(8)
        ball = find_ball(rgb, dep, f_px, a.distance, tuple(a.near) if a.near else None) if not a.at else manual_ball(rgb, dep, f_px, a.distance, a.at)
        if ball is None:
            for e, g in ((8000, 128), (12000, 128)):                    # dim: look again brighter
                cam.set(exposure_us=e, gain=g)
                rgb, dep, _ = cam.grab(8)
                ball = find_ball(rgb, dep, f_px, a.distance, tuple(a.near) if a.near else None)
                if ball:
                    break
        if ball is None:
            raise SystemExit("no round saturated ball-sized blob in view: is the ball still, in view, not cut by the edge?")
        print(f"2/2: the ball at ({ball['u']:.0f},{ball['v']:.0f}) px, {ball['range_m']:.2f} m (from {ball['range_from']}), "
              f"expected radius {ball['r_exp']:.1f} px, hue ~{ball['hue']:.0f} deg; sweeping exposure x gain ...")
        trials = []
        for g in GAINS:
            for e in EXPOSURES_US:
                cam.set(exposure_us=e, gain=g)
                rgb, dep, frames = cam.grab(8)
                b = ball                                                 # the ball is still: its place and range stay (a re-search drifted to a blue object)
                s = score(rgb, b, 8.0, 0.85)
                trials.append(dict(exposure_us=e, gain=g, s=s, ball=b, rgb=rgb, frames=frames))
                t = s["tune"] if s else None
                print(f"   {e / 1000:4.1f} ms gain {g:3d}: " + ("no colour" if not s else
                      f"hue {t['hue_min']:.0f}-{t['hue_max']:.0f} sat>={t['sat_min']:.2f} val>={t['val_min']:.2f} (ball v {t['v_p50']:.2f}), "
                      f"ball {'r %.1f px = %.0f%% of expected' % (s['blob'][2], 100 * s['fill']) if s['found'] else 'NOT found'}, "
                      f"{len(s['others'])} other blob(s)"))
        best = choose(trials)
        if best is None:
            raise SystemExit("no setting brought the ball out as a blob")
        # tighten while decoys pass and the ball stays whole
        s = best["s"]
        for margin, slack in ((5.0, 0.9), (3.0, 0.95), (2.0, 1.0)):
            if not s["others"]:
                break
            s2 = score(best["rgb"], best["ball"], margin, slack)
            if s2 and s2["found"] and s2["fill"] >= 0.75 and len(s2["others"]) < len(s["others"]):
                s = s2
        # the window on each single frame (noise): the ball found in how many?
        det = Detector(s["cfg"])
        hits = sum(1 for fr in best["frames"] if any(math.hypot(c[0] - best["ball"]["u"], c[1] - best["ball"]["v"]) < 0.6 * best["ball"]["r_exp"] + 3
                                                     for c in det.candidates(ArrayFrame(fr), 12)))
        t, cfg = s["tune"], s["cfg"]
        size_gain = float(np.clip(best["ball"]["r_exp"] / s["blob"][2], 0.8, 1.6))
        out = {"format": "ball_view/1", "saved": time.strftime("%Y-%m-%dT%H:%M:%S"), "source": "realsense", "by": "rs_autotune.py",
               "detector": dict(hue_deg=[t["hue_min"], t["hue_max"]], min_sat=t["sat_min"], min_val=t["val_min"], min_blob_px=8,
                                size_gain=round(size_gain, 3), coarse_stride=4),
               "camera": dict(exposure_us=best["exposure_us"], gain=best["gain"], awb=False, white_balance=wb),
               "ball_mm": BALL_D * 1000, "focal_px": f_px, "image_px": [int(cam.intr.width), int(cam.intr.height)],
               "autotune": dict(ball=best["ball"], blob=s["blob"], fill=s["fill"], others=s["others"], single_frame_hits=f"{hits}/{len(best['frames'])}",
                                ball_v_median=t["v_p50"], ball_s_median=t["s_p50"],
                                sweep=[dict(exposure_us=x["exposure_us"], gain=x["gain"], found=bool(x["s"] and x["s"]["found"]),
                                            fill=x["s"]["fill"] if x["s"] else None, others=len(x["s"]["others"]) if x["s"] else None,
                                            tune=x["s"]["tune"] if x["s"] else None) for x in trials])}
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(out, indent=1, default=float))
        snap = Path(a.out).parent / f"autotune_{time.strftime('%Y%m%d_%H%M%S')}.npz"
        np.savez_compressed(snap, rgb=best["rgb"], settings=json.dumps(out, default=float))
        print(f"\nchosen: {best['exposure_us'] / 1000:.1f} ms, gain {best['gain']}, white balance {wb} K; hue {t['hue_min']:.0f}-{t['hue_max']:.0f} deg, "
              f"sat >= {t['sat_min']:.2f}, val >= {t['val_min']:.2f}, size_gain {size_gain:.2f}")
        print(f"ball r {s['blob'][2]:.1f} px of {best['ball']['r_exp']:.1f} expected ({100 * s['fill']:.0f}%), found in {hits}/{len(best['frames'])} single frames; "
              f"{len(s['others'])} other blob(s) pass: {[(round(o[0]), round(o[1]), round(o[2], 1)) for o in s['others']]}")
        print(f"wrote {a.out}\nsnapshot {snap}")
    finally:
        cam.close()


def show(npz: str, out: str | None = None):
    """PC: the chosen frame, the ball (green), the expected size (dashed), what else passes the window (red), the mask."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from omni_catch.detector import ball_mask
    z = np.load(npz)
    rgb, st = z["rgb"], json.loads(str(z["settings"]))
    d, at = st["detector"], st["autotune"]
    cfg = replace(DetectorConfig(), hue_deg=tuple(d["hue_deg"]), min_sat=d["min_sat"], min_val=d["min_val"])
    fig, ax = plt.subplots(1, 2, figsize=(15, 4.6))
    ax[0].imshow(rgb)
    b = at["ball"]
    ax[0].add_patch(plt.Circle((b["u"], b["v"]), b["r_exp"], fill=False, ls="--", color="w", lw=0.8))
    ax[0].add_patch(plt.Circle(at["blob"][:2], at["blob"][2], fill=False, color="lime", lw=1.2))
    for o in at["others"]:
        ax[0].add_patch(plt.Circle(o[:2], o[2] + 4, fill=False, color="r", lw=1.2))
    ax[0].set_title(f"{st['camera']['exposure_us'] / 1000:.1f} ms gain {st['camera']['gain']}, WB {st['camera']['white_balance']} K; ball {b['range_m']:.2f} m "
                    f"({b['range_from']}), r {at['blob'][2]:.1f}/{b['r_exp']:.1f} px; {len(at['others'])} other", fontsize=9)
    ax[1].imshow(ball_mask(rgb, cfg), cmap="gray")
    ax[1].set_title(f"mask: hue {d['hue_deg'][0]:.0f}-{d['hue_deg'][1]:.0f}, sat >= {d['min_sat']:.2f}, val >= {d['min_val']:.2f}", fontsize=9)
    for a_ in ax:
        a_.set_xticks([])
        a_.set_yticks([])
    fig.tight_layout()
    out = out or str(Path(npz).with_suffix(".png"))
    fig.savefig(out, dpi=100)
    print(f"wrote {out}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--distance", type=float, default=2.5, help="the ball's range, m, if the depth map has none there")
    ap.add_argument("--near", type=float, nargs=2, help="look for the ball near this pixel (default: the image centre)")
    ap.add_argument("--at", type=float, nargs=2, metavar=("U", "V"), help="the ball's centre pixel, when it touches something of its own colour")
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--show", help="PC: draw an autotune_*.npz")
    a = ap.parse_args(argv)
    sys.stdout.reconfigure(line_buffering=True)
    if a.show:
        show(a.show)
    else:
        run(a)


if __name__ == "__main__":
    main()
