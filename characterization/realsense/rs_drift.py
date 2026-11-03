"""Step 5: calibration drift from cold, over hours and between days (README.md).

design.md 4.1 / 8 (phase 1C): stereo calibration stability decides the catch rate (0.04 deg -> 97%,
0.3 deg -> 78%, 0.5 deg -> 58%); the pass band is <~ 0.1 deg between sessions. The D435 is a factory-
calibrated pair on a 50 mm aluminium stiffener: how much does it move while it warms up, and from day to
day?

Fix the camera rigidly (clamp or tape it to a table, not a wobbly tripod) facing a flat matte wall or the
floor 1.5-3 m away, start it cold (unplugged for 30 min) and leave it for --minutes. Every --every
seconds it fits the plane over --frames frames and logs the distance, the plane's normal and the ASIC and
projector temperatures. A drift in depth dz at range z is a drift in disparity, read as the pair's
relative yaw: B dz / z^2 rad, the design's calibration angle. A drift in the normal is the pair (or the
mount) tilting.

    python rs_drift.py --minutes 60
    python rs_drift.py --minutes 10 --label "after knocks"      # same setup, another day / after handling

Each sample is appended to results/drift_<serial>_<time>.jsonl as it's taken; a summary JSON and a plot
come at the end (Ctrl+C ends early and still writes them).
"""
from __future__ import annotations

import argparse
import datetime
import json
import time

import numpy as np

import rs_common as C
from rs_common import rs
from rs_depth_plane import robust_plane, roi_mask


def sample(pipe, n, scale, intr, mask):
    ds, normals, rms, fill = [], [], [], []
    for _ in range(n):
        z = np.asanyarray(pipe.wait_for_frames(3000).get_depth_frame().get_data()).astype(np.float32) * scale
        valid = (z > 0) & mask
        fill.append(valid.sum() / mask.sum())
        pts = C.deproject(z, intr["fx"], intr["fy"], intr["ppx"], intr["ppy"], mask=mask, step=2)
        if len(pts) < 500:
            continue
        nrm, d, r = robust_plane(pts)
        ds.append(d)
        normals.append(nrm)
        rms.append(float(np.sqrt(np.mean(r ** 2))))
    if not ds:
        return None
    n_mean = np.mean(normals, axis=0)
    n_mean /= np.linalg.norm(n_mean)
    return dict(dist_m=float(np.median(ds)), dist_std_m=float(np.std(ds)), normal=n_mean.tolist(),
                rms_mm=float(np.median(rms) * 1000), fill=float(np.median(fill)))


def summarize(rows: list[dict], baseline_m: float) -> dict:
    if len(rows) < 2:
        return dict(samples=len(rows))
    t = np.array([r["t_s"] for r in rows]) / 60.0
    d = np.array([r["dist_m"] for r in rows])
    z0 = float(np.median(d[:3]))
    ang = C.depth_change_to_angle_deg(z0, d - z0, baseline_m)
    n0 = np.array(rows[0]["normal"])
    tilt = np.array([C.angle_between_deg(r["normal"], n0) for r in rows])
    asic = [r["temperatures"].get("asic_temperature") for r in rows]
    proj = [r["temperatures"].get("projector_temperature") for r in rows]

    def span(x, sel):
        x = np.asarray(x, float)[sel]
        return float(np.nanmax(x) - np.nanmin(x)) if x.size else None

    warm = t <= 20
    late = t > 20
    out = dict(samples=len(rows), minutes=float(t[-1]), range_m=z0, baseline_m=baseline_m,
               depth_drift_mm=dict(all=span(d * 1000, slice(None)), first_20_min=span(d * 1000, warm),
                                   after_20_min=span(d * 1000, late) if late.any() else None),
               equiv_pair_yaw_deg=dict(all=span(ang, slice(None)), first_20_min=span(ang, warm),
                                       after_20_min=span(ang, late) if late.any() else None,
                                       end_minus_start=float(ang[-1] - ang[0])),
               normal_drift_deg=dict(max=float(np.max(tilt)), end=float(tilt[-1])),
               asic_temperature_c=[min(v for v in asic if v is not None), max(v for v in asic if v is not None)]
               if any(v is not None for v in asic) else None,
               projector_temperature_c=[min(v for v in proj if v is not None), max(v for v in proj if v is not None)]
               if any(v is not None for v in proj) else None,
               pass_band_deg=C.REQ["calib_drift_deg"], sim_good_deg=C.REQ["calib_drift_good_deg"])
    a = out["equiv_pair_yaw_deg"]["all"]
    out["verdict"] = ("inside the sim's 0.04 deg" if a <= C.REQ["calib_drift_good_deg"] else
                      "inside the 0.1 deg pass band" if a <= C.REQ["calib_drift_deg"] else "outside the 0.1 deg pass band")
    return out


def plot(rows, summ, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.array([r["t_s"] for r in rows]) / 60.0
    d = np.array([r["dist_m"] for r in rows])
    z0 = summ["range_m"]
    ang = C.depth_change_to_angle_deg(z0, d - z0, summ["baseline_m"])
    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax[0].plot(t, (d - z0) * 1000, label="depth change (mm)")
    ax0b = ax[0].twinx()
    ax0b.plot(t, ang, color="C1", alpha=0.6, label="equivalent pair yaw (deg)")
    ax0b.axhspan(-C.REQ["calib_drift_deg"], C.REQ["calib_drift_deg"], color="C2", alpha=0.08)
    ax[0].set_ylabel("depth change, mm")
    ax0b.set_ylabel("equiv. yaw, deg")
    ax[0].set_title(f"D435 drift at {z0:.2f} m (shaded: the 0.1 deg pass band)")
    for key, lab in (("asic_temperature", "ASIC"), ("projector_temperature", "projector")):
        v = [r["temperatures"].get(key) for r in rows]
        if any(x is not None for x in v):
            ax[1].plot(t, [np.nan if x is None else x for x in v], label=lab)
    ax[1].set_ylabel("deg C")
    ax[1].set_xlabel("minutes")
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial")
    ap.add_argument("--mode", default="depth:848x480@30")
    ap.add_argument("--minutes", type=float, default=60)
    ap.add_argument("--every", type=float, default=15, help="seconds between samples")
    ap.add_argument("--frames", type=int, default=15, help="frames per sample")
    ap.add_argument("--roi", type=float, default=0.4)
    ap.add_argument("--preset", default="high_accuracy")
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    dev = C.find_device(a.serial)
    cfg = rs.config()
    cfg.enable_device(C.info(dev, "serial_number"))
    mode = C.parse_mode(a.mode)
    C.enable_streams(cfg, mode)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    C.configure_stereo(dev, emitter=True, preset=a.preset)
    scale = C.depth_sensor(dev).get_depth_scale()
    intr = C.intrinsics_dict(prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics())
    mask = roi_mask(intr["height"], intr["width"], a.roi)
    baseline = C.ir_baseline_m(dev)

    C.RESULTS.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    serial = C.info(dev, "serial_number")
    jl = C.RESULTS / f"drift_{serial}_{stamp}.jsonl"
    rows = []
    t0 = time.time()
    print(f"logging to {jl.name}; Ctrl+C ends early")
    try:
        with jl.open("w") as f:
            while time.time() - t0 < a.minutes * 60:
                ts = time.time()
                s = sample(pipe, a.frames, scale, intr, mask)
                if s is not None:
                    s.update(t_s=ts - t0, temperatures=C.temperatures(dev))
                    rows.append(s)
                    f.write(json.dumps(s) + "\n")
                    f.flush()
                    z0 = rows[0]["dist_m"]
                    ang = float(C.depth_change_to_angle_deg(z0, s["dist_m"] - z0, baseline))
                    T = s["temperatures"]
                    print(f"{s['t_s'] / 60:6.1f} min  {s['dist_m']:.4f} m  ({1000 * (s['dist_m'] - z0):+.2f} mm, "
                          f"{ang:+.4f} deg)  ASIC {T.get('asic_temperature')} C  projector "
                          f"{T.get('projector_temperature')} C")
                # keep streaming between samples (the camera stays at its working temperature)
                while time.time() - ts < a.every:
                    pipe.wait_for_frames(3000)
    except KeyboardInterrupt:
        print("stopped")
    finally:
        pipe.stop()
    summ = summarize(rows, baseline)
    if len(rows) >= 2:
        png = C.RESULTS / f"drift_{serial}_{stamp}.png"
        plot(rows, summ, png)
        summ["plot"] = png.name
        e = summ["equiv_pair_yaw_deg"]
        print(f"\nover {summ['minutes']:.0f} min: depth drift {summ['depth_drift_mm']['all']:.2f} mm at "
              f"{summ['range_m']:.2f} m = {e['all']:.4f} deg of pair yaw ({summ['verdict']}); "
              f"normal drift {summ['normal_drift_deg']['max']:.3f} deg")
    C.save_result("drift", dict(mode=a.mode, preset=a.preset, label=a.label, log=jl.name, summary=summ,
                                intrinsics=intr), dev)


if __name__ == "__main__":
    main()
