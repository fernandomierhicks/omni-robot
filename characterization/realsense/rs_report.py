"""Step 8: everything measured so far, against design.md's requirements (README.md).

Reads the newest results/*.json of each test (all of them for plane, ball and drift) and writes
results/REPORT.md and its plots: the requirement checks (compute_choice.py's CAMERA_REQ + phase 1C's pass
bands), timing per mode, depth noise against range with the stereo model fitted, the ball at range,
throws, drift, and the values to carry into database/cameras.csv (meas_*) and the sim (vision.py
VisionConfig, randomize.py ranges). Doesn't touch either: the numbers are copied over by hand, with
their source, once they're trusted (design.md 8, rule 2).

    python rs_report.py
"""
from __future__ import annotations

import datetime

import numpy as np

import rs_common as C


def fmt(x, nd=1, unit=""):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return "-"
    return f"{x:.{nd}f}{unit}"


def fit_disparity_sigma(z, rms, f, B):
    """Least-squares sigma_d in rms = z^2 sigma_d / (f B)."""
    z, rms = np.asarray(z, float), np.asarray(rms, float)
    g = z ** 2 / (f * B)
    return float((g @ rms) / (g @ g)) if len(z) else None


def plot_plane(caps, f, B, sd, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    z = np.array([c["perpendicular_m"] for c in caps])
    rms = np.array([c["spatial_rms_mm"]["median"] for c in caps])
    zz = np.linspace(0.2, 7.5, 100)
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(z, rms, "o", label="measured, plane-fit RMS")
    if sd:
        ax.plot(zz, 1000 * C.stereo_sigma_z(zz, f, B, sd), "-", label=f"z^2 sd/(fB), sd {sd:.3f} px (D435)")
        ax.plot(zz, 1000 * C.stereo_sigma_z(zz, C.SIM_F_PX, C.SIM_BASELINE_M, sd), "--",
                label="same sd on the design's 190 mm bar")
    ax.plot(zz, 40 * (zz / 2) ** 2, ":", color="gray", label="datasheet limit 2% at 2 m (z^2 scaled)")
    ax.axvspan(*C.THROW_RANGE_M, color="C3", alpha=0.08, label="thrower 6-7 m")
    ax.set_xlabel("range, m")
    ax.set_ylabel("depth noise, mm")
    ax.set_yscale("log")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    lines = ["# RealSense D435 characterization report", "",
             f"Generated {datetime.datetime.now():%Y-%m-%d %H:%M} by `rs_report.py` from `results/`. Measured on "
             "the bench, not the robot; datasheet values are marked so. The design's requirements: design.md 4.1, "
             "4.9 and section 8 (phase 1C).", ""]
    info = C.latest("info")
    if info is None:
        lines += ["No `info` result yet: run `rs_info.py` first.", ""]
    else:
        d = info["device"]
        g = info["geometry"]
        lines += ["## Device", "",
                  f"- {d['name']}, serial {d['serial_number']}, firmware {d['firmware_version']} "
                  f"(recommended {d['recommended_firmware_version']}), USB {d['usb_type_descriptor']}, "
                  f"librealsense {d['librealsense']}",
                  f"- IR baseline {g.get('ir1_to_ir2', {}).get('baseline_mm', '-')} mm (calibration; datasheet 50); "
                  + ", ".join(f"{k} fx {v['fx']:.0f} px FOV {v['fov_deg'][0]}x{v['fov_deg'][1]}"
                              for k, v in g.items() if k in ("infrared1_848x480", "infrared1_1280x800", "color_640x480")),
                  "- Timestamps: " + ", ".join(f"{k} {v.get('domain', v.get('error'))}"
                                               for k, v in info.get("timestamps", {}).items()), ""]

    # ------------------------------------------------ requirements
    timing, flash = C.latest("timing"), C.latest("flash")
    drifts = C.all_results("drift")
    rows = []
    if info:
        for k, v in info["requirements"].items():
            rows.append((k, v))
    lat_ir = None
    if timing:
        cands = [(m["mode"], s["latency_mid_exposure_to_python_ms"]) for m in timing["modes"]
                 for st, s in m.get("streams", {}).items()
                 if st.startswith("infrared") and s.get("latency_mid_exposure_to_python_ms", {}).get("n")]
        if cands:
            mode, lat_ir = min(cands, key=lambda c: c[1]["p95"])
            ok = lat_ir["p95"] <= C.REQ["latency_max_ms"]
            rows.append(("capture -> host (rs_timing, best IR mode)",
                         f"{'PASS' if ok else 'FAIL'}: {mode} median {lat_ir['median']:.1f}, p95 {lat_ir['p95']:.1f} ms "
                         f"of {C.REQ['latency_max_ms']:.0f} (detection comes on top)"))
    if flash and flash["camera_ms"].get("n"):
        cm = flash["camera_ms"]
        rows.append(("photons -> host (rs_flash_latency)",
                     f"{flash['mode']}: camera part median {cm['median']:.1f}, p95 {cm['p95']:.1f} ms; display "
                     f"{fmt(flash['display_ms'].get('median'))} ms; {flash['matched']} flips"))
    if drifts:
        worst = max(drifts, key=lambda r: r["summary"].get("equiv_pair_yaw_deg", {}).get("all") or 0)
        e = worst["summary"].get("equiv_pair_yaw_deg", {})
        if e.get("all") is not None:
            rows.append(("calibration drift (rs_drift, worst run)",
                         f"{e['all']:.4f} deg over {worst['summary']['minutes']:.0f} min ({worst['summary']['verdict']})"))
    if rows:
        lines += ["## Against the design's camera requirements", "", "| Check | Result |", "| --- | --- |"]
        lines += [f"| {k} | {v} |" for k, v in rows]
        lines += ["", "The catch's colour-blob detector needs colour + global shutter in one camera; the D435 has "
                      "global-shutter mono (IR pair) or rolling-shutter colour, not both. Uses that fit as it is: the "
                      "arm's floor depth, low obstacles under the lidar plane, a bench reference for the OV9282 "
                      "sensor family the design's OV9782 belongs to.", ""]

    # ------------------------------------------------ timing
    if timing:
        lines += ["## Timing per mode (rs_timing.py)", "",
                  "| Mode | Stream | fps | jitter ms | lost | exposure us | capture -> Python ms (median / p95 / max) | CPU cores |",
                  "| --- | --- | --- | --- | --- | --- | --- | --- |"]
        for m in timing["modes"]:
            if "streams" not in m:
                lines.append(f"| {m['mode']} | - | {m.get('skipped', m.get('error', ''))} | | | | | |")
                continue
            for st, s in m["streams"].items():
                iv = s["intervals_hw"] or s["intervals_host"]
                L = s.get("latency_mid_exposure_to_python_ms", {})
                lines.append(f"| {m['mode']} | {st} | {fmt(iv.get('fps'))} | {fmt(iv.get('dt_std_ms'), 2)} | "
                             f"{s['lost_by_counter']} | {fmt(s['exposure_us'].get('median'), 0)} | "
                             + (f"{L['median']:.1f} / {L['p95']:.1f} / {L['max']:.1f}" if L.get("n") else "no hw stamps")
                             + f" | {m['cpu_cores']:.2f} |")
        lines.append("")

    # ------------------------------------------------ depth on a plane
    planes = [c for r in C.all_results("plane") for c in r["captures"] if "error" not in c]
    if planes:
        r0 = C.latest("plane")
        f, B = r0["intrinsics"]["fx"], r0["baseline_m"]
        planes.sort(key=lambda c: c["perpendicular_m"])
        sd = fit_disparity_sigma([c["perpendicular_m"] for c in planes],
                                 [c["spatial_rms_mm"]["median"] / 1000 for c in planes], f, B)
        png = C.RESULTS / "report_plane.png"
        plot_plane(planes, f, B, sd, png)
        lines += ["## Depth on a flat target (rs_depth_plane.py)", "",
                  "| Range m | Fill % | Spatial RMS mm (%) | Temporal mm | Subpixel px | Normal noise deg | Tape bias mm |",
                  "| --- | --- | --- | --- | --- | --- | --- |"]
        for c in planes:
            lines.append(f"| {c['perpendicular_m']:.3f} | {100 * c['fill_rate']['median']:.1f} | "
                         f"{c['spatial_rms_mm']['median']:.2f} ({c['spatial_rms_pct']:.2f}) | "
                         f"{fmt(c['temporal_std_mm']['median'], 2)} | {c['disparity_sigma_px']:.3f} | "
                         f"{c['normal_noise_deg']['median']:.3f} | {fmt(c.get('bias_mm'), 1)} |")
        zc = float(np.mean(C.THROW_RANGE_M))
        lines += ["", f"Fitted subpixel noise {sd:.3f} px (`z^2 sd / (f B)`, f {f:.0f} px, B {1000 * B:.1f} mm). At "
                      f"{zc:.1f} m that is {100 * C.stereo_sigma_z(zc, f, B, sd):.1f} cm of dense-depth noise on a plane; "
                      f"the same pixel noise on the design's bar (190 mm, f {C.SIM_F_PX:.0f} px) "
                      f"{100 * C.stereo_sigma_z(zc, C.SIM_F_PX, C.SIM_BASELINE_M, sd):.1f} cm. A 74 mm ball there is "
                      f"{C.ball_radius_px(zc, f):.1f} px in radius on the D435.", "", f"![depth noise]({png.name})", ""]

    floors = [c for r in C.all_results("floor") for c in r["captures"] if "error" not in c]
    if floors:
        lines += ["## Pose from the floor (rs_depth_plane.py --floor)", "",
                  "| Height mm | Pitch deg (std) | Roll deg (std) | Objects found (height mm @ range m) |",
                  "| --- | --- | --- | --- |"]
        for c in floors:
            P = c["floor_pose"]
            objs = ", ".join(f"{o['max_height_mm']:.0f} @ {o['range_m']:.2f}" for o in c["objects"][:5]) or "none"
            lines.append(f"| {1000 * P['height_m']['median']:.1f} +- {1000 * P['height_m']['std']:.2f} | "
                         f"{P['pitch_deg']['median']:.2f} ({P['pitch_deg']['std']:.3f}) | "
                         f"{P['roll_deg']['median']:.2f} ({P['roll_deg']['std']:.3f}) | {objs} |")
        lines += ["", "One frame's pose noise against the sim's `ego_sigma` (1 mm) and the calibration angles "
                      "(0.02-0.1 deg): the design recovers the camera bar's pose from the floor plane.", ""]

    # ------------------------------------------------ ball
    balls = [(r, c) for r in C.all_results("ball") for c in r["captures"] if "error" not in c]
    if balls:
        lines += ["## The ball at range (rs_ball.py)", "",
                  "| Tape m | r px (seen / model) | Dense fill % | Dense centre std mm | Dense bias mm | Blob disparity std px | "
                  "Blob centre std mm | Blob bias mm | Same px noise on the design's bar, mm |",
                  "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
        for r, c in sorted(balls, key=lambda rc: rc[1].get("truth_centre_m") or rc[1]["dense"]["centre_m"].get("median", 0)):
            b = c.get("blob", {})
            dz = c["dense"]["centre_m"]
            lines.append(f"| {fmt(c.get('truth_centre_m'), 2)} | {c['r_seen_px']:.1f} / {c['r_model_px']:.1f} | "
                         f"{100 * c['dense']['fill']['median']:.0f} | {fmt(1000 * dz['std'] if dz.get('n') else None)} | "
                         f"{fmt(c['dense'].get('bias_mm'), 0)} | {fmt(b.get('disparity_sigma_px'), 3)} | "
                         f"{fmt(1000 * b['centre_m']['std'] if b else None)} | {fmt(b.get('bias_mm'), 0)} | "
                         f"{fmt(1000 * b['design_bar_sigma_m'] if b else None)} |")
        lines += ["", "The sim's detector assumes ~0.28 / sqrt(r_px) px of centre noise at 0.4 px edge noise "
                      "(vision.py VisionConfig.sigma_c / edge_sigma); compare the blob disparity std (two centres) "
                      "against sqrt(2) times that.", ""]

    # ------------------------------------------------ throws
    throws = [(r, t) for r in C.all_results("throw") for t in r["tracks"]]
    if throws:
        lines += ["## Throws (rs_throw.py)", "",
                  "| Recording | Points | Duration s | Frames with the ball % | Range m (first -> last) | Speed m/s | |a| m/s^2 | Residual mm | Blur |",
                  "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
        for r, t in throws:
            lines.append(f"| {r['bag'].split(chr(92))[-1].split('/')[-1]} | {t['detections']} | {t['duration_s']:.2f} | "
                         f"{100 * t['detection_rate']:.0f} | {t['max_range_m']:.2f} -> {t['last_range_m']:.2f} | "
                         f"{t['speed0']:.1f} | {t['a_norm']:.1f} | {1000 * t['rms_m']:.0f} | {t['elongation']['median']:.1f} |")
        lines.append("")

    # ------------------------------------------------ drift
    if drifts:
        lines += ["## Drift (rs_drift.py)", "",
                  "| Date | Label | Minutes | Range m | Depth drift mm (all / first 20 / after) | Equiv. pair yaw deg (all / first 20 / after) | Normal drift deg | ASIC C |",
                  "| --- | --- | --- | --- | --- | --- | --- | --- |"]
        for r in drifts:
            s = r["summary"]
            if "equiv_pair_yaw_deg" not in s:
                continue
            dd, e = s["depth_drift_mm"], s["equiv_pair_yaw_deg"]
            lines.append(f"| {r['date'][:16]} | {r.get('label', '')} | {s['minutes']:.0f} | {s['range_m']:.2f} | "
                         f"{fmt(dd['all'], 2)} / {fmt(dd['first_20_min'], 2)} / {fmt(dd['after_20_min'], 2)} | "
                         f"{fmt(e['all'], 4)} / {fmt(e['first_20_min'], 4)} / {fmt(e['after_20_min'], 4)} | "
                         f"{s['normal_drift_deg']['max']:.3f} | {s.get('asic_temperature_c')} |")
        lines += ["", "Day to day: the same setup (camera clamped, same wall, same distance) on several days; the "
                      "spread of the runs' start distances is the between-session drift.", ""]

    # ------------------------------------------------ carry-over
    lines += ["## Values to carry over (by hand, with their source)", ""]
    carry = []
    if lat_ir:
        carry.append(f"- `database/cameras.csv` (a new `intel-realsense-d435` row): `meas_latency_ms` = "
                     f"{lat_ir['median']:.1f} (capture -> host, median; p95 {lat_ir['p95']:.1f})")
        carry.append(f"- sim `vision.py` VisionConfig: `latency_mean` = {lat_ir['median'] / 1000:.4f} s + the "
                     f"detector's time; `latency_jitter` = {lat_ir['std'] / 1000:.4f} s (camera part only)")
    if timing:
        fps = [(m["mode"], s["intervals_hw"]["fps"]) for m in timing["modes"] for st, s in m.get("streams", {}).items()
               if st.startswith("infrared") and s.get("intervals_hw") and s["intervals_hw"].get("fps")]
        if fps:
            carry.append("- `meas_fps`: " + ", ".join(f"{m} {v:.1f}" for m, v in fps[:4]))
        lost = [(m["mode"], s["lost_by_counter"], s["frames"]) for m in timing["modes"]
                for st, s in m.get("streams", {}).items() if s["frames"]]
        if lost:
            worst = max(lost, key=lambda x: x[1] / max(1, x[2]))
            carry.append(f"- dropped frames (sim `drop_prob` floor): worst {worst[0]} {worst[1]} of {worst[2]}")
    if drifts:
        e = [r["summary"]["equiv_pair_yaw_deg"]["all"] for r in drifts if "equiv_pair_yaw_deg" in r["summary"]]
        if e:
            carry.append(f"- `meas_calib_drift_deg` = {max(e):.4f} (the worst run); sim `calib_rot_deg` / "
                         f"randomize.py `cam_calib_deg` (0.04-0.30 assumed)")
    lines += carry or ["- nothing measured yet"]
    lines += ["", "Rule (design.md 8): a value goes into the database and the sim with its source, then the scoring "
                  "study is rerun."]
    out = C.RESULTS / "REPORT.md"
    C.RESULTS.mkdir(exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
