"""Step 1: what the camera is and what it offers (README.md).

Device, firmware (against the recommended one), USB link, every stream mode, intrinsics, the IR pair's
baseline, the options that matter (sync, emitter, exposure), whether hardware timestamps and metadata
work (Windows needs a registry change: enable_metadata.ps1), and the static checks against design.md's
camera requirements.

    python rs_info.py              # writes results/info_<serial>_<time>.json
"""
from __future__ import annotations

import argparse
import time

import rs_common as C
from rs_common import rs

OPTIONS = ["visual_preset", "emitter_enabled", "laser_power", "enable_auto_exposure", "exposure", "gain",
           "inter_cam_sync_mode", "output_trigger_enabled", "global_time_enabled", "frames_queue_size",
           "emitter_on_off", "emitter_always_on", "hdr_enabled", "depth_units", "asic_temperature",
           "projector_temperature", "error_polling_enabled", "auto_exposure_priority", "enable_auto_white_balance",
           "power_line_frequency", "thermal_compensation"]


def sensor_report(s) -> dict:
    name = s.get_info(rs.camera_info.name)
    modes = {}
    for p in s.get_stream_profiles():
        if not p.is_video_stream_profile():
            continue
        v = p.as_video_stream_profile()
        key = f"{str(p.stream_type()).split('.')[-1]}{p.stream_index() or ''}"
        modes.setdefault(key, set()).add((v.width(), v.height(), p.fps(), str(p.format()).split(".")[-1]))
    opts = {}
    for o in s.get_supported_options():
        oname = C.option_name(o)
        if oname not in OPTIONS:
            continue
        try:
            r = s.get_option_range(o)
            opts[oname] = dict(value=s.get_option(o), min=r.min, max=r.max, step=r.step, default=r.default,
                               description=s.get_option_description(o))
        except RuntimeError as e:
            opts[oname] = dict(error=str(e))
    return dict(name=name,
                modes={k: sorted([list(m) for m in v], key=lambda m: (m[0] * m[1], m[2])) for k, v in modes.items()},
                options=opts)


def stereo_geometry(dev) -> dict:
    """IR1 / IR2 / depth / colour intrinsics at the catch-relevant modes, and the IR1 -> IR2 baseline."""
    out = {}
    s = C.depth_sensor(dev)
    profs = {}
    for p in s.get_stream_profiles():
        if not p.is_video_stream_profile():
            continue
        v = p.as_video_stream_profile()
        st = str(p.stream_type()).split(".")[-1] + str(p.stream_index() or "")
        profs.setdefault((st, v.width(), v.height()), v)
    for (st, w, h), v in sorted(profs.items()):
        if (w, h) in ((1280, 800), (1280, 720), (848, 480), (640, 480)):
            try:
                intr = v.get_intrinsics()
            except RuntimeError:         # the Linux V4L backend has none for some profiles (e.g. Y8 1280x800)
                continue
            fov = rs.rs2_fov(intr)
            out[f"{st}_{w}x{h}"] = dict(C.intrinsics_dict(intr), fov_deg=[round(fov[0], 2), round(fov[1], 2)])
    ir1 = next((v for (st, w, h), v in profs.items() if st == "infrared1" and (w, h) == (848, 480)), None)
    ir2 = next((v for (st, w, h), v in profs.items() if st == "infrared2" and (w, h) == (848, 480)), None)
    if ir1 is not None and ir2 is not None:
        ext = ir1.get_extrinsics_to(ir2)
        out["ir1_to_ir2"] = dict(rotation=list(ext.rotation), translation_m=list(ext.translation),
                                 baseline_mm=round(abs(ext.translation[0]) * 1000, 3))
    cs = C.color_sensor(dev)
    if cs is not None and ir1 is not None:
        cp = next((p.as_video_stream_profile() for p in cs.get_stream_profiles() if p.is_video_stream_profile()), None)
        if cp is not None:
            ext = ir1.get_extrinsics_to(cp)
            out["ir1_to_color"] = dict(rotation=list(ext.rotation), translation_m=list(ext.translation))
    out["depth_scale_m"] = s.get_depth_scale()
    return out


def timestamp_check(dev, seconds: float = 1.5) -> dict:
    """Stream depth and colour briefly and see which clock and metadata the frames carry."""
    res = {}
    # the first mode the link offers (USB 2 has no 848x480 depth / IR)
    tries = {"depth": ["depth:848x480@30", "depth:640x480@30", "depth:480x270@30"],
             "ir": ["ir:848x480@30", "ir:640x480@30", "ir:480x270@30"],
             "color": ["color:640x480@30", "color:424x240@30"]}
    for kind, modes in tries.items():
        mode = next((x for x in modes if C.mode_supported(dev, C.parse_mode(x))), None)
        if mode is None:
            res[kind] = dict(error=f"none of {modes} offered")
            continue
        m = C.parse_mode(mode)
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_device(C.info(dev, "serial_number"))
        C.enable_streams(cfg, m)
        try:
            pipe.start(cfg)
            C.set_opt(C.depth_sensor(dev), "global_time_enabled", 1)
            last = pipe.wait_for_frames(8000)[0]          # the RGB camera can take a few seconds to start
            t_end = time.time() + seconds
            while time.time() < t_end:
                last = pipe.wait_for_frames(3000)[0]
            ft = C.frame_times(last)
            md = [str(k).split(".")[-1] for k in rs.frame_metadata_value.__members__.values()
                  if last.supports_frame_metadata(k)]
            res[kind] = dict(mode=mode, domain=ft["domain"], metadata=md,
                             hw_timestamps=ft["domain"] in ("global_time", "hardware_clock"),
                             exposure_us=ft["exposure_us"])
        except RuntimeError as e:
            res[kind] = dict(error=str(e))
        finally:
            try:
                pipe.stop()
            except RuntimeError:
                pass
    return res


def requirement_checks(rep: dict) -> dict:
    """Static checks against design.md's camera requirements (the measured ones come from the other steps)."""
    stereo_modes = rep["sensors"].get("Stereo Module", {}).get("modes", {})
    color_modes = next((v["modes"] for k, v in rep["sensors"].items() if "RGB" in k), {})
    ir = stereo_modes.get("infrared1", []) or stereo_modes.get("infrared", [])
    col = color_modes.get("color", [])
    fast_ir = sorted({tuple(m[:3]) for m in ir if m[2] >= C.REQ["fps_min"] and m[0] >= 640 and m[1] >= 400})
    fast_col = sorted({tuple(m[:3]) for m in col if m[2] >= C.REQ["fps_min"] and m[0] >= 640 and m[1] >= 400})
    sync = rep["sensors"].get("Stereo Module", {}).get("options", {}).get("inter_cam_sync_mode")
    return {
        "ir_pair: global shutter": "PASS (OV9282, datasheet)",
        "ir_pair: colour": "FAIL: mono (the robot's colour-blob detector would need a shape / motion detector)",
        "ir_pair: >= 60 fps at >= 640x400": ("PASS: " + ", ".join(f"{m[0]}x{m[1]}@{m[2]}" for m in fast_ir[:6]))
                                             if fast_ir else "FAIL",
        "rgb: global shutter": "FAIL: rolling shutter (OV2740, datasheet); the sweep put rolling 20 ms at 72%",
        "rgb: >= 60 fps at >= 640x400": ("PASS: " + ", ".join(f"{m[0]}x{m[1]}@{m[2]}" for m in fast_col[:4]))
                                         if fast_col else "FAIL",
        "hw sync (inter_cam_sync_mode)": (f"offered, range {sync.get('min')}..{sync.get('max')}: VERIFY a slave "
                                          "mode follows an external 1.8 V pulse") if sync and "min" in sync
                                         else "not offered by this firmware",
        "usb3": "PASS" if (rep["device"]["usb_type_descriptor"] or "").startswith("3") else "FAIL: " + C.USB2_HINT,
        "latency <= 60 ms": "measure: rs_timing.py (to host), rs_flash_latency.py (photons to host)",
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial")
    ap.add_argument("--no-stream", action="store_true", help="skip the short streaming check")
    a = ap.parse_args()

    C.need_rs()
    devs = list(rs.context().query_devices())
    print(f"librealsense {getattr(rs, '__version__', '?')}: {len(devs)} device(s)")
    dev = C.find_device(a.serial)
    summ = C.device_summary(dev)
    for k, v in summ.items():
        print(f"  {k:30s} {v}")
    if not C.usb3(dev):
        print("WARNING: " + C.USB2_HINT)
    fw, rec = summ.get("firmware_version"), summ.get("recommended_firmware_version")
    if fw and rec and tuple(map(int, fw.split("."))) < tuple(map(int, rec.split("."))):
        print(f"NOTE: firmware {fw} is older than the recommended {rec} (update with the RealSense Viewer, "
              "attended; README.md)")

    rep = dict(device=summ, datasheet=C.D435_DATASHEET, temperatures=C.temperatures(dev),
               sensors={}, geometry=stereo_geometry(dev))
    for s in dev.query_sensors():
        sr = sensor_report(s)
        rep["sensors"][sr["name"]] = sr
        print(f"\n{sr['name']}:")
        for k, v in sr["modes"].items():
            fast = sorted({(m[0], m[1], m[2]) for m in v if m[2] >= 60}, key=lambda m: (m[0] * m[1], m[2]))
            print(f"  {k:10s} {len(v)} modes; >= 60 fps: " + ", ".join(f"{w}x{h}@{f}" for w, h, f in fast))
        for k in ("inter_cam_sync_mode", "emitter_enabled", "exposure", "laser_power"):
            if k in sr["options"]:
                o = sr["options"][k]
                print(f"  {k:22s} = {o.get('value')} (range {o.get('min')}..{o.get('max')})")
    g = rep["geometry"]
    if "ir1_to_ir2" in g:
        print(f"\nIR baseline {g['ir1_to_ir2']['baseline_mm']} mm (datasheet 50)")
    for k in ("infrared1_848x480", "infrared1_1280x800", "depth_848x480", "color_640x480"):
        if k in g:
            print(f"  {k:20s} fx {g[k]['fx']:.1f} px, FOV {g[k]['fov_deg'][0]} x {g[k]['fov_deg'][1]} deg")

    if not a.no_stream:
        rep["timestamps"] = timestamp_check(dev)
        print("\ntimestamps:")
        for k, v in rep["timestamps"].items():
            print(f"  {k:6s} {v.get('domain', v.get('error'))}; {len(v.get('metadata', []))} metadata fields")
        if any(v.get("domain") == "system_time" for v in rep["timestamps"].values()):
            print("WARNING: frames carry host arrival times, not the camera's clock: metadata is off. Run "
                  "enable_metadata.ps1 as administrator, then replug the camera (README.md). Latency can't be "
                  "measured without it.")

    rep["requirements"] = requirement_checks(rep)
    print("\nagainst design.md's camera requirements:")
    for k, v in rep["requirements"].items():
        print(f"  {k:36s} {v}")
    C.save_result("info", rep, dev)


if __name__ == "__main__":
    main()
