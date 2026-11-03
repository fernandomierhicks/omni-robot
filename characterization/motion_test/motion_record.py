"""Camera-motion test, recording side (Pi, D435): what moving the camera does to seeing the ball.

The ball rests in view next to a printed AprilTag; Fernando moves the camera by hand (slow pans, fast pans, shakes,
like a robot turning and rocking). Every colour frame (848x480, 60 fps) and depth (424x240, 30 fps) is kept in RAM
while the colour exposure cycles every `--block` seconds through `--exposures` (so every exposure sees the whole
range of motions; nobody has to repeat a routine per setting). The first `--settle` seconds after each switch are
marked `settling`. Afterwards the colour frames are written as JPEG (q95), depth as compressed npz, with every
frame's stamps and the exposure / gain that was set.

  ~/.venvs/cam/bin/python motion_record.py --seconds 40 [--exposures 1000,2000,4000,8000] [--gains 128,128,64,64]

Only one process can hold the camera: stop the bucket lab first (mt.sh does). Analysis: motion_analyze.py on the PC.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--countdown", type=float, default=5.0, help="seconds before recording starts")
    ap.add_argument("--exposures", default="1000,2000,4000,8000", help="colour exposures, us, cycled")
    ap.add_argument("--gains", default="128,128,64,64", help="colour gain for each exposure")
    ap.add_argument("--block", type=float, default=0.5, help="seconds per exposure before the next")
    ap.add_argument("--settle", type=float, default=0.1, help="seconds after a switch marked settling")
    ap.add_argument("--white-balance", type=float, default=4200.0)
    ap.add_argument("--out", type=Path, default=Path.home() / "omni_ws" / "motion_test")
    ap.add_argument("--name", default="motion")
    a = ap.parse_args()
    exposures = [int(x) for x in a.exposures.split(",")]
    gains = [float(x) for x in a.gains.split(",")]
    assert len(exposures) == len(gains), "one gain per exposure"

    import pyrealsense2 as rs
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.rgb8, 60)
    cfg.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
    queue = rs.frame_queue(120, keep_frames=True)
    prof = pipe.start(cfg, queue)
    dev = prof.get_device()
    colour = dev.first_color_sensor()
    depth_sensor = dev.first_depth_sensor()
    colour.set_option(rs.option.enable_auto_exposure, 0)
    colour.set_option(rs.option.enable_auto_white_balance, 0)
    colour.set_option(rs.option.white_balance, a.white_balance)
    cs, ds = prof.get_stream(rs.stream.color).as_video_stream_profile(), prof.get_stream(rs.stream.depth).as_video_stream_profile()

    def intr(p):
        i = p.get_intrinsics()
        return dict(width=i.width, height=i.height, fx=i.fx, fy=i.fy, ppx=i.ppx, ppy=i.ppy, model=str(i.model), coeffs=list(i.coeffs))
    ext = ds.get_extrinsics_to(cs)
    calib = dict(color=intr(cs), depth=intr(ds), depth_scale=depth_sensor.get_depth_scale(),
                 depth_to_color=dict(R=list(ext.rotation), t=list(ext.translation)),
                 serial=dev.get_info(rs.camera_info.serial_number), firmware=dev.get_info(rs.camera_info.firmware_version))

    def apply(k):
        colour.set_option(rs.option.exposure, exposures[k] / 100.0)          # D435 colour exposure is in 100 us units
        colour.set_option(rs.option.gain, gains[k])

    apply(1 % len(exposures))
    end = time.monotonic() + a.countdown
    while time.monotonic() < end:                                          # drain while counting down (and let AE-off settle)
        queue.poll_for_frame()
        print(f"\rrecording in {end - time.monotonic():4.1f} s", end="", flush=True)
        time.sleep(.02)
    while queue.poll_for_frame():
        pass
    print("\rRECORDING: move the camera (slow, fast, shake) with the ball and tag in view", flush=True)

    # Raw colour for 40 s is ~2.9 GB (the 4 GB Pi has ~3.3 free): JPEG-encode as they come, on two threads
    # (simplejpeg releases the GIL), ~0.1 MB a frame.
    import simplejpeg
    from concurrent.futures import ThreadPoolExecutor
    pool = ThreadPoolExecutor(2)
    encode = lambda arr: simplejpeg.encode_jpeg(arr, quality=95, colorspace="RGB")
    frames, depths = [], []
    t0 = time.monotonic()
    k, switched = 0, t0
    apply(k)
    last_c = last_d = -1
    while time.monotonic() - t0 < a.seconds:
        now = time.monotonic()
        if now - switched >= a.block:
            k, switched = (k + 1) % len(exposures), now
            apply(k)
        f = queue.wait_for_frame(2000)
        host = time.time_ns()
        fs = f.as_frameset() if f.is_frameset() else None
        for fr in (fs if fs else [f]):
            n = fr.get_frame_number()
            st = fr.get_profile().stream_type()
            if st == rs.stream.color and n != last_c:
                last_c = n
                frames.append(dict(jpeg=pool.submit(encode, np.asanyarray(fr.get_data()).copy()), number=n, t_ms=fr.get_timestamp(),
                                   domain=str(fr.get_frame_timestamp_domain()), host_ns=host,
                                   exposure_us=exposures[k], gain=gains[k], block=k,
                                   since_switch_s=now - switched, settling=now - switched < a.settle))
            elif st == rs.stream.depth and n != last_d:
                last_d = n
                depths.append(dict(arr=np.asanyarray(fr.get_data()).copy(), number=n, t_ms=fr.get_timestamp(), host_ns=host))
        print(f"\r{time.monotonic() - t0:5.1f} s  {len(frames)} colour  {len(depths)} depth  exposure {exposures[k]} us",
              end="", flush=True)
    pipe.stop()
    print()

    out = a.out / (time.strftime("%Y%m%d_%H%M%S") + "_" + a.name)
    (out / "colour").mkdir(parents=True)
    rows = []
    for i, fr in enumerate(frames):
        name = f"colour/{i:05d}.jpg"
        (out / name).write_bytes(fr.pop("jpeg").result())
        rows.append(dict(fr, file=name))
    pool.shutdown()
    (out / "colour.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    np.savez_compressed(out / "depth.npz", depth=np.stack([d.pop("arr") for d in depths]) if depths else np.zeros((0, 240, 424), np.uint16))
    (out / "depth.jsonl").write_text("".join(json.dumps(d) + "\n" for d in depths))
    nums = np.array([r["number"] for r in rows])
    lost = int(np.sum(np.clip(np.diff(nums) - 1, 0, None))) if len(nums) > 1 else 0
    meta = dict(kind="motion-test/1", calib=calib, exposures_us=exposures, gains=gains, block_s=a.block, settle_s=a.settle,
                white_balance=a.white_balance, seconds=a.seconds, colour_frames=len(rows), depth_frames=len(depths),
                colour_lost=lost, recorded=time.strftime("%Y-%m-%dT%H:%M:%S"))
    (out / "session.json").write_text(json.dumps(meta, indent=1))
    print(f"saved {out}  ({len(rows)} colour, {lost} lost, {len(depths)} depth)")


if __name__ == "__main__":
    main()
