"""Step 3: photons-to-host latency with a flashing screen (README.md).

Point the camera at this PC's monitor (20-50 cm, the window filling most of the view) and run it. A
full-screen window flips between black and white at random intervals; the camera's frames are reduced
to their mean brightness. Three numbers per flip:

  display = first bright (dark) frame's mid-exposure - the flip command   (the monitor's own lag + vsync)
  camera  = that frame reaching Python - its mid-exposure                  (exposure/2 + readout + USB + driver)
  total   = that frame reaching Python - the flip command

`camera` is the part the robot pays (design.md 4.9: capture -> detection <= 60 ms, 50 ms assumed in the
sims), and it checks rs_timing.py's timestamp chain from outside: if the camera's clock were wrong,
`display` would come out negative or jump around. With hardware the Teensy pulses an LED instead
(design.md 8, phase 1C); the screen stands in until then.

    python rs_flash_latency.py --mode ir:848x480@90 --flips 60
    python rs_flash_latency.py --mode color:640x480@60
"""
from __future__ import annotations

import argparse
import random
import threading
import time

import cv2
import numpy as np

import rs_common as C
from rs_common import rs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial")
    ap.add_argument("--mode", default="ir:848x480@90", help="one stream: ir:..., color:...")
    ap.add_argument("--flips", type=int, default=60)
    ap.add_argument("--min-ms", type=float, default=250, help="shortest time between flips")
    ap.add_argument("--max-ms", type=float, default=450)
    ap.add_argument("--exposure-us", type=float, default=2000, help="fixed exposure (auto-exposure would chase the flashes)")
    ap.add_argument("--roi", type=float, default=0.5, help="central fraction of the image averaged")
    ap.add_argument("--monitor-x", type=int, default=0, help="x of the monitor to fill (multi-monitor)")
    a = ap.parse_args()

    dev = C.find_device(a.serial)
    mode = C.parse_mode(a.mode)
    if len(mode) != 1 or mode[0][0] not in ("ir", "color"):
        raise SystemExit("--mode takes one ir: or color: stream")
    if not C.mode_supported(dev, mode):
        raise SystemExit(f"{a.mode} is not offered (USB link {C.info(dev, 'usb_type_descriptor')})")
    kind, w, h, fps = mode[0]

    samples, lock = [], threading.Lock()
    y0, y1 = int(h * (1 - a.roi) / 2), int(h * (1 + a.roi) / 2)
    x0, x1 = int(w * (1 - a.roi) / 2), int(w * (1 + a.roi) / 2)

    def on_frame(f):
        t_host = C.host_ms()
        img = np.asanyarray(f.get_data())
        lvl = float(img[y0:y1, x0:x1].mean())
        ft = C.frame_times(f, t_host)
        with lock:
            samples.append((ft["t_mid_ms"], t_host, lvl, ft["domain"], ft["exposure_us"]))

    cfg = rs.config()
    cfg.enable_device(C.info(dev, "serial_number"))
    C.enable_streams(cfg, mode)
    C.configure_stereo(dev, emitter=False, exposure_us=a.exposure_us if kind == "ir" else None)
    if kind == "color":
        cs = C.color_sensor(dev)
        C.set_opt(cs, "enable_auto_exposure", 0)
        C.set_opt(cs, "exposure", max(1, round(a.exposure_us / 100)))
        C.set_opt(cs, "enable_auto_white_balance", 0)

    win = "flash (point the camera here; Esc stops)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.moveWindow(win, a.monitor_x, 0)
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    black = np.zeros((360, 640), np.uint8)
    white = np.full((360, 640), 255, np.uint8)
    cv2.imshow(win, black)
    cv2.waitKey(500)

    pipe = rs.pipeline()
    pipe.start(cfg, on_frame)
    print(f"aim the camera at the screen; {a.flips} flips at {a.mode} ...")
    cv2.waitKey(1500)
    commands = []
    state = False
    try:
        for i in range(a.flips):
            state = not state
            cv2.imshow(win, white if state else black)
            cv2.waitKey(1)                      # the draw is queued here; the flip lands on a later vsync
            commands.append((C.host_ms(), 1 if state else -1))
            wait = random.uniform(a.min_ms, a.max_ms)
            if (cv2.waitKey(max(1, int(wait))) & 0xFF) == 27:
                break
    finally:
        time.sleep(0.3)
        pipe.stop()
        cv2.destroyAllWindows()

    with lock:
        s = list(samples)
    dom = s[0][3] if s else None
    if dom != "global_time":
        print("no hardware timestamps (metadata off): only `total` (to host arrival) can be measured; run "
              "enable_metadata.ps1 for the split")
    t_mid = np.array([x[0] if x[0] is not None else np.nan for x in s])
    t_rx = np.array([x[1] for x in s])
    lvl = np.array([x[2] for x in s])
    t_seen = t_mid if dom == "global_time" else t_rx
    edges = C.flash_edges(t_seen, lvl)
    if len(edges) < len(commands) // 2:
        print(f"only {len(edges)} edges seen for {len(commands)} flips: is the screen in view and the exposure "
              f"right? (brightness {lvl.min():.0f}..{lvl.max():.0f})")
    lags = C.match_flashes(commands, edges)
    # each matched edge -> its frame's receipt time
    disp, cam, tot = [], [], []
    for (tc, _), lag in zip(commands, lags):
        if lag is None:
            continue
        te = tc + lag
        k = int(np.argmin(np.abs(t_seen - te)))
        if dom == "global_time":
            disp.append(t_mid[k] - tc)
            cam.append(t_rx[k] - t_mid[k])
        tot.append(t_rx[k] - tc)
    res = dict(mode=a.mode, flips=len(commands), matched=len(tot), exposure_us=a.exposure_us, domain=dom,
               display_ms=C.stats(disp), camera_ms=C.stats(cam), total_ms=C.stats(tot),
               frame_period_ms=1000.0 / fps, budget_ms=C.REQ["latency_max_ms"],
               brightness=dict(min=float(lvl.min()) if lvl.size else None, max=float(lvl.max()) if lvl.size else None))
    print(f"\n{len(tot)} of {len(commands)} flips matched")
    for k in ("display_ms", "camera_ms", "total_ms"):
        st = res[k]
        if st.get("n"):
            print(f"  {k[:-3]:8s} median {st['median']:6.1f}  p95 {st['p95']:6.1f}  min {st['min']:6.1f}  "
                  f"max {st['max']:6.1f} ms")
    if res["camera_ms"].get("n"):
        print(f"camera part, p95 {res['camera_ms']['p95']:.1f} ms against the {C.REQ['latency_max_ms']:.0f} ms "
              "capture -> detection budget (the frame period quantizes `display` by "
              f"{1000.0 / fps:.1f} ms)")
    if res["display_ms"].get("n") and res["display_ms"]["min"] < -2:
        print("WARNING: negative display lag: the hardware timestamps are off against the host clock")
    C.save_result("flash", res, dev)


if __name__ == "__main__":
    main()
