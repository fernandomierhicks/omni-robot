"""Does this computer (the Pi, rpi.md) see the D435, and what does it deliver? One screen (README.md).

Linux only. Checks, in order: the USB device and its link speed from sysfs (works without librealsense),
the camera through librealsense (firmware, USB descriptor, hardware timestamps), the Pi's own health
(throttling, temperature, load), then streams the catch-relevant modes for --seconds each with every
frame's pixels read into numpy (rs_timing.py's --touch) and prints fps / lost / jitter / latency / CPU
against the nominal rate. Writes results/pi_check_<serial>_<time>.json.

    python rs_pi_check.py                  # 10 s per mode
    python rs_pi_check.py --seconds 60     # a sustained run: drops and throttling show up here
    python rs_pi_check.py --mode stereo:848x480@90 --mode color:640x480@60
"""
from __future__ import annotations

import argparse
import glob
import pathlib
import subprocess

import rs_common as C
import rs_timing as T

# What the robot would use first: the IR pair (the catch sensor family), depth, colour, and the 300 fps strip
CHECK_MODES = [
    "stereo:848x480@90",
    "depth:848x480@90",
    "stereo:640x480@90",
    "ir:848x100@300",
    "color:640x480@60",
    "color:1280x720@30",
]


def read(path: str) -> str | None:
    try:
        return pathlib.Path(path).read_text().strip()
    except OSError:
        return None


def usb_devices() -> list[dict]:
    """Intel (8086) devices on the USB bus from sysfs: speed in Mb/s (5000 = USB 3, 480 = USB 2)."""
    out = []
    for d in glob.glob("/sys/bus/usb/devices/*"):
        if read(d + "/idVendor") == "8086":
            out.append(dict(path=pathlib.Path(d).name, product_id=read(d + "/idProduct"),
                            name=read(d + "/product"), usb_serial=read(d + "/serial"),
                            speed_mbps=read(d + "/speed"), usb_version=read(d + "/version"),
                            max_power=read(d + "/bMaxPower")))
    return out


def vcgencmd(*args) -> str | None:
    try:
        return subprocess.run(["vcgencmd", *args], capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def host_health() -> dict:
    thr = vcgencmd("get_throttled")
    bits = int(thr.split("=")[1], 16) if thr and "=" in thr else None
    temp = read("/sys/class/thermal/thermal_zone0/temp")
    return dict(throttled=thr, throttled_now=None if bits is None else bool(bits & 0xF),
                undervoltage_since_boot=None if bits is None else bool(bits & 0x10000),
                cpu_temp_c=float(temp) / 1000 if temp else None, loadavg=read("/proc/loadavg"),
                governor=read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
                cpu_mhz=(float(read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") or 0) / 1000) or None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial")
    ap.add_argument("--mode", action="append")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--exposure-us", type=float, default=None)
    a = ap.parse_args()

    rep = dict(usb=usb_devices(), host_before=host_health())
    print("== USB (sysfs)")
    for u in rep["usb"]:
        ok = (u["speed_mbps"] or "0").isdigit() and int(u["speed_mbps"]) >= 5000
        print(f"  {u['name']}  id 8086:{u['product_id']}  {u['speed_mbps']} Mb/s  USB {u['usb_version']}  "
              f"{'OK (USB 3)' if ok else 'SLOW: not on a USB 3 link'}")
    if not rep["usb"]:
        print("  no Intel device on the bus: cable, or the port (the blue ones are USB 3); rs_pi.sh usb for dmesg")
        C.save_result("pi_check", rep)
        raise SystemExit(1)

    dev = C.find_device(a.serial)
    rep["device"] = C.device_summary(dev)
    print("== librealsense")
    for k in ("name", "serial_number", "firmware_version", "usb_type_descriptor", "asic_serial_number", "librealsense"):
        print(f"  {k:22s} {rep['device'][k]}")
    print("== host (before)")
    h = rep["host_before"]
    print(f"  CPU {h['cpu_temp_c']} C, {h['cpu_mhz']:.0f} MHz, governor {h['governor']}, load {h['loadavg']}, "
          f"throttled {h['throttled']}")

    results = []
    for m in a.mode or CHECK_MODES:
        try:
            r = T.run_mode(dev, m, a.seconds, 2.0, a.exposure_us, None, touch=True)
        except RuntimeError as e:
            print(f"{m}: {e}")
            r = dict(mode=m, error=str(e))
        T.print_mode(r)
        results.append(r)
    rep["modes"] = results
    rep["host_after"] = host_health()
    rep["temperatures"] = C.temperatures(dev)

    print("\n== verdict")
    h = rep["host_after"]
    print(f"  host after: CPU {h['cpu_temp_c']} C, throttled {h['throttled']}"
          + (" (UNDERVOLTAGE: use a 5 V 3 A supply; the D435 draws up to ~0.7 A on top)"
             if h["undervoltage_since_boot"] else ""))
    t = rep["temperatures"]       # listed but not readable over the Linux V4L backend (2.59.1): None
    print("  camera temperatures: " + (f"ASIC {t['asic_temperature']} C, projector {t['projector_temperature']} C"
                                       if t.get("asic_temperature") is not None else "n/a on this backend"))
    rows = []
    for r in results:
        for st, s in r.get("streams", {}).items():
            fps_nom = next((f for k, w, hh, f in C.parse_mode(r["mode"])
                            if {"depth": "depth", "color": "color"}.get(k, "infrared") ==
                            {"depth": "depth", "color": "color"}.get(st.rstrip("12"), "infrared")), None)
            iv = s["intervals_hw"] or s["intervals_host"]
            rows.append((r["mode"], st, fps_nom, iv.get("fps"), s["lost_by_counter"],
                         s.get("latency_mid_exposure_to_pixels_ms", {}).get("p95"), r["cpu_cores"]))
    bad = [x for x in rows if x[3] is None or x[2] is None or x[3] < 0.97 * x[2] or x[4] > 0]
    print(f"  {len(rows)} streams measured; {len(rows) - len(bad)} deliver the nominal rate with no lost frames")
    for x in bad:
        print(f"  CHECK {x[0]} {x[1]}: {x[3]} fps of {x[2]}, lost {x[4]}")
    C.save_result("pi_check", rep, dev)


if __name__ == "__main__":
    main()
