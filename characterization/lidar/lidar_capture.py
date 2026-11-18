"""Record the RPLIDAR's raw byte stream with each chunk's arrival time, for the lidar bench (README.md).

Runs on the Pi (~/.venvs/cam/bin/python) next to a copy of the robot's own driver, omni_console/rplidar.py: the
device is opened, spun up and scanned exactly as the console does it; only the reader thread is replaced by one that
keeps every chunk of bytes as it arrived (host CLOCK_MONOTONIC) instead of only the newest turn. Decoding happens
offline (lidar_timing.py), so a capture can be re-analysed when the driver or the questions change.

    python lidar_capture.py --session S [--plan MODE:PWM:SECONDS,...] [--gap 5] [--port P]

Each run starts from a stopped motor (the gap lets it coast down), so its first seconds show the spin-up. Writes
<out>/<session>/run_NN_<mode>_<pwm>.npz (t: arrival times, n: chunk sizes, data: the bytes) and session.json
(device, modes, the plan, per-run start times and the Pi's load and temperature).
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rplidar  # noqa: E402  (the robot's driver, copied next to this file by lidar.sh)

# 1: cold start, the device's typical mode at the default PWM for 5 min (rate drift while it warms; also the first
# data for the warm-up test); 2: every mode at the default PWM; 3: a PWM sweep in the typical mode.
DEFAULT_PLAN = ("typical:660:300,Standard:660:30,Express:660:30,Boost:660:30,Stability:660:30,"
                "typical:300:20,typical:400:20,typical:500:20,typical:800:20,typical:1023:20")


class Recorder(rplidar.RPLidar):
    """The driver with a reader that keeps the stream: (arrival time, bytes) per read."""

    def _run(self, parser):
        self.ans_type = next(k for k, v in rplidar.PARSERS.items() if isinstance(parser, v))
        self.t_scan = time.monotonic()
        self.chunks: list[tuple[float, bytes]] = []
        while not self.stop_ev.is_set():
            try:
                n = getattr(self.ser, "in_waiting", 0)
                data = self.ser.read(min(max(n, 1), 8192))
            except Exception as e:
                self.error = self.dead = f"{type(e).__name__}: {e}"
                return
            if data:
                self.chunks.append((time.monotonic(), data))
                self.bytes_in += len(data)


def pi_state() -> dict:
    out = {}
    try:
        out["loadavg"] = [float(x) for x in open("/proc/loadavg").read().split()[:3]]
    except OSError:
        pass
    try:
        out["cpu_temp_c"] = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
    except (OSError, ValueError):
        pass
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--session", required=True)
    ap.add_argument("--plan", default=DEFAULT_PLAN, help="MODE:PWM:SECONDS,... (MODE 'typical' = the device's own)")
    ap.add_argument("--gap", type=float, default=5.0, help="s with the motor off before each run")
    ap.add_argument("--port", default=None)
    ap.add_argument("--out", default=os.path.expanduser("~/lidar_data"))
    a = ap.parse_args()

    port = a.port or rplidar.find_port()
    if not port:
        sys.exit("no lidar port")
    others = rplidar.holders(port)
    if others:
        sys.exit(f"{port} is in use by {', '.join(others)} (the console's Lidar page? camera_snap's self-test?)")
    out = os.path.join(a.out, a.session)
    os.makedirs(out, exist_ok=True)

    lid = Recorder(port)
    typical = lid.mode(None)["name"]
    meta = dict(session=a.session, host=socket.gethostname(), port=lid.path, baud=lid.baud, info=lid.info,
                health=lid.health, rates=lid.rates, modes=lid.modes, typical=typical, plan=a.plan, gap_s=a.gap,
                t_wall=time.time(), t_mono=time.monotonic(), pi=pi_state(), runs=[])
    print(f"{lid.info['name']} fw {lid.info['firmware']} hw {lid.info['hardware']} on {lid.path} at {lid.baud}; "
          f"health {lid.health['status']}; typical {typical}; modes "
          + ", ".join(f"{m['name']} {m['rate']}/s" for m in lid.modes), flush=True)
    try:
        for i, item in enumerate(a.plan.split(",")):
            mode, pwm, secs = item.split(":")
            mode = typical if mode == "typical" else mode
            pwm, secs = int(pwm), float(secs)
            lid.stop()
            time.sleep(a.gap)
            pre = pi_state()
            t_cmd = time.monotonic()
            try:
                lid.start(mode, pwm)                      # the console's own start: 1 s soft ramp, then the scan
            except rplidar.LidarError as e:
                print(f"run {i} {mode} {pwm}: skipped ({e})", flush=True)
                meta["runs"].append(dict(name=f"run_{i:02d}_{mode}_{pwm}", mode=mode, pwm=pwm, skipped=str(e)))
                continue
            time.sleep(secs)
            chunks, t_scan, ans = lid.chunks, lid.t_scan, lid.ans_type
            err, bad_dev = lid.error, lid.dead
            lid.stop()
            name = f"run_{i:02d}_{mode}_{pwm}"
            ts = np.array([c[0] for c in chunks])
            ns = np.array([len(c[1]) for c in chunks], dtype=np.int32)
            data = np.frombuffer(b"".join(c[1] for c in chunks), dtype=np.uint8)
            np.savez_compressed(os.path.join(out, name + ".npz"), t=ts, n=ns, data=data)
            m = lid.mode(mode)
            run = dict(name=name, mode=mode, mode_id=m["id"], pwm=pwm, seconds=secs, ans_type=ans,
                       us_per_sample=m["us"], t_cmd=t_cmd, t_motor=lid.motor_t, t_scan=t_scan,
                       bytes=int(data.size), chunks=int(ts.size), error=err, dead=bad_dev, pi_before=pre,
                       pi_after=pi_state())
            meta["runs"].append(run)
            print(f"{name}: {data.size} bytes in {ts.size} chunks, {data.size / secs:.0f} B/s"
                  + (f", error {err}" if err else ""), flush=True)
            with open(os.path.join(out, "session.json"), "w") as f:
                json.dump(meta, f, indent=1)
            if bad_dev:
                break
    finally:
        lid.close()
    print(f"done -> {out}")


if __name__ == "__main__":
    main()
