"""Watch a running camera head (pi.sh head) for N seconds and record what it does: the capture node's /cam/left/stats
(fps, drops, sensor -> published lag), catch_perception's /catch/perception (frames processed, detections, the
per-frame processing time), /catch/track messages, and the camera_head container's CPU (from /proc, all its threads).

It doesn't subscribe to the images (a second, cross-process subscriber would add its own copy and change what is
measured). Sensor -> detection is about lag + the intra-process handoff + proc_ms; the handoff isn't measured here.

On the Pi:  bash characterization/pi_camera/pi.sh probe 10      (results/head_probe_*.json)
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from omni_interfaces.msg import BallTrack

HERE = Path(__file__).resolve().parent
TICK = os.sysconf("SC_CLK_TCK")


def container_pid(name="camera_head"):
    for d in Path("/proc").iterdir():
        if d.name.isdigit():
            try:
                cmd = (d / "cmdline").read_bytes().split(b"\0")
            except OSError:
                continue
            if any(c.endswith(b"component_container") for c in cmd[:1]) and f"__node:={name}".encode() in cmd:
                return int(d.name)
    return None


def cpu_s(pid):
    f = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    return (int(f[11]) + int(f[12])) / TICK                      # utime + stime


class Probe(Node):
    def __init__(self):
        super().__init__("head_probe")
        self.cam, self.perc, self.tracks = [], [], 0
        self.create_subscription(String, "/cam/left/stats", lambda m: self.cam.append(json.loads(m.data)), 10)
        self.create_subscription(String, "/catch/perception", lambda m: self.perc.append(json.loads(m.data)), 10)
        self.create_subscription(BallTrack, "/catch/track", self.on_track, 20)

    def on_track(self, m):
        self.tracks += m.tracking


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--out", default=str(HERE / "results"))
    a = ap.parse_args()
    pid = container_pid()
    if pid is None:
        raise SystemExit("no camera_head container running (start it with pi.sh head)")
    rclpy.init()
    n = Probe()
    c0, t0 = cpu_s(pid), time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.1)
    cores = (cpu_s(pid) - c0) / (time.monotonic() - t0)
    cam, perc = n.cam[1:], n.perc[1:]                              # the first report may cover a partial second
    n.destroy_node()
    rclpy.shutdown()
    if not cam:
        raise SystemExit("no /cam/left/stats in the window")

    def med(xs, k):
        v = sorted(x[k] for x in xs if x.get(k) is not None)
        return v[len(v) // 2] if v else None

    res = {
        "test": "head_probe", "time": time.strftime("%Y-%m-%dT%H:%M:%S"), "host": os.uname().nodename,
        "seconds": a.seconds, "container_cores": round(cores, 3),
        "governor": Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").read_text().strip(),
        "capture": {"fps_median": med(cam, "fps"), "dropped": sum(x["dropped_1s"] for x in cam),
                    "frames": cam[-1]["frames"] - cam[0]["frames"],
                    "lag_ms_median": med(cam, "lag_ms_median"), "lag_ms_95_worst": max(x["lag_ms_95"] for x in cam),
                    "exposure_us": cam[-1]["exposure_us"]},
        "perception": {"frames": perc[-1]["pairs"] - perc[0]["pairs"] if len(perc) > 1 else None,
                       "proc_ms_median": med(perc, "proc_ms_median"), "proc_ms_95": med(perc, "proc_ms_95"),
                       "detections": perc[-1]["detections"] - perc[0]["detections"] if len(perc) > 1 else None,
                       "pending_max": max((x["pending"] for x in perc), default=None)} if perc else None,
        "track_msgs_tracking": n.tracks,
        "notes": "lag = SensorTimestamp (start of readout) -> published; excludes exposure; handoff not measured",
    }
    print(json.dumps(res, indent=1))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    f = out / f"head_probe_{time.strftime('%Y%m%d_%H%M%S')}.json"
    f.write_text(json.dumps(res, indent=1))
    print(f"-> {f}")


if __name__ == "__main__":
    main()
