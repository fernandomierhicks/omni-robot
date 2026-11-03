"""ROS 2 round trip between the PC (WSL) and the camera head: is DDS getting through, and how long does it take.

On the Pi:  python3 pi_camera/dds_ping.py echo            (pi.sh run ...: answers /dds/ping on /dds/pong)
In WSL:     python3 dds_ping.py ping --count 200          (stamps each ping with its clock, times the pong)

Both on the same ROS_DOMAIN_ID (default here 42, away from the sim's 0). Round trip / 2 is the one-way transport a
/catch/track message would add (small messages, the same QoS; over Wi-Fi here, Ethernet on the robot). Results go
to results/dds_ping_*.json.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

HERE = Path(__file__).resolve().parent


def ns_now():
    return time.monotonic_ns()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mode", choices=("echo", "ping"))
    ap.add_argument("--count", type=int, default=200)
    ap.add_argument("--rate", type=float, default=50.0, help="pings a second")
    ap.add_argument("--seconds", type=float, default=0, help="echo: stop after this long (0: never)")
    ap.add_argument("--domain", type=int, default=42)
    ap.add_argument("--out", default=str(HERE / "results"))
    a = ap.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(a.domain)
    rclpy.init()
    n = Node(f"dds_{a.mode}")

    if a.mode == "echo":
        pub = n.create_publisher(Header, "/dds/pong", 50)
        n.create_subscription(Header, "/dds/ping", pub.publish, 50)
        print(f"echoing /dds/ping -> /dds/pong on domain {a.domain} ({socket.gethostname()})", flush=True)
        t_end = time.monotonic() + a.seconds if a.seconds else None
        while rclpy.ok() and (t_end is None or time.monotonic() < t_end):
            rclpy.spin_once(n, timeout_sec=0.2)
        return

    rtt, sent = {}, {}
    pub = n.create_publisher(Header, "/dds/ping", 50)

    def on_pong(m):
        k = int(m.frame_id)
        if k in sent and k not in rtt:
            rtt[k] = (ns_now() - sent[k]) / 1e6

    n.create_subscription(Header, "/dds/pong", on_pong, 50)
    t0 = time.monotonic()                                        # wait for the echo to be matched
    while pub.get_subscription_count() == 0 or n.count_publishers("/dds/pong") == 0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.monotonic() - t0 > 20:
            raise SystemExit("no echo found in 20 s (is dds_ping.py echo running on the Pi, same domain?)")
    t_disc = time.monotonic() - t0
    for i in range(a.count):
        m = Header()
        m.frame_id = str(i)
        sent[i] = ns_now()
        pub.publish(m)
        t_next = time.monotonic() + 1.0 / a.rate
        while time.monotonic() < t_next:
            rclpy.spin_once(n, timeout_sec=max(0.0, t_next - time.monotonic()))
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end and len(rtt) < a.count:
        rclpy.spin_once(n, timeout_sec=0.05)
    v = sorted(rtt.values())
    q = lambda p: v[min(len(v) - 1, int(p / 100 * len(v)))] if v else None
    res = {"test": "dds_ping", "time": time.strftime("%Y-%m-%dT%H:%M:%S"), "host": socket.gethostname(),
           "domain": a.domain, "discovery_s": round(t_disc, 2), "sent": a.count, "answered": len(v),
           "rtt_ms": {"median": q(50), "p95": q(95), "max": v[-1] if v else None},
           "notes": "round trip PC <-> Pi, std_msgs/Header, default QoS; one way ~ rtt / 2"}
    print(json.dumps(res, indent=1))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    f = out / f"dds_ping_{time.strftime('%Y%m%d_%H%M%S')}.json"
    f.write_text(json.dumps(res, indent=1))
    print(f"-> {f}")


if __name__ == "__main__":
    main()
