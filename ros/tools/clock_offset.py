"""Clock offset between two computers, NTP-style over UDP (design.md 8, phase 1C step 2: clock:=wall needs the PC and the
robot computer within a few ms; rpi.md).

  on the robot computer:   python3 clock_offset.py serve [--port 9871] [--seconds 120]
  on the PC (WSL):         python3 clock_offset.py probe omni-cam.local [--port 9871] [--n 200]

Each probe sends t0, the server answers with its own time, the client notes t1: offset = t_server - (t0 + t1) / 2, exact
if the two legs take equally long. The sample with the shortest round trip is the best (least queueing); the spread of
the best tenth says how good that is. "offset" is server minus client: add it to the client's clock to get the server's.
"""
from __future__ import annotations

import argparse
import json
import socket
import statistics as st
import struct
import time


def serve(port: int, seconds: float):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", port))
    s.settimeout(1.0)
    t_end = time.time() + seconds
    while time.time() < t_end:
        try:
            data, addr = s.recvfrom(64)
        except socket.timeout:
            continue
        s.sendto(data + struct.pack("<d", time.time()), addr)


def probe(host: str, port: int, n: int):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", port + 1))       # inside WSL's Hyper-V firewall rule (UDP 7400-32999 from the LAN), not an ephemeral port
    s.settimeout(0.3)
    rows = []
    for _ in range(n):
        t0 = time.time()
        s.sendto(struct.pack("<d", t0), (host, port))
        try:
            data, _ = s.recvfrom(64)
        except socket.timeout:
            continue
        t1 = time.time()
        ts = struct.unpack("<d", data[8:16])[0]
        rows.append((t1 - t0, ts - (t0 + t1) / 2))
        time.sleep(0.01)
    if not rows:
        raise SystemExit("no answers: is `serve` running, and the UDP port open?")
    rows.sort()
    best = rows[:max(3, len(rows) // 10)]
    offs = [o for _, o in best]
    return dict(host=host, answers=len(rows), sent=n, rtt_ms_min=1e3 * rows[0][0], rtt_ms_median=1e3 * st.median(r for r, _ in rows),
                offset_ms=1e3 * st.median(offs), offset_spread_ms=1e3 * (max(offs) - min(offs)),
                meaning="server clock minus this computer's, from the shortest round trips")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mode", choices=("serve", "probe"))
    ap.add_argument("host", nargs="?")
    ap.add_argument("--port", type=int, default=9871)
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--n", type=int, default=200)
    a = ap.parse_args()
    if a.mode == "serve":
        serve(a.port, a.seconds)
    else:
        print(json.dumps(probe(a.host, a.port, a.n), indent=1))


if __name__ == "__main__":
    main()
