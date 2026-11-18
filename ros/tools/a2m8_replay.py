"""a2m8_replay: a stand-in RPLIDAR A2M8 on a Linux pseudo-terminal, for running omni_cpp's rplidar_node without the
lidar. It answers the protocol as the bench's unit did (GET_INFO: A2M8, firmware 1.26, hardware 5; health good; its four
scan modes; SET_PWM; STOP) and, when a scan starts, replays a bench recording's bytes at their recorded times
(characterization/lidar: lidar_capture.py's run_*.npz, or omni_cpp/test/data/a2m8_<mode>.npz).

    python3 ros/tools/a2m8_replay.py [--data FILE.npz] [--link /tmp/a2m8]      # prints the tty; ^C ends
    ros2 run omni_cpp rplidar_node --ros-args -p port:=/tmp/a2m8

The recording plays once per scan (a scan that outlasts it gets no more bytes: the node's "no data" path). Its mode is
the one the recording was made in; asking for another scan mode gets no answer.
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import threading
import time
import tty
from pathlib import Path

import numpy as np

STOP, RESET, SCAN, EXPRESS_SCAN, GET_INFO, GET_HEALTH, GET_SAMPLERATE, GET_CONF, SET_PWM = (
    0x25, 0x40, 0x20, 0x82, 0x50, 0x52, 0x59, 0x84, 0xF0)
MODES = (("Standard", 500, 0x81), ("Express", 250, 0x82), ("Boost", 125, 0x84), ("Stability", 250, 0x84))
TYPICAL = 3
SIZES = {0x81: 5, 0x82: 84, 0x84: 132}
HERE = Path(__file__).resolve().parent
DEFAULT = HERE.parent / "src" / "omni_cpp" / "test" / "data" / "a2m8_stability.npz"


def desc(size: int, typ: int, continuous: bool = False) -> bytes:
    return b"\xA5\x5A" + struct.pack("<I", size | (1 << 30 if continuous else 0)) + bytes((typ,))


def load(path: Path):
    """(chunk times from 0 s, chunk sizes, bytes, answer type)."""
    d = np.load(path)
    t = d["t_us"] * 1e-6 if "t_us" in d.files else d["t"] - d["t"][0]
    if "ans_type" in d.files:
        ans = int(d["ans_type"])
    else:                                                    # a bench run: its mode is in its name
        name = path.stem.split("_")[2]
        ans = next(a for m, _, a in MODES if m == name)
    return np.asarray(t, float), d["n"].astype(int), d["data"].tobytes(), ans


class Device:
    def __init__(self, fd: int, rec):
        self.fd, self.rec = fd, rec
        self.lock = threading.Lock()
        self.scan_id = 0                                     # bumps at each start / stop: the streamer checks it
        self.pwm = 0

    def write(self, b: bytes):
        with self.lock:
            os.write(self.fd, b)

    def stream(self, my_id: int):
        t, n, data, _ = self.rec
        off = np.concatenate([[0], np.cumsum(n)])
        t_start = time.monotonic() + 0.05
        for i in range(len(n)):
            if self.scan_id != my_id:
                return
            wait = t_start + t[i] - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            if self.scan_id != my_id:
                return
            self.write(data[off[i]:off[i + 1]])
        print("replay: the recording ended", flush=True)

    def handle(self, cmd: int, payload: bytes):
        if cmd in (STOP, RESET):
            self.scan_id += 1
        elif cmd == SET_PWM:
            self.pwm = struct.unpack("<H", payload[:2])[0]
        elif cmd == GET_INFO:
            self.write(desc(20, 0x04) + bytes((0x28, 26, 1, 5)) + bytes.fromhex("C79D9AF2C1EA9FC2A2EB92F1544E3C00"))
        elif cmd == GET_HEALTH:
            self.write(desc(3, 0x06) + b"\0\0\0")
        elif cmd == GET_SAMPLERATE:
            self.write(desc(4, 0x15) + struct.pack("<HH", 500, 250))
        elif cmd == GET_CONF:
            key = struct.unpack("<I", payload[:4])[0]
            m = struct.unpack("<H", payload[4:6])[0] if len(payload) >= 6 else 0
            name, us, ans = MODES[min(m, 3)]
            body = {0x70: struct.pack("<H", 4), 0x7C: struct.pack("<H", TYPICAL), 0x71: struct.pack("<I", us * 256),
                    0x74: struct.pack("<I", 12 * 256), 0x75: bytes((ans,)), 0x7F: name.encode() + b"\0"}.get(key)
            if body is not None:
                self.write(desc(4 + len(body), 0x20) + struct.pack("<I", key) + body)
        elif cmd in (SCAN, EXPRESS_SCAN):
            ans = 0x81 if cmd == SCAN else MODES[payload[0] if payload else 1][2]
            if cmd == EXPRESS_SCAN and payload and payload[0] < 4:
                print(f"replay: scan in {MODES[payload[0]][0]} at PWM {self.pwm}", flush=True)
            if ans != self.rec[3]:
                print(f"replay: no recording in answer type 0x{ans:02X} (it's 0x{self.rec[3]:02X})", flush=True)
                return
            self.scan_id += 1
            self.write(desc(SIZES[ans], ans, True))
            threading.Thread(target=self.stream, args=(self.scan_id,), daemon=True).start()

    def serve(self):
        buf = b""
        while True:
            r, _, _ = select.select([self.fd], [], [], 0.5)
            if not r:
                continue
            try:
                buf += os.read(self.fd, 256)
            except OSError:                                  # nobody has the tty open: wait for the next one
                time.sleep(0.1)
                continue
            while len(buf) >= 2:
                if buf[0] != 0xA5:
                    buf = buf[1:]
                    continue
                cmd = buf[1]
                if cmd & 0x80:                               # a payload: len, payload, checksum
                    if len(buf) < 3 or len(buf) < 4 + buf[2]:
                        break
                    payload, buf = buf[3:3 + buf[2]], buf[4 + buf[2]:]
                else:
                    payload, buf = b"", buf[2:]
                self.handle(cmd, payload)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data", type=Path, default=DEFAULT)
    ap.add_argument("--link", default="", help="also make this path a symlink to the tty")
    a = ap.parse_args()
    rec = load(a.data)
    master, slave = os.openpty()
    tty.setraw(slave)
    name = os.ttyname(slave)
    os.close(slave)                                          # the driver's open is then the only one on it
    if a.link:
        if os.path.islink(a.link):
            os.unlink(a.link)
        os.symlink(name, a.link)
    print(f"replay: an A2M8 on {name}{' (' + a.link + ')' if a.link else ''}, {len(rec[2])} bytes over "
          f"{rec[0][-1]:.1f} s in answer type 0x{rec[3]:02X}", flush=True)
    try:
        Device(master, rec).serve()
    except KeyboardInterrupt:
        pass
    finally:
        if a.link and os.path.islink(a.link):
            os.unlink(a.link)


if __name__ == "__main__":
    sys.exit(main())
