"""Serialized ROS 2 messages (CDR, as rmw writes them), read field by field, ROS-free.

The console reads a few high-rate topics as raw bytes (rclpy raw=True) and decodes only the message it needs, when it
needs it: building Python message objects for every /tf or /catch/track message cost the Jetson more than everything
else the console does. web/js/cdr.js is the page's twin (the map, the scan and the MCU samples come to it raw).
"""

from __future__ import annotations

import struct


class CdrReader:
    """XCDR1 little or big endian: alignment counts from after the 4-byte encapsulation header."""

    def __init__(self, data: bytes):
        self.d = memoryview(data)
        self.e = "<" if len(data) > 1 and data[1] & 1 else ">"
        self.p = 4

    def _align(self, n: int):
        self.p += (-(self.p - 4)) % n

    def _get(self, fmt: str, size: int, n: int = 1):
        self._align(size)
        v = struct.unpack_from(f"{self.e}{n}{fmt}", self.d, self.p)
        self.p += size * n
        return v

    def u8(self) -> int:
        return self._get("B", 1)[0]

    def bool(self) -> bool:
        return bool(self.u8())

    def u32(self) -> int:
        return self._get("I", 4)[0]

    def i32(self) -> int:
        return self._get("i", 4)[0]

    def f32(self) -> float:
        return self._get("f", 4)[0]

    def f64(self) -> float:
        return self._get("d", 8)[0]

    def f64s(self, n: int) -> tuple:
        return self._get("d", 8, n)

    def string(self) -> str:
        n = self.u32()
        s = bytes(self.d[self.p:self.p + max(0, n - 1)]).decode(errors="replace")
        self.p += n
        return s

    def stamp(self) -> float:
        sec = self.i32()
        return sec + self.u32() * 1e-9

    def header(self) -> tuple:
        """std_msgs/Header -> (t, frame_id)."""
        t = self.stamp()
        return t, self.string()


class CdrWriter:
    """The reader's mirror, little endian (for tests, and anything that hands raw bytes on)."""

    def __init__(self):
        self.b = bytearray(b"\x00\x01\x00\x00")

    def _align(self, n: int):
        self.b += bytes((-(len(self.b) - 4)) % n)

    def _put(self, fmt: str, size: int, *v):
        self._align(size)
        self.b += struct.pack(f"<{len(v)}{fmt}", *v)
        return self

    def u8(self, v):
        return self._put("B", 1, int(v))

    def u32(self, v):
        return self._put("I", 4, v)

    def i32(self, v):
        return self._put("i", 4, v)

    def f32(self, *v):
        return self._put("f", 4, *v)

    def f64(self, *v):
        return self._put("d", 8, *v)

    def string(self, s: str):
        raw = s.encode() + b"\0"
        self.u32(len(raw))
        self.b += raw
        return self

    def stamp(self, t: float):
        sec = int(t // 1)
        return self.i32(sec).u32(int(round((t - sec) * 1e9)))

    def header(self, t: float, frame: str):
        return self.stamp(t).string(frame)

    def bytes(self) -> bytes:
        return bytes(self.b)


def parse_laser_scan(data: bytes):
    """sensor_msgs/LaserScan -> (t, frame, angle_min, angle_increment, range_min, range_max, ranges float32 array)."""
    import numpy as np
    r = CdrReader(data)
    t, frame = r.header()
    angle_min, _, inc, _, _, rmin, rmax = (r.f32() for _ in range(7))
    n = r.u32()
    r._align(4)
    ranges = np.frombuffer(bytes(r.d[r.p:r.p + 4 * n]), dtype=np.dtype(np.float32).newbyteorder(r.e))
    return t, frame, angle_min, inc, rmin, rmax, ranges


def parse_ball_track(data: bytes) -> dict:
    """omni_interfaces/BallTrack -> the fields the console shows (no covariance)."""
    r = CdrReader(data)
    t, _ = r.header()
    tracking, ready = r.bool(), r.bool()
    frames = r.u32()
    x = r.f64s(6)
    r.f64s(36)
    crossing = r.f64s(3)
    t_go, sigma = r.f64(), r.f64()
    return dict(t_stamp=t, tracking=tracking, ready=ready, frames=frames, x=[round(v, 3) for v in x],
                crossing=[round(v, 3) for v in crossing], t_go=round(t_go, 3), crossing_sigma=round(sigma, 3))
