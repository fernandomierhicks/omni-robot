"""Feetech STS3215 serial bus servos, the Jetson's side (design.md 11): the SO-101's six servos on a USB bus board
(Waveshare's bus servo adapter, 1 Mbaud, half duplex). No ROS; the transport is anything with write(bytes) and
read(n) (pyserial on the robot, a TCP socket to the sim's bus: transport.open_port).

Packets (Feetech's SCS / STS protocol, as LeRobot's feetech backend speaks it):
  instruction  FF FF id len instr params.. chk      len = len(params) + 2, chk = ~(id + len + instr + params) & 0xFF
  status       FF FF id len error params.. chk
Instructions: PING 0x01, READ 0x02, WRITE 0x03, SYNC_READ 0x82, SYNC_WRITE 0x83 (broadcast id 0xFE: no reply).
The STS control table (STS3215, little-endian words; VERIFY the firmware's table against Feetech's for the units that
arrive): see REG. Signed words carry the sign in one bit (sign-magnitude): bit 15 for speeds, bit 10 for the load,
bit 11 for the homing offset.

Calibration (LeRobot's, so its calibrate tool's file works here): per joint id, drive_mode, homing_offset, range_min,
range_max in ticks (4096 a turn). The joint's angle is 0 at the middle of its range (LeRobot's "new calibration", the
sim's convention): rad = sign * (ticks - (range_min + range_max) / 2) * 2 pi / 4096, sign -1 when drive_mode is 1.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

INST_PING, INST_READ, INST_WRITE, INST_SYNC_READ, INST_SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE
TICKS = 4096

# address, size (STS3215 memory table)
REG = dict(
    id=(5, 1), baud=(6, 1), min_limit=(9, 2), max_limit=(11, 2), homing_offset=(31, 2), mode=(33, 1),
    torque_enable=(40, 1), acceleration=(41, 1), goal_position=(42, 2), goal_time=(44, 2), goal_speed=(46, 2),
    torque_limit=(48, 2), lock=(55, 1), present_position=(56, 2), present_speed=(58, 2), present_load=(60, 2),
    present_voltage=(62, 1), present_temperature=(63, 1), moving=(66, 1), present_current=(69, 2),
)
# the block a status read takes in one SYNC_READ: present position .. temperature (56..63)
STATUS_ADDR, STATUS_LEN = 56, 8
SPEED_UNIT = 2 * math.pi / TICKS          # rad/s per goal_speed / present_speed step (ticks/s)
LOAD_UNIT = 0.001                         # present_load: 0.1% of the torque limit's full scale per step
CURRENT_UNIT = 0.0065                     # A per present_current step (Feetech: 6.5 mA)


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def packet(sid: int, instr: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, instr]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


def status_packet(sid: int, error: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, error]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


def to_sm(v: int, bit: int) -> int:
    """A signed value as sign-magnitude with the sign at bit."""
    return (abs(v) & ((1 << bit) - 1)) | ((1 << bit) if v < 0 else 0)


def from_sm(v: int, bit: int) -> int:
    return -(v & ((1 << bit) - 1)) if v & (1 << bit) else v & ((1 << bit) - 1)


def u16(v: int) -> bytes:
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


class Parser:
    """Splits a byte stream into packets: (id, len, instr/error, params) once the checksum is good."""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> list:
        self.buf += data
        out = []
        while True:
            k = self.buf.find(b"\xff\xff")
            if k < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                return out
            if k:
                del self.buf[:k]
            if len(self.buf) < 6:
                return out
            sid, n = self.buf[2], self.buf[3]
            if sid == 0xFF:                       # a third FF: resync on the next byte
                del self.buf[:1]
                continue
            if n < 2 or len(self.buf) < 4 + n:
                if n < 2:
                    del self.buf[:2]
                    self.errors += 1
                    continue
                return out
            body = bytes(self.buf[2:4 + n - 1])
            chk = self.buf[3 + n]
            del self.buf[:4 + n]
            if checksum(body) != chk:
                self.errors += 1
                continue
            out.append((body[0], body[1], body[2], body[3:]))


@dataclass
class JointCal:
    id: int
    drive_mode: int = 0
    homing_offset: int = 0
    range_min: int = 0
    range_max: int = TICKS - 1

    @property
    def mid(self) -> float:
        return 0.5 * (self.range_min + self.range_max)

    @property
    def sign(self) -> float:
        return -1.0 if self.drive_mode else 1.0

    def to_rad(self, ticks: int) -> float:
        return self.sign * (ticks - self.mid) * 2 * math.pi / TICKS

    def to_ticks(self, rad: float) -> int:
        t = int(round(self.mid + self.sign * rad * TICKS / (2 * math.pi)))
        return min(max(t, self.range_min), self.range_max)


def load_calibration(path: str | Path, joints) -> dict:
    """{joint: JointCal} from a LeRobot calibration file (its SO-101 follower's json)."""
    raw = json.loads(Path(path).read_text())
    return {j: JointCal(int(raw[j]["id"]), int(raw[j].get("drive_mode", 0)), int(raw[j].get("homing_offset", 0)),
                        int(raw[j]["range_min"]), int(raw[j]["range_max"])) for j in joints}


def nominal_calibration(joints, ids, ranges) -> dict:
    """The calibration of a servo set up so that each joint's range sits symmetric about tick 2048 (the sim's)."""
    out = {}
    for j, i, (lo, hi) in zip(joints, ids, ranges):
        half = max(abs(lo), abs(hi)) * TICKS / (2 * math.pi)
        out[j] = JointCal(int(i), 0, 0, int(round(2048 - half)), int(round(2048 + half)))
    return out


@dataclass
class ServoStatus:
    position: int = 0          # ticks
    speed: int = 0             # ticks/s (signed)
    load: float = 0.0          # fraction of the torque limit, signed
    volts: float = 0.0
    temp_c: int = 0
    ok: bool = False


class Bus:
    """One half-duplex bus: write instructions, read status packets. timeout s per reply."""

    def __init__(self, port, timeout: float = 0.01, clock=time.monotonic):
        self.port = port
        self.timeout = timeout
        self.parser = Parser()
        self.clock = clock
        self.n_timeouts = 0
        self.n_tx = self.n_rx = 0

    def _send(self, data: bytes):
        self.port.write(data)
        self.n_tx += 1

    def _replies(self, ids: list) -> dict:
        """Status packets from these ids (in any order) until all came or the timeout."""
        want, got = set(ids), {}
        t_end = self.clock() + self.timeout
        while want - set(got):
            for sid, _, err, params in self.parser.feed(self.port.read(256)):
                if sid in want:
                    got[sid] = (err, params)
                    self.n_rx += 1
            if want - set(got) and self.clock() > t_end:
                self.n_timeouts += 1
                break
        return got

    def ping(self, sid: int) -> bool:
        self._send(packet(sid, INST_PING))
        return sid in self._replies([sid])

    def write(self, sid: int, reg: str, value: int, reply: bool = True) -> bool:
        addr, size = REG[reg]
        data = bytes([value & 0xFF]) if size == 1 else u16(value)
        self._send(packet(sid, INST_WRITE, bytes([addr]) + data))
        return sid in self._replies([sid]) if reply else True

    def read(self, sid: int, reg: str) -> int | None:
        addr, size = REG[reg]
        self._send(packet(sid, INST_READ, bytes([addr, size])))
        r = self._replies([sid]).get(sid)
        if r is None or len(r[1]) != size:
            return None
        return r[1][0] if size == 1 else r[1][0] | r[1][1] << 8

    def sync_write(self, reg: str, values: dict):
        """{id: value} into one register on every servo, one packet, no replies."""
        addr, size = REG[reg]
        params = bytes([addr, size])
        for sid, v in values.items():
            params += bytes([sid]) + (bytes([v & 0xFF]) if size == 1 else u16(v))
        self._send(packet(BROADCAST, INST_SYNC_WRITE, params))

    def sync_write_goals(self, goals: dict, speeds: dict | None = None):
        """Goal position (and goal time 0, goal speed) for each id in one SYNC_WRITE starting at goal_position."""
        params = bytes([REG["goal_position"][0], 6])
        for sid, pos in goals.items():
            sp = 0 if speeds is None else int(speeds.get(sid, 0))
            params += bytes([sid]) + u16(pos) + u16(0) + u16(sp)
        self._send(packet(BROADCAST, INST_SYNC_WRITE, params))

    def sync_read_status(self, ids: list) -> dict:
        """{id: ServoStatus} for present position .. temperature, one SYNC_READ."""
        self._send(packet(BROADCAST, INST_SYNC_READ, bytes([STATUS_ADDR, STATUS_LEN]) + bytes(ids)))
        out = {}
        for sid, (err, p) in self._replies(ids).items():
            if len(p) != STATUS_LEN:
                continue
            out[sid] = ServoStatus(position=p[0] | p[1] << 8, speed=from_sm(p[2] | p[3] << 8, 15),
                                   load=from_sm(p[4] | p[5] << 8, 10) * LOAD_UNIT, volts=p[6] * 0.1, temp_c=p[7],
                                   ok=err == 0)
        return out
