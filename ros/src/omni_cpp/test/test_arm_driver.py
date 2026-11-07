"""The robot's C++ arm driver (ArmDriverCore, arm_driver_node.cpp's core) through sil/arm_sil.py, the library the v11 sim
runs it from, against a stand-in servo bus written here from the STS3215 protocol (independently of the C++).
    python -m pytest -q ros/src/omni_cpp/test
"""

from __future__ import annotations

import math
import shutil
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "sil"))
CONFIG = PKG.parent / "omni_arm" / "omni_arm" / "config" / "robot_arm.json"

if sys.platform != "win32" and not shutil.which("g++"):
    pytest.skip("no g++ to build the arm driver library", allow_module_level=True)

import arm_sil  # noqa: E402

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
TORQUE_ENABLE = 40


def _sum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def status(sid: int, err: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, err]) + params
    return b"\xff\xff" + body + bytes([_sum(body)])


def sm(v: int, bit: int) -> int:
    return (abs(v) | (1 << bit)) if v < 0 else v


class Servos:
    """Six STS3215s: SYNC_READ answers position 2048 + 100 id, speed -50, load -20 %, 12.0 V, 31 C; goals and torque
    writes kept. err: the error byte every reply carries; silent: ids that stop answering after the first read."""

    def __init__(self, err: int = 0, silent_after_connect=()):
        self.buf, self.tx = bytearray(), bytearray()
        self.goals, self.torque = {}, {}
        self.err, self.silent, self.reads = err, set(silent_after_connect), 0

    def write(self, data: bytes):
        self.buf += data
        while True:
            k = self.buf.find(b"\xff\xff")
            if k < 0 or len(self.buf) < k + 4:
                return
            n = self.buf[k + 3]
            if len(self.buf) < k + 4 + n:
                return
            pkt = bytes(self.buf[k:k + 4 + n])
            del self.buf[:k + 4 + n]
            assert _sum(pkt[2:-1]) == pkt[-1], "the driver sent a bad checksum"
            self.handle(pkt[2], pkt[4], pkt[5:-1])

    def handle(self, sid: int, instr: int, p: bytes):
        if instr == SYNC_READ:
            self.reads += 1
            for s in p[2:]:
                if self.reads > 1 and s in self.silent:
                    continue
                body = ((2048 + 100 * s).to_bytes(2, "little") + sm(-50, 15).to_bytes(2, "little")
                        + sm(-200, 10).to_bytes(2, "little") + bytes([120, 31]))
                self.tx += status(s, self.err, body)
        elif instr == SYNC_WRITE:
            addr, n = p[0], p[1]
            for i in range(2, len(p), n + 1):
                if addr == 42:
                    self.goals[p[i]] = p[i + 1] | p[i + 2] << 8
        elif instr in (PING, WRITE):
            if instr == WRITE and p[0] == TORQUE_ENABLE:
                self.torque[sid] = p[1]
            self.tx += status(sid, 0)

    def read(self, n: int = 256) -> bytes:
        out = bytes(self.tx[:n])
        del self.tx[:n]
        return out


def test_connects_holding_and_reads_in_radians():
    s = Servos()
    d = arm_sil.CppArmDriver(s, CONFIG)
    assert d.connect() == []
    assert all(v == 1 for v in s.torque.values()) and len(s.torque) == 6          # torque on at connect
    st = d.read(0.0)
    assert st.ok and st.connected and st.torque and st.have_goal and not st.moving
    assert st.q[0] == pytest.approx(100 * 2 * math.pi / 4096, abs=1e-9)
    assert st.load[0] == pytest.approx(-0.2) and st.temp_c[0] == 31 and st.volts == pytest.approx(12.0)
    d.drive(0.0)
    assert s.goals[1] == 2148                                                       # holding where it was read


def test_a_move_plays_from_the_goal_and_ends_on_its_last_waypoint():
    s = Servos()
    d = arm_sil.CppArmDriver(s, CONFIG)
    d.connect()
    q0 = d.read(0.0).q
    assert d.command(arm_sil.MOVE, [1.0, 2.0], [[9.0] * 6, [0.0] * 6])            # its first row is replaced
    d.cycle(1.0)
    assert s.goals[1] == 2148                                                       # starts where the arm was held
    d.cycle(1.5)
    assert d.state().moving and 2048 < s.goals[1] < 2148
    d.cycle(2.0)
    assert not d.state().moving and all(v == 2048 for v in s.goals.values())
    assert q0[0] > 0


def test_refuses_a_malformed_move_and_relaxes_on_request():
    s = Servos()
    d = arm_sil.CppArmDriver(s, CONFIG)
    d.connect()
    assert not d.command(arm_sil.MOVE, [1.0, 2.0], [[0.0] * 6]) and "waypoint" in d.why()
    d.cycle(0.5)
    assert len(s.goals) == 6                                                        # holding: goals every cycle
    assert d.command(arm_sil.RELAX)
    st = d.state()
    assert not st.torque and not st.have_goal and all(v == 0 for v in s.torque.values())
    s.goals.clear()
    d.cycle(1.0)
    assert not s.goals                                                              # limp: no goals written


def test_a_servo_error_or_silence_is_not_ok():
    d = arm_sil.CppArmDriver(Servos(err=0x20), CONFIG)
    d.connect()
    st = d.read(0.0)
    assert not st.ok and st.errors == (1, 2, 3, 4, 5, 6) and st.missing == ()
    d = arm_sil.CppArmDriver(Servos(silent_after_connect=(3,)), CONFIG)
    d.connect()
    st = d.read(0.0)
    assert not st.ok and st.missing == (3,) and st.errors == ()


def test_a_missing_servo_keeps_it_from_connecting():
    class Absent(Servos):
        def handle(self, sid, instr, p):
            if sid != 5:
                super().handle(sid, instr, p)

    d = arm_sil.CppArmDriver(Absent(), CONFIG)
    assert d.connect() == [5] and not d.state().connected
