"""omni_mcu's own checks (no ROS, no firmware build): the frames, the CMD / STATE layouts against protocol.h, the stream
decoder's error handling and the clock map. The firmware side of the same bytes is checked in simulation/v11 sil
(test_v11.py, through the SIL library) and omni_cpp/test (the C++ link against this one).
    python -m pytest -q ros/src/omni_mcu/test
"""

from __future__ import annotations

import math
import random
import re
import struct
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from omni_mcu import protocol as P          # noqa: E402
from omni_mcu.clock_sync import ClockSync   # noqa: E402

# the repo keeps firmware/ beside ros/; omni.sh mirrors it beside src/ in ~/omni_ws
PROTOCOL_H = next((r / "firmware" / "lib" / "omni_fw" / "protocol.h" for r in (PKG.parents[2], PKG.parents[1])
                   if (r / "firmware").is_dir()), None)


@pytest.mark.skipif(PROTOCOL_H is None, reason="firmware/ isn't beside this checkout")
def test_version_and_sizes_match_protocol_h():
    h = PROTOCOL_H.read_text()
    assert int(re.search(r"PROTOCOL_VERSION = (\d+);", h).group(1)) == P.PROTOCOL_VERSION
    sizes = dict(re.findall(r"sizeof\((\w+)\) == (\d+)", h))
    assert int(sizes["CmdBody"]) == P.CMD_FMT.size
    assert int(sizes["StateBody"]) == P.STATE_FMT.size
    assert int(sizes["ImuBody"]) == P.IMU_FMT.size
    assert int(sizes["InfoBody"]) == P.INFO_FMT.size
    assert int(sizes["TimeBody"]) == P.TIME_FMT.size


def test_crc_is_ccitt_false():
    assert P.crc16(b"123456789") == 0x29B1                 # the standard check value


@pytest.mark.parametrize("raw", [b"", b"\x00", b"\x00\x00", bytes(range(256)), bytes([1] * 600), b"\x11\x00\x22"])
def test_cobs_round_trips_and_never_emits_zero(raw):
    enc = P.cobs_encode(raw)
    assert 0 not in enc
    assert P.cobs_decode(enc) == raw


def test_cmd_carries_its_protocol_version_last():
    body = P.Command(drive_mode=P.DRIVE_VEL_BODY, a=0.5, fire_seq=70000).pack(123)
    assert len(body) == P.CMD_FMT.size
    assert struct.unpack_from("<H", body, len(body) - 2)[0] == P.PROTOCOL_VERSION
    assert struct.unpack_from("<H", body, len(body) - 4)[0] == 70000 & 0xFFFF


def _state_frame(seq=1, t_mcu=1_000_000, **kw):
    v = dict(t_mcu_us=t_mcu, echo_t_host_ns=0, echo_rx_us=0, mode=2, profile=0, drive_mode=1, feed_state=0, faults=0,
             dm_err=(1, 1, 1, 1), x=1.0, y=2.0, yaw=0.5, vx=0.1, vy=0.0, wz=0.0, slip=0.0, roll=0.0, pitch=0.0,
             v_cmd_x=0.0, v_cmd_y=0.0, w_cmd=0.0, fly_rpm=(0.0, 0.0), fly_target_rpm=0.0, tilt=0.0, tilt_cmd=0.0,
             beams=0, fly_ready=0, fire_seq_done=0, fire_rejected=0, v_bus=24.0, cmd_count=5, rx_errors=0,
             cmd_rejected=3, tick_late=2, tick_max_us=1400)
    v.update(kw)
    flat = []
    for x in v.values():
        flat += list(x) if isinstance(x, tuple) else [x]
    return P.frame(P.MSG_STATE, seq, P.STATE_FMT.pack(*flat))


def test_host_link_decodes_a_state_with_protocol_6_counters():
    link = P.HostLink()
    out = link.feed(_state_frame(), 5_000_000_000)
    assert len(out) == 1 and isinstance(out[0], P.State)
    s = out[0]
    assert (s.x, s.y, s.mode_name) == (1.0, 2.0, P.MODES[2])
    assert (s.cmd_rejected, s.tick_late, s.tick_max_us) == (3, 2, 1400)


def test_stream_decoder_skips_garbage_and_corrupt_frames():
    rnd = random.Random(3)
    good = _state_frame()
    bad = bytearray(good)
    bad[5] ^= 0x40                                          # one flipped bit: the CRC must catch it
    stream = bytes(rnd.randrange(1, 256) for _ in range(40)) + b"\x00" + bytes(bad) + good + good
    link = P.HostLink()
    out = []
    for i in range(0, len(stream), 7):                      # arriving in small pieces
        out += link.feed(stream[i:i + 7], 5_000_000_000 + i)
    assert len(out) == 2
    assert link.stats.errors >= 2                           # the garbage and the corrupt frame


def test_decoder_drops_a_runaway_buffer():
    link = P.HostLink()
    link.feed(bytes([7]) * 5000, 0)                        # no delimiter, ever
    assert not link.buf and link.stats.errors == 1
    assert len(link.feed(_state_frame(), 0)) == 1           # and recovers


def test_clock_sync_learns_a_constant_offset_from_echoes():
    cs = ClockSync()
    offset_ns = 3_000_000_000                               # host = mcu + 3 s
    for k in range(50):
        t_mcu_us = 1_000_000 + k * 10_000
        t1 = t_mcu_us * 1000 + offset_ns - 200_000          # sent 0.2 ms before the MCU read it
        t4 = t_mcu_us * 1000 + offset_ns + 300_000          # the reply back 0.3 ms after
        cs.add(t1, t_mcu_us, t_mcu_us, t4)
    t_host = cs.to_host_s(2_000_000)
    assert math.isclose(t_host, 2.0 + 3.0, abs_tol=0.5e-3)
    assert abs(cs.to_mcu_us(t_host) - 2_000_000) <= 2
