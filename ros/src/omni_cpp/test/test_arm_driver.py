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
    """Six STS3215s, each a register image: position 2048 + 100 id, speed -50, load -20 %, 12.0 V, 31 C, current
    -0.65 A, factory P 32 / D 32 and protections; READ, WRITE and SYNC_READ answer from it (whatever range is asked),
    goals and torque writes are kept, writes logged (id, address, value bytes). err: the error byte every reply
    carries; silent: ids that stop answering after the first SYNC_READ."""

    def __init__(self, err: int = 0, silent_after_connect=()):
        self.buf, self.tx = bytearray(), bytearray()
        self.goals, self.torque = {}, {}
        self.err, self.silent, self.reads = err, set(silent_after_connect), 0
        self.writes = []
        self.mem = {}
        for s in range(1, 7):
            m = bytearray(80)
            m[21], m[22], m[23], m[13], m[14], m[15], m[34], m[35], m[36] = 32, 32, 0, 70, 140, 40, 20, 200, 80
            m[28:30] = (500).to_bytes(2, "little")
            m[55] = 1
            m[56:58] = (2048 + 100 * s).to_bytes(2, "little")
            m[58:60] = sm(-50, 15).to_bytes(2, "little")
            m[60:62] = sm(-200, 10).to_bytes(2, "little")
            m[62], m[63] = 120, 31
            m[69:71] = sm(-100, 15).to_bytes(2, "little")
            self.mem[s] = m

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
            addr, n = p[0], p[1]
            for s in p[2:]:
                if self.reads > 1 and s in self.silent:
                    continue
                self.tx += status(s, self.err, bytes(self.mem[s][addr:addr + n]))
        elif instr == READ and sid in self.mem:
            self.tx += status(sid, self.err, bytes(self.mem[sid][p[0]:p[0] + p[1]]))
        elif instr == SYNC_WRITE:
            addr, n = p[0], p[1]
            for i in range(2, len(p), n + 1):
                if addr == 42:
                    self.goals[p[i]] = p[i + 1] | p[i + 2] << 8
        elif instr in (PING, WRITE):
            if instr == WRITE:
                self.writes.append((sid, p[0], bytes(p[1:])))
                self.mem[sid][p[0]:p[0] + len(p) - 1] = p[1:]
                if p[0] == TORQUE_ENABLE:
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


# -- design.md 4.5: the servo settings, the telemetry, the load model, the reflex, the hold trim ---------------------

SRC = PKG.parent
sys.path[:0] = [str(SRC / "omni_calib"), str(SRC / "omni_arm")]


def test_connect_writes_the_settings_that_differ_with_the_lock_open():
    s = Servos()
    d = arm_sil.CppArmDriver(s, CONFIG)
    d.connect()
    notes = d.config_notes()
    assert "id 1 p_gain 32 -> 16" in notes and "id 6 protect_current 500 -> 250" in notes
    for sid in range(1, 7):
        w = [(a, v) for i, a, v in s.writes if i == sid]
        k_open, k_p, k_shut = w.index((55, b"\x00")), w.index((21, b"\x10")), w.index((55, b"\x01"))
        assert k_open < k_p < k_shut                                              # EEPROM: unlocked for the write
    assert s.mem[1][21] == 16 and s.mem[1][13] == 65 and s.mem[6][36] == 25
    s.writes.clear()
    d2 = arm_sil.CppArmDriver(s, CONFIG)                                          # a second connect: nothing differs
    d2.connect()
    assert d2.config_notes() == [] and not [w for w in s.writes if w[1] < 40]


def test_the_status_read_carries_the_current_and_the_error_bits():
    s = Servos()
    s.mem[2][65] = 0x20
    d = arm_sil.CppArmDriver(s, CONFIG)
    d.connect()
    d.read(0.0)
    ld = d.load_state()
    assert ld.current[0] == pytest.approx(-0.65) and ld.has_current
    assert ld.status[1] == 0x20 and ld.volts[0] == pytest.approx(12.0)


def test_a_new_torque_limit_reaches_the_servos_without_a_relax():
    s = Servos()
    d = arm_sil.CppArmDriver(s, CONFIG)
    d.connect()
    s.writes.clear()
    d.settings(arm_torque=600, grip_torque=300)
    d.cycle(0.02)
    lim = {sid: int.from_bytes(v, "little") for sid, a, v in s.writes if a == 48}
    assert lim == {1: 600, 2: 600, 3: 600, 4: 600, 5: 600, 6: 300}


def _model(tmp_path, **joint):
    from omni_arm.kinematics import ArmConfig
    from omni_calib import arm_load as AL
    cfg = ArmConfig(CONFIG)
    m = AL.LoadModel.nominal(cfg)
    for j in m.joints:
        for k, v in joint.items():
            setattr(j, k, v)
    m.source = "test"
    p = tmp_path / "load.json"
    import json
    p.write_text(json.dumps(m.to_json(cfg), default=float))
    return p


def test_a_strained_joint_stops_the_move_backs_off_and_softens_until_the_next_command(tmp_path):
    s = Servos()
    for sid in s.mem:                                                             # a still arm
        s.mem[sid][58:60] = (0).to_bytes(2, "little")
    d = arm_sil.CppArmDriver(s, CONFIG, load=_model(tmp_path, thr=0.2, F=0.01, Fs=0.01, mu=0.0, mus=0.0))
    d.connect()
    q0 = d.read(0.0).q
    d.set_gravity((0.0, 0.0, 0.0))                                                # no gravity: all of it is outside
    assert d.command(arm_sil.MOVE, [0.0, 4.0], [q0, [v + 0.3 for v in q0]])
    s.writes.clear()
    for k in range(1, 10):
        st = d.cycle(0.02 * k)
    ld = d.load_state()
    # the stand-in reports duty -20 % on every joint: a torque the model can't explain, past the threshold
    assert ld.reflexes == 1 and ld.reflex_active and not st.moving and any(ld.strained[:5])
    soft = {sid for sid, a, v in s.writes if a == 48 and int.from_bytes(v, "little") == 350}
    assert soft and soft <= {1, 2, 3, 4, 5}
    # the goal backs off with the push (load -20 %: the outside torque is +): every strained joint gave +0.05 rad
    for j in range(5):
        if ld.strained[j]:
            assert st.goal[j] == pytest.approx(st.q[j] + 0.05, abs=0.02)
    s.writes.clear()
    assert d.command(arm_sil.HOLD)                                                # the next command restores them
    assert {sid for sid, a, v in s.writes if a == 48 and int.from_bytes(v, "little") == 1000} == {1, 2, 3, 4, 5}
    assert not d.load_state().reflex_active


def test_the_reflex_waits_for_a_calibration_unless_allowed():
    s = Servos()
    for sid in s.mem:
        s.mem[sid][60:62] = (900 | 1 << 10).to_bytes(2, "little")             # duty -90 %: plainly strained
    d = arm_sil.CppArmDriver(s, CONFIG)
    d.connect()
    d.set_gravity((0.0, 0.0, 0.0))
    for k in range(1, 10):
        d.cycle(0.02 * k)
    ld = d.load_state()
    assert any(ld.strained) and ld.reflexes == 0                                  # the nominal model: it reports only
    d.settings(reflex_nominal=True)
    d.command(arm_sil.HOLD)
    for k in range(10, 20):
        d.cycle(0.02 * k)
    assert d.load_state().reflexes == 1


def test_a_calibrated_zero_and_sag_move_the_joints_both_ways(tmp_path):
    s = Servos()
    d = arm_sil.CppArmDriver(s, CONFIG, load=_model(tmp_path, zero=0.02, compliance=0.01))
    d.connect()
    st = d.read(0.0)
    ld = d.load_state()
    assert st.q[1] != pytest.approx(ld.q_enc[1]) and st.q[1] > ld.q_enc[1] - 0.03
    d.settings(hold_trim=False)
    d.drive(0.0)                                                                  # holding: the encoder goal is the read
    assert all(abs(s.goals[sid] - (2048 + 100 * sid)) <= 1 for sid in range(1, 6))


def test_the_hold_trim_pulls_the_encoders_onto_the_goal():
    s = Servos()
    for sid in s.mem:
        s.mem[sid][58:60] = (0).to_bytes(2, "little")
        s.mem[sid][60:62] = (0).to_bytes(2, "little")
    d = arm_sil.CppArmDriver(s, CONFIG)
    d.connect()
    q0 = d.read(0.0).q
    target = list(q0)
    target[1] += 0.01
    d.command(arm_sil.MOVE, [0.0, 0.1], [q0, target])
    for k in range(1, 6):
        d.cycle(0.02 * k)                                                         # the stand-in never moves: a droop
    g0 = s.goals[2]
    for k in range(6, 60):
        d.cycle(0.02 * k)
    assert s.goals[2] > g0 + 5                                                    # it asks for more than the goal


def test_the_load_model_matches_its_python_twin():
    import numpy as np
    from omni_arm.kinematics import ArmConfig, ArmKinematics
    from omni_calib import arm_load as AL
    kin = ArmKinematics(ArmConfig(CONFIG))
    d = arm_sil.CppArmDriver(Servos(), CONFIG)
    rng = np.random.default_rng(3)
    m = AL.LoadModel.nominal(kin.cfg)
    for k in range(60):
        q = np.r_[rng.uniform(-1, 1, 5), -0.17]
        args = [q, rng.normal(0, 0.4, 6), rng.uniform(-1, 1, 6), rng.uniform(11, 12.4, 6), rng.uniform(20, 60, 6),
                rng.normal(0, 1, 6), rng.normal(0, 0.3, 6) * (k % 2), rng.normal(0, 2, 6)]
        g = rng.normal(0, 0.3, 3) + [0, 0, -9.81]
        c = d.estimate(*args, g=g)
        p = AL.estimate(m, kin, *args, g=g)
        for key in ("grav", "motor", "ext", "band"):
            assert np.allclose(c[key][:5], p[key][:5], atol=1e-9), key
        assert np.allclose(c["force"], AL.tool_force(kin, q, p["ext"]), atol=1e-7)
        assert np.allclose(c["q_true"], AL.true_angles(m, kin, q, g), atol=1e-9)


def test_a_settled_joint_that_gives_is_strained(tmp_path):
    """The deflection cue: holding, settled (the hold trim's second), a joint the encoders say is off its goal past
    0.7 deg is strained; the reflex lets it give the way it was pushed. Before it settles, nothing."""
    s = Servos()
    for sid in s.mem:
        s.mem[sid][58:60] = (0).to_bytes(2, "little")
        s.mem[sid][60:62] = (0).to_bytes(2, "little")                           # no duty: the load model sees nothing
    d = arm_sil.CppArmDriver(s, CONFIG, load=_model(tmp_path, thr=5.0, thr_move=5.0))
    d.connect()
    d.settings(hold_trim=False)
    q0 = d.read(0.0).q
    goal = list(q0)
    goal[1] += 0.05                                                               # the stand-in never gets there
    d.command(arm_sil.MOVE, [0.0, 0.1], [q0, goal])
    t = 0.0
    while t < 1.0:
        t += 0.02
        d.cycle(t)
    assert d.load_state().reflexes == 0                                           # settling: not yet
    while t < 1.4:
        t += 0.02
        d.cycle(t)
    ld, st = d.load_state(), d.state()
    assert ld.reflexes == 1 and ld.strained[1] and not any(ld.strained[k] for k in (0, 2, 3, 4))
    assert st.goal[1] == pytest.approx(q0[1] - 0.05, abs=0.01)                   # gave further, away from the goal
