"""omni_cpp's C++ cores against the Python ones they replace on the robot: the same inputs through both, the same
outputs (to rounding). The C++ side is test/core_check.cpp, built here on first use (g++ on Linux, zig c++ on Windows,
as omni_catch's fast detector) into ~/.cache/omni_cpp/<hash of the sources>. Run from the repo root, Windows or WSL:
    & $py -m pytest -q ros/src/omni_cpp/test

The arm cores (STS3215 bus, calibration, forward kinematics, motion player) beside the glue (clock map, link, mode mux, heading, scans) and the catch path (design.md 4.9: the robot config, ball flight,
intercept planner, detector, the perception's filter on synthetic stereo throws, and the game tick by tick against a
small world the Python's own commands drive).
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import random
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
for p in ("omni_mcu", "omni_catch", "omni_bringup", "omni_perception", "omni_arm"):
    sys.path.insert(0, str(SRC / p))

from omni_arm import motion as arm_motion                                        # noqa: E402
from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path, rot_quat   # noqa: E402
from omni_bringup.heading import HeadingConfig, HeadingShaper                    # noqa: E402
from omni_catch import fast_detector                                             # noqa: E402
from omni_catch.detector import Detector, DetectorConfig                         # noqa: E402
from omni_catch.flight import BallFlight                                         # noqa: E402
from omni_catch.frames import ArrayFrame, SparseFrame                            # noqa: E402
from omni_catch.game import CatchBehavior, CatchCommand, GameParams, RobotView   # noqa: E402
from omni_catch.intercept import SafetyZone, choose_intercept                    # noqa: E402
from omni_catch.modes import ModeMux                                             # noqa: E402
from omni_catch.perception import Perception                                     # noqa: E402
from omni_catch.robot_config import CONFIG, RobotConfig                          # noqa: E402
from omni_catch.tracker import TrackView, sphere_image                           # noqa: E402
from omni_mcu import protocol as P                                               # noqa: E402
from omni_mcu.clock_sync import ClockSync                                        # noqa: E402
from omni_perception.scan_time import ScanClock, deskew, floor_mask, sample_times   # noqa: E402


def firmware_dir() -> Path:
    for d in (os.environ.get("OMNI_FIRMWARE_DIR"), PKG.parents[2] / "firmware", PKG.parents[1] / "firmware"):
        if d and (Path(d) / "lib" / "omni_fw" / "protocol.cpp").exists():
            return Path(d) / "lib" / "omni_fw"
    pytest.skip("no firmware/lib/omni_fw beside the ROS sources")


def build() -> Path:
    fw = firmware_dir()
    srcs = [PKG / "test" / "core_check.cpp", fw / "protocol.cpp"] + sorted((PKG / "src").glob("[!_]*.cpp"))
    srcs = [s for s in srcs if not s.name.endswith("_node.cpp") and s.name != "gpio_edges.cpp"]
    heads = sorted((PKG / "include" / "omni_cpp").glob("*.hpp")) + [fw / "protocol.h"]
    h = hashlib.sha1(b"".join(p.read_bytes() for p in srcs + heads)).hexdigest()[:12]
    out = Path(os.environ.get("OMNI_NATIVE_CACHE", Path.home() / ".cache" / "omni_cpp")) / h / (
        "core_check.exe" if sys.platform == "win32" else "core_check")
    if out.exists():
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    flags = ["-std=c++17", "-O2", "-Wall", "-I", str(PKG / "include"), "-I", str(fw)]
    if sys.platform == "win32":
        cmd = [sys.executable, "-m", "ziglang", "c++", "-target", "x86_64-windows-gnu", *flags]
    else:
        cmd = ["g++", *flags]
    r = subprocess.run(cmd + [str(s) for s in srcs] + ["-o", str(out)], capture_output=True, text=True)
    assert r.returncode == 0, f"build failed:\n{' '.join(cmd)}\n{r.stdout}\n{r.stderr}"
    return out


EXE = []


def run(core: str, lines: list[str]) -> list[str]:
    if not EXE:
        EXE.append(build())
    r = subprocess.run([str(EXE[0]), core], input="\n".join(lines) + "\n", capture_output=True, text=True,
                       timeout=60)
    assert r.returncode == 0, r.stderr
    return r.stdout.splitlines()


def close(a: float, b: float, tol: float) -> bool:
    return (math.isnan(a) and math.isnan(b)) or a == b or abs(a - b) <= tol


# -- the clock map ------------------------------------------------------------------------------------------------
def clock_events(seed: int):
    """Echoes at 100 Hz for 50 s with USB-like delays, 1 Hz pulses with interrupt-like late stamps (and a lost one),
    an MCU reboot at 35 s; queries along the way."""
    rnd = random.Random(seed)
    off, drift = 1_234_567_890_123 + seed, 37e-6 * (1 if seed % 2 else -1)
    ev, boot = [], 0

    def mcu_us(t_ns):
        return int(((t_ns - off) * (1 + drift) - boot) / 1000)

    for i in range(5000):
        t1 = off + 1_000_000_000 + i * 10_000_000
        if i == 3500:
            boot = (t1 - off) * (1 + drift) - 5e9            # the MCU rebooted: its clock restarts at 5 s
        up, down = rnd.uniform(60e3, 400e3), rnd.uniform(60e3, 400e3)
        t2 = mcu_us(t1 + up)
        t3 = t2 + rnd.randint(50, 900)
        t4 = int(t1 + up + (t3 - t2) * 1000 + down)
        ev.append(f"echo {t1} {t2} {t3} {t4}")
        if i % 100 == 50 and i != 2050:
            tp = t1 + 5_000_000
            ev.append(f"pulse {i // 100} {mcu_us(tp)}")
            late = rnd.expovariate(1 / 8e3) + (200e3 if rnd.random() < 0.05 else 0)
            ev.append(f"edge {int(tp + 2e3 + late)}")
        if i % 37 == 0:
            ev.append(f"host {mcu_us(t1 + 3_000_000)}")
            ev.append(f"mcu {(t1 + 7_000_000) * 1e-9!r}")
            ev.append("status")
    return ev


def clock_python(ev: list[str]) -> list[str]:
    c = ClockSync(edge_latency_ns=2000)
    out = []
    for e in ev:
        op, *a = e.split()
        if op == "cfg":
            c = ClockSync(*map(float, a[:3]), edge_latency_ns=int(a[3]), refit_s=float(a[4]))
        elif op == "echo":
            c.add(*map(int, a))
        elif op == "pulse":
            c.add_mcu_pulse(int(a[0]), int(a[1]))
        elif op == "edge":
            c.add_host_edge(int(a[0]))
        elif op == "host":
            out.append(repr(c.to_host_s(int(a[0]))))
        elif op == "mcu":
            out.append(str(c.to_mcu_us(float(a[0]))))
        elif op == "status":
            out.append(f"{c.mode} {c.rtt_ns if c.rtt_ns is not None else -1} {c.n_echo_used} {c.n_pulse_used} "
                       f"{c.n_rejected} {c.n_resets} {c.n_paired}")
    return out


@pytest.mark.parametrize("seed", [1, 2])
def test_clock_sync_maps_like_the_python(seed):
    ev = ["cfg 30 120 1 2000 0.25"] + clock_events(seed)
    py, cc = clock_python(ev), run("clock", ev)
    assert len(py) == len(cc)
    modes = set()
    for a, b in zip(py, cc):
        if " " in a:                                          # status
            assert a == b
            modes.add(a.split()[0])
        elif "." in a or "nan" in a:                          # a host time, s
            assert close(float(a), float(b), 2e-9), (a, b)
        else:                                                 # an MCU time, us
            assert abs(int(a) - int(b)) <= 1, (a, b)
    assert modes >= {"echo", "pulse"}                         # both lines exercised


# -- the link's bytes -------------------------------------------------------------------------------------------------
def state_body(rnd, t_mcu, echo_t1, echo_rx):
    v = [t_mcu, echo_t1, echo_rx, rnd.randint(0, 4), rnd.randint(0, 1), rnd.randint(0, 3), rnd.randint(0, 3),
         rnd.randint(0, 255), *[rnd.randint(0, 3) for _ in range(4)], *[rnd.uniform(-3, 3) for _ in range(12)],
         *[rnd.uniform(0, 9000) for _ in range(3)], rnd.uniform(-1, 1), rnd.uniform(-1, 1), rnd.randint(0, 3),
         rnd.randint(0, 1), rnd.randint(0, 65535), rnd.randint(0, 9), rnd.uniform(20, 29), rnd.randint(0, 65535),
         rnd.randint(0, 9), rnd.randint(0, 9), rnd.randint(0, 9), rnd.randint(900, 2000)]   # + protocol 6's counters
    return P.STATE_FMT.pack(*v)


def test_link_decodes_and_encodes_like_the_python():
    rnd = random.Random(7)
    t_host, t_mcu = 5_000_000_000, 1_000_000
    stream, lines = bytearray(), []
    for i in range(600):
        t_host += 5_000_000
        t_mcu += 5000
        msgs = [P.frame(P.MSG_STATE, i, state_body(rnd, t_mcu, t_host - 900_000, t_mcu - 400))]
        if i % 2 == 0:
            msgs.append(P.frame(P.MSG_IMU, i, P.IMU_FMT.pack(t_mcu - 300, *[rnd.uniform(-1, 1) for _ in range(10)])))
        if i % 20 == 0:
            msgs.append(P.frame(P.MSG_INFO, i, P.INFO_FMT.pack(t_mcu, 4, b"abc123", 1, 0,
                                                               *[rnd.randint(-1000, 1000) for _ in range(8)],
                                                               *[rnd.randint(20, 90) for _ in range(8)])))
        if i % 200 == 0:
            msgs.append(P.frame(P.MSG_TIME, i, P.TIME_FMT.pack(t_mcu, P.TE_SYNC, i // 200, 0)))
        if i % 97 == 0:
            msgs.append(bytes(rnd.randint(1, 255) for _ in range(rnd.randint(1, 40))) + b"\x00")   # garbage
        if i % 131 == 0:
            msgs.append(P.frame(0x55, i, b"\x01\x02"))                                              # unknown type
        data = b"".join(msgs)
        k = rnd.randint(0, len(data))                          # split across two reads
        for part, dt in ((data[:k], 0), (data[k:], 200_000)):
            if part:
                stream += part
                lines.append(f"feed {part.hex()} {t_host + dt}")
        if i % 50 == 49:
            lines.append(f"cmd {t_host + 300_000} 1 {i % 2} 2 1 1.5 -0.25 0.75 0.5 2.0 1.8 "
                         f"{(t_host + 400_000_000) * 1e-9!r} 0.2 3.0 0.35 {(t_host + 900_000_000) * 1e-9!r} -0.1 "
                         f"6200.0 {i}")
    lines.append("stats")
    link, out = P.HostLink(), []
    for ln in lines:
        op, *a = ln.split()
        if op == "feed":
            for m in link.feed(bytes.fromhex(a[0]), int(a[1])):
                kind = {P.State: "S", P.Imu: "I", P.Info: "N", P.TimeEvent: "T"}[type(m)]
                out.append(f"{kind} {m.t_host!r}")
        elif op == "cmd":
            v = a[1:]
            r = P.Request(req_mode=int(v[0]), kid=bool(int(v[1])), drive_mode=int(v[2]), hold_yaw=bool(int(v[3])),
                          a=float(v[4]), b=float(v[5]), c=float(v[6]), v_end=float(v[7]), a_cap=float(v[8]),
                          speed_cap=float(v[9]), t_brake=float(v[10]), coast_s=float(v[11]), a_brake=float(v[12]),
                          tilt=float(v[13]), t_tilt=float(v[14]), tilt_after=float(v[15]), fly_rpm=float(v[16]),
                          fire_seq=int(v[17]))
            out.append(link.encode(P.to_command(r, link.sync), int(a[0])).hex())
        else:
            s = link.stats
            out.append(f"{s.frames} {s.errors} {s.states} {s.imus} {s.infos} {s.cmds} {s.times}")
    cc = run("link", lines)
    assert len(out) == len(cc)
    for a, b in zip(out, cc):
        if a[:2] in ("S ", "I ", "N ", "T "):
            assert a[0] == b[0] and close(float(a[2:]), float(b[2:]), 2e-9), (a, b)
        else:
            assert a == b, (a, b)
    assert out[-1].split()[1] != "0"                           # the garbage counted as errors


# -- who drives -----------------------------------------------------------------------------------------------------
def test_mode_mux_requests_like_the_python():
    rnd = random.Random(3)
    caps = (ArmConfig().arm_out["speed_cap"], ArmConfig().arm_out["a_cap"])     # robot_arm.json, as mode_manager
    lines, py = ["init TELEOP 0", f"caps {caps[0]!r} {caps[1]!r}"], []
    mux = ModeMux("TELEOP", False, arm_out_caps=caps)
    for _ in range(3000):
        x = rnd.random()
        if x < 0.05:
            m = rnd.choice(["idle", "TELEOP", "catch", "THROWER", "pick", "dance"])
            k = rnd.choice([-1, 0, 1])
            lines.append(f"set {m} {k}")
            ok, _ = mux.set_mode(m, None if k < 0 else bool(k))
            py.append(f"{int(ok)} {mux.mode} {int(mux.kid)}")
        elif x < 0.08:
            a = rnd.randint(0, 1)
            lines.append(f"arm {a}")
            mux.arm_out = bool(a)
        elif x < 0.3:
            s = rnd.choice([0, 1, 2, 3, 65535])
            lines.append(f"fire {s}")
            py.append(str(mux.fire(s)))
        else:
            k = rnd.choice("ntph")
            tw = (rnd.uniform(-2, 2), rnd.uniform(-2, 2), rnd.uniform(-3, 3))
            if k in "tp":
                lines.append(f"req {k} {tw[0]!r} {tw[1]!r} {tw[2]!r}")
                r = mux.request(twist=tw if k == "t" else None, pick=tw if k == "p" else None)
            elif k == "h":
                th = P.Request(tilt=rnd.uniform(-0.5, 0.7), fly_rpm=rnd.uniform(0, 9000), fire_seq=rnd.randint(0, 3))
                lines.append(f"req h {th.tilt!r} {th.fly_rpm!r} {th.fire_seq}")
                r = mux.request(thrower=th)
            else:
                lines.append("req n")
                r = mux.request()
            py.append(" ".join([f"{r.req_mode} {int(r.kid)} {r.drive_mode} {int(r.hold_yaw)}"] +
                               [repr(float(getattr(r, f))) for f in ("a", "b", "c", "v_end", "a_cap", "speed_cap",
                                                                     "t_brake", "coast_s", "a_brake", "tilt", "t_tilt",
                                                                     "tilt_after", "fly_rpm")] + [str(r.fire_seq)]))
    cc = run("mux", lines)
    assert len(py) == len(cc)
    for a, b in zip(py, cc):
        fa, fb = a.split(), b.split()
        assert fa[:1] == fb[:1] and len(fa) == len(fb), (a, b)
        for u, v in zip(fa, fb):
            assert u == v or close(float(u), float(v), 1e-12), (a, b)


# -- which way it faces ---------------------------------------------------------------------------------------------
@pytest.mark.parametrize("mode", ["agnostic", "forward", "spinning"])
def test_heading_shaper_like_the_python(mode):
    rnd = random.Random(len(mode))
    cfg = HeadingConfig(mode=mode, spin_rate=rnd.choice([2.0, -1.5]), turn_max=1.4, lead=0.04, period=0.02)
    sh = HeadingShaper(cfg)
    lines, py = [f"cfg {mode} {cfg.spin_rate!r} {cfg.turn_max!r} {cfg.lead!r} {cfg.period!r}"], []
    x = y = yaw = 0.0
    t = 0.0
    for i in range(4000):
        t += 0.005
        if i % 2 == 0:
            x, y, yaw = x + rnd.uniform(0, 0.01), y + rnd.uniform(-0.005, 0.005), yaw + rnd.uniform(-0.02, 0.02)
            wz = rnd.uniform(-2, 2)
            lines.append(f"odom {t!r} {x!r} {y!r} {yaw!r} {wz!r}")
            sh.on_odom(t, x, y, yaw, wz)
        if i % 200 == 0:
            ln = max(0.0, 6.0 - x) * rnd.uniform(0.9, 1.1)
            lines.append(f"plan {t!r} {ln!r}")
            sh.on_plan(t, ln)
        out = None
        if i % 10 == 0 and not 1500 < i < 1700:                 # Nav2 at 20 Hz, a pause
            v = (rnd.uniform(-1.5, 1.5), rnd.uniform(-1.5, 1.5), rnd.uniform(-1, 1))
            lines.append(f"cmd {t!r} {v[0]!r} {v[1]!r} {v[2]!r}")
            out = sh.on_cmd(t, *v)
            py.append(out)
        elif i % 4 == 0:
            lines.append(f"step {t!r}")
            py.append(sh.step(t))
    cc = run("heading", lines)
    assert len(py) == len(cc)
    for a, b in zip(py, cc):
        if a is None:
            assert b == "none"
        else:
            assert all(close(u, float(v), 1e-9) for u, v in zip(a, b.split())), (a, b)


# -- scans ----------------------------------------------------------------------------------------------------------
def test_floor_mask_like_the_python():
    rnd = np.random.default_rng(5)
    n = 720
    a = -math.pi + 2 * math.pi * np.arange(n) / n
    lines, py = [], []
    for _ in range(20):
        r = rnd.uniform(0.2, 12.0, n)
        r[rnd.random(n) < 0.1] = np.inf
        cal = (math.tan(rnd.uniform(0, 0.03)), rnd.uniform(-math.pi, math.pi), rnd.uniform(0.1, 0.2))
        rp = (rnd.uniform(-0.1, 0.1), rnd.uniform(-0.1, 0.1))
        lines += [f"floor {n} {cal[0]!r} {cal[1]!r} {cal[2]!r} {rp[0]!r} {rp[1]!r} 0.3",
                  " ".join(repr(float(v)) for v in a), " ".join(repr(float(v)) for v in r)]
        py.append("".join("1" if m else "0" for m in floor_mask(a, r, cal, rp, 0.3)))
    cc = run("scan", lines)
    assert py == cc
    assert any("1" in s for s in py)


def test_deskew_like_the_python():
    rnd = np.random.default_rng(6)
    n = 400
    inc = 2 * math.pi / n
    a = -math.pi + inc * np.arange(n)
    lines, py = [], []
    for case in range(12):
        r = rnd.uniform(0.3, 8.0, n)
        r[rnd.random(n) < 0.05] = np.inf
        vx, vy, w = rnd.uniform(-2, 2), rnd.uniform(-2, 2), rnd.uniform(-4, 4)
        x0, y0, yaw0 = rnd.uniform(-3, 3), rnd.uniform(-3, 3), rnd.uniform(-3, 3)
        t0 = 100.0 + case
        ts = sample_times(n, t0, 0.1 / n, -1 if case % 2 else 1)

        def pose(t):
            yaw = (yaw0 + w * t + math.pi) % (2 * math.pi) - math.pi
            return x0 + vx * t, y0 + vy * t, yaw

        lines += [f"deskew {n} 0.05 -0.02 {t0!r} {-math.pi!r} {inc!r} 11 {n} {vx!r} {vy!r} {w!r} {x0!r} {y0!r} "
                  f"{yaw0!r}", " ".join(repr(float(v)) for v in a), " ".join(repr(float(v)) for v in r),
                  " ".join(repr(float(v)) for v in ts)]
        py.append(deskew(a, r, ts, pose, (0.05, -0.02), t_ref=t0, angle_min=-math.pi, angle_increment=inc, n_out=n,
                         n_key=11))
    cc = run("scan", lines)
    assert len(cc) == len(py)
    for want, got in zip(py, cc):
        got = np.array([float(v) for v in got.split()])
        with np.errstate(invalid="ignore"):                    # inf - inf
            same = (want == got) | (np.abs(want - got) <= 1e-9)
        assert same.mean() >= 0.995, f"{(~same).sum()} of {n} bins differ"      # a tie in the rounding may flip


def test_scan_clock_like_the_python():
    rnd = random.Random(11)
    lines, py = ["sclock 0.0015"], []
    sc = ScanClock(0.0015)
    t = 1_761_000_000.0
    for i in range(400):
        t += 0.1002 * (2 if i == 150 else 1) * (200 if i == 300 else 1)       # a lost turn, a long gap
        stamp = t + 0.001 + rnd.expovariate(1 / 0.0008)
        st = 0.0 if i == 80 else 0.0999
        lines.append(f"upd {stamp!r} {st!r}")
        v = sc.update(stamp, st)
        py.append((v, sc.T or 0.0))
    cc = run("scan", lines)
    assert len(cc) == len(py)
    for (v, T), got in zip(py, cc):
        gv, gT = map(float, got.split())
        assert close(v, gv, 1e-6) and close(T, gT, 1e-9), ((v, T), got)
    # a turn without a scan_time (i 80) keeps counting with the known period: the line stays sane
    assert all(abs(T - 0.1002) < 2e-3 for _, T in py[5:])


# -- the catch path (design.md 4.9) -----------------------------------------------------------------------------------
def nclose(a, b, rel=1e-9, abs_=1e-9) -> bool:
    return (math.isnan(a) and math.isnan(b)) or a == b or abs(a - b) <= abs_ + rel * max(abs(a), abs(b))


def same_nums(a: str, b: str, rel=1e-9, abs_=1e-9) -> bool:
    fa, fb = a.split(), b.split()
    return len(fa) == len(fb) and all(u == v or nclose(float(u), float(v), rel, abs_) for u, v in zip(fa, fb))


def same_json(a, b, rel=1e-9, abs_=1e-9) -> bool:
    """Equal JSON values, numbers to rounding, keys in the same order."""
    if isinstance(a, dict):
        return isinstance(b, dict) and list(a) == list(b) and all(same_json(a[k], b[k], rel, abs_) for k in a)
    if isinstance(a, list):
        return isinstance(b, list) and len(a) == len(b) and all(same_json(u, v, rel, abs_) for u, v in zip(a, b))
    if isinstance(a, bool) or a is None or isinstance(a, str):
        return a == b
    return isinstance(b, (int, float)) and not isinstance(b, bool) and nclose(float(a), float(b), rel, abs_)


def r(v) -> str:
    return " ".join(repr(float(x)) for x in v)


def test_json_numbers_like_python():
    import json
    rnd = random.Random(21)
    vals = [0.0, -0.0, 1.0, 100.0, 1e16, 1e-5, 0.0001, 123456789.125, 2.675, -3.5e-7, 1.5e300, float("nan"), 0.1, 1 / 3]
    vals += [rnd.uniform(-1e4, 1e4) * 10 ** rnd.randint(-8, 8) for _ in range(300)]
    lines = [f"pyfloat {v!r}" for v in vals] + [f"round {v!r} {nd}" for v in vals for nd in (2, 3, 4)]
    want = [json.dumps(v) for v in vals] + [json.dumps(round(v, nd)) for v in vals for nd in (2, 3, 4)]
    cc = run("catch", lines)
    assert cc == want


def test_robot_config_like_the_python():
    rc, rnd = RobotConfig(), random.Random(22)
    lines, py = [f"config {CONFIG}"], []
    for tilt in list(rc._tilt[::7]) + [rnd.uniform(-0.7, 0.9) for _ in range(60)]:
        for op, f in (("rim", rc.rim_center), ("exit", rc.exit_point)):
            lines.append(f"{op} {float(tilt)!r}")
            py.append(r(f(tilt)))
    for _ in range(80):
        v, tilt, el = rnd.uniform(1.0, 13.0), rnd.uniform(-0.6, 0.1), rnd.uniform(0.3, 1.0)
        lines += [f"speed {v!r} {tilt!r}", f"elev {v!r} {tilt!r}", f"tiltfor {v!r} {el!r}"]
        py += [r([rc.speed_for(v, tilt)]), r([rc.elevation_at(v, tilt)]), r([rc.tilt_for(v, el)])]
    for k in (0, 1):
        lines.append(f"accel {k}")
        py.append(r([rc.plan_accel(bool(k))]))
        for _ in range(30):
            a, yaw = rnd.uniform(-math.pi, math.pi), rnd.uniform(-math.pi, math.pi)
            lines.append(f"top {math.cos(a)!r} {math.sin(a)!r} {yaw!r} {k}")
            py.append(r([rc.top_speed((math.cos(a), math.sin(a)), yaw, bool(k))]))
    cc = run("catch", lines)
    assert len(cc) == len(py)
    for a, b in zip(py, cc):
        assert same_nums(a, b, 1e-12, 1e-15), (a, b)


def test_flight_and_intercept_like_the_python():
    rc, rnd = RobotConfig(), random.Random(23)
    b = rc.ball
    fl = BallFlight(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])
    lines, py = [f"config {CONFIG}"], []
    for _ in range(150):
        p = (rnd.uniform(-1, 7), rnd.uniform(-3, 3), rnd.uniform(0.3, 2.5))
        v = (rnd.uniform(-12, 2), rnd.uniform(-3, 3), rnd.uniform(-4, 8))
        z = rnd.uniform(0.2, 1.0)
        h = fl.crossing_state(p, v, z)
        lines.append(f"cross {r(p)} {r(v)} {z!r}")
        py.append("none" if h is None else r([*h[0], *h[1], h[2]]))
        lines.append(f"apex {r(p)} {r(v)}")
        py.append(r([fl.apex(p, v)]))
        rp = fl.release_point(p, v, 1.0)
        lines.append(f"release {r(p)} {r(v)} 1.0")
        py.append("none" if rp is None else r([*rp[0], rp[1]]))
        a, el = rnd.uniform(-math.pi, math.pi), rnd.uniform(0.4, 0.95)
        t = (p[0] + rnd.uniform(3, 7) * math.cos(a), p[1] + rnd.uniform(3, 7) * math.sin(a), 1.2)
        lines.append(f"launch {r(p)} {math.cos(a)!r} {math.sin(a)!r} {el!r} {r(t)}")
        py.append(r([fl.solve_launch_speed(p, (math.cos(a), math.sin(a)), el, t)]))
    for _ in range(40):
        people = [(rnd.uniform(-3, 3), rnd.uniform(-2, 2), 0.3) for _ in range(rnd.randint(0, 2))]
        z = SafetyZone(half_x=rnd.uniform(1.5, 6), half_y=rnd.uniform(1.5, 3), people=people, robot_radius=0.21,
                       margin=0.2, center=(rnd.uniform(-1, 1), rnd.uniform(-1, 1)))
        lines.append(f"zone {z.half_x!r} {z.half_y!r} 0.21 0.2 {z.center[0]!r} {z.center[1]!r} {len(people)} " +
                     " ".join(r(q) for q in people))
        for _ in range(10):
            p0, v0 = (rnd.uniform(-1, 1), rnd.uniform(-1, 1)), (rnd.uniform(-1, 1), rnd.uniform(-1, 1))
            pc = (p0[0] + rnd.uniform(-2.5, 2.5), p0[1] + rnd.uniform(-2.5, 2.5))
            T, mv = rnd.uniform(0.0, 1.5), rnd.random() < 0.8
            ch = choose_intercept(p0, v0, pc, T, a=2.3, v_max=1.8, zone=z, coast=0.05, a_ball=2.5, moving=mv)
            lines.append(f"choose {r(p0)} {r(v0)} {r(pc)} {T!r} 2.3 1.8 0.05 2.5 {int(mv)}")
            py.append(r([ch.v_f, *ch.direction, float(ch.feasible), float(ch.gated)]))
    cc = run("catch", lines)
    assert len(cc) == len(py)
    for a, b in zip(py, cc):
        assert (a == "none") == (b == "none") and (a == "none" or same_nums(a, b, 1e-9, 1e-12)), (a, b)
    assert any(x.endswith("1.0") for x in py)                  # the gate capped some


def py_detector():
    cfg = DetectorConfig(backend="native")
    return fast_detector.FastDetector(cfg) if fast_detector.available() else Detector(cfg)


def blob_line(d) -> str:
    return "D " + r([d[0], d[1], d[2], d[3]])


def disc(W: int, H: int, u: float, v: float, rad: float) -> list:
    """The pixels (flat indices) of a disc: a ball drawn where its sphere images."""
    return [int(y * W + x) for y in range(max(0, int(v - rad - 1)), min(H, int(v + rad + 2)))
            for x in range(max(0, int(u - rad - 1)), min(W, int(u + rad + 2)))
            if (x + 0.5 - u) ** 2 + (y + 0.5 - v) ** 2 <= rad * rad]


def test_detector_like_the_python():
    rnd = np.random.default_rng(24)
    W, H = 320, 200
    lines, py = [], []
    det = py_detector()
    for case in range(40):
        ball, clutter = [], []
        for _ in range(rnd.integers(0, 4)):
            ball += disc(W, H, rnd.uniform(-5, W + 5), rnd.uniform(-5, H + 5), rnd.uniform(0.8, 14))
        for _ in range(rnd.integers(0, 6)):
            clutter += disc(W, H, rnd.uniform(0, W), rnd.uniform(0, H), rnd.uniform(1, 25))
        if rnd.random() < 0.6:
            fr = SparseFrame(W, H, ball, clutter, seed=case)
            lines += [f"sparse {W} {H}", " ".join(map(str, ball)), " ".join(map(str, clutter))]
        else:                                          # an image: the ball colour over noise, sent as BGR to the C++
            img = rnd.integers(60, 140, (H, W, 3)).astype(np.int16)
            ys, xs = np.divmod(np.array(ball + clutter, dtype=np.int64), W)
            shade = rnd.uniform(0.7, 1.0, (len(ys), 1))
            img[ys, xs] = (np.array((209, 242, 31)) * shade).astype(np.int16)
            img = img.astype(np.uint8)
            fr = ArrayFrame(img)
            lines += [f"rgb {W} {H} 1", np.ascontiguousarray(img[..., ::-1]).tobytes().hex()]
        d = det.detect(fr)
        lines.append("detect")
        py.append(blob_line(d) if d else f"none {det.n_coarse}")
        pred = (rnd.uniform(0, W), rnd.uniform(0, H), rnd.uniform(0.5, 10))
        d = det.detect(fr, pred)
        lines.append(f"detect {r(pred)}")
        py.append(blob_line(d) if d else f"none {det.n_coarse}")
        for n in (3, 40):
            cs = det.candidates(fr, n)
            lines.append(f"cands {n}")
            py += [f"N {len(cs)}"] + [blob_line(c) for c in cs]
    cc = run("detect", lines)
    assert len(cc) == len(py)
    for a, b in zip(py, cc):
        # the minor axis comes from a cancelling moment sum (sum u^2 / n - mean^2): built with -march=native (FMA)
        # the Python's native detector differs from it by ~1e-9
        assert a.split()[0] == b.split()[0] and same_nums(a[2:], b[2:], 1e-7, 1e-9), (a, b)
    assert sum(x.startswith("D") for x in py) > 60


def stereo_throws(seed: int):
    """Three throws at the robot's cameras (the ball's pixels a disc where its sphere images), a still ball-coloured
    toy and a ball on the floor as clutter, the robot swaying (odometry at 200 Hz), frames at 120 Hz with some dropped
    or occluded. Yields ("pose", t, x, y, yaw, roll, pitch) and ("frame", t, [left or None, right or None])."""
    rnd = random.Random(seed)
    rc = RobotConfig()
    b = rc.ball
    fl = BallFlight(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])
    geo = Perception(rc)                                       # just for its camera geometry
    W, H = rc.cameras[0]["width"], rc.cameras[0]["height"]

    def pose(t):
        return (0.2 * math.sin(0.7 * t), 0.1 * math.sin(0.5 * t), 0.05 * math.sin(0.9 * t), 0.01 * math.sin(3 * t),
                0.015 * math.sin(2.5 * t))

    throws = [(1.0, (6.5, 0.3, 1.0), (0.3, 0.0, 0.45), 1.4), (4.0, (6.0, -1.0, 1.1), (0.5, 0.9, 0.5), 1.0),
              (7.0, (5.5, 1.6, 0.9), (-0.2, -0.4, 0.4), 1.8)]
    traj = []                                                  # (t, pos) every 1 ms while a ball flies
    for t0, rel, tgt, apex in throws:
        v = fl.solve_toss(rel, tgt, apex) + np.array([rnd.uniform(-0.2, 0.2), rnd.uniform(-0.2, 0.2), 0.0])
        p, k = np.array(rel, float), 0
        while p[2] > 0.0:
            traj.append((t0 + 0.001 * k, p.copy()))
            v = v + 0.001 * fl.accel(v)
            p = p + 0.001 * v
            k += 1
    tr_t = np.array([x[0] for x in traj])
    still = [np.array((3.0, -1.2, 0.3)), np.array((4.0, 1.0, 0.037))]
    t_pose, k = 0.0, 0
    while True:
        tc = 0.5 + k / 120 + 0.0003
        if tc > 9.5:
            return
        while t_pose <= tc + 0.03:
            yield ("pose", t_pose, *pose(t_pose))
            t_pose = round(t_pose + 0.005, 6)
        geo.poses.clear()
        for dt in (-0.01, 0.0, 0.01):
            geo.add_pose(tc + dt, *pose(tc + dt))
        cams = geo.cams_at(tc)
        i = min(int(np.searchsorted(tr_t, tc)), len(traj) - 1)
        ball_p = traj[i][1] if abs(tr_t[i] - tc) < 0.0011 else None
        frames = []
        for cam, (c, Rc, f, cx, cy, Rb) in enumerate(cams):
            if (cam == 1 and k % 17 == 5) or (cam == 0 and k % 23 == 7):
                frames.append(None)
                continue

            def draw(center, rad):
                img = sphere_image(Rc.T @ (center - c), rad, f)
                return [] if img is None else disc(W, H, cx + img[0], cy + img[1], img[2])

            ball = draw(ball_p, Rb) if ball_p is not None and k % 31 != 3 else []
            frames.append(SparseFrame(W, H, ball, draw(still[0], 0.05) + draw(still[1], 0.037), seed=k))
        yield ("frame", tc, frames)
        k += 1


def test_perception_like_the_python():
    perc = Perception(RobotConfig(), dcfg=DetectorConfig(backend="native"))
    perc.set_play_area((3.5, 0.0), (6.0, 3.0))
    z_rim = float(perc.robot.rim_center(0.0)[1])
    lines, py = [f"config {CONFIG}", "area 3.5 0.0 6.0 3.0"], []
    for ev in stereo_throws(25):
        if ev[0] == "pose":
            lines.append("pose " + r(ev[1:]))
            perc.add_pose(*ev[1:])
            continue
        _, t, frames = ev
        lines.append(f"frame {t!r} {int(frames[0] is not None)} {int(frames[1] is not None)}")
        for fr in frames:
            if fr is not None:
                lines += [f"{fr.width} {fr.height}", " ".join(map(str, fr.idx)), " ".join(map(str, fr.clutter))]
        blobs = perc.process(t, frames)
        py.append(f"B {len(blobs)} {perc.frames} {perc.detections} {perc.stereo} {perc.det.n_coarse}")
        v = perc.view()
        if v is None:
            py.append("V none")
            continue
        py.append(f"V {v.t!r} {v.n} {int(v.ready)} " + r(v.x) + " " + r(v.P.reshape(-1)))
        cr = v.crossing(v.t, z_rim)
        py.append("C none" if cr is None else "C " + r([*cr[0], cr[2], math.sqrt(max(np.linalg.eigvalsh(cr[3])))]))
    cc = run("percep", lines)
    assert len(cc) == len(py)
    tracked = 0
    for a, b in zip(py, cc):
        if a.startswith("B") or a.endswith("none"):
            assert a == b, (a, b)
        else:
            # the blobs' ~1e-9 (FMA, above) carried through the filter: up to ~5e-9 on a component near zero
            assert a[:2] == b[:2] and same_nums(a[2:], b[2:], 1e-6, 1e-8), (a, b)
            tracked += a.startswith("V")
    assert tracked > 150 and sum(x.startswith("C ") for x in py) > 100     # the throws were tracked to the rim


class World:
    """A small world for the game: the base moves toward the command's target (2 m/s, 3 m/s^2), the tilt and the
    flywheels follow theirs, a ball flies from the thrower (drag) and is caught if it crosses the rim within 0.2 m of
    the funnel; the perception's track is the true state 30 ms late at 120 Hz (ready after 6 frames), with noise.
    bounce: s after a catch that the ball bounces in the pocket, the beam blinking every 70 ms."""

    def __init__(self, rc: RobotConfig, throws, seed: int, bounce: float = 0.0):
        b = rc.ball
        self.rc, self.rnd = rc, random.Random(seed)
        self.fl = BallFlight(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])
        self.throws = list(throws)                          # (t, release, target, apex)
        self.pos, self.vel, self.yaw, self.wz = np.zeros(2), np.zeros(2), 0.0, 0.0
        self.v_des = np.zeros(2)
        self.tilt, self.fly = 0.0, [0.0, 0.0]
        self.ball, self.hist, self.t0 = None, [], 0.0
        self.held, self.t_fire, self.fire_seen = False, None, 0
        self.bounce, self.t_caught = bounce, -1e9
        self.cmd = None

    def step(self, t: float, cmd, dt=0.01):
        if self.throws and t >= self.throws[0][0]:
            _, rel, tgt, apex = self.throws.pop(0)
            v = self.fl.solve_toss(rel, tgt, apex) + np.array([self.rnd.uniform(-0.3, 0.3),
                                                               self.rnd.uniform(-0.3, 0.3), 0.0])
            self.ball, self.hist, self.t0 = (np.array(rel, float), v), [], t
        if self.ball is not None:
            p, v = self.ball
            for _ in range(10):
                z0 = p[2]
                self.hist.append((p.copy(), v.copy()))
                v = v + 0.001 * self.fl.accel(v)
                p = p + 0.001 * v
                rim = self.rc.rim_center(self.tilt)
                c = self.pos + rim[0] * np.array([math.cos(self.yaw), math.sin(self.yaw)])
                if v[2] < 0 and z0 >= rim[1] > p[2] and np.linalg.norm(p[:2] - c) < 0.2:
                    self.held, p, self.t_caught = True, None, t
                    break
                if p[2] < 0:
                    p = None
                    break
            self.ball = None if p is None else (p, v)
        if cmd is not None:
            tx, ty, tyaw = cmd.target
            v_des = (np.array([tx, ty]) - self.pos) * 3.0
            sp = float(np.linalg.norm(v_des))
            if sp > 2.0:
                v_des *= 2.0 / sp
            dv = v_des - self.vel
            n = float(np.linalg.norm(dv))
            self.vel = self.vel + (dv if n <= 3.0 * dt else dv * 3.0 * dt / n)
            self.pos = self.pos + self.vel * dt
            err = (tyaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.wz = max(-3.0, min(3.0, 4.0 * err))
            self.yaw += self.wz * dt
            self.v_des = v_des
            self.tilt += max(-2 * dt, min(2 * dt, cmd.tilt - self.tilt))
            for i in range(2):
                self.fly[i] += max(-200.0, min(200.0, cmd.fly_rpm - self.fly[i]))
            if cmd.fire_seq != self.fire_seen:
                self.fire_seen, self.t_fire = cmd.fire_seq, t
            if self.t_fire is not None and t - self.t_fire > 0.3:
                self.held, self.t_fire = False, None
        self.cmd = cmd

    def rv(self, t: float) -> RobotView:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vb = (c * self.vel[0] + s * self.vel[1], -s * self.vel[0] + c * self.vel[1], self.wz)
        tgt = self.cmd.fly_rpm if self.cmd is not None else 0.0
        ready = tgt > 0 and all(abs(w - tgt) < 0.03 * tgt for w in self.fly)
        return RobotView(t=t, pose=np.array([self.pos[0], self.pos[1], self.yaw]), vel_body=np.array(vb),
                         v_cmd=np.array(self.v_des, float), tilt=self.tilt,
                         tilt_cmd=self.cmd.tilt if self.cmd is not None else 0.0, fly_rpm=tuple(self.fly),
                         fly_target_rpm=tgt, fly_ready=ready, ball_held=self.held and not self.blink(t), mode="AUTO")

    def blink(self, t: float) -> bool:
        return t - self.t_caught < self.bounce and int((t - self.t_caught) / 0.07) % 2 == 1

    def track(self, t: float):
        """The perception's last track, (stamp, x, P, frames), or None."""
        if not self.hist:
            return None
        ts = math.floor((t - 0.03) * 120) / 120
        n = int((ts - self.t0) * 120)
        k = int(round((ts - self.t0) / 0.001))
        if n < 1 or not 0 <= k < len(self.hist):
            return None
        p, v = self.hist[k]
        x = np.concatenate([p, v]) + np.array([self.rnd.gauss(0, 0.01) for _ in range(3)] +
                                              [self.rnd.gauss(0, 0.05) for _ in range(3)])
        P = np.diag([0.01 ** 2] * 3 + [0.05 ** 2] * 3) * (1 + 3.0 / n)
        return ts, x, P, n


GAME_SCENARIOS = {         # game params, throws (t, release, target, apex), World options
    "one thrower": (dict(), [(1.0, (6.5, 0.2, 1.0), (0.2, 0.05, 0.45), 1.3),
                             (8.0, (6.6, -0.3, 1.0), (0.6, 0.4, 0.45), 1.2),
                             (15.0, (6.4, 0.0, 1.1), (1.8, -1.2, 0.45), 0.8),
                             (21.0, (6.5, 0.4, 0.9), (0.1, 0.2, 0.45), 1.6)], dict()),
    "pass game, kid, ceiling": (dict(return_to="other", kid=True, ceiling=2.6),
                                [(1.0, (6.5, 1.2, 1.0), (0.2, 0.05, 0.45), 1.2),
                                 (8.0, (6.4, -1.3, 1.0), (0.3, -0.1, 0.45), 1.2),
                                 (15.0, (6.5, 1.25, 1.0), (0.0, 0.2, 0.45), 1.3)], dict()),
    "bouncing in the pocket": (dict(), [(1.0, (6.5, 0.2, 1.0), (0.2, 0.05, 0.45), 1.3),
                                        (8.0, (6.6, -0.3, 1.0), (0.3, 0.2, 0.45), 1.2),
                                        (15.0, (6.4, 0.0, 1.1), (0.1, -0.2, 0.45), 1.4)], dict(bounce=1.5)),
}


@pytest.mark.parametrize("name", list(GAME_SCENARIOS))
def test_game_like_the_python(name):
    """The state machine's timing: every 10 ms tick the same robot state and track into both, the same command,
    state, counts, log lines and last-throw record out."""
    import json
    extra, throws, world = GAME_SCENARIOS[name]
    rc = RobotConfig()
    logs = []
    game = CatchBehavior(rc, GameParams(thrower=(6.8, 0.0), **extra), log=logs.append)
    lines = [f"config {CONFIG}"]
    for k, v in extra.items():
        lines.append(f"param {k} {v if isinstance(v, str) else repr(float(v))}")
    lines.append("new")
    w = World(rc, throws, seed=len(name), **world)
    py, cmd = [], None
    for i in range(int(28 / 0.01)):
        t = round(i * 0.01, 6)
        w.step(t, cmd)
        rv = w.rv(t)
        lines.append(f"rv {t!r} " + r([*rv.pose, *rv.vel_body, *rv.v_cmd, rv.tilt, rv.tilt_cmd, *rv.fly_rpm,
                                       rv.fly_target_rpm]) + f" {int(rv.fly_ready)} {int(rv.ball_held)}")
        tk, track = w.track(t), None
        if tk is None or t - tk[0] > 0.6:
            lines.append("notrack")
        else:
            ts, x, P, n = tk
            track = TrackView(game.flight, 0.5, ts, x, P, n, n >= 6)
            lines.append(f"track {ts!r} {n} {int(n >= 6)} " + r(x) + " " + r(P.reshape(-1)))
        logs.clear()
        cmd = game.step(t, rv, track)
        lines.append(f"step {t!r}")
        py += [f"L {s}" for s in logs]
        py.append(f"S {game.state} {game.tosses} {game.catches} {cmd.fire_seq} " +
                  r([cmd.drive_mode, *cmd.target, cmd.v_end, cmd.t_brake, cmd.coast_s, cmd.a_brake, cmd.tilt,
                     cmd.t_tilt, cmd.tilt_after, cmd.fly_rpm]))
        py.append("J " + json.dumps(game.last))
    cc = run("game", lines)
    assert len(cc) == len(py), (len(cc), len(py))
    for k, (a, b) in enumerate(zip(py, cc)):
        if a.startswith("L"):
            assert a == b, (k, a, b)
        elif a.startswith("S"):
            sa, sb = a.split(), b.split()
            assert sa[:5] == sb[:5] and same_nums(" ".join(sa[5:]), " ".join(sb[5:]), 1e-7, 1e-9), (k, a, b)
        else:
            assert same_json(json.loads(a[2:]), json.loads(b[2:]), 1e-7, 1e-9), (k, a, b)
    states = {x.split()[1] for x in py if x.startswith("S")}
    assert {"wait", "flight", "settle", "aim", "launch", "return"} <= states
    assert game.catches >= 2 and game.tosses == len(throws)
    assert cmd.fire_seq == game.catches            # every ball the beam saw counts and goes back, once


def test_mode_mux_catch_like_the_python():
    rnd = random.Random(26)
    caps = (ArmConfig().arm_out["speed_cap"], ArmConfig().arm_out["a_cap"])
    lines, py = ["init CATCH 0", f"caps {caps[0]!r} {caps[1]!r}"], []
    mux = ModeMux("CATCH", False, arm_out_caps=caps)
    for i in range(400):
        if i % 50 == 25:
            m, k = rnd.choice(["CATCH", "TELEOP", "CATCH"]), rnd.choice([0, 1])
            lines.append(f"set {m} {k}")
            ok, _ = mux.set_mode(m, bool(k))
            py.append(f"{int(ok)} {mux.mode} {int(mux.kid)}")
            continue
        if i % 40 == 10:
            lines.append(f"arm {int(i % 80 == 10)}")
            mux.arm_out = i % 80 == 10
        c = CatchCommand(drive_mode=rnd.choice([0, 2]),
                         target=(rnd.uniform(-3, 3), rnd.uniform(-3, 3), rnd.uniform(-3, 3)),
                         v_end=rnd.uniform(0, 2), t_brake=rnd.uniform(0, 1e3), coast_s=0.05, a_brake=3.0,
                         tilt=rnd.uniform(-0.5, 0.6), t_tilt=rnd.uniform(0, 1e3), tilt_after=0.0,
                         fly_rpm=rnd.uniform(0, 5000), fire_seq=i // 30)
        lines.append(f"catch {c.drive_mode} " + r([*c.target, c.v_end, c.t_brake, c.coast_s, c.a_brake, c.tilt,
                                                    c.t_tilt, c.tilt_after, c.fly_rpm]) + f" {c.fire_seq}")
        q = mux.request(catch=c)
        py.append(" ".join([f"{q.req_mode} {int(q.kid)} {q.drive_mode} {int(q.hold_yaw)}"] +
                           [repr(float(getattr(q, f))) for f in ("a", "b", "c", "v_end", "a_cap", "speed_cap",
                                                                 "t_brake", "coast_s", "a_brake", "tilt", "t_tilt",
                                                                 "tilt_after", "fly_rpm")] + [str(q.fire_seq)]))
    cc = run("mux", lines)
    assert len(cc) == len(py)
    for a, b in zip(py, cc):
        assert same_nums(a, b, 1e-15, 0), (a, b)


# -- the arm (design.md 11): omni_arm's STS3215 bus, calibration, kinematics and motions against arm_driver's cores -----
def _floats(v) -> str:
    return " ".join(repr(float(x)) for x in np.asarray(v).reshape(-1))


# The STS3215 bus: omni_arm's Python feetech.py was deleted Nov 7 (the robot and the sim run omni_cpp's), so these check
# the C++ against what the Python gave for the same inputs, recorded in test/data/sts3215_golden.json.
GOLDEN = json.loads((PKG / "test" / "data" / "sts3215_golden.json").read_text())


def test_sts3215_packets_and_parser_match_the_recorded_python():
    packets, parser = GOLDEN["test_sts3215_packets_and_parser_like_the_python"]
    assert run("arm", packets["lines"]) == packets["expect"]
    assert run("arm", parser["lines"]) == parser["expect"]
    assert parser["expect"][-1].split()[-1] != "0"                 # the noisy stream had bad packets


def test_sts3215_calibration_matches_the_recorded_python():
    (case,) = GOLDEN["test_sts3215_calibration_like_the_python"]
    out = run("arm", case["lines"])
    assert len(out) == len(case["expect"])
    for a, b in zip(out, case["expect"]):
        assert same_nums(a, b, 1e-15, 1e-15), (a, b)


def test_sts3215_bus_matches_the_recorded_python():
    (case,) = GOLDEN["test_sts3215_bus_like_the_python"]
    out = run("arm", case["lines"])
    assert len(out) == len(case["expect"])
    for k, (a, b) in enumerate(zip(out, case["expect"])):
        ta, tb = a.replace(";", " ").replace("=", " ").split(), b.replace(";", " ").replace("=", " ").split()
        assert len(ta) == len(tb) and all(close(float(x), float(y), 1e-12) for x, y in zip(ta, tb)), (k, a, b)


@pytest.mark.parametrize("name", ["", "d435"])
def test_arm_kinematics_like_the_python(name):
    kin = ArmKinematics(ArmConfig(config_path(name)))
    rnd = np.random.default_rng(13)
    lines = [f"cfg {config_path(name)}"]
    expect = []                                           # (kind, python text)
    for _ in range(40):
        q = rnd.uniform(kin.lo, kin.hi)
        lines.append("fk " + _floats(q))
        fk = kin.fk(q)
        for k in sorted(fk):
            expect.append(("fk", k + " " + _floats(fk[k])))
        lines.append("tool " + _floats(q))
        expect.append(("num", _floats(kin.tool(q))))
        lines.append("toolout " + _floats(q))
        expect.append(("int", str(int(kin.tool_out(q)))))
        for T in (fk["tool"], fk["arm_jaw"], fk["wrist_cam"] @ np.diag([1.0, -1.0, -1.0, 1.0])):
            lines.append("quat " + _floats(T))
            expect.append(("num", _floats(rot_quat(T[:3, :3]))))
    for ax in range(3):                                   # the quaternion's other branch (trace <= 0)
        R = np.diag([-1.0, -1.0, -1.0])
        R[ax, ax] = 1.0
        T = np.eye(4)
        T[:3, :3] = R
        lines.append("quat " + _floats(T))
        expect.append(("num", _floats(rot_quat(R))))
    out = run("arm", lines)
    assert len(out) == len(expect)
    for a, (kind, b) in zip(out, expect):
        if kind == "int":
            assert a == b
        elif kind == "fk":
            assert a.split()[0] == b.split()[0] and same_nums(" ".join(a.split()[1:]), " ".join(b.split()[1:]), 1e-9, 1e-9), (a, b)
        else:
            assert same_nums(a, b, 1e-9, 1e-9), (a, b)


def test_arm_motions_like_the_python():
    rnd = np.random.default_rng(14)
    kin = ArmKinematics()
    sp = arm_motion.speeds(kin)
    lines, py = [], []
    for case in range(12):
        n = int(rnd.integers(2, 7))
        wps = [rnd.uniform(kin.lo, kin.hi) for _ in range(n)]
        m = arm_motion.Motion(wps, sp, 100.0 + case)
        lines += [f"play 0 {n}", " ".join(repr(float(t)) for t in m.t), _floats(m.q)]
        for t in list(np.linspace(m.t[0] - 0.1, m.t[-1] + 0.2, 40)) + list(m.t):
            lines.append(f"sample {float(t)!r}")
            py.append(_floats(m.sample(t)))
        times = [100.0 + case] + sorted(100.0 + case + float(x) for x in rnd.uniform(0.2, 3.0, n - 1))
        lin = arm_motion.Linear(times, wps)
        lines += [f"play 1 {n}", " ".join(repr(t) for t in times), _floats(wps)]
        for t in list(np.linspace(times[0] - 0.1, times[-1] + 0.2, 40)) + times:
            lines.append(f"sample {float(t)!r}")
            py.append(_floats(lin.sample(t)))
    out = run("arm", lines)
    assert len(out) == len(py)
    for a, b in zip(out, py):
        assert same_nums(a, b, 1e-9, 1e-12), (a, b)
