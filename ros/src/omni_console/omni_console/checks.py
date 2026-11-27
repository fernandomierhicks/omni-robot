"""The Test workspace's robot side, ROS-free: the preflight (automatic, nothing moves) and the bring-up checks (guided,
attended, one at a time: design.md 8). missions.py serves both through /console/command (verbs preflight, tests,
test_record), so the page, a script or an AI client can run them.

Preflight: preflight(status, fw) reads the status the node is about to publish (mission_node.publish_status: the MCU,
the RC, localization, the arm, the computer, Wi-Fi) and returns one row per check:
  {id, label, state: ok | warn | fail | na, detail, blocks}
state "fail" with blocks set refuses the features named there (missions.py gate()), with the check's detail as the
reason. Only failures that would make the task fail anyway block (an MCU that is silent, latched in a fault stop or
speaks another protocol, drive motors missing, a silent IMU, no lidar scan, a flat pack, a lost localization, an
unhealthy arm); everything else only warns. A part a task needs and the robot doesn't hear from (no lidar scan, no
heading) refuses that task: it would fail. A part nothing needs to start is "na" when it can't be seen.

blocks names features, or one of three groups: MOTION (every feature that moves the base or the arm: moves), BASE
(every feature that drives the base, by hand too: base) and SELF_DRIVE (the base driving itself: base and moves).
unavailable(rows, caps) turns the rows and the caps into {feature: why} for every feature that can't start now: the
robot says what works, the page greys the rest with these words (status "avail"), and gate() refuses with them.

The lidar's self-test (camera_snap at boot, rplidar.LidarSession.self_test: it spins the lidar, counts a few turns and
stops it) reaches the preflight as status "lidar_test": the lidar row says whether the device itself works when no
ROS scan comes.

Bring-up: BRINGUP is the catalog. Each check says what it proves, the preconditions the operator ticks, the steps,
what to expect, the numbers to write down, the status fields to watch live, and buttons for the console verbs that
help (every one an ordinary verb: nothing here moves the robot by itself). test_record stores the outcome with the
robot's configuration (Results), in bringup.json and the session log. The thresholds are estimates until the real
robot has been measured (design.md: VERIFY).
"""

from __future__ import annotations

import math

STATES = ("ok", "warn", "fail", "na")
# blocks groups (never a feature id: a row's blocks hold both): MOTION every feature that moves the base or the arm
# (FEATURES moves), BASE every feature that drives the base, by hand too (FEATURES base), SELF_DRIVE the base driving
# itself (base and moves: not driving by hand)
MOTION = "motion"
BASE = "driving"
SELF_DRIVE = "self_driving"
# the bench tests a person runs and judges (the motor test, wheels up): refused only by what makes them impossible (no
# MCU, firmware of another protocol, the e-stop), never by a fault stop, a low pack, the IMU or a missing motor
BENCH = ("motors",)
BATT_STOP_V = 21.0                 # the firmware's low-battery stop (design.md 2: provisional, 6S)
BATT_WARN_V = 22.2                 # the fault stop's re-arm level: below it, a stop latches until the pack is above
MCU_SILENT_S = 1.0                 # s without MCU state: the link is down
# MCU faults that refuse a start: motion (a person has to clear them); ESTOP and MOTOR_ERR also driving by hand (the
# motors can't turn). LOW_BATT and FAULT_STOP leave driving by hand, to drive to the charger. MOTOR_SILENT and
# IMU_SILENT have their own rows (drive motors, IMU)
FAULTS_BLOCK = ("ESTOP", "FAULT_STOP", "LOW_BATT", "MOTOR_ERR")
FAULTS_BASE = ("ESTOP", "MOTOR_ERR")
OWN_ROWS = ("IMU_SILENT", "MOTOR_SILENT")
DM_NO_REPLY = 0xFF                 # protocol 8: STATE dm_err of a motor that isn't answering (protocol.h)
N_WHEELS = 4
ARM_HOT_C = 65.0                   # servo temperature worth a warning (est, STS3215 class)
CPU_HOT_C = 85.0                   # the robot computer's hottest sensor (est)
WIFI_WEAK_DBM = -72.0
SYNC_SLOW_MS = 5.0                 # MCU clock-sync round trip (est): above it event times get worse
NAV_FEATURES = ("goto", "patrol", "explore")
LIDAR_FEATURES = NAV_FEATURES + ("follow", "message")      # what a silent lidar refuses (Nav2, the people)
LIDAR_SILENT_S = 1.0               # s without a scan: the lidar is down
LIDAR_SLOW_HZ = 5.0                # the A2M12 turns at ~10 Hz: slower, its motor or its USB link is struggling
TILT_WARN_DEG = 15.0               # the base tilted this much while the IMU reports: a ramp, a step, or tipped
ARM_FEATURES = ("tidy", "fetch", "tower", "arm")
CAMERA_FEATURES = ("tidy", "fetch", "tower", "goalie")     # what the floor camera's depth finds things for
LIDAR_TEST_OLD_S = 24 * 3600.0     # a self-test older than this is said to be old (the lidar may have changed since)
LIDAR_TEST_FILE = "lidar_selftest.json"   # in the state dir: camera_snap writes it, mission_node reads it


def _row(id_, label, state, detail="", blocks=()):
    assert state in STATES, state
    return dict(id=id_, label=label, state=state, detail=detail, blocks=list(blocks) if state == "fail" else [])


def preflight(s: dict, fw: dict | None = None) -> list:
    """s: the console status (mission_node.publish_status, before it is sent). fw: dict(expected, protocol_host), the
    firmware this software was built for (None: unknown)."""
    sim = s.get("source") == "sim"
    out = []
    mcu = s.get("mcu")
    if mcu is None:
        out.append(_row("mcu_link", "MCU link", "fail", "No state from the MCU yet: is mcu_link running and the "
                                                        "Teensy plugged in?", (MOTION, BASE) + BENCH))
    elif (mcu.get("age") or 0.0) > MCU_SILENT_S:
        out.append(_row("mcu_link", "MCU link", "fail", f"The MCU has been silent for {mcu['age']:.0f} s (USB link "
                                                        "down); it stops the robot by itself", (MOTION, BASE) + BENCH))
    else:
        rtt = mcu.get("sync_rtt_ms")
        slow = rtt is not None and rtt > SYNC_SLOW_MS
        out.append(_row("mcu_link", "MCU link", "warn" if slow else "ok",
                        f"Clock sync round trip {rtt:.1f} ms (over {SYNC_SLOW_MS:g} ms: event times get worse)"
                        if slow else f"{mcu.get('mode', '?')}, {mcu.get('profile', '?')} profile"
                        + (f", sync {rtt:.1f} ms" if rtt else "")))

    f = s.get("fw") or {}
    fw = fw or {}
    if not f:
        out.append(_row("firmware", "Firmware version", "na", "The MCU hasn't sent INFO yet"))
    elif fw.get("protocol_host") is not None and f.get("protocol") != fw["protocol_host"]:
        out.append(_row("firmware", "Firmware version", "fail",
                        f"The MCU speaks protocol {f.get('protocol')}, the robot computer {fw['protocol_host']}: it "
                        "drops every command. Build and flash the firmware from this checkout", (MOTION, BASE) + BENCH))
    elif fw.get("expected") and f.get("build") and f["build"] != fw["expected"]:
        out.append(_row("firmware", "Firmware version", "warn",
                        f"The MCU runs {f['build']}, this software expects {fw['expected']}: rebuild and flash"))
    else:
        out.append(_row("firmware", "Firmware version", "ok", f"build {f.get('build') or '?'}, protocol "
                                                              f"{f.get('protocol')}"))

    if mcu is not None:
        names = [x.get("name") for x in mcu.get("faults") or [] if x.get("name") not in OWN_ROWS]
        hard = [n for n in names if n in FAULTS_BLOCK]
        if mcu.get("mode") == "ESTOP" and "ESTOP" not in hard:
            hard.append("ESTOP")
        if hard:
            base = any(n in FAULTS_BASE for n in hard)
            out.append(_row("faults", "MCU faults", "fail", f"{', '.join(hard)}: a person has to clear it "
                                                            "(STOP disarms; FAULT_STOP also needs the pack ≥ 22.2 V)"
                            + ("" if base else ". Driving by hand still works"),
                            ((MOTION, BASE) if base else (MOTION,)) + (BENCH if "ESTOP" in hard else ())))
        elif names:
            out.append(_row("faults", "MCU faults", "warn", ", ".join(n for n in names if n)))
        else:
            out.append(_row("faults", "MCU faults", "ok", "none"))

        v = mcu.get("v_bus")
        if v is None or not math.isfinite(v) or v < 1.0:
            out.append(_row("battery", "Battery", "na", "No pack voltage (bench supply off, or not measured)"))
        elif v < BATT_STOP_V:
            out.append(_row("battery", "Battery", "fail", f"{v:.1f} V: under the {BATT_STOP_V:g} V stop. Charge it",
                            (MOTION,)))
        elif v < BATT_WARN_V:
            out.append(_row("battery", "Battery", "warn", f"{v:.1f} V: low (a fault stop would latch under "
                                                          f"{BATT_WARN_V:g} V)"))
        else:
            pct = mcu.get("battery_pct")
            out.append(_row("battery", "Battery", "ok", f"{v:.1f} V" + (f", about {pct}%" if pct is not None else "")))

    if mcu is not None:
        out.append(drive_row(mcu.get("dm_err")))
        imu_silent = any(x.get("name") == "IMU_SILENT" for x in mcu.get("faults") or [])
        roll, pitch = mcu.get("roll"), mcu.get("pitch")
        if imu_silent:
            out.append(_row("imu", "IMU", "fail", "The BNO085 isn't reporting: no heading, so nothing that drives by "
                                                  "itself can start (odometry would drift in every turn). Driving "
                                                  "by hand still works", (SELF_DRIVE,)))
        elif roll is None or pitch is None:
            out.append(_row("imu", "IMU", "na", "No attitude in the MCU's state"))
        else:
            tilt = math.degrees(max(abs(roll), abs(pitch)))
            out.append(_row("imu", "IMU", "warn" if tilt > TILT_WARN_DEG else "ok",
                            f"tilted {tilt:.0f} deg: a ramp, a step, or tipped?" if tilt > TILT_WARN_DEG else
                            f"roll {math.degrees(roll):+.1f} deg, pitch {math.degrees(pitch):+.1f} deg"))

    lid = s.get("lidar")
    test = lidar_test_text(s.get("lidar_test"), s.get("t"))
    if lid is None:
        out.append(_row("lidar", "Lidar", "fail", "No scan: no navigation, no people (the lidar's ROS driver isn't "
                                                  "running" + (f", or is still starting). {test}" if test else
                                                               ", or is still starting)"), LIDAR_FEATURES))
    elif lid["age"] > LIDAR_SILENT_S:
        out.append(_row("lidar", "Lidar", "fail", f"No scan for {lid['age']:.0f} s: no navigation, no people "
                                                  "(USB cable, the motor, the driver)" + (f". {test}" if test else ""),
                        LIDAR_FEATURES))
    else:
        slow = lid["hz"] < LIDAR_SLOW_HZ
        out.append(_row("lidar", "Lidar", "warn" if slow else "ok",
                        f"{lid['hz']:.1f} Hz" + (" (slow: ~10 Hz expected)" if slow else "")))

    rc = s.get("rc")
    if rc is None:
        out.append(_row("rc", "RC receiver", "na", "No INFO from the MCU"))
    elif not rc.get("ok"):
        out.append(_row("rc", "RC receiver", "na" if sim else "warn",
                        "The sim has no receiver" if sim else "No transmitter: the RC override and its failsafe "
                                                              "are not available. Turn it on and check the binding"))
    else:
        out.append(_row("rc", "RC receiver", "ok", "MANUAL: RC has control" if rc.get("manual") else "link up"))

    caps = s.get("caps") or {}
    out.append(_row("nav", "Navigation", "ok" if caps.get("nav") else "warn",
                    "Nav2 is active" if caps.get("nav") else "Nav2 isn't running: Go To, Patrol and Explore are off"))
    fl = s.get("floor")                                 # omni_arm's height map: Explore waits / stops while blind
    if fl is not None:
        b = fl.get("blind")
        out.append(_row("floor_cam", "Floor camera (drops)", "warn" if b else "ok",
                        f"can't see a step down: {b}. Explore stows the arm and waits for it; it stops if it goes "
                        "blind on the way" if b else "the height map sees the floor"))
    st = s.get("stuck")                                 # missions._jammed: warn only, a new mode clears it
    if st is not None:
        out.append(_row("stuck", "Stuck", "warn",
                        f"Got stuck at ({st.get('x', 0):.1f}, {st.get('y', 0):.1f}) on the way to {st.get('goal', '?')}"
                        f": {st.get('why', '?')}. Look at that spot; starting a mode clears this"))

    loc = s.get("localization")
    if loc is None:
        out.append(_row("localization", "Localization", "na", "Nobody assesses it (omni_slam's health isn't running)"))
    else:
        lvl = str(loc.get("level", "?"))
        why = "; ".join(loc.get("reasons") or [])
        state = "fail" if lvl == "LOST" else "warn" if lvl == "POOR" else "ok"
        sig = loc.get("sigma_xy")
        out.append(_row("localization", "Localization", state,
                        lvl + (f" (σ {100 * sig:.0f} cm)" if isinstance(sig, (int, float)) else "")
                        + (f": {why}" if why else ""), NAV_FEATURES))

    arm = s.get("arm")
    if arm is None:
        out.append(_row("arm", "Arm", "na", "No arm state (arm_node not running, or not the arm robot)"))
    elif arm.get("ok") is False:
        out.append(_row("arm", "Arm", "fail", "The arm reports unhealthy servo feedback: check the bus and the "
                                              "Arm tab's temperatures and loads", ARM_FEATURES))
    else:
        temps = [t for t in arm.get("temps") or [] if isinstance(t, (int, float))]
        hot = max(temps) if temps else None
        out.append(_row("arm", "Arm", "warn" if hot is not None and hot > ARM_HOT_C else "ok",
                        (f"a servo at {hot:.0f} °C" if hot is not None and hot > ARM_HOT_C else
                         f"{'torque on' if arm.get('torque') else 'limp'}"
                         + (f", supply {arm['volts']} V" if arm.get("volts") else ""))))

    vis = s.get("vision") or {}
    imgs = vis.get("images") or []
    if not vis.get("video_port"):
        out.append(_row("cameras", "Camera streams", "warn" if arm is not None else "na",
                        "web_video_server isn't running: no pictures on the page or for the AI"))
    else:
        out.append(_row("cameras", "Camera streams", "ok" if imgs else "warn",
                        f"{len(imgs)} image topics" if imgs else "No image topics (is the D435 driver running?)"))
    if arm is not None:                            # the arm robot finds things on the floor in the D435's depth
        depth = [t for t in imgs if "depth" in t]
        out.append(_row("floor_camera", "Floor camera", "ok" if depth else "fail",
                        depth[0] if depth else "No depth image (the D435's driver isn't running): the arm can't find "
                                               "anything on the floor", CAMERA_FEATURES))

    sys_ = s.get("sys") or {}
    t = sys_.get("temp_c")
    if not sys_:
        out.append(_row("computer", "Robot computer", "na", ""))
    else:
        hot = t is not None and t > CPU_HOT_C
        out.append(_row("computer", "Robot computer", "warn" if hot else "ok",
                        f"load {sys_.get('load')}" + (f", {t} °C" if t is not None else "")
                        + (" (hot: it may throttle)" if hot else "")))

    w = s.get("wifi")
    if w is None:
        out.append(_row("wifi", "Wi-Fi", "na", "No Wi-Fi interface (the sim runs in WSL)" if sim else
                        "No Wi-Fi interface reported"))
    else:
        weak = w.get("dbm") is not None and w["dbm"] < WIFI_WEAK_DBM
        out.append(_row("wifi", "Wi-Fi", "warn" if weak else "ok",
                        f"{w.get('dbm')} dBm" + (" (weak: the page and the gamepad may lag)" if weak else "")))
    return out


def brief(rows: list) -> dict:
    """The status's short form: counts, the worst rows' labels, and each row's id, label and state (with its detail
    when it isn't ok): Home's health strip. The Preflight page asks for the full rows."""
    n = {k: sum(1 for r in rows if r["state"] == k) for k in STATES}
    bad = [r["label"] for r in rows if r["state"] == "fail"] or [r["label"] for r in rows if r["state"] == "warn"]
    return dict(n, worst=bad[:3], rows=[dict(id=r["id"], label=r["label"], state=r["state"],
                                             **({"detail": r["detail"]} if r["state"] in ("fail", "warn") else {}))
                                        for r in rows])


def _wheels(ix: list) -> str:
    n = [str(i + 1) for i in ix]
    return n[0] if len(n) == 1 else f"{', '.join(n[:-1])} and {n[-1]}"


def drive_row(dm_err) -> dict:
    """Which drive motors answer on CAN (STATE dm_err, DM_NO_REPLY = silent, protocol 8). Driving needs all four: a
    missing one refuses everything that drives the base, by hand too; the motor test turns the ones that answer."""
    codes = list(dm_err or [])[:N_WHEELS]
    if len(codes) < N_WHEELS:
        return _row("drive", "Drive motors", "na", "The MCU's state has no motor codes")
    gone = [i for i, e in enumerate(codes) if e == DM_NO_REPLY]
    if not gone:
        return _row("drive", "Drive motors", "ok", f"all {N_WHEELS} answering on CAN")
    have = [i for i in range(N_WHEELS) if i not in gone]
    if not have:
        return _row("drive", "Drive motors", "fail", "No drive motor answers on CAN (motor power off? the CAN cable? "
                                                     "each motor's CAN status broadcast on?)", (BASE,))
    return _row("drive", "Drive motors", "fail",
                f"Wheel{'s' if len(gone) > 1 else ''} {_wheels(gone)} not answering on CAN (only {len(have)} of "
                f"{N_WHEELS}): driving needs all four. Test › Motors turns wheel{'s' if len(have) > 1 else ''} "
                f"{_wheels(have)}", (BASE,))


def lidar_test_text(t: dict | None, now: float | None = None) -> str:
    """The boot self-test's result in a sentence ('' if there is none)."""
    if not t or "ok" not in t:
        return ""
    import time
    when = time.strftime("%H:%M", time.localtime(t.get("t", 0)))
    old = now is not None and t.get("t") and now - t["t"] > LIDAR_TEST_OLD_S
    when = f"{when}{', over a day ago' if old else ''}"
    if t["ok"] is None:
        return f"Lidar self-test skipped at {when}: {t.get('detail', '')}"
    if t["ok"]:
        return (f"The lidar itself works: self-test passed at {when} ({t.get('model', '?')}, health "
                f"{t.get('health', '?')}, {t.get('hz', 0):.1f} turns/s, {t.get('samples', 0)} samples a turn)")
    return f"The lidar's self-test failed at {when}: {t.get('detail', '')}"


def blocking(rows: list | None, feat: dict) -> list:
    """The failing rows that refuse this feature (a FEATURES entry: id, moves, base)."""
    fid, moves, base = feat["id"], bool(feat.get("moves")), bool(feat.get("base"))
    groups = {MOTION} if moves else set()
    if base:
        groups.add(BASE)
        if moves:
            groups.add(SELF_DRIVE)
    return [r for r in rows or () if r["state"] == "fail" and (fid in r["blocks"] or groups & set(r["blocks"]))]


# what a missing cap means, in the operator's words (the page's features.js NEED_TEXT is its fallback for a robot
# that doesn't send "avail")
NEED_TEXT = {"nav": "Needs navigation (Nav2 and SLAM aren't running)", "arm": "Needs the arm (arm_node isn't running)",
             "catch": "Needs the catch code (the parked catch robot)", "sim": "The sim only",
             "collision": "Needs Nav2's collision monitor"}


def unavailable(rows: list | None, caps: dict, features) -> dict:
    """{feature id: why} for every feature that can't start now: a cap it needs is down, or a failing check refuses
    it. The robot's own answer to "what works": the page greys these with the reason, gate() refuses them."""
    out = {}
    for f in features:
        why = [NEED_TEXT.get(n, f"Needs {n}") for n in f.get("needs", ()) if not caps.get(n)]
        why += [f"{r['label']}: {r['detail']}" for r in blocking(rows, f)]
        if why:
            out[f["id"]] = "; ".join(why)
    return out


# -- bring-up (design.md 8): guided, attended checks, in the order the build meets them ---------------------------------
def _c(id_, step, title, why, pre, steps, expect, measure=(), watch=(), actions=(), sim=True, auto=None):
    """auto: the robot runs this check by itself ("lidar": camera_snap's self-test at boot); the page shows its last
    result and a button to run it again."""
    return dict(id=id_, step=step, title=title, why=why, pre=list(pre), steps=list(steps), expect=expect,
                measure=[dict(key=k, label=lbl, unit=u) for k, lbl, u in measure], watch=list(watch),
                actions=[dict(label=lbl, verb=v, args=a) for lbl, v, a in actions], sim=sim, auto=auto)


BRINGUP = (
    _c("ak_setup", "3 Drive bench", "AK40-10 setup (CubeMars tool)",
       "The firmware drives the AK40-10s in servo mode (firmware/lib/omni_fw/ak_motor.h). The motor keeps these settings; "
       "none of them can be set over CAN by the firmware.",
       ["Each motor on its own, on the CubeMars tool (R-Link / USB-CAN)", "Current-limited supply"],
       ["Set servo mode and the controller id: 1 front-left ... 4, as fw_params_arm.h DM_IDS.",
        "Turn send_can_status on at 500 Hz (it is off at the factory: then the firmware sees nothing, MOTOR_SILENT).",
        "Set timeout_msec to 100 and timeout_brake_current to a few amps (the factory's 1000 ms and 0 A coast for a "
        "second after the MCU goes quiet).",
        "Wire a common ground to the motor (its UART connector's GND): without it no frame arrives (wheeled-leg robot).",
        "Never 'set origin permanent' at start-up: it writes the motor's flash."],
       "Each motor's status reaches the console (no MOTOR_SILENT) with the bus at 1 Mbit/s.",
       [("status_hz", "status rate seen", "Hz")], ["mcu.dm_err", "mcu.age"], []),
    _c("ak_scale", "3 Drive bench", "AK40-10 speed scale and direction",
       "ERPM = output rpm x 10 (gear) x 14 (pole pairs, from listings: VERIFY). A wrong pole-pair count scales every "
       "wheel speed and the odometry with no error anywhere.",
       ["Wheels off the floor", "A mark on one wheel"],
       ["Take control and drive forward at 0.2 m/s for 30 s.",
        "Count that wheel's turns: expect 0.2 / (2 pi x 0.07) x 30 = 13.6 turns."],
       "Within 2% of the expected turns, and every wheel turns the way the kinematics expect.",
       [("turns", "turns counted", "")], ["mcu.vx"],
       [("Take control", "mission", {"name": "MANUAL"}), ("Stop", "stop", {})]),
    _c("link", "3 Drive bench", "MCU link and watchdog",
       "The MCU stops the robot by itself when the computer goes quiet (0.5 s link watchdog, then disarm).",
       ["Wheels off the floor", "Current-limited supply (or the pack with the hardware e-stop in reach)"],
       ["Take control in Drive and hold a slow forward stick.",
        "Unplug the MCU's USB cable (or stop mcu_link) while the wheels turn."],
       "The wheels ramp to a stop within about 0.5 s and the MCU disarms; the status bar shows MCU SILENT.",
       [("stop_s", "time to stop", "s")], ["mcu.mode", "mcu.age"],
       [("Take control", "mission", {"name": "MANUAL"}), ("Stop", "stop", {})], sim=True),
    _c("wheels", "3 Drive bench", "Wheel directions and encoder signs",
       "Each wheel turns the way the kinematics expect, and its encoder counts the same way.",
       ["Wheels off the floor", "Current-limited supply", "Kid profile on"],
       ["Take control and push forward slowly: all four wheels roll forward.",
        "Slide left, then turn left: compare with the X layout (Base tab shows each wheel's speed).",
        "Watch the Base tab: the measured body velocity has the sign of the stick."],
       "Every wheel moves the right way and vx / vy / wz in the Base tab follow the stick's signs.",
       [("wrong_wheels", "wheels reversed", "")], ["mcu.vx", "mcu.vy", "mcu.wz", "mcu.dm_err"],
       [("Kid profile on", "set", {"key": "profile.kid", "value": True}),
        ("Take control", "mission", {"name": "MANUAL"}), ("Stop", "stop", {})]),
    _c("ramped_stop", "4 Rolling base", "Ramped stop",
       "STOP ramps the base down instead of locking the wheels (the arm robot's tip margin depends on it).",
       ["Clear floor, 3 m in front", "Kid profile on for the first runs"],
       ["Drive forward at a steady speed.", "Press STOP (or the space bar)."],
       "A smooth stop, no wheel lock, no tip; the robot ends disarmed (IDLE).",
       [("speed", "speed before", "m/s"), ("distance", "stopping distance", "m")], ["mcu.vx", "mcu.mode"],
       [("Take control", "mission", {"name": "MANUAL"}), ("Stop", "stop", {})]),
    _c("low_batt", "4 Rolling base", "Low-battery fault stop",
       "In AUTO, a pack under 21 V for 1 s gives a ramped stop and a latched FAULT_STOP.",
       ["Bench supply set to 22 V, current limited", "Wheels off the floor"],
       ["Start a task that puts the MCU in AUTO (Go To, or Program).",
        "Lower the supply to 20.5 V for 2 s, then back to 23 V.",
        "Press STOP to clear it."],
       "FAULT_STOP and LOW_BATT show in the status bar, the base stops and stays stopped until STOP with the pack "
       "≥ 22.2 V.",
       [("trip_v", "trip voltage", "V")], ["mcu.v_bus", "mcu.mode", "mcu.faults"],
       [("Stop (clears it)", "stop", {})], sim=False),
    _c("rc_failsafe", "4 Rolling base", "RC override and failsafe",
       "The transmitter's MANUAL switch beats the computer, and losing the transmitter releases the sticks.",
       ["Transmitter bound and on", "Wheels off the floor"],
       ["Start any task from the console.", "Flip channel 5 (MANUAL) up: the sticks drive, the console is ignored.",
        "Flip it down, then switch the transmitter off."],
       "RC takes over at once (the control chip shows RC); with the transmitter off the MCU ignores the sticks and "
       "the base ramps to rest.",
       [], ["rc.ok", "rc.manual", "mcu.mode"], [], sim=False),
    _c("odometry", "5 Navigation", "Odometry: straight line and spin",
       "The wheel odometry and the IMU heading agree with a tape measure (the calibrations save the correction).",
       ["2.5 m of clear floor, a tape on the floor", "Navigation not required"],
       ["Run the straight-line calibration, measure how far it really went.",
        "Run the spin calibration, check the final heading against a mark."],
       "Odometry within 2% of the tape, heading within 2° after three turns (est: tune from the results).",
       [("line_err_pct", "distance error", "%"), ("spin_err_deg", "heading error", "°")],
       ["pose.x", "pose.y", "pose.yaw"],
       [("Straight line", "calib_start", {"proc": "odom_line"}), ("Spin", "calib_start", {"proc": "odom_spin"})]),
    _c("lidar_self", "5 Navigation", "Lidar self-test (automatic)",
       "The robot checks its lidar at every boot (camera_snap, its own service): it opens the serial port, reads the "
       "model and health, spins it, counts a few turns and stops it again. The preflight's lidar row uses the result "
       "whenever no ROS scan comes, so the console says whether the lidar or its driver is missing.",
       [], ["Nothing to do: it ran at boot. Press Test again after replugging or moving the lidar (it spins for a few "
            "seconds; not while the ROS lidar driver has the port)."],
       "Passed: the model, health 'good', ~10 turns/s (A2M12; the bench A2M8 ~12 at PWM 660) and a full turn of samples.",
       [], ["lidar_test.ok", "lidar_test.hz", "lidar_test.samples"], [], sim=False, auto="lidar"),
    _c("lidar_frame", "5 Navigation", "Lidar and frames",
       "The scan lines up with the walls as the robot turns (laser frame, mounting yaw).",
       ["A room with straight walls", "SLAM running (Map)"],
       ["Open Map and watch the red scan on the walls.", "Turn slowly on the spot by hand (Drive)."],
       "The scan stays on the walls while turning; no ghost walls or rotation lag.",
       [("yaw_err_deg", "visible yaw offset", "°")], ["localization.level"],
       [("Take control", "mission", {"name": "MANUAL"})]),
    _c("arm_stow", "5 Picking", "Arm poses and stow path",
       "Every named pose and the way between them clear the robot (checked waypoints), before any floor grasp.",
       ["Arm powered, the robot parked", "Hands clear of the arm"],
       ["Move through Ready, High, Over the bin and back to Stow.", "Open and close the gripper."],
       "No collision or servo error; it ends folded; temperatures stay low.",
       [("max_temp", "hottest servo", "°C")], ["arm.torque", "arm.ok", "arm.temps"],
       [("Ready", "arm", {"do": "pose", "name": "ready"}), ("High", "arm", {"do": "pose", "name": "high"}),
        ("Over the bin", "arm", {"do": "pose", "name": "drop"}), ("Stow", "arm", {"do": "pose", "name": "stow"}),
        ("Open", "arm", {"do": "gripper", "gap_mm": 70}), ("Close", "arm", {"do": "gripper", "gap_mm": 0})]),
    _c("camera", "5 Picking", "D435 depth and colour",
       "The riser D435 sees the floor from about 0.31 m and its colour lines up with its depth.",
       ["A toy on the floor 0.5 m in front"],
       ["Open See and check the depth and colour streams.", "Use the arm calibration (Calibrate) for the camera "
                                                           "on the arm."],
       "Both streams run at their rate; the toy shows in depth at the right range.",
       [("range_m", "toy range read", "m")], ["vision.images"], []),
    _c("face_audio", "5 Picking", "Face, speaker and microphone",
       "The face shows, the speaker is heard and the microphone hears its name.",
       ["Room quiet enough to talk"],
       ["Press Say a line.", "Say \"Omni, hello\" and check the Face tab heard it."],
       "The line is heard clearly; the voice command shows in the Face tab.",
       [], [], [("Say a line", "say", {"text": "Testing, one two three.", "expression": "happy"})]),
)
BRINGUP_IDS = tuple(c["id"] for c in BRINGUP)
RESULTS = ("pass", "fail", "skip")
KEEP = 20                          # results kept per check


def record(results: dict, a: dict, wall: float, config: dict) -> dict:
    """Validate one outcome and add it to results (id -> [records], the newest last). Returns the record."""
    cid = str(a.get("id", ""))
    if cid not in BRINGUP_IDS:
        raise ValueError(f"test: one of {', '.join(BRINGUP_IDS)}")
    res = str(a.get("result", ""))
    if res not in RESULTS:
        raise ValueError(f"result: one of {', '.join(RESULTS)}")
    c = BRINGUP[BRINGUP_IDS.index(cid)]
    keys = {m["key"] for m in c["measure"]}
    measured = {}
    for k, v in (a.get("measured") or {}).items():
        if k not in keys:
            raise ValueError(f"measured: {k} isn't one of this check's numbers ({', '.join(sorted(keys)) or 'none'})")
        if v in (None, ""):
            continue
        v = float(v)
        if not math.isfinite(v):
            raise ValueError(f"measured {k}: not a number")
        measured[k] = v
    pre = [str(p) for p in a.get("pre") or [] if str(p) in c["pre"]]
    rec = dict(id=cid, result=res, measured=measured, pre=pre, notes=str(a.get("notes", ""))[:500],
               t=round(wall, 1), config=config)
    results.setdefault(cid, []).append(rec)
    results[cid] = results[cid][-KEEP:]
    return rec
