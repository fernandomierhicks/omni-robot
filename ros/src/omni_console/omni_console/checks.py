"""The Test workspace's robot side, ROS-free: the preflight (automatic, nothing moves) and the bring-up checks (guided,
attended, one at a time: design.md 8). missions.py serves both through /console/command (verbs preflight, tests,
test_record), so the page, a script or an AI client can run them.

Preflight: preflight(status, fw) reads the status the node is about to publish (mission_node.publish_status: the MCU,
the RC, localization, the arm, the computer, Wi-Fi) and returns one row per check:
  {id, label, state: ok | warn | fail | na, detail, blocks}
state "fail" with blocks set refuses the features named there (missions.py gate()), with the check's detail as the
reason. Only failures that would make the task fail anyway block (an MCU that is silent, latched in a fault stop or
speaks another protocol, a flat pack, a lost localization, an unhealthy arm); everything else only warns. A missing
input is "na", never "fail": the console doesn't refuse on what it can't see.

Bring-up: BRINGUP is the catalog. Each check says what it proves, the preconditions the operator ticks, the steps,
what to expect, the numbers to write down, the status fields to watch live, and buttons for the console verbs that
help (every one an ordinary verb: nothing here moves the robot by itself). test_record stores the outcome with the
robot's configuration (Results), in bringup.json and the session log. The thresholds are estimates until the real
robot has been measured (design.md: VERIFY).
"""

from __future__ import annotations

import math

STATES = ("ok", "warn", "fail", "na")
MOTION = "motion"                  # blocks: every feature that moves the base or the arm
BATT_STOP_V = 21.0                 # the firmware's low-battery stop (design.md 2: provisional, 6S)
BATT_WARN_V = 22.2                 # the fault stop's re-arm level: below it, a stop latches until the pack is above
MCU_SILENT_S = 1.0                 # s without MCU state: the link is down
FAULTS_BLOCK = ("ESTOP", "FAULT_STOP", "LOW_BATT", "MOTOR_ERR", "MOTOR_SILENT")
ARM_HOT_C = 65.0                   # servo temperature worth a warning (est, STS3215 class)
CPU_HOT_C = 85.0                   # the robot computer's hottest sensor (est)
WIFI_WEAK_DBM = -72.0
SYNC_SLOW_MS = 5.0                 # MCU clock-sync round trip (est): above it event times get worse
NAV_FEATURES = ("goto", "patrol", "explore")
LIDAR_FEATURES = NAV_FEATURES + ("follow", "message")      # what a silent lidar refuses (Nav2, the people)
LIDAR_SILENT_S = 1.0               # s without a scan: the lidar is down
LIDAR_SLOW_HZ = 5.0                # the A2M12 turns at ~10 Hz: slower, its motor or its USB link is struggling
TILT_WARN_DEG = 15.0               # the base tilted this much while the IMU reports: a ramp, a step, or tipped
ARM_FEATURES = ("tidy", "arm")


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
                                                        "Teensy plugged in?", (MOTION,)))
    elif (mcu.get("age") or 0.0) > MCU_SILENT_S:
        out.append(_row("mcu_link", "MCU link", "fail", f"The MCU has been silent for {mcu['age']:.0f} s (USB link "
                                                        "down); it stops the robot by itself", (MOTION,)))
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
                        "drops every command. Build and flash the firmware from this checkout", (MOTION,)))
    elif fw.get("expected") and f.get("build") and f["build"] != fw["expected"]:
        out.append(_row("firmware", "Firmware version", "warn",
                        f"The MCU runs {f['build']}, this software expects {fw['expected']}: rebuild and flash"))
    else:
        out.append(_row("firmware", "Firmware version", "ok", f"build {f.get('build') or '?'}, protocol "
                                                              f"{f.get('protocol')}"))

    if mcu is not None:
        names = [x.get("name") for x in mcu.get("faults") or [] if x.get("name") != "IMU_SILENT"]   # its own row
        hard = [n for n in names if n in FAULTS_BLOCK]
        if mcu.get("mode") == "ESTOP" and "ESTOP" not in hard:
            hard.append("ESTOP")
        if hard:
            out.append(_row("faults", "MCU faults", "fail", f"{', '.join(hard)}: a person has to clear it "
                                                            "(STOP disarms; FAULT_STOP also needs the pack ≥ 22.2 V)",
                            (MOTION,)))
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
        imu_silent = any(x.get("name") == "IMU_SILENT" for x in mcu.get("faults") or [])
        roll, pitch = mcu.get("roll"), mcu.get("pitch")
        if imu_silent:
            out.append(_row("imu", "IMU", "warn", "The BNO085 stopped reporting: heading from the wheels only "
                                                  "(odometry drifts in turns)"))
        elif roll is None or pitch is None:
            out.append(_row("imu", "IMU", "na", "No attitude in the MCU's state"))
        else:
            tilt = math.degrees(max(abs(roll), abs(pitch)))
            out.append(_row("imu", "IMU", "warn" if tilt > TILT_WARN_DEG else "ok",
                            f"tilted {tilt:.0f} deg: a ramp, a step, or tipped?" if tilt > TILT_WARN_DEG else
                            f"roll {math.degrees(roll):+.1f} deg, pitch {math.degrees(pitch):+.1f} deg"))

    lid = s.get("lidar")
    if lid is None:
        out.append(_row("lidar", "Lidar", "na", "No scan yet (the A2M12's driver not running, or still starting)"))
    elif lid["age"] > LIDAR_SILENT_S:
        out.append(_row("lidar", "Lidar", "fail", f"No scan for {lid['age']:.0f} s: no navigation, no people "
                                                  "(USB cable, the motor, the driver)", LIDAR_FEATURES))
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
    """The status's short form: counts, and the worst rows' labels (the page asks for the rest)."""
    n = {k: sum(1 for r in rows if r["state"] == k) for k in STATES}
    bad = [r["label"] for r in rows if r["state"] == "fail"] or [r["label"] for r in rows if r["state"] == "warn"]
    return dict(n, worst=bad[:3])


def blocking(rows: list | None, feature: str, moves: bool) -> list:
    """The failing rows that refuse this feature (moves: it drives the base or the arm)."""
    return [r for r in rows or () if r["state"] == "fail"
            and (feature in r["blocks"] or (moves and MOTION in r["blocks"]))]


# -- bring-up (design.md 8): guided, attended checks, in the order the build meets them ---------------------------------
def _c(id_, step, title, why, pre, steps, expect, measure=(), watch=(), actions=(), sim=True):
    return dict(id=id_, step=step, title=title, why=why, pre=list(pre), steps=list(steps), expect=expect,
                measure=[dict(key=k, label=lbl, unit=u) for k, lbl, u in measure], watch=list(watch),
                actions=[dict(label=lbl, verb=v, args=a) for lbl, v, a in actions], sim=sim)


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
