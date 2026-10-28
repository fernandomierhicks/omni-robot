"""The console's status bar, ROS-free: battery, faults and states in words an operator can act on.

Battery % is an estimate from the pack voltage alone (7S Molicel P50B, light load; design.md 2): it reads low while
the motors pull hard and recovers at rest. The firmware's LOW_BATT fault (22.4 V, 3.2 V/cell) is the real limit.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

CELLS = 7
BATT_LOW_V = 22.4                  # firmware fw_params BATT_LOW: 3.2 V/cell
# (V/cell, %) at light load, Li-ion NMC 21700 (est, not measured on the P50B)
CELL_CURVE = ((3.00, 0), (3.30, 5), (3.45, 10), (3.55, 20), (3.62, 30), (3.68, 40), (3.74, 50), (3.80, 60),
              (3.88, 70), (3.96, 80), (4.06, 90), (4.20, 100))

FAULT_HINTS = {
    "LINK_LOST": "The MCU heard nothing from the Jetson for 0.5 s and stopped. It recovers when commands return.",
    "ESTOP": "The e-stop is pressed: power to the motors is cut. Release it, then start a mode again.",
    "BUMPER": "The bumper is pressed. Move the robot clear by hand.",
    "MOTOR_ERR": "A drive motor reports an error (Base tab). Usually over-temperature or under-voltage.",
    "MOTOR_SILENT": "A drive motor stopped answering on CAN: check its cable.",
    "LOW_BATT": "Battery below 22.4 V (3.2 V a cell): charge it now.",
    "IMU_SILENT": "The BNO085 stopped reporting: heading comes from the wheels only.",
    "FLY_TELEM": "A flywheel ESC stopped sending rpm: the thrower won't fire.",
}
MCU_MODE_HINTS = {
    "BOOT": "The MCU is starting.",
    "DISARMED": "Motors off. Start a mode to arm them.",
    "AUTO": "Armed: the Jetson drives, within the firmware's limits.",
    "MANUAL": "The RC transmitter has taken over (its switch). The console can't drive until it's switched back.",
    "ESTOP": "Emergency stop: motors unpowered.",
}
CATCH_TEXT = {
    "wait": "Waiting for a throw",
    "flight": "Ball in the air: moving under it",
    "settle": "Settling after the catch",
    "aim": "Aiming the return throw",
    "launch": "Throwing back",
    "return": "Going back to its spot",
}


def battery_pct(v_bus: float, cells: int = CELLS) -> float | None:
    if v_bus is None or v_bus <= 0:
        return None
    v = v_bus / cells
    if v <= CELL_CURVE[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(CELL_CURVE, CELL_CURVE[1:]):
        if v <= v1:
            return p0 + (p1 - p0) * (v - v0) / (v1 - v0)
    return 100.0


def faults(names) -> list:
    return [dict(name=n, hint=FAULT_HINTS.get(n, "")) for n in names]


def parse_wireless(text: str) -> dict | None:
    """/proc/net/wireless: the first interface's link quality and signal (dBm), or None (no Wi-Fi, e.g. in WSL)."""
    for line in text.splitlines()[2:]:
        parts = line.split()
        if len(parts) >= 4 and parts[0].endswith(":"):
            try:
                return dict(iface=parts[0][:-1], quality=float(parts[2].rstrip(".")), dbm=float(parts[3].rstrip(".")))
            except ValueError:
                continue
    return None


def mcu_summary(m: dict, age: float) -> dict:
    """m: the McuState fields the console uses (mission_node fills it); age: s since the last STATE."""
    pct = battery_pct(m.get("v_bus"))
    return dict(mode=m["mode"], mode_hint=MCU_MODE_HINTS.get(m["mode"], ""), profile=m["profile"],
                faults=faults(m["faults"]), v_bus=round(m["v_bus"], 2),
                battery_pct=None if pct is None else round(pct), battery_low=m["v_bus"] < BATT_LOW_V,
                age=round(age, 2), dm_err=list(m["dm_err"]), fly_rpm=[round(r) for r in m["fly_rpm"]],
                tilt=round(m["tilt"], 3), ball_held=m["ball_held"], drive_mode=m["drive_mode"],
                **{k: round(m[k], 3) if isinstance(m[k], float) else m[k] for k in EXTRA if k in m})


# McuState fields the Base and Thrower tabs show, passed through as they are
EXTRA = ("fly_target_rpm", "fly_ready", "tilt_cmd", "feed_state", "feed_exit_beam", "fire_seq_done", "fire_rejected",
         "vx", "vy", "wz", "slip", "roll", "pitch", "sync_rtt_ms")


def clean(v):
    """NaN and infinities to None, all the way down: JSON has no NaN, and the page's JSON.parse refuses the whole
    message for one."""
    if isinstance(v, float):
        return v if v == v and v not in (float("inf"), float("-inf")) else None
    if isinstance(v, dict):
        return {k: clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [clean(x) for x in v]
    return v


def catch_summary(c: dict) -> dict:
    """The status's catch part. The last throw's path stays out (it goes once, on /console/throw)."""
    last = {k: v for k, v in (c.get("last") or {}).items() if k != "path"}
    return clean(dict(state=c["state"], text=CATCH_TEXT.get(c["state"], c["state"]), tosses=c["tosses"],
                      catches=c["catches"], rate=round(100.0 * c["catches"] / c["tosses"]) if c["tosses"] else None,
                      last=last, ball=c.get("ball"), target=c.get("target")))


def firmware_build_id(core) -> str | None:
    """firmware/fw_build_id.py's id of a copy of lib/omni_fw (None if absent): what the MCU should report."""
    core = Path(core)
    files = sorted(core.glob("*.h")) + sorted(core.glob("*.cpp"))
    if not files:
        return None
    h = hashlib.sha1()
    for p in files:
        h.update(p.name.encode())
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()[:8]


def system() -> dict:
    """The computer the robot code runs on (the Jetson; WSL in the sim): load and the hottest thermal zone."""
    try:
        load = os.getloadavg()[0]
    except (OSError, AttributeError):                    # AttributeError: Windows
        load = None
    temps = []
    for z in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            temps.append(int(z.read_text().strip()) / 1000.0)
        except (OSError, ValueError):
            pass
    return dict(load=None if load is None else round(load, 2), cpus=os.cpu_count(),
                temp_c=round(max(temps), 1) if temps else None)
