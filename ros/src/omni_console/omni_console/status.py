"""The console's status bar, ROS-free: battery, faults and states in words an operator can act on.

Battery % is an estimate from the pack voltage alone (7S Molicel P50B, light load; design.md 2): it reads low while
the motors pull hard and recovers at rest. The firmware's LOW_BATT fault (22.4 V, 3.2 V/cell) is the real limit.
"""

from __future__ import annotations

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
                tilt=round(m["tilt"], 3), ball_held=m["ball_held"], drive_mode=m["drive_mode"])


def catch_summary(c: dict) -> dict:
    return dict(state=c["state"], text=CATCH_TEXT.get(c["state"], c["state"]), tosses=c["tosses"],
                catches=c["catches"], rate=round(100.0 * c["catches"] / c["tosses"]) if c["tosses"] else None,
                last=c.get("last") or {}, ball=c.get("ball"), target=c.get("target"))
