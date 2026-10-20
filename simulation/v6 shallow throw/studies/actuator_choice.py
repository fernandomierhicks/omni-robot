"""Actuator choice with cost first, every actuator class (design.md 24).

Reads the shared actuator database (omni-robot/database, or $ACTUATOR_DB), checks each candidate against this robot's
requirement records (from the sim's duty, design.md 23.5), ranks by the cost of the complete function (motor + its
share of a controller + power conversion), joins the sim results (motor_study.json, feed_motor.json) and prints the
ranked tables and the two motor + controller BOMs (cheapest compliant, preferred integrated).

  python actuator_choice.py            # writes results/actuator_choice.md
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DB_DIR = Path(os.environ.get("ACTUATOR_DB", HERE.parents[2] / "database"))
sys.path.insert(0, str(DB_DIR))
import actuator_db as adb  # noqa: E402

RESULTS = HERE / "results"
V7_LOW, V7_FULL = 22.4, 29.4          # V: 7S at 10% charge under load (design.md 10), full
V6_LOW, V6_FULL = 19.2, 25.2          # 6S, same per-cell numbers

# Drive speed: firmware adult mode peaks at 376 rpm (2 m/s along a diagonal, motor_study A); physics limits reach 406.
REQ = {
    "drive": dict(
        r7=adb.Requirement("drive wheel (7S)", 4, speed_rpm=376, v_supply_min_V=V7_LOW, v_supply_max_V=V7_FULL,
                           torque_peak_Nm=1.3, torque_rms_Nm=0.36, feedback_required=True, mass_max_kg=0.4),
        r6=adb.Requirement("drive wheel (6S)", 4, speed_rpm=376, v_supply_min_V=V6_LOW, v_supply_max_V=V6_FULL,
                           torque_peak_Nm=1.3, torque_rms_Nm=0.36, feedback_required=True, mass_max_kg=0.4)),
    "feed": adb.Requirement("feed wheel (12 V rail)", 2, speed_rpm=764, torque_at_speed_Nm=0.06, v_supply_min_V=12.0,
                            feedback_required=True),
    "tilt": adb.Requirement("tilt", 1, speed_rpm=6.0 / adb.RPM, torque_peak_Nm=3.0, torque_rms_Nm=1.05,
                            feedback_required=True),
}

# (part, controller, motors per controller, extra USD per actuator, pack, firmware effort, sim key)
DRIVE = [
    ("pololu-37d-24v-19-enc", "pololu-drv8874-4035", 1, 0, "7S", "medium: PWM + encoder + PID on the MCU", "X4 x Pololu 37D, 7S 10%"),
    ("dji-m2006-p36", "dji-c610", 1, 0, "6S", "medium: speed loop on the MCU, current over CAN", "X4 x M2006, 6S 10%"),
    ("dji-m2006-p36", "dji-c610", 1, 0, "7S", "medium: as above; C610 at 29.4 V unverified", "X4 x M2006, 7S 10%"),
    ("pololu-37d-24v-19-enc", "basicmicro-roboclaw-2x7a", 2, 0, "7S", "low: RoboClaw runs the loops", "X4 x Pololu 37D, 7S 10%"),
    ("steadywin-gim6010-8", None, 1, 0, "7S", "low: CAN setpoints", "X4 x GIM6010-8"),
    ("maytech-mto5065-70-ha", "odesc-3.6-dual", 2, 0, "7S", "high: ODrive Hall setup, belt reduction", None),
    ("cubemars-ak40-10-v3", None, 1, 0, "7S", "low: CAN setpoints", "X4 x AK40-10, 7S 10%"),
    ("waveshare-ddsm115", None, 1, 0, "7S", "low: RS485 setpoints", None),
    ("makerbase-mks-servo42d-can", None, 1, 0, "6S", "low: CAN setpoints", None),
    ("damiao-dm-j4310-2ec", None, 1, 0, "7S", "low: CAN setpoints", None),
]
FEED = [
    ("pololu-25d-12v-9.7-hp-enc", "pololu-drv8874-4035", 1, 0, "medium: PWM + encoder + PID on the MCU", "Pololu-25D-9.7"),
    ("iflight-gm3506-as5048a", "simplefoc-mini-v1", 1, 0, "high: FOC + encoder on the MCU", "GM3506"),
    ("iflight-gm2804-as5048a", "simplefoc-mini-v1", 1, 0, "high: FOC + encoder on the MCU", "GM2804"),
    ("lk-ms4005-v3", None, 1, 0, "low: CAN setpoints", "LK-MS4005"),
    ("pololu-25d-12v-9.7-hp-enc", "basicmicro-roboclaw-2x7a", 2, 0, "low: RoboClaw runs the loops", "Pololu-25D-RoboClaw"),
    ("lk-mf4005-v2", None, 1, 0, "low: CAN setpoints", "LK-MF4005"),
    ("pololu-micro-30-hpcb-12v-enc", "pololu-drv8874-4035", 1, 0, "medium", "Pololu-HPCB-30"),
    ("pololu-20d-25-12v", "pololu-drv8874-4035", 1, 0, "medium", None),
]
TILT = [   # servos (the 12 V rail is shared with the feed; the DS3235 needs its own BEC)
    ("dsservo-ds3235", None, 1, 10, "low: PWM, no feedback", "DS3235 at 8.4 V (3.4 N m, 9.5 rad/s)"),
    ("feetech-sts3215", None, 1, 0, "low: TTL bus", "Feetech STS3215 at 12 V (2.9 N m, 4.7 rad/s)"),
    ("feetech-sts3250", None, 1, 0, "low: TTL bus", "Feetech STS3250 at 12 V (4.9 N m, 7.9 rad/s)"),
    ("lk-mg4010e-i10-v3", None, 1, 0, "low: CAN (+ DG40E driver, price VERIFY)", "LK MG4010E-i10 at 24 V (4.5 N m, 27 rad/s)"),
    ("dynamixel-xm430-w210-t", None, 1, 0, "low: TTL bus", None),
]
FLY = [("tmotor-f90-2806.5-1300", "F90-2806.5-1300"), ("iflight-xing-2806.5-1300", "XING-2806.5-1300"),
       ("generic-2207-1750", "2207-1750"), ("generic-3115-900", "3115-900")]
ESCS = ["pilotix-65a-am32-8s", "sequre-blueson-a2-70a-8s", "tbs-lucid-8s-60a-4in1"]
BUCK = "generic-buck-12v-3a"


def money(x):
    return "?" if x is None else f"${x:,.0f}" if x >= 100 else f"${x:,.2f}"


def verdict_cell(checks):
    fails = [c.name for c in checks if c.status == "FAIL"]
    unk = [c.name for c in checks if c.status == "UNKNOWN"]
    if fails:
        return "FAIL: " + ", ".join(fails)
    return "PASS" + (f" (unknown: {', '.join(unk)})" if unk else "")


def sim_drive(ms, key):
    if key is None or "B" not in ms:
        return "", None
    out = []
    for lim in ("firmware", "physics"):
        rs = [x for x in ms["B"] if x["base"] == key and x["limits"] == lim]
        ts = [t for x in rs for t in x["tosses"]]
        if ts:
            out.append(f"{100 * sum(t['caught'] for t in ts) / len(ts):.0f}%")
    rs = [x for x in ms["B"] if x["base"] == key and x["limits"] == "firmware"]
    t_ss = max((x.get("t_steady", float("nan")) for x in rs), default=None)
    return " / ".join(out), t_ss


def drive_table(db, ms):
    rows = []
    for pid, cid, share, extra, pack, effort, key in DRIVE:
        m, c = db.motors[pid], db.controllers.get(cid) if cid else None
        req = REQ["drive"]["r6" if pack == "6S" else "r7"]
        ch = adb.check(m, req, c, share, extra)
        if c is not None:
            ch += [adb.Check("compat", "FAIL", i) for i in adb.compatible(m, c, req.v_supply_max_V)
                   if not i.startswith("note:") and "unknown" not in i and "set a current limit" not in i]
        cost = adb.function_cost(m, c, share, extra)
        sim, t_ss = sim_drive(ms, key)
        if t_ss is not None and not math.isnan(t_ss):
            ok = t_ss <= (m.get("t_winding_max_C") or 100) - 20
            ch.append(adb.Check("sim steady-state winding", "PASS" if ok else "FAIL", t_ss, (m.get("t_winding_max_C") or 100) - 20))
        rows.append((cost, f"{m['name']}" + (f" + {c['name']}" + (f" (1 per {share})" if share > 1 else "") if c else ""),
                     pack, m.get("integration") if not (c and c.get("kind") == "smart_controller") else "smart (controller)",
                     effort, m.get("mass_kg"), sim, t_ss, verdict_cell(ch), pid, cid, share))
    return sorted(rows, key=lambda r: (r[0] is None, r[0] or 0))


def feed_table(db, fm):
    rows = []
    for pid, cid, share, extra, effort, key in FEED:
        m, c = db.motors[pid], db.controllers.get(cid) if cid else None
        ch = adb.check(m, REQ["feed"], c, share, extra)
        sim = ""
        if fm and key:
            b = [x for x in fm["bench"] if x["motor"] == key]
            h = next((x for x in fm["hold"] if x["motor"] == key), None)
            if b and h:
                slip = max(x["slip"] for x in h["trials"]) * 0.02 * 1000
                sim = f"fed {sum(x['exit'] is not None for x in b)}/{len(b)}, back-driven {slip:.1f} mm"
                ch.append(adb.Check("sim feed", "PASS" if all(x["exit"] is not None for x in b) else "FAIL"))
                ch.append(adb.Check("sim hold <= 5 mm", "PASS" if slip <= 5.0 else "FAIL", slip, 5.0))
        cost = adb.function_cost(m, c, share, extra)
        rows.append((cost, m["name"] + (f" + {c['name']}" + (f" (1 per {share})" if share > 1 else "") if c else ""),
                     m.get("integration") if not (c and c.get("kind") == "smart_controller") else "smart (controller)",
                     effort, m.get("mass_kg"), sim, verdict_cell(ch), pid, cid, share))
    return sorted(rows, key=lambda r: (r[0] is None, r[0] or 0))


def tilt_table(db, ms):
    rows = []
    for pid, cid, share, extra, effort, key in TILT:
        p = db.servos.get(pid) or db.motors[pid]
        ch = adb.check(p, REQ["tilt"], None, share, extra)
        if pid == "dsservo-ds3235":
            ch.append(adb.Check("datasheet voltage", "FAIL", 8.4, 7.4, "sim used the 8.4 V listing figures"))
        sim = ""
        if key and "D" in ms:
            ts = [t for x in ms["D"] if x["base"] == key for t in x["tosses"]]
            sim = f"{100 * sum(t['caught'] for t in ts) / len(ts):.0f}%" if ts else ""
        rows.append((adb.function_cost(p, None, share, extra), p["name"], effort, p.get("mass_kg"), sim, verdict_cell(ch), pid))
    return sorted(rows, key=lambda r: (r[0] is None, r[0] or 0))


def fly_table(db, ms):
    rows = []
    for pid, key in FLY:
        m = db.motors[pid]
        c = [x for x in ms.get("C", []) if x["motor"] == key and x.get("battery", "7S1P-P50B") == "7S1P-P50B"]
        c6 = [x for x in ms.get("C", []) if x["motor"] == key and x.get("battery") == "6S1P-P50B"]
        s = f"{c[0]['spinup_3000']:.2f} / {c[0]['spinup_4500']:.2f} s" if c else ""
        s6 = f"{c6[0]['spinup_3000']:.2f} / {c6[0]['spinup_4500']:.2f} s" if c6 else ""
        rows.append((m.get("price_usd"), m["name"], m.get("mass_kg"), s, s6, pid))
    return sorted(rows, key=lambda r: r[0] or 0)


def bom(db, items):
    """items: (label, part id, qty, unit price override or None) -> (lines, total)."""
    lines, tot = [], 0.0
    for label, pid, qty in items:
        p = db.get(pid)
        each = p.get("price_usd")
        tot += qty * each
        prov = p.prov("price_usd")
        lines.append(f"| {label} | {p['name']} | {qty} | {money(each)}{' (est)' if prov == 'est' else ''} | {money(qty * each)} |")
    return lines, tot


CHEAPEST = [("Drive", "dji-m2006-p36", 4), ("Drive ESC", "dji-c610", 4), ("Flywheels", "tmotor-f90-2806.5-1300", 2),
            ("Flywheel ESC", "pilotix-65a-am32-8s", 1), ("Tilt", "feetech-sts3250", 1),
            ("Feed", "pololu-25d-12v-9.7-hp-enc", 2), ("Feed driver", "pololu-drv8874-4035", 2), ("12 V rail", BUCK, 1)]
PREFERRED = [("Drive", "cubemars-ak40-10-v3", 4), ("Flywheels", "tmotor-f90-2806.5-1300", 2),
             ("Flywheel ESC", "pilotix-65a-am32-8s", 1), ("Tilt", "feetech-sts3250", 1),
             ("Feed", "pololu-25d-12v-9.7-hp-enc", 2), ("Feed controller", "basicmicro-roboclaw-2x7a", 1), ("12 V rail", BUCK, 1)]


def main():
    db = adb.load(DB_DIR)
    errs = adb.validate(db)
    if errs:
        print("database errors:\n" + "\n".join(errs))
        return 1
    ms = json.loads((RESULTS / "motor_study.json").read_text())
    fm = json.loads((RESULTS / "feed_motor.json").read_text()) if (RESULTS / "feed_motor.json").exists() else None
    L = ["# Actuator choice (design.md 24): generated by studies/actuator_choice.py", ""]
    L += ["## Drive wheel (per wheel; checks at 10% charge: 7S 22.4 V, 6S 19.2 V)", "",
          "| Cost / wheel | Option | Pack | Integration | Firmware | Mass kg | Sim firmware / physics (10% charge) | Session winding C (est) | Checks |",
          "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for cost, name, pack, integ, eff, mass, sim, t_ss, v, *_ in drive_table(db, ms):
        L.append(f"| {money(cost)} | {name} | {pack} | {integ} | {eff} | {mass or '?'} | {sim} | "
                 f"{'' if t_ss is None or math.isnan(t_ss) else f'{t_ss:.0f}'} | {v} |")
    L += ["", "## Feed wheel (per wheel, 12 V rail)", "",
          "| Cost / wheel | Option | Integration | Firmware | Mass kg | Sim (feed, worst back-drive) | Checks |",
          "| --- | --- | --- | --- | --- | --- | --- |"]
    for cost, name, integ, eff, mass, sim, v, *_ in feed_table(db, fm):
        L.append(f"| {money(cost)} | {name} | {integ} | {eff} | {mass or '?'} | {sim} | {v} |")
    L += ["", "## Tilt", "", "| Cost | Option | Firmware | Mass kg | Sim caught (firmware adult) | Checks |",
          "| --- | --- | --- | --- | --- | --- |"]
    for cost, name, eff, mass, sim, v, _ in tilt_table(db, ms):
        L.append(f"| {money(cost)} | {name} | {eff} | {mass or '?'} | {sim} | {v} |")
    L += ["", "## Flywheel motors (ESC: one 8S 4-in-1 board, 2 of 4 channels)", "",
          "| Cost | Motor | Mass kg | Spin-up to 3000 / 4500 rpm, 7S | Same, 6S |", "| --- | --- | --- | --- | --- |"]
    for cost, name, mass, s, s6, _ in fly_table(db, ms):
        L.append(f"| {money(cost)} | {name} | {mass} | {s} | {s6} |")
    L += ["", "| ESC | Price | 7S full |", "| --- | --- | --- |"]
    for e in sorted(ESCS, key=lambda i: db.controllers[i]["price_usd"]):
        c = db.controllers[e]
        L.append(f"| {c['name']} | {money(c['price_usd'])} ({c.prov('price_usd')}) | {c['ok_7s_full']} |")
    for title, items in (("Cheapest compliant set (6S pack)", CHEAPEST), ("Preferred integrated set (7S pack)", PREFERRED)):
        lines, tot = bom(db, items)
        L += ["", f"## {title}", "", "| Function | Part | Qty | Each | Total |", "| --- | --- | --- | --- | --- |"] + lines
        L.append(f"| **Total** | | | | **{money(tot)}** |")
    txt = "\n".join(L)
    (RESULTS / "actuator_choice.md").write_text(txt + "\n", encoding="utf-8")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
