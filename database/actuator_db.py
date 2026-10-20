"""Actuator database: motors, servos, controllers and their sources, kept as plain CSV.

The CSV files are the data; this module is the schema, the validator, the derived quantities and the queries.
Shared by every robotics project (it lives in omni-robot/database for now and will move one level up), so it
imports nothing project-specific: the sim exports take the project's catalog module as an argument.

  python actuator_db.py validate            # schema, units, provenance, sources, consistency
  python actuator_db.py gaps [--csv]        # empty key fields per part (writes gaps.csv with --csv)
  python actuator_db.py show ID             # one part, with derived quantities
  python actuator_db.py schema              # the column tables for README.md

Conventions (README.md has the full schema):
  - SI units, written into the column name (torque_stall_Nm, speed_noload_rpm, mass_kg, price_usd ...).
  - Motor speeds and torques are at the OUTPUT shaft (after any gearbox); kt, ke, kv, r and inertia are motor side.
  - Blank = unknown. Never fill a number you don't have; mark estimates through the provenance columns.
  - Provenance: prov_default applies to every filled value; prov_overrides names the exceptions,
    e.g. "est: rotor_inertia_kgm2, gear_eff; reseller: kt_Nm_per_A".
  - meas_* columns hold bench data; where filled, they win over the datasheet value (sim-to-real).
"""

from __future__ import annotations

import csv
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
V_7S_FULL = 29.4                       # V: a full 7S Li-ion pack (4.2 V per cell)
V_6S_FULL = 25.2
PROVENANCE = ("datasheet", "reseller", "forum", "est", "measured")
RPM = 2 * math.pi / 60                 # rad/s per rpm
KGCM = 0.0980665                       # N m per kg cm


# ---------------------------------------------------------------------------------------------------------------
# Schema

@dataclass(frozen=True)
class Col:
    name: str
    type: str                # str, float, int, bool, date, list (";"-separated), enum
    desc: str
    enum: tuple = ()
    key: bool = False        # listed by the gaps report when blank


def _cols(*rows) -> list[Col]:
    return [Col(*r) if not isinstance(r, Col) else r for r in rows]


YESNO = ("yes", "no", "partly", "VERIFY")

COMMON_HEAD = _cols(
    ("id", "str", "Unique slug, maker-part (e.g. pololu-37d-24v-19-enc). Never reuse or rename: projects refer to it"),
    ("manufacturer", "str", "Maker"),
    ("part_number", "str", "Maker's part number or SKU"),
    ("name", "str", "Descriptive name"),
    ("vendor", "str", "Where the price comes from (shop)"),
)
COMMON_TAIL = _cols(
    Col("price_usd", "float", "Unit price in USD at quantity 1 (converted prices are est)", key=True),
    ("price_breaks", "str", "Quantity breaks 'qty:usd;qty:usd' (e.g. 5:56.07;25:51.59)"),
    ("availability", "str", "Stock seen on date_checked (in stock / out of stock / backorder / discontinued)"),
    Col("prov_default", "enum", "Provenance of every filled value unless overridden", PROVENANCE),
    ("prov_overrides", "str", "Exceptions: 'est: col, col; reseller: col' (column names of this table)"),
    ("source_ids", "list", "Source ids from sources.csv, ';'-separated"),
    ("date_checked", "date", "YYYY-MM-DD the values and price were last checked"),
    ("notes", "str", "Anything else: caveats, VERIFY items, why rejected"),
)

MOTOR_COLS = COMMON_HEAD + _cols(
    Col("kind", "enum", "Motor class",
        ("brushed_dc", "bldc_outrunner", "bldc_inrunner", "gimbal", "stepper", "closed_loop_stepper", "hub_motor",
         "integrated_actuator"), key=True),
    Col("integration", "enum", "What comes in the box: bare motor / +gearbox / +encoder / +gearbox+encoder / "
        "+driver (no loops) / smart (driver with its own closed loops)",
        ("bare", "gearbox", "encoder", "gearbox_encoder", "driver", "smart"), key=True),
    Col("commutation", "enum", "brushed, sensorless BLDC, Hall BLDC, encoder FOC, open- or closed-loop stepping",
        ("brushed", "sensorless", "hall", "encoder_foc", "stepper_open", "stepper_closed"), key=True),
    ("sensor_type", "str", "Built-in sensor: hall, magnetic, optical, none"),
    Col("sensor_absolute", "enum", "Absolute (yes) or incremental (no) position", YESNO),
    ("sensor_cpr_motor", "float", "Counts per motor revolution (quadrature counts where it applies)"),
    Col("sensor_cpr_output", "float", "Counts per output revolution", key=True),
    Col("sensor_multiturn", "enum", "Keeps the turn count over power cycles", YESNO),
    Col("gear_type", "enum", "Reduction", ("none", "spur", "planetary", "worm", "harmonic", "cycloidal", "belt",
                                           "spur_helical")),
    Col("gear_ratio", "float", "Reduction ratio (1 = direct drive)", key=True),
    ("gear_eff", "float", "Gearbox efficiency 0-1"),
    ("backlash_arcmin", "float", "Output backlash, arc minutes"),
    Col("backdrivable", "enum", "Can a load turn the unpowered output", YESNO),
    Col("v_rated_V", "float", "Rated supply voltage", key=True),
    ("v_min_V", "float", "Lowest allowed supply voltage"),
    Col("v_max_V", "float", "Highest allowed supply voltage", key=True),
    ("kv_rpm_per_V", "float", "Speed constant, motor side"),
    ("kt_Nm_per_A", "float", "Torque constant, motor side (per phase amp / q-axis amp for BLDC)"),
    ("ke_Vs_per_rad", "float", "Back-EMF constant, motor side"),
    ("r_ohm", "float", "Winding resistance (see r_kind)"),
    Col("r_kind", "enum", "How r_ohm is measured", ("phase", "line_line", "terminal", "unclear")),
    ("inductance_H", "float", "Winding inductance (same convention as r_kind)"),
    ("pole_pairs", "int", "Rotor pole pairs"),
    Col("v_test_V", "float", "Voltage of the no-load / stall / rated points below", key=True),
    Col("speed_noload_rpm", "float", "Output no-load speed at v_test_V", key=True),
    ("current_noload_A", "float", "No-load current at v_test_V"),
    ("torque_stall_Nm", "float", "Output stall torque at v_test_V"),
    ("current_stall_A", "float", "Stall current at v_test_V"),
    Col("stall_extrapolated", "enum", "The maker extrapolated the stall point (common for gearmotors)", YESNO),
    ("speed_rated_rpm", "float", "Output speed at the rated point"),
    Col("torque_rated_Nm", "float", "Output continuous (rated) torque", key=True),
    ("current_rated_A", "float", "Current at the rated point"),
    Col("torque_peak_Nm", "float", "Output peak torque (driver or gear limit)", key=True),
    ("current_peak_A", "float", "Peak current"),
    ("rotor_inertia_kgm2", "float", "Rotor inertia, motor side"),
    Col("mass_kg", "float", "Mass as sold (with encoder / driver if integrated)", key=True),
    ("diameter_m", "float", "Body diameter (or width for square frames)"),
    ("length_m", "float", "Body length without shaft"),
    ("shaft", "str", "Output shaft or flange (e.g. 6 mm D, 16 mm long)"),
    ("mounting", "str", "Mounting pattern (e.g. 6x M3 on PCD 31)"),
    ("t_winding_max_C", "float", "Maximum winding temperature"),
    ("r_th_KperW", "float", "Winding-to-ambient thermal resistance (steady state)"),
    ("thermal_notes", "str", "Duty limits, derating, time constants"),
    ("brush_life_h", "float", "Brush life, hours (brushed only)"),
    ("noise", "str", "Noise notes or dB"),
    ("ip_rating", "str", "Ingress protection"),
    Col("interface", "list", "none, pwm, dshot, can, can_fd, rs485, ttl_bus, uart, usb, i2c, step_dir, quadrature",
        key=True),
    ("control_modes", "list", "torque, speed, position, mit (impedance), voltage"),
    ("telemetry", "list", "position, speed, current, torque, temperature, voltage, errors"),
    ("protections", "list", "over_voltage, under_voltage, over_current, thermal, stall, comm_loss"),
    ("controller", "str", "Controller it needs: 'integrated', a controllers.csv id, or blank (any of its type)"),
) + COMMON_TAIL + _cols(
    ("meas_speed_noload_rpm", "float", "MEASURED output no-load speed at meas_v_V"),
    ("meas_current_noload_A", "float", "MEASURED no-load current"),
    ("meas_torque_stall_Nm", "float", "MEASURED stall torque (or locked-rotor torque at a stated current)"),
    ("meas_current_stall_A", "float", "MEASURED stall current"),
    ("meas_v_V", "float", "Voltage of the measured points"),
    ("meas_r_ohm", "float", "MEASURED resistance (r_kind convention)"),
    ("meas_kt_Nm_per_A", "float", "MEASURED torque constant, motor side"),
    ("meas_rotor_inertia_kgm2", "float", "MEASURED rotor inertia"),
    ("meas_backlash_arcmin", "float", "MEASURED backlash"),
    ("meas_r_th_KperW", "float", "MEASURED winding-to-ambient thermal resistance"),
    ("meas_mass_kg", "float", "MEASURED mass"),
    ("meas_date", "date", "Date of the bench test"),
    ("meas_notes", "str", "Bench setup, log file"),
)

SERVO_COLS = COMMON_HEAD + _cols(
    Col("kind", "enum", "smart_bus (serial bus with feedback), hobby_pwm (no feedback), industrial",
        ("smart_bus", "hobby_pwm", "industrial"), key=True),
    Col("v_test_V", "float", "Voltage of the torque / speed figures", key=True),
    ("v_min_V", "float", "Lowest supply voltage"),
    Col("v_max_V", "float", "Highest supply voltage", key=True),
    Col("torque_stall_Nm", "float", "Stall torque at v_test_V", key=True),
    ("torque_rated_Nm", "float", "Rated (continuous) torque"),
    Col("speed_s_per_60deg", "float", "No-load speed, seconds per 60 degrees, at v_test_V", key=True),
    ("speed_noload_rad_s", "float", "Same speed in rad/s (derived when blank: (pi/3) / s_per_60deg)"),
    ("current_stall_A", "float", "Stall current"),
    ("current_noload_A", "float", "No-load current"),
    ("gear_ratio", "float", "Gear ratio"),
    ("gear_material", "str", "metal, steel, plastic ..."),
    Col("feedback", "list", "position, speed, load, current, temperature, voltage (blank list = none)", key=True),
    ("position_bits", "float", "Position resolution, bits per turn"),
    Col("absolute", "enum", "Absolute position over power cycles", YESNO),
    ("range_deg", "float", "Servo travel in position mode"),
    Col("continuous_rotation", "enum", "Has a wheel / speed mode", YESNO),
    Col("bus", "str", "pwm, ttl_half_duplex, rs485, can ...", key=True),
    ("baud_max", "float", "Highest bus rate, bit/s"),
    Col("mass_kg", "float", "Mass", key=True),
    ("size_mm", "str", "L x W x H"),
    ("ip_rating", "str", "Ingress protection"),
) + COMMON_TAIL + _cols(
    ("meas_torque_stall_Nm", "float", "MEASURED stall torque"),
    ("meas_speed_noload_rad_s", "float", "MEASURED no-load speed"),
    ("meas_v_V", "float", "Voltage of the measurements"),
    ("meas_date", "date", "Date of the bench test"),
    ("meas_notes", "str", "Bench setup, log file"),
)

CONTROLLER_COLS = COMMON_HEAD + _cols(
    Col("kind", "enum", "h_bridge (brushed), esc (BLDC, RC style), foc_driver (power stage only), foc_controller "
        "(runs FOC + loops), smart_controller (brushed with its own loops), stepper_driver, mcu, dcdc",
        ("h_bridge", "esc", "foc_driver", "foc_controller", "smart_controller", "stepper_driver", "mcu", "dcdc"),
        key=True),
    Col("channels", "int", "Motors per board", key=True),
    Col("motor_types", "list", "brushed, bldc, stepper", key=True),
    ("power_stage", "str", "full_h_bridge, half_bridge, 3_phase, none"),
    Col("v_min_V", "float", "Lowest supply voltage", key=True),
    Col("v_max_V", "float", "Highest supply voltage", key=True),
    Col("ok_7s_full", "enum", "Takes a full 7S pack (29.4 V): yes (>= 31.4 V), marginal (29.4-31.4), no, VERIFY "
        "(unknown max). Checked against v_max_V by validate", ("yes", "marginal", "no", "VERIFY"), key=True),
    Col("i_cont_A", "float", "Continuous current per channel", key=True),
    ("i_peak_A", "float", "Peak current per channel"),
    ("current_sense", "str", "How current is sensed / reported"),
    ("current_regulation", "str", "Current limiting (cycle-by-cycle, FOC torque loop, none)"),
    ("encoder_inputs", "int", "Encoder inputs (total)"),
    ("encoder_type", "str", "quadrature, hall, spi_absolute, motor's own ..."),
    ("encoder_max_rate_Hz", "float", "Highest count rate"),
    Col("loops", "list", "none, current, speed_pid, position_pid, foc, autotune", key=True),
    Col("interface", "list", "pwm, dshot, can, can_fd, uart, packet_serial, usb, i2c, rc_pwm, analog, step_dir",
        key=True),
    ("logic_level_V", "str", "Logic levels"),
    ("telemetry", "list", "speed, position, current, voltage, temperature, errors"),
    ("protections", "list", "over_voltage, under_voltage, over_current, thermal, reverse_polarity, comm_loss"),
    ("regen", "str", "Regenerative braking handling"),
    ("size_mm", "str", "L x W (x H)"),
    ("mass_kg", "float", "Mass"),
    ("config_tool", "str", "Configuration software"),
) + COMMON_TAIL

SOURCE_COLS = _cols(
    ("id", "str", "Source id (S001 ...)"),
    Col("kind", "enum", "maker (maker's page or datasheet), reseller, forum, project (own docs / purchase records)",
        ("maker", "reseller", "forum", "project")),
    ("url", "str", "Link (or a repo path for project sources)"),
    ("supports", "str", "Which parts and values it supports"),
    ("date_checked", "date", "YYYY-MM-DD"),
    ("notes", "str", "Notes"),
)

TABLES = {"motors": MOTOR_COLS, "servos": SERVO_COLS, "controllers": CONTROLLER_COLS, "sources": SOURCE_COLS}


# ---------------------------------------------------------------------------------------------------------------
# Loading

class Part(dict):
    """One row: typed values (None = blank) keyed by column name, plus the table it came from."""

    table: str = ""

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError as e:
            raise AttributeError(k) from e

    def prov(self, col: str) -> str:
        """Provenance of one value: an override if the row names the column, else prov_default."""
        for src, cols in parse_overrides(self.get("prov_overrides") or "").items():
            if col in cols:
                return src
        return self.get("prov_default") or ""

    def meas(self, col: str):
        """The measured value if there is one, else the datasheet value (sim-to-real)."""
        m = self.get("meas_" + col)
        return m if m is not None else self.get(col)


def parse_overrides(s: str) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for chunk in filter(None, (c.strip() for c in s.split(";"))):
        src, _, cols = chunk.partition(":")
        out.setdefault(src.strip(), []).extend(c.strip() for c in cols.split(",") if c.strip())
    return out


def _parse(col: Col, raw: str):
    raw = raw.strip()
    if raw == "":
        return [] if col.type == "list" else None
    if col.type == "float":
        return float(raw)
    if col.type == "int":
        return int(float(raw))
    if col.type == "list":
        return [x.strip() for x in raw.split(";") if x.strip()]
    return raw


def _fmt(v) -> str:
    if v is None:
        return ""
    if isinstance(v, list):
        return ";".join(v)
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


@dataclass
class DB:
    motors: dict[str, Part] = field(default_factory=dict)
    servos: dict[str, Part] = field(default_factory=dict)
    controllers: dict[str, Part] = field(default_factory=dict)
    sources: dict[str, Part] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)       # parse errors met while loading

    def table(self, name: str) -> dict[str, Part]:
        return getattr(self, name)

    def get(self, pid: str) -> Part:
        for t in ("motors", "servos", "controllers", "sources"):
            if pid in self.table(t):
                return self.table(t)[pid]
        raise KeyError(pid)

    def parts(self):
        for t in ("motors", "servos", "controllers"):
            yield from self.table(t).values()


def load(folder: Path | str = HERE) -> DB:
    folder = Path(folder)
    db = DB()
    for name, cols in TABLES.items():
        path = folder / f"{name}.csv"
        bycol = {c.name: c for c in cols}
        with open(path, newline="", encoding="utf-8-sig") as f:
            rd = csv.DictReader(f)
            missing = [c.name for c in cols if c.name not in (rd.fieldnames or [])]
            extra = [c for c in (rd.fieldnames or []) if c not in bycol]
            if missing:
                db.errors.append(f"{name}.csv: missing columns {missing}")
            if extra:
                db.errors.append(f"{name}.csv: unknown columns {extra}")
            for n, row in enumerate(rd, start=2):
                p = Part()
                p.table = name
                for c in cols:
                    try:
                        p[c.name] = _parse(c, row.get(c.name) or "")
                    except ValueError:
                        db.errors.append(f"{name}.csv line {n} ({row.get('id')}): {c.name}={row.get(c.name)!r} "
                                         f"is not a {c.type}")
                        p[c.name] = None
                if not p.get("id"):
                    continue
                if p["id"] in db.table(name):
                    db.errors.append(f"{name}.csv: duplicate id {p['id']}")
                db.table(name)[p["id"]] = p
    return db


def write_table(db: DB, name: str, folder: Path | str = HERE):
    """Write one table back (column order from the schema), e.g. after a script fills measured values."""
    cols = TABLES[name]
    with open(Path(folder) / f"{name}.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([c.name for c in cols])
        for p in db.table(name).values():
            w.writerow([_fmt(p.get(c.name)) for c in cols])


# ---------------------------------------------------------------------------------------------------------------
# Validation

DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")


def ok_7s(v_max: float | None) -> str:
    if v_max is None:
        return "VERIFY"
    if v_max >= V_7S_FULL + 2.0:
        return "yes"
    return "marginal" if v_max >= V_7S_FULL else "no"


def validate(db: DB) -> list[str]:
    errs = list(db.errors)
    for name, cols in TABLES.items():
        names = {c.name for c in cols}
        for pid, p in db.table(name).items():
            where = f"{name}:{pid}"
            for c in cols:
                v = p.get(c.name)
                if c.type == "enum" and v is not None and v not in c.enum:
                    errs.append(f"{where}: {c.name}={v!r} not in {c.enum}")
                if c.type == "date" and v is not None and not DATE_RE.match(v):
                    errs.append(f"{where}: {c.name}={v!r} is not YYYY-MM-DD")
                if c.type in ("float", "int") and v is not None and v < 0 and c.name not in ():
                    errs.append(f"{where}: {c.name}={v} is negative")
            if name == "sources":
                continue
            if not re.match(r"^[a-z0-9][a-z0-9.\-]*$", pid):
                errs.append(f"{where}: id must be lowercase letters, digits, '-' and '.'")
            if p.get("prov_default") is None:
                errs.append(f"{where}: prov_default is blank")
            for src, ocols in parse_overrides(p.get("prov_overrides") or "").items():
                if src not in PROVENANCE:
                    errs.append(f"{where}: prov_overrides source {src!r} not in {PROVENANCE}")
                for oc in ocols:
                    if oc not in names:
                        errs.append(f"{where}: prov_overrides names unknown column {oc!r}")
            if not p.get("source_ids"):
                errs.append(f"{where}: no source_ids")
            for s in p.get("source_ids") or []:
                if s not in db.sources:
                    errs.append(f"{where}: source {s} not in sources.csv")
            if p.get("date_checked") is None:
                errs.append(f"{where}: date_checked is blank")
            errs += [f"{where}: {e}" for e in _consistency(name, p, db)]
    return errs


def _consistency(name: str, p: Part, db: DB) -> list[str]:
    e = []
    g = lambda k: p.get(k)  # noqa: E731
    if name == "controllers" and g("kind") not in ("mcu", "dcdc") and g("ok_7s_full") is not None:
        want = ok_7s(g("v_max_V"))
        if g("ok_7s_full") != want:
            e.append(f"ok_7s_full={g('ok_7s_full')} but v_max_V={g('v_max_V')} says {want}")
    if g("v_min_V") is not None and g("v_max_V") is not None and g("v_min_V") > g("v_max_V"):
        e.append("v_min_V > v_max_V")
    if name == "motors":
        if g("current_noload_A") is not None and g("current_stall_A") is not None and \
                g("current_noload_A") >= g("current_stall_A"):
            e.append("current_noload_A >= current_stall_A")
        if g("torque_rated_Nm") is not None and g("torque_peak_Nm") is not None and \
                g("torque_rated_Nm") > g("torque_peak_Nm") * 1.001:
            e.append("torque_rated_Nm > torque_peak_Nm")
        has_pts = any(g(k) is not None for k in ("speed_noload_rpm", "torque_stall_Nm", "torque_rated_Nm"))
        if has_pts and g("v_test_V") is None:
            e.append("speed/torque points without v_test_V")
        c = g("controller")
        if c and c != "integrated" and c not in db.controllers:
            e.append(f"controller {c!r} not in controllers.csv")
        if g("integration") == "smart" and c not in (None, "integrated"):
            e.append("integration=smart but controller is not 'integrated'")
    if name == "servos" and g("kind") == "hobby_pwm" and g("feedback"):
        e.append("hobby_pwm servo with feedback")
    return e


# ---------------------------------------------------------------------------------------------------------------
# Derived quantities

@dataclass(frozen=True)
class DCModel:
    """Linear DC-motor model at the OUTPUT shaft (brushed, or a BLDC under FOC / trapezoidal drive).

    From five datasheet points (rated V, no-load speed and current, stall torque and current), as
    catalog.FeedMotor does: R = V / i_stall, kt = T_stall / (i_stall - i_noload) (gear losses included),
    ke = (V - i_noload R) / w_noload, drag = kt i_noload. Or from kt / kv / R / gear when the points are missing.
    """
    r: float                 # ohm, DC-equivalent (copper loss = r i^2)
    kt: float                # N m/A at the output (gear losses included)
    ke: float                # V s/rad at the output
    drag: float              # N m at the output (no-load loss)
    i_peak: float | None     # A, the part's own limit (integrated driver), if any
    source: str              # "points" or "constants"

    def speed_noload(self, v: float) -> float:
        """rad/s at the output."""
        return max(0.0, (v - self.drag / self.kt * self.r) / self.ke)

    def torque_at(self, w: float, v: float, i_limit: float | None = None) -> float:
        """Most motoring torque at output speed w (rad/s) on supply v, net of drag, capped by the current limit."""
        lim = [x for x in (i_limit, self.i_peak) if x is not None]
        i = (v - self.ke * abs(w)) / self.r
        if lim:
            i = min(i, min(lim))
        return max(0.0, self.kt * max(0.0, i) - self.drag)

    def current_for(self, torque: float) -> float:
        return (abs(torque) + self.drag) / self.kt

    def copper_loss(self, torque_rms: float) -> float:
        i = torque_rms / self.kt
        return self.r * i * i


def dc_model(m: Part, measured: bool = True) -> DCModel | None:
    """The output-shaft DC model of a motor row (measured values first when measured=True)."""
    val = (lambda k: m.meas(k)) if measured else (lambda k: m.get(k))  # noqa: E731
    v, n0, i0, ts, is_ = (val(k) for k in ("v_test_V", "speed_noload_rpm", "current_noload_A", "torque_stall_Nm",
                                           "current_stall_A"))
    if measured and m.get("meas_v_V") is not None and m.get("meas_speed_noload_rpm") is not None:
        v = m["meas_v_V"]
    gear = m.get("gear_ratio") or 1.0
    eff = m.get("gear_eff") or (1.0 if gear == 1.0 else 0.9)
    if None not in (v, n0, ts, is_) and is_ > 0:
        i0 = i0 or 0.0
        r = v / is_
        kt = ts / (is_ - i0)
        ke = (v - i0 * r) / (n0 * RPM)
        return DCModel(r, kt, ke, kt * i0, m.get("current_peak_A") if m.get("kind") == "integrated_actuator" else None,
                       "points")
    kt_m = val("kt_Nm_per_A")
    r = val("r_ohm")
    if kt_m is None or r is None:
        return None
    if m.get("commutation") not in ("brushed",) and m.get("r_kind") == "line_line":
        r = 0.75 * r                                   # FOC: copper loss 1.5 R_phase iq^2, R_phase = r_ll / 2
    elif m.get("commutation") not in ("brushed",) and m.get("r_kind") == "phase":
        r = 1.5 * r
    if n0 is not None and v is not None:
        ke = v / (n0 * RPM)                            # output side, from the no-load speed
    elif m.get("kv_rpm_per_V"):
        ke = gear / (m["kv_rpm_per_V"] * RPM)
    else:
        return None
    drag = kt_m * gear * eff * i0 if i0 else 0.0
    return DCModel(r, kt_m * gear * eff, ke, drag, m.get("current_peak_A"), "constants")


def output_speed_ms(rpm: float, wheel_diameter_m: float) -> float:
    return rpm * RPM * wheel_diameter_m / 2


def wheel_rpm(speed_ms: float, wheel_diameter_m: float) -> float:
    return speed_ms / (wheel_diameter_m / 2) / RPM


def servo_speed(s: Part) -> float | None:
    """No-load speed in rad/s."""
    if s.meas("speed_noload_rad_s") is not None:
        return s.meas("speed_noload_rad_s")
    return (math.pi / 3) / s["speed_s_per_60deg"] if s.get("speed_s_per_60deg") else None


def servo_torque_at(s: Part, w: float) -> float | None:
    """Linear servo curve: stall torque falling to zero at the no-load speed (at v_test_V)."""
    ts, w0 = s.meas("torque_stall_Nm"), servo_speed(s)
    if ts is None or w0 is None:
        return None
    return max(0.0, ts * (1 - abs(w) / w0))


# ---------------------------------------------------------------------------------------------------------------
# Requirements, checks, figures of merit

INTEGRATION_RANK = {"bare": 0, "gearbox": 1, "encoder": 1, "gearbox_encoder": 2, "driver": 3, "smart": 4}


@dataclass
class Requirement:
    """What one function asks of one actuator. Blank (None) = not checked."""
    name: str
    n_units: int = 1
    speed_rpm: float | None = None           # output speed it must reach ...
    torque_at_speed_Nm: float | None = None  # ... with this much torque left
    v_supply_min_V: float | None = None      # lowest supply it must do that on (low battery or a regulated rail)
    v_supply_max_V: float | None = None      # highest supply it must survive (full pack)
    torque_peak_Nm: float | None = None
    torque_rms_Nm: float | None = None       # continuous duty: compared with rated torque (or a thermal model)
    holding_torque_Nm: float | None = None   # held at standstill against a back-driving load
    t_ambient_C: float = 25.0
    t_margin_C: float = 20.0                 # winding must stay this far under its maximum
    interfaces: tuple = ()                   # any of these is acceptable
    integration_min: str | None = None
    feedback_required: bool = False
    mass_max_kg: float | None = None
    price_max_usd: float | None = None       # per unit, motor + controller share
    notes: str = ""


@dataclass
class Check:
    name: str
    status: str              # PASS, FAIL, UNKNOWN
    value: object = None
    need: object = None
    note: str = ""

    def __str__(self):
        v = f"{self.value:.3g}" if isinstance(self.value, float) else self.value
        n = f"{self.need:.3g}" if isinstance(self.need, float) else self.need
        return f"{self.status:7s} {self.name}: {v} (need {n}){' - ' + self.note if self.note else ''}"


def controller_limit(m: Part, ctl: Part | None) -> float | None:
    if ctl is None:
        return None
    return ctl.get("i_peak_A") or ctl.get("i_cont_A")


def check(part: Part, req: Requirement, ctl: Part | None = None, share: int = 1, extra_usd: float = 0.0) -> list[Check]:
    """Check a motor or servo against a requirement (ctl: its controller when it isn't integrated)."""
    out: list[Check] = []
    add = lambda *a, **k: out.append(Check(*a, **k))  # noqa: E731
    is_servo = part.table == "servos"
    vmin = req.v_supply_min_V
    if is_servo:
        w0 = servo_speed(part)
        if req.speed_rpm is not None:
            need = req.speed_rpm * RPM
            add("speed", "UNKNOWN" if w0 is None else ("PASS" if w0 >= need else "FAIL"), w0, need,
                f"rad/s no-load at {part.get('v_test_V')} V")
            if req.torque_at_speed_Nm is not None:
                t = servo_torque_at(part, need)
                add("torque at speed", "UNKNOWN" if t is None else ("PASS" if t >= req.torque_at_speed_Nm else "FAIL"),
                    t, req.torque_at_speed_Nm, "linear servo curve")
        if req.torque_peak_Nm is not None:
            t = part.meas("torque_stall_Nm")
            add("peak torque", "UNKNOWN" if t is None else ("PASS" if t >= req.torque_peak_Nm else "FAIL"), t,
                req.torque_peak_Nm)
        if req.torque_rms_Nm is not None:
            t = part.get("torque_rated_Nm")
            if t is None and part.get("torque_stall_Nm"):
                add("RMS vs rated", "UNKNOWN", part["torque_stall_Nm"] / 3, req.torque_rms_Nm,
                    "no rated torque; a third of stall shown (est rule of thumb)")
            else:
                add("RMS vs rated", "UNKNOWN" if t is None else ("PASS" if t >= req.torque_rms_Nm else "FAIL"), t,
                    req.torque_rms_Nm)
        if req.feedback_required:
            add("feedback", "PASS" if "position" in (part.get("feedback") or []) else "FAIL", part.get("feedback"),
                "position")
    else:
        mdl = dc_model(part)
        ilim = controller_limit(part, ctl)
        if req.speed_rpm is not None:
            v = vmin if vmin is not None else part.get("v_test_V")
            if mdl is None or v is None:
                add("speed at min supply", "UNKNOWN", None, req.speed_rpm)
            else:
                n = mdl.speed_noload(v) / RPM
                add("speed at min supply", "PASS" if n >= req.speed_rpm else "FAIL", n, req.speed_rpm,
                    f"rpm no-load at {v} V")
                if req.torque_at_speed_Nm is not None:
                    t = mdl.torque_at(req.speed_rpm * RPM, v, ilim)
                    add("torque at speed", "PASS" if t >= req.torque_at_speed_Nm else "FAIL", t, req.torque_at_speed_Nm,
                        f"N m at {req.speed_rpm:.0f} rpm, {v} V")
        if req.torque_peak_Nm is not None:
            cands = [part.meas("torque_peak_Nm")]
            if mdl is not None and ilim is not None:
                cands.append(mdl.kt * ilim)
            if mdl is not None and part.get("v_test_V") and part.get("torque_peak_Nm") is None:
                cands.append(mdl.torque_at(0.0, vmin or part["v_test_V"], ilim))
            cands = [c for c in cands if c is not None]
            t = min(cands) if cands else None
            add("peak torque", "UNKNOWN" if t is None else ("PASS" if t >= req.torque_peak_Nm else "FAIL"), t,
                req.torque_peak_Nm)
        if req.torque_rms_Nm is not None:
            out.append(_thermal(part, mdl, req))
        if req.holding_torque_Nm is not None:
            if part.get("backdrivable") == "no":
                add("holding", "PASS", "self-locking", req.holding_torque_Nm)
            else:
                t = mdl.kt * ilim if (mdl is not None and ilim is not None) else part.meas("torque_peak_Nm")
                add("holding", "UNKNOWN" if t is None else ("PASS" if t >= req.holding_torque_Nm else "FAIL"), t,
                    req.holding_torque_Nm, "current-limited hold")
        if req.feedback_required:
            ok = part.get("sensor_cpr_output") or part.get("sensor_type") not in (None, "none")
            add("feedback", "PASS" if ok else "FAIL", part.get("sensor_type"), "an encoder")
    if req.v_supply_max_V is not None:
        vm = part.get("v_max_V")
        add("survives max supply", "UNKNOWN" if vm is None else ("PASS" if vm >= req.v_supply_max_V else "FAIL"), vm,
            req.v_supply_max_V, "" if vm is not None else f"rated {part.get('v_rated_V') or part.get('v_test_V')} V, max VERIFY")
        if ctl is not None:
            cm = ctl.get("v_max_V")
            add("controller max supply", "UNKNOWN" if cm is None else ("PASS" if cm >= req.v_supply_max_V else "FAIL"),
                cm, req.v_supply_max_V, ctl["id"])
    if req.interfaces:
        have = set(part.get("interface") or []) | set((ctl or {}).get("interface") or []) | \
            ({part.get("bus")} if is_servo else set())
        add("interface", "PASS" if have & set(req.interfaces) else "FAIL", sorted(x for x in have if x),
            list(req.interfaces))
    if req.integration_min is not None and not is_servo:
        lvl = part.get("integration")
        if ctl is not None and ctl.get("kind") == "smart_controller" and part.get("sensor_cpr_output"):
            lvl = "smart"                               # a gearmotor + encoder on a controller that closes the loops
        add("integration", "PASS" if INTEGRATION_RANK.get(lvl, -1) >= INTEGRATION_RANK[req.integration_min] else "FAIL",
            lvl, req.integration_min)
    if req.mass_max_kg is not None:
        mk = part.meas("mass_kg")
        add("mass", "UNKNOWN" if mk is None else ("PASS" if mk <= req.mass_max_kg else "FAIL"), mk, req.mass_max_kg)
    if req.price_max_usd is not None:
        c = function_cost(part, ctl, share, extra_usd)
        add("price", "UNKNOWN" if c is None else ("PASS" if c <= req.price_max_usd else "FAIL"), c, req.price_max_usd)
    return out


def _thermal(m: Part, mdl: DCModel | None, req: Requirement) -> Check:
    """RMS torque against a steady-state thermal model when r_th is known, else against the rated torque."""
    rth, tmax = m.meas("r_th_KperW"), m.get("t_winding_max_C")
    if mdl is not None and rth is not None and tmax is not None:
        t = req.t_ambient_C + mdl.copper_loss(req.torque_rms_Nm) * rth
        return Check("RMS thermal", "PASS" if t <= tmax - req.t_margin_C else "FAIL", t, tmax - req.t_margin_C,
                     "steady-state winding C")
    tr = m.get("torque_rated_Nm")
    if tr is None:
        return Check("RMS vs rated", "UNKNOWN", None, req.torque_rms_Nm, "no rated torque or thermal data")
    return Check("RMS vs rated", "PASS" if tr >= req.torque_rms_Nm else "FAIL", tr, req.torque_rms_Nm)


def passes(checks: list[Check]) -> str:
    """Overall verdict: FAIL if any check fails, else UNKNOWN if any is unknown, else PASS."""
    s = {c.status for c in checks}
    return "FAIL" if "FAIL" in s else ("UNKNOWN" if "UNKNOWN" in s else "PASS")


def function_cost(part: Part, ctl: Part | None = None, share: int = 1, extra_usd: float = 0.0) -> float | None:
    """USD per actuator for the complete function: the part, its share of the controller (one board drives `share`
    motors), plus power conversion or adapters (extra_usd, per actuator)."""
    if part.get("price_usd") is None:
        return None
    c = part["price_usd"] + extra_usd
    if ctl is not None:
        if ctl.get("price_usd") is None:
            return None
        c += ctl["price_usd"] / share
    return c


def usd_per_Nm(part: Part, speed_rpm: float, v: float, ctl: Part | None = None, share: int = 1) -> float | None:
    """Function cost per N m available at the operating speed and supply."""
    c = function_cost(part, ctl, share)
    if c is None:
        return None
    if part.table == "servos":
        t = servo_torque_at(part, speed_rpm * RPM)
    else:
        mdl = dc_model(part)
        t = None if mdl is None else mdl.torque_at(speed_rpm * RPM, v, controller_limit(part, ctl))
    return c / t if t else None


def Nm_per_kg(part: Part) -> float | None:
    t = part.meas("torque_peak_Nm") if part.table == "motors" else part.meas("torque_stall_Nm")
    mk = part.meas("mass_kg")
    return t / mk if t and mk else None


def compatible(m: Part, ctl: Part, v_supply: float | None = None) -> list[str]:
    """Problems with driving motor m from controller ctl ([] = compatible) on supply v_supply. Messages starting with
    'note:' are cautions, not incompatibilities."""
    issues = []
    kind = m.get("kind")
    need = "brushed" if kind == "brushed_dc" else ("stepper" if "stepper" in (kind or "") else "bldc")
    if need not in (ctl.get("motor_types") or []):
        issues.append(f"{ctl['id']} does not drive {need} motors")
    if m.get("controller") == "integrated":
        issues.append(f"{m['id']} has its own driver")
    elif m.get("controller") and m["controller"] != ctl["id"]:
        issues.append(f"{m['id']} needs {m['controller']}")
    if v_supply is not None:
        lo, hi = ctl.get("v_min_V"), ctl.get("v_max_V")
        if hi is None:
            issues.append(f"{ctl['id']} max voltage unknown (VERIFY) for {v_supply} V")
        elif v_supply > hi:
            issues.append(f"{v_supply} V over {ctl['id']}'s {hi} V")
        if lo is not None and v_supply < lo:
            issues.append(f"{v_supply} V under {ctl['id']}'s {lo} V")
        if m.get("v_max_V") is not None and v_supply > m["v_max_V"] * 1.0001:
            issues.append(f"{v_supply} V over {m['id']}'s max {m['v_max_V']} V")
        elif m.get("v_max_V") is None and m.get("v_rated_V") is not None and v_supply > m["v_rated_V"] * 1.0001:
            issues.append(f"note: {v_supply} V over {m['id']}'s rated {m['v_rated_V']} V (max unknown)")
    i_need = m.get("current_peak_A") or m.get("current_stall_A")
    if i_need is not None and ctl.get("i_cont_A") is not None:
        cap = ctl.get("i_peak_A") or ctl["i_cont_A"]
        if i_need > cap:
            issues.append(f"motor peak/stall {i_need} A > controller {cap} A: set a current limit")
    if m.get("commutation") == "brushed" and m.get("sensor_cpr_output") and "speed_pid" in (ctl.get("loops") or []) \
            and not ctl.get("encoder_inputs"):
        issues.append(f"{ctl['id']} has no encoder inputs for its speed loop")
    if m.get("commutation") == "encoder_foc" and ctl.get("kind") == "foc_driver":
        issues.append(f"{ctl['id']} is a power stage only: FOC and the encoder run on an MCU (DIY)")
    return issues


# ---------------------------------------------------------------------------------------------------------------
# Export to a sim catalog (omni-robot's catalog.py dataclasses; the catalog module is passed in)

def sim_catalog(sim_dir: Path | str):
    """Import a project's catalog.py (e.g. omni-robot/simulation/v6 shallow throw)."""
    sim_dir = str(Path(sim_dir).resolve())
    if sim_dir not in sys.path:
        sys.path.insert(0, sim_dir)
    import importlib
    return importlib.import_module("catalog")


def _need(m: Part, *cols):
    miss = [c for c in cols if m.get(c) is None]
    if miss:
        raise ValueError(f"{m['id']}: export needs {miss}")


def to_drive_motor(cat, m: Part, ctl: Part | None = None, **over):
    """catalog.DriveMotor (geared actuator with an FOC or H-bridge driver; output torque = kt gear eff i)."""
    mdl = dc_model(m)
    _need(m, "v_test_V", "speed_noload_rpm", "gear_ratio", "mass_kg")
    gear = m["gear_ratio"]
    eff = m.get("gear_eff") or 0.9
    kt = mdl.kt / (gear * eff)
    i_peak = over.pop("i_peak", None) or controller_limit(m, ctl) or m.get("current_peak_A") or m.get("current_stall_A")
    if ctl is not None and m.get("current_peak_A"):
        i_peak = min(i_peak, m["current_peak_A"])
    i_rated = m.get("current_rated_A") or (m["torque_rated_Nm"] / mdl.kt if m.get("torque_rated_Nm") else i_peak / 3)
    price = function_cost(m, ctl) or 0.0
    kw = dict(name=m["name"], part=m["part_number"] or m["id"], price=price, mass=m["mass_kg"],
              diameter=m.get("diameter_m") or 0.04, length=m.get("length_m") or 0.06,
              kv=m.get("kv_rpm_per_V") or gear / (mdl.ke * RPM), kt=kt, gear=gear, r_ll=mdl.r / 0.75,
              i_rated=i_rated, i_peak=i_peak, v_rated=m["v_test_V"], noload_rpm=m["speed_noload_rpm"],
              rotor_inertia=m.get("rotor_inertia_kgm2") or 1e-6, backdrive_torque=max(mdl.drag, 0.02), gear_eff=eff,
              notes=f"Exported from actuator_db ({m['id']}); r_ll = r_eq / 0.75.")
    kw.update(over)
    return cat.DriveMotor(**kw)


def to_feed_motor(cat, m: Part, ctl: Part | None = None, v_supply: float = 12.0, share: int = 1, kind=None, **over):
    """catalog.FeedMotor: the five datasheet points at the output, the driver's rail and current limit."""
    _need(m, "v_test_V", "speed_noload_rpm", "torque_stall_Nm", "current_stall_A")
    k = kind or {"brushed": "dc"}.get(m.get("commutation"), "bldc")
    if m.get("gear_type") == "worm":
        k = "worm"
    i_lim = over.pop("i_limit", None) or controller_limit(m, ctl) or m.get("current_peak_A") or m["current_stall_A"]
    kw = dict(name=m["name"], part=m["part_number"] or m["id"], price=function_cost(m, ctl, share) or 0.0,
              mass=m.get("mass_kg") or 0.05, kind=k, v_rated=m["v_test_V"], noload_rpm=m["speed_noload_rpm"],
              i_noload=m.get("current_noload_A") or 0.0, stall_torque=m["torque_stall_Nm"],
              stall_current=m["current_stall_A"], gear=m.get("gear_ratio") or 1.0,
              rotor_inertia=m.get("rotor_inertia_kgm2") or 1e-6,
              driver=ctl["name"] if ctl is not None else "integrated", v_supply=v_supply, i_limit=i_lim,
              notes=f"Exported from actuator_db ({m['id']}).")
    kw.update(over)
    return cat.FeedMotor(**kw)


def to_flywheel_motor(cat, m: Part, **over):
    """catalog.FlywheelMotor (drone outrunner): KV, phase-to-phase R, no-load current at 10 V."""
    _need(m, "kv_rpm_per_V", "r_ohm", "mass_kg")
    r = m["r_ohm"] * (2.0 if m.get("r_kind") == "phase" else 1.0)
    i0 = m.get("current_noload_A")
    kw = dict(name=m["name"], part=m["part_number"] or m["id"], price=m.get("price_usd") or 0.0, mass=m["mass_kg"],
              kv=m["kv_rpm_per_V"], r_m=r, i0=i0 if i0 is not None else 1.0,
              i_max=m.get("current_peak_A") or 40.0, rotor_inertia=m.get("rotor_inertia_kgm2") or 4.5e-6,
              stator=over.pop("stator", ""), mount=m.get("mounting") or "",
              notes=f"Exported from actuator_db ({m['id']}).")
    kw.update(over)
    return cat.FlywheelMotor(**kw)


def to_esc(cat, c: Part, current_limit: float = 25.0, telemetry_period: float = 0.0005, **over):
    """catalog.Esc: price, the current limit set in its firmware, its max voltage."""
    _need(c, "v_max_V")
    kw = dict(name=c["name"], part=c["part_number"] or c["id"], price=c.get("price_usd") or 0.0,
              current_limit=current_limit, v_max=c["v_max_V"], telemetry_period=telemetry_period,
              notes=f"Exported from actuator_db ({c['id']}).")
    kw.update(over)
    return cat.Esc(**kw)


# ---------------------------------------------------------------------------------------------------------------
# Gaps

KEY_BY_KIND = {   # extra key columns by motor kind (on top of Col.key)
    "brushed_dc": ("current_noload_A", "torque_stall_Nm", "current_stall_A", "gear_eff", "backlash_arcmin"),
    "integrated_actuator": ("kt_Nm_per_A", "r_ohm", "current_peak_A", "backlash_arcmin", "t_winding_max_C"),
    "bldc_outrunner": ("kv_rpm_per_V", "r_ohm", "current_noload_A", "current_peak_A", "rotor_inertia_kgm2"),
    "bldc_inrunner": ("kt_Nm_per_A", "r_ohm", "current_peak_A", "backlash_arcmin"),
    "gimbal": ("kv_rpm_per_V", "r_ohm", "current_noload_A", "rotor_inertia_kgm2"),
}


def gaps(db: DB) -> dict[str, list[str]]:
    """Blank key fields per part: what to look up or measure next (price first: cost is a primary criterion)."""
    out = {}
    for name in ("motors", "servos", "controllers"):
        cols = TABLES[name]
        for pid, p in db.table(name).items():
            keys = [c.name for c in cols if c.key]
            if name == "motors":
                keys += [k for k in KEY_BY_KIND.get(p.get("kind"), ()) if k not in keys]
                drop = set()
                if p.get("commutation") == "sensorless":
                    drop.add("sensor_cpr_output")
                if p.get("kind") in ("bldc_outrunner", "gimbal") and (p.get("gear_ratio") or 1) == 1:
                    drop |= {"speed_noload_rpm", "torque_rated_Nm", "torque_peak_Nm", "v_rated_V"}  # KV/R-rated motors
                keys = [k for k in keys if k not in drop]
            if name == "controllers" and p.get("kind") in ("mcu", "dcdc"):
                keys = [k for k in keys if k in ("price_usd", "v_min_V", "v_max_V", "interface")]
            miss = [k for k in keys if p.get(k) in (None, [])]
            if name == "controllers" and p.get("ok_7s_full") == "VERIFY" and "v_max_V" not in miss:
                miss.append("v_max_V")
            if miss:
                out[f"{name}:{pid}"] = miss
    return out


def write_gaps_csv(db: DB, path: Path | str = HERE / "gaps.csv"):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["table", "id", "missing_fields"])
        for k, miss in gaps(db).items():
            t, pid = k.split(":", 1)
            w.writerow([t, pid, ";".join(miss)])


# ---------------------------------------------------------------------------------------------------------------
# CLI

def schema_markdown() -> str:
    lines = []
    for name, cols in TABLES.items():
        lines += [f"### {name}.csv", "", "| Column | Type | Meaning |", "| --- | --- | --- |"]
        for c in cols:
            t = c.type + (f" ({' / '.join(c.enum)})" if c.enum else "")
            lines.append(f"| `{c.name}`{' *' if c.key else ''} | {t} | {c.desc} |")
        lines.append("")
    return "\n".join(lines)


def show(db: DB, pid: str) -> str:
    p = db.get(pid)
    lines = [f"{p.table}:{pid}"] + [f"  {k} = {_fmt(v)}  [{p.prov(k)}]" for k, v in p.items()
                                      if v not in (None, []) and not k.startswith("prov")]
    if p.table == "motors":
        mdl = dc_model(p)
        if mdl:
            lines.append(f"  derived ({mdl.source}): R {mdl.r:.3g} ohm, kt {mdl.kt:.3g} N m/A, ke {mdl.ke:.3g} V s/rad, "
                         f"drag {mdl.drag:.3g} N m (output side)")
        if Nm_per_kg(p):
            lines.append(f"  peak N m per kg {Nm_per_kg(p):.2f}")
    return "\n".join(lines)


def main(argv):
    cmd = argv[0] if argv else "validate"
    db = load()
    if cmd == "validate":
        errs = validate(db)
        print("\n".join(errs) if errs else
              f"OK: {len(db.motors)} motors, {len(db.servos)} servos, {len(db.controllers)} controllers, "
              f"{len(db.sources)} sources")
        return 1 if errs else 0
    if cmd == "gaps":
        g = gaps(db)
        for k, miss in g.items():
            print(f"{k}: {', '.join(miss)}")
        if "--csv" in argv:
            write_gaps_csv(db)
            print(f"wrote {HERE / 'gaps.csv'}")
        return 0
    if cmd == "show":
        print(show(db, argv[1]))
        return 0
    if cmd == "schema":
        print(schema_markdown())
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
