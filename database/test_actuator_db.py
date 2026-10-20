"""Checks for the actuator database (python -m pytest -q in this folder)."""

from __future__ import annotations

import math
import shutil
from pathlib import Path

import pytest

import actuator_db as adb

DB = adb.load()
SIM = Path(__file__).resolve().parent.parent / "simulation" / "v6 shallow throw"


def test_loads_and_validates():
    assert adb.validate(DB) == []
    assert len(DB.motors) > 20 and len(DB.servos) >= 5 and len(DB.controllers) > 10


def test_every_part_cites_a_known_source_and_date():
    for p in DB.parts():
        assert p["source_ids"] and all(s in DB.sources for s in p["source_ids"]), p["id"]
        assert p["date_checked"], p["id"]


def test_validate_catches_bad_rows():
    bad = adb.load()
    p = bad.controllers["pololu-drv8874-4035"]
    p["ok_7s_full"] = "no"                                   # 37 V max says yes
    p["source_ids"] = ["S999"]
    p["prov_overrides"] = "guess: price_usd; est: no_such_column"
    errs = "\n".join(adb.validate(bad))
    assert "ok_7s_full" in errs and "S999" in errs and "guess" in errs and "no_such_column" in errs


def test_7s_flag():
    assert adb.ok_7s(34) == "yes" and adb.ok_7s(30) == "marginal" and adb.ok_7s(24) == "no" and adb.ok_7s(None) == "VERIFY"
    assert DB.controllers["dji-c610"]["ok_7s_full"] == "VERIFY"
    assert DB.controllers["basicmicro-roboclaw-2x7a"]["ok_7s_full"] == "yes"
    assert DB.controllers["simplefoc-mini-v1"]["ok_7s_full"] == "no"


def test_provenance_lookup():
    ak = DB.motors["cubemars-ak40-10-v3"]
    assert ak.prov("gear_eff") == "est" and ak.prov("price_usd") == "datasheet"
    assert DB.motors["dji-m2006-p36"].prov("r_ohm") == "reseller"


def test_dc_model_from_five_points():
    m = DB.motors["pololu-25d-12v-9.7-hp-enc"]
    mdl = adb.dc_model(m)
    assert mdl.source == "points"
    assert mdl.r == pytest.approx(12 / 5.0)
    assert mdl.kt == pytest.approx(m["torque_stall_Nm"] / (5.0 - 0.3))
    assert mdl.speed_noload(12.0) / adb.RPM == pytest.approx(1000, rel=1e-6)
    assert mdl.torque_at(0.0, 12.0) == pytest.approx(m["torque_stall_Nm"], rel=1e-6)    # stall is net of drag
    # the brushed 25D still has ~0.1 N m at the chute's 764 rpm on 12 V (design.md 23.1)
    assert 0.09 < mdl.torque_at(764 * adb.RPM, 12.0, 2.1) < 0.13


def test_measured_values_win():
    m = adb.Part(DB.motors["pololu-25d-12v-9.7-hp-enc"])
    m.table = "motors"
    m["meas_speed_noload_rpm"], m["meas_v_V"] = 900.0, 12.0
    assert adb.dc_model(m).speed_noload(12.0) / adb.RPM == pytest.approx(900, rel=1e-6)
    assert adb.dc_model(m, measured=False).speed_noload(12.0) / adb.RPM == pytest.approx(1000, rel=1e-6)


def test_wheel_speed_helpers():
    assert adb.output_speed_ms(adb.wheel_rpm(2.0, 0.096), 0.096) == pytest.approx(2.0)
    assert adb.wheel_rpm(2.0, 0.096) == pytest.approx(398, abs=1)


def test_servo_speed_and_curve():
    s = DB.servos["feetech-sts3250"]
    assert adb.servo_speed(s) == pytest.approx(math.pi / 3 / 0.133)
    assert adb.servo_torque_at(s, adb.servo_speed(s) / 2) == pytest.approx(s["torque_stall_Nm"] / 2)


def test_requirement_checks():
    feed = adb.Requirement("feed", speed_rpm=764, torque_at_speed_Nm=0.06, v_supply_min_V=12.0, feedback_required=True)
    drv = DB.controllers["pololu-drv8874-4035"]
    ok = adb.check(DB.motors["pololu-25d-12v-9.7-hp-enc"], feed, drv)
    assert adb.passes(ok) == "PASS"
    weak = adb.check(DB.motors["pololu-micro-30-hpcb-12v-enc"], feed, drv)
    assert any(c.name == "torque at speed" and c.status == "FAIL" for c in weak)
    drive = adb.Requirement("drive", speed_rpm=376, v_supply_min_V=22.4, v_supply_max_V=29.4, torque_peak_Nm=1.3)
    c37 = adb.check(DB.motors["pololu-37d-24v-19-enc"], drive, drv)
    assert any(c.name == "peak torque" and c.status == "FAIL" for c in c37)
    ak = adb.check(DB.motors["cubemars-ak40-10-v3"], drive)
    assert [c.status for c in ak if c.name == "survives max supply"] == ["UNKNOWN"]     # max voltage not published


def test_costs_and_figures_of_merit():
    m, rc = DB.motors["pololu-37d-24v-19-enc"], DB.controllers["basicmicro-roboclaw-2x7a"]
    assert adb.function_cost(m, rc, share=2) == pytest.approx(60.95 + 79.95 / 2)
    assert adb.function_cost(m, rc, share=2, extra_usd=5) == pytest.approx(60.95 + 79.95 / 2 + 5)
    assert adb.function_cost(DB.motors["generic-jgb37-520-24v"]) is None              # no price yet
    assert adb.Nm_per_kg(DB.motors["cubemars-ak40-10-v3"]) == pytest.approx(4.1 / 0.19)
    assert adb.usd_per_Nm(DB.motors["pololu-25d-12v-9.7-hp-enc"], 764, 12.0, DB.controllers["pololu-drv8874-4035"]) > 0


def test_compatibility():
    sf, c610 = DB.controllers["simplefoc-mini-v1"], DB.controllers["dji-c610"]
    assert any("brushed" in i for i in adb.compatible(DB.motors["pololu-37d-24v-19-enc"], sf))
    assert any("DIY" in i for i in adb.compatible(DB.motors["iflight-gm3506-as5048a"], sf, 12.0))
    assert any("over" in i for i in adb.compatible(DB.motors["iflight-gm3506-as5048a"], sf, 29.4))
    assert any("unknown" in i for i in adb.compatible(DB.motors["dji-m2006-p36"], c610, 29.4))
    assert adb.compatible(DB.motors["pololu-25d-12v-9.7-hp-enc"], DB.controllers["basicmicro-roboclaw-2x7a"], 12.0) == []


def test_gaps_report(tmp_path):
    g = adb.gaps(DB)
    assert "price_usd" in g["motors:generic-jgb37-520-24v"]
    assert "speed_noload_rpm" in g["motors:lk-ms3506-v3"]
    assert "v_max_V" in g["controllers:dji-c610"]
    adb.write_gaps_csv(DB, tmp_path / "gaps.csv")
    assert (tmp_path / "gaps.csv").read_text().count("\n") == len(g) + 1


def test_table_round_trip(tmp_path):
    for f in Path(adb.HERE).glob("*.csv"):
        shutil.copy(f, tmp_path / f.name)
    for name in ("motors", "servos", "controllers", "sources"):
        adb.write_table(DB, name, tmp_path)
    again = adb.load(tmp_path)
    assert adb.validate(again) == []
    assert again.motors["cubemars-ak40-10-v3"] == DB.motors["cubemars-ak40-10-v3"]


@pytest.mark.skipif(not (SIM / "catalog.py").exists(), reason="omni-robot sim not next to the database")
def test_export_matches_the_sim_catalog():
    cat = adb.sim_catalog(SIM)
    ak = adb.to_drive_motor(cat, DB.motors["cubemars-ak40-10-v3"])
    ref = cat.DRIVE_MOTORS["AK40-10"]
    for k in ("kt", "gear", "r_ll", "i_peak", "noload_rpm", "v_rated", "gear_eff", "mass", "price"):
        assert getattr(ak, k) == pytest.approx(getattr(ref, k), rel=1e-3), k
    p37 = adb.to_drive_motor(cat, DB.motors["pololu-37d-24v-19-enc"], DB.controllers["pololu-drv8874-4035"], i_peak=3.0)
    ref = cat.DRIVE_MOTORS["Pololu-37D-19"]
    for k in ("kt", "r_ll", "noload_rpm", "gear"):
        assert getattr(p37, k) == pytest.approx(getattr(ref, k), rel=0.02), k
    f = adb.to_feed_motor(cat, DB.motors["pololu-25d-12v-9.7-hp-enc"], DB.controllers["pololu-drv8874-4035"], i_limit=2.1)
    ref = cat.FEED_MOTORS["Pololu-25D-9.7"]
    for k in ("r", "kt", "ke", "friction", "price", "i_limit", "v_supply"):      # the catalog rounds 3.9 kg cm to 0.382
        assert getattr(f, k) == pytest.approx(getattr(ref, k), rel=3e-3), k
    assert f.torque_at(80.0) == pytest.approx(ref.torque_at(80.0), rel=3e-3)
    ms = adb.to_feed_motor(cat, DB.motors["lk-ms4005-v3"])
    ref = cat.FEED_MOTORS["LK-MS4005"]
    assert ms.torque_at(80.0) == pytest.approx(ref.torque_at(80.0), rel=1e-3)
    fly = adb.to_flywheel_motor(cat, DB.motors["tmotor-f90-2806.5-1300"])
    ref = cat.FLYWHEEL_MOTORS["F90-2806.5-1300"]
    assert (fly.kv, fly.r_m, fly.i0, fly.i_max) == (ref.kv, ref.r_m, ref.i0, ref.i_max)
    esc = adb.to_esc(cat, DB.controllers["tbs-lucid-8s-60a-4in1"])
    assert esc.current_limit == 25.0 and esc.v_max > 29.4
