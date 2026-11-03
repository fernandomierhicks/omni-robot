"""Electrical regressions for the selected RC LiPo and retained lithium-ion packs."""

from dataclasses import replace

import pytest

from catalog import BATTERIES
from models import Battery
from params import SimParams
from robot_export import firmware_header, v11_params


def test_selected_pack_has_six_cells_and_delivered_pack_mass():
    p = SimParams()
    assert p.battery == "6S1P-OVONIC-5200"
    assert p.pack.capacity == pytest.approx(5.2)
    assert p.pack.mass == pytest.approx(0.706)  # not another 25% packaging allowance
    assert Battery(p.pack).v == pytest.approx(25.2)
    assert Battery(p.pack, soc=0.2).ocv() < Battery(p.pack, soc=0.8).ocv()


def test_lipo_pulse_sags_then_recovers_without_recharging():
    b = Battery(SimParams().pack, soc=0.5)
    unloaded = b.v
    for _ in range(100):
        b.step(20.0, 0.1)
    loaded, charge = b.v, b.soc
    assert loaded < unloaded
    for _ in range(1000):
        b.step(0.0, 0.1)
    assert loaded < b.v < unloaded
    assert b.soc == charge


def test_half_hour_at_one_c_consumes_half_the_charge():
    b = Battery(SimParams().pack)
    for _ in range(1800):
        b.step(5.2, 1.0)
    assert b.soc == pytest.approx(0.5)
    assert b.ocv() < 25.2
    assert not b.tripped


def test_rc_pack_does_not_invent_a_bms_but_old_pack_retains_trip():
    rc = Battery(SimParams().pack)
    protected = Battery(BATTERIES["7S1P-P50B"])
    rc.step(50.0, 0.01)
    protected.step(50.0, 0.01)
    assert not rc.tripped
    assert protected.tripped


def test_firmware_low_voltage_tracks_selected_pack_and_explicit_legacy_override():
    p = v11_params()
    assert "BATT_LOW = 21.0f;" in firmware_header(p)
    old = replace(p, battery="7S1P-P50B")
    assert "BATT_LOW = 22.4f;" in firmware_header(old)
