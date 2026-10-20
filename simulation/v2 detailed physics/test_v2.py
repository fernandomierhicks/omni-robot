"""Headless checks for the v2 sim. Run: python -m pytest -q   (about a minute)"""

import math

import mujoco
import numpy as np
import pytest

from catalog import BATTERIES, DRIVE_MOTORS
from catch_game import CatchGame
from launcher_bench import Bench
from models import Battery, DriveActuator
from params import SimParams
from robot import Robot
from robot_model import build_model


def make(p=None, facing_x=True):
    p = p or SimParams(home_xy=(0.0, 0.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    if facing_x:
        adr = m.joint("kiwi").qposadr[0]
        d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    return p, m, d, Robot(m, d, p)


@pytest.mark.parametrize("wheel_model", ["sphere", "rollers"])
def test_model_compiles_with_plausible_mass(wheel_model):
    p, m, d, r = make(SimParams(home_xy=(0, 0), wheel_model=wheel_model))
    kiwi = m.body("kiwi").id
    assert 4.5 < m.body_subtreemass[kiwi] < 9.0
    assert d.subtree_com[kiwi][2] < 0.2


def test_drive_torque_fades_with_speed_and_battery():
    drv = DriveActuator(DRIVE_MOTORS["AK40-10"])
    lo, hi = drv.torque_limits(0.0, 25.0)
    assert hi == pytest.approx(drv.s.peak_torque, rel=1e-6)                 # current limited at stall
    w_noload = 25.0 / (drv.s.ke * drv.s.gear)
    fade = [drv.torque_limits(f * w_noload, 25.0)[1] for f in (0.7, 0.8, 0.9, 0.99, 1.0)]
    assert all(a >= b for a, b in zip(fade, fade[1:]))                     # back-EMF limited near no-load
    assert fade[-2] < 0.1 * hi and fade[-1] == 0.0
    assert drv.torque_limits(0.75 * w_noload, 21.0)[1] < 0.6 * drv.torque_limits(0.75 * w_noload, 25.0)[1]


def test_battery_sags_under_load_and_recovers():
    b = Battery(BATTERIES["7S1P-P50B"])
    v0 = b.v
    for _ in range(100):
        b.step(30.0, 0.01)
    sag = v0 - b.v
    assert 3.0 < sag < 6.0                     # ~0.13 ohm pack plus polarization
    for _ in range(3000):
        b.step(0.0, 0.01)
    assert v0 - b.v < 0.2


def test_drive_tracks_commanded_twist():
    p, m, d, r = make()
    for _ in range(int(1.0 / m.opt.timestep)):
        k = min(1.0, d.time / 0.4)
        r.wheel_cmd = r.kin.wheel_speeds([1.0 * k, 0.0, 0.0])
        r.step()
    v = d.sensor("base_linvel").data
    assert np.hypot(v[0], v[1]) == pytest.approx(1.0, abs=0.03)
    assert abs(d.sensor("base_angvel").data[2]) < 0.02


def test_launcher_shot_matches_calibration_range():
    bench = Bench()
    shot = bench.shot(3000)
    assert shot.ok
    assert 7.0 < shot.exit_speed < 9.5
    assert 48 < shot.elevation < 56
    assert abs(shot.azimuth) < 5


def test_catch_and_return():
    p = SimParams()
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, seed=3, verbose=False)
    while len(game.results) < 3 or game.state != "wait":
        game.step()
    assert all(res.caught for res in game.results)
    assert all(res.launched for res in game.results[:2])
