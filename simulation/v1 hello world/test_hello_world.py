"""Headless checks for the v1 hello-world sim. Run: python -m pytest -q"""

import math

import mujoco
import numpy as np
import pytest

from ball_flight import BallFlight
from hello_world import CatchGame, DriveDemo, Robot, World
from kiwi_model import SimParams, build_model


def make(p=None, facing_x=False):
    p = p or SimParams(home_xy=(0.0, 0.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    if facing_x:
        adr = m.joint("kiwi").qposadr[0]
        d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    return p, m, d


def run(m, d, seconds, fn=None):
    for _ in range(int(round(seconds / m.opt.timestep))):
        if fn:
            fn()
        mujoco.mj_step(m, d)


def test_mass_and_cg_in_design_range():
    p, m, d = make()
    kiwi = m.body("kiwi").id
    assert 6.0 <= m.body_subtreemass[kiwi] <= 9.0
    assert d.subtree_com[kiwi][2] < 0.25


def test_robot_rests_still():
    p, m, d = make()
    robot = Robot(m, d, p)
    start = robot.pose()
    run(m, d, 2.0)
    end = robot.pose()
    assert np.hypot(*(end[:2] - start[:2])) < 1e-3
    assert abs(end[2] - start[2]) < 1e-3


@pytest.mark.parametrize("twist", [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-0.7, 0.7, 0.0), (0.0, 0.0, 3.0)])
def test_kinematics_track_body_twist(twist):
    """Open-loop wheel commands from the kinematics give the commanded body twist."""
    p, m, d = make(facing_x=True)
    robot = Robot(m, d, p)
    acts = robot.wheel_act

    def ramp():
        k = min(1.0, d.time / 0.5)
        d.ctrl[acts] = robot.kin.wheel_speeds(np.array(twist) * k)

    run(m, d, 1.2, ramp)
    yaw = robot.pose()[2]
    v = d.sensor("base_linvel").data
    c, s = math.cos(yaw), math.sin(yaw)
    body = np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], d.sensor("base_angvel").data[2]])
    assert np.allclose(body, twist, atol=0.02 * max(1.0, max(map(abs, twist))))


def test_drag_model_matches_mujoco():
    """Predicted crossing of a plane matches the simulated ball to within 1 cm."""
    p, m, d = make()
    world = World(m, d)
    flight = BallFlight(p.ball_mass, p.ball_radius, p.ball_drag_cd, p.air_density, p.air_viscosity)
    start, vel = np.array([-1.5, 1.2, 1.0]), np.array([2.5, -0.5, 5.0])
    world.place_ball(start, vel)
    mujoco.mj_forward(m, d)
    predicted, _ = flight.crossing(start, vel, 0.6)
    prev = d.sensor("ball_pos").data.copy()
    while True:
        mujoco.mj_step(m, d)
        pos = d.sensor("ball_pos").data.copy()
        if d.sensor("ball_vel").data[2] < 0 and pos[2] < 0.6:
            f = (prev[2] - 0.6) / (prev[2] - pos[2])
            actual = prev + f * (pos - prev)
            break
        prev = pos
    assert np.linalg.norm(actual[:2] - predicted[:2]) < 0.01
    # and drag matters for this ball: a vacuum throw would land far away
    vacuum = start[:2] + vel[:2] * (vel[2] + math.sqrt(vel[2] ** 2 + 2 * 9.81 * 0.4)) / 9.81
    assert np.linalg.norm(vacuum - predicted[:2]) > 0.2


def test_ball_dropped_in_funnel_trips_beam():
    p, m, d = make()
    robot, world = Robot(m, d, p), World(m, d)
    world.place_ball([0.06, 0.03, 1.2], [0, 0, 0])
    run(m, d, 1.0)
    assert robot.ball_detected_in_chamber()


def test_catch_game_catches_tosses():
    p = SimParams()
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, seed=1, verbose=False)
    while len(game.results) < 10:
        game.step()
        mujoco.mj_step(m, d)
    assert sum(r["caught"] for r in game.results) >= 9


def test_drive_demo_stays_in_area():
    p, m, d = make(facing_x=True)
    demo = DriveDemo(m, d, p)
    robot = demo.robot
    worst = 0.0

    def step():
        nonlocal worst
        demo.step()
        worst = max(worst, float(np.hypot(*robot.pose()[:2])))

    run(m, d, 25.0, step)
    assert demo.i > 3          # got through several moves
    assert worst < 1.6         # never wandered toward the netting
