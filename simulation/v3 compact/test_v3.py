"""Headless checks for the v3 sim. Run: python -m pytest -q   (about a minute)"""

import math

import mujoco
import numpy as np
import pytest

from catch_game import CatchGame
from launcher_bench import Bench
from omni_control import OmniKinematics
from params import SimParams
from robot import Robot
from robot_model import build_model, geometry
from traction import Base


def make(p=None):
    p = p or SimParams(home_xy=(0.0, 0.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    return p, m, d, Robot(m, d, p)


def test_axle_deck_lowers_cg_and_keeps_clearance():
    lo = make(SimParams(home_xy=(0, 0)))
    hi = make(SimParams(home_xy=(0, 0), deck_mode="high", feed="j_bend"))
    cg = [d.subtree_com[m.body("base").id][2] for _, m, d, _ in (lo, hi)]
    assert cg[0] < cg[1] - 0.03
    # nothing on the base hangs below the motors (ground clearance ~21 mm)
    p, m, d, r = lo
    b = m.body("base").id
    for gid in range(m.ngeom):
        if m.geom_bodyid[gid] == b and m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX:
            assert d.geom_xpos[gid][2] - m.geom_size[gid][2] > 0.019


@pytest.mark.parametrize("kw", [dict(layout="x4"), dict(layout="x4", rocker_deg=0.0), dict(layout="kiwi3"),
                                dict(layout="x4", wheel_model="rollers"), dict(feed="j_bend"),
                                dict(deck_mode="high", feed="j_bend")])
def test_model_compiles_with_plausible_mass(kw):
    p, m, d, r = make(SimParams(home_xy=(0, 0), **kw))
    b = m.body("base").id
    assert 4.5 < m.body_subtreemass[b] < 9.0
    assert d.subtree_com[b][2] < 0.2


def test_four_wheel_kinematics_round_trip():
    kin = OmniKinematics((45, 135, 225, 315), 0.15, 0.048)
    tw = np.array([0.7, -0.3, 1.2])
    assert np.allclose(kin.body_twist(kin.wheel_speeds(tw)), tw)


def test_traction_model_matches_v2_kiwi_formula():
    """For three wheels and a centered CG the model reduces to v2's closed form (v2 studies/common.py;
    v2 ignored the yaw torque an off-center CG adds, so compare at a centered CG)."""
    L, h, mu, g = 0.18, 0.158, 0.95, 9.81
    b = Base((60, 180, 300), L, h, 0.0, 0.0, mu)
    u = np.array([math.cos(math.radians(240)), math.sin(math.radians(240))])
    best = float("inf")
    for th in np.radians((60, 180, 300)):
        r_hat = np.array([math.cos(th), math.sin(th)])
        t_hat = np.array([-math.sin(th), math.cos(th)])
        n0 = 1 / 3
        lt = 2 / 3 * h / L * float(u @ r_hat)
        ft = 2 / 3 * abs(float(u @ t_hat))
        if ft + mu * lt > 1e-9:
            best = min(best, mu * n0 * g / (ft + mu * lt))
    assert b.slip_accel(u) == pytest.approx(best, rel=0.02)


def test_x4_optimal_allocation_worst_case_is_half_mu_g():
    """Diagonal moves load and unload only the two wheels that can't push: mu g / 2, independent of
    CG height as long as no wheel lifts first (up to h ~ 0.15 m at r 150 mm)."""
    for h in (0.10, 0.15):
        b = Base((45, 135, 225, 315), 0.15, h, mu=0.95)
        worst = min(b.limit(a, "optimal")["slip"] for a in range(0, 360, 15))
        assert worst == pytest.approx(0.95 * 9.81 / 2, rel=0.02)


def test_drive_tracks_and_loads_sum_to_weight():
    p, m, d, r = make()
    for _ in range(int(0.3 / m.opt.timestep)):
        r.step()
    assert r.wheel_loads().sum() == pytest.approx(r.mass * 9.81, rel=0.03)
    for _ in range(int(1.0 / m.opt.timestep)):
        k = min(1.0, (d.time - 0.3) / 0.4)
        r.wheel_cmd = r.kin.wheel_speeds([0.0, 1.0 * k, 0.0])
        r.step()
    v = d.sensor("base_linvel").data
    assert np.hypot(v[0], v[1]) == pytest.approx(1.0, abs=0.03)
    assert abs(d.sensor("base_angvel").data[2]) < 0.02


def test_tilt_moves_rim_where_geometry_says():
    p, m, d, r = make()
    r.tilt_target = math.radians(25)
    for _ in range(int(0.6 / m.opt.timestep)):
        r.step()
    assert math.degrees(r.tilt()) == pytest.approx(25, abs=1.0)
    x, z = r.g.rim_center_robot(r.tilt())
    fc = r.funnel_center_world() - d.sensor("base_pos").data
    assert fc[0] == pytest.approx(x, abs=0.004) and fc[2] == pytest.approx(z, abs=0.004)


def test_module_clears_plate_over_tilt_range():
    g = geometry(SimParams())
    assert g.pivot_r[1] > g.plate_top


@pytest.mark.parametrize("feed,tilt,elev", [("kicker", -30, 60), ("kicker", 0, 30), ("j_bend", -20, 60)])
def test_launcher_feeds_and_tilt_sets_elevation(feed, tilt, elev):
    shot = Bench(SimParams(feed=feed)).shot(3000, tilt)
    assert shot.ok
    assert 6.5 < shot.exit_speed < 9.0
    assert shot.elevation == pytest.approx(elev, abs=3.0)
    assert abs(shot.azimuth) < 5


@pytest.mark.parametrize("noise", [0.0, 0.02])
def test_tracking_filter_is_exact_without_noise_and_honest_with_it(noise):
    """0.5 s of detections of a known throw: exact landing without noise; with 2 cm noise the landing
    error stays within 3 sigma of the filter's own landing uncertainty."""
    from ball_flight import BallFlight
    from perception import BallTracker, PerceptionConfig
    fl = BallFlight(0.022, 0.037, 0.5)
    rng = np.random.default_rng(1)
    p0, v0 = np.array([-1.7, 0.0, 1.0]), np.array([3.0, 0.2, 5.0])
    truth = fl.crossing(p0, v0, 0.4)[0]
    tr = BallTracker(fl, PerceptionConfig().accel_sigma)
    p, v, dt = p0.copy(), v0.copy(), 1 / 120
    for k in range(60):
        z = p + rng.normal(0, noise, 3) if noise else p.copy()
        tr.update(k * dt, z, max(noise, 1e-3) ** 2 * np.eye(3))
        for _ in range(100):                              # fine steps: the true path, not an Euler approximation
            a = fl.accel(v)
            p, v = p + v * dt / 100, v + a * dt / 100
    x, P = tr.state_at(60 * dt)
    est = fl.crossing(x[:3], x[3:], 0.4)[0]
    err = float(np.linalg.norm(est[:2] - truth[:2]))
    if not noise:
        assert err < 0.002
        return
    J = np.zeros((2, 6))
    for k in range(6):
        dx = np.zeros(6)
        dx[k] = 1e-3
        J[:, k] = (fl.crossing(x[:3] + dx[:3], x[3:] + dx[3:], 0.4)[0][:2] - est[:2]) / 1e-3
    sigma = float(np.sqrt(np.max(np.linalg.eigvalsh(J @ P @ J.T))))
    assert err < 3 * sigma


def test_intercept_prefers_stopping_and_respects_the_gate():
    from intercept import SafetyZone, choose_intercept
    zone = SafetyZone(half_x=2.5, half_y=2.0)
    kw = dict(a=2.4, v_max=1.8, zone=zone, coast=0.05, a_ball=3.0)
    near = choose_intercept((1.0, 0.0), (0.0, 0.0), (1.3, 0.0), 0.8, **kw)
    assert near.v_f == 0.0 and near.feasible                             # plenty of time: stop-and-catch
    far = choose_intercept((1.0, 0.0), (0.0, 0.0), (1.0, 0.7), 0.85, **kw)
    assert 0.3 < far.v_f < 1.8 and far.feasible and not far.gated        # can't stop in time: pass through slowly
    wall = choose_intercept((1.5, 0.0), (0.0, 0.0), (2.0, 0.0), 0.5, **kw)
    assert wall.gated and wall.v_f < 0.7                                 # toward the netting: the gate caps it


def test_catch_and_return():
    p = SimParams()
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, seed=3, verbose=False)
    while len(game.results) < 3 or game.state != "wait":
        game.step()
    assert all(res.caught for res in game.results)
    assert all(res.launched for res in game.results)
    assert np.nanmedian([res.return_error for res in game.results]) < 0.4
