"""Headless checks for the v11 sim: the v3-v10 checks it inherits (v10's sensor parts, the ROS bridge's core, the map
scorer), plus v11's firmware in the loop and the robot's catch code (design.md 30): the SIL library and its protocol
against the Python side, the DM-2325's CAN behaviour, the firmware's safety rules, the detector, the robot config
against the sim's geometry, and a short catch game through the whole stack. No ROS needed (ros/omni.sh test runs the
same in WSL). design.md 31: the house scene's decoration leaves the physics alone, a rendered frame pair shows the ball
where it projects (Windows only), acquisition starts on the moving ball, not on still clutter.
Run: python -m pytest -q   (a few minutes)"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

from catch_game import CatchGame
from launcher_bench import Bench
from omni_control import OmniKinematics
from params import SimParams
from robot import Robot
from robot_model import build_model, geometry, mass_properties
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


# --- v5 ----------------------------------------------------------------------------

@pytest.mark.parametrize("battery,limits,lo,hi", [
    ("7S1P-P50B", "firmware", 1.3, 1.45),
    ("7S1P-P50B", "physics", 3.0, 3.8),
    ("6S1P-OVONIC-5200", "firmware", 1.3, 1.45),
    ("6S1P-OVONIC-5200", "physics", 2.7, 3.2),
])
def test_top_speed_follows_the_limits_profile(battery, limits, lo, hi):
    """The 96 mm test robot retains its firmware cap; uncapped speed depends on the selected pack's voltage."""
    p, m, d, r = make(SimParams(home_xy=(0.0, 0.0), area_size=(30.0, 30.0), limits=limits, battery=battery))
    for _ in range(int(2.0 / m.opt.timestep)):
        r.drive_velocity((20.0, 0.0), 0.0, r.pose()[2])
        r.step()
    v = float(np.hypot(*d.sensor("base_linvel").data[:2]))
    assert lo < v < hi


def test_sphere_image_and_blob_detector():
    """On-axis sphere radius is f tan(asin(R/d)); a rendered static blob is found within a fraction of a pixel."""
    from ball_flight import BallFlight
    from vision import VisionConfig, VisionPerception, sphere_image
    du, dv, b = sphere_image(np.array([0.0, 0.0, -2.0]), 0.037, 571.0)
    assert du == dv == 0.0 and b == pytest.approx(571.0 * math.tan(math.asin(0.037 / 2.0)), rel=1e-6)
    p, m, d, r = make(SimParams(home_xy=(0.0, 0.0), area_size=(20.0, 20.0)))
    vp = VisionPerception(m, d, BallFlight(0.022, 0.037, 0.5), p, VisionConfig(drop_prob=0.0), np.random.default_rng(0))
    vp.keep_stats = True
    c = d.cam_xpos[vp.cams[0]]
    R = d.cam_xmat[vp.cams[0]].reshape(3, 3)
    bq = m.joint("ball").qposadr[0]
    for rng_m in (1.0, 2.5):
        d.qpos[bq:bq + 3] = c + R @ np.array([0.05, 0.03, -rng_m])
        mujoco.mj_forward(m, d)
        assert vp._render_blob(0, 0.0) is not None
    s = np.array(vp.stats)
    assert np.all(s[:, 2] < 0.4) and np.all(np.abs(s[:, 3]) < 0.4)


def test_catch_with_vision_model():
    from catch_game import GameConfig
    p = SimParams()
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, cfg=GameConfig(perception="vision", catch_mode="moving", return_throws=False),
                     seed=2, verbose=False)
    while len(game.results) < 3:
        game.step()
    assert all(res.caught for res in game.results)
    assert np.nanmedian([res.pos_error for res in game.results]) < 0.06


def test_magazine_tube_fires_every_ball():
    """Single file in the tube: no two balls can arch over the throat; 4 balls, 4 shots, no double feeds."""
    import sys
    sys.path.insert(0, "studies")
    from hopper_rig import burst
    b = burst((4, -15.0, 3000, 0.4, "none", dict(hopper="tube")))
    assert b["fired"] == 4 and b["doubles"] == 0 and b["lost"] == 0
    assert b["speed_spread"] < 0.05


# --- v6 ----------------------------------------------------------------------------

def test_roller_feed_holds_the_ball_past_the_launch_angle_and_fires_one_per_trigger():
    """Stopped feed wheels close the pocket: at a 35 deg catch tilt (the floor tips forward) the ball stays put with
    the flywheels running; one trigger fires one ball even with a second one waiting above it."""
    import sys
    sys.path.insert(0, "studies")
    from hopper_rig import load
    p = SimParams(n_balls=3, hopper="tube")
    m, d, r, bodies, qadr, vadr = load(p, 2)
    r.fly_target = 3000 * 2 * math.pi / 60
    r.tilt_target = math.radians(35)
    for _ in range(int(1.0 / m.opt.timestep)):
        r.step()
    assert r.ball_held()
    r.tilt_target = math.radians(-15)
    for _ in range(int(0.8 / m.opt.timestep)):
        r.step()
    gone = lambda: sum(float(np.linalg.norm(d.qvel[v:v + 3])) > 3.0 for v in vadr)   # noqa: E731
    r.gate_open = True
    fired = 0
    for _ in range(int(0.6 / m.opt.timestep)):
        r.step()
        fired = max(fired, gone())
    assert fired == 1 and r.trigger_done()


def test_feed_wheel_mounts_sit_on_their_stops():
    """The preloaded mounts must not sag inward past the stop (a soft limit let the wheels catch a dropping ball
    above the floor, which scattered return throws by 0.5 m)."""
    p, m, d, r = make()
    for _ in range(int(0.3 / m.opt.timestep)):
        r.step()
    for s in "LR":
        assert d.qpos[m.joint(f"feed_slide_{s}").qposadr[0]] > -0.0005


def test_v6_module_catches_a_fast_flat_arrival():
    import sys
    sys.path.insert(0, "studies")
    from catch_rig import trial
    b = Bench(SimParams())
    assert all(trial(b, 8.0, th, 0.6, 35.0, (0.0, 0.0)) for th in (40.0, 55.0))


def test_honest_track_start_at_7_m():
    """A track started on a ball 7 m away carries the stereo geometry's depth uncertainty (range^2), not v5's
    5 cm: the first landing estimate's error is within 3 sigma of its own uncertainty."""
    from ball_flight import BallFlight
    from catch_game import GameConfig
    from vision import VisionConfig
    p = SimParams(home_xy=(3.5, 0.0), thrower_xy=(-3.8, 0.0), area_size=(12.0, 6.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, cfg=GameConfig(perception="vision", vision_cfg=VisionConfig(), return_throws=False,
                                             apex_range=(1.0, 1.0), catch_zone_radius=0.0), seed=4, verbose=False)
    while not (game.state == "flight" and game.perc.ready()):
        game.step()
    z = game.robot.g.rim_center_robot(0.0)[1]
    cr = game.perc.crossing(d.time, z)
    p0, v0 = game.ball()
    truth = BallFlight(p.ball_spec.mass, p.ball_spec.radius, p.ball_spec.drag_cd).crossing(p0, v0, z)[0]
    err = cr[0][:2] - truth[:2]
    assert float(err @ np.linalg.solve(cr[3] + 1e-9 * np.eye(2), err)) < 3.0 ** 2 * 2


def test_indexer_drops_one_pocket_into_the_launch_pocket():
    p, m, d, r = make(SimParams(home_xy=(0.0, 0.0), hopper="indexer", feed="roller"))
    for _ in range(int(0.3 / m.opt.timestep)):
        r.step()
    mb = r.module
    z0 = r.g.tube_top + p.indexer_plate
    bq, bv = m.joint("ball").qposadr[0], m.joint("ball").dofadr[0]
    d.qpos[bq:bq + 3] = d.xpos[mb] + d.xmat[mb].reshape(3, 3) @ np.array([0.0, p.indexer_pitch, z0 + 0.0375])
    d.qvel[bv:bv + 6] = 0
    mujoco.mj_forward(m, d)
    for _ in range(int(0.4 / m.opt.timestep)):
        r.step()
    assert r.ball_aboard() and not r.ball_held()
    r.index_step(-1)                         # the 90 deg pocket over the exit
    for _ in range(int(1.0 / m.opt.timestep)):
        r.step()
    assert r.ball_held() and r.index_jams == 0


def test_return_throw_clears_an_8_ft_ceiling_at_7_m():
    """Indoors the return lob flattens until its apex is below the ceiling minus the margin, and still reaches 7 m."""
    from catch_game import GameConfig
    p = SimParams(home_xy=(3.5, 0.0), thrower_xy=(-3.8, 0.0), area_size=(12.0, 6.0), ceiling_height=2.44)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, cfg=GameConfig(ceiling=2.44), seed=1, verbose=False)
    pose = game.robot.pose()
    game._plan_return(np.array([*pose[:2], game._face(pose, game.hand)]), final=False)
    assert game.ret_apex_plan < 2.44 - 0.25 + 1e-6
    assert 30.0 <= math.degrees(game.elev) < 45.0
    assert 9.5 < game.v_needed < 12.2                 # inside the 5000 rpm launcher calibration


# --- v6 round 3: real feed motors, viscoelastic contacts, randomized worlds, real floors -------------------------

def test_feed_motor_constants_follow_the_datasheet():
    """kt, ke and R from the five datasheet points reproduce them: no-load speed at the rated voltage and stall torque."""
    from catalog import FEED_MOTORS
    for key in ("Pololu-HPCB-30", "Pololu-25D-9.7", "GM2804", "GM3506"):
        fm = FEED_MOTORS[key]
        assert fm.ke * fm.w_noload + fm.i_noload * fm.r == pytest.approx(fm.v_rated, rel=1e-9)
        assert fm.kt * (fm.stall_current - fm.i_noload) == pytest.approx(fm.stall_torque, rel=1e-9)
    w = 1.6 / 0.02                                     # the chute's 1.6 m/s on 40 mm wheels
    assert FEED_MOTORS["GM3506"].torque_at(w) > 0.06
    assert FEED_MOTORS["Pololu-HPCB-30"].torque_at(w) < 0.02      # too weak at that speed
    assert FEED_MOTORS["N20-worm-380"].torque_at(w) == 0.0        # can't reach it


def test_gm3506_feeds_at_the_steepest_launch_and_holds_a_fast_catch():
    import sys
    sys.path.insert(0, "studies")
    from catch_rig import trial
    b = Bench(SimParams(feed_motor="GM3506"))
    shot = b.shot(3000, -30)
    assert shot.ok and 7.0 < shot.exit_speed < 8.2
    for f in b.r.feed_drives:
        f.slip_max = 0.0
        f.q_hold = None
    assert trial(b, 8.0, 50.0, 0.6, 35.0, (0.0, 0.0))
    assert max(f.slip_max for f in b.r.feed_drives) < math.radians(15)     # back-driven < 5 mm at the tire


def test_hunt_crossley_restitution_falls_with_impact_speed():
    import contact_model
    b = SimParams().ball_spec
    hc = contact_model.fit(b.stiffness, b.mass, b.e_slow, b.e_fast)
    e = [contact_model.drop_restitution(hc, v) for v in (1.0, 3.0, 5.0, 7.0)]
    assert e[0] == pytest.approx(b.e_slow, abs=0.01) and e[-1] == pytest.approx(b.e_fast, abs=0.01)
    assert all(a > c for a, c in zip(e, e[1:]))


def test_viscoelastic_ball_bounces_lower_when_thrown_harder_in_the_scene():
    p, m, d, r = make()
    bq, bv = m.joint("ball").qposadr[0], m.joint("ball").dofadr[0]
    out = []
    for v in (1.5, 6.0):
        d.qpos[bq:bq + 7] = [-1.5, 1.2, p.ball_spec.radius + 0.002, 1, 0, 0, 0]
        d.qvel[bv:bv + 6] = [0, 0, -v, 0, 0, 0]
        vz = -v
        for _ in range(int(0.06 / m.opt.timestep)):
            r.step()
            vz = max(vz, float(d.qvel[bv + 2]))
        out.append(vz / v)
    assert out[0] > out[1] + 0.05 and 0.3 < out[1] < 0.5


def test_randomized_world_changes_the_ball_but_not_what_the_robot_assumes():
    import randomize
    w = dict(randomize.nominal(), ball_diameter=0.0755, ball_mass=0.025, floor_mu=0.8)
    p, m, d, r = make()
    randomize.apply(r, w)
    g = m.geom("ball").id
    assert m.geom_size[g][0] == pytest.approx(0.03775) and m.body_mass[m.geom_bodyid[g]] == pytest.approx(0.025)
    floor = m.geom("floor").id
    assert all(m.pair_friction[k][0] == pytest.approx(0.8) for k in range(m.npair) if m.pair_geom1[k] == floor)
    assert p.ball_spec.diameter == 0.074 and r.g.rb == pytest.approx(0.037)      # the robot is still built for 74 mm


def test_rolling_resistance_costs_current_but_not_odometry():
    """Rolling resistance as load-dependent joint drag: the base still goes where its wheels say (no lost grip, as
    MuJoCo rolling friction in the elliptic cone caused), and cruising draws more current."""
    res = {}
    for crr in (0.0, 0.02):
        p, m, d, r = make(SimParams(home_xy=(0.0, 0.0), wheel_crr=crr))
        for _ in range(int(0.3 / m.opt.timestep)):
            r.step()
        x0, odo, i = r.pose()[0], 0.0, []
        for k in range(int(1.5 / m.opt.timestep)):
            r.drive_velocity((1.0, 0.0), 0.0, r.pose()[2], a_cap=2.0)
            r.step()
            odo += m.opt.timestep * r.kin.body_twist(r.wheel_speeds())[0]
            i.append(r.i_total)
        res[crr] = (odo / (r.pose()[0] - x0), float(np.mean(i[len(i) // 2:])))
    assert res[0.02][0] == pytest.approx(1.0, abs=0.01)
    assert res[0.02][1] > res[0.0][1] + 0.02


def test_rollers_cross_a_12_mm_threshold_with_the_ball_aboard():
    import sys
    sys.path.insert(0, "studies")
    from floor_wheels import cross_job
    out = cross_job(("rollers", "threshold 12 mm", 0.0, 1.0))
    assert out["crossed"] and out["held"] and out["tilt"] < 15     # pitches ~10 deg at 1 m/s (3 deg geometric)


def test_slow_zone_caps_the_speed_before_a_threshold():
    p, m, d, r = make(SimParams(home_xy=(0.0, 0.0), limits="firmware", base_max_speed=2.0,
                                slow_zones=((1.5, -2.0, 1.6, 2.0, 1.0),)))
    v_in = []
    for _ in range(int(2.0 / m.opt.timestep)):
        r.drive_velocity((2.0, 0.0), 0.0, r.pose()[2])
        r.step()
        if 1.2 < r.pose()[0] < 1.9:
            v_in.append(float(np.linalg.norm(d.sensor("base_linvel").data[:2])))
    assert v_in and max(v_in) < 1.1


def test_skirt_reaches_below_a_resting_balls_centre_and_touches_the_balls():
    """The bumper ring alone meets a ball above its centre, so a robot pitched up by a threshold rode over a missed
    ball and turned over: the skirt must reach below the centre, and the balls (only) must collide with it and with
    the wheel guards."""
    from robot_model import guard_geoms
    p, m, d, r = make(SimParams(home_xy=(0.0, 0.0), n_balls=2))
    low = min(d.geom_xpos[m.geom(f"skirt{k}").id][2] - m.geom_size[m.geom(f"skirt{k}").id][0] for k in range(36))
    assert low < p.ball_spec.radius - 0.01
    ball_ids = {m.geom(n).id for n in ("ball", "ball_1", "ball_2")}
    paired = {(int(m.pair_geom1[k]), int(m.pair_geom2[k])) for k in range(m.npair)}
    for gn in guard_geoms(p):
        g = m.geom(gn).id
        assert m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0     # no floor, obstacle or module contacts
        assert all((b, g) in paired for b in ball_ids)


def test_floor_friction_field_and_patches():
    from floor import FloorFriction
    p, m, d, r = make(SimParams(home_xy=(0.0, 0.0), floor_mu_field=(0.1, 0.5, 1), floor_patches=((1, -1, 2, 1, 0.5),)))
    ff = next(h for h in r.floor if isinstance(h, FloorFriction))
    xy = np.random.default_rng(0).uniform(-2, 0, (400, 2))
    mu = ff.mu_at(xy)
    assert 0.06 < np.std(mu) < 0.14 and abs(np.mean(mu) - p.mu) < 0.05
    assert ff.mu_at(np.array([1.5, 0.0]))[0] == pytest.approx(0.5)


@pytest.mark.parametrize("dia", [72.0, 76.0])
def test_chute_feeds_undersized_and_oversized_balls_at_the_steepest_launch(dia):
    """The chute floor slopes down into the feed wheels' nip at every launch tilt, so a ball 2 mm under or over the
    nominal 74 mm still rests against both wheels and is fired (the round-1 roller pocket missed undersized ones)."""
    b = Bench(SimParams(feed="chute"))
    gid = b.m.geom("ball").id
    b.m.geom_size[gid][0] = b.m.geom_rbound[gid] = dia / 2000.0
    shot = b.shot(3000, -30)
    assert shot.ok and 6.5 < shot.exit_speed < 8.5
    assert shot.elevation == pytest.approx(59.0, abs=1.5)
    assert shot.spin < 40


# -- v7: home floors (design.md 25) -------------------------------------------------------------

def _cross(cfg, obstacle, direction, speed, **extra):
    import sys
    sys.path.insert(0, "studies")
    from home_floor import cross_job
    return cross_job((cfg, obstacle, direction, speed, extra))


@pytest.mark.parametrize("kw", [dict(), dict(suspension="pod"), dict(suspension="pod", rocker="both", rocker_deg=6.0),
                                dict(suspension="pod", swing="parallel"), dict(wheel="Printed-152-2x7-36", suspension="pod")])
def test_wheel_loads_from_the_pod_force_sensors_sum_to_the_weight(kw):
    b = Bench(SimParams(wheel_model="rollers", **kw))
    b.run(0.6)
    loads = b.r.wheel_loads()
    assert loads.sum() == pytest.approx(b.r.mass * 9.81, rel=0.02)
    assert np.all(loads > 3.0)


def test_mecanum_kinematics_drive_straight():
    p = SimParams(wheel="goBILDA-140-mecanum", wheel_model="rollers", area_size=(14.0, 14.0))
    assert np.linalg.matrix_rank(OmniKinematics(p.wheel_angles_deg, 0.15, p.wheel_radius, 45.0).J) == 3
    b = Bench(p)
    for _ in range(int(1.5 / b.dt)):
        b.r.drive_velocity((0.0, 1.0), 0.0, b.r.pose()[2], a_cap=2.0)
        b.r.step()
    v = b.d.sensor("base_linvel").data[:2]
    assert v[1] == pytest.approx(1.0, abs=0.05) and abs(v[0]) < 0.05


def test_floor_edges_of_turned_and_beveled_rugs():
    from floor import Carpet, Threshold, edges
    p = SimParams(floor_features=(Carpet(0, 0, 2, 1, yaw_deg=90.0, bevel=0.03), Threshold(3, 0, height=0.012)))
    e = edges(p)
    assert len(e) == 5
    assert [x[5] for x in e[:4]] == [False, False, True, True]        # beveled x edges, square y edges
    assert e[0][:2] == pytest.approx((1.5, -0.5))                    # (0, 0) turned 90 deg about the centre (1, 0.5)
    assert not e[4][5] and e[4][4] == 0.012


def test_square_12_mm_rug_at_45_deg_stops_the_v6_base_but_not_big_rollers():
    """The finding behind v7: at body 45 deg two X4 wheels meet a square edge on their rollers alone; a 16 mm roller
    can't climb 12 mm, a 36 mm one can."""
    assert not _cross("v6", "rug edge 12 mm", 45.0, 0.5)["crossed"]
    out = _cross("printed 127 2x5 36 mm + pods", "rug edge 12 mm", 45.0, 0.5)
    assert out["crossed"] and out["held"]


def test_terrain_assist_turns_the_base_to_cross_a_square_edge():
    out = _cross("v6", "rug edge 12 mm", 45.0, 1.0, terrain_assist=True)
    assert out["crossed"] and out["held"]
    assert abs(abs(out["heading"]) - 45.0) < 5.0          # it turned 45 deg to meet the edge at body 0 / 90 deg


def test_pods_stop_the_hop_over_a_12_mm_threshold_at_2_m_s():
    v6 = _cross("v6", "threshold 12 mm", 0.0, 2.0)
    pods = _cross("v6 + pods", "threshold 12 mm", 0.0, 2.0)
    assert pods["crossed"] and pods["held"] and pods["airborne_ms"] == 0.0
    assert pods["tilt"] < 0.7 * v6["tilt"]


def test_house_scene_builds_and_drives():
    import house
    drv = house.Driver(house.house_params("v7"))     # (v8's house defaults to the swerve; this checks the X4)
    drv.teleport(house.SPOTS["2"])                           # in front of the kitchen reducer
    for _ in range(int(1.5 / drv.m.opt.timestep)):
        drv.step(np.array([1.0, 0.0]), 0.0, True)
    assert drv.r.pose()[0] > 0.4 and drv.r.ball_held()
    assert "tile" in drv.under_wheels()


# -- v8: swerve (design.md 26) ---------------------------------------------------------------------

def test_swerve_kinematics_flip_and_odometry():
    from omni_control import SwerveKinematics
    k = SwerveKinematics((60, 180, 300), 0.15, 0.05)
    k.steer = np.zeros(3)
    w = k.wheel_speeds((-0.5, 0.0, 0.0))            # straight back: flip the wheels, don't turn the modules 180 deg
    assert np.allclose(k.target, 0.0) and np.allclose(w, -10.0)
    w = k.wheel_speeds((0.3, -0.2, 1.0))
    k.steer = k.target.copy()
    assert np.allclose(k.body_twist(w), (0.3, -0.2, 1.0))


@pytest.mark.parametrize("ang", [0.0, 90.0, 225.0])
def test_swerve_drives_any_direction(ang):
    p = SimParams(layout="swerve3", wheel="Printed-100-tire", drive_motor="DM-2325", suspension="pod",
                  area_size=(14.0, 14.0))
    b = Bench(p)
    u = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang))])
    for _ in range(int(1.5 / b.dt)):
        b.r.drive_velocity(1.0 * u, 0.0, b.r.pose()[2], a_cap=2.0)
        b.r.step()
    v = b.d.sensor("base_linvel").data[:2]
    assert np.linalg.norm(v - u) < 0.05
    assert b.r.wheel_loads().sum() == pytest.approx(b.r.mass * 9.81, rel=0.1)


def test_swerve_climbs_the_16_mm_plush_rug_at_45_deg_where_the_omni_x4_sticks():
    assert not _cross("v7 on DM-2325", "plush rug 16 mm", 45.0, 0.5)["crossed"]
    out = _cross("swerve3 100", "plush rug 16 mm", 45.0, 0.5)
    assert out["crossed"] and out["held"] and out["odo_err"] < 0.1


def test_house_drives_the_swerve():
    import house
    drv = house.Driver(house.house_params("v8"))
    drv.teleport(house.SPOTS["6"])                           # in front of the 16 mm plush rug
    for _ in range(int(1.5 / drv.m.opt.timestep)):
        drv.step(np.array([1.0, 0.0]), 0.0, True)
    assert drv.r.ball_held() and "rug 16 mm" in drv.under_wheels()


# -- v9: sensors and localization (design.md 27) -----------------------------------------------------

def _house(base="v7", **cfg):
    import house
    from sensors import Sensors, SensorConfig
    drv = house.Driver(house.house_params(base, assist=False))
    return drv, Sensors(drv.r, SensorConfig(**cfg))


def test_imu_noise_at_rest_matches_the_datasheet():
    drv, sn = _house(seed=3)
    g = []
    for _ in range(4000):
        drv.step(np.zeros(2), 0.0, False)
        rd = sn.step()
        if rd.imu is not None:
            g.append(rd.imu[1])
    sigma = 2.8e-3 * math.pi / 180 * math.sqrt(500)          # 2.8 mdps/sqrt(Hz) over a 500 Hz band
    assert np.std(g) == pytest.approx(sigma, rel=0.25)


def test_lidar_scan_has_a_turn_of_samples_and_ranges_a_wall():
    import house
    drv, sn = _house(seed=1)
    drv.teleport(house.SPOTS["9"])                            # hallway, a solid wall 0.7 m to the left
    scans = []
    while len(scans) < 3:
        drv.step(np.zeros(2), 0.0, False)
        rd = sn.step()
        if rd.scan is not None:
            scans.append(rd.scan)
    s = scans[-1]
    assert 480 <= len(s.rng) <= 520 and s.t[-1] - s.t[0] == pytest.approx(0.1, abs=0.01)
    assert np.isfinite(s.rng).mean() > 0.8
    # straight left (y 1.25 -> the wall face at y 1.95)
    k = int(np.argmin(np.abs(s.ang - math.pi / 2)))
    assert s.rng[k - 2:k + 3][np.isfinite(s.rng[k - 2:k + 3])].mean() == pytest.approx(0.70, abs=0.06)


def test_flow_drops_out_too_fast_or_too_close():
    from sensors import Sensors, SensorConfig
    drv, sn = _house(seed=2)
    v = []
    for k in range(int(2.5 / drv.m.opt.timestep)):
        drv.step(np.array([2.0, 0.0]), 0.0, False)              # 2 m/s: over the PAA5100JE's ~1.1 m/s at 25 mm
        rd = sn.step()
        if rd.flow is not None and drv.d.time > 1.8:
            v.append(rd.flow[1])
    assert v and all(x is None for x in v)


def test_scan_matcher_finds_the_pose_to_about_a_centimetre():
    from estimator import Localizer
    drv, sn = _house(seed=1)
    loc = Localizer(drv.r, sn, variants=["all"])
    hist = lambda t: drv.r.pose()                             # noqa: E731  (standing still: a perfect de-skew)
    for _ in range(int(0.5 / drv.m.opt.timestep)):
        drv.step(np.zeros(2), 0.0, False)
        rd = sn.step()
        if rd.scan is not None:
            pose, cov, (frac, rms) = loc.matcher.match(rd.scan, hist, init=drv.r.pose() + np.array([0.04, -0.03, 0.02]))
    assert np.linalg.norm(pose[:2] - drv.r.pose()[:2]) < 0.02 and frac > 0.9


def test_fused_estimate_survives_pushing_a_wall():
    from estimator import Localizer
    drv, sn = _house(seed=1)
    loc = Localizer(drv.r, sn, variants=["wheels+IMU", "all"])
    for v, dur in (((1.0, 0.0), 1.5), ((0.0, 0.6), 2.5), ((0.0, -0.4), 1.0)):
        for _ in range(int(dur / drv.m.opt.timestep)):
            drv.step(np.array(v), 0.0, False)
            loc.step()
    e = loc.errors()
    assert e["all"][0] < 0.04 and e["wheels+IMU"][0] > 0.2


# -- v10: the parts Fernando chose, the ROS bridge's core, the map scorer (design.md 28) ---------------------

def _parts_house(seed=0, **kw):
    import house
    from sensors import Sensors, SensorConfig
    drv = house.Driver(house.house_params("v7", assist=False))
    return drv, Sensors(drv.r, SensorConfig.parts("BNO085", "A2M12", seed=seed, **kw))


def test_a2m12_scans_are_whole_turns_and_noise_grows_with_range():
    import house
    drv, sn = _parts_house(seed=1, lidar_flatness=0.0)
    drv.teleport(house.SPOTS["9"])                            # hallway: the wall 0.7 m left, the far end ~11 m ahead
    scans = []
    while len(scans) < 6:
        drv.step(np.zeros(2), 0.0, False)
        rd = sn.step()
        if rd.scan is not None:
            scans.append(rd.scan)
    for s in scans[1:]:
        assert len(s.rng) == 1600 and s.ang[0] == 0.0          # exactly one turn, starting at angle 0
        assert np.allclose(np.diff(s.t), 1 / 16000)
    assert len(scans[0].rng) <= 1600                           # even after the teleport's jump in time
    R = np.array([s.rng for s in scans[1:]])
    near = R[:, int(round(90 / 0.225)) - 2:int(round(90 / 0.225)) + 3].ravel()     # left wall, 0.7 m
    far = R[:, :3].ravel()                                                            # ahead, ~11 m
    near, far = near[np.isfinite(near)], far[np.isfinite(far)]
    assert np.std(near) < 0.01 and np.std(far) > 0.04         # 1% of range / 2.5 at 0.7 m; 2.5% / 2.5 at ~11 m


def test_a2m12_tilted_scan_plane_sees_the_floor():
    drv, sn = _parts_house(seed=0, lidar_flatness=0.0)
    drv.teleport((0.5, -1.0, 0.0))                             # open floor: every wall > 3 m away, west 6.6 m
    sn.flat_tilt = (math.tan(math.radians(3.0)), 0.0)          # a 3 deg unit, lowest straight back (west)
    scans = []
    while len(scans) < 2:
        drv.step(np.zeros(2), 0.0, False)
        rd = sn.step()
        if rd.scan is not None:
            scans.append(rd.scan)
    s = scans[-1]
    h = drv.d.site_xpos[drv.m.site("lidar").id][2]
    r_floor = h / math.tan(math.radians(3.0))                   # ~2.5 m
    back = np.abs(np.angle(np.exp(1j * (s.ang - math.pi)))) < math.radians(10)
    r = s.rng[back]
    assert np.nanmedian(r) == pytest.approx(r_floor, rel=0.1)   # it returns the floor, not the wall 6.6 m away


def test_bno085_game_rotation_vector_drifts_with_its_gyro_bias():
    drv, sn = _parts_house(seed=4)
    reps = []
    while drv.d.time < 4.0:
        drv.step(np.zeros(2), 0.0, False)
        rd = sn.step()
        if rd.imu3 is not None:
            reps.append(rd.imu3)
    assert len(reps) == pytest.approx(400 * (4.0 - reps[0][0]), abs=3)        # 400 Hz
    t, q, g, a = reps[-1]
    assert abs(np.linalg.norm(q) - 1) < 1e-9 and a[2] == pytest.approx(9.81, abs=4 * sn.cfg.accel_bias)
    yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    drift = (t - reps[0][0]) * sn.bg                                         # the residual z bias, integrated
    assert yaw == pytest.approx(drift, abs=math.radians(0.4))                  # + 0.1 deg orientation noise
    assert abs(sn.bg) < math.radians(4 * 0.5 / 60)                             # ~0.5 deg/min (reseller, VERIFY)


def test_bridge_core_events_odometry_and_cmd_timeout():
    from bridge_core import CoreConfig, SimCore
    core = SimCore(CoreConfig(shadow=()))
    n = dict(imu=0, odom=0, scan=0, truth=0)
    p0 = core.r.pose().copy()
    t0 = core.d.time
    while core.d.time - t0 < 1.0:
        core.set_cmd(0.5, 0.0, 0.0)
        ev = core.step()
        for k in n:
            n[k] += getattr(ev, k) is not None
    assert n["imu"] == pytest.approx(400, abs=2) and n["odom"] == pytest.approx(50, abs=1)
    assert n["truth"] == pytest.approx(50, abs=1) and n["scan"] in (9, 10, 11)
    moved = core.r.pose()[:2] - p0[:2]
    assert np.linalg.norm(core.odom[:2] - moved) < 0.02 and moved[0] > 0.3      # wheel odometry follows the truth
    # no more commands: after cmd_timeout the firmware ramps to a stop (never a step to zero)
    vs = []
    t1 = core.d.time
    while core.d.time - t1 < 1.5:
        core.step()
        vs.append(float(np.linalg.norm(core.d.sensor("base_linvel").data[:2])))
    k = int(round(0.05 / core.dt))                              # deceleration over 50 ms (the rollers buzz per step)
    v50 = np.array(vs[::k])
    a_max = float(np.max(-np.diff(v50) / 0.05))
    assert vs[int(0.4 / core.dt)] > 0.4                         # still moving inside the 0.5 s timeout
    # within the firmware's direction-aware table (v7: up to 4.3 m/s^2 straight ahead), and not a step (> 0.1 s)
    assert vs[-1] < 0.02 and a_max < 1.2 * max(core.p.accel_table[1])
    moving = np.array(vs) > 0.05
    assert (np.flatnonzero(moving)[-1] - np.flatnonzero(np.array(vs) < 0.45)[0]) * core.dt > 0.1


def test_map_quality_scores_a_perfect_map_zero_and_finds_a_shift(tmp_path):
    import map_quality as Q
    mp, walls, z = Q.truth_map()
    nx, ny = mp.occ.shape
    img = np.where(mp.occ.T[::-1], 0, 254).astype(np.uint8)
    (tmp_path / "m.pgm").write_bytes(f"P5\n{nx} {ny}\n255\n".encode() + img.tobytes())
    (tmp_path / "m.yaml").write_text(f"image: m.pgm\nresolution: {mp.res}\norigin: [{mp.origin[0]}, {mp.origin[1]}, 0]\n"
                                     "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    s = Q.score(tmp_path / "m.yaml", (0.0, 0.0, 0.0), truth=(mp, walls, z))
    assert s["anchored"]["med"] == 0 and s["coverage_anchored"] > 0.99
    s = Q.score(tmp_path / "m.yaml", (0.15, 0.0, 0.0), truth=(mp, walls, z))      # a map 15 cm off
    assert s["anchored"]["p95"] > 0.1 and s["fit"]["dx"] == pytest.approx(-0.15, abs=0.01)
    assert s["best_fit"]["med"] == 0


# -- v11: the firmware in the loop and the robot's catch code (design.md 30) ----------------------------------------

def _sil():
    import sil_link  # noqa: F401  (puts ros/src on the path)
    import sil_robot  # noqa: F401  (puts firmware/sil on the path)
    import omni_fw_sil as FW
    from omni_mcu import protocol as PR
    return FW, PR


def sil_rig(**kw):
    from robot_export import v11_params
    from sil_link import SimLink
    from sil_robot import SilRobot
    p = v11_params(area_size=(8.0, 6.0), home_xy=(0.0, 0.0), thrower_xy=(-3.0, 0.0), **kw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = SilRobot(m, d, p, seed=1)
    return p, m, d, r, SimLink(r)


def sil_run(rig, seconds, cmd=None):
    """Step the robot; cmd(t) -> Command (or None) every 10 ms. Samples: t, physical speed, commanded vx/vy."""
    p, m, d, r, link = rig
    out, t_next, t0 = [], d.time, d.time
    while d.time - t0 < seconds:
        t = d.time
        if cmd is not None and t >= t_next - 1e-9:
            t_next = t + 0.01
            c = cmd(t)
            if c is not None:
                link.send(c, t)
        link.poll(t)
        r.step()
        out.append((d.time, float(np.linalg.norm(d.sensor("base_linvel").data[:2])), *r.fw.outputs.dbg_v_cmd))
    return np.array(out)


def test_robot_export_is_current():
    import robot_export
    assert robot_export.main(check=True), "run `python robot_export.py`: fw_params.h / robot_v11.json are stale"


def test_sil_protocol_matches_the_python_side():
    FW, PR = _sil()
    assert (FW.sizeof(2), FW.sizeof(3), FW.sizeof(4), FW.sizeof(6)) == (PR.CMD_FMT.size, PR.STATE_FMT.size,
                                                                        PR.IMU_FMT.size, PR.INFO_FMT.size)
    rng = np.random.default_rng(3)
    for n in (0, 1, 74, 116, 200):
        body = bytes(rng.integers(0, 256, n, dtype=np.uint8))
        body = body[:n // 2] + bytes(n - n // 2)                       # zeros for COBS to escape
        assert PR.frame(0x81, n & 0xFF, body) == FW.frame_encode(0x81, n & 0xFF, body)
        assert PR.crc16(body) == FW.crc16(body)


def test_firmware_state_reaches_the_host_and_time_syncs():
    FW, PR = _sil()
    fw, host = FW.Firmware(), PR.HostLink()
    boot = 3.25                                                          # the MCU booted 3.25 s before host time 0
    states, infos = [], []
    for k in range(400):
        t = k * 0.001
        if k % 10 == 0:
            fw.rx(host.encode(PR.Command(drive_mode=PR.DRIVE_IDLE), int(t * 1e9)))
        fw.inputs.t_us = int(round((t + boot) * 1e6))
        fw.inputs.v_bus = 26.0
        fw.tick()
        msgs = host.feed(fw.tx(), int((t + 0.0002) * 1e9))
        states += [m for m in msgs if isinstance(m, PR.State)]
        infos += [m for m in msgs if isinstance(m, PR.Info)]
    assert len(states) == pytest.approx(80, abs=1) and host.stats.errors == 0
    # INFO, 10 Hz: the protocol version both sides were built with and the build id the console checks
    sys.path.insert(0, str(Path(FW.__file__).resolve().parents[1]))          # firmware/ (the SIL is firmware/sil)
    from fw_build_id import build_id
    assert len(infos) == 4 and infos[-1].protocol == PR.PROTOCOL_VERSION and infos[-1].build == build_id()
    assert not infos[-1].rc_ok and infos[-1].rc == (0.0,) * 8
    assert any(s.mode_name == "AUTO" for s in states) and states[-1].cmd_count == 40
    # no drive motor answers on this bare MCU: armed, then the fault stop (firmware.h) once its grace has passed
    assert {"MOTOR_SILENT", "FAULT_STOP"} <= set(states[-1].fault_names()) and states[-1].mode_name == "DISARMED"
    assert abs(host.sync.offset_ns * 1e-9 - boot) < 0.0015              # within the 1 ms tick


def test_dm2325_feedback_codec_round_trip():
    FW, PR = _sil()
    from sil_robot import DmMotor
    mot = DmMotor(2, 0x10, (FW.param("DM_PMAX"), FW.param("DM_VMAX"), FW.param("DM_TMAX")))
    for pos, vel, tau in ((0.3, 12.5, -1.2), (-11.0, -40.0, 3.3), (25.1, 0.01, 0.0)):
        mid, err, p, v, q = FW.dm_decode(mot.feedback(pos, vel, tau, 30.0))
        assert (mid, err) == (2, 0)
        assert p == pytest.approx((pos + 12.5) % 25.0 - 12.5, abs=4e-4)    # 16 bits over +/-12.5 rad, wrapped
        assert v == pytest.approx(vel, abs=0.03) and q == pytest.approx(tau, abs=0.003)


def test_firmware_drives_then_ramps_to_a_stop_and_disarms_when_the_link_dies():
    FW, PR = _sil()
    rig = sil_rig()
    p, m, d, r, link = rig
    fwd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=1.0, flags=PR.CMD_HOLD_YAW)
    s = sil_run(rig, 2.0, lambda t: fwd)
    assert r.fw.mode == 2 and s[-1, 1] == pytest.approx(1.0, abs=0.05)
    s = sil_run(rig, 2.0, lambda t: None)                                # the Jetson goes silent
    v = s[:, 1]
    # Check the MCU ramp directly at its 1 kHz ticks. Contact/roller oscillation can briefly exceed the
    # command's acceleration (5.19 m/s² over 50 ms versus a 4.24 command); average physical braking over 100 ms.
    ticks = s[::r.fw_div]
    u = np.linalg.norm(ticks[:, 2:4], axis=1)
    assert np.max(np.abs(np.diff(u)) / np.diff(ticks[:, 0])) <= max(p.accel_table[1]) + 1e-3
    k = int(round(0.1 / m.opt.timestep))
    a_max = float(np.max(-np.diff(v[::k]) / 0.1))
    assert v[int(0.4 / m.opt.timestep)] > 0.9                            # still going inside the 0.5 s timeout
    assert v[-1] < 0.02 and a_max < 1.2 * max(p.accel_table[1])          # ramped, never a step
    assert r.fw.mode == 1 and r.fw.faults & 1                            # disarmed, LINK_LOST


def test_drive_motor_comm_loss_stops_the_base_until_its_mode_restarts():
    """The fault stop (firmware.h): a CAN cut stops and disarms the base, latched while AUTO is still asked for; a
    DISARM request (the console's IDLE) and AUTO again clear the motors' errors and drive on."""
    FW, PR = _sil()
    rig = sil_rig()
    p, m, d, r, link = rig
    fwd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.5, flags=PR.CMD_HOLD_YAW)
    disarm = PR.Command(req_mode=PR.REQ_DISARM, drive_mode=PR.DRIVE_IDLE)
    sil_run(rig, 1.0, lambda t: fwd)
    assert all(mt.err == 1 for mt in r.dm)
    r.can_cut_until = d.time + 0.25                                      # the CAN bus is cut for 0.25 s
    sil_run(rig, 0.2, lambda t: fwd)
    assert all(not mt.enabled and mt.err == 0xD for mt in r.dm)          # each motor timed out (Damiao: error D)
    assert r.fw.faults & 16                                              # the firmware saw them go silent
    s = sil_run(rig, 1.5, lambda t: fwd)
    assert r.fw.mode == 1 and r.fw.faults & 256 and s[-1, 1] < 0.05      # stopped, disarmed, latched
    sil_run(rig, 0.2, lambda t: disarm)
    assert not r.fw.faults & 256
    s = sil_run(rig, 1.5, lambda t: fwd)
    assert all(mt.enabled and mt.err == 1 for mt in r.dm)                # cleared and re-enabled
    assert s[-1, 1] == pytest.approx(0.5, abs=0.05) and not r.fw.faults & (24 | 256)


def test_bumper_stops_ramped_and_estop_disables_the_motors():
    FW, PR = _sil()
    rig = sil_rig()
    p, m, d, r, link = rig
    fwd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=1.0, flags=PR.CMD_HOLD_YAW)
    sil_run(rig, 1.5, lambda t: fwd)
    r.bumper = True
    s = sil_run(rig, 1.2, lambda t: fwd)                                 # still asked for 1 m/s
    k = int(round(0.05 / m.opt.timestep))
    assert s[-1, 1] < 0.02 and float(np.max(-np.diff(s[::k, 1]) / 0.05)) < 1.2 * max(p.accel_table[1])
    r.bumper, r.estop = False, True
    sil_run(rig, 0.3, lambda t: fwd)
    assert r.fw.mode == 4 and all(not mt.enabled for mt in r.dm)


def test_firmware_drops_a_non_finite_or_foreign_cmd():
    """cmd_ok (firmware.h): a NaN, an infinity, an unknown enum or another protocol version never reaches the base; each
    is counted and none is a heartbeat, so a host sending only those ramps to a stop on the link timeout."""
    FW, PR = _sil()
    rig = sil_rig()
    p, m, d, r, link = rig
    fwd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=1.0, flags=PR.CMD_HOLD_YAW)
    sil_run(rig, 1.5, lambda t: fwd)
    assert r.fw.mode == 2
    bad = [PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=math.nan, flags=PR.CMD_HOLD_YAW),
           PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=1.0, c=math.inf),
           PR.Command(drive_mode=7, a=1.0)]
    s = sil_run(rig, 2.0, lambda t: bad[int(t * 100) % 3])
    assert np.all(np.isfinite(s[:, 1:4]))
    assert s[-1, 1] < 0.02 and r.fw.faults & 1                           # stopped on the link timeout
    assert link.host.state.cmd_rejected >= 150
    raw = bytearray(fwd.pack(0))
    raw[-2:] = (PR.PROTOCOL_VERSION - 1).to_bytes(2, "little")         # the same CMD from other software
    n0 = link.host.state.cmd_rejected
    for k in range(50):
        r.fw.rx(PR.frame(PR.MSG_CMD, k, bytes(raw)), 0)
        sil_run(rig, 0.01)
    assert link.host.state.cmd_rejected - n0 >= 45 and r.fw.mode == 1


def test_leaving_manual_ramps_the_base_to_rest_before_disarming():
    """The RC switch off (or its link lost) mid-drive: DISARMED at once, but the motors stay enabled through a ramped
    stop, never a cut at speed (firmware.h, MANUAL)."""
    FW, PR = _sil()
    rig = sil_rig()
    p, m, d, r, link = rig
    r.rc = [0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]                      # MANUAL, stick full forward
    s = sil_run(rig, 2.0)
    assert r.fw.mode == 3 and s[-1, 1] > 0.8
    r.rc = None                                                          # the RC link drops
    s = sil_run(rig, 0.05)
    assert r.fw.mode == 1 and all(mt.enabled for mt in r.dm)             # disarmed, yet still braking
    s = sil_run(rig, 1.5)
    k = int(round(0.05 / m.opt.timestep))
    assert s[-1, 1] < 0.02 and float(np.max(-np.diff(s[::k, 1]) / 0.05)) < 1.2 * max(p.accel_table[1])
    assert all(not mt.enabled for mt in r.dm)                            # then off


def test_kid_profile_turns_off_only_at_rest():
    FW, PR = _sil()
    rig = sil_rig()
    p, m, d, r, link = rig
    kid = PR.Command(profile=PR.PROFILE_KID, drive_mode=PR.DRIVE_VEL_BODY, a=2.0, flags=PR.CMD_HOLD_YAW)
    s = sil_run(rig, 2.0, lambda t: kid)
    v_kid = s[-1, 1]
    assert link.host.state.profile == PR.PROFILE_KID
    adult = PR.Command(profile=PR.PROFILE_ADULT, drive_mode=PR.DRIVE_VEL_BODY, a=2.0, flags=PR.CMD_HOLD_YAW)
    s = sil_run(rig, 1.0, lambda t: adult)                               # asked mid-drive: refused
    assert link.host.state.profile == PR.PROFILE_KID and s[-1, 1] == pytest.approx(v_kid, abs=0.05)
    sil_run(rig, 2.0, lambda t: PR.Command(profile=PR.PROFILE_ADULT, drive_mode=PR.DRIVE_IDLE))
    assert link.host.state.profile == PR.PROFILE_ADULT                   # at rest: granted


def test_the_arm_robot_ignores_bumper_inputs_it_does_not_have():
    """HAS_BUMPERS (fw_params_arm.h: false until switches are in the CAD, todo.md)."""
    import robot_export
    from sil_link import SimLink
    from sil_robot import SilRobot
    FW, PR = _sil()
    p, m, d, _ = make(robot_export.arm_params(home_xy=(0.0, 0.0)))
    r = SilRobot(m, d, p, seed=1)
    assert r.fw.variant == "arm"
    rig = (p, m, d, r, SimLink(r))
    link = rig[4]
    fwd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.5, flags=PR.CMD_HOLD_YAW)
    sil_run(rig, 1.5, lambda t: fwd)
    r.bumper = True
    s = sil_run(rig, 0.5, lambda t: fwd)
    assert s[-1, 1] == pytest.approx(0.5, abs=0.05) and not r.fw.faults & 4


def test_fire_needs_the_flywheels_at_speed():
    FW, PR = _sil()
    rig = sil_rig()
    p, m, d, r, link = rig
    c = PR.Command(drive_mode=PR.DRIVE_IDLE, fire_seq=0)
    sil_run(rig, 0.3, lambda t: c)
    c.fire_seq = 1                                                       # fire with the flywheels off
    sil_run(rig, 0.1, lambda t: c)
    st = link.host.state
    assert st.fire_rejected == 1 and st.feed_state == 0
    c.fly_rpm = 3000.0
    sil_run(rig, 1.0, lambda t: c)
    assert link.host.state.fly_ready and link.host.state.fly_rpm[0] == pytest.approx(3000, rel=0.02)
    c.tilt = math.radians(20.0)                    # a 10 deg launch: into the face (design.md 12.2), refused
    sil_run(rig, 1.0, lambda t: c)
    c.fire_seq = 2
    sil_run(rig, 0.05, lambda t: c)
    st = link.host.state
    assert st.tilt > math.radians(15) and st.fire_rejected == 2 and st.feed_state == 0
    c.tilt = 0.0
    sil_run(rig, 1.0, lambda t: c)
    c.fire_seq = 3
    sil_run(rig, 0.05, lambda t: c)
    st = link.host.state
    assert st.fire_rejected == 2 and st.fire_seq_done == 3 and st.feed_state in (2, 3)


def test_kid_profile_caps_what_the_jetson_asks():
    FW, PR = _sil()
    rig = sil_rig()
    fast = PR.Command(profile=PR.PROFILE_KID, drive_mode=PR.DRIVE_VEL_BODY, a=2.0, flags=PR.CMD_HOLD_YAW)
    s = sil_run(rig, 2.5, lambda t: fast)
    assert 0.7 < s[-1, 1] < 0.85


def test_dshot_erpm_telemetry_resolution():
    from sil_robot import EscMotor
    for rpm in (1500, 3000, 4500, 6000):
        w = rpm * 2 * math.pi / 60
        erpm = EscMotor.erpm_telemetry(w, 7)
        assert erpm / 7 == pytest.approx(rpm, rel=0.0025)


def _disk(u, v, r, W=1280):
    uu, vv = np.meshgrid(np.arange(int(u - r - 2), int(u + r + 3)), np.arange(int(v - r - 2), int(v + r + 3)))
    ok = (uu + 0.5 - u) ** 2 + (vv + 0.5 - v) ** 2 < r * r
    return (vv[ok] * W + uu[ok]).astype(np.int64)


def test_detector_finds_the_ball_on_sparse_and_dense_frames():
    _sil()
    from omni_catch.detector import Detector
    from omni_catch.frames import ArrayFrame, SparseFrame
    det = Detector()
    fr = SparseFrame(1280, 800, _disk(400.3, 300.7, 6.0), clutter=_disk(900.0, 600.0, 9.0), seed=4)
    u, v, b, n = det.detect(fr, pred=(402.0, 299.0, 6.0))               # tracking: the window around the prediction
    assert (u, v) == (pytest.approx(400.3, abs=0.15), pytest.approx(300.7, abs=0.15)) and b == pytest.approx(6.0, abs=0.5)
    u2, v2, _, n2 = det.detect(fr)                                       # searching: the largest blob is the clutter
    assert (u2, v2) == (pytest.approx(900.0, abs=0.2), pytest.approx(600.0, abs=0.2)) and n2 > n
    dense = ArrayFrame(fr.roi(0, 0, 1280, 800))                          # the same image as a real one
    u3, v3, b3, _ = det.detect(dense, pred=(402.0, 299.0, 6.0))
    assert (u3, v3, b3) == (pytest.approx(u), pytest.approx(v), pytest.approx(b))


def test_robot_config_matches_the_sim():
    _sil()
    from omni_catch.perception import Perception
    from omni_catch.robot_config import RobotConfig
    from robot import LauncherCal
    from robot_export import v11_params
    p = v11_params()
    g, rc, cal = geometry(p), RobotConfig(), LauncherCal(p)
    for tilt in (-0.4, 0.0, 0.2385, 0.6):
        assert np.allclose(rc.rim_center(tilt), g.rim_center_robot(tilt), atol=2e-4)
        assert rc.speed_for(8.0, tilt) == pytest.approx(cal.speed_for(8.0, tilt), rel=1e-6)
        assert rc.elevation_at(8.0, tilt) == pytest.approx(cal.elevation_at(8.0, tilt), abs=1e-6)
    # the cameras where the model has them, from the base's pose
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    b = m.body("base").id
    R = d.xmat[b].reshape(3, 3)
    pc = Perception(rc)
    pc.add_pose(0.0, float(d.xpos[b][0]), float(d.xpos[b][1]), math.atan2(R[1, 0], R[0, 0]),
                math.atan2(R[2, 1], R[2, 2]), math.asin(-R[2, 0]))
    for k, (c, Rc, *_) in enumerate(pc.cams_at(0.0)):
        cid = m.camera(f"stereo_{('left', 'right')[k]}").id
        assert np.allclose(c, d.cam_xpos[cid], atol=2e-3) and np.allclose(Rc, d.cam_xmat[cid].reshape(3, 3), atol=2e-3)


def test_catch_plan_matches_the_sims_game():
    _sil()
    from catch_game import CatchGame, GameConfig
    from omni_catch.game import CatchBehavior, GameParams
    from robot_export import v11_params
    p = v11_params(home_xy=(0.0, 0.0), thrower_xy=(-6.8, 0.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    sim = CatchGame(m, d, p, GameConfig(limits="firmware", max_catch_tilt_deg=35.0), verbose=False)
    beh = CatchBehavior(params=GameParams(thrower=(-6.5, 0.0)))
    ball_p, ball_v = np.array([-3.0, 0.2, 2.1]), np.array([4.5, -0.1, -1.0])
    for v_robot in ((0.0, 0.0), (0.8, -0.3)):
        a, b = sim.catch_plan(ball_p, ball_v, v_robot), beh.catch_plan(ball_p, ball_v, v_robot)
        assert np.allclose(a[0], b[0], atol=2e-3) and a[1] == pytest.approx(b[1], abs=1e-3)


def test_catch_game_through_the_whole_stack():
    from catch_world import WorldConfig
    from sil_game import SilCatchGame
    g = SilCatchGame(WorldConfig(dist=6.5, apex_range=(1.5, 1.5), seed=9012, return_throws=True))
    res = g.run(2, t_max=40.0)
    assert len(res) == 2 and sum(r["caught"] for r in res) >= 1
    assert any(r["launched"] and r["return_error"] < 1.0 for r in res)   # a return throw reached the thrower
    st = g.link.host.stats
    assert st.errors == 0 and abs(g.link.host.sync.offset_ns * 1e-9 - g.robot.boot_offset) < 0.001
    assert g.perc.detections > 100 and not g.link.host.state.fault_names()
    g.world.close()


def test_pin_map_has_every_signal():
    FW, _ = _sil()
    rep = FW.board_report()
    for sig in ("CAN1 TX", "BNO085 CS", "RoboClaw", "STS3250", "DShot ch1", "DShot ch2", "catch beam", "e-stop",
                "SYNC_OUT", "CAM_TRIG", "CAM_STROBE", "BNO085 BOOTN"):
        assert sig in rep
    assert "8 spare" in rep


# -- v11: rendered cameras in a house (design.md 31) ----------------------------------------------------------------

def _catch_params(scene):
    from catch_world import AREA, HOME
    from robot_export import v11_params
    return v11_params(home_xy=HOME, thrower_xy=(HOME[0] - 6.8, 0.0), area_size=AREA, scene=scene)


def test_house_decoration_never_collides_and_leaves_the_field_clear():
    import house_scene
    pa, ph = _catch_params("arena"), _catch_params("house")
    ma, mh = build_model(pa), build_model(ph)
    assert (ma.nq, ma.nv, ma.nu) == (mh.nq, mh.nv, mh.nu) and mh.ngeom > ma.ngeom + 500
    dh = mujoco.MjData(mh)
    mujoco.mj_forward(mh, dh)
    fx, fy = house_scene.FIELD
    for g in range(mh.ngeom):
        body = mh.body(mh.geom_bodyid[g]).name
        if not (body.startswith("house_") or body in ("thrower", "held_ball")):
            continue
        assert mh.geom_contype[g] == 0 and mh.geom_conaffinity[g] == 0, body
        if body in ("thrower", "held_ball"):
            continue
        c, h = mh.geom_aabb[g, :3], mh.geom_aabb[g, 3:]
        R = dh.geom_xmat[g].reshape(3, 3)
        cw, hw = dh.geom_xpos[g] + R @ c, np.abs(R) @ h
        inside = (abs(cw[0]) - hw[0] < fx - 1e-6) and (abs(cw[1]) - hw[1] < fy - 1e-6) and cw[2] - hw[2] < 3.3
        assert not inside, f"{body}: decoration inside the field"
    # the same throw, the robot left alone: the physics can't tell the scenes apart
    runs = []
    for m in (ma, mh):
        d = mujoco.MjData(m)
        j = m.joint("ball")
        d.qpos[j.qposadr[0]:j.qposadr[0] + 3] = (-3.0, 0.0, 1.0)
        d.qvel[j.dofadr[0]:j.dofadr[0] + 6] = (6.2, 0.1, 4.0, 3.0, -2.0, 1.0)
        mujoco.mj_forward(m, d)
        for _ in range(int(2.5 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        runs.append(d.qpos.copy())
    assert np.array_equal(runs[0], runs[1])


@pytest.mark.skipif(sys.platform != "win32", reason="renders on Windows (design.md 28: no MuJoCo drawing in WSL)")
def test_rendered_frames_show_the_ball_where_it_projects():
    from catch_world import CatchWorld, WorldConfig
    from sil_game import robot_frame  # noqa: F401  (puts ros/src on the path)
    from omni_catch.detector import Detector, DetectorConfig
    w = CatchWorld(WorldConfig(scene="house", frames="rendered", seed=9000))
    home = np.array(w.p.home_xy)
    w.place_ball((home[0] - 3.0, home[1] + 0.2, 1.7), (5.0, 0.0, -1.0))           # 3 m ahead, coming in
    mujoco.mj_forward(w.m, w.d)
    pair = w.cams.frame_pair(w.d.time)
    det = Detector(DetectorConfig(acquire_candidates=40))
    for f in pair:
        assert f.img.shape == (800, 1280, 3) and f.img.dtype == np.uint8
        u, v, r, vis = f.truth
        assert vis and 4 < r < 12
        fr = robot_frame(f)
        bu, bv, bb, _ = det.detect(fr, pred=(u + 3.0, v - 2.0, r))                 # tracking window
        assert math.hypot(bu - u, bv - v) < 1.0 and bb == pytest.approx(r, rel=0.2)
        cands = det.candidates(fr, 40)                                             # acquisition: among the candidates
        assert any(math.hypot(c[0] - u, c[1] - v) < 1.0 for c in cands)
    assert 1 <= np.mean(w.cams.n_sub) <= 6                                         # motion blur: a few sub-renders
    w.close()


def test_acquisition_starts_on_the_moving_ball_not_the_still_clutter():
    _sil()
    from omni_catch.detector import DetectorConfig
    from omni_catch.frames import SparseFrame
    from omni_catch.perception import Perception
    from omni_catch.tracker import sphere_image
    flight_g = np.array([0.0, 0.0, -9.81])
    still = np.array([4.0, -0.8, 0.1])                     # a bigger yellow thing lying in the play area
    p0, v0 = np.array([6.3, 0.0, 1.0]), np.array([-6.5, 0.1, 4.0])

    def frames(pc, t):
        ball = p0 + v0 * t + 0.5 * flight_g * t * t
        out = []
        for c, Rc, f, cx, cy, _ in pc.cams_at(t):
            idx = []
            for pos, R in ((ball, 0.033), (still, 0.09)):
                im = sphere_image(Rc.T @ (pos - c), R, f)
                idx.append(_disk(cx + im[0], cy + im[1], im[2]))
            out.append(SparseFrame(1280, 800, idx[0], clutter=idx[1], seed=int(1000 * t)))
        return out, ball

    for n, starts in ((1, False), (40, True)):
        pc = Perception(dcfg=DetectorConfig(acquire_candidates=n))
        pc.add_pose(0.0, 0.0, 0.0, 0.0)
        pc.set_play_area((3.5, 0.0), (6.0, 3.0))
        start = None
        for k in range(8):
            t = k / 120
            frs, ball = frames(pc, t)
            pc.process(t, frs)
            if start is None and pc.tracker.x is not None:
                start = (k, np.linalg.norm(pc.tracker.x[:3] - ball))
        assert (start is not None) == starts
        if starts:                       # on the ball (its depth velocity takes longer: ~0.2 m of stereo noise at 6 m)
            assert start[0] <= 2 and start[1] < 0.1
            assert pc.tracker.x is not None and pc.tracker.n == 8 - start[0]


def test_mode_mux_thrower_mode_and_one_fire_counter():
    """THROWER (the operator console's bench test) parks the base and passes tilt / rpm / fire; whoever drives the
    module, the MCU sees one fire counter that moves once per new request, and a source taking over never fires."""
    _sil()                                                                # ros/src on the path
    from omni_catch.modes import ModeMux
    from omni_mcu.protocol import DRIVE_IDLE, REQ_AUTO, Request
    m = ModeMux("THROWER")
    t = Request(tilt=0.2, fly_rpm=3000.0, fire_seq=7)
    r = m.request(thrower=t)
    assert (r.req_mode, r.drive_mode, r.fly_rpm, r.tilt) == (REQ_AUTO, DRIVE_IDLE, 3000.0, 0.2)
    assert r.fire_seq == 0                                                # the console's counter adopted: no shot
    t.fire_seq = 8
    assert m.request(thrower=t).fire_seq == 1 and m.request(thrower=t).fire_seq == 1     # one request, one shot
    assert m.request().fly_rpm == 0.0 and m.request().fire_seq == 1       # the console went quiet: flywheels down
    m.set_mode("CATCH")
    assert m.fire(55) == 1 and m.fire(56) == 2                            # the behavior's stale counter adopted
    m.set_mode("TELEOP")
    assert m.request(twist=(0.1, 0.0, 0.0)).fire_seq == 2
    m.set_mode("THROWER")
    assert m.request(thrower=t).fire_seq == 2                             # back again: adopted, not fired
    assert m.set_mode("THROWER")[0] and not m.set_mode("FLY")[0]


# -- the operator console's round 3 (design.md 2, 7): faults, the native detector, logs, the pass game, a person ------

def _mcu_dict(s, info=None):
    """The McuState fields the console's status uses, from the link's State (as mission_node.on_mcu does)."""
    return dict(mode=s.mode_name, profile="kid" if s.profile else "adult", faults=s.fault_names(), v_bus=s.v_bus,
                dm_err=list(s.dm_err), fly_rpm=list(s.fly_rpm), tilt=s.tilt, ball_held=bool(s.ball_held),
                drive_mode=s.drive_mode, vx=s.vx, vy=s.vy, wz=s.wz, roll=s.roll, pitch=s.pitch)


def test_each_injected_fault_stops_the_robot_safely_and_the_console_says_so():
    """faults.py on the SIL robot, the link's bytes, the console's status (status.mcu_summary) and its core's reaction
    (missions.Console.on_mcu): the same path as the ROS run, in one process."""
    FW, PR = _sil()
    from faults import EXPECT, FaultInjector
    from omni_console import status as st
    from omni_console.missions import Console
    rig = sil_rig()
    p, m, d, r, link = rig
    inj = FaultInjector(r)
    link.faults = inj
    con = Console()
    con.on_robot_mode("TELEOP adult")
    con.command("mission", {"name": "MANUAL"})
    cmd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.5, c=0.6)          # round and round (0.8 m): no walls
    t_state = [d.time]

    def run(seconds, c=cmd):
        out, t_next, t0 = [], d.time, d.time
        while d.time - t0 < seconds:
            if d.time >= t_next - 1e-9:
                t_next = d.time + 0.01
                link.send(c, d.time)
                inj.step()
            for msg in link.poll(d.time):
                if isinstance(msg, PR.State):
                    t_state[0] = d.time
            r.step()
            out.append(float(np.linalg.norm(d.sensor("base_linvel").data[:2])))
            if int(d.time * 1000) % 50 == 0 and link.host.state is not None:     # the node's 20 Hz summary
                con.on_mcu(st.mcu_summary(_mcu_dict(link.host.state), d.time - t_state[0]))
        if link.host.state is not None:
            con.on_mcu(st.mcu_summary(_mcu_dict(link.host.state), d.time - t_state[0]))
        return np.array(out)

    def said(name):
        return any(name in text for _, text in con.pop_events())

    def names():
        return link.host.state.fault_names()

    disarm = PR.Command(req_mode=PR.REQ_DISARM, drive_mode=PR.DRIVE_IDLE)     # the console's IDLE
    run(1.5)
    con.pop_events()
    # nothing latched: the IMU (the base carries on), the bumper (a ramped stop, then on again), a short sag
    for kind, secs in (("imu", 0.5), ("bumper", None), ("battery", None)):
        inj.inject(kind, True, secs, motor=1)
        run(0.3)                                                          # during the fault
        name = EXPECT[kind]
        assert name in names() and "FAULT_STOP" not in names(), kind
        hint = st.mcu_summary(_mcu_dict(link.host.state), 0.0)["faults"]
        assert said(name) and all(f["hint"] for f in hint), kind
        if secs is None:
            inj.inject(kind, False)
        run(1.5)
        assert name not in names() and said(f"{name} cleared"), kind
    v = run(1.0)
    assert v[-1] == pytest.approx(0.5, abs=0.05)                        # driving again after them all
    # the fault stop (firmware.h): a pack low for 1 s, a silent motor -> a ramped stop and DISARMED, the mode ended;
    # still stopped once the fault is gone; re-armed only after a DISARM request (with the pack recovered)
    dt = m.opt.timestep
    for kind, secs in (("battery", None), ("motor", 0.5)):
        inj.inject(kind, True, secs, motor=1)
        v = run(2.5)
        assert "FAULT_STOP" in names() and link.host.state.mode_name == "DISARMED", kind
        assert said("FAULT_STOP") and con.mission == "IDLE" and ("set_mode", "IDLE", False) in con.pop_fx(), kind
        fast, slow = np.argmax(v < 0.4), np.argmax(v < 0.05)
        assert v[-1] < 0.05 and (slow - fast) * dt > 0.08, kind              # ramped, not a step to zero
        if kind == "battery":
            inj.inject("battery", True, volts=21.5)                       # a resting low pack: above the line, not
            run(0.3, disarm)                                              # enough to re-arm
            assert "FAULT_STOP" in names()
            run(0.1)                                                      # AUTO asked again before the pack is back
            inj.inject("battery", False)
        v = run(1.5)                                                      # fault gone, AUTO still asked: stays put
        assert v[-1] < 0.05 and "FAULT_STOP" in names() and link.host.state.mode_name == "DISARMED", kind
        run(0.2, disarm)
        assert "FAULT_STOP" not in names(), kind
        con.command("mission", {"name": "MANUAL"})
        v = run(1.5)
        assert v[-1] == pytest.approx(0.5, abs=0.05) and link.host.state.mode_name == "AUTO", kind
    con.pop_events()
    # the flywheels' telemetry: only a fault while they run
    spin = PR.Command(drive_mode=PR.DRIVE_IDLE, fly_rpm=3000.0)
    run(0.8, spin)
    inj.inject("flywheel", True, 0.3)
    run(0.1, spin)
    assert "FLY_TELEM" in link.host.state.fault_names() and said("FLY_TELEM")
    run(1.0, spin)
    # the link drops while driving: the firmware stops the robot by itself, the console says the MCU went quiet
    run(1.5)
    inj.inject("link", True, 2.0)
    v = run(2.0)
    assert v[-1] < 0.05 and v[int(0.3 / m.opt.timestep)] > 0.4 and said("went quiet")
    run(0.5)
    assert said("MCU is back")
    # the e-stop: motors off, and the console ends the mission
    run(1.0)
    inj.inject("estop", True)
    run(0.3)
    assert link.host.state.mode_name == "ESTOP" and con.mission == "IDLE" and all(not mt.enabled for mt in r.dm)
    assert ("set_mode", "IDLE", False) in con.pop_fx()


def test_native_detector_finds_the_same_blobs():
    _sil()
    from omni_catch.detector import Detector, DetectorConfig
    from omni_catch.fast_detector import FastDetector, available
    from omni_catch.frames import ArrayFrame, BALL_RGB, SparseFrame
    if not available():
        pytest.skip("no C++ compiler here")
    rng = np.random.default_rng(5)
    img = rng.integers(60, 140, (800, 1280, 3), dtype=np.uint8)
    yy, xx = np.mgrid[0:800, 0:1280]
    for _ in range(30):
        u, v, rad = rng.uniform(-10, 1290), rng.uniform(-10, 810), rng.uniform(2, 28)
        mk = (xx + 0.5 - u) ** 2 + ((yy + 0.5 - v) * rng.uniform(0.6, 1.4)) ** 2 < rad * rad
        img[mk] = (np.array(BALL_RGB) * rng.uniform(0.5, 1.0, (int(mk.sum()), 1))).clip(0, 255).astype(np.uint8)
    cfg = DetectorConfig(size_gain=1.08)
    py, cc = Detector(cfg), FastDetector(cfg)
    for fr in (ArrayFrame(img), SparseFrame(1280, 800, _disk(400.3, 300.7, 6.0), clutter=_disk(900.0, 600.0, 9.0))):
        for call in (lambda d: d.detect(fr), lambda d: d.detect(fr, pred=(402.0, 299.0, 6.0)),
                     lambda d: d.candidates(fr, 40)):
            a, b = call(py), call(cc)
            assert (a is None) == (b is None)
            if a is not None:
                assert np.allclose(np.array(a, dtype=float), np.array(b, dtype=float), atol=1e-9)


def test_a_session_log_is_written_and_its_throws_replay_in_the_sim(tmp_path):
    from catch_world import WorldConfig
    from sil_game import SilCatchGame
    from omni_console import session_log
    sys.path.insert(0, str(Path(__file__).parent / "studies"))
    import replay_log
    g = SilCatchGame(WorldConfig(dist=6.5, apex_range=(1.2, 1.5), seed=9101))
    g.open_log(tmp_path)
    g.run(3, t_max=45.0)
    s = g.close_log()
    g.world.close()
    head, recs = session_log.read(tmp_path / s["name"])
    throws = [x for x in recs if x["type"] == "throw"]
    assert head["source"] == "sim" and len(throws) == 3 and s["throws"] == 3
    assert all(t["statef"] and t["truth"] is not None and t["path"] for t in throws)
    assert all(t["caught"] == t["truth"]["caught"] for t in throws)      # the robot's beam agrees with the world
    r = replay_log.replay(tmp_path / s["name"], seed=9101)
    assert r["throws"] == 3 and r["agree"] >= 66.0


def test_pass_game_throws_each_ball_to_the_other_player():
    from catch_world import WorldConfig
    from sil_game import SilCatchGame
    g = SilCatchGame(WorldConfig(dist=6.5, apex_range=(1.2, 1.5), seed=9102, return_throws=True, throwers=2),
                     game_kw=dict(return_to="other"))
    res = g.run(4, t_max=80.0)
    g.world.close()
    assert [r["thrower"] for r in res] == [0, 1, 0, 1]
    # the first return goes back to the thrower (the robot hasn't seen the other player throw yet); from then on each
    # goes to the one who didn't throw, 2.5 m away
    launched = [r for r in res[1:] if r["launched"] and r["return_error"] == r["return_error"]]
    assert len(launched) >= 2 and all(r["return_error"] < 1.2 for r in launched)
    assert len(g.game.hands) == 2


def test_a_walking_person_is_seen_by_the_lidar_as_legs():
    from bridge_core import CoreConfig, SimCore
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros" / "src" / "omni_console"))
    from omni_console.people import detect
    core = SimCore(CoreConfig(base="v7", seed=0, shadow=(), person=True))
    scan = None
    while core.d.time < 1.0 or scan is None:
        ev = core.step()
        scan = ev.scan if ev.scan is not None else scan
    x, y = core.frame("lidar")[0][:2]
    r = np.where(np.isfinite(scan.rng), scan.rng, np.inf)
    people = detect(r, 0.0, 2 * math.pi / len(r), (x, y, 0.0))          # as the bridge publishes it (/scan)
    assert people and abs(math.hypot(*people[0]) - 1.6) < 0.25 and abs(people[0][1]) < 0.3


def _d435_scene():
    W, H = 480, 270
    fy = (H / 2) / math.tan(math.radians(58.0) / 2)
    return W, H, (fy, fy, W / 2, H / 2), fy, 0.04985


def test_d435_depth_noise_grows_with_range_squared_and_drops_grazing_floor():
    """depth_noise.d435 (design.md 4.11): the axial noise of a wall follows sigma_d z^2 / (f b); a floor seen from the
    nose (0.11 m, 7.5 deg down) loses more pixels the more edge-on it is; the ideal model (Nov 4) doesn't."""
    import depth_noise as dn
    W, H, K, fy, b = _d435_scene()
    rng = np.random.default_rng(0)
    for z0 in (1.0, 2.0):
        o = dn.d435(np.full((H, W), z0), K, fy, b, rng, scale=0.0)
        assert np.nanstd(o) == pytest.approx(z0 ** 2 * 0.1 / (fy * b), rel=0.1)
    t, h = math.radians(7.5), 0.11
    v = np.mgrid[0:H, 0:W][0]
    ry = (v - K[3]) / fy
    down = ry * math.cos(t) + math.sin(t)
    z = np.where(down > 1e-6, h / np.maximum(down, 1e-9), np.inf)
    ahead = z * (math.cos(t) - ry * math.sin(t))
    real = dn.d435(z, K, fy, b, rng, scale=0.0)
    ideal = dn.d435(z, K, fy, b, rng, dn.IDEAL, scale=0.0)
    near, far = (ahead > 0.3) & (ahead < 0.6), (ahead > 1.5) & (ahead < 2.0)
    assert np.isfinite(real[near]).mean() > 0.85 > 0.5 > np.isfinite(real[far]).mean()
    assert np.isfinite(ideal[far]).mean() > 0.95


def test_d435_depth_noise_flying_pixels_occlusion_and_dark_surfaces():
    import depth_noise as dn
    W, H, K, fy, b = _d435_scene()
    rng = np.random.default_rng(1)
    z = np.full((H, W), 2.0)
    z[100:170, 200:280] = 1.0                                      # a box 1 m in front of a wall 2 m away
    o = dn.d435(z, K, fy, b, rng, scale=0.0)
    assert np.sum((o > 1.1) & (o < 1.9)) > 20                      # flying pixels between the two
    band = int(round(fy * b * (1 / 1.0 - 1 / 2.0)))                # the right imager can't see this much of the wall
    assert np.isnan(o[110:160, 200 - band:198]).mean() > 0.9       # left of the box
    assert np.isnan(o[110:160, 290:300]).mean() < 0.1              # right of it: seen
    albedo = np.full((H, W), 0.5)
    albedo[:, :W // 2] = 0.02                                      # black on the left half
    o = dn.d435(np.full((H, W), 2.0), K, fy, b, rng, albedo=albedo, scale=0.0)
    assert np.isnan(o[:, :W // 2]).mean() > 0.4 > 0.1 > np.isnan(o[:, W // 2:]).mean()


def test_the_launchers_live_world_person_clutter_and_thrower():
    """design.md 2, the Launcher as the sim's controller (sim_world.World): the model holds the person and every piece
    of clutter, parked outside the house while off; switched on they come in where they touch nothing (obstacles in
    the rooms, clear of the robot, the hallway and the doorways), the lidar sees the tall ones; off, they leave. The
    viewer builds the same model from the bridge's hello."""
    import json
    import arm_objects
    from bridge_core import CoreConfig, SimCore, model_hello, model_params
    from robot_model import build_model
    from sim_world import DOORS, HALL_Y, ControlPort, World
    core = SimCore(CoreConfig(base="drawn", seed=0, shadow=(), live_world=True, clutter="off"))
    w = core.world
    m, d = core.m, core.d
    inside = lambda: [k for k, a in enumerate(w.adr) if abs(d.qpos[a]) < 10]           # noqa: E731
    assert w.state() == dict(person=False, clutter="off", thrower=True, obstacles=0, toys=6) and inside() == []
    assert d.mocap_pos[w.mocap][0] > 40
    assert w.set(clutter="toys") == "toys on the floor" and inside() == list(range(6))
    assert "8 larger things" in w.set(person=True, clutter="all") and len(inside()) == 14
    for _ in range(int(1.0 / m.opt.timestep)):
        core.step()
    rx, ry, _ = core.r.pose()
    for k in w.placed:
        x, y, z = d.qpos[w.adr[k]:w.adr[k] + 3]
        assert not HALL_Y[0] < y < HALL_Y[1] and min(math.hypot(x - a, y - b) for a, b in DOORS) >= 1.0
        assert math.hypot(x - rx, y - ry) > 1.1 and abs(x) < 6 and abs(y) < 4.5
        assert not w._touching(k), k                                                   # resting, not inside a wall
        h = arm_objects.height(w.objs[k])
        assert abs(z - h / 2) < 0.03, (k, z)                                           # on the floor (or a rug)
    # the tall ones reach the lidar's 15 cm plane in a group its rays see (sensors: groups 0, 1, 3)
    tall = [k for k in w.placed if arm_objects.height(w.objs[k]) > 0.2]
    assert len(tall) >= 5 and all(m.geom_group[w.geom[k]] in (0, 1, 3) for k in tall)
    assert abs(math.hypot(*(d.mocap_pos[w.mocap][:2] - (rx, ry))) - 1.6) < 0.5         # the person ahead of the robot
    assert w.set(person=False, clutter="off") == "the person left; the floor is clear" and inside() == []
    assert w.set(thrower=False) == "no thrower" and w.state()["thrower"] is False
    with pytest.raises(ValueError):
        w.set(clutter="lots")
    # the viewer's model, from the bridge's hello (JSON)
    vm = build_model(model_params(CoreConfig(**json.loads(json.dumps(model_hello(core.cfg))))))
    assert (vm.nq, vm.nbody, vm.nmocap) == (m.nq, m.nbody, m.nmocap)
    # the arm robot: its toys are on at the start, it has no thrower
    arm = SimCore(CoreConfig(base="drawn", payload="arm", seed=0, shadow=(), live_world=True, clutter="toys")).world
    assert arm.state()["thrower"] is None and arm.state()["toys"] == 6
    # a study's scene (ros/tools/toy_route.py): toys at chosen spots, one in a wall refused (parked), the rest parked
    placed = arm.place_toys([(-1.0, 0.8, 30.0), (0.0, -0.6, 0.0), (-5.0, 2.0, 0.0)])      # the last in a partition
    assert placed == [True, True, False]
    q = arm.d.qpos
    assert abs(q[arm.adr[0]] + 1.0) < 1e-9 and abs(q[arm.adr[0] + 1] - 0.8) < 1e-9
    assert all(abs(q[arm.adr[k]]) > 30 for k in range(2, 6))
    assert arm.place_toys([]) == [] and all(abs(q[arm.adr[k]]) > 30 for k in range(6))
    # the Launcher's port: one JSON line each way
    import socket
    import threading
    import time
    cp = ControlPort(0, host="127.0.0.1")
    got = []

    def ask():
        s = socket.create_connection(("127.0.0.1", cp.port), timeout=2)
        s.sendall(b'{"person": true}\n')
        got.append(json.loads(s.makefile().readline()))
        s.close()
    th = threading.Thread(target=ask)
    th.start()
    t0 = time.monotonic()
    while not got and time.monotonic() - t0 < 3:
        cp.poll(lambda r: dict(ok=True, message=w.set(**r), world=w.state()))
        time.sleep(0.01)
    th.join()
    cp.close()
    assert got and got[0]["ok"] and got[0]["world"]["person"] is True


# -- design.md 10: time synchronization and sensor timing --------------------------------------------------------------

def test_bno085_reports_keep_each_samples_own_time():
    import struct
    FW, PR = _sil()

    def rep(rid, seq, delay, vals):
        return struct.pack("<BBBB", rid, seq, (delay >> 8) << 2 | 3, delay & 0xFF) + struct.pack(f"<{len(vals)}h", *vals)
    t_int = 5_000_000
    batch = (struct.pack("<Bi", 0xFB, 25) +                                  # the batch's base: 2.5 ms before INT
             rep(0x02, 1, 5, (256, 0, -512)) + rep(0x01, 1, 5, (0, 0, 2511)) + rep(0x08, 1, 6, (0, 0, 0, 16384)) +
             struct.pack("<Bi", 0xFA, 25) + rep(0x02, 2, 0, (0, 512, 0)))   # rebased back to INT
    s = FW.sh2_samples(batch, t_int)
    assert [x[0] for x in s] == [t_int - 2000, t_int]                        # -base + rebase + delay, in 100 us
    assert s[0][2] == pytest.approx((0.5, 0.0, -1.0)) and s[1][2] == pytest.approx((0.0, 1.0, 0.0))
    assert s[0][3][2] == pytest.approx(9.81, abs=0.01) and s[0][1] == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_firmware_schedules_exact_sync_pulses_and_camera_triggers():
    FW, PR = _sil()
    fw, host = FW.Firmware(), PR.HostLink()
    out, events = None, []
    for k in range(2200):
        t_us = 3_250_000 + 1000 * k
        inp = fw.inputs
        inp.sync_fired_us = inp.cam_fired_us = 0
        if out is not None:                                   # the board's timers fire when scheduled
            if out.sync_next_us and t_us >= out.sync_next_us:
                inp.sync_fired_us = out.sync_next_us
            if out.cam_next_us and t_us >= out.cam_next_us:
                inp.cam_fired_us = out.cam_next_us
        inp.t_us, inp.v_bus = t_us, 26.0
        out = fw.tick()
        events += [m for m in host.feed(fw.tx(), t_us * 1000) if isinstance(m, PR.TimeEvent)]
    sync = [e.t_mcu_us for e in events if e.kind == PR.TE_SYNC]
    cam = np.array([e.t_mcu_us for e in events if e.kind == PR.TE_CAM_TRIGGER])
    assert sync == [4_000_000, 5_000_000]                     # on whole seconds of MCU time
    assert cam[0] == 4_000_000 and set(np.diff(cam)) <= {8333, 8334}
    assert cam[120] == 5_000_000                              # 120 Hz with no drift from rounding the period


def test_can_bus_replies_land_up_to_08_ms_after_the_tick():
    from sil_robot import DmBus, can_frame_time
    s = DmBus().schedule(0.0, [4, 4, 4, 4])
    arrivals = [x[2] for x in s]
    assert arrivals == sorted(arrivals) and 0.70e-3 < arrivals[-1] < 0.95e-3      # the bus is ~85% busy at 1 kHz
    for cmd_end, sample, rx in s:                              # each sample precedes its reply's end by >= the frame
        assert rx - sample >= can_frame_time(8) and sample > cmd_end


def _timing_rig(gpio, seconds=6.0, drive=False, **link_kw):
    from sil_link import SimLink
    p, m, d, r, _ = sil_rig()
    r2 = type(r)(m, d, p, seed=1, timing="real", clock_ppm=20.0)
    r.close()
    link = SimLink(r2, latency_up=0.00025, latency_down=0.00075, jitter=0.0001, gpio=gpio, **link_kw)
    _, PR = _sil()
    errs, vx = [], []
    t_next = d.time
    fwd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.8, flags=PR.CMD_HOLD_YAW)
    while d.time < seconds:
        t = d.time
        if t >= t_next - 1e-9:
            t_next = t + 0.01
            link.send(fwd if drive and t > 1.0 else PR.Command(drive_mode=PR.DRIVE_IDLE), t)
        for msg in link.poll(t):
            if t > seconds - 1.0 and isinstance(msg, (PR.Imu, PR.State, PR.TimeEvent)) and msg.t_host == msg.t_host:
                errs.append(msg.t_host - r2.sim_time_of(msg.t_mcu_us))     # the Jetson's stamp vs the truth
                if isinstance(msg, PR.State):
                    vx.append(msg.vx)
        r2.step()
    r2.close()
    return link.host.sync, np.array(errs), np.array(vx)


def test_sync_pulses_map_the_mcu_clock_to_microseconds_the_usb_echo_does_not():
    sync, e, _ = _timing_rig("hte")
    st = sync.status()
    assert st["mode"] == "pulse" and st["pulses"] >= 4 and len(e) > 700      # IMU, STATE and TIME of the last second
    assert np.max(np.abs(e)) < 5e-6                            # HTE-stamped pulses: the Jetson's stamps to ~1 us
    assert abs(st["drift_ppm"] - 20.0) < 1.5                   # the crystal's 20 ppm (+/- 1 ppm of wander)
    # the echo alone is biased by the USB path's asymmetry (0.25 vs 0.75 ms here, + the SIL's half-step to fetch
    # the reply), which it cannot see; the pulses can
    assert st["echo_minus_pulse_us"] > 200
    sync, e, _ = _timing_rig(None)
    assert sync.mode == "echo" and np.mean(e) > 200e-6


def test_a_plain_gpio_interrupt_still_syncs_within_15_us_and_wheel_speeds_stay_right():
    sync, e, vx = _timing_rig("irq", drive=True)
    assert sync.mode == "pulse" and np.max(np.abs(e)) < 15e-6     # 8 us of fixed IRQ latency: calibrate it out
    assert np.mean(vx) == pytest.approx(0.8, abs=0.01) and np.std(vx) < 0.04


def _ros_pkg(name):
    import os                                   # ros/omni.sh mirrors the sources into WSL and says where
    p = str(Path(os.environ.get("OMNI_ROS_SRC", Path(__file__).resolve().parents[2] / "ros" / "src")) / name)
    if p not in sys.path:
        sys.path.insert(0, p)


def test_exported_frames_follow_rep105_and_calibration_stays_in_bounds():
    _ros_pkg("omni_bringup")
    from omni_bringup.frames import angle_between, apply_calibration, check_tree, load_frames, quat_to_R, R_to_quat
    import robot_export
    fr = load_frames(robot_export.ROBOT_JSON)
    edges = [(f.parent, f.child, f.R) for f in fr] + [("odom", "base_link", None), ("map", "odom", None)]
    assert check_tree(edges) == []
    names = {f.child for f in fr}
    assert {"laser", "imu_link", "cam_left_optical", "cam_right_optical", "module_pivot", "arm_mount"} <= names
    # the cameras look 30 deg up along x; their optical z is that view
    cam = next(f for f in fr if f.child == "cam_left")
    opt = next(f for f in fr if f.child == "cam_left_optical")
    z_view = cam.R @ opt.R @ np.array([0, 0, 1.0])
    assert math.degrees(math.atan2(z_view[2], z_view[0])) == pytest.approx(30.0, abs=0.5)
    # the rules catch: a second parent, base_link off odom, a non-optical optical frame
    bad = edges + [("map", "base_link", None), ("cam_left", "cam_left_optical", np.eye(3))]
    probs = " | ".join(check_tree(bad))
    assert "two parents" in probs and "REP-103" in probs
    # a calibrated laser 8 mm / 1 deg off the drawing is applied; one 5 cm off is refused
    las = next(f for f in fr if f.child == "laser")
    turn = quat_to_R((0.0, 0.0, math.sin(math.radians(0.5)), math.cos(math.radians(0.5))))
    ok = dict(laser=dict(xyz=list(las.xyz + [0.008, 0, 0]), quat=R_to_quat(las.R @ turn)))
    out, rep = apply_calibration(fr, ok)
    assert rep[0]["applied"] and angle_between(next(f for f in out if f.child == "laser").R, las.R) > 0.01
    out, rep = apply_calibration(fr, dict(laser=dict(xyz=list(las.xyz + [0.05, 0, 0]), quat=R_to_quat(las.R))))
    assert not rep[0]["applied"] and np.allclose(next(f for f in out if f.child == "laser").xyz, las.xyz)


def test_odom_jumps_are_found_and_map_corrections_measured():
    _ros_pkg("omni_bringup")
    from omni_bringup.frames import check_motion
    smooth = [(0.02 * k, 0.02 * k, 0.0, 0.01 * k) for k in range(100)]           # 1 m/s, 0.5 rad/s
    assert check_motion(smooth)["jumps"] == 0
    jumpy = smooth[:50] + [(t, x + 0.15, y, a) for t, x, y, a in smooth[50:]]
    r = check_motion(jumpy)
    assert r["jumps"] == 1 and r["largest_jump_m"] == pytest.approx(0.17, abs=0.01)


def test_camera_frames_take_their_trigger_time_at_mid_exposure():
    _ros_pkg("omni_perception")
    from omni_perception.camera_time import FrameStamper
    rng = np.random.default_rng(4)
    fs = FrameStamper(exposure_s=0.003, trig_to_exp_s=20e-6)
    T = 1 / 120
    d_true = 20e-6 + 0.003 + 0.25e-3                       # trigger -> exposure -> readout's SOF stamp
    out = []
    fseq = 1000
    for k in range(400):
        t_trig = 10.0 + k * T
        if k == 200:                                       # a trigger the camera missed: the offset moves by one
            fs.on_trigger(k, t_trig)
            continue
        fs.on_trigger(k, t_trig)
        if k == 300:                                       # a frame lost on the Jetson: no offset change
            fseq += 1
            continue
        st = fs.stamp(fseq, t_trig + d_true + rng.normal(0, 20e-6))
        fseq += 1
        out.append((k, st))
    good = [(k, s) for k, s in out if s.source == "trigger"]
    assert len(good) > 350 and all(s.trig_seq == k for k, s in good)
    err = [s.t - (10.0 + k * T + 20e-6 + 0.0015) for k, s in good]
    assert max(abs(e) for e in err) < 1e-6                 # the trigger's own time, not the SOF's jitter
    assert fs.report()["relocks"] == 1 and abs(fs.d - d_true) < 5e-6


def _box_range(px, py, ang, half=(3.0, 2.0)):
    """Ray from (px, py) at world angle ang to the walls of a box room centred on 0."""
    c, s = math.cos(ang), math.sin(ang)
    ts = []
    for lim, comp, p in ((half[0], c, px), (-half[0], c, px), (half[1], s, py), (-half[1], s, py)):
        if abs(comp) > 1e-9 and (lim - p) / comp > 0:
            ts.append((lim - p) / comp)
    return min(ts)


def test_deskew_puts_a_moving_robots_scan_back_on_the_walls_only_in_the_right_time_order():
    _ros_pkg("omni_perception")
    from omni_perception.scan_time import deskew, sample_times
    n, dt = 1600, 0.1 / 1600
    v, w, lx = 1.5, 3.0, 0.14
    pose = lambda t: (v * t - 1.0, 0.0, w * t)                    # noqa: E731
    a = 2 * math.pi * np.arange(n) / n
    for order in (1, -1):                                          # the array's index order against time
        ts = sample_times(n, 0.0, dt, order)
        r = np.empty(n)
        for i in range(n):
            x, y, yaw = pose(ts[i])
            ox, oy = x + lx * math.cos(yaw), y + lx * math.sin(yaw)
            r[i] = _box_range(ox, oy, yaw + a[i])
        def wall_err(rr):
            x, y, yaw = pose(0.0)
            ox, oy = x + lx * math.cos(yaw), y + lx * math.sin(yaw)
            ok = np.isfinite(rr)
            px, py = ox + rr[ok] * np.cos(yaw + a[ok]), oy + rr[ok] * np.sin(yaw + a[ok])
            return np.max(np.minimum(np.abs(np.abs(px) - 3.0), np.abs(np.abs(py) - 2.0)))
        raw = wall_err(r)
        good = wall_err(deskew(a, r, ts, pose, (lx, 0.0), t_ref=0.0, angle_increment=2 * math.pi / n))
        wrong = wall_err(deskew(a, r, sample_times(n, 0.0, dt, -order), pose, (lx, 0.0), t_ref=0.0,
                                angle_increment=2 * math.pi / n))
        assert raw > 0.15 and good < 0.01 and wrong > 0.15


def test_scan_clock_removes_the_drivers_delivery_jitter():
    _ros_pkg("omni_perception")
    from omni_perception.scan_time import ScanClock
    rng = np.random.default_rng(2)
    sc = ScanClock(latency_s=0.002)
    errs = []
    for k in range(200):
        t_true = 5.0 + k * 0.1004                                 # the motor's real period
        if k == 120:
            continue                                              # a lost turn
        stamp = t_true + 0.002 + rng.exponential(0.0015)          # serial + USB delivery: late, never early
        t = sc.update(stamp, 0.1)
        if k > 20:
            errs.append(t - t_true)
    errs = np.abs(np.array(errs))
    assert np.max(errs) < 0.6e-3 and np.median(errs) < 0.3e-3     # vs ~1.5 ms of raw jitter


# -- payload "arm": an SO-101 on the payload plate instead of the catch module (design.md 11) ----------------------

def _arm_robot(**kw):
    from robot_export import v11_params
    p = v11_params(payload="arm", home_xy=(0.0, 0.0), **kw)
    return make(p)


def test_arm_payload_replaces_the_catch_module():
    import arm_so101 as A
    from robot_export import v11_params
    from robot_model import mass_properties
    pc = v11_params()
    p, m, d, r = _arm_robot()
    names = {m.joint(j).name for j in range(m.njnt)}
    assert "tilt" not in names and not any(n.startswith(("flywheel", "feed_")) for n in names)
    assert all(f"arm_{j}" in names for j in A.JOINTS) and m.nu == 4 + 6
    mc, _ = mass_properties(build_model(pc), pc)
    ma, _ = mass_properties(m, p)
    arm = m.body_subtreemass[m.body("arm_mount").id]
    assert arm == pytest.approx(0.632 + A.M_BASE_EXTRA + A.M_WRIST_CAM, abs=0.002)     # the official links + est parts
    assert 0.9 < mc - (ma - arm) < 1.5                                   # the catch module (~1.2 kg) is gone
    assert r.flywheel_speeds().tolist() == [0.0, 0.0] and r.tilt() == 0.0 and not r.ball_held()


def test_arm_holds_the_official_zero_pose_and_slews_at_servo_speed():
    import arm_so101 as A
    p, m, d, r = _arm_robot()
    r.arm.set_pose(A.POSES["zero"], snap=True)
    mujoco.mj_forward(m, d)
    for _ in range(int(0.6 / m.opt.timestep)):
        r.step()
    b = m.body("arm_mount").id
    tool = d.xmat[b].reshape(3, 3).T @ (d.sensor("arm_tool_pos").data - d.xpos[b])
    assert tool == pytest.approx([0.391, 0.0, 0.006 + 0.2265], abs=0.006)  # so101_new_calib.xml's tool at zero
    r.arm.set_pose(A.POSES["stow"])
    dq = np.max(np.abs(np.array(A.POSES["stow"]) - np.array(A.POSES["zero"])))
    t0 = d.time
    while not r.arm.settled() and d.time - t0 < 3.0:
        r.step()
    assert r.arm.settled() and d.time - t0 >= dq / A.SERVO_SPEED - 0.01
    d.qvel[r.arm.vadr[1]] = 0.5 * A.SERVO_SPEED
    r.arm.step(m.opt.timestep)
    lo, hi = m.actuator_forcerange[r.arm.a[1]]
    assert hi == pytest.approx(0.5 * A.SERVO_STALL, rel=0.02) and lo == pytest.approx(-A.SERVO_STALL)


def test_firmware_drives_the_arm_robot():
    FW, PR = _sil()
    rig = sil_rig(payload="arm")
    p, m, d, r, link = rig
    fwd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.5, flags=PR.CMD_HOLD_YAW)
    s = sil_run(rig, 1.5, lambda t: fwd)
    assert r.fw.mode == 2 and s[-1, 1] == pytest.approx(0.5, abs=0.05)
    assert r.arm.settled()                                               # the arm stayed folded while it drove


# -- the picking arm robot (design.md 11, Oct 24; the arm kit, 12.3, Oct 25): servo bus, omni_arm, the tidy ---------

def _pick_robot(**kw):
    from robot_export import arm_params
    return make(arm_params(home_xy=(0.0, 0.0), **kw))


def _omni_arm():
    import sil_link  # noqa: F401  (puts ros/src/omni_arm on the path)
    from omni_arm.kinematics import ArmKinematics
    return ArmKinematics()


def test_the_pick_robots_layout():
    """The arm kit as drawn (design.md 12.3): the arm on its riser at the plate's front edge, the cameras on the head's
    bar 40 deg down, the lidar at the front, the bin with the face behind it; the catch robot's own layout unchanged."""
    import arm_so101 as A
    from robot_export import ARM_POSES, v11_params
    p, m, d, r = _pick_robot()
    b = m.body("arm_mount").id
    assert d.xpos[b] == pytest.approx([0.14, 0.0, 0.1685 + 0.05], abs=1e-6)
    for s in ("left", "right"):
        c = m.camera(f"stereo_{s}").id
        view = -d.cam_xmat[c].reshape(3, 3)[:, 2]
        assert math.degrees(math.asin(-view[2])) == pytest.approx(40.0, abs=0.5) and d.cam_xpos[c][0] > 0.15
    assert d.site_xpos[m.site("lidar").id][0] > 0.1                                   # at the deck's front edge
    names = {m.geom(g).name for g in range(m.ngeom)}
    assert {"arm_bin0", "arm_face", "arm_audio", "arm_pad_fixed", "arm_pad_moving", "arm_tip0", "arm_tip1"} <= names
    assert r.arm.q() == pytest.approx(np.array(A.POSES["kit_stow"]), abs=1e-6)                       # folded
    _omni_arm()
    from omni_arm.kinematics import ArmKinematics
    from omni_arm.pick import TidyTask
    assert np.allclose(TidyTask(ArmKinematics()).poses["stow"], ARM_POSES["stow"])                  # the robot's stow
    pc, mc, dc, rc = make(v11_params(home_xy=(0.0, 0.0)))
    assert dc.site_xpos[mc.site("lidar").id][0] > 0.1 and "arm_bin0" not in {mc.geom(g).name for g in range(mc.ngeom)}


def test_omni_arm_kinematics_are_the_sims():
    """omni_arm's chain (from robot_arm.json) against MuJoCo's: the tool point, the jaw and the wrist camera."""
    import arm_so101 as A
    p, m, d, r = _pick_robot()
    K = _omni_arm()
    qadr = [m.joint(A.name(j)).qposadr[0] for j in A.JOINTS]
    adr = m.joint("base").qposadr[0]
    d.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
    rng = np.random.default_rng(3)
    for _ in range(30):
        q = rng.uniform(K.lo, K.hi)
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_camlight(m, d)
        T = K.fk(q)
        assert T["tool"][:3, 3] == pytest.approx(d.site_xpos[m.site("arm_tool").id], abs=1e-9)
        assert T["arm_jaw"][:3, 3] == pytest.approx(d.xpos[m.body("arm_jaw").id], abs=1e-9)
        c = m.camera("arm_wrist").id
        assert T["wrist_cam"][:3, :3] == pytest.approx(d.cam_xmat[c].reshape(3, 3), abs=1e-9)


def test_the_d435_on_the_nose_clears_the_robot():
    """design.md 4.11: the D435 (robot_export.ARM_D435) on the deck's nose: >= 3 mm from every part of the robot (the
    lidar, the bumper, the wheels' rollers and guards), its top >= 3 mm under the lidar's scan plane (no shadow), inside
    the bumper's 217 mm, looking 7.5 deg down (Nov 4: the room as well as the floor); the stereo arm robot has its box
    but not its cameras, the catch robot none."""
    from robot_export import arm_d435_params
    p, m, d, r = make(arm_d435_params(home_xy=(0.0, 0.0)))
    adr = m.joint("base").qposadr[0]
    d.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    g = m.geom("d435").id
    ft = np.zeros(6)
    near = min(mujoco.mj_geomDistance(m, d, g, k, 0.05, ft) for k in range(m.ngeom)
               if k != g and m.body(m.geom_bodyid[k]).name != "world")
    assert near >= 0.003
    R, c, h = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g], m.geom_size[g]
    corners = c + (np.array([(a, b, e) for a in (-1, 1) for b in (-1, 1) for e in (-1, 1)]) * h) @ R.T
    assert corners[:, 2].max() < d.site_xpos[m.site("lidar").id][2] - 0.003
    assert np.hypot(corners[:, 0], corners[:, 1]).max() < 0.217
    view = -d.cam_xmat[m.camera("d435_depth").id].reshape(3, 3)[:, 2]
    assert math.degrees(math.asin(-view[2])) == pytest.approx(7.5, abs=0.1)
    pa, ma, da, ra = _pick_robot()
    assert "d435" in {ma.geom(k).name for k in range(ma.ngeom)} and pa.depth_camera == "none"     # the box only
    assert mujoco.mj_name2id(ma, mujoco.mjtObj.mjOBJ_CAMERA, "d435_depth") == -1
    assert not any(ma.geom(k).name == "d435" for k in range(ma.ngeom) if ma.geom_contype[k])       # no collision


def test_the_d435_on_its_side_for_studies():
    """d435_roll_deg 90 (studies/d435_mount.py, Nov 5): the body stands 90 mm tall with the imagers above one another,
    the images stay upright and turn tall (270 x 480, the 87-89 deg field tall), the view axis doesn't move; the
    robot's config (robot_export.arm_json) carries the size and field; landscape (roll 0) is the baseline's."""
    from robot_export import arm_d435_params, arm_json
    from robot_model import d435_view
    kw = dict(home_xy=(0.0, 0.0), d435_pos=(0.20, 0.0, 0.20), d435_tilt_deg=-30.0)
    views = {}
    for roll in (0.0, 90.0):
        p = arm_d435_params(**kw, d435_roll_deg=roll)
        m = build_model(p)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        c = m.camera("d435_depth").id
        R = d.cam_xmat[c].reshape(3, 3)
        g = m.geom("d435").id
        ext = np.abs(d.geom_xmat[g].reshape(3, 3)) @ m.geom_size[g]       # half extents along the world's axes
        views[roll] = (d.cam_xpos[c] - d.xpos[m.body("base").id], -R[:, 2], R[:, 0], m.cam_fovy[c], d435_view(p), ext)
    (p0, f0, r0, fov0, v0, e0), (p1, f1, r1, fov1, v1, e1) = views[0.0], views[90.0]
    assert v0[:2] == (480, 270) and fov0 == pytest.approx(58.0) and e0[2] < 0.03
    assert v1[:2] == (270, 480) and fov1 == pytest.approx(89.2, abs=0.1) and e1[2] > 0.04
    assert np.allclose(f0, f1) and np.allclose(r0, r1)                    # the same view axis, upright images
    assert abs(p1[2] - p0[2]) == pytest.approx(0.0175 * math.cos(math.radians(30)), abs=1e-3)   # left imager now on top
    cam = arm_json(arm_d435_params(**kw, d435_roll_deg=90.0))["cameras"][0]
    assert (cam["width"], cam["height"]) == (270, 480) and cam["fovy_deg"] == pytest.approx(89.2, abs=0.1)


def test_the_d435_sees_the_floor_the_arm_reaches():
    """The D435's depth model (sil_pick.PickCameras) through the robot's own code (robot_arm_d435.json, floor_vision):
    nothing nearer than its min-Z, millimetre steps; the floor seen from the tidy's close look (look_back short of the
    sweet spot, design.md 4.11) out past 1 m; floor points within 3 cm of z = 0 out to 1.2 m (the noise plus the arm
    robot's ~0.9 deg pitch on its pods)."""
    from robot_export import arm_d435_params
    from sil_pick import PickCameras
    _omni_arm()
    from omni_arm.floor_vision import DepthFrame, points
    from omni_arm.kinematics import ArmConfig, config_path
    from omni_arm.pick import TidyTask
    from omni_arm.kinematics import ArmKinematics
    p, m, d, r = make(arm_d435_params(home_xy=(0.0, 0.0), area_size=(9.0, 9.0)))
    for _ in range(400):
        mujoco.mj_step(m, d)
    cams = PickCameras(m, d, p, cam_err_deg=0.0)
    z = cams.depth()
    assert z.shape == (270, 480) and np.nanmin(z) >= cams.z_min - 0.0005                 # (to the mm)
    assert np.nanmax(np.abs(z * 1000 - np.round(z * 1000))) < 1e-3
    T = TidyTask(ArmKinematics(ArmConfig(config_path("d435")))).T_base_cam
    P = points(DepthFrame(0.0, z, cams.Kd, T))
    floor = P[(np.abs(P[:, 1]) < 0.15) & (np.hypot(P[:, 0], P[:, 1]) < 1.2)]
    assert np.abs(floor[:, 2]).max() < 0.03
    x = P[np.abs(P[:, 2]) < 0.03, 0]
    t = TidyTask(ArmKinematics(ArmConfig(config_path("d435"))))
    look = t.p.sweet[0] + t.p.look_back                     # where the close look puts a thing
    assert x.min() < look - 0.05 and np.percentile(x, 99) > 1.0          # its ring of floor in view
    assert x.min() + 0.05 <= t.p.view_near + 0.01                        # the tidy's "in view" agrees
    cams.close()


def test_the_bridge_builds_the_d435_arm_robot():
    """bridge_core: arm_camera "d435" puts the D435 on the arm robot in the house (and the viewer's hello says so)."""
    from bridge_core import CoreConfig, model_hello, model_params
    c = CoreConfig(payload="arm", arm_camera="d435")
    assert model_params(c).depth_camera == "d435" and model_hello(c)["arm_camera"] == "d435"
    assert model_params(CoreConfig(payload="arm")).depth_camera == "none"
    with pytest.raises(ValueError):
        model_params(CoreConfig(payload="arm", arm_camera="kinect"))


def test_the_driver_runs_the_arm_through_the_servo_bus():
    """omni_arm's driver over the STS3215s as their bus behaves (arm_bus): it reads the joints, its goals move them,
    torque off leaves the arm limp, a calibration error shows as an offset between what it reads and the joint."""
    import arm_so101 as A
    from arm_bus import ServoBusSim
    _omni_arm()
    from omni_arm.driver import ArmDriver
    from omni_arm.feetech import Bus
    from omni_arm.kinematics import ArmConfig
    p, m, d, r = _pick_robot()
    bus = ServoBusSim(r.arm)
    drv = ArmDriver(Bus(bus, timeout=0.0, clock=lambda: 0.0), ArmConfig())
    assert drv.ping() == []
    drv.enable(True)
    st = drv.read(0.0)
    assert st.ok and st.q == pytest.approx(r.arm.q(), abs=2 * 2 * math.pi / 4096)
    goal = np.array([0.4, -1.0, 0.6, 0.8, 0.5, 0.3])
    drv.send(goal)
    for _ in range(int(1.5 / m.opt.timestep)):
        r.step()
    assert drv.read(1.5).q == pytest.approx(goal, abs=0.03) and bus.n_errors == 0
    drv.enable(False)
    assert np.all(m.actuator_forcerange[r.arm.a] == 0.0) or np.all(r.arm.torque_scale == 0.0)
    off = ServoBusSim(r.arm, joint_err_deg=1.0, seed=5)
    drv2 = ArmDriver(Bus(off, timeout=0.0, clock=lambda: 0.0), ArmConfig())
    q_true = r.arm.q()
    assert np.abs(drv2.read(0.0).q[:5] - (q_true[:5] - off.err[:5])).max() < 2e-3 and np.abs(off.err[:5]).max() > 1e-3
    del A


def test_mode_mux_caps_the_base_while_the_arm_is_out():
    import json
    _omni_arm()
    from omni_arm.kinematics import CONFIG
    from omni_catch.modes import ModeMux
    ao = json.loads(CONFIG.read_text())["arm_out"]
    ARM_OUT_CAPS = (ao["speed_cap"], ao["a_cap"])                                    # the one source: the export
    mux = ModeMux("PICK", arm_out_caps=ARM_OUT_CAPS)
    r = mux.request(pick=(0.3, 0.0, 0.0))
    assert r.drive_mode == 1 and r.a == 0.3 and r.speed_cap == 0.0 and r.a_cap == 0.0
    mux.arm_out = True
    for mode, kw in (("PICK", dict(pick=(0.3, 0.0, 0.0))), ("TELEOP", dict(twist=(1.5, 0.0, 0.0)))):
        mux.set_mode(mode)
        r = mux.request(**kw)
        assert r.speed_cap == ARM_OUT_CAPS[0] and r.a_cap == ARM_OUT_CAPS[1]
    mux.set_mode("PICK")
    assert mux.request().drive_mode == 0                                             # no fresh twist: a ramped stop


def test_the_gripper_picks_up_a_block():
    """The physics of a grasp: the IK's hover, straight down, close, up; the block comes with it, the fingers
    stopped apart at its width."""
    import arm_objects
    import arm_so101 as A
    K = _omni_arm()
    from omni_arm.kinematics import DOWN, q_for_gap
    obj = arm_objects.objects_for(["cube30"], [(0.33, 0.0)], [0.0])
    p, m, d, r = _pick_robot(floor_objects=obj)
    for _ in range(int(0.3 / m.opt.timestep)):
        r.step()
    tool = np.array([0.33, -(0.015 + 0.008), 0.005 + 0.006])         # the fixed finger 8 mm off the cube's side
    seq = []
    for z in (0.12, tool[2]):
        q = K.ik([tool[0], tool[1], z], DOWN, yaw=math.pi / 2, seed=np.array([0, -0.5, 0.6, 1.5, 0, 0]), flip_ok=False)
        assert q is not None
        q[5] = q_for_gap(K.cfg, 0.06)
        seq.append(q)
    grip = seq[1].copy()
    grip[5] = A.GRIPPER_CLOSED
    up = seq[0].copy()
    up[5] = A.GRIPPER_CLOSED
    for target, s in ((seq[0], 2.5), (seq[1], 1.5), (grip, 1.0), (up, 2.0)):
        r.arm.set_pose(target)
        for _ in range(int(s / m.opt.timestep)):
            r.step()
    cube = d.xpos[m.body("obj0").id]
    from omni_arm.kinematics import gap_at
    assert cube[2] > 0.08 and 0.024 < gap_at(K.cfg, r.arm.q()[5]) < 0.034          # the pads' gap near the tips


def test_the_tidy_picks_up_a_toy_through_the_whole_stack():
    """sil_pick: the robot's own code (driver, vision, the tidy task, the mode mux, the C++ firmware) finds one cube
    near the robot, picks it up and drops it in the bin; then finds nothing more."""
    from sil_pick import SilPick
    g = SilPick(1, "toys", seed=2, spread=0.4)
    s = g.run(t_max=150.0)
    g.cams.close()
    assert s["in_bin"] == 1 and s["task_picked"] == 1 and s["reason"] == "nothing in sight", s["events"]
    assert s["bus_errors"] == 0 and s["max_tilt_deg"] < 5.0


def test_the_drawn_robot_has_the_cads_mass_cg_wheels_and_scan_plane():
    """design.md 7 step 1: robot_export.drawn_params is the robot as cad/v7 draws it: 8.19 kg and the BOM's CG, the
    515 g printed wheels, no front rocker, the bumper ring at r 205-217 mm, the A2M12's plane 150 mm up. The constants
    are the BOM's (cad/v7/out/BOM.md, written by build.py): this fails when a CAD change moves them."""
    import re
    from robot_export import CAD_CG, CAD_MASS_KG, drawn_params
    from robot_model import mass_properties
    bom = (Path(__file__).resolve().parents[2] / "cad" / "v7" / "out" / "BOM.md").read_text(encoding="utf-8")
    mt = re.search(r"CAD estimate: \*\*([\d.]+) kg\*\*, CG at \[([-\d.]+), ([-\d.]+), ([-\d.]+)\] mm", bom)
    assert mt, "BOM.md's mass line moved"
    assert float(mt.group(1)) == pytest.approx(CAD_MASS_KG, abs=0.005)
    assert [float(mt.group(k)) / 1000 for k in (2, 3, 4)] == pytest.approx(list(CAD_CG), abs=0.0006)
    p = drawn_params()
    m = build_model(p)
    mass, cg = mass_properties(m, p, 0.0)
    assert mass == pytest.approx(CAD_MASS_KG, abs=1e-3) and list(cg) == pytest.approx(list(CAD_CG), abs=1e-3)
    assert p.omni.mass == pytest.approx(0.515) and p.rocker == "none" and p.accel_table is not None
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert d.site_xpos[m.site("lidar").id][2] == pytest.approx(0.150, abs=1e-3)
    rb = max(float(np.hypot(*m.geom_pos[g][:2])) for g in range(m.ngeom) if m.geom_matid[g] == m.material("tpu_orange").id
             and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE and m.geom_size[g][0] > 0.01)
    assert 0.20 < rb + 0.014 <= 0.2175                                   # the ring's outer edge at the CAD's 217 mm


def test_the_lidar_mask_hides_the_drawn_robots_standoff_sectors():
    """The A2M12 gets no return where the drawn robot's own parts stand (sensors.BLOCKED_CAD, 22 deg in all)."""
    from bridge_core import CoreConfig, SimCore
    from sensors import BLOCKED_CAD
    c = SimCore(CoreConfig(base="drawn", shadow=()))
    scan = None
    while scan is None and c.d.time < 0.5:
        scan = c.step().scan
    assert scan is not None
    deg = np.degrees(scan.ang) % 360
    blocked = np.zeros(len(deg), bool)
    for a0, w in BLOCKED_CAD:
        blocked |= (deg - a0) % 360 < w
    assert np.isnan(scan.rng[blocked]).all()
    assert np.isfinite(scan.rng[~blocked]).mean() > 0.9                  # the house's walls everywhere else
    assert 0.055 < blocked.mean() < 0.07                                 # 22 of 360 deg


def test_the_house_catch_places_a_thrower_where_there_is_room():
    """house_catch: in the house (the robot step's one world) a thrower stands dist m in front of the robot when
    there is room, else where there is (the robot turns to them); the throws fly and are scored. Where no direction
    has room it refuses and says why."""
    from bridge_core import CoreConfig, SimCore
    from house_catch import HouseCatch
    c = SimCore(CoreConfig(base="drawn", firmware=True, shadow=()))
    hc = HouseCatch(c.m, c.d, c.p, c.r, seed=1, returns=False)
    x, y, yaw = c.r.pose()                               # the hallway start, facing down the hallway
    s = hc.start((x, y, yaw + math.pi), c.d.time)        # turned round, facing the wall: the thrower goes behind it
    assert s["ok"] and abs(abs(s["bearing"]) - math.pi) < 0.6 and "deg to its" in s["message"]
    hc.MIN_DIST = 20.0                                   # nowhere in the house has 20 m
    s = hc.start((x, y, yaw), c.d.time)
    assert not s["ok"] and "No room" in s["message"]
    hc.MIN_DIST = HouseCatch.MIN_DIST
    s = hc.start((x, y, yaw), c.d.time)
    assert s["ok"] and s["dist"] == pytest.approx(6.5) and s["bearing"] == 0.0 and 0.5 < s["half"] <= 2.0
    seen = 0
    while c.d.time < 9.0:
        c.step()
        hc.step(c.d.time)
        seen += sum(len(pair[0].idx) > 0 for pair in hc.pop_frames(c.d.time))
    assert len(hc.results) >= 2 and seen > 100           # (nobody runs the catch code here: the throws miss)
    assert all(6.0 < np.hypot(*(np.array(r["release"][:2]) - [x, y])) < 7.0 for r in hc.results)
    hc.stop()
    n = len(hc.results)
    while c.d.time < 12.0:
        c.step()
        hc.step(c.d.time)
    assert len(hc.results) == n and not hc.pop_frames(c.d.time)


# -- design.md 16: the base trade, the terrain course ------------------------------------------------

def test_terrain_course_is_optional_and_drivable():
    """The kitchen's terrain course (house.TERRAIN_COURSE) is in the house only when asked for (house_params course,
    CoreConfig.terrain: the bridge and its 3D view build the same model), and the drawn robot climbs its 20 mm
    platform with the ball aboard."""
    import house
    from bridge_core import CoreConfig, model_hello, model_params
    n = len(house.HOUSE_FEATURES)
    assert len(house.house_params("v7").floor_features) == n
    assert len(model_params(CoreConfig(base="drawn", terrain=True)).floor_features) == n + len(house.TERRAIN_COURSE)
    assert model_hello(CoreConfig(terrain=True))["terrain"] is True
    drv = house.Driver(house.house_params("drawn", assist=False, course=True))
    drv.teleport((4.1, -2.7, -90.0))                         # north of the 20 mm platform, facing it
    for _ in range(int(2.5 / drv.m.opt.timestep)):
        drv.step(np.array([0.5, 0.0]), 0.0, False)
    assert drv.r.pose()[1] < -3.5 and drv.r.ball_held()      # up on the platform (it starts at y -3.3)
    assert drv.d.sensor("base_pos").data[2] > 0.02


def test_base_trade_bases():
    """studies/base_trade.py: the four bases keep the drawn robot's extra mass; the kiwi is a wheel, motor and pod
    lighter, has three wheels and its own acceleration table; the rigid X4 has no pods and no rocker, the X4 rocker its
    front beam (mass_model "cad" drops the rocker only on pods)."""
    sys.path.insert(0, "studies")
    import base_trade
    from robot_model import mass_properties
    masses = {}
    for b in base_trade.BASES:
        p = base_trade.params(b)
        masses[b] = mass_properties(build_model(p), p)[0]
        assert p.mass_model == "cad" and p.extra_masses == base_trade._drawn_extra()
    assert masses["X4 + pods"] == pytest.approx(8.19, abs=0.01)
    assert 0.9 < masses["X4 + pods"] - masses["kiwi rigid"] < 1.5
    assert base_trade.params("kiwi rigid").n_wheels == 3 and base_trade.params("X4 rigid").suspension == "roller"
    assert base_trade.params("kiwi rigid").accel_table == base_trade.KIWI_TABLE
    assert base_trade.params("X4 rigid").rocker == "none" and base_trade.params("X4 + pods").rocker == "none"
    m = build_model(base_trade.params("X4 rocker"))
    assert m.joint("rocker").id >= 0


def test_measured_cameras_overrides_only_what_the_throws_measured(tmp_path):
    """catch_world.measured_cameras: characterization/throw_track's vision_measured.json sets the fields it names."""
    import json as _json
    from catch_world import expected_cameras, measured_cameras
    f = tmp_path / "vision_measured.json"
    f.write_text(_json.dumps({"vision": {"edge_sigma": 1.8, "latency_mean": 0.019, "drop_prob": 0.03, "not_a_field": 1}}))
    m, e = measured_cameras(f), expected_cameras()
    assert (m.edge_sigma, m.latency_mean, m.drop_prob) == (1.8, 0.019, 0.03)
    assert (m.calib_rot_deg, m.exposure, m.false_prob) == (e.calib_rot_deg, e.exposure, e.false_prob)   # not measured: kept
    from catch_world import ball_pixels
    import numpy as _np
    # the split-out blob model: a 4.3 px-radius ball (f 463, 4 m ahead) is found, with the same pixels for the same random draws
    cfg = e
    a = ball_pixels(cfg, _np.random.default_rng(1), _np.array([0.2, 0.1, -4.0]), _np.zeros(3), 0.037, 463.0, 384, 216, 768, 432)
    b = ball_pixels(cfg, _np.random.default_rng(1), _np.array([0.2, 0.1, -4.0]), _np.zeros(3), 0.037, 463.0, 384, 216, 768, 432)
    assert a is not None and (a[0] == b[0]).all() and 40 < a[0].sum() < 100


def _v10_robot(**kw):
    from robot_export import arm_params
    return make(arm_params(home_xy=(0.0, 0.0), **kw))


def test_the_v10_pod_is_the_cads_fork():
    """cad/v10 human editable: each wheel hangs on a fork that pivots on a shaft beside it, level with the axle at its
    nominal height (y 80.6, z 0 since the Nov 2 redraw; the unmirrored pods FR / RL pivot on hub +y, the mirrored FL / RR
    on -y), the axle moving -3 to +15 mm (CAD limits 87.87 to 100.73 deg = 12.86 deg) on an arc, the 140 mm wheel on r 146
    with the AK40 inside it."""
    from robot_model import fork_kinematics
    from robot_export import CAD_POD_PIVOT, POD_PIVOT
    p, m, d, r = _v10_robot(pod_pivot=CAD_POD_PIVOT)         # the fork as drawn (the baseline's: both level since Nov 2)
    assert CAD_POD_PIVOT == pytest.approx(POD_PIVOT, abs=1e-6) and POD_PIVOT[1] == 0.0
    assert p.wheel == "Printed-140-2x6-36-cad" and p.wheel_base_radius == 0.146 and p.drive_motor == "AK40-10"
    assert p.omni.width == pytest.approx(0.072) and p.omni.rollers_per_row == 6 and p.omni.roller_diameter == 0.036
    for i, mirrored in zip((1, 2, 3, 4), (True, False, True, False)):          # 45, 135, 225, 315 deg
        f = fork_kinematics(p, i)
        assert (f["sgn"] < 0) == mirrored
        assert math.degrees(f["a_max"]) == pytest.approx(100.7255175 - 87.8669100, abs=0.05)
        j = m.joint(f"susp{i}")
        assert j.type[0] == mujoco.mjtJoint.mjJNT_HINGE and tuple(m.jnt_range[j.id]) == pytest.approx((0.0, f["a_max"]), abs=1e-4)
        hub, pod = m.body(f"hub{i}").id, m.body(f"carrier{i}").id
        d.qpos[j.qposadr[0]] = 0.0
        mujoco.mj_forward(m, d)
        z0, c0 = d.xpos[hub][2], d.xpos[hub].copy()
        assert np.linalg.norm(d.xpos[pod] - c0) == pytest.approx(0.0806, abs=0.0005)                       # a rigid fork
        d.qpos[j.qposadr[0]] = f["a_max"]
        mujoco.mj_forward(m, d)
        assert d.xpos[hub][2] - z0 == pytest.approx(0.018, abs=2e-4)                          # droop stop to bump stop
        away = d.xpos[hub][:2] - c0[:2]
        assert math.hypot(*away) == pytest.approx(0.00135, abs=0.0002)                         # the arc: the axle 80.5 -> 79.2 mm from the shaft
        th = math.radians(p.wheel_angles_deg[i - 1])
        tangent = np.array([math.sin(th), -math.cos(th)]) * f["sgn"]                           # toward the pivot
        assert np.dot(away, tangent) > 0                                                       # level pivot: the axle moves toward it
    p, m = _v10_robot()[:2]                    # the baseline (pivot at the axle) keeps the CAD's mass and CG via its ballast
    mass, cg = mass_properties(m, p)
    assert mass == pytest.approx(9.632, abs=0.01) and cg[2] == pytest.approx(0.1264, abs=0.002)


def test_the_v10_robot_stands_on_its_pods_inside_its_ring():
    """The v10 arm robot settles on its four pods (all loaded, none at a stop, the deck level), the wheels and forks and the
    guards' ears inside the deck's r 217 and the bumper ring 8 mm outside it; the lidar 140 mm out, the plate at 181.5
    and the scan plane at 163; the D435 on the nose clear of the robot; the old robot's numbers unchanged."""
    from robot_model import wheel_loads
    p, m, d, r = _v10_robot()
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 2] = 0.004
    for _ in range(2500):
        mujoco.mj_step(m, d)
    q = np.array([d.qpos[m.joint(f"susp{i}").qposadr[0]] for i in range(1, 5)])
    hi = m.jnt_range[m.joint("susp1").id][1]
    assert (q > 0.01 * hi).all() and (q < 0.99 * hi).all()
    loads = wheel_loads(m, d, p)
    assert loads.sum() == pytest.approx(9.632 * 9.81, rel=0.03) and loads.min() > 0.5 * loads.mean()
    assert abs(d.xquat[m.body("base").id][1]) < 0.02 and abs(d.xquat[m.body("base").id][2]) < 0.02
    # everything on the pods (rollers, hubs, forks, motor) is inside the deck
    base = d.xpos[m.body("base").id]
    rmax = 0.0
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        n = m.body(b).name
        if n.startswith(("carrier", "hub")) or n.startswith("roller"):
            R, c = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g] - base
            lo, hi_ = m.geom_aabb[g, :3] - m.geom_aabb[g, 3:], m.geom_aabb[g, :3] + m.geom_aabb[g, 3:]
            pts = np.array([(a, b_, e) for a in (lo[0], hi_[0]) for b_ in (lo[1], hi_[1]) for e in (lo[2], hi_[2])])
            rmax = max(rmax, np.hypot(*(c + pts @ R.T)[:, :2].T).max())
    assert rmax < p.deck_radius and p.deck_radius == 0.217 and p.bumper_radius + 0.014 == pytest.approx(0.225)
    lidar = d.xmat[m.body("base").id].reshape(3, 3).T @ (d.site_xpos[m.site("lidar").id] - base)    # in the base frame
    assert lidar[0] == pytest.approx(0.14, abs=0.001) and lidar[2] == pytest.approx(0.163, abs=0.001)
    geo = geometry(p)
    assert geo.plate_top == pytest.approx(0.1815, abs=1e-4) and p.lidar_scan_z == 0.163
    # the D435's box: >= 3 mm from every part, under the scan plane, inside the bumper, looking 30 deg down
    d.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    g = m.geom("d435").id
    ft = np.zeros(6)
    near = min(mujoco.mj_geomDistance(m, d, g, k, 0.05, ft) for k in range(m.ngeom)
               if k != g and m.body(m.geom_bodyid[k]).name != "world")
    assert near >= 0.003
    assert m.geom_size[g] * 2 == pytest.approx([0.025, 0.090, 0.025])
    # the baseline robot is as it was
    p0 = SimParams()
    assert p0.pod_pivot is None and p0.wheel_base_radius == 0.15 and p0.deck_radius == 0.185 and p0.lidar_x is None


def test_the_v10_arm_robot_reaches_the_floor_and_clears_its_head():
    """The arm robot's baseline (cad/v10's coarse model) with the arm on a 37 mm riser: its base at the v7 robot's 218.5 mm
    (the CAD's 231.5 mm leaves the SO-101 no fingers-down floor reach), the fingers-down reach 0.26-0.35 m as before; the
    head 5 mm up, 4 mm between the lidar's top and the camera bar and between the bar's top and the adapter."""
    import json as _json
    from robot_export import ARM, ARM_D435, ARM_V7, arm_params, main
    assert main(check=True)
    cfg = _json.loads((Path(__file__).resolve().parents[2] / "ros/src/omni_arm/omni_arm/config/robot_arm.json").read_text())
    kin = _omni_arm()
    from omni_arm import motion as mo
    from omni_arm.pick import PickParams
    from omni_arm.kinematics import DOWN
    tp = PickParams()
    assert kin.mount[2] == pytest.approx(0.2185, abs=1e-4) and ARM["arm_riser"] == 0.037
    reach = []
    for x in np.arange(0.20, 0.46, 0.01):
        ok = True
        for top in (0.015, 0.04):
            tool = np.array([x, 0.0, (tp.tip_z_low if top < tp.low_top else tp.tip_z) + kin.cfg.pads["fingertip_below_tool"]])
            hover = tool + np.array([0, 0, top + tp.hover_dz])
            qh = kin.ik(hover, DOWN, yaw=0.0, flip_ok=False)
            ok &= qh is not None and kin.clear(qh)[0] and mo.line(kin, qh, tool, DOWN, 0.0) is not None
        if ok:
            reach.append(x)
    assert reach and min(reach) <= 0.27 and max(reach) >= 0.34
    box = {k["note"]: k for k in cfg["keep_out"] if k["kind"] == "box"}
    lidar_top = 0.163 - 0.026 + 0.045                                            # robot_model's lidar stand-in
    assert box["the camera bar"]["min"][2] - lidar_top >= 0.003
    assert box["the arm's adapter"]["min"][2] <= 0.1685 and kin.mount[2] - box["the camera bar"]["max"][2] >= 0.003
    assert cfg["cameras"][0]["name"] == "left" and ARM_D435["depth_camera"] == "d435" and ARM["d435_box"]
    assert ARM_V7["arm_riser"] == 0.05 and "pod_pivot" not in ARM_V7
    p = arm_params()
    assert p.extra_masses and p.depth_camera == "none" and p.d435_box


def test_the_arm_robot_runs_the_firmware_built_for_its_wheels():
    """The firmware's wheel radius, kinematics and wheel speed limit are compile-time constants: the arm robot (cad/v10's
    140 mm wheels on r 146, AK40-10) gets fw_params_arm.h in its own SIL library, the catch robot fw_params.h."""
    import robot_export
    FW, _ = _sil()
    assert robot_export.firmware_variant(robot_export.arm_params()) == "arm"
    assert robot_export.firmware_variant(robot_export.v11_params()) == ""
    assert robot_export.firmware_variant(robot_export.arm_d435_params()) == "arm"
    assert FW.param("MAX_WHEEL_SPEED", "arm") == pytest.approx(
        robot_export.arm_params().wheel_speed_frac * robot_export.arm_params().motor.noload_rpm * 2 * math.pi / 60, rel=1e-5)
    assert FW.param("MAX_WHEEL_SPEED") != FW.param("MAX_WHEEL_SPEED", "arm")
    p, m, d, r = make(robot_export.arm_params(home_xy=(0.0, 0.0)))
    from sil_robot import SilRobot
    assert SilRobot(m, d, p).fw.variant == "arm"
    assert SilRobot(*make(robot_export.v11_params(home_xy=(0.0, 0.0)))[1:3], robot_export.v11_params(home_xy=(0.0, 0.0))).fw.variant == ""


# -- Nov 1 motor study (studies/drive_char.py): the drive model and the arm robot's new physics -------------------------

def test_the_ak40_model_follows_the_makers_24v_curve():
    """catalog's AK40-10 (fitted by studies/motor_fit.py) on the maker's full-throttle chart: at each torque the speed
    where the drive's motoring limit, less the gearbox friction, equals it, within 3% RMS."""
    from catalog import DRIVE_MOTORS
    from models import DriveActuator
    mot = DRIVE_MOTORS["AK40-10"]
    drv = DriveActuator(mot)
    rpm = 2 * math.pi / 60
    errs = []
    for ln in (Path(__file__).parent / "studies" / "data" / "ak40_10_kv170_24v.csv").read_text().splitlines():
        if not ln.strip() or ln.startswith("#") or ln.startswith("torque"):
            continue
        T, n = (float(x) for x in ln.split(",")[:2])
        lo, hi = 0.0, 60.0
        for _ in range(50):
            w = 0.5 * (lo + hi)
            if drv.torque_limits(w, 24.0)[1] - mot.backdrive_torque - mot.viscous * w > T:
                lo = w
            else:
                hi = w
        errs.append((lo / rpm - n) / n)
    assert math.sqrt(np.mean(np.square(errs))) < 0.03
    # at the shaft (the catalog's torques are before the gearbox friction): the datasheet's 4.1 peak, 1.3 rated
    assert mot.peak_torque - mot.backdrive_torque == pytest.approx(4.1, abs=0.05)
    assert mot.rated_torque - mot.backdrive_torque == pytest.approx(1.3, abs=0.05)
    assert DRIVE_MOTORS["AK40-10-dc"].r_eq == pytest.approx(0.75 * 0.978)      # the earlier model, kept


def test_the_arm_robot_at_the_motor_studys_masses():
    """arm_params(mass_kg, cg_dz): the total and CG land where asked, and the pods' damping uses the sprung mass."""
    import robot_export as E
    for kg, dz in ((15.0, 0.0), (20.0, 0.04)):
        p = E.arm_params(kg, dz)
        mass, cg = mass_properties(build_model(p), p, 0.0)
        assert mass == pytest.approx(kg, abs=0.01) and cg[2] == pytest.approx(E.CAD_ARM_CG[2] + dz, abs=0.002)
        assert p.pod_sprung_mass == pytest.approx(E.sprung_mass(p, kg), abs=1e-3) and p.pod_sprung_mass < kg - 3.0
    p = E.arm_params()
    assert p.pod_sprung_mass == pytest.approx(5.95, abs=0.1)          # ~ the 6 kg the pods were tuned on
    assert p.roller_pin_mu > 0 and p.roller_mesh == (13, 64)


def test_plain_rollers_drag_with_their_load_and_an_incline_builds():
    """floor.RollingResistance puts mu_pin x load x r_pin on each roller's hinge; floor.Incline is a ramp the wheels
    meet through their own contact pairs."""
    import robot_export as E
    from floor import Incline, edges
    inc = Incline(1.0, 0.0, angle_up=10.0, height=0.1)
    p = E.arm_params(home_xy=(0.0, 0.0), floor_features=(inc,))
    m, d = build_model(p), None
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = Robot(m, d, p)
    for _ in range(400):
        r.step()
    assert m.geom("floorfeat0").id >= 0 and any(abs(e[4] - 0.1) < 1e-9 for e in edges(p))
    rr = [h for h in r.floor if type(h).__name__ == "RollingResistance"][0]
    load = r.wheel_loads()
    pin = p.roller_pin_mu * p.roller_pin_radius * load[0]
    assert m.dof_frictionloss[rr.roller_dofs[0]].min() >= pin - 1e-9 > 0.0
