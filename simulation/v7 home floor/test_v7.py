"""Headless checks for the v7 sim: the v3, v5 and v6 checks it inherits, plus v7's home-floor base (design.md 25).
Run: python -m pytest -q   (a minute or two)"""

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


# --- v5 ----------------------------------------------------------------------------

@pytest.mark.parametrize("limits,lo,hi", [("firmware", 1.3, 1.45), ("physics", 3.0, 3.8)])
def test_top_speed_follows_the_limits_profile(limits, lo, hi):
    """Firmware caps 1.4 m/s; with no cap the X drive reaches its back-EMF limit (~3.4 m/s along its sides)."""
    p, m, d, r = make(SimParams(home_xy=(0.0, 0.0), area_size=(30.0, 30.0), limits=limits))
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
    drv = house.Driver(house.house_params("v7"))
    drv.teleport(house.SPOTS["2"])                           # in front of the kitchen reducer
    for _ in range(int(1.5 / drv.m.opt.timestep)):
        drv.step(np.array([1.0, 0.0]), 0.0, True)
    assert drv.r.pose()[0] > 0.4 and drv.r.ball_held()
    assert "tile" in drv.under_wheels()
