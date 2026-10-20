"""Play-catch behavior (v3): toss -> predict -> stop under the ball with the funnel
tilted into its path -> catch -> aim (yaw + tilt for elevation) -> launch back.

New in v3: the landing prediction also gives the ball's velocity at the rim,
so the robot faces the incoming ball and tilts the funnel to meet it along its
axis. The base target accounts for the rim center moving forward as the module
tilts. For the return throw, the same tilt joint sets the launch elevation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from ball_flight import BallFlight
from intercept import SafetyZone, choose_intercept
from omni_control import wrap_angle
from params import SimParams
from perception import PerceptionConfig, StereoPerception
from robot import Robot


# Game settings per limits profile (SimParams.limits). "firmware" = v3b: the 20 cm clear-path gate, 3 m/s^2 braking
# with the ball aboard. "physics": no safety margin, but the path still may not run the robot into the netting or
# the thrower (margin 0: that is a crash, not a limit); braking with the ball at the hardest rate that held it in
# studies/envelope.py.
GAME_PROFILES = {
    "firmware": dict(gate=True, brake_with_ball=3.0, plan_margin=0.9, safety_margin=0.20),
    # the ball stayed in at every deceleration the floor allows (envelope.py F: 4.3-5.4 m/s^2 from 2.5 m/s),
    # so braking with the ball aboard is only capped by the traction table
    "physics": dict(gate=True, brake_with_ball=math.inf, plan_margin=0.95, safety_margin=0.0),
}


@dataclass
class GameConfig:
    limits: str = "physics"             # which GAME_PROFILES fill the None fields below (match SimParams.limits)
    perception: str = "truth"           # "truth": exact ball state after perception_delay; "model": stereo cameras +
                                        # tracking filter (perception.py); "vision": per-pixel blob detector, stereo +
                                        # known-size ranging, image-space filter (vision.py)
    perception_cfg: PerceptionConfig = field(default_factory=PerceptionConfig)
    vision_cfg: object = None           # vision.VisionConfig for perception="vision" (None = defaults)
    catch_mode: str = "stop"            # "stop": stop under the ball; "moving": meet it at a planned velocity when
                                        # stopping in time isn't possible (intercept.py)
    brake_with_ball: float | None = None    # m/s^2 after a catch (v3 moving_rig: the ball stays in up to 4)
    safety_margin: float | None = None  # m: moving catches only if the path and the stop after keep this clearance
    person_radius: float = 0.30         # m around the thrower
    coast_moving: float = 0.05          # s of coast after a moving catch: braking can start with the dump (moving_rig.py:
                                        # held up to 4 m/s^2), and every 0.1 s of coast at 2 m/s costs 0.2 m of stopping room
    gate: bool | None = None            # False: ignore walls and people (a reference for what the safety rule costs)
    plan_margin: float | None = None    # the intercept planner assumes this fraction of the speed and acceleration limits
                                        # (the motors fade near top speed)
    moving_commit_sigma: float = 0.10   # m: commit to a moving catch only once the landing prediction is this good
                                        # (1-sigma); before that, head for the estimate as a stop-and-catch
    perception_delay: float = 0.25
    replan_period: float = 0.05
    catch_zone_radius: float = 0.37
    zone_mode: str = "disk"             # "disk": landing points uniform within the radius; "ring": exactly at it
    return_throws: bool = True          # False: skip aiming and launching (faster sweeps)
    apex_range: tuple = (1.0, 2.0)
    release_height: float = 1.0
    return_height: float = 1.2
    return_elevation_deg: float = 50.0  # launch elevation for the return lob
    toss_scatter: float = 0.0
    toss_spin: float = 0.0              # rad/s, 1-sigma per axis of the thrown ball's spin (MuJoCo's air model has
                                        # Magnus lift; the predictor doesn't). 0 = v3's spin-free tosses (est: an
                                        # underhand lob spins ~5-15 rad/s; VERIFY by filming throws)
    wait_between: float = 1.0
    align_tilt: bool = True             # tilt the funnel into the ball's path (False: stay at 0)
    tilt_gain: float = 0.6              # fraction of the arrival angle: the ball still glances off one wall
                                        # (fully aligned, a centered ball hits the gate at full speed and bounces out)
    dump_delay: float = 0.03            # s after the ball enters: swing the funnel upright so it rolls into the throat
    coast_after_catch: float = 0.2      # s to keep the current velocity after the ball enters, before braking (a ball
                                        # in a tilted pocket rolls forward if the base brakes before the dump is done)
    max_catch_tilt_deg: float = 15.0    # with the kicker's floor at 30 deg, a 25 deg catch tilt leaves only 5 deg of
                                        # back-slope; line drives caught on the move rolled out (hard cells: 169 -> 177/180)
    tilt_freeze: float = 0.15           # s before arrival after which the tilt and heading stop changing
    learn_return: bool = True           # correct the throw from logged landings (the design's calibration refinement);
                                        # the predictor ignores spin, and each feed leaves its own spin and range bias
    learn_rate: float = 0.25            # 0.5 chased throw-to-throw scatter

    def __post_init__(self):
        for k, v in GAME_PROFILES[self.limits].items():
            if getattr(self, k) is None:
                setattr(self, k, v)


@dataclass
class TossResult:
    zone_dist: float
    apex: float
    caught: bool
    pos_error: float                    # funnel center to ball where the ball crosses the rim-center height, m
    moved: float
    arrival_deg: float = float("nan")   # ball velocity from vertical at the rim
    tilt_deg: float = float("nan")      # funnel tilt at arrival
    arrival_speed: float = float("nan")
    flight_time: float = float("nan")   # release to rim crossing
    launched: bool = False
    return_error: float = float("nan")
    return_range_err: float = float("nan")  # signed: + = long, along the throw
    heading_off_deg: float = float("nan")   # robot heading vs bearing to the thrower at the catch
    aim_turn_deg: float = float("nan")      # yaw the base turns between the catch and the launch
    aim_time: float = float("nan")          # s from the catch decision to the gate opening
    catch_speed: float = float("nan")       # base speed when the ball reached the rim, m/s
    moving: bool = False                    # the plan at the freeze was a moving catch
    gated: bool = False                     # a moving catch was needed but the clear-path gate refused it
    land_sigma: float = float("nan")        # landing-prediction 1-sigma (m, larger axis) at the freeze (perception model)
    peak_current: float = 0.0
    v_min: float = 0.0


class CatchGame:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, p: SimParams,
                 cfg: GameConfig = GameConfig(), seed: int = 0, verbose: bool = True):
        self.m, self.d, self.p, self.cfg = model, data, p, cfg
        self.robot = Robot(model, data, p)
        self.verbose = verbose
        b = p.ball_spec
        self.flight = BallFlight(b.mass, b.radius, b.drag_cd, p.air_density, p.air_viscosity)
        self.rng = np.random.default_rng(seed)
        self.spin_rng = np.random.default_rng(seed + 99)
        self.home = np.array([*p.home_xy, p.home_yaw])
        self.hand = np.array([p.thrower_xy[0] + 0.3, p.thrower_xy[1], cfg.release_height])
        self.target = self.home.copy()
        self.state, self.t_next = "wait", 1.0
        self.results: list[TossResult] = []
        j = model.joint("ball")
        self.bq, self.bv = j.qposadr[0], j.dofadr[0]
        self.marker = model.body_mocapid[model.body("landing_marker").id]
        self.range_bias = 0.0           # learned from logged return throws (m, + = they land long)
        g = self.robot.g
        self.exit_m = (g.N[0] + 0.06 * g.u[0], g.N[1] + 0.06 * g.u[1])
        self.peak_i = 0.0
        self.perc = None
        if cfg.perception == "model":
            self.perc = StereoPerception(model, data, self.flight, p, cfg.perception_cfg, np.random.default_rng(seed + 7))
        elif cfg.perception == "vision":
            from vision import VisionConfig, VisionPerception
            self.perc = VisionPerception(model, data, self.flight, p, cfg.vision_cfg or VisionConfig(),
                                         np.random.default_rng(seed + 7))
        ax, ay = p.area_size
        self.zone = (SafetyZone(half_x=ax / 2, half_y=ay / 2, margin=cfg.safety_margin,
                                people=[(p.thrower_xy[0], p.thrower_xy[1], cfg.person_radius)])
                     if cfg.gate else SafetyZone(half_x=1e3, half_y=1e3, margin=0.0))
        self.v_c = np.zeros(2)
        self.cross_err = None
        self.moving_plan, self.gated, self.stopped, self.t_in, self.t_c, self.v_end = False, False, False, 0.0, 0.0, 0.0

    # -- world side ---------------------------------------------------------
    def place_ball(self, pos, vel=(0, 0, 0), spin=(0, 0, 0)):
        self.d.qpos[self.bq:self.bq + 7] = [*pos, 1, 0, 0, 0]
        self.d.qvel[self.bv:self.bv + 6] = [*vel, *spin]

    def show_marker(self, xy, visible=True):
        self.d.mocap_pos[self.marker] = [xy[0], xy[1], 0.002 if visible else -0.05]

    # -- helpers ------------------------------------------------------------
    def _log(self, msg):
        if self.verbose:
            print(f"[{self.d.time:7.2f}s] {msg}")

    def _face(self, xy, toward):
        return math.atan2(toward[1] - xy[1], toward[0] - xy[0])

    def ball(self):
        return self.d.sensor("ball_pos").data.copy(), self.d.sensor("ball_vel").data.copy()

    def catch_plan(self, ball_p, ball_v, v_robot=(0.0, 0.0)):
        """Base pose and tilt that put the tilted funnel's rim center on the ball's path, facing and
        tilting into the ball's velocity relative to a base moving at v_robot.
        Returns (target pose, tilt, crossing state) or None."""
        vr = np.array([v_robot[0], v_robot[1], 0.0])
        g = self.robot.g
        lo, hi = self.p.tilt_limits
        tilt = self.robot.tilt_target
        hit = None
        for _ in range(3):                      # the rim height depends on the tilt
            x_rc, z_rc = g.rim_center_robot(tilt)
            hit = self.flight.crossing_state(ball_p, ball_v, z_rc)
            if hit is None:
                return None
            v = hit[1] - vr
            ang = math.atan2(math.hypot(v[0], v[1]), -v[2])
            hi_c = min(hi, math.radians(self.cfg.max_catch_tilt_deg))
            tilt = float(np.clip(self.cfg.tilt_gain * ang, max(lo, 0.0), hi_c)) if self.cfg.align_tilt else 0.0
        xy, _, t_go = hit
        yaw = math.atan2(-v[1], -v[0]) if math.hypot(v[0], v[1]) > 0.3 else self._face(xy, self.hand)
        x_rc, _ = g.rim_center_robot(tilt)
        base_xy = xy[:2] - x_rc * np.array([math.cos(yaw), math.sin(yaw)])
        return np.array([*base_xy, yaw]), tilt, hit

    # -- main loop ------------------------------------------------------------
    def step(self):
        self._behave()
        self.robot.pre_step()
        mujoco.mj_step(self.m, self.d)
        self.robot.post_step()
        self.peak_i = max(self.peak_i, self.robot.i_total)

    def _behave(self):
        t, r, cfg = self.d.time, self.robot, self.cfg
        pose = r.pose()
        ball_p, ball_v = self.ball()

        if self.state == "wait":
            self.target = np.array([*self.home[:2], self._face(self.home, self.hand)])
            r.tilt_target = 0.0
            if t >= self.t_next:
                self._toss(pose)

        elif self.state == "flight":
            if self.perc is not None and self.cross_err is None:
                self.perc.step(t)
            if self.perc is None:
                have = t - self.t_release >= cfg.perception_delay
                est_p, est_v = ball_p, ball_v
            else:
                have = self.perc.ready()
                if have and t >= self.t_plan and not self.frozen:     # only when replanning: propagation is costly
                    est_p, est_v, _ = self.perc.estimate(t)
            if have and t >= self.t_plan and not self.frozen:
                self.t_plan = t + cfg.replan_period
                plan = self.catch_plan(est_p, est_v, self.v_c)
                if plan is not None and cfg.catch_mode == "moving":
                    tgt, _, (xy, v, t_go) = plan
                    confident = True
                    if self.perc is not None:
                        cr = self.perc.crossing(t, xy[2])
                        confident = cr is not None and math.sqrt(max(np.linalg.eigvalsh(cr[3]))) < cfg.moving_commit_sigma
                    k = cfg.plan_margin
                    # the braking curve decelerates at brake_margin x the worst-direction limit: plan with that;
                    # top speed toward the meeting point (the wheels' ceiling with no cap)
                    dvec = tgt[:2] - pose[:2]
                    nd = float(np.linalg.norm(dvec))
                    v_top = r.ctl.top_speed(dvec / nd, pose[2]) if nd > 1e-6 else r.ctl.top_speed((1.0, 0.0), pose[2])
                    ch = choose_intercept(pose[:2], r.ctl.v_cmd.copy(), tgt[:2], t_go,
                                          a=r.ctl.brake_margin * self._plan_accel(), v_max=k * v_top, zone=self.zone,
                                          coast=cfg.coast_moving, a_ball=min(cfg.brake_with_ball, self._plan_accel()),
                                          moving=confident)
                    self.v_end = ch.v_f
                    self.v_c = ch.v_f * ch.direction
                    self.gated = self.gated or ch.gated
                    plan = self.catch_plan(est_p, est_v, self.v_c)     # face and tilt into the relative velocity
                if plan is not None:
                    tgt, tilt, (xy, v, t_go) = plan
                    if self.predicted is None:
                        self._log(f"  predicted landing {np.linalg.norm(xy[:2] - self.home[:2]):.2f} m from home, "
                                  f"{t_go:.2f} s to go, arriving {math.degrees(math.atan2(math.hypot(v[0], v[1]), -v[2])):.0f} deg "
                                  f"from vertical -> tilt {math.degrees(tilt):.0f} deg")
                    self.predicted = xy
                    self.target = tgt
                    self.t_c = t + t_go
                    r.tilt_target = tilt
                    self.show_marker(xy)
                    if t_go < cfg.tilt_freeze:
                        self.frozen = True
                        self.moving_plan = self.v_end > 0.3
                        if self.perc is not None:
                            cr = self.perc.crossing(t, r.g.rim_center_robot(tilt)[1])
                            if cr is not None:
                                self.land_sigma = float(math.sqrt(max(np.linalg.eigvalsh(cr[3]))))
            fc = r.funnel_center_world()
            if self.cross_err is None and ball_v[2] < 0 and ball_p[2] < fc[2]:
                self.cross_err = float(np.hypot(*(ball_p[:2] - fc[:2])))
                self.arrival = math.degrees(math.atan2(math.hypot(ball_v[0], ball_v[1]), -ball_v[2]))
                self.tilt_at_arrival = math.degrees(r.tilt())
                self.arrival_speed = float(np.linalg.norm(ball_v))
                self.catch_speed = float(np.linalg.norm(self.d.sensor("base_linvel").data[:2]))
                self.t_in = t
            if r.ball_held() or ball_p[2] < 0.15 or t - self.t_release > 4.0:
                self.state, self.t_settle = "settle", t

        if self.state in ("flight", "settle") and self.cross_err is not None and t - self.t_in > cfg.dump_delay:
            r.tilt_target = 0.0                  # dump: a ball resting on the shallow far wall rolls in

        if self.state == "settle":
            # wait while the ball is still moving inside the funnel (a bounce off the gate can land it back
            # on the wall, from where it rolls in); give up after 2 s
            rel = ball_p - r.funnel_center_world()
            in_funnel = np.hypot(rel[0], rel[1]) < self.p.funnel_rim_radius and rel[2] < 0.15
            busy = in_funnel and not r.ball_held() and np.linalg.norm(ball_v) > 0.02
            busy = busy or float(np.linalg.norm(self.d.sensor("base_linvel").data[:2])) > 0.05   # still braking
            if t - self.t_settle > 0.5 and (not busy or t - self.t_settle > 3.0):
                caught = r.ball_held()
                res = TossResult(self.zone_dist, self.apex, caught,
                                 self.cross_err if self.cross_err is not None else float("nan"),
                                 float(np.hypot(*(pose[:2] - self.start_pose[:2]))),
                                 arrival_deg=self.arrival, tilt_deg=self.tilt_at_arrival,
                                 arrival_speed=self.arrival_speed,
                                 flight_time=(self.t_in - self.t_release) if self.cross_err is not None else float("nan"),
                                 catch_speed=self.catch_speed, moving=self.moving_plan, gated=self.gated,
                                 land_sigma=self.land_sigma, peak_current=self.peak_i, v_min=r.battery.v_min)
                self.results.append(res)
                self.show_marker(self.home[:2], visible=False)
                n, k = len(self.results), sum(x.caught for x in self.results)
                self._log(f"toss {n}: {'CAUGHT' if caught else 'missed'} (funnel offset {res.pos_error * 100:.1f} cm, "
                          f"arrival {res.arrival_deg:.0f} deg, tilt {res.tilt_deg:.0f} deg, moved {res.moved:.2f} m) - {k}/{n}")
                if caught and not cfg.return_throws:
                    self.state, self.t_next = "wait", t + cfg.wait_between
                    self.place_ball((self.p.thrower_xy[0], self.p.thrower_xy[1] + 0.6, 0.04))   # hand it back
                elif caught:
                    res.heading_off_deg = math.degrees(abs(wrap_angle(pose[2] - self._face(pose, self.hand))))
                    self.yaw_at_catch = pose[2]
                    self.state, self.t_state = "aim", t
                    self.target = np.array([*pose[:2], self._face(pose, self.hand)])
                    self._plan_return(self.target, final=False)
                    self.final_plan = False
                    self.t_ready = None
                else:
                    self.state, self.t_next = "wait", t + cfg.wait_between

        elif self.state == "aim":
            aimed = abs(wrap_angle(self.target[2] - pose[2])) < math.radians(0.5) and \
                abs(self.d.sensor("base_angvel").data[2]) < 0.05 and r.tilt_settled()
            if aimed and not self.final_plan:
                self._plan_return(pose, final=True)       # refine from the final pose and the real exit point
                self.final_plan = True
            # fire only once the flywheels have held speed for 0.1 s and the base is still (its velocity
            # adds to the ball's)
            ready = r.flywheels_ready()
            self.t_ready = (self.t_ready or t) if ready else None
            still = float(np.linalg.norm(self.d.sensor("base_linvel").data[:2])) < 0.03
            if (aimed and self.final_plan and ready and t - self.t_ready >= 0.1 and still and r.tilt_settled()) \
                    or t - self.t_state > 3.0:
                r.gate_open = True
                self.results[-1].aim_turn_deg = math.degrees(abs(wrap_angle(pose[2] - self.yaw_at_catch)))
                self.results[-1].aim_time = t - self.t_state
                self.state, self.t_state = "launch", t
                self.launch_seen = False
                self._log(f"  gate open: tilt {math.degrees(r.tilt()):.1f} deg, flywheels {r.telemetry()['fly_rpm']:.0f} rpm "
                          f"for {self.v_needed:.2f} m/s at {math.degrees(self.elev):.1f} deg")

        elif self.state == "launch":
            rel = ball_p - self.d.xpos[r.module]
            if not self.launch_seen and np.linalg.norm(rel) > 0.4:
                self.launch_seen = True
                v = ball_v
                self._log(f"  launched {np.linalg.norm(v):.2f} m/s at "
                          f"{math.degrees(math.atan2(v[2], math.hypot(v[0], v[1]))):.1f} deg")
            if t - self.t_state > 0.6:
                r.gate_open = False
                self.state, self.t_state = "return", t
                self.results[-1].launched = self.launch_seen

        elif self.state == "return":
            r.fly_target = 0.0
            r.tilt_target = 0.0
            self.target = np.array([*self.home[:2], self._face(self.home, self.hand)])
            if ball_v[2] < 0 and ball_p[2] < self.cfg.return_height and self.results[-1].launched and \
                    np.isnan(self.results[-1].return_error):
                self.results[-1].return_error = float(np.hypot(*(ball_p[:2] - self.hand[:2])))
                h = np.array([math.cos(pose[2]), math.sin(pose[2])])
                rng_err = float((ball_p[:2] - self.hand[:2]) @ h)
                self.results[-1].return_range_err = rng_err
                if cfg.learn_return:
                    self.range_bias += cfg.learn_rate * rng_err
                self._log(f"  return throw arrives {self.results[-1].return_error * 100:.0f} cm from the thrower's hands")
            if t - self.t_state > 1.6:
                self.state, self.t_next = "wait", t + 0.4

        after = self.state in ("flight", "settle") and self.cross_err is not None
        coast = cfg.coast_moving if (self.moving_plan or float(np.linalg.norm(self.v_c)) > 0.05) else cfg.coast_after_catch
        if after and t - self.t_in < coast:
            r.drive_velocity(r.ctl.v_cmd.copy(), 0.0, pose[2])     # hold until the dump has tipped the floor back
        elif after and not self.stopped:
            # then brake gently with the ball aboard, and hold wherever the base comes to rest
            if float(np.linalg.norm(r.ctl.v_cmd)) > 0.02:
                r.drive_velocity((0.0, 0.0), 0.0, pose[2], a_cap=cfg.brake_with_ball)
            else:
                self.stopped = True
                self.target = pose.copy()
                r.drive_to(pose, self.target)
        elif self.state == "flight" and self.v_end > 0.0 and self.predicted is not None:
            # moving catch: the braking curve ends at v_end instead of rest, so the funnel passes the
            # meeting point on time; replanning lowers v_end if it gets ahead
            r.drive_to(pose, self.target, v_end=self.v_end)
        else:
            r.drive_to(pose, self.target)

    def _plan_accel(self) -> float:
        """Acceleration the intercept planner assumes: the worst direction of the firmware limit."""
        ctl = self.robot.ctl
        return float(min(ctl.accel_table[1])) if ctl.accel_table is not None else ctl.max_accel

    def _toss(self, pose):
        cfg = self.cfg
        a = self.rng.uniform(0, 2 * math.pi)
        rad = cfg.catch_zone_radius * (1.0 if cfg.zone_mode == "ring" else math.sqrt(self.rng.uniform()))
        aim = self.home[:2] + rad * np.array([math.cos(a), math.sin(a)])
        self.apex = self.rng.uniform(*cfg.apex_range)
        z_aim = self.robot.g.rim_center_robot(0.0)[1]
        v0 = self.flight.solve_toss(self.hand, [*aim, z_aim], self.apex)
        if cfg.toss_scatter > 0:
            v0[:2] += self.rng.normal(0, cfg.toss_scatter, 2) * 0.8
        # spin from its own generator, so the throws stay the same with or without it
        spin = self.spin_rng.normal(0, cfg.toss_spin, 3) if cfg.toss_spin > 0 else np.zeros(3)
        self.place_ball(self.hand, v0, spin)
        self.zone_dist = rad
        self.start_pose = pose.copy()
        self.peak_i = 0.0
        self.robot.battery.v_min = self.robot.battery.v
        self.state, self.t_release, self.t_plan = "flight", self.d.time, 0.0
        self.predicted, self.cross_err, self.frozen = None, None, False
        self.arrival = self.tilt_at_arrival = self.arrival_speed = self.catch_speed = self.land_sigma = float("nan")
        self.v_c, self.gated, self.moving_plan, self.stopped, self.t_c = np.zeros(2), False, False, False, 0.0
        self.v_end = 0.0
        if self.perc is not None:
            self.perc.reset()
            self.perc.t_next = self.d.time
        self._log(f"toss: aim {rad:.2f} m from home, apex {self.apex:.2f} m, release {np.linalg.norm(v0):.2f} m/s")

    def _exit_world(self, pose, tilt):
        x, z = self.robot.g.module_to_robot(self.exit_m, tilt)
        return np.array([pose[0] + x * math.cos(pose[2]), pose[1] + x * math.sin(pose[2]), z])

    def _plan_return(self, pose, final: bool):
        """Exit speed for the chosen elevation, and the tilt that gives that elevation."""
        r, cal = self.robot, self.robot.cal
        heading = np.array([math.cos(pose[2]), math.sin(pose[2])])
        aim_xy = self.hand[:2] - self.range_bias * heading
        target = [*aim_xy, self.cfg.return_height]
        el = math.radians(self.cfg.return_elevation_deg)
        tilt = cal.tilt_for(5.0, el)
        v = 5.0
        for _ in range(3):
            exit_pos = r.launch_exit_world() if final else self._exit_world(pose, tilt)
            v = self.flight.solve_launch_speed(exit_pos, heading, el, target)
            tilt = cal.tilt_for(v, el)
        lo, hi = self.p.tilt_limits
        tilt = float(np.clip(tilt, lo, hi))
        if final:                      # the tilt is fixed now: take the elevation it really gives
            tilt = r.tilt_target
            for _ in range(2):
                el = cal.elevation_at(v, tilt)
                v = self.flight.solve_launch_speed(r.launch_exit_world(), heading, el, target)
        self.v_needed, self.elev = v, el
        r.tilt_target = tilt
        r.fly_target = cal.speed_for(v, tilt)

    # -- reporting --------------------------------------------------------------
    def summary(self) -> str:
        n = len(self.results)
        if not n:
            return "no tosses completed"
        k = sum(x.caught for x in self.results)
        errs = np.array([x.pos_error for x in self.results])
        errs = errs[np.isfinite(errs)]
        ret = np.array([x.return_error for x in self.results if x.launched])
        ret = ret[np.isfinite(ret)]
        s = (f"{k}/{n} caught ({100 * k / n:.0f}%), funnel offset median {100 * np.median(errs):.1f} cm"
             f" (max {100 * np.max(errs):.1f})")
        arr = np.array([x.arrival_deg for x in self.results])
        til = np.array([x.tilt_deg for x in self.results])
        ok = np.isfinite(arr) & np.isfinite(til)
        if ok.any():
            s += f"; arrival {np.min(arr[ok]):.0f}-{np.max(arr[ok]):.0f} deg from vertical, funnel tilt " \
                 f"{np.min(til[ok]):.0f}-{np.max(til[ok]):.0f} deg"
        if len(ret):
            s += f"; return throws land median {100 * np.median(ret):.0f} cm from the hands"
            if self.cfg.learn_return and len(ret) > 8:
                s += (f" ({100 * np.median(ret[5:]):.0f} cm after the first 5; learned range bias "
                      f"{100 * self.range_bias:+.0f} cm)")
        turn = np.array([x.aim_turn_deg for x in self.results if x.launched])
        if len(turn):
            s += (f"; yaw to aim median {np.median(turn):.1f} deg (max {np.max(turn):.1f}), aim time median "
                  f"{np.median([x.aim_time for x in self.results if x.launched]):.2f} s")
        r = self.robot
        s += (f"; battery {r.battery.v:.2f} V ({100 * r.battery.soc:.1f}% SoC), "
              f"motor winding max {max(dr.thermal.t_w for dr in r.drives):.1f} C")
        return s
