"""Play-catch behavior: toss -> predict -> stop under the ball -> catch -> aim -> launch back.

Runs one physics step at a time (step()), so the same code drives the viewer,
headless trials and the studies. The robot side only uses Robot's sensors and
commands; tossing the ball and the thrower catching the return are "world" actions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from ball_flight import BallFlight
from kiwi_control import wrap_angle
from params import SimParams
from robot import Robot


@dataclass
class GameConfig:
    perception_delay: float = 0.25      # s after release before the first landing estimate
    replan_period: float = 0.05         # landing estimate refresh
    catch_zone_radius: float = 0.37     # tosses aim within this distance of home
    apex_range: tuple = (1.0, 2.0)      # m above the release point
    release_height: float = 1.0
    return_height: float = 1.2          # chest height
    toss_scatter: float = 0.0           # m, extra 1-sigma landing scatter of the human throw
    wait_between: float = 1.0


@dataclass
class TossResult:
    zone_dist: float
    apex: float
    caught: bool
    pos_error: float                    # funnel center to ball at rim height, m
    moved: float                        # robot travel, m
    launched: bool = False
    return_error: float = float("nan")  # return-throw miss distance at chest height, m
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
        self.home = np.array([*p.home_xy, p.home_yaw])
        self.hand = np.array([p.thrower_xy[0] + 0.3, p.thrower_xy[1], cfg.release_height])
        self.target = self.home.copy()
        self.state, self.t_next = "wait", 1.0
        self.results: list[TossResult] = []
        self.predicted_path = None
        j = model.joint("ball")
        self.bq, self.bv = j.qposadr[0], j.dofadr[0]
        self.marker = model.body_mocapid[model.body("landing_marker").id]
        self.rim_z = p.funnel_rim_height

    # -- world side ---------------------------------------------------------
    def place_ball(self, pos, vel=(0, 0, 0)):
        self.d.qpos[self.bq:self.bq + 7] = [*pos, 1, 0, 0, 0]
        self.d.qvel[self.bv:self.bv + 6] = [*vel, 0, 0, 0]

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

    # -- main loop ------------------------------------------------------------
    def step(self):
        """Advance one physics step (behavior + robot models + MuJoCo)."""
        self._behave()
        self.robot.pre_step()
        mujoco.mj_step(self.m, self.d)
        self.robot.post_step()
        self.peak_i = max(getattr(self, "peak_i", 0.0), self.robot.i_total)

    def _behave(self):
        t, r, cfg = self.d.time, self.robot, self.cfg
        pose = r.pose()
        ball_p, ball_v = self.ball()

        if self.state == "wait":
            self.target = np.array([*self.home[:2], self._face(self.home, self.hand)])
            if t >= self.t_next:
                self._toss(pose)

        elif self.state == "flight":
            if t - self.t_release >= cfg.perception_delay and t >= self.t_plan:
                self.t_plan = t + cfg.replan_period
                hit = self.flight.crossing(ball_p, ball_v, self.rim_z)
                if hit is not None:
                    xy = hit[0][:2]
                    if self.predicted is None:
                        self._log(f"  predicted landing {np.linalg.norm(xy - self.home[:2]):.2f} m from home, "
                                  f"{hit[1]:.2f} s to go")
                    self.predicted = xy
                    self.target = np.array([*xy, self._face(xy, self.hand)])
                    self.show_marker(xy)
            if ball_p[2] < self.rim_z and ball_v[2] < 0 and self.cross_err is None:
                self.cross_err = float(np.hypot(*(ball_p[:2] - pose[:2])))
            if r.ball_held() or ball_p[2] < 0.15 or t - self.t_release > 4.0:
                self.state, self.t_settle = "settle", t

        elif self.state == "settle":
            if t - self.t_settle > 0.5:
                caught = r.ball_held()
                res = TossResult(self.zone_dist, self.apex, caught,
                                 self.cross_err if self.cross_err is not None else float("nan"),
                                 float(np.hypot(*(pose[:2] - self.start_pose[:2]))),
                                 peak_current=self.peak_i, v_min=r.battery.v_min)
                self.results.append(res)
                self.show_marker(self.home[:2], visible=False)
                n, k = len(self.results), sum(x.caught for x in self.results)
                self._log(f"toss {n}: {'CAUGHT' if caught else 'missed'} (funnel offset {res.pos_error * 100:.1f} cm, "
                          f"moved {res.moved:.2f} m) - {k}/{n}")
                if caught:
                    self.state, self.t_state = "aim", t
                    self._plan_return(pose)
                    self.final_plan = False
                else:
                    self.state, self.t_next = "wait", t + cfg.wait_between

        elif self.state == "aim":
            self.target = np.array([*self.target[:2], self._face(self.target, self.hand)])
            aimed = abs(wrap_angle(self.target[2] - pose[2])) < math.radians(0.5) and \
                abs(self.d.sensor("base_angvel").data[2]) < 0.05
            if aimed and not self.final_plan:
                self._plan_return(pose)          # refine the flywheel speed from the final pose
                self.final_plan = True
            if (aimed and r.flywheels_ready()) or t - self.t_state > 3.0:
                r.gate_open = True
                self.state, self.t_state = "launch", t
                self.launch_seen = False
                self._log(f"  gate open: flywheels {r.telemetry()['fly_rpm']:.0f} rpm for {self.v_needed:.2f} m/s")

        elif self.state == "launch":
            rel = ball_p - self.d.xpos[r.module]
            if not self.launch_seen and np.linalg.norm(rel[:2]) > 0.3:
                self.launch_seen = True
                v = ball_v
                self._log(f"  launched {np.linalg.norm(v):.2f} m/s at "
                          f"{math.degrees(math.atan2(v[2], math.hypot(v[0], v[1]))):.1f} deg")
            if t - self.t_state > 0.6:
                r.gate_open = False
                self.state, self.t_state = "return", t
                self.results[-1].launched = self.launch_seen
                self.return_track = []

        elif self.state == "return":
            r.fly_target = 0.0
            self.target = np.array([*self.home[:2], self._face(self.home, self.hand)])
            if ball_v[2] < 0 and ball_p[2] < self.cfg.return_height and self.results[-1].launched and \
                    np.isnan(self.results[-1].return_error):
                self.results[-1].return_error = float(np.hypot(*(ball_p[:2] - self.hand[:2])))
                self._log(f"  return throw arrives {self.results[-1].return_error * 100:.0f} cm from the thrower's hands")
            if t - self.t_state > 1.6:
                self.state, self.t_next = "wait", t + 0.4

        r.drive_to(pose, self.target)

    def _toss(self, pose):
        cfg = self.cfg
        a = self.rng.uniform(0, 2 * math.pi)
        rad = cfg.catch_zone_radius * math.sqrt(self.rng.uniform())
        aim = self.home[:2] + rad * np.array([math.cos(a), math.sin(a)])
        self.apex = self.rng.uniform(*cfg.apex_range)
        v0 = self.flight.solve_toss(self.hand, [*aim, self.rim_z], self.apex)
        if cfg.toss_scatter > 0:
            v0[:2] += self.rng.normal(0, cfg.toss_scatter, 2) * 0.8
        self.place_ball(self.hand, v0)
        self.zone_dist = rad
        self.start_pose = pose.copy()
        self.peak_i = 0.0
        self.robot.battery.v_min = self.robot.battery.v
        self.state, self.t_release, self.t_plan = "flight", self.d.time, 0.0
        self.predicted, self.cross_err = None, None
        self._log(f"toss: aim {rad:.2f} m from home, apex {self.apex:.2f} m, release {np.linalg.norm(v0):.2f} m/s")

    def _plan_return(self, pose):
        r = self.robot
        exit_pos = r.launch_exit_world()
        heading = np.array([math.cos(pose[2]), math.sin(pose[2])])
        target = [*self.hand[:2], self.cfg.return_height]
        v = self.flight.solve_launch_speed(exit_pos, heading, math.radians(self.p.launch_angle_deg), target)
        for _ in range(2):    # the calibrated elevation depends on the speed
            el = r.cal.elevation_at(v)
            v = self.flight.solve_launch_speed(exit_pos, heading, el, target)
        self.v_needed = v
        r.fly_target = r.cal.speed_for(v)

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
        if len(ret):
            s += f"; return throws land median {100 * np.median(ret):.0f} cm from the hands"
        r = self.robot
        s += (f"; battery {r.battery.v:.2f} V ({100 * r.battery.soc:.1f}% SoC), "
              f"motor winding max {max(dr.thermal.t_w for dr in r.drives):.1f} C")
        return s
