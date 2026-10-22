"""The play-catch behavior on the Jetson (design.md 8, 9, 22, 30): the robot half of the sim's catch_game.CatchGame.

  wait     at home, facing the thrower; the perception's track starting on an incoming ball starts a catch
  flight   predict where the ball crosses the funnel rim (tracker), plan a stop or a moving catch under it
           (intercept.choose_intercept, the clear-path gate), tilt the funnel into its path; tilt_freeze s before
           arrival freeze the plan and hand the MCU the schedule: hold the velocity for the coast, brake with the ball
           aboard, hold; dump the funnel upright dump_delay s after arrival
  settle   wait for the base to stop; the catch beam says caught or not
  aim      turn to the thrower, set tilt and flywheel speed for the return lob (launcher calibration); fire when
           aimed, still, and the flywheels have held speed for 0.1 s
  launch   0.6 s for the feed and flywheels to throw it
  return   flywheels off, back home

Every decision uses only what the robot has: the track (TrackView), the MCU's STATE (RobotView: odometry, tilt,
flywheels, beams) and the robot config. Positions are in the MCU's odom frame. Firmware-side timing (the coast,
brake and dump) is scheduled in MCU time through to_mcu_us. What the sim's game read from the world (the true ball,
the true crossing time) is replaced by the prediction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from omni_catch.flight import BallFlight
from omni_catch.intercept import SafetyZone, choose_intercept
from omni_catch.robot_config import RobotConfig

DRIVE_IDLE, DRIVE_VEL_BODY, DRIVE_GOTO = 0, 1, 2


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class GameParams:
    """catch_game.GameConfig, firmware adult profile + v6's catch module (shallow_sweep.FWG, V6G)."""
    catch_mode: str = "moving"
    brake_with_ball: float = 3.0
    plan_margin: float = 0.9
    safety_margin: float = 0.20
    gate: bool = True
    person_radius: float = 0.30
    coast_moving: float = 0.05
    coast_after_catch: float = 0.2
    moving_commit_sigma: float = 0.10
    replan_period: float = 0.05
    align_tilt: bool = True
    tilt_gain: float = 0.6
    max_catch_tilt_deg: float = 35.0
    tilt_freeze: float = 0.15
    dump_delay: float = 0.03
    return_throws: bool = True
    return_height: float = 1.2
    return_elevation_deg: float = 50.0
    min_return_elevation_deg: float = 30.0
    ceiling: float | None = None
    ceiling_margin: float = 0.25
    fire_settle: float = 0.0
    release_height: float = 1.0
    home: tuple = (0.0, 0.0)            # odom frame: where the robot waits
    thrower: tuple = (6.8, 0.0)         # odom frame: the thrower's hand, a first guess; refined from each throw
    arena_center: tuple = (3.5, 0.0)    # odom frame: the play area (from the map)
    arena_half: tuple = (6.0, 3.0)
    learn_thrower: bool = True
    kid: bool = False


@dataclass
class RobotView:
    """What the behavior knows about the robot: the MCU's last STATE."""
    t: float = 0.0
    pose: np.ndarray = field(default_factory=lambda: np.zeros(3))      # x, y, yaw (odom)
    vel_body: np.ndarray = field(default_factory=lambda: np.zeros(3))  # vx, vy, wz
    v_cmd: np.ndarray = field(default_factory=lambda: np.zeros(2))     # the firmware's commanded velocity (odom)
    tilt: float = 0.0
    tilt_cmd: float = 0.0
    fly_rpm: tuple = (0.0, 0.0)
    fly_target_rpm: float = 0.0
    fly_ready: bool = False
    ball_held: bool = False
    fire_seq_done: int = 0
    mode: str = "DISARMED"

    @classmethod
    def from_state(cls, s) -> "RobotView":
        return cls(t=s.t_host, pose=np.array([s.x, s.y, s.yaw]), vel_body=np.array([s.vx, s.vy, s.wz]),
                   v_cmd=np.array([s.v_cmd_x, s.v_cmd_y]), tilt=s.tilt, tilt_cmd=s.tilt_cmd, fly_rpm=tuple(s.fly_rpm),
                   fly_target_rpm=s.fly_target_rpm, fly_ready=bool(s.fly_ready), ball_held=s.ball_held,
                   fire_seq_done=s.fire_seq_done, mode=s.mode_name)


@dataclass
class CatchCommand:
    """The behavior's request; the mode manager turns it into an MCU Command."""
    drive_mode: int = DRIVE_GOTO
    target: tuple = (0.0, 0.0, 0.0)
    v_end: float = 0.0
    t_brake: float = 0.0             # host time (s); 0 = none
    coast_s: float = 0.0
    a_brake: float = 0.0
    tilt: float = 0.0
    t_tilt: float = 0.0              # host time (s); 0 = none
    tilt_after: float = 0.0
    fly_rpm: float = 0.0
    fire_seq: int = 0


class CatchBehavior:
    def __init__(self, robot: RobotConfig | None = None, params: GameParams | None = None,
                 flight: BallFlight | None = None, log=None):
        self.cfg = robot or RobotConfig()
        self.p = params or GameParams()
        b = self.cfg.ball
        self.flight = flight or BallFlight(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])
        self.log = log or (lambda msg: None)
        self.state, self.t_state = "wait", 0.0
        self.t_beam = None
        self.hand = np.array([*self.p.thrower, self.p.release_height])
        self.home = np.array([*self.p.home, self._face(self.p.home, self.hand)])
        self.target = self.home.copy()
        self.tilt_target = 0.0
        self.fly_rpm = 0.0
        self.fire_seq = 0
        self.range_bias = 0.0
        self.tosses = 0
        self.catches = 0
        self.last = {}                    # the last catch's record (for logs and the scorer)
        self._new_toss_state()

    def _new_toss_state(self):
        self.t_flight = 0.0
        self.t_plan = 0.0
        self.predicted = None
        self.frozen = False
        self.t_c = None
        self.v_c = np.zeros(2)
        self.v_end = 0.0
        self.moving_plan = False
        self.gated = False
        self.land_sigma = math.nan
        self.t_track_seen = 0.0

    # -- helpers ------------------------------------------------------------------------------------------------------
    @staticmethod
    def _face(xy, toward) -> float:
        return math.atan2(toward[1] - xy[1], toward[0] - xy[0])

    def zone(self) -> SafetyZone:
        p = self.p
        if not p.gate:
            return SafetyZone(half_x=1e3, half_y=1e3, margin=0.0)
        return SafetyZone(half_x=p.arena_half[0], half_y=p.arena_half[1], margin=p.safety_margin,
                          people=[(self.hand[0], self.hand[1], p.person_radius)], robot_radius=self.cfg.robot_radius,
                          center=p.arena_center)

    def catch_plan(self, ball_p, ball_v, v_robot=(0.0, 0.0)):
        """Base pose and tilt that put the tilted funnel's rim center on the ball's path (catch_game.catch_plan)."""
        vr = np.array([v_robot[0], v_robot[1], 0.0])
        lo, hi = self.cfg.tilt_limits
        tilt = self.tilt_target
        hit = v = None
        for _ in range(3):
            x_rc, z_rc = self.cfg.rim_center(tilt)
            hit = self.flight.crossing_state(ball_p, ball_v, z_rc)
            if hit is None:
                return None
            v = hit[1] - vr
            ang = math.atan2(math.hypot(v[0], v[1]), -v[2])
            hi_c = min(hi, math.radians(self.p.max_catch_tilt_deg))
            tilt = float(np.clip(self.p.tilt_gain * ang, max(lo, 0.0), hi_c)) if self.p.align_tilt else 0.0
        xy, _, t_go = hit
        yaw = math.atan2(-v[1], -v[0]) if math.hypot(v[0], v[1]) > 0.3 else self._face(xy, self.hand)
        x_rc, _ = self.cfg.rim_center(tilt)
        base_xy = xy[:2] - x_rc * np.array([math.cos(yaw), math.sin(yaw)])
        return np.array([*base_xy, yaw]), tilt, hit

    def _exit_world(self, pose, tilt):
        x, z = self.cfg.exit_point(tilt)
        return np.array([pose[0] + x * math.cos(pose[2]), pose[1] + x * math.sin(pose[2]), z])

    def _plan_return(self, pose, final: bool, tilt_meas: float):
        """Exit speed for the chosen elevation, and the tilt that gives it (catch_game._plan_return)."""
        cfg, fl = self.cfg, self.flight
        heading = np.array([math.cos(pose[2]), math.sin(pose[2])])
        aim_xy = self.hand[:2] - self.range_bias * heading
        target = [*aim_xy, self.p.return_height]

        def solve(el):
            tilt, v = cfg.tilt_for(5.0, el), 5.0
            exit_pos = None
            for _ in range(3):
                exit_pos = self._exit_world(pose, tilt_meas if final else tilt)
                v = fl.solve_launch_speed(exit_pos, heading, el, target)
                tilt = cfg.tilt_for(v, el)
            vel = v * np.array([math.cos(el) * heading[0], math.cos(el) * heading[1], math.sin(el)])
            return tilt, v, fl.apex(exit_pos, vel)

        el = math.radians(self.p.return_elevation_deg)
        tilt, v, apex = solve(el)
        if self.p.ceiling is not None and not final:
            top = self.p.ceiling - self.p.ceiling_margin
            el_deg = self.p.return_elevation_deg
            while apex > top and el_deg > self.p.min_return_elevation_deg:
                el_deg -= 1.0
                el = math.radians(el_deg)
                tilt, v, apex = solve(el)
        lo, hi = cfg.tilt_limits
        tilt = float(np.clip(tilt, lo, hi))
        if final:                       # the tilt is set now: take the elevation it really gives
            tilt = self.tilt_target
            for _ in range(2):
                el = cfg.elevation_at(v, tilt)
                v = fl.solve_launch_speed(self._exit_world(pose, tilt_meas), heading, el, target)
        self.v_needed, self.elev = v, el
        self.tilt_target = tilt
        self.fly_rpm = cfg.speed_for(v, tilt) * 60 / (2 * math.pi)

    def _set(self, state: str, t: float):
        self.state, self.t_state = state, t
        self.t_beam = None

    def _incoming(self, t, pose, track):
        """(position, velocity) of a ball flying toward the robot, from a track ready to plan on, or None."""
        if track is None or not track.ready:
            return None
        bp, bv, _ = track.estimate(t)
        if float(np.linalg.norm(bv)) > 2.0 and float(bv[:2] @ (pose[:2] - bp[:2])) > 0 and bp[2] > 0.3:
            return bp, bv
        return None

    def _settle_done(self, t, rv, pose):
        caught = rv.ball_held
        self.catches += caught
        self.last = dict(toss=self.tosses, caught=caught, moving=self.moving_plan, gated=self.gated,
                         land_sigma=self.land_sigma, v_end=self.v_end,
                         predicted=None if self.predicted is None else [float(c) for c in self.predicted])
        self.log(f"toss {self.tosses}: {'CAUGHT' if caught else 'missed'} ({self.catches}/{self.tosses})")
        if caught and self.p.return_throws:
            self._set("aim", t)
            self.target = np.array([*pose[:2], self._face(pose, self.hand)])
            self._plan_return(self.target, final=False, tilt_meas=rv.tilt)
            self.final_plan = False
            self.t_ready = None
        else:
            self._set("wait", t)

    # -- the step --------------------------------------------------------------------------------------------------
    def step(self, t: float, rv: RobotView, track) -> CatchCommand:
        """track: a TrackView (or None). Returns what to ask of the MCU now."""
        p = self.p
        pose = rv.pose
        cmd = CatchCommand()
        if track is not None:
            self.t_track_seen = t

        incoming = self._incoming(t, pose, track)
        if self.state == "return" and incoming is not None:
            self._set("wait", t)                      # a throw while going home: catch it from here
        if self.state == "settle" and incoming is not None and t - self.t_state > 0.3:
            self._settle_done(t, rv, pose)            # the next throw is coming: decide now
        if self.state == "wait":
            self.target = self.home.copy()
            self.target[2] = self._face(self.home, self.hand)
            self.tilt_target, self.fly_rpm = 0.0, 0.0
            if incoming is not None:
                bp, bv = incoming
                self._new_toss_state()
                self._set("flight", t)
                self.t_flight = t
                self.tosses += 1
                if p.learn_thrower:
                    rp = self.flight.release_point(bp, bv, p.release_height)
                    if rp is not None and 1.0 < float(np.linalg.norm(rp[0][:2] - pose[:2])) < 12.0:
                        self.hand[:2] = 0.5 * self.hand[:2] + 0.5 * rp[0][:2]
                self.log(f"toss {self.tosses}: tracking, ball {np.linalg.norm(bv):.1f} m/s, "
                         f"{np.linalg.norm(pose[:2] - bp[:2]):.1f} m out")

        if self.state == "flight":
            if track is not None and track.ready and t >= self.t_plan and not self.frozen:
                self.t_plan = t + p.replan_period
                est_p, est_v, _ = track.estimate(t)
                plan = self.catch_plan(est_p, est_v, self.v_c)
                if plan is not None and p.catch_mode == "moving":
                    tgt, _, (xy, v, t_go) = plan
                    cr = track.crossing(t, xy[2])
                    confident = cr is not None and math.sqrt(max(np.linalg.eigvalsh(cr[3]))) < p.moving_commit_sigma
                    dvec = tgt[:2] - pose[:2]
                    nd = float(np.linalg.norm(dvec))
                    u = dvec / nd if nd > 1e-6 else np.array([1.0, 0.0])
                    v_top = self.cfg.top_speed(u, pose[2], p.kid)
                    a_plan = self.cfg.plan_accel(p.kid)
                    ch = choose_intercept(pose[:2], rv.v_cmd.copy(), tgt[:2], t_go,
                                          a=self.cfg.brake_margin * a_plan, v_max=p.plan_margin * v_top,
                                          zone=self.zone(), coast=p.coast_moving,
                                          a_ball=min(p.brake_with_ball, a_plan), moving=confident)
                    self.v_end = ch.v_f
                    self.v_c = ch.v_f * ch.direction
                    self.gated = self.gated or ch.gated
                    plan = self.catch_plan(est_p, est_v, self.v_c)
                if plan is not None:
                    tgt, tilt, (xy, v, t_go) = plan
                    if self.predicted is None:
                        self.log(f"  predicted landing {np.linalg.norm(xy[:2] - self.home[:2]):.2f} m from home, "
                                 f"{t_go:.2f} s to go -> tilt {math.degrees(tilt):.0f} deg")
                    self.predicted = xy
                    self.target = tgt
                    self.t_c = t + t_go
                    self.tilt_target = tilt
                    if t_go < p.tilt_freeze:
                        self.frozen = True
                        self.moving_plan = self.v_end > 0.3
                        cr = track.crossing(t, self.cfg.rim_center(tilt)[1])
                        if cr is not None:
                            self.land_sigma = float(math.sqrt(max(np.linalg.eigvalsh(cr[3]))))
            arrived = self.t_c is not None and t > self.t_c + 0.3
            lost = self.predicted is None and t - self.t_track_seen > 1.0
            if rv.ball_held or arrived or lost or t - self.t_flight > 4.0:
                if lost and not rv.ball_held:
                    self.log("  track lost before a plan")
                self._set("settle", t)

        if self.state == "settle":
            # the ball is out of the cameras' view in the funnel: the catch beam decides, once it has held for 0.3 s
            # (a ball bouncing in the pocket blinks it), or after 1.5 s without it (one still rolling in after the
            # dump), with the base at rest (catch_game waited for both on the truth)
            sp = float(np.hypot(rv.vel_body[0], rv.vel_body[1]))
            self.t_beam = (self.t_beam or t) if rv.ball_held else None
            sure = rv.ball_held and t - self.t_beam >= 0.3
            if t - self.t_state > 0.5 and ((sp < 0.05 and (sure or t - self.t_state > 0.8)) or t - self.t_state > 3.0):
                self._settle_done(t, rv, pose)

        if self.state == "aim":
            aimed = abs(wrap(self.target[2] - pose[2])) < math.radians(0.5) and abs(rv.vel_body[2]) < 0.05 and \
                abs(rv.tilt - self.tilt_target) < math.radians(1.0) and abs(rv.tilt_cmd - self.tilt_target) < 1e-3
            if aimed and not self.final_plan:
                self._plan_return(pose, final=True, tilt_meas=rv.tilt)
                self.final_plan = True
            tgt = self.fly_rpm
            ready = rv.fly_ready and tgt > 0 and all(abs(w - tgt) < 0.02 * tgt for w in rv.fly_rpm)
            self.t_ready = (self.t_ready or t) if ready else None
            still = float(np.hypot(rv.vel_body[0], rv.vel_body[1])) < 0.03
            if (aimed and self.final_plan and ready and t - self.t_ready >= 0.1 and still) or t - self.t_state > 3.0:
                self.fire_seq = (self.fire_seq + 1) & 0xFFFF
                self.last["return_speed"], self.last["return_elev_deg"] = self.v_needed, math.degrees(self.elev)
                self._set("launch", t)
                self.log(f"  fire: {self.v_needed:.2f} m/s at {math.degrees(self.elev):.1f} deg, "
                         f"{self.fly_rpm:.0f} rpm")

        if self.state == "launch" and t - self.t_state > 0.6:
            self._set("return", t)

        if self.state == "return":
            self.fly_rpm, self.tilt_target = 0.0, 0.0
            self.target = self.home.copy()
            self.target[2] = self._face(self.home, self.hand)
            if t - self.t_state > 1.6:
                self._set("wait", t)

        # -- the command ------------------------------------------------------------------------------------------
        cmd.target = tuple(float(c) for c in self.target)
        cmd.tilt = float(self.tilt_target)
        cmd.fly_rpm = float(self.fly_rpm)
        cmd.fire_seq = self.fire_seq
        if self.state in ("flight", "settle"):
            cmd.v_end = float(self.v_end) if (self.state == "flight" and self.predicted is not None) else 0.0
            if self.frozen and self.t_c is not None:
                cmd.t_brake = self.t_c
                moving = self.moving_plan or float(np.linalg.norm(self.v_c)) > 0.05
                cmd.coast_s = p.coast_moving if moving else p.coast_after_catch
                cmd.a_brake = p.brake_with_ball
                cmd.t_tilt = self.t_c + p.dump_delay
                cmd.tilt_after = 0.0
        return cmd
