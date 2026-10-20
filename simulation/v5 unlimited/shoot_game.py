"""Hopper mode: the funnel (with the magazine tube) loaded with balls, the robot drives around and
shoots at targets.

Targets are points in a gym (floor bins at z = 0.15, or marks on the netting at
chest height). For each target the robot:
  - "stop": drives to a firing spot (or stays put), stops, faces the target, tilts
    the module to the launch elevation, sets the flywheels from the launcher
    calibration and fires once the base is still;
  - "move": keeps driving its route (an omni base can translate one way and face
    another) and fires on the move. The launch is solved in the world frame: the
    ball leaves with the launcher's exit velocity plus the base's velocity, so the
    heading and exit speed are iterated until the predicted (drag) flight passes
    through the target. The base's velocity is predicted to the exit moment.
Scored by the miss distance in the target's horizontal plane.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from ball_flight import BallFlight
from omni_control import wrap_angle
from params import SimParams
from robot import Robot
from robot_model import ball_names


@dataclass
class ShootConfig:
    mode: str = "stop"                  # "stop" or "move"
    elevation_deg: float = 40.0         # launch elevation (module tilt sets it: 30 deg launch line minus tilt)
    route_speed: float = 1.5            # m/s along the route in "move" mode
    route: tuple = ((-2.5, -2.0), (2.5, -2.0), (2.5, 2.0), (-2.5, 2.0))   # waypoints (loop), "move" mode
    targets: tuple = ((4.0, 3.0, 0.15), (-4.0, 3.2, 0.15), (0.0, -3.8, 1.2), (4.6, -1.0, 1.2), (-4.6, 0.0, 0.15),
                      (1.0, 3.8, 1.2))
    aim_tol_deg: float = 0.7
    fire_timeout: float = 4.0
    stop_spot: tuple | None = None      # "stop": drive here first; None = shoot from wherever the robot is
    settle_time: float = 0.15           # s the next ball must sit in the pocket before the base may turn
    learn_rate: float = 0.25            # range-bias learning from logged hits (spin lift makes shots fly long)
    brake_while_turning: bool = True    # flywheels braked until the heading is within 5 deg of the shot (the pocket
                                        # is open toward the nip: a ball shaken forward by a hard turn got fired)
    yaw_accel: float | None = None      # cap the base's yaw acceleration in hopper mode (None = the profile's)


@dataclass
class Shot:
    target: tuple
    miss: float = float("nan")          # m, horizontal miss at the target height
    range_err: float = float("nan")     # m along the shot, + = long
    t_fire: float = float("nan")
    base_speed: float = 0.0
    exit_speed: float = float("nan")
    distance: float = float("nan")
    origin: np.ndarray = field(default_factory=lambda: np.zeros(2))
    cross_err: float = float("nan")     # m, + = left of the line of fire


class ShootGame:
    def __init__(self, model, data, p: SimParams, cfg: ShootConfig = ShootConfig(), verbose: bool = True):
        self.m, self.d, self.p, self.cfg = model, data, p, cfg
        self.r = Robot(model, data, p)
        b = p.ball_spec
        self.flight = BallFlight(b.mass, b.radius, b.drag_cd, p.air_density, p.air_viscosity)
        self.verbose = verbose
        self.balls = [model.body(n).id for n in ball_names(p)]
        self.bv = [model.joint(n).dofadr[0] for n in ball_names(p)]
        self.shots: list[Shot] = []
        self.k = 0                       # target index
        self.state, self.t_state = "aim", 0.0
        self.wp = 0
        self.flying = []                 # (ball index, shot) in the air
        self.launched = set()
        g = self.r.g
        self.exit_m = (g.N[0] + 0.06 * g.u[0], g.N[1] + 0.06 * g.u[1])
        self.r.fly_target = 0.0
        self._sol_k = -1
        self.t_held = None
        self.yaw_hold = None
        self.range_bias = 0.0            # learned: fraction of the distance the shots fly long
        if cfg.yaw_accel is not None:
            self.r.ctl.max_yaw_accel = cfg.yaw_accel

    def _log(self, msg):
        if self.verbose:
            print(f"[{self.d.time:6.2f}s] {msg}")

    # -- launch solution -----------------------------------------------------------
    def _exit_world(self, pose, tilt):
        x, z = self.r.g.module_to_robot(self.exit_m, tilt)
        return np.array([pose[0] + x * math.cos(pose[2]), pose[1] + x * math.sin(pose[2]), z])

    def solve(self, pose, v_base, target, w_base: float = 0.0):
        """(heading, exit speed relative to the launcher, tilt) whose world flight passes through target.
        v_base, w_base: the base's velocity and yaw rate at the exit moment (the exit point moves at
        v_base + w x r_exit, and the ball keeps that)."""
        cal, el = self.r.cal, math.radians(self.cfg.elevation_deg)
        tgt = np.asarray(target, float)
        aim = tgt.copy()
        tilt = cal.tilt_for(6.0, el)
        psi, s = math.atan2(tgt[1] - pose[1], tgt[0] - pose[0]), 6.0
        for _ in range(8):
            psi = math.atan2(aim[1] - pose[1], aim[0] - pose[0])
            ex = self._exit_world((*pose[:2], psi), tilt)
            u = np.array([math.cos(psi), math.sin(psi)])
            el_true = cal.elevation_at(s, tilt)
            s = self.flight.solve_launch_speed(ex, u, el_true, aim)
            tilt = float(np.clip(cal.tilt_for(s, el), *self.p.tilt_limits))
            # the world flight: launcher velocity + the base's
            v = np.array([s * math.cos(el_true) * u[0], s * math.cos(el_true) * u[1], s * math.sin(el_true)])
            rx, ry = ex[0] - pose[0], ex[1] - pose[1]
            v[:2] += np.asarray(v_base) + w_base * np.array([-ry, rx])
            hit = self.flight.crossing(ex, v, tgt[2])
            if hit is None:
                break
            miss = hit[0][:2] - tgt[:2]
            if np.linalg.norm(miss) < 0.005:
                break
            aim[:2] -= miss
        return psi, s, tilt

    # -- loop -------------------------------------------------------------------------
    def done(self) -> bool:
        return self.k >= len(self.cfg.targets) and not self.flying

    def step(self):
        self._behave()
        self.r.pre_step()
        mujoco.mj_step(self.m, self.d)
        self.r.post_step()
        self._score()

    def _route_velocity(self, pose):
        wps = self.cfg.route
        tgt = np.array(wps[self.wp % len(wps)])
        e = tgt - pose[:2]
        if np.linalg.norm(e) < 0.3:
            self.wp += 1
            tgt = np.array(wps[self.wp % len(wps)])
            e = tgt - pose[:2]
        return self.cfg.route_speed * e / max(1e-6, np.linalg.norm(e))

    def _behave(self):
        r, d, cfg, t = self.r, self.d, self.cfg, self.d.time
        pose = r.pose()
        v_base = d.sensor("base_linvel").data[:2].copy()
        if self.k >= len(cfg.targets):
            r.fly_target = 0.0
            r.tilt_target = 0.0
            if cfg.mode == "move":
                r.drive_velocity((0.0, 0.0), 0.0, pose[2])
            else:
                r.drive_to(pose, pose)
            return
        target = cfg.targets[self.k]
        # predict the base 0.1 s ahead (gate command to ball exit) when moving
        v_plan = r.ctl.v_cmd.copy() if cfg.mode == "move" else np.zeros(2)
        if t >= getattr(self, "_t_sol", -1.0) or self._sol_k != self.k:     # re-solve at 50 Hz
            pose_fire = pose.copy()
            pose_fire[:2] += v_plan * 0.10
            # aim short by the learned fractional range bias
            aim = np.array(target, float)
            aim[:2] = pose_fire[:2] + (aim[:2] - pose_fire[:2]) * (1.0 - self.range_bias)
            w_plan = r.ctl.w_cmd if cfg.mode == "move" else 0.0
            self._sol, self._t_sol, self._sol_k = self.solve(pose_fire, v_plan, aim, w_plan), t + 0.02, self.k
        psi, s, tilt = self._sol
        r.tilt_target = tilt
        r.fly_target = r.cal.speed_for(s, tilt)
        if cfg.brake_while_turning and self.state == "aim" and abs(wrap_angle(psi - pose[2])) > math.radians(5.0):
            r.fly_target = 0.0             # braked (AM32 brake-on-stop): a ball jostled forward can't be fired
        # the pocket is open toward the running flywheels: a freshly dropped ball that bounces forward while the
        # base turns hard gets fired unaimed (one ball lost per turn in the first runs). Hold the heading until
        # the next ball has sat in the pocket for settle_time.
        held = r.ball_held()
        self.t_held = (self.t_held if self.t_held is not None else t) if held else None
        settled = held and t - self.t_held >= cfg.settle_time
        yaw_cmd = psi if (settled or self.state == "push") else pose[2]
        if cfg.mode == "move":
            v = self._route_velocity(pose)
            w = float(np.clip(12.0 * wrap_angle(yaw_cmd - pose[2]), -12, 12)) if settled else 0.0
            r.drive_velocity(v, w, pose[2])
        else:
            spot = cfg.stop_spot if cfg.stop_spot is not None else None
            xy = np.array(spot) if spot is not None else (self.hold_xy if hasattr(self, "hold_xy") else pose[:2].copy())
            self.hold_xy = xy
            if not settled and self.yaw_hold is None:
                self.yaw_hold = pose[2]
            if settled:
                self.yaw_hold = None
            r.drive_to(pose, (*xy, psi if settled else self.yaw_hold))
        if self.state == "aim":
            aimed = abs(wrap_angle(psi - pose[2])) < math.radians(cfg.aim_tol_deg) and r.tilt_settled()
            still = cfg.mode == "move" or (np.linalg.norm(v_base) < 0.03 and abs(d.sensor("base_angvel").data[2]) < 0.05)
            if (aimed and still and r.flywheels_ready() and r.ball_held()) or t - self.t_state > cfg.fire_timeout:
                if not r.ball_held():
                    self._log(f"target {self.k}: no ball in the pocket")
                r.gate_open = True
                dist = float(np.hypot(target[0] - pose[0], target[1] - pose[1]))
                self.shots.append(Shot(target, t_fire=t, base_speed=float(np.linalg.norm(v_base)), distance=dist,
                                       origin=pose[:2].copy()))
                self.state, self.t_state = "push", t
                self._log(f"fire at target {self.k} ({dist:.1f} m, {s:.1f} m/s, base {np.linalg.norm(v_base):.1f} m/s)")
        elif self.state == "push":
            if t - self.t_state > 0.05 + self.p.kick_stroke / self.p.kick_speed:
                r.gate_open = False
                self.state, self.t_state = "reload", t
        elif self.state == "reload":
            if t - self.t_state > 0.15:
                self.k += 1
                self.state, self.t_state = "aim", t

    def _score(self):
        d = self.d
        ex = self.r.launch_exit_world()
        for i, bid in enumerate(self.balls):
            if i in self.launched:
                continue
            v = d.qvel[self.bv[i]:self.bv[i] + 3]
            if np.linalg.norm(d.xpos[bid] - ex) > 0.3 and np.linalg.norm(v) > 2.0 and self.shots:
                self.launched.add(i)
                shot = next((s for s in reversed(self.shots) if np.isnan(s.exit_speed)), None)
                if shot is not None:
                    shot.exit_speed = float(np.linalg.norm(v))
                    self.flying.append((i, shot, d.xpos[bid].copy()))
        still = []
        for i, shot, prev in self.flying:
            pos = d.xpos[self.balls[i]].copy()
            z = shot.target[2]
            if d.qvel[self.bv[i] + 2] < 0 and prev[2] >= z > pos[2]:
                f = (prev[2] - z) / (prev[2] - pos[2])
                xy = prev[:2] + f * (pos[:2] - prev[:2])
                e = xy - np.array(shot.target[:2])
                shot.miss = float(np.linalg.norm(e))
                launch = np.array(shot.target[:2]) - shot.origin
                u = launch / max(1e-6, np.linalg.norm(launch))
                shot.range_err = float(e @ u)
                shot.cross_err = float(e @ np.array([-u[1], u[0]]))
                self.range_bias += self.cfg.learn_rate * shot.range_err / max(0.5, shot.distance)
                self._log(f"  target at {shot.distance:.1f} m: missed by {100 * shot.miss:.0f} cm "
                          f"({100 * shot.range_err:+.0f} long, {100 * shot.cross_err:+.0f} left)")
            elif pos[2] < 0.05 or self.d.time - shot.t_fire > 4.0:
                pass
            else:
                still.append((i, shot, pos))
        self.flying = still


def load_tube(m, d, r, p, n):
    """Drop n balls into the magazine tube one at a time, settle."""
    names = ball_names(p)
    fc = r.funnel_center_world()
    z0 = fc[2] + (r.g.throat_z + p.hopper_tube_height - r.g.rim_z if p.hopper == "tube" else 0.0) + 0.06
    for k in range(n):
        adr, vadr = m.joint(names[k]).qposadr[0], m.joint(names[k]).dofadr[0]
        d.qpos[adr:adr + 7] = [fc[0], fc[1], z0, 1, 0, 0, 0]
        d.qvel[vadr:vadr + 6] = [0, 0, -0.5, 0, 0, 0]
        mujoco.mj_forward(m, d)
        for _ in range(int(0.35 / m.opt.timestep)):
            r.step()
    for _ in range(int(0.5 / m.opt.timestep)):
        r.step()


def run(mode="stop", n=6, verbose=True, detent=None, **ckw):
    """detent: pocket_detent height (m); None = the SimParams default."""
    kw = {} if detent is None else dict(pocket_detent=detent)
    p = SimParams(hopper="tube", n_balls=max(0, n - 1), home_xy=(0.0, 0.0), area_size=(10.0, 8.0),
                  thrower_xy=(-6.0, 0.0), **kw)
    m = mujoco.MjModel.from_xml_string(__import__("robot_model").build_xml(p))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = ShootConfig(mode=mode, **ckw)
    game = ShootGame(m, d, p, cfg, verbose=verbose)
    load_tube(m, d, game.r, p, n)
    game.t_state = d.time
    while not game.done() and d.time < 60:
        game.step()
    return game


if __name__ == "__main__":
    import sys
    g = run(sys.argv[1] if len(sys.argv) > 1 else "stop")
    misses = [s.miss for s in g.shots]
    print(f"{len(g.shots)} shots, misses (cm): {[round(100 * x) if np.isfinite(x) else None for x in misses]}, "
          f"median {100 * np.nanmedian(misses):.0f} cm")
