"""Hello-world MuJoCo simulation of the v1 play-catch kiwi robot.

  python hello_world.py                 catch demo in the interactive viewer
  python hello_world.py --drive         scripted omni-drive demo (square, spin, circle)
  python hello_world.py --headless -n 30          run 30 tosses without graphics, print catch rate
  python hello_world.py --record catch.mp4 -s 40  render a video with the side camera

Catch demo loop: a person lobs the ball underhand; after a 0.25 s perception
delay the robot predicts where the ball crosses its funnel-rim height (ground-truth
ball state plus a drag model), drives there and stops; the chamber break-beam
scores the catch; the robot turns to face the thrower and the launcher returns the
ball (launcher modeled by its output: exit speed and angle with scatter).
"""

from __future__ import annotations

import argparse
import math
import time

import mujoco
import numpy as np

from ball_flight import BallFlight
from kiwi_control import BaseController, KiwiKinematics, wrap_angle, yaw_from_quat
from kiwi_model import SimParams, build_model, describe


class Robot:
    """Thin wrapper that reads the robot's sensors and writes wheel commands."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, p: SimParams):
        self.m, self.d, self.p = model, data, p
        self.kin = KiwiKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius)
        self.ctl = BaseController(self.kin, max_speed=p.base_max_speed, max_accel=p.base_max_accel,
                                  max_wheel_speed=p.motor_max_speed)
        self.wheel_act = [model.actuator(f"wheel{i}").id for i in (1, 2, 3)]
        self.module = model.body("catch_module").id
        self.exit_site = model.site("launch_exit").id

    def pose(self):
        pos = self.d.sensor("base_pos").data
        return np.array([pos[0], pos[1], yaw_from_quat(self.d.sensor("base_quat").data)])

    def ball_detected_in_chamber(self) -> bool:
        r = self.d.sensor("catch_beam").data[0]
        return 0 <= r < 2 * self.p.funnel_throat_radius

    def command(self, wheels):
        self.d.ctrl[self.wheel_act] = wheels

    def launch_exit_world(self) -> np.ndarray:
        return self.d.site_xpos[self.exit_site].copy()


class World:
    """The parts of the simulation that are not the robot: the ball and the thrower."""

    def __init__(self, model, data):
        self.m, self.d = model, data
        j = model.joint("ball")
        self.qadr, self.vadr = j.qposadr[0], j.dofadr[0]
        self.marker = model.body_mocapid[model.body("landing_marker").id]

    def place_ball(self, pos, vel=(0, 0, 0)):
        self.d.qpos[self.qadr:self.qadr + 7] = [*pos, 1, 0, 0, 0]
        self.d.qvel[self.vadr:self.vadr + 6] = [*vel, 0, 0, 0]

    def ball_state(self):
        return self.d.sensor("ball_pos").data.copy(), self.d.sensor("ball_vel").data.copy()

    def show_marker(self, xy):
        self.d.mocap_pos[self.marker] = [xy[0], xy[1], 0.002]


class CatchGame:
    perception_delay = 0.25     # s after release before the first landing estimate
    replan_period = 0.05        # landing estimate refresh (20 Hz)
    catch_zone_radius = 0.37    # tosses aim within this distance of home (doc: stop-and-catch reach)
    apex_range = (1.0, 2.0)     # m above the release point
    release_height = 1.0
    return_height = 1.2         # chest height for return throws
    launch_scatter = (0.02, math.radians(0.7))   # speed fraction, angle (1 sigma)

    def __init__(self, model, data, p: SimParams, seed=0, verbose=True):
        self.p, self.verbose = p, verbose
        self.robot, self.world = Robot(model, data, p), World(model, data)
        self.d = data
        self.dt = model.opt.timestep
        self.flight = BallFlight(p.ball_mass, p.ball_radius, p.ball_drag_cd, p.air_density, p.air_viscosity)
        self.rng = np.random.default_rng(seed)
        self.home = np.array([*p.home_xy, p.home_yaw])
        self.hand = np.array([p.thrower_xy[0] + 0.3, p.thrower_xy[1], self.release_height])
        self.target = self.home.copy()
        self.state, self.t_next = "wait", 1.0
        self.results = []

    # -- helpers --------------------------------------------------------------
    def _log(self, msg):
        if self.verbose:
            print(f"[{self.d.time:7.2f}s] {msg}")

    def _face(self, xy, toward):
        return math.atan2(toward[1] - xy[1], toward[0] - xy[0])

    # -- state machine -------------------------------------------------------
    def step(self):
        t, pose = self.d.time, self.robot.pose()
        ball_p, ball_v = self.world.ball_state()

        if self.state == "wait":
            self.target = np.array([*self.home[:2], self._face(self.home, self.hand)])
            if t >= self.t_next:
                self._toss()

        elif self.state == "flight":
            if t - self.t_release >= self.perception_delay and t >= self.t_plan:
                self.t_plan = t + self.replan_period
                hit = self.flight.crossing(ball_p, ball_v, self.p.funnel_rim_height)
                if hit is not None:
                    xy = hit[0][:2]
                    if self.predicted is None:
                        self._log(f"landing predicted {np.linalg.norm(xy - self.home[:2]):.2f} m from home, "
                                  f"{hit[1]:.2f} s to go")
                    self.predicted = xy
                    self.target = np.array([*xy, self._face(xy, self.hand)])
                    self.world.show_marker(xy)
            if ball_p[2] < self.p.funnel_rim_height and ball_v[2] < 0 and self.cross_err is None:
                self.cross_err = float(np.hypot(*(ball_p[:2] - pose[:2])))
            # in the chamber, on the floor, or stuck somewhere (e.g. on the deck)
            if self.robot.ball_detected_in_chamber() or ball_p[2] < 0.2 or t - self.t_release > 4.0:
                self.state, self.t_settle = "settle", t

        elif self.state == "settle":
            if t - self.t_settle > 0.6:
                caught = self.robot.ball_detected_in_chamber()
                err = self.cross_err if self.cross_err is not None else float("nan")
                self.results.append(dict(caught=caught, error=err, zone=self.zone_dist))
                n, k = len(self.results), sum(r["caught"] for r in self.results)
                self._log(f"toss {n}: {'CAUGHT' if caught else 'missed'} (funnel offset at rim "
                          f"{err * 100:.1f} cm) - {k}/{n} caught")
                if caught:
                    self.state, self.t_aim = "aim", t
                else:
                    self.state, self.t_next = "wait", t + 1.0

        elif self.state == "aim":
            self.target = np.array([*self.target[:2], self._face(self.target, self.hand)])
            aimed = abs(wrap_angle(self.target[2] - pose[2])) < math.radians(0.5) and \
                abs(self.d.sensor("base_angvel").data[2]) < 0.05
            if aimed or t - self.t_aim > 3.0:
                self._launch(pose)

        elif self.state == "return":
            self.target = np.array([*self.home[:2], self._face(self.home, self.hand)])
            if t >= self.t_next:
                self.state, self.t_next = "wait", t + 0.5

        self.robot.command(self.robot.ctl.goto_step(self.dt, pose, self.target))

    def _toss(self):
        a = self.rng.uniform(0, 2 * math.pi)
        r = self.catch_zone_radius * math.sqrt(self.rng.uniform())
        aim = self.home[:2] + r * np.array([math.cos(a), math.sin(a)])
        apex = self.rng.uniform(*self.apex_range)
        v0 = self.flight.solve_toss(self.hand, [*aim, self.p.funnel_rim_height], apex)
        self.world.place_ball(self.hand, v0)
        self.zone_dist = r
        self.state, self.t_release, self.t_plan = "flight", self.d.time, 0.0
        self.predicted, self.cross_err = None, None
        self._log(f"toss: aim {r:.2f} m from home, apex {apex:.2f} m, release speed {np.linalg.norm(v0):.2f} m/s")

    def _launch(self, pose):
        exit_pos = self.robot.launch_exit_world()
        heading = np.array([math.cos(pose[2]), math.sin(pose[2])])
        elev = math.radians(self.p.launch_angle_deg)
        target = [*self.hand[:2], self.return_height]
        speed = self.flight.solve_launch_speed(exit_pos, heading, elev, target)
        ds, da = self.launch_scatter
        speed *= 1 + self.rng.normal(0, ds)
        elev += self.rng.normal(0, da)
        yaw = pose[2] + self.rng.normal(0, da)
        v = speed * np.array([math.cos(elev) * math.cos(yaw), math.cos(elev) * math.sin(yaw), math.sin(elev)])
        v[:2] += self.d.sensor("base_linvel").data[:2]
        self.world.place_ball(exit_pos, v)
        self._log(f"launch back to thrower at {speed:.2f} m/s, {math.degrees(elev):.1f} deg")
        self.state, self.t_next = "return", self.d.time + 1.5

    def summary(self) -> str:
        n = len(self.results)
        if not n:
            return "no tosses completed"
        k = sum(r["caught"] for r in self.results)
        errs = np.array([r["error"] for r in self.results])
        errs = errs[np.isfinite(errs)]
        return (f"{k}/{n} caught ({100 * k / n:.0f}%), funnel offset at rim: median "
                f"{100 * np.median(errs):.1f} cm, max {100 * np.max(errs):.1f} cm")


class DriveDemo:
    """Scripted moves that show off the holonomic base."""

    def __init__(self, model, data, p: SimParams):
        self.robot, self.d, self.dt = Robot(model, data, p), data, model.opt.timestep
        self.steps = [
            ("goto", (0.8, 0.0, 0.0), "forward"),
            ("goto", (0.8, 0.8, 0.0), "strafe left"),
            ("goto", (0.0, 0.0, math.pi), "diagonal back while turning 180 deg"),
            ("spin", 2 * math.pi, "spin in place one turn"),
            ("circle", (0.5, 0.9), "circle r=0.5 m at 0.9 m/s, heading fixed"),
            ("goto", (0.0, 0.0, 0.0), "back to start"),
        ]
        self.i = -1
        self._advance(0.0, self.robot.pose())

    def _advance(self, t, pose):
        """Hold the current pose for a moment, then start the next move."""
        self.i = (self.i + 1) % len(self.steps)
        kind, arg, label = self.steps[self.i]
        self.hold = tuple(pose)
        self.t0 = t + 0.6
        self.turned, self.last_yaw = 0.0, pose[2]
        self.center = np.array([pose[0], pose[1] + (arg[0] if kind == "circle" else 0.0)])
        print(f"[{self.t0:6.2f}s] {label}")

    def step(self):
        t, pose = self.d.time, self.robot.pose()
        ctl = self.robot.ctl
        kind, arg, _ = self.steps[self.i]
        done = False
        if t < self.t0:
            wheels = ctl.goto_step(self.dt, pose, self.hold)
        elif kind == "goto":
            wheels = ctl.goto_step(self.dt, pose, arg)
            done = np.hypot(arg[0] - pose[0], arg[1] - pose[1]) < 0.005 and \
                abs(wrap_angle(arg[2] - pose[2])) < 0.01
        elif kind == "spin":
            self.turned += wrap_angle(pose[2] - self.last_yaw)
            self.last_yaw = pose[2]
            remaining = arg - self.turned
            w = math.copysign(min(4.0, math.sqrt(2 * 10 * abs(remaining)), 8 * abs(remaining)), remaining)
            wheels = ctl.velocity_step(self.dt, (0, 0), w, pose[2])
            done = abs(remaining) < 0.01
        else:  # circle around a center beside the start point, heading held fixed
            r, v = arg
            w = v / r
            ang = -math.pi / 2 + w * (t - self.t0)
            ref = self.center + r * np.array([math.cos(ang), math.sin(ang)])
            v_ff = v * np.array([-math.sin(ang), math.cos(ang)])
            wheels = ctl.goto_step(self.dt, pose, (*ref, self.hold[2]), v_ff=v_ff)
            done = w * (t - self.t0) >= 2 * math.pi
        if done:
            self._advance(t, pose)
        self.robot.command(wheels)


def make_sim(drive_demo: bool, seed: int = 0, verbose=True):
    p = SimParams()
    if drive_demo:
        p.home_xy = (0.0, 0.0)
    model = build_model(p)
    data = mujoco.MjData(model)
    if drive_demo:
        # start at the center of the area facing +x
        adr = model.joint("kiwi").qposadr[0]
        data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    behavior = DriveDemo(model, data, p) if drive_demo else CatchGame(model, data, p, seed=seed, verbose=verbose)
    print(describe(model, p))
    return p, model, data, behavior


def run_viewer(model, data, behavior):
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        viewer.cam.lookat[:] = (-0.4, 0.0, 0.7)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 6.5, 90, -16
        frame = 1 / 60
        while viewer.is_running():
            start = time.perf_counter()
            stop = data.time + frame
            while data.time < stop:
                behavior.step()
                mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, frame - (time.perf_counter() - start)))


def run_headless(model, data, behavior, tosses: int):
    t0 = time.perf_counter()
    while len(behavior.results) < tosses:
        behavior.step()
        mujoco.mj_step(model, data)
    wall = time.perf_counter() - t0
    print(behavior.summary())
    print(f"simulated {data.time:.1f} s in {wall:.1f} s ({data.time / wall:.0f}x real time)")


def record(model, data, behavior, path: str, seconds: float, fps: int = 30):
    import imageio
    renderer = mujoco.Renderer(model, 720, 1280)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    frames, next_frame = [], 0.0
    while data.time < seconds:
        behavior.step()
        mujoco.mj_step(model, data)
        if data.time >= next_frame:
            renderer.update_scene(data, camera="side", scene_option=opt)
            frames.append(renderer.render())
            next_frame += 1 / fps
    imageio.mimwrite(path, frames, fps=fps, quality=8)
    print(f"wrote {path} ({len(frames)} frames)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drive", action="store_true", help="omni-drive demo instead of the catch demo")
    ap.add_argument("--headless", action="store_true", help="no graphics; run tosses and print stats")
    ap.add_argument("-n", "--tosses", type=int, default=20)
    ap.add_argument("--record", metavar="FILE.mp4", help="render a video instead of opening the viewer")
    ap.add_argument("-s", "--seconds", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    p, model, data, behavior = make_sim(args.drive, seed=args.seed)
    if args.record:
        record(model, data, behavior, args.record, args.seconds)
    elif args.headless:
        run_headless(model, data, behavior, args.tosses)
    else:
        run_viewer(model, data, behavior)


if __name__ == "__main__":
    main()
