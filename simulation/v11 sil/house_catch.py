"""The catch game in the house (design.md 7, 13: one sim world for every console mode). PC only, no ROS.

In the 'robot' step the console's Catch used to need its own world (catch_world.CatchWorld: an open field). Here the
house's robot gets the same pieces instead: the stereo pair's frames (catch_world.CameraSim, the same sensor model)
and a thrower (catch_world.Thrower, the same throws), who steps up when the robot's mode becomes CATCH: `dist` m
straight ahead where the house leaves that much room (the walls, furniture and people on the throw's line, at three
heights and three lanes), else along the clearest of 12 headings (the robot turns to face them), at least MIN_DIST
away, else the game is refused with the reason. They throw every wait_between s, the
robot's code catches (and throws back, with returns); a throw is scored as catch_world scores it. Leaving CATCH, the
thrower leaves too (the ball stays where it is).

What the robot is told is relative, the way a person detector and the map would tell it (the bridge publishes it on
/catch/setup; catch_behavior turns it into its own odom frame): the thrower is dist m away at a bearing from the
robot's heading, and how much room there is around it (the play area's half size).
"""

from __future__ import annotations

import math
from collections import deque

import mujoco
import numpy as np

from ball_flight import BallFlight
from catch_world import CameraSim, Thrower, WorldConfig


class HouseCatch:
    MIN_DIST = 2.5                  # m: a throw shorter than this isn't the game (and the cameras want the flight)

    def __init__(self, m, d, p, robot, seed: int = 0, dist: float = 6.5, apex_range=(0.75, 2.0),
                 returns: bool = True):
        self.m, self.d, self.p, self.robot = m, d, p, robot
        self.cfg = WorldConfig(dist=dist, apex_range=tuple(apex_range), seed=seed, return_throws=returns)
        self.cams = CameraSim(m, d, p, self.cfg.vision, np.random.default_rng(seed + 7))
        b = p.ball_spec
        self.flight = BallFlight(b.mass, b.radius, b.drag_cd, p.air_density, p.air_viscosity)
        self.z_aim = robot.g.rim_center_robot(0.0)[1]
        self.lat_rng = np.random.default_rng(seed + 11)
        j = m.joint("ball")
        self.bq, self.bv = j.qposadr[0], j.dofadr[0]
        self.base = m.body("base").id
        self.on = False
        self.thrower = None
        self.home = None
        self.n_start = 0
        self.frames: deque = deque()          # (t due, [SimFrame, SimFrame])
        self.t_frame = 0.0
        self.results: list = []
        self.cur = None
        self.phase, self.t_next = "wait", 0.0

    # -- where the thrower can stand ------------------------------------------------------------------------------
    def _ray(self, start, direction, max_dist: float) -> float:
        gid = np.array([-1], dtype=np.int32)
        h = mujoco.mj_ray(self.m, self.d, np.asarray(start, float), np.asarray(direction, float), None, 1, self.base,
                          gid)
        return max_dist if gid[0] < 0 or h < 0 else min(max_dist, float(h))

    def room_ahead(self, xy, yaw: float, max_dist: float = 12.0) -> float:
        """Free distance from xy along yaw: the throw's line at 0.3, 1.0 and 1.7 m, in three lanes 0.4 m apart (the
        ball's path and the thrower's body), from just ahead of the robot."""
        u = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        n = np.array([-u[1], u[0], 0.0])
        free = max_dist
        for z in (0.3, 1.0, 1.7):
            for lane in (-0.4, 0.0, 0.4):
                s = np.array([xy[0], xy[1], z]) + 0.3 * u + lane * n
                free = min(free, 0.3 + self._ray(s, u, max_dist))
        return free

    def room_around(self, xy, max_r: float = 2.0) -> float:
        """How far the robot can move from home before something is in the way (8 directions at bumper height)."""
        r = max_r
        for k in range(8):
            a = 2 * math.pi * k / 8
            u = np.array([math.cos(a), math.sin(a), 0.0])
            r = min(r, 0.25 + self._ray(np.array([xy[0], xy[1], 0.12]) + 0.25 * u, u, max_r))
        return r

    # -- the game on and off ------------------------------------------------------------------------------------------
    def start(self, pose, t: float) -> dict:
        """The mode became CATCH with the robot at pose (x, y, yaw, world): stand a thrower where there's room,
        straight ahead if the full distance is clear, else along the clearest of 12 headings (the robot turns to face
        them while it waits). -> the setup for /catch/setup: {ok, message, seq, dist (m), bearing (rad from the robot's
        heading, + = left), half (the play area's half size), returns}."""
        x, y, yaw = (float(c) for c in pose)
        rooms = [(self.room_ahead((x, y), yaw + b), b) for b in np.radians(np.arange(0, 360, 30))]
        free, bearing = rooms[0] if rooms[0][0] - 0.6 >= self.cfg.dist else max(rooms)
        bearing = float(math.remainder(bearing, 2 * math.pi))
        dist = min(self.cfg.dist, free - 0.6)               # the thrower stands clear of what's behind them
        self.n_start += 1
        if dist < self.MIN_DIST:
            self.on = False
            return dict(ok=False, seq=self.n_start, dist=round(dist, 2),
                        message=f"No room to play catch here: {free:.1f} m at most around the robot, the thrower "
                                f"needs {self.MIN_DIST + 0.6:.1f} m. Drive to a longer room or the hallway and try again")
        yaw += bearing
        u = np.array([math.cos(yaw), math.sin(yaw)])
        self.home = np.array([x, y])
        hand = np.array([*(self.home + dist * u), self.cfg.release_height])
        self.thrower = Thrower(self.p, self.cfg, self.flight, self.z_aim, home=self.home, hand=hand)
        self.on = True
        self.phase, self.t_next = "wait", t + 3.0             # time to turn and get ready
        self.cur = None
        half = self.room_around((x, y))
        where = "in front of the robot" if abs(bearing) < 0.1 else \
            f"from the robot, {abs(math.degrees(bearing)):.0f} deg to its {'left' if bearing > 0 else 'right'}"
        return dict(ok=True, seq=self.n_start, dist=round(dist, 2), bearing=round(bearing, 4), half=round(half, 2),
                    returns=self.cfg.return_throws,
                    message=f"A thrower stands {dist:.1f} m {where} (room around it {half:.1f} m)")

    def stop(self):
        self.on = False
        self.frames.clear()

    def pop_frames(self, t: float) -> list:
        out = []
        while self.frames and self.frames[0][0] <= t:
            out.append(self.frames.popleft()[1])
        return out

    # -- every physics step -----------------------------------------------------------------------------------------
    def step(self, t: float):
        if not self.on:
            return
        c = self.cfg
        if t >= self.t_frame - 1e-9:                          # the cameras (catch_world.CatchWorld.step)
            self.t_frame = t + 1.0 / c.vision.fps
            pair = self.cams.frame_pair(t)
            lat = max(0.005, self.lat_rng.normal(c.vision.latency_mean - c.processing, c.vision.latency_jitter))
            self.frames.append((max(t + lat, self.frames[-1][0] if self.frames else 0.0), pair))
        self._game(t)

    def _ball(self):
        return self.d.sensor("ball_pos").data.copy(), self.d.sensor("ball_vel").data.copy()

    def _game(self, t: float):
        """catch_world.CatchWorld._game: throw, score the catch (held 0.5 s) or the miss, wait for the return."""
        r, c = self.robot, self.cfg
        bp, bv = self._ball()
        if self.phase == "wait":
            if t >= self.t_next:
                pos, v0, spin, aim, apex = self.thrower.toss()
                self.d.qpos[self.bq:self.bq + 7] = [*pos, 1, 0, 0, 0]
                self.d.qvel[self.bv:self.bv + 6] = [*v0, *spin]
                self.cur = dict(t_release=t, apex=apex, zone_dist=float(np.linalg.norm(aim - self.home)),
                                release_speed=float(np.linalg.norm(v0)), caught=False, pos_error=math.nan,
                                catch_speed=math.nan, flight_time=math.nan, launched=False, return_error=math.nan,
                                t_held=None, release=[float(x) for x in pos],
                                return_hand=[float(x) for x in self.thrower.hand[:2]])
                self.phase = "flight"
            return
        cur = self.cur
        if self.phase == "flight":
            fc = r.funnel_center_world()
            if math.isnan(cur["pos_error"]) and bv[2] < 0 and bp[2] < fc[2]:
                cur["pos_error"] = float(np.hypot(*(bp[:2] - fc[:2])))
                cur["catch_speed"] = float(np.linalg.norm(self.d.sensor("base_linvel").data[:2]))
                cur["flight_time"] = t - cur["t_release"]
                cur["t_in"] = t
            held = r.ball_held()
            cur["t_held"] = (cur["t_held"] or t) if held else None
            if held and t - cur["t_held"] >= 0.5:
                self._resolve(t, True)
            elif not held and bp[2] < 0.15 and t - cur.get("t_in", math.inf) > 0.3:
                self._resolve(t, False)
            elif t - cur["t_release"] > 5.0:
                self._resolve(t, r.ball_held())
        elif self.phase == "held":
            mod = self.d.xpos[r.module]
            if not cur["launched"] and np.linalg.norm(bp - mod) > 0.4 and not r.ball_held():
                cur["launched"] = True
            if cur["launched"] and bv[2] < 0 and bp[2] < c.return_height and math.isnan(cur["return_error"]):
                cur["return_error"] = float(np.hypot(*(bp[:2] - np.array(cur["return_hand"]))))
                self.phase, self.t_next = "wait", t + c.wait_between
            elif t - self.t_held_since > 8.0:
                self.phase, self.t_next = "wait", t + c.wait_between

    def _resolve(self, t: float, caught: bool):
        cur = self.cur
        cur["caught"], cur["t_result"] = bool(caught), t
        self.results.append(cur)
        if caught and self.cfg.return_throws:
            self.phase, self.t_held_since = "held", t
        else:
            self.phase, self.t_next = "wait", t + self.cfg.wait_between

