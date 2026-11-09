"""Goalie, ROS-free: the robot keeps a goal; a ball rolled at it is blocked by sliding sideways (the omni base's best
move), facing the shooter all the while. missions.py's GOALIE feeds it the ball's track and the robot's pose (both odom)
at 20 Hz and sends the twist it returns (body frame) straight to the MCU's caps.

The goal: the line through where the robot stood when the game started, across its heading, half_width each side. In
the goal's frame (u ahead toward the shooter, v to the left):
  rolling at it  (u falling faster than min_speed, still ahead): slide to where it will cross u = line_u (constant
                 velocity: a floor ball rolls straight; its slowing makes it arrive later, not elsewhere)
  otherwise      shadow it: between the ball and the goal's middle (shadow share of its v), back at the middle without one
The robot holds u = 0 and the start's heading (P control), the slide at up to max_speed and accel (the firmware's
profile caps both again). A ball that crosses the line inside the goal is a goal; one that comes within save_r of the
robot and goes back out (or stops) a save. Speeds and distances are estimates for a 15-25 cm play ball; VERIFY.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class GoalieParams:
    half_width: float = 0.6         # m each side of the goal's middle
    line_u: float = 0.25            # m: the robot's front (bumper) ahead of its centre: where it blocks
    min_speed: float = 0.25         # m/s toward the goal: a shot (slower: a ball being placed)
    shadow: float = 0.5             # share of the ball's v the robot follows without a shot
    max_speed: float = 1.2          # m/s sideways
    accel: float = 4.0              # m/s^2 the slide's command may change by
    k_v: float = 6.0                # 1/s: sideways position gain
    k_u: float = 2.0                # 1/s: back onto the line
    k_yaw: float = 3.0              # 1/s: back to the heading
    save_r: float = 0.45            # m: a ball this near the robot's centre was reached
    stale: float = 0.4              # s: a track older than this is no ball
    shot_lost: float = 1.0          # s: a shot ends for want of a ball only after this long (gaps in the track are not a
                                    #   new shot each: a slow computer or a sim under real time drops frames)
    shot_gap: float = 1.0           # s: a new shot counts only this long after the last ended (a saved ball's bounce)
    trust: float = 0.4              # s: the prediction followed in full only this close to the line; farther out it
                                    #   is blended toward the shadow (its error grows with the time left), 0: always.
                                    #   0.4 s: best of 0-0.9 s against noisy D435-like tracks offline (est)


@dataclass
class Goalie:
    home: tuple                     # (x, y, yaw) odom: the goal's middle and the way it faces
    p: GoalieParams = field(default_factory=GoalieParams)
    saves: int = 0
    goals: int = 0
    shots: int = 0
    state: str = "ready"            # ready | shot | (the last result's) save | goal
    events: list = field(default_factory=list)
    _cmd_v: float = 0.0
    _t: float | None = None
    _shot: dict | None = None       # the shot being played: its predicted v, closest approach
    _t_end: float = -math.inf       # when the last shot ended

    def to_goal(self, x: float, y: float) -> tuple:
        hx, hy, a = self.home
        dx, dy = x - hx, y - hy
        c, s = math.cos(a), math.sin(a)
        return c * dx + s * dy, -s * dx + c * dy

    def _vel_goal(self, vx: float, vy: float) -> tuple:
        c, s = math.cos(self.home[2]), math.sin(self.home[2])
        return c * vx + s * vy, -s * vx + c * vy

    def tick(self, now: float, pose: tuple | None, ball: dict | None) -> tuple:
        """(vx, vy, wz) body frame. pose: the robot (x, y, yaw) odom; ball: {x, y, vx, vy, t (age s)} odom or None."""
        p = self.p
        dt = 0.05 if self._t is None else max(1e-3, min(0.2, now - self._t))
        self._t = now
        if pose is None:
            self._cmd_v = 0.0
            return 0.0, 0.0, 0.0
        ru, rv = self.to_goal(pose[0], pose[1])
        target = 0.0
        if ball is not None and ball.get("age", 0.0) < p.stale:
            bu, bv = self.to_goal(ball["x"], ball["y"])
            du, dv = self._vel_goal(ball["vx"], ball["vy"])
            near = math.hypot(ball["x"] - pose[0], ball["y"] - pose[1]) < p.save_r
            if du < -p.min_speed and bu > p.line_u - 0.05:
                t_hit = (bu - p.line_u) / -du
                cross = bv + dv * t_hit
                if p.trust > 0.0:                  # far out, the lateral speed's noise x the time left: shade it
                    w = min(1.0, p.trust / max(t_hit, 1e-3))
                    shadow = p.shadow * bv
                    cross = shadow + w * (cross - shadow)
                target = max(-p.half_width, min(p.half_width, cross))
                if self._shot is None and now - self._t_end >= p.shot_gap:
                    self._shot = dict(cross=cross, closest=math.inf)
                    self.shots += 1
                    self.state = "shot"
                if self._shot is not None:
                    self._shot["cross"] = cross
            else:
                target = max(-p.half_width, min(p.half_width, p.shadow * bv))
            if self._shot is not None:
                self._shot["closest"] = min(self._shot["closest"], math.hypot(ball["x"] - pose[0], ball["y"] - pose[1]))
                if bu < 0.0 and abs(bv) < p.half_width and not near:      # behind the line, inside the posts
                    self._result("goal", now)
                elif bu < -0.3 or (du > 0.1 and self._shot["closest"] < p.save_r) or \
                        (abs(du) < 0.05 and self._shot["closest"] < p.save_r):
                    self._result("save" if self._shot["closest"] < p.save_r else "miss", now)
        elif self._shot is not None and (ball is None or ball.get("age", 0.0) >= p.shot_lost):   # out of sight mid-shot
            self._result("save" if self._shot["closest"] < p.save_r else "miss", now)
        want = max(-p.max_speed, min(p.max_speed, p.k_v * (target - rv)))
        step = p.accel * dt
        self._cmd_v += max(-step, min(step, want - self._cmd_v))
        cu = max(-0.4, min(0.4, -p.k_u * ru))
        e_yaw = math.remainder(self.home[2] - pose[2], 2 * math.pi)
        wz = max(-1.5, min(1.5, p.k_yaw * e_yaw))
        # goal frame -> body frame (the robot faces the goal's heading, give or take e_yaw)
        a = -e_yaw
        vx = math.cos(a) * cu + math.sin(a) * self._cmd_v
        vy = -math.sin(a) * cu + math.cos(a) * self._cmd_v
        return vx, vy, wz

    def _result(self, what: str, now: float):
        self._shot, self._t_end = None, now
        if what == "goal":
            self.goals += 1
        elif what == "save":
            self.saves += 1
        self.state = what
        self.events.append(what)

    def summary(self) -> dict:
        return dict(state=self.state, saves=self.saves, goals=self.goals, shots=self.shots)
