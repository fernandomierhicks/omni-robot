"""Which way the robot faces while Nav2 drives it (the console's "Driving" choice). No ROS here; heading_node runs it.

Nav2's controller (MPPI, Omni model) moves the base along the path with whatever heading suits it, and its footprint
is a circle, so the heading never matters for collisions. This changes only the turn rate:
  agnostic   Nav2's own turn rate (passed straight through: Nav2 as before)
  forward    the face turns toward the direction of travel
  spinning   it spins at spin_rate while it travels
Near the goal (path distance under `handback`) it hands the turn back to Nav2, which settles the goal's heading;
forward also hands it back while the robot is slow (its direction of travel means nothing then).

The firmware turns each CMD's body-frame velocity into a world-frame target with its yaw at every 1 kHz tick
(firmware.cpp DRIVE_VEL_BODY), so a body-frame command held while the robot turns sweeps the target round with it
(at 2 rad/s, 0.1 rad in one of Nav2's 50 ms periods). The shaper keeps Nav2's velocity in the odom frame and turns it
back into the body frame at its own rate (50 Hz) with the yaw predicted for the middle of the time the firmware will
hold it (`lead`, the path's latency to the firmware, plus half a period), so the robot keeps its course while it turns:
in the test's loop, spinning at 120 deg/s over 4 m, 41 cm off the line with Nav2's command as it is, 1 cm shaped.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

MODES = ("agnostic", "forward", "spinning")


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ramp(x: float, lo: float, hi: float) -> float:
    """0 at or below lo, 1 at or above hi, linear between."""
    return min(1.0, max(0.0, (x - lo) / (hi - lo)))


@dataclass
class HeadingConfig:
    mode: str = "agnostic"
    spin_rate: float = math.radians(120.0)   # rad/s, spinning (negative: clockwise)
    turn_max: float = math.radians(90.0)     # rad/s, forward's top turn rate (the console's Go To turn rate)
    yaw_accel: float = 3.2                   # rad/s^2, as the velocity smoother's
    gain: float = 3.0                        # 1/s, forward: turn rate per rad of heading error
    lead: float = 0.03                       # s from here to the firmware acting on it (est: mode manager + link)
    period: float = 0.02                     # s between our commands: each is aimed at the middle of its hold
    handback: tuple = (0.4, 0.9)             # m of path left: Nav2's turn rate at the first, ours from the second
    slow: tuple = (0.08, 0.2)                # m/s, forward: Nav2's turn rate at the first, ours from the second
    stale: float = 0.3                       # s without Nav2's command: stop publishing (the mode manager stops)
    plan_stale: float = 3.0                  # s without a plan (Nav2 replans at 1 Hz): the goal distance is unknown


class HeadingShaper:
    def __init__(self, cfg: HeadingConfig | None = None):
        self.cfg = cfg or HeadingConfig()
        self.v_odom = None              # Nav2's velocity in the odom frame (x, y), its turn rate, when it came
        self.w_nav = 0.0
        self.t_cmd = None
        self.odom = None                # (t, yaw, wz) of the latest odometry
        self.travelled = 0.0            # odometry path length, for the goal distance between plans
        self._xy = None
        self.plan = None                # (t, path length, travelled at that time)
        self.w_out = 0.0                # the turn rate last sent, and when
        self.t_out = None

    # -- inputs ---------------------------------------------------------------------------------------------------
    def on_odom(self, t: float, x: float, y: float, yaw: float, wz: float):
        if self._xy is not None:
            self.travelled += math.hypot(x - self._xy[0], y - self._xy[1])
        self._xy = (x, y)
        self.odom = (t, yaw, wz)

    def on_plan(self, t: float, length: float):
        self.plan = (t, length, self.travelled)

    def on_cmd(self, t: float, vx: float, vy: float, wz: float):
        """Nav2's command (body frame). Returns what to publish now: the command itself when agnostic (or before any
        odometry), else the shaped one."""
        if self.cfg.mode == "agnostic" or self.odom is None:
            self.t_cmd = None
            return vx, vy, wz
        if not self.active(t):                      # a new drive: take over from Nav2's rate, then ramp
            self.w_out, self.t_out = wz, t
        yaw = self.yaw_at(t)
        c, s = math.cos(yaw), math.sin(yaw)
        self.v_odom = (c * vx - s * vy, s * vx + c * vy)
        self.w_nav = wz
        self.t_cmd = t
        return self.step(t)

    # -- state ----------------------------------------------------------------------------------------------------
    def yaw_at(self, t: float) -> float:
        t0, yaw, wz = self.odom
        return yaw + wz * (t - t0)

    def goal_distance(self, t: float) -> float | None:
        if self.plan is None or t - self.plan[0] > self.cfg.plan_stale:
            return None
        return max(0.0, self.plan[1] - (self.travelled - self.plan[2]))

    def active(self, t: float) -> bool:
        return self.cfg.mode != "agnostic" and self.t_cmd is not None and t - self.t_cmd <= self.cfg.stale

    # -- output ---------------------------------------------------------------------------------------------------
    def target_rate(self, t: float) -> float:
        """The turn rate the mode asks for, blended back to Nav2's near the goal (and, forward, when slow)."""
        cfg = self.cfg
        d = self.goal_distance(t)
        k = 0.0 if d is None else ramp(d, *cfg.handback)
        vx, vy = self.v_odom
        if cfg.mode == "forward":
            v = math.hypot(vx, vy)
            k *= ramp(v, *cfg.slow)
            if k <= 0.0:
                return self.w_nav
            e = wrap(math.atan2(vy, vx) - self.yaw_at(t))
            # proportional, and never faster than it can stop at the heading with yaw_accel
            w = min(cfg.turn_max, cfg.gain * abs(e), math.sqrt(2.0 * cfg.yaw_accel * abs(e)))
            mine = math.copysign(w, e)
        else:
            mine = cfg.spin_rate
        return self.w_nav + k * (mine - self.w_nav)

    def step(self, t: float):
        """The shaped command (body frame) at time t; None once Nav2's command is stale."""
        if not self.active(t):
            return None
        cfg = self.cfg
        w_t = self.target_rate(t)
        a = cfg.yaw_accel * min(0.1, max(0.0, t - self.t_out))
        self.w_out += min(a, max(-a, w_t - self.w_out))
        self.t_out = t
        yaw = self.yaw_at(t) + self.w_out * (cfg.lead + 0.5 * cfg.period)
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = self.v_odom
        return c * vx + s * vy, -s * vx + c * vy, self.w_out
