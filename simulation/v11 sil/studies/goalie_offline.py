"""The Goalie's aim, offline and fast: omni_console's goalie.py against omni_arm's ball tracker fed with D435-like
detections (depth error 0.01 + 0.02 r^2 m along the view ray, est; 15 Hz, 70 ms old), a base that follows its command
with a 0.12 s lag (est) at the console's 20 Hz, a ball slowing at 0.3 m/s^2 (est). A save: the ball reaches the robot's
front within 0.15 m of its centre line (wider ones glanced off the round bumper into the goal in the MuJoCo runs).
It chose goalie.trust 0.4 s and ball_track's process noise 1.5 m/s^2 (Nov 7); studies/goalie.py is the real test.

  python studies/goalie_offline.py            300 shots per setting, a few seconds
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROS = Path(__file__).resolve().parents[3] / "ros" / "src"
sys.path[:0] = [str(ROS / "omni_console"), str(ROS / "omni_arm")]
from omni_arm.ball_track import BallParams, BallTracker  # noqa: E402
from omni_console.goalie import Goalie, GoalieParams  # noqa: E402


def shot(rng, p: GoalieParams, accel: float, rate: float = 15.0, lag: float = 0.07, decel: float = 0.3) -> tuple:
    """(saved, on target) for one random shot from 3 m."""
    g = Goalie((0.0, 0.0, 0.0), p)
    tr = BallTracker(BallParams(accel=accel))
    side, aim, v = rng.uniform(-0.4, 0.4), rng.uniform(-0.55, 0.55), rng.uniform(1.5, 3.0)
    bx, by = 3.0, side
    d = np.array([-3.0, aim - side])
    bv = v * d / np.linalg.norm(d)
    rx = ry = vx = vy = 0.0
    t, dt, t_frame, t_ctl = 0.0, 0.005, 0.0, 0.0
    cmd, ball, hist = (0.0, 0.0, 0.0), None, []
    while t < 4.0:
        sp = float(np.linalg.norm(bv))
        if sp > 0:
            bv = bv * max(0.0, sp - decel * dt) / sp
        bx, by = bx + bv[0] * dt, by + bv[1] * dt
        hist.append((t, bx, by))
        if t >= t_frame:
            t_frame = t + 1.0 / rate
            old = [h for h in hist if h[0] <= t - lag]
            if old:
                _, ox, oy = old[-1]
                r = math.hypot(ox - rx, oy - ry)
                ang = math.atan2(oy - ry, ox - rx)
                e = rng.normal(0, 0.01 + 0.02 * r * r)
                tr.update(t - lag, (ox + e * math.cos(ang) + rng.normal(0, 0.01), oy + e * math.sin(ang)
                                    + rng.normal(0, 0.01)))
                st = tr.state(t)
                ball = dict(st, age=st.get("age", 0.0)) if st.get("seen") else None
        if t >= t_ctl:
            t_ctl = t + 0.05
            cmd = g.tick(t, (rx, ry, 0.0), ball)
        vx += (cmd[0] - vx) * dt / 0.12
        vy += (cmd[1] - vy) * dt / 0.12
        rx, ry = rx + vx * dt, ry + vy * dt
        if bx <= 0.25:
            return abs(by - ry) < 0.15, abs(by) < 0.6
        t += dt
    return True, False


def main():
    for accel in (4.0, 1.5):
        for trust in (0.0, 0.4, 0.6, 0.9):
            rng = np.random.default_rng(1)
            p = GoalieParams(trust=trust)
            res = [shot(rng, p, accel) for _ in range(300)]
            on = [s for s, o in res if o]
            print(f"tracker accel {accel} m/s^2, trust {trust} s: {sum(on)}/{len(on)} saved")


if __name__ == "__main__":
    main()
