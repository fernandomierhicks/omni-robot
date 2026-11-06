"""Programs for kids (the console's Program tab), ROS-free: a list of blocks the robot runs one by one.

  forward / back / left / right  {"m": 0.1..3}       drive straight that far (left / right slide: it's an omni base)
  turn                           {"deg": -360..360}  + turns left
  spin                           {"turns": 1..5}
  wiggle                         {}                  a little dance: left, right, left, right
  wait                           {"s": 0.1..30}
  goto                           {"place": "..."}    a saved place, through Nav2 (needs navigation)
  repeat                         {"times": 1..20, "body": [blocks]}

Moves are closed on the odometry pose (odom frame, from the node at 20 Hz): a P controller with a speed cap, done
within 2 cm / 1.5 deg, and a time limit per block (a block that can't finish stops the program and says why). Each
block is measured from where the previous one should have ended (the intended pose), not where it did, so the small
leftovers don't add up: a square comes back to its start. The MCU
still applies its own limits (kid profile, ramps), and the node sends these commands through Nav2's collision monitor
when navigation runs, so a program can't drive into a wall the lidar sees.

The runner also measures what it drove on the same odometry (measured(): the turn unwrapped, the path length, the
fastest turn rate) and estimates a program's duration (estimate()): the AI's move reports both
(ai_robot_architecture.md 5.1).
"""

from __future__ import annotations

import math

KINDS = ("forward", "back", "left", "right", "turn", "spin", "wiggle", "wait", "goto", "repeat")
MAX_STEPS = 200                  # a program flattened (repeats unrolled) may have at most this many steps
POS_TOL, YAW_TOL = 0.02, math.radians(1.5)
K_POS, K_YAW = 1.6, 2.5          # 1/s
RATE_WINDOW = 0.5              # s: measured()'s fastest turn rate is over at least this long


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _num(b: dict, key: str, lo: float, hi: float, default: float) -> float:
    v = float(b.get(key, default))
    if not math.isfinite(v) or not lo <= v <= hi:
        raise ValueError(f"{b.get('do')}: {key} must be {lo:g} to {hi:g}")
    return v


def check(blocks, depth: int = 0) -> list:
    """The blocks as they will be saved (numbers checked and clipped to their ranges), or ValueError."""
    if not isinstance(blocks, list):
        raise ValueError("a program is a list of blocks")
    out = []
    for b in blocks:
        if not isinstance(b, dict) or b.get("do") not in KINDS:
            raise ValueError(f"unknown block {b!r} (one of {', '.join(KINDS)})")
        k = b["do"]
        if k in ("forward", "back", "left", "right"):
            out.append(dict(do=k, m=_num(b, "m", 0.05, 3.0, 0.5)))
        elif k == "turn":
            out.append(dict(do=k, deg=_num(b, "deg", -360.0, 360.0, 90.0)))
        elif k == "spin":
            out.append(dict(do=k, turns=_num(b, "turns", 0.25, 5.0, 1.0)))
        elif k == "wait":
            out.append(dict(do=k, s=_num(b, "s", 0.1, 30.0, 1.0)))
        elif k == "goto":
            if not str(b.get("place", "")).strip():
                raise ValueError("goto: which place?")
            out.append(dict(do=k, place=str(b["place"]).strip()))
        elif k == "wiggle":
            out.append(dict(do=k))
        elif k == "repeat":
            if depth >= 2:
                raise ValueError("repeat: at most two repeats inside each other")
            out.append(dict(do=k, times=int(_num(b, "times", 1, 20, 2)), body=check(b.get("body", []), depth + 1)))
    return out


def flatten(blocks) -> list:
    """Primitive steps: ("move", dx, dy) body frame, ("turn", rad), ("wait", s), ("goto", place)."""
    out = []
    for b in blocks:
        k = b["do"]
        if k == "repeat":
            for _ in range(b["times"]):
                out += flatten(b["body"])
        elif k in ("forward", "back", "left", "right"):
            dx, dy = {"forward": (1, 0), "back": (-1, 0), "left": (0, 1), "right": (0, -1)}[k]
            out.append(("move", dx * b["m"], dy * b["m"], k))
        elif k == "turn":
            out.append(("turn", math.radians(b["deg"]), k))
        elif k == "spin":
            out.append(("turn", 2 * math.pi * b["turns"], k))
        elif k == "wiggle":
            out += [("turn", math.radians(a), k) for a in (25, -50, 50, -25)]
        elif k == "wait":
            out.append(("wait", b["s"], k))
        elif k == "goto":
            out.append(("goto", b["place"], k))
        if len(out) > MAX_STEPS:
            raise ValueError(f"the program is too long (over {MAX_STEPS} steps with the repeats)")
    return out


class ProgramRunner:
    """Runs one flattened program. tick() -> twist (vx, vy, wz body), None (nothing to send) or the string "done"."""

    def __init__(self, name: str, blocks: list, speed: float, turn_rate: float):
        self.name = name
        self.steps = flatten(check(blocks))
        self.speed, self.turn_rate = speed, turn_rate
        self.i = -1
        self.start = None             # (x, y, yaw, t) when the current step began
        self.target = None
        self.error = None
        self.goto_state = None        # None | "sent" | "done" | "failed"
        self.ref = None               # the intended pose (x, y, yaw) after the blocks so far
        self.t0 = self.t_last = None  # what it measured on the odometry: measured()
        self.last_pose = None
        self.yaw_total = self.dist_total = self.max_rate = 0.0
        self.yaw_hist = []            # (now, yaw_total) over the last RATE_WINDOW or so

    def label(self) -> str:
        if self.i < 0:
            return "starting"
        if self.i >= len(self.steps):
            return "done"
        return f"step {self.i + 1} of {len(self.steps)}: {self.steps[self.i][-1]}"

    def _begin(self, pose, now):
        self.i += 1
        self.start = (*pose, now)
        if self.ref is None:
            self.ref = tuple(pose)
        if self.i >= len(self.steps):
            return
        s = self.steps[self.i]
        rx, ry, ryaw = self.ref
        if s[0] == "move":
            c, sn = math.cos(ryaw), math.sin(ryaw)
            self.target = (rx + c * s[1] - sn * s[2], ry + sn * s[1] + c * s[2], ryaw)
            self.ref = self.target
            self.limit = 5.0 + 3.0 * math.hypot(s[1], s[2]) / max(self.speed, 0.05)
        elif s[0] == "turn":
            # turn the block's angle plus whatever the last block left of the intended heading
            self.target = s[1] + wrap(ryaw - pose[2])
            self.turned, self.last_yaw = 0.0, pose[2]
            self.ref = (rx, ry, ryaw + s[1])
            self.limit = 5.0 + 3.0 * abs(s[1]) / max(self.turn_rate, 0.1)
        elif s[0] == "goto":
            self.ref = None                           # Nav2 decides where it ends: start again from there
        elif s[0] == "wait":
            self.limit = s[1]
        if s[0] == "goto":
            self.goto_state = None
            self.limit = 180.0

    def tick(self, now: float, pose):
        """pose: (x, y, yaw) in odom, or None (no odometry: hold still)."""
        if self.error:
            return "done"
        if pose is None:
            return (0.0, 0.0, 0.0)
        self._measure(now, pose)
        if self.i < 0 or self.i >= len(self.steps):
            if self.i >= len(self.steps):
                return "done"
            self._begin(pose, now)
            if self.i >= len(self.steps):
                return "done"
        s = self.steps[self.i]
        x, y, yaw = pose
        t_in = now - self.start[3]
        if s[0] != "wait" and t_in > self.limit:
            self.error = f"{self.label()} didn't finish in {self.limit:.0f} s (blocked?)"
            return "done"
        if s[0] == "move":
            ex, ey = self.target[0] - x, self.target[1] - y
            c, sn = math.cos(yaw), math.sin(yaw)
            bx, by = c * ex + sn * ey, -sn * ex + c * ey            # error in the body frame
            d = math.hypot(bx, by)
            eyaw = wrap(self.target[2] - yaw)
            if d < POS_TOL:
                return self._next(pose, now)
            v = min(self.speed, K_POS * d)
            return (v * bx / d, v * by / d, max(-self.turn_rate, min(self.turn_rate, K_YAW * eyaw)))
        if s[0] == "turn":
            self.turned += wrap(yaw - self.last_yaw)
            self.last_yaw = yaw
            left = self.target - self.turned
            if abs(left) < YAW_TOL:
                return self._next(pose, now)
            return (0.0, 0.0, max(-self.turn_rate, min(self.turn_rate, K_YAW * left)))
        if s[0] == "wait":
            if t_in >= s[1]:
                return self._next(pose, now)
            return (0.0, 0.0, 0.0)
        if s[0] == "goto":
            if self.goto_state == "done":
                return self._next(pose, now)
            if self.goto_state == "failed":
                self.error = f"couldn't reach '{s[1]}'"
                return "done"
            return None                                  # Nav2 drives; the node sends nothing
        return (0.0, 0.0, 0.0)

    def _next(self, pose, now):
        self._begin(pose, now)
        if self.i >= len(self.steps):
            return "done"
        return (0.0, 0.0, 0.0)

    def _measure(self, now: float, pose):
        if self.last_pose is not None:
            dyaw = wrap(pose[2] - self.last_pose[2])
            self.yaw_total += dyaw
            self.dist_total += math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
            # the rate over at least RATE_WINDOW: between two ticks, the pose's arrival jitter read 236 deg/s for a
            # true 188 (ai_smoke, Nov 5), which looks like the AI's 180 deg/s ceiling broken
            self.yaw_hist.append((now, self.yaw_total))
            while len(self.yaw_hist) > 2 and now - self.yaw_hist[1][0] >= RATE_WINDOW:
                self.yaw_hist.pop(0)
            t_a, y_a = self.yaw_hist[0]
            if now - t_a >= RATE_WINDOW:
                self.max_rate = max(self.max_rate, abs(self.yaw_total - y_a) / (now - t_a))
        else:
            self.t0 = now
            self.yaw_hist = [(now, 0.0)]
        self.last_pose, self.t_last = tuple(pose), now

    def measured(self) -> dict:
        """What the odometry says the program did so far: the turn (deg, + left, unwrapped), the path (m), the time
        and the fastest turn rate over RATE_WINDOW (deg/s)."""
        return dict(turned_deg=round(math.degrees(self.yaw_total), 1), distance_m=round(self.dist_total, 3),
                    duration_s=round((self.t_last or 0.0) - (self.t0 or 0.0), 2),
                    max_turn_rate_deg_s=round(math.degrees(self.max_rate), 1), frame="odom")

    def estimate(self, yaw_accel: float = 15.0, accel: float = 2.0) -> float:
        """Seconds the program should take (no goto blocks): each move or turn at its speed cap, plus getting up to it
        (the firmware's acceleration) and the P controller's slow tail into the tolerance; waits as they are."""
        def leg(dist, v, k, tol, a):
            if dist <= tol:
                return 0.0
            if dist <= v / k:                           # never reaches the cap: all tail
                return math.log(dist / tol) / k
            return (dist - v / k) / v + math.log(v / (k * tol)) / k + v / (2.0 * a)
        t = 0.0
        for s in self.steps:
            if s[0] == "move":
                t += leg(math.hypot(s[1], s[2]), self.speed, K_POS, POS_TOL, accel)
            elif s[0] == "turn":
                t += leg(abs(s[1]), self.turn_rate, K_YAW, YAW_TOL, yaw_accel)
            elif s[0] == "wait":
                t += s[1]
            t += 0.05                                    # a tick to start the next block
        return t

    def wants_goto(self):
        """The place of a goto step that hasn't been sent to Nav2 yet, or None."""
        if 0 <= self.i < len(self.steps) and self.steps[self.i][0] == "goto" and self.goto_state is None:
            self.goto_state = "sent"
            return self.steps[self.i][1]
        return None
