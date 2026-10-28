"""Follow me and Tag (the console's Follow tab), ROS-free: people from the lidar, and a follower.

People. A person's legs cut the A2M12's scan plane (~15 cm up) as two short arcs. The scan is split into clusters
where consecutive returns jump apart; a cluster 4 to 30 cm wide is a leg, two legs within 55 cm (or one leg alone,
the other hidden behind it) a person. Chair and table legs look the same: with a map, candidates on its occupied
cells are dropped (the furniture was there when the map was made; people weren't).

Follow. The person nearest straight ahead within 3 m is taken when the mode starts; after that the track keeps to the
candidate nearest its last position (within 0.7 m per scan), so another person walking past doesn't take over. The
robot keeps `distance` from them (the base is holonomic: it closes the gap along the line to them and turns to face
them). Lost for 2 s: it stops and waits.

Tag. The same track, approached to 0.5 m ("Tagged!"), then the robot backs off to 1.5 m, counts 3 s and chases
again, at the kid speed. The collision monitor (when navigation runs) still stops it short of anything in its path.
"""

from __future__ import annotations

import math

import numpy as np

LEG_WIDTH = (0.04, 0.30)
PAIR_GAP = 0.55
JUMP = 0.12                      # m between consecutive returns that splits clusters (more at range: 3% of it)
MAX_RANGE = 4.0
TRACK_GATE = 0.7
LOST_S = 2.0


def detect(ranges, angle_min: float, angle_inc: float, laser=(0.0, 0.0, 0.0), occupied=None) -> list:
    """People as (x, y) in the robot's base frame, nearest first. laser: its (x, y, yaw) on the base. occupied:
    f(x_base, y_base) -> True on the map's occupied cells (furniture), or None."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.1) & (r < MAX_RANGE)
    if ok.sum() < 3:
        return []
    r, a = r[ok], a[ok]
    lx, ly, lyaw = laser
    x = lx + r * np.cos(a + lyaw)
    y = ly + r * np.sin(a + lyaw)
    d = np.hypot(np.diff(x), np.diff(y))
    split = np.nonzero(d > np.maximum(JUMP, 0.03 * r[:-1]))[0] + 1
    legs = []
    for idx in np.split(np.arange(len(r)), split):
        if len(idx) < 3:
            continue
        w = math.hypot(x[idx[-1]] - x[idx[0]], y[idx[-1]] - y[idx[0]])
        if LEG_WIDTH[0] <= w <= LEG_WIDTH[1]:
            cx, cy = float(x[idx].mean()), float(y[idx].mean())
            if occupied is None or not occupied(cx, cy):
                legs.append((cx, cy))
    # the scan wraps around: the first and last clusters may be one leg (rare; a duplicate leg costs nothing)
    people, used = [], set()
    for i, p in enumerate(legs):
        if i in used:
            continue
        best, bd = None, PAIR_GAP
        for j in range(i + 1, len(legs)):
            if j in used:
                continue
            dd = math.hypot(legs[j][0] - p[0], legs[j][1] - p[1])
            if dd < bd:
                best, bd = j, dd
        if best is not None:
            used.add(best)
            q = legs[best]
            people.append(((p[0] + q[0]) / 2, (p[1] + q[1]) / 2))
        else:
            people.append(p)
        used.add(i)
    people.sort(key=lambda p: math.hypot(*p))
    return people


class Follower:
    """mode: "follow" or "tag". tick(now, people in the base frame, base pose (x, y, yaw) in odom) -> twist."""

    def __init__(self, mode: str = "follow", distance: float = 1.0, speed: float = 0.6, turn_rate: float = 1.5):
        self.mode, self.distance, self.speed, self.turn_rate = mode, distance, speed, turn_rate
        self.target = None              # the person, in odom (the track survives the robot turning)
        self.t_seen = -math.inf
        self.phase = "look"             # look | follow | chase | tagged | back | count
        self.t_phase = 0.0
        self.tags = 0
        self.events = []

    def say(self, text: str):
        self.events.append(text)

    def text(self) -> str:
        return {"look": "Looking for a person: stand 1 to 2 m in front of the robot",
                "follow": f"Following you ({self.distance:.1f} m behind)",
                "chase": "Chasing you!", "tagged": "Tagged!", "back": "Backing off",
                "count": "Run! Coming in 3 seconds"}[self.phase] + (f" · {self.tags} tags" if self.mode == "tag" else "")

    def _set(self, phase, now):
        if phase != self.phase:
            self.phase, self.t_phase = phase, now

    def tick(self, now: float, people, pose):
        if pose is None:
            return (0.0, 0.0, 0.0)
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        world = [(x + c * px - s * py, y + s * px + c * py) for px, py in (people or [])]
        if self.target is None:
            ahead = [(wx, wy) for (wx, wy), (px, py) in zip(world, people or [])
                     if px > 0.2 and abs(math.atan2(py, px)) < math.radians(50) and math.hypot(px, py) < 3.0]
            if ahead:
                self.target, self.t_seen = ahead[0], now
                self.say("Got you: " + ("following" if self.mode == "follow" else "tag, you're it!"))
                self._set("follow" if self.mode == "follow" else "chase", now)
        else:
            near = min(world, key=lambda w: math.hypot(w[0] - self.target[0], w[1] - self.target[1]), default=None)
            if near is not None and math.hypot(near[0] - self.target[0], near[1] - self.target[1]) < TRACK_GATE:
                self.target = (0.5 * self.target[0] + 0.5 * near[0], 0.5 * self.target[1] + 0.5 * near[1])
                self.t_seen = now
            elif now - self.t_seen > LOST_S:
                self.target = None
                self.say("Lost you: stand in front of the robot")
                self._set("look", now)
        if self.target is None:
            return (0.0, 0.0, 0.0)
        dx, dy = self.target[0] - x, self.target[1] - y
        bx, by = c * dx + s * dy, -s * dx + c * dy            # the person in the base frame
        dist = math.hypot(bx, by)
        bearing = math.atan2(by, bx)
        wz = max(-self.turn_rate, min(self.turn_rate, 2.0 * bearing))
        keep = self.distance
        if self.mode == "tag":
            if self.phase == "chase" and dist < 0.5:
                self.tags += 1
                self.say(f"Tagged! ({self.tags})")
                self._set("tagged", now)
            if self.phase == "tagged":
                if now - self.t_phase > 0.8:
                    self._set("back", now)
                return (0.0, 0.0, wz)
            if self.phase == "back":
                if dist > 1.4 or now - self.t_phase > 4.0:
                    self._set("count", now)
                keep = 1.5
            elif self.phase == "count":
                if now - self.t_phase > 3.0:
                    self._set("chase", now)
                return (0.0, 0.0, wz)
            else:
                keep = 0.35                                    # aim inside the tag radius
        err = dist - keep
        v = max(-self.speed, min(self.speed, 1.2 * err))
        if abs(bearing) > math.radians(60) and v > 0:        # turn toward them before closing in
            v *= 0.3
        ux, uy = (bx / dist, by / dist) if dist > 1e-6 else (1.0, 0.0)
        return (v * ux, v * uy, wz)
