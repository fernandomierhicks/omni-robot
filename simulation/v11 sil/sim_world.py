"""The Launcher as the sim's controller (design.md 2): the ROS sim's world changed while everything runs. No ROS.

  World        the walking person, the clutter and the house's thrower, on and off. The model holds the person and
               every piece of clutter from the start (bridge_core.CoreConfig(live_world=True)); what is off waits
               parked far outside the house, where no sensor reaches, floating with no contacts and no weight (so
               it costs the physics nearly nothing: resting on the floor out there, 14 things took ~13% of it). Clutter is "off", "toys" (the
               arm's things, where the arm's world puts them: ARM_AREA in the hallway) or "all" (the toys and larger
               things: boxes, a bag, shoes, a basket, a stool, a play ball, arm_objects.SETS["obstacles"]) put where
               they touch nothing, clear of the robot, the hallway and the doorways. The thrower is only a flag: the
               bridge's CatchSide reads it (no thrower, no catch game).
  ControlPort  the Launcher's link into the bridge (TCP 9873, from Windows through WSL's localhost forwarding): one
               JSON line in, one JSON line out, then closed. Polled from the bridge's loop, so the world changes
               between two physics steps.
  shoot / tick the Goalie's ball on the ROS sim: studies/goalie.py's rolled shot (the "obstacles" play ball, 22 cm, 0.3 kg)
               from 3 m ahead of the robot, scored on the truth: never crosses the goal line, a save; crosses between
               the posts, a goal; outside them, wide. ros/tools/goalie_smoke.py plays them through the control port.
"""

from __future__ import annotations

import json
import math
import socket

import mujoco
import numpy as np

import arm_objects

CLUTTER = ("off", "toys", "all")
BALL_BIG = arm_objects.SETS["obstacles"].index("ball_big")
GOAL_HALF = 0.6                   # m: the posts either side of the goal's middle (studies/goalie.py)
SHOT_MAX_S = 4.0                  # s (sim): the longest a shot is watched
PARK = (40.0, 40.0)               # m: where things wait while off (the lidar reaches 12 m; >= arm_objects.PARKED_X)
PERSON_PARK = (50.0, 50.0, 0.0)   # robot_model._person's own spot
DOORS = ((-3.0, 2.0), (0.45, 2.0), (3.45, 2.0))      # house.HOUSE_FEATURES' doorways
ROOMS_X, ROOMS_Y = (-5.6, 5.6), (-4.1, 4.1)          # inside the outer walls with a margin
HALL_Y = (0.3, 2.3)               # the hallway (and the doorways' mouths) stays clear of obstacles


def obstacle_objects() -> tuple:
    return arm_objects.objects_for(arm_objects.SETS["obstacles"], [(0.0, 0.0)] * len(arm_objects.SETS["obstacles"]),
                                   [0.0] * len(arm_objects.SETS["obstacles"]))


def parked_objects(c) -> tuple:
    """params.floor_objects for a live world: c's toys (bridge_core.toy_objects), then the obstacles, all parked."""
    from bridge_core import toy_objects
    out = []
    for k, o in enumerate(toy_objects(c) + obstacle_objects()):
        kind, size, _, _, _, rgba, kg = o
        out.append((kind, size, PARK[0] + 1.5 * k, PARK[1], 0.0, rgba, kg))
    return tuple(out)


class World:
    def __init__(self, core, person: bool = False, clutter: str = "off", thrower: bool | None = True):
        from bridge_core import toy_objects
        self.core, self.m, self.d = core, core.m, core.d
        c = core.cfg
        self.toys = toy_objects(c)
        self.objs = self.toys + obstacle_objects()
        self.adr, self.dof, self.geom, self.bodies = [], [], [], []
        for k in range(len(self.objs)):
            j = self.m.joint(f"obj{k}")
            self.adr.append(j.qposadr[0])
            self.dof.append(j.dofadr[0])
            self.geom.append(self.m.geom(f"obj{k}").id)
            self.bodies.append(self.m.body(f"obj{k}").id)
        self.floor = self.m.geom("floor").id
        self.mocap = self.m.body("person").mocapid[0]
        self.seed = c.seed
        self.person, self.clutter = False, "off"
        self.thrower = thrower if c.payload == "catch" else None     # the arm robot has no catch game
        self.placed: list[int] = []        # obstacles in the house (their index in objs)
        self.base = self.m.body("base").id
        self.shot: dict | None = None      # the shot rolling now (shoot, tick)
        self.n_shots = 0
        self.set(person=person, clutter=clutter)

    # -- changes ---------------------------------------------------------------------------------------------------
    def set(self, person: bool | None = None, clutter: str | None = None, thrower: bool | None = None) -> str:
        said = []
        if person is not None and bool(person) != self.person:
            self.person = bool(person)
            if self.person:                # in front of the robot, wherever it is now
                from bridge_core import Walker
                self.core.walker = Walker(self.m, self.d, self.core.r.pose(), self.core.base_id, t0=self.d.time)
                said.append("a person walks in front of the robot")
            else:
                self.core.walker = None
                self.d.mocap_pos[self.mocap] = PERSON_PARK
                said.append("the person left")
        if clutter is not None and clutter != self.clutter:
            if clutter not in CLUTTER:
                raise ValueError(f"clutter: one of {', '.join(CLUTTER)}")
            self.clutter = clutter
            self._toys(clutter in ("toys", "all"))
            n = self._obstacles(clutter == "all")
            said.append({"off": "the floor is clear", "toys": "toys on the floor",
                         "all": f"toys and {n} larger things on the floor"}[clutter])
        if thrower is not None and self.thrower is not None and bool(thrower) != self.thrower:
            self.thrower = bool(thrower)
            said.append("a thrower is ready (Catch)" if self.thrower else "no thrower")
        return "; ".join(said)

    def state(self) -> dict:
        return dict(person=self.person, clutter=self.clutter, thrower=self.thrower, obstacles=len(self.placed),
                    toys=len(self.toys))

    def place_toys(self, spots) -> list:
        """The toys at chosen spots (a study's scene: ros/tools/toy_route.py): toy k at spots[k] = (x, y, yaw deg),
        world, resting on the floor or a rug; the toys past the list parked. Per spot True, or False where furniture,
        a wall or the robot is (that toy parked)."""
        mujoco.mj_fwdPosition(self.m, self.d)
        gid = np.array([-1], dtype=np.int32)
        ok = []
        for k in range(len(self.toys)):
            if k >= len(spots):
                self._park(k)
                continue
            x, y, yaw = (float(v) for v in spots[k])
            hit = mujoco.mj_ray(self.m, self.d, np.array([x, y, 2.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, gid)
            ground = 2.0 - hit if hit >= 0 else 0.0
            if ground > 0.03:
                self._park(k)
                ok.append(False)
                continue
            self._put(k, x, y, ground + arm_objects.height(self.toys[k]) / 2 + 0.003, yaw)
            mujoco.mj_fwdPosition(self.m, self.d)
            if self._touching(k):
                self._park(k)
                ok.append(False)
            else:
                ok.append(True)
        mujoco.mj_fwdPosition(self.m, self.d)
        self.clutter = "toys"
        return ok

    def place_obstacles(self, spots) -> list:
        """Larger things at chosen spots (a study's scene): spots[i] = (k, x, y, yaw deg), k an index into
        arm_objects.SETS["obstacles"] (3 and 4 are the shoes, under the lidar's plane). The others stay parked. Per
        spot True, or False where it can't rest (that one parked)."""
        first = len(self.toys)
        for k in range(first, len(self.objs)):
            self._park(k)
        mujoco.mj_fwdPosition(self.m, self.d)
        gid = np.array([-1], dtype=np.int32)
        ok, self.placed = [], []
        for k, x, y, yaw in spots:
            o = first + int(k)
            hit = mujoco.mj_ray(self.m, self.d, np.array([float(x), float(y), 2.0]), np.array([0.0, 0.0, -1.0]),
                                None, 1, -1, gid)
            ground = 2.0 - hit if hit >= 0 else 0.0
            if ground > 0.03:
                ok.append(False)
                continue
            self._put(o, float(x), float(y), ground + arm_objects.height(self.objs[o]) / 2 + 0.003, float(yaw))
            mujoco.mj_fwdPosition(self.m, self.d)
            if self._touching(o):
                self._park(o)
                ok.append(False)
            else:
                ok.append(True)
                self.placed.append(o)
        mujoco.mj_fwdPosition(self.m, self.d)
        return ok

    # -- the Goalie's shots ----------------------------------------------------------------------------------------------
    def shoot(self, side: float = 0.0, aim: float = 0.0, speed: float = 2.0, dist: float = 3.0) -> dict:
        """The play ball from dist m ahead of the robot as it stands now and side m to its left, rolling (not skidding) at
        speed m/s toward the point aim m to the left of the robot's centre. {ok, id} or {ok: False, message}."""
        if self.shot is not None:
            return dict(ok=False, message="a shot is still rolling")
        m, d = self.m, self.d
        k = len(self.toys) + BALL_BIG
        rad = arm_objects.height(self.objs[k]) / 2
        mujoco.mj_fwdPosition(m, d)
        R, p0 = d.xmat[self.base].reshape(3, 3).copy(), d.xpos[self.base].copy()
        fwd, left = R[:, 0].copy(), R[:, 1].copy()
        fwd[2] = left[2] = 0.0
        fwd /= np.linalg.norm(fwd)
        left /= np.linalg.norm(left)
        start = p0 + dist * fwd + side * left
        target = p0 + aim * left
        start[2] = target[2] = rad + 0.001
        along = target - start
        length = float(np.linalg.norm(along))
        along /= length
        across = np.array([-along[1], along[0], 0.0])
        gid = np.array([-1], dtype=np.int32)
        for off in (-0.9 * rad, 0.0, 0.9 * rad):                  # the ball's whole width clear, up to the robot
            hit = mujoco.mj_ray(m, d, start + off * across, along, None, 1, self.bodies[k], gid)
            if 0 <= hit < length - 0.6:
                return dict(ok=False, message=f"something stands in the ball's way {hit:.2f} m along it")
        self._put(k, float(start[0]), float(start[1]), float(start[2]), 0.0)
        mujoco.mj_fwdPosition(m, d)
        if self._touching(k):
            self._park(k)
            return dict(ok=False, message="no room for the ball there")
        dof = self.dof[k]
        d.qvel[dof:dof + 3] = speed * along
        d.qvel[dof + 3:dof + 6] = np.cross([0.0, 0.0, 1.0], along) * (speed / rad)
        self.n_shots += 1
        self.shot = dict(id=self.n_shots, k=k, t0=float(d.time), p0=p0, fwd=fwd, left=left, crossed=None, closest=9.0,
                         side=float(side), aim=float(aim), speed=float(speed))
        return dict(ok=True, id=self.n_shots)

    def tick(self) -> dict | None:
        """Each few ms of sim time: the shot's truth once it is over (the ball parked again), else None."""
        s = self.shot
        if s is None:
            return None
        d = self.d
        b = d.xpos[self.bodies[s["k"]]]
        rel = b - s["p0"]
        u, w = float(rel @ s["fwd"]), float(rel @ s["left"])
        s["closest"] = min(s["closest"], float(np.hypot(*(b[:2] - d.xpos[self.base][:2]))))
        if s["crossed"] is None and u < 0.0:
            s["crossed"] = w
        dof = self.dof[s["k"]]
        t = float(d.time) - s["t0"]
        if s["crossed"] is None and t < SHOT_MAX_S and not (t > 1.0 and np.linalg.norm(d.qvel[dof:dof + 2]) < 0.05):
            return None
        c = s["crossed"]
        result = "goal" if c is not None and abs(c) < GOAL_HALF else ("wide" if c is not None else "save")
        self._park(s["k"])
        self.shot = None
        return dict(id=s["id"], result=result, crossed_at=None if c is None else round(c, 3),
                    closest_m=round(s["closest"], 3), side=round(s["side"], 3), aim=round(s["aim"], 3),
                    speed=round(s["speed"], 2), t=round(float(d.time), 2))

    # -- the things --------------------------------------------------------------------------------------------------
    def _put(self, k: int, x: float, y: float, z: float, yaw_deg: float, live: bool = True):
        """Object k at (x, y, z) turned yaw; live: it touches things and has weight, else it floats there touching
        nothing (parked: it costs the physics almost nothing)."""
        m, b, g = self.m, self.bodies[k], self.geom[k]
        ct, ca = (arm_objects.CONTYPE, arm_objects.CONAFFINITY) if live else (0, 0)
        m.geom_contype[g], m.geom_conaffinity[g] = ct, ca
        m.body_contype[b], m.body_conaffinity[b] = ct, ca    # the broadphase's per-body masks
        m.body_gravcomp[b] = 0.0 if live else 1.0
        a = math.radians(yaw_deg) / 2
        self.d.qpos[self.adr[k]:self.adr[k] + 7] = (x, y, z, math.cos(a), 0.0, 0.0, math.sin(a))
        self.d.qvel[self.dof[k]:self.dof[k] + 6] = 0.0

    def _park(self, k: int):
        self._put(k, PARK[0] + 1.5 * k, PARK[1], arm_objects.height(self.objs[k]) / 2 + 0.001, 0.0, live=False)

    def _toys(self, on: bool):
        for k, (kind, size, x, y, yaw, _, _) in enumerate(self.toys):
            if on:
                self._put(k, x, y, arm_objects.height((kind, size)) / 2 + 0.02, yaw)    # dropped 2 cm (rugs)
            else:
                self._park(k)

    def _obstacles(self, on: bool) -> int:
        first = len(self.toys)
        ks = list(range(first, len(self.objs)))
        for k in ks:
            self._park(k)
        self.placed = []
        if not on:
            return 0
        mujoco.mj_fwdPosition(self.m, self.d)
        rng = np.random.default_rng(self.seed + 5000)        # the same arrangement each time (when there's room)
        rx, ry, _ = self.core.r.pose()
        from bridge_core import ARM_AREA
        keep = [((rx, ry), 1.2), (ARM_AREA[0], ARM_AREA[1] + 0.4)]
        if self.person:
            p = self.d.mocap_pos[self.mocap]
            keep.append(((p[0], p[1]), 1.0))
        for k in ks:
            if self._place(k, rng, keep):
                self.placed.append(k)
                q = self.d.qpos[self.adr[k]:self.adr[k] + 2]
                keep.append(((float(q[0]), float(q[1])), 0.9))
        mujoco.mj_fwdPosition(self.m, self.d)
        return len(self.placed)

    def _place(self, k: int, rng, keep) -> bool:
        """A spot for obstacle k: in a room (not the hallway, not a doorway), clear of keep, on the floor or a rug,
        touching nothing else; else it stays parked."""
        h = arm_objects.height(self.objs[k])
        gid = np.array([-1], dtype=np.int32)
        for _ in range(300):
            x, y = rng.uniform(*ROOMS_X), rng.uniform(*ROOMS_Y)
            if HALL_Y[0] < y < HALL_Y[1] or any(math.hypot(x - a, y - b) < 1.0 for a, b in DOORS):
                continue
            if any(math.hypot(x - a, y - b) < r for (a, b), r in keep):
                continue
            hit = mujoco.mj_ray(self.m, self.d, np.array([x, y, 2.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, gid)
            ground = 2.0 - hit if hit >= 0 else 0.0
            if ground > 0.03:                            # furniture, the robot, a wall
                continue
            self._put(k, x, y, ground + h / 2 + 0.003, float(rng.uniform(0, 180)))
            mujoco.mj_fwdPosition(self.m, self.d)
            if not self._touching(k):
                return True
        self._park(k)
        return False

    def _touching(self, k: int) -> bool:
        """Obstacle k overlaps something other than the floor and the rugs and thresholds on it."""
        m, d, g = self.m, self.d, self.geom[k]
        g1, g2 = d.contact.geom1[:d.ncon], d.contact.geom2[:d.ncon]
        for a, b in zip(g1, g2):
            if g not in (a, b):
                continue
            o = int(b if a == g else a)
            if o == self.floor or (m.body_weldid[m.geom_bodyid[o]] == 0 and d.geom_xpos[o][2] < 0.02):
                continue
            return True
        return False


class ControlPort:
    """TCP, one JSON request per connection: {"get": 1} or changes ({"person": true, "clutter": "all", ...}); the
    reply is handle(request) as one JSON line."""

    def __init__(self, port: int = 9873, host: str = "0.0.0.0"):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(4)
        self.srv.setblocking(False)
        self.port = self.srv.getsockname()[1]

    def poll(self, handle) -> int:
        n = 0
        while n < 4:
            try:
                conn, _ = self.srv.accept()
            except (BlockingIOError, OSError):
                return n
            n += 1
            conn.settimeout(0.2)
            try:
                buf = b""
                while b"\n" not in buf and len(buf) < 4096:
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    buf += chunk
                req = json.loads(buf.split(b"\n")[0] or b"{}")
                if not isinstance(req, dict):
                    raise ValueError("a JSON object, please")
                rep = handle(req)
            except (OSError, ValueError) as e:
                rep = dict(ok=False, message=str(e))
            try:
                conn.sendall((json.dumps(rep) + "\n").encode())
            except OSError:
                pass
            conn.close()
        return n

    def close(self):
        self.srv.close()
