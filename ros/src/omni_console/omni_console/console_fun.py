"""The console core's fun missions (ROS-free, a mixin of missions.Console), each a short state machine over what the
console already has: Nav2 goals, its own twist, omni_arm's requests, the people the lidar (or the people node) sees,
the face's lines.

  FETCH    "bring me the red cube": find (omni_arm's colour and shape find; a quarter turn and again, four looks at
           most), pick it and keep it (omni_arm's tidy, keep), to the person who asked (the nearest person seen: Nav2 to
           0.7 m short of them, facing them; nobody seen: a look round, then back where the fetch started), put it down
           ahead (omni_arm's place), "here you go". Modes: PICK for the arm's parts, TELEOP for Nav2's
  MESSAGE  "tell the kids dinner's ready": room by room (the map's rooms, nearest first: Nav2 to each one's middle), a
           look round in each (one slow turn) until a person is seen, up to them (Nav2, 0.9 m short), the message said;
           nobody in any room: says so
  TOWER    omni_arm's tidy in tower mode: the first block it finds is the base, the others go on top (PICK)
  DANCE    dance.py's routine on the beat: the console's twist, the face's tricks (TELEOP)
  GOALIE   goalie.py against the ball the D435 tracks (omni_arm's ball_track_node, enabled while it plays): sliding
           sideways to block, the score on the status and quips on the face (TELEOP, the twist unguarded: the ball is
           what it must meet)

tick() advances the phases (the node's 2 Hz tick, the arm's state, Nav2's results, the people); fun_twist() gives the
twist of the phases the console drives (look rounds, the dance, the goalie). Every speed is a request the MCU caps by
its profile; a person's command, STOP or the RC ends them like any mission (missions._enter).
"""

from __future__ import annotations

import math

FUN_MISSIONS = ("FETCH", "MESSAGE", "TOWER", "DANCE", "GOALIE")
MODES_OK = {"FETCH": ("PICK", "TELEOP")}            # missions that switch the mode manager's mode as they go
LOOK_W = 0.7                                         # rad/s: a look round's turn (people and finds need still-ish frames)
FIND_LOOKS = 4                                       # quarter turns a fetch looks in before it gives up
PERSON_RANGE = 4.5                                   # m: a person farther than this isn't "seen" for a message
PEOPLE_STALE = 1.0                                   # s


def _centroid(pts) -> tuple:
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return sum(xs) / len(xs), sum(ys) / len(ys)


class FunMixin:
    # -- shared pieces -------------------------------------------------------------------------------------------------
    def _fun_init(self):
        self.fun = None                     # the running fun mission's state (dict), or None
        self.people_seen, self.t_people = [], -math.inf
        self.ball, self.t_ball = None, -math.inf
        self.goalie = None                  # goalie.Goalie while GOALIE runs
        self.dance = None                   # dance.Dance while DANCE runs
        self.fun_n = 0

    def wants_people(self) -> bool:
        """The node runs the people detector (the lidar's scan) while this is true."""
        return self.mission in ("FOLLOW", "MESSAGE", "FETCH")

    def mode_ok(self, mission: str, mode: str) -> bool:
        return mode in MODES_OK.get(mission, (self.mode_of(mission),))

    def _mode(self, mode: str) -> list:
        if mode == self.robot_mode and self.pending_mode is None:
            return []
        self.pending_mode = mode
        return [("set_mode", mode, self.values["profile.kid"])]

    def _say_fx(self, text: str, mood: str = "happy", act=None) -> tuple:
        return ("face", dict(text=text, mood=mood, act=act))

    def _people_now(self) -> list:
        return self.people_seen if self.now - self.t_people < PEOPLE_STALE else []

    def _person_map(self, rng: float = PERSON_RANGE):
        """The nearest person seen, in the map frame (x, y), and the robot's map pose; or None."""
        if self.pose is None or self.pose[0] != "map":
            return None
        _, rx, ry, ryaw = self.pose
        for bx, by in self._people_now():
            if math.hypot(bx, by) <= rng:
                c, s = math.cos(ryaw), math.sin(ryaw)
                return rx + c * bx - s * by, ry + s * bx + c * by
        return None

    def _goal_short_of(self, px: float, py: float, short: float) -> tuple:
        """A Nav2 goal short m before (px, py) on the way from the robot, facing it."""
        _, rx, ry, _ = self.pose
        a = math.atan2(py - ry, px - rx)
        d = max(0.0, math.hypot(px - rx, py - ry) - short)
        return rx + d * math.cos(a), ry + d * math.sin(a), a

    def _phase(self, phase: str, task: str | None = None):
        self.fun["phase"], self.fun["t_phase"] = phase, self.now
        if task:
            self.task = task
            self.event("info", task)

    def _fun_end(self, ok: bool, text: str, fx: list | None = None) -> list:
        """The fun mission is over: back to MANUAL (motors on, the stick ready), its message as the task."""
        out = list(fx or [])
        if self.goal is not None:
            out.append(("nav_cancel",))
            self.goal = None
        if self.mission == "GOALIE":
            out.append(("goalie", False))
        self.fun, self.dance, self.goalie = None, None, None
        self.mission, self.progress = "MANUAL", None
        self.task = text
        self.zero_until = self.now + 0.3
        self.event("info" if ok else "warn", text)
        out += self._mode("TELEOP")
        self.activity_sync()
        return out

    def _fun_nav(self, x: float, y: float, yaw: float, name: str) -> list:
        return self._mode("TELEOP") + self._nav_goal(x, y, yaw, name, self.values["goto.speed"], self.values["goto.turn"])

    # -- the verbs -------------------------------------------------------------------------------------------------------
    def _fetch(self, a: dict):
        from omni_console.console_defs import FIND_SHAPES, Reply
        if not self._arm_up():
            return Reply(False, "Fetch needs the arm robot (omni_arm's arm_node)")
        color = str(a.get("color") or "").strip().lower()[:20] or None
        shape = str(a.get("shape") or "any").lower()
        if shape not in FIND_SHAPES:
            return Reply(False, f"shape: one of {', '.join(FIND_SHAPES)}")
        to = str(a.get("to") or self.values["fetch.to"])
        if to not in ("me", "here"):
            return Reply(False, "to: me (the nearest person) or here (where the fetch starts)")
        what = " ".join(x for x in (color, None if shape == "any" else shape) if x) or "something"
        r = self._enter("FETCH", f"Fetch: looking for {'the ' if what != 'something' else ''}{what}", "info")
        start = None if self.pose is None or self.pose[0] != "map" else tuple(self.pose[1:])
        self.fun = dict(kind="fetch", what=what, color=color, shape=shape, to=to, looks=0, tries=0, start=start,
                        oid=None, find_id=None, phase="", t_phase=self.now, turned=0.0, yaw0=None)
        fx = list(r.effects) + self._fetch_find()
        return Reply(True, r.message, None, tuple(fx))

    def _message(self, a: dict):
        from omni_console.console_defs import SAY_MAX, Reply
        text = str(a.get("text", "")).strip()[:SAY_MAX]
        if not text:
            return Reply(False, "What's the message?")
        names = a.get("rooms") or sorted(self.rooms)
        rooms = [(n, *_centroid(self.rooms[n])) for n in names if n in self.rooms]
        if a.get("rooms") and not rooms:
            return Reply(False, f"No such rooms on this map ({', '.join(sorted(self.rooms)) or 'none drawn'})")
        if rooms and not self.caps["nav"]:
            return Reply(False, "Going room to room needs navigation running")
        if rooms and (self.pose is None or self.pose[0] != "map"):
            return Reply(False, "The robot isn't localized on the map: it can't go room to room")
        # nearest first, then each next one nearest the last
        order, here = [], (self.pose[1], self.pose[2]) if self.pose is not None else (0.0, 0.0)
        while rooms:
            k = min(range(len(rooms)), key=lambda i: math.hypot(rooms[i][1] - here[0], rooms[i][2] - here[1]))
            order.append(rooms.pop(k))
            here = order[-1][1:]
        r = self._enter("MESSAGE", "Message: looking round here first", "info")
        self.fun = dict(kind="message", text=text, rooms=order, i=-1, phase="", t_phase=self.now, turned=0.0,
                        yaw0=None, said=False)
        self._phase("look")
        return Reply(True, f"Taking the message ({len(order)} rooms to try)" if order else "Looking round here for "
                                                                                       "someone", None, r.effects)

    def _tower(self, a: dict):
        from omni_console.console_defs import Reply
        if not self._arm_up():
            return Reply(False, "Block tower needs the arm robot (omni_arm's arm_node)")
        n = int(a.get("max_blocks") or self.values["tower.max_blocks"])
        if not 2 <= n <= 5:
            return Reply(False, "max_blocks: 2 to 5")
        r = self._enter("TOWER", "Block tower: looking for blocks", "info")
        req = dict(verb="tidy", tower=True, max_blocks=n, radius=float(self.values["tidy.radius"]))
        self.fun = dict(kind="tower", phase="build", t_phase=self.now)
        return Reply(True, r.message, None, r.effects + (("arm", req),))

    def _dance(self, a: dict):
        from omni_console.console_defs import Reply
        from omni_console.dance import Dance
        v = self.values
        try:
            d = Dance(str(a.get("routine") or v["dance.routine"]), float(a.get("bpm") or v["dance.bpm"]),
                      float(v["dance.speed"]), float(v["dance.seconds"]))
        except ValueError as e:
            return Reply(False, str(e))
        r = self._enter("DANCE", f"Dancing: {d.routine} at {d.bpm:.0f} bpm", "info")
        self.dance = d
        self.fun = dict(kind="dance", phase="dance", t_phase=self.now)
        return Reply(True, r.message, None, r.effects + (self._say_fx("", "happy", "sing"),))

    def _goalie(self, a: dict):
        from omni_console.console_defs import Reply
        from omni_console.goalie import Goalie, GoalieParams
        if self.odom_pose is None:
            return Reply(False, "Goalie needs the robot's odometry (the MCU link)")
        p = GoalieParams(half_width=float(self.values["goalie.width"]) / 2, max_speed=float(self.values["goalie.speed"]))
        r = self._enter("GOALIE", "Goalie: roll the ball at me", "info")
        self.goalie = Goalie(tuple(self.odom_pose), p)
        self.ball, self.t_ball = None, -math.inf
        self.fun = dict(kind="goalie", phase="play", t_phase=self.now)
        return Reply(True, r.message, None, r.effects + (("goalie", True),
                                                         self._say_fx("Ready. Do your worst.", "focused")))

    # -- inputs -----------------------------------------------------------------------------------------------------------
    def on_ball(self, b: dict | None, now: float):
        """omni_arm's /goalie/ball (odom): {x, y, vx, vy, seen}."""
        if b and b.get("seen"):
            self.ball, self.t_ball = b, now

    def fun_people(self, people, now: float):
        self.people_seen, self.t_people = list(people), now

    # -- the phases ---------------------------------------------------------------------------------------------------
    def _fetch_find(self) -> list:
        f = self.fun
        self.fun_n += 1
        f["find_id"] = f"fetch-{self.fun_n}"
        f["looks"] += 1
        self._phase("find")
        req = dict(verb="find", id=f["find_id"], color=f["color"], shape=f["shape"], max_range=2.0)
        return self._mode("PICK") + [("arm", req)]

    def _start_look(self, phase: str, turn: float):
        self.fun.update(turned=0.0, yaw0=None, turn=turn)
        self._phase(phase)

    def fun_tick(self, now: float) -> list:
        """The node's 2 Hz tick: timeouts and the phases that wait on time."""
        f = self.fun
        if f is None or self.mission not in FUN_MISSIONS:
            return []
        if f["kind"] == "message":
            return self._message_tick(now)
        if f["kind"] == "fetch":
            if f["phase"] == "find" and now - f["t_phase"] > 20.0:
                return self._fun_end(False, "Fetch: the find didn't answer")
            if f["phase"] == "say" and now - f["t_phase"] > 3.0:
                return self._fun_end(True, f"Fetched the {f['what']}")
            if f["phase"] == "look_person" and self._person_map() is not None:
                return self._fetch_deliver()
        if f["kind"] == "goalie" and self.goalie is not None:
            out = []
            for e in self.goalie.events:
                g = self.goalie
                line = {"save": ("Saved!", "happy"), "goal": ("Goal. I'll allow it.", "sad"),
                        "miss": ("Wide. Nice try.", "happy")}.get(e)
                if line:
                    out.append(self._say_fx(*line))
                self.event("info", f"Goalie: {e} ({g.saves} saved, {g.goals} goals)")
            self.goalie.events.clear()
            self.task = f"Goalie: {self.goalie.saves} saved, {self.goalie.goals} let in"
            return out
        return []

    def _message_tick(self, now: float) -> list:
        f = self.fun
        if f["phase"] == "look":
            person = self._person_map() if self.pose is not None else None
            near = [p for p in self._people_now() if math.hypot(*p) < 2.0]
            if person is not None:
                if math.hypot(person[0] - self.pose[1], person[1] - self.pose[2]) < 1.3:
                    return self._message_say()
                x, y, yaw = self._goal_short_of(person[0], person[1], 0.9)
                self._phase("approach", "Message: someone's there, going over")
                return self._fun_nav(x, y, yaw, "the person")
            if near and self.pose is None:                       # no map: someone right here hears it
                return self._message_say()
            if f["turned"] >= 2 * math.pi:
                return self._message_next()
        if f["phase"] == "say" and now - f["t_phase"] > 4.0:
            return self._fun_end(True, f"Message delivered: \"{f['text']}\"")
        return []

    def _message_next(self) -> list:
        f = self.fun
        f["i"] += 1
        if f["i"] >= len(f["rooms"]):
            tried = len(f["rooms"])
            return self._fun_end(False, f"Message not delivered: nobody found{f' in {tried} rooms' if tried else ''}",
                                 [self._say_fx("I couldn't find anyone to tell.", "sad")])
        name, x, y = f["rooms"][f["i"]]
        self._phase("go", f"Message: going to the {name}")
        return self._fun_nav(x, y, math.atan2(y - self.pose[2], x - self.pose[1]), name)

    def _message_say(self) -> list:
        f = self.fun
        self._phase("say", f"Message: telling them \"{f['text']}\"")
        return [self._say_fx(f"Message for you: {f['text']}", "happy")]

    def _fetch_deliver(self) -> list:
        """Holding it: to the person, or back where the fetch started."""
        f = self.fun
        person = self._person_map(6.0) if f["to"] == "me" else None
        if person is not None and self.caps["nav"]:
            if math.hypot(person[0] - self.pose[1], person[1] - self.pose[2]) < 1.1:
                return self._fetch_place()
            x, y, yaw = self._goal_short_of(person[0], person[1], 0.7)
            self._phase("deliver", f"Fetch: bringing the {f['what']} over")
            return self._fun_nav(x, y, yaw, "the person")
        if f["to"] == "me" and f["phase"] != "look_person":
            self._start_look("look_person", 2 * math.pi)
            self.task = f"Fetch: got the {f['what']}, looking for you"
            return self._mode("TELEOP")
        if f["start"] is not None and self.caps["nav"]:
            x, y, yaw = f["start"]
            self._phase("deliver", f"Fetch: bringing the {f['what']} back here")
            return self._fun_nav(x, y, yaw, "the start")
        return self._fetch_place()

    def _fetch_place(self) -> list:
        self._phase("place", f"Fetch: putting the {self.fun['what']} down")
        return self._mode("PICK") + [("arm", dict(verb="place", ahead=True))]

    def fun_on_arm(self, s: dict, was) -> list:
        """omni_arm's /arm/state for FETCH and TOWER: a find's answer, the tidy's end."""
        f = self.fun
        if f is None or self.mission not in ("FETCH", "TOWER"):
            return []
        t = s.get("tidy") or {}
        ended = was is not None and was.get("tidy_on") and not s.get("tidy_on")
        if f["kind"] == "tower":
            tw = t.get("tower") or {}
            if s.get("tidy_on"):
                st = t.get("state", "").lower().replace("_", " ")
                self.task = f"Block tower: {tw.get('n', 0)} high ({st})" if tw else f"Block tower: {st}"
            if ended:
                n = tw.get("n", 0)
                return self._fun_end(n >= 2, f"Block tower: {n} block{'s' * (n != 1)} high"
                                     + (f" ({t.get('outcome') or t.get('reason')})" if n < 2 else ""),
                                     [self._say_fx("Behold. A tower." if n >= 2 else "No tower today.",
                                                   "happy" if n >= 2 else "sad")])
            return []
        found = s.get("found")
        if f["phase"] == "find" and isinstance(found, dict) and found.get("id") == f["find_id"]:
            cands = found.get("candidates") or []
            if found.get("error") or not cands:
                if f["looks"] >= FIND_LOOKS:
                    return self._fun_end(False, f"Fetch: no {f['what']} in sight after a look round",
                                         [self._say_fx(f"I can't see any {f['what']}.", "sad")])
                self._start_look("turn", math.pi / 2)
                return self._mode("TELEOP")
            c = cands[0]
            f["oid"] = int(c["id"])
            self._phase("pick", f"Fetch: picking up the {f['what']} ({float(c.get('range_m', 0)):.1f} m away)")
            radius = max(float(self.values["tidy.radius"]), float(c.get("range_m", 1.0)) + 0.5)
            return self._mode("PICK") + [("arm", dict(verb="tidy", only=f["oid"], keep=True, max_objects=1,
                                                      radius=radius))]
        if f["phase"] == "pick" and ended:
            if t.get("outcome") == "held":
                return self._fetch_deliver()
            f["tries"] += 1
            if f["tries"] < 2:
                self.event("warn", f"Fetch: the pick didn't work ({t.get('outcome') or t.get('reason')}): again")
                return self._fetch_find()
            return self._fun_end(False, f"Fetch: couldn't pick up the {f['what']} ({t.get('outcome') or t.get('reason')})")
        if f["phase"] == "place" and ended:
            if t.get("outcome") == "placed":
                self._phase("say", f"Fetch: here's the {f['what']}")
                return [self._say_fx("Here you go.", "happy", "smile")]
            return self._fun_end(False, f"Fetch: couldn't put it down ({t.get('outcome') or t.get('reason')})")
        return []

    def fun_nav_done(self, ok: bool, why: str) -> list:
        f = self.fun
        if f is None:
            return []
        if f["kind"] == "message":
            if f["phase"] == "go":
                if not ok:
                    self.event("warn", f"Message: couldn't reach the {f['rooms'][f['i']][0]} ({why})")
                    return self._message_next()
                self._start_look("look", 2 * math.pi)
                self.task = f"Message: looking round the {f['rooms'][f['i']][0]}"
                return []
            if f["phase"] == "approach":
                if ok or self._person_map(2.0) is not None:
                    return self._message_say()
                self._start_look("look", 2 * math.pi)
                return []
        if f["kind"] == "fetch" and f["phase"] == "deliver":
            if not ok:
                self.event("warn", f"Fetch: couldn't get all the way ({why}): putting it down here")
            return self._fetch_place()
        return []

    # -- the console's own twist ----------------------------------------------------------------------------------------
    def fun_twist(self, now: float):
        """(vx, vy, wz) for the phases the console drives, None to send nothing (Nav2 or the arm drive)."""
        f = self.fun
        if f is None:
            return None
        if f["kind"] == "dance" and self.dance is not None:
            tw, move, tricks = self.dance.tick(now)
            for tr in tricks:
                self.fx_queue.append(self._say_fx("", "happy", tr))
            if tw == "done":
                self.fx_queue += self._fun_end(True, "Dance over. Applause welcome.",
                                               [self._say_fx("Thank you, thank you.", "happy", "smile")])
                return (0.0, 0.0, 0.0)
            self.task = f"Dancing: {move}"
            self.progress = min(1.0, (now - self.dance.t0) / self.dance.length)
            return tw
        if f["kind"] == "goalie" and self.goalie is not None:
            ball = None
            if self.ball is not None:
                ball = dict(self.ball, age=now - self.t_ball)
            return self.goalie.tick(now, self.odom_pose, ball)
        if f.get("phase") in ("look", "turn", "look_person") and self.odom_pose is not None:
            yaw = self.odom_pose[2]
            if f["yaw0"] is None:
                f["yaw0"], f["last_yaw"] = yaw, yaw
            f["turned"] += abs(math.remainder(yaw - f["last_yaw"], 2 * math.pi))
            f["last_yaw"] = yaw
            goal = f.get("turn", 2 * math.pi)
            if f["turned"] >= goal:
                if f["phase"] == "turn":                    # a fetch's quarter turn: look again
                    self.fx_queue += self._fetch_find()
                elif f["phase"] == "look_person":           # nobody to bring it to: back where it started
                    f["to"] = "here"
                    self.fx_queue += self._fetch_deliver()
                return (0.0, 0.0, 0.0)
            return 0.0, 0.0, LOOK_W
        return None
