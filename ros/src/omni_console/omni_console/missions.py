"""The operator console's robot side, ROS-free: missions, settings, saved places and routes, the SLAM requests.
mission_node.py is the ROS node around it; the web page (web/) is a thin skin over its status and its one command
service.

  IDLE     the MCU disarms (ramped stop first)                              mode_manager IDLE
  MANUAL   the console's stick drives (Foxglove / the joystick still can)   TELEOP
  GOTO     Nav2 drives to a goal on the map                                 TELEOP
  EXPLORE  Nav2 drives to frontiers (explore.py) while SLAM maps            TELEOP
  PATROL   Nav2 drives a saved route: once, in a loop or back and forth     TELEOP
  CATCH    the catch behavior plays                                         CATCH
  THROWER  bench test of the module: flywheels, tilt, fire one ball         THROWER
  FOLLOW   follow the nearest person, or play tag (people.py, the lidar)    TELEOP
  PROGRAM  run a kid's block program (program.py)                           TELEOP
  CALIBRATE  a guided calibration drive (calibration.py)                    TELEOP
  TIDY     the arm robot picks up small things and drops them in its bin    PICK
           (omni_arm's tidy task drives the base and the arm; design.md 11)
A command returns effects (set_mode, nav_goal, nav_cancel, nav_limits, param, slam_start, slam_save, initial_pose,
masks, arm: a request for omni_arm's arm_brain, face / face_hear: a line for omni_face to say, a typed voice command)
that the node carries out; tick() advances Explore and Patrol, drive_output() gives the twist of the driving
missions (MANUAL, FOLLOW, PROGRAM, CALIBRATE) at 20 Hz, capped by the map's slow bands (zones.py). The core never
blocks and never touches ROS. The MCU stays the safety authority: every speed here is a request it caps by its own
profile (adult 2 m/s, kid 0.8 m/s), and its interlocks decide whether a ball is fired.
Also kept here: the map's no-go zones and edges, the kids' programs, the calibration results, the throw log and
leaderboard (throws.py) and the health counters (health.py); the node saves them.

An AI client (ai_robot_architecture.md: omni_mcp, for Claude or Codex) uses the same verbs and a few of its own
(AI_VERBS), always with its session id in args["ai"]. It chooses tasks, never wheel or servo commands: move runs a
program (PROGRAM) under its own speed ceilings (ai.max_speed, ai.max_turn, then the firmware's profile), goto drives
with Nav2, find asks omni_arm for the floor's candidates by colour and shape, pick has the tidy fetch one of them, arm
moves the arm to a named pose. Each becomes a task (ai_tasks.py) that ends with its mission. Motion needs a lease the
client renews every second (ai_lease); when it lapses, the AI's task is cancelled and the robot stops. A person
always wins: any of HUMAN_VERBS from the page or the voice, or the RC's manual switch, cancels the AI's task, and the
AI may not start motion for HUMAN_HOLD seconds after. stop and cancel need no lease.
"""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
from collections import deque
from omni_console.console_defs import (  # noqa: F401  (re-exported: tests and the node import them from here)
    AI_MOVE_MAX,
    AI_TIMEOUT,
    AI_TTL,
    AI_VERBS,
    ARM_JOG_MAX,
    ARM_POSES,
    ARM_STALE,
    DRIVE_MISSIONS,
    EXPRESSIONS,
    FAULTS_STOP_AI,
    FEATURE,
    FEATURES,
    FIND_SHAPES,
    FIND_STALE,
    FLY_RPM_MAX,
    FW_LIMITS,
    HUMAN_HOLD,
    HUMAN_VERBS,
    KID_SPEED,
    LAUNCHER_RPMS,
    MANUAL_TIMEOUT,
    MAP_NAME,
    MISSIONS,
    MISSION_FEATURE,
    MODE_OF,
    NAME,
    NAV_MISSIONS,
    NEW_MAP,
    Reply,
    SAY_MAX,
    SETTING,
    SETTINGS,
    Setting,
    TILT_RANGE,
    TRICKS,
    VERB_FEATURE,
    VERBS,
    VERSION,
    WORKSPACES,
    ZERO_TAIL,
    _LAUNCH,
    _name,
    _points,
    _program_check,
    launch_limits,
    shape,
)
from omni_console.console_ai import AiMixin
from omni_console.console_calib import CalibMixin
from omni_console.console_edit import EditMixin
from omni_console.console_fun import FUN_MISSIONS, FunMixin


class Console(AiMixin, CalibMixin, EditMixin, FunMixin):
    def __init__(self, settings: dict | None = None, places: dict | None = None, routes: dict | None = None,
                 programs: dict | None = None, calib: dict | None = None, throws=None, health=None,
                 ai_tasks: dict | None = None, bringup: dict | None = None):
        from omni_console.ai_tasks import TaskBook
        from omni_console.health import Health
        from omni_console.throws import ThrowLog
        self.values = {s.key: s.default for s in SETTINGS}
        for k, v in (settings or {}).items():
            if k in SETTING:
                try:
                    self.values[k] = SETTING[k].coerce(v)
                except (TypeError, ValueError):
                    pass
        self.places, self.routes = {}, {}
        self.zones, self.edges, self.rooms = {}, {}, {}
        self.tf = None                     # the node's TfLite (lookup, frames): the tf verb's read-only answers
        self.set_map(None, places, routes)
        self.programs = {}
        for n, b in (programs or {}).items():
            try:
                self.programs[str(n)] = _program_check(b)
            except (TypeError, ValueError):
                pass
        self.calib_results = {k: list(v) for k, v in (calib or {}).items() if isinstance(v, list)}
        self.calib_run = None              # calibration.DriveCal while CALIBRATE runs
        self.calib_shots = []              # the launcher calibration's shots so far
        self.throws = throws or ThrowLog()
        self.health = health or Health()
        self.follower = None               # people.Follower while FOLLOW runs
        self.follow_cmd, self.t_follow = (0.0, 0.0, 0.0), -math.inf
        self.runner = None                 # program.ProgramRunner while PROGRAM runs
        self.odom_pose = None              # (x, y, yaw) in odom, from the node (programs, calibrations, follow)
        self.mcu = None                    # the MCU's summary (status.mcu_summary), from the node
        self.slow = None                   # (speed, edge) while the robot is in an edge's slow band
        self.loc_health = None             # omni_slam's /localization/health (design.md 10), and when it came
        self.t_loc_health = -1e9
        self.fx_queue = []                 # effects that come up outside a command (a program's goto)
        self.last_cmd = None               # the twist drive_output last gave (after the slow bands)
        self.programs_changed = self.calib_changed = self.zones_changed = False
        self.mission = "IDLE"
        self.adopted = False               # the first /robot_mode sets the mission (a step may start in CATCH)
        self.robot_mode = None
        self.robot_kid = None
        self.pending_mode = None           # a set_mode on its way
        self.task = "Waiting for the robot"
        self.progress = None
        self.goal = None                   # dict(x, y, yaw, name, seq): the Nav2 goal of GOTO / EXPLORE / PATROL
        self.goal_seq = 0
        self.d0 = None
        self.caps = {"nav": False, "catch": False, "collision": False, "arm": False, "sim": False}
        self.arm, self.t_arm = None, -math.inf     # omni_arm's last /arm/state, and when it came
        self.arm_ev = 0                            # the last arm event passed on
        self.slam = dict(mode=None, map=None, managed=False, maps=[], busy=False)
        self.pose = None                   # (frame, x, y, yaw), from the node
        self.explore = None                # dict: phase, start, t0, blacklist, info, t_plan, t_goal
        self.patrol = None                 # dict: route, points, i, step, lap, laps, dwell_until, fails
        self.thrower = dict(rpm=0.0, tilt=0.0, seq=0)
        self.manual_in = None              # (vx, vy, wz, boost) normalized to -1..1
        self.t_manual = -math.inf
        self.now = 0.0
        self.events = deque(maxlen=200)
        self.settings_changed = self.places_changed = False
        self.ai = None                     # the AI client's lease: dict(session, until, ttl)
        self.ai_book = TaskBook(ai_tasks)  # its tasks (the node saves them: ai_tasks.json)
        self.human_t = -math.inf           # when a person last took the robot back (the AI waits HUMAN_HOLD)
        self.rc_manual = False             # the RC transmitter's manual switch (the node, from /mcu/info)
        self.zero_until = -math.inf        # zero twist until then, after a driving mission stops (a clean stop)
        self.found, self.t_found = None, -math.inf     # omni_arm's last find, and when it came
        self.loc_epoch = self.found_epoch = 0          # SLAM changes since start; the find's (older: stale)
        # the Test workspace (checks.py): the node's latest preflight rows, the bring-up results, what a result records
        self.preflight = None              # list of checks.preflight rows, or None (no status assessed yet)
        self.bringup = {k: list(v) for k, v in (bringup or {}).items() if isinstance(v, list)}
        self.bringup_changed = False
        self.bringup_new = []              # records to log (the node writes them to the session log)
        self.robot_config = {}             # from the node: source, firmware build, host (stored with each result)
        # the activity history: every task that ran, whoever started it (a person on the page, the voice, the AI)
        self._fun_init()                   # Fetch, Messenger, Block tower, Dance party, Goalie (console_fun.py)
        self.activity = deque(maxlen=50)
        self.activity_n = 0                # bumped on every change: the page asks for the list again
        self.act = None                    # the running activity record
        self.who = "console"               # who sent the command being handled: console | voice | ai | ...
        self.last_level = "info"           # the last event's level: how a task that ended by itself went

    def set_map(self, name, places: dict | None, routes: dict | None, zones: dict | None = None,
                edges: dict | None = None, rooms: dict | None = None):
        """The map changed (another map loaded, or a new one named): its own places, routes, no-go zones, edges and
        rooms (named outlines, map frame: what "the kitchen" means; the AI reads them, nothing drives by them)."""
        from omni_console.zones import check_edge, check_zone
        self.map_name = name
        self.places, self.routes, self.zones, self.edges, self.rooms = {}, {}, {}, {}, {}
        for n, z in (rooms or {}).items():
            try:
                self.rooms[str(n)] = check_zone(z)
            except (TypeError, ValueError, IndexError):
                pass
        for n, p in (places or {}).items():
            try:
                self.places[str(n)] = [float(p[0]), float(p[1]), float(p[2])]
            except (TypeError, ValueError, IndexError):
                pass
        for n, r in (routes or {}).items():
            try:
                self.routes[str(n)] = _points(r)
            except (TypeError, ValueError, IndexError):
                pass
        for n, z in (zones or {}).items():
            try:
                self.zones[str(n)] = check_zone(z)
            except (TypeError, ValueError, IndexError):
                pass
        for n, e in (edges or {}).items():
            try:
                self.edges[str(n)] = check_edge(e["points"], e.get("height_mm", 10), e.get("speed", 1.0),
                                                e.get("kind", "rug"))
            except (TypeError, ValueError, IndexError, KeyError):
                pass
        self.zones_changed = True

    def extras(self) -> dict:
        """What <map>.places.json keeps."""
        return dict(places=self.places, routes=self.routes, zones=self.zones, edges=self.edges, rooms=self.rooms)

    # -- events ---------------------------------------------------------------------------------------------------
    def event(self, level: str, text: str):
        self.events.append((level, text))
        self.last_level = level

    def pop_events(self) -> list:
        out = list(self.events)
        self.events.clear()
        return out

    # -- commands ---------------------------------------------------------------------------------------------
    def command(self, verb: str, args: dict | None = None) -> Reply:
        """A verb from the page, the voice (omni_face) or an AI client (args["ai"]: its session). A person's verb that
        moves the robot takes it back from the AI first (its task cancelled, its mission stopped)."""
        a = args or {}
        self.who = "ai" if a.get("ai") else re.sub(r"[^a-z_]", "", str(a.get("by") or "console").lower())[:16] or "console"
        bad = self.gate(verb, a)
        if bad is not None:
            self.who = "console"
            return bad
        pre = ()
        if not a.get("ai") and verb in HUMAN_VERBS:
            pre = self._ai_takeover(f"a person took over ({verb} from the {self.who})")
        r = self._command(verb, a)
        self.activity_sync()
        self.who = "console"
        return Reply(r.ok, r.message, r.data, pre + r.effects) if pre else r

    # -- the preflight's gate (checks.py) --------------------------------------------------------------------------
    def gate(self, verb: str, a: dict) -> Reply | None:
        """A start refused by a failing preflight check: the check's own words. Stop, cancel and the arm's hold and
        torque-off are never refused; neither is driving by hand (MANUAL)."""
        from omni_console.checks import blocking
        if not self.preflight:
            return None
        if verb == "mission":
            fid = MISSION_FEATURE.get(str(a.get("name", "")).upper())
        elif verb == "arm":
            fid = "arm" if str(a.get("do", "")) in ("pose", "gripper", "jog") else None
        else:
            fid = VERB_FEATURE.get(verb)
        if fid is None:
            return None
        rows = blocking(self.preflight, fid, bool(FEATURE.get(fid, {}).get("moves")))
        if not rows:
            return None
        why = "; ".join(f"{r['label']}: {r['detail']}" for r in rows)
        return Reply(False, f"Preflight refuses {FEATURE[fid]['label']}: {why}",
                     dict(error="fault", preflight=[r["id"] for r in rows]))

    # -- the activity history: what ran, who started it, how it ended --------------------------------------------
    def _act_open(self):
        if self.mission in ("IDLE", "MANUAL"):
            return
        t = self.ai_book.running()
        self.act = dict(n=self.activity_n + 1, mission=self.mission, label=self.task, who=self.who if t is None else "ai",
                        task_id=None if t is None else t.id, t0=round(self.wall_time(), 1), t1=None, state="running",
                        result="")
        self.activity.append(self.act)
        self.activity_n += 1

    def _act_close(self, state: str, result: str):
        if self.act is None:
            return
        self.act.update(t1=round(self.wall_time(), 1), state=state, result=result)
        self.act = None
        self.activity_n += 1

    def activity_sync(self):
        """A task that ended by itself (Nav2 arrived, the tidy finished, the robot was switched elsewhere): closed with
        how its last message went; a task that started outside a command (the robot came up in CATCH): opened."""
        if self.act is not None and self.act["mission"] != self.mission:
            self._act_close("done" if self.last_level == "info" else "failed", self.task)
        if self.act is None and self.mission not in ("IDLE", "MANUAL"):
            self._act_open()

    # -- who has the robot, and how fast it may go ---------------------------------------------------------------
    CONTROL_LADDER = (("estop", "E-stop"), ("rc", "RC switch"), ("stick", "Stick / gamepad"),
                      ("task", "Task (Nav2, Tidy, Follow ...)"), ("ai", "AI"))

    def control(self, now: float) -> dict:
        """Who drives the robot now, highest first: the hardware/firmware e-stop, the RC's MANUAL switch, a person's
        stick, a task (a mission), the AI (its own task, the lowest: any person's command takes it back)."""
        m = self.mcu or {}
        t = self.ai_book.running()
        if m.get("mode") == "ESTOP":
            who, text = "estop", "E-stop: motors off"
        elif self.rc_manual:
            who, text = "rc", "RC transmitter (MANUAL switch)"
        elif self.manual_active(now):
            who, text = "stick", "Stick / gamepad on the console"
        elif t is not None:
            who, text = "ai", f"AI: {t.kind.replace('_', ' ')}"
        elif self.mission in NAV_MISSIONS:
            who, text = "task", f"Nav2 ({self.mission.title().replace('Goto', 'Go To')})"
        elif self.mission not in ("IDLE", "MANUAL"):
            who, text = "task", self.mission.title()
        elif self.mission == "MANUAL":
            who, text = "ready", "Console: ready to drive"
        else:
            who, text = "idle", "Nobody: motors off"
        lease = self.ai is not None and now <= self.ai["until"]
        hold = max(0.0, HUMAN_HOLD - (now - self.human_t)) if lease else 0.0     # only news while an AI is connected
        return dict(who=who, text=text, ladder=[dict(id=k, label=lbl) for k, lbl in self.CONTROL_LADDER],
                    ai_hold_s=round(hold, 1) if hold > 0 else None, ai_lease=lease)

    def speed_cap(self) -> dict | None:
        """The top speed the robot may drive at now, and why (the lowest of the firmware's profile, the task's own
        top speed, the arm being out, a slow band on the map)."""
        if self.mission == "IDLE":
            return None
        v = self.values
        kid = bool(v["profile.kid"])
        caps = [(FW_LIMITS["kid" if kid else "adult"][0], "kid profile" if kid else "firmware limit")]
        own = {"MANUAL": ("manual.max_speed", "Drive's top speed"), "GOTO": ("goto.speed", "Go To's speed"),
               "EXPLORE": ("explore.speed", "Explore's speed"), "PATROL": ("patrol.speed", "Patrol's speed"),
               "PROGRAM": ("program.speed", "Program's speed"), "DANCE": ("dance.speed", "the dance's moves"),
               "GOALIE": ("goalie.speed", "Goalie's dive speed")}
        if self.ai_book.running() is not None:
            caps.append((v["ai.max_speed"], "AI top speed"))
        elif self.mission == "FOLLOW":
            tag = self.follower is not None and self.follower.mode == "tag"
            caps.append((v["tag.speed" if tag else "follow.speed"], "Tag speed" if tag else "Follow's speed"))
        elif self.mission in own:
            k, why = own[self.mission]
            caps.append((v[k], why))
        if self._arm_up() and self.arm.get("out"):
            caps.append((1.0, "arm out"))
        if self.slow is not None:
            caps.append((float(self.slow[0]), f"slow band: {self.slow[1]}"))
        speed, why = min(caps, key=lambda c: c[0])
        return dict(speed=round(float(speed), 2), reason=why)

    def _command(self, verb: str, a: dict) -> Reply:
        ai = str(a.get("ai") or "")
        try:
            if verb == "ping":
                return Reply(True, "pong", {"version": VERSION})
            if verb == "schema":
                return Reply(True, "", self.schema())
            if verb == "stop":
                if ai:
                    self._ai_end_running("CANCELLED", "stopped by the AI client (robot_stop)")
                return self._enter("IDLE", "Stopped by the AI client" if ai else "Stopped from the console", "warn")
            if verb == "mission":
                return self._mission(str(a.get("name", "")).upper())
            if verb == "goto":
                if ai:
                    return self._ai_goto(a)
                return self._goto(float(a["x"]), float(a["y"]), float(a.get("yaw", 0.0)), str(a.get("name", "")))
            if verb == "ai_lease":
                return self._ai_lease(a)
            if verb == "move":
                return self._ai_move(a)
            if verb == "find":
                return self._ai_find(a)
            if verb == "pick":
                return self._ai_pick(a)
            if verb == "task":
                return self._ai_task(a)
            if verb == "look":
                return self._look(a)
            if verb == "cancel":
                if ai or "task_id" in a:
                    return self._ai_cancel(a)
                if self.mission not in NAV_MISSIONS:
                    return Reply(True, "Nothing to cancel")
                return self._enter("MANUAL", f"{self.mission.title()} cancelled", "info")
            if verb == "set":
                return self._set(str(a["key"]), a["value"])
            if verb == "place_save":
                name = str(a["name"]).strip()
                if not name:
                    return Reply(False, "A place needs a name")
                self.places[name] = [float(a["x"]), float(a["y"]), float(a.get("yaw", 0.0))]
                self.places_changed = True
                return Reply(True, f"Saved '{name}'", self.schema())
            if verb == "place_delete":
                name = str(a["name"])
                if self.places.pop(name, None) is None:
                    return Reply(False, f"No place called '{name}'")
                self.places_changed = True
                return Reply(True, f"Deleted '{name}'", self.schema())
            if verb == "route_save":
                name = str(a["name"]).strip()
                pts = _points(a["points"])
                if not name or len(pts) < 2:
                    return Reply(False, "A route needs a name and at least two points")
                self.routes[name] = pts
                self.places_changed = True
                return Reply(True, f"Saved route '{name}' ({len(pts)} points)", self.schema())
            if verb == "route_delete":
                name = str(a["name"])
                if self.routes.pop(name, None) is None:
                    return Reply(False, f"No route called '{name}'")
                self.places_changed = True
                return Reply(True, f"Deleted route '{name}'", self.schema())
            if verb == "explore":
                return self._explore(bool(a.get("new_map", False)))
            if verb == "patrol":
                return self._patrol(str(a["route"]))
            if verb == "thrower":
                return self._thrower(a)
            if verb == "fire":
                if self.mission != "THROWER":
                    return Reply(False, "Fire works in the Thrower tab's bench mode: press 'Take control' there")
                launch_angle, lowest = launch_limits()
                if launch_angle - self.thrower["tilt"] < lowest - 1e-6:
                    return Reply(False, f"Tilt {self.thrower['tilt']:+.0f}° launches at "
                                        f"{launch_angle - self.thrower['tilt']:.0f}°: the face is in the way below "
                                        f"{lowest:g}°. Tilt {launch_angle - lowest:+g}° or less to fire")
                self.thrower["seq"] = (self.thrower["seq"] + 1) & 0xFFFF
                self.event("info", "Fire requested (the firmware fires if the flywheels are at speed)")
                return Reply(True, "Fire requested")
            if verb in ("slam_new", "slam_save", "slam_load", "initial_pose"):
                return self._slam(verb, a)
            if verb == "follow":
                return self._follow(str(a.get("mode", "follow")))
            if verb.startswith("program_"):
                return self._program(verb, a)
            if verb in ("zone_save", "zone_delete", "edge_save", "edge_delete"):
                return self._zones(verb, a)
            if verb in ("room_save", "room_delete"):
                return self._rooms(verb, a)
            if verb in ("tf", "frames"):
                return self._tf(verb, a)
            if verb.startswith("calib"):
                return self._calib(verb, a)
            if verb == "player_set":
                name = self.throws.set_player(a["name"])
                return Reply(True, f"Now throwing: {name}", self.throws.summary())
            if verb == "stats":
                return Reply(True, "", self.throws.summary())
            if verb == "health":
                return Reply(True, "", self.health.report(int(a.get("step", 1)), float(a.get("since", -math.inf))))
            if verb == "maint_done":
                self.health.done(str(a["item"]))
                return Reply(True, f"Marked '{a['item']}' done", self.health.report(1, math.inf))
            if verb == "say":
                return self._say(a)
            if verb == "hear":
                text = str(a.get("text", "")).strip()[:SAY_MAX]
                if not text:
                    return Reply(False, "Type what the robot should take as heard")
                return Reply(True, f"Heard (typed): {text}", None, (("face_hear", text),))
            if verb == "preflight":
                return Reply(True, "", dict(rows=self.preflight or [], assessed=self.preflight is not None))
            if verb == "tests":
                from omni_console.checks import BRINGUP
                return Reply(True, "", dict(checks=list(BRINGUP), results=self.bringup))
            if verb == "test_record":
                from omni_console.checks import BRINGUP, BRINGUP_IDS, record
                cfg = dict(self.robot_config, console=VERSION, map=self.map_name, kid=bool(self.values["profile.kid"]))
                rec = record(self.bringup, a, self.wall_time(), cfg)
                self.bringup_changed = True
                self.bringup_new.append(rec)
                title = BRINGUP[BRINGUP_IDS.index(rec["id"])]["title"]
                self.event("info" if rec["result"] != "fail" else "warn", f"Bring-up '{title}': {rec['result']}")
                return Reply(True, f"Recorded: {title} {rec['result']}", dict(results=self.bringup))
            if verb == "activity":
                return Reply(True, "", dict(n=self.activity_n, items=list(reversed(self.activity))))
            if verb == "tidy":
                return self._tidy()
            if verb == "fetch":
                return self._fetch(a)
            if verb == "message":
                return self._message(a)
            if verb == "tower":
                return self._tower(a)
            if verb == "dance":
                return self._dance(a)
            if verb == "goalie":
                return self._goalie(a)
            if verb == "ai_mission":
                return self._ai_mission(a)
            if verb == "clip":
                from omni_console.console_defs import CLIP_NAME
                name = str(a.get("name", ""))
                if not CLIP_NAME.match(name):
                    return Reply(False, "clip: a name the page server gave (/api/clip)")
                caption = re.sub(r"[\x00-\x1f]+", " ", str(a.get("caption") or "(caller)"))[:80]
                self.event("info", f"Playing a clip: {caption}")
                return Reply(True, "Playing it on the robot", None, (("face_clip", name, caption),))
            if verb == "arm":
                return self._ai_arm(a) if ai else self._arm(a)
        except (KeyError, TypeError, ValueError, IndexError) as e:
            return Reply(False, f"{verb}: {e}" if isinstance(e, ValueError) else f"{verb}: bad arguments ({e})",
                         {"error": "bad_request"})
        return Reply(False, f"Unknown command '{verb}' (one of {', '.join(VERBS)})", {"error": "bad_request"})

    # -- the face and the voice (omni_face) --------------------------------------------------------------------
    def _say(self, a: dict) -> Reply:
        """The Say box: a line for the robot to say, with an expression; or a trick (an antic) on its own."""
        text = re.sub(r"[\x00-\x1f]+", " ", str(a.get("text", ""))).strip()
        if len(text) > SAY_MAX:
            return Reply(False, f"Up to {SAY_MAX} characters at once")
        expr = str(a.get("expression", "auto"))
        if expr not in EXPRESSIONS:
            return Reply(False, f"expression: one of {', '.join(EXPRESSIONS)}")
        trick = a.get("act")
        if trick is not None and trick not in TRICKS:
            return Reply(False, f"trick: one of {', '.join(TRICKS)}")
        if not text and trick is None:
            return Reply(False, "Type something for the robot to say")
        mood, act = EXPRESSIONS[expr]
        req = dict(text=text, mood=mood, act=trick or act)
        if text:
            self.event("info", f"Robot says: {text}")
        return Reply(True, f"Saying: {text}" if text else f"Trick: {trick}", None, (("face", req),))

    # -- the arm (design.md 11) -----------------------------------------------------------------------------------
    def _arm_up(self) -> bool:
        return self.arm is not None and self.now - self.t_arm < ARM_STALE and bool(self.arm.get("connected"))

    def _tidy(self) -> Reply:
        if not self._arm_up():
            return Reply(False, "Tidy needs the arm robot: start 'Arm robot: tidy up' (in the sim) or the arm node")
        v = self.values
        r = self._enter("TIDY", "Tidying up: looking for things on the floor", "info")
        req = dict(verb="tidy", max_objects=int(v["tidy.max_objects"]), radius=float(v["tidy.radius"]))
        return Reply(True, r.message, None, r.effects + (("arm", req),))

    def _arm(self, a: dict) -> Reply:
        if not self._arm_up():
            return Reply(False, "The arm isn't connected")
        if self.mission == "TIDY":
            return Reply(False, "The tidy runs the arm: stop it first")
        what = str(a.get("do", ""))
        if what == "pose":
            name = str(a.get("name", ""))
            if name not in ARM_POSES:
                return Reply(False, f"arm pose: one of {', '.join(ARM_POSES)}")
            return Reply(True, f"Arm to {name}", None, (("arm", dict(verb="pose", name=name)),))
        if what == "gripper":
            gap = float(a.get("gap_mm", 0.0)) / 1000.0
            if not 0.0 <= gap <= 0.085:
                return Reply(False, "gripper: 0 to 85 mm")
            return Reply(True, f"Gripper to {1000 * gap:.0f} mm", None, (("arm", dict(verb="gripper", gap=gap)),))
        if what == "jog":
            joint, names = str(a.get("joint", "")), list(self.arm.get("names") or ())
            if joint not in names:
                return Reply(False, f"arm joint: one of {', '.join(names)}")
            deg = float(a.get("deg", 0.0))
            if not 0.0 < abs(deg) <= ARM_JOG_MAX:
                return Reply(False, f"jog: up to {ARM_JOG_MAX:.0f}° at a time")
            return Reply(True, f"{joint.replace('_', ' ')} {deg:+g}°", None,
                         (("arm", dict(verb="jog", joint=joint, delta=math.radians(deg))),))
        if what in ("hold", "relax"):
            text = "Arm holding" if what == "hold" else "Arm torque off: it drops, hold it"
            return Reply(True, text, None, (("arm", dict(verb=what)),))
        return Reply(False, "arm: do = pose | gripper | jog | hold | relax")

    # -- the AI client: console_ai.AiMixin; programs, zones, rooms, tf: console_edit.EditMixin; calibration:
    #    console_calib.CalibMixin -----------------------------------------------------------------------------


    def on_arm_state(self, s: dict, now: float):
        """omni_arm's /arm/state (5 Hz). A tidy that ended by itself ends the mission; its events become ours."""
        was = self.arm
        self.arm, self.t_arm = s, now
        self.now = now
        new = []
        for ev in s.get("events", []):
            if ev.get("n", 0) > self.arm_ev:
                self.arm_ev = ev["n"]
                if was is not None:                       # not the backlog of a node that was already running
                    self.event("info", f"Arm: {ev['text']}")
                    new.append(str(ev.get("text", "")))
        self._ai_on_arm(s, new)
        fun = self.fun_on_arm(s, was)
        if fun:
            return tuple(fun)
        t = s.get("tidy") or {}
        if self.mission == "TIDY":
            st = t.get("state", "")
            self.task = f"Tidying: {st.lower().replace('_', ' ')} ({t.get('picked', 0)} in the bin)" if st else self.task
            if was is not None and was.get("tidy_on") and not s.get("tidy_on"):
                self._ai_pick_end(t)
                self.mission, self.progress = "MANUAL", None
                self.task = (f"Tidy done: {t.get('picked', 0)} in the bin, {t.get('failed', 0)} not picked up"
                             + (f" ({t['reason']})" if t.get("reason") else ""))
                self.event("info", self.task)
                self.activity_sync()
                return (("set_mode", "TELEOP", self.values["profile.kid"]),)
        return ()

    def _mission(self, name: str) -> Reply:
        if name == "GOTO":
            return Reply(False, "Pick a goal on the map (Go To tab) to start Go To")
        if name == "TIDY":
            return self._tidy()
        if name in ("EXPLORE", "PATROL", "FOLLOW", "PROGRAM", "CALIBRATE"):
            return Reply(False, f"Start {name.title()} from its tab")
        if name not in MISSIONS:
            return Reply(False, f"Unknown mode '{name}'")
        if name == "CATCH" and not self.caps["catch"]:
            return Reply(False, "Catch needs the catch code running: in the sim, Home > 'Switch to the catch world' "
                                "(or 'Play catch' in the Launcher)")
        if name == "THROWER":
            self.thrower = dict(rpm=0.0, tilt=0.0, seq=0)
        text = {"IDLE": "Motors off", "MANUAL": "Driving by hand", "CATCH": "Playing catch",
                "THROWER": "Thrower bench test: base parked"}[name]
        return self._enter(name, text, "info")

    def _nav_goal(self, x, y, yaw, name, speed, turn_deg) -> list:
        """Effects for a new Nav2 goal (replacing any), and the goal it becomes."""
        fx = [("nav_cancel",)] if self.goal is not None else []
        self.goal_seq += 1
        self.goal = dict(x=x, y=y, yaw=yaw, name=name, seq=self.goal_seq)
        self.d0 = None
        fx.append(self._nav_limits(speed, turn_deg))
        fx.append(("nav_goal", x, y, yaw, self.goal_seq))
        return fx

    def _nav_limits(self, speed, turn_deg) -> tuple:
        """Nav2's top speed and turn rate, and which way the robot faces on the way (omni_bringup heading_node).
        Explore never spins: a spinning lidar maps worse."""
        facing = self.values["goto.facing"]
        if facing == "spinning" and self.mission == "EXPLORE":
            facing = "forward"
        return "nav_limits", speed, math.radians(turn_deg), facing, math.radians(self.values["goto.spin"])

    def _need_nav(self) -> Reply | None:
        if not self.caps["nav"]:
            return Reply(False, "This needs navigation running: start 'Robot (all modes)' or 'Navigate (Nav2)' in "
                                "the Launcher")
        return None

    def _goto(self, x: float, y: float, yaw: float, name: str) -> Reply:
        bad = self._need_nav()
        if bad:
            return bad
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return Reply(False, "The goal is not a number")
        label = f"'{name}'" if name else f"({x:.1f}, {y:.1f})"
        fx = list(self._enter("GOTO", f"Going to {label}", "info", cancel_nav=False).effects)
        fx += self._nav_goal(x, y, yaw, name, self.values["goto.speed"], self.values["goto.turn"])
        return Reply(True, f"Going to {label}", None, tuple(fx))

    def _explore(self, new_map: bool) -> Reply:
        bad = self._need_nav()
        if bad:
            return bad
        fx = []
        if new_map:
            if not self.slam["managed"]:
                return Reply(False, "A new map needs the console to run SLAM: start 'Robot (all modes)'")
            fx.append(("slam_start", "mapping", None, None))
            self.slam.update(mode=None, busy=True)
        elif self.slam["mode"] != "mapping":
            return Reply(False, "Exploring builds a map: press 'New map + explore' (or map in the SLAM tab first)")
        fx = list(self._enter("EXPLORE", "Exploring: waiting for the map", "info").effects) + fx
        self.explore = dict(phase="wait", start=None, t0=self.now, blacklist=[], info={}, t_plan=0.0, t_goal=0.0)
        return Reply(True, "Exploring", None, tuple(fx))

    def _patrol(self, route: str) -> Reply:
        bad = self._need_nav()
        if bad:
            return bad
        pts = self.routes.get(route)
        if not pts:
            return Reply(False, f"No route called '{route}'")
        r = self._enter("PATROL", f"Patrol '{route}'", "info")
        laps = int(self.values["patrol.laps"])
        self.patrol = dict(route=route, points=pts, i=0, step=1, lap=1, laps=laps, dwell_until=0.0, fails=0)
        return Reply(True, f"Patrolling '{route}'", None, r.effects)

    def _thrower(self, a: dict) -> Reply:
        if self.mission != "THROWER":
            return Reply(False, "Press 'Take control' in the Thrower tab first")
        if "rpm" in a:
            rpm = float(a["rpm"])
            if not 0.0 <= rpm <= FLY_RPM_MAX:
                return Reply(False, f"Flywheels: 0 to {FLY_RPM_MAX:g} rpm")
            self.thrower["rpm"] = rpm
        if "tilt" in a:
            t = float(a["tilt"])
            if not TILT_RANGE[0] <= t <= TILT_RANGE[1]:
                return Reply(False, f"Tilt: {TILT_RANGE[0]:g} to {TILT_RANGE[1]:g} deg")
            self.thrower["tilt"] = t
        launch_angle, lowest = launch_limits()
        el = launch_angle - self.thrower["tilt"]
        return Reply(True, f"Flywheels {self.thrower['rpm']:.0f} rpm, tilt {self.thrower['tilt']:.0f} deg (launch "
                           f"{el:.0f} deg" + (f": too flat to fire, {lowest:g} deg at least)" if el < lowest else ")"))

    def thrower_cmd(self):
        """(tilt rad, flywheel rpm, fire_seq) for /thrower/cmd while THROWER runs, else None."""
        if self.mission != "THROWER":
            return None
        t = self.thrower
        return math.radians(t["tilt"]), t["rpm"], t["seq"]

    # -- Follow (Program, Map edit, Calibrate: the mixins) ---------------------------------------------------------
    def _follow(self, mode: str) -> Reply:
        from omni_console.people import Follower
        if mode not in ("follow", "tag"):
            return Reply(False, "follow: mode 'follow' or 'tag'")
        v = self.values
        r = self._enter("FOLLOW", "Follow me: looking for a person" if mode == "follow" else "Tag: looking for a person",
                        "info")
        speed = min(v["tag.speed"], KID_SPEED) if mode == "tag" else v["follow.speed"]
        self.follower = Follower(mode, v["follow.distance"], speed)
        self.follow_cmd, self.t_follow = (0.0, 0.0, 0.0), self.now
        return Reply(True, r.message, None, r.effects)

    def on_people(self, people, now: float):
        """The node's people (base frame, people.detect) from each lidar scan while FOLLOW, MESSAGE or FETCH runs."""
        self.fun_people(people, now)
        if self.mission != "FOLLOW" or self.follower is None:
            return
        f = self.follower
        self.follow_cmd = f.tick(now, people, self.odom_pose)
        self.t_follow = now
        self.task = f.text()
        for e in f.events:
            self.event("info", e)
        f.events.clear()


    # -- the driving missions' twist ------------------------------------------------------------------------------
    def drive_output(self, now: float):
        """(vx, vy, wz) to send now for MANUAL, FOLLOW, PROGRAM, CALIBRATE, or None (send nothing). Translation is
        capped in an edge's slow band (zones.py)."""
        from omni_console.zones import speed_cap
        self.now = now
        self.fx_queue += self._ai_tick(now)
        tw = None
        if self.mission == "MANUAL":
            tw = self.manual_output(now)
        elif self.mission == "FOLLOW" and self.follower is not None:
            tw = self.follow_cmd if now - self.t_follow < 0.5 else (0.0, 0.0, 0.0)      # no scan: stop
        elif self.mission in FUN_MISSIONS:
            tw = self.fun_twist(now)
        elif self.mission == "PROGRAM" and self.runner is not None:
            p = self.runner
            tw = p.tick(now, self.odom_pose)
            place = p.wants_goto()
            if place is not None:
                x, y, yaw = self.places[place]
                self.fx_queue += self._nav_goal(x, y, yaw, place, self.values["goto.speed"], self.values["goto.turn"])
            if tw == "done":
                name, err = p.name, p.error
                t = self.ai_book.running(("move",))
                if t is not None:
                    m = p.measured()
                    self.ai_book.finish(t, "FAILED" if err else "SUCCEEDED", self.wall_time(),
                                        f"stopped: {err}" if err else
                                        f"done: turned {m['turned_deg']:.0f} deg, drove {m['distance_m']:.2f} m in "
                                        f"{m['duration_s']:.1f} s", result=dict(error="timeout") if err else None,
                                        evidence=m)
                self.runner = None
                self.mission, self.progress = "MANUAL", None
                self.zero_until = now + ZERO_TAIL
                self.task = f"Program '{name}' stopped: {err}" if err else f"Program '{name}' finished"
                self.event("warn" if err else "info", self.task)
                tw = (0.0, 0.0, 0.0)
            else:
                self.task = f"Program '{p.name}': {p.label()}"
                self.progress = max(0, p.i) / max(1, len(p.steps))
        elif self.mission == "CALIBRATE" and self.calib_run is not None:
            c = self.calib_run
            tw = c.tick(now, self.odom_pose, self.mcu)
            self.task = f"Calibrating ({c.proc}): {c.text()}"
            if c.phase == "done" and c.result is not None and c.proc == "imu_level" and not getattr(c, "saved", False):
                c.saved = True
                self._calib_save(c.proc, c.result)
        if tw is None and self.mission in ("MANUAL", "IDLE") and now < self.zero_until:
            tw = (0.0, 0.0, 0.0)                         # a driving mission just stopped: hold zero a moment
        self.slow = None
        if tw is not None and self.pose is not None and self.pose[0] == "map" and self.edges:
            cap, e = speed_cap(self.edges.values(), self.pose[1], self.pose[2])
            if cap is not None:
                name = next((n for n, x in self.edges.items() if x is e), "edge")
                self.slow = (cap, name)
                v = math.hypot(tw[0], tw[1])
                if v > cap:
                    tw = (tw[0] * cap / v, tw[1] * cap / v, tw[2])
        # the robot's own driving slows with its localization (omni_slam's health); a person at the sticks doesn't
        h = self.loc_health_now()
        if tw is not None and h is not None and self.mission in ("FOLLOW", "PROGRAM", "CALIBRATE"):
            k = float(h.get("speed_scale", 1.0))
            if k < 1.0:
                tw = (tw[0] * k, tw[1] * k, tw[2] * k)
        self.last_cmd = tw
        return tw

    def _person_rel(self):
        """The followed person as [distance m, bearing rad] from the robot (the page places it on its map)."""
        f, p = self.follower, self.odom_pose
        if f is None or f.target is None or p is None:
            return None
        dx, dy = f.target[0] - p[0], f.target[1] - p[1]
        return [round(math.hypot(dx, dy), 2), round(math.atan2(dy, dx) - p[2], 3)]

    def on_mcu(self, mcu: dict | None):
        """The MCU's summary (status.mcu_summary), from the node at 20 Hz. A fault that appears is said once, with its
        hint, and so is its clearing; an e-stop or the MCU's fault stop ends the mission (the robot doesn't re-arm by
        itself); an MCU that goes quiet (the link) is said once."""
        prev, self.mcu = self.mcu, mcu
        if mcu is None:
            return
        hints = {f["name"]: f["hint"] for f in mcu.get("faults", [])}
        old = {f["name"] for f in (prev or {}).get("faults", [])}
        for n in sorted(set(hints) - old):
            serious = n in ("ESTOP", "LINK_LOST", "MOTOR_SILENT", "MOTOR_ERR", "LOW_BATT", "FAULT_STOP")
            self.event("error" if serious else "warn", f"{n}: {hints[n]}")
        for n in sorted(old - set(hints)):
            self.event("info", f"{n} cleared")
        quiet, was = mcu.get("age", 0.0) > 1.0, (prev or {}).get("age", 0.0) > 1.0
        if quiet and not was:
            self.event("error", "The MCU went quiet (no state for 1 s): the link is down. The MCU stops the robot "
                                "by itself after 0.5 s without commands.")
        elif was and not quiet:
            self.event("info", "The MCU is back")
        if mcu.get("mode") == "ESTOP" and (prev or {}).get("mode") != "ESTOP" and self.mission != "IDLE":
            r = self._enter("IDLE", "E-stop: motors off. Release it, then start a mode again", "error")
            self.fx_queue += list(r.effects)
        elif "FAULT_STOP" in hints and "FAULT_STOP" not in old and self.mission != "IDLE":
            # the MCU stopped the base (firmware.h's fault stop); IDLE's DISARM request is what lets it re-arm
            r = self._enter("IDLE", "The MCU stopped the base on a fault: fix it, then start a mode again", "error")
            self.fx_queue += list(r.effects)

    def pop_fx(self) -> list:
        fx, self.fx_queue = self.fx_queue, []
        return fx

    def _slam(self, verb: str, a: dict) -> Reply:
        s = self.slam
        if verb in ("slam_new", "slam_load", "initial_pose"):
            self.loc_epoch += 1                           # the AI's last find no longer counts (odom may jump)
        if verb == "initial_pose":
            x, y, yaw = float(a["x"]), float(a["y"]), float(a.get("yaw", 0.0))
            if s["mode"] != "localization":
                return Reply(False, "Setting the position is for localizing on a saved map (SLAM tab: Load)")
            self.event("info", f"Robot placed at ({x:.1f}, {y:.1f})")
            return Reply(True, "Position set: the scan matcher refines it as the robot moves", None,
                         (("initial_pose", x, y, yaw),))
        if verb == "slam_save":
            name = str(a["name"]).strip()
            if not MAP_NAME.match(name):
                return Reply(False, "Map names: letters, digits, - and _ (up to 40)")
            if s["mode"] != "mapping":                     # slam_toolbox can't serialize a map it only localizes on
                return Reply(False, "Saving is for a map being made (New map, or Explore): a loaded map is already "
                                    "saved" if s["mode"] else "SLAM isn't running")
            return Reply(True, f"Saving map '{name}'...", None, (("slam_save", name),))
        if not s["managed"]:
            return Reply(False, "Switching maps needs the console to run SLAM: start 'Robot (all modes)' in the "
                                "Launcher")
        fx = list(self._enter("MANUAL", "Changing maps", "info").effects) if self.mission in NAV_MISSIONS else []
        if verb == "slam_new":
            s.update(mode=None, busy=True)
            self.event("info", "Starting a new map")
            return Reply(True, "Starting a new map: drive (or Explore) to fill it", None,
                         tuple(fx + [("slam_start", "mapping", None, None)]))
        name = str(a["name"])
        if name not in s["maps"]:
            return Reply(False, f"No saved map called '{name}'")
        here = bool(a.get("here", False)) and self.pose is not None and self.pose[0] == "map"
        pose = list(self.pose[1:]) if here else [0.0, 0.0, 0.0]
        s.update(mode=None, busy=True)
        self.event("info", f"Loading map '{name}'")
        msg = (f"Localizing on '{name}' where the robot is now" if here else
               f"Localizing on '{name}': the robot assumes it is at the map's start; if not, set its position")
        return Reply(True, msg, None, tuple(fx + [("slam_start", "localization", name, pose)]))

    def set_slam(self, **kw):
        """The node's view of SLAM: mode (mapping / localization / None), map, managed, maps, busy."""
        old = self.slam.get("mode")
        self.slam.update(kw)
        if old == "mapping" and self.slam["mode"] != "mapping" and self.mission == "EXPLORE":
            self._finish_explore("SLAM stopped mapping")

    def _set(self, key: str, value) -> Reply:
        s = SETTING.get(key)
        if s is None:
            return Reply(False, f"Unknown setting '{key}'")
        v = s.coerce(value)
        self.values[key] = v
        self.settings_changed = True
        fx = []
        if key == "profile.kid":
            fx.append(("set_mode", self.robot_mode or MODE_OF[self.mission], v))
            fx.append(("param", "catch_behavior", "kid", v))
        elif key == "catch.returns":
            fx.append(("param", "catch_behavior", "return_throws", v))
        elif key == "catch.return_to":
            fx.append(("param", "catch_behavior", "return_to", "other" if v == "the other player" else "thrower"))
        elif key.startswith("goto.") and self.mission == "GOTO":
            fx.append(self._nav_limits(self.values["goto.speed"], self.values["goto.turn"]))
        elif key in ("goto.facing", "goto.spin", "goto.turn") and self.mission in ("EXPLORE", "PATROL"):
            fx.append(self._nav_limits(self.values[f"{self.mission.lower()}.speed"], self.values["goto.turn"]))
        elif key == "explore.speed" and self.mission == "EXPLORE":
            fx.append(self._nav_limits(v, self.values["goto.turn"]))
        elif key == "patrol.speed" and self.mission == "PATROL":
            fx.append(self._nav_limits(v, self.values["goto.turn"]))
        return Reply(True, f"{s.label}: {self._fmt(s, v)}", self.schema(), tuple(fx))

    @staticmethod
    def _fmt(s: Setting, v) -> str:
        if isinstance(v, str):
            return v
        return ("on" if v else "off") if isinstance(v, bool) else f"{v:g} {s.unit}".rstrip()

    def _enter(self, mission: str, task: str, level: str, cancel_nav: bool = True) -> Reply:
        fx = []
        t = self.ai_book.running()
        if t is not None:                                # the robot switched away under the AI's task: it ends
            self._ai_end_running("CANCELLED", f"the robot was switched to {mission}: {task}")
        if self.mission in ("FOLLOW", "PROGRAM", "CALIBRATE") and mission in ("MANUAL", "IDLE"):
            self.zero_until = self.now + ZERO_TAIL       # a program stopped mid-turn: zeros, not silence
        if cancel_nav and self.goal is not None:
            fx.append(("nav_cancel",))
            self.goal = None
        if self.mission in ("TIDY", "FETCH", "TOWER") and mission != self.mission:
            fx.append(("arm", dict(verb="stop")))       # the arm stows (a drop under way finishes first; held: the bin)
        if self.mission == "GOALIE" and mission != "GOALIE":
            fx.append(("goalie", False))                # the ball tracker sleeps again
        if self.fun is not None:
            self.fun, self.dance, self.goalie = None, None, None
        if mission != "EXPLORE":
            self.explore = None
        if mission != "PATROL":
            self.patrol = None
        if mission != "FOLLOW":
            self.follower = None
        if mission != "PROGRAM":
            self.runner = None
        if mission != "CALIBRATE":
            self.calib_run = None
        mode = MODE_OF[mission]
        if mode != self.robot_mode or self.pending_mode is not None:
            fx.append(("set_mode", mode, self.values["profile.kid"]))
            self.pending_mode = mode
        if mission != "MANUAL":
            self.manual_in = None
        if self.act is not None:                         # a person (or the AI) moved the robot on: the task is over
            self._act_close("stopped" if mission in ("IDLE", "MANUAL") else "replaced", task)
        self.mission, self.task = mission, task
        self.progress = 0.0 if mission == "GOTO" else None
        self.event(level, task)
        self._act_open()
        return Reply(True, task, None, tuple(fx))

    # -- Explore and Patrol: what to do next ---------------------------------------------------------------------
    def tick(self, now: float, grid=None) -> tuple:
        """Every ~0.5 s from the node (grid: explore.Grid of the current map while exploring). Returns effects."""
        self.now = now
        fx = self._ai_tick(now)
        if not fx and self.mission in FUN_MISSIONS:
            fx = self.fun_tick(now)
        if not fx and self.mission == "EXPLORE" and self.explore is not None:
            fx = self._explore_tick(now, grid)
        elif not fx and self.mission == "PATROL" and self.patrol is not None:
            fx = self._patrol_tick(now)
        self.activity_sync()
        return tuple(fx)

    def _explore_tick(self, now: float, grid) -> list:
        from omni_console.explore import ExploreParams, frontier_goal, still_open
        e = self.explore
        v = self.values
        if e["phase"] == "wait":
            ready = (self.slam["mode"] == "mapping" and grid is not None and self.pose is not None
                     and self.pose[0] == "map")
            if not ready:
                if now - e["t0"] > 45.0:
                    self._finish_explore("the map never came (is SLAM running?)", "warn")
                return []
            e.update(phase="explore", start=self.pose[1:], t0=now)
        if e["phase"] == "home":
            return []
        time_up = now - e["t0"] > 60.0 * v["explore.max_time"]
        need = self.goal is None and now >= e["t_plan"]
        seen = (self.goal is not None and grid is not None and now - e["t_goal"] > 2.0
                and not still_open(grid, (self.goal["x"], self.goal["y"])))
        if time_up or ((need or seen) and grid is not None):
            goal, info = (None, e["info"]) if time_up else frontier_goal(
                grid, self.pose[1:], e["start"],
                ExploreParams(v["explore.radius"], v["explore.min_frontier"]), e["blacklist"])
            e["info"] = info
            e["t_plan"] = now + 1.0
            if goal is None:
                why = "time is up" if time_up else "nothing left to explore"
                if v["explore.return_home"] and e["start"] is not None:
                    e["phase"] = "home"
                    self.task = f"Exploring done ({why}): driving back"
                    self.event("info", self.task)
                    sx, sy, syaw = e["start"]
                    return self._nav_goal(sx, sy, syaw, "start", v["explore.speed"], v["goto.turn"])
                self._finish_explore(why)
                return [("nav_cancel",)] if self.goal is not None else []
            if self.goal is not None and math.hypot(goal[0] - self.goal["x"], goal[1] - self.goal["y"]) < 0.3:
                return []
            e["t_goal"] = now
            self.task = (f"Exploring: {info['frontiers']} openings left, {info['known_m2']:.0f} m² mapped")
            return self._nav_goal(goal[0], goal[1], goal[2], "frontier", v["explore.speed"], v["goto.turn"])
        return []

    def _finish_explore(self, why: str, level: str = "info"):
        area = (self.explore or {}).get("info", {}).get("known_m2")
        self.explore, self.goal = None, None
        self.mission, self.progress = "MANUAL", None
        self.task = f"Exploring done: {why}" + (f", {area:.0f} m² mapped" if area else "")
        self.event(level, self.task)

    def _patrol_tick(self, now: float) -> list:
        p = self.patrol
        if self.goal is not None or now < p["dwell_until"]:
            return []
        x, y, yaw = p["points"][p["i"]]
        n = len(p["points"])
        lap = f", lap {p['lap']}" + (f" of {p['laps']}" if p["laps"] else "")
        self.task = f"Patrol '{p['route']}': point {p['i'] + 1} of {n}{lap}"
        self.progress = p["i"] / max(1, n - 1)
        return self._nav_goal(x, y, yaw, f"point {p['i'] + 1}", self.values["patrol.speed"], self.values["goto.turn"])

    def _patrol_next(self) -> bool:
        """Advance to the next point; False when the patrol is over."""
        p = self.patrol
        n = len(p["points"])
        mode = self.values["patrol.mode"]
        i = p["i"] + p["step"]
        if mode == "back and forth":
            if i >= n or i < 0:
                p["step"] = -p["step"]
                i = p["i"] + p["step"]
                if p["step"] > 0:
                    p["lap"] += 1
        elif i >= n:
            if mode == "once":
                return False
            i = 0
            p["lap"] += 1
        if p["laps"] and p["lap"] > p["laps"]:
            return False
        p["i"] = i
        return True

    # -- what the robot reports ----------------------------------------------------------------------------------
    def on_robot_mode(self, text: str) -> tuple:
        fx = self._on_robot_mode(text)
        self.activity_sync()
        return fx

    def _on_robot_mode(self, text: str) -> tuple:
        """/robot_mode, e.g. "CATCH adult". Adopts the first one; afterwards a change nobody here asked for (another
        client of /set_mode) moves the mission along. Returns effects (a nav cancel when a Nav2 mission loses its
        mode)."""
        parts = text.split()
        if not parts:
            return ()
        mode, kid = parts[0].upper(), len(parts) > 1 and parts[1] == "kid"
        self.robot_mode, self.robot_kid = mode, kid
        if self.pending_mode is not None:
            if mode == self.pending_mode:
                self.pending_mode = None
            return ()
        if self.values["profile.kid"] != kid:
            self.values["profile.kid"] = kid
            self.settings_changed = True
        adopt = {"CATCH": "CATCH", "TELEOP": "MANUAL", "THROWER": "THROWER", "PICK": "MANUAL"}
        if not self.adopted:
            self.adopted = True
            self.mission = adopt.get(mode, "IDLE")
            self.task = {"CATCH": "Playing catch", "MANUAL": "Ready to drive", "IDLE": "Motors off",
                         "THROWER": "Thrower bench test"}[self.mission]
            self.event("info", f"Robot is in {mode} ({'kid' if kid else 'adult'} profile)")
            return ()
        if not self.mode_ok(self.mission, mode):
            fx = (("nav_cancel",),) if self.goal is not None else ()
            self.goal = self.explore = self.patrol = None
            self.follower = self.runner = self.calib_run = None
            self.mission = adopt.get(mode, "IDLE")
            self.task = f"Switched to {mode} outside the console"
            self.progress = None
            self.event("warn", self.task)
            return fx
        return ()

    def on_set_mode_result(self, ok: bool, message: str):
        if not ok:
            self.pending_mode = None
            self.event("error", f"mode change refused: {message}")

    def on_nav_feedback(self, seq: int, distance: float):
        if self.goal is None or seq != self.goal["seq"]:
            return
        if self.mission == "GOTO":
            if self.d0 is None or distance > self.d0:
                self.d0 = max(distance, 1e-3)
            self.progress = max(0.0, min(1.0, 1.0 - distance / self.d0))
            name = self.goal["name"] or f"({self.goal['x']:.1f}, {self.goal['y']:.1f})"
            self.task = f"Going to {name}: {distance:.1f} m left"

    def on_nav_done(self, seq: int, outcome: str, now: float | None = None):
        """outcome: succeeded | aborted | canceled | rejected. A goal replaced or cancelled here reports nothing."""
        self._on_nav_done(seq, outcome, now)
        self.activity_sync()

    def _on_nav_done(self, seq: int, outcome: str, now: float | None = None):
        if self.goal is not None and seq == self.goal["seq"] and self.mission in ("FETCH", "MESSAGE"):
            self.goal = None
            why = {"aborted": "Nav2 gave up", "rejected": "Nav2 refused the goal", "canceled": "cancelled"}.get(outcome,
                                                                                                         outcome)
            self.fx_queue += self.fun_nav_done(outcome == "succeeded", why)
            return
        if self.goal is None or seq != self.goal["seq"] or self.mission not in NAV_MISSIONS + ("PROGRAM",):
            return
        now = self.now if now is None else now
        g, self.goal = self.goal, None
        ok = outcome == "succeeded"
        if self.mission == "PROGRAM":                      # a program's goto block
            if self.runner is not None:
                self.runner.goto_state = "done" if ok else "failed"
            return
        why = {"aborted": "Nav2 gave up (blocked or no path)", "rejected": "Nav2 refused the goal",
               "canceled": "cancelled by Nav2"}.get(outcome, outcome)
        if self.mission == "GOTO":
            name = g["name"] or f"({g['x']:.1f}, {g['y']:.1f})"
            self.mission, self.progress = "MANUAL", None
            self.task = f"Arrived at {name}" if ok else f"Could not reach {name}: {why}"
            self.event("info" if ok else "warn", self.task)
            t = self.ai_book.running(("goto",))
            if t is not None:
                ev = dict(nav2=outcome)
                if self.pose is not None:
                    fr, x, y, yaw = self.pose
                    ev.update(pose=dict(frame=fr, x=round(x, 3), y=round(y, 3), yaw_deg=round(math.degrees(yaw), 1)),
                              distance_to_goal_m=round(math.hypot(x - g["x"], y - g["y"]), 3),
                              heading_error_deg=round(math.degrees(math.remainder(yaw - g["yaw"], 2 * math.pi)), 1))
                self.ai_book.finish(t, "SUCCEEDED" if ok else "FAILED", self.wall_time(), self.task,
                                    result=None if ok else dict(error=f"nav2_{outcome}"), evidence=ev)
        elif self.mission == "EXPLORE":
            e = self.explore
            if e is None:
                return
            if e["phase"] == "home":
                self._finish_explore("back at the start" if ok else f"could not drive back: {why}")
            elif not ok:
                e["blacklist"].append((g["x"], g["y"]))
                self.event("warn", f"Skipping an opening at ({g['x']:.1f}, {g['y']:.1f}): {why}")
        elif self.mission == "PATROL":
            p = self.patrol
            if p is None:
                return
            if ok:
                p["fails"] = 0
                p["dwell_until"] = now + self.values["patrol.dwell"]
            else:
                p["fails"] += 1
                self.event("warn", f"Patrol: skipping {g['name']}: {why}")
                if p["fails"] >= 3:
                    self.patrol = None
                    self.mission, self.progress = "MANUAL", None
                    self.task = "Patrol stopped: three points in a row unreachable"
                    self.event("warn", self.task)
                    return
            if not self._patrol_next():
                route = p["route"]
                self.patrol = None
                self.mission, self.progress = "MANUAL", None
                self.task = f"Patrol '{route}' finished"
                self.event("info", self.task)

    def set_caps(self, **caps):
        self.caps.update(caps)
        if self.mission in NAV_MISSIONS and not self.caps["nav"]:
            self.goal = self.explore = self.patrol = None
            self.mission, self.progress = "MANUAL", None
            self.task = "Navigation stopped: goal dropped"
            self.event("warn", self.task)
        self.activity_sync()

    # -- the console's stick -----------------------------------------------------------------------------------
    def manual_input(self, vx: float, vy: float, wz: float, boost: bool, now: float):
        """The page's stick, -1..1 per axis (vx forward, vy left, wz turn left), sent while the dead man is held."""
        if self.mission != "MANUAL":
            return
        c = lambda u: max(-1.0, min(1.0, float(u))) if math.isfinite(float(u)) else 0.0      # noqa: E731
        vx, vy, wz = c(vx), c(vy), c(wz)
        n = math.hypot(vx, vy)
        if n > 1.0:
            vx, vy = vx / n, vy / n
        self.manual_in = (vx, vy, wz, bool(boost))
        self.t_manual = now

    def manual_output(self, now: float):
        """The twist to send this tick (m/s, rad/s, body frame), or None (send nothing)."""
        if self.mission != "MANUAL" or self.manual_in is None:
            return None
        age = now - self.t_manual
        if age > MANUAL_TIMEOUT + ZERO_TAIL:
            self.manual_in = None
            return None
        if age > MANUAL_TIMEOUT:
            return (0.0, 0.0, 0.0)
        vx, vy, wz, boost = self.manual_in
        e = self.values["manual.expo"]
        speed = self.values["manual.boost_speed" if boost else "manual.max_speed"]
        n = math.hypot(vx, vy)
        k = speed * shape(n, e) / n if n > 1e-6 else 0.0
        return (vx * k, vy * k, shape(wz, e) * math.radians(self.values["manual.max_turn"]))

    def manual_active(self, now: float) -> bool:
        return self.mission == "MANUAL" and self.manual_in is not None and now - self.t_manual <= MANUAL_TIMEOUT

    # -- what the page shows ------------------------------------------------------------------------------------
    def schema(self) -> dict:
        return dict(version=VERSION, missions=list(MISSIONS), features=list(FEATURES),
                    workspaces=[dict(id=k, label=lbl) for k, lbl in WORKSPACES],
                    settings=[s.describe(self.values[s.key]) for s in SETTINGS],
                    places={k: v for k, v in sorted(self.places.items())},
                    routes={k: v for k, v in sorted(self.routes.items())}, map=self.map_name,
                    zones={k: v for k, v in sorted(self.zones.items())},
                    rooms={k: v for k, v in sorted(self.rooms.items())},
                    edges={k: v for k, v in sorted(self.edges.items())},
                    programs={k: v for k, v in sorted(self.programs.items())}, player=self.throws.player,
                    ai=dict(verbs=list(AI_VERBS), poses=list(ARM_POSES), shapes=list(FIND_SHAPES),
                            human_hold_s=HUMAN_HOLD, lease_ttl_s=list(AI_TTL),
                            firmware={k: dict(max_speed=v[0], max_turn_deg_s=round(math.degrees(v[1]), 1))
                                      for k, v in FW_LIMITS.items()}))

    def summary(self) -> dict:
        g = None if self.goal is None else {k: self.goal[k] for k in ("x", "y", "yaw", "name")}
        e = self.explore
        p = self.patrol
        return dict(mission=self.mission, task=self.task, progress=self.progress, goal=g, robot_mode=self.robot_mode,
                    kid=self.values["profile.kid"], caps=dict(self.caps, arm=self._arm_up()), version=VERSION,
                    control=self.control(self.now), cap=self.speed_cap(), activity_n=self.activity_n,
                    slam=dict(self.slam),
                    explore=None if e is None else dict(phase=e["phase"], start=e["start"], info=e["info"],
                                                         failed=e["blacklist"]),
                    patrol=None if p is None else dict(route=p["route"], i=p["i"], n=len(p["points"]),
                                                       lap=p["lap"], laps=p["laps"]),
                    thrower=dict(self.thrower, fire_tilt_max=launch_limits()[0] - launch_limits()[1])
                    if self.mission == "THROWER" else None,
                    follow=None if self.follower is None else dict(
                        mode=self.follower.mode, phase=self.follower.phase, text=self.follower.text(),
                        tags=self.follower.tags, person=self._person_rel()),
                    program=None if self.runner is None else dict(name=self.runner.name, i=self.runner.i,
                                                                  n=len(self.runner.steps), label=self.runner.label()),
                    calib=None if self.calib_run is None else dict(proc=self.calib_run.proc, phase=self.calib_run.phase,
                                                                   text=self.calib_run.text()),
                    slow=None if self.slow is None else dict(speed=self.slow[0], edge=self.slow[1]),
                    player=self.throws.player, localization=self.loc_health_now(),
                    arm=self._arm_summary(), ai=self.ai_summary(),
                    goalie=None if self.goalie is None else dict(self.goalie.summary(),
                                                                  ball=self.now - self.t_ball < 0.5),
                    fun=None if self.fun is None else {k: v for k, v in self.fun.items()
                                                       if k in ("kind", "phase", "what", "text", "i")},
                    dance=None if self.dance is None else dict(routine=self.dance.routine, bpm=self.dance.bpm))

    @staticmethod
    def mode_of(mission: str) -> str:
        return MODE_OF[mission]

    def _arm_summary(self):
        if not self._arm_up():
            return None
        s = self.arm
        return {k: s.get(k) for k in ("torque", "source", "ok", "joints", "names", "loads", "temps", "volts", "out",
                                      "tidy", "tidy_on", "wrist_cam")}

    # -- localization health (design.md 10: omni_slam's /localization/health) ----------------------------------------
    def on_loc_health(self, h: dict, now: float):
        prev = (self.loc_health or {}).get("level")
        self.loc_health, self.t_loc_health = h, now
        if prev is not None and h.get("level") != prev:
            why = "; ".join(h.get("reasons") or []) or "fixes agree again"
            self.event("warn" if h.get("level") in ("POOR", "LOST") else "info",
                       f"Localization {h.get('level')}: {why}")

    def loc_health_now(self):
        h = getattr(self, "loc_health", None)
        if h is None or self.now - getattr(self, "t_loc_health", -1e9) > 2.0:
            return None                                       # stale: nobody is assessing (no omni_slam running)
        return h


def load_json(path) -> dict:
    try:
        with open(path) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def save_json(path, obj):
    """Atomic: a crash mid-write leaves the old file."""
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=d, suffix=".tmp")
    with os.fdopen(fd, "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
    os.replace(tmp, path)
