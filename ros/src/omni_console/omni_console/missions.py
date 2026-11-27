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
  MOTORS   the motor test (wheels up): the stick drives the chosen wheels   WHEELS
           each at its own speed (motors_output: /wheels/cmd), ramped at motors.ramp; motors_run plays a
           timed profile on them instead (steps, a speed sweep, or the caller's segments: the wheel-load captures)
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
    BACKOUT_CLEAR,
    BACKOUT_MAX,
    BACKOUT_MOVED,
    BACKOUT_S,
    BACKOUT_STALL,
    BACKOUT_SPEED,
    BLOCK_AHEAD,
    BLOCK_R,
    CREEP_AFTER,
    CREEP_LEN,
    CREEP_S,
    CREEP_SPEED,
    TRAIL_N,
    TRAIL_STEP,
    EXPLORE_NEAR,
    EXPLORE_NEAR_S,
    HOME_RETRIES,
    HOME_RETRY_S,
    JAM_MOVE,
    KID_SPEED,
    LAUNCHER_RPMS,
    MANUAL_TIMEOUT,
    MAP_NAME,
    MISSIONS,
    MISSION_FEATURE,
    MODE_OF,
    N_WHEELS,
    NAME,
    NAV_MISSIONS,
    NEW_MAP,
    Reply,
    MAP_WAIT,
    SAY_MAX,
    SETTING,
    SETTINGS,
    Setting,
    TILT_RANGE,
    TRICKS,
    FEELINGS,
    FLOOR_GRACE,
    FLOOR_STALE,
    FLOOR_WAIT,
    VERB_FEATURE,
    VERBS,
    VERSION,
    WHEEL_PROFILE_MAX_S,
    WHEEL_TEST_ACCEL,
    WHEEL_TEST_ACCEL_MAX,
    WHEEL_TEST_MAX,
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
        self.blocks = []                   # (x, y): where Explore found it couldn't pass (nav_zones; cleared per run)
        self.plan_xy = None                # Nav2's global plan [(x, y), ...] (the node sets it), for blocks
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
        self.pose_t = None                 # its TF stamp, s on the robot's clock (None: use now)
        self.explore = None                # dict: phase, start, t0, blacklist, info, t_plan, t_goal
        self.patrol = None                 # dict: route, points, i, step, lap, laps, dwell_until, fails
        # the jam check of GOTO / EXPLORE / PATROL: where it last made headway (x, y), the goal-driving time since
        # (still), and the last tick; the start of the task (home when the map has no place 'home'); the stuck fault
        self.jam = None                    # dict(x, y, still, t)
        self.creep = None                  # dict(path, i, t0, start): following Nav2's plan through a tight spot
        self.nav_home = None               # (x, y, yaw) in map
        self.homing = False                # the GOTO home after a jam: stuck again there = the fault
        self.stuck = None                  # dict(x, y, why): latched until a person starts another mode
        self.floor, self.t_floor = None, -math.inf     # omni_arm's /floor_obstacles/state (the height map), when
        self.thrower = dict(rpm=0.0, tilt=0.0, seq=0)
        # the motor test: the wheels the stick drives (0-based, the firmware's order); a profile motors_run plays
        self.motors = dict(wheels=[0], profile=None)
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
        # its feelings (omni_face /face/affect) and the wheel gesture playing (gestures.Playing), its last start
        self.affect, self.t_affect = None, -math.inf
        self.gesture, self.t_gesture = None, -math.inf

    def set_map(self, name, places: dict | None, routes: dict | None, zones: dict | None = None,
                edges: dict | None = None, rooms: dict | None = None):
        """The map changed (another map loaded, or a new one named): its own places, routes, no-go zones, edges and
        rooms (named outlines, map frame: what "the kitchen" means; the AI reads them, nothing drives by them)."""
        from omni_console.zones import check_edge, check_zone
        self.map_name = name
        self.nav_home = None
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
    def unavailable(self) -> dict:
        """{feature id: why} for what can't start now (checks.unavailable: the caps and the preflight): the status's
        "avail", what the page greys."""
        from omni_console.checks import unavailable
        return unavailable(self.preflight, dict(self.caps, arm=self._arm_up()), FEATURES)

    def gate(self, verb: str, a: dict) -> Reply | None:
        """A start refused by a failing preflight check (the check's own words) or a cap it needs being down. Stop,
        cancel and the arm's hold and torque-off are never refused; driving by hand (MANUAL) only by what stops it
        (missing drive motors, no MCU, the e-stop)."""
        from omni_console.checks import blocking
        if not self.preflight:
            return None
        if verb == "mission":
            fid = MISSION_FEATURE.get(str(a.get("name", "")).upper())
        elif verb == "arm":
            fid = "arm" if str(a.get("do", "")) in ("pose", "gripper", "jog", "cal_load") else None
        else:
            fid = VERB_FEATURE.get(verb)
        if fid is None or fid not in FEATURE:
            return None
        f = FEATURE[fid]
        rows = blocking(self.preflight, f)
        caps = dict(self.caps, arm=self._arm_up())
        miss = [n for n in f["needs"] if not caps.get(n)]
        if not rows and not miss:
            return None
        why = self.unavailable().get(fid, "")
        return Reply(False, f"{'Preflight refuses' if rows else 'Not available:'} {f['label']}: {why}",
                     dict(error="fault" if rows else "unavailable", preflight=[r["id"] for r in rows], needs=miss))

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
        elif self.manual_active(now) or self.motors_active(now):
            who, text = "stick", "Stick / gamepad on the console" + (" (motor test)" if self.mission == "MOTORS" else "")
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
            if verb == "motors":
                return self._motors(a)
            if verb == "motors_run":
                return self._motors_run(a)
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
            if verb == "express":
                return self._express(a)
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
        feel = a.get("feel")                       # a feeling to feel now (omni_face: face, sounds, the body if free)
        if feel is not None and feel not in FEELINGS:
            return Reply(False, f"feel: one of {', '.join(FEELINGS)}")
        if not text and trick is None and feel is None:
            return Reply(False, "Type something for the robot to say")
        mood, act = EXPRESSIONS[expr]
        req = dict(text=text, mood=None if feel and expr == "auto" else mood, act=trick or act)
        if feel:
            req.update(feel=feel, intensity=max(0.0, min(1.0, float(a.get("intensity", 0.6)))))
        if text:
            self.event("info", f"Robot says: {text}")
        return Reply(True, f"Saying: {text}" if text else f"Trick: {trick}" if trick else f"Feeling {feel}", None,
                     (("face", req),))

    # -- its feelings in the body (omni_face expression.py; gestures.py, omni_arm gestures.py) --------------------
    def on_affect(self, d: dict, now: float):
        """omni_face's /face/affect: what it feels (its emotion, and the pace its mood allows: affect.tempo)."""
        if isinstance(d, dict):
            self.affect, self.t_affect = d, now

    def tempo(self) -> float:
        """The pace its own driving takes from its mood (0.85-1.0), 1.0 unless emotion and emotion.nav are on (the
        switch that keeps navigation exactly as set)."""
        v = self.values
        if not (v["emotion.on"] and v["emotion.nav"]) or self.affect is None or self.now - self.t_affect > 5.0:
            return 1.0
        try:
            return max(0.85, min(1.0, float(self.affect.get("tempo", 1.0))))
        except (TypeError, ValueError):
            return 1.0

    def base_free(self, now: float) -> str:
        """"" if a wheel gesture may play now, else why not: only resting in MANUAL with nobody at the stick for 2 s,
        no fault, no AI task, not the RC's manual switch."""
        if self.mission != "MANUAL":
            return "the base is busy" if self.mission != "IDLE" else "the motors are off"
        if self.manual_in is not None or now - self.t_manual < 2.0:
            return "someone is driving"
        if (self.mcu or {}).get("faults"):
            return "a fault"
        if self.rc_manual:
            return "the RC has it"
        if self.ai_book.running() is not None:
            return "an AI task runs"
        return ""

    def _express(self, a: dict) -> Reply:
        """omni_face asks the body to show a feeling: {emotion, intensity, base: gesture or None, arm: gesture or
        None}. Each part plays only if its level allows (emotion.base / emotion.arm > 0) and that part of the robot is
        free; never during a task. The size is the level / 10 times the moment's intensity."""
        from omni_console.gestures import GAP, GESTURES, Playing, size_for
        v = self.values
        if not v["emotion.on"]:
            return Reply(False, "Emotion is off")
        k = max(0.0, min(1.0, float(a.get("intensity", 0.5))))
        done, why = [], []
        fx = ()
        base, arm = a.get("base"), a.get("arm")
        if base:
            no = ("not a gesture" if base not in GESTURES else "wheel gestures are off" if v["emotion.base"] < 1
                  else "one just played" if self.now - self.t_gesture < GAP else self.base_free(self.now))
            if no:
                why.append(f"wheels: {no}")
            else:
                self.gesture = Playing(base, size_for(v["emotion.base"], k), self.now)
                self.t_gesture = self.now
                done.append(f"wheels {base}")
        if arm:
            busy = self.mission in ("TIDY", "FETCH", "TOWER", "CALIBRATE")
            s = self.arm or {}
            no = ("arm gestures are off" if v["emotion.arm"] < 1 else "the arm isn't connected" if not self._arm_up()
                  else "the arm is busy" if busy or s.get("tidy_on") or s.get("cal") or s.get("source") not in
                  ("hold", None) else "")
            if no:
                why.append(f"arm: {no}")
            else:
                size = max(0.0, min(1.0, v["emotion.arm"] / 10.0)) * (0.6 + 0.4 * k)
                fx = (("arm", dict(verb="express", gesture=str(arm), size=round(size, 3))),)
                done.append(f"arm {arm}")
        msg = (", ".join(done) or "nothing") + (f" ({'; '.join(why)})" if why else "")
        return Reply(bool(done), f"Feeling {a.get('emotion', '')}: {msg}", None, fx)

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
        # the arm load calibration (design.md 4.5, omni_calib arm_load): arm_brain runs it, the person answers
        if what == "cal_load":
            if self.mission not in ("IDLE", "MANUAL"):
                return Reply(False, f"The arm load calibration needs the robot still: stop {self.mission} first")
            self.event("info", "Arm load calibration: started (the arm moves by itself)")
            return Reply(True, "Arm load calibration started", None,
                         (("arm", dict(verb="cal_load", quick=bool(a.get("quick", False)))),))
        if what == "cal_input":
            return Reply(True, "Sent", None, (("arm", dict(verb="cal_input", value=a.get("value", True))),))
        if what == "cal_cancel":
            return Reply(True, "Arm load calibration cancelled", None, (("arm", dict(verb="cal_cancel")),))
        return Reply(False, "arm: do = pose | gripper | jog | hold | relax | cal_load | cal_input | cal_cancel")

    # -- the AI client: console_ai.AiMixin; programs, zones, rooms, tf: console_edit.EditMixin; calibration:
    #    console_calib.CalibMixin -----------------------------------------------------------------------------


    def on_floor_state(self, s: dict, now: float):
        """omni_arm's height map state (2 Hz): its speed limit, and blind (null | "arm out" | "camera silent")."""
        self.floor, self.t_floor = s, now

    def floor_blind(self, now: float) -> str | None:
        """Why the floor camera can't see a drop now, or None (it can, or this robot has no height map: the catch
        robot, depth_layer:=raw). The arm robot needs one: its height map is all that keeps a new map's robot off a
        step down (Nov 25: the arm never read as stowed, every frame was skipped, and Explore drove to the den's
        edge)."""
        if self.t_floor == -math.inf and not self.caps.get("arm"):
            return None
        if now - self.t_floor > FLOOR_STALE:
            return "no word from the height map"
        return (self.floor or {}).get("blind") or None

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
        if name == "MOTORS":
            self.manual_in = None
            self.motors["profile"] = None
            gone = self.wheels_missing()
            have = [i for i in range(N_WHEELS) if i not in gone]
            if not have:
                return Reply(False, "No drive motor answers on CAN: nothing to test (motor power, the CAN cable)")
            self.motors["wheels"] = [i for i in self.motors["wheels"] if i in have] or have[:1]
        text = {"IDLE": "Motors off", "MANUAL": "Driving by hand", "CATCH": "Playing catch",
                "THROWER": "Thrower bench test: base parked",
                "MOTORS": "Motor test: hold the dead man to turn the chosen wheels"}[name]
        return self._enter(name, text, "info")

    def _nav_goal(self, x, y, yaw, name, speed, turn_deg) -> list:
        """Effects for a new Nav2 goal (replacing any), and the goal it becomes."""
        fx = [("nav_cancel",)] if self.goal is not None else []
        self.goal_seq += 1
        self.creep, self.plan_xy = None, None              # a creep follows the old goal's plan
        lim = self._nav_limits(speed, turn_deg)
        here = self.pose[1:3] if self.pose is not None and self.pose[0] == "map" else (x, y)
        v = max(0.05, lim[1])
        self.goal = dict(x=x, y=y, yaw=yaw, name=name, seq=self.goal_seq, t0=self.robot_now(), v=v,
                         limit=self.values["goto.goal_time"] + 3.0 * math.hypot(x - here[0], y - here[1]) / v)
        self.d0 = None
        fx.append(lim)
        fx.append(("nav_goal", x, y, yaw, self.goal_seq))
        return fx

    def _nav_limits(self, speed, turn_deg) -> tuple:
        """Nav2's top speed and turn rate, and which way the robot faces on the way (omni_bringup heading_node).
        Explore never spins: a spinning lidar maps worse."""
        facing = self.values["goto.facing"]
        if facing == "spinning" and self.mission == "EXPLORE":
            facing = "forward"
        speed = speed * self.tempo()                 # its mood's pace (emotion.nav off: exactly as set)
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
        if self.caps.get("arm"):
            fx.append(("arm", dict(verb="stop")))       # the arm stows: out, it hides the floor camera's drops
        self.explore = dict(phase="wait", start=None, t0=self.now, blacklist=[], reached=[], info={}, t_plan=0.0,
                            t_goal=0.0, t_blind=None)
        if self.blocks:
            self.blocks = []
            fx.append(("masks",))
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

    def _motors(self, a: dict) -> Reply:
        """Which wheels the motor test's stick drives: wheels = a list of 0-based indices (the firmware's order)."""
        if self.mission != "MOTORS":
            return Reply(False, "Press 'Take control' in the Motors tab first")
        try:
            sel = sorted({int(i) for i in a.get("wheels", [])})
        except (TypeError, ValueError):
            return Reply(False, "wheels: a list of wheel numbers")
        if not sel or any(not 0 <= i < N_WHEELS for i in sel):
            return Reply(False, f"Pick one or more wheels, 1 to {N_WHEELS}")
        gone = [i for i in sel if i in self.wheels_missing()]
        if gone:                               # the firmware would fault-stop on turning it
            return Reply(False, f"Wheel {', '.join(str(i + 1) for i in gone)} isn't answering on CAN: nothing to test")
        self.motors["wheels"] = sel
        return Reply(True, "Stick drives wheel" + ("s " if len(sel) > 1 else " ") + ", ".join(str(i + 1) for i in sel))

    def _motors_run(self, a: dict) -> Reply:
        """A timed profile on the chosen wheels (wheels up), for seeing how a motor and its wheel speed up and slow down
        (protocol 9: the firmware ramps at the asked rate, STATE carries each motor's current). preset "steps": rest,
        +speed, rest, -speed, +speed (a reversal), rest, each held `hold` s; "sweep": steady speeds up to `speed` and
        back, both ways (friction against speed). Or segments = [[rad/s, rad/s^2, hold s], ...]: ramp to the speed at
        that rate, then hold. speed / ramp default to the settings motors.max_speed / motors.ramp. stop = true ends it
        (the wheels ramp to rest at the default rate). Holding the stick's dead man takes over from a profile."""
        if self.mission != "MOTORS":
            return Reply(False, "Press 'Take control' in the Motors tab first")
        if a.get("stop"):
            was = self.motors.get("profile")
            self.motors["profile"] = None
            return Reply(True, f"Profile '{was['name']}' stopped: the wheels ramp to rest" if was else "No profile running")
        try:
            speed = min(abs(float(a.get("speed", self.values["motors.max_speed"]))), WHEEL_TEST_MAX)
            ramp = float(a.get("ramp", self.values["motors.ramp"]))
            hold = float(a.get("hold", 1.0))
            if not (WHEEL_TEST_ACCEL / 4 <= ramp <= WHEEL_TEST_ACCEL_MAX and 0.1 <= hold <= 10.0 and speed > 0.1):
                raise ValueError
        except (TypeError, ValueError):
            return Reply(False, f"speed 0.1 to {WHEEL_TEST_MAX:g} rad/s, ramp {WHEEL_TEST_ACCEL / 4:g} to "
                                f"{WHEEL_TEST_ACCEL_MAX:g} rad/s², hold 0.1 to 10 s")
        preset = str(a.get("preset", "steps" if "segments" not in a else "custom"))
        if preset == "steps":
            segs = [(0.0, ramp, 0.5), (speed, ramp, hold), (0.0, ramp, hold), (-speed, ramp, hold), (speed, ramp, hold),
                    (0.0, ramp, hold)]
        elif preset == "sweep":
            levels = [v for v in (2.5, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0) if v < speed - 0.5] + [speed]
            r = min(ramp, 200.0)                                   # steady speeds: how it gets there doesn't matter
            segs = [(s * v, r, 1.5) for s in (1.0, -1.0) for v in levels + levels[-2::-1]] + [(0.0, r, 0.5)]
        elif preset == "custom":
            try:
                segs = [(float(w), float(r), float(h)) for w, r, h in a["segments"]]
            except (KeyError, TypeError, ValueError):
                return Reply(False, "segments: a list of [rad/s, rad/s², hold s]")
            bad = [s for s in segs if not (abs(s[0]) <= WHEEL_TEST_MAX + 1e-6 and
                                           WHEEL_TEST_ACCEL / 4 <= s[1] <= WHEEL_TEST_ACCEL_MAX and 0.0 <= s[2] <= 10.0)]
            if not segs or len(segs) > 40 or bad:
                return Reply(False, f"1 to 40 segments, each |speed| ≤ {WHEEL_TEST_MAX:g} rad/s, ramp "
                                    f"{WHEEL_TEST_ACCEL / 4:g}-{WHEEL_TEST_ACCEL_MAX:g} rad/s², hold 0-10 s")
        else:
            return Reply(False, "preset: steps or sweep (or give segments)")
        # each segment lasts its ramp (from the speed before) plus its hold
        plan, w0, t = [], 0.0, 0.0
        for w, r, h in segs:
            t += abs(w - w0) / r + h
            plan.append((t, w, r))
            w0 = w
        if t > WHEEL_PROFILE_MAX_S:
            return Reply(False, f"That profile lasts {t:.0f} s: {WHEEL_PROFILE_MAX_S:g} s at most")
        self.manual_in = None
        self.motors["profile"] = dict(name=preset, plan=plan, total=t, t0=None, t=0.0)
        wh = ", ".join(str(i + 1) for i in self.motors["wheels"])
        self.event("info", f"Motor test: profile '{preset}' on wheel {wh}, {t:.1f} s")
        return Reply(True, f"Profile '{preset}' on wheel {wh}: {len(plan)} segments, {t:.1f} s",
                     dict(total=round(t, 3), segments=[[w, r, round(te, 3)] for te, w, r in plan]))

    def _profile_now(self, now: float):
        """(speed, ramp) of the running profile now, or None once it's over."""
        p = self.motors.get("profile")
        if p is None:
            return None
        if p["t0"] is None:
            p["t0"] = now
        p["t"] = now - p["t0"]
        for t_end, w, r in p["plan"]:
            if p["t"] < t_end:
                return w, r
        self.motors["profile"] = None
        self.event("info", f"Motor test: profile '{p['name']}' done")
        return None

    def motors_ramp(self) -> float:
        """The ramp the motor test asks the firmware for now (rad/s^2: /wheels/cmd a_brake, protocol 9)."""
        p = self.motors.get("profile")
        if p is not None and p["t0"] is not None:
            for t_end, _, r in p["plan"]:
                if p["t"] < t_end:
                    return r
        return float(self.values["motors.ramp"])

    def motors_output(self, now: float):
        """The motor test's wheel speeds this tick (rad/s, the firmware's wheel order), or None (send nothing): a
        running profile (motors_run) on the chosen wheels, else the stick's forward axis on them while the dead man is
        held, zeros for ZERO_TAIL after it is let go. Holding the dead man ends a profile."""
        if self.mission != "MOTORS":
            return None
        if self.motors.get("profile") is not None:
            if self.motors_active(now):
                self.motors["profile"] = None
                self.event("info", "Motor test: the stick took over from the profile")
            else:
                wr = self._profile_now(now)
                w = 0.0 if wr is None else wr[0]
                return [w if i in self.motors["wheels"] else 0.0 for i in range(N_WHEELS)]
        if self.manual_in is None:
            return None
        age = now - self.t_manual
        if age > MANUAL_TIMEOUT + ZERO_TAIL:
            self.manual_in = None
            return None
        if age > MANUAL_TIMEOUT:
            return [0.0] * N_WHEELS
        w = shape(self.manual_in[0], self.values["motors.expo"]) * self.values["motors.max_speed"]
        return [w if i in self.motors["wheels"] else 0.0 for i in range(N_WHEELS)]

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
        if self.gesture is not None and (self.mission != "MANUAL" or self.manual_in is not None or
                                         self.gesture.done(now) or self.base_free(now) not in ("",)):
            if self.mission == "MANUAL" and not self.gesture.done(now) and self.manual_in is None:
                self.zero_until = now + ZERO_TAIL                  # stopped short (a fault): a clean stop
            self.gesture = None                     # a feeling's gesture ends at any stick, mission, fault or its end
        if self.mission == "MANUAL":
            tw = self.manual_output(now)
            if tw is None and self.gesture is not None:
                tw = self.gesture.twist(now)
                if tw is None:
                    self.gesture, self.zero_until = None, now + ZERO_TAIL
        elif self.mission == "FOLLOW" and self.follower is not None:
            tw = self.follow_cmd if now - self.t_follow < 0.5 else (0.0, 0.0, 0.0)      # no scan: stop
            k = self.tempo()
            if k < 1.0:
                tw = (tw[0] * k, tw[1] * k, tw[2])
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
        elif self.mission == "EXPLORE" and self.explore is not None:
            tw = self._backout_twist()
            if tw is None and self.creep is not None:
                tw = self._creep_twist()
        elif self.mission in NAV_MISSIONS and self.creep is not None:
            tw = self._creep_twist()
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

    def wheels_missing(self) -> list:
        """The wheels (0-based) whose motor isn't answering on CAN (STATE dm_err DM_NO_REPLY, protocol 8)."""
        from omni_console.checks import DM_NO_REPLY
        return [i for i, e in enumerate((self.mcu or {}).get("dm_err") or []) if e == DM_NO_REPLY]

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
        self.jam, self.homing = None, False              # a new task: its own jam check (_jammed sets homing after)
        if mission != "EXPLORE":
            self._drop_blocks()
        if mission != "IDLE":
            self.stuck = None                            # the stuck fault holds through the stop, until a new mode
        if mission in NAV_MISSIONS:
            self.nav_home = tuple(self.pose[1:4]) if self.pose is not None and self.pose[0] == "map" else None
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
        if not fx:
            fx = self._jam_tick(now)
        if not fx and self.mission in FUN_MISSIONS:
            fx = self.fun_tick(now)
        if not fx and self.mission == "EXPLORE" and self.explore is not None:
            fx = self._explore_tick(now, grid)
        elif not fx and self.mission == "PATROL" and self.patrol is not None:
            fx = self._patrol_tick(now)
        self.activity_sync()
        return tuple(fx)

    # -- the jam check: no headway toward a Nav2 goal, or a goal taking too long --------------------------------------
    def robot_now(self) -> float:
        """The robot's clock: the pose's TF stamp (sim time in the sim, which runs ~0.35x real time in WSL: on the
        console's wall clock a moving robot timed out, Nov 25), else the console's."""
        return self.now if self.pose_t is None else self.pose_t

    def _jam_tick(self, now: float) -> list:
        """GOTO / EXPLORE / PATROL: a goal out for goto.stuck_time without the robot getting JAM_MOVE from where it
        last got to, or a goal past its own time limit (_nav_goal), is a jam (_jammed). Robot time (robot_now). The
        still clock runs only while a goal is out and carries across goals: Explore replanning while wedged, or Nav2
        aborting goal after goal from the same spot, is still a jam. A goal's time limit also catches wheels spinning
        against something the lidar can't see, where the pose keeps moving (Nov 25, the sim's den barrier).
        Explore stalled on its way to a frontier goal skips that opening first, once per spot: the controller failing
        the last bit (Nov 25, hm9: 0.17 m short, it drove off at once toward home) or a way that closed (Nov 26, hm18: a
        door open across the hallway) is not a wedged robot. Stalled again without headway, it is a jam."""
        if self.mission not in NAV_MISSIONS or self.pose is None or self.pose[0] != "map":
            return []
        now = self.robot_now()
        x, y = self.pose[1], self.pose[2]
        j, g = self.jam, self.goal
        if j is None or math.hypot(x - j["x"], y - j["y"]) >= JAM_MOVE:
            self.jam = j = dict(x=x, y=y, still=0.0, t=now, skipped=False, plan=list(self.plan_xy or []))
        elif self.plan_xy and not j["still"]:            # the plan it follows while making headway (blocks)
            j["plan"] = list(self.plan_xy)
        dt, j["t"] = min(1.0, max(0.0, now - j["t"])), now
        if g is None:
            return []
        j["still"] += dt
        if (j["still"] >= CREEP_AFTER and not j.get("crept") and self.creep is None
                and not (self.explore or {}).get("backout")):
            j["crept"] = True                              # once per spot (headway makes a new jam dict)
            self._creep(f"no headway for {j['still']:.0f} s")
        if j["still"] >= self.values["goto.stuck_time"]:
            e = self.explore
            if (self.mission == "EXPLORE" and e is not None and e["phase"] == "explore" and g["name"] == "frontier"
                    and not j.get("skipped")):
                j.update(still=0.0, skipped=True)
                if self._excuse(g, "no headway"):          # wedged, not a way that closed: no block, goal kept
                    self.goal = None
                    e["stale"], e["t_done"] = e.get("grid"), self.now
                    return [("nav_cancel",)]
                fx = self._explore_skip(self.now, f"no headway for {self.values['goto.stuck_time']:.0f} s on the way",
                                        reached=False)
                if math.hypot(x - g["x"], y - g["y"]) > 2 * EXPLORE_NEAR:
                    fx += self._block_ahead(x, y, j.get("plan"))
                self._backout("no headway", BACKOUT_STALL)  # and off whatever holds it, the way it came (hm30)
                return fx
            return self._jammed(f"no headway for {j['still']:.0f} s")
        if now - g.get("t0", now) > g.get("limit", math.inf):
            return self._jammed(f"not there after {now - g['t0']:.0f} s")
        return []

    def _jammed(self, why: str) -> list:
        """Stuck on the way to a goal: the stuck fault (status.stuck, an error event), the goal given up, and a drive
        home (GOTO 'home'): the map's place 'home', else where the task started (Explore: where exploring started).
        Stuck on the way home, already home, or no home known: IDLE (the motors off) to wait for a person. The fault
        holds until a person starts another mode."""
        x, y = self.pose[1], self.pose[2]
        g, e, v = self.goal, self.explore, self.values
        label = "its goal" if g is None else (g["name"] or f"({g['x']:.1f}, {g['y']:.1f})")
        stuck = dict(x=round(x, 2), y=round(y, 2), why=why, goal=label)
        home = self.places.get("home") or (e or {}).get("start") or self.nav_home
        homing = self.homing or (e is not None and e["phase"] == "home")
        area = (e or {}).get("info", {}).get("known_m2")
        what = f"Stuck at ({x:.1f}, {y:.1f}) on the way to {label} ({why}" + (f"; {area:.0f} m² mapped)" if area
                                                                             else ")")
        if homing or home is None or math.hypot(home[0] - x, home[1] - y) < JAM_MOVE:
            fx = list(self._enter("IDLE", f"{what}: free it, then start a mode again", "error").effects)
            self.stuck = stuck
            return fx
        speed = {"EXPLORE": v["explore.speed"], "PATROL": v["patrol.speed"]}.get(self.mission, v["goto.speed"])
        fx = list(self._enter("GOTO", f"{what}: driving home", "error").effects)
        self.homing, self.stuck = True, stuck
        return fx + self._nav_goal(home[0], home[1], home[2], "home", speed, v["goto.turn"])

    def _explore_tick(self, now: float, grid) -> list:
        from omni_console.explore import ExploreParams, frontier_goal, still_open
        e = self.explore
        v = self.values
        blind = self.floor_blind(now)
        if e["phase"] == "wait":
            ready = (self.slam["mode"] == "mapping" and grid is not None and self.pose is not None
                     and self.pose[0] == "map")
            if not ready or blind:
                if ready:
                    self.task = f"Exploring: waiting for the floor camera ({blind})"
                if now - e["t0"] > FLOOR_WAIT:
                    self._finish_explore("the map never came (is SLAM running?)" if not ready else
                                         f"the floor camera can't see drops ({blind})", "warn")
                return []
            e.update(phase="explore", start=self.pose[1:], t0=now, t0_robot=self.robot_now())
        if blind:                                          # no drop can be seen: stop rather than drive on blind
            e["t_blind"] = now if e["t_blind"] is None else e["t_blind"]
            if now - e["t_blind"] >= FLOOR_GRACE:
                cancel = self.goal is not None
                self._finish_explore(f"stopped: the floor camera can't see drops ({blind})", "error")
                return [("nav_cancel",)] if cancel else []
        else:
            e["t_blind"] = None
        if e.get("backout"):                               # drive_output drives it; no goal meanwhile
            return []
        if e["phase"] == "home":
            if self.goal is None and e.get("t_home") is not None and now >= e["t_home"] and e["start"] is not None:
                e["t_home"] = None
                sx, sy, syaw = e["start"]
                return self._nav_goal(sx, sy, syaw, "start", v["explore.speed"], v["goto.turn"])
            return []
        if grid is not None and e["info"]:                 # the area mapped so far, not only at the last plan
            e["info"]["known_m2"] = round(grid.known_area(), 1)
            if self.goal is not None:
                self.task = f"Exploring: {e['info']['frontiers']} openings left, {e['info']['known_m2']:.0f} m² mapped"
        # on the robot's clock (robot_now), as the jam check: the sim runs at ~0.35x real time in WSL
        time_up = self.robot_now() - e.get("t0_robot", e["t0"]) > 60.0 * v["explore.max_time"]
        g = self.goal                                      # hovering just short of a frontier goal: there
        if (g is not None and g["name"] == "frontier" and self.pose is not None and self.pose[0] == "map"
                and math.hypot(self.pose[1] - g["x"], self.pose[2] - g["y"]) < EXPLORE_NEAR):
            rt = self.robot_now()
            if e.get("near") is None or e["near"][0] != g["seq"]:
                e["near"] = (g["seq"], rt)
            if rt - e["near"][1] >= EXPLORE_NEAR_S:
                return self._explore_skip(now, "there", reached=True)
        else:
            e["near"] = None
        # after an arrival (or a failure) plan on the next /map, not the one from before the drive: slam_toolbox
        # refreshes it every map_update_interval (2 s), and the stale map made reached openings look uncleared
        fresh = grid is not e.get("stale") or now - e.get("t_done", -math.inf) > MAP_WAIT
        need = self.goal is None and now >= e["t_plan"] and fresh
        if grid is not None:
            e["grid"] = grid
        seen = (self.goal is not None and grid is not None and now - e["t_goal"] > 2.0
                and not still_open(grid, (self.goal["x"], self.goal["y"])))
        if time_up or ((need or seen) and grid is not None):
            goal, info = (None, e["info"]) if time_up else frontier_goal(
                grid, self.pose[1:], e["start"],
                ExploreParams(v["explore.radius"], v["explore.min_frontier"]), e["blacklist"] + e["reached"])
            e["info"] = info
            e["t_plan"] = now + 1.0
            if goal is None and not time_up and e["blacklist"] and not e.get("second"):
                # the skipped spots once more, from wherever the robot is now: a failure is often the moment's (a
                # floor mark since cleared, the robot's own spot; Nov 26, hm23 left the kitchen north this way)
                e["second"] = True                         # and without the blocks (hm23: one shut bedroom 3's door)
                self.event("info", f"Trying the {len(e['blacklist'])} skipped openings once more")
                e["blacklist"] = []
                self._drop_blocks()
                goal, info = frontier_goal(grid, self.pose[1:], e["start"],
                                           ExploreParams(v["explore.radius"], v["explore.min_frontier"]), e["reached"])
                e["info"] = info
            if goal is None:
                why = "time is up" if time_up else "nothing left to explore"
                if v["explore.return_home"] and e["start"] is not None:
                    e["phase"] = "home"
                    self._drop_blocks()
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

    def _block_ahead(self, x: float, y: float, plan=None) -> list:
        """A no-go spot where the robot stalled on its way (Nov 26, hm18-19: a door open across the hallway left a gap
        the global planner took and the robot couldn't pass, goal after goal): on Nav2's plan BLOCK_AHEAD from the robot
        (else nothing), a BLOCK_R disc in the keepout mask for the rest of this Explore and its drive home. It closes a
        1 m corridor by itself (the keepout filter isn't inflated) and stays clear of the robot's own cell. The plan is
        the one it followed while it last made headway (Nav2 replans during its recoveries: the plan of the moment led
        back the other way, and the block closed the hallway behind the robot, hm21), searched on from its point
        nearest the robot."""
        pts = plan if plan else (self.plan_xy or [])
        if not pts:
            return []
        k0 = min(range(len(pts)), key=lambda k: math.hypot(pts[k][0] - x, pts[k][1] - y))
        e = self.explore or {}
        for px, py in pts[k0:]:
            if math.hypot(px - x, py - y) >= BLOCK_AHEAD:
                from omni_console.explore import connected
                grid, start = e.get("grid"), e.get("start")
                if (grid is not None and start is not None
                        and not connected(grid, (x, y), start[:2], self.blocks + [(px, py)], BLOCK_R)):
                    self.event("warn", f"Not marking ({px:.1f}, {py:.1f}) impassable: it would cut the way back")
                    return []
                self.blocks.append((round(px, 2), round(py, 2)))
                self.event("warn", f"Marked ({px:.1f}, {py:.1f}) as impassable for this exploration")
                return [("masks",)]
        return []

    def nav_zones(self) -> dict:
        """The no-go zones for Nav2's keepout mask: the map's, and Explore's blocks as small polygons while it explores
        (not on the way back: a block must never keep the robot from home)."""
        if not self.blocks or self.explore is None or self.explore["phase"] != "explore":
            return self.zones
        z = dict(self.zones)
        for k, (bx, by) in enumerate(self.blocks):
            z[f"_block{k}"] = [(bx + BLOCK_R * math.cos(a * math.pi / 4), by + BLOCK_R * math.sin(a * math.pi / 4))
                               for a in range(8)]
        return z

    def _explore_skip(self, now: float, why: str, reached: bool) -> list:
        """Done with the frontier goal out without Nav2's verdict: within EXPLORE_NEAR of it (reached, as a success),
        or stalled just short of it (skipped, as a failure). The goal is cancelled; the next plan waits for a fresh map
        as after any drive (_nav_done)."""
        g, e = self.goal, self.explore
        self.goal = None
        e["stale"], e["t_done"] = e.get("grid"), now
        (e["reached"] if reached else e["blacklist"]).append((g["x"], g["y"]))
        if not reached:
            self.event("warn", f"Skipping an opening at ({g['x']:.1f}, {g['y']:.1f}): {why}")
        return [("nav_cancel",)]

    # -- backing out of a tight spot (Nov 26, hm23): the robot drove in beside the kitchen island's stools, whose seats
    # the floor camera marked only once it stood there; from inside Nav2's inscribed radius NavFn finds no path and the
    # backup recovery refuses ("collision ahead"), so goal after goal failed from the one spot for 400 s and the whole
    # kitchen north and the rooms past it were blacklisted and never seen
    def _boxed(self) -> bool:
        """Is the robot nearer walls or floor marks (Explore's grid) than Nav2 can plan from?"""
        from omni_console.explore import clearance_at
        grid = (self.explore or {}).get("grid")
        return (grid is not None and self.pose is not None and self.pose[0] == "map"
                and clearance_at(grid, self.pose[1:3]) < BACKOUT_CLEAR)

    def _backout(self, why: str, min_len: float = 0.0) -> bool:
        """Start backing out along the robot's own trail (at least min_len) to the nearest point Nav2 can plan from
        (drive_output drives it, on the manual input above Nav2's). False: no such point within BACKOUT_MAX of trail.
        Stalled on a doorway's jamb by a ramp (Nov 26, hm30) the wheels' slip had put the pose 0.4 m off, onto the
        doorway's middle, so the spot didn't look tight; Explore skipped the goal, jammed, and Nav2 couldn't drive home
        from there: a stall backs out too."""
        from omni_console.explore import backout_path
        e = self.explore
        if e is None or e.get("grid") is None or self.pose is None or self.pose[0] != "map":
            return False
        x, y = self.pose[1], self.pose[2]
        path = backout_path(e["grid"], list(e.get("trail") or []) + [(x, y)], BACKOUT_CLEAR, BACKOUT_MAX, min_len)
        if not path:
            return False
        e["backout"] = dict(path=path, i=0, t0=self.robot_now(), start=(x, y))
        self.task = "Exploring: backing out of a tight spot"
        self.event("warn", f"Backing out of a tight spot at ({x:.1f}, {y:.1f}) the way it came ({why})")
        return True

    # -- creeping along Nav2's plan through a tight spot (Nov 26, world:=course: MPPI asked 0.01-0.04 m/s at the spiral's
    # 0.8 m gap for a minute, the guard clear, while NavFn's plan ran down the gap's middle; the home's 24 in door)
    def _creep(self, why: str) -> bool:
        """Start following Nav2's current plan CREEP_LEN m from its point nearest the robot (drive_output drives it on
        the manual input, above Nav2's; the drive guard still applies). False: no plan to follow."""
        if self.pose is None or self.pose[0] != "map" or not self.plan_xy or len(self.plan_xy) < 2:
            return False
        x, y = self.pose[1], self.pose[2]
        pts = self.plan_xy
        k = min(range(len(pts)), key=lambda i: (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2)
        path, run = [tuple(pts[k])], 0.0
        for a, b in zip(pts[k:], pts[k + 1:]):
            run += math.hypot(b[0] - a[0], b[1] - a[1])
            path.append(tuple(b))
            if run >= CREEP_LEN:
                break
        if run < 0.2:                                      # the goal is right here: nothing to creep along
            return False
        self.creep = dict(path=path, i=0, t0=self.robot_now(), start=(x, y))
        self.event("warn", f"Creeping along the plan from ({x:.1f}, {y:.1f}) through a tight spot ({why})")
        return True

    def _creep_twist(self):
        """The creep's twist (body frame), or None when no creep is on (it ends at its path's end or after CREEP_S)."""
        c = self.creep
        if c is None or self.pose is None or self.pose[0] != "map" or self.mission not in NAV_MISSIONS:
            self.creep = None
            return None
        _, x, y, yaw = self.pose
        path = c["path"]
        while c["i"] < len(path) and math.hypot(path[c["i"]][0] - x, path[c["i"]][1] - y) < (
                0.08 if c["i"] < len(path) - 1 else 0.04):
            c["i"] += 1
        if c["i"] >= len(path) or self.robot_now() - c["t0"] > CREEP_S:
            sx, sy = c["start"]
            self.event("info", f"Creep {'done' if c['i'] >= len(path) else 'timed out'}: "
                               f"{math.hypot(x - sx, y - sy):.2f} m from where it stalled")
            self.creep = None
            return (0.0, 0.0, 0.0)
        tx, ty = path[c["i"]]
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        v = CREEP_SPEED if c["i"] < len(path) - 1 else min(CREEP_SPEED, max(0.04, d))
        dx, dy = dx / d * v, dy / d * v
        cs, sn = math.cos(yaw), math.sin(yaw)
        return (cs * dx + sn * dy, -sn * dx + cs * dy, 0.0)

    def _excuse(self, g: dict, why: str) -> bool:
        """A frontier goal failed with the robot in a tight spot: the spot's fault, so the goal isn't blacklisted and
        the robot backs out. Once per goal: failing again it is blacklisted as usual."""
        e = self.explore
        if e is None or not self._boxed():
            return False
        ex = e.setdefault("excused", [])
        if any(math.hypot(g["x"] - a, g["y"] - b) < 0.3 for a, b in ex) or not self._backout(why):
            return False
        ex.append((g["x"], g["y"]))
        return True

    def _backout_twist(self):
        """EXPLORE's own driving: None while Nav2 drives (the trail recorded: where the robot's body has been), a
        back-out's twist (body frame; the base is holonomic) while one is on."""
        e = self.explore
        if self.pose is None or self.pose[0] != "map":
            return None
        _, x, y, yaw = self.pose
        b = e.get("backout")
        tr = e.setdefault("trail", [])
        if b is None:
            if not tr or math.hypot(x - tr[-1][0], y - tr[-1][1]) >= TRAIL_STEP:
                tr.append((x, y))
                del tr[:-TRAIL_N]
            return None
        path = b["path"]
        while b["i"] < len(path) and math.hypot(path[b["i"]][0] - x, path[b["i"]][1] - y) < (
                0.05 if b["i"] < len(path) - 1 else 0.02):        # passing through; the last point itself
            b["i"] += 1
        if b["i"] >= len(path) or self.robot_now() - b["t0"] > BACKOUT_S:
            e["backout"] = None
            sx, sy = b.get("start", (x, y))
            if math.hypot(x - sx, y - sy) >= BACKOUT_MOVED:
                self.jam = None                            # it got off: headway starts again from here (else the
            #                                                next stall without headway is a jam, as before)
            e["trail"] = tr[:max(0, len(tr) - len(path))] + [(x, y)]
            self.task = "Exploring: out of the tight spot" if b["i"] >= len(path) else "Exploring: backing out timed out"
            return (0.0, 0.0, 0.0)
        tx, ty = path[b["i"]]
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        v = BACKOUT_SPEED if b["i"] < len(path) - 1 else min(BACKOUT_SPEED, max(0.04, d))
        dx, dy = dx / d * v, dy / d * v
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy, 0.0)

    def _drop_blocks(self):
        """Explore's blocks off Nav2's keepout mask (the run is over)."""
        if self.blocks:
            self.blocks = []
            self.fx_queue.append(("masks",))

    def _finish_explore(self, why: str, level: str = "info"):
        area = (self.explore or {}).get("info", {}).get("known_m2")
        self._drop_blocks()
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
        adopt = {"CATCH": "CATCH", "TELEOP": "MANUAL", "THROWER": "THROWER", "PICK": "MANUAL", "WHEELS": "MOTORS"}
        if not self.adopted:
            self.adopted = True
            self.mission = adopt.get(mode, "IDLE")
            self.task = {"CATCH": "Playing catch", "MANUAL": "Ready to drive", "IDLE": "Motors off",
                         "THROWER": "Thrower bench test", "MOTORS": "Motor test"}[self.mission]
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
        g = self.goal                    # the time limit grows to Nav2's route (around walls: Nov 25, ~3x the line)
        if "v" in g and math.isfinite(distance):
            g["limit"] = max(g["limit"], self.values["goto.goal_time"] + 3.0 * distance / g["v"])
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
        if self.mission == "GOTO" and self.homing:            # the drive home after a jam
            s = self.stuck or {}
            if ok:
                self.mission, self.progress, self.homing = "MANUAL", None, False
                self.task = f"Home after getting stuck at ({s.get('x', 0):.1f}, {s.get('y', 0):.1f}): {s.get('why')}"
                self.event("warn", self.task)
            else:
                self.fx_queue += list(self._enter("IDLE", f"Stuck: could not drive home either ({why}): free it, then "
                                                          "start a mode again", "error").effects)
                self.stuck = s or dict(why=why)
            return
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
                if not ok and e.get("home_tries", 0) < HOME_RETRIES:      # a passing mark may have closed the way
                    e["home_tries"] = e.get("home_tries", 0) + 1
                    e["t_home"] = now + HOME_RETRY_S
                    if self._boxed():                                     # and out of a tight spot first
                        self._backout(f"driving back: {why}")
                    self.event("warn", f"Driving back: {why}; trying again ({e['home_tries']} of {HOME_RETRIES})")
                    return
                self._finish_explore("back at the start" if ok else f"could not drive back: {why}")
                return
            e["stale"], e["t_done"] = e.get("grid"), now   # the next plan waits for a map drawn after this drive
            if ok:                                         # if its opening is still there, it can't be cleared from here
                e["reached"].append((g["x"], g["y"]))
            elif self._excuse(g, why):                     # the robot's tight spot, not the goal: backing out
                pass
            else:
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
        """The page's stick, -1..1 per axis (vx forward, vy left, wz turn left), sent while the dead man is held. In
        the motor test (MOTORS) its forward axis drives the chosen wheels (motors_output)."""
        if self.mission not in ("MANUAL", "MOTORS"):
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

    def motors_active(self, now: float) -> bool:
        """The motor test's stick is held (its dead man) now."""
        return self.mission == "MOTORS" and self.manual_in is not None and now - self.t_manual <= MANUAL_TIMEOUT

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
                    slam=dict(self.slam), stuck=self.stuck,
                    floor=None if self.t_floor == -math.inf and not self.caps.get("arm")
                    else dict(blind=self.floor_blind(self.now)),
                    explore=None if e is None else dict(phase=e["phase"], start=e["start"], info=e["info"],
                                                         failed=e["blacklist"], blocks=self.blocks),
                    patrol=None if p is None else dict(route=p["route"], i=p["i"], n=len(p["points"]),
                                                       lap=p["lap"], laps=p["laps"]),
                    thrower=dict(self.thrower, fire_tilt_max=launch_limits()[0] - launch_limits()[1])
                    if self.mission == "THROWER" else None,
                    motors=dict(wheels=list(self.motors["wheels"]), max_speed=self.values["motors.max_speed"],
                                active=self.motors_active(self.now), missing=self.wheels_missing(),
                                ramp=self.motors_ramp(),
                                profile=None if self.motors.get("profile") is None else dict(
                                    name=self.motors["profile"]["name"], t=round(self.motors["profile"]["t"], 2),
                                    total=round(self.motors["profile"]["total"], 2)))
                    if self.mission == "MOTORS" else None,
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
                    dance=None if self.dance is None else dict(routine=self.dance.routine, bpm=self.dance.bpm),
                    emotion=dict(tempo=round(self.tempo(), 3),
                                 gesture=None if self.gesture is None else self.gesture.name))

    @staticmethod
    def mode_of(mission: str) -> str:
        return MODE_OF[mission]

    def _arm_summary(self):
        if not self._arm_up():
            return None
        s = self.arm
        return {k: s.get(k) for k in ("torque", "source", "ok", "joints", "names", "loads", "temps", "volts", "out",
                                      "tidy", "tidy_on", "wrist_cam", "external", "strained", "force", "load_model",
                                      "reflexes", "reflex", "reflex_active", "current", "servo_notes", "cal",
                                      "cal_last")}

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
