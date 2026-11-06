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
masks, arm: a request for omni_arm's arm_node, face / face_hear: a line for omni_face to say, a typed voice command)
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
from dataclasses import dataclass

VERSION = "0.3.0"                 # the console protocol: the page warns when it differs from the robot's
MISSIONS = ("IDLE", "MANUAL", "GOTO", "EXPLORE", "PATROL", "CATCH", "THROWER", "FOLLOW", "PROGRAM", "CALIBRATE",
            "TIDY")
MODE_OF = {"IDLE": "IDLE", "MANUAL": "TELEOP", "GOTO": "TELEOP", "EXPLORE": "TELEOP", "PATROL": "TELEOP",
           "CATCH": "CATCH", "THROWER": "THROWER", "FOLLOW": "TELEOP", "PROGRAM": "TELEOP",
           "CALIBRATE": "TELEOP", "TIDY": "PICK"}            # mission -> mode_manager mode
ARM_POSES = ("stow", "ready", "high", "drop", "look")   # look: the wrist camera ahead, 24 deg up (omni_arm pick.py)
ARM_JOG_MAX = 30.0                 # deg: one jog of one joint (omni_arm's motion.JOG_MAX)
ARM_STALE = 2.0                    # s without /arm/state: the arm node isn't running
NAV_MISSIONS = ("GOTO", "EXPLORE", "PATROL")
DRIVE_MISSIONS = ("MANUAL", "FOLLOW", "PROGRAM", "CALIBRATE")   # the console drives: drive_output()
LAUNCHER_RPMS = (2500.0, 3000.0, 3500.0, 4000.0, 4500.0)        # the launcher calibration's shots (est: the range used)
MANUAL_TIMEOUT = 0.3               # s: the console's stick stops counting (dead man released, tab closed, Wi-Fi gone)
ZERO_TAIL = 0.3                    # s of zero twist after that, then silence (Nav2 / Foxglove / the joystick may drive)
KID_SPEED = 0.8                    # m/s, the firmware's kid cap (shown, not enforced, here)
FLY_RPM_MAX = 6000.0               # the console's slider; the firmware caps by profile (kid lower)
TILT_RANGE = (-30.0, 40.0)         # deg, the module's travel
MAP_NAME = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
NEW_MAP = "_new"                   # places / routes made while mapping, before the map has a name
VERBS = ("ping", "schema", "stop", "mission", "goto", "cancel", "set", "place_save", "place_delete", "explore",
         "patrol", "route_save", "route_delete", "thrower", "fire", "slam_new", "slam_save", "slam_load",
         "initial_pose", "follow", "program_save", "program_delete", "program_run", "zone_save", "zone_delete",
         "edge_save", "edge_delete", "calib", "calib_start", "calib_measure", "calib_shot", "calib_fit",
         "calib_record", "calib_clear", "player_set", "stats", "health", "maint_done", "tidy", "arm", "say", "hear",
         "ai_lease", "move", "find", "pick", "task", "look", "room_save", "room_delete", "tf", "frames")
AI_VERBS = ("ai_lease", "move", "find", "pick", "task", "look", "tf", "frames")   # the AI client's own (its plan 4)
# what a person's command (the page, the voice) takes the robot back from the AI with
HUMAN_VERBS = ("stop", "mission", "goto", "cancel", "explore", "patrol", "thrower", "fire", "follow", "program_run",
               "calib_start", "tidy", "arm", "slam_new", "slam_load")
HUMAN_HOLD = 10.0                  # s after a person takes over that the AI may not start motion
# the firmware's profile limits, fw_params_arm.h ADULT_LIMITS / KID_LIMITS: m/s, yaw rad/s, yaw rad/s^2
FW_LIMITS = {"adult": (2.0, 4.0, 15.0), "kid": (0.8, 2.0, 7.5)}
AI_TTL = (1.0, 10.0)               # s: the AI lease's time to live, its range (omni_mcp renews a 3 s lease every 1 s)
AI_TIMEOUT = {"goto": (120.0, 300.0), "pick": (180.0, 180.0), "arm_pose": (30.0, 60.0), "find": (15.0, 15.0)}
AI_MOVE_MAX = 300.0                # s: the longest move (its default deadline: 1.5 x the estimate + 10 s)
FIND_SHAPES = ("square", "round", "long", "any")
FIND_STALE = 30.0                  # s: a find older than this can't be picked from (find again)
FAULTS_STOP_AI = ("ESTOP", "FAULT_STOP", "LOW_BATT", "LINK_LOST", "MOTOR_ERR", "MOTOR_SILENT")
NAME = re.compile(r"^[^\x00-\x1f]{1,40}$")
SAY_MAX = 200                      # characters the Say box sends at once
# the Say box's expressions: what the face shows while it says the line (omni_face moments: mood, antic)
EXPRESSIONS = {"auto": (None, None), "happy": ("happy", "smile"), "excited": ("surprised", "wheee"),
               "sad": ("sad", None), "worried": ("worried", None), "sleepy": ("sleepy", "yawn"),
               "love": ("happy", "love"), "silly": ("happy", "giggle"), "wink": (None, "wink"),
               "thinking": ("focused", "think")}
# the antics the console can ask for (the Face tab's tricks; omni_face moments.ANTICS)
TRICKS = ("sing", "hum", "whistle", "dance", "giggle", "smile", "wink", "love", "yawn", "nap", "sneeze", "hiccup",
          "babble", "look", "dizzy", "sigh", "quip")


@dataclass(frozen=True)
class Setting:
    key: str
    tabs: tuple
    label: str
    default: float | bool | str
    unit: str = ""
    lo: float | None = None
    hi: float | None = None
    step: float | None = None
    hint: str = ""
    options: tuple = ()              # a choice setting: its values

    def coerce(self, v):
        if self.options:
            v = str(v)
            if v not in self.options:
                raise ValueError(f"{self.label}: one of {', '.join(self.options)}")
            return v
        if isinstance(self.default, bool):
            if isinstance(v, str):
                v = v.strip().lower() in ("1", "true", "yes", "on")
            return bool(v)
        v = float(v)
        if not math.isfinite(v) or (self.lo is not None and v < self.lo) or (self.hi is not None and v > self.hi):
            raise ValueError(f"{self.label}: {self.lo:g} to {self.hi:g} {self.unit}".rstrip())
        return v

    def describe(self, value) -> dict:
        kind = "choice" if self.options else "bool" if isinstance(self.default, bool) else "float"
        d = dict(key=self.key, tabs=list(self.tabs), label=self.label, kind=kind, default=self.default, value=value,
                 unit=self.unit, hint=self.hint)
        if kind == "float":
            d.update(min=self.lo, max=self.hi, step=self.step)
        if kind == "choice":
            d.update(options=list(self.options))
        return d


SETTINGS = (
    Setting("manual.max_speed", ("manual",), "Top speed", 1.0, "m/s", 0.1, 2.0, 0.1,
            "Speed at full stick. The firmware caps it anyway: 2 m/s adult, 0.8 m/s kid."),
    Setting("manual.boost_speed", ("manual",), "Boost speed", 2.0, "m/s", 0.5, 2.0, 0.1,
            "Speed at full stick while boost is held (gamepad right trigger, the flight stick's trigger, Shift)."),
    Setting("manual.max_turn", ("manual",), "Turn rate", 120.0, "deg/s", 20.0, 200.0, 10.0,
            "Turn speed at full twist (right stick, stick twist, Q / E)."),
    Setting("manual.expo", ("manual",), "Stick softness", 0.3, "", 0.0, 1.0, 0.05,
            "0 = linear. Higher makes small stick moves gentler for fine positioning; full stick is still full speed."),
    Setting("manual.obstacle_stop", ("manual",), "Obstacle slow-down", True,
            hint="Slows and stops before obstacles the lidar sees (Nav2's collision monitor). Only while navigation "
                 "runs; without it the stick drives straight into things."),
    Setting("goto.speed", ("goto",), "Speed", 2.0, "m/s", 0.1, 2.0, 0.05,
            "Nav2's top speed between goals: 2 m/s is the firmware's limit (the kid profile caps it at 0.8). It slows "
            "down for the goal by itself."),
    Setting("goto.turn", ("goto",), "Turn rate", 90.0, "deg/s", 20.0, 105.0, 5.0,
            "Nav2's top turn rate, and how fast it turns to face the way it's going."),
    Setting("goto.facing", ("goto", "patrol", "explore"), "Driving", "forward",
            options=("forward", "agnostic", "spinning"),
            hint="forward: the face turns toward where it's going (it hands the turn back to Nav2 near the goal, to "
                 "end at the goal's heading). agnostic: Nav2 picks the heading (it often drives sideways). spinning: "
                 "it spins all the way there (Explore drives forward instead: spinning maps worse)."),
    Setting("goto.spin", ("goto", "patrol"), "Spin rate", 120.0, "deg/s", 45.0, 220.0, 5.0,
            "How fast it spins when driving is 'spinning' (the kid profile caps turning at 115 deg/s)."),
    Setting("explore.radius", ("explore",), "How far", 10.0, "m", 2.0, 30.0, 0.5,
            "Openings farther than this from where exploring started are left alone."),
    Setting("explore.max_time", ("explore",), "Time limit", 10.0, "min", 1.0, 60.0, 1.0,
            "Stops exploring after this long, mapped or not."),
    Setting("explore.min_frontier", ("explore",), "Smallest opening", 0.5, "m", 0.2, 2.0, 0.1,
            "Gaps into unknown space narrower than this are ignored (chair legs, clutter)."),
    Setting("explore.speed", ("explore",), "Speed", 0.5, "m/s", 0.1, 0.8, 0.05,
            "Slower maps better: the lidar has more scans of each wall."),
    Setting("explore.return_home", ("explore",), "Come back when done", True,
            hint="Drive back to where exploring started when nothing is left (or the time is up)."),
    Setting("patrol.mode", ("patrol",), "Order", "loop", options=("loop", "once", "back and forth"),
            hint="loop: last point back to the first. once: stop at the last point. back and forth: reverse at each "
                 "end."),
    Setting("patrol.laps", ("patrol",), "Laps", 0.0, "", 0.0, 100.0, 1.0, "How many times round; 0 = until stopped."),
    Setting("patrol.dwell", ("patrol",), "Wait at each point", 3.0, "s", 0.0, 300.0, 1.0,
            "Time the robot stays at every point before driving on."),
    Setting("patrol.speed", ("patrol",), "Speed", 0.8, "m/s", 0.1, 2.0, 0.05, "Nav2's top speed between points."),
    Setting("profile.kid", ("manual", "catch", "thrower"), "Kid profile", False,
            hint="The firmware's kid limits in every mode: 0.8 m/s, gentle accelerations, lobbed returns, lower "
                 "flywheel speed."),
    Setting("catch.returns", ("catch",), "Throw back", True,
            hint="Throw the ball back to the thrower after each catch. Off: the robot keeps the ball."),
    Setting("catch.return_to", ("catch",), "Throw it to", "the thrower", options=("the thrower", "the other player"),
            hint="The other player: the pass game. Two people stand apart (2 m or more) and take turns; the robot "
                 "throws each ball it catches to the one who didn't throw it, and turns to face them."),
    Setting("follow.distance", ("follow",), "Distance", 1.0, "m", 0.6, 2.5, 0.1,
            "How far behind the person the robot keeps."),
    Setting("follow.speed", ("follow",), "Top speed", 0.6, "m/s", 0.2, 1.2, 0.05,
            "Follow's top speed. Tag always chases at kid speed or less."),
    Setting("tag.speed", ("follow",), "Tag speed", 0.5, "m/s", 0.2, 0.8, 0.05,
            "How fast the robot chases in Tag (the kid profile caps everything at 0.8 m/s anyway)."),
    Setting("program.speed", ("program",), "Speed", 0.3, "m/s", 0.1, 0.6, 0.05,
            "How fast programs drive their moves."),
    Setting("program.turn", ("program",), "Turn rate", 60.0, "deg/s", 20.0, 120.0, 5.0,
            "How fast programs turn and spin."),
    Setting("calib.distance", ("calibrate",), "Straight-line distance", 2.0, "m", 1.0, 4.0, 0.5,
            "How far the odometry check drives (more is more precise; leave room in front of the robot)."),
    Setting("calib.turns", ("calibrate",), "Spin turns", 3.0, "", 1.0, 5.0, 1.0,
            "How many turns the heading check spins."),
    Setting("tidy.max_objects", ("tidy",), "How many", 20.0, "", 1.0, 50.0, 1.0,
            "Stops after picking up this many things (the bin holds about a dozen small toys)."),
    Setting("face.antics", ("face",), "Antics", "normal", options=("off", "quiet", "normal", "lively"),
            hint="How often the robot looks around, hums, sings, yawns, naps or says something dry when nothing "
                 "else is going on (quiet: every few minutes, normal: every two to five minutes, lively: often). "
                 "Never during a throw or a fault."),
    Setting("face.humor", ("face",), "Humor", 75.0, "%", 0.0, 100.0, 5.0,
            hint="How funny it tries to be: 0 is plain and businesslike, 100 is a wisecrack whenever it can. Also "
                 "by voice: \"Omni, humor sixty percent\". Faults are always said plainly."),
    Setting("face.preset", ("face",), "Voice", "deadpan", options=("deadpan", "bright", "cute"),
            hint="Who it sounds like. Deadpan: a calm, low, dry voice from a small speaker box (TARS-like). Bright: "
                 "clean and light (EVE-like). Cute: the earlier high voice."),
    Setting("face.voice", ("face",), "Voice commands", "on", options=("off", "chat only", "on"),
            hint="Talk to the robot: its name first (\"Omni, follow me\"). Chat only: it answers and stops but "
                 "nothing else drives it. On: also follow, play catch, go to a saved place, explore. Off: the "
                 "microphones stay off."),
    Setting("tidy.radius", ("tidy",), "How far", 2.0, "m", 0.5, 5.0, 0.25,
            "Only things within this distance of where the tidy started. Clear the way: the tidy drives straight to "
            "each thing (no Nav2), slowly, and stops with its bumper."),
    Setting("ai.max_speed", ("ai",), "AI top speed", 0.5, "m/s", 0.1, 1.0, 0.05,
            "The fastest an AI client (Claude or Codex through omni_mcp) may drive the robot in its moves. The "
            "firmware's profile still caps it (kid 0.8 m/s)."),
    Setting("ai.max_turn", ("ai",), "AI turn rate", 180.0, "deg/s", 20.0, 229.0, 1.0,
            "The fastest an AI client may turn the robot ('as fast as you can'). The firmware caps turning at "
            "229 deg/s (adult) or 115 deg/s (kid)."),
)
SETTING = {s.key: s for s in SETTINGS}


@dataclass
class Reply:
    ok: bool
    message: str = ""
    data: dict | list | None = None
    effects: tuple = ()


def shape(u: float, expo: float) -> float:
    """Stick curve: linear at expo 0, cubic at 1; keeps sign and the end points."""
    return (1.0 - expo) * u + expo * u ** 3


def _points(raw) -> list:
    pts = [[float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0] for p in raw]
    if not all(math.isfinite(v) for p in pts for v in p):
        raise ValueError("not a number")
    return pts


def _program_check(blocks) -> list:
    from omni_console.program import check, flatten
    out = check(blocks)
    flatten(out)                         # too long once the repeats are unrolled: ValueError now, not at Run
    return out


_LAUNCH = None


def launch_limits() -> tuple:
    """(launch angle, the flattest launch fired), deg: the generated robot_v11.json (launch elevation = the launch angle
    minus the tilt). Below 24 deg the ball hits the face between the cameras (design.md 12.2); the firmware refuses
    those shots too (FIRE_TILT_MAX)."""
    global _LAUNCH
    if _LAUNCH is None:
        from omni_catch.robot_config import RobotConfig
        mo = RobotConfig().raw["module"]
        _LAUNCH = (float(mo["launch_angle_deg"]), float(mo.get("fire_min_elevation_deg", 25.0)))
    return _LAUNCH


def _name(a: dict, what: str) -> str:
    n = str(a.get("name", "")).strip()
    if not NAME.match(n):
        raise ValueError(f"{what} needs a name (up to 40 characters)")
    return n


class Console:
    def __init__(self, settings: dict | None = None, places: dict | None = None, routes: dict | None = None,
                 programs: dict | None = None, calib: dict | None = None, throws=None, health=None,
                 ai_tasks: dict | None = None):
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
        self.caps = {"nav": False, "catch": False, "collision": False, "arm": False}
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

    def pop_events(self) -> list:
        out = list(self.events)
        self.events.clear()
        return out

    # -- commands ---------------------------------------------------------------------------------------------
    def command(self, verb: str, args: dict | None = None) -> Reply:
        """A verb from the page, the voice (omni_face) or an AI client (args["ai"]: its session). A person's verb that
        moves the robot takes it back from the AI first (its task cancelled, its mission stopped)."""
        a = args or {}
        pre = ()
        if not a.get("ai") and verb in HUMAN_VERBS:
            pre = self._ai_takeover(f"a person took over ({verb} from the console)")
        r = self._command(verb, a)
        return Reply(r.ok, r.message, r.data, pre + r.effects) if pre else r

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
            if verb == "tidy":
                return self._tidy()
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

    # -- the AI client (ai_robot_architecture.md) -----------------------------------------------------------------
    @staticmethod
    def _err(code: str, message: str, **data) -> Reply:
        return Reply(False, message, dict(error=code, **data))

    def owner(self, now: float):
        """Who has the robot: "ai" (its task runs), "rc", "page" (a person: the page or the voice), or None (free)."""
        if self.ai_book.running() is not None:
            return "ai"
        if self.rc_manual:
            return "rc"
        if self.manual_active(now) or now - self.human_t < HUMAN_HOLD or self.mission not in ("IDLE", "MANUAL"):
            return "page"
        return None

    def _human_owner(self, now: float) -> str:
        if self.rc_manual:
            return "The RC transmitter has manual control"
        if now - self.human_t < HUMAN_HOLD:
            return (f"A person took control {now - self.human_t:.0f} s ago: the AI waits {HUMAN_HOLD:g} s before "
                    f"moving the robot again")
        if self.manual_active(now):
            return "Someone is driving with the console's stick"
        if self.mission not in ("IDLE", "MANUAL"):
            return f"The robot is busy ({self.mission.lower()}, from the console: {self.task}). Stop it there first"
        return ""

    def _fault(self) -> str:
        m = self.mcu or {}
        names = sorted({f["name"] for f in m.get("faults", [])} & set(FAULTS_STOP_AI))
        if m.get("mode") == "ESTOP" and "ESTOP" not in names:
            names.append("ESTOP")
        return f"The MCU reports {', '.join(names)}: a person has to clear it" if names else ""

    def _arm_out(self) -> bool:
        return self._arm_up() and bool(self.arm.get("out"))

    def _ai_gate(self, a: dict, motion: bool = True) -> Reply | None:
        """None when the AI may do this now, else the refusal: no_lease, busy, owner_conflict, fault."""
        now, sess = self.now, str(a.get("ai") or "")
        if not sess:
            return self._err("no_lease", "AI commands carry the AI session (args 'ai')")
        if self.ai is None or self.ai["session"] != sess or now > self.ai["until"]:
            return self._err("no_lease", "No valid AI lease: omni_mcp renews it every second (is it running?)")
        if motion:
            t = self.ai_book.running()
            if t is not None:
                return self._err("busy", f"Task {t.id} ({t.kind}) is still {t.state.lower()}: wait for it "
                                         f"(robot_task) or cancel it", task_id=t.id)
            who = self._human_owner(now)
            if who:
                return self._err("owner_conflict", who)
            bad = self._fault()
            if bad:
                return self._err("fault", bad)
        return None

    def _ai_dup(self, a: dict) -> Reply | None:
        """A retry with a request_id already seen: the same task back, nothing new started."""
        t = self.ai_book.by_req(str(a.get("request_id") or ""))
        if t is None:
            return None
        return Reply(True, f"Already accepted as {t.id} (same request_id): {t.state.lower()}", t.public(self.now))

    def _ai_new(self, kind: str, a: dict, args: dict, timeout: float, mission: str = ""):
        t = self.ai_book.new(kind, args, str(a.get("ai") or "page"), str(a.get("request_id") or ""), self.now,
                             self.wall_time(), self.now + timeout, mission)
        self.ai_book.start(t)
        return t

    @staticmethod
    def _timeout(a: dict, kind: str) -> float:
        default, most = AI_TIMEOUT[kind]
        v = float(a.get("timeout_s") or default)
        if not math.isfinite(v) or v <= 0:
            raise ValueError("timeout_s: a positive number")
        return min(v, most)

    def _ai_evidence(self, t) -> dict:
        if t.kind == "move" and self.runner is not None:
            return self.runner.measured()
        return {}

    def _ai_end_running(self, state: str, why: str):
        t = self.ai_book.running()
        if t is not None:
            self.ai_book.finish(t, state, self.wall_time(), why, evidence=self._ai_evidence(t),
                                result={"error": "timeout"} if state == "TIMED_OUT" else None)
            self.event("warn" if state != "SUCCEEDED" else "info", f"AI task {t.id} ({t.kind}): {why}")

    def _ai_stop(self, state: str, why: str) -> list:
        """End the AI's running task and stop what it was doing (the robot holds still, armed: MANUAL)."""
        self._ai_end_running(state, why)
        return list(self._enter("MANUAL", f"AI stopped: {why}", "warn").effects)

    def _ai_takeover(self, why: str) -> tuple:
        """A person took the robot (the page, the voice, the RC): the AI's task ends, its mission stops."""
        self.human_t = self.now
        if self.ai_book.running() is None:
            return ()
        return tuple(self._ai_stop("CANCELLED", why))

    def on_rc(self, manual: bool):
        """The RC transmitter's manual switch (/mcu/info), from the node."""
        if manual and not self.rc_manual:
            self.fx_queue += list(self._ai_takeover("the RC transmitter took manual control"))
        self.rc_manual = manual

    def _ai_tick(self, now: float) -> list:
        """The lease, the deadlines and the mission under each AI task: what ended them, and the stop."""
        fx = []
        if self.ai is not None and now > self.ai["until"]:
            self.ai = None
            self.event("warn", "The AI client's lease ran out (its connection or its process is gone)")
            if self.ai_book.running() is not None:
                fx += self._ai_stop("CANCELLED", "the AI client's lease ran out")
        t = self.ai_book.running()
        if t is not None and now > t.deadline:
            fx += self._ai_stop("TIMED_OUT", f"no result in {t.deadline - t.t0:.0f} s")
        t = self.ai_book.running()
        if t is not None and t.mission and self.mission != t.mission:
            self._ai_end_running("CANCELLED", f"the robot left {t.mission}: {self.task}")
        f = self.ai_book.running(("find",))
        if f is not None and now > f.deadline:
            self.ai_book.finish(f, "TIMED_OUT", self.wall_time(), "no answer from omni_arm's find",
                                result={"error": "timeout"})
        return fx

    def _ai_lease(self, a: dict) -> Reply:
        sess = str(a.get("session") or a.get("ai") or "")
        if not sess or len(sess) > 64:
            return self._err("bad_request", "ai_lease: a session id (up to 64 characters)")
        ttl = float(a.get("ttl_s", 3.0))
        if not AI_TTL[0] <= ttl <= AI_TTL[1]:
            return self._err("out_of_limits", f"ai_lease: ttl_s {AI_TTL[0]:g} to {AI_TTL[1]:g}")
        now = self.now
        if self.ai is not None and self.ai["session"] != sess and now <= self.ai["until"]:
            return self._err("owner_conflict", f"Another AI session ({self.ai['session'][:8]}) holds the lease")
        if self.ai is None or self.ai["session"] != sess:
            self.event("info", f"AI client connected (session {sess[:8]})")
        self.ai = dict(session=sess, until=now + ttl, ttl=ttl)
        return Reply(True, "lease", dict(session=sess, ttl_s=ttl, owner=self.owner(now)))

    @staticmethod
    def _ceiling(value, cap: float, default: float, what: str):
        """(used, capped): "max" -> the cap; a number -> itself, at most the cap; None -> the default."""
        if value is None:
            return min(default, cap), False
        if value == "max":
            return cap, False
        v = float(value)
        if not math.isfinite(v) or v <= 0:
            raise ValueError(f"{what}: a positive number or \"max\"")
        return min(v, cap), v > cap

    def _ai_move(self, a: dict) -> Reply:
        from omni_console.program import ProgramRunner, check
        dup = self._ai_dup(a)
        if dup:
            return dup
        bad = self._ai_gate(a)
        if bad:
            return bad
        try:
            blocks = check(a.get("steps"))
            runner_check = ProgramRunner("AI move", blocks, 0.1, 0.1)
        except ValueError as e:
            return self._err("bad_request", f"move: {e}")
        kinds = {s[0] for s in runner_check.steps}
        if not runner_check.steps:
            return self._err("bad_request", "move: no steps")
        if "goto" in kinds:
            return self._err("bad_request", "move has no goto blocks: use goto (robot_goto) for places")
        if "move" in kinds and not self.caps["nav"]:
            return self._err("not_localized", "Driving (forward, back, left, right) needs navigation running: its "
                                              "collision monitor guards the moves. Turns and spins in place work "
                                              "without it")
        if self._arm_out():
            return self._err("arm_out", "The arm is out: stow it first (robot_arm_pose stow); turning with it out "
                                        "hits things")
        prof = "kid" if self.values["profile.kid"] else "adult"
        v_fw, w_fw, a_fw = FW_LIMITS[prof]
        v_cap = min(self.values["ai.max_speed"], v_fw)
        w_cap = min(self.values["ai.max_turn"], math.degrees(w_fw))
        v, v_capped = self._ceiling(a.get("speed"), v_cap, self.values["program.speed"], "speed")
        w, w_capped = self._ceiling(a.get("turn_rate"), w_cap, self.values["program.turn"], "turn_rate")
        runner = ProgramRunner("AI move", blocks, v, math.radians(w))
        est = runner.estimate(a_fw, 2.0)
        timeout = float(a.get("timeout_s") or 1.5 * est + 10.0)
        if not math.isfinite(timeout) or timeout <= 0:
            return self._err("bad_request", "timeout_s: a positive number")
        timeout = min(timeout, AI_MOVE_MAX)
        if est > timeout:
            return self._err("out_of_limits", f"move: about {est:.0f} s, longer than its time limit "
                                              f"({timeout:.0f} s, at most {AI_MOVE_MAX:g})")
        r = self._enter("PROGRAM", f"AI move: {len(runner.steps)} steps, about {est:.0f} s", "info")
        self.runner = runner
        t = self._ai_new("move", a, dict(steps=blocks, speed=a.get("speed"), turn_rate=a.get("turn_rate")), timeout,
                         "PROGRAM")
        t.result.update(estimated_s=round(est, 1), limits=dict(
            speed_m_s=round(v, 3), turn_rate_deg_s=round(w, 1), speed_capped=v_capped, turn_rate_capped=w_capped,
            ai_max_speed=self.values["ai.max_speed"], ai_max_turn=self.values["ai.max_turn"], profile=prof,
            firmware_max_speed=v_fw, firmware_max_turn_deg_s=round(math.degrees(w_fw), 1)))
        t.message = f"moving: {len(runner.steps)} steps at {v:g} m/s, {w:.0f} deg/s; about {est:.1f} s"
        return Reply(True, t.message, t.public(self.now), r.effects)

    def _ai_goto(self, a: dict) -> Reply:
        dup = self._ai_dup(a)
        if dup:
            return dup
        bad = self._ai_gate(a)
        if bad:
            return bad
        if not self.caps["nav"]:
            return self._err("not_localized", "Navigation isn't running (Nav2): goto needs it")
        place = str(a.get("place") or "")
        if place:
            if place not in self.places:
                room = next((k for k in self.rooms if k.lower() == place.lower()), None)
                if room is not None:      # a room is an outline, not a pose: the caller picks a free point in it
                    pts = self.rooms[room]
                    cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
                    return self._err("unknown_place", f"'{room}' is a room, not a saved place: goto x, y inside its "
                                                      f"outline (its centre is ({cx:.2f}, {cy:.2f}); check it is free "
                                                      f"on the map first)")
                return self._err("unknown_place", f"No place called '{place}' on this map (places: "
                                                  f"{', '.join(sorted(self.places)) or 'none'}; rooms: "
                                                  f"{', '.join(sorted(self.rooms)) or 'none'})")
            x, y, yaw = self.places[place]
        else:
            x, y, yaw = float(a["x"]), float(a["y"]), math.radians(float(a.get("yaw_deg", 0.0)))
        if self.pose is None or self.pose[0] != "map":
            return self._err("not_localized", "The robot isn't localized on a map")
        h = self.loc_health_now()
        if h is not None and h.get("level") == "LOST":
            return self._err("not_localized", f"Localization is LOST: {'; '.join(h.get('reasons') or [])}")
        if self._arm_out():
            return self._err("arm_out", "The arm is out: stow it first (robot_arm_pose stow)")
        timeout = self._timeout(a, "goto")
        r = self._goto(x, y, yaw, place)
        if not r.ok:
            return r
        t = self._ai_new("goto", a, dict(place=place) if place else dict(x=x, y=y, yaw_deg=math.degrees(yaw)), timeout,
                         "GOTO")
        t.result["goal"] = dict(frame="map", x=round(x, 3), y=round(y, 3), yaw_deg=round(math.degrees(yaw), 1),
                                place=place or None)
        t.message = r.message
        return Reply(True, r.message, t.public(self.now), r.effects)

    def _ai_cancel(self, a: dict) -> Reply:
        """Cancel the AI's task (no lease needed: stopping is always allowed)."""
        t = self.ai_book.get(str(a.get("task_id", "")))
        if t is None:
            return self._err("unknown_task", f"No task '{a.get('task_id')}' (the robot's console may have restarted)")
        if t.done:
            return Reply(True, f"Task {t.id} had already ended: {t.state}", t.public(self.now))
        if t.kind == "find":
            self.ai_book.finish(t, "CANCELLED", self.wall_time(), "cancelled by the AI client")
            return Reply(True, f"Task {t.id} cancelled", t.public(self.now))
        fx = self._ai_stop("CANCELLED", "cancelled by the AI client")
        return Reply(True, f"Task {t.id} cancelled: the robot stopped", t.public(self.now), tuple(fx))

    def _ai_task(self, a: dict) -> Reply:
        t = self.ai_book.get(str(a.get("task_id", "")))
        if t is None:
            return self._err("unknown_task", f"No task '{a.get('task_id')}' (the robot's console may have restarted)")
        return Reply(True, t.message, t.public(self.now))

    def _ai_arm(self, a: dict) -> Reply:
        """The AI's arm: named poses only, on the arm's checked paths (never jog, relax or the gripper alone)."""
        if str(a.get("do", "")) != "pose":
            return self._err("bad_request", "The AI moves the arm to named poses only: do = pose")
        name = str(a.get("name", ""))
        if name not in ARM_POSES:
            return self._err("bad_request", f"arm pose: one of {', '.join(ARM_POSES)}")
        dup = self._ai_dup(a)
        if dup:
            return dup
        bad = self._ai_gate(a)
        if bad:
            return bad
        if not self._arm_up():
            return self._err("no_arm", "The arm isn't connected")
        t = self._ai_new("arm_pose", a, dict(name=name), self._timeout(a, "arm_pose"))
        t.result.update(pose=name, arm_ev=self.arm_ev)
        t.message = f"arm to {name}"
        return Reply(True, f"Arm to {name}", t.public(self.now), (("arm", dict(verb="pose", name=name)),))

    def _ai_find(self, a: dict) -> Reply:
        """Candidates on the floor by colour and shape (omni_arm: floor_vision's detections, colour_vision's labels).
        Read-only: no lease, no owner. The result comes with /arm/state (on_arm_state)."""
        if self.ai_book.running(("find",)) is not None:
            return self._err("busy", "A find is already running")
        if not self._arm_up():
            return self._err("no_arm", "find needs the arm robot running (omni_arm's arm_node)")
        color = a.get("color")
        if isinstance(color, dict):
            h = color.get("h")
            if not (isinstance(h, list) and len(h) == 2 and all(0 <= float(v) <= 179 for v in h)):
                return self._err("bad_request", "color: {h: [lo, hi] (0-179, OpenCV), s_min, v_min} or a name")
            color = dict(h=[float(h[0]), float(h[1])], s_min=float(color.get("s_min", 80)),
                         v_min=float(color.get("v_min", 50)))
        elif color is not None:
            color = str(color).strip().lower()[:20] or None
        shape = str(a.get("shape") or "any").lower()
        if shape not in FIND_SHAPES:
            return self._err("bad_request", f"shape: one of {', '.join(FIND_SHAPES)}")
        rng = float(a.get("max_range_m", 1.6))
        if not 0.3 <= rng <= 2.0:
            return self._err("out_of_limits", "max_range_m: 0.3 to 2.0 m (the depth camera's useful range)")
        t = self._ai_new("find", a, dict(color=color, shape=shape, max_range_m=rng), self._timeout(a, "find"))
        t.message = "looking"
        req = dict(verb="find", id=t.id, color=color, shape=shape, max_range=rng)
        return Reply(True, "Looking", t.public(self.now), (("arm", req),))

    def _ai_pick(self, a: dict) -> Reply:
        """The tidy, for one object of the last find (its id in omni_arm's floor map): approach, wrist look, grasp,
        the bin."""
        dup = self._ai_dup(a)
        if dup:
            return dup
        bad = self._ai_gate(a)
        if bad:
            return bad
        if not self._arm_up():
            return self._err("no_arm", "pick needs the arm robot running (omni_arm's arm_node)")
        if self._arm_out():
            return self._err("arm_out", "The arm isn't stowed: robot_arm_pose stow first")
        oid = int(a["object_id"])
        f = self.found
        if f is None:
            return self._err("stale_object", "No find yet: robot_find first")
        if self.found_epoch != self.loc_epoch:
            return self._err("stale_object", "The map or the localization changed since the find: find again")
        age = self.now - self.t_found
        if age > FIND_STALE:
            return self._err("stale_object", f"The last find is {age:.0f} s old (at most {FIND_STALE:g}): find again")
        cands = {int(c["id"]): c for c in f.get("candidates", [])}
        if oid not in cands:
            return self._err("not_found", f"No object {oid} in the last find (ids: "
                                          f"{', '.join(str(i) for i in sorted(cands)) or 'none'})")
        timeout = self._timeout(a, "pick")
        r = self._enter("TIDY", f"AI pick: object {oid}", "info")
        radius = max(float(self.values["tidy.radius"]), float(cands[oid].get("range_m", 1.0)) + 0.5)
        req = dict(verb="tidy", only=oid, max_objects=1, radius=radius)
        t = self._ai_new("pick", a, dict(object_id=oid), timeout, "TIDY")
        t.result["object"] = cands[oid]
        t.message = f"picking object {oid}"
        return Reply(True, f"Picking object {oid}", t.public(self.now), r.effects + (("arm", req),))

    def _look(self, a: dict) -> Reply:
        """A fresh frame from a camera the sim renders on request (front: the depth camera's colour, wrist). On the
        robot the cameras stream and the request is ignored."""
        cam = str(a.get("camera", "front"))
        what = {"front": "color", "d435_color": "color", "wrist": "wrist", "depth": "depth"}.get(cam)
        if what is None:
            return self._err("bad_request", "look: camera front | wrist | d435_color | depth")
        return Reply(True, f"{cam} frame requested", dict(camera=cam), (("camera_request", what),))

    def _ai_on_arm(self, s: dict, new_events: list):
        """/arm/state for the AI's tasks: a find's answer, an arm pose's end."""
        f = s.get("found")
        if isinstance(f, dict) and f.get("id"):
            t = self.ai_book.get(str(f["id"]))
            if t is not None and t.kind == "find" and not t.done:
                if f.get("error"):
                    self.ai_book.finish(t, "FAILED", self.wall_time(), f["error"], result=dict(error="not_found"))
                else:
                    self.found, self.t_found, self.found_epoch = f, self.now, self.loc_epoch
                    n = len(f.get("candidates", []))
                    self.ai_book.finish(t, "SUCCEEDED", self.wall_time(), f"{n} candidate{'s' * (n != 1)}",
                                        result={k: f.get(k) for k in ("candidates", "image_only", "query", "image",
                                                                       "frame", "t", "dropped", "hint", "pose")})
        t = self.ai_book.running(("arm_pose",))
        if t is None:
            return
        name = t.result["pose"]
        for text in new_events:
            if text.startswith(f"{name}: refused") or text.startswith(f"pose {name}:") or "isn't connected" in text:
                self.ai_book.finish(t, "FAILED", self.wall_time(), text, result=dict(error="arm_out"))
                return
            if text == f"moving to {name}":
                t.result["moving"] = True
        if t.result.get("moving") and s.get("source") == "hold":
            self.ai_book.finish(t, "SUCCEEDED", self.wall_time(), f"the arm is at {name}",
                                evidence=dict(joints_deg=s.get("joints"), out=s.get("out")))

    def _ai_pick_end(self, tidy: dict):
        """The tidy ended under an AI pick: its stages are the evidence (the robot has no bin check yet)."""
        t = self.ai_book.running(("pick",))
        if t is None:
            return
        stages = tidy.get("stages") or {}
        outcome = tidy.get("outcome") or tidy.get("reason") or ""
        ok = bool(stages.get("released"))
        err = None if ok else {"not_found": "not_found", "lost": "lost", "too_big": "out_of_limits"}.get(outcome,
                                                                                                         "failed")
        ev = dict(attempted=bool(stages.get("attempted")), grasped=bool(stages.get("grasped")), released=ok,
                  in_bin="uncertain", in_bin_reason="the robot has no bin check yet (released over the bin is what "
                                                    "it knows; the sim's truth comes from omni_mcp)",
                  outcome=outcome, tries=tidy.get("tries"))
        msg = f"object {t.args['object_id']}: " + ("released over the bin" if ok else (outcome or "not picked"))
        self.ai_book.finish(t, "SUCCEEDED" if ok else "FAILED", self.wall_time(), msg,
                            result=dict(error=err, in_bin="uncertain"), evidence=ev)

    def ai_cleanup(self) -> set:
        """What a restarted console must stop, for the AI tasks it found interrupted: "nav" (Nav2 keeps driving a goal
        whose client died), "arm" (omni_arm keeps running a tidy). Nothing resumes; the robot stops."""
        kinds = {t.kind for t in self.ai_book.tasks.values() if t.state == "INTERRUPTED" and t.t_end is None}
        return ({"nav"} if "goto" in kinds else set()) | ({"arm"} if "pick" in kinds else set())

    def ai_summary(self) -> dict:
        now = self.now
        last = next(reversed(self.ai_book.tasks.values()), None)
        live = self.ai is not None and now <= self.ai["until"]
        return dict(lease=live, session=self.ai["session"][:8] if live else None, owner=self.owner(now),
                    task=None if last is None else dict(task_id=last.id, kind=last.kind, state=last.state,
                                                        message=last.message),
                    found=None if self.found is None else dict(id=self.found.get("id"),
                                                                n=len(self.found.get("candidates", [])),
                                                                age_s=round(now - self.t_found, 1)),
                    limits=dict(max_speed=self.values["ai.max_speed"], max_turn=self.values["ai.max_turn"]))

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

    # -- Follow, Program, Map edit, Calibrate -------------------------------------------------------------------
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
        """The node's people (base frame, people.detect) from each lidar scan while FOLLOW runs."""
        if self.mission != "FOLLOW" or self.follower is None:
            return
        f = self.follower
        self.follow_cmd = f.tick(now, people, self.odom_pose)
        self.t_follow = now
        self.task = f.text()
        for e in f.events:
            self.event("info", e)
        f.events.clear()

    def _program(self, verb: str, a: dict) -> Reply:
        from omni_console.program import ProgramRunner
        if verb == "program_save":
            name = _name(a, "A program")
            self.programs[name] = _program_check(a["blocks"])
            self.programs_changed = True
            return Reply(True, f"Saved program '{name}'", self.schema())
        if verb == "program_delete":
            name = str(a["name"])
            if self.programs.pop(name, None) is None:
                return Reply(False, f"No program called '{name}'")
            self.programs_changed = True
            return Reply(True, f"Deleted program '{name}'", self.schema())
        if verb != "program_run":
            return Reply(False, f"Unknown command '{verb}'")
        name = str(a.get("name", "")).strip() or "unsaved"
        blocks = a.get("blocks", self.programs.get(name))
        if blocks is None:
            return Reply(False, f"No program called '{name}'")
        runner = ProgramRunner(name, _program_check(blocks), self.values["program.speed"],
                               math.radians(self.values["program.turn"]))
        if not runner.steps:
            return Reply(False, "The program is empty: add some blocks")
        if any(s[0] == "goto" for s in runner.steps):
            missing = [s[1] for s in runner.steps if s[0] == "goto" and s[1] not in self.places]
            if missing:
                return Reply(False, f"No saved place called '{missing[0]}' on this map")
            bad = self._need_nav()
            if bad:
                return bad
        r = self._enter("PROGRAM", f"Program '{name}': starting", "info")
        self.runner = runner
        return Reply(True, f"Running '{name}' ({len(runner.steps)} steps)", None, r.effects)

    def _zones(self, verb: str, a: dict) -> Reply:
        from omni_console.zones import check_edge, check_zone
        if self.map_name in (None,) and self.slam.get("mode") is None:
            return Reply(False, "No map: no-go zones and edges are drawn on a map")
        if verb == "zone_save":
            name = _name(a, "A no-go zone")
            self.zones[name] = check_zone(a["points"])
            msg = f"No-go zone '{name}' saved: Nav2 plans around it"
        elif verb == "edge_save":
            name = _name(a, "An edge")
            self.edges[name] = check_edge(a["points"], a.get("height_mm", 10.0), a.get("speed", 1.0),
                                          a.get("kind", "rug"))
            msg = f"Edge '{name}' saved: {self.edges[name]['speed']:g} m/s within 35 cm of it"
        else:
            book = self.zones if verb == "zone_delete" else self.edges
            name = str(a["name"])
            if book.pop(name, None) is None:
                return Reply(False, f"Nothing called '{name}'")
            msg = f"Deleted '{name}'"
        self.places_changed = self.zones_changed = True
        return Reply(True, msg, self.schema(), (("masks",),))

    def _rooms(self, verb: str, a: dict) -> Reply:
        """A room's outline on the map (map frame), by name: labels for "drive to the kitchen"; not a no-go zone."""
        from omni_console.zones import check_zone
        if verb == "room_save":
            name = _name(a, "A room")
            self.rooms[name] = check_zone(a["points"])
            msg = f"Room '{name}' saved ({len(self.rooms[name])} corners)"
        else:
            name = str(a["name"])
            if self.rooms.pop(name, None) is None:
                return Reply(False, f"No room called '{name}'")
            msg = f"Deleted room '{name}'"
        self.places_changed = True
        return Reply(True, msg, self.schema())

    def _tf(self, verb: str, a: dict) -> Reply:
        """Read-only: a frame's planar pose in another (x, y m, yaw deg), or the frames the console knows. The console
        keeps the newest transform of each frame in 2D (TfLite): enough to place things on the map."""
        tf = self.tf
        if tf is None:
            return self._err("no_robot", "No transforms yet")
        if verb == "frames":
            return Reply(True, "", dict(frames={c: e[0] for c, e in sorted(tf.tf.items())}))
        parent, child = str(a.get("parent", "map")), str(a["child"])
        p = tf.lookup(parent, child)
        if p is None:
            return self._err("not_found", f"No transform from '{parent}' to '{child}' (the frames: verb frames; "
                                          f"the child must lie under the parent in the tree)")
        return Reply(True, "", dict(parent=parent, child=child, x=round(p[0], 4), y=round(p[1], 4),
                                    yaw_deg=round(math.degrees(math.remainder(p[2], 2 * math.pi)), 2),
                                    stamp=round(p[3], 3), planar=True))

    def calib_state(self) -> dict:
        c = self.calib_run
        return dict(results=self.calib_results, shots=self.calib_shots, rpms=list(LAUNCHER_RPMS),
                    run=None if c is None else dict(proc=c.proc, phase=c.phase, text=c.text(), odom=c.odom,
                                                    result=c.result))

    def _calib_save(self, proc: str, result: dict):
        self.calib_results.setdefault(proc, []).append(dict(result, t=round(self.wall_time(), 1)))
        self.calib_results[proc] = self.calib_results[proc][-20:]
        self.calib_changed = True
        self.event("info", f"Calibration ({proc}): {result.get('summary', 'saved')}")

    def wall_time(self) -> float:
        import time
        return time.time()

    def _calib(self, verb: str, a: dict) -> Reply:
        from omni_console.calibration import PROCS, DriveCal, launcher_fit
        if verb == "calib":
            return Reply(True, "", self.calib_state())
        if verb == "calib_start":
            proc = str(a["proc"])
            if proc not in ("odom_line", "odom_spin", "imu_level"):
                return Reply(False, f"'{proc}' isn't a driving calibration (one of odom_line, odom_spin, imu_level)")
            if self.odom_pose is None:
                return Reply(False, "No odometry from the robot yet")
            r = self._enter("CALIBRATE", f"Calibrating: {proc}", "info")
            self.calib_run = DriveCal(proc, self.values["calib.distance"], int(self.values["calib.turns"]))
            return Reply(True, r.message, self.calib_state(), r.effects)
        if verb == "calib_measure":
            c = self.calib_run
            if c is None:
                return Reply(False, "No calibration is waiting for a measurement")
            res = c.measured(float(a["value"]))
            self._calib_save(c.proc, res)
            self.task = f"Calibration done: {res['summary']}"
            return Reply(True, res["summary"], self.calib_state())
        if verb == "calib_shot":
            if self.mission != "THROWER":
                return Reply(False, "The launcher calibration runs in the Thrower bench mode: press 'Take control'")
            d = float(a["distance_m"])
            if not 0.3 < d < 15:
                return Reply(False, "The landing distance in metres, from the robot's centre")
            shot = dict(rpm=float(a.get("rpm", self.thrower["rpm"])), tilt_deg=float(a.get("tilt_deg", self.thrower["tilt"])),
                        distance_m=d)
            if shot["rpm"] <= 0:
                return Reply(False, "The flywheels were off: set the rpm and fire first")
            self.calib_shots.append(shot)
            return Reply(True, f"Shot {len(self.calib_shots)}: {shot['rpm']:.0f} rpm, {d:.2f} m", self.calib_state())
        if verb == "calib_fit":
            from omni_catch.flight import BallFlight
            from omni_catch.robot_config import RobotConfig
            rc = RobotConfig()
            b = rc.ball
            fl = BallFlight(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])
            res = launcher_fit(self.calib_shots, rc, fl)
            self._calib_save("launcher", res)
            self.calib_shots = []
            return Reply(True, res["summary"], self.calib_state())
        if verb == "calib_record":
            proc = str(a["proc"])
            if proc not in PROCS:
                return Reply(False, f"No calibration '{proc}'")
            v = float(a["value"])
            self._calib_save(proc, dict(value=v, note=str(a.get("note", ""))[:200],
                                        summary=f"{proc}: {v:g}" + (" ms" if proc == "camera_latency" else "")))
            return Reply(True, "Recorded", self.calib_state())
        if verb == "calib_clear":
            proc = str(a.get("proc", ""))
            if proc == "shots":
                self.calib_shots = []
            elif self.calib_results.pop(proc, None) is not None:
                self.calib_changed = True
            return Reply(True, "Cleared", self.calib_state())
        return Reply(False, f"Unknown command '{verb}'")

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
        if self.mission == "TIDY" and mission != "TIDY":
            fx.append(("arm", dict(verb="stop")))       # the arm stows (a drop under way finishes first)
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
        self.mission, self.task = mission, task
        self.progress = 0.0 if mission == "GOTO" else None
        self.event(level, task)
        return Reply(True, task, None, tuple(fx))

    # -- Explore and Patrol: what to do next ---------------------------------------------------------------------
    def tick(self, now: float, grid=None) -> tuple:
        """Every ~0.5 s from the node (grid: explore.Grid of the current map while exploring). Returns effects."""
        self.now = now
        fx = self._ai_tick(now)
        if fx:
            return tuple(fx)
        if self.mission == "EXPLORE" and self.explore is not None:
            return tuple(self._explore_tick(now, grid))
        if self.mission == "PATROL" and self.patrol is not None:
            return tuple(self._patrol_tick(now))
        return ()

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
        if MODE_OF[self.mission] != mode:
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
        return dict(version=VERSION, missions=list(MISSIONS),
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
                    kid=self.values["profile.kid"], caps=dict(self.caps), version=VERSION,
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
                    arm=self._arm_summary(), ai=self.ai_summary())

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
