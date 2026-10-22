"""The operator console's robot side, ROS-free: missions, settings and saved places. mission_node.py is the ROS node
around it; the web page (web/) is a thin skin over its status and its one command service.

  IDLE     the MCU disarms (ramped stop first)                         mode_manager IDLE
  MANUAL   the console's stick drives (Foxglove / the joystick still can)   TELEOP
  GOTO     Nav2 drives to a goal on the map                            TELEOP
  CATCH    the catch behavior plays                                    CATCH
A command returns effects (set_mode, nav_goal, nav_cancel, nav_limits, param) that the node carries out; the core
never blocks and never touches ROS. The MCU stays the safety authority: every speed here is a request it caps by its
own profile (adult 2 m/s, kid 0.8 m/s).
"""

from __future__ import annotations

import json
import math
import os
import tempfile
from collections import deque
from dataclasses import dataclass

VERSION = "0.1.0"                  # the console protocol: the page warns when it differs from the robot's
MISSIONS = ("IDLE", "MANUAL", "GOTO", "CATCH")
MODE_OF = {"IDLE": "IDLE", "MANUAL": "TELEOP", "GOTO": "TELEOP", "CATCH": "CATCH"}   # mission -> mode_manager mode
MANUAL_TIMEOUT = 0.3               # s: the console's stick stops counting (dead man released, tab closed, Wi-Fi gone)
ZERO_TAIL = 0.3                    # s of zero twist after that, then silence (Nav2 / Foxglove / the joystick may drive)
KID_SPEED = 0.8                    # m/s, the firmware's kid cap (shown, not enforced, here)
VERBS = ("ping", "schema", "stop", "mission", "goto", "cancel", "set", "place_save", "place_delete")


@dataclass(frozen=True)
class Setting:
    key: str
    tabs: tuple
    label: str
    default: float | bool
    unit: str = ""
    lo: float | None = None
    hi: float | None = None
    step: float | None = None
    hint: str = ""

    def coerce(self, v):
        if isinstance(self.default, bool):
            if isinstance(v, str):
                v = v.strip().lower() in ("1", "true", "yes", "on")
            return bool(v)
        v = float(v)
        if not math.isfinite(v) or (self.lo is not None and v < self.lo) or (self.hi is not None and v > self.hi):
            raise ValueError(f"{self.label}: {self.lo:g} to {self.hi:g} {self.unit}".rstrip())
        return v

    def describe(self, value) -> dict:
        d = dict(key=self.key, tabs=list(self.tabs), label=self.label, kind="bool" if isinstance(self.default, bool)
                 else "float", default=self.default, value=value, unit=self.unit, hint=self.hint)
        if not isinstance(self.default, bool):
            d.update(min=self.lo, max=self.hi, step=self.step)
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
    Setting("goto.speed", ("goto",), "Speed", 0.6, "m/s", 0.1, 0.8, 0.05,
            "Nav2's top speed on the way (its planner tops out at 0.8 m/s)."),
    Setting("goto.turn", ("goto",), "Turn rate", 90.0, "deg/s", 20.0, 105.0, 5.0, "Nav2's top turn rate."),
    Setting("profile.kid", ("manual", "catch"), "Kid profile", False,
            hint="The firmware's kid limits in every mode: 0.8 m/s, gentle accelerations, lobbed returns."),
    Setting("catch.returns", ("catch",), "Throw back", True,
            hint="Throw the ball back to the thrower after each catch. Off: the robot keeps the ball."),
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


class Console:
    def __init__(self, settings: dict | None = None, places: dict | None = None):
        self.values = {s.key: s.default for s in SETTINGS}
        for k, v in (settings or {}).items():
            if k in SETTING:
                try:
                    self.values[k] = SETTING[k].coerce(v)
                except (TypeError, ValueError):
                    pass
        self.places = {}
        for name, p in (places or {}).items():
            try:
                self.places[str(name)] = [float(p[0]), float(p[1]), float(p[2])]
            except (TypeError, ValueError, IndexError):
                pass
        self.mission = "IDLE"
        self.adopted = False               # the first /robot_mode sets the mission (a step may start in CATCH)
        self.robot_mode = None
        self.robot_kid = None
        self.pending_mode = None           # a set_mode on its way
        self.task = "Waiting for the robot"
        self.progress = None
        self.goal = None                   # dict(x, y, yaw, name, seq)
        self.goal_seq = 0
        self.d0 = None
        self.caps = {"nav": False, "catch": False, "collision": False}
        self.manual_in = None              # (vx, vy, wz, boost) normalized to -1..1
        self.t_manual = -math.inf
        self.events = deque(maxlen=200)
        self.settings_changed = self.places_changed = False

    # -- events ---------------------------------------------------------------------------------------------------
    def event(self, level: str, text: str):
        self.events.append((level, text))

    def pop_events(self) -> list:
        out = list(self.events)
        self.events.clear()
        return out

    # -- commands ---------------------------------------------------------------------------------------------
    def command(self, verb: str, args: dict | None = None) -> Reply:
        a = args or {}
        try:
            if verb == "ping":
                return Reply(True, "pong", {"version": VERSION})
            if verb == "schema":
                return Reply(True, "", self.schema())
            if verb == "stop":
                return self._enter("IDLE", "Stopped from the console", "warn")
            if verb == "mission":
                return self._mission(str(a.get("name", "")).upper())
            if verb == "goto":
                return self._goto(float(a["x"]), float(a["y"]), float(a.get("yaw", 0.0)), str(a.get("name", "")))
            if verb == "cancel":
                if self.mission != "GOTO":
                    return Reply(True, "No goal to cancel")
                return self._enter("MANUAL", "Goal cancelled", "info")
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
        except (KeyError, TypeError, ValueError) as e:
            return Reply(False, f"{verb}: bad arguments ({e})")
        return Reply(False, f"Unknown command '{verb}' (one of {', '.join(VERBS)})")

    def _mission(self, name: str) -> Reply:
        if name == "GOTO":
            return Reply(False, "Pick a goal on the map (Go To tab) to start Go To")
        if name not in MISSIONS:
            return Reply(False, f"Unknown mode '{name}'")
        if name == "CATCH" and not self.caps["catch"]:
            return Reply(False, "Catch needs the catch code running: start 'Play catch' in the Launcher")
        text = {"IDLE": "Motors off", "MANUAL": "Driving by hand", "CATCH": "Playing catch"}[name]
        return self._enter(name, text, "info")

    def _goto(self, x: float, y: float, yaw: float, name: str) -> Reply:
        if not self.caps["nav"]:
            return Reply(False, "Go To needs navigation running: start 'Navigate (Nav2)' in the Launcher")
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return Reply(False, "The goal is not a number")
        fx = []
        if self.mission == "GOTO":
            fx.append(("nav_cancel",))
        self.goal_seq += 1
        self.goal = dict(x=x, y=y, yaw=yaw, name=name, seq=self.goal_seq)
        self.d0 = None
        label = f"'{name}'" if name else f"({x:.1f}, {y:.1f})"
        r = self._enter("GOTO", f"Going to {label}", "info", cancel_nav=False)
        fx += list(r.effects)
        fx.append(("nav_limits", self.values["goto.speed"], math.radians(self.values["goto.turn"])))
        fx.append(("nav_goal", x, y, yaw, self.goal_seq))
        return Reply(True, r.message, None, tuple(fx))

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
        elif key.startswith("goto.") and self.mission == "GOTO":
            fx.append(("nav_limits", self.values["goto.speed"], math.radians(self.values["goto.turn"])))
        return Reply(True, f"{s.label}: {self._fmt(s, v)}", self.schema(), tuple(fx))

    @staticmethod
    def _fmt(s: Setting, v) -> str:
        return ("on" if v else "off") if isinstance(v, bool) else f"{v:g} {s.unit}".rstrip()

    def _enter(self, mission: str, task: str, level: str, cancel_nav: bool = True) -> Reply:
        fx = []
        if cancel_nav and self.mission == "GOTO":
            fx.append(("nav_cancel",))
            self.goal = None
        mode = MODE_OF[mission]
        if mode != self.robot_mode or self.pending_mode is not None:
            fx.append(("set_mode", mode, self.values["profile.kid"]))
            self.pending_mode = mode
        if mission != "MANUAL":
            self.manual_in = None
        self.mission, self.task = mission, task
        self.progress = None if mission != "GOTO" else 0.0
        self.event(level, task)
        return Reply(True, task, None, tuple(fx))

    # -- what the robot reports ----------------------------------------------------------------------------------
    def on_robot_mode(self, text: str) -> tuple:
        """/robot_mode, e.g. "CATCH adult". Adopts the first one; afterwards a change nobody here asked for (another
        client of /set_mode) moves the mission along. Returns effects (a nav cancel when Go To loses its mode)."""
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
        if not self.adopted:
            self.adopted = True
            self.mission = {"CATCH": "CATCH", "TELEOP": "MANUAL"}.get(mode, "IDLE")
            self.task = {"CATCH": "Playing catch", "MANUAL": "Ready to drive", "IDLE": "Motors off"}[self.mission]
            self.event("info", f"Robot is in {mode} ({'kid' if kid else 'adult'} profile)")
            return ()
        if MODE_OF[self.mission] != mode:
            fx = (("nav_cancel",),) if self.mission == "GOTO" else ()
            self.goal = None
            self.mission = {"CATCH": "CATCH", "TELEOP": "MANUAL"}.get(mode, "IDLE")
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
        if self.mission != "GOTO" or self.goal is None or seq != self.goal["seq"]:
            return
        if self.d0 is None or distance > self.d0:
            self.d0 = max(distance, 1e-3)
        self.progress = max(0.0, min(1.0, 1.0 - distance / self.d0))
        name = self.goal["name"] or f"({self.goal['x']:.1f}, {self.goal['y']:.1f})"
        self.task = f"Going to {name}: {distance:.1f} m left"

    def on_nav_done(self, seq: int, outcome: str):
        """outcome: succeeded | aborted | canceled | rejected. A goal replaced or cancelled here reports nothing."""
        if self.mission != "GOTO" or self.goal is None or seq != self.goal["seq"]:
            return
        name = self.goal["name"] or f"({self.goal['x']:.1f}, {self.goal['y']:.1f})"
        self.goal = None
        self.mission, self.progress = "MANUAL", None
        if outcome == "succeeded":
            self.task = f"Arrived at {name}"
            self.event("info", self.task)
        else:
            why = {"aborted": "Nav2 gave up (blocked or no path)", "rejected": "Nav2 refused the goal",
                   "canceled": "cancelled by Nav2"}.get(outcome, outcome)
            self.task = f"Could not reach {name}: {why}"
            self.event("warn", self.task)

    def set_caps(self, **caps):
        self.caps.update(caps)
        if self.mission == "GOTO" and not self.caps["nav"]:
            self.goal = None
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
                    places={k: v for k, v in sorted(self.places.items())})

    def summary(self) -> dict:
        g = None if self.goal is None else {k: self.goal[k] for k in ("x", "y", "yaw", "name")}
        return dict(mission=self.mission, task=self.task, progress=self.progress, goal=g, robot_mode=self.robot_mode,
                    kid=self.values["profile.kid"], caps=dict(self.caps), version=VERSION)


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
