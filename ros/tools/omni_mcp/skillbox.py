"""Skills (voice_brain_handoff_2026-11-23.md): small routines built from the toolkit's actions, run in their own
process with one `robot` object, saved for next time. The voice brain (omni_face brain.py) sees each as a tool; the
inventor (inventor.py) writes new ones.

A skill is a Python file (skill_check.py says what it may contain):

    \"\"\"Come to me: drive to about 1 m in front of the nearest person, facing them.\"\"\"
    SKILL = dict(name="come_here", says=["come to me", "come over here"], params={}, moves=True, max_s=120)

    def run(robot):
        p = robot.nearest_person()
        if p is None:
            return "I can't see anyone."
        robot.approach(x=p["x"], y=p["y"], stop_m=1.0)
        return "Here I am."

params: the JSON-schema properties of run's keyword arguments (all optional unless listed in SKILL["required"]).
run returns what happened, in a sentence (or a dict); the brain turns it into a spoken line.

Hand-written skills live in skills/ beside this file; invented ones on the PC (USER_DIR, with a .json beside each: the
request, when, tried, kept). The brain asks before a skill's first run.

Running: run_skill() starts skill_child.py (`python -I`), answers its robot calls with an API (RealAPI: the robot
through robot_client, under the voice brain's AI lease; FakeAPI: a made-up house for dry runs and tests), and kills it
at its time limit or when the brain's job is stopped. Every motion is a console AI verb, so the robot's own rules
apply: the lease, the AI speed caps, the collision monitor, the arm stowed for driving, and a person's stop.
"""

from __future__ import annotations

import json
import math
import os
import queue
import re
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from skill_check import metadata, problems  # noqa: E402

BUILTIN_DIR = HERE / "skills"
USER_DIR = Path(os.environ.get("OMNI_SKILLS_DIR", Path.home() / ".local" / "share" / "omni_voice" / "skills"))
PHOTOS = Path(os.environ.get("OMNI_PHOTOS", Path.home() / "omni_photos"))
NAME = re.compile(r"^[a-z][a-z0-9_]{2,40}$")
MAX_S = 900.0
TERMINAL = ("SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT", "INTERRUPTED")
STOP_ERRORS = ("owner_conflict", "no_lease", "fault", "no_robot")

# what `robot.` means inside a skill (keyword arguments only): the inventor's reference and the child's whitelist
API = {
    "status": "status() -> {mission, owner, battery_pct, source, pose}: what the robot is doing now",
    "pose": "pose() -> {x, y, yaw_deg, frame} (map frame, m, deg; + yaw = left of +x) or None",
    "places": "places() -> {name: {x, y, yaw_deg}}: the saved places on the map",
    "rooms": "rooms() -> {name: {x, y}}: the rooms' centres on the map",
    "people": "people() -> [{status, range_m, bearing_deg, x, y}], nearest first: status \"person\" (the camera saw "
              "a person) or \"candidate\" (lidar legs only); bearing + = to the left; x, y map frame (None without a "
              "map pose). Not identities",
    "nearest_person": "nearest_person() -> the person \"me\" means: the nearest confirmed person, else the nearest "
                      "candidate, else None",
    "move": "move(steps, speed=None, turn_rate=None) -> {ok, state, message, evidence}: robot_move's blocks, e.g. "
            "[{\"do\": \"forward\", \"m\": 0.5}, {\"do\": \"turn\", \"deg\": 90}, {\"do\": \"spin\", \"turns\": 1}]: "
            "forward/back/left/right {m 0.05-3}, turn {deg -360..360, + left}, spin {turns 0.25-5}, wiggle {}, wait "
            "{s}, repeat {times, body}; waits until done. Driving needs navigation running",
    "turn": "turn(deg) -> {ok, ...}: turn in place (+ = left), waits",
    "goto": "goto(place=None, x=None, y=None, yaw_deg=None) -> {ok, state, message, evidence}: drive there with "
            "navigation (a saved place, or a map point; no yaw: face the way it drove) and wait",
    "approach": "approach(x, y, stop_m=1.0) -> {ok, ...}: drive to stop_m short of the map point, facing it, and "
                "wait (turns to face it if it is already that close)",
    "find": "find(color=None, shape=\"any\") -> [{id, color, shape, range_m, bearing_deg, score}]: things on the "
            "floor ahead (colours red orange yellow green blue purple pink white grey black; shapes square round "
            "long any), best first",
    "pick": "pick(object_id) -> {ok, ...}: pick one thing of the last find into the bin and wait",
    "mission": "mission(name, args=None) -> {ok, ...}: one of the robot's own missions, waited for: fetch {color, "
               "shape, to: me|here}, message {text, rooms}, tower {max_blocks}, dance {routine: groove|party|robot}, "
               "goalie {seconds}",
    "arm_pose": "arm_pose(name) -> {ok, ...}: stow | ready | high | drop | look (stow before driving), waits",
    "say": "say(text, feeling=None, intensity=0.6): the robot says a line (up to 200 characters), with a feeling",
    "express": "express(feeling, intensity=0.6): show a feeling (face, a chirp, a gesture if the body is free); "
               "feelings: " + "neutral content happy excited proud affection relieved curious surprised startled "
                              "focused effort worried frustrated sad tired bored",
    "gesture": "gesture(base=None, arm=None, feeling=\"happy\"): a body gesture when that part is free (never in a "
               "task): base wiggle shimmy bounce hop recoil lean huff shrink sag settle; arm nod tilt perk "
               "flourish flinch shake droop shrink settle chatter",
    "photo": "photo(camera=\"front\") -> {path, width, height}: take a picture and save it (front, wrist, "
             "d435_color)",
    "scan": "scan() -> [{bearing_deg, nearest_m}]: the lidar's nearest return per 10 deg sector, robot frame",
    "wait": "wait(s): pause up to 60 s a call (a person's stop still ends the skill)",
    "now": "now() -> seconds: a clock for timing",
    "log": "log(text): a line in the brain's log (print does the same)",
}


class StopSkill(Exception):
    """A person stopped the robot, or the brain ended the skill."""


class RobotRefused(Exception):
    """A robot call was refused (the message says why): RobotError inside the skill."""


def _short(task: dict) -> dict:
    d = task.get("data") or {}
    return dict(ok=d.get("state") == "SUCCEEDED", state=d.get("state"), message=d.get("message") or task.get("message"),
                evidence=d.get("evidence"), result=d.get("result"))


class RealAPI:
    """A skill's robot calls on the real (or simulated) robot, through robot_client.Robot under the brain's lease."""

    def __init__(self, robot, stop: threading.Event | None = None, log=None):
        self.robot = robot
        self.stop = stop or threading.Event()
        self.logf = log or (lambda m: None)
        self.task_id = None                       # the motion task running now (cancelled if the skill is killed)
        self.lines: list = []

    # -- plumbing ----------------------------------------------------------------------------------------------
    def _check(self):
        if self.stop.is_set():
            raise StopSkill("stopped")

    def _task(self, r: dict, limit: float = 600.0) -> dict:
        """A motion verb's reply -> wait for its task -> {ok, state, ...}; refusals raise."""
        self._check()
        if not r.get("ok"):
            err = (r.get("data") or {}).get("error")
            if err in STOP_ERRORS:
                raise StopSkill(r.get("message") or err)
            raise RobotRefused(r.get("message") or err or "refused")
        tid = (r.get("data") or {}).get("task_id")
        if not tid:
            return dict(ok=True, state="SUCCEEDED", message=r.get("message"))
        self.task_id = tid
        t = self.robot.wait_task(tid, limit, stop=self.stop.is_set)
        self.task_id = None
        out = _short(t)
        if out["state"] in ("CANCELLED", "INTERRUPTED"):
            raise StopSkill(f"the task was {out['state'].lower()}: {out['message']}")
        if self.stop.is_set():
            self.robot.cancel(tid)
            raise StopSkill("stopped")
        return out

    def _pose(self):
        p, frame = self.robot.pose()
        return None if p is None else dict(x=round(p[0], 3), y=round(p[1], 3), yaw_deg=round(math.degrees(p[2]), 1),
                                           frame=frame)

    # -- the API -------------------------------------------------------------------------------------------------
    def status(self):
        d = self.robot.status().get("data") or {}
        return dict(mission=d.get("mission"), owner=(d.get("ai") or {}).get("owner"), source=d.get("source"),
                    battery_pct=(d.get("mcu") or {}).get("battery_pct"), pose=self._pose())

    def pose(self):
        return self._pose()

    def places(self):
        return (self.robot.places().get("data") or {}).get("places") or {}

    def rooms(self):
        rooms = (self.robot.places().get("data") or {}).get("rooms") or {}
        return {k: dict(x=v["centre"][0], y=v["centre"][1]) for k, v in rooms.items()}

    def people(self):
        r, _ = self.robot.people()
        ppl = (r.get("data") or {}).get("people") or []
        out = [dict(status=p.get("status", "candidate"), range_m=p.get("range_m"), bearing_deg=p.get("bearing_deg"),
                    x=p.get("x"), y=p.get("y")) for p in ppl]
        return sorted(out, key=lambda p: p["range_m"] if p["range_m"] is not None else 99.0)

    def nearest_person(self):
        ppl = self.people()
        sure = [p for p in ppl if p["status"] == "person"]
        return (sure or ppl or [None])[0]

    def move(self, steps, speed=None, turn_rate=None):
        return self._task(self.robot.move(steps, speed, turn_rate), 300.0)

    def turn(self, deg):
        deg = float(deg)
        if abs(deg) < 2.0:
            return dict(ok=True, state="SUCCEEDED", message="already facing it")
        return self.move([{"do": "turn", "deg": max(-360.0, min(360.0, deg))}])

    def goto(self, place=None, x=None, y=None, yaw_deg=None):
        if place is None and yaw_deg is None and x is not None and y is not None:
            p = self._pose()
            yaw_deg = 0.0 if p is None else math.degrees(math.atan2(float(y) - p["y"], float(x) - p["x"]))
        return self._task(self.robot.goto(place, x, y, yaw_deg or 0.0), 300.0)

    def approach(self, x, y, stop_m=1.0):
        p = self._pose()
        if p is None:
            raise RobotRefused("no pose on the map: can't drive to a point")
        dx, dy = float(x) - p["x"], float(y) - p["y"]
        d = math.hypot(dx, dy)
        heading = math.degrees(math.atan2(dy, dx))
        if d <= float(stop_m) + 0.15:
            return self.turn((heading - p["yaw_deg"] + 180.0) % 360.0 - 180.0)
        k = (d - float(stop_m)) / d
        return self.goto(x=p["x"] + dx * k, y=p["y"] + dy * k, yaw_deg=heading)

    def find(self, color=None, shape="any"):
        self._check()
        r, _ = self.robot.find(color, shape)
        if not r.get("ok"):
            return []
        out = []
        for c in (r.get("data") or {}).get("candidates") or []:
            out.append(dict(id=c.get("id"), color=(c.get("color") or {}).get("name"),
                            shape=(c.get("shape") or {}).get("class"), range_m=c.get("range_m") or c.get("range"),
                            bearing_deg=c.get("bearing_deg") or c.get("bearing"), score=c.get("score")))
        return out

    def pick(self, object_id):
        return self._task(self.robot.pick(int(object_id)), 240.0)

    def mission(self, name, args=None):
        if name not in ("fetch", "message", "tower", "dance", "goalie"):
            raise RobotRefused("mission: fetch, message, tower, dance or goalie")
        return self._task(self.robot.mission(name, dict(args or {})), 900.0)

    def arm_pose(self, name):
        return self._task(self.robot.arm_pose(name), 60.0)

    def say(self, text, feeling=None, intensity=0.6):
        self._check()
        text = str(text)[:200]
        self.lines.append(text)
        r = self.robot.express(feeling, float(intensity), text) if feeling else self.robot.say(text)
        return bool(r.get("ok"))

    def express(self, feeling, intensity=0.6):
        self._check()
        return bool(self.robot.express(feeling, float(intensity)).get("ok"))

    def gesture(self, base=None, arm=None, feeling="happy"):
        self._check()
        r = self.robot.gesture(feeling, 0.7, base, arm)
        return dict(ok=bool(r.get("ok")), message=r.get("message"))

    def photo(self, camera="front"):
        self._check()
        r, jpeg = self.robot.look(camera)
        if not jpeg:
            raise RobotRefused(r.get("message") or "no picture")
        PHOTOS.mkdir(parents=True, exist_ok=True)
        path = PHOTOS / time.strftime(f"omni-%Y%m%d-%H%M%S-{camera}.jpg")
        path.write_bytes(jpeg)
        d = r.get("data") or {}
        return dict(path=str(path), width=d.get("width"), height=d.get("height"))

    def scan(self):
        r, _ = self.robot.scan(picture=False)
        return [dict(bearing_deg=s.get("bearing_deg"), nearest_m=s.get("nearest_m"))
                for s in (r.get("data") or {}).get("sectors") or []]

    def wait(self, s):
        t_end = time.monotonic() + max(0.0, min(60.0, float(s)))
        while time.monotonic() < t_end:
            self._check()
            time.sleep(min(0.2, max(0.0, t_end - time.monotonic())))
        return True

    def now(self):
        return round(time.monotonic(), 3)

    def log(self, text):
        self.logf(f"skill: {str(text)[:300]}")
        return True


class FakeAPI(RealAPI):
    """A made-up house for dry runs and tests: a robot at the origin on a map, four places, one person 2 m away to
    the front left, a red block on the floor ahead; every motion succeeds at once. calls lists what the skill did."""

    def __init__(self, people=None, found=None, fail: dict | None = None):
        self.stop = threading.Event()
        self.calls: list = []
        self.lines: list = []
        self.t = 0.0
        self.p = [0.0, 0.0, 0.0]                         # x, y, yaw deg
        self._people = people if people is not None else [dict(status="person", range_m=2.0, bearing_deg=30.0,
                                                               x=1.73, y=1.0)]
        self._found = found if found is not None else [dict(id=7, color="red", shape="square", range_m=0.8,
                                                            bearing_deg=-5.0, score=0.9)]
        self.fail = fail or {}                         # method -> "refused" | "failed" | "stopped"
        self.task_id = None
        self.logf = lambda m: None

    def _rec(self, method, **kw):
        self._check()
        self.calls.append((method, kw))
        how = self.fail.get(method)
        if how == "stopped":
            raise StopSkill("a person stopped the robot")
        if how == "refused":
            raise RobotRefused(f"{method} refused (fake)")
        self.t += 1.0
        return dict(ok=how != "failed", state="FAILED" if how == "failed" else "SUCCEEDED", message=method,
                    evidence={})

    def status(self):
        return dict(mission="MANUAL", owner=None, source="fake", battery_pct=80, pose=self.pose())

    def pose(self):
        return dict(x=self.p[0], y=self.p[1], yaw_deg=self.p[2], frame="map")

    def _pose(self):
        return self.pose()

    def places(self):
        return {"kitchen": dict(x=3.0, y=1.0, yaw_deg=0.0), "living room": dict(x=0.0, y=2.0, yaw_deg=90.0),
                "home": dict(x=0.0, y=0.0, yaw_deg=0.0), "front door": dict(x=5.0, y=0.0, yaw_deg=180.0)}

    def rooms(self):
        return {"kitchen": dict(x=3.0, y=1.0), "living room": dict(x=0.0, y=2.5)}

    def people(self):
        self.calls.append(("people", {}))
        return [dict(p) for p in self._people]

    def move(self, steps, speed=None, turn_rate=None):
        if not isinstance(steps, list) or not steps:
            raise RobotRefused("move: steps is a non-empty list of blocks")
        r = self._rec("move", steps=steps)
        for s in steps:
            if s.get("do") == "turn":
                self.p[2] = (self.p[2] + float(s.get("deg", 0)) + 180.0) % 360.0 - 180.0
        return r

    def goto(self, place=None, x=None, y=None, yaw_deg=None):
        if place is not None and place not in self.places():
            raise RobotRefused(f"Unknown place '{place}'")
        if place is None and (x is None or y is None):
            raise RobotRefused("goto: a place, or x and y")
        r = self._rec("goto", place=place, x=x, y=y, yaw_deg=yaw_deg)
        if place is not None:
            q = self.places()[place]
            self.p = [q["x"], q["y"], q["yaw_deg"]]
        else:
            self.p = [float(x), float(y), float(yaw_deg if yaw_deg is not None else self.p[2])]
        return r

    def find(self, color=None, shape="any"):
        self._rec("find", color=color, shape=shape)
        return [dict(f) for f in self._found if color in (None, f["color"])]

    def pick(self, object_id):
        return self._rec("pick", object_id=object_id)

    def mission(self, name, args=None):
        if name not in ("fetch", "message", "tower", "dance", "goalie"):
            raise RobotRefused("mission: fetch, message, tower, dance or goalie")
        return self._rec("mission", name=name, args=args)

    def arm_pose(self, name):
        if name not in ("stow", "ready", "high", "drop", "look"):
            raise RobotRefused("arm_pose: stow, ready, high, drop or look")
        return self._rec("arm_pose", name=name)

    def say(self, text, feeling=None, intensity=0.6):
        self.calls.append(("say", dict(text=text, feeling=feeling)))
        self.lines.append(str(text))
        return True

    def express(self, feeling, intensity=0.6):
        self.calls.append(("express", dict(feeling=feeling)))
        return True

    def gesture(self, base=None, arm=None, feeling="happy"):
        self.calls.append(("gesture", dict(base=base, arm=arm)))
        return dict(ok=True, message="")

    def photo(self, camera="front"):
        self._rec("photo", camera=camera)
        return dict(path=f"/fake/omni-photo-{camera}.jpg", width=640, height=480)

    def scan(self):
        return [dict(bearing_deg=b, nearest_m=2.5) for b in range(-175, 180, 10)]

    def wait(self, s):
        self._check()
        self.calls.append(("wait", dict(s=s)))
        self.t += max(0.0, min(60.0, float(s)))
        return True

    def now(self):
        return self.t

    def log(self, text):
        self.calls.append(("log", dict(text=text)))
        return True


def run_skill(code: str, args: dict | None = None, api: RealAPI | None = None, max_s: float = 180.0,
              stop: threading.Event | None = None) -> dict:
    """Run a skill in its own process -> {"outcome": done | failed | stopped | timeout, "result", "said", "s"}.
    api: RealAPI (the robot) or FakeAPI (a dry run; the default). stop: set it to end the skill now."""
    api = api if api is not None else FakeAPI()
    if stop is not None:
        api.stop = stop
    bad = problems(code)
    if bad:
        return dict(outcome="failed", result="refused: " + "; ".join(bad), said=[], s=0.0)
    t0 = time.monotonic()
    max_s = max(1.0, min(MAX_S, float(max_s)))
    child = subprocess.Popen([sys.executable, "-I", str(HERE / "skill_child.py")], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding="utf-8",
                             cwd=tempfile.gettempdir(),
                             creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    lines: queue.Queue = queue.Queue()
    threading.Thread(target=lambda: [lines.put(x) for x in child.stdout] + [lines.put(None)], daemon=True).start()
    err_tail: list = []
    threading.Thread(target=lambda: [err_tail.append(x) for x in child.stderr], daemon=True).start()

    def reply(obj):
        try:
            child.stdin.write(json.dumps(obj, default=str) + "\n")
            child.stdin.flush()
        except OSError:
            pass
    reply(dict(code=code, args=args or {}, api=sorted(API), check_dir=str(HERE)))
    out = None
    while out is None:
        left = max_s - (time.monotonic() - t0)
        if left <= 0:
            out = dict(outcome="timeout", result=f"the skill ran out of time ({max_s:.0f} s)")
            break
        if api.stop.is_set():
            out = dict(outcome="stopped", result="stopped")
            break
        try:
            line = lines.get(timeout=min(0.25, left))
        except queue.Empty:
            continue
        if line is None:
            out = dict(outcome="failed", result="the skill's process ended: " + "".join(err_tail[-5:])[-400:])
            break
        try:
            m = json.loads(line)
        except ValueError:
            continue
        if "call" in m:
            name, a = m["call"], m.get("args") or {}
            if name not in API:
                reply(dict(error=f"robot has no '{name}'"))
                continue
            try:
                reply(dict(ok=getattr(api, name)(**a)))
            except StopSkill as e:
                reply(dict(stop=str(e)))
            except RobotRefused as e:
                reply(dict(error=str(e)))
            except TypeError as e:
                reply(dict(error=f"robot.{name}: {e}"))
            except Exception as e:                     # noqa: BLE001  (the robot's error, given to the skill)
                reply(dict(error=f"robot.{name}: {type(e).__name__}: {e}"))
        elif "done" in m:
            out = dict(outcome="done", result=m["done"])
        elif "failed" in m:
            out = dict(outcome="failed", result=m["failed"])
        elif "stopped" in m:
            out = dict(outcome="stopped", result=m["stopped"])
    if child.poll() is None:
        child.kill()
    if out["outcome"] in ("timeout", "stopped") and api.task_id and hasattr(api, "robot"):
        try:
            api.robot.cancel(api.task_id)                # the robot stops what the skill started
        except Exception:                              # noqa: BLE001
            pass
    out.update(said=list(api.lines), s=round(time.monotonic() - t0, 2))
    return out


# -- the library ----------------------------------------------------------------------------------------------------
@dataclass
class Skill:
    name: str
    code: str
    meta: dict
    path: Path
    builtin: bool
    info: dict = field(default_factory=dict)      # invented: request, created, tried, kept

    @property
    def description(self) -> str:
        return self.meta.get("description", "")

    @property
    def moves(self) -> bool:
        return bool(self.meta.get("moves", True))

    @property
    def max_s(self) -> float:
        return float(self.meta.get("max_s", 180.0))

    @property
    def needs_ok(self) -> bool:
        """The brain asks first: an invented skill nobody has run yet."""
        return not self.builtin and not self.info.get("tried")

    def schema(self) -> dict:
        props = dict(self.meta.get("params") or {})
        return dict(type="object", properties=props, required=list(self.meta.get("required") or []))

    def public(self) -> dict:
        return dict(name=self.name, description=self.description, says=self.meta.get("says", []),
                    params=self.meta.get("params", {}), moves=self.moves, builtin=self.builtin,
                    path=str(self.path), **{k: self.info.get(k) for k in ("request", "created", "tried", "kept")})


def sample_args(meta: dict) -> dict:
    """Arguments for a dry run in FakeAPI's house: a value for each required parameter."""
    out = {}
    for k in meta.get("required") or []:
        p = (meta.get("params") or {}).get(k) or {}
        if p.get("enum"):
            out[k] = p["enum"][0]
        else:
            out[k] = {"number": 1.0, "integer": 1, "boolean": True, "array": [], "object": {}}.get(p.get("type"),
                                                                                                 "kitchen")
    return out


def validate(code: str) -> tuple:
    """(meta, problems): the skill's SKILL and docstring, and what is wrong with it."""
    bad = problems(code)
    try:
        meta = metadata(code)
    except (ValueError, SyntaxError) as e:
        return {}, bad + [f"SKILL: {e}"]
    if not NAME.match(str(meta.get("name", ""))):
        bad.append("SKILL name: lower case letters, digits and _ (3-41 characters)")
    if not meta.get("description"):
        bad.append("no docstring saying what it does")
    if not isinstance(meta.get("says", []), list):
        bad.append("SKILL says: a list of phrases")
    if not isinstance(meta.get("params", {}), dict):
        bad.append("SKILL params: a dict of JSON-schema properties")
    return meta, bad


class Library:
    """The hand-written skills (skills/) and the invented ones (USER_DIR); an invented name never shadows a
    hand-written one."""

    def __init__(self, builtin_dir: Path = BUILTIN_DIR, user_dir: Path = USER_DIR):
        self.builtin_dir, self.user_dir = Path(builtin_dir), Path(user_dir)
        self.lock = threading.Lock()

    def _load(self, path: Path, builtin: bool):
        try:
            code = path.read_text(encoding="utf-8")
            meta, bad = validate(code)
        except (OSError, UnicodeDecodeError):
            return None
        if bad or meta.get("name") != path.stem:
            return None
        info = {}
        if not builtin:
            try:
                info = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
            except (OSError, ValueError):
                info = {}
        return Skill(meta["name"], code, meta, path, builtin, info)

    def all(self) -> dict:
        out = {}
        for d, builtin in ((self.user_dir, False), (self.builtin_dir, True)):
            for p in sorted(d.glob("*.py")) if d.is_dir() else ():
                s = self._load(p, builtin)
                if s is not None:
                    out[s.name] = s                     # hand-written last: they win a clash
        return out

    def get(self, name: str):
        return self.all().get(name)

    def _info(self, name: str, **kw):
        p = self.user_dir / f"{name}.json"
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            d = {}
        d.update(kw)
        p.write_text(json.dumps(d, indent=1), encoding="utf-8")
        return d

    def save(self, code: str, request: str) -> Skill:
        meta, bad = validate(code)
        if bad:
            raise ValueError("; ".join(bad))
        with self.lock:
            name = meta["name"]
            if (self.builtin_dir / f"{name}.py").exists():
                raise ValueError(f"{name} is a hand-written skill's name")
            self.user_dir.mkdir(parents=True, exist_ok=True)
            p = self.user_dir / f"{name}.py"
            p.write_text(code, encoding="utf-8")
            (self.user_dir / f"{name}.json").write_text(json.dumps(dict(
                request=request, created=time.strftime("%Y-%m-%d %H:%M"), tried=False, kept=False), indent=1),
                encoding="utf-8")
        return self.get(name)

    def mark_tried(self, name: str):
        if (self.user_dir / f"{name}.py").exists():
            with self.lock:
                self._info(name, tried=True)

    def keep(self, name: str):
        if not (self.user_dir / f"{name}.py").exists():
            raise KeyError(f"no invented skill {name}")
        with self.lock:
            self._info(name, kept=True)

    def delete(self, name: str):
        p = self.user_dir / f"{name}.py"
        if not p.exists():
            raise KeyError(f"no invented skill {name}")
        with self.lock:
            p.unlink()
            p.with_suffix(".json").unlink(missing_ok=True)

    def rename(self, name: str, new: str):
        if not NAME.match(new):
            raise ValueError("a name: lower case letters, digits and _ (3-41 characters)")
        p = self.user_dir / f"{name}.py"
        if not p.exists():
            raise KeyError(f"no invented skill {name}")
        if new in self.all():
            raise ValueError(f"{new} is taken")
        with self.lock:
            code = p.read_text(encoding="utf-8")
            code2 = re.sub(r"(name\s*=\s*)([\"'])" + re.escape(name) + r"\2", lambda m: m.group(1) + m.group(2) + new
                           + m.group(2), code, count=1)
            code2 = re.sub(r"([\"']name[\"']\s*:\s*)([\"'])" + re.escape(name) + r"\2",
                           lambda m: m.group(1) + m.group(2) + new + m.group(2), code2, count=1)
            if metadata(code2).get("name") != new:
                raise ValueError("couldn't rename it inside the file")
            (self.user_dir / f"{new}.py").write_text(code2, encoding="utf-8")
            j = p.with_suffix(".json")
            if j.exists():
                j.replace(self.user_dir / f"{new}.json")
            p.unlink()
