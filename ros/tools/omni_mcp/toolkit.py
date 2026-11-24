"""The robot's toolkit, written once (voice_brain_handoff_2026-11-23.md): every robot_* tool an AI client gets, as one
table. The MCP server (server.py: Claude Code, Codex) registers each Tool with MCP; the voice brain (omni_face
brain.py: Claude Haiku in the voice service) hands the same tools to the Claude API. Both see the same names,
descriptions and arguments.

A tool is a typed function: its signature is the argument schema (MCP builds it the same way; Tool.schema() makes the
JSON schema for the Claude API) and its docstring the description. It returns the client's reply dict {"ok", "message",
"data"} or (that dict, image bytes or None) when Tool.image says what the bytes are.

No `from __future__ import annotations` here, on purpose: MCP and pydantic read the annotations as real types.
"""

import inspect
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Literal

sys.path.insert(0, str(Path(__file__).resolve().parent))

FEELINGS = ("neutral", "content", "happy", "excited", "proud", "affection", "relieved", "curious", "surprised",
            "startled", "focused", "effort", "worried", "frustrated", "sad", "tired", "bored")


@dataclass
class Tool:
    name: str
    fn: Callable
    moves: bool = False            # it starts a motion task (the voice brain asks before a second one)
    image: str | None = None       # "jpeg" | "png": fn returns (reply, bytes or None)
    mcp: bool = True               # offered to MCP clients (Claude Code, Codex)
    voice: bool = True             # offered to the voice brain

    @property
    def description(self) -> str:
        return inspect.getdoc(self.fn) or ""

    def schema(self) -> dict:
        """The arguments as JSON schema (the Claude API's input_schema), from the signature."""
        import pydantic
        fields = {}
        for n, p in inspect.signature(self.fn).parameters.items():
            ann = Any if p.annotation is inspect.Parameter.empty else p.annotation
            fields[n] = (ann, ... if p.default is inspect.Parameter.empty else p.default)
        s = pydantic.create_model(f"{self.name}_args", **fields).model_json_schema()

        def clean(v):
            if isinstance(v, dict):
                return {k: clean(x) for k, x in v.items() if k != "title"}
            if isinstance(v, list):
                return [clean(x) for x in v]
            return v
        s = clean(s)
        s.setdefault("properties", {})
        return s

    def call(self, args: dict | None = None) -> tuple:
        """-> (reply dict, image bytes or None); a bad argument is a reply, not an exception."""
        try:
            r = self.fn(**(args or {}))
        except TypeError as e:
            return dict(ok=False, message=f"{self.name}: {e}", data=dict(error="bad_request")), None
        if self.image:
            return r
        return r, None


def make_tools(robot) -> list:
    """The toolkit on one robot_client.Robot."""
    T = []

    def tool(**kw):
        def deco(fn):
            T.append(Tool(fn.__name__, fn, **kw))
            return fn
        return deco

    @tool()
    def robot_status() -> dict:
        """The robot now: sim or real robot, mission and task, owner (ai | page | rc | none) and the AI's last task,
        pose (map or odom frame, m, deg) and localization, battery and MCU faults, the arm (out? tidying?), cameras,
        and whether this client holds the AI lease. Read-only; call it first."""
        return robot.status()

    @tool(voice=False)
    def robot_arm_effort() -> dict:
        """What the arm feels beyond its own weight and friction (design.md 4.5): per joint the outside torque (N m;
        0 while friction could still be holding it), strained or not, temperature, current; the outside torques as one
        force at the fingertip (N, the robot's frame: x ahead, y left, z up); the reflex (a guarded arm motion the driver
        stopped and backed off) and the load model it uses (nominal until the arm load calibration has run: then only
        large forces show). Read-only: check a grasp held, a push landed, a door or drawer resists."""
        return robot.arm_effort()

    @tool()
    def robot_events(since: int = 0) -> dict:
        """The console's events (mission changes, warnings, errors, the arm's steps) since event number `since` (0:
        all kept, up to 200). Each has n, level, text, source. Read-only: use it to explain why the robot stopped."""
        return robot.events(since)

    @tool(voice=False)
    def robot_capabilities() -> dict:
        """What the robot can do for you: the AI verbs, the arm's named poses, find's colours and shapes, the speed and
        turn limits (yours, ai.max_speed / ai.max_turn, then the firmware's profile), saved places, cameras, and what
        is deliberately not available. Read-only."""
        return robot.capabilities()

    @tool(image="jpeg")
    def robot_look(camera: Literal["front", "wrist", "d435_color", "arducam"] = "front", width: int = 640):
        """A picture from a camera: front (the floor camera's colour. On the stereo robot it looks ~35 deg down: the
        floor 0.3-2 m ahead and the feet of things, never walls above knee height. The nose D435 looks 7.5 deg down
        and sees the room ahead, wall tags included, but no floor nearer than ~0.35 m), wrist (on the arm's gripper: after
        robot_arm_pose("look") it looks straight ahead and 24 deg up from 0.57 m, for walls, signs, doors and faces;
        stow before driving), d435_color, arducam (the Pi's cameras, when present). width <= 640 px. Returns the image
        and the capture time and pose. Read-only."""
        return robot.look(camera, width)

    @tool()
    def robot_places() -> dict:
        """The saved places (x, y m, yaw deg) and the rooms (outline and centre) on the current map, map frame: for
        robot_goto. Room labels may be placeholders until the house is labelled. Read-only."""
        return robot.places()

    @tool(moves=True)
    def robot_goto(place: str | None = None, x: float | None = None, y: float | None = None, yaw_deg: float = 0.0,
                   timeout_s: float = 120.0, request_id: str | None = None) -> dict:
        """Drive to a saved place (robot_places) or to x, y, yaw_deg in the map frame, with Nav2 (needs navigation and
        a localized robot; the arm stowed). timeout_s <= 300. Returns a task_id once accepted: follow it with
        robot_task. Its evidence is Nav2's result and the pose on arrival with its distance to the goal."""
        return robot.goto(place, x, y, yaw_deg, timeout_s, request_id)

    @tool(moves=True)
    def robot_move(steps: list[dict], speed: float | Literal["max"] | None = None,
                   turn_rate: float | Literal["max"] | None = None, timeout_s: float | None = None,
                   request_id: str | None = None) -> dict:
        """Bounded relative moves, run by the robot's Program runner on its odometry. steps: blocks, e.g.
        [{"do": "spin", "turns": 5}], [{"do": "forward", "m": 0.5}, {"do": "turn", "deg": -90}]:
        forward / back / left / right {"m": 0.05-3}, turn {"deg": -360..360, + = left}, spin {"turns": 0.25-5},
        wiggle {}, wait {"s": 0.1-30}, repeat {"times": 1-20, "body": [...]}. speed m/s and turn_rate deg/s, or "max":
        capped by the AI's limits and the firmware's profile (the reply says what it used). Driving (not turning in
        place) needs navigation running (its collision monitor); the arm must be stowed. Returns a task_id with the
        estimated duration once accepted: follow it with robot_task; the evidence is the turn and distance the
        odometry measured."""
        return robot.move(steps, speed, turn_rate, timeout_s, request_id)

    @tool(image="jpeg")
    def robot_find(color: str | dict | None = None, shape: Literal["square", "round", "long", "any"] = "any",
                   max_range_m: float = 1.6):
        """Things on the floor in front of the robot by colour and shape: the depth camera's candidates, each with an
        id (stable across finds and for robot_pick), its position (odom frame), range and bearing, size (mm), colour
        (name, hue, saturation, value, share of its pixels in the asked colour), shape (class square | round | long |
        other, sure or a guess, its measures) and a score (colour x shape). color: red, orange, yellow, green, blue,
        purple, pink, white, grey, black, or {"h": [lo, hi] (OpenCV 0-179), "s_min", "v_min"} when the light is odd.
        Also image_only blobs (the colour seen with no pickable candidate: turn or drive toward its bearing; one
        with flat: true lies flat on the floor, a rug, mat or marking, not an object to seek) and a
        hint when a shape is only a guess from too far (move closer, find again). Returns the annotated image: asked
        colour hatched cyan, candidates boxed (green the best match) with id and score, blobs red "?". Read-only: the
        robot doesn't move."""
        return robot.find(color, shape, max_range_m)

    @tool(moves=True)
    def robot_pick(object_id: int, timeout_s: float = 180.0, request_id: str | None = None) -> dict:
        """Pick up one object of the last robot_find (its id; the find must be under 30 s old) and drop it in the bin:
        the robot approaches it, looks closer, refines with the wrist camera, grasps, lifts and drops. Needs the arm
        stowed. Returns a task_id once accepted: follow it with robot_task (a pick takes ~30-90 s). The evidence:
        attempted, grasped, released (over the bin); in_bin is uncertain on the robot (no bin sensor yet), and in the
        sim robot_task adds sim_truth (which things really went in)."""
        return robot.pick(object_id, timeout_s, request_id)

    @tool(moves=True)
    def robot_fetch(color: str | None = None, shape: Literal["square", "round", "long", "any"] = "any",
                    to: Literal["me", "here"] = "me", timeout_s: float = 600.0, request_id: str | None = None) -> dict:
        """The robot's Fetch mission: it finds the thing by colour and shape (turning a quarter at a time and driving
        closer to a patch of its colour, four looks at most), picks it up and keeps it in the gripper, drives to the
        nearest person it sees (to "me"; it looks round if nobody is in view) or back to where it started ("here"),
        and puts it down in front. Needs the arm stowed and, to reach a person, navigation. Returns a task_id: follow
        it with robot_task (a fetch takes 1-5 min); the evidence is the robot's account of how it went."""
        return robot.mission("fetch", dict(color=color, shape=shape, to=to), timeout_s, request_id)

    @tool(moves=True)
    def robot_message(text: str, rooms: list[str] | None = None, timeout_s: float = 900.0,
                      request_id: str | None = None) -> dict:
        """The robot's Messenger mission: it looks round where it is, then goes room by room (the map's rooms,
        nearest first, or only those named) and looks round each until it sees a person, goes up to them and says
        the text aloud. Nobody in any room: the task fails saying so. Needs navigation and a localized robot.
        Returns a task_id: follow it with robot_task."""
        return robot.mission("message", dict(text=text, rooms=rooms), timeout_s, request_id)

    @tool(moves=True)
    def robot_tower(max_blocks: int = 4, timeout_s: float = 900.0, request_id: str | None = None) -> dict:
        """The robot's Block tower mission: the first block (a 3-5 cm cube) it finds on the floor is the base, it
        stacks the others on it, checking the tower with its depth camera before each, up to max_blocks (2-5).
        In the sim a block gripped off square can still topple. Returns a task_id; the evidence has the height."""
        return robot.mission("tower", dict(max_blocks=max_blocks), timeout_s, request_id)

    @tool(moves=True)
    def robot_dance(routine: Literal["groove", "party", "robot"] | None = None, bpm: float | None = None,
                    timeout_s: float = 600.0, request_id: str | None = None) -> dict:
        """The robot's Dance party: slides, circles, spins and shimmies on the beat (60-160 bpm) and sings, for the
        length set on the console (dance.seconds), never more than 0.6 m from where it started. Returns a task_id."""
        return robot.mission("dance", dict(routine=routine, bpm=bpm), timeout_s, request_id)

    @tool(moves=True)
    def robot_goalie(seconds: float = 60.0, timeout_s: float = 600.0, request_id: str | None = None) -> dict:
        """The robot's Goalie: it keeps a goal where it stands and slides sideways to block a ball rolled at it
        (the D435 tracks a red play ball by default), for `seconds` (0: until stopped). Returns a task_id; the
        evidence is the score (saves, goals, shots)."""
        return robot.mission("goalie", dict(seconds=seconds), timeout_s, request_id)

    @tool(voice=False)
    def robot_task(task_id: str, wait_s: float = 10.0) -> dict:
        """A task's state and evidence. Waits up to wait_s (<= 30) and returns early when the state changes. States:
        ACCEPTED, RUNNING, then SUCCEEDED | FAILED | CANCELLED | TIMED_OUT | INTERRUPTED (the robot's console restarted:
        nothing resumes by itself). Call it again until the state is final."""
        return robot.task(task_id, wait_s)

    @tool(mcp=False)
    def robot_wait(task_id: str, max_s: float = 300.0) -> dict:
        """Wait until a task ends (SUCCEEDED, FAILED, CANCELLED, TIMED_OUT or INTERRUPTED) or max_s (<= 900) passes,
        and return its record with the evidence. Use it after every task you start. CANCELLED usually means a person
        stopped the robot: then stop too."""
        return robot.wait_task(task_id, max_s)

    @tool()
    def robot_cancel(task_id: str) -> dict:
        """Cancel a task: the robot stops what it was doing (the arm stows on its checked paths, never goes limp) and
        holds still. Needs no lease. Returns the task's final state."""
        return robot.cancel(task_id)

    @tool()
    def robot_stop() -> dict:
        """Stop the robot now: motors off (ramped), any task cancelled. Always allowed, needs no lease. Use it whenever
        unsure."""
        return robot.stop()

    @tool()
    def robot_say(text: str, expression: Literal["auto", "happy", "excited", "sad", "worried", "sleepy", "love",
                                                 "silly", "wink", "thinking"] = "auto") -> dict:
        """The robot says a line (up to 200 characters) on its face and speaker, with an expression."""
        return robot.say(text, expression)

    @tool()
    def robot_express(feeling: Literal[FEELINGS], intensity: float = 0.6, text: str = "") -> dict:
        """The robot feels something now and shows it, as much as its Emotion levels allow: its face, a chirp or
        vocalization, and a wheel or arm gesture only if that part is free (never during a task). intensity 0-1.
        Optional text is said with that feeling (up to 200 characters)."""
        return robot.express(feeling, intensity, text)

    @tool(moves=True)
    def robot_arm_pose(name: Literal["stow", "ready", "high", "drop", "look"]) -> dict:
        """Move the arm to a named pose on its checked paths (stow before driving or spinning). Returns a task_id:
        follow it with robot_task."""
        return robot.arm_pose(name)

    @tool(image="png", voice=False)
    def robot_map(x0: float | None = None, y0: float | None = None, x1: float | None = None, y1: float | None = None,
                  max_px: int = 900, marks: list[dict] | None = None):
        """The occupancy map (SLAM's) as a picture, north (+y) up, a metre grid labelled in map metres, with the rooms'
        outlines and names, saved places, AprilTags and the robot (green: circle and heading). Optionally a crop
        (x0, y0, x1, y1 in map metres) and marks [{"x", "y", "label"}] (red x's: check goals before sending them). The
        reply gives the pixel-to-map formula, the rooms' polygons, places, tags and the robot's pose. Read-only. For
        geometry (largest shape that fits, clearances), write a script: Robot.attach().map_grid() returns the cells."""
        crop = (x0, y0, x1, y1) if None not in (x0, y0, x1, y1) else None
        return robot.map_view(crop, max(200, min(1600, int(max_px))), marks)

    @tool(image="png", voice=False)
    def robot_scan(topic: str = "/scan_filtered"):
        """The lidar now, around the robot: the nearest return in each 10 deg sector (base frame: bearing + left, range
        from the robot's centre) and a plan-view picture (forward up, rings every metre). Read-only: clearances before
        a move, what is near, which way is open."""
        return robot.scan(topic)

    @tool(image="png")
    def robot_people():
        """People around the robot, in its frame (range, bearing) and on the map, with the scan's picture (orange
        circles). With the forward camera (the people node running) each has an id kept while they move and a status:
        "person" (the camera saw a person there) or "candidate" (lidar legs only, e.g. outside the camera's view: turn
        towards them to check). Without it, lidar shapes only (legs and bodies; chair legs too): confirm with
        robot_look facing them. Not identities. Read-only."""
        return robot.people()

    @tool(voice=False)
    def robot_tags() -> dict:
        """The AprilTags on the map (tag_slam's estimates: id, x, y, z in the map frame, and faces_deg: the direction the
        tag's face looks, into the room). To stand in front of tag N facing it, aim for a point ~1 m along faces_deg
        from it with yaw = faces_deg + 180. Read-only."""
        return robot.tags()

    @tool(image="jpeg", voice=False)
    def robot_floor_unknowns():
        """Things on the floor the depth camera found but nobody has named yet (the robot's height map, design.md 4.5):
        each with an id, its place (map frame), size and height, what the map made of it (low: under 3 cm, slowed
        for; obstacle: 3 cm and up, driven round), and a picture of the newest ones' crops (3 x 2 tiles, row by row,
        the ids in data.tiles). Look at them and name each with robot_floor_label: a rug's or mat's edge, a door sill,
        a cable, a toy, a shoe, a box... This builds the robot's catalog of what is in the house. Read-only."""
        return robot.floor_unknowns()

    @tool(voice=False)
    def robot_floor_label(thing_id: int, label: Literal["floor", "rug", "step", "low", "toy", "obstacle", "drop"],
                          note: str = "") -> dict:
        """Name one thing of robot_floor_unknowns: floor (nothing there), rug or step (a rug, mat, sill or raised
        floor: drivable), low (something low to slow for), toy (to tidy), obstacle (to drive round), drop (a step
        down: never drive there). note: what it is, in a few words. A label that relaxes (floor, rug, step) never
        clears what the camera measures as an obstacle or a drop; obstacle and drop always apply."""
        return robot.floor_label(thing_id, label, note)

    @tool(voice=False)
    def robot_tf(child: str, parent: str = "map") -> dict:
        """Where a frame is in another (x, y m, yaw deg; planar), e.g. child base_link, laser, d435_depth_optical_frame,
        arm_tool in map, odom or base_link. Read-only."""
        return robot.tf(child, parent)

    @tool(voice=False)
    def robot_topics() -> dict:
        """Every ROS topic (with its type) and node on the robot. Read-only."""
        return robot.topics()

    @tool(voice=False)
    def robot_topic(name: str) -> dict:
        """One message of any topic as JSON (long arrays shortened; images and clouds refused: use robot_look /
        robot_scan), e.g. /localization/health, /mcu/state_10hz, /arm/state, /odom. Read-only."""
        return robot.topic(name)

    @tool(voice=False)
    def robot_session() -> dict:
        """For a script you write mid-session (Python, the omni-mcp venv): it attaches to this server's session, so its
        motion verbs ride on this server's lease and limits. Read-only."""
        return dict(ok=True, message="", data=dict(
            session=robot.session, file=str(robot.SESSION_FILE),
            python=r"C:\Users\ferna\.venvs\omni-mcp\Scripts\python.exe",
            usage=("import sys; sys.path.insert(0, r'<repo>/ros/tools/omni_mcp'); from server import Robot; "
                   "r = Robot.attach(); g = r.map_grid(); t = r.goto(x=1.0, y=2.0, yaw_deg=90); "
                   "r.task(t['data']['task_id'], 30); r.close()"),
            methods=["status", "map_grid", "map_view", "scan", "people", "tags", "tf", "topic", "places", "look",
                     "goto", "move", "find", "pick", "task", "cancel", "stop", "say", "arm_pose"]))

    return T


def as_text(r) -> str:
    return json.dumps(r, default=str)
