# omni_mcp: the robot as tools for Claude Code and Codex

The plan and its rules are in [ai_robot_architecture.md](../../../ai_robot_architecture.md). This is a stdio MCP server on
the Windows desktop. It holds one websocket to the robot's rosbridge, sends the console's `/console/command` verbs, and
takes camera snapshots from web_video_server. The model chooses tasks. The robot runs them under its own limits, and
stops by itself when this server goes away (the lease). This side needs no ROS.

**Status (Nov 5, 2026):** phases 1–4 are implemented and smoke-tested in the sim only, on both arm cameras. Hardware
(phase 5) and voice (phase 6) haven't been started. Nothing here has been physically validated. A model (Claude, in a
Claude Code session without the MCP tools registered) has driven it through `call.py` and an attached script; the MCP
transport itself is exercised by `ai_smoke.py`'s child-process checks. For what was run and what passed, see "Checks"
below.

## Setup

```powershell
C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe -m venv C:\Users\ferna\.venvs\omni-mcp
C:\Users\ferna\.venvs\omni-mcp\Scripts\python.exe -m pip install mcp websockets pillow pytest numpy
```

Use this venv only. Don't install into PlatformIO's Python, and leave the sim venv unchanged. The server itself needs
`mcp` 2.x (where `FastMCP` was renamed `MCPServer`), `websockets` and `pillow`. The tests also need `numpy`, because
their end-to-end fake runs the console's real core.

**Register it with Claude Code.** Put this `.mcp.json` at the repo root (not checked in yet: add it when you want the
tools in every session opened here). Then `/mcp` in a Claude Code session in this repo lists `omni`.

```json
{
  "mcpServers": {
    "omni": {
      "command": "C:\\Users\\ferna\\.venvs\\omni-mcp\\Scripts\\python.exe",
      "args": ["ros/tools/omni_mcp/server.py"],
      "env": { "OMNI_HOST": "127.0.0.1", "OMNI_ROSBRIDGE_PORT": "9090", "OMNI_VIDEO_PORT": "8081" }
    }
  }
}
```

**Codex:** the same server in `~/.codex/config.toml`:

```toml
[mcp_servers.omni]
command = 'C:\Users\ferna\.venvs\omni-mcp\Scripts\python.exe'
args = ['C:\Dropbox\Personal Projects\Robotics\omni-robot\ros\tools\omni_mcp\server.py']
env = { OMNI_HOST = "127.0.0.1", OMNI_ROSBRIDGE_PORT = "9090", OMNI_VIDEO_PORT = "8081" }
```

**Browser or phone:** continue the desktop session through Claude Code's Remote Control. The server keeps running on
the desktop, so nothing is exposed publicly. VERIFY the current Remote Control command in the Claude Code docs; it isn't
exercised here.

**Environment:**

- `OMNI_HOST` (127.0.0.1: on this PC "localhost" resolves to ::1 first, and that path to WSL2 hangs)
- `OMNI_ROSBRIDGE_PORT` (9090)
- `OMNI_VIDEO_PORT` (8081)
- `OMNI_SNAP_PORT` (8086: the Pi's camera_snap)
- `OMNI_AI_SESSION` (random when unset)

**Beside Fernando's stack:** start the test stack with another `ROS_DOMAIN_ID`, `mcu_port`, and `OMNI_ROSBRIDGE_PORT` /
`OMNI_VIDEO_PORT` in its environment (`omni_bringup/stack.py` reads both). Then point this server's variables at those
ports.

- **Avoid 8181:** the 3Dconnexion driver (`3dxnlserver`) holds 8181 on Windows, and WSL's mirrored networking shares
  ports with Windows. The Nov 4 runs used domain 57 with rosbridge on 9190 and video on 8191.
- **Shared task book:** the console keeps its task book over restarts (`ai_tasks.json` in its state dir). Explicit
  `request_id`s must therefore be new per run; the automatic ones carry a random part per process.

`python server.py --check` connects, prints the robot's status and exits, without MCP and without a lease.

## Tools

| Tool | What it does |
| --- | --- |
| `robot_status`, `robot_events`, `robot_capabilities`, `robot_places` | Read-only. Status says `source` (sim or robot), the owner (`ai`, `page`, `rc` or none) and whether the lease is held |
| `robot_look(camera)` | `front` (the depth camera's colour), `wrist`, `d435_color` or `arducam`, ≤ 640 px JPEG |
| `robot_move(steps, speed, turn_rate)` | Program blocks (`spin`, `turn`, `forward`…) run on odometry. `"max"` = `ai.max_speed` / `ai.max_turn` (0.5 m/s, 180°/s by default), then the firmware's profile. Driving needs Nav2 (its collision monitor); spins in place don't |
| `robot_goto(place \| x, y, yaw_deg)` | Nav2 to a saved place or a map pose |
| `robot_find(color, shape)` | omni_arm's floor candidates labelled by colour and shape (`color_vision.py`), with ids, scores, image-only blobs, a hint, and the annotated image (`/arm/find_image`) |
| `robot_pick(object_id)` | The tidy for that one object (`TidyTask` `only`): approach, close look, wrist refine, grasp, bin |
| `robot_task(task_id, wait_s)` | State and evidence. Picks get `sim_truth` in the sim (`/sim/objects`, scoring only) |
| `robot_cancel`, `robot_stop` | Need no lease. Cancel stops the task; the arm stows on checked paths and never goes limp |
| `robot_say`, `robot_arm_pose(name)` | A line on the face; a named pose (`stow`, `ready`, `high`, `drop`) |
| `robot_map(crop, marks)` | The SLAM map as a picture with a labelled metre grid (north up), rooms, places, AprilTags, the robot, and red marks to check goals against. The reply gives the pixel-to-map formula |
| `robot_scan`, `robot_people` | The lidar now: the nearest return per 10° sector and a plan view; people by the Follow mode's leg and body detector, in the robot's frame and on the map |
| `robot_tags`, `robot_tf` | AprilTag poses as `tag_slam` estimates them (with the direction each faces); any frame's planar pose in another |
| `robot_floor_unknowns`, `robot_floor_label(thing_id, label, note)` | The things on the floor the D435's height map found (arm step, `depth_layer:=height`; design.md 4.5): ids, places, sizes, heights, what the map made of them, and a picture of the newest unnamed ones' crops. The model names each (`floor`, `rug`, `step`, `low`, `toy`, `obstacle`, `drop`); the label goes into the robot's map and its catalog (`$OMNI_STATE_DIR/floor_catalog.json`). A label that relaxes never clears what the camera measures as an obstacle or a drop. The only tool that publishes a topic (`/floor_obstacles/label`) |
| `robot_topics`, `robot_topic(name)` | Generic read-only access: every topic and node; one message of any topic as JSON |
| `robot_session` | The session and a recipe for scripts (below) |

**The intelligence is in the model.** The tools are primitives and senses. A request like "drive a hexagon as large as
the room allows" or "find the person in the office and say hello" is worked out live: read the map and rooms, compute
goals, check them on the map, drive, look, decide again.

**Scripts written mid-session:** `Robot.attach()` (in `server.py`) joins the running server's session, so a script's
motion verbs ride on that server's lease, limits and take-over rules. When the server goes, so does the lease.
`map_grid()` gives the occupancy cells for geometry. Two more ways in:

- `python server.py --hold` holds a lease and publishes the session without MCP.
- `python call.py <method> '{json}' [--img file]` makes one call from the shell (an AI session without the tools, or
  a shell script).

**Rooms:** named outlines in each map's `<map>.places.json` (`rooms`), read by `robot_places` and drawn by `robot_map`.
For now they are placeholders for the sim house (`house`, `house_furnished`); the real house gets real labels when it
is mapped. The console's `room_save` / `room_delete` verbs set them (not an AI tool).

The tools deliberately leave out raw velocity, jog, torque off, the gripper alone, fire, thrower, calibration, settings,
maps, firmware and the shell.

**Replies:** every reply is `{"ok", "message", "data"}`. A refusal carries `data.error`, one of:

- `no_lease`
- `busy`
- `owner_conflict`
- `arm_out`
- `not_localized`
- `unknown_place`
- `not_found`
- `lost`
- `out_of_limits`
- `fault`
- `stale_object`
- `timeout`
- `no_arm`
- `bad_request`
- `unknown_task`
- `no_robot`
- `no_camera`

## Safety rules (enforced on the robot, in omni_console/missions.py)

- **The lease.** `lease.py` renews a 3 s lease every second. When it lapses, the console cancels the AI's task and holds
  the robot still (MANUAL, a zero-twist tail). The firmware's 0.5 s link watchdog stays underneath that.
- **A person always wins.** Any person's motion verb (the page, the voice) or the RC's manual switch cancels the AI's
  task. The AI then may not start motion for 10 s. Neither may it while the stick drives or a console mission runs.
- **One motion task at a time.** A second request is refused (`busy`), not queued. A retry with the same `request_id`
  returns the same task.
- **Every task has a deadline.** When it passes, the task ends `TIMED_OUT` and the robot stops. After a console restart,
  running tasks become `INTERRUPTED` and nothing resumes by itself.
- **Image text is data.** Words seen in images, labels or logs are never permissions (stated in the server's
  instructions and the tool descriptions).

## Checks

- **Unit tests:** run `& C:\Users\ferna\.venvs\omni-mcp\Scripts\python.exe -m pytest -q ros/tools/omni_mcp/test`. They
  use a fake rosbridge and cover status caching, reconnection, event paging, image downscaling, the error when rosbridge
  is down, lease renewal, the session and request ids, and the MCP tools end to end on the console's real core (a spin of
  5 turns measured as 1800°, a duplicate `request_id`, a stop). A lapsed lease stops the robot within the TTL.
- **The console's AI verbs:** `ros/src/omni_console/test/test_ai.py`.
- **Find and the single-object pick:** `ros/src/omni_arm/test/test_color_vision.py`.
- **In the sim, end to end:** `ros/tools/ai_smoke.py` against the arm step with `objects:=colours` (see its
  docstring). On Nov 4, against the stereo ARM at 0.67× real time, all 16 checks passed over two runs:

  | Check | Result |
  | --- | --- |
  | 5 spins at "max", through a real MCP child process | 1798.5° true turn (1800 ± 3 required); 10.7 s of sim time against an 11.3 s estimate; true peak 188°/s against the 180°/s ceiling |
  | MCP process killed mid-spin | Stopped in 2.8 s (TTL + 0.5 s allowed); the task `CANCELLED` |
  | Duplicate `request_id` | One task, one 359° turn |
  | Page takes control | A move is refused (`owner_conflict`); a running task is cancelled |
  | Goto | An unknown place is refused. A map pose and back: Nav2 succeeded, 6–9 mm from the goal, 8–9° heading error (Nav2's goal checker; the evidence reports it) |
  | "Go get the purple square object" | From a heading with the cube out of view: two 45° turns, find (score 0.99, sure), pick. Attempted, grasped and released; the sim's truth shows only `cube40_purple` went into the bin. "The purple round one" matched nothing |

  These are single runs.

  **Nov 5 robustness run (sim, house, `objects:=colours`, a separate domain and ports, 0.6–0.67× real time; single
  runs):**
  - **Stereo, Python arm:** 19/19 smoke checks, `restart` included.
  - **Nose D435, C++ arm (`arm_camera:=d435 arm_impl:=cpp`), the first end-to-end run of that pair:** spin, lease,
    duplicate id, take-over, gotos, blocked goal, lease lapse mid-goto and the purple pick passed (15/16). "Bring me
    the red cube" failed at first: the D435 at 7.5° sees the red living-room rug, and the seek chased it for 12 looks.
    After the fix below it found the cube in three looks, binned only `cube30` and came back within 4 mm; a fresh
    full run then passed 18/18.
  - **"Go to the kitchen and spin 3 times"** (D435, C++ arm), worked out as a model would: `goto("kitchen")` refused
    with the room's centre; the centre checked on `map_view` with a mark; goto (0.12 m from the goal by the truth,
    localization 0.08 m off); a 3-turn spin (1078.6° on odometry).
  - **"Visit every room, turn round once in each, take a picture"** (an attached script): 6/6 rooms reached, true
    distance to the room centre 0.03–0.23 m, localization 0.04–0.09 m off, each turn 358.5–358.8°, a picture each.

  **What the Nov 5 runs found and fixed:**
  - Floor-coloured blobs: `robot_find`'s `image_only` blobs now say `flat: true` when the depth shows them lying flat
    on the floor (under 18 mm: a rug, a mat), not something to drive to (`color_vision.FLAT_H`).
  - `goto` with a room's name now says it is a room and gives its centre (was "no place called 'kitchen'").
  - The move evidence's `max_turn_rate_deg_s` read 236°/s for a true 188°/s (rate between two console ticks, inflated
    by the pose's arrival jitter). It is now measured over at least 0.5 s. It is wall-clock: in the sim at 0.65× it reads
    below the true rate.
  - `map_view` marks given as `[x, y]` or `[x, y, label]` (as scripts write them) crashed it; both forms work now.
  - A console killed hard (crash, SIGKILL) left its `slam_toolbox` running, and the next console localized 0.9–1.9 m
    and up to 156° off. Fixed in omni_console (design.md 4.6).
  - Not fixed: Nav2 accepts goals 8–9° off the asked heading (its goal checker); a spin in place reports 0.1–0.16 m of
    odometry path (wobble, 4 mm true).

  **Live rehearsal of open requests (Nov 4, sim, furnished house `props:=true` on map `house_furnished`).** Done
  with the generic tools and one script written for the occasion, no mission code. Positions are checked against the
  sim's truth.

  | Request | What happened |
  | --- | --- |
  | "Drive to the bedroom" | Read the room outlines on `robot_map`, sent a goal through the bedroom door: 2 cm from the goal (truth) |
  | "Find the sign that says HELLO" | The floor camera sees only the floor, so the arm went to the `look` pose; the wrist camera showed HELLO (with tag 17 under it) |
  | "Drive a hexagon as large as the living room allows" | A script took `map_grid()` and the room outline, grew obstacles by 0.37 m and searched the largest hexagon (1.53 m side), marked it on the map, then drove it: 7/7 vertices, true error 3–37 mm |
  | "Go to the office, find the person, say hello there" | At the door, `robot_people` gave 7 candidates (chair and desk legs too). The one inside the office outline away from the desk was confirmed with the wrist camera; `robot_say` was accepted, but the test stack ran without the face (`face:=false`), and the events said nothing was spoken |
  | "Drive to AprilTag X" | Not done on Nov 4: with `tags:=true` the localization health read LOST and AI motion was refused. That was `omni_slam.graph`'s marginals, fixed later on Nov 4 (design.md 7); not rehearsed since |

  **What the rehearsal found:**
  - Furniture missing from the saved map put lidar localization 2.7 m off, and only looking at the camera exposed it.
    The furnished house got its own map.
  - The floor camera can't see walls, so the `look` pose was added. Its picture comes upside down and is turned
    upright.

  **What the runs found and fixed:**
  - The mission node crashed logging a finished AI task (`ai_task` is now a session-log record).
  - `web_video_server` needs the `/`s in topic names unencoded.
  - A find asked for frames before its settle time and so never got any (`Finder.due`).
  - Automatic request ids collided across runs.
