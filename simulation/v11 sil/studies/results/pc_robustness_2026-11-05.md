# PC robustness run, Nov 5, 2026 (sim only)

Whole-robot check of the arm robot on the desktop PC (WSL2 + Windows), no Pi: unit and parity tests, AI control
through omni_mcp, Nav2 routes and goals, navigate-then-tidy, console patrol, and a console crash. Every stack ran in an
isolated workspace (`~/omni_ws_claude`, ROS domain 57, rosbridge 9190, video 8191, MCU 5771, servo 5773, its own
state dir) with `viewer:=false foxglove:=false web:=false face:=false`, map `house`, sim clock, `fast` on. The sim ran
at 0.56–0.69× real time. Single runs per configuration; robot truth from `/ground_truth/odom` and `/sim/objects`.
Configurations are those of `robot_export.ARM` (stereo) and `ARM_D435` (`arm_camera:=d435`) as of this date.

## Tests

| Suite | Result |
| --- | --- |
| v11 `test_v11.py` (Windows) | 132 passed |
| ROS packages (WSL, built workspace) | omni_arm 41, omni_bringup 32, omni_calib 8, omni_console 105, omni_cpp 25 (Python/C++ parity), omni_face 18 (+4 skipped), omni_slam 7: all passed |
| omni_mcp (its venv) | 12 passed |

## AI control (`ros/tools/ai_smoke.py`, `objects:=colours`)

| Configuration | Result |
| --- | --- |
| Stereo, Python arm (`arm_node`), restart check included | 19/19. 5-spin 1798.6° true; lease kill stopped in 2.9 s (spin) and 3.4 s (goto); gotos 5–9 mm from the goal; purple and red picks binned only the asked object; console restart: task INTERRUPTED, robot stopped in 2.8 s |
| D435, C++ arm (`arm_impl:=cpp`), first end-to-end run | 15/16: "bring me the red cube" failed. The D435 at 7.5° sees the red living-room rug; `robot_find` reported it as image-only blobs 4–13 mm high and the seek drove at it for 12 looks |
| Same, after marking flat blobs (`color_vision.FLAT_H`) | 18/18 (fresh stack). Red cube found in 3 looks |

Requests a model worked out live through `call.py` / an attached script (D435, C++ arm):

| Request | Result |
| --- | --- |
| "Go to the kitchen and spin 3 times" | `goto("kitchen")` refused with the room's centre (5.97, −3.25); centre checked on `map_view`; goto succeeded, 0.12 m from the goal by the truth, localization 0.08 m off; spin 1078.6° (odometry), 4 mm true drift |
| "Visit every room, turn once in each, take a picture" | 6/6 rooms; true distance to the centre 0.085 / 0.058 / 0.031 / 0.046 / 0.043 / 0.229 m (office, bedroom, hallway, living room, kitchen, bathroom); localization 0.04–0.09 m; turns 358.5–358.8°; bedroom → hallway took 68 s against ~25 s for the others |

## Routes, navigate-then-tidy, patrol (`ros/tools/arm_load.py`, `objects:=toys`, six toys, seed 0)

**Stereo, Python arm, phases ROUTE (2 laps) → NAVTIDY (2 legs) → PATROL (120 s):**

- ROUTE: 16/16 Go Tos over the eight house spots. True arrival error 0.04–0.13 m. Localization 0.012–0.13 m. Legs
  9–28 s. Top speed 1.30 m/s.
- NAVTIDY leg 1: the tidy reported 6 picked, 0 failed in 455 s, but the bin truth went 0 → 5.
- NAVTIDY leg 2: drove to the remaining toy, `ball40`. Three grasps on it missed (gripper closed to −10°); each nudged
  it, so the close look found it gone; "nothing in sight" after 461 s.
- PATROL: ran.

Cause, from a second fresh run with every object's true position logged: the route laps had already pushed `ball40`
from (−1.28, 0.62) to (−3.02, −1.17) (world). The stereo robot's Nav2 has no depth layer and the lidar plane is at
163 mm, so it drives through floor toys.

**Stereo, Python arm, NAVTIDY first (fresh stack):** 6/6 into the bin, 0 failed, in 334 s (the Nov 3 regression's
result).

**D435, C++ arm, ROUTE (2 laps) → NAVTIDY (2 legs):**

- ROUTE: 16/16. True arrival error 0.04–0.14 m. Localization 0.025–0.135 m. Legs 10–24 s. Top speed 1.24 m/s.
- NAVTIDY, after the same two laps: 6/6 into the bin, 0 failed, in 327 s. Leg 2 wasn't needed.
- The D435's Nav2 depth layer sees the toys, and this time the laps left them pickable. One run each, so this is
  consistent with the stereo explanation above, not a measured rate of pushed toys.

No node crashed in any run. Every "process has died" was the shutdown's SIGINT, or the deliberate SIGKILL of the
mission node.

## Console crash and restart (D435, C++ arm, `objects:=colours`)

| Case | Before the fix | After |
| --- | --- | --- |
| Mission node SIGKILLed and started again (as a respawn would) | its slam_toolbox kept running in its own session; the new one couldn't configure (`/slam_toolbox` taken); pose 0.9 m / 156° off. With the old one gone, relocalizing at the map's start: 1.9 m / 87° off | the leftover stopped first ("stopping a slam_toolbox an earlier console left running"); localization resumed from the last pose: 0.09 m off in the kitchen, as before the kill |
| A new stack started within 2 min of the last | (not checked) | first version resumed from the old session's record (harmless at the origin); now refused: the MCU's clock started again |
| A slam_toolbox left over from an earlier sim session | its map → odom, stamped in that session's future, made Nav2, the EKF and scan_prep refuse the new map → odom for the whole session (75,000 `TF_OLD_DATA` warnings; gotos still landed because the sim's odometry hardly drifts) | reaped at the console's construction; `omni.sh stop` and the supervisor already kill it; the test harness's stop now does too |

## Fixes made

- `omni_console/program.py`: the move's `max_turn_rate_deg_s` evidence read 236°/s for a true 188°/s (two-tick
  differences under pose jitter); now over ≥ 0.5 s. It is wall-clock: in the sim below real time it reads low.
- `omni_console/missions.py`: `goto` with a room's name says it's a room and gives its centre.
- `omni_console/mission_node.py`, `slam.py`: slam_toolbox's process group recorded and a leftover one stopped; the last
  localized pose resumed after a restart on the same MCU boot within 2 min.
- `omni_arm/color_vision.py`: image-only blobs flat on the floor are marked `flat`.
- `omni_mcp/views.py`: `map_view` marks as `[x, y]` lists no longer crash; `server.py`'s `robot_look` / `robot_find`
  descriptions updated (the D435's view, flat blobs).

## Open

- The stereo arm robot can't see floor toys while navigating and pushes them. The D435 option's depth layer is the
  existing answer; the stereo pair has none.
- `in_bin` is "released over the bin". One of six claimed picks didn't stay in the bin, most likely the ball. A bin
  sensor or a post-release look is needed before the evidence can say "in the bin".
- Balls on a displaced spot defeated three grasps in a row.
- Nav2 accepts goals 8–9° off the asked heading (goal checker).
- A spin in place reports 0.1–0.16 m of odometry path (wobble; 4 mm true).
- Composed Nav2 still needs SIGKILL at shutdown (each stop logged `component_container_isolated` escalating to
  SIGKILL), as on Nov 3.
