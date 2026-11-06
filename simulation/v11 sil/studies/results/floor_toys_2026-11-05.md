# Floor toys in Nav2's costmap, Nov 5, 2026 (sim only)

The arm robot (`ARM_D435`, `arm_camera:=d435`: the D435 is its only camera since Nov 5) drove the house route with toys
on the floor. Does Nav2 see them, drive over or push them, and leave stale marks once the tidy has picked them? The
Nov 5 robustness run had one stereo run push `ball40` 2.6 m (pc_robustness_2026-11-05.md); the stereo pair is out
of the arm robot now, so only the D435 was studied.

**Conditions.** PC only (WSL2 + Windows), isolated stacks in `~/omni_ws_claude` (ROS domains 57–60, their own ports and
state; up to four at once), `viewer:=false foxglove:=false web:=false face:=false`, map `house` (no props), sim clock,
`fast` on, Python arm (`arm_impl:=py`), full Nav2 profile. Real-time factor not recorded per run (~0.5–0.7× in the
Nov 5 runs; lower here with several stacks at once). Robot and toy truth from `/ground_truth/odom` and `/sim/objects`.
Three seeds per configuration, one run each.

**Harness:** `ros/tools/toy_route.py` (new). It puts the six toys of `objects:=toys` (seeded) on the route through the
bridge's control port (`sim_world.World.place_toys`, new): four on the path the robot drove (within 5 cm of it) and
two 0.30–0.40 m beside it, each ≥ 0.8 m from a goal spot, ≥ 0.7 m from the hallway's doorways, ≥ 1 m apart, with room
round them (`path.json`: one lap with the toys away, 45.5 m). Then two laps of the eight house spots (Go To), then
the tidy toy by toy (Go To 0.8 m short of the nearest toy still on the floor, the console's tidy; a toy given up after
two legs). Scores:

- **Overrun:** the robot's centre came within 0.225 m (the bumper's radius) of a toy. In the sim this is the only way
  a toy and the robot meet: the lowest part that can touch a toy is the bumper ring 50 mm up, and the wheels' rollers
  touch only the floor, so every toy (12–40 mm) passes under the robot. On the robot a wheel or the bumper meets it.
- **Moved:** the toy's truth moved > 3 cm during the route. No toy moved in any run (see above).
- **Marked:** the global costmap's cost at the toy after the route (99 inscribed, 100 lethal).
- **In bin:** the bin's truth after the tidy. **Stale:** cost ≥ 99 left where a binned toy lay.
- **False marks** (final runs only): floor-grid cells marked > 0.3 m from every toy and from the map's walls, the
  largest count seen at any goal (the house has no furniture without props).

## The layers

| Layer | What it is |
| --- | --- |
| `raw` (Nov 4–5's, the default) | The D435's point cloud in a Nav2 ObstacleLayer: marks above 4 cm out to 1.2 m, clears along every ray to the floor. Every toy here is ≤ 40 mm: it sees none. |
| `floor` (new, proposed) | `omni_arm/floor_obstacles_node.py`: each frame against the floor it sees (`floor_vision`: plane fit plus rug levels), so pitch drops out; things 8 mm and up (`floor_vision`'s objects) and anything over 25 mm, kept on the map's cells until the camera plainly sees floor there; `/floor_obstacles/grid` into a StaticLayer in both costmaps. |

`floor` versions run: **v1** (things of any length from 8 mm), **v2** (+ a long low thing is floor; marks under the
robot cleared), **final** (+ things marked after 2 sightings with no clear floor between, and only within 1.5 m of
the camera). The study's diagnoses drove each step (below).

## Results

| Run | Goals | Route s | Loc. err max m | On-route overrun | Beside overrun | Closest on-route m | Moved > 3 cm | Marked after route | In bin | Tidy legs | Tidy s | Stale marks | False marks (max) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| raw_s0 | 16/16 | 399 | 0.124 | 3/4 | 0/2 | 0.01 | 0 | 0/6 | 6/6 | 5 | 720 | 0/6 | - |
| raw_s1 | 16/16 | 335 | 0.143 | 2/4 | 0/2 | 0.01 | 0 | 0/6 | 5/6 | 6 | 908 | 0/6 | - |
| raw_s2 | 16/16 | 416 | 0.123 | 2/4 | 1/2 | 0.07 | 0 | 0/6 | 5/6 | 5 | 949 | 0/5 | - |
| v1_floor_s0 | 16/16 | 403 | 0.135 | 0/4 | 0/2 | 0.34 | 0 | 5/6 | 5/6 | 6 | 995 | 1/5 | - |
| v1_floor_s1 | 9/16 + every tidy drive | - | - | - | - | - | - | - | - | - | - | - | stopped: boxed in at the bedroom door |
| v2_floor_s0 | 16/16 | 372 | 0.134 | 0/4 | 0/2 | 0.31 | 0 | 6/6 | 5/6 | 6 | 929 | 0/5 | - |
| v2_floor_s1 | 14/16 | 446 | 0.164 | 1/4 | 0/2 | 0.10 | 0 | 5/6 | 6/6 | 6 | 998 | 0/6 | - |
| v2_floor_s2 | 15/16 | 367 | 0.145 | 0/4 | 0/2 | 0.35 | 0 | 6/6 | 6/6 | 4 | 1173 | 0/6 | - |
| floor_s0 | 16/16 | 346 | 0.133 | 1/4 | 0/2 | 0.04 | 0 | 3/6 | 5/6 | 6 | 926 | 0/5 | 404 |
| floor_s1 | 16/16 | 364 | 0.152 | 0/4 | 0/2 | 0.27 | 0 | 5/6 | 5/6 | 7 | 1136 | 0/6 | 37 |
| floor_s2 | 9/16 | 547 | 0.13 | 0/4 | 0/2 | 0.25 | 0 | 3/6 | 6/6 | 4 | 1144 | 0/6 | 124 |

Totals over three seeds: `raw` 48/48 goals, 7/12 on-route toys and 1/6 beside driven over, 16/18 in the bin; final
`floor` 41/48 goals, 1/12 on-route toys driven over (the 15 mm brick, 0.04 m), 16/18 in the bin; v2 `floor` 45/48
goals, 1/12. Per-toy rows are in each run's JSON (`floor_toys_2026-11-05/`).

The `raw` robot drove over the low toys on its path (closest 0.01 m in two of three runs, the brick and the crayon
twice). Nothing marked them: the global costmap read 0 at every toy after every `raw` route. With `floor` the robot
went round them (closest on-route toy 0.25–0.52 m in eight of nine runs), and most toys were still marked after the
route. The tidy binned the same with both (16/18): toys the robot drives over in the sim stay where they were.

## What the diagnoses found

1. **The raw layer sees no toy.** Its 4 cm marking floor (set for the pods' 0.9° pitch and depth noise) is above every
   toy (12–40 mm). The Nov 5 "the D435's depth layer sees the toys" was the toys being off the stereo robot's path.
2. **Nav2's own clearing can't hold a toy out of view** (`floor` drafts with a VoxelLayer). A ray to the floor beside a
   toy crosses its 5 cm cell near the floor and erases it once the toy leaves the view (under the D435 at 7.5°, which
   sees no floor nearer than ~0.34 m from the centre, or past the field's side). Rays aimed to clear only their last
   cell never reach it: the VoxelLayer pulls every clearing ray's end back two cells. The rolling local costmap also
   lost its voxel marks when its window moved. Nav2's recoveries clear whole costmaps. Hence the node keeps its own
   grid (`ObstacleGrid`) and a StaticLayer reads it: a check (turn away, pass beside, pass beyond, look back) kept a
   toy at cost 100 in both costmaps; taken away and looked at, it cleared.
3. **A mark under the robot, or a wall of marks across a door, strands it.** v1_floor_s1: the bedroom's 12 mm carpet,
   seen through its door from the hallway, came out of `floor_vision` as a 40 cm "big" thing 7–12 mm tall; its marks
   closed the door, the robot stood on them, every plan started in collision, and Nav2's recoveries can't clear a
   StaticLayer. v2: a long low thing is floor (marked only from 25 mm), and marks under the robot are cleared each
   frame (Nav2's footprint clearing). The doors then stayed open (checked from both sides, `diag_office.log`).
4. **Raised floor edges still give false things.** v2_floor_s1: patches of the 16 mm white rug's square edge passed as
   small things and fenced off spot 7, which stands on that rug. The final rules (two sightings without a clear look
   at the floor between; ≤ 1.5 m) cut these but not out: with no toys at all, driving spots 4 → 5 → 6 left 181 marks
   on the open floor at the kitchen's raised tile edge (6 mm plus a 40 mm reducer ramp, `diag_spot6.log`), and
   floor_s2 lost seven goals in a row stuck at the office. `floor_vision`'s object test was made for the tidy's close,
   square-on looks; at driving speed it sees level steps at grazing angles, and in a costmap a few false cells inflate
   into door-closing blobs. The final rules also cost a late mark: floor_s0's brick was driven over.

## Picking and AI control with `floor`

- **Picks:** the tidy drives 0.8 m short through Nav2 and creeps in outside it; every tidy drive in a run that was not
  stranded arrived. The node skips frames while the arm is out and forgets the marks within 0.5 m of the robot when
  it comes back in; one mark was left where a binned toy lay in v1 (cost 100), none in v2 and final (0/34).
- **AI control:** `ros/tools/ai_smoke.py` on the D435 robot (`objects:=colours`, `depth_layer:=floor`, Python arm):
  **18/18** (spins, leases, take-overs, gotos, an unreachable goal, finds, both picks with only the asked thing binned,
  the fetch back).
- **Gotos at a marked toy:** 0.5 m short of it: arrived 0.06 m from the goal. Aimed 0.15 m from it, or onto it: Nav2
  stops at the nearest free point (~0.3 m from the goal, its planner's tolerance), robot centre 0.33–0.45 m from the
  toy, the toy untouched, and reports "Arrived". `robot_goto`'s evidence carries `distance_to_goal_m`.
- **Cost:** the node takes ~60 ms a D435 frame at stride 1 (Python, desktop PC, 5 Hz: ~0.3 core). At stride 2 it
  loses a 15 mm brick (`floor_vision`'s 1 cm grid).

## Conclusions

- **Adopted: nothing new.** `depth_layer:=raw` stays the default; it is what every earlier result ran.
- **Proposed: `depth_layer:=floor`.** It nearly ends driving over floor toys (7/12 → 1/12 on-route) but its false marks
  where the floor's level steps cost route goals (48/48 → 41/48; 45/48 for v2). Before it can be the default: tell a
  level step from a thing (e.g. absorb points into a floor level fitted over a larger window, or require floor at the
  thing's own level all round), score false marks near walls too, and rerun this study plus clutter and props.
- On the robot, the bumper (and the wheels) will meet the toys the sim lets pass under it: the sim understates
  pushing. A toy displacement model (rollers and fork touching toys) would let this study score pushes.

## Reproduce

```
bash toy_run.sh LAYER SEED TAG     # a stack on its own domain, then ros/tools/toy_route.py OUT.json --seed SEED --path path.json
```
(the run script used here lives with the session's scratch files; any isolated `omni.sh arm map:=house objects:=toys
seed:=S depth_layer:=raw|floor` stack plus `OMNI_CONTROL_PORT` and `python3 ros/tools/toy_route.py` reproduces a
run). Tests: `ros/src/omni_arm/test/test_floor_obstacles.py`, `ros/src/omni_bringup/test/test_depth_layer.py`,
`test_v11.py::test_the_launchers_live_world_person_clutter_and_thrower` (place_toys).
