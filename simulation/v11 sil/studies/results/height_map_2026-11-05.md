# What the D435 adds to the lidar: realistic depth, a fused height map, the mount (Nov 5, 2026, sim only)

Follow-up to `floor_toys_2026-11-05.md`. Fernando's direction: obstacles and general navigation first (the RealSense
augments what the lidar can't see), toys second; realistic D435 noise in the sim; a multi-frame fused map; speed by
certainty (once a thing is known to be a rug, speed up); the AI names uncertain things through MCP; explore mounting
the D435 higher, where the arm is (used with the arm stowed); a dedicated tag camera is an option, so the D435 can be
placed for depth. PC only (WSL2 + Windows), isolated stacks (ROS domains 57–60), sim clock; three stacks ran at once,
so routes ran slower than real time (not recorded per run). Single runs per seed.

## 1. Realistic D435 depth (simulation/v11 sil/depth_noise.py)

Now the default for the D435 in the sim (`sil_pick.PickCameras(noise="realistic")`, bridge `d435_noise`; `ideal` is
the Nov 4 model). Every term is an estimate until `characterization/realsense/rs_depth_plane.py` measures the unit:
disparity noise 0.1 px (half of its variance smooth over ~5 px: the matcher's window), a 0.3% range scale error per
camera, grazing-angle noise and dropout (from 75° between the ray and the surface's normal, everything gone by 88°),
flying pixels at depth steps, stereo occlusion left of nearer objects, dropout on IR-dark surfaces (by each geom's
albedo), 2% random dropout.

| Check (synthetic, 480 × 270) | Result |
| --- | --- |
| Wall at 1 / 2 m | range sd 8.2 / 32.8 mm (z² law: 8.2 / 32.9) |
| Floor from the nose (0.11 m up, 7.5° down) | valid pixels 93% at 0.3–0.6 m ahead, 63% at 0.6–1.0, 35% at 1.0–1.5, 17% at 1.5–2.0, 5% at 2–3 |
| Box 1 m before a wall 2 m away | flying pixels between; a 6 px band left of the box unmatched (its disparity step) |
| Cost | 43 ms a frame in the bridge (+ a segmentation render for albedo) |
| Picking (`sil_pick --camera d435`, arena, 6 toys) | seed 0 5/6, seed 1 6/6 in the bin (ideal model: 6/6, 6/6) |

The grazing-angle loss is the term that matters most and the least certain one: measure it on a floor first.

## 2. The fused height map (`depth_layer:=height`, proposed; ros/src/omni_arm/omni_arm/height_map.py)

Each frame's points against the floor plane that frame sees, binned per map cell (median, 90th percentile, 10th
percentile, range); fused per cell (recent frames weigh more; a look 8 mm off restarts it, so a moved thing clears in
a couple of frames); kept while out of view. Classes: floor; step (raised 4–30 mm over a big region: rug, sill, tile;
drivable); low (8–30 mm, compact; slowed for, catalogued); obstacle (3 cm up to the robot's top, overhangs too, seen
twice; stale after 2 min unseen); drop (≥ 5 cm under the robot's floor, seen within 1.5 m in 3 frames over 6 cells,
plus its unseen shadow 0.6 m); unknown (and any unconfirmed drop). Low things and obstacles are judged on relief over
the local floor (0.5 m round), which cancels the frame planes' small tilt errors; those grow with range and, in the
first version, faked a drop whose shadow spread over 2,500 cells. Only obstacles and drops go to Nav2 (a StaticLayer:
Nav2's recoveries can't clear it). The node also publishes `/speed_limit`: along the way the robot is going, the
nearest unknown cell sets a speed it can stop in (0.8 m/s², never under 0.25 m/s), a low thing near the path caps
0.4 m/s, a silent camera 0.3 m/s; known floor or step leaves the speed alone; merged with the console's floor-edge
zones (`/speed_limit_zones`). Driving sideways or backwards into ground the forward camera hasn't seen is slow by
itself. ~25 ms a frame on the PC (Python, 5 Hz).

### Obstacle study (`ros/tools/toy_route.py --shoes 2`, house, 2 laps, 3 seeds, realistic noise, nose D435 at 7.5°)

Six toys (four on the path) and two shoes (11 cm: under the lidar's 163 mm plane, 0.4 kg, they move when hit) placed
on the route.

| Run | Goals | Route s | Top speed m/s (median of legs) | Shoes closest m | Shoes moved | Shoes marked | On-route toys driven over | False obstacle cells (max) | Speed capped < 0.8 m/s | Why (samples) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| raw_s0 | 16/16 | 577 | 0.89 | 0.47, 0.46 | 0 | 1/2 | 3/4 | – | – | – |
| raw_s1 | 16/16 | 498 | 0.97 | **0.05**, 0.45 | **1 (1.24 m)** | 1/2 | 1/4 | – | – | – |
| raw_s2 | 16/16 | 595 | 0.88 | 0.46, 0.36 | 0 | 0/2 | 2/4 | – | – | – |
| height_s0 | 16/16 | 655 | 0.82 | 0.61, 0.69 | 0 | 2/2 | 3/4 | 9 | 0.41 | unknown 213, low thing 139 |
| height_s1 | 16/16 | 651 | 0.71 | 0.57, 0.44 | 0 | 2/2 | 2/4 | 10 | 0.42 | unknown 229, low thing 115 |
| height_s2 | 16/16 | 669 | 0.79 | 0.47, 0.54 | 0 | 2/2 | 3/4 | 32 | 0.44 | unknown 231, low thing 133 |

The raw layer (marks above 4 cm out to 1.2 m, clears along rays) drove into a shoe and pushed it 1.24 m, and had lost
4 of the 6 shoes' marks by the end of the route (its rays to the floor beyond erase what left the view). The height map kept every shoe marked and the robot ≥ 0.44 m from
them, with no blocked goal, at a cost of ~15–30% route time (slower top speeds: it slows for unknown floor ahead and
for the low toys on the route). Low toys are still driven over by design: they only slow the robot (Fernando: toys
second). The false obstacle cells (open floor, ≥ 0.3 m from walls and toys) stayed small and blocked nothing.

## 3. The AI's catalog (omni_mcp `robot_floor_unknowns`, `robot_floor_label`)

The height map adds small low and obstacle regions within 2 m (not walls) to a catalog
(`$OMNI_STATE_DIR/floor_catalog.json`) with a crop of the colour image round each; `/floor_obstacles/candidates`
tiles the newest unnamed crops. The model looks and names each (floor, rug, step, low, toy, obstacle, drop); the label
goes into the map. A label that relaxes never clears what the camera measures as an obstacle or a drop.

- **End to end through omni_mcp** (`objects:=colours`, `depth_layer:=height`): after one spin, 5 things catalogued
  (3 low, 2 obstacle: the colours scene's cubes, block and ball), the picture of their crops returned
  (`height_map_2026-11-05/floor_unknowns.jpg`: each object recognizable; crops not always centred), a label sent
  (`toy`) came back in the catalog. The first try returned no picture: the tiles were sent only when the catalog
  changed, and web_video_server's snapshot waits for the next frame; now sent again each second.
- **ai_smoke** with `depth_layer:=height`: 17/18, the 5-turn spin's task over its wall-clock deadline (true turn
  1797°; sim at 0.43× real time beside other stacks); the spin alone, the machine quieter: 2/2. Gotos, the lease, the
  page's take-over, finds and both picks passed.

## 4. Where the D435 should sit (simulation/v11 sil/studies/d435_mount.py)

Arena floor, the arm stowed, realistic noise. Floor: valid pixels at 0.5 / 1.0 / 1.5 / 2.0 m ahead. Shoe: the
range band where ≥ 30 of its pixels stand ≥ 3 cm up. Robot: the share of the image the robot (the stowed arm) fills.
Tags: house poses (8 headings) that see a wall tag in the colour image, tags at 0.9 / 1.3 m. Close look: the tidy's
CHECK at the reach's sweet spot (8 things × 5 offsets, floor_vision).

| Mount (body centre), tilt down | Floor valid | Blind zone (from the centre) | Robot in view | Shoe seen | Tags 0.9 / 1.3 m | Close look |
| --- | --- | --- | --- | --- | --- | --- |
| Nose (0.19, 0, 0.11), 7.5° (today) | 97 / 59 / 32 / 18% | 0.34 m | 0% | 0.4–2.2 m | 40 / 7% | 10/40 |
| Riser front (0.20, 0, 0.215), 5° | 95 / 95 / 71 / 51% | 0.51 m | 1% | 0.4–2.2 m | 55 / 32% | 0/40 |
| Riser front, 10° | 96 / 94 / 74 / 54% | 0.46 m | 0.5% | 0.4–2.2 m | 32 / 0% | 0/40 |
| Riser front, 20° | 98 / 95 / 69 / 51% | 0.39 m | 0% | 0.4–2.3 m | 0 / 0% | 0/40 |
| Riser front, 30° | 98 / 94 / 71 / 52% | 0.33 m | 0% | 0.4–2.5 m | 0 / 0% | 12/40 |
| Riser front, 35° | 98 / 95 / 71 / 0% | 0.31 m | 0% | 0.4–1.5 m | 0 / 0% | 30/40 |
| Riser front, 40° | 97 / 95 / 0 / 0% | 0.29 m | 0% | 0.4–1.0 m | 0 / 0% | 40/40 (no step-back) |
| Head over the arm (0.08, 0, 0.42), 30° | 45 / 66 / 78 / 88% | 0.33 m | 27% | 1.3–2.3 m | 0 / 0% | 0/40 |
| Head, further forward (0.16, 0, 0.40), 30° | 21 / 27 / 48 / 87% | 0.40 m | 45% | 1.3–2.3 m | – | – |

- **Height is what buys floor sight:** at 0.21 m the D435 sees the floor at 1.5–2 m about twice as well as at 0.11 m,
  whatever its tilt (grazing angles lose the floor). The nose mount's tilt hardly changes its floor.
- **At head height the stowed arm blocks a quarter to half of the image**, and nothing nearer than 1.2 m is seen. Not
  a candidate unless the arm stows elsewhere.
- **The riser front's tilt trades tags and range against picking.** With tags on the D435: 5° (more tags than the nose
  and twice its floor; the tidy's close look then needs another step-back or the wrist camera). With a dedicated tag
  camera (Fernando's suggestion): 30° (obstacles to 2.5 m) to 35° (to 1.5 m, close look 30/40).
- **Not checked:** the arm's own path past a camera at (0.20, 0, 0.215) (it sits in front of the arm's base, where the
  arm sweeps down to the floor: a CAD keep-out check), the USB lead, the real unit's grazing-angle returns.
- Raw: `d435_mount.json` (quick sweep), `d435_mount_full.json`, `d435_mount_riser_low.json`,
  `d435_mount_riser_steep.json`.

## 5. The D435 on its side, overhangs, and the whole tidy per mount

Fernando's question: the riser at 30–35° but turned 90° (portrait), so the 87° field is the tall one; and the arm's
picking with each mount, the cameras watched while the arm moves. Sim: `d435_roll_deg` (90 = portrait: the body
stands 90 mm tall, the imagers above one another, the image upright at 270 × 480 with an 89° tall field, 58° across;
the noise model's occlusion band then lies along columns). A study variant, not the baseline.

**Portrait doesn't fit at the landscape's height.** At the riser front (0.20, 0, 0.215) the 90 mm body reaches
0.265 m and meets the arm in every fixed pose (stow, high, predrop, drop; omni_arm's keep-outs, 5 mm margin). It
clears with its centre at ≤ 0.205 m; portrait runs below use 0.20 m (body 0.155–0.245 m). Not checked in CAD.

### Views (`d435_mount.py --overhang`, arena, the arm stowed, realistic noise)

Overhang: a 1.0 m × 0.6 m table top with its underside 0.30 m (a low table, a bed frame) or 0.45 m (a table, a
chair seat) up, its near edge straight ahead; the band of ranges where ≥ 30 of its pixels are seen. The lidar's
163 mm plane passes under both; the robot stands ~0.6 m.

| Mount, tilt down | Blind zone | Floor valid 1.5 / 2.0 m | Shoe seen | Table 0.30 m | Table 0.45 m | Tags 0.9 / 1.3 m | Close look | Robot in view |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Nose 7.5° (baseline) | 0.34 m | 32 / 18% | 0.4–2.2 m | 0.3–2.2 m | 0.6–2.3 m | 40 / 7% | 10/40 | 0% |
| Riser 20° | 0.39 m | 69 / 51% | 0.4–2.3 m | 0.4–2.4 m | 1.5–2.4 m | 0 / 0% | 0/40 | 0% |
| Riser 30° | 0.33 m | 71 / 52% | 0.4–2.5 m | never | never | 0 / 0% | 12/40 | 0% |
| Riser 35° | 0.31 m | 71 / 0% | 0.4–1.5 m | never | never | 0 / 0% | 30/40 | 0% |
| Riser portrait 20° (z 0.20) | 0.31 m | 77 / 56% | 0.4–2.3 m | 0.3–2.4 m | 0.3–2.4 m | 40 / 21% | 6/40 | 2% |
| Riser portrait 25° | 0.29 m | 76 / 58% | 0.4–2.4 m | 0.3–2.5 m | 0.4–2.5 m | 16 / 0% | 37/40 | 1% |
| Riser portrait 30° | 0.27 m | 76 / 58% | 0.4–2.5 m | 0.3–2.5 m | 0.7–2.5 m | 0 / 0% | 39/40 | 0% |
| Riser portrait 35° | 0.25 m | 76 / 58% | 0.4–2.6 m | 0.3–2.5 m | 1.4–2.5 m | 0 / 0% | 39/40 | 0% |

(Portrait at z 0.215, which doesn't fit, scored within a few points of these: `d435_mount_portrait.json`.)

- **Portrait wins on every depth measure at the same tilt**: a smaller blind zone, more valid floor far out (480 rows
  instead of 270 over the floor), and it sees overhangs the steep landscape mounts never see. At 30° landscape the
  field's top is 1° below the horizontal; portrait's is 15° above.
- **The nose's level view also sees overhangs** (its field's top is 21° up), but it sees the floor poorly past 1 m.
- **Portrait's cost is width**: 58° across instead of 87°. The look round turns in smaller steps (`pick.TidyTask`
  now takes the turn and the in-view bearing from the camera's field: 48° turns and ±22° for portrait; the landscape
  D435 and the stereo pair keep 60° and ±38°). Obstacles beside the path are seen later on turns.
- Tags: portrait 20° sees them as well as the nose; at 25° and steeper, a tag camera is needed.

### The whole tidy per mount (`d435_mount_pick.py`, sil_pick arena, 6 toys, seeds 0–3, realistic noise)

The robot's code given each mount's camera (robot_export.arm_json of that pose; the nose's equals
`robot_arm_d435.json`), the close look's step back per mount (nose 0.10 m, riser 30° 0.10, riser 35° 0.05, portrait
25° 0.05, portrait 30° 0). An observer samples every 0.25 s of sim time without touching the run (the plain
`sil_pick.py --camera d435 --seed 0` gives the same 5/6 at the same 170.8 s).

| Mount | Toys in the bin | Picked (task) | Failed grasps | Sim time per tidy |
| --- | --- | --- | --- | --- |
| Nose 7.5° (baseline) | **22/24** (5, 6, 6, 5) | 23 | 0 | 171–269 s |
| Riser 30° | 16/24 (4, 4, 5, 3) | 18 | 0 | 117–235 s |
| Riser 35° | 20/24 (5, 4, 6, 5) | 21 | 0 | 176–228 s |
| Riser portrait 25° | 21/24 (5, 5, 6, 5) | 21 | 0 | 193–237 s |
| Riser portrait 30° | **22/24** (6, 5, 6, 5) | 23 | 0 | 208–263 s |

Every tidy ended "nothing in sight": the losses are toys never found (and 1–2 picked ones that missed the bin), not
failed grasps. Landscape riser 30° found fewest; its close look (12/40) is the weak step. One run per seed: a
difference of 1–2 toys is within the noise.

**Imaging while the arm moves** (all seeds pooled; the arm's share of the D435's image; the target "hidden" when its
centre is in the field but < 10 of its pixels show):

| State | Uses the D435 | Arm in the D435's view, mean (max): nose / riser 30° / portrait 30° | Target hidden | Target in the wrist camera's field |
| --- | --- | --- | --- | --- |
| APPROACH, CHECK, CREEP | yes (snapshots) | 0 / 0 / 0 | 0% | – |
| HOVER | no | 3% (11) / 10% (25) / 19% (76) | 0–1% | 58–64% |
| REFINE | no | 6% (11) / 18% (25) / 31% (76) | 0% | 100% |
| DESCEND | no | 16% (25) / 23% (28) / 34% (49) | 0–3% | 81–85% |
| CLOSE, LIFT | no | 14–23% / 22–24% / 34–36% | 0–22% | 9–37% (too near to centre) |
| TO_BIN, STOW | no | ~0 | – | – |

- **The tidy never reads the D435 while the arm is out.** It looks (LOOK, CHECK), drives (APPROACH, CREEP) and then
  hands over to the wrist camera (REFINE) and the arm's own kinematics. Not visual servoing: the D435 places the
  thing once; the wrist camera refines it before the descent. So the arm filling the D435's view (a quarter at the
  nose, up to three quarters for a moment in portrait) costs the tidy nothing today. It would matter for an AI
  watching the grasp through the D435, or for obstacle sight while the arm is out (the robot is stopped then).
- The target stays visible to the D435 through the grasp at every mount (hidden ≤ 8% of samples, except riser 30°
  at the close: 22%), so the D435 could confirm a grasp or a miss if wanted.
- Pictures: `d435_mount_pick/<mount>_arm_in_view.png` (the moment the arm fills most of the view, seed 0).

## Conclusions

- **Adopted: the realistic D435 noise in the sim** (estimates; `ideal` reproduces earlier runs).
- **Proposed: `depth_layer:=height`** (default stays `raw`). In the sim it kept every under-lidar obstacle marked and
  avoided, blocked no goal, at ~15–30% route time. Before it is the default: a real-camera bench (noise, grazing
  returns on the house's floors), furniture and props in the sim, a drop (stairs) course, the Pi's CPU.
- **Proposed (replaces the landscape riser): the D435 on its side on the riser front, centre ~0.20 m, 25–30° down,
  with a dedicated tag camera.** Against the nose (sim): the floor at 1.5–2 m seen ~2.5–3× as well, a smaller blind
  zone, overhangs seen, tidy as good (22/24 at 30°, 21/24 at 25°; nose 22/24), the close look needing no step back
  at 30°. Costs: a 58° wide view, the arm across a third of its view while it picks (unused then), and the tags. Without a
  tag camera: portrait 20° (tags as the nose, 6/40 close look, so the step back stays). Before adopting: a CAD check
  of the body and its lead under the arm's base and past its sweep, and the real unit on its side (its driver
  still streams landscape images: the robot's code would turn them and their intrinsics; the sim assumes that done), and a rerun of the arm calibration study
  (`studies/arm_calib.py`, run at the nose only, where its colour camera sees the gripper's tag 0.28–0.60 m out).
- Landscape riser 30–35°: dropped. It sees no overhang, and at 30° the tidy found fewest toys.
- Toys under 3 cm: slowed for, not avoided; the catalog lets the AI name them.

## Reproduce

`bash toy_run.sh height|raw SEED TAG --laps 2 --tidy 0 --shoes 2` (any isolated arm stack with `objects:=toys
depth_layer:=...` and `OMNI_CONTROL_PORT`, then `python3 ros/tools/toy_route.py OUT.json --seed S --path path.json
--shoes 2`); `python studies/d435_mount.py --quick` / `--only riser:30 --out x.json` / `--overhang --only
riser_p:30` (`d435_mount_portrait.json`, `d435_mount_portrait_low.json`); `python studies/d435_mount_pick.py --only
riser_p30` (`d435_mount_pick/pick_<mount>.json`, the configs `robot_arm_d435_<mount>.json`). Tests: `test_v11.py` (the noise
model), `ros/src/omni_arm/test/test_height_map.py`, `ros/src/omni_bringup/test/test_depth_layer.py`, omni_mcp's.
