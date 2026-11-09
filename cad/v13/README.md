# cad/v13: the D435 on the arm riser (Nov 5 2026, PROPOSAL)

A copy of cad/v12 round 15's generating scripts with one change: the arm robot's RealSense D435
moves from the nose to the front of the arm riser. Everything else is round 15 (`../v12/README.md` has the robot and its history). STEP first;
**the SolidWorks viewing assembly of the whole robot was rebuilt here on Nov 9 (Fernando), see "The SolidWorks assembly"**, and **this folder
reads no file from any other version's folder** (see "Files this version reads"). Status:
**the camera position is baselined (Fernando, Nov 5)** in design.md 4.11 and the sim (`robot_export.ARM` / `ARM_D435`, `D435_RISER`; the
nose kept as `ARM_D435_NOSE`); the CAD itself is nominal, nothing printed.

## The brief and the answer

The sim study (`simulation/v11 sil/studies/results/height_map_2026-11-05.md` sections 4-5) proposed the D435 on its side (portrait) on the
riser front, body centre (200, 0, 200) mm, 25-30 deg down. Priorities: 1 clear of the arm (>= 5 mm, every pose and move), 2 no lidar blind
angle, 3 nothing in the field of view, 4 inside the bumper, 5 a notch that cuts no fin / boss / dovetail, 6 a real USB-C plug and lead.

**Portrait does not fit, anywhere** (`validation/d435/scan.json`: body centre x 170..235, z 180..235, 25 / 30 / 35 deg). The 90 mm body
standing 30 deg off vertical spans 90 mm in z; the lidar wants its bottom >= 166 and the SO-101 wants its top under its base (222) or ahead
of the arm's shoulder, where the 0.26 m floor grasps sweep down past the riser. The sim's pose: 3.4 mm *into* the arm (stow <-> high), its
lowest corner 8.2 mm below the scan plane, 8.5 mm past the bumper. Every portrait pose that clears the arm reaches at least 18 mm below
the scan plane.

**Landscape at the sim's higher height (z 0.215) does not fit either**: its top meets the SO-101's base (7.3 mm into it). Forward of
x ~ 199 the floor grasps hit it at any height.

**What fits: landscape, low, in a pocket in the riser's front under the adapter**, its lower edge down through a notch in the floor's front
edge. Arm clearance there is set by the SO-101's fixed base above the camera, not by any move.

### Candidates (landscape, body centre z 193, tilt 30 deg; 25 deg in brackets)

Robot frame: x forward from the centre, z up from the floor, mm. Arm: the camera box (90 x 25 x 25) against the swept arm cloud.
Nearest floor: the depth image's nearest floor point, ray-cast against the chassis, pods at bump and the notched floor.

| Lens | Body centre (x, y, z) | Arm (worst) | Lowest point (scan 163) | Bumper (r 225.15) | Notch | Nearest floor |
| --- | --- | --- | --- | --- | --- | --- |
| at the floor's edge (218) | (210.8, 0, 193) | **-4.3** (ready -> hover, 0.26 m grasp) | 175.9 | 7.1 mm out | 1,180 mm2, x 203.6.. | 0.33 m (0.36) |
| 1 cm back | (200.8, 0, 193) | **3.8** (same move) | 175.9 | inside | 2,100 mm2, x 193.6.. | 0.32 m (0.35) |
| **2 cm back (recommended)** | **(190.8, 0, 193)** | **11.9** (SO-101 base) | 175.9 | inside | 3,050 mm2, x 183.4.. | **0.31 m (0.34)** |
| 3 cm back | (180.8, 0, 193) | 11.9 (SO-101 base) | 175.9 | inside | 4,010 mm2, x 173.6.., to y +54.8 (under the riser's left wall) | 0.30 m (0.33) |

(Notch: the floor area cut, from that x to the edge over y +-46; the lowest point is the camera box's, 0.6 mm under the floor's underside
for every landscape candidate; Intel's model reaches 178.1.)
| sim portrait (reference) | (200, 0, 200), roll 90 | -3.4 (stow -> high) | 154.8 (below) | 8.5 mm out | | 0.27 m |
| sim landscape z 0.215 (reference) | (200, 0, 215) | -7.3 (SO-101 base) | 197.9 | inside | none | 0.33 m |

No candidate puts any robot part in the depth field (0 % of the image), and none adds lidar blind angle (all >= 12.9 mm above the scan
plane). With the notch the floor's edge costs nothing (the same nearest floor as with no floor at all); without it the edge would sit in
the bottom of the image. 2 cm back is the most forward pose with a real margin to the 0.26 m grasps; 3 cm sees 1 cm nearer but takes a
third more floor, and its RGB field needs the notch under the riser's left wall.

## The recommended mount (drawn)

`tools/d435_riser.py` (`POSE = CANDIDATES['back_2cm_t30']`), built into the robot by `upper.py`:

- **Camera:** Intel's model, body centre (190.8, 0, 193.0), 30 deg down, landscape. Depth lens (left imager) at (198.0, 17.5, 188.8).
- **Cradle `bracket_d435_riser`** (printed, 16.1 cm3, prints on its feet without supports): a 4 mm back plate (the camera's two rear
  M3 x 8 from behind, on the bench) and an ear behind each end of it, standing on the riser's foot plate. One M3 x 12 through each riser
  side wall from outside into a hex nut dropped into a slot in the ear. To take the camera out: unplug it, the two side screws out, slide the
  cradle forward out of the pocket (the notch is open at the front). The camera then belongs to the arm module (riser, adapter, SO-101, buck,
  bus board).
- **Riser** (`riser_arm`): the extension's front opened between the side walls (y +-46) from x 153 (front wall, centre wall's front, and the
  foot plate where the camera dips below it, x > 177); the side screws' holes; a slot for the plug in the right wall. The side walls stay
  full height to the front and carry the adapter's front corners over the camera (the adapter spans 92 mm between them there). Its two floor
  screws moved (188, +-30) -> (158, +-30), out of the notch.
- **Floor notch** (`floor2_FL`, `floor2_FR`): from x 183.4 to the edge, y -46..+46 under the riser's walls, widening to +49.6 at the edge
  for the RGB field; sized from both fields at 30 and 25 deg (2 mm clear) and the body (1 mm). Removes 3,050 mm2 of plate (21.5 cm3 with the
  rim lip). It cuts no fin, fin screw, boss or dovetail; the front seam's dovetail tab at x 190 moved to 120 and its two lip screws (172,
  205) became one at 167 (the lip runs only between the lidar's keep-out and the notch).
- **USB-C:** a straight plug (12.4 x 6.5 x 21, as the bench cable) out of the camera's -y end through the riser's right wall, the lead
  (Ø4.5, VERIFY) bending sharply back 4 mm past the plug, then back along the riser's right side under the SO-101 base's right edge to the
  riser's rear (x 62); on to the computer is not drawn. No L plug needed here.
- **Nose:** `chassis.NOSE_D435 = False`: no nose cradle, camera, plug or field envelopes, nor the cradle's two pilots in `node_front`. The
  node keeps its round-15 outline, so nothing else in the chassis moves.

### Checks (`validation/`; omni-robot-sim venv, from `tools/`)

| Check | Result |
| --- | --- |
| `upper.py check` | no part overlaps, also against the chassis; no screw short of 6 mm; hole audit: nothing new (the face crown's two screws and the mic board's vendor holes as round 13); battery slides out |
| arm (`d435_riser.check_mount`, the swept cloud: 8,141 configurations, every pose and move in the brief; 9 of 50 grasp targets have no IK at 0.33-0.36 m, one move has no planned path) | camera 13.3 mm, cradle 11.0, plug and lead 17.8, each from the SO-101's fixed base; every moving part is further |
| `arm_sweep.py` (official meshes, 33 planned moves) | >= 12.2 mm from everything (the gripper to the bumper at the 0.26 m grasp); none near the D435 |
| lidar | mount's lowest point z 178.1 (camera), 15.1 mm over the scan plane: blind angle unchanged, 20.9 deg (`chassis.py check`) |
| fields of view (500 mm frustums; depth 89 x 58, RGB 69 x 42; arm at stow, front pods at bump) | nothing of the robot in either |
| view (ray cast) | nearest floor 0.31 m ahead at 30 deg (0.34 at 25); nose today 0.34, sim portrait 0.27 |
| bumper | camera r <= 209.4, plug and lead <= 199.9: inside 225.15 |
| static gaps | camera to the riser 0.9 mm, to the floor's notch 1.2, to the adapter 3.2, to the front fins 6.5; plug to its wall slot 0.6 |
| `chassis.py check` | no overlaps, nothing thinner than 4 mm, every fastener lands, every part fits the bed, lidar blind 20.9 deg |

Renders: `validation/d435/side_stow.png`, `side_grasp_0.26.png` (from the right), `top_notch.png`, `iso_front.png`. STEPs:
`step_export/` (printed parts only, as v12) and `step_export/d435_mount_review.step` (the printed parts round the mount **plus the D435's
model and its plug + lead**, for review; drop it if only printed parts are wanted).

### In the sim (baselined Nov 5)

`robot_export.ARM` (and so `ARM_D435`) carries **body centre (0.1908, 0, 0.1930) m, 30 deg down, roll 0 (landscape)**, depth origin
(0.1980, 0.0175, 0.1888) m, and cuts the sim's riser block back 10 mm for it (`arm_riser_pocket`); the generated `robot_arm*.json` and
frames follow. Alternative 25 deg down: centre (0.1905, 0, 0.1930). Rerun there (Nov 6; design.md 4.11):

- `studies/d435_mount.py` (`riser_cad`): close look 25/40 at 30 deg, 35/40 at 35; floor valid 63/46 % at 1.5/2 m; no tags; no table
  or chair overhang at 25-35 deg (`--overhang`, `d435_mount_riser_cad_overhang.json`).
- `studies/d435_mount_pick.py` (`riser_cad`, `riser_cad35`; 6 toys, 4 seeds): 18/24 binned at 30 deg, 20/24 at 35 (nose 22/24),
  no failed grasps.
- `studies/arm_calib.py --out arm_calib_riser.json`: keyed block p95 0.27 mm, gripped 0.95 mm, as at the nose.

Expect against the portrait proposal: lower (z 0.193 instead of 0.200, lens 0.189 instead of 0.211), so less far floor; a 58 deg tall
field, so the 0.27 m blind zone becomes ~0.31 m; at 30 deg the field's top is 1 deg below the horizon, so no table or chair overhang (the
sim's landscape 30 deg saw neither table; portrait's top was 15 deg above); the arm's close look and the floor grasps will look different again. Tilt was not re-chosen: 30 deg was the brief's;
the notch is sized for 25 and 30 (35 would need 0.4 mm more on the left).

### Open / VERIFY

- The camera's two rear M3 (45 apart, thread depth) and the USB port's end: from the vendor model (`-y` end, VERIFY on the unit).
- The plug and lead envelopes (straight plug 21 mm, lead Ø4.5): VERIFY on the cable bought; the lead's tightest bend is modelled sharp.
- The adapter now spans the camera's pocket: 92 mm between the riser's side walls, with no centre wall or front wall under its front 52 mm
  (x 153..205; the SO-101's front M4s at x 195.5 thread into the adapter alone, as before). No strength check. The camera rides with the arm
  module.
- The camera's static gaps (0.9 mm to the riser, 1.2 to the notch) are nominal: print tolerance may want 0.5 mm more each side.
- Lens positions in the field envelopes are assumed (17.5 / 32.5 mm left of the centre), as round 15.
- The SolidWorks files follow the STEPs only when their stages are re-run (section below): after any geometry change regenerate the STEPs and rebuild
  the changed parts and every assembly above them. The Nov 9 build is of the geometry as it stands in this README (nothing changed since).

## The SolidWorks assembly (Nov 9, Fernando: "all parts in there")

Open **`solidworks/V13_robot.SLDASM`** (the one file to open; its sub-assemblies are in `solidworks/assemblies/`, the parts in `solidworks/parts/`, 203 files in all).
Imported STEP bodies in sub-assemblies, not native features (`../solidworks.md` section 17), and a viewing assembly of a **proposal**: it carries
the status of the CAD above, nothing printed. 686 components at every depth, none suppressed or hidden (SolidWorks loads them lightweight; its API
reports lightweight as "suppressed", which is why `sw_robot.py paths` counts both):

| Sub-assembly | Components |
| --- | --- |
| `V13_frame` / `mounts` / `bumper` | hub + 4 nodes / lidar pedestal + 5 fins / 4 bumper quarters |
| `V13_reference` | the vendor RPLIDAR and the D435's depth and RGB fields as see-through envelopes (blue / green, 500 mm) |
| `V13_upper` (36) | 4 floor quadrants, arm riser + adapter, the D435 cradle, electronics holders, bin deck + posts, face frame, housings |
| `V13_electronics` (39) | computer, battery, power board, Teensy carrier, servo adapter (envelopes, plugs, labels), 12 V buck, bin, 5 in display, mic array, speaker, e-stop, switch, **the D435 with its USB plug + lead** |
| `V13_arm` (17) | the SO-101's official parts at the stow pose |
| `V13_fasteners` (172) | every screw, nut and dowel outside the pods, magenta (`fst_*`) |
| `V13_pod` (RL, FR) / `V13_pod_m` (FL, RR) (98 each) | static frame (keel, sleeve, dowel, springs), unsprung fork + 627s + wheel (hub, cartridge, 6804, 12 rollers = core + tread, axles) + AK40-10 (stator, rotor, XT30, leads), 4 spring plungers; the four pods are flexible |

Only the suspension has mates (hinge on the inner dowel, each plunger in its guide with its dome tangent to the fork pad, `Suspension_travel` limit); everything
else is fixed where build123d put it. Orientation: robot z = SolidWorks Y, front toward the Front view. Printed parts carry a `PRINT_BED (... up)` plane.
Not modelled: cables and the harness beyond the D435's lead and the motor's, strength, printing. The FL / RR pods are mirror images, not a real opposite-hand motor plug (as v12).

Checks (`validation/solidworks/`, logs `solidworks/sw_*.log`), Nov 9:

| Check | Result |
| --- | --- |
| mates | 0 errors in every file; `Suspension_travel` and hinge mates in both pod hands |
| `sw_robot.py test` (pod motion, both hands) | axle droop -3.0 / nominal 0 / bump +15.0 mm (lateral 0.06 / 0 / 1.41), plunger lifts within 0.05 mm of build123d's, dragged 6 deg past each end it stops at the limit; the top assembly opens with 4 flexible pods, axles 0.000 mm off their pod origins |
| `sw_robot.py paths` (the one-folder rule) | all 686 components = 203 files, **every one inside `cad/v13`** |
| `sw_robot.py interference` (whole robot, nominal; 393 rows) | **`pass: true`: no overlap between printed or purchased parts and anything else, 0 fasteners beyond the thread allowance** (357 thread-formed screws). Listed apart in `vendor_internal` (36 rows, identical to v12's, not counted): 20 inside the SO-101's own official parts at their joints (up to 1,994 mm3) and 16 where the motor model's pins meet their leads (<= 2.9 mm3). The field envelopes touch only each other and the camera (they start at its lens) |
| self-containment | the generator re-run in a sandbox holding only `tools/`, `reference/` and the caches (no other cad folder) gave the identical manifest and the same 182 STEPs |

Views: `validation/robot/solidworks_robot_iso.png`, `solidworks_robot_trimetric.png` (zoom to fit includes the 500 mm field frustums, so the robot is small),
and `solidworks_robot_{iso,front,right,top}_close.png` from `sw_robot.py views_close` (the two field envelopes hidden for the picture only, nothing saved; an explicit
zoom box, so they are previews: the wide window cuts the top of the arm and display in the front and right views; open the assembly for the real look).
Not run here: the d435 / arm clearances are the build123d checks above; SolidWorks was not used to sweep the arm.

## Files this version reads (Nov 9, Fernando: always pull from the current version's folder; `../solidworks.md`, top)

Everything v13's scripts and SolidWorks files use is inside `cad/v13`. What other folders used to supply was copied into `reference/` (names kept):

| Copied into `reference/` | From | Used by |
| --- | --- | --- |
| `parts/SO-101_*.step` (13 official parts), `12_V_buck_arm`, `label_12V_BUCK_ARM`, `bin`, `5_in_display_800x480`, `USB_mic_array`, `micro_speaker_4_ohm_3_W` (`.step`) | cad/v10 `step_parts/` | `upper.py` (arm, buck, bin, face) |
| `parts/REC_arm_riser.step`, `REC_arm_adapter.step` | cad/v10 `step_export/sim_reconciliation/parts/` | `upper.arm_kit` |
| `parts/d435_source.step` (Intel's D435) | cad/v10 `reference/` | `chassis.d435_real`, `d435_riser.build_mount` |
| `parts/AK40-10 with connector.STEP` | `components/ak40-10/` | `wheel_motor.py` |
| `parts/RPLIDAR.SLDPRT` (vendor lidar, Ø76 x 41) | the Roomba robot project, `CAD/v2/` | `sw_robot.py` (VENDOR) |
| `so101_links.json`, `robot_sim_instances.json` | cad/v10 `reference/` | `screen_study.py` (arm part poses), `upper.py` (buck, bin and screw positions) |
| `fov_meshes/SO-101_*.npz` (13) | cad/v10 `validation/fov_meshes/` | studies only (`arm_sweep.py`, `d435_riser.py`) |

`tools/subasm.py` and `tools/sw_native.py` are trimmed copies of cad/v10's SolidWorks helpers, `tools/mate_api.py` an unchanged copy. Already here
from before: `reference/bearing_6804.step`, `bolt_M3_12_FLAT.step`, `bolt_M3_50_PAN.step`, `imu_bno085.step`. **One import that is not CAD:** the arm's
poses come from the robot's own kinematics, `ros/src/omni_arm` (`screen_study.py` puts it on the path), so the stow pose follows the robot code;
`imx662_fit.py` (a study) reads the sim's `simulation/v11 sil` the same way. Neither feeds the SolidWorks files except the stow pose.
Check after any change: `grep -rnE "v1[0-9] human|v1[0-2]/|components|Roomba|C:/" tools/*.py` shows only comments, and the SolidWorks check `sw_robot.py paths`
(below) fails if any component file is outside this folder.

## Regenerate

```
python -B tools/d435_riser.py motion | cloud | scan | candidates | render   (the study; caches in validation/d435/)
python -B tools/chassis.py all      python -B tools/upper.py all     python -B tools/arm_sweep.py     python -B tools/robot_step.py
```

The SolidWorks assembly (`C:/Users/ferna/cadenv/Scripts/python.exe` for `sw_robot.py`; one stage group per process, SolidWorks leaks memory; never with
other people's documents open: it closes only files under `solidworks/`):

```
python -B tools/sw_robot_parts.py                      (omni-robot-sim venv: part STEPs + solidworks/manifest.json; minutes)
sw_robot.py import                                     (STEP -> V13_*.SLDPRT, only the parts that changed; tens of minutes the first time)
sw_robot.py roller wheel motor unsprung static         (the pod's sub-assemblies, both hands)
sw_robot.py pod
sw_robot.py group_frame | group_mounts | group_bumper | group_reference | group_upper | group_electronics | group_arm | group_fasteners
sw_robot.py robot                                      (the top assembly)
sw_robot.py test | paths | interference | views | views_close   (checks and pictures; -> validation/solidworks/, validation/robot/)
```

`solidworks/run_stages.sh <stage> <stage> ...` runs them one process each and stops at the first failure (log `solidworks/sw_stage2.log`).
