# v10 assemblies: the robot as sub-assemblies (Nov 1)

`V10H_robot.SLDASM` is the arm robot. Nothing in it is one flat list: every group is its own file in `assemblies/`.

## Nov 1 simulation height reconciliation

The active assembly now uses `reference/robot_sim_instances.json`; the extracted
v9 table and the previous assemblies are preserved (`superseded/before_sim_height_reconciliation/`).
Only CAD files were changed; simulation, ROS, firmware and the root design document
were deliberately left for the agents working there.

| Control (ROBOT frame, mm) | Revised CAD |
| --- | --- |
| Payload plate top | 181.5, unchanged |
| Riser top / physical height above plate | 212 / 30.5 |
| Adapter top / SO-101 frame origin | 222.1 / nominal 224.5 |
| Whole arm relative to extracted v9 | −13 in Z |
| Camera-bar axis | (155, 0, 200), +5 over v7, −8 from v9 |
| Rear mast face seat | (−187, 0, 436.5), +6.5 over v7 |
| Mast tube length / bottom datum | 357 / 73 |

The simulation's `arm_riser=37 mm` measures to its simplified adapter's
underside at 218.5; the official arm's frame is another 6 mm above that.
The detailed CAD has a 10.1 mm adapter and the arm underside is 2.4 mm below
its origin, hence its distinct physical riser height. These frame conventions
must not be confused with conflicting target arm heights.

The camera-bar ring/web moves down 8 mm, but the cheeks' plate clamps and
their screw seats remain on the plate. The riser's buck bosses and buck move
down 13 mm; plate attachment screws remain seated. The detailed adapter needed
a local underside recess, x137.5–172.5, with underside z217.2, to clear the
bar's full square-tube envelope by at least 3 mm. It leaves 4.9 mm of bridge
material there and full 10.1 mm thickness around the arm/riser mounting screws.
Strength of this thinner bridge remains untested.

Fernando chose to **retain the rear mast layout**. The mast's feet stay on
the deck while shorter posts lower the crown, screen and audio together by
6.5 mm. The simulation still uses the earlier face-over-bin boxes; that
horizontal-layout difference is intentional and is not claimed reconciled.

Reproduction: `47_reconcile_geometry.py` generates the revised STEP parts and
instance table without editing frozen v9. `48_reconcile_solidworks.py` imports
them and updates the assemblies. The revised imported parts are `REC_*` in
`parts/step/`; existing native wheel/pod parts are preserved. Independent
STEP/arm checks are in `49_check_reconciliation.py` and
`validation/sim_reconciliation_*.json`. These checks do not certify print
strength, all possible arm poses, or the previously open deck/pod interfaces.

Completed checks for this revision: 66 exact STEP interface pairs with no
volume clashes; high, drop and three floor-grasp poses (x270/310/340) without
robot or nonadjacent arm-link overlaps. At the floor grasps the official
finger meshes end approximately 2.0 mm above the floor. All 18 native
SolidWorks joint-motion checks pass. The separate
`V10H_robot_floor_reach.SLDASM` is a saved x310 floor-grasp witness: SolidWorks
reads tool (310.42, 0.02, 8.26), target (310, 0, 8.3), with no mate errors.
The small native/ideal difference comes from the inherited face-based joint
mates. The main robot remains in its delivered high pose. Printed STEP files
are in `step_export/sim_reconciliation/parts/`.

## RealSense D435: centre camera bar and separate optical fields (Nov 2)

**The old nose mount is superseded.** Its body did not clash, but its optical
fields crossed the front bumper (all three imagers) and deck (both IR imagers).
Those earlier body-clearance results never established unobstructed vision.
The old assembly, deck and written conclusions are preserved under
`superseded/d435_nose_before_fov_fix/`.

`assemblies/V10H_d435_bar.SLDASM` now puts Fernando's actual D435 in the middle
of the camera bar, on a forward saddle. Its nominal body centre is
**(198.991, 0, 183.974) mm**, looking **40° down**. In the indexed bar frame
its centre is (44, 0, 16). Moving the camera directly against the tube clashes
with the lidar; the short forward bracket clears both the lidar and optics.
The source camera part remains unmodified.

The single printed U saddle leaves the tube's upper corners bare, retaining
the tube/adapter gap. Two M3×30 screws go through new Ø3.4 holes at bar-local
(y, z) = (±27, −2) into 2.8 mm printed pilots, nominal 6.6 mm engagement. The
quarter-inch tripod screw goes through the 8 mm platform. Physical casting
fits, actual tripod thread depth, USB cable clearance and clamp strength
remain untested. The obsolete nose wedge/screws are removed and the original
front deck is restored; only the camera tube gains mounting holes.
The 68 mm-wide saddle puts the clamp screws outside the 40 mm-wide camera
support tongue; both pan heads and a Ø5 driver envelope clear the new parts.
Fasten the tripod screw on the bench, then assemble the clamp and camera bar
before installing the head; the lidar limits access to the underside in place.

`V10H_D435_FOV_reference.SLDASM` has independently named, coloured reference
volumes for **IR left (cyan), IR right (purple), RGB (orange)** and the
**IR projector (yellow; illumination, not an imager)**. They are hidden as a
group in the main and floor-grasp assemblies; show the group in the tree or
open `V10H_robot_FOV.SLDASM`, where they are visible. They represent 700 mm
of optical-forward distance and are not physical/printable robot parts.

Sources: [D400 datasheet rev 017, tables 3-13/3-17/3-49 and section 4.4](https://realsenseai.com/wp-content/uploads/2025/08/Intel-RealSense-D400-Series-Datasheet-August-2025.pdf)
and the saved calibration for unit 827312072573 (Oct 29), copied to
`reference/d435_saved_calibration.json`. Pupil positions use the 17.5 mm
left-imager offset and 4.2 mm glass offset; the 49.85 mm IR baseline and RGB
six-degree transform come from the saved calibration. Nominal full IR sensor
fields are 90.1°×64.1°, not the smaller 87°×58° stereo-depth overlap. RGB is
69°×42°, projector 90°×63°. The saved modes have asymmetric principal points;
clearance uses larger guard fields (IR 94°×68°, RGB 73°×46°, projector 94°×67°)
and 1 mm plane expansion. The displayed volumes are nominal field models,
not a new physical calibration or a guarantee of usable depth at every range.

Validation clips complete CAD-mesh triangles against each six-plane convex
frustum, rather than just checking four corner rays. Mesh chord tolerance is
0.15 mm. The fixed robot and mount clear every guard field. All four original/
opposite-hand pods clear conservative full-stroke hinge envelopes, and full
360° wheel cylinders clear at axle heights −3…+15 mm. The camera and saddle
have no physical clashes in high, drop and floor-grasp x270/310/340 poses.
Both existing side cameras' nominal enlarged stereo fields also clear the
new D435, bracket and hardware (the actual module lenses remain unconfirmed).
High and drop have no arm optical obstruction. **During floor grasps the arm
and hand enter the views**; this is recorded in `arm_pose_checks`, not hidden
behind an overall "unobstructed in every pose" claim.

Reproduction: `52_d435_fov_study.py search`, `53_d435_bar_mount.py build`,
`53_d435_bar_mount.py verify` (or `pods`/`wheel_spin` for those independent
envelopes), then `54_d435_bar_solidworks.py stage|commit`. Results are
`validation/d435_fov_study.json`, `d435_bar_mount_geometry.json`, and
`d435_bar_mount_solidworks.json`. Printed STEP: `step_export/d435_bar/parts/`.

**CAD-only change:** simulation/ROS still have the previous nose-camera pose.
`reference/d435_bar_camera_frames.json` supplies the new body and imager poses
for their owners to update. The root design document is deliberately untouched
under Fernando's CAD-only instruction.

```
V10H_robot.SLDASM                      Base fixed at the origin; everything else locked to it, except the arm's joints
├─ V10H_base                           deck / bumper / electronics / fasteners  (STEP parts, fixed in place)
├─ V10H_mast, V10H_head, V10H_arm_kit  each = <name>_parts + <name>_fasteners
├─ V10H_d435_bar                      actual D435, centre-bar saddle, mounting hardware
├─ V10H_D435_FOV_reference            four optical reference volumes (normally hidden)
├─ V10H_arm_link0_base … link6_jaw     the SO-101, ONE SUB-ASSEMBLY PER LINK, all at the top level
├─ V10H_wheel_pod  (FR 315°, RL 135°)  the native SolidWorks pod
└─ mirror/v10h_wheel_pod-mirror (FL 45°, RR 225°)   its mirror image
```

## The pod (`V10H_wheel_pod.SLDASM`, was one flat file of 123 parts and 222 mates)
```
V10H_wheel_pod        chassis (fixed) · fork (hinge, travel limit 87.4–101.9°) · wheel · motor · 2 shocks · 13 pins/screws
├─ V10H_pod_chassis   guard, pod mount, pivot shaft + spacers, shock-pin screw
├─ V10H_pod_fork      inner / outer fork, motor cup, cross bar, 2 × 627, fork screws
├─ V10H_pod_wheel     rigid hub assembly + 12 rollers
│   ├─ V10H_pod_hub   hubs A/B/C, cartridge, 2 × 6804, joint / cartridge / axle screws, nuts
│   └─ V10H_pod_roller ×12   core + tread
├─ V10H_dummy_motor   (unchanged file, in the folder root)
└─ V10H_pod_shock ×2  case + rod, sliding
```
* **Roller axial play removed:** each roller is concentric with its axle bolt and held at a plain 23.5 mm distance (it still spins).
* The bench plate and its 11 foot screws are not in the pod: the robot's deck and its screws (the v9 base fasteners, which already carry the pod foot bolts) do that job.
* Hardware that joins two sub-assemblies (cup ↔ stator screws, flange screws, shock pins) is mated at pod level.
* Checked in SolidWorks (`tools/human/test_pod_motion.py`, `validation/pod_subassembly_motion.json`): axle −3…+15 mm with both shocks 40–50 mm; the wheel turns with its rotor and flange screws while fork and stator stay; a roller turns on its axle.
* The old flat file is kept in `superseded/flat_wheel_pod/`.

## The mirrored pod (`assemblies/mirror/`)
SolidWorks' Mirror Components (`44_mirror_pod.py`, ImportOptions 1) wrote an opposite-hand copy of every part and sub-assembly (derived from the originals). It keeps only a few mates, so `46_mirror_remate.py` restores them: hub / chassis / fork are rigid bodies (all components fixed), rollers, shocks, motor and the pod top get their mates back (39 at the top, same as the right-hand pod). **Weaker than the original:** the mirrored sub-assemblies are rigid groups, not fully mated. Re-run 44 then 46 after the pod changes.

## The arm (SO-101, links = sub-assemblies at the top level)
Each link file holds its servo(s) and printed parts (TheRobotStudio STEP). Link 0 is locked to the arm kit's adapter. Every other link joins its parent with a **concentric** mate (servo horn / bore), a **seat** mate (faces ⟂ axis, at the real gap) and an **angle limit mate** between two faces parallel to the axis. Limits = the servo range clipped to the range where the links do not collide (`45_arm_limits_mesh.py`, 1 mm voxel overlap of the official meshes, joint swept alone from the delivered pose; `reference/so101_joint_limits.json`):

| joint | servo range | delivered pose | limit mate |
|---|---|---|---|
| shoulder_pan | ±110° | 0° | ±110° |
| shoulder_lift | ±100° | −12° | ±100° |
| elbow_flex | ±96.8° | −72.7° | −96.8…+89.3° (links hit above) |
| wrist_flex | ±95° | −12° | ±95° |
| wrist_roll | −157…163° | 0° | −157…163° |
| gripper | −10…100° | −10° | −10…100° |

* **Delivered pose = `ARM_POSES["high"]`, not the drawn "stow".** At stow the shoulder bracket and the gripper overlap (2.4 cm³ by the voxel check, 4.4 cm³ by SolidWorks' own), so single-joint limits from a colliding pose are meaningless. Another pose is one angle-mate edit away.
* Limits are per joint with the others where they are; 89 % of random poses inside the whole limit box are collision-free (11 % are folded extremes, mostly base/shoulder ↔ gripper/wrist).
* Driven end to end by `tools/human/test_arm_motion.py` (`validation/arm_motion.json`: each joint turns exactly the commanded angle, nothing else moves, no mate errors).

## Where things are
`parts/step/` imported STEP parts (`step_parts/` the STEP sources, `reference/robot_v9_instances.json` the v9 placement table, `reference/so101_links.json` link frames). Builders: `tools/human/20_pod_subassemblies.py` (pod), `30/31/32` (STEP split, arm frames, STEP → part), `40_robot_subassemblies.py`, `41_robot.py body|arm|arm_limits|pods`, `44/46` (mirror). SolidWorks degrades over long scripted sessions and eventually crashes or refuses files: run one stage per session (`subasm.restart()`).

## Known gaps
* v9's catch module (tilting funnel) is not in the robot: this is the arm robot only.
* Not checked: interference between the new v10 pods and the v9 deck; v9 has two fewer foot bolts than the v10 chassis holes (Guard_foot_bolt_4 of two pods).
* Parts from v9 are imported STEP (not editable features), as asked.
