# v10 human-editable wheel pod — work in progress

**Nov 1: the pod is now sub-assemblies and the whole arm robot is assembled from sub-assemblies: see [ASSEMBLIES.md](ASSEMBLIES.md)** (`V10H_robot.SLDASM`).

**Nov 1 height update:** the active robot now has the lowered arm (nominal frame
z224.5, riser top 212), camera bar +5 mm over v7, and the retained rear mast
+6.5 mm over v7. Cheek clamps stay on the payload plate. See the reconciliation
section in [ASSEMBLIES.md](ASSEMBLIES.md), including the adapter's new bar-clearance
recess and the remaining face-layout difference from the simulation.

**Nov 2 optical correction:** the actual RealSense D435 is now on the centre
of the camera bar, centred at (198.991, 0, 183.974) mm and tilted 40° down.
The old nose mount blocked its views. `V10H_robot_FOV.SLDASM` shows independent
left IR, right IR, RGB and projector fields; the main/floor-grasp assemblies
have the same reference group hidden. See [ASSEMBLIES.md](ASSEMBLIES.md) for
checks and the unavoidable arm intrusion during grasps. Simulation/ROS camera
extrinsics have not been changed (CAD-only scope).

Native SolidWorks reconstruction requested October 31, 2026. **Not yet ready
to print or use as a completed assembly.** See `validation/` for completed
per-part checks; `INCOMPLETE_*` files are recovery checkpoints only.

**November 1 rebuild:** the contract is now [cad/solidworks.md](../solidworks.md):
named global controls, readable design features, genuine arcs, and native
patterns/mirrors. The earlier files in `parts/` and `V10_wheel_pod.SLDASM`
are the superseded CSG reconstruction. Replacement models are being built
one at a time in `parts/human/`, with audits in `validation/human/`.
`V10_hubA_human.SLDPRT` is Fernando's modeling example and is preserved.

The source wheel is the latest `cad/v9/out/wheel_assembly_6804.step`: three
hubs, removable bearing cartridge, twelve PLA roller cores and separate TPU
treads. Fernando explicitly selected adapting the suspension supports to
this wheel, rather than reproducing the incompatible older full-pod export.

The v10 motor is a printable, free-turning dummy with AK40-10 mounting
interfaces: two printed parts (stator and rotor) held by one 687 bearing and
one M3 screw with a large washer (`DUMMY_MOTOR.md`). It has no electrical
drive. The earlier four-part version is in `superseded/dummy_motor_4part/`. The wheel uses the owned 6804 bearings; the pivot
changes from 608/8 mm to owned 627/7 mm. Dimensions and print fits are nominal.

Open the replacement native parts from `parts/human/`. The mated pod is
`V10H_wheel_pod.SLDASM` (222 mates, no mate errors; the dummy motor alone is
`V10H_dummy_motor.SLDASM`). In it the stator is locked to the inner fork and the
rotor to the wheel, so spinning the wheel turns the rotor; the suspension is a
hinge on the pivot shaft with an angle limit, and the shocks slide. Checked
(`tools/human/test_assembly_motion.py`, `check_pod_motor.py`): axle travel -3 to
+15 mm with both shocks between 40 and 50 mm, wheel and roller spin, and no
interference of the motor parts through the travel and wheel spin. The shocks
have no spring or damping. To move it in SolidWorks, drag the wheel (spins) or the fork
(travels), or add a temporary angle mate as those scripts do. All final parts must have native editable
features, fully constrained dimensioned sketches and valid solids. Assembly
motion, interference, fasteners and the printable motor are not certified
complete merely because an individual part file exists.

Current replacement checkpoint: 39 native part types, 169 fully constrained
editable sketches and 276 native design features. `V10H_master_layout.SLDPRT`
adds the shared coordinate patterns and named controls. The replacement
geometry has passed the suspension-position clash checks, dummy rotor spin,
roller end-float clearance (minimum 1.177 mm), and all 63 screw engagement and
access checks. Master-reference edit tests and the new assembly are still
being completed. Per-part native audits are in `validation/human/`.

The automation workflow, recovery notes and verified API pitfalls are in
[solidworks.md](../solidworks.md). v9 files and the robot's simulation baseline
are preserved. Construction and validation tools live in `tools/`.

## Nov 1 status and open items

- **Hub joint screws moved out (r 38 to 52 mm).** `joint_radius` in the master layout is now 52. The twelve
  M3 x 18 screws that join hubs A/B/C sit beside the roller-axle clamp instead of in the thin web between the roller
  pockets; the walls around the hole went from about 1.5 mm to 3.4 mm. The previous state is in
  `superseded/joint_r38/`. The spoke web between the roller pockets is still only about 2.5 mm wide at r 41 to 43
  (set by the pocket clearance), so it remains the weakest section.
- **Outer fork clearances (fixed).** A ring groove in the cap's inner face (Ø40 to Ø60, 3 mm deep) clears the three
  cartridge screw heads, which turn with the wheel, and a slot in the cap's outer face (12.6 mm wide, 20 mm long from the
  shock eye) clears the 11 mm shock case. The whole-pod interference check (check_pod_all.py: three suspension heights,
  two wheel angles) finds nothing.
- **Not ready to print yet.** Open: the spoke web is still about 2.25 mm wide at r 41 to 43, set by how close adjacent
  roller ends come (shortening the rollers is the lever; the pocket clearance is not); the wall scan flags sub-0.3 mm slivers
  at the spoke noses where the axle access tunnels cut the tapered tip. Nothing has been test-printed.
