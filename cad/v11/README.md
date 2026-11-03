# cad/v11: compact wheel + motor and its suspension pod

Started Nov 2 2026. A self-contained wheel pod (wheel, AK40-10, sprung fork, two side frames) that drops into a cutout in the flat robot plate, which is designed later.
It is built in build123d, reviewed as STEP, and assembled in SolidWorks from those STEPs. The SolidWorks parts are imported bodies, not native editable features.
Everything is nominal CAD; nothing has been printed.

| What | Where |
| --- | --- |
| **SolidWorks pod (drag the fork or the wheel)** | `solidworks/V11_pod.SLDASM`; sub-assemblies `V11_chassis`, `V11_fork`, `V11_wheel` (12 x `V11_roller`), `V11_motor`; parts in `solidworks/parts/` |
| Pod STEP (nominal pose; coloured per face; springs shown; no fasteners, bearings or shaft) | `step_export/pod/pod_assembly.step`; printed parts in `step_export/pod/parts/` (2 side frames, fork, axle stub) |
| Wheel + motor STEP | `step_export/wheel_motor/wheel_motor_assembly.step`; printed parts in `step_export/wheel_motor/parts/` (hub, cartridge, roller core, roller tread) |
| Checks and views | `validation/pod/` (`checks.json`: droop / nominal / bump, spring table, plate cutout; `section_*.png`; `iso_*.png`), `validation/wheel_motor/`, `solidworks/motion_test.json` |
| Build | `tools/dims.py` (every dimension), `wheel_motor.py`, `pod.py`, `sw_parts.py` (omni-robot-sim venv, `python -B <script> all`), then `sw_build.py` (cadenv, see below) |
| Inputs | `reference/` (bolt and bearing STEPs used by the checks), `../../components/ak40-10/AK40-10 with connector.STEP` (the maker's motor with its XT30(2+2) plug; only the plug is used) |

## The pod
- **Mounting to the robot plate:** the plate is flat with its top 3 mm above the axle (73 mm above the floor), as v10's deck. The pivot is at plate height.
  The pod drops into a cutout and its two side frames stand on the plate top beside it. Each frame takes **3 vertical M3 screws from the top**: x -67.5 / +64.7, y 25, 35 and 117 in the pod frame.
  The screws go into Ø2.8 pilots in a printed plate (≥ 8 mm deep), or through the plate into nuts. The pod comes out from above without turning the robot over.
  The plate needs (assuming 6 mm thick):
  - the moving parts' cutout x -63.5..57.7, y -70..106 (`checks.json` plate; add about 2 mm all round);
  - a notch under each frame's keel at x -73.5..-61.5 and 58.7..70.7, y 64..112.

  Side holes are not used: a thin plate edge is a weak place for screws.
- **Hinge level with the axle** (design.md 2): Ø7 steel shaft 80.6 mm from the axle at axle height, through both frames' keels, pinned by one M3 cross screw (inner keel).
  The fork turns on 2 x 627. Axle travel is -3 / +15 mm (12.9 deg), as in the sim baseline.
- **Fork** (one printed part, 118 x 136 x 59 mm, 185 cm3):
  - the outer arm takes a bolt-in **axle stub** into the wheel's left 6804;
  - the inner arm is bolted straight to the **motor's rear face** (4 x M3 x 12 in counterbores; rear holes Ø2.8 x 4.3 in the maker's STEP, taken as M3, VERIFY);
  - a beam behind the wheel joins the arms, and its tail meets the frames' droop and bump ledges;
  - a flat pad on each arm sits under a spring plunger.
- **Side frames** (two printed parts, 48 and 47 cm3, each 12 mm thick with a flat outer face: print lying on it). Each has:
  - a foot on the plate;
  - a spring tower over the fork arm (top 66 mm above the plate);
  - a keel down through the plate notch carrying the shaft;
  - droop and bump ledges over and under the fork's tail.
- **Springs, no damper:** one ISO 10243 die spring per side, Ø10 hole / Ø5 rod x 38 free. Use HPC R203106 **green** (6.8 N/mm) for a 10-12.5 kg robot and R204106 **blue**
  (11.9 N/mm) for 12.5-15 kg; the same size is sold by McMaster and Misumi. Each spring sits on an **M5 x 50 button-head bolt used as a plunger**. Its dome presses on the fork pad 30 mm from the pivot.
  Its shank slides in the tower's guide hole, an M5 washer seats the spring, and an M5 nyloc on top keeps it captive.
  The springs act 0.372 x the axle travel (plunger -1.1 / +6.2 mm). Wheel rate is 1.9 N/mm (green) or 3.3 N/mm (blue); the sim wants 0.8-3.
  The geometry is set for green at 10 kg (4.8 mm preload compression). Another mass or spring needs a printed shim under the spring of `shim_mm` (`checks.json` spring table) to keep the ride height.
  - 20 kg on blue reaches 11.7 mm at full bump: past the 11.4 mm long-life limit but under its 14.4 mm maximum.
  - Torsion springs were ruled out: off-the-shelf ones reach about 0.3 N·m, against the 2-4 N·m needed.
- **Checks** (`pod.py check`, droop / nominal / bump): no interference anywhere.
  - Clearances: frames to anything rotating ≥ 7.9 mm; frames to the motor leads ≥ 40 mm; fork to anything rotating 4.0 mm; fork to the leads 3.8 mm (they move together); axle stub 1.5 mm radially inside the cartridge lip.
  - The ledges touch the tail exactly at droop and bump (0.26 mm above it at nominal: print tolerance moves droop by about ±1 mm).
  - The plunger dome stays on its pad: it slides 6.1 mm of 9.
- **Hardware per pod:**
  - pivot: 2 x 627; Ø7 x 144 steel shaft cross-drilled Ø3.4 at x +64.7, 6 mm from the inner end; M3 x 30 pan (shaft pin, from under the inner keel);
  - fork and stub: 4 x M3 x 12 pan (fork to motor), 3 x M3 x 12 pan (stub);
  - springs: 2 x die spring, 2 x M5 x 50 button head (ISO 7380), 2 x M5 washer, 2 x M5 nyloc;
  - mounting: 6 x M3 into the plate (length = foot 8 + engagement).
- **Printing:** each printed part is one solid. Frames print on their outer face. The fork is still heavy and blocky; lightening it is a later pass.

## The wheel + motor
Ø140 wheel (rollers on r 52, tread profile R70: same as the v10/v9 wheel and the sim). One printed hub (drum r 30..33, 2 x 6 spokes 16 mm thick with R3 root fillets, axle blocks), x -39.5 .. +29.
- **Rollers** drop in radially after printing; M3 x 50 axles slide in along their own axis into Ø2.9 pilots (no nuts, 6.5 mm engagement). Ø3.4 bore, 0.75 mm thrust journals, flat ends, R2 TPU end fillets,
  half length 19.5 (2.35 deg overlap), rows at ±17.25 mm. Each spoke's flat thrust face runs across the whole roller end (1.05 mm from the end face) and blends R3 into the 0.95 mm tread clearance.
- **Bearings:** the hub rides on the motor's output bearing and the left 6804. The 6804 presses into the **cartridge** from its open side; the cartridge has no floor, so there is no trapped cavity and it prints plate-down without support.
  The 6804 floats axially; the motor bearing locates the wheel.
- **AK40-10** sits outboard (output face x 4.5, rear 44.7), turned so its radially mating XT30 points back and slightly up (az ~160 deg), away from the pivot and springs.
  The plug and leads sit outside the hub, 4.7 mm from anything rotating.
- **Hub flange:** the flange plate (x 1..4.5) joins the drum by a 45 deg cone, and the left end wall runs out on a second 45 deg cone, so the hub cavity needs no internal support printed either end down.
- **Own motor model:** the maker's STEP is surfaces only (B-spline faces, no solid). `wheel_motor.build_motor` is a solid stator + rotor with the measured interfaces: can Ø53, output disk Ø37 with
  3 x M2.5 at r 13.5 and dowel holes, rear 4 x Ø2.8 at r 23.5, front 6 x Ø2.6, and a socket pocket shaped by the real plug. Only the plug and its pins come from the maker's file.

## SolidWorks
`C:/Users/ferna/cadenv/Scripts/python.exe tools/sw_build.py import roller wheel motor fork chassis pod test` (after `sw_parts.py`; a few stages per call, SolidWorks leaks memory).
It uses cad/v10's helpers (`subasm`, `mate_api`, `sw_native`) unchanged.
- **Mates:**
  - fork: Right Plane on the outer frame's, and the 627 bores concentric with the shaft (it hinges);
  - motor: stator locked to the fork, hub locked to the rotor (the rotor turns in `V11_motor`);
  - rollers: each concentric on its hub hole with a 0.3 mm thrust gap (turns);
  - plungers: each concentric with its tower guide, with its dome **tangent** to the fork pad, so it rides up and down as the fork moves;
  - `Suspension_travel`: a distance limit between the outer plunger and the outer frame's foot top, for plunger lift -1.10 .. +6.21 mm = axle -3 .. +15.
  The wheel and motor are flexible sub-assemblies. The springs are shown at their nominal length (they don't compress in SolidWorks).
- **Motion test** (`sw_build.py test`, `solidworks/motion_test.json`, passes):
  - With the limit suppressed, the fork set to droop / nominal / bump lands on axle z -3.0 / 0 / +15.0 with both plungers at -1.10 / 0 / +6.21, matching build123d to 0.01 mm, with no mate errors.
  - With the limit on, a GUI-style drag (`IDragOperator`) moves the fork and both plungers and stays inside the travel.
- An angle limit between fork and frame planes left the fork fully defined, which is why the limit sits on the plunger. API `Transform2` cannot move a part held by a limit mate; a GUI drag can.
- Component names are SolidWorks' file names (`V11_motor-1` ...); the scripts map them by file. STEP colours are written per face, in sRGB (`tools/step_colour.py`).

## Open items
- **Motor output bearing load:** it now carries part of the wheel load; its rating is unknown (VERIFY with CubeMars). Rear-hole thread (M3?) VERIFY.
- **Spoke neck about 1.9 mm** tangential x 16 mm at r 42. Measured options: journals 0.5 + gap 0.25: 2.5 mm; rollers 1 mm shorter (half length 18.5, overlap 0.65 deg): 3.7 mm; both: 4.1 mm.
- **Springs:** die-spring OD/ID modelled as 9.6 / 5.4 (VERIFY on the part); the button head slides PETG on the pad (a steel wear plate could be added); no damping (design.md: damping matters little in the sim).
- **Mirror-hand pod** (opposite diagonals, design.md 2) is not generated. It needs mirrored frames, fork and hub (the hub's flange holes) and the motor turned to put the plug at az ~20 deg.
- Lead route from the plug into the robot harness (modelled to x 56.7, inside the cutout); the robot plate itself.
- Spokes print with external supports under their faces; axle bolt heads graze the neighbouring roller's TPU tread on insertion (upper bound 9 mm3 per tunnel).

## History
- Pod round 2 (Nov 3): the RC shocks were too soft, so ISO 10243 die springs on button-head plungers replace them (torsion springs ruled out). The roof chassis is replaced by two side frames on the plate, with the pivot at plate height as v10.
  The cartridge has no floor (no trapped cavity). The plunger carries the SolidWorks travel limit.
- Round 5 / pod round 1 (Nov 3): motor mount dropped (the fork bolts to the motor), motor turned to az 160, second hub chamfer, own solid motor, face colours, shock pod, SolidWorks assemblies.
- Round 4 (Nov 2, option 1): the XT30 plug mates radially (pins r 31.3, leads r 35.9), so the motor moved 27.5 mm outboard; the hub ends at x 29; the right 6804 went. Rule since: `cad/solidworks.md` section 16.
- Round 3 (Nov 2): real plug and leads, thrust cut across the whole roller end, roller bore 3.6 -> 3.4, one script + `dims.py`.
- Round 1-2: `cad/v10 human editable/` (`step_export/wheel_motor_v2`, `tools/human/70_wheel_motor_v2.py`, ASSEMBLIES.md).
