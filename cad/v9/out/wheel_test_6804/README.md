# v9: one 140 mm wheel test

Print the STLs here at **100% scale**; they are already oriented on the bed.
Start with **one roller_core (PLA) and one roller_tread (your medium TPU)**.
Dry the TPU according to its maker. Use the material maker's temperatures and
speeds; the printer and exact Shore hardness have not been specified.

## Fit first

Stretch the sleeve onto the barrel, lining up its three internal keys with the
core's grooves. The barrel retains it axially; the keys resist twisting. It must
stay seated when pulled and rolled firmly by hand. It must not split, wrinkle or
turn independently of the core. Do not glue it for this first fit check.
If the fit is wrong, measure it and adjust the mating geometry before printing
the other eleven. Optionally print fit_bearing_32mm.stl and
fit_bearing_20mm_journal.stl in PLA to check the 6804 outer-race seat and
inner-race journal first. Adjust the fit allowance before committing to hubs;
do not force a bearing into a tight PLA part or preload the two races.

## Complete one wheel

| Print | Count | Material | Suggested starting settings |
| --- | --- | --- | --- |
| wheel_cartridge | 1 | PLA | 6 walls, 80% infill; flange down |
| wheel_hub_A | 1 | PLA | 6 walls, 60% infill, 0.20 mm layers; boss down |
| wheel_hub_B | 1 | PLA | 6 walls, 60% infill, 0.20 mm layers; split face down |
| wheel_hub_C | 1 | PLA | 6 walls, 60% infill, 0.20 mm layers; split face down |
| roller_core | 12 | PLA | 4 walls, 20% infill, 0.20 mm layers; axis vertical |
| roller_tread | 12 | medium TPU | 100% infill, 0.20 mm layers; axis vertical |

Settings are starting points, not tested print profiles. The TPU treads print without supports. Support the core lower cone around its 1.5 mm thrust journal and keep support off the bore and flat thrust faces. Enable removable supports for the hubs: A has
overhanging plates / spokes above its bearing boss, B has a second spoke row,
and C has an inner bearing-retaining ledge. Review every layer in the slicer;
use build-plate tree supports where accessible and keep contact off bearing
seats and axle channels. Remove all support through the hub openings before
assembly. Inspect the axle half-channels, roller-clearance edges and core
keyways; the full wall scan flags thin edge remnants there. Remove loose
feathers and clear strings / first-layer bulges. See ../wheel_checks_6804.json for the
wall-scan coordinates; this is not a zero-warning wall report. Turn off automatic orientation; use elephant-foot compensation as needed.
The supplied STEP files retain the CAD frame; use STLs for the stated orientation.

Hardware for one wheel: **12 M3 × 50 mm pan-head axle screws plus 12 plain M3 nuts**, **3 M3 × 12 screws for the removable bearing cartridge**, **12 M3 × 20
shallow pan-head hex-drive screws**, **3 M2.5 × 6 pan-head screws** into the
AK40-10 output flange, and **2 × 6804 bearings (20 × 32 × 7 mm)**.
Wheel-only prototype: the 20 mm bearing interfaces require new supports later. Do not use the previous fork / cradle as a fit reference.
No Pololu shaft hub, D-shaft adapter, inserts or glue is used.

Fit the treads and lay the rollers / M3 axle screws into the open hub-B half-channels. Fit the plain nuts while the halves are open, snugging against the end blocks without clamping the rollers. Then assemble hub A to B with one row of
six rollers, then C with the staggered second row. The nuts fit 5.8 mm across-flats hex pockets so they cannot turn. The axle screws lie in the split
planes. Thread the M3 screws gently into the 2.8 mm printed holes. All rollers
must spin freely with some end clearance. Tighten the axle against the hub end blocks, leaving the roller free. The core is 39 mm long plus two 1.5 mm journals (42 mm overall). The two thrust faces limit nominal end float to 0.6 mm total. TPU stops 2.5 mm behind the journal ends. Only the small journal thrust faces may contact the fixed PLA; the core body and TPU must stay at least 0.5 mm clear throughout travel. The finished CAD clearance check covers all twelve rollers at both stops and in the centre, using full-rotation envelopes; see ../wheel_checks_6804.json for the nominal distances. Actual printed clearance must be checked after support removal. The pocket extends past the coned ends; unused pointed spoke tips are clipped, the cheeks taper beyond the joint pads, nose edges have 1 mm bevels, and both sides of each narrow triangular cheek root have reinforcing 2 mm fillets. These add material at the neck; the 2 mm spoke/drum root fillets also remain. The end relief tapers down toward the core tip instead of staying barrel-wide: the slot underneath each triangular support is filled with continuous PLA. The roller running clearance is still checked throughout axial travel. Its 3.6 mm bore clears the threads; inspect bore and thrust wear during the bench test.
The M2.5 flange screws are reached through hub A's large centre opening; their
2.5 mm engagement leaves 0.5 mm before the maker's 3 mm blind-hole limit.
Fasten the motor flange first, then fit the removable outer bearing cartridge with three M3x12 screws. Fit the two 6804 bearings on 20 mm test supports; these are wheel-only interfaces, not the previous base geometry.

## Bench record before printing the other three

Measure actual diameter / runout, free spinning and end play, tread retention,
rolling force and sideways rolling, grip on your actual floors under about 25 N
wheel load, pin / bore wear and tread migration. Hand-push across your measured
rug edges and thresholds. Check the motor, connector bend, bearing fits and that
the two wheel bearings do not preload the motor output bearing. Record filament,
TPU hardness from the spool, print settings and measured mass. Those measurements
set the final fits and the sim's friction / rolling resistance. The first wheel
is a bench prototype; loaded driving follows the design.md phase-1 checks.

Separate PLA and TPU are the default: replaceable treads and no dependence on
inter-material adhesion. A combined print would still need the rollers to turn
on their pins; it cannot fuse a whole wheel into one moving solid. A printer
with suitable multi-material capability could print each keyed core and tread
together later, using their shared STEP coordinates, after a material trial.
