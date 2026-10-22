# CAD v7: the home-floor robot, down to the fasteners (build123d → STEP / STL)

The robot design.md describes today: the **v7 base** (X4, printed 127 mm omni wheels with 2 × 5 rollers of 36 mm,
sprung pods; section 25, adopted) carrying the **v6 round-3 catch / launch module** (chute pocket, two Pololu 25D feed
wheels, J-bend, two F90 flywheels, STS3250 tilt; sections 22-24), on section 24's **Damiao DM-2325** drive and a
7S1P pack. Every dimension the sim knows comes from it: `dims.py` builds `SimParams` + `geometry()` from
`simulation/v7 home floor`, so the CAD and the sim can't drift. The v2 CAD in `cad/` is untouched.

![overview](out/renders/overview.png)

## Build and view

```powershell
$py = "$env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe"
& $py build.py            # ~6 min: STEP + STL of every printed part, assembly.step, viewer meshes, all checks, BOM, print list
& $py build.py --quick    # parts, BOM, mass, thread engagement, static clash check only (~1.5 min)
& $py view.py             # interactive 3D viewer (MuJoCo); --meshes rebuilds its meshes first
& $py step_export.py      # out/assembly.step again; --bodies, --groups, --no-fasteners, --tilt, --bump (see below)
& $py render.py           # out/renders/*.png (MuJoCo), --turntable for a video
```

`Launcher.cmd` (repo root) has buttons for all of them: Tools > Build CAD, View CAD 3D, Open CAD STEP, STEP as one part.

**Viewer** (`view.py`): left-drag rotates, right-drag pans, scroll zooms, double-click centres on a part. Keys 0-5
toggle groups: 0 base, 1 pods, 2 module, 3 yokes and cameras, 4 fasteners, 5 foam pads, net and ball. The right
panel's Control slider `tilt` turns the module about its pivot (−0.52 … 0.70 rad = −30 … +40°).

`build.py` checks every pair of placed solids (printed parts, purchased parts, every screw, nut, pin and bearing)
for interference at the static pose, at module tilts −30 … +40° and with all four wheels at their bump stop and full
droop (tyres pressed on hubs, rollers' treads on their cores are intended fits). It checks that **every screw engages
≥ 6 mm of printed thread** (or a nut / a metal thread ≥ 2.4 mm), from a probe tube around each shank. It also measures
the lidar's self-occlusion and the mass and CG against the sim (`out/checks.json`).

## Rules this CAD follows (Fernando, Oct 21)

- Printed in PETG / PLA / TPU 95A, every part ≤ 250 mm (a 256 mm bed).
- **No heat-set inserts.** M3 screws form their own threads in **2.8 mm** printed holes with ≥ 6 mm of engagement
  (checked for every screw); through holes are 3.4 mm. Nuts only on pivots and bolted joints (pivot bolts, shock
  top, the pivot shaft's clamp).
- **Pan head and flat head screws only** (Fernando's stock; no socket heads): pan heads (ISO 7045 / 14583) in 6.4 ×
  2.2 mm counterbores or on faces, flat heads (ISO 7046 / 14584) where a face must stay flush (payload plate top,
  motor cup face).
- M3 everywhere it can be. The exceptions are the vendors' own threads: M2 (camera boards), M2.5 (the A2M12's base),
  M5 (the F90's prop nut).

## Files

| File | What it is |
| --- | --- |
| `dims.py` | Every dimension, the frame contracts (ROBOT, POD, MODULE, PAYLOAD), the hole conventions |
| `hw.py` | Pan / flat head screws (ISO 7045 / 7046), nuts (ISO 4032 / 10511), bearings, pins, hole cutters, engagement probes, BOM bookkeeping |
| `vendor.py`, `vendor/` | Damiao DM-2325 (Damiao's STEP + drawing), Pololu 25D 9.7:1 + encoder, #1999 and #1997 hubs (Pololu STEPs + drawings); envelopes for the rest (VERIFY) |
| `wheel.py` | The omni wheel: barrel rollers (PETG core, TPU tread, 3 mm pin), three-piece hub around the in-wheel motor |
| `pod.py` | Motor cup, trailing arm, pod mount (clevis + shock tower), open-top wheel guard (front panel, side walls) |
| `base.py` | Deck (centre + 4 lobes, T-slots, printed posts and bosses), splices, six standoffs, payload plate, battery tray, lidar riser, e-stop riser, bumpers, electronics layout |
| `module.py` | Chute halves (pocket, chute, J, launch run, keel), collar with pivot arms, rim arcs, struts, feed carriages, flywheel mounts, wheels, foam pads |
| `mounts.py` | Yokes (608 bearing / STS3250 window), servo cap, stereo bar, camera arms and wedges, boards on the yokes |
| `assembly.py` | Places everything at a tilt and a bump; the single source for the print list and the BOM |
| `build.py`, `render.py` | Exports, checks, lists; renders |
| `step_export.py`, `colors.py` | Named, coloured STEP with shared parts (assembly or one multi-body part); the colour table |
| `view.py` | Interactive viewer with a tilt slider |
| `out/step/`, `out/stl/` | One STEP per printed part in its contract frame (SolidWorks wrapper parts); print-ready STLs; foam-pad templates |
| `out/assembly.step` | The whole robot, every screw included: a nested, coloured assembly with shared parts |
| `out/PRINT_LIST.md`, `out/BOM.md` | Generated: material / infill / orientation / mass per print; every purchased part and fastener with counts |
| `out/funnel_net_pattern.svg` | 1:1 sewing pattern for the 55° net |

## Design in one page

**Wheel and drive.** The DM-2325 sits *inside* the wheel. Its body (Ø28 × 60.6) is held by its face (4 × M3 flat
heads on Ø22) in a printed cup; its 17 mm D-shaft ends at the wheel's outer face, where a Pololu #1999 hub drives the wheel's
outer plate. The wheel's inner end runs on a 61808 bearing on the arm's boss, so the motor's shaft carries only about
half the wheel load. The cup's bore has a channel for the motor's leads, from its mid-body connector to the inner end.
The wheel is three PETG hub prints split on the two row planes; each roller's 3 mm pin lies in a split plane, clamped
between half-channels. Rollers have the exact barrel profile (the wheel's envelope is a true Ø127 circle) and coned
ends. 10 × M3 × 20 join the hub.

**Pods.** A 48 mm trailing arm (pivot axis along the axle, two 623s on an M3 × 35) carries the cup, motor and wheel. A
1/16-scale RC shock runs from the arm to the pod mount's tower: motion ratio 0.50, static length 48.3 mm, 16.8 mm of
bump and 3.4 mm of droop in its stroke (the sim: 15 mm, ~2.6 mm sag). The pod mount (outer ear, bridge, web) stands
on the deck. The left pods (and the right) are mirror images, so both mounts on each side share that side's lobe, and
the front and rear lobes stay free. The mirrored pods use mirrored prints of the arm, cup, mount and guard (`_M`); the
wheel is the same print, turned half a roller pitch (its mirror image), and every purchased part is the same part.

**Deck and base.** The deck is in the axle plane: a 140 mm centre and four lobes (≤ 250 mm prints) with T-shaped
wheel slots, 141 mm wide at the wheel and 72 mm inboard of it. An open-top wheel guard closes each slot's outer end
with a curved front panel (to robot z 100) and two side walls on the lobes; the wheel's top is uncovered. TPU bumpers
wrap the deck and the guards at r 205-217 with a 1.5 mm skirt down to 20 mm. The battery (flat 7S1P) hangs under the
centre in a tray. The Jetson stands on 16 mm posts printed with the centre deck, over the power board and BMS on
printed bosses (the battery is right under the centre deck, so nothing screws up from below there). The e-stop is on
the rear lobe, and the MCU board hangs under it.

**Lidar layer.** The payload plate rides on **six printed standoffs** (Ø12, Ø20 flanges) 96 mm over the deck, so the
A2M12's scan plane (z 150, on a riser at x 140) passes over the wheels at their bump stop (142) and under the plate
(162.5): **only the six standoffs cross it.** The side pair stands under the yokes; the front pair sits on the lidar's
rays through the side pair, so their shadows overlap; the rear pair is far away (thin shadows). Everything else on
the deck stays under z 128. The module and its pivot rose with the plate (`dims.PLATE_GAP`, the sim's `plate_gap`).

**Module.** The ball path is two PETG halves split on the centreline (outer wall down, smooth path), lined with 6 mm
low-resilience foam. The chute floor slopes at 40° into the feed wheels' nip; the slick J (40 mm) turns the ball up the
30° run into the Ø70 flywheels. Each Pololu 25D sits on a hinged carriage: a torsion spring holds it on an M3 stop
screw, set so the wheels just touch a seated ball. Each F90 sits on a solid wedge (y 48-87, the motor's full width)
that fills the space behind and under its plate, back to the side wall and keel (4 × M3 × 16) and down to the line the
payload plate leaves at +40° (the plate's front edge is the module's lowest point there, so it can't get thicker
downward). The tilt pivot is 13 mm above the throat, inside the cone. So the
collar's two arms run under the net to bosses outside it: an 8 mm shaft in a 608 on yoke L, and the STS3250's horn on
yoke R. The ESC, the RoboClaw and both bucks are on the yokes' outer faces, outside the swept volume. The stereo pair
is on a 6061 bar on two arms bolted to the payload plate's top, cantilevered forward at |y| 89-103 (beside the
flywheels' sweep, above the lidar).

**Payload interface.** Still a Ø240 plate with a 25 mm grid, two asymmetric Ø4 dowels and a Ø28 pass-through. Now
M3 on 2.8 mm holes (no inserts), a full disc on six standoffs, flat heads flush with its top.

## What the CAD found (for the next simulation round)

See design.md section 29 for the full list, with what each one changes in the sim.

## Before printing: VERIFY

Search the code for `VERIFY`. The ones that set geometry: the A2M12's hole circle and scan height, the Jetson
carrier's holes, the F90's 19 × 19 pattern, the STS3250's case and horn, the RoboClaw's holes, the shock's length,
stroke and spring, the OV9782 board holes. Print one wheel (3 hubs, 10 rollers) and one pod first.

## SolidWorks / eDrawings

Open `out/assembly.step` (File > Open, STEP; import as an assembly). It is a nested assembly: robot > base, pod FL /
FR / RL / RR chassis and wheel, module (tilting), mounts > each group's fasteners in their own sub-assembly. Parts are
coloured and shared: every M3 × 10 pan head is one part with many instances, so SolidWorks makes ~135 part files
rather than 640 and its BOM counts them. Hide all of a group's screws, or a whole pod, with one click. eDrawings opens
the same file directly, for a quick look.

`step_export.py` writes other cuts of the same model:

```powershell
& $py step_export.py --bodies                       # out/assembly_bodies.step: one part, every solid a coloured body
& $py step_export.py --groups module,mounts --tilt 40 --out out/module_40.step
& $py step_export.py --no-fasteners --bump 15       # wheels at the bump stop, no screws
```

These are references at one tilt and bump, not mated assemblies: the model is the Python, and SolidWorks edits
don't flow back. `out/step/*.step` are the printed parts in their contract frames (POD parts in the pod frame,
module parts in the MODULE frame, base parts in the ROBOT frame), ready for wrapper parts; `_M` files are the mirrored
pod parts (front-left, rear-right).
