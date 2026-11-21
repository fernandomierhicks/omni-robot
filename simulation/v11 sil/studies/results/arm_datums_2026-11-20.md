# Touch blocks for zeroing the arm: findings and a CAD plan (Nov 20, 2026)

Status: **proposal, simulation only.** Nothing is drawn in CAD and no servo has been measured. The physics behind
these numbers is estimated (`arm_detailed.py`, `servo_emulator.py`).

## In one paragraph

Fixed blocks on the robot that the arm's fingertip touches can zero the arm without a person. In the detailed sim
they reliably fix one thing: **the pan joint's zero**, to 0.1–0.5°. Fingertip heights read with a ruler (the arm load
calibration's method) can't see pan at all. Fitting the *whole* arm's zeros from the blocks made reach worse on 2 of
3 sim arms, so the version worth building takes **only pan's zero from the blocks** and keeps the ruler for the rest.
That version was never worse, and on the arm whose pan was 2° off it cut the fingertip error from 6.2 to 1.8 mm. One
or two small removable blocks with a vertical face are enough.

## What was tested

- **The arm:** each seed is a different detailed SO-101, the same three arms as the arm load calibration study
  (`arm_load_cal.md`). The hand calibration (LeRobot's) leaves each joint 0.1–2° off.
- **The blocks:** two 20 × 20 × 70 mm blocks on the payload plate beside the arm's adapter, at x 150 mm,
  y ±130 mm in the sim's coarse layout. Each is placed 0.3 mm rms off its drawing (print and assembly); the robot
  knows only the drawing.
- **The touch-off** (`ros/src/omni_calib/omni_calib/arm_datum.py`):
  - The gripper opens so only the fixed finger's 4 mm tip sphere can touch.
  - Each touch is a slow straight move into one face (3 mm/s) at 30 % torque.
  - A touch is the last moment before the joints start falling behind the commanded goal.
  - 22 touches are planned per run, on the tops and the vertical faces; 13–17 landed. A run takes about 4 min.
- **The fits:**
  - **none:** no zeros at all;
  - **ruler:** the arm load calibration's own;
  - **blocks:** zeros and sag from the touches;
  - **both:** the touches plus the ruler's heights;
  - **pan:** the ruler's model with only pan's zero from the touches.
- **The score:** 12 fingers-down points 2 cm over the floor. Each is the tool's true position against where the robot
  sent it, with the hold trim on.

## Results

| Reach error, mm rms | none | ruler | blocks | both | **pan from the blocks** |
| --- | --- | --- | --- | --- | --- |
| Seed 0 | 5.3 | 4.95 | 7.8 | 6.0 | **4.5** |
| Seed 1 | 7.6 | 6.2 | 3.7 | 2.3 | **1.8** |
| Seed 2 | 4.0 | 4.3 | 15.5 | 11.7 | **4.2** |

| Pan zero error | ruler | blocks |
| --- | --- | --- |
| Seed 0 | 0.47° | 0.08° |
| Seed 1 | 2.04° | 0.43° |
| Seed 2 | 0.41° | 0.18° |

Full tables, with every joint's zero error: `arm_datums.md`. The study is `studies/arm_datums.py`.

## Why only pan

- **A five-joint arm can't tilt its fingers sideways.** The finger direction always lies in the vertical plane through
  the pan axis. So most of a block's faces can only be reached by moving the fingertip, pointing down, sideways into
  them. That works for pan: a sideways move across the arm's radius is a pan rotation.
- **Close to the arm's base, the other joints are poorly seen.** The fingertip reaches the blocks only in a few
  similar folded poses. Wrist flex and roll trade against each other, and gravity barely loads the joints there, so
  sag can't be measured. Their fitted zeros came out 1.5–2.7° wrong, and that wrecked the far-reach accuracy.
- **About a quarter of the touches missed or landed off.** Residuals reached ~1 mm on side faces, against 0.3 mm of
  block error.
- **Roll's zero is seen by neither method.** If roll ever matters for grasps, the tool is the D435 tag-cube
  calibration with the cube keyed to the gripper (omni_calib item 9).

## What to build

**A pan block:** one block, or one each side for symmetry. Its job is a **flat vertical face that faces across the
arm's radius**, so the fingertip, pointing down, can move sideways into it about 5 mm below the top edge. A top face is
optional; the runs show it adds little.

**What the software would do with it:**
- check pan's zero at boot, or after a reflex or a knock;
- warn if it moved more than about 0.5°;
- apply only pan's zero.

**Removable:** two screws hold it. When the robot's config lists no blocks, the software skips the touch-off.

## CAD plan for a future session

Read `cad/AGENTS.md`, `cad/solidworks.md` (section 18, printed-part rules) and the `cad/v16` README first. v16 is the
baseline CAD. **Ask Fernando where a proposal part goes:** in v16 as an optional part, or elsewhere. v16 is meant to
hold the baseline only, and v15 keeps history but is not developed in.

### 1. Find a place that works in v16's real geometry

The sim's coarse layout, which these runs used, is not v16's. Its camera pods and cheeks don't exist on the real
robot, and the arm mount is reconciled differently (design.md 4.4, the CAD/sim gap). So the position has to be found
again.

**Candidates:** the arm riser's sides (`tools/upper.py`, `parts['riser_arm']`, built from v10's `REC_arm_riser`) or
floor 2 next to the riser. Keep it **on the riser if possible**: it is the part the arm's base sits on, so the
tolerance chain from the face to the arm is shortest.

**What must keep clear of the block:**
- the D435 and its depth and RGB fields (`tools/d435_riser.py check`);
- the carrier board on floor 2 beside the riser (x 26–128, y −122 to −60 mm) and its plugs and leads;
- the harness (`tools/harness_route.py`);
- the bumper and the deck rim;
- **every planned arm move and the stow pose** (`tools/arm_sweep.py`; the arm's stow collision is already an open
  item).

**The reach check:** run `omni_calib.arm_datum.plan()` against the drawn block, with the robot's kinematics
(`omni_arm` `robot_arm_d435.json`), and keep the spots where both side touches plan.
- Fingers down, the fingertip sphere's centre 5 mm under the top edge.
- A 15 mm standoff before the face, 6 mm past it.
- The arm must reach these spots with the gripper open (~0.6 rad).

The arm's fingertip reach over the robot is narrow: in the sim it was roughly x 140–220 mm, |y| 100–180 mm, z 190–
300 mm. The arm reach sweep is a long check: ask Fernando before running it (his standing rule).

### 2. The part

- **Body:** about 20 × 20 mm in plan, tall enough that the touch point is reachable. In the sim the top was 70 mm over
  the plate.
- **The datum face:** flat and vertical (aim for within 0.2°), facing across the arm's radius, at least 10 mm wide and
  15 mm tall below the top edge. A small chamfer on the top edge, so the fingertip can't catch on it.
- **Print orientation:** print so the datum face is a side wall, the most accurate surface of a print. Avoid
  supports on it. PETG or PLA.
- **Locating:** placement must repeat after removal; that is the point of the part. No dowels (Fernando's rule). Use a
  printed step or tongue that registers against two edges of the riser, plus two M3 screws.
- **Fasteners:**
  - M3 thread-forming into 2.8 mm pilots in the riser, at least 6 mm engagement; 3.4 mm through holes in the block.
  - Hex-drive pan heads 6.5 × 2.5 mm, holes through (no blind holes), no captive nuts, pilots along the print axis.
  - Fasteners drawn magenta (`cad/solidworks.md` 18; memory rules).
- **Build conventions:**
  - Dimensions go in `tools/dims.py`, as a new section.
  - A new generator module (for example `tools/arm_datum.py`), or a part in `upper.py` next to the riser.
  - Its label, and its colour from `tools/palette.py` (orange, like the riser).
  - Add it to `tools/world.py` and its STEP to `step_export/`.
- **Checks:**
  - the build's clash, wall, half-hole and pilot checks;
  - the bed fit (trivial here);
  - extend `quick.sh` for the upper group rather than running `regen_all.sh` (Fernando: minor changes must not cost
    30 min).
- **For the software:** the drawn datum face as a plane (a point and its outward normal) in base_link, in mm. Report
  it in the README.

### 3. Hand back to the software (separate from the CAD)

1. **Export:** the face planes go into the robot's arm config, as a `datums` list in `robot_arm*.json` through
   `simulation/v11 sil/robot_export.py`. Mirror the block in the sim (`params.arm_datums`, `arm_so101.DATUMS`) and rerun
   `studies/arm_datums.py` on the real position.
2. **Planner exception:** the planner keeps the arm away from the robot's own volumes, so the block needs an exception
   that lets the fingertip approach its face.
3. **The procedure:** a `datum_check` request in arm_brain, running `TouchOff` with only the side touches, at 30 %
   torque. Fit with `arm_datum.fit(..., free=(0,))`, then store, compare with the last run, and apply.
4. **Console:** a Calibrate card ("Check the arm's pan zero"), and optionally a run at boot.
5. **Tests:** `ros/src/omni_calib/test/test_arm_datum.py` already checks that the touches recover pan's zero; extend
   it for the config entry.

## To verify on the real arm

- Whether the real fixed finger's side touches the face before its tip, and the real tip's shape. The sim uses a 4 mm
  sphere.
- Whether the touch can be seen on real servos: the joints falling behind the goal by about 0.2°, with 0.2° of
  gearbox play and the servo's dead zone.
- How far off LeRobot's hand calibration really leaves pan. The sim assumes about 1° rms.
- How flat and how repeatable a printed face is after the block is removed and refitted.

## Files

- `ros/src/omni_calib/omni_calib/arm_datum.py`: the plan, the touch-off and the fit.
- `ros/src/omni_calib/test/test_arm_datum.py`: its tests.
- `simulation/v11 sil/arm_so101.py` (`DATUMS`, `datum_truth`, `datums_xml`) and `params.py` (`arm_datums`,
  `arm_datum_err`): the blocks in the sim, opt-in.
- `simulation/v11 sil/studies/arm_datums.py`: the study. Results in `arm_datums.md`, `arm_datums.json`, and the
  fitted models in `arm_datums_seed<N>_<fit>.json`.
- Background: design.md 4.5; `arm_load_cal.md` (the arm load calibration these build on).
