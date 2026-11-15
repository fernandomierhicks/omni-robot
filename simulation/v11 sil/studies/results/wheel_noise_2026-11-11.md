# Wheel clicks and spoke "petals" at steps (Nov 11 2026)

Fernando built a printed v13 wheel. Rolled by hand on a hard floor, it clicks as every roller meets the floor. He also saw the plastic spoke ends that hold the roller axles (the "petals") hold the wheel back when one meets a step. Sim only: `studies/wheel_noise.py`, `studies/petal_step.py`. Proposal CAD: `cad/v13/tools/wheel_quiet.py`. Nothing is adopted.

## 1. Clicks: what makes them

The model is one wheel in its own plane, integrated at 1 µs.

- **Rollers:** each is a separate body held to its axle by contacts, not a joint. Radial play is 0.23 mm (a Ø3.4 bore on M3 thread crests). Axial play is 0.3 mm at each thrust face. Bore and thrust contacts are hard (PLA on steel, PLA on PLA).
- **Tread:** Hertzian TPU contact on a rigid floor.
- **Hub:** the hub's roll is resisted by the fork. Under drive, the AK40-10 speed loop and the pod spring carry a quarter of the robot.
- **Noise proxy:** the A-weighted 0.4–10 kHz force the rollers put into the hub and into the floor.
- **Limits:** only differences between variants mean anything. Contact stiffness and damping are estimates, and the model has no structural modes or sound radiation.

**Mechanism.** At 1 m/s the wheel spins at 14 rad/s, so centrifugal acceleration on a roller is about 1.1 g. It holds every unloaded roller against the outside of its bore. A roller landing on the floor is knocked across its whole 0.46 mm of play, at about 0.1 m/s. A roller leaving the floor is flung back the other way. When hand-rolled, gravity does the same job.

As each axle tilts going round, the roller also slides end to end across its 0.6 mm of thrust play and hits the thrust face, at up to 0.1 m/s. That happens in the air, not only at the floor. Both are hard-on-hard impacts, twice per roller per turn.

Change in dB against the v13 wheel (negative is quieter):

| Variant | Hub, 1 m/s | Floor, 1 m/s | Hub, by hand 0.3 m/s | Floor, by hand |
| --- | --- | --- | --- | --- |
| Bore 3.2 (0.13 mm play) | −0.8 | −1.2 | −0.9 | −1.2 |
| Smooth 3 mm pin, 0.05 mm play | −2.6 | −5.9 | −1.5 | −1.5 |
| 623 bearings, 0.1 mm axial play left | −1.9 | −11.1 | −0.9 | −4.5 |
| **623 bearings + axial preload** | **−10.4** | **−18.0** | **−7.7** | **−10.0** |
| Axial play 0.1 only (bore unchanged) | +0.2 | −0.2 | +0.6 | +0.9 |
| TPU 85A tread | −0.2 | −2.0 | −0.5 | −2.5 |
| 2 × 8 rollers | +0.1 | +0.6 | +0.6 | +1.2 |
| 2 × 5 rollers | −1.1 | −1.3 | −0.8 | −0.8 |
| Roller overlap 6° (v13: 2.35°) | −0.7 | −0.1 | +0.5 | +0.5 |
| Tread end fillet R0.5 (v13: R2) | −0.4 | +0.3 | +0.2 | 0.0 |
| Roller radius error 0.1 mm (1σ) | +0.4 | +0.3 | +0.4 | +0.2 |
| 623 + 85A tread | −3.1 | −11.3 | −1.2 | −3.7 |

With 8 N of steady traction, 623 bearings plus preload give −10.8 dB at the hub and −17.9 dB at the floor.

**Reading.** Play is the click. Roller count, roller overlap, end fillet, tread hardness and print run-out each move the result by about 1 dB or less. Removing the radial play quiets the floor but leaves the thrust-face slams in the hub. Only removing both radial and axial play quiets everything.

## 2. Suspension and wheel count

These runs use the same wheel at 1 m/s. "Chassis" is the 0.4–10 kHz force reaching the robot body:

- **Vertical:** through the pod spring and damper.
- **Horizontal:** through the pod's pivot. The pod swings only vertically, so horizontal forces pass straight through. The chassis takes its share of them by mass.

| Mount | Chassis, vertical (dB) | Chassis, horizontal (dB) | Hub (dB) |
| --- | --- | --- | --- |
| v13 pod (2.6 N/mm, damped ζ 0.3) | −23.0 | 13.0 | 20.6 |
| Pod springs only, no damper | −46.8 | 12.3 | 20.2 |
| Stiff rubber mount (200 N/mm) | −15.1 | 10.7 | 18.7 |
| Rigid mount | +1.3 | 11.5 | 19.1 |
| Pod, 3-wheel load per wheel | −21.0 | 13.9 | 20.9 |
| Rigid, 3-wheel load | −1.3 | 12.2 | 18.9 |

The wheel with 623 bearings but its 0.1 mm axial play left gave −25.2 / 12.3 / 18.7 on the pod and −7.9 / 7.2 / 13.5 rigid.

- **The suspension helps.** The sprung pod keeps the vertical clicks out of the chassis: 24 dB less than a rigid mount, and about 47 dB less if the pod has no viscous damper. A damper passes high-frequency force through, so friction or spring damping is quieter than a fluid damper.
- **The horizontal path is not isolated by the pod.** It carries more than the vertical path even when the wheel is rigidly mounted, so the clicks reach the body through the fork, the pivot bearings and any play there. Only a quieter wheel fixes that path.
- **Four wheels or three makes about no difference.** Four similar uncorrelated sources sum to +6.0 dB. Three wheels give +4.8 dB, plus 0.3–2 dB louder per wheel from the heavier load. The totals are within about 1 dB, so the wheel count is not a noise lever.

## 3. Petals at a step

The rig is one sprung wheel on the v13 pod rate under a quarter of the robot, pushed at 0.3 m/s with up to 30 N.

- **Geometry:** v13 rollers, each a convex hull with the R2 end fillet. v13 spokes, each the hull of the R8 axle block and its drum feet, running from the outboard face at x ±29.25 to the channel. PLA μ 0.35, TPU μ 0.9.
- **Sweep:** approach 0–90°, with either side leading at 60° and above; steps 12–24 mm; 20 contact phases; 3,600 runs.

Crossing fraction over the contact phases:

| Approach | Step (mm) | v13 petals | Chamfer 45°, r 58 | Chamfer 30°, r 52 | Petals 4 mm shorter | No petals |
| --- | --- | --- | --- | --- | --- | --- |
| 0–60° | 12–24 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| 75° | 12 | 1.00 | 0.75 | 0.75 | 0.75 | 0.80 |
| 75° | 16 | 0.90 | 0.60 | 0.60 | 0.60 | 0.65 |
| 75° | 20 | 0.75 | 0.50 | 0.50 | 0.50 | 0.50 |
| 75° | 24 | 0.65 | 0.40 | 0.40 | 0.40 | 0.45 |
| 90° (along the axle) | 12–24 | 0 | 0 | 0 | 0 | 0 |

- **The sim does not reproduce the problem.** The petals do touch the step at oblique approaches, with peak forces of 350–810 N. They never stop a crossing the rollers would make. At 75° they act as paddles and rescue phases the rollers alone lose. The chamfers remove exactly the part that does that, so they fall to the no-petal result.
- **Sensitivity:** the result holds with PLA μ 0.15–0.6 and with a 12 N push.
- **Moving along the axle,** nothing crosses 12 mm, petals or not (8 mm does). With the gap between rollers in front, the step meets the two roller end faces either side of it, about 3 mm ahead of the petal face, not the petal.
- **Not modelled:** an unpowered, hand-pushed wheel; the petals' real edge rounds; rug or rounded edges; the fork and motor.

## 5. Round 2: the 687 prototype (same day, Fernando's brief)

Round 2 supersedes section 4. `cad/v13/tools/wheel_quiet.py` now draws roller cartridges: two 687s (7 × 14 × 5, from his kit) on a printed stepped sleeve.

- **Sleeve:** Ø7 ends through the bearings and a Ø8.6 middle on the inner races. The same M3 × 50 clamps it between the axle blocks.
- **No axial play:** pocket B is deeper, so the second bearing stops on the sleeve's shoulder and the pair holds the roller from the inside. In the noise model this is the "bearings + axial preload" case: −10 dB hub, −18 dB floor.
- **Petals (round 3, which replaced round 2's r 62.5 / outboard-cone cuts):** the axle blocks are built smaller, R8 to R6, and seen along each axle are the R6 round with symmetric 45° flanks. The tips are at r 63.6.
- **Step sim with round 3** (`petal_step.py trim`, 2,400 runs; round 2's cuts gave the same numbers): the petals no longer touch a step, so the wheel climbs as one without petals.
  - 45–60°: 100 % at 12–24 mm, as before.
  - 75°: 40–75 % against the v13 petals' 65–100 %.
  - 82°: 50 % against 30–35 % at 12 mm, but 5 % against 0 % at 16 mm.
  - 90° (along the axle): 0 % either way.
- The trade (clearance and the outcome Fernando saw on the real wheel, against the paddle effect the sim credits the petals with) is his to judge on the printed part.

## 4. Round 1 proposal geometry (superseded by section 5)

- **Rollers:** a 623 (3 × 10 × 4, in the kit) pressed flush into each end of the core, with a Ø4 clearance bore between them. The thrust journals are gone, and the tread is unchanged.
- **Hub:** a Ø5 boss on each block thrust face, so the hub touches only the stationary inner races. The pilot-side boss face sits 0.3 mm off its inner race; the head-side boss face sits 0.8 mm off.
- **Washer:** a printed TPU ring (Ø3.4 / Ø5 × 1.2) on the axle at the head end. Squeezed 0.1 mm nominally (up to 0.4 with print tolerance), it seats the roller against the pilot-side boss, which removes the axial play. The inner races do not turn, so the preload adds no rolling drag.
- **Petals:** the same hub with the 45° / r 58 petal chamfer, exported only for a physical A/B (sim: no gain).
- **Checks** (`validation/wheel_quiet/checks.json`): every part is one valid solid. Nothing clashes with the hub, and the gaps are hub to inner race 0.3, hub to core 1.05 and hub to tread 0.95. The core wall at a bearing pocket is 6.15 mm. The chamfer cut leaves six 0.3 mm³ slivers, which are dropped.
- **VERIFY:** the 623 inner-race shoulder (taken as r 2.6); the pocket fit; the TPU washer's squeeze force; the axle (an M3 thread in the inner race, or a smooth shank).
