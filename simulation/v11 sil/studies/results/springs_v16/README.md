# cad/v16 pod springs (Nov 21, 2026): `studies/springs_v16.py`

Fernando: tune the suspension for what is built (three deck quarters with their pods, wheels and motors in, weigh 3 kg:
1.0 kg a quarter); check that the drawn springs are the right ones; one or two per tower; two in series for more stroke?

## Inputs

- **Spring:** 7.85 OD x 4.25 ID x 20.2 free, solid at 12.0 (measured, Nov 20). It matches a JIS B 5012 yellow (extra
  light) 8 x 20 die spring, listed at **7.6 N/mm** (Century Spring DJ-8X20YCS 7.62; Raymond ASF 8x20 0.78 kgf/mm; Dingli
  DTF8-20 7.8). **Estimate:** clones vary. Kitchen-scale check: 5.0 mm of compression (15.2 mm long) should read ~38 N
  (3.9 kg).
- **Geometry (cad/v16 `dims.py`, the lever maths of `pod.py`):** wheel 80.6 mm from the pivot. The far bore sits 31.3 mm
  from it (0.388 of the wheel's travel at nominal) and the near bore 20.7 mm (0.257). The pads are 24 mm above the
  pivot's line, so the ratio grows toward bump (far: 0.38 at droop, 0.48 at bump; near: 0.24, 0.34). Droop −3, bump +15. The ceiling
  puts the far spring 0.1 mm into compression at droop.
- **Mass:** unsprung 0.675 kg a corner (est from the CAD's volumes at the density the weighing gives; the sim had 0.92).
  The robot: 4 kg as built now (base only), ~8 kg bottom-up full estimate (battery 0.56, arm ~0.75, computer 0.43,
  converters ~0.45, lidar 0.19, printed upper ~0.7, wiring, face; est), 9.63 kg the sim's CAD estimate.

## Static (per wheel, `static.json`)

| Arrangement | Wheel rate N/mm (droop → bump) | Preload N | Force at bump N | First spring solid at | Ride (mm from nominal, + up) / left to bump: 4 / 8 / 9.63 kg |
| --- | --- | --- | --- | --- | --- |
| **as drawn: 1 spring per bore, 2 per tower** | **3.3 → 6.3 (mean 4.6)** | 2.4 | 85 | +16.1 | −2.8 / 17.8, **+0.2 / 14.8, +1.3 / 13.7** |
| 1 per tower, far bore | mean 3.1 | 0.6 | 56 | +16.1 | −1.8 / 16.8, +2.3 / 12.7, +3.8 / 11.2 |
| 1 per tower, near bore | mean 1.5 | 0.4 | 27 | +22.9 | −0.1 / 15.1, +7.5 / 7.5, +9.9 / 5.1 |
| 2 per tower, bump stop at +17 | - | 2.4 | - | **+16.25: binds before the stop** | - |
| 2 in series x 2 per tower (towers ~20 mm taller) | mean 2.3 | 1.2 | 43 | +30.7 | −1.8 / 16.8, +3.7 / 11.3, +5.6 / 9.4 |
| 2 in series x 2, droop −6 | mean 2.3 | 1.9 | 50 | +28.9 | −5.1 / 20.1, +0.8 / 14.2, +2.9 / 12.2 |
| 2 in series, far bore only | mean 1.5 | 0.3 | 28 | +30.7 | −0.4 / 15.4, +6.9 / 8.1, +9.4 / 5.6 |

- At 4 kg (the base alone) the as-drawn pods sit 0.2 mm off their droop stops: the springs barely move until the upper
  robot is on. That is expected and not a reason to change springs.
- At 8–9.6 kg the drawn springs put the wheel 3.2–4.3 mm above droop. That is the same ride height as the sim's tuned pod
  (3 N/mm, 2 N preload; Oct 27 base trade, Nov 1 part P). It also leaves 14 mm to bump, and the rate then stiffens toward bump.
- **Travel on the bump side is set by the springs, not the stop:** the far spring goes solid at +16.1 mm, 1.1 mm past
  the stop. The wheel's envelope is also built round +15: the deck opening 2 mm, the fins 3 mm, the wheel's top z 155
  under the lidar plane z 163.

## Sim (drive_char part P's scenarios, arm robot, AK40-10)

The sim's pod is a linear spring, so each arrangement is linearized: real preload, and the secant to the static ride
height at that mass. The real pod stiffens toward bump, so it bottoms less than this predicts. The "rate to the bump force" row is the
other bracket. The drawn pod has no damper (plunger friction only: ζ 0.1 est).

| Setting | kg | Passed | Max tilt ° | On bump stop, all runs, ms | Start 0 / 90°, brake m/s² |
| --- | --- | --- | --- | --- | --- |
| old sim pod: 3 N/mm, 2 N, ζ 0.6 | 8 / 9.63 | 7/9, 8/9 | 15.3, 14.9 | 64, 188 | 6.4/6.0/7.2, 6.5/5.4/7.1 |
| **as drawn, no damper** | 8 / 9.63 | 8/9, 8/9 | 16.1, 15.4 | 202, 249 | 6.6/6.7/7.2, 6.8/6.3/7.2 |
| as drawn, rate to the bump force | 8 / 9.63 | 8/9, 8/9 | 15.9, 20.4 | 97, 158 | 5.9/5.5/7.3, 6.7/6.6/7.4 |
| as drawn, ζ 0.6 (a damper) | 8 / 9.63 | 7/9, 7/9 | 14.9, 14.9 | 56, 116 | 6.4/6.1/7.0, 6.5/6.4/7.0 |
| 1 per tower, far bore, no damper | 8 / 9.63 | 8/9, 8/9 | 14.6, 13.9 | 534, 732 | 6.5/5.1/7.0, 6.4/4.5/7.0 |
| 2 in series x 2, droop −6, no damper | 8 / 9.63 | 8/9, 8/9 | 15.8, 14.5 | 729, 2224 | 6.3/4.2/7.0, 6.3/4.6/6.7 |
| 2 in series x 2, droop −6, bump +17, no damper | 8 / 9.63 | 8/9, 8/9 | 16.0, 15.0 | 495, 724 | 6.2/4.5/6.9, 6.3/3.6/6.7 |

Every setting fails the 20 mm step taken at 45° (a wheel meets the edge along its axle, the known X4 diagonal limit);
the 16 mm rug at 45° fails at random in some. 30 mm step, driving forward: 14–15° with no damper (20° in the stiff
bracket) against 11–12° with ζ 0.6. Full rows: `sim_table.md`, `sim_detail.md`, `sim.jsonl`.

The sim's settled ride heights (`settle` rows) read 1.5–5 mm lower than the static calculation. During the settle the
held drive motors stop the wheels rolling, and the contact's friction, 70 mm under the pivot, holds the forks up by a
few N (more on soft springs). Use the static table for ride height. A real robot powered and holding its wheels may
show the same thing depending on how it is set down.

## Verdict

1. **The drawn springs are right: keep one spring per bore, two per tower.** They give the ride height and pitch the
   sim was tuned on, keep 14 mm to bump at 8–9.6 kg, and stiffen toward bump. That is the best of the arrangements tried.
2. **One per tower: no.** It is softer, rides higher, sits on the bump stop 3× as long, and starts sideways 1.6–1.8 m/s² slower.
3. **Two in series: no.** It needs towers ~20 mm taller, which puts the plunger tips at the lidar plane (Nov 20). It also
   halves the rate, so the robot bottoms 3–10× as long and starts sideways ~2 m/s² slower. More stroke does not help
   when the wheel's envelope and the bump stop stay at +15.
4. **Travel:** moving the bump stop gains at most ~1 mm before the far spring binds. More bump travel needs other
   springs and a new deck opening and fins, and the sim gives no reason for it. Extra droop below −3 leaves the springs
   slack at full droop.
5. **What the drawn pod lacks is damping.** Without it the pod spends longer on the bump stop and pitches 3–4° more on a
   30 mm step. Friction in the plunger guide or a TPU pad on the stop are the cheap options (to try on the pod jig).
6. **Next:** the kitchen-scale rate test. A rate 25 % off moves the ride height by ~1 mm, so the verdict holds over that
   range. Then weigh the robot when the upper half is on.
