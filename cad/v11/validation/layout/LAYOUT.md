# Exploratory layout study: second floor, electronics, computer, lidar (Nov 3 2026)

Preliminary, high-level layouts only: boxes and cylinders for the devices, real-size plugs with their leads bent at about 5 × OD. This is not detail CAD.
The bin and the face/screen are not studied; only their second-floor area is marked.
Script: `tools/layout_study.py`, run with the omni-robot-sim interpreter. Data: `layout.json` and `dsweep.json`. Views: `layout_<A..F>.png`, `iso_C_*.png`, `iso_F_*.png`.

## What was laid out

| Item | Envelope used | Source |
| --- | --- | --- |
| Computer | BMAX B8A Pro 126 × 112 × 52 mm, 430 g. Front: D435 and Teensy on USB-A 3. Rear: SO-101 adapter and A2M12 adapter on USB-A 2, 19 V barrel. | Maker size (components/computer trade). **Port layout assumed (VERIFY on the unit).** |
| Plugs + leads | USB-A 3: 16 × 8.5 × 30 overmold, Ø4.6 lead, R23 bend. USB-A 2: 15 × 8 × 28, Ø3.8, R19. DC barrel: Ø10 × 32, Ø4.5, R15. Micro-B and USB-C similar. All bent upward. | Typical moulded cables (**VERIFY** against the bought ones) |
| Battery | OVONIC 6S 5200: 157 × 47 × 46, 706 g, plus 50 mm in line for the XT90-S pigtail bend, mated pair and balance lead | Maker |
| Power board | 130 × 70 × 32 (est): fuse, e-stop contactor, current sensor, 19 V/100 W buck, 12 V/8 A arm buck, 5 V buck, motor distribution. Keep-outs: 6S lead 40, motor leads 30. | Estimate |
| MCU carrier | 100 × 65 × 25 (est): Teensy 4.1, CAN transceivers, BNO085, ELRS receiver. Micro-B lead plus header keep-outs. | Estimate |
| Servo adapter | Waveshare Bus Servo Adapter (A), 45 × 35 × 15 | Estimate |
| Lidar | A2M12 Ø76 × 41, scan plane z 163 (sim ARM). Plane height within the body VERIFY. | Sim + datasheet class |
| D435 | 90 × 25 × 25 at the nose, (190, 0, 111.5), tilted 30° | Sim ARM_D435 |
| Arm | SO-101 base at (140, 0), z 218.5, on a 37 mm riser on the second floor (top 181.5). Floor kept clear r 100 around its base. | Sim ARM (fingers-down floor reach needs this height) |
| Base | Lean pods (`pod_lean.py`). Deck top 62. Pods swept droop..bump, + 3 mm. | `base.py` |

## Fixed vertical stack

- Deck top 62.
- The pods reach 155 (wheel top at bump; spring plungers 143).
- The lidar's window is from about 158 to 176. Its scan plane at 163 sees over every pod part.
- The second floor's underside is at 176.5 and its top at 181.5; the arm base at 218.5 needs this height.

There is no room for a shelf between the pods' tops and the second floor. Electronics can only go on the deck or on top of the second floor.

## Results

| Config | Hand pattern | D / slide | Robot Ø | Electronics on the deck? | D435 at the nose | Lidar | Tip fwd / side (g) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A | sim (pivots toward the sides) | 146 / −32 | 440 | **No**: computer, battery, power, MCU do not fit | **Hits FL/FR wheels** (129 mm²) | centre | 1.06 / 0.78 (deck-only CG) |
| B | turned 90° (pivots front/back) | 146 / −32 | 440 | **No** | clear 57 mm | x 140, clear 16 mm | 0.59 / 1.21 (deck-only CG) |
| C | turned 90° | 146 / −32 | 440 | battery + electronics on the second floor | clear 57 mm | x 140 (as sim), 13° post shadows | 0.61 / 1.02 |
| D | sim | 146 / −32 | 440 | second floor | **hits wheels** | centre pedestal | 1.00 / 0.65 |
| E | turned 90°, pods-as-ring | 134 / −34 | 416 | second floor | clear 60 mm | x 140, clear 19 mm | 0.53 / 0.96 |
| **F** | **sim** | **154 / −20** | **462** | second floor | **clear 7 mm** | **centre pedestal**, 18° post shadows | **0.98 / 0.77** |

**Tip column:** static rigid-body tipping at the computed CG.
- C–F: about 10.0 kg, CG z 123. That includes pods at 1.45 kg each (est) and the arm at 0.85 kg in its stow pose.
- Not included: bin, face, head, wiring. These will raise the CG and lower every tip figure.
- These are estimates, not sim results.

**Finding 1: the deck between the pods is not an electronics bay.**
- The four 174 mm pods in a pinwheel break the free deck into a central band and lobes about 110–120 mm across.
- With modelled plugs, the computer needs 222 × 126 mm (body 112 × 126) and the battery 215 × 47.
- Neither fits in any lobe. Growing the base doesn't fix it: up to D 186 (Ø536) the battery and power board still don't fit beside the computer (`dsweep.json`).

**Finding 2: one electronics block on the second floor's rear works on every base.**
- Contents: battery, computer, power board with the MCU carrier on standoffs over it, and the servo adapter by the arm.
- Keep-outs total about 476 cm², against about 1,100–1,250 cm² free outside the arm's area.
- Height: the computer top is at 236.5. The block reaches 272.5 with the MCU stacked and its lead bent up.
- Benefits: all the electronics and the short high-current runs are together, and the battery is reachable from the top.
- Costs:
  - the four motor XT30(2+2) leads and CAN pass down through the floor to the pods;
  - the CG rises: 0.7 kg of battery at about 210 instead of about 90;
  - the bin, later, sits on the block, so the arm's drop pose rises about 90 mm and must be re-planned.

**Finding 3: the hand pattern is a packaging vs. stability trade.**
- The facing pod pairs force the wheels to slide away from their pivots.
- With the sim's pattern, the front wheels end up at the nose. The D435 hits them at D 146, clears from D 150, and the x 140 lidar clears only from D 166.
- Turning the pattern 90° opens the front, but the wheelbase drops to ±81 mm. Forward tipping then comes at about 0.6 g at this CG, close to the sim's 6.2 m/s² forward start.

## Proposal (not adopted)

**F**: keep the sim's hand pattern and grow the wheel line to D 154 with a −20 mm slide (Ø462, v10 Ø450).
- The D435 stays at the sim's nose spot.
- The lidar moves to a pedestal at the deck centre (scan z 163). It is blind only behind the four Ø16 second-floor posts, about 18° in total.
- The electronics block sits on the second floor's rear.
- Static tipping: about 1.0 g forward/back and 0.77 g sideways.

C (Ø440) is the compact alternative if a capped forward acceleration is acceptable.

## Open / VERIFY

- The BMAX port positions, and every plug and lead size. Right-angle USB and DC plugs would cut each port face's keep-out by about 30 mm.
- Sim changes needed before deciding: wheel positions (D, slide), `lidar_x` 0.14 → 0, and the mass/CG of the block. Then braking, tipping and arm-out acceleration limits.
- The arm's stow, drop and floor-reach paths against the block, and the bin on it. The r 100 keep-out is a placeholder.
- Motor/CAN lead routing from the four pods up to the block; e-stop on the block lid; lidar pedestal on the deck centre.
