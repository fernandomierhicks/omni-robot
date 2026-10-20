# Component study and v2 simulation results

Oct 17, 2026 · overnight run by Claude (sim v2 + CAD v1)

> **Archived.** Merged into [../design.md](../design.md), which is now the single source of truth. Kept for the full study tables. Some numbers here predate the trapdoor gate and the CAD masses.

This checks the design doc's component choices against the v2 MuJoCo simulation, which now models motors, battery sag, omni-wheel traction, a physical flywheel launcher and the foam ball. It ends with a part-number BOM and the changes I'd propose to the design doc. Numbers come from `simulation/v2 detailed physics/studies/` (re-run any study with the venv Python). Values marked (est) are engineering estimates waiting for bench data.

## Recommendations at a glance

| Function | Pick | Part number | Qty | Verdict |
| --- | --- | --- | --- | --- |
| Drive actuator | CubeMars AK40-10 V3.0 KV170, with driver | AK40-10 V3.0 KV170 | 3 | Confirmed: lightest, fast enough, cool |
| Omni wheel | goBILDA 96 mm omni, 14 mm bore, 50A rollers | 3624-0014-0096 | 3 | **Change** from 125 mm |
| Battery | 7S1P Molicel P50B 21700 (5 Ah, ~126 Wh) + 7S BMS ≥40 A | INR21700-P50B | 7 cells | Confirmed (1P is enough) |
| Flywheel motor | T-Motor F90 2806.5 1300KV (iFlight XING 2806.5 1300KV equivalent) | F90 2806.5 KV1300 | 2 | New |
| Flywheel ESC | TBS Lucid 8S 60A 4-in-1, AM32, bidirectional DShot | TBS Lucid 8S 4in1 | 1 | New: must be 8S-rated for 7S |
| Flywheels | Printed: PETG hub + TPU 95A tire, Ø70 × 28 mm, 8 × M4 steel ring | cad/out/stl | 2 | New |
| Gate servo | Standard-size 25 kg·cm metal-gear servo, 25T horn | e.g. DSSERVO DS3225MG | 1 | New |
| Ball | Pickleball-size (74 mm) foam practice ball, ~20–25 g | measure before buying in bulk | | **Change** from 70–75 mm 30+ g ball |

The rest of the electronics (Jetson Orin Nano Super, STM32G4 board, RPLIDAR C1, 2 × OV9782, ELRS receiver, e-stop) is unchanged from the design doc and not part of this study.

## 1. How the simulation models each part

| Part | Model | Parameters from |
| --- | --- | --- |
| AK40-10 | FOC driver with PI speed loop; current limit; voltage limit from bus voltage minus back-EMF; 90% gearbox efficiency (est); 0.06 N·m backdrive friction; reflected rotor inertia; two-node thermal model | CubeMars datasheet (Kt 0.056, 978 mΩ, 7.3 A peak, 435 rpm no-load, rotor 97.35 g·cm²); thermal (est) |
| Battery | Thevenin: OCV(SoC) curve, R0 rising at low SoC and low temperature, one RC branch, BMS trip | Molicel P50B/P45B datasheets (est for RC branch) |
| Omni wheels | Fast "sphere" model (driven hub + contact sphere on a ball joint) **validated against** an explicit double-row barrel-roller model | goBILDA/Nexus specs; roller friction μ (est) |
| Launcher | Two flywheels on sprung slides (the foam squeeze), tilted axes, driven by drone motor + ESC models with closed-loop rpm on DShot telemetry | Drone motor KV/R/I0 datasheets; foam stiffness and restitution (est) |
| Ball | 74 mm, 22 g, Cd 0.5 drag (MuJoCo ellipsoid fluid model), restitution 0.45 on the floor, 0.15 on the gate's EVA pad | (est), measure |

All the sim checks run headless; `python -m pytest -q` in `simulation/v2 detailed physics` covers the motor/battery models, drive tracking, a launcher shot and a catch-and-return cycle.

## 2. Omni-wheel traction and slip

![wheel slip](../simulation/v2%20detailed%20physics/studies/results/wheel_slip.png)

- **Traction, not motors, limits acceleration.** A kiwi base accelerating between two wheels is pushed by those two wheels alone (each needs 0.58·m·a), and load transfer unloads them. Quasi-static limit: a_max = μ·N₀·g / ((2/3)|t·u| + μ(2/3)(h/L)(u·r)), per wheel, minimum over wheels.
- **The sim agrees with that model in shape**, and runs about 0.5 m/s² lower in the worst directions. The extra loss comes from the instant acceleration step rocking the robot. With μ = 0.95 (soft 50A rollers) the worst direction allows ~2.4 m/s² in sim (3.0 in the model). Toward a wheel it allows up to ~5–7 m/s², where tipping takes over.
- **The CG sits 1.5 cm forward** (launcher, lidar), which shifts the worst directions to the two rear diagonals (120° / 240°).
- **Explicit rollers agree** with the sphere model within ±25% on the slip limit. They also add the real double-row behavior: 2–3% speed ripple and ±2° heading wander per 2 s. The fast sphere model is good enough for training; the roller model is there for validation (`--wheel-model rollers`).
- **The design doc's 4–5 m/s² target isn't reachable in every direction** with omni rollers on a smooth floor; it would need μ ≥ 1.1 or a much lower CG. It also isn't needed: see section 3.

Recommendations: set the firmware acceleration limit to 2.5 m/s². Then make it direction-aware using the quasi-static model (cheap to compute on the MCU), which gives 30–50% more acceleration in good directions. Add jerk limiting, which recovers most of the 0.5 m/s² dynamic penalty.

## 3. Drive motor × wheel

The firmware acceleration limit for each combination is 85% of its worst-direction traction limit. Catches use ground-truth perception after a 0.25 s delay.

| Motor | Wheel | Mass | CG | Clearance | Top speed full / 10% battery | Traction-limited accel | 0.37 m move | Winding steady-state | Catches (0.45 m zone) | Drive cost |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **AK40-10** | **goBILDA 96** | 5.37 kg | 15.8 cm | 22 mm | 2.63 / 2.00 m/s | 2.98 m/s² | 0.97 s | 27 °C | 20/20 | $468 |
| AK40-10 | Nexus 100 | 6.02 | 14.9 | 24 | 2.74 / 2.08 | 2.66 | 1.00 | 27 | 20/20 | $513 |
| AK40-10 | Nexus 127 | 6.45 | 16.3 | 37 | 3.48 / 2.64 | 2.62 | 1.00 | 28 | 20/20 | $543 |
| AK45-10 | goBILDA 96 | 5.58 | 15.4 | 22 | **1.09 / 0.82** | 3.04 | 0.97 | 26 | 20/20 | $510 |
| DM-J4310 | goBILDA 96 | 5.70 | 15.1 | 20 | **1.21 / 0.92** | 3.07 | 0.96 | 26 | 20/20 | $435 |
| GIM6010-8 | Nexus 100 | 6.61 | 14.0 | 16 | 2.64 / 2.00 | 2.75 | 0.99 | 27 | 20/20 | $420 |
| GIM6010-8 | goBILDA 96 | 5.96 | | **14** | | | | | | |
| AK60-6 | Nexus 127 | 7.02 | 15.5 | 25 | 2.57 / 1.94 | 2.70 | 0.99 | 26 | 20/20 | $1215 |
| AK60-6 | 96 / 100 mm | | | **10–12** | | | | | | |

Bold marks a failed requirement: top speed below 1.4 m/s, or ground clearance below 15 mm.

What the sim says:

- **Catching doesn't separate the motors.** With good perception, every combination catches 30/30 even in a stretched 0.65 m zone, with 0.6 m moves taking 1.2–1.3 s. The 33 cm funnel forgives ±12 cm. Only cutting the acceleration limit to 1.5 m/s² costs catches (26/30). Accel limits from 2.0 to 3.5 m/s² all score 30/30.
- **So the choice comes down to mass, cost, clearance, top speed and heat.** The AK40-10 is the lightest actuator (190 g), its 2.0 m/s low-battery top speed covers RC driving and the later moving-catch mode, and it runs cool. Steady-state winding rise is about 2–3 °C over a continuous catch-move cycle, because traction caps the torque near 1 N·m. That retires the doc's open question about thermal limits under repeated 1.6 N·m bursts (pending real thermal constants).
- **AK45-10 and DM-J4310 are too slow** at 24 V (under 1.2 m/s). **AK60-6** costs 2.6× as much for torque the floor can't use. **GIM6010-8** is the budget option ($105 each), but it is 200 g heavier per motor and needs ≥100 mm wheels for clearance.
- **96 mm wheels beat 125 mm.** They are lighter (119 g vs 480 g), grippier (soft 50A rubber), and give a lower deck and CG. A 127 mm wheel adds ~1 kg and raises the CG for no catch benefit. Smaller rollers cope worse with carpet edges and thresholds, so test on the real play floor. goBILDA doesn't publish a load rating; at ~1.9 kg per wheel this robot is lighter per wheel than typical FTC robots on the same wheel.

## 4. Launcher: drone motors and flywheel size

![launcher](../simulation/v2%20detailed%20physics/studies/results/launcher_components.png)

Requirement: a 52° return throw to chest height needs 5.5 m/s at 1.5 m and 8.8 m/s at 5 m. The foam ball's drag is large: about 25% of its weight at 6 m/s. Design maximum is 10.1 m/s (+15% margin); a typical 3 m throw needs 6.8 m/s.

| Flywheel Ø | Inertia (each) | rpm for 3 m / max | Speed dip per shot | Repeat scatter (5 shots) |
| --- | --- | --- | --- | --- |
| 50 mm | 2.6e-5 kg·m² | 3960 / 5920 | 26% | ±0.22 m/s, ±1.1° |
| 60 mm | 4.7e-5 | 3260 / 5050 | 23% | ±0.15 m/s, ±1.5° |
| **70 mm** | **8.0e-5** | **2740 / 3940** | **19%** | **±0.15 m/s, ±0.7°** |
| 80 mm | 1.3e-4 | 2240 / 3420 | 19% | ±0.12 m/s, ±1.3° |

With 70 mm flywheels:

| Motor | Throttle at max speed (22 V) | Spin-up to 3 m speed | Recovery after a shot | Peak bus current |
| --- | --- | --- | --- | --- |
| **T-Motor F90 2806.5 1300KV** | 14% | 0.13 s | 22 ms | 4.1 A |
| iFlight XING 2806.5 1300KV | 14% | 0.13 s | 22 ms | 3.4 A |
| 2207 1750KV (5-inch class) | 11% | 0.16 s | 30 ms | 3.3 A |
| 3115 900KV (cinelifter class) | 20% | 0.09 s | 16 ms | 4.4 A |

- Every candidate has huge margin. A flywheel needs only ~4000 rpm out of 28,000 available, and each shot takes ~1 J. The 2806.5 1300KV is a good middle pick: common, cheap, 47 g, with an M5 shaft for the hub. A lower-KV motor (the 3115 900KV) would use more of the throttle range, but it's heavier and not needed.
- **70 mm flywheels** fit the geometry, keep the shot dip around 20% (the calibration table absorbs that), and gave the tightest scatter.
- Ball-to-rim transfer is about 0.70. That depends on TPU-to-foam friction and the squeeze (10 mm, ~15 N), so measure it on the bench. The calibration table built from logged shots is still the right tool, as the doc says.
- The ESC must be 8S-rated: a full 7S pack is 29.4 V, above most 6S ESC limits. Use AM32 with bidirectional DShot so the MCU closes the rpm loop on eRPM telemetry.
- These rows came from the earlier sliding-gate version of the launcher. After the switch to the trapdoor gate, the 70 mm calibration gives the same transfer (7.7 m/s at 3000 rpm, 10.2 at 4000) and repeat pairs within ~0.2 m/s.

## 5. Battery

| Pack | Mass | R0 | Worst burst (hard move + flywheel spin-up) | Avg play power → runtime | At 10% SoC |
| --- | --- | --- | --- | --- | --- |
| **7S1P P50B** | 612 g | 137 mΩ | 12.6 A, 29.3 → 27.5 V | 28 W → **3.9 h** | 22.4 V: 1.97 m/s top speed |
| 7S1P P45B | 612 g | 130 mΩ | 12.6 A, 29.3 → 27.6 V | 28 W → 3.5 h | 22.5 V: 1.98 m/s |
| 7S2P P45B | 1225 g | 72 mΩ | 12.4 A, 29.3 → 28.4 V | 28 W → 6.9 h | 23.2 V: 2.05 m/s |

Currents are small (a traction-limited base and ~1 J shots), so voltage sag is under 2 V and costs little speed even near empty. 7S1P is enough. The Jetson and sensors (~22 W) dominate the energy budget.

## 6. What the simulation work changed in the mechanical design

1. **Catch pocket and gate.** The ball must stop in a vertical-walled pocket one ball diameter below the funnel throat. With the gate at the throat, balls arriving at 6.5 m/s skated across and climbed out of the cone. The gate needs a soft (EVA) top, or the ball bounces out of the pocket.
2. **Gate = trapdoor on the servo axis**, opening 95° into a window in the side wall, so the ball drops straight with full tube width. A sliding shutter needs ~250 mm of side travel for its rack and drags the ball sideways. A standard 25 kg·cm servo holds the padded catch impact (~0.6 N·m) without a latch.
3. **Gravity feed into the flywheels through a J-bend.** At a 52° launch angle the ball can't be gravity-fed into over-under wheels. With left/right wheels on tilted axes, a J-bend under the gate turns the falling ball up into the nip. That needs ≥15 cm of drop; the lidar/Jetson layer under the payload plate went from 105 to 65 mm to make room. Left/right wheels give no backspin, which is fine for gentle lobs.
4. **The funnel net needs grip and rolling loss.** In sim, a slippery cone lets off-center balls swirl around and climb out. A soft fabric net (high μ, high rolling resistance) catches 6/6 off-center arrivals.

## 7. Verify on hardware first

- [ ] Roller-floor friction μ on the real play floor (sets the acceleration limit: 2.4 m/s² at μ 0.95, 1.6 at μ 0.6).
- [ ] goBILDA 96 mm omni width (assumed 38 mm), 32 mm-pattern hole phase, and radial and moment load on the AK40-10 output bearing (the wheel is overhung ~20 mm).
- [ ] AK40-10 output Ø15 feature: recess (assumed) or boss; whether the driver accepts a full 7S pack (29.4 V); thermal constants.
- [ ] Foam ball: mass, diameter, restitution on the EVA pad and on the floor, and Cd (film a few throws).
- [ ] Launcher transfer on the bench, ~20 shots: exit speed vs rpm at 52°. Update `launcher_cal.json`.
- [ ] Drone motor bell top and M5 shaft fit for the flywheel hub; servo horn hole pattern; Jetson and RPLIDAR C1 mounting patterns (all marked VERIFY in `cad/`).

## 8. Proposed design-doc changes (not applied)

- Ball: 74 mm pickleball-size foam practice ball, ~22 g. Drag is a first-order effect, so landing prediction must include it.
- Wheels: 96 mm goBILDA omni instead of 100–125 mm.
- Requirements: base acceleration limit 2.5 m/s², direction-aware and jerk-limited, instead of 4–5. The catch rate is insensitive above 2 m/s².
- Tipping: the support polygon is the wheel triangle, so the lever is half the wheel-circle radius (9 cm here). The doc's "45 cm wide, CG 40 cm, ~5.5 m/s²" example tips at about 2.2–2.5 m/s².
- Catching: catch pocket below the throat + EVA-padded trapdoor gate + J-bend gravity feed. Break-beam across the held ball.
- Throwing: left/right flywheels (no backspin) at 70 mm, T-Motor F90 2806.5 1300KV, 8S AM32 4-in-1 ESC.
- Mass: ~5.6 kg with printed parts (2.3 kg of PETG/PLA/TPU), below the 6–9 kg planning range.
