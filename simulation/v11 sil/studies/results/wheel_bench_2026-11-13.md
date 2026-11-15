# One AK40-10 and a bare omni wheel on the bench against the sim (Nov 13, 2026)

**Hardware measurement**, wheels up, attended. One CubeMars AK40-10 V3.0 KV170 (CAN id 2, servo mode) turning the
`cad/v13` 140 mm omni wheel with 12 plain-bore rollers and TPU treads (the older wheel, **not** the 687 bearing wheel
the sim's arm robot carries), on a bench supply at 24 V current-limited to 4 A. The robot's own chain: a **Teensy 4.1**
with the arm firmware (env `arm_bench`: the e-stop input ignored, a fixed 24.0 V reported, since neither is wired on
the bench; core unchanged, build e42b756f, protocol 9) on the Pi 4's USB, the Pi's robot step (`mcu_link`,
`mode_manager`, the console), the console's motor test (`motors_run`) → `mode_manager` WHEELS → the firmware's
`DRIVE_WHEELS` ramp → servo-mode ERPM. Recorded at the MCU's 200 Hz by `ros/tools/wheel_capture.py`; compared by
`studies/wheel_bench.py`, which replays each run's commanded speed (as the motor gets it, every 4 ms) through the sim's
own wheel drive (`models.DriveActuator`: PI kp 2 / ki 40, current and voltage limits; the arm model's drive-joint
inertia 1.88e-3 kg m², 0.1208 N m Coulomb + 5e-4 N m s/rad).

Raw captures (outside Dropbox): `C:/Users/ferna/bench_data/wheel_2026-11-13/*.csv`. Summaries here:
`wheel_bench_2026-11-13/steps_w20.json`, `wmax_sweep.json`, and the plots.

## Runs

| Run | Profile |
| --- | --- |
| `steps_r{50,300,1000,2000}_w20` | rest, +20, rest, −20, +20 (reversal), rest rad/s; each change ramped at 50 / 300 / 1000 / 2000 rad/s² |
| `steps_r2000_wmax` | the same to ±44.9 rad/s (the firmware's cap) at 2000 rad/s² |
| `sweep_wmax` | steady speeds 2.5 … 44.9 rad/s and back, both ways, 1.5 s each (friction against speed) |

## Findings

1. **At the robot's own wheel accelerations the motor matches the sim.** At 50 rad/s² (about the peak a wheel sees on
   the robot: 2.5–4.3 m/s² body, ≤ ~45 rad/s² a wheel) the wheel tracks the firmware's ramp: 90 % in 0.37 s (sim
   0.36 s), 0.2 rad/s overshoot, ±0.08 rad/s ripple at a hold.
2. **The motor caps its own acceleration at ~148 rad/s²** (≈ 200,000 ERPM/s), whatever the firmware asks: the
   ramps at 300, 1000 and 2000 rad/s² all come out as a straight 147–149 rad/s² ramp (slope fitted over each ramp's
   middle), at a steady 0.7 A up / −0.4 A down, far below its current limit. So it is a limit inside servo mode's
   velocity loop (a CubeMars-tool setting, likely; not read yet), not torque. The sim has none: it follows the asked
   ramp into its 6.5 A current limit (~1000 rad/s² on this wheel). Rim acceleration 10.4 m/s²: 3× what the robot asks,
   so not a limit for driving; it bounds how fast a wheel can respond to slip, and a step command takes |Δω| / 148 s.
3. **Top speed at 24 V is 38.4–38.6 rad/s (368 rpm)**, at only 0.2 A (voltage-limited, as the R4 bench found). The
   sim's AK40-10 (`noload_rpm` 461 at 24 V, fitted to the maker's chart) reaches the 44.9 rad/s cap: its back-EMF is
   ~25 % optimistic against this motor. Speed scales with the bus: ~35.6 rad/s on a nominal 6S pack (22.2 V), ~33.7 at
   the 21 V stop (inference). The arm robot's 2 m/s needs ≤ ~29 rad/s a wheel diagonally, plus yaw.
4. **Inertia: J/k = 3.9–4.1 mA s²/rad**, the same in every run (including the tracked 50 rad/s² one). Which J depends
   on the torque per amp of the motor's reported current, not yet measured: the sim's 0.73 N m/A (the chart fit) gives
   **2.85e-3 kg m², 51 % over the sim's 1.88e-3** (rotor 0.97e-3 reflected + wheel 0.91e-3); the firmware's 0.48 N m/A
   (`AK_KT`, est) gives 1.87e-3, the sim's exactly. This is a different (older) wheel from the sim's.
5. **Friction rises with speed**: I ≈ 0.073 + 0.0032 |ω| A (0.08 A at 2.5 rad/s, 0.17 A at 30, 0.19 A at 38.5; a
   little higher speeding up than slowing down). The sim's is nearly flat (0.168 + 0.0008 |ω| A): the same around
   30 rad/s, twice the real at low speed. In torque: 0.053 N m + 0.0023 N m s/rad at 0.73 N m/A.
6. **The supply kept up.** No motor error code (no under-voltage) in any run; peak current 1.4 A (motor), and a bare
   wheel needs ≲ 25 W at the top speed. The 4 A limit is not what limits anything here; 7 A changes nothing on this
   bench. The robot's mass is what will load it (sim: up to ~7 A a wheel at peak acceleration).
7. Smaller: the motor ends each ramp 0.4–0.6 rad/s over (sim 0.05); the sim's stiffer speed loop makes a torque pulse
   at every 4 ms command step (its current, averaged over 5 ms here, compares on the mean).

## Did the bench reach what the robot asks?

- **Accelerations: yes, with margin, both ways.** The arm robot's adult limits ask a wheel for ≤ ~43 rad/s²
  (4.26 m/s² forward, the accel table's top), ~74 rad/s² with a 15 rad/s² yaw on top; ramped stops the same. The
  motor tracked 50 rad/s² exactly and reached its own 148 rad/s² speeding up and slowing down.
- **Speeds: to the motor's top.** The adult worst case (2 m/s diagonal while turning at 4 rad/s) asks ~37 rad/s of a
  wheel: within the 38.5 measured at 24 V, past the ~35.6 a nominal 6S pack would give (est).
- **Torques: no.** A bare wheel needs ~0.5 N m at 148 rad/s²; the robot's mass needs ~0.7–1.1 N m a wheel at
  2.5–4.3 m/s² (~1–1.5 A at 0.73 N m/A), the sim's peak 4.1 N m. That needs the robot's inertia share on the bench
  (m r² / 2 ≈ 0.024 kg m² a wheel, ~8× this wheel and rotor: e.g. two Ø200 × 10 mm steel discs) or a load.

## Applied to the sim (Nov 13, Fernando's ask)

`catalog.DRIVE_MOTORS["AK40-10"]` is now the chart fit corrected by this bench: `noload_rpm` 461.5 → 374.2 (top
speed 38.5 rad/s at 24 V through the model's own resistance and friction), friction `backdrive_torque` 0.1208 →
0.0533 N m and `viscous` 0.0005 → 0.00232 N m s/rad (finding 5, through the model's own torque per amp), and the new
`accel_limit` 148 rad/s²: `models.DriveActuator` ramps its setpoint at that rate (from the wheel's own speed after
coasting), on the SIL path and the direct-physics studies alike. The chart fit is kept as `AK40-10-chart` for
reproducing earlier results. Exports regenerated: the firmware's `MAX_WHEEL_SPEED` (0.95 × no-load) 44.9 → 37.2 rad/s
(the console's motor-test cap with it). Replayed through the updated sim (`after_*.png`, `after_model.json`): the
2000 rad/s² step to 20 rad/s reaches 90 % in 0.122 s (bench 0.135, before 0.0135), the ±38.5 reversal 0.546 s
(bench 0.55), the top speed 38.5 rad/s, the friction current 0.137 A at 20 rad/s (bench 0.13–0.19). What still differs:
the current while accelerating (bench ~0.7 A, sim ~0.5 A: the inertia / torque-per-amp question, and this is not the
sim's wheel), the motor's 0.4–0.6 rad/s overshoot at a ramp's end (sim 0.1), its ±0.08 rad/s speed ripple.

## More to explore (not measured yet)

- **Torque per amp**, which fixes the inertia from finding 4 and the friction torques: a known mass on a string round
  the hub, held at zero speed (0.5 kg at 70 mm = 0.34 N m reads 0.47 A at 0.73 N m/A, 0.72 A at 0.48); weigh this
  wheel to separate its inertia from the rotor's.
- **Under load**: the torque-speed curve and current limit (an added inertia or a brake), r_fit and i_peak, heating.
- **On the pack**: top speed and acceleration at 25.2 / 22.2 / 21 V; whether 368 rpm is the voltage (likely) or a
  setting (an ERPM limit near 51.5k).
- **The motor's own settings** in the CubeMars tool: the servo-mode acceleration (the 148 rad/s² ≈ 200k ERPM/s), the
  ERPM limit, the speed-loop gains (its overshoot and ripple), the timeout.
- **A second motor** and the robot's own wheel (687 bearings), for spread; the arm robot's drive results before Nov 13
  used the chart fit and are not rerun.
