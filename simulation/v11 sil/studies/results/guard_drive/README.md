# The lidar drive guard (Nov 23, 2026; sim)

**Asked (Fernando):** a low-level lidar collision layer, on whenever the lidar is, under SLAM and Nav2, for every way
of driving (RC, flight stick, console, Nav2); switchable off for good or held off for a moment; liberal: full speed,
then braking only as late as the robot can, stopping ~10 cm short and letting you creep in to touch. Then: what top
speed and acceleration the robot has on the RC sticks, and what a full-speed run into a wall looks like.

**Built:** `omni_cpp` `drive_guard.hpp/.cpp` (ROS-free core) run by `mode_manager` on every TELEOP and PICK twist;
the RC through the robot computer (protocol 10: STATE carries the sticks; the firmware's direct MANUAL only as a
deliberate fallback). Details: [ros/README.md](../../../../../ros/README.md) "The lidar drive guard",
[firmware README](../../../../../firmware/README.md), `robot.yaml` `mode_manager` `guard_*` / `rc_*`.
Charts of these runs: the published page "Lidar Drive Guard".

## Setup

The full arm stack (`omni.sh arm`, house, clutter off, the robot's firmware in SIL, ~0.5x real time; results in sim
time). The sim's RC receiver (`/sim/rc`) at full stick, through `mode_manager` and its guard (circle r 0.24 m, buffer
0.10 m, decel 2.5 m/s², latency 0.10 s, creep 0.05 m/s). `ros/tools/guard_drive.py` drives; truth from
`/ground_truth/odom`; `studies/guard_drive_clear.py` measures the gap ahead on the house model from the sim robot's
body ring (r 0.225; its stowed arm reaches ~0.24 ahead). Data: `rc_runs_2026-11-23.json` (+ `_summary.json`).

## Results (RC full stick, 2 m/s = the adult profile's cap)

| Run | Top | Accel | Rest to 95 % | At 50 cm | At 20 cm | At 10 cm | Stopped at | Back off |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| From rest, forward | 2.01 m/s | 4.4 m/s² | 0.56 s | | | | | |
| From rest, sideways | 2.04 | 4.3 | 0.68 | | | | | |
| From rest, diagonal 45° | 1.99 | 3.1 | 0.86 | | | | | |
| From rest, backward | 2.01 | 4.7 | 0.56 | | | | | |
| Wall, forward | 2.00 | | | 1.23 m/s | 0.48 | 0.04 | 5 cm, creeping | 2.03 m/s |
| Wall, sideways | 2.04 | | | 1.25 | 0.48 | 0.04 | 5 cm | 2.05 |
| Wall, diagonal | 1.99 | | | 1.25 | 0.48 | 0.04 | 5 cm | 2.00 |
| Wall, backward | 2.01 | | | 1.12 | 0.43 | 0.02 | 8 cm | 2.02 |
| Wall, forward at 1 m/s | 0.98 | | | 0.96 | 0.47 | 0.04 | 6 cm | 0.99 |
| Wall, forward, guard held off (RC ch 6) | 2.00 | | | 2.0 | 2.0 | — | **hit at ~2 m/s** | 2.04 |
| Console stick, forward (its 1 m/s default) | 1.01 | | | 1.00 | 0.52 | 0.04 | 4 cm | 1.05 |

At 2 m/s the speed starts to fall ~0.9–1.05 m from the wall; braking peaks at 2.2–3.0 m/s² (the firmware can brake
2.8–4.3). "Stopped at" is where the run ended (2 s under 0.05 m/s): the robot was still creeping in. No guarded run
touched. Leaving the wall was never limited.

## Findings

- **Full stick into a wall from any direction ends in a creep, not a crash**, backward too: the blind wedge behind
  the lidar's own posts (±11° in the sim) is filled from the walls either side. The profile is the planned one: the
  speed follows the law within a few cm/s down to 20 cm, then the creep.
- **The latency budget holds:** the speed at a given distance is slightly above the law (0.48 vs 0.45 m/s at 20 cm),
  never into the buffer at speed. In a ROS-free model of the loop (`test_drive_guard.py`) 30 ms of tick, link and
  motor plus the firmware's jerk ramp creeps into the buffer from 1, 2 and 3.5 m/s; 100 ms still never touches.
- **Acceleration:** 4.3–4.7 m/s² along the axes, 3.1 diagonally (the firmware's direction table: traction, est),
  2 m/s in ~0.6 s. Top speed is the adult profile's 2 m/s cap, not the motors (AK40-10 at 37.2 rad/s: ~3.7 m/s along
  an axis, ~2.6 diagonally, no load, est).
- **Held off, it hits** at full speed (RC channel 6), as asked.

## Slide (Nov 24)

**Asked (Fernando):** slow only the part of the motion going into the wall. The guard now keeps the part along what
it would hit (the contact's normal: a wall's own) and cuts only the part into it, as little as the law allows; every
twist it sends still obeys the law along its own way (`guard_slide`, live; false = the scaling above). Square-on runs
are unchanged (the table above). `guard_drive.py slide:speed:deg` holds full stick forward at the nearest wall, the
way `deg` to it; `guard_drive_clear.py` scores it on the house model: speed along / into the nearest surface against
the true gap all round. Data: `slide_runs_2026-11-24.json` (+ `_summary.json`).

| Run (RC full stick, 2 m/s) | At 50 cm along / into | At 20 cm | At 10 cm | At 5 cm | Sliding | Closest |
| --- | --- | --- | --- | --- | --- | --- |
| 25° to the wall | 1.56 / 1.02 m/s | 1.59 / 0.40 | 1.61 / 0.20 | 1.34 / 0.06 | ~4.5 s at 0.9–1.5 m/s, 1–4 cm off | 1.4 cm |
| 45° to the wall | 1.44 / 1.02 | 1.52 / 0.41 | 1.45 / 0.20 | 1.41 / 0.02 | ~1.5 s at 1.3–1.5 m/s, 3 cm off | 2.3 cm |

(Two more runs, `slide:2:45` first and `slide:1:60`, started against a wall left by an earlier run: no approach to
score; no contact either.) Before (Nov 23, scaling) the 45° run slowed to a 0.04 m/s creep 5 cm off.

- **It glides:** the along-wall speed is kept through the approach and along the wall (~1.4–1.6 m/s, the asked part
  less a little); the into-wall speed falls as the law asks for a shallow way: 0.4 m/s at 20 cm, 0.2 at 10, ~0 at 3 cm.
- **It slides closer than square-on:** 1–4 cm off the wall, not ~10. A shallow way has a long free run, so the law
  lets some speed into the wall; the into speed near the wall stays under 0.25–0.33 m/s and nothing touched.
- **At the wall's end** (a corner, furniture) it creeps 6–9 cm off, the stick still held, with a small wobble along
  (up to ~0.2 m/s) as the two walls take turns.
- Unit checks (`test_drive_guard.py`): along kept within 1 cm/s at gaps 5–40 cm, the old cap with `slide` false,
  corners cut by both walls, a post passed on its side; on lidar-spaced walls a square push drifts 0 sideways.

## Limits

Simulation: braking, latency and grip are estimates; the guard's numbers are robot.yaml's. Only what crosses the
16.3 cm scan plane is seen. The sim's lidar shadows are the drawn (v7-based) robot's, not `cad/v16`'s. The guard's
circle (0.24) must be the real robot's outermost part. One run per case.

## Before (Nov 23, same day): Nav2's collision monitor

The first study, on Nav2's collision monitor (approach, 1.2 s): it guarded only the console's stick, and only with
Nav2 up; forward/sideways/diagonal from 2 m/s it stopped 15–18 cm short, slowing from ~1.1 m per m/s; **backward it
hit at 1.9 m/s** (the blind wedge); after a contact it blocked every direction. Data:
`collision_monitor_runs*_2026-11-23.json`. Superseded by the guard above.
