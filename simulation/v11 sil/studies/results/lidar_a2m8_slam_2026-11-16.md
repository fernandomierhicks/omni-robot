# The A2M8 lidar model and SLAM, Nov 16, 2026 (sim)

The question: does the measured A2M8 (characterization/lidar, test 1) change the robot's SLAM against the A2M12 the sim
modelled from its datasheet?

**Setup.** ROS steps `auto_map` (slam_toolbox mapping, then `score_map`) and `tour` (slam_toolbox localization on that
map, scored against the truth every 0.1 s). Both drive v9's house tour (45.7 m, 44 s, turns included) on the sim's
truth, so every configuration drives the same path. The robot is the published `v7` base (scan plane 150 mm), not the
arm robot, as for all earlier scored tours. Each configuration has its own map, seeds 0–2, one run each, on the PC in
WSL (isolated workspace, ROS domain 61). Logs: `~/lidcmp` in WSL. Results:
`lidcmp_tour_<config>_s<seed>.json` and `ros_map_lidcmp_<config>.json` here.

| Config | Lidar | Stamps on /scan |
| --- | --- | --- |
| a2m12 | A2M12 datasheet model (the sim's before today): 1,600 samples at 10 Hz, at the turn's end | ideal (first sample's own time) |
| a2m8 | A2M8 Stability (the sim's default now): 320 samples at 12.5 Hz, re-binned, handed over 36–61 ms late | sllidar_ros2's (late by a delivery, ±capsule jitter) |
| a2m8true | the same | ideal |
| a2m8ref | the same (on the a2m8 map) | sllidar's, re-timed by scan_prep's `ScanClock` (`scan_refine:=true scan_latency:=0.0365`) |

## Localization (slam_toolbox on its map), per seed: median / p95 / max position error, p95 yaw error

| Config | Seed 0 | Seed 1 | Seed 2 |
| --- | --- | --- | --- |
| a2m12 | 2.8 / 8.4 / 69 cm, 0.70° | 2.8 / 8.4 / 83 cm, 0.53° | 3.1 / 9.4 / 75 cm, 0.68° |
| a2m8 (sllidar) | 7.4 / 16.8 / 35 cm, **9.9°** | 6.9 / 16.7 / 46 cm, **9.4°** | no run: slam_toolbox never started in 4 tries (below) |
| a2m8true | 5.8 / 15.1 / 42 cm, 1.53° | 5.3 / 16.6 / 58 cm, 1.52° | 5.2 / 18.6 / 49 cm, 1.64° |
| a2m8ref | 4.8 / 10.3 / 44 cm, 1.13° | 4.6 / 9.6 / 56 cm, 1.08° | 3.9 / 10.9 / 45 cm, 1.16° |

Means: A2M12 2.9 / 8.7 cm, 0.64°. A2M8 with the ROS driver's stamps 7.2 / 16.8 cm, 9.7°. A2M8 with ideal stamps
5.4 / 16.8 cm, 1.56°. A2M8 with re-timed stamps 4.4 / 10.3 cm, 1.12°.

The odometry-only EKF (`ros_odom`) is the same in every config (11–26 cm median, 71–96 cm p95: the wall push slips), as
it should be: it uses no lidar. The maxima (35–83 cm) come at the wall push for every lidar.

## Maps (`score_map`: fit of the map onto the house)

| Config | Cells | Coverage (anchored) | Best fit offset | Wall offsets |
| --- | --- | --- | --- | --- |
| a2m12 | 3591 | 1.000 | 2.0 cm, 0.12° | 1–26 mm |
| a2m8 (sllidar) | 3149 | 0.934 | 8.1 cm, 0.34° | 1–44 mm |
| a2m8true | 3295 | 0.969 | 4.2 cm, 0.44° | 1–41 mm |

## Findings

- **Yes, it matters.** With the ROS driver's own stamps, the A2M8 roughly doubles the position error (median 2.9 →
  7.2 cm, p95 8.7 → 16.8 cm) and multiplies the heading error ~15× (p95 0.64° → 9.7°). The map gets worse too (fit 2
  → 8 cm, coverage 100 → 93 %).
- **Most of the heading error is the stamps**, not the lidar. sllidar stamps a turn when the previous turn was handed
  over, 36–61 ms late in Stability and jittering by up to a capsule. The de-skew and slam_toolbox then pose each scan
  at the wrong time, and a turning robot moves ~2–6° in that time. With ideal stamps the yaw p95 is 1.5°.
- **What's left is the lidar itself:** 5× fewer points a turn (320 against 1,600), coarser range steps, and a seam of
  turn-start samples placed at the end of the turn. Median 5.4 against 2.9 cm with ideal stamps.
- **The fix exists and works:** scan_prep's `refine_stamps` (`ScanClock`) with a 36.5 ms latency brings the A2M8 to
  4.4 / 10.3 cm and 1.1°. That is about 1.5 cm and 0.5° worse than the old A2M12 model. That run localized on the
  sllidar-stamped map yet beat the ideal-stamp config on its own map; with one run per seed, the difference between
  those two is not explained. 36.5 ms is the sim's model (one capsule + transmission + USB); on the robot it is a
  calibration (omni_calib step 6).
- **Not yet tried:** Boost (640 points a turn, 12–23 ms delivery), a driver that stamps from the lidar's sample clock,
  Nav2 goals and the arm robot's own geometry, and the Pi.
- **The v9 shadow estimator** (`v9:wheels_imu_lidar`, sim-internal, true sample times) also degrades with the A2M8
  (median 1.2 → 5.3 cm, yaw p95 0.5° → 10°). It applies a scan's fix, matched at the turn's end, to the current state,
  so the delivery delay is error to it. It is a reference estimator, not robot code, and was not changed.

**Harness note.** slam_toolbox (Jazzy, `online_async_launch.py` / `localization_launch.py`) sometimes never configures
at launch (no "Configuring" / "Activating" lines; `save_map` waits forever, the tour scores no `ros_map`). The same
happened to the A2M12 (3 of 4 tries on one seed), so it is a launch race, not the lidar. The runs were retried until it
started; the a2m8 seed 2 tour never did in 4 tries. Fixed Nov 17: stack.py starts slam_toolbox with `autostart:=false`
and `omni_bringup lifecycle_up` takes it to active, checking its real state and repeating lost requests, so no retries
are needed (8 of 8 map/localize starts were active in 0.3–1.6 s).

## Nov 17: the robot's own driver (`omni_cpp` `rplidar_node`, stamps from the sample clock)

After this report the robot got its own lidar driver, stamping each sample from the device's sample clock
(characterization/lidar/README.md, "The robot's driver"), and the sim's A2M8 was brought in line with it: the head
turns clockwise as the real one (so /scan's array runs against time, scan_prep `time_order: -1`), turns are cut where
the driver cuts them (half a bin past 0 in bearing), the returns whose bearing lands across the seam are dropped, and
`lidar_stamp:=clock` (the new default) stamps as the driver does. Same setup as above: auto_map then 3 tours, own map.

| Config | Seed 0 | Seed 1 | Seed 2 |
| --- | --- | --- | --- |
| a2m8clk (clockwise, the driver's stamps) | 3.2 / 10.2 / 57 cm, 0.70° | 2.7 / 9.5 / 61 cm, 0.87° | 3.1 / 12.0 / 60 cm, 0.83° |

Mean 3.0 / 10.6 cm, yaw p95 0.80°: the old A2M12 model's median (2.9 cm) and nearly its heading (0.64°), a p95 2 cm
worse (8.7 → 10.6). Map (`ros_map_lidcmp_a2m8clk.json`): 2966 cells, coverage 0.995, best-fit offset 2.5 cm and
0.02°, walls 5–24 mm. Odometry (`ros_odom`) as before. slam_toolbox's start race hit twice (the map took 3 tries, the
seed 0 tour 2).

So with the driver the A2M8 costs almost nothing against the A2M12 model in this test, against 2.5× the median and
15× the heading error with sllidar's stamps. It also beats Nov 16's "ideal stamp" config (5.4 / 16.8 cm, 1.6°), the
result that report could not explain. The likely reason (not isolated): that config kept the seam, ~5–7 turn-start
samples a turn binned at the turn's end and so timed a whole turn (80 ms) off, which the driver's cut and seam drop
remove. Still sim only, on the v7 base and a fixed route; the driver has not yet run on the lidar.

## Nov 17: the bin count (empty bins and slam_toolbox's match scores)

The runs above publish one bin a sample (320 a turn), while the driver then published 360. The sim's A2M8 now bins as
the driver does (`lidar_bins`, robot.yaml's `rplidar` `bins`). Same setup, auto_map then 3 tours on its own map, except
the last row, which localizes on the 320-bin map. slam_toolbox started by `lifecycle_up`: every run first try.

| /scan bins | Empty bins | Seed 0 | Seed 1 | Seed 2 | Mean (med / p95, yaw p95) | Map cells, coverage |
| --- | --- | --- | --- | --- | --- | --- |
| 320 (`b0`, one a sample) | few | 3.2 / 9.8 / 60 cm, 0.72° | 2.9 / 9.5 / 61 cm, 0.87° | 3.2 / 12.6 / 58 cm, 0.82° | 3.1 / 10.6 cm, 0.80° | 2966, 0.995 |
| 360 (`b360`, the driver's until now) | ~10 % | 4.0 / 12.0 / 65 cm, 1.86° | 3.3 / 10.6 / 63 cm, 1.26° | 4.2 / 20.7 / 64 cm, 1.62° | 3.8 / 14.4 cm, 1.58° | 3183, 0.904 |
| 720 (`b720`) | ~55 % | 7.4 / 27.4 / 75 cm, 3.32° | 4.7 / 21.2 / 73 cm, 2.32° | 4.5 / 12.7 / 47 cm, 1.41° | 5.6 / 20.4 cm, 2.35° | 3299, 0.946 |
| 720 on the 320 map (`b720m0`) | ~55 % | 4.6 / 20.2 / 55 cm, 1.55° | 3.3 / 17.5 / 58 cm, 1.00° | 3.3 / 14.0 / 40 cm, 1.68° | 3.7 / 17.3 cm, 1.41° | (b0's) |

`b0` reproduces `a2m8clk` exactly. 720 bins were meant to halve the angle quantization (each return within 0.25° of its
bin, not 0.56°); instead the map and the localization got worse, on its own map and on a good one.

**A first explanation was wrong.** It read Karto's `ComputeOffsets` as checking the filtered points against the raw
beams by index; in fact `GetPointReadings()` defaults to the unfiltered points, one per beam, so the inf check is right.
The "fix" built on it (`fill` rows below: empty bins written as `range_max` + 1 m on a `/scan_slam` copy) made Karto
match those as points 13 m away, and was removed:

| Config (one bin a sample at 12.5 Hz unless noted) | Seed 0 | Seed 1 | Seed 2 | Mean | Map cells, coverage |
| --- | --- | --- | --- | --- | --- |
| fill, 320 bins (`fill320`) | 2.6 / 8.7 / 56 cm, 0.82° | 2.7 / 8.5 / 58 cm, 0.94° | 3.2 / 12.3 / 46 cm, 1.08° | 2.8 / 9.8 cm, 0.95° | 3111, 0.984 |
| fill, 360 bins (`fill360`) | 5.7 / 12.1 / 62 cm, 1.79° | 4.3 / 12.9 / 65 cm, 1.96° | 4.8 / 20.3 / 53 cm, 1.59° | 4.9 / 15.1 cm, 1.78° | 3368, 0.896 |
| fill, 720 bins (`fill720`) | 7.1 / 124.8 / 127 cm, 8.11° | 7.9 / 44.7 / 87 cm, 3.91° | 8.6 / 125.0 / 127 cm, 8.55° | 7.9 / 98.1 cm, 6.85° | 3433, 0.952 |

**The sim's 320 was too good.** The sim's turn holds a whole number of samples (4000 / 12.5 = 320), so with 320 bins
every turn's samples land at the same place in their bins: no quantization from turn to turn, which the real head (~327
samples a turn, Nov 17 on the Pi) never has. `lidar_hz:=12.232` (327 a turn, the samples drifting through the bins):

| Bins at 327 samples a turn | Mean (med / p95, yaw p95) | Map cells, coverage |
| --- | --- | --- |
| 327, one a sample (`r327b0`) | 4.5 / 9.6 cm, 1.22° | 3078, 0.941 |
| **320** (`r327b320`, the robot's) | **3.7 / 8.3 cm, 1.48°** | 3045, 0.942 |
| 360 (`r327b360`) | 3.6 / 8.5 cm, 1.04° | 3115, 0.998 |
| 720 (`r327b720`) | 4.6 / 11.4 cm, 1.92° | 3107, 0.861 |

Each row builds one map, and one map's quality moves the tours by about ±1 cm and ±0.4° (compare `r327b0` with
`r327b320`): 320 and 360 are alike; 720 is worse here as at 12.5 Hz.

**Why empty bins cost (from slam_toolbox's source, jazzy `lib/karto_sdk/src/Mapper.cpp`, `ScanMatcher::GetResponse`):**
a match's score sums the correlation grid under each beam's point, skipping empty beams, but divides by the number of
all beams. A scan's best score is therefore about its share of filled bins, judged against fixed thresholds
(`loop_match_minimum_response_coarse` / `_fine` 0.35 / 0.45 mapping, 0.55 localizing; `link_match_minimum_response_fine`
0.1). 720 bins for ~327 samples are ~40 % filled: loop closures rarely pass. A real room's no-return sectors (dark,
glossy, slanted, open) cap scores the same way: the Pi bench spot's scans were 66 % filled.

Adopted: `bins: 320` (cold, the head turns slower and gives up to ~338, so a few bins hold two returns, the nearer
kept). The sim's A2M8 bins as robot.yaml (test_v11 checks they agree). Realistic sim SLAM for the A2M8 is the
`r327b320` row; the default sim (12.5 Hz) stays optimistic until its turns can hold a fractional number of samples.
Results: `lidcmp_tour_{b0,b360,b720,b720m0,fill320,fill360,fill720,r327b0,r327b320,r327b360,r327b720}_s{0,1,2}.json`,
`ros_map_lidcmp_*.json`.
