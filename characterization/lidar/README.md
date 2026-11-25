# Lidar bench: the RPLIDAR A2M8

This bench measures the robot's lidar so the simulation's model can follow it. Since Nov 16 **the A2M8 is the baseline**
(Fernando: it is the unit he has; design.md 2). Since Nov 16 the v11 sim's lidar is an A2M8 built from
these measurements (`simulation/v11 sil/sensors.py` `SensorConfig.parts(lidar="A2M8")`; `lidar="A2M12"`, the old datasheet
model, reproduces earlier runs). The section "For the simulation" lists what the measurements changed.

**Unit:** RPLIDAR A2M8, firmware 1.26, hardware 5, health good. It sits on its CP2102 USB adapter at 115200 baud on the
Pi 4 (`omni-cam`, rpi.md), through the VIA USB 2 hub. It lists four scan modes:

| Mode | Samples/s | Answer format | Samples a packet | Packet bytes |
| --- | --- | --- | --- | --- |
| Standard | 2000 | 0x81 standard | 1 | 5 |
| Express | 4000 | 0x82 express capsule | 32 | 84 |
| Boost | 8000 | 0x84 ultra capsule | 96 | 132 |
| **Stability** (the device's "typical") | 4000 | 0x84 ultra capsule | 96 | 132 |

Every test runs the robot's own driver, `ros/src/omni_console/omni_console/rplidar.py` (the console's Test › Lidar).
It opens, spins up and scans the lidar exactly as the console does. Only the reader is swapped for one that keeps every
byte with its arrival time on the Pi (CLOCK_MONOTONIC; the Pi is NTP-synced). The analysis decodes offline with the
same driver's parsers, so a capture can be re-analysed later.

```
bash characterization/lidar/lidar.sh capture SESSION [--plan MODE:PWM:SECONDS,...] [--gap 5]   # background, on the Pi
bash characterization/lidar/lidar.sh status SESSION
bash characterization/lidar/lidar.sh fetch SESSION          # raw -> C:/Users/ferna/lidar_data/SESSION (outside Dropbox)
C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe characterization/lidar/lidar_timing.py C:/Users/ferna/lidar_data/SESSION
```

The capture refuses to run while anything else holds the port, for example the console's Lidar page or
`camera_snap`'s boot self-test. Raw captures stay in `~/lidar_data` on the Pi and `C:/Users/ferna/lidar_data` here;
`results/` holds only the analysis JSON.

## Test plan (static, from the design review of Nov 16)

| # | Test | Setup | Status |
| --- | --- | --- | --- |
| 1 | Timing and stability: rate, samples a turn, angle spacing, the sample clock, delivery, spin-up, PWM | None | **Done Nov 16** (below) |
| 2 | Range noise and bias against range, for each mode | Walls at 1–7 m seen from one spot (or a wall stepped along a tape) | **Done Nov 17** (below); bias only bounded (< 2.5 cm, rough tape) |
| 3 | Surface reflectivity and incidence angle: dropouts and bias | Material samples at ~1.5 m; white and black boards turned 0–80° | To do (attended) |
| 4 | Mixed pixels at depth edges | A box edge 1 m in front of a wall at ~2 m | To do (attended) |
| 5 | This unit's scan-plane tilt and beam height | Levelled lidar; a box edge raised on shims at 0.5 and 3 m, 4 headings | To do (attended) |
| 6 | Minimum range | A target slid in from 0.5 to 0.05 m | To do (attended) |
| 7 | Warm-up drift of range | Wall, 30 min from cold | To do (test 1's cold run covers the turn rate only) |
| 8 | Interference from the D435's projector | Both on one wall, projector off / on | To do (attended) |

## Test 1: timing and stability (Nov 16; hardware, static)

**Conditions:** the lidar sat still on the desk, seeing the room (median range ~0.4 m, 63–72 % of samples returned). The
Pi was wired, NTP-synced, at 0.2–1.0 load average with the robot step running (`hardware.launch.py`, mcu_link on the
Teensy). The lidar had been off since boot. Each run started from a stopped motor after a 5 s gap, through the driver's
own start: a 1 s soft ramp from PWM 200, then the scan request. Session `timing_2026-11-16`: a 5 min cold run in
Stability, then 30 s in each mode at PWM 660, then 20 s in Stability at each of PWM 300–1023. There were no framing
errors in any run (0 bytes skipped).
Result: [results/timing_2026-11-16.json](results/timing_2026-11-16.json).

| Mode, PWM 660 | Turn rate | Turn-to-turn period std | Samples a turn | Mean angle step | Link use at 115200 | Turn ready after its last sample |
| --- | --- | --- | --- | --- | --- | --- |
| Standard | 12.35 Hz | 0.27 ms (0.34 %) | 162 | 2.23° | 87 % | 0.1–1.1 ms |
| Express | 12.46 Hz | 0.17 ms (0.21 %) | 321 | 1.12° | 91 % | 8–17 ms |
| Boost | 12.50 Hz | 0.07 ms (0.09 %) | 640 | 0.56° | **96 %** | 12–23 ms |
| Stability | 12.50 Hz | 0.26 ms (0.32 %) | 320 | 1.12° | 48 % | 24–49 ms |

The turn rate is from the lidar's own sample clock (samples in the turn × the sample period). The period's resolution is
one sample (0.125–0.5 ms), so the std is partly quantization. "Turn ready" is measured above the fastest delivery and
excludes a fixed delay that a static test cannot see (below).

**Findings:**

- **The sample clock is exact.** The fitted sample period is within ±10 ppm of nominal in every run (125 / 250 / 500 µs,
  against the Pi's NTP clock). Samples per second are a fixed property of the mode; the turn rate is not.
- **The turn rate follows the PWM with no closed loop, and it drifts.** Warm, 20 s after start in Stability:

  | PWM | 300 | 400 | 500 | 660 | 800 | 1023 |
  | --- | --- | --- | --- | --- | --- | --- |
  | Hz | 4.85 | 7.00 | 9.17 | 12.50 | 15.56 | 21.51 |

  About 10 Hz needs PWM ≈ 540 (interpolated, not run). From a cold start at PWM 660, the rate was 11.83 Hz at 5 s,
  12.05 at 30 s, 12.20 at 2 min and 12.38 at 5 min, still rising 0.06 Hz/min. Warm runs settle within ~10–20 s: about
  9.5–9.9 Hz at 2 s from the motor's start, 11.3 at 3 s, 12.2–12.3 at 5 s and 12.46 at 10 s. The scan's first bytes
  arrive 1.7–2.0 s after the motor command (the driver's 1 s ramp, then the device). These rates differ from rpi.md's
  quick Nov 11 reading (6.0 Hz at 300, 20.0 at 1023), which was taken without a warm-up.
- **Turn to turn the period is steady.** The std is 0.07–0.5 ms (CV 0.09–0.44 %), which confirms
  `omni_perception/scan_time.py`'s "< 1 %" estimate. Inside a turn the head's encoder angle stays within
  0.11–0.18° rms of a constant speed: ~0.03 ms at 12.5 Hz. The constant-rate sample times that `scan_time.py` assumes
  are good to well under a millisecond. A small speed wobble is fixed to the head and repeats in every run: about
  −0.16° near 60–90° and +0.24° near 210–240°, i.e. ≤ 0.06 ms of time.
- **Angles are not in time order.** The device corrects each sample's angle for triangulation parallax, by an amount
  that depends on its range. Consecutive samples step from about −6° to +9° (median steps as above), with 12–44
  backward steps a turn, and a turn's first sample reads −5.4° to −8.5° instead of 0°. Anything that assumes the array
  index is monotonic in angle (a LaserScan with a constant `angle_increment`) needs the driver's angle re-binning
  (sllidar's `angle_compensate`). Each re-binned sample's time is still its index's time, within ~±2 ms (±9° at
  12.5 Hz): VERIFY with the ROS driver.
- **Capsule modes deliver a turn late, in capsule-sized steps.** A capsule is decoded only when the next one arrives,
  because its sample angles are interpolated up to the next start angle. This is in Slamtec's protocol, so sllidar's
  SDK does the same. So a turn's last sample is usable after the rest of its capsule plus all of the next one. A driver
  that stamps a turn when it becomes available therefore jumps by up to a capsule from turn to turn. In Stability its
  stamp-to-stamp period was off by up to 23 ms (p95 13–22 ms in six of seven runs). The size depends on how the turn's
  sample count falls against the 96-sample capsule. In Boost it was off by up to 8 ms, in Express mostly ≤ 1 ms (max
  8 ms), and in Standard by ≤ 1 ms. The
  per-packet USB delivery itself is steady: p95 ≈ 1.0 ms, max 1.0–2.5 ms, and one 21 ms outlier in 315,000 reads over
  5 min. Not measured (needs motion: omni_calib step 6) is the fixed delay. It includes at least the packet's own
  transmission at 115200 baud, 11.5 ms for a 132-byte capsule and 0.4 ms for a 5-byte sample, plus the device's
  processing.
- **Boost uses 96 % of the 115200 baud line** (Express 91 %, Standard 87 %, Stability 48 %). There were no errors in
  30 s, but Boost has no margin, which the A2M12's 256000 baud would have had.
- **Pi cost:** the capture's reader was negligible beside the robot step; load average was 0.2–1.0 throughout.

**Choosing the robot's mode (test 2 below favours Stability; test 3 to confirm):** Boost gives twice the samples
(0.56° at 12.5 Hz) and arrives sooner than Stability, but fills the line. Stability is the device's own default, with
half the samples and the longest delay. Express's capsules are the shortest but it fills the line nearly as much as
Boost. Standard arrives sooner than all of them but has the coarsest spacing.

## Test 2: range noise, mode offsets and dropouts on walls (Nov 17; hardware, static)

**Setup:** the lidar stood still in a room with walls at 1.0–7 m around it (Pi on Wi-Fi, just rebooted; the lidar had
spun ~1 min in the previous few minutes, so not a cold start). Session `walls_2026-11-17`: 300 s of Stability (warm-up),
then 60 s each of Stability, Boost, Express and Standard at PWM 660, 2 s apart. Analysis:
[lidar_range.py](lidar_range.py) finds the straight walls in the median scan (13 segments, 0.24–2 m long, 1.0–7.0 m
away; `walls.png` beside the raw data), fixes each wall's direction there and measures every run's samples against it.
Result: [results/range_walls_2026-11-17.json](results/range_walls_2026-11-17.json).

```
python characterization/lidar/lidar_range.py C:/Users/ferna/lidar_data/SESSION [--truth WALL:METRES,...]
```

**Range noise** (robust std of a sample's range about the wall's own mean shape, i.e. turn to turn; walls seen at
< 50° incidence, pooled by range):

| Range | 1.0–1.5 m | 1.5–2.0 m | 2.0–2.5 m | 2.5–3.0 m | 4.5–5.0 m | 7.0–7.5 m |
| --- | --- | --- | --- | --- | --- | --- |
| Stability | 1.9–2.1 mm | 3.2–3.3 | 4.5–4.7 | 9.5–9.8 | 12.2–12.6 | 36–38 |
| Boost | 1.2 | 2.2 | 4.0 | 6.2 | 11.8 | 30 |
| Express | 1.6 | 3.1 | 5.4 | 7.6 | 16.8 | (64 % dropped) |
| Standard | 1.6 | 3.0 | 5.2 | 6.4 | 16.6 | (63 % dropped) |
| Sim model (A2 datasheet, accuracy / 2.5) | 5 | 7 | 9 | 11 | 38 | 73 |

**Dropouts** (samples on the wall with no return): Stability 0 % on every wall, including 7 m and 62° incidence
(0.4–0.7 % on two). Boost loses 12 % at 5 and 7 m, 8 % on a 2.2 m wall and 18 % at 62° incidence; Express and
Standard 21–30 % at 5 m, 63 % at 7 m and 19 % at 62°.

**Findings:**

- **Noise is millimetres and grows faster than range:** about 2 mm at 1.2 m, 5 mm at 2.2 m, 12 mm at 5 m and
  37 mm at 7 m in Stability. Slanted walls are noisier at the same range (5 mm at 1.4–1.6 m and 41–63°, against 2 mm
  head on). The sim's datasheet model is 2–3× too noisy up to 5 m and has 3 % random dropouts that Stability
  doesn't show: **to update in the sim after test 3** (with its dropouts by surface).
- **Stability trades a little noise for no dropouts.** Boost's samples are slightly less noisy and twice as many, but
  it drops far, slanted or dim returns. Empty bins cost slam_toolbox's match scores (the bin count, below). So
  **Stability stays the robot's mode**, pending test 3 (dark and glossy surfaces will make the difference bigger).
- **Each mode's scan is rotated against Stability's**, the same on every wall at 1–2 m: Boost by −1.5° (−1.0 to
  −1.9), Express and Standard by −3.1° (−3.0 to −3.5; device clockwise, so the scan turns counterclockwise).
  Without the rotation the modes agree on range to a few mm (medians +1.6 mm Express and Standard, −1.2 mm Boost).
  A constant rotation (not one that shrinks with range) rules out a parallax sign error in the decode; it is a
  per-mode angle reference or delay in the device (3.1° is 0.7 ms at 12.5 Hz). Which mode's 0° is the true 0° mark
  needs the 0° mark check (a target at a known bearing). **base_link → laser's yaw therefore belongs to the mode:**
  calibrate it in the robot's mode and calibrate again if the mode changes.
- **Range drift while warming:** under 1 mm over the 5 min run on the walls within 2.2 m; the 5 m wall moved 12 mm
  in the first minute, then held (one wall: unconfirmed).
- **Bias against true distance: within 2.5 cm, finer not resolved.** Fernando's rough tape readings to the three
  room walls (40, 55 and 70.5 in from the head's centre; `--truth 1:1.016,13:1.016,9:1.7907,11:1.397`): the lidar
  read +13, +4 and +24 mm long. The readings carry about an inch, so this bounds the bias but doesn't measure it; a
  constant offset would go into base_link → laser anyway. To measure it: a wall stepped along a tape to the mm.
  Yesterday's capture had one surface at 0.28 m, where Boost read 9 mm long and
  Stability's range moved with the motor's speed by ~1.4 mm per Hz (PWM 300–1023); at 1–2 m today the modes'
  offsets are within a few mm.

## In the simulation: the `A2M8` part (Nov 16)

The v11 sim's default lidar is now `SensorConfig.parts(lidar="A2M8", lidar_mode="Stability")`; `sensors.py`'s
docstring has the details. The ROS steps take `lidar:=a2m8|a2m12`, `lidar_mode:=stability|boost|express|standard` and
`lidar_stamp:=clock|sllidar|true` (`clock`, the default since Nov 17: as the robot's driver stamps, below).
`lidar:=a2m12` reproduces earlier runs: 16 k samples/s, exactly 10 Hz, even angles and ideal stamps. Pass
`map:=house_a2m12` with it, the map it was built on; since Nov 24 `maps/house` is the A2M8's (design.md 4.6).

| Modelled | From |
| --- | --- |
| Samples/s by mode; 12.5 Hz at PWM 660 (`A2M8_PWM_HZ` for others), 320 samples a turn in Stability | Test 1 |
| Each sample at its hit's bearing: the encoder's angle − 8° + the parallax (b = 24.0 mm), re-binned by angle with each bin's own sample time. Since Nov 17 the head turns clockwise as the real one, so the bins run against time, and a turn is cut where the driver cuts it (half a bin past 0 in far-field bearing) | Slamtec's SDK correction; test 1's turn start and angle steps; `rplidar.hpp` |
| Delivery: the rest of the capsule plus the next capsule, plus the packet's transmission at 115200 baud, plus USB (0.5 ms fixed, est; 0–1 ms jitter); the turn's alignment with the capsules follows a sample count that wanders ~0.8 a turn | Test 1 |
| Stamps (`lidar_stamp`): `clock` as `rplidar_node` gives them (bin n − 1's time from the turn's own sample times; seam crossers dropped); `sllidar` as sllidar_ros2 would (the previous handover, `scan_time` the time between handovers); `true` the first sample's time | `rplidar.hpp`; sllidar_ros2's source |
| Range quantization: Standard 0.25 mm, Express 1 mm, Boost and Stability 1/2/4/8 mm by band | Test 1's data and the protocol |
| Range noise, dropouts (3 %), tilt (≤ 1.5°), range 0.15–12 m | Still the datasheet and estimates (tests 2–6) |

Not modelled: the 0.1–0.4 % turn-to-turn jitter, the ~0.03 ms speed wobble inside a turn, and the cold-start creep.

**Check against the bench** ([sim_check.py](sim_check.py), a still robot in the house's hallway, 15 s per mode;
Nov 16, before the direction change; `sim_check.py` runs with `lidar_stamp="sllidar"`, what the bench measured):

| Mode | Turn ready after its last sample, p0 / p50 / p95 / max (ms), sim vs bench | Stamp-to-stamp error, p50 / p95 / max (ms), sim vs bench |
| --- | --- | --- |
| Standard | 0.0 / 0.5 / 1.0 / 1.0 vs 0.1 / 0.6 / 1.0 / 1.1 | 0.3 / 0.8 / 1.0 vs 0.1 / 0.5 / 0.9 |
| Express | 8.2 / 10.6 / 15.8 / 16.7 vs 8.3 / 12.5 / 16.1 / 16.8 | 0.3 / 0.9 / 8.5 vs 0.3 / 1.0 / 7.8 |
| Boost | 13.3 / 19.1 / 23.5 / 23.9 vs 12.4 / 17.8 / 22.5 / 22.9 | 4.3 / 8.3 / 8.8 vs 4.1 / 8.0 / 8.1 |
| Stability | 25.2 / 36.9 / 46.3 / 47.6 vs 24.2 / 37.9 / 47.8 / 48.6 | 8.3 / 16.4 / 17.1 vs 8.1 / 16.2 / 16.3 |

Rerun Nov 17 with the clockwise head and the driver's turn cut ([results/sim_check_2026-11-17.json](results/sim_check_2026-11-17.json)):
delivery and stamp error unchanged within the run-to-run spread (Stability 25.1 / 34.8 / 43.7 / 45.0 and 8.4 / 16.4 /
17.0 ms). `first_deg` now reads 1.1° against the bench's −6° by design: the sim's turns start where the driver cuts
them, the bench's at the device's own start flag.

**SLAM with it** (slam_toolbox, house tour, 3 seeds): with sllidar's stamps, much worse than the old A2M12 model
(yaw p95 9.7° against 0.6°); with the robot's driver's stamps (Nov 17), 3.0 / 10.6 cm and 0.8°, about the A2M12's.
[Report](../../simulation/v11%20sil/studies/results/lidar_a2m8_slam_2026-11-16.md).

Rates, samples a turn and mean angle steps agree too. The angle steps' spread and backward steps do not, because they
depend on the scene. The parallax is 24 mm / range, so the bench's clutter at a median 0.4 m gives steps of −4 to +6°
and ~19 backward a turn (Stability), where the hallway's walls at a median 1.5 m give 0.9–1.7° and 2.

## The robot's driver: `omni_cpp` `rplidar_node` (Nov 17; implemented, not yet run on the lidar)

The device sends no times, and sllidar_ros2 stamps a turn when the previous one was handed over (36–61 ms late in
Stability, jittering by up to a capsule), which cost slam_toolbox most of its heading accuracy in the sim. So the robot
has its own driver, [rplidar.hpp](../../ros/src/omni_cpp/include/omni_cpp/rplidar.hpp) (ROS-free core) and
`rplidar_node.cpp`, the console's protocol in C++:

- **Stamps from the sample clock.** A sample's time is its count: `t(n) = line(n + 1) − delay`, where `line` is the
  lower envelope of every packet's arrival against its sample count (42 packets a second in Stability) and `delay` the
  packet's transmission (132 × 10 / 115200 s) plus `latency` (0.5 ms est, a parameter; omni_calib measures it). USB
  and read jitter, the motor's open-loop speed and its creep don't move the stamps.
- **Lost packets** are counted from the gap in the capsules' encoder angle; a gap it can't count (no data for 0.5 s)
  starts a new line. A turn's samples are timed again when it closes, so a just-begun line doesn't spoil it.
- **The grid:** REP-103 in `laser` (x at the lidar's 0° mark), bin i at i × 2π / bins (320 bins since Nov 17: the
  warm turn's samples; see the bin count below); the head turns
  clockwise, so the array runs against time: bin bins − 1 at the stamp (scan_prep `time_order: -1`). Turns are cut
  half a bin past 0 in bearing (the encoder plus the measured mean offset, −8° and the parallax), and a return whose own
  bearing lands across the seam is dropped (0.2 % on the bench recording).
- **The node:** port exclusive, DTR cleared, info / health / GET_LIDAR_CONF modes, a soft motor start, then the scan;
  it retries every 2 s on any failure. Parameters in `robot.yaml` (`rplidar`): `mode` Stability, `pwm` 660 (live),
  `bins` 320, `latency` 0.0005 (live). `/lidar/status` (1 Hz JSON): rate, samples a turn, bytes skipped, packets lost,
  clock lines, ppm, the envelope's spread. `hardware.launch.py lidar:=true` starts it (off by default until it has run
  on the lidar).

**Checks (no lidar needed):**

| Check | Result |
| --- | --- |
| Decode against `rplidar.py` on 2–6 s of each mode's bench recording (`omni_cpp/test/data/a2m8_*.npz`) | Identical samples in all four modes, every sample counted |
| Stamps on the bench's 6 s of Stability as the Pi read it | Every arrival on or above the line; envelope spread 0.03 ms; stamp to stamp = the turn's own scan time within 0.08 ms once warm (1.1 ms while the motor still speeds up) |
| Stamps against known sample times (synthetic Express: USB jitter, late reads, a lost capsule, garbage, a corrupted capsule, a 1.2 s stall) | Every sample within 0.2 ms up to the stall (median < 0.05 ms); every turn's stamp within 0.25° of its true angle, after the stall too |
| The node in WSL against [a2m8_replay.py](../../ros/tools/a2m8_replay.py) (a pseudo-terminal answering as the A2M8 and replaying the 30 s Stability run at its recorded times) | 12.4 Hz, 360 bins (then the default); stamp to stamp = scan time, median 0.02 ms; /scan reaches subscribers 37–61 ms after the turn's end (the capsule wait: stamps right, delivery unchanged); one clock line, no lost packets; reconnects when the stream ends |

**On the A2M8 (Nov 17, Pi 4, attended).** Built in `~/omni_ws_claude` on the Pi (rplidar_node only, over the deployed
`~/omni_ws`), run with robot.yaml's `rplidar` values on ROS domain 61 beside the console's robot step; `/scan` and
`/lidar/status` recorded by a subscriber on the Pi. Stability from a cold motor 150 s, a restart, then 30 s each of
Boost, Express and Standard. Raw records: `C:/Users/ferna/lidar_data/lidar_2026-11-17_1820/`; summary:
[results/driver_pi_2026-11-17.json](results/driver_pi_2026-11-17.json).

| Mode | Samples / turn | Stamp to stamp − scan time (p50 / p95 / max) | /scan after the turn's end (min / p50 / max) | CPU (one core) |
| --- | --- | --- | --- | --- |
| Stability (cold, 150 s) | 327 | 0.03 / 0.11 / 0.99 ms | 36 / 49 / 62 ms | 3.3 % |
| Express | 324 | 0.04 / 0.09 / 0.90 ms | 17 / 21 / 26 ms | 5.5 % |
| Boost | 651 | 0.04 / 0.11 / 0.92 ms | 25 / 31 / 38 ms | 5.6 % |
| Standard | 162 | 0.42 / 0.50 / 2.3 ms | 2 / 3 / 5 ms | 5.8 % |

- Every run: A2M8 firmware 1.26 hardware 5, health good, 0 bad bytes, 0 lost packets, one clock line, no breaks; the
  restart reopened the port at once. The cold motor's rate matched test 1: 11.84 Hz at 5 s, 12.04 at 30 s, 12.19 at 2
  min.
- Against sllidar's stamps on the same lidar (test 1: 8.4 / 16.4 / 17.0 ms) the stamps are ~250× steadier. The /scan
  delay is the device's packet wait, as test 1 measured (the stamps don't depend on it).
- The ~1 ms worst gaps come in pairs every ~7 s in every mode, not at spin-up: most likely the clock line stepping
  when the fastest arrival in its 10 s window leaves it (the envelope's spread here was 0.2–0.55 ms, against 0.03 ms on
  the bench recording). 1 ms is 0.12° at 120°/s. A smoother envelope would remove it (not done).
- Standard's stamps sit a constant 0.42 ms (~one of its samples) off its scan times: a small offset where the 320 bins
  outnumber its 162 samples. Not the robot's mode; not fixed.
- `bins` 320 suits Stability and Express (324–327 samples); Boost's 651 a turn want 640 (two samples a bin otherwise,
  the nearer kept), Standard's 162 leave half the bins empty.
- This bench spot leaves a third of the bins empty (finite 0.66), mostly sectors at 170–290° (the bench's clutter;
  median range 0.37 m), with ~6 % scattered single empty bins. Empty bins lower slam_toolbox's match scores (below), so
  a real room's no-return sectors matter as much as the bin count.

**The bin count (Nov 17, sim).** slam_toolbox's scan matcher (Karto, `ScanMatcher::GetResponse`) sums the map's
occupancy under each beam's point, skipping empty (+inf) beams, but divides by the number of all beams. A scan's best
score is therefore about its share of filled bins, and its matches are judged against fixed thresholds (loop closure
0.35 / 0.45 mapping, 0.55 localizing). Half-empty bins (720 for ~327 samples) cap every score near 0.4–0.5: few loop
closures. In the house tour (3 seeds each, the head at the real ~327 samples a turn,
[report](../../simulation/v11%20sil/studies/results/lidar_a2m8_slam_2026-11-16.md)): 320 bins 3.7 / 8.3 cm, 1.5°;
360 3.6 / 8.5 cm, 1.0°; 720 4.6 / 11.4 cm, 1.9°. 320 and 360 differ by less than one map's run-to-run spread (about
±1 cm, ±0.4°); 720 is worse at both turn rates tried. So `bins` stays 320, about the turn's samples at PWM 660 (a
cold, slower head gives up to ~340: a few bins hold two, the nearer kept); move it with `pwm`.

The same arithmetic applies to a room's real no-return sectors: the bench spot's 66 % filled bins cap a match near
0.66, close to localization's 0.55. Test 3 (dark, glossy and slanted surfaces) measures how full a real room's scans
are. Not done: an empty bin written as a finite range past `range_max` (tried Nov 17) is worse, because Karto then
matches it as a point (720 bins: 7.9 / 98 cm, 6.9°).

**Still to do on the lidar (attended, Pi on):** deploy the driver with the robot's workspace (`pi.sh build`); calibrate `latency` (omni_calib temporal: the
lidar's yaw against the IMU's in a spin); confirm the lidar's 0° mark faces `laser`'s x on the robot (VERIFY: the
mount); add base_link → laser to the hardware launch.
