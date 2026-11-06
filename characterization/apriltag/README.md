# AprilTag lock bench

How well the real cameras lock on to an AprilTag, for the tag-augmented SLAM (design.md 4.6, 4.11). The bench
reports four things:

- the noise in the forms the SLAM's models use: the corners' covariance, the frame-to-frame correlation and the
  6-DoF pose covariance;
- the detection rate, the decoder margin and misreads;
- the detector's cost on the Pi 4;
- an emulated reach.

It uses the robot's own pieces. The cameras run through `omni_console/camera_snap.py`, and detection and pose through
`omni_console/apriltag_view.py`, which calls `omni_slam.tags.tag_pose` and its gates. A pose the bench calls usable is
one `tag_slam` would accept.

```
bash characterization/apriltag/tl.sh run --size 0.14 --tape 1.35 --seconds 25 --light "words" [--only NAME...]
python characterization/apriltag/sim_inputs.py RUN.json [RUN.json ...]     # sim venv: results/sim_inputs.json
```

`tl.sh` copies the files to `~/omni_ws/apriltag` on the Pi and stops the Cameras tab's server (`cam_snap.sh`), which
holds the cameras. It then runs `tag_lock.py` and fetches `results/tag_lock_<PC time>.json`; the Pi's clock has no
NTP, so the PC's time names the file. Prefix `OMNI_PI=omni-cam-eth` to use the cable. Restart the server afterwards
with `bash ros/tools/cam_snap.sh start`. The configurations are in `tag_lock.py` `CONFIGS`. `sim_inputs.py` takes the
newest run of each configuration among the files given and writes one entry per camera solution
(section "For the simulation").

The same detection runs live in the console's **Cameras** tab: switch on *AprilTags*, then use *Live video*. The
overlay draws the outline, axes and range: green when usable by the SLAM, orange when ambiguous, red when refused. A
table under each picture gives the pose, the SLAM's verdict and the lock's scatter over the last 5 s.

## Setup (Nov 4; one still scene, two lights)

- **Tag:** tag25h9, ID 0, printed and taped flat to a wall, about 8° off-axis to both cameras. Its black square is
  **0.14 m**, derived from the D435's depth and Fernando's rough tape (~1.35 m, 4 ft 5 in, from the D435). Fernando
  gave 200 mm, which may include the white border; **VERIFY** with a ruler. The ranges here are a sanity check. The
  scatter is the point: a wrong size scales every range and position, but not the pixel noise or the angles.
- **Cameras:** the Arducam IMX708 and the D435 sat still on the desk, aimed separately, both about 1.34 m from the
  tag. The detector was the C apriltag library with 2 threads, on the Pi 4 over wired Ethernet.
- **Good light (the representative one):** Fernando's room lit for the evening, run
  `results/tag_lock_20261104_214348.json`. The Arducam's auto exposure held 32.7 ms at gain 3.1 (it keeps the
  longest exposure the frame period allows, then sets gain); the D435 IR ran at 32.8 ms. During the first D435 IR
  run and the 848 px run the camera moved: the whole tag drifted ~2 px across the picture (100% of the corner
  scatter shared by the four corners, frame-to-frame r 0.99). Those runs were repeated without drift in
  `results/tag_lock_20261104_215027.json`, which is used for them.
- **Dim light (comparison):** the same room at night, runs `tag_lock_20261104_2110.json` and
  `tag_lock_20261104_212411.json`. The Arducam ran at its maximum gain of 16 with the same 32.7 ms exposure.

## Results in good light

*Corners* is each corner's scatter about its mean, in px RMS per coordinate. *Sim* is `tag_sim`'s model at that side
length, σ = 0.25 + 4 / side px. The dim-light columns compare the same configuration in the night runs.

| Configuration | Picture | Found | Detect ms (tag rate) | Side px | Margin (dim) | Corners px (dim) / sim | Range ± mm | Normal ± ° |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Arducam full field, decimate 2 | 1152×648 | 394/394 | 31 (16 Hz) | 89 | 68 (57) | 0.045 (0.083) / 0.30 | 0.40 | 0.25 |
| Arducam full field, full resolution | 1152×648 | 171/171 | 113 (7 Hz) | 89 | 69 (43) | 0.021 (0.080) / 0.30 | 0.17 | 0.12 |
| Arducam 2304 px, decimate 2 | 2304×1296 | 133/133 | 142 (5 Hz) | 177 | 69 (56) | 0.046 (0.21) / 0.27 | 0.17 | 0.10 |
| Arducam, **OpenCV aruco** | 1152×648 | 176/176 | 88 (7 Hz) | 89 | – | 0.064 (0.11) / 0.30 | 0.59 | **1.65**, 15% ambiguous |
| Arducam 2/3 crop (100 fps mode; 9.5 ms, gain 9.9) | 768×432 | 290/290 | 50 (12 Hz) | 89 | 66 (**18**) | 0.040 (0.068) / 0.30 | 0.33 | 0.18 |
| D435 IR 1280, projector off, decimate 2 | 1280×720 | 402/402 | 26 (16 Hz) | 67 | 62 (72) | 0.031 (0.13) / 0.31 | 0.28 | 0.25 |
| D435 IR 1280, projector off, full resolution | 1280×720 | 243/243 | 67 (10 Hz) | 67 | 57 (71) | 0.030 (0.048) / 0.31 | 0.28 | 0.28 |
| D435 IR 1280, **OpenCV aruco** | 1280×720 | 181/181 | 80 (7 Hz) | 67 | – | 0.030 (0.09) / 0.31 | 0.49 | 0.36, 7% ambiguous |
| D435 IR 848 (60 fps; 15.4 ms), full resolution | 848×480 | 310/310 | 40 (12 Hz) | 44 | 60 (44) | 0.027 (0.045) / 0.34 | 0.59 | 0.43 |
| D435 IR 640, full resolution | 640×480 | 411/411 | 27 (16 Hz) | 40 | 59 (67) | 0.021 (0.047) / 0.35 | 0.58 | 0.40 |
| D435 IR 1280, **projector on** | 1280×720 | **359/510** (dim: 0/182) | 14 | 67 | 62 | 0.045 / 0.31 | 0.42 | 0.54 |
| D435 colour, decimate 2 | 1280×720 | 308/308 | 43 (12 Hz) | 95 | 53 (40) | 0.056 (0.091) / 0.29 | 0.39 | 0.38 |

**Accuracy:** the D435 IR pose read 1.340–1.344 m and the colour pose 1.354 m, against ~1.35 m by tape. The D435's
depth over the tag (projector on) read 1.364–1.372 m. The Arducam's range uses its lens-spec focal length (846 px at
1152 px wide) and reads 1.340 m; its scale is **unchecked**, since nothing measured its distance.

## Covariance: what the SLAM's noise models need

`omni_slam.graph` models a tag observation as its four corners. Each corner coordinate gets independent noise of
`sigma_px` = 0.7 px (Huber at 3σ), and every frame is treated as independent of the last. `tag_lock.covariance`
measures how far that holds for a still tag. In `results/sim_inputs.json` each camera's `cov` block has:

- **Corners:** σ per coordinate (u, v), and the share of their variance in a shift all four make together. With
  independent noise that share is **25%**; above it, the four corners move as one.
- **Frame-to-frame:** the lag-1 correlation r, and the share of frames that count as independent, (1 − r) / (1 + r).
- **Pose:** the 6-DoF covariance (`pose_cov_m_rad`: x, y, z in m, then rotations about the camera's x, y, z in rad;
  camera frame x right, y down, z ahead) and its correlations. Position is also split into along and across the view
  ray.
- **Prediction:** the pose σ predicted from the corner σ through the projection, σ² (JᵀJ)⁻¹, for both the measured σ
  and the graph's 0.7 px. Their ratio is *measured / predicted*.

The computation was checked on synthetic corners. Independent noise of 0.1 px came back as σ 0.100, 26% shared,
r 0.03, and measured / predicted 0.97–1.1. A shared, slowly wandering shift came back as 82% shared with r 0.63.

| Camera (good light) | Corner σ px (u, v) | Shared (25% = none) | Frame r (independent) | Position σ mm: along / across the ray | Rotation σ °: tilt x, y / roll z | Measured / predicted (x y z, rx ry rz) | Graph's 0.7 px implies: range mm, tilt ° |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Arducam 1152, decimate 2 | 0.045 (0.039, 0.050) | 52% | 0.29 (55%) | 0.40 / 0.058 | 0.19, 0.17 / 0.032 | 0.9 1.2 0.6, 0.8 0.7 0.7 | 9.8, 3.9 |
| Arducam 2304, decimate 2 | 0.046 | 70% | 0.24 (61%) | 0.17 / 0.035 | 0.08, 0.06 / 0.015 | 0.9 1.1 0.5, 0.6 0.5 0.6 | 4.9, 2.0 |
| Arducam 2/3 crop | 0.040 | 69% | 0.20 (67%) | 0.33 / 0.062 | 0.14, 0.12 / 0.027 | 1.1 1.3 0.6, 0.6 0.5 0.7 | 9.8, 4.0 |
| D435 IR 1280, decimate 2 | 0.031 (0.033, 0.029) | 61% | 0.46 (37%) | 0.28 / 0.058 | 0.12, 0.22 / 0.019 | 1.4 1.5 0.5, 0.5 0.9 0.7 | 13, 5.4 |
| D435 IR 848, full resolution | 0.027 | 19% | 0.04 (92%) | 0.59 / 0.047 | 0.26, 0.34 / 0.047 | 1.0 1.0 0.8, 0.9 1.1 1.2 | 20, 7.8 |
| D435 IR 640, full resolution | 0.021 | 23% | 0.08 (85%) | 0.58 / 0.045 | 0.23, 0.32 / 0.038 | 1.4 0.7 0.9, 0.9 1.2 1.1 | 22, 8.9 |
| D435 colour, decimate 2 | 0.056 (0.054, 0.058) | 52% | 0.38 (45%) | 0.39 / 0.070 | 0.19, 0.33 / 0.044 | 1.3 1.3 0.5, 0.6 1.1 1.2 | 9.2, 3.8 |

What the covariance says, for a still camera 1.35 m from a 0.14 m tag:

1. **Corner noise is 0.02–0.06 px in good light**, 6–15× below the sim's σ (0.29–0.35 px). In dim light it was
   0.05–0.2 px. Full-resolution detection halves it against decimate 2 (Arducam 0.021 against 0.045 px).
2. **The corners are not fully independent on some cameras.** On the Arducam and the D435 colour (rolling shutters)
   and on the D435 IR at 1280, 52–70% of the scatter is a shift all four corners share, and it persists from frame to
   frame (r 0.2–0.46). The D435 IR at 848 and 640 is independent (19–23% shared, r < 0.1). The shared part looks
   like the picture moving slightly, from vibration, auto exposure or rolling shutter; what causes it was not
   isolated. A shared shift moves the tag sideways in the picture but barely changes its size, so it hardly affects
   range.
3. **Consecutive detections at 12–16 Hz count as 0.4–0.9 independent frames each.** The graph treats every frame as
   independent, so standing still and averaging N frames does not shrink the error by √N on those cameras. The graph's
   generous σ makes this harmless today.
4. **The pose's covariance has the classic shape.** Along the view ray it is 0.17–0.6 mm against 0.03–0.07 mm across
   it (5–13× larger). The tilt about the tag's in-plane axes is the weak rotation (0.06–0.34°), and roll about the
   view axis is 5–10× tighter (0.015–0.047°). Range and tilt are correlated (r 0.5–0.8 between z and rx/ry): an error
   in the tag's apparent size also moves its tilt. Use the full 6×6 matrix, not only its diagonal.
5. **Independent corner noise predicts the pose covariance within a factor of 2.** Measured / predicted is 0.9–1.5
   across the ray and 0.5–0.9 along it and in tilt. Predicting from σ_corner is conservative for range and tilt,
   because much of the real scatter is the shared shift. So the graph's corner model has the right shape, and
   `sigma_px` is the only knob.
6. **The graph's 0.7 px is 12–35× the still-camera corner noise.** It implies range σ 5–22 mm and tilt 2–9°, against
   the measured 0.2–0.6 mm and 0.1–0.3°. That margin is meant to cover what a still bench cannot see:
   - motion blur and rolling shutter while driving;
   - calibration error (intrinsics and the camera's extrinsic: a bias, not noise);
   - the sprung base's pitch;
   - misdetections the Huber loss down-weights.

   Do not shrink it from these numbers. Fit it from a turning run and a calibrated camera instead.

## Detection, margin and misreads

- **Every configuration locked in every frame** in good light, except the projector-on IR. With the projector on,
  IR found the tag in 70% of frames in good light, with gaps of up to 73 frames (2.4 s); in dim light it found
  0 of 182. The projector's dots matter less in a bright scene, but they still cost the lock.
- **Light raised the margin.** The Arducam went from 57 to 68. The 100 fps crop went from 18, which the SLAM's
  `min_margin` 30 refused every time, to 66, accepted every time. The genuine detections' median margins were 53–69.
- **Misreads were 1–4% of frames** with the C library in good light: 32 in 3,126 frames, more than the 0.6% in dim
  light. More resolved texture gives the decoder more to misread; the 2304 px picture had the most, 3.8%. Of the
  misreads recorded (up to 20 per configuration), all but one had a margin under 3 with 1–2 corrected bits. One
  reached **23.1**. None passed the SLAM's gates, but 30 sits only modestly above the worst misread and below every
  genuine median. Keep `min_margin` at 30 or higher; don't lower it to rescue dim or short-exposure frames.
- A self-crossing misread made `omni_slam.tags.tag_pose` raise. That would have stopped `tag_slam`, which passes a
  frame's detections to `TagSlam.on_tags` without catching errors. Since Nov 4, `tag_pose` refuses it ("bad fit")
  instead; there is a test in `omni_slam/test`.
- **OpenCV aruco**, what today's `tag_detector` uses, had no misreads in these runs. It is still 3× slower and its
  orientation is worse: on the Arducam, tilt σ was 1.65°, 3.4× its corner-noise prediction, with 15% of poses
  ambiguous. In dim light it called 69% of D435 IR poses ambiguous.

**Detector cost on the Pi 4, 2 threads, one camera:**
- 26–31 ms for a 720p-class picture at decimate 2: 15–16 tag frames a second;
- 27–40 ms for the D435 IR at 640 and 848 px, 67–113 ms for 720p-class pictures, at full resolution;
- 142 ms at 2304 px;
- 14 ms for the projector-on IR picture.

**Reach, emulated** by shrinking the kept frames (full-resolution detection; this averages the noise away, so it is
optimistic):

| Camera | Last side px found | For a 0.14 m tag | For a 0.20 m tag |
| --- | --- | --- | --- |
| Arducam (846 px focal length) | 9.1–9.3 | 13 m | 18 m |
| D435 IR 1280 | 12.1 | 7.4 m | 10.6 m |
| D435 IR 848 / 640 | 11.2 / 10.2 | 5.3 m | 7.6 m |

The same limits came out in both lights. `tag_slam` itself refuses tags under 18 px or beyond 6 m.

**Live video in the console** (Pi 4, 640 px JPEGs at 22–28 kB):
- without tags: 30 / 56 / 99 fps in the Arducam's three modes, and 30 fps on the D435;
- one stream with tags: ~29 fps video, with the tags 30–150 ms behind;
- three streams with tags: the Pi saturates, the Arducam drops to 11 fps and the D435 to ~23 fps, and the tags fall
  150–200 ms behind.

## For the simulation: per camera solution

`results/sim_inputs.json` (made by `python sim_inputs.py results/tag_lock_20261104_214348.json
results/tag_lock_20261104_215027.json`) has one entry per camera solution in good light. Each entry has the picture
and field, the frame rate, exposure and gain, the tag rate and detection time, the margin, the misread rate, the
corner σ and the sim's σ, the emulated minimum side, an estimate of blur while turning, and the `cov` block above.
Each entry names its source run.

| Solution | Picture, field | Camera fps | Exposure, gain | Tag rate | Corner σ / sim | Margin | Misreads / frame | Min side (emulated) | Blur px per rad/s (est) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Arducam full field | 1152×648, 68.5×41.9° | 27–30 | 32.7 ms, 3.2 | 16 Hz | 0.045 / 0.30 | 68 | 1.8% | 9.3 | 28 |
| Arducam full field, 2304 | 2304×1296, 68.5×41.9° | 25 | 32.7 ms, 3.1 | 5 Hz | 0.046 / 0.27 | 69 | 3.8% | – | 55 |
| Arducam 2/3 crop | 768×432, 48.8×28.6° | 48 | 9.5 ms, 9.9 | 12 Hz | 0.040 / 0.30 | 66 | 1.0% | 9.1 | 8 |
| D435 IR 1280, projector off | 1280×720, 90×58.7° | 30 | 32.8 ms | 16 Hz | 0.031 / 0.31 | 62 | 0% | 12.1 | 21 |
| D435 IR 848, 60 fps | 848×480, 90×59° | 59 | 15.4 ms | 12 Hz | 0.027 / 0.34 | 60 | 0.3% | 11.2 | 6.5 |
| D435 IR 640 | 640×480, 79.6×64° | 30 | 32.0 ms | 16 Hz | 0.021 / 0.35 | 59 | 0.2% | 10.2 | 12 |
| D435 colour 1280 | 1280×720, 69.4×42.6° | 30 | not reported by the driver | 12 Hz | 0.056 / 0.29 | 53 | 2.0% | – | – |

Notes on the table:
- *Tag rate* is what one camera alone sustains on the Pi 4; two cameras share the CPU.
- *Blur* is f × exposure, the smear per rad/s of yaw near the image centre. It is computed, **not measured**. A few
  pixels of smear likely stop decoding, so at 33 ms even 0.2–0.3 rad/s matters.
- The Arducam's auto exposure keeps 32.7 ms in both lights. At this light's gain of 3.2, a ~6.5 ms exposure would
  need its maximum gain of 16 (computed). Capping exposure for motion costs noise and margin; the 100 fps crop shows
  what that looks like (9.5 ms, gain 9.9, margin 66).

How `TagCamera` (`simulation/v11 sil/tag_sim.py`), one model for every camera today, stands against the bench:

| `TagCamera` | Today | What the bench says |
| --- | --- | --- |
| Corner noise σ = 0.25 + 4/side | 0.27–0.35 px at these sides, independent per corner and frame | 0.02–0.06 px still in good light; 50–70% shared by the four corners on the rolling-shutter cameras; frames 40–90% independent. The side dependence and motion are **unmeasured**: keep the model until a range sweep and a turning run fit it |
| Detection p = 0.97 × a ramp from 12 to 32 px side | 50% at 22 px | 100% at 40–177 px; emulated limit 9–12 px, which supports the 12 px start; the ramp's shape is unmeasured |
| `blur`: p × 0.6 above 1.5 rad/s | | 21–28 px of smear per rad/s (est) on the 33 ms modes: the threshold is far too lenient. It needs exposure as a parameter and a turning run |
| `rate` | 10 Hz | 12–16 Hz for one camera on the Pi 4 (5 Hz at 2304 px) |
| `misdecode` 0.002 at margin 45 | passes the SLAM's gate | 1–4% of frames in good light at margin < 3 (one at 23), all refused. The sim's misdecodes are rarer but more dangerous than the real ones |
| Margin 40 + 2 × side | | 53–69 at 40–177 px; 18 at short exposure in dim light. Margin follows light and exposure, which the sim doesn't model |
| Latency | none | detection 26–43 ms after the frame on the Pi 4. The Arducam's picture left the server 29–44 ms after its sensor timestamp, JPEG encoding included (live stream, no tags); the D435's latency was not measured |
| `max_angle_deg` 75, `max_range` 7 m | | not measured (one angle, one range); emulated reach 5–13 m for a 0.14 m tag |

**Still needed before fitting a per-camera model into `tag_sim` and `sigma_px`:**
- a range sweep (1–6 m) and an angle sweep (0–70°) for σ, p and margin against side length and angle;
- a turning run (0.2–1 rad/s) for blur and rolling shutter, with the exposure;
- an Arducam calibration (focal length, distortion);
- a tag on the robot's own mounts.

The bench only needs the tag moved between runs; a turntable or the robot turning in place covers the turning run.

## What this means for the tag SLAM (design.md 4.6)

1. **A still tag locks completely on every camera**, with sub-millimetre range scatter and corner noise far below the
   sim's model. Motion is now the open question: 33 ms exposures smear a moving view.
2. **The graph's corner model is the right shape.** Independent corners predict the pose covariance within 2×, and
   `sigma_px` 0.7 is a wide margin over the still-camera noise. Fit it from moving and calibrated runs, not from
   these.
3. **Detect with the C apriltag library.** OpenCV aruco is 3× slower and has worse orientation (1.6° tilt σ on the
   Arducam; 7–69% of poses ambiguous). This is proposed, not changed.
4. **Keep the margin gate at 30.** It is the only thing between real misreads (up to margin 23) and the graph.
5. **The D435's projector disturbs IR tags:** 0% in dim light, 70% with dropouts in good light. Turn it off for IR
   tags, or alternate it with depth (`emitter_on_off`, untested). Colour locks too, at a lower margin. The IR's
   taller field (58° against 42°) kept a tag the colour picture cut off.
6. **Family:** this tag is tag25h9 (35 IDs, bigger cells, more reach). `tag_detector` and the sim use 36h11, so the
   robot today would not see it. The choice is open.

Not tested: motion and blur; oblique and far tags with real pixels; the Arducam's calibration; its autofocus while
moving (focus held at 1 m); more than one tag in view; a tag on the robot's mounts.
