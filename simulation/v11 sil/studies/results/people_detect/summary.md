# People from the forward IMX662: sim study, Nov 7 2026

`studies/people_detect.py` (design.md 4.12). Detector: YOLOX nano and tiny (ONNX, onnxruntime CPU, 4 threads), the
`omni_perception.people_vision` code the robot's `people` node runs (detection, `plausible`, `sight`, `fuse`). Camera:
`imx662_cam` (pinhole 58.4° tall, sensor noise, vignetting, MJPEG; the robot still, so no rolling-shutter blur), at
the CAD fit's pose (pupil (0.210, 0, 0.104) m, 20° up). Images at 1920 × 1080 and 960 × 540 (the bridge's size).

**Limits.** MuJoCo renders of primitive people (`house_scene.person`: capsules, ellipsoids, plain colours), one
lighting setup, no motion blur, no lens distortion. Not a real camera, real people or real clutter. CPU times are
on the cloud VM (4-core 2.1 GHz Xeon) with another study sometimes running: indicative only.

## grid: one person at a time on an open floor

144 frames: distance 1.2–7 m × bearing 0 / 25 / −38° × standing / seated (0.45 m stool) × facing the robot /
sideways / away; clothes and skin varied; a sofa, a plant and a wall behind. Found = a box whose bearing interval
(±3°) holds the person's.

| Distance | nano 1920 | tiny 1920 | nano 960 | camera range error, median (nano 1920) |
| --- | --- | --- | --- | --- |
| 1.2 m | 8/18 | 4/18 | 3/18 | 0.19 m |
| 1.5 m | 14/18 | 9/18 | 13/18 | 0.26 m |
| 2 m | 18/18 | 13/18 | 18/18 | 0.31 m |
| 3 m | 18/18 | 15/18 | 18/18 | 0.42 m |
| 4 m | 18/18 | 18/18 | 18/18 | 0.53 m |
| 5 m | 15/18 | 17/18 | 16/18 | 0.47 m |
| 6 m | 18/18 | 17/18 | 18/18 | 0.56 m |
| 7 m | 15/18 | 18/18 | 14/18 | 0.58 m |

- No box on anything but the person (all four configurations).
- Under ~1.5 m the head leaves the frame's top (the field tops out 49° up) and detection drops. The Follow mode's
  1 m distance is inside this zone.
- The camera's own range (foot point and head top, people_vision.sight) was always within 2 sigma; it is rough
  because the camera is only 0.1 m up (the feet are ~2° under the horizon at 3 m). The lidar gives the range.
- Misses at 5–7 m were mostly seated people.
- ms a frame (median): nano 9.6 (1920) / 16.8 (960, including the resize), tiny 51 / 47.

## house: the furnished house

200 frames at free-floor poses (4 headings) in the props house, with three `house_scene` people added (two standing,
one seated on a stool) plus the house's own two figures (house_props' office figure, the catch thrower). In view
means inside the frame, 0.8–8 m, and not hidden (a ray from the camera to the chest or head reaches the person
first). The lidar: a 720-ray cast at the A2M12's plane through `omni_console.people.detect`, **without** the map's
furniture filter (the console uses it when localized).

| | nano 1920 | tiny 1920 | nano 960 | tiny 960 |
| --- | --- | --- | --- | --- |
| People found (of 104 in view) | 86 | 77 | 84 | 73 |
| at 2–4 m | 35/38 | 26/38 | 35/38 | 23/38 |
| at ≥ 4 m | 51/65 | 51/65 | 49/65 | 50/65 |
| Boxes on no person (200 frames) | 2 | 0 | 1 | 0 |
| ms a frame, median | 9.1 | 47.5 | 16.0 | 46.0 |

- The boxes on no person were wall signs (AprilTag placards), scored 0.36–0.45, floating above the floor. Before
  `people_vision.plausible` (a box whose bottom floats above the horizon needs 0.5), nano gave 5 at 1920 and 2 at 960.
- Lidar leg candidates: 3.3 a frame, 446 of them not a person over the run (chair, table and stool legs). One camera
  frame marked 61 of those (14 %) as looked at with no person. The rest were outside the camera's 90° field, nearer
  than 0.8 m or farther than 6 m. Each look lowers a candidate's odds, so a Follow that turns to face people gets
  more looks.

## Conclusions

- **YOLOX nano is the default** (`robot.yaml` people.model): it found more people than tiny on these renders at
  every distance under 4 m, at about a fifth of the CPU. Re-choose on real frames.
- **The camera confirms and the lidar ranges.** fuse keeps the lidar's position when both see a person.
- **Close people are the camera's blind spot.** "Looked at, no box" counts only when a whole person (0.1–1.6 m)
  would be in the frame, so a person followed at 1 m isn't talked out of being a person. A camera-confirmed track
  stays a person while the lidar keeps it.

Raw rows: `grid.json`, `house.json` (with the people's positions); summaries `grid_summary.json`, `house_summary.json`.
