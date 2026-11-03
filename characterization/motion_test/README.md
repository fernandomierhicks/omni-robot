# Camera-motion test: what moving the camera does to seeing the ball

Nov 1, 2026. Experimental bench tooling for the D435 on the Pi (the bucket rig), not robot code. **Paused (Fernando, Nov 1)** with the other ball benches until the stereo global-shutter colour cameras and a mobile robot exist (design.md 4.10); the tooling carries over to those cameras (the recorder is D435-specific, the analysis isn't). The question: when the robot turns, rocks over a threshold or brakes, the whole image sweeps; how much blur does that put on the ball, and do the detectors still find it, at which exposure?

## Procedure

1. The tag: Fernando's print (Nov 1) is **AprilTag 25h9, id 0**, ~140 mm (5.5 in) black square on a 200 mm sheet, the analysis's default (`--family`, `--tag-id`). Keep it flat (tape it to cardboard). The analysis doesn't need the size; a big tag just makes the rotation estimate better.
2. Put the ball **still** on a table or box and the tag **right next to it** (within ~30 cm), both 2–3 m from the camera. Light the room as you would for throws.
3. `bash characterization/motion_test/mt.sh record --seconds 40` (stops the bucket lab, 5 s countdown). While it records, with the ball and tag in view throughout:
   - hold the camera **still** for ~3 s at the start, again ~2 s in the middle and at the end (the truth is built from still frames, at every exposure);
   - **slow pans**, **fast pans** like a robot turning, **short quick shakes** like rocking over a threshold, small up/down tilts.
   The exposure cycles every 0.5 s through 1, 2, 4 and 8 ms (gains 128, 128, 64, 64), so each one sees every kind of motion; the first 0.1 s after each switch is marked settling and ignored.
4. `bash characterization/motion_test/mt.sh fetch`, then `mt.sh analyze` (PC, realsense venv). Restart the lab: `bash characterization/ball_view/bucket.sh start`.

## What the analysis does (`motion_analyze.py`)

- **Truth from the tag, not from the detectors under test:** the camera's rotation in every frame from the tag's four corner directions (3 parameters, closed form, the colour camera's calibrated focal length). In still frames the ball is found by colour and stored as a viewing direction of a reference camera; in every frame the rotation says where it must appear. Exact for a turning camera; sliding the camera adds parallax between the tag and the ball (hence: tag next to the ball). Two other choices failed on the synthetic test: the tag's 3D pose (tilt ambiguity of a small square-on tag, ~15 px at the ball) and a homography fitted to the corners (8 parameters from a ~50 px square, 5–15 px).
- **Image speed** of the ball: the truth's change between frames; the turn rate that causes it is speed / f.
- **Detectors:** `colour`, the robot's colour detector (`omni_catch`) with a window learned per exposure from still frames (what the lab's tuner would set); `blob`, the lab's colour-independent `BlobProposer` over the frames in order. A hit is within 0.6 ball radii (+ half the predicted blur) of the truth.
- **Blur:** each pixel's ball coverage (where its colour lies between the local background and the ball's), then a fit of the smeared-disc profile: across the motion for the radius and optical softness, along it for the smear length L. Moments and thresholded masks were tried and can't measure it (a 2 px smear on a 32 px ball changes the variance 0.13%; a 50% mask of a disc blurred by L ≤ 2r stays 2r long).
- Output: a table per exposure × image-speed bin (hit rates, centroid error, measured vs predicted blur, the ball's brightness and saturation, the equivalent turn rate) and `motion_results.json` in the session folder.

Synthetic check (`test_motion.py`, a turning camera rendered by K R K⁻¹ with sub-frame averaging): truth within ~1 px at every speed; blur 6.5 / 6.0 / 2.2 px measured against 6.3 / 6.5 / 2.1 predicted, ±1.5 px at the small end. `BlobProposer` found the still ball 0–7% of the time and the moving one 12–78% on the synthetic scene: it proposes only where something moved, and a camera sweep moves everything.

## Results (Nov 1, two hand-held takes, the bucket rig, `C:/Users/ferna/throw_data/motion_test/`)

Orange ball (160.7 mm) still beside the tag, ~1.5 m, radius ~30 px; evening room light; exposures cycled 1 / 2 / 4 / 8 ms at gains 128 / 128 / 64 / 64.

- **Take 1 (`20261101_124129_motion1`) was too gentle:** image speed median 16, max 302 px/s (≤ ~28°/s). The ball stood on a bottle with a similar dark orange-brown top, which biased the truth and blur.
- **Take 2 (`20261101_130346_motion2_fast`, ball on a white cup):** speeds reached 1,000–2,000 px/s (~100°/s) but only briefly: 4–17 frames per fast bin, ~1,560 of 2,384 frames still. Tag seen in 2,303 frames; the still ball's direction spread 1.4 px.
  - **The robot's colour detector (window learned per exposure) found the ball in 100% of frames at 2, 4 and 8 ms up to the fastest bins** (1,000–2,000 px/s at 4 and 8 ms; 600–1,000 at 2 ms). At **1 ms it fell to 89% (300–600 px/s) and 78% (600–1,000)**: the ball's brightness there was 0.08–0.12, so darkness, not blur. Centroid error vs the truth: 1–2 px still, 2–7 px moving (part of it the truth's own parallax: a hand-held bucket also shifts).
  - **Blur, by eye on crops of the fastest frames:** 1 and 2 ms sharp at 800–950 px/s; 4 ms a slight smear at ~1,500 px/s (predicted ~6 px); 8 ms visibly smeared sideways, ~8–10 px at 1,000–1,200 px/s, as predicted. Speed × exposure holds. **The automatic blur fit is not usable on these real frames** (4–6 px on still frames, 0–39 px scatter when moving: the support, shading and a dark, low-contrast background): use the prediction, checked by eye, until it is fixed.
  - **The colour-independent `BlobProposer` (the gesture's) found the ball 0–11% of the time:** it proposes only where things move, and a moving camera moves everything. Identification by gesture needs a still camera (fine on the bucket; on the robot, identify while parked).
- **Take 3 (`20261101_131642_motion3_far`): ball ~5–6 m, tag ~2 m (43 px).** Still frames: the robot's detector found the ball in 99% (2 ms), 100% (4 ms), 100% (8 ms); at 1 ms no colour window could be learned (too dark at that range). **Moving frames could not be scored:** with the tag at 2 m and the ball at 6 m, the hand-held bucket's sideways shifts put the tag-based truth 20–60 px off a plainly visible ball (crops), so the table's 0–80% moving hit rates are mostly truth error; a loose-colour fallback truth merged with other orange things at 8 ms. By eye on the crops: the far ball stays visible and only mildly smeared at 2–4 ms up to ~1,600 px/s. Fast frames were few (1–16 per bin).
- **For a valid far-ball run:** pure rotation (turn the bucket about the camera, e.g. on a swivel chair or turntable, no sideways shift) makes the truth exact wherever the tag is; or put the tag next to the far ball, which needs a bigger tag (at 6 m a 140 mm tag is ~14 px; ~280 mm, two sheets, gives ~29 px).
- Not covered: the robot's 74 mm ball, a thrown ball, sustained fast motion, vibration from a driving robot.

## Expected numbers (estimates, to check)

D435 colour at 848×480: f ≈ 616 px, so **1°/s of camera turn ≈ 10.8 px/s** on the image. Blur = image speed × exposure.

| Source | Image speed (est) | Blur at 1 / 2 / 4 / 8 ms |
| --- | --- | --- |
| Robot turning 90°/s | ~970 px/s | 1 / 2 / 4 / 8 px |
| Pod rocking, ~50°/s over a threshold | ~540 px/s | 0.5 / 1 / 2 / 4 px |
| The thrown ball itself, 7 m/s across at 4 m | ~1,080 px/s | 1 / 2 / 4 / 9 px |

Ball radius at 6 m: ~8 px (this 160.7 mm test ball), ~4 px (the robot's 74 mm ball). Keeping blur under about half the radius means ≤ 4 ms for the test ball and ≤ 2 ms for the robot's ball. The probe of Nov 1 (camera's own auto exposure vs fixed settings, the room's normal light): auto exposure gave a mean scene brightness of 0.47, about 8 ms at gain 64; 2 ms at gain 128 gives 0.30 and 2 ms at gain 64 gives 0.19.

## Files

| File | What |
| --- | --- |
| `motion_record.py` | Pi: colour 848×480 at 60 fps (JPEG q95, encoded on two threads during recording), depth 424×240 at 30 fps, exposure cycling, per-frame stamps; checked 359 frames in 6 s, 0 lost |
| `motion_analyze.py` | PC: truth, detectors, blur, table |
| `mt.sh` | record / fetch / analyze / list / test |
| `test_motion.py` | Blur fit on a smeared disc, a dark-ball colour mask, the whole analysis on a synthetic turning camera |
