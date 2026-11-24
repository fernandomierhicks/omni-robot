# Cabinet door opening: from the sim study to the ROS stack (handoff, Nov 21, 2026)

The arm robot opens a floor-level kitchen cabinet door in the v11 sim. It finds the door by an AprilTag, grips the low bar pull and pulls the door to 90°, with the base doing the pulling. This note is for the session that puts it on the ROS stack, so it can be run from the console, from omni_mcp, and later on the real robot. It covers:

- what was proven;
- what made it work, and the traps already found;
- how the task's inputs map onto existing topics;
- a plan in steps;
- the decisions to ask Fernando about before starting.

The design summary is design.md 4.5 (force awareness and door opening), with 4.12 for the forward camera. The algorithm's full description is the docstring at the top of [door.py](../../../../ros/src/omni_arm/omni_arm/door.py). Read both before you start.

## 1. Status

| | |
| --- | --- |
| Core | `omni_arm/door.py`: `DoorTask`, `Door`, `DoorTag`, `DoorCamera`, `upright_tag_pose`, `face_plane`. ROS-free, about 890 lines. Tests: `omni_arm/test/test_door.py`, 13 passing (about 120 s). |
| Sim harness | `simulation/v11 sil/cabinet.py`: the cabinet, the self-closing hinge, the painted tag, and the door-map entry from the spec. `studies/door_open.py` runs the detailed arm (its own truth per seed) with: the calibrated load model, the robot's C++ arm driver, the firmware SIL, the ModeMux, and the real C apriltag detector on rendered frames. |
| Results | [door_open.md](door_open.md): D435 finds the tag, 20/21 opened. [door_open_front.md](door_open_front.md): forward IMX662 finds it, 21/21 opened. [door_open_given.md](door_open_given.md): door told with an error, no tag, 21/21. Locked doors let go at 29–32 N, 3/3 in every set. |
| ROS | **Nothing yet.** No `arm_brain` verb, no cabinet in the ROS sim world (`omni_sim` bridge), no console button, no omni_mcp primitive. |
| Hardware | Nothing measured. Hinge torque (2 N m at shut, est), the wheels' torque per amp `wheel_kt` and the pull's friction are all estimates. |

Reproduce, from `simulation/v11 sil` with the sim venv:

```
python studies/door_open.py                       # D435 finder sweep  -> results/door_open.*
python studies/door_open.py --finder imx662       # forward camera     -> results/door_open_front.*
python studies/door_open.py --given               # no tag, door told  -> results/door_open_given.*
python studies/door_open.py --one --seed 1 --mirror --start -1 0.5 90 --finder imx662 --view
```

A sweep takes several minutes on 4 processes: run it in the background.

## 2. What the task does

`DoorTask.step(t, DoorInputs) -> DoorOutputs` runs at the arm's cycle (50 Hz).

**Finding the door**

| Stage | What happens |
| --- | --- |
| FIND | Still for a moment, then turn on the spot at 0.4 rad/s until the tag is seen. At most once round, else `not_found`. |
| LOOK | Base still: 5 sightings averaged. Done where FIND stopped, and again from the standoff, about 0.3 m off. A look that loses the tag a third time ends `not_found`. |

**Opening**

| Stage | What happens |
| --- | --- |
| APPROACH | The base goes to the standoff. |
| REACH | The arm goes level, jaws open. |
| CREEP | The base moves straight in, bringing the bar between the open jaws. |
| PROBE | The fingertips touch the door face; this gives the grasp depth. |
| SEAT | The arm moves back so the bar sits at the pads' middle. |
| CLOSE | The jaws close. The compliant arm is pulled across onto the bar, which gives the bar's position along the door. |
| PULL | The base drives the grip round the arc, turning with the door. |
| RELEASE | The jaws open, the base slides sideways, backs off, and the arm stows. |

Outcomes are `opened`, `stalled`, `lost`, `blocked`, `unreachable`, `missed` and `not_found`.

Fernando asked how each part is decided:

- **Perception per stage:**
  - Finding the door: the tag plus D435 depth.
  - The base's motion: odometry from the MCU.
  - The grasp's depth: the fingertip touch.
  - The bar's position along the door: the jaw touch.
  - Pulling: the arm's give, meaning its goal against where it really is, and the drive motor currents.
- **Forces:** none are commanded, and the hinge's torque is not known. The base drives the grip along the arc at a set rate. The measured force only scales and stops that rate: slow from `f_soft` 15 N, stop at `f_max` 25 N, `STALLED` after 3 s.
- **The arc:** the hinge position, the side it opens to and the pull's radius come from the tag plus its door-map entry. During the pull, the base corrects the hinge estimate by taking up the arm's radial give.

## 3. Why it works: lessons to keep

These cost time to find. Don't undo them.

1. **The base pulls and the arm only holds.** The SO-101 stalls at 2.9 N m at reach. It holds one pose all the way round, and the base drives and turns.
2. **The 5-joint arm can't move its tool sideways with the fingers level.** Every lateral move is a base move, including following the jaws across while they close (`k_follow`).
3. **The arm can't feel a steady pull.** Gearbox friction carries it: a locked door's 40 N read as 13 N. **The base's drive currents can:** wrench W = Jᵀτ with `wheel_kt`, within about 4 N. The pull is therefore limited on the base's reading, using the middle of the friction band, in `pull_force`. Keep `wheel_cur` in the inputs.
4. **Contact is found by give, not by the reflex.** A level push barely loads the wrist and elbow, so the reflex fires late. The probe stops on the arm's give along the fingers (`contact_give` 1.2 mm), with the reflex as backup.
5. **A small tag gives position, not heading.** At about 20 px, a tag's heading was 3–5° off even with a perfect camera model: the error comes from corner bias, not noise. The heading and the face's distance come from the D435 depth of the door face beside the tag (`face_plane`, a total-least-squares vertical plane). That brought the looks to ≤ 6 mm and ≤ 0.26°. Without depth, the tag's own heading is the fallback.
6. **Fit the tag upright, with roll free.** The door is vertical, so the pose has 5 unknowns: centre (3), normal heading, in-plane roll. That removes the planar mirror ambiguity. Roll matters: a tag stuck on 1° crooked broke the close looks at 176 px until roll was added.
7. **The D435 is a marginal finder.** It is 0.19 m up and tilted 30° down, and the top of its view is 1° below horizontal. It sees nothing on a door above about 0.18 m, so its tag (48 mm) sits at the door's bottom edge under the pull. From 0.88 m that tag lands at the image's top edge, and one sweep run never saw it. **The forward IMX662** (nose, 20° up, design.md 4.12 proposal) sees an 80 mm tag beside the pull, 0.30 m up, from 2.1 m. The tag must not sit at the pull's height (0.38 m), or it leaves the view at the close look.
8. **Look only while still.** Sightings count after `look_settle` (0.4 s) with the base stopped, so latency and rolling shutter don't matter. During the pull, the tag's door angle (`th_tag`, about 1° rms) is logged only and doesn't steer.
9. **Release sideways first.** Backing straight out along the fingers drags the fixed pad on the bar. Back off before the arm folds, or the arm sweeps the open door.
10. **The pull's height reads 4–12 mm high.** This is the ruler error plus the base's pitch on its springs, which the upright model ignores. It's harmless on a vertical bar but would matter for a horizontal one.

Sim-only traps:

- The tag's code is painted into a MuJoCo builtin texture **before the Renderer is created**, rotated 180° (`np.rot90(code, 2)`), or the pose comes out rolled 180°.
- The D435 tags were detected on the colour image registered to depth, standing in for the left IR (the depth's frame), so the tags and depth share one camera model.
- The forward camera's frames go through JPEG q85, as MJPEG would.

## 4. Mapping the inputs onto the ROS stack

Every `DoorInputs` field already exists on a topic. Nothing new is needed from the driver or the MCU.

| `DoorInputs` | Source | Notes |
| --- | --- | --- |
| `pose` (x, y, yaw) | `/odom` (MCU odometry) | Use odom, not the map frame: a SLAM correction mid-pull would jump the arc. |
| `q`, `goal`, `tool_force`, `motor`, `gravity` | `/arm/servo` (`ArmServoState`, every driver cycle): `position`, `goal`, `tool_force`, `motor`, `gravity` | Step the task on this message, not `/arm/joint_states`: it carries everything at the driver's rate. `arm_brain` subscribes to it today only during load calibration. |
| `reflex`, `moving` | `/arm/hw` (`ArmHardware`): `reflex_active`, `moving` | 5 Hz plus every change of motion. A reflex stop is a change, so it arrives at once. |
| `wheel_cur` | `/mcu/state` (`McuState.wheel_cur`, protocol 9, 50 Hz) | The base force reading. Signs as `wheel_vel`. |
| `tags` | A `tag_detector` instance (`omni_slam`) on the finder camera, publishing `TagDetections` | Corners are undistorted and CCW from the bottom left, the same as `door._tag_corners`. Pass `(id, corners.reshape(4, 2), margin)` for a **new** frame only, else `None`. |
| `depth` | D435 depth: `/d435/depth/image_rect_raw` 16UC1 mm, which `arm_brain.on_depth` already decodes | Metres, in the depth camera's frame. |
| camera models | `DoorCamera(fx, fy, cx, cy, T_base_cam)` | Today it is built from the MuJoCo camera (`from_config`) or `FrontEye.camera`. Add a constructor from `CameraInfo` K plus the base_link→optical TF, or the brain's `T_depth` for the D435. The optical frame is x right, y down, z forward. |

Outputs:

| `DoorOutputs` | Goes to |
| --- | --- |
| `twist` (vx, vy, wz body; `None` = stop) | `/pick/cmd_vel`. The mode manager forwards it in PICK and applies the arm-out caps; the sim ran the same `ModeMux`. Publish a zero Twist for `None`. |
| `arm` [(cmd, times, waypoints, linear, guard)] | `/arm/motion` through `arm_brain.send(cmd, times, wps, LINEAR or QUINTIC, GUARD_ON or GUARD_OFF)`. The times are absolute on the task's clock, so step the task with the brain's `self.now()` (ROS time). **These motions go straight out; don't route them through `brain_core.MotionSync`, which is the tidy's.** |
| `status` | `/arm/state` JSON (state, outcome, give_mm, pull force, looks), alongside the tidy's. |
| `wants_camera` / `wants_depth` | `/arm/camera_request` ("depth" and the finder's colour). The sim bridge renders on request. |

Camera per finder:

| Finder | Tags from | Depth (for `face_plane`) | Tag on the door |
| --- | --- | --- | --- |
| D435 (baseline) | Sim: `/d435/color/image_raw` (registered to depth). Robot: left IR, `infra1`, which is in the depth frame. Check the topic name against the realsense launch. | D435 depth, the same camera model | 48 mm, centre 0.142 m up, under the pull |
| IMX662 (`front_cam:=imx662`, proposal) | The front camera's image (`omni_sim` bridge with `front_cam:=imx662`; robot: the 4.12 camera node) | D435 depth, its own camera model | 80 mm, centre 0.30 m up, beside the pull |

## 5. Plan

Steps 1–4 make it runnable in the ROS sim. Step 5 makes it usable. Step 6 is hardware.

1. **The door map.** Each tag needs a door-map entry: `DoorTag(tag_id, size, hinge_x, pull_dy, pull_x, door_t, bar_off)`, measured once with a ruler (see the `DoorTag` docstring for the signs). Load it from a file given by a parameter, for example `door_map_path`, a small JSON or YAML keyed by tag id. Ask Fernando where it lives (decision 2).
2. **Add an `arm_brain` verb `door {tag, finder}`**, modelled on the tidy:
   - Start: `_begin_door` (requires PICK and an empty gripper), then `_listen` adding `/arm/servo`, `/mcu/state`, the tags topic and `/arm/hw`.
   - Each cycle: step `DoorTask` on `/arm/servo`; publish the twist and motions; send camera requests while `wants_camera`/`wants_depth`.
   - End: unsubscribe on DONE, and hold.
   - Reuse the tidy's health gate (servo feedback unhealthy: hold, zero the base).
   - `stop` also ends the door task: zero the base, open the jaws (the self-closing hinge takes the door back), then stow.
   - **Refuse the verb if the arm load calibration hasn't passed**: the reflex is armed only after calibration (design.md 4.5, memory "arm force awareness").
3. **Make `DoorParams` described parameters** (AGENTS.md: `omni_params`, values in `ros/src/omni_bringup/config/robot.yaml`, checked by `omni_bringup/test`). At least: `f_soft`, `f_max`, `open_to`, `w_max`, `v_pull`, `stall_s`, `give_max`, `wheel_kt`, `w_search`, `look_n`, `face_depth`, `close_look`. Follow how `arm_brain` exposes its other parameters; don't leave robot-tunable constants hard-coded.
4. **Add the cabinet to the ROS sim world** in the `omni_sim` bridge, as a world option for example `door:=none|left|right` (plus `locked`):
   - Build: `cabinet.with_cabinet(scene_xml, DoorSpec(...))`, then `cabinet.paint_tag(model, spec)` before any renderer exists.
   - Each physics step: call `cabinet.Hinge(m, d, spec).apply()` (the self-closing torque).
   - Place the robot facing the door, as `DoorRun` does.
   - Publish the door's truth (angle, hinge) on a `/sim/...` topic for scoring only.
   - If this adds a launch option, keep `ros/omni.sh`, the console's `supervisor.SIM_STEPS`, `launcher.ps1` and `launcher_full.ps1` consistent (AGENTS.md).
   - Launch a second `tag_detector` on the finder's image. Its intrinsics: none if the image is a rectified render.
5. **Make it usable:**
   - A console button "Open door" on the arm/Tidy tab, health-gated: greyed with the reason when there's no calibration, not in PICK, no finder camera, or the gripper is full (memory "health-gated console").
   - An omni_mcp primitive `robot_open_door(tag_id)` that returns a task id, plus the outcome and pull force in `robot_task` (memory "AI control as a toolkit": a primitive, not a mission).
   - The omni_arm README row and the omni_mcp README.
6. **On the robot, attended:**
   - Measure Fernando's door: hinge torque at a few angles with the luggage scale; pull geometry and tag placement for the map entry.
   - Check `wheel_kt` (a static pull against the luggage scale).
   - Check D435 IR tag detection with the emitter on: its dot pattern lands on the tag. Turn the emitter off during LOOK, or check that the decoder copes.
   - With the IMX662: calibrate its distortion (`omni_calib`). The sim used a pinhole.

**Done when, in the ROS sim** (`ros/omni.sh` arm step plus the door world):

- `door` opens left and right doors to about 90°, from a start about 1 m off and turned.
- A locked door ends `stalled` near 25–30 N, with the base stopped.
- `stop` mid-pull leaves the robot holding nothing, with the door free to close.
- The console and omni_mcp show the outcome.
- `test_door.py` still passes. Add a ROS-level smoke in the style of `ros/tools/ai_smoke.py`.
- The study (`door_open.py`) stays as the detailed-arm reference. Don't change its configuration; its results are the baseline to compare against.

## 6. Decisions to ask Fernando first

Recommend, then ask before building (memory "architecture check-in").

1. **Finder camera.** The D435 is the baseline but marginal: the robot must already be within about 0.9 m and facing the door. The IMX662 works from 2 m but is still a 4.12 proposal, not baselined. *Recommend:* implement both, since the core already takes either camera. Default to the D435, and use the front camera when `front_cam:=imx662`.
2. **Where the door map lives.** Options: a separate `door_map.yaml` next to the tag map (`omni_slam`), or entries in the tag map itself. *Recommend:* its own file, keyed by tag id, path as a parameter.
3. **Getting near the door.** Options: FIND spins from wherever the robot is, or Nav2 first drives to a stored door pose in the map frame. *Recommend:* FIND only for the first version. A Nav2 "go to door N" needs the tag's map pose (tag_slam already maps tags) and can follow.
4. **Where the task runs.** *Recommend:* inside `arm_brain`, as the tidy does, so it is the one writer of `/arm/motion` and `/pick/cmd_vel`. A separate node would need a handover of both.

## 7. Still unverified (est, VERIFY on hardware)

- Hinge closing torque: 2 N m at shut, est. The study also ran ×0.5 and ×2; all opened.
- `wheel_kt` 0.48 N m/A at the wheel, bench-inferred.
- Pad friction on the pull: μ 0.7, also run at 0.4.
- D435 IR with the projector on; IMX662 lens distortion and rolling shutter.
- The base's pitch on its springs: the pull height reads about 7 mm high.
- The real pull's geometry: a vertical bar 12 mm in diameter, 32 mm off the face, its middle 0.38 m up. A horizontal or knob pull needs a different grasp; the task assumes a vertical bar.
- Drawers: not studied. Most of the task carries over: a straight line instead of an arc, with no hinge to correct.
