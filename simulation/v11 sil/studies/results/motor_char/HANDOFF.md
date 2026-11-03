# Motor study — hand-off for the overnight runs (phases 2–6)

Written Nov 1 2026 at the end of phases 0–1 (model fixes, harness, smoke). The plan is in
`C:\Users\ferna\.claude\plans\read-design-md-lets-do-declarative-pebble.md`. Your job is to **run** phases 2–6,
check that the results are sane, and leave them ready for the Phase 7 write-up. Do not change the robot model, the
catalog, `design.md` or the firmware sources. Do not commit.

## What exists

| File | What |
| --- | --- |
| `studies/drive_char.py` | Physics-capability harness. Parts C A R S F I U T P, resumable JSONL per part |
| `studies/drive_mission.py` | Firmware-in-the-loop missions. M = routes, E = endurance to the 21 V stop |
| `studies/drive_char_report.py` | `results/motor_char/REPORT.md` tables (also `drive_char.py report`) |
| `studies/motor_fit.py`, `studies/data/ak40_10_kv170_24v.csv` | AK40 curve fit (done; `ak40_fit.png`, `ak40_fit.json`) |
| `catalog.py` | `AK40-10` (curve fit), `AK40-10-dc` (old model), `-ds` datasheet fits of every candidate |
| `robot_export.study_firmware / study_limits / study_from_env` | Study firmware builds (never `lib/omni_fw`) |
| `ros/.../stack.py` `nav_profile:=study<v>[a<acc>]` | Nav2 with every speed/accel cap raised for the study |

Python: `C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe` (call it `$py`). Run from `simulation/v11 sil/studies`.
The PC has 28 cores. The default is 27 workers; leave Fernando some headroom with `--workers 24` if he is using it.

## Model facts that affect how to read the results (found in phase 0–1)

1. **AK40 fit:** R_eff 2.13 Ω, friction 0.12 N·m, 4.1 N·m peak at the shaft. Datasheet fits are optimistic: the
   AK40 by the datasheet rule has R 1.45 Ω. Compare alternatives against `AK40-10-ds` (same rule), and use `AK40-10`
   for absolute numbers.
2. **Roller hull** is now 13 × 64. The old 14-flat hull produced 0–120 N load chatter and was 7% slow.
3. **Fork torque jacking:** CAD pivot 43 mm above the axle. Forward clean acceleration is about 2.6 m/s²; with the
   pivot at axle height it is about 6.2. Part P sweeps `pivot_z`. Expect forward/backward asymmetry everywhere.
4. **Speed-loop fight:** about 1 N·m of null-space torque at cruise. Cruise drive power at 1 m/s is about 21 W at
   `loop` 1 and about 40% less at `loop` 0.25. This is an unknown real gain, so run the loop sensitivity below.
5. **Wheel hop** under hard traction shows as loads swinging 0 to more than 100 N at roughly 25–40 Hz. The classifier
   cause `wheel-hop` means that.
6. `ratio(cfg, dir)` calibrates commanded vs true speed per direction (about 0.99 now). Tests judge true speed.
7. Pre-existing test failures, not from this work: `test_firmware_drives_then_ramps...` (decel 5.19 vs 5.12 bound)
   and `test_each_injected_fault...` (fault fixture injects 21.5 V, but the 6S stop is now 21 V).

## Sanity gates (check after the first minutes of each part; stop and report if one fails)

- `topspeed` dir 0 at 10 kg, full charge: 4.0–4.4 m/s, cause `rpm`. Dir 45: about 3.2 m/s.
- `cruise` 1 m/s, dir 0, 10 kg floor: `p_drive_W` 15–30 W, `v_true` within 2%.
- `accel brake` dir 0, 10 kg: 6–8 m/s². `accel start` dir 0: 2–3.5 m/s², cause `wheel-hop` (fork jacking).
- `slope start` dir 0, 10 kg floor: about 18° (first failure 20°, `wheel-hop`). Creep step at 0.25 m/s dir 0: about
  65 mm, then hull grounding.
- `thermal climb 6°` 60 s, 10 kg carpet: winding about 33 °C, no derate.
- `ratio` values 0.95–1.02 (print one: `$py -c "import drive_char as D; print(D.ratio(D.Cfg(), 0))"`).
- No row with `err`, no `NaN`/`null` in a numeric boundary. If more than 2% of a part's jobs error, stop that part,
  read the traceback in the JSONL and report it. Don't patch the model to make errors go away.
- `t_derate` should not occur below 15° climbing at 10 kg (thermal is est).

## Run order (highest value first; every command resumes if interrupted)

```powershell
cd "C:\Dropbox\Personal Projects\Robotics\omni-robot\simulation\v11 sil\studies"
$py = "C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe"

# Phase 2 — AK40 physics capability (~4 h; S is the long one, ~2 h)
& $py drive_char.py C          # 1232 jobs, ~10 min
& $py drive_char.py A          # 1344 jobs, ~20 min
& $py drive_char.py T          # thermal incl. +-50% thermal-resistance variants, ~10 min
& $py drive_char.py I          # slopes (gravity tilt) and ramps, ~40 min
& $py drive_char.py U          # ~2 min
& $py drive_char.py R          # ~3 min
& $py drive_char.py F          # 4576 jobs, ~20 min
& $py drive_char.py S          # ~1100 ladders (5 mm rungs then 2.5 mm refinement, plus slick-edge mu 0.5), ~1.5 h
# speed-loop sensitivity (unknown CubeMars gains) on cruise / accel / thermal, 10 and 20 kg, floor
& $py drive_char.py CAT --masses 10,20 --surfaces floor --socs 1.0 --loops 0.25

# Phase 3 — suspension and fork pivot (~20 min)
& $py drive_char.py P --masses 10,20

# Phase 4 — alternatives (coarse grids, ~3 h)
$alts = "AK40-10-ds,DM-2325-ds,DM-2325-low-ds,DM-J3507-ds,GIM4305-10-ds,GIM3510-8-ds,EduLite05-ds,M2006-ds"
& $py drive_char.py CAIT --motors $alts --masses 10,20 --coarse --no-thermal-variants
& $py drive_char.py S --motors $alts --masses 10,20 --socs 1.0 --coarse

# Phase 5 — firmware-in-the-loop missions (~1.5 h; E is the long pole, about an hour of sim each)
& $py drive_mission.py M --motors AK40-10 --masses 10,15,20
& $py drive_mission.py E --motors AK40-10 --masses 10,20 --profiles adult,capmax
#   then the 2-3 best alternatives from phase 4 (pick by REPORT.md: closest to AK40 on S, I, A at 20 kg):
& $py drive_mission.py M --motors <best1>,<best2> --masses 10,20 --profiles adult,capmax,fast
& $py drive_mission.py E --motors <best1>,<best2> --masses 20 --profiles adult

& $py drive_char.py report       # writes results/motor_char/REPORT.md
```

Run parts one after another (each one uses all workers). Use `run_in_background` and wait for completion.
The progress line prints about every 2.5% of a part.

## Phase 6 — Nav2 subset (WSL; ~1–2 h; lowest priority)

The ROS `goals` step only runs the catch robot, so use the `arm` step plus the `nav_goals` node:

- Use your own ROS domain and ports: `ROS_DOMAIN_ID=42`, `mcu_port:=5771`, `servo_port:=5773`. Never `omni.sh stop`,
  `stop-console` or a broad `pkill`; stop only your own PIDs. Fernando's console may own 8080/9090.
- `OMNI_SIM_STUDY` makes the sim bridge load the study motor, mass and firmware caps (`robot_export.study_from_env`).
- For each motor in {AK40-10, best alternative} and v in {2, 2.5, 3, 4}, acceleration 4 (and 5 for v ≥ 3), inside WSL:

```bash
export ROS_DOMAIN_ID=42
export OMNI_SIM_STUDY='{"motor": "AK40-10", "mass": 10, "speed": 3.0, "accel_scale": 1.5}'
bash ros/omni.sh arm nav_profile:=study3a5 mcu_port:=5771 servo_port:=5773 console:=false rviz:=false \
     foxglove:=false viewer:=false clutter:=off &          # note its PID
# when Nav2 is active (~30-60 s), in a second WSL shell with the same ROS_DOMAIN_ID:
source /opt/ros/jazzy/setup.bash; source ~/omni_ws/install/setup.bash
ros2 run omni_sim nav_goals --ros-args -p speed:=3.0 -p route:=house -p out:=motor_nav_ak40_v3 -p use_sim_time:=true
ros2 run omni_sim nav_goals --ros-args -p speed:=3.0 -p route:=sprint -p out:=motor_nav_ak40_v3_sprint -p use_sim_time:=true
# then stop your launch by its PID (kill -INT <pid>)
```

- `nav_goals` writes `studies/results/<out>.json` (`omni.sh` copies results back on exit). Record the peak and median
  speed against the cap. If Nav2 can't reach the cap, say which layer stopped it: MPPI (speeds sampled), the smoother,
  the costmap/horizon, the firmware cap, or physics (compare with phase 2's top speed and phase 5's corridor).
- If `nav_goals` won't run against the `arm` step (parameter or route mismatch), don't fight it for long. Publish
  `/cmd_vel` along the corridor with a short script on the same domain, and log `/odom` speed instead.

## What to leave for the write-up

- `results/motor_char/*.jsonl`, `REPORT.md`, the Phase 6 JSONs, and a short `RUNLOG.md`: what ran, wall times,
  anything odd (gates that failed, errors, parts skipped).
- Don't interpret the verdict or edit `design.md`. The Opus session writes the verdict, the study README and the
  design update (Phase 7).
