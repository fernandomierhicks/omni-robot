# Motor study run log (Nov 1-2 2026, overnight session)

Run from `simulation/v11 sil/studies`, 24 workers (28 cores). Nothing committed. Wall times are the harness's own.

| Part | Jobs | Wall | Errors | Gates |
| --- | --- | --- | --- | --- |
| C speed / cruise / straights (AK40) | 1232 | 13 min | 0 | top speed dir 0 4.27 m/s (rpm), dir 45 3.20; cruise 1 m/s 21.2 W, v_true 0.998 |
| A accel / brake / reverse / turn / panic | 1344 | 33 min | 0 | start dir 0 2.61 (wheel-hop), brake 7.39 |
| T thermal (incl. +-50% thermal-resistance variants) | 120 | 12 min | 0 | 10 kg climb 6 deg: winding 33.2 C, no derate |
| I slopes + ramps | 696 | 64 min | 0 | slope start first failure 20 deg (wheel-hop) |
| U uneven loading | 336 | 1.5 min | 0 | - |
| R rotation | 624 | 7 min | 0 | - |
| F house transitions | 4576 | 25 min | 0 | - |
| S steps / bumps | 1052 | 91 min | 0 | creep step 0.25 m/s dir 0: 65 mm, hull grounding |
| loop 0.25 sensitivity (C, A; floor, 10/20 kg) | 266 + 168 | ~7 min | 0 | - |
| P suspension / fork pivot | 1380 | 20 min | 0 | - |
| Phase 4 alternatives CAIT (8 `-ds` motors, coarse) | 4352 | 344 min | 0 | - |
| Phase 4 alternatives S (coarse) | 576 | 56 min | 0 | - |
| Phase 5 M, AK40-10 (10/15/20 kg, 5 profiles, 5 routes) | 73 (+2 earlier) | 6.7 min | 0 | - |
| Phase 5 E, AK40-10 (10/20 kg, adult+capmax) | 4 | running when this was written | - | - |
| Nov 2 `pivot_quick.py` (drawn 43 mm vs level pivot, 10/20 kg) | 100 | 4.1 min | 0 | level: start 6.2, slope 28 deg, cruise 2 m/s 41 W (10 kg) |
| Nov 2 `hands_quick.py` (level pivot: diagonal vs same hand, 10/20 kg) | 140 | 5.4 min | 0 | same hand: start 6.2 -> 4.1, slope 28 -> 16-18 deg, panic tilt 0.5 -> 8.8 deg, cruise 41 -> 35 W (10 kg); diagonal kept |

## Not run (by decision, Nov 2)

- Phase 5 missions for the alternatives (GIM3510-8-ds, DM-2325-ds were the picks: closest to AK40-10-ds at 20 kg on
  S, I, A) and their endurance runs. Fernando asked that the AK40 endurance be the last simulation.
- Phase 6 Nav2 subset. Fernando's own ROS stack was live in WSL (console on 8080/9090, arm launch + MuJoCo bridge on
  5761); `omni.sh` mirrors and rebuilds the shared `~/omni_ws`, so it was not started unattended.

## Odd things

1. **`drive_char.py report` writes an empty REPORT.md.** It passes `sys.argv[1:]` (`["report"]`) as the part list.
   Workaround used: `python -c "import drive_char, drive_char_report as R; R.main()"` (REPORT.md, ~600 kB, covers
   everything run up to Nov 2 10:40; E and the alternatives' missions are absent). `drive_char_report.py` run directly
   fails with `No module named catalog` (needs `drive_char` imported first).
2. **Thermal ignores `--loops`.** The T job list builds its own `Cfg` without `loop`, so the loop-0.25 pass reused the
   60 existing thermal jobs ("60 already done"). There is no thermal loop sensitivity.
3. **drive_mission firmware race (fixed, one harness edit).** All workers share one firmware variant per (motor,
   profile), but the study header differs between the house routes and the corridor/open worlds in a feed-hardware
   comment line only. Every house job rewrote the header and rebuilt the DLL concurrently, so the first M run failed
   (FileExists, access denied, zig no input). `firmware_for()` in `drive_mission.py` now always builds from the
   open-world 10 kg params; the firmware behaviour is identical. The 10 errored rows are kept in
   `M_race_errors_nov2.jsonl`; `M.jsonl` holds only clean rows. Killing that run orphaned zig compiles and workers that
   held the `mc_*` DLLs open; they were killed by PID and the stale `mc_*` build artifacts under `firmware/sil/build`
   were deleted and rebuilt.
4. **The handoff's loop claim does not reproduce.** Cruise drive power at `loop` 0.25 is 19% lower at 10 kg and 29%
   lower at 20 kg at 1 m/s (not "about 40%"). Acceleration is much more sensitive (see the preliminary read).
5. Thermal derate shows at 20 kg on 14-18 deg climbs (onset 12-31 s) and the level-5 `bursts` duty (56 s). At 10 kg there is none.
   The handoff gate (none below 15 deg at 10 kg) holds. Thermal parameters are estimates.
6. `omni.sh`-style `nav_goals` was not exercised.
