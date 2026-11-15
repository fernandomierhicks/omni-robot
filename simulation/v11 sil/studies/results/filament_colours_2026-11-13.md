# The arm robot in its filament colours: perception before and after (Nov 13)

Fernando baselined the print colours on Nov 13 (cad/v13 README "Filament colours"): Bambu PLA Basic Jade White, Dark Gray and Orange plus
black TPU treads. The sim's arm robot now uses them (`filament.py`, `SimParams.colours`):

- **Jade White:** the deck and its top, the payload plate, the lidar pedestal, the bin, the arm's riser and adapter.
- **Dark Gray:** the printed SO-101, the fork plates, the guard's ears, the wheel guards, the battery tray and the e-stop's housing.
- **Orange:** the wheel hubs and their mid-plane disc, the bumper ring and the skirt.
- **Black TPU:** the rollers.

Bought parts keep their own colours.

`colours="legacy"` gives the earlier look (yellow SO-101, orange bin and riser, dark decks). The catch robot is unchanged. The model differs
only in material names: an auto vs legacy diff of the arm XML changes 41 lines, all in `material` attributes. Renders:
`filament_colours/arm_robot_filament.png` and `arm_robot_legacy.png`.

Colour matters to perception: `omni_arm/color_vision.py` finds things by saturated hue and masks only pixels within `ROBOT_R` = 0.25 m of
the arm's base. The yellow SO-101's gripper jaws reach past that mask and show in the wrist camera. So the checks below ran before and
after, on the same working tree (it has changed since the Nov 7 results).

| Check | Before (legacy colours) | After (filament colours) |
| --- | --- | --- |
| `test_v11.py` | 149 passed | 149 passed |
| `omni_arm/test/test_color_vision.py` (saved stereo render, not regenerated) | 18 passed | 18 passed |
| `studies/colour_scene.py` (stereo) | right 7, closer 0, wrong 0 | identical output (0.39% of pixels changed: a sliver of the robot) |
| `studies/colour_scene.py --camera d435` | right 5, closer 2, wrong 0 | identical output (no pixel changed: the riser D435 does not see the robot here) |
| tidy, `studies/arm_pick.py --seeds 6` (D435, 6 seeds × toys / clutter × arena / house, 5 objects, 420 s) | **105/120** (88%), 96 s per object; missed 19, dropped 11, not there 161, too big 16 | **108/120** (90%), 92 s per object; missed 3, dropped 9, not there 175, too big 16 |

Tidy per cell, before → after:

| | Toys | Clutter |
| --- | --- | --- |
| Arena | 26 → 29 | 24 → 24 |
| House | 29 → 29 | 26 → 26 |

**Per run:**

- 21 of 24 runs binned the same count, and no run errored.
- The changed runs are arena toys seed 5 (2 → 5), house clutter 0 (5 → 4) and house clutter 1 (4 → 5).
- Each run's events match before and after, at the same sim times, up to the first divergence, so the sim is deterministic. The
  divergences come from changed pixels.
- In arena toys seed 5 they come at the wrist camera's hover. With the yellow jaws in view it asked a 13 mm correction and then dropped
  the object. With grey jaws it lifted straight away and binned it. That is consistent with "missed" grasps falling from 19 to 3.
- The two house-clutter runs diverge at a D435 approach after a turn (the object measured 33 × 21 vs 30 × 22 mm).

**Reading:** the scheme does not hurt perception, and it slightly helps the wrist step (+3/120, within the size of seed-to-seed swings).
The neutral arm is what the colour choice aimed for. Not material enough for design.md.

**Separately**, the same tidy on the current tree scored **105/120** before any colour change, against the Nov 7 baseline of 112/120
(`arm_pick_d435.json`, still cited in design.md). Something in the tree changed since Nov 7, such as the Nov 12 roller wheel. That
change, not the colours, accounts for the gap; it is not investigated here.

Results: `arm_pick_d435_2026-11-13_legacy_colours.json` (before) and `arm_pick_d435_2026-11-13_filament_colours.json` (after).
`arm_pick_d435.json` stays the Nov 7 run.

Conditions: the before tidy ran beside a SolidWorks import on the same PC; the after tidy ran alone. Both used 24 workers (the default
on this PC), each holding about 3.6 GB committed. Do not run a full `arm_pick.py` beside SolidWorks: it ran out of memory.
