# v7: home floors

v7 is the v6 round-3 robot (X4 omni base, axle-plane deck, tilting catch/launch module, chute feed) with a base for real home floors. Fernando's goal: handle thresholds, rugs and floor transitions much better without losing flat-floor catching. The answers and numbers are in [../../design.md](../../design.md) section 24. Everything here is **proposed**.

Every crossing study uses the explicit-roller wheel model (`wheel_model="rollers"`). The fast sphere wheel can't represent a roller meeting a step sideways, so it can't get stuck where a real omni wheel does.

## What changed from v6

| | v6 (still the default `SimParams()`) | v7 options |
| --- | --- | --- |
| Wheels | goBILDA 96 mm, 2 x 8 rollers of 16 mm | Catalog entries for 120-152 mm omni wheels (goBILDA, AndyMark DuraOmni, Rotacaster, Nexus), a goBILDA 140 mm mecanum, and printed designs (`catalog.printed_omni`). `OmniWheel.rows` (2 or 3 staggered rows), `kind="mecanum"` (rollers at 45 deg, with matching kinematics), `edge_radius()` |
| Suspension | The rollers' rubber, 0-6 mm at 30 N/mm | `suspension="pod"`: each wheel and its motor on a sprung, damped pod (`pod_travel`, `pod_rate`, `pod_preload`, `pod_damping_ratio`) |
| Cross-axis compliance | none | `swing="pendulum"` or `"parallel"` (`swing_length`, `swing_range_deg`, `swing_stiffness`, `swing_preload`, `swing_damping`). Tested and rejected (design.md 24.2) |
| Rocker | front pair, +/- 3 deg | `rocker="both"`: front and rear beams tied by a differential; `rocker_deg` |
| Wheel loads | from the roller slide's deflection | from a force sensor on each pod (`robot_model.wheel_loads`): right for any suspension |
| Firmware | slow zones (rectangles) | `terrain_assist`: a map of floor edges (`floor.edges`, or `terrain_map`) caps the speed before each edge (`edge_speeds`) and turns the base so a square edge's normal sits at 0 / 90 deg in the body frame (`edge_align`); 100 Hz |
| Floor features | thresholds, rugs with square or beveled x edges | rugs turned (`Carpet.yaw_deg`), beveled on all sides (`bevel_y`), coloured; `floor.Wall`; `floor.HOUSE_OBSTACLES` (T-molding, reducer, bath mat, plush rug) |

## Run it

```powershell
$py = "$env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe"
& $py house.py                     # drive the proposed v7 robot through a house (keyboard, gamepad or flight stick)
& $py house.py --base v6           # the v6 base on the same floor (T toggles the terrain assist)
& $py -m pytest -q                 # inherited v6 checks + v7 checks
cd studies
& $py home_floor.py A              # crossings: every base x obstacle x 24 body directions x 3 speeds (~30 min)
& $py home_floor.py B              # catch games on the flat and home floors (~15 min)
& $py home_floor.py C              # costs: mass, CG, tip-over, clearance, motor margins, price
& $py home_floor.py tune           # pod travel / rate / damping / preload grid
& $py home_floor.py probe "v6" "rug edge 12 mm" 45 1.0      # one crossing, printed
& $py home_floor.py report
```

`house.py` needs pygame (`& $py -m pip install pygame`, listed in `../requirements.txt`).

## Driving the house

| Input | Keyboard | Gamepad (XInput) | Flight stick (Logitech Extreme 3D) |
| --- | --- | --- | --- |
| Translate | W A S D (or arrows for forward/back) | left stick | stick |
| Turn | Q / E (or left/right arrows) | right stick | twist |
| Speed | 1 m/s, SHIFT = 2 m/s | right trigger to 2 m/s | throttle slider, 1-2 m/s |
| Fire / load ball | F / L | A / B | trigger / thumb |
| Terrain assist | T | X | button 3 |
| Camera | C (follow, chase, top); mouse drag and wheel | Y | button 4 |
| Reset / spots | R / 1-9 | START | button 8 |

The HUD shows speed and its cap, body tilt and its peak, the longest airborne phase, whether the assist is aligning the base, the ball, the floor under each wheel and its load, and how fast the sim runs relative to real time (the roller model is heavy; on a busy PC it runs in slow motion rather than skipping physics).

The floor: wood living room and hallway (laminate), a tile kitchen 6 mm up behind 40 mm reducers, wall-to-wall bedroom carpet (12 mm) behind carpet trim, a 12 mm marble threshold into the bathroom, a 5 mm T-molding into the office, a 12 mm rug turned 15 deg, an 8 mm runner turned -30 deg, a 16 mm plush rug and a 6 mm bath mat. Spots 1-9 put the robot in front of each.

## Files (new or changed from v6)

| File | What changed |
| --- | --- |
| `catalog.py` | `OmniWheel.rows / coverage / kind / printed`, `edge_radius()`, `printed_omni()`, new wheel entries |
| `params.py` | suspension, swing, rocker, terrain-assist fields; deck slot follows the wheel size; the firmware acceleration table scales with the rollers' grip |
| `robot_model.py` | pods (carrier body with the vertical slide and the motor, hub with the drive), swing pivots, both-rocker differential, roller rows and mecanum rollers, `wheel_loads()` from `wload` force sensors |
| `robot.py` | terrain-aware firmware (`_terrain`, `terrain_yaw`, `drive_velocity(w=None)`), swing centring preload, mecanum kinematics |
| `omni_control.py` | `OmniKinematics(roller_angle_deg=...)` for mecanum |
| `floor.py` | `edges()`, `seg_dist()`, turned and all-side-beveled rugs, `Wall`, `HOUSE_OBSTACLES`; rolling resistance reads the pod loads |
| `house.py` | the house floor and the drivable scene |
| `studies/home_floor.py` | crossings, catch games, costs, pod tuning |
| `test_v7.py` | v6's checks + v7's |
