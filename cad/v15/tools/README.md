# cad/v15 tools

Every script here builds the v15 robot (the mid deck). Run them with `C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B`,
from this folder; `../regen_all.sh` runs the whole chain in order. The SolidWorks scripts run with `C:/Users/ferna/cadenv/Scripts/python.exe`.

Rules kept by every module (Nov 14 cleanup):
- **No module changes another module's constants.** Each value lives in the module that uses it (`dims.py` for shared dimensions).
- **One world.** Anything that checks against "the whole robot" takes it from `world.py`.
- **Layout F is `base.py`'s own** (rolling lines tangent to r 154, wheels slid −20).
- **Entry points write only inside cad/v15:** `validation/<name>/`, `step_export/<name>/` and `solidworks/`.

## Entry points (in `regen_all.sh` order)

| Script | Builds / checks | Writes |
| --- | --- | --- |
| `wheel_motor.py all` | the omni wheel (687 rollers) and the AK40-10 with its plug | `validation/wheel_motor`, `step_export/wheel_motor` |
| `coupon_687.py`, `coupon_627.py` | the bearing fit coupons | `validation/coupon_*`, `step_export/coupon_*` |
| `pod_mid.py all` | the pod: fork, hangers, beam (+ its pin and droop lug), towers, springs; every pose; hole audit; hand loads; its footprints for the deck; the marked parts once per pod (`parts/<part>_<n>_<pod>.step`) | `validation/pod_mid`, `step_export/pod_mid` |
| `chassis_mid.py all` | the base: the deck (4 pieces, laps, pockets, accessory holes), the bumper, the pedestal, the fins (`fins.py`), the battery and its clips, the service-stand pads; checks against the pods | `validation/chassis_mid`, `step_export/chassis_mid` |
| `d435_riser.py motion / render` | the arm's planned poses and moves (cache), the D435 mount pictures | `validation/d435` |
| `upper.py all` | the second floor and everything on it; its check against the base and the D435's (`d435_riser.check_mount`) | `validation/upper`, `step_export/upper` |
| `arm_sweep.py [robot_ring]` | the SO-101's meshes through every planned move against the whole robot | `validation/upper/arm_sweep*.json` |
| `harness_route.py`, `harness.py all` | the leads' routes; the leads drawn and checked | `validation/harness`, `step_export/harness` |
| `service.py all` | driver access to every screw; the pods and the battery dropping out | `validation/service` |
| `robot_step.py` | one STEP of every printed part | `step_export/robot_assembly.step` |
| `print_files.py [--bumper] [--new-tracker]` | **not in `regen_all.sh`; run after the STEP exports.** The first floor's and the four pods' print STEPs sorted by filament colour (one folder per part type, every pod's file in it; the wheel hub and droop lug written once per pod because they come in two hands), the folder README and the Excel print tracker (written only if missing, never overwritten). Needs `openpyxl` in the omni-robot-sim venv | `print_files/First floor/` |
| `sw_robot_parts.py`, `sw_robot.py <stages>` | the SolidWorks viewing assembly | `solidworks/`, `validation/solidworks` |

## Libraries

| Module | What it holds |
| --- | --- |
| `dims.py` | shared dimensions (wheel, motor, springs, M3 holes, pod geometry) |
| `base.py` | layout F: where the four pods sit (`frame`, `place2d`, `place3d`), plan helpers (`proj`, `solid_of`, `prism`) |
| `pod.py` → `pod_lean.py` → `pod_rx.py` → `pod_p4.py` → `pod_mid.py` | the pod, in layers: poses, springs and plungers (`pod.py`); the 6804 retainer, lead-tie geometry, colours and iso renders (`pod_lean.py`); arms, 627 housings, wheel envelope, checks and renders (`pod_rx.py`); P4's fork and beam (`pod_p4.py`); the hangers, towers and deck interface (`pod_mid.py`). Each layer is a library except `pod_mid.py`. v14 history lives in the docstrings |
| `fins.py` | the five fins (their lower edges generated from a height map), the lidar and its pedestal, the second floor's height |
| `chassis_mid.py` | also a library: `built()`, `pods_placed(a)`, `BATTERY`, `STAND`, `colour` |
| `world.py` | the whole robot as solids: `base()`, `pods(a)`, `upper()`, `static_robot(a)` |
| `upper.py`, `devices.py` | the second floor, arm kit, electronics (device envelopes and plugs) |
| `d435_riser.py` | the D435 on the arm riser: its model, fields of view, mount, checks |
| `hardware.py`, `palette.py`, `step_colour.py`, `print_beds.py` | fastener records and solids, filament colours, coloured STEPs, each printed part's bed face |
| `labels.py` | Nov 15: assembly marks (wheel numbers 1 FL, 2 FR, 3 RL, 4 RR; engraved text and arrows on flat faces, readable on mirrored pods). `pod_mid.labelled(parts, i)` marks a pod's hangers, beam, towers and fork halves; `chassis_mid.deck_marks` numbers the deck pieces |
| `arm_pose.py` | the SO-101's official parts posed by the robot's own `ros/src/omni_arm` kinematics (the one import from outside cad/) |
| `subasm.py`, `mate_api.py`, `sw_native.py` | SolidWorks COM helpers (from cad/v10) |

Caches, safe to delete: `validation/pod_mid/pod_fp.pkl` (pod footprints), `validation/chassis_mid/hmap.pkl` (the fins' height map),
`validation/d435/*` (arm motion, arm cloud, view world: delete `view_world.npz` after a base change), and `pod_rx`'s wheel envelope in the
system temp folder. The first three are keyed on the sources' times, except `validation/d435`.
