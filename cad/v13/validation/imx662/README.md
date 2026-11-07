# IMX662 forward camera: fit check (Nov 6 2026, study, not drawn into the robot)

A generic IMX662 USB 2 module (Arducam-sized: 38 × 38 mm board, M12 lens holder, Ø14 × 16 mm lens, USB-C on the back with a straight plug)
on node_front under the lidar, in front of its pedestal, tilted up. Made by `../../tools/imx662_fit.py`. The camera is a bought-part
envelope and the stand is a placeholder; neither is a design. Nothing in v13's own scripts or STEPs changed. Results, the sim study and
the conclusion: `simulation/v11 sil/studies/results/imx662_2026-11-06.md`, design.md 4.12.

| File | What |
| --- | --- |
| `fit.json` | 15 poses (lens front x 214; z 95 / 105 / 115; 10–30° up): clearances to the arm sweep, the robot (front pods at droop / nominal / bump), the scan plane, the bumper, the riser D435's fields and its own field. All fit. Default `z105_up20` |
| `imx662_module.step` | the module alone (generic, VERIFY) |
| `imx662_fit_review.step` | the robot's front with the camera, placeholder stand, plug and lead, default pose |
| `side_right.png`, `iso_front.png` | renders (the side one with its field green, the riser D435's blue, the scan plane red) |
| `arm_cloud.npz`, `arm_motion.json` | the SO-101 swept round the nose (`d435_riser`'s motions, copied; cloud kept x 150–330, z 0–175) |

```
cd cad/v13/tools
python -B imx662_fit.py cloud      # once (cached)
python -B imx662_fit.py check      # fit.json, STEPs, renders (~10 min: builds the whole robot)
```
