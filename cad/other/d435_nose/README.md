# D435 on the arm robot's nose (concept, Nov 1)

**The active integration is now in
[`cad/v10 human editable/ASSEMBLIES.md`](../../v10%20human%20editable/ASSEMBLIES.md).**
The nose mount was rejected Nov 2 because its imager fields crossed the bumper
and deck. The active D435 is now on the centre camera bar, at (198.991, 0, 183.974)
mm, 40° down, with separate CAD field-of-view volumes. The earlier concept and
dimensions below remain for reproduction; they are not the active assembly.

design.md 4.11 (proposed). Fernando's RealSense D435 as the arm robot's floor camera and Nav2's near-field depth, on
the deck's nose under the lidar's scan plane. **This is a placement concept for SolidWorks, not a finished part:** the
real part is drawn there (`cad/solidworks.md`); Fernando's D435 model is in `../Realsense to bucket/`.

`python d435_nose.py` (the sim venv) writes, in the ROBOT frame (mm: x forward, y left, z up from the floor; insert at
the assembly origin):

- `out/d435_nose.step`: the camera's envelope (90 × 25 × 25), the wedge, the screws (magenta)
- `out/d435_wedge.stl`: the wedge alone (PETG, ~30 cm³)

## What the SolidWorks part must keep (from the v11 sim)

| | Value | Where it's checked |
| --- | --- | --- |
| Camera body middle | (190, 0, 105) mm, view 30° below horizontal | `params.py` `d435_pos`, `d435_tilt_deg`; `test_v11.py` `test_the_d435_on_the_nose_clears_the_robot` |
| Depth origin (left imager) | (197, 18, 101) mm: what `robot_arm_d435.json` and the TF tree carry | `robot_export.py`; recalibrate on the robot |
| Top of the body | ≥ 3 mm under the lidar's scan plane (sim: top 122, plane 131; drawn plane 150) | same test: no lidar shadow |
| Outline | inside the bumper's 217 mm radius (sim: 212) | same test |
| Clearance | ≥ 3 mm to the lidar, bumper, wheels, guards (sim: 6.6 mm to the lidar body) | same test |
| Floor seen | from 0.27 m from the robot's centre (the arm reaches 0.26–0.42 m) past 1 m, at 480 × 270 (min-Z ~0.11 m) | `test_the_d435_sees_the_floor_the_arm_reaches` |

If the drawn nose puts it elsewhere, change `d435_pos` / `d435_tilt_deg` in `simulation/v11 sil/params.py`, rerun
`robot_export.py` and the tests: the robot's code reads the pose from the generated json.

## The concept and what it doesn't settle

- Camera to wedge: the camera's 1/4-20 tripod thread, a 1/2 in screw from a counterbore on the wedge's rear (screwed
  on the bench before the wedge goes on); a 2 × 3 mm lip along the camera's front-bottom edge stops it turning. The
  D435's two rear M3 threads would hold it better, but a back plate behind it doesn't fit before the lidar (VERIFY the
  rear holes' spacing, 45 mm per Intel, and the tripod thread's depth).
- Wedge to deck: two M3 × 12 pan heads up through the deck into 2.8 mm holes (6 mm engagement), the CAD rules.
- **Known conflict:** the wedge's front foot (x 173–198, z 66.5–87) runs over the sim's bumper stand-in (a ring of
  capsules at r 179–207 mm, up to z 77.5). The drawn bumper (r 205–217) is further out; settle it against the real deck
  and bumper in SolidWorks.
- The deck top (66.5 mm) and thickness (6 mm) are the sim's; the drawn deck differs (VERIFY).
- Not drawn: the USB-C cable's route (it must be USB 3 to the computer), strain relief.
