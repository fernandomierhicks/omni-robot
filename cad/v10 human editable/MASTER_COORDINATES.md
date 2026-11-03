# Editing the shared layout

Open `parts/human/V10H_master_layout.SLDPRT`. Its named global variables (Tools > Equations) control shared hole positions, bearing interfaces and roller geometry. Each variable has a comment. The `Layout …` sketches contain the shared axes; the `… Local` views express the same controls in a part's local frame.

Slave hole circles have native concentric relations to these derived guides. Their diameter and depth remain local fit/manufacturing controls. Native counterbores, fillets and chamfers follow their parent geometry. Body dimensions that must follow the layout use named aliases to the master controls.

| Controls | Consumers |
|---|---|
| `cradle_radius`, `cradle_phase` | Motor cradle and inner fork attachment |
| `joint_radius`, spoke phases | Hub A/B/C joining screws and pads |
| `cartridge_radius`, `cartridge_phase` | Removable cartridge and hub A |
| `output_radius`, `output_phase` | Hub A and dummy rotor output |
| `rear_radius`, `rear_phase` | Cradle and dummy rear mounting |
| `cover_radius`, `cover_phase` | Dummy case and rear cover |
| Foot coordinates (`guard_foot_y` far side, `pivot_foot_y` and `inner_ear_foot_x` pivot side) | Bench, guard and shock tower |
| Bar coordinates / pitch | Fork plates and cross bar |
| `pivot_y`, `pivot_z`, `pin_x` | Fork pivot, guard ears and shaft pin (bores, seats, shaft, clearance cuts follow the `Layout Pivot` marker) |
| Shock eye coordinates | Forks, guard and shock tower |
| `roller_axis_radius`, `row_distance`, `inner_roller_phase` | Hub axle supports, holes, pockets and roller barrel dimensions |
| Bearing bore / outside diameters | Bearing envelopes, seats and journals |

**Level pivot (Nov 2, `18_level_pivot_master.py`).** `pivot_y` 80.6, `pivot_z` 0: the pivot is level with the axle. A diameter cannot be zero, so `pivot_z` has no Control Measures circle; the `Layout Pivot` marker (measured from the guide datum) carries it, and every bore, seat and clearance cut is bound to that marker. The fork outlines and the guard ears hold their pivot lobe on the axle's horizontal by a relation and take only `pivot_y` as a dimension: a non-zero `pivot_z` would need that relation replaced by a dimension. Shock eyes: both at y 37, lower z 11, upper z 59.2. The cross bar is 5 mm lower (`bar_mid_z`/`bar_origin_z` 23).

The coordinate sketches use an explicit construction datum at (−100, −100) mm because SolidWorks drops model-origin dimensions when deriving sketches. Edit the named controls; leave the guide-datum and marker display dimensions unchanged. Right-plane coordinates are (−pod Z, pod Y), Front coordinates are (pod X, pod Y), and Top coordinates are (pod X, −pod Z).

Rebuild/save the master first, then affected parts, then the containing assembly. Cross-part equations may need two rebuild passes. Keep the files together so references resolve.

For individual part work, keep the master and one consumer open, save/close the consumer, then open the next. Open only one assembly at a time; its referenced part data necessarily loads with it. Do not preload the full library.

The construction scripts are initial-build/recovery tools. Normal dimension edits use the native SolidWorks feature tree and equations; they do not require regenerating the models from Python.
