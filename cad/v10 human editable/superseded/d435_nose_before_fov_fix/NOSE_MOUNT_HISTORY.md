## RealSense D435 on the nose

`V10H_d435_nose.SLDASM` contains Fernando's actual eight-body camera model
(`cad/other/Realsense to bucket/IntelRealsense_D435_Multibody.SLDPRT`), a
printed wedge and its mounting hardware. Both the main robot and floor-reach
assembly include this group, locked to the base. The source camera part is
referenced without modification; its hash is recorded in the validation.

The body centre is (190, 0, 111.5) mm and its optical direction is 30° down,
matching `robot_export.ARM`. The nominal source body centre is (0, 0, −12.5)
in its native coordinates; source +Z points through the lenses, +X is width,
and +Y is up. Its recorded placement is in `validation/d435_integration_geometry.json`.

The earlier nose concept is adapted to the actual z73 deck and actual camera
casting. Two Ø3.4 deck holes at (184.616, ±18) accept M3×12 screws from below
(6 mm deck, 6 mm nominal printed engagement). `REC_deck_front_D435` replaces
the old front deck. The camera's ¼-20 screw length/thread depth and cable
strain relief remain fit-check items; this is not a tested physical mount.

Exact camera clearances: 8.927 mm to the lidar body, 3.385 mm to its riser,
15.492 mm to the bumper. The printed wedge is one valid solid, with no volume
clashes against the camera or adjacent nose parts. Printed STEP parts are in
`step_export/d435_nose/parts/`; purchased camera/hardware are excluded there.
`50_add_realsense_sw.py inspect_camera|integrate` and
`51_realsense_geometry.py` reproduce this addition after the height update.
The high, drop and x270/310/340 floor-grasp checks were repeated with the
actual D435 and wedge included: no arm/robot or nonadjacent arm-link overlaps.
Both saved SolidWorks assemblies have 853 components and no mate errors.


