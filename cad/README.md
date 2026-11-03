# CAD

One folder per version; each keeps its own generated `out/` (STEP, STL, BOM, renders, checks).

| Folder | What it is |
| --- | --- |
| [`v10 human editable/`](v10%20human%20editable/ASSEMBLIES.md) | **Active whole robot:** native wheel pods and articulated SO-101 with imported body parts; reconciled arm/head heights, retained rear mast, Nov 2 centre-bar D435 with separate FOV models. Open `V10H_robot.SLDASM` or `V10H_robot_FOV.SLDASM`; print-readiness gaps remain documented |
| [`v9/`](v9/README.md) | **Latest wheel prototype** (Oct 31): 140 mm wheel, 6804 bearings, separate PLA / medium TPU, 1.5 mm journals and relieved/bevelled roller pockets (≥0.5 mm body clearance required). Wheel-only kit / STEP / X2D plates; earlier full-robot assemblies are frozen snapshots |
| [`v8/`](v8/README.md) | Oct 26–27 proposal: DM-2325 drive, double-sided forks, face mast, electronics bay, hex-drive pan heads and labelled boards |
| [`v7/`](v7/README.md) | The baseline as drawn on Oct 25 (design.md 12.2, 12.3, 29). The v11 sim reads it (`cad/v7/out/BOM.md`, `out/head_stl`), so it stays as it is |
| [`v2/`](v2/README.md) | The original v2 kiwi robot (moved here from `cad/` on Oct 26) |
| [`other/d435_nose/`](other/d435_nose/README.md) | Superseded Nov 1 concept: nose D435 placement. Its imager views were obstructed; the active camera is on the v10 centre bar |
