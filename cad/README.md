# CAD

One folder per version; each is self-contained (its own scripts, `reference/`, checks and exports). **Read [solidworks.md](solidworks.md) first** (the main CAD reference and printed-part rules), then [AGENTS.md](AGENTS.md) and the version's README. Adoption status is in [design.md](../design.md) 4.4; a README's own status line may lag it.

| Folder | What it is |
| --- | --- |
| [`v15/`](v15/README.md) | **Working CAD (Nov 14 proposal, not formally adopted):** the mid deck: a dovetailed deck at z 80–100 with the pods hanging under it, generated lidar fins, the HRB pack under the deck, service-stand pads, the Nov 16 colours (`tools/palette.py`). `./regen_all.sh` rebuilds it; SolidWorks viewing assembly `solidworks/V15_robot.SLDASM` |
| [`v14/`](v14/README.md) | **Last adopted whole robot (Nov 13):** layout F, a clean copy of v13 with the 687-bearing roller wheel baselined; scripts and STEPs (`step_export/`) |
| [`v13/`](v13/README.md) | Nov 5: the D435 moved to the arm riser; whole-robot SolidWorks assembly; keeps the wiring-harness study, jigs (`motor_jig.py`, `pod_jig.py`), print-fit coupons and wheel-noise rounds |
| [`v12/`](v12/README.md) | Nov 4: layout F rounds 13–15, clean SolidWorks assembly |
| [`v11/`](v11/README.md) | Nov 2: compact wheel + AK40-10 sprung pod; layout rounds to 12 |
| [`v10 human editable/`](v10%20human%20editable/ASSEMBLIES.md) | Native SolidWorks robot (wheel pods, articulated SO-101); the v11 sim's `ARM` is a coarse model of it. Its 43 mm-high fork pivot is superseded (pivot level with the axle since v11) |
| [`v9/`](v9/README.md) | Oct 31 wheel prototype: 140 mm wheel, 6804 bearings, PLA / TPU; frozen source |
| [`v8/`](v8/README.md) | Oct 26–27 proposal: DM-2325 drive, double-sided forks, face mast, electronics bay |
| [`v7/`](v7/README.md) | The baseline as drawn on Oct 25. The v11 sim reads it (`cad/v7/out/BOM.md`, `out/head_stl`), so it stays as it is |
| [`v2/`](v2/README.md) | The original v2 kiwi robot |
| [`other/`](other/) | Side concepts and shared reference parts, e.g. the superseded Nov 1 nose D435 (`other/d435_nose/`) |
