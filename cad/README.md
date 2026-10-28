# CAD

One folder per version; each keeps its own generated `out/` (STEP, STL, BOM, renders, checks).

| Folder | What it is |
| --- | --- |
| [`v8/`](v8/README.md) | **Latest** (Oct 26, proposed; design.md section 14): v7 plus a fork round each wheel, hex-drive pan heads, magenta fasteners, labelled boards, stronger cheeks. Both robots: `out/assembly.step` (catch) and `out/assembly_arm.step` (arm) |
| [`v7/`](v7/README.md) | The baseline as drawn on Oct 25 (design.md 12.2, 12.3, 29). The v11 sim reads it (`cad/v7/out/BOM.md`, `out/head_stl`), so it stays as it is |
| [`v2/`](v2/README.md) | The original v2 kiwi robot (moved here from `cad/` on Oct 26) |
