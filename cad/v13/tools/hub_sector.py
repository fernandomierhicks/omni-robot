"""cad/v13: cut a sector of the real wheel hub round one spoke and save it as STEP, for fe_hub.py (Nov 11 whole-CAD review).

    python -B hub_sector.py ROW AZ [HALF_DEG] [OUT_DIR]        (omni-robot-sim interpreter)

ROW 1/2 and AZ = the spoke's azimuth in the pod y-z plane (y = cos, z = sin): spokes are at 30, 90, 150 ... in row 1 (head spokes at 30, 150, 270, pilot spokes
at 90, 210, 330) and at 0, 60, 120 ... in row 2 (pilot at 0, 120, 240, head at 60, 180, 300). The sector is the hub inside +-HALF_DEG (default 35) of the spoke, outside
r 29 (the drum and the spokes) and 9 mm past the row's webs on the other side of the channel. Default OUT_DIR: the system temp folder (the STEPs are 0.7-1.2 MB each).
"""
import math
import sys
import tempfile
from pathlib import Path

import build123d as b

ROOT = Path(__file__).resolve().parents[1]
row, az = int(sys.argv[1]), float(sys.argv[2])
HALF = float(sys.argv[3]) if len(sys.argv) > 3 else 35.0
OUT = Path(sys.argv[4]) if len(sys.argv) > 4 else Path(tempfile.gettempdir()) / 'hub_fe'
OUT.mkdir(parents=True, exist_ok=True)
hub = b.import_step(ROOT / 'step_export' / 'wheel_motor' / 'parts' / 'wheel_hub.step')
R = 120.0
pts = [(0, 0)] + [(R * math.cos(math.radians(az + d)), R * math.sin(math.radians(az + d))) for d in (-HALF, -HALF / 2, 0, HALF / 2, HALF)]
x0, x1 = (-9.0, 30.0) if row == 2 else (-30.0, 9.0)                       # the web side's far end is fixed in the FE; the collar and the other row's stubs stay in
wedge = b.extrude(b.Plane.YZ.offset(x0) * b.Polygon(*pts, align=None), amount=x1 - x0)
bore = b.Cylinder(29.0, 100.0).rotate(b.Axis.Y, 90)
sec = (hub & wedge) - bore
sol = sorted(sec.solids(), key=lambda s: -s.volume)
s = sol[0]
bb = s.bounding_box()
print(f'{len(sol)} solid(s), volume {s.volume:.0f} mm3, valid {s.is_valid}, bbox x {bb.min.X:.1f}..{bb.max.X:.1f}')
out = OUT / f'hub_sector_r{row}_a{int(az)}.step'
b.export_step(s, str(out))
print(out)
