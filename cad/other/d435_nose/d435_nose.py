"""The RealSense D435 on the arm robot's nose (design.md 4.11, Nov 1, proposed): a concept for SolidWorks, not a
finished part. It places the camera where the v11 sim has it (simulation/v11 sil/params.py d435_pos / d435_tilt_deg,
robot_export.ARM_D435) and draws a printed wedge under it, in the ROBOT frame (mm: x forward, z up from the floor), so
the files can be dropped into the SolidWorks assembly at the origin.

    python d435_nose.py          -> out/d435_nose.step (camera envelope, wedge, screws), out/d435_wedge.stl

The wedge: the camera's 1/4-20 tripod thread down through the wedge (screwed on the bench first, the head in a
counterbore on the wedge's rear), a lip along the camera's front-bottom edge against turning, and two M3 pan heads up
through the deck into 2.8 mm holes in the wedge (6 mm engagement), as the CAD rules. README.md lists what the
SolidWorks part must keep (the sim's checks) and what this concept doesn't settle.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

from build123d import Axis, Box, Color, Compound, Cylinder, Location, Plane, Polygon, Pos, export_step, export_stl, extrude

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"

# the sim's numbers (mm): keep in step with simulation/v11 sil/params.py (test_v11 checks the sim's placement)
CAM = (190.0, 0.0, 105.0)            # the body's middle
TILT = -30.0                         # deg: its view below the horizontal
BODY = (25.0, 90.0, 25.0)            # D435 depth (along the view), width, height (Intel; VERIFY on the unit)
DECK_TOP = 66.5                      # the sim's deck top under the nose (deck_z + 6 mm); the drawn deck: VERIFY
DECK_T = 6.0                         # deck thickness (sim)
WEDGE_W = 50.0                       # across the robot (y)
LIP = (2.0, 3.0)                     # mm: thickness along the view, height up the camera's front face
TRIPOD = dict(hole=6.6, cbore=12.0, cbore_from_cam=8.0)   # 1/4-20 clearance; counterbore stops 8 mm under the camera
M3 = dict(pilot=2.8, head_d=6.5, head_h=2.5, length=12.0, y=18.0)


def frame():
    t = math.radians(-TILT)
    f = (math.cos(t), -math.sin(t))          # along the view (x, z)
    u = (math.sin(t), math.cos(t))           # the camera's up
    return f, u


def at(c, f, u, a, b):
    """(x, z) of a point a along the view and b up the camera from c (x, z)."""
    return (c[0] + a * f[0] + b * u[0], c[1] + a * f[1] + b * u[1])


def build():
    f, u = frame()
    c = (CAM[0], CAM[2])
    hx, hz = BODY[0] / 2, BODY[2] / 2
    rear_bot, front_bot = at(c, f, u, -hx, -hz), at(c, f, u, hx, -hz)
    lip_out = at(c, f, u, hx + LIP[0], -hz)
    lip_top_out, lip_top_in = at(c, f, u, hx + LIP[0], -hz + LIP[1]), at(c, f, u, hx, -hz + LIP[1])
    # the side profile (x, z), extruded across y
    prof = [(rear_bot[0], DECK_TOP), (lip_out[0], DECK_TOP), lip_out, lip_top_out, lip_top_in, front_bot, rear_bot]
    wedge = extrude(Polygon(*[(x, z) for x, z in prof], align=None).moved(Location(Plane.XZ)), amount=WEDGE_W / 2,
                    both=True)
    # the tripod screw: along -u from the middle of the camera's bottom, its head's counterbore from the rear
    bot_mid = at(c, f, u, 0.0, -hz)
    ang = math.degrees(math.atan2(u[0], u[1]))         # u's tilt from vertical, about y
    axis_loc = Location((bot_mid[0], 0.0, bot_mid[1]), (0, ang, 0))
    hole = axis_loc * Pos(0, 0, -20) * Cylinder(TRIPOD["hole"] / 2, 40)
    cb = axis_loc * Pos(0, 0, -TRIPOD["cbore_from_cam"] - 20) * Cylinder(TRIPOD["cbore"] / 2, 40)
    mid_x = (rear_bot[0] + lip_out[0]) / 2
    pilots = [Pos(mid_x, s * M3["y"], DECK_TOP + 4) * Cylinder(M3["pilot"] / 2, 8) for s in (-1, 1)]
    wedge = wedge - hole - cb
    for p in pilots:
        wedge = wedge - p
    wedge.label, wedge.color = "d435_wedge (PETG)", Color(0.25, 0.25, 0.28)
    cam = Location((CAM[0], CAM[1], CAM[2]), (0, -TILT, 0)) * Box(*BODY)
    cam.label, cam.color = "RealSense D435 (envelope)", Color(0.6, 0.6, 0.62)
    screws = []
    for s in (-1, 1):
        z0 = DECK_TOP - DECK_T
        sh = Pos(mid_x, s * M3["y"], z0 + M3["length"] / 2) * Cylinder(1.5, M3["length"])
        hd = Pos(mid_x, s * M3["y"], z0 - M3["head_h"] / 2) * Cylinder(M3["head_d"] / 2, M3["head_h"])
        sc = sh + hd
        sc.label, sc.color = f"M3x{M3['length']:.0f} pan head (from under the deck)", Color(1.0, 0.0, 1.0)
        screws.append(sc)
    # 1/2 in from the counterbore's floor: ~4.7 mm into the camera's tripod thread (its depth: VERIFY)
    tri = axis_loc * Pos(0, 0, -TRIPOD["cbore_from_cam"] + 12.7 / 2) * Cylinder(3.175, 12.7)
    tri.label, tri.color = "1/4-20 x 1/2 in (camera to wedge)", Color(1.0, 0.0, 1.0)
    screws.append(tri)
    return cam, wedge, screws, dict(rear_x=rear_bot[0], front_x=lip_out[0], top_z=max(z for _, z in prof))


def main():
    cam, wedge, screws, ext = build()
    OUT.mkdir(exist_ok=True)
    asm = Compound(label="d435_nose (ROBOT frame, mm)", children=[cam, wedge, *screws])
    export_step(asm, str(OUT / "d435_nose.step"))
    export_stl(wedge, str(OUT / "d435_wedge.stl"))
    clash = (cam & wedge).volume
    print(f"wedge {wedge.volume / 1000:.1f} cm3, x {ext['rear_x']:.1f}-{ext['front_x']:.1f} mm, top {ext['top_z']:.1f} mm; "
          f"camera / wedge overlap {clash:.3f} mm3; one solid: {len(wedge.solids()) == 1}")
    print(f"wrote {OUT / 'd435_nose.step'} and {OUT / 'd435_wedge.stl'}")
    return 0 if clash < 1e-3 and len(wedge.solids()) == 1 else 1


if __name__ == "__main__":
    sys.exit(main())
