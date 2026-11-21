"""Coloured copies of the two uncoloured bought-part models, for the footprints (Nov 19 2026): KiCad's 3D view and the carrier's colour
STEP (tools/colour_step.py) then show them painted.

    & $env:USERPROFILE\\.venvs\\omni-robot-sim\\Scripts\\python.exe tools/paint_models.py      (from pcb/v1)

    models/teensy_4_1.step -> models/teensy_4_1_colour.step      Fernando's Teensy 4.1 (cad/v16/reference/electronics, from his
                                                                  SolidWorks part; AP203, no colours)
    models/can.step        -> models/can_colour.step             the SN65HVD230 module (same source)

Each face is painted by where it is in the model's own frame (sizes measured on the models, Nov 19):
  Teensy (USB at -x, centred; its board z -0.82..0.82, its headers' plastic z -3.61..-1.11, pins to z -10.31): the board green, the
  header plastic black, the pins (faces under 0.7 mm across) gold, the micro-B (x < -27.5, over the board) silver, the parts on it black.
  CAN module (x 0..16.4, its board y 0..1, the transceiver y 1..2): the board blue, the transceiver black.
"""
import os
import sys

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import colour_step as CS                      # noqa: E402

HERE = CS.HERE
GREEN, BLUE, BLACK = (0.10, 0.42, 0.22), (0.12, 0.30, 0.70), (0.10, 0.10, 0.10)
GOLD, SILVER, PLASTIC = (0.85, 0.68, 0.30), (0.78, 0.78, 0.80), (0.08, 0.08, 0.08)


def box(face):
    b = Bnd_Box()
    BRepBndLib.Add_s(face, b)
    lo, hi = b.CornerMin(), b.CornerMax()
    return lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()


def teensy(face):
    x0, y0, z0, x1, y1, z1 = box(face)
    if x1 - x0 < 0.7 and y1 - y0 < 0.7 and z1 - z0 > 0.05 or (x1 - x0 < 0.7 and y1 - y0 < 0.7 and z0 < -3.6):
        return GOLD                           # pins: thin, standing
    if z0 >= -0.83 and z1 <= 0.83:
        return GREEN                          # the board
    if z0 >= -3.62 and z1 <= -1.10:
        return PLASTIC                        # the headers' plastic
    if x0 < -27.5 and z0 >= 0.80:
        return SILVER                         # micro-B
    return BLACK


def can(face):
    x0, y0, z0, x1, y1, z1 = box(face)
    return BLUE if y1 <= 1.001 else BLACK


def paint(src, dst, rule):
    items = CS.leaves(os.path.join(HERE, "models", src))
    out, counts = [], {}
    for nm, shape, fmap, _ in items:
        cols = []
        for i in range(1, fmap.Extent() + 1):
            rgb = rule(fmap.FindKey(i))
            counts[rgb] = counts.get(rgb, 0) + 1
            cols.append(Quantity_Color(*rgb, Quantity_TOC_sRGB))
        out.append((os.path.splitext(dst)[0], shape, fmap, cols))
    n, total = CS.write(out, os.path.join(HERE, "models", dst), os.path.splitext(dst)[0])
    print(f"{dst}: {n} of {total} faces painted {sorted(counts.items(), key=lambda kv: -kv[1])}")


def main():
    paint("teensy_4_1.step", "teensy_4_1_colour.step", teensy)
    paint("can.step", "can_colour.step", can)


if __name__ == "__main__":
    main()
