"""BZ1's 3D model: the 12085 passive magnetic buzzer (Ø12 x 8.5 mm, 42 R) soldered on the carrier, in the 3D frame of its footprint
(omni_carrier:Buzzer_D12_P6.5-7.6_Slot), so KiCad's 3D view and its STEP export show it.

    & $env:USERPROFILE\\.venvs\\omni-robot-sim\\Scripts\\python.exe tools/buzzer_model.py      (from pcb/v1) -> models/buzzer_12085.step

Fernando, Nov 18 (components/buzzer: the vendor's 12085 listing, Ø12 x 8.5). Frame: pin 1 (+) at the origin, pin 2 along +x, z up from
the carrier's top. The pin pitch varies by maker (6.5 or 7.6 mm; the footprint's pad 2 is a slot that takes either): drawn at 7.0, the
body centred between the pins at x 3.5 like the footprint's outline. From the pictures: a black body standing on the board, a groove a
third of the way down, a Ø2 sound hole on top, a small green disc under it, Ø0.6 pins. The groove's place, the hole and the pins' length
are read from the pictures (ASSUMED); VERIFY the pitch and the + mark on the delivered buzzers.
"""
import os

from build123d import Color, Compound, Cylinder, Pos

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "models", "buzzer_12085.step")

D, H = 12.0, 8.5                     # body (vendor: 12 x 8.5)
PITCH = 7.0                          # pins 1 and 2 (6.5 or 7.6 by maker; drawn mid-range)
CX = 3.5                             # the body's centre over the footprint (its outline's centre)
BASE_T = 0.5                         # the green disc under the body
GROOVE_Z, GROOVE_W, GROOVE_DEPTH = 5.6, 0.4, 0.3
HOLE_D, HOLE_DEPTH = 2.0, 1.5
PIN_D, PIN_BELOW = 0.6, 3.0          # pins through the 1.6 mm board, trimmed ~1.4 below it

BLACK, GREEN, TIN = Color(0.06, 0.06, 0.06), Color(0.10, 0.45, 0.25), Color(0.80, 0.80, 0.78)


def build():
    parts = []

    def add(shape, colour, label):
        shape.color, shape.label = colour, label
        parts.append(shape)

    add(Pos(CX, 0, BASE_T / 2) * Cylinder(D / 2 - 0.4, BASE_T), GREEN, "buzzer_base")
    body = Pos(CX, 0, BASE_T + (H - BASE_T) / 2) * Cylinder(D / 2, H - BASE_T)
    body = body - Pos(CX, 0, GROOVE_Z) * (Cylinder(D / 2 + 1, GROOVE_W) - Cylinder(D / 2 - GROOVE_DEPTH, GROOVE_W))
    body = body - Pos(CX, 0, H - HOLE_DEPTH / 2) * Cylinder(HOLE_D / 2, HOLE_DEPTH + 0.01)
    add(body, BLACK, "buzzer_body")
    for k, x in enumerate((0.0, PITCH)):
        add(Pos(x, 0, (BASE_T - PIN_BELOW) / 2) * Cylinder(PIN_D / 2, BASE_T + PIN_BELOW), TIN, f"buzzer_pin_{k + 1}")
    return Compound(children=parts, label="buzzer_12085")


def main():
    from build123d import export_step
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    c = build()
    export_step(c, OUT)
    bb = c.bounding_box()
    print(f"{OUT}: {len(c.solids())} solids, x {bb.min.X:.2f}..{bb.max.X:.2f}, y {bb.min.Y:.2f}..{bb.max.Y:.2f}, z {bb.min.Z:.2f}..{bb.max.Z:.2f}")


if __name__ == "__main__":
    main()
