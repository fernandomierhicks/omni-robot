"""The ELRS receiver on its male pin header, plugged into J5 (a 1x4 2.54 mm female socket), and its T antenna: one STEP for J5's
footprint, so KiCad's 3D view and its STEP export (cad/v15 reads that) show the receiver where it sits on the carrier.

    & $env:USERPROFILE\\.venvs\\omni-robot-sim\\Scripts\\python.exe tools/elrs_model.py      (from pcb/v1) -> models/elrs_receiver_on_J5.step

Fernando, Nov 18: the receiver lives on the carrier, on male / female headers (components/radio receiver: HPXGRC ELRS 2.4 GHz, 20 x 11 mm,
four pads T R V G on one short edge, a U.FL at the other end, a 40 mm T antenna).

Frame = KiCad's 3D frame of footprint PinSocket_1x04_P2.54mm_Vertical: pin 1 at the origin, pins 2-4 along -y (the footprint's +y, down
on the board), z up from the carrier's top. Pin 1 = G, 2 = V, 3 = R, 4 = T (J5's order: GND, 5V, the receiver's RX, its TX). The receiver lies
flat over the carrier, components up, its body toward -x of the pin row (gen_pcb.py turns the footprint so that is inboard).

Stack (z, mm): socket body 0..8.5 (KiCad's model); the male header's plastic 8.5..11.0 under the receiver, its long pins (6 mm) down in the
socket, its short pins (3 mm) up through the receiver's pads; the receiver's board 11.0..12.0, its parts to ~13, the U.FL plug to ~14.5.
Sizes read from the vendor's pictures (VERIFY with calipers before ordering the board): **the pads' pitch reads 2.3-2.45 mm in the
pictures, a 2.54 mm header needs 2.54**, and the pads must be through-holes (they look plated through). Board thickness 1.0, pad row
centre 1.2 mm from the edge, the U.FL 2.7 mm from the far edge and 1.4 mm from the G side: ASSUMED. The antenna: 40 mm of Ø1.13 coax,
a T of 2 x 22 mm arms (ASSUMED, from the picture's proportions); it is drawn standing straight up, which is how it should be routed.
"""
import os

from build123d import Box, Color, Compound, Cylinder, Location, Pos, export_step, Rot

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "models", "elrs_receiver_on_J5.step")

PITCH = 2.54
N = 4
SOCKET_H = 8.5                  # KiCad's PinSocket_1x04 vertical body
SPACER = 2.5                    # the male header's plastic
PCB_T = 1.0                     # receiver board (ASSUMED)
L, Wd = 20.0, 11.0              # receiver board (vendor: 20 x 11)
PAD_EDGE = 1.2                  # pad row centre to the near short edge (ASSUMED)
UFL = (-16.2, 0.29)             # U.FL centre (read from the picture: 2.7 from the far edge, 1.4 from the G side)
COAX_D, COAX_L = 1.13, 40.0
T_ARM, T_ARM_D, T_SLEEVE = 22.0, 1.0, (3.0, 8.0)

BLACK, GOLD, DARK, BEIGE = Color(0.08, 0.08, 0.08), Color(0.85, 0.68, 0.25), Color(0.25, 0.25, 0.28), Color(0.85, 0.8, 0.7)


def build():
    z_board = SOCKET_H + SPACER
    yc = -(N - 1) * PITCH / 2                                   # the pin row's middle
    x_near, x_far = PAD_EDGE, PAD_EDGE - L                      # the board's short edges (pads' end, U.FL end)
    parts = []

    def add(shape, colour, label):
        shape.color, shape.label = colour, label
        parts.append(shape)

    add(Pos((x_near + x_far) / 2, yc, z_board + PCB_T / 2) * Box(L, Wd, PCB_T), BLACK, "elrs_board")
    for k in range(N):                                          # pads, top side (1.8 mm squares)
        add(Pos(0, -k * PITCH, z_board + PCB_T + 0.025) * Box(1.8, 1.8, 0.05), GOLD, f"elrs_pad_{k + 1}")
    add(Pos(-11.5, yc + 0.6, z_board + PCB_T + 0.45) * Box(4.0, 4.0, 0.9), DARK, "elrs_sx1280")      # the SX1280 (QFN)
    add(Pos(-15.5, yc - 2.6, z_board + PCB_T + 0.45) * Box(3.0, 3.0, 0.9), DARK, "elrs_pa_lna")
    ux, uy = UFL                                                # U.FL receptacle and the mated plug
    add(Pos(ux, uy, z_board + PCB_T + 0.625) * Box(2.6, 2.6, 1.25), GOLD, "elrs_ufl")
    zp = z_board + PCB_T + 1.25
    add(Pos(ux, uy, zp + 0.6) * Cylinder(1.0, 1.2), GOLD, "ufl_plug")
    # the male header: plastic under the board, pins from 6 mm below it (in the socket) to 3 mm above it (through the pads)
    add(Pos(0, yc, SOCKET_H + SPACER / 2) * Box(2.5, N * PITCH, SPACER), BLACK, "header_plastic")
    for k in range(N):
        add(Pos(0, -k * PITCH, (SOCKET_H - 6.0 + z_board + 3.0) / 2) * Box(0.64, 0.64, 9.0 + SPACER), GOLD, f"header_pin_{k + 1}")
    # the antenna: the coax leaves the plug sideways (toward the near edge), 3 mm, then straight up; the T at its end
    add(Pos(ux + 1.5, uy, zp + 0.6) * Rot(0, 90, 0) * Cylinder(COAX_D / 2, 3.0), BLACK, "coax_out")
    zc = zp + 0.6
    up = COAX_L - 3.0 - T_SLEEVE[1]
    add(Pos(ux + 3.0, uy, zc + up / 2) * Cylinder(COAX_D / 2, up), BLACK, "coax_up")
    add(Pos(ux + 3.0, uy, zc + up + T_SLEEVE[1] / 2) * Cylinder(T_SLEEVE[0] / 2, T_SLEEVE[1]), BLACK, "t_sleeve")
    zt = zc + up + T_SLEEVE[1] + T_ARM_D / 2
    add(Pos(ux + 3.0, uy, zt) * Rot(90, 0, 0) * Cylinder(T_ARM_D / 2, 2 * T_ARM), BLACK, "t_arms")
    return Compound(children=parts, label="elrs_receiver_on_J5")


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    c = build()
    export_step(c, OUT)
    bb = c.bounding_box()
    print(f"{OUT}: {len(c.solids())} solids, x {bb.min.X:.2f}..{bb.max.X:.2f}, y {bb.min.Y:.2f}..{bb.max.Y:.2f}, z {bb.min.Z:.2f}..{bb.max.Z:.2f}")


if __name__ == "__main__":
    main()
