"""Purchased parts: makers' STEP files where they publish them, envelopes from datasheets otherwise.

Each function returns (shape, frame described in its docstring). Envelopes marked VERIFY come from listings, not
drawings: check the real part before printing anything that mates to it. STEP sources (in vendor/):
  DM-S2325-1EC.stp                  Damiao DM-2325 (aifitlab / Damiao, Sept 2025) + its installation drawing PDF
  pololu-25d-9.7-encoder.step       Pololu 25D 9.7:1 gearmotor with 48 CPR encoder (pololu.com, 25D 3D models zip)
  pololu-1999-hub-6mm-m3.step       Pololu universal aluminium hub, 6 mm shaft, 6 x M3
  pololu-1997-hub-4mm-m3.step       Pololu universal aluminium hub, 4 mm shaft, 4 x M3
  so101/*.step                      TheRobotStudio SO-101 follower arm, every printed part and the STS3215 servo
                                    (github.com/TheRobotStudio/SO-ARM100 at the sim's pinned commit, Apache-2.0;
                                    so101/SOURCES.md)
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

from build123d import (Align, Box, Cone, Cylinder, Location, Part, Pos, Rot, Sphere, Torus, import_step)

import dims as D

VENDOR = Path(__file__).with_name("vendor")
CB = (Align.CENTER, Align.CENTER, Align.MIN)
CT = (Align.CENTER, Align.CENTER, Align.MAX)


def _label(p, s):
    p.label = s
    return p


@lru_cache(maxsize=None)
def dm2325(trim: bool = False) -> Part:
    """Damiao DM-2325 (DM-S2325-1EC) from Damiao's STEP. Frame: axis +Z, output face at z = 0, shaft toward +Z,
    connector toward +X (36-40 mm behind the face). trim=True cuts the cable stub off at r 19 (the leads bend back
    along the body in the motor cup's channel; the STEP shows them straight out)."""
    s = import_step(str(VENDOR / "DM-S2325-1EC.stp")).moved(Location((0, 0, 0), (90, 0, 0)) * Pos(0, -32.2, 0))
    if trim:
        s = s & Cylinder(19.0, 200)
    return _label(s, "Damiao DM-2325")


@lru_cache(maxsize=None)
def pololu_25d() -> Part:
    """Pololu #4842 9.7:1 25Dx63L HP 12 V with 48 CPR encoder (Pololu STEP). Frame: axis +Z, face plate at z = 0,
    shaft toward +Z, encoder leads toward +Y. Face: 2 x M3 at x = +-8.5 (6 mm deep)."""
    return _label(import_step(str(VENDOR / "pololu-25d-9.7-encoder.step")), "Pololu 25D 9.7:1 + encoder")


@lru_cache(maxsize=None)
def hub_1999() -> Part:
    """Pololu #1999 hub (6 mm, 6 x M3 on PCD 19). Frame: flange mounting face at z = 0, flange z 0..-5,
    set-screw boss z -5..-9.2 (toward the motor), shaft along Z."""
    s = import_step(str(VENDOR / "pololu-1999-hub-6mm-m3.step")).moved(Pos(0, 0, -2.5) * Rot(180, 0, 0))
    return _label(s, "Pololu 1999 hub")


@lru_cache(maxsize=None)
def hub_1997() -> Part:
    """Pololu #1997 hub (4 mm, 4 x M3 on PCD 12.7). Frame: mounting face at z = 0, body z 0..-5."""
    s = import_step(str(VENDOR / "pololu-1997-hub-4mm-m3.step")).moved(Pos(0, 0, -2.5))
    return _label(s, "Pololu 1997 hub")


@lru_cache(maxsize=None)
def f90() -> Part:
    """T-Motor F90 2806.5 1300KV envelope. Frame: stator mounting face at z = 0, bell and M5 shaft toward +Z.
    4 x M3 on a 19 x 19 mm square (VERIFY; some 2806.5 batches are 16 x 16)."""
    f = D.F90
    p = Cylinder(f["base_d"] / 2, 4.0, align=CB) + Cylinder(f["d"] / 2, f["h"] - 6.0, align=CB).moved(Pos(0, 0, 4.0))
    p += Cylinder(f["d"] / 2 - 4, 2.0, align=CB).moved(Pos(0, 0, f["h"] - 2.0))
    p += Cylinder(f["shaft"] / 2, 12.0, align=CB).moved(Pos(0, 0, f["h"]))
    for dx in (-f["mount"] / 2, f["mount"] / 2):
        for dy in (-f["mount"] / 2, f["mount"] / 2):
            p -= Cylinder(1.25, 5.0, align=CB).moved(Pos(dx, dy, -0.01))
    return _label(p, "T-Motor F90 2806.5")


def sts3250() -> Part:
    """Feetech STS3250 envelope (STS3215 case, VERIFY). Frame: output spline axis = +Z at the origin, case top at
    z = 0, case below, long side along +X (spline 11 mm from the case end)."""
    f = D.STS3250
    body = Box(f["l"], f["w"], f["h"], align=CT).moved(Pos(f["l"] / 2 - 11.0, 0, 0))
    spline = Cylinder(3.0, 4.0, align=CB)
    return _label(body + spline, "Feetech STS3250")


def servo_horn() -> Part:
    """25T aluminium disc horn, dia 25 x 3, 4 x M3 on PCD 14 + centre M3. Frame: spline face at z = 0, +Z away."""
    p = Cylinder(12.5, 3.0, align=CB).moved(Pos(0, 0, 0.5)) + Cylinder(4.0, 0.5, align=CB)
    p -= Cylinder(3.1, 10)                                                     # spline bore
    for k in range(4):
        x, y = D.polar(D.STS3250["horn_pcd"] / 2, 90 * k + 45)
        p -= Cylinder(1.6, 10).moved(Pos(x, y, 0))
    return _label(p, "servo horn 25T disc")


def a2m12() -> Part:
    """Slamtec RPLIDAR A2M12 envelope (dia 76 x 41, 190 g). Frame: base underside at z = 0, scan plane at
    z = A2M12['scan_z'] (VERIFY), cable exit toward -X. Mount: 4 x M2.5 on a 65 mm circle (VERIFY)."""
    f = D.A2M12
    p = Cylinder(f["d"] / 2, f["base_h"], align=CB) + Cylinder(f["head_d"] / 2, f["h"] - f["base_h"], align=CB).moved(
        Pos(0, 0, f["base_h"]))
    n, d, pcd = f["holes"]
    for k in range(n):
        x, y = D.polar(pcd / 2, 45 + 360 / n * k)
        p -= Cylinder(1.0, 12, align=CB).moved(Pos(x, y, -0.01))
    return _label(p, "RPLIDAR A2M12")


def jetson() -> Part:
    """Jetson Orin Nano Super dev kit envelope: carrier 100 x 79 on 4 mm feet, module, heatsink + fan to 34.8 mm.
    Frame: underside of the feet at z = 0, I/O edge toward -X. Holes: 4 x M2.5/M3 on 86 x 58 (VERIFY)."""
    p = Box(100.0, 79.0, 1.6, align=CB).moved(Pos(0, 0, 4.0))
    p += Box(70.0, 70.0, 18.0, align=CB).moved(Pos(3.0, 0, 5.6))            # module + heatsink
    p += Box(85.0, 70.0, 11.2, align=CB).moved(Pos(4.0, 0, 23.6)) & Box(70.0, 70.0, 11.2, align=CB).moved(Pos(3.0, 0, 23.6))
    p += Box(18.0, 48.0, 14.0, align=CB).moved(Pos(-41.0, 0, 5.6))         # I/O connectors at -X
    for x in (-43, 43):
        for y in (-29, 29):
            p += Cylinder(2.5, 4.0, align=CB).moved(Pos(x, y, 0))
            p -= Cylinder(1.7, 20).moved(Pos(x, y, 0))
    return _label(p, "Jetson Orin Nano Super dev kit")


def battery() -> Part:
    """7S1P Molicel P50B pack, 7 cells side by side in one flat row, wrapped (est 153 x 72 x 23, 612 g).
    Frame: top centre at z = 0, cells along Y, balance + main leads at +X."""
    b = D.BATTERY
    p = Box(b["l"], b["w"], b["h"], align=CT)
    return _label(p, "7S1P P50B pack")


def board(w: float, d: float, h: float, label: str) -> Part:
    """Generic box envelope. Frame: underside at z = 0."""
    return _label(Box(w, d, h, align=CB), label)


def pcb(w: float, d: float, h: float, holes, label: str, inset: float = 6.0, hole_d: float = 3.2) -> Part:
    """PCB envelope: a 1.6 mm board with its mounting holes, components up to h inside an inset keep-out (so the
    screw heads have room). Frame: board underside at z = 0, holes at (dx, dy) from the centre."""
    p = Box(w, d, 1.6, align=CB)
    for dx, dy in holes:
        p -= Cylinder(hole_d / 2, 5).moved(Pos(dx, dy, 0))
    if h > 1.6:
        p += Box(w - 2 * inset, d - 2 * inset, h - 1.6, align=CB).moved(Pos(0, 0, 1.6))
    return _label(p, label)


@lru_cache(maxsize=None)
def camera() -> Part:
    """OV9782 global-shutter module, 38 x 38 board, M12 lens (Arducam-style, VERIFY). Frame: board back at z = 0,
    lens toward +Z. 4 x M2 on 34 x 34."""
    p = Box(38.0, 38.0, 1.6, align=CB) + Box(16.0, 16.0, 6.0, align=CB).moved(Pos(0, 0, 1.6))
    p += Cylinder(7.0, 16.0, align=CB).moved(Pos(0, 0, 7.6))
    for x in (-17.0, 17.0):
        for y in (-17.0, 17.0):
            p -= Cylinder(1.1, 5).moved(Pos(x, y, 0))
    return _label(p, "OV9782 camera")


def on_off_switch(panel: float = 4.0) -> Part:
    """22 mm round latching push button (metal anti-vandal, LED ring), with its ring nut under a `panel` mm panel.
    Frame: panel top surface at z = 0, the button up, the body and its tags down (VERIFY: body 30 mm long, the deck
    hole is 24 mm)."""
    p = Cylinder(12.5, 2.5, align=CB) + Cylinder(9.0, 1.5, align=CB).moved(Pos(0, 0, 2.5))      # bezel, button
    p += Cylinder(11.0, 30.0, align=CT) + Cylinder(8.0, 6.0, align=CT).moved(Pos(0, 0, -30.0))   # body, tags
    p += Cylinder(14.0, 3.0, align=CT).moved(Pos(0, 0, -panel)) - Cylinder(11.0, 20)           # ring nut
    return _label(p, "on/off switch 22 mm")


@lru_cache(maxsize=None)
def shock(L: float = D.SHOCK_L) -> Part:
    """1/16-scale RC oil shock (est, VERIFY), 50 mm eye to eye extended, 10 mm stroke, 3 mm eyes, ~11 mm spring.
    Frame: lower eye at the origin, axis +Z to the upper eye at z = L (its current length), eye bolts along X."""
    p = Cylinder(5.5, L - 14, align=CB).moved(Pos(0, 0, 7.0))
    for z in (0.0, L):
        p += Cylinder(4.0, 6.0, rotation=(0, 90, 0)).moved(Pos(0, 0, z)) - Cylinder(1.6, 8, rotation=(0, 90, 0)).moved(Pos(0, 0, z))
    p += Cylinder(2.0, 6.0, align=CB).moved(Pos(0, 0, 3.0)) + Cylinder(2.0, 6.0, align=CT).moved(Pos(0, 0, L - 3.0))
    return _label(p, "RC shock 50 mm")


# the sim's SO-101 mesh (simulation/v11 sil/assets/so101/<mesh>.stl) -> TheRobotStudio's STEP of the same part, in
# the same frame (checked to 0.1 mm on every part's bounding box). The wrist's servo mesh is the STS3215 without its
# horn; there is no STEP of that, so it gets the servo with its horn.
SO101_STEP = {
    "base_so101_v2": "Base_SO101", "base_motor_holder_so101_v1": "Base_motor_holder_SO101",
    "motor_holder_so101_base_v1": "Motor_holder_SO101_Base", "motor_holder_so101_wrist_v1": "Motor_holder_SO101_Wrist",
    "rotation_pitch_so101_v1": "Rotation_Pitch_SO101", "upper_arm_so101_v1": "Upper_arm_SO101",
    "under_arm_so101_v1": "Under_arm_SO101", "wrist_roll_pitch_so101_v2": "Wrist_Roll_Pitch_SO101",
    "wrist_roll_follower_so101_v1": "Wrist_Roll_Follower_SO101", "moving_jaw_so101_v1": "Moving_Jaw_SO101",
    "waveshare_mounting_plate_so101_v2": "WaveShare_Mounting_Plate_SO101", "sts3215_03a_v1": "STS3215_03a",
    "sts3215_03a_no_horn_v1": "STS3215_03a"}


@lru_cache(maxsize=None)
def so101_part(mesh: str) -> Part:
    """The SO-101's part for one of the sim's meshes, from TheRobotStudio's STEP (mm), in the mesh file's frame."""
    name = SO101_STEP[mesh]
    return _label(import_step(str(VENDOR / "so101" / f"{name}.step")), f"SO-101 {name}")


def ball() -> Part:
    return _label(Sphere(D.RB), "foam ball 74 mm")


@lru_cache(maxsize=None)
def ir_pair() -> Part:
    """IR break-beam emitter or receiver, 5 mm LED in a 20 x 10 x 8 housing (Adafruit 2168 style). Frame: lens
    axis +Z, housing back at z = 0."""
    p = Box(10.0, 8.0, 10.0, align=CB) + Cylinder(2.5, 5.0, align=CB).moved(Pos(0, 0, 10.0))
    return _label(p, "IR break-beam 5 mm")


def spring_feed() -> Part:
    """Compression spring for a feed-wheel arm, 5 mm OD x 12 mm free (McMaster 9657K, ~0.8 N/mm, est). Frame: axis
    +Z from z = 0."""
    return _label(Cylinder(2.5, 12.0, align=CB) - Cylinder(1.9, 13.0, align=CB), "feed spring")
