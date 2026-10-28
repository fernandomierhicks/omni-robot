"""Purchased parts for the assembly: CubeMars STEP plus simplified envelopes.

Envelope dimensions marked VERIFY come from listings, not drawings; check them
against the real part before printing anything that mates to them.
Each function returns a part in its own frame (documented per function).
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

from build123d import (Align, Axis, Box, Compound, Cylinder, Location, Part, Pos, Rot, import_step)

import dims as D

VENDOR = Path(__file__).with_name("vendor")
CENTER_BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)


@lru_cache(maxsize=None)
def ak40_10() -> Part | Compound:
    """CubeMars AK40-10 V3.0 KV170 from the official STEP. Frame: axis +Y,
    output face at y = -14.0, rear face at y = +26.2."""
    step = VENDOR / "AK40-10 V3.0 KV170.STEP"
    try:
        shape = import_step(str(step))
        shape.label = "AK40-10"
        return shape
    except Exception:
        body = Cylinder(D.AK_D / 2, 39.2, rotation=(90, 0, 0)).moved(Pos(0, 39.2 / 2 - 13.0, 0))
        out = Cylinder(D.AK_OUT_D / 2, 1.0, rotation=(90, 0, 0)).moved(Pos(0, -13.5, 0))
        return body + out


def gobilda_omni_96() -> Part:
    """goBILDA 3624-0014-0096 envelope. Frame: axle = X, centered (VERIFY width)."""
    rim = Cylinder(D.WHEEL_R, D.WHEEL_W, rotation=(0, 90, 0))
    hub = Cylinder(D.GB_BORE / 2, D.WHEEL_W + 2, rotation=(0, 90, 0))
    groove = Cylinder(D.WHEEL_R + 1, 3.0, rotation=(0, 90, 0)) - Cylinder(D.WHEEL_R - 9, 3.0, rotation=(0, 90, 0))
    p = rim - hub - groove
    p.label = "goBILDA omni 96"
    return p


def drone_motor_2806() -> Part:
    """T-Motor F90 2806.5 envelope. Frame: axis +Z, stator base at z = 0, shaft up (VERIFY)."""
    base = Cylinder(D.FM_D / 2 - 1, 5.0, align=CENTER_BOTTOM)
    bell = Cylinder(D.FM_D / 2, D.FM_H - 5.0, align=CENTER_BOTTOM).moved(Pos(0, 0, 5.0))
    shaft = Cylinder(D.FM_SHAFT / 2, 10.0, align=CENTER_BOTTOM).moved(Pos(0, 0, D.FM_H))
    p = base + bell + shaft
    p.label = "F90 2806.5"
    return p


def servo_270() -> Part:
    """Standard-size 270-degree servo (e.g. DS3225MG 270). Frame: output shaft on +Z
    at the origin, body below, long side along X (VERIFY dims)."""
    L, W, H = D.SERVO
    body = Box(L, W, H, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(L / 2 - 10.0, 0, -2.0))
    tabs = Box(D.SERVO_TAB[0], W, D.SERVO_TAB[1], align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(L / 2 - 10.0, 0, -10.0))
    spline = Cylinder(3.0, 4.0, align=CENTER_BOTTOM).moved(Pos(0, 0, -2.0))
    p = body + tabs + spline
    p.label = "servo 270"
    return p


def jetson_devkit() -> Part:
    """Jetson Orin Nano Super dev kit envelope (carrier 103 x 90.5, with fan). Frame: base at z = 0 (VERIFY)."""
    p = Box(103.0, 90.5, 6.0, align=CENTER_BOTTOM) + Box(70.0, 70.0, 28.0, align=CENTER_BOTTOM).moved(Pos(5, 0, 6))
    p.label = "Jetson Orin Nano devkit"
    return p


def rplidar_c1() -> Part:
    """RPLIDAR C1 envelope (55.6 x 55.6 x 41.3). Frame: base at z = 0 (VERIFY)."""
    p = Box(55.6, 55.6, 26.0, align=CENTER_BOTTOM) + Cylinder(25.0, 15.3, align=CENTER_BOTTOM).moved(Pos(0, 0, 26.0))
    p.label = "RPLIDAR C1"
    return p


def battery_7s1p() -> Part:
    """7S1P 21700 pack, cells 4 + 3 staggered on end, with BMS (est). Frame: top center at z = 0."""
    p = Box(98.0, 44.0, 74.0, align=(Align.CENTER, Align.CENTER, Align.MAX))
    p.label = "7S1P P50B pack"
    return p


def camera_ov9782() -> Part:
    """OV9782 global-shutter module (38 x 38 board, M12 lens). Frame: board back at z = 0, lens +Z (VERIFY)."""
    p = Box(38.0, 38.0, 4.0, align=CENTER_BOTTOM) + Cylinder(8.0, 16.0, align=CENTER_BOTTOM).moved(Pos(0, 0, 4.0))
    p.label = "OV9782 camera"
    return p


def esc_4in1() -> Part:
    p = Box(42.0, 42.0, 7.0, align=CENTER_BOTTOM)
    p.label = "TBS Lucid 8S 4in1"
    return p


def estop_22() -> Part:
    """22 mm mushroom e-stop. Frame: panel surface at z = 0, head up, body down."""
    p = Cylinder(20.0, 14.0, align=CENTER_BOTTOM).moved(Pos(0, 0, 8.0)) + Cylinder(14.0, 8.0, align=CENTER_BOTTOM) + \
        Cylinder(11.0, 40.0, align=(Align.CENTER, Align.CENTER, Align.MAX))
    p.label = "e-stop 22 mm"
    return p


def board(w, d, h, label) -> Part:
    p = Box(w, d, h, align=CENTER_BOTTOM)
    p.label = label
    return p
