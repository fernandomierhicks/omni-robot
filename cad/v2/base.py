"""Printed parts of the drive base. All dimensions in mm.

Frames: see dims.py. Parts that are placed per wheel (pod, adapter) are modeled
in the POD frame: origin at the wheel center on the axle, +X along the axle
toward the robot center, +Z up.
"""

from __future__ import annotations

import math

from build123d import (Align, Axis, Box, BuildPart, BuildSketch, Circle, Cylinder, Location, Locations, Mode,
                       Part, Plane, Polygon, Pos, RegularPolygon, extrude, revolve)

import dims as D

M3_CLEAR, M3_INSERT = 3.4, 4.2           # through hole, heat-set insert hole (M3 x 4 x 5.7)
M4_CLEAR, M4_INSERT = 4.4, 5.6
M25_CLEAR = 2.8

# wheel stack along the axle (POD frame x): wheel | adapter | motor
WHEEL_IN = D.WHEEL_W / 2                       # wheel inner face
ADAPTER_T = 6.0
MOTOR_OUT = WHEEL_IN + ADAPTER_T               # motor output face
MOTOR_REAR = MOTOR_OUT + D.AK_L                # motor rear face
BULK_T = 6.0
FLANGE_TOP = D.DECK_Z - D.WHEEL_R              # pod z of the deck underside (60)
FLANGE_T = 6.0
GUARD_X = -(WHEEL_IN + 4.0)                    # inner face of the outer wheel guard


def _cyl_x(r, length, x0):
    """Cylinder along +X from x0 to x0+length."""
    return Cylinder(r, length, rotation=(0, 90, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x0, 0, 0))


def wheel_adapter() -> Part:
    """AK40-10 output -> goBILDA 96 mm omni hub. POD frame, sits x = WHEEL_IN .. MOTOR_OUT.
    3 x M2.5x6 SHCS into the motor output (counterbored from the wheel side), 3 x M4 bolts
    through the wheel's 32 mm pattern into captive nuts, Ã˜14 spigot into the wheel bore,
    Ã˜14.9 pilot into the motor's Ã˜15 recess. Print PETG 100% infill, or machine in aluminum."""
    x0 = WHEEL_IN
    p = _cyl_x(22.0, ADAPTER_T, x0)
    p += _cyl_x(13.85 / 2, 6.0, x0 - 6.0)                       # into the wheel bore
    p += _cyl_x(14.9 / 2, 1.0, x0 + ADAPTER_T)                  # pilot into the motor output
    for a in (30, 150, 270):                                    # motor output pattern
        y, z = D.polar(D.AK_OUT_PCD / 2, a)
        p -= _cyl_x(M25_CLEAR / 2, 20, x0 - 7).moved(Pos(0, y, z))
        p -= _cyl_x(2.6, 2.8, x0 - 0.01).moved(Pos(0, y, z))    # head counterbore, wheel side
    for a in (0, 90, 180):                                      # wheel pattern (4th hole collides with an M2.5)
        y, z = D.polar(D.GB_PCD_OUTER / 2, a)
        p -= _cyl_x(M4_CLEAR / 2, 20, x0 - 7).moved(Pos(0, y, z))
        nut = extrude(RegularPolygon(4.05, 6), 3.4).rotate(Axis.Y, 90)  # M4 nut, 7 mm AF
        p -= nut.moved(Pos(x0 + ADAPTER_T - 3.4 + 0.01, y, z))
    p.label = "wheel_adapter"
    return p


def wheel_pod() -> Part:
    """Motor bulkhead + top flange + outer wheel guard, one print (flange face down).
    POD frame. The AK40-10 bolts to the bulkhead through its rear 4 x M2.5 (PCD 47);
    the flange bolts up to the center plate (inserts) and the deck sector above."""
    with BuildPart() as pod:
        # bulkhead behind the motor
        with Locations(Pos(MOTOR_REAR + BULK_T / 2, 0, (FLANGE_TOP - 34.0) / 2)):
            Box(BULK_T, 70.0, FLANGE_TOP + 34.0)
        # top flange, under the deck
        x_in, x_out = 104.0, GUARD_X - 4.0
        with Locations(Pos((x_in + x_out) / 2, 0, FLANGE_TOP - FLANGE_T / 2)):
            Box(x_in - x_out, 84.0, FLANGE_T)
        # outer wheel guard (covers the wheel's outer face down to just below the axle)
        with Locations(Pos(GUARD_X - 2.0, 0, (FLANGE_TOP - 6.0) / 2)):
            Box(4.0, 2 * (D.WHEEL_R + 8.0), FLANGE_TOP + 6.0)
        # gussets between bulkhead and flange
        for y in (-30.0, 30.0):
            with BuildSketch(Plane.XZ.offset(-y)):
                with Locations((0, 0)):
                    Polygon((MOTOR_REAR + BULK_T, FLANGE_TOP - FLANGE_T), (MOTOR_REAR + BULK_T + 28, FLANGE_TOP - FLANGE_T),
                            (MOTOR_REAR + BULK_T, FLANGE_TOP - FLANGE_T - 28), align=None)
            extrude(amount=2.0, both=True)
        # motor rear pattern + center clearance
        for a in (0, 90, 180, 270):
            y, z = D.polar(D.AK_REAR_PCD / 2, a)
            with Locations(Pos(MOTOR_REAR, y, z)):
                Cylinder(M25_CLEAR / 2, 40, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        with Locations(Pos(MOTOR_REAR, 0, 0)):
            Cylinder(13.0, 40, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        # flange holes: two into the center plate, two into the deck sector (heat-set inserts)
        for x, y in ((80.0, 25.0), (80.0, -25.0), (96.0, 25.0), (96.0, -25.0), (0.0, 32.0), (0.0, -32.0)):
            with Locations(Pos(x, y, FLANGE_TOP)):
                Cylinder(M3_INSERT / 2, 10.0, mode=Mode.SUBTRACT)
    p = pod.part
    p.label = "wheel_pod"
    return p


def pod_to_robot(th_deg: float) -> Location:
    """POD frame -> ROBOT frame for the wheel at angle th_deg."""
    th = math.radians(th_deg)
    return Location((D.WHEEL_BASE_R * math.cos(th), D.WHEEL_BASE_R * math.sin(th), D.WHEEL_R), (0, 0, th_deg + 180))


def center_plate() -> Part:
    """Central chassis plate, ROBOT frame, z = DECK_Z .. DECK_Z + 6. Its outer 12 mm is a
    rabbet (top half only) that laps over the deck sectors' inner lips."""
    z0 = D.DECK_Z
    p = Cylinder(D.CENTER_R, D.DECK_T, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(0, 0, z0))
    ring = Cylinder(D.CENTER_R + 1, D.DECK_T / 2, align=(Align.CENTER, Align.CENTER, Align.MIN)) - \
        Cylinder(D.CENTER_R - 12.0, D.DECK_T / 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= ring.moved(Pos(0, 0, z0))
    holes = []
    for th in D.WHEEL_ANGLES:                                       # pod flange bolts
        loc = pod_to_robot(th)
        for px, py in ((80.0, 25.0), (80.0, -25.0), (96.0, 25.0), (96.0, -25.0)):
            v = loc * Pos(px, py, 0)
            holes.append((v.position.X, v.position.Y, M3_CLEAR))
    for a in D.STANDOFF_ANGLES:                                     # payload-plate standoffs
        x, y = D.polar(D.STANDOFF_R, a)
        holes.append((x, y, M4_CLEAR))
    for k in range(6):                                              # sector inner lips
        for da in (-15, 15):
            x, y = D.polar(D.CENTER_R - 6.0, 60 * k + da)
            holes.append((x, y, M3_CLEAR))
    for x, y in ((50, 24), (50, -24), (-50, 24), (-50, -24)):       # battery tray
        holes.append((x, y, M3_CLEAR))
    for x, y in ((-60 + 43, 29), (-60 + 43, -29), (-60 - 43, 29), (-60 - 43, -29)):   # Jetson standoffs (VERIFY)
        holes.append((x, y, 2.9))
    for x, y, dia in holes:
        p -= Cylinder(dia / 2, 30).moved(Pos(x, y, z0))
    p -= Cylinder(14.0, 30).moved(Pos(0, 0, z0))                    # CAN + power up to the Jetson / payload
    p.label = "center_plate"
    return p


def deck_sector(k: int) -> Part:
    """60-degree deck sector k (centered at 60k deg, so sectors 1, 3, 5 sit over the wheel
    pods), ROBOT frame. Inner lip (bottom half) slides under the center plate's rabbet;
    bumper arcs (offset 30 deg) clip over the outer edge and bridge the sector joints."""
    z0 = D.DECK_Z
    a0, a1 = 60 * k - 30, 60 * k + 30
    with BuildPart() as sec:
        with BuildSketch(Plane.XY.offset(z0)):
            Circle(D.DECK_R)
            Circle(D.CENTER_R + 0.3, mode=Mode.SUBTRACT)
        extrude(amount=D.DECK_T)
        with BuildSketch(Plane.XY.offset(z0)):                        # inner lip, bottom half
            Circle(D.CENTER_R + 0.5)
            Circle(D.CENTER_R - 12.0, mode=Mode.SUBTRACT)
        extrude(amount=D.DECK_T / 2 - 0.2)
    # keep only this 60-degree wedge
    big = 3 * D.DECK_R
    pts = [(0.0, 0.0)] + [(big * math.cos(math.radians(a)), big * math.sin(math.radians(a))) for a in (a0, (a0 + a1) / 2, a1)]
    keep = extrude(Plane.XY.offset(z0 - 1) * Polygon(*pts, align=None), D.DECK_T + 2)
    p = sec.part & keep
    # lip bolts, matching the center plate
    for da in (-15, 15):
        x, y = D.polar(D.CENTER_R - 6.0, 60 * k + da)
        p -= Cylinder(M3_CLEAR / 2, 30).moved(Pos(x, y, z0))
    th_c = 60 * k
    if any(abs(((th_c - w + 180) % 360) - 180) < 1 for w in D.WHEEL_ANGLES):
        loc = pod_to_robot(th_c)
        for px, py in ((0.0, 32.0), (0.0, -32.0)):                   # into the pod flange inserts
            v = loc * Pos(px, py, 0)
            p -= Cylinder(M3_CLEAR / 2, 30).moved(Pos(v.position.X, v.position.Y, z0))
    else:
        # lightening slots on the sectors that carry nothing below
        for rr in (150.0, 190.0):
            x, y = D.polar(rr, th_c)
            p -= Box(26.0, 60.0, 30.0).rotate(Axis.Z, th_c).moved(Pos(x, y, z0))
    for x, y in LIDAR_HOLES + ESTOP_HOLE:                             # only where they fall on this sector
        p -= Cylinder((M3_CLEAR if (x, y) in LIDAR_HOLES else 22.5) / 2, 30).moved(Pos(x, y, z0))
    p.label = f"deck_sector_{k}"
    return p


def bumper_arc(k: int) -> Part:
    """TPU 95A bumper, 60 degrees centered on 30 + 60k deg (spans a sector joint).
    Hollow C-channel clips over the deck edge (the 8 mm air gap behind the edge makes it
    squashy); the 3 mm skirt hangs to 45 mm above the floor in front of the wheels."""
    zd, t = D.DECK_Z, D.DECK_T
    r_in, r_wall, r_out = D.DECK_R - 7.0, D.DECK_R + 8.0, D.DECK_R + 11.0
    with BuildPart() as b:
        with BuildSketch(Plane.XZ):
            Polygon((r_in, zd - 3), (r_wall, zd - 3), (r_wall, 45.0), (r_out, 45.0), (r_out, zd + t + 3),
                    (r_in, zd + t + 3), (r_in, zd + t + 0.2), (r_wall, zd + t + 0.2), (r_wall, zd - 0.2),
                    (r_in, zd - 0.2), align=None)
        revolve(axis=Axis.Z, revolution_arc=60.0)
    p = b.part.rotate(Axis.Z, 60 * k)
    p.label = f"bumper_arc_{k}"
    return p


def battery_tray() -> Part:
    """Cradle under the center plate for the 7S1P pack (strap slots), ROBOT frame."""
    z0 = D.DECK_Z
    W, Dp, H, t = 104.0, 50.0, 78.0, 3.0
    outer = Box(W + 2 * t, Dp + 2 * t, H + t, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(0, 0, z0))
    inner = Box(W, Dp, H, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(0, 0, z0 + 0.01))
    p = outer - inner
    flange = Box(W + 30, Dp + 2 * t, 4.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(0, 0, z0))
    p += flange - Box(W, Dp, 10).moved(Pos(0, 0, z0))
    for x, y in ((50, 24), (50, -24), (-50, 24), (-50, -24)):
        p -= Cylinder(M3_INSERT / 2, 12).moved(Pos(x, y, z0))
    for x in (-30, 30):                                              # hook-and-loop strap slots
        p -= Box(22.0, 80.0, 4.0).moved(Pos(x, 0, z0 - H - t / 2))
    p -= Box(30.0, 80.0, 20.0).moved(Pos(0, 0, z0 - 20))             # side window for leads/BMS
    p.label = "battery_tray"
    return p


def lidar_riser() -> Part:
    """Raises the RPLIDAR C1 so its scan plane clears the deck hardware. ROBOT frame."""
    z0 = D.DECK_Z + D.DECK_T
    p = Box(66.0, 66.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(150.0, 0, z0))
    for x, y in LIDAR_HOLES:
        p -= Cylinder(M3_CLEAR / 2, 30).moved(Pos(x, y, z0))
    p -= Box(30.0, 30.0, 30.0).moved(Pos(150.0, 0, z0))
    p.label = "lidar_riser"
    return p


def payload_plate() -> Part:
    """Payload interface plate, PAYLOAD frame (z = -6 .. 0). 25 mm grid of M4 heat-set
    inserts, two Ã˜4 dowel pins (asymmetric so a module only fits one way), four of the
    grid inserts take the module's captive thumbscrews, Ã˜28 connector hole."""
    p = Cylinder(D.PLATE_R, D.PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MAX))
    for a in D.STANDOFF_ANGLES:
        x, y = D.polar(D.STANDOFF_R, a)
        p -= Cylinder(M4_CLEAR / 2, 30).moved(Pos(x, y, 0))
        p -= Cylinder(4.2, 3.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(x, y, 0.01))
    for i in range(-4, 5):
        for j in range(-4, 5):
            x, y = 25.0 * i, 25.0 * j
            r = math.hypot(x, y)
            if r > 98 or r < 20:
                continue
            if any(math.hypot(x - sx, y - sy) < 12 for sx, sy in (D.polar(D.STANDOFF_R, a) for a in D.STANDOFF_ANGLES)):
                continue
            p -= Cylinder(M4_INSERT / 2, 30).moved(Pos(x, y, 0))
    for x, y in PIN_XY:
        p -= Cylinder(3.95 / 2, 30).moved(Pos(x, y, 0))
    p -= Cylinder(14.0, 30)
    p.label = "payload_plate"
    return p


PIN_XY = ((87.5, 37.5), (-37.5, -62.5))         # asymmetric: a module fits only one way
LIDAR_HOLES = [(150 + x, y) for x, y in ((25, 25), (25, -25), (-25, 25), (-25, -25))]
ESTOP_HOLE = [D.polar(185.0, 135.0)]           # rear-left sector (no pod underneath)
THUMBSCREW_XY = ((50.0, 50.0), (50.0, -50.0), (-50.0, 50.0), (-50.0, -50.0))

