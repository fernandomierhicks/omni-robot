"""The catch / launch module (design.md 22.4 chute pocket, 23.1 feed motors, 24 actuator choice), MODULE frame:
x forward, z up the funnel axis, origin at the module base on the centreline (as the sim's robot_model.geometry).

Ball path (y = 0 plane, from the sim): the ball drops through the Ø84 throat into a square pocket (78 mm between the
pads), onto a padded floor sloping down at 40 deg into the nip of two feed wheels (Ø40 x 12, axes perpendicular to
the floor, at y = +-53). Feeding, they push it down the chute into a slick PETG J-bend (40 mm ball-centre radius) that
turns it up the 30 deg launch run into the two Ø70 x 28 flywheels at y = +-67.

Parts (module):
  chute_half L / R   pocket walls, chute floor, J, launch run and a keel under the J, split on y = 0 (print outer
                     wall down); four M3 joint screws across the split; 6 mm foam pads line the pocket
  collar             under the net's lower edge; carries the chute halves, the net lacing, six strut sockets and the
                     two pivot arms, which run under the net to bosses outside the cone (the pivot, 13 mm above the
                     throat, is inside the cone)
  rim_arc x 6        the Ø330 rim (60 deg each; joints between the struts), carbon struts to the collar
  feed_carriage x 2  Pololu 25D bracket, hinged on an M3 along the wheel axis, a torsion spring holds it on an
                     adjustable M3 stop (design.md 22.2: a rigid stop, preload so the wheels just touch a ball)
  fw_mount x 2       F90 plate on the launch line on a solid wedge (y 48-87) that fills the space behind and under
                     it, down to the line the payload plate leaves at +40 deg of tilt; 4 screws into the keel / wall
  feed wheel and flywheel hubs (PETG) and tyres (TPU 95A)
Base side (fixed, ROBOT frame; the module's interface): yoke_L (608 + 8 mm shaft), yoke_R (STS3250 pocket + cap),
camera bar posts on the front wheel houses.
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import (Align, Axis, Box, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, Torus, extrude, revolve)

import dims as D
import hw
import vendor as V

CB = (Align.CENTER, Align.CENTER, Align.MIN)
CT = (Align.CENTER, Align.CENTER, Align.MAX)
RB = D.RB
W_PAD = D.W_IN                    # 39: pad surface
W_IN = D.W_IN + D.PAD_T           # 45: printed wall inner face
W_OUT = W_IN + D.WALL_T           # 48
C = (math.cos(D.CHUTE), math.sin(D.CHUTE))       # down the chute
L = (-math.sin(D.CHUTE), math.cos(D.CHUTE))      # chute-floor normal (up)
U, NRM = D.U, D.NRM
P0, W = D.HOLD, D.FEED_C
CJ, RHO, J1, NIP = D.J_C, D.J_RHO, D.J_END, D.NIP
TH, BE = D.THETA, D.CHUTE
S_REST = math.hypot(W[0] - P0[0], W[1] - P0[1])
J0 = (W[0] + (S_REST + 4.0) * C[0], W[1] + (S_REST + 4.0) * C[1])
SIDE_END = D.GEO.side_wall_end * D.MM            # 77.8: side walls stop short of the flywheels
Z_TOP = D.THROAT_Z                               # 136.9
COLLAR_T = 5.0
TUBE_TOP = Z_TOP - COLLAR_T                      # the chute halves stop under the collar
S_BACK = 38.0                                    # the chute floor starts this far behind the resting ball
LAUNCH_END = D.LAUNCH_RUN - 30.0                 # the run stops 30 mm short of the nip (sim: L2 - 0.03)
J_T = 5.0                                        # J and launch-run wall thickness (slick PETG, no pad)


def _lin(a, b, n):
    return [a + (b - a) * k / (n - 1) for k in range(n)]


def _pt(o, d, s):
    return (o[0] + s * d[0], o[1] + s * d[1])


def xz_prism(pts, y0: float, y1: float) -> Part:
    """Prism of an (x, z) outline between y0 and y1."""
    return extrude(Plane.XZ.offset(-y0) * Polygon(*pts, align=None), y1 - y0, dir=(0, 1, 0))


def _y_cyl(r, x, z, y0, y1):
    return Cylinder(r, y1 - y0, rotation=(-90, 0, 0), align=CB).moved(Pos(x, y0, z))


def band(o0: float, o1: float, s_back: float, s_end: float, chute=True, bend=True, run=True):
    """(x, z) band between offsets o0 < o1 below the ball path (offset = distance from the ball centre)."""
    sides = []
    for o in (o1, o0):
        lst = []
        if chute:
            lst += [_pt(_pt(P0, C, -s_back), L, -o), _pt(J0, L, -o)]
        if bend:
            lst += [(CJ[0] + (RHO + o) * math.sin(ph), CJ[1] - (RHO + o) * math.cos(ph)) for ph in _lin(BE, TH, 24)]
        if run:
            lst += [_pt(J1, NRM, -o), _pt(_pt(J1, U, s_end), NRM, -o)]
        sides.append(lst)
    return sides[0] + sides[1][::-1]


def floor_z(x: float, o: float = RB) -> float:
    """z of the chute floor (offset o below the path) at x, along the straight chute."""
    x0, z0 = _pt(P0, L, -o)
    return z0 + (x - x0) / C[0] * C[1]


# --- feed wheels -------------------------------------------------------------------------------------------------------
FEED_FACE = 17.5                  # wheel centre to the Pololu face along -L: wheel 6 + #1997 hub 5 + 6 mm plate + 0.5
FEED_HINGE = 32.0                 # hinge axis this far up the chute (-C) from the wheel centre
HINGE_DY = 7.0                    # ... and this far outboard of the wheel centre
KNUCKLE = (-23.5, -11.5)          # knuckle along the wheel axis (local z), flush with the arm; lugs either side
SPRING_GAP = 3.8                  # torsion spring between the lower lug and the knuckle
FEED_Y = D.FEED_Y
FEED_ANG = math.degrees(math.atan2(L[0], L[1]))


def feed_frame(side: int) -> Location:
    """At a feed wheel's centre: local z = the wheel axis (L), local x = C (down the chute), local y = global y."""
    return Pos(W[0], side * FEED_Y, W[1]) * Rot(0, FEED_ANG, 0)


def _hinge_frame(side: int) -> Location:
    return feed_frame(side) * Pos(-FEED_HINGE, side * HINGE_DY, 0)


def _feed_window() -> Part:
    """Clearance through the +y side wall, pad and floor for the feed wheel (+4), its hub, the carriage plate (+-3 of
    swing) and the motor body (+1.5)."""
    wheel = Cylinder(D.FEED_WHEEL[0] / 2 + 4.0, D.FEED_WHEEL[1] + 8.0)
    hub = Cylinder(D.HUB_1997["d"] / 2 + 2.0, 8.0, align=CT).moved(Pos(0, 0, -4.0))
    plate = Box(40.0, 36.0, 10.0, align=CT).moved(Pos(0, 0, -9.0))
    motor = Cylinder(D.POLOLU_25D["d"] / 2 + 1.5, 66.0, align=CT).moved(Pos(0, 0, -FEED_FACE))
    return (wheel + hub + plate + motor).moved(feed_frame(1))


def _hinge_lugs() -> Part:
    """Two lugs on the +y side wall straddling the carriage's knuckle (with room for the torsion spring)."""
    out = None
    for z0, z1 in ((KNUCKLE[0] - SPRING_GAP - 5.0, KNUCKLE[0] - SPRING_GAP), (KNUCKLE[1] + 0.5, KNUCKLE[1] + 5.5)):
        lug = Box(12.0, HINGE_DY + FEED_Y - W_OUT + 6.0, z1 - z0, align=(Align.CENTER, Align.MAX, Align.MIN))
        lug = lug.moved(_hinge_frame(1) * Pos(0, 6.0, z0))
        out = lug if out is None else out + lug
    return out & Box(400, 200, 400, align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(0, W_OUT - 2.0, 0))


@lru_cache(maxsize=None)
def feed_carriage() -> Part:
    """Holds the 25D's face (its 2 x M3 at +-8.5 along y, counterbored); an arm up the chute to the hinge knuckle;
    a boss with the M3 stop screw (bears on the side wall). Feed frame of the +y wheel."""
    z0, z1 = -FEED_FACE, -FEED_FACE + 6.0
    p = Box(30.0, 30.0, 6.0, align=CB).moved(Pos(0, 0, z0))
    p -= Cylinder(4.2, 20).moved(Pos(0, 0, z0))                                          # bushing
    for dy in (-8.5, 8.5):
        p -= hw.cbore(6.0, top=z1).moved(Pos(0, dy, 0))
    p += Box(FEED_HINGE - 10.0, 10.0, 6.0, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Pos(-10.0, HINGE_DY, z0))
    p += Cylinder(5.0, KNUCKLE[1] - KNUCKLE[0], align=CB).moved(Pos(-FEED_HINGE, HINGE_DY, KNUCKLE[0]))
    p -= Cylinder(D.CLR_M3 / 2, 60).moved(Pos(-FEED_HINGE, HINGE_DY, 0))
    # stop boss: an M3 through it along -y, its tip on the side wall's outer face
    p += Box(10.0, 9.0, 6.0, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(Pos(-18.0, HINGE_DY - 5.0, z0))
    p -= Cylinder(D.TAP_M3 / 2, 30, rotation=(-90, 0, 0)).moved(Pos(-18.0, 0, z0 + 3.0))
    p.label = "feed_carriage"
    return p


@lru_cache(maxsize=None)
def feed_wheel_hub() -> Part:
    """PETG hub for the Ø40 x 12 TPU tyre, on a Pololu #1997 (4 x M3 on PCD 12.7). Feed frame (local z = axis)."""
    h = D.FEED_WHEEL[1]
    p = Cylinder(D.FEED_WHEEL[0] / 2 - 4.0, h) - Cylinder(2.6, h + 1)
    for k in range(4):
        x, y = D.polar(D.HUB_1997["pcd"] / 2, 90 * k + 45)
        p -= hw.cbore(h, top=h / 2).moved(Pos(x, y, 0))
    p.label = "feed_wheel_hub"
    return p


@lru_cache(maxsize=None)
def feed_wheel_tyre() -> Part:
    r = D.FEED_WHEEL[0] / 2
    p = Cylinder(r, D.FEED_WHEEL[1]) - Cylinder(r - 4.0 - 0.1, D.FEED_WHEEL[1] + 1)
    p.label = "feed_wheel_tyre"
    return p


# --- flywheels -----------------------------------------------------------------------------------------------------------
FW_ANG = math.degrees(math.atan2(NRM[0], NRM[1]))   # -30: local z = NRM, local x = U
MOTOR_FACE_Z = -(D.FW_W / 2 + 0.5 + D.F90["h"])     # -40.5: F90 mount face (local z)
FW_PLATE_T = 6.0


def fw_frame(side: int) -> Location:
    return Pos(NIP[0], side * D.FW_Y, NIP[1]) * Rot(0, FW_ANG, 0)


@lru_cache(maxsize=None)
def flywheel_hub() -> Part:
    """PETG hub, Ø58 x 28, clamped on the F90's M5 shaft by its prop nut (counterbore from the outer face)."""
    h = D.FW_W
    p = Cylinder(D.FW_R - 6.0, h) - Cylinder(2.6, h + 1)
    p -= Cylinder(5.5, h, align=CB).moved(Pos(0, 0, -h / 2 + 12.0))
    for k in range(8):
        x, y = D.polar(17.0, 45 * k)
        p -= Cylinder(4.0, h - 6).moved(Pos(x, y, 0))
    p.label = "flywheel_hub"
    return p


@lru_cache(maxsize=None)
def flywheel_tyre() -> Part:
    p = Cylinder(D.FW_R, D.FW_W) - Cylinder(D.FW_R - 6.0 - 0.2, D.FW_W + 1)
    p.label = "flywheel_tyre"
    return p


KEEL = dict(x=(52.0, 104.0), y=(0.0, W_IN + 0.5), z0=2.0)
FW_Y = (W_OUT, D.FW_Y + 20.0)                        # fw_mount wedge: against the side wall's outer face .. the plate's edge
FW_SCREWS = ((64.0, 5.5), (77.0, 5.5), (90.0, 5.5), (100.0, 5.5))   # (x, z) along -y into the keel (through the wall
FW_SCREW_SEAT = 54.5                                 # at x < 78); heads on pocket floors at this y


def tilt_floor_z(x: float, margin: float = 2.0) -> float:
    """Lowest module z at x that stays `margin` over the payload plate at the largest forward tilt."""
    a = math.radians(D.TILT_RANGE[1])
    s, c = math.sin(a), math.cos(a)
    return D.PIVOT_M[1] + (D.PLATE_TOP + margin - D.PIVOT_R[1] + s * (x - D.PIVOT_M[0])) / c


def _keel_outline():
    """(x, z) of the keel: from z0 up to the J's outer wall underside."""
    x0, x1 = KEEL["x"]
    r = RHO + RB + J_T
    arc = []
    for ph in _lin(math.asin((x1 - CJ[0]) / r), math.asin((x0 - CJ[0]) / r), 16):
        arc.append((CJ[0] + r * math.sin(ph), CJ[1] - r * math.cos(ph) + 0.5))
    return [(x0, KEEL["z0"]), (x1, KEEL["z0"])] + arc


FW_PLATE = 40.0                                      # motor plate along the launch line (local x); its front-bottom
                                                     # edge is the module's lowest point at +40 deg (1.5 mm over the plate)


def _fw_local(lx: float, lz: float):
    """MODULE (x, z) of a point in the +y flywheel's frame (local x along the launch line, local z along NRM)."""
    v = fw_frame(1) * Pos(lx, 0, lz)
    return v.position.X, v.position.Z


@lru_cache(maxsize=None)
def fw_mount() -> Part:
    """F90 plate (4 x M3 on 19 x 19, Ø14 centre) on a solid wedge, modelled for the +y flywheel, MODULE frame.

    The wedge spans y 48..87 (the motor's full width) and fills the space behind and under the plate: back to the
    chute half's side wall and keel (x 60), down to the module's underside, and under the plate down to the line the
    payload plate leaves at full forward tilt (the plate's front edge is the module's lowest point at +40 deg, so the
    plate can't get thicker downward). It stays under the plate's top face wherever the F90 and the flywheel are.
    Four M3 x 16 along -y into the keel (through the side wall's foot at x < 78), heads on the floors of 7 mm pockets
    from the outer face. Print with the outer face (y 87) down: every section is inside the one below it."""
    f = fw_frame(1)
    z0 = MOTOR_FACE_Z
    h = FW_PLATE / 2
    plate = Box(FW_PLATE, 2 * (FW_Y[1] - D.FW_Y), FW_PLATE_T, align=CT).moved(Pos(0, 0, z0))
    p = plate.moved(f) & Box(400, 400, 400, align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(0, FW_Y[0], 0))
    front_bot = _fw_local(h, z0 - FW_PLATE_T)
    back_top = _fw_local(-h, z0)
    # wedge profile (x, z): along the module's underside, up the +40 deg clearance line to the plate's front-bottom
    # edge, through the plate to its back-top edge, back over the J to the side wall
    b = tilt_floor_z(1.0) - tilt_floor_z(0.0)
    x1 = (1.5 - tilt_floor_z(0.0)) / b
    x2 = front_bot[0] - 1.0
    prof = [(60.0, 1.5), (x1, 1.5), (x2, min(tilt_floor_z(x2), front_bot[1] - 0.5)), front_bot, back_top, (78.0, 32.0), (60.0, 30.0)]
    wedge = xz_prism(prof, FW_Y[0], FW_Y[1])
    wedge += xz_prism(prof, KEEL["y"][1], FW_Y[0] + 0.5) & Box(200, 200, 200, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(
        Pos(D.GEO.side_wall_end * D.MM + 0.7, 0, 0))                # on the keel's face beyond the side wall
    p += wedge
    # the F90 and its screws: centre bore, clearance holes, head pockets under the plate
    for dx in (-9.5, 9.5):
        for dy in (-9.5, 9.5):
            p -= hw.clear(30.0, top=z0 + 1.0).moved(f * Pos(dx, dy, 0))
            p -= Cylinder(3.5, 30.0, align=CT).moved(f * Pos(dx, dy, z0 - FW_PLATE_T))
    p -= Cylinder(7.0, 60.0).moved(f * Pos(0, 0, z0))
    # keel screws: clearance from the pocket floor in, 7 mm pockets out to the outer face
    for x, z in FW_SCREWS:
        p -= _y_cyl(D.CLR_M3 / 2, x, z, KEEL["y"][1] - 1, FW_SCREW_SEAT + 0.1)
        p -= _y_cyl(3.5, x, z, FW_SCREW_SEAT, FW_Y[1] + 1)
    p.label = "fw_mount"
    return p


# --- chute halves ---------------------------------------------------------------------------------------------------------
JOINT_TABS = [(-W_OUT - 6.0, 118.0), (-W_OUT - 6.0, 92.0), (60.0, 6.0), (W_OUT + 6.0, 118.0)]   # heads clear the flange
FLANGE_R = 62.0
COLLAR_SCREWS = [(58.0 * math.cos(math.radians(a)), 58.0 * math.sin(math.radians(a))) for a in (30, 90, 150, 210, 270, 330)]
EXIT_BEAM_S = 10.0                # feed-exit beam this far down the bend from J0 (the sim's spot is inside the
                                  # feed-wheel window, where there's no wall to hold the sensor)
CATCH_BEAM = _pt(_pt(P0, C, -12.0), L, 10.0)   # a chord through the resting ball, clear of the feed wheels (the sim's
                                               # beam crosses the ball's centre, where the wheels' sensors would sit)
BEAMS = (CATCH_BEAM, (CJ[0] + RHO * math.sin(BE + EXIT_BEAM_S / RHO), CJ[1] - RHO * math.cos(BE + EXIT_BEAM_S / RHO)))


@lru_cache(maxsize=None)
def chute_half(side: int = 1) -> Part:
    """One half (side +1: y >= 0), modelled for +y and mirrored."""
    # side wall: a band around the ball path plus the pocket, between the pad and the outer face
    prof_band = band(-(RB + 8.0), RB + D.PAD_T + D.WALL_T + 3.0, S_BACK + 4.0, LAUNCH_END + 6.0)
    z_back = floor_z(-W_OUT, RB + D.PAD_T + D.WALL_T) - 4.0
    pocket = [(-W_OUT, z_back), (W_OUT, z_back - 50.0), (W_OUT, TUBE_TOP), (-W_OUT, TUBE_TOP)]
    clip = [(-W_OUT, 0.0), (SIDE_END, 0.0), (SIDE_END, TUBE_TOP), (-W_OUT, TUBE_TOP)]
    wall = (xz_prism(prof_band, W_IN, W_OUT) + xz_prism(pocket, W_IN, W_OUT)) & xz_prism(clip, W_IN - 1, W_OUT + 1)
    p = wall
    # floors from the centreline to the wall: printed chute floor under its pad; slick J and launch run
    chute = band(RB + D.PAD_T, RB + D.PAD_T + D.WALL_T, S_BACK + 4.0, 0, bend=False, run=False)
    p += xz_prism(chute, 0.0, W_IN + 0.5) & xz_prism(clip, -1, 100)
    p += xz_prism(band(RB, RB + J_T, 0, LAUNCH_END, chute=False), 0.0, W_IN + 0.5)
    # the chute floor's printed part meets the J under the pad's end (the pad stops at J0)
    p += xz_prism([_pt(J0, L, -(RB + D.PAD_T + D.WALL_T)), _pt(J0, L, -RB), _pt(_pt(J0, C, 6.0), L, -RB),
                   _pt(_pt(J0, C, 6.0), L, -(RB + D.PAD_T + D.WALL_T))], 0.0, W_IN + 0.5)
    # back wall and front wall (behind their pads)
    p += Box(D.WALL_T, W_IN + 0.5, TUBE_TOP - z_back, align=(Align.MAX, Align.MIN, Align.MAX)).moved(Pos(-W_IN, 0, TUBE_TOP))
    p += Box(D.WALL_T, W_IN + 0.5, TUBE_TOP - D.FRONT_WALL_Z, align=(Align.MIN, Align.MIN, Align.MAX)).moved(Pos(W_IN, 0, TUBE_TOP))
    # keel under the J (stiffens the half, carries the flywheel mount)
    p += xz_prism(_keel_outline(), 0.0, KEEL["y"][1])
    # top flange under the collar (7 mm: the collar's M3 x 10 get 7 mm of thread where no wall is under the flange)
    fl = Cylinder(FLANGE_R, 7.0, align=CT).moved(Pos(0, 0, TUBE_TOP)) & Box(200, 100, 100, align=(
        Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(0, 0, TUBE_TOP))
    p += fl - Box(2 * W_IN, W_IN + 0.1, 20, align=(Align.CENTER, Align.MIN, Align.MAX)).moved(Pos(0, -0.01, TUBE_TOP + 1))
    # joint tabs on the centreline
    for x, z in JOINT_TABS:
        p += Box(12.0, 8.0, 12.0, align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(x, 0, z))
    # feed wheel / motor window, hinge lugs, IR beam windows and sensor pads
    p -= _feed_window()
    p += _hinge_lugs()
    p -= Cylinder(D.CLR_M3 / 2, 80).moved(_hinge_frame(1))
    for x, z in BEAMS:
        p += Box(16.0, 3.0, 16.0, align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(x, W_OUT, z))
        p -= _y_cyl(2.7, x, z, W_PAD - 1, W_OUT + 4)
    # holes: collar screws, joint screws, flywheel mount flange
    for x, y in COLLAR_SCREWS:
        if y > 0:
            p -= hw.tap(10.0, top=TUBE_TOP).moved(Pos(x, y, 0))
    for x, z in FW_SCREWS:                       # flywheel mount: through the side wall's foot (x < 78) into the keel
        p -= _y_cyl(D.TAP_M3 / 2, x, z, 36.0, W_OUT + 0.1)
    p &= Box(400, 200, 400, align=(Align.CENTER, Align.MIN, Align.CENTER))
    if side > 0:                     # L: thread-forming holes for the joint screws coming from R
        for x, z in JOINT_TABS:
            p -= _y_cyl(D.TAP_M3 / 2, x, z, -1, 9)
        p.label = "chute_half"
        return p
    p = _mirror(p, "chute_half")    # R: clearance, and an access bore to the keel joint's head
    for x, z in JOINT_TABS:
        p -= _y_cyl(D.CLR_M3 / 2, x, z, -9, 1)
        if x > 0 and z < 20:
            p -= _y_cyl(3.4, x, z, -KEEL["y"][1] - 1, -8.0)
    p.label = "chute_half"
    return p


def _mirror(p: Part, label: str) -> Part:
    m = p.mirror(Plane.XZ)
    m.label = label
    return m


# --- collar, rim, struts, net --------------------------------------------------------------------------------------------
STRUT_ANGLES = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)
COLLAR_R = 78.0
STRUT_R0, STRUT_Z0 = 72.0, Z_TOP - 3.0
RIM_TUBE = 8.0
PIVOT_BOSS_Y = (76.0, 90.0)


def net_z(r: float) -> float:
    """z of the funnel net at radius r (55 deg from the axis)."""
    return Z_TOP + (r - D.FUNNEL_THROAT_R) / math.tan(D.FUNNEL_ANGLE)


def _under_net(gap: float) -> Part:
    r0, r1 = D.FUNNEL_THROAT_R, 260.0
    pts = [(0.0, -100.0), (r1, -100.0), (r1, net_z(r1) - gap), (r0, net_z(r0) - gap), (0.0, net_z(r0) - gap)]
    return revolve(Plane.XZ * Polygon(*pts, align=None), Axis.Z)


def strut_ends(a: float):
    ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
    r1, z1 = D.FUNNEL_RIM_R + 6.0, D.RIM_Z - 9.0
    return (STRUT_R0 * ca, STRUT_R0 * sa, STRUT_Z0), (r1 * ca, r1 * sa, z1)


def _seg_cyl(p0, p1, r, s0, s1) -> Part:
    """Cylinder of radius r along p0 -> p1, from s0 to s1 mm past p0."""
    d = [p1[i] - p0[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in d))
    u = [v / n for v in d]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, u[2]))))
    yaw = math.degrees(math.atan2(u[1], u[0]))
    return Cylinder(r, s1 - s0, align=CB).moved(Pos(*p0) * Rot(0, 0, yaw) * Rot(0, tilt, 0) * Pos(0, 0, s0))


SOCKET = (6.0, 14.0)              # strut socket: starts / ends this far along the strut from the collar point


def strut_span(a: float):
    """(s0, s1): the carbon tube runs from s0 past the collar point to s1 (8 mm into each socket)."""
    p0, p1 = strut_ends(a)
    n = math.dist(p0, p1)
    return SOCKET[1] - 8.0, n - (SOCKET[1] - 8.0)


@lru_cache(maxsize=None)
def collar() -> Part:
    p = Cylinder(COLLAR_R, COLLAR_T, align=CT).moved(Pos(0, 0, Z_TOP)) - Cylinder(D.FUNNEL_THROAT_R, 20).moved(Pos(0, 0, Z_TOP))
    for k in range(24):                                                 # net lacing
        x, y = D.polar(D.FUNNEL_THROAT_R + 4.0, 15 * k + 7.5)
        p -= Cylinder(1.25, 20).moved(Pos(x, y, Z_TOP))
    for x, y in COLLAR_SCREWS:
        p -= hw.cbore(COLLAR_T, top=Z_TOP).moved(Pos(x, y, 0))
    for a in STRUT_ANGLES:                                              # strut sockets, under the net
        s0, s1 = strut_ends(a)
        p += _seg_cyl(s0, s1, 7.0, -4.0, SOCKET[1]) & _under_net(1.5)
    for a in STRUT_ANGLES:
        s0, s1 = strut_ends(a)
        p -= _seg_cyl(s0, s1, RIM_TUBE / 2 + 0.15, SOCKET[0], 60)
    px, pz = D.PIVOT_M
    y0, y1 = PIVOT_BOSS_Y
    for s in (1, -1):                                                   # pivot arms under the net
        arm = xz_prism([(px - 8, TUBE_TOP), (px + 8, TUBE_TOP), (px + 8, pz + 4), (px - 8, pz + 4)], COLLAR_R - 14, y1)
        arm = (arm & _under_net(3.0)) + _y_cyl(9.0, px, pz, y0, y1)
        p += arm if s > 0 else arm.mirror(Plane.XZ)
    p -= Cylinder(D.FUNNEL_THROAT_R, 400).moved(Pos(0, 0, Z_TOP))
    # +y boss: 8 mm shaft clamped by an M3 across a slit; -y boss: 4 x M3 on PCD 14 for the servo horn
    p -= _y_cyl(4.0, px, pz, y0 + 2.0, y1 + 1)
    p -= Box(1.2, y1 - y0 - 1, 12.0, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(Pos(px, y0 + 2.0, pz))
    p -= Cylinder(D.CLR_M3 / 2, 30, rotation=(0, 90, 0)).moved(Pos(px, (y0 + y1) / 2 + 1, pz + 6.0))
    for k in range(4):
        x, z = D.polar(D.STS3250["horn_pcd"] / 2, 90 * k + 45)
        p -= _y_cyl(D.TAP_M3 / 2, px + x, pz + z, -y1 - 0.01, -y1 + 9.0)
    p.label = "collar"
    return p


RIM_JOINTS = tuple(a + 30.0 for a in STRUT_ANGLES)
RIM_Z0, RIM_Z1 = D.RIM_Z - 10.0, D.RIM_Z + 2.0     # 12 mm: 6 mm laps, so the lap screws get 6 mm of thread


@lru_cache(maxsize=None)
def rim_arc(k: int) -> Part:
    """Rim arc k (60 deg, centred on strut k), lap joints (2 x M3 each end) with its neighbours."""
    a_c = STRUT_ANGLES[k]
    a0, a1 = a_c - 30.0, a_c + 30.0
    r0, r1 = D.FUNNEL_RIM_R, D.FUNNEL_RIM_R + 12.0
    zm = (RIM_Z0 + RIM_Z1) / 2

    def ring(z0, z1, b0, b1):
        return revolve(Plane.XZ * Polygon((r0, z0), (r1, z0), (r1, z1), (r0, z1), align=None), Axis.Z, b1 - b0).rotate(Axis.Z, b0)
    p = ring(RIM_Z0, RIM_Z1, a0 + 4.0, a1 - 4.0) + ring(zm, RIM_Z1, a1 - 4.0, a1 + 4.0) + ring(RIM_Z0, zm, a0 - 4.0, a0 + 4.0)
    for a in _lin(a0 + 6, a1 - 6, 5):                                           # lacing
        x, y = D.polar(r0 + 3.5, a)
        p -= Cylinder(1.25, 40).moved(Pos(x, y, D.RIM_Z))
    s0, s1 = strut_ends(a_c)
    n = math.dist(s0, s1)
    p += (_seg_cyl(s0, s1, 7.0, n - SOCKET[1], n + 2.0) - _seg_cyl(s0, s1, RIM_TUBE / 2 + 0.15, n - 40, n - SOCKET[0]))
    p -= _seg_cyl(s0, s1, 30.0, n + 2.0, n + 40) & ring(RIM_Z1, RIM_Z1 + 30, a0 - 10, a1 + 10)
    for a in (a0, a1):
        x, y = D.polar((r0 + r1) / 2, a)
        p -= Cylinder(D.TAP_M3 / 2, 40).moved(Pos(x, y, D.RIM_Z))
    x, y = D.polar((r0 + r1) / 2, a1)                                           # the upper lap is the clearance side
    p -= Cylinder(D.CLR_M3 / 2, RIM_Z1 - zm + 0.1, align=CT).moved(Pos(x, y, RIM_Z1 + 0.05))
    p.label = "rim_arc"
    return p


def net() -> Part:
    """The funnel net (fabric) as a thin cone, for pictures and the BOM; excluded from the clash check."""
    r0, r1 = D.FUNNEL_THROAT_R, D.FUNNEL_RIM_R
    pts = [(r0, Z_TOP), (r1, D.RIM_Z), (r1, D.RIM_Z + 0.8), (r0, Z_TOP + 0.8)]
    p = revolve(Plane.XZ * Polygon(*pts, align=None), Axis.Z)
    p.label = "funnel_net"
    return p


# --- foam pads ---------------------------------------------------------------------------------------------------------

def pads() -> list[tuple[str, Part]]:
    """6 mm low-resilience PU foam pads (MODULE frame): back, front, the chute floor, two sides."""
    z_back = floor_z(-W_PAD, RB)
    z_back = floor_z(-W_IN, RB + D.PAD_T)
    out = [("pad_back", Box(D.PAD_T, 2 * W_PAD, TUBE_TOP - z_back, align=(Align.MAX, Align.CENTER, Align.MAX)).moved(
               Pos(-W_PAD, 0, TUBE_TOP))),
           ("pad_front", Box(D.PAD_T, 2 * W_PAD, TUBE_TOP - D.FRONT_WALL_Z, align=(Align.MIN, Align.CENTER, Align.MAX)).moved(
               Pos(W_PAD, 0, TUBE_TOP)))]
    win = _feed_window()
    win += win.mirror(Plane.XZ)
    floor = xz_prism(band(RB, RB + D.PAD_T, S_BACK, 0, bend=False, run=False), -W_PAD, W_PAD)
    floor &= Box(2 * W_PAD - 0.4, 200, 400)
    out.append(("pad_chute_floor", floor - win))
    slope = C[1] / C[0]
    upper = [(-W_PAD, P0[1] + (-W_PAD - P0[0]) * slope), (J0[0], J0[1]), (W_PAD, TUBE_TOP), (-W_PAD, TUBE_TOP)]
    for s in (1, -1):
        pad = (xz_prism(band(-(RB + 8.0), RB + D.PAD_T, S_BACK, 0, bend=False, run=False), W_PAD, W_IN) +
               xz_prism(band(-(RB + 8.0), RB, 0, LAUNCH_END, chute=False), W_PAD, W_IN) + xz_prism(upper, W_PAD, W_IN))
        pad &= xz_prism([(-W_PAD, 0), (SIDE_END - 2, 0), (SIDE_END - 2, TUBE_TOP), (-W_PAD, TUBE_TOP)], W_PAD - 1, W_IN + 1)
        pad -= win
        pad -= Box(100, 200, TUBE_TOP - D.FRONT_WALL_Z + 2, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
            Pos(W_PAD, 0, D.FRONT_WALL_Z - 1))                       # the front pad covers the corner
        for x, z in BEAMS:
            pad -= _y_cyl(2.7, x, z, W_PAD - 1, W_IN + 1)
        out.append((f"pad_side_{'L' if s > 0 else 'R'}", pad if s > 0 else pad.mirror(Plane.XZ)))
    return out


# --- module hardware ---------------------------------------------------------------------------------------------------

@lru_cache(maxsize=None)
def _torsion_spring() -> Part:
    return Torus(3.0, 0.6) + Torus(3.0, 0.6).moved(Pos(0, 0, 1.3)) + Torus(3.0, 0.6).moved(Pos(0, 0, 2.6))


@lru_cache(maxsize=None)
def _prop_nut() -> Part:
    return Cylinder(4.0, 5.0, align=CB) - Cylinder(2.5, 6, align=CB)


def hardware() -> hw.Hardware:
    h = hw.Hardware()
    px, pz = D.PIVOT_M
    y0, y1 = PIVOT_BOSS_Y
    for s in (1, -1):
        f = feed_frame(s)
        flip = Rot(0, 0, 0) if s > 0 else Rot(0, 0, 180)
        h.part("Pololu 25D 9.7:1 HP 12 V + encoder", "Pololu #4842 9.7:1 25Dx63L HP 12 V gearmotor, 48 CPR encoder",
               "pololu.com #4842 ($56.95)", V.pololu_25d(), f * Pos(0, 0, -FEED_FACE) * flip)
        h.part("Pololu #1997 hub", "Pololu universal aluminium mounting hub, 4 mm shaft, 4 x M3 (pair: #1997)",
               "pololu.com #1997 ($7.95 per 2)", V.hub_1997(), f * Pos(0, 0, -D.FEED_WHEEL[1] / 2))
        for k in range(4):                         # feed wheel -> #1997 hub
            x, y = D.polar(D.HUB_1997["pcd"] / 2, 90 * k + 45)
            h.screw("PAN", 14, f * Pos(x, y, D.FEED_WHEEL[1] / 2 - D.CB_M3[1]))
        for dy in (-8.5, 8.5):                     # 25D face -> carriage
            h.screw("PAN", 8, f * Pos(0, dy, -FEED_FACE + 6.0 - D.CB_M3[1]))
        hf = _hinge_frame(s)
        h.screw("PAN", 30, hf * Pos(0, 0, KNUCKLE[1] + 5.5))
        h.nut(hf * Pos(0, 0, KNUCKLE[0] - SPRING_GAP - 5.0) * Rot(180, 0, 0), kind="nyloc")
        h.part("feed torsion spring", "torsion spring, 0.5 mm wire, 3.2 mm ID, 90 deg legs (~6 N at the wheel)",
               "McMaster 9271K (torsion spring, est)", _torsion_spring(), hf * Pos(0, 0, KNUCKLE[0] - SPRING_GAP + 0.6))
        h.screw("SSCS", 10, f * Pos(-18.0, s * (HINGE_DY + 4.0), -FEED_FACE + 3.0) * hw.rot_to("y" if s > 0 else "-y"))
        g = fw_frame(s)
        h.part("T-Motor F90 2806.5 1300KV", "T-Motor F90 2806.5 1300KV (or iFlight XING 2806.5)",
               "T-Motor via GetFPV / Pyrodrone (~$30)", V.f90(), g * Pos(0, 0, MOTOR_FACE_Z))
        for dx in (-9.5, 9.5):                     # 6 mm plate + 4 mm into the F90's holes
            for dy in (-9.5, 9.5):
                h.screw("PAN", 10, g * Pos(dx, dy, MOTOR_FACE_Z - FW_PLATE_T) * Rot(180, 0, 0))
        h.part("M5 prop nut", "M5 nylon-insert prop nut (comes with the F90)", "with the motor",
               _prop_nut(), g * Pos(0, 0, -D.FW_W / 2 + 12.0))
        for x, z in FW_SCREWS:                     # mount -> keel, heads on the pocket floors
            h.screw("PAN", 16, Pos(x, s * FW_SCREW_SEAT, z) * hw.rot_to("y" if s > 0 else "-y"))
        for x, z in BEAMS:   # emitter / receiver press-fit in the sensor pad (a drop of hot glue), lens in the Ø5.4 hole
            h.part("IR break-beam half", "5 mm IR break-beam sensor half (emitter or receiver; Adafruit #2168 pair)",
                   "adafruit.com #2168 ($5.95 per pair)", V.ir_pair(), Pos(x, s * (W_OUT + 13.0), z) * hw.rot_to("-y" if s > 0 else "y"))
    for x, z in JOINT_TABS:                        # across the split, from -y into +y
        h.screw("PAN", 14, Pos(x, -8.0, z) * hw.rot_to("-y"))
    for x, y in COLLAR_SCREWS:                     # collar -> chute flanges
        h.screw("PAN", 10, Pos(x, y, Z_TOP - D.CB_M3[1]))
    for a in STRUT_ANGLES:
        p0, p1 = strut_ends(a)
        s0, s1 = strut_span(a)
        h.part("carbon tube 8 x 6", f"carbon-fibre tube 8 mm OD x 6 mm ID (6 struts, cut to {s1 - s0:.0f} mm)",
               "McMaster 2153T (carbon-fibre tube 8 mm) or RC hobby supplier", _seg_cyl(p0, p1, RIM_TUBE / 2, s0, s1), Location())
    for a in RIM_JOINTS:                           # rim laps
        x, y = D.polar(D.FUNNEL_RIM_R + 6.0, a)
        h.screw("PAN", 12, Pos(x, y, RIM_Z1))
    h.part("shaft 8 x 40", "8 mm steel shaft, h6, 40 mm (tilt pivot, +y side)", "McMaster 6112K (8 mm shaft, cut)",
           _y_cyl(4.0, 0, 0, 0, 40.0), Pos(px, y0 + 2.0, pz))
    h.screw("PAN", 20, Pos(px - 8.0, (y0 + y1) / 2 + 1, pz + 6.0) * hw.rot_to("-x"))
    h.nut(Pos(px + 8.5, (y0 + y1) / 2 + 1, pz + 6.0) * hw.rot_to("x"))
    for k in range(4):                              # servo horn -> -y boss
        x, z = D.polar(D.STS3250["horn_pcd"] / 2, 90 * k + 45)
        h.screw("PAN", 10, Pos(px + x, -y1 - 3.5, pz + z) * hw.rot_to("-y"))
    h.part("servo horn", "Feetech 25T aluminium disc horn (with the STS3250)", "with the servo", V.servo_horn(),
           Pos(px, -y1, pz) * hw.rot_to("-y"))
    return h


def printed() -> list[tuple[str, Part, str, int, str]]:
    """(name, part placed in the MODULE frame, material, infill %, orientation)."""
    out = [("chute_half_L", chute_half(1), "PETG", 30, "outer side wall down"),
           ("chute_half_R", chute_half(-1), "PETG", 30, "outer side wall down (mirror of L)"),
           ("collar", collar(), "PETG", 40, "top face down")]
    out += [(f"rim_arc_{k}", rim_arc(k), "PETG", 30, "flat") for k in range(6)]
    for s, nm in ((1, "L"), (-1, "R")):
        car = feed_carriage().moved(feed_frame(1))
        out.append((f"feed_carriage_{nm}", car if s > 0 else _mirror(car, "feed_carriage"), "PETG", 60, "plate down"))
        out.append((f"feed_wheel_hub_{nm}", feed_wheel_hub().moved(feed_frame(s)), "PETG", 60, "flat"))
        out.append((f"feed_wheel_tyre_{nm}", feed_wheel_tyre().moved(feed_frame(s)), "TPU 95A", 100, "flat"))
        out.append((f"flywheel_hub_{nm}", flywheel_hub().moved(fw_frame(s)), "PETG", 100, "flat"))
        out.append((f"flywheel_tyre_{nm}", flywheel_tyre().moved(fw_frame(s)), "TPU 95A", 100, "flat"))
        fm = fw_mount()
        out.append((f"fw_mount_{nm}", fm if s > 0 else _mirror(fm, "fw_mount"), "PETG", 60, "flange down"))
    return out
