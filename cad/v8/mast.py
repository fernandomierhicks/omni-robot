"""The face mast (round 2, Oct 27): one place for the face, the speaker and the mics on both robots. ROBOT frame.

Fernando (Oct 27): "The screen and speakers need to be in the same place in both robot configs. I am not going to be
moving those for each variant." Where both payloads leave room:
  - The front is out: the SO-101 comes down over the camera bar to the floor exactly where the face was (design.md
    12), and the catch module's launch line passes over it.
  - The plate's rear is out: the catch module's funnel tilts back over it (its rim reaches x -193 at -25 deg).
  - Behind the funnel, above its rim, the face is seen from the front over the rim (its strut sockets' tops are
    420 mm up at tilt 0) and stays clear of the arm (its bin and drop are at x -130 ... -10).
So two posts rise from the deck's rear edge, outside the funnel's sweep, and carry a crown: the face unit (display.py,
the same housing, window, panel and back cover) between the crown's two ears, its front-bottom edge at (-187, 430),
leaning back 6 deg; the audio cap (mic array and speaker, display.py) on the ears' tops (the robot's top: 0.54 m).
The funnel rim's strut sockets stand 16 mm over the rim: they pass 7.5 mm under the face (at -2 deg of tilt) and
3.4 mm in front of the crown (at -14 deg; build.py "head").

  mast post    6061 square tube 20 x 20 x 1.5 (the camera bar's stock), 357 mm, standing on the deck at
               (-202.4, +-48); the cables (HDMI, USB, the speaker's) run down inside the left one
  mast_foot    printed socket on the deck (2 x M3 up from under the deck, an M3 x 30 and a nyloc across the tube); it
               reaches forward (the funnel is 200 mm higher) to stay inside the deck's edge
  crown        printed: a socket over each post's top (M3 x 30 and nyloc across), a bridge between them under the face,
               two ears (|y| 63.5-73) that take the face's four side screws and the audio cap's four
The posts cross the lidar's scan plane 340 mm behind it: two thin shadows next to the rear standoffs'.
"""

from __future__ import annotations

from functools import lru_cache

from build123d import Align, Box, Cylinder, Location, Part, Plane, Pos, Rot

import dims as D
import hw

MIN = (Align.MIN, Align.MIN, Align.MIN)
CB = (Align.CENTER, Align.CENTER, Align.MIN)
TUBE = dict(a=20.0, t=1.5)
POST_X, POST_Y = -202.4, 48.0           # the posts' centres: front faces 3 mm behind the funnel rim's sweep at |y| 38
POST_TOP = 423.5                        # into the crown's sockets
POST_L = POST_TOP - D.DECK_TOP          # 357
SOCK = TUBE["a"] / 2 + 0.2              # socket half width
WALL = 3.0
FOOT = dict(x=(-180.0, POST_X - SOCK - 1.6), z=30.0)      # the foot reaches forward (+x) for its two screws
FOOT_SCREWS = [(-186.0, POST_Y - 8.0), (-186.0, POST_Y + 8.0)]
FOOT_BOLT_Z = D.DECK_TOP + 15.0
CROWN = dict(z=(396.0, 426.0), x=(POST_X + SOCK + WALL, POST_X - SOCK - WALL), top=POST_TOP + 2.5)
CROWN_BOLT_Z = 410.0
EAR_Y = (63.5, 73.0)                    # beside the face (its housing is +-63)
EAR_X = (-193.0, -217.0)
EAR_TOP = 522.0                         # the audio cap's floor
CAP_SCREWS = [(x, s * sum(EAR_Y) / 2) for x in (-196.0, -204.0) for s in (1, -1)]   # clear of the face's top screw


def foot_screws():
    """(x, y) of the feet's screws (up from under the deck), both sides."""
    return [(x, s * y) for x, y in FOOT_SCREWS for s in (1, -1)]


def _box(x, y, z) -> Part:
    x0, x1 = min(x), max(x)
    y0, y1 = min(y), max(y)
    return Box(x1 - x0, y1 - y0, z[1] - z[0], align=MIN).moved(Pos(x0, y0, z[0]))


def _ycyl(r, y0, y1, x, z) -> Part:
    return Cylinder(r, abs(y1 - y0), rotation=(-90, 0, 0), align=CB).moved(Pos(x, min(y0, y1), z))


def _socket(z0: float, z1: float) -> Part:
    return Box(2 * SOCK, 2 * SOCK, z1 - z0, align=CB).moved(Pos(POST_X, POST_Y, z0))


def foot_envelopes(grow: float = 0.0) -> list[Part]:
    """The two feet's outlines (for the rear bumper's lip to step round), ROBOT frame."""
    y0, y1 = POST_Y - SOCK - WALL - grow, POST_Y + SOCK + WALL + grow
    x0, x1 = FOOT["x"][0] + grow, FOOT["x"][1] - grow
    return [_box((x0, x1), (s * y0, s * y1), (D.DECK_TOP - 1.0, D.DECK_TOP + 5.0)) for s in (1, -1)]


@lru_cache(maxsize=None)
def foot() -> Part:
    """The left foot (+y); the right one is its mirror image."""
    y0, y1 = POST_Y - SOCK - WALL, POST_Y + SOCK + WALL
    z0 = D.DECK_TOP
    p = _box(FOOT["x"], (y0, y1), (z0, z0 + FOOT["z"]))
    p -= _socket(z0 - 0.01, z0 + FOOT["z"] + 0.01)
    p -= hw.cove_at(4.0, (FOOT["x"][0], y0, z0 + FOOT["z"]), (-1, 0, 0), (0, 0, -1), (0, 1, 0), y1 - y0)   # round
    p -= _box((POST_X - 5.0, POST_X + 5.0), (y0 - 1.0, y0 + WALL + 0.5), (z0 - 0.01, z0 + 8.0))           # cables out
    p -= _ycyl(D.CLR_M3 / 2, y0 - 1, y1 + 1, POST_X, FOOT_BOLT_Z)                                       # cross bolt
    for x, y in FOOT_SCREWS:
        p -= hw.tap(10.0, top=z0 + 10.0, entry="bottom").moved(Pos(x, y, 0))
    p.label = "mast_foot"
    return p


@lru_cache(maxsize=None)
def post() -> Part:
    """6061 square tube 20 x 20 x 1.5, POST_L long, two 3.4 mm cross holes (the bolts at the foot and the crown).
    Frame: its bottom at z = 0, centred."""
    a, t = TUBE["a"], TUBE["t"]
    p = Box(a, a, POST_L, align=CB) - Box(a - 2 * t, a - 2 * t, POST_L + 2, align=CB).moved(Pos(0, 0, -1))
    for z in (FOOT_BOLT_Z - D.DECK_TOP, CROWN_BOLT_Z - D.DECK_TOP):
        p -= _ycyl(D.CLR_M3 / 2, -a, a, 0.0, z)
    p.label = "mast post"
    return p


@lru_cache(maxsize=None)
def crown() -> Part:
    """Sockets over both posts' tops, the bridge between them (under the face), the two ears up to the audio cap."""
    (cz0, cz1), (cx0, cx1) = CROWN["z"], CROWN["x"]
    yo = EAR_Y[1]
    p = _box((cx0, cx1), (-yo, yo), (cz0, cz1))
    p -= _box((cx0 + 3.0, cx1 - 3.0), (-(POST_Y - SOCK - WALL), POST_Y - SOCK - WALL), (cz0 - 0.01, cz1 - 6.0))   # hollow bridge
    for s in (1, -1):
        p -= _socket(cz0 - 0.01, POST_TOP).moved(Pos(0, (s - 1) * POST_Y, 0))
        p -= _ycyl(D.CLR_M3 / 2, s * (yo + 1), s * (POST_Y - SOCK - WALL - 1), POST_X, CROWN_BOLT_Z)
        p += _box(EAR_X, (s * EAR_Y[0], s * EAR_Y[1]), (cz1 - 0.01, EAR_TOP))                       # the ears
    p -= _box((POST_X - 4.0, POST_X + 4.0), (POST_Y - 6.0, POST_Y + 6.0), (POST_TOP - 0.01, cz1 + 0.01))   # cables down L
    p -= _box((cx0 + 0.01, -193.0), (-(POST_Y - SOCK - WALL), POST_Y - SOCK - WALL), (cz0 - 0.01, cz1 + 0.01))   # the
    # bridge's front set back 4 mm: the funnel rim passes 3 mm under its corner at -8 deg
    import display as DP
    for x, z in DP.seat_screws():                                          # the face's side screws, clear through the ears
        p -= _ycyl(D.CLR_M3 / 2, -(yo + 1), yo + 1, x, z)
    for x, y in CAP_SCREWS:
        p -= hw.tap(12.0, top=EAR_TOP, entry="top").moved(Pos(x, y, 0))
    p.label = "crown"
    return p


def hardware() -> hw.Hardware:
    import vendor as V
    h = hw.Hardware()
    for s in (1, -1):
        h.part("mast post 20 x 20 tube", f"6061 square tube 20 x 20 x 1.5, {POST_L:.0f} mm, 2 x 3.4 mm cross holes (the face "
               "mast; the camera bar's stock)", "McMaster 6546K (6061 square tube 3/4 in) or 20 x 20 x 1.5 metric stock",
               post(), Pos(POST_X, s * POST_Y, D.DECK_TOP))
        yo = s * (POST_Y + SOCK + WALL)
        for z, yb in ((FOOT_BOLT_Z, yo), (CROWN_BOLT_Z, s * EAR_Y[1])):   # across socket and tube, nyloc inside
            h.screw("PAN", 30 if z == FOOT_BOLT_Z else 45, Pos(POST_X, yb, z) * hw.rot_to("y" if s > 0 else "-y"))
            h.nut(Pos(POST_X, s * (POST_Y - SOCK - WALL), z) * hw.rot_to("-y" if s > 0 else "y"), kind="nyloc")
        for x, y in FOOT_SCREWS:                                  # up through the deck into the foot: 8 mm of thread
            h.screw("PAN", 14, Pos(x, s * y, D.DECK_Z) * hw.rot_to("-z"))
    for x, y in CAP_SCREWS:                                       # the audio cap -> the ears, from inside the cap
        h.screw("PAN", 14, Pos(x, y, EAR_TOP + 3.0))
    h.part("ELRS receiver", "ELRS 2.4 GHz receiver (RadioMaster RP2 class), 3M dual-lock on top of the left mast foot, "
           "its antenna up the post (a base part)", "RadioMaster ($15)", V.board(22.0, 14.0, 5.0, "ELRS receiver"),
           Pos(-185.0, POST_Y, D.DECK_TOP + FOOT["z"] + 0.5) * Rot(0, 0, 90), text="ELRS RX")
    return h


def printed() -> list[tuple[str, Part, Location, str, int, str]]:
    """(name, the print, where it goes, material, infill, orientation)."""
    f = foot()
    fm = f.mirror(Plane.XZ)
    fm.label = "mast_foot"
    return [("mast_foot_L", f, Location(), "PETG", 40, "socket up"),
            ("mast_foot_R", fm, Location(), "PETG", 40, "socket up (mirror of L)"),
            ("crown", crown(), Location(), "PETG", 40, "on its back (x -217 face) down")]
