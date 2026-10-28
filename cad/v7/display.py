"""The robot's face and voice: a 5 in display at the front, between the cameras (design.md 12, Oct 24), and the USB
microphone array and speaker in a box at the payload plate's rear. ROBOT frame.

Both move as units when the payload changes (design.md 12.3): on the arm robot the face stands over the bin on the arm
kit's uprights, facing forward 0.40 m up, and the audio box sits on the shelf above it (arm_kit.py). The prints are
the same; SEATS and AUDIO_SEATS say where each goes, seat_move() / audio_move() take them there.

Where: the face hangs from the head's cheeks (head.py) between the two cameras, leaning back 10 deg, its front face
6 mm behind the lenses at their height and its bottom 3 mm over the A2M12's top, ahead of it (x > 175), so the scan
passes under it. The catch game never launches flatter than 30 deg, so below the launch line there is room: the
launch line clears the face from about 22 deg up (bench mode's lowest elevation rises to ~25). Every viewer in front
sees all of it; the thrower sees it 2 deg off its axis. The 7 in panel doesn't fit there (it hits the 30 deg launch
line and needs a 230 mm baseline); the rear-facing 7 in it replaces was seen by nobody in front.

Parts:
  face_housing  printed tray, open at the back, in the face's own frame: a 2 mm front lip round the window, 2 mm
                walls, a 5 mm rail along the top and the bottom (above / below the panel) that takes the screws: two
                M3 x 16 from each cheek along y, two M3 x 12 from the back cover. Prints face down.
  face_back     2 mm back cover: clamps the panel against the window (2 mm foam tape on its back), cable slot
  audio_base    open-top box on the plate's rear (4 x M3 into its tapped holes) with the USB mic array on bosses
  audio_lid     3 mm lid (4 x M3): two mic holes over the array's mics, a grille over the speaker
Purchased (VERIFY every size before printing): a 5 in 800 x 480 HDMI IPS panel with its driver on its back (Waveshare
5inch HDMI LCD class, 121 x 77 x 12 est), a 2 mm polycarbonate window cut to 121 x 77 (a low ball hits the face), a
USB 2-mic array with echo cancellation and a speaker output (Seeed ReSpeaker Lite class), a 4 ohm 3 W enclosed micro
speaker on VHB tape. The face's software draws for 800 x 480 (omni_face: redraw from 1024 x 600).
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import Align, Box, Cylinder, Location, Part, Pos, Rot

import base as B
import dims as D
import hw

CB = (Align.CENTER, Align.CENTER, Align.MIN)
MIN = (Align.MIN, Align.MIN, Align.MIN)

# --- the face unit (its local frame: front face at x = 0 facing +x, bottom at z = 0, y as the robot) --------------------
SEATS = {"catch": (198.0, 160.0),         # the housing's front-bottom edge (x, z), ROBOT frame: between the cameras
         "arm": (-124.10, 355.19)}        # over the bin: the front face's middle at (-132, 400), as layout_trade's A5
FRONT = SEATS["catch"]
LEAN = 10.0                               # deg back
PANEL = dict(w=121.0, h=77.0, t=12.0, active=(108.0, 64.8))      # 5 in 800 x 480 HDMI with its driver (VERIFY)
W_HALF, HEIGHT = 63.0, 91.0               # housing outer: 126 wide (the cheeks at |y| 64), 91 tall
LIP, WALL, RAIL = 2.0, 2.0, 5.0
WINDOW_T = 2.0
DEPTH = LIP + WINDOW_T + PANEL["t"]       # housing 16 deep; the back cover behind it
BACK_T = 2.0
Z_PANEL = WALL + RAIL                     # the panel (and the window) between the rails
SCREW_X = -13.0                           # the cheek screws, 13 mm behind the front face, in the rails
SIDE_SCREWS_Z = (WALL + 2.2, HEIGHT - WALL - 2.2)    # 1.4 mm of rail over each hole, 2.8 under
BACK_SCREWS = [(s * 35.0, z) for s in (-1, 1) for z in SIDE_SCREWS_Z]


def face_loc(seat: str = "catch") -> Location:
    """The face unit's frame -> ROBOT: front-bottom edge at its seat, leaning back LEAN."""
    x, z = SEATS[seat]
    return Pos(x, 0, z) * Rot(0, -LEAN, 0)


def seat_move(seat: str) -> Location:
    """Moves the face's parts (modelled at the catch seat) to a seat."""
    return face_loc(seat) * face_loc("catch").inverse()


def to_robot(lx: float, lz: float, seat: str = "catch"):
    """A point (x, z) of the face's frame in the ROBOT frame."""
    x0, z0 = SEATS[seat]
    c, s = math.cos(math.radians(LEAN)), math.sin(math.radians(LEAN))
    return x0 + lx * c - lz * s, z0 + lx * s + lz * c


def seat_screws(seat: str):
    """(x, z) of the face's two side screws (bottom, top) at a seat: they go along y into its rails."""
    return [to_robot(SCREW_X, z, seat) for z in SIDE_SCREWS_Z]


def front_x(z: float) -> float:
    """x of the face's front plane at height z (ROBOT frame)."""
    return FRONT[0] - (z - FRONT[1]) * math.tan(math.radians(LEAN))


BOTTOM_SCREW, TOP_SCREW = (to_robot(SCREW_X, z) for z in SIDE_SCREWS_Z)     # (x, z) of the cheek screws


def _box(x, y, z) -> Part:
    return Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=MIN).moved(Pos(x[0], y[0], z[0]))


@lru_cache(maxsize=None)
def housing() -> Part:
    yi = W_HALF - WALL
    p = _box((-DEPTH, 0.0), (-W_HALF, W_HALF), (0.0, HEIGHT))
    p -= _box((-DEPTH - 1, -LIP), (-yi, yi), (WALL, HEIGHT - WALL))
    for z0 in (WALL, HEIGHT - WALL - RAIL):                                    # the rails
        p += _box((-DEPTH, -LIP + 0.01), (-yi - 0.01, yi + 0.01), (z0, z0 + RAIL))
    aw, ah = PANEL["active"]
    zc = Z_PANEL + PANEL["h"] / 2
    p -= _box((-LIP - 1, 1.0), (-aw / 2 - 1, aw / 2 + 1), (zc - ah / 2 - 1, zc + ah / 2 + 1))   # the window opening
    for z in SIDE_SCREWS_Z:
        for s in (1, -1):                                                      # cheek screws, along y
            p -= hw.tap(9.0, entry="top").moved(Pos(SCREW_X, s * W_HALF, z) * hw.rot_to("y" if s > 0 else "-y"))
    for y, z in BACK_SCREWS:                                                   # back cover screws, along x
        p -= hw.tap(10.0, entry="top").moved(Pos(-DEPTH, y, z) * hw.rot_to("-x"))
    p = p.moved(face_loc())
    p.label = "face_housing"
    return p


@lru_cache(maxsize=None)
def back_cover() -> Part:
    p = _box((-DEPTH - BACK_T, -DEPTH), (-W_HALF, W_HALF), (0.0, HEIGHT))
    for y, z in BACK_SCREWS:
        p -= hw.clear(BACK_T).moved(Pos(-DEPTH - BACK_T, y, z) * hw.rot_to("-x"))
    p -= _box((-DEPTH - BACK_T - 1, -DEPTH + 1), (-25.0, 25.0), (Z_PANEL + 4.0, Z_PANEL + 18.0))   # HDMI + USB out
    p = p.moved(face_loc())
    p.label = "face_back"
    return p


def panel(seat: str = "catch") -> Part:
    zp = Z_PANEL
    p = _box((-LIP - WINDOW_T - PANEL["t"], -LIP - WINDOW_T), (-PANEL["w"] / 2, PANEL["w"] / 2), (zp, zp + PANEL["h"]))
    p.label = "5 in display"
    return p.moved(face_loc(seat))


def window(seat: str = "catch") -> Part:
    zp = Z_PANEL
    p = _box((-LIP - WINDOW_T, -LIP), (-PANEL["w"] / 2, PANEL["w"] / 2), (zp, zp + PANEL["h"]))
    p.label = "polycarbonate window"
    return p.moved(face_loc(seat))


# --- the audio box at the plate's rear (its top stays 14 mm under the module at +40 deg: z 223 over x -118..-70) ---------
MICS = dict(w=86.0, d=35.0, t=8.0, holes=[(dx, dy) for dx in (-13.5, 13.5) for dy in (-40.0, 40.0)],
            mics=[(0.0, -38.0), (0.0, 38.0)])                     # USB 2-mic array board (ReSpeaker Lite class, VERIFY)
SPEAKER = (40.0, 28.0, 12.0)                                      # enclosed micro speaker, y x x x z (VERIFY)
BASE_X, BASE_Y, BASE_TOP = (-121.0, -70.0), 60.0, 206.0
FLOOR = 4.0
LID_T = 3.0
LID_SCREWS = [(x, s * (BASE_Y - 5.0)) for x in (BASE_X[0] + 5.0, BASE_X[1] - 5.0) for s in (-1, 1)]
MIC_XY = (-95.0, 0.0)
# where the audio box stands: on the plate's rear (catch robot), on the arm kit's shelf over the face (its floor's
# underside, the box's middle in x): the same 4 screw holes, tapped in the plate or in the shelf
AUDIO_SEATS = {"plate": (sum(BASE_X) / 2, D.PLATE_TOP), "shelf": (-148.0, 456.0)}


def audio_move(seat: str) -> Location:
    x, z = AUDIO_SEATS[seat]
    x0, z0 = AUDIO_SEATS["plate"]
    return Pos(x - x0, 0, z - z0)


def audio_screws(seat: str):
    """(x, y) of the audio box's 4 floor screws at a seat."""
    dx = AUDIO_SEATS[seat][0] - AUDIO_SEATS["plate"][0]
    return [(x + dx, y) for x, y in B.AUDIO_SCREWS]


@lru_cache(maxsize=None)
def audio_base() -> Part:
    """Open-top box on the payload plate, screwed down through its floor from inside."""
    x0, x1 = BASE_X
    z0 = D.PLATE_TOP
    p = _box((x0, x1), (-BASE_Y, BASE_Y), (z0, BASE_TOP))
    p -= _box((x0 + WALL + 1, x1 - WALL - 1), (-BASE_Y + WALL + 1, BASE_Y - WALL - 1), (z0 + FLOOR, BASE_TOP + 1))
    for x, y in LID_SCREWS:                                                     # lid bosses in the corners
        p += Cylinder(4.5, BASE_TOP - z0, align=CB).moved(Pos(x, y, z0))
        p -= hw.tap(10.0, top=BASE_TOP, entry="top").moved(Pos(x, y, 0))
    for dx, dy in MICS["holes"]:                                                # mic board: 6 mm bosses
        x, y = MIC_XY[0] + dx, MIC_XY[1] + dy
        p += Cylinder(3.5, FLOOR + 6.0, align=CB).moved(Pos(x, y, z0))
        p -= hw.tap(10.0, top=z0 + FLOOR + 6.0, entry="top").moved(Pos(x, y, 0))
    for x, y in B.AUDIO_SCREWS:                                                 # down into the plate
        p -= hw.clear(FLOOR, top=z0 + FLOOR).moved(Pos(x, y, 0))
    p -= _box((x1 - WALL - 2, x1 + 1), (-10.0, 10.0), (z0 + FLOOR, z0 + FLOOR + 10.0))          # USB to the Jetson
    p.label = "audio_base"
    return p


@lru_cache(maxsize=None)
def audio_lid() -> Part:
    x0, x1 = BASE_X
    p = _box((x0, x1), (-BASE_Y, BASE_Y), (BASE_TOP, BASE_TOP + LID_T))
    for x, y in LID_SCREWS:
        p -= hw.clear(LID_T, top=BASE_TOP + LID_T).moved(Pos(x, y, 0))
    for dx, dy in MICS["mics"]:
        p -= Cylinder(1.5, 10).moved(Pos(MIC_XY[0] + dx, MIC_XY[1] + dy, BASE_TOP))
    for i in range(-3, 4):                                                      # speaker grille
        for j in range(-2, 3):
            p -= Cylinder(1.25, 10).moved(Pos(MIC_XY[0] + 4.5 * j, MIC_XY[1] + 4.5 * i, BASE_TOP))
    p.label = "audio_lid"
    return p


def mic_board() -> Part:
    z = D.PLATE_TOP + FLOOR + 6.0
    p = _box((MIC_XY[0] - MICS["d"] / 2, MIC_XY[0] + MICS["d"] / 2), (-MICS["w"] / 2, MICS["w"] / 2), (z, z + 1.6))
    p += _box((MIC_XY[0] - 10, MIC_XY[0] + 10), (-25.0, 25.0), (z + 1.6, z + MICS["t"]))
    for dx, dy in MICS["holes"]:
        p -= Cylinder(1.6, 10).moved(Pos(MIC_XY[0] + dx, MIC_XY[1] + dy, z))
    p.label = "USB mic array"
    return p


def speaker() -> Part:
    w, d, h = SPEAKER
    p = _box((MIC_XY[0] - d / 2, MIC_XY[0] + d / 2), (-w / 2, w / 2), (BASE_TOP - h, BASE_TOP))
    p.label = "micro speaker"
    return p


def hardware(seat: str = "catch", audio: str = "plate") -> hw.Hardware:
    """The face unit's hardware at its seat ("catch": the head's cheeks, "arm": the arm kit's uprights) and the audio
    box's at its seat ("plate", "shelf")."""
    h = hw.Hardware()
    if seat:
        h.part("5 in display 800x480", "5 in 800 x 480 IPS HDMI display + USB touch, 121 x 77 x 12 with its driver on its back "
               "(Waveshare 5inch HDMI LCD class; VERIFY size, connectors and depth); HDMI + USB to the Jetson",
               "Waveshare / generic (~$40)", panel(seat), Location())
        h.part("polycarbonate window 2 mm", "2 mm clear polycarbonate, cut to 121 x 77 (in front of the panel)",
               "McMaster 8574K (polycarbonate sheet, 1/16 in) (est)", window(seat), Location())
        fl = face_loc(seat)
        for z in SIDE_SCREWS_Z:                     # cheeks / uprights -> housing rails: 8 mm, then 7 mm of thread
            for s in (1, -1):
                h.screw("PAN", 16, fl * Pos(SCREW_X, s * 72.0, z) * hw.rot_to("y" if s > 0 else "-y"))
        for y, z in BACK_SCREWS:                    # back cover -> rails
            h.screw("PAN", 12, fl * Pos(-DEPTH - BACK_T, y, z) * hw.rot_to("-x"))
    am = audio_move(audio)
    h.part("USB mic array", "USB 2-mic array, echo cancellation and beamforming, speaker output (Seeed ReSpeaker Lite class, "
           "VERIFY board size, holes and amp power)", "Seeed Studio (~$25 est)", mic_board(), am)
    h.part("micro speaker 4 ohm 3 W", "4 ohm 3 W enclosed micro speaker 40 x 28 x 12, on VHB tape under the lid (VERIFY)",
           "generic (~$5)", speaker(), am)
    for x, y in B.AUDIO_SCREWS:                     # audio base -> plate / shelf, from inside the box: all 6 mm of it
        h.screw("PAN", 10, am * Pos(x, y, D.PLATE_TOP + FLOOR))
    for dx, dy in MICS["holes"]:                    # mic board -> bosses
        h.screw("PAN", 8, am * Pos(MIC_XY[0] + dx, MIC_XY[1] + dy, D.PLATE_TOP + FLOOR + 6.0 + 1.6))
    for x, y in LID_SCREWS:
        h.screw("PAN", 10, am * Pos(x, y, BASE_TOP + LID_T))
    return h


def printed(seat: str = "catch", audio: str = "plate") -> list[tuple[str, Part, Location, str, int, str]]:
    """(name, the print in its contract frame (the catch seat, the plate), where it goes, material, infill, orientation)."""
    am = audio_move(audio)
    out = [("audio_base", audio_base(), am, "PETG", 40, "floor down"),
           ("audio_lid", audio_lid(), am, "PETG", 30, "flat")]
    if seat:
        sm = seat_move(seat)
        out = [("face_housing", housing(), sm, "PETG", 30, "front face down"),
               ("face_back", back_cover(), sm, "PETG", 30, "flat")] + out
    return out
