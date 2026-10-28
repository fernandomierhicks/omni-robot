"""The robot's face and voice: the 5 in display, the USB microphone array and the speaker, on the face mast (mast.py).
ROBOT frame.

Round 2 (Fernando, Oct 27): one place for the screen and the speaker on both robots; he won't move them for each
payload. Until Oct 27 the face hung from the head's cheeks between the cameras on the catch robot and stood on uprights
over the bin on the arm robot, with the audio box on the plate's rear or on a shelf over the face. Now the face stands
between the mast crown's ears behind the funnel, its front-bottom edge at (-187, 430), leaning back 6 deg: over the
funnel's rim and its strut sockets (420 mm at tilt 0), so a thrower in front sees all of it at tilt 0, and well clear
of the arm (its bin and drop are at x -130 ... -10). The audio cap sits on the ears' tops over the face (the robot's
top: 0.54 m), its mics on top, the speaker beside them.

Parts:
  face_housing  printed tray, open at the back, in the face's own frame: a 2 mm front lip round the window, 2 mm
                walls, a 5 mm rail along the top and the bottom (above / below the panel) that takes the screws: two
                M3 x 18 from each ear along y, two M3 x 12 from the back cover. Prints face down.
  face_back     2 mm back cover: clamps the panel against the window (2 mm foam tape on its back), cable slot
  audio_base    a 15 mm deep open box on the crown's ears (4 x M3 into them from inside), the USB mic array on bosses
  audio_lid     3 mm lid (4 x M3): two mic holes over the array's mics, a grille over the speaker
Purchased (VERIFY every size before printing): a 5 in 800 x 480 HDMI IPS panel with its driver on its back (Waveshare
5inch HDMI LCD class, 121 x 77 x 12 est), a 2 mm polycarbonate window cut to 121 x 77, a USB 2-mic array with echo
cancellation and a speaker output (Seeed ReSpeaker Lite class), a 4 ohm 3 W enclosed micro speaker on VHB tape. The
cables (HDMI, USB) run down the left mast post. The face's software draws for 800 x 480 (omni_face).
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import Align, Box, Cylinder, Location, Part, Pos, Rot

import dims as D
import hw

CB = (Align.CENTER, Align.CENTER, Align.MIN)
MIN = (Align.MIN, Align.MIN, Align.MIN)

# --- the face unit (its local frame: front face at x = 0 facing +x, bottom at z = 0, y as the robot) --------------------
SEAT = (-187.0, 430.0)                    # the housing's front-bottom edge (x, z), ROBOT frame: on the face mast
LEAN = 6.0                                # deg back (the viewers' eyes are above it)
PANEL = dict(w=121.0, h=77.0, t=12.0, active=(108.0, 64.8))      # 5 in 800 x 480 HDMI with its driver (VERIFY)
W_HALF, HEIGHT = 63.0, 91.0               # housing outer: 126 wide (the crown's ears at |y| 63.5), 91 tall
LIP, WALL, RAIL = 2.0, 2.0, 5.0
WINDOW_T = 2.0
DEPTH = LIP + WINDOW_T + PANEL["t"]       # housing 16 deep; the back cover behind it
BACK_T = 2.0
Z_PANEL = WALL + RAIL                     # the panel (and the window) between the rails
SCREW_X = -13.0                           # the ears' screws, 13 mm behind the front face, in the rails
SIDE_SCREWS_Z = (WALL + 2.2, HEIGHT - WALL - 2.2)    # 1.4 mm of rail over each hole, 2.8 under
BACK_SCREWS = [(s * 35.0, z) for s in (-1, 1) for z in SIDE_SCREWS_Z]
EAR_Y_OUT = 73.0                          # the ears' outer faces (mast.EAR_Y)


def face_loc() -> Location:
    """The face unit's frame -> ROBOT: front-bottom edge at its seat, leaning back LEAN."""
    x, z = SEAT
    return Pos(x, 0, z) * Rot(0, -LEAN, 0)


def to_robot(lx: float, lz: float):
    """A point (x, z) of the face's frame in the ROBOT frame."""
    x0, z0 = SEAT
    c, s = math.cos(math.radians(LEAN)), math.sin(math.radians(LEAN))
    return x0 + lx * c - lz * s, z0 + lx * s + lz * c


def seat_screws():
    """(x, z) of the face's two side screws (bottom, top): they go along y through the ears into its rails."""
    return [to_robot(SCREW_X, z) for z in SIDE_SCREWS_Z]


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
        for s in (1, -1):                                                      # ear screws, along y
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


def panel() -> Part:
    zp = Z_PANEL
    p = _box((-LIP - WINDOW_T - PANEL["t"], -LIP - WINDOW_T), (-PANEL["w"] / 2, PANEL["w"] / 2), (zp, zp + PANEL["h"]))
    p.label = "5 in display"
    return p.moved(face_loc())


def window() -> Part:
    zp = Z_PANEL
    p = _box((-LIP - WINDOW_T, -LIP), (-PANEL["w"] / 2, PANEL["w"] / 2), (zp, zp + PANEL["h"]))
    p.label = "polycarbonate window"
    return p.moved(face_loc())


# --- the audio cap on the crown's ears, over the face (its front overhangs the funnel: nothing reaches 0.5 m there) ---
MICS = dict(w=86.0, d=35.0, t=8.0, holes=[(dx, dy) for dx in (-13.5, 13.5) for dy in (-40.0, 40.0)],
            mics=[(0.0, -38.0), (0.0, 38.0)])                     # USB 2-mic array board (ReSpeaker Lite class, VERIFY)
SPEAKER = (40.0, 28.0, 12.0)                                      # enclosed micro speaker, y x x x z (VERIFY)
CAP_X, CAP_Y = (-216.0, -170.0), 75.0
CAP_Z0 = 522.0                                                    # mast.EAR_TOP
CAP_TOP = CAP_Z0 + 18.0
FLOOR = 3.0
LID_T = 3.0
LID_SCREWS = [(x, s * (CAP_Y - 5.0)) for x in (-188.0, CAP_X[1] - 6.0) for s in (-1, 1)]   # clear of the ears' screws
MIC_XY = (-193.0, -22.0)                                          # the board's centre (its 86 along y)
SPEAKER_XY = (-193.0, 44.0)


def _cap_screws():
    import mast as MS
    return MS.CAP_SCREWS


@lru_cache(maxsize=None)
def audio_base() -> Part:
    """Open-top box on the ears' tops, screwed down through its floor from inside."""
    x0, x1 = CAP_X
    z0 = CAP_Z0
    p = _box((x0, x1), (-CAP_Y, CAP_Y), (z0, CAP_TOP))
    p -= _box((x0 + WALL, x1 - WALL), (-CAP_Y + WALL, CAP_Y - WALL), (z0 + FLOOR, CAP_TOP + 1))
    for x, y in LID_SCREWS:                                                     # lid bosses in the corners
        p += Cylinder(4.5, CAP_TOP - z0, align=CB).moved(Pos(x, y, z0))
        p -= hw.tap(10.0, top=CAP_TOP, entry="top").moved(Pos(x, y, 0))
    for dx, dy in MICS["holes"]:                                                # mic board: 4 mm bosses
        x, y = MIC_XY[0] + dx, MIC_XY[1] + dy
        p += Cylinder(3.5, FLOOR + 4.0, align=CB).moved(Pos(x, y, z0))
        p -= hw.tap(7.0, top=z0 + FLOOR + 4.0, entry="top").moved(Pos(x, y, 0))
    for x, y in _cap_screws():                                                  # down into the ears
        p -= hw.clear(FLOOR, top=z0 + FLOOR).moved(Pos(x, y, 0))
    p -= _box((-212.0, -204.0), (52.0, 60.0), (z0 - 1, z0 + FLOOR + 1))        # the cables down behind the face
    p.label = "audio_base"
    return p


@lru_cache(maxsize=None)
def audio_lid() -> Part:
    x0, x1 = CAP_X
    p = _box((x0, x1), (-CAP_Y, CAP_Y), (CAP_TOP, CAP_TOP + LID_T))
    for x, y in LID_SCREWS:
        p -= hw.clear(LID_T, top=CAP_TOP + LID_T).moved(Pos(x, y, 0))
    for dx, dy in MICS["mics"]:
        p -= Cylinder(1.5, 10).moved(Pos(MIC_XY[0] + dx, MIC_XY[1] + dy, CAP_TOP))
    for i in range(-3, 4):                                                      # speaker grille
        for j in range(-2, 3):
            p -= Cylinder(1.25, 10).moved(Pos(SPEAKER_XY[0] + 4.5 * j, SPEAKER_XY[1] + 4.5 * i, CAP_TOP))
    p.label = "audio_lid"
    return p


def mic_board() -> Part:
    z = CAP_Z0 + FLOOR + 4.0
    p = _box((MIC_XY[0] - MICS["d"] / 2, MIC_XY[0] + MICS["d"] / 2), (MIC_XY[1] - MICS["w"] / 2, MIC_XY[1] + MICS["w"] / 2),
             (z, z + 1.6))
    p += _box((MIC_XY[0] - 10, MIC_XY[0] + 10), (MIC_XY[1] - 25.0, MIC_XY[1] + 25.0), (z + 1.6, z + MICS["t"]))
    for dx, dy in MICS["holes"]:
        p -= Cylinder(1.6, 10).moved(Pos(MIC_XY[0] + dx, MIC_XY[1] + dy, z))
    p.label = "USB mic array"
    return p


def speaker() -> Part:
    w, d, h = SPEAKER
    p = _box((SPEAKER_XY[0] - d / 2, SPEAKER_XY[0] + d / 2), (SPEAKER_XY[1] - w / 2, SPEAKER_XY[1] + w / 2), (CAP_TOP - h, CAP_TOP))
    p.label = "micro speaker"
    return p


def hardware() -> hw.Hardware:
    """The face unit's and the audio cap's hardware on the mast."""
    h = hw.Hardware()
    h.part("5 in display 800x480", "5 in 800 x 480 IPS HDMI display + USB touch, 121 x 77 x 12 with its driver on its back "
           "(Waveshare 5inch HDMI LCD class; VERIFY size, connectors and depth); HDMI + USB to the Jetson",
           "Waveshare / generic (~$40)", panel(), Location())
    h.part("polycarbonate window 2 mm", "2 mm clear polycarbonate, cut to 121 x 77 (in front of the panel)",
           "McMaster 8574K (polycarbonate sheet, 1/16 in) (est)", window(), Location())
    fl = face_loc()
    for z in SIDE_SCREWS_Z:                     # ears -> housing rails: 9.5 mm of ear, then 8 mm of thread
        for s in (1, -1):
            h.screw("PAN", 18, fl * Pos(SCREW_X, s * EAR_Y_OUT, z) * hw.rot_to("y" if s > 0 else "-y"))
    for y, z in BACK_SCREWS:                    # back cover -> rails
        h.screw("PAN", 12, fl * Pos(-DEPTH - BACK_T, y, z) * hw.rot_to("-x"))
    h.part("USB mic array", "USB 2-mic array, echo cancellation and beamforming, speaker output (Seeed ReSpeaker Lite class, "
           "VERIFY board size, holes and amp power)", "Seeed Studio (~$25 est)", mic_board(), Location(), text="MIC ARRAY")
    h.part("micro speaker 4 ohm 3 W", "4 ohm 3 W enclosed micro speaker 40 x 28 x 12, on VHB tape under the lid (VERIFY)",
           "generic (~$5)", speaker(), Location(), text="SPEAKER", face="bottom")
    for dx, dy in MICS["holes"]:                # mic board -> bosses
        h.screw("PAN", 8, Pos(MIC_XY[0] + dx, MIC_XY[1] + dy, CAP_Z0 + FLOOR + 4.0 + 1.6))
    for x, y in LID_SCREWS:
        h.screw("PAN", 10, Pos(x, y, CAP_TOP + LID_T))
    return h


def printed() -> list[tuple[str, Part, Location, str, int, str]]:
    """(name, the print (ROBOT frame), where it goes, material, infill, orientation)."""
    return [("face_housing", housing(), Location(), "PETG", 30, "front face down"),
            ("face_back", back_cover(), Location(), "PETG", 30, "flat"),
            ("audio_base", audio_base(), Location(), "PETG", 40, "floor down"),
            ("audio_lid", audio_lid(), Location(), "PETG", 30, "flat")]
