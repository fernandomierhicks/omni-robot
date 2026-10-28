"""The head (design.md 12.1, Oct 24): one stiff camera bar across the front, indexed between the catch robot's 30 deg up
and the arm robot's 40 deg down, and the 5 in face (display.py) between the cameras. ROBOT frame.

  bar          6061 square tube 20 x 20 x 1.5, 228 long, along y; its axis at AXIS, behind the display. It turns about
               its own axis between two indexed stops; both cameras ride on it, so their relative pose (what stereo
               depth depends on) never changes. The pair is calibrated once on the bar.
  cam_holder   printed, one per camera at the bar's ends (|y| 64.5-114): a sleeve on the tube (square bore, one M3 x 30
               through the sleeve and both tube walls: rotation and position) with the OV9782 board on its front face
               (4 x M2). The sleeve's inner end is a 34 mm journal in its cheek's bore. L: two tapped index holes in
               its journal (catch, arm); R: plain, it slides in its bore (plate flex can't bend the bar).
  cheek        printed, one per side at |y| 64-72: a pad on the plate's front edge (4 x M3 into the plate), a web under
               the module's swept volume, a ring round the bar (34.4 bore), and a horn beside the display that carries
               the face unit (2 x M3 each side). Cheek L takes the index screw: an M3 x 10 through its ring into the
               holder's journal. A swap moves that one screw.

Where things are and why:
  - The cameras' optical centres sit at (192, +-95, 211) at the catch index (the layout study's 0.184 / 0.201 camera
    body, moved 7 mm back so the lens stays inside the bumper's 217 mm radius), 190 mm apart, 30 deg up. At the arm
    index (-40) they come to (183, +-95, 166): the lens isn't on the bar's axis (the display stands in front of the
    bar), so turning 70 deg moves it 46 mm, mostly down; the holders' lowest corner stays 9 mm over the scan plane,
    outboard of the lidar. The axis sits as low as that allows: the arm robot's SO-101, behind the bar, reaches the
    floor over it only while the bar's top stays under ~210 mm (layout_trade.py A6).
  - Nothing of the head is below z 157 (the face's back-bottom edge, the holders' corners at the arm index): the
    A2M12's scan plane (150) passes under it, so the lidar sees only the standoffs.
  - The module's swept volume (-30 ... +40 deg) comes down to the plate's front edge beside the flywheels (z 175-190 at
    x 60-90, |y| 30-90): the cheeks' pads stay 6 mm thick there and start at x 78, and the webs rise under that
    envelope. The closest the module comes is a flywheel mount, ~6 mm from a cheek (build.py's head check gives the
    exact number and tilt). The bar and the holders are ahead of it (x >= 134).
  - The horns stay behind the display's front face (seen from the front) and, at the arm index, out of the cameras'
    view (which the horn's lower front corner would otherwise enter): their front edge follows the line x = 188.5 +
    0.839 (z - 160) near the bottom.
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import Align, Box, Circle, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, extrude

import dims as D
import hw

CB = (Align.CENTER, Align.CENTER, Align.MIN)
AXIS = (155.0, 195.0)                  # the bar's axis (x, z): its top at the arm index stays under 210 (the arm
                                       # behind it reaches the floor over it; at 219 it reached nothing)
INDEX = {"catch": 30.0, "arm": -40.0}  # the bar's pitch (deg, + up) = the cameras' pitch
TUBE = dict(a=20.0, t=1.5, half=114.0)
CAM_Y = D.CAM_BASELINE / 2             # 95
# in the bar's frame (x' along the cameras' view, y along the bar, z' the cameras' up; origin on the axis)
BOARD_X, BOARD_Z = 24.0, -4.64         # the camera board's back face and its centre
LENS_OC = 16.0                         # board back -> optical centre (M12 lens, est; VERIFY on the module)
SLEEVE_R = 17.0                        # holder sleeve / journal radius
HOLDER_Y = (64.5, TUBE["half"] + 3.0)  # holder (journal + sleeve) along |y|, 3 mm past the tube's end
BLOCK = dict(x=(8.0, BOARD_X), y=(74.0, TUBE["half"] + 3.0))   # the board's seat ahead of the sleeve (3 mm round
                                       # the board's M2 holes; the cheek ends at 72)
CROSS_Z = 13.5                         # holder -> tube cross screw: head seat (z'), counterbored from the sleeve's top
CHEEK_Y = (64.0, 72.0)
BORE_R, RING_R = 17.2, 21.2            # cheek bore (0.2 mm round the journal: the R holder slides) and ring
INDEX_ANGLE = 130.0                    # index screw's direction from the axis (deg from +x toward +z), ROBOT frame:
                                       # in the journal at 100 deg (catch) and 170 deg (arm), 10 deg off a tube face
                                       # each, so an M3 x 10 from a flat at INDEX_FLAT has 6.6 mm of thread and ends
                                       # 0.3 mm short of the tube. (The cheek's web and horn fill the other angles
                                       # where both holes would be that close to a face.)
INDEX_FLAT = 20.4                      # radius of the flat cut on the ring for its head
PAD = dict(x=(78.0, 97.0), y=(55.0, 81.0), t=6.0)        # cheek's pad on the plate (trimmed to the plate's edge);
                                                         # the flywheel mounts pass 8 mm behind its end at +34 deg
WEB_BOT = 158.0                        # cheek's underside beyond the plate's edge (scan plane 150)


def bar_loc(pitch: float = INDEX["catch"]) -> Location:
    """The bar's frame -> ROBOT at a pitch (deg): x' along the cameras' view, y along the bar, z' their up."""
    return Pos(AXIS[0], 0, AXIS[1]) * Rot(0, -pitch, 0)


def index_loc(pitch: float) -> Location:
    """Moves the bar's parts (modelled at the catch index) to another pitch."""
    return bar_loc(pitch) * bar_loc(INDEX["catch"]).inverse()


def lens(pitch: float = INDEX["catch"], side: int = 1):
    """Optical centre of a camera (x, y, z) in the ROBOT frame."""
    c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    a, b = BOARD_X + LENS_OC, BOARD_Z
    return AXIS[0] + a * c - b * s, side * CAM_Y, AXIS[1] + a * s + b * c


def cam_loc(side: int = 1, pitch: float = INDEX["catch"]) -> Location:
    """OV9782 board frame (vendor.camera(): back at z = 0, lens +Z) -> ROBOT."""
    return bar_loc(pitch) * Pos(BOARD_X, side * CAM_Y, BOARD_Z) * Rot(0, 90, 0)


def _ycyl(r, y0, y1, x=0.0, z=0.0) -> Part:
    return Cylinder(r, y1 - y0, rotation=(-90, 0, 0), align=CB).moved(Pos(x, y0, z))


def _radial(angle_deg: float, r: float, y: float) -> Location:
    """A frame on the axis' circle of radius r at angle (from +x' toward +z'), +Z pointing outward."""
    return Pos(0, y, 0) * Rot(0, 90 - angle_deg, 0) * Pos(0, 0, r)


# --- the bar ---------------------------------------------------------------------------------------------------------

def tube() -> Part:
    """6061 square tube 20 x 20 x 1.5 x 228 in the bar's frame (at the origin): 3.4 mm cross holes at +-95."""
    a, t, h = TUBE["a"], TUBE["t"], TUBE["half"]
    p = Box(a, 2 * h, a) - Box(a - 2 * t, 2 * h + 1, a - 2 * t)
    for s in (1, -1):
        p -= Cylinder(D.CLR_M3 / 2, a + 2).moved(Pos(0, s * CAM_Y, 0))
    p.label = "camera bar"
    return p


@lru_cache(maxsize=None)
def cam_holder(side: int) -> Part:
    """+1: L (index holes in its journal), -1: R. ROBOT frame at the catch index."""
    y0, y1 = HOLDER_Y
    p = _ycyl(SLEEVE_R, y0, y1)
    bx, by = BLOCK["x"], BLOCK["y"]
    zb = (BOARD_Z - 21.0, BOARD_Z + 21.0)               # 2 mm past the board: 3 mm round its M2 holes
    p += Box(bx[1] - bx[0], by[1] - by[0], zb[1] - zb[0], align=(Align.MIN, Align.MIN, Align.MIN)).moved(
        Pos(bx[0], by[0], zb[0]))
    a = TUBE["a"] + 0.3
    p -= Box(a, 2 * y1, a)                                                      # the tube's bore
    board = Pos(BOARD_X, CAM_Y, BOARD_Z) * Rot(0, 90, 0)
    for dx in (-17.0, 17.0):                                                    # the board: 4 x M2 x 6
        for dy in (-17.0, 17.0):
            p -= hw.tap(5.0, d=D.TAP_M2).moved(board * Pos(dx, dy, 0))
    p -= Box(18.0, 18.0, 3.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(board * Pos(0, 0, 0.01))   # its back
    p -= Box(4.0, 22.0, 30.0, align=(Align.MAX, Align.CENTER, Align.MAX)).moved(Pos(BOARD_X + 0.01, CAM_Y, BOARD_Z - 2))  # FFC
    # cross screw through the sleeve and the tube: counterbored head, clearance through the top, thread in the bottom
    p -= Cylinder(D.CB_M3[0] / 2, 10.0, align=CB).moved(Pos(0, CAM_Y, CROSS_Z))
    p -= Cylinder(D.CLR_M3 / 2, CROSS_Z - TUBE["a"] / 2 + 0.2, align=CB).moved(Pos(0, CAM_Y, TUBE["a"] / 2 - 0.1))
    p -= hw.tap(SLEEVE_R - TUBE["a"] / 2 + 0.2, top=-TUBE["a"] / 2 + 0.1).moved(Pos(0, CAM_Y, 0))
    if side > 0:                                                                # index holes: the screw's thread
        ym = sum(CHEEK_Y) / 2
        for pitch in INDEX.values():
            p -= hw.tap(7.0, top=0.0).moved(_radial(INDEX_ANGLE - pitch, SLEEVE_R + 0.01, ym))
    p -= hw.edge_chamfer(SLEEVE_R, 0.5, 0.0, up=False).moved(Pos(0, y0, 0) * Rot(-90, 0, 0))       # journal lead-in
    p = p.moved(bar_loc(INDEX["catch"]))
    if side < 0:
        p = p.mirror(Plane.XZ)
    p.label = "cam_holder"
    return p


# --- cheeks ----------------------------------------------------------------------------------------------------------

def _xz_prism(pts, y0, y1) -> Part:
    return extrude(Plane.XZ.offset(-y0) * Polygon(*pts, align=None), y1 - y0, dir=(0, 1, 0))


def horn_front(z: float) -> float:
    """x of the horn's front edge at height z: behind the display's front face, and out of the cameras' view at the
    arm index (a point of the cheek's outer face is in view there only ahead of x = 190 + 0.839 (z - 160))."""
    import display as DP
    return min(188.5 + 0.839 * (z - 160.0), DP.front_x(z) - 0.5)


@lru_cache(maxsize=None)
def cheek(side: int) -> Part:
    """+1: L (+y, the index screw), -1: R. ROBOT frame."""
    import display as DP
    y0, y1 = CHEEK_Y
    ax, az = AXIS
    w0 = PAD["x"][0] + 6.0                                       # the web starts 6 mm into the pad (the flywheel
    web = [(w0, D.PLATE_TOP), (108.0, D.PLATE_TOP),               # mounts pass over its back end at +32 deg) (116.0, WEB_BOT), (ax + 5, WEB_BOT), (ax + 5, az - 10),
           (ax - RING_R + 0.5, az + 0.5), (120.0, 196.0), (100.0, 183.0), (w0 + 1.0, 176.0), (w0, D.PLATE_TOP + PAD["t"])]
    zt = DP.TOP_SCREW[1] + 6.0                                   # the horn's top: 6 mm over the face's top screw
    zs = [WEB_BOT] + [160.0 + 2.0 * k for k in range(1, 12)] + [zt]
    horn = [(ax, WEB_BOT)] + [(horn_front(z), z) for z in zs] + [(DP.TOP_SCREW[0] - 7.0, zt), (ax - 2.0, az + RING_R - 1.0)]
    p = _xz_prism(web, y0, y1) + _xz_prism(horn, y0, y1) + _ycyl(RING_R, y0, y1, ax, az)
    px, py = PAD["x"], PAD["y"]
    pad = Box(px[1] - px[0], py[1] - py[0], PAD["t"], align=(Align.MIN, Align.MIN, Align.MIN)).moved(Pos(px[0], py[0], D.PLATE_TOP))
    for yy, u in ((y0, (0, -1, 0)), (y1, (0, 1, 0))):             # fillets: web into the pad (1.5: the pad's
        pad += hw.cove_at(1.5, (w0, yy, D.PLATE_TOP + PAD["t"]), u, (0, 0, 1), (1, 0, 0), px[1] - w0 - 1.0)  # heads)
    pad -= hw.cove_at(2.5, (px[0], py[0], D.PLATE_TOP + PAD["t"]), (1, 0, 0), (0, 0, -1), (0, 1, 0), py[1] - py[0])  # round
    pad &= Cylinder(D.PLATE_R - 1.0, 50, align=CB).moved(Pos(0, 0, D.PLATE_TOP - 1))
    p += pad
    p -= _ycyl(BORE_R, y0 - 1, y1 + 1, ax, az)
    for c in (y0, y1):
        p -= hw.lead_in(BORE_R).moved(Pos(ax, c, az) * (Rot(90, 0, 0) if c == y0 else Rot(-90, 0, 0)))
    for x, y in HEAD_SCREWS:                                     # pad -> plate
        p -= hw.clear(PAD["t"], top=D.PLATE_TOP + PAD["t"]).moved(Pos(x, y, 0))
    for x, z in (DP.BOTTOM_SCREW, DP.TOP_SCREW):                 # face unit: M3 along -y through the horn
        p -= _ycyl(D.CLR_M3 / 2, y0 - 1, y1 + 1, x, z)
    if side > 0:                                                 # index screw: radial, its head on a flat
        p -= hw.clear(INDEX_FLAT, top=0.0).moved(Pos(ax, 0, az) * _radial(INDEX_ANGLE, INDEX_FLAT, sum(CHEEK_Y) / 2))
        p -= Box(12.0, y1 - y0 + 2, 5.0, align=(Align.CENTER, Align.MIN, Align.MIN)).moved(
            Pos(ax, 0, az) * _radial(INDEX_ANGLE, INDEX_FLAT, y0 - 1))
    p.label = "cheek"
    if side < 0:
        p = p.mirror(Plane.XZ)
        p.label = "cheek"
    return p


# the cheeks' pads -> the payload plate (tapped there), +y; mirrored for -y
HEAD_SCREWS = ((81.5, 59.5), (93.0, 59.5), (84.0, 76.5))


def hardware(pitch: float = INDEX["catch"]) -> hw.Hardware:
    """The bar's own hardware at a pitch: the tube, the cameras and their screws."""
    import vendor as V
    h = hw.Hardware()
    h.part("camera bar 20 x 20 tube", "6061 square tube 20 x 20 x 1.5, 228 mm; 2 x 3.4 cross holes at +-95 mm (holders)",
           "McMaster 6546K (6061 square tube 3/4 in) or 20 x 20 x 1.5 metric stock", tube(), bar_loc(pitch))
    for s in (1, -1):
        h.screw("PAN", 30, bar_loc(pitch) * Pos(0, s * CAM_Y, CROSS_Z))            # holder -> tube -> holder
        cam = cam_loc(s, pitch)
        h.part("OV9782 camera", "OV9782 global-shutter colour camera, M12 lens, 22-pin FFC (MCU-triggered)",
               "Arducam / Luxonis-class module (VERIFY driver at 1280x800 @ 120 fps)", V.camera(), cam)
        for dx in (-17.0, 17.0):
            for dy in (-17.0, 17.0):
                h.screw("PAN", 6, cam * Pos(dx, dy, 1.6), size="M2")
    return h


def cheek_hardware(pitch: float = INDEX["catch"]) -> hw.Hardware:
    """The cheeks' screws into the plate, and the index screw (only at an index: between them the bar turns free)."""
    h = hw.Hardware()
    ax, az = AXIS
    if any(abs(pitch - v) < 1e-6 for v in INDEX.values()):
        h.screw("PAN", 10, Pos(ax, 0, az) * _radial(INDEX_ANGLE, INDEX_FLAT, sum(CHEEK_Y) / 2))
    import vendor as V
    h.part("ELRS receiver", "ELRS 2.4 GHz receiver (RadioMaster RP2 class), 3M dual-lock on cheek R's outer face, antenna "
           "up (a base part: it stays when the payload kits are swapped)", "RadioMaster ($15)",
           V.board(22.0, 14.0, 5.0, "ELRS receiver"), Pos(115.0, -CHEEK_Y[1] - 0.5, 177.0) * hw.rot_to("-y"))
    for s in (1, -1):
        for x, y in HEAD_SCREWS:                                    # pads -> plate (all 6 mm of it)
            h.screw("PAN", 12, Pos(x, s * y, D.PLATE_TOP + PAD["t"]))
    return h


def printed() -> list[tuple[str, Part, str, int, str]]:
    return [("cheek_L", cheek(1), "PETG", 40, "on its inner face"),
            ("cheek_R", cheek(-1), "PETG", 40, "on its inner face (mirror of L without the index hole)"),
            ("cam_holder_L", cam_holder(1), "PETG", 60, "on the sleeve's outer end"),
            ("cam_holder_R", cam_holder(-1), "PETG", 60, "on the sleeve's outer end (mirror of L without the index holes)")]
