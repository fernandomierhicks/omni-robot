"""Place the omni-robot Teensy carrier's footprints on a board (placement study: no tracks yet).

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" tools/gen_pcb.py      (from pcb/v1, after tools/check.py)

Reads KiCad's own netlist (out/omni_carrier.net.xml), loads every footprint, assigns its nets and its symbol link (so
KiCad's "Update PCB from Schematic" keeps working), places it from PLACE below and writes omni_carrier.kicad_pcb with
the outline, OSH Park 2-layer rules and the mounting holes. Board frame: x right = robot +x (forward), y down =
robot -y (right side); the carrier sits on the power stack in cad/v15 (upper.py BLOCK['mcu'], 100 x 65 envelope at
robot (37, -143.5)). Edges follow the cables: -x the computer (Teensy USB), +y robot (top edge here) the IMU, ELRS and
the face / LEDs, -y robot (bottom edge) the motor chain slot and the converters below, +x the spares.
"""
import json
import os
import sys
import xml.etree.ElementTree as ET

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_sch                                  # noqa: E402  (CAN_MODEL; gen_sch imports only the standard library)

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KFP = os.environ.get("KICAD_FP", r"C:\Program Files\KiCad\10.0\share\kicad\footprints")
NET_XML = os.path.join(HERE, "out", "omni_carrier.net.xml")
OUT = os.path.join(HERE, "omni_carrier.kicad_pcb")
W, H = 102.0, 62.0                      # Nov 18: 7 mm wider for the corner holes (was 95 x 62; the CAD adapts)
MARGIN = 0.5                            # courtyard to board edge
HOLE_IN = 3.5                           # M3 corner holes: centre 3.5 mm in from both edges (Fernando, Nov 18)
CORNER = 7.0                            # corner kept clear for the hole (its courtyard: r 3.45) and the screw head
GAP = 0.6                               # courtyard gap between neighbouring edge connectors
MM = pcbnew.FromMM


def load_fp(fpid):
    lib, name = fpid.split(":")
    path = os.path.join(HERE, "omni_carrier.pretty") if lib == "omni_carrier" else os.path.join(KFP, lib + ".pretty")
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        raise FileNotFoundError(fpid)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def courtyard(fp):
    """courtyard bbox (mm) relative to the footprint's position, at its current orientation"""
    bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
    if bb.GetWidth() == 0:
        bb = fp.GetBoundingBox(False)
    p = fp.GetPosition()
    return (pcbnew.ToMM(bb.GetLeft() - p.x), pcbnew.ToMM(bb.GetRight() - p.x),
            pcbnew.ToMM(bb.GetTop() - p.y), pcbnew.ToMM(bb.GetBottom() - p.y))


def put(fp, rot=0, x=None, y=None, left=None, right=None, top=None, bottom=None):
    """orient, then place so the given courtyard edges (or the origin, x / y) land where asked"""
    fp.SetOrientationDegrees(rot)
    fp.SetPosition(pcbnew.VECTOR2I(0, 0))
    l, r, t, b = courtyard(fp)
    px = x if x is not None else (left - l if left is not None else right - r)
    py = y if y is not None else (top - t if top is not None else bottom - b)
    fp.SetPosition(pcbnew.VECTOR2I(MM(px), MM(py)))
    return px + l, px + r, py + t, py + b                 # placed courtyard


# ----------------------------------------------------------------------------------------------- placement
def place(fps):
    """every footprint's place (positions set in place)"""
    E = MARGIN
    # top edge (robot +y: IMU, ELRS, face): pins along x, bodies inboard
    x = CORNER
    left = {}
    for ref in ("J4", "J3", "J20", "J15"):
        left[ref], x, _, _ = put(fps[ref], 0, left=x, top=E)
        x += GAP
    # Nov 18 (Fernando): the ELRS receiver plugs into J5 (1x4 female socket) and lies flat along the top edge, 11 mm up,
    # its pads at J5 (right), its body and U.FL toward -x over the strip above the Teensy (tools/elrs_model.py); the
    # footprint (omni_carrier:ELRS_Rx_Socket_1x04_P2.54mm) carries the receiver's outline in its courtyard and its 3D model
    put(fps["J5"], 0, right=BIAS_AT["J11"][0] - 0.1, top=E)
    # the Teensy: USB end at the left edge (rot 90: pad 1 at the USB end, pads 25-48 in the row 15.24 above)
    put(fps["U1"], 90, x=3.6, y=30.0)
    # right edge: the three XH6 spares, each with its bias column on its left; in from the edge by the corner holes
    y = E
    for ref in ("J11", "J16", "J17"):
        _, _, _, y = put(fps[ref], 90, right=W - CORNER - 0.2, top=y)
        y += 1.2
    # bottom edge (robot -y: the motor chain's slot, the converters below)
    x = CORNER
    for ref in ("J6", "J10", "J12", "J21", "J13"):              # Nov 19: J1 (pack sense) gone, the INA226 reads the pack
        _, x, _, _ = put(fps[ref], 180, left=x, bottom=H - E)
        x += GAP
    # left edge under the Teensy's USB end: J18 and its bias column
    put(fps["J18"], 90, left=E, top=33.0)
    # CAN modules (standing in 1x6 sockets)
    put(fps["U2"], 90, left=15.0, top=34.0)
    put(fps["U3"], 90, left=15.0, top=41.5)
    # J19 (analog spare) in the interior above the bottom row, its two bias resistors beside it
    put(fps["J19"], 180, left=58.0, bottom=52.5)

    def row(refs, x0, y0, rot=0, dx=5.6):
        for i, ref in enumerate(refs):
            put(fps[ref], rot, left=x0 + i * dx, top=y0)
    row(["R26", "R20", "R21", "R22"], left["J20"] + 2.0, 9.0)    # NeoPixel, RGB series resistors: under J20 / J15
    put(fps["BZ1"], 0, x=BIAS_AT["J11"][0] - 10.55, y=19.6)                          # Ø12 buzzer, soldered on: top-right corner, past the Teensy's end
    put(fps["Q1"], 0, left=46.4, top=41.0)                        # its switch, pull-down and flyback: between C6 and J19
    row(["R12", "D1"], 54.8, 40.6, rot=90, dx=3.4)                # right of Q1 (TO-92 wide, Nov 18)
    row(["R15", "D3"], 33.0, 34.0)                                # 3V3 LED
    row(["C5", "C6"], 33.0, 40.5, dx=6.6)                         # bulk caps
    # spare bias resistors (a column beside each spare connector) and that connector's 3-pad jumper at its end
    for conn, (x0, y0, step) in BIAS_AT.items():
        rs = sorted(BIAS[conn], key=lambda r: int(r[1:]))
        for i, ref in enumerate(rs):
            put(fps[ref], 0, left=x0, top=y0 + i * step)
        put(fps[f"JP{conn[1:]}"], 0, left=x0, top=y0 + len(rs) * step + 0.3)
    # M3 mounting holes at the corners
    for ref, (hx, hy) in zip(("H1", "H2", "H3", "H4"), HOLES):
        put(fps[ref], 0, x=hx, y=hy)


BIAS = {}                                        # spare connector -> its bias resistors (filled from the netlist)
BIAS_AT = {"J11": (W - 19.9, 0.8, 3.2), "J16": (W - 19.9, 20.4, 3.2), "J17": (W - 19.9, 40.0, 3.2),
           "J18": (8.2, 33.0, 3.2), "J19": (75.6, 44.0, 3.2)}           # J19: XH5 since Nov 19 (pin 41 joined it)
# Nov 18 (Fernando): the four M3 holes at the board's corners (were inside the board); the CAD adapts
HOLES = ((HOLE_IN, H - HOLE_IN), (HOLE_IN, HOLE_IN), (W - HOLE_IN, HOLE_IN), (W - HOLE_IN, H - HOLE_IN))


# OSH Park 2-layer fab limits and the routing classes, in the project file: KiCad 10 keeps a board's design rules in the .kicad_pro, so
# the values set on the BOARD above do not survive board.Save and DRC would use KiCad's defaults (0.2 mm track, 0.5 mm to the edge).
RULES = {"min_clearance": 0.1524, "min_track_width": 0.1524, "min_through_hole_diameter": 0.33, "min_via_diameter": 0.6858,
         "min_via_annular_width": 0.1778, "min_copper_edge_clearance": 0.381, "min_hole_clearance": 0.25,
         "min_hole_to_hole": 0.25}
CLASSES = [{"name": "Default", "track_width": 0.2, "clearance": 0.2, "via_diameter": 0.8, "via_drill": 0.4},
           {"name": "Power", "track_width": 0.5, "clearance": 0.2, "via_diameter": 0.8, "via_drill": 0.4}]
POWER_NETS = ("+5V", "+3V3", "GND")


def project_rules():
    pro = os.path.join(HERE, "omni_carrier.kicad_pro")
    with open(pro, encoding="utf-8") as f:
        d = json.load(f)
    d.setdefault("board", {}).setdefault("design_settings", {}).setdefault("rules", {}).update(RULES)
    ns = d.setdefault("net_settings", {})
    ns["classes"] = CLASSES
    ns["netclass_patterns"] = [{"netclass": "Power", "pattern": n} for n in POWER_NETS]
    with open(pro, "w", encoding="utf-8", newline="\n") as f:
        json.dump(d, f, indent=2)


def main():
    root = ET.parse(NET_XML).getroot()
    board = pcbnew.BOARD()
    ds = board.GetDesignSettings()
    ds.m_TrackMinWidth = MM(0.1524)                   # OSH Park 2-layer: 6 mil trace / space, 13 mil drill, 7 mil ring
    ds.m_MinClearance = MM(0.1524)
    ds.m_MinThroughDrill = MM(0.33)
    ds.m_ViasMinSize = MM(0.33 + 2 * 0.1778)
    ds.m_CopperEdgeClearance = MM(0.381)
    nets = {}
    for net in root.iter("net"):
        name = net.get("name")
        if name.startswith("/BIAS_J") or name.startswith("BIAS_J"):
            BIAS[name.split("BIAS_")[1]] = [n.get("ref") for n in net.iter("node") if n.get("ref").startswith("R")]
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        for node in net.iter("node"):
            nets[(node.get("ref"), node.get("pin"))] = ni
    fps = {}
    for comp in root.iter("comp"):
        ref = comp.get("ref")
        fpid = comp.findtext("footprint")
        if not fpid:
            continue
        fp = load_fp(fpid)
        fp.SetReference(ref)
        fp.SetValue(comp.findtext("value"))
        ts = comp.find("tstamps")
        if ts is not None and ts.text:
            fp.SetPath(pcbnew.KIID_PATH("/" + ts.text.strip()))
        for pad in fp.Pads():
            ni = nets.get((ref, pad.GetNumber()))
            if ni is not None:
                pad.SetNet(ni)
        if ref in ("U2", "U3"):                       # the CAN module standing in KiCad's 1x6 socket (gen_sch.CAN_MODEL)
            path, off, rot = gen_sch.CAN_MODEL
            m = pcbnew.FP_3DMODEL()
            m.m_Filename = path
            m.m_Offset = pcbnew.VECTOR3D(*off)
            m.m_Rotation = pcbnew.VECTOR3D(*rot)
            fp.Models().push_back(m)
        board.Add(fp)
        fps[ref] = fp
    place(fps)
    edge = pcbnew.PCB_SHAPE(board)
    edge.SetShape(pcbnew.SHAPE_T_RECT)
    edge.SetStart(pcbnew.VECTOR2I(0, 0))
    edge.SetEnd(pcbnew.VECTOR2I(MM(W), MM(H)))
    edge.SetLayer(pcbnew.Edge_Cuts)
    edge.SetWidth(MM(0.1))
    board.Add(edge)
    board.Save(OUT)
    project_rules()
    print(f"{len(fps)} footprints placed on {W} x {H} mm -> {os.path.basename(OUT)}")


if __name__ == "__main__":
    main()
