"""Place the omni-robot Teensy carrier's footprints on a board (placement study: no tracks yet).

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" tools/gen_pcb.py      (from pcb/v1, after tools/check.py)

Reads KiCad's own netlist (out/omni_carrier.net.xml), loads every footprint, assigns its nets and its symbol link (so
KiCad's "Update PCB from Schematic" keeps working), places it from PLACE below and writes omni_carrier.kicad_pcb with
the outline, OSH Park 2-layer rules and the mounting holes. Board frame: x right = robot +x (forward), y down =
robot -y (right side); the carrier sits on the power stack in cad/v15 (upper.py BLOCK['mcu'], 100 x 65 envelope at
robot (37, -143.5)). Edges follow the cables: -x the computer (Teensy USB), +y robot (top edge here) the IMU, ELRS and
the face / LEDs, -y robot (bottom edge) the motor chain slot and the converters below, +x the spares.
"""
import os
import xml.etree.ElementTree as ET

import pcbnew

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KFP = os.environ.get("KICAD_FP", r"C:\Program Files\KiCad\10.0\share\kicad\footprints")
NET_XML = os.path.join(HERE, "out", "omni_carrier.net.xml")
OUT = os.path.join(HERE, "omni_carrier.kicad_pcb")
W, H = 95.0, 62.0                       # inside the CAD envelope (cad/v15 devices.py 'mcu': 100 x 65)
MARGIN = 0.5                            # courtyard to board edge
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
    x = E
    for ref in ("J4", "J3", "J5", "J20", "J15", "J14"):
        _, x, _, _ = put(fps[ref], 0, left=x, top=E)
        x += 1.2
    # the Teensy: USB end at the left edge (rot 90: pad 1 at the USB end, pads 25-48 in the row 15.24 above)
    put(fps["U1"], 90, x=3.6, y=30.0)
    # right edge: the three XH6 spares, each with its bias column on its left
    y = E
    for ref in ("J11", "J16", "J17"):
        _, _, _, y = put(fps[ref], 90, right=W - E, top=y)
        y += 1.2
    # bottom edge (robot -y: the motor chain's slot, the converters below)
    x = E
    for ref in ("J6", "J10", "J1", "J12", "J21", "J13"):
        _, x, _, _ = put(fps[ref], 180, left=x, bottom=H - E)
        x += 1.2
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
    row(["R26", "R20", "R21", "R22"], 52.0, 9.0)                 # NeoPixel, RGB series resistors: under J20 / J15
    put(fps["Q1"], 0, left=66.5, top=14.0)                        # buzzer switch, pull-down and flyback past the Teensy's end
    row(["R12", "D1"], 66.5, 19.8)
    row(["R15", "D3"], 33.0, 34.0)                                # 3V3 LED
    row(["C5", "C6"], 33.0, 40.5, dx=6.6)                         # bulk caps
    row(["R1", "R2", "C1"], 33.0, 47.0)                           # pack divider, by J1
    # spare bias resistors (a column beside each spare connector) and that connector's 3-pad jumper at its end
    for conn, (x0, y0, step) in BIAS_AT.items():
        rs = sorted(BIAS[conn], key=lambda r: int(r[1:]))
        for i, ref in enumerate(rs):
            put(fps[ref], 0, left=x0, top=y0 + i * step)
        put(fps[f"JP{conn[1:]}"], 0, left=x0, top=y0 + len(rs) * step + 0.3)
    # M3 mounting holes, in the gaps
    for ref, (hx, hy) in zip(("H1", "H2", "H3", "H4"), HOLES):
        put(fps[ref], 0, x=hx, y=hy)


BIAS = {}                                        # spare connector -> its bias resistors (filled from the netlist)
BIAS_AT = {"J11": (W - 13.4, 0.8, 3.2), "J16": (W - 13.4, 20.4, 3.2), "J17": (W - 13.4, 40.0, 3.2),
           "J18": (8.2, 33.0, 3.2), "J19": (73.0, 44.0, 3.2)}
HOLES = ((24.0, 50.5), (49.0, 36.5), (74.5, 26.5), (64.0, 39.5))


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
    print(f"{len(fps)} footprints placed on {W} x {H} mm -> {os.path.basename(OUT)}")


if __name__ == "__main__":
    main()
