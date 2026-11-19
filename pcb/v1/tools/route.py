"""Preliminary 2-layer autoroute of the carrier (Nov 18 2026): can the placement study route on two layers?

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" tools/route.py [passes]      (from pcb/v1, after tools/gen_pcb.py)

Reads omni_carrier.kicad_pcb (the placement, untouched), sets the routing rules, exports Specctra DSN, runs the Freerouting CLI
(FREEROUTING env var, else freerouting-cli.exe on PATH; v2.5.0 from github.com/freerouting/freerouting releases, no Java needed), imports
the session, pours GND on both layers and writes omni_carrier_routed.kicad_pcb (a study: the placement board stays the source until the
layout is approved; then the routed board becomes KiCad's). Then run KiCad's DRC on it:
    kicad-cli pcb drc --format json --severity-all -o out/route/drc_routed.json omni_carrier_routed.kicad_pcb

Rules (OSH Park 2-layer: 6 mil trace / space, 13 mil drill, 7 mil ring; the project file's classes, gen_pcb.project_rules): signals
0.25 mm, power (+5V, +3V3, GND) 0.5 mm, clearance 0.2 mm, vias 0.8 mm / 0.4 mm drill, copper 0.5 mm in from the edge. Freerouting's
automatic neckdown is off (it took tracks to 0.125 mm between pins, under OSH Park's 0.152).
"""
import os
import re
import shutil
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(HERE, "omni_carrier.kicad_pcb")
OUT = os.path.join(HERE, "omni_carrier_routed.kicad_pcb")
WORK = os.path.join(HERE, "out", "route")
MM = pcbnew.FromMM
SIGNAL_W, POWER_W, CLEAR = 0.2, 0.5, 0.2
VIA_D, VIA_DRILL = 0.8, 0.4
POWER = ("+5V", "+3V3", "GND")


def router():
    """the router command: FREEROUTING_JAR (+ JAVA, a Java 25 java.exe: the jar is class version 69) runs freerouting-2.5.0.jar
    headless. The v2.5.0 native freerouting-cli.exe crashed on Windows in its board-snapshot serialisation (Nov 18), so the jar it is.
    Neckdown stays on: without it the router stalls at 34-37 unrouted on this board (Nov 19); it narrows tracks to 0.12-0.19 mm between
    pads (neck_width_um is not honoured), and NECK_MIN below widens those after import."""
    jar = os.environ.get("FREEROUTING_JAR")
    if jar:
        return [os.environ.get("JAVA") or shutil.which("java") or "java", "-jar", jar, "--gui.enabled=false",
                "--router.automatic_neckdown=true"]
    exe = os.environ.get("FREEROUTING") or shutil.which("freerouting-cli.exe")
    if not exe:
        sys.exit("set FREEROUTING_JAR to freerouting-2.5.0.jar (and JAVA to a Java 21 java.exe)")
    return [exe]


def power_class(dsn):
    """move the power nets out of kicad_default into a class of their own, POWER_W wide"""
    m = re.search(r"\(class kicad_default (.*?)\n\s*\(circuit", dsn, re.S)
    nets = m.group(1).split()
    rest = [n for n in nets if n not in POWER]
    via = re.search(r'\(use_via "([^"]+)"\)', dsn).group(1)
    um = lambda v: int(round(v * 1000))
    default = "(class kicad_default " + " ".join(rest) + "\n      (circuit"
    dsn = dsn[:m.start()] + default + dsn[m.end():]
    pclass = (f'    (class power {" ".join(n for n in POWER if n in nets)}\n'
              f'      (circuit\n        (use_via "{via}")\n      )\n'
              f'      (rule\n        (width {um(POWER_W)})\n        (clearance {um(CLEAR)})\n      )\n    )\n')
    i = dsn.index("(class kicad_default")
    return dsn[:i] + pclass.lstrip() + "    " + dsn[i:]


EDGE_KEEP = 0.3                             # the router's boundary 0.3 mm inside the board edge: with its 0.2 clearance, copper 0.5 from the edge


def shrink_boundary(dsn):
    """move the DSN's rectangular board boundary EDGE_KEEP inward (Freerouting keeps only its clearance to the boundary)"""
    m = re.search(r"\(boundary\s*\(path pcb 0 ([-\d. ]+)\)", dsn)
    v = [float(x) for x in m.group(1).split()]
    xs, ys = v[0::2], v[1::2]
    k = EDGE_KEEP * 1000
    x0, x1, y0, y1 = min(xs) + k, max(xs) - k, min(ys) + k, max(ys) - k
    path = f"{x1:g} {y0:g}  {x0:g} {y0:g}  {x0:g} {y1:g}  {x1:g} {y1:g}  {x1:g} {y0:g}"
    return dsn[:m.start(1)] + path + dsn[m.end(1):]


NECK_MIN = 0.16                             # no track under this (OSH Park: 0.1524); the router kept CLEAR around the narrow one, so
                                            # widening a 0.12 to 0.16 leaves >= CLEAR - 0.02 = 0.18 to its neighbours


def widen_necks(board):
    n = 0
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T and t.GetWidth() < MM(NECK_MIN):
            t.SetWidth(MM(NECK_MIN))
            n += 1
    return n


def gnd_pour(board):
    net = board.FindNet("GND")
    bb = board.GetBoardEdgesBoundingBox()
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(net)
        z.SetLocalClearance(MM(0.3))
        z.SetMinThickness(MM(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(MM(0.4))
        z.SetThermalReliefSpokeWidth(MM(0.4))
        z.SetIsFilled(False)
        ol = z.Outline()
        ol.NewOutline()
        for x, y in ((bb.GetLeft(), bb.GetTop()), (bb.GetRight(), bb.GetTop()), (bb.GetRight(), bb.GetBottom()), (bb.GetLeft(), bb.GetBottom())):
            ol.Append(x, y)
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def main():
    passes = sys.argv[1] if len(sys.argv) > 1 else "40"
    os.makedirs(WORK, exist_ok=True)
    # LoadBoard locks the project (~omni_carrier.kicad_pro.lck) and does not release it on exit; the next LoadBoard then waits on an
    # invisible "project is locked" prompt (Nov 18). Clear our own stale lock first and ours again at the end. Do not run this while the
    # project is open in KiCad.
    lock = os.path.join(HERE, "~omni_carrier.kicad_pro.lck")
    if os.path.exists(lock):
        os.remove(lock)
    try:
        route(passes)
    finally:
        if os.path.exists(lock):
            os.remove(lock)


def route(passes):
    board = pcbnew.LoadBoard(SRC)
    nc = board.GetDesignSettings().m_NetSettings.GetDefaultNetclass()
    nc.SetTrackWidth(MM(SIGNAL_W))
    nc.SetClearance(MM(CLEAR))
    nc.SetViaDiameter(MM(VIA_D))
    nc.SetViaDrill(MM(VIA_DRILL))
    dsn = os.path.join(WORK, "omni_carrier.dsn")
    ses = os.path.join(WORK, "omni_carrier.ses")
    assert pcbnew.ExportSpecctraDSN(board, dsn), "DSN export failed"
    with open(dsn, encoding="utf-8") as f:
        text = f.read()
    # the project's net classes (gen_pcb.project_rules) export the power class themselves; power_class only for a board without them
    own = re.search(r"\(class (?!kicad_default)[^\n]*\+5V", text)
    text = shrink_boundary(text if own else power_class(text))
    with open(dsn, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    if os.path.exists(ses):
        os.remove(ses)
    log = os.path.join(WORK, "freerouting.log")
    with open(log, "w", encoding="utf-8") as f:
        subprocess.run(router() + ["-de", dsn, "-do", ses, "-mp", passes], stdout=f, stderr=subprocess.STDOUT, check=False,
                       timeout=1800)
    if not os.path.exists(ses):
        sys.exit(f"no session written: see {log}")
    assert pcbnew.ImportSpecctraSES(board, ses), "SES import failed"
    widened = widen_necks(board)
    gnd_pour(board)
    board.Save(OUT)
    tracks = [t for t in board.GetTracks()]
    vias = sum(1 for t in tracks if t.Type() == pcbnew.PCB_VIA_T)
    length = sum(pcbnew.ToMM(t.GetLength()) for t in tracks if t.Type() == pcbnew.PCB_TRACE_T)
    print(f"{len(tracks) - vias} track segments ({length:.0f} mm), {vias} vias, {widened} necks widened to {NECK_MIN} mm -> {os.path.basename(OUT)}")


if __name__ == "__main__":
    main()
