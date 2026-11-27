"""The home world's geometry from the house's permit drawings (home.py builds the MuJoCo world from home_plan.json).

Source: "Patio Plans.pdf" (RG Permits & Design Services, job RG23-044, 5-4-23), sheet A-1 "Proposed floor plan", 1/4" =
1'-0" on a 36 x 24 in sheet. The sheet is vector: every wall, door, window and fixture is a PDF path, so nothing here is
traced by eye. What the strokes mean on that sheet:

  0 pt (hairline)  the walls, as double-line outlines (closed at the door jambs). Filled between the lines: the regions
                   they enclose that are thinner than 24 cm, plus the strip between any two parallel lines 5-21 cm apart
                   with nothing between them (some outlines are left open at a jamb). Specks under 0.02 m2 dropped.
  0.72 pt          doors (leaves, swing arcs), windows (pane lines inside the outer walls), counters and fixtures
  0.48 pt          room names, plumbing symbols (sinks, toilets), the patio's beams and fans
The frame: metres, x along the sheet's right (north: the site plan's arrow, "NORTH/RIGHT ELEVATION"), y along its top
(west, the pool side), origin at the middle of the walls' extent. Checks: the walls span 19.18 x 19.85 m against 63 x 65
ft (19.20 x 19.81 m) on FLOOR PLAN.PNG's outline.

  walls    [x0, y0, x1, y1] boxes, cm-rounded, from the filled raster (1 cm) cut into rectangles
  windows  pane lines in an outer wall (one side reaches the sheet's edge clear of every wall): [x0, y0, x1, y1] along
           the wall's middle, merged where they overlap or nearly touch
  arcs     the doors' swings: hinge, the leaf's end when shut (in the opening) and open (against the wall), width;
           home.py says which are doors (the washer / dryer lids and the range's front are arcs too)

  python home_extract.py ["<path to Patio Plans.pdf>"]      -> home_plan.json (+ renders/home_plan.png: walls, windows, doors)

Needs pypdfium2 (not in the sim's requirements: only this one-off extraction reads PDFs; any venv with pypdfium2, numpy,
scipy and pillow runs it). home_plan.json is checked in, so the sim never needs the PDF.
"""

from __future__ import annotations

import ctypes
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage as ndi

HERE = Path(__file__).resolve().parent
PDF = Path(r"C:\Dropbox\Annie Fernando\Apartments and housing\431 Monte Vista\Improvement projects\Patio\Patio Plans.pdf")
SHEET = 1                                  # 0-based: A-1
K = 0.3048 / 18                            # m per PDF point at 1/4" = 1'-0" (18 pt to the foot)
BOX = (1110, 2280, 320, 1512)              # the house's walls on the sheet, PDF points (x0, x1, y0, y1; y up)
DOOR_BOX = (1080, 2320, 300, 1560)         # with the outer doors' leaves, which swing out past the walls
RES = 0.01                                 # m: the wall raster
# openings in the outer walls with no swing to find them by, read off the sheet (world m, along the wall's middle):
# the den's sliding glass door to the pool (the gap between the den's north wall boxes; pane lines at y 8.45-8.48)
SLIDERS = {"den_slider": [-2.009, 8.455, -0.109, 8.455]}


def paths(pdf: Path, sheet: int) -> list:
    """Every path on the sheet: dict(w=stroke width pt, pts=[(type, X, Y, close)]) in page points (y up)."""
    import pypdfium2 as pdfium
    import pypdfium2.raw as raw
    doc = pdfium.PdfDocument(str(pdf))
    page = doc[sheet]
    out = []

    def walk(objs):
        for o in objs:
            if o.type == raw.FPDF_PAGEOBJ_FORM:
                n = raw.FPDFFormObj_CountObjects(o.raw)
                walk([pdfium.PdfObject(raw.FPDFFormObj_GetObject(o.raw, i), page=page, pdf=doc) for i in range(n)])
                continue
            if o.type != raw.FPDF_PAGEOBJ_PATH:
                continue
            w = ctypes.c_float()
            raw.FPDFPageObj_GetStrokeWidth(o.raw, ctypes.byref(w))
            m = raw.FS_MATRIX()
            raw.FPDFPageObj_GetMatrix(o.raw, ctypes.byref(m))
            pts = []
            for i in range(raw.FPDFPath_CountSegments(o.raw)):
                s = raw.FPDFPath_GetPathSegment(o.raw, i)
                x, y = ctypes.c_float(), ctypes.c_float()
                raw.FPDFPathSegment_GetPoint(s, ctypes.byref(x), ctypes.byref(y))
                pts.append((raw.FPDFPathSegment_GetType(s), m.a * x.value + m.c * y.value + m.e,
                            m.b * x.value + m.d * y.value + m.f, bool(raw.FPDFPathSegment_GetClose(s))))
            out.append(dict(w=round(w.value * abs(m.a * m.d - m.b * m.c) ** 0.5, 3), pts=pts))

    walk(list(page.get_objects(max_depth=1)))
    return out


def segments(ps: list, width: float, box=BOX) -> np.ndarray:
    """The straight pieces of the paths drawn at `width` (curves as their chords), page points, inside box."""
    out = []
    for p in ps:
        if p["w"] != width:
            continue
        prev = start = None
        for t, X, Y, close in p["pts"]:
            if t == 2 or prev is None:                    # FPDF_SEGMENT_MOVETO
                prev = start = (X, Y)
            else:
                out.append((*prev, X, Y))
                prev = (X, Y)
            if close and start is not None and prev != start:
                out.append((*prev, *start))
                prev = start
    a = np.array(out).reshape(-1, 4)
    keep = ((a[:, [0, 2]] > box[0]) & (a[:, [0, 2]] < box[1]) & (a[:, [1, 3]] > box[2]) & (a[:, [1, 3]] < box[3])).all(1)
    return a[keep]


def axis(a: np.ndarray) -> np.ndarray:
    return a[(np.abs(a[:, 0] - a[:, 2]) < 0.05) | (np.abs(a[:, 1] - a[:, 3]) < 0.05)]


def wall_raster(lines_m: np.ndarray, x0: float, y0: float, nx: int, ny: int) -> np.ndarray:
    """Filled walls on a RES grid (row = y, col = x) from their outlines (metres, relative to x0, y0)."""
    im = Image.new("L", (nx, ny), 0)
    dr = ImageDraw.Draw(im)
    for a, b, c, d in lines_m:
        dr.line([((a - x0) / RES, (b - y0) / RES), ((c - x0) / RES, (d - y0) / RES)], fill=255, width=2)
    line = np.array(im) > 0
    free = ~line
    lab, n = ndi.label(free)
    half = ndi.maximum(ndi.distance_transform_edt(free), lab, range(1, n + 1)) * RES
    wall = line | np.isin(lab, [i for i in range(1, n + 1) if i != lab[0, 0] and half[i - 1] < 0.12])
    # parallel pairs 5-21 cm apart with no line between them (outlines left open at a jamb)
    im2 = Image.new("L", (nx, ny), 0)
    d2 = ImageDraw.Draw(im2)
    rel = lines_m - np.array([x0, y0, x0, y0])
    for along in (0, 1):                                   # 0: horizontal lines (along x), 1: vertical
        across = 1 - along
        group = [s for s in rel if abs(s[across] - s[across + 2]) < 1e-3]
        for i, s in enumerate(group):
            lo, hi = sorted((s[along], s[along + 2]))
            for t in group[i + 1:]:
                if not 0.05 <= abs(t[across] - s[across]) <= 0.21:
                    continue
                lo2, hi2 = sorted((t[along], t[along + 2]))
                o0, o1 = max(lo, lo2), min(hi, hi2)
                if o1 - o0 < 0.04:
                    continue
                c0, c1 = sorted((s[across], t[across]))
                if any(c0 + 1e-3 < u[across] < c1 - 1e-3 and min(u[along], u[along + 2]) < o1 and
                       max(u[along], u[along + 2]) > o0 for u in group):
                    continue
                box = [(o0, c0), (o1, c1)] if along == 0 else [(c0, o0), (c1, o1)]
                d2.rectangle([(box[0][0] / RES, box[0][1] / RES), (box[1][0] / RES, box[1][1] / RES)], fill=255)
    wall |= np.array(im2) > 0
    wall = ndi.binary_opening(wall, np.ones((5, 5)))          # 2 px stray lines (a dimension's leader, a mark)
    holes = ndi.binary_fill_holes(wall) & ~wall               # pinholes at T junctions; not a closet
    lab, n = ndi.label(holes)
    if n:
        area = ndi.sum(holes, lab, range(1, n + 1)) * RES * RES
        wall |= np.isin(lab, 1 + np.flatnonzero(area < 0.01))
    lab, n = ndi.label(wall)
    area = ndi.sum(wall, lab, range(1, n + 1)) * RES * RES
    return np.isin(lab, 1 + np.flatnonzero(area > 0.02))


def rectangles(mask: np.ndarray) -> list:
    """The mask cut into rectangles (row runs merged downward while their extent stays the same): [(c0, r0, c1, r1)]
    half-open cells."""
    out, open_ = [], {}                                     # (c0, c1) -> r0
    for r in range(mask.shape[0] + 1):
        runs = set()
        if r < mask.shape[0]:
            row = np.concatenate([[False], mask[r], [False]])
            d = np.flatnonzero(np.diff(row.astype(np.int8)))
            runs = set(zip(d[::2], d[1::2]))
        for run in list(open_):
            if run not in runs:
                out.append((run[0], open_.pop(run), run[1], r))
        for run in runs:
            open_.setdefault(run, r)
    return out


def doors(ps: list, to_m) -> list:
    """Quarter-circle swings (0.72 pt curves with a chord over 0.4 m): the hinge is the corner of their square whose
    far side the arc bulges away from; the open end is the one the leaf is drawn to (a straight line from the hinge),
    the other is where the leaf's end is when shut, in the opening."""
    leaves = np.array([(*to_m(*s[:2]), *to_m(*s[2:])) for s in axis(segments(ps, 0.72, DOOR_BOX))])
    out = []
    for p in ps:
        if p["w"] != 0.72 or not any(t == 1 for t, *_ in p["pts"]):     # FPDF_SEGMENT_BEZIERTO
            continue
        pts = np.array([to_m(X, Y) for _, X, Y, _ in p["pts"]])
        a, b = pts[0], pts[-1]
        chord = float(np.linalg.norm(a - b))
        if chord < 0.4 or not (-10.5 < pts[:, 0].min() and pts[:, 0].max() < 10.5 and -10.5 < pts[:, 1].min()
                                and pts[:, 1].max() < 10.6):
            continue
        r = chord / 2 ** 0.5
        mid = pts[1:-1].mean(0)                             # the control points: on the bulge's side
        cands = [np.array([a[0], b[1]]), np.array([b[0], a[1]])]
        hinge = max(cands, key=lambda h: np.linalg.norm(h - mid))
        if abs(np.linalg.norm(a - hinge) - r) > 0.05 or abs(np.linalg.norm(b - hinge) - r) > 0.05:
            continue                                        # not a quarter circle (a curved front, a lid)
        # the sheet draws each leaf open: a straight 0.72 pt line about r long from the hinge to the arc's open end
        leaf = []
        for e in (a, b):
            u = (e - hinge) / r
            n = 0
            for s in leaves:
                p, q = s[:2], s[2:]
                for p0, p1 in ((p, q), (q, p)):
                    d = p1 - p0
                    L = np.linalg.norm(d)
                    if abs(L - r) < 0.12 * r and np.dot(d / L, u) > 0.99 and np.linalg.norm(
                            (p0 - hinge) - np.dot(p0 - hinge, u) * u) < 0.06 and abs(np.dot(p0 - hinge, u)) < 0.08:
                        n += 1
            leaf.append(n)
        if leaf[0] == leaf[1]:
            continue                                        # no leaf drawn: not a door (a lid, a front)
        open_, shut = (a, b) if leaf[0] > leaf[1] else (b, a)
        out.append(dict(hinge=np.round(hinge, 3).tolist(), shut=np.round(shut, 3).tolist(),
                        open=np.round(open_, 3).tolist(), width=round(r, 3)))
    return sorted(out, key=lambda d: (-d["hinge"][1], d["hinge"][0]))


def outside(wall: np.ndarray, x0: float, y0: float, swings: list) -> np.ndarray:
    """The cells outside the house, with every door shut (its leaf drawn across its opening) and the sliders closed:
    flood-filled from the raster's corner."""
    im = Image.fromarray((wall * 255).astype(np.uint8))
    dr = ImageDraw.Draw(im)
    shut = [(s["hinge"], s["shut"]) for s in swings] + [(v[:2], v[2:]) for v in SLIDERS.values()]
    for h, e in shut:                                       # 12 cm past each end: the leaf meets both jambs
        h, e = np.array(h), np.array(e)
        u = (e - h) / np.linalg.norm(e - h)
        a, b = h - 0.12 * u, e + 0.12 * u
        dr.line([((a[0] - x0) / RES, (a[1] - y0) / RES), ((b[0] - x0) / RES, (b[1] - y0) / RES)], fill=255, width=9)
    lab, _ = ndi.label(np.array(im) == 0)
    return lab == lab[0, 0]


def windows(ps: list, wall: np.ndarray, x0: float, y0: float, to_m, rects: list, swings: list) -> list:
    """Pane lines (0.72 pt, 0.25 m or longer, along x or y) lying in an outer wall (outside on one side, every door
    shut) and not across a door's opening, merged into spans along the wall's middle."""
    def cell(x, y):
        return int(round((y - y0) / RES)), int(round((x - x0) / RES))

    out_ = outside(wall, x0, y0, swings)

    def clear_to_edge(r, c, dr, dc):                        # the first cell past the wall is outside
        while 0 <= r < wall.shape[0] and 0 <= c < wall.shape[1] and wall[r, c]:
            r, c = r + dr, c + dc
        return 0 <= r < wall.shape[0] and 0 <= c < wall.shape[1] and bool(out_[r, c])

    def in_opening(x, y):
        for s in swings:
            h, e = np.array(s["hinge"]), np.array(s["shut"])
            t = np.clip(np.dot(np.array([x, y]) - h, e - h) / np.dot(e - h, e - h), 0, 1)
            if np.linalg.norm(np.array([x, y]) - (h + t * (e - h))) < 0.16:
                return True
        return False

    near = ndi.binary_dilation(wall, np.ones((7, 7)))
    found = {}
    for s in axis(segments(ps, 0.72)):
        (ax_, ay_), (bx_, by_) = to_m(*s[:2]), to_m(*s[2:])
        if np.hypot(bx_ - ax_, by_ - ay_) < 0.25:
            continue
        ra, ca = cell(ax_, ay_)
        rb, cb = cell(bx_, by_)
        if not (0 <= ra < wall.shape[0] and 0 <= rb < wall.shape[0] and 0 <= ca < wall.shape[1] and 0 <= cb < wall.shape[1]):
            continue
        if not (near[ra, ca] and near[rb, cb]):
            continue
        horiz = abs(ay_ - by_) < 1e-3
        mx, my = (ax_ + bx_) / 2, (ay_ + by_) / 2
        if in_opening(mx, my):
            continue
        home_rect = None
        for k, (wx0, wy0, wx1, wy1) in enumerate(rects):     # the wall box it lies in, running the same way
            if wx0 - 0.03 <= mx <= wx1 + 0.03 and wy0 - 0.03 <= my <= wy1 + 0.03 and \
                    ((wx1 - wx0) > (wy1 - wy0)) == horiz:
                home_rect = k
                break
        if home_rect is None:
            continue
        wx0, wy0, wx1, wy1 = rects[home_rect]
        r, c = cell((wx0 + wx1) / 2 if not horiz else mx, (wy0 + wy1) / 2 if horiz else my)
        dirs = ((1, 0), (-1, 0)) if horiz else ((0, 1), (0, -1))
        if not any(clear_to_edge(r, c, dr, dc) for dr, dc in dirs):
            continue                                        # an inner wall: a counter's edge, a door leaf
        lo, hi = sorted((ax_, bx_) if horiz else (ay_, by_))
        found.setdefault(home_rect, []).append((lo, hi))
    out = []
    for k, spans in found.items():
        wx0, wy0, wx1, wy1 = rects[k]
        horiz = (wx1 - wx0) > (wy1 - wy0)
        spans.sort()
        merged = [list(spans[0])]
        for lo, hi in spans[1:]:
            if lo <= merged[-1][1] + 0.12:
                merged[-1][1] = max(merged[-1][1], hi)
            else:
                merged.append([lo, hi])
        for lo, hi in merged:
            if hi - lo < 0.3:
                continue
            if horiz:
                y = round((wy0 + wy1) / 2, 3)
                out.append([round(lo, 3), y, round(hi, 3), y])
            else:
                x = round((wx0 + wx1) / 2, 3)
                out.append([x, round(lo, 3), x, round(hi, 3)])
    return sorted(out, key=lambda w: (round(w[1], 1), w[0]))


def main():
    pdf = Path(sys.argv[1]) if len(sys.argv) > 1 else PDF
    ps = paths(pdf, SHEET)
    w0 = axis(segments(ps, 0.0))
    xc = (w0[:, [0, 2]].min() + w0[:, [0, 2]].max()) / 2
    yc = (w0[:, [1, 3]].min() + w0[:, [1, 3]].max()) / 2

    def to_m(X, Y):
        return (X - xc) * K, (Y - yc) * K

    lines = np.array([(*to_m(a, b), *to_m(c, d)) for a, b, c, d in w0])
    x0, y0 = lines[:, [0, 2]].min() - 0.3, lines[:, [1, 3]].min() - 0.3
    nx = int((lines[:, [0, 2]].max() + 0.3 - x0) / RES) + 1
    ny = int((lines[:, [1, 3]].max() + 0.3 - y0) / RES) + 1
    wall = wall_raster(lines, x0, y0, nx, ny)
    rects = [(round(x0 + c0 * RES, 3), round(y0 + r0 * RES, 3), round(x0 + c1 * RES, 3), round(y0 + r1 * RES, 3))
             for c0, r0, c1, r1 in rectangles(wall)]
    span = lines[:, [0, 2]].max() - lines[:, [0, 2]].min(), lines[:, [1, 3]].max() - lines[:, [1, 3]].min()
    swings = doors(ps, to_m)
    plan = dict(source=dict(file=pdf.name, sheet="A-1 Proposed floor plan", scale='1/4" = 1\'-0"', job="RG23-044",
                            drawn="5-4-23"),
                frame=dict(m_per_pt=K, origin_pt=[round(xc, 3), round(yc, 3)], x="sheet right (north)",
                           y="sheet up (west)"),
                extent_m=[round(float(span[0]), 3), round(float(span[1]), 3)],
                walls=[list(r) for r in rects], windows=windows(ps, wall, x0, y0, to_m, rects, swings), arcs=swings,
                sliders=SLIDERS)
    (HERE / "home_plan.json").write_text(json.dumps(plan, indent=1))
    im = Image.fromarray(((~wall) * 255).astype(np.uint8)[::-1]).convert("RGB")
    dr = ImageDraw.Draw(im)
    px = lambda x, y: ((x - x0) / RES, ny - 1 - (y - y0) / RES)          # noqa: E731
    for wx0, wy0, wx1, wy1 in plan["windows"]:
        dr.line([px(wx0, wy0), px(wx1, wy1)], fill=(0, 140, 255), width=9)
    for a in plan["arcs"]:
        dr.line([px(*a["hinge"]), px(*a["shut"])], fill=(255, 0, 0), width=5)
        dr.line([px(*a["hinge"]), px(*a["open"])], fill=(0, 170, 0), width=5)
    (HERE / "renders").mkdir(exist_ok=True)
    im.save(HERE / "renders" / "home_plan.png")
    print(f"{len(rects)} wall boxes, {len(plan['windows'])} windows, {len(plan['arcs'])} swings; walls span "
          f"{span[0]:.2f} x {span[1]:.2f} m")


if __name__ == "__main__":
    main()
