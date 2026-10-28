"""Wall thickness of printed parts, by ray casting on the exact geometry (OCC): from points all over each part's
surface, the distance through the material along the inward normal. Thin skins over holes, thin walls and thin
columns show as small values. build.py reports the clusters under THIN (PETG / PLA; TPU parts are thin on purpose).

    python walls.py [part-name prefix ...]
"""

from __future__ import annotations

import math
import random
import sys
from collections import defaultdict

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cone
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

THIN = 1.2            # mm: three 0.4 mm perimeters
MIN_HIT = 0.08        # closer hits are the ray grazing an edge, not material
SAMPLES = 5000


def _mesh(shape, tol=0.2):
    """(points, triangles) face by face (faces OCC can't mesh are skipped, and 45 deg cones: the chamfers and
    countersinks, whose rays leave through the face beside them within a fraction of a mm)."""
    BRepMesh_IncrementalMesh(shape.wrapped, tol, False, 0.3, True)
    pts, tris = [], []
    ex = TopExp_Explorer(shape.wrapped, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face(ex.Current())
        srf = BRepAdaptor_Surface(f)
        if srf.GetType() == GeomAbs_Cone and abs(abs(srf.Cone().SemiAngle()) - math.pi / 4) < 0.02:
            ex.Next()
            continue
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            tr = loc.Transformation()
            base = len(pts)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(tr)
                pts.append((p.X(), p.Y(), p.Z()))
            rev = f.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                tris.append((base + a - 1, base + c - 1, base + b - 1) if rev else (base + a - 1, base + b - 1, base + c - 1))
        ex.Next()
    return pts, tris


def thickness(shape, n_max=SAMPLES, seed=1):
    """[(point, thickness, area)] for up to n_max surface samples (triangle centroids, shuffled)."""
    P, tris = _mesh(shape)
    samples = []
    for a, b, c in tris:
        pa, pb, pc = P[a], P[b], P[c]
        u = [pb[i] - pa[i] for i in range(3)]
        v = [pc[i] - pa[i] for i in range(3)]
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        a2 = math.sqrt(sum(x * x for x in n))
        if a2 < 1e-9:
            continue
        samples.append((tuple((pa[i] + pb[i] + pc[i]) / 3 for i in range(3)), tuple(x / a2 for x in n), a2 / 2))
    random.Random(seed).shuffle(samples)
    inter = IntCurvesFace_ShapeIntersector()
    inter.Load(shape.wrapped, 1e-4)
    cls = BRepClass3d_SolidClassifier(shape.wrapped)
    out = []
    for cen, n, area in samples[:n_max]:
        cls.Perform(gp_Pnt(*(cen[i] - 0.03 * n[i] for i in range(3))), 1e-5)
        d = tuple(-x for x in n) if cls.State() == TopAbs_IN else n
        inter.Perform(gp_Lin(gp_Pnt(*cen), gp_Dir(*d)), 0.02, 500.0)
        if inter.IsDone():
            ws = [w for w in (inter.WParameter(i) for i in range(1, inter.NbPnt() + 1)) if w > MIN_HIT]
            if ws:
                out.append((cen, min(ws), area))
    return out


def clusters(pts, limit=THIN, cell=12.0, min_n=3):
    """Thin samples grouped in cell-sized boxes (at least min_n each): [{at, n, t}] thinnest first (t = the
    cluster's lower quartile)."""
    g = defaultdict(list)
    for c, t, _ in pts:
        if t < limit:
            g[tuple(int(math.floor(v / cell)) for v in c)].append((c, t))
    res = []
    for lst in g.values():
        if len(lst) < min_n:
            continue
        ts = sorted(t for _, t in lst)
        res.append(dict(at=[round(sum(c[i] for c, _ in lst) / len(lst), 1) for i in range(3)], n=len(lst), t=round(ts[len(ts) // 4], 2)))
    return sorted(res, key=lambda r: r["t"])


def report(printed: dict) -> dict:
    """{part: {material, t_min, thin: [...]}} for a registry of printed parts (assembly.Robot.printed)."""
    out = {}
    for name, pp in sorted(printed.items()):
        pts = thickness(pp.shape)
        out[name] = dict(material=pp.material, t_min=round(min((t for _, t, _ in pts), default=0.0), 2),
                         thin=clusters(pts) if pp.material != "TPU 95A" else [])
    return out


if __name__ == "__main__":
    import assembly as A
    robot = A.build(0.0, 0.0)
    want = sys.argv[1:]
    sel = {n: p for n, p in robot.printed.items() if not want or any(n.startswith(w) for w in want)}
    for name, r in report(sel).items():
        print(f"{name:24s} {r['material']:8s} thinnest {r['t_min']:.2f}  clusters under {THIN}: {len(r['thin'])}")
        for c in r["thin"][:6]:
            print(f"      {c['t']:.2f} mm x{c['n']} at {c['at']}")
