"""v10 (design.md 28): how good a SLAM map of the house is, against the true geometry.

A map is an occupancy grid (map_server / slam_toolbox .pgm + .yaml) in the map frame; world_from_map puts it in the
house (the map frame is where the mapping session started). The truth is the house's static geometry cut at the lidar's
scan height and rasterized at 2 cm (estimator.Map2D, the map v9's matcher used). Measured:

  wall error      every occupied map cell's distance from the nearest true surface: median / 95% / max, and the share
                  of cells more than 10 cm off (phantoms: floor hits from a tilted scan plane, smear from skew)
  as anchored     with the map placed where its frame really is (what the robot uses), and
  best fit        after the SE(2) that best overlays it on the truth (removes a global offset / rotation, keeps bends)
  per wall        for each partition wall face: the mean offset of the cells mapped on it (mm), and its tilt
  coverage        share of the true wall faces seen from inside the house that have a mapped cell within 7.5 cm

  python map_quality.py ros/maps/house.yaml [--origin x y yaw_deg]     (origin from ros/maps/<name>_origin.json)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "studies"))

import house  # noqa: E402
from estimator import Map2D  # noqa: E402
from floor import Wall  # noqa: E402


def load_map(yaml_path: Path):
    """(occupied mask [row, col] with row 0 = top, resolution, origin (x, y, yaw))."""
    meta = {}
    for line in yaml_path.read_text().splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    img = meta["image"]
    pgm = Path(img) if Path(img).is_absolute() else yaml_path.parent / img
    data = pgm.read_bytes()
    # P5 header: magic, width, height, maxval (comments allowed)
    toks, i = [], 0
    while len(toks) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        toks.append(data[i:j])
        i = j
    i += 1
    w, h, maxv = int(toks[1]), int(toks[2]), int(toks[3])
    pix = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w).astype(float)
    negate = int(meta.get("negate", "0"))
    occ_th = float(meta.get("occupied_thresh", "0.65"))
    p = pix / maxv if negate else (maxv - pix) / maxv
    origin = [float(x) for x in meta["origin"].strip("[]").split(",")]
    return p > occ_th, float(meta["resolution"]), origin


def truth_map():
    """The house's static geometry at the lidar's scan height (2 cm raster) and the scene's partition walls."""
    drv = house.Driver(house.house_params("v7"))
    m, d = drv.m, drv.d
    z = float(d.site_xpos[m.site("lidar").id][2])
    mp = Map2D(m, d, z, exclude_root=m.body_rootid[m.body("base").id])
    walls = [f for f in house.HOUSE_FEATURES if isinstance(f, Wall)]
    return mp, walls, z


def cells_world(occ, res, origin, world_from_map):
    rows, cols = np.nonzero(occ)
    h = occ.shape[0]
    x = origin[0] + (cols + 0.5) * res
    y = origin[1] + (h - 1 - rows + 0.5) * res
    if len(origin) > 2 and origin[2]:
        c, s = math.cos(origin[2]), math.sin(origin[2])
        x, y = origin[0] + c * (x - origin[0]) - s * (y - origin[1]), origin[1] + s * (x - origin[0]) + c * (y - origin[1])
    ox, oy, oyaw = world_from_map
    c, s = math.cos(oyaw), math.sin(oyaw)
    return np.column_stack([ox + c * x - s * y, oy + s * x + c * y])


def best_fit(mp: Map2D, pts, iters=30, huber=0.03, outlier=0.3):
    """SE(2) (dx, dy, dyaw about the points' centroid) minimizing the robust distance of the cells to the truth."""
    ctr = pts.mean(axis=0)
    q = pts - ctr
    x = np.zeros(3)
    for _ in range(iters):
        c, s = math.cos(x[2]), math.sin(x[2])
        P = np.column_stack([ctr[0] + x[0] + c * q[:, 0] - s * q[:, 1], ctr[1] + x[1] + s * q[:, 0] + c * q[:, 1]])
        dd, gx, gy = mp.lookup(P)
        k = dd < outlier
        w = np.where(dd <= huber, 1.0, huber / np.maximum(dd, 1e-9))[k]
        J = np.column_stack([gx, gy, gx * (-s * q[:, 0] - c * q[:, 1]) + gy * (c * q[:, 0] - s * q[:, 1])])[k]
        H = (J * w[:, None]).T @ J + 1e-6 * np.eye(3)
        step = -np.linalg.solve(H, (J * w[:, None]).T @ dd[k])
        x += step
        if np.linalg.norm(step[:2]) < 1e-5 and abs(step[2]) < 1e-6:
            break
    c, s = math.cos(x[2]), math.sin(x[2])
    P = np.column_stack([ctr[0] + x[0] + c * q[:, 0] - s * q[:, 1], ctr[1] + x[1] + s * q[:, 0] + c * q[:, 1]])
    # as a world transform: rotation about the centroid then the shift
    shift = (ctr + x[:2]) - np.array([c * ctr[0] - s * ctr[1], s * ctr[0] + c * ctr[1]])
    return P, dict(dx=float(shift[0]), dy=float(shift[1]), dyaw_deg=math.degrees(x[2]))


def err_stats(mp: Map2D, P):
    dd = mp.lookup(P)[0]
    return dict(n=int(len(dd)), med=float(np.median(dd)), p95=float(np.percentile(dd, 95)), max=float(dd.max()),
                rms_in=float(np.sqrt(np.mean(dd[dd < 0.10] ** 2))), phantom=float(np.mean(dd > 0.10)))


def coverage(mp: Map2D, P, start_xy, tol=0.075):
    """Share of the true wall-face cells facing the inside of the house that have a mapped cell within tol."""
    occ = mp.occ
    ij = np.floor((np.asarray(start_xy) - mp.origin) / mp.res).astype(int)
    lab, _ = ndimage.label(~occ)
    inside = lab == lab[ij[0], ij[1]]
    face = occ & ndimage.binary_dilation(inside, structure=ndimage.generate_binary_structure(2, 1))
    fi = np.argwhere(face)
    fxy = mp.origin + (fi + 0.5) * mp.res
    # mapped cells -> a distance field on the truth's grid
    hit = np.zeros_like(occ)
    k = np.floor((P - mp.origin) / mp.res).astype(int)
    ok = (k[:, 0] >= 0) & (k[:, 1] >= 0) & (k[:, 0] < occ.shape[0]) & (k[:, 1] < occ.shape[1])
    hit[k[ok, 0], k[ok, 1]] = True
    dmap = ndimage.distance_transform_edt(~hit) * mp.res
    near = dmap[fi[:, 0], fi[:, 1]] <= tol
    return float(near.mean()), fxy, near


def per_wall(walls, P, half=0.05, band=0.2, end=0.1):
    """Mean offset (mm, + = the face mapped proud of the true face) and tilt (deg) of the cells mapped on each face."""
    out = []
    for w in walls:
        a, b = np.array([w.x0, w.y0]), np.array([w.x1, w.y1])
        L = float(np.linalg.norm(b - a))
        u = (b - a) / L
        n = np.array([-u[1], u[0]])
        rel = P - a
        s, dn = rel @ u, rel @ n
        for side in (1, -1):
            k = (s > end) & (s < L - end) & (side * dn > 0) & (side * dn < band)
            if k.sum() < 5:
                continue
            off = side * dn[k] - half
            tilt = math.degrees(math.atan(np.polyfit(s[k], side * dn[k], 1)[0])) if k.sum() > 10 else float("nan")
            out.append(dict(wall=f"({w.x0:+.2f},{w.y0:+.2f})-({w.x1:+.2f},{w.y1:+.2f}) {'+' if side > 0 else '-'}n",
                            n=int(k.sum()), offset_mm=float(1000 * np.mean(off)), tilt_deg=tilt))
    return out


def score(yaml_path, world_from_map, truth=None):
    mp, walls, z = truth or truth_map()
    occ, res, origin = load_map(Path(yaml_path))
    P = cells_world(occ, res, origin, world_from_map)
    out = dict(map=str(yaml_path), resolution=res, world_from_map=list(world_from_map), scan_z=z, cells=int(len(P)))
    out["anchored"] = err_stats(mp, P)
    Pf, fit = best_fit(mp, P)
    out["fit"] = fit
    out["best_fit"] = err_stats(mp, Pf)
    out["coverage_anchored"] = coverage(mp, P, house.START[:2])[0]
    out["coverage_fit"] = coverage(mp, Pf, house.START[:2])[0]
    out["walls_anchored"] = per_wall(walls, P)
    return out


def report(s):
    a, f = s["anchored"], s["best_fit"]
    print(f"map {s['map']}: {s['cells']} occupied cells at {100 * s['resolution']:.1f} cm")
    for name, e, cov in (("as anchored", a, s["coverage_anchored"]), ("best fit   ", f, s["coverage_fit"])):
        print(f"  {name}: wall error {100 * e['med']:.1f} / {100 * e['p95']:.1f} / {100 * e['max']:.1f} cm "
              f"(median / 95% / max), {100 * e['phantom']:.1f}% of cells > 10 cm off, coverage {100 * cov:.0f}%")
    print(f"  best fit moves the map {100 * s['fit']['dx']:+.1f} / {100 * s['fit']['dy']:+.1f} cm, "
          f"{s['fit']['dyaw_deg']:+.2f} deg")
    print("  partition wall faces (as anchored): offset mm, tilt deg")
    for w in s["walls_anchored"]:
        print(f"    {w['wall']:34s} {w['offset_mm']:+6.1f} mm  {w['tilt_deg']:+5.2f} deg  ({w['n']} cells)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_yaml")
    ap.add_argument("--origin", nargs=3, type=float, help="world_from_map: x y yaw_deg (default: <name>_origin.json)")
    ap.add_argument("--out", help="write the score as JSON here")
    a = ap.parse_args()
    y = Path(a.map_yaml)
    if a.origin:
        o = (a.origin[0], a.origin[1], math.radians(a.origin[2]))
    else:
        j = y.with_name(y.stem + "_origin.json")
        o = tuple(json.loads(j.read_text())["world_from_map"]) if j.exists() else \
            (house.START[0], house.START[1], math.radians(house.START[2]))
    s = score(y, o)
    report(s)
    if a.out:
        Path(a.out).write_text(json.dumps(s, indent=1))


if __name__ == "__main__":
    main()
