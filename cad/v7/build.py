"""Build the v7 robot: STEP + STL of every printed part, the assembly STEP, clash checks over the tilt and
suspension range, every screw's thread engagement, the lidar's self-occlusion, mass and CG against the sim, a BOM
down to the fasteners.

  python build.py              # everything (several minutes)
  python build.py --quick      # parts, BOM, mass, engagement; clash check at the static pose only
  python build.py --no-export  # skip STEP / STL files
  python build.py --render-only  # only the meshes for render.py and view.py

Outputs (cad/v7/out/):
  step/<part>.step        printed parts in their contract frame (ROBOT / POD / MODULE, see dims.py); mirrored pods'
                          parts as <part>_M
  stl/<part>.stl          the same, moved to sit on z = 0 at the origin (orient per PRINT_LIST.md)
  assembly.step           the robot at tilt 0, static ride height: a nested, coloured assembly with shared parts
                          (step_export.py; --bodies there writes a single multi-body part instead)
  render_stl/, view_stl/  meshes for render.py and view.py
  PRINT_LIST.md, BOM.md, checks.json (clashes, sweep, engagement, lidar, mass)
"""

from __future__ import annotations

import json
import math
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

from build123d import Box, Compound, Pos, export_step, export_stl

import assembly as A
import base as B
import dims as D
import hw
import pod as P
import step_export
import view
from colors import color_class

OUT = Path(__file__).with_name("out")
DENSITY = {"PETG": 1.27, "PLA": 1.24, "TPU 95A": 1.21}
STEEL = 7.9
T0 = time.time()

# purchased masses (g); est where no datasheet value
MASS_G = {
    "ELRS receiver": 3.0,
    "Damiao DM-2325": 172.2, "Pololu 25D": 98.0, "Pololu #1999 hub": 6.5, "Pololu #1997 hub": 3.5,
    "T-Motor F90": 46.6, "Feetech STS3250": 72.0, "RPLIDAR A2M12": 190.0, "Jetson Orin Nano": 250.0,
    "battery 7S1P": 612.0, "BMS": 40.0, "power board": 40.0, "MCU board": 15.0, "speaker": 15.0,
    "e-stop": 45.0, "ESC 8S": 15.0, "RoboClaw": 50.0, "OV9782 camera": 12.0, "stereo bar": 99.0,
    "bearing 61808": 26.0, "bearing 623": 1.8, "608-2RS": 12.0, "RC shock": 18.0, "carbon tube": 4.0,
    "shaft 8 x 40": 15.8, "pin 3 x 58": 3.2, "pin 4 x 16": 1.6, "servo horn": 4.0, "IR break-beam": 2.0,
    "M5 prop nut": 1.5, "feed torsion spring": 0.3, "shim": 0.1, "12 V buck": 15.0, "19 V buck": 15.0,
}
FOAM_DENSITY, NET_G = 0.07, 30.0


def log(*a):
    print(f"[{time.time() - T0:6.1f}s]", *a, flush=True)


def purchased_mass(key: str) -> float:
    for k, v in MASS_G.items():
        if key.startswith(k) or k in key:
            return v
    raise KeyError(key)


def item_mass(it) -> float:
    """g. Printed: volume x density x fill (infill + 25% for walls, as the v2 CAD)."""
    if it.kind == "printed":
        return None                       # filled in from the registry
    if it.kind == "fastener":
        return it.shape.volume / 1000 * STEEL * 1.5         # modelled shank is under the real one
    if it.kind == "foam":
        return it.shape.volume / 1000 * FOAM_DENSITY
    if it.kind == "fabric":
        return NET_G
    if it.kind == "ball":
        return 0.0
    return purchased_mass(it.key)


def fill(infill: int) -> float:
    return min(1.0, infill / 100 + 0.25)


# --- clash checking ---------------------------------------------------------------------------------------------------

INTENDED = (("flywheel_hub", "flywheel_tyre"), ("feed_wheel_hub", "feed_wheel_tyre"), ("roller_core", "roller_tread"),
            ("payload_plate", "pin 4"))            # press fits
SKIP_KINDS = {"fabric"}


def _bbox(s):
    b = s.bounding_box()
    return (b.min.X, b.min.Y, b.min.Z, b.max.X, b.max.Y, b.max.Z)


def _overlap(a, b, pad=0.0):
    return not (a[3] + pad < b[0] or b[3] + pad < a[0] or a[4] + pad < b[1] or b[4] + pad < a[1] or a[5] + pad < b[2] or b[5] + pad < a[2])


def clashes(items, only=None, min_vol=2.0):
    """Pairwise interference (mm^3). only(i_a, i_b) -> bool restricts the pairs checked."""
    live = [(i, it, _bbox(it.shape)) for i, it in enumerate(items) if it.kind not in SKIP_KINDS]
    out = []
    for x in range(len(live)):
        ia, a, ba = live[x]
        for y in range(x + 1, len(live)):
            ib, b, bb = live[y]
            if only and not only(a, b):
                continue
            if not _overlap(ba, bb):
                continue
            if any(a.name.startswith(p) and b.name.startswith(q) or b.name.startswith(p) and a.name.startswith(q)
                   for p, q in INTENDED):
                continue
            try:
                inter = a.shape & b.shape
                v = inter.volume if inter is not None else 0.0
            except Exception as e:  # noqa: BLE001
                v, inter = -1.0, None
            if v > min_vol or v < 0:
                c = None
                if inter is not None and v > 0:
                    bx = inter.bounding_box()
                    c = [round((bx.min.X + bx.max.X) / 2, 1), round((bx.min.Y + bx.max.Y) / 2, 1), round((bx.min.Z + bx.max.Z) / 2, 1)]
                out.append(dict(a=a.name, b=b.name, volume_mm3=round(v, 1), at=c, ga=a.group, gb=b.group))
    return out


# --- thread engagement --------------------------------------------------------------------------------------------------

NOT_THREAD = ("bearing", "608", "shim", "pin ", "shaft", "carbon", "servo horn")   # bores a probe may touch, not threads


def engagement(items):
    """Thread each screw engages, from its probe (hw.py): mm in printed parts and in nuts / metal parts. A screw passes
    with >= ENGAGE_MIN in printed parts or >= ENGAGE_METAL in a nut or metal thread."""
    hosts = [(it, _bbox(it.shape)) for it in items
             if (it.kind == "printed" or (it.kind == "purchased" and not it.key.startswith(NOT_THREAD))
                 or (it.kind == "fastener" and "nut" in it.key))]
    out = []
    for it in items:
        if it.probe is None or "SSCS" in it.key:
            continue
        r0, r1 = hw.PROBE_R[it.size]
        area = math.pi * (r1 * r1 - r0 * r0)
        pb = _bbox(it.probe)
        printed = metal = 0.0
        into = []
        for o, ob in hosts:
            if not _overlap(pb, ob):
                continue
            try:
                v = (it.probe & o.shape).volume
            except Exception:  # noqa: BLE001
                v = 0.0
            if v < 0.05:
                continue
            length = v / area
            if o.kind == "printed":
                printed += length
            else:
                metal += length
            into.append(f"{o.name} {length:.1f}")
        ok = printed >= D.ENGAGE_MIN[it.size] - 0.05 or metal >= D.ENGAGE_METAL[it.size] - 0.05
        c = it.shape.center()
        out.append(dict(screw=it.key, group=it.group, at=[round(c.X, 1), round(c.Y, 1), round(c.Z, 1)],
                        printed_mm=round(printed, 1), metal_mm=round(metal, 1), into=into, ok=ok))
    return out


# --- lidar self-occlusion ---------------------------------------------------------------------------------------------

def lidar_occlusion(items, z_scan: float, centre=(B.LIDAR_XY[0], B.LIDAR_XY[1])):
    """Angular sectors of the scan plane blocked by the robot itself (seen from the lidar's axis)."""
    slab = Box(2000, 2000, 0.4).moved(Pos(0, 0, z_scan))
    hits = []
    for it in items:
        if it.kind in SKIP_KINDS or it.key.startswith("RPLIDAR") or it.name.startswith("lidar_riser"):
            continue
        b = _bbox(it.shape)
        if not (b[2] <= z_scan <= b[5]):
            continue
        try:
            sec = it.shape & slab
        except Exception:  # noqa: BLE001
            continue
        if sec is None or sec.volume < 0.05:
            continue
        angs, dmin = [], 1e9
        pts = [e @ (k / 12) for e in sec.edges() for k in range(13)]      # along the edges (a round section has no corners)
        for v in pts:
            dx, dy = v.X - centre[0], v.Y - centre[1]
            d = math.hypot(dx, dy)
            if d < D.A2M12["d"] / 2:          # inside the lidar's own head: not an obstacle to its beam
                continue
            angs.append(math.degrees(math.atan2(dy, dx)))
            dmin = min(dmin, d)
        if not angs:
            continue
        angs.sort()
        gaps = [(angs[(k + 1) % len(angs)] - angs[k]) % 360 for k in range(len(angs))]
        k = max(range(len(gaps)), key=lambda i: gaps[i])
        start, end = angs[(k + 1) % len(angs)], angs[k]
        hits.append(dict(part=it.name, start=round(start, 1), end=round(end, 1), width=round((end - start) % 360, 1),
                         dist_mm=round(dmin)))
    # merge
    iv = []
    for h in hits:
        s = h["start"] % 360
        iv.append((s, s + h["width"]))
    iv.sort()
    merged = []
    for s, e in iv:
        if merged and s <= merged[-1][1] + 0.2:
            merged[-1][1] = max(merged[-1][1], e)
        else:
            merged.append([s, e])
    if len(merged) > 1 and merged[-1][1] >= 360 + merged[0][0]:
        merged[0][0] = merged[-1][0] - 360
        merged.pop()
    blocked = sum(e - s for s, e in merged)
    return dict(z_scan=z_scan, centre=centre, blocked_deg=round(blocked, 1), sectors=[[round(s, 1), round(e, 1)] for s, e in merged],
                parts=sorted(hits, key=lambda h: h["start"]))


# --- outputs --------------------------------------------------------------------------------------------------------

def export_parts(robot):
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    for name, pp in robot.printed.items():
        export_step(pp.shape, str(OUT / "step" / f"{name}.step"))
        bb = pp.shape.bounding_box()
        export_stl(pp.shape.moved(Pos(-bb.center().X, -bb.center().Y, -bb.min.Z)), str(OUT / "stl" / f"{name}.stl"),
                   tolerance=0.02, angular_tolerance=0.1)
    log("exported", len(robot.printed), "printed parts")


def net_pattern(path: Path) -> dict:
    """1:1 SVG of the 55 deg cone's flat pattern (a ring sector), with 24 + 36 lacing marks and a 10 mm seam allowance."""
    a = D.FUNNEL_ANGLE
    r_in, r_out = D.FUNNEL_THROAT_R / math.sin(a), D.FUNNEL_RIM_R / math.sin(a)
    sweep = 360.0 * math.sin(a)
    pad = 15.0
    size = 2 * (r_out + pad)
    cx = cy = size / 2

    def pt(r, deg):
        return cx + r * math.cos(math.radians(deg)), cy - r * math.sin(math.radians(deg))
    large = 1 if sweep > 180 else 0
    (x0, y0), (x1, y1) = pt(r_out, 0), pt(r_out, sweep)
    (x2, y2), (x3, y3) = pt(r_in, sweep), pt(r_in, 0)
    d = (f"M {x0:.2f} {y0:.2f} A {r_out:.2f} {r_out:.2f} 0 {large} 0 {x1:.2f} {y1:.2f} L {x2:.2f} {y2:.2f} "
         f"A {r_in:.2f} {r_in:.2f} 0 {large} 1 {x3:.2f} {y3:.2f} Z")
    (s0, t0), (s1, t1) = pt(r_out, sweep + 10.0 / r_out * 57.3), pt(r_in, sweep + 10.0 / r_in * 57.3)
    marks = "".join(f'<circle cx="{pt(r_in + 4, sweep * k / 24)[0]:.2f}" cy="{pt(r_in + 4, sweep * k / 24)[1]:.2f}" r="1.2"/>'
                    for k in range(25))
    marks += "".join(f'<circle cx="{pt(r_out - 4, sweep * k / 36)[0]:.2f}" cy="{pt(r_out - 4, sweep * k / 36)[1]:.2f}" r="1.2"/>'
                     for k in range(37))
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{size:.1f}mm" height="{size:.1f}mm" viewBox="0 0 {size:.1f} {size:.1f}">'
           f'<path d="{d}" fill="none" stroke="black" stroke-width="0.3"/>'
           f'<path d="M {x1:.2f} {y1:.2f} L {s0:.2f} {t0:.2f} L {s1:.2f} {t1:.2f} L {x2:.2f} {y2:.2f}" fill="none" '
           f'stroke="black" stroke-width="0.2" stroke-dasharray="2 2"/><g fill="black">{marks}</g>'
           f'<text x="{cx - 60:.1f}" y="{cy:.1f}" font-size="6">v7 funnel net 1:1 (55 deg cone): print at 100 %, dashed = seam</text>'
           f'</svg>')
    path.write_text(svg, encoding="utf-8")
    return dict(inner_r=round(r_in, 1), outer_r=round(r_out, 1), sweep_deg=round(sweep, 1))


def export_pads(robot):
    """The foam pads (MODULE frame at tilt 0 = ROBOT here) as STEP, to print cutting templates or cut by hand."""
    d = OUT / "step"
    for it in robot.items:
        if it.kind == "foam":
            export_step(it.shape, str(d / f"{it.name}.step"))


def export_render(robot, sub="render_stl", keep=lambda it: True):
    d = OUT / sub
    d.mkdir(parents=True, exist_ok=True)
    for f in d.glob("*.stl"):
        f.unlink()
    groups = defaultdict(list)
    for it in robot.items:
        if keep(it):
            groups[color_class(it)].append(it.shape)
    n = 0
    for c, shapes in groups.items():          # chunks: MuJoCo takes at most 200 000 faces per mesh
        for k in range(0, len(shapes), 6):
            export_stl(Compound(children=shapes[k:k + 6]), str(d / f"{c}__{k // 6}.stl"), tolerance=0.25, angular_tolerance=0.4)
            n += 1
    log("render meshes", n)


def export_assembly(robot):
    res = step_export.write_assembly(robot.items, OUT / "assembly.step", group_label=A.group_label)
    log("assembly.step", res)


def masses(robot):
    tot, mom = 0.0, [0.0, 0.0, 0.0]
    by_group = defaultdict(float)
    printed_mass = {}
    for name, pp in robot.printed.items():
        printed_mass[name] = pp.shape.volume / 1000 * DENSITY[pp.material] * fill(pp.infill)
    for it in robot.items:
        m = printed_mass[it.key] if it.kind == "printed" else item_mass(it)
        if m == 0:
            continue
        c = it.shape.center()
        tot += m
        for k, v in enumerate((c.X, c.Y, c.Z)):
            mom[k] += m * v
        by_group[it.group.rstrip("u0123456789") or it.group] += m
    cg = [v / tot for v in mom]
    return dict(total_kg=round(tot / 1000, 3), cg_mm=[round(v, 1) for v in cg], printed_kg=round(sum(
        printed_mass[n] * pp.qty for n, pp in robot.printed.items()) / 1000, 3),
        by_group_kg={k: round(v / 1000, 3) for k, v in by_group.items()}), printed_mass


def sim_mass():
    """The v7 sim's mass and CG (module at tilt 0) for the same configuration."""
    import mujoco
    from robot_model import build_model, mass_properties
    model = build_model(D.SIM)
    mp = mass_properties(model, D.SIM, 0.0)
    return mp


def write_lists(robot, printed_mass, mass, sim):
    rows = ["# Print list (generated by build.py)", "",
            f"{len(robot.printed)} unique printed parts, {sum(pp.qty for pp in robot.printed.values())} prints, "
            f"about {mass['printed_kg']:.2f} kg of filament (volume x density x (infill + 25 %)). `_M` = the mirrored version "
            "(front-left and rear-right pods); print the mirror in the slicer or use the `_M` file.", "",
            "| Part | Qty | Material | Infill | Orientation | Size (mm) | Mass each | Frame |", "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for name, pp in sorted(robot.printed.items()):
        bb = pp.shape.bounding_box()
        sz = sorted([bb.size.X, bb.size.Y, bb.size.Z], reverse=True)
        flag = "" if sz[1] <= D.BED and min(sz) <= D.BED and (sz[0] <= D.BED or sz[2] <= D.BED) else " **TOO BIG?**"
        rows.append(f"| {name} | {pp.qty} | {pp.material} | {pp.infill}% | {pp.orientation} | "
                    f"{bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f}{flag} | {printed_mass[name]:.0f} g | {pp.frame} |")
    (OUT / "PRINT_LIST.md").write_text("\n".join(rows) + "\n", encoding="utf-8")

    purch, fast, mats = Counter(), Counter(), Counter()
    info = {}
    for it in robot.items:
        if it.kind == "fastener":
            fast[it.key] += 1
        elif it.kind in ("purchased",):
            purch[it.key] += 1
        elif it.kind in ("foam", "fabric"):
            mats[it.key] += 1
        else:
            continue
        info[it.key] = (it.desc, it.source)
    lines = ["# Bill of materials, v7 CAD (generated by build.py)", "",
             "Everything placed in `assembly.step`, counted from the model. Prices and part numbers: section 24 and the "
             "actuator database. McMaster numbers are product families (their site can't be read from here): pick the size given.", "",
             "## Purchased parts", "", "| Qty | Part | Description | Source |", "| --- | --- | --- | --- |"]
    for k, n in sorted(purch.items()):
        d, s = info[k]
        lines.append(f"| {n} | {k} | {d} | {s} |")
    lines += ["", "## Fasteners", "",
              "Thread-forming into 2.8 mm printed holes unless a nut is listed. Totals include the module, base and pods.", "",
              "| Qty | Size | Description | McMaster family |", "| --- | --- | --- | --- |"]

    def fkey(k):
        size, _, rest = k.partition(" x ")
        try:
            return (size, rest.split()[1] if " " in rest else "", float(rest.split()[0]))
        except (ValueError, IndexError):
            return (size, k, 0.0)
    for k, n in sorted(fast.items(), key=lambda kv: fkey(kv[0])):
        d, s = info[k]
        lines.append(f"| {n} | {k} | {d} | {s} |")
    lines += ["", f"Total fasteners: {sum(fast.values())} (M3: {sum(n for k, n in fast.items() if k.startswith('M3 '))}).",
              "", "## Materials", "", "| Qty | Item | Description | Source |", "| --- | --- | --- | --- |"]
    for k, n in sorted(mats.items()):
        d, s = info[k]
        lines.append(f"| {n} | {k} | {d} | {s} |")
    lines += ["", "## Mass", "", f"CAD estimate: **{mass['total_kg']:.2f} kg**, CG at {mass['cg_mm']} mm (ROBOT frame, module at "
              f"tilt 0). Printed parts {mass['printed_kg']:.2f} kg.", "",
              f"The v7 sim (DM-2325): {sim['mass']:.2f} kg, CG at "
              f"{[round(v * 1000, 1) for v in sim['cg']]} mm." if sim else "", "",
              "By group: " + ", ".join(f"{k} {v:.2f} kg" for k, v in sorted(mass["by_group_kg"].items()))]
    (OUT / "BOM.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return dict(purchased=dict(purch), fasteners=dict(fast), n_fasteners=sum(fast.values()))


def main():
    quick = "--quick" in sys.argv
    OUT.mkdir(exist_ok=True)
    if "--render-only" in sys.argv:
        robot = A.build(0.0, 0.0)
        export_render(robot)
        # cutaways: one pod without its wheel guard (front-left), the module on its own
        export_render(robot, "render_stl_pod", lambda it: it.group in ("pod0", "pod0u") and not it.name.startswith("wheel_guard"))
        export_render(robot, "render_stl_module", lambda it: it.group in ("module", "mounts") and not it.name.startswith("funnel_net"))
        log("view meshes", view.export_meshes(robot))
        return
    log("building the robot (tilt 0, static)")
    robot = A.build(0.0, 0.0)
    log(len(robot.items), "items,", len(robot.printed), "unique printed parts")
    checks = {}
    mass, printed_mass = masses(robot)
    try:
        mp = sim_mass()
        sim = dict(mass=float(mp[0]), cg=[float(v) for v in mp[1]])
    except Exception as e:  # noqa: BLE001
        log("sim mass failed:", e)
        sim = None
    checks["mass"] = dict(cad=mass, sim=sim)
    log("mass", mass, "sim", sim)
    checks["bom"] = write_lists(robot, printed_mass, mass, sim)
    st = P.shock_travel()
    checks["suspension"] = {k: round(v, 2) for k, v in st.items()}
    log("shock", checks["suspension"])
    checks["net_pattern"] = net_pattern(OUT / "funnel_net_pattern.svg")
    if "--no-export" not in sys.argv:
        export_parts(robot)
        export_pads(robot)
        export_render(robot)
        log("view meshes", view.export_meshes(robot))
        export_assembly(robot)
    log("thread engagement")
    eng = engagement(robot.items)
    bad = [e for e in eng if not e["ok"]]
    checks["engagement"] = dict(screws=len(eng), short=bad, all=eng)
    log(len(eng), "screws,", len(bad), "short of thread")
    for e in bad:
        log("  short:", e["screw"], e["group"], e["at"], "printed", e["printed_mm"], "metal", e["metal_mm"], e["into"])
    log("clash check, static pose")
    checks["clashes_static"] = clashes(robot.items)
    log(len(checks["clashes_static"]), "clashes")
    for c in checks["clashes_static"]:
        log("  clash:", c)
    checks["lidar"] = lidar_occlusion(robot.items, D.LIDAR_SCAN_Z)
    log("lidar blocked", checks["lidar"]["blocked_deg"], "deg in", len(checks["lidar"]["sectors"]), "sectors:",
        sorted({p["part"].split("#")[0] for p in checks["lidar"]["parts"]}))
    if not quick:
        sweep = {}
        fixed = A.build(0.0, 0.0, "base,pods,mounts").items
        for tilt in (D.TILT_RANGE[0], -20.0, -10.0, 10.0, 15.0, 20.0, 30.0, D.TILT_RANGE[1]):
            mod = A.build(tilt, 0.0, "module").items
            c = clashes(fixed + mod, only=lambda a, b: (a.group == "module") != (b.group == "module"))
            sweep[f"tilt {tilt:+.0f}"] = c
            log("tilt", tilt, len(c), "clashes", [(x["a"], x["b"], x["volume_mm3"]) for x in c][:6])
        for bump in (D.POD_TRAVEL, -3.0):
            rb = A.build(0.0, bump, "base,pods,mounts")
            mod = A.build(0.0, 0.0, "module").items
            c = clashes(rb.items + mod, only=lambda a, b: a.group.endswith("u") != b.group.endswith("u"))
            sweep[f"bump {bump:+.1f}"] = c
            log("bump", bump, len(c), "clashes", [(x["a"], x["b"], x["volume_mm3"]) for x in c][:6])
        checks["sweep"] = sweep
    (OUT / "checks.json").write_text(json.dumps(checks, indent=1), encoding="utf-8")
    log("done")


if __name__ == "__main__":
    main()
