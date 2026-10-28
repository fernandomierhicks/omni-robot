"""Build the v8 robot (both payloads): STEP + STL of every printed part, the assembly STEP, clash checks over the tilt and
suspension range, every screw's thread engagement, head, clamping and tool access, insertion paths, single-solid
parts, wall thickness, the lidar's self-occlusion, mass and CG against the sim, a BOM down to the fasteners.

  python build.py              # everything (several minutes)
  python build.py --quick      # parts, BOM, mass, engagement, access; clash check at the static pose; no walls
  python build.py --no-export  # skip STEP / STL files
  python build.py --render-only  # only the meshes for render.py and view.py
  python build.py --head-only  # only the head's meshes (out/head_stl, for layout_trade.py) and clearances

Outputs (cad/v8/out/):
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
import re
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

from build123d import Box, Compound, Location, Pos, export_step, export_stl

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
    "battery 7S1P": 612.0, "BMS": 40.0, "power board": 40.0, "MCU carrier": 30.0, "micro speaker": 20.0,
    "5 in display": 110.0, "polycarbonate window": 22.0, "USB mic array": 15.0,
    "on/off switch": 30.0, "ESC 8S": 15.0, "RoboClaw": 50.0, "OV9782 camera": 12.0, "camera bar": 68.0, "standoff rod": 13.3,
    "bearing 61808": 26.0, "bearing 623": 1.8, "608-2RS": 12.0, "RC shock": 18.0, "carbon tube": 4.0,
    "SO-101 arm": 780.0, "shaft 8 x 40": 15.8, "pin 3 x 58": 3.2, "pin 4 x 16": 1.6, "servo horn": 4.0, "IR break-beam": 2.0,
    "M5 prop nut": 1.5, "feed torsion spring": 0.3, "shim": 0.1, "12 V buck": 15.0, "19 V buck": 15.0,
    "5 V buck": 10.0, "fork bar": 59.4, "shaft 8 x 104": 41.0, "mast post": 101.0, "XT30": 3.0, "Micro-Fit": 4.0,
    "USB-C panel": 6.0,
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
    if it.kind in ("ball", "label"):
        return 0.0
    if it.kind == "proxy":
        return purchased_mass(it.key)
    return purchased_mass(it.key)


def fill(infill: int) -> float:
    return min(1.0, infill / 100 + 0.25)


# --- clash checking ---------------------------------------------------------------------------------------------------

INTENDED = (("flywheel_hub", "flywheel_tyre"), ("feed_wheel_hub", "feed_wheel_tyre"), ("roller_core", "roller_tread"),
            ("payload_plate", "pin 4"))            # press fits
SKIP_KINDS = {"fabric", "proxy", "label"}


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


def engagement(items, only=None):
    """Thread each screw engages, from its probe (hw.py): mm in printed parts and in nuts / metal parts. A screw passes
    with >= ENGAGE_MIN in printed parts or >= ENGAGE_METAL in a nut or metal thread."""
    hosts = [(it, _bbox(it.shape)) for it in items
             if (it.kind == "printed" or (it.kind == "purchased" and not it.key.startswith(NOT_THREAD))
                 or (it.kind == "fastener" and "nut" in it.key))]
    out = []
    for it in items:
        if it.probe is None or "SSCS" in it.key or (only and not only(it)):
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


# --- assembly: heads, clamping, tool access, insertion paths, single solids ------------------------------------------

DRIVER_R = {"M2": 1.4, "M2.5": 1.6, "M3": 2.4}       # screwdriver shaft (set screws: a 1.5 mm hex key)
BURIED_MM = 40.0                                    # a screw's own parts blocking its driver this close: it can't be driven
CLR_R = {"M2": 1.25, "M2.5": 1.5, "M3": 1.75}
OUT_R = {"M2": 2.2, "M2.5": 2.6, "M3": 3.0}


def _local_z(shape, loc):
    b = shape.moved(loc.inverse()).bounding_box()
    return round(b.min.Z, 1), round(b.max.Z, 1)


def access(items, only=None):
    """Every screw: what its head bears on, what it clamps (parts between the head and the thread), and a straight
    driver path out of its head. Fails: a head bearing on nothing, a screw that clamps nothing, a driver path blocked
    by the screw's own parts within BURIED_MM (no assembly order can drive it). Blockers further out or by other
    parts are assembly-order constraints (listed in checks.json)."""
    from build123d import Cylinder
    hosts = [(it, _bbox(it.shape)) for it in items if it.kind not in ("fabric", "ball", "foam", "proxy", "label")
             and not (it.kind == "fastener" and not ("nut" in it.key or "washer" in it.key))]
    out = []
    for it in items:
        if it.kind != "fastener" or it.loc is None or not any(k in it.key for k in ("PAN", "FLAT", "SSCS")) or \
                (only and not only(it)):
            continue
        size, _, rest = it.key.partition(" x ")
        length, kind = float(rest.split()[0]), rest.split()[1]
        sr = CLR_R[size]
        if kind == "SSCS":
            head, z_sh = None, 0.0
            driver = Cylinder(0.75, 60, align=hw.CB).moved(Pos(0, 0, 0.2))
        else:
            hd, hh = hw.SCREWS[kind][2][size]
            if kind == "PAN":
                head = Cylinder(hd / 2 + 0.3, 0.6, align=hw.CT).moved(Pos(0, 0, -0.02)) - Cylinder(sr, 2)
                driver, z_sh = Cylinder(DRIVER_R[size], 120, align=hw.CB).moved(Pos(0, 0, hh + 0.3)), 0.0
            else:
                head = Cylinder(hd / 2 + 0.6, hh + 0.5, align=hw.CT).moved(Pos(0, 0, -0.05)) - Cylinder(sr, 5)
                driver, z_sh = Cylinder(DRIVER_R[size], 120, align=hw.CB).moved(Pos(0, 0, 0.3)), -hh
        tube = (Cylinder(OUT_R[size], length + z_sh, align=hw.CT) - Cylinder(sr, length + 2, align=hw.CT).moved(Pos(0, 0, 1))).moved(
            Pos(0, 0, z_sh - 0.05))
        T = it.loc
        tube_w, driver_w = tube.moved(T), driver.moved(T)
        head_w = head.moved(T) if head is not None else None
        sb, db = _bbox(it.shape), _bbox(driver_w)
        on, stack, thread, blockers = [], [], set(), []
        for o, ob in hosts:
            if o is it:
                continue
            if _overlap(sb, ob, 1.5):
                try:
                    if head_w is not None and (head_w & o.shape) is not None and (head_w & o.shape).volume > 0.05:
                        on.append(o.name)
                    t = tube_w & o.shape
                    if t is not None and t.volume > 0.3:
                        stack.append(o.name)
                        pr = it.probe & o.shape if it.probe is not None else None
                        if pr is not None and pr.volume > 0.3:
                            thread.add(o.name)
                except Exception:  # noqa: BLE001
                    pass
            if _overlap(db, ob):
                try:
                    d = driver_w & o.shape
                    if d is not None and d.volume > 0.5:
                        blockers.append((_local_z(d, T)[0], o.name))
                except Exception:  # noqa: BLE001
                    pass
        blockers.sort()
        own = set(on) | set(stack)
        clamped = (set(on) | set(stack)) - thread - {n for n in stack if n.startswith(("bearing", "shim"))}
        rec = dict(screw=it.key, group=it.group, at=[round(v, 1) for v in (T.position.X, T.position.Y, T.position.Z)],
                   head_on=on, thread=sorted(thread), clamps=sorted(clamped),
                   driver_blocked_by=[f"{n} at {z:.0f} mm" for z, n in blockers[:3]])
        problems = []
        if kind != "SSCS":
            if not on:
                problems.append("head bears on nothing")
            if not clamped:
                problems.append("clamps nothing")
            if blockers and blockers[0][0] < BURIED_MM and blockers[0][1] in own:
                problems.append(f"buried: {blockers[0][1]} blocks the driver at {blockers[0][0]:.0f} mm")
        rec["problems"] = problems
        out.append(rec)
    return out


def insertion_paths():
    """Bought parts that slide into printed ones: the DM-2325 into its cup from the inner end (+x, POD frame), the
    STS3250 out of yoke R's window (-y). The worst interference along each path (mm^3)."""
    import mounts as MT
    import vendor as V
    res = {}
    cup, mot = P.motor_cup(), V.dm2325(trim=True).moved(P.MOTOR_LOC)
    worst = 0.0
    for dx in range(1, 66, 2):
        i = mot.moved(Pos(dx, 0, 0)) & cup
        worst = max(worst, i.volume if i is not None else 0.0)
    res["DM-2325 into the motor cup"] = round(worst, 2)
    yr, srv = MT.yoke(-1), V.sts3250().moved(MT.servo_loc())
    worst = 0.0
    for dy in range(0, 46, 3):
        i = srv.moved(Pos(0, -dy, 0)) & yr
        worst = max(worst, i.volume if i is not None else 0.0)
    res["STS3250 through yoke R's window"] = round(worst, 2)
    return res


def ball_path(items, step=4.0):
    """The ball (0.5 mm under size) swept down the chute from its resting spot, round the J and up the launch run:
    printed parts it touches (mm^3), other than the feed wheels and flywheels that grip it. MODULE frame at tilt 0."""
    import math as m
    from build123d import Sphere
    import module as M
    pts = []
    n = max(2, int(m.dist(D.HOLD, M.J0) / step))
    pts += [(D.HOLD[0] + (M.J0[0] - D.HOLD[0]) * k / n, D.HOLD[1] + (M.J0[1] - D.HOLD[1]) * k / n) for k in range(n)]
    arc = M.RHO * (M.TH - M.BE)
    for k in range(max(2, int(arc / step))):
        ph = M.BE + (M.TH - M.BE) * k / max(2, int(arc / step))
        pts.append((M.CJ[0] + M.RHO * m.sin(ph), M.CJ[1] - M.RHO * m.cos(ph)))
    for k in range(int(M.LAUNCH_END / step) + 1):
        pts.append((M.J1[0] + M.U[0] * k * step, M.J1[1] + M.U[1] * k * step))
    ml = A.module_loc(0.0)
    parts = [(it, _bbox(it.shape)) for it in items if it.group == "module" and it.kind == "printed"
             and not it.name.startswith(("feed_wheel", "flywheel"))]
    hits = {}
    for x, z in pts:
        ball = Sphere(D.RB - 0.5).moved(ml * Pos(x, 0, z))
        bb = _bbox(ball)
        for it, ob in parts:
            if _overlap(bb, ob):
                i = ball & it.shape
                if i is not None and i.volume > 0.05:
                    hits[it.name] = max(hits.get(it.name, 0.0), round(i.volume, 2))
    return dict(positions=len(pts), hits=hits)


def index_sweep():
    """The camera bar turned from the catch index to the arm index (the swap: the module and the face off first)
    against everything that stays: clashes at every 10 deg (the index screw out between the stops)."""
    import head as H
    out = {}
    fixed = A.build(0.0, 0.0, "base,pods,mounts", index=-5.0).items     # between the stops: no index screw
    a, b = H.INDEX["catch"], H.INDEX["arm"]
    for k in range(int(round(abs(a - b) / 10)) + 1):
        pitch = a + (b - a) * k / round(abs(a - b) / 10)
        bar = A.build(0.0, 0.0, "head", index=pitch).items
        out[f"index {pitch:+.0f}"] = clashes(fixed + bar, only=lambda x, y: (x.group == "head") != (y.group == "head"))
    return out


HEAD_PARTS = ("cheek", "cam_holder", "camera bar", "face_housing", "face_back", "5 in display", "polycarbonate",
              "crown", "mast", "audio_")          # round 2: the face mast (the module's sweep and the arm see it too)
ARM_KIT_PARTS = ("arm_riser", "arm_adapter", "bin", "12 V buck (arm)")


def _head_items(items):
    return [it for it in items if it.kind != "fastener" and it.name.startswith(HEAD_PARTS)]


def export_head_meshes():
    """out/head_stl/<kit>/<part>.stl: the head's parts in the ROBOT frame (mm), at the catch and the arm index, with
    the face at its seat and (arm) the arm kit, for simulation/v11 sil/studies/layout_trade.py to cast its rays at
    and measure the SO-101's meshes against the drawn geometry. The cameras are left out: the study puts its own
    camera bodies at the CAD's lenses."""
    import head as H
    for name in H.INDEX:
        d = OUT / "head_stl" / name
        d.mkdir(parents=True, exist_ok=True)
        for f in d.glob("*.stl"):
            f.unlink()
        items = A.build(0.0, 0.0, "mounts,head", kit=name).items
        keep = _head_items(items) + ([it for it in items if it.kind != "fastener" and it.name.startswith(ARM_KIT_PARTS)]
                                     if name == "arm" else [])
        for it in keep:
            export_stl(it.shape, str(d / f"{re.sub(r'[^A-Za-z0-9]+', '_', it.name).strip('_')}.stl"), tolerance=0.1,
                       angular_tolerance=0.2)
    log("head meshes for layout_trade.py")


def head_clearance(margin: float = 5.0):
    """Exact distances (OCC) from the head and the face: the module over its tilt range (closest part and tilt), and
    the launch line (the ball's centre from the nip out to 0.6 m, at each launch elevation) less the ball's radius and
    a 5 mm margin: the lowest elevation from which every steeper one clears, and the clearance at 30 deg (the game's
    flattest)."""
    from build123d import Edge, Vector
    head = [(it, _bbox(it.shape)) for it in _head_items(A.build(0.0, 0.0, "mounts,head").items)]
    best = (1e9, None, None, None)
    for tilt in range(int(D.TILT_RANGE[0]), int(D.TILT_RANGE[1]) + 1, 2):
        for m in A.build(float(tilt), 0.0, "module").items:
            if m.kind in SKIP_KINDS or m.kind == "fastener":
                continue
            mb = _bbox(m.shape)
            for h, hb in head:
                if not _overlap(mb, hb, best[0] if best[0] < 60 else 60):
                    continue
                dist = m.shape.distance_to(h.shape)
                if dist < best[0]:
                    best = (dist, tilt, m.name, h.name)
    out = dict(module=dict(min_mm=round(best[0], 1), at_tilt=best[1], module_part=best[2], head_part=best[3]))
    clear = {}
    for el in range(0, 61, 2):
        tilt = math.degrees(D.THETA) - el
        if not D.TILT_RANGE[0] <= tilt <= D.TILT_RANGE[1]:
            continue
        nx, nz = D.module_to_robot_xz(*D.NIP, tilt)
        u = (math.cos(math.radians(el)), math.sin(math.radians(el)))
        line = Edge.make_line(Vector(nx, 0, nz), Vector(nx + 600 * u[0], 0, nz + 600 * u[1]))
        clear[el] = min(line.distance_to(h.shape) for h, _ in head) - D.RB - margin
    lowest = None
    for el in sorted(clear, reverse=True):
        if clear[el] < 0:
            break
        lowest = el
    out["launch"] = dict(lowest_clear_elev_deg=lowest, clear_at_30_mm=round(clear.get(30, float("nan")), 1),
                         by_elevation_mm={str(k): round(v, 1) for k, v in sorted(clear.items())})
    return out


def arm_clearance():
    """The SO-101's own meshes against the arm robot as drawn (out/head_stl/arm), at its fixed poses and grasps, mm:
    simulation/v11 sil/studies/layout_trade.py arm-check (its own process: the v11 sim's modules share names with
    the v7 sim's that dims.py loads)."""
    import os
    import subprocess
    study = D.ROOT / "simulation" / "v11 sil" / "studies"
    r = subprocess.run([sys.executable, "layout_trade.py", "arm-check"], cwd=study, capture_output=True, text=True,
                       env={**os.environ, "OMNI_CAD_OUT": str(OUT)})          # this CAD's meshes, not cad/v7's
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith("{")]
    return json.loads(lines[-1]) if lines else dict(error=(r.stderr or r.stdout)[-400:])


def arm_robot_checks(robot, export: bool, walls_too: bool):
    """The arm robot (kit "arm": design.md 12.3): its own parts' STEP / STL, the arm kit's screws (thread, access),
    every pair of placed solids at its static pose, single solids, walls, mass; out/ARM_KIT.md (what the swap takes
    off and puts on)."""
    arm = A.build(0.0, 0.0, kit="arm")
    new = {n: pp for n, pp in arm.printed.items() if n not in robot.printed}
    if export:
        for name, pp in new.items():
            write_step(pp.shape, OUT / "step" / f"{name}.step")
            bb = pp.shape.bounding_box()
            export_stl(pp.shape.moved(Pos(-bb.center().X, -bb.center().Y, -bb.min.Z)), str(OUT / "stl" / f"{name}.stl"),
                       tolerance=0.02, angular_tolerance=0.1)
    kit = lambda it: it.group == "mounts"                      # noqa: E731  (the kit's and the moved face's)
    eng = engagement(arm.items, only=kit)
    acc = access(arm.items, only=kit)
    mass, pm = masses(arm)
    out = dict(mass=mass, printed=sorted(new), clashes_static=clashes(arm.items),
               engagement=dict(screws=len(eng), short=[e for e in eng if not e["ok"]]),
               access=dict(screws=len(acc), problems=[a for a in acc if a["problems"]]),
               solids={n: dict(solids=len(pp.shape.solids()), valid=pp.shape.is_valid) for n, pp in new.items()
                       if len(pp.shape.solids()) != 1 or not pp.shape.is_valid})
    if walls_too:
        import walls
        out["walls"] = {n: r for n, r in walls.report(new).items() if r["thin"]}
    cat = Counter(it.key for it in robot.items if it.kind in ("purchased", "proxy", "fastener"))
    arm_c = Counter(it.key for it in arm.items if it.kind in ("purchased", "proxy", "fastener"))
    rows = ["# Arm kit (generated by build.py)", "",
            "What the swap from the catch robot to the arm robot takes off and puts on (design.md 12.3). The base, the head, "
            "the face mast (the face, speaker and mics) and the electronics bay stay; the camera bar turns to its arm index; "
            "the kits' harnesses plug into the plate's payload connector panel.", "", f"Arm robot: **{mass['total_kg']:.2f} kg**, CG at {mass['cg_mm']} mm (the SO-101 stowed, est).",
            "", "## Printed parts of the kit", "", "| Part | Qty | Material | Infill | Orientation | Mass each |", "| --- | --- | --- | --- | --- | --- |"]
    for n, pp in sorted(new.items()):
        rows.append(f"| {n} | {pp.qty} | {pp.material} | {pp.infill}% | {pp.orientation} | {pm[n]:.0f} g |")
    rows += ["", "## Purchased parts and fasteners: put on (+) / taken off (-)", "", "| Change | Part |", "| --- | --- |"]
    for k in sorted(set(cat) | set(arm_c)):
        dlt = arm_c[k] - cat[k]
        if dlt:
            rows.append(f"| {dlt:+d} | {k} |")
    rows += ["", "Taken off with the catch kit and not listed above: the module and its prints, the yokes, the servo cap."]
    (OUT / "ARM_KIT.md").write_text("\n".join(rows) + "\n", encoding="utf-8")
    return out


def solids(robot):
    """Printed parts that aren't one valid solid (a feature joined to nothing prints loose)."""
    return {n: dict(solids=len(pp.shape.solids()), valid=pp.shape.is_valid) for n, pp in robot.printed.items()
            if len(pp.shape.solids()) != 1 or not pp.shape.is_valid}


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

def write_step(shape, path):
    """export_step, retried: a new file in the Dropbox folder is sometimes locked for a moment while it syncs."""
    for k in range(4):
        try:
            return export_step(shape, str(path))
        except RuntimeError:
            if k == 3:
                raise
            time.sleep(2.0)


def export_parts(robot):
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    for name, pp in robot.printed.items():
        write_step(pp.shape, OUT / "step" / f"{name}.step")
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
           f'<text x="{cx - 60:.1f}" y="{cy:.1f}" font-size="6">v8 funnel net 1:1 (55 deg cone): print at 100 %, dashed = seam</text>'
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
            export_stl(Compound(shapes[k:k + 6]), str(d / f"{c}__{k // 6}.stl"), tolerance=0.25, angular_tolerance=0.4)
            n += 1
    log("render meshes", n)


def export_render_arm():
    """out/render_stl_arm: the arm robot (the arm kit, the bar at its arm index) with the SO-101 stowed (its meshes, from
    simulation/v11 sil/studies/layout_trade.py arm-stl)."""
    import subprocess
    export_render(A.build(0.0, 0.0, kit="arm"), "render_stl_arm")
    subprocess.run([sys.executable, "layout_trade.py", "arm-stl", str(OUT / "render_stl_arm")],
                   cwd=D.ROOT / "simulation" / "v11 sil" / "studies", check=False, capture_output=True)


def so101_items(pose: str = "stow"):
    """The SO-101 at a pose for the arm robot's STEP: TheRobotStudio's own STEP of every part (vendor/so101), placed
    where the sim puts its mesh (simulation/v11 sil/studies/layout_trade.py arm-poses: the STEP and the mesh share a
    frame). Kind "arm": in the STEP, not in the checks (build.py checks the arm's meshes: arm_clearance)."""
    import subprocess
    import numpy as np
    from OCP.gp import gp_Trsf
    import vendor as V
    r = subprocess.run([sys.executable, "layout_trade.py", "arm-poses", pose], check=True, capture_output=True,
                       text=True, cwd=D.ROOT / "simulation" / "v11 sil" / "studies")
    poses = json.loads([ln for ln in r.stdout.splitlines() if ln.startswith("[")][-1])
    out = []
    for p in poses:
        u, _, vt = np.linalg.svd(np.array(p["R"]))      # exactly orthonormal: a rounded matrix made OCC call the
        g = gp_Trsf()                                    # moved solids invalid
        g.SetValues(*[v for row, t in zip((u @ vt).tolist(), p["t"]) for v in (*row, t)])
        part = V.so101_part(p["mesh"])
        out.append(A.Item(part.label, "arm", "arm", part.moved(Location(g)), part.label))
    return out


def export_assembly(robot):
    """out/assembly.step (the catch robot) and out/assembly_arm.step (the arm robot: the arm kit, the bar at its arm
    index, the SO-101 stowed, its own STEP parts)."""
    res = step_export.write_assembly(robot.items, OUT / "assembly.step", group_label=A.group_label)
    log("assembly.step", res)
    try:
        arm = so101_items()
    except Exception as e:  # noqa: BLE001  (the sim's SO-101 model: `python arm_so101.py fetch` in simulation/v11 sil)
        log("SO-101 skipped:", e)
        arm = []
    res = step_export.write_assembly(A.build(0.0, 0.0, kit="arm").items + arm, OUT / "assembly_arm.step",
                                     name="omni_robot_v8_arm", group_label=A.group_label)
    log("assembly_arm.step", res)


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
        elif it.kind in ("purchased", "proxy"):
            purch[it.key] += 1
        elif it.kind in ("foam", "fabric"):
            mats[it.key] += 1
        else:
            continue
        info[it.key] = (it.desc, it.source)
    lines = ["# Bill of materials, v8 CAD, catch robot (generated by build.py)", "",
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
    if "--head-only" in sys.argv:                   # the head's meshes and clearances (checks.json untouched)
        export_head_meshes()
        res = head_clearance()
        res["arm"] = arm_clearance()
        (OUT / "head_checks.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
        log(res["module"], {k: v for k, v in res["launch"].items() if k != "by_elevation_mm"})
        return
    if "--render-only" in sys.argv:
        robot = A.build(0.0, 0.0)
        export_render(robot)
        # cutaways: one pod without its wheel guard (front-left), the module on its own
        export_render(robot, "render_stl_pod", lambda it: it.group in ("pod0", "pod0u") and not it.name.startswith("wheel_guard"))
        export_render(robot, "render_stl_module", lambda it: it.group in ("module", "mounts", "head") and not it.name.startswith("funnel_net"))
        export_render_arm()
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
        export_render_arm()
        log("view meshes", view.export_meshes(robot))
        export_assembly(robot)
        export_head_meshes()
        log("the SO-101 against the arm robot (its own meshes, fixed poses and grasps)")
        checks["arm"] = arm_clearance()
        log("arm clearance, mm:", checks["arm"])
    log("the head's clearances: the module over its tilt range, the launch line")
    checks["head"] = head_clearance()
    log("head:", checks["head"]["module"], {k: v for k, v in checks["head"]["launch"].items() if k != "by_elevation_mm"})
    log("thread engagement")
    eng = engagement(robot.items)
    bad = [e for e in eng if not e["ok"]]
    checks["engagement"] = dict(screws=len(eng), short=bad, all=eng)
    log(len(eng), "screws,", len(bad), "short of thread")
    for e in bad:
        log("  short:", e["screw"], e["group"], e["at"], "printed", e["printed_mm"], "metal", e["metal_mm"], e["into"])
    checks["solids"] = solids(robot)
    log("printed parts that aren't one valid solid:", checks["solids"] or "none")
    log("assembly access (heads, clamping, drivers)")
    acc = access(robot.items)
    bad_acc = [a for a in acc if a["problems"]]
    checks["access"] = dict(screws=len(acc), problems=bad_acc, all=acc)
    log(len(acc), "screws,", len(bad_acc), "with access / clamping problems")
    for a in bad_acc:
        log("  ", a["screw"], a["group"], a["at"], a["problems"])
    checks["insertion"] = insertion_paths()
    log("insertion paths (worst interference, mm3):", checks["insertion"])
    checks["ball_path"] = ball_path(robot.items)
    log("ball path:", checks["ball_path"]["positions"], "positions, printed parts in the way:", checks["ball_path"]["hits"] or "none")
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
        fixed = A.build(0.0, 0.0, "base,pods,mounts,head").items
        for tilt in (D.TILT_RANGE[0], -20.0, -10.0, 10.0, 15.0, 20.0, 30.0, D.TILT_RANGE[1]):
            mod = A.build(tilt, 0.0, "module").items
            c = clashes(fixed + mod, only=lambda a, b: (a.group == "module") != (b.group == "module"))
            sweep[f"tilt {tilt:+.0f}"] = c
            log("tilt", tilt, len(c), "clashes", [(x["a"], x["b"], x["volume_mm3"]) for x in c][:6])
        for bump in (D.POD_TRAVEL, -3.0):
            rb = A.build(0.0, bump, "base,pods,mounts,head")
            mod = A.build(0.0, 0.0, "module").items
            c = clashes(rb.items + mod, only=lambda a, b: a.group.endswith("u") != b.group.endswith("u"))
            sweep[f"bump {bump:+.1f}"] = c
            log("bump", bump, len(c), "clashes", [(x["a"], x["b"], x["volume_mm3"]) for x in c][:6])
        checks["sweep"] = sweep
        log("camera bar index: catch (+30) -> arm (-40), the face and the module off (the swap)")
        checks["index"] = index_sweep()
        for k, c in checks["index"].items():
            log(k, len(c), "clashes", [(x["a"], x["b"], x["volume_mm3"]) for x in c][:6])
        import walls
        log("wall thickness (ray casting)")
        w = walls.report(robot.printed)
        checks["walls"] = w
        thin = {n: r["thin"] for n, r in w.items() if r["thin"]}
        log(len(thin), f"printed parts with walls under {walls.THIN} mm:")
        for n, cl in thin.items():
            log("  ", n, [(c["t"], c["at"]) for c in cl[:4]])
    log("the arm robot: its kit's parts, screws, clashes, solids")
    checks["arm_robot"] = a = arm_robot_checks(robot, "--no-export" not in sys.argv, not quick)
    log("arm robot:", a["mass"]["total_kg"], "kg;", len(a["clashes_static"]), "clashes;", len(a["engagement"]["short"]),
        "short of", a["engagement"]["screws"], "screws;", len(a["access"]["problems"]), "access problems;",
        "not one solid:", a["solids"] or "none", "; thin walls:", list(a.get("walls", {})) or "none")
    for c in a["clashes_static"]:
        log("  arm robot clash:", c)
    for e in a["engagement"]["short"]:
        log("  arm robot short:", e["screw"], e["at"], e["into"])
    for x in a["access"]["problems"]:
        log("  arm robot access:", x["screw"], x["at"], x["problems"])
    (OUT / "checks.json").write_text(json.dumps(checks, indent=1), encoding="utf-8")
    log("done")


if __name__ == "__main__":
    main()
