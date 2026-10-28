"""Build every part, export STEP + STL, the assembly STEP, a clash check, masses and a print list.

  python build.py            # everything (about a minute)
  python build.py --no-clash # skip the interference check

Outputs (cad/out/):
  step/<part>.step       printed parts in their contract frame (ROBOT / POD / PAYLOAD, see dims.py)
  stl/<part>.stl         print-ready: moved so the part sits at the origin, same orientation
  assembly.step          everything placed in the ROBOT frame, purchased parts included
  render_stl/*.stl       placed meshes for render.py
  parts.json, PRINT_LIST.md
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from build123d import (Align, Box, Compound, Cone, Cylinder, Location, Pos, Rot, Sphere, export_step, export_stl)

import base as B
import catch as C
import dims as D
import vendor as V

OUT = Path(__file__).with_name("out")
DENSITY = {"PETG": 1.27, "PLA": 1.24, "TPU 95A": 1.21}


@dataclass
class Printed:
    name: str
    build: callable
    material: str
    infill: int
    orientation: str
    placements: list           # Locations in the ROBOT frame


def payload(loc: Location = Location()) -> Location:
    return Location((0, 0, D.PLATE_TOP)) * loc


def pods():
    return [B.pod_to_robot(th) for th in D.WHEEL_ANGLES]


def printed_parts() -> list[Printed]:
    one = [Location()]
    parts = [
        Printed("wheel_pod", B.wheel_pod, "PETG", 40, "flange face down", pods()),
        Printed("wheel_adapter", B.wheel_adapter, "PETG", 100, "wheel-side face down (or machine in 6061)", pods()),
        Printed("center_plate", B.center_plate, "PETG", 30, "flat", one),
    ]
    parts += [Printed(f"deck_sector_{k}", (lambda k=k: B.deck_sector(k)), "PETG", 25, "flat, lip side down", one) for k in range(6)]
    parts += [Printed(f"bumper_arc_{k}", (lambda k=k: B.bumper_arc(k)), "TPU 95A", 15, "skirt down", one) for k in range(6)]
    parts += [
        Printed("battery_tray", B.battery_tray, "PETG", 30, "flange down", one),
        Printed("lidar_riser", B.lidar_riser, "PLA", 30, "flat", one),
        Printed("payload_plate", B.payload_plate, "PETG", 30, "flat", [payload()]),
        Printed("launcher_base", C.launcher_base, "PETG", 30, "flat (motor plates overhang 38 deg)", [payload()]),
        Printed("channel_half_L", lambda: C.channel_half(1), "PETG", 30, "outer side wall down", [payload()]),
        Printed("channel_half_R", lambda: C.channel_half(-1), "PETG", 30, "outer side wall down", [payload()]),
        Printed("gate_flap", C.gate_flap, "PETG", 100, "flap face down", [payload()]),
        Printed("servo_tower", C.servo_tower, "PETG", 40, "on its back", [payload()]),
        Printed("flywheel_hub", C.flywheel_hub, "PETG", 100, "motor side down",
                [payload(C.flywheel_frame(s)) for s in (1, -1)]),
        Printed("flywheel_tire", C.flywheel_tire, "TPU 95A", 100, "flat", [payload(C.flywheel_frame(s)) for s in (1, -1)]),
        Printed("funnel_collar", C.funnel_collar, "PETG", 30, "flat", [payload()]),
    ]
    parts += [Printed(f"funnel_rim_arc_{k}", (lambda k=k: C.funnel_rim_arc(k)), "PETG", 30, "flat", [payload()]) for k in range(4)]
    parts += [Printed(f"camera_bracket_{s}", (lambda sd=sd: C.camera_bracket(sd)), "PETG", 40, "base down", [payload()])
              for s, sd in (("L", 1), ("R", -1))]
    return parts


def purchased():
    """(label, shape, location in ROBOT frame) for the assembly and renders."""
    items = []
    motor_in_pod = Location((B.MOTOR_OUT + 14.0, 0, 0), (0, 0, -90))     # STEP output face y=-14 -> pod x = MOTOR_OUT
    for k, loc in enumerate(pods()):
        items.append((f"AK40-10_{k}", V.ak40_10(), loc * motor_in_pod))
        items.append((f"omni_wheel_{k}", V.gobilda_omni_96(), loc))
    items.append(("battery", V.battery_7s1p(), Location((0, 0, D.DECK_Z - 1.0))))
    items.append(("jetson", V.jetson_devkit(), Location((-60, 0, D.DECK_Z + D.DECK_T + 6.0))))
    items.append(("lidar", V.rplidar_c1(), Location((150, 0, D.DECK_Z + D.DECK_T + 8.0))))
    ex, ey = D.polar(185.0, 135.0)
    items.append(("estop", V.estop_22(), Location((ex, ey, D.DECK_Z + D.DECK_T))))
    items.append(("mcu_board", V.board(60, 40, 12, "MCU + IMU board"), Location((95, 0, D.DECK_Z - 20.0))))
    px, py = D.polar(95.0, 120.0)
    items.append(("power_board", V.board(60, 45, 15, "power board (relay, DC-DC)"), Location((px, py, D.DECK_Z - 22.0), (0, 0, 120))))
    for a in D.STANDOFF_ANGLES:
        x, y = D.polar(D.STANDOFF_R, a)
        gap = D.PLATE_TOP - D.PLATE_T - (D.DECK_Z + D.DECK_T)
        items.append((f"standoff_{int(a)}", Cylinder(4.0, gap, align=(Align.CENTER, Align.CENTER, Align.MIN)),
                      Location((x, y, D.DECK_Z + D.DECK_T))))
    for s in (1, -1):
        fw = C.flywheel_frame(s)
        items.append((f"flywheel_motor_{'L' if s > 0 else 'R'}", V.drone_motor_2806(),
                      payload(fw * Pos(0, 0, -(D.FW_W / 2 + 2.0 + D.FM_H)))))
    items.append(("gate_servo", V.servo_270(), payload(C.servo_location())))
    items.append(("esc_4in1", V.esc_4in1(), payload(Location((-60, 70, 5.0)))))
    for k in range(6):
        items.append((f"strut_{k}", Cylinder(4.0, C.strut_length(k), align=(Align.CENTER, Align.CENTER, Align.MIN)),
                      payload(C.strut_location(k))))
    rt, rr = D.FUNNEL_THROAT_R, D.FUNNEL_R
    h = D.RIM_Z - D.THROAT_Z
    net = Cone(rt + 1.0, rr + 1.0, h, align=(Align.CENTER, Align.CENTER, Align.MIN)) - \
        Cone(rt, rr, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
    items.append(("funnel_net", net, payload(Location((0, 0, D.THROAT_Z)))))
    for s in (1, -1):
        items.append((f"camera_{'L' if s > 0 else 'R'}", V.camera_ov9782(), payload(C.camera_location(s))))
    items.append(("ball", Sphere(D.BALL_R), payload(Location((0, 0, D.HOLD_Z)))))
    return items


def place(shape, loc: Location, label: str):
    s = shape.moved(loc)
    s.label = label
    return s


def main():
    do_clash = "--no-clash" not in sys.argv
    for sub in ("step", "stl", "render_stl"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    rows, placed = [], []
    for pp in printed_parts():
        part = pp.build()
        export_step(part, str(OUT / "step" / f"{pp.name}.step"))
        bb = part.bounding_box()
        export_stl(part.moved(Pos(-bb.center().X, -bb.center().Y, -bb.min.Z)), str(OUT / "stl" / f"{pp.name}.stl"))
        fill = min(1.0, pp.infill / 100 + 0.25)
        mass = part.volume / 1000 * DENSITY[pp.material] * fill
        rows.append(dict(name=pp.name, qty=len(pp.placements), material=pp.material, infill=pp.infill,
                         orientation=pp.orientation, volume_cm3=part.volume / 1000, mass_g=mass,
                         size_mm=[round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)],
                         fits_bed=max(bb.size.X, bb.size.Y) <= D.BED or min(bb.size.X, bb.size.Y, bb.size.Z) <= D.BED and
                         sorted([bb.size.X, bb.size.Y, bb.size.Z])[1] <= D.BED))
        for k, loc in enumerate(pp.placements):
            placed.append(("printed", f"{pp.name}" + (f"_{k}" if len(pp.placements) > 1 else ""), place(part, loc, pp.name), pp.material))
        print(f"{pp.name:18s} x{len(pp.placements)}  {pp.material:7s} {part.volume / 1000:7.1f} cm3  ~{mass:5.0f} g each  "
              f"{bb.size.X:5.0f} x {bb.size.Y:5.0f} x {bb.size.Z:5.0f} mm", flush=True)
    for label, shape, loc in purchased():
        placed.append(("purchased", label, place(shape, loc, label), None))

    asm = Compound(children=[s for _, _, s, _ in placed], label="play_catch_robot")
    export_step(asm, str(OUT / "assembly.step"))
    for kind, label, s, mat in placed:
        try:
            export_stl(s, str(OUT / "render_stl" / f"{kind}__{label}.stl"), tolerance=0.2, angular_tolerance=0.3)
        except Exception as e:  # noqa: BLE001
            print("render export failed:", label, e)

    total = sum(r["mass_g"] * r["qty"] for r in rows)
    print(f"printed parts total ~{total / 1000:.2f} kg")
    net = C.funnel_net_pattern(OUT / "funnel_net_pattern.svg")
    print(f"funnel net pattern: outer R {net['outer_radius']:.1f} mm, inner R {net['inner_radius']:.1f} mm, "
          f"sweep {net['sweep_deg']:.1f} deg (1:1 SVG)")
    clashes = clash_check(placed) if do_clash else []
    (OUT / "parts.json").write_text(json.dumps(dict(printed=rows, printed_total_g=total, clashes=clashes), indent=1))
    write_print_list(rows, total, clashes)


INTENDED = {("flywheel_hub", "flywheel_tire")}       # 0.2 mm press fit


def clash_check(placed, min_vol=5.0):
    """Pairwise interference (mm^3) between placed solids, skipping intended press fits."""
    solids = []
    for kind, label, s, _ in placed:
        if label.startswith("AK40-10"):
            loc = s.location
            env = Cylinder(D.AK_D / 2, 39.2, rotation=(90, 0, 0)).moved(Pos(0, 39.2 / 2 - 13.0, 0))
            env = env.moved(loc)
            solids.append((label, env))
        elif label.startswith(("funnel_net", "ball")):
            continue
        else:
            solids.append((label, s))
    out = []
    for i in range(len(solids)):
        for j in range(i + 1, len(solids)):
            (la, a), (lb, b) = solids[i], solids[j]
            if any(la.startswith(x) and lb.startswith(y) for x, y in INTENDED):
                continue
            ba, bb = a.bounding_box(), b.bounding_box()
            if (ba.max.X < bb.min.X or bb.max.X < ba.min.X or ba.max.Y < bb.min.Y or bb.max.Y < ba.min.Y or
                    ba.max.Z < bb.min.Z or bb.max.Z < ba.min.Z):
                continue
            try:
                v = (a & b).volume
            except Exception:  # noqa: BLE001
                continue
            if v > min_vol:
                out.append(dict(a=la, b=lb, volume_mm3=round(v, 1)))
                print(f"CLASH {la} x {lb}: {v:.0f} mm3", flush=True)
    print(f"{len(out)} clashes above {min_vol} mm3")
    return out


def write_print_list(rows, total, clashes):
    lines = ["# Print list (generated by build.py)", "",
             f"Printed parts total about {total / 1000:.2f} kg (volume x density x (infill + 25% for walls)).", "",
             "| Part | Qty | Material | Infill | Orientation | Size (mm) | Est. mass each |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        sx, sy, sz = r["size_mm"]
        lines.append(f"| {r['name']} | {r['qty']} | {r['material']} | {r['infill']}% | {r['orientation']} | "
                     f"{sx:.0f} x {sy:.0f} x {sz:.0f} | {r['mass_g']:.0f} g |")
    lines += ["", f"Interference check: {len(clashes)} clash(es) above 5 mm3."]
    for c in clashes:
        lines.append(f"- {c['a']} x {c['b']}: {c['volume_mm3']} mm3")
    (OUT / "PRINT_LIST.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
