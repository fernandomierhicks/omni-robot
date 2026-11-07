"""Tables from the motor study's results (drive_char.py, drive_mission.py): results/motor_char/REPORT.md.

  python drive_char_report.py            # every part that has results
Each table row is one robot (motor, mass, surface, charge); each boundary carries its binding cause (drive_char.classify).
Costs: 4 drives + their controllers at the catalog's price.
"""

from __future__ import annotations

import collections
import math
import sys

import drive_char as DC  # noqa: E402  (first: its common import puts the sim on the path)
from catalog import DRIVE_MOTORS  # noqa: E402

OUT = DC.OUT


def rows(part):
    out = []
    for r in DC.load(part):
        if r.get("err") or r.get("res") is None:
            continue
        out.append(r)
    return out


def cfg_name(c: dict) -> str:
    s = f"{c['motor']} {c['mass']} kg {c['surface']}"
    if c.get("soc", 1.0) != 1.0:
        s += f" {int(100 * c['soc'])}%"
    for k, lab in (("pod_rate", "k"), ("pod_zeta", "z"), ("pod_preload", "pre"), ("pivot_z", "pivot"),
                   ("loop", "loop"), ("bleed", "bleed")):
        v = c.get(k)
        if v not in (None, False) and not (k == "loop" and v == 1.0):
            s += f" {lab}={v}"
    if c.get("springs") == "fixed" and c.get("pod_rate") is None:
        s += " fixed springs"
    return s


def price(motor: str) -> float:
    m = DRIVE_MOTORS.get(motor.split("@")[0])
    return 4 * m.price if m else math.nan


def fmt(x, nd=2):
    if x is None:
        return "-"
    if isinstance(x, float):
        return f"{x:.{nd}f}"
    return str(x)


def table(head, body) -> str:
    L = ["| " + " | ".join(head) + " |", "| " + " | ".join("---" for _ in head) + " |"]
    L += ["| " + " | ".join(fmt(c) for c in r) + " |" for r in body]
    return "\n".join(L)


def by_cfg(rs):
    g = collections.defaultdict(list)
    for r in rs:
        g[cfg_name(r["cfg"])].append(r)
    return g


def part_C(rs):
    out = ["## C. Speed, cruise, straights"]
    g = by_cfg([r for r in rs if r["kind"] == "topspeed"])
    body = []
    for name, xs in sorted(g.items()):
        v = {r["args"][0]: r["res"] for r in xs}
        body.append([name, f"${price(xs[0]['cfg']['motor']):.0f}"] +
                    [f"{v[d]['v_top']:.2f} {v[d]['cause']}" if d in v else "-" for d in DC.DIRS9])
    if body:
        out += ["### Top speed (m/s, cause) per body direction", table(["robot", "4 drives"] + [f"{d:g} deg" for d in DC.DIRS9], body)]
    g = by_cfg([r for r in rs if r["kind"] == "cruise"])
    body = []
    for name, xs in sorted(g.items()):
        for dr in (0.0, 45.0):
            cells = []
            for v in DC.CRUISE_V:
                x = next((r["res"] for r in xs if r["args"] == [dr, v]), None)
                cells.append("-" if x is None else (f"{x['p_drive_W']:.0f} W" if x.get("reached") else f"no ({x['cause']})"))
            body.append([name, f"{dr:g}"] + cells)
    if body:
        out += ["### Steady cruise: drive power (4 drives, W)", table(["robot", "dir"] + [f"{v} m/s" for v in DC.CRUISE_V], body)]
    g = by_cfg([r for r in rs if r["kind"] == "straight"])
    body = []
    for name, xs in sorted(g.items()):
        for L in DC.STRAIGHT_L:
            best = {}
            for r in xs:
                _, LL, cap, a = r["args"]
                if LL != L:
                    continue
                t = r["res"]["t_settle"]
                key = cap if cap is not None else math.inf
                if t is not None and (key not in best or t < best[key][0]):
                    best[key] = (t, a)
            caps = sorted(best, key=lambda c: c if c is not None else math.inf)
            body.append([name, f"{L:g}"] + [f"{best[c][0]:.2f} (a {best[c][1]:g})" if c in best else "-"
                                            for c in [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, None]])
    if body:
        out += ["### Rest-to-rest straights: fastest settle time (s) per speed cap (best acceleration)",
                table(["robot", "L m", "1", "1.5", "2", "2.5", "3", "3.5", "none"], body)]
    return out


def part_A(rs):
    out = ["## A. Acceleration and braking"]
    g = by_cfg([r for r in rs if r["kind"] == "accel"])
    for case in DC.ACCEL_CASES:
        body = []
        for name, xs in sorted(g.items()):
            cells = []
            for d in DC.DIRS9:
                x = next((r["res"] for r in xs if r["args"] == [d, case]), None)
                cells.append("-" if x is None else f"{x['a_max']:.1f} {x['cause']}")
            body.append([name] + cells)
        if body:
            out += [f"### {case} {DC.ACCEL_CASES[case]} m/s: clean acceleration (m/s^2, cause at the first failure)",
                    table(["robot"] + [f"{d:g}" for d in DC.DIRS9], body)]
    for kind, keys in (("reverse", ("t_rev", "t_ideal", "ok", "cause", "tilt_max", "lift_ms")),
                       ("turn", ("t_turn", "t_ideal", "ok", "cause", "track_max", "lift_ms")),
                       ("panic", ("stop_m", "tilt_max", "lift_ms", "flipped", "cause"))):
        body = []
        for name, xs in sorted(by_cfg([r for r in rs if r["kind"] == kind]).items()):
            for r in sorted(xs, key=lambda r: r["args"]):
                body.append([name, " ".join(fmt(a, 1) for a in r["args"])] + [r["res"].get(k) for k in keys])
        if body:
            out += [f"### {kind}", table(["robot", "args"] + list(keys), body)]
    return out


def part_R(rs):
    out = ["## R. Rotation"]
    body = [[cfg_name(r["cfg"]), r["args"][0], r["res"]["w_top"], r["res"]["t95"], r["res"]["cause"]]
            for r in rs if r["kind"] == "spin"]
    if body:
        out += ["### Spin in place", table(["robot", "yaw accel", "top rad/s", "t95 s", "cause"], sorted(body))]
    g = by_cfg([r for r in rs if r["kind"] == "combo"])
    for name, xs in sorted(g.items()):
        vs = sorted({r["args"][0] for r in xs})
        ws = sorted({r["args"][1] for r in xs})
        body = []
        for v in vs:
            cells = []
            for w in ws:
                x = next((r["res"] for r in xs if r["args"] == [v, w]), None)
                cells.append("-" if x is None else ("ok" if x["ok"] else x["cause"]))
            body.append([f"{v} m/s"] + cells)
        out += [f"### Translate while turning: {name}", table(["v \\ w"] + [f"{w} rad/s" for w in ws], body)]
    return out


def part_S(rs):
    out = ["## S. Steps and bumps (tallest crossed, mm; the first failure's cause)"]
    def mu(r):
        return r["args"][4] if len(r["args"]) > 4 else 0.85

    for kind, rest, emu in (("step", False, 0.85), ("bump", False, 0.85), ("step", True, 0.85), ("step", False, 0.5)):
        g = by_cfg([r for r in rs if r["kind"] == "ladder" and r["args"][0] == kind and
                    (len(r["args"]) > 3 and r["args"][3]) == rest and mu(r) == emu])
        if not g:
            continue
        speeds = sorted({r["args"][2] for xs in g.values() for r in xs})
        body = []
        for name, xs in sorted(g.items()):
            for v in speeds:
                cells = []
                for d in DC.DIRS13:
                    x = next((r["res"] for r in xs if r["args"][1] == d and r["args"][2] == v), None)
                    ff = x and x["first_fail"]
                    cells.append("-" if x is None else f"{x['max_mm']:g}" + (f" {ff['cause']}" if ff else ""))
                body.append([name, v] + cells)
        lab = "restart from rest at the edge" if rest else f"{kind}s" + (" (slick edge, mu 0.5)" if emu != 0.85 else "")
        out += [f"### {lab}", table(["robot", "m/s"] + [f"{d:g}" for d in DC.DIRS13], body)]
    return out


def part_F(rs):
    out = ["## F. House transitions (crossed / runs, the failures' causes)"]
    g = collections.defaultdict(list)
    for r in rs:
        if r["kind"] == "house":
            g[(cfg_name(r["cfg"]), r["args"][0], r["args"][2])].append(r["res"])
    body = []
    for (name, ob, v), xs in sorted(g.items()):
        n_ok = sum(1 for x in xs if x["crossed"])
        causes = collections.Counter(x["cause"] for x in xs if not x["crossed"])
        body.append([name, ob, v, f"{n_ok}/{len(xs)}", ", ".join(f"{k} {n}" for k, n in causes.most_common()),
                     max(x["tilt_max"] for x in xs)])
    if body:
        out.append(table(["robot", "obstacle", "m/s", "crossed", "failures", "max tilt"], body))
    return out


def part_I(rs):
    out = ["## I. Slopes (steepest passed, deg; the first failure's cause) and ramps"]
    g = by_cfg([r for r in rs if r["kind"] == "slope"])
    body = []
    for name, xs in sorted(g.items()):
        for t in DC.SLOPE_TESTS:
            cells = []
            for d in DC.SLOPE_DIRS:
                x = next((r["res"] for r in xs if r["args"] == [d, t]), None)
                ff = x and x["first_fail"]
                cells.append("-" if x is None else f"{x['max_deg']:g}" + (f" {ff['cause']}" if ff else ""))
            body.append([name, t] + cells)
    if body:
        out.append(table(["robot", "test"] + [f"{d:g} deg" for d in DC.SLOPE_DIRS], body))
    g = by_cfg([r for r in rs if r["kind"] == "ramp"])
    body = []
    for name, xs in sorted(g.items()):
        for ang in DC.RAMP_ANGLES:
            cells = []
            for d in DC.SLOPE_DIRS:
                ys = [r["res"] for r in xs if r["args"][0] == ang and r["args"][1] == d]
                cells.append("-" if not ys else " ".join("ok" if y["crossed"] else y["cause"] for y in ys))
            body.append([name, ang] + cells)
    if body:
        out += ["### Ramps 120 mm up, 1 m top (0.5 / 1 / 2 m/s)", table(["robot", "deg"] + [f"{d:g}" for d in DC.SLOPE_DIRS], body)]
    return out


def part_U(rs):
    out = ["## U. Uneven loading"]
    for kind, keys in (("side_bump", ("crossed", "yaw_kick_deg", "lateral_m", "lift_ms", "load_min", "bottom_ms", "cause")),
                       ("wheel_offset", ("ok", "static_loads", "slip_ms", "lift_ms", "load_min", "cause"))):
        body = []
        for r in sorted((r for r in rs if r["kind"] == kind), key=lambda r: (cfg_name(r["cfg"]), r["args"])):
            res = r["res"]
            vals = [([round(x, 1) for x in res[k]] if isinstance(res.get(k), list) else res.get(k)) for k in keys]
            body.append([cfg_name(r["cfg"]), " ".join(fmt(a, 1) for a in r["args"])] + vals)
        if body:
            out += [f"### {kind}", table(["robot", "args"] + list(keys), body)]
    return out


def part_T(rs):
    out = ["## T. Thermal (60 s duty unless noted; winding peak, time to the derate's onset)"]
    body = []
    for r in sorted((r for r in rs if r["kind"] == "thermal"), key=lambda r: (cfg_name(r["cfg"]), r["args"])):
        x = r["res"]
        body.append([cfg_name(r["cfg"]), r["args"][0], r["args"][1], x["tw_max"], x["t_derate"], x["p_loss_end"],
                     x["tw_ss_est"], x["cause"]])
    if body:
        out.append(table(["robot", "scenario", "level", "winding max C", "derate at s", "loss W/motor", "steady C (est)",
                          "cause"], body))
    return out


def part_P(rs):
    out = ["## P. Suspension (per setting: scenarios passed, worst tilt, wheel-lift ms, bottomed ms, accel m/s^2)"]
    g = by_cfg([r for r in rs if r["kind"] == "susp"])
    body = []
    for name, xs in sorted(g.items()):
        # crossings only (steps, rug, side bumps, bumpy cruise): the accel rows carry a_max and the panic stop has no
        # pass/fail, both reported in their own columns
        cross = [r for r in xs if not r["args"][0].startswith(("accel", "panic"))]
        ok = sum(1 for r in cross if r["res"].get("ok", r["res"].get("crossed", False)))
        xs_n = len(cross)
        acc = {r["args"][0]: r["res"].get("a_max") for r in xs if "a_max" in r["res"]}
        tilt = max((r["res"].get("tilt_max") or 0) for r in xs)
        lift = max((r["res"].get("lift_ms") or 0) for r in xs)
        bot = sum((r["res"].get("bottom_ms") or 0) for r in xs)
        body.append([name, f"{ok}/{xs_n}", tilt, lift, bot] + [acc.get(k) for k in
                                                                 ("accel start 0", "accel start 90", "accel brake 0")])
    if body:
        out.append(table(["setting", "passed", "max tilt", "lift ms", "bottomed ms", "start 0", "start 90", "brake 0"],
                         body))
    return out


def part_M(path_part):
    rs = [r for r in DC.load(path_part) if not r.get("err")]
    if not rs:
        return []
    out = [f"## {path_part}. Missions through the firmware (drive_mission.py)"]
    body = []
    for r in sorted(rs, key=lambda r: r["args"]):
        x = r["res"]
        body.append([" ".join(str(a) for a in r["args"]), x["time_s"], x["n_done"], x["stuck"], x["v_peak"], x["wh_total"],
                     x["wh_drive"], x["tw_max"], x["tilt_max"], x["stop"], x["cause"]])
    out.append(table(["motor mass profile route assist", "time s", "legs", "stuck", "v peak", "Wh total", "Wh drive",
                      "winding C", "max tilt", "stop", "cause"], body))
    return out


def main(parts=None):
    parts = parts or sys.argv[1:] or list("CARSFIUTP") + ["M", "E"]
    md = ["# Motor study results (generated by drive_char_report.py; see the study README for the method)", ""]
    for p in parts:
        if p in ("M", "E"):
            md += part_M(p) + [""]
            continue
        rs = rows(p)
        if not rs:
            continue
        md += globals()[f"part_{p}"](rs) + [""]
    (OUT / "REPORT.md").write_text("\n\n".join(md), encoding="utf-8")
    print(f"wrote {OUT / 'REPORT.md'}")


if __name__ == "__main__":
    main()
