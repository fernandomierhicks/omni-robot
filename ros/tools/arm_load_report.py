"""Per-phase numbers for a robot computer in the loop: pi_keepup.py --raw's samples cut by arm_load.py's phase times.

  python arm_load_report.py PI_ARM.json ARM_LOAD.json [--json OUT]

Per phase (MANUAL, TIDY, PATROL): the computer's CPU (all cores, 100 = all busy; the busiest core), each process
group's % of one core (cpu_watch.py's groups), the topic rates seen there, and what the phase achieved. Both files'
times are wall clocks: the two computers' clocks must agree (clock_offset.py) to well under a phase.
"""

from __future__ import annotations

import argparse
import json
import statistics as st
from pathlib import Path

OTHER = ("pi_keepup.py", "sshd:", "avahi-daemon:", "systemd-journald", "ros2 launch/daemon", "polkitd", "fwupd",
         "systemd-resolved", "systemd-timesyncd", "NetworkManager", "wpa_supplicant", "irqbalance", "snapd",
         "unattended-upgr", "cron", "cpu_watch.py", "ssh")


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p / 100 * len(xs)))] if xs else None


def phase_numbers(pi: dict, t0: float, t1: float) -> dict:
    host = [r for r in pi.get("host_rows", []) if t0 <= r["t"] <= t1]
    wins = [w for w in pi.get("cpu_windows", []) if t0 <= w["t"] <= t1]
    out = {"seconds": round(t1 - t0), "host_samples": len(host), "cpu_windows": len(wins)}
    if host:
        tot = [r["cpu_total"] for r in host]
        busy = [max(r["cpu_cores"]) for r in host]
        out["cpu_total_pct"] = {"median": round(st.median(tot), 1), "p95": round(q(tot, 95), 1), "max": round(max(tot), 1)}
        out["busiest_core_pct"] = {"median": round(st.median(busy), 1), "p95": round(q(busy, 95), 1)}
        out["cores_used_median"] = round(st.median(tot) / 100 * len(host[0]["cpu_cores"]), 2)
        out["mem_avail_mb_min"] = round(min(r["mem_avail_mb"] for r in host))
        out["swap_mb_max"] = round(max(r["swap_used_mb"] for r in host))
        out["temp_c_max"] = max((r["temp_c"] or 0) for r in host)
        out["freq_mhz_min"] = min(r["freq_mhz"] for r in host)
        thr = 0
        for r in host:
            thr |= r.get("throttled") or 0
        out["throttled_hex"] = hex(thr)
    if wins:
        groups = sorted({g for w in wins for g in w["cpu"]})
        per = {g: round(st.median([w["cpu"].get(g, 0.0) for w in wins]), 1) for g in groups}
        out["groups_pct_of_one_core"] = dict(sorted(((g, v) for g, v in per.items() if v >= 0.5), key=lambda kv: -kv[1]))
        out["robot_total_pct_of_one_core"] = round(st.median(
            [sum(v for g, v in w["cpu"].items() if g not in OTHER) for w in wins]), 1)
        out["rss_mb"] = {g: max(w["rss_mb"].get(g, 0) for w in wins) for g in groups if per.get(g, 0) >= 0.5}
    rates = {}
    for n, ts in (pi.get("topic_times") or {}).items():
        ts = [t for t in ts if t0 <= t <= t1]
        if len(ts) >= 3:
            gaps = sorted(1e3 * (b - a) for a, b in zip(ts, ts[1:]))
            rates[n] = {"hz": round((len(ts) - 1) / (ts[-1] - ts[0]), 1), "gap_p99_ms": round(q(gaps, 99)),
                        "gap_max_ms": round(gaps[-1])}
        else:
            rates[n] = {"hz": 0.0, "messages": len(ts)}
    out["topics"] = rates
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("pi")
    ap.add_argument("load")
    ap.add_argument("--json")
    a = ap.parse_args()
    pi = json.loads(Path(a.pi).read_text())
    load = json.loads(Path(a.load).read_text())
    rep = {"label": pi.get("label"), "model": pi.get("model"), "cores": pi.get("cores"), "phases": {}}
    for ph, d in load["phases"].items():
        r = phase_numbers(pi, d["start"]["t"], d["end"]["t"])
        r["outcome"] = {"command": d.get("command"), "start": {k: d["start"][k] for k in ("mission", "patrol")},
                        "end": {k: d["end"][k] for k in ("mission", "patrol")},
                        "arm_start": d["start"]["arm"], "arm_end": d["end"]["arm"]}
        if "goals" in d:                     # ROUTE: the scorecard
            r["route"] = {k: d[k] for k in ("reached", "n", "mean_err_m", "vmax", "time_s")}
        rep["phases"][ph] = r
        for leg in d.get("legs", []):        # NAVTIDY: the drive and the tidy of each leg cut apart
            if "t_drive" in leg:
                for part, t0, t1 in (("drive", leg["t_drive"], leg["t_tidy"]), ("tidy", leg["t_tidy"], leg["t_end"])):
                    p = phase_numbers(pi, t0, t1)
                    p["outcome"] = {"command": None, "start": {"mission": None, "patrol": None},
                                    "end": {"mission": None, "patrol": None}, "arm_start": {}, "arm_end": {}}
                    p["leg"] = {k: leg[k] for k in ("drive", "tidy", "bin_before", "bin_after")} if part == "tidy" \
                        else {"drive": leg["drive"]}
                    rep["phases"][f"{ph}{leg['leg']}_{part}"] = p
    if a.json:
        Path(a.json).write_text(json.dumps(rep, indent=1))
    print(f"{rep['label']}: {rep['model']}, {rep['cores']} cores")
    for ph, r in rep["phases"].items():
        c = r.get("cpu_total_pct", {})
        print(f"== {ph}: CPU {c.get('median')}% median / p95 {c.get('p95')}% of all cores "
              f"(~{r.get('cores_used_median')} cores), busiest core p95 {r.get('busiest_core_pct', {}).get('p95')}%, "
              f"robot {r.get('robot_total_pct_of_one_core')}% of one core; mem min {r.get('mem_avail_mb_min')} MB, "
              f"swap {r.get('swap_mb_max')} MB, {r.get('temp_c_max')} C, throttled {r.get('throttled_hex')}")
        for g, v in list(r.get("groups_pct_of_one_core", {}).items())[:16]:
            print(f"     {g:36s} {v:6.1f}   {r['rss_mb'].get(g, 0):5d} MB")
        print("   topics: " + ", ".join(f"{n} {s['hz']} Hz" + (f" (gap max {s['gap_max_ms']} ms)" if "gap_max_ms" in s else "")
                                        for n, s in r["topics"].items()))
        o = r["outcome"]
        if "route" in r:
            print(f"   route: {r['route']}")
        elif "leg" in r:
            print(f"   leg: {json.dumps(r['leg'], default=str)[:400]}")
        else:
            print(f"   {o['command']}; mission {o['end']['mission']}; patrol {o['start']['patrol']} -> {o['end']['patrol']}")
            print(f"   arm: {o['arm_start'].get('tidy')} -> {o['arm_end'].get('tidy')}")


if __name__ == "__main__":
    main()
