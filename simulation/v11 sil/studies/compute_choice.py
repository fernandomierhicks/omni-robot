"""Robot computer and stereo cameras, checked against this robot's measured software load (design.md 4.9, Oct 28).

Reads the shared parts database (omni-robot/database: computers.csv, cameras.csv) and the CPU the robot's own ROS 2
processes used while the sim drove them (results/compute_load.json, from ros/tools/cpu_watch.py beside
`omni.sh goals` and `omni.sh catch-score` on an i7-14700K in WSL). Turns that into a requirement in cores of the
reference CPU, scales it to each board by single-thread speed (Geekbench 6 and 7-Zip bracket it), checks the cameras
against what the vision sweep says matters (design history 21.2: global shutter, hardware sync, latency, colour for
the blob detector; 60 vs 120 fps and resolution barely matter), and costs complete computer + camera systems.

  python compute_choice.py            # prints the tables, writes results/compute_choice.md and .json
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DB_DIR = Path(os.environ.get("ACTUATOR_DB", HERE.parents[2] / "database"))
sys.path.insert(0, str(DB_DIR))
import actuator_db as adb  # noqa: E402

RESULTS = HERE / "results"

# Which measured processes ran on sim time (they took the sim's 1 kHz /clock, which the robot doesn't have) and in
# which language: stack.py's use_sim_time. heading_node and the console's nodes run on wall time.
SIM_TIME = {
    "goals": {"py": ("mcu_link", "mode_manager", "scan_prep"),
              "cpp": ("localization_slam_toolbox_node", "controller_server", "bt_navigator", "planner_server",
                      "ekf_node", "behavior_server", "collision_monitor", "smoother_server", "lifecycle_manager",
                      "velocity_smoother")},
    "catch": {"py": ("catch_perception", "mcu_link", "catch_behavior", "mode_manager"), "cpp": ()},
}
PYTHON = {"mcu_link", "mode_manager", "scan_prep", "catch_perception", "catch_behavior", "heading_node"}
NOT_ROBOT = {"SIM", "ros2 launch/daemon", "systemd-journald"}

# What the real robot runs that these two steps didn't, in cores of the reference CPU (est unless noted)
EXTRAS = {
    "operator console + face node": (0.11, "console 5-6% of a WSL core measured Oct 22 (design history App. A 2); face est"),
    "voice: Kokoro bursts + Vosk listening": (0.15, "est"),
    "slam_toolbox mapping instead of localization": (0.20, "est"),
    "camera capture, stamping, Bayer ROI at 2 x 120 fps": (0.20, "est (the sim's frames are sparse blobs)"),
    "lidar driver": (0.03, "est"),
    "AprilTag detection + tag_slam": (0.10, "est"),
    "frames, misc": (0.03, "est"),
}
CPP_PORT = 0.25          # a hot Python node rewritten in C++ costs this share of its CPU (est)

# A camera head: a Pi 5 with the two CSI cameras does capture, detection and tags, and sends detections over
# Ethernet; the main computer runs the rest (and takes no camera at all)
HEAD_PROCESSES = ("catch_perception",)
HEAD_EXTRAS = ("camera capture, stamping, Bayer ROI at 2 x 120 fps", "AprilTag detection + tag_slam")


# Second pass (Oct 28): the glue nodes in C++ (ros/src/omni_cpp) and the robot step's modes measured one at a time
# (ros/tools/mode_load.py -> results/mode_load.json): the console, slam_toolbox, the EKF and Nav2 all up, the robot
# idle, patrolling the house, then catching the house's thrower. The peak is the busiest mode with the rest idling
# beside it, not the sum of the two scored steps (which counted Nav2 driving and the catch at once).
WALL_TIME = {"heading_node", "mission_node", "rosbridge_websocket", "rosapi_node", "face_node", "image_preview",
             "console_web", "web_video_server", "mode_load.py"}
PY_NODES = {"catch_perception", "catch_behavior", "mission_node", "rosbridge_websocket", "rosapi_node", "face_node",
            "image_preview", "console_web", "tag_slam"}
GLUE = {"mcu_link", "mode_manager", "scan_prep", "heading_node"}
# what each mode adds on the robot that the sim's robot step doesn't run (EXTRAS' estimates)
MODE_EXTRAS = {
    "MANUAL": ("voice: Kokoro bursts + Vosk listening", "lidar driver", "AprilTag detection + tag_slam", "frames, misc"),
    "PATROL": ("voice: Kokoro bursts + Vosk listening", "slam_toolbox mapping instead of localization", "lidar driver",
               "AprilTag detection + tag_slam", "frames, misc"),       # Explore maps while it drives
    "CATCH": ("voice: Kokoro bursts + Vosk listening", "camera capture, stamping, Bayer ROI at 2 x 120 fps",
              "lidar driver", "AprilTag detection + tag_slam", "frames, misc"),
}


# mode_load.json's runs, in order: key, label (a column in the tables, or None: the modes line only), glue, catch
# (catch_perception / catch_behavior: py | cpp), and the sim-time nodes in Nav2's container (compose:=true: the 8
# Nav2 nodes and the container; all: + mcu_link, mode_manager, scan_prep; 0: no container)
MODE_RUNS = [
    ("py", None, "py", "py", 0),
    ("cpp", "C++ glue", "cpp", "py", 0),
    ("lean", "C++ glue + lean Nav2", "cpp", "py", 0),
    ("lean_rerun", None, "cpp", "py", 0),
    ("sleep", "+ catch asleep outside CATCH", "cpp", "py", 0),
    ("compose", "+ Nav2 composed", "cpp", "py", 9),
    ("compose_all", None, "cpp", "py", 12),
    ("catch_cpp", "+ catch nodes in C++", "cpp", "cpp", 12),
]
CATCH_NODES = {"catch_perception", "catch_behavior"}
CONTAINER = "component_container_isolated"


def mode_loads(run: dict, glue: str, ov: dict, catch: str = "py", container: int = 0) -> dict:
    """Each mode's load on the robot in reference cores (sim-time overhead removed per node, the extras added), its
    camera-head share, and the peak. A container pays the sim's clock per sim-time node it hosts, at
    component_share of a process's cost (compute_load.json; helper nodes such as the costmaps aren't counted, as
    they aren't for the separate processes)."""
    out = {}
    for ph, r in run["phases"].items():
        per = {}
        for g, s in r["groups"].items():
            if g in NOT_ROBOT:
                continue
            py = (g in PY_NODES and not (catch == "cpp" and g in CATCH_NODES)) or (glue == "py" and g in GLUE)
            o = 0 if g in WALL_TIME else (ov["python_node"] if py else ov["cpp_node"])
            if g == CONTAINER:
                o = container * ov["cpp_node"] * ov["component_share"]
            per[g] = max(0.0, s["median"] - o) / 100.0
        measured = sum(per.values())
        extras = sum(EXTRAS[k][0] for k in MODE_EXTRAS[ph])
        head = sum(per.get(g, 0.0) for g in HEAD_PROCESSES) + sum(EXTRAS[k][0] for k in HEAD_EXTRAS
                                                                 if k in MODE_EXTRAS[ph])
        out[ph] = {"measured": round(measured, 3), "extras": round(extras, 3), "total": round(measured + extras, 3),
                   "head": round(head, 3), "main": round(measured + extras - head, 3),
                   "per_process_cores": {g: round(v, 3) for g, v in sorted(per.items(), key=lambda kv: -kv[1]) if v}}
    peak = max(out, key=lambda p: out[p]["total"])
    return {"modes": out, "peak_mode": peak, "peak": out[peak]["total"],
            "head_peak": max(v["head"] for v in out.values()), "main_peak": max(v["main"] for v in out.values())}


def robot_load(m: dict) -> dict:
    """Per-process load on the robot (sim-time overhead removed), both steps together (shared nodes counted once)."""
    ov = m["sim_time_overhead_pct"]
    per: dict = {}
    for step, run in m["runs"].items():
        st = SIM_TIME[step]
        for g, s in run["groups"].items():
            if g in NOT_ROBOT:
                continue
            o = ov["python_node"] if g in st["py"] else (ov["cpp_node"] if g in st["cpp"] else 0)
            per[g] = max(per.get(g, 0.0), max(0.0, s["median"] - o) / 100.0)
    measured = sum(per.values())
    py = sum(v for g, v in per.items() if g in PYTHON)
    extras = sum(v for v, _ in EXTRAS.values())
    head_ex = sum(EXTRAS[k][0] for k in HEAD_EXTRAS)
    head = sum(per.get(g, 0.0) for g in HEAD_PROCESSES)
    head_py = sum(per.get(g, 0.0) for g in HEAD_PROCESSES if g in PYTHON)
    return {"per_process_cores": {g: round(v, 3) for g, v in sorted(per.items(), key=lambda kv: -kv[1])},
            "measured": round(measured, 3), "python_share": round(py, 3), "extras": round(extras, 3),
            "today": round(measured + extras, 3),
            "cpp_ports": round(measured - py * (1 - CPP_PORT) + extras, 3),
            "head_today": round(head + head_ex, 3),
            "head_cpp_ports": round(head - head_py * (1 - CPP_PORT) + head_ex, 3)}


def requirement(load: float, ref: dict, label: str) -> adb.ComputeRequirement:
    return adb.ComputeRequirement(label, load_ref_cores=load, ref_gb6_single=ref["gb6_single"],
                                  ref_7zip_single=ref["7zip_single_MIPS"], cpu_share_max=0.75, ubuntu_2404=True)


CAMERA_REQ = adb.CameraRequirement(
    "catch stereo pair", global_shutter=True, color=True, fps_min=60, res_min=(640, 400), hw_sync=True,
    latency_max_ms=60,
    notes="design history 21.2 / 21.4: rolling 20 ms -> 72%; sync 5 ms -> 86%; latency 20/50/100 ms -> 92/89/85%; "
          "60 vs 120 fps and 640x400 vs 1920x1200 within noise; the detector is a colour blob")

# Complete systems: (label, computer ids, camera ids with counts, extra USD est, note[, camera head computer id])
LENSES_HEAD = 75         # two Arducam LN051 3.2 mm CS lenses ($59.98) + a Pi 5 cooler and a 5 V 5 A buck (est $15)
SYSTEMS = [
    ("GMKtec M8 (6650H) + Pi 5 2GB camera head + 2 Pi GS cameras", ["gmktec-m8-6650h-16gb"],
     [("raspberrypi-gs-camera-imx296", 2)], LENSES_HEAD, "budget: both computers $467; lenses, cooler, buck",
     "raspberrypi-pi5-2gb"),
    ("GMKtec M6 Ultra (7640HS) + Pi 5 2GB camera head + 2 Pi GS cameras", ["gmktec-m6-ultra-7640hs-16gb"],
     [("raspberrypi-gs-camera-imx296", 2)], LENSES_HEAD, "lenses, cooler, buck", "raspberrypi-pi5-2gb"),
    ("GMKtec M8 (6650H) + Pi 5 4GB camera head (in stock) + 2 Pi GS cameras", ["gmktec-m8-6650h-16gb"],
     [("raspberrypi-gs-camera-imx296", 2)], LENSES_HEAD, "the in-stock Pi (Adafruit)", "raspberrypi-pi5-4gb"),
    ("Orin Nano Super kit + 2 e-con AR0234 (trigger)", ["nvidia-jetson-orin-nano-super-devkit"],
     [("econ-ecam25-cuonx", 2)], 0, "cameras' price on request"),
    ("Orin Nano Super kit + Arducam AR0234 sync stereo", ["nvidia-jetson-orin-nano-super-devkit"],
     [("arducam-b0492n-ar0234-stereo-jetson", 1)], 0, "trigger VERIFY on this kit"),
    ("Orin NX 16GB on a J401 + 2 e-con AR0234", ["nvidia-jetson-orin-nx-16gb-module", "seeed-j401-carrier"],
     [("econ-ecam25-cuonx", 2)], 60, "+ NVMe and cooler (est $60)"),
    ("Ryzen 7 8845HS mini PC + OAK-FFC 4P + 2 OV9782", ["generic-ryzen7-8845hs-minipc"],
     [("luxonis-oak-ffc-4p", 1), ("luxonis-oak-ffc-ov9782", 2)], 200, "+ 32 GB DDR5 and an SSD (est $200)"),
    ("Core Ultra 5 NUC 14 Pro + OAK-FFC 4P + 2 OV9782", ["asus-nuc-14-pro-cu5-125h"],
     [("luxonis-oak-ffc-4p", 1), ("luxonis-oak-ffc-ov9782", 2)], 200, "+ RAM and SSD (est $200); launch price"),
    ("N150 mini PC 16GB + OAK-FFC 4P + 2 OV9782", ["generic-n150-minipc-16gb"],
     [("luxonis-oak-ffc-4p", 1), ("luxonis-oak-ffc-ov9782", 2)], 0, ""),
    ("Pi 5 16GB + 2 Pi Global Shutter cameras (XTR)", ["raspberrypi-pi5-16gb"],
     [("raspberrypi-gs-camera-imx296", 2)], 50, "+ two C/CS lenses (est $50)"),
    ("Orin Nano Super kit + OAK-FFC 4P + 2 OV9782", ["nvidia-jetson-orin-nano-super-devkit"],
     [("luxonis-oak-ffc-4p", 1), ("luxonis-oak-ffc-ov9782", 2)], 0, "the OAK does capture and ISP; CPU as the kit"),
]


def fmt_checks(checks) -> str:
    return "; ".join(f"{c.name} {c.status}" + (f" ({c.value})" if c.status != "PASS" and c.value is not None else "")
                     for c in checks)


def main():
    db = adb.load(DB_DIR)
    m = json.loads((RESULTS / "compute_load.json").read_text())
    ref = m["reference_cpu"]
    L = robot_load(m)
    # each load: (whole robot, main computer beside a camera head, the camera head) in reference cores
    loads = {"first pass": (L["today"], L["today"] - L["head_today"], L["head_today"])}
    ml_path = RESULTS / "mode_load.json"
    ML = {}
    if ml_path.exists():
        mlj = json.loads(ml_path.read_text())
        for name, label, glue, catch, container in MODE_RUNS:
            if name in mlj["runs"]:
                ML[name] = mode_loads(mlj["runs"][name], glue, m["sim_time_overhead_pct"], catch, container)
                if label:
                    loads[label] = (ML[name]["peak"], ML[name]["main_peak"], ML[name]["head_peak"])
    reqs = {k: requirement(v[0], ref, k) for k, v in loads.items()}
    split = {k: (requirement(v[1], ref, k + ", main"), requirement(v[2], ref, k + ", head")) for k, v in loads.items()}
    out = {"load": L, "mode_loads": ML, "loads": loads, "extras": {k: v for k, (v, _) in EXTRAS.items()},
           "computers": {}, "cameras": {}, "systems": []}
    md = ["# Robot computer and cameras (compute_choice.py)", "",
          f"Reference CPU: {ref['name']} (GB6 single {ref['gb6_single']}, 7-Zip single {ref['7zip_single_MIPS']} MIPS).",
          f"First pass (Python glue, the two scored steps summed): measured {L['measured']:.2f} (Python "
          f"{L['python_share']:.2f}) + extras (est) {L['extras']:.2f} = {L['today']:.2f} reference cores."]
    for name, r in ML.items():
        md.append(f"Modes ({name}): " + "; ".join(f"{p} {v['measured']:.2f} measured + {v['extras']:.2f} est = "
                                                  f"{v['total']:.2f}" for p, v in r["modes"].items()) +
                  f"; peak {r['peak']:.2f} ({r['peak_mode']}); camera head {r['head_peak']:.2f}, the rest "
                  f"{r['main_peak']:.2f}.")
    md += ["", "## Computers (CPU: cores needed of the board's, 75% cap)", "",
           "| Computer | $ | Cores | " + " | ".join(loads) + " | Ubuntu 24.04 | CSI |",
           "| --- | --- | --- | " + " | ".join("---" for _ in loads) + " | --- | --- |"]
    print("\n".join(md[2:4 + len(ML)]))
    print(f"\n{'computer':42s} {'$':>7s} {'eff':>5s}  " + " ".join(f"{k[:16]:>16s}" for k in loads))
    for pid, b in sorted(db.computers.items(), key=lambda kv: kv[1].get("price_usd") or 1e9):
        if b.get("kind") in ("carrier", "accelerator"):
            continue
        row, cells = {}, []
        for k, r in reqs.items():
            cpu = next(c for c in adb.check_compute(b, r) if c.name == "CPU")
            row[k] = {"status": cpu.status, "need": cpu.value, "cap": cpu.need}
            cells.append(f"{cpu.status} {cpu.value or ''}")
        eff = adb.effective_cores(b)
        out["computers"][pid] = row
        print(f"{pid:42s} {b.get('price_usd') or '':>7} {eff if eff is not None else '':>5}  " +
              " ".join(f"{c:>16s}" for c in cells))
        md.append(f"| {b['name']} | {b.get('price_usd') or '?'} | {eff if eff is not None else '?'} | " + " | ".join(cells) +
                  f" | {b.get('ubuntu_2404') or '?'} | {b.get('csi_cameras') if b.get('csi_cameras') is not None else '?'} |")
    md += ["", "## Cameras", "", "| Camera | $ | Verdict | Checks |", "| --- | --- | --- | --- |"]
    print("\ncameras:")
    for pid, c in db.cameras.items():
        if c.get("kind") == "capture_board":
            continue
        ch = adb.check_camera(c, CAMERA_REQ)
        v = adb.passes(ch)
        out["cameras"][pid] = {"verdict": v, "checks": [str(x) for x in ch]}
        print(f"  {v:7s} {pid:40s} {fmt_checks(ch)}")
        md.append(f"| {c['name']} | {c.get('price_usd') or '?'} | {v} | {fmt_checks(ch)} |")
    md += ["", "## Systems (computer + cameras)", "",
           "| System | $ (known parts + est extras) | " + " | ".join(f"CPU, {k}" for k in loads) + " | Cameras | Notes |",
           "| --- | --- | " + " | ".join("---" for _ in loads) + " | --- | --- |"]
    print("\nsystems:")
    cpu_of = lambda b, rq: next(c for c in adb.check_compute(b, rq) if c.name == "CPU")  # noqa: E731
    for label, comps, cams, extra, note, *head_id in SYSTEMS:
        board = db.computers[comps[0]]
        head = db.computers[head_id[0]] if head_id else None
        cost, unknown = extra, []
        for p in comps + head_id:
            pr = db.computers[p].get("price_usd")
            cost += pr or 0
            unknown += [] if pr else [p]
        cam_v = []
        card = next((db.cameras[cid] for cid, _ in cams if db.cameras[cid].get("kind") == "capture_board"), None)
        for cid, n in cams:
            c = db.cameras[cid]
            cost += (c.get("price_usd") or 0) * n
            unknown += [] if c.get("price_usd") else [cid]
            if c.get("kind") != "capture_board":
                if card is not None and c.get("latency_ms") is None and card.get("latency_ms") is not None:
                    c = adb.Part(c, latency_ms=card["latency_ms"])      # a module's latency is its capture board's
                    c.table = "cameras"
                cam_v.append(adb.passes(adb.check_camera(c, CAMERA_REQ)))
            assert adb.camera_fits(c, head or board) or c.get("hosts") == ["oak_ffc"], (cid, comps[0])
        rec = {"system": label, "usd": round(cost), "price_unknown": unknown, "cameras": cam_v, "note": note,
               "cpu": {}}
        cells = []
        for k in loads:
            main_cpu = cpu_of(board, split[k][0] if head else reqs[k])
            hc = cpu_of(head, split[k][1]) if head else None
            st = min([main_cpu.status] + ([hc.status] if hc else []), key=["FAIL", "UNKNOWN", "PASS"].index)
            rec["cpu"][k] = {"status": st, "main": main_cpu.value, "head": hc.value if hc else None}
            cells.append(f"{st} ({main_cpu.value}{'; head ' + hc.value if hc else ''})")
        out["systems"].append(rec)
        print(f"  ${cost:6.0f}{'+' if unknown else ' '} " + " / ".join(f"{v['status']:7s}" for v in rec["cpu"].values()) +
              f" cams {cam_v}  {label}")
        md.append(f"| {label} | {cost:.0f}{' + ' + ', '.join(unknown) if unknown else ''} | " + " | ".join(cells) +
                  f" | {', '.join(cam_v)} | {note} |")
    pi = db.computers["raspberrypi-pi5-4gb"]
    first = next(iter(reqs.values()))
    per_pi = first.cpu_share_max * adb.effective_cores(pi)
    n_pi = {k: [math.ceil(x / per_pi) for x in adb.cores_needed(pi, r)] for k, r in reqs.items()}
    out["pi5_count"] = n_pi
    md += ["", f"Camera head (a Pi 5 with the cameras): {', '.join(HEAD_PROCESSES + HEAD_EXTRAS)}.",
           f"Raspberry Pi 5s for the whole stack at {first.cpu_share_max:.0%} of 4 cores each, if it split perfectly: " +
           "; ".join(f"{k} {v[0]}-{v[1]}" for k, v in n_pi.items()) + ".",
           "", "Extras (est, reference cores): " + "; ".join(f"{k} {v}" for k, (v, _) in EXTRAS.items()) +
           ". The first pass adds them all; the modes add what each mode would run (MODE_EXTRAS).", ""]
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "compute_choice.md").write_text("\n".join(md), encoding="utf-8")
    (RESULTS / "compute_choice.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"\nloads (reference cores: robot, main beside a head, head): {json.dumps(loads)}")
    print(f"wrote {RESULTS / 'compute_choice.md'}")


if __name__ == "__main__":
    main()
