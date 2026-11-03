"""Does the robot computer keep up with the simulated robot? (design.md 8, phase 1C step 2; rpi.md)

Run ON the robot computer (a Pi 4 here), beside the robot's nodes (`side:=robot`, mcu_link talking to the PC's bridge):

  python3 pi_keepup.py OUT.json [--seconds 120] [--label "T1 catch, Ethernet"] [--topics /scan ...]

Records, over the run:
  host     CPU (total, busiest core, per-process by ROS executable through cpu_watch.py), load, memory and swap, SoC
           temperature, clock speed, and the Pi's throttling flags (vcgencmd get_throttled: under-voltage, frequency
           cap, throttled, soft temperature limit; "now" and "since boot")
  topics   rate, gaps, and age (this machine's clock at receipt minus the message's stamp, sim time when the nodes run
           on it) for /mcu/state_10hz, /scan, /mcu/link, /catch/status and any --topics; with sim-time stamps it says whether the sim
           advances at wall rate here (drift: wall elapsed minus sim elapsed)
  link     the last /mcu/link and /catch/status messages (mcu_link's own counters; the game's tosses and catches)
The verdict is the reader's: results/pi_robot_*.json next to the PC's score for the same run (catch_score / goals).
"""
from __future__ import annotations

import argparse
import json
import os
import statistics as st
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
HZ = os.sysconf("SC_CLK_TCK")


def read(path: str, default=None):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return default


def cpu_times():
    out = []
    for line in Path("/proc/stat").read_text().splitlines():
        if line.startswith("cpu"):
            f = [int(x) for x in line.split()[1:]]
            out.append((sum(f), f[3] + f[4]))                     # total, idle (+iowait)
    return out                                                    # [all, cpu0, cpu1, ...]


def throttled() -> int | None:
    try:
        o = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout
        return int(o.split("=")[1], 16)
    except Exception:
        return None


FLAGS = {0: "under-voltage", 1: "arm frequency capped", 2: "throttled", 3: "soft temperature limit",
         16: "under-voltage has occurred", 17: "frequency cap has occurred", 18: "throttling has occurred",
         19: "soft temperature limit has occurred"}


def host_sampler(stop: threading.Event, rows: list, period: float = 1.0):
    prev = cpu_times()
    while not stop.wait(period):
        cur = cpu_times()
        pct = []
        for (t1, i1), (t0, i0) in zip(cur, prev):
            pct.append(100.0 * (1 - (i1 - i0) / max(t1 - t0, 1)))
        prev = cur
        mem = {k: int(v.split()[0]) for k, v in (l.split(":", 1) for l in Path("/proc/meminfo").read_text().splitlines())}
        temp = read("/sys/class/thermal/thermal_zone0/temp")
        rows.append(dict(t=time.time(), cpu_total=pct[0], cpu_cores=pct[1:], load1=os.getloadavg()[0],
                         mem_avail_mb=mem["MemAvailable"] / 1024, swap_used_mb=(mem["SwapTotal"] - mem["SwapFree"]) / 1024,
                         temp_c=int(temp) / 1000 if temp else None, freq_mhz=int(read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "0")) / 1000,
                         throttled=throttled()))


def summarize_host(rows: list) -> dict:
    if not rows:
        return {}
    def q(xs, p):
        xs = sorted(x for x in xs if x is not None)
        return xs[min(len(xs) - 1, int(p / 100 * len(xs)))] if xs else None
    cores = [max(r["cpu_cores"]) for r in rows]
    thr = [r["throttled"] for r in rows if r["throttled"] is not None]
    seen = 0
    for x in thr:
        seen |= x
    return dict(
        seconds=len(rows),
        cpu_total_pct=dict(median=st.median(r["cpu_total"] for r in rows), p95=q([r["cpu_total"] for r in rows], 95), max=max(r["cpu_total"] for r in rows),
                           note="100 = all cores busy"),
        busiest_core_pct=dict(median=st.median(cores), p95=q(cores, 95), max=max(cores)),
        load1_max=max(r["load1"] for r in rows),
        mem_available_mb_min=min(r["mem_avail_mb"] for r in rows), swap_used_mb_max=max(r["swap_used_mb"] for r in rows),
        temp_c=dict(start=rows[0]["temp_c"], max=max(r["temp_c"] for r in rows if r["temp_c"] is not None) if any(r["temp_c"] for r in rows) else None),
        freq_mhz_min=min(r["freq_mhz"] for r in rows),
        throttle_flags_seen=[FLAGS[b] for b in FLAGS if seen & (1 << b)] if thr else "vcgencmd not available",
        throttle_flags_hex=hex(seen) if thr else None)


def watch_processes(seconds: float, window: float = 10.0):
    out = Path(f"/tmp/pi_keepup_cpu_{os.getpid()}.jsonl")
    p = subprocess.Popen([sys.executable, str(HERE / "cpu_watch.py"), str(out), "--window", str(window), "--seconds", str(int(seconds))],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return p, out


def summarize_processes(path: Path) -> dict:
    rows = []
    try:
        rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    except OSError:
        return {}
    groups: dict = {}
    for r in rows:
        for g, v in r.get("cpu", {}).items():
            groups.setdefault(g, []).append(v)
    rss = {}
    for r in rows:
        for g, v in r.get("rss_mb", {}).items():
            rss[g] = max(rss.get(g, 0), v)
    res = {g: dict(median_pct=st.median(v), max_pct=max(v), rss_mb_max=rss.get(g)) for g, v in groups.items()}
    # the robot's own processes (not this monitor, ssh, the system's daemons): % of one core, summed per window
    other = ("pi_keepup.py", "sshd:", "avahi-daemon:", "systemd-journald", "polkitd", "fwupd", "ros2 launch/daemon", "record_throws.py",
             "systemd-resolved", "systemd-timesyncd", "NetworkManager", "wpa_supplicant", "irqbalance", "snapd", "unattended-upgr", "cron")
    res["_robot_total_pct_of_one_core"] = (st.median(sum(v for g, v in r.get("cpu", {}).items() if g not in other) for r in rows)
                                           if rows else None)
    res["_monitor_pct_of_one_core"] = st.median(r.get("cpu", {}).get("pi_keepup.py", 0.0) for r in rows) if rows else None
    return res


class Topics:
    """rclpy subscribers: rate, gaps, age of the robot graph's main topics."""

    def __init__(self, names, node):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from rosidl_runtime_py.utilities import get_message
        self.node, self.stats, self.names = node, {}, names
        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.get_message = get_message
        self.subs: dict = {}
        self.last_text: dict = {}

    def connect(self):
        """Subscribe to each topic once it has a publisher (types from the graph)."""
        known = dict(self.node.get_topic_names_and_types())
        for n in self.names:
            if n in self.subs or n not in known:
                continue
            tname = known[n][0]
            msg = self.get_message(tname)
            self.stats[n] = dict(type=tname, t=[], age=[], first=None, sim0=None)
            if tname.endswith("String"):
                self.subs[n] = self.node.create_subscription(msg, n, lambda m, n=n: self.last_text.__setitem__(n, m.data), self.qos)
            else:
                self.subs[n] = self.node.create_subscription(msg, n, lambda m, n=n: self.on(n, m), self.qos)

    def on(self, n, m):
        s = self.stats[n]
        wall = time.monotonic()
        now = self.node.get_clock().now().nanoseconds * 1e-9
        s["t"].append(wall)
        if hasattr(m, "header"):
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if stamp > 1e9:                                     # system-clock stamps (clock:=wall): age = our clock - stamp
                s["age"].append(1e3 * (now - stamp))
            elif stamp > 0:                                     # sim-time stamps (clock:=sim): the age is meaningless here,
                if s["first"] is None:                          # but the sim's progress against ours says if it keeps pace
                    s["first"], s["sim0"] = wall, stamp
                s["drift_ms"] = 1e3 * ((wall - s["first"]) - (stamp - s["sim0"]))

    def summary(self) -> dict:
        out = {}
        for n, s in self.stats.items():
            t = s["t"]
            if len(t) < 3:
                out[n] = dict(type=s["type"], messages=len(t))
                continue
            gaps = [1e3 * (b - a) for a, b in zip(t, t[1:])]
            gaps.sort()
            d = dict(type=s["type"], messages=len(t), rate_hz=(len(t) - 1) / (t[-1] - t[0]),
                     gap_ms=dict(median=st.median(gaps), p99=gaps[int(0.99 * (len(gaps) - 1))], max=gaps[-1]))
            if s["age"]:
                a = sorted(s["age"])
                d["age_ms"] = dict(median=st.median(a), p95=a[int(0.95 * (len(a) - 1))], max=a[-1])
            if "drift_ms" in s:
                d["clock_drift_ms"] = s["drift_ms"]
            out[n] = d
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--topics", nargs="*", default=[])
    ap.add_argument("--wait", type=float, default=0.0, help="seconds to wait first (for the graph to come up)")
    ap.add_argument("--no-ros", action="store_true", help="host numbers only")
    ap.add_argument("--raw", action="store_true", help="keep every 1 s host sample and every cpu_watch window (per-phase "
                    "numbers afterwards: arm_load.py's phase times)")
    ap.add_argument("--window", type=float, default=10.0, help="cpu_watch's window, s")
    a = ap.parse_args()
    # light topics only: a Python subscriber to a 200 Hz topic costs a Pi 4 real CPU, and this monitor must not be the load
    topics = ["/mcu/state_10hz", "/scan", "/mcu/link", "/catch/status"] + a.topics
    node = topics_obj = None
    if not a.no_ros:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        rclpy.init()
        # on the system clock: a node on sim time would take the 1 kHz /clock itself and be the biggest load on the Pi
        node = rclpy.create_node("pi_keepup")
        topics_obj = Topics(topics, node)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
    if a.wait:
        time.sleep(a.wait)
    stop, rows = threading.Event(), []
    th = threading.Thread(target=host_sampler, args=(stop, rows), daemon=True)
    th.start()
    proc, cpu_path = watch_processes(a.seconds, a.window)
    t_end = time.monotonic() + a.seconds
    t_conn = 0.0
    print(f"keep-up: {a.seconds:g} s {a.label}")
    try:
        while time.monotonic() < t_end:
            if node is not None:
                if time.monotonic() - t_conn > 1.0:                  # the graph query is not cheap: once a second
                    topics_obj.connect()
                    t_conn = time.monotonic()
                ex.spin_once(timeout_sec=0.05)
            else:
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    stop.set()
    th.join(timeout=2)
    try:
        proc.wait(timeout=15)                                        # cpu_watch ends with its own --seconds
    except subprocess.TimeoutExpired:
        proc.kill()
    res = dict(test="pi_keepup", label=a.label, time=time.strftime("%Y-%m-%dT%H:%M:%S"), host=os.uname().nodename, machine=os.uname().machine,
               governor=read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"), cores=os.cpu_count(),
               model=(read("/proc/device-tree/model", "") or "").strip("\0"), host_load=summarize_host(rows),
               processes=summarize_processes(cpu_path), topics=topics_obj.summary() if topics_obj else {},
               last_text=topics_obj.last_text if topics_obj else {})
    if a.raw:
        res["host_rows"] = rows
        try:
            res["cpu_windows"] = [json.loads(l) for l in cpu_path.read_text().splitlines() if l.strip()]
        except OSError:
            res["cpu_windows"] = []
        if topics_obj:                                               # each message's arrival time, for per-phase rates
            off = time.time() - time.monotonic()
            res["topic_times"] = {n: [round(t + off, 3) for t in s["t"]]
                                  for n, s in topics_obj.stats.items()}
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(res, indent=1, default=float))
    h = res["host_load"]
    if h:
        print(f"CPU {h['cpu_total_pct']['median']:.0f}% median / {h['cpu_total_pct']['max']:.0f}% max of {res['cores']} cores, busiest core "
              f"{h['busiest_core_pct']['max']:.0f}%; temp max {h['temp_c']['max']}; mem avail min {h['mem_available_mb_min']:.0f} MB, swap {h['swap_used_mb_max']:.0f} MB; "
              f"throttling: {h['throttle_flags_seen']}")
    for n, s in res["topics"].items():
        if "rate_hz" in s:
            print(f"  {n}: {s['rate_hz']:.1f} Hz, gap max {s['gap_ms']['max']:.0f} ms" + (f", age median {s['age_ms']['median']:.1f} ms p95 {s['age_ms']['p95']:.1f}" if "age_ms" in s else "")
                  + (f", clock drift {s['clock_drift_ms']:.0f} ms" if "clock_drift_ms" in s else ""))
    print(f"-> {a.out}")
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
