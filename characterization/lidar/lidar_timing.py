"""Test 1 of the lidar bench (README.md): timing and stability from lidar_capture.py's raw streams. No setup needed.

    python lidar_timing.py C:/Users/ferna/lidar_data/SESSION [--out results]

Every packet (a standard sample, or an express / ultra capsule) is found in the byte stream with the robot's own
driver's framing (omni_console/rplidar.py) and given the host time its last byte arrived. Then:

  sample clock   the lidar has no clock on the wire, but it samples at a fixed rate: packet j ends at sample N_j, so
                 arrival t_j = a + b N_j + delivery delay (>= 0). A line through the lower envelope of (N_j, t_j) gives
                 the true sample period b (against the Pi's NTP clock) and each packet's delivery delay above the
                 fastest (its jitter; the fixed part needs a motion test: omni_calib.temporal).
  turns          samples between start flags; a turn's period = its samples x b (the lidar's own clock), so the
                 turn-to-turn variation is the motor's, not USB's. Spin-up: from the scan's start until the rate stays
                 within 1 % of its steady median. A driver that stamps a turn when its last packet arrives (sllidar
                 does about that) carries the delivery jitter: reported per turn.
  angles         the step between samples, its spread, backward steps and the widest gap; where turns start. Within a
                 turn: the encoder angle against the sample clock (capsule headers; Standard's own sample angles),
                 a line per turn, its residual = how far the head's speed wanders inside a turn (scan_time.py assumes
                 constant).
  link           bytes/s against the serial line's 115200 baud (10 bits a byte).

Writes results/timing_<session>.json (no doubled prefix) and prints a summary table.
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "ros", "src", "omni_console", "omni_console"))
import rplidar  # noqa: E402  (the robot's driver)

PER_PACKET = {rplidar.ANS_STANDARD: 1, rplidar.ANS_EXPRESS: 32, rplidar.ANS_ULTRA: 96}


def packets(ans: int, data: bytes):
    """(end offset, bytes) of every well-formed packet, with the driver's own sync rules; and the bytes skipped."""
    out, i, bad = [], 0, 0
    if ans == rplidar.ANS_STANDARD:
        while len(data) - i >= 5:
            b0, b1 = data[i], data[i + 1]
            if (b0 & 1) == ((b0 >> 1) & 1) or not b1 & 1:
                bad, i = bad + 1, i + 1
                continue
            out.append((i + 5, data[i:i + 5]))
            i += 5
        return out, bad
    n = 84 if ans == rplidar.ANS_EXPRESS else 132
    while len(data) - i >= n:
        c = data[i:i + n]
        if c[0] >> 4 != 0xA or c[1] >> 4 != 0x5:
            bad, i = bad + 1, i + 1
            continue
        cs = 0
        for b in c[2:]:
            cs ^= b
        if cs != ((c[0] & 0xF) | ((c[1] & 0xF) << 4)):
            bad, i = bad + 1, i + 1
            continue
        out.append((i + n, c))
        i += n
    return out, bad


def decode(ans: int, pk):
    """Samples (angle deg, mm, quality, start) per packet index, as the driver decodes them; capsule header angles."""
    samples, owner, hdr = [], [], []
    if ans == rplidar.ANS_STANDARD:
        p = rplidar.StandardParser()
        for j, (_, b) in enumerate(pk):
            s = p.feed(b)
            samples += s
            owner += [j] * len(s)
            hdr.append(s[0][0] if s else np.nan)
        return samples, np.array(owner), np.array(hdr), np.zeros(len(pk), bool)
    dec = rplidar.PARSERS[ans]()
    new = np.zeros(len(pk), bool)
    prev = None
    for j, (_, c) in enumerate(pk):
        start = c[2] | (c[3] << 8)
        hdr.append((start & 0x7FFF) / 64.0)
        new[j] = bool(start & 0x8000)
        if new[j]:
            prev = None
        if prev is not None:
            s = dec.decode(pk[prev][1], c)
            samples += s
            owner += [prev] * len(s)
        prev = j
    return samples, np.array(owner), np.array(hdr), new


def envelope_fit(N, t, bin_s=1.0):
    """t = a + b N through the lower envelope: least squares, then the minimum per bin, refit; delays >= 0."""
    b, a = np.polyfit(N, t, 1)
    edges = np.arange(t[0], t[-1] + bin_s, bin_s)
    idx = np.digitize(t, edges)
    sel = []
    for k in np.unique(idx):
        m = np.where(idx == k)[0]
        sel.append(m[np.argmin(t[m] - (a + b * N[m]))])
    sel = np.array(sel)
    if len(sel) >= 3:
        b, a = np.polyfit(N[sel], t[sel], 1)
    d = t - (a + b * N)
    a += d.min()
    return a, b, t - (a + b * N)


def pct(x, qs=(50, 95, 99, 100)):
    x = np.asarray(x)
    return {f"p{q}" if q < 100 else "max": round(float(np.percentile(x, q)), 3) for q in qs} if x.size else {}


def analyse(run: dict, path: str) -> dict:
    z = np.load(path)
    t_chunk, n_chunk, data = z["t"], z["n"], z["data"].tobytes()
    ans = run["ans_type"]
    k = PER_PACKET[ans]
    pk, bad = packets(ans, data)
    ends = np.cumsum(n_chunk)
    t_pk = t_chunk[np.searchsorted(ends, np.array([e for e, _ in pk]), side="left")]
    samples, owner, hdr, new = decode(ans, pk)
    check = sum(len(tr.angle) for tr in rplidar.decode_bytes(ans, data))
    ang = np.array([s[0] for s in samples])
    mm = np.array([s[1] for s in samples])
    q = np.array([s[2] for s in samples])
    st = np.array([s[3] for s in samples])

    # sample clock: packet j ends at sample (j + 1) k (no lost packets: checked by the skipped bytes)
    N = (np.arange(len(pk)) + 1) * k
    a, b, delay = envelope_fit(N.astype(float), t_pk)
    us_nom = run["us_per_sample"]
    res = dict(name=run["name"], mode=run["mode"], pwm=run["pwm"], seconds=run["seconds"], answer=f"0x{ans:02X}",
               bytes=len(data), skipped_bytes=bad, packets=len(pk), samples=len(samples), driver_samples=check,
               bytes_per_s=round(len(data) / (t_chunk[-1] - t_chunk[0]), 1),
               link_use=round(len(data) / (t_chunk[-1] - t_chunk[0]) / (115200 / 10), 3),
               sample_us_nominal=us_nom, sample_us=round(b * 1e6, 4), clock_ppm=round((b * 1e6 / us_nom - 1) * 1e6),
               delivery_ms=pct(delay * 1e3), reads=len(t_chunk), bytes_per_read=round(len(data) / len(t_chunk), 1))

    # turns from the start flags; the first (partial) one dropped
    starts = np.where(st)[0]
    turns = []
    for i0, i1 in zip(starts[:-1], starts[1:]):
        n = i1 - i0
        last_pk = owner[i1 - 1]
        turns.append(dict(i0=i0, n=n, t_end=t_pk[last_pk], N_end=(last_pk + 1) * k, first_deg=ang[i0]))
    if len(turns) < 5:
        res["error"] = f"only {len(turns)} turns"
        return res
    n_t = np.array([x["n"] for x in turns])
    period = n_t * b                                       # s, the lidar's own clock
    hz = 1 / period
    t_rel = np.array([x["t_end"] for x in turns]) - t_chunk[0]
    tail = hz[len(hz) // 2:]
    f_ss = float(np.median(tail))
    out_band = np.where(np.abs(hz / f_ss - 1) > 0.01)[0]
    settle = int(out_band[-1] + 1) if out_band.size else 0
    if settle >= len(hz) - 3:
        settle = 0
    ss = slice(settle, None)
    res.update(turns=len(turns), hz=round(f_ss, 3), hz_mean=round(float(hz[ss].mean()), 3),
               period_cv_pct=round(float(period[ss].std() / period[ss].mean() * 100), 3),
               period_ms=pct(period[ss] * 1e3, (0, 50, 100)) | {"std": round(float(period[ss].std() * 1e3), 3)},
               samples_per_turn=dict(mean=round(float(n_t[ss].mean()), 1), std=round(float(n_t[ss].std()), 1),
                                     min=int(n_t[ss].min()), max=int(n_t[ss].max())),
               spinup_s=round(float(t_rel[settle - 1]) if settle else 0.0, 2), spinup_turns=settle,
               scan_after_motor_s=round(float(t_chunk[0] - run["t_motor"]), 2) if run.get("t_motor") else None,
               first_turn_hz=round(float(hz[0]), 2))
    # the turn rate's curve from the motor's start (the scan starts after the driver's 1 s soft ramp)
    t_mot = t_rel + (t_chunk[0] - run["t_motor"] if run.get("t_motor") else 0.0)
    res["hz_at_s"] = {f"{s:g}": round(float(np.median(hz[np.abs(t_mot - s) < max(0.25, s * 0.02)])), 3)
                      for s in (1.5, 2, 3, 5, 10, 20, 30, 60, 120, 180, 240, 290)
                      if (np.abs(t_mot - s) < max(0.25, s * 0.02)).any()}
    if len(hz[ss]) > 20:                                   # drift of the turn rate over the run
        slope = np.polyfit(t_rel[ss], hz[ss], 1)[0]
        res["hz_drift_per_min"] = round(float(slope * 60), 4)
    # a stamp at the arrival of the turn's last packet, against the sample-clock line
    stamp_err = np.array([x["t_end"] - (a + b * x["N_end"]) for x in turns])[ss]
    res["turn_stamp_jitter_ms"] = pct(stamp_err * 1e3)
    # how long after a turn's last sample was measured the driver can have it: the rest of its packet, the packet's
    # transmission, and for capsules the next capsule (decoding needs its start angle); above the fastest delivery
    first_of = {}
    for i, o in enumerate(owner):
        first_of.setdefault(int(o), i)
    ready = []
    for i0, i1 in zip(starts[:-1], starts[1:]):
        o = int(owner[i1 - 1])
        ns = o * k + (i1 - 1 - first_of[o]) + 1          # samples measured up to and including the turn's last
        dec = o if ans == rplidar.ANS_STANDARD else o + 1
        if dec < len(t_pk):
            ready.append(t_pk[dec] - (a + b * ns))
    res["turn_ready_ms"] = pct(np.array(ready[settle:]) * 1e3, (0, 50, 95, 100))
    res["packet_tx_ms"] = round({rplidar.ANS_STANDARD: 5, rplidar.ANS_EXPRESS: 84, rplidar.ANS_ULTRA: 132}[ans]
                                * 10 / run.get("baud", 115200) * 1e3, 2)
    # period as an arrival-stamping driver would see it (difference of stamps) vs the lidar's own
    dt_stamp = np.diff([x["t_end"] for x in turns])[settle:]
    res["stamp_period_err_ms"] = pct(np.abs(dt_stamp - period[ss][1:][:len(dt_stamp)]) * 1e3)

    # angles within steady turns
    steps, back, gaps, first, resid = [], 0, [], [], []
    for x in turns[settle:]:
        aa = ang[x["i0"]:x["i0"] + x["n"]]
        d = (np.diff(aa) + 180.0) % 360.0 - 180.0
        back += int((d < 0).sum())
        steps.append(d)
        gaps.append(float(d.max()))
        first.append(float(x["first_deg"] if x["first_deg"] < 180 else x["first_deg"] - 360))
    steps = np.concatenate(steps)
    res["angle"] = dict(step_mean_deg=round(float(steps.mean()), 4), step_std_deg=round(float(steps.std()), 4),
                        step_p1_p99_deg=[round(float(np.percentile(steps, 1)), 4), round(float(np.percentile(steps, 99)), 4)],
                        backward_steps_per_turn=round(back / len(turns[settle:]), 2),
                        widest_gap_deg=pct(gaps, (50, 100)), first_sample_deg=pct(first, (0, 50, 100)))
    # head speed inside a turn: the encoder's angle (each capsule header's; the samples between are interpolated and
    # then shifted by the triangulation's range-dependent parallax correction, as are Standard's, so Standard has no
    # clean encoder angle and is skipped) against the sample clock, a line per two-turn window; the residual is how
    # far the head runs ahead of / behind a constant speed. Boost/Stability have ~3.5 headers a turn, Express ~11.
    enc_ang, enc_N, enc_t = hdr, (np.arange(len(pk)) * k).astype(float), t_pk
    if ans == rplidar.ANS_STANDARD:
        enc_ang = enc_ang[:0]
        enc_N, enc_t = enc_N[:0], enc_t[:0]
    unw = np.degrees(np.unwrap(np.radians(enc_ang)))
    t0_ss = turns[settle]["t_end"]
    win = 2 * float(n_t[ss].mean())
    use = enc_t >= t0_ss
    x_all, y_all = enc_N[use], unw[use]
    phase, res_deg = [], []
    for w0 in np.arange(x_all[0], x_all[-1] - win, win) if x_all.size else []:
        m = (x_all >= w0) & (x_all < w0 + win)
        if m.sum() < 5:
            continue
        p = np.polyfit(x_all[m], y_all[m], 1)
        r = y_all[m] - np.polyval(p, x_all[m])
        resid.append(float(np.sqrt(np.mean(r ** 2))))
        phase.append(np.mod(y_all[m], 360.0))
        res_deg.append(r)
    if resid:
        res["in_turn_angle_rms_deg"] = pct(resid, (50, 95, 100))
        res["in_turn_time_rms_ms"] = round(float(np.median(resid)) / 360 / f_ss * 1e3, 3)
        # the once-a-turn part: the residual's mean by angle (12 bins), i.e. a speed wobble fixed to the head
        ph, rr = np.concatenate(phase), np.concatenate(res_deg)
        bins = np.minimum((ph / 30).astype(int), 11)
        prof = [float(rr[bins == i].mean()) if (bins == i).any() else 0.0 for i in range(12)]
        res["angle_vs_constant_speed_by_30deg"] = [round(x, 3) for x in prof]
    res["valid_share"] = round(float((mm > 0).mean()), 3)
    if ans == rplidar.ANS_STANDARD:
        res["quality"] = pct(q[mm > 0], (1, 50, 99))
    res["range_m_median"] = round(float(np.median(mm[mm > 0]) / 1000), 3) if (mm > 0).any() else None
    res["pi"] = dict(before=run.get("pi_before"), after=run.get("pi_after"))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("session_dir")
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.session_dir, "session.json")))
    rows = []
    for run in meta["runs"]:
        if run.get("skipped"):
            continue
        rows.append(analyse(run, os.path.join(a.session_dir, run["name"] + ".npz")))
    os.makedirs(a.out, exist_ok=True)
    keep = {k: meta[k] for k in ("session", "host", "port", "baud", "info", "health", "rates", "modes", "typical",
                                 "plan", "gap_s", "t_wall", "pi")}
    s = meta["session"]
    path = os.path.join(a.out, (s if s.startswith("timing") else f"timing_{s}") + ".json")
    with open(path, "w") as f:
        json.dump(dict(device=keep, runs=rows), f, indent=1)
    print(f"{meta['info']['name']} fw {meta['info']['firmware']} at {meta['baud']} baud -> {path}\n")
    hdr = ("run", "mode", "pwm", "Hz", "CV%", "n/turn", "spin-up s", "us/sample", "ppm", "link", "deliv p95/max ms",
           "stamp p95 ms", "ready p50/max ms", "step deg", "back/turn", "in-turn rms deg", "valid", "skipped B")
    print(" | ".join(hdr))
    for r in rows:
        if "error" in r:
            print(r["name"], r["error"])
            continue
        print(" | ".join(str(x) for x in (
            r["name"][4:6], r["mode"], r["pwm"], r["hz"], r["period_cv_pct"],
            f"{r['samples_per_turn']['mean']}+/-{r['samples_per_turn']['std']}", r["spinup_s"], r["sample_us"],
            r["clock_ppm"], r["link_use"], f"{r['delivery_ms']['p95']}/{r['delivery_ms']['max']}",
            r["turn_stamp_jitter_ms"]["p95"], f"{r['turn_ready_ms']['p50']}/{r['turn_ready_ms']['max']}",
            r["angle"]["step_mean_deg"], r["angle"]["backward_steps_per_turn"],
            r.get("in_turn_angle_rms_deg", {}).get("p50"), r["valid_share"], r["skipped_bytes"])))


if __name__ == "__main__":
    main()
