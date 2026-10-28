"""The MCU's clock on the Jetson's clock (design.md 10): t_host = a * t_mcu + b, offset and drift. No ROS here; the
link node, the sim's in-process harness and the tests share it.

Two kinds of evidence, used in this order:

  pulses  the Teensy raises SYNC_OUT on a hardware timer once a second and reports the edge's MCU time (a TIME event);
          the Jetson timestamps the same edge on a GPIO (the kernel's GPIO event stamp: HTE on an Orin AON pin is ~1 us,
          a plain interrupt ~5-30 us with a late tail). Each pair is one point of the line. A least-squares line over
          the last pulse_window_s, late outliers rejected (an interrupt stamp is only ever late), gives the offset and
          the drift to a few microseconds.
  echoes  the NTP-style exchange on the link itself: the CMD carries the Jetson's send time t1, STATE echoes it with
          the MCU's receive time t2 and send time t3, the Jetson notes the arrival t4. The offset of an exchange is
          only known to +/- its round trip / 2, and any asymmetry of the USB path (host -> device and device -> host
          are scheduled differently) biases it by (d_up - d_down) / 2, unseen. The fastest exchange of each second
          over echo_window_s gives a line too: the fallback when no pulses come, and the coarse map that pairs a
          pulse with its edge.

Every time the MCU reports is mapped with the active line; status() says which one is active, its residual, the
drift, and how far the echo line sits from the pulse line (the USB path's asymmetry, which only the pulses can see).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np


class ClockSync:
    def __init__(self, echo_window_s: float = 30.0, pulse_window_s: float = 120.0, pulse_period_s: float = 1.0,
                 edge_latency_ns: int = 0, refit_s: float = 0.25):
        self.echo_window_ns = int(echo_window_s * 1e9)
        self.pulse_window_ns = int(pulse_window_s * 1e9)
        self.period_ns = int(pulse_period_s * 1e9)
        self.edge_latency_ns = int(edge_latency_ns)     # the GPIO stamp's own fixed delay (bench: scope vs stamp)
        self.refit_ns = int(refit_s * 1e9)
        self.m0 = self.h0 = None                        # reference point (MCU ns, host ns): the line is fitted about it
        self.bins: dict = {}                            # echo: host second -> (rtt, mcu mid ns, host mid ns)
        self.pairs: deque = deque()                     # pulses: (mcu ns, host ns, seq)
        self.mcu_pulses: deque = deque()                # unpaired: (seq, mcu ns)
        self.edges: deque = deque()                     # unpaired host edge stamps (ns)
        self.echo_fit = self.pulse_fit = None           # (b ns, e) with host = h0 + b + (1 + e) (mcu - m0)
        self.echo_rms = self.pulse_rms = math.nan
        self.n_echo_used = self.n_pulse_used = 0
        self.rtt_ns = None                              # the fastest round trip in the echo window
        self.t_last_host = None
        self._t_refit = None
        self.n_resets = 0
        self.n_pulses = self.n_edges = self.n_paired = self.n_rejected = 0

    # -- evidence in ---------------------------------------------------------------------------------------------
    def add(self, t1_host_ns: int, t2_mcu_us: int, t3_mcu_us: int, t4_host_ns: int):
        """One CMD / STATE echo (protocol.TimeSync's interface)."""
        if not t1_host_ns or not t2_mcu_us or t4_host_ns < t1_host_ns:
            return
        t2, t3 = t2_mcu_us * 1000, t3_mcu_us * 1000
        rtt = (t4_host_ns - t1_host_ns) - (t3 - t2)
        if rtt < 0:
            return
        mcu_mid, host_mid = (t2 + t3) // 2, (t1_host_ns + t4_host_ns) // 2
        if self.m0 is None:
            self._reset(mcu_mid, host_mid)
        elif abs(self.mcu_to_host_ns(mcu_mid) - host_mid) > 1_000_000_000:
            self._reset(mcu_mid, host_mid)              # the MCU rebooted (its clock restarted) or the host's jumped
        key = host_mid // 1_000_000_000
        cur = self.bins.get(key)
        if cur is None or rtt < cur[0]:
            self.bins[key] = (rtt, mcu_mid, host_mid)
        self.t_last_host = t4_host_ns
        old = (t4_host_ns - self.echo_window_ns) // 1_000_000_000
        for k in [k for k in self.bins if k < old]:
            del self.bins[k]
        self.rtt_ns = min(v[0] for v in self.bins.values())
        if self._t_refit is None or t4_host_ns - self._t_refit >= self.refit_ns or self.echo_fit is None:
            self._t_refit = t4_host_ns
            self._fit_echo()
            self._pair()

    def add_mcu_pulse(self, seq: int, t_mcu_us: int):
        """The MCU says: SYNC_OUT pulse seq rose at t_mcu_us (its TIME event)."""
        self.n_pulses += 1
        self.mcu_pulses.append((int(seq), int(t_mcu_us) * 1000))
        while len(self.mcu_pulses) > 16:
            self.mcu_pulses.popleft()
        self._pair()

    def add_host_edge(self, t_host_ns: int):
        """The Jetson's GPIO saw SYNC_OUT rise at t_host_ns (this clock)."""
        self.n_edges += 1
        self.edges.append(int(t_host_ns) - self.edge_latency_ns)
        self.t_last_host = max(self.t_last_host or 0, int(t_host_ns))
        while len(self.edges) > 16:
            self.edges.popleft()
        self._pair()

    # -- the lines ---------------------------------------------------------------------------------------------
    def _reset(self, mcu_ns: int, host_ns: int):
        if self.m0 is not None:
            self.n_resets += 1
        self.m0, self.h0 = mcu_ns, host_ns
        self.bins.clear()
        self.pairs.clear()
        self.echo_fit = self.pulse_fit = None

    @staticmethod
    def _line(x: np.ndarray, y: np.ndarray, w: np.ndarray, prior_e: float):
        """Weighted least squares y = b + e x (x, y in ns about the reference). One point: the offset only, with the
        drift kept at its prior; points over less than 2 s: the same (the slope isn't observable yet)."""
        if len(x) == 1 or x.max() - x.min() < 2e9:
            b = float(np.sum(w * (y - prior_e * x)) / np.sum(w))
            return b, prior_e
        W = np.sum(w)
        xm, ym = np.sum(w * x) / W, np.sum(w * y) / W
        e = float(np.sum(w * (x - xm) * (y - ym)) / np.sum(w * (x - xm) ** 2))
        return float(ym - e * xm), e

    def _fit_echo(self):
        if not self.bins:
            return
        v = np.array(list(self.bins.values()), dtype=float)
        x = v[:, 1] - self.m0
        y = v[:, 2] - self.h0 - x                       # what the clocks disagree by, beyond rate 1
        w = 1.0 / np.maximum(v[:, 0] * 0.5, 1e3) ** 2   # +/- rtt / 2
        prior = self.echo_fit[1] if self.echo_fit else 0.0
        b, e = self._line(x, y, w, prior)
        r = y - (b + e * x)
        self.echo_fit = (b, e)
        self.echo_rms = float(np.sqrt(np.mean(r ** 2)))
        self.n_echo_used = len(x)

    def _pair(self):
        """Match each MCU pulse with the host edge nearest its predicted time (within a fifth of the period)."""
        if self.m0 is None or not self.mcu_pulses or not self.edges:
            return
        tol = max(self.period_ns // 5, 20_000_000)
        changed = False
        for seq, t_m in list(self.mcu_pulses):
            pred = self.mcu_to_host_ns(t_m)
            best = min(self.edges, key=lambda h: abs(h - pred))
            if abs(best - pred) <= tol:
                self.pairs.append((t_m, best, seq))
                self.mcu_pulses.remove((seq, t_m))
                self.edges.remove(best)
                self.n_paired += 1
                changed = True
        if self.edges and self.t_last_host is not None:      # edges nobody claimed (noise on the line, a lost event)
            while self.edges and self.edges[0] < self.edges[-1] - 3 * self.period_ns:
                self.edges.popleft()
        if changed:
            last = self.pairs[-1][1]
            while self.pairs and self.pairs[0][1] < last - self.pulse_window_ns:
                self.pairs.popleft()
            self._fit_pulses()

    def _fit_pulses(self):
        v = np.array([(m, h) for m, h, _ in self.pairs], dtype=float)
        x = v[:, 0] - self.m0
        y = v[:, 1] - self.h0 - x
        keep = np.ones(len(x), bool)
        prior = self.pulse_fit[1] if self.pulse_fit else (self.echo_fit[1] if self.echo_fit else 0.0)
        b, e = self._line(x, y, np.ones(len(x)), prior)
        for _ in range(3):                              # a GPIO stamp is only ever late: drop the late tail
            r = y - (b + e * x)
            s = 1.4826 * np.median(np.abs(r[keep] - np.median(r[keep]))) if keep.sum() > 4 else 2e3
            new = r <= max(3.0 * s, 2e3)
            new &= r >= -max(6.0 * s, 1e4)
            if new.sum() < 2 or np.array_equal(new, keep):
                break
            keep = new
            b, e = self._line(x[keep], y[keep], np.ones(int(keep.sum())), prior)
        r = (y - (b + e * x))[keep]
        self.n_rejected = int((~keep).sum())
        self.pulse_fit = (b, e)
        self.pulse_rms = float(np.sqrt(np.mean(r ** 2)))
        self.n_pulse_used = int(keep.sum())

    # -- mapping -----------------------------------------------------------------------------------------------
    @property
    def mode(self) -> str:
        if self.pulse_fit is not None and self.n_pulse_used >= 3 and self.t_last_host is not None and \
                self.pairs and self.t_last_host - self.pairs[-1][1] < 5 * self.period_ns:
            return "pulse"
        return "echo" if self.echo_fit is not None else "none"

    def _active(self):
        return self.pulse_fit if self.mode == "pulse" else self.echo_fit

    def mcu_to_host_ns(self, t_mcu_ns: float) -> float:
        f = self._active()
        if f is None:
            return math.nan
        x = t_mcu_ns - self.m0
        return self.h0 + x + f[0] + f[1] * x

    def host_to_mcu_ns(self, t_host_ns: float) -> float:
        f = self._active()
        if f is None:
            return math.nan
        return self.m0 + (t_host_ns - self.h0 - f[0]) / (1.0 + f[1])

    def to_host_s(self, t_mcu_us: int) -> float:
        if self.m0 is None:
            return math.nan
        return self.mcu_to_host_ns(t_mcu_us * 1000) * 1e-9

    def to_mcu_us(self, t_host_s: float) -> int:
        if self.m0 is None or self._active() is None:
            return 0
        return int(round(self.host_to_mcu_ns(t_host_s * 1e9) / 1000))

    @property
    def offset_ns(self):
        """MCU minus host (ns) now: protocol.TimeSync's view."""
        if self.m0 is None or self._active() is None:
            return None
        t = self.t_last_host if self.t_last_host is not None else self.h0
        return int(round(self.host_to_mcu_ns(t) - t))

    def status(self) -> dict:
        """What the link node publishes (/mcu/clock) and the console shows."""
        mode, f = self.mode, self._active()
        # drift_ppm: how fast the MCU's clock runs against the Jetson's (+ = it gains)
        d = dict(mode=mode, offset_us=None if self.offset_ns is None else round(self.offset_ns / 1000, 1),
                 drift_ppm=None if f is None else round(-f[1] / (1.0 + f[1]) * 1e6, 3),
                 rms_us=round((self.pulse_rms if mode == "pulse" else self.echo_rms) / 1000, 2) if f else None,
                 rtt_us=None if self.rtt_ns is None else round(self.rtt_ns / 1000, 1),
                 pulses=self.n_pulse_used, echoes=self.n_echo_used, rejected=self.n_rejected,
                 resets=self.n_resets, unpaired_edges=len(self.edges))
        if self.pulse_fit is not None and self.echo_fit is not None and self.t_last_host is not None:
            x = self.host_to_mcu_ns(self.t_last_host) - self.m0
            d["echo_minus_pulse_us"] = round(((self.echo_fit[0] + self.echo_fit[1] * x) -
                                              (self.pulse_fit[0] + self.pulse_fit[1] * x)) / 1000, 1)
        return d
