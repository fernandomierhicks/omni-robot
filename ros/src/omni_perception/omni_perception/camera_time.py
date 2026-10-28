"""Camera frame timestamps from the MCU's trigger (design.md 10). No ROS here; the camera node and the tests use it.

The two OV9782s run in external-trigger mode: the Teensy pulses both FSIN pins from one hardware timer (120 Hz) and
reports each pulse as a TIME event (seq, MCU time), which the link maps onto the Jetson's clock (clock_sync). The
left camera's STROBE output (high while it exposes) goes back to a Teensy capture pin, so the exposure's real start
and length are measured in the same clock (TIME events of kind STROBE).

What a frame's stamp means here: the **middle of its exposure**, on the Jetson's clock = trigger time + the sensor's
trigger-to-exposure delay + half the exposure. Both cameras get the same stamp (one trigger), which is what makes them
a stereo pair; their real skew (each sensor's own trigger response) is a calibration item measured on the bench with
the strobes (VERIFY: OV9782 FSIN response; a few line times).

What the Jetson's capture gives: V4L2 hands each buffer with a sequence number and a timestamp the VI took at the
frame's start of readout (SOF, CLOCK_MONOTONIC; turned into ROS time by the camera node). SOF comes a fixed d after
the trigger (delay + exposure + the readout's start), plus capture jitter. FrameStamper pairs every frame with its
trigger: first by finding the sequence offset whose SOF - trigger times agree (lock), then by sequence number, with
d's residual checked on every frame (a dropped trigger, a dropped frame or a camera restart breaks the lock: it
re-locks). Frames that can't be paired keep a fallback stamp from SOF (SOF - d), marked as such.

Without triggers (free-running, a USB camera): stamp = SOF - sof_to_mid (a per-camera constant from the bench).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class Stamp:
    t: float                  # s, the middle of the exposure on the Jetson's clock
    trig_seq: int | None      # the trigger it came from (None: not paired)
    source: str               # "trigger" | "sof" (fallback) | "free" (no triggers)
    d_residual: float = 0.0   # s, this frame's SOF - trigger against the running median (capture jitter)


class FrameStamper:
    """One per camera. exposure_s / trig_to_exp_s: the configured exposure and the sensor's delay from the trigger
    edge to the exposure start (VERIFY on the bench; updated online from the strobe when it is wired)."""

    def __init__(self, exposure_s: float = 0.003, trig_to_exp_s: float = 20e-6, period_s: float = 1 / 120,
                 d_tol_s: float = 300e-6, lock_n: int = 12, sof_to_mid_s: float | None = None,
                 readout_lag_s: float = 0.2e-3):
        self.exposure, self.trig_to_exp, self.period = exposure_s, trig_to_exp_s, period_s
        self.readout_lag = readout_lag_s               # exposure end -> the VI's SOF stamp (est; locks the pairing)
        self.d_tol, self.lock_n = d_tol_s, lock_n
        self.sof_to_mid = sof_to_mid_s
        self.triggers: deque = deque(maxlen=256)       # (seq, t host s)
        self.trig_by_seq: dict = {}
        self.frames: deque = deque(maxlen=64)          # (frame seq, t_sof) waiting for a lock
        self.offset = None                             # frame seq - trigger seq, once locked
        self.d_hist: deque = deque(maxlen=200)         # SOF - trigger of paired frames
        self.n_paired = self.n_fallback = self.n_relock = self.n_bad = 0
        self._bad_streak = 0

    # -- inputs ----------------------------------------------------------------------------------------------
    def on_trigger(self, seq: int, t_host: float):
        self.triggers.append((seq, t_host))
        self.trig_by_seq[seq] = t_host
        while len(self.trig_by_seq) > 256:
            self.trig_by_seq.pop(next(iter(self.trig_by_seq)))

    def on_strobe(self, rise_host: float, width_s: float, trig_t_host: float | None):
        """The left camera's measured exposure: start (Jetson clock) and length; with its trigger's time, the delay."""
        if width_s > 0:
            self.exposure = 0.9 * self.exposure + 0.1 * width_s
        if trig_t_host is not None and 0 <= rise_host - trig_t_host < 0.005:
            self.trig_to_exp = 0.9 * self.trig_to_exp + 0.1 * (rise_host - trig_t_host)

    @property
    def d(self) -> float | None:
        return float(np.median(self.d_hist)) if self.d_hist else None

    def mid_from_trigger(self, t_trig: float) -> float:
        return t_trig + self.trig_to_exp + 0.5 * self.exposure

    # -- a frame ---------------------------------------------------------------------------------------------
    def stamp(self, frame_seq: int, t_sof: float) -> Stamp:
        if not self.triggers:
            mid = t_sof - (self.sof_to_mid if self.sof_to_mid is not None else 0.5 * self.exposure)
            return Stamp(mid, None, "free")
        if self.offset is None:
            self.frames.append((frame_seq, t_sof))
            self._try_lock()
        if self.offset is not None:
            ts = frame_seq - self.offset
            t_trig = self.trig_by_seq.get(ts)
            if t_trig is not None:
                dd = t_sof - t_trig
                d0 = self.d
                res = 0.0 if d0 is None else dd - d0
                if abs(res) <= self.d_tol:
                    self._bad_streak = 0
                    self.d_hist.append(dd)
                    self.n_paired += 1
                    return Stamp(self.mid_from_trigger(t_trig), ts, "trigger", res)
                self.n_bad += 1
                self._bad_streak += 1
                if self._bad_streak >= 3:              # a trigger or a frame went missing: the offset moved
                    self.offset, self._bad_streak = None, 0
                    self.n_relock += 1
                    self.frames.clear()
        # fallback: the nearest trigger before SOF by the running delay, if one is close; else SOF - d
        self.n_fallback += 1
        d0 = self.d
        if d0 is not None:
            best = min(self.triggers, key=lambda s: abs(t_sof - s[1] - d0))
            if abs(t_sof - best[1] - d0) <= self.d_tol:
                return Stamp(self.mid_from_trigger(best[1]), best[0], "sof", t_sof - best[1] - d0)
            return Stamp(t_sof - d0 + self.trig_to_exp + 0.5 * self.exposure, None, "sof")
        return Stamp(t_sof - 0.5 * self.exposure, None, "sof")

    def _try_lock(self):
        """The sequence offset whose SOF - trigger times agree best over the last lock_n frames."""
        if len(self.frames) < self.lock_n:
            return
        fr = list(self.frames)[-self.lock_n:]
        cands = {}
        for fseq, t_sof in fr:
            for tseq, t_trig in self.triggers:
                dd = t_sof - t_trig
                if 0.0 < dd < 0.05:                    # SOF follows its trigger by < 50 ms
                    cands.setdefault(fseq - tseq, []).append(dd)
        # every offset one period apart fits equally well (the delays just shift by a period): among the consistent
        # ones, take the one whose delay is nearest the expected exposure + readout start
        d_prior = self.trig_to_exp + self.exposure + self.readout_lag
        best, best_e = None, math.inf
        for off, dds in cands.items():
            if len(dds) < 0.8 * len(fr) or float(np.std(dds)) >= self.d_tol / 2:
                continue
            e = abs(float(np.mean(dds)) - d_prior)
            if e < best_e:
                best, best_e = off, e
        if best is not None and best_e < 0.4 * self.period:
            self.offset = best
            dds = cands[best]
            self.d_hist.extend(dds)

    def report(self) -> dict:
        d = np.array(self.d_hist) if self.d_hist else np.zeros(1)
        return dict(locked=self.offset is not None, paired=self.n_paired, fallback=self.n_fallback,
                    relocks=self.n_relock, bad=self.n_bad, sof_delay_ms=round(float(np.median(d)) * 1e3, 4),
                    sof_jitter_us=round(float(np.std(d)) * 1e6, 2), exposure_ms=round(self.exposure * 1e3, 3),
                    trig_to_exp_us=round(self.trig_to_exp * 1e6, 1))


def stereo_skew(left: FrameStamper, right: FrameStamper) -> float | None:
    """The two cameras' SOF delays apart (s): their readouts' skew (their exposures share the trigger)."""
    if left.d is None or right.d is None:
        return None
    return right.d - left.d
