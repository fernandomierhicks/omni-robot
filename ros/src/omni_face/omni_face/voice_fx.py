"""The robot's voice character (ROS-free): WORLD for pitch, formants and intonation, then a pedalboard chain.

Why (Nov 21: "the current voice is subpar, sounds like the cheapest"): the old chain raised the pitch by resampling
(the formants move with it: a chipmunk / slow tape) and rang a 4 ms comb (a tin can). Here the WORLD vocoder
(pyworld) splits speech into its pitch (f0), spectral envelope (the formants: the size of the "head") and
aperiodicity (breath), so each changes on its own:
  pitch      the voice higher or lower, the formants kept (the same speaker, sung higher)
  formant    the head smaller (> 1: a small robot) or bigger (< 1), the pitch kept
  f0_range   intonation: > 1 more expressive (excited), < 1 flatter (tired, deadpan), 0 a monotone
  flat       0-1 toward one note (a classic robot's monotone); quantize 0-1 snaps it to semitones (a synthetic sheen)
  breath     < 1 cleaner, more synthetic (less breath noise); > 1 breathier
  vibrato    (Hz, depth): a wobble (a small machine's warble, a worried tremble)
  rate       the pace (> 1 quicker), the pitch kept
Then a pedalboard chain (CHAINS) gives the body: band limits, presence, saturation, a whisper of ring modulation,
chorus, a short room. EMOTION_PROSODY carries a feeling for engines that have no emotion of their own (Kokoro, Pocket
TTS): mostly the intonation range and the pace, as people do.

Runs in the voice venv (C:\\Users\\ferna\\.venvs\\omni-voice: pyworld, pedalboard, soundfile); without them, render()
returns the speech as it came.
"""

from __future__ import annotations

import numpy as np

try:
    import pyworld as pw
except ImportError:                     # the robot's own venv: no WORLD
    pw = None
try:
    import pedalboard as pb
except ImportError:
    pb = None

# emotion -> (pitch x, intonation range x, pace x, vibrato (Hz, depth)): for voices with no emotion of their own
EMOTION_PROSODY = {
    "": (1.0, 1.0, 1.0, (0.0, 0.0)), "neutral": (1.0, 1.0, 1.0, (0.0, 0.0)),
    "happy": (1.04, 1.25, 1.04, (0.0, 0.0)), "excited": (1.08, 1.5, 1.1, (0.0, 0.0)),
    "proud": (1.02, 1.2, 0.97, (0.0, 0.0)), "affection": (1.0, 1.12, 0.94, (0.0, 0.0)),
    "content": (0.99, 0.95, 0.97, (0.0, 0.0)), "relieved": (0.98, 0.9, 0.92, (0.0, 0.0)),
    "curious": (1.03, 1.3, 1.0, (0.0, 0.0)), "surprised": (1.08, 1.5, 1.06, (0.0, 0.0)),
    "startled": (1.12, 1.45, 1.14, (0.0, 0.0)), "focused": (0.99, 0.85, 1.03, (0.0, 0.0)),
    "effort": (1.0, 1.15, 0.94, (7.0, 0.012)), "worried": (1.03, 0.9, 1.06, (6.0, 0.018)),
    "frustrated": (0.97, 1.1, 1.04, (0.0, 0.0)), "sad": (0.94, 0.7, 0.88, (0.0, 0.0)),
    "tired": (0.93, 0.6, 0.84, (0.0, 0.0)), "bored": (0.95, 0.5, 0.9, (0.0, 0.0)),
}


def world(x: np.ndarray, sr: int, pitch=1.0, formant=1.0, f0_range=1.0, flat=0.0, quantize=0.0, breath=1.0,
          vibrato=(0.0, 0.0), rate=1.0, bands=0) -> np.ndarray:
    """x (mono float) re-voiced through WORLD (see the module's docstring). bands > 0: the spectral envelope
    averaged into that many log-spaced bands per frame (a vocoder's filter bank: fewer is more robotic, the melody
    stays). Unchanged without pyworld."""
    if pw is None or len(x) < sr // 20:
        return x
    x = np.ascontiguousarray(x, dtype=np.float64)
    period = 5.0
    f0, t = pw.dio(x, sr, frame_period=period, f0_floor=60.0, f0_ceil=700.0)
    f0 = pw.stonemask(x, f0, t, sr)
    sp = pw.cheaptrick(x, f0, t, sr)
    ap = pw.d4c(x, f0, t, sr)
    v = f0 > 0
    if v.any():
        lf = np.log(np.where(v, f0, 1.0))
        mean = float(np.mean(lf[v]))
        lf = mean + (lf - mean) * f0_range                       # intonation range round its mean
        lf = (1.0 - flat) * lf + flat * mean                     # toward a monotone
        if quantize > 0:                                         # snap toward semitones (a synthetic sheen)
            semi = lf / np.log(2.0) * 12.0
            lf = (semi + quantize * (np.round(semi) - semi)) * np.log(2.0) / 12.0
        f0 = np.where(v, np.exp(lf) * pitch, 0.0)
        hz, depth = vibrato
        if hz > 0 and depth > 0:
            f0 = f0 * (1.0 + depth * np.sin(2 * np.pi * hz * t))
    if abs(formant - 1.0) > 1e-3:                                # the envelope stretched along frequency
        n = sp.shape[1]
        src = np.clip(np.arange(n) / formant, 0, n - 1)
        lo = np.floor(src).astype(int)
        hi = np.minimum(lo + 1, n - 1)
        w = src - lo
        sp = sp[:, lo] * (1 - w) + sp[:, hi] * w
    if bands and bands > 0:                                      # a vocoder's filter bank
        n = sp.shape[1]
        f = np.linspace(0.0, sr / 2.0, n)
        edges = np.geomspace(80.0, sr / 2.0, int(bands) + 1)
        idx = np.clip(np.searchsorted(edges, f) - 1, 0, int(bands) - 1)
        lg = np.log(np.maximum(sp, 1e-16))
        out = np.empty_like(lg)
        for b in range(int(bands)):
            m = idx == b
            if m.any():
                out[:, m] = lg[:, m].mean(axis=1, keepdims=True)
        sp = np.exp(out)
    if abs(breath - 1.0) > 1e-3:
        ap = np.clip(ap ** (1.0 / max(0.02, breath)), 0.0, 1.0)
    y = pw.synthesize(np.ascontiguousarray(f0), np.ascontiguousarray(sp), np.ascontiguousarray(ap), sr,
                      frame_period=period / max(0.3, rate))
    return y


def ring(x, sr, hz, mix):
    """A whisper of ring modulation (the classic robot's metal) at mix 0-1."""
    if mix <= 0:
        return x
    t = np.arange(len(x)) / sr
    return (1 - mix) * x + mix * x * np.sin(2 * np.pi * hz * t)


def octave_ghost(x, sr, mix):
    """A faint copy an octave up (WORLD), for a synthetic shimmer."""
    if mix <= 0 or pw is None:
        return x
    up = world(x, sr, pitch=2.0, formant=1.0, breath=0.5)
    n = min(len(x), len(up))
    y = x.copy()
    y[:n] += mix * up[:n]
    return y


# the characters: WORLD settings (vox) then the board (fx: built by _board) and extras
CHAINS = {
    "raw": dict(vox={}, fx=[], desc="the model's voice as it comes (loudness only)"),
    "clean": dict(vox={}, fx=["hp:80", "comp", "room:0.06"],
                  desc="studio clean: a high-pass, gentle compression, a touch of room"),
    "warm": dict(vox=dict(formant=0.97), fx=["hp:70", "lowshelf:180:2", "comp", "room:0.08"],
                 desc="natural and friendly: a little warmth, a small room"),
    "tars": dict(vox=dict(formant=0.95, f0_range=0.8, breath=0.8), ring=(1100.0, 0.05),
                 fx=["hp:140", "lp:7200", "peak:2600:3", "drive:4", "comp", "plate:0.1"],
                 desc="a dry machine with a radio edge: band-limited, presence, light drive, a faint metallic ring "
                      "(no comb), flatter intonation"),
    "eve": dict(vox=dict(pitch=1.08, formant=1.1, breath=0.45, quantize=0.25), ghost=0.07,
                fx=["hp:160", "highshelf:6000:3", "chorus", "comp", "plate:0.14"],
                desc="bright and clean: higher, a smaller head, breath taken out, a hint of semitone snap and an "
                     "octave shimmer, airy"),
    "wall": dict(vox=dict(pitch=1.22, formant=1.22, f0_range=1.35, vibrato=(6.0, 0.02)),
                 fx=["hp:320", "lp:5200", "crush:11", "drive:6", "comp", "room:0.05"],
                 desc="a small curious machine: high, a tiny head, a warble, a small gritty speaker"),
    "droid": dict(vox=dict(flat=0.85, quantize=1.0, breath=0.3, formant=1.04), ring=(55.0, 0.18),
                  fx=["hp:150", "lp:6500", "comb", "comp", "room:0.04"],
                  desc="the classic robot (for contrast): monotone snapped to one note, ring-modulated metal"),
    # Nov 22 (Fernando: the human male voices with their feelings, "more robotic"): the robot is LAYERED on the voice
    # and never flattens its intonation (the feeling lives there). layer = a WORLD copy of the voice mixed in at
    # `mix`: breath taken out (a pure harmonic buzz), a little semitone snap, optionally through a coarse filter bank
    "synth_light": dict(vox={}, layer=dict(mix=0.35, breath=0.06, quantize=0.2),
                        fx=["hp:80", "comp", "chorus", "plate:0.06"],
                        desc="android, light: a third of a breathless harmonic copy blended in; mostly the actor"),
    "synth": dict(vox={}, layer=dict(mix=0.6, breath=0.04, quantize=0.35, formant=0.97),
                  fx=["hp:90", "peak:2800:2:1", "comp", "chorus", "plate:0.08"],
                  desc="android: the voice and its breathless harmonic copy half and half, a little semitone snap"),
    "synth_strong": dict(vox={}, layer=dict(mix=0.85, breath=0.02, quantize=0.55, formant=0.96),
                         fx=["hp:100", "peak:2800:3:1", "comp", "chorus", "plate:0.1"],
                         desc="android, strong: almost all synthetic, the melody snapped toward notes, the feeling kept"),
    "vocoder": dict(vox={}, layer=dict(mix=0.75, breath=0.02, quantize=0.4, bands=18),
                    fx=["hp:110", "lp:7500", "comp", "chorus", "room:0.06"],
                    desc="vocoder: re-made through an 18-band filter bank on its own melody; the classic robot "
                         "that still sounds happy or sad"),
    "metal": dict(vox={}, layer=dict(mix=0.45, breath=0.08, quantize=0.25, formant=0.94), ring=(880.0, 0.07),
                  fx=["hp:90", "peak:610:6:9", "peak:1340:5:9", "peak:2370:5:10", "peak:3900:4:10", "comp",
                      "plate:0.12"],
                  desc="big machine: a lower head, metallic body resonances, a hint of ring modulation"),
    "digital": dict(vox={}, layer=dict(mix=0.5, breath=0.06, quantize=0.6),
                    fx=["hp:120", "resample:11025", "crush:10", "comp", "room:0.04"],
                    desc="retro computer: 11 kHz, 10-bit, its melody snapped to semitones"),
    # round 3 (Nov 22, Fernando picked Chatterbox Turbo's dry voice: "needs more emotion, a little more robotic"):
    # no intonation flattening at all (the old 'tars' narrowed it to 0.8x), the android layer a little past light
    "tars_open": dict(vox=dict(formant=0.95, breath=0.8), ring=(1100.0, 0.05),
                      fx=["hp:140", "lp:7200", "peak:2600:3", "drive:4", "comp", "plate:0.1"],
                      desc="the radio-edged machine of 'tars' with its intonation left whole"),
    "robot_plus": dict(vox={}, layer=dict(mix=0.45, breath=0.04, quantize=0.3, formant=0.96), ring=(1100.0, 0.04),
                       fx=["hp:120", "lp:7500", "peak:2600:3:1", "drive:3", "comp", "chorus", "plate:0.09"],
                       desc="a little more robotic: 45 % android layer on the radio-edged machine, intonation whole"),
    "vocoder_lite": dict(vox={}, layer=dict(mix=0.5, breath=0.03, quantize=0.35, bands=22),
                         fx=["hp:110", "lp:7500", "drive:2", "comp", "chorus", "room:0.06"],
                         desc="half a soft 22-band vocoder on its own melody"),
    # round 4 (Nov 23, Fernando picked vocoder_lite: "emotion level is correct ... even more robotic synthesized"):
    # eight steps up from it, every one keeping the intonation's contour (quantize snaps it to notes, never flattens)
    "voc_1": dict(vox={}, layer=dict(mix=0.65, breath=0.03, quantize=0.4, bands=20),
                  fx=["hp:110", "lp:7500", "drive:2", "comp", "chorus", "room:0.06"],
                  desc="1 · a bit more vocoder: two thirds through a 20-band bank"),
    "voc_2": dict(vox={}, layer=dict(mix=0.8, breath=0.02, quantize=0.5, bands=18),
                  fx=["hp:120", "lp:7200", "drive:3", "comp", "chorus", "room:0.06"],
                  desc="2 · mostly vocoder: 80 % through an 18-band bank, half-snapped to notes"),
    "voc_3": dict(vox={}, layer=dict(mix=0.8, breath=0.02, quantize=0.5, bands=18, formant=0.95), ring=(880.0, 0.06),
                  fx=["hp:110", "peak:610:5:9", "peak:1340:4:9", "peak:2370:4:10", "drive:3", "comp", "plate:0.1"],
                  desc="3 · vocoder in a metal body: step 2 with metal resonances and a hint of ring modulation"),
    "voc_4": dict(vox={}, layer=dict(mix=0.85, breath=0.02, quantize=0.55, bands=18), ghost=0.12, ring=(1500.0, 0.06),
                  fx=["hp:130", "highshelf:5500:4", "drive:3", "comp", "chorus", "plate:0.1"],
                  desc="4 · chrome: an octave-up synthetic shimmer and a bright metallic edge on the vocoder"),
    "voc_5": dict(vox={}, layer=dict(mix=0.85, breath=0.02, quantize=0.6, bands=18),
                  fx=["hp:120", "resample:16000", "crush:12", "drive:3", "comp", "chorus", "room:0.05"],
                  desc="5 · digital: step 2 a little lo-fi (16 kHz, 12-bit), more snapped to notes"),
    "voc_6": dict(vox={}, layer=dict(mix=0.97, breath=0.02, quantize=0.6, bands=16),
                  fx=["hp:120", "lp:7000", "drive:4", "comp", "chorus", "room:0.06"],
                  desc="6 · full vocoder: almost all synthetic, 16 bands"),
    "voc_7": dict(vox={}, layer=dict(mix=1.0, breath=0.01, quantize=0.75, bands=14), ghost=0.08, ring=(880.0, 0.06),
                  fx=["hp:120", "peak:1340:4:9", "peak:2370:4:10", "drive:4", "comp", "chorus", "plate:0.1"],
                  desc="7 · synthesized machine: pure vocoder (14 bands), strongly snapped to notes, metal and shimmer"),
    "voc_8": dict(vox={}, layer=dict(mix=1.0, breath=0.01, quantize=0.9, bands=12, formant=0.95), ghost=0.1,
                  ring=(60.0, 0.1),
                  fx=["hp:120", "peak:610:5:9", "peak:1340:5:9", "peak:2370:4:10", "drive:5", "comp", "chorus",
                      "plate:0.12"],
                  desc="8 · maximum robot: 12 bands, melody locked to notes (its rise and fall kept), low ring "
                       "modulation, metal body"),
    "metal_lite": dict(vox={}, layer=dict(mix=0.35, breath=0.08, quantize=0.2, formant=0.95), ring=(880.0, 0.05),
                       fx=["hp:100", "peak:610:4:9", "peak:1340:4:9", "peak:2370:3:10", "drive:2", "comp",
                           "plate:0.1"],
                       desc="a lighter big machine: metal resonances, a hint of ring modulation"),
}


def _board(spec: list, sr: int):
    """pedalboard effects from short specs: hp:Hz lp:Hz peak:Hz:dB lowshelf:Hz:dB highshelf:Hz:dB comp drive:dB
    crush:bits chorus room:wet plate:wet comb (a faint 1.6 ms comb)."""
    out = []
    for s in spec:
        k, *a = s.split(":")
        a = [float(v) for v in a]
        if k == "hp":
            out.append(pb.HighpassFilter(cutoff_frequency_hz=a[0]))
        elif k == "lp":
            out.append(pb.LowpassFilter(cutoff_frequency_hz=a[0]))
        elif k == "peak":                               # peak:Hz:dB[:Q]
            out.append(pb.PeakFilter(cutoff_frequency_hz=a[0], gain_db=a[1], q=a[2] if len(a) > 2 else 0.9))
        elif k == "resample":
            out.append(pb.Resample(target_sample_rate=a[0]))
        elif k == "lowshelf":
            out.append(pb.LowShelfFilter(cutoff_frequency_hz=a[0], gain_db=a[1]))
        elif k == "highshelf":
            out.append(pb.HighShelfFilter(cutoff_frequency_hz=a[0], gain_db=a[1]))
        elif k == "comp":
            out.append(pb.Compressor(threshold_db=-20, ratio=3.0, attack_ms=4, release_ms=80))
        elif k == "drive":
            out.append(pb.Distortion(drive_db=a[0]))
        elif k == "crush":
            out.append(pb.Bitcrush(bit_depth=a[0]))
        elif k == "chorus":
            out.append(pb.Chorus(rate_hz=0.8, depth=0.12, centre_delay_ms=9.0, feedback=0.0, mix=0.22))
        elif k == "room":
            out.append(pb.Reverb(room_size=0.12, damping=0.6, wet_level=a[0], dry_level=1.0, width=0.6))
        elif k == "plate":
            out.append(pb.Reverb(room_size=0.3, damping=0.35, wet_level=a[0], dry_level=1.0, width=0.8))
        elif k == "comb":
            out.append(pb.Delay(delay_seconds=0.0016, feedback=0.25, mix=0.12))
    return pb.Pedalboard(out)


def render(x: np.ndarray, sr: int, chain: str = "clean", emotion: str = "", prosody: bool | float = True,
           extra_vox: dict | None = None) -> np.ndarray:
    """Speech x (mono float, any level) -> the character `chain` (CHAINS), with the emotion's prosody (prosody=False:
    the engine voiced the feeling itself; only the character then; a number 0-1: that share of it, a boost on top of
    an engine's own feeling). Peak-normalized to -1 dBFS."""
    c = CHAINS.get(chain, CHAINS["clean"])
    vox = dict(c.get("vox") or {})
    if extra_vox:
        vox.update(extra_vox)
    if prosody:
        g = 1.0 if prosody is True else float(prosody)
        p, rng, pace, vib = EMOTION_PROSODY.get(emotion, EMOTION_PROSODY[""])
        p, rng, pace = 1 + (p - 1) * g, 1 + (rng - 1) * g, 1 + (pace - 1) * g
        vib = (vib[0], vib[1] * g)
        vox["pitch"] = vox.get("pitch", 1.0) * p
        vox["f0_range"] = vox.get("f0_range", 1.0) * rng
        vox["rate"] = vox.get("rate", 1.0) * pace
        if vib[0] > 0 and not vox.get("vibrato"):
            vox["vibrato"] = vib
    y = np.asarray(x, np.float64)
    if vox:
        y = world(y, sr, **vox)
    if c.get("layer") and pw is not None:                      # the robot layered on, the intonation kept
        lay = dict(c["layer"])
        mix = float(lay.pop("mix", 0.5))
        z = world(y, sr, **lay)
        n = min(len(y), len(z))
        pz, py = np.abs(z[:n]).max() or 1.0, np.abs(y[:n]).max() or 1.0
        y = (1.0 - mix) * y[:n] / py + mix * z[:n] / pz
    if c.get("ghost"):
        y = octave_ghost(y, sr, c["ghost"])
    if c.get("ring"):
        y = ring(y, sr, *c["ring"])
    if pb is not None and c["fx"]:
        y = _board(c["fx"], sr)(y.astype(np.float32)[None, :], sr)[0].astype(np.float64)
    peak = float(np.max(np.abs(y))) if len(y) else 0.0
    return y * (0.89 / peak) if peak > 1e-6 else y
