"""The robot's sound effects, synthesized (no files to ship or license): short, friendly, and quieter than its voice.

    sound("catch", rate=22050) -> int16 mono samples
Names: hello, catch, miss, launch, whoosh, alert (a fault), warn, beep; the antics (moments.ANTICS): yawn, snore,
giggle, sneeze, hic, babble, aww, woozy, whistle, sigh, boing, sparkle; the voice commands: listen (it heard its
name), heard (a command understood), huh (not understood).
Songs (moments.ANTICS "sing" / "hum" / "dance"), 3-7 s: twinkle, jacques, ode, jingle (sung on "la", a vowel-ish
voice), hum. All are public-domain tunes or the robot's own.
"""

from __future__ import annotations

import numpy as np


def _env(n, rate, attack=0.01, release=0.08):
    e = np.ones(n)
    a, r = (min(max(1, int(x * rate)), max(1, n // 2)) for x in (attack, release))     # short notes: shorter fades
    e[:a] = np.linspace(0, 1, a)
    e[-r:] *= np.linspace(1, 0, r)
    return e


def _tone(f, dur, rate, kind="sine", vib=0.0):
    t = np.arange(int(dur * rate)) / rate
    f = np.broadcast_to(np.asarray(f, float), t.shape) if np.ndim(f) else np.full(t.shape, float(f))
    ph = 2 * np.pi * np.cumsum(f * (1 + vib * np.sin(2 * np.pi * 6 * t))) / rate
    if kind == "square":
        w = np.sign(np.sin(ph)) * 0.6
    elif kind == "tri":
        w = 2 / np.pi * np.arcsin(np.sin(ph))
    else:
        w = np.sin(ph) + 0.2 * np.sin(2 * ph)
    return w * _env(len(t), rate)


def _seq(notes, rate, kind="sine", gap=0.01):
    parts = []
    for f, d in notes:
        parts += [_tone(f, d, rate, kind), np.zeros(int(gap * rate))]
    return np.concatenate(parts)


def _sweep(f0, f1, dur, rate, kind="sine"):
    n = int(dur * rate)
    return _tone(np.geomspace(f0, f1, n), dur, rate, kind)


def _noise(dur, rate, smooth=40, seed=0):
    n = int(dur * rate)
    return np.convolve(np.random.default_rng(seed).normal(0, 1, n), np.ones(smooth) / smooth, "same")


def _voice(f, dur, rate, vowel="a", vib=0.012, attack=0.03, release=0.08):
    """A sung vowel: harmonics of f weighted by two formants (a = "ah", o = "oh", u = "oo", i = "ee"), vibrato."""
    F = {"a": (750, 1200), "o": (500, 900), "u": (330, 800), "i": (300, 2200), "e": (500, 1800)}[vowel]
    n = int(dur * rate)
    t = np.arange(n) / rate
    f = np.broadcast_to(np.asarray(f, float), t.shape) if np.ndim(f) else np.full(t.shape, float(f))
    ph = 2 * np.pi * np.cumsum(f * (1 + vib * np.sin(2 * np.pi * 5.5 * t))) / rate
    w = np.zeros(n)
    f0 = float(np.mean(f))
    for k in range(1, 16):
        fk = k * f0
        if fk > rate / 2.2:
            break
        a = np.exp(-((fk - F[0]) / 160) ** 2) + 0.6 * np.exp(-((fk - F[1]) / 220) ** 2) + 0.25 / k
        w += a * np.sin(k * ph)
    return w * _env(n, rate, attack, release)


def _la(notes, rate, vowel="a", gap=0.025):
    """A tune sung on "la": each note a vowel with a soft start (the l), [(Hz or 0 = rest, s), ...]."""
    parts = []
    for f, d in notes:
        if f <= 0:
            parts.append(np.zeros(int(d * rate)))
            continue
        v = _voice(f, d - gap, rate, vowel)
        k = min(len(v), int(0.04 * rate))
        v[:k] *= np.linspace(0.25, 1.0, k)                        # "l"
        parts += [v, np.zeros(int(gap * rate))]
    return np.concatenate(parts)


def _normal(w, volume):
    w = w / max(1e-9, float(np.max(np.abs(w)))) * volume
    return (w * 32767).astype(np.int16)


# note frequencies (Hz), equal temperament from A4 = 440
_N = {n: 440.0 * 2 ** ((i - 9) / 12) for i, n in enumerate(("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"))}


def _hz(name: str) -> float:
    """"C5", "G4", "-" (a rest) -> Hz (0 for a rest)."""
    if name == "-":
        return 0.0
    return _N[name[:-1]] * 2 ** (int(name[-1]) - 4)


def _tune(text: str, beat: float):
    """"C5 C5 G5:2 -" -> [(Hz, s)]: a note per word, ":k" = k beats."""
    out = []
    for w in text.split():
        n, _, k = w.partition(":")
        out.append((_hz(n), beat * (float(k) if k else 1.0)))
    return out


SONG_NOTES = {
    # Twinkle, Twinkle, Little Star (traditional), the first line
    "twinkle": ("C5 C5 G5 G5 A5 A5 G5:2 F5 F5 E5 E5 D5 D5 C5:2", 0.26, "a"),
    # Frere Jacques (traditional), the first two lines
    "jacques": ("F4 G4 A4 F4 F4 G4 A4 F4 A4 A#4 C5:2 A4 A#4 C5:2", 0.27, "o"),
    # Ode to Joy (Beethoven), the first phrase
    "ode": ("E5 E5 F5 G5 G5 F5 E5 D5 C5 C5 D5 E5 E5:1.5 D5:0.5 D5:2", 0.25, "a"),
    # the robot's own jingle
    "jingle": ("G5:0.5 C6:0.5 B5:0.5 G5:0.5 E5 - E5:0.5 F5:0.5 G5:0.5 A5:0.5 G5:2", 0.24, "i"),
    # a content hum (mouth closed: "oo"), quieter
    "hum": ("E4 G4:0.5 A4:0.5 G4 E4 D4:2 E4 G4:2", 0.3, "u"),
}
SONGS = tuple(SONG_NOTES)


def song_seconds(name: str) -> float:
    notes, beat, _ = SONG_NOTES[name]
    return sum(d for _, d in _tune(notes, beat))


def song(name: str, rate: int = 22050, volume: float = 0.5) -> np.ndarray:
    notes, beat, vowel = SONG_NOTES[name]
    w = _la(_tune(notes, beat), rate, vowel)
    return _normal(w, volume * (0.7 if name == "hum" else 1.0))


def sound(name: str, rate: int = 22050, volume: float = 0.5) -> np.ndarray:
    c5, e5, g5, c6 = 523.3, 659.3, 784.0, 1046.5
    if name in SONG_NOTES:
        return song(name, rate, volume)
    if name == "hello":
        w = _seq([(g5, 0.09), (c6, 0.16)], rate)
    elif name == "catch":
        w = _seq([(c5, 0.07), (e5, 0.07), (g5, 0.07), (c6, 0.2)], rate, "tri")
    elif name == "miss":
        w = np.concatenate([_sweep(440, 300, 0.22, rate, "tri"), _sweep(300, 180, 0.35, rate, "tri")])
    elif name == "launch":
        w = _sweep(200, 1400, 0.35, rate, "tri")
    elif name == "whoosh":
        n = int(0.4 * rate)
        w = _noise(0.4, rate) * _env(n, rate, 0.15, 0.2) * 1.5
    elif name == "alert":
        w = _seq([(880, 0.14), (660, 0.14), (880, 0.14), (660, 0.2)], rate, "square", gap=0.03)
    elif name == "warn":
        w = _seq([(660, 0.12), (520, 0.2)], rate, "square", gap=0.04)
    elif name == "yawn":                             # "aaah-ohh": up a little, then a long slide down
        f = np.concatenate([np.geomspace(330, 420, int(0.35 * rate)), np.geomspace(420, 190, int(1.0 * rate))])
        v = np.concatenate([_voice(f[: int(0.35 * rate)], 0.35, rate, "a", 0.0, 0.15, 0.01),
                            _voice(f[int(0.35 * rate):], 1.0, rate, "o", 0.02, 0.01, 0.35)])
        w = v + 0.15 * _noise(len(v) / rate, rate, 20, 3) * _env(len(v), rate, 0.2, 0.4)
    elif name == "snore":                            # a low buzzy breath in, a whistle out
        n = int(0.7 * rate)
        t = np.arange(n) / rate
        buzz = _noise(0.7, rate, 8, 1) * (0.6 + 0.4 * np.sign(np.sin(2 * np.pi * 38 * t))) * _env(n, rate, 0.3, 0.2)
        w = np.concatenate([buzz, np.zeros(int(0.1 * rate)), 0.5 * _sweep(900, 600, 0.45, rate)])
    elif name == "giggle":                           # "hehehe": quick falling chirps
        parts = []
        for k, f in enumerate((900, 860, 800, 760, 700)):
            parts += [_voice(np.geomspace(f * 1.15, f, int(0.07 * rate)), 0.07, rate, "e", 0.0, 0.005, 0.03),
                      np.zeros(int(0.045 * rate))]
        w = np.concatenate(parts)
    elif name == "sneeze":                           # "ah... ah... CHOO!"
        ah = lambda f, d: _voice(np.geomspace(f, f * 1.2, int(d * rate)), d, rate, "a", 0.0, 0.05, 0.05)  # noqa: E731
        choo = _noise(0.32, rate, 3, 2) * _env(int(0.32 * rate), rate, 0.005, 0.25) * 1.6
        w = np.concatenate([ah(380, 0.3), np.zeros(int(0.12 * rate)), ah(440, 0.35), np.zeros(int(0.08 * rate)),
                            choo + 0.4 * _voice(260, 0.32, rate, "u", 0.0, 0.005, 0.25)])
    elif name == "hic":
        w = np.concatenate([_sweep(500, 1300, 0.06, rate, "tri"), np.zeros(int(0.02 * rate)),
                            0.5 * _tone(1300, 0.04, rate)])
    elif name == "babble":                           # R2-style beeps and boops
        rng = np.random.default_rng(7)
        parts = []
        for _ in range(9):
            f0, f1, d = rng.uniform(600, 2400), rng.uniform(600, 2400), rng.uniform(0.04, 0.11)
            parts += [_sweep(f0, f1, d, rate, "sine"), np.zeros(int(rng.uniform(0.01, 0.05) * rate))]
        w = np.concatenate(parts)
    elif name == "aww":                              # "awww": a warm slide down
        w = _voice(np.geomspace(620, 420, int(0.7 * rate)), 0.7, rate, "o", 0.02, 0.05, 0.3)
    elif name == "woozy":                            # dizzy: a wobbling slide
        n = int(1.1 * rate)
        t = np.arange(n) / rate
        w = _tone(500 * (1 + 0.25 * np.sin(2 * np.pi * 3.0 * t)) * np.geomspace(1.2, 0.7, n), 1.1, rate, "tri")
    elif name == "whistle":                          # two notes, the second sliding down
        w = np.concatenate([_tone(np.geomspace(1400, 2100, int(0.25 * rate)), 0.25, rate, vib=0.01),
                            np.zeros(int(0.06 * rate)),
                            _tone(np.geomspace(2100, 1300, int(0.5 * rate)), 0.5, rate, vib=0.015)])
    elif name == "sigh":
        n = int(0.9 * rate)
        w = _noise(0.9, rate, 25, 4) * _env(n, rate, 0.25, 0.5) * 0.8 + \
            0.3 * _voice(np.geomspace(300, 200, n), 0.9, rate, "a", 0.0, 0.25, 0.5)
    elif name == "boing":
        n = int(0.5 * rate)
        t = np.arange(n) / rate
        w = _tone(220 * (1 + 0.5 * np.exp(-t * 6) * np.sin(2 * np.pi * 14 * t)), 0.5, rate, "tri")
    elif name == "sparkle":                          # a wink's twinkle
        w = _seq([(1568, 0.05), (2093, 0.05), (2637, 0.12)], rate, gap=0.005)
    elif name == "listen":                           # it heard its name: a soft rising pair
        w = _seq([(e5, 0.07), (c6, 0.12)], rate)
    elif name == "heard":                            # a command understood
        w = _seq([(g5, 0.06), (c6, 0.06), (g5 * 2, 0.1)], rate)
    elif name == "huh":                              # not understood: a questioning rise
        w = _voice(np.geomspace(300, 520, int(0.35 * rate)), 0.35, rate, "a", 0.0, 0.03, 0.08)
    elif name == "scan":                             # patrol: a radar sweep out and back
        w = 0.6 * np.concatenate([_sweep(260, 760, 0.45, rate, "tri"), _sweep(760, 260, 0.45, rate, "tri")])
    elif name == "tada":                             # tidy: one more in the bin
        w = _seq([(g5, 0.07), (c6, 0.07), (e5 * 2, 0.18)], rate, "tri")
    else:                                            # "beep"
        w = _tone(1000, 0.08, rate)
    return _normal(w, volume)


NAMES = ("hello", "catch", "miss", "launch", "whoosh", "alert", "warn", "beep", "yawn", "snore", "giggle", "sneeze",
         "hic", "babble", "aww", "woozy", "whistle", "sigh", "boing", "sparkle", "listen", "heard", "huh", "scan", "tada")
