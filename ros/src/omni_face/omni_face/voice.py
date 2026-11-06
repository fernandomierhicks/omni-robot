"""Text to speech for the robot's voice: text -> a WAV file (cached), played by the display (ROS-free).

Backends, the first that works is used (choose()):
  kokoro   Kokoro-82M (Apache 2.0), an offline neural voice through onnxruntime: the most natural of these, on the
           PC and the robot. `pip install kokoro-onnx` and its two model files in ~/.local/share/kokoro
           (`python voice.py fetch`, 120 MB; OMNI_KOKORO_DIR elsewhere). Its voice comes from the preset (a blend of
           its stock voices); OMNI_KOKORO_VOICE forces one. PC: ~0.2 x real time on the CPU (measured Oct 26); the
           robot computer's CPU VERIFY (est 0.5-1 x: phrases are cached)
  piper    Piper, an offline neural voice that runs on the CPU. `pip install piper-tts` (or its binary) and a voice
           model (.onnx + .onnx.json, e.g. en_US-amy-medium); OMNI_PIPER_MODEL points at it, else the first one in
           ~/.local/share/piper (VERIFY its speed on the robot computer: est 0.1-0.3 x real time)
  espeak   espeak-ng: tiny, robotic, on every Linux (`sudo apt install espeak-ng`)
  sapi     Windows' own voice (System.Speech through one PowerShell process kept open)
  none     no voice: the display still shows what it would have said

Presets (PRESETS; the console's Face tab "Voice", OMNI_VOICE_PRESET before the node says): who the robot sounds like.
  deadpan  the default (Fernando, Nov 5: "something like TARS"): a calm, low, dry male voice (Kokoro am_michael with
           some am_fenrir), a touch slower and lower, through a small "speaker in a metal box" (band-limited, a faint
           metallic ring, a short reflection): a machine that talks well, not a cartoon
  bright   clean, light and warm (EVE-like): af_bella with af_heart, a little higher, a faint shimmer (a detuned double)
  cute     the voice before Nov 5: af_heart raised 12% (a small speaker), no effects
The pitch is raised or lowered by resampling the WAV (which moves the formants too: a smaller or bigger speaker);
Kokoro speaks that much slower or quicker first so the pace stays. Each persona (moments.PERSONAS) speaks in its own
style on top (STYLES: patrol lower and slower, explore and tidy a little brighter and quicker). Each phrase is made
once per preset and style and kept in ~/.cache/omni_face/tts/<backend>/ (lines repeat: after the first time they play
at once). `python voice.py audition` writes every preset saying the same lines, to listen and choose.
"""

from __future__ import annotations

import glob
import hashlib
import os
import shutil
import subprocess
import sys
import threading
import urllib.request
import wave
from pathlib import Path

import numpy as np

CACHE = Path(os.environ.get("OMNI_FACE_CACHE", Path.home() / ".cache" / "omni_face")) / "tts"
POWERSHELL = r"C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe"
KOKORO_DIR = Path(os.environ.get("OMNI_KOKORO_DIR", Path.home() / ".local" / "share" / "kokoro"))
KOKORO_URL = "https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.0/"
KOKORO_FILES = ("kokoro-v1.0.int8.onnx", "voices-v1.0.bin")
OUT_RATE = 22050                   # the display's mixer
# preset -> kokoro: its voices blended (name, weight); sapi: a Windows voice (a name pattern); pitch (x, 1 = as made);
# pace (x); fx: the effect after (FX)
PRESETS = {
    "deadpan": dict(kokoro=(("am_michael", 0.65), ("am_fenrir", 0.35)), sapi="David|Guy|Mark", pitch=0.95, pace=0.94,
                    fx="box"),
    "bright": dict(kokoro=(("af_bella", 0.6), ("af_heart", 0.4)), sapi="Aria|Jenny|Zira", pitch=1.05, pace=1.02,
                   fx="shimmer"),
    "cute": dict(kokoro=(("af_heart", 1.0),), sapi="Zira|Aria|Jenny", pitch=1.12, pace=1.0, fx=""),
}
PRESET = os.environ.get("OMNI_VOICE_PRESET", "deadpan")
# a persona's change to the preset's pitch (x) and pace (x)
STYLES = {"": (1.0, 1.0), "patrol": (0.93, 0.95), "explore": (1.03, 1.05), "tidy": (1.02, 1.04),
          "catch": (1.02, 1.03)}


class Backend:
    name = "none"
    key = "none"                   # what makes its WAVs differ (the cache key)

    def synth(self, text: str, out: Path, rate: float = 1.0, preset: dict | None = None) -> bool:
        """Write text spoken to out (a WAV). rate: the pace asked for; preset: PRESETS[...] (its voice). A backend
        that can't do either ignores it."""
        return False


class Kokoro(Backend):
    name = "kokoro"

    def __init__(self, model: str, voices: str, voice: str | None = None):
        self.model, self.voices = model, voices
        self.voice = voice or os.environ.get("OMNI_KOKORO_VOICE")      # one voice for every preset
        self.key = f"kokoro-{Path(model).stem}" + (f"-{self.voice}" if self.voice else "")
        self.k = None
        self.lock = threading.Lock()

    def style(self, preset: dict | None):
        """The voice: a stock one, or the preset's blend (a weighted mean of the voices' style vectors)."""
        mix = ((self.voice, 1.0),) if self.voice else (preset or PRESETS["cute"])["kokoro"]
        if len(mix) == 1:
            return mix[0][0]
        w = sum(x for _, x in mix)
        return sum(self.k.get_voice_style(n) * (x / w) for n, x in mix)

    def synth(self, text, out, rate=1.0, preset=None):
        with self.lock:
            if self.k is None:
                from kokoro_onnx import Kokoro as K
                self.k = K(self.model, self.voices)
            a, sr = self.k.create(text, voice=self.style(preset), speed=max(0.5, min(2.0, rate)), lang="en-us")
        write_wav(out, np.asarray(a, float), sr)
        return out.exists()


class Piper(Backend):
    name = "piper"

    def __init__(self, model: str):
        self.model = model
        self.key = f"piper-{Path(model).stem}"
        self.cmd = shutil.which("piper") or [sys.executable, "-m", "piper"]

    def synth(self, text, out, rate=1.0, preset=None):
        cmd = ([self.cmd] if isinstance(self.cmd, str) else self.cmd) + [
            "--model", self.model, "--output_file", str(out), "--length_scale", f"{1.0 / rate:.3f}"]
        r = subprocess.run(cmd, input=text.encode(), capture_output=True, timeout=30)
        return r.returncode == 0 and out.exists()


class Espeak(Backend):
    name = key = "espeak"

    def __init__(self, exe: str):
        self.exe = exe

    def synth(self, text, out, rate=1.0, preset=None):
        r = subprocess.run([self.exe, "-s", str(int(160 * rate)), "-w", str(out), text], capture_output=True, timeout=30)
        return r.returncode == 0 and out.exists()


class Sapi(Backend):
    """One PowerShell kept open: each line in is "<wav path>\\t<voice pattern>\\t<text>", each line out "ok" when the
    file is written. The voice: OMNI_SAPI_VOICE (or the one given), else the first installed that matches the
    preset's pattern."""
    name = "sapi"
    SCRIPT = (
        "Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
        "$names = $s.GetInstalledVoices() | ForEach-Object { $_.VoiceInfo.Name }; $s.Rate = 1; "
        "while ($true) { $l = [Console]::In.ReadLine(); if ($l -eq $null) { break }; $p = $l.Split(\"`t\", 3); "
        "try { if ($env:OMNI_SAPI_VOICE) { $s.SelectVoice($env:OMNI_SAPI_VOICE) } "
        "else { $v = $names | Where-Object { $_ -match $p[1] } | Select-Object -First 1; if ($v) { $s.SelectVoice($v) } }; "
        "$s.SetOutputToWaveFile($p[0]); $s.Speak($p[2]); $s.SetOutputToNull(); [Console]::Out.WriteLine('ok') } "
        "catch { [Console]::Out.WriteLine('err') }; [Console]::Out.Flush() }")

    def __init__(self, voice: str | None = None):
        self.lock = threading.Lock()
        self.proc = None
        self.voice = voice or os.environ.get("OMNI_SAPI_VOICE", "")
        self.key = f"sapi-{self.voice}" if self.voice else "sapi"

    def _start(self):
        self.proc = subprocess.Popen([POWERSHELL, "-NoProfile", "-NonInteractive", "-Command", self.SCRIPT],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                     text=True, env={**os.environ, "OMNI_SAPI_VOICE": self.voice},
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

    def synth(self, text, out, rate=1.0, preset=None):
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                self._start()
            line = text.replace("\t", " ").replace("\n", " ")
            pattern = (preset or {}).get("sapi") or "Zira|Aria|Jenny"
            self.proc.stdin.write(f"{out}\t{pattern}\t{line}\n")
            self.proc.stdin.flush()
            return self.proc.stdout.readline().strip() == "ok" and out.exists()


def kokoro_files(d: Path = KOKORO_DIR):
    """(model, voices) in d, the full model before the int8 one; None if either is missing."""
    models = sorted(glob.glob(str(d / "kokoro-v1.0*.onnx")), key=lambda p: "int8" in p)
    voices = d / "voices-v1.0.bin"
    return (models[0], str(voices)) if models and voices.exists() else None


def choose(prefer: str | None = None) -> Backend:
    prefer = prefer or os.environ.get("OMNI_FACE_VOICE")
    if prefer == "none":
        return Backend()
    kok = kokoro_files()
    if prefer in (None, "kokoro") and kok and _has_module("kokoro_onnx"):
        return Kokoro(*kok)
    model = os.environ.get("OMNI_PIPER_MODEL") or next(iter(sorted(glob.glob(
        str(Path.home() / ".local" / "share" / "piper" / "*.onnx")))), None)
    if prefer in (None, "piper") and model and (shutil.which("piper") or _has_module("piper")):
        return Piper(model)
    exe = shutil.which("espeak-ng") or shutil.which("espeak")
    if prefer in (None, "espeak") and exe:
        return Espeak(exe)
    if prefer in (None, "sapi") and sys.platform == "win32" and os.path.exists(POWERSHELL):
        return Sapi()
    return Backend()


def _has_module(name: str) -> bool:
    import importlib.util
    return importlib.util.find_spec(name) is not None


# -- WAVs ---------------------------------------------------------------------------------------------------------

def read_wav(path: Path):
    """-> (mono float samples in -1..1, rate). 16-bit PCM (what every backend here writes)."""
    with wave.open(str(path), "rb") as w:
        n, ch, sw, rate = w.getnframes(), w.getnchannels(), w.getsampwidth(), w.getframerate()
        raw = w.readframes(n)
    if sw != 2:
        raise ValueError(f"{path}: {8 * sw}-bit WAV")
    a = np.frombuffer(raw, np.int16).astype(float) / 32768.0
    return (a.reshape(-1, ch).mean(axis=1) if ch > 1 else a), rate


def write_wav(path: Path, a, rate: int):
    a = np.clip(np.asarray(a, float), -1.0, 1.0)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(rate))
        w.writeframes((a * 32767).astype(np.int16).tobytes())


def repitch(a, rate: int, pitch: float, out_rate: int = OUT_RATE):
    """a played pitch x higher (and that much shorter), resampled to out_rate."""
    n = max(1, int(round(len(a) * out_rate / (rate * pitch))))
    return np.interp(np.linspace(0.0, len(a) - 1, n), np.arange(len(a)), a)


# -- the effects (numpy only: the robot needs nothing more) ---------------------------------------------------------

def _shape(a, rate: int, gain_db):
    """a through a zero-phase filter: gain_db(f in Hz, an array) -> dB (the whole phrase at once: phrases are short)."""
    n = len(a)
    f = np.fft.rfftfreq(n, 1.0 / rate)
    return np.fft.irfft(np.fft.rfft(a) * 10 ** (gain_db(f) / 20.0), n)


def _taps(a, rate: int, taps):
    """a plus delayed copies of itself: taps = [(seconds, gain)] (a comb's ring, a room's first reflections)."""
    out = a.copy()
    for s, g in taps:
        d = int(round(s * rate))
        if 0 < d < len(a):
            out[d:] += g * a[:-d]
    return out


def _box(a, rate):
    """A small speaker in a metal box: band-limited (no boom, no fizz), a presence lift, a faint metallic ring
    (a short comb, 4.1 ms: ~240 Hz and its harmonics), a little drive and two early reflections."""
    a = _shape(a, rate, lambda f: (-12 * np.clip((140 - f) / 100, 0, 1)            # below ~140 Hz down
                                   - 14 * np.clip((f - 6800) / 2500, 0, 1)          # above ~6.8 kHz down
                                   + 3.0 * np.exp(-((f - 2600) / 900) ** 2)))       # presence
    a = _taps(a, rate, [(0.0041, 0.22), (0.0082, 0.07)])
    peak = np.abs(a).max() or 1.0
    a = np.tanh(1.5 * a / peak) / np.tanh(1.5) * peak
    return _taps(a, rate, [(0.019, 0.10), (0.031, 0.06)])


def _shimmer(a, rate):
    """Clean and light: a detuned double 11 ms behind (+0.5%: a faint chorus) and a little more air."""
    d = repitch(a, rate, 1.005, rate)
    k = int(0.011 * rate)
    dbl = np.zeros_like(a)
    m = min(len(a) - k, len(d))
    if m > 0:
        dbl[k:k + m] = d[:m]
    a = a + 0.22 * dbl
    a = _shape(a, rate, lambda f: 2.5 * np.clip((f - 5000) / 3000, 0, 1) - 10 * np.clip((120 - f) / 80, 0, 1))
    return _taps(a, rate, [(0.023, 0.07)])


FX = {"": None, "box": _box, "shimmer": _shimmer}


def effect(a, rate: int, fx: str):
    """a through the preset's effect, at the same loudness (peak) as it came."""
    f = FX.get(fx)
    if f is None or not len(a):
        return a
    peak = np.abs(a).max()
    b = f(np.asarray(a, float), rate)
    return b * (peak / (np.abs(b).max() or 1.0))


class Voice:
    """speak(text) -> the cached WAV (made now if needed, None if there's no voice). style: a persona (STYLES);
    preset: who it sounds like (PRESETS), changed any time (the console's setting)."""

    def __init__(self, backend: Backend | None = None, pitch: float | None = None, preset: str | None = None):
        self.b = backend or choose()
        self.preset = preset if preset in PRESETS else PRESET if PRESET in PRESETS else "deadpan"
        self.pitch = pitch                 # None: the preset's (OMNI_VOICE_PITCH overrides both)
        self.dir = CACHE / self.b.name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.failures = 0

    @property
    def name(self) -> str:
        return self.b.name

    def set_preset(self, name: str) -> bool:
        if name in PRESETS and name != self.preset:
            self.preset = name
            return True
        return False

    def wav(self, text: str, style: str = "") -> Path | None:
        if self.b.name == "none" or not text.strip():
            return None
        pr = PRESETS[self.preset]
        sp, sr = STYLES.get(style, STYLES[""])
        base = float(os.environ.get("OMNI_VOICE_PITCH") or self.pitch or pr["pitch"])
        pitch, rate = base * sp, pr["pace"] * sr
        fx = pr["fx"] if os.environ.get("OMNI_VOICE_FX", "1") != "0" else ""
        key = f"{self.b.key}|{pr['kokoro']}|{pr['sapi']}|{pitch:.3f}|{rate:.3f}|{fx}|{text}"
        p = self.dir / (hashlib.sha1(key.encode()).hexdigest()[:16] + ".wav")
        if p.exists() and p.stat().st_size > 1000:
            return p
        tmp = p.with_suffix(".tmp.wav")
        try:
            # Kokoro speaks slower by the pitch so that raising it keeps the pace; the others just get quicker
            ok = self.b.synth(text, tmp, rate / pitch if self.b.name == "kokoro" else rate, pr)
            if ok and (abs(pitch - 1.0) > 1e-3 or fx):
                a, r = read_wav(tmp)
                a = repitch(a, r, pitch)
                write_wav(tmp, effect(a, OUT_RATE, fx), OUT_RATE)
        except (OSError, subprocess.SubprocessError, ValueError, RuntimeError, ImportError, KeyError):
            ok = False
        if not ok:
            self.failures += 1
            return None
        os.replace(tmp, p)
        return p


def fetch(d: Path = KOKORO_DIR):
    """Download Kokoro's model (int8, 88 MB) and voices (27 MB) into d (not in git)."""
    d.mkdir(parents=True, exist_ok=True)
    for f in KOKORO_FILES:
        if not (d / f).exists():
            print(f"fetching {f} ...")
            urllib.request.urlretrieve(KOKORO_URL + f, d / (f + ".part"))
            os.replace(d / (f + ".part"), d / f)
    print(f"Kokoro in {d}" + ("" if _has_module("kokoro_onnx") else "; now `pip install kokoro-onnx`"))


AUDITION = ("Tidying up. Nobody touch anything.",
            "Five in the bin. Not that I'm counting. I am counting.",
            "Humor setting, seventy five percent.",
            "My battery is low. Please charge me.")


def audition(out: Path, voice: Voice, play: bool = False):
    """Every preset saying the same lines, into out/<preset>_<n>.wav (and played, on Windows, with play)."""
    out.mkdir(parents=True, exist_ok=True)
    for name in PRESETS:
        voice.set_preset(name)
        for i, line in enumerate(AUDITION):
            p = voice.wav(line)
            if p is None:
                print(f"{name}: no voice ({voice.name})")
                break
            dst = out / f"{name}_{i + 1}.wav"
            shutil.copyfile(p, dst)
            print(f"{name:8s} {dst}  {line}")
            if play and sys.platform == "win32":
                import winsound
                winsound.PlaySound(str(dst), winsound.SND_FILENAME)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="the robot's voice: fetch Kokoro, say a line into a WAV, or audition "
                                             "the presets")
    ap.add_argument("cmd", choices=("fetch", "say", "audition"))
    ap.add_argument("text", nargs="?", default="Online. Good to see you.")
    ap.add_argument("--voice", default=None, help="kokoro | piper | espeak | sapi")
    ap.add_argument("--preset", default=None, choices=sorted(PRESETS))
    ap.add_argument("--style", default="", choices=sorted(STYLES))
    ap.add_argument("--out", default=str(Path.home() / "omni_voice_audition"), help="audition: where the WAVs go")
    ap.add_argument("--play", action="store_true", help="play it (Windows)")
    a = ap.parse_args()
    if a.cmd == "fetch":
        fetch()
    elif a.cmd == "audition":
        audition(Path(a.out), Voice(choose(a.voice)), a.play)
    else:
        v = Voice(choose(a.voice), preset=a.preset)
        p = v.wav(a.text, a.style)
        print(v.name, v.preset, p)
        if a.play and p is not None and sys.platform == "win32":
            import winsound
            winsound.PlaySound(str(p), winsound.SND_FILENAME)
