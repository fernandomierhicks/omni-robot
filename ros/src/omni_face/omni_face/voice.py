"""Text to speech for the robot's voice: text -> a WAV file (cached), played by the display (ROS-free).

Backends, the first that works is used (choose()):
  kokoro   Kokoro-82M (Apache 2.0), an offline neural voice through onnxruntime: the most natural of these, on the
           PC and the robot. `pip install kokoro-onnx` and its two model files in ~/.local/share/kokoro
           (`python voice.py fetch`, 120 MB; OMNI_KOKORO_DIR elsewhere). Voice OMNI_KOKORO_VOICE (default af_heart).
           PC: ~0.2 x real time on the CPU (measured Oct 26); the Orin's CPU VERIFY (est 0.5-1 x: phrases are cached)
  piper    Piper, an offline neural voice that runs on the CPU. `pip install piper-tts` (or its binary) and a voice
           model (.onnx + .onnx.json, e.g. en_US-amy-medium); OMNI_PIPER_MODEL points at it, else the first one in
           ~/.local/share/piper (VERIFY its speed on the Orin: est 0.1-0.3 x real time)
  espeak   espeak-ng: tiny, robotic, on every Linux (`sudo apt install espeak-ng`)
  sapi     Windows' own voice (System.Speech through one PowerShell process kept open)
  none     no voice: the display still shows what it would have said
The voice is made cute by raising its pitch (the WAV resampled, which also shrinks the formants: a smaller speaker);
Kokoro speaks that much slower first so the pace stays. Each persona (moments.PERSONAS) speaks in its own style
(STYLES: patrol lower and slower, explore and tidy a little brighter and quicker). Each phrase is made once per
style and kept in ~/.cache/omni_face/tts/<backend>/ (the game's phrases repeat: after the first time they play at once).
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
# the voice's pitch (x, 1 = as made) and pace (x) for each persona; PITCH is the robot's own (cute: a small speaker)
PITCH = float(os.environ.get("OMNI_VOICE_PITCH", "1.12"))
STYLES = {"": (1.0, 1.0), "patrol": (0.9, 0.93), "explore": (1.04, 1.06), "tidy": (1.05, 1.05),
          "catch": (1.02, 1.03)}


class Backend:
    name = "none"
    key = "none"                   # what makes its WAVs differ (the cache key)

    def synth(self, text: str, out: Path, rate: float = 1.0) -> bool:
        """Write text spoken to out (a WAV). rate: the pace asked for (a backend that can't ignores it)."""
        return False


class Kokoro(Backend):
    name = "kokoro"

    def __init__(self, model: str, voices: str, voice: str | None = None):
        self.model, self.voices = model, voices
        self.voice = voice or os.environ.get("OMNI_KOKORO_VOICE", "af_heart")
        self.key = f"kokoro-{Path(model).stem}-{self.voice}"
        self.k = None
        self.lock = threading.Lock()

    def synth(self, text, out, rate=1.0):
        with self.lock:
            if self.k is None:
                from kokoro_onnx import Kokoro as K
                self.k = K(self.model, self.voices)
            a, sr = self.k.create(text, voice=self.voice, speed=max(0.5, min(2.0, rate)), lang="en-us")
        write_wav(out, np.asarray(a, float), sr)
        return out.exists()


class Piper(Backend):
    name = "piper"

    def __init__(self, model: str):
        self.model = model
        self.key = f"piper-{Path(model).stem}"
        self.cmd = shutil.which("piper") or [sys.executable, "-m", "piper"]

    def synth(self, text, out, rate=1.0):
        cmd = ([self.cmd] if isinstance(self.cmd, str) else self.cmd) + [
            "--model", self.model, "--output_file", str(out), "--length_scale", f"{1.0 / rate:.3f}"]
        r = subprocess.run(cmd, input=text.encode(), capture_output=True, timeout=30)
        return r.returncode == 0 and out.exists()


class Espeak(Backend):
    name = key = "espeak"

    def __init__(self, exe: str):
        self.exe = exe

    def synth(self, text, out, rate=1.0):
        r = subprocess.run([self.exe, "-s", str(int(160 * rate)), "-w", str(out), text], capture_output=True, timeout=30)
        return r.returncode == 0 and out.exists()


class Sapi(Backend):
    """One PowerShell kept open: each line in is "<wav path>\\t<text>", each line out "ok" when the file is written."""
    name = key = "sapi"
    SCRIPT = (
        "Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
        "$v = $s.GetInstalledVoices() | Where-Object { $_.VoiceInfo.Name -match 'Zira|Aria|Jenny' } | Select-Object -First 1; "
        "if ($v) { $s.SelectVoice($v.VoiceInfo.Name) }; $s.Rate = 1; "
        "while ($true) { $l = [Console]::In.ReadLine(); if ($l -eq $null) { break }; $p = $l.Split(\"`t\", 2); "
        "try { $s.SetOutputToWaveFile($p[0]); $s.Speak($p[1]); $s.SetOutputToNull(); [Console]::Out.WriteLine('ok') } "
        "catch { [Console]::Out.WriteLine('err') }; [Console]::Out.Flush() }")

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None

    def _start(self):
        self.proc = subprocess.Popen([POWERSHELL, "-NoProfile", "-NonInteractive", "-Command", self.SCRIPT],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                     text=True, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

    def synth(self, text, out, rate=1.0):
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                self._start()
            line = text.replace("\t", " ").replace("\n", " ")
            self.proc.stdin.write(f"{out}\t{line}\n")
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


class Voice:
    """speak(text) -> the cached WAV (made now if needed, None if there's no voice). style: a persona (STYLES)."""

    def __init__(self, backend: Backend | None = None, pitch: float = PITCH):
        self.b = backend or choose()
        self.pitch = pitch
        self.dir = CACHE / self.b.name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.failures = 0

    @property
    def name(self) -> str:
        return self.b.name

    def wav(self, text: str, style: str = "") -> Path | None:
        if self.b.name == "none" or not text.strip():
            return None
        sp, sr = STYLES.get(style, STYLES[""])
        pitch, rate = self.pitch * sp, sr
        key = f"{self.b.key}|{pitch:.3f}|{rate:.3f}|{text}"
        p = self.dir / (hashlib.sha1(key.encode()).hexdigest()[:16] + ".wav")
        if p.exists() and p.stat().st_size > 1000:
            return p
        tmp = p.with_suffix(".tmp.wav")
        try:
            # Kokoro speaks slower by the pitch so that raising it keeps the pace; the others just get quicker
            ok = self.b.synth(text, tmp, rate / pitch if self.b.name == "kokoro" else rate)
            if ok and abs(pitch - 1.0) > 1e-3:
                a, r = read_wav(tmp)
                write_wav(tmp, repitch(a, r, pitch), OUT_RATE)
        except (OSError, subprocess.SubprocessError, ValueError, RuntimeError, ImportError):
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


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="the robot's voice: fetch Kokoro, or say a line into a WAV")
    ap.add_argument("cmd", choices=("fetch", "say"))
    ap.add_argument("text", nargs="?", default="Hi! I'm Omni. Throw it to me!")
    ap.add_argument("--voice", default=None, help="kokoro | piper | espeak | sapi")
    ap.add_argument("--style", default="", choices=sorted(STYLES))
    a = ap.parse_args()
    if a.cmd == "fetch":
        fetch()
    else:
        v = Voice(choose(a.voice))
        print(v.name, v.wav(a.text, a.style))
