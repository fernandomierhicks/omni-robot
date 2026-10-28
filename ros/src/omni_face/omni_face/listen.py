"""Speech in: the robot's microphones -> the phrases it heard (ROS-free). The display runs it (it owns the audio on
the robot and in the sim); what it hears goes to the face node, which decides what it means (commands.py).

Backends, the first that works is used (choose()); each listens for a grammar: the wake word ("omni", "hey omni",
"robot" ...) followed by one of the command phrases, the wake word alone, or a phrase alone (the face node takes a
phrase alone only in the few seconds after the wake word). A closed grammar is what makes a small offline recognizer
reliable: it can only answer with a phrase it was given, and its confidence says how sure it is.
  vosk     the robot (Jetson): Vosk (Kaldi), offline, on the CPU, with a small English model (~40 MB:
           vosk-model-small-en-us-0.15 from alphacephei.com/vosk/models; OMNI_VOSK_MODEL points at it, else
           ~/.local/share/vosk/<the first model>); `pip install vosk sounddevice`. Audio from the USB mic array
           (echo cancelled; OMNI_MIC names the ALSA device, VERIFY its name on the Jetson)
  sapi     Windows' own recognizer (System.Speech, one PowerShell process per grammar): the sim on the PC, from the
           PC's default microphone; recognize_wav() runs it on a file (the tests)
  none     no microphones
Results: (text, confidence 0-1), newest last, from results() (the display polls it every frame).
"""

from __future__ import annotations

import glob
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

from omni_face.commands import WAKE

POWERSHELL = r"C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe"


class Listener:
    name = "none"

    def __init__(self):
        self.q: queue.Queue = queue.Queue()

    def start(self, phrases: list):
        """Listen for these command phrases (the wake word goes in front by itself); restarts if running."""

    def stop(self):
        pass

    def running(self) -> bool:
        return False

    def results(self) -> list:
        out = []
        while not self.q.empty():
            out.append(self.q.get())
        return out


def full_phrases(phrases) -> list:
    """Every sentence a closed grammar accepts: wake + phrase, wake alone, phrase alone."""
    out = list(WAKE) + list(phrases)
    out += [f"{w} {p}" for w in WAKE for p in phrases]
    return sorted(set(out))


class Sapi(Listener):
    """System.Speech: a PowerShell process that loads the grammar from a file and prints "H<tab>conf<tab>text" for
    each phrase; from the default microphone, or from a WAV (then it exits at the file's end)."""
    name = "sapi"
    SCRIPT = r"""
param([string]$Grammar, [string]$Wav, [int]$Parent = 0)
Add-Type -AssemblyName System.Speech
$e = New-Object System.Speech.Recognition.SpeechRecognitionEngine (New-Object System.Globalization.CultureInfo "en-US")
$g = Get-Content -LiteralPath $Grammar -Encoding UTF8 | ConvertFrom-Json
$wake = New-Object System.Speech.Recognition.Choices; foreach ($w in $g.wake) { $wake.Add($w) }
$cmds = New-Object System.Speech.Recognition.Choices; foreach ($p in $g.phrases) { $cmds.Add($p) }
$withCmd = New-Object System.Speech.Recognition.GrammarBuilder; $withCmd.Append($wake); $withCmd.Append($cmds)
$alone = New-Object System.Speech.Recognition.GrammarBuilder; $alone.Append($wake)
$bare = New-Object System.Speech.Recognition.GrammarBuilder; $bare.Append($cmds)
$all = New-Object System.Speech.Recognition.Choices
$all.Add($withCmd); $all.Add($alone); $all.Add($bare)
$e.LoadGrammar((New-Object System.Speech.Recognition.Grammar (New-Object System.Speech.Recognition.GrammarBuilder $all)))
if ($Wav) { $e.SetInputToWaveFile($Wav) } else { $e.SetInputToDefaultAudioDevice() }
[Console]::Out.WriteLine("R"); [Console]::Out.Flush()
while ($true) {
  # from the microphone: listen in 1 s turns and quit once the face that started it is gone (no orphan on the mic)
  if ($Parent -and -not (Get-Process -Id $Parent -ErrorAction SilentlyContinue)) { break }
  try { $r = if ($Wav) { $e.Recognize() } else { $e.Recognize([TimeSpan]::FromSeconds(1)) } }
  catch { [Console]::Out.WriteLine("E`t" + $_.Exception.Message); break }
  if ($r) { [Console]::Out.WriteLine("H`t" + $r.Confidence + "`t" + $r.Text); [Console]::Out.Flush() }
  elseif ($Wav) { break }
}
"""

    def __init__(self):
        super().__init__()
        self.proc = None
        self.dir = Path(tempfile.gettempdir()) / "omni_face_listen"
        self.dir.mkdir(exist_ok=True)
        self.script = self.dir / "sapi_listen.ps1"
        self.script.write_text(self.SCRIPT, encoding="utf-8")

    def _grammar_file(self, phrases) -> Path:
        p = self.dir / "grammar.json"
        p.write_text(json.dumps(dict(wake=list(WAKE), phrases=sorted(set(phrases)))), encoding="utf-8")
        return p

    def _spawn(self, phrases, wav=None):
        args = [POWERSHELL, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(self.script),
                "-Grammar", str(self._grammar_file(phrases))] + (["-Wav", str(wav)] if wav else
                                                                 ["-Parent", str(os.getpid())])
        return subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                text=True, encoding="utf-8", creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

    @staticmethod
    def _parse(line):
        parts = line.rstrip("\n").split("\t")
        if parts[0] == "H" and len(parts) == 3:
            try:
                return parts[2], float(parts[1].replace(",", "."))
            except ValueError:
                return None
        return None

    def start(self, phrases):
        self.stop()
        self.proc = self._spawn(phrases)

        def read(proc):
            for line in proc.stdout:
                r = self._parse(line)
                if r is not None:
                    self.q.put(r)
        threading.Thread(target=read, args=(self.proc,), daemon=True).start()

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
        self.proc = None

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def recognize_wav(self, phrases, wav, timeout=30.0) -> list:
        """Every phrase heard in a WAV file (for the tests)."""
        p = self._spawn(phrases, wav)
        out, _ = p.communicate(timeout=timeout)
        return [r for r in (self._parse(l) for l in out.splitlines()) if r is not None]


class Vosk(Listener):
    """Vosk with a closed grammar on the microphone (16 kHz mono)."""
    name = "vosk"

    def __init__(self, model_dir: str):
        super().__init__()
        import vosk
        vosk.SetLogLevel(-1)
        self.model = vosk.Model(model_dir)
        self.stop_ev = None
        self.thread = None

    def _recognizer(self, phrases):
        import vosk
        return vosk.KaldiRecognizer(self.model, 16000, json.dumps(full_phrases(phrases) + ["[unk]"]))

    @staticmethod
    def _result(js) -> tuple | None:
        r = json.loads(js)
        text = r.get("text", "").strip()
        words = r.get("result") or []
        if not text or text == "[unk]":
            return None
        conf = sum(w.get("conf", 0.0) for w in words) / len(words) if words else 0.5
        return text, conf

    def start(self, phrases):
        self.stop()
        import sounddevice as sd
        rec = self._recognizer(phrases)
        self.stop_ev = threading.Event()
        ev = self.stop_ev

        def run():
            with sd.RawInputStream(samplerate=16000, blocksize=4000, dtype="int16", channels=1,
                                   device=os.environ.get("OMNI_MIC") or None) as stream:
                while not ev.is_set():
                    data, _ = stream.read(4000)
                    if rec.AcceptWaveform(bytes(data)):
                        r = self._result(rec.Result())
                        if r is not None:
                            self.q.put(r)
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def stop(self):
        if self.stop_ev is not None:
            self.stop_ev.set()
        self.stop_ev = None

    def running(self):
        return self.thread is not None and self.thread.is_alive() and self.stop_ev is not None

    def recognize_wav(self, phrases, wav) -> list:
        import wave
        rec, out = self._recognizer(phrases), []
        with wave.open(str(wav), "rb") as w:
            while True:
                data = w.readframes(4000)
                if not data:
                    break
                if rec.AcceptWaveform(data):
                    r = self._result(rec.Result())
                    if r:
                        out.append(r)
        r = self._result(rec.FinalResult())
        return out + ([r] if r else [])


def vosk_model() -> str | None:
    m = os.environ.get("OMNI_VOSK_MODEL")
    if m and os.path.isdir(m):
        return m
    found = sorted(glob.glob(str(Path.home() / ".local" / "share" / "vosk" / "vosk-model*")))
    return found[0] if found else None


def choose(prefer: str | None = "auto") -> Listener:
    prefer = prefer or "auto"
    if prefer == "none":
        return Listener()
    if prefer in ("auto", "vosk"):
        model = vosk_model()
        try:
            if model:
                import sounddevice  # noqa: F401
                return Vosk(model)
        except Exception:                               # no vosk, no audio device, a bad model: the next one
            pass
    if prefer in ("auto", "sapi") and sys.platform == "win32" and os.path.exists(POWERSHELL):
        return Sapi()
    return Listener()
