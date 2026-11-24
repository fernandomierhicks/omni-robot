"""The robot's voices on the main PC (ROS-free; Nov 21): neural TTS engines + the character chains (voice_fx.py).

Fernando (Nov 21): "voice stuff can be processed on the main PC, delays are not as important". So the heavy voices
run there, in their own venvs (they pin different torch / transformers):
  C:\\Users\\ferna\\.venvs\\omni-voice      torch 2.11 CUDA: Qwen3-TTS (Apache 2.0), Kokoro-82M (Apache 2.0), Pocket TTS
                                           (Kyutai, MIT code, CPU), pyworld, pedalboard, anthropic (chat.py)
  C:\\Users\\ferna\\.venvs\\omni-voice-cb   torch 2.6 CUDA: Chatterbox and Chatterbox Turbo (Resemble AI, MIT; output
                                           watermarked), run as a worker process (voice_cb_worker.py)
The PC has an RTX 4070 Ti SUPER (16 GB) and an i7-14700K. voice_server.py serves these to the face (display.py, through
voice.py's Server backend); voice_lab.py renders the audition.

VOICES: the candidates. Each names an engine and its settings, a character chain (voice_fx.CHAINS) and whether the
feeling is the engine's own (Qwen's instruction, Chatterbox's exaggeration and tags) or added by WORLD prosody
(Kokoro, Pocket). Designed voices (Qwen3-TTS VoiceDesign, DESIGNS) are original: described in words, not copied from
anyone; their reference clips (made once: voice_lab.py design) let Chatterbox and Pocket speak in the same voice.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import threading
from pathlib import Path

import numpy as np

from omni_face import voice_fx

HOME = Path(os.environ.get("OMNI_VOICE_HOME", Path.home() / ".local" / "share" / "omni_voice"))
REFS = HOME / "refs"
CB_PY = Path(os.environ.get("OMNI_VOICE_CB_PY", r"C:\Users\ferna\.venvs\omni-voice-cb\Scripts\python.exe"))
KOKORO_DIR = Path(os.environ.get("OMNI_KOKORO_DIR", Path.home() / ".local" / "share" / "kokoro"))
OUT_RATE = 22050                  # the display's mixer

# Qwen3-TTS VoiceDesign: original voices, described (the feeling goes after: "... Speaks <how>.")
DESIGNS = {
    "dry": "A calm, dry, low-pitched male voice of a capable household machine. Precise diction, understated, "
           "unhurried, with deadpan humor and a faintly synthetic smoothness.",
    "bright": "A bright, clear, youthful female voice of a friendly futuristic robot. Light and airy, crisp "
              "consonants, gentle warmth and a faintly synthetic sheen.",
    "small": "A small, curious, childlike robot voice: high, warm and slightly squeaky, endearing and playful, "
             "eager to please.",
}
REF_TEXT = ("Hello. I'm Omni, a household robot. I pick things up from the floor, I find my way around the house, "
            "and now and then, I make a joke.")

# how a feeling is said, in words (Qwen's instruction)
EMO_WORDS = {
    "": "naturally", "neutral": "calmly and matter-of-factly", "plain": "clearly and seriously, without emotion",
    "happy": "happily and warmly, with a smile in the voice", "excited": "very excitedly, with high energy",
    "proud": "proudly, pleased with itself, a little smug", "affection": "softly and affectionately",
    "content": "in a relaxed, content way", "relieved": "with relief, exhaling first, then calm",
    "curious": "curiously, intrigued, with a questioning lilt", "surprised": "with surprise",
    "startled": "startled and alarmed, suddenly", "focused": "concisely and focused",
    "effort": "while straining hard, breathy and broken up", "worried": "worriedly, hesitant and slightly shaky",
    "frustrated": "with frustration, exasperated", "sad": "sadly, quietly and slowly",
    "tired": "tiredly, sleepy and slow", "bored": "in a bored, flat monotone", "whisper": "in a soft whisper",
    "sarcastic": "dryly and sarcastically, deadpan", "playful": "playfully, sing-song", "nervous": "nervously, "
    "with hesitations", "dizzy": "woozily, a little disoriented", "triumphant": "triumphantly",
    "thinking": "thoughtfully, thinking aloud with pauses", "wonder": "with quiet wonder",
    "warm": "warmly and kindly",
}
# the extra styles a phrase may ask for -> the nearest feeling (WORLD prosody, Chatterbox's exaggeration)
EMO_ALIAS = {"plain": "neutral", "whisper": "tired", "sarcastic": "bored", "playful": "happy", "nervous": "worried",
             "dizzy": "surprised", "triumphant": "excited", "thinking": "focused", "wonder": "curious",
             "warm": "affection"}
# Chatterbox: (exaggeration, cfg weight) per feeling: more exaggeration is more expressive, a lower cfg slower
CB_EXAG = {"neutral": (0.4, 0.5), "happy": (0.65, 0.5), "excited": (0.9, 0.35), "proud": (0.6, 0.5),
           "affection": (0.55, 0.4), "content": (0.4, 0.5), "relieved": (0.5, 0.4), "curious": (0.6, 0.5),
           "surprised": (0.8, 0.4), "startled": (0.9, 0.35), "focused": (0.35, 0.55), "effort": (0.75, 0.4),
           "worried": (0.6, 0.4), "frustrated": (0.7, 0.45), "sad": (0.45, 0.3), "tired": (0.3, 0.3),
           "bored": (0.25, 0.5)}

VOICES = {
    "current": dict(engine="current", chain="raw", prosody=False, label="Today's voice (reference)",
                    desc="What the robot uses now: Kokoro int8, am_michael + am_fenrir, resampled lower, the old "
                         "metal box (a comb filter). No feeling.", runtime="robot CPU"),
    "kokoro_dry": dict(engine="kokoro", voice="am_michael", chain="tars", prosody=True, label="Kokoro · dry machine",
                       desc="Kokoro-82M full precision, one clean voice, the new 'tars' chain (no comb); feeling "
                            "from WORLD intonation and pace.", runtime="robot CPU (fast)"),
    "kokoro_bright": dict(engine="kokoro", voice="af_heart", chain="eve", prosody=True,
                          label="Kokoro · bright robot", desc="Kokoro af_heart through the 'eve' chain: higher, "
                          "smaller head, breath taken out, octave shimmer; WORLD feeling.", runtime="robot CPU (fast)"),
    "qwen_natural": dict(engine="qwen_custom", speaker="ryan", chain="clean", prosody=False,
                         label="Qwen3 · natural actor", desc="Qwen3-TTS CustomVoice, a stock English speaker, told "
                         "how to feel in words; studio-clean chain. The most human.", runtime="PC GPU"),
    "qwen_dry": dict(engine="qwen_design", design="dry", chain="tars", prosody=False, label="Qwen3 · designed dry",
                     desc="An original voice designed from words (a calm, dry, capable machine), feeling in the "
                          "instruction, the 'tars' chain.", runtime="PC GPU"),
    "qwen_bright": dict(engine="qwen_design", design="bright", chain="eve", prosody=False,
                        label="Qwen3 · designed bright", desc="An original designed voice (bright, clear, friendly "
                        "futuristic robot), the 'eve' chain.", runtime="PC GPU"),
    "qwen_small": dict(engine="qwen_design", design="small", chain="wall", prosody=False,
                       label="Qwen3 · designed small bot", desc="An original designed voice (a small curious "
                       "robot), the 'wall' chain: tiny head, warble, small gritty speaker.", runtime="PC GPU"),
    "cb_bright": dict(engine="chatterbox", ref="bright", chain="eve", prosody=False, label="Chatterbox · bright",
                      desc="Chatterbox speaking in the designed bright voice (cloned from its reference), feeling "
                           "by its exaggeration knob; the 'eve' chain.", runtime="PC GPU"),
    "cbt_dry": dict(engine="chatterbox_turbo", ref="dry", chain="tars", prosody=False,
                    label="Chatterbox Turbo · dry", desc="Chatterbox Turbo in the designed dry voice; real sighs "
                    "and laughs from its [sigh] [laugh] tags; the 'tars' chain.", runtime="PC GPU (fast)"),
    # Pocket's voice cloning is gated (accept Kyutai's terms on Hugging Face and log in): a stock voice until then
    "pocket_alba": dict(engine="pocket", voice="alba", chain="eve", prosody=True, label="Pocket TTS · alba (CPU)",
                        desc="Kyutai Pocket TTS on the CPU, its stock 'alba' voice through the 'eve' chain; WORLD "
                             "feeling. The candidate that could run on the robot itself.", runtime="robot CPU"),
    "droid": dict(engine="kokoro", voice="am_michael", chain="droid", prosody=False, label="Classic droid (contrast)",
                  desc="The old-movie robot for contrast: monotone snapped to one note, ring-modulated metal.",
                  runtime="robot CPU"),
}
# Nov 22, Fernando: "the female chipmunk voices" no; the rest "too robotic or too subtle and human". So: the human male
# voices with their feelings, made robotic by degrees (voice_fx's layered chains keep the intonation). Each robot
# flavour on the same performance (the raw cache), and the android ladder light -> strong on the most expressive actor
_ROBOT_MEN = (
    ("ryan_synth_light", dict(engine="qwen_custom", speaker="ryan"), "synth_light", "Ryan · android light"),
    ("ryan_synth", dict(engine="qwen_custom", speaker="ryan"), "synth", "Ryan · android"),
    ("ryan_synth_strong", dict(engine="qwen_custom", speaker="ryan"), "synth_strong", "Ryan · android strong"),
    ("ryan_vocoder", dict(engine="qwen_custom", speaker="ryan"), "vocoder", "Ryan · vocoder"),
    ("ryan_metal", dict(engine="qwen_custom", speaker="ryan"), "metal", "Ryan · big machine"),
    ("ryan_digital", dict(engine="qwen_custom", speaker="ryan"), "digital", "Ryan · retro computer"),
    ("aiden_synth", dict(engine="qwen_custom", speaker="aiden"), "synth", "Aiden · android"),
    ("aiden_vocoder", dict(engine="qwen_custom", speaker="aiden"), "vocoder", "Aiden · vocoder"),
    ("dry_synth", dict(engine="qwen_design", design="dry"), "synth", "Designed dry · android"),
    ("michael_synth", dict(engine="kokoro", voice="am_michael"), "synth", "Kokoro Michael · android"),
)
# round 3 (Nov 22): Fernando picked cbt_dry ("needs more emotion, little more robotic effects"). Chatterbox Turbo
# copies its reference clip's delivery, so the designed dry voice gets one reference per feeling (EMO_REFS: made once,
# design_emotion_refs) and each line takes the matching one; a moderate WORLD intonation boost on top; then the robot
# layered on (voice_fx robot_plus). The others: the emotion alone, and his other favourites' flavours, lighter
VOICES.update({
    "cbt_robot": dict(engine="chatterbox_turbo", ref="dry_emo", chain="robot_plus", prosody=0.6, robot=True,
                      label="Turbo dry · emotion + robot", runtime="PC GPU (fast)",
                      desc="Your pick, round 3: a reference clip per feeling (more emotion) and a 45 % android layer "
                           "with the intonation whole (a little more robotic)."),
    "cbt_emotion": dict(engine="chatterbox_turbo", ref="dry_emo", chain="tars_open", prosody=0.6, robot=True,
                        label="Turbo dry · emotion only", runtime="PC GPU (fast)",
                        desc="Your pick with the per-feeling references and intonation boost, the original radio "
                             "edge, no android layer: to hear what the emotion change alone does."),
    "cbt_vocoder": dict(engine="chatterbox_turbo", ref="dry_emo", chain="vocoder_lite", prosody=0.6, robot=True,
                        label="Turbo dry · emotion + soft vocoder", runtime="PC GPU (fast)",
                        desc="The emotional take through half a soft vocoder (after your Aiden and Ryan vocoder "
                             "favourites)."),
    "cbt_metal": dict(engine="chatterbox_turbo", ref="dry_emo", chain="metal_lite", prosody=0.6, robot=True,
                      label="Turbo dry · emotion + light machine", runtime="PC GPU (fast)",
                      desc="The emotional take with lighter metal resonances (after your Ryan big machine)."),
})
# round 4 (Nov 23): Fernando's pick cbt_vocoder ("emotion level is correct"), eight steps more robotic / metallic /
# synthesized on the same emotional takes (voice_fx voc_1 .. voc_8)
for _i in range(1, 9):
    VOICES[f"cbt_voc{_i}"] = dict(engine="chatterbox_turbo", ref="dry_emo", chain=f"voc_{_i}", prosody=0.6, robot=True,
                                  label=f"Vocoder {_i}", runtime="PC GPU (fast)",
                                  desc=f"Your pick (soft vocoder), same emotion; {voice_fx.CHAINS[f'voc_{_i}']['desc']}.")
EMO_REFS = {          # feeling -> what the designed dry voice says in that feeling's reference clip
    "neutral": REF_TEXT,
    "happy": "Oh, that's wonderful! I finished the whole room, and everything is exactly where it belongs. What a "
             "good day.",
    "excited": "Wow! Look at that! A brand new room to explore! I can't wait to see what's in there!",
    "proud": "Five things in the bin, all on the first try. I have to say, that was some very fine work.",
    "affection": "You came back. I'm really glad you're here. I missed having you around, you know.",
    "sad": "Oh. I couldn't reach it. I tried, I really did. I'm sorry. Maybe next time.",
    "worried": "Um, I'm not sure where I am. Something's wrong. I don't know which way to go.",
    "frustrated": "That's the third time it slipped. Again! Why won't this thing just stay in my hand?",
    "tired": "My battery is almost empty. I'm so tired. I think I need to rest for a while.",
    "curious": "Hmm? What's that under the sofa? I wonder what it is. Let me take a closer look.",
    "startled": "Whoa! What was that? You scared me! Don't push my arm like that!",
}
EMO_REF_OF = {"content": "happy", "relieved": "happy", "surprised": "startled", "focused": "neutral",
              "effort": "frustrated", "bored": "tired", "": "neutral"}   # the rest: their own, or neutral


def emotion_ref(design: str, emotion: str) -> Path:
    """The designed voice's reference clip for this feeling (the nearest one made)."""
    e = EMO_ALIAS.get(emotion, emotion)
    e = e if e in EMO_REFS else EMO_REF_OF.get(e, "neutral")
    p = REFS / f"{design}_{e}.wav"
    return p if p.exists() else REFS / f"{design}.wav"


for _vid, _src, _chain, _label in _ROBOT_MEN:
    _who = {"ryan": "Qwen3's 'Ryan' (a dynamic English actor)", "aiden": "Qwen3's 'Aiden' (a sunny American)",
            "dry": "the designed dry machine voice", "am_michael": "Kokoro's Michael (on the robot's own CPU)"}[
        _src.get("speaker") or _src.get("design") or _src.get("voice")]
    VOICES[_vid] = dict(_src, chain=_chain, prosody=_src["engine"] == "kokoro", label=_label, robot=True,
                        desc=f"{_who}, told how to feel, then: {voice_fx.CHAINS[_chain]['desc']}.",
                        runtime="robot CPU (fast)" if _src["engine"] == "kokoro" else "PC GPU")

TAG = re.compile(r"\[(laugh|sigh|gasp|chuckle|cough|groan|sniff|clear throat|shush)\]\s*", re.I)


def plain_text(text: str) -> str:
    """The text without Chatterbox Turbo's sound tags (other engines would read them out)."""
    return re.sub(r"\s+", " ", TAG.sub("", text)).strip()


def resample(a: np.ndarray, sr: int, out: int = OUT_RATE) -> np.ndarray:
    if sr == out or not len(a):
        return a
    n = int(round(len(a) * out / sr))
    return np.interp(np.linspace(0, len(a) - 1, n), np.arange(len(a)), a)


class Engines:
    """The engines, each loaded on first use (and kept: the GPU has room for all of them)."""

    def __init__(self, raw_dir: Path | None = None):
        self.raw_dir = raw_dir             # the engines' raw speech kept here: a new character chain reuses it
        self.lock = threading.Lock()
        self.kokoro = None
        self.qwen = {}
        self.pocket = None
        self.pocket_states = {}
        self.cb = None

    # -- each engine -> (mono float, sample rate) ------------------------------------------------------------------
    def _kokoro(self, text, voice, speed=1.0):
        if self.kokoro is None:
            from kokoro_onnx import Kokoro
            full = KOKORO_DIR / "kokoro-v1.0.onnx"
            if not full.exists():
                import urllib.request
                KOKORO_DIR.mkdir(parents=True, exist_ok=True)
                urllib.request.urlretrieve("https://github.com/thewh1teagle/kokoro-onnx/releases/download/"
                                           "model-files-v1.0/kokoro-v1.0.onnx", full)
            self.kokoro = Kokoro(str(full), str(KOKORO_DIR / "voices-v1.0.bin"))
        a, sr = self.kokoro.create(text, voice=voice, speed=speed, lang="en-us")
        return np.asarray(a, float), sr

    def _qwen_model(self, kind):
        if kind not in self.qwen:
            import torch
            from qwen_tts import Qwen3TTSModel
            name = {"custom": "Qwen/Qwen3-TTS-12Hz-1.7B-CustomVoice",
                    "design": "Qwen/Qwen3-TTS-12Hz-1.7B-VoiceDesign"}[kind]
            try:                                         # the downloaded copy: no network needed (nor wanted)
                from huggingface_hub import snapshot_download
                name = snapshot_download(name, local_files_only=True)
            except Exception:                            # noqa: BLE001  (not downloaded yet: from the hub)
                pass
            self.qwen[kind] = Qwen3TTSModel.from_pretrained(name, device_map="cuda:0", dtype=torch.bfloat16,
                                                            attn_implementation="sdpa")
        return self.qwen[kind]

    def _qwen(self, text, kind, emotion, speaker=None, design=None):
        m = self._qwen_model(kind)
        how = EMO_WORDS.get(emotion, EMO_WORDS[""])
        if kind == "custom":
            wavs, sr = m.generate_custom_voice(text=text, speaker=speaker, language="English",
                                               instruct=f"Speak {how}.")
        else:
            wavs, sr = m.generate_voice_design(text=text, language="English",
                                               instruct=f"{DESIGNS[design]} Speaks {how}.")
        return np.asarray(wavs[0], float), sr

    def _pocket(self, text, voice=None, ref=None):
        """voice: one of Pocket's stock voices; ref: a designed voice's reference clip (needs the gated cloning model)."""
        if self.pocket is None:
            from pocket_tts import TTSModel
            self.pocket = TTSModel.load_model()
        key = voice or ref
        if key not in self.pocket_states:
            self.pocket_states[key] = self.pocket.get_state_for_audio_prompt(voice or str(REFS / f"{ref}.wav"))
        a = self.pocket.generate_audio(self.pocket_states[key], text, max_tokens=400)
        return a.detach().cpu().numpy().astype(float), self.pocket.sample_rate

    def _chatterbox(self, text, ref, emotion, turbo):
        if self.cb is None:
            self.cb = CbWorker()
        exag, cfg = CB_EXAG.get(EMO_ALIAS.get(emotion, emotion), CB_EXAG["neutral"])
        path = emotion_ref(ref[:-4], emotion) if ref.endswith("_emo") else REFS / f"{ref}.wav"
        return self.cb.say(text, str(path), exag, cfg, turbo)

    # -- a voice saying a line -------------------------------------------------------------------------------------
    def render(self, voice_id: str, text: str, emotion: str = "") -> tuple:
        """-> (mono float at OUT_RATE, OUT_RATE): the voice VOICES[voice_id] saying text with the feeling."""
        v = VOICES[voice_id]
        emo = EMO_ALIAS.get(emotion, emotion)
        eng = v["engine"]
        if eng == "current":
            with self.lock:
                return self._current(text)
        raw = None
        if self.raw_dir is not None:                    # the same performance for every chain on this engine
            import hashlib
            sig = f"{eng}-{v.get('speaker') or v.get('design') or v.get('voice') or v.get('ref')}"
            raw = self.raw_dir / sig / (hashlib.sha1(f"{emotion}|{text}".encode()).hexdigest()[:16] + ".wav")
            if raw.exists():
                import soundfile as sf
                a, sr = sf.read(str(raw))
                y = voice_fx.render(np.asarray(a, float), sr, v["chain"], emo, prosody=v["prosody"])
                return resample(y, sr), OUT_RATE
        a, sr = self._raw(v, eng, text, emotion)
        if raw is not None:
            import soundfile as sf
            raw.parent.mkdir(parents=True, exist_ok=True)
            sf.write(str(raw), np.asarray(a, np.float32), sr)
        y = voice_fx.render(a, sr, v["chain"], emo, prosody=v["prosody"])
        return resample(y, sr), OUT_RATE

    def _raw(self, v, eng, text, emotion) -> tuple:
        """The engine's own speech, before any character: (mono float, sample rate)."""
        with self.lock:
            if eng == "kokoro":
                a, sr = self._kokoro(plain_text(text), v["voice"])
            elif eng == "qwen_custom":
                a, sr = self._qwen(plain_text(text), "custom", emotion, speaker=v["speaker"])
            elif eng == "qwen_design":
                a, sr = self._qwen(plain_text(text), "design", emotion, design=v["design"])
            elif eng == "pocket":
                a, sr = self._pocket(plain_text(text), v.get("voice"), v.get("ref"))
            elif eng in ("chatterbox", "chatterbox_turbo"):
                turbo = eng == "chatterbox_turbo"
                a, sr = self._chatterbox(text if turbo else plain_text(text), v["ref"], emotion, turbo)
            else:
                raise ValueError(f"engine {eng}")
        return np.asarray(a, float), sr

    def _current(self, text):
        """Today's voice exactly (voice.py: Kokoro int8, the deadpan preset, its old box effect)."""
        import tempfile
        from omni_face import voice as old
        files = old.kokoro_files()
        if files is None:
            raise RuntimeError("no Kokoro model for today's voice")
        int8 = KOKORO_DIR / "kokoro-v1.0.int8.onnx"
        k = old.Kokoro(str(int8) if int8.exists() else files[0], files[1])
        vc = old.Voice(k, preset="deadpan")
        vc.dir = Path(tempfile.mkdtemp(prefix="omni_current_"))
        p = vc.wav(plain_text(text))
        a, sr = old.read_wav(p)
        return resample(a, sr), OUT_RATE

    def design_refs(self, force=False):
        """The designed voices' reference clips (REFS/<design>.wav + .txt), for Chatterbox and Pocket to clone."""
        import soundfile as sf
        REFS.mkdir(parents=True, exist_ok=True)
        for d in DESIGNS:
            wav = REFS / f"{d}.wav"
            if wav.exists() and not force:
                continue
            with self.lock:
                a, sr = self._qwen(REF_TEXT, "design", "neutral", design=d)
            sf.write(str(wav), a, sr)
            (REFS / f"{d}.txt").write_text(REF_TEXT, encoding="utf-8")

    def design_emotion_refs(self, design: str = "dry", force: bool = False):
        """One reference clip per feeling for a designed voice (EMO_REFS): REFS/<design>_<feeling>.wav."""
        import soundfile as sf
        REFS.mkdir(parents=True, exist_ok=True)
        for e, text in EMO_REFS.items():
            wav = REFS / f"{design}_{e}.wav"
            if wav.exists() and not force:
                continue
            with self.lock:
                a, sr = self._qwen(text, "design", e, design=design)
            sf.write(str(wav), a, sr)
            (REFS / f"{design}_{e}.txt").write_text(text, encoding="utf-8")


class CbWorker:
    """Chatterbox in its own venv (torch 2.6): voice_cb_worker.py, JSON lines over its stdin / stdout."""

    def __init__(self):
        script = Path(__file__).with_name("voice_cb_worker.py")
        self.p = subprocess.Popen([str(CB_PY), "-u", str(script)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, text=True, encoding="utf-8",
                                  creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        self.n = 0

    def say(self, text, ref, exag, cfg, turbo):
        import soundfile as sf
        import tempfile
        self.n += 1
        out = Path(tempfile.gettempdir()) / f"omni_cb_{os.getpid()}_{self.n}.wav"
        self.p.stdin.write(json.dumps(dict(text=text, ref=ref, exag=exag, cfg=cfg, turbo=turbo, out=str(out))) + "\n")
        self.p.stdin.flush()
        while True:
            line = self.p.stdout.readline()
            if not line:
                raise RuntimeError("the Chatterbox worker stopped")
            if line.startswith("{"):
                r = json.loads(line)
                break
        if not r.get("ok"):
            raise RuntimeError(f"Chatterbox: {r.get('error')}")
        a, sr = sf.read(str(out))
        out.unlink(missing_ok=True)
        return np.asarray(a, float).reshape(len(a), -1).mean(axis=1), sr
