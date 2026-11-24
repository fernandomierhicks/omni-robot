"""The voice audition (ROS-free; Nov 21, Fernando: "30 or more sample phrases in different voices or effects so I can
choose; include intonation and other details so I can judge fully"). Renders PHRASES in every voice_engines.VOICES
candidate and the chirps (sounds.py) into a folder with an index.json, for the audition page.

  C:\\Users\\ferna\\.venvs\\omni-voice\\Scripts\\python.exe -m omni_face.voice_lab render [--out DIR] [--voices a,b]
  (from ros/src/omni_face; the designed voices' reference clips are made first: voice_engines.Engines.design_refs)

Each phrase carries its feeling (how the engines are asked to say it) and what to listen for: where the pitch
should rise or fall, the pauses, the timing of a joke, a laugh or a sigh. The phrases cover the robot's real moments
(a pick, a slip, a push, a flat battery, praise, a question) and the hard cases for a voice (questions, sarcasm,
whispers, counting, a long list, hesitations, nonverbal sounds).
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

OUT = Path.home() / "omni_voice_audition"

# (id, feeling, text, what to listen for)
PHRASES = [
    ("hello", "neutral", "Online. Good morning. All systems nominal.",
     "A calm start: three short statements, each falling at the end; no sing-song."),
    ("question", "curious", "Hmm? What's that under the sofa?",
     "A thinking 'hmm', then a real question: the pitch must rise on 'sofa'."),
    ("excited", "excited", "Ooh! A red block! I've been looking for one of those!",
     "High energy and pitch jumps on 'Ooh' and 'red block'; quicker pace."),
    ("counting", "proud", "Five in the bin. Not that I'm counting. I am counting.",
     "Comic timing: a small pause before the last sentence, which lands dry and a little smug."),
    ("slip", "frustrated", "That one fought back. Again. Third time.",
     "Clipped, falling sentences; irritation building through 'Again' and 'Third time'."),
    ("sorry", "sad", "Oh. I couldn't reach it. Sorry.",
     "Soft, slower, falling; a real pause after 'Oh'."),
    ("startled", "startled", "Whoa! Easy! That's my arm!",
     "Sudden and loud, quick; the voice jumps up on 'Whoa'."),
    ("missed_you", "affection", "You came back. I missed you. Well. I noticed you were gone.",
     "Warm, then a hesitation and a self-correction ('Well.') that stays fond, not sarcastic."),
    ("battery", "tired", "Battery at twelve percent. I could really use a nap.",
     "Slow and low, trailing off at the end; the number said clearly."),
    ("lost", "worried", "I'm not sure where I am. Let me look around... carefully.",
     "Hesitant and wavering; a pause before 'carefully'."),
    ("relief", "relieved", "Phew. Found the hallway. Never in doubt.",
     "An exhale on 'Phew', then deadpan pride on the last sentence."),
    ("bored", "bored", "Nothing to do. I've counted my wheels. Still four.",
     "Flat and monotone on purpose; does it stay intelligible?"),
    ("joke", "playful", "I told the vacuum a joke. It sucked.",
     "Setup, a beat, then the punchline a touch lower and drier."),
    ("sarcasm", "sarcastic", "Oh good. Another sock. My favourite.",
     "Dry sarcasm: 'Oh good' flat, 'My favourite' slightly stretched."),
    ("effort", "effort", "Hnngh... this door is... heavier than it looks!",
     "Straining, broken phrasing at the dots, a push of breath at the end."),
    ("on_my_way", "focused", "On my way to the kitchen.",
     "Crisp and confident; the plain command acknowledgement."),
    ("say_again", "curious", "Sorry, I didn't catch that. Could you say it again?",
     "Polite, rising on 'again'."),
    ("countdown", "excited", "Three. Two. One. Ready or not, here I come!",
     "Rhythmic, evenly spaced numbers, building to the last line."),
    ("whisper", "whisper", "Shh. Everyone's asleep. I'll tidy quietly.",
     "Hushed and breathy (only some engines can truly whisper)."),
    ("laugh", "happy", "[laugh] Ha! You almost got me that time.",
     "A real laugh at the start (Chatterbox Turbo's [laugh] tag); others read 'Ha!'."),
    ("sigh", "frustrated", "[sigh] Fine. I'll pick up the Legos too.",
     "An audible sigh first (Turbo's [sigh]); then resigned, falling."),
    ("wonder", "wonder", "Every room is a new world. This one has a chair.",
     "Quiet awe in the first sentence, then a deadpan drop."),
    ("good_day", "proud", "I mapped forty square metres, found eleven toys, and only bumped into one plant. "
                          "A good day.",
     "A long sentence: list intonation (up, up, down), breath in the right place, a satisfied close."),
    ("estop", "plain", "Emergency stop. Motors off.",
     "Plain and serious: no feeling, no character effects should hide the words."),
    ("low_batt", "plain", "My battery is low. Please charge me.",
     "A plain warning, clear at a distance."),
    ("hello_again", "warm", "Hello there. Nice to see you again.",
     "Warm greeting, a smile in the voice."),
    ("thanks", "happy", "You're welcome. I accept payment in electricity.",
     "Friendly, then a dry little joke."),
    ("dizzy", "dizzy", "Whoa. The room is spinning. Or I am. Unclear.",
     "Woozy, uneven pace."),
    ("sing_song", "playful", "La la la. Clean floors, happy robot.",
     "Sing-song melody on 'La la la'."),
    ("nervous", "nervous", "Um... is that a cat? Hi, cat. Please don't.",
     "Nervous hesitations; small pleading 'Please don't'."),
    ("tower", "triumphant", "Ta-da! Tower complete. Five blocks high!",
     "Triumphant: 'Ta-da' bright and high, a proud finish."),
    ("thinking", "thinking", "Let me think... left is blocked, right is clear. Right it is.",
     "Thinking aloud: a pause, two weighed options, a decisive end."),
    ("goodnight", "tired", "Goodnight. I'll be right here, charging and dreaming of clean floors.",
     "Soft and slow, gentle falling end."),
    ("vocals", "curious", "Mm-hm! ... Uh-oh. ... Ooh? ... Hmph.",
     "Its vocalizations: each a different feeling (yes, uh-oh, a question, a huff)."),
]


def render(out: Path, voices: list | None = None, force: bool = False):
    import soundfile as sf
    from omni_face import sounds
    from omni_face.voice_engines import VOICES, Engines
    out.mkdir(parents=True, exist_ok=True)
    eng = Engines(raw_dir=out / "_raw")                 # each engine's performance once; every chain reuses it
    t0 = time.time()
    eng.design_refs()
    for d in {VOICES[v]["ref"][:-4] for v in (voices or VOICES) if str(VOICES[v].get("ref", "")).endswith("_emo")}:
        eng.design_emotion_refs(d)                      # one reference per feeling (Chatterbox copies its delivery)
    print(f"designed voices' references: {time.time() - t0:.0f} s", flush=True)
    index = dict(voices={}, phrases=[dict(id=i, emotion=e, text=t, listen=w) for i, e, t, w in PHRASES], chirps=[])
    old = out / "index.json"
    if old.exists():
        index["voices"] = json.loads(old.read_text(encoding="utf-8")).get("voices", {})
    for vid in voices or list(VOICES):
        v = VOICES[vid]
        d = out / vid
        d.mkdir(exist_ok=True)
        t_v, times, made = time.time(), [], 0
        for pid, emo, text, _ in PHRASES:
            p = d / f"{pid}.mp3"
            if p.exists() and not force:
                continue
            try:
                t = time.time()
                a, sr = eng.render(vid, text, emo)
                times.append((time.time() - t, len(a) / sr))
                sf.write(str(p), np.clip(a, -1, 1).astype(np.float32), sr, format="MP3")
                made += 1
            except Exception as e:                         # noqa: BLE001  (one bad line, keep going)
                print(f"{vid}/{pid}: {type(e).__name__}: {e}", flush=True)
        rtf = sum(a for a, _ in times) / max(1e-6, sum(b for _, b in times)) if times else None
        cur = json.loads(old.read_text(encoding="utf-8")) if old.exists() else index    # another render may run
        cur.setdefault("voices", {})[vid] = dict(label=v["label"], desc=v["desc"], runtime=v["runtime"],
                                                 chain=v["chain"], robot=bool(v.get("robot")),
                                                 rtf=None if rtf is None else round(rtf, 2))
        cur["phrases"] = index["phrases"]
        index["voices"] = cur["voices"]
        print(f"{vid}: {made} lines in {time.time() - t_v:.0f} s" + (f", {rtf:.2f} x real time" if rtf else ""),
              flush=True)
        old.write_text(json.dumps(cur, indent=1), encoding="utf-8")
    cd = out / "chirps"
    cd.mkdir(exist_ok=True)
    for e in sounds.CHIRP_EMOTIONS:
        if e == "love":
            continue
        for k in range(sounds.VARIANTS):
            a = sounds.chirp(e, 22050, 0.6, k).astype(np.float32) / 32767.0
            sf.write(str(cd / f"{e}_{k}.mp3"), a, 22050, format="MP3")
        index["chirps"].append(e)
    (out / "index.json").write_text(json.dumps(index, indent=1), encoding="utf-8")
    print(f"done: {out}", flush=True)


def main():
    ap = argparse.ArgumentParser(description="the robot's voice audition")
    ap.add_argument("cmd", choices=("render", "design"))
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--voices", default="", help="comma-separated VOICES ids (default: all)")
    ap.add_argument("--force", action="store_true", help="render again what exists")
    a = ap.parse_args()
    if a.cmd == "design":
        from omni_face.voice_engines import Engines
        Engines().design_refs(force=a.force)
    else:
        render(Path(a.out), [v for v in a.voices.split(",") if v] or None, a.force)


if __name__ == "__main__":
    main()
