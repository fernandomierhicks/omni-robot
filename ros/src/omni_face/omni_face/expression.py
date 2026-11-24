"""How a feeling shows (ROS-free): one emotion, every channel at once, each as much as its level allows.

A reaction (REACTIONS[emotion]) is a small performance across channels: the face's antic (display.py draws it), a
vocalization (a voiced "whoa!" in its own voice, VOCALS) or a chirp (sounds.py's beeps with the emotion's
intonation), and a body gesture: the base's (omni_console gestures.py: a wiggle, a bounce, a recoil; only when the
base is free) and the arm's (omni_arm gestures.py: a nod, a head tilt, a flinch; only when the arm is free). The
face and the sounds are the robot's own (the display); a body gesture is a request to the console (`express`), which
plays it only if nothing else owns that part of the robot, at the size its level sets. A task always wins: during a
pick or a drive the feeling shows on the face and in the voice, the body waits.

Levels (affect.Levels, 0-10 each): a channel at 0 never reacts; at 5 (the design) it reacts as designed; at 10 about
twice as often and at full size. The chance a channel reacts grows with the event's intensity, so a big moment (a
finished tidy, a startle) gets through at low levels and small ones don't.
"""

from __future__ import annotations

import json
import random

# emotion -> the face's antic (moments.ANTICS), voiced vocalizations (VOCALS keys), the chirp (sounds: "chirp_<name>"),
# the base's gesture (omni_console gestures.GESTURES) and the arm's (omni_arm gestures.GESTURES); None: that channel
# stays still for this emotion
REACTIONS = {
    "happy": dict(antic="smile", vocal=("mmhm", "heh", "yay"), chirp="happy", base="wiggle", arm="nod"),
    "excited": dict(antic="bounce", vocal=("whoo", "yay", "ooh"), chirp="excited", base="shimmy", arm="perk"),
    "proud": dict(antic="proud", vocal=("tada", "heh", "yay"), chirp="proud", base="bounce", arm="flourish"),
    "affection": dict(antic="love", vocal=("aww", "mmhm"), chirp="love", base="wiggle", arm="nod"),
    "content": dict(antic="smile", vocal=("mmhm",), chirp="content", base=None, arm=None),
    "relieved": dict(antic="sigh", vocal=("phew",), chirp="relieved", base="settle", arm="settle"),
    "curious": dict(antic="tilt", vocal=("hmm_q", "ooh"), chirp="curious", base="lean", arm="tilt"),
    "surprised": dict(antic="pop", vocal=("oh", "whoa"), chirp="surprised", base="hop", arm="perk"),
    "startled": dict(antic="startle", vocal=("eep", "whoa"), chirp="startled", base="recoil", arm="flinch"),
    "focused": dict(antic=None, vocal=("mm",), chirp="focused", base=None, arm=None),
    "effort": dict(antic="strain", vocal=("hnngh",), chirp="effort", base=None, arm=None),
    "worried": dict(antic="glance", vocal=("uhoh",), chirp="worried", base="shrink", arm="shrink"),
    "frustrated": dict(antic="huff", vocal=("hmph", "ugh"), chirp="frustrated", base="huff", arm="shake"),
    "sad": dict(antic="droop", vocal=("aww_sad", "oh_no"), chirp="sad", base="sag", arm="droop"),
    "tired": dict(antic="yawn", vocal=("yawn",), chirp="tired", base=None, arm="droop"),
    "bored": dict(antic="sigh", vocal=("hmm",), chirp="bored", base=None, arm=None),
}
# a voiced vocalization: what it says (the voice renders it with the emotion's prosody: voice_server)
VOCALS = {
    "mmhm": "Mm-hm!", "heh": "Heh heh.", "yay": "Yay!", "whoo": "Whoo!", "ooh": "Ooh!", "tada": "Ta-da!",
    "aww": "Awww.", "phew": "Phew.", "hmm_q": "Hmm?", "oh": "Oh!", "whoa": "Whoa!", "eep": "Eep!", "mm": "Mm.",
    "hnngh": "Hnnngh!", "uhoh": "Uh-oh.", "hmph": "Hmph.", "ugh": "Ugh.", "aww_sad": "Aww...", "oh_no": "Oh no.",
    "yawn": "Hwaaah...", "hmm": "Hmmm.", "wheee": "Wheee!", "huh": "Huh?", "oops": "Oops.",
}
REACT_GAP = 1.5          # s: one reaction at a time
BODY_GAP = 10.0          # s between two body gestures at level 5 (shorter at higher levels)
SOUND_STYLES = ("mix", "voice", "chirps")


class Expressor:
    """react(emotion, intensity, now, levels) -> (antic or None, [outs as dicts]): what each channel does about it.
    Outs: {"kind": "vocal", "value": text, "emotion"}, {"kind": "sound", "value": "chirp_<emotion>"}, and
    {"kind": "express", "value": JSON {emotion, intensity, base, arm}} for the console (the body)."""

    def __init__(self, seed: int = 0):
        self.rng = random.Random(seed)
        self.t_react = -1e9
        self.t_body = -1e9
        self.recent: list = []

    def chance(self, often: float, k: float) -> bool:
        """Reacts with a chance that grows with its level (often: 0-2) and the moment's intensity k (0-1)."""
        return often > 0 and self.rng.random() < min(1.0, often * (0.35 + 0.65 * k))

    def react(self, emotion: str, k: float, now: float, levels, sound_style: str = "mix", body_ok: bool = True,
              force: bool = False) -> tuple:
        r = REACTIONS.get(emotion)
        if r is None or (now - self.t_react < REACT_GAP and not force):
            return None, []
        self.t_react = now
        outs, antic = [], None
        if r["antic"] and levels.level("face") >= 1 and (k >= 0.5 or self.chance(levels.often("face"), k)):
            antic = r["antic"]
        if self.chance(levels.often("sounds"), k):
            use_voice = sound_style == "voice" or (sound_style == "mix" and self.rng.random() < 0.55)
            if use_voice and r["vocal"]:
                key = self._fresh(r["vocal"])
                outs.append(dict(kind="vocal", value=VOCALS[key], emotion=emotion, prio=1))
            elif r["chirp"]:
                outs.append(dict(kind="sound", value=f"chirp_{r['chirp']}", prio=1))
        base = r["base"] if levels.level("base") >= 1 else None
        arm = r["arm"] if levels.level("arm") >= 1 else None
        gap = BODY_GAP / max(0.5, max(levels.often("base"), levels.often("arm")))
        if body_ok and (base or arm) and k >= 0.3 and now - self.t_body >= gap:
            base = base if self.chance(levels.often("base"), k) else None
            arm = arm if self.chance(levels.often("arm"), k) else None
            if base or arm:
                self.t_body = now
                outs.append(dict(kind="express", prio=1, value=json.dumps(dict(
                    emotion=emotion, intensity=round(k, 2), base=base, arm=arm))))
        return antic, outs

    def _fresh(self, keys) -> str:
        """Not the one it just made, if there is another."""
        fresh = [x for x in keys if x not in self.recent[-2:]] or list(keys)
        key = self.rng.choice(fresh)
        self.recent = (self.recent + [key])[-6:]
        return key
