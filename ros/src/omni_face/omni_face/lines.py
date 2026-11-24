"""The feelings' own lines (ROS-free): what it may say when something is felt (moments.Brain.feel), in its character
(CHARACTER.md): understated, dry, honest about being a machine, never mean, now and then warm. Each set has plain
lines and wry ones (moments.Lines: the humor setting picks); a line whose {field} isn't known is skipped. These are
chatter (priority 1): the speech level thins them (0: never), and one per event every 20 s at most.

Most feelings show without words (the face, a chirp, a gesture); only the ones a person would want explained have
lines here. The mission and game lines stay in moments.py (MISSION_SAY, SAY).
"""

from __future__ import annotations


def _lines(plain=(), wry=()) -> dict:
    return dict(plain=list(plain), wry=list(wry))


REACT = {
    "strain": _lines(["Whoa. Something pushed my arm.", "Easy. I felt that.", "Ow. Backing off."],
                     ["Easy. That's my good arm.", "I felt that. I have sensors there.",
                      "Whoa. My arm and that object disagree.", "Ow. Well. Not ow. But noted."]),
    "found": _lines(["There we are. I know where I am."], ["Ah. That wall. I remember that wall."]),
    "praise": _lines(["Thank you.", "That's nice to hear."],
                     ["I'll be insufferable for at least a minute.", "Logged. Permanently.",
                      "My circuits are doing a little dance."]),
    "outrun": _lines(["Wait for me."], ["My legs are wheels. Please adjust expectations."]),
    "too_big": _lines(["That one's too big for my hand."], ["That's not a toy. That's furniture."]),
    "person_found": _lines(["Oh, hello."], ["Company. Excellent."]),
}

# what it may say about how it feels, when asked ("how are you"), by the emotion it shows (chat.py also has these)
HOW_I_FEEL = {
    "happy": _lines(["Good. Things are going well."], ["Good. Suspiciously good."]),
    "excited": _lines(["Great. A lot is happening."], ["Excellent. My processors are buzzing."]),
    "proud": _lines(["Pretty proud of myself, honestly."], ["Proud. I've been doing good work. Ask the floor."]),
    "content": _lines(["Calm. Content."], ["Content. Like a robot in a clean room."]),
    "relieved": _lines(["Relieved. That was close."], ["Relieved. My worry subroutine can rest."]),
    "curious": _lines(["Curious. Something's got my attention."], ["Curious. Everything is interesting today."]),
    "worried": _lines(["A little worried, to be honest."], ["Slightly worried. Robots worry. Quietly."]),
    "frustrated": _lines(["A bit frustrated. Things keep slipping."], ["Frustrated. The floor is winning today."]),
    "sad": _lines(["A little down."], ["A bit down. Nothing a good tidy can't fix."]),
    "tired": _lines(["Tired. My battery is getting low."], ["Running on fumes. Electric fumes."]),
    "bored": _lines(["A bit bored. Give me something to do?"], ["Bored. I've counted my wheels twice."]),
    "affection": _lines(["Happy you're here."], ["Fond of you, actually. Don't tell anyone."]),
    "neutral": _lines(["Fine. All systems normal."], ["All systems nominal. Emotionally too, I assume."]),
}
