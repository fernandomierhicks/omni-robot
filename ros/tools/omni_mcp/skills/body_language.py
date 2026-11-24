"""Answer with the body: wave (an arm flourish: there is no real wave yet), nod, shake the head (the wrist), tilt
the head. Plays only when the arm is free and its Emotion level allows."""

SKILL = dict(name="body_language", says=["wave", "nod", "shake your head", "wave hello", "tilt your head"],
             params={"do": {"type": "string", "enum": ["wave", "nod", "shake_head", "tilt_head"]}},
             required=["do"], moves=False, max_s=20)

ARM = {"wave": "flourish", "nod": "nod", "shake_head": "shake", "tilt_head": "tilt"}
FEEL = {"wave": "happy", "nod": "content", "shake_head": "neutral", "tilt_head": "curious"}


def run(robot, do="wave"):
    if do not in ARM:
        return f"I can wave, nod, shake my head or tilt it, not {do}."
    r = robot.gesture(arm=ARM[do], feeling=FEEL[do])
    if do == "wave":
        robot.say(text="Hi!", feeling="happy")
    return "Done." if r["ok"] else f"My arm couldn't: {r['message']}"
