"""Look for a thing on the floor by colour and shape ("where's the red block?"): look ahead, then turn a quarter at
a time, and say where it is (ahead, left, right, how far). Doesn't pick it up."""

SKILL = dict(name="where_is", says=["where's the red block", "can you see the blue ball", "find my green crayon"],
             params={"color": {"type": "string",
                               "description": "red orange yellow green blue purple pink white grey black, or empty"},
                     "shape": {"type": "string", "enum": ["square", "round", "long", "any"]}},
             moves=True, max_s=120)

WORDS = {"square": "block", "round": "ball", "long": "stick", "any": "thing"}


def side(bearing):
    if abs(bearing) < 12:
        return "straight ahead"
    way = "left" if bearing > 0 else "right"
    return f"a bit to my {way}" if abs(bearing) < 35 else f"to my {way}"


def run(robot, color="", shape="any"):
    what = f"{color + ' ' if color else ''}{WORDS.get(shape, 'thing')}"
    for k in range(4):
        found = [f for f in robot.find(color=color or None, shape=shape) if (f["score"] or 0) >= 0.5]
        if found:
            f = found[0]
            far = f"about {round(f['range_m'], 1)} metres away" if f["range_m"] is not None else "nearby"
            return f"The {what} is {side(f['bearing_deg'] or 0.0)}, {far}." + \
                (" I turned to look, so that's from where I'm facing now." if k else "")
        if k < 3:
            robot.turn(deg=90)
    return f"I looked all round and couldn't see a {what} on the floor."
