"""Show someone around: visit the saved places (or the ones named) one by one and say where we are at each, then
come back to where it started."""

SKILL = dict(name="show_around", says=["show me around", "give me a tour", "show me the house"],
             params={"places": {"type": "array", "items": {"type": "string"},
                                "description": "which saved places, in order; empty: all of them"}},
             moves=True, max_s=900)


def run(robot, places=None):
    known = robot.places()
    start = robot.pose()
    todo = [p for p in (places or sorted(known)) if p in known]
    if not todo:
        return "There are no saved places to show yet."
    robot.say(text=f"Follow me. {len(todo)} stops.", feeling="happy")
    seen = []
    for name in todo:
        r = robot.goto(place=name)
        if not r["ok"]:
            robot.say(text=f"I can't get to the {name} right now.", feeling="worried")
            continue
        seen.append(name)
        robot.say(text=f"This is the {name}.", feeling="content")
        robot.wait(s=2)
    if start is not None:
        robot.goto(x=start["x"], y=start["y"], yaw_deg=start["yaw_deg"])
    return f"Tour done: {', '.join(seen) or 'nowhere, sorry'}."
