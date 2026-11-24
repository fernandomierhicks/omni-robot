"""Watch a place (the front door by default) for a while: drive there, face the way the place faces, and say so
whenever someone new turns up. Ends after the minutes asked (at most 14)."""

SKILL = dict(name="watch_door", says=["watch the door", "keep an eye on the front door", "guard the hallway"],
             params={"place": {"type": "string", "description": "a saved place (default: front door)"},
                     "minutes": {"type": "number", "description": "how long to watch, 0.5-14"}},
             moves=True, max_s=900)


def run(robot, place="front door", minutes=5.0):
    minutes = max(0.5, min(14.0, float(minutes)))
    if place not in robot.places():
        return f"I don't have a place called {place}."
    r = robot.goto(place=place)
    if not r["ok"]:
        return f"I couldn't get to the {place}: {r['message']}"
    robot.say(text=f"Watching the {place}.", feeling="focused")
    t_end = robot.now() + minutes * 60.0
    was, visits = 0, 0
    while robot.now() < t_end:
        n = len([p for p in robot.people() if p["status"] == "person"])
        if n > was:
            visits += 1
            robot.say(text=f"Someone's at the {place}.", feeling="surprised")
        was = n
        robot.wait(s=3)
    return f"Done watching the {place}: {visits} {'visit' if visits == 1 else 'visits'}."
