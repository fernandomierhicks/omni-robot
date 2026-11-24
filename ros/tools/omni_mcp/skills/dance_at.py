"""Go to a saved place or room (or stay here) and have a dance party there."""

SKILL = dict(name="dance_at", says=["go dance in the kitchen", "dance over there", "go dance"],
             params={"place": {"type": "string", "description": "a saved place or room; empty: dance here"},
                     "routine": {"type": "string", "enum": ["groove", "party", "robot"]}},
             moves=True, max_s=900)


def run(robot, place="", routine="party"):
    if place:
        places, rooms = robot.places(), robot.rooms()
        if place in places:
            r = robot.goto(place=place)
        elif place in rooms:
            r = robot.goto(x=rooms[place]["x"], y=rooms[place]["y"])
        else:
            return f"I don't know where {place} is. I know: {', '.join(sorted(places) + sorted(rooms))}."
        if not r["ok"]:
            return f"I couldn't get to {place}: {r['message']}"
    r = robot.mission(name="dance", args={"routine": routine})
    return "That was fun." if r["ok"] else f"The dance stopped early: {r['message']}"
