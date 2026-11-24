"""Go and check a room or place for people ("is anyone in the kitchen?", "go check the door"): drive there, look
round a quarter turn at a time, then come back and report."""

SKILL = dict(name="check_room", says=["is anyone in the kitchen", "go check the front door", "who's in the office"],
             params={"place": {"type": "string", "description": "a saved place or a room"},
                     "come_back": {"type": "boolean", "description": "come back to report (default true)"}},
             required=["place"], moves=True, max_s=600)


def run(robot, place, come_back=True):
    places, rooms = robot.places(), robot.rooms()
    start = robot.pose()
    if place in places:
        r = robot.goto(place=place)
    elif place in rooms:
        r = robot.goto(x=rooms[place]["x"], y=rooms[place]["y"])
    else:
        return f"I don't know where {place} is."
    if not r["ok"]:
        return f"I couldn't get to {place}: {r['message']}"
    seen = 0
    for k in range(4):
        sure = [p for p in robot.people() if p["status"] == "person" and p["range_m"] is not None
                and p["range_m"] < 5.0]
        seen = max(seen, len(sure))
        if seen:
            break
        if k < 3:
            robot.turn(deg=90)
    if come_back and start is not None:
        robot.goto(x=start["x"], y=start["y"], yaw_deg=start["yaw_deg"])
    if seen:
        return f"Yes, I saw {'someone' if seen == 1 else str(seen) + ' people'} at {place}."
    return f"I looked all round {place} and didn't see anyone."
