"""Come to the person who asked: drive to about 1 m in front of the nearest person, facing them."""

SKILL = dict(name="come_here", says=["come to me", "come over here", "come back to me"],
             params={"stop_m": {"type": "number", "description": "how far from them to stop, m (0.6-2)"}},
             moves=True, max_s=150)


def run(robot, stop_m=1.0):
    stop_m = max(0.6, min(2.0, float(stop_m)))
    p = robot.nearest_person()
    if p is None:
        robot.turn(deg=180)                       # nobody ahead: look behind once
        p = robot.nearest_person()
    if p is None:
        return "I couldn't see anyone to come to."
    if p["x"] is None:                            # no map pose: turn to them and drive straight
        robot.turn(deg=p["bearing_deg"])
        if p["range_m"] - stop_m > 0.1:
            r = robot.move(steps=[{"do": "forward", "m": min(3.0, p["range_m"] - stop_m)}])
        else:
            r = {"ok": True}
    else:
        r = robot.approach(x=p["x"], y=p["y"], stop_m=stop_m)
    if not r["ok"]:
        return f"I tried to come over but got stuck: {r['message']}"
    return "Here I am."
