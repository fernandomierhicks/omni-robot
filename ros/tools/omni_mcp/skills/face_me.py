"""Turn to face the nearest person (the one who asked)."""

SKILL = dict(name="face_me", says=["look at me", "face me", "turn to me"], params={}, moves=True, max_s=60)


def run(robot):
    p = robot.nearest_person()
    if p is None:
        robot.turn(deg=180)                       # nobody ahead: look behind once
        p = robot.nearest_person()
        if p is None:
            return "I looked around but couldn't see anyone."
    robot.turn(deg=p["bearing_deg"])
    return "Facing you."
