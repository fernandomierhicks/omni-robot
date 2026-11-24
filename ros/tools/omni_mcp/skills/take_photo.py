"""Take a picture: of the person who asked (turns to them and looks up with the wrist camera) or of what is ahead.
The picture is saved on the PC (the photos folder) and the reply says where."""

SKILL = dict(name="take_photo", says=["take a picture of me", "take a photo", "take a picture of this"],
             params={"of": {"type": "string", "enum": ["me", "ahead"],
                            "description": "me: the nearest person; ahead: what is in front of the robot"}},
             moves=True, max_s=90)


def run(robot, of="me"):
    if of == "me":
        p = robot.nearest_person()
        if p is None:
            return "I can't see you, so no picture."
        robot.turn(deg=p["bearing_deg"])
    robot.arm_pose(name="look")                   # the wrist camera: straight ahead and a little up, 0.57 m high
    robot.say(text="Say cheese.", feeling="happy")
    robot.wait(s=1.5)
    try:
        shot = robot.photo(camera="wrist")
    except RobotError as e:
        robot.arm_pose(name="stow")
        return f"My camera didn't give me a picture ({e})."
    robot.arm_pose(name="stow")
    robot.gesture(arm="perk", feeling="happy")
    return f"Got it. The picture is saved as {shot['path']}."
