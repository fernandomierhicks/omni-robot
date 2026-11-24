"""Celebrate: a happy wiggle, an arm flourish and a cheer (only the parts of the body that are free play)."""

SKILL = dict(name="celebrate", says=["celebrate", "do a happy dance", "woohoo"], params={}, moves=False, max_s=20)


def run(robot):
    robot.gesture(base="wiggle", arm="flourish", feeling="excited")
    robot.say(text="Woo hoo!", feeling="excited", intensity=0.9)
    return "Celebrated."
