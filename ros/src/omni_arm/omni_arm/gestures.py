"""The arm's feelings (ROS-free): short gestures (a nod, a head tilt, a flinch) as keyframes around wherever the arm
is commanded, checked like any other motion before they play.

omni_face asks (expression.REACTIONS) through the console's `express` verb, which passes it on only while the arm is
free (no task, no calibration, holding still); arm_brain's `express` verb refuses it again if the arm is busy or
holds something. The wrist is its "head" (the wrist camera looks out of it), so most gestures are the wrist's: a nod
is wrist flex, a tilt or a head shake is wrist roll; the gripper's open-and-close is its "mouth".

Every gesture ends exactly where it started. Each keyframe is (seconds, {joint: degrees}); the offsets scale with the
size (0-1: the console's emotion.arm level / 10 times the moment's intensity); the keys are played with the
motions' quintic ease between them (slow in, slow out: animation's easing for free), and the whole track goes through
motion.check_trajectory (joint ranges, the servos' speed with the quintic's peak, the robot's keep-outs): a gesture
that would hit anything is refused whole. The gripper moves only when nothing is held.
"""

from __future__ import annotations

import math

import numpy as np

from omni_arm import motion as mo

# name -> [(t s, {joint: deg at size 1})]: the offsets at each key from where the arm was (the last key is all zero)
GESTURES = {
    "nod": [(0.35, {"wrist_flex": -18.0}), (0.7, {"wrist_flex": 8.0}), (1.15, {})],
    "tilt": [(0.45, {"wrist_roll": 22.0}), (1.5, {"wrist_roll": 22.0}), (2.0, {})],
    "perk": [(0.35, {"wrist_flex": -14.0, "shoulder_lift": -5.0, "gripper": 12.0}),
             (0.8, {"wrist_flex": -14.0, "shoulder_lift": -5.0}), (1.3, {})],
    "flourish": [(0.35, {"wrist_roll": 25.0, "gripper": 10.0}), (0.8, {"wrist_roll": -25.0}),
                 (1.1, {"wrist_roll": -25.0, "gripper": 10.0}), (1.5, {})],
    # startled: the "head" jerks up and back (curling in instead hits the arm's adapter from ready: checked)
    "flinch": [(0.2, {"shoulder_lift": -7.0, "wrist_flex": -14.0}), (0.5, {"shoulder_lift": -7.0, "wrist_flex": -14.0}),
               (1.3, {})],
    "shake": [(0.25, {"wrist_roll": 15.0}), (0.5, {"wrist_roll": -15.0}), (0.75, {"wrist_roll": 12.0}),
              (1.0, {"wrist_roll": -8.0}), (1.3, {})],
    "droop": [(0.9, {"wrist_flex": 20.0, "shoulder_lift": 5.0}), (1.7, {"wrist_flex": 20.0, "shoulder_lift": 5.0}),
              (2.5, {})],
    "shrink": [(0.6, {"elbow_flex": 8.0, "wrist_flex": 10.0}), (1.4, {"elbow_flex": 8.0, "wrist_flex": 10.0}),
               (2.0, {})],
    "settle": [(0.6, {"wrist_flex": 6.0}), (1.4, {})],
    "chatter": [(0.15, {"gripper": 14.0}), (0.3, {}), (0.45, {"gripper": 14.0}), (0.6, {})],
}


def track(kin, q0, name: str, size: float, holding: bool = False):
    """(times from 0, waypoints from q0) of the gesture at this size, checked; or (None, why)."""
    keys = GESTURES.get(name)
    if keys is None:
        return None, f"not a gesture ({', '.join(GESTURES)})"
    size = max(0.0, min(1.0, float(size)))
    if size <= 0.0:
        return None, "size 0"
    joints = list(kin.cfg.joints)
    q0 = np.asarray(q0, float)
    points = []
    for t, offs in keys:
        q = q0.copy()
        for j, deg in offs.items():
            if j == "gripper" and holding:
                continue                                  # it holds something: the hand stays shut
            q[joints.index(j)] += math.radians(deg) * size
        points.append((t, list(q)))
    vmax = float(np.min(np.atleast_1d(kin.cfg.servos["speed"]))) / 1.875      # the quintic's peak stays under it
    times, wps = mo.check_trajectory(kin, q0, joints, points, max_speed=vmax)
    if times is None:
        return None, wps
    return times, wps


def motion(kin, q0, name: str, size: float, t0: float, holding: bool = False):
    """A motion.Motion playing the gesture from t0 (the keys' own times, eased between), or (None, why)."""
    times, wps = track(kin, q0, name, size, holding)
    if times is None:
        return None, wps
    m = mo.Motion(wps, mo.speeds(kin), t0)
    m.t = [t0 + float(s) for s in times]
    return m, ""
