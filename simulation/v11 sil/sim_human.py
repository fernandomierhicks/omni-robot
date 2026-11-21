"""The person who helps the arm load calibration (omni_calib arm_load, design.md 4.5), with Fernando's tools and their
errors (est): what they do to the detailed arm (arm_detailed.DetailedArm) and what they read off and type in.

  kitchen scale   1 g steps, +-0.3 % (its calibration) and 0.4 g of reading
  calipers        on a string's middle: 0.35 mm (where the middle of a soft string is), shown to 0.1 mm
  ruler           upright beside the fingertip: 0.6 mm reading, +-0.5 mm parallax, read to the mm
  luggage scale   he says +-0.5 lb: 0.11 kg rms, shown to 0.05 kg; the pull itself 8 deg off level ahead, 5 deg up/down
The weights: ~150 g and ~320 g (a bag of coins, a small bottle) on a string looped over the fixed finger 6-16 mm from
its end. A person takes a few seconds to answer (the arm holds meanwhile: its servos warm).

    h = ArmCalHuman(arm, base_R=lambda: R, seed=3)
    value = h.answer(prompt, t)       # does what the prompt asks (hangs a weight, pulls), returns what they'd type
"""

from __future__ import annotations

import math

import numpy as np

import arm_detailed as AD

TIP_END = np.array([-0.0135, 0.0, -0.104])      # the fixed finger's very end (gripper frame; omni_calib arm_load.TIP)


class ArmCalHuman:
    def __init__(self, arm: "AD.DetailedArm", base_R, seed: int = 0):
        self.arm = arm
        self.base_R = base_R                     # () -> the robot base's rotation in the world (3 x 3)
        self.rng = rng = np.random.default_rng(seed + 5150)
        self.weights = dict(A=0.15 + rng.uniform(-0.02, 0.02), B=0.32 + rng.uniform(-0.02, 0.02))
        self.hang_mm = {k: rng.uniform(6.0, 16.0) for k in self.weights}
        self.current = None
        self.log: list = []

    def delay(self, prompt: dict) -> float:
        """Seconds before they answer."""
        return dict(confirm=3.0, weigh=6.0, hang=12.0, unhang=4.0, ruler=5.0, pull=6.0, release=2.0).get(
            prompt.get("kind"), 3.0) * self.rng.uniform(0.8, 1.3)

    def truth(self) -> dict:
        return dict(weights=self.weights, hang_mm=self.hang_mm,
                    points={k: (TIP_END + [0, 0, v / 1000.0]).tolist() for k, v in self.hang_mm.items()})

    def answer(self, prompt: dict):
        kind, key, rng = prompt.get("kind"), prompt.get("key"), self.rng
        if kind == "weigh":
            g = self.weights[key] * 1000 * (1 + rng.normal(0, 0.003)) + rng.normal(0, 0.4)
            v = float(round(g))
        elif kind == "hang":
            self.arm.hang(self.weights[key], TIP_END + [0, 0, self.hang_mm[key] / 1000.0])
            self.current = key
            v = round(self.hang_mm[key] + rng.normal(0, 0.35), 1)
        elif kind == "unhang":
            self.arm.unhang()
            self.current = None
            v = True
        elif kind == "ruler":
            h = self.arm.tip_height() * 1000 + rng.normal(0, 0.6) + rng.uniform(-0.5, 0.5)
            v = float(round(h))
        elif kind == "pull":
            R = self.base_R()
            yaw, pitch = math.radians(rng.normal(0, 8)), math.radians(rng.normal(0, 5))
            d_base = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)])
            kg = 0.5 + rng.normal(0, 0.05)
            f = R @ d_base * kg * 9.81
            self.arm.push(f, TIP_END)
            self.pulled = dict(kg=kg, force_base=(d_base * kg * 9.81).tolist())
            v = round((kg + rng.normal(0, 0.11)) / 0.05) * 0.05
        elif kind == "release":
            self.arm.push(None, None)
            v = True
        else:
            v = True
        self.log.append(dict(kind=kind, key=key, value=v))
        return v
