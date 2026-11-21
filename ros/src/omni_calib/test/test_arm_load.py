"""omni_calib.arm_load (design.md 4.5): the fit finds a known arm from records made by its own model, the procedure
asks for its steps in order and checks the answers, the plan's poses are clear and light enough. No ROS, no sim (the
sim's whole run is simulation/v11 sil/studies/arm_load_cal.py; the C++ twin's parity is omni_cpp/test).
    python -m pytest -q ros/src/omni_calib/test/test_arm_load.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "omni_calib"), str(SRC / "omni_arm")]

from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path  # noqa: E402
from omni_calib import arm_load as AL  # noqa: E402

KIN = ArmKinematics(ArmConfig(config_path("d435")))


def truth_model(rng) -> AL.LoadModel:
    nom = AL.LoadModel.nominal(KIN.cfg)
    joints = [AL.JointLoad(A=0.11 * rng.uniform(0.9, 1.1), E=0.5 * rng.uniform(0.8, 1.2), J=0.02, o=rng.normal(0, 0.02),
                           F=0.06, mu=0.25, Fs=0.05, mus=0.3, zero=math.radians(rng.normal(0, 1.5)) if 1 <= k <= 3 else 0.0,
                           compliance=math.radians(0.8) if 1 <= k <= 3 else 0.0) for k in range(5)]
    links = [dict(body=l["body"], mass=l["mass"] * rng.uniform(0.9, 1.1), com=np.asarray(l["com"]) + rng.normal(0, 0.002, 3))
             for l in nom.links]
    return AL.LoadModel(joints, links, mount_dz=0.003)


def make_record(m: AL.LoadModel, kind, q_enc, d, rng, weights, weight=None, holdout=False, v=0.0, joint=None):
    """A record the truth model explains: the duty that balances gravity, the weight and the friction's side."""
    g = np.array([0.0, 0.0, -9.81])
    q = AL.true_angles(m, KIN, q_enc, g)
    grav = KIN.gravity(q, g, m.links)[:5]
    pay = np.zeros(5)
    if weight:
        w = weights[weight]
        pay = KIN.point_torque(q, w["kg"] * g, w["point"])[:5]
    volts, temp = np.full(6, 11.8), np.full(6, 30.0)
    gv = np.zeros(6)
    if kind == "sweep":
        gv[joint] = v
    duty = np.zeros(6)
    for j in range(5):
        L = m.joints[j]
        fb = (L.F + L.mu * abs(grav[j] + pay[j])) if kind == "sweep" else (L.Fs + L.mus * abs(grav[j] + pay[j]))
        tm = -grav[j] - pay[j] + d[j] * fb + rng.normal(0, 0.01)
        duty[j] = (tm + L.E * gv[j] + L.o) * (1 + L.alpha * (temp[j] - 25)) / (L.A * volts[j])
    return AL.Record(kind=kind, q=np.asarray(q_enc, float), duty=duty, volts=volts, temp=temp, current=np.zeros(6),
                     w=gv.copy(), goal_vel=gv, goal_acc=np.zeros(6), g=g, dir=np.asarray(d, float), weight=weight,
                     holdout=holdout)


@pytest.fixture(scope="module")
def synthetic():
    return make_synthetic()


def make_synthetic():
    rng = np.random.default_rng(4)
    m = truth_model(rng)
    steps = AL.plan(KIN, seed=2, quick=True)
    weights = {"A": dict(kg=0.152, point=AL.hang_point(10.0)), "B": dict(kg=0.318, point=AL.hang_point(12.0))}
    recs, k = [], 0
    for s in steps:
        if s["op"] == "hold":
            recs.append(make_record(m, "hold", s["q"], np.full(6, s["dir"]), rng, weights, s["weight"], s["holdout"]))
        elif s["op"] == "sweep":
            j = s["joint"]
            d = np.zeros(6)
            d[j] = math.copysign(1, s["q1"][j] - s["q0"][j])
            for f in np.linspace(0.2, 0.8, 6):
                q = s["q0"] + f * (s["q1"] - s["q0"])
                recs.append(make_record(m, "sweep", q, d, rng, weights, v=d[j] * s["speed"], joint=j))
        elif s["op"] == "ruler":
            q = AL.true_angles(m, KIN, s["q"], [0, 0, -9.81])
            r = make_record(m, "hold", s["q"], np.ones(6), rng, weights, s["weight"])
            r.kind = "ruler"
            r.value = AL._tip_height(KIN, m, s["q"], [0, 0, -9.81], weights, s["weight"]) * 1000 + rng.normal(0, 0.6)
            recs.append(r)
            k += 1
    return m, recs, weights


def test_the_fit_finds_a_known_arm(synthetic):
    m, recs, weights = synthetic
    res = AL.fit(KIN, recs, weights)
    fm, met = res["model"], res["metrics"]
    for a, b in zip(fm.links, m.links):                       # the masses within 10 %
        assert a["mass"] == pytest.approx(b["mass"], rel=0.10)
    for j in (1, 2, 3):                                       # the joints that see gravity: gains within 8 %
        assert fm.joints[j].A == pytest.approx(m.joints[j].A, rel=0.08)
    for j in (0, 4):                                          # pan and roll: held near the loaded joints' servo
        assert fm.joints[j].A == pytest.approx(m.joints[j].A, rel=0.20)
    # the still torque against the truth on held-out holds: the friction's gravity share is fitted on the nominal
    # masses' torques, so a few hundredths remain
    assert met["static_rms_nm"] < 0.08 and met["static_rms_nm"] < 0.2 * met["nominal_static_rms_nm"]
    assert met["weight_err_n"] < 0.6                          # of a 1.5 / 3.1 N weight
    for j in (1, 2):                                          # the zeros the heights see well, within 1 deg
        assert abs(math.degrees(fm.joints[j].zero - m.joints[j].zero)) < 1.0
    assert met["ruler_rms_mm"] < 2.0                          # leave-one-out heights (the person reads 0.6 mm)
    js = res["value"]
    assert set(js["joints"]) == set(KIN.cfg.joints[:5]) and js["model"] == "omni_arm_load/1"


def test_the_model_round_trips_through_its_json(synthetic, tmp_path):
    m, _, _ = synthetic
    p = tmp_path / "m.json"
    import json
    p.write_text(json.dumps(m.to_json(KIN.cfg), default=float))
    back = AL.LoadModel.load(str(p), KIN.cfg)
    assert back.joints[2].A == pytest.approx(m.joints[2].A) and back.mount_dz == pytest.approx(m.mount_dz)
    q = np.radians([10, -20, 30, 15, 5, -10])
    assert np.allclose(AL.encoder_goals(back, KIN, AL.true_angles(back, KIN, q)), q, atol=2e-5)


def test_still_inside_the_band_is_no_outside_force():
    m = AL.LoadModel.nominal(KIN.cfg)
    q = np.radians([0, -30, 40, 20, 0, -10])
    g = KIN.gravity(q)
    duty = np.zeros(6)
    duty[:5] = -g[:5] / (m.joints[0].A * 12.0)                # exactly holding gravity
    e = AL.estimate(m, KIN, q, np.zeros(6), duty, np.full(6, 12.0), np.full(6, 25.0), np.zeros(6), np.zeros(6),
                    np.zeros(6))
    assert np.allclose(e["ext"][:5], 0.0) and not e["moving"].any()
    duty[1] += 0.5                                            # a push far past the band
    e = AL.estimate(m, KIN, q, np.zeros(6), duty, np.full(6, 12.0), np.full(6, 25.0), np.zeros(6), np.zeros(6),
                    np.zeros(6))
    assert e["ext"][1] < -0.5 and e["band"][1] > 0
    # the speed register's flicker at rest is no motion; a goal moving is
    e = AL.estimate(m, KIN, q, np.full(6, 0.15), duty, np.full(6, 12.0), np.full(6, 25.0), np.zeros(6), np.zeros(6),
                    np.zeros(6))
    assert not e["moving"].any()


class FakeArm:
    """Moves arrive at once; the telemetry is constant."""

    def __init__(self):
        self.until = -1.0

    def obs(self, t):
        return dict(t=t, q_enc=[0.0] * 6, duty=[0.0] * 6, volts=[12.0] * 6, temp=[30.0] * 6, current=[0.0] * 6,
                    w=[0.0] * 6, goal_vel=[0.0] * 6, goal_acc=[0.0] * 6, g=[0, 0, -9.81], moving=t < self.until, ok=True)


def test_the_procedure_asks_in_order_and_checks_the_answers():
    cal = AL.ArmLoadCal(KIN, seed=1, quick=True)
    arm, t, prompts, refused = FakeArm(), 0.0, [], 0
    answers = dict(confirm=True, weigh=152.0, hang=11.5, unhang=True, ruler=42.0, pull=0.5, release=True)
    while not cal.done and t < 3000:
        t += 0.02
        for a in cal.step(t, arm.obs(t)):
            if a["op"] in ("move", "line"):
                arm.until = t + 0.3
            elif a["op"] == "prompt":
                prompts.append(a["kind"])
                if a["kind"] == "weigh":
                    ok, why = cal.answer(5000)                 # 5 kg: refused, ask again
                    assert not ok and "range" in why
                    refused += 1
                assert cal.answer(answers[a["kind"]])[0]
    assert cal.done and refused == 2
    assert prompts[0] == "confirm"
    assert prompts.index("weigh") < prompts.index("hang") < prompts.index("unhang")
    assert prompts.count("ruler") >= 6 and "pull" in prompts and prompts[-1] == "release"
    kinds = {r.kind for r in cal.records}
    assert {"hold", "sweep", "ruler", "pull"} <= kinds
    assert cal.weights["A"]["kg"] == pytest.approx(0.152) and cal.weights["B"]["point"][2] == pytest.approx(-0.104 + 0.0115)


def test_the_plans_poses_are_clear_and_light():
    steps = AL.plan(KIN, seed=0, quick=True)
    nom = AL.LoadModel.nominal(KIN.cfg)
    for s in steps:
        if s["op"] in ("hold", "ruler", "pull"):
            assert KIN.clear(s["q"], floor_margin=0.004)[0]
        if s["op"] == "hold":
            w = {"A": 0.15, "B": 0.32}.get(s["weight"], 0.0)   # what plan() allowed for each
            tq = KIN.gravity(s["q"], links=nom.links)[:5] + KIN.point_torque(s["q"], [0, 0, -9.81 * w], AL.hang_point(12))[:5]
            assert np.abs(tq).max() <= 1.1 + 1e-9
    assert sum(s["op"] == "hold" and s["holdout"] for s in steps) >= 4      # some kept back for the check
