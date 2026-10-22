"""Randomized worlds: catch-rate ranges instead of single numbers, and which guesses matter (design.md 23.3).

Every study so far used one fixed guess per unmeasured parameter: ball 74 mm / 22 g / Cd 0.5, restitution, grip,
net rolling resistance, floor friction, camera noise, ball spin. Here each world draws all 17 of them at once on a
Latin hypercube over their plausible ranges (randomize.PARAMS, all est). The robot doesn't know the draw: it is
built for the nominal ball, predicts with the catalog ball, uses the nominal launcher table and acceleration table.

  catch: the round-3 robot (chute, GM3506 feed, viscoelastic contacts) in firmware adult mode, the 6-7 m shallow
         throws of shallow_sweep.py (3 distances x 4 apexes x 3 tosses = 36 per world);
  returns: return throws to a thrower 6.5 m away (apex 1.5 m, 8 tosses per world, range bias learned as it goes).

Per parameter, three views of its effect: the per-throw logistic-regression coefficient (change in log-odds of a
catch per full range, with the throw's distance and apex as controls), the world-level Spearman correlation, and
the catch rate in the bottom vs top third of its range.

  python randomized.py                 # catch worlds (default 192) and return worlds (default 96)
  python randomized.py catch 192       # only one part, n worlds
Writes results/randomized.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import RESULTS, SimParams, save  # noqa: E402
from catch_game import GameConfig  # noqa: E402
from robot_model import build_model  # noqa: E402
from shallow_sweep import APEXES, AREA, DISTANCES, EXPECTED, FW, FWG, HOME, R3, V6G, HumanToss  # noqa: E402
import randomize  # noqa: E402

TOSSES = 3
RET_DIST, RET_APEX, RET_TOSSES = 6.5, 1.5, 8


def _game(w, dist, apex, seed, returns):
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, **R3, **FW)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    w = w or randomize.nominal()
    cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex),
                               return_throws=returns, wait_between=1.2, perception="vision",
                               vision_cfg=randomize.vision_cfg(EXPECTED, w), toss_spin=w["spin"]), **FWG, **V6G})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist)
    randomize.apply(game.robot, w, game.perc)
    return p, d, game


def catch_cell(args):
    wi, w, dist, apex, seed = args
    p, d, game = _game(w, dist, apex, seed, returns=False)
    rim_margin = p.funnel_catch_radius - p.ball_spec.radius
    while len(game.results) < TOSSES and d.time < 120:
        game.step()
    out = []
    for r in game.results:
        why = "caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or r.pos_error > rim_margin else "bounce")
        out.append(dict(caught=bool(r.caught), why=why, pos_error=r.pos_error, pred_err=r.pred_err))
    return dict(world=wi, distance=dist, apex=apex, tosses=out)


def return_cell(args):
    wi, w, seed = args
    p, d, game = _game(w, RET_DIST, RET_APEX, seed, returns=True)
    while (len(game.results) < RET_TOSSES or game.state != "wait") and d.time < 200:
        game.step()
    return dict(world=wi, tosses=[dict(caught=r.caught, launched=r.launched, err=r.return_error,
                                       range_err=r.return_range_err) for r in game.results], bias=game.range_bias)


# -- analysis -------------------------------------------------------------------

def _logit_effects(X, y, controls, world):
    """Per-throw logistic regression on the parameters scaled to 0..1 over their ranges, with controls. The error is
    a cluster bootstrap over worlds: the throws of one world share its draw, so resampling throws understates it
    (it made dropped frames look helpful at 4.6 sigma; a direct A/B shows no effect)."""
    from sklearn.linear_model import LogisticRegression
    if y.min() == y.max():
        return np.zeros(X.shape[1]), np.zeros(X.shape[1])
    Z = np.hstack([X, controls])
    fit = LogisticRegression(C=10.0, max_iter=5000).fit(Z, y)
    coef = fit.coef_[0][:X.shape[1]]
    rng = np.random.default_rng(1)
    ids = np.unique(world)
    rows = {w: np.nonzero(world == w)[0] for w in ids}
    boot = []
    for _ in range(200):
        i = np.concatenate([rows[w] for w in rng.choice(ids, len(ids))])
        if y[i].min() == y[i].max():
            continue
        boot.append(LogisticRegression(C=10.0, max_iter=5000).fit(Z[i], y[i]).coef_[0][:X.shape[1]])
    return coef, np.std(boot, axis=0)


def _scaled(worlds):
    return np.array([[(w[q.name] - q.lo) / (q.hi - q.lo) if not q.log else
                      (math.log(w[q.name]) - math.log(q.lo)) / (math.log(q.hi) - math.log(q.lo))
                      for q in randomize.PARAMS] for w in worlds])


def analyze_catch(worlds, rows):
    from scipy.stats import spearmanr
    n = len(worlds)
    rate = np.full(n, np.nan)
    by = {i: [] for i in range(n)}
    for r in rows:
        by[r["world"]] += [t["caught"] for t in r["tosses"]]
    for i in range(n):
        rate[i] = np.mean(by[i]) if by[i] else np.nan
    S = _scaled(worlds)
    X, y, ctl, wid = [], [], [], []
    for r in rows:
        for t in r["tosses"]:
            X.append(S[r["world"]])
            y.append(t["caught"])
            ctl.append([DISTANCES.index(r["distance"]) / 2, APEXES.index(r["apex"]) / 3])
            wid.append(r["world"])
    X, y, ctl, wid = np.array(X), np.array(y, dtype=int), np.array(ctl), np.array(wid)
    coef, se = _logit_effects(X, y, ctl, wid)
    eff = []
    for k, q in enumerate(randomize.PARAMS):
        rho = spearmanr(S[:, k], rate).statistic
        lo, hi = S[:, k] < 1 / 3, S[:, k] > 2 / 3
        eff.append(dict(name=q.name, coef=float(coef[k]), se=float(se[k]), rho=float(rho),
                        low=float(np.nanmean(rate[lo])), high=float(np.nanmean(rate[hi]))))
    eff.sort(key=lambda e: -abs(e["coef"]))
    whys = [t["why"] for r in rows for t in r["tosses"]]
    return dict(rate=rate.tolist(), n_throws=int(len(y)), caught=float(y.mean()),
                p10=float(np.nanpercentile(rate, 10)), p50=float(np.nanpercentile(rate, 50)),
                p90=float(np.nanpercentile(rate, 90)), min=float(np.nanmin(rate)), max=float(np.nanmax(rate)),
                reach=whys.count("reach"), bounce=whys.count("bounce"), effects=eff)


def analyze_returns(worlds, rows):
    from scipy.stats import spearmanr
    n = len(worlds)
    med = np.full(n, np.nan)
    for r in rows:
        e = [t["err"] for t in r["tosses"][2:] if t["launched"] and np.isfinite(t["err"])]
        med[r["world"]] = np.median(e) if e else np.nan
    S = _scaled(worlds)
    ok = np.isfinite(med)
    eff = []
    for k, q in enumerate(randomize.PARAMS):
        rho = spearmanr(S[ok, k], med[ok]).statistic
        lo, hi = ok & (S[:, k] < 1 / 3), ok & (S[:, k] > 2 / 3)
        eff.append(dict(name=q.name, rho=float(rho), low=float(np.nanmean(med[lo])), high=float(np.nanmean(med[hi]))))
    eff.sort(key=lambda e: -abs(e["rho"]))
    allerr = [t["err"] for r in rows for t in r["tosses"][2:] if t["launched"] and np.isfinite(t["err"])]
    launched = sum(t["launched"] for r in rows for t in r["tosses"])
    caught = sum(t["caught"] for r in rows for t in r["tosses"])
    return dict(world_median=med.tolist(), p10=float(np.nanpercentile(med, 10)), p50=float(np.nanpercentile(med, 50)),
                p90=float(np.nanpercentile(med, 90)), throw_median=float(np.median(allerr)),
                throw_p90=float(np.percentile(allerr, 90)), launched=launched, caught=caught, effects=eff)


def report(out):
    unit = {q.name: q for q in randomize.PARAMS}
    if "catch" in out:
        c = out["catch"]["summary"]
        print(f"Catch, 6-7 m shallow throws, firmware adult: {len(c['rate'])} worlds x 36 throws: overall "
              f"{100 * c['caught']:.1f}%; per world median {100 * c['p50']:.0f}%, 10-90% {100 * c['p10']:.0f}-"
              f"{100 * c['p90']:.0f}%, worst {100 * c['min']:.0f}%; misses not in place {c['reach']}, bounced out {c['bounce']}")
        if "nominal" in out:
            print(f"   (nominal guesses, same throw seeds: {100 * out['nominal']['caught']:.1f}%)")
        print("   parameter            range             logit/range (+-se)  Spearman  bottom third -> top third")
        for e in c["effects"]:
            q = unit[e["name"]]
            print(f"   {e['name']:18s} {q.lo:8.4g}-{q.hi:<8.4g} {e['coef']:+6.2f} (+-{e['se']:.2f})      {e['rho']:+.2f}     "
                  f"{100 * e['low']:4.0f}% -> {100 * e['high']:4.0f}%")
    if "returns" in out:
        rr = out["returns"]["summary"]
        print(f"Return throws at {RET_DIST} m, {len(rr['world_median'])} worlds x {RET_TOSSES}: miss per throw median "
              f"{100 * rr['throw_median']:.0f} cm (90% {100 * rr['throw_p90']:.0f}); world medians 10-50-90% "
              f"{100 * rr['p10']:.0f} / {100 * rr['p50']:.0f} / {100 * rr['p90']:.0f} cm; caught {rr['caught']}, launched "
              f"{rr['launched']}")
        for e in rr["effects"][:8]:
            print(f"   {e['name']:18s} Spearman {e['rho']:+.2f}: bottom third {100 * e['low']:.0f} cm -> top third "
                  f"{100 * e['high']:.0f} cm")


def main(parts, n_catch=192, n_ret=96):
    path = RESULTS / "randomized.json"
    out = json.loads(path.read_text()) if path.exists() else {}
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        if "catch" in parts:
            worlds = randomize.draw_worlds(n_catch, seed=0)
            cells = [(i, w, dd, a, 1000 * i + 10 * DISTANCES.index(dd) + APEXES.index(a))
                     for i, w in enumerate(worlds) for dd in DISTANCES for a in APEXES]
            nom = [(0, None, dd, a, 1000 * i + 10 * DISTANCES.index(dd) + APEXES.index(a))
                   for i in range(8) for dd in DISTANCES for a in APEXES]
            rows = list(pool.map(catch_cell, cells, chunksize=1))
            nrows = list(pool.map(catch_cell, nom, chunksize=1))
            out["catch"] = dict(worlds=worlds, rows=rows, summary=analyze_catch(worlds, rows))
            out["nominal"] = dict(rows=nrows, caught=float(np.mean([t["caught"] for r in nrows for t in r["tosses"]])))
        if "returns" in parts:
            worlds = randomize.draw_worlds(n_ret, seed=1)
            rows = list(pool.map(return_cell, [(i, w, 500 + i) for i, w in enumerate(worlds)], chunksize=1))
            out["returns"] = dict(worlds=worlds, rows=rows, summary=analyze_returns(worlds, rows))
    save("randomized", out)
    report(out)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] in ("catch", "returns"):
        n = int(a[1]) if len(a) > 1 else None
        main([a[0]], **({"n_catch": n} if a[0] == "catch" and n else {"n_ret": n} if n else {}))
    elif a and a[0] == "report":
        report(json.loads((RESULTS / "randomized.json").read_text()))
    elif a and a[0] == "reanalyze":             # recompute the summaries from the saved rows
        out = json.loads((RESULTS / "randomized.json").read_text())
        for part, fn in (("catch", analyze_catch), ("returns", analyze_returns)):
            if part in out:
                out[part]["summary"] = fn(out[part]["worlds"], out[part]["rows"])
        save("randomized", out)
        report(out)
    else:
        main(["catch", "returns"])
