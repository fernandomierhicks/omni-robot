"""v15 world: the whole robot as solids, built in one place for every script that checks against "everything" (arm_sweep, d435_riser, the
harness, service, upper's check, robot_step, sw_robot_parts). Library only. Each piece is built once per process.

    base()          (parts, hw, g) of the base: chassis_mid.py (deck, bumper, fins, pedestal, battery clips; g['bought']: the battery, straps)
    pods(a)         {name: solid} of the four pods at fork angle a (deg), robot frame, keys 'FL/<part>'
    upper()         (printed, bought, vendor, hw, info) of the second floor and everything on it: upper.py
    static_robot()  {name: solid}: base printed + bought, the lidar, the pods at a pose, the upper robot (+ vendor models), optionally the
                    fasteners
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chassis_mid as CM                                               # noqa: E402
import fins as F                                                       # noqa: E402
import hardware as H                                                   # noqa: E402

ROLLERS = ('roller_core', 'roller_sleeve', 'bearing_687')             # inside the treads: left out of the world unless asked
_UPPER = None


def base():
    return CM.built()


def base_items(bought=True):
    """the base's parts (+ the battery and its straps) and the lidar's envelope"""
    parts, _, g = base()
    out = dict(parts, **(g['bought'] if bought else {}))
    out['env_lidar'] = F.lidar_env()
    return out


def pods(a=0.0, rollers=False, split=False):
    """the four pods at fork angle a; split=True: (static, moving) dicts"""
    st_all, mv_all = {}, {}
    for n, (st, mv) in CM.pods_placed(a).items():
        st_all.update({f'{n}/{k}': v for k, v in st.items()})
        mv_all.update({f'{n}/{k}': v for k, v in mv.items() if rollers or not k.startswith(ROLLERS)})
    return (st_all, mv_all) if split else dict(st_all, **mv_all)


def upper():
    global _UPPER
    if _UPPER is None:
        import upper as U
        _UPPER = U.build()
    return _UPPER


def static_robot(a=0.0, vendor=True, hardware=False, arm=False):
    """everything at pod angle a: base, pods, upper robot (+ its vendor models; the SO-101 only with arm=True); hardware=True adds every
    fastener (base, pods' static ones are already in pods(), upper)"""
    import upper as U
    printed, bought, vend, uhw, _ = upper()
    out = base_items()
    out.update(pods(a))
    out.update(printed)
    out.update(bought)
    if vendor:
        out.update(U.load_vendor({k: v for k, v in vend.items() if arm or not k.startswith('arm_')}))
    if hardware:
        _, hw, _ = base()
        out.update(hw)
        for n, r in enumerate(uhw):
            out[f'{r["name"]}#{n}'] = H.solid(r)
    return out
