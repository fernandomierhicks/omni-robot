"""Pod strength check, step 1 (Nov 12 2026, Fernando: "a preliminary FEA or force analysis on the wheel pod + suspension,
I have a feeling those PLA parts will break. Nothing too fancy").

    python -B pod_fe_prep.py           (omni-robot-sim interpreter: build123d)

Writes validation/pod_fe/: the three printed load paths of one pod as STEP in the pod frame at the nominal pose (axle at
the origin, x along the axle toward the motor, y toward the pivot, z up), and patches.json, which says where fe_pod.py
puts the loads and the supports:

  fork.step   pod_fork_outer + pod_fork_inner fused (the tie bolts and the tenon taken as a bonded joint)
              loads: the left 6804 on the axle stub (STUB_D, x -36.5..-29.5); the motor on its pocket floor and wall
              supports: the two 627 seats, radial only (they turn), the outer one also axially; the four plunger domes
              on the pads as 6.1 N/mm springs (dims.SPRINGS, VERIFY)
  frame.step  pod_frame_inner: foot fixed on the plate; loads: the Ø7 dowel's bore (the inner 627 sits 10 mm outboard of
              it, so the bore takes a moment too) and the four spring seats under the tower
  keel.step   pod_keel_outer: foot fixed; load: the outer pivot's M3 in its Ø2.8 pilot (the 627 sits 10.7 mm outboard)

Nothing here changes the pod's geometry.
"""
import json
import math
import sys
from pathlib import Path

import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403

globals().update({k: v for m in (P, L) for k, v in vars(m).items() if k.isupper() and k not in ('VAL', 'OUT')})
VAL = W.ROOT / 'validation' / 'pod_fe'

# the AK40-10's output bearing along the axle (the hub rides on it and on the left 6804): near its output face at
# MOTOR_X[0] = 4.5; taken 4.5 mm in (VERIFY on the motor: it sets how a wheel load splits between the stub and the motor)
X_MOTOR_BRG = MOTOR_X[0] + 4.5
X_STUB_BRG = (X_PLATE_FLOOR + X_PLATE_FLOOR + 7.0) / 2      # the 6804's middle on the stub (x -36.5..-29.5)


def patches():
    xb0 = L.OA_X[1] - BRG_RECESS                              # outer 627: x xb0 - 7 .. xb0
    xi = L.INNER_W_X[1] - BRG_RECESS                          # inner 627: x xi - 7 .. xi
    pads = [dict(x=xs, y=y, z=PAD_Z, r=3.0) for xs in L.SPRING_X for y in L.Y_PAIR]
    seats = [dict(x=xs, y=y, z=P.Z_CEIL, r_out=(SPRING_HOLE_D + 0.6) / 2, r_in=GUIDE_R) for xs in L.SPRING_X for y in L.Y_PAIR]
    return dict(
        frame_note='pod frame: axle at the origin, x along the axle toward the motor, y toward the pivot, z up; mm',
        pivot=[PIVOT_Y, PIVOT_Z],
        x_stub_brg=X_STUB_BRG, x_motor_brg=X_MOTOR_BRG, wheel_r=R_WHEEL, d_row=D_ROW,
        fork=dict(
            stub=dict(kind='cyl', axis=[0.0, 0.0], r=STUB_D / 2, x=[X_PLATE_FLOOR, X_PLATE_FLOOR + 7.0]),
            motor_floor=dict(kind='xplane', x=L.INNER_ARM_X[0], axis=[0.0, 0.0], r_max=MOTOR_R + MOTOR_POCKET_CLR),
            motor_wall=dict(kind='cyl', axis=[0.0, 0.0], r=MOTOR_R + MOTOR_POCKET_CLR, x=[L.MOTOR_PLATE_X[0], L.INNER_ARM_X[0]]),
            seat_outer=dict(kind='cyl', axis=[PIVOT_Y, PIVOT_Z], r=B627_OD / 2, x=[xb0 - B627_W, xb0]),
            seat_inner=dict(kind='cyl', axis=[PIVOT_Y, PIVOT_Z], r=B627_OD / 2, x=[xi - B627_W, xi]),
            pads=pads, spring_rate=SPRINGS[SPRING_DESIGN][0],
            x_seat_outer=xb0 - B627_W / 2, x_seat_inner=xi - B627_W / 2),
        frame=dict(
            foot=dict(kind='zplane', z=L.PLATE_TOP_L),
            dowel=dict(kind='cyl', axis=[PIVOT_Y, PIVOT_Z], r=DOWEL_D / 2 - 0.05, x=list(L.FI_X)),
            seats=seats, x_load=xi - B627_W / 2),
        keel=dict(
            foot=dict(kind='zplane', z=L.PLATE_TOP_L),
            pin=dict(kind='cyl', axis=[PIVOT_Y, PIVOT_Z], r=M3_PILOT / 2, x=[L.KEEL_X[0], L.KEEL_PIN_BOSS_X]),
            x_load=xb0 - B627_W / 2),
        print_dir='every part prints on an x face: layers stack along x, so sigma_xx is across the layers',
    )


def main():
    VAL.mkdir(parents=True, exist_ok=True)
    parts, info = L.build_static()
    fork = parts['fork_outer'] + parts['fork_inner']
    out = dict(fork=fork, frame=parts['frame_inner'], keel=parts['keel'])
    rep = {}
    for k, s in out.items():
        s = s.clean()
        b.export_step(s, str(VAL / f'{k}.step'))
        rep[k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_mm3=round(s.volume, 1),
                      bbox=[round(v, 2) for v in (*tuple(s.bounding_box().min), *tuple(s.bounding_box().max))])
    pt = patches()
    pt['parts'] = rep
    (VAL / 'patches.json').write_text(json.dumps(pt, indent=1))
    print(json.dumps(rep, indent=1))


if __name__ == '__main__':
    main()
