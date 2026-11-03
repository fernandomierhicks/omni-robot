"""Reconcile the active CAD with robot_export.ARM without changing frozen v9.

Run with the omni-robot-sim interpreter and -B. All outputs stay inside cad/v10.
v9 sources and the extracted v9 placement table remain reproduction inputs.
"""
import copy
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import build123d as b

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT.parent / 'v9'))
import dims as D
import arm_kit as K
import head as H
import mast as MS


def box_info(s):
    bb = s.bounding_box()
    return dict(solids=len(s.solids()), valid=s.is_valid,
                volume_mm3=float(s.volume), min=list(bb.min), max=list(bb.max))


def main():
    out = ROOT / 'step_parts'
    # Keep the plate/deck geometry. The adapter/arm drop 13 mm; the shorter
    # riser starts on the SAME plate. Buck bosses move with the adapter.
    K.ARM_ORIGIN = (140.0, 0.0, 224.5)
    K.RISER = dict(K.RISER, top=212.0)
    K.ADAPTER = dict(K.ADAPTER, top=222.1)
    K.BUCK = dict(K.BUCK, z=195.0)

    # Head features rise 5 mm over v7, but its plate clamp stays at z181.5.
    # A separate dims view avoids changing the plate, the mast or frozen v9.
    H.D = SimpleNamespace(**{k: v for k, v in vars(D).items() if not k.startswith('__')})
    H.D.UPPER_LIFT = 5.0
    H.AXIS = (155.0, 200.0)
    H.WEB_BOT -= 8.0
    H.JAW_BOT -= 8.0
    H.THICK = [(x, z - 8.0) for x, z in H.THICK]

    # Rear mast retained by Fernando. Its upper end follows the deck's +6.5
    # mm over v7, rather than the payload plate's +13 mm. Feet remain seated.
    MS.POST_TOP -= 6.5
    MS.POST_L -= 6.5
    MS.CROWN_BOLT_Z -= 6.5
    adapter = K.adapter()
    # The detailed 10.1 mm adapter is thicker than the sim's 6 mm plate.
    # Clear the bar's entire 20 mm square rotational envelope by 3 mm;
    # leave full thickness at x128/x192 base screws and x70/x122 riser screws.
    relief = b.Box(35.0, 114.0, 5.2, align=(b.Align.MIN, b.Align.CENTER, b.Align.MIN)).moved(b.Pos(137.5, 0, 212.0))
    adapter = adapter - relief
    shapes = {'REC_arm_riser': K.riser(), 'REC_arm_adapter': adapter, 'REC_cheek_L': H.cheek(1),
              'REC_cheek_R': H.cheek(-1), 'REC_mast_post': MS.post()}
    reports = {}
    for name, shape in shapes.items():
        info = box_info(shape)
        assert info['valid'] and info['solids'] == 1, (name, info)
        shape.label = name
        shape.color = b.Color(*((.78, .80, .83) if 'mast_post' in name else (.98, .45, .08)))
        path = out / (name + '.step')
        b.export_step(shape, path)
        back = b.import_step(path)
        info['reimport'] = box_info(back)
        assert info['reimport']['valid'] and info['reimport']['solids'] == 1
        assert abs(info['volume_mm3'] - info['reimport']['volume_mm3']) < .01
        reports[name] = info
        print(name, info, flush=True)

    rows = json.loads((ROOT / 'reference/robot_v9_instances.json').read_text())
    rows = copy.deepcopy(rows)
    for r in rows:
        p, g = r['product'], r['group']
        z, x = r['matrix'][2][3], r['matrix'][0][3]
        dz = 0.0
        if g == 'head (camera bar, indexed)':
            dz = -8.0
        elif g == 'mounts (yokes or arm kit, cheeks)':
            if p == 'arm_riser':
                r['product'] = 'REC_arm_riser'
            elif p in ('cheek_L', 'cheek_R'):
                r['product'] = 'REC_' + p
            elif p == 'arm_adapter':
                r['product'] = 'REC_arm_adapter'
            elif p in ('12_V_buck_arm', 'label_12V_BUCK_ARM'):
                dz = -13.0
            elif r['fastener']:
                if z > 230 or (abs(z - 208) < .01 and x in (95, 125)):
                    dz = -13.0
                elif abs(x - 141.88713276239) < .01:
                    dz = -8.0       # camera-bar index screw
        elif g == 'face mast (face, speaker, mics)':
            if p == 'mast_post_20_x_20_tube':
                r['product'] = 'REC_mast_post'   # shorter tube, same foot datum
            elif p not in ('mast_foot_L', 'mast_foot_R', 'ELRS_receiver', 'label_ELRS_RX'):
                # Crown, display, audio and upper fasteners all move together.
                # Shapes in robot coordinates have zero component translation.
                if not r['fastener'] or z > 200:
                    dz = -6.5
        elif g.startswith('SO-101'):
            dz = -13.0
        r['matrix'][2][3] += dz
    (ROOT / 'reference/robot_sim_instances.json').write_text(json.dumps(rows, indent=1))
    report = dict(arm_origin_mm=list(K.ARM_ORIGIN), plate_top_mm=D.PLATE_TOP,
                  riser_top_mm=212.0, adapter_top_mm=222.1, arm_assembly_shift_mm=-13.0,
                  camera_axis_mm=list(H.AXIS), head_lift_over_v7_mm=5.0,
                  mast_lift_over_v7_mm=6.5, mast_post_length_mm=MS.POST_L,
                  adapter_bar_relief_mm=dict(x=[137.5,172.5], underside_z=217.2, remaining_web=4.9),
                  face_seat_mm=[-187.0, 0.0, 436.5], parts=reports,
                  note='Rear mast retained; simulation still uses the older face over bin. Frozen v9 not modified.')
    (ROOT / 'validation/sim_reconciliation_geometry.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
