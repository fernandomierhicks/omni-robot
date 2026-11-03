"""HISTORICAL nose integration (body clearance only); superseded by the bar/FOV studies 52–54."""
import copy
import importlib
import importlib.util
import json
import math
import sys
from pathlib import Path
import numpy as np
import build123d as b

ROOT = Path(__file__).resolve().parents[2]
CHECK = importlib.import_module('49_check_reconciliation')
spec = importlib.util.spec_from_file_location('nose_concept', ROOT.parent / 'other/d435_nose/d435_nose.py')
NOSE = importlib.util.module_from_spec(spec)
spec.loader.exec_module(NOSE)


def main():
    NOSE.CAM = (190.0, 0.0, 111.5)
    NOSE.DECK_TOP = 73.0
    _, wedge, screws, ext = NOSE.build()
    a = math.radians(30)
    ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    # Source: X is width, Y up, Z toward lenses; front face Z=0, body centre Z=-12.5.
    R = ry @ np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.array(NOSE.CAM) - R @ [0, 0, -12.5]
    camera = CHECK.transformed(b.import_step(ROOT / 'reference/d435_source.step'), M)
    # Small casting protrusions differ from the ideal box. Relieve the support
    # against the actual solid, 0.2 mm below the camera, rather than overlap it.
    cast_clear = CHECK.transformed(camera, [[1,0,0,-.1],[0,1,0,0],[0,0,1,-.1732050807569],[0,0,0,1]])
    wedge = wedge - cast_clear
    assert len(wedge.solids()) == 1 and wedge.is_valid
    # Both underside deck screws: 12 mm length = 6 mm deck + 6 mm engagement.
    mx = (ext['rear_x'] + ext['front_x']) / 2
    rows = copy.deepcopy(CHECK.ROWS)
    original = next(r for r in rows if r['product'] in ('deck_front', 'REC_deck_front_D435'))
    deck_row = copy.deepcopy(original)
    deck_row['product'] = 'deck_front'
    deck = CHECK.item(deck_row)
    for y in (-18., 18.):
        deck -= b.Pos(mx, y, 70) * b.Cylinder(1.7, 10)
    original['product'] = 'REC_deck_front_D435'
    assert len(deck.solids()) == 1 and deck.is_valid
    shapes = dict(REC_D435_wedge=wedge, REC_deck_front_D435=deck)
    for n, s in shapes.items():
        s.label = n
        s.color = b.Color(.3, .3, .33)
        path = ROOT / 'step_parts' / (n + '.step')
        b.export_step(s, path)
        back = b.import_step(path)
        assert back.is_valid and len(back.solids()) == 1
    # Hardware is a private assembly input, not a printed-parts STEP export.
    for k, s in enumerate(screws):
        flat = b.Part(s.wrapped)
        flat.label, flat.color = f'REC_D435_screw_{k}', b.Color(1, 0, 1)
        b.export_step(flat, ROOT / 'step_parts' / f'REC_D435_screw_{k}.step')
    obstacles = []
    for r in rows:
        if r['fastener'] or r['group'].startswith('SO-101') or r['product'].startswith('label_'):
            continue
        s = deck if r is original else CHECK.item(r)
        # Broad phase: avoid loading booleans far from this small nose assembly.
        bb = s.bounding_box()
        if bb.max.X < 165 or bb.min.X > 215 or bb.min.Z > 135 or bb.max.Z < 60 or bb.min.Y > 50 or bb.max.Y < -50:
            continue
        obstacles.append((r['name'], s))
    checks = []
    for n, s in [('camera', camera), ('wedge', wedge)] + [(f'screw{k}', s) for k, s in enumerate(screws)]:
        for on, os in obstacles:
            d = s.distance_to(os)
            v = CHECK.intersection_volume(s, os) if d < 1e-5 else 0.
            checks.append(dict(part=n, obstacle=on, clearance_mm=float(d), overlap_mm3=v))
    checks.append(dict(part='camera', obstacle='wedge', clearance_mm=float(camera.distance_to(wedge)),
                       overlap_mm3=CHECK.intersection_volume(camera, wedge)))
    report = dict(camera_center_mm=NOSE.CAM, tilt_down_deg=30., camera_source_to_robot=M.tolist(),
                  camera_box=dict(min=list(camera.bounding_box().min), max=list(camera.bounding_box().max)),
                  wedge_box=dict(min=list(wedge.bounding_box().min), max=list(wedge.bounding_box().max)),
                  deck_screw_xy_mm=[[mx,-18],[mx,18]], checks=checks,
                  camera_solids=len(camera.solids()), wedge_solids=len(wedge.solids()))
    report['pass'] = all(r['overlap_mm3'] < .1 for r in checks)
    (ROOT / 'validation/d435_integration_geometry.json').write_text(json.dumps(report, indent=2))
    (ROOT / 'reference/robot_sim_instances.json').write_text(json.dumps(rows, indent=1))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
