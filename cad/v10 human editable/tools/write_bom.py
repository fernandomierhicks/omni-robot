"""Parts and print quantities come from the exact native assembly manifest."""
from pathlib import Path
from collections import Counter
import json
ROOT=Path(__file__).resolve().parents[1]

if __name__=='__main__':
    rows=json.loads((ROOT/'reference/assembly_manifest.json').read_text())['parts'];counts=Counter(r['part'] for r in rows)
    lines=['# Wheel-pod parts list','', 'One pod with the free-turning printed motor, bench frame and telescoping shock mockups. Dimensions and fits are nominal.','',
           '| Part | Quantity | Material / source |','| --- | ---: | --- |']
    for name,count in sorted(counts.items()):
        note=('Stock fastener, magenta in CAD' if name.startswith(('bolt_','nut_','washer_')) else
              'Owned steel bearing' if name.startswith('bearing_') else '7 mm steel shaft, 98 mm long; Ø3.4 cross hole' if name=='pivot_shaft' else
              'Medium TPU' if name=='roller_tread' else 'PLA' if name.startswith(('wheel_','roller_')) else
              'Printed PETG; 6061 bar also fits' if name=='fork_bar' else 'PLA / PETG, printed')
        lines.append(f'| [{name}](parts/V10_{name}.SLDPRT) | {count} | {note} |')
    lines += ['', 'The printed dummy uses M2.5 × 10 at both motor interfaces. A real AK40 uses the shorter vendor-interface screws (×6 output, ×8 rear). The wheel hub joints use M3 × 18. No heat-set inserts. See [DUMMY_MOTOR.md](DUMMY_MOTOR.md) for the motor assembly order.',
              '', 'The shock mockups slide freely and have no spring or damping. Their assembly mates limit eye-to-eye length to 40–50 mm. Fit real shocks for sprung testing.']
    (ROOT/'BOM.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
