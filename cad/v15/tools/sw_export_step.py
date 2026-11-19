"""Export SolidWorks parts to STEP for the build123d pipeline (Nov 18: the models in cad/v15/electronics).

    C:/Users/ferna/cadenv/Scripts/python.exe -B sw_export_step.py [src_folder] [out_folder]

Defaults: ../electronics/*.SLDPRT -> ../reference/electronics/<name>.step (lower-case, spaces as _). Skips a STEP newer than its part.
Opens each part silently, saves it as STEP AP214 (the extension decides), closes it; release() restores the settings connect() changed.
"""
import sys
import time
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))
import sw_native as S                                                  # noqa: E402

ROOT = S.ROOT


def out_name(p):
    return p.stem.lower().replace(' ', '_').replace('.', '_') + '.step'


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'electronics'
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / 'reference' / 'electronics'
    dst.mkdir(parents=True, exist_ok=True)
    sw = S.connect()
    try:
        for p in sorted(src.glob('*.SLDPRT')):
            out = dst / out_name(p)
            if out.exists() and out.stat().st_mtime > p.stat().st_mtime:
                print(f'  up to date: {out.name}')
                continue
            r = sw.OpenDoc6(str(p), 1, 1, '', 0, 0)                   # 1 = part, 1 = silent; typed wrapper: (doc, errors, warnings)
            d, err = (r[0], r[1]) if isinstance(r, tuple) else (r, None)
            if d is None:
                print(f'  FAILED to open {p.name} (error {err})')
                continue
            m = S.wrap(d, S.sld.IModelDoc2)
            ok = e = None
            for _ in range(6):                                          # Dropbox may hold the previous file
                ok, e, w = m.Extension.SaveAs3(str(out), 0, 1, None, None, 0, 0)
                if ok and not e:
                    break
                time.sleep(3)
            sw.CloseDoc(m.GetTitle())
            print(f'  {p.name} -> {out.name}: {"ok" if ok and not e else f"FAILED {e}"}')
    finally:
        S.release(sw)


if __name__ == '__main__':
    main()
