"""calib_report [dir]: every calibration item's last run, whether it passed its limits, and how far it moved since the
run before (design.md 10). Default dir: ~/.omni/calibration."""

from __future__ import annotations

import sys
import time

from omni_calib.results import Store


def report(root: str) -> str:
    st = Store(root)
    lines = []
    for f in sorted(st.root.glob("*.json")):
        r = st.latest(f.stem, passed_only=False)
        if r is None:
            continue
        when = time.strftime("%Y-%m-%d %H:%M", time.localtime(r["time"]))
        m = ", ".join(f"{k} {v:.4g}" for k, v in r["metrics"].items() if isinstance(v, (int, float)))
        lines.append(f"{f.stem:24s} {when}  {'PASS' if r['passed'] else 'FAIL'}  {m}"
                     + ("" if r["passed"] else f"  ({'; '.join(r['failures'])})"))
    return "\n".join(lines) or f"no calibration results in {st.root}"


def main():
    print(report(sys.argv[1] if len(sys.argv) > 1 else "~/.omni/calibration"))


if __name__ == "__main__":
    main()
