"""Chatterbox in its own venv (C:\\Users\\ferna\\.venvs\\omni-voice-cb: torch 2.6, CUDA), for voice_engines.CbWorker.

One JSON line in per line to say: {text, ref (a reference WAV: whose voice), exag, cfg, turbo, out (the WAV to
write)}; one JSON line back: {"ok": true} or {"ok": false, "error"}. Chatterbox (exaggeration: how expressive) and
Chatterbox Turbo (faster; [laugh] [sigh] [gasp] ... tags in the text) are loaded on first use. ROS-free, no omni_face
imports (its venv has none of them).
"""

import json
import os
import sys


def main():
    import torch
    import torchaudio
    models = {}
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    for line in sys.stdin:
        try:
            r = json.loads(line)
            turbo = bool(r.get("turbo"))
            if turbo not in models:
                if turbo:
                    from chatterbox.tts_turbo import ChatterboxTurboTTS as M
                else:
                    from chatterbox.tts import ChatterboxTTS as M
                models[turbo] = M.from_pretrained(device=dev)
            m = models[turbo]
            ref = r.get("ref") if r.get("ref") and os.path.exists(r["ref"]) else None
            kw = dict(audio_prompt_path=ref)
            if not turbo:
                kw.update(exaggeration=float(r.get("exag", 0.5)), cfg_weight=float(r.get("cfg", 0.5)))
            wav = m.generate(r["text"], **kw)
            torchaudio.save(r["out"], wav.cpu(), m.sr)
            out = dict(ok=True)
        except Exception as e:                      # noqa: BLE001  (one bad line must not end the worker)
            out = dict(ok=False, error=f"{type(e).__name__}: {e}")
        sys.stdout.write(json.dumps(out) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
