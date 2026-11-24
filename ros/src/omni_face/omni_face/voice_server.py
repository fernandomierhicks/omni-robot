"""The robot's voice and chat service on the main PC (ROS-free; Nov 21): HTTP on port 9884.

  GET  /health          {"ok": true, "voices": [ids], "chat": model}
  GET  /voices          voice_engines.VOICES (what each is)
  POST /tts             {"text", "voice": a VOICES id, "emotion"} -> audio/wav (16-bit mono, 22050 Hz), cached
  POST /chat            {"text": what was heard, "context": the robot's facts} -> {say, emotion, intensity, remember
                        [, pending]}, from OMNI_VOICE_BRAIN: claude_code (the default: the Claude Code session kept
                        open on this PC, DeskChat), api (brain.py over the API) or chat (chat.py, talk only)
  GET  /skills          the skills the brain knows: hand-written (ros/tools/omni_mcp/skills) and invented ones
  GET  /skills/page     the same as a small page: keep, rename or delete what it invented
  POST /skills/keep     {"name"}            an invented skill marked as kept
  POST /skills/rename   {"name", "new"}
  POST /skills/delete   {"name"}            an invented skill removed (hand-written ones can't be)

Runs in the voice venv: C:\\Users\\ferna\\.venvs\\omni-voice\\Scripts\\python.exe -m omni_face.voice_server (from
ros/src/omni_face), or started by the face window itself (display.py) when it isn't running. The face (voice.py's
Server backend) asks it for every line and falls back to its own Kokoro if it doesn't answer. Lines repeat, so each
(voice, feeling, text) is made once and kept in CACHE. --bind 0.0.0.0 lets the robot computer reach it over the LAN
(its face then sets OMNI_VOICE_SERVER=http://<pc>:9884).
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
import threading
import time
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

PORT = 9884
# who answers a sentence that isn't a command: claude_code (Nov 23, Fernando: a Claude Code session kept open on this
# PC, voice_brain.cmd; through omni_mcp's voice desk), api (brain.py: Claude Haiku over the API, needs a key), chat
# (chat.py: talk only, over the API)
BRAIN = {"0": "chat", "1": "api"}.get(os.environ.get("OMNI_VOICE_BRAIN", ""),
                                     os.environ.get("OMNI_VOICE_BRAIN", "") or "claude_code")
MCP_DIR = Path(__file__).resolve().parents[3] / "tools" / "omni_mcp"
NOT_LISTENING = ("My thinking session isn't running. Start the voice brain on the PC: ros/tools/voice_brain.cmd.")


def _mcp_path():
    import sys
    if str(MCP_DIR) not in sys.path:
        sys.path.insert(0, str(MCP_DIR))


class DeskChat:
    """OMNI_VOICE_BRAIN=claude_code: the sentence goes into the voice desk (omni_mcp voice_desk.py) for the Claude Code
    session kept open on this PC (voice_brain.cmd); its quick answer within voice_desk.REPLY_S is the reply, else "One
    moment." and the session says its answer through the robot when it has one. "yes" / "no" while its plan waits
    answer the plan."""

    def reply(self, text: str, ctx: dict | None = None, now=None) -> dict:
        _mcp_path()
        import voice_desk as desk
        text = " ".join(str(text).split())[:400]
        if not text:
            return dict(say="", emotion="neutral", intensity=0.0, remember="")
        if not desk.alive():
            return dict(say=NOT_LISTENING, emotion="sad", intensity=0.3, remember="", error="no Claude Code session")
        e = desk.post(text, ctx or {})
        if e["kind"] == "answer":
            return dict(say="Okay, here I go." if text == "yes" else "Okay, I won't.",
                        emotion="focused" if text == "yes" else "neutral", intensity=0.4, remember="")
        d = desk.wait_reply(e["id"], e["t"] + desk.REPLY_S)
        if d is not None:
            return dict(say=d.get("say", ""), emotion=d.get("emotion", "neutral"),
                        intensity=float(d.get("intensity", 0.5)), remember="")
        return dict(say="One moment.", emotion="focused", intensity=0.3, remember="")
CACHE = Path(os.environ.get("OMNI_FACE_CACHE", Path.home() / ".cache" / "omni_face")) / "server"
VERSION = "1"                     # bump when the chains change: the cache keys change with it


def wav_bytes(a: np.ndarray, sr: int) -> bytes:
    a = np.clip(np.asarray(a, float), -1.0, 1.0)
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(sr))
        w.writeframes((a * 32767).astype(np.int16).tobytes())
    return buf.getvalue()


class Service:
    def __init__(self):
        from omni_face.voice_engines import VOICES, Engines
        self.voices = VOICES
        self.engines = Engines()
        self.chat = None
        self.chat_lock = threading.Lock()

    def tts(self, text: str, voice: str, emotion: str = "") -> bytes:
        if voice not in self.voices:
            raise KeyError(f"voice: one of {', '.join(self.voices)}")
        key = hashlib.sha1(f"{VERSION}|{voice}|{emotion}|{text}".encode()).hexdigest()[:20]
        p = CACHE / voice / f"{key}.wav"
        if p.exists() and p.stat().st_size > 1000:
            return p.read_bytes()
        a, sr = self.engines.render(voice, text, emotion)
        data = wav_bytes(a, sr)
        p.parent.mkdir(parents=True, exist_ok=True)
        tmp = p.with_suffix(".tmp")
        tmp.write_bytes(data)
        os.replace(tmp, p)
        return data

    def talk(self, text: str, ctx: dict) -> dict:
        with self.chat_lock:
            if self.chat is None:
                if BRAIN == "claude_code":
                    self.chat = DeskChat()
                elif BRAIN == "api":
                    from omni_face.brain import Brain
                    self.chat = Brain()
                else:
                    from omni_face.chat import Chat
                    self.chat = Chat()
            chat = self.chat
        return chat.reply(text, ctx)      # the brain answers "go" while its own request still runs: no lock here

    def library(self):
        _mcp_path()
        from skillbox import Library
        if self.chat is not None and getattr(self.chat, "library", None) is not None:
            return self.chat.library
        return Library()

    def skills(self, path: str, req: dict | None = None):
        lib = self.library()
        if path == "/skills":
            return [s.public() for s in sorted(lib.all().values(), key=lambda s: (s.builtin, s.name))]
        name = str((req or {}).get("name", ""))
        if path == "/skills/keep":
            lib.keep(name)
        elif path == "/skills/delete":
            lib.delete(name)
        elif path == "/skills/rename":
            lib.rename(name, str((req or {}).get("new", "")))
        else:
            raise KeyError("not found")
        return dict(ok=True)


SKILLS_PAGE = """<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Robot skills</title>
<style>body{font:15px system-ui;margin:16px;max-width:900px;background:#fff;color:#222}
td,th{padding:6px 8px;border-bottom:1px solid #ddd;text-align:left;vertical-align:top}
button{margin-right:4px}.b{color:#777}@media(prefers-color-scheme:dark){body{background:#151515;color:#ddd}}</style>
<h2>Robot skills</h2><p class=b>Hand-written skills are part of the robot's code; invented ones were written when
someone asked for something new. Keep, rename or delete those.</p><table id=t></table><script>
const e=x=>String(x??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function load(){const s=(await (await fetch('/skills')).json()).map(k=>Object.fromEntries(Object.entries(k).map(([a,b])=>[a,typeof b==='string'?e(b):b])));const t=document.getElementById('t');
t.innerHTML='<tr><th>Skill</th><th>What it does</th><th></th></tr>'+s.map(k=>`<tr><td><b>${k.name}</b><br>
<span class=b>${k.builtin?'hand-written':(k.kept?'kept':k.tried?'tried':'new')}</span></td><td>${k.description}
${k.request?`<br><span class=b>asked: “${k.request}” (${k.created})</span>`:''}</td><td>${k.builtin?'':
`<button onclick="act('keep','${k.name}')">Keep</button><button onclick="ren('${k.name}')">Rename</button>
<button onclick="act('delete','${k.name}')">Delete</button>`}</td></tr>`).join('')}
async function act(a,n,x){const r=await fetch('/skills/'+a,{method:'POST',body:JSON.stringify({name:n,new:x})});
if(!r.ok)alert((await r.json()).error);load()}
function ren(n){const x=prompt('New name (lower case, digits, _)',n);if(x&&x!==n)act('rename',n,x)}
load()</script>"""


def handler(svc: Service):
    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code: int, body: bytes, ctype: str = "application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj).encode())

        def do_GET(self):
            if self.path == "/health":
                from omni_face.chat import MODEL
                self._json(200, dict(ok=True, voices=list(svc.voices), chat=MODEL, brain=BRAIN))
            elif self.path == "/voices":
                self._json(200, svc.voices)
            elif self.path == "/skills":
                self._json(200, svc.skills("/skills"))
            elif self.path == "/skills/page":
                self._send(200, SKILLS_PAGE.encode(), "text/html; charset=utf-8")
            else:
                self._json(404, dict(error="not found"))

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length") or 0)
                req = json.loads(self.rfile.read(min(n, 100_000)) or b"{}")
            except (ValueError, TypeError):
                return self._json(400, dict(error="bad JSON"))
            try:
                if self.path == "/tts":
                    t0 = time.time()
                    data = svc.tts(str(req.get("text", ""))[:600], str(req.get("voice", "")),
                                   str(req.get("emotion") or ""))
                    print(f"tts {req.get('voice')} [{req.get('emotion') or '-'}] {time.time() - t0:.2f}s "
                          f"{str(req.get('text'))[:60]!r}", flush=True)
                    return self._send(200, data, "audio/wav")
                if self.path == "/chat":
                    t0 = time.time()
                    r = svc.talk(str(req.get("text", "")), dict(req.get("context") or {}))
                    print(f"chat {time.time() - t0:.1f}s {str(req.get('text'))[:60]!r} -> {r.get('say', '')[:80]!r}"
                          + (" [pending]" if r.get("pending") else "") + (f" [{r['error']}]" if r.get("error") else ""),
                          flush=True)
                    return self._json(200, r)
                if self.path.startswith("/skills/"):
                    try:
                        return self._json(200, svc.skills(self.path, req))
                    except ValueError as e:
                        return self._json(400, dict(error=str(e)))
            except KeyError as e:
                return self._json(400, dict(error=str(e)))
            except Exception as e:                       # noqa: BLE001  (a failed line must not end the service)
                print(f"error: {type(e).__name__}: {e}", flush=True)
                return self._json(500, dict(error=f"{type(e).__name__}: {e}"))
            self._json(404, dict(error="not found"))
    return H


def main():
    ap = argparse.ArgumentParser(description="the robot's voice and chat service (the main PC)")
    ap.add_argument("--bind", default=os.environ.get("OMNI_VOICE_BIND", "127.0.0.1"))
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--warm", default=os.environ.get("OMNI_VOICE_WARM", ""),
                    help="a voice to load at start (its first line is then quick)")
    a = ap.parse_args()
    svc = Service()
    srv = ThreadingHTTPServer((a.bind, a.port), handler(svc))
    print(f"voice service on http://{a.bind}:{a.port} ({len(svc.voices)} voices)", flush=True)
    if a.warm in svc.voices:
        threading.Thread(target=lambda: svc.tts("Online.", a.warm, "neutral"), daemon=True).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
