"""Review recorded RealSense throws by eye: the colour video frame by frame with the vision drawn on, and your call (in / rim / out) saved.

A local page (standard library server, plain JS). For each throw: every ball-coloured blob (grey; the scene's static ones dotted), the
tracked ball (orange), the depth and IR detections projected into the colour image (cyan / white), each source's fitted arc (solid where
it was seen, dashed out to its predicted rim crossing; an arrow at the edge when the crossing is out of view), the causal verdict at the
current frame and the time to the crossing, and a timeline of detections and verdicts. You add: the outcome you saw, notes, shift-click
to place the ball where the detector missed it, X to mark a frame's detection wrong. Saved in the throw's meta.json ("review").

  python rs_review.py ~/throw_data/<session> [--bucket bucket.json] [--port 8090]      (opens the browser)
  bash characterization/throw_track/tt.sh rsreview [SESSION DIR]
"""
from __future__ import annotations

import argparse
import io
import json
import math
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import arc2d                    # noqa: E402
import rs_analyze as A          # noqa: E402
import rs_detect as D           # noqa: E402
import rs_predict as P          # noqa: E402
import session as S             # noqa: E402

SOURCES = {"colour": "#ff8c1a", "depth": "#22d3ee", "ir_stereo": "#f5f5f5"}


class Review:
    """One session: lazily analysed throws, their JPEG frames, and the labels."""

    def __init__(self, d: Path, bucket: Path | None, cfg: D.DetectCfg):
        self.d, self.sess = Path(d), S.load_session(d)
        self.cfg = A.with_session_ball(cfg, self.sess)
        self.bucket = P.Bucket.load(bucket) if bucket else None
        if self.bucket:
            self.bucket.ball_radius = self.cfg.ball_radius
        self.dirs = {int(p.name[1:]): p for p in S.throw_dirs(self.d)}
        self.cache: dict = {}
        self.jpegs: dict = {}
        self.lock = threading.Lock()

    def throw(self, n: int) -> S.Throw:
        return S.load_throw(self.dirs[n], self.sess)

    def data(self, n: int) -> dict:
        with self.lock:
            if n in self.cache:
                d = self.cache[n]
                d["review"] = json.loads((self.dirs[n] / "meta.json").read_text()).get("review")
                return d
        th = self.throw(n)
        t0 = time.time()
        res = A.analyze_throw(th, self.sess, self.cfg, colour=True, session_dir=self.d, session_ball=False,
                              up=self.bucket.up if self.bucket else None)
        if self.bucket:
            A.add_predictions(res, self.bucket, self.cfg)
        out = self._overlay(th, res)
        out["analysis_s"] = round(time.time() - t0, 1)
        with self.lock:
            self.cache[n] = out
        return out

    def _overlay(self, th: S.Throw, res: dict) -> dict:
        cal = A.calibration(self.sess)
        ic = cal["intr"]["color"]
        R, tr = cal["ext"].get("depth_to_color", (np.eye(3), np.zeros(3)))
        tc = th.t_of("color")

        def to_px(P3):
            P3 = np.atleast_2d(np.asarray(P3, float))
            pc = P3 @ R.T + tr
            uv = ic.project(pc)
            return [[round(float(u), 1), round(float(v), 1)] if z > 0.1 else None for (u, v), z in zip(uv, pc[:, 2])]

        cands = res.get("_cands") or [[] for _ in tc]
        still = arc2d.static_mask(cands)
        blobs = [[[round(c[0], 1), round(c[1], 1), round(c[2], 1), int(s)] for c, s in zip(cs, ss)] for cs, ss in zip(cands, still)]
        sources = {}
        for name, (ts, picks, fit) in res["_keep"].items():
            per = [None] * len(tc)
            got = [(ts[i], p) for i, p in enumerate(picks) if p is not None]
            if got:
                tg = np.array([g[0] for g in got])
                j = A.nearest(tc, tg)
                for (t_, p), k in zip(got, j):
                    if abs(tc[k] - t_) < 0.012:
                        uv = to_px(p[2])[0]
                        if uv:
                            r = p[1].r if name == "colour" else ic.fx * self.cfg.ball_radius / max(float(p[2][2]), 0.2)
                            per[int(k)] = [uv[0], uv[1], round(float(r), 1)]
            entry = dict(color=SOURCES.get(name, "#fff"), picks=per)
            if fit is not None and got:
                t_first, t_last = float(got[0][0]), float(got[-1][0])
                pr = (res.get("prediction") or {}).get(name) or {}
                t_end = pr.get("reference", {}).get("t_cross", t_last + 0.3) if pr.get("ok") else t_last + 0.3
                tq = np.linspace(t_first, max(t_end, t_last), 80)
                entry["arc"] = dict(t=[round(float(x), 4) for x in tq], uv=to_px(fit.at(tq)), seen_until=t_last, first=t_first)
                if pr.get("ok"):
                    ref = pr["reference"]
                    entry["reference"] = dict(verdict=ref["verdict"], margin_cm=round(100 * ref["margin_m"], 1), radial_cm=round(100 * ref["radial_m"], 1),
                                              t_cross=ref["t_cross"], extrapolated_ms=round(1000 * ref["extrapolated_s"]), uv_cross=to_px(ref["position_cam"])[0],
                                              time_to_correct_ms=None if pr.get("time_to_correct_ttg") is None else round(1000 * pr["time_to_correct_ttg"]),
                                              ambiguous=pr.get("ambiguous"))
                    entry["series"] = [dict(t=s["t"], ttg=round(s["ttg"], 3), verdict=s["verdict"], radial_cm=round(100 * s["radial_m"], 1)) for s in pr["series"]]
            sources[name] = entry
        summ = {k: {kk: v for kk, v in s.items() if isinstance(v, (int, float, str, bool, type(None)))} for k, s in res["sources"].items()}
        meta = th.meta
        rim = None
        if self.bucket:
            ang = np.linspace(0, 2 * math.pi, 48)
            b = self.bucket
            e1 = np.cross(b.up, [1.0, 0, 0]) if abs(b.up[0]) < 0.9 else np.cross(b.up, [0, 1.0, 0])
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(b.up, e1)
            ring = b.rim_center + b.radius * (np.outer(np.cos(ang), e1) + np.outer(np.sin(ang), e2))
            rim = dict(uv=to_px(ring), centre=to_px(b.rim_center)[0])
        return dict(name=th.path.name, n=len(tc), t=[round(float(x), 4) for x in tc], width=ic.width, height=ic.height,
                    trigger_t=meta.get("t_trigger_s"), arc_trigger=meta.get("arc_trigger"), blobs=blobs, sources=sources, summary=summ,
                    rim=rim, review=meta.get("review"), describe=A.describe(res))

    def jpeg(self, n: int, i: int) -> bytes:
        key = (n, i)
        if key in self.jpegs:
            return self.jpegs[key]
        from PIL import Image
        th = self.throw(n)
        img = Image.fromarray(np.ascontiguousarray(th.frames[i]))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=82)
        self.jpegs[key] = buf.getvalue()
        return self.jpegs[key]

    def save_review(self, n: int, rv: dict):
        rv = dict(rv, at=time.strftime("%Y-%m-%dT%H:%M:%S"))
        m = self.dirs[n] / "meta.json"
        meta = json.loads(m.read_text())
        meta["review"] = rv
        m.write_text(json.dumps(meta, indent=1))

    def index(self) -> list:
        rows = []
        for n, p in sorted(self.dirs.items()):
            meta = json.loads((p / "meta.json").read_text())
            d = self.cache.get(n)
            verdicts = {k: s["reference"]["verdict"] for k, s in (d or {}).get("sources", {}).items() if s.get("reference")} if d else None
            rows.append(dict(n=n, name=p.name, saved=meta.get("saved"), call=(meta.get("review") or {}).get("call"), verdicts=verdicts,
                             trigger=meta.get("arc_trigger")))
        return rows


# ------------------------------------------------------------------------------------------------------- the page
INDEX = """<!doctype html><html><head><meta charset="utf-8"><title>Throw review</title><style>
body{font:14px system-ui,sans-serif;background:#111;color:#ddd;margin:16px}a{color:#7cc4ff}td,th{padding:4px 10px;text-align:left}
tr:nth-child(even){background:#1a1a1a}.in{color:#4ade80}.out{color:#f87171}.rim{color:#fbbf24}</style></head><body>
<h2>Throw review: %SESSION%</h2><p>Click a throw. The first open of each throw runs the analysis (a few seconds).</p>
<table id="t"><tr><th>throw</th><th>saved</th><th>trigger</th><th>verdicts (by source)</th><th>your call</th></tr></table>
<script>
fetch('index.json').then(r=>r.json()).then(rows=>{const t=document.getElementById('t');for(const r of rows){const tr=document.createElement('tr');
const v=r.verdicts?Object.entries(r.verdicts).map(([k,x])=>`${k}: <span class="${x}">${x}</span>`).join(', '):'(not analysed yet)';
const trig=r.trigger?`${r.trigger.speed_mps.toFixed(1)} m/s, ${r.trigger.acc_g.toFixed(1)} g`:'';
tr.innerHTML=`<td><a href="t/${r.n}">${r.name}</a></td><td>${r.saved||''}</td><td>${trig}</td><td>${v}</td><td class="${r.call||''}">${r.call||'-'}</td>`;t.appendChild(tr)}})
</script></body></html>"""

VIEW = r"""<!doctype html><html><head><meta charset="utf-8"><title>Throw %N%</title><style>
body{font:14px system-ui,sans-serif;background:#111;color:#ddd;margin:12px}
#wrap{max-width:1100px}canvas{display:block;width:100%;background:#000}#tl{height:90px;margin-top:4px;cursor:pointer}
.bar{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:6px 0}button{background:#2a2a2a;color:#eee;border:1px solid #444;padding:5px 10px;border-radius:4px;cursor:pointer}
button.on{background:#2563eb}button.call.sel{outline:2px solid #fff}#info{font:13px ui-monospace,monospace;white-space:pre;background:#1a1a1a;padding:8px;margin-top:6px}
.in{color:#4ade80}.out{color:#f87171}.rim{color:#fbbf24}label{user-select:none}textarea{width:100%;background:#1a1a1a;color:#ddd;border:1px solid #444}
#hud{font:13px ui-monospace,monospace}a{color:#7cc4ff}</style></head><body><div id="wrap">
<div class="bar"><a href="../">&larr; all throws</a> <b id="title"></b> <span id="hud"></span></div>
<canvas id="cv"></canvas><canvas id="tl"></canvas>
<div class="bar"><button id="prev">&#9664; frame</button><button id="play">play</button><button id="next">frame &#9654;</button>
speed <select id="speed"><option>1</option><option selected>0.25</option><option>0.1</option></select>
<label><input type="checkbox" id="o_blobs" checked> all blobs</label><label><input type="checkbox" id="o_arc" checked> arcs</label>
<label><input type="checkbox" id="o_rim" checked> rim</label><label><input type="checkbox" id="o_trail" checked> trail</label>
<span style="opacity:.7">keys: &larr; &rarr; step, space play, X wrong detection, shift-click = the ball is here</span></div>
<div class="bar"><b>What did the ball do?</b> <button class="call" data-c="in">In</button><button class="call" data-c="rim">Rim (touched)</button>
<button class="call" data-c="out">Out</button><button class="call" data-c="unsure">Unsure</button><button class="call" data-c="no_throw">No throw</button>
<button id="save">Save</button><span id="saved"></span></div>
<textarea id="notes" rows="2" placeholder="notes (where it missed, bounced off the rim, ...)"></textarea>
<div id="info"></div></div>
<script>
const N=%N%;let D,imgs=[],k=0,playing=false,call=null,clicks=[],wrong=new Set();
const cv=document.getElementById('cv'),cx=cv.getContext('2d'),tl=document.getElementById('tl'),tx=tl.getContext('2d');
const $=id=>document.getElementById(id);
fetch('data.json').then(r=>r.json()).then(d=>{D=d;cv.width=d.width;cv.height=d.height;tl.width=1100;tl.height=90;$('title').textContent=d.name;
 for(let i=0;i<d.n;i++){const im=new Image();im.src=`f/${i}.jpg`;im.onload=()=>{if(i===k)draw()};imgs.push(im)}
 const rv=d.review||{};call=rv.call||null;clicks=rv.clicks||[];wrong=new Set(rv.wrong||[]);$('notes').value=rv.notes||'';markCall();
 $('info').textContent=d.describe+(d.arc_trigger?`\ntrigger: ${JSON.stringify(d.arc_trigger)}`:'');
 const c=d.sources.colour; if(c&&c.picks){const f=c.picks.findIndex(p=>p);if(f>=0)k=Math.max(0,f-5)} draw()});
function tNow(){return D.t[k]}
function verdictAt(s,t){if(!s.series)return null;let v=null;for(const e of s.series){if(e.t<=t+1e-6)v=e;else break}return v}
function circle(u,v,r,col,w,dash){cx.beginPath();cx.setLineDash(dash||[]);cx.strokeStyle=col;cx.lineWidth=w;cx.arc(u,v,Math.max(r,3),0,7);cx.stroke();cx.setLineDash([])}
function arrowTo(u,v,col){const W=D.width,H=D.height,cu=W/2,cv_=H/2;let dx=u-cu,dy=v-cv_;const s=Math.min((W/2-14)/Math.abs(dx||1e-9),(H/2-14)/Math.abs(dy||1e-9));
 const x=cu+dx*s,y=cv_+dy*s,a=Math.atan2(dy,dx);cx.fillStyle=col;cx.beginPath();cx.moveTo(x+12*Math.cos(a),y+12*Math.sin(a));
 cx.lineTo(x+12*Math.cos(a+2.5),y+12*Math.sin(a+2.5));cx.lineTo(x+12*Math.cos(a-2.5),y+12*Math.sin(a-2.5));cx.fill()}
function draw(){if(!D)return;const t=tNow();cx.clearRect(0,0,cv.width,cv.height);if(imgs[k]&&imgs[k].complete)cx.drawImage(imgs[k],0,0);
 if($('o_rim').checked&&D.rim){cx.strokeStyle='rgba(255,255,255,.5)';cx.lineWidth=1;cx.beginPath();let pen=false;for(const p of D.rim.uv){if(!p){pen=false;continue}pen?cx.lineTo(p[0],p[1]):cx.moveTo(p[0],p[1]);pen=true}cx.stroke()}
 if($('o_blobs').checked)for(const b of D.blobs[k]||[])circle(b[0],b[1],b[2]+3,b[3]?'rgba(160,160,160,.45)':'rgba(220,220,220,.8)',1,b[3]?[2,3]:[]);
 let hud=`t ${t.toFixed(3)} s  frame ${k}/${D.n-1}`;if(D.trigger_t!=null&&Math.abs(t-D.trigger_t)<0.009)hud+='  [TRIGGER]';
 for(const [name,s] of Object.entries(D.sources)){
  if($('o_arc').checked&&s.arc){const a=s.arc;cx.lineWidth=1.5;cx.strokeStyle=s.color;let pen=false,dashed=null;
   for(let i=0;i<a.t.length;i++){const p=a.uv[i];const d=a.t[i]>a.seen_until;if(!p){pen=false;continue}
    if(dashed!==d){if(pen)cx.stroke();cx.beginPath();cx.setLineDash(d?[6,5]:[]);dashed=d;if(i>0&&a.uv[i-1]){cx.moveTo(a.uv[i-1][0],a.uv[i-1][1]);pen=true}}
    pen?cx.lineTo(p[0],p[1]):cx.moveTo(p[0],p[1]);pen=true}cx.stroke();cx.setLineDash([]);
   if(s.reference){const q=s.reference.uv_cross;if(q&&q[0]>=0&&q[0]<D.width&&q[1]>=0&&q[1]<D.height)circle(q[0],q[1],6,s.color,2,[2,2]);else if(q)arrowTo(q[0],q[1],s.color)}}
  if($('o_trail').checked)for(let i=Math.max(0,k-30);i<k;i++){const p=s.picks[i];if(p){cx.fillStyle=s.color;cx.globalAlpha=.5;cx.fillRect(p[0]-1,p[1]-1,3,3);cx.globalAlpha=1}}
  const p=s.picks[k];if(p)circle(p[0],p[1],p[2]+(name==='colour'?2:6),s.color,name==='colour'?2.5:1.5);
  const v=verdictAt(s,t);if(v)hud+=`  ${name}: <span class="${v.verdict}">${v.verdict.toUpperCase()}</span> (miss ${v.radial_cm} cm from the axis, ${Math.round(1000*v.ttg)} ms to go)`;
  else if(s.reference)hud+=`  ${name}: -`;}
 for(const c of clicks)if(c.frame===k)circle(c.u,c.v,10,'#e879f9',2);
 if(wrong.has(k)){cx.fillStyle='rgba(248,113,113,.9)';cx.font='bold 20px system-ui';cx.fillText('marked: detection wrong',12,28)}
 $('hud').innerHTML=hud;drawTL()}
function drawTL(){const W=tl.width,H=tl.height,n=D.n,x=i=>i/(n-1)*(W-20)+10;tx.fillStyle='#181818';tx.fillRect(0,0,W,H);
 const rows=Object.entries(D.sources);tx.font='11px system-ui';rows.forEach(([name,s],r)=>{const y=6+r*18;tx.fillStyle='#777';tx.fillText(name,12,y+10);
  for(let i=0;i<n;i++){if(s.picks[i]){tx.fillStyle=s.color;tx.fillRect(x(i)-1,y+12,Math.max(2,W/n),4)}}
  if(s.series)for(const e of s.series){const i=D.t.findIndex(tt=>tt>=e.t-1e-6);if(i<0)continue;tx.fillStyle=e.verdict==='in'?'#4ade80':e.verdict==='rim'?'#fbbf24':'#f87171';tx.fillRect(x(i),y+3,Math.max(2,W/n),6)}
  if(s.reference){const tc=s.reference.t_cross;const i=tc>D.t[n-1]?n-1:D.t.findIndex(tt=>tt>=tc);if(i>=0){tx.fillStyle='#fff';tx.fillRect(x(i),y,1,18)}}});
 for(const w of wrong){tx.fillStyle='#f87171';tx.fillRect(x(w),H-10,2,8)}for(const c of clicks){tx.fillStyle='#e879f9';tx.fillRect(x(c.frame),H-10,2,8)}
 if(D.trigger_t!=null){const i=D.t.findIndex(tt=>tt>=D.trigger_t);tx.fillStyle='#a78bfa';tx.fillRect(x(i),0,1,H)}
 tx.fillStyle='#fff';tx.fillRect(x(k)-1,0,2,H)}
function go(i){k=Math.max(0,Math.min(D.n-1,i));draw()}
$('prev').onclick=()=>go(k-1);$('next').onclick=()=>go(k+1);
$('play').onclick=()=>{playing=!playing;$('play').classList.toggle('on',playing);if(playing)tick()};
function tick(){if(!playing)return;if(k>=D.n-1){playing=false;$('play').classList.remove('on');return}const dt=(D.t[k+1]-D.t[k])*1000/parseFloat($('speed').value);go(k+1);setTimeout(tick,dt)}
tl.onclick=e=>{const r=tl.getBoundingClientRect();const xx=(e.clientX-r.left)/r.width*tl.width;go(Math.round((xx-10)/(tl.width-20)*(D.n-1)))};
cv.onclick=e=>{if(!e.shiftKey)return;const r=cv.getBoundingClientRect();const u=(e.clientX-r.left)/r.width*D.width,v=(e.clientY-r.top)/r.height*D.height;
 clicks=clicks.filter(c=>c.frame!==k);clicks.push({frame:k,u:Math.round(u*10)/10,v:Math.round(v*10)/10});dirty();draw()};
document.onkeydown=e=>{if(e.target.tagName==='TEXTAREA')return;if(e.key==='ArrowRight')go(k+1);else if(e.key==='ArrowLeft')go(k-1);
 else if(e.key===' '){e.preventDefault();$('play').click()}else if(e.key==='x'||e.key==='X'){wrong.has(k)?wrong.delete(k):wrong.add(k);dirty();draw()}};
document.querySelectorAll('.call').forEach(b=>b.onclick=()=>{call=b.dataset.c;markCall();dirty()});
function markCall(){document.querySelectorAll('.call').forEach(b=>b.classList.toggle('sel',b.dataset.c===call))}
function dirty(){$('saved').textContent='(not saved)'}$('notes').oninput=dirty;
$('save').onclick=()=>fetch('label',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({call,notes:$('notes').value,clicks,wrong:[...wrong].sort((a,b)=>a-b)})})
 .then(r=>r.json()).then(j=>{$('saved').textContent=j.ok?'saved':'NOT saved: '+j.error})
</script></body></html>"""


def make_handler(rv: Review):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body: bytes, ctype: str, code: int = 200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store" if ctype.startswith(("application/json", "text/html")) else "max-age=3600")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0].strip("/").split("/")
            try:
                if p == [""]:
                    return self._send(INDEX.replace("%SESSION%", rv.d.name).encode(), "text/html; charset=utf-8")
                if p == ["index.json"]:
                    return self._send(json.dumps(rv.index()).encode(), "application/json")
                if len(p) >= 2 and p[0] == "t":
                    n = int(p[1])
                    if len(p) == 2:
                        if not self.path.endswith("/"):                      # relative URLs (data.json, f/0.jpg) need the slash
                            self.send_response(301)
                            self.send_header("Location", f"/t/{n}/")
                            self.end_headers()
                            return
                        return self._send(VIEW.replace("%N%", str(n)).encode(), "text/html; charset=utf-8")
                    if p[2] == "data.json":
                        return self._send(json.dumps(rv.data(n), default=float).encode(), "application/json")
                    if p[2] == "f":
                        return self._send(rv.jpeg(n, int(p[3].split(".")[0])), "image/jpeg")
                self._send(b"not found", "text/plain", 404)
            except Exception as e:                                           # show the error on the page, keep serving
                import traceback
                traceback.print_exc()
                self._send(json.dumps(dict(error=str(e))).encode(), "application/json", 500)

        def do_POST(self):
            p = self.path.strip("/").split("/")
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                if len(p) == 3 and p[0] == "t" and p[2] == "label":
                    rv.save_review(int(p[1]), body)
                    return self._send(b'{"ok": true}', "application/json")
                self._send(b"not found", "text/plain", 404)
            except Exception as e:
                self._send(json.dumps(dict(ok=False, error=str(e))).encode(), "application/json", 500)
    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session")
    ap.add_argument("--bucket", default=str(HERE / "bucket.json"))
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--set", nargs="*", help="detector knobs (rs_detect.DetectCfg), as rs_analyze.py")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args(argv)
    rv = Review(Path(a.session), Path(a.bucket) if a.bucket else None, A.parse_set(a.set))
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(rv))
    url = f"http://127.0.0.1:{a.port}/"
    print(f"{len(rv.dirs)} throws in {rv.d}\nreview at {url}  (Ctrl-C to stop)")
    if not a.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
