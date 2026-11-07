// @ts-check
/* Drive › Call (telepresence): a visitor looks through the robot, drives it and talks through it. The robot's camera fills
   the page with the drive pads over it (and the gamepad, as everywhere); the visitor's name is announced on the robot;
   they talk with push-to-talk (the browser records a WAV, the page server keeps it, the face plays it as it is: the
   supervisor's /api/clip, the console's "clip" verb, omni_face's /face/clip) or type a line the robot says in its own
   voice; Listen plays the robot's microphone (the supervisor's /api/mic: raw 16 kHz PCM, played with Web Audio), the
   robot saying so first. Browsers allow the visitor's microphone only on a secure page: the supervisor's https port
   (--https, a self-signed certificate) serves one, rosbridge and the camera streams tunnelled through it. Not here:
   reaching the robot from outside the home network (a VPN such as Tailscale to the robot computer). */
import { html, useEffect, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { Pads } from "../drive.js";
import { act, command, note, useStore } from "../ros.js";
import { SayBox } from "../say.js";
import { Banner, HelpBox } from "../ui.js";
import { mainCamera } from "./home.js";
import { streamUrl } from "./vision.js";

const RATE = 16000;                  // Hz: the clip's sample rate (speech; small files)
const MAX_S = 30;                    // s: the longest clip

/** 16-bit mono PCM WAV of these samples. @param {Float32Array} pcm @param {number} rate */
export function wav(pcm, rate) {
  const buf = new ArrayBuffer(44 + pcm.length * 2);
  const v = new DataView(buf);
  const str = (/** @type {number} */ o, /** @type {string} */ s) => { for (let i = 0; i < s.length; i++) v.setUint8(o + i, s.charCodeAt(i)); };
  str(0, "RIFF"); v.setUint32(4, 36 + pcm.length * 2, true); str(8, "WAVE"); str(12, "fmt ");
  v.setUint32(16, 16, true); v.setUint16(20, 1, true); v.setUint16(22, 1, true); v.setUint32(24, rate, true);
  v.setUint32(28, rate * 2, true); v.setUint16(32, 2, true); v.setUint16(34, 16, true); str(36, "data");
  v.setUint32(40, pcm.length * 2, true);
  for (let i = 0; i < pcm.length; i++) v.setInt16(44 + 2 * i, Math.max(-1, Math.min(1, pcm[i])) * 0x7fff, true);
  return new Blob([buf], { type: "audio/wav" });
}

/** Down to rate by averaging (speech: plenty). @param {Float32Array} x @param {number} from @param {number} to */
export function resample(x, from, to) {
  if (from === to) return x;
  const k = from / to, n = Math.floor(x.length / k), out = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    const a = Math.floor(i * k), b = Math.min(x.length, Math.floor((i + 1) * k));
    let s = 0;
    for (let j = a; j < b; j++) s += x[j];
    out[i] = b > a ? s / (b - a) : 0;
  }
  return out;
}

/** The robot's microphone: /api/mic's raw s16le 16 kHz mono, scheduled a little ahead (a jitter buffer) on an
 *  AudioContext at 16 kHz. Returns stop(). @param {(peak: number) => void} onLevel @param {(msg: string) => void} onEnd */
export function listen(onLevel, onEnd) {
  const ctl = new AbortController();
  const ac = new AudioContext({ sampleRate: RATE });
  let next = 0, carry = new Uint8Array(0);
  let pin = "";
  try { pin = sessionStorage.getItem("console.pin") || ""; } catch { /* */ }
  (async () => {
    try {
      const res = await fetch(`api/mic?pin=${encodeURIComponent(pin)}`, { signal: ctl.signal });
      if (!res.ok || !res.body) { onEnd((await res.json().catch(() => ({}))).message || `no audio (${res.status})`); return; }
      const reader = res.body.getReader();
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        const bytes = new Uint8Array(carry.length + value.length);
        bytes.set(carry); bytes.set(value, carry.length);
        const n = bytes.length >> 1;
        carry = bytes.slice(n * 2);
        const dv = new DataView(bytes.buffer, 0, n * 2);
        const pcm = new Float32Array(n);
        let peak = 0;
        for (let i = 0; i < n; i++) { pcm[i] = dv.getInt16(2 * i, true) / 32768; peak = Math.max(peak, Math.abs(pcm[i])); }
        onLevel(peak);
        if (!n) continue;
        const buf = ac.createBuffer(1, n, RATE);
        buf.copyToChannel(pcm, 0);
        const src = ac.createBufferSource();
        src.buffer = buf;
        src.connect(ac.destination);
        next = Math.max(next, ac.currentTime + 0.2);      // 0.2 s ahead: Wi-Fi jitter, not a lag that grows
        if (next - ac.currentTime > 1.0) next = ac.currentTime + 0.2;
        src.start(next);
        next += n / RATE;
      }
      onEnd("the robot stopped sending");
    } catch (e) {
      if (/** @type {Error} */ (e).name !== "AbortError") onEnd(`no audio: ${/** @type {Error} */ (e).message}`);
    }
  })();
  return () => { ctl.abort(); ac.close(); };
}

export function CallTab() {
  const s = useStore("status", "httpsPort");
  const st = s.status;
  const [name, setName] = useState(() => { try { return localStorage.getItem("console.caller") || ""; } catch { return ""; } });
  const [rec, setRec] = useState(false);
  const [level, setLevel] = useState(0);
  const [msg, setMsg] = useState("");
  const [hear, setHear] = useState(0);
  const ctx = useRef(/** @type {any} */ (null));
  const ear = useRef(/** @type {null | (() => void)} */ (null));
  useEffect(() => () => { ear.current?.(); }, []);
  const toggleListen = () => {
    if (ear.current) { ear.current(); ear.current = null; setHear(0); return; }
    act("say", { text: `${name || "Someone"} is listening through me.`, expression: "auto" }, true);
    ear.current = listen((p) => setHear(p), (m) => { setMsg(m); ear.current = null; setHear(0); });
    setHear(0.001);
  };
  const secureUrl = s.httpsPort ? `https://${location.hostname}:${s.httpsPort}${location.pathname}#call` : null;
  const driving = st?.mission === "MANUAL";
  const v = st?.vision;
  const cam = v?.video_port && v.images?.length ? mainCamera(v.images) : null;
  const secure = window.isSecureContext && !!navigator.mediaDevices?.getUserMedia;
  useEffect(() => () => { ctx.current?.stop?.(); }, []);
  const ring = () => {
    try { localStorage.setItem("console.caller", name); } catch { /* */ }
    act("say", { text: `${name || "Someone"} is calling through me. Say hello.`, expression: "excited" });
  };
  const start = async () => {
    if (rec || !secure) return;
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true } });
      const ac = new AudioContext();
      const src = ac.createMediaStreamSource(stream);
      const proc = ac.createScriptProcessor(4096, 1, 1);
      /** @type {Float32Array[]} */
      const chunks = [];
      let n = 0;
      proc.onaudioprocess = (e) => {
        const x = new Float32Array(e.inputBuffer.getChannelData(0));
        chunks.push(x);
        n += x.length;
        let peak = 0;
        for (let i = 0; i < x.length; i += 16) peak = Math.max(peak, Math.abs(x[i]));
        setLevel(peak);
        if (n > ac.sampleRate * MAX_S) stop();
      };
      src.connect(proc);
      proc.connect(ac.destination);
      ctx.current = { ac, stream, proc, chunks, rate: ac.sampleRate, stop: () => { stream.getTracks().forEach((t) => t.stop()); ac.close(); } };
      setRec(true);
      setMsg("");
    } catch (e) {
      setMsg(`No microphone: ${/** @type {Error} */ (e).message}`);
    }
  };
  const stop = async () => {
    const c = ctx.current;
    if (!c) return;
    ctx.current = null;
    setRec(false);
    setLevel(0);
    c.stop();
    const all = new Float32Array(c.chunks.reduce((a, x) => a + x.length, 0));
    let o = 0;
    for (const x of c.chunks) { all.set(x, o); o += x.length; }
    if (all.length < c.rate * 0.3) { setMsg("Too short: hold the button while you talk"); return; }
    const blob = wav(resample(all, c.rate, RATE), RATE);
    let pin = "";
    try { pin = sessionStorage.getItem("console.pin") || ""; } catch { /* */ }
    try {
      const res = await (await fetch(`api/clip?pin=${encodeURIComponent(pin)}`, { method: "POST", body: blob,
        headers: { "Content-Type": "audio/wav" } })).json();
      if (!res.ok) { setMsg(res.message || "The page server refused the clip"); return; }
      const r = await command("clip", { name: res.name, caption: `${name || "Caller"}:` });
      setMsg(r.ok ? `Sent (${(all.length / c.rate).toFixed(1)} s)` : r.message);
    } catch (e) {
      setMsg("No page server for clips (start the console from the Launcher or the supervisor)");
      note("error", `clip: ${/** @type {Error} */ (e).message}`);
    }
  };
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="call">
        Visit through the robot. <b>Take control</b> to drive it (the pads on the picture, the gamepad, or W A S D), and
        press Ring to tell the room who's calling. <b>Hold to talk</b>: what you say plays on the robot's speaker when
        you let go. Or type a line for the robot to say. <b>Listen to the room</b> plays the robot's microphone (the
        robot says you're listening first). From outside the home you need a VPN to the robot.
      <//>
      <h3>Who's calling</h3>
      <div class="row"><input type="text" placeholder="Your name" value=${name} onInput=${(/** @type {any} */ e) => setName(e.currentTarget.value)} />
        <button class="btn" disabled=${!st} onClick=${ring}>Ring</button></div>
      <h3>Talk</h3>
      ${secure ? html`<button class=${`btn big${rec ? " danger" : " go"}`} style="width: 100%; touch-action: none"
          onPointerDown=${start} onPointerUp=${stop} onPointerLeave=${() => rec && stop()} onPointerCancel=${stop}
          disabled=${!st}>${rec ? "Recording... let go to send" : "Hold to talk"}</button>
          <div class="bar" style="margin-top: 6px"><div style=${`width: ${Math.round(Math.min(1, level * 2) * 100)}%`}></div></div>`
        : html`<${Banner} title="Push-to-talk is off.">The browser allows the microphone only on a secure page (this one
          is ${location.protocol}//${location.host}). ${secureUrl
            ? html`<a href=${secureUrl}>Open the secure page</a> (the robot's own certificate: accept the warning once).`
            : "Start the supervisor with --https 8443 for one."} Typed lines below still work.<//>`}
      ${msg && html`<div class="dim" style="margin-top: 4px">${msg}</div>`}
      <h3>Listen</h3>
      <button class=${`btn${ear.current ? " danger" : ""}`} style="width: 100%" disabled=${!st} onClick=${toggleListen}>
        ${ear.current ? "Stop listening" : "Listen to the room"}</button>
      ${ear.current && html`<div class="bar" style="margin-top: 6px"><div style=${`width: ${Math.round(Math.min(1, hear * 2) * 100)}%`}></div></div>`}
      <h3>Say a line</h3>
      <${SayBox} compact />
      <div class="row" style="margin-top: 12px">
        ${driving ? html`<span class="ok">You're driving</span>`
          : html`<button class="btn go" disabled=${!st} onClick=${() => act("mission", { name: "MANUAL" })}>Take control</button>`}
      </div>
    </div>
    <div class="main call-view">
      ${cam ? html`<img alt=${cam} src=${streamUrl(location.hostname || "localhost", v.video_port, cam)} />`
        : html`<p class="dim" style="padding: 20px">${st ? "No camera stream (web_video_server isn't running)" : "Not connected to the robot"}</p>`}
      ${driving && html`<${Pads} />`}
    </div>
  </div>`;
}
