// @ts-check
/* Test › Bring-up: the guided checks of design.md 8 (omni_console/checks.py BRINGUP), one at a time and attended. Each
   says what it proves, the preconditions to tick, the steps, what to expect, the numbers to write down and the status
   fields to watch live; its buttons send ordinary console verbs (nothing moves by itself). Pass, Fail or Skip stores the
   outcome on the robot with its configuration (bringup.json and the session log): measured results feed the models. */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { act, command, useStore } from "../ros.js";
import { HelpBox, clock } from "../ui.js";
import { defaultUrl, usePref } from "./cameras.js";

/** An automatic check's last result (checks.py BRINGUP auto="lidar"): camera_snap's lidar self-test, run at boot, and
 *  Test again (GET /lidar/test: it spins the lidar a few seconds, then stops it). The server is the one the Lidar and
 *  Cameras pages use. @param {{st: any}} p */
function LidarSelfTest({ st }) {
  const [url] = usePref("url", defaultUrl());
  const [res, setRes] = useState(/** @type {any} */ (null));
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState("");
  useEffect(() => {
    fetch(`${url}/lidar?quiet=1`, { cache: "no-store" }).then((r) => r.json())
      .then((j) => { if (j.self_test) setRes(j.self_test); }).catch(() => setErr(`camera_snap at ${url} isn't answering`));
  }, [url]);
  const run = async () => {
    setBusy(true); setErr("");
    try {
      const r = await fetch(`${url}/lidar/test`, { cache: "no-store" });
      const j = await r.json();
      if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
      setRes(j);
    } catch (e) { setErr(String(e)); }
    setBusy(false);
  };
  const t = res || st?.lidar_test;
  const cls = !t ? "dim" : t.ok ? "ok" : t.ok === null ? "warn" : "err";
  return html`<div class="card"><h2>Last result</h2>
    ${t ? html`<div class="kv">
        <span class="k">result</span><span class=${cls}><b>${t.ok ? "passed" : t.ok === null ? "skipped" : "failed"}</b> · ${clock(t.t)}</span>
        ${t.model && html`<span class="k">lidar</span><span>${t.model} (firmware ${t.firmware}) on ${t.port}</span>`}
        ${t.health && html`<span class="k">health</span><span>${t.health}</span>`}
        ${t.hz != null && html`<span class="k">turns</span><span>${t.hz} /s, ${t.samples} samples a turn${t.range_m ? `, median range ${t.range_m} m` : ""}</span>`}
        ${t.detail && html`<span class="k">detail</span><span>${t.detail}</span>`}</div>`
      : html`<p class="dim">No self-test yet on this robot (camera_snap runs one at boot).</p>`}
    <div class="row" style="margin-top: 8px"><button class="btn" disabled=${busy} onClick=${run}>${busy ? "Testing (it spins ~4 s)..." : "Test again"}</button>
      <span class="err" style="font-size: 12px">${err}</span></div></div>`;
}

/** a.b.c of the status @param {any} o @param {string} path */
const pick = (o, path) => path.split(".").reduce((v, k) => (v == null ? v : v[k]), o);

/** @param {any} v */
function fmt(v) {
  if (v == null) return "–";
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : v.toFixed(3);
  if (Array.isArray(v)) return v.map((x) => (typeof x === "object" && x ? x.name || JSON.stringify(x) : fmt(x))).join(", ") || "none";
  if (typeof v === "boolean") return v ? "yes" : "no";
  return String(v);
}

export function BringupTab() {
  const s = useStore("conn", "status");
  const [data, setData] = useState(/** @type {any} */ (null));
  const [sel, setSel] = useState(/** @type {string | null} */ (null));
  useEffect(() => {
    command("tests").then((r) => { if (r.ok) { setData(r.data); setSel((x) => x || r.data.checks[0]?.id); } });
  }, [s.conn]);
  if (!data) return html`<div class="full"><p class="dim">${s.conn === "open" ? "Asking the robot..." : "Not connected to the robot."}</p></div>`;
  const checks = /** @type {any[]} */ (data.checks);
  const last = (/** @type {string} */ id) => (data.results[id] || []).slice(-1)[0];
  const c = checks.find((x) => x.id === sel) || checks[0];
  // one tab per subsystem (the checks' step): its checks only, so none is a scroll away
  const steps = [...new Set(checks.map((x) => x.step))];
  const done = (/** @type {string} */ step) => checks.filter((x) => x.step === step && last(x.id)?.result === "pass").length;
  return html`
    <nav class="subbar">${steps.map((p) => {
      const n = checks.filter((x) => x.step === p).length;
      return html`<button key=${p} class=${`sub${p === c.step ? " sel" : ""}`} title=${`${p}: ${done(p)} of ${n} passed`}
        onClick=${() => setSel(checks.find((x) => x.step === p).id)}>${p.replace(/^\d+\s*/, "")}
        <span class="dim"> ${done(p)}/${n}</span></button>`;
    })}</nav>
    <div class="cols">
    <div class="side">
      ${checks.filter((x) => x.step === c.step).map((x) => {
        const r = last(x.id);
        return html`<button class=${`listrow${x.id === c.id ? " sel" : ""}`} onClick=${() => setSel(x.id)}>
          <span class="grow">${x.title}</span>${r ? html`<span class=${`badge ${r.result === "pass" ? "green" : r.result === "fail" ? "red" : ""}`}>${r.result}</span>` : html`<span class="dim">–</span>`}</button>`;
      })}
      <${HelpBox} tab="bringup">
        The checks the real robot goes through before it is trusted, in the order the build meets them (design.md 8),
        one tab per stage. Attended, one at a time: tick the preconditions, follow the steps, write down what you
        measured and press Pass, Fail or Skip. The results stay on the robot with its firmware, map and profile; the
        thresholds are estimates until the robot has been measured.
      <//>
    </div>
    <div class="main full"><${Check} key=${c.id} c=${c} st=${s.status} results=${data.results[c.id] || []}
      onSaved=${(/** @type {any} */ res) => setData({ ...data, results: res })} /></div>
  </div>`;
}

/** @param {{c: any, st: any, results: any[], onSaved: (r: any) => void}} p */
function Check({ c, st, results, onSaved }) {
  const [pre, setPre] = useState(/** @type {string[]} */ ([]));
  const [vals, setVals] = useState(/** @type {Record<string, string>} */ ({}));
  const [notes, setNotes] = useState("");
  const [msg, setMsg] = useState("");
  const allPre = c.pre.every((/** @type {string} */ x) => pre.includes(x));
  /** @param {string} result */
  const save = async (result) => {
    const r = await act("test_record", { id: c.id, result, measured: vals, pre, notes });
    if (r.ok) { onSaved(r.data.results); setMsg(r.message); setVals({}); setNotes(""); }
  };
  const sim = st?.source === "sim";
  return html`<div style="max-width: 820px">
    <div class="card">
      <h2>${c.title}</h2>
      <p>${c.why}</p>
      ${sim && !c.sim && html`<p class="warn">Real robot only: the sim can't show this.</p>`}
      ${c.pre.length > 0 && html`<h3>Before you start</h3>${c.pre.map((/** @type {string} */ x) => html`<label class="row" style="padding: 2px 0">
        <input type="checkbox" checked=${pre.includes(x)} onChange=${(/** @type {any} */ e) =>
          setPre(e.currentTarget.checked ? [...pre, x] : pre.filter((y) => y !== x))} /> ${x}</label>`)}`}
      <h3>Steps</h3><ol>${c.steps.map((/** @type {string} */ x) => html`<li>${x}</li>`)}</ol>
      ${c.actions.length > 0 && html`<div class="row" style="flex-wrap: wrap; gap: 6px">
        ${c.actions.map((/** @type {any} */ a) => html`<button class=${`btn small${a.verb === "stop" ? " danger" : ""}`}
          disabled=${!allPre && a.verb !== "stop"} title=${allPre ? "" : "Tick the preconditions first"}
          onClick=${() => act(a.verb, a.args)}>${a.label}</button>`)}</div>`}
      <h3>Expect</h3><p>${c.expect}</p>
    </div>
    ${c.auto === "lidar" && html`<${LidarSelfTest} st=${st} />`}
    ${c.watch.length > 0 && html`<div class="card"><h2>Live</h2><div class="kv">
      ${c.watch.map((/** @type {string} */ w) => html`<span class="k mono">${w}</span><span class="mono">${fmt(pick(st, w))}</span>`)}</div></div>`}
    <div class="card">
      <h2>Result</h2>
      ${c.measure.length > 0 && html`<div class="kv">${c.measure.map((/** @type {any} */ m) => html`<span class="k">${m.label}</span>
        <span class="row" style="gap: 4px"><input type="number" step="any" value=${vals[m.key] ?? ""} style="width: 7em"
          onInput=${(/** @type {any} */ e) => setVals({ ...vals, [m.key]: e.currentTarget.value })} /><span class="dim">${m.unit}</span></span>`)}</div>`}
      <textarea placeholder="Notes (what you saw, the setup)" value=${notes} rows="2" style="width: 100%; margin-top: 8px"
        onInput=${(/** @type {any} */ e) => setNotes(e.currentTarget.value)}></textarea>
      <div class="row" style="margin-top: 8px">
        <button class="btn go" onClick=${() => save("pass")}>Pass</button>
        <button class="btn danger" onClick=${() => save("fail")}>Fail</button>
        <button class="btn" onClick=${() => save("skip")}>Skip</button>
        <span class="dim">${msg}</span>
      </div>
    </div>
    ${results.length > 0 && html`<div class="card"><h2>Earlier results</h2>
      ${results.slice().reverse().map((r) => html`<div class="ev">
        <span class="mono dim">${new Date(r.t * 1000).toLocaleDateString()} ${clock(r.t)}</span>
        <span class=${r.result === "pass" ? "ok" : r.result === "fail" ? "err" : "dim"}>${r.result}</span>
        <span>${Object.entries(r.measured).map(([k, v]) => `${k} ${v}`).join(", ")}${r.notes ? ` · ${r.notes}` : ""}
          <span class="dim"> · ${r.config?.source || ""} fw ${r.config?.firmware || "?"}${r.config?.kid ? " · kid" : ""}</span></span></div>`)}
    </div>`}
  </div>`;
}
