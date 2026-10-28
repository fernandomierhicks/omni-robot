// @ts-check
/* System: the robot's software as the supervisor runs it (start, stop, restart, the log), the robot computer, power,
   and in the sim the fault injection (FaultsCard). This is what the Launcher's ROS buttons did. */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { useStore } from "../ros.js";
import { runningStep, saveStepArgs, startStep, stepArgs, stopStep, sysCall } from "../system.js";
import { Banner, HelpBox } from "../ui.js";
import { FaultsCard } from "./faults.js";
import { fmtUp } from "./home.js";

export function SystemTab() {
  const s = useStore("supervisor", "status");
  const sup = s.supervisor;
  const run = runningStep();
  if (sup === false || sup === null) {
    return html`<div class="full">
      <${Banner} title="This page isn't served by the supervisor.">A step started from the Launcher's step buttons (or
        a terminal) serves the page itself, so starting and stopping the robot's software from here isn't available.
        Use the Launcher's <b>Console</b> button (<span class="mono">omni.sh console</span>); on the robot the
        supervisor starts at boot.<//>
      <${FaultsCard} />
    </div>`;
  }
  const cur = sup.current;
  const groups = Object.entries(sup.groups || {});
  return html`<div class="full">
    <${HelpBox} tab="system">
      The robot's software runs as one <b>step</b> at a time (the steps share the robot). <b>Start</b> stops the running
      one first. On the robot the step <b>Robot</b> starts at boot and is started again if it crashes; in the sim the
      steps are the Launcher's ROS buttons. The log below is the running step's. Leaving or closing the page changes
      nothing here.
    <//>
    ${sup.unreachable && html`<${Banner} red=${true} title="The page server isn't answering.">The robot (or its Wi-Fi) is down.<//>`}
    ${sup.external?.length > 0 && html`<${Banner} title="Started outside the console:">${sup.external.join("; ")}
      <button class="btn small" style="margin-left: 8px" onClick=${() => sysCall("api/stop", { external: true })}>Stop it</button><//>`}
    <div class="row" style="align-items: stretch; gap: 10px">
      <div class="card grow">
        <h2>Running now</h2>
        ${cur ? html`<div class="kv">
            <span class="k">step</span><span><b>${cur.label}</b> ${Object.entries(cur.args || {}).map(([k, v]) => html`<span class="dim mono" style="margin-left: 6px">${k}:=${v}</span>`)}</span>
            <span class="k">state</span><span class=${cur.state === "running" ? "ok" : cur.state === "exited" && cur.code ? "err" : ""}>${cur.state}${cur.state === "exited" ? ` (exit ${cur.code})` : ""}</span>
            <span class="k">${cur.state === "exited" ? "started" : "up"}</span><span>${cur.state === "exited" ? new Date(cur.since * 1000).toLocaleTimeString() : fmtUp(cur.uptime_s || 0)}</span>
          </div>
          <div class="row" style="margin-top: 8px">
            ${run && html`<button class="btn danger" onClick=${() => { if (confirm(`Stop '${cur.label}'? The robot ramps to a stop.`)) stopStep(); }}>Stop</button>`}
            <button class="btn" onClick=${() => startStep(cur.step, cur.args)}>${run ? "Restart" : "Start again"}</button>
          </div>` : html`<p class="dim">Nothing yet. Pick a step below.</p>`}
      </div>
      <${Computer} sys=${sup.system} power=${sup.power || []} />
    </div>
    ${groups.map(([g, title]) => {
      const steps = sup.steps.filter((/** @type {any} */ x) => x.group === g);
      return steps.length ? html`<h3>${title}</h3><div class="steps">${steps.map((/** @type {any} */ step) => html`<${Step} key=${step.id} step=${step} running=${run?.step === step.id} />`)}</div>` : null;
    })}
    <${Log} />
    ${sup.sim && html`<${FaultsCard} />`}
  </div>`;
}

/** @param {{step: any, running: boolean}} p */
function Step({ step, running }) {
  const [args, setArgs] = useState(() => stepArgs(step));
  /** @param {string} k @param {string} v */
  const set = (k, v) => { const n = { ...args, [k]: v }; setArgs(n); saveStepArgs(step.id, n); };
  return html`<div class=${`card step${running ? " live" : ""}`}>
    <div class="row"><b class="grow">${step.label}</b>${running && html`<span class="badge green">running</span>`}
      <button class="btn small ${running ? "" : "primary"}" onClick=${() => startStep(step.id, args)}>${running ? "Restart" : "Start"}</button></div>
    <div class="dim" style="font-size: 12px; margin: 4px 0 6px">${step.hint}${step.ends ? " It stops by itself." : ""}</div>
    <div class="row" style="gap: 10px">${(step.args || []).map((/** @type {any} */ a) => a.kind === "bool"
      ? html`<label class="row" style="gap: 4px; font-size: 12px"><input type="checkbox" checked=${args[a.key] === "true"}
          onChange=${(/** @type {any} */ e) => set(a.key, e.currentTarget.checked ? "true" : "false")} />${a.label}</label>`
      : html`<label class="row" style="gap: 4px; font-size: 12px">${a.label}<input type=${a.kind === "number" ? "number" : "text"}
          style="width: 80px" value=${args[a.key]} onChange=${(/** @type {any} */ e) => set(a.key, e.currentTarget.value)} /></label>`)}</div>
  </div>`;
}

/** @param {{sys: any, power: string[]}} p */
function Computer({ sys, power }) {
  if (!sys) return null;
  return html`<div class="card" style="min-width: 260px">
    <h2>Robot computer</h2>
    <div class="kv">
      <span class="k">name</span><span class="mono">${sys.host}</span>
      <span class="k">load (1 min)</span><span>${sys.load ?? "–"} of ${sys.cpus} cores</span>
      <span class="k">memory used</span><span>${sys.mem_used_pct != null ? `${sys.mem_used_pct}%` : "–"}</span>
      <span class="k">disk free</span><span>${sys.disk_free_gb != null ? `${sys.disk_free_gb} GB` : "–"}</span>
      <span class="k">hottest sensor</span><span>${sys.temp_c != null ? `${sys.temp_c} °C` : "–"}</span>
      <span class="k">up</span><span>${sys.uptime_s != null ? fmtUp(sys.uptime_s) : "–"}</span>
    </div>
    ${power.length > 0 && html`<div class="row" style="margin-top: 8px">${power.map((a) => html`<button class="btn small danger"
      onClick=${() => { if (confirm(`${a === "shutdown" ? "Shut down" : "Reboot"} the robot computer? The robot stops first.`)) sysCall("api/power", { action: a }); }}>
      ${a === "shutdown" ? "Shut down" : "Reboot"}</button>`)}</div>`}
  </div>`;
}

function Log() {
  const [lines, setLines] = useState(/** @type {string[]} */ ([]));
  const [step, setStep] = useState("");
  const [filter, setFilter] = useState("");
  useEffect(() => {
    let live = true;
    const get = async () => {
      try {
        const r = await (await fetch("api/log?n=300", { cache: "no-store" })).json();
        if (live) { setLines(r.lines || []); setStep(r.step || ""); }
      } catch { /* the server is down: keep the last lines */ }
    };
    get();
    const t = setInterval(get, 2000);
    return () => { live = false; clearInterval(t); };
  }, []);
  const f = filter.toLowerCase();
  const shown = f ? lines.filter((l) => l.toLowerCase().includes(f)) : lines;
  return html`<div class="card">
    <div class="row"><h2 class="grow" style="margin: 0">Log${step ? `: ${step}` : ""}</h2>
      <input type="text" placeholder="Filter (e.g. error)" value=${filter} onInput=${(/** @type {any} */ e) => setFilter(e.currentTarget.value)} /></div>
    <pre class="raw log">${shown.slice(-200).join("\n") || "(nothing yet)"}</pre>
  </div>`;
}
