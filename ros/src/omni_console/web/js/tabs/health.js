// @ts-check
/* Health: the last 30 minutes (battery, the robot computer, each drive motor's temperatures), lifetime counters and
   the maintenance list (omni_console/health.py). The robot keeps a 1 Hz history; the page asks for it when the tab
   opens and every 10 s. */
import { html, useEffect, useRef, useState } from "../../vendor/htm-preact-standalone.mjs";
import { Chart, makeBuffer, push } from "../chart.js";
import { act, command } from "../ros.js";
import { HelpBox } from "../ui.js";

const WIN = 1800;
const UNITS = { km: "km", hours: "h", throws: "", catches: "", fires: "balls" };

export function HealthTab() {
  const [rep, setRep] = useState(/** @type {any} */ (null));
  const b = useRef({ batt: makeBuffer(1, WIN), comp: makeBuffer(3, WIN), motor: makeBuffer(4, WIN), driver: makeBuffer(4, WIN),
    last: -1 }).current;
  useEffect(() => {
    const get = async () => {
      const r = await command("health", { step: b.last < 0 ? 2 : 1, since: b.last });   // then only what's new
      if (!r.ok) return;
      setRep(r.data);
      for (const row of r.data.history) {
        if (row.t <= b.last) continue;
        b.last = row.t;
        push(b.batt, row.t, [row.v]);
        push(b.comp, row.t, [row.load, row.temp, row.gpu]);
        push(b.motor, row.t, [0, 1, 2, 3].map((i) => row.motor?.[i] ?? null));
        push(b.driver, row.t, [0, 1, 2, 3].map((i) => row.driver?.[i] ?? null));
      }
    };
    get();
    const t = setInterval(get, 10000);
    return () => clearInterval(t);
  }, []);
  const wheels = [{ label: "wheel 1", color: "blue" }, { label: "wheel 2", color: "green" }, { label: "wheel 3", color: "orange" },
    { label: "wheel 4", color: "yellow" }];
  const c = rep?.counters || {};
  return html`<div class="full">
    <${HelpBox} tab="health">
      How the robot has been: the last half hour of its battery, its computer and each drive motor's winding and driver
      temperature (the DM-2325s report them), its lifetime counters, and a maintenance list. A reminder turns orange
      when it's due; <b>Done</b> resets it. The intervals are guesses until the real robot has worn something.
    <//>
    <div class="row" style="align-items: stretch; gap: 10px">
      <div class="card grow">
        <h2>Lifetime</h2>
        <div class="stats" style="grid-template-columns: repeat(5, 1fr)">${Object.entries(UNITS).map(([k, u]) => html`<div class="stat">
          <div class="v">${c[k] == null ? "–" : k === "km" ? c[k].toFixed(2) : k === "hours" ? c[k].toFixed(1) : Math.round(c[k])}</div>
          <div class="k">${k === "fires" ? "balls fired" : k}</div></div>`)}</div>
      </div>
      <div class="card" style="min-width: 380px">
        <h2>Maintenance</h2>
        ${(rep?.maintenance || []).map((/** @type {any} */ m) => html`<div class="row" style="padding: 3px 0">
          <span class=${m.due ? "warn grow" : "grow"} title=${m.what}>${m.item}: ${m.due ? "due now" : `in ${m.left} ${UNITS[/** @type {keyof typeof UNITS} */ (m.counter)] || m.counter}`}</span>
          <button class="btn small" onClick=${async () => { const r = await act("maint_done", { item: m.item, step: 2 }); if (r.ok) setRep(r.data); }}>Done</button></div>`)}
      </div>
    </div>
    <div class="charts">
      <div class="card"><${Chart} title="Battery" unit="V" span=${2} buf=${b.batt} series=${[{ label: "pack", color: "green" }]} /></div>
      <div class="card"><${Chart} title="Robot computer" unit="" span=${10} buf=${b.comp} series=${[
        { label: "load", color: "blue" }, { label: "°C", color: "orange" }, { label: "GPU %", color: "green" }]} /></div>
      <div class="card"><${Chart} title="Motor windings" unit="°C" span=${10} buf=${b.motor} series=${wheels} /></div>
      <div class="card"><${Chart} title="Motor drivers" unit="°C" span=${10} buf=${b.driver} series=${wheels} /></div>
    </div>
  </div>`;
}
