// @ts-check
/* Test › Preflight: the robot checks itself every status (omni_console/checks.py preflight), nothing moves. A red row
   refuses the tasks it names, with its own words, wherever they are started from (the page, the voice, an AI). The
   page asks for the rows every 2 s while it's open; the status bar carries the counts. */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { features } from "../features.js";
import { command, useStore } from "../ros.js";
import { HelpBox } from "../ui.js";

const ICON = { ok: "✓", warn: "!", fail: "✕", na: "–" };
const CLS = { ok: "ok", warn: "warn", fail: "err", na: "dim" };

export function PreflightTab() {
  const s = useStore("conn", "status");
  const [rows, setRows] = useState(/** @type {any[] | null} */ (null));
  const list = features();
  useEffect(() => {
    let alive = true;
    const get = async () => {
      const r = await command("preflight");
      if (alive && r.ok) setRows(r.data.assessed ? r.data.rows : []);
    };
    get();
    const t = setInterval(get, 2000);
    return () => { alive = false; clearInterval(t); };
  }, [s.conn]);
  const label = (/** @type {string} */ id) => id === "motion" ? "everything that moves" : list.find((x) => x.id === id)?.label || id;
  const n = s.status?.preflight;
  return html`<div class="full" style="max-width: 900px">
    <${HelpBox} tab="preflight">
      The robot checks itself all the time; nothing here moves it. <b class="err">✕</b> fails a check that would make a
      task fail anyway (a silent or faulted MCU, firmware that speaks another protocol, a flat pack, lost localization,
      an unhealthy arm) and refuses the tasks named beside it until it clears. <b class="warn">!</b> is worth a look
      but refuses nothing; <b class="dim">–</b> means the robot can't see that part (not running, or the sim has none).
      Before a session on the real robot, go through <a href="#bringup">Bring-up</a> too.
    <//>
    ${n && html`<div class="row" style="gap: 10px; margin-bottom: 10px">
      <span class="badge green">${n.ok} ok</span>${n.warn > 0 && html`<span class="badge orange">${n.warn} to look at</span>`}
      ${n.fail > 0 && html`<span class="badge red">${n.fail} failing</span>`}<span class="dim">${n.na} not seen</span></div>`}
    ${rows === null ? html`<p class="dim">${s.conn === "open" ? "Asking the robot..." : "Not connected to the robot."}</p>`
      : !rows.length ? html`<p class="dim">The robot hasn't assessed anything yet (its console node starts it with the first status).</p>`
      : html`<div class="card"><table class="checks">
        ${rows.map((r) => html`<tr key=${r.id} class=${`st-${r.state}`}>
          <td class=${`ic ${CLS[/** @type {keyof typeof CLS} */ (r.state)]}`}>${ICON[/** @type {keyof typeof ICON} */ (r.state)]}</td>
          <td class="lbl">${r.label}</td>
          <td>${r.detail}${r.blocks?.length > 0 && html`<div class="err" style="font-size: 12px">Refuses: ${r.blocks.map(label).join(", ")}</div>`}</td>
        </tr>`)}</table></div>`}
  </div>`;
}
