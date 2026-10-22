// @ts-check
/* Pieces every tab uses: the "How to use" box, the settings form drawn from the robot's schema, small helpers. */
import { html, useEffect, useState } from "../vendor/htm-preact-standalone.mjs";
import { act, useStore } from "./ros.js";

/** Open the first time a tab is seen, then as the operator left it.
 *  @param {{tab: string, children: any}} p */
export function HelpBox({ tab, children }) {
  const key = `console.help.${tab}`;
  let open = true;
  try { open = localStorage.getItem(key) !== "closed"; } catch { /* */ }
  /** @param {any} e */
  const toggle = (e) => { try { localStorage.setItem(key, e.currentTarget.open ? "open" : "closed"); } catch { /* */ } };
  return html`<details class="help" open=${open} onToggle=${toggle}>
    <summary>How to use this tab</summary><div class="body">${children}</div></details>`;
}

/** @param {{tab: string, title?: string}} p */
export function SettingsForm({ tab, title = "Settings" }) {
  const s = useStore("schema");
  const rows = (s.schema?.settings || []).filter((/** @type {any} */ x) => x.tabs.includes(tab));
  if (!rows.length) return s.schema ? null : html`<p class="dim">Settings appear when the robot answers.</p>`;
  return html`<h3>${title}</h3>${rows.map((/** @type {any} */ r) => html`<${SettingRow} key=${r.key} s=${r} />`)}`;
}

/** One setting: dashed while it's on its way, a green flash when the robot echoes it, the reason when refused.
 *  @param {{s: any}} p */
function SettingRow({ s }) {
  const [draft, setDraft] = useState(/** @type {any} */ (null));
  const [state, setState] = useState("");
  const [msg, setMsg] = useState("");
  useEffect(() => { setDraft(null); }, [s.value]);
  const value = draft ?? s.value;
  /** @param {any} v */
  const send = async (v) => {
    setDraft(v); setState("pending"); setMsg("");
    const r = await act("set", { key: s.key, value: v }, true);
    if (r.ok) { setState("flash"); setTimeout(() => setState(""), 700); }
    else { setState(""); setDraft(null); setMsg(r.message); }
  };
  if (s.kind === "bool") {
    return html`<div class=${`setting ${state}`} title=${s.hint}>
      <div class="top"><label for=${s.key}>${s.label}</label>
        <span class="switch"><input id=${s.key} type="checkbox" checked=${!!value}
          onChange=${(/** @type {any} */ e) => send(e.currentTarget.checked)} /><span
          onClick=${() => send(!value)}></span></span></div>
      <div class="hint">${s.hint}</div>${msg && html`<div class="msg">${msg}</div>`}</div>`;
  }
  /** @param {any} e */
  const fromInput = (e) => {
    const v = parseFloat(e.currentTarget.value);
    if (Number.isFinite(v)) send(v);
  };
  return html`<div class=${`setting ${state}`} title=${s.hint}>
    <div class="top"><label for=${s.key}>${s.label}</label>
      <input id=${s.key} type="number" min=${s.min} max=${s.max} step=${s.step} value=${value}
        onChange=${fromInput} /><span class="unit">${s.unit}</span></div>
    <input type="range" min=${s.min} max=${s.max} step=${s.step} value=${value} aria-label=${s.label}
      onInput=${(/** @type {any} */ e) => setDraft(parseFloat(e.currentTarget.value))} onChange=${fromInput} />
    <div class="hint">${s.hint}</div>${msg && html`<div class="msg">${msg}</div>`}</div>`;
}

/** @param {{title: string, children: any, red?: boolean}} p */
export function Banner({ title, children, red = false }) {
  return html`<div class=${red ? "banner red" : "banner"}><b>${title}</b> ${children}</div>`;
}

/** @param {number} rad */
export const deg = (rad) => Math.round((rad * 180) / Math.PI);

/** @param {number} t seconds since 1970 */
export function clock(t) {
  const d = new Date(t * 1000);
  return d.toLocaleTimeString([], { hour12: false });
}
