// @ts-check
/* Program: kids line up blocks (forward, turn, spin, wiggle, wait, go to a place, repeat) and press Run; the robot
   does them one by one (omni_console/program.py). Programs are saved on the robot. */
import { html, useState } from "../../vendor/htm-preact-standalone.mjs";
import { MapView } from "../mapview.js";
import { act, useStore } from "../ros.js";
import { HelpBox, SettingsForm } from "../ui.js";

/** kind: [label, colour token, the block it adds] */
const PALETTE = {
  forward: ["Forward", "green", { do: "forward", m: 0.5 }],
  back: ["Back", "green", { do: "back", m: 0.5 }],
  left: ["Slide left", "green", { do: "left", m: 0.5 }],
  right: ["Slide right", "green", { do: "right", m: 0.5 }],
  turnl: ["Turn left", "blue", { do: "turn", deg: 90 }],
  turnr: ["Turn right", "blue", { do: "turn", deg: -90 }],
  spin: ["Spin", "blue", { do: "spin", turns: 1 }],
  wiggle: ["Wiggle", "yellow", { do: "wiggle" }],
  wait: ["Wait", "dim", { do: "wait", s: 1 }],
  goto: ["Go to a place", "orange", { do: "goto", place: "" }],
  repeat: ["Repeat", "red", { do: "repeat", times: 2, body: [] }],
};
const EXAMPLES = {
  "Square dance": [{ do: "repeat", times: 4, body: [{ do: "forward", m: 0.6 }, { do: "turn", deg: 90 }] }, { do: "wiggle" }],
  "Zig zag": [{ do: "repeat", times: 3, body: [{ do: "forward", m: 0.4 }, { do: "left", m: 0.3 }, { do: "forward", m: 0.4 }, { do: "right", m: 0.3 }] }],
  "Twirl": [{ do: "spin", turns: 2 }, { do: "wait", s: 1 }, { do: "spin", turns: 1 }, { do: "wiggle" }],
};

/** @param {any} b */
const colourOf = (b) => {
  if (b.do === "turn") return "blue";
  const k = Object.values(PALETTE).find((p) => p[2].do === b.do);
  return k ? k[1] : "dim";
};

export function ProgramTab() {
  const s = useStore("status", "schema");
  const st = s.status;
  const programs = s.schema?.programs || {};
  const places = Object.keys(s.schema?.places || {});
  const [blocks, setBlocks] = useState(/** @type {any[]} */ ([]));
  const [name, setName] = useState("");
  const [into, setInto] = useState(/** @type {number[]} */ ([]));        // which list the palette adds to: a path
  const run = st?.mission === "PROGRAM" ? st.program : null;
  /** @param {number[]} path @param {(list: any[]) => any[]} f */
  const edit = (path, f) => setBlocks((top) => {
    const copy = JSON.parse(JSON.stringify(top));
    let list = copy;
    for (const i of path) list = list[i].body;
    const out = f(list);
    list.splice(0, list.length, ...out);
    return copy;
  });
  /** @param {string} k */
  const add = (k) => edit(into, (l) => l.concat([JSON.parse(JSON.stringify(PALETTE[/** @type {keyof typeof PALETTE} */ (k)][2]))]));
  const start = () => act("program_run", { name: name.trim() || "unsaved", blocks });
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="program">
        <ol>
          <li>Press the coloured blocks to add them to your program. A <b>Repeat</b> block repeats what's inside it:
            press its <b>add inside</b>, then the blocks.</li>
          <li>Change the numbers (metres, degrees, seconds, times). Turn left is +, right is −.</li>
          <li>Press <b>Run</b>. The robot does the blocks one by one; <b>Stop</b> (or <kbd>Space</kbd>) stops it.</li>
          <li>Name it and press <b>Save</b> to keep it for next time.</li>
        </ol>
        Give it room: forward 1 m means a metre of floor in front of it. "Go to a place" uses the places saved in Go
        To and needs navigation. A block it can't finish (something in the way) stops the program and says why.
      <//>
      ${run ? html`<div class="card">
          <div class="row"><b class="grow">Running '${run.name}'</b><button class="btn danger small" onClick=${() => act("stop")}>Stop</button></div>
          <div style="margin: 6px 0">${run.label}</div>
          <div class="bar"><div style=${`width: ${Math.round((st.progress || 0) * 100)}%`}></div></div></div>`
        : html`<div class="row" style="gap: 8px; margin-bottom: 10px">
          <button class="btn go big grow" style="width: auto" disabled=${!st || !blocks.length} onClick=${start}>Run</button>
          <button class="btn big" style="width: auto" disabled=${!blocks.length} onClick=${() => { setBlocks([]); setInto([]); }}>Clear</button></div>`}
      <h3>Blocks ${into.length ? html`<span class="dim">(adding inside a repeat: <a href="#program" onClick=${(/** @type {any} */ e) => { e.preventDefault(); setInto([]); }}>back to the top</a>)</span>` : ""}</h3>
      <div class="palette">${Object.entries(PALETTE).map(([k, [label, col]]) => html`<button class="blk"
        style=${`--c: var(--${col})`} onClick=${() => add(k)}>${label}</button>`)}</div>
      <h3>Saved</h3>
      <ul class="list">${Object.keys(programs).map((n) => html`<li>
        <span class="grow">${n} <span class="dim">(${programs[n].length} blocks)</span></span>
        <button class="btn small" onClick=${() => { setBlocks(JSON.parse(JSON.stringify(programs[n]))); setName(n); setInto([]); }}>Load</button>
        <button class="btn small go" disabled=${!st} onClick=${() => act("program_run", { name: n })}>Run</button>
        <button class="btn small danger" onClick=${() => { if (confirm(`Delete '${n}'?`)) act("program_delete", { name: n }); }}>✕</button></li>`)}
        ${Object.entries(EXAMPLES).map(([n, b]) => html`<li><span class="grow dim">Example: ${n}</span>
          <button class="btn small" onClick=${() => { setBlocks(JSON.parse(JSON.stringify(b))); setName(n); setInto([]); }}>Load</button></li>`)}</ul>
      <${SettingsForm} tab="program" />
    </div>
    <div class="main full">
      <div class="row" style="margin-bottom: 8px">
        <input type="text" class="grow" placeholder="Program name, e.g. Ana's dance" value=${name}
          onInput=${(/** @type {any} */ e) => setName(e.currentTarget.value)} />
        <button class="btn primary" disabled=${!name.trim() || !blocks.length}
          onClick=${() => act("program_save", { name: name.trim(), blocks })}>Save</button>
      </div>
      ${blocks.length ? html`<${BlockList} list=${blocks} path=${[]} edit=${edit} places=${places} into=${into} setInto=${setInto} active=${run} />`
        : html`<p class="dim" style="font-size: 16px">Press the blocks on the left to start, or load an example.</p>`}
      <div style="height: 260px; position: relative; margin-top: 12px; border-radius: 8px; overflow: hidden">
        <${MapView} hint="Where the robot is" />
      </div>
    </div>
  </div>`;
}

/** @param {{list: any[], path: number[], edit: any, places: string[], into: number[], setInto: any, active: any}} p */
function BlockList({ list, path, edit, places, into, setInto, active }) {
  /** @param {number} i @param {Record<string, any>} patch */
  const set = (i, patch) => edit(path, (/** @type {any[]} */ l) => l.map((b, k) => (k === i ? { ...b, ...patch } : b)));
  /** @param {number} i @param {number} d */
  const move = (i, d) => edit(path, (/** @type {any[]} */ l) => {
    const j = i + d;
    if (j < 0 || j >= l.length) return l;
    const c = l.slice(); [c[i], c[j]] = [c[j], c[i]]; return c;
  });
  const num = (/** @type {number} */ i, /** @type {string} */ k, /** @type {number} */ v, /** @type {number} */ step, /** @type {string} */ unit) =>
    html`<input type="number" step=${step} value=${v} onChange=${(/** @type {any} */ e) => set(i, { [k]: parseFloat(e.currentTarget.value) || 0 })} /><span class="dim">${unit}</span>`;
  return html`<div class="blocks">${list.map((b, i) => html`<div class="block" style=${`--c: var(--${colourOf(b)})`}>
    <div class="row">
      <b class="grow">${b.do === "turn" ? (b.deg >= 0 ? "Turn left" : "Turn right") : b.do === "goto" ? "Go to" : b.do[0].toUpperCase() + b.do.slice(1)}</b>
      ${["forward", "back", "left", "right"].includes(b.do) && num(i, "m", b.m, 0.1, "m")}
      ${b.do === "turn" && num(i, "deg", b.deg, 15, "°")}
      ${b.do === "spin" && num(i, "turns", b.turns, 1, "turns")}
      ${b.do === "wait" && num(i, "s", b.s, 0.5, "s")}
      ${b.do === "repeat" && num(i, "times", b.times, 1, "times")}
      ${b.do === "goto" && html`<select value=${b.place} onChange=${(/** @type {any} */ e) => set(i, { place: e.currentTarget.value })}>
        <option value="">(pick a place)</option>${places.map((pl) => html`<option value=${pl}>${pl}</option>`)}</select>`}
      <button class="btn small" title="Up" onClick=${() => move(i, -1)}>↑</button>
      <button class="btn small" title="Down" onClick=${() => move(i, 1)}>↓</button>
      <button class="btn small danger" title="Remove" onClick=${() => edit(path, (/** @type {any[]} */ l) => l.filter((_, k) => k !== i))}>✕</button>
    </div>
    ${b.do === "repeat" && html`<div class="inside">
      <button class=${`btn small${JSON.stringify(into) === JSON.stringify([...path, i]) ? " primary" : ""}`}
        onClick=${() => setInto([...path, i])}>add inside</button>
      <${BlockList} list=${b.body} path=${[...path, i]} edit=${edit} places=${places} into=${into} setInto=${setInto} active=${active} /></div>`}
  </div>`)}</div>`;
}
