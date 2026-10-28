// @ts-check
// The Say box: type a line and the robot says it (omni_face: its voice, its face showing the expression while it
// speaks). On the Manual tab while driving, and on the Face tab with the tricks. Enter sends; on the Manual tab
// Enter also jumps into the box from driving (the keyboard's driving keys are ignored while typing) and Esc goes back.
import { html, useEffect, useRef, useState } from "../vendor/htm-preact-standalone.mjs";
import { act } from "./ros.js";

export const EXPRESSIONS = [["auto", "Plain"], ["happy", "Happy"], ["excited", "Excited"], ["silly", "Silly"],
  ["love", "Love"], ["wink", "Wink"], ["thinking", "Thinking"], ["sad", "Sad"], ["worried", "Worried"],
  ["sleepy", "Sleepy"]];
const QUICK = ["Hello!", "Excuse me, coming through!", "Beep beep!", "Watch out!", "Thank you!", "Ready to play?",
  "Good job!", "Bye bye!"];
const HISTORY_KEY = "console.say.history";

function loadHistory() {
  try { return JSON.parse(localStorage.getItem(HISTORY_KEY) || "[]").slice(0, 8); } catch { return []; }
}

/** @param {{hotkey?: boolean, compact?: boolean}} p */
export function SayBox({ hotkey = false, compact = false }) {
  const [text, setText] = useState("");
  const [expr, setExpr] = useState("auto");
  const [history, setHistory] = useState(loadHistory);
  const box = useRef(/** @type {HTMLInputElement | null} */ (null));
  /** @param {string} t */
  const send = async (t) => {
    const line = t.trim();
    if (!line) return;
    const r = await act("say", { text: line, expression: expr }, true);
    if (!r.ok) return;
    setText("");
    const h = [line, ...history.filter((x) => x !== line)].slice(0, 8);
    setHistory(h);
    try { localStorage.setItem(HISTORY_KEY, JSON.stringify(h)); } catch { /* */ }
  };
  useEffect(() => {
    if (!hotkey) return undefined;
    /** @param {KeyboardEvent} e */
    const k = (e) => {
      const el = /** @type {HTMLElement} */ (e.target);
      const typing = el && ["INPUT", "TEXTAREA", "SELECT"].includes(el.tagName);
      if (e.key === "Enter" && !typing && box.current) { e.preventDefault(); box.current.focus(); }
    };
    window.addEventListener("keydown", k);
    return () => window.removeEventListener("keydown", k);
  }, [hotkey]);
  return html`<div class="card say">
    <div class="row">
      <input type="text" class="grow" maxlength="200" placeholder=${hotkey ? "Say something (Enter)" : "Say something"}
        value=${text} ref=${box}
        onInput=${(/** @type {any} */ e) => setText(e.currentTarget.value)}
        onKeyDown=${(/** @type {any} */ e) => {
          if (e.key === "Enter") { e.preventDefault(); send(text); }
          if (e.key === "Escape") { e.currentTarget.blur(); }
        }} />
      <button class="btn primary" disabled=${!text.trim()} onClick=${() => send(text)}>Say</button>
    </div>
    <div class="row" style="margin-top: 6px">
      <span class="dim" style="font-size: 12px">Face</span>
      <select value=${expr} onChange=${(/** @type {any} */ e) => setExpr(e.currentTarget.value)}>
        ${EXPRESSIONS.map(([v, l]) => html`<option value=${v}>${l}</option>`)}</select>
    </div>
    <div class="row" style="margin-top: 6px; gap: 4px">
      ${(compact ? QUICK.slice(0, 4) : QUICK).map((q) => html`<button class="btn small" onClick=${() => send(q)}>${q}</button>`)}
    </div>
    ${history.length > 0 && html`<div class="row" style="margin-top: 6px; gap: 4px">
      <span class="dim" style="font-size: 12px">Again:</span>
      ${history.slice(0, compact ? 3 : 8).map((h) => html`<button class="btn small" title="Say it again"
        onClick=${() => send(h)}>${h.length > 28 ? h.slice(0, 26) + "…" : h}</button>`)}</div>`}
  </div>`;
}
