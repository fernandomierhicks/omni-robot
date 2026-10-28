// @ts-check
/* Manual driving from the page: a gamepad, the keyboard or the on-screen pads. Whatever holds its dead man drives;
   the page sends -1..1 per axis at 20 Hz and the robot scales it by the Manual tab's settings. Letting go stops
   sending: the robot stops by itself 0.3 s later (and the MCU 0.5 s after that if the robot's side died).

   Gamepads with the browser's "standard" mapping (Xbox, PlayStation, most pads): left stick moves, right stick turns,
   hold LB or RB to drive, RT = boost. On every tab, not just Manual: a fresh press of the dead man takes control from
   any mission (see useDriveLoop). Anything else (the Logitech Extreme 3D, other flight sticks) uses the axes and
   buttons picked in the Manual tab; the defaults follow ros/tools/joystick_sender.py: thumb button drives, trigger
   boosts.

   Every connected controller is read, not just the browser's first: a PC often lists more than the one in hand (an
   Xbox pad, a flight stick and the 3Dconnexion driver's "KMJ Emulator" on Fernando's), and whichever holds its dead
   man drives. Virtual ones (SpaceMouse, vJoy) are off by default; the Manual tab switches each one on or off. */
import { html, useEffect, useRef, useState } from "../vendor/htm-preact-standalone.mjs";
import { act, sendTwist } from "./ros.js";

const DEADZONE = 0.08;
const KEYS = { w: [1, 0, 0], s: [-1, 0, 0], a: [0, 1, 0], d: [0, -1, 0], q: [0, 0, 1], e: [0, 0, -1],
  arrowup: [1, 0, 0], arrowdown: [-1, 0, 0], arrowleft: [0, 0, 1], arrowright: [0, 0, -1] };

/** @typedef {{vx: number, vy: number, wz: number, boost: boolean, source: string}} Cmd */

export const input = {
  keys: new Set(),
  shift: false,
  pad: /** @type {Cmd | null} */ (null),       // the on-screen pads
  last: /** @type {Cmd | null} */ (null),
  /** every connected controller, as the Manual tab shows it */
  gamepads: /** @type {{index: number, id: string, standard: boolean, use: boolean, driving: boolean,
    axes: number[], buttons: number[]}[]} */ ([]),
};

/** Flight-stick mapping, per controller name, kept in this browser. Chrome and Edge on Windows put an axis at its HID
 *  usage (the Extreme 3D's twist, Rz, at 5); Firefox packs them (twist third), so a short axis list gets twist 2.
 *  @param {string} id @param {number} [nAxes] */
export function stickMap(id, nAxes = 10) {
  const d = { x: 0, y: 1, twist: nAxes > 5 ? 5 : 2, deadman: 1, boost: 0 };
  try { return { ...d, ...JSON.parse(localStorage.getItem(`console.stick.${id}`) || "{}") }; } catch { return d; }
}
export function saveStickMap(/** @type {string} */ id, /** @type {any} */ m) {
  try { localStorage.setItem(`console.stick.${id}`, JSON.stringify(m)); } catch { /* */ }
}

/* Virtual controllers that aren't for driving: the 3Dconnexion driver's joystick emulation, vJoy. */
const VIRTUAL = /3dconnexion|kmj emulator|spacemouse|spacenavigator|vjoy/i;
/** Whether a controller drives (the Manual tab's checkbox), kept in this browser. @param {string} id */
export function padUsed(id) {
  try { const v = localStorage.getItem(`console.pad.use.${id}`); if (v !== null) return v === "1"; } catch { /* */ }
  return !VIRTUAL.test(id);
}
export function setPadUsed(/** @type {string} */ id, /** @type {boolean} */ on) {
  try { localStorage.setItem(`console.pad.use.${id}`, on ? "1" : "0"); } catch { /* */ }
}

/** @param {number} u */
const dz = (u) => (Math.abs(u) < DEADZONE ? 0 : (u - Math.sign(u) * DEADZONE) / (1 - DEADZONE));

/** One controller's command while its dead man is held, else null. @param {Gamepad} g @returns {Cmd | null} */
function padCommand(g) {
  const pressed = (/** @type {number} */ i) => !!g.buttons[i] && (g.buttons[i].pressed || g.buttons[i].value > 0.5);
  if (g.mapping === "standard") {
    if (!(pressed(4) || pressed(5))) return null;
    return { vx: -dz(g.axes[1] ?? 0), vy: -dz(g.axes[0] ?? 0), wz: -dz(g.axes[2] ?? 0), boost: pressed(7),
      source: "gamepad" };
  }
  const m = stickMap(g.id, g.axes.length);
  if (!pressed(m.deadman)) return null;
  return { vx: -dz(g.axes[m.y] ?? 0), vy: -dz(g.axes[m.x] ?? 0), wz: -dz(g.axes[m.twist] ?? 0), boost: pressed(m.boost),
    source: "flight stick" };
}

/** Every connected controller; the first one in use whose dead man is held drives. @returns {Cmd | null} */
function readGamepads() {
  let pads = /** @type {(Gamepad | null)[]} */ ([]);
  try { pads = navigator.getGamepads ? [...navigator.getGamepads()] : []; } catch { /* blocked by the page's policy */ }
  let cmd = /** @type {Cmd | null} */ (null);
  input.gamepads = [];
  for (const g of pads) {
    if (!g || !g.connected) continue;
    const use = padUsed(g.id);
    const c = use && !cmd ? padCommand(g) : null;
    if (c) cmd = c;
    input.gamepads.push({ index: g.index, id: g.id, standard: g.mapping === "standard", use, driving: !!c,
      axes: g.axes.map((a) => Math.round(a * 100) / 100), buttons: g.buttons.map((b) => (b.pressed ? 1 : 0)) });
  }
  return cmd;
}

/** @returns {Cmd | null} */
function readKeys() {
  if (!input.keys.size) return null;
  let vx = 0, vy = 0, wz = 0;
  for (const k of input.keys) { const v = /** @type {any} */ (KEYS)[k]; if (v) { vx += v[0]; vy += v[1]; wz += v[2]; } }
  return { vx: Math.max(-1, Math.min(1, vx)), vy: Math.max(-1, Math.min(1, vy)), wz: Math.max(-1, Math.min(1, wz)),
    boost: input.shift, source: "keyboard" };
}

/** Runs for the whole page (App): reads the inputs and sends at 20 Hz. The gamepad / flight stick works on every tab:
 *  pressing its dead man in any other mission takes control first (the mission becomes MANUAL, which ends a Go To,
 *  Explore, Patrol... as "cancel" does). The keyboard and the on-screen pads only count on the Manual tab.
 *  @param {string | undefined} mission the robot's mission (status) @param {boolean} onManualTab */
export function useDriveLoop(mission, onManualTab) {
  const live = useRef({ mission, onManualTab });
  live.current = { mission, onManualTab };
  useEffect(() => {
    /** @param {KeyboardEvent} e */
    const kd = (e) => {
      const t = /** @type {HTMLElement} */ (e.target);
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT")) return;
      const k = e.key.toLowerCase();
      input.shift = e.shiftKey;
      if (k in KEYS && live.current.onManualTab) { input.keys.add(k); e.preventDefault(); }
    };
    /** @param {KeyboardEvent} e */
    const ku = (e) => { input.keys.delete(e.key.toLowerCase()); input.shift = e.shiftKey; };
    const blur = () => { input.keys.clear(); input.shift = false; };
    window.addEventListener("keydown", kd);
    window.addEventListener("keyup", ku);
    window.addEventListener("blur", blur);
    let zeros = 0;
    let held = false;                     // the stick's dead man was down on the last tick
    const timer = setInterval(() => {
      const { mission: m, onManualTab: onTab } = live.current;
      const stick = readGamepads();
      const cmd = stick || (onTab ? input.pad || readKeys() : null);
      input.last = cmd;
      const manual = m === "MANUAL";
      if (stick && !held && m && !manual) {         // a fresh press of the dead man: take control from any mission
        act("mission", { name: "MANUAL" }, true);
      }
      held = !!stick;
      if (manual && cmd) {
        sendTwist(cmd.vx, cmd.vy, cmd.wz, cmd.boost);
        zeros = 3;
      } else if (zeros > 0) {             // let go: say so at once rather than waiting for the robot's timeout
        sendTwist(0, 0, 0, false);
        zeros--;
      }
    }, 50);
    return () => {
      clearInterval(timer);
      window.removeEventListener("keydown", kd); window.removeEventListener("keyup", ku);
      window.removeEventListener("blur", blur);
      input.keys.clear(); input.pad = null;
      sendTwist(0, 0, 0, false);
    };
  }, []);
  return input;
}

/** Re-render the calling component 20 times a second (the Manual tab's live input bars). */
export function useInputTick() {
  const [, bump] = useState(0);
  useEffect(() => {
    const t = setInterval(() => bump((n) => (n + 1) % 1000), 50);
    return () => clearInterval(t);
  }, []);
}

/** The on-screen pads: a round one to move (drag the knob), a long one to turn. Touching either is the dead man. */
export function Pads() {
  const move = useRef({ vx: 0, vy: 0, on: false });
  const turn = useRef({ wz: 0, on: false });
  const [, bump] = useState(0);
  const sync = () => {
    const on = move.current.on || turn.current.on;
    input.pad = on ? { vx: move.current.vx, vy: move.current.vy, wz: turn.current.wz, boost: false, source: "screen" } : null;
    bump((n) => n + 1);
  };
  /** @param {any} e @param {"move" | "turn"} which */
  const handle = (e, which) => {
    const el = /** @type {HTMLElement} */ (e.currentTarget);
    if (e.type === "pointerdown") el.setPointerCapture(e.pointerId);
    const r = el.getBoundingClientRect();
    const end = e.type === "pointerup" || e.type === "pointercancel";
    if (which === "move") {
      if (end) { move.current = { vx: 0, vy: 0, on: false }; }
      else if (e.type === "pointerdown" || move.current.on) {
        const R = r.width / 2 - 27;
        let dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2);
        const n = Math.hypot(dx, dy);
        if (n > R) { dx *= R / n; dy *= R / n; }
        move.current = { vx: -dy / R, vy: -dx / R, on: true };
      }
    } else {
      if (end) { turn.current = { wz: 0, on: false }; }
      else if (e.type === "pointerdown" || turn.current.on) {
        const R = r.width / 2 - 25;
        const dx = Math.max(-R, Math.min(R, e.clientX - (r.left + r.width / 2)));
        turn.current = { wz: -dx / R, on: true };
      }
    }
    sync();
  };
  const m = move.current, t = turn.current;
  const ev = (/** @type {"move" | "turn"} */ w) => ({
    onPointerDown: (/** @type {any} */ e) => handle(e, w), onPointerMove: (/** @type {any} */ e) => handle(e, w),
    onPointerUp: (/** @type {any} */ e) => handle(e, w), onPointerCancel: (/** @type {any} */ e) => handle(e, w) });
  return html`
    <div class=${m.on ? "pad move active" : "pad move"} ...${ev("move")}>
      <div class="knob" style=${`left: calc(50% - ${m.vy * 48}px); top: calc(50% - ${m.vx * 48}px)`}></div>
      <div class="lbl">drag to move</div></div>
    <div class=${t.on ? "pad turn active" : "pad turn"} ...${ev("turn")}>
      <div class="knob" style=${`left: calc(50% - ${t.wz * 70}px)`}></div>
      <div class="lbl">drag to turn</div></div>`;
}
