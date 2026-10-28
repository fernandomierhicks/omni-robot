// @ts-check
/* The page's link to the supervisor (omni_console/supervisor.py): the process that serves this page and starts and
   stops the robot's software. It answers plain HTTP on the page's own address, also while ROS is down, so the page
   is the launcher. A page served by console_web instead (a step started from the Launcher or a terminal) has no
   supervisor: store.supervisor stays false and the start / stop buttons are hidden. */
import { note, store, update } from "./ros.js";

let pin = "";
try { pin = sessionStorage.getItem("console.pin") || ""; } catch { /* private mode */ }

async function poll() {
  try {
    const r = await fetch("api/state", { cache: "no-store" });
    if (r.status === 404) { update({ supervisor: false }); return 10000; }
    const s = await r.json();
    update({ supervisor: s });
    const busy = s.current && s.current.state === "stopping";
    return busy ? 700 : 2000;
  } catch {
    update({ supervisor: store.supervisor ? { ...store.supervisor, unreachable: true } : false });
    return 3000;
  }
}

(async function loop() {
  for (;;) {
    const wait = await poll();
    await new Promise((res) => setTimeout(res, wait));
  }
})();

/** POST to the supervisor; asks for the PIN when it has one. Never rejects.
 *  @param {string} path @param {Record<string, any>} body @returns {Promise<{ok: boolean, message: string}>} */
export async function sysCall(path, body = {}) {
  for (let attempt = 0; attempt < 2; attempt++) {
    let res;
    try {
      const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ...body, pin }) });
      res = await r.json();
    } catch (e) {
      res = { ok: false, message: `The robot's page server didn't answer (${e})` };
    }
    if (res.need_pin && attempt === 0) {
      const p = prompt("PIN for the robot's software controls:");
      if (p === null) return { ok: false, message: "Cancelled" };
      pin = p;
      try { sessionStorage.setItem("console.pin", pin); } catch { /* */ }
      continue;
    }
    if (!res.ok) note("error", res.message || `${path} failed`);
    else if (res.message) note("info", res.message);
    setTimeout(poll, 300);
    return res;
  }
  return { ok: false, message: "Wrong PIN" };
}

/** Start a step (stopping the running one first). @param {string} step @param {Record<string, any>} [args] */
export const startStep = (step, args = {}) => sysCall("api/start", { step, args });
export const stopStep = () => sysCall("api/stop", {});

/** The step running now (or null), from the supervisor. */
export function runningStep() {
  const c = store.supervisor?.current;
  return c && (c.state === "running" || c.state === "stopping") ? c : null;
}

/** The arguments a step would start with: its own defaults, the supervisor's, then what this page chose last.
 *  @param {any} step */
export function stepArgs(step) {
  /** @type {Record<string, string>} */
  const out = {};
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem(`console.step.${step.id}`) || "{}"); } catch { /* */ }
  for (const a of step.args || []) {
    out[a.key] = /** @type {any} */ (saved)[a.key] ?? store.supervisor?.defaults?.[a.key] ?? a.default ?? "";
  }
  return out;
}

/** @param {string} id @param {Record<string, string>} args */
export function saveStepArgs(id, args) {
  try { localStorage.setItem(`console.step.${id}`, JSON.stringify(args)); } catch { /* */ }
}
