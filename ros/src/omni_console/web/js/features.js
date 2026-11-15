// @ts-check
/* What the robot offers, as the robot says (omni_console/console_defs.py FEATURES, in /console/schema): each feature
   has an id (its page, location.hash), a label, a workspace (group), the caps it needs and whether it hides when it
   isn't available (optional: the parked catch robot, the sim's tools). The page draws what the robot lists: a payload
   swap or a new skill changes the bar with no page edit, and a feature the page has no page for gets the generic one
   (its card text, its settings and its start). Until the robot's software answers, the list comes from the
   supervisor (GET api/features, the same list): every page shows, and a page that needs the software offers to start
   it (app.js NeedsRobot); `server` pages work without it. A page served without a supervisor (console_web) has only
   FALLBACK until the robot answers. */
import { store, update, useStore } from "./ros.js";

/** @typedef {{id: string, label: string, group: string, needs: string[], optional: boolean, mission: string | null,
 *   card: string, start: {verb: string, args?: Record<string, any>, open?: boolean} | null, moves: boolean,
 *   hint: string, server?: boolean, base?: boolean}} Feature */

/** @param {string} id @param {string} label @param {string} group @param {string} [hint] @returns {Feature} */
const f = (id, label, group, hint = "") => ({ id, label, group, needs: [], optional: false, mission: null, card: "", start: null, moves: false, hint, server: true });

/** Without the supervisor's list either: the page server's own tools (the supervisor, camera_snap's cameras and lidar,
 * the session logs). */
export const FALLBACK = [
  f("home", "Overview", "home"), f("cameras", "Cameras", "see"), f("lidar", "Lidar", "test"), f("logbook", "Logbook", "robot"),
  f("system", "System", "robot"), f("version", "Version", "robot"), f("ros", "ROS", "robot"),
];
export const WORKSPACES = [["home", "Home"], ["map", "Map"], ["payload", "Payload"], ["see", "See"], ["test", "Test"],
  ["robot", "Robot"]];

/* A cap's meaning when the robot doesn't send "avail" (an older robot): checks.py NEED_TEXT is the robot's own */
const NEED_TEXT = /** @type {Record<string, string>} */ ({
  nav: "Needs navigation (Nav2 and SLAM aren't running)",
  arm: "Needs the arm (arm_node isn't running)",
  catch: "Needs the catch code (the parked catch robot)",
  sim: "The sim only",
  collision: "Needs Nav2's collision monitor",
});

const SERVER_IDS = new Set(FALLBACK.map((x) => x.id));
export const NO_SOFTWARE = "Needs the robot's software: start it (Robot › System)";

/** The supervisor's copy of the list, fetched once (retried while the server is down; a 404: no supervisor). */
(async function fetchList() {
  for (let wait = 2000; ; wait = Math.min(wait * 2, 30000)) {
    try {
      const r = await fetch("api/features", { cache: "no-store" });
      if (r.status === 404) return;
      const d = await r.json();
      if (Array.isArray(d.features) && d.features.length) { update({ serverFeatures: d.features }); return; }
    } catch { /* the server is down: try again */ }
    await new Promise((res) => setTimeout(res, wait));
  }
})();

/** @returns {Feature[]} */
export function features() {
  const s = useStore("schema", "serverFeatures");
  const list = s.schema?.features;
  if (Array.isArray(list) && list.length) return list;
  const sv = /** @type {any} */ (store).serverFeatures;
  return Array.isArray(sv) && sv.length ? sv : FALLBACK;
}

/** Does it work without the robot's software (the page server's own tools)? @param {Feature} feat */
export const isServer = (feat) => !!feat.server || SERVER_IDS.has(feat.id);

/** Can it run now? {ok, why}. The robot says (status "avail": checks.unavailable, from its caps and its preflight:
 *  missing drive motors, a silent IMU, no lidar scan ...); without a status only the page server's own tools can.
 *  @param {Feature} feat @param {any} st the status */
export function availability(feat, st) {
  if (!st && !isServer(feat)) return { ok: false, why: NO_SOFTWARE };
  if (st?.avail) {
    const why = st.avail[feat.id];
    return why ? { ok: false, why } : { ok: true, why: "" };
  }
  if (!feat.needs.length) return { ok: true, why: "" };
  if (!st) return { ok: false, why: NO_SOFTWARE };
  const caps = st.caps || {};
  const miss = feat.needs.filter((n) => !caps[n]);
  return miss.length ? { ok: false, why: miss.map((n) => NEED_TEXT[n] || `Needs ${n}`).join("; ") } : { ok: true, why: "" };
}

/** Shown in the bar: everything except an optional feature that can't run now.
 *  @param {Feature} feat @param {any} st */
export const visible = (feat, st) => !feat.optional || availability(feat, st).ok;

/** The payload workspace's name: the fitted payload's (Arm on the arm robot, Catch on the catch robot).
 *  @param {Feature[]} list @param {any} st */
export function payloadLabel(list, st) {
  const on = list.filter((x) => x.group === "payload" && visible(x, st) && availability(x, st).ok).map((x) => x.needs).flat();
  const arm = on.includes("arm"), cat = on.includes("catch");
  return arm && cat ? "Payload" : cat ? "Catch" : "Arm";
}

/** Can this page's start run now? Every page's start button uses it, so a part that doesn't work greys the same
 *  buttons everywhere, with the robot's reason. @param {string} id the feature @returns {{ok: boolean, why: string}} */
export function useGate(id) {
  const s = useStore("status");
  const feat = features().find((x) => x.id === id);
  if (!s.status) return { ok: false, why: NO_SOFTWARE };
  return feat ? availability(feat, s.status) : { ok: true, why: "" };
}

/** Run a feature's start verb (its Home card); open: then show its page (Drive). No start: open the page.
 *  @param {Feature} feat @param {(v: string, a: any) => any} act */
export async function start(feat, act) {
  if (!feat.start) { location.hash = feat.id; return null; }
  const r = await act(feat.start.verb, feat.start.args || {});
  if (feat.start.open && r?.ok !== false) location.hash = feat.id;
  return r;
}
