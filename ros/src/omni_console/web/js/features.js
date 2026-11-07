// @ts-check
/* What the robot offers, as the robot says (omni_console/console_defs.py FEATURES, in /console/schema): each feature
   has an id (its page, location.hash), a label, a workspace (group), the caps it needs and whether it hides when it
   isn't available (optional: the parked catch robot, the sim's tools). The page draws what the robot lists: a payload
   swap or a new skill changes the bar with no page edit, and a feature the page has no page for gets the generic one
   (its card text, its settings and its start). Until the robot answers, the page offers only what works without it
   (FALLBACK: its own server's tools). */
import { useStore } from "./ros.js";

/** @typedef {{id: string, label: string, group: string, needs: string[], optional: boolean, mission: string | null,
 *   card: string, start: {verb: string, args?: Record<string, any>} | null, moves: boolean, hint: string}} Feature */

/** @param {string} id @param {string} label @param {string} group @param {string} [hint] @returns {Feature} */
const f = (id, label, group, hint = "") => ({ id, label, group, needs: [], optional: false, mission: null, card: "", start: null, moves: false, hint });

/** Before the robot answers: the page server's own tools (the supervisor, camera_snap, the session logs). */
export const FALLBACK = [
  f("home", "Overview", "home"), f("cameras", "Cameras", "see"), f("logbook", "Logbook", "robot"),
  f("system", "System", "robot"), f("version", "Version", "robot"), f("ros", "ROS", "robot"),
];
export const WORKSPACES = [["home", "Home"], ["drive", "Drive"], ["map", "Map"], ["payload", "Payload"], ["see", "See"],
  ["test", "Test"], ["robot", "Robot"]];

const NEED_TEXT = /** @type {Record<string, string>} */ ({
  nav: "Needs navigation: start 'Robot (all modes)' or 'Navigate (Nav2)'",
  arm: "Needs the arm robot (omni_arm's arm_node)",
  catch: "Needs the catch code (the parked catch robot)",
  sim: "The sim only",
  collision: "Needs Nav2's collision monitor",
});

/** @returns {Feature[]} */
export function features() {
  const s = useStore("schema");
  const list = s.schema?.features;
  return Array.isArray(list) && list.length ? list : FALLBACK;
}

/** Can it run now? {ok, why}. Without a status nothing can.
 *  @param {Feature} feat @param {any} st the status */
export function availability(feat, st) {
  if (!feat.needs.length) return { ok: true, why: "" };
  if (!st) return { ok: false, why: "Start the robot first" };
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

/** Run a feature's start verb (its Home card). @param {Feature} feat @param {(v: string, a: any) => any} act */
export function start(feat, act) {
  if (feat.start) return act(feat.start.verb, feat.start.args || {});
  location.hash = feat.id;
  return null;
}
