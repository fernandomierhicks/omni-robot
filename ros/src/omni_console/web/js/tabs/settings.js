// @ts-check
/* Robot › Settings: every console setting in one place, grouped by the feature it belongs to (the same rows the
   feature pages show), and every robot node's ROS 2 parameters drawn from the node's own descriptions (omni_cpp
   params.hpp / omni_params: description, range, read-only or live; their robot values come from omni_bringup's
   robot.yaml). The parameters are read through each node's standard rcl_interfaces services over rosbridge, on
   demand: a new node or parameter appears here with no page change. A live parameter can be changed once edits are
   unlocked; the node checks its range and says no otherwise. A change lasts until the node restarts: put a value that
   should stay in robot.yaml. */
import { html, useEffect, useState } from "../../vendor/htm-preact-standalone.mjs";
import { features } from "../features.js";
import { getRos, note, store, useStore } from "../ros.js";
import { HelpBox, SettingsForm } from "../ui.js";

const ROSLIB = /** @type {any} */ (window).ROSLIB;
const TYPE = ["unset", "bool", "integer", "double", "string", "bytes", "bool[]", "integer[]", "double[]", "string[]"];
const FIELD = [null, "bool_value", "integer_value", "double_value", "string_value", "byte_array_value", "bool_array_value",
  "integer_array_value", "double_array_value", "string_array_value"];
const ROS_OWN = /^(use_sim_time|qos_overrides\.|start_type_description_service)/;

/** Call a node's parameter service. @param {string} node @param {string} srv @param {string} type @param {any} req
 *  @returns {Promise<any>} */
function call(node, srv, type, req) {
  return new Promise((resolve, reject) => {
    const ros = getRos();
    if (!ros || store.conn !== "open") { reject(new Error("not connected")); return; }
    const timer = setTimeout(() => reject(new Error("no answer")), 4000);
    new ROSLIB.Service({ ros, name: `${node}/${srv}`, serviceType: `rcl_interfaces/srv/${type}` }).callService(
      new ROSLIB.ServiceRequest(req), (/** @type {any} */ r) => { clearTimeout(timer); resolve(r); },
      (/** @type {any} */ e) => { clearTimeout(timer); reject(new Error(String(e))); });
  });
}

export function SettingsTab() {
  const s = useStore("schema", "conn");
  const list = features();
  const tabs = new Set((s.schema?.settings || []).flatMap((/** @type {any} */ x) => x.tabs));
  const groups = [...list.filter((x) => tabs.has(x.id)).map((x) => [x.id, x.label]),
    ...[...tabs].filter((t) => !list.some((x) => x.id === t)).map((t) => [t, t === "ai" ? "AI control" : t])];
  return html`<div class="cols">
    <div class="side">
      <${HelpBox} tab="settings">
        <b>Console settings</b>: the same rows as on each feature's page, all in one place. They are kept on the robot
        and apply at once. <b>ROS parameters</b> (right): each robot node's tunable values, as the node describes them
        (range, live or read-only). Their robot values live in <span class="mono">omni_bringup/config/robot.yaml</span>;
        a change here lasts until the node restarts.
      <//>
      ${!s.schema && html`<p class="dim">Settings appear when the robot answers.</p>`}
      ${groups.map(([id, label]) => html`<details key=${id} class="group" open=${id === "manual"}>
        <summary>${label}</summary><${SettingsForm} tab=${id} title="" /></details>`)}
    </div>
    <div class="main full"><${RosParams} /></div>
  </div>`;
}

function RosParams() {
  const s = useStore("conn");
  const [nodes, setNodes] = useState(/** @type {string[]} */ ([]));
  const [node, setNode] = useState("");
  const [rows, setRows] = useState(/** @type {any[] | null} */ (null));
  const [err, setErr] = useState("");
  const [filter, setFilter] = useState("");
  const [own, setOwn] = useState(false);
  const [edit, setEdit] = useState(false);
  useEffect(() => {
    const ros = getRos();
    if (!ros || s.conn !== "open") return;
    ros.getNodes((/** @type {string[]} */ n) => {
      const sorted = n.filter((x) => !x.includes("_ros2cli_") && !x.startsWith("/rosapi") && !x.startsWith("/rosbridge")).sort();
      setNodes(sorted);
      setNode((cur) => cur || sorted.find((x) => /mode_manager|mcu_link|omni/.test(x)) || sorted[0] || "");
    }, () => {});
  }, [s.conn]);
  const load = async (/** @type {string} */ n) => {
    setRows(null); setErr("");
    try {
      const names = (await call(n, "list_parameters", "ListParameters", { prefixes: [], depth: 0 })).result.names.sort();
      const [d, v] = await Promise.all([call(n, "describe_parameters", "DescribeParameters", { names }),
        call(n, "get_parameters", "GetParameters", { names })]);
      setRows(names.map((name, i) => ({ name, d: d.descriptors[i] || {}, v: v.values[i] || {} })));
    } catch (e) { setErr(`${n}: ${/** @type {Error} */ (e).message}`); setRows([]); }
  };
  useEffect(() => { if (node) load(node); }, [node, s.conn]);
  const shown = (rows || []).filter((r) => (own || !ROS_OWN.test(r.name)) && r.name.toLowerCase().includes(filter.toLowerCase()));
  return html`<div class="card">
    <h2>ROS parameters</h2>
    <div class="row" style="flex-wrap: wrap; gap: 8px">
      <select value=${node} onChange=${(/** @type {any} */ e) => setNode(e.currentTarget.value)}>
        ${nodes.map((n) => html`<option value=${n}>${n}</option>`)}</select>
      <input type="text" placeholder="filter" value=${filter} onInput=${(/** @type {any} */ e) => setFilter(e.currentTarget.value)} />
      <label class="row"><input type="checkbox" checked=${own} onChange=${(/** @type {any} */ e) => setOwn(e.currentTarget.checked)} /> ROS's own</label>
      <label class="row" title="Live parameters can be changed while the node runs; it checks the range"><input type="checkbox" checked=${edit}
        onChange=${(/** @type {any} */ e) => setEdit(e.currentTarget.checked)} /> allow edits</label>
      <button class="btn small" onClick=${() => node && load(node)}>Refresh</button>
    </div>
    ${err && html`<p class="err">${err}</p>`}
    ${rows === null ? html`<p class="dim">${s.conn === "open" ? "Reading..." : "Not connected to the robot."}</p>`
      : html`<table class="params">${shown.map((r) => html`<${Param} key=${r.name} node=${node} r=${r} edit=${edit} reload=${() => load(node)} />`)}</table>
        ${!shown.length && html`<p class="dim">No parameters${filter ? " match" : ""}.</p>`}`}
  </div>`;
}

/** @param {{node: string, r: any, edit: boolean, reload: () => void}} p */
function Param({ node, r, edit, reload }) {
  const t = r.v.type ?? r.d.type ?? 0;
  const field = FIELD[t];
  const val = field ? r.v[field] : null;
  const [draft, setDraft] = useState(/** @type {string | null} */ (null));
  const fr = r.d.floating_point_range?.[0], ir = r.d.integer_range?.[0];
  const range = fr ? `${fr.from_value} … ${fr.to_value}` : ir ? `${ir.from_value} … ${ir.to_value}` : "";
  const scalar = t >= 1 && t <= 4;
  const canEdit = edit && !r.d.read_only && scalar;
  const send = async () => {
    if (draft === null) return;
    const value = { type: t, [field || "string_value"]: t === 1 ? draft === "true" : t === 2 ? parseInt(draft, 10) : t === 3 ? parseFloat(draft) : draft };
    try {
      const res = await call(node, "set_parameters", "SetParameters", { parameters: [{ name: r.name, value }] });
      const ok = res.results?.[0]?.successful;
      note(ok ? "info" : "error", ok ? `${node} ${r.name} = ${draft}` : `${node} ${r.name}: ${res.results?.[0]?.reason || "refused"}`);
      setDraft(null);
      reload();
    } catch (e) { note("error", `${node} ${r.name}: ${/** @type {Error} */ (e).message}`); }
  };
  const shownVal = Array.isArray(val) ? `[${val.join(", ")}]` : String(val);
  return html`<tr>
    <td class="mono">${r.name}<div class="dim" style="font-size: 12px; font-family: var(--sans)">${r.d.description || ""}</div></td>
    <td class="mono">${canEdit
      ? html`<input type="text" style="width: 9em" value=${draft ?? shownVal} onInput=${(/** @type {any} */ e) => setDraft(e.currentTarget.value)}
          onKeyDown=${(/** @type {any} */ e) => { if (e.key === "Enter") send(); }} onBlur=${send} />`
      : shownVal}</td>
    <td class="dim" style="font-size: 12px">${TYPE[t] || t}${range ? ` · ${range}` : ""}</td>
    <td>${r.d.read_only ? html`<span class="badge">read-only</span>` : html`<span class="badge green">live</span>`}</td>
  </tr>`;
}
