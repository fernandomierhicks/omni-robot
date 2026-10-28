// @ts-check
import { html, useRef } from "../../vendor/htm-preact-standalone.mjs";
import { decodeFloat32MultiArray } from "../cdr.js";
import { Chart, makeBuffer, push } from "../chart.js";
import { useRawTopic, useStore } from "../ros.js";
import { HelpBox } from "../ui.js";

const RAD = 180 / Math.PI;

// firmware/lib/omni_fw/dm_motor.h DmErr: the DM-2325's state byte
const DM = {
  0: ["off", "Disabled (motors off: IDLE, or not yet enabled)"], 1: ["ok", "Enabled"],
  8: ["over-voltage", "Supply too high"], 9: ["under-voltage", "Battery too low under load"],
  10: ["over-current", "Drew too much current: stall or short"], 11: ["driver hot", "Its MOSFETs overheated"],
  12: ["coil hot", "Its winding overheated: let it cool"], 13: ["no link", "It lost the CAN messages"],
  14: ["overload", "Overloaded for too long"],
};
const DRIVE = ["idle", "velocity (driving)", "go-to (catch move)", "coast / brake after a catch"];

export function BaseTab() {
  const s = useStore("status");
  const st = s.status;
  const m = st?.mcu;
  const b = useRef({ v: makeBuffer(4), w: makeBuffer(2), slip: makeBuffer(1), tilt: makeBuffer(2), t0: 0 }).current;
  // mcu_link's 200 Hz samples, straight from the robot (omni_mcu link_node SAMPLE_COLS): the command comes in the
  // odom frame and is turned into the body frame here
  useRawTopic("/mcu/samples", "std_msgs/msg/Float32MultiArray", (bytes) => {
    const { dims, data } = decodeFloat32MultiArray(bytes);
    const cols = (dims[1]?.label || "").split(",");
    const n = cols.length;
    const k = Object.fromEntries(cols.map((c, i) => [c, i]));
    for (let o = 0; o + n <= data.length; o += n) {
      const t = data[o + k.t], yaw = data[o + k.yaw];
      const c = Math.cos(yaw), s = Math.sin(yaw), ox = data[o + k.v_cmd_x], oy = data[o + k.v_cmd_y];
      push(b.v, t, [data[o + k.vx], c * ox + s * oy, data[o + k.vy], -s * ox + c * oy]);
      push(b.w, t, [data[o + k.wz], data[o + k.w_cmd]]);
      push(b.slip, t, [data[o + k.slip]]);
      push(b.tilt, t, [data[o + k.roll] * RAD, data[o + k.pitch] * RAD]);
    }
  });
  return html`<div class="full">
    <${HelpBox} tab="base">
      The drive: four DM-2325 motors (integrated FOC and encoder) commanded over CAN by the MCU, which turns the
      Jetson's speed requests into wheel speeds within its limits. The charts show the last 10 s at the MCU's full
      200 Hz: <b>solid</b> what the wheels measured, <b>dashed</b> what the controller asked for. A gap between them while
      driving is acceleration limiting or slip; <b>slip</b> is how much the four wheels disagree. A wheel that isn't
      "ok" while armed stops the robot (MOTOR_ERR in the status bar).
    <//>
    <div class="row" style="align-items: stretch; gap: 10px">
      <div class="card" style="min-width: 280px">
        <h2>Wheels</h2>
        ${m ? html`<table class="t"><tbody>${m.dm_err.map((/** @type {number} */ e, /** @type {number} */ i) => {
          const d = DM[/** @type {keyof typeof DM} */ (e)] || [`error ${e}`, "Unknown code (dm_motor.h)"];
          const cls = e === 1 ? "ok" : e === 0 ? "dim" : "err";
          return html`<tr><td>Wheel ${i + 1}</td><td class=${cls} title=${d[1]}>${d[0]}</td></tr>`;
        })}</tbody></table>` : html`<p class="dim">No MCU state yet.</p>`}
      </div>
      <div class="card grow">
        <h2>Body</h2>
        ${m ? html`<div class="kv">
          <span class="k">speed (fwd, left)</span><span class="mono">${m.vx?.toFixed(2)}, ${m.vy?.toFixed(2)} m/s</span>
          <span class="k">turn rate</span><span class="mono">${((m.wz ?? 0) * 57.3).toFixed(0)} °/s</span>
          <span class="k">slip</span><span class="mono">${m.slip?.toFixed(3)} m/s</span>
          <span class="k">roll, pitch</span><span class="mono">${((m.roll ?? 0) * 57.3).toFixed(1)}°, ${((m.pitch ?? 0) * 57.3).toFixed(1)}°</span>
          <span class="k">doing</span><span>${DRIVE[m.drive_mode] ?? m.drive_mode}</span>
          <span class="k">profile</span><span>${m.profile} (${m.profile === "kid" ? "0.8 m/s, gentle" : "2 m/s"})</span>
          <span class="k">pose (${st.pose?.frame || "?"})</span><span class="mono">${st.pose ? `${st.pose.x.toFixed(2)}, ${st.pose.y.toFixed(2)} m, ${(st.pose.yaw * 57.3).toFixed(0)}°` : "–"}</span>
          <span class="k">link round trip</span><span class="mono">${m.sync_rtt_ms != null ? `${m.sync_rtt_ms.toFixed(2)} ms` : "–"}</span>
        </div>` : html`<p class="dim">No MCU state yet.</p>`}
      </div>
    </div>
    <div class="charts">
      <div class="card"><${Chart} title="Speed" unit="m/s" span=${0.5} buf=${b.v} series=${[
        { label: "forward", color: "blue" }, { label: "forward asked", color: "blue", dash: true },
        { label: "sideways", color: "green" }, { label: "sideways asked", color: "green", dash: true }]} /></div>
      <div class="card"><${Chart} title="Turn rate" unit="rad/s" span=${0.5} buf=${b.w} series=${[
        { label: "turn", color: "orange" }, { label: "turn asked", color: "orange", dash: true }]} /></div>
      <div class="card"><${Chart} title="Wheel slip" unit="m/s" span=${0.05} buf=${b.slip} series=${[{ label: "slip", color: "red" }]} /></div>
      <div class="card"><${Chart} title="Body tilt" unit="deg" span=${2} buf=${b.tilt} series=${[
        { label: "roll", color: "yellow" }, { label: "pitch", color: "blue" }]} /></div>
    </div>
  </div>`;
}
