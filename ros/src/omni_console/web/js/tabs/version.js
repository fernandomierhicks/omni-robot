// @ts-check
import { html } from "../../vendor/htm-preact-standalone.mjs";
import { PAGE_VERSION, useStore } from "../ros.js";
import { HelpBox } from "../ui.js";

/** Problems worth a banner on every tab: the firmware or the protocol not matching the software. @param {any} v */
export function mismatch(v) {
  const f = v?.firmware;
  if (!f) return null;
  if (f.protocol != null && f.protocol !== f.protocol_host) {
    return `The MCU speaks protocol ${f.protocol}, the Jetson ${f.protocol_host}: build and flash the firmware from this checkout.`;
  }
  if (f.running && f.expected && f.running !== f.expected) {
    return `The MCU runs firmware ${f.running}, this software is built for ${f.expected}: rebuild (and, on the robot, flash) the firmware.`;
  }
  return null;
}

export function VersionTab() {
  const s = useStore("status", "url", "version");
  const st = s.status;
  const v = s.version;
  const f = v?.firmware;
  const bad = mismatch(v);
  const sw = v?.software;
  return html`<div class="full">
    <${HelpBox} tab="version">
      What runs where. The firmware's build id is a hash of its source (<span class="mono">firmware/fw_build_id.py</span>):
      the MCU reports the one it was built from, and the Jetson computes the one its own copy of the source gives. If they
      differ, the MCU runs other firmware than this software expects (a red banner shows on every tab).
    <//>
    <div class="row" style="align-items: flex-start; gap: 10px">
      <div class="card grow" style="min-width: 320px">
        <h2>Firmware (MCU)</h2>
        ${f ? html`<div class="kv">
          <span class="k">running</span><span class="mono">${f.running || "– (no INFO yet)"}</span>
          <span class="k">expected</span><span class="mono">${f.expected || "– (no firmware source here)"}</span>
          <span class="k">protocol</span><span class="mono">MCU ${f.protocol ?? "–"}, Jetson ${f.protocol_host}</span>
          <span class="k">match</span><span class=${bad ? "err" : f.running ? "ok" : "dim"}>${bad ? "NO" : f.running ? "yes" : "–"}</span>
        </div>${bad && html`<p class="err">${bad}</p>`}` : html`<p class="dim">Waiting for the robot...</p>`}
      </div>
      <div class="card grow" style="min-width: 320px">
        <h2>Software (Jetson)</h2>
        ${v ? html`<div class="kv">
          <span class="k">commit</span><span class="mono">${sw?.commit || "–"}</span>
          <span class="k">committed</span><span class="mono">${sw?.commit_time || "–"}</span>
          <span class="k">synced / built</span><span class="mono">${sw?.synced || "–"}</span>
          <span class="k">console</span><span class="mono">robot ${v.console}, this page ${PAGE_VERSION}</span>
          <span class="k">ROS</span><span class="mono">${v.ros || "–"}</span>
          <span class="k">computer</span><span class="mono">${v.host}</span>
          <span class="k">system</span><span class="mono" style="font-size: 12px">${v.os}, Python ${v.python}</span>
          <span class="k">load, temperature</span><span class="mono">${st?.sys ? `${st.sys.load ?? "–"} on ${st.sys.cpus} cores, ${st.sys.temp_c != null ? st.sys.temp_c + " °C" : "no sensor"}` : "–"}</span>
          ${v.slam_log && html`<span class="k">slam_toolbox log</span><span class="mono" style="font-size: 12px">${v.slam_log}</span>`}
        </div>` : html`<p class="dim">Waiting for the robot...</p>`}
      </div>
      <div class="card" style="min-width: 240px">
        <h2>Packages</h2>
        ${v ? html`<div class="kv">${Object.entries(v.packages || {}).map(([k, val]) => html`<span class="k">${k}</span>
          <span class="mono">${val || "missing"}</span>`)}</div>` : ""}
        <h2 style="margin-top: 12px">This page</h2>
        <div class="kv"><span class="k">rosbridge</span><span class="mono">${s.url}</span>
          <span class="k">browser</span><span class="mono" style="font-size: 11px">${navigator.userAgent}</span></div>
      </div>
    </div>
  </div>`;
}
