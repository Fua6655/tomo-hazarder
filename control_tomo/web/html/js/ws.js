import { STATE_MAP } from "./state.js";
import { updateState, addLog, getButton } from "./ui.js";

const ws = new WebSocket(`ws://${location.host}/ws`);

function sendCommand(target, name, value) {
  const payload = {
    type: "cmd",
    target,
    data: buildPayload(target, name, value)
  };
  ws.send(JSON.stringify(payload));
}

function buildPayload(target, name, value) {
  const maps = {
    states: ["ARMED", "POWER", "LIGHT"],
    events: ["ENGINE", "CLUTCH", "SPEED", "MOVE"],
    lights: ["FP", "FS", "FL", "BACK", "LB", "RB"],
  };

  return maps[target].map(k => k === name ? value : 0);
}

ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);

  if (msg.type === "init") {
    Object.entries(msg.states).forEach(([k, v]) => updateState(k, v));
    msg.logs.forEach(addLog);
  }

  if (msg.type === "state") updateState(msg.name, msg.value);
  if (msg.type === "log") addLog(msg.text);
  if (msg.type === "source") {
    activeSource = msg.value;
    updateSourceUI(activeSource);
  }
};
let activeSource = "web";

const indicator = document.getElementById("source-indicator");
const forceBtn = document.getElementById("force-web");

forceBtn.onclick = () => {
  ws.send(JSON.stringify({
    type: "force_web"
  }));
};
function updateSourceUI(source) {
  indicator.textContent = `ACTIVE: ${source.toUpperCase()}`;
  indicator.className = `source-${source}`;

  const disable = (source !== "web");
  document.body.classList.toggle("disabled", disable);
}

