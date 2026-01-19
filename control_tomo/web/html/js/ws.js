// ws.js
import { updateState, initTopRow } from "./ui.js";

// --------------------------------------------------
// DEFINE GLOBAL API FIRST ❗
// --------------------------------------------------
window.sendCommand = function (target, name, value) {
  if (target === "web_ctrl") {
    ws.send(JSON.stringify({ type: "force", value }));
    return;
  }

  if (target === "emergency") {
  ws.send(JSON.stringify({ type: "emergency", value }));
  return;
  }

  ws.send(JSON.stringify({
    type: "cmd",
    target,
    data: buildPayload(target, name, value),
  }));
};

// --------------------------------------------------
// WEBSOCKET INIT
// --------------------------------------------------
const ws = new WebSocket(
  (location.protocol === "https:" ? "wss://" : "ws://") +
  location.host + "/ws"
);

ws.onopen = () => console.log("[WS] connected");

// --------------------------------------------------
// INIT UI AFTER sendCommand EXISTS ✅
// --------------------------------------------------
initTopRow();

// --------------------------------------------------
// PAYLOAD BUILDER
// --------------------------------------------------
function buildPayload(target, name, value) {
  const maps = {
    states: ["ARMED", "POWER", "LIGHT"],
    events: ["ENGINE", "CLUTCH", "SPEED", "MOVE"],
    lights: ["FP", "FS", "FL", "BACK", "LB", "RB"],
  };
  return maps[target]?.map(k => (k === name ? value : 0)) ?? [];
}

// --------------------------------------------------
// INCOMING MESSAGES
// --------------------------------------------------
ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);

  if (msg.type === "init") {
    Object.entries(msg.states).forEach(([k, v]) =>
      updateState(k, v)
    );
  }

  if (msg.type === "state") {
    updateState(msg.name, msg.value);
  }

  if (msg.type === "source") {
    updateState("WEB_CTRL", msg.value === "WEB" ? "1" : "0");
  }
};
