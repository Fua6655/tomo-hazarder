import { initUI, updateState, updateSource } from "./ui.js";

// --------------------------------------------------
// WEBSOCKET
// --------------------------------------------------
const ws = new WebSocket(
  (location.protocol === "https:" ? "wss://" : "ws://") +
  location.host + "/ws"
);

ws.onopen = () => {
  console.log("[WS] connected");
  initUI();
};

// --------------------------------------------------
// GLOBAL COMMAND API
// --------------------------------------------------
window.sendCmd = function (target, name, value) {
  ws.send(JSON.stringify({
    type: "cmd",
    target,
    data: buildPayload(target, name, value),
  }));
};

window.sendForce = function (web, auto) {
  ws.send(JSON.stringify({
    type: "force",
    web,
    auto,
  }));
};

window.sendEmergency = function (value) {
  ws.send(JSON.stringify({
    type: "emergency",
    value,
  }));
};

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
    Object.entries(msg.states || {}).forEach(([k, v]) =>
      updateState(k, v)
    );
  }

  if (msg.type === "state") {
    updateState(msg.name, msg.value);
  }

  if (msg.type === "source") {
    updateSource(msg.value);
  }
};
