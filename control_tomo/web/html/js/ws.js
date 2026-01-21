import { initUI, updateState, updateSource } from "./ui.js";
import { webState } from "./state.js";

const ws = new WebSocket(
  (location.protocol === "https:" ? "wss://" : "ws://") +
  location.host + "/ws"
);

ws.onopen = () => {
  console.log("[WS] connected");
  initUI();
};

// ==================================================
// GLOBAL API
// ==================================================
window.sendCmd = function (target) {
  ws.send(JSON.stringify({
    type: "cmd",
    target,
    data: buildPayload(target),
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

// ==================================================
// ⭐ STATEFUL PAYLOAD BUILDER ⭐
// ==================================================
function buildPayload(target) {

  const order = {
    events: ["ENGINE", "CLUTCH", "SPEED", "MOVE"],
    lights: ["FP", "FS", "FL", "BACK", "LB", "RB"],
  };

  if (!order[target]) return [];

  return order[target].map(k => webState[target][k] ?? 0);
}

// ==================================================
// INCOMING
// ==================================================
ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);

  if (msg.type === "state") {
    updateState(msg.name, msg.value);
  }

  if (msg.type === "source") {
    updateSource(msg.value);
  }
};
