import { STATE_MAP, localState } from "./state.js";

const containers = {
  top: document.getElementById("top-row"),
  states: document.getElementById("states"),
  events: document.getElementById("events"),
  lights: document.getElementById("lights"),
};

const buttons = {};

export function getButton(name, meta) {
  if (buttons[name]) return buttons[name];

  const btn = document.createElement("div");
  btn.className = "btn off";
  btn.textContent = meta.label;

  btn.onclick = () => {
    if (meta.type === "failsafe") return;

    if (meta.type === "emergency") {
      const next = localState[name] === "1" ? "0" : "1";
      localState[name] = next;
      btn.className = next === "1" ? "btn emergency" : "btn off";
      window.sendCommand("emergency", name, next === "1");
      return;
    }

    if (meta.type === "toggle") {
      const next = localState[name] === "1" ? "0" : "1";
      localState[name] = next;
      btn.className = "btn " + (next === "1" ? "on" : "off");
      window.sendCommand("web_ctrl", name, next === "1" ? 1 : 0);
      return;
    }

    const current = localState[name] === "1" ? 1 : 0;
    const next = current ? 0 : 1;
    window.sendCommand(meta.group, name, next);
  };

  containers[meta.group].appendChild(btn);
  buttons[name] = btn;
  return btn;
}

export function updateState(name, value) {
  const meta = STATE_MAP[name];
  if (!meta) return;

  localState[name] = value;
  const btn = buttons[name] || getButton(name, meta);

  if (meta.type === "failsafe") {
    btn.className = "btn " + (value === "1" ? "failsafe-active" : "failsafe-ok");
  } else if (meta.type === "emergency") {
    btn.className = value === "1" ? "btn emergency" : "btn off";
  } else {
    btn.className = "btn " + (value === "1" ? "on" : "off");
  }
}

export function initTopRow() {
  ["FAILSAFE", "WEB_CTRL", "EMERGENCY"].forEach((name) => {
    const meta = STATE_MAP[name];
    if (!meta) return;
    getButton(name, meta);
  });
}
