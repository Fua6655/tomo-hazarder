// ui.js
import { STATE_MAP, localState } from "./state.js";

const containers = {
  states: document.getElementById("states"),
  events: document.getElementById("events"),
  lights: document.getElementById("lights"),
};

const buttons = {};

export function getButton(name, meta, sendCommand) {
  if (buttons[name]) return buttons[name];

  const btn = document.createElement("div");
  btn.className = "btn";
  btn.textContent = meta.label;

  btn.onclick = () => {
    if (meta.group === "events" && localState.ARMED !== "1") return;
    if (meta.group === "lights" && localState.LIGHT !== "1") return;

    const current = localState[name] === "1" ? 1 : 0;
    const next = current ? 0 : 1;

    sendCommand(meta.group, name, next);
  };

  containers[meta.group].appendChild(btn);
  buttons[name] = btn;
  return btn;
}

export function updateState(name, value) {
  const meta = STATE_MAP[name];
  if (!meta) return;

  localState[name] = value;
  const btn = buttons[name] || getButton(name, meta, () => {});

  if (meta.type === "failsafe") {
    btn.className = "btn " + (value === "1" ? "failsafe-active" : "failsafe-ok");
    return;
  }

  btn.className = "btn " + (value === "1" ? "on" : "off");
}

