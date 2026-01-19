import { STATE_MAP, localState } from "./state.js";

const containers = {
  source: document.getElementById("source"),
  safety: document.getElementById("safety"),
  states: document.getElementById("states"),
  events: document.getElementById("events"),
  lights: document.getElementById("lights"),
};

const buttons = {};

// --------------------------------------------------
// CREATE BUTTON
// --------------------------------------------------
export function getButton(name, meta) {
  if (buttons[name]) return buttons[name];

  const btn = document.createElement("div");
  btn.className = "btn off";
  btn.textContent = meta.label;

  btn.onclick = () => {

    // ---------- FAILSAFE (read only) ----------
    if (meta.type === "failsafe") return;

    // ---------- EMERGENCY ----------
    if (meta.type === "emergency") {
      const next = localState[name] === "1" ? "0" : "1";
      localState[name] = next;
      btn.className = next === "1" ? "btn emergency" : "btn off";
      window.sendEmergency(next === "1");
      return;
    }

    // ---------- SOURCE (WEB / AUTO) ----------
    if (meta.type === "source") {
      const isActive = localState[name] === "1";

      // reset both locally
      localState.WEB_CTRL = "0";
      localState.AUTO_CTRL = "0";
      buttons.WEB_CTRL.className = "btn off";
      buttons.AUTO_CTRL.className = "btn off";

      if (!isActive) {
        localState[name] = "1";
        btn.className = "btn on";
      }

      window.sendForce(
        localState.WEB_CTRL === "1",
        localState.AUTO_CTRL === "1"
      );
      return;
    }

    // ---------- NORMAL TOGGLES ----------
    if (localState.WEB_CTRL !== "1") return; // web-only controls

    const next = localState[name] === "1" ? 0 : 1;
    localState[name] = String(next);
    window.sendCmd(meta.group, name, next);
  };

  containers[meta.group].appendChild(btn);
  buttons[name] = btn;
  return btn;
}

// --------------------------------------------------
// UPDATE SOURCE FROM FACTORY
// --------------------------------------------------
export function updateSource(source) {
  // reset
  localState.WEB_CTRL = "0";
  localState.AUTO_CTRL = "0";
  buttons.WEB_CTRL.className = "btn off";
  buttons.AUTO_CTRL.className = "btn off";

  if (source === "WEB") {
    localState.WEB_CTRL = "1";
    buttons.WEB_CTRL.className = "btn on";
  }

  if (source === "AUTO") {
    localState.AUTO_CTRL = "1";
    buttons.AUTO_CTRL.className = "btn on";
  }
}

// --------------------------------------------------
// UPDATE STATE (ESP → UI)
// --------------------------------------------------
export function updateState(name, value) {
  const meta = STATE_MAP[name];
  if (!meta) return;

  localState[name] = value;
  const btn = buttons[name];
  if (!btn) return;

  if (meta.type === "failsafe") {
    btn.className = value === "1" ? "btn failsafe-active" : "btn failsafe-ok";
  } else if (meta.type === "emergency") {
    btn.className = value === "1" ? "btn emergency" : "btn off";
  } else {
    btn.className = value === "1" ? "btn on" : "btn off";
  }
}

// --------------------------------------------------
// INIT UI
// --------------------------------------------------
export function initUI() {
  Object.entries(STATE_MAP).forEach(([name, meta]) => {
    getButton(name, meta);
  });
}
