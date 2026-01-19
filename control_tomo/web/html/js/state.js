export const STATE_MAP = {

  WEB_CTRL:  { group: "source", label: "Web Control", type: "source" },
  AUTO_CTRL: { group: "source", label: "Auto Control", type: "source" },

  FAILSAFE:  { group: "safety", label: "Failsafe", type: "failsafe" },
  EMERGENCY: { group: "safety", label: "EMERGENCY", type: "emergency" },

  ARMED: { group: "states", label: "Armed" },
  POWER: { group: "states", label: "Power Mode" },
  LIGHT: { group: "states", label: "Lights Mode" },

  ENGINE: { group: "events", label: "Start Engine" },
  CLUTCH: { group: "events", label: "Clutch Down" },
  SPEED:  { group: "events", label: "High Speed" },
  MOVE:   { group: "events", label: "Move Allowed" },

  FP: { group: "lights", label: "Front Position" },
  FS: { group: "lights", label: "Front Short" },
  FL: { group: "lights", label: "Front Long" },
  BACK: { group: "lights", label: "Back" },
  LB: { group: "lights", label: "Left Blink" },
  RB: { group: "lights", label: "Right Blink" },
};

export const localState = {
  FAILSAFE: "0",
  WEB_CTRL: "0",
  EMERGENCY: "0",
  ARMED: "0",
  POWER: "0",
  LIGHT: "0",
};
