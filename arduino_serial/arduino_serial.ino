// ================== PIN DEFINITIONS ==================
const int ENGINE_START_PIN    = 8;
const int CLUTCH_PIN          = 9;

const int FRONT_POSITION_PIN  = 2;
const int FRONT_LONG_PIN      = 3;
const int FRONT_SHORT_PIN     = 4;
const int BACK_PIN            = 5;
const int LEFT_BLINK_PIN      = 6;
const int RIGHT_BLINK_PIN     = 7;

// ================== SERIAL ==================
String inputString = "";

// ================== STATE TRACKING (EDGE LOGGING) ==================
bool armedPrev  = false;
bool powerPrev  = false;
bool lightPrev  = false;

bool enginePrev = false;
bool clutchPrev = false;

bool fpPrev = false;
bool fsPrev = false;
bool flPrev = false;
bool backPrev = false;
bool lbPrev = false;
bool rbPrev = false;

// ====================================================

void setup() {
  pinMode(ENGINE_START_PIN, OUTPUT);
  pinMode(CLUTCH_PIN, OUTPUT);

  pinMode(FRONT_POSITION_PIN, OUTPUT);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  pinMode(BACK_PIN, OUTPUT);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);

  digitalWrite(ENGINE_START_PIN, LOW);
  digitalWrite(CLUTCH_PIN, LOW);
  digitalWrite(FRONT_POSITION_PIN, LOW);
  digitalWrite(FRONT_LONG_PIN, LOW);
  digitalWrite(FRONT_SHORT_PIN, LOW);
  digitalWrite(BACK_PIN, LOW);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  digitalWrite(RIGHT_BLINK_PIN, LOW);

  Serial.begin(115200);
  Serial.println("=== ARDUINO READY ===");
}

// ====================================================

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputString.trim();
      processCommand(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }
}

// ====================================================

void processCommand(String cmd) {

  // ---------- STATES (EDGE ONLY) ----------
  if (cmd.startsWith("STATES")) {
    int armed, power, light;
    sscanf(cmd.c_str(), "STATES,%d,%d,%d", &armed, &power, &light);

    logEdge("ARMED", armed, armedPrev);
    logEdge("POWER MODE", power, powerPrev);
    logEdge("LIGHT MODE", light, lightPrev);
    return;
  }

  // ---------- EVENTS ----------
  if (cmd.startsWith("EVENTS")) {
    int engine, clutch;
    sscanf(cmd.c_str(), "EVENTS,%d,%d", &engine, &clutch);

    logEdge("ENGINE", engine, enginePrev);
    logEdge("CLUTCH", clutch, clutchPrev);

    digitalWrite(ENGINE_START_PIN, engine ? HIGH : LOW);
    digitalWrite(CLUTCH_PIN, clutch ? HIGH : LOW);
    return;
  }

  // ---------- LIGHTS ----------
  if (cmd.startsWith("LIGHTS")) {
    int fp, fs, fl, b, l, r;
    sscanf(cmd.c_str(), "LIGHTS,%d,%d,%d,%d,%d,%d",
           &fp, &fs, &fl, &b, &l, &r);

    logEdge("FRONT POSITION", fp, fpPrev);
    logEdge("FRONT SHORT",    fs, fsPrev);
    logEdge("FRONT LONG",     fl, flPrev);
    logEdge("BACK",           b,  backPrev);
    logEdge("LEFT BLINK",     l,  lbPrev);
    logEdge("RIGHT BLINK",    r,  rbPrev);

    digitalWrite(FRONT_POSITION_PIN, fp ? HIGH : LOW);
    digitalWrite(FRONT_SHORT_PIN, fs ? HIGH : LOW);
    digitalWrite(FRONT_LONG_PIN,  fl ? HIGH : LOW);
    digitalWrite(BACK_PIN,        b  ? HIGH : LOW);
    digitalWrite(LEFT_BLINK_PIN,  l  ? HIGH : LOW);
    digitalWrite(RIGHT_BLINK_PIN, r  ? HIGH : LOW);
    return;
  }
}

// ====================================================

void logEdge(const char* name, bool now, bool &prev) {
  if (now != prev) {
    prev = now;
    Serial.print(name);
    Serial.println(now ? ": ON" : ": OFF");
  }
}
