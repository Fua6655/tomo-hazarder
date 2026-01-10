#include <WiFi.h>
#include <WiFiUdp.h>

// ================== WIFI ==================
const char* ssid = "Villa_Milano";
const char* pass = "10203040";

WiFiUDP udp;
const uint16_t localPort = 8888;

// WiFi state
unsigned long wifiStartTime = 0;
bool wifiConnecting = false;
bool wifiConnected = false;

// ================== UDP ==================
char udpBuffer[256];
unsigned long lastUdpTime = 0;
bool failsafeActive = true;

const unsigned long FAILSAFE_TIMEOUT_MS = 500;

// ================== PIN DEFINITIONS ==================
const int ENGINE_START_PIN    = 8;
const int CLUTCH_PIN          = 9;
const int THROTTLE_PIN        = 10;

const int FRONT_POSITION_PIN  = 2;
const int FRONT_LONG_PIN      = 3;
const int FRONT_SHORT_PIN     = 4;
const int BACK_PIN            = 5;
const int LEFT_BLINK_PIN      = 6;
const int RIGHT_BLINK_PIN     = 7;

// ================== STATE TRACKING ==================
bool armedPrev  = false;
bool powerPrev  = false;
bool lightPrev  = false;

bool enginePrev = false;
bool clutchPrev = false;
bool speedPrev  = false;
bool movePrev   = false;

bool fpPrev = false;
bool fsPrev = false;
bool flPrev = false;
bool backPrev = false;
bool lbPrev = false;
bool rbPrev = false;

// ====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32 BOOT");

  pinMode(ENGINE_START_PIN, OUTPUT);
  pinMode(CLUTCH_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);

  pinMode(FRONT_POSITION_PIN, OUTPUT);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  pinMode(BACK_PIN, OUTPUT);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);

  allOutputsOff();
  Serial.println("GPIO READY");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  wifiStartTime = millis();
  wifiConnecting = true;

  Serial.println("WIFI CONNECTING");
}

// ====================================================

void loop() {
  handleWiFi();
  handleUDP();
  handleFailsafe();
}

// ====================================================

void handleWiFi() {
  if (!wifiConnecting || wifiConnected) return;

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    wifiConnecting = false;

    Serial.print("WIFI CONNECTED, IP=");
    Serial.println(WiFi.localIP());

    udp.begin(localPort);
    Serial.print("UDP LISTENING ON ");
    Serial.println(localPort);

    lastUdpTime = millis();
    return;
  }

  if (millis() - wifiStartTime > 15000) {
    wifiConnecting = false;
    Serial.println("WIFI TIMEOUT");
  }
}

// ====================================================

void handleUDP() {
  if (!wifiConnected) return;

  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
  if (len <= 0) return;

  udpBuffer[len] = '\0';
  String cmd = String(udpBuffer);
  cmd.trim();

  lastUdpTime = millis();

  if (failsafeActive) {
    failsafeActive = false;
    Serial.println("FAILSAFE CLEARED");
  }

  if (cmd == "HEARTBEAT") {
    return;  // samo watchdog tick
  }

  processCommand(cmd);
}

// ====================================================

void handleFailsafe() {
  if (!wifiConnected) return;

  if (!failsafeActive && (millis() - lastUdpTime > FAILSAFE_TIMEOUT_MS)) {
    failsafeActive = true;
    allOutputsOff();
    Serial.println("FAILSAFE TRIGGERED");
  }
}

// ====================================================

void allOutputsOff() {
  digitalWrite(ENGINE_START_PIN, LOW);
  digitalWrite(CLUTCH_PIN, LOW);
  digitalWrite(THROTTLE_PIN, LOW);
  digitalWrite(FRONT_POSITION_PIN, LOW);
  digitalWrite(FRONT_LONG_PIN, LOW);
  digitalWrite(FRONT_SHORT_PIN, LOW);
  digitalWrite(BACK_PIN, LOW);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  digitalWrite(RIGHT_BLINK_PIN, LOW);
}

// ====================================================

void processCommand(String cmd) {

  if (cmd.startsWith("STATES")) {
    int armed, power, light;
    sscanf(cmd.c_str(), "STATES,%d,%d,%d", &armed, &power, &light);
    logEdge("ARMED", armed, armedPrev);
    logEdge("POWER", power, powerPrev);
    logEdge("LIGHT", light, lightPrev);
    return;
  }

  if (cmd.startsWith("EVENTS")) {
    int engine, clutch, speed, move;
    sscanf(cmd.c_str(), "EVENTS,%d,%d,%d,%d",
           &engine, &clutch, &speed, &move);

    logEdge("ENGINE", engine, enginePrev);
    logEdge("CLUTCH", clutch, clutchPrev);
    logEdge("SPEED", speed, speedPrev);
    logEdge("MOVE", move, movePrev);

    digitalWrite(ENGINE_START_PIN, engine ? HIGH : LOW);
    digitalWrite(CLUTCH_PIN, clutch ? HIGH : LOW);
    digitalWrite(THROTTLE_PIN, speed ? HIGH : LOW);
    return;
  }

  if (cmd.startsWith("LIGHTS")) {
    int fp, fs, fl, b, l, r;
    sscanf(cmd.c_str(), "LIGHTS,%d,%d,%d,%d,%d,%d",
           &fp, &fs, &fl, &b, &l, &r);

    logEdge("FRONT POS", fp, fpPrev);
    logEdge("FRONT SHORT", fs, fsPrev);
    logEdge("FRONT LONG", fl, flPrev);
    logEdge("BACK", b, backPrev);
    logEdge("LEFT BLINK", l, lbPrev);
    logEdge("RIGHT BLINK", r, rbPrev);

    digitalWrite(FRONT_POSITION_PIN, fp ? HIGH : LOW);
    digitalWrite(FRONT_SHORT_PIN, fs ? HIGH : LOW);
    digitalWrite(FRONT_LONG_PIN, fl ? HIGH : LOW);
    digitalWrite(BACK_PIN, b ? HIGH : LOW);
    digitalWrite(LEFT_BLINK_PIN, l ? HIGH : LOW);
    digitalWrite(RIGHT_BLINK_PIN, r ? HIGH : LOW);
    return;
  }
}

// ====================================================

void logEdge(const char* name, bool now, bool &prev) {
  if (now != prev) {
    prev = now;
    Serial.print(name);
    Serial.print(": ");
    Serial.println(now ? "ON" : "OFF");
  }
}
