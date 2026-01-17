#include <WiFi.h>
#include <WiFiUdp.h>

// =====================================================
// ================= WIFI CONFIG =======================
// =====================================================
const char* ssid = "Villa_Milano";
const char* pass = "10203040";

// =====================================================
// ================= UDP CONFIG ========================
// =====================================================
WiFiUDP udp;
const uint16_t LOCAL_PORT = 8888;   // prima komande (ROS / control_factory)
const uint16_t WEB_PORT   = 9999;   // šalje status/log web serveru

IPAddress WEB_IP;                   // IP laptopa (dinamički zapamćen)

// =====================================================
// ================= FAILSAFE ==========================
// =====================================================
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 1000;
bool failsafeActive = false;

// =====================================================
// ================= GPIO ==============================
// =====================================================
const int ENGINE_START_PIN    = 8;
const int CLUTCH_PIN          = 9;
const int THROTTLE_PIN        = 10;

const int FRONT_POSITION_PIN  = 2;
const int FRONT_LONG_PIN      = 3;
const int FRONT_SHORT_PIN     = 4;
const int BACK_PIN            = 5;
const int LEFT_BLINK_PIN      = 6;
const int RIGHT_BLINK_PIN     = 7;

// =====================================================
// ================= STATE TRACKING ====================
// =====================================================
bool armedPrev=false, powerPrev=false, lightPrev=false;
bool enginePrev=false, clutchPrev=false, speedPrev=false, movePrev=false;
bool fpPrev=false, fsPrev=false, flPrev=false, backPrev=false, lbPrev=false, rbPrev=false;

// =====================================================
// ================= BUFFER ============================
// =====================================================
char udpBuffer[256];

// =====================================================
// ================= SETUP =============================
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== ESP32 BOOT ===");

  pinMode(ENGINE_START_PIN, OUTPUT);
  pinMode(CLUTCH_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(FRONT_POSITION_PIN, OUTPUT);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  pinMode(BACK_PIN, OUTPUT);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);

  allOutputsLow();

  // -------- WiFi --------
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi CONNECTED");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // -------- UDP --------
  udp.begin(LOCAL_PORT);
  Serial.println("UDP READY");
  Serial.println("=== ESP READY ===");

  sendLog("WiFi connected");
  sendLog("ESP READY");
  sendState("IP", WiFi.localIP().toString());
}

// =====================================================
// ================= LOOP ==============================
// =====================================================
void loop() {
  handleUDP();
  handleFailsafe();
}

// =====================================================
// ================= UDP RX ============================
// =====================================================
void handleUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
  if (len <= 0) return;

  udpBuffer[len] = '\0';
  String cmd = String(udpBuffer);
  cmd.trim();

  lastPacketTime = millis();
  WEB_IP = udp.remoteIP();   // ZAPAMTI IP LAPTOPA

  // -------- HEARTBEAT --------
  if (cmd.startsWith("HEARTBEAT")) {
    clearFailsafe();
    return;
  }

  // -------- CMD + ACK --------
  if (cmd.startsWith("CMD")) {
    int seq;
    sscanf(cmd.c_str(), "CMD,%d", &seq);

    sendRaw("ACK," + String(seq));
    clearFailsafe();

    int idx = cmd.indexOf(',', 4);
    if (idx > 0) {
      processCommand(cmd.substring(idx + 1));
    }
    return;
  }

  // -------- DIRECT --------
  clearFailsafe();
  processCommand(cmd);
}

// =====================================================
// ================= COMMANDS ==========================
// =====================================================
void processCommand(const String& cmd) {

  if (cmd.startsWith("STATES")) {
    int a,p,l;
    sscanf(cmd.c_str(),"STATES,%d,%d,%d",&a,&p,&l);
    bool armedBefore = armedPrev;
    logEdge("ARMED",a,armedPrev);
    logEdge("POWER",p,powerPrev);
    logEdge("LIGHT",l,lightPrev);
    if (!armedBefore && a == 1) {
        publishAllStates();
    }
    return;
  }

  if (cmd.startsWith("EVENTS")) {
    int e,c,s,m;
    sscanf(cmd.c_str(),"EVENTS,%d,%d,%d,%d",&e,&c,&s,&m);

    logEdge("ENGINE",e,enginePrev);
    logEdge("CLUTCH",c,clutchPrev);
    logEdge("SPEED",s,speedPrev);
    logEdge("MOVE",m,movePrev);

    digitalWrite(ENGINE_START_PIN, e);
    digitalWrite(CLUTCH_PIN, c);
    digitalWrite(THROTTLE_PIN, s);
    return;
  }

  if (cmd.startsWith("LIGHTS")) {
    int fp,fs,fl,b,l,r;
    sscanf(cmd.c_str(),"LIGHTS,%d,%d,%d,%d,%d,%d",&fp,&fs,&fl,&b,&l,&r);

    logEdge("FP",fp,fpPrev);
    logEdge("FS",fs,fsPrev);
    logEdge("FL",fl,flPrev);
    logEdge("BACK",b,backPrev);
    logEdge("LB",l,lbPrev);
    logEdge("RB",r,rbPrev);

    digitalWrite(FRONT_POSITION_PIN,fp);
    digitalWrite(FRONT_SHORT_PIN,fs);
    digitalWrite(FRONT_LONG_PIN,fl);
    digitalWrite(BACK_PIN,b);
    digitalWrite(LEFT_BLINK_PIN,l);
    digitalWrite(RIGHT_BLINK_PIN,r);
    return;
  }
}

// =====================================================
// ================= FAILSAFE ==========================
// =====================================================
void handleFailsafe() {
  if (!failsafeActive && millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
    failsafeActive = true;
    allOutputsLow();

    sendState("FAILSAFE", "1");
    sendLog("FAILSAFE ON");

    Serial.println("FAILSAFE ON");
  }
}

void clearFailsafe() {
  if (failsafeActive) {
    failsafeActive = false;

    sendState("FAILSAFE", "0");
    sendLog("FAILSAFE OFF");

    Serial.println("FAILSAFE OFF");
  }
}

// =====================================================
// ================= UDP TX HELPERS ====================
// =====================================================
void sendRaw(const String& msg) {
  if (!WEB_IP) return;
  udp.beginPacket(WEB_IP, WEB_PORT);
  udp.print(msg);
  udp.endPacket();
}

void sendLog(const String& text) {
  sendRaw("LOG," + text);
  Serial.print("[LOG] ");
  Serial.println(text);
}

void sendState(const String& name, const String& value) {
  sendRaw("STATE," + name + "," + value);
}

// =====================================================
// ================= UTIL ==============================
// =====================================================
void allOutputsLow() {
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

void publishAllStates() {
  sendState("ARMED", armedPrev ? "1" : "0");
  sendState("POWER", powerPrev ? "1" : "0");
  sendState("LIGHT", lightPrev ? "1" : "0");

  sendState("ENGINE", enginePrev ? "1" : "0");
  sendState("CLUTCH", clutchPrev ? "1" : "0");
  sendState("SPEED", speedPrev ? "1" : "0");
  sendState("MOVE", movePrev ? "1" : "0");

  sendState("FP", fpPrev ? "1" : "0");
  sendState("FS", fsPrev ? "1" : "0");
  sendState("FL", flPrev ? "1" : "0");
  sendState("BACK", backPrev ? "1" : "0");
  sendState("LB", lbPrev ? "1" : "0");
  sendState("RB", rbPrev ? "1" : "0");
}


void logEdge(const char* name, bool now, bool &prev) {
  if (now != prev) {
    prev = now;

    sendState(name, now ? "1" : "0");

    Serial.print("[EDGE] ");
    Serial.print(name);
    Serial.println(now ? ": ON" : ": OFF");
  }
}
