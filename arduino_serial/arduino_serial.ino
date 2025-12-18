const int ENGINE_START_PIN = 7;
const int FRONT_LONG_PIN = 2;
const int FRONT_SHORT_PIN = 3;
const int BACK_PIN = 4;
const int LEFT_BLINK_PIN = 5;
const int RIGHT_BLINK_PIN = 6;

String inputString = "";

// Blink state variables
bool leftBlinkActive = false;
bool rightBlinkActive = false;
bool leftBlinkState = false;
bool rightBlinkState = false;

unsigned long previousMillis = 0;
unsigned long blinkInterval = 500; // default 0.5s, može se promijeniti preko ROS parametra

void setup() {
  pinMode(ENGINE_START_PIN, OUTPUT);
  digitalWrite(ENGINE_START_PIN, LOW);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  digitalWrite(FRONT_LONG_PIN, LOW);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  digitalWrite(FRONT_SHORT_PIN, LOW);
  pinMode(BACK_PIN, OUTPUT);
  digitalWrite(BACK_PIN, LOW);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);
  digitalWrite(RIGHT_BLINK_PIN, LOW);

  Serial.begin(115200);
}

void loop() {
  // --- Serial input parsing ---
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

  // --- Blinkers handling ---
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;

    // Left blinker
    if (leftBlinkActive) {
      leftBlinkState = !leftBlinkState;
      digitalWrite(LEFT_BLINK_PIN, leftBlinkState ? HIGH : LOW);
    } else {
      digitalWrite(LEFT_BLINK_PIN, LOW);
      leftBlinkState = false;
    }

    // Right blinker
    if (rightBlinkActive) {
      rightBlinkState = !rightBlinkState;
      digitalWrite(RIGHT_BLINK_PIN, rightBlinkState ? HIGH : LOW);
    } else {
      digitalWrite(RIGHT_BLINK_PIN, LOW);
      rightBlinkState = false;
    }
  }
}

// --- Command processing ---
void processCommand(String cmd) {
  if (cmd == "ENGINE_ON") {
    digitalWrite(ENGINE_START_PIN, HIGH);
    Serial.println("ENGINE ON");
  }
  else if (cmd == "ENGINE_OFF") {
    digitalWrite(ENGINE_START_PIN, LOW);
    Serial.println("ENGINE OFF");
  }
  else if (cmd == "FRONT_SHORT_LIGHT_ON") {
    digitalWrite(FRONT_SHORT_PIN, HIGH);
    Serial.println("FRONT SHORT LIGHT ON");
  }
  else if (cmd == "FRONT_SHORT_LIGHT_OFF") {
    digitalWrite(FRONT_SHORT_PIN, LOW);
    Serial.println("FRONT SHORT LIGHT OFF");
  }
  else if (cmd == "FRONT_LONG_LIGHT_ON") {
    digitalWrite(FRONT_LONG_PIN, HIGH);
    Serial.println("FRONT LONG LIGHT ON");
  }
  else if (cmd == "FRONT_LONG_LIGHT_OFF") {
    digitalWrite(FRONT_LONG_PIN, LOW);
    Serial.println("FRONT LONG LIGHT OFF");
  }
  else if (cmd == "BACK_LIGHT_ON") {
    digitalWrite(BACK_PIN, HIGH);
    Serial.println("BACK LIGHT ON");
  }
  else if (cmd == "BACK_LIGHT_OFF") {
    digitalWrite(BACK_PIN, LOW);
    Serial.println("BACK LIGHT OFF");
  }
  else if (cmd == "LEFT_BLINK_ON") {
    leftBlinkActive = true;
    Serial.println("LEFT BLINK ON");
  }
  else if (cmd == "LEFT_BLINK_OFF") {
    leftBlinkActive = false;
    leftBlinkState = false;
    digitalWrite(LEFT_BLINK_PIN, LOW);
    Serial.println("LEFT BLINK OFF");
  }
  else if (cmd == "RIGHT_BLINK_ON") {
    rightBlinkActive = true;
    Serial.println("RIGHT BLINK ON");
  }
  else if (cmd == "RIGHT_BLINK_OFF") {
    rightBlinkActive = false;
    rightBlinkState = false;
    digitalWrite(RIGHT_BLINK_PIN, LOW);
    Serial.println("RIGHT BLINK OFF");
  }
  else if (cmd.startsWith("SET_BLINK_INTERVAL")) {
    int interval = cmd.substring(cmd.indexOf(" ") + 1).toInt();
    if (interval > 0) {
      blinkInterval = interval;
      Serial.print("Blink interval set to: ");
      Serial.println(blinkInterval);
    }
  }
  else {
    Serial.println("UNKNOWN COMMAND");
  }
}
