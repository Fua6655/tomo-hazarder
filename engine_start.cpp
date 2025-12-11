const int RELAY_PIN = 7;  // pin releja
String inputString = "";

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputString.trim();
      if (inputString == "ON") {
        digitalWrite(RELAY_PIN, HIGH);  // pali relej
      }
      inputString = "";
    } else {
      inputString += c;
    }
  }
}
