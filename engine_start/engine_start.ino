const int RELAY_PIN = 7;  // pin releja
String inputString = "";

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // relej inicijalno isključen
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputString.trim();  // uklanja eventualne razmake/newline

      if (inputString == "ON") {
        digitalWrite(RELAY_PIN, HIGH);  // pali relej
        Serial.println("RELAY ON");
      } 
      else if (inputString == "OFF") {
        digitalWrite(RELAY_PIN, LOW);   // gasi relej
        Serial.println("RELAY OFF");
      }
      else {
        Serial.println("UNKNOWN COMMAND");
      }

      inputString = ""; // reset stringa za sljedeću naredbu
    } 
    else {
      inputString += c; // dodaj znak u string
    }
  }
}
