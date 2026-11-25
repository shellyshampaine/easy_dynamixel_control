// Echoes back every line it receives over serial.
// Each line must end with '\n'.

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();                       // drop '\r' or stray spaces
    if (line.length() > 0) {
      Serial.print("Echo: ");
      Serial.println(line);
    }
  }
}