#define FOTORRESISTENCIA 32


void setup() {
  pinMode(FOTORRESISTENCIA, INPUT);
}

void loop() {
  Serial.println(analogRead(FOTORRESISTENCIA));
}
