#define FOTORRESISTENCIA 32


void setup() {
  pinMode(FOTORRESISTENCIA, INPUT);
  Serial.begin(9600);
}

void loop() {
  valor = analogRead(FOTORRESISTENCIA);
  Serial.println(valor);
}
