#define FOTORRESISTENCIA 34


void setup() {
  pinMode(FOTORRESISTENCIA, INPUT);
  Serial.begin(9600);
}

void loop() {
  int valor = analogRead(FOTORRESISTENCIA);
  int mapeado = map(valor,0,4095,0,100);
  Serial.print("luz:");
  Serial.print(mapeado);
  Serial.println("%");
}
