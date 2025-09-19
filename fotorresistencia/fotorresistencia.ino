//Barcarolo, Rodríguez, Kang y Beck
//Grupo 8

#define FOTORRESISTENCIA 33


void setup() {
  pinMode(FOTORRESISTENCIA, INPUT);
  Serial.begin(115200);
}

void loop() {
  int valor = analogRead(FOTORRESISTENCIA);
  int mapeado = map(valor, 0, 4095, 0, 100);
  Serial.print("luz:");
  Serial.print(mapeado);
  Serial.println("%");
}
