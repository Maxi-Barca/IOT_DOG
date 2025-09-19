//Barcarolo, Rodríguez, Kang y Beck
//Grupo 8

#define GAS 32
#define METANO 34

void setup() {
  Serial.begin(115200);
  Serial.println("Prueba MQ-2)");
}

void loop() {
  int gasValor = analogRead(GAS);
  int gasMap = map(gasValor, 0, 4095, 0, 100);
  Serial.print("Gas: ");
  Serial.print(gasMap);
  Serial.println("%");



  int metanoValor = analogRead(METANO);
  int metanoMap = map(metanoValor, 0, 4095, 0, 100);
  Serial.print("Metano: ");
  Serial.print(metanoMap);
  Serial.println("%");


  delay(1000);
}
