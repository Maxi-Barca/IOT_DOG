#define LED1 5
#define LED2 18
#define LED3 19
#define BOTON1 16
#define BOTON2 17


void setup() {
    pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BOTON1, INPUT_PULLUP);
  pinMode(BOTON2, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("iniciado");

}

void loop() {
  // Encender LEDs con BOTON1
  if (digitalRead(BOTON1) == LOW) {
    delay(50); // debounce
    if (digitalRead(BOTON1) == LOW) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, HIGH);
      Serial.println("boton 1 apretado");
      while (digitalRead(BOTON1) == LOW); // espera a soltar
    }
  }

  // Apagar LEDs con BOTON2
  if (digitalRead(BOTON2) == LOW) {
    delay(50); // debounce
    if (digitalRead(BOTON2) == LOW) {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      Serial.println("boton 2 apretado");
      while (digitalRead(BOTON2) == LOW); // espera a soltar
    }
  }
}
