//Barcarolo, Rodríguez, Kang y Beck
//Grupo 8

#include <Wire.h>
#include <Adafruit_AHTX0.h>

// Crear objeto para el sensor
Adafruit_AHTX0 aht;

void setup() {
  Serial.begin(115200);
  Serial.println("Prueba AHT10 / AHT20");

  Wire.begin(21, 22);

  if (!aht.begin(&Wire)) {
    Serial.println("No se encontró AHT10/AHT20, revisa conexiones!");
    while (1);
  }
  Serial.println("Sensor AHT10 detectado correctamente.");
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // obtener datos

  Serial.print("Temperatura: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Humedad: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  Serial.println("--------------------");
  delay(2000);
}
