//Barcarolo, Rodríguez, Kang y Beck
//Grupo 8

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <LiquidCrystal_I2C.h>

#define LCD_ADDR 0x27

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

Adafruit_AHTX0 aht;

#define FOTORRESISTENCIA 33
#define GAS 32
#define METANO 34
#define LED1 5
#define LED2 18
#define LED3 19
#define BOTON1 15
#define BOTON2 2
#define BOTON3 4
#define BOTON4 16
#define BOTON5 17


void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  
  if (!aht.begin(&Wire)) {
    Serial.println("No se encontró AHT10/AHT20, revisa conexiones!");
    while (1);
  }
  Serial.println("Sensor AHT10 detectado correctamente.");
  
  pinMode(FOTORRESISTENCIA, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BOTON1, INPUT_PULLUP);
  pinMode(BOTON2, INPUT_PULLUP);
  pinMode(BOTON3, INPUT_PULLUP);
  pinMode(BOTON4, INPUT_PULLUP);
  pinMode(BOTON5, INPUT_PULLUP);
  
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0); // Columna 0, fila 0
  lcd.print("Hola ESP32!");

  lcd.setCursor(0, 1); // Columna 0, fila 1
  lcd.print("LCD 16x2 prueba");
}
