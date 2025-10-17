#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LCD_ADDR 0x27

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

void setup() {
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0); // Columna 0, fila 0
  lcd.print("Hola ESP32!");

  lcd.setC  ursor(0, 1); // Columna 0, fila 1
  lcd.print("LCD 16x2 prueba");
}

void loop() {
  static int contador = 0;

  lcd.setCursor(0, 0);
  lcd.print("Contador: ");
  lcd.print(contador);
  lcd.print("   ");

  contador++;
  delay(1000);
}
