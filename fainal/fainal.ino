//Barcarolo, Rodríguez, Kang y Beck
//Grupo 8


#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <ESP32Time.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>


#define BOTtoken "7561786153:AAEAnTbyt_XnvsfXFY1onCdNb3hJCKMGF_o"
#define CHAT_ID "7389596977"


WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

const char* ssid = "MECA-IoT";
const char* password = "IoT$2026";
const char* ntpServer = "pool.ntp.org";
int gmtOffset;

ESP32Time rtc;

void sincronizarHora() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
  } else {
    Serial.println("Error al sincronizar hora");
  }
}

#define LCD_ADDR 0x27

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

Adafruit_AHTX0 aht;

sensors_event_t humidity, temp;

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
#define P1 1
#define ESPERA1 2
#define ESPERA2 3
#define ESPERA4 4
#define ESPERA6 5
#define ESPERA8 6
#define TEMP 10
#define RESTARTEMP 11
#define SUMARTEMP 12
#define GMT_MQTT 13
#define SUMARGMT 14
#define RESTARGMT 15
#define SUMARINT 16
#define RESTARINT 17
#define PANTALLA_GAS 18
#define SUMARGAS 19
#define RESTARGAS 20
#define LDR 21
#define SUMARLDR 22
#define RESTARLDR 23
#define SUMARHUM 24
#define RESTARHUM 25
#define SUMARMETANO 26
#define RESTARMETANO 27

int estado = P1;

int millis_actual;
int millis_aht;

int valorLdr;
int ldrMap;

int gasValor;
int gasMap;
int metanoValor;
int metanoMap;

int intervalo;

int umbralLdr;
int umbralMetano;
int umbralGas;
int umbralTemp;
int umbralHum;
int umbralIntervalo;
int umbralGMT;

bool mensajeEnviadoTemp = false;
bool mensajeEnviadoHum = false;
bool mensajeEnviadoLdr = false;
bool mensajeEnviadoGas = false;
bool mensajeEnviadoMetano = false;

Preferences preferences;

void pantallaGeneral(void);
void pantallaLdr(void);
void pantallaGas(void);
void pantallaTemp(void);
void pantallaGMT_MQTT(void);

TaskHandle_t Task1;
TaskHandle_t Task2;

void Task1code(void* pvParameters);
void Task2code(void* pvParameters);

void actualizarHoraConGMT() {
  int offset = umbralGMT * 3600;

  configTime(offset, 0, ntpServer);

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
    Serial.println("Hora sincronizada con GMT actualizado");
  } else {
    Serial.println("Error al sincronizar hora con nuevo GMT");
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  if (!aht.begin(&Wire)) {
    Serial.println("No se encontró AHT10/AHT20, revisa conexiones!");
    while (1)
      ;
  }
  Serial.println("Sensor AHT10 detectado correctamente.");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  bot.sendMessage(CHAT_ID, "hola", "");

  pinMode(FOTORRESISTENCIA, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BOTON1, INPUT_PULLUP);
  pinMode(BOTON2, INPUT_PULLUP);
  pinMode(BOTON3, INPUT_PULLUP);
  pinMode(BOTON4, INPUT_PULLUP);
  pinMode(BOTON5, INPUT_PULLUP);

  preferences.begin("config", false);
  umbralLdr = preferences.getInt("umbralLdr", 70);
  umbralMetano = preferences.getInt("umbralMetano", 10);
  umbralGas = preferences.getInt("umbralGas", 10);
  umbralTemp = preferences.getInt("umbralTemp", 23);
  umbralHum = preferences.getInt("umbralHum", 40);
  umbralIntervalo = preferences.getInt("umbralIntervalo", 30000);
  umbralGMT = preferences.getInt("umbralGMT", -3);
  preferences.end();

  int offset = umbralGMT * 3600;
  configTime(offset, 0, ntpServer);
  sincronizarHora();

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);  // Columna 0, fila 0
  lcd.print("Hola ESP32!");

  lcd.setCursor(0, 1);  // Columna 0, fila 1
  lcd.print("LCD 16x2 prueba");
  
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */

  delay(500);

  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */

  delay(500);
}

void Task1code(void* pvParameters) {
  for (;;) {
    millis_actual = millis();

    if (temp.temperature >= umbralTemp) {
      if (mensajeEnviadoTemp == false) {
        bot.sendMessage(CHAT_ID, "La temperatura supera el umbral", "");
        mensajeEnviadoTemp = true;
      }
    }
    if (temp.temperature < umbralTemp) {
      mensajeEnviadoTemp = false;
    }

    if (humidity.relative_humidity >= umbralHum) {
      if (mensajeEnviadoHum == false) {
        bot.sendMessage(CHAT_ID, "La humedad supera el umbral", "");
        mensajeEnviadoHum = true;
      }
    }
    if (humidity.relative_humidity < umbralHum) {
      mensajeEnviadoHum = false;
    }

    if (gasMap >= umbralGas) {
      if (mensajeEnviadoGas == false) {
        bot.sendMessage(CHAT_ID, "La cantidad de gas supera el umbral", "");
        mensajeEnviadoGas = true;
      }
    }
    if (gasMap < umbralGas) {
      mensajeEnviadoGas = false;
    }

    if (metanoMap >= umbralMetano) {
      if (mensajeEnviadoMetano == false) {
        bot.sendMessage(CHAT_ID, "La cantidad de metano supera el umbral", "");
        mensajeEnviadoMetano = true;
      }
    }
    if (metanoMap < umbralMetano) {
      mensajeEnviadoMetano = false;
    }

    if (ldrMap >= umbralLdr) {
      if (mensajeEnviadoLdr == false) {
        bot.sendMessage(CHAT_ID, "La cantidad de luz supera el umbral", "");
        mensajeEnviadoLdr = true;
      }
    }
    if (ldrMap < umbralLdr) {
      mensajeEnviadoLdr = false;
    }

    // rsto consulta la API de telegram para traer mensajes nuevos que aún no se han procesado
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    // recorrer todos los mensajes nuevos recibidos
    for (int i = 0; i < numNewMessages; ++i) {
      
      // extraer el texto del mensaje actual
      String text = bot.messages[i].text;

      // imprimir en el monitor serial el texto recibido para depuración
      Serial.print("Mensaje recibido: ");
      Serial.println(text);


      // Comparar el texto recibido
      if (text.equals("TA")) {
        String tempActual = String(temp.temperature, 2);
        bot.sendMessage(CHAT_ID, tempActual, "");
      }
      if (text.equals("HA")) {
        String humActual = String(humidity.relative_humidity, 2);
        bot.sendMessage(CHAT_ID, humActual, "");
      }
      if (text.equals("LA")) {
        String luzActual = String(ldrMap, 2);
        bot.sendMessage(CHAT_ID, luzActual, "");
      }
      if (text.equals("GA")) {
        String gasActual = String(gasMap, 2);
        bot.sendMessage(CHAT_ID, gasActual, "");
      }
      if (text.equals("MA")) {
        String metanoActual = String(metanoMap, 2);
        bot.sendMessage(CHAT_ID, metanoActual, "");
      }
    }
  }
}


void Task2code(void* pvParameters) {


  intervalo = 30000;


  for (;;) {
    millis_actual = millis();


    if (millis_actual - millis_aht >= intervalo) {
      aht.getEvent(&humidity, &temp);
      Serial.print("Temperatura: ");
      Serial.print(temp.temperature);
      Serial.println(" °C");


      Serial.print("Humedad: ");
      Serial.print(humidity.relative_humidity);
      Serial.println(" %");


      valorLdr = analogRead(FOTORRESISTENCIA);
      ldrMap = map(valorLdr, 0, 4095, 0, 100);
      Serial.print("luz:");
      Serial.print(ldrMap);
      Serial.println("%");


      gasValor = analogRead(GAS);
      gasMap = map(gasValor, 0, 4095, 0, 100);
      Serial.print("Gas: ");
      Serial.print(gasMap);
      Serial.println("%");


      metanoValor = analogRead(METANO);
      metanoMap = map(metanoValor, 0, 4095, 0, 100);
      Serial.print("Metano: ");
      Serial.print(metanoMap);
      Serial.println("%");


      millis_aht = millis_actual;
    }


    bool peligro = false;
    bool alerta = false;


    if (temp.temperature >= umbralTemp) {
      alerta = true;
    }
    if (temp.temperature >= umbralTemp * 1.3) {
      peligro = true;
    }


    if (humidity.relative_humidity >= umbralHum) {
      alerta = true;
    }
    if (humidity.relative_humidity >= umbralHum * 1.3) {
      peligro = true;
    }


    if (gasMap >= umbralGas) {
      alerta = true;
    }
    if (gasMap >= umbralGas * 1.3) {
      peligro = true;
    }




    if (metanoMap >= umbralMetano) {
      alerta = true;
    }
    if (metanoMap >= umbralMetano * 1.3) {
      peligro = true;
    }


    if (ldrMap >= umbralLdr) {
      alerta = true;
    }
    if (ldrMap >= umbralLdr * 1.3) {
      peligro = true;
    }


    if (peligro) {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, HIGH);  // rojo
    } else if (alerta) {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);  // amarillo
      digitalWrite(LED3, LOW);
    } else {
      digitalWrite(LED1, HIGH);  // verde
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
    }


    switch (estado) {
      case P1:
        pantallaGeneral();
        if (digitalRead(BOTON1) == LOW) {
          estado = ESPERA2;
        }
        if (digitalRead(BOTON2) == LOW) {
          estado = ESPERA4;
        }
        if (digitalRead(BOTON3) == LOW) {
          estado = ESPERA6;
        }
        if (digitalRead(BOTON4) == LOW) {
          estado = ESPERA8;
        }
        break;


      case ESPERA2:
      lcd.clear();
        if (digitalRead(BOTON1) == HIGH) {
          estado = LDR;
        }
        break;


      case ESPERA4:
      lcd.clear();
        if (digitalRead(BOTON2) == HIGH) {
          estado = PANTALLA_GAS;
        }
        break;


      case ESPERA6:
      lcd.clear();
        if (digitalRead(BOTON3) == HIGH) {
          estado = TEMP;
        }
        break;


      case ESPERA8:
      lcd.clear();
        if (digitalRead(BOTON4) == HIGH) {
          estado = GMT_MQTT;
        }
        break;


      case LDR:
        pantallaLdr();
        if (digitalRead(BOTON1) == LOW) {
          estado = SUMARLDR;
        }
        if (digitalRead(BOTON2) == LOW) {
          estado = RESTARLDR;
        }
        if (digitalRead(BOTON5) == LOW) {
          estado = ESPERA1;
        }
        break;


      case PANTALLA_GAS:
        pantallaGas();
        if (digitalRead(BOTON1) == LOW) {
          estado = SUMARGAS;
        }
        if (digitalRead(BOTON2) == LOW) {
          estado = RESTARGAS;
        }
        if (digitalRead(BOTON3) == LOW) {
          estado = SUMARMETANO;
        }
        if (digitalRead(BOTON4) == LOW) {
          estado = RESTARMETANO;
        }
        if (digitalRead(BOTON5) == LOW) {
          estado = ESPERA1;
        }
        break;


      case TEMP:
        pantallaTemp();
        if (digitalRead(BOTON1) == LOW) {
          estado = SUMARTEMP;
        }
        if (digitalRead(BOTON2) == LOW) {
          estado = RESTARTEMP;
        }
        if (digitalRead(BOTON3) == LOW) {
          estado = SUMARHUM;
        }
        if (digitalRead(BOTON4) == LOW) {
          estado = RESTARHUM;
        }
        if (digitalRead(BOTON5) == LOW) {
          estado = ESPERA1;
        }
        break;


      case GMT_MQTT:
        pantallaGMT_MQTT();
        if (digitalRead(BOTON1) == LOW) {
          estado = SUMARGMT;
        }
        if (digitalRead(BOTON2) == LOW) {
          estado = RESTARGMT;
        }
        if (digitalRead(BOTON3) == LOW) {
          estado = SUMARINT;
        }
        if (digitalRead(BOTON4) == LOW) {
          estado = RESTARINT;
        }
        if (digitalRead(BOTON5) == LOW) {
          estado = ESPERA1;
        }
        break;


      case ESPERA1:
      lcd.clear();
        if (digitalRead(BOTON5) == HIGH) {
          estado = P1;
        }
        break;


      case SUMARLDR:
        pantallaLdr();
        if (digitalRead(BOTON1) == HIGH) {
          umbralLdr = umbralLdr + 1;
          preferences.begin("config", false);
          preferences.putInt("umbralLdr", umbralLdr);
          preferences.end();
          if (umbralLdr > 100) {
            umbralLdr = 100;
          }
          estado = LDR;
        }
        break;


      case RESTARLDR:
        pantallaLdr();
        if (digitalRead(BOTON2) == HIGH) {
          umbralLdr = umbralLdr - 1;
          preferences.begin("config", false);
          preferences.putInt("umbralLdr", umbralLdr);
          preferences.end();
          if (umbralLdr < 0) {
            umbralLdr = 0;
          }
          estado = LDR;
        }
        break;


      case SUMARGAS:
        pantallaGas();
        if (digitalRead(BOTON1) == HIGH) {
          umbralGas = umbralGas + 1;
          preferences.begin("config", false);
          preferences.putInt("umbralGas", umbralGas);
          preferences.end();
          if (umbralGas > 100) {
            umbralGas = 100;
          }
          estado = PANTALLA_GAS;
        }
        break;


      case RESTARGAS:
        pantallaGas();
        if (digitalRead(BOTON2) == HIGH) {
          umbralGas = umbralGas - 1;
          preferences.begin("config", false);
          preferences.putInt("umbralGas", umbralGas);
          preferences.end();
          if (umbralGas < 0) {
            umbralGas = 0;
          }
          estado = PANTALLA_GAS;
        }
        break;


      case SUMARMETANO:
        pantallaGas();
        if (digitalRead(BOTON3) == HIGH) {
          if (digitalRead(BOTON3) == HIGH) {
            umbralMetano = umbralMetano + 1;
            preferences.begin("config", false);
            preferences.putInt("umbralMetano", umbralMetano);
            preferences.end();
            if (umbralMetano > 100) {
              umbralMetano = 100;
            }
            estado = PANTALLA_GAS;
          }
          break;


        case RESTARMETANO:
          pantallaGas();
          if (digitalRead(BOTON4) == HIGH) {
            umbralMetano = umbralMetano - 1;
            preferences.begin("config", false);
            preferences.putInt("umbralMetano", umbralMetano);
            preferences.end();
            if (umbralMetano < 0) {
              umbralMetano = 0;
            }
            estado = PANTALLA_GAS;
          }
          break;


        case SUMARTEMP:
          pantallaTemp();
          if (digitalRead(BOTON1) == HIGH) {
            estado = TEMP;
            umbralTemp = umbralTemp + 1;
            preferences.begin("config", false);
            preferences.putInt("umbralTemp", umbralTemp);
            preferences.end();
          }
          break;


        case RESTARTEMP:
          pantallaTemp();
          if (digitalRead(BOTON2) == HIGH) {
            estado = TEMP;
            umbralTemp = umbralTemp - 1;
            preferences.begin("config", false);
            preferences.putInt("umbralTemp", umbralTemp);
            preferences.end();
          }
          break;


        case SUMARHUM:
          pantallaTemp();
          if (digitalRead(BOTON3) == HIGH) {
            umbralHum = umbralHum + 1;
            preferences.begin("config", false);
            preferences.putInt("umbralHum", umbralHum);
            preferences.end();
            if (umbralHum > 100) {
              umbralHum = 100;
            }
            estado = TEMP;
          }
          break;


        case RESTARHUM:
          pantallaTemp();
          if (digitalRead(BOTON4) == HIGH) {
            umbralHum = umbralHum - 1;
            preferences.begin("config", false);
            preferences.putInt("umbralHum", umbralHum);
            preferences.end();
            if (umbralHum < 0) {
              umbralHum = 0;
            }
            estado = TEMP;
          }
          break;


        case SUMARGMT:
          pantallaGMT_MQTT();
          if (digitalRead(BOTON1) == HIGH) {
            umbralGMT = umbralGMT + 1;
            preferences.begin("config", false);
            preferences.putInt("umbralGMT", umbralGMT);
            preferences.end();
            if (umbralGMT > 12) {
              umbralGMT = 12;
            }
            actualizarHoraConGMT();
            estado = GMT_MQTT;
          }
          break;


        case RESTARGMT:
          pantallaGMT_MQTT();
          if (digitalRead(BOTON2) == HIGH) {
            umbralGMT = umbralGMT - 1;
            preferences.begin("config", false);
            preferences.putInt("umbralGMT", umbralGMT);
            preferences.end();
            if (umbralGMT < -12) {
              umbralGMT = -12;
            }
            actualizarHoraConGMT();
            estado = GMT_MQTT;
          }
          break;


        case SUMARINT:
          pantallaGMT_MQTT();
          if (digitalRead(BOTON3) == HIGH) {
            estado = GMT_MQTT;
          }
          break;


        case RESTARINT:
          pantallaGMT_MQTT();
          if (digitalRead(BOTON4) == HIGH) {
            estado = GMT_MQTT;
          }
          break;
        }
    }
  }
}


void loop() {
  // Vacío
}


void pantallaGeneral(void) {
  lcd.setCursor(0, 0);
  lcd.print("Luz:1 Gas:2 TH:3");
  lcd.setCursor(0, 1);
  lcd.print("GMT/mqtt:4 back5");
}


void pantallaLdr(void) {
  lcd.setCursor(0, 0);
  lcd.print("VALOR DE LUZ");
  lcd.setCursor(0, 1);
  lcd.print("A:");
  lcd.print(ldrMap);
  lcd.print("% ");
  lcd.print("U:");
  lcd.print(umbralLdr);
  lcd.print("%");
}


void pantallaTemp(void) {
  lcd.setCursor(0, 0);
  lcd.print("TA");
  lcd.print(temp.temperature);
  lcd.print("°C ");
  lcd.print("TU:");
  lcd.print(umbralTemp);
  lcd.print("°C");
  lcd.setCursor(0, 1);
  lcd.print("HA:");
  lcd.print(humidity.relative_humidity);
  lcd.print("% ");
  lcd.print("HU:");
  lcd.print(umbralHum);
  lcd.print("%");
}


void pantallaGas(void) {
  lcd.setCursor(0, 0);
  lcd.print("GA");
  lcd.print(gasMap);
  lcd.print("% ");
  lcd.print("GU:");
  lcd.print(umbralGas);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("MA:");
  lcd.print(metanoMap);
  lcd.print("% ");
  lcd.print("MU:");
  lcd.print(umbralMetano);
  lcd.print("%");
}


void pantallaGMT_MQTT(void) {
  lcd.setCursor(0, 0);
  lcd.print("inter:");
  lcd.print(umbralIntervalo / 1000);
  lcd.print("s");
  lcd.setCursor(0, 1);
  lcd.print("GMT:");
  lcd.print(umbralGMT);
}
