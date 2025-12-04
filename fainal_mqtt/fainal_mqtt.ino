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
#include "AsyncMqttClient.h"
#include "time.h"
#include "Arduino.h"


const char* ssid = "MECA-IoT";
const char* password = "IoT$2026";


const char name_device = 18;
unsigned long now = millis();    ///valor actual
unsigned long lastMeasure1 = 0;  ///variable para contar el tiempo actual
unsigned long lastMeasure2 = 0;  ///variable para contar el tiempo actual

unsigned long interval_envio = 30000;  //Intervalo de envio de datos mqtt
const unsigned long interval_leeo = 60000;   //Intervalo de lectura de datos y guardado en la cola

long unsigned int timestamp;  // hora
const char* ntpServer = "south-america.pool.ntp.org";
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;

int indice_entra = 0;
int indice_saca = 0;
bool flag_vacio = 1;

#define MQTT_HOST IPAddress(10, 162, 24, 23)  ///se debe cambiar por el ip de meca o hall del 4
#define MQTT_PORT 1884
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[200];  /////
// Test MQTT Topic
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

typedef struct
{
  long time;
  float T1;  ///temp en grados
  float H1;  ///valor entre 0 y 99 // mojado es cercano al 100
  float luz;  ///valor entre 0 y 99 . si hay luz es cercano al 100 
  float G1; ///valor entre 0 y 99 
  float G2; ///valor entre 0 y 99

} estructura;

const int valor_max_struct = 1000;          ///valor vector de struct
estructura datos_struct[valor_max_struct];  ///Guardo valores hasta que lo pueda enviar
estructura aux2;

void setupmqtt() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
}

void fun_envio_mqtt() {
  fun_saca();           ////veo si hay valores nuevos
  if (flag_vacio == 0)  ////si hay los envio
  {
    Serial.print("enviando");
    ////genero el string a enviar 1. 2.   3.   4.   5.   6.   7   8  9.       1.         2.        3.      4.       5.           6.   7.     
    snprintf(mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%.2f&%.2f&%u&%u", name_device, aux2.time, aux2.T1, aux2.H1, aux2.luz, aux2.G1, aux2.G2,);  //random(10,50)
    aux2.time = 0;                                                                                                                                                   ///limpio valores
    aux2.T1 = 0;
    aux2.H1 = 0;
    aux2.luz = 0;
    aux2.G1 = 0;
    aux2.G2 = 0;

    Serial.print("Publish message: ");
    Serial.println(mqtt_payload);
    // Publishes Temperature and Humidity values
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);
  } else {
    Serial.println("no hay valores nuevos");
  }
}  

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
} 
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
} 

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
} 

void fun_saca() {
  if (indice_saca != indice_entra) {
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.H1 = datos_struct[indice_saca].H1;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.G1 = datos_struct[indice_saca].G1;
    aux2.G2 = datos_struct[indice_saca].G2;

    flag_vacio = 0;

    Serial.println(indice_saca);
    if (indice_saca >= (valor_max_struct - 1)) {
      indice_saca = 0;
    } else {
      indice_saca++;
    }
    Serial.print("saco valores de la struct isaca:");
    Serial.println(indice_saca);
  } else {
    flag_vacio = 1;  ///// no hay datos
  }
  return;
}

void fun_entra(void) {
  if (indice_entra >= valor_max_struct) {
    indice_entra = 0;  ///si llego al maximo de la cola se vuelve a cero
  }
  //////////// timestamp/////// consigo la hora
  Serial.print("> NTP Time:");
  timestamp = time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;  //// si no puede conseguir la hora
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  ///////////////////////// fin de consigo la hora
  datos_struct[indice_entra].time = timestamp;
  datos_struct[indice_entra].T1 = temp.temperature;  /// leeo los datos //aca va la funcion de cada sensor
  datos_struct[indice_entra].H1 = humidity.relative_humidity;  //// se puede pasar por un parametro valor entre 0 y 100
  datos_struct[indice_entra].luz = ldrMap;
  datos_struct[indice_entra].G1 = gasMap;
  datos_struct[indice_entra].G2 = metanoMap;

  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}

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

  setupmqtt();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  if (!aht.begin(&Wire)) {
    Serial.println("No se encontró AHT10/AHT20, revisa conexiones!");
    while (1)
      ;
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

  preferences.begin("config", false);
  umbralLdr = preferences.getInt("umbralLdr", 70);
  umbralMetano = preferences.getInt("umbralMetano", 10);
  umbralGas = preferences.getInt("umbralGas", 10);
  umbralTemp = preferences.getInt("umbralTemp", 23);
  umbralHum = preferences.getInt("umbralHum", 40);
  interval_envio = preferences.getInt("intervalo", 30000);
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
    now = millis();
  if (now - lastMeasure1 > interval_envio) {  ////envio el doble de lectura por si falla algun envio
    lastMeasure1 = now;                       /// cargo el valor actual de millis
    fun_envio_mqtt();                         ///envio los valores por mqtt
  }
  if (now - lastMeasure2 > interval_leeo) {
    lastMeasure2 = now;  /// cargo el valor actual de millis
    fun_entra();         ///ingreso los valores a la cola struct
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
            interval_envio = interval_envio + 10000;
            preferences.begin("config", false);
            preferences.putInt("intervalo", interval_envio);
            preferences.end();
            estado = GMT_MQTT;
          }
          break;


        case RESTARINT:
          pantallaGMT_MQTT();
          if (digitalRead(BOTON4) == HIGH) {
            interval_envio = interval_envio - 10000;
            preferences.begin("config", false);
            preferences.putInt("intervalo", interval_envio);
            preferences.end();
            if (interval_envio < 10000) {
              interval_envio = 10000;
            }
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