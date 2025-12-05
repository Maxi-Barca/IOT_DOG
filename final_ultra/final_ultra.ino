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

// --------------------------- CONFIG WIFI / MQTT ---------------------------
const char* ssid = "MECA-IoT";
const char* password = "IoT$2026";

const uint8_t name_device_id = 18; // ID numérico del dispositivo

unsigned long now = 0;
unsigned long lastMeasure1 = 0;
unsigned long lastMeasure2 = 0;

unsigned long interval_envio = 30000UL;   // Intervalo de envío mqtt (ms)
const unsigned long interval_lectura = 60000UL; // Intervalo de lectura y guardado (ms)

long unsigned int timestamp;  // hora
const char* ntpServer = "south-america.pool.ntp.org";
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;

int indice_entra = 0;
int indice_saca = 0;
bool flag_vacio = true;

#define MQTT_HOST IPAddress(192, 168, 5, 123)
#define MQTT_PORT 1884
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[200];
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// --------------------------- ESTRUCTURA DE DATOS ---------------------------
typedef struct
{
  long time;
  float T1;
  float H1;
  float luz;
  float G1;
  float G2;
} estructura;

const int valor_max_struct = 1000;
estructura datos_struct[valor_max_struct];
estructura aux2;

// --------------------------- PROTOTIPOS ---------------------------
void connectToWifi();
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void setupmqtt();
void fun_envio_mqtt();
void fun_saca();
void fun_entra();
void sincronizarHora();
void actualizarHoraConGMT();

// --------------------------- MUTEX (protege datos compartidos) ------------
SemaphoreHandle_t xDatosMutex = NULL;

// --------------------------- RTC / LCD / SENSORES ------------------------
ESP32Time rtc;
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

Adafruit_AHTX0 aht;
sensors_event_t humidity, temp;

// --------------------------- PINES ---------------------------------------
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

// Algunos defines adicionales del menú (no afectan pines)
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
int umbralIntervalo; // para mostrar en pantalla (s)
int umbralGMT;

bool mensajeEnviadoTemp = false;
bool mensajeEnviadoHum = false;
bool mensajeEnviadoLdr = false;
bool mensajeEnviadoGas = false;
bool mensajeEnviadoMetano = false;

Preferences preferences;

// --------------------------- TAREAS -------------------------------------
TaskHandle_t Task1;
TaskHandle_t Task2;

void Task1code(void* pvParameters);
void Task2code(void* pvParameters);

// --------------------------- SETUP MQTT ---------------------------------
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
  // Extraigo un elemento de la cola (si hay)
  fun_saca();
  if (!flag_vacio) {
    Serial.print("enviando... ");

    // Protejo aux2 mientras genero el payload (aunque fun_saca ya lo copió)
    xSemaphoreTake(xDatosMutex, portMAX_DELAY);
    memset(mqtt_payload, 0, sizeof(mqtt_payload));
    // Enviamos: id, time, T1, H1, luz, G1, G2
    snprintf(mqtt_payload, sizeof(mqtt_payload),
             "%u&%ld&%.2f&%.2f&%.2f&%.2f&%.2f",
             (unsigned)name_device_id, aux2.time, aux2.T1, aux2.H1, aux2.luz, aux2.G1, aux2.G2);
    // Limpio aux2
    aux2.time = 0;
    aux2.T1 = 0;
    aux2.H1 = 0;
    aux2.luz = 0;
    aux2.G1 = 0;
    aux2.G2 = 0;
    xSemaphoreGive(xDatosMutex);

    Serial.print("Publish message: ");
    Serial.println(mqtt_payload);
    if (mqttClient.connected()) {
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);
      (void)packetIdPub1;
    } else {
      Serial.println("MQTT no conectado, no se publica");
    }
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
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
    default:
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
  Serial.print("Publish acknowledged. packetId: ");
  Serial.println(packetId);
}

// --------------------------- FUN_SACA / FUN_ENTRA -------------------------
void fun_saca() {
  xSemaphoreTake(xDatosMutex, portMAX_DELAY);
  if (indice_saca != indice_entra) {
    // Copio entrada a aux2
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.H1 = datos_struct[indice_saca].H1;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.G1 = datos_struct[indice_saca].G1;
    aux2.G2 = datos_struct[indice_saca].G2;

    flag_vacio = false;

    Serial.print("indice_saca previo: ");
    Serial.println(indice_saca);

    // incremento circular
    indice_saca++;
    if (indice_saca >= valor_max_struct) {
      indice_saca = 0;
    }

    Serial.print("saco valores de la struct, indice ahora: ");
    Serial.println(indice_saca);
  } else {
    flag_vacio = true;  // no hay datos
  }
  xSemaphoreGive(xDatosMutex);
}

void fun_entra(void) {
  xSemaphoreTake(xDatosMutex, portMAX_DELAY);
  if (indice_entra >= valor_max_struct) {
    indice_entra = 0;
  }

  // timestamp
  timestamp = time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    xSemaphoreGive(xDatosMutex);
    return;
  }
  char buftime[64];
  strftime(buftime, sizeof(buftime), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  Serial.print("> NTP Time: ");
  Serial.println(buftime);

  // Copio mediciones actuales (se asume que Task2 actualiza estas variables)
  datos_struct[indice_entra].time = timestamp;
  datos_struct[indice_entra].T1 = temp.temperature;
  datos_struct[indice_entra].H1 = humidity.relative_humidity;
  datos_struct[indice_entra].luz = ldrMap;
  datos_struct[indice_entra].G1 = gasMap;
  datos_struct[indice_entra].G2 = metanoMap;

  indice_entra++;
  if (indice_entra >= valor_max_struct) {
    indice_entra = 0;
  }

  Serial.print("ingreso valores a la struct, indice_entra: ");
  Serial.println(indice_entra);

  xSemaphoreGive(xDatosMutex);
}

// --------------------------- HORA / RTC ----------------------------------
void sincronizarHora() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
  } else {
    Serial.println("Error al sincronizar hora");
  }
}

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

// --------------------------- PANTALLAS ----------------------------------
void pantallaGeneral(void) {
  lcd.setCursor(0, 0);
  lcd.print("Luz:1 Gas:2 TH:3   ");
  lcd.setCursor(0, 1);
  lcd.print("GMT/mqtt:4 back5  ");
}

void pantallaLdr(void) {
  lcd.setCursor(0, 0);
  lcd.print("VALOR DE LUZ      ");
  lcd.setCursor(0, 1);
  lcd.print("A:");
  lcd.print(ldrMap);
  lcd.print("% ");
  lcd.print("U:");
  lcd.print(umbralLdr);
  lcd.print("%  ");
}

void pantallaTemp(void) {
  lcd.setCursor(0, 0);
  lcd.print("TA");
  lcd.print(temp.temperature);
  lcd.print("C ");
  lcd.print("TU:");
  lcd.print(umbralTemp);
  lcd.print("C ");
  lcd.setCursor(0, 1);
  lcd.print("HA:");
  lcd.print(humidity.relative_humidity);
  lcd.print("% ");
  lcd.print("HU:");
  lcd.print(umbralHum);
  lcd.print("% ");
}

void pantallaGas(void) {
  lcd.setCursor(0, 0);
  lcd.print("GA");
  lcd.print(gasMap);
  lcd.print("% ");
  lcd.print("GU:");
  lcd.print(umbralGas);
  lcd.print("%  ");
  lcd.setCursor(0, 1);
  lcd.print("MA:");
  lcd.print(metanoMap);
  lcd.print("% ");
  lcd.print("MU:");
  lcd.print(umbralMetano);
  lcd.print("%  ");
}

void pantallaGMT_MQTT(void) {
  lcd.setCursor(0, 0);
  lcd.print("inter:");
  lcd.print(umbralIntervalo / 1000);
  lcd.print("s    ");
  lcd.setCursor(0, 1);
  lcd.print("GMT:");
  lcd.print(umbralGMT);
  lcd.print("     ");
}

// --------------------------- SETUP --------------------------------------
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  // creo mutex
  xDatosMutex = xSemaphoreCreateMutex();
  if (xDatosMutex == NULL) {
    Serial.println("ERROR: no se pudo crear mutex");
    while (1) delay(1000);
  }

  setupmqtt();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  if (!aht.begin(&Wire)) {
    Serial.println("No se encontró AHT10/AHT20, revisa conexiones!");
    while (1) delay(1000);
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
  interval_envio = (unsigned long)preferences.getInt("intervalo", (int)interval_envio);
  umbralGMT = preferences.getInt("umbralGMT", -3);
  preferences.end();

  umbralIntervalo = interval_envio;

  int offset = umbralGMT * 3600;
  configTime(offset, 0, ntpServer);
  sincronizarHora();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Hola ESP32!");
  lcd.setCursor(0, 1);
  lcd.print("LCD 16x2 prueba");

  // Creo tareas
  xTaskCreatePinnedToCore(
    Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(
    Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  delay(500);
}

// --------------------------- TASK1: envio mqtt + guardado -----------------
void Task1code(void* pvParameters) {
  for (;;) {
    now = millis();
    if (now - lastMeasure1 > interval_envio) {
      lastMeasure1 = now;
      fun_envio_mqtt();
    }
    if (now - lastMeasure2 > interval_lectura) {
      lastMeasure2 = now;
      fun_entra();
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // aligerar la CPU
  }
}

// --------------------------- TASK2: lecturas + UI -------------------------
void Task2code(void* pvParameters) {
  intervalo = 30000;
  for (;;) {
    millis_actual = millis();
    if (millis_actual - millis_aht >= intervalo) {
      // leo sensor AHT y analógicos
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

    if (temp.temperature >= umbralTemp) alerta = true;
    if (temp.temperature >= umbralTemp * 1.3) peligro = true;
    if (humidity.relative_humidity >= umbralHum) alerta = true;
    if (humidity.relative_humidity >= umbralHum * 1.3) peligro = true;
    if (gasMap >= umbralGas) alerta = true;
    if (gasMap >= umbralGas * 1.3) peligro = true;
    if (metanoMap >= umbralMetano) alerta = true;
    if (metanoMap >= umbralMetano * 1.3) peligro = true;
    if (ldrMap >= umbralLdr) alerta = true;
    if (ldrMap >= umbralLdr * 1.3) peligro = true;

    if (peligro) {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, HIGH);
    } else if (alerta) {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, LOW);
    } else {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
    }

    // MANEJO DE MENU / BOTONES (con debounce muy básico: esperamos que el
    // botón vuelva a HIGH en estados ESPERAx)
    switch (estado) {
      case P1:
        pantallaGeneral();
        if (digitalRead(BOTON1) == LOW) estado = ESPERA2;
        if (digitalRead(BOTON2) == LOW) estado = ESPERA4;
        if (digitalRead(BOTON3) == LOW) estado = ESPERA6;
        if (digitalRead(BOTON4) == LOW) estado = ESPERA8;
        break;

      case ESPERA2:
        lcd.clear();
        if (digitalRead(BOTON1) == HIGH) estado = LDR;
        break;

      case ESPERA4:
        lcd.clear();
        if (digitalRead(BOTON2) == HIGH) estado = PANTALLA_GAS;
        break;

      case ESPERA6:
        lcd.clear();
        if (digitalRead(BOTON3) == HIGH) estado = TEMP;
        break;

      case ESPERA8:
        lcd.clear();
        if (digitalRead(BOTON4) == HIGH) estado = GMT_MQTT;
        break;

      case LDR:
        pantallaLdr();
        if (digitalRead(BOTON1) == LOW) estado = SUMARLDR;
        if (digitalRead(BOTON2) == LOW) estado = RESTARLDR;
        if (digitalRead(BOTON5) == LOW) estado = ESPERA1;
        break;

      case PANTALLA_GAS:
        pantallaGas();
        if (digitalRead(BOTON1) == LOW) estado = SUMARGAS;
        if (digitalRead(BOTON2) == LOW) estado = RESTARGAS;
        if (digitalRead(BOTON3) == LOW) estado = SUMARMETANO;
        if (digitalRead(BOTON4) == LOW) estado = RESTARMETANO;
        if (digitalRead(BOTON5) == LOW) estado = ESPERA1;
        break;

      case TEMP:
        pantallaTemp();
        if (digitalRead(BOTON1) == LOW) estado = SUMARTEMP;
        if (digitalRead(BOTON2) == LOW) estado = RESTARTEMP;
        if (digitalRead(BOTON3) == LOW) estado = SUMARHUM;
        if (digitalRead(BOTON4) == LOW) estado = RESTARHUM;
        if (digitalRead(BOTON5) == LOW) estado = ESPERA1;
        break;

      case GMT_MQTT:
        pantallaGMT_MQTT();
        if (digitalRead(BOTON1) == LOW) estado = SUMARGMT;
        if (digitalRead(BOTON2) == LOW) estado = RESTARGMT;
        if (digitalRead(BOTON3) == LOW) estado = SUMARINT;
        if (digitalRead(BOTON4) == LOW) estado = RESTARINT;
        if (digitalRead(BOTON5) == LOW) estado = ESPERA1;
        break;

      case ESPERA1:
        lcd.clear();
        if (digitalRead(BOTON5) == HIGH) estado = P1;
        break;

      case SUMARLDR:
        pantallaLdr();
        if (digitalRead(BOTON1) == HIGH) {
          umbralLdr++;
          if (umbralLdr > 100) umbralLdr = 100;
          preferences.begin("config", false);
          preferences.putInt("umbralLdr", umbralLdr);
          preferences.end();
          estado = LDR;
        }
        break;

      case RESTARLDR:
        pantallaLdr();
        if (digitalRead(BOTON2) == HIGH) {
          umbralLdr--;
          if (umbralLdr < 0) umbralLdr = 0;
          preferences.begin("config", false);
          preferences.putInt("umbralLdr", umbralLdr);
          preferences.end();
          estado = LDR;
        }
        break;

      case SUMARGAS:
        pantallaGas();
        if (digitalRead(BOTON1) == HIGH) {
          umbralGas++;
          if (umbralGas > 100) umbralGas = 100;
          preferences.begin("config", false);
          preferences.putInt("umbralGas", umbralGas);
          preferences.end();
          estado = PANTALLA_GAS;
        }
        break;

      case RESTARGAS:
        pantallaGas();
        if (digitalRead(BOTON2) == HIGH) {
          umbralGas--;
          if (umbralGas < 0) umbralGas = 0;
          preferences.begin("config", false);
          preferences.putInt("umbralGas", umbralGas);
          preferences.end();
          estado = PANTALLA_GAS;
        }
        break;

      case SUMARMETANO:
        pantallaGas();
        if (digitalRead(BOTON3) == HIGH) {
          umbralMetano++;
          if (umbralMetano > 100) umbralMetano = 100;
          preferences.begin("config", false);
          preferences.putInt("umbralMetano", umbralMetano);
          preferences.end();
          estado = PANTALLA_GAS;
        }
        break;

      case RESTARMETANO:
        pantallaGas();
        if (digitalRead(BOTON4) == HIGH) {
          umbralMetano--;
          if (umbralMetano < 0) umbralMetano = 0;
          preferences.begin("config", false);
          preferences.putInt("umbralMetano", umbralMetano);
          preferences.end();
          estado = PANTALLA_GAS;
        }
        break;

      case SUMARTEMP:
        pantallaTemp();
        if (digitalRead(BOTON1) == HIGH) {
          umbralTemp++;
          preferences.begin("config", false);
          preferences.putInt("umbralTemp", umbralTemp);
          preferences.end();
          estado = TEMP;
        }
        break;

      case RESTARTEMP:
        pantallaTemp();
        if (digitalRead(BOTON2) == HIGH) {
          umbralTemp--;
          preferences.begin("config", false);
          preferences.putInt("umbralTemp", umbralTemp);
          preferences.end();
          estado = TEMP;
        }
        break;

      case SUMARHUM:
        pantallaTemp();
        if (digitalRead(BOTON3) == HIGH) {
          umbralHum++;
          if (umbralHum > 100) umbralHum = 100;
          preferences.begin("config", false);
          preferences.putInt("umbralHum", umbralHum);
          preferences.end();
          estado = TEMP;
        }
        break;

      case RESTARHUM:
        pantallaTemp();
        if (digitalRead(BOTON4) == HIGH) {
          umbralHum--;
          if (umbralHum < 0) umbralHum = 0;
          preferences.begin("config", false);
          preferences.putInt("umbralHum", umbralHum);
          preferences.end();
          estado = TEMP;
        }
        break;

      case SUMARGMT:
        pantallaGMT_MQTT();
        if (digitalRead(BOTON1) == HIGH) {
          umbralGMT++;
          if (umbralGMT > 12) umbralGMT = 12;
          preferences.begin("config", false);
          preferences.putInt("umbralGMT", umbralGMT);
          preferences.end();
          actualizarHoraConGMT();
          estado = GMT_MQTT;
        }
        break;

      case RESTARGMT:
        pantallaGMT_MQTT();
        if (digitalRead(BOTON2) == HIGH) {
          umbralGMT--;
          if (umbralGMT < -12) umbralGMT = -12;
          preferences.begin("config", false);
          preferences.putInt("umbralGMT", umbralGMT);
          preferences.end();
          actualizarHoraConGMT();
          estado = GMT_MQTT;
        }
        break;

      case SUMARINT:
        pantallaGMT_MQTT();
        if (digitalRead(BOTON3) == HIGH) {
          interval_envio += 10000;
          if (interval_envio < 10000) interval_envio = 10000;
          preferences.begin("config", false);
          preferences.putInt("intervalo", (int)interval_envio);
          preferences.end();
          umbralIntervalo = interval_envio;
          estado = GMT_MQTT;
        }
        break;

      case RESTARINT:
        pantallaGMT_MQTT();
        if (digitalRead(BOTON4) == HIGH) {
          if (interval_envio > 10000) interval_envio -= 10000;
          if (interval_envio < 10000) interval_envio = 10000;
          preferences.begin("config", false);
          preferences.putInt("intervalo", (int)interval_envio);
          preferences.end();
          umbralIntervalo = interval_envio;
          estado = GMT_MQTT;
        }
        break;

      default:
        estado = P1;
        break;
    } // switch

    vTaskDelay(pdMS_TO_TICKS(100)); // pequeño delay en el loop de Task2
  } // for
}

// --------------------------- LOOP VACIO ----------------------------------
void loop() {
  // todo el trabajo lo hacen las tasks
  delay(1000);
}
