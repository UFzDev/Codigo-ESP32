#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <math.h>

// Pines I2C del ESP32.
const uint8_t PIN_SDA = 21;
const uint8_t PIN_SCL = 22;

// Direccion I2C del ADXL345 (SDO a GND).
const uint8_t ADXL_ADDR = 0x53;

// Registros principales del ADXL345
const uint8_t REG_POWER_CTL = 0x2D;
const uint8_t REG_BW_RATE = 0x2C;
const uint8_t REG_DATA_FORMAT = 0x31;
const uint8_t REG_DATAX0 = 0x32;

// WiFi en modo punto de acceso para mandar datos sin cable.
const char *WIFI_AP_SSID = "ESP32_DIPREMO";
const char *WIFI_AP_PASS = "12345678";  // Minimo 8 caracteres.
const uint16_t WIFI_WS_PORT = 81;

// Identidad del equipo.
const char *DEVICE_ID = "DIPREMO-001";

// Ajustes de lectura y filtro.
const uint32_t SAMPLE_PERIOD_MS = 10;  // 100 Hz
const float SAMPLE_RATE_HZ = 1000.0f / SAMPLE_PERIOD_MS;

// Variables globales de tiempo y conteo.
uint32_t g_nextSampleMs = 0;
uint32_t g_sampleId = 0;
uint32_t g_prevSampleUs = 0;
uint32_t g_bootId = 0;
uint32_t g_i2cErrorCount = 0;
WebSocketsServer g_wsServer(WIFI_WS_PORT);

struct VibrationData {
  // Muestras crudas por eje.
  int16_t rawX = 0;
  int16_t rawY = 0;
  int16_t rawZ = 0;
};

bool writeReg(uint8_t reg, uint8_t value);
bool readXYZ(int16_t &x, int16_t &y, int16_t &z);
void publishLine(const char *line);
void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t *payload, size_t length);

// Crea una red WiFi desde el ESP32 y abre un puerto TCP.
void setupWiFiAp() {  
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS)) {
    Serial.println("Error al iniciar AP WiFi");
    return;
  }

  const IPAddress ip = WiFi.softAPIP();
  // Datos de conexion para la PC.
  Serial.print("AP WiFi listo. SSID: ");
  Serial.print(WIFI_AP_SSID);
  Serial.print(" | PASS: ");
  Serial.print(WIFI_AP_PASS);
  Serial.print(" | IP ESP32: ");
  Serial.println(ip);

  g_wsServer.begin();
  g_wsServer.onEvent(onWebSocketEvent);
  // URL para cliente web: ws://192.168.4.1:81
  Serial.print("Servidor WebSocket activo en puerto ");
  Serial.println(WIFI_WS_PORT);
}

// Envia una linea por Serial y por WiFi.
void publishLine(const char *line) {
  Serial.println(line);
  g_wsServer.broadcastTXT((uint8_t *)line, strlen(line));
}

// Log basico de conexiones WebSocket.
void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t *payload, size_t length) {
  (void)payload;
  (void)length;

  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = g_wsServer.remoteIP(clientNum);
      Serial.printf("Cliente WS #%u conectado desde %u.%u.%u.%u\n",
                    clientNum,
                    ip[0], ip[1], ip[2], ip[3]);
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("Cliente WS #%u desconectado\n", clientNum);
      break;
    default:
      break;
  }
}

// Escribe un dato en un registro del sensor.
bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

// Lee X, Y y Z del acelerometro.
bool readXYZ(int16_t &x, int16_t &y, int16_t &z) {
  // El ADXL345 entrega 6 bytes: X, Y, Z.
  uint8_t d[6] = {0};
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((int)ADXL_ADDR, 6, (int)true) != 6) {
    return false;
  }
  for (uint8_t i = 0; i < 6; i++) {
    d[i] = Wire.read();
  }
  x = (int16_t)((d[1] << 8) | d[0]);
  y = (int16_t)((d[3] << 8) | d[2]);
  z = (int16_t)((d[5] << 8) | d[4]);
  return true;
}

// Reinicia estado de tiempo y contador.
void resetRuntimeState() {
  g_sampleId = 0;
  g_i2cErrorCount = 0;
  g_prevSampleUs = micros();
}

bool configureAdxl345() {
  // Configuracion basica para medir a 100 Hz.
  return writeReg(REG_POWER_CTL, 0x00) &&   // Modo standby significa que se pueden configurar otros registros.
         writeReg(REG_BW_RATE, 0x0A) &&     // 100 Hz
         writeReg(REG_DATA_FORMAT, 0x08) && // full-resolution, +/-2g
         writeReg(REG_POWER_CTL, 0x08);     // Modo medida, empieza a tomar muestras. Este es el ultimo paso, para evitar configuraciones intermedias invalidas.
}

bool acquireVibrationData(VibrationData &data) {
  return readXYZ(data.rawX, data.rawY, data.rawZ);
}

float computeDeltaMs() {
  // Tiempo real entre muestra y muestra.
  const uint32_t nowUs = micros();
  const float deltaMs = (nowUs - g_prevSampleUs) / 1000.0f;
  g_prevSampleUs = nowUs;
  return deltaMs;
}

void buildJsonLine(const VibrationData &data, float deltaMs, char *out, size_t outSize) {
  // Diagnostico interno de la placa.
  const float tempC = temperatureRead();
  const int32_t rssiDbm = WiFi.RSSI();

  // Paquete JSON que envia la ESP32.
  snprintf(out,
           outSize,
           "{\"device_id\":\"%s\",\"boot_id\":%lu,\"sample_id\":%lu,\"sample_rate_hz\":%.1f,\"uptime_ms\":%lu,\"delta_ms\":%.3f,\"raw\":{\"x\":%d,\"y\":%d,\"z\":%d},\"diag\":{\"temp_c\":%.2f,\"free_heap\":%lu,\"rssi_dbm\":%ld,\"i2c_error_count\":%lu}}",
           DEVICE_ID,
           (unsigned long)g_bootId,
           (unsigned long)g_sampleId,
           SAMPLE_RATE_HZ,
           (unsigned long)millis(),
           deltaMs,
           data.rawX,
           data.rawY,
           data.rawZ,
           tempC,
           (unsigned long)ESP.getFreeHeap(),
           (long)rssiDbm,
           (unsigned long)g_i2cErrorCount);
}

void setup() {
  // Inicia I2C en los pines elegidos.
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  // Inicia Serial para ver mensajes.
  Serial.begin(115200);
  delay(1000);
  g_bootId = esp_random();
  // ID unico por cada reinicio.
  Serial.println("\nADXL345 fijo en I2C (SDA=21, SCL=22, addr=0x53)");
  Serial.print("Boot ID: ");
  Serial.println(g_bootId);

  // Configura el ADXL345.
  if (!configureAdxl345()) {
    Serial.println("Error al inicializar ADXL345");
  } else {
    Serial.println("ADXL345 listo. Iniciando lectura de vibracion...");
  }

  setupWiFiAp();

  g_nextSampleMs = millis();
  resetRuntimeState();
}

void loop() {
  // Procesa handshakes y trafico WebSocket.
  g_wsServer.loop();

  // Lee el sensor cada 10 ms (100 Hz).
  uint32_t now = millis();
  if ((int32_t)(now - g_nextSampleMs) < 0) {
    return;
  }
  g_nextSampleMs += SAMPLE_PERIOD_MS;

  VibrationData data;
  // Si falla I2C, suma error y continua.
  if (!acquireVibrationData(data)) {
    g_i2cErrorCount++;
    Serial.println("Fallo lectura XYZ");
    delay(50);
    return;
  }

  const float deltaMs = computeDeltaMs();
  // Contador global de muestras enviadas.
  g_sampleId++;

  // Envia todas las muestras (100 Hz).
  char line[320] = {0};
  buildJsonLine(data, deltaMs, line, sizeof(line));
  publishLine(line);
}