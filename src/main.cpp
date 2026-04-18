#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
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
const uint16_t WIFI_TCP_PORT = 3333;

// Ajustes de lectura y filtro.
const uint32_t SAMPLE_PERIOD_MS = 10;      // 100 Hz
const float HPF_CUTOFF_HZ = 0.7f;          // Quita gravedad/deriva lenta.
const uint8_t PRINT_EVERY_N_SAMPLES = 10;  // 10 Hz por Serial.
const float AXIS_DEADBAND_G = 0.006f;

// Estado del filtro por cada eje.
struct HighPassState {
  float prevInput = 0.0f;
  float prevOutput = 0.0f;
  bool initialized = false;
};

// Variables globales del filtro y del tiempo.
HighPassState g_hpfX, g_hpfY, g_hpfZ;
uint32_t g_nextSampleMs = 0;
uint8_t g_samplesSincePrint = 0;
WiFiServer g_tcpServer(WIFI_TCP_PORT);
WiFiClient g_tcpClient;

// Crea una red WiFi desde el ESP32 y abre un puerto TCP.
void setupWiFiAp() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS)) {
    Serial.println("Error al iniciar AP WiFi");
    return;
  }

  const IPAddress ip = WiFi.softAPIP();
  Serial.print("AP WiFi listo. SSID: ");
  Serial.print(WIFI_AP_SSID);
  Serial.print(" | PASS: ");
  Serial.print(WIFI_AP_PASS);
  Serial.print(" | IP ESP32: ");
  Serial.println(ip);

  g_tcpServer.begin();
  g_tcpServer.setNoDelay(true);
  Serial.print("Servidor TCP activo en puerto ");
  Serial.println(WIFI_TCP_PORT);
}

// Mantiene un cliente TCP conectado (solo uno a la vez).
void handleTcpClient() {
  if (g_tcpClient && g_tcpClient.connected()) {
    return;
  }

  if (g_tcpClient) {
    g_tcpClient.stop();
  }

  WiFiClient candidate = g_tcpServer.available();
  if (candidate) {
    g_tcpClient = candidate;
    Serial.println("Cliente TCP conectado");
  }
}

// Envia una linea por Serial y por WiFi.
void publishLine(const char *line) {
  Serial.println(line);
  if (g_tcpClient && g_tcpClient.connected()) {
    g_tcpClient.println(line);
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

// Filtro pasa-altas para dejar solo vibracion.
float highPassFilter(HighPassState &s, float input, float alpha) {
  if (!s.initialized) {
    s.prevInput = input;
    s.prevOutput = 0.0f;
    s.initialized = true;
    return 0.0f;
  }
  const float out = alpha * (s.prevOutput + input - s.prevInput);
  s.prevInput = input;
  s.prevOutput = out;
  return out;
}

// Si el valor es muy pequeno, lo deja en cero.
float deadband(float v, float db) {
  return (fabsf(v) < db) ? 0.0f : v;
}

// Reinicia el estado de los filtros.
void resetFilterState() {
  g_hpfX = HighPassState{};
  g_hpfY = HighPassState{};
  g_hpfZ = HighPassState{};
  g_samplesSincePrint = 0;
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
  Serial.println("\nADXL345 fijo en I2C (SDA=21, SCL=22, addr=0x53)");

  // Configura el ADXL345.
  if (!writeReg(REG_POWER_CTL, 0x00) ||  // standby
      !writeReg(REG_BW_RATE, 0x0A) ||    // 100 Hz
      !writeReg(REG_DATA_FORMAT, 0x08) ||// full-resolution, +/-2g
      !writeReg(REG_POWER_CTL, 0x08)) {  // measure
    Serial.println("Error al inicializar ADXL345");
  } else {
    Serial.println("ADXL345 listo. Iniciando lectura de vibracion...");
  }

  setupWiFiAp();

  g_nextSampleMs = millis();
  resetFilterState();
}

void loop() {
  handleTcpClient();

  // Lee el sensor cada 10 ms (100 Hz).
  uint32_t now = millis();
  if ((int32_t)(now - g_nextSampleMs) < 0) {
    return;
  }
  g_nextSampleMs += SAMPLE_PERIOD_MS;

  // Lee datos crudos del sensor.
  int16_t xRaw = 0, yRaw = 0, zRaw = 0;
  if (!readXYZ(xRaw, yRaw, zRaw)) {
    Serial.println("Fallo lectura XYZ");
    delay(50);
    return;
  }

  // Convierte datos crudos a unidades de g.
  const float xG = xRaw * 0.0039f;
  const float yG = yRaw * 0.0039f;
  const float zG = zRaw * 0.0039f;

  // Calcula el factor del filtro.
  const float dt = SAMPLE_PERIOD_MS / 1000.0f;
  const float rc = 1.0f / (2.0f * PI * HPF_CUTOFF_HZ);
  const float alpha = rc / (rc + dt);

  // Filtra ruido y deja vibracion por eje.
  const float xV = deadband(highPassFilter(g_hpfX, xG, alpha), AXIS_DEADBAND_G);
  const float yV = deadband(highPassFilter(g_hpfY, yG, alpha), AXIS_DEADBAND_G);
  const float zV = deadband(highPassFilter(g_hpfZ, zG, alpha), AXIS_DEADBAND_G);

  // Envia datos 10 veces por segundo.
  if (++g_samplesSincePrint >= PRINT_EVERY_N_SAMPLES) {
    g_samplesSincePrint = 0;
    char line[96] = {0};
    snprintf(line, sizeof(line), "X: %.3f | Y: %.3f | Z: %.3f", xV, yV, zV);
    publishLine(line);
  }
}