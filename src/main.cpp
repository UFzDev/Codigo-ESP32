#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Pines I2C usados en la ESP32.
const uint8_t PIN_SDA = 21;
const uint8_t PIN_SCL = 22;

// Direccion I2C del ADXL345 cuando SDO esta a GND.
const uint8_t ADXL_ADDR = 0x53;

// Registros principales del ADXL345
const uint8_t REG_POWER_CTL = 0x2D;
const uint8_t REG_BW_RATE = 0x2C;
const uint8_t REG_DATA_FORMAT = 0x31;
const uint8_t REG_DATAX0 = 0x32;

// Parametros de adquisicion y post-procesado.
const uint32_t SAMPLE_PERIOD_MS = 10;      // 100 Hz
const float HPF_CUTOFF_HZ = 0.7f;          // Quita gravedad/deriva lenta.
const uint8_t PRINT_EVERY_N_SAMPLES = 10;  // 10 Hz por Serial.
const float AXIS_DEADBAND_G = 0.006f;

// Estado interno del filtro pasa-altas por eje.
struct HighPassState {
  float prevInput = 0.0f;
  float prevOutput = 0.0f;
  bool initialized = false;
};

// Estado global del DSP y del planificador de muestreo.
HighPassState g_hpfX, g_hpfY, g_hpfZ;
uint32_t g_nextSampleMs = 0;
uint8_t g_samplesSincePrint = 0;

// Escribe un registro del sensor por I2C.
bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

// Lee 6 bytes consecutivos (X, Y, Z) desde DATAX0.
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

// Filtro pasa-altas de 1er orden para aislar vibracion.
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

// Deadband: fuerza a cero valores pequenos para limpiar ruido.
float deadband(float v, float db) {
  return (fabsf(v) < db) ? 0.0f : v;
}

// Reinicia estados de los filtros cuando inicia el sistema.
void resetFilterState() {
  g_hpfX = HighPassState{};
  g_hpfY = HighPassState{};
  g_hpfZ = HighPassState{};
  g_samplesSincePrint = 0;
}

void setup() {
  // Inicializacion fisica del bus I2C en los pines definidos.
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  // Consola serie para monitoreo.
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nADXL345 fijo en I2C (SDA=21, SCL=22, addr=0x53)");

  // Configuracion del sensor.
  if (!writeReg(REG_POWER_CTL, 0x00) ||  // standby
      !writeReg(REG_BW_RATE, 0x0A) ||    // 100 Hz
      !writeReg(REG_DATA_FORMAT, 0x08) ||// full-resolution, +/-2g
      !writeReg(REG_POWER_CTL, 0x08)) {  // measure
    Serial.println("Error al inicializar ADXL345");
  } else {
    Serial.println("ADXL345 listo. Iniciando lectura de vibracion...");
  }

  g_nextSampleMs = millis();
  resetFilterState();
}

void loop() {
  // Planificador simple para muestreo periodico a 100 Hz.
  uint32_t now = millis();
  if ((int32_t)(now - g_nextSampleMs) < 0) {
    return;
  }
  g_nextSampleMs += SAMPLE_PERIOD_MS;

  // Lectura cruda de aceleracion.
  int16_t xRaw = 0, yRaw = 0, zRaw = 0;
  if (!readXYZ(xRaw, yRaw, zRaw)) {
    Serial.println("Fallo lectura XYZ");
    delay(50);
    return;
  }

  // Conversion de cuentas del ADC interno del sensor a g.
  const float xG = xRaw * 0.0039f;
  const float yG = yRaw * 0.0039f;
  const float zG = zRaw * 0.0039f;

  // Calculo del coeficiente del filtro pasa-altas segun dt y fc.
  const float dt = SAMPLE_PERIOD_MS / 1000.0f;
  const float rc = 1.0f / (2.0f * PI * HPF_CUTOFF_HZ);
  const float alpha = rc / (rc + dt);

  // Filtrado + deadband para obtener vibracion limpia por eje.
  const float xV = deadband(highPassFilter(g_hpfX, xG, alpha), AXIS_DEADBAND_G);
  const float yV = deadband(highPassFilter(g_hpfY, yG, alpha), AXIS_DEADBAND_G);
  const float zV = deadband(highPassFilter(g_hpfZ, zG, alpha), AXIS_DEADBAND_G);

  // Se imprime a 10 Hz para no saturar el puerto serie.
  if (++g_samplesSincePrint >= PRINT_EVERY_N_SAMPLES) {
    g_samplesSincePrint = 0;
    Serial.print("X: ");
    Serial.print(xV, 3);
    Serial.print(" | Y: ");
    Serial.print(yV, 3);
    Serial.print(" | Z: ");
    Serial.print(zV, 3);
  }
}