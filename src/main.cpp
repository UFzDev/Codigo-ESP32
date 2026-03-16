#include <Arduino.h>
#include <Wire.h>

const uint8_t PIN_SDA = 21;
const uint8_t PIN_SCL = 22;
const uint8_t ADXL_ADDR_LOW = 0x53;  // SDO to GND
const uint8_t ADXL_ADDR_HIGH = 0x1D; // SDO to 3V3
const uint8_t REG_DEVID = 0x00;
const uint8_t EXPECTED_DEVID = 0xE5;

uint8_t probeAck(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true);
}

void scanBus() {
  int found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    uint8_t err = probeAck(address);
    if (err == 0) {
      Serial.print("  I2C ACK en 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("  Escaneo: no hubo ACK en ningun address.");
  }
}

bool readRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  // Use STOP instead of repeated-start to avoid non-stop transaction issues.
  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  uint8_t readCount = Wire.requestFrom((int)address, 1);
  if (readCount != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

void probeAddress(uint8_t address) {
  uint8_t devid = 0;

  Serial.print("Probando ADXL345 en 0x");
  if (address < 16) {
    Serial.print("0");
  }
  Serial.println(address, HEX);

  uint8_t ack = probeAck(address);
  Serial.print("  ACK check (endTransmission): ");
  Serial.println(ack);

  if (ack != 0) {
    Serial.println("  Sin ACK: revisar VCC/CS/SDO/SDA/SCL/pull-ups.");
    return;
  }

  if (!readRegister(address, REG_DEVID, devid)) {
    Serial.println("  No responde lectura I2C.");
    return;
  }

  Serial.print("  DEVID leido: 0x");
  if (devid < 16) {
    Serial.print("0");
  }
  Serial.println(devid, HEX);

  if (devid == EXPECTED_DEVID) {
    Serial.println("  OK: ADXL345 detectado.");
  } else {
    Serial.println("  Responde I2C, pero no parece ADXL345.");
  }
}

void setup() {
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nDiagnostico ADXL345 por I2C");
  Serial.println("SDA=21, SCL=22, velocidad=100kHz");
  Serial.println("Si ACK!=0 en todo, casi seguro es cableado/alimentacion.");
}

void loop() {
  Serial.println("------------------------------");
  Serial.println("Escaneo completo de bus I2C:");
  scanBus();
  probeAddress(ADXL_ADDR_LOW);
  probeAddress(ADXL_ADDR_HIGH);
  Serial.println("Repite en 3 segundos...\n");
  delay(3000);
}