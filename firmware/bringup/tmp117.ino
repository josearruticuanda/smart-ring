#include <bluefruit.h>
#include <Wire.h>
#include <SEGGER_RTT.h>

#define SDA_PIN 26
#define SCL_PIN 27

#define TMP117_ADDR       0x48
#define TMP117_REG_TEMP   0x00
#define TMP117_RESOLUTION 0.0078125f

bool readTemperature(float &tempC) {
  // Step 1: point to temperature register, then release bus (STOP)
  Wire.beginTransmission(TMP117_ADDR);
  Wire.write(TMP117_REG_TEMP);
  byte err = Wire.endTransmission(true);  // true = send STOP
  if (err != 0) {
    SEGGER_RTT_printf(0, "endTransmission error: %d\n", err);
    return false;
  }

  // Step 2: separate read transaction
  uint8_t n = Wire.requestFrom(TMP117_ADDR, (uint8_t)2);
  if (n != 2) {
    SEGGER_RTT_printf(0, "requestFrom returned: %d\n", n);
    return false;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  int16_t raw = (int16_t)((msb << 8) | lsb);
  tempC = raw * TMP117_RESOLUTION;

  return true;
}

void setup() {
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "TMP117 reader starting...\n");

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
}

void loop() {
  float tempC;

  if (readTemperature(tempC)) {
    int whole = (int)tempC;
    int frac = (int)((tempC - whole) * 1000);
    if (frac < 0) frac = -frac;
    SEGGER_RTT_printf(0, "Temperature: %d.%03d C\n", whole, frac);
  } else {
    SEGGER_RTT_WriteString(0, "Error reading TMP117\n");
  }

  delay(1000);
}