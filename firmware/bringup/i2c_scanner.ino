#include <bluefruit.h>
#include <Wire.h>
#include <SEGGER_RTT.h>

#define SDA_PIN 26
#define SCL_PIN 27

void setup() {
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "I2C Scanner starting...\n");

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();

  int deviceCount = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      SEGGER_RTT_printf(0, "I2C device found at address: 0x%02X\n", address);
      deviceCount++;
    }
  }

  if (deviceCount == 0)
    SEGGER_RTT_WriteString(0, "No I2C devices found.\n");
  else
    SEGGER_RTT_printf(0, "Scan complete. %d device(s) found.\n", deviceCount);
}

void loop() {}