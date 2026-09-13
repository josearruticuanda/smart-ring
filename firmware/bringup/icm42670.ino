#include <bluefruit.h>
#include <Wire.h>
#include <SEGGER_RTT.h>

#define SDA_PIN   26
#define SCL_PIN   27
#define INT1_PIN  31   // P0.31 / AIN7

#define ICM_ADDR            0x68

#define REG_SIGNAL_PATH_RST 0x02
#define REG_INT_CONFIG      0x06
#define REG_TEMP_DATA1      0x09
#define REG_PWR_MGMT0       0x1F
#define REG_GYRO_CONFIG0    0x20
#define REG_ACCEL_CONFIG0   0x21
#define REG_INT_SOURCE0     0x2B
#define REG_INT_STATUS_DRDY 0x39
#define REG_WHO_AM_I        0x75

// ±2000 dps, ±16 g (the FS ranges we configure below)
const float GYRO_SENS_LSB_PER_DPS = 16.4f;
const float ACCEL_SENS_LSB_PER_G  = 2048.0f;

volatile bool dataReadyFlag = false;

void drdy_isr() {
  dataReadyFlag = true;
}

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

bool readRegs(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(true) != 0) return false;

  uint8_t n = Wire.requestFrom(ICM_ADDR, len);
  if (n != len) return false;

  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void setup() {
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "ICM-42670-P starting...\n");

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();

  // Soft reset, then give it time to come back up
  writeReg(REG_SIGNAL_PATH_RST, 0x10);  // SOFT_RESET_DEVICE_CONFIG = 1
  delay(2);

  // Sanity check
  uint8_t whoami;
  readRegs(REG_WHO_AM_I, &whoami, 1);
  SEGGER_RTT_printf(0, "WHO_AM_I: 0x%02X (expect 0x67)\n", whoami);

  // GYRO_CONFIG0: FS=00 (±2000 dps), ODR=1001 (100 Hz) -> 0x09
  writeReg(REG_GYRO_CONFIG0, 0x09);
  // ACCEL_CONFIG0: FS=00 (±16 g), ODR=1001 (100 Hz) -> 0x09
  writeReg(REG_ACCEL_CONFIG0, 0x09);

  // PWR_MGMT0: GYRO_MODE=11 (LN), ACCEL_MODE=11 (LN) -> 0x0F
  writeReg(REG_PWR_MGMT0, 0x0F);
  delay(1);     // datasheet: don't write registers for 200 us after enabling
  delay(45);    // datasheet: gyro needs to stay ON >=45 ms before it's valid

  // INT1: push-pull, active-high, pulsed mode -> bits [drive=1][pol=1] = 0x03
  writeReg(REG_INT_CONFIG, 0x03);
  // Route Data-Ready to INT1 (keep RESET_DONE_INT1_EN default bit set too)
  writeReg(REG_INT_SOURCE0, 0x18);  // RESET_DONE_INT1_EN | DRDY_INT1_EN

  pinMode(INT1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), drdy_isr, RISING);

  SEGGER_RTT_WriteString(0, "Setup complete.\n");
}

void loop() {
  if (dataReadyFlag) {
    dataReadyFlag = false;

    // Clear the DRDY status bit by reading it
    uint8_t status;
    readRegs(REG_INT_STATUS_DRDY, &status, 1);

    uint8_t raw[14];
    if (!readRegs(REG_TEMP_DATA1, raw, 14)) {
      SEGGER_RTT_WriteString(0, "Burst read failed\n");
      return;
    }

    int16_t temp_raw  = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t accel_x   = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t accel_y   = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t accel_z   = (int16_t)((raw[6]  << 8) | raw[7]);
    int16_t gyro_x    = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gyro_y    = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gyro_z    = (int16_t)((raw[12] << 8) | raw[13]);

    float tempC  = (temp_raw / 128.0f) + 25.0f;
    float ax_g   = accel_x / ACCEL_SENS_LSB_PER_G;
    float ay_g   = accel_y / ACCEL_SENS_LSB_PER_G;
    float az_g   = accel_z / ACCEL_SENS_LSB_PER_G;
    float gx_dps = gyro_x / GYRO_SENS_LSB_PER_DPS;
    float gy_dps = gyro_y / GYRO_SENS_LSB_PER_DPS;
    float gz_dps = gyro_z / GYRO_SENS_LSB_PER_DPS;

    // RTT printf has flaky float support on some builds, so split manually
    auto p3 = [](float v) -> int { return (int)(v * 1000); };

    SEGGER_RTT_printf(0, "T:%d.%03dC  A(g):%d.%03d,%d.%03d,%d.%03d  G(dps):%d.%03d,%d.%03d,%d.%03d\n",
      (int)tempC, abs(p3(tempC) % 1000),
      (int)ax_g, abs(p3(ax_g) % 1000),
      (int)ay_g, abs(p3(ay_g) % 1000),
      (int)az_g, abs(p3(az_g) % 1000),
      (int)gx_dps, abs(p3(gx_dps) % 1000),
      (int)gy_dps, abs(p3(gy_dps) % 1000),
      (int)gz_dps, abs(p3(gz_dps) % 1000));
  }
}