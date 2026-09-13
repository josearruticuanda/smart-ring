#include <bluefruit.h>
#include <Wire.h>
#include <SEGGER_RTT.h>

#define SDA_PIN 26
#define SCL_PIN 27

#define BMI088_ACC_ADDR   0x18
#define BMI088_GYRO_ADDR  0x68

#define ACC_PWR_CONF      0x7C
#define ACC_PWR_CTRL      0x7D
#define ACC_STATUS        0x03
#define ACC_DATA_START    0x12
#define GYRO_INT_STAT_1   0x0A
#define GYRO_DATA_START   0x02
#define TEMP_MSB_REG      0x22

#define ACC_SENSITIVITY   5460.0f
#define GYRO_SENSITIVITY  16.384f

void writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len);
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  uint8_t val;
  readRegs(addr, reg, &val, 1);
  return val;
}

bool accelReady() { return (readReg(BMI088_ACC_ADDR, ACC_STATUS) & 0x80) != 0; }
bool gyroReady()  { return (readReg(BMI088_GYRO_ADDR, GYRO_INT_STAT_1) & 0x80) != 0; }

void printVal(float v) {
  if (v < 0) { SEGGER_RTT_WriteString(0, "-"); v = -v; }
  SEGGER_RTT_printf(0, "%d.%03d", (int)v, (int)(v * 1000) % 1000);
}

void initAccel() {
  // Accel
  writeReg(BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00);
  delay(1);
  writeReg(BMI088_ACC_ADDR, ACC_PWR_CTRL, 0x04);
  delay(5);

  // Gyro - enable new data interrupt
  writeReg(BMI088_GYRO_ADDR, 0x15, 0x80);  // GYRO_INT_CTRL: enable drdy
  delay(1);
}

void setup() {
  SEGGER_RTT_Init();
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  initAccel();
  SEGGER_RTT_WriteString(0, "gx,gy,gz,ax,ay,az,t\n");
}

void loop() {
  uint8_t buf[6];
  float gx=0, gy=0, gz=0, ax=0, ay=0, az=0, tempC=0;
  static float lgx=0, lgy=0, lgz=0, lax=0, lay=0, laz=0;

  // Always read, use last known value if not ready
  if (gyroReady()) {
    readRegs(BMI088_GYRO_ADDR, GYRO_DATA_START, buf, 6);
    lgx = (int16_t)((buf[1] << 8) | buf[0]) / GYRO_SENSITIVITY;
    lgy = (int16_t)((buf[3] << 8) | buf[2]) / GYRO_SENSITIVITY;
    lgz = (int16_t)((buf[5] << 8) | buf[4]) / GYRO_SENSITIVITY;
  }
  gx = lgx; gy = lgy; gz = lgz;

  if (accelReady()) {
    readRegs(BMI088_ACC_ADDR, ACC_DATA_START, buf, 6);
    lax = (int16_t)((buf[1] << 8) | buf[0]) / ACC_SENSITIVITY;
    lay = (int16_t)((buf[3] << 8) | buf[2]) / ACC_SENSITIVITY;
    laz = (int16_t)((buf[5] << 8) | buf[4]) / ACC_SENSITIVITY;
  }
  ax = lax; ay = lay; az = laz;

  readRegs(BMI088_ACC_ADDR, TEMP_MSB_REG, buf, 2);
  int16_t tempRaw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
  if (tempRaw > 1023) tempRaw -= 2048;
  tempC = tempRaw * 0.125f + 23.0f;

  printVal(gx); SEGGER_RTT_WriteString(0, ",");
  printVal(gy); SEGGER_RTT_WriteString(0, ",");
  printVal(gz); SEGGER_RTT_WriteString(0, ",");
  printVal(ax); SEGGER_RTT_WriteString(0, ",");
  printVal(ay); SEGGER_RTT_WriteString(0, ",");
  printVal(az); SEGGER_RTT_WriteString(0, ",");
  printVal(tempC); SEGGER_RTT_WriteString(0, "\n");

  delay(10);  // ~100Hz print rate, matches accel ODR
}