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
#define GYRO_DATA_START   0x02
#define TEMP_MSB_REG      0x22

#define ACC_SENSITIVITY   5460.0f
#define GYRO_SENSITIVITY  16.384f

BLEService        nus("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
BLECharacteristic nusTx("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
BLECharacteristic nusRx("6e400002-b5a3-f393-e0a9-e50e24dcca9e");

bool connected = false;

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
  uint8_t val; readRegs(addr, reg, &val, 1); return val;
}

bool accelReady() { return (readReg(BMI088_ACC_ADDR, ACC_STATUS) & 0x80) != 0; }

void initSensors() {
  writeReg(BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00); delay(1);
  writeReg(BMI088_ACC_ADDR, ACC_PWR_CTRL, 0x04); delay(5);
}

void connectCallback(uint16_t conn_handle) {
  connected = true;
  // 20 = 20*1.25ms = 25ms interval, latency=0, timeout=200 (in 10ms units = 2s)
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(20, 0, 200);
  SEGGER_RTT_WriteString(0, "BLE connected\n");
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  connected = false;
  SEGGER_RTT_WriteString(0, "BLE disconnected\n");
  Bluefruit.Advertising.start(0);
}

void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("IMU-BMI088");
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  nus.begin();

  nusTx.setProperties(CHR_PROPS_NOTIFY);
  nusTx.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  nusTx.setMaxLen(64);
  nusTx.begin();

  nusRx.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  nusRx.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  nusRx.setMaxLen(64);
  nusRx.begin();

  // Primary packet: keep lean — just flags + service UUID
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(nus);

  // Name goes in scan response to avoid 31-byte overflow
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setup() {
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "Starting...\n");

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  initSensors();
  setupBLE();

  SEGGER_RTT_WriteString(0, "Advertising as IMU-BMI088\n");
}

void loop() {
  uint8_t buf[6];
  static float lgx=0, lgy=0, lgz=0, lax=0, lay=0, laz=0;

  readRegs(BMI088_GYRO_ADDR, GYRO_DATA_START, buf, 6);
  lgx = (int16_t)((buf[1] << 8) | buf[0]) / GYRO_SENSITIVITY;
  lgy = (int16_t)((buf[3] << 8) | buf[2]) / GYRO_SENSITIVITY;
  lgz = (int16_t)((buf[5] << 8) | buf[4]) / GYRO_SENSITIVITY;

  if (accelReady()) {
    readRegs(BMI088_ACC_ADDR, ACC_DATA_START, buf, 6);
    lax = (int16_t)((buf[1] << 8) | buf[0]) / ACC_SENSITIVITY;
    lay = (int16_t)((buf[3] << 8) | buf[2]) / ACC_SENSITIVITY;
    laz = (int16_t)((buf[5] << 8) | buf[4]) / ACC_SENSITIVITY;
  }

  readRegs(BMI088_ACC_ADDR, TEMP_MSB_REG, buf, 2);
  int16_t tempRaw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
  if (tempRaw > 1023) tempRaw -= 2048;
  float tempC = tempRaw * 0.125f + 23.0f;

  if (connected) {
    char packet[64];
    snprintf(packet, sizeof(packet), "%d,%d,%d,%d,%d,%d,%d\n",
      (int)(lgx * 1000), (int)(lgy * 1000), (int)(lgz * 1000),
      (int)(lax * 1000), (int)(lay * 1000), (int)(laz * 1000),
      (int)(tempC * 1000)
    );

    // Retry up to 5 times with a small backoff
    for (int i = 0; i < 5; i++) {
      if (nusTx.notify((uint8_t*)packet, strlen(packet))) {
        SEGGER_RTT_WriteString(0, packet);
        break;
      }
      delay(10);
    }
  }

  delay(100); // 10 Hz — safe floor while connection params settle
}