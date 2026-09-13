#include <bluefruit.h>
#include <Wire.h>
#include <SEGGER_RTT.h>

// ---- Pin map ----
#define SDA_PIN     26
#define SCL_PIN     27
#define RESETZ_PIN  30   // P0.30 / AIN6
#define ADCRDY_PIN  29   // P0.29 / AIN5

#define AFE_ADDR    0x58

#ifndef LED_BUILTIN
#define LED_BUILTIN 17
#endif

#define LED_CURRENT_RED_GREEN  20
#define LED_CURRENT_IR          3

void rttln(const char *s) { SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\n"); }

// ---- I2C bus recovery: clears a stuck SDA line before Wire even starts ----
void i2cBusRecovery() {
  pinMode(SCL_PIN, INPUT_PULLUP);
  pinMode(SDA_PIN, INPUT_PULLUP);
  delay(2);

  if (digitalRead(SDA_PIN) == LOW) {
    rttln("WARNING: SDA stuck low at boot, attempting bus recovery...");
    pinMode(SCL_PIN, OUTPUT);
    for (int i = 0; i < 9; i++) {
      digitalWrite(SCL_PIN, LOW);
      delayMicroseconds(5);
      digitalWrite(SCL_PIN, HIGH);
      delayMicroseconds(5);
      if (digitalRead(SDA_PIN) == HIGH) break;
    }
    // Force a STOP condition
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(5);
    rttln(digitalRead(SDA_PIN) == HIGH ? "Bus recovery: SDA released." : "Bus recovery: SDA STILL stuck low (hardware/power issue).");
  } else {
    rttln("I2C bus idle (SDA/SCL both high) - good.");
  }
}

// ---- Register I/O, ported 1:1 from the working Arduino version (REG_READ wrapped) ----
bool writeRegister(uint8_t reg, uint32_t value) {
  Wire.beginTransmission(AFE_ADDR);
  Wire.write(reg);
  Wire.write((value >> 16) & 0xFF);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  return (Wire.endTransmission() == 0);
}

uint32_t readRegister(uint8_t reg) {
  writeRegister(0x00, 0x000001); // REG_READ = 1
  Wire.beginTransmission(AFE_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AFE_ADDR, (uint8_t)3);
  if (Wire.available() < 3) { writeRegister(0x00, 0x000000); return 0; }
  uint32_t v = 0;
  v |= ((uint32_t)Wire.read() << 16);
  v |= ((uint32_t)Wire.read() << 8);
  v |= (uint32_t)Wire.read();
  writeRegister(0x00, 0x000000); // REG_READ = 0
  return v;
}

int32_t toSigned(uint32_t raw) {
  if (raw & 0x800000) return (int32_t)(raw | 0xFF000000);
  return (int32_t)raw;
}

void resetAFE() {
  digitalWrite(RESETZ_PIN, HIGH); delay(100);
  digitalWrite(RESETZ_PIN, LOW);  delayMicroseconds(40);
  digitalWrite(RESETZ_PIN, HIGH); delay(10);
}

// ---- Same configuration as the proven Arduino sketch (CLKDIV_PRF=16 case) ----
void configureAFE() {
  writeRegister(0x23, 0x000200); delay(5); // OSC_ENABLE
  writeRegister(0x39, 7);                  // CLKDIV_PRF = 16
  writeRegister(0x1D, 2499);               // PRPCT -> PRF = 100Hz
  writeRegister(0x21, 0x000005);           // TIA_GAIN = 5 (10k)

  writeRegister(0x09,  0);   writeRegister(0x0A, 24);
  writeRegister(0x01,  7);   writeRegister(0x02, 24);
  writeRegister(0x15, 26);   writeRegister(0x16, 26);
  writeRegister(0x0D, 28);   writeRegister(0x0E, 94);

  writeRegister(0x36, 26);   writeRegister(0x37, 50);
  writeRegister(0x05, 33);   writeRegister(0x06, 50);
  writeRegister(0x17, 96);   writeRegister(0x18, 96);
  writeRegister(0x0F, 98);   writeRegister(0x10, 164);

  writeRegister(0x03, 52);   writeRegister(0x04, 76);
  writeRegister(0x07, 59);   writeRegister(0x08, 76);
  writeRegister(0x19, 166);  writeRegister(0x1A, 166);
  writeRegister(0x11, 168);  writeRegister(0x12, 234);

  writeRegister(0x0B, 85);   writeRegister(0x0C, 102);
  writeRegister(0x1B, 236);  writeRegister(0x1C, 236);
  writeRegister(0x13, 238);  writeRegister(0x14, 304);

  writeRegister(0x32, 354);
  writeRegister(0x33, 2449);
  writeRegister(0x3D, 0x000003); // decimation

  uint32_t ledVal = ((uint32_t)(LED_CURRENT_IR & 0x3F) << 12) |
                    ((uint32_t)(LED_CURRENT_RED_GREEN & 0x3F) << 6) |
                     (uint32_t)(LED_CURRENT_RED_GREEN & 0x3F);
  writeRegister(0x22, ledVal);

  writeRegister(0x1E, 0x000103); // NUMAV + TIMEREN
  delay(100);
}

void setup() {
  SEGGER_RTT_Init();
  rttln("AFE4404 bring-up starting...");

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  pinMode(RESETZ_PIN, OUTPUT);
  pinMode(ADCRDY_PIN, INPUT);
  digitalWrite(RESETZ_PIN, HIGH);

  rttln("Checkpoint 1: pins configured.");

  i2cBusRecovery();
  rttln("Checkpoint 2: bus recovery done.");

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);
  rttln("Checkpoint 3: Wire.begin() returned.");

  resetAFE();
  rttln("Checkpoint 4: RESETZ pulse done.");

  rttln("Checkpoint 5: about to check ACK at 0x58...");
  Wire.beginTransmission(AFE_ADDR);
  byte err = Wire.endTransmission();
  rttln("Checkpoint 6: ACK check returned.");

  if (err == 0) {
    rttln("AFE4404 ACKed at 0x58.");
  } else {
    SEGGER_RTT_printf(0, "AFE4404 NOT responding (I2C error %d). Check power/wiring.\n", err);
    return; // don't proceed into configuration if device isn't there
  }

  configureAFE();
  rttln("AFE4404 configured. Streaming data...");
}

uint32_t sampleCount = 0;

void loop() {
  bool rdy = digitalRead(ADCRDY_PIN);
  static bool lastRdy = false;
  if (rdy && !lastRdy) {
    int32_t ir_raw  = toSigned(readRegister(0x2B));
    int32_t amb     = toSigned(readRegister(0x2D));
    int32_t red_raw = toSigned(readRegister(0x2A));
    int32_t ir_net  = ir_raw - amb;
    int32_t red_net = red_raw - amb;

    if ((sampleCount % 10) == 0) {
      SEGGER_RTT_printf(0, "IR: %d  RED: %d  AMB: %d\n", ir_net, red_net, amb);
    }
#ifdef LED_BUILTIN
    digitalWrite(LED_BUILTIN, (sampleCount / 10) % 2);
#endif
    sampleCount++;
  }
  lastRdy = rdy;
}
