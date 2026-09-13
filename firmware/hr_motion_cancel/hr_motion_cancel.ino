// green_bpm_motion_lean.ino
// ============================================================================
//  Motion-tolerant GREEN-LED heart-rate monitor  —  nRF52832 flexible PCB
//  Lean output: prints ONE line per detected beat -> beat marker, BPM, movement.
//
//  Hardware (one I2C bus, SDA=26 SCL=27):
//    AFE4404   @ 0x58   TX1=GREEN(LED1)   (red/IR left off — green-only BPM)
//    ICM-42670 @ 0x68   accel + gyro, used as a motion reference
//
//  Example RTT output:
//    BEAT  BPM 72.4  motion 31 mg (still)
//    BEAT  BPM 73.1  motion 118 mg (moving)
//    BEAT  BPM 73.0  motion 95 mg (moving)
// ============================================================================

#include <bluefruit.h>
#include <Wire.h>
#include <SEGGER_RTT.h>
#include <math.h>

// ── Pins ────────────────────────────────────────────────────────────────────
#define SDA_PIN        26
#define SCL_PIN        27
#define RESETZ_PIN     30   // AFE RESETZ
#define ADC_RDY_PIN    29   // AFE ADC_RDY — master sample clock

#define AFE4404_ADDR   0x58
#define ICM_ADDR       0x68

#ifndef LED_BUILTIN
#define LED_BUILTIN    17
#endif
#define BEAT_LED_PIN   LED_BUILTIN

// ── Detector tuning ─────────────────────────────────────────────────────────
#define GREEN_LED_START_CURRENT  20
#define MIN_PEAK_GAP_MS          333   // ~180 BPM ceiling
#define MAX_PEAK_GAP_MS          1500  // ~40 BPM floor
#define HISTORY_LEN              8
#define BEAT_FLASH_MS            80
#define MEDIAN_LEN               7
#define THRESHOLD_DECAY_MS       1500
#define EWMA_OLD_WEIGHT          8
#define EWMA_NEW_WEIGHT          2
#define EWMA_OLD_WEIGHT_MOTION   9     // smooth harder while moving
#define EWMA_NEW_WEIGHT_MOTION   1

// ── Motion / adaptive-filter tuning ─────────────────────────────────────────
#define ACCEL_LSB_PER_G   2048.0f
#define GYRO_LSB_PER_DPS  16.4f
#define HP_SHIFT          6
#define NLMS_TAPS         4
#define NLMS_MU           0.20f
#define NLMS_LEAK         0.9995f
#define NLMS_NORM_FLOOR   0.02f
#define GYRO_MOTION_W     0.001f
#define MOTION_GATE_MG    80.0f    // above -> "moving"
#define MOTION_REJECT_MG  300.0f   // above -> "heavy": reject beats, hold BPM

// ── State ────────────────────────────────────────────────────────────────────
int32_t       iBaseline      = 0;
int32_t       iThreshold     = 1000;
bool          aboveThreshold = false;
int32_t       peakValue      = 0;
unsigned long lastPeakMs     = 0;
unsigned long lastGapMs      = 600;
unsigned long beatTimes[HISTORY_LEN];
uint8_t       beatHead       = 0;
uint8_t       beatCount      = 0;
unsigned long beatFlashEnd   = 0;
int32_t       avgPeakAC      = 0;
bool          signalGood     = false;
uint16_t      ewmaBPM_x10    = 0;
volatile bool beatThisSample = false;

int32_t medBuf[MEDIAN_LEN];
uint8_t medIdx  = 0;
bool    medFull = false;

int32_t axDC = 0, ayDC = 0, azDC = 0;
float   motionEWMA_mg = 0.0f;
float   w[3][NLMS_TAPS]    = {{0}};
float   xbuf[3][NLMS_TAPS] = {{0}};

volatile bool adcReady = false;
void onAdcReady() { adcReady = true; }

// ── AFE register I/O ─────────────────────────────────────────────────────────
bool writeRegister(uint8_t reg, uint32_t value) {
  Wire.beginTransmission(AFE4404_ADDR);
  Wire.write(reg);
  Wire.write((value >> 16) & 0xFF);
  Wire.write((value >>  8) & 0xFF);
  Wire.write( value        & 0xFF);
  return (Wire.endTransmission() == 0);
}
uint32_t readRegister(uint8_t reg) {
  writeRegister(0x00, 0x000001);                 // REG_READ = 1
  Wire.beginTransmission(AFE4404_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AFE4404_ADDR, (uint8_t)3);
  if (Wire.available() < 3) { writeRegister(0x00, 0x000000); return 0xFFFFFF; }
  uint32_t v = 0;
  v |= ((uint32_t)Wire.read() << 16);
  v |= ((uint32_t)Wire.read() <<  8);
  v |=  (uint32_t)Wire.read();
  writeRegister(0x00, 0x000000);                 // REG_READ = 0
  return v;
}
int32_t toSigned(uint32_t raw) {
  if (raw & 0x800000) return (int32_t)(raw | 0xFF000000);
  return (int32_t)raw;
}

// ── I2C bus recovery ─────────────────────────────────────────────────────────
void i2cBusRecovery() {
  pinMode(SCL_PIN, INPUT_PULLUP);
  pinMode(SDA_PIN, INPUT_PULLUP);
  delay(2);
  if (digitalRead(SDA_PIN) == LOW) {
    pinMode(SCL_PIN, OUTPUT);
    for (int i = 0; i < 9; i++) {
      digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
      digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
      if (digitalRead(SDA_PIN) == HIGH) break;
    }
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);
  }
}

// ── Median filter ────────────────────────────────────────────────────────────
int32_t medianOf(int32_t *a, uint8_t n) {
  int32_t tmp[MEDIAN_LEN];
  for (uint8_t i = 0; i < n; i++) tmp[i] = a[i];
  for (uint8_t i = 1; i < n; i++) {
    int32_t key = tmp[i]; int8_t j = i - 1;
    while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
    tmp[j+1] = key;
  }
  return tmp[n / 2];
}
int32_t medianFilter(int32_t raw) {
  medBuf[medIdx] = raw;
  medIdx = (medIdx + 1) % MEDIAN_LEN;
  if (!medFull && medIdx == 0) medFull = true;
  uint8_t n = medFull ? MEDIAN_LEN : medIdx;
  return (n == 0) ? raw : medianOf(medBuf, n);
}

// ── AFE bring-up ─────────────────────────────────────────────────────────────
void resetAFE() {
  digitalWrite(RESETZ_PIN, HIGH); delay(100);
  digitalWrite(RESETZ_PIN, LOW);  delayMicroseconds(40);
  digitalWrite(RESETZ_PIN, HIGH); delay(10);
}
void setGainAndLED(uint8_t gainCode, uint8_t ledCode) {
  writeRegister(0x21, gainCode & 0x07);
  writeRegister(0x22, (uint32_t)(ledCode & 0x3F));   // ILED1 only (green)
}
void configureAFE() {
  writeRegister(0x23, 0x000200); delay(5);   // OSC_ENABLE
  writeRegister(0x39, 7);                     // CLKDIV_PRF = 16
  writeRegister(0x1D, 2499);                  // PRF = 100 Hz

  writeRegister(0x09,  0);  writeRegister(0x0A, 24);
  writeRegister(0x01,  7);  writeRegister(0x02, 24);
  writeRegister(0x15, 26);  writeRegister(0x16, 26);
  writeRegister(0x0D, 28);  writeRegister(0x0E, 94);
  writeRegister(0x36, 26);  writeRegister(0x37, 50);
  writeRegister(0x05, 33);  writeRegister(0x06, 50);
  writeRegister(0x17, 96);  writeRegister(0x18, 96);
  writeRegister(0x0F, 98);  writeRegister(0x10, 164);
  writeRegister(0x03, 52);  writeRegister(0x04, 76);
  writeRegister(0x07, 59);  writeRegister(0x08, 76);
  writeRegister(0x19, 166); writeRegister(0x1A, 166);
  writeRegister(0x11, 168); writeRegister(0x12, 234);
  writeRegister(0x0B, 85);  writeRegister(0x0C, 102);
  writeRegister(0x1B, 236); writeRegister(0x1C, 236);
  writeRegister(0x13, 238); writeRegister(0x14, 304);

  writeRegister(0x32, 354);
  writeRegister(0x33, 2449);
  writeRegister(0x3D, 0x000000);  // decimation OFF -> 100 Hz (matches IMU)

  setGainAndLED(2, GREEN_LED_START_CURRENT);
  writeRegister(0x1E, 0x000103);  // NUMAV=3, TIMEREN=1
  delay(100);
}
void autoTune() {
  SEGGER_RTT_WriteString(0, "Auto-tuning green channel...\n");
  const uint8_t gainCodes[] = {5, 4, 3, 2, 1, 0};
  const int32_t TARGET_LO = 400000L, TARGET_HI = 1600000L;
  bool tuned = false;
  for (int g = 0; g < 6 && !tuned; g++) {
    for (int8_t lc = 63; lc >= 4 && !tuned; lc -= 4) {
      setGainAndLED(gainCodes[g], lc);
      delay(80);
      int32_t sum = 0; int got = 0; bool last = false;
      unsigned long deadline = millis() + 500;
      while (got < 8 && millis() < deadline) {
        bool cur = (digitalRead(ADC_RDY_PIN) == HIGH);
        if (cur && !last) { sum += toSigned(readRegister(0x2C)); got++; }
        last = cur;
      }
      int32_t avg = got ? sum / got : 0;
      if (avg > TARGET_LO && avg < TARGET_HI) {
        SEGGER_RTT_printf(0, "Tuned: gain=%d LEDcode=%d\n", gainCodes[g], lc);
        tuned = true;
      }
    }
  }
  if (!tuned) { SEGGER_RTT_WriteString(0, "Auto-tune fallback.\n"); setGainAndLED(5, 4); }

  delay(150);
  bool last = false; unsigned long t = millis() + 500;
  while (millis() < t) {
    bool cur = (digitalRead(ADC_RDY_PIN) == HIGH);
    if (cur && !last) {
      int32_t v = toSigned(readRegister(0x2F));
      iBaseline  = v;
      iThreshold = abs(v / 50);
      if (iThreshold < 1000) iThreshold = 1000;
      break;
    }
    last = cur;
  }
}

// ── IMU ──────────────────────────────────────────────────────────────────────
bool icmWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg); Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}
bool icmRead(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(true) != 0) return false;
  if (Wire.requestFrom((int)ICM_ADDR, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}
void configureIMU() {
  icmWrite(0x02, 0x10); delay(2);
  uint8_t who = 0; icmRead(0x75, &who, 1);
  SEGGER_RTT_printf(0, "ICM WHO_AM_I: 0x%02X (expect 0x67)\n", who);
  icmWrite(0x20, 0x09);   // gyro ±2000 dps, 100 Hz
  icmWrite(0x21, 0x09);   // accel ±16 g, 100 Hz
  icmWrite(0x1F, 0x0F);   // accel+gyro low-noise
  delay(1); delay(45);
}
float readIMU(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t r[14];
  if (!icmRead(0x09, r, 14)) { ax = ay = az = 0; return 0.0f; }
  ax = (int16_t)((r[2]  << 8) | r[3]);
  ay = (int16_t)((r[4]  << 8) | r[5]);
  az = (int16_t)((r[6]  << 8) | r[7]);
  int16_t gx = (int16_t)((r[8]  << 8) | r[9]);
  int16_t gy = (int16_t)((r[10] << 8) | r[11]);
  int16_t gz = (int16_t)((r[12] << 8) | r[13]);
  float gxd = gx / GYRO_LSB_PER_DPS, gyd = gy / GYRO_LSB_PER_DPS, gzd = gz / GYRO_LSB_PER_DPS;
  return sqrtf(gxd*gxd + gyd*gyd + gzd*gzd);
}

// ── BPM history ──────────────────────────────────────────────────────────────
uint16_t computeRawBPM_x10() {
  if (beatCount < 2) return 0;
  uint8_t  n = min(beatCount, (uint8_t)HISTORY_LEN);
  uint32_t sum = 0; uint8_t used = 0;
  for (uint8_t i = 1; i < n; i++) {
    uint8_t cur  = (beatHead - i     + HISTORY_LEN) % HISTORY_LEN;
    uint8_t prev = (beatHead - i - 1 + HISTORY_LEN) % HISTORY_LEN;
    unsigned long gap = beatTimes[cur] - beatTimes[prev];
    if (gap >= MIN_PEAK_GAP_MS && gap <= MAX_PEAK_GAP_MS) { sum += 600000UL / gap; used++; }
  }
  return used ? (uint16_t)(sum / used) : 0;
}

// ── Beat detection on the cleaned signal ────────────────────────────────────
void detectBeat(int32_t cleanAC, float motion_mg) {
  unsigned long now = millis();
  int32_t dynThresh = max(iThreshold * 2 / 5, (int32_t)1000);

  if ((now - lastPeakMs) > THRESHOLD_DECAY_MS && iThreshold > 1000) {
    iThreshold -= iThreshold / 10;
    if (iThreshold < 1000) iThreshold = 1000;
  }
  if ((now - lastPeakMs) > (MAX_PEAK_GAP_MS * 6 / 5)) {
    if (beatCount > 0) { beatCount = 0; signalGood = false; }
  }

  if (cleanAC > dynThresh) {
    if (!aboveThreshold) { aboveThreshold = true; peakValue = cleanAC; }
    else if (cleanAC > peakValue) peakValue = cleanAC;
  } else {
    if (aboveThreshold) {
      aboveThreshold = false;
      unsigned long gap = now - lastPeakMs;
      unsigned long blanking = max((unsigned long)(lastGapMs * 2 / 5), 250UL);

      if (gap >= blanking && gap <= MAX_PEAK_GAP_MS) {
        int32_t peakAC = peakValue;

        if (motion_mg >= MOTION_REJECT_MG) { iThreshold -= iThreshold / 10; return; }
        if (avgPeakAC > 0 && peakAC > avgPeakAC * 10) { iThreshold -= iThreshold / 10; return; }

        avgPeakAC  = (avgPeakAC * 7 + peakAC) / 8;
        lastGapMs  = gap;
        lastPeakMs = now;
        beatTimes[beatHead] = now;
        beatHead = (beatHead + 1) % HISTORY_LEN;
        if (beatCount < HISTORY_LEN) beatCount++;
        iThreshold = (iThreshold * 9 + peakAC) / 10;

        uint16_t rawBPM = computeRawBPM_x10();
        if (rawBPM > 0) {
          bool moving = (motion_mg >= MOTION_GATE_MG);
          uint8_t ow = moving ? EWMA_OLD_WEIGHT_MOTION : EWMA_OLD_WEIGHT;
          uint8_t nw = moving ? EWMA_NEW_WEIGHT_MOTION : EWMA_NEW_WEIGHT;
          ewmaBPM_x10 = (ewmaBPM_x10 == 0)
                      ? rawBPM
                      : (uint16_t)((ewmaBPM_x10 * ow + rawBPM * nw + 5) / (ow + nw));
          signalGood = true;
        }

        beatThisSample = true;
        digitalWrite(BEAT_LED_PIN, HIGH);
        beatFlashEnd = now + BEAT_FLASH_MS;

      } else if (gap > MAX_PEAK_GAP_MS) {
        beatCount = 0; lastPeakMs = now; signalGood = false;
      }
    }
  }

  if (beatFlashEnd > 0 && now >= beatFlashEnd) {
    digitalWrite(BEAT_LED_PIN, LOW); beatFlashEnd = 0;
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "green_bpm_motion_lean starting...\n");

  pinMode(RESETZ_PIN,  OUTPUT);
  pinMode(ADC_RDY_PIN, INPUT);
  pinMode(BEAT_LED_PIN, OUTPUT);
  digitalWrite(RESETZ_PIN, HIGH);
  digitalWrite(BEAT_LED_PIN, LOW);

  i2cBusRecovery();
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  resetAFE();
  Wire.beginTransmission(AFE4404_ADDR);
  if (Wire.endTransmission() != 0) {
    SEGGER_RTT_WriteString(0, "AFE4404 not found!\n");
    while (1) {}
  }
  configureAFE();
  configureIMU();
  autoTune();

  int16_t ax, ay, az;
  readIMU(ax, ay, az);
  axDC = (int32_t)ax << HP_SHIFT; ayDC = (int32_t)ay << HP_SHIFT; azDC = (int32_t)az << HP_SHIFT;

  attachInterrupt(digitalPinToInterrupt(ADC_RDY_PIN), onAdcReady, RISING);
  SEGGER_RTT_WriteString(0, "Running. (beat / BPM / movement)\n");
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  if (beatFlashEnd > 0 && millis() >= beatFlashEnd) {
    digitalWrite(BEAT_LED_PIN, LOW); beatFlashEnd = 0;
  }

  bool doRead = false;
  noInterrupts();
  if (adcReady) { adcReady = false; doRead = true; }
  interrupts();
  if (!doRead) return;

  // green PPG (hardware ambient-subtracted)
  int32_t green_raw = toSigned(readRegister(0x2F));
  if (green_raw <= -8000000L || green_raw >= 8000000L) return;

  // IMU + accel AC (gravity removed)
  int16_t ax, ay, az;
  float gyroMag = readIMU(ax, ay, az);
  axDC += (int32_t)ax - (axDC >> HP_SHIFT);
  ayDC += (int32_t)ay - (ayDC >> HP_SHIFT);
  azDC += (int32_t)az - (azDC >> HP_SHIFT);
  float axAC = (ax - (axDC >> HP_SHIFT)) / ACCEL_LSB_PER_G;
  float ayAC = (ay - (ayDC >> HP_SHIFT)) / ACCEL_LSB_PER_G;
  float azAC = (az - (azDC >> HP_SHIFT)) / ACCEL_LSB_PER_G;

  float motionInst = (fabsf(axAC) + fabsf(ayAC) + fabsf(azAC)
                      + gyroMag * GYRO_MOTION_W) * 1000.0f;
  motionEWMA_mg = motionEWMA_mg * 0.8f + motionInst * 0.2f;

  // PPG -> median -> DC removal -> AC
  int32_t filtered = medianFilter(green_raw);
  iBaseline = iBaseline - (iBaseline / 64) + (filtered / 64);
  int32_t green_ac = filtered - iBaseline;

  // NLMS motion cancellation
  for (int8_t t = NLMS_TAPS - 1; t > 0; t--) {
    xbuf[0][t] = xbuf[0][t-1];
    xbuf[1][t] = xbuf[1][t-1];
    xbuf[2][t] = xbuf[2][t-1];
  }
  xbuf[0][0] = axAC; xbuf[1][0] = ayAC; xbuf[2][0] = azAC;
  float y = 0.0f, norm = 0.0f;
  for (uint8_t a = 0; a < 3; a++)
    for (uint8_t t = 0; t < NLMS_TAPS; t++) { y += w[a][t]*xbuf[a][t]; norm += xbuf[a][t]*xbuf[a][t]; }
  if (norm < NLMS_NORM_FLOOR) norm = NLMS_NORM_FLOOR;
  float e = (float)green_ac - y;
  float k = NLMS_MU * e / norm;
  for (uint8_t a = 0; a < 3; a++)
    for (uint8_t t = 0; t < NLMS_TAPS; t++)
      w[a][t] = NLMS_LEAK * w[a][t] + k * xbuf[a][t];

  detectBeat((int32_t)e, motionEWMA_mg);

  // ── Output: one line per beat — beat marker, BPM, movement ──
  if (beatThisSample) {
    beatThisSample = false;
    const char *mv = (motionEWMA_mg < MOTION_GATE_MG)   ? "still"
                   : (motionEWMA_mg < MOTION_REJECT_MG) ? "moving"
                                                        : "heavy";
    SEGGER_RTT_printf(0, "BEAT  BPM %u.%u  motion %d mg (%s)\n",
      (unsigned)(ewmaBPM_x10 / 10), (unsigned)(ewmaBPM_x10 % 10),
      (int)motionEWMA_mg, mv);
  }
}
