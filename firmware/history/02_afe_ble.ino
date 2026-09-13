#include <bluefruit.h>
#include <Wire.h>
#include <SEGGER_RTT.h>
#include <math.h>

// ============================================================
//  AFE4404 Health Monitor  ->  BLE (Nordic UART Service)
//  Streams: ir, red, amb, bpm_x10, spo2, sdnn, rmssd, stress,
//           resp_x10, beat   as newline-delimited CSV
// ============================================================

#define SDA_PIN         26
#define SCL_PIN         27
#define RESETZ_PIN      30   // P0.30 / AIN6
#define ADC_RDY_PIN     29   // P0.29 / AIN5
#define AFE4404_ADDR    0x58

#ifndef LED_BUILTIN
#define LED_BUILTIN 17
#endif
#define BEAT_LED_PIN    LED_BUILTIN

#define LED_CURRENT_RED_GREEN  20
#define LED_CURRENT_IR          3

#define MIN_PEAK_GAP_MS       333
#define MAX_PEAK_GAP_MS      1500
#define BEAT_FLASH_MS          80
#define MEDIAN_LEN              7
#define THRESHOLD_DECAY_MS   1500
#define EWMA_OLD_W              8
#define EWMA_NEW_W              2
#define RR_BUF_LEN             32
#define MIN_BEATS_FOR_HRV      12
#define RESP_CROSS_MIN_MS     800
#define RESP_CROSS_MAX_MS    8000
#define SPO2_A                110
#define SPO2_B                 25

// How often to push a BLE packet. The AFE samples ~100Hz, but BLE can't
// carry that many small notifies reliably, so we transmit every Nth sample.
// 4 -> ~25 Hz, plenty smooth for a PPG waveform and safe for BLE throughput.
#define BLE_DECIMATE          4

// ---- Nordic UART Service ----
BLEService        nus("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
BLECharacteristic nusTx("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
BLECharacteristic nusRx("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
bool bleConnected = false;

volatile bool adcReady = false;

// Beat detection state
float         dcFilter        = 0;
float         dcRed           = 0;
int32_t       iThreshold      = 1000;
bool          aboveThreshold  = false;
int32_t       peakValue       = 0;
unsigned long lastPeakMs      = 0;
unsigned long lastGapMs       = 600;
int32_t       avgPeakAC       = 0;
bool          signalGood      = false;
unsigned long beatFlashEnd    = 0;
uint16_t      ewmaBPM_x10     = 0;
volatile bool beatPending     = false; // sticky until next BLE packet goes out

// Median filter
int32_t medBuf[MEDIAN_LEN];
uint8_t medIdx  = 0;
bool    medFull = false;

// HRV
uint16_t rrBuf[RR_BUF_LEN];
uint8_t  rrHead  = 0;
uint8_t  rrCount = 0;
uint16_t sdnn_ms   = 0;
uint16_t rmssd_ms  = 0;
uint8_t  stressIdx = 0;

// SpO2
float   sumAcRed = 0, sumDcRed = 0;
float   sumAcIR  = 0, sumDcIR  = 0;
uint8_t spo2AccumCount = 0;
uint8_t spo2_pct = 0;

// Respiration
int32_t       respBaseline    = 0;
bool          respAbove       = false;
unsigned long lastRespCrossMs = 0;
uint16_t      respRate_x10    = 0;
uint8_t       respCrosses     = 0;

// Latest sample (for BLE packet)
int32_t lastIR = 0, lastRED = 0, lastAMB = 0;
uint32_t bleSampleCtr = 0;

void onAdcReady() { adcReady = true; }

// ── Register I/O ────────────────────────────────────────────
bool writeRegister(uint8_t reg, uint32_t value) {
  Wire.beginTransmission(AFE4404_ADDR);
  Wire.write(reg);
  Wire.write((value >> 16) & 0xFF);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  return (Wire.endTransmission() == 0);
}

uint32_t readADCReg(uint8_t reg) {
  Wire.beginTransmission(AFE4404_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AFE4404_ADDR, (uint8_t)3);
  if (Wire.available() < 3) return 0;
  uint32_t v = 0;
  v |= ((uint32_t)Wire.read() << 16);
  v |= ((uint32_t)Wire.read() << 8);
  v |= (uint32_t)Wire.read();
  return v;
}

int32_t toSigned(uint32_t raw) {
  if (raw & 0x800000) return (int32_t)(raw | 0xFF000000);
  return (int32_t)raw;
}

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

// ── Median filter ─────────────────────────────────────────────
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

// ── Hardware init ─────────────────────────────────────────────
void resetAFE() {
  digitalWrite(RESETZ_PIN, HIGH); delay(100);
  digitalWrite(RESETZ_PIN, LOW);  delayMicroseconds(40);
  digitalWrite(RESETZ_PIN, HIGH); delay(10);
}
void setGainAndLED(uint8_t gainCode, uint8_t redGreenCode, uint8_t irCode) {
  writeRegister(0x21, gainCode & 0x07);
  uint32_t ledVal = ((uint32_t)(irCode & 0x3F) << 12) |
                    ((uint32_t)(redGreenCode & 0x3F) << 6) |
                     (uint32_t)(redGreenCode & 0x3F);
  writeRegister(0x22, ledVal);
}
void setOffDAC(uint8_t dacCode, bool subtract) {
  uint8_t mag = dacCode & 0x0F;
  uint8_t pol = subtract ? 1 : 0;
  uint32_t val = ((uint32_t)pol << 19) | ((uint32_t)mag << 15) |
                 ((uint32_t)pol << 14) | ((uint32_t)mag << 10) |
                 ((uint32_t)pol <<  9) | ((uint32_t)mag <<  5) |
                 ((uint32_t)pol <<  4) |  (uint32_t)mag;
  writeRegister(0x3A, val);
}
void configureAFE() {
  writeRegister(0x23, 0x000200); delay(5);
  writeRegister(0x39, 7);
  writeRegister(0x1D, 2499);
  writeRegister(0x21, 0x000005);
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
  writeRegister(0x3D, 0x000000); // decimation off
  setOffDAC(0, false);
  setGainAndLED(2, LED_CURRENT_RED_GREEN, LED_CURRENT_IR);
  writeRegister(0x1E, 0x000103);
  delay(100);
}

// ── HRV ───────────────────────────────────────────────────────
void computeHRV() {
  uint8_t n = min(rrCount, (uint8_t)RR_BUF_LEN);
  if (n < 4) return;
  int32_t sum = 0;
  for (uint8_t i = 0; i < n; i++)
    sum += rrBuf[(rrHead - 1 - i + RR_BUF_LEN) % RR_BUF_LEN];
  int32_t mean = sum / n;
  int32_t varSum = 0;
  for (uint8_t i = 0; i < n; i++) {
    int32_t d = (int32_t)rrBuf[(rrHead - 1 - i + RR_BUF_LEN) % RR_BUF_LEN] - mean;
    varSum += d * d;
  }
  sdnn_ms = (uint16_t)sqrt((float)varSum / n);
  int32_t ssqd = 0; uint8_t used = 0;
  for (uint8_t i = 1; i < n; i++) {
    int32_t a = rrBuf[(rrHead - 1 - i     + RR_BUF_LEN) % RR_BUF_LEN];
    int32_t b = rrBuf[(rrHead - 1 - (i-1) + RR_BUF_LEN) % RR_BUF_LEN];
    int32_t d = a - b;
    ssqd += d * d; used++;
  }
  if (used > 0) rmssd_ms = (uint16_t)sqrt((float)ssqd / used);
  int32_t s = 100 - ((int32_t)rmssd_ms - 5) * 2;
  stressIdx = (uint8_t)constrain(s, 0, 100);
}

// ── SpO2 ──────────────────────────────────────────────────────
void resetSpO2Accum() { sumAcRed=0; sumDcRed=0; sumAcIR=0; sumDcIR=0; spo2AccumCount=0; }
void accumSpO2(float acRed, float dcRedVal, float acIR, float dcIRval) {
  if (dcRedVal < 100.0f || dcIRval < 100.0f) return;
  if (acRed < 1.0f || acIR < 1.0f) return;
  sumAcRed += acRed; sumDcRed += dcRedVal;
  sumAcIR  += acIR;  sumDcIR  += dcIRval;
  spo2AccumCount++;
}
uint8_t computeSpO2() {
  if (spo2AccumCount == 0) return 0;
  float avgAcRed = sumAcRed / spo2AccumCount, avgDcRed = sumDcRed / spo2AccumCount;
  float avgAcIR  = sumAcIR  / spo2AccumCount, avgDcIR  = sumDcIR  / spo2AccumCount;
  if (avgDcRed < 100.0f || avgDcIR < 100.0f) return 0;
  float R = (avgAcRed / avgDcRed) / (avgAcIR / avgDcIR);
  float spo2 = (float)SPO2_A - (float)SPO2_B * R;
  if (spo2 > 100.0f) spo2 = 100.0f;
  if (spo2 < 70.0f) return 0;
  return (uint8_t)(spo2 + 0.5f);
}

// ── Respiration ───────────────────────────────────────────────
void updateRespiration(int32_t filtered) {
  respBaseline = respBaseline - (respBaseline >> 7) + (filtered >> 7);
  int32_t ac = filtered - respBaseline;
  bool curAbove = (ac > 0);
  if (curAbove && !respAbove) {
    unsigned long now = millis();
    unsigned long gap = now - lastRespCrossMs;
    if (lastRespCrossMs > 0 && gap >= RESP_CROSS_MIN_MS && gap <= RESP_CROSS_MAX_MS) {
      uint32_t brpm_x10 = 600000UL / gap;
      respRate_x10 = (respRate_x10 == 0) ? (uint16_t)brpm_x10
                   : (uint16_t)((respRate_x10 * 7 + brpm_x10 * 3) / 10);
      respCrosses++;
    }
    lastRespCrossMs = now;
  }
  respAbove = curAbove;
}

// ── Beat processor ────────────────────────────────────────────
void processSample(int32_t primary, int32_t ir_net, int32_t red_net) {
  if (primary <= -8000000L || primary >= 8000000L) return;
  int32_t filtered = medianFilter(primary);
  dcFilter = dcFilter * 0.9f + (float)filtered * 0.1f;
  int32_t ac = filtered - (int32_t)dcFilter;
  dcRed = dcRed * 0.9f + (float)red_net * 0.1f;
  int32_t acRed = red_net - (int32_t)dcRed;
  unsigned long now = millis();

  if ((now - lastPeakMs) > THRESHOLD_DECAY_MS && iThreshold > 1000) {
    iThreshold -= iThreshold / 10;
    if (iThreshold < 1000) iThreshold = 1000;
  }
  if ((now - lastPeakMs) > (MAX_PEAK_GAP_MS * 6 / 5) && signalGood) {
    signalGood = false; resetSpO2Accum();
  }
  int32_t dynThresh = max(iThreshold * 2 / 5, (int32_t)1000);

  if (ac > dynThresh) {
    if (!aboveThreshold) { aboveThreshold = true; peakValue = ac; }
    else if (ac > peakValue) peakValue = ac;
  } else {
    if (aboveThreshold) {
      aboveThreshold = false;
      unsigned long gap = now - lastPeakMs;
      unsigned long blanking = max((unsigned long)(lastGapMs * 3 / 5), 400UL);
      if (gap >= blanking && gap <= MAX_PEAK_GAP_MS) {
        if (avgPeakAC > 0 && peakValue > avgPeakAC * 10) { iThreshold -= iThreshold / 10; return; }
        avgPeakAC = (avgPeakAC * 7 + peakValue) / 8;
        lastGapMs = gap; lastPeakMs = now;
        iThreshold = (iThreshold * 9 + peakValue) / 10;
        if (gap >= MIN_PEAK_GAP_MS && gap <= MAX_PEAK_GAP_MS) {
          rrBuf[rrHead] = (uint16_t)gap;
          rrHead = (rrHead + 1) % RR_BUF_LEN;
          if (rrCount < RR_BUF_LEN) rrCount++;
        }
        uint32_t bpm_x10 = 600000UL / gap;
        ewmaBPM_x10 = (ewmaBPM_x10 == 0) ? (uint16_t)bpm_x10
                    : (uint16_t)((ewmaBPM_x10 * EWMA_OLD_W + bpm_x10 * EWMA_NEW_W + 5) / 10);
        signalGood = true;
        beatPending = true;
        accumSpO2((float)abs(acRed), fabsf(dcRed), (float)peakValue, fabsf(dcFilter));
        if (spo2AccumCount >= 3) {
          uint8_t newSpo2 = computeSpO2();
          if (newSpo2 > 0)
            spo2_pct = (spo2_pct == 0) ? newSpo2 : (uint8_t)(((uint16_t)spo2_pct*8 + newSpo2*2 + 5)/10);
          if (spo2AccumCount > 8) { sumAcRed/=2; sumDcRed/=2; sumAcIR/=2; sumDcIR/=2; spo2AccumCount=4; }
        }
        if (rrCount >= MIN_BEATS_FOR_HRV) computeHRV();
        digitalWrite(BEAT_LED_PIN, HIGH);
        beatFlashEnd = now + BEAT_FLASH_MS;
      } else if (gap > MAX_PEAK_GAP_MS) {
        signalGood = false; lastPeakMs = now; resetSpO2Accum();
      }
    }
  }
  updateRespiration(filtered);
  if (beatFlashEnd > 0 && now >= beatFlashEnd) { digitalWrite(BEAT_LED_PIN, LOW); beatFlashEnd = 0; }
}

// ── BLE ───────────────────────────────────────────────────────
void connectCallback(uint16_t conn_handle) {
  bleConnected = true;
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(12, 0, 200); // 15ms
  SEGGER_RTT_WriteString(0, "BLE connected\n");
}
void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  bleConnected = false;
  SEGGER_RTT_WriteString(0, "BLE disconnected\n");
  Bluefruit.Advertising.start(0);
}
void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("AFE-HRM");
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
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(nus);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "AFE4404 BLE monitor starting...\n");
  pinMode(RESETZ_PIN, OUTPUT);
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
  SEGGER_RTT_WriteString(0, "AFE4404 found. Configuring...\n");
  configureAFE();
  resetSpO2Accum();

  setupBLE();
  SEGGER_RTT_WriteString(0, "Advertising as AFE-HRM\n");

  attachInterrupt(digitalPinToInterrupt(ADC_RDY_PIN), onAdcReady, RISING);
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  bool doRead = false;
  noInterrupts();
  if (adcReady) { adcReady = false; doRead = true; }
  interrupts();
  if (!doRead) return;

  int32_t ir_raw  = toSigned(readADCReg(0x2B));
  int32_t amb     = toSigned(readADCReg(0x2D));
  int32_t red_raw = toSigned(readADCReg(0x2A));
  int32_t ir_net  = ir_raw - amb;
  int32_t red_net = red_raw - amb;

  lastIR = ir_net; lastRED = red_net; lastAMB = amb;

  processSample(ir_net, ir_net, red_net);

  if (beatFlashEnd > 0 && millis() >= beatFlashEnd) {
    digitalWrite(BEAT_LED_PIN, LOW); beatFlashEnd = 0;
  }

  // Transmit every Nth sample (waveform stays smooth, BLE stays happy).
  // Beat flag is sticky so a beat is never missed even between transmitted samples.
  bleSampleCtr++;
  if (bleConnected && (bleSampleCtr % BLE_DECIMATE == 0)) {
    bool beat = beatPending; beatPending = false;
    char packet[64];
    int n = snprintf(packet, sizeof(packet), "%ld,%ld,%ld,%u,%u,%u,%u,%u,%u,%u\n",
      (long)lastIR, (long)lastRED, (long)lastAMB,
      (unsigned)ewmaBPM_x10, (unsigned)spo2_pct,
      (unsigned)sdnn_ms, (unsigned)rmssd_ms, (unsigned)stressIdx,
      (unsigned)respRate_x10, (unsigned)(beat ? 1 : 0));
    for (int i = 0; i < 3; i++) {
      if (nusTx.notify((uint8_t*)packet, n)) break;
      delay(4);
    }
  }
}
