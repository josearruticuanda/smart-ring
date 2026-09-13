#include <bluefruit.h>
#include <SEGGER_RTT.h>

// ─── EXACT 22-BYTE PUBLIC ADVERTISEMENT KEY ──────────────────────────────────
uint8_t publicKeyBytes[22] = {
  // Replace with your own 22-byte advertisement key (e.g. from OpenHaystack).
  // The original key was removed before publishing.
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "Starting FindMy beacon...\n");

  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Maximize BLE transmit power
  Bluefruit.setName("");

  // Total size of Find My raw BLE frame is exactly 30 bytes
  uint8_t advPayload[30];
  
  advPayload[0] = 0x1D;          // Length of what follows (29 bytes = 0x1D)
  advPayload[1] = 0xFF;          // AD type: Manufacturer Specific Data
  advPayload[2] = 0x4C;          // Apple Company ID Low Byte (0x004C)
  advPayload[3] = 0x00;          // Apple Company ID High Byte
  advPayload[4] = 0x12;          // Find My network sub-type specifier
  advPayload[5] = 0x19;          // Inner payload length (25 bytes follow)
  advPayload[6] = 0x00;          // Status byte: 0x00 = normal un-paired state
  
  // Copy EXACTLY 22 bytes of your public key into the frame
  memcpy(&advPayload[7], publicKeyBytes, 22);
  
  advPayload[29] = 0x00;         // Status / Hint byte (First 2 bits of key + hint)

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.setData(advPayload, sizeof(advPayload));

  // Advertise every 2 seconds to match standard tracking frequencies
  Bluefruit.Advertising.setInterval(2048, 2048);
  Bluefruit.Advertising.start(0);

  SEGGER_RTT_WriteString(0, "FindMy beacon broadcasting\n");
}

void loop() {
  sd_app_evt_wait();
}