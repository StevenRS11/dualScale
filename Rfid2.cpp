#include "Rfid2.h"

#include <MFRC522_I2C.h>

// MFRC522 over I2C at address 0x28 used in the M5Stack RFID2 unit.
static TwoWire* wireInstance = nullptr;
static MFRC522_I2C* rfid = nullptr;
static bool initialized = false;


bool rfid2Begin(TwoWire &w) {
  RFID2_DEBUG_PRINT("rfid2Begin: start\n");
  // Note: Wire.begin() should be called with correct pins BEFORE calling this function

  // Store the Wire instance
  wireInstance = &w;

  // Create MFRC522_I2C instance with the provided Wire bus
  if (rfid != nullptr) {
    delete rfid;
  }
  rfid = new MFRC522_I2C(0x28, 26, wireInstance);

  // Check if RFID module is present on I2C bus
  RFID2_DEBUG_PRINT("rfid2Begin: checking I2C presence at 0x28\n");
  yield(); // Feed watchdog
  w.beginTransmission(0x28);
  byte i2cError = w.endTransmission();
  RFID2_DEBUG_PRINT("rfid2Begin: I2C endTransmission returned %d\n", i2cError);

  if (i2cError != 0) {
    RFID2_DEBUG_PRINT("rfid2Begin: RFID module not found at 0x28\n");
    return false;
  }

  RFID2_DEBUG_PRINT("rfid2Begin: calling PCD_Init()\n");
  yield(); // Feed watchdog
  rfid->PCD_Init(); // Initialize MFRC522
  RFID2_DEBUG_PRINT("rfid2Begin: PCD_Init() completed\n");

  yield(); // Feed watchdog
  delay(50); // Give chip time to initialize

  // Verify initialization by reading version register
  byte version = rfid->PCD_ReadRegister(rfid->VersionReg);
  RFID2_DEBUG_PRINT("rfid2Begin: version register (0x37) = 0x%02X\n", version);

  // Read a few other registers to diagnose communication
  byte comIrq = rfid->PCD_ReadRegister(rfid->ComIrqReg);
  byte divIrq = rfid->PCD_ReadRegister(rfid->DivIrqReg);
  byte error = rfid->PCD_ReadRegister(rfid->ErrorReg);
  RFID2_DEBUG_PRINT("rfid2Begin: ComIrqReg=0x%02X DivIrqReg=0x%02X ErrorReg=0x%02X\n", comIrq, divIrq, error);

  // Test write/read on a scratch register to verify I2C communication
  byte testValue = 0xAA;
  rfid->PCD_WriteRegister(rfid->FIFODataReg, testValue);
  byte readBack = rfid->PCD_ReadRegister(rfid->FIFODataReg);
  RFID2_DEBUG_PRINT("rfid2Begin: I2C test - wrote 0x%02X, read back 0x%02X\n", testValue, readBack);

  // MFRC522 version should be 0x91 or 0x92, but some clones report 0x00 or other values
  // Accept 0x88-0x92 range, or warn but continue if version seems wrong
  if (version == 0x00 || version == 0xFF) {
    RFID2_DEBUG_PRINT("rfid2Begin: WARNING - invalid version 0x%02X, chip may not be responding properly\n", version);
  } else if (version < 0x88 || version > 0x92) {
    RFID2_DEBUG_PRINT("rfid2Begin: WARNING - unexpected version 0x%02X (expected 0x88-0x92)\n", version);
    RFID2_DEBUG_PRINT("rfid2Begin: This may not be an MFRC522 chip\n");
  }

  RFID2_DEBUG_PRINT("rfid2Begin: initialized successfully\n");
  initialized = true;
  return true;
}

 bool waitForCard(int wait) {
  if (!initialized || rfid == nullptr) {
    RFID2_DEBUG_PRINT("waitForCard: not initialized\n");
    return false;
  }
  RFID2_DEBUG_PRINT("waitForCard: waiting for tag\n");
  unsigned long start = millis();
  while (true) {
    if (rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
      RFID2_DEBUG_PRINT("waitForCard: tag detected\n");
      return true;
    }
    if (millis() - start > wait) {
      RFID2_DEBUG_PRINT("waitForCard: timeout\n");
      return false;
    }
    delay(50);
  }
}

bool waitForCard() {
  return waitForCard(3000);
}


bool rfid2WriteText(const String &text, String *errMsg) {
  RFID2_DEBUG_PRINT("rfid2WriteText: '%s'\n", text.c_str());
  if (!initialized || rfid == nullptr) {
    RFID2_DEBUG_PRINT("rfid2WriteText: not initialized\n");
    if (errMsg)
      *errMsg = F("not init");
    return false;
  }

  if (!waitForCard()) {
    if (errMsg)
      *errMsg = F("timeout");
    return false;
  }
  RFID2_DEBUG_PRINT("rfid2WriteText: card ready\n");

  // Build NDEF TLV for a simple text record in English.
  int textLen = text.length();
  if (textLen > 240) { // fits easily within Ultralight pages
    RFID2_DEBUG_PRINT("rfid2WriteText: text too long (%d)\n", textLen);
    if (errMsg)
      *errMsg = F("too long");
    rfid2Halt();
    return false;
  }
  const int payloadLen = textLen + 3;   // status + lang(2) + text
  const int recordLen = payloadLen + 4; // header bytes + type
  const int totalLen = recordLen + 3;   // TLV + terminator
  RFID2_DEBUG_PRINT("rfid2WriteText: payload=%d record=%d total=%d\n", payloadLen, recordLen, totalLen);
  byte ndef[totalLen];                  // will hold TLV + record
  ndef[0] = 0x03;                       // NDEF message TLV
  ndef[1] = recordLen;                  // length of NDEF message
  ndef[2] = 0xD1;                       // MB/ME/SHORT/Type=0x01
  ndef[3] = 0x01;                       // type length
  ndef[4] = payloadLen;                 // payload length
  ndef[5] = 'T';                        // type 'T'
  ndef[6] = 0x02;                       // UTF-8, language length=2
  ndef[7] = 'e';
  ndef[8] = 'n';
  for (int i = 0; i < textLen; ++i)
    ndef[9 + i] = text[i];
  ndef[2 + recordLen] = 0xFE; // terminator TLV

  byte buffer[4];
  int page = 4;
  for (int i = 0; i < totalLen; i += 4) {
    for (int j = 0; j < 4; ++j) {
      int idx = i + j;
      buffer[j] = (idx < totalLen) ? ndef[idx] : 0x00;
    }

    RFID2_DEBUG_PRINT("Writing page %d: %02X %02X %02X %02X\n", page, buffer[0], buffer[1], buffer[2], buffer[3]);
    MFRC522_I2C::StatusCode status = static_cast<MFRC522_I2C::StatusCode>(
        rfid->MIFARE_Ultralight_Write(page, buffer, 4));
    if (status != MFRC522_I2C::STATUS_OK) {
      RFID2_DEBUG_PRINT("Write failed at page %d: %s\n", page,
                        rfid->GetStatusCodeName(status));

      if (errMsg)
        *errMsg = rfid->GetStatusCodeName(status);
      rfid2Halt();
      return false;
    }
    RFID2_DEBUG_PRINT("Page %d written\n", page);
    page++;
  }

  rfid2Halt();
  RFID2_DEBUG_PRINT("rfid2WriteText: complete\n");
  return true;
}

void rfid2Halt() {
  if (!initialized || rfid == nullptr) {
    return;
  }
  rfid->PICC_HaltA();
  rfid->PCD_StopCrypto1();
}

bool rfid2ReadText(String *out, String *errMsg, bool halt) {
  RFID2_DEBUG_PRINT("rfid2ReadText: start\n");
  if (!initialized || rfid == nullptr) {
    RFID2_DEBUG_PRINT("rfid2ReadText: not initialized\n");
    if (errMsg)
      *errMsg = F("not init");
    return false;
  }

  if (!waitForCard()) {
    if (errMsg)
      *errMsg = F("timeout");
    return false;
  }
  RFID2_DEBUG_PRINT("rfid2ReadText: card ready\n");

  // Read first 4 pages starting at page 4.
  byte buffer[18];
  byte size = sizeof(buffer);
  RFID2_DEBUG_PRINT("Reading page 4\n");
  MFRC522_I2C::StatusCode status = static_cast<MFRC522_I2C::StatusCode>(
      rfid->MIFARE_Read(4, buffer, &size));
  if (status != MFRC522_I2C::STATUS_OK) {
    RFID2_DEBUG_PRINT("Read failed at page 4: %s\n",
                      rfid->GetStatusCodeName(status));

    if (errMsg)
      *errMsg = rfid->GetStatusCodeName(status);
    if (halt)
      rfid2Halt();
    return false;
  }
  byte data[64];
  memcpy(data, buffer, 16);
  int needed = 2 + buffer[1] + 1; // TLV + message + terminator
  int readBytes = 16;
  int page = 8;
  while (readBytes < needed && readBytes < (int)sizeof(data)) {
    size = sizeof(buffer);
    RFID2_DEBUG_PRINT("Reading page %d\n", page);
    status = static_cast<MFRC522_I2C::StatusCode>(
        rfid->MIFARE_Read(page, buffer, &size));
    if (status != MFRC522_I2C::STATUS_OK) {
      RFID2_DEBUG_PRINT("Read failed at page %d: %s\n", page,
                        rfid->GetStatusCodeName(status));

      if (errMsg)
        *errMsg = rfid->GetStatusCodeName(status);
      if (halt)
        rfid2Halt();
      return false;
    }
    memcpy(data + readBytes, buffer, 16);
    readBytes += 16;
    page += 4;
  }

  if (halt)
    rfid2Halt();

  if (data[0] != 0x03 || data[2] != 0xD1 || data[3] != 0x01 || data[5] != 'T') {
    RFID2_DEBUG_PRINT("rfid2ReadText: no NDEF header\n");
    if (errMsg)
      *errMsg = F("no ndef");
    return false;
  }

  int payloadLen = data[4];
  int langLen = data[6] & 0x3F;
  if (payloadLen < 1 + langLen) {
    RFID2_DEBUG_PRINT("rfid2ReadText: bad payload len=%d lang=%d\n", payloadLen, langLen);
    if (errMsg)
      *errMsg = F("bad payload");
    return false;
  }
  int textStart = 7 + langLen;
  int textLen = payloadLen - 1 - langLen;
  String text = "";
  for (int i = 0; i < textLen; ++i)
    text += char(data[textStart + i]);

  RFID2_DEBUG_PRINT("rfid2ReadText: decoded '%s'\n", text.c_str());
  if (out)
    *out = text;
  return true;
}
