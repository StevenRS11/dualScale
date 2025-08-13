#include "Rfid2.h"

#include <MFRC522.h>     // Base MFRC522 library for StatusCode definitions
#include <MFRC522_I2C.h>

// MFRC522 over I2C at address 0x28 used in the M5Stack RFID2 unit.
static MFRC522_I2C rfid(0x28, 26, &Wire);
static bool initialized = false;


bool rfid2Begin(TwoWire &w) {
  w.begin();
  rfid.PCD_Init(); // Initialize MFRC522

  initialized = true;
  return true; // Library does not expose an error code
}

static bool waitForCard() {
  unsigned long start = millis();
  while (true) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial())
      return true;
    if (millis() - start > 3000)
      return false;
    delay(50);
  }
}

bool rfid2WriteText(const String &text, String *errMsg) {
  if (!initialized) {
    if (errMsg)
      *errMsg = F("not init");
    return false;
  }

  if (!waitForCard()) {
    if (errMsg)
      *errMsg = F("timeout");
    return false;
  }

  // Build NDEF TLV for a simple text record in English.
  int textLen = text.length();
  if (textLen > 240) { // fits easily within Ultralight pages
    if (errMsg)
      *errMsg = F("too long");
    rfid.PICC_HaltA();
    return false;
  }
  const int payloadLen = textLen + 3;   // status + lang(2) + text
  const int recordLen = payloadLen + 4; // header bytes + type
  const int totalLen = recordLen + 3;   // TLV + terminator
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

    MFRC522::StatusCode status = rfid.MIFARE_Ultralight_Write(page++, buffer, 4);
    if (status != MFRC522::STATUS_OK) {
      if (errMsg)
        *errMsg = rfid.GetStatusCodeName(status);
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      return false;
    }
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return true;
}

bool rfid2ReadText(String *out, String *errMsg) {
  if (!initialized) {
    if (errMsg)
      *errMsg = F("not init");
    return false;
  }

  if (!waitForCard()) {
    if (errMsg)
      *errMsg = F("timeout");
    return false;
  }

  // Read first 4 pages starting at page 4.
  byte buffer[18];
  byte size = sizeof(buffer);
  MFRC522::StatusCode status = rfid.MIFARE_Read(4, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    if (errMsg)
      *errMsg = rfid.GetStatusCodeName(status);
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return false;
  }
  byte data[64];
  memcpy(data, buffer, 16);
  int needed = 2 + buffer[1] + 1; // TLV + message + terminator
  int readBytes = 16;
  int page = 8;
  while (readBytes < needed && readBytes < (int)sizeof(data)) {
    size = sizeof(buffer);
    status = rfid.MIFARE_Read(page, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
      if (errMsg)
        *errMsg = rfid.GetStatusCodeName(status);
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      return false;
    }
    memcpy(data + readBytes, buffer, 16);
    readBytes += 16;
    page += 4;
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  if (data[0] != 0x03 || data[2] != 0xD1 || data[3] != 0x01 || data[5] != 'T') {
    if (errMsg)
      *errMsg = F("no ndef");
    return false;
  }

  int payloadLen = data[4];
  int langLen = data[6] & 0x3F;
  if (payloadLen < 1 + langLen) {
    if (errMsg)
      *errMsg = F("bad payload");
    return false;
  }
  int textStart = 7 + langLen;
  int textLen = payloadLen - 1 - langLen;
  String text = "";
  for (int i = 0; i < textLen; ++i)
    text += char(data[textStart + i]);

  if (out)
    *out = text;
  return true;
}
