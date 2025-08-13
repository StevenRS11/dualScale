#include "Rfid2.h"

#include <MFRC522_I2C.h>


// Avoid pulling in the SPI-based MFRC522 library from some NfcAdapter
// implementations. We already included the I2C variant above which defines
// the MFRC522 class and firmware reference tables, so prevent a second copy
// from being brought in that would redefine those symbols and break the
// build (seen as "redefinition of MFRC522_firmware_referenceV0_0" errors).
#ifndef MFRC522_h
#define MFRC522_h
#endif
#ifndef MFRC522_H
#define MFRC522_H
#endif

#include <NfcAdapter.h>
#include <NdefMessage.h>
#include <NdefRecord.h>

// MFRC522 over I2C at address 0x28 used in the M5Stack RFID2 unit.
static MFRC522 rfid(0x28);
static NfcAdapter *nfc = nullptr;

bool rfid2Begin(TwoWire &w) {
  rfid.PCD_Init();          // Initialize MFRC522
  nfc = new NfcAdapter(&rfid);
  nfc->begin(false);
  return true;              // Library does not expose an error code
}

bool rfid2WriteText(const String &text, String *errMsg) {
  if (!nfc) {
    if (errMsg)
      *errMsg = F("not init");
    return false;
  }

  unsigned long start = millis();
  while (!nfc->tagPresent()) {
    if (millis() - start > 3000) {
      if (errMsg)
        *errMsg = F("timeout");
      return false;
    }
    delay(50);
  }

  NdefMessage message;
  message.addTextRecord(text.c_str());
  bool ok = nfc->write(message);
  nfc->haltTag();
  if (!ok) {
    if (errMsg)
      *errMsg = F("write error");
  }
  return ok;
}

bool rfid2ReadText(String *out, String *errMsg) {
  if (!nfc) {
    if (errMsg)
      *errMsg = F("not init");
    return false;
  }

  unsigned long start = millis();
  while (!nfc->tagPresent()) {
    if (millis() - start > 3000) {
      if (errMsg)
        *errMsg = F("timeout");
      return false;
    }
    delay(50);
  }

  NfcTag tag = nfc->read();
  nfc->haltTag();

  if (!tag.hasNdefMessage()) {
    if (errMsg)
      *errMsg = F("no ndef");
    return false;
  }

  NdefMessage msg = tag.getNdefMessage();
  if (msg.getRecordCount() == 0) {
    if (errMsg)
      *errMsg = F("no record");
    return false;
  }

  NdefRecord rec = msg.getRecord(0);
  const byte *payload = rec.getPayload();
  int len = rec.getPayloadLength();
  if (len < 3) {
    if (errMsg)
      *errMsg = F("bad payload");
    return false;
  }
  uint8_t langLen = payload[0] & 0x3F;
  String text = "";
  for (int i = 1 + langLen; i < len; i++) {
    text += char(payload[i]);
  }

  if (out)
    *out = text;
  return true;
}

