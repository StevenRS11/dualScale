#pragma once

#include <Arduino.h>
#include <Wire.h>

// Initialize the RFID2 unit. Returns true on success.  Optionally allow the
// caller to specify the I2C interface, defaulting to the global `Wire`
// instance so existing call sites can simply invoke `rfid2Begin()`.
bool rfid2Begin(TwoWire &w = Wire);

// Write a text NDEF record to the tag. Returns true on success.
// When false is returned, errMsg (if provided) contains a short reason
// such as "timeout".
bool rfid2WriteText(const String &text, String *errMsg = nullptr);

// Read a text NDEF record from the tag. The retrieved text is stored in `out`.
// Returns true on success.
bool rfid2ReadText(String *out, String *errMsg = nullptr);

