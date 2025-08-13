#pragma once

#include <Arduino.h>
#include <Wire.h>

// Initialize the RFID2 unit. Returns true on success.
bool rfid2Begin();

// Write a text NDEF record to the tag. Returns true on success.
// When false is returned, errMsg (if provided) contains a short reason
// such as "timeout".
bool rfid2WriteText(const String &text, String *errMsg = nullptr);

// Read a text NDEF record from the tag. The retrieved text is stored in `out`.
// Returns true on success.
bool rfid2ReadText(String *out, String *errMsg = nullptr);

