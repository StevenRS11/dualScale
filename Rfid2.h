#pragma once

#include <Arduino.h>
#include <Wire.h>

// Enable verbose debug logging when RFID2_DEBUG is defined.
#define RFID2_DEBUG 1
#ifdef RFID2_DEBUG
#define RFID2_DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define RFID2_DEBUG_PRINT(...)
#endif

// Initialize the RFID2 unit. Returns true on success.
bool rfid2Begin(TwoWire &w = Wire);

bool waitForCard(int wait);

bool waitForCard();


// Write a text NDEF record to the tag. Returns true on success.
// When false is returned, errMsg (if provided) contains a short reason
// such as "timeout".
bool rfid2WriteText(const String &text, String *errMsg = nullptr);

// Read a text NDEF record from the tag. The retrieved text is stored in `out`.
// When `halt` is true (default), the tag is halted after reading; pass false if
// you plan to perform additional operations, such as a subsequent write.
bool rfid2ReadText(String *out, String *errMsg = nullptr, bool halt = true);

// Halt communication with the currently selected tag.
void rfid2Halt();

