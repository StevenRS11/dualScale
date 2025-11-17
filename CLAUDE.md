# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-based dual load cell system designed for measuring golf club swing properties. The device uses two HX711 load cells to measure mass distribution and calculates Balance Point (BP) and Equivalent Swing Weight (ESW/MOI). It features NFC integration for automatic data writing to tags and supports both TFT and OLED displays.

## Hardware Configuration

**Load Cells (HX711):**
- Scale 1: DOUT=41, SCK=42 (head mass)
- Scale 2: DOUT=5, SCK=4 (handle mass)

**I2C Devices:**
- SDA: Pin 8
- SCL: Pin 9
- OLED: 0x3C (SSD1306 128x64)
- RFID2: 0x28 (MFRC522, RST=26)

**Other Pins:**
- Button: Pin 6 (currently unused)

## Display System

The project supports two display types controlled by `DISPLAY_TYPE_TFT` (0) and `DISPLAY_TYPE_OLED` (1) defines at the top of `dualScale.ino`. Currently configured for OLED.

The display abstraction is in the `Display` namespace (lines 70-114) with methods:
- `begin()` - Initialize display hardware
- `clear()` - Clear entire screen
- `printLine(y, text)` - Print text at y-coordinate

## Core Architecture

**Main Loop Structure:**
The system operates on two independent timers:
- `updateInterval` (2000ms): Updates load cell readings and display
- `nfcPollInterval` (2000ms): Checks for NFC tag presence

**Calibration System:**
- Uses 4-point calibration (0g, 100g, 200g, 300g) with linear regression
- Stores calibration factors and tare offsets in ESP32 NVS (namespace: "dualScale")
- Keys: "cal1", "cal2", "tare1", "tare2"

**Stable Reading Algorithm (`readStable`):**
Takes 10 samples, discards min/max, averages remaining 8 samples to reduce noise and provide stable readings even under vibration.

## NFC Integration (Rfid2.h/cpp)

The RFID2 module wraps MFRC522_I2C for M5Stack RFID2 unit:

**Auto-Write Behavior:**
When a tag is detected (rising edge), the system:
1. Attempts to read existing NDEF text from tag
2. If text is "CAL" → triggers calibration routine
3. If text is "TARE" → triggers tare operation
4. Otherwise → writes differential reading as "DS:{diff}" where diff = (head mass - handle mass)

**Key Functions:**
- `rfid2Begin()` - Initialize RFID hardware
- `waitForCard(timeout)` - Block until tag present or timeout
- `rfid2WriteText(text, errMsg)` - Write NDEF text record to tag
- `rfid2ReadText(out, errMsg, halt)` - Read NDEF text record from tag
- `rfid2Halt()` - Stop tag communication

**Debug Logging:**
Enable with `#define RFID2_DEBUG 1` in Rfid2.h for detailed RFID operation logging via Serial.

## Golf Club Calculations

**Balance Point (BP):** `(2 * handle + 13 * head) / (head + handle)`
- Result in inches from butt end
- Assumes 2" to handle scale, 13" to head scale

**Equivalent Swing Weight (ESW/MOI):** `(BP * 25.4) / 2.08`
- Converts BP from inches to mm, divides by constant
- Approximates moment of inertia

These formulas are in `calculate_BP()` and `estimate_MOI()` (lines 310-318).

## Display Update Optimization

The `updateReadings()` function uses differential updates to minimize flicker:
- Maintains static cache of last drawn strings (p0-p4)
- Only redraws lines that changed
- For OLED: erases specific row rectangles before redraw, calls `display.display()` once
- For TFT: repaints entire line (cheaper at 2s cadence)

## Calibration Workflow

Triggered by presenting NFC tag with "CAL" text:
1. Tares both scales
2. Prompts for 100g, 200g, 300g weights on scale 1 (advance with any NFC tap)
3. Prompts for 100g, 200g, 300g weights on scale 2 (advance with any NFC tap)
4. Calculates calibration factors using least-squares linear regression
5. Saves to NVS and applies immediately

Implementation in `calibrate()` (lines 320-364).

## Development Notes

**Arduino Environment:**
This is Arduino IDE compatible code for ESP32. Uses Arduino framework libraries.

**Required Libraries:**
- HX711 (load cell interface)
- MFRC522_I2C (NFC/RFID)
- Adafruit_GFX + Adafruit_SSD1306 (OLED) OR TFT_eSPI (TFT)
- Preferences (ESP32 NVS)
- Wire (I2C)

**No Build System:**
This is a standard Arduino sketch - compile/upload via Arduino IDE or PlatformIO.

**Serial Debugging:**
115200 baud. Extensive logging for NVS operations and RFID (when RFID2_DEBUG enabled).

## Important Constraints

- NFC writes occur only on rising edge of tag presence (once per tap)
- Maximum NDEF text length: 240 bytes (enforced in `rfid2WriteText`)
- Load cell timeout: 1000ms (if HX711 not ready)
- I2C clock: 400kHz
- All mass values internally in grams, displayed conversions to ounces (/28.35)
