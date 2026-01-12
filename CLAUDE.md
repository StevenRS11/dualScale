# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-based dual load cell system designed for measuring paddle mass distribution. The device uses two HX711 load cells to measure head and handle mass, then calculates Balance Point (BP) and Equivalent Swing Weight (ESW/MOI). It features OLED display for real-time readings.

## Hardware Configuration

**Load Cells (HX711):**
- Scale 1 (head): DOUT=3, SCK=4
- Scale 2 (handle): DOUT=6, SCK=5

**I2C Devices:**
- Display I2C (Wire): SDA=8, SCL=7
- RFID I2C (Wire1): SDA=10, SCL=11
- OLED: 0x3C (SSD1306 128x64)

**Button:**
- Pin 9 with INPUT_PULLUP

## Core Architecture

### Main Loop Structure

The system operates on two independent timers (both 2000ms):
- `updateInterval`: Updates load cell readings and display every 2 seconds
- `nfcPollInterval`: (Currently unused, to be replaced with Payload library)

### Button Handling (Lines 217-238)

Current implementation uses state machine for long-press detection:

```cpp
// Button state tracking
unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;
const unsigned long BUTTON_HOLD_TIME = 3000;  // 3 seconds

// States:
// 1. Button pressed → start timer
// 2. Button held ≥ 3s → trigger tare
// 3. Button released < 3s → no action (available for new functionality)
```

**Current Behavior:**
- Long press (≥3 seconds): Triggers tare operation
- Short press (<3 seconds): No action (available for integration)

### Calibration System

Uses 4-point calibration (0g, 100g, 200g, 300g) with linear regression:

**Storage in ESP32 NVS** (namespace: "dualScale"):
- `cal1`, `cal2`: Calibration factors (float)
- `tare1`, `tare2`: Zero-load offsets (long)
- `bootCount`: Crash detection counter (clears calibration after 3 rapid boots)

**Trigger:** Hold button during power-on to enter calibration mode

**Process:**
1. Tares both scales
2. Prompts for 100g, 200g, 300g weights on scale 1 (advance with button press)
3. Prompts for 100g, 200g, 300g weights on scale 2 (advance with button press)
4. Calculates calibration factors using least-squares linear regression
5. Saves to NVS and applies immediately

Implementation in `calibrate()` (lines 451-533).

### Stable Reading Algorithm

`readStable(HX711& scale)` (lines 252-274):
- Takes 10 samples with 1ms delays
- Discards min and max values
- Averages remaining 8 samples
- Reduces noise and provides stable readings under vibration

### Display System

Display abstraction in `Display` namespace (lines 84-129):

**Methods:**
- `begin()` - Initialize OLED hardware
- `clear()` - Clear entire screen
- `printLine(y, text)` - Print text at y-coordinate

**Display Type Configuration:**
- Set via `DISPLAY_TYPE_TFT` (0) or `DISPLAY_TYPE_OLED` (1) defines
- Currently configured for OLED

### Display Update Optimization

`updateReadings()` (lines 276-359) uses differential updates to minimize flicker:

**Features:**
- Maintains static cache of last-drawn strings (p0-p4)
- Only redraws lines that changed
- Erases specific row rectangles before redraw
- Single `display.display()` call per update (efficient for OLED)

**Display Layout** (5 lines):
```
y=0:  Static Weight: X.XX oz
y=16: BP: X.X inches
y=32: ESW: X.X
y=48: Handle Mass: X.XX g
y=56: Head Mass: X.XX g
```

## Calculations

### Balance Point (BP)

Formula (line 441-445):
```cpp
float calculate_BP() {
  float head = lastVal1 / 28.35;    // Convert grams to ounces
  float handle = lastVal2 / 28.35;
  return (2 * handle + 13 * head) / (head + handle);
}
```

**Result:** Distance in inches from butt end
**Assumes:** 2" to handle scale center, 13" to head scale center

### Equivalent Swing Weight (ESW/MOI)

Formula (line 447-449):
```cpp
float estimate_MOI() {
  return (calculate_BP() * 25.4) / (2.08);
}
```

**Converts:** BP from inches to mm, divides by constant
**Result:** Approximation of moment of inertia

## Pin Definitions

```cpp
// Display I2C (Wire)
SCREEN_SDA = 8
SCREEN_SCL = 7

// RFID I2C (Wire1) - for future Payload library integration
RFID_SDA = 10
RFID_SCL = 11

// Load Cells
LOADCELL_DOUT1 = 3   // Head scale data
LOADCELL_SCK1 = 4    // Head scale clock
LOADCELL_DOUT2 = 6   // Handle scale data
LOADCELL_SCK2 = 5    // Handle scale clock

// Button
BUTTON = 9           // INPUT_PULLUP
```

## Setup Sequence (Lines 139-214)

1. Serial initialization (115200 baud)
2. Button pin configuration with INPUT_PULLUP
3. Check if button held during boot → calibration mode flag
4. NVS crash detection (bootCount)
5. Display initialization
6. RFID I2C initialization (Wire1) - currently placeholder
7. Load cells initialization
8. Tare both scales
9. Load calibration from NVS
10. Clear bootCount (successful boot)
11. Enter calibration mode if button was held

## Important Constraints

**Load Cell Timing:**
- HX711 timeout: 1000ms per scale
- Sample delay: 1ms between each of 10 samples (10ms total per readStable call)
- Two scales read sequentially = ~20ms per updateReadings()

**NFC Timing:**
- Wire1 I2C clock: 400kHz
- Currently using placeholder RFID2 library (to be replaced with Payload library)

**NVS Storage:**
- Namespace: "dualScale"
- Keys: cal1, cal2, tare1, tare2, bootCount
- All floating-point values validated for NaN/Inf before use

**Display:**
- OLED: 128x64 pixels, I2C 0x3C
- Text size 1 (5x7 pixels per character)
- Differential updates only (5 cached strings)

**Mass Values:**
- Internal: grams
- Display conversions: ounces (/28.35) for total weight and BP calculation

## Crash Detection

Boot loop protection (lines 153-172, 197-201):
- Increments `bootCount` on each boot
- If bootCount ≥ 3: clears all calibration data
- Reset to 0 after successful boot
- Prevents permanent boot loops from bad calibration values

## Future Integration: Payload Library

**Planned Changes:**
- Remove Rfid2.h/cpp (placeholder)
- Add `#include <PaddleDNA.h>`
- Use short button press to trigger measurement cycle
- Write HeadWeight and HandleWeight measurements to NFC tags
- Preserve long press for tare functionality

**I2C Bus Allocation:**
- Wire1 (SDA=10, SCL=11) already initialized for NFC
- Compatible with Payload library's NFC class

## Development Notes

**Arduino Environment:**
- Arduino IDE or PlatformIO compatible
- ESP32 Arduino Core 2.0+

**Required Libraries:**
- HX711 (Bogdan Necula's library) - load cell interface
- Adafruit_GFX + Adafruit_SSD1306 (OLED)
- Preferences (ESP32 NVS, built-in)
- Wire (I2C, built-in)
- PaddleDNA (Payload library for NFC measurements)

**Serial Debugging:**
- Baud: 115200
- Extensive logging for NVS operations, calibration, and load cell readings

## Common Operations

**Enter Calibration Mode:**
1. Hold button during power-on
2. Release when prompted
3. Follow on-screen instructions for weight placement

**Tare Scales:**
- Hold button for 3 seconds during normal operation

**Read Values:**
- Display automatically updates every 2 seconds
