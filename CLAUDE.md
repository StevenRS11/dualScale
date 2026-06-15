# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-S3-based dual load cell system designed for measuring paddle mass distribution. The device uses a single **NAU7802 dual-channel** load cell amplifier to measure head and handle mass, then calculates Balance Point (BP) and Equivalent Swing Weight (ESW/MOI). It features an OLED display for real-time readings and writes measurements to NFC tags via the PaddleDNA Payload library.

> **Migration note:** This project previously used two HX711 amplifiers (one per load cell). It now uses one NAU7802 that multiplexes a single ADC across two channels. See "Load Cell Reading (NAU7802)" below.

## Hardware Configuration

**Microcontroller:** ESP32-S3-ZERO (only **two** hardware I2C controllers: `Wire` = I2C0, `Wire1` = I2C1)

**Load Cell Amp (NAU7802, I2C 0x2A):**
- Bus: `Wire1` (time-shared — see below), SDA=3 (SDIO), SCL=4 (SCLK), DRDY=6 (optional, polling used instead)
- CH2 (`NAU7802_CHANNEL_2`) = **head** (load cell B)
- CH1 (`NAU7802_CHANNEL_1`) = **handle**

**I2C Devices:**
- OLED (SSD1306 128x64): `Wire`, SDA=8, SCL=7, addr 0x3C
- NFC reader (MFRC522): `Wire1` (time-shared), SDA=10, SCL=11, addr 0x28
- NAU7802: `Wire1` (time-shared), SDA=3, SCL=4, addr 0x2A

**Button:**
- Pin 9 with INPUT_PULLUP

### I2C Bus Topology & Time-Sharing

The ESP32-S3 has only two hardware I2C controllers, but the PCB wires three I2C
devices on three separate pin pairs. The OLED gets its own controller (`Wire`).
The NAU7802 and the NFC reader **share the `Wire1` controller** on different pins,
which works because the scale and the NFC are used in separate phases of the
workflow (measure first, then write the tag).

`selectBus(BusOwner who)` arbitrates `Wire1`:
- `BUS_NAU` → `Wire1.begin(3, 4)` at 400 kHz. NAU7802 config persists across the
  switch, so no re-`begin()` is needed.
- `BUS_NFC` → `Wire1.begin(10, 11)` at 400 kHz, then `nfc.begin(Wire1)` to re-init
  the reader.

`busOwner` tracks the current owner; `selectBus()` is a no-op if already selected.
`nauReadChannel()` always asserts `BUS_NAU` and the NFC state handlers assert
`BUS_NFC`, so the bus self-heals on every state transition (returning to IDLE and
reading the scale automatically pulls the bus back to the NAU).

> **Planned hardware change:** Eventually the RFID connector and the screen will
> share a single I2C bus (OLED + NFC together on `Wire`), freeing `Wire1` for the
> NAU7802 exclusively. That removes the `Wire1` time-sharing entirely — `selectBus()`
> and its `selectBus(BUS_*)` call sites can be deleted, NFC moves to `Wire`, and the
> NAU7802 stays on `Wire1` without re-pointing. Until that board revision exists, the
> time-share scheme above is required.

## Core Architecture

### Main Loop Structure

`loop()` runs the button state machine, then dispatches on `currentState` (the
measurement workflow). In IDLE, `handleIdleState()` refreshes the display every
`updateInterval` (500 ms) via `updateReadings()`.

### Button Handling

Uses a three-state button state machine (BTN_IDLE, BTN_PRESSED, BTN_HELD) for short/long press detection:

```cpp
// Button state tracking
ButtonState buttonState = BTN_IDLE;
unsigned long buttonPressStart = 0;
const unsigned long BUTTON_HOLD_TIME = 3000;  // 3 seconds
```

**Behavior depends on current machine state:**

**When IDLE:**
- Short press (<3 seconds): Start measurement cycle
- Long press (≥3 seconds): Tare scales

**When in NFC workflow (MEASURING, DISPLAY_RESULTS, WAITING_FOR_NFC, WRITING_NFC, RETRY_PROMPT):**
- Short press (<3 seconds): Ignored
- Long press (≥3 seconds): **Abort** - selects the NFC bus, halts the tag, hands the bus back to the NAU, shows "Cancelled", returns to IDLE

### Calibration System

Uses 4-point calibration (0g, 100g, 200g, 300g) with linear regression:

**Storage in ESP32 NVS** (redundant: namespaces "dualScale" + "dualScaleBak"):
- `cal1`, `cal2`: Calibration factors in counts/gram (float) — field "1" = handle (CH1), "2" = head (CH2)
- `tare1`, `tare2`: Zero-load offsets in raw counts (long) — same 1=handle / 2=head mapping
- `crc`: CRC32 over the `CalData` struct (excluding the crc field) for corruption detection
- `bootCount` (in "dualScale"): rapid-boot counter; skips loading calibration after 3 rapid boots (data is preserved, not erased)

**Trigger:** Hold button during power-on to enter calibration mode

**Process** (`calibrate()`):
1. Tares both channels
2. Prompts for 100g, 200g, 300g on the **HEAD** channel (CH2), advancing on button press
3. Prompts for 100g, 200g, 300g on the **HANDLE** channel (CH1)
4. Computes counts/gram slope per channel via least-squares regression; the
   regression intercept becomes the software tare offset for that channel
5. Saves to NVS (primary + backup) and applies immediately

> Old HX711-era NVS calibration data is invalid for the NAU7802 (different counts
> scale) and fails CRC/range validation — recalibrate after migrating.

### Load Cell Reading (NAU7802)

The NAU7802 multiplexes one ADC across two channels, so the two load cells
**cannot** be read simultaneously (unlike the old per-HX711 ring buffers).

`nauReadChannel(uint8_t channel, int nSamples, long &out)`:
1. `selectBus(BUS_NAU)` — ensure the amp owns `Wire1`
2. `nau.setChannel(channel)` (bails out, returning false, on I2C failure)
3. `nau.calibrateAFE()` — internal-mode AFE offset cal, **required after each channel
   switch**. Internal mode shorts the inputs internally, so it does NOT zero the
   external load; the no-load offset is handled by the software tare instead.
4. Discard `NAU_SETTLE_SAMPLES` (2) settling conversions
5. Return a min/max-trimmed average of up to `nSamples` conversions
   (`nauNextSample()` waits on `nau.available()` with a 500 ms timeout)

Sample-depth constants: `LIVE_SAMPLES`=5 (display), `MEAS_SAMPLES`=12 (final
measurement), `TARE_SAMPLES`=12 (tare). NAU config: `GAIN_128`, `SPS_40`, `LDO_3V3`.

Converting raw to grams: `grams = (raw - tareOffset) / calFactor` where
`calFactorHead`/`calFactorHandle` are counts/gram and `tareHead`/`tareHandle` are
the software zero-load offsets.

### Display System

Display abstraction in the `Display` namespace:

**Methods:**
- `begin()` - Initialize OLED hardware (on `Wire`)
- `clear()` - Clear entire screen
- `printLine(y, text)` - Print text at y-coordinate

**Display Type Configuration:**
- Set via `DISPLAY_TYPE_TFT` (0) or `DISPLAY_TYPE_OLED` (1) defines
- Currently configured for OLED

### Display Update Optimization

`updateReadings()` uses differential updates to minimize flicker, and reads only
**one NAU channel per call**, alternating head/handle each invocation. This keeps
the loop responsive (a full dual-channel read with two AFE calibrations would
otherwise block ~0.5 s). Both `lastVal1` (head) and `lastVal2` (handle) persist
between calls so the display always shows the latest of each.

**Features:**
- Maintains a static cache of last-drawn strings; only redraws lines that changed
- Erases specific row rectangles before redraw
- Single `display.display()` call per update (efficient for OLED)

**Display Layout (IDLE live view):**
```
y=0:  Weight: X.XX oz              (live total)
y=12: BP:X.X ESW:X.X               (live, only when weight > 5 oz)
y=28: [last] X.XX oz               (previous measurement)
y=40: BP:X.X ESW:X.X               (previous measurement)
```

## Calculations

### Balance Point (BP)

```cpp
float calculate_BP() {
  float head = lastVal1 / 28.35;    // Convert grams to ounces
  float handle = lastVal2 / 28.35;
  return (2 * handle + 13 * head) / (head + handle);
}
```

**Result:** Distance in inches from butt end
**Assumes:** 2" to handle scale center, 13" to head scale center
**Note:** `lastVal1` = head (CH2), `lastVal2` = handle (CH1).

### Equivalent Swing Weight (ESW/MOI)

```cpp
float estimate_MOI() {
  return (calculate_BP() * 25.4) / (2.08);
}
```

**Converts:** BP from inches to mm, divides by constant
**Result:** Approximation of moment of inertia

## Pin Definitions

```cpp
// OLED I2C (Wire, dedicated controller)
SCREEN_SDA = 8
SCREEN_SCL = 7

// NFC I2C (Wire1, time-shared with NAU7802)
RFID_SDA = 10
RFID_SCL = 11

// NAU7802 I2C (Wire1, time-shared with NFC)
NAU_SDA  = 3   // SDIO
NAU_SCL  = 4   // SCLK
NAU_DRDY = 6   // data-ready (optional; available() polling used)

// Button
BUTTON = 9     // INPUT_PULLUP
```

## Setup Sequence (`setup()`)

1. Serial initialization (115200 baud)
2. Button pin configuration with INPUT_PULLUP
3. Check if button held during boot → calibration mode flag
4. NVS crash detection (bootCount)
5. `scanI2CModules()` — probe OLED/NAU/NFC across the time-shared bus and print a
   structured connectivity report over serial; leaves `Wire1` on the NAU pins
6. Display initialization (`Wire`)
7. NAU7802 init (`Wire1` on NAU pins): `begin`, `setLDO`/`setGain`/`setSampleRate`, `calibrateAFE`
8. Payload/NFC init: `selectBus(BUS_NFC)` → `nfc.begin`, `crypto.begin`, allocate `MeasurementAccumulator`
9. `selectBus(BUS_NAU)` — return the shared bus to the load cell amp
10. Load calibration from NVS (skipped on boot-loop detection), then tare
11. Clear bootCount (successful boot)
12. Enter calibration mode if button was held

### I2C Module Report

`scanI2CModules()` prints a fixed-width table at boot so you can see which modules
ACK on the bus, e.g.:

```
================ I2C Module Report ================
  Module        Bus   Pins     Addr  Status
  ------------------------------------------------
  OLED Display  Wire  SDA8/7   0x3C  CONNECTED
  NAU7802 amp   Wire1 SDA3/4   0x2A  CONNECTED
  NFC MFRC522   Wire1 SDA10/11 0x28  -- MISSING --
  (NAU7802 + NFC time-share the Wire1 controller)
===================================================
```

## Important Constraints

**Load Cell Timing (NAU7802):**
- Sample rate SPS_40 (~25 ms/sample); `nauNextSample()` times out at 500 ms
- `calibrateAFE()` runs after every channel switch (every read alternates channels)
- A live `updateReadings()` reads one channel (~150–250 ms); measurement reads both

**I2C / Bus Sharing:**
- `Wire` clock 400 kHz (OLED); `Wire1` clock 400 kHz (NAU and NFC)
- `Wire1` is re-pointed between NAU (3/4) and NFC (10/11) via `selectBus()`
- Switching calls `Wire1.end()` then `Wire1.begin(pins)`; the off-bus device retains
  its chip configuration while idle

**NVS Storage:**
- Namespaces: "dualScale" (primary) + "dualScaleBak" (backup), CRC-protected
- Keys: cal1, cal2, tare1, tare2, crc, plus bootCount in "dualScale"
- All floating-point values validated for NaN/Inf/range before use

**Display:**
- OLED: 128x64 pixels, I2C 0x3C
- Text size 1 (5x7 pixels per character)
- Differential updates only (cached strings)

**Mass Values:**
- Internal: grams
- Display conversions: ounces (/28.35) for total weight and BP calculation

## Crash Detection

Boot loop protection:
- Increments `bootCount` on each boot
- If bootCount ≥ 3: **skips loading** calibration this boot (data is preserved, not erased)
- Reset to 0 after a successful boot
- Prevents permanent boot loops from bad calibration values

## Development Notes

**Arduino Environment:**
- Arduino IDE or arduino-cli, FQBN `esp32:esp32:esp32s3`
- ESP32 Arduino Core 2.0+

**Required Libraries:**
- SparkFun_Qwiic_Scale_NAU7802_Arduino_Library - load cell amp interface
- Adafruit_GFX + Adafruit_SSD1306 (OLED)
- Preferences (ESP32 NVS, built-in)
- Wire (I2C, built-in)
- PaddleDNA (Payload library for NFC measurements)

**Serial Debugging:**
- Baud: 115200
- Boot-time I2C module report; extensive logging for NVS operations, calibration, and NAU7802 reads

## Common Operations

**Enter Calibration Mode:**
1. Hold button during power-on
2. Release when prompted
3. Follow on-screen instructions for weight placement (HEAD channel first, then HANDLE)

**Tare Scales:**
- Hold button for 3 seconds while in IDLE state (live display mode)

**Take Measurement:**
1. Short press button (release before 3 seconds)
2. Wait for 4-second stabilization
3. Results displayed with BP/ESW/Mass
4. Present NFC tag when prompted (firmware switches `Wire1` to the NFC reader)
5. Tag is written with HeadWeight and HandleWeight measurements

**Abort Measurement/NFC Write:**
- Hold button for 3 seconds during any measurement or NFC operation
- Tag is released, operation cancelled, bus handed back to the NAU, returns to IDLE

**Read Values:**
- Display automatically updates every 500 ms in IDLE state
