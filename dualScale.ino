/*
  ESP32-S3 Dual Scale – NAU7802 dual-channel load cell amp + Auto NFC Write
  Reads a single NAU7802 dual-channel ADC (head = CH2 / load cell B,
  handle = CH1) and displays Balance Point / Equivalent Swing Weight.

  I2C topology (ESP32-S3 has only TWO hardware I2C controllers):
    Wire  (I2C0): OLED display only            -> SDA=8,  SCL=7
    Wire1 (I2C1): TIME-SHARED on demand:
                  - NAU7802 load cell amp      -> SDA=3,  SCL=4  (DRDY=6)
                  - NFC reader (MFRC522)       -> SDA=10, SCL=11
  The scale and the NFC are used in separate phases, so Wire1 is re-pointed
  to the active device's pins via selectBus(). See selectBus().
*/

#define DISPLAY_TYPE_TFT   0
#define DISPLAY_TYPE_OLED  1

// OLED I2C pins (Wire, dedicated)
#define SCREEN_SDA    8   // GP8
#define SCREEN_SCL    7   // GP7

// NFC I2C pins (Wire1, time-shared with NAU7802)
#define RFID_SDA      10  // GP10
#define RFID_SCL      11  // GP11

// NAU7802 I2C pins (Wire1, time-shared with NFC)
#define NAU_SDA       3   // GP3 (SDIO)
#define NAU_SCL       4   // GP4 (SCLK)
#define NAU_DRDY      6   // GP6 (data-ready; optional, we poll available())

#define OLED_ADDR 0x3C
#define NFC_ADDR  0x28   // MFRC522 on Wire1
#define NAU_ADDR  0x2A   // NAU7802 on Wire1

#define BUTTON  9


#include <Arduino.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <Preferences.h>
#include <PaddleDNA.h>

using namespace PaddleDNA;

// Include Wire library - Wire and Wire1 are both available on ESP32
#include <Wire.h>

#if DISPLAY_TYPE_TFT
  #include <SPI.h>
  #include <TFT_eSPI.h>
  TFT_eSPI tft = TFT_eSPI();
#elif DISPLAY_TYPE_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 64
  // Wire drives the OLED only; Wire1 is time-shared (NAU7802 / NFC)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

NAU7802 nau;
Preferences prefs;

// NAU7802 channel assignment (CH2 = head / load cell B, CH1 = handle)
#define CH_HEAD    NAU7802_CHANNEL_2
#define CH_HANDLE  NAU7802_CHANNEL_1
#define NAU_I2C_HZ 400000

// Sampling depth (trimmed average; first NAU_SETTLE_SAMPLES are discarded
// after each channel switch + AFE calibration)
const int NAU_SETTLE_SAMPLES = 2;
const int LIVE_SAMPLES       = 5;   // per channel, live display
const int MEAS_SAMPLES       = 12;  // per channel, final measurement
const int TARE_SAMPLES       = 12;  // per channel, tare

// Wire1 is a single hardware controller shared between the NAU7802 and the
// NFC reader, which live on different pins. selectBus() re-points it.
enum BusOwner { BUS_NONE, BUS_NAU, BUS_NFC };
BusOwner busOwner = BUS_NONE;
void selectBus(BusOwner who);

// PaddleDNA Payload library
NFC nfc;
Crypto crypto;
MeasurementAccumulator* accumulator = nullptr;

// Machine credentials (TODO: load from NVS in production)
const uint8_t MACHINE_UUID[16] = {
  0x68, 0xdf, 0x84, 0x98, 0x85, 0x73, 0x46, 0xc6,
  0xa8, 0xb8, 0xfe, 0xdc, 0xc0, 0xdf, 0x07, 0x36
};

const uint8_t PRIVATE_KEY[32] = {
  0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
  0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
  0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
  0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB
};

// Calibration factors (counts per gram) and zero-load offsets (raw counts).
// The NAU7802 has no persistent hardware offset register we rely on, so the
// tare is tracked in software and subtracted at read time.
float calFactorHead   = 1.0f;   // CH2 (load cell B)
float calFactorHandle = 1.0f;   // CH1
long  tareHead   = 0;           // CH2 zero-load raw
long  tareHandle = 0;           // CH1 zero-load raw

// True only while calibrate() is running. When set, nauReadChannel() echoes
// every trimmed-average raw reading over serial for cal debugging.
bool  calActive = false;

// Calibration data structure for redundant NVS storage.
// Field "1" = handle (CH1), field "2" = head (CH2).
struct CalData {
  float cal1;
  float cal2;
  long  tare1;
  long  tare2;
  uint32_t crc;
};

// CRC32 (same polynomial as PNG/ZIP)
static uint32_t calCrc32(const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

static bool writeCalToNamespace(const char* ns, const CalData& d) {
  Preferences p;
  if (!p.begin(ns, false)) {
    Serial.printf("[NVS] begin(%s, rw) failed\n", ns);
    return false;
  }
  p.putFloat("cal1", d.cal1);
  p.putFloat("cal2", d.cal2);
  p.putLong("tare1", d.tare1);
  p.putLong("tare2", d.tare2);
  p.putULong("crc", d.crc);
  p.end();
  return true;
}

static bool readCalFromNamespace(const char* ns, CalData& d) {
  Preferences p;
  if (!p.begin(ns, true)) {
    Serial.printf("[NVS] begin(%s, ro) failed\n", ns);
    return false;
  }
  d.cal1  = p.getFloat("cal1", NAN);
  d.cal2  = p.getFloat("cal2", NAN);
  d.tare1 = p.getLong("tare1", LONG_MIN);
  d.tare2 = p.getLong("tare2", LONG_MIN);
  d.crc   = p.getULong("crc", 0);
  p.end();
  return true;
}

static bool calDataValid(const CalData& d) {
  // Check CRC over everything except the crc field itself
  uint32_t expected = calCrc32(&d, offsetof(CalData, crc));
  if (d.crc != expected) return false;
  // Range/NaN checks
  if (isnan(d.cal1) || isinf(d.cal1) || d.cal1 < 0.1f || d.cal1 > 1000000.0f) return false;
  if (isnan(d.cal2) || isinf(d.cal2) || d.cal2 < 0.1f || d.cal2 > 1000000.0f) return false;
  if (d.tare1 == LONG_MIN || d.tare2 == LONG_MIN) return false;
  return true;
}

// Display update timing
const unsigned long updateInterval = 500;    // display refresh cadence

unsigned long lastUpdate = 0;

float lastVal1 = 0.0f;   // head (grams)
float lastVal2 = 0.0f;   // handle (grams)

// Measurement workflow state machine
enum MachineState {
  IDLE,               // Live display mode
  MEASURING,          // 4-second stabilization
  DISPLAY_RESULTS,    // Show BP/ESW/Mass, prompt for NFC
  WAITING_FOR_NFC,    // Polling for tag
  WRITING_NFC,        // Writing measurements
  WRITE_SUCCESS,      // Brief success message
  WRITE_FAILED,       // Brief failure message
  RETRY_PROMPT        // "Retry X/5" - waiting for tag re-presentation
};

MachineState currentState = IDLE;
unsigned long stateStartTime = 0;
uint32_t measurementTimestamp = 0;

// Captured measurements
float measuredHeadWeight = 0.0f;
float measuredHandleWeight = 0.0f;
float calculatedBalancePoint = 0.0f;
float calculatedSwingWeight = 0.0f;
float calculatedMass = 0.0f;

// Last completed measurement (for comparative display)
bool hasLastMeasurement = false;
float lastMeasMass = 0.0f;
float lastMeasBP = 0.0f;
float lastMeasESW = 0.0f;

// NFC retry tracking
int nfcRetryCount = 0;
const int MAX_NFC_RETRIES = 5;
const unsigned long NFC_TIMEOUT = 30000;        // 30 seconds
const unsigned long STABILIZATION_TIME = 4000;  // 4 seconds

// Button state for short/long press detection
enum ButtonState {
  BTN_IDLE,
  BTN_PRESSED,
  BTN_HELD
};

ButtonState buttonState = BTN_IDLE;
unsigned long buttonPressStart = 0;
const unsigned long BUTTON_HOLD_TIME = 3000;
 
bool nauReadChannel(uint8_t channel, int nSamples, long &out);
void updateReadings();
void saveCalibration();
void loadCalibration();
void tare();
void calibrate();
void perform_test();
bool waitForButtonPress();
void handleIdleState();
void handleMeasuringState();
void handleDisplayResultsState();
void handleWaitingForNfcState();
void handleWritingNfcState();
void handleWriteError(AccumulateResult result, const String& msg);
void handleRetryPromptState();
void handleWriteSuccessState();
void handleWriteFailedState();

namespace Display {
  void begin() {
  #if DISPLAY_TYPE_TFT
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
  #elif DISPLAY_TYPE_OLED
    Wire.begin(SCREEN_SDA, SCREEN_SCL);
    Wire.setClock(400000);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
      Serial.println(F("SSD1306 allocation failed"));
      for (;;)
        delay(100);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
  #endif
  }

  void clear() {
  #if DISPLAY_TYPE_TFT
    tft.fillScreen(TFT_BLACK);
  #else
    display.clearDisplay();
  #endif
  }

  void printLine(int16_t y, const String &text) {
  #if DISPLAY_TYPE_TFT
    tft.setCursor(4, y);
    tft.setTextFont(1);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(text);
  #else
    display.setCursor(0, y);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.print(text);
    display.display();
  #endif
 }
}

bool statusShown = false;  // track when showStatus has drawn to the screen

void showStatus(const String &line1, const String &line2 = String()) {
  Display::clear();
  Display::printLine(0, line1);
  if (line2.length()) {
    Display::printLine(16, line2);
  }
  statusShown = true;
}

// ---- I2C module connectivity reporting ----

// Returns true if a device ACKs at the given address on the given bus.
static bool i2cProbe(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

// Print one fixed-width row of the I2C module report.
static void reportI2CModule(const char *name, const char *busName,
                            const char *pins, uint8_t addr, bool present) {
  Serial.printf("  %-13s %-5s %-7s 0x%02X  %s\n",
                name, busName, pins, addr,
                present ? "CONNECTED" : "-- MISSING --");
}

// Re-point the time-shared Wire1 controller between the NAU7802 and the NFC
// reader (they sit on different pins of the same hardware I2C peripheral).
void selectBus(BusOwner who) {
  if (busOwner == who) return;
  Wire1.end();
  delay(2);
  if (who == BUS_NAU) {
    Wire1.begin(NAU_SDA, NAU_SCL);
    Wire1.setClock(NAU_I2C_HZ);
    // NAU7802 retains its configuration across the bus idling, so no re-begin.
  } else {  // BUS_NFC
    Wire1.begin(RFID_SDA, RFID_SCL);
    Wire1.setClock(400000);
    nfc.begin(Wire1);   // re-init the reader now that the bus points at it
  }
  busOwner = who;
  delay(2);
}

// Probe every known module (across the time-shared Wire1) and print a
// structured connectivity report. Leaves Wire1 pointed at the NAU7802.
static bool g_oledPresent = false;
static bool g_nfcPresent  = false;
static bool g_nauPresent  = false;

static void scanI2CModules() {
  // OLED lives alone on its own controller (Wire)
  Wire.begin(SCREEN_SDA, SCREEN_SCL);
  Wire.setClock(400000);

  // Wire1 is time-shared: probe the NAU pins first...
  Wire1.begin(NAU_SDA, NAU_SCL);
  Wire1.setClock(NAU_I2C_HZ);
  delay(50);
  g_oledPresent = i2cProbe(Wire, OLED_ADDR);
  g_nauPresent  = i2cProbe(Wire1, NAU_ADDR);

  // ...then re-point Wire1 to the NFC pins and probe there
  Wire1.end();
  Wire1.begin(RFID_SDA, RFID_SCL);
  Wire1.setClock(400000);
  delay(20);
  g_nfcPresent = i2cProbe(Wire1, NFC_ADDR);

  // Leave Wire1 on the NAU for normal (idle) operation
  Wire1.end();
  Wire1.begin(NAU_SDA, NAU_SCL);
  Wire1.setClock(NAU_I2C_HZ);
  busOwner = BUS_NAU;

  Serial.println();
  Serial.println("================ I2C Module Report ================");
  Serial.println("  Module        Bus   Pins     Addr  Status");
  Serial.println("  ------------------------------------------------");
  reportI2CModule("OLED Display", "Wire",  "SDA8/7",   OLED_ADDR, g_oledPresent);
  reportI2CModule("NAU7802 amp",  "Wire1", "SDA3/4",   NAU_ADDR,  g_nauPresent);
  reportI2CModule("NFC MFRC522",  "Wire1", "SDA10/11", NFC_ADDR,  g_nfcPresent);
  Serial.println("  (NAU7802 + NFC time-share the Wire1 controller)");
  Serial.println("===================================================");
  Serial.println();
  Serial.flush();  // push the report out before the heavier init work
}

void setup() {
  Serial.begin(115200);
  // ESP32-S3 uses native USB CDC: anything printed before the host enumerates
  // the port is silently dropped. Wait (briefly) for the connection so the
  // boot-time I2C report and module init logs aren't lost.
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 2000) {
    delay(10);
  }
  delay(300);  // extra margin for the CDC host to attach
  Serial.println("Init start");

  // Configure button pin with internal pull-up
  pinMode(BUTTON, INPUT_PULLUP);

  // Check if button is held down during boot -> enter calibration mode
  bool calibrationMode = (digitalRead(BUTTON) == LOW);
  if (calibrationMode) {
    Serial.println("Button held during boot - will enter calibration mode after init");
  }

  // Check for repeated boot loops (crash detection)
  bool skipCalibration = false;
  if (!prefs.begin("dualScale", false)) {
    Serial.println("[NVS] begin() failed in setup");
  } else {
    uint32_t bootCount = prefs.getUInt("bootCount", 0);
    bootCount++;
    prefs.putUInt("bootCount", bootCount);
    Serial.printf("Boot count: %u\n", bootCount);

    // If we've rebooted 3+ times rapidly, skip loading calibration this boot
    // but do NOT delete it — the data may still be valid once the root cause is fixed
    if (bootCount >= 3) {
      Serial.println("Multiple rapid boots detected - skipping calibration this boot");
      prefs.putUInt("bootCount", 0);
      skipCalibration = true;
    }
    prefs.end();
  }

  // Probe all I2C modules (across the time-shared Wire1) and print a report.
  // Leaves Wire1 pointed at the NAU7802 (busOwner == BUS_NAU).
  scanI2CModules();

  Serial.println("Display Init start");
  Display::begin();
  Serial.println("Display Init finish");

  // NAU7802 load cell amp (Wire1 currently on NAU pins from scanI2CModules)
  Serial.println("NAU7802 Init Start");
  if (!nau.begin(Wire1)) {
    Serial.println("NAU7802 init failed - load cell amp not detected");
    showStatus("Load cell err", "NAU7802 missing");
  } else {
    nau.setLDO(NAU7802_LDO_3V3);
    nau.setGain(NAU7802_GAIN_128);
    nau.setSampleRate(NAU7802_SPS_40);   // medium rate, good noise rejection
    nau.calibrateAFE();                  // internal offset cal
    Serial.println("NAU7802 initialized successfully");
  }

  // Initialize Payload library (re-point Wire1 to the NFC pins)
  Serial.println("Payload Init Start");
  selectBus(BUS_NFC);          // Wire1 -> RFID pins + nfc.begin()
  delay(100);                  // stabilize I2C before talking to the reader
  nfc.setDebug(true);
  if (!nfc.begin(Wire1)) {
    Serial.println("NFC init failed");
    showStatus("NFC Error", "Init failed");
  } else {
    Serial.println("NFC initialized successfully");
  }

  if (!crypto.begin(MACHINE_UUID, PRIVATE_KEY)) {
    Serial.println("Crypto init failed");
    showStatus("Crypto Error", "Init failed");
  } else {
    Serial.println("Crypto initialized successfully");
  }

  accumulator = new MeasurementAccumulator(nfc, crypto, 9);
  Serial.println("Payload Init Finish");

  // Return the shared bus to the NAU7802 for live operation
  selectBus(BUS_NAU);

  if (skipCalibration) {
    Serial.println("[NVS] Skipping calibration load due to boot loop detection");
  } else {
    loadCalibration();
  }
  tare();

  // If we made it here, clear boot count (successful boot)
  if (prefs.begin("dualScale", false)) {
    prefs.putUInt("bootCount", 0);
    prefs.end();
  }

  Serial.println("Setup complete");

  // Enter calibration mode if button was held during boot
  if (calibrationMode) {
    delay(500);  // Wait for button to be released
    while (digitalRead(BUTTON) == LOW) {
      delay(10);  // Wait for user to release button
    }
    Serial.println("Entering calibration mode...");
    calibrate();
  }
}

void loop() {
  // Button monitoring with short/long press detection
  bool buttonPressed = (digitalRead(BUTTON) == LOW);

  switch (buttonState) {
    case BTN_IDLE:
      if (buttonPressed) {
        buttonPressStart = millis();
        buttonState = BTN_PRESSED;
      }
      break;

    case BTN_PRESSED:
      if (!buttonPressed) {
        // Released before hold time - SHORT PRESS
        if (currentState == IDLE) {
          Serial.println("Short press - starting measurement");
          currentState = MEASURING;
          stateStartTime = millis();
          measurementTimestamp = millis() / 1000;  // Simple timestamp
        }
        buttonState = BTN_IDLE;
      } else if (millis() - buttonPressStart >= BUTTON_HOLD_TIME) {
        // LONG PRESS
        if (currentState == IDLE) {
          // In IDLE: TARE
          Serial.println("Long press - taring");
          tare();
          buttonState = BTN_HELD;
        } else {
          // In any other state (NFC workflow): ABORT
          Serial.println("Long press - aborting operation");
          selectBus(BUS_NFC);  // ensure the reader is addressable before halt
          nfc.halt();          // Release tag if one is selected
          selectBus(BUS_NAU);  // hand the shared bus back to the load cell amp
          showStatus("Cancelled");
          delay(1000);
          currentState = IDLE;
          lastUpdate = 0;
          buttonState = BTN_HELD;
        }
      }
      break;

    case BTN_HELD:
      if (!buttonPressed) {
        buttonState = BTN_IDLE;
      }
      break;
  }

  // State machine handler
  switch (currentState) {
    case IDLE:
      handleIdleState();
      break;
    case MEASURING:
      handleMeasuringState();
      break;
    case DISPLAY_RESULTS:
      handleDisplayResultsState();
      break;
    case WAITING_FOR_NFC:
      handleWaitingForNfcState();
      break;
    case WRITING_NFC:
      handleWritingNfcState();
      break;
    case WRITE_SUCCESS:
      handleWriteSuccessState();
      break;
    case WRITE_FAILED:
      handleWriteFailedState();
      break;
    case RETRY_PROMPT:
      handleRetryPromptState();
      break;
  }
}

void handleIdleState() {
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();
    updateReadings();
  }
}

void handleMeasuringState() {
  unsigned long elapsed = millis() - stateStartTime;

  // Update display every 500ms with progress
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 500) {
    int dotsCount = (elapsed / 500) % 4;
    String dots = "";
    for (int i = 0; i < dotsCount; i++) dots += ".";

    Display::clear();
    Display::printLine(16, "Measuring" + dots);
    Display::printLine(32, String(elapsed / 1000) + "s / 4s");
    #if DISPLAY_TYPE_OLED
      display.display();
    #endif
    lastDisplayUpdate = millis();
  }

  // After 4 seconds, take final measurement
  if (elapsed >= STABILIZATION_TIME) {
    long rawHead = 0, rawHandle = 0;
    bool okHead   = nauReadChannel(CH_HEAD,   MEAS_SAMPLES, rawHead);
    bool okHandle = nauReadChannel(CH_HANDLE, MEAS_SAMPLES, rawHandle);

    if (!okHead || !okHandle) {
      showStatus("Measurement", "failed");
      delay(2000);
      currentState = IDLE;
      lastUpdate = 0;
      return;
    }

    // Capture measurements (subtract software tare, divide by counts/gram)
    measuredHeadWeight   = (rawHead   - tareHead)   / calFactorHead;
    measuredHandleWeight = (rawHandle - tareHandle) / calFactorHandle;

    // Calculate display values
    lastVal1 = measuredHeadWeight;
    lastVal2 = measuredHandleWeight;
    calculatedBalancePoint = calculate_BP();
    calculatedSwingWeight = estimate_MOI();
    calculatedMass = (measuredHeadWeight + measuredHandleWeight) / 28.35;

    Serial.printf("Measurements: Head=%.2fg Handle=%.2fg\n",
                  measuredHeadWeight, measuredHandleWeight);

    // Save as last measurement for comparative display
    hasLastMeasurement = true;
    lastMeasMass = calculatedMass;
    lastMeasBP = calculatedBalancePoint;
    lastMeasESW = calculatedSwingWeight;

    currentState = DISPLAY_RESULTS;
    stateStartTime = millis();
  }
}

void handleDisplayResultsState() {
  // Only update display once when first entering this state
  static unsigned long lastStateEntry = 0;

  if (stateStartTime != lastStateEntry) {
    // Display results and prompt for NFC
    Display::clear();
    Display::printLine(0, String("BP: ") + String(calculatedBalancePoint, 1));
    Display::printLine(12, String("ESW: ") + String(calculatedSwingWeight, 1));
    Display::printLine(24, String("Mass: ") + String(calculatedMass, 1));
    Display::printLine(36, "");
    Display::printLine(48, "Present NFC");
    #if DISPLAY_TYPE_OLED
      display.display();
    #endif
    lastStateEntry = stateStartTime;

    // Immediately transition to waiting for NFC
    currentState = WAITING_FOR_NFC;
    stateStartTime = millis();
    nfcRetryCount = 0;
  }
}

void handleWaitingForNfcState() {
  // Hand the shared Wire1 controller to the NFC reader (no-op if already there)
  selectBus(BUS_NFC);

  // Check for timeout
  if (millis() - stateStartTime >= NFC_TIMEOUT) {
    Serial.println("NFC timeout");
    currentState = IDLE;
    lastUpdate = 0;
    return;
  }

  // Poll for tag every 250ms
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll < 250) return;
  lastPoll = millis();

  if (nfc.waitForTag(100)) {
    Serial.println("Tag detected - writing measurements");
    currentState = WRITING_NFC;
    stateStartTime = millis();
  }
}

void handleWritingNfcState() {
  // Only execute write logic once when first entering this state
  static unsigned long lastStateEntry = 0;

  selectBus(BUS_NFC);  // ensure the reader owns the shared bus before writing

  if (stateStartTime != lastStateEntry) {
    // State just entered - update display and perform write
    Display::clear();
    Display::printLine(24, "Writing...");
    #if DISPLAY_TYPE_OLED
      display.display();
    #endif
    lastStateEntry = stateStartTime;

    // Create measurements array for batch write
    Measurement measurements[2] = {
      Measurement(MeasurementType::HeadWeight, MACHINE_UUID, measurementTimestamp, measuredHeadWeight),
      Measurement(MeasurementType::HandleWeight, MACHINE_UUID, measurementTimestamp, measuredHandleWeight)
    };

    // Write both measurements atomically
    String msg;
    AccumulateResult result = accumulator->accumulateBatch(measurements, 2, &msg);

    if (result == AccumulateResult::Success) {
      Serial.printf("Success! Tag has %d measurements\n",
                    accumulator->getCurrentCount());
      nfc.halt();  // Redundant but safe (accumulateBatch already halts)
      currentState = WRITE_SUCCESS;
      stateStartTime = millis();
      return;
    }

    // Handle errors
    nfc.halt();  // Redundant but safe (accumulateBatch already halts)
    handleWriteError(result, msg);
  }
  // Else: Already processed this state entry, do nothing until state changes
}

void handleWriteError(AccumulateResult result, const String& msg) {
  Serial.printf("Write error: %d - %s\n", (int)result, msg.c_str());

  switch (result) {
    case AccumulateResult::TagFull:
      Display::clear();
      Display::printLine(24, "Tag Full!");
      #if DISPLAY_TYPE_OLED
        display.display();
      #endif
      delay(2000);
      currentState = IDLE;
      lastUpdate = 0;
      break;

    default:
      // Retry if under limit
      if (nfcRetryCount < MAX_NFC_RETRIES) {
        nfcRetryCount++;
        currentState = RETRY_PROMPT;
        stateStartTime = millis();
      } else {
        currentState = WRITE_FAILED;
        stateStartTime = millis();
      }
      break;
  }
}

void handleRetryPromptState() {
  // Only update display once when first entering this state
  static unsigned long lastStateEntry = 0;
  static bool tagWasRemoved = false;

  selectBus(BUS_NFC);  // tag polling happens here; keep the reader on the bus

  if (stateStartTime != lastStateEntry) {
    // State just changed - redraw display
    Display::clear();
    Display::printLine(8, String("Retry ") + String(nfcRetryCount) + "/" + String(MAX_NFC_RETRIES));
    Display::printLine(24, "Remove tag,");
    Display::printLine(36, "then re-present");
    #if DISPLAY_TYPE_OLED
      display.display();
    #endif

    lastStateEntry = stateStartTime;
    tagWasRemoved = false;  // Reset flag for new retry attempt
  }

  // Wait for tag removal then re-presentation
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 250) return;
  lastCheck = millis();

  if (!nfc.waitForTag(100)) {
    if (!tagWasRemoved) {
      // Tag just removed - halt it
      nfc.halt();
    }
    tagWasRemoved = true;
  } else if (tagWasRemoved) {
    Serial.println("Tag re-presented - retrying write");
    tagWasRemoved = false;
    currentState = WRITING_NFC;
    stateStartTime = millis();
  }
}

void handleWriteSuccessState() {
  // Only update display once when first entering this state
  static unsigned long lastStateEntry = 0;

  if (stateStartTime != lastStateEntry) {
    showStatus("Success!");
    #if DISPLAY_TYPE_OLED
      display.display();
    #endif
    lastStateEntry = stateStartTime;
  }

  if (millis() - stateStartTime >= 2000) {
    currentState = IDLE;
    lastUpdate = 0;
  }
}

void handleWriteFailedState() {
  // Only update display once when first entering this state
  static unsigned long lastStateEntry = 0;

  if (stateStartTime != lastStateEntry) {
    Display::clear();
    Display::printLine(24, "Failed");
    #if DISPLAY_TYPE_OLED
      display.display();
    #endif
    lastStateEntry = stateStartTime;
  }

  if (millis() - stateStartTime >= 3000) {
    currentState = IDLE;
    lastUpdate = 0;
  }
}

// Wait (with timeout) for the next NAU7802 conversion and return it.
// Returns false if no sample arrives within timeoutMs.
static bool nauNextSample(long &out, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (!nau.available()) {
    if (millis() - t0 > timeoutMs) return false;
    delay(1);  // yield to avoid watchdog reset
  }
  out = nau.getReading();
  return true;
}

// Read one NAU7802 channel: switch to it, recalibrate the AFE (internal
// offset cal — does not disturb the external load), discard settling
// conversions, then return a min/max-trimmed average of nSamples.
// Returns false on I2C/timeout failure.
bool nauReadChannel(uint8_t channel, int nSamples, long &out) {
  out = 0;
  selectBus(BUS_NAU);                 // ensure the amp owns the shared bus
  if (!nau.setChannel(channel)) {
    Serial.println("NAU7802 setChannel failed");
    return false;
  }
  // The NAU7802's CH2 input pin is shared with the PGA bypass-cap ("CAP") pin.
  // With PGA_CAP_EN set (the default), that cap loads/attenuates CH2, so CH2
  // reads less per unit load than CH1. Clear it on every channel switch (not
  // done by setChannel) so both channels have matched sensitivity. Must precede
  // calibrateAFE so the AFE offset cal reflects the cap-disabled config.
  nau.clearBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);
  nau.calibrateAFE();                 // required after each channel switch

  // Discard the first few conversions after the switch (settling)
  long discard;
  for (int i = 0; i < NAU_SETTLE_SAMPLES; ++i) {
    if (!nauNextSample(discard, 500)) {
      Serial.println("NAU7802 not ready (settle)");
      return false;
    }
  }

  long minVal = LONG_MAX, maxVal = LONG_MIN, sum = 0;
  int got = 0;
  for (int i = 0; i < nSamples; ++i) {
    long v;
    if (!nauNextSample(v, 500)) break;
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
    sum += v;
    got++;
  }
  if (got < 3) {
    Serial.println("NAU7802 insufficient samples");
    return false;
  }
  sum -= minVal + maxVal;             // trim outliers
  out = sum / (got - 2);
  if (calActive) {
    Serial.printf("[CAL] %s raw=%ld (n=%d min=%ld max=%ld)\n",
                  (channel == CH_HEAD) ? "HEAD" : "HANDLE",
                  out, got, minVal, maxVal);
  }
  return true;
}

void updateReadings() {
  // The NAU7802 multiplexes a single ADC, so we can't sample both channels at
  // once. Read ONE channel per call, alternating, to keep the loop responsive.
  // lastVal1 = head (CH2), lastVal2 = handle (CH1); both persist between calls.
  static uint8_t liveToggle = 0;
  static bool headReady = false, handleReady = false;

  long raw;
  if (liveToggle == 0) {
    headReady = nauReadChannel(CH_HEAD, LIVE_SAMPLES, raw);
    if (headReady) lastVal1 = (raw - tareHead) / calFactorHead;
  } else {
    handleReady = nauReadChannel(CH_HANDLE, LIVE_SAMPLES, raw);
    if (handleReady) lastVal2 = (raw - tareHandle) / calFactorHandle;
  }
  liveToggle ^= 1;

  #if DISPLAY_TYPE_OLED
  if (!headReady && !handleReady) {
    showStatus("Load cell", "not ready");
    return;
  }
  #endif

  // Validate calibration factors to prevent NaN
  if (calFactorHead < 0.1f || calFactorHead > 1000000.0f || isnan(calFactorHead) || isinf(calFactorHead)) {
    showStatus("Invalid cal", "Head - recal");
    Serial.printf("Invalid calFactorHead: %.2f\n", calFactorHead);
    return;
  }
  if (calFactorHandle < 0.1f || calFactorHandle > 1000000.0f || isnan(calFactorHandle) || isinf(calFactorHandle)) {
    showStatus("Invalid cal", "Handle - recal");
    Serial.printf("Invalid calFactorHandle: %.2f\n", calFactorHandle);
    return;
  }

  float val1 = lastVal1;  // head (grams)
  float val2 = lastVal2;  // handle (grams)

  // Check for invalid results
  if (isnan(val1) || isinf(val1) || isnan(val2) || isinf(val2)) {
    showStatus("Scale error", "NaN/Inf result");
    Serial.printf("Invalid result: head=%.2f handle=%.2f\n", val1, val2);
    return;
  }

  // If showStatus was used, do a full clear and invalidate cache
  // so no stale pixels remain from status text at different y positions
  static String p0, p1, p2, p3;
  if (statusShown) {
    Display::clear();
    #if DISPLAY_TYPE_OLED
      display.display();
    #endif
    p0 = p1 = p2 = p3 = "";
    statusShown = false;
  }

  // Precompute strings for live readings
  float staticWeightOz = (val1 + val2) / 28.35;
  String l0 = String("Weight: ") + String(staticWeightOz, 2) + " oz";
  String l1;
  if (staticWeightOz > 5.0f) {
    l1 = String("BP:") + String(calculate_BP(), 1) + " ESW:" + String(estimate_MOI(), 1);
  } else {
    l1 = "";
  }

  // Last measurement lines
  String l2, l3;
  if (hasLastMeasurement) {
    l2 = String("[last] ") + String(lastMeasMass, 2) + " oz";
    l3 = String("BP:") + String(lastMeasBP, 1) + " ESW:" + String(lastMeasESW, 1);
  }

  auto drawLineIfChanged = [&](int y, const String& now, String& prev) {
  #if DISPLAY_TYPE_OLED
    if (now == prev) return;            // no repaint needed
    display.fillRect(0, y, 128, 10, BLACK);
    display.setCursor(0, y);
    display.setTextColor(WHITE);
    display.print(now);
  #else
    tft.fillRect(0, y, 240, 20, TFT_BLACK);
    tft.setCursor(4, y);
    tft.print(now);
  #endif
    prev = now;
  };

  // Live readings: y=0, y=12
  // Last measurement: y=28, y=40
  drawLineIfChanged(0,  l0, p0);
  drawLineIfChanged(12, l1, p1);
  drawLineIfChanged(28, l2, p2);
  drawLineIfChanged(40, l3, p3);

  // Push the buffer ONCE (OLED)
  #if DISPLAY_TYPE_OLED
    display.display();
  #endif

  lastVal1 = val1;
  lastVal2 = val2;
}

void tare() {
  showStatus("Taring...");
  long rawHead, rawHandle;
  if (nauReadChannel(CH_HEAD, TARE_SAMPLES, rawHead)) {
    tareHead = rawHead;
  } else {
    Serial.println("head (CH2) tare failed");
  }
  if (nauReadChannel(CH_HANDLE, TARE_SAMPLES, rawHandle)) {
    tareHandle = rawHandle;
  } else {
    Serial.println("handle (CH1) tare failed");
  }
  Serial.printf("Tare: head=%ld handle=%ld\n", tareHead, tareHandle);
  showStatus("Tare done");
}

void perform_test() {
  // TODO: implement test routine
}

// Returns true for short press, false if long-hold abort detected
bool waitForButtonPress() {
  // Wait for button to be pressed, showing live scale readings
  unsigned long lastReadingUpdate = 0;
  while (digitalRead(BUTTON) == HIGH) {
    // Update live scale readings every 500ms
    if (millis() - lastReadingUpdate >= 500) {
      lastReadingUpdate = millis();
      long rawHead = 0, rawHandle = 0;
      nauReadChannel(CH_HEAD,   LIVE_SAMPLES, rawHead);
      nauReadChannel(CH_HANDLE, LIVE_SAMPLES, rawHandle);
      // Display on lower lines (prompt text is on lines 0 and 16)
      Display::printLine(36, String("Head: ") + String(rawHead));
      Display::printLine(48, String("Hand: ") + String(rawHandle));
      #if DISPLAY_TYPE_OLED
        display.display();
      #endif
    }
    delay(10);
  }
  // Button is now pressed - track hold duration
  unsigned long pressStart = millis();
  while (digitalRead(BUTTON) == LOW) {
    if (millis() - pressStart >= BUTTON_HOLD_TIME) {
      // Long hold detected - abort
      // Wait for release before returning
      while (digitalRead(BUTTON) == LOW) {
        delay(10);
      }
      return false;
    }
    delay(10);
  }
  delay(50);  // Debounce
  return true;
}

float calculate_BP() {
  float head = lastVal1 / 28.35;
  float handle = lastVal2 / 28.35;
  return (2 * handle + 13 * head) / (head + handle);
}

float estimate_MOI() {
  return (calculate_BP() * 25.4) / (2.08);
}

void calibrate() {
  Serial.println();
  Serial.println("========== ENTERING CALIBRATION MODE ==========");
  Serial.println("Raw nauReadChannel() values echoed below as [CAL] lines");
  calActive = true;
  calibrateImpl();
  calActive = false;
  Serial.println("========== CALIBRATION MODE EXIT ==========");
  Serial.println();
}

void calibrateImpl() {
  tare();

  float weights[4] = {0.0f, 100.0f, 200.0f, 300.0f};
  long readingsHead[4];
  long readingsHandle[4];
  readingsHead[0]   = tareHead;    // zero-load reading from tare()
  readingsHandle[0] = tareHandle;

  // Head channel (CH2 / load cell B)
  for (int i = 1; i < 4; ++i) {
    showStatus(String("Place ") + (int)weights[i] + "g on HEAD", "then press button");
    if (!waitForButtonPress()) {
      Serial.println("Calibration aborted by user");
      showStatus("Calibration", "aborted");
      delay(1000);
      return;
    }
    if (!nauReadChannel(CH_HEAD, MEAS_SAMPLES, readingsHead[i])) {
      showStatus("Cal failed!", "Head read err");
      delay(2000);
      return;
    }
    showStatus(String("Head: ") + String(readingsHead[i]), String((int)weights[i]) + "g captured");
    delay(800);
  }

  // Handle channel (CH1)
  for (int i = 1; i < 4; ++i) {
    showStatus(String("Place ") + (int)weights[i] + "g on HANDLE", "then press button");
    if (!waitForButtonPress()) {
      Serial.println("Calibration aborted by user");
      showStatus("Calibration", "aborted");
      delay(1000);
      return;
    }
    if (!nauReadChannel(CH_HANDLE, MEAS_SAMPLES, readingsHandle[i])) {
      showStatus("Cal failed!", "Handle read err");
      delay(2000);
      return;
    }
    showStatus(String("Handle: ") + String(readingsHandle[i]), String((int)weights[i]) + "g captured");
    delay(800);
  }

  // Least-squares regression for the head channel
  float sumW = 0.0f, sumR = 0.0f;
  for (int i = 0; i < 4; ++i) { sumW += weights[i]; sumR += readingsHead[i]; }
  float meanW = sumW / 4.0f; float meanR = sumR / 4.0f;
  float num = 0.0f, den = 0.0f;
  for (int i = 0; i < 4; ++i) {
    num += (weights[i] - meanW) * (readingsHead[i] - meanR);
    den += (weights[i] - meanW) * (weights[i] - meanW);
  }

  if (den < 0.001f || isnan(den) || isnan(num)) {
    showStatus("Cal failed!", "Head bad data");
    Serial.printf("Head calibration failed: num=%.2f den=%.2f\n", num, den);
    delay(2000);
    return;
  }

  calFactorHead = num / den;
  if (calFactorHead < 0.1f || calFactorHead > 1000000.0f || isnan(calFactorHead)) {
    showStatus("Cal failed!", "Head invalid");
    Serial.printf("Head invalid factor: %.2f\n", calFactorHead);
    delay(2000);
    return;
  }

  tareHead = (long)(meanR - calFactorHead * meanW);  // regression zero-load offset
  Serial.printf("Head: calFactor=%.2f offset=%ld\n", calFactorHead, tareHead);

  // Least-squares regression for the handle channel
  sumW = 0.0f; sumR = 0.0f; num = 0.0f; den = 0.0f;
  for (int i = 0; i < 4; ++i) { sumW += weights[i]; sumR += readingsHandle[i]; }
  meanW = sumW / 4.0f; meanR = sumR / 4.0f;
  for (int i = 0; i < 4; ++i) {
    num += (weights[i] - meanW) * (readingsHandle[i] - meanR);
    den += (weights[i] - meanW) * (weights[i] - meanW);
  }

  if (den < 0.001f || isnan(den) || isnan(num)) {
    showStatus("Cal failed!", "Handle bad data");
    Serial.printf("Handle calibration failed: num=%.2f den=%.2f\n", num, den);
    delay(2000);
    return;
  }

  calFactorHandle = num / den;
  if (calFactorHandle < 0.1f || calFactorHandle > 1000000.0f || isnan(calFactorHandle)) {
    showStatus("Cal failed!", "Handle invalid");
    Serial.printf("Handle invalid factor: %.2f\n", calFactorHandle);
    delay(2000);
    return;
  }

  tareHandle = (long)(meanR - calFactorHandle * meanW);
  Serial.printf("Handle: calFactor=%.2f offset=%ld\n", calFactorHandle, tareHandle);

  saveCalibration();
  showStatus("Calibration", "complete");
}

void saveCalibration() {
  CalData d;
  d.cal1  = calFactorHandle;  // field "1" = handle (CH1)
  d.cal2  = calFactorHead;    // field "2" = head   (CH2)
  d.tare1 = tareHandle;
  d.tare2 = tareHead;
  d.crc   = calCrc32(&d, offsetof(CalData, crc));

  bool ok1 = writeCalToNamespace("dualScale", d);
  bool ok2 = writeCalToNamespace("dualScaleBak", d);

  Serial.printf("[NVS] save: cal1=%.6f cal2=%.6f tare1=%ld tare2=%ld crc=0x%08X primary=%s backup=%s\n",
                d.cal1, d.cal2, d.tare1, d.tare2, d.crc,
                ok1 ? "OK" : "FAIL", ok2 ? "OK" : "FAIL");
}

void loadCalibration() {
  CalData primary, backup;
  bool primaryOk = readCalFromNamespace("dualScale", primary) && calDataValid(primary);
  bool backupOk  = readCalFromNamespace("dualScaleBak", backup) && calDataValid(backup);

  const char* source = nullptr;
  CalData* chosen = nullptr;

  if (primaryOk) {
    chosen = &primary;
    source = "primary";
  } else if (backupOk) {
    chosen = &backup;
    source = "backup";
    // Repair primary from backup
    writeCalToNamespace("dualScale", backup);
    Serial.println("[NVS] Primary corrupt - restored from backup");
  }

  if (chosen) {
    calFactorHandle = chosen->cal1;   // field "1" = handle (CH1)
    calFactorHead   = chosen->cal2;   // field "2" = head   (CH2)
    tareHandle      = chosen->tare1;
    tareHead        = chosen->tare2;
    Serial.printf("[NVS] loaded from %s: calHandle=%.6f calHead=%.6f tareHandle=%ld tareHead=%ld crc=0x%08X\n",
                  source, calFactorHandle, calFactorHead, tareHandle, tareHead, chosen->crc);
  } else {
    Serial.println("[NVS] No valid calibration found in primary or backup - running uncalibrated");
    calFactorHead   = 1.0f;
    calFactorHandle = 1.0f;
  }
}


