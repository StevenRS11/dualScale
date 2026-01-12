/*
  ESP32 Dual Scale – Load Cell Display Test (Auto NFC Write)
  Reads two HX711 load cells and displays both values every 2 seconds.
  NFC tags are checked automatically every 2 seconds; if a tag is PRESENT,
  the device writes once (on presence edge) and does not read from NFC at all.
*/

#define DISPLAY_TYPE_TFT   0
#define DISPLAY_TYPE_OLED  1

// I2C pins (from schematic)
#define SCREEN_SDA    8   // GP6
#define SCREEN_SCL    7   // GP7

// RFID I2C pins (separate bus)
#define RFID_SDA      10  // GP10
#define RFID_SCL      11  // GP11


#define OLED_ADDR 0x3C

#define LOADCELL_DOUT2 6   // Adjust pins for your wiring
#define LOADCELL_SCK2  5
#define LOADCELL_DOUT1 3
#define LOADCELL_SCK1  4

#define BUTTON  9


#include <Arduino.h>
#include <HX711.h>
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
  // Wire is used for the screen, Wire1 for RFID (both pre-defined by ESP32)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

HX711 scale1;
HX711 scale2;
Preferences prefs;

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

float calFactor1 = 1.0f;
float calFactor2 = 1.0f;

// Display update timing
const unsigned long updateInterval = 2000;   // display + reading cadence
unsigned long lastUpdate = 0;

float lastVal1 = 0.0f;
float lastVal2 = 0.0f;

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
 
long readStable(HX711 &scale);
void updateReadings();
void saveCalibration();
void loadCalibration();
void tare();
void calibrate();
void perform_test();
void waitForButtonPress();
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

void showStatus(const String &line1, const String &line2 = String()) {
  Display::clear();
  Display::printLine(0, line1);
  if (line2.length()) {
    Display::printLine(16, line2);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100); // Let serial stabilize
  Serial.println("Init start");

  // Configure button pin with internal pull-up
  pinMode(BUTTON, INPUT_PULLUP);

  // Check if button is held down during boot -> enter calibration mode
  bool calibrationMode = (digitalRead(BUTTON) == LOW);
  if (calibrationMode) {
    Serial.println("Button held during boot - will enter calibration mode after init");
  }

  // Check for repeated boot loops (crash detection)
  if (!prefs.begin("dualScale", false)) {
    Serial.println("[NVS] begin() failed in setup");
  } else {
    uint32_t bootCount = prefs.getUInt("bootCount", 0);
    bootCount++;
    prefs.putUInt("bootCount", bootCount);
    Serial.printf("Boot count: %u\n", bootCount);

    // If we've rebooted 3+ times rapidly, clear calibration
    if (bootCount >= 3) {
      Serial.println("Multiple rapid boots detected - clearing calibration!");
      prefs.remove("cal1");
      prefs.remove("cal2");
      prefs.remove("tare1");
      prefs.remove("tare2");
      prefs.putUInt("bootCount", 0);
    }
    prefs.end();
  }

  Serial.println("Display Init start");
  Display::begin();
  Serial.println("Display Init finish");

  // Initialize Payload library
  Serial.println("Payload Init Start");
  Wire1.begin(RFID_SDA, RFID_SCL);
  Wire1.setClock(400000);
  delay(100);  // Critical: stabilize I2C before NFC init

  nfc.setDebug(true);  // Enable NFC debug output
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

  // Load cells
  Serial.println("Load cells init");
  scale1.begin(LOADCELL_DOUT1, LOADCELL_SCK1);
  scale2.begin(LOADCELL_DOUT2, LOADCELL_SCK2);

  tare();
  loadCalibration();

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
          nfc.halt();  // Release tag if one is selected
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
  // Existing live display behavior
  if (millis() - lastUpdate >= updateInterval) {
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
    long raw1 = readStable(scale1);
    long raw2 = readStable(scale2);

    if (raw1 == 0 || raw2 == 0) {
      showStatus("Measurement", "failed");
      delay(2000);
      currentState = IDLE;
      lastUpdate = 0;
      return;
    }

    // Capture measurements
    measuredHeadWeight = (raw1 - scale1.get_offset()) / calFactor1;
    measuredHandleWeight = (raw2 - scale2.get_offset()) / calFactor2;

    // Calculate display values
    lastVal1 = measuredHeadWeight;
    lastVal2 = measuredHandleWeight;
    calculatedBalancePoint = calculate_BP();
    calculatedSwingWeight = estimate_MOI();
    calculatedMass = (measuredHeadWeight + measuredHandleWeight) / 28.35;

    Serial.printf("Measurements: Head=%.2fg Handle=%.2fg\n",
                  measuredHeadWeight, measuredHandleWeight);

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
    Display::clear();
    Display::printLine(24, "Success!");
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

long readStable(HX711 &scale) {
  // Wait for the HX711 to become ready; bail out if it never does
  if (!scale.wait_ready_timeout(1000)) {
    Serial.println("HX711 not ready");
    return 0;
  }

  long samples[10];
  for (int i = 0; i < 10; ++i) {
    samples[i] = scale.read();
    delay(1);  // yield to avoid watchdog reset
  }
  long minVal = samples[0];
  long maxVal = samples[0];
  long sum = 0;
  for (int i = 0; i < 10; ++i) {
    if (samples[i] < minVal) minVal = samples[i];
    if (samples[i] > maxVal) maxVal = samples[i];
    sum += samples[i];
  }
  sum -= minVal + maxVal;
  return sum / 8;
}

void updateReadings() {
  long raw1 = readStable(scale1);
  long raw2 = readStable(scale2);
  #if DISPLAY_TYPE_OLED
  if (raw1 == 0 && raw2 == 0) {
    showStatus("Load cells", "not ready");
    return;
  } else if (raw1 == 0) {
    showStatus("Scale 1 (head)", "not ready");
    return;
  } else if (raw2 == 0) {
    showStatus("Scale 2 (handle)", "not ready");
    return;
  }
  #endif
  // Validate calibration factors to prevent NaN
  if (calFactor1 < 0.1f || calFactor1 > 1000000.0f || isnan(calFactor1) || isinf(calFactor1)) {
    showStatus("Invalid cal", "Scale 1 - recal");
    Serial.printf("Invalid calFactor1: %.2f\n", calFactor1);
    return;
  }
  if (calFactor2 < 0.1f || calFactor2 > 1000000.0f || isnan(calFactor2) || isinf(calFactor2)) {
    showStatus("Invalid cal", "Scale 2 - recal");
    Serial.printf("Invalid calFactor2: %.2f\n", calFactor2);
    return;
  }

  float val1 = (raw1 - scale1.get_offset()) / calFactor1;
  float val2 = (raw2 - scale2.get_offset()) / calFactor2;

  // Check for invalid results
  if (isnan(val1) || isinf(val1)) {
    showStatus("Scale 1 error", "NaN/Inf result");
    Serial.printf("Scale 1 invalid result: %.2f\n", val1);
    return;
  }
  if (isnan(val2) || isinf(val2)) {
    showStatus("Scale 2 error", "NaN/Inf result");
    Serial.printf("Scale 2 invalid result: %.2f\n", val2);
    return;
  }

  // Precompute strings (fixed positions):
  String l0 = String("Static Weight: ") + String((val1 + val2) / 28.35, 2);
  String l1 = String("BP: ") + String(calculate_BP());
  String l2 = String("ESW: ") + String(estimate_MOI());
  String l3 = String("Handle Mass: ") + String(val2, 2);
  String l4 = String("Head Mass: ") + String(val1, 2);

  // Static cache of last-drawn text to decide what to repaint
  static String p0, p1, p2, p3, p4;

  auto drawLineIfChanged = [&](int y, const String& now, String& prev) {
  #if DISPLAY_TYPE_OLED
    if (now == prev) return;            // no repaint needed
    // Erase only this row area (full width)
    display.fillRect(0, y, 128, 10, BLACK);
    display.setCursor(0, y);
    display.setTextColor(WHITE);
    display.print(now);
  #else
    // For TFT, just reprint (cheap enough at 2s cadence)
    tft.fillRect(0, y, 240, 20, TFT_BLACK);
    tft.setCursor(4, y);
    tft.print(now);
  #endif
    prev = now;
  };

  drawLineIfChanged(0,  l0, p0);
  drawLineIfChanged(16, l1, p1);
  drawLineIfChanged(32, l2, p2);
  drawLineIfChanged(48, l3, p3);
  drawLineIfChanged(56, l4, p4);

  // Push the buffer ONCE (OLED)
  #if DISPLAY_TYPE_OLED
    display.display();
  #endif

  lastVal1 = val1;
  lastVal2 = val2;
  lastUpdate = millis();
}

void tare() {
  showStatus("Taring...");
  if (scale1.wait_ready_timeout(1000)) {
    scale1.tare();
  } else {
    Serial.println("scale1 tare timeout");
  }
  if (scale2.wait_ready_timeout(1000)) {
    scale2.tare();
  } else {
    Serial.println("scale2 tare timeout");
  }
  showStatus("Tare done");
}

void perform_test() {
  // TODO: implement test routine
}

void waitForButtonPress() {
  // Wait for button to be pressed
  while (digitalRead(BUTTON) == HIGH) {
    delay(10);
  }
  // Wait for button to be released
  while (digitalRead(BUTTON) == LOW) {
    delay(10);
  }
  delay(50);  // Debounce
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
  tare();

  float weights[4] = {0.0f, 100.0f, 200.0f, 300.0f};
  long readings1[4];
  long readings2[4];
  readings1[0] = scale1.get_offset();
  readings2[0] = scale2.get_offset();

  for (int i = 1; i < 4; ++i) {
    showStatus(String("Place ") + (int)weights[i] + "g on scale 1", "then press button");
    waitForButtonPress();
    readings1[i] = readStable(scale1);
  }

  for (int i = 1; i < 4; ++i) {
    showStatus(String("Place ") + (int)weights[i] + "g on scale 2", "then press button");
    waitForButtonPress();
    readings2[i] = readStable(scale2);
  }

  // Calculate calibration factor for scale 1
  float sumW = 0.0f, sumR = 0.0f;
  for (int i = 0; i < 4; ++i) { sumW += weights[i]; sumR += readings1[i]; }
  float meanW = sumW / 4.0f; float meanR = sumR / 4.0f;
  float num = 0.0f, den = 0.0f;
  for (int i = 0; i < 4; ++i) {
    num += (weights[i] - meanW) * (readings1[i] - meanR);
    den += (weights[i] - meanW) * (weights[i] - meanW);
  }

  if (den < 0.001f || isnan(den) || isnan(num)) {
    showStatus("Cal failed!", "Scale 1 bad data");
    Serial.printf("Scale 1 calibration failed: num=%.2f den=%.2f\n", num, den);
    delay(2000);
    return;
  }

  calFactor1 = num / den;
  if (calFactor1 < 0.1f || calFactor1 > 1000000.0f || isnan(calFactor1)) {
    showStatus("Cal failed!", "Scale 1 invalid");
    Serial.printf("Scale 1 invalid factor: %.2f\n", calFactor1);
    delay(2000);
    return;
  }

  long offset1 = (long)(meanR - calFactor1 * meanW);
  scale1.set_scale(calFactor1);
  scale1.set_offset(offset1);
  Serial.printf("Scale 1: calFactor=%.2f offset=%ld\n", calFactor1, offset1);

  // Calculate calibration factor for scale 2
  sumW = 0.0f; sumR = 0.0f; num = 0.0f; den = 0.0f;
  for (int i = 0; i < 4; ++i) { sumW += weights[i]; sumR += readings2[i]; }
  meanW = sumW / 4.0f; meanR = sumR / 4.0f;
  for (int i = 0; i < 4; ++i) {
    num += (weights[i] - meanW) * (readings2[i] - meanR);
    den += (weights[i] - meanW) * (weights[i] - meanW);
  }

  if (den < 0.001f || isnan(den) || isnan(num)) {
    showStatus("Cal failed!", "Scale 2 bad data");
    Serial.printf("Scale 2 calibration failed: num=%.2f den=%.2f\n", num, den);
    delay(2000);
    return;
  }

  calFactor2 = num / den;
  if (calFactor2 < 0.1f || calFactor2 > 1000000.0f || isnan(calFactor2)) {
    showStatus("Cal failed!", "Scale 2 invalid");
    Serial.printf("Scale 2 invalid factor: %.2f\n", calFactor2);
    delay(2000);
    return;
  }

  long offset2 = (long)(meanR - calFactor2 * meanW);
  scale2.set_scale(calFactor2);
  scale2.set_offset(offset2);
  Serial.printf("Scale 2: calFactor=%.2f offset=%ld\n", calFactor2, offset2);

  saveCalibration();
  showStatus("Calibration", "complete");
}

void saveCalibration() {
  if (!prefs.begin("dualScale", false)) {
    Serial.println("[NVS] begin(rw) failed");
    return;
  }
  size_t w1 = prefs.putFloat("cal1", calFactor1);
  size_t w2 = prefs.putFloat("cal2", calFactor2);
  size_t w3 = prefs.putLong("tare1", scale1.get_offset());
  size_t w4 = prefs.putLong("tare2", scale2.get_offset());

  float  rc1 = prefs.getFloat("cal1", NAN);
  float  rc2 = prefs.getFloat("cal2", NAN);
  long   ro1 = prefs.getLong("tare1", LONG_MIN);
  long   ro2 = prefs.getLong("tare2", LONG_MIN);

  prefs.end();

  Serial.printf("[NVS] saved bytes: cal1=%u cal2=%u tare1=%u tare2=%u\n",
                (unsigned)w1,(unsigned)w2,(unsigned)w3,(unsigned)w4);
  Serial.printf("[NVS] saved values: cal1=%.6f cal2=%.6f tare1=%ld tare2=%ld\n",
                rc1, rc2, ro1, ro2);
}

void loadCalibration() {
  if (!prefs.begin("dualScale", true)) {
    Serial.println("[NVS] begin(ro) failed");
    return;
  }

  calFactor1 = prefs.getFloat("cal1", 1.0f);
  calFactor2 = prefs.getFloat("cal2", 1.0f);
  long offset1 = prefs.getLong("tare1", 0);
  long offset2 = prefs.getLong("tare2", 0);

  prefs.end();

  // Validate loaded calibration factors
  if (isnan(calFactor1) || isinf(calFactor1) || calFactor1 < 0.1f || calFactor1 > 1000000.0f) {
    Serial.printf("[NVS] Invalid cal1=%.6f, resetting to 1.0\n", calFactor1);
    calFactor1 = 1.0f;
    offset1 = 0;
  }
  if (isnan(calFactor2) || isinf(calFactor2) || calFactor2 < 0.1f || calFactor2 > 1000000.0f) {
    Serial.printf("[NVS] Invalid cal2=%.6f, resetting to 1.0\n", calFactor2);
    calFactor2 = 1.0f;
    offset2 = 0;
  }

  scale1.set_scale(calFactor1);
  scale2.set_scale(calFactor2);
  scale1.set_offset(offset1);
  scale2.set_offset(offset2);

  Serial.printf("[NVS] loaded values: cal1=%.6f cal2=%.6f tare1=%ld tare2=%ld\n",
                calFactor1, calFactor2, offset1, offset2);
}


