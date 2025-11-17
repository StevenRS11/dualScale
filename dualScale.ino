/*
  ESP32 Dual Scale – Load Cell Display Test (Auto NFC Write)
  Reads two HX711 load cells and displays both values every 2 seconds.
  NFC tags are checked automatically every 2 seconds; if a tag is PRESENT,
  the device writes once (on presence edge) and does not read from NFC at all.
*/

#define DISPLAY_TYPE_TFT   0
#define DISPLAY_TYPE_OLED  1

#define I2C_SDA       8
#define I2C_SCL       9


#define OLED_ADDR 0x3C

#define LOADCELL_DOUT2 5   // Adjust pins for your wiring
#define LOADCELL_SCK2  4
#define LOADCELL_DOUT1 41
#define LOADCELL_SCK1  42

#define BUTTON  6


#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>

#include "Rfid2.h"  // assumes: bool rfid2IsTagPresent(); bool rfid2WriteText(const String&, String* err);

#if DISPLAY_TYPE_TFT
  #include <SPI.h>
  #include <TFT_eSPI.h>
  TFT_eSPI tft = TFT_eSPI();
#elif DISPLAY_TYPE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 64
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

HX711 scale1;
HX711 scale2;
Preferences prefs;

float calFactor1 = 1.0f;
float calFactor2 = 1.0f;

// NFC auto-polling state
const unsigned long updateInterval = 2000;   // display + reading cadence
const unsigned long nfcPollInterval = 2000;  // NFC presence check cadence
unsigned long lastUpdate = 0;
unsigned long lastNfcPoll = 0;
bool nfcWasPresent = false;                  // edge-detect so we only write once per tap

float lastVal1 = 0.0f;
float lastVal2 = 0.0f;
 
long readStable(HX711 &scale);
void updateReadings();
void saveCalibration();
void loadCalibration();
void tare();
void calibrate();
void perform_test();
void pollNfcAndWrite();

namespace Display {
  void begin() {
  #if DISPLAY_TYPE_TFT
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
  #elif DISPLAY_TYPE_OLED
    Wire.begin(I2C_SDA, I2C_SCL);
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
  Serial.println("Init start");
  Serial.println("Display Init start");
  Display::begin();
  Serial.println("Display Init finish");

  Wire.setClock(400000);

  // NFC init
  Serial.println("RFID Init Start");

  if (!rfid2Begin()) {
    Serial.println("RFID2 init failed");
  }
  Serial.println("RFID Init Finish");


  // Load cells
  scale1.begin(LOADCELL_DOUT1, LOADCELL_SCK1);
  scale2.begin(LOADCELL_DOUT2, LOADCELL_SCK2);

  tare();
  loadCalibration();
}

void loop() {
  // Periodic readings + display
  if (millis() - lastUpdate >= updateInterval) {
     updateReadings();
  }

  // Periodic NFC polling (auto-write on presence edge)
  if (millis() - lastNfcPoll >= nfcPollInterval) {
     pollNfcAndWrite();
     lastNfcPoll = millis();
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
  float val1 = (raw1 - scale1.get_offset()) / calFactor1;
  float val2 = (raw2 - scale2.get_offset()) / calFactor2;

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

// NOTE: write-once per presence edge
void pollNfcAndWrite() {
  bool present = false;
#ifdef ARDUINO
  // If your Rfid2.h exposes a presence checker, use it:
  present = waitForCard(100);
#endif

  // On rising edge of presence -> read command or write diff
  if (present && !nfcWasPresent) {
    String text;
    String err;
    bool handled = false;
    if (rfid2ReadText(&text, &err, false)) {


      text.trim();
      text.toUpperCase();
      if (text == "CAL") {
        showStatus("NFC cmd: CAL");
        calibrate();
        handled = true;
      } else if (text == "TARE") {
        showStatus("NFC cmd: TARE");
        tare();
        handled = true;
      }
    }

    if (!handled) {
      long diff = (long)(lastVal1 - lastVal2);
      String diffStr = String(diff);
      bool ok = rfid2WriteText(String("DS:") + diffStr, &err);
      if (ok) {
        showStatus("NFC write OK", String("diff=") + diffStr);
        Serial.printf("NFC write OK: diff=%ld\n", diff);
      } else {
        showStatus("NFC write FAIL", err);
        Serial.println("NFC write FAIL: " + err);
      }
    } else {
      rfid2Halt();

    }
  }

  nfcWasPresent = present;
}

void perform_test() {
  // TODO: implement test routine
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
    showStatus(String("Place ") + (int)weights[i] + "g on scale 1", "then present card");
    while (!waitForCard()) {
      // keep waiting for any card
    }
    rfid2Halt();
    readings1[i] = readStable(scale1);
  }

  for (int i = 1; i < 4; ++i) {
    showStatus(String("Place ") + (int)weights[i] + "g on scale 2", "then present card");
    while (!waitForCard()) {
      // keep waiting for any card
    }
    rfid2Halt();
    readings2[i] = readStable(scale2);
  }

  float sumW = 0.0f, sumR = 0.0f;
  for (int i = 0; i < 4; ++i) { sumW += weights[i]; sumR += readings1[i]; }
  float meanW = sumW / 4.0f; float meanR = sumR / 4.0f;
  float num = 0.0f, den = 0.0f;
  for (int i = 0; i < 4; ++i) { num += (weights[i] - meanW) * (readings1[i] - meanR); den += (weights[i] - meanW) * (weights[i] - meanW); }
  calFactor1 = num / den; long offset1 = (long)(meanR - calFactor1 * meanW);
  scale1.set_scale(calFactor1); scale1.set_offset(offset1);

  sumW = 0.0f; sumR = 0.0f; num = 0.0f; den = 0.0f;
  for (int i = 0; i < 4; ++i) { sumW += weights[i]; sumR += readings2[i]; }
  meanW = sumW / 4.0f; meanR = sumR / 4.0f;
  for (int i = 0; i < 4; ++i) { num += (weights[i] - meanW) * (readings2[i] - meanR); den += (weights[i] - meanW) * (weights[i] - meanW); }
  calFactor2 = num / den; long offset2 = (long)(meanR - calFactor2 * meanW);
  scale2.set_scale(calFactor2); scale2.set_offset(offset2);

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

  scale1.set_scale(calFactor1);
  scale2.set_scale(calFactor2);
  scale1.set_offset(offset1);
  scale2.set_offset(offset2);

  Serial.printf("[NVS] loaded values: cal1=%.6f cal2=%.6f tare1=%ld tare2=%ld\n",
                calFactor1, calFactor2, offset1, offset2);
}


