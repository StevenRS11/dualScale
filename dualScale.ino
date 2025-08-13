/*
  ESP32 Dual Scale – Load Cell Display Test
  Reads two HX711 load cells and displays both values every 5 seconds or on button press.
*/

#define DISPLAY_TYPE_TFT   0
#define DISPLAY_TYPE_OLED  1

#define I2C_SDA       8
#define I2C_SCL       9
#define OLED_ADDR 0x3C

#define PIN_BUTTON     0
#define LOADCELL_DOUT1 4   // Adjust pins for your wiring
#define LOADCELL_SCK1  5
#define LOADCELL_DOUT2 6
#define LOADCELL_SCK2  7

#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>

#include "Rfid2.h"

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

const unsigned long debounceDelay = 25;
bool buttonState = HIGH;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 5000;

float lastVal1 = 0.0f;
float lastVal2 = 0.0f;

bool nextActionIsWrite = true;

long readStable(HX711 &scale);
void updateReadings();
void saveCalibration();
void loadCalibration();
void handleButtonPress();

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
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  Display::begin();
  if (!rfid2Begin()) {
    Serial.println("RFID2 init failed");
  }

  scale1.begin(LOADCELL_DOUT1, LOADCELL_SCK1);
  scale2.begin(LOADCELL_DOUT2, LOADCELL_SCK2);

  loadCalibration();
  scale1.tare();
  scale2.tare();
  saveCalibration();

  updateReadings();
}

void loop() {
  int reading = digitalRead(PIN_BUTTON);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        handleButtonPress();
      }
    }
  }
  lastButtonState = reading;

  if (millis() - lastUpdate >= updateInterval) {
    updateReadings();
  }
}

long readStable(HX711 &scale) {
  long samples[10];
  for (int i = 0; i < 10; ++i) {
    samples[i] = scale.read();
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
  float val1 = (raw1 - scale1.get_offset()) / scale1.get_scale();
  float val2 = (raw2 - scale2.get_offset()) / scale2.get_scale();
  float diff = val1 - val2;

  Serial.printf("S1: %.2f\tS2: %.2f\tDiff: %.2f\n", val1, val2, diff);
  Display::clear();
  Display::printLine(0, String("Scale1: ") + val1);
  Display::printLine(16, String("Scale2: ") + val2);
  Display::printLine(32, String("Diff: ") + diff);

  lastVal1 = val1;
  lastVal2 = val2;
  lastUpdate = millis();
}

void handleButtonPress() {
  if (nextActionIsWrite) {
    long diff = (long)(lastVal1 - lastVal2);
    String diffStr = String(diff);
    Serial.printf("Writing diff=%ld\n", diff);
    showStatus("NFC: Tap tag to write...");
    String err;
    bool ok = rfid2WriteText(String("DS:") + diffStr, &err);
    if (ok) {
      showStatus(String("Write OK: diff=") + diffStr);
      Serial.printf("Write OK: diff=%ld\n", diff);
      nextActionIsWrite = false;
    } else {
      showStatus(String("Write FAIL: ") + err);
      Serial.println("Write FAIL: " + err);
      if (err != "timeout")
        nextActionIsWrite = false;
    }
  } else {
    showStatus("NFC: Tap tag to read...");
    String text;
    String err;
    bool ok = rfid2ReadText(&text, &err);
    if (ok && text.startsWith("DS:")) {
      String diffStr = text.substring(3);
      showStatus(String("Read: diff=") + diffStr);
      Serial.println("Read: diff=" + diffStr);
      nextActionIsWrite = true;
    } else {
      if (!ok) {
        if (err == "timeout") {
          showStatus("Read FAIL: timeout");
          Serial.println("Read FAIL: timeout");
        } else {
          showStatus(String("Read FAIL: ") + err);
          Serial.println("Read FAIL: " + err);
          nextActionIsWrite = true;
        }
      } else {
        showStatus("Read FAIL: no DS record");
        Serial.println("Read FAIL: no DS record");
        nextActionIsWrite = true;
      }
    }
  }
}

void loadCalibration() {
  prefs.begin("dualScale", false);
  calFactor1 = prefs.getFloat("cal1", 1.0f);
  calFactor2 = prefs.getFloat("cal2", 1.0f);
  long offset1 = prefs.getLong("tare1", 0);
  long offset2 = prefs.getLong("tare2", 0);
  prefs.end();

  scale1.set_scale(calFactor1);
  scale2.set_scale(calFactor2);
  scale1.set_offset(offset1);
  scale2.set_offset(offset2);
}

void saveCalibration() {
  prefs.begin("dualScale", false);
  prefs.putFloat("cal1", calFactor1);
  prefs.putFloat("cal2", calFactor2);
  prefs.putLong("tare1", scale1.get_offset());
  prefs.putLong("tare2", scale2.get_offset());
  prefs.end();
}
