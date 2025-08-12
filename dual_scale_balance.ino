/*
  ESP32 Dual Scale – Board Init & Display Bring‑Up
  Step 1 of N: Initialize ESP32 and connect to the display used in Deflection Tester.
*/

/************** CONFIG: Choose your display **************/
#define DISPLAY_TYPE_TFT   0   // 1 = use TFT_eSPI color display (ST7789/ILI9341/etc)
#define DISPLAY_TYPE_OLED  1   // 1 = use SSD1306 128x64 I2C OLED

/************** CONFIG: Pins & options **************/
#define PIN_TFT_MOSI  11
#define PIN_TFT_MISO  13
#define PIN_TFT_SCLK  12
#define PIN_TFT_CS    10
#define PIN_TFT_DC     9
#define PIN_TFT_RST    14

#define I2C_SDA       8    // ESP32-S3: SDA
#define I2C_SCL       9    // ESP32-S3: SCL
#define OLED_ADDR 0x3C

#define PIN_BUTTON     0

#include <Arduino.h>

#if DISPLAY_TYPE_TFT
  #include <SPI.h>
  #include <TFT_eSPI.h>
  TFT_eSPI tft = TFT_eSPI();
#elif DISPLAY_TYPE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #ifndef SCREEN_WIDTH
    #define SCREEN_WIDTH 128
  #endif
  #ifndef SCREEN_HEIGHT
    #define SCREEN_HEIGHT 64
  #endif
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#else
  #error "Select a display type by setting DISPLAY_TYPE_TFT or DISPLAY_TYPE_OLED to 1"
#endif

namespace Display {
  void begin() {
  #if DISPLAY_TYPE_TFT
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Display online", 4, 4, 2);
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
    display.setCursor(0, 0);
    display.println(F("Display online"));
    display.display();
  #endif
  }

  void clear() {
  #if DISPLAY_TYPE_TFT
    tft.fillScreen(TFT_BLACK);
  #else
    display.clearDisplay();
  #endif
  }

  void printLine(int16_t y, const String &text, uint8_t font = 2) {
  #if DISPLAY_TYPE_TFT
    tft.setCursor(4, y);
    tft.setTextFont(1);
    tft.setTextSize(font >= 2 ? 2 : 1);
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

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println(F("ESP32 Dual Scale – Init & Display"));

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  Display::begin();

  Display::clear();
  Display::printLine(0,  F("Dual Scale v0.1"));
  Display::printLine(16, F("Board: ESP32‑S3"));
  Display::printLine(32, F("Display OK"));
  Display::printLine(48, F("Press button to test"));
}

uint32_t lastBlink = 0;
bool blinkState = false;

void loop() {
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    blinkState = !blinkState;
    Serial.printf("Heartbeat %s\n", blinkState ? "•" : "◦");
  }

  bool pressed = (digitalRead(PIN_BUTTON) == LOW);
  static bool wasPressed = false;

  if (pressed && !wasPressed) {
    Display::clear();
    Display::printLine(0,  F("Button pressed"));
    Display::printLine(16, String("millis: ") + millis());
  }
  wasPressed = pressed;
}
