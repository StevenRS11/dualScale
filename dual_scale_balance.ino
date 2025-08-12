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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  Display::begin();

  scale1.begin(LOADCELL_DOUT1, LOADCELL_SCK1);
  scale2.begin(LOADCELL_DOUT2, LOADCELL_SCK2);

  Display::clear();
  Display::printLine(0, "Dual Scale Test");
}

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 5000;

void loop() {
  bool pressed = (digitalRead(PIN_BUTTON) == LOW);
  if (pressed || millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();
    long reading1 = scale1.get_units();
    long reading2 = scale2.get_units();

    Serial.printf("Scale1: %ld\tScale2: %ld\n", reading1, reading2);
    Display::clear();
    Display::printLine(0, String("Scale1: ") + reading1);
    Display::printLine(16, String("Scale2: ") + reading2);
  }
}
