// ui.cpp -- see ui.h.
#include "ui.h"
#include <Wire.h>
#include <U8g2lib.h>
#include "config.h"

namespace {

#if OLED_SSD1306
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_I2C_SCL, PIN_I2C_SDA);
#else
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_I2C_SCL, PIN_I2C_SDA);
#endif

bool present = false;
bool dark = false;

// button state
bool     rawLast = false, stable = false, longSent = false;
uint32_t rawChangeMs = 0, pressMs = 0, heldSinceMs = 0;
constexpr uint32_t DEBOUNCE_MS = 30, LONG_MS = 1500;

bool rawPressed() { return digitalRead(PIN_BUTTON) == LOW; }

}  // namespace

void uiBegin() {
  // LED first, so that it is off as soon as the pins are outputs (common anode: HIGH = off)
  const uint8_t pins[3] = {PIN_LED_R, PIN_LED_G, PIN_LED_B};
  for (uint8_t p : pins) {
    digitalWrite(p, HIGH);
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.beginTransmission(OLED_I2C_ADDR);
  present = Wire.endTransmission() == 0;
  if (present) {
    oled.setI2CAddress(OLED_I2C_ADDR * 2);
    oled.setBusClock(I2C_CLOCK_HZ);  // same clock as the DAC on the shared bus
    oled.begin();
    oled.setContrast(128);
  }
}

bool uiDisplayPresent() { return present; }

void uiQuiet() {
  ledSet(LED_OFF);
  if (present && DISPLAY_DARK_DURING_SCANS && !dark) {
    oled.setPowerSave(1);
    dark = true;
  }
}

void uiShow(bool big, const char* l0, const char* l1, const char* l2, const char* l3, const char* l4) {
  if (!present) return;
  const char* lines[5] = {l0, l1, l2, l3, l4};
  oled.clearBuffer();
  int y = 0;
  for (int k = 0; k < 5 && lines[k]; k++) {
    if (k == 0 && big) {
      oled.setFont(u8g2_font_helvB12_tr);
      if (oled.getStrWidth(lines[k]) > 128) oled.setFont(u8g2_font_helvB10_tr);
      y += 14;
    } else {
      oled.setFont(u8g2_font_6x10_tf);
      y += (k == 0) ? 8 : 11;
    }
    oled.drawStr(0, y, lines[k]);
    if (k == 0 && big) y += 2;
  }
  oled.sendBuffer();
  if (dark) {
    oled.setPowerSave(0);
    dark = false;
  }
}

void ledSet(LedColor c) {
  bool r = c == LED_RED || c == LED_YELLOW;
  bool g = c == LED_GREEN || c == LED_YELLOW;
  bool b = c == LED_BLUE;
  digitalWrite(PIN_LED_R, r ? LOW : HIGH);
  digitalWrite(PIN_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_LED_B, b ? LOW : HIGH);
}

const char* ledName(LedColor c) {
  switch (c) {
    case LED_RED: return "red";
    case LED_GREEN: return "green";
    case LED_YELLOW: return "yellow";
    case LED_BLUE: return "blue";
    default: return "off";
  }
}

ButtonEvent buttonPoll() {
  uint32_t now = millis();
  bool raw = rawPressed();
  if (raw != rawLast) {
    rawLast = raw;
    rawChangeMs = now;
  }
  if (now - rawChangeMs >= DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable) {
      pressMs = now;
      longSent = false;
    } else if (!longSent) {
      return BTN_SHORT;
    }
  }
  if (stable && !longSent && now - pressMs >= LONG_MS) {
    longSent = true;
    return BTN_LONG;
  }
  return BTN_NONE;
}

bool buttonHeldFor(uint32_t ms) {
  uint32_t now = millis();
  if (!rawPressed()) {
    heldSinceMs = 0;
    return false;
  }
  if (heldSinceMs == 0) heldSinceMs = now ? now : 1;
  return now - heldSinceMs >= ms;
}

void buttonClear() {
  rawLast = stable = rawPressed();
  longSent = true;  // a press in progress produces no event when released
  rawChangeMs = millis();
  heldSinceMs = 0;
}
