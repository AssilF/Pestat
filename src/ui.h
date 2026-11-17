// ui.h -- OLED, RGB LED and button for standalone use. Nothing here touches the DAC or the ADC.
#pragma once
#include <Arduino.h>

void uiBegin();            // after Wire.begin(); safe without a display connected
bool uiDisplayPresent();

// Before any work where the DAC output or the ADC reading matters (calibration, scan):
// LED off, display dark (or frozen, see DISPLAY_DARK_DURING_SCANS). The next uiShow() wakes it.
void uiQuiet();

// Up to five lines; the first in a large font when big is true. nullptr ends the list.
void uiShow(bool big, const char* l0, const char* l1 = nullptr, const char* l2 = nullptr,
            const char* l3 = nullptr, const char* l4 = nullptr);

enum LedColor { LED_OFF, LED_RED, LED_GREEN, LED_YELLOW, LED_BLUE };
void ledSet(LedColor c);
const char* ledName(LedColor c);

enum ButtonEvent { BTN_NONE, BTN_SHORT, BTN_LONG };
ButtonEvent buttonPoll();          // debounced; SHORT on release, LONG once while held
bool buttonHeldFor(uint32_t ms);   // raw: pressed without interruption for at least ms
void buttonClear();                // forget a press in progress (after a blocking operation)
