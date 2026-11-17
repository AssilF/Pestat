// config.h -- the locked measurement configuration and the hardware constants.
//
// Part 1 is the measurement configuration of the data set (data/potentiostat/PROTOCOL.md,
// rule 1: chosen once, never changed afterwards). The firmware prints every value of
// part 1, with a CRC-32 of them, in the header of every scan file, so that a file
// recorded in another configuration can be recognized.
// If a value of part 1 must change, change CONFIG_NAME as well and do not mix the scans
// of the two configurations in one data set.
//
// Part 2 describes the circuit (Anshori et al. 2022, Fig. 2, with an MCP4725 as the DAC).
// Part 3 sets up the display, button and LED for standalone use.
#pragma once
#include <stdint.h>

// =====================================================================================
// 1. LOCKED MEASUREMENT CONFIGURATION
// =====================================================================================

#define CONFIG_NAME "PB-CV-1"

// Potential program, volts of the working electrode against the reference electrode.
// Each cycle goes start -> vertex -> start. Vertex above start = anodic (oxidation) first.
constexpr float    CFG_E_START_V  = 0.000f;
constexpr float    CFG_E_VERTEX_V = 1.000f;
constexpr int      CFG_CYCLES     = 1;

// Staircase: the potential moves by a whole number of DAC codes, then holds.
// Scan rate = step height / step time; the header gives it from the calibrated slope.
constexpr int      CFG_STEP_CODES   = 7;     // DAC codes per step (7 x 0.732 mV = 5.13 mV, nominal)
constexpr uint32_t CFG_STEP_TIME_MS = 100;   // time spent on each step

// Hold at the start potential before the first step (electrode immersed, not stirred).
constexpr uint32_t CFG_QUIET_TIME_MS = 5000;

// Current sampling within each step: CFG_ADC_SAMPLES readings, evenly spaced, in a
// window that ends with the step. 80 x 250 us = 20 ms = one period of the 50 Hz mains,
// so mains interference averages out. The rest of the step (80 ms) is settling time.
constexpr int      CFG_ADC_SAMPLES        = 80;
constexpr uint32_t CFG_SAMPLE_INTERVAL_US = 250;

// Dummy-cell program (phase 0 of PROTOCOL.md, resistor between WE and the joined RE+CE).
// Same step, step time, quiet time and sampling as above; window symmetric about 0 V so
// that both signs of current are checked: 0 -> +1 -> -1 -> 0 V.
constexpr float    DUMMY_E_HIGH_V = 1.000f;
constexpr float    DUMMY_E_LOW_V  = -1.000f;

// =====================================================================================
// 2. HARDWARE
// =====================================================================================

// Wiring (NodeMCU-32S), kept from the earlier firmware of this instrument.
constexpr uint8_t  PIN_I2C_SDA  = 21;
constexpr uint8_t  PIN_I2C_SCL  = 22;
constexpr uint8_t  PIN_ADC      = 32;    // ADC1 channel 4 <- node "vo" (output of buffer U4)
constexpr uint8_t  MCP4725_ADDR = 0x60;  // A0 to GND
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// MCP4725 supply voltage. Its output is VDD * code / 4096, so this sets the full scale.
// The user confirmed 3.3 V (ESP32 rail) on 2026-11-17.
constexpr float DAC_VDD_V = 3.3f;

// Summing amplifier U1 (eq. 2.1-2.2 of the thesis): E = (R4/R2) V_DAC + (R4/R3) V_minus
//   = 0.909 V_DAC - 1.5 V with V_minus = -5 V. Nominal values only: the multimeter
//   calibration replaces them.
constexpr float R2_OHM    = 33000.0f;
constexpr float R3_OHM    = 100000.0f;
constexpr float R4_OHM    = 30000.0f;
constexpr float V_MINUS_V = -5.0f;

// The rails come from batteries (user, 2026-11-17), so V_minus is not exactly -5 V and
// sags during a session. It sets both the offset b of the potential (b = 0.3 V_minus) and
// the output at zero current (1.70 V at -5 V). The calibrations accept a rail within this
// range and print the rail voltage they imply.
constexpr float V_MINUS_MIN_V = -5.5f;
constexpr float V_MINUS_MAX_V = -3.0f;

// Transimpedance amplifier U3 (eq. 2.3): V_out = (R6/R5)(-V_minus) + R6 I = 1.70 V + R6 I,
// I positive for an oxidation at the working electrode. R6 = 5.1 kohm on the board, output
// wired straight to the ADC (user, 2026-11-17).
// R6 is used for the current; the offset is measured (command "zero").
constexpr float R5_OHM = 15000.0f;
constexpr float R6_OHM = 5100.0f;

// V_out reaches the ADC pin directly (the D2-D1-R7 branch only clamps). If a divider is
// ever added in front of the pin, set its ratio here (V_out / V_pin).
constexpr float VOUT_PER_VPIN = 1.0f;

// Never drive the cell beyond this potential (window of the published design).
constexpr float E_LIMIT_V = 1.5f;

// =====================================================================================
// 3. STANDALONE USE: OLED, BUTTON, RGB LED (not part of the measurement configuration)
// =====================================================================================

// 1.3" 128x64 OLED on the same I2C bus as the DAC (0x60), powered from the 3.3 V rail.
// Most 1.3" modules use the SH1106 controller; set to 1 for a module with an SSD1306
// (symptom of the wrong choice: the image is shifted by two columns, or garbled).
#define OLED_SSD1306 0
constexpr uint8_t OLED_I2C_ADDR = 0x3C;

// Push button between this pin and GND (internal pull-up).
constexpr uint8_t PIN_BUTTON = 4;

// RGB LED with a common anode on the 3.3 V rail (not 5 V); each cathode through its
// resistor to a pin. LOW lights the color.
constexpr uint8_t PIN_LED_R = 25;
constexpr uint8_t PIN_LED_G = 26;
constexpr uint8_t PIN_LED_B = 27;

// The MCP4725 takes its reference from the 3.3 V rail, which also feeds the OLED (whose
// current depends on the pixels lit) and the LED. During every calibration and every scan
// the LED is off and the display dark, so the rail carries the same load in both.
// false = keep a frozen "scanning" screen instead: allowed only if "zero" shows the same
// noise with the display on and off.
constexpr bool DISPLAY_DARK_DURING_SCANS = true;

// Scans per sample in standalone use. PROTOCOL.md phase B scans each preparation twice on a
// renewed surface, and analysis/classify.py judges a preparation from all its scans.
constexpr int SAMPLE_SCANS = 2;
