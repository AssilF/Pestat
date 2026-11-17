// Firmware of the portable potentiostat -- ESP32 (NodeMCU-32S), MCP4725 DAC, analog front end
// of Anshori et al. 2022. Staircase cyclic voltammetry in one locked configuration (config.h),
// multimeter calibration of the potential and of the ADC, output over serial as the
// E_V,I_A files of data/potentiostat/PROTOCOL.md. No Wi-Fi, no Bluetooth.
// Standalone use: OLED, button and RGB LED (ui.cpp); the decision rule of classify.py runs on
// the device when a model has been exported (rule.cpp, model.h).
// Commands: type "help" in a serial monitor at 115200 baud. See README.md.

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_adc_cal.h>
#include <math.h>
#include "config.h"
#include "rule.h"
#include "ui.h"

static const char* FW_NAME    = "pbscreen-potentiostat";
static const char* FW_VERSION = "1.1.0";

// ---------------------------------------------------------------- derived constants
constexpr int      DAC_MAX_CODE  = 4095;
constexpr float    A_NOM         = (R4_OHM / R2_OHM) * DAC_VDD_V / 4096.0f;  // V of E per DAC code
constexpr float    B_NOM         = (R4_OHM / R3_OHM) * V_MINUS_V;            // E at code 0
constexpr float    VOUT0_NOM     = -V_MINUS_V * R6_OHM / R5_OHM;             // V_out at I = 0 (1.70 V)
constexpr uint32_t STEP_US       = CFG_STEP_TIME_MS * 1000UL;
constexpr uint32_t WINDOW_US     = (uint32_t)CFG_ADC_SAMPLES * CFG_SAMPLE_INTERVAL_US;
constexpr uint32_t SETTLE_MIN_US = 15000;  // settling delay of the published design (53 Hz filter)
constexpr int      MAX_CYCLES    = 10;
constexpr int      MAX_VERTICES  = 2 * MAX_CYCLES + 2;

static_assert(CFG_STEP_CODES >= 1, "a step is at least one DAC code");
static_assert(CFG_CYCLES >= 1 && CFG_CYCLES <= MAX_CYCLES, "1 to 10 cycles");
static_assert(CFG_ADC_SAMPLES >= 1 && CFG_ADC_SAMPLES <= 1000, "1 to 1000 readings per point");
static_assert(WINDOW_US + SETTLE_MIN_US <= STEP_US, "the sampling window must leave >= 15 ms of settling");
static_assert(CFG_E_START_V >= -E_LIMIT_V && CFG_E_START_V <= E_LIMIT_V, "start beyond the limit");
static_assert(CFG_E_VERTEX_V >= -E_LIMIT_V && CFG_E_VERTEX_V <= E_LIMIT_V, "vertex beyond the limit");
static_assert(CFG_E_VERTEX_V != CFG_E_START_V, "vertex equals start");
static_assert(DUMMY_E_LOW_V >= -E_LIMIT_V && DUMMY_E_HIGH_V <= E_LIMIT_V, "dummy window beyond the limit");

// point flags (bit mask), written in the "flag" column
constexpr uint8_t FLAG_BLIND         = 1;  // a reading of 0 counts: input in the ADC's blind zone
constexpr uint8_t FLAG_SATURATED     = 2;  // a reading of 4095 counts: input above the ADC range
constexpr uint8_t FLAG_OUTSIDE_TABLE = 4;  // mean outside the calibrated ADC table (extrapolated)
constexpr uint8_t FLAG_LATE          = 8;  // sampling started late (timing not as configured)

// ---------------------------------------------------------------- calibration (NVS)
constexpr uint32_t CAL_MAGIC   = 0x50424331;  // "PBC1": layout version of the record below
constexpr int      ADC_LUT_MAX = 16;

struct Calibration {
  uint32_t magic;
  uint32_t counter;                 // incremented at every saved calibration step
  // potential: E = a * code + b, fitted on multimeter readings
  uint32_t eId;                     // 0 = not calibrated
  float    a, b, eMaxResidV;
  uint8_t  eN;
  // ADC: mean raw counts -> volts at the ADC pin, piecewise linear between multimeter points
  uint32_t adcId;
  uint8_t  adcN;
  float    adcRaw[ADC_LUT_MAX];
  float    adcV[ADC_LUT_MAX];
  // zero current: mean raw counts with the working-electrode lead disconnected
  uint32_t zeroId;
  float    zeroRaw, zeroSdRaw;
};

Calibration cal;
Preferences prefs;

void calClear() {
  memset(&cal, 0, sizeof cal);
  cal.magic = CAL_MAGIC;
}

void calLoad() {
  calClear();
  if (!prefs.isKey("cal")) return;
  Calibration tmp;
  if (prefs.getBytes("cal", &tmp, sizeof tmp) == sizeof tmp && tmp.magic == CAL_MAGIC) cal = tmp;
}

bool calSave() { return prefs.putBytes("cal", &cal, sizeof cal) == sizeof cal; }

bool eCalValid()    { return cal.eId != 0; }
bool adcCalValid()  { return cal.adcId != 0 && cal.adcN >= 4; }
bool zeroCalValid() { return cal.zeroId != 0; }

// ---------------------------------------------------------------- potential <-> DAC code
float potA() { return eCalValid() ? cal.a : A_NOM; }
float potB() { return eCalValid() ? cal.b : B_NOM; }
float codeToE(int code) { return potA() * code + potB(); }
int   eToCode(float e) { return (int)lroundf((e - potB()) / potA()); }
int   clampCode(int c) { return c < 0 ? 0 : (c > DAC_MAX_CODE ? DAC_MAX_CODE : c); }

// ---------------------------------------------------------------- MCP4725
int dacCode = -1;  // last code written, -1 = unknown

bool dacWrite(int code) {
  code = clampCode(code);
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write((uint8_t)((code >> 8) & 0x0F));  // fast-mode write, normal power mode
  Wire.write((uint8_t)(code & 0xFF));
  bool ok = Wire.endTransmission() == 0;
  if (ok) dacCode = code;
  return ok;
}

// Also stores the code in the MCP4725's EEPROM: the output it takes at power-up,
// before the ESP32 has started.
bool dacWriteEeprom(int code) {
  code = clampCode(code);
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write((uint8_t)0x60);  // write DAC register and EEPROM, normal power mode
  Wire.write((uint8_t)(code >> 4));
  Wire.write((uint8_t)((code & 0x0F) << 4));
  if (Wire.endTransmission() != 0) return false;
  uint32_t t = millis();
  while (millis() - t < 200) {  // EEPROM write takes up to 50 ms; bit 7 of the status = ready
    delay(5);
    if (Wire.requestFrom((int)MCP4725_ADDR, 1) == 1 && (Wire.read() & 0x80)) {
      dacCode = code;
      return true;
    }
  }
  return false;
}

bool dacPresent() {
  Wire.beginTransmission(MCP4725_ADDR);
  return Wire.endTransmission() == 0;
}

int  restCode() { return clampCode(eToCode(0.0f)); }
void parkAtRest() { dacWrite(restCode()); }

// ---------------------------------------------------------------- ADC
esp_adc_cal_characteristics_t adcChars;
const char* adcCharName = "";

void adcInit() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC, ADC_11db);
  (void)analogRead(PIN_ADC);
  esp_adc_cal_value_t t =
      esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adcChars);
  adcCharName = t == ESP_ADC_CAL_VAL_EFUSE_TP     ? "factory eFuse two-point"
                : t == ESP_ADC_CAL_VAL_EFUSE_VREF ? "factory eFuse Vref"
                                                  : "default Vref 1100 mV";
}

// Factory characterization, interpolated between integer counts.
float efuseVolts(float raw) {
  if (raw < 0) raw = 0;
  if (raw > 4094.999f) raw = 4094.999f;
  uint32_t lo = (uint32_t)raw;
  float f = raw - lo;
  float v0 = esp_adc_cal_raw_to_voltage(lo, &adcChars);
  float v1 = esp_adc_cal_raw_to_voltage(lo + 1, &adcChars);
  return (v0 + f * (v1 - v0)) / 1000.0f;
}

// Volts at the ADC pin from a mean raw reading: multimeter table if there is one,
// else the factory characterization. *outside = beyond the table (extrapolated).
float rawToVolts(float raw, bool* outside) {
  if (outside) *outside = false;
  if (!adcCalValid()) return efuseVolts(raw);
  int n = cal.adcN, i = 0;
  if (raw < cal.adcRaw[0]) {
    i = 0;
    if (outside) *outside = true;
  } else if (raw > cal.adcRaw[n - 1]) {
    i = n - 2;
    if (outside) *outside = true;
  } else {
    while (i < n - 2 && raw > cal.adcRaw[i + 1]) i++;
  }
  float x0 = cal.adcRaw[i], x1 = cal.adcRaw[i + 1], y0 = cal.adcV[i], y1 = cal.adcV[i + 1];
  return y0 + (raw - x0) * (y1 - y0) / (x1 - x0);
}

float zeroVolts() { return zeroCalValid() ? rawToVolts(cal.zeroRaw, nullptr) : VOUT0_NOM / VOUT_PER_VPIN; }

// The negative rail implied by a calibration (eq. 2.1 and 2.3), as a cross-check: the
// potential offset and the zero-current output both follow it.
double vMinusFromB(double b) { return b * R3_OHM / R4_OHM; }
double vMinusFromVout0(double vout0) { return -vout0 * R5_OHM / R6_OHM; }

// eq. 2.3: I = (V_out - V_out(I=0)) / R6, oxidation positive
float currentFromVolts(float vPin) { return (vPin - zeroVolts()) * VOUT_PER_VPIN / R6_OHM; }

inline void waitUntilUs(uint32_t t) {
  for (;;) {
    int32_t left = (int32_t)(t - micros());
    if (left <= 0) return;
    if (left > 3000) vTaskDelay(1);  // yield while the deadline is far
  }
}

struct Window {
  float    mean, sd;  // raw counts
  uint16_t lo, hi;
  bool     late;
};

// CFG_ADC_SAMPLES readings, one every CFG_SAMPLE_INTERVAL_US, the first at tStart (micros).
Window sampleWindow(uint32_t tStart) {
  Window w;
  w.late = (int32_t)(micros() - tStart) > (int32_t)CFG_SAMPLE_INTERVAL_US;
  w.lo = 4095;
  w.hi = 0;
  uint32_t sum = 0;
  uint64_t sq = 0;
  for (int i = 0; i < CFG_ADC_SAMPLES; i++) {
    waitUntilUs(tStart + (uint32_t)i * CFG_SAMPLE_INTERVAL_US);
    uint16_t r = analogRead(PIN_ADC);
    sum += r;
    sq += (uint32_t)r * r;
    if (r < w.lo) w.lo = r;
    if (r > w.hi) w.hi = r;
  }
  w.mean = (float)sum / CFG_ADC_SAMPLES;
  float var = (float)((double)sq / CFG_ADC_SAMPLES - (double)w.mean * w.mean);
  w.sd = var > 0 ? sqrtf(var) : 0;
  return w;
}

// Several consecutive windows merged (calibration and checks).
Window sampleWindows(int n, float* sdOfMeans) {
  Window all;
  all.lo = 4095;
  all.hi = 0;
  all.late = false;
  double s = 0, s2 = 0;
  for (int k = 0; k < n; k++) {
    Window w = sampleWindow(micros() + 100);
    s += w.mean;
    s2 += (double)w.mean * w.mean;
    if (w.lo < all.lo) all.lo = w.lo;
    if (w.hi > all.hi) all.hi = w.hi;
  }
  all.mean = s / n;
  double v = n > 1 ? (s2 - s * s / n) / (n - 1) : 0;
  all.sd = 0;
  if (sdOfMeans) *sdOfMeans = v > 0 ? sqrt(v) : 0;
  return all;
}

uint8_t flagsOf(const Window& w, bool outside) {
  uint8_t f = 0;
  if (w.lo == 0) f |= FLAG_BLIND;
  if (w.hi >= 4095) f |= FLAG_SATURATED;
  if (outside) f |= FLAG_OUTSIDE_TABLE;
  if (w.late) f |= FLAG_LATE;
  return f;
}

// ---------------------------------------------------------------- serial input
String rxLine;

// true when a complete, non-empty line is available (CR, LF or CRLF terminated).
// Typed characters are echoed here, so any terminal works without a local echo.
bool pollLine(String& out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (rxLine.length()) {
        Serial.println();
        out = rxLine;
        rxLine = "";
        out.trim();
        if (out.length()) return true;
      }
    } else if (c == 8 || c == 127) {  // backspace
      if (rxLine.length()) {
        rxLine.remove(rxLine.length() - 1);
        Serial.print("\b \b");
      }
    } else if (c >= 32 && rxLine.length() < 96) {
      rxLine += c;
      Serial.print(c);
    }
  }
  return false;
}

String waitLine() {
  String s;
  while (!pollLine(s)) delay(5);
  return s;
}

void flushInput() {
  delay(20);
  while (Serial.available()) Serial.read();
  rxLine = "";
}

// a key other than CR/LF/space, or the button held for 50 ms, aborts a running scan
bool abortRequested() {
  bool stop = false;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c != '\r' && c != '\n' && c != ' ') stop = true;
  }
  if (buttonHeldFor(50)) stop = true;
  return stop;
}

bool parseNumber(String s, float& v) {
  s.trim();
  s.replace(',', '.');
  if (!s.length()) return false;
  const char* p = s.c_str();
  char* end;
  v = strtof(p, &end);
  if (end == p) return false;
  while (*end == ' ') end++;
  if (*end == 'V' || *end == 'v') end++;
  return *end == '\0' && isfinite(v);
}

bool askYes(const char* question) {
  Serial.printf("%s [y/n] ", question);
  String s = waitLine();
  return s.equalsIgnoreCase("y") || s.equalsIgnoreCase("yes");
}

// ---------------------------------------------------------------- configuration identity
uint32_t crc32(const char* s) {
  uint32_t c = 0xFFFFFFFFu;
  for (; *s; s++) {
    c ^= (uint8_t)*s;
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

// Every value of part 1 of config.h, plus the hardware values that scale the result.
uint32_t configCrc() {
  char b[192];
  snprintf(b, sizeof b, "%s|%.4f|%.4f|%d|%d|%lu|%lu|%d|%lu|%.3f|%.0f|%.3f", CONFIG_NAME,
           CFG_E_START_V, CFG_E_VERTEX_V, CFG_CYCLES, CFG_STEP_CODES,
           (unsigned long)CFG_STEP_TIME_MS, (unsigned long)CFG_QUIET_TIME_MS, CFG_ADC_SAMPLES,
           (unsigned long)CFG_SAMPLE_INTERVAL_US, DAC_VDD_V, R6_OHM, VOUT_PER_VPIN);
  return crc32(b);
}

// ---------------------------------------------------------------- scan plan
enum Mode { MODE_INFO, MODE_CV, MODE_DUMMY };

struct Plan {
  int   n;                      // number of vertices, the start included
  float target[MAX_VERTICES];   // potentials asked for
  int   code[MAX_VERTICES];     // DAC codes, all on the grid start + k * CFG_STEP_CODES
  int   segsPerCycle;
  long  points;
};

bool buildPlan(Mode mode, Plan& p, String& err) {
  float v[MAX_VERTICES];
  int n = 0;
  if (mode == MODE_DUMMY) {
    v[n++] = 0.0f;
    v[n++] = DUMMY_E_HIGH_V;
    v[n++] = DUMMY_E_LOW_V;
    v[n++] = 0.0f;
    p.segsPerCycle = 3;
  } else {
    v[n++] = CFG_E_START_V;
    for (int c = 0; c < CFG_CYCLES; c++) {
      v[n++] = CFG_E_VERTEX_V;
      v[n++] = CFG_E_START_V;
    }
    p.segsPerCycle = 2;
  }
  p.n = n;
  float a = potA();
  if (!(a > 0)) {
    err = "the potential calibration has a slope <= 0; run ecal again";
    return false;
  }
  int c0 = eToCode(v[0]);
  float stepE = CFG_STEP_CODES * a;
  p.points = 1;
  for (int k = 0; k < n; k++) {
    p.target[k] = v[k];
    long m = lroundf((v[k] - v[0]) / stepE);
    p.code[k] = c0 + (int)m * CFG_STEP_CODES;
    if (p.code[k] < 0 || p.code[k] > DAC_MAX_CODE) {
      err = String("the potential ") + String(v[k], 3) + " V needs DAC code " + p.code[k] +
            ", outside 0..4095; the reachable window is " + String(codeToE(0), 3) + " to " +
            String(codeToE(DAC_MAX_CODE), 3) + " V (low batteries? check the potential calibration)";
      return false;
    }
    if (fabsf(codeToE(p.code[k])) > E_LIMIT_V + 0.005f) {
      err = String("the potential ") + String(v[k], 3) + " V is beyond the +/-1.5 V limit";
      return false;
    }
    if (k > 0) p.points += labs((long)p.code[k] - p.code[k - 1]) / CFG_STEP_CODES;
  }
  return true;
}

// ---------------------------------------------------------------- header
void printCalibrationLines() {
  if (eCalValid())
    Serial.printf("# cal.potential: #%lu, E = a*dac_code + b, a = %.5e V/code, b = %+.5f V; "
                  "%d multimeter points, max residual %.2f mV; implied negative rail %.2f V\n",
                  (unsigned long)cal.eId, cal.a, cal.b, cal.eN, cal.eMaxResidV * 1000,
                  vMinusFromB(cal.b));
  else
    Serial.printf("# cal.potential: NONE, nominal E = %.5e V/code * dac_code %+.4f V (eq. 2.2)\n",
                  A_NOM, B_NOM);

  if (adcCalValid()) {
    Serial.printf("# cal.adc: #%lu, multimeter table, %d points (mean counts:volts at the pin)",
                  (unsigned long)cal.adcId, cal.adcN);
    for (int i = 0; i < cal.adcN; i++) Serial.printf(" %.1f:%.4f", cal.adcRaw[i], cal.adcV[i]);
    Serial.println();
  } else {
    Serial.printf("# cal.adc: NONE, %s characterization of the chip\n", adcCharName);
  }

  if (zeroCalValid())
    Serial.printf("# cal.zero: #%lu, V_out(I=0) = %.4f V (mean %.1f counts, SD of 20 ms means "
                  "%.2f counts); implied negative rail %.2f V\n",
                  (unsigned long)cal.zeroId, zeroVolts() * VOUT_PER_VPIN, cal.zeroRaw, cal.zeroSdRaw,
                  vMinusFromVout0(zeroVolts() * VOUT_PER_VPIN));
  else
    Serial.printf("# cal.zero: NONE, nominal V_out(I=0) = %.4f V (eq. 2.3)\n", VOUT0_NOM);
}

void printHeader(Mode mode, const Plan& p, float rNominal) {
  Serial.printf("# instrument: ESP32 potentiostat; analog front end of Anshori et al. 2022 "
                "(LM324, battery rails); DAC MCP4725 12-bit at I2C 0x%02X, VDD %.2f V; current read "
                "by the ESP32 ADC1 on GPIO%d, attenuation ADC_11db (12 dB in ESP-IDF), 12 bit\n",
                MCP4725_ADDR, DAC_VDD_V, PIN_ADC);
  Serial.printf("# firmware: %s %s, built %s %s\n", FW_NAME, FW_VERSION, __DATE__, __TIME__);
  if (mode == MODE_CV) Serial.println("# mode: cv, locked configuration");
  else if (mode == MODE_DUMMY) Serial.printf("# mode: dummy cell, R = %.0f ohm\n", rNominal);
  else Serial.println("# mode: info, no scan");

  Serial.printf("# config: %s, crc32 0x%08lX\n", CONFIG_NAME, (unsigned long)configCrc());
  Serial.printf("# config.program_V: %+.3f -> %+.3f -> %+.3f, %s first, %d cycle(s)\n",
                CFG_E_START_V, CFG_E_VERTEX_V, CFG_E_START_V,
                CFG_E_VERTEX_V > CFG_E_START_V ? "anodic" : "cathodic", CFG_CYCLES);
  Serial.printf("# config.step: %d DAC codes every %lu ms\n", CFG_STEP_CODES,
                (unsigned long)CFG_STEP_TIME_MS);
  Serial.printf("# config.quiet_time_s: %.1f at the start potential, not recorded\n",
                CFG_QUIET_TIME_MS / 1000.0f);
  Serial.printf("# config.sampling: mean of %d ADC readings, one every %lu us, over the last "
                "%.1f ms of each step; no other filtering\n",
                CFG_ADC_SAMPLES, (unsigned long)CFG_SAMPLE_INTERVAL_US, WINDOW_US / 1000.0f);
  if (mode == MODE_DUMMY)
    Serial.printf("# dummy.program_V: +0.000 -> %+.3f -> %+.3f -> +0.000, same step, timing and "
                  "sampling\n",
                  DUMMY_E_HIGH_V, DUMMY_E_LOW_V);

  float a = potA();
  float stepMv = CFG_STEP_CODES * a * 1000;
  Serial.printf("# scan.step_mV: %.3f (%d codes x %.4f mV per code, %s)\n", stepMv, CFG_STEP_CODES,
                a * 1000, eCalValid() ? "calibrated" : "nominal");
  Serial.printf("# scan.rate_mV_s: %.2f\n", stepMv / (CFG_STEP_TIME_MS / 1000.0f));
  Serial.print("# scan.vertices_V:");
  for (int k = 0; k < p.n; k++) Serial.printf(" %+.4f", codeToE(p.code[k]));
  Serial.print(" (DAC codes");
  for (int k = 0; k < p.n; k++) Serial.printf(" %d", p.code[k]);
  Serial.println(")");
  Serial.printf("# scan.points: %ld, duration_s: %.1f after the quiet time\n", p.points,
                p.points * CFG_STEP_TIME_MS / 1000.0f);

  printCalibrationLines();
  Serial.printf("# conversion: E_V = a*dac_code + b; I_A = (V(adc_raw) - V_out(I=0)) x %.3f / "
                "%.0f ohm (R6), oxidation positive\n",
                VOUT_PER_VPIN, R6_OHM);
  Serial.println("# columns: E_V potential vs. reference (V); I_A current (A, oxidation positive); "
                 "t_s time from the first step (s); cycle; dac_code; adc_raw mean ADC counts; "
                 "flag (0 ok, 1 ADC blind zone, 2 ADC saturated, 4 outside the ADC table, "
                 "8 sampling late; summed)");
  if (!eCalValid() || !adcCalValid() || !zeroCalValid())
    Serial.println("# WARNING: calibration incomplete (see the cal lines); values are nominal");
}

// ---------------------------------------------------------------- the scan
struct LinFit {
  double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
  void add(double x, double y) {
    n++; sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y;
  }
};

// The points of one scan, kept for the decision rule in standalone use.
constexpr int MAX_SCAN_POINTS = 1024;
struct ScanBuffer {
  int     n;
  bool    complete;
  long    flagged;
  float   e[MAX_SCAN_POINTS];
  float   i[MAX_SCAN_POINTS];
  uint8_t flags[MAX_SCAN_POINTS];
};

// Runs one scan, printing it in the file format. Keeps the points in buf if given, and adds
// the line "# <note>" to the header if given. Returns true if the scan ran to its end.
bool runScan(Mode mode, float rNominal, ScanBuffer* buf = nullptr, const char* note = nullptr) {
  Plan p;
  String err;
  if (buf) {
    buf->n = 0;
    buf->complete = false;
    buf->flagged = 0;
  }
  if (!buildPlan(mode, p, err)) {
    Serial.printf("# ERROR: %s\n", err.c_str());
    return false;
  }
  uiQuiet();  // LED off, display dark: same load on the 3.3 V rail as during calibration
  flushInput();

  Serial.println("# begin scan");
  printHeader(mode, p, rNominal);
  if (note) Serial.printf("# %s\n", note);
  Serial.println("E_V,I_A,t_s,cycle,dac_code,adc_raw,flag");

  bool aborted = false, i2cFailed = false;
  long done = 0, nBlind = 0, nSat = 0, nOutside = 0, nLate = 0;
  LinFit fit;

  // quiet time at the start potential
  if (!dacWrite(p.code[0])) i2cFailed = true;
  uint32_t tq = millis();
  while (!i2cFailed && millis() - tq < CFG_QUIET_TIME_MS) {
    if (abortRequested()) { aborted = true; break; }
    delay(5);
  }

  if (!aborted && !i2cFailed) {
    int code = p.code[0];
    int seg = 1;  // index of the vertex being approached
    uint32_t t0 = micros();
    dacWrite(code);
    for (long k = 0;; k++) {
      while (seg < p.n && code == p.code[seg]) seg++;  // vertex reached
      int segForCycle = seg < p.n ? seg : p.n - 1;
      int cycle = 1 + (segForCycle - 1) / p.segsPerCycle;
      bool last = seg >= p.n;
      int next = code;
      if (!last) next = code + (p.code[seg] > code ? CFG_STEP_CODES : -CFG_STEP_CODES);

      uint32_t tStep = t0 + (uint32_t)k * STEP_US;
      Window w = sampleWindow(tStep + STEP_US - WINDOW_US);
      waitUntilUs(tStep + STEP_US);
      bool ok = last || dacWrite(next);  // next step starts on time; the point is printed after

      bool outside;
      float v = rawToVolts(w.mean, &outside);
      float e = codeToE(code);
      float i = currentFromVolts(v);
      uint8_t f = flagsOf(w, outside);
      float t = (tStep + STEP_US - WINDOW_US / 2 - t0) / 1e6f;
      Serial.printf("%.4f,%.4e,%.3f,%d,%d,%.2f,%u\n", e, i, t, cycle, code, w.mean, f);
      done++;
      if (f & FLAG_BLIND) nBlind++;
      if (f & FLAG_SATURATED) nSat++;
      if (f & FLAG_OUTSIDE_TABLE) nOutside++;
      if (f & FLAG_LATE) nLate++;
      if (f == 0) fit.add(e, i);
      if (buf && buf->n < MAX_SCAN_POINTS) {
        buf->e[buf->n] = e;
        buf->i[buf->n] = i;
        buf->flags[buf->n] = f;
        buf->n++;
        if (f) buf->flagged++;
      }

      if (!ok) { i2cFailed = true; break; }
      if (last) break;
      if (abortRequested()) { aborted = true; break; }
      code = next;
    }
  }
  parkAtRest();

  Serial.printf("# end: points %ld of %ld; flagged: blind %ld, saturated %ld, outside table %ld, "
                "late %ld; %s\n",
                done, p.points, nBlind, nSat, nOutside, nLate,
                i2cFailed ? "STOPPED: no answer from the DAC (I2C)"
                          : (aborted ? "ABORTED by the operator" : "complete"));

  if (mode == MODE_DUMMY && fit.n >= 3) {
    double d = fit.n * fit.sxx - fit.sx * fit.sx;
    double slope = (fit.n * fit.sxy - fit.sx * fit.sy) / d;
    double icpt = (fit.sy - slope * fit.sx) / fit.n;
    double dy = fit.n * fit.syy - fit.sy * fit.sy;
    double r2 = dy > 0 ? (fit.n * fit.sxy - fit.sx * fit.sy) * (fit.n * fit.sxy - fit.sx * fit.sy) / (d * dy) : 0;
    double ssr = fit.syy - icpt * fit.sy - slope * fit.sxy;
    double rms = (fit.n > 2 && ssr > 0) ? sqrt(ssr / (fit.n - 2)) : 0;
    double rMeas = slope != 0 ? 1.0 / slope : 0;
    Serial.printf("# dummy.fit: I = E/R + I0 on %.0f unflagged points: R = %.1f ohm (nominal %.0f, "
                  "deviation %+.2f %%), I0 = %+.3e A, r2 = %.6f, rms residual %.3e A\n",
                  fit.n, rMeas, rNominal, rNominal > 0 ? 100.0 * (rMeas - rNominal) / rNominal : 0.0,
                  icpt, r2, rms);
  }
  Serial.println("# end scan");
  bool complete = !aborted && !i2cFailed && done == p.points;
  if (buf) buf->complete = complete && buf->n == done;
  return complete;
}

// ---------------------------------------------------------------- calibration commands
// Potential: E between the WE and RE leads, read by a multimeter, at five DAC codes.
void cmdEcal() {
  uiQuiet();
  Serial.println(
      "\nPOTENTIAL CALIBRATION\n"
      "Connect a resistor (the 10 kohm of the dummy cell) between the WE lead and the joined RE and\n"
      "CE leads. Multimeter on DC volts: red probe on the WE lead, black probe on the RE lead.\n"
      "At each point type the reading in volts and press Enter (q = quit).");
  const int N = 5;
  const float target[N] = {-1.2f, -0.6f, 0.0f, 0.6f, 1.2f};
  int code[N];
  float meas[N];
  for (int k = 0; k < N; k++) {
    code[k] = clampCode((int)lroundf((target[k] - B_NOM) / A_NOM));  // nominal map on purpose
    if (!dacWrite(code[k])) { Serial.println("No answer from the DAC (I2C). Stopped."); return; }
    delay(500);
    for (;;) {
      Serial.printf("Point %d/%d: DAC code %d (nominal E %+.2f V). Reading: ", k + 1, N, code[k],
                    target[k]);
      String s = waitLine();
      if (s.equalsIgnoreCase("q")) { parkAtRest(); Serial.println("Quit; nothing saved."); return; }
      if (parseNumber(s, meas[k]) && fabsf(meas[k]) < 4.0f) break;
      Serial.println("  not a number in volts, type it again");
    }
  }
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int k = 0; k < N; k++) {
    sx += code[k]; sy += meas[k]; sxx += (double)code[k] * code[k]; sxy += (double)code[k] * meas[k];
  }
  double a = (N * sxy - sx * sy) / (N * sxx - sx * sx);
  double b = (sy - a * sx) / N;
  double maxRes = 0;
  Serial.println("  code    reading_V   fit_V      residual_mV");
  for (int k = 0; k < N; k++) {
    double fitV = a * code[k] + b, res = meas[k] - fitV;
    if (fabs(res) > maxRes) maxRes = fabs(res);
    Serial.printf("  %4d   %+9.4f   %+9.4f   %+7.2f\n", code[k], meas[k], fitV, res * 1000);
  }
  double rail = vMinusFromB(b);
  Serial.printf("E = a*code + b: a = %.4f mV/code (nominal %.4f), b = %+.4f V (nominal %+.4f, "
                "implies a negative rail of %.2f V), max residual %.2f mV\n",
                a * 1000, A_NOM * 1000, b, B_NOM, rail, maxRes * 1000);
  if (eCalValid())
    Serial.printf("Saved potential calibration #%lu: E at 0 V code differs by %+.1f mV, at the vertex "
                  "code by %+.1f mV (drift since then)\n",
                  (unsigned long)cal.eId,
                  ((a * restCode() + b) - codeToE(restCode())) * 1000,
                  ((a * eToCode(CFG_E_VERTEX_V) + b) - codeToE(eToCode(CFG_E_VERTEX_V))) * 1000);
  bool plausible = a > 0.85 * A_NOM && a < 1.15 * A_NOM && rail >= V_MINUS_MIN_V &&
                   rail <= V_MINUS_MAX_V && maxRes < 0.010;
  if (!plausible) {
    parkAtRest();
    Serial.printf("REJECTED: slope off by more than 15 %%, a negative rail outside %.1f to %.1f V, or a\n"
                  "residual above 10 mV. Check the probes (red on WE), the resistor, the batteries and\n"
                  "DAC_VDD_V. Nothing saved.\n",
                  V_MINUS_MIN_V, V_MINUS_MAX_V);
    return;
  }
  if (!askYes("Save this potential calibration?")) { parkAtRest(); Serial.println("Not saved."); return; }
  cal.a = a; cal.b = b; cal.eMaxResidV = maxRes; cal.eN = N;
  cal.eId = ++cal.counter;
  bool saved = calSave();
  int rc = restCode();
  bool ee = dacWriteEeprom(rc);
  Serial.printf("%s as calibration #%lu. Rest potential 0 V = DAC code %d%s.\n",
                saved ? "Saved" : "SAVE FAILED", (unsigned long)cal.eId, rc,
                ee ? ", also stored as the DAC's power-up value" : " (power-up value NOT stored)");
}

// ADC: the DAC sweeps the TIA output through a resistor; the multimeter reads the ADC pin.
void cmdAdccal() {
  uiQuiet();
  Serial.println(
      "\nADC CALIBRATION\n"
      "Connect a 4.7 or 5.1 kohm resistor between the WE lead and the joined RE and CE leads, so that\n"
      "the output sweeps the whole ADC range. Multimeter on DC volts: red probe on the ADC input\n"
      "(GPIO32), black probe on GND. At each point type the reading in volts and press Enter\n"
      "(s = skip this point, q = quit). Points where the ADC is at a limit are skipped automatically.");
  const int N = 11;
  float raw[ADC_LUT_MAX], volts[ADC_LUT_MAX];
  int m = 0;
  // evenly spaced codes over the part of the DAC range that stays within +/-1.5 V
  int codeLo = clampCode((int)ceilf((-E_LIMIT_V - potB()) / potA()));
  int codeHi = clampCode((int)floorf((E_LIMIT_V - potB()) / potA()));
  for (int k = 0; k < N; k++) {
    int code = codeLo + (int)((long)k * (codeHi - codeLo) / (N - 1));
    float e = codeToE(code);
    if (!dacWrite(code)) { Serial.println("No answer from the DAC (I2C). Stopped."); return; }
    delay(400);
    float sdm;
    Window w = sampleWindows(10, &sdm);
    if (w.lo == 0 || w.hi >= 4095) {
      Serial.printf("Point %d/%d: E %+.2f V, ADC at its limit (%u..%u counts): skipped\n", k + 1, N, e,
                    w.lo, w.hi);
      continue;
    }
    for (;;) {
      Serial.printf("Point %d/%d: E %+.2f V, ADC mean %.1f counts (factory estimate %.4f V). Reading: ",
                    k + 1, N, e, w.mean, efuseVolts(w.mean));
      String s = waitLine();
      if (s.equalsIgnoreCase("q")) { parkAtRest(); Serial.println("Quit; nothing saved."); return; }
      if (s.equalsIgnoreCase("s")) break;
      float v;
      if (parseNumber(s, v) && v >= 0 && v < 3.6f) {
        raw[m] = w.mean;
        volts[m] = v;
        m++;
        break;
      }
      Serial.println("  not a number between 0 and 3.6 V, type it again");
    }
  }
  parkAtRest();
  // sort by counts
  for (int i = 1; i < m; i++)
    for (int j = i; j > 0 && raw[j] < raw[j - 1]; j--) {
      float t = raw[j]; raw[j] = raw[j - 1]; raw[j - 1] = t;
      t = volts[j]; volts[j] = volts[j - 1]; volts[j - 1] = t;
    }
  bool monotonic = true;
  for (int i = 1; i < m; i++)
    if (!(raw[i] > raw[i - 1] + 1.0f && volts[i] > volts[i - 1])) monotonic = false;
  Serial.println("  counts     reading_V   factory_V   difference_mV");
  for (int i = 0; i < m; i++)
    Serial.printf("  %7.1f   %8.4f    %8.4f    %+7.1f\n", raw[i], volts[i], efuseVolts(raw[i]),
                  (efuseVolts(raw[i]) - volts[i]) * 1000);
  if (m < 4 || !monotonic) {
    Serial.println("REJECTED: fewer than 4 points, or readings that do not rise with the counts.\n"
                   "Check the probes and the resistor. Nothing saved.");
    return;
  }
  Serial.printf("Table covers %.3f to %.3f V at the pin, i.e. currents of about %+.0f to %+.0f uA;\n"
                "points outside it will carry flag 4.\n",
                volts[0], volts[m - 1], (volts[0] * VOUT_PER_VPIN - VOUT0_NOM) / R6_OHM * 1e6,
                (volts[m - 1] * VOUT_PER_VPIN - VOUT0_NOM) / R6_OHM * 1e6);
  if (!askYes("Save this ADC table?")) { Serial.println("Not saved."); return; }
  cal.adcN = m;
  for (int i = 0; i < m; i++) { cal.adcRaw[i] = raw[i]; cal.adcV[i] = volts[i]; }
  cal.adcId = ++cal.counter;
  Serial.printf("%s as calibration #%lu.\n", calSave() ? "Saved" : "SAVE FAILED", (unsigned long)cal.adcId);
}

// Zero current: TIA output with nothing connected to the WE lead.
void cmdZero() {
  uiQuiet();
  Serial.println(
      "\nZERO CURRENT\n"
      "Disconnect the WE lead (nothing may be connected to it), then press Enter (q = quit).");
  String s = waitLine();
  if (s.equalsIgnoreCase("q")) { Serial.println("Quit; nothing saved."); return; }
  float sdm;
  Window w = sampleWindows(50, &sdm);  // 50 x 20 ms = 1 s
  if (w.lo == 0 || w.hi >= 4095) {
    Serial.printf("REJECTED: ADC at a limit (%u..%u counts). Nothing saved.\n", w.lo, w.hi);
    return;
  }
  bool outside;
  float v = rawToVolts(w.mean, &outside);
  float lsbV = rawToVolts(w.mean + 1, nullptr) - v;
  double rail = vMinusFromVout0(v * VOUT_PER_VPIN);
  Serial.printf("V_out(I=0) = %.4f V (nominal %.4f V, implies a negative rail of %.2f V), mean %.1f "
                "counts; SD of the 20 ms means %.2f counts = %.3f uA\n",
                v * VOUT_PER_VPIN, VOUT0_NOM, rail, w.mean, sdm,
                sdm * lsbV * VOUT_PER_VPIN / R6_OHM * 1e6);
  if (zeroCalValid())
    Serial.printf("Saved zero #%lu: %.4f V; difference %+.1f mV = %+.2f uA (drift since then)\n",
                  (unsigned long)cal.zeroId, zeroVolts() * VOUT_PER_VPIN,
                  (v - zeroVolts()) * VOUT_PER_VPIN * 1000, (v - zeroVolts()) * VOUT_PER_VPIN / R6_OHM * 1e6);
  if (rail < V_MINUS_MIN_V || rail > V_MINUS_MAX_V) {
    Serial.printf("REJECTED: implies a negative rail outside %.1f to %.1f V. Is the WE lead free?\n"
                  "Are the batteries charged? Nothing saved.\n",
                  V_MINUS_MIN_V, V_MINUS_MAX_V);
    return;
  }
  if (!askYes("Save this zero?")) { Serial.println("Not saved."); return; }
  cal.zeroRaw = w.mean;
  cal.zeroSdRaw = sdm;
  cal.zeroId = ++cal.counter;
  Serial.printf("%s as calibration #%lu.\n", calSave() ? "Saved" : "SAVE FAILED", (unsigned long)cal.zeroId);
}

// ---------------------------------------------------------------- other commands
void cmdSet(const String& arg) {
  float e;
  if (!parseNumber(arg, e)) { Serial.println("usage: set <E in volts>, e.g. set 0.5"); return; }
  if (fabsf(e) > E_LIMIT_V) { Serial.println("refused: beyond +/-1.5 V"); return; }
  int c = clampCode(eToCode(e));
  if (!dacWrite(c)) { Serial.println("no answer from the DAC (I2C)"); return; }
  Serial.printf("E = %+.4f V (DAC code %d, %s)\n", codeToE(c), c, eCalValid() ? "calibrated" : "nominal");
}

void cmdRead(const String& arg) {
  int n = arg.length() ? arg.toInt() : 1;
  if (n < 1) n = 1;
  if (n > 200) n = 200;
  Serial.println("E_V,I_A,adc_raw,adc_sd,V_pin,flag");
  for (int k = 0; k < n; k++) {
    Window w = sampleWindow(micros() + 100);
    bool outside;
    float v = rawToVolts(w.mean, &outside);
    Serial.printf("%.4f,%.4e,%.2f,%.2f,%.4f,%u\n", dacCode >= 0 ? codeToE(dacCode) : NAN,
                  currentFromVolts(v), w.mean, w.sd, v, flagsOf(w, outside));
    if (n > 1) delay(200);
    if (abortRequested()) break;
  }
}

// ---------------------------------------------------------------- the decision rule
bool ruleSelfTestOk = false;  // set at power-up and by "selftest"

bool calibrated() { return eCalValid() && adcCalValid() && zeroCalValid(); }

// The rule is used only in the configuration it was trained in, and only if it passes its
// self-test on this chip.
bool ruleUsable() {
  return ruleModelPresent() && ruleModelConfigCrc() == configCrc() && ruleSelfTestOk;
}

const char* ruleState() {
  if (!ruleModelPresent()) return "none loaded";
  if (ruleModelConfigCrc() != configCrc()) return "trained in ANOTHER CONFIGURATION, not used";
  if (!ruleSelfTestOk) return "SELF-TEST FAILED, not used";
  return "in use";
}

void printRuleLine() {
  if (!ruleModelPresent()) {
    Serial.println("# rule: none loaded (model.h is the placeholder); standalone use shows the "
                   "peak signal only");
    return;
  }
  Serial.printf("# rule: %s, trained on %d scans of %d preparations in configuration crc32 "
                "0x%08lX; P_MIN %.2f; peak blank limit %.3e A; %s\n",
                ruleModelName(), ruleTrainScans(), ruleTrainPreparations(),
                (unsigned long)ruleModelConfigCrc(), rulePMin(), rulePeakLimitA(), ruleState());
}

const char* verdictName(Verdict v) {
  switch (v) {
    case V_BLANK: return "blank";
    case V_PROPINEB: return "propineb";
    case V_OTHER: return "other substance";
    case V_INDETERMINATE: return "indeterminate";
    default: return "none";
  }
}

void cmdInfo() {
  Plan p;
  String err;
  if (!buildPlan(MODE_CV, p, err)) Serial.printf("# ERROR: %s\n", err.c_str());
  else printHeader(MODE_INFO, p, 0);
  Serial.printf("# dac: %s, current code %d\n", dacPresent() ? "answering" : "NOT ANSWERING", dacCode);
  printRuleLine();
  Serial.printf("# standalone: display %s at I2C 0x%02X; button GPIO%d; LED R/G/B GPIO%d/%d/%d; "
                "%d scans per sample\n",
                uiDisplayPresent() ? "found" : "NOT FOUND", OLED_I2C_ADDR, PIN_BUTTON, PIN_LED_R,
                PIN_LED_G, PIN_LED_B, SAMPLE_SCANS);
}

void cmdSelftest() {
  if (!ruleModelPresent()) {
    Serial.println("No rule loaded (model.h is the placeholder): nothing to test.");
    return;
  }
  int fails = ruleSelfTest([](const char* line) { Serial.printf("  %s\n", line); });
  ruleSelfTestOk = fails == 0;
  Serial.printf("Self-test of the rule %s: %s\n", ruleModelName(),
                fails == 0 ? "passed" : "FAILED; the rule will not be used");
  printRuleLine();
}

void cmdLed(String arg) {
  arg.toLowerCase();
  LedColor c = arg == "red" ? LED_RED
             : arg == "green" ? LED_GREEN
             : arg == "yellow" ? LED_YELLOW
             : arg == "blue" ? LED_BLUE
                             : LED_OFF;
  if (c == LED_OFF && arg != "off") {
    Serial.println("usage: led red|green|yellow|blue|off");
    return;
  }
  ledSet(c);
  Serial.printf("LED %s\n", ledName(c));
}

void cmdClearcal() {
  Serial.print("Erase all calibrations? Type YES to confirm: ");
  String s = waitLine();
  if (s != "YES") { Serial.println("Kept."); return; }
  uint32_t counter = cal.counter;  // keep numbering calibrations upwards
  calClear();
  cal.counter = counter;
  calSave();
  parkAtRest();
  Serial.println("Calibrations erased; nominal values apply.");
}

void cmdHelp() {
  Serial.println(
      "\nCommands (one per line):\n"
      "  run           staircase CV in the locked configuration (needs all three calibrations)\n"
      "  dummy <ohm>   dummy-cell scan, 0 -> +1 -> -1 -> 0 V, with a straight-line fit, e.g. dummy 10000\n"
      "  info          configuration and calibration, as printed in the file header\n"
      "  ecal          calibrate the potential against a multimeter (5 points)\n"
      "  adccal        calibrate the ADC against a multimeter (up to 11 points)\n"
      "  zero          measure the output at zero current (WE lead disconnected)\n"
      "  set <V>       hold the working electrode at a potential, e.g. set 0.5\n"
      "  read [n]      read the current n times at the present potential\n"
      "  clearcal      erase all calibrations\n"
      "  selftest      check the decision rule of model.h on this chip\n"
      "  led <color>   red, green, yellow, blue or off (wiring check)\n"
      "Any key, or the button, stops a running scan. Session routine: zero, ecal, one dummy scan,\n"
      "then run. Standalone: press the button for a sample of two scans and a verdict.");
}

void cmdRun() {
  if (!eCalValid() || !adcCalValid() || !zeroCalValid()) {
    Serial.printf("# ERROR: run needs all three calibrations; missing:%s%s%s\n",
                  eCalValid() ? "" : " ecal", adcCalValid() ? "" : " adccal",
                  zeroCalValid() ? "" : " zero");
    return;
  }
  runScan(MODE_CV, 0);
}

void cmdDummy(const String& arg) {
  float r;
  if (!parseNumber(arg, r) || r < 1000 || r > 1e6) {
    Serial.println("# ERROR: usage: dummy <resistance in ohm, 1000 to 1000000>, e.g. dummy 10000");
    return;
  }
  runScan(MODE_DUMMY, r);
}

// ---------------------------------------------------------------- standalone use
// idle -(press)-> ready for scan 1 -(press)-> scan -> ready for scan 2 -(press)-> scan -> result
// A long press cancels a sample; any press leaves the result. Serial commands work throughout.
enum UiState { UI_IDLE, UI_READY, UI_RESULT };
UiState    uiState = UI_IDLE;
int        uiScansDone = 0;
bool       uiRedraw = true;
ScanBuffer sampleScans[SAMPLE_SCANS];

void drawIdle() {
  char l1[24], l2[24], l3[24];
  snprintf(l1, sizeof l1, "config %s", CONFIG_NAME);
  if (calibrated())
    snprintf(l2, sizeof l2, "cal #%lu #%lu #%lu", (unsigned long)cal.eId, (unsigned long)cal.adcId,
             (unsigned long)cal.zeroId);
  else
    snprintf(l2, sizeof l2, "NOT CALIBRATED");
  if (ruleUsable()) snprintf(l3, sizeof l3, "%s", ruleModelName());
  else if (ruleModelPresent()) snprintf(l3, sizeof l3, "rule NOT USABLE");
  else snprintf(l3, sizeof l3, "no rule loaded");
  uiShow(true, "Propineb screen", l1, l2, l3, calibrated() ? "press: new sample" : "calibrate via serial");
}

void drawReady() {
  char t[32];
  snprintf(t, sizeof t, "Scan %d of %d", uiScansDone + 1, SAMPLE_SCANS);
  if (uiScansDone == 0) uiShow(true, t, "electrode in the", "sample", "press: start", "hold: cancel");
  else uiShow(true, t, "renew the surface,", "electrode back in", "press: start", "hold: cancel");
}

// The verdict of one sample: the rule of classify.py on its scans, made conservative here in
// one way only: a sample with flagged readings is indeterminate.
void finishSample() {
  ScanJudgement j[SAMPLE_SCANS];
  long flagged = 0;
  for (int k = 0; k < SAMPLE_SCANS; k++) {
    j[k] = ruleJudgeScan(sampleScans[k].e, sampleScans[k].i, sampleScans[k].n);
    flagged += sampleScans[k].flagged;
  }
  SampleResult r = ruleCombine(j, SAMPLE_SCANS);
  if (ruleModelPresent() && !ruleUsable()) {
    r.verdict = V_NO_RULE;
    r.reason = "rule not usable";
  } else if (r.verdict != V_NO_RULE && flagged > 0) {
    r.verdict = V_INDETERMINATE;
    r.reason = "readings out of range";
  }

  // one comment line on the serial output, for a terminal that keeps a log
  Serial.printf("# sample: verdict %s", verdictName(r.verdict));
  if (r.reason[0]) Serial.printf(" (%s)", r.reason);
  if (ruleUsable()) Serial.printf("; p blank %.3f, propineb %.3f, other %.3f", r.p[0], r.p[1], r.p[2]);
  Serial.print("; peak");
  for (int k = 0; k < SAMPLE_SCANS; k++) {
    if (j[k].ok) Serial.printf(" %.3e", j[k].peakA);
    else Serial.printf(" n/a (%s)", j[k].why);
  }
  Serial.print(" A");
  if (ruleUsable())
    Serial.printf(" (blank limit %.3e A, %s)", rulePeakLimitA(), r.peakAbove ? "above" : "not above");
  Serial.printf("; flagged points %ld; rule %s\n", flagged,
                ruleModelPresent() ? ruleModelName() : "none");

  // screen and LED
  char pl[24];
  snprintf(pl, sizeof pl, "p b%.2f p%.2f o%.2f", r.p[0], r.p[1], r.p[2]);
  String kl = "peak";
  for (int k = 0; k < SAMPLE_SCANS; k++) kl += j[k].ok ? " " + String(j[k].peakA * 1e6f, 1) : String(" --");
  kl += " uA";
  switch (r.verdict) {
    case V_PROPINEB:
      ledSet(LED_RED);
      uiShow(true, "PROPINEB", "screen positive:", "confirm in the lab", pl, kl.c_str());
      break;
    case V_BLANK:
      ledSet(LED_GREEN);
      uiShow(true, "BLANK", "screen negative", pl, kl.c_str());
      break;
    case V_OTHER:
      ledSet(LED_YELLOW);
      uiShow(true, "OTHER SUBSTANCE", "not classed propineb", pl, kl.c_str());
      break;
    case V_INDETERMINATE:
      ledSet(LED_BLUE);
      uiShow(true, "INDETERMINATE", r.reason, "retest or send to lab", pl, kl.c_str());
      break;
    default:
      ledSet(LED_OFF);
      uiShow(true, ruleModelPresent() ? "RULE NOT USABLE" : "NO RULE LOADED", "signal only,",
             "no verdict", kl.c_str(), flagged ? "readings flagged" : nullptr);
  }
}

void runSampleScan() {
  int k = uiScansDone;
  Plan p;
  String err;
  if (!buildPlan(MODE_CV, p, err)) {
    Serial.printf("# ERROR: %s\n", err.c_str());
    uiShow(true, "ERROR", "no scan possible,", "see serial output", "press: back");
    uiState = UI_RESULT;
    return;
  }
  char t[32], l1[24], note[96];
  snprintf(t, sizeof t, "Scanning %d of %d", k + 1, SAMPLE_SCANS);
  snprintf(l1, sizeof l1, "about %lu s",
           (unsigned long)((CFG_QUIET_TIME_MS + p.points * CFG_STEP_TIME_MS) / 1000 + 1));
  uiShow(true, t, l1, DISPLAY_DARK_DURING_SCANS ? "screen dark meanwhile" : "", "button: stop");
  delay(1500);
  snprintf(note, sizeof note, "standalone: scan %d of %d of one sample, started with the button",
           k + 1, SAMPLE_SCANS);
  bool complete = runScan(MODE_CV, 0, &sampleScans[k], note);
  buttonClear();
  if (!complete || !sampleScans[k].complete) {
    uiShow(true, "STOPPED", "scan not complete,", "no result", "press: back");
    uiState = UI_RESULT;
    return;
  }
  uiScansDone++;
  if (uiScansDone < SAMPLE_SCANS) {
    uiState = UI_READY;
    uiRedraw = true;
  } else {
    finishSample();
    uiState = UI_RESULT;
  }
}

void standaloneStep(ButtonEvent ev) {
  switch (uiState) {
    case UI_IDLE:
      if (ev != BTN_SHORT) break;
      if (!calibrated()) {
        uiShow(true, "NOT CALIBRATED", "run ecal, adccal and", "zero from a serial", "terminal first");
        delay(3000);
        uiRedraw = true;
        break;
      }
      uiScansDone = 0;
      uiState = UI_READY;
      uiRedraw = true;
      break;
    case UI_READY:
      if (ev == BTN_LONG) {
        uiState = UI_IDLE;
        uiRedraw = true;
      } else if (ev == BTN_SHORT) {
        runSampleScan();
      }
      break;
    case UI_RESULT:
      if (ev != BTN_NONE) {
        ledSet(LED_OFF);
        uiState = UI_IDLE;
        uiRedraw = true;
      }
      break;
  }
  if (uiRedraw) {
    uiRedraw = false;
    if (uiState == UI_IDLE) drawIdle();
    else if (uiState == UI_READY) drawReady();
  }
}

// after a serial command that drove the cell: the sample in progress, if any, is abandoned
void uiAfterCellCommand() {
  ledSet(LED_OFF);
  uiState = UI_IDLE;
  uiScansDone = 0;
  uiRedraw = true;
  buttonClear();
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(100);

  prefs.begin("pbscreen", false);
  calLoad();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  bool dacOk = dacWrite(restCode());  // rest at 0 V as early as possible

  adcInit();
  uiBegin();
  const LedColor check[3] = {LED_RED, LED_GREEN, LED_BLUE};  // LED wiring check at power-up
  for (LedColor c : check) {
    ledSet(c);
    delay(150);
  }
  ledSet(LED_OFF);
  if (ruleModelPresent()) ruleSelfTestOk = ruleSelfTest([](const char*) {}) == 0;

  Serial.printf("\n%s %s ready. Configuration %s (crc32 0x%08lX). Type help.\n", FW_NAME,
                FW_VERSION, CONFIG_NAME, (unsigned long)configCrc());
  if (!dacOk) Serial.printf("ERROR: the MCP4725 does not answer at I2C 0x%02X; check the wiring.\n", MCP4725_ADDR);
  Serial.printf("Calibration: potential %s, ADC %s, zero %s. Display %s.\n",
                eCalValid() ? "ok" : "MISSING", adcCalValid() ? "ok" : "MISSING",
                zeroCalValid() ? "ok" : "MISSING", uiDisplayPresent() ? "found" : "not found");
  printRuleLine();
  buttonClear();
}

void handleCommand(const String& line) {
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String arg = sp < 0 ? String("") : line.substring(sp + 1);
  cmd.toLowerCase();
  arg.trim();
  bool drivesCell = cmd == "run" || cmd == "dummy" || cmd == "ecal" || cmd == "adccal" ||
                    cmd == "zero" || cmd == "set";

  if (cmd == "run") cmdRun();
  else if (cmd == "dummy") cmdDummy(arg);
  else if (cmd == "info") cmdInfo();
  else if (cmd == "ecal") cmdEcal();
  else if (cmd == "adccal") cmdAdccal();
  else if (cmd == "zero") cmdZero();
  else if (cmd == "set") cmdSet(arg);
  else if (cmd == "read") cmdRead(arg);
  else if (cmd == "clearcal") cmdClearcal();
  else if (cmd == "selftest") cmdSelftest();
  else if (cmd == "led") cmdLed(arg);
  else if (cmd == "help" || cmd == "?") cmdHelp();
  else Serial.printf("unknown command '%s'; type help\n", cmd.c_str());

  if (drivesCell) uiAfterCellCommand();
  else if (uiState == UI_IDLE) uiRedraw = true;  // calibration or rule status may have changed
}

void loop() {
  String line;
  if (pollLine(line)) {
    handleCommand(line);
    return;
  }
  standaloneStep(buttonPoll());
  delay(5);
}
