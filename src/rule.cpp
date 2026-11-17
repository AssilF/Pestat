// rule.cpp -- see rule.h. Mirrors analysis/classify.py line by line; keep the two in step.
#include "rule.h"
#include "model.h"

#include <math.h>
#include <stdio.h>

#ifndef MODEL_PRESENT
#error "model.h must define MODEL_PRESENT"
#endif

namespace {

// ------------------------------------------------------------------ grid and peak window
// Without a model: the settings of classify.py, fixed on 2026-11-16 before any measurement:
// GRID = np.linspace(0.02, 0.98, 49); PEAK_WINDOW = (0.02, 0.30), which selects grid points
// 0..13 (np.linspace gives 0.30000000000000004 for point 14). A model carries its own copy.
#if MODEL_PRESENT
constexpr int N_GRID  = MODEL_N_GRID;
constexpr int PEAK_K0 = MODEL_PEAK_K0;
constexpr int PEAK_K1 = MODEL_PEAK_K1;
const float* grid() { return MODEL_GRID; }
#else
constexpr int N_GRID  = 49;
constexpr int PEAK_K0 = 0;
constexpr int PEAK_K1 = 13;
float defaultGrid[N_GRID];
bool  defaultGridReady = false;
const float* grid() {
  if (!defaultGridReady) {  // as np.linspace: k * step + start, last point = stop
    double step = (0.98 - 0.02) / (N_GRID - 1);
    for (int k = 0; k < N_GRID; k++) defaultGrid[k] = (float)(k * step + 0.02);
    defaultGrid[N_GRID - 1] = 0.98f;
    defaultGridReady = true;
  }
  return defaultGrid;
}
#endif

// np.interp(x, xp, fp) with xp increasing; xp[k] = xs[k * stride], fp[k] = fs[k * stride]
float interp(float x, const float* xs, const float* fs, int n, int stride) {
  auto X = [&](int k) { return xs[k * stride]; };
  auto F = [&](int k) { return fs[k * stride]; };
  if (x <= X(0)) return F(0);
  if (x >= X(n - 1)) return F(n - 1);
  int lo = 0, hi = n - 1;  // X(lo) < x < X(hi)
  while (hi - lo > 1) {
    int mid = (lo + hi) / 2;
    if (X(mid) <= x) lo = mid;
    else hi = mid;
  }
  double slope = ((double)F(hi) - F(lo)) / ((double)X(hi) - X(lo));
  return (float)(F(lo) + slope * ((double)x - X(lo)));
}

// classify.read_scan: forward sweep up to the first maximum of E, return sweep from there
bool sweeps(const float* e, const float* i, int n, float* fwd, float* ret, bool* retOk,
            const char** why) {
  if (n < 3) { *why = "too few points"; return false; }
  int top = 0;
  for (int k = 1; k < n; k++)
    if (e[k] > e[top]) top = k;
  float fmin = e[0];
  for (int k = 1; k <= top; k++)
    if (e[k] < fmin) fmin = e[k];
  const float* g = grid();
  if (fmin > g[0] || e[top] < g[N_GRID - 1]) { *why = "scan not on the grid"; return false; }
  for (int k = 0; k < N_GRID; k++) fwd[k] = interp(g[k], e, i, top + 1, 1);
  int m = n - top;  // points of the return sweep, the vertex included
  float rmin = e[top];
  for (int k = top; k < n; k++)
    if (e[k] < rmin) rmin = e[k];
  *retOk = m >= 2 && rmin <= g[0];
  if (*retOk)
    for (int k = 0; k < N_GRID; k++) ret[k] = interp(g[k], e + n - 1, i + n - 1, m, -1);
  return true;
}

// classify.peak_height
float peakHeight(const float* fwd) {
  const float* g = grid();
  double x0 = g[PEAK_K0], x1 = g[PEAK_K1], y0 = fwd[PEAK_K0], y1 = fwd[PEAK_K1];
  double best = -INFINITY;
  for (int k = PEAK_K0; k <= PEAK_K1; k++) {
    double line = y0 + (y1 - y0) * (g[k] - x0) / (x1 - x0);
    if (fwd[k] - line > best) best = fwd[k] - line;
  }
  return (float)best;
}

}  // namespace

// ------------------------------------------------------------------ model facts
bool ruleModelPresent() { return MODEL_PRESENT; }

#if MODEL_PRESENT
const char* ruleModelName() { return MODEL_NAME; }
uint32_t    ruleModelConfigCrc() { return MODEL_CONFIG_CRC; }
float       rulePMin() { return MODEL_P_MIN; }
float       rulePeakLimitA() { return MODEL_PEAK_LIMIT_A; }
int         ruleTrainScans() { return MODEL_N_TRAIN_SCANS; }
int         ruleTrainPreparations() { return MODEL_N_TRAIN_PREPS; }
#else
const char* ruleModelName() { return ""; }
uint32_t    ruleModelConfigCrc() { return 0; }
float       rulePMin() { return 0; }
float       rulePeakLimitA() { return 0; }
int         ruleTrainScans() { return 0; }
int         ruleTrainPreparations() { return 0; }
#endif

// ------------------------------------------------------------------ one scan
ScanJudgement ruleJudgeScan(const float* e, const float* i, int n) {
  ScanJudgement s;
  s.ok = false;
  s.why = "";
  s.peakA = 0;
  s.p[0] = s.p[1] = s.p[2] = 0;
  s.resid = 0;
  s.novel = false;

  float fwd[N_GRID], ret[N_GRID];
  bool retOk = false;
  if (!sweeps(e, i, n, fwd, ret, &retOk, &s.why)) return s;
  s.peakA = peakHeight(fwd);

#if MODEL_PRESENT
  static_assert(MODEL_N_FEAT == MODEL_N_GRID || MODEL_N_FEAT == 2 * MODEL_N_GRID, "features");
  static_assert(MODEL_N_CLASS == 3, "three classes: blank, propineb, other");
  if (MODEL_N_FEAT == 2 * MODEL_N_GRID && !retOk) {
    s.why = "no return sweep";
    return s;
  }
  float x[MODEL_N_FEAT];
  for (int k = 0; k < MODEL_N_GRID; k++) x[k] = fwd[k];
  if (MODEL_N_FEAT == 2 * MODEL_N_GRID)
    for (int k = 0; k < MODEL_N_GRID; k++) x[MODEL_N_GRID + k] = ret[k];

  // StandardScaler
  double z[MODEL_N_FEAT];
  for (int j = 0; j < MODEL_N_FEAT; j++) z[j] = ((double)x[j] - MODEL_X_MEAN[j]) / MODEL_X_SCALE[j];
  // PCA.transform
  double t[MODEL_N_COMP];
  for (int c = 0; c < MODEL_N_COMP; c++) {
    double acc = 0;
    for (int j = 0; j < MODEL_N_FEAT; j++) acc += (z[j] - MODEL_PCA_MEAN[j]) * MODEL_PCA_COMP[c][j];
    t[c] = acc;
  }
  // distance from the model: norm of z - PCA.inverse_transform(t)
  double ss = 0;
  for (int j = 0; j < MODEL_N_FEAT; j++) {
    double zh = MODEL_PCA_MEAN[j];
    for (int c = 0; c < MODEL_N_COMP; c++) zh += t[c] * MODEL_PCA_COMP[c][j];
    ss += (z[j] - zh) * (z[j] - zh);
  }
  s.resid = (float)sqrt(ss);
  s.novel = s.resid > MODEL_NOVELTY_LIMIT;
  // LogisticRegression.predict_proba: softmax of the class scores
  double score[MODEL_N_CLASS], top = -INFINITY, sum = 0;
  for (int k = 0; k < MODEL_N_CLASS; k++) {
    double acc = MODEL_INTERCEPT[k];
    for (int c = 0; c < MODEL_N_COMP; c++) acc += MODEL_COEF[k][c] * t[c];
    score[k] = acc;
    if (acc > top) top = acc;
  }
  for (int k = 0; k < MODEL_N_CLASS; k++) sum += exp(score[k] - top);
  for (int k = 0; k < MODEL_N_CLASS; k++) s.p[k] = (float)(exp(score[k] - top) / sum);
#endif
  s.ok = true;
  return s;
}

// ------------------------------------------------------------------ one sample
SampleResult ruleCombine(const ScanJudgement* s, int nScans) {
  SampleResult r;
  r.verdict = V_NO_RULE;
  r.p[0] = r.p[1] = r.p[2] = 0;
  r.reason = "";
  r.peakAbove = false;
  if (!MODEL_PRESENT) { r.reason = "no rule loaded"; return r; }
  if (nScans < 1) { r.verdict = V_INDETERMINATE; r.reason = "no scan"; return r; }
  bool novel = false;
  r.peakAbove = true;
  for (int k = 0; k < nScans; k++) {
    if (!s[k].ok) { r.verdict = V_INDETERMINATE; r.reason = s[k].why; r.peakAbove = false; return r; }
    for (int c = 0; c < 3; c++) r.p[c] += s[k].p[c] / nScans;
    novel = novel || s[k].novel;
    r.peakAbove = r.peakAbove && s[k].peakA > rulePeakLimitA();
  }
  int best = 0;
  for (int c = 1; c < 3; c++)
    if (r.p[c] > r.p[best]) best = c;
  r.verdict = (Verdict)best;
  // reasons fit one line of the display (21 characters)
  if (novel) { r.verdict = V_INDETERMINATE; r.reason = "unlike training scans"; }
  else if (r.p[best] < rulePMin()) { r.verdict = V_INDETERMINATE; r.reason = "probability too low"; }
  return r;
}

// ------------------------------------------------------------------ self-test
int ruleSelfTest(void (*report)(const char* line)) {
#if MODEL_PRESENT
  char line[160];
  int fails = 0, off = 0;
  for (int t = 0; t < MODEL_N_TEST; t++) {
    ScanJudgement s = ruleJudgeScan(MODEL_TEST_E + off, MODEL_TEST_I + off, MODEL_TEST_LEN[t]);
    off += MODEL_TEST_LEN[t];
    double dp = 0;
    for (int c = 0; c < 3; c++) dp = fmax(dp, fabs(s.p[c] - MODEL_TEST_P[t][c]));
    double dr = fabs(s.resid - MODEL_TEST_RESID[t]) / fmax(1e-9, fabs(MODEL_TEST_RESID[t]));
    double dk = fabs(s.peakA - MODEL_TEST_PEAK[t]);  // A; tolerance 1e-4 relative + 0.1 nA
    bool pass = s.ok && dp < 1e-4 && dr < 1e-4 && dk < 1e-4 * fabs(MODEL_TEST_PEAK[t]) + 1e-10;
    if (!pass) fails++;
    snprintf(line, sizeof line, "%s %s: max |dp| %.1e, residual rel. %.1e, peak %.1e A",
             pass ? "PASS" : "FAIL", MODEL_TEST_FILE[t], dp, dr, dk);
    report(line);
  }
  return fails;
#else
  (void)report;
  return -1;
#endif
}
