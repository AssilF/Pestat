// rule.h -- the decision rule of the screening test, run on the device.
//
// The same arithmetic as analysis/classify.py, applied to the scans of one sample:
//   features   forward (and return) sweep read at the grid potentials (classify.read_scan)
//   peak       largest rise of the forward sweep above the straight line across the peak
//              window (classify.peak_height), shown as a signal in amperes
//   classifier standard scaling, principal components, logistic regression (classify.fit_predict),
//              with the novelty limit; the scans of a sample are combined as classify.py combines
//              the scans of a preparation (mean probability; P_MIN; any novel scan -> indeterminate)
// The numbers come from model.h, written by firmware/host/export_model.py from the phase-B data set.
// No Arduino dependency: the same file is compiled on the PC to check it against classify.py.
#pragma once
#include <stdint.h>

// The first three in the order of CLASSES in analysis/classify.py.
enum Verdict { V_BLANK = 0, V_PROPINEB = 1, V_OTHER = 2, V_INDETERMINATE = 3, V_NO_RULE = 4 };

struct ScanJudgement {
  bool        ok;      // the scan could be read on the grid
  const char* why;     // reason when !ok
  float       peakA;   // peak signal (A)
  float       p[3];    // class probabilities (blank, propineb, other); model only
  float       resid;   // distance from the model in scaled units; model only
  bool        novel;   // resid above the novelty limit; model only
};

struct SampleResult {
  Verdict     verdict;
  float       p[3];        // mean over the scans; model only
  const char* reason;      // why the verdict is indeterminate or absent; "" otherwise
  bool        peakAbove;   // every scan's peak above the blank limit (rule 1 of classify.py)
};

bool        ruleModelPresent();
const char* ruleModelName();       // "" without a model
uint32_t    ruleModelConfigCrc();  // configuration the model was trained in; 0 without a model
float       rulePMin();
float       rulePeakLimitA();      // blank limit of the peak rule (A); 0 without a model
int         ruleTrainScans();
int         ruleTrainPreparations();

// One scan, as recorded: n points of potential (V) and current (A, oxidation positive).
ScanJudgement ruleJudgeScan(const float* e, const float* i, int n);

// The scans of one sample.
SampleResult ruleCombine(const ScanJudgement* s, int nScans);

// Runs the test scans embedded in model.h and compares with the values classify.py's model gave
// them. Calls report() with one line per test. Returns the number of failures; -1 without a model.
int ruleSelfTest(void (*report)(const char* line));
