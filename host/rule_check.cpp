// rule_check.cpp -- runs firmware/src/rule.cpp on this computer, for export_model.py.
// stdin:  number of samples; for each sample, number of scans; for each scan, number of points
//         followed by one "E I" pair per point (V, A).
// stdout: one "scan ..." line per scan, one "sample ..." line per sample, then the self-test.
#include <cstdio>
#include <vector>

#include "rule.h"

int main() {
  int nSamples = 0;
  if (scanf("%d", &nSamples) != 1) return 1;
  for (int s = 0; s < nSamples; s++) {
    int nScans = 0;
    if (scanf("%d", &nScans) != 1) return 1;
    std::vector<ScanJudgement> js;
    for (int k = 0; k < nScans; k++) {
      int n = 0;
      if (scanf("%d", &n) != 1) return 1;
      std::vector<float> e(n), i(n);
      for (int q = 0; q < n; q++) {
        double a, b;
        if (scanf("%lf %lf", &a, &b) != 2) return 1;
        e[q] = (float)a;
        i[q] = (float)b;
      }
      ScanJudgement j = ruleJudgeScan(e.data(), i.data(), n);
      js.push_back(j);
      if (j.ok)
        printf("scan ok %.9g %.9g %.9g %.9g %d %.9g\n", j.p[0], j.p[1], j.p[2], j.resid,
               j.novel ? 1 : 0, j.peakA);
      else
        printf("scan fail %s\n", j.why);
    }
    SampleResult r = ruleCombine(js.data(), nScans);
    printf("sample %d %.9g %.9g %.9g %d\n", (int)r.verdict, r.p[0], r.p[1], r.p[2],
           r.peakAbove ? 1 : 0);
  }
  int fails = ruleSelfTest([](const char* line) { printf("selftest %s\n", line); });
  printf("selftest_failures %d\n", fails);
  return 0;
}
