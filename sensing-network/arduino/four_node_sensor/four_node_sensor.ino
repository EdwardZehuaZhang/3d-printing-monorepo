/*
 * 4-Node Bidirectional Touch Sensor
 * -----------------------------------
 * Charges the trace from BOTH ends alternately to create a unique
 * 2D signature (fwdB, revA) for each touch node.
 *
 * Why previous approaches failed:
 *   - micros() overhead (~4µs) masked actual RC (~100ns stray C)
 *   - Voltage divider: with 10k external, nodes >10k from start
 *     put Pin A above 2.5V instantly → zero counts, no discrimination
 *   - Charging from one end only → half the nodes are invisible
 *
 * This version:
 *   - Forward: Pin 5 → 10k → trace start → sense Pin B at trace end
 *   - Reverse: Pin 3 drives trace end directly → sense Pin A at trace start
 *     (Pin 5 set to INPUT to disconnect the 10k resistor)
 *   - Tight loop counting (no micros) → ~0.2µs per iteration
 *   - 100 accumulated cycles per direction → stable counts
 *
 * WIRING (same as before, no changes):
 *
 *                 10k ohm resistor
 *   Pin 5 ────[████████]──── Pin 2 ──── TRACE START
 *
 *   Pin 3 ──────────────────────────── TRACE END
 *
 * Trace: START -[3k]- N1 -[10k]- N2 -[6k]- N3 -[8k]- N4 - END
 * Cumulative from start: 3k, 13k, 19k, 27k.  Total: ~28k.
 */

// ── Pin config ───────────────────────────────────────────────────────
#define SEND_PIN    5   // through 10k to trace start
#define PIN_A       2   // trace START
#define PIN_B       3   // trace END

// ── Trace ────────────────────────────────────────────────────────────
#define NUM_NODES 4
const float nodeResKohm[NUM_NODES] = {3.0, 13.0, 19.0, 27.0};
#define EXT_RES_KOHM 20.0  // external resistor on SEND path (Pin5 -> trace start)

// ── Quality heuristics ────────────────────────────────────────────────
#define SIGMA_WARN_TH 2.0
#define SIGMA_GOOD_TH 3.0

// ── Measurement ──────────────────────────────────────────────────────
#define SAMPLES_PER_READ  100   // accumulated cycles per direction
#define PER_CYCLE_LIMIT   10000 // max counts per single charge cycle
#define DISCHARGE_US      20    // µs to discharge (plenty for ~10pF stray)

// ── Calibration ──────────────────────────────────────────────────────
#define BASELINE_READS     30
#define CAL_READS_PER_NODE 40
#define CAL_NODE_DURATION_MS 7000
#define CAL_MIN_VALID_SAMPLES 30
#define CAL_PROGRESS_MS     700
#define DEBOUNCE_COUNT      6
#define RELEASE_COUNT       5

// ── Runtime robustness ──────────────────────────────────────────────
#define CLASS_STD_FLOOR     18
#define CLASS_MAX_SCORE     45.0f
#define PRESS_SCORE_MAX      12.0f
#define N23_AMBIG_MARGIN     1.25f
#define SWITCH_CONFIRM_COUNT 7
#define SWITCH_SCORE_MARGIN  2.5f
#define SWITCH_LOCKOUT_FRAMES 4
#define FILTER_ALPHA         0.18f
#define BASELINE_TRACK_ALPHA 0.02f

// ── Calibration data ─────────────────────────────────────────────────
long   nodeFwdMean[NUM_NODES];     // forward Pin B count mean
long   nodeRevMean[NUM_NODES];     // reverse Pin A count mean
long   nodeFwdStd[NUM_NODES];
long   nodeRevStd[NUM_NODES];
long   nodeFwdMin[NUM_NODES];
long   nodeFwdMax[NUM_NODES];
long   nodeRevMin[NUM_NODES];
long   nodeRevMax[NUM_NODES];
int    sortIdx[NUM_NODES];

long   baselineFwd  = 0;
long   baselineRev  = 0;
long   baselineSum  = 0;
long   touchThreshold = 0;
long   touchThresholdDiff = 0;
long   phase0DirectionalDelta = 0;
bool   calibrated   = false;

// ── Runtime ──────────────────────────────────────────────────────────
int    currentNode   = -1;
int    candidateNode = -1;
int    hitCounter    = 0;
int    missCounter   = 0;
int    switchCandidate = -1;
int    switchCounter   = 0;
int    switchLockoutCounter = 0;
unsigned long lastPrintMs = 0;
float  filtFwd = 0.0f;
float  filtRev = 0.0f;
bool   filterInitialized = false;

// ═════════════════════════════════════════════════════════════════════
// CORE MEASUREMENT
// ═════════════════════════════════════════════════════════════════════

// Bidirectional reading: accumulates counts over `samples` cycles
// for both forward (Pin B sensing) and reverse (Pin A sensing).
void readBidir(unsigned long &fwdB, unsigned long &revA, int samples) {
  fwdB = 0;
  revA = 0;

  for (int s = 0; s < samples; s++) {
    // ── FORWARD: charge from start (through 10k), sense Pin B ────
    // Discharge
    pinMode(SEND_PIN, OUTPUT); digitalWrite(SEND_PIN, LOW);
    pinMode(PIN_A, OUTPUT);    digitalWrite(PIN_A, LOW);
    pinMode(PIN_B, OUTPUT);    digitalWrite(PIN_B, LOW);
    delayMicroseconds(DISCHARGE_US);

    // Release both receive pins to INPUT (float)
    pinMode(PIN_A, INPUT);
    pinMode(PIN_B, INPUT);

    // Charge from start and count Pin B
    volatile unsigned long cyc = 0;
    digitalWrite(SEND_PIN, HIGH);
    while (digitalRead(PIN_B) == LOW) {
      if (++cyc >= PER_CYCLE_LIMIT) break;
    }
    fwdB += cyc;
    digitalWrite(SEND_PIN, LOW);

    // ── REVERSE: charge from end (direct drive Pin 3), sense Pin A ──
    // Discharge
    pinMode(SEND_PIN, INPUT);   // disconnect 10k from circuit
    pinMode(PIN_A, OUTPUT);     digitalWrite(PIN_A, LOW);
    pinMode(PIN_B, OUTPUT);     digitalWrite(PIN_B, LOW);
    delayMicroseconds(DISCHARGE_US);

    // Release Pin A to INPUT (sense pin)
    pinMode(PIN_A, INPUT);
    // Pin 5 stays INPUT (disconnected)

    // Charge from end and count Pin A
    cyc = 0;
    digitalWrite(PIN_B, HIGH);  // direct drive, ~25Ω output
    while (digitalRead(PIN_A) == LOW) {
      if (++cyc >= PER_CYCLE_LIMIT) break;
    }
    revA += cyc;
    digitalWrite(PIN_B, LOW);
  }
}

// Shorthand with default sample count
void readBidir(unsigned long &fwdB, unsigned long &revA) {
  readBidir(fwdB, revA, SAMPLES_PER_READ);
}

// ═════════════════════════════════════════════════════════════════════
// STATS
// ═════════════════════════════════════════════════════════════════════
void statsN(int n, long &fMean, long &rMean, long &fStd, long &rStd) {
  long fs[60], rs[60];
  if (n > 60) n = 60;
  long fSum = 0, rSum = 0;
  for (int i = 0; i < n; i++) {
    unsigned long f, r;
    readBidir(f, r);
    fs[i] = (long)f;
    rs[i] = (long)r;
    fSum += fs[i];
    rSum += rs[i];
  }
  fMean = fSum / n;
  rMean = rSum / n;
  long fSqSum = 0, rSqSum = 0;
  for (int i = 0; i < n; i++) {
    long df = fs[i] - fMean; fSqSum += df * df;
    long dr = rs[i] - rMean; rSqSum += dr * dr;
  }
  fStd = (long)sqrt((double)fSqSum / n);
  rStd = (long)sqrt((double)rSqSum / n);
}

// ═════════════════════════════════════════════════════════════════════
// HELPERS
// ═════════════════════════════════════════════════════════════════════
String readSerialLine() {
  while (true) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() > 0) return s;
    }
  }
}

void pLong(long v, int w) {
  String s = String(v);
  for (int i = s.length(); i < w; i++) Serial.print(' ');
  Serial.print(s);
}

long absLong(long v) {
  return v >= 0 ? v : -v;
}

long signalDiffFromBaseline(long fwd, long rev) {
  return absLong(fwd - baselineFwd) + absLong(rev - baselineRev);
}

// Euclidean distance squared in 2D (fwd, rev) space
long dist2(long fwd, long rev, long fMean, long rMean) {
  long df = fwd - fMean;
  long dr = rev - rMean;
  return df * df + dr * dr;
}

float nodeScore(int idx, long fwd, long rev) {
  float fStd = (float)nodeFwdStd[idx];
  float rStd = (float)nodeRevStd[idx];
  if (fStd < CLASS_STD_FLOOR) fStd = CLASS_STD_FLOOR;
  if (rStd < CLASS_STD_FLOOR) rStd = CLASS_STD_FLOOR;

  float df = ((float)fwd - (float)nodeFwdMean[idx]) / fStd;
  float dr = ((float)rev - (float)nodeRevMean[idx]) / rStd;
  float score = df * df + dr * dr;

  const float slack = 20.0f;
  if ((float)fwd < ((float)nodeFwdMin[idx] - slack) ||
      (float)fwd > ((float)nodeFwdMax[idx] + slack) ||
      (float)rev < ((float)nodeRevMin[idx] - slack) ||
      (float)rev > ((float)nodeRevMax[idx] + slack)) {
    score += 0.75f;
  }

  return score;
}

void updateBaselineIfIdle(long fwd, long rev, long sum, long diff) {
  long sumGate = (touchThreshold * 3L) / 5L;
  long diffGate = (touchThresholdDiff * 3L) / 5L;
  if (sum < sumGate && diff < diffGate) {
    baselineFwd = (long)((1.0f - BASELINE_TRACK_ALPHA) * baselineFwd + BASELINE_TRACK_ALPHA * (float)fwd);
    baselineRev = (long)((1.0f - BASELINE_TRACK_ALPHA) * baselineRev + BASELINE_TRACK_ALPHA * (float)rev);
    baselineSum = baselineFwd + baselineRev;
  }
}

void calibrateNodeLongWindow(int nodeIdx,
                             long &fMean, long &rMean,
                             long &fStd, long &rStd,
                             long &fMin, long &fMax,
                             long &rMin, long &rMax) {
  double fMeanAcc = 0.0;
  double rMeanAcc = 0.0;
  double fM2 = 0.0;
  double rM2 = 0.0;
  int n = 0;

  fMin = 2147483647L;
  rMin = 2147483647L;
  fMax = -2147483647L;
  rMax = -2147483647L;

  unsigned long startMs = millis();
  unsigned long lastProgress = 0;

  while (millis() - startMs < CAL_NODE_DURATION_MS) {
    unsigned long fRaw, rRaw;
    readBidir(fRaw, rRaw);

    long f = (long)fRaw;
    long r = (long)rRaw;
    long s = f + r;
    long d = signalDiffFromBaseline(f, r);

    if (s > touchThreshold || d > touchThresholdDiff) {
      n++;

      double df = (double)f - fMeanAcc;
      fMeanAcc += df / (double)n;
      fM2 += df * ((double)f - fMeanAcc);

      double dr = (double)r - rMeanAcc;
      rMeanAcc += dr / (double)n;
      rM2 += dr * ((double)r - rMeanAcc);

      if (f < fMin) fMin = f;
      if (f > fMax) fMax = f;
      if (r < rMin) rMin = r;
      if (r > rMax) rMax = r;
    }

    if (millis() - lastProgress >= CAL_PROGRESS_MS) {
      lastProgress = millis();
      Serial.print(F("    collecting Node "));
      Serial.print(nodeIdx + 1);
      Serial.print(F(": fwd=")); pLong(f, 6);
      Serial.print(F(" rev=")); pLong(r, 6);
      Serial.print(F(" sum=")); pLong(s, 6);
      Serial.print(F(" diff=")); pLong(d, 6);
      Serial.print(F(" valid=")); Serial.println(n);
    }
  }

  if (n < CAL_MIN_VALID_SAMPLES) {
    long sfm, srm, sfs, srs;
    statsN(CAL_READS_PER_NODE, sfm, srm, sfs, srs);
    fMean = sfm;
    rMean = srm;
    fStd = sfs;
    rStd = srs;
    fMin = sfm - max(2L * sfs, 15L);
    fMax = sfm + max(2L * sfs, 15L);
    rMin = srm - max(2L * srs, 15L);
    rMax = srm + max(2L * srs, 15L);
    return;
  }

  fMean = (long)(fMeanAcc + 0.5);
  rMean = (long)(rMeanAcc + 0.5);
  fStd = (long)(sqrt(fM2 / (double)n) + 0.5);
  rStd = (long)(sqrt(rM2 / (double)n) + 0.5);

  if (fStd < 1) fStd = 1;
  if (rStd < 1) rStd = 1;
}

// ═════════════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println();
  Serial.println(F("====================================================="));
  Serial.println(F("  4-Node Bidirectional Touch Sensor"));
  Serial.println(F("====================================================="));
  Serial.println();
  Serial.println(F("  Trace: START -[3k]- N1 -[10k]- N2 -[6k]- N3 -[8k]- N4 - END"));
  Serial.println();
  Serial.println(F("  WIRING:"));
  Serial.print(F("               ")); Serial.print(EXT_RES_KOHM, 0); Serial.println(F("k resistor"));
  Serial.println(F("    Pin 5 ───[████████]─── Pin 2 ── TRACE START"));
  Serial.println(F("    Pin 3 ─────────────────────── TRACE END"));
  Serial.println();
  Serial.print(F("  Forward: Pin5→")); Serial.print(EXT_RES_KOHM, 0); Serial.println(F("k→start→trace→Pin3(sense)"));
  Serial.println(F("  Reverse: Pin3(drive)→trace→Pin2(sense), Pin5=INPUT"));
  Serial.println();

  // ── PHASE 0: Signal test ──────────────────────────────────────
  Serial.println(F("[PHASE 0] Signal test"));
  Serial.println(F("  DON'T TOUCH for 4 seconds..."));
  Serial.println();

  unsigned long noTouchFwdMax = 0, noTouchRevMax = 0;
  unsigned long noTouchFwdAcc = 0, noTouchRevAcc = 0;
  int ntCount = 0;
  unsigned long phaseEnd = millis() + 4000;
  while (millis() < phaseEnd) {
    unsigned long f, r;
    readBidir(f, r);
    noTouchFwdAcc += f;
    noTouchRevAcc += r;
    if (f > noTouchFwdMax) noTouchFwdMax = f;
    if (r > noTouchRevMax) noTouchRevMax = r;
    ntCount++;

    Serial.print(F("  [no-touch] fwd="));
    pLong((long)f, 6);
    Serial.print(F("  rev="));
    pLong((long)r, 6);
    Serial.print(F("  sum="));
    Serial.println((long)(f + r));
  }

  long ntFwdAvg = ntCount > 0 ? noTouchFwdAcc / ntCount : 0;
  long ntRevAvg = ntCount > 0 ? noTouchRevAcc / ntCount : 0;
  long ntSumAvg = ntFwdAvg + ntRevAvg;

  Serial.println();
  Serial.println(F("  >>> TOUCH ANY NODE FIRMLY and hold for 4 seconds <<<"));
  Serial.println();

  unsigned long touchFwdAcc = 0, touchRevAcc = 0;
  int tCount = 0;
  phaseEnd = millis() + 4000;
  while (millis() < phaseEnd) {
    unsigned long f, r;
    readBidir(f, r);
    touchFwdAcc += f;
    touchRevAcc += r;
    tCount++;

    Serial.print(F("  [touch]    fwd="));
    pLong((long)f, 6);
    Serial.print(F("  rev="));
    pLong((long)r, 6);
    Serial.print(F("  sum="));
    Serial.println((long)(f + r));
  }

  long tFwdAvg = tCount > 0 ? touchFwdAcc / tCount : 0;
  long tRevAvg = tCount > 0 ? touchRevAcc / tCount : 0;
  long tSumAvg = tFwdAvg + tRevAvg;
  long delta = tSumAvg - ntSumAvg;
  long directionalDelta = absLong(tFwdAvg - ntFwdAvg) + absLong(tRevAvg - ntRevAvg);
  phase0DirectionalDelta = directionalDelta;

  Serial.println();
  Serial.print(F("  No-touch avg: fwd=")); Serial.print(ntFwdAvg);
  Serial.print(F("  rev=")); Serial.print(ntRevAvg);
  Serial.print(F("  sum=")); Serial.println(ntSumAvg);
  Serial.print(F("  Touch avg:    fwd=")); Serial.print(tFwdAvg);
  Serial.print(F("  rev=")); Serial.print(tRevAvg);
  Serial.print(F("  sum=")); Serial.println(tSumAvg);
  Serial.print(F("  Delta: +")); Serial.println(delta);
  Serial.print(F("  Directional delta: +")); Serial.println(directionalDelta);
  Serial.println();

  if (delta < 50 && directionalDelta < 40) {
    Serial.println(F("  *** WARNING: weak signal (need delta > 50 OR directional > 40) ***"));
    Serial.println(F("  Check wiring and that you're touching conductive material."));
  } else {
    Serial.print(F("  Signal OK! Delta = +"));
    Serial.print(delta);
    Serial.print(F(", Directional = +"));
    Serial.println(directionalDelta);
  }

  Serial.println();
  Serial.println(F("  Press ENTER to continue..."));
  readSerialLine();

  // ── PHASE 1: Baseline ──────────────────────────────────────────
  Serial.println(F("[PHASE 1] Baseline measurement"));
  Serial.println(F("  HANDS OFF completely. Measuring in 3 seconds..."));
  delay(3000);

  long bfm, brm, bfs, brs;
  statsN(BASELINE_READS, bfm, brm, bfs, brs);
  baselineFwd = bfm;
  baselineRev = brm;
  baselineSum = bfm + brm;

  // Adaptive threshold using Phase 0 touch data
  touchThreshold = baselineSum + max(delta / 3, 30L);
  touchThresholdDiff = max((bfs + brs) * 3, max(phase0DirectionalDelta / 3, 25L));

  Serial.print(F("  Baseline: fwd=")); Serial.print(baselineFwd);
  Serial.print(F("  rev=")); Serial.print(baselineRev);
  Serial.print(F("  sum=")); Serial.println(baselineSum);
  Serial.print(F("  Baseline std: fwd=")); Serial.print(bfs);
  Serial.print(F("  rev=")); Serial.println(brs);
  Serial.print(F("  Threshold: sum > ")); Serial.println(touchThreshold);
  Serial.print(F("  Threshold: diff > ")); Serial.println(touchThresholdDiff);
  Serial.println();

  // ── PHASE 2: Per-node calibration ──────────────────────────────
  Serial.println(F("[PHASE 2] Per-node calibration"));
  Serial.println(F("  Touch each node when prompted."));
  Serial.println(F("  Keep contact for 7s and vary from light to firm pressure."));
  Serial.println(F("  fwdB = charge from start, sense end"));
  Serial.println(F("  revA = charge from end, sense start"));
  Serial.println();

  for (int i = 0; i < NUM_NODES; i++) {
    Serial.println(F("  ────────────────────────────────────────────────"));
    Serial.print(F("  >>> TOUCH AND HOLD  Node "));
    Serial.print(i + 1);
    Serial.print(F("  ("));
    Serial.print(nodeResKohm[i], 0);
    Serial.println(F("k from START) <<<"));

    // Wait for touch
    int waitCount = 0;
    while (true) {
      unsigned long f, r;
      readBidir(f, r);
      long s = (long)(f + r);
      long diff = signalDiffFromBaseline((long)f, (long)r);
      waitCount++;
      if (waitCount % 3 == 0) {
        Serial.print(F("    fwd="));  pLong((long)f, 6);
        Serial.print(F("  rev="));    pLong((long)r, 6);
        Serial.print(F("  sum="));    pLong(s, 6);
        Serial.print(F("  diff="));   pLong(diff, 6);
        Serial.print(F("  (need sum > ")); Serial.print(touchThreshold);
        Serial.print(F(" or diff > ")); Serial.print(touchThresholdDiff); Serial.println(F(")"));
      }
      if (s > touchThreshold || diff > touchThresholdDiff) break;
    }

    Serial.println(F("  Touch detected! Keep touching and vary pressure..."));
    delay(250);

    long fm, rm, fs, rs, fmin, fmax, rmin, rmax;
    calibrateNodeLongWindow(i, fm, rm, fs, rs, fmin, fmax, rmin, rmax);
    nodeFwdMean[i] = fm;
    nodeRevMean[i] = rm;
    nodeFwdStd[i]  = fs;
    nodeRevStd[i]  = rs;
    nodeFwdMin[i]  = fmin;
    nodeFwdMax[i]  = fmax;
    nodeRevMin[i]  = rmin;
    nodeRevMax[i]  = rmax;

    Serial.print(F("  Node ")); Serial.print(i + 1);
    Serial.print(F(": fwd=")); Serial.print(fm);
    Serial.print(F("±")); Serial.print(fs);
    Serial.print(F("  rev=")); Serial.print(rm);
    Serial.print(F("±")); Serial.println(rs);
    Serial.print(F("          range fwd[")); Serial.print(fmin);
    Serial.print(F(", ")); Serial.print(fmax);
    Serial.print(F("] rev[")); Serial.print(rmin);
    Serial.print(F(", ")); Serial.print(rmax);
    Serial.println(F("]"));

    Serial.println(F("  >>> RELEASE NOW <<<"));
    delay(800);
    int relCount = 0;
    while (relCount < 8) {
      unsigned long f, r;
      readBidir(f, r);
      long s = (long)(f + r);
      long diff = signalDiffFromBaseline((long)f, (long)r);
      if (s < touchThreshold && diff < touchThresholdDiff) relCount++;
      else relCount = 0;
    }
    Serial.println(F("  Released."));
    Serial.println();
  }

  // ── PHASE 3: Report ────────────────────────────────────────────
  Serial.println(F("[PHASE 3] Calibration results"));
  Serial.println();
  Serial.println(F("  Node | R(k) | fwdB       | revA       "));
  Serial.println(F("  ─────|──────|────────────|────────────"));

  for (int i = 0; i < NUM_NODES; i++) {
    Serial.print(F("   ")); Serial.print(i + 1);
    Serial.print(F("   | "));
    Serial.print(nodeResKohm[i], 0);
    Serial.print(F("   | "));
    pLong(nodeFwdMean[i], 6); Serial.print(F(" ±"));
    pLong(nodeFwdStd[i], 3);
    Serial.print(F(" | "));
    pLong(nodeRevMean[i], 6); Serial.print(F(" ±"));
    pLong(nodeRevStd[i], 3);
    Serial.println();
  }

  // 2D distance between all pairs
  Serial.println();
  Serial.println(F("  2D distances between nodes:"));
  bool allGood = true;
  float minSigmaAll = 9999.0f;
  int minSigmaNodeA = -1;
  int minSigmaNodeB = -1;
  float minSigmaAdj = 9999.0f;
  int minAdjIdx = -1;

  for (int i = 0; i < NUM_NODES; i++) {
    for (int j = i + 1; j < NUM_NODES; j++) {
      long d = (long)sqrt((double)dist2(nodeFwdMean[i], nodeRevMean[i],
                                         nodeFwdMean[j], nodeRevMean[j]));
      Serial.print(F("    N")); Serial.print(i + 1);
      Serial.print(F(" ↔ N")); Serial.print(j + 1);
      Serial.print(F(": "));   Serial.print(d);
      long maxStd = max(max(nodeFwdStd[i], nodeRevStd[i]),
                        max(nodeFwdStd[j], nodeRevStd[j]));
      float sigma = 9999.0f;
      if (maxStd > 0) {
        sigma = (float)d / (float)maxStd;
        Serial.print(F("  (")); Serial.print(sigma, 1); Serial.print(F("σ)"));
        if (sigma < minSigmaAll) {
          minSigmaAll = sigma;
          minSigmaNodeA = i;
          minSigmaNodeB = j;
        }
        if ((j == i + 1) && (sigma < minSigmaAdj)) {
          minSigmaAdj = sigma;
          minAdjIdx = i;
        }
        if (sigma < SIGMA_WARN_TH) { Serial.print(F(" *** OVERLAP ***")); allGood = false; }
      }
      Serial.println();
    }
  }

  Serial.println();
  if (allGood) {
    Serial.println(F("  >> All nodes well-separated! Calibration GOOD."));
  } else {
    Serial.println(F("  >> WARNING: some nodes may overlap."));
    Serial.println(F("     Touch each node at dead center during calibration."));
  }

  // ── Practical tuning guidance ─────────────────────────────────────
  float totalPathK = nodeResKohm[NUM_NODES - 1];
  float extRatio = totalPathK > 0.0f ? (EXT_RES_KOHM / totalPathK) : 0.0f;

  Serial.println();
  Serial.println(F("  TUNING GUIDE"));
  Serial.println(F("  ------------"));
  Serial.print(F("  Total path: ~")); Serial.print(totalPathK, 1); Serial.println(F("k"));
  Serial.print(F("  External resistor: ")); Serial.print(EXT_RES_KOHM, 1); Serial.println(F("k"));
  Serial.print(F("  ext/total ratio: ")); Serial.print(extRatio, 2);
  Serial.println(F("  (recommended ~0.3 to 1.0 for low-path traces)"));

  Serial.print(F("  Phase-0 signal delta: +")); Serial.println(delta);
  Serial.print(F("  Phase-0 directional delta: +")); Serial.println(phase0DirectionalDelta);
  Serial.print(F("  Runtime diff threshold: ")); Serial.println(touchThresholdDiff);
  if (delta < 50 && phase0DirectionalDelta < 40) {
    Serial.println(F("  -> Signal is weak in both metrics. Improve contact and consider smaller external R."));
  } else if (delta < 120 || phase0DirectionalDelta < 100) {
    Serial.println(F("  -> Signal is usable but moderate; calibration quality matters."));
  } else {
    Serial.println(F("  -> Signal amplitude is strong (sum and directional)."));
  }

  if (extRatio > 1.0f) {
    Serial.println(F("  -> External R likely too large for this trace; node values may compress."));
  } else if (extRatio < 0.3f) {
    Serial.println(F("  -> External R is low; fast response, but verify noise/consistency."));
  } else {
    Serial.println(F("  -> External R is in a good operating window for this trace."));
  }

  if (minSigmaNodeA >= 0) {
    Serial.print(F("  Worst pair: N")); Serial.print(minSigmaNodeA + 1);
    Serial.print(F(" ↔ N")); Serial.print(minSigmaNodeB + 1);
    Serial.print(F(" at ")); Serial.print(minSigmaAll, 2); Serial.println(F("σ"));
  }

  if (!allGood && minAdjIdx >= 0) {
    float oldGap = nodeResKohm[minAdjIdx + 1] - nodeResKohm[minAdjIdx];
    float targetGap = oldGap * 1.30f;
    float suggestedCum = nodeResKohm[minAdjIdx] + targetGap;

    Serial.println(F("  -> First adjustment recommendation:"));
    Serial.print(F("     Increase segment between N")); Serial.print(minAdjIdx + 1);
    Serial.print(F(" and N")); Serial.print(minAdjIdx + 2); Serial.println(F("."));
    Serial.print(F("     Current gap: ")); Serial.print(oldGap, 1); Serial.println(F("k"));
    Serial.print(F("     Try gap: ~")); Serial.print(targetGap, 1); Serial.println(F("k"));
    Serial.print(F("     That means cumulative N")); Serial.print(minAdjIdx + 2);
    Serial.print(F(" from ")); Serial.print(nodeResKohm[minAdjIdx + 1], 1);
    Serial.print(F("k -> ~")); Serial.print(suggestedCum, 1); Serial.println(F("k"));
    Serial.println(F("     Recalibrate after this single change before further edits."));
  } else if (allGood) {
    Serial.println(F("  -> Keep current resistance map. No increase needed now."));
    Serial.println(F("  -> UNO R4 is suitable; no need to switch to UNO R3 for stability."));
  }

  if ((minSigmaAdj < SIGMA_WARN_TH) && (minAdjIdx >= 0)) {
    Serial.print(F("  Adjacent bottleneck: N")); Serial.print(minAdjIdx + 1);
    Serial.print(F(" ↔ N")); Serial.print(minAdjIdx + 2);
    Serial.print(F(" at ")); Serial.print(minSigmaAdj, 2); Serial.println(F("σ"));
  }

  calibrated = true;
  Serial.println();
  Serial.println(F("  ═════════════════════════════════════════════════════════════"));
  Serial.println(F("  LIVE DETECTION"));
  Serial.println(F("  ═════════════════════════════════════════════════════════════"));
  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════
// LOOP
// ═════════════════════════════════════════════════════════════════════
void loop() {
  if (!calibrated) return;

  unsigned long fwdRaw, revRaw;
  readBidir(fwdRaw, revRaw);

  if (!filterInitialized) {
    filtFwd = (float)fwdRaw;
    filtRev = (float)revRaw;
    filterInitialized = true;
  } else {
    filtFwd = (1.0f - FILTER_ALPHA) * filtFwd + FILTER_ALPHA * (float)fwdRaw;
    filtRev = (1.0f - FILTER_ALPHA) * filtRev + FILTER_ALPHA * (float)revRaw;
  }

  long fwd = (long)(filtFwd + 0.5f);
  long rev = (long)(filtRev + 0.5f);
  long sum = fwd + rev;
  long diff = signalDiffFromBaseline(fwd, rev);

  updateBaselineIfIdle(fwd, rev, sum, diff);
  diff = signalDiffFromBaseline(fwd, rev);

  // ── Node identification by nearest centroid ─────────────────────
  int detectedNode = -1;
  int secondNode = -1;
  float bestScore = 1.0e9f;
  float secondScore = 1.0e9f;
  if (sum > touchThreshold || diff > touchThresholdDiff) {
    for (int i = 0; i < NUM_NODES; i++) {
      float score = nodeScore(i, fwd, rev);
      if (score < bestScore) {
        secondScore = bestScore;
        secondNode = detectedNode;
        bestScore = score;
        detectedNode = i;
      } else if (score < secondScore) {
        secondScore = score;
        secondNode = i;
      }
    }

    if (bestScore > CLASS_MAX_SCORE) {
      detectedNode = -1;
    }
  }

  float scoreGap = secondScore - bestScore;
  bool n23Boundary = ((detectedNode == 1 && secondNode == 2) || (detectedNode == 2 && secondNode == 1));
  bool n23Ambiguous = n23Boundary && (scoreGap < N23_AMBIG_MARGIN);

  // ── Debounce ──────────────────────────────────────────────────
  if (currentNode == -1) {
    if (detectedNode != -1 && bestScore <= PRESS_SCORE_MAX && !n23Ambiguous) {
      if (detectedNode == candidateNode) hitCounter++;
      else { candidateNode = detectedNode; hitCounter = 1; }
      if (hitCounter >= DEBOUNCE_COUNT) {
        currentNode = candidateNode;
        Serial.println();
        Serial.print(F(">>> Node ")); Serial.print(currentNode + 1);
        Serial.println(F(" PRESSED <<<"));
        hitCounter = 0; missCounter = 0; switchCandidate = -1; switchCounter = 0;
        switchLockoutCounter = SWITCH_LOCKOUT_FRAMES;
      }
    } else {
      hitCounter = 0; candidateNode = -1;
    }
  } else {
    if (detectedNode == -1) {
      missCounter++;
      if (missCounter >= RELEASE_COUNT) {
        Serial.print(F("[RELEASE] Node ")); Serial.println(currentNode + 1);
        Serial.println();
        currentNode = -1; candidateNode = -1;
        hitCounter = 0; missCounter = 0;
        switchCandidate = -1; switchCounter = 0;
        switchLockoutCounter = 0;
      }
    } else if (detectedNode == currentNode) {
      missCounter = 0;
      switchCandidate = -1;
      switchCounter = 0;
      if (switchLockoutCounter > 0) switchLockoutCounter--;
    } else {
      missCounter = 0;
      if (switchLockoutCounter > 0 || n23Ambiguous) {
        if (switchLockoutCounter > 0) switchLockoutCounter--;
        switchCandidate = -1;
        switchCounter = 0;
      } else {
        float currentScore = nodeScore(currentNode, fwd, rev);
        bool challengerIsClearlyBetter = (bestScore + SWITCH_SCORE_MARGIN < currentScore);

        if (challengerIsClearlyBetter) {
          if (detectedNode == switchCandidate) switchCounter++;
          else {
            switchCandidate = detectedNode;
            switchCounter = 1;
          }

          if (switchCounter >= SWITCH_CONFIRM_COUNT) {
            Serial.print(F("[SWITCH → Node ")); Serial.print(detectedNode + 1);
            Serial.println(F("]"));
            currentNode = detectedNode;
            switchCandidate = -1;
            switchCounter = 0;
            switchLockoutCounter = SWITCH_LOCKOUT_FRAMES;
          }
        } else {
          switchCandidate = -1;
          switchCounter = 0;
        }
      }
    }
  }

  // ── Debug output ────────────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    Serial.print(F("fwd=")); pLong(fwd, 6);
    Serial.print(F(" rev=")); pLong(rev, 6);
    Serial.print(F(" sum=")); pLong(sum, 7);
    Serial.print(F(" diff=")); pLong(diff, 7);

    if (currentNode >= 0) {
      Serial.print(F(" [N")); Serial.print(currentNode + 1); Serial.print(F("]"));
    } else if (sum > touchThreshold || diff > touchThresholdDiff) {
      Serial.print(F(" [?]"));
    } else {
      Serial.print(F(" [-]"));
    }

    // Show distance to each node centroid
    if (sum > touchThreshold / 2 || diff > touchThresholdDiff / 2) {
      Serial.print(F(" d:"));
      for (int i = 0; i < NUM_NODES; i++) {
        if (i > 0) Serial.print(',');
        Serial.print((long)sqrt((double)dist2(fwd, rev,
                                              nodeFwdMean[i], nodeRevMean[i])));
      }
      Serial.print(F(" s:"));
      Serial.print(bestScore, 2);
      Serial.print(',');
      if (secondScore < 1.0e8f) Serial.print(secondScore, 2);
      else Serial.print(F("-"));
    }
    Serial.println();
  }
}
