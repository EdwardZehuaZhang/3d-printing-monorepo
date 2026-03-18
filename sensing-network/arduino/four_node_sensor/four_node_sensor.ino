/*
 * 4-Node Dual-Ended Touch Sensor — micros() Edition
 * ---------------------------------------------------
 * Uses the RATIO of charge times from both trace ends.
 * No library needed. Uses micros() for CPU-speed-independent timing.
 *
 * Why previous attempts failed:
 *   - 10k resistor: RC in nanoseconds, pin charges before one loop runs (count = 1-2)
 *   - Single-ended 1M/2M: trace (34k) is only 3% of 1M → all nodes look the same
 *   - Dual-ended but wrong wiring: Pin 3 was at same junction as Pin 2, so both
 *     pins saw identical voltage and ratio was always 1000
 *
 * This version fixes all three: correct wiring + 1M external + micros() timing.
 *
 * WIRING:
 *
 *                    1M ohm resistor (brown-black-green)
 *   Pin 5 (SEND) ───[████████████]─── Pin 2 ─── wire to TRACE START
 *                                      ↑
 *                             NO connection to Pin 3 here!
 *
 *   Pin 3 ──────────────────────────────────── wire to TRACE END only
 *
 *   (Charge flows: Pin5 → 1MΩ → Pin2 → through 34kΩ trace → Pin3)
 *
 * Your trace:
 *   START -[4k]- N1 -[11k]- N2 -[7k]- N3 -[10k]- N4 - END
 *   Cumulative from start: 4k, 15k, 22k, 32k ohm. Total: ~34k.
 */

// ── Pin config ───────────────────────────────────────────────────────
#define SEND_PIN    5
#define RCV_A_PIN   2   // START of trace (Node 1 side)
#define RCV_B_PIN   3   // END of trace   (Node 4 side)

// ── Trace ────────────────────────────────────────────────────────────
#define NUM_NODES 4
const float nodeResKohm[NUM_NODES] = {4.0, 15.0, 22.0, 32.0}; // cumulative from start

// ── Measurement parameters ───────────────────────────────────────────
#define CYCLES          60      // measurements per pin per reading
#define TRIM_EACH_SIDE   6      // discard 6 lowest + 6 highest per pin (10%)
#define KEPT_CYCLES     48      // CYCLES - 2*TRIM_EACH_SIDE
#define CYCLE_TIMEOUT  5000UL   // µs — open circuit / wiring error sentinel
#define DISCHARGE_US    200     // µs — fully discharges even 200pF body C through trace

// ── Calibration ──────────────────────────────────────────────────────
#define BASELINE_READS     50
#define CAL_READS_PER_NODE 40
#define DEBOUNCE_COUNT      3
#define RELEASE_COUNT       4

// ── Calibration data ─────────────────────────────────────────────────
long   nodeRatioMean[NUM_NODES];
long   nodeRatioStd[NUM_NODES];
long   nodeRatioLow[NUM_NODES];   // detection boundary low
long   nodeRatioHigh[NUM_NODES];  // detection boundary high
long   nodeSumMean[NUM_NODES];
int    sortIdx[NUM_NODES];

long   baselineSum         = 0;
long   touchThreshold      = 0;
unsigned long phase0TouchAvg = 0;  // saved from Phase 0 for adaptive threshold
bool   calibrated          = false;

// ── Runtime ──────────────────────────────────────────────────────────
int    currentNode   = -1;
int    candidateNode = -1;
int    hitCounter    = 0;
int    missCounter   = 0;
unsigned long lastPrintMs = 0;

// ═════════════════════════════════════════════════════════════════════
// CORE MEASUREMENT
// ═════════════════════════════════════════════════════════════════════

// Measures the charge time for ONE pin in ONE cycle.
// Both receive pins are discharged to LOW before each measurement,
// then both released to INPUT (floating) so neither loads the other.
// Returns elapsed µs, or CYCLE_TIMEOUT if pin never went HIGH.
unsigned long measureOneCycle(int receivePin) {
  // 1. Discharge: clamp both receive pins and send pin to GND
  pinMode(SEND_PIN,  OUTPUT); digitalWrite(SEND_PIN,  LOW);
  pinMode(RCV_A_PIN, OUTPUT); digitalWrite(RCV_A_PIN, LOW);
  pinMode(RCV_B_PIN, OUTPUT); digitalWrite(RCV_B_PIN, LOW);
  delayMicroseconds(DISCHARGE_US);

  // 2. Release BOTH pins to INPUT (floating) before charging.
  //    Keeping the non-measured pin as OUTPUT LOW would drain current
  //    from the trace during the charge pulse, distorting the timing.
  pinMode(RCV_A_PIN, INPUT);
  pinMode(RCV_B_PIN, INPUT);

  // 3. Start charge pulse and timer simultaneously
  unsigned long t0 = micros();
  digitalWrite(SEND_PIN, HIGH);

  // 4. Wait for the target pin to cross the HIGH threshold (~2.5V)
  while (digitalRead(receivePin) == LOW) {
    if ((micros() - t0) >= CYCLE_TIMEOUT) {
      digitalWrite(SEND_PIN, LOW);
      return CYCLE_TIMEOUT;  // wiring error or open circuit
    }
  }

  unsigned long elapsed = micros() - t0;
  digitalWrite(SEND_PIN, LOW);
  return elapsed;
}

// Collects CYCLES measurements for one pin, sorts them, discards
// top and bottom TRIM_EACH_SIDE outliers, returns sum of the rest.
// Returns 0xFFFFFFFF if too many timeouts (wiring problem).
unsigned long collectTrimmedSum(int receivePin) {
  unsigned long samples[CYCLES];
  int timeouts = 0;

  for (int i = 0; i < CYCLES; i++) {
    samples[i] = measureOneCycle(receivePin);
    if (samples[i] == CYCLE_TIMEOUT) timeouts++;
  }

  if (timeouts > CYCLES / 5) return 0xFFFFFFFFUL;  // >20% timeouts = error

  // Insertion sort (CYCLES=60, fast enough on Arduino)
  for (int i = 1; i < CYCLES; i++) {
    unsigned long key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  // Sum the middle KEPT_CYCLES values
  unsigned long sum = 0;
  for (int i = TRIM_EACH_SIDE; i < CYCLES - TRIM_EACH_SIDE; i++) {
    sum += samples[i];
  }
  return sum;
}

// Full dual reading: returns trimmed sum for both pins (in µs)
bool readDual(unsigned long &totalA, unsigned long &totalB) {
  totalA = collectTrimmedSum(RCV_A_PIN);
  totalB = collectTrimmedSum(RCV_B_PIN);
  if (totalA == 0xFFFFFFFFUL || totalB == 0xFFFFFFFFUL) return false;
  return true;
}

// Ratio = (totalA * 1000) / totalB  — integer fixed-point
long computeRatio(unsigned long a, unsigned long b) {
  if (b == 0) return 1000;
  return (long)((a * 1000UL) / b);
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

// Stats over N dual readings: fills outRatioMean, outRatioStd, outSumMean
bool statsN(int n, long &outRatioMean, long &outRatioStd, long &outSumMean) {
  long ratios[100];
  long sums[100];
  if (n > 100) n = 100;

  long ratioSum = 0;
  long sumSum = 0;
  for (int i = 0; i < n; i++) {
    unsigned long a, b;
    if (!readDual(a, b)) { ratios[i] = 1000; sums[i] = 0; continue; }
    ratios[i] = computeRatio(a, b);
    sums[i]   = (long)(a + b);
    ratioSum += ratios[i];
    sumSum   += sums[i];
  }

  outRatioMean = ratioSum / n;
  outSumMean   = sumSum   / n;

  long sqSum = 0;
  for (int i = 0; i < n; i++) {
    long d = ratios[i] - outRatioMean;
    sqSum += d * d;
  }
  outRatioStd = (long)sqrt((double)sqSum / (double)n);
  return true;
}

void pLong(long v, int w) {
  String s = String(v);
  for (int i = s.length(); i < w; i++) Serial.print(' ');
  Serial.print(s);
}

// ═════════════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println();
  Serial.println(F("====================================================="));
  Serial.println(F("  4-Node Dual-Ended Touch Sensor  (micros edition)"));
  Serial.println(F("====================================================="));
  Serial.println();
  Serial.println(F("  Trace: START -[4k]- N1 -[11k]- N2 -[7k]- N3 -[10k]- N4 - END"));
  Serial.println();
  Serial.println(F("  WIRING:"));
  Serial.println(F("                1M ohm resistor"));
  Serial.println(F("    Pin 5 ────[████████████]──── Pin 2 ── TRACE START"));
  Serial.println(F("                                  ↑ no Pin 3 here!"));
  Serial.println(F("    Pin 3 ────────────────────────────── TRACE END"));
  Serial.println();
  Serial.println(F("  Charge flows: Pin5 → 1MΩ → Pin2 → trace → Pin3"));
  Serial.println();

  // ── PHASE 0: Hardware verification ──────────────────────────────
  Serial.println(F("[PHASE 0] Hardware verification"));
  Serial.println(F("  DON'T TOUCH for 4 seconds..."));
  Serial.println();

  // No-touch measurements with live readout
  unsigned long noTouchSumMin = 0xFFFFFFFFUL;
  unsigned long noTouchSumMax = 0;
  unsigned long noTouchSumAcc = 0;
  int noTouchCount = 0;
  unsigned long phaseEnd = millis() + 4000;

  while (millis() < phaseEnd) {
    unsigned long a, b;
    if (readDual(a, b)) {
      unsigned long s = a + b;
      if (s < noTouchSumMin) noTouchSumMin = s;
      if (s > noTouchSumMax) noTouchSumMax = s;
      noTouchSumAcc += s;
      noTouchCount++;

      Serial.print(F("  [no-touch]  A="));
      pLong((long)a, 6);
      Serial.print(F("us  B="));
      pLong((long)b, 6);
      Serial.print(F("us  sum="));
      pLong((long)s, 6);
      Serial.print(F("us  ratio="));
      Serial.println(computeRatio(a, b));
    }
  }

  unsigned long noTouchAvgSum = noTouchCount > 0 ? noTouchSumAcc / noTouchCount : 0;
  Serial.println();
  Serial.println(F("  >>> TOUCH ANY NODE FIRMLY and hold for 4 seconds <<<"));
  Serial.println();

  // Touch measurements with live readout
  unsigned long touchSumMax = 0;
  unsigned long touchSumAcc = 0;
  long touchRatioMin = 999999L;
  long touchRatioMax = -999999L;
  int touchCount = 0;
  phaseEnd = millis() + 4000;

  while (millis() < phaseEnd) {
    unsigned long a, b;
    if (readDual(a, b)) {
      unsigned long s = a + b;
      long r = computeRatio(a, b);
      if (s > touchSumMax) touchSumMax = s;
      touchSumAcc += s;
      touchCount++;
      if (r < touchRatioMin) touchRatioMin = r;
      if (r > touchRatioMax) touchRatioMax = r;

      Serial.print(F("  [touch]     A="));
      pLong((long)a, 6);
      Serial.print(F("us  B="));
      pLong((long)b, 6);
      Serial.print(F("us  sum="));
      pLong((long)s, 6);
      Serial.print(F("us  ratio="));
      Serial.println(r);
    }
  }

  unsigned long touchAvgSum = touchCount > 0 ? touchSumAcc / touchCount : 0;
  phase0TouchAvg = touchAvgSum;  // save for adaptive threshold in Phase 1
  long delta = (long)(touchAvgSum - noTouchAvgSum);

  Serial.println();
  Serial.print(F("  No-touch avg sum: ")); Serial.print(noTouchAvgSum); Serial.println(F("us"));
  Serial.print(F("  Touch avg sum:    ")); Serial.print(touchAvgSum);   Serial.println(F("us"));
  Serial.print(F("  Delta:            +")); Serial.print(delta);         Serial.println(F("us"));
  Serial.print(F("  Ratio range on touch: "));
  Serial.print(touchRatioMin); Serial.print(F(" - ")); Serial.println(touchRatioMax);
  Serial.println();

  if (delta < 30) {
    Serial.println(F("  *** WARNING: weak signal (need delta > 30us) ***"));
    Serial.println(F("  Check:"));
    Serial.println(F("    - 10k ohm resistor between pin 5 and pin 2"));
    Serial.println(F("    - Pin 2 wire → TRACE START (not trace end!)"));
    Serial.println(F("    - Pin 3 wire → TRACE END (no other connections on pin 3)"));
    Serial.println(F("    - Trace is conductive (measure ~34k with multimeter)"));
    Serial.println(F("    - Touch the conductive material directly"));
  } else {
    Serial.print(F("  Signal OK! Delta = +"));
    Serial.print(delta);
    Serial.println(F("us on touch."));
    if (touchRatioMax - touchRatioMin < 30) {
      Serial.println(F("  NOTE: Ratio spread is small (<30). Node discrimination"));
      Serial.println(F("        may be limited. Calibration will determine feasibility."));
    }
  }

  Serial.println();
  Serial.println(F("  Press ENTER to continue..."));
  readSerialLine();

  // ── PHASE 1: Baseline ────────────────────────────────────────────
  Serial.println(F("[PHASE 1] Baseline measurement"));
  Serial.println(F("  HANDS OFF completely. Measuring in 3 seconds..."));
  delay(3000);

  long bRatioMean, bRatioStd, bSumMean;
  statsN(BASELINE_READS, bRatioMean, bRatioStd, bSumMean);
  baselineSum = bSumMean;

  // Adaptive threshold using Phase 0 touch data
  // Place threshold at 1/3 of the way from baseline to average touch sum
  long touchDelta = (long)phase0TouchAvg - baselineSum;
  touchThreshold = baselineSum + max(touchDelta / 3, 20L);

  Serial.print(F("  Baseline sum:       ")); Serial.println(bSumMean);
  Serial.print(F("  Baseline ratio:     ")); Serial.println(bRatioMean);
  Serial.print(F("  Preliminary threshold: > ")); Serial.println(touchThreshold);
  Serial.println();

  // ── PHASE 2: Per-node calibration ────────────────────────────────
  Serial.println(F("[PHASE 2] Per-node calibration"));
  Serial.println(F("  Touch each node when prompted. Hold firmly until released."));
  Serial.println(F("  Ratio is (Pin2_time / Pin3_time) x 1000:"));
  Serial.println(F("    Low ratio = Pin2 slower = body C near START = Node 1 side"));
  Serial.println(F("    High ratio = Pin3 slower = body C near END  = Node 4 side"));
  Serial.println();

  long minTouchSum = 999999999L;

  for (int i = 0; i < NUM_NODES; i++) {
    Serial.println(F("  ────────────────────────────────────────────────"));
    Serial.print(F("  >>> TOUCH AND HOLD Node "));
    Serial.print(i + 1);
    Serial.print(F("  ("));
    Serial.print(nodeResKohm[i], 0);
    Serial.println(F("k ohm from START) <<<"));

    // Wait for touch with live feedback
    int waitCount = 0;
    while (true) {
      unsigned long a, b;
      if (!readDual(a, b)) { Serial.println(F("    [wiring error?]")); continue; }
      long s = (long)(a + b);
      long r = computeRatio(a, b);
      waitCount++;
      if (waitCount % 3 == 0) {
        Serial.print(F("    sum="));  pLong(s, 6);
        Serial.print(F("us  ratio=")); pLong(r, 6);
        Serial.print(F("  (need sum > ")); Serial.print(touchThreshold); Serial.println(F(")"));
      }
      if (s > touchThreshold) break;
    }

    Serial.println(F("  Touch detected! Hold steady..."));
    delay(300);

    long rMean, rStd, sMean;
    statsN(CAL_READS_PER_NODE, rMean, rStd, sMean);
    nodeRatioMean[i] = rMean;
    nodeRatioStd[i]  = rStd;
    nodeSumMean[i]   = sMean;
    if (sMean < minTouchSum) minTouchSum = sMean;

    Serial.print(F("  Node ")); Serial.print(i + 1);
    Serial.print(F(": ratio=")); Serial.print(rMean);
    Serial.print(F(" ±")); Serial.print(rStd);
    Serial.print(F("  sum=")); Serial.print(sMean);
    Serial.print(F("us  delta=+")); Serial.println(sMean - baselineSum);

    Serial.println(F("  >>> RELEASE NOW <<<"));
    delay(800);

    int relCount = 0;
    while (relCount < 8) {
      unsigned long a, b;
      if (readDual(a, b) && (long)(a + b) < touchThreshold) relCount++;
      else relCount = 0;
    }
    Serial.println(F("  Released."));
    Serial.println();
  }

  // ── PHASE 3: Thresholds ──────────────────────────────────────────
  Serial.println(F("[PHASE 3] Computing thresholds..."));

  // Refine touch threshold using observed touch data
  touchThreshold = baselineSum + (minTouchSum - baselineSum) / 3;

  // Sort nodes by ratio mean (ascending)
  for (int i = 0; i < NUM_NODES; i++) sortIdx[i] = i;
  for (int i = 0; i < NUM_NODES - 1; i++) {
    for (int j = i + 1; j < NUM_NODES; j++) {
      if (nodeRatioMean[sortIdx[i]] > nodeRatioMean[sortIdx[j]]) {
        int tmp = sortIdx[i]; sortIdx[i] = sortIdx[j]; sortIdx[j] = tmp;
      }
    }
  }

  // Boundaries at midpoints between adjacent ratio means
  for (int rank = 0; rank < NUM_NODES; rank++) {
    int idx = sortIdx[rank];
    nodeRatioLow[idx]  = (rank == 0)           ? 0            : (nodeRatioMean[sortIdx[rank-1]] + nodeRatioMean[idx]) / 2;
    nodeRatioHigh[idx] = (rank == NUM_NODES-1) ? 999999L      : (nodeRatioMean[idx] + nodeRatioMean[sortIdx[rank+1]]) / 2;
  }

  // ── Calibration report ───────────────────────────────────────────
  Serial.println();
  Serial.println(F("  ═══════════════════ CALIBRATION RESULTS ═══════════════════"));
  Serial.print(F("  Baseline sum: ")); Serial.print(baselineSum);
  Serial.print(F("us  |  Touch threshold: > ")); Serial.print(touchThreshold); Serial.println(F("us"));
  Serial.println();
  Serial.println(F("  Node | R_start(k) | RatioMean | Std | SumDelta | Bounds"));
  Serial.println(F("  ─────|────────────|───────────|─────|──────────|──────────────"));

  for (int rank = 0; rank < NUM_NODES; rank++) {
    int idx = sortIdx[rank];
    Serial.print(F("   "));
    Serial.print(idx + 1);
    Serial.print(F("   |    "));
    Serial.print(nodeResKohm[idx], 0);
    Serial.print(F("       | "));
    pLong(nodeRatioMean[idx], 5);
    Serial.print(F("     | "));
    pLong(nodeRatioStd[idx], 3);
    Serial.print(F("  |  +"));
    pLong(nodeSumMean[idx] - baselineSum, 6);
    Serial.print(F("  | "));
    pLong(nodeRatioLow[idx], 6);
    Serial.print(F(" - "));
    pLong(nodeRatioHigh[idx], 6);
    Serial.println();
  }

  Serial.println();
  Serial.println(F("  Separation between adjacent nodes:"));
  bool allGood = true;
  for (int rank = 0; rank < NUM_NODES - 1; rank++) {
    int a = sortIdx[rank], b = sortIdx[rank + 1];
    long gap = nodeRatioMean[b] - nodeRatioMean[a];
    long boundary = (nodeRatioMean[a] + nodeRatioMean[b]) / 2;
    // sigma margin: how many std deviations from each node's mean to boundary
    float sigA = nodeRatioStd[a] > 0 ? (float)(boundary - nodeRatioMean[a]) / nodeRatioStd[a] : 99.0;
    float sigB = nodeRatioStd[b] > 0 ? (float)(nodeRatioMean[b] - boundary) / nodeRatioStd[b] : 99.0;
    float minSig = sigA < sigB ? sigA : sigB;

    Serial.print(F("    N")); Serial.print(a+1);
    Serial.print(F(" → N")); Serial.print(b+1);
    Serial.print(F(":  gap=")); Serial.print(gap);
    Serial.print(F("  margin="));
    Serial.print(minSig, 1);
    Serial.print(F("σ"));

    if (minSig < 2.0) {
      Serial.print(F("  *** POOR — nodes may overlap! ***"));
      allGood = false;
    } else if (minSig < 3.0) {
      Serial.print(F("  (marginal, press firmly and centrally)"));
    }
    Serial.println();
  }

  Serial.println();
  if (allGood) {
    Serial.println(F("  >> All nodes well-separated. Calibration GOOD."));
  } else {
    Serial.println(F("  >> WARNING: some nodes have poor separation."));
    Serial.println(F("     Try a smaller external resistor (470k) for wider ratio spread."));
    Serial.println(F("     Or re-run calibration touching each node at dead-center."));
  }

  calibrated = true;
  Serial.println();
  Serial.println(F("  ═════════════════════════════════════════════════════════════"));
  Serial.println(F("  LIVE DETECTION  (200ms readout)"));
  Serial.println(F("  ═════════════════════════════════════════════════════════════"));
  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════
// LOOP — live detection
// ═════════════════════════════════════════════════════════════════════
void loop() {
  if (!calibrated) return;

  unsigned long a, b;
  if (!readDual(a, b)) return;  // wiring error, skip

  long sum   = (long)(a + b);
  long ratio = computeRatio(a, b);

  // ── Node identification ─────────────────────────────────────────
  int detectedNode = -1;
  if (sum > touchThreshold) {
    // Range-based lookup
    for (int i = 0; i < NUM_NODES; i++) {
      if (ratio >= nodeRatioLow[i] && ratio < nodeRatioHigh[i]) {
        detectedNode = i;
        break;
      }
    }
    // Fallback: nearest mean
    if (detectedNode == -1) {
      long best = 999999L;
      for (int i = 0; i < NUM_NODES; i++) {
        long d = abs(ratio - nodeRatioMean[i]);
        if (d < best) { best = d; detectedNode = i; }
      }
    }
  }

  // ── Debounce state machine ──────────────────────────────────────
  if (currentNode == -1) {
    if (detectedNode != -1) {
      if (detectedNode == candidateNode) hitCounter++;
      else { candidateNode = detectedNode; hitCounter = 1; }
      if (hitCounter >= DEBOUNCE_COUNT) {
        currentNode = candidateNode;
        Serial.println();
        Serial.print(F(">>> Node ")); Serial.print(currentNode + 1); Serial.println(F(" PRESSED <<<"));
        hitCounter = 0; missCounter = 0;
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
      }
    } else if (detectedNode == currentNode) {
      missCounter = 0;
    } else {
      missCounter++;
      if (missCounter >= RELEASE_COUNT) {
        Serial.print(F("[SWITCH → Node ")); Serial.print(detectedNode + 1); Serial.println(F("]"));
        currentNode = detectedNode; missCounter = 0;
      }
    }
  }

  // ── 200ms debug readout ─────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    Serial.print(F("A="));  pLong((long)a, 6); Serial.print(F("us "));
    Serial.print(F("B="));  pLong((long)b, 6); Serial.print(F("us "));
    Serial.print(F("sum=")); pLong(sum, 7);     Serial.print(F("us "));
    Serial.print(F("r="));  pLong(ratio, 5);

    if (currentNode >= 0) {
      Serial.print(F("  [N")); Serial.print(currentNode + 1); Serial.print(F("]"));
    } else if (sum > touchThreshold) {
      Serial.print(F("  [? unclassified]"));
    } else {
      Serial.print(F("  [-]"));
    }

    // Show distance to each node's ratio mean
    if (sum > touchThreshold / 2) {
      Serial.print(F("  d:"));
      for (int i = 0; i < NUM_NODES; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(abs(ratio - nodeRatioMean[i]));
      }
    }
    Serial.println();
  }
}
