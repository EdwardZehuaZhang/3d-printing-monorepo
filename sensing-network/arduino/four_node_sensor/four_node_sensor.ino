/*
 * 4-Node Dual-Ended Ratio Touch Sensor
 * -------------------------------------
 * Uses the TWO-ENDED RATIO method from the research paper.
 * Much more reliable than single-ended capacitive sensing because:
 *   - Measures from BOTH ends of the trace simultaneously
 *   - Uses the RATIO of charge times (immune to absolute drift)
 *   - No external library needed (custom charge/discharge cycle)
 *
 * Your 3D-printed trace:
 *   START -[4k]- N1 -[11k]- N2 -[7k]- N3 -[10k]- N4 -[2k]- END
 *   Cumulative from start: 4k, 15k, 22k, 32k ohm
 *   Total: ~34 kohm
 *
 * WIRING:
 *   Pin 5 (SEND)   -> 10k resistor -> junction
 *   Pin 2 (RCV_A)  -> wire to START of trace
 *   Pin 3 (RCV_B)  -> wire to END of trace
 *   The junction connects to BOTH Pin 2 and Pin 3 (and the resistor)
 *
 *   See wiring diagram in serial output on boot.
 */

// ── Pin config ──────────────────────────────────────────────────────
#define SEND_PIN   5
#define RCV_A_PIN  2   // START of trace (Node 1 side)
#define RCV_B_PIN  3   // END of trace (Node 4 side)

// ── Trace parameters ────────────────────────────────────────────────
#define NUM_NODES 4
const float cumulativeResKohm[NUM_NODES] = {4.0, 15.0, 22.0, 32.0};

// ── Sensing parameters ──────────────────────────────────────────────
#define TIMEOUT_LOOPS      30000L  // max count before giving up
#define AVG_SAMPLES        30      // readings to average per measurement
#define CALIBRATION_READS  50      // readings per node during calibration
#define DEBOUNCE_COUNT     3       // consecutive hits to confirm touch
#define RELEASE_COUNT      4       // consecutive misses to confirm release
#define SMOOTH_SIZE        5       // running average window

// ── Calibration data ────────────────────────────────────────────────
long   nodeRatioMean[NUM_NODES];   // ratio * 1000 (integer math)
long   nodeRatioMin[NUM_NODES];
long   nodeRatioMax[NUM_NODES];
long   nodeRatioStd[NUM_NODES];
long   nodeThresholdLow[NUM_NODES];
long   nodeThresholdHigh[NUM_NODES];
int    sortIdx[NUM_NODES];

// ── Baseline ────────────────────────────────────────────────────────
long   baselineTotalA  = 0;   // average no-touch count on pin A
long   baselineTotalB  = 0;   // average no-touch count on pin B
long   baselineSum     = 0;   // A + B baseline
long   touchThreshold  = 0;   // minimum (A+B) to count as touch

// ── Runtime state ───────────────────────────────────────────────────
int    currentNode    = -1;
int    candidateNode  = -1;
int    hitCounter     = 0;
int    missCounter    = 0;
bool   calibrated     = false;

// ── Smoothing (on ratio) ────────────────────────────────────────────
long   smoothBuf[SMOOTH_SIZE];
int    smoothIdx  = 0;
bool   smoothFull = false;
long   smoothSumBuf[SMOOTH_SIZE];  // for total (touch detection)
unsigned long lastPrintMs = 0;

// ═════════════════════════════════════════════════════════════════════
// CORE: Dual-ended charge measurement
// ═════════════════════════════════════════════════════════════════════
void readDual(long &timeA, long &timeB) {
  timeA = TIMEOUT_LOOPS;
  timeB = TIMEOUT_LOOPS;

  // 1. Discharge everything
  pinMode(SEND_PIN, OUTPUT);
  pinMode(RCV_A_PIN, OUTPUT);
  pinMode(RCV_B_PIN, OUTPUT);
  digitalWrite(SEND_PIN, LOW);
  digitalWrite(RCV_A_PIN, LOW);
  digitalWrite(RCV_B_PIN, LOW);
  delayMicroseconds(500);

  // 2. Set receive pins to INPUT (high impedance)
  pinMode(RCV_A_PIN, INPUT);
  pinMode(RCV_B_PIN, INPUT);

  // 3. Send charge pulse
  digitalWrite(SEND_PIN, HIGH);

  // 4. Count until each pin goes HIGH
  volatile long count = 0;
  bool doneA = false;
  bool doneB = false;

  while ((!doneA || !doneB) && count < TIMEOUT_LOOPS) {
    count++;
    if (!doneA && digitalRead(RCV_A_PIN) == HIGH) {
      timeA = count;
      doneA = true;
    }
    if (!doneB && digitalRead(RCV_B_PIN) == HIGH) {
      timeB = count;
      doneB = true;
    }
  }

  // 5. Discharge
  digitalWrite(SEND_PIN, LOW);
}

// Get averaged dual reading
void readDualAvg(int samples, long &avgA, long &avgB) {
  long totalA = 0, totalB = 0;
  long a, b;
  for (int i = 0; i < samples; i++) {
    readDual(a, b);
    totalA += a;
    totalB += b;
  }
  avgA = totalA / samples;
  avgB = totalB / samples;
}

// Compute ratio: (A * 1000) / B  — integer fixed-point
long computeRatio(long a, long b) {
  if (b == 0) b = 1;
  return (a * 1000L) / b;
}

// Stats on the ratio over N readings
void statsRatio(int n, long &outMean, long &outMin, long &outMax,
                long &outStd, long &outSumMean) {
  long buf[100];
  long sumBuf[100];
  if (n > 100) n = 100;
  long sum = 0;
  long sumSum = 0;
  outMin = 999999999L;
  outMax = -999999999L;

  for (int i = 0; i < n; i++) {
    long a, b;
    readDual(a, b);
    buf[i] = computeRatio(a, b);
    sumBuf[i] = a + b;
    sum += buf[i];
    sumSum += sumBuf[i];
    if (buf[i] < outMin) outMin = buf[i];
    if (buf[i] > outMax) outMax = buf[i];
    delay(5);
  }

  outMean = sum / n;
  outSumMean = sumSum / n;

  long sqSum = 0;
  for (int i = 0; i < n; i++) {
    long d = buf[i] - outMean;
    sqSum += d * d;
  }
  outStd = (long)sqrt((double)sqSum / (double)n);
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

void printLong(long val, int width) {
  String s = String(val);
  for (int i = s.length(); i < width; i++) Serial.print(' ');
  Serial.print(s);
}

void printFloat(float val, int decimals, int width) {
  String s = String(val, decimals);
  for (int i = s.length(); i < width; i++) Serial.print(' ');
  Serial.print(s);
}

// ═════════════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("  4-Node Dual-Ended Ratio Touch Sensor"));
  Serial.println(F("================================================"));
  Serial.println();
  Serial.println(F("  Trace: START -[4k]- N1 -[11k]- N2 -[7k]- N3 -[10k]- N4 - END"));
  Serial.println();
  Serial.println(F("  WIRING:"));
  Serial.println(F("              10k resistor"));
  Serial.println(F("    Pin 5 ---[████████]---+--- Pin 2 --- wire to TRACE START"));
  Serial.println(F("                          |"));
  Serial.println(F("                          +--- Pin 3 --- wire to TRACE END"));
  Serial.println();

  // ── PHASE 0: Raw signal test ────────────────────────────────────
  Serial.println(F("[PHASE 0] Raw signal test"));
  Serial.println(F("  DON'T TOUCH for 4 seconds..."));

  long noTouchSumMax = 0;
  long noTouchSumMin = 999999999L;
  unsigned long startMs = millis();
  int readCount = 0;
  while (millis() - startMs < 4000) {
    long a, b;
    readDual(a, b);
    long s = a + b;
    if (s > noTouchSumMax) noTouchSumMax = s;
    if (s < noTouchSumMin) noTouchSumMin = s;
    readCount++;
    if (readCount % 10 == 0) {
      Serial.print(F("  [no-touch] A="));
      printLong(a, 6);
      Serial.print(F("  B="));
      printLong(b, 6);
      Serial.print(F("  sum="));
      printLong(s, 6);
      Serial.print(F("  ratio="));
      Serial.println(computeRatio(a, b));
    }
    delay(10);
  }

  Serial.println();
  Serial.println(F("  >>> NOW TOUCH any node and hold <<<"));
  Serial.println();

  long touchSumMax = 0;
  startMs = millis();
  readCount = 0;
  while (millis() - startMs < 4000) {
    long a, b;
    readDual(a, b);
    long s = a + b;
    if (s > touchSumMax) touchSumMax = s;
    readCount++;
    if (readCount % 10 == 0) {
      Serial.print(F("  [touch]    A="));
      printLong(a, 6);
      Serial.print(F("  B="));
      printLong(b, 6);
      Serial.print(F("  sum="));
      printLong(s, 6);
      Serial.print(F("  ratio="));
      Serial.println(computeRatio(a, b));
    }
    delay(10);
  }

  long delta = touchSumMax - noTouchSumMax;
  Serial.println();
  Serial.print(F("  No-touch sum range: "));
  Serial.print(noTouchSumMin);
  Serial.print(F(" - "));
  Serial.println(noTouchSumMax);
  Serial.print(F("  Touch sum peak:     "));
  Serial.println(touchSumMax);
  Serial.print(F("  Delta:              "));
  Serial.println(delta);

  if (delta < 3) {
    Serial.println();
    Serial.println(F("  *** WARNING: Very weak signal! Check: ***"));
    Serial.println(F("    - 10k resistor between pin 5 and the junction"));
    Serial.println(F("    - Pin 2 wire to START of conductive trace"));
    Serial.println(F("    - Pin 3 wire to END of conductive trace"));
    Serial.println(F("    - Trace is continuous (test with multimeter)"));
  } else if (delta < 20) {
    Serial.println(F("  Signal weak but detectable. Should work."));
  } else {
    Serial.println(F("  Signal looks good!"));
  }

  Serial.println();
  Serial.println(F("  Press ENTER to continue..."));
  readSerialLine();

  // ── PHASE 1: Baseline ──────────────────────────────────────────
  Serial.println(F("[PHASE 1] Baseline measurement"));
  Serial.println(F("  HANDS OFF completely! Measuring in 3 seconds..."));
  delay(3000);

  long bA, bB;
  readDualAvg(50, bA, bB);
  baselineTotalA = bA;
  baselineTotalB = bB;
  baselineSum = bA + bB;

  // Touch threshold: 20% above baseline sum, minimum +5
  long margin = max(baselineSum / 5, (long)5);
  touchThreshold = baselineSum + margin;

  Serial.print(F("  Baseline A: ")); Serial.println(baselineTotalA);
  Serial.print(F("  Baseline B: ")); Serial.println(baselineTotalB);
  Serial.print(F("  Baseline sum: ")); Serial.println(baselineSum);
  Serial.print(F("  Touch threshold (A+B > ): ")); Serial.println(touchThreshold);
  Serial.println();

  // ── PHASE 2: Per-node calibration ──────────────────────────────
  Serial.println(F("[PHASE 2] Per-node calibration"));
  Serial.println(F("  Touch each node when prompted. Hold firmly until told to release."));
  Serial.println(F("  The RATIO of A/B uniquely identifies each node position."));
  Serial.println();

  for (int i = 0; i < NUM_NODES; i++) {
    Serial.println(F("  ----------------------------------------"));
    Serial.print(F("  >>> TOUCH AND HOLD  Node "));
    Serial.print(i + 1);
    Serial.print(F("  ("));
    Serial.print(cumulativeResKohm[i], 0);
    Serial.println(F("k from start) <<<"));
    Serial.println(F("  Waiting for touch..."));

    // Wait for touch with feedback
    int waitCount = 0;
    while (true) {
      long a, b;
      readDual(a, b);
      long s = a + b;
      waitCount++;
      if (waitCount % 15 == 0) {
        Serial.print(F("    waiting... sum="));
        Serial.print(s);
        Serial.print(F("  (need > "));
        Serial.print(touchThreshold);
        Serial.print(F(")  ratio="));
        Serial.println(computeRatio(a, b));
      }
      if (s > touchThreshold) break;
      delay(10);
    }

    Serial.println(F("  Touch detected! Hold steady... collecting data..."));
    delay(500);  // settle time

    long rMean, rMin, rMax, rStd, sMean;
    statsRatio(CALIBRATION_READS, rMean, rMin, rMax, rStd, sMean);

    nodeRatioMean[i] = rMean;
    nodeRatioMin[i]  = rMin;
    nodeRatioMax[i]  = rMax;
    nodeRatioStd[i]  = rStd;

    Serial.print(F("    ratio mean="));
    Serial.print(rMean);
    Serial.print(F("  min="));
    Serial.print(rMin);
    Serial.print(F("  max="));
    Serial.print(rMax);
    Serial.print(F("  std="));
    Serial.print(rStd);
    Serial.print(F("  avgSum="));
    Serial.println(sMean);

    Serial.println(F("  >>> RELEASE NOW <<<"));
    delay(1500);

    // Wait for confirmed release
    int relCount = 0;
    while (relCount < 10) {
      long a, b;
      readDual(a, b);
      if ((a + b) < touchThreshold) relCount++;
      else relCount = 0;
      delay(10);
    }
    Serial.println(F("  Released. OK."));
    Serial.println();
  }

  // ── PHASE 3: Compute thresholds ────────────────────────────────
  Serial.println(F("[PHASE 3] Computing thresholds..."));

  // Sort by calibrated ratio mean (ascending)
  for (int i = 0; i < NUM_NODES; i++) sortIdx[i] = i;
  for (int i = 0; i < NUM_NODES - 1; i++) {
    for (int j = i + 1; j < NUM_NODES; j++) {
      if (nodeRatioMean[sortIdx[i]] > nodeRatioMean[sortIdx[j]]) {
        int tmp = sortIdx[i];
        sortIdx[i] = sortIdx[j];
        sortIdx[j] = tmp;
      }
    }
  }

  // Thresholds as midpoints between adjacent ratio means
  for (int rank = 0; rank < NUM_NODES; rank++) {
    int idx = sortIdx[rank];
    if (rank == 0) {
      nodeThresholdLow[idx] = 0;  // any ratio is valid if touch detected
    } else {
      int prevIdx = sortIdx[rank - 1];
      long midpoint = (nodeRatioMean[prevIdx] + nodeRatioMean[idx]) / 2;
      nodeThresholdLow[idx] = midpoint;
      nodeThresholdHigh[prevIdx] = midpoint;
    }
    if (rank == NUM_NODES - 1) {
      nodeThresholdHigh[idx] = 999999L;
    }
  }

  // ── Calibration report ─────────────────────────────────────────
  Serial.println();
  Serial.println(F("  ============ CALIBRATION RESULTS ============"));
  Serial.print(F("  Baseline sum: ")); Serial.print(baselineSum);
  Serial.print(F("  |  Touch threshold: ")); Serial.println(touchThreshold);
  Serial.println();

  Serial.println(F("  Node | R_cum(k) | Ratio  | Std  | Range       | Thresh lo-hi"));
  Serial.println(F("  -----|----------|--------|------|-------------|-------------"));

  for (int rank = 0; rank < NUM_NODES; rank++) {
    int idx = sortIdx[rank];
    Serial.print(F("   "));
    if (idx + 1 < 10) Serial.print(' ');
    Serial.print(idx + 1);
    Serial.print(F("   | "));
    printFloat(cumulativeResKohm[idx], 0, 7);
    Serial.print(F(" | "));
    printLong(nodeRatioMean[idx], 6);
    Serial.print(F(" | "));
    printLong(nodeRatioStd[idx], 4);
    Serial.print(F(" | "));
    printLong(nodeRatioMin[idx], 5);
    Serial.print(F(" - "));
    printLong(nodeRatioMax[idx], 5);
    Serial.print(F(" | "));
    printLong(nodeThresholdLow[idx], 5);
    Serial.print(F(" - "));
    printLong(nodeThresholdHigh[idx], 5);
    Serial.println();
  }

  // Separation quality
  Serial.println();
  Serial.println(F("  Separation quality:"));
  bool allGood = true;
  for (int rank = 0; rank < NUM_NODES - 1; rank++) {
    int a = sortIdx[rank];
    int b = sortIdx[rank + 1];
    long gap = nodeRatioMean[b] - nodeRatioMean[a];
    long worstOverlap = nodeRatioMax[a] - nodeRatioMin[b];

    Serial.print(F("    Node "));
    Serial.print(a + 1);
    Serial.print(F(" -> Node "));
    Serial.print(b + 1);
    Serial.print(F(" :  ratio gap="));
    Serial.print(gap);

    if (worstOverlap > 0) {
      Serial.print(F("  ** OVERLAP="));
      Serial.print(worstOverlap);
      Serial.print(F(" **"));
      allGood = false;
    } else {
      Serial.print(F("  margin="));
      Serial.print(-worstOverlap);
    }
    Serial.println();
  }

  Serial.println();
  if (allGood) {
    Serial.println(F("  >> All nodes well-separated. Calibration GOOD!"));
  } else {
    Serial.println(F("  >> WARNING: Overlapping ratios. Nodes may be confused."));
    Serial.println(F("     Make sure you're touching nodes firmly in the center."));
  }

  calibrated = true;
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("  LIVE DETECTION ACTIVE"));
  Serial.println(F("  Format: A | B | sum | ratio | node"));
  Serial.println(F("================================================"));
  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════
// LOOP — live detection
// ═════════════════════════════════════════════════════════════════════
void loop() {
  if (!calibrated) return;

  // Take averaged reading
  long a = 0, b = 0;
  long tmpA, tmpB;
  for (int i = 0; i < AVG_SAMPLES; i++) {
    readDual(tmpA, tmpB);
    a += tmpA;
    b += tmpB;
  }
  a /= AVG_SAMPLES;
  b /= AVG_SAMPLES;

  long totalSum = a + b;
  long ratio = computeRatio(a, b);

  // Smooth the ratio
  smoothBuf[smoothIdx] = ratio;
  smoothSumBuf[smoothIdx] = totalSum;
  smoothIdx = (smoothIdx + 1) % SMOOTH_SIZE;
  if (!smoothFull && smoothIdx == 0) smoothFull = true;

  int cnt = smoothFull ? SMOOTH_SIZE : max(smoothIdx, 1);
  long ratioSmooth = 0;
  long sumSmooth = 0;
  for (int i = 0; i < cnt; i++) {
    ratioSmooth += smoothBuf[i];
    sumSmooth += smoothSumBuf[i];
  }
  ratioSmooth /= cnt;
  sumSmooth /= cnt;

  // ── Identify node ─────────────────────────────────────────────
  int detectedNode = -1;

  if (sumSmooth > touchThreshold) {
    // Range-based match on ratio
    for (int i = 0; i < NUM_NODES; i++) {
      if (ratioSmooth >= nodeThresholdLow[i] && ratioSmooth < nodeThresholdHigh[i]) {
        detectedNode = i;
        break;
      }
    }
    // Fallback: nearest ratio mean
    if (detectedNode == -1) {
      long bestDist = 999999999L;
      for (int i = 0; i < NUM_NODES; i++) {
        long dist = abs(ratioSmooth - nodeRatioMean[i]);
        if (dist < bestDist) {
          bestDist = dist;
          detectedNode = i;
        }
      }
    }
  }

  // ── Debounce state machine ────────────────────────────────────
  if (currentNode == -1) {
    if (detectedNode != -1) {
      if (detectedNode == candidateNode) {
        hitCounter++;
      } else {
        candidateNode = detectedNode;
        hitCounter = 1;
      }
      if (hitCounter >= DEBOUNCE_COUNT) {
        currentNode = candidateNode;
        Serial.println();
        Serial.print(F(">>> Node "));
        Serial.print(currentNode + 1);
        Serial.println(F(" PRESSED <<<"));
        hitCounter = 0;
        missCounter = 0;
      }
    } else {
      hitCounter = 0;
      candidateNode = -1;
    }
  } else {
    if (detectedNode == -1) {
      missCounter++;
      if (missCounter >= RELEASE_COUNT) {
        Serial.print(F("[RELEASE] Node "));
        Serial.println(currentNode + 1);
        Serial.println();
        currentNode = -1;
        candidateNode = -1;
        hitCounter = 0;
        missCounter = 0;
      }
    } else if (detectedNode == currentNode) {
      missCounter = 0;
    } else {
      missCounter++;
      if (missCounter >= RELEASE_COUNT) {
        Serial.print(F("[SWITCH] Node "));
        Serial.print(currentNode + 1);
        Serial.print(F(" -> Node "));
        Serial.println(detectedNode + 1);
        currentNode = detectedNode;
        missCounter = 0;
      }
    }
  }

  // ── Debug output (every 200ms) ────────────────────────────────
  unsigned long now = millis();
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    Serial.print(F("A="));
    printLong(a, 5);
    Serial.print(F(" B="));
    printLong(b, 5);
    Serial.print(F(" sum="));
    printLong(totalSum, 5);
    Serial.print(F(" r="));
    printLong(ratioSmooth, 5);

    if (currentNode >= 0) {
      Serial.print(F(" [N"));
      Serial.print(currentNode + 1);
      Serial.print(F("]"));
    } else {
      Serial.print(F(" [-]"));
    }
    Serial.println();
  }

  delay(10);
}
