/*
 * Touch Calibration & Detection for 3D-Printed Single-Wire Sensing
 * -----------------------------------------------------------------
 * Works with CapacitiveSensorR4 on Arduino Uno R4.
 * Change to Paul Badger's CapacitiveSensor library for Uno R3.
 *
 * WORKFLOW:
 *   Phase 0 — Raw signal test (verify hardware works before anything else)
 *   Phase 1 — Enter number of touch nodes
 *   Phase 2 — Enter segment resistances from Rhino (node-to-node pairs)
 *   Phase 3 — Baseline measurement (hands off)
 *   Phase 4 — Per-node touch calibration
 *   Phase 5 — Threshold computation & quality report
 *   Phase 6 — Live detection with debug output
 *
 * HARDWARE:
 *   External resistor between SEND_PIN and RECEIVE_PIN.
 *   Use 10k-47k for low total path resistance (<50 kohm).
 *   Use 100k-1M  for high total path resistance (>100 kohm).
 *   Conductive wire from RECEIVE_PIN to the printed in-node.
 *
 * RESISTOR GUIDE (match to your Rhino total resistance):
 *   Total path ~35 kohm  -> use 10 kohm external resistor
 *   Total path ~100 kohm -> use 47 kohm external resistor
 *   Total path ~200 kohm -> use 100 kohm-1M external resistor
 *   Rule of thumb: external R should be 0.3x-1x of total path R.
 *   Too large = all nodes read the same. Too small = no signal.
 */

#include <CapacitiveSensorR4.h>

// ── Pin config ──────────────────────────────────────────────────────
#define SEND_PIN    5
#define RECEIVE_PIN 2

// ── Sensing parameters ──────────────────────────────────────────────
#define CAP_SAMPLES        30      // samples per capacitiveSensor() call
#define BASELINE_WINDOW    50      // readings to establish baseline
#define CALIBRATION_READS  60      // readings per node during calibration
#define DEBOUNCE_COUNT     3       // consecutive hits to confirm touch
#define RELEASE_COUNT      4       // consecutive misses to confirm release
#define MAX_NODES          12      // max touch nodes
#define SMOOTH_SIZE        5       // running average window

// ── Objects ─────────────────────────────────────────────────────────
CapacitiveSensor cs = CapacitiveSensor(SEND_PIN, RECEIVE_PIN);

// ── Node data ───────────────────────────────────────────────────────
int    numNodes = 0;
float  segmentResKohm[MAX_NODES];           // segment resistance between consecutive nodes
float  cumulativeResKohm[MAX_NODES];        // cumulative resistance from in-node to each node
long   nodeCalMean[MAX_NODES];              // mean sensor reading when touched
long   nodeCalMin[MAX_NODES];               // min during calibration
long   nodeCalMax[MAX_NODES];               // max during calibration
long   nodeCalStd[MAX_NODES];               // std dev during calibration
long   nodeThresholdLow[MAX_NODES];         // lower threshold
long   nodeThresholdHigh[MAX_NODES];        // upper threshold

// ── Runtime state ───────────────────────────────────────────────────
long   baseline          = 0;
long   baselineNoise     = 0;
long   touchThreshold    = 0;
int    currentNode       = -1;
int    candidateNode     = -1;
int    hitCounter        = 0;
int    missCounter       = 0;
bool   calibrated        = false;
int    sortIdx[MAX_NODES];                  // nodes sorted by reading magnitude

// ── Smoothing ───────────────────────────────────────────────────────
long   smoothBuf[SMOOTH_SIZE];
int    smoothIdx = 0;
bool   smoothFull = false;
unsigned long lastPrintMs = 0;

// ═════════════════════════════════════════════════════════════════════
// HELPERS
// ═════════════════════════════════════════════════════════════════════

float readSerialFloat() {
  while (true) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() == 0) continue;
      float v = s.toFloat();
      if (v == 0.0 && s.charAt(0) != '0') {
        Serial.println(F("  [!] Invalid number. Try again:"));
        continue;
      }
      return v;
    }
  }
}

int readSerialInt() {
  return (int)readSerialFloat();
}

String readSerialLine() {
  while (true) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() > 0) return s;
    }
  }
}

void statsReading(int n, long &outMean, long &outMin, long &outMax, long &outStd) {
  long buf[100];
  if (n > 100) n = 100;
  long sum = 0;
  outMin = 999999999L;
  outMax = -999999999L;
  for (int i = 0; i < n; i++) {
    buf[i] = cs.capacitiveSensor(CAP_SAMPLES);
    sum += buf[i];
    if (buf[i] < outMin) outMin = buf[i];
    if (buf[i] > outMax) outMax = buf[i];
    delay(10);
  }
  outMean = sum / n;
  long sqSum = 0;
  for (int i = 0; i < n; i++) {
    long d = buf[i] - outMean;
    sqSum += d * d;
  }
  outStd = (long)sqrt((double)sqSum / (double)n);
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
  Serial.println(F("  3DP Single-Wire Touch Calibration Tool"));
  Serial.println(F("================================================"));
  Serial.println();

  // ──────────────────────────────────────────────────────────────────
  // PHASE 0: Raw signal test
  // ──────────────────────────────────────────────────────────────────
  Serial.println(F("[PHASE 0] Raw signal test"));
  Serial.println(F("  Verifying sensor hardware before calibration."));
  Serial.println(F("  Streaming raw readings for 8 seconds..."));
  Serial.println(F("  First 4s: DON'T TOUCH.  Last 4s: TOUCH any node."));
  Serial.println();

  long phase0_noTouch_max = 0;
  long phase0_noTouch_min = 999999999L;
  long phase0_touch_max = 0;

  // 4 seconds no-touch
  unsigned long startMs = millis();
  int readCount = 0;
  while (millis() - startMs < 4000) {
    long v = cs.capacitiveSensor(CAP_SAMPLES);
    if (v > phase0_noTouch_max) phase0_noTouch_max = v;
    if (v < phase0_noTouch_min) phase0_noTouch_min = v;
    readCount++;
    if (readCount % 5 == 0) {
      Serial.print(F("  [no-touch] raw="));
      Serial.println(v);
    }
    delay(20);
  }

  Serial.println();
  Serial.println(F("  >>> NOW TOUCH any node and hold <<<"));
  Serial.println();

  // 4 seconds with touch
  startMs = millis();
  readCount = 0;
  while (millis() - startMs < 4000) {
    long v = cs.capacitiveSensor(CAP_SAMPLES);
    if (v > phase0_touch_max) phase0_touch_max = v;
    readCount++;
    if (readCount % 5 == 0) {
      Serial.print(F("  [touch]    raw="));
      Serial.println(v);
    }
    delay(20);
  }

  Serial.println();
  Serial.print(F("  No-touch range: "));
  Serial.print(phase0_noTouch_min);
  Serial.print(F(" - "));
  Serial.println(phase0_noTouch_max);
  Serial.print(F("  Touch peak:     "));
  Serial.println(phase0_touch_max);

  long signalRange = phase0_touch_max - phase0_noTouch_max;
  Serial.print(F("  Signal delta:   "));
  Serial.println(signalRange);

  if (signalRange < 5) {
    Serial.println();
    Serial.println(F("  *** WARNING: Very weak or no signal change detected! ***"));
    Serial.println(F("  Possible causes:"));
    Serial.println(F("    - External resistor too large (try 10k-47k for ~35 kohm path)"));
    Serial.println(F("    - Wiring not connected to conductive path"));
    Serial.println(F("    - Conductive filament path is broken/discontinuous"));
    Serial.println(F("    - Wrong pin wiring (send=5, receive=2, resistor on pin 5)"));
    Serial.println();
    Serial.println(F("  Continuing anyway... but calibration may fail."));
  } else if (signalRange < 50) {
    Serial.println(F("  Signal is weak but detectable. A smaller external resistor may help."));
  } else {
    Serial.println(F("  Signal looks good!"));
  }

  Serial.println();
  Serial.println(F("  Press ENTER to continue to calibration..."));
  readSerialLine();

  // ──────────────────────────────────────────────────────────────────
  // PHASE 1: Number of nodes
  // ──────────────────────────────────────────────────────────────────
  Serial.println(F("[PHASE 1] How many touch nodes?"));
  Serial.print(F("  Enter count (1-"));
  Serial.print(MAX_NODES);
  Serial.println(F("):"));

  numNodes = readSerialInt();
  if (numNodes < 1) numNodes = 1;
  if (numNodes > MAX_NODES) numNodes = MAX_NODES;
  Serial.print(F("  -> "));
  Serial.print(numNodes);
  Serial.println(F(" node(s)"));
  Serial.println();

  // ──────────────────────────────────────────────────────────────────
  // PHASE 2: Segment resistances from Rhino
  // ──────────────────────────────────────────────────────────────────
  Serial.println(F("[PHASE 2] Enter segment resistances from Rhino."));
  Serial.println(F("  Rhino prints lines like:"));
  Serial.println(F("    NodeA - NodeB (length: Xmm, resistance: 10.5-11.0 kohm)"));
  Serial.println();
  Serial.println(F("  Enter the MID value of each segment resistance (kohm)."));
  Serial.println(F("  e.g. if Rhino says 10.5-11.0 kohm, enter 10.75"));
  Serial.println();

  if (numNodes == 1) {
    // Single node: just need in-node to the one touch node
    Serial.println(F("  Segment: in-node -> Node 1"));
    Serial.print(F("  Resistance (kohm): "));
    segmentResKohm[0] = readSerialFloat();
    Serial.print(F("    -> "));
    Serial.print(segmentResKohm[0], 1);
    Serial.println(F(" kohm"));
    cumulativeResKohm[0] = segmentResKohm[0];
  } else {
    // Multiple nodes: segments between consecutive nodes
    // First segment is in-node to first touch node
    Serial.println(F("  Segment 1: in-node -> Node 1"));
    Serial.print(F("  Resistance (kohm): "));
    segmentResKohm[0] = readSerialFloat();
    Serial.print(F("    -> "));
    Serial.print(segmentResKohm[0], 1);
    Serial.println(F(" kohm"));
    cumulativeResKohm[0] = segmentResKohm[0];

    for (int i = 1; i < numNodes; i++) {
      Serial.print(F("  Segment "));
      Serial.print(i + 1);
      Serial.print(F(": Node "));
      Serial.print(i);
      Serial.print(F(" -> Node "));
      Serial.println(i + 1);
      Serial.print(F("  Resistance (kohm): "));
      segmentResKohm[i] = readSerialFloat();
      Serial.print(F("    -> "));
      Serial.print(segmentResKohm[i], 1);
      Serial.println(F(" kohm"));
      cumulativeResKohm[i] = cumulativeResKohm[i - 1] + segmentResKohm[i];
    }
  }

  // Print summary
  Serial.println();
  Serial.println(F("  Resistance map (cumulative from in-node):"));
  Serial.println(F("  -----------------------------------------"));
  for (int i = 0; i < numNodes; i++) {
    Serial.print(F("    Node "));
    Serial.print(i + 1);
    Serial.print(F(":  segment="));
    printFloat(segmentResKohm[i], 1, 6);
    Serial.print(F("  cumulative="));
    printFloat(cumulativeResKohm[i], 1, 6);
    Serial.println(F(" kohm"));
  }
  Serial.print(F("  Total path resistance: ~"));
  Serial.print(cumulativeResKohm[numNodes - 1], 1);
  Serial.println(F(" kohm"));

  // Resistor recommendation
  float totalR = cumulativeResKohm[numNodes - 1];
  Serial.println();
  if (totalR < 50) {
    Serial.println(F("  TIP: With ~35 kohm total, use a 10k-33k external resistor."));
    Serial.println(F("       1M is too large — it masks the node differences."));
  } else if (totalR < 150) {
    Serial.println(F("  TIP: With this range, try 47k-100k external resistor."));
  } else {
    Serial.println(F("  TIP: With high path resistance, 100k-1M external works well."));
  }
  Serial.println();

  // ──────────────────────────────────────────────────────────────────
  // PHASE 3: Baseline measurement
  // ──────────────────────────────────────────────────────────────────
  Serial.println(F("[PHASE 3] Baseline measurement."));
  Serial.println(F("  *** HANDS OFF the object completely! ***"));
  Serial.println(F("  Measuring in 3 seconds..."));
  delay(3000);

  long bMean, bMin, bMax, bStd;
  statsReading(BASELINE_WINDOW, bMean, bMin, bMax, bStd);
  baseline = bMean;
  baselineNoise = max(bStd * 3, bMax - bMean);

  Serial.print(F("  Baseline mean : "));
  Serial.println(baseline);
  Serial.print(F("  Baseline noise: +/- "));
  Serial.print(baselineNoise);
  Serial.print(F("  (range: "));
  Serial.print(bMin);
  Serial.print(F(" - "));
  Serial.print(bMax);
  Serial.print(F(", std: "));
  Serial.print(bStd);
  Serial.println(F(")"));

  // Touch threshold: must be clearly above noise
  touchThreshold = baseline + max(baselineNoise * 2, (long)5) + 3;
  Serial.print(F("  Touch threshold: > "));
  Serial.println(touchThreshold);
  Serial.println();

  // ──────────────────────────────────────────────────────────────────
  // PHASE 4: Per-node calibration
  // ──────────────────────────────────────────────────────────────────
  Serial.println(F("[PHASE 4] Per-node touch calibration."));
  Serial.println(F("  Touch and HOLD each node when prompted."));
  Serial.println(F("  Release only when told."));
  Serial.println();

  for (int i = 0; i < numNodes; i++) {
    Serial.println(F("  ----------------------------------------"));
    Serial.print(F("  >>> TOUCH AND HOLD  Node "));
    Serial.print(i + 1);
    Serial.print(F("  (cumulative: "));
    Serial.print(cumulativeResKohm[i], 1);
    Serial.println(F(" kohm) <<<"));
    Serial.println(F("  Waiting for touch..."));

    // Stream raw values while waiting so user can see what's happening
    int waitCount = 0;
    while (true) {
      long v = cs.capacitiveSensor(CAP_SAMPLES);
      waitCount++;
      if (waitCount % 8 == 0) {
        Serial.print(F("    waiting... raw="));
        Serial.print(v);
        Serial.print(F("  (need > "));
        Serial.print(touchThreshold);
        Serial.println(F(")"));
      }
      if (v > touchThreshold) break;
      delay(20);
    }

    Serial.println(F("  Touch detected! Hold steady... collecting data..."));
    delay(500);  // settle time

    long cMean, cMin, cMax, cStd;
    statsReading(CALIBRATION_READS, cMean, cMin, cMax, cStd);

    nodeCalMean[i] = cMean;
    nodeCalMin[i]  = cMin;
    nodeCalMax[i]  = cMax;
    nodeCalStd[i]  = cStd;

    Serial.print(F("    mean="));
    Serial.print(cMean);
    Serial.print(F("  min="));
    Serial.print(cMin);
    Serial.print(F("  max="));
    Serial.print(cMax);
    Serial.print(F("  std="));
    Serial.println(cStd);

    Serial.println(F("  >>> RELEASE NOW <<<"));
    delay(1000);

    // Wait for confirmed release
    int relCount = 0;
    while (relCount < 8) {
      long v = cs.capacitiveSensor(CAP_SAMPLES);
      if (v < touchThreshold) relCount++;
      else relCount = 0;
      delay(20);
    }
    Serial.println(F("  Released. OK."));
    Serial.println();
  }

  // ──────────────────────────────────────────────────────────────────
  // PHASE 5: Compute thresholds
  // ──────────────────────────────────────────────────────────────────
  Serial.println(F("[PHASE 5] Computing thresholds..."));

  // Sort nodes by calibrated mean (ascending)
  for (int i = 0; i < numNodes; i++) sortIdx[i] = i;
  for (int i = 0; i < numNodes - 1; i++) {
    for (int j = i + 1; j < numNodes; j++) {
      if (nodeCalMean[sortIdx[i]] > nodeCalMean[sortIdx[j]]) {
        int tmp = sortIdx[i];
        sortIdx[i] = sortIdx[j];
        sortIdx[j] = tmp;
      }
    }
  }

  // Thresholds as midpoints between adjacent node means
  for (int rank = 0; rank < numNodes; rank++) {
    int idx = sortIdx[rank];

    if (rank == 0) {
      nodeThresholdLow[idx] = touchThreshold;
    } else {
      int prevIdx = sortIdx[rank - 1];
      long midpoint = (nodeCalMean[prevIdx] + nodeCalMean[idx]) / 2;
      nodeThresholdLow[idx] = midpoint;
      nodeThresholdHigh[prevIdx] = midpoint;
    }

    if (rank == numNodes - 1) {
      nodeThresholdHigh[idx] = nodeCalMean[idx] * 3 + 1000;
    }
  }

  // ── Calibration report ────────────────────────────────────────────
  Serial.println();
  Serial.println(F("  ============ CALIBRATION RESULTS ============"));
  Serial.print(F("  Baseline: "));
  Serial.print(baseline);
  Serial.print(F("  |  Touch threshold: "));
  Serial.println(touchThreshold);
  Serial.println();

  Serial.println(F("  Node | R_cum(k) | Mean   | Min    | Max    | Std  | Thresh lo-hi"));
  Serial.println(F("  -----|----------|--------|--------|--------|------|-------------"));

  for (int rank = 0; rank < numNodes; rank++) {
    int idx = sortIdx[rank];
    Serial.print(F("   "));
    if (idx + 1 < 10) Serial.print(' ');
    Serial.print(idx + 1);
    Serial.print(F("   | "));
    printFloat(cumulativeResKohm[idx], 1, 7);
    Serial.print(F(" | "));
    printLong(nodeCalMean[idx], 6);
    Serial.print(F(" | "));
    printLong(nodeCalMin[idx], 6);
    Serial.print(F(" | "));
    printLong(nodeCalMax[idx], 6);
    Serial.print(F(" | "));
    printLong(nodeCalStd[idx], 4);
    Serial.print(F(" | "));
    printLong(nodeThresholdLow[idx], 6);
    Serial.print(F(" - "));
    printLong(nodeThresholdHigh[idx], 6);
    Serial.println();
  }

  // ── Separation quality ────────────────────────────────────────────
  Serial.println();
  Serial.println(F("  Separation quality:"));
  bool allGood = true;
  for (int rank = 0; rank < numNodes - 1; rank++) {
    int a = sortIdx[rank];
    int b = sortIdx[rank + 1];
    long gap = nodeCalMean[b] - nodeCalMean[a];
    long worstOverlap = nodeCalMax[a] - nodeCalMin[b];

    Serial.print(F("    Node "));
    Serial.print(a + 1);
    Serial.print(F(" -> Node "));
    Serial.print(b + 1);
    Serial.print(F(" :  mean gap="));
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

    // Show as percentage of mean
    if (nodeCalMean[a] > 0) {
      float pct = (float)gap / (float)nodeCalMean[a] * 100.0;
      Serial.print(F("  ("));
      Serial.print(pct, 0);
      Serial.print(F("%)"));
    }
    Serial.println();
  }

  Serial.println();
  if (allGood && numNodes > 1) {
    Serial.println(F("  >> All nodes well-separated. Calibration GOOD."));
  } else if (!allGood) {
    Serial.println(F("  >> WARNING: Overlapping nodes detected!"));
    Serial.println(F("     Try a smaller external resistor, or increase the"));
    Serial.println(F("     resistance delta in Rhino GenerateInternalWire."));
  }

  if (numNodes == 1) {
    Serial.println(F("  >> Single node mode — only touch/no-touch detection."));
  }

  Serial.println();
  calibrated = true;
  Serial.println(F("================================================"));
  Serial.println(F("  LIVE DETECTION ACTIVE"));
  Serial.println(F("  Debug: raw | smooth | delta | node | dist[]"));
  Serial.println(F("================================================"));
  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════
// LOOP — live detection
// ═════════════════════════════════════════════════════════════════════
void loop() {
  if (!calibrated) return;

  long raw = cs.capacitiveSensor(CAP_SAMPLES);

  // Running average
  smoothBuf[smoothIdx] = raw;
  smoothIdx = (smoothIdx + 1) % SMOOTH_SIZE;
  if (!smoothFull && smoothIdx == 0) smoothFull = true;
  int count = smoothFull ? SMOOTH_SIZE : max(smoothIdx, 1);
  long sum = 0;
  for (int i = 0; i < count; i++) sum += smoothBuf[i];
  long smoothed = sum / count;

  // ── Identify node ─────────────────────────────────────────────────
  int detectedNode = -1;

  if (smoothed > touchThreshold) {
    // Try range-based match first
    for (int i = 0; i < numNodes; i++) {
      if (smoothed >= nodeThresholdLow[i] && smoothed < nodeThresholdHigh[i]) {
        detectedNode = i;
        break;
      }
    }
    // Fallback: nearest mean
    if (detectedNode == -1) {
      long bestDist = 999999999L;
      for (int i = 0; i < numNodes; i++) {
        long dist = abs(smoothed - nodeCalMean[i]);
        if (dist < bestDist) {
          bestDist = dist;
          detectedNode = i;
        }
      }
    }
  }

  // ── Debounce state machine ────────────────────────────────────────
  if (currentNode == -1) {
    // Currently no touch
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
    // Currently touching a node
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
      missCounter = 0;  // still on same node
    } else {
      // Switched to a different node without releasing
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

  // ── Debug output (every 200ms) ────────────────────────────────────
  unsigned long now = millis();
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    Serial.print(F("raw="));
    printLong(raw, 6);
    Serial.print(F(" sm="));
    printLong(smoothed, 6);
    Serial.print(F(" d="));
    printLong(smoothed - baseline, 5);

    if (currentNode >= 0) {
      Serial.print(F(" [Node "));
      Serial.print(currentNode + 1);
      Serial.print(F("]"));
    } else {
      Serial.print(F(" [-]"));
    }

    // Distance to each node mean
    Serial.print(F(" dist:"));
    for (int i = 0; i < numNodes; i++) {
      if (i > 0) Serial.print(',');
      long d = smoothed - nodeCalMean[i];
      if (d >= 0) Serial.print('+');
      Serial.print(d);
    }
    Serial.println();
  }

  delay(15);
}
