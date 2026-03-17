/*
 * 4-Node Single-Wire Touch Sensor
 * --------------------------------
 * Pre-configured for a 3D-printed conductive trace with:
 *   - 4 touch nodes
 *   - Total path resistance: ~34 kohm
 *   - Cumulative resistances from start: 4k, 15k, 22k, 32k ohm
 *   - Node-to-node deltas: 4k, 11k, 7k, 10k ohm
 *
 * HARDWARE:
 *   - Arduino Uno R4
 *   - 10k external resistor between Pin 5 (send) and Pin 2 (receive)
 *   - Wire from Pin 2 junction to START of conductive trace
 *
 * Requires CapacitiveSensorR4 library in:
 *   arduino/libraries/CapacitiveSensorUnoR4/
 */

#include <CapacitiveSensorR4.h>

// ── Pin config ──────────────────────────────────────────────────────
#define SEND_PIN    5
#define RECEIVE_PIN 2

// ── Your trace parameters ───────────────────────────────────────────
#define NUM_NODES 4
const float cumulativeResKohm[NUM_NODES] = {4.0, 15.0, 22.0, 32.0};

// ── Sensing parameters ──────────────────────────────────────────────
#define CAP_SAMPLES        30   // samples per capacitiveSensor() call
#define BASELINE_READS     50   // readings to establish baseline
#define CALIBRATION_READS  60   // readings per node during calibration
#define DEBOUNCE_COUNT     3    // consecutive hits to confirm touch
#define RELEASE_COUNT      4    // consecutive misses to confirm release
#define SMOOTH_SIZE        5    // running average window

// ── Sensor object ───────────────────────────────────────────────────
CapacitiveSensor cs = CapacitiveSensor(SEND_PIN, RECEIVE_PIN);

// ── Calibration data ────────────────────────────────────────────────
long   nodeCalMean[NUM_NODES];
long   nodeCalMin[NUM_NODES];
long   nodeCalMax[NUM_NODES];
long   nodeCalStd[NUM_NODES];
long   nodeThresholdLow[NUM_NODES];
long   nodeThresholdHigh[NUM_NODES];
int    sortIdx[NUM_NODES];

// ── Runtime state ───────────────────────────────────────────────────
long   baseline       = 0;
long   baselineNoise  = 0;
long   touchThreshold = 0;
int    currentNode    = -1;
int    candidateNode  = -1;
int    hitCounter     = 0;
int    missCounter    = 0;
bool   calibrated     = false;

// ── Smoothing ───────────────────────────────────────────────────────
long   smoothBuf[SMOOTH_SIZE];
int    smoothIdx  = 0;
bool   smoothFull = false;
unsigned long lastPrintMs = 0;

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
  Serial.println(F("  4-Node Single-Wire Touch Sensor"));
  Serial.println(F("  Trace: 4k - 15k - 22k - 32k ohm cumulative"));
  Serial.println(F("================================================"));
  Serial.println();

  // ── Print trace layout ──────────────────────────────────────────
  Serial.println(F("  Your trace layout:"));
  Serial.println(F("  START -[4k]- N1 -[11k]- N2 -[7k]- N3 -[10k]- N4"));
  Serial.println();

  // ── PHASE 0: Quick signal test ──────────────────────────────────
  Serial.println(F("[PHASE 0] Quick signal test"));
  Serial.println(F("  DON'T TOUCH for 3 seconds..."));

  long noTouchMax = 0;
  unsigned long startMs = millis();
  while (millis() - startMs < 3000) {
    long v = cs.capacitiveSensor(CAP_SAMPLES);
    if (v > noTouchMax) noTouchMax = v;
    delay(20);
  }

  Serial.println(F("  NOW TOUCH any node for 3 seconds..."));
  long touchMax = 0;
  startMs = millis();
  while (millis() - startMs < 3000) {
    long v = cs.capacitiveSensor(CAP_SAMPLES);
    if (v > touchMax) touchMax = v;
    delay(20);
  }

  long delta = touchMax - noTouchMax;
  Serial.print(F("  No-touch peak: ")); Serial.println(noTouchMax);
  Serial.print(F("  Touch peak:    ")); Serial.println(touchMax);
  Serial.print(F("  Delta:         ")); Serial.println(delta);

  if (delta < 5) {
    Serial.println(F("  *** WARNING: Very weak signal! Check wiring: ***"));
    Serial.println(F("    - 10k resistor between pin 5 and pin 2"));
    Serial.println(F("    - Wire from pin 2 to START of conductive trace"));
  } else if (delta < 50) {
    Serial.println(F("  Signal weak but detectable."));
  } else {
    Serial.println(F("  Signal looks good!"));
  }
  Serial.println();

  // ── PHASE 1: Baseline ──────────────────────────────────────────
  Serial.println(F("[PHASE 1] Baseline measurement"));
  Serial.println(F("  HANDS OFF! Measuring in 2 seconds..."));
  delay(2000);

  long bMean, bMin, bMax, bStd;
  statsReading(BASELINE_READS, bMean, bMin, bMax, bStd);
  baseline = bMean;
  baselineNoise = max(bStd * 3, bMax - bMean);
  touchThreshold = baseline + max(baselineNoise * 2, (long)5) + 3;

  Serial.print(F("  Baseline: ")); Serial.println(baseline);
  Serial.print(F("  Noise: +/- ")); Serial.println(baselineNoise);
  Serial.print(F("  Touch threshold: > ")); Serial.println(touchThreshold);
  Serial.println();

  // ── PHASE 2: Per-node calibration ──────────────────────────────
  Serial.println(F("[PHASE 2] Per-node calibration"));
  Serial.println(F("  Touch each node when prompted. Hold until told to release."));
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

    Serial.println(F("  Touch detected! Hold steady..."));
    delay(500);  // settle

    long cMean, cMin, cMax, cStd;
    statsReading(CALIBRATION_READS, cMean, cMin, cMax, cStd);

    nodeCalMean[i] = cMean;
    nodeCalMin[i]  = cMin;
    nodeCalMax[i]  = cMax;
    nodeCalStd[i]  = cStd;

    Serial.print(F("    mean=")); Serial.print(cMean);
    Serial.print(F("  min=")); Serial.print(cMin);
    Serial.print(F("  max=")); Serial.print(cMax);
    Serial.print(F("  std=")); Serial.println(cStd);

    Serial.println(F("  >>> RELEASE NOW <<<"));
    delay(1000);

    // Wait for release
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

  // ── PHASE 3: Compute thresholds ────────────────────────────────
  Serial.println(F("[PHASE 3] Computing thresholds..."));

  // Sort by calibrated mean (ascending)
  for (int i = 0; i < NUM_NODES; i++) sortIdx[i] = i;
  for (int i = 0; i < NUM_NODES - 1; i++) {
    for (int j = i + 1; j < NUM_NODES; j++) {
      if (nodeCalMean[sortIdx[i]] > nodeCalMean[sortIdx[j]]) {
        int tmp = sortIdx[i];
        sortIdx[i] = sortIdx[j];
        sortIdx[j] = tmp;
      }
    }
  }

  // Thresholds as midpoints between adjacent means
  for (int rank = 0; rank < NUM_NODES; rank++) {
    int idx = sortIdx[rank];
    if (rank == 0) {
      nodeThresholdLow[idx] = touchThreshold;
    } else {
      int prevIdx = sortIdx[rank - 1];
      long midpoint = (nodeCalMean[prevIdx] + nodeCalMean[idx]) / 2;
      nodeThresholdLow[idx] = midpoint;
      nodeThresholdHigh[prevIdx] = midpoint;
    }
    if (rank == NUM_NODES - 1) {
      nodeThresholdHigh[idx] = nodeCalMean[idx] * 3 + 1000;
    }
  }

  // ── Print calibration report ───────────────────────────────────
  Serial.println();
  Serial.println(F("  ============ CALIBRATION RESULTS ============"));
  Serial.print(F("  Baseline: ")); Serial.print(baseline);
  Serial.print(F("  |  Touch threshold: ")); Serial.println(touchThreshold);
  Serial.println();

  Serial.println(F("  Node | R_cum(k) | Mean   | Std  | Thresh lo-hi"));
  Serial.println(F("  -----|----------|--------|------|-------------"));

  for (int rank = 0; rank < NUM_NODES; rank++) {
    int idx = sortIdx[rank];
    Serial.print(F("   "));
    if (idx + 1 < 10) Serial.print(' ');
    Serial.print(idx + 1);
    Serial.print(F("   | "));
    printFloat(cumulativeResKohm[idx], 0, 7);
    Serial.print(F(" | "));
    printLong(nodeCalMean[idx], 6);
    Serial.print(F(" | "));
    printLong(nodeCalStd[idx], 4);
    Serial.print(F(" | "));
    printLong(nodeThresholdLow[idx], 6);
    Serial.print(F(" - "));
    printLong(nodeThresholdHigh[idx], 6);
    Serial.println();
  }

  // Separation quality
  Serial.println();
  Serial.println(F("  Separation quality:"));
  bool allGood = true;
  for (int rank = 0; rank < NUM_NODES - 1; rank++) {
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
    Serial.println();
  }

  Serial.println();
  if (allGood) {
    Serial.println(F("  >> All nodes well-separated. GOOD!"));
  } else {
    Serial.println(F("  >> WARNING: Overlapping nodes! Try smaller external resistor."));
  }

  calibrated = true;
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("  LIVE DETECTION ACTIVE"));
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

  // ── Identify node ─────────────────────────────────────────────
  int detectedNode = -1;

  if (smoothed > touchThreshold) {
    // Range-based match
    for (int i = 0; i < NUM_NODES; i++) {
      if (smoothed >= nodeThresholdLow[i] && smoothed < nodeThresholdHigh[i]) {
        detectedNode = i;
        break;
      }
    }
    // Fallback: nearest mean
    if (detectedNode == -1) {
      long bestDist = 999999999L;
      for (int i = 0; i < NUM_NODES; i++) {
        long dist = abs(smoothed - nodeCalMean[i]);
        if (dist < bestDist) {
          bestDist = dist;
          detectedNode = i;
        }
      }
    }
  }

  // ── Debounce ──────────────────────────────────────────────────
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
    for (int i = 0; i < NUM_NODES; i++) {
      if (i > 0) Serial.print(',');
      long d = smoothed - nodeCalMean[i];
      if (d >= 0) Serial.print('+');
      Serial.print(d);
    }
    Serial.println();
  }

  delay(15);
}
