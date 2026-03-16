/*
 * Touch Calibration & Detection for 3D-Printed Single-Wire Sensing
 * -----------------------------------------------------------------
 * Works with CapacitiveSensorR4 on Arduino Uno R4.
 * Change to Paul Badger's CapacitiveSensor library for Uno R3.
 *
 * WORKFLOW:
 *   1. Power on -> enter number of touch nodes
 *   2. Enter the Rhino-calculated resistance (kohm) for each node
 *   3. Auto-calibration: touch each node when prompted
 *   4. Live detection with raw value debugging
 *
 * HARDWARE: 1 MΩ resistor between SEND_PIN and RECEIVE_PIN.
 *           Conductive wire from RECEIVE_PIN to the printed in-node.
 */

#include <CapacitiveSensorR4.h>

// ── Pin config ──────────────────────────────────────────────────────
#define SEND_PIN    5
#define RECEIVE_PIN 2

// ── Sensing parameters ──────────────────────────────────────────────
#define CAP_SAMPLES        30      // samples per capacitiveSensor() call
#define BASELINE_WINDOW    50      // readings to establish baseline (no touch)
#define CALIBRATION_READS  80      // readings per node during calibration
#define TOUCH_HOLD_MS      3000    // how long user holds each node during cal
#define DEBOUNCE_COUNT     3       // consecutive hits before declaring a touch
#define RELEASE_COUNT      3       // consecutive misses before declaring release
#define MAX_NODES          12      // maximum number of touch nodes supported

// ── Objects ─────────────────────────────────────────────────────────
CapacitiveSensor cs = CapacitiveSensor(SEND_PIN, RECEIVE_PIN);

// ── Node data ───────────────────────────────────────────────────────
int    numNodes = 0;
float  nodeResistanceKohm[MAX_NODES];       // user-entered resistance per node
long   nodeCalMean[MAX_NODES];              // mean sensor reading when touched
long   nodeCalMin[MAX_NODES];               // min seen during calibration
long   nodeCalMax[MAX_NODES];               // max seen during calibration
long   nodeThresholdLow[MAX_NODES];         // lower threshold (enter touch zone)
long   nodeThresholdHigh[MAX_NODES];        // upper threshold (above this = different node)

// ── Runtime state ───────────────────────────────────────────────────
long   baseline          = 0;               // sensor value with no touch
long   baselineNoise     = 0;               // noise amplitude at baseline
long   touchThreshold    = 0;               // global "something is touched" threshold
int    currentNode       = -1;              // currently detected node (-1 = none)
int    hitCounter        = 0;
int    missCounter       = 0;
bool   calibrated        = false;

// ── Helpers ─────────────────────────────────────────────────────────

// Blocking read of a float from Serial.  Ignores blank lines.
float readSerialFloat() {
  while (true) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() == 0) continue;
      float v = s.toFloat();
      if (v == 0.0 && s.charAt(0) != '0') {
        Serial.println(F("[!] Invalid number.  Try again:"));
        continue;
      }
      return v;
    }
  }
}

int readSerialInt() {
  return (int)readSerialFloat();
}

// Take N readings and return the median (more robust than mean).
long medianReading(int n) {
  long buf[100];
  if (n > 100) n = 100;
  for (int i = 0; i < n; i++) {
    buf[i] = cs.capacitiveSensor(CAP_SAMPLES);
    delay(10);
  }
  // simple insertion sort
  for (int i = 1; i < n; i++) {
    long key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = key;
  }
  return buf[n / 2];
}

// Take N readings and compute mean + std deviation.
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
  // std dev
  long sqSum = 0;
  for (int i = 0; i < n; i++) {
    long d = buf[i] - outMean;
    sqSum += d * d;
  }
  outStd = (long)sqrt((double)sqSum / (double)n);
}

// ── SETUP ───────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F(" 3DP Single-Wire Touch Calibration Tool"));
  Serial.println(F("========================================"));
  Serial.println(F(""));

  // ── Step 1: Number of nodes ───────────────────────────────────
  Serial.println(F("[STEP 1] How many touch nodes?"));
  Serial.print(F("  Enter number (1-"));
  Serial.print(MAX_NODES);
  Serial.println(F("):"));

  numNodes = readSerialInt();
  if (numNodes < 1) numNodes = 1;
  if (numNodes > MAX_NODES) numNodes = MAX_NODES;

  Serial.print(F("  -> "));
  Serial.print(numNodes);
  Serial.println(F(" node(s) configured."));
  Serial.println();

  // ── Step 2: Resistance values from Rhino ──────────────────────
  Serial.println(F("[STEP 2] Enter resistance values from Rhino command line."));
  Serial.println(F("  These are the segment resistances (kohm) between consecutive"));
  Serial.println(F("  nodes as reported by the GenerateInternalWire command."));
  Serial.println(F("  (Cumulative resistance from in-node to each touch node.)"));
  Serial.println();

  for (int i = 0; i < numNodes; i++) {
    Serial.print(F("  Node "));
    Serial.print(i + 1);
    Serial.print(F(" resistance (kohm): "));
    nodeResistanceKohm[i] = readSerialFloat();
    Serial.print(F("    -> "));
    Serial.print(nodeResistanceKohm[i], 1);
    Serial.println(F(" kohm"));
  }

  Serial.println();
  Serial.println(F("  Resistance summary:"));
  for (int i = 0; i < numNodes; i++) {
    Serial.print(F("    Node "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(nodeResistanceKohm[i], 1);
    Serial.println(F(" kohm"));
  }
  Serial.println();

  // ── Step 3: Baseline measurement ──────────────────────────────
  Serial.println(F("[STEP 3] Baseline measurement."));
  Serial.println(F("  *** DO NOT TOUCH the object! ***"));
  Serial.println(F("  Measuring in 2 seconds..."));
  delay(2000);

  long bMean, bMin, bMax, bStd;
  statsReading(BASELINE_WINDOW, bMean, bMin, bMax, bStd);
  baseline = bMean;
  baselineNoise = max(bStd * 3, bMax - bMean);  // 3-sigma or peak noise

  Serial.print(F("  Baseline: "));
  Serial.print(baseline);
  Serial.print(F("  noise(3σ): "));
  Serial.print(bStd * 3);
  Serial.print(F("  peak: "));
  Serial.print(bMax);
  Serial.print(F("  range: ["));
  Serial.print(bMin);
  Serial.print(F(" - "));
  Serial.print(bMax);
  Serial.println(F("]"));

  touchThreshold = baseline + baselineNoise * 2 + 5;  // generous margin
  Serial.print(F("  Touch detection threshold: > "));
  Serial.println(touchThreshold);
  Serial.println();

  // ── Step 4: Per-node calibration ──────────────────────────────
  Serial.println(F("[STEP 4] Touch calibration — one node at a time."));
  Serial.println(F("  When prompted, TOUCH and HOLD the specified node"));
  Serial.print(F("  for ~"));
  Serial.print(TOUCH_HOLD_MS / 1000);
  Serial.println(F(" seconds.  Release when told."));
  Serial.println();

  for (int i = 0; i < numNodes; i++) {
    Serial.println(F("  ──────────────────────────────────"));
    Serial.print(F("  >>> TOUCH Node "));
    Serial.print(i + 1);
    Serial.print(F(" ("));
    Serial.print(nodeResistanceKohm[i], 1);
    Serial.println(F(" kohm) NOW <<<"));
    Serial.println(F("  Waiting for touch..."));

    // Wait until we see a reading above touchThreshold
    while (true) {
      long v = cs.capacitiveSensor(CAP_SAMPLES);
      if (v > touchThreshold) break;
      delay(20);
    }

    Serial.println(F("  Touch detected!  Collecting data..."));
    delay(300);  // settle

    long cMean, cMin, cMax, cStd;
    statsReading(CALIBRATION_READS, cMean, cMin, cMax, cStd);

    nodeCalMean[i] = cMean;
    nodeCalMin[i]  = cMin;
    nodeCalMax[i]  = cMax;

    Serial.print(F("    mean="));
    Serial.print(cMean);
    Serial.print(F("  min="));
    Serial.print(cMin);
    Serial.print(F("  max="));
    Serial.print(cMax);
    Serial.print(F("  std="));
    Serial.println(cStd);

    Serial.println(F("  >>> RELEASE now <<<"));
    delay(1500);

    // Wait for release
    int releaseCount = 0;
    while (releaseCount < 5) {
      long v = cs.capacitiveSensor(CAP_SAMPLES);
      if (v < touchThreshold) releaseCount++;
      else releaseCount = 0;
      delay(20);
    }
    Serial.println(F("  Released."));
    Serial.println();
  }

  // ── Step 5: Compute thresholds ────────────────────────────────
  Serial.println(F("[STEP 5] Computing thresholds..."));
  Serial.println();

  // Sort nodes by calibrated mean value for threshold assignment.
  // We create an index array sorted by nodeCalMean.
  int sortIdx[MAX_NODES];
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

  // Assign thresholds as midpoints between adjacent node means,
  // but also respect the min/max ranges seen during calibration.
  for (int rank = 0; rank < numNodes; rank++) {
    int idx = sortIdx[rank];

    // Lower bound
    if (rank == 0) {
      // First node: lower bound is just above touch threshold
      nodeThresholdLow[idx] = touchThreshold;
    } else {
      int prevIdx = sortIdx[rank - 1];
      // Midpoint between this node's min and previous node's max
      long midpoint = (nodeCalMean[prevIdx] + nodeCalMean[idx]) / 2;
      nodeThresholdLow[idx] = midpoint;
      nodeThresholdHigh[prevIdx] = midpoint;
    }

    // Upper bound for last node
    if (rank == numNodes - 1) {
      nodeThresholdHigh[idx] = nodeCalMean[idx] * 3;  // generous upper bound
    }
  }

  // ── Print calibration results ─────────────────────────────────
  Serial.println(F("  ╔═══════════════════════════════════════════════════════════════╗"));
  Serial.println(F("  ║              CALIBRATION RESULTS                             ║"));
  Serial.println(F("  ╠═══════════════════════════════════════════════════════════════╣"));

  Serial.print(F("  ║ Baseline: "));
  Serial.print(baseline);
  Serial.print(F("  |  Touch threshold: "));
  Serial.print(touchThreshold);
  printPadded(55);
  Serial.println(F("║"));

  Serial.println(F("  ╠═══════════════════════════════════════════════════════════════╣"));
  Serial.println(F("  ║ Node | R(kohm) |  Mean  |  Min   |  Max   | Thresh [lo-hi]  ║"));
  Serial.println(F("  ╠═══════════════════════════════════════════════════════════════╣"));

  for (int rank = 0; rank < numNodes; rank++) {
    int idx = sortIdx[rank];
    Serial.print(F("  ║  "));
    if (idx + 1 < 10) Serial.print(' ');
    Serial.print(idx + 1);
    Serial.print(F("  | "));
    printFloat(nodeResistanceKohm[idx], 1, 6);
    Serial.print(F(" | "));
    printLong(nodeCalMean[idx], 6);
    Serial.print(F(" | "));
    printLong(nodeCalMin[idx], 6);
    Serial.print(F(" | "));
    printLong(nodeCalMax[idx], 6);
    Serial.print(F(" | "));
    printLong(nodeThresholdLow[idx], 6);
    Serial.print(F("-"));
    printLong(nodeThresholdHigh[idx], 6);
    Serial.println(F("  ║"));
  }

  Serial.println(F("  ╚═══════════════════════════════════════════════════════════════╝"));
  Serial.println();

  // ── Separation quality report ─────────────────────────────────
  Serial.println(F("  Separation quality:"));
  bool allGood = true;
  for (int rank = 0; rank < numNodes - 1; rank++) {
    int a = sortIdx[rank];
    int b = sortIdx[rank + 1];
    long gap = nodeCalMean[b] - nodeCalMean[a];
    long overlap = nodeCalMax[a] - nodeCalMin[b];
    Serial.print(F("    Node "));
    Serial.print(a + 1);
    Serial.print(F(" <-> Node "));
    Serial.print(b + 1);
    Serial.print(F(": gap="));
    Serial.print(gap);
    if (overlap > 0) {
      Serial.print(F("  *** OVERLAP="));
      Serial.print(overlap);
      Serial.print(F(" WARNING ***"));
      allGood = false;
    } else {
      Serial.print(F("  margin="));
      Serial.print(-overlap);
      Serial.print(F("  OK"));
    }
    Serial.println();
  }

  if (allGood) {
    Serial.println(F("  -> All nodes are well-separated. Calibration looks GOOD."));
  } else {
    Serial.println(F("  -> WARNING: Some nodes overlap! Consider increasing resistance"));
    Serial.println(F("     delta in Rhino or adjusting node geometry."));
  }
  Serial.println();

  calibrated = true;
  Serial.println(F("========================================"));
  Serial.println(F(" LIVE DETECTION ACTIVE"));
  Serial.println(F(" Format: raw | smoothed | node"));
  Serial.println(F("========================================"));
  Serial.println();
}

// ── LOOP — live touch detection ─────────────────────────────────
// Running average for smoothing
#define SMOOTH_SIZE 5
long smoothBuf[SMOOTH_SIZE];
int  smoothIdx = 0;
bool smoothFull = false;
unsigned long lastPrintMs = 0;

void loop() {
  if (!calibrated) return;

  long raw = cs.capacitiveSensor(CAP_SAMPLES);

  // Update running average
  smoothBuf[smoothIdx] = raw;
  smoothIdx = (smoothIdx + 1) % SMOOTH_SIZE;
  if (smoothIdx == 0) smoothFull = true;

  int count = smoothFull ? SMOOTH_SIZE : smoothIdx;
  long sum = 0;
  for (int i = 0; i < count; i++) sum += smoothBuf[i];
  long smoothed = sum / count;

  // Determine which node (if any)
  int detectedNode = -1;
  if (smoothed > touchThreshold) {
    // Find the node whose threshold range contains this reading
    long bestDist = 999999999L;
    for (int i = 0; i < numNodes; i++) {
      if (smoothed >= nodeThresholdLow[i] && smoothed < nodeThresholdHigh[i]) {
        // Exact range match
        long dist = abs(smoothed - nodeCalMean[i]);
        if (dist < bestDist) {
          bestDist = dist;
          detectedNode = i;
        }
      }
    }
    // Fallback: if no range matched, find nearest node
    if (detectedNode == -1) {
      for (int i = 0; i < numNodes; i++) {
        long dist = abs(smoothed - nodeCalMean[i]);
        if (dist < bestDist) {
          bestDist = dist;
          detectedNode = i;
        }
      }
    }
  }

  // Debounce
  if (detectedNode == currentNode) {
    hitCounter = min(hitCounter + 1, DEBOUNCE_COUNT + 1);
    missCounter = 0;
  } else if (detectedNode == -1 && currentNode != -1) {
    missCounter++;
    if (missCounter >= RELEASE_COUNT) {
      Serial.print(F("[RELEASE] Node "));
      Serial.println(currentNode + 1);
      currentNode = -1;
      hitCounter = 0;
      missCounter = 0;
    }
  } else if (detectedNode != -1 && detectedNode != currentNode) {
    hitCounter++;
    if (hitCounter >= DEBOUNCE_COUNT) {
      if (currentNode != -1) {
        Serial.print(F("[RELEASE] Node "));
        Serial.println(currentNode + 1);
      }
      currentNode = detectedNode;
      hitCounter = 0;
      missCounter = 0;
      Serial.print(F(">>> Node "));
      Serial.print(currentNode + 1);
      Serial.println(F(" PRESSED <<<"));
    }
  } else {
    hitCounter = 0;
    missCounter = 0;
  }

  // ── Debug output (throttled to every 150ms) ───────────────────
  unsigned long now = millis();
  if (now - lastPrintMs >= 150) {
    lastPrintMs = now;

    Serial.print(F("raw="));
    printLong(raw, 6);
    Serial.print(F(" | smooth="));
    printLong(smoothed, 6);
    Serial.print(F(" | base="));
    printLong(baseline, 5);
    Serial.print(F(" | delta="));
    printLong(smoothed - baseline, 6);

    if (currentNode >= 0) {
      Serial.print(F(" | ** Node "));
      Serial.print(currentNode + 1);
      Serial.print(F(" **"));
    } else {
      Serial.print(F(" | (none)"));
    }

    // Show distance to each node's mean for troubleshooting
    Serial.print(F(" | dist:["));
    for (int i = 0; i < numNodes; i++) {
      if (i > 0) Serial.print(',');
      Serial.print(smoothed - nodeCalMean[i]);
    }
    Serial.print(']');

    Serial.println();
  }

  delay(15);
}

// ── Formatting helpers ──────────────────────────────────────────
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

void printPadded(int targetCol) {
  // placeholder — just print spaces to roughly align
  Serial.print(F("  "));
}
