// Pin definitions
const int SEND_PIN = 5;
const int RECEIVE_START_PIN = 2; // Connected to Node 1 side
const int RECEIVE_END_PIN = 3;   // Connected to Node 4 side

// Sensor configuration variables
const int MAX_NODES = 20;
int numNodes = 0;

// Calibration variables
long baselineTotal = 0;          
long nodeRatios[MAX_NODES];      
long touchThreshold = 20; // Increased slightly for stability

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); } 
  
  Serial.println("=========================================");
  Serial.println(" UNO R4 Two-Ended Ratio Touch Calibrator ");
  Serial.println("=========================================");

  Serial.println("\nHow many touch nodes are on your trace? (Max 20)");
  numNodes = waitForInt();
  if (numNodes > MAX_NODES) numNodes = MAX_NODES;
  if (numNodes < 1) numNodes = 1;
  Serial.print("Registered Nodes: ");
  Serial.println(numNodes);

  Serial.println("\n--- CALIBRATION ---");
  Serial.println("Step 1: DO NOT touch anything.");
  Serial.println("Type 'c' and hit Enter to calibrate the baseline.");
  waitForChar('c');
  
  long t2, t3;
  getAverageDualReading(50, t2, t3);
  baselineTotal = t2 + t3; 
  
  Serial.print("Baseline Total Capacitance set to: ");
  Serial.println(baselineTotal);

  for (int i = 0; i < numNodes; i++) {
    Serial.print("\nStep ");
    Serial.print(i + 2);
    Serial.print(": Touch Node ");
    Serial.print(i + 1);
    Serial.println(", hold your finger firmly there, type 'c', and hit Enter.");
    
    waitForChar('c');
    
    getAverageDualReading(50, t2, t3);
    
    Serial.print("Raw Loop Counts -> Pin 2: ");
    Serial.print(t2);
    Serial.print(" | Pin 3: ");
    Serial.println(t3);

    if (t3 == 0) t3 = 1; // Prevent division by zero
    nodeRatios[i] = (t2 * 1000) / t3; 
    
    Serial.print("Node ");
    Serial.print(i + 1);
    Serial.print(" calibrated to Ratio: ");
    Serial.println(nodeRatios[i]);
  }

  Serial.println("\n=========================================");
  Serial.println(" Calibration Complete! Running sensor... ");
  Serial.println("=========================================");
  delay(1000);
}

void loop() {
  long t2, t3;
  getAverageDualReading(30, t2, t3);
  long currentTotal = t2 + t3;

  if (currentTotal > baselineTotal + touchThreshold) {
    if (t3 == 0) t3 = 1;
    long currentRatio = (t2 * 1000) / t3;

    int closestNode = -1;
    long smallestDifference = 2147483647; 

    for (int i = 0; i < numNodes; i++) {
      long difference = abs(currentRatio - nodeRatios[i]);
      if (difference < smallestDifference) {
        smallestDifference = difference;
        closestNode = i + 1;
      }
    }

    Serial.print("Touched! Total Cap: ");
    Serial.print(currentTotal);
    Serial.print(" | Current Ratio: ");
    Serial.print(currentRatio);
    Serial.print(" | Detected: Node ");
    Serial.println(closestNode);
    
    delay(250); 
  }
}

// ==========================================
// Custom Dual Capacitive Sensing Engine
// ==========================================

void readDualCapacitance(long &time2, long &time3) {
  // Initialize with massive numbers. If a pin times out, it won't crash the math.
  time2 = 20000;
  time3 = 20000;

  // 1. Discharge all pins to ground (increased delay to clear capacitors)
  pinMode(SEND_PIN, OUTPUT);
  pinMode(RECEIVE_START_PIN, OUTPUT);
  pinMode(RECEIVE_END_PIN, OUTPUT);
  digitalWrite(SEND_PIN, LOW);
  digitalWrite(RECEIVE_START_PIN, LOW);
  digitalWrite(RECEIVE_END_PIN, LOW);
  delayMicroseconds(500); 
  
  // 2. Isolate the receive pins 
  pinMode(RECEIVE_START_PIN, INPUT);
  pinMode(RECEIVE_END_PIN, INPUT);
  
  // 3. Send the charge pulse
  digitalWrite(SEND_PIN, HIGH);
  
  // 4. Measure
  // 'volatile' prevents the compiler from compressing this loop, ensuring slower, more accurate counts
  volatile long count = 0; 
  bool p2_done = false;
  bool p3_done = false;
  
  while ((!p2_done || !p3_done) && count < 20000) {
    count++;
    
    if (!p2_done && digitalRead(RECEIVE_START_PIN) == HIGH) {
      time2 = count;
      p2_done = true;
    }
    if (!p3_done && digitalRead(RECEIVE_END_PIN) == HIGH) {
      time3 = count;
      p3_done = true;
    }
  }
  
  // 5. Discharge again 
  digitalWrite(SEND_PIN, LOW);
}

// ==========================================
// Helper Functions
// ==========================================

void getAverageDualReading(int samples, long &avgT2, long &avgT3) {
  long totalT2 = 0;
  long totalT3 = 0;
  long currentT2 = 0;
  long currentT3 = 0;
  
  for (int i = 0; i < samples; i++) {
    readDualCapacitance(currentT2, currentT3);
    totalT2 += currentT2;
    totalT3 += currentT3;
  }
  
  avgT2 = totalT2 / samples;
  avgT3 = totalT3 / samples;
}

long waitForInt() {
  while (Serial.available() == 0) { } 
  long val = Serial.parseInt();
  while (Serial.available() > 0) { Serial.read(); delay(2); }
  return val;
}

void waitForChar(char target) {
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == target || c == toupper(target)) {
        while (Serial.available() > 0) { Serial.read(); delay(2); }
        return;
      }
    }
  }
}