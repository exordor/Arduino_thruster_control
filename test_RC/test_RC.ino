// Simple RC range measurement; no external libs required

// Pins
const int CH_RIGHT_IN = 2;  // Pin 2 → Right stick
const int CH_LEFT_IN  = 3;  // Pin 3 → Left stick

// Capture state
volatile unsigned long rRiseMicros = 0, rPulseMicros = 0;
volatile unsigned long lRiseMicros = 0, lPulseMicros = 0;

// Measured ranges
unsigned long rMin = 3000, rMax = 0, rLast = 0;
unsigned long lMin = 3000, lMax = 0, lLast = 0;

// Calibration from your measurements
const int R_CAL_MIN = 1435;
const int R_CAL_MID = 1493;  // observed neutral cluster ~1493
const int R_CAL_MAX = 1994;
const int L_CAL_MIN = 1498;  // left min cluster ~1498–1504
const int L_CAL_MID = 1529;  // neutral cluster ~1516–1529
const int L_CAL_MAX = 2006;

static inline int normalizePiecewise(int raw, int minV, int midV, int maxV) {
  if (raw <= midV) {
    long span = (long)midV - minV;
    if (span < 1) span = 1;
    long pos = (long)raw - minV;
    long out = 1000 + (pos * 500) / span;  // min→mid maps to 1000→1500
    if (out < 1000) out = 1000;
    if (out > 1500) out = 1500;
    return (int)out;
  } else {
    long span = (long)maxV - midV;
    if (span < 1) span = 1;
    long pos = (long)raw - midV;
    long out = 1500 + (pos * 500) / span;  // mid→max maps to 1500→2000
    if (out < 1500) out = 1500;
    if (out > 2000) out = 2000;
    return (int)out;
  }
}

// Valid window to reject noise
const int VALID_MIN = 800;
const int VALID_MAX = 2200;

void onRightChange() {
  int level = digitalRead(CH_RIGHT_IN);
  unsigned long now = micros();
  if (level == HIGH) {
    rRiseMicros = now;
  } else {
    unsigned long width = now - rRiseMicros;
    if (width >= VALID_MIN && width <= VALID_MAX) {
      rPulseMicros = width;
    }
  }
}

void onLeftChange() {
  int level = digitalRead(CH_LEFT_IN);
  unsigned long now = micros();
  if (level == HIGH) {
    lRiseMicros = now;
  } else {
    unsigned long width = now - lRiseMicros;
    if (width >= VALID_MIN && width <= VALID_MAX) {
      lPulseMicros = width;
    }
  }
}

void setup() {
  pinMode(CH_RIGHT_IN, INPUT);
  pinMode(CH_LEFT_IN, INPUT);

  Serial.begin(115200);
  while (!Serial) {}

  attachInterrupt(digitalPinToInterrupt(CH_RIGHT_IN), onRightChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_LEFT_IN),  onLeftChange,  CHANGE);

  Serial.println("=== RC Range Measurement ===");
  Serial.println("Pins: Right=2, Left=3");
  Serial.println("Move sticks to extremes; ranges print every second.");
}

void loop() {
  // Read latest pulses (non-blocking)
  noInterrupts();
  unsigned long inR = rPulseMicros;
  unsigned long inL = lPulseMicros;
  interrupts();

  // Track min/max and last
  if (inR >= VALID_MIN && inR <= VALID_MAX) {
    rLast = inR;
    if (inR < rMin) rMin = inR;
    if (inR > rMax) rMax = inR;
  }
  if (inL >= VALID_MIN && inL <= VALID_MAX) {
    lLast = inL;
    if (inL < lMin) lMin = inL;
    if (inL > lMax) lMax = inL;
  }

  // Compute normalized values
  int normR = 1500, normL = 1500;
  if (rLast) normR = normalizePiecewise((int)rLast, R_CAL_MIN, R_CAL_MID, R_CAL_MAX);
  if (lLast) normL = normalizePiecewise((int)lLast, L_CAL_MIN, L_CAL_MID, L_CAL_MAX);

  // Print once per second
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.print("Right: raw="); Serial.print(rLast);
    Serial.print(" us, min="); Serial.print(rMin);
    Serial.print(" us, max="); Serial.print(rMax);
    Serial.print(" us, norm="); Serial.print(normR);

    Serial.print(" | Left: raw="); Serial.print(lLast);
    Serial.print(" us, min="); Serial.print(lMin);
    Serial.print(" us, max="); Serial.print(lMax);
    Serial.print(" us, norm="); Serial.println(normL);
  }

  delay(5);
}
