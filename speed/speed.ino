// D7 polling version (UNO R4): count level changes via digitalRead()
// pulses/s ≈ changes/s / 2
// Q(L/min) = (pulses/s) / 5
// total L += pulses / 300

const byte sensorPin = 7;

const float K_HZ_PER_LMIN = 5.0f;     // f = 5*Q
const float PULSES_PER_L  = 300.0f;   // 1L ≈ 300 pulses

const float DIAMETER_M = 0.026f; // 26 mm
const float pipeArea = 3.1415926f * (DIAMETER_M * 0.5f) * (DIAMETER_M * 0.5f);

unsigned long lastMs = 0;
double totalLiters = 0.0;

int lastState;
unsigned long changeCount = 0;

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT_PULLUP);
  lastState = digitalRead(sensorPin);
  lastMs = millis();

  Serial.println("D7 polling flow meter");
  Serial.println("dt(ms), changes, f(Hz), Q(L/min), v(m/s), total(L)");
}

void loop() {
  // fast polling
  int s = digitalRead(sensorPin);
  if (s != lastState) {
    changeCount++;
    lastState = s;
  }

  unsigned long now = millis();
  if (now - lastMs >= 1000) {
    unsigned long dtMs = now - lastMs;
    float dtS = dtMs / 1000.0f;

    unsigned long changes = changeCount;
    changeCount = 0;

    // each pulse produces 2 changes (HIGH->LOW->HIGH)
    float freqHz = (dtS > 0) ? ((changes / 2.0f) / dtS) : 0.0f; // ← 這就是頻率
    float flowLmin = freqHz / K_HZ_PER_LMIN;

    float flow_m3s = (flowLmin * 0.001f) / 60.0f;
    float velocity = (pipeArea > 0) ? (flow_m3s / pipeArea) : 0.0f;

    // pulses in this window:
    double pulsesThisWindow = (double)changes / 2.0;
    totalLiters += pulsesThisWindow / (double)PULSES_PER_L;

    Serial.print(dtMs); Serial.print(", ");
    Serial.print(changes); Serial.print(", ");
    Serial.print(freqHz, 2); Serial.print(", ");   // ← 頻率輸出
    Serial.print(flowLmin, 2); Serial.print(", ");
    Serial.print(velocity, 4); Serial.print(", ");
    Serial.println(totalLiters, 3);

    lastMs = now;
  }
}

