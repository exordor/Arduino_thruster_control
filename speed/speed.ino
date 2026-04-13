// Dual-pin diagnostic sketch for YF-S403 on Arduino UNO R4.
// YF-S403 is normally a 3-wire, single-output flow sensor:
//   red   -> 5V
//   black -> GND
//   yellow-> signal
//
// This sketch watches both D6 and D7 so we can quickly confirm which pin
// actually carries the pulse train and whether the idle level is stable.
//
// Some YF-S403 documents disagree on the transfer function:
//   f(Hz) = 5.0 * Q(L/min)
//   f(Hz) = 7.5 * Q(L/min)
// So this diagnostic sketch reports frequency first, plus both candidate
// flow estimates. Once wiring is confirmed, calibrate with a known volume.

const byte SENSOR_PIN_6 = 6;
const byte SENSOR_PIN_7 = 7;

const unsigned long REPORT_INTERVAL_MS = 1000;
const float K_HZ_PER_LMIN_OPTION_A = 5.0f;
const float K_HZ_PER_LMIN_OPTION_B = 7.5f;

volatile unsigned long totalEdgeCount6 = 0;
volatile unsigned long totalEdgeCount7 = 0;
volatile unsigned long lastEdge6Us = 0;
volatile unsigned long lastEdge7Us = 0;

unsigned long lastReportMs = 0;
unsigned long lastSnapshotEdges6 = 0;
unsigned long lastSnapshotEdges7 = 0;

void onPin6Change() {
  totalEdgeCount6++;
  lastEdge6Us = micros();
}

void onPin7Change() {
  totalEdgeCount7++;
  lastEdge7Us = micros();
}

const char* levelText(int level) {
  return (level == HIGH) ? "HIGH" : "LOW";
}

const char* inferActivePin(unsigned long edges6, unsigned long edges7) {
  if (edges6 == 0 && edges7 == 0) return "none";
  if (edges6 > 0 && edges7 == 0) return "D6";
  if (edges7 > 0 && edges6 == 0) return "D7";
  if (abs((long)edges6 - (long)edges7) <= 2) return "both_same_signal?";
  return "both_active_or_noisy";
}

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_PIN_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_7, INPUT_PULLUP);

  delay(20);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_6), onPin6Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_7), onPin7Change, CHANGE);

  lastReportMs = millis();

  Serial.println();
  Serial.println("YF-S403 dual-pin diagnostic");
  Serial.println("Expected wiring: red=5V, black=GND, yellow=signal -> D6 or D7 (one only)");
  Serial.println("Idle state should normally read HIGH because INPUT_PULLUP is enabled.");
  Serial.println("If both pins show the same edges, they may be tied to the same signal wire.");
  Serial.println("dt_ms,pin6_state,pin6_edges,pin6_hz,pin6_q5_lmin,pin6_q7_5_lmin,pin7_state,pin7_edges,pin7_hz,pin7_q5_lmin,pin7_q7_5_lmin,active,last_edge6_ms,last_edge7_ms");
}

void loop() {
  unsigned long now = millis();
  if (now - lastReportMs < REPORT_INTERVAL_MS) {
    return;
  }

  unsigned long total6 = 0;
  unsigned long total7 = 0;
  unsigned long edge6Us = 0;
  unsigned long edge7Us = 0;

  noInterrupts();
  total6 = totalEdgeCount6;
  total7 = totalEdgeCount7;
  edge6Us = lastEdge6Us;
  edge7Us = lastEdge7Us;
  interrupts();

  unsigned long dtMs = now - lastReportMs;
  float dtS = dtMs / 1000.0f;

  unsigned long deltaEdges6 = total6 - lastSnapshotEdges6;
  unsigned long deltaEdges7 = total7 - lastSnapshotEdges7;

  float freq6Hz = (dtS > 0.0f) ? ((deltaEdges6 / 2.0f) / dtS) : 0.0f;
  float freq7Hz = (dtS > 0.0f) ? ((deltaEdges7 / 2.0f) / dtS) : 0.0f;

  float flow6Q5 = freq6Hz / K_HZ_PER_LMIN_OPTION_A;
  float flow6Q75 = freq6Hz / K_HZ_PER_LMIN_OPTION_B;
  float flow7Q5 = freq7Hz / K_HZ_PER_LMIN_OPTION_A;
  float flow7Q75 = freq7Hz / K_HZ_PER_LMIN_OPTION_B;

  unsigned long lastEdge6MsAgo = (edge6Us == 0) ? 0UL : (micros() - edge6Us) / 1000UL;
  unsigned long lastEdge7MsAgo = (edge7Us == 0) ? 0UL : (micros() - edge7Us) / 1000UL;

  int state6 = digitalRead(SENSOR_PIN_6);
  int state7 = digitalRead(SENSOR_PIN_7);

  Serial.print(dtMs);
  Serial.print(",");
  Serial.print(levelText(state6));
  Serial.print(",");
  Serial.print(deltaEdges6);
  Serial.print(",");
  Serial.print(freq6Hz, 2);
  Serial.print(",");
  Serial.print(flow6Q5, 2);
  Serial.print(",");
  Serial.print(flow6Q75, 2);
  Serial.print(",");
  Serial.print(levelText(state7));
  Serial.print(",");
  Serial.print(deltaEdges7);
  Serial.print(",");
  Serial.print(freq7Hz, 2);
  Serial.print(",");
  Serial.print(flow7Q5, 2);
  Serial.print(",");
  Serial.print(flow7Q75, 2);
  Serial.print(",");
  Serial.print(inferActivePin(deltaEdges6, deltaEdges7));
  Serial.print(",");
  Serial.print(lastEdge6MsAgo);
  Serial.print(",");
  Serial.println(lastEdge7MsAgo);

  lastSnapshotEdges6 = total6;
  lastSnapshotEdges7 = total7;
  lastReportMs = now;
}
