// YF-S403 single-pin measurement sketch for Arduino UNO R4.
// Based on the latest debug run, the pulse output is on D7.
//
// Wiring:
//   red    -> 5V
//   black  -> GND
//   yellow -> D7
//
// Sensor data used here (from the provided product sheet):
//   Output signal: NPN pulse
//   Flow range: 1 to 30 L/min
//   Frequency: f(Hz) = 5 * Q(L/min)
//   Pulse count: 1 L ~= 300 pulses
//
// UNO R4 note:
// D7 does not expose external interrupt support in the current core, so this
// sketch uses fast polling plus a small glitch filter.
//
// Serial output format:
//   dt[ms]=..., state=..., edges[count]=..., glitches[count]=...,
//   freq[Hz]=..., flow[L/min]=..., velocity_26mm_raw[m/s]=...,
//   velocity_ts_cal[m/s]=..., total[L]=..., last_edge[ms]=...
//
// Field meanings:
//   dt_ms
//     Length of the current reporting window in milliseconds.
//     With the current settings this is normally about 1000 ms.
//
//   state
//     Instantaneous logic level currently seen on D7 when the row is printed.
//     This is only a snapshot of the pin, not a direct "flow on/off" flag.
//
//   edges
//     Number of accepted signal transitions in this report window.
//     One full pulse is normally two edges:
//       HIGH -> LOW
//       LOW  -> HIGH
//     So pulses_in_window ~= edges / 2.
//
//   glitches
//     Number of rejected transitions that happened too quickly after the
//     previous accepted edge. This helps reveal wiring noise or contact bounce.
//
//   freq_hz
//     Pulse frequency in hertz, computed from accepted edges:
//       freq_hz = (edges / 2) / dt_seconds
//
//   flow_lmin
//     Estimated flow in liters per minute using the sensor datasheet formula:
//       f(Hz) = 5 * Q(L/min)
//       Q = f / 5
//
//   velocity_26mm_raw
//     Raw velocity converted from flow assuming a full 26 mm circular pipe:
//       v = Q / A
//     This is the geometric conversion before any pool calibration.
//
//   velocity_ts_cal
//     Velocity after applying the swimming-pool calibration factor derived from
//     the total station mean speed, treated as ground truth.
//
//   total_liters
//     Cumulative passed volume since boot using the datasheet pulse count:
//       1 liter ~= 300 pulses
//
//   last_edge_ms
//     Time in milliseconds since the last accepted signal edge.
//     This is useful for detecting whether the flow has stopped.

const byte FLOW_PIN = 7;

const unsigned long REPORT_INTERVAL_MS = 1000;
const unsigned long GLITCH_FILTER_US = 250;

const float K_HZ_PER_LMIN_SPEC = 5.0f;
const float PULSES_PER_LITER_SPEC = 300.0f;
const float PIPE_DIAMETER_M = 0.026f;
const float PIPE_AREA_M2 = 3.1415926f * (PIPE_DIAMETER_M * 0.5f) * (PIPE_DIAMETER_M * 0.5f);

// Keep this at 1.0 for the datasheet calibration.
// After a real water test with a known collected volume, you can fine-tune
// this factor without changing the nominal sensor constants above.
const float FLOW_CALIBRATION_SCALE = 1.0f;

// Swimming-pool calibration using total station as ground truth.
// This factor is relative to the raw 26 mm full-pipe velocity used in this
// sketch, not to the already-scaled sensor series inside velocity_analysis.py.
// Derived from:
//   total_station mean speed = 0.5904929809950917 m/s
//   raw 26 mm equivalent sensor mean speed -> total factor = 7.649039733989088
// Mean speed is used instead of peak speed because UDP/WiFi transport delay can
// shift peaks in time, while the interval mean is much less delay-sensitive.
const float VELOCITY_CALIBRATION_SCALE_TS = 7.6490397f;

unsigned long acceptedEdges = 0;
unsigned long rejectedGlitches = 0;
unsigned long lastAcceptedEdgeUs = 0;
unsigned long lastReportMs = 0;
unsigned long lastSnapshotEdges = 0;

int lastObservedState = HIGH;

const char* levelName(int level) {
  return (level == HIGH) ? "HIGH" : "LOW";
}

void setup() {
  Serial.begin(115200);

  pinMode(FLOW_PIN, INPUT_PULLUP);
  delay(20);

  lastObservedState = digitalRead(FLOW_PIN);
  lastReportMs = millis();

  Serial.println();
  Serial.println("YF-S403 flow meter on D7");
  Serial.println("Using datasheet calibration: f = 5 * Q, 1L ~= 300 pulses");
  Serial.println("Velocity output includes raw 26 mm conversion and total-station-calibrated speed.");
  Serial.println("dt[ms]=..., state=..., edges[count]=..., glitches[count]=..., freq[Hz]=..., flow[L/min]=..., velocity_26mm_raw[m/s]=..., velocity_ts_cal[m/s]=..., total[L]=..., last_edge[ms]=...");
}

void loop() {
  int state = digitalRead(FLOW_PIN);
  if (state != lastObservedState) {
    unsigned long nowUs = micros();
    unsigned long sinceLastEdgeUs = nowUs - lastAcceptedEdgeUs;

    // Ignore transitions that happen unrealistically fast.
    // For this sensor, valid pulses at the expected flow range are far slower
    // than this threshold, so sub-250 us changes are most likely noise.
    if (lastAcceptedEdgeUs == 0 || sinceLastEdgeUs >= GLITCH_FILTER_US) {
      acceptedEdges++;
      lastAcceptedEdgeUs = nowUs;
    } else {
      rejectedGlitches++;
    }

    lastObservedState = state;
  }

  unsigned long nowMs = millis();
  if (nowMs - lastReportMs < REPORT_INTERVAL_MS) {
    return;
  }

  unsigned long dtMs = nowMs - lastReportMs;
  float dtS = dtMs / 1000.0f;

  unsigned long deltaEdges = acceptedEdges - lastSnapshotEdges;

  // Convert edge count to pulse frequency.
  // One pulse is approximately two edges, so pulses/s = (edges/2)/seconds.
  float freqHz = (dtS > 0.0f) ? ((deltaEdges / 2.0f) / dtS) : 0.0f;

  // Convert frequency to flow with the provided YF-S403 datasheet formula.
  float flowLminNominal = freqHz / K_HZ_PER_LMIN_SPEC;

  // Convert total accepted edges to total volume.
  // acceptedEdges/2 gives total pulses since boot.
  float totalLitersNominal = (acceptedEdges / 2.0f) / PULSES_PER_LITER_SPEC;

  // Optional correction multiplier for future bucket-test calibration.
  float flowLmin = flowLminNominal * FLOW_CALIBRATION_SCALE;
  float totalLiters = totalLitersNominal * FLOW_CALIBRATION_SCALE;

  // First convert flow to a raw average velocity using the full 26 mm pipe area.
  float flowM3s = (flowLmin * 0.001f) / 60.0f;
  float velocity26mmRaw = (PIPE_AREA_M2 > 0.0f) ? (flowM3s / PIPE_AREA_M2) : 0.0f;

  // Then apply the total-station-based calibration factor.
  float velocityTsCal = velocity26mmRaw * VELOCITY_CALIBRATION_SCALE_TS;

  // If this value keeps growing, it means no new valid pulse has arrived.
  unsigned long ageMs = (lastAcceptedEdgeUs == 0) ? 0UL : (micros() - lastAcceptedEdgeUs) / 1000UL;

  Serial.print("dt[ms]=");
  Serial.print(dtMs);
  Serial.print(", state=");
  Serial.print(levelName(state));
  Serial.print(", edges[count]=");
  Serial.print(deltaEdges);
  Serial.print(", glitches[count]=");
  Serial.print(rejectedGlitches);
  Serial.print(", freq[Hz]=");
  Serial.print(freqHz, 2);
  Serial.print(", flow[L/min]=");
  Serial.print(flowLmin, 2);
  Serial.print(", velocity_26mm_raw[m/s]=");
  Serial.print(velocity26mmRaw, 4);
  Serial.print(", velocity_ts_cal[m/s]=");
  Serial.print(velocityTsCal, 4);
  Serial.print(", total[L]=");
  Serial.print(totalLiters, 3);
  Serial.print(", last_edge[ms]=");
  Serial.println(ageMs);

  lastSnapshotEdges = acceptedEdges;
  lastReportMs = nowMs;
}
