#include <Servo.h>

/*
 * Dual thrusters (Left/Right) — read PWM from RX, drive ESCs
 * Rule:
 *   - Valid RX range: 1000..2000 µs  → linearly map to 1100..1900 µs
 *   - Near center (±deadband)        → 1500 µs (stop)
 *   - Out of range (<1000 or >2000)  → 1500 µs (treat as disconnect/fault)
 * Notes:
 *   - Pin 1 is HW Serial TX; using it as input while Serial is active can be flaky.
 */

// === Pins ===
const int CH_LEFT_IN    = 2;   // RX left channel (PWM)  (consider moving off pin 1 if you see glitches)
const int CH_RIGHT_IN   = 3;   // RX right channel (PWM)
const int ESC_LEFT_OUT  = 9;   // Left ESC signal
const int ESC_RIGHT_OUT = 10;  // Right ESC signal

// === Timing ===
const unsigned long PULSE_TIMEOUT_US = 25000; // pulseIn timeout (~25 ms)
const unsigned long FAILSAFE_MS      = 200;   // backup: stop if no update in 200 ms

// === RX valid range and ESC range ===
const int RX_VALID_MIN = 950;   // treat <1000 as disconnect/fault (your log shows ~871)
const int RX_VALID_MAX = 2000;
const int ESC_MIN = 1100;
const int ESC_MID = 1500;
const int ESC_MAX = 1900;

// === Deadband around center ===
const int RC_MID       = 1500;
const int DEADBAND_US  = 25;

Servo escL, escR;
unsigned long lastUpdateL = 0, lastUpdateR = 0;

// Map: 1000..2000 → 1100..1900 with center deadband; else return 1500
int mapLinearToEsc(long inUs) {
  if (inUs == 0) return ESC_MID; // no pulse (timeout) → stop
  if (inUs < RX_VALID_MIN || inUs > RX_VALID_MAX) return ESC_MID; // out of range → stop

  // snap to exact center within deadband
  if (abs((int)inUs - RC_MID) <= DEADBAND_US) return ESC_MID;

  // linear map full span; then clamp
  long out = map(inUs, RX_VALID_MIN, RX_VALID_MAX, ESC_MIN, ESC_MAX);
  if (out < ESC_MIN) out = ESC_MIN;
  if (out > ESC_MAX) out = ESC_MAX;
  return (int)out;
}

void setup() {
  pinMode(CH_LEFT_IN,  INPUT);
  pinMode(CH_RIGHT_IN, INPUT);

  Serial.begin(115200);
  while (!Serial) {}

  escL.attach(ESC_LEFT_OUT);
  escR.attach(ESC_RIGHT_OUT);

  // ESC init: hold 1500 µs for 2 s
  escL.writeMicroseconds(ESC_MID);
  escR.writeMicroseconds(ESC_MID);
  delay(2000);

  Serial.println("inL|outL|inR|outR");
}

void loop() {
  unsigned long now = millis();

  // Read RX pulses (blocking with timeout)
  unsigned long inL = pulseIn(CH_LEFT_IN,  HIGH, PULSE_TIMEOUT_US);
  unsigned long inR = pulseIn(CH_RIGHT_IN, HIGH, PULSE_TIMEOUT_US);

  if (inL != 0) lastUpdateL = now;
  if (inR != 0) lastUpdateR = now;

  // Map according to the rule above
  int outL = mapLinearToEsc((long)inL);
  int outR = mapLinearToEsc((long)inR);

  // Backup timeout failsafe (if pulses stall for a while)
  if (now - lastUpdateL > FAILSAFE_MS) outL = ESC_MID;
  if (now - lastUpdateR > FAILSAFE_MS) outR = ESC_MID;

  // Drive ESCs
  escL.writeMicroseconds(outL);
  escR.writeMicroseconds(outR);

  // Log: inL|outL|inR|outR
 Serial.print(outL);Serial.print('|');Serial.println(outR);

  delay(5);
}