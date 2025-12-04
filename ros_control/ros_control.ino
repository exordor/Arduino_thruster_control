#include <Servo.h>

/*
 * Dual thrusters (Left/Right) — RC + ROS mixed control
 *
 * RC path:
 *   - Read PWM from RX (CH_LEFT_IN, CH_RIGHT_IN)
 *   - Valid range: 950..2000 µs → map to 1100..1900 µs
 *   - Deadband near center → 1500 µs
 *
 * ROS path (via USB Serial):
 *   - Incoming command format (ASCII line):
 *       C <left_us> <right_us>\n
 *     Example: "C 1500 1500\n"
 *   - Values are ESC pulses in microseconds, 1100..1900
 *   - If ROS commands are received recently, thrusters follow ROS.
 *   - If ROS commands time out, fall back to RC.
 *
 * Status output:
 *   - Arduino prints:
 *       S <mode> <outL> <outR>\n
 *     where mode = 0 (RC), 1 (ROS)
 */

 // === Pins ===
const int CH_LEFT_IN    = 2;   // RX left channel (PWM)
const int CH_RIGHT_IN   = 3;   // RX right channel (PWM)
const int ESC_LEFT_OUT  = 10;   // Left ESC signal
const int ESC_RIGHT_OUT = 9;  // Right ESC signal

// === Timing ===
const unsigned long PULSE_TIMEOUT_US = 25000; // pulseIn timeout (~25 ms)
const unsigned long FAILSAFE_MS      = 200;   // RC backup: stop if no update in 500 ms

// === RX valid range and ESC range ===
const int RX_VALID_MIN = 950;   // treat <1000 as disconnect/fault
const int RX_VALID_MAX = 2000;
const int ESC_MIN = 1100;
const int ESC_MID = 1500;
const int ESC_MAX = 1900;

// === Deadband around center ===
const int RC_MID       = 1500;
const int DEADBAND_US  = 25;

// === ROS command timeout ===
const unsigned long ROS_CMD_TIMEOUT_MS = 200;   // if no ROS cmd in 200 ms → fall back to RC

Servo escL, escR;
unsigned long lastUpdateL = 0, lastUpdateR = 0;

// === ROS control state ===
bool haveRosCmd       = false;
unsigned long lastRosCmdMs = 0;
int rosOutL = ESC_MID;
int rosOutR = ESC_MID;

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

// === parse ROS command from Serial ===
// Format: C <left_us> <right_us>\n
void handleSerialCommand() {
  // We only parse when first char is 'C'
  if (!Serial.available()) return;

  if (Serial.peek() != 'C') {
    // discard unknown leading char
    Serial.read();
    return;
  }

  // consume 'C'
  Serial.read();

  // parseInt will skip spaces/newlines and read an integer
  long leftUs  = Serial.parseInt();
  long rightUs = Serial.parseInt();

  // optional: consume until end of line
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') break;
  }

  // constrain to ESC range
  leftUs  = constrain(leftUs,  ESC_MIN, ESC_MAX);
  rightUs = constrain(rightUs, ESC_MIN, ESC_MAX);

  rosOutL = (int)leftUs;
  rosOutR = (int)rightUs;
  lastRosCmdMs = millis();
  haveRosCmd   = true;
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

  Serial.println("# Thruster controller ready: RC + ROS");
}

void loop() {
  unsigned long now = millis();

  // 1) Read any incoming ROS command
  handleSerialCommand();

  // 2) Read RX pulses (blocking with timeout)
  unsigned long inL = pulseIn(CH_LEFT_IN,  HIGH, PULSE_TIMEOUT_US);
  unsigned long inR = pulseIn(CH_RIGHT_IN, HIGH, PULSE_TIMEOUT_US);

  if (inL != 0) lastUpdateL = now;
  if (inR != 0) lastUpdateR = now;

  // Map RC input to ESC pulses
  int rcOutL = mapLinearToEsc((long)inL);
  int rcOutR = mapLinearToEsc((long)inR);

  // Backup timeout failsafe on RC
  if (now - lastUpdateL > FAILSAFE_MS) rcOutL = ESC_MID;
  if (now - lastUpdateR > FAILSAFE_MS) rcOutR = ESC_MID;

  // 3) Decide control mode: ROS or RC
  bool rosActive = haveRosCmd && (now - lastRosCmdMs < ROS_CMD_TIMEOUT_MS);

  int finalOutL = rosActive ? rosOutL : rcOutL;
  int finalOutR = rosActive ? rosOutR : rcOutR;

  // 4) Drive ESCs
  escL.writeMicroseconds(finalOutL);
  escR.writeMicroseconds(finalOutR);

  // 5) Status report: mode (0=RC, 1=ROS), outL, outR
  Serial.print("S ");
  Serial.print(rosActive ? 1 : 0);
  Serial.print(' ');
  Serial.print(finalOutL);
  Serial.print(' ');
  Serial.println(finalOutR);

  delay(5);
}
