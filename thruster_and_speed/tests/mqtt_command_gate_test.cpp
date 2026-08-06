#include "../mqtt_command_gate.h"
#include "../transport_freshness.h"

#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void testStopThenDirectionWithinRateWindowAppliesImmediately() {
  ThrusterCommandGate gate(20);

  ThrusterCommandDecision stop = gate.observe(1500, 1500, 100);
  ThrusterCommandDecision forward = gate.observe(1700, 1700, 110);

  require(stop.shouldApply, "initial neutral command must apply");
  require(forward.shouldApply,
          "direction command 10 ms after neutral must apply");
  require(forward.receivedAtMs == 110,
          "changed command must carry its receive time for freshness");
}

void testRapidDirectionChangesApplyEvenWithSharedCallbackTime() {
  ThrusterCommandGate gate(20);

  require(gate.observe(1700, 1700, 200).shouldApply,
          "initial forward command must apply");
  require(gate.observe(1300, 1700, 200).shouldApply,
          "left turn at the same callback time must apply");
  require(gate.observe(1700, 1300, 205).shouldApply,
          "right turn inside the rate window must apply");
  require(gate.observe(1300, 1300, 210).shouldApply,
          "reverse command inside the rate window must apply");
}

void testIdenticalHighFrequencyHeartbeatRefreshesWithoutReapplying() {
  ThrusterCommandGate gate(20);

  require(gate.observe(1600, 1600, 1000).shouldApply,
          "initial command must apply");

  ThrusterCommandDecision duplicate = gate.observe(1600, 1600, 1005);
  require(!duplicate.shouldApply,
          "identical command inside 20 ms should be rate limited");
  require(duplicate.receivedAtMs == 1005,
          "rate-limited heartbeat must still refresh command freshness");

  require(gate.observe(1600, 1600, 1020).shouldApply,
          "identical command may apply again after the rate window");
}

void testRepeatedLeaseHeartbeatKeepsControllerFresh() {
  const unsigned long controllerTimeoutMs = 2000;
  unsigned long lastLeaseMs = 100;

  require(isTransportTimestampFresh(1146, lastLeaseMs, controllerTimeoutMs),
          "observed 1046 ms lease gap must remain fresh");

  lastLeaseMs = 1146;
  require(isTransportTimestampFresh(2192, lastLeaseMs, controllerTimeoutMs),
          "repeated lease heartbeat must refresh controller freshness");
  require(!isTransportTimestampFresh(3146, lastLeaseMs, controllerTimeoutMs),
          "controller must expire at the existing 2000 ms boundary");
}

}  // namespace

int main() {
  testStopThenDirectionWithinRateWindowAppliesImmediately();
  testRapidDirectionChangesApplyEvenWithSharedCallbackTime();
  testIdenticalHighFrequencyHeartbeatRefreshesWithoutReapplying();
  testRepeatedLeaseHeartbeatKeepsControllerFresh();
  std::cout << "PASS: mqtt command gate regression tests\n";
  return 0;
}
