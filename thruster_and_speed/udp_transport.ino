#include "transport_config.h"

bool isControllerOnline(unsigned long now) {
  return (lastControllerLeaseMs > 0) &&
         (now - lastControllerLeaseMs < JETSON_ONLINE_TIMEOUT_MS);
}

bool isTransportConnected() {
  return udpServersStarted && cachedWifiConnected;
}

void pollTransportInput(unsigned long now, bool wifiConnected) {
#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_UDP
  if (!udpServersStarted || !wifiConnected) {
    return;
  }
  readUdpCommands();
  readHeartbeatPing();
#else
  pollMqttTransport(now, wifiConnected);
#endif
}

void serviceOneTransportSendTask(unsigned long now, bool wifiConnected) {
#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_UDP
  serviceOneUdpSendTask(now, wifiConnected);
#else
  (void)now;
  (void)wifiConnected;
#endif
}
