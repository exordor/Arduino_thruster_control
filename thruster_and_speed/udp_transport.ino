#include "transport_config.h"

bool isControllerOnline(unsigned long now) {
  return (lastControllerLeaseMs > 0) &&
         (now - lastControllerLeaseMs < JETSON_ONLINE_TIMEOUT_MS);
}

#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_UDP
bool isTransportConnected() {
  return udpServersStarted && cachedWifiConnected;
}
#endif

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
  if (!mqttClient.connected()) {
    return;
  }

  if (servicePendingOnlineState()) {
    return;
  }

  // Flow is the most timing-sensitive telemetry, send it first.
  publishFlowStatusMqtt(now);

  for (byte offset = 0; offset < MQTT_TELEMETRY_TASK_COUNT; ++offset) {
    byte task = (nextMqttTelemetryTask + offset) % MQTT_TELEMETRY_TASK_COUNT;
    bool sent = false;

    switch (task) {
      case MQTT_TELEMETRY_TASK_STATUS:
        sent = publishThrusterStatusMqtt(now, wifiConnected);
        break;
      case MQTT_TELEMETRY_TASK_DHT:
        sent = publishDhtStatusMqtt(now);
        break;
      default:
        break;
    }

    if (sent) {
      nextMqttTelemetryTask = (task + 1) % MQTT_TELEMETRY_TASK_COUNT;
      return;
    }
  }
#endif
}
