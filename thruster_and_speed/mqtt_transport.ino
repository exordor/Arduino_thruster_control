#include "transport_config.h"

#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_MQTT
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>

WiFiClient mqttWifiClient;
MqttClient mqttClient(mqttWifiClient);
unsigned long lastMqttReconnectAttemptMs = 0;

bool connectMqttBroker() {
  mqttClient.setId(MQTT_CLIENT_ID);
  mqttClient.setConnectionTimeout(MQTT_CONNECT_TIMEOUT_MS);
  mqttClient.setKeepAliveInterval(15000);
  return mqttClient.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
}

void ensureMqttConnected(unsigned long now, bool wifiConnected) {
  if (!wifiConnected) {
    if (mqttClient.connected()) {
      mqttClient.stop();
    }
    return;
  }

  if (mqttClient.connected()) {
    return;
  }

  if (lastMqttReconnectAttemptMs != 0 &&
      now - lastMqttReconnectAttemptMs < MQTT_RECONNECT_INTERVAL_MS) {
    return;
  }

  lastMqttReconnectAttemptMs = now;
  connectMqttBroker();
}

void pollMqttTransport(unsigned long now, bool wifiConnected) {
  ensureMqttConnected(now, wifiConnected);
  if (mqttClient.connected()) {
    mqttClient.poll();
  }
}
#endif
