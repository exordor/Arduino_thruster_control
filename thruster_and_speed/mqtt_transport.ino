#include "transport_config.h"

#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_MQTT
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>

WiFiClient mqttWifiClient;
MqttClient mqttClient(mqttWifiClient);
unsigned long lastMqttReconnectAttemptMs = 0;

void applyTransportCommand(int leftUs, int rightUs, unsigned long now);

bool connectMqttBroker() {
  mqttClient.setId(MQTT_CLIENT_ID);
  mqttClient.setConnectionTimeout(MQTT_CONNECT_TIMEOUT_MS);
  mqttClient.setKeepAliveInterval(15000);
  if (!mqttClient.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT)) {
    return false;
  }

  mqttClient.subscribe(MQTT_TOPIC_THRUSTER_CMD);
  mqttClient.subscribe(MQTT_TOPIC_THRUSTER_LEASE);
  return true;
}

bool readMqttPayload(char* buffer, size_t bufferSize) {
  size_t index = 0;
  bool overflowed = false;

  while (mqttClient.available()) {
    if (index + 1 >= bufferSize) {
      overflowed = true;
      break;
    }
    buffer[index++] = static_cast<char>(mqttClient.read());
  }

  while (mqttClient.available()) {
    mqttClient.read();
  }

  buffer[index] = '\0';
  return index > 0 && !overflowed;
}

bool parseThrusterCommand(const char* payload, int& leftUs, int& rightUs) {
  StaticJsonDocument<192> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    return false;
  }

  if (!doc["left_us"].is<int>() || !doc["right_us"].is<int>()) {
    return false;
  }

  leftUs = doc["left_us"].as<int>();
  rightUs = doc["right_us"].as<int>();
  return true;
}

bool parseLeaseMessage(const char* payload) {
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    return false;
  }

  return doc["client"].is<const char*>() || doc["seq"].is<long>() ||
         doc["ts_ms"].is<long>();
}

bool isTransportConnected() {
  return cachedWifiConnected && mqttClient.connected();
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

    int messageSize = mqttClient.parseMessage();
    if (!messageSize) {
      return;
    }

    char payload[MQTT_RX_BUFFER_SIZE];
    if (!readMqttPayload(payload, sizeof(payload))) {
      return;
    }

    String topic = mqttClient.messageTopic();
    if (topic == MQTT_TOPIC_THRUSTER_CMD) {
      int leftUs = ESC_MID;
      int rightUs = ESC_MID;
      if (parseThrusterCommand(payload, leftUs, rightUs)) {
        applyTransportCommand(leftUs, rightUs, now);
      }
    } else if (topic == MQTT_TOPIC_THRUSTER_LEASE) {
      if (parseLeaseMessage(payload)) {
        lastControllerLeaseMs = now;
      }
    }
  }
}
#endif
