#include "transport_config.h"

#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_MQTT
#include <WiFiS3.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
unsigned long lastMqttReconnectAttemptMs = 0;
bool mqttOnlineStateDirty = false;
byte nextMqttTelemetryTask = 0;
unsigned long mqttCallbackNow = 0;

enum MqttTelemetryTask : byte {
  MQTT_TELEMETRY_TASK_STATUS = 0,
  MQTT_TELEMETRY_TASK_DHT = 1,
  MQTT_TELEMETRY_TASK_COUNT = 2
};

void applyTransportCommand(int leftUs, int rightUs, unsigned long now);

void mqttMessageCallback(char* topic, uint8_t* payload, unsigned int length) {
  char payloadBuf[MQTT_RX_BUFFER_SIZE];
  if (length == 0 || length >= MQTT_RX_BUFFER_SIZE) {
    return;
  }
  memcpy(payloadBuf, payload, length);
  payloadBuf[length] = '\0';

  unsigned long now = mqttCallbackNow;

  if (strcmp(topic, MQTT_TOPIC_THRUSTER_CMD) == 0) {
    int leftUs = ESC_MID;
    int rightUs = ESC_MID;
    if (parseThrusterCommand(payloadBuf, leftUs, rightUs)) {
      if (now - lastWifiCommandSentMs >= MIN_CMD_INTERVAL_MS) {
        applyTransportCommand(leftUs, rightUs, now);
        lastWifiCommandSentMs = now;
      }
    }
  } else if (strcmp(topic, MQTT_TOPIC_THRUSTER_LEASE) == 0) {
    if (parseLeaseMessage(payloadBuf)) {
      lastControllerLeaseMs = now;
    }
  }
}

bool connectMqttBroker() {
  static const char offlinePayload[] = "{\"state\":\"offline\"}";
  static bool callbackSet = false;

  mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);

  if (!callbackSet) {
    mqttClient.setCallback(mqttMessageCallback);
    callbackSet = true;
  }

  if (!mqttClient.connect(MQTT_CLIENT_ID,
                          MQTT_TOPIC_SYSTEM_ONLINE, 0, true, offlinePayload)) {
    return false;
  }

  if (!mqttClient.subscribe(MQTT_TOPIC_THRUSTER_CMD) ||
      !mqttClient.subscribe(MQTT_TOPIC_THRUSTER_LEASE)) {
    mqttClient.disconnect();
    return false;
  }
  return true;
}

bool publishJsonTopic(const char* topic, bool retain, const char* jsonPayload, size_t payloadSize) {
  if (payloadSize == 0) {
    return false;
  }

  if (payloadSize >= MQTT_TX_BUFFER_SIZE) {
    return false;
  }

  return mqttClient.publish(topic, jsonPayload, retain);
}

bool publishOnlineState() {
  StaticJsonDocument<64> doc;
  doc["state"] = "online";
  char payload[MQTT_TX_BUFFER_SIZE];
  size_t payloadSize = serializeJson(doc, payload, sizeof(payload));
  return publishJsonTopic(MQTT_TOPIC_SYSTEM_ONLINE, true, payload, payloadSize);
}

bool servicePendingOnlineState() {
  if (!mqttOnlineStateDirty || !mqttClient.connected()) {
    return false;
  }

  if (publishOnlineState()) {
    mqttOnlineStateDirty = false;
    nextMqttTelemetryTask = MQTT_TELEMETRY_TASK_STATUS;
  }

  return true;
}

bool publishThrusterStatusMqtt(unsigned long now, bool wifiConnected) {
  if (now - lastStatusSendMs < STATUS_SEND_INTERVAL_MS || !mqttClient.connected()) {
    return false;
  }

  StaticJsonDocument<192> doc;
  doc["mode"] = currentMode == 1 ? "mqtt" : "rc";
  doc["left_us"] = currentLeftUs;
  doc["right_us"] = currentRightUs;
  doc["cmd_age_ms"] = haveTransportCmd ? now - lastTransportCmdMs : 0;
  doc["controller_online"] = isControllerOnline(now);
  doc["wifi_connected"] = wifiConnected;

  char payload[MQTT_TX_BUFFER_SIZE];
  size_t payloadSize = serializeJson(doc, payload, sizeof(payload));
  if (!publishJsonTopic(MQTT_TOPIC_THRUSTER_STATUS, false, payload, payloadSize)) {
    return false;
  }

  lastStatusSendMs = now;
  return true;
}

bool publishFlowStatusMqtt(unsigned long now) {
  if (now - lastFlowSendMs < FLOW_SEND_INTERVAL_MS || !mqttClient.connected()) {
    return false;
  }

  StaticJsonDocument<192> doc;
  doc["freq_hz"] = flowFreqHz;
  doc["flow_lmin"] = flowLmin;
  doc["velocity_ms"] = flowVelocity;
  doc["total_liters"] = totalLiters;

  char payload[MQTT_TX_BUFFER_SIZE];
  size_t payloadSize = serializeJson(doc, payload, sizeof(payload));
  if (!publishJsonTopic(MQTT_TOPIC_FLOW_STATUS, false, payload, payloadSize)) {
    return false;
  }

  lastFlowSendMs = now;
  return true;
}

bool publishDhtStatusMqtt(unsigned long now) {
  if (!ENABLE_DHT_SENSORS || now - lastDhtSendMs < DHT_SEND_INTERVAL_MS ||
      !mqttClient.connected()) {
    return false;
  }

  StaticJsonDocument<256> doc;
  JsonObject sensor1 = doc.createNestedObject("sensor_1");
  sensor1["temp_c"] = dht1Temperature;
  sensor1["hum_pct"] = dht1Humidity;

  JsonObject sensor2 = doc.createNestedObject("sensor_2");
  sensor2["temp_c"] = dht2Temperature;
  sensor2["hum_pct"] = dht2Humidity;

  char payload[MQTT_TX_BUFFER_SIZE];
  size_t payloadSize = serializeJson(doc, payload, sizeof(payload));
  if (!publishJsonTopic(MQTT_TOPIC_DHT_STATUS, false, payload, payloadSize)) {
    return false;
  }

  lastDhtSendMs = now;
  return true;
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
  if (connectMqttBroker()) {
    mqttOnlineStateDirty = true;
    lastMqttReconnectAttemptMs = 0;
    servicePendingOnlineState();
  }
}

void pollMqttTransport(unsigned long now, bool wifiConnected) {
  ensureMqttConnected(now, wifiConnected);
  if (mqttClient.connected()) {
    mqttCallbackNow = now;
    mqttClient.loop();
  }
}
#endif
