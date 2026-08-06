#include "transport_config.h"

#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_MQTT
#include <WiFiS3.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "mqtt_command_gate.h"

WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

// PubSubClient defaults to 15 s socket timeout — that blocks the main loop
// (and therefore RC processing) for up to 30 s per connect attempt and 15 s
// per partial MQTT read inside loop().  Keep it short so RC stays responsive.
constexpr uint16_t MQTT_SOCKET_TIMEOUT_S = 1;

// WiFiS3 WiFiClient::connect() sends the TCP SYN via modem.write() and blocks
// until the ESP32 responds.  Without a connection timeout the ESP32 uses its
// default TCP timeout (30+ s), freezing the main loop the entire time.
// 1 s is enough for a LAN broker and keeps RC responsive.
constexpr int MQTT_TCP_CONNECT_TIMEOUT_MS = 1000;
unsigned long lastMqttReconnectAttemptMs = 0;
bool mqttOnlineStateDirty = false;
byte nextMqttTelemetryTask = 0;
unsigned long mqttCallbackNow = 0;
int mqttConnectFailCount = 0;
bool mqttGaveUp = false;
bool mqttPrevWifiConnected = false;
ThrusterCommandGate mqttCommandGate(MIN_CMD_INTERVAL_MS);

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
      if (!haveTransportCmd) {
        mqttCommandGate = ThrusterCommandGate(MIN_CMD_INTERVAL_MS);
      }

      ThrusterCommandDecision decision =
          mqttCommandGate.observe(leftUs, rightUs, now);

      // Every valid command is fresh controller input, including an identical
      // high-rate repeat that does not need to be written to the ESC state.
      lastTransportCmdMs = decision.receivedAtMs;
      lastControllerLeaseMs = decision.receivedAtMs;
      haveTransportCmd = true;

      if (decision.shouldApply) {
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
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
  mqttWifiClient.setConnectionTimeout(MQTT_TCP_CONNECT_TIMEOUT_MS);

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
    mqttPrevWifiConnected = false;
    return;
  }

  // Reset on WiFi reconnect
  if (!mqttPrevWifiConnected) {
    mqttGaveUp = false;
    mqttConnectFailCount = 0;
    lastMqttReconnectAttemptMs = 0;
  }
  mqttPrevWifiConnected = true;

  if (mqttClient.connected()) {
    mqttConnectFailCount = 0;
    return;
  }

  if (mqttGaveUp) {
    return;
  }

  // Exponential backoff: 2s, 4s, 8s, ... up to 16s
  unsigned long backoff = MQTT_RECONNECT_INTERVAL_MS << mqttConnectFailCount;
  if (backoff > 16000) backoff = 16000;

  if (lastMqttReconnectAttemptMs != 0 &&
      now - lastMqttReconnectAttemptMs < backoff) {
    return;
  }

  lastMqttReconnectAttemptMs = now;
  if (connectMqttBroker()) {
    mqttOnlineStateDirty = true;
    lastMqttReconnectAttemptMs = 0;
    mqttConnectFailCount = 0;
    servicePendingOnlineState();
  } else {
    mqttConnectFailCount++;
    if (MQTT_MAX_CONNECT_ATTEMPTS > 0 &&
        mqttConnectFailCount >= MQTT_MAX_CONNECT_ATTEMPTS) {
      mqttGaveUp = true;
      Serial.print("MQTT gave up after ");
      Serial.print(mqttConnectFailCount);
      Serial.println(" attempts (reconnect on WiFi reset)");
    }
  }
}

void pollMqttTransport(unsigned long now, bool wifiConnected) {
  ensureMqttConnected(now, wifiConnected);
  if (mqttClient.connected()) {
    mqttCallbackNow = now;
    mqttClient.loop();
  }
}

bool isMqttConnected() {
  return mqttClient.connected();
}

bool isMqttGivenUp() {
  return mqttGaveUp;
}

void resetMqttGaveUp() {
  mqttGaveUp = false;
  mqttConnectFailCount = 0;
  lastMqttReconnectAttemptMs = 0;
}
#endif
