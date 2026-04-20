# Thruster MQTT Transport Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a compile-time-selectable MQTT transport to the UNO R4 WiFi thruster sketch while preserving the existing UDP behavior, RC fallback, and 2-second controller timeout semantics.

**Architecture:** Keep `thruster_and_speed.ino` as the hardware-and-control core, move the legacy UDP path behind transport-neutral wrappers, and add a second transport implementation for MQTT using `ArduinoMqttClient` plus `ArduinoJson`. The selected transport is controlled by a preprocessor macro so UDP and MQTT builds can be compiled and validated independently.

**Tech Stack:** Arduino UNO R4 WiFi, `WiFiS3`, `ArduinoMqttClient`, `ArduinoJson`, `arduino-cli`, Python 3

---

## File Structure

### Existing files to modify

- `thruster_and_speed/thruster_and_speed.ino`
  Core RC input, flow meter, DHT, ESC output, WiFi reconnect logic, and transport-neutral control-state decisions.
- `thruster_and_speed/README.md`
  Update transport documentation, build commands, MQTT topic reference, and verification instructions.

### New files to create

- `thruster_and_speed/transport_config.h`
  Compile-time transport selection, MQTT broker/topic constants, shared buffer sizes, and transport-mode helper macros.
- `thruster_and_speed/udp_transport.ino`
  Transport-neutral wrappers around the existing UDP command, heartbeat, and telemetry code.
- `thruster_and_speed/mqtt_transport.ino`
  MQTT connection management, topic subscription, JSON payload parsing, JSON telemetry publishing, and online-state handling.
- `thruster_and_speed/mqtt_test.py`
  Manual integration helper that can subscribe to telemetry topics and optionally publish `lease` and `cmd` traffic for bench testing.

## Common Validation Commands

- UDP build:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=0" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

- MQTT build:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=1" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

The current baseline sketch already compiles with `arduino-cli` under `arduino:renesas_uno:unor4wifi`, so use these commands throughout the plan as the main regression gate.

### Task 1: Introduce Transport-Neutral State And Preserve UDP Behavior

**Files:**
- Create: `thruster_and_speed/transport_config.h`
- Create: `thruster_and_speed/udp_transport.ino`
- Modify: `thruster_and_speed/thruster_and_speed.ino`
- Test: UDP compile command above

- [ ] **Step 1: Switch the sketch to generic transport hooks and verify the build fails before the wrappers exist**

Edit `thruster_and_speed/thruster_and_speed.ino` so the transport state and loop no longer use UDP-specific names directly:

```cpp
#include "transport_config.h"

unsigned long lastTransportCmdMs = 0;
unsigned long lastControllerLeaseMs = 0;
bool haveTransportCmd = false;

bool isControllerOnline(unsigned long now);
bool isTransportConnected();
void pollTransportInput(unsigned long now, bool wifiConnected);
void serviceOneTransportSendTask(unsigned long now, bool wifiConnected);
```

Update the control logic to reference the generic state:

```cpp
bool controllerOnline = isControllerOnline(now);
if (!controllerOnline) {
  currentMode = 0;
  currentLeftUs = rcOutL;
  currentRightUs = rcOutR;
  wifiAvgL = currentLeftUs;
  wifiAvgR = currentRightUs;
  wifiOutL = wifiAvgL;
  wifiOutR = wifiAvgR;
  haveTransportCmd = false;
  return;
}

bool transportActive = haveTransportCmd && (now - lastTransportCmdMs < UDP_TIMEOUT_MS);
```

Update the main loop call sites:

```cpp
pollTransportInput(now, wifiConnected);
serviceOneTransportSendTask(now, wifiConnected);
```

Run:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=0" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

Expected: FAIL with undefined references such as `pollTransportInput`, `serviceOneTransportSendTask`, or `isControllerOnline`.

- [ ] **Step 2: Add the compile-time transport configuration header**

Create `thruster_and_speed/transport_config.h`:

```cpp
#pragma once

#include <Arduino.h>

#define TRANSPORT_MODE_UDP 0
#define TRANSPORT_MODE_MQTT 1

#ifndef THRUSTER_TRANSPORT_MODE
#define THRUSTER_TRANSPORT_MODE TRANSPORT_MODE_UDP
#endif

static_assert(
    THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_UDP ||
        THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_MQTT,
    "THRUSTER_TRANSPORT_MODE must be TRANSPORT_MODE_UDP or TRANSPORT_MODE_MQTT");

constexpr uint16_t MQTT_BROKER_PORT = 1883;
constexpr char MQTT_BROKER_HOST[] = "192.168.50.200";
constexpr char MQTT_CLIENT_ID[] = "arduino-thruster";
constexpr char MQTT_TOPIC_THRUSTER_CMD[] = "arduino/thruster/cmd";
constexpr char MQTT_TOPIC_THRUSTER_LEASE[] = "arduino/thruster/lease";
constexpr char MQTT_TOPIC_THRUSTER_STATUS[] = "arduino/thruster/status";
constexpr char MQTT_TOPIC_FLOW_STATUS[] = "arduino/flow/status";
constexpr char MQTT_TOPIC_DHT_STATUS[] = "arduino/dht/status";
constexpr char MQTT_TOPIC_SYSTEM_ONLINE[] = "arduino/system/online";

constexpr size_t MQTT_RX_BUFFER_SIZE = 256;
constexpr size_t MQTT_TX_BUFFER_SIZE = 256;
constexpr unsigned long MQTT_RECONNECT_INTERVAL_MS = 2000;
```

- [ ] **Step 3: Wrap the existing UDP logic in a dedicated transport file**

Create `thruster_and_speed/udp_transport.ino` with transport-neutral forwarding:

```cpp
#include "transport_config.h"

bool isControllerOnline(unsigned long now) {
  return (lastControllerLeaseMs > 0) &&
         (now - lastControllerLeaseMs < JETSON_ONLINE_TIMEOUT_MS);
}

void pollTransportInput(unsigned long now, bool wifiConnected) {
#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_UDP
  if (!udpServersStarted || !wifiConnected) {
    return;
  }
  readUdpCommands();
  readHeartbeatPing();
#else
  (void)now;
  (void)wifiConnected;
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
```

- [ ] **Step 4: Rename the old UDP timestamps and flags in place so the control layer is transport-neutral**

Update the state block in `thruster_and_speed/thruster_and_speed.ino`:

```cpp
unsigned long lastTransportCmdMs = 0;
unsigned long lastUdpReceiveMs = 0;
unsigned long lastControllerLeaseMs = 0;
unsigned long lastHeartbeatMs = 0;
int wifiOutL = ESC_MID;
int wifiOutR = ESC_MID;
bool haveTransportCmd = false;
```

Update the old UDP handlers so they write to the new names:

```cpp
lastTransportCmdMs = millis();
haveTransportCmd = true;
lastControllerLeaseMs = now;
```

and

```cpp
if (strcmp(udpBuffer, "PING") == 0 || strcmp(udpBuffer, "P") == 0) {
  lastControllerLeaseMs = millis();
}
```

- [ ] **Step 5: Rebuild the UDP configuration and confirm the behavior-preserving refactor compiles**

Run:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=0" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

Expected: PASS with the normal `Sketch uses ...` memory summary and no new warnings about undefined transport symbols.

- [ ] **Step 6: Commit the transport-neutral UDP refactor**

```bash
git add /Users/jlw/Documents/Arduino/thruster_and_speed/transport_config.h \
        /Users/jlw/Documents/Arduino/thruster_and_speed/udp_transport.ino \
        /Users/jlw/Documents/Arduino/thruster_and_speed/thruster_and_speed.ino
git commit -m "refactor: abstract UDP transport behind generic hooks"
```

### Task 2: Add MQTT Build Skeleton And Dual-Build Support

**Files:**
- Create: `thruster_and_speed/mqtt_transport.ino`
- Modify: `thruster_and_speed/transport_config.h`
- Test: UDP compile command, MQTT compile command

- [ ] **Step 1: Create an MQTT transport file that intentionally fails to compile before the libraries are installed**

Create `thruster_and_speed/mqtt_transport.ino` with the library includes and MQTT-only globals:

```cpp
#include "transport_config.h"

#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_MQTT
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>

WiFiClient mqttWifiClient;
MqttClient mqttClient(mqttWifiClient);

unsigned long lastMqttReconnectAttemptMs = 0;
bool mqttConnected = false;
#endif
```

Run:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=1" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

Expected: FAIL with `fatal error: ArduinoMqttClient.h: No such file or directory` or `ArduinoJson.h: No such file or directory`.

- [ ] **Step 2: Install the MQTT and JSON libraries**

Run:

```bash
arduino-cli lib install "ArduinoMqttClient"
arduino-cli lib install "ArduinoJson"
```

Expected: both libraries install successfully and appear in `arduino-cli lib list`.

- [ ] **Step 3: Flesh out a compile-safe MQTT skeleton with explicit connect and poll helpers**

Replace the temporary `mqtt_transport.ino` contents with:

```cpp
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
  mqttClient.setConnectionTimeout(5000);
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
```

- [ ] **Step 4: Wire the generic transport hooks to the MQTT skeleton in MQTT builds**

Update `thruster_and_speed/udp_transport.ino` so the wrappers dispatch by compile-time mode:

```cpp
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
```

- [ ] **Step 5: Compile both transport modes**

Run:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=0" \
  /Users/jlw/Documents/Arduino/thruster_and_speed

arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=1" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

Expected: both commands PASS, even though the MQTT path does not yet consume topics or publish telemetry.

- [ ] **Step 6: Commit the dual-build transport skeleton**

```bash
git add /Users/jlw/Documents/Arduino/thruster_and_speed/transport_config.h \
        /Users/jlw/Documents/Arduino/thruster_and_speed/udp_transport.ino \
        /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_transport.ino
git commit -m "build: add compile-time MQTT transport skeleton"
```

### Task 3: Implement MQTT Lease And Command Intake

**Files:**
- Modify: `thruster_and_speed/mqtt_transport.ino`
- Modify: `thruster_and_speed/thruster_and_speed.ino`
- Test: MQTT compile command

- [ ] **Step 1: Add payload-reading and JSON-deserialization helpers for incoming MQTT messages**

Extend `thruster_and_speed/mqtt_transport.ino` with these helpers:

```cpp
bool readMqttPayload(char* buffer, size_t bufferSize) {
  size_t index = 0;
  while (mqttClient.available() && index + 1 < bufferSize) {
    buffer[index++] = static_cast<char>(mqttClient.read());
  }
  buffer[index] = '\0';
  return index > 0;
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
```

- [ ] **Step 2: Apply the existing filter-and-ramp behavior to MQTT commands**

Add a command application helper that reuses the existing control math:

```cpp
void applyTransportCommand(int leftUs, int rightUs, unsigned long now) {
  int rawL = constrain(leftUs, ESC_MIN, ESC_MAX);
  int rawR = constrain(rightUs, ESC_MIN, ESC_MAX);

  int filtL = (wifiAvgL * (100 - WIFI_FILTER_ALPHA) + rawL * WIFI_FILTER_ALPHA) / 100;
  int filtR = (wifiAvgR * (100 - WIFI_FILTER_ALPHA) + rawR * WIFI_FILTER_ALPHA) / 100;

  int deltaL = constrain(filtL - wifiAvgL, -WIFI_MAX_STEP_US, WIFI_MAX_STEP_US);
  int deltaR = constrain(filtR - wifiAvgR, -WIFI_MAX_STEP_US, WIFI_MAX_STEP_US);

  wifiAvgL += deltaL;
  wifiAvgR += deltaR;
  wifiOutL = wifiAvgL;
  wifiOutR = wifiAvgR;

  lastTransportCmdMs = now;
  lastControllerLeaseMs = now;
  haveTransportCmd = true;
}
```

- [ ] **Step 3: Dispatch incoming MQTT topics to the new parsers**

Add message dispatch to `pollMqttTransport()`:

```cpp
void pollMqttTransport(unsigned long now, bool wifiConnected) {
  ensureMqttConnected(now, wifiConnected);
  if (!mqttClient.connected()) {
    return;
  }

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
```

- [ ] **Step 4: Treat MQTT disconnects as transport loss in the control logic**

Update the transport-connected test in `thruster_and_speed/thruster_and_speed.ino`:

```cpp
bool isTransportConnected() {
#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_UDP
  return cachedWifiConnected && udpServersStarted;
#else
  return cachedWifiConnected && mqttClient.connected();
#endif
}
```

Use it in `determineControlMode()`:

```cpp
if (!isTransportConnected()) {
  currentMode = 0;
  currentLeftUs = rcOutL;
  currentRightUs = rcOutR;
  wifiAvgL = currentLeftUs;
  wifiAvgR = currentRightUs;
  wifiOutL = wifiAvgL;
  wifiOutR = wifiAvgR;
  haveTransportCmd = false;
  return;
}
```

- [ ] **Step 5: Rebuild the MQTT configuration**

Run:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=1" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

Expected: PASS, with MQTT topic intake compiled and linked successfully.

- [ ] **Step 6: Commit the MQTT input path**

```bash
git add /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_transport.ino \
        /Users/jlw/Documents/Arduino/thruster_and_speed/thruster_and_speed.ino
git commit -m "feat: add MQTT command and lease intake"
```

### Task 4: Publish MQTT Telemetry And Online State

**Files:**
- Modify: `thruster_and_speed/mqtt_transport.ino`
- Modify: `thruster_and_speed/thruster_and_speed.ino`
- Test: UDP compile command, MQTT compile command

- [ ] **Step 1: Configure MQTT last-will and subscriptions during broker connect**

Replace `connectMqttBroker()` with a version that sets the retained offline will and re-subscribes after connect:

```cpp
bool connectMqttBroker() {
  static const char offlinePayload[] = "{\"state\":\"offline\"}";

  mqttClient.setId(MQTT_CLIENT_ID);
  mqttClient.setConnectionTimeout(5000);
  mqttClient.setKeepAliveInterval(15000);
  mqttClient.beginWill(MQTT_TOPIC_SYSTEM_ONLINE,
                       strlen(offlinePayload),
                       true,
                       0);
  mqttClient.print(offlinePayload);
  mqttClient.endWill();

  if (!mqttClient.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT)) {
    return false;
  }

  mqttClient.subscribe(MQTT_TOPIC_THRUSTER_CMD);
  mqttClient.subscribe(MQTT_TOPIC_THRUSTER_LEASE);
  return true;
}
```

- [ ] **Step 2: Add JSON telemetry publishing helpers**

Append these helpers to `thruster_and_speed/mqtt_transport.ino`:

```cpp
bool publishJsonTopic(const char* topic, bool retain, JsonDocument& doc) {
  char payload[MQTT_TX_BUFFER_SIZE];
  size_t payloadSize = serializeJson(doc, payload, sizeof(payload));
  if (payloadSize == 0) {
    return false;
  }

  mqttClient.beginMessage(topic, payloadSize, retain, 0, false);
  mqttClient.print(payload);
  return mqttClient.endMessage();
}

bool publishOnlineState() {
  StaticJsonDocument<64> doc;
  doc["state"] = "online";
  return publishJsonTopic(MQTT_TOPIC_SYSTEM_ONLINE, true, doc);
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

  if (!publishJsonTopic(MQTT_TOPIC_THRUSTER_STATUS, false, doc)) {
    return false;
  }

  lastStatusSendMs = now;
  return true;
}
```

- [ ] **Step 3: Add flow and DHT MQTT publishers that mirror the existing telemetry cadence**

Add these two helpers:

```cpp
bool publishFlowStatusMqtt(unsigned long now) {
  if (now - lastFlowSendMs < FLOW_SEND_INTERVAL_MS || !mqttClient.connected()) {
    return false;
  }

  StaticJsonDocument<192> doc;
  doc["freq_hz"] = flowFreqHz;
  doc["flow_lmin"] = flowLmin;
  doc["velocity_ms"] = flowVelocity;
  doc["total_liters"] = totalLiters;

  if (!publishJsonTopic(MQTT_TOPIC_FLOW_STATUS, false, doc)) {
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

  if (!publishJsonTopic(MQTT_TOPIC_DHT_STATUS, false, doc)) {
    return false;
  }

  lastDhtSendMs = now;
  return true;
}
```

- [ ] **Step 4: Mirror the one-message-per-loop telemetry scheduler in MQTT mode**

Update `serviceOneTransportSendTask()`:

```cpp
void serviceOneTransportSendTask(unsigned long now, bool wifiConnected) {
#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_UDP
  serviceOneUdpSendTask(now, wifiConnected);
#else
  if (!mqttClient.connected()) {
    return;
  }

  if (publishFlowStatusMqtt(now)) {
    return;
  }
  if (publishThrusterStatusMqtt(now, wifiConnected)) {
    return;
  }
  if (publishDhtStatusMqtt(now)) {
    return;
  }
#endif
}
```

Publish the retained online state after a successful connect:

```cpp
if (connectMqttBroker()) {
  lastMqttReconnectAttemptMs = 0;
  publishOnlineState();
}
```

- [ ] **Step 5: Rebuild both transport modes**

Run:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=0" \
  /Users/jlw/Documents/Arduino/thruster_and_speed

arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=1" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

Expected: both commands PASS, with the MQTT build now including topic subscriptions, JSON publish helpers, and LWT setup.

- [ ] **Step 6: Commit the MQTT telemetry path**

```bash
git add /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_transport.ino \
        /Users/jlw/Documents/Arduino/thruster_and_speed/thruster_and_speed.ino
git commit -m "feat: publish MQTT telemetry and online state"
```

### Task 5: Document MQTT Usage And Add Manual Bench Test Tooling

**Files:**
- Create: `thruster_and_speed/mqtt_test.py`
- Modify: `thruster_and_speed/README.md`
- Test: `python3 -m py_compile`, UDP compile command, MQTT compile command

- [ ] **Step 1: Add a simple Python MQTT bench tool alongside the existing UDP helper**

Create `thruster_and_speed/mqtt_test.py`:

```python
#!/usr/bin/env python3
import argparse
import json
import time

import paho.mqtt.client as mqtt


TOPICS = [
    "arduino/system/online",
    "arduino/thruster/status",
    "arduino/flow/status",
    "arduino/dht/status",
]


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"connected: rc={reason_code}")
    for topic in TOPICS:
        client.subscribe(topic)


def on_message(client, userdata, msg):
    print(f"{msg.topic}: {msg.payload.decode()}")


def publish_control(client, left_us, right_us):
    payload = {"left_us": left_us, "right_us": right_us, "seq": int(time.time() * 1000)}
    client.publish("arduino/thruster/cmd", json.dumps(payload), qos=0, retain=False)


def publish_lease(client):
    payload = {"client": "mqtt-test", "ts_ms": int(time.time() * 1000)}
    client.publish("arduino/thruster/lease", json.dumps(payload), qos=0, retain=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.50.200")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--left", type=int, default=1500)
    parser.add_argument("--right", type=int, default=1500)
    parser.add_argument("--publish", action="store_true")
    parser.add_argument("--lease-only", action="store_true")
    args = parser.parse_args()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="mqtt-test-helper")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, 60)
    client.loop_start()

    try:
        while True:
            if args.publish or args.lease_only:
                publish_lease(client)
            if args.publish and not args.lease_only:
                publish_control(client, args.left, args.right)
        time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Update the README with transport modes, dependencies, build commands, and MQTT topics**

Add these sections to `thruster_and_speed/README.md`:

````md
## Transport Modes

The sketch now supports compile-time transport selection:

- UDP build: legacy fixed-IP sockets
- MQTT build: broker at `192.168.50.200:1883`

### Build Commands

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=0" \
  /Users/jlw/Documents/Arduino/thruster_and_speed

arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=1" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

### MQTT Topics

- `arduino/thruster/cmd`
- `arduino/thruster/lease`
- `arduino/thruster/status`
- `arduino/flow/status`
- `arduino/dht/status`
- `arduino/system/online`

### Extra Libraries

```bash
arduino-cli lib install "ArduinoMqttClient"
arduino-cli lib install "ArduinoJson"
```

### Bench Test Helper

```bash
python3 -m pip install paho-mqtt
python3 /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_test.py --publish --left 1600 --right 1600
```
````

- [ ] **Step 3: Validate the new Python helper**

Run:

```bash
python3 -m py_compile /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_test.py
```

Expected: PASS with no output.

- [ ] **Step 4: Re-run both firmware compile modes as the final code regression gate**

Run:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=0" \
  /Users/jlw/Documents/Arduino/thruster_and_speed

arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi \
  --build-property compiler.cpp.extra_flags="-DTHRUSTER_TRANSPORT_MODE=1" \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

Expected: both commands PASS.

- [ ] **Step 5: Commit the docs and bench-test helper**

```bash
git add /Users/jlw/Documents/Arduino/thruster_and_speed/README.md \
        /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_test.py
git commit -m "docs: document MQTT transport usage"
```

### Task 6: Run Hardware Bench Validation Before Merge

**Files:**
- Modify: none
- Test: firmware upload plus bench verification against the Jetson broker

- [ ] **Step 1: Upload the MQTT build to the UNO R4 WiFi**

Run:

```bash
arduino-cli upload -p /dev/tty.usbmodem* \
  --fqbn arduino:renesas_uno:unor4wifi \
  /Users/jlw/Documents/Arduino/thruster_and_speed
```

Expected: upload completes successfully to the connected UNO R4 WiFi after the MQTT build has been compiled.

- [ ] **Step 2: Verify MQTT online state appears on the broker**

Run on the Jetson or another broker-connected machine:

```bash
python3 /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_test.py
```

Expected: the helper prints `arduino/system/online: {"state":"online"}` soon after the board connects to WiFi and the broker.

- [ ] **Step 3: Verify lease-only traffic does not activate network control**

Run:

```bash
python3 /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_test.py --lease-only
```

Expected: the sketch remains in RC mode because `haveTransportCmd` stays false.

- [ ] **Step 4: Verify command + lease traffic activates MQTT mode and telemetry updates**

Publish control plus lease at 2 Hz or faster and observe:

```bash
python3 /Users/jlw/Documents/Arduino/thruster_and_speed/mqtt_test.py --publish --left 1600 --right 1600
```

Expected: the thrusters enter MQTT mode, `arduino/thruster/status` reports `"mode":"mqtt"`, and `arduino/flow/status` plus `arduino/dht/status` continue publishing at the configured cadences.

- [ ] **Step 5: Verify the 2-second safety fallback**

Stop the helper with `Ctrl+C`.

Expected:

- within about 2 seconds, `arduino/thruster/status` reports RC mode again
- outputs decay toward neutral during the grace window
- RC input remains responsive through WiFi and broker reconnect attempts

- [ ] **Step 6: Commit only after the manual bench checklist passes**

```bash
git status --short
```

Expected: only the intended firmware, helper, and documentation changes are present, with both compile modes already verified and the bench checks completed.
