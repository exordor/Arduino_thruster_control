#pragma once

#include <Arduino.h>

#define TRANSPORT_MODE_UDP 0
#define TRANSPORT_MODE_MQTT 1

#ifndef THRUSTER_TRANSPORT_MODE
#define THRUSTER_TRANSPORT_MODE TRANSPORT_MODE_MQTT
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
constexpr int MQTT_MAX_CONNECT_ATTEMPTS = 2;       // 0 = unlimited

#if THRUSTER_TRANSPORT_MODE == TRANSPORT_MODE_MQTT
void pollMqttTransport(unsigned long now, bool wifiConnected);
#endif
