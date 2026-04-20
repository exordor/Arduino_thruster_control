# MQTT Payload Format

This document defines the agreed MQTT topic tree and payload format for the Arduino thruster and sensor system.

## Broker Topology

- Broker host: `192.168.50.200`
- Broker role: Jetson local MQTT broker
- Arduino role: MQTT client
- Control priority: `MQTT > RC > neutral failsafe`

## Topic Tree

- `arduino/thruster/cmd`
- `arduino/thruster/lease`
- `arduino/thruster/status`
- `arduino/flow/status`
- `arduino/dht/status`
- `arduino/system/online`

## Topic Rules

- All control and telemetry topics use JSON payloads.
- `arduino/thruster/cmd` must not use retained messages.
- `arduino/thruster/lease` must not use retained messages.
- Telemetry topics should publish current state snapshots.
- `arduino/system/online` should use retained messages and MQTT Last Will.

## Payload Definitions

### `arduino/thruster/cmd`

Direction: Jetson to Arduino

```json
{
  "left_us": 1500,
  "right_us": 1500,
  "seq": 1234,
  "ts_ms": 1710000000
}
```

Field notes:

- `left_us`: required, target PWM pulse width in microseconds for left thruster
- `right_us`: required, target PWM pulse width in microseconds for right thruster
- `seq`: optional sequence number for tracing or duplicate detection
- `ts_ms`: optional sender timestamp for diagnostics

### `arduino/thruster/lease`

Direction: Jetson to Arduino

```json
{
  "client": "jetson-control",
  "seq": 5678,
  "ts_ms": 1710000000
}
```

Field notes:

- `client`: optional control client identifier
- `seq`: optional sequence number for tracing
- `ts_ms`: optional sender timestamp for diagnostics

### `arduino/thruster/status`

Direction: Arduino to subscribers

```json
{
  "mode": "mqtt",
  "left_us": 1600,
  "right_us": 1580,
  "cmd_age_ms": 120,
  "controller_online": true,
  "wifi_connected": true
}
```

Field notes:

- `mode`: current control source, expected values are `"mqtt"` or `"rc"`
- `left_us`: actual left ESC output in microseconds
- `right_us`: actual right ESC output in microseconds
- `cmd_age_ms`: age of the last accepted MQTT command
- `controller_online`: whether the Jetson control publisher is still considered online
- `wifi_connected`: whether the Arduino WiFi link is connected

### `arduino/flow/status`

Direction: Arduino to subscribers

```json
{
  "freq_hz": 12.4,
  "flow_lmin": 2.48,
  "velocity_ms": 0.1342,
  "total_liters": 18.352
}
```

Field notes:

- `freq_hz`: measured pulse frequency in hertz
- `flow_lmin`: estimated flow rate in liters per minute
- `velocity_ms`: scaled flow velocity in meters per second
- `total_liters`: accumulated total volume in liters

### `arduino/dht/status`

Direction: Arduino to subscribers

```json
{
  "sensor_1": {
    "temp_c": 24.3,
    "hum_pct": 55.2
  },
  "sensor_2": {
    "temp_c": 24.1,
    "hum_pct": 54.8
  }
}
```

Field notes:

- `sensor_1`: DHT sensor on the first configured pin
- `sensor_2`: DHT sensor on the second configured pin
- `temp_c`: temperature in Celsius
- `hum_pct`: relative humidity percentage

### `arduino/system/online`

Direction: Arduino to subscribers

Online payload:

```json
{
  "state": "online"
}
```

Offline last-will payload:

```json
{
  "state": "offline"
}
```

## Messaging Guidance

- Prefer `QoS 0` for control and telemetry to preserve low latency.
- Use retained messages only for `arduino/system/online`.
- Do not place stale control commands in retained topics.
- Keep telemetry payloads as flat state snapshots rather than event streams in the first migration stage.
