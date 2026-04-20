# Thruster MQTT Migration Design

## Summary

This design migrates the current Arduino thruster control transport from UDP to MQTT while preserving the existing safety model:

- control priority remains `network control > RC > neutral failsafe`
- network control should still time out after about `2 seconds`
- RC must remain responsive during WiFi and broker reconnect attempts
- the transport must be selectable at compile time so the system can still be built in legacy UDP mode

The selected design is a topic-oriented MQTT transport with separate control-command and controller-lease topics, plus compile-time selection between UDP and MQTT transports.

In this document, "network control" means the active compile-time-selected transport input, which is UDP in legacy builds and MQTT in migrated builds.

## Goals

- Replace the current fixed-IP UDP transport with MQTT when desired
- Preserve the current thruster safety behavior and transition semantics
- Keep the existing RC, ESC, flow, and DHT logic intact as much as possible
- Allow multiple telemetry consumers without adding more point-to-point sockets
- Support staged rollout by keeping a legacy UDP build available

## Non-Goals

- No runtime transport switching in the first version
- No cloud broker design
- No major refactor of RC capture, ESC output, flow calculation, or DHT logic
- No event-sourcing telemetry model in the first version

## Current System Baseline

The current sketch uses fixed-IP UDP between the Arduino and Jetson:

- commands arrive on UDP port `8888`
- Jetson heartbeat arrives on UDP port `8889`
- status, flow, and DHT telemetry are sent back to the Jetson fixed IP
- a separate monitor stream is also emitted on another UDP port

The control logic has two important notions of freshness:

- whether the Jetson controller is still online
- whether a fresh network command has been received recently

Those two notions are currently represented by separate timestamps and drive the control-mode selection and fallback behavior. That behavior is retained in the new design.

## Selected Architecture

### Broker placement

- Broker host: `192.168.50.200`
- Broker location: Jetson local machine
- Arduino role: MQTT client
- Jetson role: MQTT client and control publisher

### Transport modes

The sketch will support two compile-time transport modes:

- `TRANSPORT_MODE_UDP`
- `TRANSPORT_MODE_MQTT`

Only one transport mode is compiled in for a given build.

This keeps memory use predictable, avoids mixed protocol state, and reduces the risk of stale commands surviving a mid-run transport switch.

### Topic tree

The approved MQTT topic tree is:

- `arduino/thruster/cmd`
- `arduino/thruster/lease`
- `arduino/thruster/status`
- `arduino/flow/status`
- `arduino/dht/status`
- `arduino/system/online`

The separate payload specification is documented in:

- [2026-04-20-mqtt-payload-format.md](/Users/jlw/Documents/Arduino/docs/superpowers/specs/2026-04-20-mqtt-payload-format.md)

### Messaging model

- `cmd` carries thruster setpoints
- `lease` carries controller liveness
- `status`, `flow`, and `dht` carry telemetry snapshots
- `system/online` carries MQTT session online state through retained publish and MQTT Last Will

The command and lease channels stay separate so the design preserves the existing distinction between controller-online state and fresh-command state.

## Payload Format

All MQTT messages in the new transport use JSON payloads.

### Command payload

Topic: `arduino/thruster/cmd`

Required fields:

- `left_us`
- `right_us`

Optional fields:

- `seq`
- `ts_ms`

### Lease payload

Topic: `arduino/thruster/lease`

Recommended fields:

- `client`
- `seq`
- `ts_ms`

### Telemetry payloads

- `arduino/thruster/status` publishes current mode, actual outputs, command age, controller-online flag, and WiFi state
- `arduino/flow/status` publishes flow frequency, flow rate, scaled velocity, and total liters
- `arduino/dht/status` publishes the two DHT sensor snapshots
- `arduino/system/online` publishes `{"state":"online"}` and uses LWT for `{"state":"offline"}`

### Topic behavior rules

- command topic must not use retained messages
- lease topic must not use retained messages
- telemetry topics publish current-state snapshots
- `arduino/system/online` uses retained state and MQTT Last Will
- control and telemetry should use `QoS 0` initially to preserve low latency

## Safety And State Machine

The safety model stays aligned with the current implementation.

### Timing parameters

- `lease_interval_ms = 500`
- `controller_timeout_ms = 2000`
- `cmd_timeout_ms = 2000`
- `grace_ms = 400`

### Internal transport-neutral state

The transport-facing timestamps are generalized to:

- `lastControllerLeaseMs`
- `lastTransportCmdMs`
- `haveTransportCmd`
- `transportConnected`

These replace transport-specific naming so the control logic can remain the same in both UDP and MQTT builds.

### Control decisions

1. If WiFi is disconnected or the selected transport session is disconnected, the sketch must immediately leave network control and use RC.
2. If the transport is connected but controller lease freshness exceeds `controller_timeout_ms`, the sketch must immediately leave network control and use RC.
3. If the controller lease is fresh and the command is fresh, the sketch uses network control.
4. If the controller lease is fresh but the command has expired, the sketch stays in network mode during the `grace_ms` window and soft-decays toward neutral.
5. If no valid network command has ever been accepted since boot or reconnect, the sketch must stay in RC mode even if controller lease traffic is present.

### Output continuity

Whenever the sketch falls back from network control to RC, it should synchronize the network smoothing state to the active RC output so that a later reconnect does not create a control jump.

## Arduino Responsibilities

The Arduino sketch keeps its current high-level loop rhythm:

1. sample flow and RC first
2. maintain WiFi connection
3. service the selected transport input
4. determine control mode
5. update thruster PWM outputs
6. publish at most one telemetry message per loop iteration
7. perform slower sensor refresh work

### WiFi

The existing non-blocking multi-network WiFi reconnect logic remains in place and should not be substantially reworked during the transport migration.

### Transport input

The sketch introduces a transport-neutral input function:

- `pollTransportInput()`

Behavior by build mode:

- UDP build: process command and heartbeat packets from the existing UDP sockets
- MQTT build: poll the MQTT client and process `cmd` and `lease` messages

### Transport output

The sketch introduces a transport-neutral output function:

- `serviceOneTransportSendTask()`

Behavior by build mode:

- UDP build: preserve the existing staged UDP send behavior
- MQTT build: publish one telemetry snapshot topic per loop iteration when due

### MQTT responsibilities

In MQTT builds the Arduino must:

- maintain broker connection in the background
- set an MQTT Last Will on `arduino/system/online`
- publish retained `{"state":"online"}` after a successful connect
- subscribe to `arduino/thruster/cmd`
- subscribe to `arduino/thruster/lease`
- reject reliance on retained control commands

## Jetson Responsibilities

The Jetson control-side process must:

- connect to the local broker at `192.168.50.200:1883`
- publish `arduino/thruster/lease` every `500 ms`
- publish complete left and right thruster setpoints to `arduino/thruster/cmd`
- avoid retained command publishes
- subscribe to telemetry topics as needed

Recommended telemetry subscriptions:

- `arduino/thruster/status`
- `arduino/flow/status`
- `arduino/dht/status`
- `arduino/system/online`

The old dedicated monitor UDP stream is no longer needed in MQTT builds because monitoring becomes a normal MQTT subscription use case.

## Code Structure Changes

The migration should preserve the core control logic and local hardware handling while isolating transport behavior.

### Keep mostly unchanged

- RC input capture and filtering
- ESC output generation
- flow polling and rolling-window calculation
- DHT sampling cadence
- control-priority and failsafe behavior

### Introduce transport abstraction

The sketch should add transport-neutral entry points such as:

- `pollTransportInput()`
- `serviceOneTransportSendTask()`
- `isTransportConnected()`
- transport-specific connect and reconnect helpers

### Generalize transport state

Transport-specific variables should be normalized so the control layer does not care whether commands come from UDP or MQTT.

Examples:

- `lastJetsonPingMs` becomes `lastControllerLeaseMs`
- `lastWifiCmdMs` becomes `lastTransportCmdMs`
- `haveWifiCmd` becomes `haveTransportCmd`

### Compile-time selection

Recommended pattern:

```cpp
#define TRANSPORT_MODE_UDP 0
#define TRANSPORT_MODE_MQTT 1
#define TRANSPORT_MODE TRANSPORT_MODE_MQTT
```

This is preferred over a runtime switch in the first version because it limits state complexity and makes rollback safer.

## Migration Plan

Implementation should be staged:

1. Refactor the existing UDP path behind transport-neutral helper functions while keeping the active build in UDP mode.
2. Add MQTT transport support with topic subscriptions, telemetry publishing, and compile-time selection.
3. Verify that UDP and MQTT builds both compile cleanly.
4. Update the Jetson control process to publish lease and command topics and consume telemetry topics.
5. Roll field testing from UDP build to MQTT build after verifying identical safety behavior.

## Error Handling

### WiFi loss

- RC remains active
- transport is treated as disconnected
- no network control resumes until WiFi and the transport session recover

### Broker disconnect

- in MQTT build, immediately treat network control as unavailable
- keep RC active and retry MQTT in the background
- do not block the loop on reconnect

### Missing commands with healthy lease

- stay in network mode only through the grace-decay window
- then fall back to RC

### Stale retained commands

- protocol rules prohibit retained commands
- the implementation should avoid any design that depends on replaying a last command from the broker

## Testing Strategy

### Functional tests

- verify that UDP build preserves current behavior
- verify that MQTT build accepts `cmd` and `lease` topics and drives thrusters correctly
- verify that no command means RC remains active even if lease is present
- verify that telemetry arrives on the expected MQTT topics

### Safety tests

- stop publishing `lease` and confirm RC takeover within about `2 seconds`
- stop publishing `cmd` while still publishing `lease` and confirm neutral soft-decay followed by RC fallback
- disconnect WiFi and confirm RC remains responsive
- reconnect WiFi and broker and confirm no output jump occurs when MQTT control resumes

### Regression tests

- confirm RC-only operation still works with no WiFi available
- confirm ESC outputs remain neutral-safe during boot
- confirm the flow and DHT publishing cadence remains unchanged in effect

## Risks And Mitigations

- Risk: stale commands after reconnect
  Mitigation: no retained commands, QoS 0 command path, command freshness timeout

- Risk: blocking MQTT reconnect harms RC responsiveness
  Mitigation: non-blocking reconnect scheduling and loop-first RC processing

- Risk: transport abstraction accidentally changes safety behavior
  Mitigation: preserve the existing control-state machine and only replace input and output transport layers

- Risk: memory pressure from adding MQTT support
  Mitigation: compile-time transport selection and staged rollout

## Acceptance Criteria

- The sketch can be built in UDP mode or MQTT mode using a compile-time switch.
- MQTT mode uses the approved `arduino/...` topic tree.
- MQTT mode preserves `network > RC > neutral failsafe` semantics.
- Loss of controller lease causes fallback within about `2 seconds`.
- Presence of lease without a fresh command never activates network control.
- Telemetry topics replace the dedicated monitor UDP stream in MQTT mode.
