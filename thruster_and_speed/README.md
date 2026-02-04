# Thruster & Flow Meter Control (Arduino UNO R4 WiFi)

A dual thruster control system with integrated flow meter sensor for Arduino UNO R4 WiFi, using dual-port UDP communication for low-latency real-time control.

## Features

- **Dual Thruster Control**: Two ESC outputs for differential thrust
- **RC Fallback**: Automatic switch to RC receiver when UDP timeout
- **Flow Meter**: Real-time flow rate and volume measurement
- **Dual-Port UDP**: Separated data and heartbeat channels for reliable communication
- **PING/PONG Protocol**: Handshake and keep-alive without affecting thruster commands
- **Multi-Network WiFi**: Auto-connect to configured networks with reconnection

## Hardware

| Component | Pin | Description |
|-----------|-----|-------------|
| RC Right IN | D2 | PWM input from RC receiver |
| RC Left IN | D3 | PWM input from RC receiver |
| ESC Right OUT | D9 | Output to right ESC |
| ESC Left OUT | D10 | Output to left ESC |
| **Flow Sensor** | **D7** | **Flow meter signal (polling mode)** |

## Specifications

| Parameter | Value |
|-----------|-------|
| Pipe diameter | 26 mm |
| Calibration factor (K) | 5 Hz per L/min |
| Pulses per liter | 300 |
| Flow update rate | 1 Hz |
| Status update rate | 10 Hz |
| Heartbeat interval | 500 ms |
| UDP timeout | 2000 ms |
| **WiFi command rate limit** | **20 ms min interval (50 Hz max)** |
| **RC deadband** | **±40 µs (joystick drift resistance)** |
| **WiFi filter** | **100% alpha (direct control)** |
| **RC filter** | **25% alpha (smooth)** |

## Wiring

```
Flow Sensor Signal  →  D7
RC Receiver Right   →  D2
RC Receiver Left    →  D3
Right ESC           →  D9
Left ESC            →  D10
```

## UDP Communication (Dual Port)

### Architecture

The system uses two separate UDP ports for cleaner protocol separation:

| Port | Direction | Purpose |
|------|-----------|---------|
| **8888** | Bidirectional | Data (commands, status, flow, PING/PONG) |
| **8889** | Arduino → Client | Heartbeat (HEARTBEAT messages only) |

### Connection

- **Protocol**: UDP
- **Data Port**: 8888
- **Heartbeat Port**: 8889
- **Arduino IP**: 192.168.50.100 (configurable)

### Message Protocol

#### Data Port (8888)

| Direction | Format | Purpose | Rate Limit |
|-----------|--------|---------|-------------|
| Client → Arduino | `C <left_us> <right_us>\n` | Thruster command | **20ms min** |
| Client → Arduino | `PING\n` | Handshake/keep-alive | No limit |
| Arduino → Client | `S <mode> <left_us> <right_us>\n` | Thruster status | 10 Hz |
| Arduino → Client | `F <freq_hz> <flow_lmin> <velocity_ms> <total_liters>\n` | Flow data | 1 Hz |
| Arduino → Client | `PONG\n` | Response to PING | On PING |

#### Heartbeat Port (8889)

| Direction | Format | Purpose | Rate |
|-----------|--------|---------|------|
| Arduino → Client | `HEARTBEAT\n` | Keep-alive | 500ms |

### Protocol Examples

```
# Command (Client → Arduino, port 8888)
C 1600 1600\n

# PING/PONG handshake (port 8888)
Client:  PING\n
Arduino: PONG\n

# Status response (Arduino → Client, port 8888)
S 1 1600 1600\n

# Flow data (Arduino → Client, port 8888)
F 25.50 5.10 0.1601 12.345\n

# Heartbeat (Arduino → Client, port 8889)
HEARTBEAT\n
```

### Message Fields

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `C` command | `<left_us>` | 1100-1900 | Left ESC pulse width in µs |
| | `<right_us>` | 1100-1900 | Right ESC pulse width in µs |
| `S` status | `<mode>` | 0=RC, 1=WiFi | Current control mode |
| | `<left_us>` | 1100-1900 | Current left output |
| | `<right_us>` | 1100-1900 | Current right output |
| `F` flow | `<freq_hz>` | float | Flow frequency in Hz |
| | `<flow_lmin>` | float | Flow rate in L/min |
| | `<velocity_ms>` | float | Velocity in m/s |
| | `<total_liters>` | float | Accumulated volume in L |

## Connection Detection & Handshake

### Dual-Port Heartbeat System

```
Client startup:
1. Bind to local port for data (ephemeral)
2. Bind to local port 8889 for heartbeat
3. Send PING to Arduino:8888
4. Receive PONG from Arduino:8888 (handshake complete)
5. Receive HEARTBEAT from Arduino:8889 (every 500ms)
6. Receive STATUS/FLOW from Arduino:8888

Continuous operation:
- Send PING every 1s (keep-alive)
- Receive HEARTBEAT every 500ms
- If 2s without any data → Arduino switches to RC mode
```

### Why Separate Ports?

1. **Protocol Clarity**: Heartbeat traffic separated from data
2. **No Interference**: PING doesn't affect command rate limiting
3. **Easy Filtering**: Can filter heartbeat traffic separately if needed
4. **Independent Channels**: Different QoS can be applied per port

## Control Characteristics

### RC vs WiFi Filtering

The system uses different filtering parameters for optimal performance with each input type:

| Characteristic | RC (Joystick) | WiFi (UDP) |
|----------------|---------------|-----------|
| Filter Alpha | 25% (smooth) | 80% (fast) |
| Max Step | 15µs/cycle | 50µs/cycle |
| Deadband | ±40µs | N/A |
| Purpose | Resist drift | Low latency |

**Why Different?**
- **RC inputs** have physical joystick drift, hand tremors, and signal noise → needs strong smoothing
- **WiFi inputs** are precise digital values with no drift → can use fast, responsive filtering

### Performance

| Metric | RC | WiFi |
|--------|-------|------|
| Response Time | ~200-300ms (smooth ramp) | ~40-80ms (fast) |
| Max Command Rate | Limited by human | 50 Hz (20ms min) |
| Drift Resistance | High (±40µs deadband) | N/A (digital) |
| Precision | Medium (joystick dependent) | High (exact values) |

## Control Priority

1. **WiFi/UDP commands** (if receiving data)
2. **RC receiver** (if UDP timeout)
3. **Neutral failsafe** (if both unavailable)

## RC Control Mode

The system supports a 9-gear mode for discrete speed control:

| Gear | Pulse Width | Description |
|------|-------------|-------------|
| 1 | 1100 µs | Reverse max |
| 2 | 1200 µs | Reverse high |
| 3 | 1300 µs | Reverse mid |
| 4 | 1400 µs | Reverse low |
| 5 | 1500 µs | Neutral stop |
| 6 | 1600 µs | Forward low |
| 7 | 1700 µs | Forward mid |
| 8 | 1800 µs | Forward high |
| 9 | 1900 µs | Forward max |

To switch to continuous mode, set `ENABLE_GEAR_MODE = false` in the code.

## WiFi Configuration

The Arduino automatically tries to connect to configured networks in order:

1. IGE-Geomatics-sense-mobile (static IP: 192.168.50.100)
2. GL-MT1300-a42 (static IP: 192.168.50.100)
3. ISSE_2.4 (static IP: 192.168.50.100)

Edit the `wifiNetworks[]` array in the code to add your networks.

## Serial Output

Connect via USB at 115200 baud for debugging:

```
=== WiFi UDP + RC Thruster Control + Flow Meter ===
RC Control Mode: Gear Mode (9 gears, 100µs intervals)

RC input pins configured
RC interrupts attached
Flow meter sensor configured on D7

Connected!
  Network: IGE-Geomatics-sense-mobile
  IP Address: 192.168.50.100
  RSSI: -45 dBm

Data UDP server started on port 8888
Heartbeat UDP server started on port 8889
Ready for UDP control commands

ESCs initialized to neutral (1500 µs)

=== System Ready ===
Control Priority: UDP > RC > Failsafe
RC Filter: 25% alpha, ±40µs deadband (smooth, drift-resistant)
WiFi Filter: 80% alpha, 20ms min interval (low latency)
Flow Meter: D7 polling mode, 1 Hz update rate
Heartbeat: 500ms (port 8889), Timeout: 2000ms
```

## Python Client Example

### Basic Client (Dual Port)

```python
import socket
import select

ARDUINO_IP = "192.168.50.100"
DATA_PORT = 8888
HEARTBEAT_PORT = 8889

# Create two sockets
data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock.bind(('', 0))  # Any available port

heartbeat_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
heartbeat_sock.bind(('', HEARTBEAT_PORT))

print(f"Data socket: {data_sock.getsockname()[1]} -> {ARDUINO_IP}:{DATA_PORT}")
print(f"Heartbeat socket: {HEARTBEAT_PORT} <- {ARDUINO_IP}:{HEARTBEAT_PORT}")

# Send handshake PING
data_sock.sendto(b"PING\n", (ARDUINO_IP, DATA_PORT))

# Listen for responses
while True:
    sockets = [data_sock, heartbeat_sock]
    readable, _, _ = select.select(sockets, [], [], 0.1)

    for sock in readable:
        data, addr = sock.recvfrom(256)
        msg = data.decode('utf-8').strip()

        if sock == heartbeat_sock:
            print(f"[HEARTBEAT] {msg}")
        elif msg == "PONG":
            print(f"[PONG] {msg}")
        elif msg.startswith('S '):
            print(f"[STATUS] {msg}")
        elif msg.startswith('F '):
            print(f"[FLOW] {msg}")
```

### Using the Test Script

A comprehensive test script is included:

```bash
# Monitor mode (listen only)
python3 udp_test.py --mode monitor

# Monitor with keep-alive (send PING every second)
python3 udp_test.py --mode monitor --keep-alive

# Interactive mode (send commands)
python3 udp_test.py --mode interactive

# 10Hz latency test (measure control delay)
python3 udp_test.py --mode hz10 --duration 10

# Thruster control test
python3 udp_test.py --mode thruster

# Heartbeat test (test for 30 seconds)
python3 udp_test.py --mode heartbeat --duration 30

# Custom ports
python3 udp_test.py --data-port 8888 --heartbeat-port 8889

# Custom IP
python3 udp_test.py --ip 192.168.50.100
```

### Interactive Commands

| Command | Action |
|---------|--------|
| `n` / `neutral` | Send neutral (1500, 1500) |
| `f` / `forward` | Send forward (1600, 1600) |
| `b` / `backward` | Send backward (1400, 1400) |
| `l` / `left` | Turn left (1500, 1600) |
| `r` / `right` | Turn right (1600, 1500) |
| `ping` | Send PING (handshake) |
| `1550 1600` | Custom command |
| `stats` | Show statistics |
| `q` / `quit` | Exit |

## Jetson/ROS Integration

### Key Implementation Points

1. **Dual Socket Setup**: Create two UDP sockets, one for data (port 8888) and one for heartbeat (port 8889)
2. **Initial Handshake**: Send `PING` immediately after connection
3. **Keep-Alive**: Send `PING` every 1 second to maintain connection
4. **Use `select()`**: Monitor both sockets simultaneously for incoming data
5. **Handle Rate Limiting**: WiFi commands have 20ms minimum interval, PING does not

### C++ ROS Node Structure

```cpp
// Thread 1: UDP receive (both ports)
void udpReceiveThread() {
    while (running) {
        // Use select() to monitor both sockets
        // Process messages based on source port
    }
}

// Thread 2: Keep-alive
void keepAliveThread() {
    while (running) {
        send_ping(data_sock);
        sleep(1);
    }
}

// Main: ROS publishers/subscribers
// Publish: /thruster_status, /flow_data
// Subscribe: /cmd_thruster
```

## Calibration

### Flow Meter

Modify these constants if needed:

```cpp
const float K_HZ_PER_LMIN = 5.0f;       // f = 5*Q (frequency per flow rate)
const float PULSES_PER_L = 300.0f;      // 1L ≈ 300 pulses
const float DIAMETER_M = 0.026f;        // 26 mm pipe diameter
```

### Control Response Tuning

The following constants can be adjusted to change control behavior:

```cpp
// WiFi (Low Latency)
const int WIFI_FILTER_ALPHA = 80;     // Higher = faster response (20-100%)
const int WIFI_MAX_STEP_US = 50;      // Higher = faster ramp (10-100µs)
const unsigned long MIN_CMD_INTERVAL_MS = 20;  // Min time between commands

// RC (Smooth, Drift-Resistant)
const int RC_FILTER_ALPHA = 25;       // Lower = smoother (10-50%)
const int RC_MAX_STEP_US = 15;        // Lower = slower ramp (5-50µs)
const int DEADBAND_US = 40;            // Deadband around center (20-100µs)
```

**Tuning Guide:**
- Increase `WIFI_FILTER_ALPHA` for smoother WiFi control (less jitter)
- Increase `WIFI_MAX_STEP_US` for faster WiFi changes
- Decrease `RC_FILTER_ALPHA` for faster RC response
- Increase `DEADBAND_US` if joystick causes unintended movement

## Troubleshooting

### "Jetson: OFFLINE" in Serial Monitor

- Check client is sending data to port 8888
- Verify heartbeat is being received on port 8889
- Send a PING command to trigger response
- Verify IP address configuration matches

### No PONG response to PING

- Ensure Arduino code is uploaded with PING handler
- Check data port (8888) is correct
- Verify firewall is not blocking UDP

### No heartbeat on port 8889

- Verify heartbeat port (8889) is correct
- Check client is bound to port 8889
- Ensure Arduino has received at least one packet (to learn client address)

### Command rate limiting

- WiFi commands have **20ms minimum interval** (max 50 Hz)
- PING is NOT rate limited (can be sent anytime)
- Use keep-alive mode to maintain connection
- RC control has **separate filtering** (25% alpha, smooth) to resist joystick drift

### No flow data

- Verify flow sensor connected to D7
- Check sensor power supply
- Increase `FLOW_UPDATE_INTERVAL_MS` for testing

### RC control not working

- Verify RC receiver connected to D2 and D3
- Check RC receiver is bound (TX light on)
- Verify PWM output range (1000-2000µs)

## License

MIT License
