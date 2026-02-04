# Changelog

All notable changes to the thruster and speed control project will be documented in this file.

## [Unreleased]

### Added
- **Dual-Port UDP Architecture** - Separated data and heartbeat communication onto different ports
  - Port 8888: Data (commands, status, flow, PING/PONG)
  - Port 8889: Heartbeat (HEARTBEAT messages only)
- **PING/PONG Protocol** - Handshake and keep-alive mechanism that doesn't affect thruster commands
  - PING is NOT subject to command rate limiting (100ms minimum)
  - PONG response confirms Arduino received the PING
- **Keep-Alive Thread** - Background thread in Python client that sends PING every 1s
- **Enhanced Python Test Script** - Comprehensive UDP testing utility
  - Monitor mode: Listen for all messages
  - Heartbeat test mode: Test heartbeat reception for specified duration
  - Interactive mode: Send commands and see responses
  - Keep-alive support: `--keep-alive` flag

### Changed
- **Arduino Protocol Documentation** - Updated comments to reflect dual-port architecture
- **WiFi Reconnection** - Now restarts both UDP servers (data and heartbeat) on reconnect
- **Command Rate Limiting** - Fixed bug where `continue` statement skipped PING handler
  - Changed from skipping entire loop to only skipping C command processing
  - PING commands now work regardless of rate limit state

### Fixed
- **Rate Limiting Bug** - PING handler was being skipped by `continue` statement in C command processing
  - Old behavior: Rate-limited C commands used `continue`, skipping all subsequent handlers
  - New behavior: Rate limit check only affects C command processing, PING handler always executes

### Technical Details

#### Dual-Port Architecture Benefits
1. **Protocol Clarity** - Heartbeat traffic separated from data
2. **No Interference** - PING doesn't affect command rate limiting
3. **Easy Filtering** - Can filter heartbeat traffic separately
4. **Independent Channels** - Different QoS can be applied per port

#### Protocol Changes

**Before (Single Port):**
```
Port 8888 (bidirectional):
- C <left_us> <right_us>  (commands)
- S <mode> <left_us> <right_us>  (status)
- F <freq> <flow> <velocity> <total>  (flow data)
- HEARTBEAT  (heartbeat)
```

**After (Dual Port):**
```
Port 8888 (bidirectional data):
- C <left_us> <right_us>  (commands, rate limited to 100ms)
- PING  (handshake/keep-alive, NO rate limit)
- PONG  (response to PING)
- S <mode> <left_us> <right_us>  (status)
- F <freq> <flow> <velocity> <total>  (flow data)

Port 8889 (Arduino → Client heartbeat):
- HEARTBEAT  (every 500ms)
```

#### Arduino Code Changes

**File:** `thruster_and_speed.ino`

**Added Constants:**
```cpp
const unsigned int HEARTBEAT_PORT = 8889;  // Heartbeat port
WiFiUDP udpHeartbeat;                       // Second UDP object for heartbeat
```

**Modified Functions:**
- `sendHeartbeat()` - Now sends to port 8889 using `udpHeartbeat`
- `readUdpCommands()` - Added PING command handler
- `setup()` - Starts both UDP servers
- `reconnectWiFi()` - Restarts both UDP servers on reconnect

**New PING Handler:**
```cpp
if (cmdBufferIndex > 0 && strcmp(cmdBuffer, "PING") == 0) {
  Serial.println("UDP: PING received");
  udp.beginPacket(jetsonIp, jetsonPort);
  udp.print("PONG\n");
  udp.endPacket();
}
```

#### Python Client Changes

**File:** `udp_test.py`

**New Methods:**
- `send_ping()` - Send PING for handshake/keep-alive
- `start_keep_alive()` - Start background keep-alive thread
- `_keep_alive_loop()` - Background thread that sends PING every second

**Modified Methods:**
- `__init__()` - Now accepts `data_port` and `heartbeat_port` parameters
- `connect()` - Creates two UDP sockets (data and heartbeat)
- `disconnect()` - Closes both sockets
- `_receive_loop()` - Uses `select()` to monitor both sockets simultaneously
- `send_handshake()` - Now sends PING instead of C 1500 1500

**New Command-Line Arguments:**
- `--data-port` - Specify data port (default: 8888)
- `--heartbeat-port` - Specify heartbeat port (default: 8889)
- `--keep-alive` - Enable keep-alive mode

**New Interactive Commands:**
- `ping` - Send PING manually

### Migration Guide

If you have existing code using the old single-port protocol:

1. **Update your UDP socket setup** to use two ports
2. **Send PING on startup** to trigger Arduino response
3. **Bind to port 8889** locally to receive heartbeats
4. **Use `select()`** (or equivalent) to monitor both ports

**Old Code:**
```python
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 54321))
sock.sendto(b"C 1500 1500\n", (ARDUINO_IP, 8888))
```

**New Code:**
```python
import socket
import select

# Data socket
data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock.bind(('', 0))  # Ephemeral port

# Heartbeat socket
heartbeat_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
heartbeat_sock.bind(('', 8889))

# Send handshake PING
data_sock.sendto(b"PING\n", (ARDUINO_IP, 8888))

# Listen on both ports
while True:
    readable, _, _ = select.select([data_sock, heartbeat_sock], [], [], 0.1)
    for sock in readable:
        data, addr = sock.recvfrom(256)
        # Process message...
```

### Testing

Run the test script to verify dual-port functionality:

```bash
# Basic monitor test
python3 udp_test.py --mode monitor

# With keep-alive (recommended)
python3 udp_test.py --mode monitor --keep-alive

# Heartbeat stress test
python3 udp_test.py --mode heartbeat --duration 30
```

Expected output:
- `[PONG]` - PING response on port 8888
- `[HEARTBEAT]` - Heartbeat on port 8889
- `[STATUS]` - Status updates on port 8888
- `[FLOW]` - Flow data on port 8888
