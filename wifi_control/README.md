# WiFi Control Module

WiFi-based control system for dual thrusters using Arduino UNO R4 WiFi and ROS communication.

## Development Process

### Phase 1: WiFi Connection Setup
- **Objective**: Establish stable WiFi connection to ISSE_2.4 network
- **Implementation**:
  - Used `WiFiS3.h` library (native to Arduino UNO R4)
  - Configured static IP: 192.168.50.100
  - Network gateway: 192.168.50.1
  - Connection retry logic with 20 attempts
  - Serial debug output for connection status and signal strength (RSSI)
  
- **Result**: ✅ Successfully connects to WiFi on startup and auto-reconnects on disconnection

### Phase 2: TCP Client Communication
- **Objective**: Establish bidirectional communication with ROS computer at 192.168.50.108
- **Implementation**:
  - Implemented TCP client connecting to 192.168.50.108:8888
  - Auto-reconnection every 5 seconds if connection fails
  - Serial port 8888 for ROS communication
  - Error handling for connection failures

- **Protocol**:
  ```
  Command (ROS → Arduino):  C <left_us> <right_us>\n
  Status (Arduino → ROS):   S <left_us> <right_us>\n
  ```

- **Result**: ✅ Successfully tested with `netcat` commands

### Phase 3: Thruster Command Parsing
- **Objective**: Parse incoming ROS commands and control ESC signals
- **Implementation**:
  - Command format: `C <left_us> <right_us>` (microseconds)
  - Input validation with constrain() to ESC range [1100-1900µs]
  - Center position (neutral): 1500µs
  - Full reverse: 1100µs
  - Full forward: 1900µs
  - Buffer overflow protection (max 50 chars)
  - Newline-based command termination

- **Control Logic**:
  ```cpp
  - If connected: Follow ROS commands
  - If disconnected: Set thrusters to neutral (1500µs)
  ```

- **Result**: ✅ Successfully receives and executes thrust commands

### Phase 4: Status Reporting
- **Objective**: Send real-time thruster status back to ROS
- **Implementation**:
  - Status format: `S <left_us> <right_us>\n`
  - Update frequency: 100ms (10 Hz)
  - Only sends when connected to ROS
  - Tracks current thruster positions

- **Result**: ✅ Real-time status streaming to ROS computer

## Hardware Configuration

### Pins Used
- **Pin 2**: RC Left Channel Input (not used in WiFi mode)
- **Pin 3**: RC Right Channel Input (not used in WiFi mode)
- **Pin 9**: Right ESC Signal Output
- **Pin 10**: Left ESC Signal Output

### ESC Specifications
- **Control Range**: 1100-1900µs
- **Neutral Position**: 1500µs
- **Min (Full Reverse)**: 1100µs
- **Max (Full Forward)**: 1900µs
- **Calibration**: 2-second neutral hold on startup

## Network Configuration

### Arduino UNO R4 WiFi
- **SSID**: ISSE_2.4
- **Password**: TheAnswerIs42
- **IP Address**: 192.168.50.100 (static)
- **Gateway**: 192.168.50.1
- **Subnet**: 255.255.255.0

### ROS Computer
- **IP Address**: 192.168.50.108
- **Server Port**: 8888 (to be started by user)

## Testing & Validation

### Test 1: WiFi Connection
1. Upload sketch to Arduino
2. Open Serial Monitor (115200 baud)
3. Verify: "Connected to WiFi!" message with IP address

**Expected Output**:
```
Starting WiFi connection...
Connected to WiFi!
IP Address: 192.168.50.100
RSSI (signal strength): -45 dBm
ESCs initialized to neutral (1500 µs)
```

### Test 2: TCP Communication
1. Start receive listener on ROS computer:
   ```bash
   python3 test_client.py
   ```

2. You should see:
   ```
   Connecting to Arduino at 192.168.50.100:8888...
   ✓ Connected to Arduino!
   ← Status: Left=1500µs | Right=1500µs
   ← Status: Left=1500µs | Right=1500µs
   ```

3. Send command from same terminal:
   ```
   cmd> C 1600 1500
   ```

**Expected Output**:
```
→ Sent: C 1600 1500
← Status: Left=1600µs | Right=1500µs
← Status: Left=1600µs | Right=1500µs
```

### Test 3: Status Reporting
Run the test client:
```bash
python3 test_client.py
```

You will see real-time status updates every 100ms:
```
← Status: Left=1500µs | Right=1500µs
← Status: Left=1500µs | Right=1500µs
← Status: Left=1500µs | Right=1500µs
```

### Test 4: Interactive Command Testing
While the test client is running, send commands:
```
cmd> forward        # Full speed ahead
cmd> stop           # Stop thrusters
cmd> reverse        # Full speed reverse
cmd> C 1600 1400    # Custom values
cmd> test           # Run automated test sequence
```

## Architecture

### TCP Server (Arduino)
- **Mode**: Server (listening on 192.168.50.100:8888)
- **Behavior**: Waits for ROS client to connect
- **Status**: "WAITING" when no client, "CONNECTED" when client attached
- **Failsafe**: Thrusters set to neutral (1500µs) when client disconnects
```cpp
WiFi.config(local_IP, gateway, subnet);
WiFi.begin(ssid, password);
// Auto-reconnect on disconnection
```

#### 2. TCP Client Connection
```cpp
WiFiClient rosClient;
rosClient.connect(ros_host, ros_port);
// Retry every 5 seconds
```

#### 3. Command Processing
```cpp
// Parse "C <left_us> <right_us>"
sscanf(inputBuffer.c_str(), "C %d %d", &leftUs, &rightUs);
// Constrain to valid ESC range
leftUs = constrain(leftUs, ESC_MIN, ESC_MAX);
```

#### 4. Status Sending
```cpp
// Send "S <left_us> <right_us>" every 100ms
snprintf(statusBuf, sizeof(statusBuf), "S %d %d\n", currentLeftUs, currentRightUs);
rosClient.print(statusBuf);
```

#### 5. ESC Control
```cpp
escL.attach(ESC_LEFT_OUT);
escR.attach(ESC_RIGHT_OUT);
escL.writeMicroseconds(currentLeftUs);
escR.writeMicroseconds(currentRightUs);
```

## Performance Metrics

- **WiFi Connection Time**: ~2-3 seconds
- **TCP Reconnection Interval**: 5 seconds
- **Command Latency**: ~5-10ms (WiFi overhead)
- **Status Update Rate**: 10 Hz (100ms interval)
- **Max Command Frequency**: 100+ Hz (network limited)

## Known Limitations

1. **No RC Fallback**: Unlike `ros_control.ino`, this version doesn't support RC receiver fallback
2. **Single Client**: Only one ROS connection at a time
3. **No Command Timeout**: If commands stop, thrusters stay at last command value until disconnection
4. **Memory**: Limited SRAM (32KB) may restrict buffer sizes

## Future Improvements

1. Add RC receiver fallback with priority logic
2. Implement command timeout (e.g., failsafe after 500ms without commands)
3. Add command acknowledgment mechanism
4. Support multiple simultaneous clients
5. Implement error correction codes for reliability
6. Add battery voltage monitoring
7. Support UDP as alternative to TCP

## Dependencies

- **Hardware**: Arduino UNO R4 WiFi, 2x ESCs, Power supply
- **Software**:
  - Arduino IDE 2.0+
  - WiFiS3 library (built-in with Arduino UNO R4)
  - Servo library (built-in)

## Troubleshooting

### Issue: Cannot connect to WiFi
- Verify SSID and password are correct
- Check WiFi network is 2.4GHz (not 5GHz)
- Verify Arduino is within WiFi range
- Check Serial output for error messages

### Issue: Cannot connect to ROS computer
- Verify ROS computer is on same network (192.168.50.x)
- Check server is running: `nc -l 8888`
- Verify IP address 192.168.50.108 is correct
- Check firewall isn't blocking port 8888

### Issue: Commands not working
- Verify command format: `C <left> <right>\n`
- Check values are in range: 1100-1900µs
- Verify ESC connections (pins 9 and 10)
- Test with Serial Monitor to isolate issue

### Issue: Status not appearing
- Verify ROS connection is active
- Check status is being sent every 100ms
- Verify buffer is receiving data correctly

## Files

- `wifi_control.ino` - Main Arduino sketch
- `README.md` - This documentation

## Related Files

- `../original/original.ino` - RC-only control version
- `../ros_control/ros_control.ino` - ROS + RC mixed control version
- `../README.md` - Project overview
