# WiFi + RC Control Module

Hybrid control system combining WiFi/ROS commands and RC receiver input with automatic priority switching.

## Features

### Dual Control Modes
1. **WiFi Mode** (Priority 1)
   - TCP server on port 8888
   - Receives commands from ROS computer
   - Active when connected and receiving commands

2. **RC Mode** (Priority 2)
   - Reads PWM signals from RC receiver
   - Automatic fallback when WiFi unavailable
   - Standard RC mapping (1000-2000µs → 1100-1900µs)

3. **Failsafe** (Priority 3)
   - Neutral position (1500µs) when both unavailable
   - Protects against signal loss

## Control Priority Logic

```
┌─────────────────────────────────────────┐
│ Is WiFi client connected?               │
│ AND received command within 500ms?      │
└──────────┬──────────────────────────────┘
           │
     Yes   │   No
    ┌──────▼──────┐         ┌──────────────┐
    │ WiFi Mode   │         │ Is RC signal │
    │ (Mode = 1)  │         │ valid?       │
    │             │         └──────┬───────┘
    │ Use WiFi    │                │
    │ commands    │      Yes   No  │
    └─────────────┘        ┌───▼──▼────┐
                          │ RC Mode    │
                          │ (Mode = 0) │
                          │            │
                          │ Use RC or  │
                          │ Failsafe   │
                          └────────────┘
```

## Hardware Configuration

### Pins
- **Pin 2**: RC Left Channel Input
- **Pin 3**: RC Right Channel Input  
- **Pin 9**: Right ESC Output
- **Pin 10**: Left ESC Output

### Network
- **SSID**: ISSE_2.4
- **IP**: 192.168.50.100 (static)
- **Port**: 8888 (TCP Server)

## Protocol

### WiFi Commands (ROS → Arduino)
```
C <left_us> <right_us>\n
```
- Range: 1100-1900µs
- Example: `C 1600 1500\n`

### Status Updates (Arduino → ROS)
```
S <mode> <left_us> <right_us>\n
```
- mode: 0=RC, 1=WiFi
- Frequency: 10Hz (every 100ms)
- Example: `S 1 1600 1500\n`

## Operation Modes

### Mode 1: WiFi Control
```
WiFi Client Connected → Receiving Commands → WiFi Active
Status: "Mode: WiFi | WiFi: CONNECTED | L=1600 R=1500"
```

### Mode 2: RC Fallback
```
WiFi Timeout (>500ms) OR No WiFi → RC Active
Status: "Mode: RC | WiFi: DISCONNECTED | L=1500 R=1500"
```

### Mode 3: Full Failsafe
```
No WiFi AND No RC → Neutral Position
Status: "Mode: RC | WiFi: DISCONNECTED | L=1500 R=1500"
```

## Timeout Settings

| Timeout | Duration | Effect |
|---------|----------|--------|
| RC Signal | 200ms | Switch from RC value to neutral |
| WiFi Command | 500ms | Switch from WiFi to RC mode |
| Status Send | 100ms | WiFi status update interval |

## Testing

### Test 1: RC Only (No WiFi)
1. Power on Arduino without WiFi network
2. Use RC transmitter to control thrusters
3. Verify Serial shows "Mode: RC"

### Test 2: WiFi Only (No RC)
1. Connect to WiFi
2. Don't connect RC receiver
3. Send WiFi commands from ROS
4. Verify Serial shows "Mode: WiFi"

### Test 3: Priority Switching
1. Start with RC control active
2. Connect WiFi client and send commands
3. Verify switches to "Mode: WiFi"
4. Disconnect WiFi or stop commands
5. Verify automatically switches back to "Mode: RC"

### Test 4: Full Failsafe
1. Disconnect both WiFi and RC
2. Verify thrusters go to neutral (1500µs)

## Usage Examples

### WiFi Control (Python)
```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.50.100', 8888))

# Send commands
sock.send(b'C 1600 1500\n')  # Forward right

# Receive status
data = sock.recv(256).decode()
print(data)  # "S 1 1600 1500"
```

### RC Control
- Standard RC transmitter with 2 channels
- Connect to RC receiver outputs
- Left stick → Pin 2
- Right stick → Pin 3

## Advantages over Single-Mode Systems

| Feature | wifi_control.ino | ros_control.ino | **wifi_RC_control.ino** |
|---------|------------------|-----------------|------------------------|
| WiFi Control | ✓ | ✗ | ✓ |
| RC Control | ✗ | ✓ | ✓ |
| Wireless Freedom | ✓ | ✗ | ✓ |
| RC Failsafe | ✗ | ✓ | ✓ |
| Status Reporting | ✓ | ✓ | ✓ |
| Auto Mode Switch | N/A | Manual | **Automatic** |

## Troubleshooting

### WiFi won't switch to active
- Check WiFi commands arriving within 500ms
- Verify client is connected
- Check Serial output for "WiFi Command: ..."

### RC won't work
- Verify RC receiver is powered and bound
- Check PWM signals on Pin 2/3 with oscilloscope
- Confirm RC signal is 1000-2000µs range
- Check Serial output shows valid RC values

### Stuck in failsafe
- Both WiFi and RC unavailable
- Check WiFi connection status
- Verify RC transmitter is on and bound
- Check Serial debug output

## Code Architecture

### Main Loop Flow
```cpp
1. handleClientConnections()  // Check for new WiFi clients
2. readRcInputs()             // Read RC PWM (blocking ~20ms)
3. readWifiCommands()         // Parse WiFi commands (non-blocking)
4. determineControlMode()     // WiFi > RC > Failsafe logic
5. updateThrusters()          // Apply final values to ESCs
6. sendStatus()               // Send status to WiFi client
7. delay(5)                   // Small delay for stability
```

### Key Functions

#### `readRcInputs()`
- Uses `pulseIn()` to read PWM
- Maps 950-2000µs to 1100-1900µs
- Applies deadband around center
- Implements timeout failsafe

#### `readWifiCommands()`
- Non-blocking parse of TCP stream
- Command format validation
- Constrains values to safe range

#### `determineControlMode()`
- Evaluates control priority
- Sets `currentMode` (0=RC, 1=WiFi)
- Updates `currentLeftUs` and `currentRightUs`

## Safety Features

1. **Input Validation**: All commands constrained to 1100-1900µs
2. **Timeout Protection**: Automatic failsafe on signal loss
3. **Deadband**: 25µs around center prevents drift
4. **Priority Logic**: Prevents control conflicts
5. **Status Monitoring**: Real-time mode reporting

## Performance

- **Loop Rate**: ~100Hz (10ms cycle with RC reads)
- **WiFi Latency**: 5-10ms
- **RC Latency**: 20ms (pulseIn blocking time)
- **Status Update**: 10Hz (100ms interval)
- **Mode Switch**: <100ms response time

## Files

- `wifi_RC_control.ino` - Main sketch
- `README.md` - This documentation

## Related Modules

- `../wifi_control/` - WiFi-only control
- `../ros_control/` - USB Serial + RC control
- `../original/` - RC-only control
