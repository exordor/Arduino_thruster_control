# WiFi + RC Hybrid Control - Multi-Configuration Management System

> A dual-mode thruster control system for Arduino UNO R4 WiFi, supporting WiFi and RC remote control with multi-network configuration management and web-based setup interface.

## 🎯 Key Features

- **Multi-WiFi Configuration** — Save up to 5 WiFi network profiles with different priorities
- **Web Configuration Portal** — Manage profiles via phone/computer browser at 192.168.4.1
- **Hybrid Dual-Mode Control** — WiFi and RC remote control, WiFi takes priority when active
- **Auto Startup** — Automatically enters AP mode if no profile exists; auto-connects if configured
- **Background Auto-Reconnect** — Automatically retries WiFi connection every 30 seconds if disconnected
- **EEPROM Persistence** — All configurations permanently saved to device memory

## ⚡ Quick Start

### 1. Upload Firmware
```bash
Arduino IDE → Open wifi_manager_RC_control.ino → Upload
```

### 2. First Boot
- **No config**: Automatically starts AP (SSID: `ArduinoR4-Config`, Password: `12345678`)
- **Manual entry**: Send `CONFIG` command via Serial Monitor

### 3. Add WiFi Profile
1. Connect phone/computer to AP: **ArduinoR4-Config**
2. Open browser: **http://192.168.4.1**
3. Click "Add WiFi" tab, enter SSID and password
4. Click "Add Profile" button to save
5. AP automatically closes, device connects to new network

### 4. Test Thrusters
- **RC Mode** — Receives PWM signals on CH_LEFT_IN (pin 2) and CH_RIGHT_IN (pin 3)
- **WiFi Mode** — Send TCP command to port 8888: `C <left_us> <right_us>\n`
  ```bash
  # Example using netcat
  echo "C 1600 1400" | nc 192.168.50.100 8888
  
  # Example using Python
  import socket
  s = socket.socket()
  s.connect(('192.168.50.100', 8888))
  s.sendall(b'C 1600 1400\n')
  # Receive status
  print(s.recv(1024))  # S 1 1600 1400
  s.close()
  ```

## 🎮 Control Logic

### Priority-Based WiFi Connection
The system **automatically tries all saved WiFi profiles in priority order** on startup:

1. **On Boot**: Loads all profiles from EEPROM
2. **Sort by Priority**: Arranges profiles by priority (highest first)
3. **Try Each**: Attempts to connect to each profile in order (30 seconds each)
4. **Success**: Stops trying once any profile connects successfully
5. **Fallback**: If all fail, enables auto-retry every 30 seconds in background

**Example**: If you have 3 profiles:
```
Profile A: Priority 100 ← Tries first (highest)
Profile B: Priority  50 ← Tries second if A fails
Profile C: Priority  10 ← Tries third if A & B fail
```

At startup, it will try A first (30s), then B (30s), then C (30s), for a maximum 90-second boot sequence.

### Serial Connection Status
```
[WiFi] Attempting connections in priority order:
[WiFi] Attempt 1/3 - Priority 100: HomeNetwork
[WiFi] Connecting to: HomeNetwork (Priority: 100)
............ (connecting dots)
[WiFi] ✓ Connected!
[WiFi] IP: 192.168.50.100
```

### Mode Priority System
For **thruster control**, WiFi has priority over RC:

```
┌─────────────────────────────────┐
│   WiFi Command Valid? (age < 500ms)
├─────────────────────────────────┤
│  YES → Use WiFi values          │ ← WiFi takes priority
│  NO ↓                            │
├─────────────────────────────────┤
│   Both RC Channels Valid?        │
│   (PWM: 950-2000µs, age < 200ms)│
├─────────────────────────────────┤
│  YES → Use RC values             │ ← RC fallback
│  NO ↓                            │
├─────────────────────────────────┤
│   Center Position (1500µs)      │ ← Safe default
└─────────────────────────────────┘
```

### Control Flow Diagram
```
Every 10ms loop cycle:
  1. Read RC Channels
     ↓ (pulseIn on pins 2, 3)
  2. Process WiFi Client Commands
     ↓ (TCP server on port 8888)
  3. Evaluate Control Mode
     ↓ (WiFi priority > RC > center)
  4. Update ESC Outputs
     ↓ (writeMicroseconds on pins 9, 10)
  5. Send Status to Client
     ↓ (format: "S <mode> <left_us> <right_us>\n")
  6. Handle Serial Commands
     ↓ (CONFIG, LIST, SWITCH, etc.)
  7. Auto-Reconnect WiFi
     ↓ (retry every 30 seconds if disconnected)
```

### Timeout Constants
| Constant | Value | Purpose |
|----------|-------|---------|
| `RC_FAILSAFE_MS` | 200 ms | RC signal validity window |
| `WIFI_CMD_TIMEOUT_MS` | 500 ms | WiFi command validity window |
| `WIFI_RETRY_INTERVAL_MS` | 30 s | Auto-reconnect interval |
| `STATUS_SEND_INTERVAL_MS` | 100 ms | Status update frequency |
| `PULSE_TIMEOUT_US` | 25 ms | RC pulse timeout |

### WiFi Command Format
Send TCP commands to `<device_ip>:8888`:

**Command Format:**
```
C <left> <right>\n

Example:
C 1600 1400
C 1500 1500
```

- `C`: Command marker (required)
- `<left>`: Left thruster PWM (1100-1900 µs)
- `<right>`: Right thruster PWM (1100-1900 µs)
- `\n`: Newline terminator (required)

**Status Response (automatic, 10Hz):**
```
S <mode> <left> <right>\n

Example:
S 1 1600 1400
  │  │   └────── Right thruster current output
  │  └────────── Left thruster current output
  └────────────── Mode: 1=WiFi active, 0=RC active
```

**Connection Behavior:**
- **First connection**: Arduino prints `[TCP] New WiFi client connected!`
- **Each command**: Arduino prints `[TCP] WiFi Command: Left=1600 Right=1400`
- **Status stream**: Client receives status every 100ms (10Hz)
- **Persistent**: No reconnection needed, stays open until client disconnects
- **Failsafe**: If no commands for 500ms, switches back to RC mode (connection stays open)

### RC Input Processing
```
Raw RC Input (pin 2, 3)
  ↓ pulseIn() measures pulse width
Valid Range: 950-2000 µs
  ↓
Clamp to ESC Range: 1100-1900 µs
  ↓
Apply Failsafe (200ms timeout)
  ↓
Output to ESC (pins 9, 10)
```

## 🔧 Serial Commands

Enter commands in Arduino IDE Serial Monitor (115200 baud):

| Command | Function |
|---------|----------|
| `CONFIG` | Start configuration AP (web setup) |
| `LIST` | Display all saved profiles |
| `STATUS` | Show WiFi connection status and signal strength |
| `SWITCH <i>` | Switch to profile index i (0-4) |
| `DELETE <i>` | Delete profile index i |
| `RESET` | Clear all profiles and restart |

### Example Usage
```
> CONFIG
[AP] Starting Configuration Access Point
[AP] SSID: ArduinoR4-Config, Password: 12345678
[AP] Visit: http://192.168.4.1

> LIST
========== WiFi Profiles ==========
Total: 2 / 5
[0] ✓ ACTIVE
  SSID: HomeNetwork
  Priority: 1
  IP: 192.168.50.100
  Port: 8888
[1]
  SSID: OfficeNetwork
  Priority: 1
  IP: 192.168.50.101
  Port: 8888
===================================

> SWITCH 1
[WiFi] Loaded profile 1: OfficeNetwork
[WiFi] Connecting to: OfficeNetwork
```

## 📊 System Status Output

Every 5 seconds, the system prints status:
```
[Status] Mode: WiFi | L: 1600 | R: 1400 | WiFi: ✓ | Client: ✓
         └─────┬────┘ └──┬──┘ └──┬──┘ └───┬───┘ └────┬─────┘
            Control    Left     Right   WiFi      Client
            Mode      Thruster Thruster Status    Status
```

Status breakdown:
- **Mode**: WiFi (WiFi command active) or RC (RC input active)
- **L/R**: Current output in microseconds (1100-1900 µs)
- **WiFi**: ✓ (Connected) or ✗ (Disconnected)
- **Client**: ✓ (TCP client connected) or ✗ (No client)

## 🌐 Web Interface

Visit **http://192.168.4.1** to access the configuration portal:

### Profiles Tab
- Display all saved WiFi configurations
- **Switch Button** — Activate a different profile
- **Delete Button** — Remove a profile

### Add WiFi Tab
- **SSID**: Network name (required)
- **Password**: Network password (optional for open networks)
- **Priority**: Connection priority (0-255, **higher = attempted first** on boot)
  - Profile with Priority 100 tries before Priority 50
  - Useful for switching between home/office WiFi automatically
- **Static IP**: Fixed IP address for device
- **Gateway**: Router gateway IP
- **Subnet**: Network subnet mask
- **Port**: TCP communication port (default 8888)

## 🔌 Pin Configuration

| Function | Pin | Type | Description |
|----------|-----|------|-------------|
| CH_LEFT_IN | 2 | Input | RC left channel PWM input |
| CH_RIGHT_IN | 3 | Input | RC right channel PWM input |
| ESC_LEFT_OUT | 10 | Output | Left thruster ESC signal |
| ESC_RIGHT_OUT | 9 | Output | Right thruster ESC signal |

### Typical Wiring
```
RC Receiver:
  CH1 → Pin 2 (CH_LEFT_IN)
  CH2 → Pin 3 (CH_RIGHT_IN)
  GND → GND

ESC Controllers:
  Left ESC Signal → Pin 10 (ESC_LEFT_OUT)
  Right ESC Signal → Pin 9 (ESC_RIGHT_OUT)
  GND → GND (common)

Power:
  ESC Power → 5-12V (depends on ESC spec)
  Arduino USB or external 5V
```

## 🐛 Troubleshooting

### ✅ ROS/TCP Connection Working
The system now uses the **tested and stable** TCP communication logic from `wifi_RC_control.ino`:
- **Persistent connections** — No reconnect needed between commands
- **Status streaming** — Receives `S <mode> <left> <right>` at 10Hz (every 100ms)
- **Command format** — Send `C 1600 1400` (with newline `\n`)
- **No artificial timeouts** — Connection stays until client disconnects

Expected behavior:
```
[TCP] New WiFi client connected!
[TCP] WiFi Command: Left=1600 Right=1400
[Status] Mode: WiFi | L: 1600 | R: 1400 | WiFi: ✓ | Client: ✓
```

### WiFi Cannot Connect
- Verify SSID and password are correct
- Check signal strength: `STATUS` command shows RSSI in dBm
- Move closer to router if signal is weak (<-80 dBm)
- Ensure WiFi network is 2.4GHz (R4 doesn't support 5GHz)

### AP Doesn't Close After Config
- Check if device actually connected to WiFi (see serial output)
- If "WiFi ✗ Failed", connection timeout may be too short
- Move device closer to router and try again

### Thrusters Not Responding
- **RC Mode**: Verify RC receiver connected to pins 2 and 3
- **RC Mode**: Check PWM signals are within 950-2000 µs range
- **WiFi Mode**: Confirm TCP connection successful (`Client: ✓`)
- **WiFi Mode**: Send valid command: `C 1500 1500` or `1500 1500`
- Check ESC calibration (may require throttle range setup)

### No Status Output
- Confirm baud rate is 115200
- Press Arduino reset button
- Check if device is in configuration mode (AP running)

### Frequent WiFi Disconnection
- Check WiFi signal strength (goal: > -70 dBm)
- Try different location away from interference
- Update WiFi router firmware
- Check if password has special characters (may cause parsing issues)

## 📁 File Reference

| File | Purpose |
|------|---------|
| `wifi_manager_RC_control.ino` | Main program (complete, ready to upload) |
| `wifi_config_web_interface.cpp` | Web UI code (disabled, reference only) |
| `README.md` | This documentation |

### EEPROM Memory Layout
```
Address Range | Size | Content
──────────────┼──────┼─────────────────────
0x00 - 0x00   | 1B   | Magic marker (0xA5)
0x01 - 0x01   | 1B   | Version (1)
0x02 - 0x02   | 1B   | Config count
0x03 - 0x03   | 1B   | Active config index
0x04 - 0x513  | 1040B| Config slots (5 × 208B each)
```

Each config slot contains:
- Priority: 1B
- SSID: 32B
- Password: 64B
- IP: 4B
- Gateway: 4B
- Subnet: 4B
- Port: 2B

## ⚙️ Advanced Configuration

### Adjust Timeout Values
Edit these constants in the sketch:
```cpp
const unsigned long RC_FAILSAFE_MS = 200;        // RC signal timeout
const unsigned long WIFI_CMD_TIMEOUT_MS = 500;   // WiFi command timeout
const unsigned long WIFI_RETRY_INTERVAL_MS = 30000; // Reconnect interval
```

### Change ESC Pulse Range
Modify these values based on your ESC specifications:
```cpp
const int RX_VALID_MIN = 950;   // Minimum valid RC pulse
const int RX_VALID_MAX = 2000;  // Maximum valid RC pulse
const int ESC_MIN = 1100;       // ESC minimum pulse (full reverse)
const int ESC_MID = 1500;       // ESC center position (neutral)
const int ESC_MAX = 1900;       // ESC maximum pulse (full forward)
```

### WiFi Connection Timeout
Current setting: 60 attempts × 500ms = 30 seconds
To adjust, change in `connectToWiFi()`:
```cpp
while (WiFi.status() != WL_CONNECTED && attempts < 60)  // Change 60
```

## 📚 Technical Details

- **Microcontroller**: Arduino UNO R4 WiFi (Renesas RA4M1)
- **WiFi Library**: WiFiS3 (built-in)
- **Storage**: 1024-byte EEPROM (limited writes, ~10,000 cycles)
- **Max Profiles**: 5 configurations
- **Default TCP Port**: 8888
- **AP Default Credentials**: SSID: `ArduinoR4-Config`, Password: `12345678`
- **AP Default IP**: 192.168.4.1
- **Serial Baud Rate**: 115200

## 🔗 References

- [Arduino UNO R4 WiFi Documentation](https://docs.arduino.cc/hardware/uno-r4-wifi)
- [WiFiS3 Library Reference](https://www.arduino.cc/reference/en/libraries/wifis3/)
- [RC PWM Standards](https://en.wikipedia.org/wiki/Pulse-width_modulation)
- [ESC Calibration Guide](https://oscarliang.com/esc-calibration/)

---

**Version**: 2.0  
**Last Updated**: 2025-12-05  
**Status**: Production Ready
