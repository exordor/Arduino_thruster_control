## RC Range Measurement & Calibration

To resolve asymmetric detent mapping (e.g., left stick reaches 1500→2000 in only 2 detents, right stick in ~3.1 detents), use `test_RC/test_RC.ino`:

- Purpose: Interrupt-based PWM capture on pins `2` (Right) and `3` (Left). Prints per-channel `raw` (last/min/max) and per-channel calibrated `norm` (normalized to 1000–2000).
- How to use:
  - Upload and open Serial at `115200`. Move sticks across detents and extremes; observe once-per-second prints.
  - Record `min/mid/max` (mid = around center detent).
  - Example calibration: Right `min=1435, mid=1493, max=1994`; Left `min=1498, mid=1529, max=2006`.
  - Normalization: piecewise around mid — `min→mid` maps to `1000→1500`, `mid→max` maps to `1500→2000` to keep center stable and stretch both sides.

### When to adjust the transmitter (RC controller)

If a channel hits `2000 µs` within few detents, Arduino cannot increase its physical resolution. Adjust on the transmitter:

- Endpoints/ATV/Travel: increase the faster-saturating channel or unify both sides, so equal detent span maps to similar PWM.
- Subtrim/Center: ensure both channels center at ~`1500 µs`.
- Dual Rates/Expo: disable or unify during testing to avoid nonlinear detent effects.
- Output range (if supported): unify to `1000–2000 µs`.
- Run the transmitter’s stick/channel calibration.

After transmitter calibration, re-measure with `test_RC.ino` and update calibration constants; we will apply the same per-channel normalization in `test/test.ino` and `wifi_RC_control/wifi_RC_control.ino` to align gears with physical detents.
# Arduino Dual Thruster Control

[中文文档](./README_zh.md)

This is an Arduino project for controlling dual thrusters (Left/Right) with multiple control modes.

## Project Structure

- **original/** - Original RC remote control version
- **ros_control/** - ROS + RC mixed control version
- **wifi_control/** - WiFi control version
- **wifi_RC_control/** - WiFi + RC hybrid control with automatic priority switching
- **wifi_manager_RC_control/** - ⭐ **Advanced WiFi + RC control with multi-network management** (Recommended)

## Features Comparison

### Original
- Reads PWM signals from RX receiver
- Valid range: 950-2000 µs, mapped to ESC range 1100-1900 µs
- Center deadband control
- Timeout protection: auto-stop after 200ms without signal

### ROS Control
- Supports mixed RC and ROS control
- Receives ROS commands via USB Serial
- Command format: `C <left_us> <right_us>\n`
- Auto fallback to RC control on ROS command timeout
- Real-time status output: `S <mode> <outL> <outR>\n`

### WiFi Control
- TCP server on Arduino (192.168.50.100:8888)
- Receives commands from ROS via network
- Command format: `C <left_us> <right_us>\n`
- Status reporting: `S <left_us> <right_us>\n` at 10Hz
- WiFi-only, no RC fallback

### WiFi + RC Control
- Combines WiFi and RC with automatic priority switching
- **Priority**: WiFi > RC > Failsafe
- WiFi timeout: 500ms (auto-switch to RC)
- RC timeout: 200ms (auto-switch to neutral)
- Best of both worlds: wireless control with RC safety backup

### WiFi Manager + RC Control ⭐ (Latest)
**The most advanced version with multi-network support:**

- 🌐 **Multi-WiFi Configuration** — Save up to 5 WiFi networks with priorities
- 🔄 **Priority-Based Auto-Connect** — Tries networks by priority on boot
- 📱 **Web Configuration Portal** — Manage networks via phone/computer (192.168.4.1)
- 🔌 **Persistent TCP Connection** — Stable ROS communication, no reconnects
- 🔁 **Auto-Reconnect** — Background WiFi retry every 30 seconds
- 💾 **EEPROM Storage** — Configurations saved permanently
- 🎮 **Hybrid Control** — WiFi > RC > Failsafe with automatic switching
- 📊 **Real-Time Status** — Streaming status at 10Hz with detailed logging

**Perfect for:**
- Multi-location deployment (home, office, field)
- Production environments requiring reliability
- Easy setup without code changes
- Automatic network failover

See [wifi_manager_RC_control/README.md](./wifi_manager_RC_control/README.md) for detailed documentation.

## Quick Start

### For beginners or simple setups:
Use **wifi_RC_control** — Single WiFi network, quick setup

### For production or multi-location use:
Use **wifi_manager_RC_control** — Multiple networks, web configuration, automatic failover

### Basic RC control only:
Use **original** — No dependencies, pure RC control

## Hardware Connection

### Pin Definition
- **CH_LEFT_IN**: Pin 2 - Left channel PWM input (RC receiver)
- **CH_RIGHT_IN**: Pin 3 - Right channel PWM input (RC receiver)
- **ESC_LEFT_OUT**: Pin 10 - Left ESC signal output
- **ESC_RIGHT_OUT**: Pin 9 - Right ESC signal output

### Typical Wiring
```
RC Receiver:
  CH1 → Pin 2 (CH_LEFT_IN)
  CH2 → Pin 3 (CH_RIGHT_IN)
  GND → GND

ESC Controllers:
  Left ESC Signal → Pin 10
  Right ESC Signal → Pin 9
  GND → GND (common)

Power:
  ESC Power → 5-12V (depends on ESC spec)
  Arduino USB or external 5V
```

## Communication Protocol

### WiFi Command Format
```
Command: C <left_us> <right_us>\n
Example: C 1600 1400

Response: S <mode> <left_us> <right_us>\n
Example: S 1 1600 1400
  mode: 0=RC active, 1=WiFi active
```

### Python Example
```python
import socket

# Connect to Arduino
s = socket.socket()
s.connect(('192.168.50.100', 8888))

# Send command
s.sendall(b'C 1600 1400\n')

# Receive status (10Hz stream)
while True:
    data = s.recv(1024)
    print(data.decode())  # S 1 1600 1400

s.close()
```

## Usage

### Option 1: WiFi Manager (Recommended)
```bash
1. Open wifi_manager_RC_control/wifi_manager_RC_control.ino
2. Upload to Arduino UNO R4 WiFi
3. First boot: Connects to AP "ArduinoR4-Config"
4. Visit http://192.168.4.1 to add WiFi networks
5. System auto-connects by priority
```

### Option 2: WiFi RC Control (Simple)
```bash
1. Open wifi_RC_control/wifi_RC_control.ino
2. Edit WiFi credentials in code:
   const char* ssid = "Your_SSID";
   const char* password = "Your_Password";
3. Upload to Arduino
4. Connect and control via TCP port 8888
```

### Option 3: RC Control Only
```bash
1. Open original/original.ino
2. Upload to Arduino
3. Connect RC receiver to pins 2, 3
4. Manual control via RC transmitter
```

## Parameter Configuration

- **RX Valid Range**: 950-2000 µs
- **ESC Output Range**: 1100-1900 µs
- **Center Position**: 1500 µs
- **RC Timeout**: 200 ms (failsafe to neutral)
- **WiFi Command Timeout**: 500 ms (fallback to RC)
- **Status Update Rate**: 10 Hz (every 100ms)

## Serial Commands (WiFi Manager version)

Open Serial Monitor at 115200 baud:

| Command | Function |
|---------|----------|
| `CONFIG` | Start configuration AP |
| `LIST` | Display all saved WiFi profiles |
| `STATUS` | Show WiFi connection status |
| `SWITCH <i>` | Switch to profile index i |
| `DELETE <i>` | Delete profile index i |
| `RESET` | Clear all profiles and restart |

## Safety Features

- ✅ **Auto-stop on signal timeout** — RC 200ms, WiFi 500ms
- ✅ **Input range validation** — Constrains to safe ESC range
- ✅ **Center deadband protection** — Prevents drift
- ✅ **Priority failsafe** — WiFi → RC → Neutral (1500µs)
- ✅ **Persistent configuration** — EEPROM storage survives power loss
- ✅ **Connection monitoring** — Real-time status logging

## Development Environment

- **Board**: Arduino UNO R4 WiFi (Renesas RA4M1)
- **IDE**: Arduino IDE 2.x or later
- **Required Libraries**: 
  - `Servo.h` (built-in)
  - `WiFiS3.h` (built-in for R4)
  - `EEPROM.h` (built-in)

## Tested Configurations

✅ **Hardware**:
- Arduino UNO R4 WiFi
- Standard RC receiver (PWM output)
- Standard ESCs (1100-1900µs range)

✅ **Software**:
- ROS Noetic + Python 3
- TCP socket communication
- Multiple WiFi networks (2.4GHz)

✅ **Performance**:
- TCP connection: Stable, no drops
- Status update: 10Hz consistent
- WiFi switching: < 30 seconds
- Command latency: < 50ms

## Troubleshooting

### WiFi won't connect
- Ensure 2.4GHz network (R4 doesn't support 5GHz)
- Check SSID and password are correct
- Move closer to router for initial setup

### TCP connection drops
- Verify using latest `wifi_manager_RC_control` version
- Check serial output for connection logs
- Ensure client sends commands with `\n` terminator

### Thrusters not responding
- Check RC receiver wiring (pins 2, 3)
- Verify ESC calibration (neutral = 1500µs)
- Confirm ESC power supply is adequate

### Configuration not saving
- Wait for serial confirmation after adding profile
- Don't power off during EEPROM write
- Use `RESET` command to clear corrupted data

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Test thoroughly on actual hardware
4. Submit pull request with description

## Version History

- **v2.0** (2025-12-05) - WiFi Manager with multi-network support
- **v1.3** - WiFi + RC hybrid control
- **v1.2** - WiFi control via TCP
- **v1.1** - ROS + RC mixed control
- **v1.0** - Original RC control

## License

MIT License - See LICENSE file for details

## Contact

- **Repository**: https://github.com/exordor/Arduino_thruster_control
- **Issues**: Report bugs via GitHub Issues
- **Documentation**: See individual project README files
