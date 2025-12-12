# Arduino Thruster Control Test Program

English Version | **[中文版本](README.md)**

## Overview

This is an Arduino program for testing and controlling dual thrusters (left/right). The program reads PWM signals from an RC receiver, processes them through filtering and safety protection, then drives two ESCs (Electronic Speed Controllers) to control the thrusters.

## Hardware Connections

### Input Pins (RC Receiver)
- **Pin 2**: Right channel PWM input (CH_RIGHT_IN)
- **Pin 3**: Left channel PWM input (CH_LEFT_IN)

### Output Pins (ESC Control)
- **Pin 9**: Right thruster ESC output (ESC_RIGHT_OUT)
- **Pin 10**: Left thruster ESC output (ESC_LEFT_OUT)

### Wiring Requirements
1. **Common Ground (Required)**: Arduino GND must be connected to RC receiver GND
2. **Independent Power**: ESCs use independent power supply (battery), do not power from Arduino 5V
3. **Signal Protocol**: Ensure RC receiver outputs standard PWM signal (1000-2000µs), not S.BUS/PPM/iBUS protocols

```
RC Receiver        Arduino         ESC          Thruster
┌─────────┐       ┌────────┐      ┌───┐        ┌────┐
│ CH1 ────┼──────→│ Pin 2  │      │   │        │    │
│ CH2 ────┼──────→│ Pin 3  │      │ R │←───────┤ R  │
│         │       │        │      │   │        │    │
│ GND ────┼───────┤ GND    │      │ L │←───────┤ L  │
└─────────┘       │ Pin 9  ├─────→│   │        │    │
                  │ Pin 10 ├─────→│   │        │    │
                  └────────┘      └───┘        └────┘
                                   ↑
                              Battery Power
```

## Core Features

### 1. Interrupt-based PWM Capture (Non-blocking)

**Traditional Problem**: Using `pulseIn()` is blocking and may miss pulses or cause discontinuous sampling while waiting.

**Solution**: Use interrupt capture (`attachInterrupt`):
- Triggers on every pin level change
- Records timestamp on rising edge, calculates pulse width on falling edge
- Non-blocking, doesn't affect other tasks in main loop
- Only accepts pulse widths in 800-2200µs range, rejects noise

```cpp
void onRightChange() {
  int level = digitalRead(CH_RIGHT_IN);
  unsigned long now = micros();
  if (level == HIGH) {
    rRiseMicros = now;  // Record rising edge
  } else {
    unsigned long width = now - rRiseMicros;  // Calculate pulse width
    if (width >= 800 && width <= 2200) {
      rPulseMicros = width;  // Save valid pulse width
    }
  }
}
```

### 2. Signal Mapping and Deadband

**Input Range**: 950-2000µs (RC receiver standard output)
**Output Range**: 1100-1900µs (ESC safe operating range)

**Mapping Rules**:
- 950µs → 1100µs (minimum thrust)
- 1500µs → 1500µs (stop)
- 2000µs → 1900µs (maximum thrust)
- < 950µs or > 2000µs → 1500µs (failsafe)

**Center Deadband**:
- All inputs within 1500±25µs map to 1500µs
- Eliminates small jitter near transmitter center position
- Ensures thrusters completely stop at center

```cpp
if (abs((int)inUs - RC_MID) <= DEADBAND_US) return ESC_MID;
```

### 3. Low-pass Filter

**Purpose**: Smooth high-frequency jitter in input signal to prevent ESC from detecting signal instability.

**Implementation**: Exponential Weighted Moving Average (EWMA)
```cpp
filtL = (avgL * 80 + outL * 20) / 100;  // 20% new value, 80% old value
```

**Parameter**: `FILTER_ALPHA = 20` (percentage)
- Higher: Faster response, but weaker filtering
- Lower: Smoother, but slower response
- Recommended range: 15-30

### 4. Soft-start Ramp Limiting

**Purpose**: Prevent sudden output changes that could cause ESC to re-arm or enter protection mode.

**Implementation**: Limit maximum change per loop cycle
```cpp
int deltaL = filtL - avgL;
if (deltaL > MAX_STEP_US) deltaL = MAX_STEP_US;    // Limit acceleration
if (deltaL < -MAX_STEP_US) deltaL = -MAX_STEP_US;  // Limit deceleration
avgL += deltaL;
```

**Parameter**: `MAX_STEP_US = 10` (microseconds/cycle)
- At 5ms loop period, going from 1100→1900 takes 80 cycles (0.4 seconds)
- Higher value: Faster response, but may cause jitter
- Lower value: Smoother, but slower response
- Recommended range: 5-20

### 5. Failsafe Protection

**Trigger Condition**: No valid PWM signal received for more than 200ms

**Protection Action**: 
- Force output to 1500µs (thrusters stop)
- Continue monitoring, automatically resume control when signal returns

**Application Scenarios**:
- Transmitter turned off or lost connection
- Receiver power loss
- Loose signal cable
- Signal loss due to electromagnetic interference

```cpp
if (now - lastUpdateL > FAILSAFE_MS) avgL = ESC_MID;
```

## Parameter Tuning

### Response Speed vs Stability Trade-off

| Requirement | FILTER_ALPHA | MAX_STEP_US | Effect |
|-------------|--------------|-------------|--------|
| **Fast Response** | 30-40 | 15-20 | Responsive, suitable for racing |
| **Balanced** | 20 | 10 | Default config, balances stability & response |
| **Maximum Stability** | 10-15 | 5-8 | Smoothest, suitable for precision control |

### Deadband Adjustment

| DEADBAND_US | Use Case |
|-------------|----------|
| 15-20 | High-precision transmitter, accurate center |
| **25** | Default, suitable for most RC systems |
| 30-40 | Transmitter with significant center jitter |

## Serial Monitor Output

### Output Format
```
[Loop#] Right(Pin2): XXXµs (status) | Left(Pin3): XXXµs (status) | Last update: L=XXms R=XXms | outL|outR=XXXX|XXXX
```

### Status Descriptions
- **(Timeout)**: No pulse detected within 25ms
- **(Too Low)**: Pulse width < 950µs, abnormal signal
- **(Too High)**: Pulse width > 2000µs, abnormal signal
- **(Center)**: Pulse width within 1500±25µs range
- **(Reverse)**: Pulse width < 1475µs
- **(Forward)**: Pulse width > 1525µs

### Example Output
```
[1234] Right(Pin2): 1523µs (Center) | Left(Pin3): 1498µs (Center) | Last update: L=0ms R=0ms | outL|outR=1500|1500
[1235] Right(Pin2): 1678µs (Forward) | Left(Pin3): 1682µs (Forward) | Last update: L=0ms R=0ms | outL|outR=1510|1510
[1236] Right(Pin2): 1756µs (Forward) | Left(Pin3): 1748µs (Forward) | Last update: L=0ms R=0ms | outL|outR=1520|1520
```

## Troubleshooting

### 1. Thrusters Rotate Intermittently

**Possible Causes**:
- RC receiver and Arduino don't share common ground → **Connect GND**
- Receiver output protocol is not PWM → **Check receiver settings**
- Unstable power supply → **Use independent regulated power**

**Diagnosis Method**:
```
// Check serial output, if you frequently see:
Last update: L=0ms R=0ms  // Signal normal
Last update: L=234ms R=234ms [Left failsafe] [Right failsafe]  // Signal lost
```

### 2. No PWM Input (Always Timeout)

**Checklist**:
1. ✓ Is receiver powered on (LED indicator lit)
2. ✓ Is transmitter turned on and paired
3. ✓ Are signal wires correctly connected to Pin 2 and Pin 3
4. ✓ Is receiver configured for PWM output mode (not S.BUS/PPM)
5. ✓ Are Arduino GND and receiver GND connected

### 3. Abnormally Low PWM Values (<100µs)

**Cause**: Detecting high-frequency noise instead of real PWM signal

**Solutions**:
- Ensure common ground connection
- Use shielded cable or shorten signal cable length
- Check if receiver outputs correct PWM protocol
- Add decoupling capacitors (between receiver VCC/GND)

### 4. Thrusters Respond Too Slowly

**Adjust Parameters**:
```cpp
const int FILTER_ALPHA = 30;    // Increase to 30-40
const int MAX_STEP_US  = 20;    // Increase to 15-20
```

### 5. Severe Thruster Jitter

**Adjust Parameters**:
```cpp
const int FILTER_ALPHA = 10;    // Decrease to 10-15
const int MAX_STEP_US  = 5;     // Decrease to 5-8
const int DEADBAND_US  = 35;    // Increase to 30-40
```

## ESC Initialization

The program performs ESC initialization on startup:
```cpp
escL.writeMicroseconds(ESC_MID);  // Output 1500µs
escR.writeMicroseconds(ESC_MID);
delay(2000);  // Hold for 2 seconds
```

**Notes**:
- Ensure ESC is in armed state when receiving 1500µs signal
- Some ESCs require special arming procedure (high-low-center), refer to ESC manual
- Modify initialization code in `setup()` if custom arming needed

## Technical Specifications

| Parameter | Value | Description |
|-----------|-------|-------------|
| Serial Baud Rate | 115200 | For debug output |
| Loop Period | 5ms | 200Hz update frequency |
| PWM Frequency | 50Hz | Standard RC PWM |
| Interrupt Mode | CHANGE | Triggers on both edges |
| Filter Time Constant | ~25ms | Depends on FILTER_ALPHA |
| Maximum Ramp Time | ~400ms | From min to max (1100→1900µs) |

## Safety Precautions

⚠️ **Warning**: Thrusters are dangerous, please follow these safety rules:

1. **Initial Testing**: Remove propellers, test motor rotation only
2. **Secure Fixture**: Ensure device is firmly fixed to prevent unexpected movement
3. **Power-off Operations**: Disconnect power when wiring
4. **Emergency Stop**: Ensure ability to cut power or transmitter at any time
5. **Clear Surroundings**: Keep test area clear of people and obstacles
6. **Underwater Testing**: Test in water tank first if for underwater thrusters

## Common RC Receiver Protocol Comparison

| Protocol | Pin Count | Frequency | Compatible with This Program |
|----------|-----------|-----------|------------------------------|
| **PWM** | 1 per channel | 50Hz | ✅ Compatible |
| PPM | 1 (multi-channel) | 50Hz | ❌ Not compatible |
| S.BUS | 1 serial | 100Hz | ❌ Not compatible |
| iBUS | 1 serial | 50Hz | ❌ Not compatible |
| DSM2/DSMX | 1 serial | 22ms | ❌ Not compatible |

If receiver doesn't support PWM output, code must be modified to support the respective protocol.

## Extension Suggestions

1. **Add Third Channel**: For mode switching or speed limiting
2. **Speed Limiting**: Limit maximum thrust percentage
3. **Mixing Mode**: Implement differential steering control
4. **Data Logging**: Save operational data to SD card
5. **Wireless Telemetry**: Send status information via Bluetooth/WiFi

## License

This project is developed based on Arduino thruster control system, for learning and testing purposes only.

## Version History

- **v1.0** (2025-12-12): Initial release
  - Interrupt-based PWM capture
  - Low-pass filter and soft-start
  - Center deadband and failsafe
  - Detailed serial debug output

---

**Author**: Arduino Thruster Control Project Team  
**Last Updated**: December 12, 2025
