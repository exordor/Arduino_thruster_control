# Flow Meter Sensor (Arduino UNO R4)

An Arduino sketch for reading flow rate data from a pulse-based flow meter sensor using polling on digital pin D7.

## Features

- Real-time flow rate measurement in L/min
- Flow velocity calculation based on pipe diameter
- Total volume accumulation in liters
- CSV output for easy data logging and analysis

## Hardware

- **Board**: Arduino UNO R4
- **Sensor**: Pulse-output flow meter
- **Pin**: Digital pin 7 (with internal pullup)

## Specifications

| Parameter | Value |
|-----------|-------|
| Pipe diameter | 26 mm |
| Calibration factor (K) | 5 Hz per L/min |
| Pulses per liter | 300 |
| Update rate | 1 second |

## Wiring

```
Flow Meter Signal  →  D7
VCC                →  5V
GND                →  GND
```

## Serial Output

The sketch outputs CSV data at 115200 baud:

```
dt(ms), changes, f(Hz), Q(L/min), v(m/s), total(L)
```

| Column | Description |
|--------|-------------|
| dt(ms) | Time elapsed since last reading |
| changes | Number of pin state changes detected |
| f(Hz) | Calculated pulse frequency |
| Q(L/min) | Flow rate in liters per minute |
| v(m/s) | Flow velocity in meters per second |
| total(L) | Accumulated volume in liters |

## Calibration

The sketch uses the following calibration constants (modify in code if needed):

```cpp
const float K_HZ_PER_LMIN = 5.0f;       // Frequency per flow rate
const float PULSES_PER_L  = 300.0f;     // Pulses per volume
const float DIAMETER_M   = 0.026f;      // Pipe diameter in meters
```

## Limitations

- **Polling method**: May miss pulses at high flow rates. For accurate measurement at high frequencies, consider using an interrupt-based approach.
- **Update interval**: Fixed at 1 second; modify `REPORT_INTERVAL_MS` for different timing.

## License

MIT License
