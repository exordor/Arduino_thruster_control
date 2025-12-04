# Arduino Dual Thruster Control

[中文文档](./README_zh.md)

This is an Arduino project for controlling dual thrusters (Left/Right) with multiple control modes.

## Project Structure

- **original/** - Original RC remote control version
- **ros_control/** - ROS + RC mixed control version
- **wifi_control/** - WiFi control version (in development)

## Features

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
- In development...

## Hardware Connection

### Pin Definition
- **CH_LEFT_IN**: Pin 2 - Left channel PWM input
- **CH_RIGHT_IN**: Pin 3 - Right channel PWM input
- **ESC_LEFT_OUT**: Pin 9/10 - Left ESC signal output
- **ESC_RIGHT_OUT**: Pin 10/9 - Right ESC signal output

## Usage

1. Open the corresponding `.ino` file with Arduino IDE
2. Select the correct board model
3. Connect Arduino to computer
4. Upload the program

## Parameter Configuration

- **RX Valid Range**: 950-2000 µs
- **ESC Output Range**: 1100-1900 µs
- **Center Position**: 1500 µs
- **Deadband**: 50 µs
- **Timeout**: 200 ms

## Safety Features

- Auto-stop on input signal timeout
- Input range validation
- Center deadband protection

## Development Environment

- Arduino UNO WiFi R4
- Arduino IDE
- Required library: Servo.h

## License

[Add license information]
