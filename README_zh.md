# Arduino 双推进器控制

[English](./README.md)

这是一个用于控制双推进器（左/右）的 Arduino 项目，支持多种控制模式。

## 项目结构

- **original/** - 原始 RC 遥控器控制版本
- **ros_control/** - ROS + RC 混合控制版本
- **wifi_control/** - WiFi 控制版本（开发中）

## 功能特性

### Original（原始版本）
- 读取 RX 遥控器的 PWM 信号
- 有效范围：950-2000 µs，映射到 ESC 范围 1100-1900 µs
- 中心位置死区控制
- 超时保护：200ms 无信号自动停止

### ROS Control（ROS 控制版本）
- 支持 RC 遥控器和 ROS 混合控制
- 通过 USB Serial 接收 ROS 命令
- 命令格式：`C <left_us> <right_us>\n`
- ROS 命令超时后自动切换回 RC 控制
- 实时状态输出：`S <mode> <outL> <outR>\n`

### WiFi Control（WiFi 控制版本）
- 开发中...

## 硬件连接

### 引脚定义
- **CH_LEFT_IN**: Pin 2 - 左通道 PWM 输入
- **CH_RIGHT_IN**: Pin 3 - 右通道 PWM 输入
- **ESC_LEFT_OUT**: Pin 9/10 - 左 ESC 信号输出
- **ESC_RIGHT_OUT**: Pin 10/9 - 右 ESC 信号输出

## 使用说明

1. 使用 Arduino IDE 打开对应的 `.ino` 文件
2. 选择正确的开发板型号
3. 连接 Arduino 到电脑
4. 上传程序

## 参数配置

- **RX 有效范围**: 950-2000 µs
- **ESC 输出范围**: 1100-1900 µs
- **中心位置**: 1500 µs
- **死区**: 50 µs
- **超时时间**: 200 ms

## 安全特性

- 输入信号超时自动停止
- 输入范围检查
- 中心死区保护

## 开发环境

- Arduino UNO WiFi R4
- Arduino IDE
- 所需库：Servo.h

## 许可

[添加许可信息]
