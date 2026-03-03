#!/usr/bin/env python3
"""
持续发送 UDP 命令到 Arduino 推进器控制
"""

import socket
import time
import argparse

# Arduino 配置
ARDUINO_IP = "192.168.50.100"
ARDUINO_PORT = 8888
SEND_INTERVAL = 0.1  # 发送间隔（秒），10Hz

# 推进器值
ESC_MIN = 1100
ESC_MID = 1500
ESC_MAX = 1900

def send_command(sock, ip, port, left, right):
    """发送单个 UDP 命令"""
    command = f"C {left} {right}\n"
    sock.sendto(command.encode(), (ip, port))
    return command

def main():
    parser = argparse.ArgumentParser(description="持续发送 UDP 命令到 Arduino 推进器")
    parser.add_argument("--speed", "-s", type=str, default="mid",
                        choices=["low", "mid", "high", "max", "stop"],
                        help="速度档位: low=1600, mid=1700, high=1800, max=1900, stop=1500")
    parser.add_argument("--ip", "-i", type=str, default=ARDUINO_IP,
                        help=f"Arduino IP 地址 (默认: {ARDUINO_IP})")
    parser.add_argument("--port", "-p", type=int, default=ARDUINO_PORT,
                        help=f"Arduino 端口 (默认: {ARDUINO_PORT})")
    parser.add_argument("--interval", "-t", type=float, default=SEND_INTERVAL,
                        help=f"发送间隔秒数 (默认: {SEND_INTERVAL})")
    parser.add_argument("--left", "-l", type=int, help="左推进器值 (1100-1900, 覆盖 --speed)")
    parser.add_argument("--right", "-r", type=int, help="右推进器值 (1100-1900, 覆盖 --speed)")
    parser.add_argument("--count", "-c", type=int, help="发送次数后停止 (默认: 无限循环)")
    parser.add_argument("--once", "-o", action="store_true", help="只发送一次命令")

    args = parser.parse_args()

    # 确定推进器值
    speed_map = {
        "stop": (ESC_MID, ESC_MID),
        "low": (1600, 1600),
        "mid": (1700, 1700),
        "high": (1800, 1800),
        "max": (ESC_MAX, ESC_MAX),
    }

    if args.left is not None or args.right is not None:
        left = args.left if args.left is not None else ESC_MID
        right = args.right if args.right is not None else ESC_MID
        left = max(ESC_MIN, min(ESC_MAX, left))
        right = max(ESC_MIN, min(ESC_MAX, right))
    else:
        left, right = speed_map[args.speed]

    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    print(f"\n{'='*50}")
    print(f"目标: {args.ip}:{args.port}")
    print(f"命令: C {left} {right}")
    if args.speed == "stop":
        print("模式: 停止")
    else:
        print(f"模式: 前进 ({args.speed})")
    print(f"间隔: {args.interval}s")
    print(f"{'='*50}\n")

    if args.once:
        # 只发送一次
        cmd = send_command(sock, args.ip, args.port, left, right)
        print(f"已发送: {cmd.strip()}")
        sock.close()
        return

    # 持续发送
    count = 0
    try:
        while True:
            if args.count and count >= args.count:
                print(f"\n已发送 {count} 次命令，停止。")
                break

            cmd = send_command(sock, args.ip, args.port, left, right)
            count += 1
            print(f"\r[{count}] 发送: {cmd.strip()}", end="", flush=True)

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print(f"\n\n发送停止，共发送 {count} 次")

    # 发送停止命令
    print("\n发送停止命令...")
    for _ in range(3):
        send_command(sock, args.ip, args.port, ESC_MID, ESC_MID)
        time.sleep(0.05)
    print("已停止")

    sock.close()

if __name__ == "__main__":
    main()
