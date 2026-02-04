#!/usr/bin/env python3
"""
UDP Thruster Control Test Script

Tests the heartbeat mechanism and data reception from the Arduino thruster controller.

Arduino Protocol (Dual Port):
- Data Port: 8888 (commands C, status S, flow F, PING/PONG)
- Heartbeat Port: 8889 (HEARTBEAT messages only)

Data Port (8888):
- Command: "C <left_us> <right_us>\n" to send thruster commands
- Status: "S <mode> <left_us> <right_us>\n" every 100ms
- Flow: "F <freq_hz> <flow_lmin> <velocity_ms> <total_liters>\n" every 1s
- Ping: "PING" -> "PONG" (handshake/keep-alive, no rate limit)

Heartbeat Port (8889):
- Heartbeat: "HEARTBEAT\n" every 500ms (from Arduino)

Mode: 0=RC, 1=WiFi
Timeout: 2s without data = Arduino switches to RC mode
"""

import socket
import threading
import time
import argparse
from dataclasses import dataclass
from datetime import datetime
from typing import Optional


@dataclass
class StatusData:
    """Container for status data from Arduino"""
    mode: int = 0  # 0=RC, 1=WiFi
    left_us: int = 1500
    right_us: int = 1500
    timestamp: float = 0.0

    @property
    def mode_str(self) -> str:
        return "WiFi" if self.mode == 1 else "RC"


@dataclass
class FlowData:
    """Container for flow meter data from Arduino"""
    freq_hz: float = 0.0
    flow_lmin: float = 0.0
    velocity_ms: float = 0.0
    total_liters: float = 0.0
    timestamp: float = 0.0


class UDPTestClient:
    """UDP client for testing Arduino thruster communication with dual ports"""

    def __init__(self, arduino_ip: str = "192.168.50.100", data_port: int = 8888, heartbeat_port: int = 8889):
        self.arduino_ip = arduino_ip
        self.data_port = data_port
        self.heartbeat_port = heartbeat_port
        self.data_sock: Optional[socket.socket] = None
        self.heartbeat_sock: Optional[socket.socket] = None
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        self.keep_alive_thread: Optional[threading.Thread] = None

        # Statistics
        self.heartbeat_count = 0
        self.status_count = 0
        self.flow_count = 0
        self.last_heartbeat_time = 0.0
        self.last_status_time = 0.0
        self.last_receive_time = 0.0

        # Latest data
        self.latest_status = StatusData()
        self.latest_flow = FlowData()

        # Heartbeat timeout detection (Arduino sends every 500ms)
        self.heartbeat_timeout = 1.0  # Consider offline if no heartbeat for 1s

    def connect(self) -> bool:
        """Create and bind two UDP sockets (data and heartbeat)"""
        try:
            # Data socket (commands, status, flow, PING/PONG)
            self.data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.data_sock.bind(('', 0))
            self.data_sock.settimeout(0.1)  # Non-blocking with timeout
            data_local_port = self.data_sock.getsockname()[1]

            # Heartbeat socket (HEARTBEAT messages only)
            self.heartbeat_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.heartbeat_sock.bind(('', self.heartbeat_port))
            self.heartbeat_sock.settimeout(0.1)  # Non-blocking with timeout

            print(f"[INFO] Data socket: local port {data_local_port} -> {self.arduino_ip}:{self.data_port}")
            print(f"[INFO] Heartbeat socket: local port {self.heartbeat_port} <- {self.arduino_ip}:{self.heartbeat_port}")
            return True
        except socket.error as e:
            print(f"[ERROR] Failed to create sockets: {e}")
            return False

    def disconnect(self):
        """Stop receiving and close both sockets"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        if self.keep_alive_thread:
            self.keep_alive_thread.join(timeout=1.0)
        if self.data_sock:
            self.data_sock.close()
            self.data_sock = None
        if self.heartbeat_sock:
            self.heartbeat_sock.close()
            self.heartbeat_sock = None

    def send_command(self, left_us: int, right_us: int) -> bool:
        """Send thruster command to Arduino (on data port)

        Args:
            left_us: Left thruster pulse width in microseconds (1100-1900)
            right_us: Right thruster pulse width in microseconds (1100-1900)

        Returns:
            True if command was sent successfully
        """
        if not self.data_sock:
            print("[ERROR] Data socket not connected")
            return False

        command = f"C {left_us} {right_us}\n"
        try:
            self.data_sock.sendto(command.encode(), (self.arduino_ip, self.data_port))
            return True
        except socket.error as e:
            print(f"[ERROR] Failed to send command: {e}")
            return False

    def send_handshake(self) -> bool:
        """Send a PING handshake command to trigger Arduino response

        Arduino requires receiving at least one UDP packet before it will
        start sending heartbeats back (to learn the client's address).

        Returns:
            True if handshake was sent successfully
        """
        return self.send_ping()

    def send_ping(self) -> bool:
        """Send PING for handshake/keep-alive (on data port)

        Returns:
            True if PING was sent successfully
        """
        if not self.data_sock:
            print("[ERROR] Data socket not connected")
            return False

        try:
            self.data_sock.sendto(b"PING\n", (self.arduino_ip, self.data_port))
            return True
        except socket.error as e:
            print(f"[ERROR] Failed to send PING: {e}")
            return False

    def _parse_status(self, data: str) -> Optional[StatusData]:
        """Parse status message: S <mode> <left_us> <right_us>"""
        parts = data.split()
        if len(parts) == 4 and parts[0] == 'S':
            try:
                return StatusData(
                    mode=int(parts[1]),
                    left_us=int(parts[2]),
                    right_us=int(parts[3]),
                    timestamp=time.time()
                )
            except ValueError:
                pass
        return None

    def _parse_flow(self, data: str) -> Optional[FlowData]:
        """Parse flow data message: F <freq_hz> <flow_lmin> <velocity_ms> <total_liters>"""
        parts = data.split()
        if len(parts) == 5 and parts[0] == 'F':
            try:
                return FlowData(
                    freq_hz=float(parts[1]),
                    flow_lmin=float(parts[2]),
                    velocity_ms=float(parts[3]),
                    total_liters=float(parts[4]),
                    timestamp=time.time()
                )
            except ValueError:
                pass
        return None

    def _receive_loop(self):
        """Background thread for receiving UDP messages from both ports"""
        import select

        while self.running:
            try:
                # Use select to wait for data on either socket
                sockets = [s for s in (self.data_sock, self.heartbeat_sock) if s]
                readable, _, _ = select.select(sockets, [], [], 0.1)

                for sock in readable:
                    data, addr = sock.recvfrom(256)
                    self.last_receive_time = time.time()
                    message = data.decode('utf-8', errors='ignore').strip()

                    # Determine which socket received the data
                    is_heartbeat = (sock == self.heartbeat_sock)

                    # Handle different message types
                    if message == "HEARTBEAT":
                        self.heartbeat_count += 1
                        self.last_heartbeat_time = time.time()
                        port_str = f":{addr[1]}" if is_heartbeat else ""
                        print(f"[HEARTBEAT] #{self.heartbeat_count} from {addr[0]}{port_str}")

                    elif message == "PONG":
                        print(f"[PONG] Response from {addr[0]}:{addr[1]}")

                    elif message.startswith('S '):
                        status = self._parse_status(message)
                        if status:
                            self.status_count += 1
                            self.last_status_time = time.time()
                            self.latest_status = status
                            print(f"[STATUS] Mode={status.mode_str} L={status.left_us} R={status.right_us}us")

                    elif message.startswith('F '):
                        flow = self._parse_flow(message)
                        if flow:
                            self.flow_count += 1
                            self.latest_flow = flow
                            print(f"[FLOW] {flow.freq_hz:.2f}Hz, {flow.flow_lmin:.2f}L/min, "
                                  f"{flow.velocity_ms:.4f}m/s, Total={flow.total_liters:.3f}L")

                    else:
                        print(f"[UNKNOWN] {message} from {addr[0]}:{addr[1]}")

            except socket.timeout:
                continue
            except (socket.error, ValueError) as e:
                if self.running:
                    print(f"[ERROR] Receive error: {e}")
                break

    def start_receiving(self):
        """Start background receive thread"""
        if not self.data_sock or not self.heartbeat_sock:
            print("[ERROR] Sockets not connected")
            return

        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        print("[INFO] Receive thread started (listening on both ports)")

    def _keep_alive_loop(self, interval: float = 1.0):
        """Background thread for sending keep-alive PING messages

        Args:
            interval: Seconds between keep-alive pings (default: 1.0)
        """
        while self.running:
            time.sleep(interval)
            if self.running:
                self.send_ping()

    def start_keep_alive(self, interval: float = 1.0):
        """Start background keep-alive thread

        Args:
            interval: Seconds between keep-alive pings (default: 1.0)
                      Arduino timeout is 2s, so 1s is safe
        """
        if not self.data_sock:
            print("[ERROR] Data socket not connected")
            return

        self.keep_alive_thread = threading.Thread(
            target=self._keep_alive_loop,
            args=(interval,),
            daemon=True
        )
        self.keep_alive_thread.start()
        print(f"[INFO] Keep-alive thread started (interval: {interval}s)")

    def is_arduino_online(self) -> bool:
        """Check if Arduino is online based on heartbeat timeout"""
        if self.last_heartbeat_time == 0.0:
            return False
        return (time.time() - self.last_heartbeat_time) < self.heartbeat_timeout

    def get_time_since_last_heartbeat(self) -> float:
        """Get time in seconds since last heartbeat"""
        if self.last_heartbeat_time == 0.0:
            return float('inf')
        return time.time() - self.last_heartbeat_time

    def print_statistics(self):
        """Print communication statistics"""
        uptime = time.time() - self.last_receive_time if self.last_receive_time > 0 else 0

        print("\n=== Communication Statistics ===")
        print(f"  Heartbeats received: {self.heartbeat_count}")
        print(f"  Status messages: {self.status_count}")
        print(f"  Flow data messages: {self.flow_count}")
        print(f"  Time since last heartbeat: {self.get_time_since_last_heartbeat():.2f}s")
        print(f"  Arduino online: {self.is_arduino_online()}")

        if self.latest_status.timestamp > 0:
            print(f"\n  Latest Status:")
            print(f"    Mode: {self.latest_status.mode_str}")
            print(f"    Left: {self.latest_status.left_us} us")
            print(f"    Right: {self.latest_status.right_us} us")

        if self.latest_flow.timestamp > 0:
            print(f"\n  Latest Flow Data:")
            print(f"    Frequency: {self.latest_flow.freq_hz} Hz")
            print(f"    Flow Rate: {self.latest_flow.flow_lmin} L/min")
            print(f"    Velocity: {self.latest_flow.velocity_ms} m/s")
            print(f"    Total Volume: {self.latest_flow.total_liters} L")


def interactive_mode(client: UDPTestClient):
    """Interactive command mode"""
    print("\n=== Interactive Command Mode ===")
    print("Commands:")
    print("  n / neutral   - Send neutral command (1500, 1500)")
    print("  f / forward   - Send forward command (1600, 1600)")
    print("  b / backward  - Send backward command (1400, 1400)")
    print("  l / left      - Turn left (1500, 1600)")
    print("  r / right     - Turn right (1600, 1500)")
    print("  ping          - Send PING (handshake/keep-alive)")
    print("  <L> <R>       - Custom command (e.g., '1550 1600')")
    print("  stats         - Show statistics")
    print("  status        - Show latest status")
    print("  flow          - Show latest flow data")
    print("  q / quit      - Exit")

    while True:
        try:
            cmd = input("\n> ").strip().lower()

            if not cmd:
                continue

            if cmd in ('q', 'quit'):
                break

            elif cmd in ('n', 'neutral'):
                client.send_command(1500, 1500)
                print("[SENT] Neutral (1500, 1500)")

            elif cmd in ('f', 'forward'):
                client.send_command(1600, 1600)
                print("[SENT] Forward (1600, 1600)")

            elif cmd in ('b', 'backward'):
                client.send_command(1400, 1400)
                print("[SENT] Backward (1400, 1400)")

            elif cmd in ('l', 'left'):
                client.send_command(1500, 1600)
                print("[SENT] Left (1500, 1600)")

            elif cmd in ('r', 'right'):
                client.send_command(1600, 1500)
                print("[SENT] Right (1600, 1500)")

            elif cmd == 'ping':
                client.send_ping()
                print("[SENT] PING")

            elif cmd == 'stats':
                client.print_statistics()

            elif cmd == 'status':
                s = client.latest_status
                if s.timestamp > 0:
                    print(f"[STATUS] Mode={s.mode_str} L={s.left_us} R={s.right_us}us")
                else:
                    print("[STATUS] No status data received yet")

            elif cmd == 'flow':
                f = client.latest_flow
                if f.timestamp > 0:
                    print(f"[FLOW] {f.freq_hz:.2f}Hz, {f.flow_lmin:.2f}L/min, "
                          f"{f.velocity_ms:.4f}m/s, Total={f.total_liters:.3f}L")
                else:
                    print("[FLOW] No flow data received yet")

            else:
                # Try to parse as custom command "L R"
                parts = cmd.split()
                if len(parts) == 2:
                    try:
                        l, r = int(parts[0]), int(parts[1])
                        if 1100 <= l <= 1900 and 1100 <= r <= 1900:
                            client.send_command(l, r)
                            print(f"[SENT] Custom ({l}, {r})")
                        else:
                            print("[ERROR] Values must be between 1100 and 1900")
                        continue
                    except ValueError:
                        pass
                print(f"[ERROR] Unknown command: {cmd}")

        except (EOFError, KeyboardInterrupt):
            print("\nExiting...")
            break


def heartbeat_test_mode(client: UDPTestClient, duration: int = 10):
    """Test heartbeat reception for a specified duration"""
    print(f"\n=== Heartbeat Test Mode ({duration}s) ===")
    print("Monitoring heartbeat messages from Arduino...")
    print(f"Expected: ~{duration * 2} heartbeats (every 500ms)")

    start_time = time.time()
    expected_interval = 0.5  # Arduino sends every 500ms
    last_heartbeat_interval = 0.0
    missed_heartbeats = 0
    previous_heartbeat_time = 0.0

    while time.time() - start_time < duration:
        time.sleep(0.1)

        # Check heartbeat intervals
        if client.last_heartbeat_time > previous_heartbeat_time:
            if previous_heartbeat_time > 0:
                interval = client.last_heartbeat_time - previous_heartbeat_time
                last_heartbeat_interval = interval
                # Allow some jitter (±100ms)
                if interval > expected_interval + 0.2:
                    missed_heartbeats += int(interval / expected_interval) - 1
            previous_heartbeat_time = client.last_heartbeat_time

        # Print status every second
        elapsed = time.time() - start_time
        if int(elapsed) > int(elapsed - 0.1):
            online = client.is_arduino_online()
            print(f"[{elapsed:.0f}s] Heartbeats: {client.heartbeat_count}, "
                  f"Online: {online}, "
                  f"Last interval: {last_heartbeat_interval:.2f}s")

    # Summary
    actual_duration = time.time() - start_time
    expected_count = int(actual_duration / expected_interval)
    received_percent = (client.heartbeat_count / expected_count * 100) if expected_count > 0 else 0

    print("\n=== Heartbeat Test Results ===")
    print(f"  Duration: {actual_duration:.2f}s")
    print(f"  Expected heartbeats: ~{expected_count}")
    print(f"  Received heartbeats: {client.heartbeat_count}")
    print(f"  Success rate: {received_percent:.1f}%")
    print(f"  Missed heartbeats: {missed_heartbeats}")

    if client.heartbeat_count > 0:
        avg_interval = actual_duration / client.heartbeat_count
        print(f"  Average interval: {avg_interval:.3f}s (expected: {expected_interval}s)")


def thruster_test_mode(client: UDPTestClient):
    """Test thruster control commands

    Sends various commands and verifies status responses.
    Tests mode switching (RC → WiFi) and thruster output.
    """
    print("\n=== Thruster Control Test ===")
    print("Sending test commands and verifying status responses...\n")

    # Test sequence: (left, right, expected_mode, description)
    test_commands = [
        (1500, 1500, 1, "Neutral - Should switch to WiFi mode"),
        (1600, 1600, 1, "Forward - Both thrusters forward"),
        (1400, 1400, 1, "Backward - Both thrusters backward"),
        (1500, 1600, 1, "Turn Left - Right forward"),
        (1600, 1500, 1, "Turn Right - Left forward"),
        (1700, 1700, 1, "High Forward - Both thrusters high"),
        (1300, 1300, 1, "High Backward - Both thrusters high reverse"),
        (1500, 1500, 1, "Back to Neutral"),
        (1100, 1900, 1, "Max differential - Left reverse, right forward"),
        (1500, 1500, 1, "Return to Neutral"),
    ]

    passed = 0
    failed = 0

    for i, (left, right, expected_mode, description) in enumerate(test_commands):
        print(f"[Test {i+1}/{len(test_commands)}] {description}")
        print(f"  Sending: C {left} {right}")

        # Clear latest status to verify new response
        client.latest_status = StatusData()

        # Send command
        if not client.send_command(left, right):
            print(f"  [FAIL] Could not send command")
            failed += 1
            continue

        # Wait for status response (with timeout)
        deadline = time.time() + 1.0
        while time.time() < deadline:
            time.sleep(0.05)
            if client.latest_status.timestamp > 0:
                break

        # Verify response
        if client.latest_status.timestamp == 0:
            print(f"  [WARN] No status response received")
        else:
            s = client.latest_status
            mode_match = (s.mode == expected_mode)
            left_match = (s.left_us == left)
            right_match = (s.right_us == right)

            if mode_match and left_match and right_match:
                print(f"  [PASS] Mode={s.mode_str} L={s.left_us} R={s.right_us}us ✓")
                passed += 1
            else:
                print(f"  [FAIL] Mode={s.mode_str} (expected: {'WiFi' if expected_mode==1 else 'RC'}) "
                      f"L={s.left_us} (expected: {left}) "
                      f"R={s.right_us} (expected: {right})")
                failed += 1

        # Delay between tests
        time.sleep(0.3)

    # Test rate limiting
    print("\n=== Rate Limiting Test ===")
    print("Sending 5 commands rapidly (should be rate limited to 100ms min)...")
    start = time.time()
    for i in range(5):
        client.send_command(1600, 1600)
        time.sleep(0.02)  # 20ms between commands
    elapsed = time.time() - start
    print(f"  Sent 5 commands in {elapsed:.3f}s (20ms intervals)")
    print(f"  Expected minimum time: {4 * 0.100:.1f}s (100ms rate limit)")
    if elapsed >= 0.4:
        print(f"  [INFO] Commands appear to be rate limited correctly")

    # Summary
    print(f"\n=== Test Summary ===")
    print(f"  Total tests: {len(test_commands)}")
    print(f"  Passed: {passed}")
    print(f"  Failed: {failed}")
    print(f"  Success rate: {(passed / len(test_commands) * 100):.1f}%")

    # Show final status
    if client.latest_status.timestamp > 0:
        s = client.latest_status
        print(f"\n  Final Status: Mode={s.mode_str} L={s.left_us} R={s.right_us}us")


def hz_10_test_mode(client: UDPTestClient, duration: int = 30):
    """Send commands at 10Hz and measure latency

    This test sends alternating commands at 10Hz (100ms intervals) to match
    the Arduino's rate limit and measure actual control latency.
    """
    print("\n=== 10Hz Latency Test ===")
    print(f"Sending commands at 10Hz for {duration}s...")
    print("Commands will alternate: 1500->1600->1500->1400->1500")

    import statistics

    # Test sequence
    commands = [
        (1500, 1500, "Neutral"),
        (1600, 1600, "Forward"),
        (1500, 1500, "Neutral"),
        (1400, 1400, "Backward"),
    ]
    cmd_index = 0

    start_time = time.time()
    sent_count = 0
    response_times = []
    last_command_time = 0
    last_expected_left = 1500
    last_expected_right = 1500

    print(f"\n{'Time':<8} {'Sent':<20} {'Status':<20} {'Latency':<10} {'Delta':<10}")
    print("-" * 75)

    while time.time() - start_time < duration:
        now = time.time()
        elapsed = now - start_time

        # Send command at 10Hz (every 100ms)
        if elapsed - last_command_time >= 0.1:
            left, right, desc = commands[cmd_index % len(commands)]
            client.send_command(left, right)
            sent_time = time.time()
            sent_count += 1
            last_expected_left = left
            last_expected_right = right
            last_command_time = elapsed

            # Wait for status response
            deadline = sent_time + 0.2  # 200ms timeout
            response_received = False
            latency = 0

            while time.time() < deadline:
                time.sleep(0.005)  # 5ms poll
                if client.latest_status.timestamp > sent_time:
                    latency = (client.latest_status.timestamp - sent_time) * 1000  # ms
                    response_times.append(latency)
                    response_received = True
                    break

            # Check if status matches expected
            if response_received:
                s = client.latest_status
                match = (s.left_us == left and s.right_us == right)
                match_str = "✓" if match else "✗"
                delta = abs(s.left_us - left) + abs(s.right_us - right)
                print(f"{elapsed:>6.1f}s  {desc:<20}  L={s.left_us} R={s.right_us:<4}  "
                      f"{latency:>6.0f}ms  {delta:>4}µs {match_str}")
            else:
                print(f"{elapsed:>6.1f}s  {desc:<20}  (no response)     ---        ---")

            cmd_index += 1

        time.sleep(0.01)  # 10ms sleep between checks

    # Summary statistics
    print("\n=== 10Hz Test Results ===")
    print(f"  Duration: {duration}s")
    print(f"  Commands sent: {sent_count}")
    print(f"  Expected rate: 10 Hz")
    print(f"  Actual rate: {sent_count / duration:.2f} Hz")

    if response_times:
        print(f"\n  Latency Statistics:")
        print(f"    Min: {min(response_times):.1f} ms")
        print(f"    Max: {max(response_times):.1f} ms")
        print(f"    Avg: {statistics.mean(response_times):.1f} ms")
        print(f"    Median: {statistics.median(response_times):.1f} ms")
        if len(response_times) > 1:
            print(f"    Std Dev: {statistics.stdev(response_times):.1f} ms")

    # Final status
    if client.latest_status.timestamp > 0:
        s = client.latest_status
        print(f"\n  Final Status: Mode={s.mode_str} L={s.left_us} R={s.right_us}us")


def main():
    parser = argparse.ArgumentParser(
        description="Test UDP communication with Arduino thruster controller"
    )
    parser.add_argument(
        '--ip',
        default='192.168.50.100',
        help='Arduino IP address (default: 192.168.50.100)'
    )
    parser.add_argument(
        '--data-port',
        type=int,
        default=8888,
        help='Data UDP port for commands, status, flow (default: 8888)'
    )
    parser.add_argument(
        '--heartbeat-port',
        type=int,
        default=8889,
        help='Heartbeat UDP port (default: 8889)'
    )
    parser.add_argument(
        '--mode',
        choices=['interactive', 'heartbeat', 'thruster', 'hz10', 'monitor'],
        default='monitor',
        help='Test mode: interactive (send commands), heartbeat (test only), thruster (control test), hz10 (10Hz latency test), monitor (listen only)'
    )
    parser.add_argument(
        '--duration',
        type=int,
        default=10,
        help='Duration for heartbeat test in seconds (default: 10)'
    )
    parser.add_argument(
        '--initial-command',
        nargs=2,
        type=int,
        metavar=('LEFT', 'RIGHT'),
        help='Send initial command on connect (e.g., --initial-command 1500 1500)'
    )
    parser.add_argument(
        '--no-handshake',
        action='store_true',
        help='Disable automatic handshake (Arduino will not respond until it receives a packet)'
    )
    parser.add_argument(
        '--keep-alive',
        action='store_true',
        help='Enable keep-alive (send PING every 1s to maintain connection)'
    )

    args = parser.parse_args()

    # Create client
    client = UDPTestClient(args.ip, args.data_port, args.heartbeat_port)

    # Connect
    if not client.connect():
        print("[ERROR] Failed to connect. Exiting.")
        return 1

    # Start receiving
    client.start_receiving()

    # Auto-handshake: Arduino needs to receive a packet first to learn client address
    if not args.no_handshake:
        print("\n[HANDSHAKE] Sending PING to trigger Arduino response...")
        if args.initial_command:
            left, right = args.initial_command
            client.send_command(left, right)
            print(f"[CMD] Sent custom command: C {left} {right}")
        client.send_handshake()
        print("[HANDSHAKE] Sent PING")
    else:
        print("\n[INFO] Auto-handshake disabled")
        print(f"       Send a UDP packet to {args.ip}:{args.data_port} to trigger Arduino response")

    # Send initial command if specified (and not in handshake mode)
    if args.initial_command and not args.no_handshake:
        pass  # Already sent in handshake
    elif args.initial_command:
        left, right = args.initial_command
        print(f"\n[CMD] Sending initial command: C {left} {right}")
        client.send_command(left, right)

    # Start keep-alive if requested
    if args.keep_alive:
        client.start_keep_alive(interval=1.0)

    # Run selected mode
    try:
        if args.mode == 'interactive':
            # Wait for first heartbeat
            print("\n[WAIT] Waiting for first heartbeat...")
            for _ in range(50):  # Wait up to 5 seconds
                if client.is_arduino_online():
                    break
                time.sleep(0.1)

            if client.is_arduino_online():
                print("[OK] Arduino is online! Starting interactive mode...\n")
                interactive_mode(client)
            else:
                print("[WARN] No heartbeat received. Check Arduino is powered and connected.")
                print("       Verify Arduino IP address and network connectivity.")

        elif args.mode == 'heartbeat':
            heartbeat_test_mode(client, args.duration)

        elif args.mode == 'thruster':
            # Wait for first heartbeat
            print("\n[WAIT] Waiting for first heartbeat...")
            for _ in range(50):  # Wait up to 5 seconds
                if client.is_arduino_online():
                    break
                time.sleep(0.1)

            if client.is_arduino_online():
                print("[OK] Arduino is online! Starting thruster test...\n")
                thruster_test_mode(client)
            else:
                print("[WARN] No heartbeat received. Check Arduino is powered and connected.")
                print("       Verify Arduino IP address and network connectivity.")

        elif args.mode == 'hz10':
            # Wait for first heartbeat
            print("\n[WAIT] Waiting for first heartbeat...")
            for _ in range(50):  # Wait up to 5 seconds
                if client.is_arduino_online():
                    break
                time.sleep(0.1)

            if client.is_arduino_online():
                print("[OK] Arduino is online! Starting 10Hz latency test...\n")
                hz_10_test_mode(client, args.duration)
            else:
                print("[WARN] No heartbeat received. Check Arduino is powered and connected.")
                print("       Verify Arduino IP address and network connectivity.")

        elif args.mode == 'monitor':
            print("\n=== Monitor Mode ===")
            print("Press Ctrl+C to stop and show statistics...")
            try:
                while True:
                    time.sleep(1)
                    online = client.is_arduino_online()
                    time_since = client.get_time_since_last_heartbeat()
                    status_icon = "✓" if online else "✗"
                    print(f"[{status_icon}] Heartbeats: {client.heartbeat_count}, "
                          f"Time since last: {time_since:.2f}s")
            except KeyboardInterrupt:
                pass

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    # Print final statistics
    client.print_statistics()

    # Cleanup
    print("\nDisconnecting...")
    client.disconnect()

    return 0


if __name__ == '__main__':
    exit(main())
