#!/usr/bin/env python3
"""
Arduino Thruster WiFi Control - Client Script
Connects to Arduino TCP server and sends/receives thruster commands
"""
import socket
import sys
import time
from threading import Thread

class ArduinoThrusterClient:
    def __init__(self, arduino_ip='192.168.50.100', arduino_port=8888):
        self.arduino_ip = arduino_ip
        self.arduino_port = arduino_port
        self.sock = None
        self.connected = False
        
    def connect(self):
        """Connect to Arduino TCP server"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            print(f"Connecting to Arduino at {self.arduino_ip}:{self.arduino_port}...")
            self.sock.connect((self.arduino_ip, self.arduino_port))
            self.connected = True
            print("✓ Connected to Arduino!")
            return True
        except socket.timeout:
            print("✗ Connection timeout - Arduino not responding")
            return False
        except ConnectionRefusedError:
            print(f"✗ Connection refused - Make sure Arduino is running and TCP server is listening")
            return False
        except OSError as e:
            print(f"✗ Connection error: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.sock:
            self.sock.close()
        self.connected = False
        print("Disconnected from Arduino")
        
    def send_command(self, left_us, right_us):
        """Send thruster command to Arduino
        
        Args:
            left_us: Left thruster microseconds (1100-1900)
            right_us: Right thruster microseconds (1100-1900)
        """
        if not self.connected:
            print("✗ Not connected to Arduino")
            return False
            
        # Constrain values
        left_us = max(1100, min(1900, left_us))
        right_us = max(1100, min(1900, right_us))
        
        cmd = f"C {left_us} {right_us}\n"
        
        try:
            self.sock.sendall(cmd.encode())
            print(f"→ Sent: C {left_us} {right_us}")
            return True
        except Exception as e:
            print(f"✗ Send error: {e}")
            self.connected = False
            return False
            
    def receive_status(self):
        """Receive status from Arduino in background thread"""
        buffer = ""
        while self.connected:
            try:
                data = self.sock.recv(256).decode('utf-8')
                if not data:
                    print("\n✗ Connection closed by Arduino")
                    self.connected = False
                    break
                    
                buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.startswith('S '):
                        parts = line.split()
                        if len(parts) == 3:
                            try:
                                left_us = int(parts[1])
                                right_us = int(parts[2])
                                print(f"← Status: Left={left_us}µs | Right={right_us}µs")
                            except ValueError:
                                pass
                                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"\n✗ Receive error: {e}")
                self.connected = False
                break
                
    def start_receive_thread(self):
        """Start receiving status in background"""
        thread = Thread(target=self.receive_status, daemon=True)
        thread.start()
        return thread

def main():
    """Main interactive control loop"""
    client = ArduinoThrusterClient()
    
    # Try to connect
    if not client.connect():
        sys.exit(1)
        
    # Start receiving status in background
    client.start_receive_thread()
    
    print("\n=== Thruster Control Commands ===")
    print("Commands:")
    print("  C <left> <right>  - Send command (e.g., 'C 1500 1500')")
    print("  stop              - Stop thrusters (1500 1500)")
    print("  forward           - Full forward (1900 1900)")
    print("  reverse           - Full reverse (1100 1100)")
    print("  test              - Run test sequence")
    print("  quit              - Exit\n")
    
    # Give time for receive thread to start
    time.sleep(0.5)
    
    try:
        while client.connected:
            try:
                user_input = input("cmd> ").strip().lower()
                
                if not user_input:
                    continue
                    
                elif user_input == 'quit' or user_input == 'exit':
                    break
                    
                elif user_input == 'stop':
                    client.send_command(1500, 1500)
                    
                elif user_input == 'forward':
                    client.send_command(1900, 1900)
                    
                elif user_input == 'reverse':
                    client.send_command(1100, 1100)
                    
                elif user_input == 'test':
                    print("\n=== Running Test Sequence ===")
                    test_commands = [
                        (1500, 1500, "Stop"),
                        (1600, 1500, "Turn right"),
                        (1500, 1600, "Turn left"),
                        (1700, 1700, "Forward slow"),
                        (1900, 1900, "Forward full"),
                        (1100, 1100, "Reverse full"),
                        (1500, 1500, "Stop"),
                    ]
                    
                    for left, right, desc in test_commands:
                        print(f"\n{desc}...")
                        client.send_command(left, right)
                        time.sleep(1)
                    print("\nTest sequence complete\n")
                    
                elif user_input.startswith('c '):
                    parts = user_input.split()
                    if len(parts) == 3:
                        try:
                            left = int(parts[1])
                            right = int(parts[2])
                            client.send_command(left, right)
                        except ValueError:
                            print("✗ Invalid values. Use: C <left_us> <right_us>")
                    else:
                        print("✗ Invalid format. Use: C <left_us> <right_us>")
                        
                else:
                    print("✗ Unknown command. Type 'help' for commands.")
                    
            except KeyboardInterrupt:
                print("\n")
                break
                
    finally:
        client.disconnect()
        print("Goodbye!")

if __name__ == '__main__':
    main()
