#!/usr/bin/env python3
"""
Latency Diagnostic Script for SENTRY UAS Dashboard

This script helps diagnose where telemetry latency is occurring.
Run this on the client machine to measure latency from different sources.

Usage: python3 latency_diagnostic.py
"""

import asyncio
import websockets
import socket
import time
import json
import statistics

# Configuration - update these to match your setup
SERVER_IP = "100.112.223.17"
TELEMETRY_WS_PORT = 8764  # WebSocket telemetry (if implemented)
UDP_PORT = 5000  # UDP broadcast port

class LatencyDiagnostic:
    def __init__(self):
        self.ws_latencies = []
        self.udp_latencies = []
        self.ws_connected = False
        self.udp_received = False
        
    async def test_websocket_telemetry(self, duration=10):
        """Test WebSocket telemetry connection and measure latency."""
        print(f"\n=== Testing WebSocket Telemetry (ws://{SERVER_IP}:{TELEMETRY_WS_PORT}) ===")
        
        try:
            async with websockets.connect(
                f'ws://{SERVER_IP}:{TELEMETRY_WS_PORT}',
                open_timeout=5
            ) as ws:
                print("✓ WebSocket connected successfully!")
                self.ws_connected = True
                
                start_time = time.time()
                message_count = 0
                
                while time.time() - start_time < duration:
                    try:
                        data = await asyncio.wait_for(ws.recv(), timeout=2)
                        receive_time = time.time()
                        
                        msg = json.loads(data)
                        server_time = msg.get('timestamp', 0)
                        server_latency = msg.get('server_latency_ms', 0)
                        
                        if server_time > 0:
                            network_latency = (receive_time - server_time) * 1000
                            total_latency = network_latency + server_latency
                            self.ws_latencies.append(total_latency)
                            
                            message_count += 1
                            if message_count <= 5 or message_count % 20 == 0:
                                print(f"  Message {message_count}: Total latency = {total_latency:.1f}ms "
                                      f"(network: {network_latency:.1f}ms, server: {server_latency:.1f}ms)")
                                
                    except asyncio.TimeoutError:
                        print("  ⚠ Timeout waiting for message")
                        
                print(f"\n✓ Received {message_count} messages in {duration} seconds")
                
        except websockets.exceptions.InvalidURI:
            print("✗ Invalid WebSocket URI")
        except ConnectionRefusedError:
            print(f"✗ Connection refused - WebSocket server not running on port {TELEMETRY_WS_PORT}")
            print("  This suggests the server.py may not have WebSocket telemetry implemented.")
        except Exception as e:
            print(f"✗ WebSocket error: {type(e).__name__}: {e}")
            
    def test_udp_telemetry(self, duration=10):
        """Test UDP telemetry broadcast and measure timing."""
        print(f"\n=== Testing UDP Telemetry (port {UDP_PORT}) ===")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', UDP_PORT))
        sock.settimeout(2)
        
        start_time = time.time()
        message_count = 0
        last_receive_time = None
        intervals = []
        
        try:
            while time.time() - start_time < duration:
                try:
                    data, addr = sock.recvfrom(1024)
                    receive_time = time.time()
                    self.udp_received = True
                    
                    if last_receive_time:
                        interval = (receive_time - last_receive_time) * 1000
                        intervals.append(interval)
                        
                    last_receive_time = receive_time
                    message_count += 1
                    
                    if message_count <= 3:
                        values = data.decode().split(',')
                        print(f"  Message {message_count} from {addr}: {len(values)} fields")
                        
                except socket.timeout:
                    print("  ⚠ Timeout waiting for UDP packet")
                    
            print(f"\n✓ Received {message_count} UDP packets in {duration} seconds")
            
            if intervals:
                avg_interval = statistics.mean(intervals)
                print(f"  Average interval between packets: {avg_interval:.1f}ms")
                print(f"  This corresponds to approximately {1000/avg_interval:.1f} Hz update rate")
                
        except Exception as e:
            print(f"✗ UDP error: {e}")
        finally:
            sock.close()
            
    async def test_video_websocket(self, port, name, duration=5):
        """Test video WebSocket connection."""
        print(f"\n=== Testing Video WebSocket ({name}, ws://{SERVER_IP}:{port}) ===")
        
        try:
            async with websockets.connect(
                f'ws://{SERVER_IP}:{port}',
                open_timeout=5
            ) as ws:
                print(f"✓ {name} WebSocket connected!")
                
                start_time = time.time()
                frame_count = 0
                total_bytes = 0
                
                while time.time() - start_time < duration:
                    try:
                        data = await asyncio.wait_for(ws.recv(), timeout=2)
                        frame_count += 1
                        total_bytes += len(data)
                    except asyncio.TimeoutError:
                        print("  ⚠ Timeout waiting for frame")
                        break
                        
                fps = frame_count / duration
                bandwidth = (total_bytes / 1024) / duration
                print(f"  Received {frame_count} frames in {duration}s ({fps:.1f} FPS)")
                print(f"  Average bandwidth: {bandwidth:.1f} KB/s")
                
        except ConnectionRefusedError:
            print(f"✗ {name} - Connection refused (server not running)")
        except Exception as e:
            print(f"✗ {name} error: {e}")
            
    def print_summary(self):
        """Print diagnostic summary."""
        print("\n" + "="*60)
        print("DIAGNOSTIC SUMMARY")
        print("="*60)
        
        print("\n1. WebSocket Telemetry:")
        if self.ws_connected and self.ws_latencies:
            avg = statistics.mean(self.ws_latencies)
            min_lat = min(self.ws_latencies)
            max_lat = max(self.ws_latencies)
            print(f"   ✓ Connected and receiving data")
            print(f"   Average latency: {avg:.1f}ms")
            print(f"   Min/Max latency: {min_lat:.1f}ms / {max_lat:.1f}ms")
            if avg > 500:
                print(f"   ⚠ HIGH LATENCY - Check network or server CPU")
            elif avg > 200:
                print(f"   ⚠ Moderate latency - Acceptable but could be improved")
            else:
                print(f"   ✓ Good latency!")
        else:
            print(f"   ✗ Not connected or no data received")
            print(f"   SUGGESTION: Check if server.py has WebSocket telemetry on port {TELEMETRY_WS_PORT}")
            
        print("\n2. UDP Telemetry:")
        if self.udp_received:
            print(f"   ✓ Receiving UDP broadcast data")
        else:
            print(f"   ✗ Not receiving UDP data")
            
        print("\n3. Potential Causes of High Latency:")
        print("   - Server CPU overload (check server CPU usage)")
        print("   - Network congestion or high ping")
        print("   - Low update frequency (10Hz = 100ms minimum latency)")
        print("   - WebSocket telemetry not implemented on server")
        
        print("\n4. Recommendations:")
        if not self.ws_connected:
            print("   ★ Implement WebSocket telemetry on server.py (port 8764)")
            print("   ★ Current server.py appears to only use UDP broadcast")
        print("   - Increase telemetry frequency to 50Hz (20ms interval)")
        print("   - Monitor server CPU/memory usage")
        print("   - Check network latency: ping " + SERVER_IP)

async def main():
    print("="*60)
    print("SENTRY UAS Dashboard - Latency Diagnostic Tool")
    print("="*60)
    print(f"Server IP: {SERVER_IP}")
    print(f"Testing duration: 10 seconds per test")
    
    diag = LatencyDiagnostic()
    
    # Test video streams
    await diag.test_video_websocket(8765, "Camera 0", duration=5)
    await diag.test_video_websocket(8766, "Camera 1", duration=5)
    
    # Test telemetry
    await diag.test_websocket_telemetry(duration=10)
    diag.test_udp_telemetry(duration=10)
    
    # Print summary
    diag.print_summary()

if __name__ == "__main__":
    asyncio.run(main())
