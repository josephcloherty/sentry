#!/usr/bin/env python3
"""
Low-latency H.264 video streaming server with telemetry broadcast
Runs on Raspberry Pi - streams raw H.264 video over TCP and telemetry via WebSocket
"""

import asyncio
import socket
import time
import math
import json
import sys
import os
import threading

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
from pymavlink import mavutil
import socketio

# Configuration
VIDEO_PORT = 8000
TELEMETRY_PORT = 8001
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
VIDEO_FPS = 30
VIDEO_BITRATE = 2000000  # 2 Mbps
TELEMETRY_RATE = 10  # Hz

# Initialize telemetry dictionary
telemetry = {
    'roll': 0, 'pitch': 0, 'yaw': 0,
    'lat': 0, 'lon': 0, 'alt': 0, 'heading': 0,
    'ground_speed': 0, 'climb_rate': 0,
    'battery': 0, 'current_battery': 0, 'battery_remaining': 0,
    'airspeed': 0, 'throttle': 0,
    'satellites': 0, 'fix_type': 0, 'h_acc': 0, 'v_acc': 0,
    'accel_x': 0, 'accel_y': 0, 'accel_z': 0,
    'gyro_x': 0, 'gyro_y': 0, 'gyro_z': 0,
    'rc_channels': (0, 0, 0, 0, 0, 0, 0, 0),
    'cpu_temp': 0,
    'cpu_usage': 0,
    'timestamp': 0
}


class TcpStreamOutput(FileOutput):
    """Custom output that streams H.264 directly to TCP socket"""
    def __init__(self, sock):
        # Don't call parent init - we'll write directly
        self.sock = sock
        
    def outputframe(self, frame, keyframe=True, timestamp=None):
        """Write frame data directly to socket"""
        try:
            # Write the raw H.264 bytes to socket
            self.sock.sendall(bytes(frame))
        except (BrokenPipeError, ConnectionResetError):
            # Client disconnected
            raise
        except Exception as e:
            print(f"Error writing to socket: {e}")
            raise


def get_cpu_stats():
    """Get CPU temperature and usage"""
    try:
        import subprocess
        import psutil
        
        # Get temperature (Raspberry Pi specific)
        temp_output = subprocess.check_output(['vcgencmd', 'measure_temp']).decode()
        temp = float(temp_output.split('=')[1].split("'")[0])
        
        # Get CPU usage
        cpu_usage = int(psutil.cpu_percent(interval=0.1))
        
        return temp, cpu_usage
    except Exception as e:
        print(f"Error getting CPU stats: {e}")
        return 0, 0


def update_telemetry(master):
    """Update telemetry dictionary from MAVLink (non-blocking)"""
    msg = master.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'SYS_STATUS', 
                                   'GPS_RAW_INT', 'VFR_HUD', 'HIGHRES_IMU', 'RC_CHANNELS'], 
                           blocking=False)
    
    if msg:
        msg_type = msg.get_type()
        
        if msg_type == 'ATTITUDE':
            telemetry['roll'] = math.degrees(msg.roll)
            telemetry['pitch'] = math.degrees(msg.pitch)
            telemetry['yaw'] = math.degrees(msg.yaw)

        elif msg_type == 'GLOBAL_POSITION_INT':
            telemetry['lat'] = msg.lat / 1e7
            telemetry['lon'] = msg.lon / 1e7
            telemetry['alt'] = msg.relative_alt / 1000.0
            telemetry['heading'] = msg.hdg / 100.0

        elif msg_type == 'VFR_HUD':
            telemetry['ground_speed'] = msg.groundspeed
            telemetry['climb_rate'] = msg.climb
            telemetry['airspeed'] = msg.airspeed
            telemetry['throttle'] = msg.throttle

        elif msg_type == 'GPS_RAW_INT':
            telemetry['satellites'] = msg.satellites_visible
            telemetry['fix_type'] = msg.fix_type
            telemetry['h_acc'] = msg.eph / 100.0
            telemetry['v_acc'] = msg.epv / 100.0

        elif msg_type == 'SYS_STATUS':
            telemetry['battery'] = msg.voltage_battery / 1000.0
            telemetry['current_battery'] = msg.current_battery / 100.0
            telemetry['battery_remaining'] = msg.battery_remaining

        elif msg_type == 'HIGHRES_IMU':
            telemetry['accel_x'] = msg.xacc / 1000.0
            telemetry['accel_y'] = msg.yacc / 1000.0
            telemetry['accel_z'] = msg.zacc / 1000.0
            telemetry['gyro_x'] = msg.xgyro / 1000.0
            telemetry['gyro_y'] = msg.ygyro / 1000.0
            telemetry['gyro_z'] = msg.zgyro / 1000.0

        elif msg_type == 'RC_CHANNELS':
            telemetry['rc_channels'] = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, 
                                       msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, 
                                       msg.chan7_raw, msg.chan8_raw)


def handle_video_client(sock, addr, picam2_instance=None):
    """Handle H.264 video streaming to a client (runs in thread)"""
    print(f"Video client connected: {addr}")
    
    try:
        # Set TCP_NODELAY to disable Nagle's algorithm (reduce latency)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        
        # Create new Picamera2 instance for this client
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (VIDEO_WIDTH, VIDEO_HEIGHT), "format": "YUV420"},
            encode="main",
            buffer_count=2,  # Minimize buffering
            queue=False
        )
        picam2.configure(config)
        
        # Configure H.264 encoder for low latency
        encoder = H264Encoder()
        encoder.bitrate = VIDEO_BITRATE
        encoder.framerate = VIDEO_FPS
        encoder.intra_period = 15  # Keyframe every 0.5s at 30fps
        
        # Start encoding and streaming to socket
        output = TcpStreamOutput(sock)
        picam2.start_recording(encoder, output)
        
        print(f"Streaming H.264 to {addr}")
        
        # Keep streaming until client disconnects
        while True:
            time.sleep(0.1)
            
    except (BrokenPipeError, ConnectionResetError):
        print(f"Client disconnected: {addr}")
    except Exception as e:
        print(f"Video streaming error for {addr}: {e}")
    finally:
        try:
            picam2.stop_recording()
            picam2.close()
        except:
            pass
        try:
            sock.close()
        except:
            pass
        print(f"Video client closed: {addr}")


def start_video_server_thread():
    """Start TCP server for H.264 video streaming in a thread"""
    def video_server():
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', VIDEO_PORT))
        server_sock.listen(5)
        
        print(f"Video server listening on 0.0.0.0:{VIDEO_PORT}")
        
        try:
            while True:
                client_sock, addr = server_sock.accept()
                # Handle each client in a separate thread
                client_thread = threading.Thread(
                    target=handle_video_client, 
                    args=(client_sock, addr),
                    daemon=True
                )
                client_thread.start()
        except KeyboardInterrupt:
            pass
        finally:
            server_sock.close()
    
    server_thread = threading.Thread(target=video_server, daemon=True)
    server_thread.start()
    return server_thread
    
    async with server:
        await server.serve_forever()


async def telemetry_broadcaster(master, sio):
    """Broadcast telemetry at fixed rate"""
    interval = 1.0 / TELEMETRY_RATE
    
    while True:
        start_time = time.time()
        
        # Update telemetry from MAVLink
        update_telemetry(master)
        
        # Get CPU stats
        try:
            temp, cpu = get_cpu_stats()
            telemetry['cpu_temp'] = temp
            telemetry['cpu_usage'] = cpu
        except:
            pass
        
        # Add timestamp
        telemetry['timestamp'] = time.time()
        
        # Broadcast to all connected clients
        await sio.emit('telemetry', telemetry)
        
        # Maintain fixed rate
        elapsed = time.time() - start_time
        sleep_time = max(0, interval - elapsed)
        await asyncio.sleep(sleep_time)


async def start_telemetry_server(master):
    """Start WebSocket server for telemetry broadcasting"""
    # Create Socket.IO server
    sio = socketio.AsyncServer(
        async_mode='aiohttp',
        cors_allowed_origins='*',
        engineio_logger=False,
        logger=False,
        ping_timeout=60,
        ping_interval=25
    )
    
    # Create aiohttp web application
    from aiohttp import web
    app = web.Application()
    sio.attach(app)
    
    @sio.event
    async def connect(sid, environ):
        print(f"Telemetry client connected: {sid}")
    
    @sio.event
    async def disconnect(sid):
        print(f"Telemetry client disconnected: {sid}")
    
    # Start telemetry broadcaster
    asyncio.create_task(telemetry_broadcaster(master, sio))
    
    # Start web server
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', TELEMETRY_PORT)
    await site.start()
    
    print(f"Telemetry server listening on 0.0.0.0:{TELEMETRY_PORT}")
    
    # Keep running
    while True:
        await asyncio.sleep(3600)


async def main():
    """Main entry point"""
    print("Low-latency H.264 Streaming Server")
    print("=" * 50)
    
    # Connect to MAVLink
    print("Connecting to mavlink-router...")
    master = None
    try:
        master = mavutil.mavlink_connection('udpin:localhost:14550')
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print("MAVLink connected!")
        
        # Request data stream at 10 Hz
        master.mav.request_data_stream_send(
            master.target_system, 
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 
            10, 
            1
        )
    except Exception as e:
        print(f"MAVLink connection error: {e}")
        print("Continuing without MAVLink (telemetry will be zero)")
    
    print("\nStarting servers...")
    print(f"  Video (H.264): TCP port {VIDEO_PORT}")
    print(f"  Telemetry: WebSocket port {TELEMETRY_PORT}")
    print(f"  Telemetry rate: {TELEMETRY_RATE} Hz")
    print("\nPress Ctrl+C to stop")
    
    # Start video server in background thread
    start_video_server_thread()
    
    # Start telemetry server (async)
    try:
        await start_telemetry_server(master)
    except KeyboardInterrupt:
        print("\nShutting down...")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExited")
