from picamera2 import Picamera2
import asyncio, websockets, cv2
import socket
from pymavlink import mavutil
import math
import json
import time
import struct

# ===== Configuration Variables =====
VIDEO_FPS = 15  
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
JPEG_QUALITY = 75  # 0-95, lower = faster but lower quality
VIDEO_PORT_1 = 8765
VIDEO_PORT_2 = 8766
TELEMETRY_PORT = 8764
TELEMETRY_HZ = 50  # 50Hz = 20ms interval for low latency

print("Connecting to MAVLink...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Awaiting MAVLink heartbeat...")
mav.wait_heartbeat()
print("MAVLink heartbeat received.")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

cam0 = Picamera2(0)
cam0.configure(cam0.create_preview_configuration(main={"format": 'XRGB8888', "size": (VIDEO_WIDTH, VIDEO_HEIGHT)}))
cam0.start()

cam1 = Picamera2(1)
cam1.configure(cam1.create_preview_configuration(main={"format": 'YUV420', "size": (VIDEO_WIDTH, VIDEO_HEIGHT)}))
cam1.start()

async def stream_cam0(ws):
    print("Client connected to video stream (cam0).")
    while True:
        frame = cam0.capture_array()
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        # Embed timestamp for latency measurement: 8-byte double + JPEG data
        timestamp_bytes = struct.pack('d', time.time())
        await ws.send(timestamp_bytes + buffer.tobytes())
        await asyncio.sleep(1.0 / VIDEO_FPS)

async def stream_cam1(ws):
    print("Client connected to video stream (cam1).")
    while True:
        frame = cam1.capture_array()
        mono_frame = frame[0:VIDEO_HEIGHT, 0:VIDEO_WIDTH] 
        ret, buffer = cv2.imencode('.jpg', mono_frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        # Embed timestamp for latency measurement: 8-byte double + JPEG data
        timestamp_bytes = struct.pack('d', time.time())
        await ws.send(timestamp_bytes + buffer.tobytes())
        await asyncio.sleep(1.0 / VIDEO_FPS)

# Global storage for latest MAVLink data (shared between WebSocket and UDP)
mavlink_data = {
    'roll': 0, 'pitch': 0, 'yaw': 0,
    'lat': 0, 'lon': 0, 'alt': 0,
    'battery': 0, 'battery_remaining': 0,
    'ground_speed': 0, 'throttle': 0,
    'timestamp': 0
}

# Set of connected WebSocket telemetry clients
telemetry_clients = set()

async def stream_telemetry(ws):
    """WebSocket handler for telemetry streaming."""
    telemetry_clients.add(ws)
    print(f"Telemetry client connected. Total clients: {len(telemetry_clients)}")
    try:
        # Keep connection alive, data is pushed via broadcast_telemetry_to_clients
        await ws.wait_closed()
    finally:
        telemetry_clients.discard(ws)
        print(f"Telemetry client disconnected. Total clients: {len(telemetry_clients)}")

async def broadcast_telemetry_to_clients():
    """Broadcast telemetry to all connected WebSocket clients at high frequency."""
    while True:
        if telemetry_clients:
            # Create JSON payload with timestamp for latency measurement
            payload = json.dumps({
                **mavlink_data,
                'server_time': time.time()
            })
            
            # Broadcast to all connected clients
            disconnected = set()
            for ws in telemetry_clients:
                try:
                    await ws.send(payload)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(ws)
            
            # Remove disconnected clients
            telemetry_clients.difference_update(disconnected)
        
        await asyncio.sleep(1.0 / TELEMETRY_HZ)  # 50Hz = 20ms

async def mavlink_broadcast():
    # Store latest messages
    latest_msgs = {}
    mavlink_connected = False
    
    async def read_mavlink():
        while True:
            msg = await asyncio.to_thread(mav.recv_match, blocking=False)
            if msg:
                latest_msgs[msg.get_type()] = msg
                nonlocal mavlink_connected
                if not mavlink_connected:
                    mavlink_connected = True
                    print("MAVLink connection established.")
            await asyncio.sleep(0.005)  # Check more frequently (200Hz)
    
    # Start mavlink reader
    asyncio.create_task(read_mavlink())
    
    # Update mavlink_data and broadcast UDP at 10Hz (legacy)
    while True:
        att_msg = latest_msgs.get('ATTITUDE')
        gps_msg = latest_msgs.get('GLOBAL_POSITION_INT')
        batt_msg = latest_msgs.get('BATTERY_STATUS')
        vfr_msg = latest_msgs.get('VFR_HUD')
        
        roll = math.degrees(att_msg.roll) if att_msg else 0
        pitch = math.degrees(att_msg.pitch) if att_msg else 0
        yaw = math.degrees(att_msg.yaw) if att_msg else 0
        
        lat = gps_msg.lat / 1e7 if gps_msg else 0
        lon = gps_msg.lon / 1e7 if gps_msg else 0
        alt = gps_msg.alt / 1000 if gps_msg else 0
        
        battery = batt_msg.voltages[0] / 1000 if batt_msg else 0
        battery_remaining = batt_msg.battery_remaining if batt_msg else 0
        
        ground_speed = vfr_msg.groundspeed if vfr_msg else 0
        throttle = vfr_msg.throttle if vfr_msg else 0
        
        # Update global mavlink_data for WebSocket telemetry
        mavlink_data.update({
            'roll': round(roll, 4),
            'pitch': round(pitch, 4),
            'yaw': round(yaw, 2),
            'lat': round(lat, 6),
            'lon': round(lon, 6),
            'alt': round(alt, 1),
            'battery': round(battery, 2),
            'battery_remaining': round(battery_remaining, 0),
            'ground_speed': round(ground_speed, 1),
            'throttle': round(throttle, 0),
            'timestamp': time.time()
        })
        
        # Legacy UDP broadcast (keep for backward compatibility)
        data = f"{roll:.4f},{pitch:.4f},{yaw:.2f},{lat:.6f},{lon:.6f},{alt:.1f},{battery:.2f},{battery_remaining:.0f},{ground_speed:.1f},{throttle:.0f}".encode()
        sock.sendto(data, ('<broadcast>', 5000))
        
        await asyncio.sleep(0.1)  # 10Hz UDP broadcast (legacy)

async def main():
    async with websockets.serve(stream_cam0, '0.0.0.0', VIDEO_PORT_1), \
            websockets.serve(stream_cam1, '0.0.0.0', VIDEO_PORT_2), \
            websockets.serve(stream_telemetry, '0.0.0.0', TELEMETRY_PORT):
        print(f"Server running:")
        print(f"  - Video cam0: ws://0.0.0.0:{VIDEO_PORT_1}")
        print(f"  - Video cam1: ws://0.0.0.0:{VIDEO_PORT_2}")
        print(f"  - Telemetry:  ws://0.0.0.0:{TELEMETRY_PORT} ({TELEMETRY_HZ}Hz)")
        print(f"  - UDP legacy: broadcast:5000 (10Hz)")
        asyncio.create_task(mavlink_broadcast())
        asyncio.create_task(broadcast_telemetry_to_clients())
        await asyncio.Future()

asyncio.run(main())

    