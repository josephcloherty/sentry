from picamera2 import Picamera2
import asyncio, websockets, cv2
import socket
from pymavlink import mavutil
import math
import json
import time
from collections import deque

# ===== Configuration Variables =====
VIDEO_FPS = 15  
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
JPEG_QUALITY = 75  # 0-95, lower = faster but lower quality
VIDEO_PORT_1 = 8765
VIDEO_PORT_2 = 8766
TELEMETRY_PORT = 8764
TELEMETRY_HZ = 50  # 50Hz telemetry updates (20ms interval)

print("Connecting to MAVLink...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Awaiting MAVLink heartbeat...")
mav.wait_heartbeat()
print("MAVLink heartbeat received.")

# Request 50Hz data stream from autopilot
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, TELEMETRY_HZ, 1
)

# Legacy UDP socket (for backward compatibility)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Telemetry state storage (shared across WebSocket clients)
telemetry_state = {
    "roll": 0, "pitch": 0, "yaw": 0,
    "lat": 0, "lon": 0, "alt": 0,
    "battery": 0, "battery_remaining": 0,
    "ground_speed": 0, "throttle": 0,
    "timestamp": time.time(),
    "server_latency_ms": 0
}
telemetry_clients = set()  # WebSocket clients
latency_buffer = deque(maxlen=100)  # Track last 100 latency measurements

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
        await ws.send(buffer.tobytes())
        await asyncio.sleep(1.0 / VIDEO_FPS)

async def stream_cam1(ws):
    print("Client connected to video stream (cam1).")
    while True:
        frame = cam1.capture_array()
        mono_frame = frame[0:VIDEO_HEIGHT, 0:VIDEO_WIDTH] 
        ret, buffer = cv2.imencode('.jpg', mono_frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        await ws.send(buffer.tobytes())
        await asyncio.sleep(1.0 / VIDEO_FPS)

async def telemetry_websocket(websocket, path):
    """WebSocket handler for telemetry - broadcasts to all connected clients"""
    telemetry_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        telemetry_clients.discard(websocket)

async def mavlink_broadcast():
    """
    Read MAVLink messages and broadcast telemetry at TELEMETRY_HZ (50Hz).
    Sends to both WebSocket clients (new) and UDP broadcast (legacy).
    """
    latest_msgs = {}
    mavlink_connected = False
    
    async def read_mavlink():
        """Read MAVLink messages continuously"""
        nonlocal mavlink_connected
        while True:
            msg = await asyncio.to_thread(mav.recv_match, blocking=False)
            if msg:
                latest_msgs[msg.get_type()] = msg
                if not mavlink_connected:
                    mavlink_connected = True
                    print("MAVLink connection established.")
            await asyncio.sleep(0.001)  # 1ms polling (non-blocking)
    
    # Start mavlink reader task
    asyncio.create_task(read_mavlink())
    
    # Broadcast at TELEMETRY_HZ (50Hz = 20ms interval)
    while True:
        start_time = time.time()
        
        # Extract telemetry from latest messages
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
        
        # Update telemetry state
        telemetry_state.update({
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "battery": battery,
            "battery_remaining": battery_remaining,
            "ground_speed": ground_speed,
            "throttle": throttle,
            "timestamp": time.time()
        })
        
        # Measure server-side latency
        server_latency = (time.time() - start_time) * 1000
        telemetry_state["server_latency_ms"] = server_latency
        latency_buffer.append(server_latency)
        
        # Broadcast via WebSocket to all connected clients
        if telemetry_clients:
            telemetry_json = json.dumps(telemetry_state)
            disconnected = set()
            for client in telemetry_clients:
                try:
                    await client.send(telemetry_json)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
            telemetry_clients.difference_update(disconnected)
        
        # Legacy UDP broadcast (backward compatibility)
        data = f"{roll:.4f},{pitch:.4f},{yaw:.2f},{lat:.6f},{lon:.6f},{alt:.1f},{battery:.2f},{battery_remaining:.0f},{ground_speed:.1f},{throttle:.0f}".encode()
        sock.sendto(data, ('<broadcast>', 5000))
        
        # Log average latency periodically
        if len(latency_buffer) == 100:
            avg_latency = sum(latency_buffer) / len(latency_buffer)
            print(f"[Telemetry] Broadcasting at {TELEMETRY_HZ}Hz, avg server latency: {avg_latency:.2f}ms, WebSocket clients: {len(telemetry_clients)}")
        
        # Sleep to maintain TELEMETRY_HZ frequency
        await asyncio.sleep(1.0 / TELEMETRY_HZ)

async def main():
    async with websockets.serve(stream_cam0, '0.0.0.0', VIDEO_PORT_1), \
            websockets.serve(stream_cam1, '0.0.0.0', VIDEO_PORT_2), \
            websockets.serve(telemetry_websocket, '0.0.0.0', TELEMETRY_PORT):
        print(f"Server running and ready for connections on ports {VIDEO_PORT_1} (cam0), {VIDEO_PORT_2} (cam1), {TELEMETRY_PORT} (telemetry).")
        asyncio.create_task(mavlink_broadcast())
        await asyncio.Future()

asyncio.run(main())

    