from picamera2 import Picamera2
import asyncio, websockets, cv2
import socket
from pymavlink import mavutil
import math

# ===== Configuration Variables =====
VIDEO_FPS = 15  
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
JPEG_QUALITY = 75  # 0-95, lower = faster but lower quality
VIDEO_PORT_1 = 8765
VIDEO_PORT_2 = 8766

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
cam1.configure(cam1.create_preview_configuration(main={"format": 'XBGR8888', "size": (VIDEO_WIDTH, VIDEO_HEIGHT)}))
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
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        await ws.send(buffer.tobytes())
        await asyncio.sleep(1.0 / VIDEO_FPS)

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
            await asyncio.sleep(0.01)
    
    # Start mavlink reader
    asyncio.create_task(read_mavlink())
    
    # Broadcast at 10Hz
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
        
        data = f"{roll:.4f},{pitch:.4f},{yaw:.2f},{lat:.6f},{lon:.6f},{alt:.1f},{battery:.2f},{battery_remaining:.0f},{ground_speed:.1f},{throttle:.0f}".encode()
        sock.sendto(data, ('<broadcast>', 5000))
        
        await asyncio.sleep(0.1)  # 10Hz (100ms)

async def main():
    async with websockets.serve(stream_cam0, '0.0.0.0', VIDEO_PORT_1), \
            websockets.serve(stream_cam1, '0.0.0.0', VIDEO_PORT_2):
        print(f"Server running and ready for connections on ports {VIDEO_PORT_1} and {VIDEO_PORT_2}.")
        asyncio.create_task(mavlink_broadcast())
        await asyncio.Future()

asyncio.run(main())

    