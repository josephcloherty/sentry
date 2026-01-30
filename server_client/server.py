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

print("Connecting to MAVLink...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"format": 'XRGB8888', "size": (VIDEO_WIDTH, VIDEO_HEIGHT)}))
cam.start()

async def stream(ws):
    while True:
        frame = cam.capture_array()
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        await ws.send(buffer.tobytes())
        await asyncio.sleep(1.0 / VIDEO_FPS)

async def mavlink_broadcast():
    while True:
        # Get ATTITUDE data
        att_msg = await asyncio.to_thread(mav.recv_match, type='ATTITUDE', blocking=True)
        gps_msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        batt_msg = mav.recv_match(type='BATTERY_STATUS', blocking=False)
        vfr_msg = mav.recv_match(type='VFR_HUD', blocking=False)
        
        if att_msg:
            roll = math.degrees(att_msg.roll)
            pitch = math.degrees(att_msg.pitch)
            yaw = math.degrees(att_msg.yaw)
            
            # Default values if messages not available
            lat = gps_msg.lat / 1e7 if gps_msg else 0
            lon = gps_msg.lon / 1e7 if gps_msg else 0
            alt = gps_msg.alt / 1000 if gps_msg else 0
            
            battery = batt_msg.voltages[0] / 1000 if batt_msg else 0
            battery_remaining = batt_msg.battery_remaining if batt_msg else 0
            
            ground_speed = vfr_msg.groundspeed if vfr_msg else 0
            throttle = vfr_msg.throttle if vfr_msg else 0
            
            data = f"{roll:.2f},{pitch:.2f},{yaw:.2f},{lat:.6f},{lon:.6f},{alt:.1f},{battery:.2f},{battery_remaining:.0f},{ground_speed:.1f},{throttle:.0f}".encode()
            sock.sendto(data, ('<broadcast>', 5000))

async def main():
    async with websockets.serve(stream, '0.0.0.0', 8765):
        asyncio.create_task(mavlink_broadcast())
        await asyncio.Future()

asyncio.run(main())

    