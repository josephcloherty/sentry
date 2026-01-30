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
        msg = await asyncio.to_thread(mav.recv_match, type='ATTITUDE', blocking=True)
        if msg:
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            data = f"{roll:.2f},{pitch:.2f},{yaw:.2f}".encode()
            sock.sendto(data, ('<broadcast>', 5000))

async def main():
    async with websockets.serve(stream, '0.0.0.0', 8765):
        asyncio.create_task(mavlink_broadcast())
        await asyncio.Future()

asyncio.run(main())

    