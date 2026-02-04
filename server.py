import asyncio
import json
import math
import socket
import struct
import time

import cv2
import numpy as np
import websockets
from picamera2 import Picamera2
from pymavlink import mavutil

from infrared import process_ir_frame

# ===== Configuration =====
VIDEO_FPS = 15
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
JPEG_QUALITY = 75
VIDEO_PORT_1 = 8765
VIDEO_PORT_2 = 8766
TELEMETRY_PORT = 8764
TELEMETRY_HZ = 50

# ===== Global State =====
mavlink_data = {
    'roll': 0, 'pitch': 0, 'yaw': 0,
    'lat': 0, 'lon': 0, 'alt': 0,
    'battery': 0, 'battery_remaining': 0,
    'ground_speed': 0, 'throttle': 0,
    'timestamp': 0
}
telemetry_clients = set()

# ===== MAVLink & Socket Setup =====
print("Connecting to MAVLink...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Awaiting MAVLink heartbeat...")
mav.wait_heartbeat()
print("MAVLink heartbeat received.")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


# ===== Mock Camera =====
class MockCamera:
    """Fallback camera generating synthetic frames when Picamera2 unavailable."""
    def __init__(self, width, height, color=True):
        self.w, self.h, self.color, self._t = width, height, color, 0

    def capture_array(self):
        self._t += 1
        cx = int((self._t * 5) % (self.w + 40)) - 20
        if self.color:
            img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
            cv2.circle(img, (max(0, cx), self.h // 2), 20, (255, 255, 255), -1)
        else:
            img = np.zeros((self.h, self.w), dtype=np.uint8)
            cv2.circle(img, (max(0, cx), self.h // 2), 20, 255, -1)
        return img


# ===== Camera Initialization =====
def init_camera(cam_id, formats, color=True):
    """Initialize camera with format fallback chain."""
    try:
        cam = Picamera2(cam_id)
        for fmt in formats:
            try:
                cam.configure(cam.create_preview_configuration(
                    main={"format": fmt, "size": (VIDEO_WIDTH, VIDEO_HEIGHT)}
                ))
                cam.start()
                print(f"cam{cam_id}: Picamera2 started ({fmt})")
                return cam, fmt
            except Exception:
                continue
    except Exception as e:
        print(f"Warning: cam{cam_id} Picamera2 init failed: {e}. Using MockCamera.")
    return MockCamera(VIDEO_WIDTH, VIDEO_HEIGHT, color=color), 'MOCK'


cam0, _ = init_camera(1, ['XRGB8888'], color=True)
cam1, cam1_format = init_camera(0, ['YUV420', 'XRGB8888'], color=False)


# ===== Helper Functions =====
def encode_frame_with_timestamp(frame):
    """Encode frame as JPEG with prepended timestamp."""
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return struct.pack('d', time.time()) + buffer.tobytes()


def to_grayscale(frame):
    """Convert frame to grayscale handling various formats."""
    if not isinstance(frame, np.ndarray):
        return frame
    if frame.ndim == 2:
        return frame
    if frame.ndim == 3:
        if frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)
        try:
            return cv2.cvtColor(frame, cv2.COLOR_YUV2GRAY_I420)
        except Exception:
            return frame[..., 0]
    return frame


def find_brightest_region(gray):
    """Find brightest region in grayscale image, returns (center, radius) or (None, None)."""
    _, maxV, _, _ = cv2.minMaxLoc(gray)
    if maxV < 180:
        return None, None
    
    thresh_val = int(maxV * 0.65)
    _, th = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, None
    
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area <= 10:
        return None, None
    
    M = cv2.moments(c)
    if M.get('m00', 0) == 0:
        return None, None
    
    return (int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])), max(5, int(math.sqrt(area) / 2))


def draw_crosshair(frame, center, radius):
    """Draw circle and crosshair at center point."""
    cv2.circle(frame, center, radius, (255, 255, 255), 2)
    cv2.line(frame, (center[0]-10, center[1]), (center[0]+10, center[1]), (255, 255, 255), 1)
    cv2.line(frame, (center[0], center[1]-10), (center[0], center[1]+10), (255, 255, 255), 1)


# ===== Video Streaming =====
async def stream_cam0(ws):
    print("Client connected to video stream (cam0).")
    while True:
        frame = cam0.capture_array()
        await ws.send(encode_frame_with_timestamp(frame))
        await asyncio.sleep(1.0 / VIDEO_FPS)


async def stream_cam1(ws):
    print("Client connected to video stream (cam1).")
    while True:
        frame = cam1.capture_array()[:VIDEO_HEIGHT, :VIDEO_WIDTH]
        
        try:
            gray = to_grayscale(frame)
        except Exception as e:
            print(f"Warning: failed to convert frame to gray: {e}")
            gray = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.uint8)
        
        ir = process_ir_frame(gray)
        display_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        if ir.locked:
            center = (int(ir.cx), int(ir.cy))
            radius = max(5, int(math.sqrt(ir.area) / 2))
            draw_crosshair(display_frame, center, radius)
        else:
            try:
                center, radius = find_brightest_region(gray)
                if center:
                    draw_crosshair(display_frame, center, radius)
            except Exception as e:
                print(f"Warning: brightest-region fallback failed: {e}")
        
        await ws.send(encode_frame_with_timestamp(display_frame))
        await asyncio.sleep(1.0 / VIDEO_FPS)


# ===== Telemetry =====
async def stream_telemetry(ws):
    """WebSocket handler for telemetry streaming."""
    telemetry_clients.add(ws)
    print(f"Telemetry client connected. Total: {len(telemetry_clients)}")
    try:
        await ws.wait_closed()
    finally:
        telemetry_clients.discard(ws)
        print(f"Telemetry client disconnected. Total: {len(telemetry_clients)}")


async def broadcast_telemetry():
    """Broadcast telemetry to all connected WebSocket clients."""
    while True:
        if telemetry_clients:
            payload = json.dumps({**mavlink_data, 'server_time': time.time()})
            disconnected = set()
            for ws in telemetry_clients:
                try:
                    await ws.send(payload)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(ws)
            telemetry_clients.difference_update(disconnected)
        await asyncio.sleep(1.0 / TELEMETRY_HZ)


async def mavlink_broadcast():
    """Read MAVLink data and broadcast via UDP."""
    latest_msgs = {}
    mavlink_connected = False

    async def read_mavlink():
        nonlocal mavlink_connected
        while True:
            msg = await asyncio.to_thread(mav.recv_match, blocking=False)
            if msg:
                latest_msgs[msg.get_type()] = msg
                if not mavlink_connected:
                    mavlink_connected = True
                    print("MAVLink connection established.")
            await asyncio.sleep(0.005)

    asyncio.create_task(read_mavlink())

    while True:
        att = latest_msgs.get('ATTITUDE')
        gps = latest_msgs.get('GLOBAL_POSITION_INT')
        batt = latest_msgs.get('BATTERY_STATUS')
        vfr = latest_msgs.get('VFR_HUD')

        roll = math.degrees(att.roll) if att else 0
        pitch = math.degrees(att.pitch) if att else 0
        yaw = math.degrees(att.yaw) if att else 0
        lat = gps.lat / 1e7 if gps else 0
        lon = gps.lon / 1e7 if gps else 0
        alt = gps.alt / 1000 if gps else 0
        battery = batt.voltages[0] / 1000 if batt else 0
        battery_remaining = batt.battery_remaining if batt else 0
        ground_speed = vfr.groundspeed if vfr else 0
        throttle = vfr.throttle if vfr else 0

        mavlink_data.update({
            'roll': round(roll, 4), 'pitch': round(pitch, 4), 'yaw': round(yaw, 2),
            'lat': round(lat, 6), 'lon': round(lon, 6), 'alt': round(alt, 1),
            'battery': round(battery, 2), 'battery_remaining': round(battery_remaining, 0),
            'ground_speed': round(ground_speed, 1), 'throttle': round(throttle, 0),
            'timestamp': time.time()
        })

        # Legacy UDP broadcast
        data = f"{roll:.4f},{pitch:.4f},{yaw:.2f},{lat:.6f},{lon:.6f},{alt:.1f},{battery:.2f},{battery_remaining:.0f},{ground_speed:.1f},{throttle:.0f}".encode()
        sock.sendto(data, ('<broadcast>', 5000))
        await asyncio.sleep(0.1)


# ===== Main =====
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
        asyncio.create_task(broadcast_telemetry())
        await asyncio.Future()


asyncio.run(main())
