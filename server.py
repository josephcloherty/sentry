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

from functions.infrared import process_ir_frame

# ===== Configuration =====
VIDEO_FPS = 15
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
JPEG_QUALITY = 75
VIDEO_PORT_1 = 8765
VIDEO_PORT_2 = 8766
TELEMETRY_PORT = 8764
TELEMETRY_HZ = 50
COMMAND_PORT = 8763

# ===== Global State =====
mavlink_data = {
    'roll': 0, 'pitch': 0, 'yaw': 0,
    'lat': 0, 'lon': 0, 'alt': 0,
    'battery': 0, 'battery_remaining': 0,
    'ground_speed': 0, 'throttle': 0,
    'timestamp': 0
}
telemetry_clients = set()

# Command state and clients
command_state = {
    'go_dark': False,
    'night_vision': False,
    'auto_rth': False,
    'drop_gps_pin': False,
    'emergency': False,
    'loiter': False,
    'landing_mode': False,
}
command_clients = set()

# ===== MAVLink & Socket Setup =====
print("Connecting to MAVLink...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Awaiting MAVLink heartbeat...")
mav.wait_heartbeat()
print("MAVLink heartbeat received.")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


# ===== Camera Initialization =====
def detect_camera_type(cam_id):
    """Detect if camera is a regular Pi Cam or Pi Cam Noir."""
    try:
        cam = Picamera2(cam_id)
        camera_props = cam.camera_properties
        model = camera_props.get('Model', 'unknown').lower()
        
        # Pi Cam Noir typically has 'noir' in the model name
        # Also check for specific model numbers:
        # - imx219 noir, ov5647 noir, etc.
        is_noir = 'noir' in model
        
        print(f"Camera {cam_id}: Model={camera_props.get('Model', 'unknown')}, Is Noir={is_noir}")
        cam.close()
        return is_noir
    except Exception as e:
        print(f"Warning: Could not detect camera {cam_id} type: {e}")
        return None


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
        print(f"Warning: cam{cam_id} Picamera2 init failed: {e}.")
    return None, None


def discover_cameras(max_cams=2):
    cameras = []
    for i in range(max_cams):
        try:
            cam_type = detect_camera_type(i)
            if cam_type is not None:
                cameras.append({'id': i, 'is_noir': cam_type})
        except Exception as e:
            print(f"Camera {i} not available: {e}")
    return cameras


def select_camera_ids(cameras):
    cam0_id = next((c['id'] for c in cameras if not c['is_noir']), None)
    cam1_id = next((c['id'] for c in cameras if c['is_noir']), None)

    if cam0_id is None and cameras:
        cam0_id = next((c['id'] for c in cameras if c['id'] != cam1_id), None)
    if cam1_id is None and cameras:
        cam1_id = next((c['id'] for c in cameras if c['id'] != cam0_id), None)

    if len(cameras) == 1:
        only = cameras[0]
        cam0_id, cam1_id = (None, only['id']) if only['is_noir'] else (only['id'], None)

    return cam0_id, cam1_id


available_cameras = discover_cameras()
print(f"Available cameras: {available_cameras}")

cam0_id, cam1_id = select_camera_ids(available_cameras)

print(f"Assigning cam0 (color) to camera ID {cam0_id}")
print(f"Assigning cam1 (noir/IR) to camera ID {cam1_id}")

# Initialize cameras only if an ID was selected; otherwise leave as None
cam0 = None
cam1 = None
cam1_format = None
if cam0_id is not None:
    cam0, _ = init_camera(cam0_id, ['XRGB8888'], color=True)
if cam1_id is not None:
    cam1, cam1_format = init_camera(cam1_id, ['YUV420', 'XRGB8888'], color=False)


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
async def _send_unavailable(ws, label):
    print(f"{label} not available; closing connection")
    try:
        await ws.send(json.dumps({'type': 'error', 'message': f'{label} not available'}))
    except Exception:
        pass
    await ws.close()


async def _stream_camera(ws, cam, label, transform=None):
    print(f"Client connected to video stream ({label}).")
    if cam is None:
        await _send_unavailable(ws, label)
        return

    while True:
        frame = cam.capture_array()
        if transform:
            frame = transform(frame)
        await ws.send(encode_frame_with_timestamp(frame))
        await asyncio.sleep(1.0 / VIDEO_FPS)


def _process_cam1_frame(frame):
    # If the camera is returning YUV420 (I420) as a single-channel
    # vertically-stacked array, the shape will often be (H * 3/2, W).
    # In that case we only want the Y plane (top H rows) to produce
    # a final 640x480 grayscale feed — crop off the U/V planes.
    if isinstance(frame, np.ndarray) and frame.ndim == 2 and frame.shape[0] > VIDEO_HEIGHT:
        try:
            frame = frame[:VIDEO_HEIGHT, :VIDEO_WIDTH]
        except Exception:
            frame = frame[0:VIDEO_HEIGHT, 0:VIDEO_WIDTH]

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

    return display_frame


async def stream_cam0(ws):
    await _stream_camera(ws, cam0, 'cam0')


async def stream_cam1(ws):
    await _stream_camera(ws, cam1, 'cam1', transform=_process_cam1_frame)


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


# ===== Commands =====
async def broadcast_command_state():
    if not command_clients:
        return
    payload = json.dumps({'type': 'state', 'commands': command_state})
    disconnected = set()
    for ws in command_clients:
        try:
            await ws.send(payload)
        except websockets.exceptions.ConnectionClosed:
            disconnected.add(ws)
    command_clients.difference_update(disconnected)


async def command_handler(ws):
    """WebSocket handler for command toggles/pulses."""
    command_clients.add(ws)
    print(f"Command client connected. Total: {len(command_clients)}")

    # Send current state on connect
    try:
        await ws.send(json.dumps({'type': 'state', 'commands': command_state}))
    except Exception:
        pass

    try:
        async for message in ws:
            try:
                msg = json.loads(message)
            except json.JSONDecodeError:
                continue

            cmd_id = msg.get('id')
            if cmd_id not in command_state:
                continue

            msg_type = msg.get('type')
            if msg_type == 'toggle':
                command_state[cmd_id] = bool(msg.get('value'))
                await broadcast_command_state()
            elif msg_type == 'pulse':
                # Pulse commands do not change persistent state
                try:
                    await ws.send(json.dumps({'type': 'pulse', 'id': cmd_id, 'value': True}))
                except Exception:
                    pass
            else:
                continue
    finally:
        command_clients.discard(ws)
        print(f"Command client disconnected. Total: {len(command_clients)}")


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
         websockets.serve(stream_telemetry, '0.0.0.0', TELEMETRY_PORT), \
         websockets.serve(command_handler, '0.0.0.0', COMMAND_PORT):
        print(f"Server running:")
        print(f"  - Video cam0: ws://0.0.0.0:{VIDEO_PORT_1}")
        print(f"  - Video cam1: ws://0.0.0.0:{VIDEO_PORT_2}")
        print(f"  - Telemetry:  ws://0.0.0.0:{TELEMETRY_PORT} ({TELEMETRY_HZ}Hz)")
        print(f"  - Commands:   ws://0.0.0.0:{COMMAND_PORT}")
        print(f"  - UDP legacy: broadcast:5000 (10Hz)")
        asyncio.create_task(mavlink_broadcast())
        asyncio.create_task(broadcast_telemetry())
        await asyncio.Future()


asyncio.run(main())
