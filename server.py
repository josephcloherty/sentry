import asyncio
import json
import math
import socket
import struct
import time
import threading

import cv2
import numpy as np
import websockets
from picamera2 import Picamera2
from pymavlink import mavutil

from fiducial_tracker import process_fiducial_frame

# ===== Configuration =====
VIDEO_FPS = 15
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
JPEG_QUALITY = 75
VIDEO_PORT_1 = 8765
VIDEO_PORT_2 = 8766
VIDEO_PORT_HQ = 8767
GOPRO_UDP_URL = 'udp://@0.0.0.0:8554?overrun_nonfatal=1&fifo_size=2000000'
HQ_CAPTURE_SOURCE = '/dev/video42'
HQ_CAPTURE_FALLBACK_SOURCE = GOPRO_UDP_URL
HQ_CAPTURE_BACKEND = cv2.CAP_V4L2
HQ_USE_FALLBACK_SOURCE = True
HQ_VIDEO_WIDTH = 1280
HQ_VIDEO_HEIGHT = 720
HQ_VIDEO_FPS = 15
HQ_FORCE_RESIZE = True
HQ_FORCE_MJPEG = True
HQ_BUFFER_SIZE = 1
HQ_FLUSH_GRABS = 2
HQ_CROP_BOTTOM_PX = 0
HQ_CROP_RIGHT_PX = 0
HQ_TARGET_WIDTH = 854
HQ_TARGET_HEIGHT = 480
HQ_JPEG_QUALITY = 60
FIDUCIAL_ENABLE = True
FIDUCIAL_SEND_RATE_HZ = 10
FIDUCIAL_MIN_CONFIDENCE = 0.2
FIDUCIAL_INCLUDE_CORNERS = True
TELEMETRY_PORT = 8764
TELEMETRY_HZ = 50
COMMAND_PORT = 8763
FIDUCIAL_PORT = 8770
MAVLINK_CONNECTION_STRING = 'udp:127.0.0.1:14550'
MAVLINK_ALLOWED_MODES = [
    'STABILIZE',
    'ALT_HOLD',
    'LOITER',
    'GUIDED',
    'AUTO',
    'RTL',
    'LAND'
]
GUIDED_GOTO_DEFAULT_ALT_M = 20.0
GUIDED_GOTO_MIN_ALT_M = 1.0
GUIDED_GOTO_MAX_ALT_M = 500.0
GUIDED_GOTO_LAT_MIN = -90.0
GUIDED_GOTO_LAT_MAX = 90.0
GUIDED_GOTO_LON_MIN = -180.0
GUIDED_GOTO_LON_MAX = 180.0
GUIDED_GOTO_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

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
    'auto_rth': False,
    'drop_gps_pin': False,
    'emergency': False,
    'loiter': False,
    'landing_mode': False,
}
command_clients = set()

# ===== MAVLink & Socket Setup =====
print("Connecting to MAVLink...")
mav = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING)
print(f"MAVLink connection endpoint: {MAVLINK_CONNECTION_STRING}")
print("Awaiting MAVLink heartbeat...")
mav.wait_heartbeat()
print("MAVLink heartbeat received.")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


# ===== Camera Initialization =====
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


def init_hq_capture(source):
    """Initialize HQ camera capture from a device or UDP stream."""
    try:
        if isinstance(source, int):
            print(f"Attempting to open HQ device index: {source}...")
            cap = cv2.VideoCapture(source, HQ_CAPTURE_BACKEND)
        elif isinstance(source, str) and source.startswith('/dev/video'):
            print(f"Attempting to open HQ device path: {source}...")
            cap = cv2.VideoCapture(source, HQ_CAPTURE_BACKEND)
        else:
            print(f"Attempting to open HQ stream: {source}...")
            cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)

        if not cap.isOpened():
            raise RuntimeError("Failed to open HQ camera source")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, HQ_VIDEO_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HQ_VIDEO_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, HQ_VIDEO_FPS)
        if HQ_BUFFER_SIZE is not None:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, HQ_BUFFER_SIZE)
        print("HQ camera initialized.")
        return cap
    except Exception as e:
        print(f"Warning: HQ camera init failed: {e}")
        return None


def serialize_fiducial_result(result, frame_width=None, frame_height=None):
    payload = {
        'locked': bool(result.locked),
        'error_x': float(result.error_x),
        'error_y': float(result.error_y),
        'area': float(result.area),
        'confidence': float(result.confidence),
        'fiducial_id': result.fiducial_id,
        'timestamp': time.time()
    }
    if frame_width is not None and frame_height is not None:
        try:
            payload['frame_width'] = int(frame_width)
            payload['frame_height'] = int(frame_height)
        except Exception:
            pass
    corners = getattr(result, 'corners', None)
    if FIDUCIAL_INCLUDE_CORNERS and corners is not None:
        try:
            payload['corners'] = np.array(corners).astype(int).tolist()
        except Exception:
            pass
    return payload


def ensure_hq_thread_started():
    global hq_thread_started
    if hq_capture is None or not hq_capture.isOpened():
        return
    if hq_thread_started:
        return
    hq_thread_started = True

    def hq_capture_loop():
        global hq_latest, hq_fiducial_latest
        while True:
            try:
                for _ in range(max(0, HQ_FLUSH_GRABS)):
                    hq_capture.grab()
            except Exception:
                pass

            ret, frame = hq_capture.read()
            if not ret or frame is None:
                time.sleep(1.0 / HQ_VIDEO_FPS)
                continue

            if HQ_CROP_BOTTOM_PX > 0:
                frame = frame[:-HQ_CROP_BOTTOM_PX, :]
            if HQ_CROP_RIGHT_PX > 0:
                frame = frame[:, :-HQ_CROP_RIGHT_PX]

            if HQ_FORCE_RESIZE and (frame.shape[1] != HQ_VIDEO_WIDTH or frame.shape[0] != HQ_VIDEO_HEIGHT):
                try:
                    frame = cv2.resize(frame, (HQ_VIDEO_WIDTH, HQ_VIDEO_HEIGHT))
                except Exception:
                    pass

            fiducial_frame = frame
            fid_h, fid_w = fiducial_frame.shape[:2]

            if HQ_TARGET_WIDTH and HQ_TARGET_HEIGHT:
                try:
                    frame = cv2.resize(frame, (HQ_TARGET_WIDTH, HQ_TARGET_HEIGHT))
                except Exception:
                    pass

            if FIDUCIAL_ENABLE:
                try:
                    res = process_fiducial_frame(fiducial_frame)
                    if res is not None:
                        payload = serialize_fiducial_result(
                            res,
                            frame_width=fid_w,
                            frame_height=fid_h
                        )
                        if payload['confidence'] < FIDUCIAL_MIN_CONFIDENCE:
                            payload['locked'] = False
                        with hq_fiducial_lock:
                            hq_fiducial_latest = payload
                except Exception as e:
                    print(f"Warning: fiducial processing failed: {e}")

            with hq_lock:
                hq_latest = frame

    threading.Thread(target=hq_capture_loop, daemon=True).start()


# Detect available cameras and their types..
available_cameras = []
for i in range(2):
    try:
        cam_type = detect_camera_type(i)
        if cam_type is not None:
            available_cameras.append({'id': i, 'is_noir': cam_type})
    except Exception as e:
        print(f"Camera {i} not available: {e}")

print(f"Available cameras: {available_cameras}")

# Assign cameras ensuring cam0=normal, cam1=noir
cam0_id = None
cam1_id = None

for cam_info in available_cameras:
    if not cam_info['is_noir'] and cam0_id is None:
        cam0_id = cam_info['id']
    elif cam_info['is_noir'] and cam1_id is None:
        cam1_id = cam_info['id']

# If we can't find both types, use whatever is available
if cam0_id is None and available_cameras:
    # Use first non-noir or any available camera
    for cam_info in available_cameras:
        if cam_info['id'] != cam1_id:
            cam0_id = cam_info['id']
            break

if cam1_id is None and available_cameras:
    # Use first noir or any available camera
    for cam_info in available_cameras:
        if cam_info['id'] != cam0_id:
            cam1_id = cam_info['id']
            break

# If only one camera was detected, assign it to the correct feed; leave the other None.
if len(available_cameras) == 0:
    cam0_id = None
    cam1_id = None

if len(available_cameras) == 1:
    only = available_cameras[0]
    if only['is_noir']:
        cam0_id = None
        cam1_id = only['id']
    else:
        cam0_id = only['id']
        cam1_id = None

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

hq_capture = init_hq_capture(HQ_CAPTURE_SOURCE)
if (hq_capture is None or not hq_capture.isOpened()) and HQ_USE_FALLBACK_SOURCE:
    hq_capture = init_hq_capture(HQ_CAPTURE_FALLBACK_SOURCE)
hq_latest = None
hq_lock = threading.Lock()
hq_thread_started = False
fiducial_clients = set()
hq_fiducial_latest = None
hq_fiducial_lock = threading.Lock()


# ===== Helper Functions =====
def encode_frame_with_timestamp(frame):
    """Encode frame as JPEG with prepended timestamp."""
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return struct.pack('d', time.time()) + buffer.tobytes()


def encode_frame_with_timestamp_quality(frame, quality):
    """Encode frame as JPEG with prepended timestamp and custom quality."""
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return struct.pack('d', time.time()) + buffer.tobytes()


def send_arm_command(armed):
    """Send a MAVLink arm/disarm command."""
    try:
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if armed else 0,
            0,
            0,
            0,
            0,
            0,
            0
        )
        return True, 'Arm command sent' if armed else 'Disarm command sent'
    except Exception as e:
        return False, f'Failed to send arm command: {e}'


def set_vehicle_mode(mode):
    """Set the vehicle flight mode if it is allowed."""
    if not mode:
        return False, 'Missing mode'

    mode_name = str(mode).strip().upper()
    if mode_name not in MAVLINK_ALLOWED_MODES:
        return False, f'Mode {mode_name} not allowed'

    try:
        mav.set_mode_apm(mode_name)
        return True, f'Mode set to {mode_name}'
    except Exception:
        pass

    try:
        mode_mapping = mav.mode_mapping() or {}
        custom_mode = mode_mapping.get(mode_name)
        if custom_mode is None:
            return False, f'Mode {mode_name} not supported'
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode
        )
        return True, f'Mode set to {mode_name}'
    except Exception as e:
        return False, f'Failed to set mode: {e}'


def set_guided_mode():
    """Attempt to set vehicle mode to GUIDED."""
    try:
        mav.set_mode_apm('GUIDED')
        return True
    except Exception:
        pass

    try:
        mode_mapping = mav.mode_mapping() or {}
        guided_mode = mode_mapping.get('GUIDED')
        if guided_mode is None:
            return False
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode
        )
        return True
    except Exception:
        return False


def send_guided_goto(lat, lon, alt=None):
    """Send a MAVLink guided goto command to a global lat/lon target."""
    try:
        lat = float(lat)
        lon = float(lon)
    except Exception:
        return False, 'Invalid coordinates'

    if not (GUIDED_GOTO_LAT_MIN <= lat <= GUIDED_GOTO_LAT_MAX):
        return False, 'Latitude out of range'
    if not (GUIDED_GOTO_LON_MIN <= lon <= GUIDED_GOTO_LON_MAX):
        return False, 'Longitude out of range'

    try:
        alt_val = GUIDED_GOTO_DEFAULT_ALT_M if alt is None else float(alt)
    except Exception:
        return False, 'Invalid altitude'
    alt_val = max(GUIDED_GOTO_MIN_ALT_M, min(GUIDED_GOTO_MAX_ALT_M, alt_val))

    mode_ok = set_guided_mode()

    try:
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0
        )
    except Exception:
        pass

    try:
        mav.mav.set_position_target_global_int_send(
            0,
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            GUIDED_GOTO_TYPE_MASK,
            int(lat * 1e7),
            int(lon * 1e7),
            alt_val,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        )
        print(
            "[GUIDED] Sent MAVLink SET_POSITION_TARGET_GLOBAL_INT "
            f"via {MAVLINK_CONNECTION_STRING} -> lat={lat:.6f}, lon={lon:.6f}, alt={alt_val:.1f}m"
        )
    except Exception as e:
        return False, f'Failed to send position target: {e}'

    if not mode_ok:
        return True, 'Guided goto sent (mode switch unconfirmed)'
    return True, 'Guided goto sent'


# ===== Video Streaming =====
async def stream_cam0(ws):
    print("Client connected to video stream (cam0).")
    if cam0 is None:
        print("cam0 not available; closing connection")
        try:
            await ws.send(json.dumps({'type': 'error', 'message': 'cam0 not available'}))
        except Exception:
            pass
        await ws.close()
        return

    while True:
        frame = cam0.capture_array()
        await ws.send(encode_frame_with_timestamp(frame))
        await asyncio.sleep(1.0 / VIDEO_FPS)


async def stream_cam1(ws):
    print("Client connected to video stream (cam1).")
    if cam1 is None:
        print("cam1 not available; closing connection")
        try:
            await ws.send(json.dumps({'type': 'error', 'message': 'cam1 not available'}))
        except Exception:
            pass
        await ws.close()
        return

    while True:
        frame = cam1.capture_array()
        # If the camera is returning YUV420 (I420) as a single-channel
        # vertically-stacked array, the shape will often be (H * 3/2, W).
        # In that case we only want the Y plane (top H rows) to produce
        # a final 640x480 grayscale feed — crop off the U/V planes.
        if isinstance(frame, np.ndarray) and frame.ndim == 2 and frame.shape[0] > VIDEO_HEIGHT:
            try:
                frame = frame[:VIDEO_HEIGHT, :VIDEO_WIDTH]
            except Exception:
                # Fallback: if slicing fails for any reason, ensure we at least
                # reshape or take top region to VIDEO_HEIGHT x VIDEO_WIDTH
                frame = frame[0:VIDEO_HEIGHT, 0:VIDEO_WIDTH]

        await ws.send(encode_frame_with_timestamp(frame))
        await asyncio.sleep(1.0 / VIDEO_FPS)


async def stream_hq(ws):
    print("Client connected to video stream (hq).")
    if hq_capture is None or not hq_capture.isOpened():
        print("hq camera not available; closing connection")
        try:
            await ws.send(json.dumps({'type': 'error', 'message': 'hq camera not available'}))
        except Exception:
            pass
        await ws.close()
        return

    ensure_hq_thread_started()

    while True:
        with hq_lock:
            frame = None if hq_latest is None else hq_latest.copy()

        if frame is None:
            await asyncio.sleep(1.0 / HQ_VIDEO_FPS)
            continue

        await ws.send(encode_frame_with_timestamp_quality(frame, HQ_JPEG_QUALITY))
        await asyncio.sleep(1.0 / HQ_VIDEO_FPS)


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


async def stream_fiducials(ws):
    """WebSocket handler for fiducial detection results."""
    fiducial_clients.add(ws)
    print(f"Fiducial client connected. Total: {len(fiducial_clients)}")
    try:
        if hq_capture is None or not hq_capture.isOpened():
            try:
                await ws.send(json.dumps({'type': 'error', 'message': 'hq camera not available'}))
            except Exception:
                pass
            await ws.close()
            return
        ensure_hq_thread_started()
        while True:
            await asyncio.sleep(1.0 / FIDUCIAL_SEND_RATE_HZ)
            with hq_fiducial_lock:
                payload = hq_fiducial_latest
            if not payload:
                continue
            try:
                await ws.send(json.dumps(payload))
            except websockets.exceptions.ConnectionClosed:
                break
    finally:
        fiducial_clients.discard(ws)
        print(f"Fiducial client disconnected. Total: {len(fiducial_clients)}")


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

            msg_type = msg.get('type')
            if msg_type == 'guided_goto':
                print(
                    "[GUIDED] Received guided_goto request from client: "
                    f"lat={msg.get('lat')}, lon={msg.get('lon')}, alt={msg.get('alt')}"
                )
                ok, message = send_guided_goto(
                    msg.get('lat'),
                    msg.get('lon'),
                    msg.get('alt')
                )
                try:
                    await ws.send(json.dumps({
                        'type': 'guided_goto_ack',
                        'success': bool(ok),
                        'message': message,
                        'lat': msg.get('lat'),
                        'lon': msg.get('lon')
                    }))
                except Exception:
                    pass
                continue

            if msg_type == 'arm':
                desired = bool(msg.get('armed'))
                ok, message = send_arm_command(desired)
                try:
                    await ws.send(json.dumps({
                        'type': 'arm_ack',
                        'success': bool(ok),
                        'armed': desired,
                        'message': message
                    }))
                except Exception:
                    pass
                continue

            if msg_type == 'mode':
                mode = msg.get('mode')
                ok, message = set_vehicle_mode(mode)
                try:
                    await ws.send(json.dumps({
                        'type': 'mode_ack',
                        'success': bool(ok),
                        'mode': str(mode).strip().upper() if mode else None,
                        'message': message
                    }))
                except Exception:
                    pass
                continue

            cmd_id = msg.get('id')
            if cmd_id not in command_state:
                continue

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
           websockets.serve(stream_hq, '0.0.0.0', VIDEO_PORT_HQ), \
           websockets.serve(stream_telemetry, '0.0.0.0', TELEMETRY_PORT), \
           websockets.serve(stream_fiducials, '0.0.0.0', FIDUCIAL_PORT), \
           websockets.serve(command_handler, '0.0.0.0', COMMAND_PORT):
        print(f"Server running:")
        print(f"  - Video cam0: ws://0.0.0.0:{VIDEO_PORT_1}")
        print(f"  - Video cam1: ws://0.0.0.0:{VIDEO_PORT_2}")
        print(f"  - Video hq:   ws://0.0.0.0:{VIDEO_PORT_HQ}")
        print(f"  - Telemetry:  ws://0.0.0.0:{TELEMETRY_PORT} ({TELEMETRY_HZ}Hz)")
        print(f"  - Fiducials:  ws://0.0.0.0:{FIDUCIAL_PORT} ({FIDUCIAL_SEND_RATE_HZ}Hz)")
        print(f"  - Commands:   ws://0.0.0.0:{COMMAND_PORT}")
        print(f"  - UDP legacy: broadcast:5000 (10Hz)")
        asyncio.create_task(mavlink_broadcast())
        asyncio.create_task(broadcast_telemetry())
        await asyncio.Future()


asyncio.run(main())