# Import libraries
import asyncio
import cv2
import json
import numpy as np
import secrets
import socket
import struct
import time
import websockets
from copy import deepcopy
from threading import Thread
from collections import deque
import statistics

from flask import Flask, Response, jsonify, render_template, request, make_response

from functions.attitude import draw_attitude_indicator
from functions.battery import draw_battery_widget
from functions.map import generate_map_html
from functions.compass import draw_compass
from functions.throttle import draw_throttle_widget

# ===== Configuration =====
STATUS_UPDATE_INTERVAL_MS = 2000
STATUS_TIMEOUT = 3.0
LATENCY_SAMPLE_SIZE = 200

# Toggle test mode to use local test telemetry/streams (set True to enable)
# When enabled: telemetry WS -> ws://localhost:8888, cam0 -> localhost:8886, cam1 -> localhost:8887
TEST_MODE = False

if TEST_MODE:
    SERVER_IP = 'localhost'
    CAM0_PORT = 8886
    CAM1_PORT = 8887
    TELEMETRY_PORT = 8888
    COMMAND_PORT = 8889
else:
    SERVER_IP = '100.112.223.17'
    CAM0_PORT = 8765
    CAM1_PORT = 8766
    TELEMETRY_PORT = 8764
    COMMAND_PORT = 8763
VALID_USERNAME = 'argus'
VALID_PASSWORD = 'sentry'

# Camera function tabs (shown on right side of camera cards)
# Each entry must match a command id in templates/index.html COMMAND_DEFS
DEFAULT_CAMERA_FUNCTION_TABS = {
    'cam0': ['go_dark', 'drop_gps_pin', 'emergency'],
    'cam1': ['go_dark', 'night_vision', 'loiter', 'emergency'],
    'hq': ['landing_mode', 'loiter', 'drop_gps_pin', 'emergency']
}

# Per-user UI settings defaults
DEFAULT_SETTINGS = {
    'test_mode': False,
    'latency_polling_rate_ms': 500,
    'status_update_interval_ms': 30000,
    'visible_tiles': {
        'cam0': True,
        'cam1': True,
        'hq': True,
        'map': True,
        'latency': True,
        'commands': True
    },
    'camera_function_tabs': DEFAULT_CAMERA_FUNCTION_TABS
}

ALLOWED_TILE_IDS = {'cam0', 'cam1', 'hq', 'map', 'latency', 'commands'}
ALLOWED_COMMAND_IDS = {
    'go_dark', 'night_vision', 'auto_rth', 'drop_gps_pin', 'emergency', 'loiter', 'landing_mode'
}

# ===== Global State =====
app = Flask(__name__, static_folder='templates', static_url_path='')

# Video frames (store dicts with timing info)
frame_cam0 = None
frame_cam1 = None

# Per-camera latency sample buffers (store tuples of (network_ms, render_ms))
cam0_latency_samples = deque(maxlen=LATENCY_SAMPLE_SIZE)
cam1_latency_samples = deque(maxlen=LATENCY_SAMPLE_SIZE)

# Connection timestamps
last_cam0_time = 0
last_cam1_time = 0
last_mav_time = 0

# Video latency tracking
video_latency_cam0_ms = 0  # Network latency (server to client receive)
video_latency_cam1_ms = 0
cam0_frame_timestamp = 0
cam1_frame_timestamp = 0
cam0_processing_latency_ms = 0  # Backend processing time (overlays + JPEG encode)
cam1_processing_latency_ms = 0

# Connection status
cam0_online = False
cam1_online = False
mav_online = False
hq_online = False

# Telemetry data
mavlink_data = {
    "roll": 0, "pitch": 0, "yaw": 0,
    "lat": 0, "lon": 0, "alt": 0,
    "battery": 0, "battery_remaining": 0,
    "ground_speed": 0, "throttle": 0,
    "timestamp": 0, "server_latency_ms": 0
}
telemetry_latency_ms = 0

# Command state (mirrors server-side boolean flags)
command_state = {
    'go_dark': False,
    'night_vision': False,
    'auto_rth': False,
    'drop_gps_pin': False,
    'emergency': False,
    'loiter': False,
    'landing_mode': False,
}
command_ws = None  # Persistent WebSocket to server for commands

# Session + settings state
session_tokens = {}
settings_by_user = {}
settings_by_token = {}

# Legacy UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 5000))
sock.setblocking(False)


# ===== Helper Functions =====
def update_connection_status():
    """Update connection status based on last received data timestamps."""
    global cam0_online, cam1_online, mav_online
    current_time = time.time()
    cam0_online = last_cam0_time > 0 and (current_time - last_cam0_time) < STATUS_TIMEOUT
    cam1_online = last_cam1_time > 0 and (current_time - last_cam1_time) < STATUS_TIMEOUT
    mav_online = last_mav_time > 0 and (current_time - last_mav_time) < STATUS_TIMEOUT


def is_main_online():
    return cam0_online or cam1_online or mav_online


def is_map_online():
    return mav_online


def decode_video_frame(data):
    """Extract timestamp and decode JPEG from video data."""
    if len(data) > 8:
        timestamp = struct.unpack('d', data[:8])[0]
        jpeg_data = data[8:]
    else:
        timestamp = None
        jpeg_data = data
    frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
    return frame, timestamp


def draw_telemetry_text(frame, data):
    """Draw telemetry overlay text on frame."""
    font = cv2.FONT_HERSHEY_DUPLEX
    color = (255, 255, 255)
    scale, thickness = 0.4, 1
    x_margin = 10
    
    # Use safe getters and coerce to numeric where appropriate so malformed
    # telemetry won't crash overlay rendering.
    def fnum(key, fmt, default=0):
        return fmt.format(float(data.get(key, default)))

    lines = [
        fnum('roll', 'Roll: {:.1f}'),
        fnum('pitch', 'Pitch: {:.1f}'),
        fnum('yaw', 'Yaw: {:.1f}'),
        fnum('lat', 'Lat: {:.6f}', default=0.0),
        fnum('lon', 'Lon: {:.6f}', default=0.0),
        fnum('alt', 'Alt: {:.1f}m', default=0.0),
        (f"Battery: {float(data.get('battery', 0)):.2f}V ({int(data.get('battery_remaining', 0))}%)"
         if data.get('battery') is not None else "Battery: N/A"),
        fnum('ground_speed', 'GS: {:.1f}m/s', default=0.0),
        fnum('throttle', 'Throttle: {:.0f}%', default=0),
    ]
    
    frame_width = frame.shape[1]
    for i, text in enumerate(lines):
        text_width = cv2.getTextSize(text, font, scale, thickness)[0][0]
        y = 15 + i * 20
        cv2.putText(frame, text, (frame_width - text_width - x_margin, y), font, scale, color, thickness)


def get_request_token():
    header_token = request.headers.get('X-Session-Token')
    cookie_token = request.cookies.get('sentry_token')
    return header_token or cookie_token


def get_settings_for_request():
    token = get_request_token()
    if token and token in session_tokens:
        username = session_tokens[token]
        if username in settings_by_user:
            return deepcopy(settings_by_user[username])
    if token and token in settings_by_token:
        return deepcopy(settings_by_token[token])
    return deepcopy(DEFAULT_SETTINGS)


def save_settings_for_request(settings):
    token = get_request_token()
    if token and token in session_tokens:
        username = session_tokens[token]
        settings_by_user[username] = settings
        return
    if token:
        settings_by_token[token] = settings


def sanitize_settings(payload):
    settings = deepcopy(DEFAULT_SETTINGS)

    if isinstance(payload, dict):
        test_mode = payload.get('test_mode')
        if isinstance(test_mode, bool):
            settings['test_mode'] = test_mode

        latency_rate = payload.get('latency_polling_rate_ms')
        if isinstance(latency_rate, (int, float)):
            settings['latency_polling_rate_ms'] = int(max(200, min(5000, latency_rate)))

        status_rate = payload.get('status_update_interval_ms')
        if isinstance(status_rate, (int, float)):
            settings['status_update_interval_ms'] = int(max(2000, min(120000, status_rate)))

        visible_tiles = payload.get('visible_tiles')
        if isinstance(visible_tiles, dict):
            settings['visible_tiles'] = {
                tile_id: bool(visible_tiles.get(tile_id, True))
                for tile_id in ALLOWED_TILE_IDS
            }

        camera_tabs = payload.get('camera_function_tabs')
        if isinstance(camera_tabs, dict):
            cleaned_tabs = {}
            for cam_id, command_ids in camera_tabs.items():
                if cam_id not in DEFAULT_CAMERA_FUNCTION_TABS:
                    continue
                if not isinstance(command_ids, list):
                    continue
                cleaned = [cmd_id for cmd_id in command_ids if cmd_id in ALLOWED_COMMAND_IDS]
                cleaned_tabs[cam_id] = cleaned
            if cleaned_tabs:
                settings['camera_function_tabs'] = cleaned_tabs

    return settings


# ===== Video Receivers =====
async def receive_video(cam_id, port):
    """Generic video receiver for a camera."""
    global frame_cam0, frame_cam1, last_cam0_time, last_cam1_time
    global video_latency_cam0_ms, video_latency_cam1_ms, cam0_frame_timestamp, cam1_frame_timestamp
    
    while True:
        try:
            async with websockets.connect(f'ws://{SERVER_IP}:{port}') as ws:
                print(f"Connected to video server (cam{cam_id}).")
                while True:
                    data = await ws.recv()
                    receive_time = time.time()
                    frame, timestamp = decode_video_frame(data)
                    
                    # Store frame along with timing info so we can compute network + render latency later
                    if cam_id == 0:
                        network_ms = (receive_time - timestamp) * 1000 if timestamp else None
                        frame_cam0 = {
                            'frame': frame,
                            'server_ts': timestamp,
                            'receive_time': receive_time,
                            'network_ms': network_ms
                        }
                        last_cam0_time = receive_time
                        if timestamp:
                            cam0_frame_timestamp = timestamp
                    else:
                        network_ms = (receive_time - timestamp) * 1000 if timestamp else None
                        frame_cam1 = {
                            'frame': frame,
                            'server_ts': timestamp,
                            'receive_time': receive_time,
                            'network_ms': network_ms
                        }
                        last_cam1_time = receive_time
                        if timestamp:
                            cam1_frame_timestamp = timestamp
        except websockets.exceptions.ConnectionClosed:
            print(f"Camera {cam_id} WebSocket disconnected, reconnecting in 2 seconds...")
        except Exception as e:
            print(f"Camera {cam_id} error: {e}, reconnecting in 2 seconds...")
        await asyncio.sleep(2)


async def receive_telemetry():
    """Connect to telemetry WebSocket and update mavlink_data in real-time."""
    global mavlink_data, telemetry_latency_ms, last_mav_time
    
    while True:
        try:
            async with websockets.connect(f'ws://{SERVER_IP}:{TELEMETRY_PORT}') as ws:
                print("Connected to telemetry server (WebSocket).")
                while True:
                    data = await ws.recv()
                    try:
                        telemetry_msg = json.loads(data)
                        current_time = time.time()
                        # server may send either 'server_time' or 'timestamp'
                        server_time = telemetry_msg.get('server_time', telemetry_msg.get('timestamp', current_time))
                        try:
                            telemetry_latency_ms = (current_time - float(server_time)) * 1000
                        except Exception:
                            telemetry_latency_ms = 0

                        # Merge incoming telemetry into the existing dict instead
                        # of replacing the object. This preserves any code that
                        # holds references to `mavlink_data` and avoids races.
                        if isinstance(telemetry_msg, dict):
                            mavlink_data.update(telemetry_msg)
                        last_mav_time = current_time
                    except json.JSONDecodeError:
                        print(f"Failed to parse telemetry JSON: {data}")
        except websockets.exceptions.ConnectionRefused:
            print("Telemetry server not available, retrying in 2 seconds...")
        except Exception as e:
            print(f"Telemetry WebSocket error: {e}")
        await asyncio.sleep(2)


def receive_mavlink():
    """Legacy UDP receiver for backward compatibility."""
    global mavlink_data, last_mav_time
    
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            values = data.decode().split(',')
            mavlink_data = {
                "roll": float(values[0]), "pitch": float(values[1]), "yaw": float(values[2]),
                "lat": float(values[3]), "lon": float(values[4]), "alt": float(values[5]),
                "battery": float(values[6]), "battery_remaining": float(values[7]),
                "ground_speed": float(values[8]), "throttle": float(values[9]),
                "timestamp": time.time(), "server_latency_ms": 0
            }
            last_mav_time = time.time()
        except:
            pass


def status_monitor():
    """Background thread to update connection status."""
    while True:
        update_connection_status()
        time.sleep(1)


# ===== Frame Generators =====
def gen_frames_cam0():
    global video_latency_cam0_ms, cam0_processing_latency_ms, cam0_frame_timestamp
    while True:
        if frame_cam0 is None:
            continue
        entry = frame_cam0
        f = entry['frame'].copy()
        h, w = f.shape[:2]
        
        # Start timing backend processing
        processing_start = time.time()
        
        # Draw overlays
        try:
            draw_compass(f, mavlink_data.get('yaw', 0), 0, h - 130, 120)
            draw_attitude_indicator(f, mavlink_data.get('roll', 0), mavlink_data.get('pitch', 0), x=w - 130, y=h - 130, size=120)
            draw_battery_widget(f, mavlink_data.get('battery_remaining', 0), position=(10, 10), width=60, height=20)
            draw_throttle_widget(f, mavlink_data.get('throttle', 0), position=(10, 40), width=60, height=20)
            draw_telemetry_text(f, mavlink_data)
        except Exception as e:
            # Don't let telemetry/overlay errors break the video stream; log and continue
            print(f"Overlay drawing error (cam0): {e}")
        
        # Encode to JPEG
        _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        
        # Compute backend processing latency (overlays + encoding)
        processing_time = time.time()
        processing_ms = (processing_time - processing_start) * 1000
        network_ms = entry.get('network_ms')
        
        # Record samples: (network_latency, backend_processing_latency)
        cam0_latency_samples.append((network_ms, processing_ms))

        # Update median stats for network latency
        nets = [n for n, r in cam0_latency_samples if n is not None]
        procs = [r for n, r in cam0_latency_samples if r is not None]
        if procs:
            cam0_processing_latency_ms = statistics.median(procs)
        else:
            cam0_processing_latency_ms = 0
        if nets:
            video_latency_cam0_ms = statistics.median(nets)
        else:
            video_latency_cam0_ms = 0

        # Expose latest frame timestamp
        try:
            cam0_frame_timestamp = entry.get('server_ts') or cam0_frame_timestamp
        except NameError:
            cam0_frame_timestamp = entry.get('server_ts')

        # Store latest JPEG bytes and per-frame metadata for one-shot snapshot endpoint
        try:
            entry['last_jpeg'] = jpeg.tobytes()
            entry['last_processing_ms'] = processing_ms
            entry['last_server_ts'] = entry.get('server_ts')
            # preserve the network latency computed when frame was received via websocket
            entry['last_network_ms'] = entry.get('network_ms')
        except Exception:
            pass

        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + entry['last_jpeg'] + b'\r\n'


def gen_frames_cam1():
    global video_latency_cam1_ms, cam1_processing_latency_ms, cam1_frame_timestamp
    while True:
        if frame_cam1 is None:
            continue
        entry = frame_cam1
        f = entry['frame'].copy()

        # Start timing backend processing
        processing_start = time.time()
        
        # Encode to JPEG (no overlays for cam1)
        _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        
        # Compute backend processing latency
        processing_time = time.time()
        processing_ms = (processing_time - processing_start) * 1000
        network_ms = entry.get('network_ms')
        
        # Record samples
        cam1_latency_samples.append((network_ms, processing_ms))

        # Update median stats
        nets = [n for n, r in cam1_latency_samples if n is not None]
        procs = [r for n, r in cam1_latency_samples if r is not None]
        if procs:
            cam1_processing_latency_ms = statistics.median(procs)
        else:
            cam1_processing_latency_ms = 0
        if nets:
            video_latency_cam1_ms = statistics.median(nets)
        else:
            video_latency_cam1_ms = 0

        try:
            cam1_frame_timestamp = entry.get('server_ts') or cam1_frame_timestamp
        except NameError:
            cam1_frame_timestamp = entry.get('server_ts')

        # Store latest JPEG bytes and per-frame metadata for one-shot snapshot endpoint
        try:
            entry['last_jpeg'] = jpeg.tobytes()
            entry['last_processing_ms'] = processing_ms
            entry['last_server_ts'] = entry.get('server_ts')
            entry['last_network_ms'] = entry.get('network_ms')
        except Exception:
            pass

        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + entry['last_jpeg'] + b'\r\n'


# ===== Flask Routes =====
@app.route('/')
def index():
    update_connection_status()
    user_settings = get_settings_for_request()
    return render_template(
        'index.html',
        main_online=is_main_online(),
        cam0_online=cam0_online,
        cam1_online=cam1_online,
        hq_online=hq_online,
        map_online=is_map_online(),
        map_html=generate_map_html(
            mavlink_data['lat'],
            mavlink_data['lon'],
            mavlink_data['yaw'],
            test_mode=bool(user_settings.get('test_mode'))
        ),
        status_update_interval_ms=user_settings.get('status_update_interval_ms', STATUS_UPDATE_INTERVAL_MS),
        camera_function_tabs=user_settings.get('camera_function_tabs', DEFAULT_CAMERA_FUNCTION_TABS),
        user_settings=user_settings
    )


@app.route('/video_feed_cam0')
def video_feed_cam0():
    return Response(gen_frames_cam0(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed_cam1')
def video_feed_cam1():
    return Response(gen_frames_cam1(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/telemetry')
def telemetry():
    update_connection_status()
    result = mavlink_data.copy()
    result['client_latency_ms'] = telemetry_latency_ms
    return jsonify(result)


@app.route('/api/status')
def api_status():
    update_connection_status()
    return jsonify({
        'main_online': is_main_online(),
        'cam0_online': cam0_online,
        'cam1_online': cam1_online,
        'mav_online': mav_online,
        'map_online': is_map_online(),
        'hq_online': hq_online
    })


@app.route('/api/video_latency')
def api_video_latency():
    """Return video latency metrics.
    
    Returns:
        - network_latency_ms: Time from server capture to client receipt
        - processing_latency_ms: Gateway processing (overlays + JPEG encoding)
        - render_latency_ms: Frontend rendering (measured by browser)
        - Total latency = network + processing + render
    """
    return jsonify({
        'cam0_network_latency_ms': round(video_latency_cam0_ms, 2),
        'cam0_processing_latency_ms': round(cam0_processing_latency_ms, 2),
        'cam0_render_latency_ms': 0,  # Placeholder - measured by frontend
        'cam1_network_latency_ms': round(video_latency_cam1_ms, 2),
        'cam1_processing_latency_ms': round(cam1_processing_latency_ms, 2),
        'cam1_render_latency_ms': 0,  # Placeholder - measured by frontend
        'cam0_frame_timestamp': cam0_frame_timestamp,
        'cam1_frame_timestamp': cam1_frame_timestamp,
        'cam0_samples': len(cam0_latency_samples),
        'cam1_samples': len(cam1_latency_samples),
        'cam0_online': cam0_online,
        'cam1_online': cam1_online,
        'server_time': time.time()
    })


@app.route('/api/authenticate', methods=['POST'])
def authenticate():
    data = request.json
    username = data.get('username', '').strip()
    password = data.get('password', '').strip()
    
    if username == VALID_USERNAME and password == VALID_PASSWORD:
        token = secrets.token_hex(32)
        session_tokens[token] = username
        response = make_response(jsonify({'success': True, 'token': token}))
        response.set_cookie(
            'sentry_token',
            token,
            httponly=True,
            samesite='Lax'
        )
        return response
    return jsonify({'success': False, 'message': 'Invalid credentials'}), 401


@app.route('/api/logout', methods=['POST'])
def logout():
    token = get_request_token()
    if token and token in session_tokens:
        session_tokens.pop(token, None)
    response = make_response(jsonify({'success': True}))
    response.set_cookie('sentry_token', '', expires=0)
    return response


@app.route('/api/settings', methods=['GET', 'POST'])
def api_settings():
    if request.method == 'GET':
        return jsonify(get_settings_for_request())

    payload = request.json or {}
    settings = sanitize_settings(payload)
    save_settings_for_request(settings)
    return jsonify({'success': True, 'settings': settings})


async def receive_commands():
    """Connect to the server's command WebSocket and keep command_state in sync."""
    global command_state, command_ws

    while True:
        try:
            async with websockets.connect(f'ws://{SERVER_IP}:{COMMAND_PORT}') as ws:
                command_ws = ws
                print("Connected to command WebSocket.")
                async for message in ws:
                    try:
                        msg = json.loads(message)
                        if msg.get('type') == 'state' and 'commands' in msg:
                            command_state.update(msg['commands'])
                    except json.JSONDecodeError:
                        pass
        except websockets.exceptions.ConnectionRefused:
            print("Command server not available, retrying in 2 seconds...")
        except Exception as e:
            print(f"Command WebSocket error: {e}")
        finally:
            command_ws = None
        await asyncio.sleep(2)


@app.route('/api/commands')
def api_commands():
    """Return current command states."""
    return jsonify(command_state)


@app.route('/api/command', methods=['POST'])
def api_command_toggle():
    """Toggle a single command and forward to server via WebSocket."""
    data = request.json
    cmd_id = data.get('id')
    value = data.get('value')
    is_pulse = data.get('pulse', False)
    
    if is_pulse:
        print(f"[Flask] Received pulse command: '{cmd_id}'")
    else:
        print(f"[Flask] Received command toggle: '{cmd_id}' -> {value}")
    
    if cmd_id not in command_state:
        return jsonify({'success': False, 'message': 'Unknown command'}), 400
    
    # For pulse commands, don't update local state
    if not is_pulse:
        command_state[cmd_id] = bool(value)
    
    # Forward to server via the persistent WebSocket
    if command_ws:
        try:
            msg = {'type': 'pulse' if is_pulse else 'toggle', 'id': cmd_id, 'value': bool(value)}
            print(f"[Flask] Forwarding to server: {msg}")
            asyncio.run_coroutine_threadsafe(
                command_ws.send(json.dumps(msg)),
                command_loop
            )
        except Exception as e:
            print(f"Failed to forward command to server: {e}")
    else:
        print("[Flask] Warning: command_ws is None, cannot forward to server")
    return jsonify({'success': True, 'commands': command_state})


@app.route('/api/snapshot_cam0')
def api_snapshot_cam0():
    """Return the latest JPEG snapshot for cam0 (one-shot)."""
    if frame_cam0 and isinstance(frame_cam0, dict) and frame_cam0.get('last_jpeg'):
        headers = {}
        # include per-frame metadata if available
        try:
            if frame_cam0.get('last_server_ts'):
                headers['X-Frame-Ts'] = str(frame_cam0.get('last_server_ts'))
            if frame_cam0.get('last_processing_ms') is not None:
                headers['X-Processing-Ms'] = str(frame_cam0.get('last_processing_ms'))
            if frame_cam0.get('last_network_ms') is not None:
                headers['X-Network-Ms'] = str(frame_cam0.get('last_network_ms'))
        except Exception:
            pass
        return Response(frame_cam0['last_jpeg'], mimetype='image/jpeg', headers=headers)
    return ('', 204)


@app.route('/api/snapshot_cam1')
def api_snapshot_cam1():
    """Return the latest JPEG snapshot for cam1 (one-shot)."""
    if frame_cam1 and isinstance(frame_cam1, dict) and frame_cam1.get('last_jpeg'):
        headers = {}
        try:
            if frame_cam1.get('last_server_ts'):
                headers['X-Frame-Ts'] = str(frame_cam1.get('last_server_ts'))
            if frame_cam1.get('last_processing_ms') is not None:
                headers['X-Processing-Ms'] = str(frame_cam1.get('last_processing_ms'))
            if frame_cam1.get('last_network_ms') is not None:
                headers['X-Network-Ms'] = str(frame_cam1.get('last_network_ms'))
        except Exception:
            pass
        return Response(frame_cam1['last_jpeg'], mimetype='image/jpeg', headers=headers)
    return ('', 204)


@app.route('/map_embed')
def map_embed():
    """Return a full standalone map HTML page for embedding in an iframe."""
    user_settings = get_settings_for_request()
    html = generate_map_html(
        mavlink_data['lat'],
        mavlink_data['lon'],
        mavlink_data['yaw'],
        test_mode=bool(user_settings.get('test_mode'))
    )
    return Response(html, mimetype='text/html')


# ===== Main =====
if __name__ == '__main__':
    # Create a dedicated event loop for the command WebSocket
    import threading
    command_loop = asyncio.new_event_loop()
    def run_command_loop():
        asyncio.set_event_loop(command_loop)
        command_loop.run_until_complete(receive_commands())
    Thread(target=run_command_loop, daemon=True).start()

    Thread(target=lambda: asyncio.run(receive_video(0, CAM0_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_video(1, CAM1_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_telemetry()), daemon=True).start()
    Thread(target=receive_mavlink, daemon=True).start()
    Thread(target=status_monitor, daemon=True).start()
    app.run(port=8000, threaded=True)
