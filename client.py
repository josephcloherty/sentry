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
from threading import Thread
from collections import deque
import statistics

from flask import Flask, Response, jsonify, render_template, request

from functions.attitude import draw_attitude_indicator
from functions.battery import draw_battery_widget
from functions.map import generate_map_html
from functions.compass import draw_compass

# ===== Configuration =====
STATUS_UPDATE_INTERVAL_MS = 2000
STATUS_TIMEOUT = 3.0

# Toggle test mode to use local test telemetry/streams (set True to enable)
# When enabled: telemetry WS -> ws://localhost:8888, cam0 -> localhost:8886, cam1 -> localhost:8887
TEST_MODE = True

if TEST_MODE:
    SERVER_IP = 'localhost'
    CAM0_PORT = 8886
    CAM1_PORT = 8887
    TELEMETRY_PORT = 8888
else:
    SERVER_IP = '100.112.223.17'
    CAM0_PORT = 8765
    CAM1_PORT = 8766
    TELEMETRY_PORT = 8764
VALID_USERNAME = 'argus'
VALID_PASSWORD = 'sentry'

# ===== Global State =====
app = Flask(__name__, static_folder='templates', static_url_path='')

# Video frames (store dicts with timing info)
frame_cam0 = None
frame_cam1 = None

# Per-camera latency sample buffers (store tuples of (network_ms, render_ms))
cam0_latency_samples = deque(maxlen=200)
cam1_latency_samples = deque(maxlen=200)

# Connection timestamps
last_cam0_time = 0
last_cam1_time = 0
last_mav_time = 0

# Video latency tracking
video_latency_cam0_ms = 0
video_latency_cam1_ms = 0
cam0_frame_timestamp = 0
cam1_frame_timestamp = 0
cam0_render_latency_ms = 0
cam1_render_latency_ms = 0

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
    
    lines = [
        f"Roll: {data['roll']:.1f}",
        f"Pitch: {data['pitch']:.1f}",
        f"Yaw: {data['yaw']:.1f}",
        f"Lat: {data['lat']:.6f}",
        f"Lon: {data['lon']:.6f}",
        f"Alt: {data['alt']:.1f}m",
        f"Battery: {data['battery']:.2f}V ({data['battery_remaining']}%)",
        f"GS: {data['ground_speed']:.1f}m/s",
        f"Throttle: {data['throttle']}%",
    ]
    
    frame_width = frame.shape[1]
    for i, text in enumerate(lines):
        text_width = cv2.getTextSize(text, font, scale, thickness)[0][0]
        y = 15 + i * 20
        cv2.putText(frame, text, (frame_width - text_width - x_margin, y), font, scale, color, thickness)


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
                        server_time = telemetry_msg.get('timestamp', current_time)
                        telemetry_latency_ms = (current_time - server_time) * 1000
                        mavlink_data = telemetry_msg
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
    global video_latency_cam0_ms, cam0_render_latency_ms, cam0_frame_timestamp
    while True:
        if frame_cam0 is None:
            continue
        entry = frame_cam0
        f = entry['frame'].copy()
        h, w = f.shape[:2]
        
        draw_compass(f, mavlink_data['yaw'], 0, h - 130, 120)
        draw_attitude_indicator(f, mavlink_data['roll'], mavlink_data['pitch'], x=w - 130, y=h - 130, size=120)
        draw_battery_widget(f, mavlink_data['battery_remaining'], position=(10, 10), width=60, height=20)
        draw_telemetry_text(f, mavlink_data)
        
        # Compute render latency (time from receive to actual render) and record samples
        render_time = time.time()
        render_ms = (render_time - entry['receive_time']) * 1000 if entry.get('receive_time') else None
        network_ms = entry.get('network_ms')
        cam0_latency_samples.append((network_ms, render_ms))

        # Update median stats
        nets = [n for n, r in cam0_latency_samples if n is not None]
        renders = [r for n, r in cam0_latency_samples if r is not None]
        if renders:
            cam0_render_latency_ms = statistics.median(renders)
        else:
            cam0_render_latency_ms = 0
        if nets:
            video_latency_cam0_ms = statistics.median(nets)
        else:
            video_latency_cam0_ms = cam0_render_latency_ms

        # expose latest frame timestamp
        try:
            cam0_frame_timestamp = entry.get('server_ts') or cam0_frame_timestamp
        except NameError:
            cam0_frame_timestamp = entry.get('server_ts')

        _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'


def gen_frames_cam1():
    global video_latency_cam1_ms, cam1_render_latency_ms, cam1_frame_timestamp
    while True:
        if frame_cam1 is None:
            continue
        entry = frame_cam1
        f = entry['frame'].copy()

        # Compute render latency and record
        render_time = time.time()
        render_ms = (render_time - entry['receive_time']) * 1000 if entry.get('receive_time') else None
        network_ms = entry.get('network_ms')
        cam1_latency_samples.append((network_ms, render_ms))

        nets = [n for n, r in cam1_latency_samples if n is not None]
        renders = [r for n, r in cam1_latency_samples if r is not None]
        if renders:
            cam1_render_latency_ms = statistics.median(renders)
        else:
            cam1_render_latency_ms = 0
        if nets:
            video_latency_cam1_ms = statistics.median(nets)
        else:
            video_latency_cam1_ms = cam1_render_latency_ms

        try:
            cam1_frame_timestamp = entry.get('server_ts') or cam1_frame_timestamp
        except NameError:
            cam1_frame_timestamp = entry.get('server_ts')

        _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'


# ===== Flask Routes =====
@app.route('/')
def index():
    update_connection_status()
    return render_template(
        'index.html',
        main_online=is_main_online(),
        cam0_online=cam0_online,
        cam1_online=cam1_online,
        hq_online=hq_online,
        map_online=is_map_online(),
        map_html=generate_map_html(mavlink_data['lat'], mavlink_data['lon'], mavlink_data['yaw'], test_mode=TEST_MODE),
        status_update_interval_ms=STATUS_UPDATE_INTERVAL_MS
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
    return jsonify({
        'cam0_network_latency_ms': round(video_latency_cam0_ms, 2),
        'cam0_render_latency_ms': round(cam0_render_latency_ms, 2),
        'cam1_network_latency_ms': round(video_latency_cam1_ms, 2),
        'cam1_render_latency_ms': round(cam1_render_latency_ms, 2),
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
        return jsonify({'success': True, 'token': secrets.token_hex(32)})
    return jsonify({'success': False, 'message': 'Invalid credentials'}), 401


@app.route('/api/logout', methods=['POST'])
def logout():
    return jsonify({'success': True})


# ===== Main =====
if __name__ == '__main__':
    Thread(target=lambda: asyncio.run(receive_video(0, CAM0_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_video(1, CAM1_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_telemetry()), daemon=True).start()
    Thread(target=receive_mavlink, daemon=True).start()
    Thread(target=status_monitor, daemon=True).start()
    app.run(port=8000, threaded=True)
