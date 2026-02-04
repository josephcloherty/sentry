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
from functools import lru_cache
from pathlib import Path
from threading import Thread

import folium
import geopandas as gpd
from flask import Flask, Response, jsonify, render_template, request

from attitude import draw_attitude_indicator
from battery import draw_battery_widget
from compass import draw_compass

# ===== Configuration =====
STATUS_UPDATE_INTERVAL_MS = 500
STATUS_TIMEOUT = 3.0
SERVER_IP = '100.112.223.17'
CAM0_PORT = 8765
CAM1_PORT = 8766
TELEMETRY_PORT = 8764
VALID_USERNAME = 'argus'
VALID_PASSWORD = 'sentry'

# ===== Global State =====
app = Flask(__name__, static_folder='templates', static_url_path='')

# Video frames
frame_cam0 = None
frame_cam1 = None

# Connection timestamps
last_cam0_time = 0
last_cam1_time = 0
last_mav_time = 0

# Video latency tracking
video_latency_cam0_ms = 0
video_latency_cam1_ms = 0
cam0_frame_timestamp = 0
cam1_frame_timestamp = 0

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


@lru_cache(maxsize=1)
def load_geodata():
    gpkg_path = Path(__file__).parent / 'dev' / 'map' / 'NorthWest_Railways.gpkg'
    if not gpkg_path.exists():
        return None, None
    gdf = gpd.read_file(gpkg_path)
    gdf['geometry'] = gdf['geometry'].simplify(tolerance=0.001, preserve_topology=True)
    if gdf.crs and gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)
    return gdf.__geo_interface__, gdf.total_bounds


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
                    
                    if cam_id == 0:
                        frame_cam0 = frame
                        last_cam0_time = receive_time
                        if timestamp:
                            cam0_frame_timestamp = timestamp
                            video_latency_cam0_ms = (receive_time - timestamp) * 1000
                    else:
                        frame_cam1 = frame
                        last_cam1_time = receive_time
                        if timestamp:
                            cam1_frame_timestamp = timestamp
                            video_latency_cam1_ms = (receive_time - timestamp) * 1000
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
    while True:
        if frame_cam0 is None:
            continue
        f = frame_cam0.copy()
        h, w = f.shape[:2]
        
        draw_compass(f, mavlink_data['yaw'], 0, h - 130, 120)
        draw_attitude_indicator(f, mavlink_data['roll'], mavlink_data['pitch'], x=w - 130, y=h - 130, size=120)
        draw_battery_widget(f, mavlink_data['battery_remaining'], position=(10, 10), width=60, height=20)
        draw_telemetry_text(f, mavlink_data)
        
        _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'


def gen_frames_cam1():
    while True:
        if frame_cam1 is None:
            continue
        _, jpeg = cv2.imencode('.jpg', frame_cam1.copy(), [cv2.IMWRITE_JPEG_QUALITY, 90])
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'


# ===== Map Generation =====
def generate_map_html():
    """Generate map HTML with current telemetry data."""
    geojson_data, bounds = load_geodata()
    lat = mavlink_data['lat'] if mavlink_data['lat'] != 0 else 53.406049
    lon = mavlink_data['lon'] if mavlink_data['lon'] != 0 else -2.968585
    yaw = mavlink_data['yaw']
    location = [lat, lon]
    
    m = folium.Map(
        location=location,
        zoom_start=20,
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri'
    )
    
    if geojson_data:
        folium.GeoJson(geojson_data, style_function=lambda x: {'color': '#0066cc', 'weight': 2}).add_to(m)

    if bounds is not None:
        m.fit_bounds([[bounds[1], bounds[0]], [bounds[3], bounds[2]]])
    
    # Add sentry icon marker
    icon_path = Path(__file__).parent / 'templates' / 'images' / 'sentry_icon_white.png'
    if icon_path.exists():
        icon_html = f'''
        <div class="sentry-marker-icon" style="width: 50px; height: 50px; transform: translate(-50%, -50%) rotate({yaw}deg);">
            <img src="/images/sentry_icon_white.png" style="width: 50px; height: 50px; display: block;" />
        </div>'''
        folium.Marker(location=location, icon=folium.DivIcon(html=icon_html, class_name='sentry-marker-container')).add_to(m)
    
    # Add map controls script
    map_id = m.get_name()
    bounds_js = f"[[{bounds[1]}, {bounds[0]}], [{bounds[3]}, {bounds[2]}]]" if bounds is not None else "null"
    
    control_script = f"""
    <div id="reset-btn" style="position: fixed; top: 15px; right: 15px; z-index: 1000; display: none;">
        <button onclick="resetView()" style="padding: 10px 15px; background: white; border: 2px solid rgba(0,0,0,0.2); border-radius: 4px; cursor: pointer; font-weight: bold; box-shadow: 0 1px 5px rgba(0,0,0,0.4);">Reset View</button>
    </div>
    <script>
        var __defaultCenter = [{location[0]}, {location[1]}];
        var __defaultZoom = 20;
        var __bounds = {bounds_js};
        var __sentryMarker = null;
        var __mapObj = null;

        document.addEventListener('DOMContentLoaded', function() {{
            __mapObj = window['{map_id}'];
            if (__mapObj) {{
                __mapObj.eachLayer(function(layer) {{
                    if (layer instanceof L.Marker) __sentryMarker = layer;
                }});
            }}
        }});

        window.updateSentryMarker = function(lat, lon, yaw) {{
            if (__sentryMarker && __mapObj) {{
                __sentryMarker.setLatLng([lat, lon]);
                var iconDiv = document.querySelector('.sentry-marker-icon');
                if (iconDiv) iconDiv.style.transform = 'translate(-50%, -50%) rotate(' + yaw + 'deg)';
            }}
        }};

        window.resetView = function() {{
            var mapObj = window['{map_id}'];
            if (mapObj) {{ mapObj.setView(__defaultCenter, __defaultZoom); mapObj.invalidateSize(); }}
        }};

        window.fitBoundsView = function() {{
            var mapObj = window['{map_id}'];
            if (mapObj) {{
                if (__bounds) mapObj.fitBounds(__bounds);
                else mapObj.setView(__defaultCenter, __defaultZoom);
                mapObj.invalidateSize();
            }}
        }};
    </script>"""
    m.get_root().html.add_child(folium.Element(control_script))
    
    return m._repr_html_()


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
        map_html=generate_map_html(),
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
        'cam1_network_latency_ms': round(video_latency_cam1_ms, 2),
        'cam0_frame_timestamp': cam0_frame_timestamp,
        'cam1_frame_timestamp': cam1_frame_timestamp,
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
