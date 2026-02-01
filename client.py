import asyncio, cv2, numpy as np, websockets
from flask import Flask, render_template, Response, jsonify, request
from threading import Thread
import socket
from compass import draw_compass
from attitude import draw_attitude_indicator
from battery import draw_battery_widget
from functools import lru_cache
from pathlib import Path
import geopandas as gpd
import folium
import secrets
import time
import json

# ===== Configuration Variables =====
STATUS_UPDATE_INTERVAL_MS = 2000   # How often to poll connection status (milliseconds)
STATUS_TIMEOUT = 3.0               # Seconds before marking a connection as offline

# Legacy UDP socket (for backward compatibility)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 5000))
sock.setblocking(False)

app = Flask(__name__, static_folder='templates', static_url_path='')
frame_cam0 = None
frame_cam1 = None

# Connection status tracking
last_cam0_time = 0
last_cam1_time = 0
last_mav_time = 0

# Status variables (updated automatically based on connection health)
cam0_online = False
cam1_online = False
mav_online = False
hq_online = False  # Not implemented yet

# Derived status
@property
def main_online():
    return cam0_online or cam1_online or mav_online

@property
def map_online():
    return mav_online  # Map requires MAVLink GPS data

mavlink_data = {"roll": 0, "pitch": 0, "yaw": 0, "lat": 0, "lon": 0, "alt": 0, "battery": 0, "battery_remaining": 0, "ground_speed": 0, "throttle": 0, "timestamp": 0, "server_latency_ms": 0}
telemetry_latency_ms = 0  # Track round-trip latency

def update_connection_status():
    """Update connection status based on last received data timestamps."""
    global cam0_online, cam1_online, mav_online
    current_time = time.time()
    cam0_online = (current_time - last_cam0_time) < STATUS_TIMEOUT if last_cam0_time > 0 else False
    cam1_online = (current_time - last_cam1_time) < STATUS_TIMEOUT if last_cam1_time > 0 else False
    mav_online = (current_time - last_mav_time) < STATUS_TIMEOUT if last_mav_time > 0 else False

def get_main_online():
    """Check if any system is online."""
    return cam0_online or cam1_online or mav_online

def get_map_online():
    """Map is online when MAVLink data is available."""
    return mav_online

@lru_cache(maxsize=1)
def load_geodata():
    gpkg_path = Path(__file__).parent / 'map' / 'NorthWest_Railways.gpkg'
    if gpkg_path.exists():
        gdf = gpd.read_file(gpkg_path)
        gdf['geometry'] = gdf['geometry'].simplify(tolerance=0.001, preserve_topology=True)
        if gdf.crs and gdf.crs.to_epsg() != 4326:
            gdf = gdf.to_crs(epsg=4326)
        return gdf.__geo_interface__, gdf.total_bounds
    return None, None

async def receive_video_cam0():
    global frame_cam0, last_cam0_time
    while True:
        try:
            async with websockets.connect('ws://100.112.223.17:8765') as ws:
                print("Connected to video server (cam0).")
                while True:
                    data = await ws.recv()
                    frame_cam0 = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    last_cam0_time = time.time()  # Update timestamp on successful receive
        except websockets.exceptions.ConnectionClosed:
            print("Camera 0 WebSocket disconnected, reconnecting in 2 seconds...")
            await asyncio.sleep(2)
        except Exception as e:
            print(f"Camera 0 error: {e}, reconnecting in 2 seconds...")
            await asyncio.sleep(2)

async def receive_video_cam1():
    global frame_cam1, last_cam1_time
    while True:
        try:
            async with websockets.connect('ws://100.112.223.17:8766') as ws:
                print("Connected to video server (cam1).")
                while True:
                    data = await ws.recv()
                    frame_cam1 = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    last_cam1_time = time.time()  # Update timestamp on successful receive
        except websockets.exceptions.ConnectionClosed:
            print("Camera 1 WebSocket disconnected, reconnecting in 2 seconds...")
            await asyncio.sleep(2)
        except Exception as e:
            print(f"Camera 1 error: {e}, reconnecting in 2 seconds...")
            await asyncio.sleep(2)

async def receive_telemetry():
    """
    Connect to telemetry WebSocket and update mavlink_data in real-time.
    This replaces the old UDP polling method.
    """
    global mavlink_data, telemetry_latency_ms, last_mav_time
    while True:
        try:
            async with websockets.connect('ws://100.112.223.17:8764') as ws:
                print("Connected to telemetry server (WebSocket).")
                while True:
                    data = await ws.recv()
                    try:
                        telemetry_msg = json.loads(data)
                        # Calculate round-trip latency
                        current_time = time.time()
                        server_time = telemetry_msg.get('timestamp', current_time)
                        telemetry_latency_ms = (current_time - server_time) * 1000
                        
                        mavlink_data = telemetry_msg
                        last_mav_time = time.time()  # Update timestamp on successful receive
                    except json.JSONDecodeError:
                        print(f"Failed to parse telemetry JSON: {data}")
        except websockets.exceptions.ConnectionRefused:
            print("Telemetry server not available, retrying in 2 seconds...")
            await asyncio.sleep(2)
        except Exception as e:
            print(f"Telemetry WebSocket error: {e}")
            await asyncio.sleep(2)

def receive_mavlink():
    """Legacy UDP receiver for backward compatibility"""
    global mavlink_data, last_mav_time
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            values = data.decode().split(',')
            mavlink_data = {
                "roll": float(values[0]),
                "pitch": float(values[1]),
                "yaw": float(values[2]),
                "lat": float(values[3]),
                "lon": float(values[4]),
                "alt": float(values[5]),
                "battery": float(values[6]),
                "battery_remaining": float(values[7]),
                "ground_speed": float(values[8]),
                "throttle": float(values[9]),
                "timestamp": time.time(),
                "server_latency_ms": 0
            }
            last_mav_time = time.time()  # Update timestamp on successful receive
        except:
            pass

def status_monitor():
    """Background thread to update connection status every second."""
    while True:
        update_connection_status()
        time.sleep(1)

def gen_frames_cam0():
    colour = (255, 255, 255)
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.4
    font_width = 1
    while True:
        if frame_cam0 is None:
            continue
        f = frame_cam0.copy()  # Work with a copy to avoid accumulation
        frame_size = (f.shape[1], f.shape[0])
        
        # Draw Compass
        compass_size = 120
        draw_compass(f, mavlink_data['yaw'], 0, frame_size[1]-compass_size-10, compass_size)
        
        # Draw Attitude Indicator
        attitude_size = 120
        draw_attitude_indicator(f, mavlink_data['roll'], mavlink_data['pitch'], x=frame_size[0]-attitude_size-10, y=frame_size[1]-attitude_size-10, size=attitude_size)

        # Draw Battery Indicator
        draw_battery_widget(f, mavlink_data['battery_remaining'], position=(10, 10), width=60, height=20)

        # Draw Telemetry Text
        text = f"Roll: {mavlink_data['roll']:.1f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 15), font, font_scale, colour, font_width)
        
        text = f"Pitch: {mavlink_data['pitch']:.1f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 35), font, font_scale, colour, font_width)
        
        text = f"Yaw: {mavlink_data['yaw']:.1f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 55), font, font_scale, colour, font_width)
        
        text = f"Lat: {mavlink_data['lat']:.6f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 75), font, font_scale, colour, font_width)
        
        text = f"Lon: {mavlink_data['lon']:.6f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 95), font, font_scale, colour, font_width)
        
        text = f"Alt: {mavlink_data['alt']:.1f}m"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 115), font, font_scale, colour, font_width)
        
        text = f"Battery: {mavlink_data['battery']:.2f}V ({mavlink_data['battery_remaining']}%)"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 135), font, font_scale, colour, font_width)

        text = f"GS: {mavlink_data['ground_speed']:.1f}m/s"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 155), font, font_scale, colour, font_width)

        text = f"Throttle: {mavlink_data['throttle']}%"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 175), font, font_scale, colour, font_width)

        # Encode frame as JPEG with higher quality
        ret, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

def gen_frames_cam1():
    while True:
        if frame_cam1 is None:
            continue
        f = frame_cam1.copy()
        ret, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route('/api/authenticate', methods=['POST'])
def authenticate():
    data = request.json
    username = data.get('username', '').strip()
    password = data.get('password', '').strip()
    
    VALID_USERNAME = 'argus'
    VALID_PASSWORD = 'sentry'
    
    if username == VALID_USERNAME and password == VALID_PASSWORD:
        return jsonify({'success': True, 'token': secrets.token_hex(32)})
    else:
        return jsonify({'success': False, 'message': 'Invalid credentials'}), 401

@app.route('/api/logout', methods=['POST'])
def logout():
    return jsonify({'success': True})

@app.route('/telemetry')
def telemetry():
    """Return current telemetry data as JSON"""
    update_connection_status()  # Update status before returning
    telemetry_with_latency = mavlink_data.copy()
    telemetry_with_latency['client_latency_ms'] = telemetry_latency_ms
    return jsonify(telemetry_with_latency)

@app.route('/api/status')
def api_status():
    """Return current connection status for all systems."""
    update_connection_status()  # Update status before returning
    return jsonify({
        'main_online': get_main_online(),
        'cam0_online': cam0_online,
        'cam1_online': cam1_online,
        'mav_online': mav_online,
        'map_online': get_map_online(),
        'hq_online': hq_online
    })

def generate_map_html():
    """Generate map HTML with current telemetry data."""
    geojson_data, bounds = load_geodata()
    GPS_Location = [mavlink_data['lat'] if mavlink_data['lat'] != 0 else 53.406049,
                    mavlink_data['lon'] if mavlink_data['lon'] != 0 else -2.968585]
    yaw = mavlink_data['yaw']
    
    m = folium.Map(
        location=GPS_Location,
        zoom_start=20,
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri'
    )
    
    if geojson_data:
        folium.GeoJson(
            geojson_data,
            style_function=lambda x: {'color': '#0066cc', 'weight': 2}
        ).add_to(m)

    if bounds is not None:
        m.fit_bounds([[bounds[1], bounds[0]], [bounds[3], bounds[2]]])
    
    # Add icon at GPS location (centered and rotated)
    # Use a unique class name for JavaScript access
    icon_path = Path(__file__).parent / 'templates' / 'images' / 'sentry_icon_white.png'
    if icon_path.exists():
        icon_html = f'''
        <div class="sentry-marker-icon" style="
            width: 50px;
            height: 50px;
            transform: translate(-50%, -50%) rotate({yaw}deg);
        ">
            <img src="/images/sentry_icon_white.png" style="width: 50px; height: 50px; display: block;" />
        </div>
        '''
        marker = folium.Marker(
            location=GPS_Location,
            icon=folium.DivIcon(html=icon_html, class_name='sentry-marker-container')
        )
        marker.add_to(m)
    
    # Add reset button and marker update functionality
    map_id = m.get_name()
    bounds_js = "null"
    if bounds is not None:
        bounds_js = f"[[{bounds[1]}, {bounds[0]}], [{bounds[3]}, {bounds[2]}]]"
    reset_button = f"""
    <div id="reset-btn" style="position: fixed; top: 15px; right: 15px; z-index: 1000; display: none;">
        <button onclick="resetView()" style="
            padding: 10px 15px;
            background: white;
            border: 2px solid rgba(0,0,0,0.2);
            border-radius: 4px;
            cursor: pointer;
            font-weight: bold;
            box-shadow: 0 1px 5px rgba(0,0,0,0.4);
        ">Reset View</button>
    </div>
    <script>
        var __defaultCenter = [{GPS_Location[0]}, {GPS_Location[1]}];
        var __defaultZoom = 20;
        var __bounds = {bounds_js};
        var __sentryMarker = null;
        var __mapObj = null;

        // Find and store the marker reference on load
        document.addEventListener('DOMContentLoaded', function() {{
            __mapObj = window['{map_id}'];
            if (__mapObj) {{
                // Find all markers and get the sentry marker
                __mapObj.eachLayer(function(layer) {{
                    if (layer instanceof L.Marker) {{
                        __sentryMarker = layer;
                    }}
                }});
            }}
        }});

        // Update marker position and rotation
        window.updateSentryMarker = function(lat, lon, yaw) {{
            if (__sentryMarker && __mapObj) {{
                // Update position
                __sentryMarker.setLatLng([lat, lon]);
                
                // Update rotation
                var iconDiv = document.querySelector('.sentry-marker-icon');
                if (iconDiv) {{
                    iconDiv.style.transform = 'translate(-50%, -50%) rotate(' + yaw + 'deg)';
                }}
            }}
        }};

        window.resetView = function() {{
            var mapObj = window['{map_id}'];
            if (mapObj) {{
                mapObj.setView(__defaultCenter, __defaultZoom);
                mapObj.invalidateSize();
            }}
        }};

        window.fitBoundsView = function() {{
            var mapObj = window['{map_id}'];
            if (mapObj) {{
                if (__bounds) {{
                    mapObj.fitBounds(__bounds);
                }} else {{
                    mapObj.setView(__defaultCenter, __defaultZoom);
                }}
                mapObj.invalidateSize();
            }}
        }};
    </script>
    """
    m.get_root().html.add_child(folium.Element(reset_button))
    
    return m._repr_html_()

@app.route('/')
def index():
    update_connection_status()  # Update status before rendering
    return render_template(
        'index.html',
        main_online=get_main_online(),
        cam0_online=cam0_online,
        cam1_online=cam1_online,
        hq_online=hq_online,
        map_online=get_map_online(),
        map_html=generate_map_html(),
        status_update_interval_ms=STATUS_UPDATE_INTERVAL_MS
    )

@app.route('/video_feed_cam0')
def video_feed_cam0():
    return Response(gen_frames_cam0(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed_cam1')
def video_feed_cam1():
    return Response(gen_frames_cam1(), mimetype='multipart/x-mixed-replace; boundary=frame')

Thread(target=lambda: asyncio.run(receive_video_cam0()), daemon=True).start()
Thread(target=lambda: asyncio.run(receive_video_cam1()), daemon=True).start()
Thread(target=lambda: asyncio.run(receive_telemetry()), daemon=True).start()
# Legacy UDP receiver (optional, for backward compatibility)
Thread(target=receive_mavlink, daemon=True).start()
# Status monitor thread
Thread(target=status_monitor, daemon=True).start()
app.run(port=8000, threaded=True)
