from flask import Flask, render_template
from functools import lru_cache
from pathlib import Path
import geopandas as gpd
import folium
from folium.plugins import FloatImage
import asyncio
import time

app = Flask(__name__)

@lru_cache(maxsize=1)
def load_geodata():
    gpkg_path = Path(__file__).parent / 'NorthWest_Railways.gpkg'
    gdf = gpd.read_file(gpkg_path)
    gdf['geometry'] = gdf['geometry'].simplify(tolerance=0.001, preserve_topology=True)
    if gdf.crs and gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)
    return gdf.__geo_interface__



@app.route('/')
def index():
    geojson_data = load_geodata()
    
    GPS_Location = [53.4084, -2.9916]
    # Remove blocking server-side loop. Rotation/animation should run in the
    # browser via CSS/JS so it updates live without blocking the Flask server.
    yaw = 0.0  # Initial yaw value; will be updated client-side via WebSocket.

    m = folium.Map(
        location=GPS_Location,
        zoom_start=20,
        tiles='https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}',
        attr='Google'
    )
    
    folium.GeoJson(
        geojson_data,
        style_function=lambda x: {'color': '#0066cc', 'weight': 2}
    ).add_to(m)
    
    # Add PNG image at GPS location - fixed pixel size at all zoom levels
    icon_path = Path(__file__).parent / 'static' / 'sentry_icon_white.png'
    if icon_path.exists():
        # Create a marker element the client can rotate based on live yaw telemetry.
        icon_html = f'''
        <style>
        .sentry-rotating {{
            width: 50px;
            height: 50px;
            background-image: url('/static/sentry_icon_white.png');
            background-size: contain;
            background-repeat: no-repeat;
            background-position: center;
            transform: translate(-50%, -50%) rotate(0deg);
            will-change: transform;
        }}
        </style>
        <div class="sentry-rotating" id="sentry-rotating"></div>
        '''
        folium.Marker(
            location=GPS_Location,
            icon=folium.DivIcon(html=icon_html)
        ).add_to(m)
    
    # Add reset button with corrected script
    map_id = m.get_name()
    reset_button = f"""
    <div id="reset-btn" style="position: fixed; top: 80px; right: 10px; z-index: 1000;">
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
        function resetView() {{
            var mapObj = window['{map_id}'];
            if (mapObj) {{
                mapObj.setView([{GPS_Location[0]}, {GPS_Location[1]}], 20);
            }}
        }}
    </script>
    """
    m.get_root().html.add_child(folium.Element(reset_button))
    # Client-side WebSocket to receive live MAVLink telemetry (yaw) and rotate icon.
    # Connects to the telemetry WebSocket served by `server.py` on port 8764.
    telemetry_ws_script = f"""
    <script>
    (function() {{
        // Build telemetry websocket URL using current host so it works across hosts.
        var host = window.location.hostname || 'localhost';
        var port = 8764; // must match TELEMETRY_PORT in server.py
        var url = 'ws://100.112.223.17:8764';

        function findSentry() {{
            return document.getElementById('sentry-rotating') || document.querySelector('.sentry-rotating') || document.querySelector('[class*="sentry-rotating"]');
        }}

        var el = null;
        function ensureEl() {{
            if (!el) el = findSentry();
            return !!el;
        }}

        // If the element isn't present yet, observe DOM mutations and resolve when found.
        function waitForSentry(timeoutMs) {{
            return new Promise(function(resolve) {{
                if (ensureEl()) return resolve(el);
                var obs = new MutationObserver(function() {{
                    if (ensureEl()) {{
                        obs.disconnect();
                        resolve(el);
                    }}
                }});
                obs.observe(document.body || document.documentElement, {{ childList: true, subtree: true }});
                if (timeoutMs) setTimeout(function() {{ obs.disconnect(); resolve(null); }}, timeoutMs);
            }});
        }}

        // Smoothly apply yaw (degrees) to the element.
        function applyYaw(yaw) {{
            if (!ensureEl()) return;
            // Ensure numeric
            var deg = Number(yaw) || 0;
            el.style.transform = 'translate(-50%, -50%) rotate(' + deg + 'deg)';
        }}

        // Try connecting to telemetry websocket and update yaw on messages.
        function connect() {{
            var ws = new WebSocket(url);
            ws.onopen = function() {{
                console.log('Telemetry websocket connected to ' + url);
            }};
            ws.onmessage = function(evt) {{
                console.log('telemetry msg:', evt.data);
                try {{
                    var data = JSON.parse(evt.data);
                    if (data && typeof data.yaw !== 'undefined') {{
                        // ensure element is ready before applying
                        if (!ensureEl()) {{
                            // wait up to 2s for the element to appear, then apply
                            waitForSentry(2000).then(function(found) {{ if (found) applyYaw(data.yaw); }});
                        }} else {{
                            applyYaw(data.yaw);
                        }}
                    }} else {{
                        // If payload is legacy CSV or array, try to extract yaw heuristically
                        if (typeof evt.data === 'string' && evt.data.indexOf(',') !== -1) {{
                            var parts = evt.data.split(',');
                            // assume yaw is third field in legacy UDP format
                            if (parts.length >= 3) {{
                                var maybeYaw = parseFloat(parts[2]);
                                if (!isNaN(maybeYaw)) applyYaw(maybeYaw);
                            }}
                        }}
                    }}
                }} catch (e) {{
                    console.warn('telemetry parse error', e);
                }}
            }};
            ws.onclose = function() {{
                console.log('Telemetry websocket closed, reconnecting in 1s');
                setTimeout(connect, 1000);
            }};
            ws.onerror = function(err) {{
                console.warn('Telemetry websocket error', err);
                ws.close();
            }};
        }}

        // Delay start until DOM and folium-generated DOM are ready.
        document.addEventListener('DOMContentLoaded', function() {{
            setTimeout(connect, 300);
        }});
    }})();
    </script>
    """
    m.get_root().html.add_child(folium.Element(telemetry_ws_script))
    
    return render_template('index.html', map_html=m._repr_html_())

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=9000)