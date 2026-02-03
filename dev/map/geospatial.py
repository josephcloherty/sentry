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
    
    icon_path = Path(__file__).parent / 'static' / 'sentry_icon_white.png'
    if icon_path.exists():
        icon_html = '''
        <style>
        .sentry-rotating {
            width: 50px;
            height: 50px;
            background-image: url('/static/sentry_icon_white.png');
            background-size: contain;
            background-repeat: no-repeat;
            background-position: center;
            transform: translate(-50%, -50%) rotate(0deg);
            will-change: transform;
        }
        </style>
        <div class="sentry-rotating" id="sentry-icon"></div>
        '''
        folium.Marker(
            location=GPS_Location,
            icon=folium.DivIcon(html=icon_html)
        ).add_to(m)
    
    map_id = m.get_name()
    reset_button = f"""
    <div style="position: fixed; top: 80px; right: 10px; z-index: 1000;">
        <button onclick="window['{map_id}'].setView([{GPS_Location[0]}, {GPS_Location[1]}], 20)" style="
            padding: 10px 15px;
            background: white;
            border: 2px solid rgba(0,0,0,0.2);
            border-radius: 4px;
            cursor: pointer;
            font-weight: bold;
            box-shadow: 0 1px 5px rgba(0,0,0,0.4);
        ">Reset View</button>
    </div>
    """
    m.get_root().html.add_child(folium.Element(reset_button))
    
    telemetry_ws_script = """
    <script>
    (function() {
        var urls = ['ws://100.112.223.17:8764', 'ws://localhost:8888'];
        var urlIndex = 0;
        var el = null;
        var reconnectAttempts = 0;
        var maxReconnectAttempts = 3;

        function connect() {
            console.log('Attempting to connect to ' + urls[urlIndex] + ' (attempt ' + (reconnectAttempts + 1) + ')');
            
            var ws = new WebSocket(urls[urlIndex]);
            var connectionTimeout = setTimeout(function() {
                console.warn('Connection timeout for ' + urls[urlIndex]);
                ws.close();
            }, 3000);
            
            ws.onopen = function() {
                clearTimeout(connectionTimeout);
                console.log('✓ Connected to ' + urls[urlIndex]);
                reconnectAttempts = 0;
            };
            
            ws.onmessage = function(evt) {
                if (!el) el = document.getElementById('sentry-icon');
                if (!el) return;
                
                try {
                    var data = JSON.parse(evt.data);
                    var yaw = data.yaw || 0;
                } catch (e) {
                    var parts = evt.data.split(',');
                    var yaw = parts.length >= 3 ? parseFloat(parts[2]) : 0;
                }
                
                el.style.transform = 'translate(-50%, -50%) rotate(' + (yaw || 0) + 'deg)';
            };
            
            ws.onerror = function(err) {
                clearTimeout(connectionTimeout);
                console.error('WebSocket error for ' + urls[urlIndex], err);
            };
            
            ws.onclose = function() {
                clearTimeout(connectionTimeout);
                console.log('✗ Connection closed for ' + urls[urlIndex]);
                
                reconnectAttempts++;
                if (reconnectAttempts >= maxReconnectAttempts) {
                    console.log('Max attempts reached, trying next URL');
                    urlIndex++;
                    reconnectAttempts = 0;
                    
                    if (urlIndex >= urls.length) {
                        console.error('All telemetry URLs exhausted');
                        urlIndex = 0;
                    }
                }
                
                setTimeout(connect, 1000);
            };
        }
        
        setTimeout(connect, 300);
    })();
    </script>
    """
    m.get_root().html.add_child(folium.Element(telemetry_ws_script))
    
    return render_template('index.html', map_html=m._repr_html_())

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=9000)