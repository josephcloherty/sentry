"""
Map generation module for the Sentry Ground Control Station.
This module handles all map-related functionality including geodata loading
and dynamic map HTML generation with telemetry overlays.
"""

import folium
import geopandas as gpd
from functools import lru_cache
from pathlib import Path


@lru_cache(maxsize=1)
def load_geodata():
    """Load and cache geodata from GeoPackage file."""
    gpkg_path = Path(__file__).parent.parent / 'dev' / 'map' / 'NorthWest_Railways.gpkg'
    if not gpkg_path.exists():
        return None
    gdf = gpd.read_file(gpkg_path)
    gdf['geometry'] = gdf['geometry'].simplify(tolerance=0.001, preserve_topology=True)
    if gdf.crs and gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)
    return gdf.__geo_interface__


def generate_map_html(lat, lon, yaw, test_mode=False):
    """Generate interactive map HTML with current telemetry data."""
    geojson_data = load_geodata()

    # Use default location or current telemetry
    GPS_Location = [lat if lat != 0 else 53.406241, lon if lon != 0 else -2.96737]

    m = folium.Map(
        location=GPS_Location,
        zoom_start=20,
        tiles='https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}',
        attr='Google'
    )

    if geojson_data:
        folium.GeoJson(
            geojson_data,
            style_function=lambda x: {'color': '#0066cc', 'weight': 2}
        ).add_to(m)

    icon_path = Path(__file__).parent.parent / 'templates' / 'images' / 'sentry_icon_white.png'
    if icon_path.exists():
        icon_html = f'''
        <style>
        .sentry-rotating {{
            width: 50px;
            height: 50px;
            background-image: url('/images/sentry_icon_white.png');
            background-size: contain;
            background-repeat: no-repeat;
            background-position: center;
            transform: translate(-50%, -50%) rotate({yaw}deg);
            will-change: transform;
        }}
        </style>
        <div class="sentry-rotating" id="sentry-icon"></div>
        '''
        folium.Marker(
            location=GPS_Location,
            icon=folium.DivIcon(html=icon_html)
        ).add_to(m)

    map_id = m.get_name()
    reset_button = f"""
    <div id="reset-btn" style="position: absolute; top: 16px; right: 16px; z-index: 1000;">
        <button onclick="resetView()" style="
            padding: 10px 15px;
            background: white;
            border: 2px solid rgba(0,0,0,0.2);
            border-radius: 4px;
            cursor: pointer;
            font-weight: bold;
            box-shadow: 0 1px 5px rgba(0,0,0,0.4);
        ">Follow</button>
    </div>
    """
    m.get_root().html.add_child(folium.Element(reset_button))

    # Choose telemetry URL list. When `test_mode` is True include the local test WS as a fallback.
    if test_mode:
        # Prefer localhost first in test mode to reduce connection delays during development
        urls_js = "['ws://localhost:8888', 'ws://100.112.223.17:8764']"
    else:
        urls_js = "['ws://100.112.223.17:8764']"

    telemetry_ws_script = """
    <script>
    (function() {
        var urls = """ + urls_js + """;
        var urlIndex = 0;
        var el = null;
        var marker = null;
        var mapObj = null;
        var isFollowing = true;  // Start with following enabled
        
        // Make isFollowing globally accessible
        window.isFollowing = isFollowing;
        var reconnectAttempts = 0;
        var maxReconnectAttempts = 3;

        // Initialize marker reference when DOM is ready
        document.addEventListener('DOMContentLoaded', function() {
            mapObj = window['""" + map_id + """'];
            if (mapObj) {
                mapObj.eachLayer(function(layer) {
                    if (layer instanceof L.Marker) {
                        marker = layer;
                    }
                });

                // Disable following when user drags the map
                mapObj.on('dragstart', function() {
                    isFollowing = false;
                    window.isFollowing = false;
                });
            }
        });

        function resetView() {
            if (marker && mapObj) {
                var currentPos = marker.getLatLng();
                mapObj.setView(currentPos, 18);  // Use default zoom level
                mapObj.invalidateSize();
                isFollowing = true;  // Re-enable following when reset view is pressed
                window.isFollowing = true;
            }
        }

        // Make resetView globally accessible
        window.resetView = resetView;

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
                    var lat = data.lat || 0;
                    var lon = data.lon || 0;
                    var yaw = data.yaw || 0;

                    // Update marker position if we have valid coordinates
                    if (marker && lat !== 0 && lon !== 0) {
                        marker.setLatLng([lat, lon]);
                        // Auto-follow: center map on new position only if following is enabled
                        if (mapObj && isFollowing) {
                            mapObj.setView([lat, lon], mapObj.getZoom());
                        }
                    }

                    // Update rotation
                    el.style.transform = 'translate(-50%, -50%) rotate(' + yaw + 'deg)';
                } catch (e) {
                    var parts = evt.data.split(',');
                    var yaw = parts.length >= 3 ? parseFloat(parts[2]) : 0;
                    el.style.transform = 'translate(-50%, -50%) rotate(' + (yaw || 0) + 'deg)';
                }
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

    return m._repr_html_()