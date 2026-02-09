"""
Map generation module for the Sentry Ground Control Station.
This module handles all map-related functionality including geodata loading
and dynamic map HTML generation with telemetry overlays.
"""

import folium
import geopandas as gpd
from functools import lru_cache
from pathlib import Path

# ===== MAP UI CONFIG (adjustable) =====
PIN_STORAGE_KEY = 'sentry_map_pins'
PIN_MAX_COUNT = 500
PIN_BUTTON_LABEL = 'Drop GPS Pin'
CLEAR_PINS_BUTTON_LABEL = 'Clear Pins'
FOLLOW_BUTTON_LABEL = 'Follow'


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
    <div id="map-action-buttons" style="position: absolute; top: 16px; right: 16px; z-index: 1000; display: flex; flex-direction: column; gap: 8px;">
        <button id="reset-btn" onclick="resetView()" style="
            padding: 10px 15px;
            background: white;
            border: 2px solid rgba(0,0,0,0.2);
            border-radius: 4px;
            cursor: pointer;
            font-weight: bold;
            box-shadow: 0 1px 5px rgba(0,0,0,0.4);
        ">{FOLLOW_BUTTON_LABEL}</button>
        <button id="clear-pins-btn" onclick="clearPins()" style="
            padding: 10px 15px;
            background: white;
            border: 2px solid rgba(0,0,0,0.2);
            border-radius: 4px;
            cursor: pointer;
            font-weight: bold;
            box-shadow: 0 1px 5px rgba(0,0,0,0.4);
        ">{CLEAR_PINS_BUTTON_LABEL}</button>
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
        var pinLayer = null;
        var pinStorageKey = '""" + PIN_STORAGE_KEY + """';
        var maxPins = """ + str(PIN_MAX_COUNT) + """;
        var isFollowing = true;  // Start with following enabled
        
        // Make isFollowing globally accessible
        window.isFollowing = isFollowing;
        var reconnectAttempts = 0;
        var maxReconnectAttempts = 3;

        function readPins() {
            try {
                var raw = localStorage.getItem(pinStorageKey);
                var pins = raw ? JSON.parse(raw) : [];
                if (!Array.isArray(pins)) return [];
                return pins.filter(function(p) {
                    return p && typeof p.lat === 'number' && typeof p.lon === 'number';
                });
            } catch (e) {
                return [];
            }
        }

        function writePins(pins) {
            try {
                var trimmed = pins.slice(-maxPins);
                localStorage.setItem(pinStorageKey, JSON.stringify(trimmed));
            } catch (e) {
                console.warn('Failed to persist pins', e);
            }
        }

        function ensurePinLayer() {
            if (!mapObj) return null;
            if (!pinLayer) {
                pinLayer = L.layerGroup().addTo(mapObj);
            }
            return pinLayer;
        }

        function renderPins(pins) {
            var layer = ensurePinLayer();
            if (!layer) return;
            layer.clearLayers();
            pins.forEach(function(p) {
                L.marker([p.lat, p.lon]).addTo(layer);
            });
        }

        function getCurrentLocation() {
            if (marker) {
                return marker.getLatLng();
            }
            if (mapObj) {
                return mapObj.getCenter();
            }
            return null;
        }

        function dropGpsPin() {
            var loc = getCurrentLocation();
            if (!loc) return;
            var pins = readPins();
            pins.push({ lat: loc.lat, lon: loc.lng, ts: Date.now() });
            writePins(pins);
            renderPins(pins);
        }

        function clearPins() {
            writePins([]);
            renderPins([]);
        }

        // Make pin methods globally accessible
        window.dropGpsPin = dropGpsPin;
        window.clearPins = clearPins;

        function resolveSentryMarker() {
            if (!mapObj) return null;
            var sentryMarker = null;
            mapObj.eachLayer(function(layer) {
                if (!(layer instanceof L.Marker)) return;
                var el = typeof layer.getElement === 'function' ? layer.getElement() : null;
                if (el && el.querySelector && el.querySelector('#sentry-icon')) {
                    sentryMarker = layer;
                }
            });
            return sentryMarker;
        }

        // Initialize marker reference when DOM is ready
        document.addEventListener('DOMContentLoaded', function() {
            mapObj = window['""" + map_id + """'];
            if (mapObj) {
                marker = resolveSentryMarker();

                // Disable following when user drags the map
                mapObj.on('dragstart', function() {
                    isFollowing = false;
                    window.isFollowing = false;
                });

                renderPins(readPins());
            }
        });

        window.addEventListener('storage', function(evt) {
            if (evt.key === pinStorageKey) {
                renderPins(readPins());
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

                    if (!marker && mapObj) {
                        marker = resolveSentryMarker();
                    }

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