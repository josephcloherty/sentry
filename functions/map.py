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
GUIDED_MAP_MESSAGE_TYPE = 'sentry_guided_map_dblclick'
GUIDED_MARKER_LABEL = 'Guided target'
GUIDED_MARKER_COLOR = '#ef4444'
GUIDED_MARKER_SIZE_PX = 22
GUIDED_MARKER_STROKE_PX = 3
GUIDED_MARKER_TIME_LABEL = 'Sent'

# ===== MAP DATA CONFIG (adjustable) =====
MAP_GPKG_FILENAME = 'NorthWest_Railways.gpkg'
MAP_GPKG_DIRS = [
    Path(__file__).parent.parent / 'maps',
    Path(__file__).parent.parent / 'dev' / 'map',
    Path(__file__).parent.parent,
]
MAP_GPKG_LAYER_PREFERENCES = [
    'railways',
    'railway',
    'rails',
    'tracks',
]


@lru_cache(maxsize=1)
def load_geodata():
    """Load and cache geodata from GeoPackage file."""
    # Find any .gpkg files in the candidate directories
    gpkg_files = []
    for d in MAP_GPKG_DIRS:
        try:
            if d.exists() and d.is_dir():
                gpkg_files.extend(sorted(d.glob('*.gpkg')))
        except Exception:
            continue

    if not gpkg_files:
        return None

    results = []
    for gf in gpkg_files:
        try:
            import fiona  # type: ignore
            layers = [layer.lower() for layer in fiona.listlayers(gf)]
        except Exception:
            layers = []

        # pick preferred layer if present, otherwise let geopandas pick
        chosen = None
        if layers:
            pref = next((layer for layer in MAP_GPKG_LAYER_PREFERENCES if layer in layers), None)
            if pref:
                try:
                    gdf = gpd.read_file(gf, layer=pref)
                    chosen = (pref, gdf)
                except Exception:
                    chosen = None

        if chosen is None:
            try:
                gdf = gpd.read_file(gf)
            except Exception:
                continue

        if gdf is None or gdf.empty:
            continue

        # simplify and ensure WGS84
        try:
            gdf['geometry'] = gdf['geometry'].simplify(tolerance=0.001, preserve_topology=True)
        except Exception:
            pass
        if gdf.crs and getattr(gdf.crs, 'to_epsg', lambda: None)() != 4326:
            try:
                gdf = gdf.to_crs(epsg=4326)
            except Exception:
                pass

        results.append({
            'file': str(gf.name),
            'geojson': gdf.__geo_interface__,
            'bounds': gdf.total_bounds.tolist(),
        })

    if not results:
        return None

    return results


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

    # If load_geodata returned multiple datasets, add them all and fit bounds to combined extent
    if geojson_data:
        combined_bounds = None  # [minx, miny, maxx, maxy]
        datasets = geojson_data if isinstance(geojson_data, list) else [geojson_data]
        for ds in datasets:
            geojson = ds.get('geojson') if isinstance(ds, dict) else ds
            bounds = ds.get('bounds') if isinstance(ds, dict) else None
            fname = ds.get('file', '').lower() if isinstance(ds, dict) else ''

            # If this dataset appears to be a stations/points layer, render as red circle markers
            if 'station' in fname or 'stations' in fname or (isinstance(geojson, dict) and all((f.get('geometry', {}).get('type','').lower() in ('point','multipoint') for f in geojson.get('features', []) if f.get('geometry')))):
                fg = folium.FeatureGroup(name=ds.get('file', None) if isinstance(ds, dict) else None)
                features = geojson.get('features', []) if isinstance(geojson, dict) else []
                for feat in features:
                    geom = feat.get('geometry') if isinstance(feat, dict) else None
                    if not geom:
                        continue
                    gtype = geom.get('type', '').lower()
                    coords = geom.get('coordinates')
                    if not coords:
                        continue
                    if gtype == 'point':
                        lon, lat = coords[0], coords[1]
                        folium.CircleMarker(location=[lat, lon], radius=5, color='red', fill=True, fill_color='red', fill_opacity=0.9).add_to(fg)
                    elif gtype == 'multipoint':
                        for c in coords:
                            lon, lat = c[0], c[1]
                            folium.CircleMarker(location=[lat, lon], radius=5, color='red', fill=True, fill_color='red', fill_opacity=0.9).add_to(fg)
                fg.add_to(m)
            else:
                folium.GeoJson(
                    geojson,
                    name=ds.get('file', None) if isinstance(ds, dict) else None,
                    style_function=lambda x: {'color': '#0066cc', 'weight': 2}
                ).add_to(m)

            if bounds and len(bounds) == 4:
                if combined_bounds is None:
                    combined_bounds = bounds.copy()
                else:
                    combined_bounds[0] = min(combined_bounds[0], bounds[0])
                    combined_bounds[1] = min(combined_bounds[1], bounds[1])
                    combined_bounds[2] = max(combined_bounds[2], bounds[2])
                    combined_bounds[3] = max(combined_bounds[3], bounds[3])

        # Fit map to combined bounds
        try:
            if combined_bounds and len(combined_bounds) == 4:
                minx, miny, maxx, maxy = combined_bounds
                sw = [miny, minx]
                ne = [maxy, maxx]
                m.fit_bounds([sw, ne])
        except Exception:
            pass

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
        var ws = null;
        var pinLayer = null;
        var guidedLayer = null;
        var guidedModeEnabled = false;
        var guidedMarkerColor = '""" + GUIDED_MARKER_COLOR + """';
        var guidedMarkerSizePx = """ + str(int(GUIDED_MARKER_SIZE_PX)) + """;
        var guidedMarkerStrokePx = """ + str(int(GUIDED_MARKER_STROKE_PX)) + """;
        var guidedMessageType = '""" + GUIDED_MAP_MESSAGE_TYPE + """';
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

        function ensureGuidedLayer() {
            if (!mapObj) return null;
            if (!guidedLayer) {
                guidedLayer = L.layerGroup().addTo(mapObj);
            }
            return guidedLayer;
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

        function buildGuidedIcon() {
            var size = guidedMarkerSizePx;
            var stroke = guidedMarkerStrokePx;
            var html = '<div style="position:relative;width:' + size + 'px;height:' + size + 'px;">'
                + '<span style="position:absolute;left:50%;top:0;transform:translateX(-50%);width:' + stroke + 'px;height:' + size + 'px;background:' + guidedMarkerColor + ';border-radius:999px;"></span>'
                + '<span style="position:absolute;left:0;top:50%;transform:translateY(-50%);width:' + size + 'px;height:' + stroke + 'px;background:' + guidedMarkerColor + ';border-radius:999px;"></span>'
                + '</div>';

            return L.divIcon({
                className: 'guided-target-plus',
                html: html,
                iconSize: [size, size],
                iconAnchor: [size / 2, size / 2]
            });
        }

        function formatGuidedSecondsSince(sentTsMs) {
            if (!Number.isFinite(sentTsMs)) return '--';
            var elapsedMs = Math.max(0, Date.now() - sentTsMs);
            var elapsedSec = elapsedMs / 1000.0;
            if (elapsedSec < 60) {
                return elapsedSec.toFixed(1) + 's ago';
            }
            var mins = Math.floor(elapsedSec / 60);
            var secs = Math.floor(elapsedSec % 60);
            return mins + 'm ' + secs + 's ago';
        }

        function buildGuidedTooltipHtml(lat, lon, alt, sentTsMs) {
            var latText = Number.isFinite(lat) ? lat.toFixed(6) : '--';
            var lonText = Number.isFinite(lon) ? lon.toFixed(6) : '--';
            var altText = Number.isFinite(alt) ? alt.toFixed(1) + ' m' : '--';
            var ageText = formatGuidedSecondsSince(sentTsMs);
            return '<div style="font-size:12px;line-height:1.4;">'
                + '<div><strong>Lat:</strong> ' + latText + '</div>'
                + '<div><strong>Lon:</strong> ' + lonText + '</div>'
                + '<div><strong>Alt:</strong> ' + altText + '</div>'
                + '<div><strong>""" + GUIDED_MARKER_TIME_LABEL + """:</strong> ' + ageText + '</div>'
                + '</div>';
        }

        function addGuidedTargetMarker(lat, lon, alt, sentTsMs) {
            var layer = ensureGuidedLayer();
            if (!layer) return;
            var marker = L.marker([lat, lon], { icon: buildGuidedIcon() }).addTo(layer);
            var sentAt = Number.isFinite(sentTsMs) ? sentTsMs : Date.now();
            marker.bindTooltip(buildGuidedTooltipHtml(Number(lat), Number(lon), Number(alt), sentAt), {
                direction: 'top',
                opacity: 0.95
            });
            marker.on('mouseover', function() {
                marker.setTooltipContent(buildGuidedTooltipHtml(Number(lat), Number(lon), Number(alt), sentAt));
            });
        }

        function setGuidedModeEnabled(enabled) {
            guidedModeEnabled = !!enabled;
            window.guidedModeEnabled = guidedModeEnabled;
            if (!mapObj) return;
            try {
                if (guidedModeEnabled) {
                    mapObj.doubleClickZoom.disable();
                } else {
                    mapObj.doubleClickZoom.enable();
                }
            } catch (e) {
                console.warn('Unable to toggle map double-click zoom', e);
            }
        }

        // Make pin methods globally accessible
        window.dropGpsPin = dropGpsPin;
        window.clearPins = clearPins;
        window.setGuidedModeEnabled = setGuidedModeEnabled;
        window.addGuidedTargetMarker = addGuidedTargetMarker;

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

                mapObj.on('dblclick', function(evt) {
                    if (!guidedModeEnabled || !evt || !evt.latlng) {
                        return;
                    }

                    try {
                        if (window.parent && window.parent !== window) {
                            window.parent.postMessage({
                                type: guidedMessageType,
                                lat: Number(evt.latlng.lat),
                                lon: Number(evt.latlng.lng)
                            }, '*');
                        }
                    } catch (e) {
                        console.warn('Failed to notify parent about guided map click', e);
                    }
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

        function reloadTelemetrySocket() {
            reconnectAttempts = 0;
            urlIndex = 0;
            if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
                ws.close();
                return;
            }
            connect();
        }

        // Make reload available to the parent page
        window.reloadTelemetrySocket = reloadTelemetrySocket;

        function connect() {
            console.log('Attempting to connect to ' + urls[urlIndex] + ' (attempt ' + (reconnectAttempts + 1) + ')');

            ws = new WebSocket(urls[urlIndex]);
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