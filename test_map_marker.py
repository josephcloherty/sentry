#!/usr/bin/env python3
"""
Test script to verify the map marker update approach.
This creates a simple HTML file with a Leaflet map and a marker that can be updated.
Run this and open the generated HTML file in a browser to verify.
"""

html_content = '''
<!DOCTYPE html>
<html>
<head>
    <title>Map Marker Update Test</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        #map { height: 400px; width: 100%; }
        .rotated-icon {
            width: 50px;
            height: 50px;
        }
        .rotated-icon img {
            width: 100%;
            height: 100%;
        }
    </style>
</head>
<body>
    <h1>Map Marker Update Test</h1>
    <div id="map"></div>
    <p>Marker position: <span id="position">-</span></p>
    <p>Marker rotation: <span id="rotation">-</span></p>
    <button onclick="updateMarker()">Update Marker (Random)</button>
    
    <script>
        // Initialize map
        const map = L.map('map').setView([53.406049, -2.968585], 15);
        
        L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            attribution: 'Esri'
        }).addTo(map);
        
        // Create a custom icon with rotation support
        let currentRotation = 0;
        let currentLat = 53.406049;
        let currentLon = -2.968585;
        
        // Use DivIcon for rotation support
        function createRotatedIcon(rotation) {
            return L.divIcon({
                className: 'rotated-icon',
                html: `<div style="transform: rotate(${rotation}deg); width: 50px; height: 50px;">
                    <svg viewBox="0 0 24 24" fill="white" stroke="black" stroke-width="1">
                        <path d="M12 2L4 20h16L12 2z"/>
                    </svg>
                </div>`,
                iconSize: [50, 50],
                iconAnchor: [25, 25]
            });
        }
        
        // Create marker
        let marker = L.marker([currentLat, currentLon], {
            icon: createRotatedIcon(currentRotation)
        }).addTo(map);
        
        // Function to update marker position and rotation
        function updateMarkerPosition(lat, lon, rotation) {
            marker.setLatLng([lat, lon]);
            marker.setIcon(createRotatedIcon(rotation));
            document.getElementById('position').textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
            document.getElementById('rotation').textContent = `${rotation.toFixed(1)}°`;
        }
        
        // Test function - random updates
        function updateMarker() {
            currentLat += (Math.random() - 0.5) * 0.001;
            currentLon += (Math.random() - 0.5) * 0.001;
            currentRotation = (currentRotation + 30) % 360;
            updateMarkerPosition(currentLat, currentLon, currentRotation);
        }
        
        // Update marker every 2 seconds for testing
        setInterval(() => {
            currentRotation = (currentRotation + 5) % 360;
            updateMarkerPosition(currentLat, currentLon, currentRotation);
        }, 2000);
        
        console.log('Test script loaded. Marker will rotate every 2 seconds.');
    </script>
</body>
</html>
'''

# Write the test file
with open('/Users/josephcloherty/Desktop/AERO420/sentry-git/test_map_marker.html', 'w') as f:
    f.write(html_content)

print("Test file created: test_map_marker.html")
print("Open this file in a browser to verify the marker rotation works.")
print("")
print("Expected behavior:")
print("1. Map loads with a white triangle marker")
print("2. Marker rotates 5 degrees every 2 seconds")
print("3. Click 'Update Marker' button to move marker randomly")
print("")
print("If this works, the approach is valid for the real implementation.")
