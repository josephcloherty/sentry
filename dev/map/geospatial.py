from flask import Flask, render_template
from functools import lru_cache
from pathlib import Path
import geopandas as gpd
import folium
from folium.plugins import FloatImage

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
    yaw = 45  # Yaw angle in degrees (0-360)
    
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
        icon_html = f'''
        <div style="
            width: 50px;
            height: 50px;
            background-image: url('/static/sentry_icon_white.png');
            background-size: contain;
            background-repeat: no-repeat;
            background-position: center;
            transform: translate(-50%, -50%) rotate({yaw}deg);
        "></div>
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
    
    return render_template('index.html', map_html=m._repr_html_())

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=9000)