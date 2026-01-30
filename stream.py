import cv2
from flask import Flask, Response, stream_with_context, render_template, request, jsonify
from picamera2 import Picamera2
import numpy as np
import time
import math
from attitude import draw_attitude_indicator
from mavlink_data import get_mavlink_data
from cpu_stats import get_cpu_stats
from compass import draw_compass
from pymavlink import mavutil

print("Connecting to mavlink-router...")
master = mavutil.mavlink_connection('udpin:localhost:14550')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected!")

master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)

telemetry = {
    'roll': 0, 'pitch': 0, 'yaw': 0,
    'lat': 0, 'lon': 0, 'alt': 0, 'heading': 0,
    'ground_speed': 0, 'climb_rate': 0, 
    'battery': 0,  'current_battery': 0, 'battery_remaining': 0,
    'heading': 0, 'airspeed': 0, 'throttle': 0,                        
    'satellites': 0, 'fix_type': 0, 'h_acc': 0, 'v_acc': 0,
    'accel_x': 0, 'accel_y': 0, 'accel_z': 0,
    'gyro_x': 0, 'gyro_y': 0, 'gyro_z': 0,
    'rc_channels': (0, 0, 0, 0, 0, 0, 0, 0)     
}

def track_latency(start=None):
    return time.time() if start is None else f"{(time.time() - start) * 1000:.1f}ms"

frame_size = (640, 480)

app = Flask(__name__)
overlay_enabled = True
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": frame_size}))
picam2.start()

last_print = 0

def gen_frames():
    while True:
        start = track_latency()
        frame = picam2.capture_array()
        
        # Always fetch data (non-blocking), but only display if overlay enabled
        data = get_mavlink_data(master, telemetry)
        now = time.time()

        if overlay_enabled:
           # Draw Compass
            compass_size = 120
            draw_compass(frame, data['yaw'], 0, frame_size[1]-compass_size-10, compass_size)
            
           # Draw Attitude Inicator
            attitude_size = 120
            draw_attitude_indicator(frame, data['roll'], data['pitch'], x=frame_size[0]-attitude_size-10, y=frame_size[1]-attitude_size-10, size=attitude_size)

           # Telemetry Overlay 
            last_print = 0
            if now - last_print >= 0.1:
                font = cv2.FONT_HERSHEY_DUPLEX
                font_scale = 0.4
                thickness = 1
                
                text = f"Roll: {data['roll']:.1f}"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 15), font, font_scale, (255, 255, 255), thickness)
                
                text = f"Pitch: {data['pitch']:.1f}"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 35), font, font_scale, (255, 255, 255), thickness)
                
                text = f"Yaw: {data['yaw']:.1f}"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 55), font, font_scale, (255, 255, 255), thickness)
                
                text = f"Lat: {data['lat']:.6f}"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 75), font, font_scale, (255, 255, 255), thickness)
                
                text = f"Lon: {data['lon']:.6f}"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 95), font, font_scale, (255, 255, 255), thickness)
                
                text = f"Alt: {data['alt']:.1f}m"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 115), font, font_scale, (255, 255, 255), thickness)
                
                text = f"Battery: {data['battery']:.2f}V ({data['battery_remaining']}%)"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 135), font, font_scale, (255, 255, 255), thickness)

                text = f"GS: {data['ground_speed']:.1f}m/s"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 155), font, font_scale, (255, 255, 255), thickness)

                text = f"Throttle: {data['throttle']}%"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 175), font, font_scale, (255, 255, 255), thickness)

                last_print = now  

            # CPU Stats 
            temp, cpu = get_cpu_stats()
            font = cv2.FONT_HERSHEY_DUPLEX
            font_scale = 0.4
            thickness = 1
            
            if temp is not None:
                temp_text = f"CPU: {temp:.1f}C"
                text_size = cv2.getTextSize(temp_text, font, font_scale, thickness)[0]
                cv2.putText(frame, temp_text, (10, 15), font, font_scale, (255, 255, 255), thickness)
            if cpu is not None:
                cpu_text = f"Usage: {cpu}%"
                text_size = cv2.getTextSize(cpu_text, font, font_scale, thickness)[0]
                cv2.putText(frame, cpu_text, (10, 35), font, font_scale, (255, 255, 255), thickness)
            
            # Latency Tracking (before encoding to include it in the overlay)
            lat_before_text = track_latency(start)
            font = cv2.FONT_HERSHEY_DUPLEX
            font_scale = 0.4
            thickness = 1
            latency_text = f"Latency: {lat_before_text}"
            text_size = cv2.getTextSize(latency_text, font, font_scale, thickness)[0]
            cv2.putText(frame, latency_text, (10, 55), font, font_scale, (255, 255, 255), thickness)
        
        else:
            # Only show latency when overlay is disabled (raw stream)
            lat_text = track_latency(start)
            font = cv2.FONT_HERSHEY_DUPLEX
            font_scale = 0.4
            thickness = 1
            latency_text = f"Latency: {lat_text}"
            text_size = cv2.getTextSize(latency_text, font, font_scale, thickness)[0]
            cv2.putText(frame, latency_text, (10, 15), font, font_scale, (255, 255, 255), thickness)
        
        # Encoding
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_overlay', methods=['POST'])
def set_overlay():
    global overlay_enabled
    data = request.get_json()
    overlay_enabled = data.get('overlay', True)
    return jsonify({'status': 'success', 'overlay': overlay_enabled})

@app.route('/video_feed')
def video_feed():
    return Response(stream_with_context(gen_frames()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)