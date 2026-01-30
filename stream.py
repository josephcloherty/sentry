import cv2
from flask import Flask, Response, stream_with_context, render_template
from picamera2 import Picamera2
import numpy as np
import subprocess
import psutil
import time
import math
from pymavlink import mavutil

print("Connecting to mavlink-router...")
master = mavutil.mavlink_connection('udpin:localhost:14550')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected!")

def track_latency(start=None):
    return time.time() if start is None else f"{(time.time() - start) * 1000:.1f}ms"

master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)

frame_size = (640, 480)

app = Flask(__name__)
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": frame_size}))
picam2.start()

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

def get_mavlink_data():
    global telemetry
    
    msg = master.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'SYS_STATUS', 'GPS_RAW_INT', 'VFR_HUD'], blocking=False)
    
    if msg:
        msg_type = msg.get_type()
        
        if msg_type == 'ATTITUDE':
            telemetry['roll'] = math.degrees(msg.roll)
            telemetry['pitch'] = math.degrees(msg.pitch)
            telemetry['yaw'] = math.degrees(msg.yaw)

        if msg_type == 'GLOBAL_POSITION_INT':
            telemetry['lat'] = msg.lat / 1e7          # Convert from int to degrees
            telemetry['lon'] = msg.lon / 1e7          # Convert from int to degrees
            telemetry['alt'] = msg.relative_alt / 1000.0 # Convert mm to meters
            telemetry['heading'] = msg.hdg / 100.0    # Convert from cdeg to deg

        if msg_type == 'VFR_HUD':
            telemetry['ground_speed'] = msg.groundspeed
            telemetry['climb_rate'] = msg.climb     # Vertical speed (m/s)
            telemetry['airspeed'] = msg.airspeed
            telemetry['throttle'] = msg.throttle

        if msg_type == 'GPS_RAW_INT':
            telemetry['satellites'] = msg.satellites_visible
            telemetry['fix_type'] = msg.fix_type
            telemetry['h_acc'] = msg.eph / 100.0  # Horizontal accuracy in meters
            telemetry['v_acc'] = msg.epv / 100.0  # Vertical accuracy in meters

        if msg_type == 'SYS_STATUS':
            telemetry['battery'] = msg.voltage_battery / 1000.0 # mV to Volts
            telemetry['current_battery'] = msg.current_battery / 100.0 # cA to A
            telemetry['battery_remaining'] = msg.battery_remaining # Percentage

        if msg_type == 'HIGHRES_IMU':
            telemetry['accel_x'] = msg.xacc / 1000.0  # mg to g
            telemetry['accel_y'] = msg.yacc / 1000.0
            telemetry['accel_z'] = msg.zacc / 1000.0
            telemetry['gyro_x'] = msg.xgyro / 1000.0  # mdps to dps
            telemetry['gyro_y'] = msg.ygyro / 1000.0
            telemetry['gyro_z'] = msg.zgyro / 1000.0

        if msg_type == 'RC_CHANNELS':
            telemetry['rc_channels'] = msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw

    return telemetry

last_print = 0

def get_cpu_stats():
   # Get CPU temperature 
    temp_result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
    temp_str = temp_result.stdout.strip()
    temp_celsius = float(temp_str.split('=')[1].split("'")[0])
   # Get CPU Usage
    cpu_usage = psutil.cpu_percent(interval=None)
    return temp_celsius, cpu_usage

def gen_frames():
    while True:
        start = track_latency()
        frame = picam2.capture_array()
        
        data = get_mavlink_data()
        now = time.time()
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
        
        # Encoding
        ret, buffer = cv2.imencode('.jpg', frame)
        
        # Calculate total latency including encoding
        lat = track_latency(start)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(stream_with_context(gen_frames()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)