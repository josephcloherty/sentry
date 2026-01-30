import cv2
from flask import Flask, Response, stream_with_context, render_template
from picamera2 import Picamera2
import numpy as np
import subprocess
import psutil
import time

app = Flask(__name__)
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (800, 400)}))
picam2.start()

def track_latency(start=None):
    return time.time() if start is None else f"{(time.time() - start) * 1000:.1f}ms"

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
        
        # CPU Stats 
        temp, cpu = get_cpu_stats()
        if temp is not None:
            temp_text = f"CPU: {temp:.1f}C"
            cv2.putText(frame, temp_text, (10, 15), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255, 255, 255), 1)
        if cpu is not None:
            cpu_text = f"Usage: {cpu}%"
            cv2.putText(frame, cpu_text, (10, 35), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255, 255, 255), 1)
        
        # Latency Tracking
        lat = track_latency(start)
        latency_text = f"Latency: {lat}"
        cv2.putText(frame, latency_text, (10, 55), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255, 255, 255), 1)
        
        # Encoding
        ret, buffer = cv2.imencode('.jpg', frame)
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