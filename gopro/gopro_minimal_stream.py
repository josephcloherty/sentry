#!/usr/bin/env python3
"""
Minimal GoPro Hero 10 USB Streaming Script
Most compatible version with detailed debugging
"""

import cv2
import time
import logging
import subprocess
import requests
from flask import Flask, Response, render_template_string

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

app = Flask(__name__)

# Configuration
GOPRO_IPS = [f"172.{i}.1.1" for i in range(20, 26)]  # Try common IPs
STREAM_PORT = 8554
WEB_PORT = 5000

# Global state
current_gopro_ip = None
video_capture = None

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>GoPro Stream</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: #1a1a1a;
            font-family: Arial, sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
            padding: 20px;
        }
        h1 {
            color: #fff;
            margin-bottom: 20px;
            font-size: 2em;
        }
        #stream-container {
            position: relative;
            max-width: 95vw;
            background: #000;
            border: 3px solid #00a8e8;
            border-radius: 10px;
            overflow: hidden;
            box-shadow: 0 0 30px rgba(0, 168, 232, 0.3);
        }
        img {
            width: 100%;
            height: auto;
            display: block;
        }
        .status {
            color: #0f0;
            margin-top: 15px;
            font-size: 16px;
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .status::before {
            content: '●';
            font-size: 20px;
            animation: pulse 1.5s ease-in-out infinite;
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.3; }
        }
        .info {
            color: #888;
            margin-top: 10px;
            font-size: 14px;
        }
    </style>
</head>
<body>
    <h1>🎥 GoPro Hero 10 Live Stream</h1>
    <div id="stream-container">
        <img src="{{ url_for('video_feed') }}" alt="GoPro Stream">
    </div>
    <div class="status">LIVE</div>
    <div class="info">Streaming from GoPro Hero 10 Black</div>
</body>
</html>
"""

def find_gopro():
    """Find GoPro IP address by testing common addresses"""
    logger.info("Searching for GoPro on USB network...")
    
    for ip in GOPRO_IPS:
        try:
            logger.info(f"Testing {ip}...")
            response = requests.get(f"http://{ip}:8080/gopro/camera/state", timeout=2)
            if response.status_code == 200:
                logger.info(f"✓ Found GoPro at {ip}")
                return ip
        except requests.exceptions.RequestException:
            continue
    
    logger.error("✗ No GoPro found. Is it connected and powered on?")
    return None

def start_webcam(gopro_ip):
    """Start GoPro webcam mode via HTTP API"""
    try:
        logger.info("Starting webcam mode...")
        
        # Start webcam
        url = f"http://{gopro_ip}:8080/gopro/webcam/start"
        response = requests.get(url, timeout=5)
        logger.info(f"Webcam start: {response.status_code}")
        
        time.sleep(1)
        
        # Set resolution to 1080p (resolution=12)
        url = f"http://{gopro_ip}:8080/gopro/webcam/resolution?resolution=12"
        response = requests.get(url, timeout=5)
        logger.info(f"Set 1080p: {response.status_code}")
        
        # Set FOV to wide (fov=0)
        url = f"http://{gopro_ip}:8080/gopro/webcam/fov?fov=0"
        response = requests.get(url, timeout=5)
        logger.info(f"Set wide FOV: {response.status_code}")
        
        time.sleep(2)
        logger.info("✓ Webcam mode activated")
        return True
        
    except Exception as e:
        logger.error(f"✗ Failed to start webcam: {e}")
        return False

def open_stream(gopro_ip):
    """Open video stream with OpenCV"""
    global video_capture
    
    stream_url = f"udp://{gopro_ip}:{STREAM_PORT}"
    logger.info(f"Opening stream: {stream_url}")
    
    # Try with different backends
    backends = [cv2.CAP_FFMPEG, cv2.CAP_ANY]
    
    for backend in backends:
        logger.info(f"Trying backend: {backend}")
        cap = cv2.VideoCapture(stream_url, backend)
        
        if cap.isOpened():
            # Test if we can actually read a frame
            ret, frame = cap.read()
            if ret:
                logger.info(f"✓ Stream opened successfully with backend {backend}")
                video_capture = cap
                video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
                return True
            else:
                logger.warning(f"Stream opened but can't read frames with backend {backend}")
                cap.release()
        else:
            logger.warning(f"Failed to open with backend {backend}")
    
    logger.error("✗ Could not open video stream")
    return False

def generate_frames():
    """Generate video frames for Flask streaming"""
    global video_capture
    
    while True:
        if video_capture and video_capture.isOpened():
            ret, frame = video_capture.read()
            
            if ret:
                # Encode frame as JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                _, buffer = cv2.imencode('.jpg', frame, encode_param)
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            else:
                logger.warning("Failed to read frame")
                time.sleep(0.1)
        else:
            logger.warning("Video capture not ready")
            time.sleep(0.5)

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    """Health check endpoint"""
    status_info = {
        'gopro_ip': current_gopro_ip,
        'stream_active': video_capture is not None and video_capture.isOpened()
    }
    return status_info

def main():
    global current_gopro_ip
    
    logger.info("=" * 60)
    logger.info("GoPro Hero 10 USB Streaming")
    logger.info("=" * 60)
    
    # Step 1: Find GoPro
    current_gopro_ip = find_gopro()
    if not current_gopro_ip:
        logger.error("\nSetup checklist:")
        logger.error("1. Connect GoPro to Raspberry Pi via USB-C cable")
        logger.error("2. Power on the GoPro")
        logger.error("3. Wait 10 seconds for USB ethernet to initialize")
        logger.error("4. Check: ip addr | grep 172")
        return
    
    # Step 2: Start webcam mode
    if not start_webcam(current_gopro_ip):
        logger.error("Could not start webcam mode")
        return
    
    # Step 3: Open video stream
    if not open_stream(current_gopro_ip):
        logger.error("\nTroubleshooting:")
        logger.error("1. Try running: ffplay -fflags nobuffer udp://{}:8554".format(current_gopro_ip))
        logger.error("2. Check if ffmpeg is installed: ffmpeg -version")
        logger.error("3. Restart camera and try again")
        return
    
    # Step 4: Start web server
    logger.info("=" * 60)
    logger.info(f"✓ Stream is LIVE!")
    logger.info(f"✓ Open browser to: http://localhost:{WEB_PORT}")
    logger.info(f"✓ Or from network: http://YOUR_PI_IP:{WEB_PORT}")
    logger.info("=" * 60)
    
    try:
        app.run(host='0.0.0.0', port=WEB_PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
        if video_capture:
            video_capture.release()
        
        # Stop webcam
        try:
            requests.get(f"http://{current_gopro_ip}:8080/gopro/webcam/stop", timeout=2)
            logger.info("Webcam stopped")
        except:
            pass

if __name__ == "__main__":
    main()