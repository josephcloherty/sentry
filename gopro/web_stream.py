#!/usr/bin/env python3
"""
GoPro Hero 10 USB Web Streaming Script
Connects to GoPro Hero 10 via USB and streams to a web interface
"""

import asyncio
import cv2
import threading
import logging
from pathlib import Path
from typing import Optional
from flask import Flask, Response, render_template_string
from open_gopro import WiredGoPro
from open_gopro.constants import WebcamFOV, WebcamResolution

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Flask app
app = Flask(__name__)

# Global variables
frame_buffer = None
buffer_lock = threading.Lock()
gopro_instance = None

# HTML template for video display
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>GoPro Live Stream</title>
    <style>
        body {
            margin: 0;
            padding: 20px;
            background-color: #1a1a1a;
            font-family: Arial, sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
        }
        h1 {
            color: #ffffff;
            margin-bottom: 20px;
        }
        #video-container {
            position: relative;
            max-width: 90vw;
            max-height: 80vh;
            background-color: #000;
            border: 2px solid #00a8e8;
            border-radius: 8px;
            overflow: hidden;
        }
        img {
            width: 100%;
            height: auto;
            display: block;
        }
        .status {
            color: #00ff00;
            margin-top: 10px;
            font-size: 14px;
        }
        .info {
            color: #aaa;
            margin-top: 5px;
            font-size: 12px;
        }
    </style>
</head>
<body>
    <h1>🎥 GoPro Hero 10 Live Stream</h1>
    <div id="video-container">
        <img src="{{ url_for('video_feed') }}" alt="GoPro Stream">
    </div>
    <p class="status">● LIVE</p>
    <p class="info">Stream from GoPro Hero 10 Black via USB</p>
</body>
</html>
"""


class GoProStreamer:
    """Handles GoPro connection and streaming"""
    
    def __init__(self):
        self.gopro: Optional[WiredGoPro] = None
        self.stream_url: Optional[str] = None
        self.cap: Optional[cv2.VideoCapture] = None
        self.running = False
        
    async def connect_and_start_stream(self):
        """Connect to GoPro and start webcam stream"""
        try:
            logger.info("Connecting to GoPro via USB...")
            logger.info("Make sure camera is powered ON and connected via USB-C")
            
            # Connect to GoPro
            async with WiredGoPro() as gopro:
                self.gopro = gopro
                logger.info(f"Connected to: {gopro.identifier}")
                
                # Start webcam mode with settings
                logger.info("Starting webcam stream...")
                # Hero 10 doesn't need wired_usb_control disabled
                await asyncio.sleep(1)
                
                # Configure and start webcam
                await gopro.http_command.set_webcam_fov(fov=WebcamFOV.WIDE)
                await gopro.http_command.set_webcam_resolution(resolution=WebcamResolution.RES_1080)
                
                # Start the webcam
                await gopro.http_command.webcam_start()
                await asyncio.sleep(2)
                
                # Get stream URL
                self.stream_url = f"udp://172.2{gopro.identifier[-2:]}.1.1:8554"
                logger.info(f"Stream URL: {self.stream_url}")
                
                # Start capturing in a separate thread
                self.running = True
                capture_thread = threading.Thread(target=self.capture_frames, daemon=True)
                capture_thread.start()
                
                # Keep the stream alive
                logger.info("Stream active. Press Ctrl+C to stop.")
                try:
                    while self.running:
                        await asyncio.sleep(1)
                except KeyboardInterrupt:
                    logger.info("Stopping stream...")
                finally:
                    self.running = False
                    await gopro.http_command.webcam_stop()
                    
        except Exception as e:
            logger.error(f"Error: {e}")
            raise
    
    def capture_frames(self):
        """Capture frames from GoPro stream"""
        global frame_buffer
        
        try:
            # Try the calculated URL first, fallback to common addresses
            possible_urls = [
                self.stream_url,
                "udp://172.20.1.1:8554",
                "udp://172.21.1.1:8554",
                "udp://172.22.1.1:8554",
                "udp://172.23.1.1:8554",
                "udp://172.24.1.1:8554",
                "udp://172.25.1.1:8554",
            ]
            
            self.cap = None
            for url in possible_urls:
                logger.info(f"Trying to open stream: {url}")
                cap_test = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                if cap_test.isOpened():
                    self.cap = cap_test
                    logger.info(f"Successfully opened stream: {url}")
                    break
                cap_test.release()
            
            if not self.cap or not self.cap.isOpened():
                logger.error("Could not open video stream")
                return
            
            # Set buffer size to minimize latency
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            while self.running:
                ret, frame = self.cap.read()
                if ret:
                    # Encode frame as JPEG
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    
                    # Update global frame buffer
                    with buffer_lock:
                        frame_buffer = buffer.tobytes()
                else:
                    logger.warning("Failed to read frame")
                    
        except Exception as e:
            logger.error(f"Capture error: {e}")
        finally:
            if self.cap:
                self.cap.release()


def generate_frames():
    """Generator function to yield video frames"""
    global frame_buffer
    
    while True:
        with buffer_lock:
            if frame_buffer is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_buffer + b'\r\n')
        
        # Small delay to prevent busy waiting
        import time
        time.sleep(0.03)  # ~30 fps


@app.route('/')
def index():
    """Serve the main page"""
    return render_template_string(HTML_TEMPLATE)


@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


def run_gopro_stream():
    """Run the GoPro streaming in asyncio"""
    streamer = GoProStreamer()
    asyncio.run(streamer.connect_and_start_stream())


if __name__ == "__main__":
    # Start GoPro streaming in a separate thread
    gopro_thread = threading.Thread(target=run_gopro_stream, daemon=True)
    gopro_thread.start()
    
    # Give it time to connect
    import time
    logger.info("Waiting for GoPro to connect...")
    time.sleep(5)
    
    # Start Flask web server
    logger.info("Starting web server on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)