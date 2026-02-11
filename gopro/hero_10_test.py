#!/usr/bin/env python3
"""
GoPro Webcam Stream Server
Streams GoPro camera feed to a web browser
"""

from flask import Flask, render_template, Response
import cv2
import time

app = Flask(__name__)

# GoPro typically appears as /dev/video0, /dev/video1, or /dev/video2
# You may need to adjust this
CAMERA_INDEX = 0

def get_camera():
    """Initialize camera with retry logic"""
    camera = cv2.VideoCapture(CAMERA_INDEX)
    
    # Set camera properties for better performance
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_FPS, 30)
    
    return camera

def generate_frames():
    """Generate frames from GoPro camera"""
    camera = get_camera()
    
    try:
        while True:
            success, frame = camera.read()
            
            if not success:
                print("Failed to read frame")
                time.sleep(0.1)
                continue
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            if not ret:
                continue
            
            frame_bytes = buffer.tobytes()
            
            # Yield frame in multipart format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    
    finally:
        camera.release()

@app.route('/')
def index():
    """Render the main page"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    """Check camera status"""
    camera = cv2.VideoCapture(CAMERA_INDEX)
    is_opened = camera.isOpened()
    camera.release()
    
    return {
        'camera_connected': is_opened,
        'camera_index': CAMERA_INDEX
    }

if __name__ == '__main__':
    print("Starting GoPro Stream Server...")
    print("Connect your GoPro in webcam mode")
    print("Open http://localhost:5000 in your browser")
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)