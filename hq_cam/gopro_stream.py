#!/usr/bin/env python3
"""
GoPro USB Camera Stream via Flask
Captures video from GoPro connected via USB and serves it through a web interface
"""

from flask import Flask, render_template, Response
import cv2
import threading
import time

app = Flask(__name__)

# Global variables
camera = None
lock = threading.Lock()
output_frame = None

class VideoCamera:
    def __init__(self, camera_index=0):
        """Initialize the camera"""
        self.camera = None
        
        # Try different backends in order of preference
        backends = [
            (cv2.CAP_V4L2, "V4L2"),
            (cv2.CAP_ANY, "ANY"),
        ]
        
        for backend, name in backends:
            print(f"Trying backend: {name}")
            try:
                self.camera = cv2.VideoCapture(camera_index, backend)
                
                if self.camera.isOpened():
                    print(f"Successfully opened camera with {name} backend")
                    break
                else:
                    print(f"{name} backend failed to open camera")
                    if self.camera is not None:
                        self.camera.release()
                    self.camera = None
            except Exception as e:
                print(f"Error with {name} backend: {e}")
                if self.camera is not None:
                    self.camera.release()
                self.camera = None
        
        if self.camera is None or not self.camera.isOpened():
            raise ValueError(f"Unable to open camera at index {camera_index}")
        
        # Try to set camera properties for better quality
        # Start with lower resolution for better compatibility
        print("Setting camera properties...")
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer size for lower latency
        
        # Read actual properties
        actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        
        print(f"Camera opened at {actual_width}x{actual_height} @ {actual_fps}fps")
        
        # Warm up camera
        print("Warming up camera...")
        for _ in range(5):
            self.camera.read()
        time.sleep(1)
        
        print("Camera initialization complete!")
    
    def __del__(self):
        """Release the camera when done"""
        if self.camera is not None:
            self.camera.release()
    
    def get_frame(self):
        """Capture a single frame from the camera"""
        success, frame = self.camera.read()
        if not success:
            return None
        
        # Encode frame as JPEG
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            return None
        
        return jpeg.tobytes()

def capture_frames():
    """Background thread to continuously capture frames"""
    global camera, output_frame, lock
    
    while True:
        try:
            frame = camera.get_frame()
            
            if frame is not None:
                with lock:
                    output_frame = frame
            
            # Small delay to prevent CPU overuse
            time.sleep(0.03)  # ~30 FPS
            
        except Exception as e:
            print(f"Error capturing frame: {e}")
            time.sleep(1)

def generate_frames():
    """Generator function to yield frames for streaming"""
    global output_frame, lock
    
    while True:
        with lock:
            if output_frame is None:
                continue
            frame = output_frame.copy()
        
        # Yield the frame in multipart format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    """Video streaming home page"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    """Check camera status"""
    global camera
    if camera is not None and camera.camera.isOpened():
        return {'status': 'online', 'message': 'Camera is streaming'}
    return {'status': 'offline', 'message': 'Camera not available'}, 503

if __name__ == '__main__':
    try:
        # Initialize camera (try different indices if GoPro isn't at 0)
        print("Initializing GoPro camera...")
        camera = VideoCamera(camera_index=0)  # Change to 1, 2, etc. if needed
        print("Camera initialized successfully")
        
        # Start background thread for frame capture
        thread = threading.Thread(target=capture_frames, daemon=True)
        thread.start()
        print("Frame capture thread started")
        
        # Start Flask server
        print("Starting Flask server on http://0.0.0.0:5000")
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
        
    except Exception as e:
        print(f"Error: {e}")
        if camera is not None:
            del camera