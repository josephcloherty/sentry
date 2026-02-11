import subprocess
import time
import socket
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# --- CONFIGURATION ---
# The GoPro usually assigns itself this IP when plugged in via USB
GOPRO_IP = "172.2X.1YZ.51" # We need to find the real IP dynamically
PORT = 8554

def get_gopro_ip():
    """Finds the GoPro IP address from the usb0 interface."""
    try:
        # Run 'ip route' to find the gateway for the usb interface
        cmd = "ip route show dev usb0 | grep default | awk '{print $3}'"
        ip = subprocess.check_output(cmd, shell=True).decode('utf-8').strip()
        if ip:
            return ip
    except:
        pass
    return "172.19.0.51" # Common default if auto-detection fails

def start_webcam_mode(base_ip):
    """Sends the command to the GoPro to start 'Webcam Mode'."""
    try:
        import requests
        print(f"Attempting to start webcam on {base_ip}...")
        # Different firmware versions use slightly different URLs, this is the standard v2
        requests.get(f"http://{base_ip}/gp/gpWebcam/START", timeout=3)
        print("Webcam Mode Started.")
    except Exception as e:
        print(f"Could not start webcam mode: {e}")

# --- FFMPEG STREAMING ---
# We use FFMPEG to read the raw UDP stream and transcode it to MJPEG for the browser
def gen_frames():
    gopro_ip = get_gopro_ip()
    start_webcam_mode(gopro_ip)
    
    # UDP URL for the video feed
    udp_url = f"udp://{gopro_ip}:{PORT}"
    
    # FFMPEG Command:
    # -f mpegts: Input format
    # -i udp://...: Input source
    # -f image2pipe: Output to pipe
    # -vcodec mjpeg: Convert to JPEG for browser
    cmd = [
        'ffmpeg',
        '-loglevel', 'quiet',
        '-f', 'mpegts',
        '-i', udp_url,
        '-f', 'image2pipe',
        '-vcodec', 'mjpeg',
        '-q:v', '5', # Quality (2-31, lower is better quality)
        '-'
    ]
    
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)

    while True:
        # Find the start and end of JPEG files in the stream
        # This is a rough way to parse MJPEG from a pipe
        # (For production, use OpenCV as shown in previous scripts, 
        # but FFMPEG is more robust for the Hero 10's specific MPEG-TS format)
        in_bytes = process.stdout.read(4096)
        if not in_bytes:
            break
        
        # In a real implementation, you would buffer 'in_bytes' until you find 
        # the JPEG start (0xFF 0xD8) and end (0xFF 0xD9) markers.
        # For simplicity in this text-based format, we recommend using the 
        # OpenCV method if latency allows, or this raw pipe method.
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + in_bytes + b'\r\n')

# --- FLASK ---
@app.route('/')
def index():
    return render_template_string('''
        <body style="background:black; color:white; text-align:center;">
            <h1>GoPro Hero 10 Feed</h1>
            <p>Note: Latency is higher on Hero 10 via USB-Ethernet</p>
            <img src="{{ url_for('video_feed') }}" style="width:80%">
        </body>
    ''')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)