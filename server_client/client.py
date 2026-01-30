import asyncio, cv2, numpy as np, websockets
from flask import Flask, render_template, Response
from threading import Thread
import socket
from compass import draw_compass
from attitude import draw_attitude_indicator

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 5000))
sock.setblocking(False)

app = Flask(__name__)
frame = None
mavlink_data = {"roll": 0, "pitch": 0, "yaw": 0, "lat": 0, "lon": 0, "alt": 0, "battery": 0, "battery_remaining": 0, "ground_speed": 0, "throttle": 0}

async def receive_video():
    global frame
    async with websockets.connect('ws://100.112.223.17:8765') as ws:
        print("Connected to video server.")
        while True:
            data = await ws.recv()
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

def receive_mavlink():
    global mavlink_data
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            values = data.decode().split(',')
            mavlink_data = {
                "roll": float(values[0]),
                "pitch": float(values[1]),
                "yaw": float(values[2]),
                "lat": float(values[3]),
                "lon": float(values[4]),
                "alt": float(values[5]),
                "battery": float(values[6]),
                "battery_remaining": float(values[7]),
                "ground_speed": float(values[8]),
                "throttle": float(values[9])
            }
        except:
            pass

def gen_frames():
    colour = (255, 255, 255)
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.4
    font_width = 1
    while True:
        if frame is None:
            continue
        f = frame.copy()  # Work with a copy to avoid accumulation
        frame_size = (f.shape[1], f.shape[0])
        
        # Draw Compass
        compass_size = 120
        draw_compass(f, mavlink_data['yaw'], 0, frame_size[1]-compass_size-10, compass_size)
        
        # Draw Attitude Indicator
        attitude_size = 120
        draw_attitude_indicator(f, mavlink_data['roll'], mavlink_data['pitch'], x=frame_size[0]-attitude_size-10, y=frame_size[1]-attitude_size-10, size=attitude_size)

        # Draw Telemetry Text
        text = f"Roll: {mavlink_data['roll']:.1f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 15), font, font_scale, colour, font_width)
        
        text = f"Pitch: {mavlink_data['pitch']:.1f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 35), font, font_scale, colour, font_width)
        
        text = f"Yaw: {mavlink_data['yaw']:.1f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 55), font, font_scale, colour, font_width)
        
        text = f"Lat: {mavlink_data['lat']:.6f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 75), font, font_scale, colour, font_width)
        
        text = f"Lon: {mavlink_data['lon']:.6f}"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 95), font, font_scale, colour, font_width)
        
        text = f"Alt: {mavlink_data['alt']:.1f}m"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 115), font, font_scale, colour, font_width)
        
        text = f"Battery: {mavlink_data['battery']:.2f}V ({mavlink_data['battery_remaining']}%)"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 135), font, font_scale, colour, font_width)

        text = f"GS: {mavlink_data['ground_speed']:.1f}m/s"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 155), font, font_scale, colour, font_width)

        text = f"Throttle: {mavlink_data['throttle']}%"
        text_size = cv2.getTextSize(text, font, font_scale, font_width)[0]
        cv2.putText(f, text, (frame_size[0] - text_size[0] - 10, 175), font, font_scale, colour, font_width)

        # Encode frame as JPEG with higher quality
        ret, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

Thread(target=lambda: asyncio.run(receive_video()), daemon=True).start()
Thread(target=receive_mavlink, daemon=True).start()
app.run(port=8000, threaded=True)
