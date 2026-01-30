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
mavlink_data = {"roll": 0, "pitch": 0, "yaw": 0}

async def receive_video():
    global frame
    async with websockets.connect('ws://100.112.223.17:8765') as ws:
        while True:
            data = await ws.recv()
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

def receive_mavlink():
    global mavlink_data
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            roll, pitch, yaw = data.decode().split(',')
            mavlink_data = {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)}
        except:
            pass

def gen_frames():
    colour = (255, 255, 255)
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.4
    font_width = 1
    frame_size = (frame.shape[1], frame.shape[0])
    while True:
        # Draw Compass
        compass_size = 120
        draw_compass(frame, mavlink_data['yaw'], 0, frame_size[1]-compass_size-10, compass_size)
        
        # Draw Attitude Indicator
        attitude_size = 120
        draw_attitude_indicator(frame, mavlink_data['roll'], mavlink_data['pitch'], x=frame_size[0]-attitude_size-10, y=frame_size[1]-attitude_size-10, size=attitude_size)

        # Draw Telemetry Text
        text = f"Roll: {mavlink_data['roll']:.1f}"
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 15), font, font_scale, (255, 255, 255), thickness)
        
        text = f"Pitch: {mavlink_data['pitch']:.1f}"
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        cv2.putText(frame, text, (frame_size[0] - text_size[0] - 10, 35), font, font_scale, (255, 255, 255), thickness)
        
        text = f"Yaw: {mavlink_data['yaw']:.1f}"
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

        # Encode frame as JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
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
