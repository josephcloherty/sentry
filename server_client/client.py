import asyncio, cv2, numpy as np, websockets
from flask import Flask, render_template, Response
from threading import Thread

app = Flask(__name__)
frame = None

async def receive():
    global frame
    async with websockets.connect('ws://100.112.223.17:8765') as ws:
        while True:
            data = await ws.recv()
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

def gen():
    while True:
        if frame is not None:
            f = frame.copy()
            cv2.putText(f, "Stream", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            ret, jpeg = cv2.imencode('.jpg', f)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

Thread(target=lambda: asyncio.run(receive()), daemon=True).start()
app.run(port=8000, threaded=True)