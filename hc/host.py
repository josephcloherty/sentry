from picamera2 import Picamera2
import asyncio, websockets, cv2

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
cam.start()

async def stream(ws):
    while True:
        await ws.send(cv2.imencode('.jpg', cam.capture_array())[1].tobytes())
        await asyncio.sleep(0.033)

async def main():
    async with websockets.serve(stream, '0.0.0.0', 8765):
        await asyncio.Future()

asyncio.run(main())