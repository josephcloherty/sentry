#!/usr/bin/env python3
"""
Simple WebSocket telemetry publisher for testing the map client.
Sends JSON messages like: {"yaw": 123.45, "timestamp": 167...}
Listens on port 8764 by default (matches your server/client code).
"""
import asyncio
import json
import math
import time
import cv2
import numpy as np
import struct

import websockets

PORT = 8888
CLIENTS = set()

# Camera stream ports (simple MJPEG placeholders)
CAM0_PORT = 8886
CAM1_PORT = 8887

async def producer():
    """Broadcast a synthetic yaw value to all connected clients at 20Hz."""
    t0 = time.time()
    try:
        while True:
            now = time.time()
            t = now - t0
            # Simulated GPS-based heading: we'll compute yaw from motion direction
            roll = 5.0 * math.sin(t * 1.5)  # +/-5 deg
            pitch = 3.0 * math.sin(t * 1.2)  # +/-3 deg
            # Throttle: pseudo throttle oscillating 5-95%
            throttle = 50.0 + 45.0 * math.sin(t * 5)
            # Battery: slow decreasing cyclic battery level between 20-100%
            battery = 60.0 + 40.0 * math.sin(t * 0.03)
            # Simulated GPS: circular motion around a base point
            # Base coordinate (example): somewhere in mid-latitude
            base_lat = 53.406581
            base_lon = -2.966903
            # radius in meters and convert to degrees (~111320 m per degree latitude)
            radius_m = 50.0
            deg_per_meter = 1.0 / 111320.0
            radius_deg = radius_m * deg_per_meter
            # use a slow angular speed for the circular motion
            angular_speed = 0.1  # rad/s
            angle = t * angular_speed
            lat = base_lat + radius_deg * math.sin(angle)
            # scale longitude by cos(latitude) to approximate degrees->meters
            lon = base_lon + (radius_deg * math.cos(angle)) / max(0.0001, math.cos(math.radians(base_lat)))
            # compute heading (yaw) analytically so the nose stays tangent to the circle
            # For the chosen param: lat ~ sin(angle), lon ~ cos(angle) -> heading = -angle (radians)
            yaw = ( -math.degrees(angle) + 360.0 ) % 360.0
            # ground speed (m/s) = radius (m) * angular_speed (rad/s)
            ground_speed = radius_m * angular_speed

            payload = json.dumps({
                "yaw": round(yaw, 2),
                "roll": round(roll, 2),
                "pitch": round(pitch, 2),
                "throttle": round(throttle, 1),
                "battery": round(battery, 1),
                # battery_remaining expected by client (percentage)
                "battery_remaining": int(max(0, min(100, round(battery)))),
                # additional fields client may expect
                "alt": round(100.0 + 5.0 * math.sin(t * 0.2), 1),
                "ground_speed": round(ground_speed, 1),
                "lat": round(lat, 6),
                "lon": round(lon, 6),
                "timestamp": time.time(),
            })
            if CLIENTS:
                await asyncio.gather(*(ws.send(payload) for ws in set(CLIENTS)))
            await asyncio.sleep(1.0 / 20.0)
    except asyncio.CancelledError:
        return

async def handler(ws):
    print('Telemetry test client connected')
    CLIENTS.add(ws)
    try:
        await ws.wait_closed()
    finally:
        CLIENTS.discard(ws)
        print('Telemetry test client disconnected')


def make_loading_frame(t, w=640, h=480, name='cam'):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cx, cy = w // 2, h // 2
    radius = min(w, h) // 6
    ticks = 12
    # rotating index based on time
    rot = int((t * 5) % ticks)
    for i in range(ticks):
        angle = 2 * math.pi * (i / ticks)
        x1 = int(cx + math.cos(angle) * (radius - 6))
        y1 = int(cy + math.sin(angle) * (radius - 6))
        x2 = int(cx + math.cos(angle) * (radius + 6))
        y2 = int(cy + math.sin(angle) * (radius + 6))
        idx = (i - rot) % ticks
        intensity = 255 - int(200 * (idx / ticks))
        color = (intensity, intensity, intensity)
        cv2.line(img, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)

    # small central dot
    cv2.circle(img, (cx, cy), 4, (180, 180, 180), -1, cv2.LINE_AA)
    # label
    cv2.putText(img, name + ' testing...', (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                cv2.LINE_AA)
    cv2.putText(img,'TEST MODE ENABLED', (cx-80, cy - 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                cv2.LINE_AA)
    return img


def make_cam_ws_handler(name):
    async def handler(ws):
        print(f'Camera server "{name}": client connected')
        try:
            while True:
                img = make_loading_frame(time.time(), name=name)
                _, jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                jpeg_bytes = jpeg.tobytes()
                payload = struct.pack('d', time.time()) + jpeg_bytes
                try:
                    await ws.send(payload)
                except Exception as e:
                    print(f'Camera server "{name}": send error: {e}')
                    raise
                await asyncio.sleep(0.1)
        except websockets.exceptions.ConnectionClosed:
            print(f'Camera server "{name}": client disconnected')
            return
        except Exception as e:
            print(f'Camera server "{name}": handler exception: {e}')
            try:
                import traceback
                traceback.print_exc()
            except Exception:
                pass
            return

    return handler

async def main():
    # websockets >=11 calls the handler with a single `websocket` argument,
    # so pass the `handler` directly (it accepts one argument).
    server = await websockets.serve(handler, '0.0.0.0', PORT)
    print(f'Telemetry test publisher running on ws://0.0.0.0:{PORT}')

    # Start WebSocket camera servers (send timestamped JPEG frames)
    camera_servers = []
    cam0_server = await websockets.serve(make_cam_ws_handler('cam0'), 'localhost', CAM0_PORT)
    cam1_server = await websockets.serve(make_cam_ws_handler('cam1'), 'localhost', CAM1_PORT)
    camera_servers.extend([cam0_server, cam1_server])

    producer_task = asyncio.create_task(producer())
    try:
        await asyncio.Future()  # run forever
    finally:
        producer_task.cancel()
        for s in camera_servers:
            try:
                s.close()
                await s.wait_closed()
            except Exception:
                pass
        server.close()
        await server.wait_closed()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('\nStopped by user')
