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
from typing import Optional, Tuple

import websockets

PORT = 8888
CLIENTS = set()

# Camera stream ports (simple MJPEG placeholders)
CAM0_PORT = 8886
CAM1_PORT = 8887
CAM_HQ_PORT = 8885
CAM_FRAME_WIDTH = 640
CAM_FRAME_HEIGHT = 480
CAM_FPS = 10
HQ_FRAME_WIDTH = 854
HQ_FRAME_HEIGHT = 480
HQ_FPS = 10
HQ_TAG_COLOR = (0, 0, 0)
HQ_BG_COLOR = (235, 235, 235)
HQ_TAG_BORDER = 4
HQ_MIN_SCALE = 0.2
HQ_MAX_SCALE = 0.9
HQ_SCALE_SPEED = 0.35
TEST_VIDEO_FILE_PATH = '/Users/josephcloherty/Downloads/dronetest2.mp4'  # Example: '/Users/you/Videos/test_clip.mp4' (plays on all feeds when set)
TEST_VIDEO_LOOP = True
TEST_VIDEO_REOPEN_DELAY_SEC = 2.0
TEST_VIDEO_FRAME_SLEEP_SEC = 0.01
TEST_VIDEO_FALLBACK_TO_PLACEHOLDER = True
TEST_VIDEO_LABEL_COLOR = (60, 60, 60)
TEST_VIDEO_CAPTURE_BACKENDS = (cv2.CAP_FFMPEG, cv2.CAP_ANY)

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
        cap = None
        next_reopen_time = 0.0
        last_open_message = None
        try:
            while True:
                now = time.time()
                img = None
                has_video_source = bool(TEST_VIDEO_FILE_PATH.strip())

                if has_video_source:
                    if cap is None and now >= next_reopen_time:
                        cap, open_message = _open_test_video_capture()
                        if open_message != last_open_message:
                            print(f'Camera server "{name}": {open_message}')
                            last_open_message = open_message
                        if cap is None:
                            next_reopen_time = now + TEST_VIDEO_REOPEN_DELAY_SEC

                    if cap is not None:
                        ok, frame = cap.read()
                        if ok and frame is not None:
                            img = _prepare_video_frame(frame, CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, f'{name.upper()} TEST SOURCE: FILE')
                        else:
                            rewound = False
                            if TEST_VIDEO_LOOP:
                                rewound = cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                                if rewound:
                                    ok, frame = cap.read()
                                    if ok and frame is not None:
                                        img = _prepare_video_frame(frame, CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, f'{name.upper()} TEST SOURCE: FILE')
                            if img is None:
                                try:
                                    cap.release()
                                except Exception:
                                    pass
                                cap = None
                                next_reopen_time = now + TEST_VIDEO_REOPEN_DELAY_SEC

                if img is None:
                    img = make_loading_frame(now, w=CAM_FRAME_WIDTH, h=CAM_FRAME_HEIGHT, name=name)
                    if has_video_source and TEST_VIDEO_FALLBACK_TO_PLACEHOLDER:
                        cv2.putText(
                            img,
                            'VIDEO FILE UNAVAILABLE - FALLBACK ACTIVE',
                            (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (50, 50, 180),
                            1,
                            cv2.LINE_AA,
                        )

                _, jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                jpeg_bytes = jpeg.tobytes()
                payload = struct.pack('d', time.time()) + jpeg_bytes
                try:
                    await ws.send(payload)
                except Exception as e:
                    print(f'Camera server "{name}": send error: {e}')
                    raise
                await asyncio.sleep(1.0 / CAM_FPS)
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
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    return handler


def make_apriltag_frame(t, w=HQ_FRAME_WIDTH, h=HQ_FRAME_HEIGHT):
    img = np.full((h, w, 3), HQ_BG_COLOR, dtype=np.uint8)

    # Animate scale with a smooth oscillation
    scale = (math.sin(t * HQ_SCALE_SPEED * 2 * math.pi) + 1) / 2
    scale = HQ_MIN_SCALE + (HQ_MAX_SCALE - HQ_MIN_SCALE) * scale

    size = int(min(w, h) * scale)
    size = max(40, size)
    cx, cy = w // 2, h // 2
    half = size // 2

    x1, y1 = cx - half, cy - half
    x2, y2 = cx + half, cy + half

    # Outer black square
    cv2.rectangle(img, (x1, y1), (x2, y2), HQ_TAG_COLOR, -1)

    # Inner white border to mimic AprilTag
    border = max(6, size // 10)
    cv2.rectangle(img, (x1 + border, y1 + border), (x2 - border, y2 - border), HQ_BG_COLOR, -1)

    # Inner black square
    inner = max(10, size // 3)
    ix1, iy1 = cx - inner // 2, cy - inner // 2
    ix2, iy2 = cx + inner // 2, cy + inner // 2
    cv2.rectangle(img, (ix1, iy1), (ix2, iy2), HQ_TAG_COLOR, -1)

    # Label
    cv2.putText(img, 'APRILTAG DEMO', (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 1, cv2.LINE_AA)
    return img


def _open_test_video_capture() -> Tuple[Optional[cv2.VideoCapture], str]:
    def _open_capture_with_probe(file_path: str) -> Tuple[Optional[cv2.VideoCapture], str]:
        if not file_path:
            return None, 'TEST_VIDEO_FILE_PATH is empty'
        for backend in TEST_VIDEO_CAPTURE_BACKENDS:
            try:
                cap = cv2.VideoCapture(file_path, backend)
            except Exception:
                cap = cv2.VideoCapture(file_path)
            if not cap or not cap.isOpened():
                try:
                    cap.release()
                except Exception:
                    pass
                continue
            ok, frame = cap.read()
            if ok and frame is not None and frame.size > 0:
                return cap, f'opened stream with backend={backend}'
            try:
                cap.release()
            except Exception:
                pass
        return None, 'opencv could not decode video file'

    cap, cap_msg = _open_capture_with_probe(TEST_VIDEO_FILE_PATH.strip())
    if cap is not None:
        return cap, f'file stream opened ({cap_msg})'
    return None, f'could not open test video source (file): {cap_msg}'


def _prepare_video_frame(frame, width, height, source_label):
    if frame is None:
        return None
    resized = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    cv2.putText(
        resized,
        source_label,
        (10, height - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        TEST_VIDEO_LABEL_COLOR,
        1,
        cv2.LINE_AA,
    )
    return resized


def make_hq_ws_handler():
    async def handler(ws):
        print('Camera server "hq": client connected')
        cap = None
        next_reopen_time = 0.0
        last_open_message = None
        try:
            while True:
                now = time.time()
                img = None
                has_video_source = bool(TEST_VIDEO_FILE_PATH.strip())

                if has_video_source:
                    if cap is None and now >= next_reopen_time:
                        cap, open_message = _open_test_video_capture()
                        if open_message != last_open_message:
                            print(f'Camera server "hq": {open_message}')
                            last_open_message = open_message
                        if cap is None:
                            next_reopen_time = now + TEST_VIDEO_REOPEN_DELAY_SEC

                    if cap is not None:
                        ok, frame = cap.read()
                        if ok and frame is not None:
                            img = _prepare_video_frame(frame, HQ_FRAME_WIDTH, HQ_FRAME_HEIGHT, 'HQ TEST SOURCE: FILE')
                        else:
                            rewound = False
                            if TEST_VIDEO_LOOP:
                                rewound = cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                                if rewound:
                                    ok, frame = cap.read()
                                    if ok and frame is not None:
                                        img = _prepare_video_frame(frame, HQ_FRAME_WIDTH, HQ_FRAME_HEIGHT, 'HQ TEST SOURCE: FILE')
                            if img is None:
                                try:
                                    cap.release()
                                except Exception:
                                    pass
                                cap = None
                                next_reopen_time = now + TEST_VIDEO_REOPEN_DELAY_SEC

                if img is None:
                    img = make_apriltag_frame(now)
                    if has_video_source and TEST_VIDEO_FALLBACK_TO_PLACEHOLDER:
                        cv2.putText(
                            img,
                            'VIDEO FILE UNAVAILABLE - FALLBACK ACTIVE',
                            (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (50, 50, 180),
                            1,
                            cv2.LINE_AA,
                        )

                if img is None:
                    await asyncio.sleep(TEST_VIDEO_FRAME_SLEEP_SEC)
                    continue

                _, jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                payload = struct.pack('d', time.time()) + jpeg.tobytes()
                try:
                    await ws.send(payload)
                except Exception as e:
                    print(f'Camera server "hq": send error: {e}')
                    raise
                await asyncio.sleep(1.0 / HQ_FPS)
        except websockets.exceptions.ConnectionClosed:
            print('Camera server "hq": client disconnected')
            return
        except Exception as e:
            print(f'Camera server "hq": handler exception: {e}')
            try:
                import traceback
                traceback.print_exc()
            except Exception:
                pass
            return
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

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
    hq_server = await websockets.serve(make_hq_ws_handler(), 'localhost', CAM_HQ_PORT)
    camera_servers.extend([cam0_server, cam1_server, hq_server])

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
