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

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

PORT = 8888
CLIENTS = set()

# MAVLink UDP test publisher (for client MAVLink terminal tile)
MAVLINK_TEST_ENABLED = True
MAVLINK_TEST_UDP_ENDPOINT = 'udpout:127.0.0.1:14550'
MAVLINK_TEST_SOURCE_SYSTEM = 42
MAVLINK_TEST_SOURCE_COMPONENT = 1
MAVLINK_TEST_AUTOPILOT_TYPE = 8  # MAV_AUTOPILOT_PX4
MAVLINK_TEST_VEHICLE_TYPE = 2    # MAV_TYPE_QUADROTOR
MAVLINK_TEST_SEND_HZ = 10
MAVLINK_TEST_STATUSTEXT_INTERVAL_SEC = 2.5
MAVLINK_TEST_PILOT_MESSAGES = [
    (4, 'PreArm: GPS fix required'),
    (4, 'PreArm: Compass not calibrated'),
    (5, 'Arming checks running'),
    (6, 'GPS: 3D fix acquired'),
    (6, 'EKF2 IMU0 is using GPS'),
    (5, 'Ready to arm'),
]

_mavlink_test_conn = None
_mavlink_test_last_heartbeat_s = 0.0
_mavlink_test_last_statustext_s = 0.0
_mavlink_test_statustext_idx = 0

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


def _get_mavlink_test_connection():
    global _mavlink_test_conn

    if not MAVLINK_TEST_ENABLED or mavutil is None:
        return None

    if _mavlink_test_conn is not None:
        return _mavlink_test_conn

    try:
        _mavlink_test_conn = mavutil.mavlink_connection(
            MAVLINK_TEST_UDP_ENDPOINT,
            source_system=MAVLINK_TEST_SOURCE_SYSTEM,
            source_component=MAVLINK_TEST_SOURCE_COMPONENT,
        )
        print(f'MAVLink test publisher: sending to {MAVLINK_TEST_UDP_ENDPOINT}')
    except Exception as exc:
        print(f'MAVLink test publisher: failed to open endpoint: {exc}')
        _mavlink_test_conn = None

    return _mavlink_test_conn


def _send_mavlink_test_messages(*, now, roll, pitch, yaw, throttle, battery, lat, lon, alt, ground_speed):
    global _mavlink_test_last_heartbeat_s, _mavlink_test_last_statustext_s, _mavlink_test_statustext_idx

    conn = _get_mavlink_test_connection()
    if conn is None:
        return

    def _send_named_value(now_ms_value, name_value, numeric_value):
        last_exc = None
        name_candidates = [str(name_value)]
        try:
            name_candidates.append(str(name_value).encode('utf-8'))
        except Exception:
            pass

        for candidate in name_candidates:
            try:
                conn.mav.named_value_float_send(now_ms_value, candidate, float(numeric_value))
                return
            except Exception as exc:
                last_exc = exc
        if last_exc is not None:
            raise last_exc

    def _send_statustext(severity_value, text_value):
        last_exc = None
        text_candidates = [str(text_value)]
        try:
            text_candidates.append(str(text_value).encode('utf-8'))
        except Exception:
            pass

        for candidate in text_candidates:
            try:
                conn.mav.statustext_send(int(severity_value), candidate)
                return
            except Exception as exc:
                last_exc = exc
        if last_exc is not None:
            raise last_exc

    try:
        yaw_deg_signed = ((float(yaw) + 180.0) % 360.0) - 180.0
        roll_rad = math.radians(float(roll))
        pitch_rad = math.radians(float(pitch))
        yaw_rad = math.radians(yaw_deg_signed)
        throttle_pct = max(0.0, min(100.0, float(throttle)))
        battery_pct = int(max(0, min(100, round(float(battery)))))
        voltage_v = max(0.0, float(10.5 + (battery_pct / 100.0) * 2.1))
        voltage_mv = int(voltage_v * 1000)
        current_cA = int((6.0 + 4.0 * (throttle_pct / 100.0)) * 100)  # centi-amps
        alt_m = float(alt)
        rel_alt_m = max(0.0, alt_m - 95.0)
        lat_int = int(float(lat) * 1e7)
        lon_int = int(float(lon) * 1e7)
        alt_mm = int(alt_m * 1000)
        rel_alt_mm = int(rel_alt_m * 1000)
        vx = int(float(ground_speed) * 100)  # cm/s

        now_us = int(now * 1_000_000)
        now_ms = int(now * 1000) & 0xFFFFFFFF

        if (now - _mavlink_test_last_heartbeat_s) >= 1.0:
            conn.mav.heartbeat_send(
                MAVLINK_TEST_VEHICLE_TYPE,
                MAVLINK_TEST_AUTOPILOT_TYPE,
                0,
                0,
                0,
            )
            _mavlink_test_last_heartbeat_s = now

        conn.mav.attitude_send(
            now_ms,
            roll_rad,
            pitch_rad,
            yaw_rad,
            0.0,
            0.0,
            0.0,
        )

        conn.mav.sys_status_send(
            0,
            0,
            0,
            500,
            voltage_mv,
            current_cA,
            battery_pct,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        conn.mav.global_position_int_send(
            now_ms,
            lat_int,
            lon_int,
            alt_mm,
            rel_alt_mm,
            vx,
            0,
            0,
            int((float(yaw) % 360.0) * 100),
        )

        conn.mav.vfr_hud_send(
            float(ground_speed),
            float(ground_speed),
            int(yaw) % 360,
            int(throttle_pct),
            alt_m,
            0.0,
        )

        _send_named_value(now_ms, 'THROTTLE', throttle_pct)
        _send_named_value(now_ms, 'BATT_PCT', battery_pct)
        _send_named_value(now_ms, 'TEST_ALT', alt_m)

        if (now - _mavlink_test_last_statustext_s) >= MAVLINK_TEST_STATUSTEXT_INTERVAL_SEC:
            severity, text = MAVLINK_TEST_PILOT_MESSAGES[_mavlink_test_statustext_idx % len(MAVLINK_TEST_PILOT_MESSAGES)]
            _send_statustext(severity, text)
            _mavlink_test_statustext_idx += 1
            _mavlink_test_last_statustext_s = now
    except Exception as exc:
        print(f'MAVLink test publisher: send failed: {exc}')

async def producer():
    """Broadcast a synthetic yaw value to all connected clients at 20Hz."""
    t0 = time.time()
    send_interval = 1.0 / max(1, int(MAVLINK_TEST_SEND_HZ))
    next_mavlink_send = 0.0
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

            if now >= next_mavlink_send:
                _send_mavlink_test_messages(
                    now=now,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    throttle=throttle,
                    battery=battery,
                    lat=lat,
                    lon=lon,
                    alt=100.0 + 5.0 * math.sin(t * 0.2),
                    ground_speed=ground_speed,
                )
                next_mavlink_send = now + send_interval

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
