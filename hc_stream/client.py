#!/usr/bin/env python3
"""
Low-latency H.264 video client with telemetry overlay
Receives H.264 stream over TCP and telemetry via WebSocket, renders overlays locally
"""

import sys
import os
import time
import math
import threading
import queue
import argparse
import subprocess
from collections import deque

import cv2
import numpy as np
import socketio

# Configuration
DEFAULT_HOST = '192.168.1.100'  # Change to your Raspberry Pi IP
VIDEO_PORT = 8000
TELEMETRY_PORT = 8001
FRAME_SIZE = (640, 480)

# Global state
telemetry_queue = queue.Queue(maxsize=50)
latest_telemetry = {
    'roll': 0, 'pitch': 0, 'yaw': 0,
    'lat': 0, 'lon': 0, 'alt': 0, 'heading': 0,
    'ground_speed': 0, 'climb_rate': 0,
    'battery': 0, 'current_battery': 0, 'battery_remaining': 0,
    'airspeed': 0, 'throttle': 0,
    'satellites': 0, 'fix_type': 0, 'h_acc': 0, 'v_acc': 0,
    'accel_x': 0, 'accel_y': 0, 'accel_z': 0,
    'gyro_x': 0, 'gyro_y': 0, 'gyro_z': 0,
    'rc_channels': (0, 0, 0, 0, 0, 0, 0, 0),
    'cpu_temp': 0,
    'cpu_usage': 0,
    'timestamp': 0
}
telemetry_lock = threading.Lock()

# Overlay state
overlay_enabled = True

# Cache for overlay elements
_compass_cache = {}
_attitude_cache = {}


# ========================================
# OVERLAY RENDERING FUNCTIONS
# (Copied from attitude.py, compass.py)
# ========================================

def _create_compass_background(size):
    """Prerender the static compass background with transparency"""
    bg = np.zeros((size, size, 4), dtype=np.uint8)
    c = (size//2, size//2)
    r = size // 3
    
    cv2.circle(bg, c, r, (255, 255, 255, 255), -1, lineType=cv2.LINE_AA)
    cv2.circle(bg, c, r, (0, 0, 0, 255), 1, lineType=cv2.LINE_AA)
    cv2.circle(bg, c, r//20, (0, 0, 0, 255), 1, lineType=cv2.LINE_AA)
    
    # Cardinal directions
    for angle, label in [(0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')]:
        rad = math.radians(angle)
        tx = int(c[0] + r * 0.8 * math.sin(rad))
        ty = int(c[1] - r * 0.8 * math.cos(rad))
        text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, 0.3, 1)[0]
        cv2.putText(bg, label, (tx - text_size[0]//2, ty + text_size[1]//2),
                    cv2.FONT_HERSHEY_DUPLEX, 0.3, (0, 0, 0, 255), 1, lineType=cv2.LINE_AA)
    
    return bg


def draw_compass(img, yaw, x, y, size):
    """Draw compass with transparent background overlay"""
    if size not in _compass_cache:
        _compass_cache[size] = _create_compass_background(size)
    
    bg = _compass_cache[size].copy()
    c_bg = (size//2, size//2)
    r = size // 3
    
    # Needle
    yaw_rad = math.radians(yaw)
    nx = int(c_bg[0] + r * 0.8 * math.sin(yaw_rad))
    ny = int(c_bg[1] - r * 0.8 * math.cos(yaw_rad))
    cv2.arrowedLine(bg, c_bg, (nx, ny), (0, 0, 200, 255), 2, tipLength=0.2, line_type=cv2.LINE_AA)
    
    cv2.circle(bg, c_bg, r//20, (0, 0, 0, 255), -1, lineType=cv2.LINE_AA)
    
    # Yaw text
    text = f"{yaw:.1f} deg"
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
    text_x = c_bg[0] - text_size[0] // 2
    cv2.putText(bg, text, (text_x, size-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (255, 255, 255, 255), 1, lineType=cv2.LINE_AA)
    
    # Blend onto main image
    overlay_rgb = bg[:, :, :3]
    alpha = bg[:, :, 3:4] / 255.0
    roi = img[y:y+size, x:x+size]
    img[y:y+size, x:x+size] = (alpha * overlay_rgb + (1 - alpha) * roi).astype(np.uint8)
    
    return img


def _create_attitude_background(size):
    """Prerender the static attitude indicator background"""
    bg = np.zeros((size, size, 4), dtype=np.uint8)
    c = (size//2, size//2)
    
    # Create circular mask
    y, x = np.ogrid[:size, :size]
    mask = (x - c[0])**2 + (y - c[1])**2 <= (size//2)**2
    
    # Sky (top half) - blue
    bg[:c[1], :, 0] = 255
    bg[:c[1], :, 1] = 150
    bg[:c[1], :, 2] = 0
    bg[:c[1], :, 3] = 255
    
    # Ground (bottom half) - brown
    bg[c[1]:, :, 0] = 50
    bg[c[1]:, :, 1] = 100
    bg[c[1]:, :, 2] = 139
    bg[c[1]:, :, 3] = 255
    
    bg[~mask] = 0
    
    # Horizon line
    cv2.line(bg, (0, c[1]), (size, c[1]), (255, 255, 255, 255), 2, lineType=cv2.LINE_AA)
    
    return bg


def _create_static_overlay(size):
    """Prerender static overlay elements (aircraft symbol, scales, etc.)"""
    overlay = np.zeros((size, size, 4), dtype=np.uint8)
    c = (size//2, size//2)
    r = size // 2
    
    # Outer circle border
    cv2.circle(overlay, c, r-2, (255, 255, 255, 255), 2, lineType=cv2.LINE_AA)
    
    # Aircraft symbol (fixed yellow chevron in center)
    wing_width = size // 5
    wing_height = size // 25
    nose_height = size // 15
    
    # Left wing
    cv2.rectangle(overlay, (c[0] - wing_width, c[1] - wing_height//2),
                  (c[0] - wing_height, c[1] + wing_height//2), (0, 255, 255, 255), -1)
    # Right wing
    cv2.rectangle(overlay, (c[0] + wing_height, c[1] - wing_height//2),
                  (c[0] + wing_width, c[1] + wing_height//2), (0, 255, 255, 255), -1)
    # Center dot
    cv2.circle(overlay, c, wing_height//2, (0, 255, 255, 255), -1, lineType=cv2.LINE_AA)
    
    # Nose indicator
    pts = np.array([[c[0], c[1] - nose_height],
                    [c[0] - wing_height//2, c[1]],
                    [c[0] + wing_height//2, c[1]]], np.int32)
    cv2.fillPoly(overlay, [pts], (0, 255, 255, 255), lineType=cv2.LINE_AA)
    
    # Roll scale markers at top
    for angle in range(-60, 61, 15):
        if angle == 0:
            length = size // 12
            thickness = 3
        elif angle % 30 == 0:
            length = size // 15
            thickness = 2
        else:
            length = size // 20
            thickness = 1
        
        rad = math.radians(angle)
        x1 = int(c[0] + (r - 10) * math.sin(rad))
        y1 = int(c[1] - (r - 10) * math.cos(rad))
        x2 = int(c[0] + (r - 10 - length) * math.sin(rad))
        y2 = int(c[1] - (r - 10 - length) * math.cos(rad))
        cv2.line(overlay, (x1, y1), (x2, y2), (255, 255, 255, 255), thickness, lineType=cv2.LINE_AA)
    
    # Roll indicator triangle at top
    tri_size = size // 20
    pts = np.array([[c[0], r - 5],
                    [c[0] - tri_size//2, r - 5 - tri_size],
                    [c[0] + tri_size//2, r - 5 - tri_size]], np.int32)
    cv2.fillPoly(overlay, [pts], (255, 255, 255, 255), lineType=cv2.LINE_AA)
    
    return overlay


def _rotate_image(image, angle, center):
    """Rotate image around center point while preserving alpha channel"""
    rot_mat = cv2.getRotationMatrix2D(center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1],
                           flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                           borderValue=(0, 0, 0, 0))
    return result


def _apply_circular_mask(image, size):
    """Apply circular mask to keep content within circle bounds"""
    c = (size//2, size//2)
    r = size // 2
    
    y, x = np.ogrid[:size, :size]
    mask = (x - c[0])**2 + (y - c[1])**2 <= (r - 1)**2
    
    image[:, :, 3] = image[:, :, 3] * mask.astype(np.uint8)
    
    return image


def draw_attitude_indicator(img, roll, pitch, x, y, size):
    """Draw aircraft attitude indicator"""
    cache_key = size
    if cache_key not in _attitude_cache:
        _attitude_cache[cache_key] = {
            'background': _create_attitude_background(size),
            'overlay': _create_static_overlay(size)
        }
    
    c = (size//2, size//2)
    bg = _attitude_cache[cache_key]['background'].copy()
    
    # Apply pitch offset
    pitch_pixels = int(pitch * size / 120)
    if pitch_pixels != 0:
        M = np.float32([[1, 0, 0], [0, 1, pitch_pixels]])
        bg = cv2.warpAffine(bg, M, (size, size), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
    
    bg = _apply_circular_mask(bg, size)
    
    # Draw pitch ladder lines
    for pitch_angle in range(-60, 61, 10):
        if pitch_angle == 0:
            continue
        
        y_pos = c[1] - int((pitch_angle - pitch) * size / 120)
        if 0 <= y_pos < size:
            line_length = size // 6 if pitch_angle % 20 == 0 else size // 10
            color = (255, 255, 255, 255)
            thickness = 2 if pitch_angle % 20 == 0 else 1
            
            cv2.line(bg, (c[0] - line_length, y_pos), (c[0] + line_length, y_pos),
                    color, thickness, lineType=cv2.LINE_AA)
            
            text = f"{abs(pitch_angle)}"
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)[0]
            cv2.putText(bg, text, (c[0] - line_length - text_size[0] - 5, y_pos + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1, lineType=cv2.LINE_AA)
            cv2.putText(bg, text, (c[0] + line_length + 5, y_pos + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1, lineType=cv2.LINE_AA)
    
    # Rotate by roll angle
    bg_rotated = _rotate_image(bg, roll, c)
    bg_rotated = _apply_circular_mask(bg_rotated, size)
    
    # Get static overlay
    overlay = _attitude_cache[cache_key]['overlay'].copy()
    
    # Combine
    combined = bg_rotated.copy()
    overlay_rgb = overlay[:, :, :3]
    overlay_alpha = overlay[:, :, 3:4] / 255.0
    combined[:, :, :3] = (overlay_alpha * overlay_rgb + (1 - overlay_alpha) * combined[:, :, :3]).astype(np.uint8)
    combined[:, :, 3] = np.maximum(combined[:, :, 3], overlay[:, :, 3])
    
    # Blend onto main image
    alpha = combined[:, :, 3:4] / 255.0
    roi = img[y:y+size, x:x+size]
    img[y:y+size, x:x+size] = (alpha * combined[:, :, :3] + (1 - alpha) * roi).astype(np.uint8)
    
    return img


def draw_telemetry_overlay(frame, data):
    """Draw telemetry text overlay"""
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.4
    thickness = 1
    
    telemetry_items = [
        (f"Roll: {data['roll']:.1f}", 15),
        (f"Pitch: {data['pitch']:.1f}", 35),
        (f"Yaw: {data['yaw']:.1f}", 55),
        (f"Lat: {data['lat']:.6f}", 75),
        (f"Lon: {data['lon']:.6f}", 95),
        (f"Alt: {data['alt']:.1f}m", 115),
        (f"Battery: {data['battery']:.2f}V ({data['battery_remaining']}%)", 135),
        (f"GS: {data['ground_speed']:.1f}m/s", 155),
        (f"Throttle: {data['throttle']}%", 175),
    ]
    
    for text, y_pos in telemetry_items:
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        cv2.putText(frame, text, (FRAME_SIZE[0] - text_size[0] - 10, y_pos), 
                   font, font_scale, (255, 255, 255), thickness)


def draw_cpu_stats(frame, data):
    """Draw CPU stats overlay"""
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.4
    thickness = 1
    
    if data['cpu_temp'] > 0:
        temp_text = f"CPU: {data['cpu_temp']:.1f}C"
        cv2.putText(frame, temp_text, (10, 15), font, font_scale, (255, 255, 255), thickness)
    
    if data['cpu_usage'] > 0:
        cpu_text = f"Usage: {data['cpu_usage']}%"
        cv2.putText(frame, cpu_text, (10, 35), font, font_scale, (255, 255, 255), thickness)


def apply_overlays(frame, data, latency_ms):
    """Apply all overlays to frame"""
    if not overlay_enabled:
        # Only show latency when overlay disabled
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.4
        thickness = 1
        latency_text = f"Latency: {latency_ms:.1f}ms"
        cv2.putText(frame, latency_text, (10, 15), font, font_scale, (255, 255, 255), thickness)
        return frame
    
    # Draw compass
    compass_size = 120
    draw_compass(frame, data['yaw'], 0, FRAME_SIZE[1]-compass_size-10, compass_size)
    
    # Draw attitude indicator
    attitude_size = 120
    draw_attitude_indicator(frame, data['roll'], data['pitch'], 
                           x=FRAME_SIZE[0]-attitude_size-10, 
                           y=FRAME_SIZE[1]-attitude_size-10, 
                           size=attitude_size)
    
    # Draw telemetry text
    draw_telemetry_overlay(frame, data)
    
    # Draw CPU stats
    draw_cpu_stats(frame, data)
    
    # Draw latency
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.4
    thickness = 1
    latency_text = f"Latency: {latency_ms:.1f}ms"
    cv2.putText(frame, latency_text, (10, 55), font, font_scale, (255, 255, 255), thickness)
    
    return frame


# ========================================
# NETWORK RECEIVING FUNCTIONS
# ========================================

def telemetry_receiver(host):
    """Receive telemetry via WebSocket in separate thread"""
    global latest_telemetry
    
    sio = socketio.Client(
        reconnection=True,
        reconnection_delay=0.5,
        engineio_logger=False,
        logger=False
    )
    
    @sio.on('telemetry')
    def on_telemetry(data):
        with telemetry_lock:
            latest_telemetry.update(data)
    
    @sio.on('connect')
    def on_connect():
        print(f"Telemetry connected to {host}:{TELEMETRY_PORT}")
    
    @sio.on('disconnect')
    def on_disconnect():
        print("Telemetry disconnected")
    
    try:
        sio.connect(f'http://{host}:{TELEMETRY_PORT}')
        sio.wait()
    except Exception as e:
        print(f"Telemetry connection error: {e}")


def video_receiver(host):
    """Receive and decode H.264 video stream using ffmpeg"""
    print(f"Connecting to video stream at {host}:{VIDEO_PORT}...")
    
    try:
        import socket as sock_module
        
        # Connect to TCP video server
        sock = sock_module.socket(sock_module.AF_INET, sock_module.SOCK_STREAM)
        sock.setsockopt(sock_module.IPPROTO_TCP, sock_module.TCP_NODELAY, 1)
        sock.setsockopt(sock_module.SOL_SOCKET, sock_module.SO_RCVBUF, 65536)
        sock.settimeout(5.0)
        sock.connect((host, VIDEO_PORT))
        sock.settimeout(None)
        
        print("Video stream connected, decoding with ffmpeg...")
        
        # Start ffmpeg process to decode H.264 to raw BGR24 frames
        ffmpeg_cmd = [
            'ffmpeg',
            '-i', 'pipe:',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{FRAME_SIZE[0]}x{FRAME_SIZE[1]}',
            '-r', '30',
            '-fflags', 'nobuffer',
            'pipe:'
        ]
        
        process = subprocess.Popen(
            ffmpeg_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0
        )
        
        # Create window
        cv2.namedWindow('Low-Latency Stream', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Low-Latency Stream', FRAME_SIZE[0], FRAME_SIZE[1])
        
        frame_times = deque(maxlen=30)
        last_fps_update = time.time()
        fps = 0
        frame_count = 0
        frame_size = FRAME_SIZE[0] * FRAME_SIZE[1] * 3  # BGR24
        
        def feed_ffmpeg():
            """Feed H.264 data from socket to ffmpeg stdin"""
            try:
                while True:
                    chunk = sock.recv(65536)
                    if not chunk:
                        print("Server closed connection")
                        break
                    try:
                        process.stdin.write(chunk)
                        process.stdin.flush()
                    except Exception as e:
                        print(f"Error feeding ffmpeg: {e}")
                        break
            except Exception as e:
                print(f"Socket error: {e}")
            finally:
                try:
                    process.stdin.close()
                except:
                    pass
        
        # Start thread to feed ffmpeg
        feed_thread = threading.Thread(target=feed_ffmpeg, daemon=True)
        feed_thread.start()
        
        # Read decoded frames from ffmpeg
        while True:
            frame_start = time.time()
            
            # Read raw frame data
            frame_data = process.stdout.read(frame_size)
            if len(frame_data) < frame_size:
                print(f"Incomplete frame (got {len(frame_data)} bytes, expected {frame_size})")
                break
            
            # Convert bytes to numpy array
            img = np.frombuffer(frame_data, dtype=np.uint8).reshape((FRAME_SIZE[1], FRAME_SIZE[0], 3))
            
            # Make a copy to avoid issues
            img = img.copy()
            
            # Calculate latency
            latency_ms = (time.time() - frame_start) * 1000
            
            # Get latest telemetry
            with telemetry_lock:
                data_copy = latest_telemetry.copy()
            
            # Apply overlays
            img = apply_overlays(img, data_copy, latency_ms)
            
            # Calculate FPS
            frame_times.append(time.time())
            if time.time() - last_fps_update > 1.0:
                if len(frame_times) > 1:
                    fps = len(frame_times) / (frame_times[-1] - frame_times[0])
                last_fps_update = time.time()
            
            # Add FPS counter
            cv2.putText(img, f"FPS: {fps:.1f}", (10, FRAME_SIZE[1] - 10),
                       cv2.FONT_HERSHEY_DUPLEX, 0.4, (255, 255, 255), 1)
            
            # Display
            cv2.imshow('Low-Latency Stream', img)
            frame_count += 1
            
            # Handle key press
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print(f"User quit (frames decoded: {frame_count})")
                break
            elif key == ord('o'):
                global overlay_enabled
                overlay_enabled = not overlay_enabled
                print(f"Overlay: {'enabled' if overlay_enabled else 'disabled'}")
        
        print(f"Stream ended. Total frames decoded: {frame_count}")
        
        # Cleanup
        try:
            process.terminate()
            process.wait(timeout=5)
        except:
            process.kill()
                
    except FileNotFoundError:
        print("ERROR: ffmpeg not found. Please install ffmpeg:")
        print("  macOS: brew install ffmpeg")
        print("  Ubuntu: sudo apt install ffmpeg")
        print("  Raspberry Pi: sudo apt install ffmpeg")
    except Exception as e:
        print(f"Video connection error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        cv2.destroyAllWindows()
        try:
            sock.close()
        except:
            pass


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Low-latency video client')
    parser.add_argument('--host', default=DEFAULT_HOST, help='Raspberry Pi IP address')
    args = parser.parse_args()
    
    print("Low-latency H.264 Client")
    print("=" * 50)
    print(f"Connecting to: {args.host}")
    print(f"  Video: TCP port {VIDEO_PORT}")
    print(f"  Telemetry: WebSocket port {TELEMETRY_PORT}")
    print("\nControls:")
    print("  'o' - Toggle overlays")
    print("  'q' - Quit")
    print()
    
    # Start telemetry receiver in separate thread
    telemetry_thread = threading.Thread(target=telemetry_receiver, args=(args.host,), daemon=True)
    telemetry_thread.start()
    
    # Start video receiver in main thread
    try:
        video_receiver(args.host)
    except KeyboardInterrupt:
        print("\nExiting...")
    
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
