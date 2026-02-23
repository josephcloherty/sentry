# Import libraries
import asyncio
import cv2
import json
import logging
import numpy as np
import os
import platform
import secrets
import socket
import struct
import time
import websockets
from copy import deepcopy
from threading import Thread, Lock
from collections import deque
import statistics

from flask import Flask, Response, jsonify, render_template, request, make_response

from functions.attitude import draw_attitude_indicator
from functions.battery import draw_battery_widget
from functions.map import generate_map_html
from functions.compass import draw_compass
from functions.throttle import draw_throttle_widget

try:
    from fiducial_tracker import process_fiducial_frame
except Exception:
    process_fiducial_frame = None

# ===== Configuration =====
STATUS_UPDATE_INTERVAL_MS = 2000
STATUS_TIMEOUT = 3.0
LATENCY_SAMPLE_SIZE = 200
TELEMETRY_RECONNECT_DELAY_SEC = 2.0
TELEMETRY_RECV_TIMEOUT_SEC = 5.0
TELEMETRY_PING_INTERVAL_SEC = 5.0
TELEMETRY_PING_TIMEOUT_SEC = 5.0
VIDEO_RECONNECT_DELAY_SEC = 2.0
FIDUCIAL_RECONNECT_DELAY_SEC = 2.0
COMMAND_RECONNECT_DELAY_SEC = 2.0
FIDUCIAL_OVERLAY_ENABLED = True
FIDUCIAL_MIN_CONFIDENCE = 0.2
FIDUCIAL_STALE_TIMEOUT_SEC = 2.0
FIDUCIAL_TEXT_SCALE = 0.5
FIDUCIAL_TEXT_THICKNESS = 1
FIDUCIAL_LINE_THICKNESS = 2
FIDUCIAL_COLOR = (0, 0, 255)
FIDUCIAL_APPLY_FRAME_SCALING = True
FIDUCIAL_CLAMP_TO_FRAME = True
HQ_CV_MODE_UNSELECTED = ''
HQ_CV_MODE_NONE = 'none'
HQ_CV_MODE_LANDING = 'landing_mode'
HQ_CV_MODE_TARGET_TRACKING = 'target_tracking'
HQ_CV_DEFAULT_MODE = HQ_CV_MODE_NONE
HQ_CV_ALLOWED_MODES = {
    HQ_CV_MODE_UNSELECTED,
    HQ_CV_MODE_NONE,
    HQ_CV_MODE_LANDING,
    HQ_CV_MODE_TARGET_TRACKING,
}
HQ_CV_TARGET_TRACKING_TEXT = 'Target Tracking (Coming Soon)'
HQ_CV_TARGET_TRACKING_TEXT_SCALE = 0.7
HQ_CV_TARGET_TRACKING_TEXT_THICKNESS = 2
HQ_CV_TARGET_TRACKING_TEXT_COLOR = (255, 220, 120)

# Target tracking workflow settings (mirrors object_detection defaults)
HQ_TT_MODEL_COREML_PATH = './object_detection/coreml/best8np2.mlpackage'
HQ_TT_MODEL_COREML_FALLBACK_PATH = './object_detection/coreml/best26p2.mlpackage'
HQ_TT_MODEL_PATH = './object_detection/best8np2.pt'
HQ_TT_MODEL_FALLBACK_PATH = './object_detection/best8np2.pt'
HQ_TT_TRACKER_PATH = './object_detection/bytetrack_persist.yaml'
HQ_TT_CONFIDENCE = 0.35
HQ_TT_IOU = 0.2
HQ_TT_IMAGE_SIZE = 800
HQ_TT_PERSIST = True
HQ_TT_DEVICE = 'auto'
HQ_TT_PERSON_ONLY = False
HQ_TT_LOCK_ON_CONF = 0.50
HQ_TT_STICK_MIN_CONF = 0.15
HQ_TT_HOLD_FRAMES = 12
HQ_TT_SELECTED_HOLD_MULTIPLIER = 2
HQ_TT_REACQUIRE_MIN_IOU = 0.08
HQ_TT_REACQUIRE_MAX_CENTER_DIST_RATIO = 0.20
HQ_TT_BOX_COLOR = (255, 170, 60)
HQ_TT_TARGET_COLOR = (40, 220, 120)
HQ_TT_TEXT_COLOR = (255, 255, 255)
HQ_LANDING_LOCAL_FIDUCIAL_FALLBACK_IN_TEST = True
HQ_LANDING_STATUS_TEXT_COLOR = (120, 220, 255)
HQ_TT_FALLBACK_IN_TEST = True
HQ_TT_FALLBACK_MIN_AREA_RATIO = 0.002
GUIDED_GOTO_LAT_MIN = -90.0
GUIDED_GOTO_LAT_MAX = 90.0
GUIDED_GOTO_LON_MIN = -180.0
GUIDED_GOTO_LON_MAX = 180.0
GUIDED_GOTO_MIN_ALT_M = 1.0
GUIDED_GOTO_MAX_ALT_M = 500.0
HTTP_REQUEST_LOG_FILENAME = 'client_log'
HTTP_REQUEST_LOG_ENCODING = 'utf-8'
HTTP_REQUEST_LOG_LEVEL = logging.INFO

TEST_MODE = True

if TEST_MODE:
    SERVER_IP = 'localhost'
    CAM0_PORT = 8886
    CAM1_PORT = 8887
    CAM_HQ_PORT = 8885
    TELEMETRY_PORT = 8888
    COMMAND_PORT = 8889
    FIDUCIAL_PORT = 8890
else:
    SERVER_IP = '100.112.223.17'
    CAM0_PORT = 8765
    CAM1_PORT = 8766
    CAM_HQ_PORT = 8767
    TELEMETRY_PORT = 8764
    COMMAND_PORT = 8763
    FIDUCIAL_PORT = 8770
VALID_USERNAME = 'argus'
VALID_PASSWORD = 'sentry'

# Camera function tabs (shown on right side of camera cards)
# Each entry must match a command id in templates/index.html COMMAND_DEFS
DEFAULT_CAMERA_FUNCTION_TABS = {
    'cam0': ['go_dark', 'drop_gps_pin', 'emergency'],
    'cam1': ['go_dark', 'loiter', 'emergency'],
    'hq': ['loiter', 'drop_gps_pin', 'emergency']
}

# Home screen tile ordering
DEFAULT_TILE_ORDER = ['cam0', 'cam1', 'hq', 'map', 'latency', 'commands']

# Per-user UI settings defaults
DEFAULT_SETTINGS = {
    'test_mode': TEST_MODE,
    'latency_polling_rate_ms': 500,
    'status_update_interval_ms': 30000,
    'visible_tiles': {
        'cam0': True,
        'cam1': True,
        'hq': True,
        'map': True,
        'latency': True,
        'commands': True
    },
    'camera_function_tabs': DEFAULT_CAMERA_FUNCTION_TABS,
    'tile_order': DEFAULT_TILE_ORDER
}

ALLOWED_TILE_IDS = {'cam0', 'cam1', 'hq', 'map', 'latency', 'commands'}
ALLOWED_COMMAND_IDS = {
    'go_dark', 'auto_rth', 'drop_gps_pin', 'emergency', 'loiter', 'landing_mode'
}

# ===== Global State =====
app = Flask(__name__, static_folder='templates', static_url_path='')


def configure_http_request_logging():
    """Route Werkzeug access logs to file while keeping startup messages in terminal."""
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), HTTP_REQUEST_LOG_FILENAME)

    # Clear log file on each client restart
    with open(log_path, 'w', encoding=HTTP_REQUEST_LOG_ENCODING):
        pass

    def _is_http_access_log(record):
        try:
            message = record.getMessage()
        except Exception:
            return False
        return isinstance(message, str) and 'HTTP/' in message

    werkzeug_logger = logging.getLogger('werkzeug')
    werkzeug_logger.setLevel(HTTP_REQUEST_LOG_LEVEL)
    werkzeug_logger.handlers.clear()
    werkzeug_logger.propagate = False

    class AccessLogOnlyFilter(logging.Filter):
        def filter(self, record):
            return _is_http_access_log(record)

    class NonAccessLogFilter(logging.Filter):
        def filter(self, record):
            return not _is_http_access_log(record)

    terminal_handler = logging.StreamHandler()
    terminal_handler.setLevel(HTTP_REQUEST_LOG_LEVEL)
    terminal_handler.setFormatter(logging.Formatter('%(message)s'))
    terminal_handler.addFilter(NonAccessLogFilter())
    werkzeug_logger.addHandler(terminal_handler)

    http_log_handler = logging.FileHandler(log_path, mode='a', encoding=HTTP_REQUEST_LOG_ENCODING)
    http_log_handler.setLevel(HTTP_REQUEST_LOG_LEVEL)
    http_log_handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    http_log_handler.addFilter(AccessLogOnlyFilter())
    werkzeug_logger.addHandler(http_log_handler)

# Video frames (store dicts with timing info)
frame_cam0 = None
frame_cam1 = None
frame_hq = None

# Per-camera latency sample buffers (store tuples of (network_ms, render_ms))
cam0_latency_samples = deque(maxlen=LATENCY_SAMPLE_SIZE)
cam1_latency_samples = deque(maxlen=LATENCY_SAMPLE_SIZE)
hq_latency_samples = deque(maxlen=LATENCY_SAMPLE_SIZE)

# Connection timestamps
last_cam0_time = 0
last_cam1_time = 0
last_hq_time = 0
last_mav_time = 0

# Video latency tracking
video_latency_cam0_ms = 0  # Network latency (server to client receive)
video_latency_cam1_ms = 0
video_latency_hq_ms = 0
cam0_frame_timestamp = 0
cam1_frame_timestamp = 0
hq_frame_timestamp = 0
cam0_processing_latency_ms = 0  # Backend processing time (overlays + JPEG encode)
cam1_processing_latency_ms = 0
hq_processing_latency_ms = 0

# Connection status
cam0_online = False
cam1_online = False
hq_online = False
mav_online = False

# Fiducial detection state
fiducial_state = None
fiducial_lock = Lock()
hq_cv_mode = HQ_CV_DEFAULT_MODE
hq_cv_mode_lock = Lock()

# Target tracking runtime state
hq_tt_model = None
hq_tt_model_lock = Lock()
hq_tt_infer_lock = Lock()
hq_tt_error = None
hq_tt_class_filter = None
hq_tt_state_lock = Lock()
hq_tt_selected_track_id = None
hq_tt_selected_last_box = None
hq_tt_selected_missed_frames = 0
hq_tt_latest_detections = []
hq_tt_latest_frame_width = 0
hq_tt_latest_frame_height = 0

# Telemetry data
mavlink_data = {
    "roll": 0, "pitch": 0, "yaw": 0,
    "lat": 0, "lon": 0, "alt": 0,
    "battery": 0, "battery_remaining": 0,
    "ground_speed": 0, "throttle": 0,
    "timestamp": 0, "server_latency_ms": 0
}
telemetry_latency_ms = 0

# Command state (mirrors server-side boolean flags)
command_state = {
    'go_dark': False,
    'auto_rth': False,
    'drop_gps_pin': False,
    'emergency': False,
    'loiter': False,
    'landing_mode': False,
}
command_ws = None  # Persistent WebSocket to server for commands

# Session + settings state
session_tokens = {}
settings_by_user = {}
settings_by_token = {}

# Legacy UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 5000))
sock.setblocking(False)


# ===== Helper Functions =====
def update_connection_status():
    """Update connection status based on last received data timestamps."""
    global cam0_online, cam1_online, hq_online, mav_online
    current_time = time.time()
    cam0_online = last_cam0_time > 0 and (current_time - last_cam0_time) < STATUS_TIMEOUT
    cam1_online = last_cam1_time > 0 and (current_time - last_cam1_time) < STATUS_TIMEOUT
    hq_online = last_hq_time > 0 and (current_time - last_hq_time) < STATUS_TIMEOUT
    mav_online = last_mav_time > 0 and (current_time - last_mav_time) < STATUS_TIMEOUT


def is_main_online():
    return cam0_online or cam1_online or mav_online


def is_map_online():
    return mav_online


def decode_video_frame(data):
    """Extract timestamp and decode JPEG from video data."""
    if len(data) > 8:
        timestamp = struct.unpack('d', data[:8])[0]
        jpeg_data = data[8:]
    else:
        timestamp = None
        jpeg_data = data
    frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
    return frame, timestamp


def draw_telemetry_text(frame, data):
    """Draw telemetry overlay text on frame."""
    font = cv2.FONT_HERSHEY_DUPLEX
    color = (255, 255, 255)
    scale, thickness = 0.4, 1
    x_margin = 10
    
    # Use safe getters and coerce to numeric where appropriate so malformed
    # telemetry won't crash overlay rendering.
    def fnum(key, fmt, default=0):
        return fmt.format(float(data.get(key, default)))

    lines = [
        fnum('roll', 'Roll: {:.1f}'),
        fnum('pitch', 'Pitch: {:.1f}'),
        fnum('yaw', 'Yaw: {:.1f}'),
        fnum('lat', 'Lat: {:.6f}', default=0.0),
        fnum('lon', 'Lon: {:.6f}', default=0.0),
        fnum('alt', 'Alt: {:.1f}m', default=0.0),
        (f"Battery: {float(data.get('battery', 0)):.2f}V ({int(data.get('battery_remaining', 0))}%)"
         if data.get('battery') is not None else "Battery: N/A"),
        fnum('ground_speed', 'GS: {:.1f}m/s', default=0.0),
        fnum('throttle', 'Throttle: {:.0f}%', default=0),
    ]
    
    frame_width = frame.shape[1]
    for i, text in enumerate(lines):
        text_width = cv2.getTextSize(text, font, scale, thickness)[0][0]
        y = 15 + i * 20
        cv2.putText(frame, text, (frame_width - text_width - x_margin, y), font, scale, color, thickness)


def draw_fiducial_overlay(frame):
    if not FIDUCIAL_OVERLAY_ENABLED:
        return

    with fiducial_lock:
        payload = deepcopy(fiducial_state) if isinstance(fiducial_state, dict) else None

    if not payload:
        return

    timestamp = payload.get('timestamp', 0)
    if timestamp and (time.time() - float(timestamp)) > FIDUCIAL_STALE_TIMEOUT_SEC:
        return

    if not payload.get('locked'):
        return

    try:
        confidence = float(payload.get('confidence', 0))
    except Exception:
        confidence = 0
    if confidence < FIDUCIAL_MIN_CONFIDENCE:
        return

    h, w = frame.shape[:2]
    source_w = payload.get('frame_width')
    source_h = payload.get('frame_height')
    scale_x = 1.0
    scale_y = 1.0

    if FIDUCIAL_APPLY_FRAME_SCALING:
        try:
            sw = float(source_w)
            sh = float(source_h)
            if sw > 0 and sh > 0:
                scale_x = w / sw
                scale_y = h / sh
        except Exception:
            scale_x = 1.0
            scale_y = 1.0

    corners = payload.get('corners')
    cx = cy = None
    if isinstance(corners, list) and len(corners) >= 4:
        try:
            pts = np.array(corners, dtype=np.float32)
            if FIDUCIAL_APPLY_FRAME_SCALING:
                pts[:, 0] *= scale_x
                pts[:, 1] *= scale_y
            if FIDUCIAL_CLAMP_TO_FRAME:
                pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
                pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
            pts = pts.astype(np.int32)
            cv2.polylines(frame, [pts], True, FIDUCIAL_COLOR, FIDUCIAL_LINE_THICKNESS)
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
        except Exception:
            cx = cy = None

    if cx is None or cy is None:
        try:
            error_x = float(payload.get('error_x', 0))
            error_y = float(payload.get('error_y', 0))
            cx = int((w / 2) + error_x * (w / 2))
            cy = int((h / 2) + error_y * (h / 2))
            if FIDUCIAL_CLAMP_TO_FRAME:
                cx = int(np.clip(cx, 0, w - 1))
                cy = int(np.clip(cy, 0, h - 1))
        except Exception:
            return

    cv2.drawMarker(frame, (cx, cy), FIDUCIAL_COLOR, markerType=cv2.MARKER_CROSS, markerSize=14, thickness=FIDUCIAL_LINE_THICKNESS)

    fid_id = payload.get('fiducial_id')
    label = f"FID {fid_id}" if fid_id is not None else "FID"
    text = f"{label}  conf {confidence:.2f}"
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, FIDUCIAL_TEXT_SCALE, FIDUCIAL_COLOR, FIDUCIAL_TEXT_THICKNESS)
    return True


def draw_fiducial_overlay_from_result(frame, result):
    if result is None or not getattr(result, 'locked', False):
        return False

    try:
        confidence = float(getattr(result, 'confidence', 0.0))
    except Exception:
        confidence = 0.0
    if confidence < FIDUCIAL_MIN_CONFIDENCE:
        return False

    h, w = frame.shape[:2]
    corners = getattr(result, 'corners', None)
    cx = cy = None

    if corners is not None:
        try:
            pts = np.array(corners, dtype=np.float32)
            if FIDUCIAL_CLAMP_TO_FRAME:
                pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
                pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
            pts = pts.astype(np.int32)
            cv2.polylines(frame, [pts], True, FIDUCIAL_COLOR, FIDUCIAL_LINE_THICKNESS)
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
        except Exception:
            cx = cy = None

    if cx is None or cy is None:
        try:
            error_x = float(getattr(result, 'error_x', 0.0))
            error_y = float(getattr(result, 'error_y', 0.0))
            cx = int((w / 2) + error_x * (w / 2))
            cy = int((h / 2) + error_y * (h / 2))
            if FIDUCIAL_CLAMP_TO_FRAME:
                cx = int(np.clip(cx, 0, w - 1))
                cy = int(np.clip(cy, 0, h - 1))
        except Exception:
            return False

    cv2.drawMarker(frame, (cx, cy), FIDUCIAL_COLOR, markerType=cv2.MARKER_CROSS, markerSize=14, thickness=FIDUCIAL_LINE_THICKNESS)
    fid_id = getattr(result, 'fiducial_id', None)
    label = f"FID {fid_id}" if fid_id is not None else "FID"
    text = f"{label}  conf {confidence:.2f}"
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, FIDUCIAL_TEXT_SCALE, FIDUCIAL_COLOR, FIDUCIAL_TEXT_THICKNESS)
    return True


def get_request_token():
    header_token = request.headers.get('X-Session-Token')
    cookie_token = request.cookies.get('sentry_token')
    return header_token or cookie_token


def get_settings_for_request():
    token = get_request_token()
    if token and token in session_tokens:
        username = session_tokens[token]
        if username in settings_by_user:
            return deepcopy(settings_by_user[username])
    if token and token in settings_by_token:
        return deepcopy(settings_by_token[token])
    return deepcopy(DEFAULT_SETTINGS)


def save_settings_for_request(settings):
    token = get_request_token()
    if token and token in session_tokens:
        username = session_tokens[token]
        settings_by_user[username] = settings
        return
    if token:
        settings_by_token[token] = settings


def sanitize_settings(payload):
    settings = deepcopy(DEFAULT_SETTINGS)

    if isinstance(payload, dict):
        test_mode = payload.get('test_mode')
        if isinstance(test_mode, bool):
            settings['test_mode'] = test_mode

        latency_rate = payload.get('latency_polling_rate_ms')
        if isinstance(latency_rate, (int, float)):
            settings['latency_polling_rate_ms'] = int(max(200, min(5000, latency_rate)))

        status_rate = payload.get('status_update_interval_ms')
        if isinstance(status_rate, (int, float)):
            settings['status_update_interval_ms'] = int(max(2000, min(120000, status_rate)))

        visible_tiles = payload.get('visible_tiles')
        if isinstance(visible_tiles, dict):
            settings['visible_tiles'] = {
                tile_id: bool(visible_tiles.get(tile_id, True))
                for tile_id in ALLOWED_TILE_IDS
            }

        tile_order = payload.get('tile_order')
        if isinstance(tile_order, list):
            cleaned_order = []
            seen = set()
            for tile_id in tile_order:
                if tile_id in ALLOWED_TILE_IDS and tile_id not in seen:
                    cleaned_order.append(tile_id)
                    seen.add(tile_id)
            for tile_id in DEFAULT_TILE_ORDER:
                if tile_id not in seen:
                    cleaned_order.append(tile_id)
                    seen.add(tile_id)
            settings['tile_order'] = cleaned_order

        camera_tabs = payload.get('camera_function_tabs')
        if isinstance(camera_tabs, dict):
            cleaned_tabs = {}
            for cam_id, command_ids in camera_tabs.items():
                if cam_id not in DEFAULT_CAMERA_FUNCTION_TABS:
                    continue
                if not isinstance(command_ids, list):
                    continue
                cleaned = [cmd_id for cmd_id in command_ids if cmd_id in ALLOWED_COMMAND_IDS]
                cleaned_tabs[cam_id] = cleaned
            if cleaned_tabs:
                settings['camera_function_tabs'] = cleaned_tabs

    return settings


def get_hq_cv_mode():
    with hq_cv_mode_lock:
        return hq_cv_mode


def set_hq_cv_mode(mode):
    global hq_cv_mode, hq_tt_selected_track_id, hq_tt_selected_last_box, hq_tt_selected_missed_frames
    if mode not in HQ_CV_ALLOWED_MODES:
        return False
    with hq_cv_mode_lock:
        hq_cv_mode = mode
    if mode != HQ_CV_MODE_TARGET_TRACKING:
        with hq_tt_state_lock:
            hq_tt_selected_track_id = None
            hq_tt_selected_last_box = None
            hq_tt_selected_missed_frames = 0
    return True


def get_hq_target_selection_state():
    with hq_tt_state_lock:
        selected_id = hq_tt_selected_track_id
    return {
        'selected': selected_id is not None,
        'track_id': selected_id,
    }


def clear_hq_target_selection():
    global hq_tt_selected_track_id, hq_tt_selected_last_box, hq_tt_selected_missed_frames
    with hq_tt_state_lock:
        hq_tt_selected_track_id = None
        hq_tt_selected_last_box = None
        hq_tt_selected_missed_frames = 0


def _hq_tt_box_iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    if inter_area <= 0.0:
        return 0.0

    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = area_a + area_b - inter_area
    if denom <= 0.0:
        return 0.0
    return inter_area / denom


def _hq_tt_center_distance(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    acx = (ax1 + ax2) / 2.0
    acy = (ay1 + ay2) / 2.0
    bcx = (bx1 + bx2) / 2.0
    bcy = (by1 + by2) / 2.0
    dx = acx - bcx
    dy = acy - bcy
    return (dx * dx + dy * dy) ** 0.5


def select_hq_target_from_normalized(x_norm, y_norm):
    global hq_tt_selected_track_id, hq_tt_selected_last_box, hq_tt_selected_missed_frames

    try:
        x_norm = float(x_norm)
        y_norm = float(y_norm)
    except Exception:
        return False, 'Invalid click coordinates', None

    if not (0.0 <= x_norm <= 1.0 and 0.0 <= y_norm <= 1.0):
        return False, 'Click coordinates out of range', None

    with hq_tt_state_lock:
        detections = list(hq_tt_latest_detections)
        frame_w = int(hq_tt_latest_frame_width)
        frame_h = int(hq_tt_latest_frame_height)

    if frame_w <= 0 or frame_h <= 0:
        return False, 'No tracking frame available', None

    click_x = int(x_norm * frame_w)
    click_y = int(y_norm * frame_h)

    candidates = []
    for det in detections:
        track_id = det.get('track_id')
        box = det.get('box')
        if track_id is None or box is None or len(box) != 4:
            continue
        x1, y1, x2, y2 = [int(v) for v in box]
        if x1 <= click_x <= x2 and y1 <= click_y <= y2:
            area = max(1, (x2 - x1) * (y2 - y1))
            conf = float(det.get('conf', 0.0))
            candidates.append((area, -conf, int(track_id)))

    if not candidates:
        return False, 'No target at click point', None

    candidates.sort(key=lambda item: (item[0], item[1]))
    selected_id = candidates[0][2]

    selected_box = None
    for det in detections:
        if det.get('track_id') == selected_id:
            selected_box = det.get('box')
            break

    with hq_tt_state_lock:
        hq_tt_selected_track_id = selected_id
        hq_tt_selected_last_box = selected_box
        hq_tt_selected_missed_frames = 0

    return True, 'Target selected', selected_id


def _hq_tt_pick_device(device_arg):
    if device_arg != 'auto':
        return device_arg
    try:
        import torch
        if torch.cuda.is_available():
            return 'cuda'
        if hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            return 'mps'
    except Exception:
        pass
    return 'cpu'


def _hq_tt_normalize_names(model_names):
    if isinstance(model_names, list):
        return {idx: name for idx, name in enumerate(model_names)}
    normalized = {}
    for key, value in model_names.items():
        normalized[int(key)] = str(value)
    return normalized


def _hq_tt_should_prefer_coreml():
    return platform.system() == 'Darwin'


def _hq_tt_model_candidates():
    if _hq_tt_should_prefer_coreml():
        preferred = [
            HQ_TT_MODEL_COREML_PATH,
            HQ_TT_MODEL_COREML_FALLBACK_PATH,
            HQ_TT_MODEL_PATH,
            HQ_TT_MODEL_FALLBACK_PATH,
        ]
    else:
        preferred = [
            HQ_TT_MODEL_PATH,
            HQ_TT_MODEL_FALLBACK_PATH,
            HQ_TT_MODEL_COREML_PATH,
            HQ_TT_MODEL_COREML_FALLBACK_PATH,
        ]

    candidates = []
    seen = set()
    for model_path in preferred:
        if not model_path:
            continue
        if model_path in seen:
            continue
        seen.add(model_path)
        if os.path.exists(model_path):
            candidates.append(model_path)
    return candidates


def _hq_tt_resolve_model_path():
    candidates = _hq_tt_model_candidates()
    return candidates[0] if candidates else None


def _hq_tt_resolve_tracker_path():
    if HQ_TT_TRACKER_PATH and os.path.exists(HQ_TT_TRACKER_PATH):
        return HQ_TT_TRACKER_PATH
    return HQ_TT_TRACKER_PATH


def _hq_tt_people_related_class_ids(names_by_id):
    people_related_exact = {
        'person', 'pedestrian', 'people', 'bicycle', 'cyclist', 'rider',
        'motor', 'motorbike', 'motorcycle', 'tricycle', 'awning-tricycle'
    }
    people_related_keywords = ('person', 'pedestrian', 'people', 'rider', 'cyclist', 'bicycle', 'bike', 'motor', 'tricycle', 'human')
    action_keywords = ('walk', 'run', 'sit', 'stand', 'ride', 'jump', 'fall', 'action')

    selected = set()
    for idx, name in names_by_id.items():
        lowered = str(name).lower()
        if lowered in people_related_exact:
            selected.add(idx)
            continue
        if any(keyword in lowered for keyword in people_related_keywords):
            selected.add(idx)
            continue
        if any(keyword in lowered for keyword in action_keywords):
            selected.add(idx)
    return sorted(selected)


def _hq_tt_build_runtime():
    global hq_tt_model, hq_tt_error, hq_tt_class_filter
    if hq_tt_model is not None:
        return True

    with hq_tt_model_lock:
        if hq_tt_model is not None:
            return True
        try:
            from ultralytics import YOLO

            model_candidates = _hq_tt_model_candidates()
            if not model_candidates:
                hq_tt_error = 'Target Tracking model not found.'
                return False

            last_error = None
            loaded_model_path = None
            for model_path in model_candidates:
                try:
                    hq_tt_model = YOLO(model_path)
                    loaded_model_path = model_path
                    break
                except Exception as model_error:
                    last_error = model_error
                    hq_tt_model = None

            if hq_tt_model is None:
                hq_tt_error = f'Target Tracking init failed: {last_error}'
                print(hq_tt_error)
                return False

            if HQ_TT_PERSON_ONLY:
                try:
                    names_by_id = _hq_tt_normalize_names(hq_tt_model.names)
                    class_ids = _hq_tt_people_related_class_ids(names_by_id)
                    hq_tt_class_filter = class_ids if class_ids else None
                except Exception:
                    hq_tt_class_filter = None
            else:
                hq_tt_class_filter = None

            hq_tt_error = None
            print(f"HQ Target Tracking initialized with model: {loaded_model_path}")
            return True
        except Exception as e:
            hq_tt_model = None
            hq_tt_error = f'Target Tracking init failed: {e}'
            print(hq_tt_error)
            return False


def _hq_tt_draw_status(frame, text, color):
    cv2.putText(
        frame,
        text,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
    )


def _hq_tt_extract_detections(result):
    detections = []
    if result is None or not hasattr(result, 'boxes') or result.boxes is None:
        return detections

    boxes = result.boxes
    xyxy = boxes.xyxy
    confs = boxes.conf if hasattr(boxes, 'conf') else None
    classes = boxes.cls if hasattr(boxes, 'cls') else None
    ids = boxes.id if hasattr(boxes, 'id') else None

    if xyxy is None:
        return detections

    try:
        boxes_np = xyxy.cpu().numpy()
    except Exception:
        boxes_np = np.array(xyxy)

    conf_np = None
    if confs is not None:
        try:
            conf_np = confs.cpu().numpy()
        except Exception:
            conf_np = np.array(confs)

    cls_np = None
    if classes is not None:
        try:
            cls_np = classes.cpu().numpy()
        except Exception:
            cls_np = np.array(classes)

    id_np = None
    if ids is not None:
        try:
            id_np = ids.cpu().numpy()
        except Exception:
            id_np = np.array(ids)

    for i, box in enumerate(boxes_np):
        x1, y1, x2, y2 = [int(v) for v in box[:4]]
        conf = float(conf_np[i]) if conf_np is not None and i < len(conf_np) else 0.0
        cls_id = int(cls_np[i]) if cls_np is not None and i < len(cls_np) else -1
        track_id = int(id_np[i]) if id_np is not None and i < len(id_np) else None
        detections.append({
            'box': (x1, y1, x2, y2),
            'conf': conf,
            'cls_id': cls_id,
            'track_id': track_id,
        })

    return detections


def _hq_tt_fallback_detect_and_draw(frame):
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = max(80, int(w * h * HQ_TT_FALLBACK_MIN_AREA_RATIO))
    detections = []
    for idx, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        detections.append({
            'box': (x, y, x + bw, y + bh),
            'conf': 0.5,
            'cls_id': -1,
            'track_id': idx + 1,
        })

    with hq_tt_state_lock:
        global hq_tt_latest_detections, hq_tt_latest_frame_width, hq_tt_latest_frame_height, hq_tt_selected_track_id
        hq_tt_latest_detections = detections
        hq_tt_latest_frame_width = w
        hq_tt_latest_frame_height = h
        selected_id = hq_tt_selected_track_id

    target_det = None
    if selected_id is not None:
        for det in detections:
            if det.get('track_id') == selected_id:
                target_det = det
                break
        if target_det is None:
            clear_hq_target_selection()

    cx, cy = w // 2, h // 2
    cv2.drawMarker(frame, (cx, cy), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=1)

    for det in detections:
        x1, y1, x2, y2 = det['box']
        is_target = target_det is not None and det.get('track_id') == target_det.get('track_id')
        color = HQ_TT_TARGET_COLOR if is_target else HQ_TT_BOX_COLOR
        thickness = 3 if is_target else 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(frame, 'Fallback target', (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, HQ_TT_TEXT_COLOR, 1)

    if target_det is not None:
        tx1, ty1, tx2, ty2 = target_det['box']
        tx, ty = int((tx1 + tx2) / 2), int((ty1 + ty2) / 2)
        cv2.line(frame, (cx, cy), (tx, ty), HQ_TT_TARGET_COLOR, 2)
        _hq_tt_draw_status(frame, f'Target Tracking: LOCKED (ID {target_det.get("track_id")}) (fallback)', HQ_TT_TARGET_COLOR)
    else:
        _hq_tt_draw_status(frame, 'Target Tracking: CLICK TARGET TO LOCK (fallback)', HQ_TT_BOX_COLOR)


def apply_hq_cv_landing_mode(frame):
    drawn = bool(draw_fiducial_overlay(frame))

    if (not drawn and TEST_MODE and HQ_LANDING_LOCAL_FIDUCIAL_FALLBACK_IN_TEST and process_fiducial_frame is not None):
        try:
            local_result = process_fiducial_frame(frame)
            drawn = bool(draw_fiducial_overlay_from_result(frame, local_result))
        except Exception:
            drawn = False

    if drawn:
        _hq_tt_draw_status(frame, 'Landing Mode: LOCKED', HQ_LANDING_STATUS_TEXT_COLOR)
    else:
        _hq_tt_draw_status(frame, 'Landing Mode: SEARCHING', HQ_LANDING_STATUS_TEXT_COLOR)


def apply_hq_cv_target_tracking_mode(frame):
    if not _hq_tt_build_runtime():
        if TEST_MODE and HQ_TT_FALLBACK_IN_TEST:
            _hq_tt_fallback_detect_and_draw(frame)
            return
        _hq_tt_draw_status(frame, hq_tt_error or HQ_CV_TARGET_TRACKING_TEXT, (0, 0, 255))
        return

    with hq_tt_infer_lock:
        try:
            device = _hq_tt_pick_device(HQ_TT_DEVICE)
            result_list = hq_tt_model.track(
                source=frame,
                conf=HQ_TT_CONFIDENCE,
                iou=HQ_TT_IOU,
                imgsz=HQ_TT_IMAGE_SIZE,
                tracker=_hq_tt_resolve_tracker_path(),
                persist=HQ_TT_PERSIST,
                rect=False,
                device=device,
                classes=hq_tt_class_filter,
                verbose=False,
            )
            result = result_list[0] if result_list else None
            detections = _hq_tt_extract_detections(result)
        except Exception as e:
            _hq_tt_draw_status(frame, f'Target Tracking error: {e}', (0, 0, 255))
            return

    h, w = frame.shape[:2]
    with hq_tt_state_lock:
        global hq_tt_latest_detections, hq_tt_latest_frame_width, hq_tt_latest_frame_height
        global hq_tt_selected_track_id, hq_tt_selected_last_box, hq_tt_selected_missed_frames
        hq_tt_latest_detections = detections
        hq_tt_latest_frame_width = w
        hq_tt_latest_frame_height = h
        selected_id = hq_tt_selected_track_id
        selected_last_box = hq_tt_selected_last_box
        selected_missed_frames = hq_tt_selected_missed_frames

    by_track_id = {
        d['track_id']: d
        for d in detections
        if d.get('track_id') is not None
    }

    target_det = None
    if selected_id is not None:
        target_det = by_track_id.get(selected_id)
        if target_det is not None and target_det.get('conf', 0.0) >= HQ_TT_STICK_MIN_CONF:
            selected_last_box = target_det.get('box')
            selected_missed_frames = 0
        else:
            target_det = None
            # Keep holding/looking in last known area, but never reassign to a different ID.
            selected_missed_frames += 1

            selected_hold_limit = max(1, int(HQ_TT_HOLD_FRAMES * HQ_TT_SELECTED_HOLD_MULTIPLIER))
            if target_det is None and selected_missed_frames > selected_hold_limit:
                selected_id = None
                selected_last_box = None
                selected_missed_frames = 0

    with hq_tt_state_lock:
        hq_tt_selected_track_id = selected_id
        hq_tt_selected_last_box = selected_last_box
        hq_tt_selected_missed_frames = selected_missed_frames

    names_by_id = None
    try:
        names_by_id = _hq_tt_normalize_names(hq_tt_model.names)
    except Exception:
        names_by_id = None

    for det in detections:
        x1, y1, x2, y2 = det['box']
        is_target = target_det is not None and det.get('track_id') == target_det.get('track_id')
        color = HQ_TT_TARGET_COLOR if is_target else HQ_TT_BOX_COLOR
        thickness = 3 if is_target else 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

        label_parts = []
        if det.get('track_id') is not None:
            label_parts.append(f"ID {det['track_id']}")
        if names_by_id and det['cls_id'] in names_by_id:
            label_parts.append(names_by_id[det['cls_id']])
        label_parts.append(f"{det['conf']:.2f}")
        label = ' | '.join(label_parts)
        cv2.putText(frame, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, HQ_TT_TEXT_COLOR, 1)

    cx, cy = w // 2, h // 2
    cv2.drawMarker(frame, (cx, cy), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=1)

    if target_det is not None:
        tx1, ty1, tx2, ty2 = target_det['box']
        tx = int((tx1 + tx2) / 2)
        ty = int((ty1 + ty2) / 2)
        cv2.line(frame, (cx, cy), (tx, ty), HQ_TT_TARGET_COLOR, 2)
        _hq_tt_draw_status(frame, f"Target Tracking: LOCKED (ID {target_det.get('track_id')})", HQ_TT_TARGET_COLOR)
    else:
        if selected_id is not None and selected_last_box is not None:
            x1, y1, x2, y2 = [int(v) for v in selected_last_box]
            cv2.rectangle(frame, (x1, y1), (x2, y2), HQ_TT_TARGET_COLOR, 2)
            _hq_tt_draw_status(
                frame,
                f'Target Tracking: HOLDING AREA (ID {selected_id}) [{selected_missed_frames}/{max(1, int(HQ_TT_HOLD_FRAMES * HQ_TT_SELECTED_HOLD_MULTIPLIER))}]',
                HQ_TT_TARGET_COLOR,
            )
        else:
            _hq_tt_draw_status(frame, 'Target Tracking: CLICK TARGET TO LOCK', HQ_TT_BOX_COLOR)


# ===== Video Receivers =====
async def receive_video(cam_id, port):
    """Generic video receiver for a camera."""
    global frame_cam0, frame_cam1, frame_hq, last_cam0_time, last_cam1_time, last_hq_time
    global video_latency_cam0_ms, video_latency_cam1_ms, video_latency_hq_ms
    global cam0_frame_timestamp, cam1_frame_timestamp, hq_frame_timestamp
    
    while True:
        try:
            async with websockets.connect(f'ws://{SERVER_IP}:{port}') as ws:
                print(f"Connected to video server (cam{cam_id}).")
                while True:
                    data = await ws.recv()
                    receive_time = time.time()
                    frame, timestamp = decode_video_frame(data)
                    
                    # Store frame along with timing info so we can compute network + render latency later
                    if cam_id == 0:
                        network_ms = (receive_time - timestamp) * 1000 if timestamp else None
                        frame_cam0 = {
                            'frame': frame,
                            'server_ts': timestamp,
                            'receive_time': receive_time,
                            'network_ms': network_ms
                        }
                        last_cam0_time = receive_time
                        if timestamp:
                            cam0_frame_timestamp = timestamp
                    elif cam_id == 1:
                        network_ms = (receive_time - timestamp) * 1000 if timestamp else None
                        frame_cam1 = {
                            'frame': frame,
                            'server_ts': timestamp,
                            'receive_time': receive_time,
                            'network_ms': network_ms
                        }
                        last_cam1_time = receive_time
                        if timestamp:
                            cam1_frame_timestamp = timestamp
                    else:
                        network_ms = (receive_time - timestamp) * 1000 if timestamp else None
                        frame_hq = {
                            'frame': frame,
                            'server_ts': timestamp,
                            'receive_time': receive_time,
                            'network_ms': network_ms
                        }
                        last_hq_time = receive_time
                        if timestamp:
                            hq_frame_timestamp = timestamp
        except websockets.exceptions.ConnectionClosed:
            print(f"Camera {cam_id} disconnected, reconnecting in {VIDEO_RECONNECT_DELAY_SEC:.0f} seconds...")
        except (ConnectionRefusedError, OSError):
            print(f"Camera {cam_id} connect failed, reconnecting in {VIDEO_RECONNECT_DELAY_SEC:.0f} seconds...")
        except Exception:
            print(f"Camera {cam_id} reconnecting in {VIDEO_RECONNECT_DELAY_SEC:.0f} seconds...")
        await asyncio.sleep(VIDEO_RECONNECT_DELAY_SEC)


async def receive_telemetry():
    """Connect to telemetry WebSocket and update mavlink_data in real-time."""
    global mavlink_data, telemetry_latency_ms, last_mav_time
    
    while True:
        try:
            async with websockets.connect(
                f'ws://{SERVER_IP}:{TELEMETRY_PORT}',
                ping_interval=TELEMETRY_PING_INTERVAL_SEC,
                ping_timeout=TELEMETRY_PING_TIMEOUT_SEC
            ) as ws:
                print("Connected to telemetry server (WebSocket).")
                while True:
                    try:
                        data = await asyncio.wait_for(ws.recv(), timeout=TELEMETRY_RECV_TIMEOUT_SEC)
                    except asyncio.TimeoutError:
                        print("Telemetry receive timeout, reconnecting...")
                        await ws.close()
                        break
                    try:
                        telemetry_msg = json.loads(data)
                        current_time = time.time()
                        # server may send either 'server_time' or 'timestamp'
                        server_time = telemetry_msg.get('server_time', telemetry_msg.get('timestamp', current_time))
                        try:
                            telemetry_latency_ms = (current_time - float(server_time)) * 1000
                        except Exception:
                            telemetry_latency_ms = 0

                        # Merge incoming telemetry into the existing dict instead
                        # of replacing the object. This preserves any code that
                        # holds references to `mavlink_data` and avoids races.
                        if isinstance(telemetry_msg, dict):
                            mavlink_data.update(telemetry_msg)
                        last_mav_time = current_time
                    except json.JSONDecodeError:
                        print(f"Failed to parse telemetry JSON: {data}")
        except (ConnectionRefusedError, OSError):
            print(f"Telemetry connect failed, reconnecting in {TELEMETRY_RECONNECT_DELAY_SEC:.0f} seconds...")
        except Exception:
            print(f"Telemetry reconnecting in {TELEMETRY_RECONNECT_DELAY_SEC:.0f} seconds...")
        await asyncio.sleep(TELEMETRY_RECONNECT_DELAY_SEC)


async def receive_fiducials():
    """Connect to fiducial WebSocket and update latest fiducial state."""
    global fiducial_state

    while True:
        try:
            async with websockets.connect(f'ws://{SERVER_IP}:{FIDUCIAL_PORT}') as ws:
                print("Connected to fiducial server (WebSocket).")
                async for message in ws:
                    try:
                        payload = json.loads(message)
                        if isinstance(payload, dict):
                            with fiducial_lock:
                                fiducial_state = payload
                    except json.JSONDecodeError:
                        pass
        except (ConnectionRefusedError, OSError):
            print(f"Fiducial connect failed, reconnecting in {FIDUCIAL_RECONNECT_DELAY_SEC:.0f} seconds...")
        except Exception:
            print(f"Fiducial reconnecting in {FIDUCIAL_RECONNECT_DELAY_SEC:.0f} seconds...")
        await asyncio.sleep(FIDUCIAL_RECONNECT_DELAY_SEC)


def receive_mavlink():
    """Legacy UDP receiver for backward compatibility."""
    global mavlink_data, last_mav_time
    
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            values = data.decode().split(',')
            mavlink_data = {
                "roll": float(values[0]), "pitch": float(values[1]), "yaw": float(values[2]),
                "lat": float(values[3]), "lon": float(values[4]), "alt": float(values[5]),
                "battery": float(values[6]), "battery_remaining": float(values[7]),
                "ground_speed": float(values[8]), "throttle": float(values[9]),
                "timestamp": time.time(), "server_latency_ms": 0
            }
            last_mav_time = time.time()
        except:
            pass


def status_monitor():
    """Background thread to update connection status."""
    while True:
        update_connection_status()
        time.sleep(1)


# ===== Frame Generators =====
def gen_frames_cam0():
    global video_latency_cam0_ms, cam0_processing_latency_ms, cam0_frame_timestamp
    while True:
        if frame_cam0 is None:
            continue
        entry = frame_cam0
        f = entry['frame'].copy()
        h, w = f.shape[:2]
        
        # Start timing backend processing
        processing_start = time.time()
        
        # Draw overlays
        try:
            draw_compass(f, mavlink_data.get('yaw', 0), 0, h - 130, 120)
            draw_attitude_indicator(f, mavlink_data.get('roll', 0), mavlink_data.get('pitch', 0), x=w - 130, y=h - 130, size=120)
            draw_battery_widget(f, mavlink_data.get('battery_remaining', 0), position=(10, 10), width=60, height=20)
            draw_throttle_widget(f, mavlink_data.get('throttle', 0), position=(10, 40), width=60, height=20)
            draw_telemetry_text(f, mavlink_data)
        except Exception as e:
            # Don't let telemetry/overlay errors break the video stream; log and continue
            print(f"Overlay drawing error (cam0): {e}")
        
        # Encode to JPEG
        _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        
        # Compute backend processing latency (overlays + encoding)
        processing_time = time.time()
        processing_ms = (processing_time - processing_start) * 1000
        network_ms = entry.get('network_ms')
        
        # Record samples: (network_latency, backend_processing_latency)
        cam0_latency_samples.append((network_ms, processing_ms))

        # Update median stats for network latency
        nets = [n for n, r in cam0_latency_samples if n is not None]
        procs = [r for n, r in cam0_latency_samples if r is not None]
        if procs:
            cam0_processing_latency_ms = statistics.median(procs)
        else:
            cam0_processing_latency_ms = 0
        if nets:
            video_latency_cam0_ms = statistics.median(nets)
        else:
            video_latency_cam0_ms = 0

        # Expose latest frame timestamp
        try:
            cam0_frame_timestamp = entry.get('server_ts') or cam0_frame_timestamp
        except NameError:
            cam0_frame_timestamp = entry.get('server_ts')

        # Store latest JPEG bytes and per-frame metadata for one-shot snapshot endpoint
        try:
            entry['last_jpeg'] = jpeg.tobytes()
            entry['last_processing_ms'] = processing_ms
            entry['last_server_ts'] = entry.get('server_ts')
            # preserve the network latency computed when frame was received via websocket
            entry['last_network_ms'] = entry.get('network_ms')
        except Exception:
            pass

        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + entry['last_jpeg'] + b'\r\n'


def gen_frames_cam1():
    global video_latency_cam1_ms, cam1_processing_latency_ms, cam1_frame_timestamp
    while True:
        if frame_cam1 is None:
            continue
        entry = frame_cam1
        f = entry['frame'].copy()

        # Start timing backend processing
        processing_start = time.time()
        
        # Encode to JPEG (no overlays for cam1)
        _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        
        # Compute backend processing latency
        processing_time = time.time()
        processing_ms = (processing_time - processing_start) * 1000
        network_ms = entry.get('network_ms')
        
        # Record samples
        cam1_latency_samples.append((network_ms, processing_ms))

        # Update median stats
        nets = [n for n, r in cam1_latency_samples if n is not None]
        procs = [r for n, r in cam1_latency_samples if r is not None]
        if procs:
            cam1_processing_latency_ms = statistics.median(procs)
        else:
            cam1_processing_latency_ms = 0
        if nets:
            video_latency_cam1_ms = statistics.median(nets)
        else:
            video_latency_cam1_ms = 0

        try:
            cam1_frame_timestamp = entry.get('server_ts') or cam1_frame_timestamp
        except NameError:
            cam1_frame_timestamp = entry.get('server_ts')

        # Store latest JPEG bytes and per-frame metadata for one-shot snapshot endpoint
        try:
            entry['last_jpeg'] = jpeg.tobytes()
            entry['last_processing_ms'] = processing_ms
            entry['last_server_ts'] = entry.get('server_ts')
            entry['last_network_ms'] = entry.get('network_ms')
        except Exception:
            pass

        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + entry['last_jpeg'] + b'\r\n'


def gen_frames_hq():
    global video_latency_hq_ms, hq_processing_latency_ms, hq_frame_timestamp
    while True:
        if frame_hq is None:
            continue
        entry = frame_hq
        f = entry['frame'].copy()

        # Start timing backend processing
        processing_start = time.time()

        try:
            mode = get_hq_cv_mode()
            if mode == HQ_CV_MODE_LANDING:
                apply_hq_cv_landing_mode(f)
            elif mode == HQ_CV_MODE_TARGET_TRACKING:
                apply_hq_cv_target_tracking_mode(f)
        except Exception as e:
            print(f"Overlay drawing error (hq): {e}")

        _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 90])

        # Compute backend processing latency
        processing_time = time.time()
        processing_ms = (processing_time - processing_start) * 1000
        network_ms = entry.get('network_ms')

        # Record samples
        hq_latency_samples.append((network_ms, processing_ms))

        # Update median stats
        nets = [n for n, r in hq_latency_samples if n is not None]
        procs = [r for n, r in hq_latency_samples if r is not None]
        if procs:
            hq_processing_latency_ms = statistics.median(procs)
        else:
            hq_processing_latency_ms = 0
        if nets:
            video_latency_hq_ms = statistics.median(nets)
        else:
            video_latency_hq_ms = 0

        try:
            hq_frame_timestamp = entry.get('server_ts') or hq_frame_timestamp
        except NameError:
            hq_frame_timestamp = entry.get('server_ts')

        try:
            entry['last_jpeg'] = jpeg.tobytes()
            entry['last_processing_ms'] = processing_ms
            entry['last_server_ts'] = entry.get('server_ts')
            entry['last_network_ms'] = entry.get('network_ms')
        except Exception:
            pass

        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + entry['last_jpeg'] + b'\r\n'


# ===== Flask Routes =====
@app.route('/')
def index():
    update_connection_status()
    user_settings = get_settings_for_request()
    effective_test_mode = bool(user_settings.get('test_mode')) or TEST_MODE
    return render_template(
        'index.html',
        main_online=is_main_online(),
        cam0_online=cam0_online,
        cam1_online=cam1_online,
        hq_online=hq_online,
        map_online=is_map_online(),
        map_html=generate_map_html(
            mavlink_data['lat'],
            mavlink_data['lon'],
            mavlink_data['yaw'],
            test_mode=effective_test_mode
        ),
        status_update_interval_ms=user_settings.get('status_update_interval_ms', STATUS_UPDATE_INTERVAL_MS),
        camera_function_tabs=user_settings.get('camera_function_tabs', DEFAULT_CAMERA_FUNCTION_TABS),
        user_settings=user_settings
    )


@app.route('/video_feed_cam0')
def video_feed_cam0():
    return Response(gen_frames_cam0(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed_cam1')
def video_feed_cam1():
    return Response(gen_frames_cam1(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed_hq')
def video_feed_hq():
    return Response(gen_frames_hq(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/telemetry')
def telemetry():
    update_connection_status()
    result = mavlink_data.copy()
    result['client_latency_ms'] = telemetry_latency_ms
    return jsonify(result)


@app.route('/api/status')
def api_status():
    update_connection_status()
    return jsonify({
        'main_online': is_main_online(),
        'cam0_online': cam0_online,
        'cam1_online': cam1_online,
        'mav_online': mav_online,
        'map_online': is_map_online(),
        'hq_online': hq_online
    })


@app.route('/api/fiducial')
def api_fiducial():
    with fiducial_lock:
        payload = deepcopy(fiducial_state) if isinstance(fiducial_state, dict) else None
    return jsonify(payload or {'locked': False})


@app.route('/api/video_latency')
def api_video_latency():
    """Return video latency metrics.
    
    Returns:
        - network_latency_ms: Time from server capture to client receipt
        - processing_latency_ms: Gateway processing (overlays + JPEG encoding)
        - render_latency_ms: Frontend rendering (measured by browser)
        - Total latency = network + processing + render
    """
    return jsonify({
        'cam0_network_latency_ms': round(video_latency_cam0_ms, 2),
        'cam0_processing_latency_ms': round(cam0_processing_latency_ms, 2),
        'cam0_render_latency_ms': 0,  # Placeholder - measured by frontend
        'cam1_network_latency_ms': round(video_latency_cam1_ms, 2),
        'cam1_processing_latency_ms': round(cam1_processing_latency_ms, 2),
        'cam1_render_latency_ms': 0,  # Placeholder - measured by frontend
        'hq_network_latency_ms': round(video_latency_hq_ms, 2),
        'hq_processing_latency_ms': round(hq_processing_latency_ms, 2),
        'hq_render_latency_ms': 0,  # Placeholder - measured by frontend
        'cam0_frame_timestamp': cam0_frame_timestamp,
        'cam1_frame_timestamp': cam1_frame_timestamp,
        'hq_frame_timestamp': hq_frame_timestamp,
        'cam0_samples': len(cam0_latency_samples),
        'cam1_samples': len(cam1_latency_samples),
        'hq_samples': len(hq_latency_samples),
        'cam0_online': cam0_online,
        'cam1_online': cam1_online,
        'hq_online': hq_online,
        'server_time': time.time()
    })


@app.route('/api/authenticate', methods=['POST'])
def authenticate():
    data = request.json
    username = data.get('username', '').strip()
    password = data.get('password', '').strip()
    
    if username == VALID_USERNAME and password == VALID_PASSWORD:
        token = secrets.token_hex(32)
        session_tokens[token] = username
        response = make_response(jsonify({'success': True, 'token': token}))
        response.set_cookie(
            'sentry_token',
            token,
            httponly=True,
            samesite='Lax'
        )
        return response
    return jsonify({'success': False, 'message': 'Invalid credentials'}), 401


@app.route('/api/logout', methods=['POST'])
def logout():
    token = get_request_token()
    if token and token in session_tokens:
        session_tokens.pop(token, None)
    response = make_response(jsonify({'success': True}))
    response.set_cookie('sentry_token', '', expires=0)
    return response


@app.route('/api/settings', methods=['GET', 'POST'])
def api_settings():
    if request.method == 'GET':
        return jsonify(get_settings_for_request())

    payload = request.json or {}
    settings = sanitize_settings(payload)
    save_settings_for_request(settings)
    return jsonify({'success': True, 'settings': settings})


async def receive_commands():
    """Connect to the server's command WebSocket and keep command_state in sync."""
    global command_state, command_ws

    while True:
        try:
            async with websockets.connect(f'ws://{SERVER_IP}:{COMMAND_PORT}') as ws:
                command_ws = ws
                print("Connected to command WebSocket.")
                async for message in ws:
                    try:
                        msg = json.loads(message)
                        if msg.get('type') == 'state' and 'commands' in msg:
                            command_state.update(msg['commands'])
                    except json.JSONDecodeError:
                        pass
        except (ConnectionRefusedError, OSError):
            print(f"Command connect failed, reconnecting in {COMMAND_RECONNECT_DELAY_SEC:.0f} seconds...")
        except Exception:
            print(f"Command reconnecting in {COMMAND_RECONNECT_DELAY_SEC:.0f} seconds...")
        finally:
            command_ws = None
        await asyncio.sleep(COMMAND_RECONNECT_DELAY_SEC)


@app.route('/api/commands')
def api_commands():
    """Return current command states."""
    return jsonify(command_state)


@app.route('/api/command', methods=['POST'])
def api_command_toggle():
    """Toggle a single command and forward to server via WebSocket."""
    data = request.json
    cmd_id = data.get('id')
    value = data.get('value')
    is_pulse = data.get('pulse', False)
    
    if is_pulse:
        print(f"[Flask] Received pulse command: '{cmd_id}'")
    else:
        print(f"[Flask] Received command toggle: '{cmd_id}' -> {value}")
    
    if cmd_id not in command_state:
        return jsonify({'success': False, 'message': 'Unknown command'}), 400
    
    # For pulse commands, don't update local state
    if not is_pulse:
        command_state[cmd_id] = bool(value)
    
    # Forward to server via the persistent WebSocket
    if command_ws:
        try:
            msg = {'type': 'pulse' if is_pulse else 'toggle', 'id': cmd_id, 'value': bool(value)}
            print(f"[Flask] Forwarding to server: {msg}")
            asyncio.run_coroutine_threadsafe(
                command_ws.send(json.dumps(msg)),
                command_loop
            )
        except Exception as e:
            print(f"Failed to forward command to server: {e}")
    else:
        print("[Flask] Warning: command_ws is None, cannot forward to server")
    return jsonify({'success': True, 'commands': command_state})


@app.route('/api/guided_goto', methods=['POST'])
def api_guided_goto():
    """Forward a guided goto request (lat/lon[/alt]) to the server command socket."""
    data = request.json or {}

    try:
        lat = float(data.get('lat'))
        lon = float(data.get('lon'))
    except Exception:
        return jsonify({'success': False, 'message': 'Invalid coordinates'}), 400

    if not (GUIDED_GOTO_LAT_MIN <= lat <= GUIDED_GOTO_LAT_MAX):
        return jsonify({'success': False, 'message': 'Latitude out of range'}), 400
    if not (GUIDED_GOTO_LON_MIN <= lon <= GUIDED_GOTO_LON_MAX):
        return jsonify({'success': False, 'message': 'Longitude out of range'}), 400

    alt = data.get('alt')
    if alt is not None:
        try:
            alt = float(alt)
        except Exception:
            return jsonify({'success': False, 'message': 'Invalid altitude'}), 400
        alt = max(GUIDED_GOTO_MIN_ALT_M, min(GUIDED_GOTO_MAX_ALT_M, alt))

    if not command_ws:
        return jsonify({'success': False, 'message': 'Command link unavailable'}), 503

    msg = {
        'type': 'guided_goto',
        'lat': lat,
        'lon': lon,
    }
    if alt is not None:
        msg['alt'] = alt

    try:
        asyncio.run_coroutine_threadsafe(
            command_ws.send(json.dumps(msg)),
            command_loop
        )
    except Exception as e:
        print(f"Failed to forward guided goto to server: {e}")
        return jsonify({'success': False, 'message': 'Failed to send guided command'}), 500

    return jsonify({'success': True, 'lat': lat, 'lon': lon, 'alt': alt})


@app.route('/api/snapshot_cam0')
def api_snapshot_cam0():
    """Return the latest JPEG snapshot for cam0 (one-shot)."""
    if frame_cam0 and isinstance(frame_cam0, dict) and frame_cam0.get('last_jpeg'):
        headers = {}
        # include per-frame metadata if available
        try:
            if frame_cam0.get('last_server_ts'):
                headers['X-Frame-Ts'] = str(frame_cam0.get('last_server_ts'))
            if frame_cam0.get('last_processing_ms') is not None:
                headers['X-Processing-Ms'] = str(frame_cam0.get('last_processing_ms'))
            if frame_cam0.get('last_network_ms') is not None:
                headers['X-Network-Ms'] = str(frame_cam0.get('last_network_ms'))
        except Exception:
            pass
        return Response(frame_cam0['last_jpeg'], mimetype='image/jpeg', headers=headers)
    return ('', 204)


@app.route('/api/snapshot_cam1')
def api_snapshot_cam1():
    """Return the latest JPEG snapshot for cam1 (one-shot)."""
    if frame_cam1 and isinstance(frame_cam1, dict) and frame_cam1.get('last_jpeg'):
        headers = {}
        try:
            if frame_cam1.get('last_server_ts'):
                headers['X-Frame-Ts'] = str(frame_cam1.get('last_server_ts'))
            if frame_cam1.get('last_processing_ms') is not None:
                headers['X-Processing-Ms'] = str(frame_cam1.get('last_processing_ms'))
            if frame_cam1.get('last_network_ms') is not None:
                headers['X-Network-Ms'] = str(frame_cam1.get('last_network_ms'))
        except Exception:
            pass
        return Response(frame_cam1['last_jpeg'], mimetype='image/jpeg', headers=headers)
    return ('', 204)


@app.route('/api/snapshot_hq')
def api_snapshot_hq():
    """Return the latest JPEG snapshot for HQ camera (one-shot)."""
    if frame_hq and isinstance(frame_hq, dict) and frame_hq.get('last_jpeg'):
        headers = {}
        try:
            if frame_hq.get('last_server_ts'):
                headers['X-Frame-Ts'] = str(frame_hq.get('last_server_ts'))
            if frame_hq.get('last_processing_ms') is not None:
                headers['X-Processing-Ms'] = str(frame_hq.get('last_processing_ms'))
            if frame_hq.get('last_network_ms') is not None:
                headers['X-Network-Ms'] = str(frame_hq.get('last_network_ms'))
        except Exception:
            pass
        return Response(frame_hq['last_jpeg'], mimetype='image/jpeg', headers=headers)
    return ('', 204)


@app.route('/api/hq_cv_mode', methods=['GET', 'POST'])
def api_hq_cv_mode():
    if request.method == 'GET':
        return jsonify({'mode': get_hq_cv_mode()})

    payload = request.json or {}
    mode = payload.get('mode')
    if not isinstance(mode, str) or mode not in HQ_CV_ALLOWED_MODES:
        return jsonify({'success': False, 'message': 'Invalid HQ CV mode'}), 400

    set_hq_cv_mode(mode)
    return jsonify({'success': True, 'mode': get_hq_cv_mode()})


@app.route('/api/hq_target_selection', methods=['GET', 'POST'])
def api_hq_target_selection():
    if request.method == 'GET':
        return jsonify(get_hq_target_selection_state())

    payload = request.json or {}
    action = payload.get('action', 'select')

    if action == 'clear':
        clear_hq_target_selection()
        state = get_hq_target_selection_state()
        return jsonify({'success': True, **state})

    x_norm = payload.get('x_norm')
    y_norm = payload.get('y_norm')
    success, message, track_id = select_hq_target_from_normalized(x_norm, y_norm)
    state = get_hq_target_selection_state()
    status_code = 200 if success else 400
    return jsonify({
        'success': success,
        'message': message,
        'track_id': track_id,
        **state,
    }), status_code


@app.route('/map_embed')
def map_embed():
    """Return a full standalone map HTML page for embedding in an iframe."""
    user_settings = get_settings_for_request()
    html = generate_map_html(
        mavlink_data['lat'],
        mavlink_data['lon'],
        mavlink_data['yaw'],
        test_mode=bool(user_settings.get('test_mode'))
    )
    return Response(html, mimetype='text/html')


# ===== Main =====
if __name__ == '__main__':
    configure_http_request_logging()

    # Create a dedicated event loop for the command WebSocket
    import threading
    command_loop = asyncio.new_event_loop()
    def run_command_loop():
        asyncio.set_event_loop(command_loop)
        command_loop.run_until_complete(receive_commands())
    Thread(target=run_command_loop, daemon=True).start()

    Thread(target=lambda: asyncio.run(receive_video(0, CAM0_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_video(1, CAM1_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_video(2, CAM_HQ_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_telemetry()), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_fiducials()), daemon=True).start()
    Thread(target=receive_mavlink, daemon=True).start()
    Thread(target=status_monitor, daemon=True).start()
    app.run(port=8000, threaded=True)