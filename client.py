import asyncio
import cv2
import json
import logging
import numpy as np
import os
import secrets
import socket
import struct
import time
import sys
import websockets
from copy import deepcopy
from threading import Thread, Lock
from collections import deque
import statistics

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

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
HQ_CV_TARGET_TRACKING_TEXT = 'Target Tracking Ready'
HQ_CV_TARGET_TRACKING_TEXT_SCALE = 0.7
HQ_CV_TARGET_TRACKING_TEXT_THICKNESS = 2
HQ_CV_TARGET_TRACKING_TEXT_COLOR = (255, 220, 120)

# Target tracking workflow settings
HQ_TT_MODEL_CATALOG = [
    {
        'key': 'YOLOv8',
        'label': 'best8np2',
        'path': './object_detection/best8np2.pt',
    },
    {
        'key': 'YOLO26',
        'label': 'best26p2',
        'path': './object_detection/best26p2.pt',
    },
]
HQ_TT_MODEL_KEYS = [model.get('key') for model in HQ_TT_MODEL_CATALOG if model.get('key')]
HQ_TT_DEFAULT_MODEL_KEY = HQ_TT_MODEL_KEYS[0] if HQ_TT_MODEL_KEYS else HQ_CV_MODE_UNSELECTED
HQ_TT_MODEL_OPTIONS = {
    model['key']: {
        'label': model.get('label', model['key']),
        'path': model.get('path'),
    }
    for model in HQ_TT_MODEL_CATALOG
    if model.get('key')
}
HQ_TT_TRACKER_PATH = './object_detection/bytetrack_persist.yaml'
HQ_TT_STATUS_MAX_ATTEMPTS_TO_REPORT = 8
HQ_TT_VALIDATE_MODEL_ON_INIT = True
HQ_TT_INIT_VALIDATION_IMAGE_SIZE = 64
HQ_TT_INIT_VALIDATION_DEVICE = 'auto'
HQ_TT_INIT_RETRY_COOLDOWN_SEC = 5.0
HQ_TT_ASYNC_INIT_ENABLED = True
HQ_TT_INIT_STATUS_TEXT = 'Target Tracking: INITIALIZING MODEL...'
HQ_TT_IGNORE_DYLIB_CONFLICT = True
HQ_TT_CONFIDENCE = 0.45
HQ_TT_IOU = 0.2
HQ_TT_IMAGE_SIZE = 800
HQ_TT_PERSIST = True
HQ_TT_DEVICE = 'auto'
HQ_TT_INFER_MIN_INTERVAL_SEC = 0.12
HQ_TT_PERSON_ONLY = False
HQ_TT_LOCK_ON_CONF = 0.50
HQ_TT_STICK_MIN_CONF = 0.15
HQ_TT_HOLD_FRAMES = 6
HQ_TT_SELECTED_HOLD_MULTIPLIER = 4
HQ_TT_REACQUIRE_MIN_IOU = 0.08
HQ_TT_REACQUIRE_MAX_CENTER_DIST_RATIO = 0.4
HQ_TT_BOX_COLOR = (255, 170, 60)
HQ_TT_TARGET_COLOR = (40, 220, 120)
HQ_TT_TEXT_COLOR = (255, 255, 255)
HQ_TT_STABLE_ID_MAX_GAP = 60
HQ_TT_STABLE_ID_MIN_IOU = 0.20
HQ_TT_STABLE_ID_MAX_CENTER_DIST = 140.0
HQ_TT_SMOOTH_ALPHA = 0.60
HQ_TT_MIN_TRACK_HITS = 2
HQ_TT_STICKY_LOCK_CLASS_NAMES = {'person', 'pedestrian', 'people'}
HQ_LANDING_LOCAL_FIDUCIAL_FALLBACK_IN_TEST = True
HQ_LANDING_STATUS_TEXT_COLOR = (120, 220, 255)
HQ_TT_FALLBACK_IN_TEST = False
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
MAVLINK_MONITOR_ENABLED = True
MAVLINK_MONITOR_MESSAGE_BUFFER_SIZE = 250
MAVLINK_MONITOR_RECONNECT_DELAY_SEC = 2.0
MAVLINK_MONITOR_RECV_TIMEOUT_SEC = 1.0
MAVLINK_MONITOR_SOURCE_SYSTEM = 255
MAVLINK_MONITOR_SOURCE_COMPONENT = 0
MAVLINK_MONITOR_ALLOWED_MESSAGE_TYPES = {'STATUSTEXT'}
MAVLINK_MONITOR_STATUSTEXT_SEVERITY_LABELS = {
    0: 'EMERGENCY',
    1: 'ALERT',
    2: 'CRITICAL',
    3: 'ERROR',
    4: 'WARNING',
    5: 'NOTICE',
    6: 'INFO',
    7: 'DEBUG',
}
FLIGHT_MODE_OPTIONS = [
    {'id': 'STABILIZE', 'label': 'Stabilize'},
    {'id': 'ALT_HOLD', 'label': 'Alt Hold'},
    {'id': 'LOITER', 'label': 'Loiter'},
    {'id': 'GUIDED', 'label': 'Guided'},
    {'id': 'AUTO', 'label': 'Auto'},
    {'id': 'RTL', 'label': 'RTL'},
    {'id': 'LAND', 'label': 'Land'},
]
FLIGHT_MODE_DEFAULT = 'LOITER'
FLIGHT_MODE_IDS = {mode['id'] for mode in FLIGHT_MODE_OPTIONS}

TEST_MODE = True
TEST_MODE_ENABLE_COMMAND_WS = False
TEST_MODE_ENABLE_FIDUCIAL_WS = False

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
MAVLINK_MONITOR_UDP_ENDPOINT = 'udpin:0.0.0.0:14550'
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
DEFAULT_TILE_ORDER = ['cam0', 'cam1', 'hq', 'map', 'latency', 'mavlink_terminal', 'commands']

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
        'mavlink_terminal': True,
        'commands': True
    },
    'camera_function_tabs': DEFAULT_CAMERA_FUNCTION_TABS,
    'tile_order': DEFAULT_TILE_ORDER
}

ALLOWED_TILE_IDS = {'cam0', 'cam1', 'hq', 'map', 'latency', 'mavlink_terminal', 'commands'}
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
hq_tt_loaded_model_path = None
hq_tt_loaded_runtime_path = None
hq_tt_last_model_attempts = []
hq_tt_selected_model_key = HQ_TT_DEFAULT_MODEL_KEY
hq_tt_last_init_attempt_ts = 0.0
hq_tt_init_in_progress = False
hq_tt_class_filter = None
hq_tt_state_lock = Lock()
hq_tt_stable_assigner = None
hq_tt_track_smoother = None
hq_tt_sticky_gate = None
hq_tt_names_by_id = None
hq_tt_frame_index = 0
hq_tt_selected_track_id = None
hq_tt_selected_last_box = None
hq_tt_selected_missed_frames = 0
hq_tt_latest_detections = []
hq_tt_latest_frame_width = 0
hq_tt_latest_frame_height = 0
hq_tt_last_infer_ts = 0.0

# Telemetry data
mavlink_data = {
    "roll": 0, "pitch": 0, "yaw": 0,
    "lat": 0, "lon": 0, "alt": 0,
    "battery": 0, "battery_remaining": 0,
    "ground_speed": 0, "throttle": 0,
    "timestamp": 0, "server_latency_ms": 0
}
telemetry_latency_ms = 0

# MAVLink message monitor state (raw decoded messages from Pixhawk UDP endpoint)
mavlink_message_log = deque(maxlen=MAVLINK_MONITOR_MESSAGE_BUFFER_SIZE)
mavlink_message_lock = Lock()
mavlink_monitor_online = False
last_mavlink_message_time = 0

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
command_loop = None

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
    if len(data) > 8:
        timestamp = struct.unpack('d', data[:8])[0]
        jpeg_data = data[8:]
    else:
        timestamp = None
        jpeg_data = data
    frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
    return frame, timestamp


def draw_telemetry_text(frame, data):
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


def _hq_tt_stable_color(stable_id):
    return (
        int((stable_id * 37) % 255),
        int((stable_id * 17 + 91) % 255),
        int((stable_id * 67 + 53) % 255),
    )


class _HqStableIdAssigner:
    def __init__(self, max_gap, min_iou, max_center_dist):
        self.max_gap = max_gap
        self.min_iou = min_iou
        self.max_center_dist = max_center_dist
        self.next_stable_id = 1
        self.tracker_to_stable = {}
        self.tracker_last_seen = {}
        self.stable_last_box = {}
        self.stable_last_seen = {}

    def _match_existing_stable(self, box, used_stable_ids, frame_idx):
        best_stable = None
        best_score = -1.0

        for stable_id, prev_box in self.stable_last_box.items():
            if stable_id in used_stable_ids:
                continue

            age = frame_idx - self.stable_last_seen.get(stable_id, -999999)
            if age < 0 or age > self.max_gap:
                continue

            iou_score = _hq_tt_box_iou(box, prev_box)
            dist = _hq_tt_center_distance(box, prev_box)
            if iou_score < self.min_iou:
                continue
            if dist > self.max_center_dist:
                continue

            score = iou_score - (dist / max(self.max_center_dist, 1.0)) * 0.25
            if score > best_score:
                best_score = score
                best_stable = stable_id

        return best_stable

    def assign(self, detections, frame_idx):
        used_stable_ids = set()

        for det in detections:
            tracker_id = det.get('track_id')
            box = det.get('box')
            stable_id = None

            if tracker_id is not None and tracker_id in self.tracker_to_stable:
                candidate = self.tracker_to_stable[tracker_id]
                if candidate not in used_stable_ids:
                    stable_id = candidate

            if stable_id is None:
                matched = self._match_existing_stable(box, used_stable_ids, frame_idx)
                if matched is not None:
                    stable_id = matched
                else:
                    stable_id = self.next_stable_id
                    self.next_stable_id += 1

            if tracker_id is not None:
                self.tracker_to_stable[tracker_id] = stable_id
                self.tracker_last_seen[tracker_id] = frame_idx

            self.stable_last_box[stable_id] = box
            self.stable_last_seen[stable_id] = frame_idx
            used_stable_ids.add(stable_id)
            det['stable_id'] = stable_id

        stale_trackers = [
            tracker_id
            for tracker_id, last_seen in self.tracker_last_seen.items()
            if frame_idx - last_seen > self.max_gap * 2
        ]
        for tracker_id in stale_trackers:
            self.tracker_last_seen.pop(tracker_id, None)
            self.tracker_to_stable.pop(tracker_id, None)

        stale_stable = [
            stable_id
            for stable_id, last_seen in self.stable_last_seen.items()
            if frame_idx - last_seen > self.max_gap * 3
        ]
        for stable_id in stale_stable:
            self.stable_last_seen.pop(stable_id, None)
            self.stable_last_box.pop(stable_id, None)

        return detections


class _HqTrackSmoother:
    def __init__(self, alpha, hold_frames, min_track_hits):
        self.alpha = max(0.0, min(alpha, 1.0))
        self.hold_frames = max(0, hold_frames)
        self.min_track_hits = max(1, min_track_hits)
        self.state = {}

    def _smooth_box(self, previous, current):
        alpha = self.alpha
        inv = 1.0 - alpha
        return (
            previous[0] * inv + current[0] * alpha,
            previous[1] * inv + current[1] * alpha,
            previous[2] * inv + current[2] * alpha,
            previous[3] * inv + current[3] * alpha,
        )

    def update(self, detections, frame_idx):
        visible = []
        seen_stable_ids = set()

        for det in detections:
            stable_id = det.get('stable_id')
            if stable_id is None:
                continue

            current_box = det['box']
            track_state = self.state.get(stable_id)
            if track_state is None:
                smoothed_box = current_box
                hits = 1
            else:
                smoothed_box = self._smooth_box(track_state['box'], current_box)
                hits = int(track_state['hits']) + 1

            self.state[stable_id] = {
                'box': smoothed_box,
                'conf': det.get('conf'),
                'class_id': det.get('class_id', 0),
                'last_seen': frame_idx,
                'hits': hits,
            }
            seen_stable_ids.add(stable_id)

            if hits >= self.min_track_hits:
                visible.append({
                    **det,
                    'box': smoothed_box,
                    'hits': hits,
                    'ghost': False,
                })

        to_remove = []
        for stable_id, track_state in self.state.items():
            if stable_id in seen_stable_ids:
                continue

            age = frame_idx - int(track_state['last_seen'])
            hits = int(track_state['hits'])
            if age <= self.hold_frames and hits >= self.min_track_hits:
                visible.append({
                    'box': track_state['box'],
                    'conf': track_state.get('conf'),
                    'class_id': int(track_state.get('class_id', 0)),
                    'track_id': None,
                    'stable_id': stable_id,
                    'hits': hits,
                    'ghost': True,
                })
            elif age > self.hold_frames:
                to_remove.append(stable_id)

        for stable_id in to_remove:
            self.state.pop(stable_id, None)

        return visible


class _HqStickyConfidenceGate:
    def __init__(self, lock_on_conf, min_conf, max_gap, lock_class_ids=None):
        self.lock_on_conf = max(0.0, min(lock_on_conf, 1.0))
        self.min_conf = max(0.0, min(min_conf, 1.0))
        self.max_gap = max(1, max_gap)
        self.lock_class_ids = lock_class_ids or set()
        self.locked_last_seen = {}

    def filter(self, detections, frame_idx):
        visible = []

        for det in detections:
            stable_id = det.get('stable_id')
            conf = det.get('conf')
            class_id = det.get('class_id')
            if stable_id is None or conf is None:
                continue

            if class_id not in self.lock_class_ids:
                if conf >= self.min_conf:
                    visible.append(det)
                continue

            if stable_id in self.locked_last_seen:
                if conf >= self.min_conf:
                    self.locked_last_seen[stable_id] = frame_idx
                    visible.append(det)
                continue

            if conf >= self.lock_on_conf:
                self.locked_last_seen[stable_id] = frame_idx
                visible.append(det)

        stale_ids = [
            stable_id
            for stable_id, last_seen in self.locked_last_seen.items()
            if frame_idx - last_seen > self.max_gap
        ]
        for stable_id in stale_ids:
            self.locked_last_seen.pop(stable_id, None)

        return visible


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
        stable_id = det.get('stable_id')
        box = det.get('box')
        if stable_id is None or box is None or len(box) != 4:
            continue
        if det.get('ghost'):
            continue
        x1, y1, x2, y2 = [int(v) for v in box]
        if x1 <= click_x <= x2 and y1 <= click_y <= y2:
            area = max(1, (x2 - x1) * (y2 - y1))
            conf = float(det.get('conf', 0.0))
            candidates.append((area, -conf, int(stable_id)))

    if not candidates:
        return False, 'No target at click point', None

    candidates.sort(key=lambda item: (item[0], item[1]))
    selected_id = candidates[0][2]

    selected_box = None
    for det in detections:
        if det.get('stable_id') == selected_id:
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


def _hq_tt_get_available_model_options():
    options = []
    for key, meta in HQ_TT_MODEL_OPTIONS.items():
        options.append({
            'key': key,
            'label': str(key),
        })
    return options


def _hq_tt_get_selected_model_key():
    with hq_tt_model_lock:
        selected = hq_tt_selected_model_key
    if selected in HQ_TT_MODEL_OPTIONS:
        return selected
    return HQ_TT_DEFAULT_MODEL_KEY


def _hq_tt_reset_runtime_locked():
    global hq_tt_model, hq_tt_error, hq_tt_class_filter
    global hq_tt_loaded_model_path, hq_tt_loaded_runtime_path, hq_tt_last_model_attempts
    global hq_tt_stable_assigner, hq_tt_track_smoother, hq_tt_sticky_gate, hq_tt_names_by_id, hq_tt_frame_index
    hq_tt_model = None
    hq_tt_error = None
    hq_tt_class_filter = None
    hq_tt_loaded_model_path = None
    hq_tt_loaded_runtime_path = None
    hq_tt_last_model_attempts = []
    hq_tt_stable_assigner = None
    hq_tt_track_smoother = None
    hq_tt_sticky_gate = None
    hq_tt_names_by_id = None
    hq_tt_frame_index = 0


def _hq_tt_set_selected_model_key(model_key):
    global hq_tt_selected_model_key, hq_tt_last_init_attempt_ts
    if model_key not in HQ_TT_MODEL_OPTIONS:
        return False

    with hq_tt_model_lock:
        if hq_tt_selected_model_key == model_key:
            return True
        hq_tt_selected_model_key = model_key
        _hq_tt_reset_runtime_locked()
        hq_tt_last_init_attempt_ts = 0.0

    clear_hq_target_selection()
    return True


def _hq_tt_get_selected_model_path():
    selected_key = _hq_tt_get_selected_model_key()
    selected_meta = HQ_TT_MODEL_OPTIONS.get(selected_key, {})
    model_path = selected_meta.get('path')
    if isinstance(model_path, str) and model_path:
        return model_path
    return None


def _hq_tt_resolve_runtime_model_path(selected_model_path):
    if not selected_model_path:
        return None, 'No target-tracking model selected.'

    model_path = str(selected_model_path)
    lower = model_path.lower()

    if lower.endswith('.pt'):
        if os.path.exists(model_path):
            return model_path, None
        return None, f'Model not found: {model_path}'

    stem, ext = os.path.splitext(model_path)
    if lower.endswith('.pth'):
        pt_path = f'{stem}.pt'
        return None, (
            f'Configured model is .pth ({model_path}). '
            f'Use the .pt file directly instead (e.g. {pt_path}).'
        )

    if lower.endswith('.mlpackage'):
        return None, (
            f'Configured model is .mlpackage ({model_path}). '
            'CoreML runtime is disabled. Use a .pt model path instead.'
        )

    return None, f'Unsupported model format: {ext} (use .pt only)'


def _hq_tt_validate_model_runtime(model):
    if not HQ_TT_VALIDATE_MODEL_ON_INIT:
        return True, None

    try:
        size = int(max(32, HQ_TT_INIT_VALIDATION_IMAGE_SIZE))
        test_frame = np.zeros((size, size, 3), dtype=np.uint8)
        validation_device = _hq_tt_pick_device(HQ_TT_INIT_VALIDATION_DEVICE)
        model.predict(
            source=test_frame,
            conf=0.01,
            imgsz=size,
            device=validation_device,
            verbose=False,
        )
        return True, None
    except Exception as validation_error:
        return False, validation_error


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


def _hq_tt_init_pipeline(model):
    global hq_tt_stable_assigner, hq_tt_track_smoother, hq_tt_sticky_gate, hq_tt_names_by_id, hq_tt_frame_index

    names_by_id = _hq_tt_normalize_names(model.names)
    hq_tt_names_by_id = names_by_id

    hq_tt_stable_assigner = _HqStableIdAssigner(
        max_gap=HQ_TT_STABLE_ID_MAX_GAP,
        min_iou=HQ_TT_STABLE_ID_MIN_IOU,
        max_center_dist=HQ_TT_STABLE_ID_MAX_CENTER_DIST,
    )
    hq_tt_track_smoother = _HqTrackSmoother(
        alpha=HQ_TT_SMOOTH_ALPHA,
        hold_frames=HQ_TT_HOLD_FRAMES,
        min_track_hits=HQ_TT_MIN_TRACK_HITS,
    )

    lock_class_ids = {
        idx
        for idx, name in names_by_id.items()
        if str(name).lower() in HQ_TT_STICKY_LOCK_CLASS_NAMES
    }
    hq_tt_sticky_gate = _HqStickyConfidenceGate(
        lock_on_conf=HQ_TT_LOCK_ON_CONF,
        min_conf=HQ_TT_STICK_MIN_CONF,
        max_gap=max(HQ_TT_HOLD_FRAMES, HQ_TT_STABLE_ID_MAX_GAP),
        lock_class_ids=lock_class_ids,
    )
    hq_tt_frame_index = 0


def _hq_tt_build_runtime():
    global hq_tt_model, hq_tt_error, hq_tt_class_filter
    global hq_tt_loaded_model_path, hq_tt_loaded_runtime_path, hq_tt_last_model_attempts, hq_tt_last_init_attempt_ts
    if hq_tt_model is not None:
        return True

    now = time.time()
    if hq_tt_error and (now - hq_tt_last_init_attempt_ts) < HQ_TT_INIT_RETRY_COOLDOWN_SEC:
        return False

    with hq_tt_model_lock:
        if hq_tt_model is not None:
            return True
        try:
            print('HQ TT: build runtime starting')
            print('HQ TT: importing ultralytics')
            from ultralytics import YOLO
            print('HQ TT: imported ultralytics')

            hq_tt_last_init_attempt_ts = time.time()

            # Note: we already hold hq_tt_model_lock here; avoid calling helpers
            # that also attempt to acquire the same lock (would deadlock).
            selected_key = hq_tt_selected_model_key if hq_tt_selected_model_key in HQ_TT_MODEL_OPTIONS else HQ_TT_DEFAULT_MODEL_KEY
            selected_meta = HQ_TT_MODEL_OPTIONS.get(selected_key, {})
            selected_model_path = selected_meta.get('path')
            print(f'HQ TT: selected_model_key={selected_key}', flush=True)
            print(f'HQ TT: selected_model_path={selected_model_path}', flush=True)
            try:
                exists_flag = os.path.exists(selected_model_path) if selected_model_path else False
            except Exception:
                exists_flag = False
            print(f'HQ TT: selected_model_path exists={exists_flag}', flush=True)
            if not selected_model_path or not exists_flag:
                hq_tt_error = 'Target Tracking model not found.'
                hq_tt_loaded_model_path = None
                hq_tt_loaded_runtime_path = None
                hq_tt_last_model_attempts = []
                return False

            runtime_model_path, resolve_error = _hq_tt_resolve_runtime_model_path(selected_model_path)
            if runtime_model_path is None:
                hq_tt_error = resolve_error or 'Target Tracking runtime model unavailable.'
                hq_tt_loaded_model_path = selected_model_path
                hq_tt_loaded_runtime_path = None
                hq_tt_last_model_attempts = []
                return False

            hq_tt_last_model_attempts = [runtime_model_path]

            print(f'HQ TT: creating YOLO model from runtime path: {runtime_model_path}')
            candidate_model = YOLO(runtime_model_path)
            print('HQ TT: YOLO model object created')

            is_valid, validation_error = _hq_tt_validate_model_runtime(candidate_model)
            if not is_valid:
                raise RuntimeError(f'Runtime validation failed for {runtime_model_path}: {validation_error}')

            hq_tt_model = candidate_model

            if HQ_TT_PERSON_ONLY:
                try:
                    names_by_id = _hq_tt_normalize_names(hq_tt_model.names)
                    class_ids = _hq_tt_people_related_class_ids(names_by_id)
                    hq_tt_class_filter = class_ids if class_ids else None
                except Exception:
                    hq_tt_class_filter = None
            else:
                hq_tt_class_filter = None

            _hq_tt_init_pipeline(hq_tt_model)

            hq_tt_error = None
            hq_tt_loaded_model_path = selected_model_path
            hq_tt_loaded_runtime_path = runtime_model_path
            hq_tt_last_model_attempts = hq_tt_last_model_attempts[-HQ_TT_STATUS_MAX_ATTEMPTS_TO_REPORT:]
            if runtime_model_path and runtime_model_path != selected_model_path:
                print(f"HQ Target Tracking initialized with model: {selected_model_path} (runtime: {runtime_model_path})")
            else:
                print(f"HQ Target Tracking initialized with model: {selected_model_path}")
            return True
        except Exception as e:
            import traceback
            hq_tt_model = None
            hq_tt_error = f'Target Tracking init failed: {e}'
            hq_tt_loaded_model_path = None
            hq_tt_loaded_runtime_path = None
            hq_tt_last_model_attempts = hq_tt_last_model_attempts[-HQ_TT_STATUS_MAX_ATTEMPTS_TO_REPORT:]
            print(hq_tt_error)
            traceback.print_exc()
            return False


def _hq_tt_initialize_runtime_worker():
    global hq_tt_init_in_progress
    try:
        print('HQ TT: runtime worker starting', flush=True)
        _hq_tt_build_runtime()
        print('HQ TT: runtime worker completed', flush=True)
    finally:
        with hq_tt_model_lock:
            hq_tt_init_in_progress = False


def _hq_tt_start_async_runtime_init_if_needed():
    global hq_tt_init_in_progress, hq_tt_error

    now = time.time()
    with hq_tt_model_lock:
        if hq_tt_model is not None or hq_tt_init_in_progress:
            return
        if hq_tt_error and (now - hq_tt_last_init_attempt_ts) < HQ_TT_INIT_RETRY_COOLDOWN_SEC:
            return
        # Detect a common macOS crash scenario: both OpenCV (cv2) and PyAV
        # bundle different libavdevice dylibs which conflict at load time.
        # Skip this check entirely when HQ_TT_IGNORE_DYLIB_CONFLICT is enabled
        # to avoid importing PyAV and triggering the conflict.
        try:
            if os.name == 'posix' and sys.platform == 'darwin' and not HQ_TT_IGNORE_DYLIB_CONFLICT:
                av_pkg = None
                try:
                    import av as _av
                    av_pkg = _av
                except Exception:
                    av_pkg = None

                if av_pkg is not None:
                    try:
                        cv2_path = os.path.abspath(cv2.__file__)
                    except Exception:
                        cv2_path = '<unknown cv2 path>'
                    try:
                        av_path = os.path.abspath(av_pkg.__file__)
                    except Exception:
                        av_path = '<unknown av path>'

                    conflict_msg = (
                        'Detected both OpenCV and PyAV in the environment which often '
                        'bundle conflicting libavdevice dylibs on macOS. This can crash '
                        'the process when model code loads libav.\n'
                        f'OpenCV path: {cv2_path}\n'
                        f'PyAV path: {av_path}\n'
                    )
                    hq_tt_error = ('Model init disabled: conflicting libavdevice dylibs detected. '
                                   'Set HQ_TT_IGNORE_DYLIB_CONFLICT=True to override or remove one package.')
                    print('HQ TT: dylib conflict detected, aborting model init')
                    print(conflict_msg)
                    return
        except Exception:
            pass
        hq_tt_init_in_progress = True
        hq_tt_error = HQ_TT_INIT_STATUS_TEXT

    print('HQ TT: scheduling async model init')
    Thread(target=_hq_tt_initialize_runtime_worker, daemon=True).start()


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
        x1, y1, x2, y2 = [float(v) for v in box[:4]]
        conf = float(conf_np[i]) if conf_np is not None and i < len(conf_np) else None
        class_id = int(cls_np[i]) if cls_np is not None and i < len(cls_np) else 0
        track_id = int(id_np[i]) if id_np is not None and i < len(id_np) else None
        detections.append({
            'box': (x1, y1, x2, y2),
            'conf': conf,
            'class_id': class_id,
            'track_id': track_id,
            'stable_id': None,
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
            'class_id': -1,
            'track_id': None,
            'stable_id': idx + 1,
            'ghost': False,
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
            if det.get('stable_id') == selected_id:
                target_det = det
                break
        if target_det is None:
            clear_hq_target_selection()

    cx, cy = w // 2, h // 2
    cv2.drawMarker(frame, (cx, cy), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=1)

    # Fallback detector is deprecated — show message indicating it was disabled.
    _hq_tt_draw_status(frame, 'Target Tracking fallback disabled; model required', (0, 0, 255))


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
    global hq_tt_last_infer_ts
    global hq_tt_latest_detections, hq_tt_latest_frame_width, hq_tt_latest_frame_height
    global hq_tt_selected_track_id, hq_tt_selected_last_box, hq_tt_selected_missed_frames
    global hq_tt_stable_assigner, hq_tt_track_smoother, hq_tt_sticky_gate, hq_tt_names_by_id, hq_tt_frame_index

    try:
        if hq_tt_model is None and HQ_TT_ASYNC_INIT_ENABLED:
            _hq_tt_start_async_runtime_init_if_needed()
            # Do not run the local fallback detector — show initializing/error status instead.
            _hq_tt_draw_status(frame, hq_tt_error or HQ_TT_INIT_STATUS_TEXT, (0, 180, 255))
            return

        if not _hq_tt_build_runtime():
            # Model not ready — display error/status. No fallback detector.
            _hq_tt_draw_status(frame, hq_tt_error or HQ_CV_TARGET_TRACKING_TEXT, (0, 0, 255))
            return

        if hq_tt_stable_assigner is None or hq_tt_track_smoother is None or hq_tt_sticky_gate is None:
            _hq_tt_init_pipeline(hq_tt_model)

        detections = None
        smoothed_detections = None
        should_run_infer = True
        now = time.time()
        with hq_tt_state_lock:
            last_infer_ts = float(hq_tt_last_infer_ts)

        if HQ_TT_INFER_MIN_INTERVAL_SEC > 0 and (now - last_infer_ts) < HQ_TT_INFER_MIN_INTERVAL_SEC:
            should_run_infer = False

        infer_lock_acquired = False
        if should_run_infer:
            infer_lock_acquired = hq_tt_infer_lock.acquire(blocking=False)
            if not infer_lock_acquired:
                should_run_infer = False

        if should_run_infer:
            try:
                device = _hq_tt_pick_device(HQ_TT_DEVICE)
                tracking_conf = min(HQ_TT_CONFIDENCE, HQ_TT_STICK_MIN_CONF)
                result_list = hq_tt_model.track(
                    source=frame,
                    conf=tracking_conf,
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
                hq_tt_frame_index += 1

                if hq_tt_stable_assigner and hq_tt_track_smoother:
                    detections = hq_tt_stable_assigner.assign(detections, hq_tt_frame_index)
                    smoothed_detections = hq_tt_track_smoother.update(detections, hq_tt_frame_index)
                else:
                    smoothed_detections = detections

                if hq_tt_sticky_gate:
                    detections = hq_tt_sticky_gate.filter(smoothed_detections, hq_tt_frame_index)
                else:
                    detections = smoothed_detections

                with hq_tt_state_lock:
                    hq_tt_last_infer_ts = time.time()
            except Exception as e:
                _hq_tt_draw_status(frame, f'Target Tracking error: {e}', (0, 0, 255))
                print(f'HQ TT: inference error: {e}')
                return
            finally:
                if infer_lock_acquired:
                    hq_tt_infer_lock.release()

        if detections is None:
            with hq_tt_state_lock:
                detections = list(hq_tt_latest_detections)
            smoothed_detections = list(detections)

        h, w = frame.shape[:2]
        with hq_tt_state_lock:
            try:
                hq_tt_latest_detections = detections
                hq_tt_latest_frame_width = w
                hq_tt_latest_frame_height = h
                selected_id = hq_tt_selected_track_id
                selected_last_box = hq_tt_selected_last_box
                selected_missed_frames = hq_tt_selected_missed_frames
            except Exception as e:
                print(f'HQ TT: state update error: {e}')
                selected_id = None
                selected_last_box = None
                selected_missed_frames = 0
                hq_tt_latest_detections = []
                hq_tt_latest_frame_width = w
                hq_tt_latest_frame_height = h
    except Exception as top_e:
        print(f'HQ TT: unexpected error in apply_hq_cv_target_tracking_mode: {top_e}')
        _hq_tt_draw_status(frame, 'Target Tracking unexpected error', (0, 0, 255))
        return

    smoothed_detections = smoothed_detections or []
    by_stable_id = {
        d['stable_id']: d
        for d in smoothed_detections
        if d.get('stable_id') is not None
    }

    target_det = None
    if selected_id is not None:
        target_det = by_stable_id.get(selected_id)
        if target_det is not None:
            if target_det.get('ghost'):
                selected_missed_frames += 1
            else:
                selected_last_box = target_det.get('box')
                selected_missed_frames = 0
        else:
            selected_missed_frames += 1

        selected_hold_limit = max(1, int(HQ_TT_HOLD_FRAMES * HQ_TT_SELECTED_HOLD_MULTIPLIER))
        if target_det is None and selected_last_box is not None:
            best_det = None
            best_score = -1.0
            max_center_dist = max(w, h) * HQ_TT_REACQUIRE_MAX_CENTER_DIST_RATIO
            for det in smoothed_detections:
                if det.get('ghost'):
                    continue
                if det.get('conf') is None or det.get('conf') < HQ_TT_LOCK_ON_CONF:
                    continue
                box = det.get('box')
                if box is None:
                    continue
                iou_score = _hq_tt_box_iou(selected_last_box, box)
                if iou_score < HQ_TT_REACQUIRE_MIN_IOU:
                    continue
                dist = _hq_tt_center_distance(selected_last_box, box)
                if dist > max_center_dist:
                    continue
                score = iou_score - (dist / max(1.0, max_center_dist)) * 0.25
                if score > best_score:
                    best_score = score
                    best_det = det

            if best_det is not None:
                selected_id = best_det.get('stable_id')
                selected_last_box = best_det.get('box')
                selected_missed_frames = 0
                target_det = best_det
        if selected_missed_frames > selected_hold_limit:
            selected_id = None
            selected_last_box = None
            selected_missed_frames = 0

    with hq_tt_state_lock:
        hq_tt_selected_track_id = selected_id
        hq_tt_selected_last_box = selected_last_box
        hq_tt_selected_missed_frames = selected_missed_frames

    names_by_id = hq_tt_names_by_id

    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det['box']]
        is_target = target_det is not None and det.get('stable_id') == target_det.get('stable_id')
        color = HQ_TT_TARGET_COLOR if is_target else HQ_TT_BOX_COLOR
        thickness = 3 if is_target else 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

        label_parts = []
        if det.get('stable_id') is not None:
            label_parts.append(f"ID {det['stable_id']}")
        if names_by_id and det.get('class_id') in names_by_id:
            label_parts.append(names_by_id[det['class_id']])
        if det.get('conf') is not None:
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
        _hq_tt_draw_status(frame, f"Target Tracking: LOCKED (ID {target_det.get('stable_id')})", HQ_TT_TARGET_COLOR)
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
                        # Wait for incoming telemetry without an artificial short timeout.
                        # Rely on the WebSocket ping/pong (configured via
                        # `ping_interval` / `ping_timeout`) to detect dead peers
                        # and close the connection when necessary.
                        data = await ws.recv()
                    except websockets.exceptions.ConnectionClosed:
                        print("Telemetry WebSocket closed, reconnecting...")
                        break
                    except Exception as e:
                        print(f"Telemetry recv error: {e}, reconnecting...")
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


def _format_mavlink_message(msg):
    msg_type = 'UNKNOWN'
    message_dict = {}

    try:
        msg_type = str(msg.get_type())
    except Exception:
        return None

    if msg_type not in MAVLINK_MONITOR_ALLOWED_MESSAGE_TYPES:
        return None

    try:
        message_dict = msg.to_dict() or {}
    except Exception:
        message_dict = {}

    if msg_type == 'STATUSTEXT':
        severity = int(message_dict.get('severity', 6))
        severity_label = MAVLINK_MONITOR_STATUSTEXT_SEVERITY_LABELS.get(severity, f'SEV{severity}')

        raw_text = message_dict.get('text', '')
        if isinstance(raw_text, bytes):
            raw_text = raw_text.decode('utf-8', errors='replace')
        text = str(raw_text).replace('\x00', '').strip()
        if not text:
            return None

        timestamp = time.time()
        line = f"[{time.strftime('%H:%M:%S', time.localtime(timestamp))}] {severity_label}: {text}"
        return {
            'timestamp': timestamp,
            'type': msg_type,
            'line': line,
            'payload': {
                'severity': severity,
                'severity_label': severity_label,
                'text': text,
            },
        }

    return None


def _append_mavlink_message_entry(entry):
    global last_mavlink_message_time
    with mavlink_message_lock:
        mavlink_message_log.append(entry)
    last_mavlink_message_time = entry.get('timestamp', time.time())


def receive_mavlink_udp_messages():
    """Receive decoded MAVLink messages from the configured Pixhawk UDP endpoint."""
    global mavlink_monitor_online, last_mavlink_message_time

    if not MAVLINK_MONITOR_ENABLED:
        return

    if mavutil is None:
        mavlink_monitor_online = False
        return

    while True:
        mav = None
        try:
            mav = mavutil.mavlink_connection(
                MAVLINK_MONITOR_UDP_ENDPOINT,
                source_system=MAVLINK_MONITOR_SOURCE_SYSTEM,
                source_component=MAVLINK_MONITOR_SOURCE_COMPONENT,
            )

            while True:
                msg = mav.recv_match(blocking=True, timeout=MAVLINK_MONITOR_RECV_TIMEOUT_SEC)
                if msg is None:
                    if (time.time() - last_mavlink_message_time) > STATUS_TIMEOUT:
                        mavlink_monitor_online = False
                    continue

                # Keep terminal online status tied to MAVLink link health,
                # regardless of whether this specific packet is shown.
                mavlink_monitor_online = True
                last_mavlink_message_time = time.time()

                msg_type = ''
                try:
                    msg_type = msg.get_type()
                except Exception:
                    msg_type = ''

                if msg_type in ('BAD_DATA',):
                    continue

                entry = _format_mavlink_message(msg)
                if entry is not None:
                    _append_mavlink_message_entry(entry)
        except Exception:
            mavlink_monitor_online = False
            time.sleep(MAVLINK_MONITOR_RECONNECT_DELAY_SEC)
        finally:
            try:
                if mav is not None and getattr(mav, 'close', None):
                    mav.close()
            except Exception:
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
            draw_throttle_widget(f, mavlink_data.get('throttle', 0), position=(10, 190), width=20, height=100)
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

        # Update median stats for network latency (copy deque first to avoid
        # 'deque mutated during iteration' if another thread appends)
        _samples = list(cam0_latency_samples)
        nets = [n for n, r in _samples if n is not None]
        procs = [r for n, r in _samples if r is not None]
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

        # Update median stats (copy deque first to avoid mutation during iteration)
        _samples = list(cam1_latency_samples)
        nets = [n for n, r in _samples if n is not None]
        procs = [r for n, r in _samples if r is not None]
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

        # Update median stats (copy deque first to avoid mutation during iteration)
        _samples = list(hq_latency_samples)
        nets = [n for n, r in _samples if n is not None]
        procs = [r for n, r in _samples if r is not None]
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
        user_settings=user_settings,
        flight_mode_options=FLIGHT_MODE_OPTIONS,
        flight_mode_default=FLIGHT_MODE_DEFAULT
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
        'mavlink_terminal_online': mavlink_monitor_online,
        'map_online': is_map_online(),
        'hq_online': hq_online
    })


@app.route('/api/mavlink_messages')
def api_mavlink_messages():
    with mavlink_message_lock:
        messages = list(mavlink_message_log)
    return jsonify({
        'online': mavlink_monitor_online,
        'endpoint': MAVLINK_MONITOR_UDP_ENDPOINT,
        'count': len(messages),
        'messages': messages,
        'server_time': time.time(),
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


@app.route('/api/arm', methods=['POST'])
def api_arm():
    """Send arm/disarm request to the server command socket."""
    data = request.json or {}
    armed = bool(data.get('armed'))

    if not command_ws:
        return jsonify({'success': False, 'message': 'Command link unavailable'}), 503

    msg = {'type': 'arm', 'armed': armed}
    try:
        asyncio.run_coroutine_threadsafe(
            command_ws.send(json.dumps(msg)),
            command_loop
        )
    except Exception as e:
        print(f"Failed to forward arm command to server: {e}")
        return jsonify({'success': False, 'message': 'Failed to send arm command'}), 500

    return jsonify({'success': True, 'armed': armed})


@app.route('/api/flight_mode', methods=['POST'])
def api_flight_mode():
    """Send a flight mode change request to the server command socket."""
    data = request.json or {}
    mode = str(data.get('mode', '')).strip().upper()

    if mode not in FLIGHT_MODE_IDS:
        return jsonify({'success': False, 'message': 'Invalid flight mode'}), 400

    if not command_ws:
        return jsonify({'success': False, 'message': 'Command link unavailable'}), 503

    msg = {'type': 'mode', 'mode': mode}
    try:
        asyncio.run_coroutine_threadsafe(
            command_ws.send(json.dumps(msg)),
            command_loop
        )
    except Exception as e:
        print(f"Failed to forward flight mode to server: {e}")
        return jsonify({'success': False, 'message': 'Failed to send mode change'}), 500

    return jsonify({'success': True, 'mode': mode})


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


@app.route('/api/hq_target_tracking_status', methods=['GET'])
def api_hq_target_tracking_status():
    selected_model_key = _hq_tt_get_selected_model_key()
    selected_meta = HQ_TT_MODEL_OPTIONS.get(selected_model_key, {})
    return jsonify({
        'mode': get_hq_cv_mode(),
        'ready': hq_tt_model is not None,
        'initializing': bool(hq_tt_init_in_progress),
        'error': hq_tt_error,
        'selected_model_key': selected_model_key,
        'selected_model_label': str(selected_meta.get('label', selected_model_key)),
        'available_models': _hq_tt_get_available_model_options(),
        'loaded_model_path': hq_tt_loaded_model_path,
        'loaded_runtime_path': hq_tt_loaded_runtime_path,
        'attempted_models': list(hq_tt_last_model_attempts[-HQ_TT_STATUS_MAX_ATTEMPTS_TO_REPORT:]),
    })


@app.route('/api/hq_target_tracking_model', methods=['GET', 'POST'])
def api_hq_target_tracking_model():
    if request.method == 'GET':
        selected_model_key = _hq_tt_get_selected_model_key()
        selected_meta = HQ_TT_MODEL_OPTIONS.get(selected_model_key, {})
        return jsonify({
            'selected_model_key': selected_model_key,
            'selected_model_label': str(selected_meta.get('label', selected_model_key)),
            'available_models': _hq_tt_get_available_model_options(),
        })

    payload = request.json or {}
    model_key = payload.get('model_key')
    if not isinstance(model_key, str) or model_key not in HQ_TT_MODEL_OPTIONS:
        return jsonify({'success': False, 'message': 'Invalid target-tracking model key'}), 400

    ok = _hq_tt_set_selected_model_key(model_key)
    if not ok:
        return jsonify({'success': False, 'message': 'Failed to update target-tracking model'}), 500

    selected_model_key = _hq_tt_get_selected_model_key()
    selected_meta = HQ_TT_MODEL_OPTIONS.get(selected_model_key, {})
    return jsonify({
        'success': True,
        'selected_model_key': selected_model_key,
        'selected_model_label': str(selected_meta.get('label', selected_model_key)),
        'available_models': _hq_tt_get_available_model_options(),
    })


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
    if not TEST_MODE or TEST_MODE_ENABLE_COMMAND_WS:
        command_loop = asyncio.new_event_loop()
        def run_command_loop():
            asyncio.set_event_loop(command_loop)
            command_loop.run_until_complete(receive_commands())
        Thread(target=run_command_loop, daemon=True).start()
    else:
        print('TEST_MODE: command websocket disabled (TEST_MODE_ENABLE_COMMAND_WS=False).')

    Thread(target=lambda: asyncio.run(receive_video(0, CAM0_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_video(1, CAM1_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_video(2, CAM_HQ_PORT)), daemon=True).start()
    Thread(target=lambda: asyncio.run(receive_telemetry()), daemon=True).start()
    if not TEST_MODE or TEST_MODE_ENABLE_FIDUCIAL_WS:
        Thread(target=lambda: asyncio.run(receive_fiducials()), daemon=True).start()
    else:
        print('TEST_MODE: fiducial websocket disabled (TEST_MODE_ENABLE_FIDUCIAL_WS=False).')
    Thread(target=receive_mavlink, daemon=True).start()
    Thread(target=receive_mavlink_udp_messages, daemon=True).start()
    Thread(target=status_monitor, daemon=True).start()
    app.run(port=8000, threaded=True)