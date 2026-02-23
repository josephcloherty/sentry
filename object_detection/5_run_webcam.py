import argparse
import importlib.util
import os
import shutil
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
from ultralytics import YOLO

from websockets.exceptions import ConnectionClosed
from websockets.sync.client import connect as ws_connect


DEFAULT_WS_URL = "ws://127.0.0.1:8886"
DEFAULT_WS_HAS_TIMESTAMP_HEADER = True
DEFAULT_WS_RECONNECT_DELAY_SEC = 2.0
DEFAULT_WS_RECEIVE_TIMEOUT_SEC = 5.0
DEFAULT_WS_OUTPUT_FPS = 30.0

DEFAULT_SOURCE = "webcam"
DEFAULT_CAMERA_INDEX = 0
DEFAULT_CAMERA_BACKEND = "auto"
DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FPS = 15.0
DEFAULT_PROCESSING_FPS = 15.0
DEFAULT_CAMERA_WARMUP_FRAMES = 20
DEFAULT_CAMERA_BLACK_MEAN_THRESHOLD = 8.0
DEFAULT_CAMERA_BLACK_STREAK_LIMIT = 45
DEFAULT_CAMERA_BLACK_STD_THRESHOLD = 2.5
DEFAULT_CAMERA_PROBE_FRAMES = 24
DEFAULT_CAMERA_MAX_INDEX_TO_PROBE = 4
DEFAULT_WEBCAM_CAPTURE_MODE = "opencv"
DEFAULT_WEBCAM_READ_FAIL_STREAK_LIMIT = 20
DEFAULT_WEBCAM_REOPEN_DELAY_SEC = 0.2
DEFAULT_USE_THREADED_READER = True

DEFAULT_INPUT_VIDEO = None
DEFAULT_NEW_MODEL_PATH = './object_detection/best26p2.pt'
DEFAULT_OLD_MODEL_PATH = "./object_detection/best8np2.pt"
DEFAULT_MODEL_FAMILY = "new"
DEFAULT_OUTPUT_VIDEO = None#r"C:\Users\josep\Downloads\test1080_annotated.mp4"
DEFAULT_CONFIDENCE = 0.35
DEFAULT_IMAGE_SIZE = 640
DEFAULT_DEVICE = "auto"
DEFAULT_IOU = 0.2
DEFAULT_TRACKER = "bytetrack_persist.yaml"
DEFAULT_PERSIST = True
DEFAULT_STABLE_ID_MAX_GAP = 60
DEFAULT_STABLE_ID_MIN_IOU = 0.20
DEFAULT_STABLE_ID_MAX_CENTER_DIST = 140.0
DEFAULT_SMOOTH_ALPHA = 0.60
DEFAULT_HOLD_FRAMES = 12
DEFAULT_MIN_TRACK_HITS = 2

DEFAULT_LOCK_ON_CONF = 0.5
DEFAULT_STICK_MIN_CONF = 0.15
DEFAULT_STICKY_FALLBACK_TO_UNFILTERED = True

DEFAULT_INCLUDE_CLASSES = None
DEFAULT_EXCLUDE_CLASSES = None
DEFAULT_PERSON_ONLY = True

PEOPLE_RELATED_EXACT = {
    "person",
    "pedestrian",
    "people",
    "bicycle",
    "cyclist",
    "rider",
    "motor",
    "motorbike",
    "motorcycle",
    "tricycle",
    "awning-tricycle",
}
PEOPLE_RELATED_KEYWORDS = ("person", "pedestrian", "people", "rider", "cyclist", "bicycle", "bike", "motor", "tricycle", "human")
ACTION_CLASS_KEYWORDS = ("walk", "run", "sit", "stand", "ride", "jump", "fall", "action")

DEFAULT_HIDE_LABELS = False
DEFAULT_HIDE_CONF = False
DEFAULT_HIDE_BOXES = False
DEFAULT_LINE_WIDTH = None
DEFAULT_SHOW_PREVIEW = True
DEFAULT_FRAME_READ_TIMEOUT_SEC = 0.20


def is_macos() -> bool:
    return sys.platform == "darwin"


def pick_camera_backend(backend_arg: str) -> int:
    backend = backend_arg.lower().strip()
    if backend == "auto":
        return cv2.CAP_AVFOUNDATION if is_macos() else cv2.CAP_ANY
    if backend == "avfoundation":
        return cv2.CAP_AVFOUNDATION
    if backend == "any":
        return cv2.CAP_ANY
    raise ValueError("Unsupported --camera-backend. Use: auto, avfoundation, any")


class FFmpegWebcamCapture:
    def __init__(self, camera_index: int, frame_width: int, frame_height: int, frame_fps: float) -> None:
        self.camera_index = camera_index
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.frame_fps = frame_fps
        self.process: subprocess.Popen | None = None
        self.frame_size = int(self.frame_width * self.frame_height * 3)

    def open(self) -> None:
        ffmpeg_path = shutil.which("ffmpeg")
        if ffmpeg_path is None:
            raise RuntimeError(
                "FFmpeg is required for macOS webcam mode but was not found in PATH. "
                "Install FFmpeg or run with --webcam-capture-mode opencv."
            )

        if not is_macos():
            raise RuntimeError("FFmpeg AVFoundation webcam mode is only supported on macOS in this script.")

        input_candidates = [f"{self.camera_index}:none", f"{self.camera_index}"]
        last_error: str | None = None

        for input_device in input_candidates:
            command = [
                ffmpeg_path,
                "-hide_banner",
                "-loglevel",
                "error",
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
                "-analyzeduration",
                "0",
                "-probesize",
                "32",
                "-f",
                "avfoundation",
                "-framerate",
                f"{self.frame_fps:.2f}",
                "-video_size",
                f"{self.frame_width}x{self.frame_height}",
                "-i",
                input_device,
                "-pix_fmt",
                "bgr24",
                "-f",
                "rawvideo",
                "pipe:1",
            ]

            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=max(self.frame_size * 2, 10**6),
            )

            time.sleep(0.25)
            if self.process.poll() is None:
                return

            stderr_output = b""
            if self.process.stderr is not None:
                try:
                    stderr_output = self.process.stderr.read() or b""
                except Exception:
                    stderr_output = b""
            last_error = stderr_output.decode(errors="ignore").strip()
            self.process = None

        raise RuntimeError(
            "FFmpeg failed to open webcam stream. "
            f"Last error: {last_error or 'unknown ffmpeg camera error'}"
        )

    def isOpened(self) -> bool:
        return self.process is not None and self.process.poll() is None and self.process.stdout is not None

    def read(self) -> tuple[bool, np.ndarray | None]:
        if not self.isOpened() or self.process is None or self.process.stdout is None:
            return False, None

        data = self.process.stdout.read(self.frame_size)
        if not data or len(data) != self.frame_size:
            return False, None

        frame = np.frombuffer(data, dtype=np.uint8).reshape((self.frame_height, self.frame_width, 3))
        return True, frame

    def get(self, prop: int) -> float:
        if prop == cv2.CAP_PROP_FPS:
            return float(self.frame_fps)
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self.frame_width)
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self.frame_height)
        return 0.0

    def release(self) -> None:
        if self.process is None:
            return

        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except Exception:
                self.process.kill()
                self.process.wait(timeout=1.0)

        self.process = None


class LatestFrameReader:
    def __init__(self, capture) -> None:
        self.capture = capture
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._latest_frame: np.ndarray | None = None
        self._latest_seq = 0

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            ok, frame = self.capture.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue
            with self._lock:
                self._latest_frame = frame
                self._latest_seq += 1

    def read_latest(self, last_seq: int, timeout_sec: float) -> tuple[bool, np.ndarray | None, int]:
        deadline = time.time() + max(0.01, timeout_sec)
        while time.time() < deadline:
            with self._lock:
                if self._latest_frame is not None and self._latest_seq != last_seq:
                    return True, self._latest_frame.copy(), self._latest_seq
            time.sleep(0.002)

        with self._lock:
            if self._latest_frame is not None:
                return True, self._latest_frame.copy(), self._latest_seq

        return False, None, last_seq

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)


def open_capture_by_mode(
    capture_mode: str,
    camera_index: int,
    camera_backend: str,
    camera_width: int,
    camera_height: int,
    camera_fps: float,
) -> tuple[object, int, str, str]:
    mode = capture_mode.lower().strip()

    if mode == "ffmpeg":
        cap = FFmpegWebcamCapture(
            camera_index=camera_index,
            frame_width=camera_width,
            frame_height=camera_height,
            frame_fps=camera_fps,
        )
        cap.open()
        return cap, camera_index, "ffmpeg", "ffmpeg-avfoundation"

    cap, active_index, active_backend = open_verified_webcam_capture(
        camera_index=camera_index,
        camera_backend=camera_backend,
        frame_width=camera_width,
        frame_height=camera_height,
        frame_fps=camera_fps,
    )
    return cap, active_index, "opencv", active_backend


def capture_mode_candidates(preferred_mode: str) -> list[str]:
    modes: list[str] = []
    for mode in [preferred_mode, "ffmpeg", "opencv"]:
        normalized = mode.lower().strip()
        if normalized not in modes:
            modes.append(normalized)
    return modes


def recovery_capture_mode_candidates(current_mode: str) -> list[str]:
    current = current_mode.lower().strip()
    if current == "ffmpeg":
        return ["opencv", "ffmpeg"]
    return ["ffmpeg", "opencv"]


def build_frame_reader(capture_mode: str, cap) -> tuple[LatestFrameReader | None, int]:
    if not DEFAULT_USE_THREADED_READER:
        return None, -1
    if capture_mode != "opencv":
        return None, -1

    reader = LatestFrameReader(cap)
    reader.start()
    return reader, -1


def open_webcam_capture(
    camera_index: int,
    camera_backend: str,
    frame_width: int,
    frame_height: int,
    frame_fps: float,
    allow_index_fallback: bool = True,
) -> cv2.VideoCapture:
    backend_id = pick_camera_backend(camera_backend)
    cap = cv2.VideoCapture(camera_index, backend_id)
    if allow_index_fallback and not cap.isOpened() and camera_index == 0:
        cap.release()
        for fallback_index in (1, 2):
            cap = cv2.VideoCapture(fallback_index, backend_id)
            if cap.isOpened():
                print(f"Primary camera index 0 unavailable. Using index {fallback_index}.")
                break

    if not cap.isOpened():
        raise RuntimeError(
            "Could not open webcam. On macOS, verify Camera access is enabled for Terminal/VS Code in "
            "System Settings -> Privacy & Security -> Camera."
        )

    if frame_width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    if frame_height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    if frame_fps > 0:
        cap.set(cv2.CAP_PROP_FPS, frame_fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)

    return cap


def frame_mean_luma(frame: np.ndarray) -> float:
    if frame is None or frame.size == 0:
        return 0.0
    if len(frame.shape) == 2:
        return float(frame.mean())
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(gray.mean())


def frame_std_luma(frame: np.ndarray) -> float:
    if frame is None or frame.size == 0:
        return 0.0
    if len(frame.shape) == 2:
        return float(frame.std())
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(gray.std())


def is_black_frame(frame: np.ndarray) -> bool:
    mean_luma = frame_mean_luma(frame)
    std_luma = frame_std_luma(frame)
    if mean_luma <= DEFAULT_CAMERA_BLACK_MEAN_THRESHOLD:
        return True
    return mean_luma <= (DEFAULT_CAMERA_BLACK_MEAN_THRESHOLD * 3.0) and std_luma <= DEFAULT_CAMERA_BLACK_STD_THRESHOLD


def camera_backend_candidates(camera_backend: str) -> list[str]:
    backend = camera_backend.lower().strip()
    if backend == "auto":
        return ["avfoundation", "any"] if is_macos() else ["any"]
    if backend in {"avfoundation", "any"}:
        if is_macos():
            alternate = "any" if backend == "avfoundation" else "avfoundation"
            return [backend, alternate]
        return [backend]
    return ["any"]


def camera_index_candidates(preferred_index: int) -> list[int]:
    ordered = [preferred_index]
    for idx in range(0, DEFAULT_CAMERA_MAX_INDEX_TO_PROBE + 1):
        if idx not in ordered:
            ordered.append(idx)
    return ordered


def has_non_black_samples(cap: cv2.VideoCapture, sample_frames: int) -> bool:
    non_black = 0
    total = max(1, sample_frames)
    for _ in range(total):
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        if not is_black_frame(frame):
            non_black += 1
        time.sleep(0.005)
    return non_black >= max(3, total // 6)


def open_verified_webcam_capture(
    camera_index: int,
    camera_backend: str,
    frame_width: int,
    frame_height: int,
    frame_fps: float,
) -> tuple[cv2.VideoCapture, int, str]:
    size_candidates: list[tuple[int, int]] = []
    for width, height in [(frame_width, frame_height), (640, 480), (0, 0)]:
        if (width, height) not in size_candidates:
            size_candidates.append((width, height))

    for backend in camera_backend_candidates(camera_backend):
        for index in camera_index_candidates(camera_index):
            for width, height in size_candidates:
                try:
                    cap = open_webcam_capture(
                        camera_index=index,
                        camera_backend=backend,
                        frame_width=width,
                        frame_height=height,
                        frame_fps=frame_fps,
                        allow_index_fallback=False,
                    )
                except RuntimeError:
                    continue

                if has_non_black_samples(cap, sample_frames=DEFAULT_CAMERA_PROBE_FRAMES):
                    print(f"Using webcam index {index}, backend {backend}, size request {width}x{height}")
                    return cap, index, backend

                cap.release()

    raise RuntimeError(
        "No usable non-black webcam feed found. Check camera permissions, close other apps using the camera, "
        "or try a different webcam index/backend."
    )


def reopen_webcam_with_fallback(
    current_cap: cv2.VideoCapture,
    camera_index: int,
    camera_backend: str,
    camera_width: int,
    camera_height: int,
    camera_fps: float,
) -> tuple[cv2.VideoCapture, str]:
    current_cap.release()

    fallback_backend = "any"
    if camera_backend == "any":
        fallback_backend = "avfoundation" if is_macos() else "any"

    cap = open_webcam_capture(
        camera_index=camera_index,
        camera_backend=fallback_backend,
        frame_width=camera_width,
        frame_height=camera_height,
        frame_fps=camera_fps,
    )
    return cap, fallback_backend


def has_lap() -> bool:
    return importlib.util.find_spec("lap") is not None


def process_frame(
    frame,
    model: YOLO,
    device: str,
    tracking_conf: float,
    iou: float,
    imgsz: int,
    resolved_tracker: str,
    persist: bool,
    selected_classes: list[int] | None,
    use_tracking: bool,
    stable_assigner: "StableIdAssigner",
    track_smoother: "TrackSmoother",
    sticky_gate: "StickyConfidenceGate",
    frame_idx: int,
) -> tuple[list[dict], int]:
    if use_tracking:
        result_list = model.track(
            source=frame,
            conf=tracking_conf,
            iou=iou,
            imgsz=imgsz,
            tracker=resolved_tracker,
            persist=persist,
            rect=False,
            device=device,
            classes=selected_classes,
            verbose=False,
        )
    else:
        result_list = model.predict(
            source=frame,
            conf=tracking_conf,
            iou=iou,
            imgsz=imgsz,
            device=device,
            classes=selected_classes,
            verbose=False,
        )
    result = result_list[0] if result_list else None
    if result is None:
        return [], frame_idx + 1

    detections = extract_tracked_detections(result)
    detections = stable_assigner.assign(detections, frame_idx)
    detections = track_smoother.update(detections, frame_idx)
    filtered_detections = sticky_gate.filter(detections, frame_idx)
    if filtered_detections:
        detections = filtered_detections
    elif DEFAULT_STICKY_FALLBACK_TO_UNFILTERED:
        pass
    else:
        detections = filtered_detections
    return detections, frame_idx + 1


def pick_device(device_arg: str) -> str:
    if device_arg != "auto":
        return device_arg

    try:
        import torch

        if torch.cuda.is_available():
            return "cuda"

        if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            return "mps"
    except Exception:
        pass

    return "cpu"


def resolve_model_path(model_path: str) -> str:
    if os.path.exists(model_path):
        return model_path

    fallback = "yolov8n.pt"
    if os.path.exists(fallback):
        print(f"Model not found at '{model_path}'. Falling back to '{fallback}'.")
        return fallback

    raise FileNotFoundError(
        f"Model not found at '{model_path}' and fallback '{fallback}' is missing."
    )


def decode_ws_frame(payload: bytes, has_timestamp_header: bool) -> np.ndarray | None:
    jpeg_data = payload
    if has_timestamp_header and len(payload) > 8:
        try:
            struct.unpack("d", payload[:8])
            jpeg_data = payload[8:]
        except struct.error:
            jpeg_data = payload

    frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
    return frame


def _expand_user_path(path_value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path_value))).resolve()


def _find_latest_mp4(paths: Iterable[Path]) -> Path | None:
    candidates: list[Path] = []
    for path in paths:
        if path.exists() and path.is_file() and path.suffix.lower() == ".mp4":
            candidates.append(path)

    if not candidates:
        return None

    return max(candidates, key=lambda candidate: candidate.stat().st_mtime)


def resolve_input_video(input_video: str | None) -> Path:
    if input_video:
        input_path = _expand_user_path(input_video)
        if input_path.exists() and input_path.is_file() and input_path.suffix.lower() == ".mp4":
            return input_path

        raise FileNotFoundError(
            f"Input video not found or not an .mp4 file: {input_path}"
        )

    downloads_dir = Path.home() / "Downloads"
    latest_download = _find_latest_mp4(downloads_dir.glob("*.mp4"))
    if latest_download is not None:
        print(f"No input provided. Using latest Downloads video: {latest_download}")
        return latest_download

    raise FileNotFoundError(
        "No input video provided and no .mp4 files found in your Downloads folder."
    )


def build_output_path(input_path: Path, output_path: str | None) -> Path:
    if output_path:
        return Path(output_path)

    return input_path.with_name(f"{input_path.stem}_annotated.mp4")


def resolve_tracker_path(tracker_path: str) -> str:
    tracker_candidate = _expand_user_path(tracker_path)
    if tracker_candidate.exists() and tracker_candidate.is_file():
        return str(tracker_candidate)

    local_candidate = (Path(__file__).resolve().parent / tracker_path).resolve()
    if local_candidate.exists() and local_candidate.is_file():
        return str(local_candidate)

    return tracker_path


def normalize_model_names(model_names: dict | list) -> dict[int, str]:
    if isinstance(model_names, list):
        return {idx: name for idx, name in enumerate(model_names)}

    normalized: dict[int, str] = {}
    for key, value in model_names.items():
        normalized[int(key)] = str(value)
    return normalized


def parse_class_selector(selector: str | None, names_by_id: dict[int, str]) -> set[int]:
    if not selector:
        return set()

    valid_ids = set(names_by_id.keys())
    by_name = {name.lower(): idx for idx, name in names_by_id.items()}
    selected: set[int] = set()

    for token in selector.split(","):
        value = token.strip()
        if not value:
            continue

        if value.isdigit():
            class_id = int(value)
            if class_id not in valid_ids:
                raise ValueError(f"Unknown class id: {class_id}")
            selected.add(class_id)
            continue

        lowered = value.lower()
        if lowered not in by_name:
            raise ValueError(
                f"Unknown class name: '{value}'. Available: {', '.join(names_by_id.values())}"
            )
        selected.add(by_name[lowered])

    return selected


def resolve_classes(
    model: YOLO,
    include_classes: str | None,
    exclude_classes: str | None,
    person_only: bool,
) -> list[int] | None:
    names_by_id = normalize_model_names(model.names)
    include_ids = parse_class_selector(include_classes, names_by_id)
    exclude_ids = parse_class_selector(exclude_classes, names_by_id)

    if person_only:
        person_like: set[int] = set()
        for idx, name in names_by_id.items():
            lowered = name.lower()
            if lowered in PEOPLE_RELATED_EXACT:
                person_like.add(idx)
                continue
            if any(keyword in lowered for keyword in PEOPLE_RELATED_KEYWORDS):
                person_like.add(idx)
                continue
            if any(keyword in lowered for keyword in ACTION_CLASS_KEYWORDS):
                person_like.add(idx)

        if not person_like:
            raise ValueError("--person-only requested but no person-like class names were found in this model.")
        include_ids = include_ids.intersection(person_like) if include_ids else person_like

    if include_ids and exclude_ids:
        include_ids -= exclude_ids

    if include_ids:
        return sorted(include_ids)

    if exclude_ids:
        remaining = sorted(set(names_by_id.keys()) - exclude_ids)
        if not remaining:
            raise ValueError("All classes were excluded. Adjust --exclude-classes.")
        return remaining

    return None


def box_iou(box_a: tuple[float, float, float, float], box_b: tuple[float, float, float, float]) -> float:
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


def box_center(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x1, y1, x2, y2 = box
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def center_distance(box_a: tuple[float, float, float, float], box_b: tuple[float, float, float, float]) -> float:
    ax, ay = box_center(box_a)
    bx, by = box_center(box_b)
    dx = ax - bx
    dy = ay - by
    return (dx * dx + dy * dy) ** 0.5


def stable_color(stable_id: int) -> tuple[int, int, int]:
    return (
        int((stable_id * 37) % 255),
        int((stable_id * 17 + 91) % 255),
        int((stable_id * 67 + 53) % 255),
    )


class StableIdAssigner:
    def __init__(self, max_gap: int, min_iou: float, max_center_dist: float) -> None:
        self.max_gap = max_gap
        self.min_iou = min_iou
        self.max_center_dist = max_center_dist
        self.next_stable_id = 1
        self.tracker_to_stable: dict[int, int] = {}
        self.tracker_last_seen: dict[int, int] = {}
        self.stable_last_box: dict[int, tuple[float, float, float, float]] = {}
        self.stable_last_seen: dict[int, int] = {}

    def _match_existing_stable(
        self,
        box: tuple[float, float, float, float],
        used_stable_ids: set[int],
        frame_idx: int,
    ) -> int | None:
        best_stable: int | None = None
        best_score = -1.0

        for stable_id, prev_box in self.stable_last_box.items():
            if stable_id in used_stable_ids:
                continue

            age = frame_idx - self.stable_last_seen.get(stable_id, -999999)
            if age < 0 or age > self.max_gap:
                continue

            iou_score = box_iou(box, prev_box)
            dist = center_distance(box, prev_box)
            if iou_score < self.min_iou:
                continue
            if dist > self.max_center_dist:
                continue

            score = iou_score - (dist / max(self.max_center_dist, 1.0)) * 0.25
            if score > best_score:
                best_score = score
                best_stable = stable_id

        return best_stable

    def assign(self, detections: list[dict], frame_idx: int) -> list[dict]:
        used_stable_ids: set[int] = set()

        for det in detections:
            tracker_id = det["track_id"]
            box = det["box"]
            stable_id: int | None = None

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
            det["stable_id"] = stable_id

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


class TrackSmoother:
    def __init__(self, alpha: float, hold_frames: int, min_track_hits: int) -> None:
        self.alpha = max(0.0, min(alpha, 1.0))
        self.hold_frames = max(0, hold_frames)
        self.min_track_hits = max(1, min_track_hits)
        self.state: dict[int, dict] = {}

    def _smooth_box(
        self,
        previous: tuple[float, float, float, float],
        current: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        alpha = self.alpha
        inv = 1.0 - alpha
        return (
            previous[0] * inv + current[0] * alpha,
            previous[1] * inv + current[1] * alpha,
            previous[2] * inv + current[2] * alpha,
            previous[3] * inv + current[3] * alpha,
        )

    def update(self, detections: list[dict], frame_idx: int) -> list[dict]:
        visible: list[dict] = []
        seen_stable_ids: set[int] = set()

        for det in detections:
            stable_id = det.get("stable_id")
            if stable_id is None:
                continue

            current_box = det["box"]
            track_state = self.state.get(stable_id)
            if track_state is None:
                smoothed_box = current_box
                hits = 1
            else:
                smoothed_box = self._smooth_box(track_state["box"], current_box)
                hits = int(track_state["hits"]) + 1

            self.state[stable_id] = {
                "box": smoothed_box,
                "conf": det.get("conf"),
                "class_id": det.get("class_id", 0),
                "last_seen": frame_idx,
                "hits": hits,
            }
            seen_stable_ids.add(stable_id)

            if hits >= self.min_track_hits:
                visible.append(
                    {
                        **det,
                        "box": smoothed_box,
                        "hits": hits,
                        "ghost": False,
                    }
                )

        to_remove: list[int] = []
        for stable_id, track_state in self.state.items():
            if stable_id in seen_stable_ids:
                continue

            age = frame_idx - int(track_state["last_seen"])
            hits = int(track_state["hits"])
            if age <= self.hold_frames and hits >= self.min_track_hits:
                visible.append(
                    {
                        "box": track_state["box"],
                        "conf": track_state.get("conf"),
                        "class_id": int(track_state.get("class_id", 0)),
                        "track_id": None,
                        "stable_id": stable_id,
                        "hits": hits,
                        "ghost": True,
                    }
                )
            elif age > self.hold_frames:
                to_remove.append(stable_id)

        for stable_id in to_remove:
            self.state.pop(stable_id, None)

        return visible


class StickyConfidenceGate:
    def __init__(
        self,
        lock_on_conf: float,
        min_conf: float,
        max_gap: int,
        lock_class_ids: set[int] | None = None,
    ) -> None:
        self.lock_on_conf = max(0.0, min(lock_on_conf, 1.0))
        self.min_conf = max(0.0, min(min_conf, 1.0))
        self.max_gap = max(1, max_gap)
        self.lock_class_ids = lock_class_ids or set()
        self.locked_last_seen: dict[int, int] = {}

    def filter(self, detections: list[dict], frame_idx: int) -> list[dict]:
        visible: list[dict] = []

        for det in detections:
            stable_id = det.get("stable_id")
            conf = det.get("conf")
            class_id = det.get("class_id")
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


def extract_tracked_detections(result) -> list[dict]:
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []

    xyxy_values = boxes.xyxy.cpu().tolist()
    conf_values = boxes.conf.cpu().tolist() if boxes.conf is not None else [None] * len(xyxy_values)
    cls_values = boxes.cls.cpu().tolist() if boxes.cls is not None else [0.0] * len(xyxy_values)
    id_values = boxes.id.cpu().tolist() if boxes.id is not None else [None] * len(xyxy_values)

    detections: list[dict] = []
    for index, coords in enumerate(xyxy_values):
        track_id_value = id_values[index]
        track_id = int(track_id_value) if track_id_value is not None else None
        detections.append(
            {
                "box": (float(coords[0]), float(coords[1]), float(coords[2]), float(coords[3])),
                "conf": float(conf_values[index]) if conf_values[index] is not None else None,
                "class_id": int(cls_values[index]),
                "track_id": track_id,
                "stable_id": None,
            }
        )

    return detections


def draw_stable_labels(
    frame,
    detections: list[dict],
    names_by_id: dict[int, str],
    hide_labels: bool,
    hide_conf: bool,
    hide_boxes: bool,
    line_width: int | None,
) -> None:
    if hide_labels and hide_conf and hide_boxes:
        return

    for det in detections:
        stable_id = det.get("stable_id")
        if stable_id is None:
            continue

        x1, y1, x2, y2 = det["box"]
        frame_h, frame_w = frame.shape[:2]
        left = int(max(0, min(x1, frame_w - 1)))
        top = int(max(0, min(y1, frame_h - 1)))
        right = int(max(0, min(x2, frame_w - 1)))
        bottom = int(max(0, min(y2, frame_h - 1)))

        color = stable_color(int(stable_id))
        if det.get("ghost"):
            color = tuple(int(channel * 0.6) for channel in color)

        if not hide_boxes:
            thickness = line_width if line_width is not None else 2
            cv2.rectangle(frame, (left, top), (right, bottom), color, thickness)

        class_id = det["class_id"]
        class_name = names_by_id.get(class_id, str(class_id))
        text_parts: list[str] = []

        if not hide_labels:
            text_parts.append(f"{class_name} #{stable_id}")
        if not hide_conf and det.get("conf") is not None:
            text_parts.append(f"{det['conf']:.2f}")

        if not text_parts:
            continue

        anchor_x = left
        anchor_y = top

        text = " ".join(text_parts)
        font_scale = 0.5
        thickness = 1
        font = cv2.FONT_HERSHEY_SIMPLEX
        (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)

        box_top = max(anchor_y - text_h - baseline - 6, 0)
        box_bottom = min(box_top + text_h + baseline + 6, frame.shape[0] - 1)
        box_right = min(anchor_x + text_w + 8, frame.shape[1] - 1)

        cv2.rectangle(frame, (anchor_x, box_top), (box_right, box_bottom), color, -1)
        cv2.putText(
            frame,
            text,
            (anchor_x + 4, box_bottom - baseline - 3),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )


def run_video_inference(
    source: str,
    websocket_url: str,
    webcam_index: int,
    camera_backend: str,
    camera_width: int,
    camera_height: int,
    camera_fps: float,
    processing_fps: float,
    webcam_capture_mode: str,
    model_path: str,
    output_video: str | None,
    conf: float,
    imgsz: int,
    device_arg: str,
    iou: float,
    tracker: str,
    persist: bool,
    show: bool,
    classes_include: str | None,
    classes_exclude: str | None,
    person_only: bool,
    hide_labels: bool,
    hide_conf: bool,
    hide_boxes: bool,
    line_width: int | None,
    smooth_alpha: float,
    hold_frames: int,
    min_track_hits: int,
) -> None:
    resolved_model = resolve_model_path(model_path)
    output_path = Path(output_video) if output_video else None
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)

    device = pick_device(device_arg)
    print(f"Using device: {device}")
    print(f"Loading model: {resolved_model}")
    model = YOLO(resolved_model)
    names_by_id = normalize_model_names(model.names)
    resolved_tracker = resolve_tracker_path(tracker)
    selected_classes = resolve_classes(
        model=model,
        include_classes=classes_include,
        exclude_classes=classes_exclude,
        person_only=person_only,
    )

    source_mode = source.lower().strip()

    if source_mode == "websocket":
        print(f"Input WebSocket: {websocket_url}")
    else:
        print(f"Input Webcam: index={webcam_index}, backend={camera_backend}")
    print(f"Output:          {output_path if output_path is not None else 'disabled (lower latency)'}")
    print(f"Tracker: {resolved_tracker}")
    if selected_classes is not None:
        selected_names = ", ".join(f"{idx}:{names_by_id[idx]}" for idx in selected_classes)
        print(f"Classes: {selected_names}")

    use_tracking = True
    if "bytetrack" in str(resolved_tracker).lower() and not has_lap():
        use_tracking = False
        print(
            "Warning: 'lap' is not installed, so ByteTrack is disabled. "
            "Falling back to detection-only mode (still usable on webcam)."
        )

    frame_idx = 0
    stable_assigner = StableIdAssigner(
        max_gap=DEFAULT_STABLE_ID_MAX_GAP,
        min_iou=DEFAULT_STABLE_ID_MIN_IOU,
        max_center_dist=DEFAULT_STABLE_ID_MAX_CENTER_DIST,
    )
    track_smoother = TrackSmoother(
        alpha=smooth_alpha,
        hold_frames=hold_frames,
        min_track_hits=min_track_hits,
    )
    core_person_ids = {
        idx
        for idx, name in names_by_id.items()
        if name.lower() in {"person", "pedestrian", "people"}
    }
    sticky_gate = StickyConfidenceGate(
        lock_on_conf=DEFAULT_LOCK_ON_CONF,
        min_conf=DEFAULT_STICK_MIN_CONF,
        max_gap=max(hold_frames, DEFAULT_HOLD_FRAMES),
        lock_class_ids=core_person_ids,
    )
    tracking_conf = min(conf, DEFAULT_STICK_MIN_CONF)
    print(
        f"Sticky confidence mode enabled: lock-on >= {DEFAULT_LOCK_ON_CONF:.2f}, keep >= {DEFAULT_STICK_MIN_CONF:.2f}"
    )
    if core_person_ids:
        locked_names = ", ".join(names_by_id[class_id] for class_id in sorted(core_person_ids))
        print(f"Sticky lock classes: {locked_names}")
    effective_conf = tracking_conf if use_tracking else conf
    if use_tracking:
        print(f"Tracker input confidence set to {tracking_conf:.2f}")
    else:
        print(f"Detection confidence set to {effective_conf:.2f}")

    writer = None
    process_interval_sec = 1.0 / max(1.0, processing_fps)
    next_process_time = time.time()
    try:
        if source_mode == "webcam":
            capture_mode = webcam_capture_mode.lower().strip()
            print(f"Opening webcam index {webcam_index} (mode={capture_mode})")
            cap = None
            open_errors: list[str] = []
            for candidate_mode in capture_mode_candidates(capture_mode):
                try:
                    cap, active_index, capture_mode, active_backend = open_capture_by_mode(
                        capture_mode=candidate_mode,
                        camera_index=webcam_index,
                        camera_backend=camera_backend,
                        camera_width=camera_width,
                        camera_height=camera_height,
                        camera_fps=camera_fps,
                    )
                    print(f"Opened webcam using mode={capture_mode}, backend={active_backend}, index={active_index}")
                    break
                except RuntimeError as exc:
                    open_errors.append(f"{candidate_mode}: {exc}")

            if cap is None:
                raise RuntimeError("Could not open webcam using ffmpeg/opencv. " + " | ".join(open_errors))
            warmup_remaining = max(0, DEFAULT_CAMERA_WARMUP_FRAMES)
            black_streak = 0
            read_fail_streak = 0
            attempted_fallback = False
            frame_reader, last_frame_seq = build_frame_reader(capture_mode, cap)
            try:
                while True:
                    if frame_reader is not None:
                        ok, frame, last_frame_seq = frame_reader.read_latest(
                            last_seq=last_frame_seq,
                            timeout_sec=DEFAULT_FRAME_READ_TIMEOUT_SEC,
                        )
                    else:
                        ok, frame = cap.read()
                    if not ok or frame is None:
                        read_fail_streak += 1
                        if read_fail_streak >= DEFAULT_WEBCAM_READ_FAIL_STREAK_LIMIT:
                            print("Webcam read failed repeatedly. Reopening webcam stream...")
                            if frame_reader is not None:
                                frame_reader.stop()
                            cap.release()
                            time.sleep(DEFAULT_WEBCAM_REOPEN_DELAY_SEC)
                            reopened = False
                            for recovery_mode in recovery_capture_mode_candidates(capture_mode):
                                try:
                                    cap, active_index, capture_mode, active_backend = open_capture_by_mode(
                                        capture_mode=recovery_mode,
                                        camera_index=active_index,
                                        camera_backend=camera_backend,
                                        camera_width=camera_width,
                                        camera_height=camera_height,
                                        camera_fps=camera_fps,
                                    )
                                    print(
                                        "Reopened webcam using "
                                        f"mode={capture_mode}, backend={active_backend}, index={active_index}"
                                    )
                                    reopened = True
                                    break
                                except RuntimeError:
                                    continue
                            if not reopened:
                                raise RuntimeError("Unable to reopen webcam stream in any mode (ffmpeg/opencv).")
                            frame_reader, last_frame_seq = build_frame_reader(capture_mode, cap)
                            read_fail_streak = 0
                            warmup_remaining = max(0, DEFAULT_CAMERA_WARMUP_FRAMES)
                            continue
                        time.sleep(0.03)
                        continue
                    read_fail_streak = 0

                    if warmup_remaining > 0:
                        warmup_remaining -= 1
                        continue

                    now = time.time()
                    if now < next_process_time:
                        if show:
                            cv2.imshow("YOLO Mac Webcam Detection", frame)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                return
                        continue
                    next_process_time = now + process_interval_sec

                    frame_luma = frame_mean_luma(frame)
                    if frame_luma <= DEFAULT_CAMERA_BLACK_MEAN_THRESHOLD:
                        black_streak += 1
                    else:
                        black_streak = 0

                    if black_streak >= DEFAULT_CAMERA_BLACK_STREAK_LIMIT and not attempted_fallback:
                        print(
                            "Webcam frames appear black. Switching capture mode "
                            f"from {capture_mode} for recovery."
                        )
                        if frame_reader is not None:
                            frame_reader.stop()
                        cap.release()
                        switched = False
                        for recovery_mode in recovery_capture_mode_candidates(capture_mode):
                            try:
                                cap, active_index, capture_mode, active_backend = open_capture_by_mode(
                                    capture_mode=recovery_mode,
                                    camera_index=active_index,
                                    camera_backend=camera_backend,
                                    camera_width=camera_width,
                                    camera_height=camera_height,
                                    camera_fps=camera_fps,
                                )
                                print(
                                    "Recovered webcam stream using "
                                    f"mode={capture_mode}, backend={active_backend}, index={active_index}"
                                )
                                switched = True
                                break
                            except RuntimeError:
                                continue
                        if not switched:
                            raise RuntimeError("Unable to recover from black webcam frames in any mode (ffmpeg/opencv).")
                        frame_reader, last_frame_seq = build_frame_reader(capture_mode, cap)
                        attempted_fallback = True
                        warmup_remaining = max(0, DEFAULT_CAMERA_WARMUP_FRAMES)
                        black_streak = 0
                        continue

                    if output_path is not None and writer is None:
                        frame_h, frame_w = frame.shape[:2]
                        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                        fps = cap.get(cv2.CAP_PROP_FPS)
                        out_fps = fps if fps and fps > 1.0 else camera_fps
                        writer = cv2.VideoWriter(
                            str(output_path),
                            fourcc,
                            out_fps,
                            (frame_w, frame_h),
                        )
                        if not writer.isOpened():
                            raise RuntimeError(f"Could not create output video: {output_path}")
                        print(f"Output size: {frame_w}x{frame_h} @ {out_fps:.2f} FPS")

                    detections, frame_idx = process_frame(
                        frame=frame,
                        model=model,
                        device=device,
                        tracking_conf=effective_conf,
                        iou=iou,
                        imgsz=imgsz,
                        resolved_tracker=resolved_tracker,
                        persist=persist,
                        selected_classes=selected_classes,
                        use_tracking=use_tracking,
                        stable_assigner=stable_assigner,
                        track_smoother=track_smoother,
                        sticky_gate=sticky_gate,
                        frame_idx=frame_idx,
                    )

                    annotated_frame = frame.copy()
                    draw_stable_labels(
                        frame=annotated_frame,
                        detections=detections,
                        names_by_id=names_by_id,
                        hide_labels=hide_labels,
                        hide_conf=hide_conf,
                        hide_boxes=hide_boxes,
                        line_width=line_width,
                    )
                    if writer is not None:
                        writer.write(annotated_frame)

                    if show:
                        cv2.imshow("YOLO Mac Webcam Detection", annotated_frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            return

                    if frame_idx % 60 == 0:
                        print(f"Processed {frame_idx} frames")
            finally:
                if frame_reader is not None:
                    frame_reader.stop()
                cap.release()

        elif source_mode == "websocket":
            while True:
                print(f"Connecting to camera websocket: {websocket_url}")
                try:
                    with ws_connect(websocket_url, open_timeout=10) as ws:
                        print("Websocket connected.")

                        while True:
                            payload = ws.recv(timeout=DEFAULT_WS_RECEIVE_TIMEOUT_SEC)
                            if not isinstance(payload, (bytes, bytearray)):
                                continue

                            frame = decode_ws_frame(payload, DEFAULT_WS_HAS_TIMESTAMP_HEADER)
                            if frame is None:
                                continue

                            if output_path is not None and writer is None:
                                frame_h, frame_w = frame.shape[:2]
                                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                                writer = cv2.VideoWriter(
                                    str(output_path),
                                    fourcc,
                                    DEFAULT_WS_OUTPUT_FPS,
                                    (frame_w, frame_h),
                                )
                                if not writer.isOpened():
                                    raise RuntimeError(f"Could not create output video: {output_path}")
                                print(f"Output size: {frame_w}x{frame_h} @ {DEFAULT_WS_OUTPUT_FPS:.2f} FPS")

                            detections, frame_idx = process_frame(
                                frame=frame,
                                model=model,
                                device=device,
                                tracking_conf=effective_conf,
                                iou=iou,
                                imgsz=imgsz,
                                resolved_tracker=resolved_tracker,
                                persist=persist,
                                selected_classes=selected_classes,
                                use_tracking=use_tracking,
                                stable_assigner=stable_assigner,
                                track_smoother=track_smoother,
                                sticky_gate=sticky_gate,
                                frame_idx=frame_idx,
                            )

                            annotated_frame = frame.copy()
                            draw_stable_labels(
                                frame=annotated_frame,
                                detections=detections,
                                names_by_id=names_by_id,
                                hide_labels=hide_labels,
                                hide_conf=hide_conf,
                                hide_boxes=hide_boxes,
                                line_width=line_width,
                            )
                            if writer is not None:
                                writer.write(annotated_frame)

                            if show:
                                cv2.imshow("YOLO Webcam WebSocket Detection", annotated_frame)
                                if cv2.waitKey(1) & 0xFF == ord("q"):
                                    return

                            if frame_idx % 60 == 0:
                                print(f"Processed {frame_idx} frames")

                except (ConnectionClosed, TimeoutError) as exc:
                    print(f"Websocket disconnected ({exc}). Reconnecting in {DEFAULT_WS_RECONNECT_DELAY_SEC:.1f}s...")
                    time.sleep(DEFAULT_WS_RECONNECT_DELAY_SEC)
                except OSError as exc:
                    print(f"Websocket connection error ({exc}). Reconnecting in {DEFAULT_WS_RECONNECT_DELAY_SEC:.1f}s...")
                    time.sleep(DEFAULT_WS_RECONNECT_DELAY_SEC)
        else:
            raise ValueError("Unsupported --source. Use: webcam or websocket")
    finally:
        if writer is not None:
            writer.release()

    if show:
        cv2.destroyAllWindows()

    print("Done.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run YOLO inference on webcam or websocket stream and save an annotated MP4 output."
    )
    parser.add_argument(
        "--source",
        choices=["webcam", "websocket"],
        default=DEFAULT_SOURCE,
        help="Input source type (default: webcam)",
    )
    parser.add_argument(
        "--websocket-url",
        default=DEFAULT_WS_URL,
        help="WebSocket URL when --source websocket",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=DEFAULT_CAMERA_INDEX,
        help="Camera index for webcam mode (default: 0)",
    )
    parser.add_argument(
        "--camera-backend",
        choices=["auto", "avfoundation", "any"],
        default=DEFAULT_CAMERA_BACKEND,
        help="OpenCV camera backend (default: auto; uses avfoundation on macOS)",
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=DEFAULT_CAMERA_WIDTH,
        help="Requested camera width in pixels (default: 1280)",
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=DEFAULT_CAMERA_HEIGHT,
        help="Requested camera height in pixels (default: 720)",
    )
    parser.add_argument(
        "--camera-fps",
        type=float,
        default=DEFAULT_CAMERA_FPS,
        help="Requested camera fps in webcam mode (default: 15)",
    )
    parser.add_argument(
        "--processing-fps",
        type=float,
        default=DEFAULT_PROCESSING_FPS,
        help="Target inference/processing fps (default: 15)",
    )
    parser.add_argument(
        "--webcam-capture-mode",
        choices=["ffmpeg", "opencv"],
        default=DEFAULT_WEBCAM_CAPTURE_MODE,
        help="Webcam ingest mode (default: opencv; falls back to ffmpeg automatically)",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Path to YOLO model weights (.pt). Overrides --model-family when provided.",
    )
    parser.add_argument(
        "--model-family",
        choices=["new", "old"],
        default=DEFAULT_MODEL_FAMILY,
        help="Select default weights: 'new' uses yolo26-p2 run, 'old' uses yolov8n-p2 run.",
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_VIDEO,
        help="Path to output annotated .mp4 (default: disabled for lower latency)",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=DEFAULT_CONFIDENCE,
        help="Confidence threshold (default: 0.35)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=DEFAULT_IMAGE_SIZE,
        help="Inference image size (default: 640)",
    )
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help="Device: auto, cpu, cuda, mps (default: auto)",
    )
    parser.add_argument(
        "--iou",
        type=float,
        default=DEFAULT_IOU,
        help="NMS IoU threshold (default: 0.5)",
    )
    parser.add_argument(
        "--tracker",
        default=DEFAULT_TRACKER,
        help="Tracker config file (default: bytetrack.yaml)",
    )
    parser.add_argument(
        "--persist",
        action="store_true",
        default=DEFAULT_PERSIST,
        help="Persist tracks across frames (default: enabled)",
    )
    parser.add_argument(
        "--no-persist",
        action="store_false",
        dest="persist",
        help="Disable track persistence",
    )
    parser.add_argument(
        "--classes",
        default=DEFAULT_INCLUDE_CLASSES,
        help="Comma-separated class ids/names to keep (example: 'pedestrian,people' or '0,1')",
    )
    parser.add_argument(
        "--exclude-classes",
        default=DEFAULT_EXCLUDE_CLASSES,
        help="Comma-separated class ids/names to hide",
    )
    parser.add_argument(
        "--person-only",
        action="store_true",
        default=DEFAULT_PERSON_ONLY,
        help="Only detect people-related classes and action-like classes when present",
    )
    parser.add_argument(
        "--hide-labels",
        action="store_true",
        default=DEFAULT_HIDE_LABELS,
        help="Hide class labels on boxes",
    )
    parser.add_argument(
        "--hide-conf",
        action="store_true",
        default=DEFAULT_HIDE_CONF,
        help="Hide confidence scores on labels",
    )
    parser.add_argument(
        "--hide-boxes",
        action="store_true",
        default=DEFAULT_HIDE_BOXES,
        help="Hide box outlines (useful for testing only labels)",
    )
    parser.add_argument(
        "--line-width",
        type=int,
        default=DEFAULT_LINE_WIDTH,
        help="Bounding box line width in pixels (default: auto)",
    )
    parser.add_argument(
        "--smooth-alpha",
        type=float,
        default=DEFAULT_SMOOTH_ALPHA,
        help="Temporal smoothing factor for boxes, 0-1 (default: 0.60)",
    )
    parser.add_argument(
        "--hold-frames",
        type=int,
        default=DEFAULT_HOLD_FRAMES,
        help="How many frames to hold last box when detection drops (default: 12)",
    )
    parser.add_argument(
        "--min-track-hits",
        type=int,
        default=DEFAULT_MIN_TRACK_HITS,
        help="Minimum consecutive hits before showing a track (default: 2)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        default=DEFAULT_SHOW_PREVIEW,
        help="Show live annotated preview while processing (default: enabled)",
    )
    parser.add_argument(
        "--no-show",
        action="store_false",
        dest="show",
        help="Disable live preview window",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    selected_model = args.model
    if not selected_model:
        selected_model = DEFAULT_NEW_MODEL_PATH if args.model_family == "new" else DEFAULT_OLD_MODEL_PATH

    run_video_inference(
        source=args.source,
        websocket_url=args.websocket_url,
        webcam_index=args.camera_index,
        camera_backend=args.camera_backend,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        camera_fps=args.camera_fps,
        processing_fps=args.processing_fps,
        webcam_capture_mode=args.webcam_capture_mode,
        model_path=selected_model,
        output_video=args.output,
        conf=args.conf,
        imgsz=args.imgsz,
        device_arg=args.device,
        iou=args.iou,
        tracker=args.tracker,
        persist=args.persist,
        show=args.show,
        classes_include=args.classes,
        classes_exclude=args.exclude_classes,
        person_only=args.person_only,
        hide_labels=args.hide_labels,
        hide_conf=args.hide_conf,
        hide_boxes=args.hide_boxes,
        line_width=args.line_width,
        smooth_alpha=args.smooth_alpha,
        hold_frames=args.hold_frames,
        min_track_hits=args.min_track_hits,
    )
