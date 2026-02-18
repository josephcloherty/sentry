import argparse
import os
import struct
import sys
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

DEFAULT_INPUT_VIDEO = r"C:\Users\josep\Downloads\test720.mp4"
DEFAULT_NEW_MODEL_PATH = r"C:\Users\josep\Desktop\AERO420\runs\detect\runs\detect\VisDrone_Project\yolo11n_p2_visdrone2\weights\best.pt"
DEFAULT_OLD_MODEL_PATH = r"C:\Users\josep\Desktop\AERO420\runs\detect\_archive\2026-02-16_yolov8n_visdrone\yolov8n_visdrone\weights\best.pt"
DEFAULT_MODEL_FAMILY = "old"
DEFAULT_OUTPUT_VIDEO = None#r"C:\Users\josep\Downloads\test1080_annotated.mp4"
DEFAULT_CONFIDENCE = 0.35
DEFAULT_IMAGE_SIZE = 800
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

DEFAULT_INCLUDE_CLASSES = None
DEFAULT_EXCLUDE_CLASSES = None
DEFAULT_PERSON_ONLY = False

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
    if output_video:
        output_path = Path(output_video)
    else:
        output_path = Path.cwd() / "websocket_annotated.mp4"
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

    print(f"Input WebSocket: {DEFAULT_WS_URL}")
    print(f"Output:          {output_path}")
    print(f"Tracker: {resolved_tracker}")
    if selected_classes is not None:
        selected_names = ", ".join(f"{idx}:{names_by_id[idx]}" for idx in selected_classes)
        print(f"Classes: {selected_names}")

    if "bytetrack" in str(resolved_tracker).lower():
        try:
            import lap  # noqa: F401
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "ByteTrack requires the 'lap' package in this interpreter. "
                f"Install it with: {sys.executable} -m pip install lap>=0.5.12 and rerun."
            ) from exc

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
    print(f"Tracker input confidence set to {tracking_conf:.2f}")

    writer = None
    try:
        while True:
            print(f"Connecting to camera websocket: {DEFAULT_WS_URL}")
            try:
                with ws_connect(DEFAULT_WS_URL, open_timeout=10) as ws:
                    print("Websocket connected.")

                    while True:
                        payload = ws.recv(timeout=DEFAULT_WS_RECEIVE_TIMEOUT_SEC)
                        if not isinstance(payload, (bytes, bytearray)):
                            continue

                        frame = decode_ws_frame(payload, DEFAULT_WS_HAS_TIMESTAMP_HEADER)
                        if frame is None:
                            continue

                        if writer is None:
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
                        result = result_list[0] if result_list else None
                        if result is None:
                            continue

                        detections = extract_tracked_detections(result)
                        detections = stable_assigner.assign(detections, frame_idx)
                        detections = track_smoother.update(detections, frame_idx)
                        detections = sticky_gate.filter(detections, frame_idx)

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
                        writer.write(annotated_frame)

                        if show:
                            cv2.imshow("YOLO Webcam WebSocket Detection", annotated_frame)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                return

                        frame_idx += 1
                        if frame_idx % 60 == 0:
                            print(f"Processed {frame_idx} frames")

            except (ConnectionClosed, TimeoutError) as exc:
                print(f"Websocket disconnected ({exc}). Reconnecting in {DEFAULT_WS_RECONNECT_DELAY_SEC:.1f}s...")
                time.sleep(DEFAULT_WS_RECONNECT_DELAY_SEC)
            except OSError as exc:
                print(f"Websocket connection error ({exc}). Reconnecting in {DEFAULT_WS_RECONNECT_DELAY_SEC:.1f}s...")
                time.sleep(DEFAULT_WS_RECONNECT_DELAY_SEC)
    finally:
        if writer is not None:
            writer.release()

    if show:
        cv2.destroyAllWindows()

    print("Done.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run YOLO inference on a websocket camera feed and save an annotated MP4 output."
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
        help="Path to output annotated .mp4 (default: <input>_annotated.mp4)",
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
