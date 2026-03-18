import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional

# ===== Configuration (tunable) =====
# OpenCV fallback tuning
OPENCV_MIN_AREA_ABS = 400
OPENCV_MIN_AREA_RATIO = 0.0006
OPENCV_MAX_DEVIATION = 0.12  # stricter squareness requirement
OPENCV_REQUIRE_NESTING = False

# AprilTag detector tuning (for pupil_apriltags)
# - families: tag family to detect (tag36h11 is common)
# - quad_decimate: 1.0 = full resolution, >1 = faster but less accurate on small tags
# - quad_sigma: gaussian blur applied to image for quad detection (0.0 = none)
# - refine_edges: enable edge refinement
AT_FAMILIES = 'tag36h11'
AT_NTHREADS = 2
AT_QUAD_DECIMATE = 1.0
AT_QUAD_SIGMA = 0.8
AT_REFINE_EDGES = True
AT_DECODE_SHARPENING = 0.25

# Multi-pass detection settings
# Running detection at multiple preprocessed versions of the frame dramatically
# improves detection at steep angles and in poor lighting.
#
# Pass 1 – CLAHE only (base pass, good contrast)
# Pass 2 – Unsharp mask sharpening (enhances foreshortened edges at angles)
# Pass 3 – 2× upscale crop (helps when the tag appears small / far away)
#
# Deduplication: if the same tag_id is found in multiple passes, keep the
# detection with the highest decision_margin so each tag appears only once.
AT_MULTIPASS_ENABLED = True
AT_UPSCALE_FACTOR = 2.0       # upscale factor for pass 3
AT_UPSCALE_MIN_DIM = 320      # only upscale if shortest frame dim < this (px)
AT_SHARPEN_AMOUNT = 1.5       # unsharp mask strength (higher = sharper edges)

@dataclass
class FiducialLockResult:
    locked: bool
    error_x: float      # -1.0 (left) → +1.0 (right)
    error_y: float      # -1.0 (up)   → +1.0 (down)
    area: float         # Blob area (proxy for distance)
    confidence: float  # 0.0 → 1.0
    fiducial_id: Optional[int] = None


# Try to import AprilTag detector (prefer pupil_apriltags)
_APRILTAG_AVAILABLE = False
_AT_DETECTOR = None
try:
    from pupil_apriltags import Detector
    try:
        _AT_DETECTOR = Detector(
            families=AT_FAMILIES,
            nthreads=AT_NTHREADS,
            quad_decimate=AT_QUAD_DECIMATE,
            quad_sigma=AT_QUAD_SIGMA,
            refine_edges=AT_REFINE_EDGES,
            decode_sharpening=AT_DECODE_SHARPENING,
        )
    except Exception:
        # Fallback to simple constructor
        _AT_DETECTOR = Detector()
    _APRILTAG_AVAILABLE = True
except Exception as e:
    print(f"Warning: pupil_apriltags not available: {e}")
    _APRILTAG_AVAILABLE = False


# ===== Preprocessing helpers =====

def _prep_clahe(gray: np.ndarray) -> np.ndarray:
    """Apply CLAHE to improve local contrast."""
    try:
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        return clahe.apply(gray)
    except Exception:
        return gray


def _prep_sharpen(gray: np.ndarray, amount: float = 1.5) -> np.ndarray:
    """Unsharp mask: sharpens edges which helps at steep angles."""
    try:
        blur = cv2.GaussianBlur(gray, (0, 0), 3)
        sharp = cv2.addWeighted(gray, 1.0 + amount, blur, -amount, 0)
        return sharp
    except Exception:
        return gray


def _prep_upscale(gray: np.ndarray, factor: float = 2.0) -> tuple:
    """Upscale the frame. Returns (upscaled_gray, scale_factor_used)."""
    try:
        h, w = gray.shape[:2]
        new_w = int(w * factor)
        new_h = int(h * factor)
        up = cv2.resize(gray, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
        return up, factor
    except Exception:
        return gray, 1.0


def _dets_to_dicts(dets, h: int, w: int, scale: float = 1.0) -> list:
    """Convert pupil_apriltags detections to dicts, scaling corners back to original coords."""
    cx_img, cy_img = w // 2, h // 2
    out = []
    for d in dets:
        try:
            corners = np.array(d.corners, dtype=np.float32) / scale
            pts = corners.astype(np.int32)
            area = float(cv2.contourArea(pts))
            M = cv2.moments(pts)
            if M.get('m00', 0) == 0:
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
            else:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            error_x = (cx - cx_img) / max(cx_img, 1)
            error_y = (cy - cy_img) / max(cy_img, 1)
            margin = float(getattr(d, 'decision_margin', 0.0))
            tag_id = int(d.tag_id) if getattr(d, 'tag_id', None) is not None else None
            out.append({
                'tag_id': tag_id,
                'corners': pts.tolist(),
                'area': area,
                'decision_margin': margin,
                'error_x': float(error_x),
                'error_y': float(error_y),
            })
        except Exception:
            continue
    return out


def _merge_detections(passes: list) -> list:
    """Merge detections from multiple preprocessing passes.

    For each tag_id, keep the detection with the highest decision_margin.
    Detections with tag_id=None are only kept if no tagged detections exist.
    """
    best: dict = {}  # tag_id -> det
    untagged = []
    for det in passes:
        tid = det.get('tag_id')
        margin = float(det.get('decision_margin', 0.0))
        if tid is None:
            untagged.append(det)
        elif tid not in best or margin > float(best[tid].get('decision_margin', 0.0)):
            best[tid] = det
    merged = list(best.values())
    if not merged:
        merged = untagged
    return merged


def _run_multipass(gray_orig: np.ndarray, h: int, w: int) -> list:
    """Run AprilTag detection on multiple preprocessed versions and merge results."""
    all_dets = []

    # Pass 1: CLAHE only
    try:
        g1 = _prep_clahe(gray_orig)
        dets1 = _AT_DETECTOR.detect(g1)
        all_dets.extend(_dets_to_dicts(dets1, h, w, scale=1.0))
    except Exception:
        pass

    # Pass 2: CLAHE + sharpen (helps with low-contrast edges at angles)
    try:
        g2 = _prep_sharpen(_prep_clahe(gray_orig), AT_SHARPEN_AMOUNT)
        dets2 = _AT_DETECTOR.detect(g2)
        all_dets.extend(_dets_to_dicts(dets2, h, w, scale=1.0))
    except Exception:
        pass

    # Pass 3: upscale (helps when tag is small / far away)
    try:
        short_dim = min(h, w)
        if short_dim < AT_UPSCALE_MIN_DIM or True:  # always try upscale
            g3, scale = _prep_upscale(_prep_clahe(gray_orig), AT_UPSCALE_FACTOR)
            dets3 = _AT_DETECTOR.detect(g3)
            all_dets.extend(_dets_to_dicts(dets3, h, w, scale=scale))
    except Exception:
        pass

    return _merge_detections(all_dets)


def detect_all_apriltags(frame: np.ndarray) -> list:
    """Return a list of detected AprilTag dicts for visualization/debugging.

    Each dict contains: tag_id, corners (4x2), area, decision_margin, error_x, error_y.
    Uses multi-pass detection when AT_MULTIPASS_ENABLED is True.
    """
    if not _APRILTAG_AVAILABLE or _AT_DETECTOR is None:
        return []

    try:
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    except Exception:
        return []

    h, w = gray.shape[:2]

    if AT_MULTIPASS_ENABLED:
        return _run_multipass(gray, h, w)

    # Single-pass fallback
    try:
        dets = _AT_DETECTOR.detect(_prep_clahe(gray))
        return _dets_to_dicts(dets, h, w, scale=1.0)
    except Exception:
        return []


def process_fiducial_frame(frame: np.ndarray) -> FiducialLockResult:
    """Detect AprilTag, QR, or brightest blob and return lock metrics."""

    h, w = frame.shape[:2]
    cx_img, cy_img = w // 2, h // 2

    if frame.ndim == 2:
        gray = frame
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 1) Try AprilTag detection (PRIMARY METHOD - most accurate)
    #    Uses multi-pass preprocessing for better steep-angle consistency.
    if _APRILTAG_AVAILABLE and _AT_DETECTOR is not None:
        try:
            if AT_MULTIPASS_ENABLED:
                dets = _run_multipass(gray, h, w)
            else:
                # Single-pass CLAHE fallback
                g = _prep_clahe(gray)
                dets = _dets_to_dicts(_AT_DETECTOR.detect(g), h, w, scale=1.0)

            if dets:
                # Prefer the largest detected tag (outer); break ties by decision_margin.
                best = max(dets, key=lambda d: (d.get('area', 0.0), d.get('decision_margin', 0.0)))

                area = float(best.get('area', 0.0))
                error_x = float(best.get('error_x', 0.0))
                error_y = float(best.get('error_y', 0.0))
                margin = float(best.get('decision_margin', 0.0))

                # Map decision_margin -> confidence (tunable)
                confidence = 0.6 + min(margin / 20.0, 0.35)
                confidence = float(min(0.99, max(0.0, confidence)))

                tag_id = best.get('tag_id')
                result = FiducialLockResult(
                    True, error_x, error_y, area, confidence,
                    int(tag_id) if tag_id is not None else None
                )
                # Attach corners for visualization
                try:
                    corners = np.array(best.get('corners', []), dtype=np.int32)
                    setattr(result, 'corners', corners)
                except Exception:
                    pass
                return result
        except Exception:
            pass

    # 2) Try OpenCV quad fallback
    try:
        from fiducial_opencv import detect_apriltag_opencv
        min_area = max(OPENCV_MIN_AREA_ABS, int(OPENCV_MIN_AREA_RATIO * w * h))
        res = detect_apriltag_opencv(
            frame,
            min_area=min_area,
            max_deviation=OPENCV_MAX_DEVIATION,
            require_nesting=OPENCV_REQUIRE_NESTING
        )
        if res and res.get('locked'):
            corners = res.get('corners')
            area = float(res.get('area', 0.0))
            M = cv2.moments(corners)
            if M.get('m00', 0) != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                error_x = (cx - cx_img) / cx_img
                error_y = (cy - cy_img) / cy_img
                confidence = float(res.get('confidence', 0.0))
                out = FiducialLockResult(True, error_x, error_y, area, confidence, None)
                setattr(out, 'corners', corners)
                return out
    except Exception:
        pass

    # 3) Fallback to brightest blob (IR-like detection)
    # NOTE: Disabled for image-based AprilTag detection
    # If you need IR LED tracking, enable this section
    
    return FiducialLockResult(False, 0.0, 0.0, 0.0, 0.0, None)