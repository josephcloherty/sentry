import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional

# ===== Configuration (tunable) =====
OPENCV_MIN_AREA_ABS = 200
OPENCV_MIN_AREA_RATIO = 0.0008
OPENCV_MAX_DEVIATION = 0.25
OPENCV_REQUIRE_NESTING = False

@dataclass
class FiducialLockResult:
    locked: bool
    error_x: float      # -1.0 (left) → +1.0 (right)
    error_y: float      # -1.0 (up)   → +1.0 (down)
    area: float         # Blob area (proxy for distance)
    confidence: float  # 0.0 → 1.0
    fiducial_id: Optional[int] = None


# Try to import AprilTag detector (using pupil-apriltags)
_APRILTAG_AVAILABLE = False
_AT_DETECTOR = None
try:
    from pupil_apriltags import Detector
    _APRILTAG_AVAILABLE = True
    _AT_DETECTOR = Detector()
except Exception as e:
    print(f"Warning: AprilTag library not available: {e}")


def process_fiducial_frame(frame: np.ndarray) -> FiducialLockResult:
    """Detect AprilTag, QR, or brightest blob and return lock metrics."""

    h, w = frame.shape[:2]
    cx_img, cy_img = w // 2, h // 2

    if frame.ndim == 2:
        gray = frame
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 1) Try AprilTag detection (PRIMARY METHOD - most accurate)
    if _APRILTAG_AVAILABLE and _AT_DETECTOR is not None:
        try:
            dets = _AT_DETECTOR.detect(gray)
            if dets:
                # With nested AprilTags, select by TAG ID
                # Try highest ID first (outer tag might be numbered higher)
                best = max(dets, key=lambda d: d.tag_id)
                
                # pupil-apriltags returns corners as (x, y) tuples
                corners = np.array(best.corners, dtype=np.float32)
                pts = corners.astype(np.int32)
                
                area = float(cv2.contourArea(pts))
                M = cv2.moments(pts)
                if M.get('m00', 0) != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    error_x = (cx - cx_img) / cx_img
                    error_y = (cy - cy_img) / cy_img
                    
                    # High confidence for real AprilTag detections
                    confidence = 0.95
                    
                    result = FiducialLockResult(
                        True, error_x, error_y, area, confidence, 
                        int(best.tag_id) if hasattr(best, 'tag_id') else None
                    )
                    # Attach corners for visualization
                    setattr(result, 'corners', pts)
                    return result
        except Exception as e:
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
