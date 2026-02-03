import cv2
import numpy as np
from dataclasses import dataclass

@dataclass
class IRLockResult:
    locked: bool
    error_x: float      # -1.0 (left) → +1.0 (right)
    error_y: float      # -1.0 (up)   → +1.0 (down)
    area: float         # Blob area (proxy for distance)
    confidence: float  # 0.0 → 1.0


def process_ir_frame(frame: np.ndarray) -> IRLockResult:
    """
    Process a single camera frame and attempt to lock onto an IR beacon.
    Assumes IR beacon appears as the brightest object in the frame.
    """

    h, w = frame.shape[:2]
    cx_img, cy_img = w // 2, h // 2

    # Convert to grayscale (IR shows up bright)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Adaptive threshold for varying lighting
    _, thresh = cv2.threshold(
        blurred,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # Morphological cleanup
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return IRLockResult(False, 0.0, 0.0, 0.0, 0.0)

    # Select brightest / largest contour
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < 50:  # Reject tiny noise blobs
        return IRLockResult(False, 0.0, 0.0, area, 0.0)

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return IRLockResult(False, 0.0, 0.0, area, 0.0)

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    # Normalised pixel error
    error_x = (cx - cx_img) / cx_img
    error_y = (cy - cy_img) / cy_img

    # Confidence based on blob dominance
    confidence = min(1.0, area / (0.05 * w * h))

    return IRLockResult(
        locked=True,
        error_x=error_x,
        error_y=error_y,
        area=area,
        confidence=confidence
    )
