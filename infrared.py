import cv2
import numpy as np
from dataclasses import dataclass

@dataclass
class IRLockResult:
    locked: bool
    cx: int            # pixel x coordinate of detected center
    cy: int            # pixel y coordinate of detected center
    error_x: float      # -1.0 (left) → +1.0 (right)
    error_y: float      # -1.0 (up)   → +1.0 (down)
    area: float         # Blob area (proxy for distance)
    confidence: float  # 0.0 → 1.0


def process_ir_frame(frame: np.ndarray) -> IRLockResult:
    """
    Process a single camera frame and attempt to lock onto an IR beacon.
    Assumes IR beacon appears as the brightest object in the frame.
    """

    # Ensure we operate on a single-channel (grayscale) image
    if isinstance(frame, np.ndarray):
        if frame.ndim == 2:
            # Check if this is YUV420 format (height is 1.5x width for planar YUV420)
            h, w = frame.shape
            if h == int(w * 1.5):
                # Extract Y plane only (top 2/3 of the frame)
                gray = frame[:w, :].copy()
            else:
                gray = frame
        elif frame.ndim == 3:
            if frame.shape[2] == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            elif frame.shape[2] == 4:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)
            else:
                gray = frame[..., 0]
        else:
            gray = frame
    else:
        # If not a numpy array, bail out with empty result
        return IRLockResult(False, 0, 0, 0.0, 0.0, 0.0, 0.0)

    h, w = gray.shape[:2]
    cx_img, cy_img = w // 2, h // 2

    # Find the brightest point in the image
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray)
    
    # Require a reasonably bright peak (at least 180/255)
    if max_val < 180:
        return IRLockResult(False, 0, 0, 0.0, 0.0, 0.0, 0.0)

    # Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Threshold relative to the brightest point (capture bright blob around peak)
    thresh_val = int(max_val * 0.65)
    _, thresh = cv2.threshold(
        blurred,
        thresh_val,
        255,
        cv2.THRESH_BINARY
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
        return IRLockResult(False, 0, 0, 0.0, 0.0, 0.0, 0.0)

    # Select brightest / largest contour
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < 50:  # Reject tiny noise blobs
        return IRLockResult(False, 0, 0, 0.0, 0.0, area, 0.0)

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return IRLockResult(False, 0, 0, 0.0, 0.0, area, 0.0)

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    # Normalised pixel error
    error_x = (cx - cx_img) / cx_img
    error_y = (cy - cy_img) / cy_img

    # Confidence based on blob dominance
    confidence = min(1.0, area / (0.05 * w * h))

    return IRLockResult(
        locked=True,
        cx=cx,
        cy=cy,
        error_x=error_x,
        error_y=error_y,
        area=area,
        confidence=confidence
    )
