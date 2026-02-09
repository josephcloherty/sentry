import cv2
import numpy as np

def detect_apriltag_opencv(frame: np.ndarray, min_area=800, max_deviation=0.15, require_nesting=False) -> dict:
    """OpenCV fallback to detect square fiducials and prefer nested quads.

    This is conservative: rejects non-square quads and prefers nested patterns.
    Parameters:
      - min_area: minimum pixel area to consider (default 800; higher = fewer false positives)
      - max_deviation: max ratio of aspect ratio from 1.0 to accept as square (default 0.15)
      - require_nesting: if True, only accept quads with nested patterns inside (strict mode)

    Returns dict: {locked, corners, area, confidence, nested_count}
    """
    h, w = frame.shape[:2]
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Use RETR_TREE to capture nested contours (for detecting nested AprilTags)
    contours, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    quads = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            pts = approx.reshape((4, 2)).astype(np.int32)
            M = cv2.moments(pts)
            if M.get('m00', 0) == 0:
                continue
            
            # Reject if contour is too close to frame edge (likely frame/background)
            margin = 10
            if np.any(pts < margin) or np.any(pts[:, 0] > w - margin) or np.any(pts[:, 1] > h - margin):
                continue
            
            # Enforce square-ness: compute side lengths
            sides = []
            for i in range(4):
                p1 = pts[i]
                p2 = pts[(i + 1) % 4]
                side = np.linalg.norm(p1 - p2)
                sides.append(side)
            sides = np.array(sides)
            aspect = max(sides) / (min(sides) + 1e-6)
            
            # Only accept if reasonably square (aspect ratio close to 1.0)
            if aspect > 1.0 + max_deviation:
                continue  # Skip non-square shapes (rectangles, circles, etc.)
            
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            quads.append({'area': float(area), 'pts': pts, 'centroid': (cx, cy), 'aspect': float(aspect)})

    if not quads:
        return {'locked': False, 'corners': None, 'area': 0.0, 'confidence': 0.0, 'nested_count': 0}

    # Sort by descending area so larger quads come first
    quads.sort(key=lambda q: q['area'], reverse=True)

    # For each quad, count how many smaller quads are inside it
    for i, outer in enumerate(quads):
        nested = 0
        poly = outer['pts']
        for j, inner in enumerate(quads):
            if j == i:
                continue
            # test inner centroid inside outer polygon
            val = cv2.pointPolygonTest(poly, tuple(inner['centroid']), False)
            if val >= 0:
                nested += 1
        outer['nested'] = nested

    # Optional: filter to only accept quads with nesting (strict mode for real AprilTags)
    if require_nesting:
        quads = [q for q in quads if q.get('nested', 0) > 0]
        if not quads:
            return {'locked': False, 'corners': None, 'area': 0.0, 'confidence': 0.0, 'nested_count': 0}

    # Prefer: (1) largest nesting count, (2) best squareness, (3) largest area
    best = max(quads, key=lambda q: (q.get('nested', 0), -abs(q.get('aspect', 1.0) - 1.0), q['area']))
    area = best['area']
    corners = best['pts']
    nested_count = int(best.get('nested', 0))
    aspect = best.get('aspect', 1.0)

    # Order corners: top-left, top-right, bottom-right, bottom-left
    s = corners.sum(axis=1)
    diff = np.diff(corners, axis=1).reshape(-1)
    tl = corners[np.argmin(s)]
    tr = corners[np.argmin(diff)]
    br = corners[np.argmax(s)]
    bl = corners[np.argmax(diff)]
    ordered = np.array([tl, tr, br, bl], dtype=np.int32)

    # Confidence calculation:
    # - Start at 0.4 (OpenCV fallback is less reliable than real apriltag)
    # - Bonus for nested quads: +0.15 per nested (up to +0.45)
    # - Bonus for squareness: +0.15 if aspect < 1.05
    # - Bonus for area size
    confidence = 0.4
    confidence += min(nested_count * 0.15, 0.45)  # nested bonus (max +0.45)
    if aspect < 1.05:
        confidence += 0.15  # squareness bonus
    # Scale confidence slightly based on area (larger = slightly more confident)
    area_factor = min(area / (0.02 * w * h), 0.1)  # max +0.1
    confidence += area_factor
    confidence = float(min(1.0, confidence))

    return {
        'locked': True,
        'corners': ordered,
        'area': area,
        'confidence': confidence,
        'nested_count': nested_count
    }
