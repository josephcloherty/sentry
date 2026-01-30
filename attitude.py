import cv2
import numpy as np
import math

_attitude_cache = {}

def _create_attitude_background(size):
    """Prerender the static attitude indicator background"""
    bg = np.zeros((size, size, 4), dtype=np.uint8)
    c = (size//2, size//2)
    
    # Create circular mask
    y, x = np.ogrid[:size, :size]
    mask = (x - c[0])**2 + (y - c[1])**2 <= (size//2)**2
    
    # Sky (top half) - blue
    bg[:c[1], :, 0] = 255  # B
    bg[:c[1], :, 1] = 150  # G
    bg[:c[1], :, 2] = 0  # R
    bg[:c[1], :, 3] = 255  # A
    
    # Ground (bottom half) - brown
    bg[c[1]:, :, 0] = 50   # B
    bg[c[1]:, :, 1] = 100  # G
    bg[c[1]:, :, 2] = 139  # R
    bg[c[1]:, :, 3] = 255  # A
    
    # Apply circular mask
    bg[~mask] = 0
    
    # Draw horizon line (will be rotated in main function)
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
    wing_width = size // 4
    wing_height = size // 20
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
    
    # Create circular mask
    y, x = np.ogrid[:size, :size]
    mask = (x - c[0])**2 + (y - c[1])**2 <= (r - 1)**2
    
    # Apply mask to alpha channel
    image[:, :, 3] = image[:, :, 3] * mask.astype(np.uint8)
    
    return image

def draw_attitude_indicator(img, roll, pitch, x, y, size):
    """
    Draw aircraft attitude indicator
    roll: roll angle in degrees (positive = right wing down)
    pitch: pitch angle in degrees (positive = nose up)
    """
    # Get or create cached backgrounds
    cache_key = size
    if cache_key not in _attitude_cache:
        _attitude_cache[cache_key] = {
            'background': _create_attitude_background(size),
            'overlay': _create_static_overlay(size)
        }
    
    c = (size//2, size//2)
    
    # Get background and shift it for pitch
    bg = _attitude_cache[cache_key]['background'].copy()
    
    # Apply pitch offset (shift background vertically)
    pitch_pixels = int(pitch * size / 120)  # Scale factor: size span = 120 degrees (±60)
    if pitch_pixels != 0:
        M = np.float32([[1, 0, 0], [0, 1, pitch_pixels]])
        bg = cv2.warpAffine(bg, M, (size, size), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
    
    # Apply circular mask after pitch shift to maintain color fill
    bg = _apply_circular_mask(bg, size)
    
    # Draw pitch ladder lines on the background
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
            
            # Add pitch value text
            text = f"{abs(pitch_angle)}"
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)[0]
            cv2.putText(bg, text, (c[0] - line_length - text_size[0] - 5, y_pos + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1, lineType=cv2.LINE_AA)
            cv2.putText(bg, text, (c[0] + line_length + 5, y_pos + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1, lineType=cv2.LINE_AA)
    
    # Rotate background by roll angle
    bg_rotated = _rotate_image(bg, roll, c)
    
    # Apply circular mask to rotated background to prevent spillover
    bg_rotated = _apply_circular_mask(bg_rotated, size)
    
    # Get static overlay
    overlay = _attitude_cache[cache_key]['overlay'].copy()
    
    # Combine rotated background with static overlay
    combined = bg_rotated.copy()
    
    # Blend overlay on top
    overlay_rgb = overlay[:, :, :3]
    overlay_alpha = overlay[:, :, 3:4] / 255.0
    combined[:, :, :3] = (overlay_alpha * overlay_rgb + (1 - overlay_alpha) * combined[:, :, :3]).astype(np.uint8)
    combined[:, :, 3] = np.maximum(combined[:, :, 3], overlay[:, :, 3])
    
    # Blend onto main image
    alpha = combined[:, :, 3:4] / 255.0
    roi = img[y:y+size, x:x+size]
    img[y:y+size, x:x+size] = (alpha * combined[:, :, :3] + (1 - alpha) * roi).astype(np.uint8)
    
    return img