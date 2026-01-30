import cv2
import numpy as np
import math

_compass_cache = {}

def _create_compass_background(size):
    """Prerender the static compass background with transparency"""
    bg = np.zeros((size, size, 4), dtype=np.uint8)  # 4 channels for RGBA
    c = (size//2, size//2)
    r = size // 3
    
    # Circle
    cv2.circle(bg, c, r, (255, 255, 255, 255), 1, lineType=cv2.LINE_AA)
    cv2.circle(bg, c, r//20, (255, 255, 255, 255), 1, lineType=cv2.LINE_AA)
    
    # Cardinal directions
    for angle, label in [(0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')]:
        rad = math.radians(angle)
        tx = int(c[0] + r * 0.8 * math.sin(rad))
        ty = int(c[1] - r * 0.8 * math.cos(rad))
        text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, 0.3, 1)[0]
        cv2.putText(bg, label, (tx - text_size[0]//2, ty + text_size[1]//2), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.3, (255, 255, 255, 255), 1, lineType=cv2.LINE_AA)
    
    return bg

def draw_compass(img, yaw, x, y, size):
    """Draw compass with transparent background overlay"""
    # Get or create cached background
    if size not in _compass_cache:
        _compass_cache[size] = _create_compass_background(size)
    
    # Get prerendered background
    bg = _compass_cache[size].copy()
    
    # Draw dynamic elements on background
    c_bg = (size//2, size//2)
    r = size // 3
    
    # Needle
    yaw_rad = math.radians(yaw)
    nx = int(c_bg[0] + r * 0.8 * math.sin(yaw_rad))
    ny = int(c_bg[1] - r * 0.8 * math.cos(yaw_rad))
    cv2.arrowedLine(bg, c_bg, (nx, ny), (0, 0, 200, 255), 2, tipLength=0.2, line_type=cv2.LINE_AA)
    
    # Center dot
    cv2.circle(bg, c_bg, r//20, (255, 255, 255, 255), -1, lineType=cv2.LINE_AA)
    
    # Yaw text
    text = f"{yaw:.1f} deg"
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
    text_x = c_bg[0] - text_size[0] // 2
    cv2.putText(bg, text, (text_x, size-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 
                (255, 255, 255, 255), 1, lineType=cv2.LINE_AA)
    
    # Extract RGB and alpha channels
    overlay_rgb = bg[:, :, :3]
    alpha = bg[:, :, 3:4] / 255.0  # Keep as 3D array for broadcasting
    
    # Blend onto main image using vectorized operations
    roi = img[y:y+size, x:x+size]
    img[y:y+size, x:x+size] = (alpha * overlay_rgb + (1 - alpha) * roi).astype(np.uint8)
    
    return img