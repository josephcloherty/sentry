import cv2
import numpy as np

def draw_throttle_widget(img, throttle, position=(20, 20), width=20, height=100, max_throttle=85):
    """
    Draw a throttle widget on an image
    
    Args:
        img: Input image (numpy array)
        throttle: Throttle percentage (0-100)
        position: Top-left corner position (x, y)
        width: Width of the throttle icon
        height: Height of the throttle icon
    
    Returns:
        Image with throttle widget drawn
    """
    x, y = position

    outline_thickness = 1

    # Colors (BGR)
    bg_color = (40, 40, 40)  # Dark gray background
    outline_color = (200, 200, 200)  # Light gray outline
    normal_fill = (255, 255, 255)  # White for normal throttle (per spec)
    over_fill = (0, 0, 255)  # Red when over max_throttle
    text_color = (255, 255, 255)  # White text

    # Draw background rectangle and outline (width x height)
    cv2.rectangle(img, (x, y), (x + width, y + height), bg_color, -1)
    cv2.rectangle(img, (x, y), (x + width, y + height), outline_color, outline_thickness)

    # Inner area (respect margin)
    fill_margin = 4
    inner_left = x + fill_margin
    inner_right = x + width - fill_margin
    inner_top = y + fill_margin
    inner_bottom = y + height - fill_margin
    inner_height = max(0, inner_bottom - inner_top)

    # Map throttle -> fill height. Visual max corresponds to `max_throttle`.
    throttle_clamped = max(0.0, min(100.0, float(throttle)))
    fill_ratio = min(throttle_clamped, float(max_throttle)) / float(max_throttle) if max_throttle > 0 else 0
    fill_height = int(inner_height * fill_ratio)

    # Choose fill color: red if above max_throttle, otherwise normal
    fill_color = over_fill if throttle_clamped > float(max_throttle) else normal_fill

    # Draw fill from bottom up
    if fill_height > 0:
        fill_top = inner_bottom - fill_height
        cv2.rectangle(img, (inner_left, fill_top), (inner_right, inner_bottom), fill_color, -1)

    # If over limit, draw a thin red band at the top to indicate overrange
    if throttle_clamped > float(max_throttle):
        over_ratio = (throttle_clamped - float(max_throttle)) / (100.0 - float(max_throttle)) if throttle_clamped < 100.0 else 1.0
        over_height = int(inner_height * min(1.0, over_ratio))
        if over_height > 0:
            over_top = inner_top
            over_bottom = inner_top + over_height
            cv2.rectangle(img, (inner_left, over_top), (inner_right, over_bottom), over_fill, -1)
    
    # Draw percentage text
    text = f"{int(throttle)}%"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_thickness = 1
    
    # Get text size to center it
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_x = x + (width - text_width) // 2
    text_y = y + (height + text_height) // 2
    
    # Draw text with black outline for better visibility
    cv2.putText(img, text, (text_x, text_y), font, font_scale, (0, 0, 0), font_thickness + 1)
    cv2.putText(img, text, (text_x, text_y), font, font_scale, text_color, font_thickness)
    
    return img


# Example usage
if __name__ == "__main__":
    # Create a blank image (simulating video frame)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (50, 50, 50)  # Gray background
    
    # Simulate MAVLink data
    mavlink_data = {'throttle': 75}
    
    # Draw throttle widget
    frame = draw_throttle_widget(frame, mavlink_data.get('throttle', 0))
    
    # Display the result
    cv2.imshow('Throttle Widget', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Example with different throttle levels
    test_levels = [100, 75, 50, 25, 10, 5]
    for i, level in enumerate(test_levels):
        test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        test_frame[:] = (50, 50, 50)
        
        mavlink_data['throttle'] = level
        draw_throttle_widget(test_frame, mavlink_data['throttle'])
        
        # Add label
        cv2.putText(test_frame, f"Throttle Level: {level}%", (20, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Throttle Widget', test_frame)
        cv2.waitKey(1000)  # Show each level for 1 second
    
    cv2.destroyAllWindows()