import cv2
import numpy as np

def draw_throttle_widget(img, throttle, position=(20, 20), width=100, height=40):
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
    
    # Throttle outline thickness
    outline_thickness = 1
    
    # Colors
    bg_color = (40, 40, 40)  # Dark gray background
    outline_color = (200, 200, 200)  # Light gray outline
    

    fill_color = (255, 255, 255)  # White
    
    text_color = (255, 255, 255)  # White text
    
    # Draw background rectangle
    cv2.rectangle(img, (x, y), (x + height, y + width), bg_color, -1)
    
    # Draw throttle outline
    cv2.rectangle(img, (x, y), (x + height, y + width), outline_color, outline_thickness)
    
    # Calculate fill width based on throttle percentage
    fill_margin = 4  # Margin inside the throttle
    fill_width = int((width - 2 * fill_margin) * (throttle / 100.0))
    
    # Draw throttle fill
    if fill_width > 0:
        #cv2.rectangle(img, 
                      #(x + fill_margin, y + fill_margin), 
                      #(x + fill_margin + fill_width, y + height - fill_margin), 
                      #fill_color, -1)
        cv2.rectangle(img, 
                      (x + height - fill_margin, y + fill_margin + fill_width),
                      (x + fill_margin, y + fill_margin), 
                      fill_color, -1)
    
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