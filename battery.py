import cv2
import numpy as np

def draw_battery_widget(img, battery_remaining, position=(20, 20), width=100, height=40):
    """
    Draw a battery widget on an image
    
    Args:
        img: Input image (numpy array)
        battery_remaining: Battery percentage (0-100)
        position: Top-left corner position (x, y)
        width: Width of the battery icon
        height: Height of the battery icon
    
    Returns:
        Image with battery widget drawn
    """
    x, y = position
    
    # Battery outline thickness
    outline_thickness = 2
    
    # Battery terminal (the small protrusion on the right)
    terminal_width = 5
    terminal_height = height // 3
    
    # Colors
    bg_color = (40, 40, 40)  # Dark gray background
    outline_color = (200, 200, 200)  # Light gray outline
    
    # Determine fill color based on battery level
    if battery_remaining > 60:
        fill_color = (0, 255, 0)  # Green
    elif battery_remaining > 20:
        fill_color = (0, 255, 255)  # Yellow
    else:
        fill_color = (0, 0, 255)  # Red
    
    text_color = (255, 255, 255)  # White text
    
    # Draw background rectangle
    cv2.rectangle(img, (x, y), (x + width, y + height), bg_color, -1)
    
    # Draw battery outline
    cv2.rectangle(img, (x, y), (x + width, y + height), outline_color, outline_thickness)
    
    # Draw battery terminal
    terminal_y = y + (height - terminal_height) // 2
    cv2.rectangle(img, 
                  (x + width, terminal_y), 
                  (x + width + terminal_width, terminal_y + terminal_height), 
                  outline_color, -1)
    
    # Calculate fill width based on battery percentage
    fill_margin = 4  # Margin inside the battery
    fill_width = int((width - 2 * fill_margin) * (battery_remaining / 100.0))
    
    # Draw battery fill
    if fill_width > 0:
        cv2.rectangle(img, 
                      (x + fill_margin, y + fill_margin), 
                      (x + fill_margin + fill_width, y + height - fill_margin), 
                      fill_color, -1)
    
    # Draw percentage text
    text = f"{int(battery_remaining)}%"
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
    mavlink_data = {'battery_remaining': 75}
    
    # Draw battery widget
    frame = draw_battery_widget(frame, mavlink_data['battery_remaining'])
    
    # Display the result
    cv2.imshow('Battery Widget', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Example with different battery levels
    test_levels = [100, 75, 50, 25, 10, 5]
    for i, level in enumerate(test_levels):
        test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        test_frame[:] = (50, 50, 50)
        
        mavlink_data['battery_remaining'] = level
        draw_battery_widget(test_frame, mavlink_data['battery_remaining'])
        
        # Add label
        cv2.putText(test_frame, f"Battery Level: {level}%", (20, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Battery Widget', test_frame)
        cv2.waitKey(1000)  # Show each level for 1 second
    
    cv2.destroyAllWindows()