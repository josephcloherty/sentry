import cv2
import sys

# CONFIGURATION
# The gopro tool usually creates /dev/video42. 
# If that fails, try 0 (standard webcam) or 1.
DEVICE_INDEX = 42
WINDOW_NAME = "GoPro Feed (Press 'q' to quit)"

def main():
    print(f"Attempting to open /dev/video{DEVICE_INDEX}...")
    
    # 1. Open the connection
    cap = cv2.VideoCapture(DEVICE_INDEX)
    
    # 2. Check if connection was successful
    if not cap.isOpened():
        print(f"Error: Could not open video device {DEVICE_INDEX}.")
        print("Tip: If using the gopro script, it defaults to device 42.")
        print("Try changing DEVICE_INDEX to 0 or 1 in this script.")
        return

    # 3. Low Latency Setting (Crucial for Pi!)
    # This limits the internal buffer so it doesn't store old frames.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print("Stream started. Press 'q' to exit.")

    while True:
        # 4. Read a frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting ...")
            break

        # 5. Display the frame
        cv2.imshow(WINDOW_NAME, frame)

        # 6. Exit logic (Press 'q')
        if cv2.waitKey(1) == ord('q'):
            break

    # 7. Cleanup
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()