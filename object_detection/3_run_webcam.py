import cv2
from ultralytics import YOLO

# Load your custom trained model
# REPLACE this path with the actual path generated after training
model_path = "VisDrone_Project/yolov8n_visdrone/weights/best.pt"

# Fallback to standard yolov8n if you haven't trained yet, just to test the camera
if not os.path.exists(model_path):
    print("Custom model not found, using default YOLOv8n for testing...")
    model_path = "yolov8n.pt"

model = YOLO(model_path)

# Initialize Webcam (0 is usually the default Mac webcam)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Starting inference... Press 'q' to quit.")

while True:
    success, frame = cap.read()
    if not success:
        break

    # Run inference
    # conf=0.5: Only show detections with >50% confidence
    # imgsz=640: Inference size (keep consistent with training)
    results = model(frame, stream=True, conf=0.5, imgsz=640, device="mps")

    # Visualize results
    for result in results:
        annotated_frame = result.plot()
        
        # Display the frame
        cv2.imshow("VisDrone Real-Time Detection", annotated_frame)

    # Quit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()