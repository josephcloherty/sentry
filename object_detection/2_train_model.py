import os
from ultralytics import YOLO

# 1. Create the Dataset YAML file
yaml_content = f"""
path: {os.path.abspath('./visdrone_yolo')} 
train: images/train
val: images/val   # <--- UPDATED: Points to the actual val folder now
names:
  0: pedestrian
  1: people
  2: bicycle
  3: car
  4: van
  5: truck
  6: tricycle
  7: awning-tricycle
  8: bus
  9: motor
"""

with open("VisDrone_Custom.yaml", "w") as f:
    f.write(yaml_content)

def train():
    # Load a pretrained YOLOv8 nano model
    model = YOLO("yolov8n.pt") 

    # Train the model
    # imgsz=640 is standard. VisDrone has small objects, so 1024 is better 
    # but 640 is much faster for Mac Air.
    results = model.train(
        data="VisDrone_Custom.yaml",
        epochs=50,             # Adjust based on patience
        imgsz=640,             # Use 640 for speed, 1024 for accuracy on small objects
        batch=16,              # Lower if you run out of memory
        device="mps",          # Use Apple Silicon GPU
        patience=10,           # Early stopping
        project="VisDrone_Project",
        name="yolov8n_visdrone"
    )

if __name__ == "__main__":
    train()