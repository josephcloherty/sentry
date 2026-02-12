import os
import shutil
from PIL import Image
from tqdm import tqdm

# --- CONFIGURATION ---
# UPDATE THESE PATHS to match your actual folders
SOURCE_DIRS = {
    # Label: Path to that specific dataset folder
    'train': '/Users/josephcloherty/Desktop/AERO420/Dataset-UAV-Human/VisDrone2019-DET-train',
    'val':   '/Users/josephcloherty/Desktop/AERO420/Dataset-UAV-Human/VisDrone2019-DET-val'
    # 'test': '/path/to/VisDrone2019-DET-test-dev' # Optional
}

OUTPUT_DIR = "./object_detection/visdrone_yolo"

# VisDrone Class ID to YOLO Class ID Mapping
CLASS_MAP = {
    1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 
    6: 5, 7: 6, 8: 7, 9: 8, 10: 9
}

def convert_to_yolo(size, box):
    dw = 1. / size[0]
    dh = 1. / size[1]
    x_center = (box[0] + box[2] / 2.0) * dw
    y_center = (box[1] + box[3] / 2.0) * dh
    w = box[2] * dw
    h = box[3] * dh
    return (x_center, y_center, w, h)

def process_split(split_name, source_path):
    print(f"\n--- Processing {split_name} data from {source_path} ---")
    
    # Define source directories
    # Note: Some VisDrone downloads use 'images' and others use 'Images'. Check yours!
    if os.path.exists(os.path.join(source_path, 'images')):
        img_src_dir = os.path.join(source_path, 'images')
    elif os.path.exists(os.path.join(source_path, 'Images')):
         img_src_dir = os.path.join(source_path, 'Images')
    else:
        print(f"Error: Could not find an 'images' folder inside {source_path}")
        return

    anno_src_dir = os.path.join(source_path, 'annotations')
    
    # Define destination directories (YOLO structure)
    img_dst_dir = os.path.join(OUTPUT_DIR, 'images', split_name)
    label_dst_dir = os.path.join(OUTPUT_DIR, 'labels', split_name)
    os.makedirs(img_dst_dir, exist_ok=True)
    os.makedirs(label_dst_dir, exist_ok=True)

    img_files = [f for f in os.listdir(img_src_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    
    for img_file in tqdm(img_files):
        filename_no_ext = os.path.splitext(img_file)[0]
        
        # 1. Copy Image
        src_img_path = os.path.join(img_src_dir, img_file)
        dst_img_path = os.path.join(img_dst_dir, img_file)
        shutil.copy(src_img_path, dst_img_path)
        
        # 2. Get Dimensions
        with Image.open(src_img_path) as img:
            w_img, h_img = img.size
            
        # 3. Process Annotation
        anno_file = os.path.join(anno_src_dir, filename_no_ext + '.txt')
        yolo_anno_file = os.path.join(label_dst_dir, filename_no_ext + '.txt')
        
        if os.path.exists(anno_file):
            with open(anno_file, 'r') as f_in, open(yolo_anno_file, 'w') as f_out:
                for line in f_in:
                    data = line.strip().split(',')
                    if len(data) < 8: continue
                    
                    category = int(data[5])
                    
                    # If category is valid (not 0 or 11)
                    if category in CLASS_MAP:
                        x, y, w, h = float(data[0]), float(data[1]), float(data[2]), float(data[3])
                        
                        # Fix for negative coordinates (sometimes happens in VisDrone)
                        if x < 0: x = 0
                        if y < 0: y = 0
                        
                        bb = convert_to_yolo((w_img, h_img), (x, y, w, h))
                        cls_id = CLASS_MAP[category]
                        
                        # Only write if width/height are positive
                        if bb[2] > 0 and bb[3] > 0:
                            f_out.write(f"{cls_id} {bb[0]:.6f} {bb[1]:.6f} {bb[2]:.6f} {bb[3]:.6f}\n")

if __name__ == "__main__":
    # Clear previous runs if you want a fresh start
    # if os.path.exists(OUTPUT_DIR): shutil.rmtree(OUTPUT_DIR)
    
    for split, path in SOURCE_DIRS.items():
        if os.path.exists(path):
            process_split(split, path)
        else:
            print(f"Skipping {split}: Path {path} does not exist.")
            
    print(f"\nAll Done! Data ready in {os.path.abspath(OUTPUT_DIR)}")