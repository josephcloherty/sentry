from pathlib import Path
import argparse
import shutil

from PIL import Image
from tqdm import tqdm


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATASET_ROOT = PROJECT_ROOT / "Dataset-UAV-Human"
OUTPUT_DIR = PROJECT_ROOT / "object_detection" / "visdrone_yolo"

SOURCE_DIRS = {
    "train": DATASET_ROOT / "VisDrone2019-DET-train",
    "val": DATASET_ROOT / "VisDrone2019-DET-val",
}

CLASS_MAP = {
    1: 0,
    2: 1,
    3: 2,
    7: 3,
    8: 4,
    10: 5,
}


def convert_to_yolo(image_size, box_xywh):
    image_w, image_h = image_size
    x, y, w, h = box_xywh

    x_center = (x + w / 2.0) / image_w
    y_center = (y + h / 2.0) / image_h
    width = w / image_w
    height = h / image_h
    return x_center, y_center, width, height


def clip_box_to_image(image_w, image_h, x, y, w, h):
    x1 = max(0.0, x)
    y1 = max(0.0, y)
    x2 = min(float(image_w), x + w)
    y2 = min(float(image_h), y + h)

    clipped_w = x2 - x1
    clipped_h = y2 - y1
    return x1, y1, clipped_w, clipped_h


def find_image_folder(split_path):
    lower_images = split_path / "images"
    upper_images = split_path / "Images"
    if lower_images.exists():
        return lower_images
    if upper_images.exists():
        return upper_images
    return None


def process_split(split_name, source_path):
    print(f"\n--- Processing {split_name} from {source_path} ---")

    img_src_dir = find_image_folder(source_path)
    if img_src_dir is None:
        print(f"Skipping {split_name}: missing images folder in {source_path}")
        return

    anno_src_dir = source_path / "annotations"
    if not anno_src_dir.exists():
        print(f"Skipping {split_name}: missing annotations folder in {source_path}")
        return

    img_dst_dir = OUTPUT_DIR / "images" / split_name
    label_dst_dir = OUTPUT_DIR / "labels" / split_name
    img_dst_dir.mkdir(parents=True, exist_ok=True)
    label_dst_dir.mkdir(parents=True, exist_ok=True)

    img_files = [
        p
        for p in img_src_dir.iterdir()
        if p.is_file() and p.suffix.lower() in {".jpg", ".jpeg", ".png"} and not p.name.startswith("._")
    ]

    for src_img_path in tqdm(img_files, desc=f"{split_name} images"):
        filename_no_ext = src_img_path.stem
        dst_img_path = img_dst_dir / src_img_path.name
        shutil.copy2(src_img_path, dst_img_path)

        with Image.open(src_img_path) as img:
            image_w, image_h = img.size

        anno_file = anno_src_dir / f"{filename_no_ext}.txt"
        yolo_anno_file = label_dst_dir / f"{filename_no_ext}.txt"

        if not anno_file.exists():
            continue

        with anno_file.open("r", encoding="utf-8") as f_in, yolo_anno_file.open("w", encoding="utf-8") as f_out:
            for line in f_in:
                parts = line.strip().split(",")
                if len(parts) < 8:
                    continue

                category = int(parts[5])
                if category not in CLASS_MAP:
                    continue

                x, y, w, h = map(float, parts[:4])
                x, y, w, h = clip_box_to_image(image_w, image_h, x, y, w, h)
                if w <= 0 or h <= 0:
                    continue

                x_center, y_center, width, height = convert_to_yolo((image_w, image_h), (x, y, w, h))
                f_out.write(
                    f"{CLASS_MAP[category]} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n"
                )


def main():
    parser = argparse.ArgumentParser(description="Convert VisDrone annotations to YOLO format.")
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Delete existing processed output folder before regenerating.",
    )
    args = parser.parse_args()

    if args.clean and OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)

    for split, path in SOURCE_DIRS.items():
        if path.exists():
            process_split(split, path)
        else:
            print(f"Skipping {split}: {path} does not exist")

    print(f"\nDone. YOLO dataset is ready at: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()