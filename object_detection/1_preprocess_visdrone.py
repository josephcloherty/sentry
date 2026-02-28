from pathlib import Path
import argparse
import shutil

from PIL import Image
from tqdm import tqdm
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATASET_ROOT = PROJECT_ROOT / "Dataset-UAV-Human"
OUTPUT_DIR = PROJECT_ROOT / "object_detection" / "visdrone_yolo"
DATASET_YAML_PATH = PROJECT_ROOT / "VisDrone_PeopleRelated_YOLO11nP2.yaml"

SOURCE_DIRS = {
    "train": DATASET_ROOT / "VisDrone2019-DET-train",
    "val": DATASET_ROOT / "VisDrone2019-DET-val",
    "test": DATASET_ROOT / "VisDrone2019-DET-test-dev",
}

CLASS_MAP = {
    1: 0,
    2: 1,
    3: 2,
    7: 3,
    8: 4,
    10: 5,
}

CLASS_NAMES = [
    "pedestrian",
    "people",
    "bicycle",
    "tricycle",
    "awning-tricycle",
    "motor",
]


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


def xywh_to_yolo(image_w, image_h, x, y, w, h):
    x_center = (x + (w / 2.0)) / float(image_w)
    y_center = (y + (h / 2.0)) / float(image_h)
    w_norm = w / float(image_w)
    h_norm = h / float(image_h)
    return x_center, y_center, w_norm, h_norm


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
    lbl_dst_dir = OUTPUT_DIR / "labels" / split_name
    img_dst_dir.mkdir(parents=True, exist_ok=True)
    lbl_dst_dir.mkdir(parents=True, exist_ok=True)

    img_files = [
        p
        for p in img_src_dir.iterdir()
        if p.is_file() and p.suffix.lower() in {".jpg", ".jpeg", ".png"} and not p.name.startswith("._")
    ]

    annotation_count = 0

    for src_img_path in tqdm(img_files, desc=f"{split_name} images"):
        filename_no_ext = src_img_path.stem
        dst_img_path = img_dst_dir / src_img_path.name
        shutil.copy2(src_img_path, dst_img_path)

        with Image.open(src_img_path) as img:
            image_w, image_h = img.size

        label_lines = []

        anno_file = anno_src_dir / f"{filename_no_ext}.txt"
        if anno_file.exists():
            with anno_file.open("r", encoding="utf-8") as f_in:
                for line in f_in:
                    parts = line.strip().split(",")
                    if len(parts) < 8:
                        continue

                    visdrone_category = int(parts[5])
                    if visdrone_category not in CLASS_MAP:
                        continue

                    x, y, w, h = map(float, parts[:4])
                    x, y, w, h = clip_box_to_image(image_w, image_h, x, y, w, h)
                    if w <= 0 or h <= 0:
                        continue

                    cls_id = CLASS_MAP[visdrone_category]
                    x_center, y_center, w_norm, h_norm = xywh_to_yolo(image_w, image_h, x, y, w, h)
                    label_lines.append(
                        f"{cls_id} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}"
                    )
                    annotation_count += 1

        output_label = lbl_dst_dir / f"{filename_no_ext}.txt"
        output_label.write_text("\n".join(label_lines), encoding="utf-8")

    print(f"Processed {len(img_files)} images and {annotation_count} labels for split '{split_name}'")


def write_dataset_yaml():
    payload = {
        "path": str(OUTPUT_DIR.resolve()).replace("\\", "/"),
        "train": "images/train",
        "val": "images/val",
        "names": {index: class_name for index, class_name in enumerate(CLASS_NAMES)},
    }
    DATASET_YAML_PATH.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    print(f"Wrote dataset YAML: {DATASET_YAML_PATH}")


def main():
    parser = argparse.ArgumentParser(description="Convert VisDrone annotations to YOLO format for YOLO11n training.")
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Delete existing processed output folder before regenerating.",
    )
    parser.add_argument(
        "--skip-test",
        action="store_true",
        help="Skip processing VisDrone test-dev split.",
    )
    args = parser.parse_args()

    if args.clean and OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)

    for split, path in SOURCE_DIRS.items():
        if split == "test" and args.skip_test:
            print("Skipping test: --skip-test provided")
            continue

        if path.exists():
            process_split(split, path)
        else:
            print(f"Skipping {split}: {path} does not exist")

    write_dataset_yaml()
    print(f"\nDone. YOLO dataset is ready at: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()