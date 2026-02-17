from pathlib import Path
import argparse

from ultralytics import YOLO

try:
    import torch
except Exception:
    torch = None


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATASET_DIR = PROJECT_ROOT / "object_detection" / "visdrone_yolo"
DATA_YAML = PROJECT_ROOT / "VisDrone_PeopleRelated.yaml"

CLASS_NAMES = {
    0: "pedestrian",
    1: "people",
    2: "bicycle",
    3: "tricycle",
    4: "awning-tricycle",
    5: "motor",
}


def detect_device(preferred_device=None):
    if preferred_device:
        return preferred_device
    if torch is not None and torch.cuda.is_available():
        return 0
    return "cpu"


def write_dataset_yaml():
    lines = [
        f"path: {DATASET_DIR.as_posix()}",
        "train: images/train",
        "val: images/val",
        "names:",
    ]
    for class_id, class_name in CLASS_NAMES.items():
        lines.append(f"  {class_id}: {class_name}")

    DATA_YAML.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote dataset yaml: {DATA_YAML}")


def train(args):
    if not DATASET_DIR.exists():
        raise FileNotFoundError(
            f"Dataset directory not found: {DATASET_DIR}. Run 1_preprocess_visdrone.py first."
        )

    write_dataset_yaml()
    device = detect_device(args.device)
    print(f"Using device: {device}")

    model = YOLO(args.weights)
    batch_value = int(args.batch) if float(args.batch).is_integer() else float(args.batch)

    model.train(
        data=str(DATA_YAML),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=batch_value,
        device=device,
        rect=args.rect,
        patience=args.patience,
        workers=args.workers,
        project=args.project,
        name=args.name,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Train people-related YOLO model on VisDrone data.")
    parser.add_argument("--weights", default="yolo26-p2.yaml", help="Model checkpoint or architecture to fine-tune.")
    parser.add_argument("--epochs", type=int, default=50, help="Number of training epochs.")
    parser.add_argument("--imgsz", type=int, default=1024, help="Training image size.")
    parser.add_argument(
        "--batch",
        type=float,
        default=8,
        help="Batch size (int) or AutoBatch fraction 0.0-1.0 (example: 0.95).",
    )
    parser.add_argument("--rect", action="store_true", help="Enable rectangular training.")
    parser.add_argument("--patience", type=int, default=20, help="Early stopping patience.")
    parser.add_argument("--workers", type=int, default=8, help="Dataloader workers.")
    parser.add_argument(
        "--device",
        default=None,
        help="Override device (examples: 0, 1, cpu). Default auto-detects CUDA then CPU.",
    )
    parser.add_argument("--project", default="runs/detect/VisDrone_Project", help="Output project directory.")
    parser.add_argument("--name", default="yolo26n_p2_visdrone_people_related", help="Training run name.")
    return parser.parse_args()


if __name__ == "__main__":
    train(parse_args())