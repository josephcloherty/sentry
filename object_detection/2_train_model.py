from pathlib import Path
import argparse
import platform

from ultralytics import YOLO, RTDETR


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATASET_DIR = PROJECT_ROOT / "object_detection" / "visdrone_yolo"
DEFAULT_DATA_YAML = PROJECT_ROOT / "VisDrone_PeopleRelated_YOLO11nP2.yaml"
DEFAULT_WEIGHTS = "yolov8-ghost-p2.yaml"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "runs" / "detect"
IS_WINDOWS = platform.system().lower() == "windows"
DEFAULT_WORKERS = 0 if IS_WINDOWS else 8


def ensure_paths(args):
    if not DATASET_DIR.exists():
        raise FileNotFoundError(
            f"Dataset directory not found: {DATASET_DIR}. Run 1_preprocess_visdrone.py first."
        )

    train_img = DATASET_DIR / "images" / "train"
    val_img = DATASET_DIR / "images" / "val"
    train_lbl = DATASET_DIR / "labels" / "train"
    val_lbl = DATASET_DIR / "labels" / "val"

    required_paths = [train_img, val_img, train_lbl, val_lbl]
    missing = [str(path_obj) for path_obj in required_paths if not path_obj.exists()]
    if missing:
        raise FileNotFoundError(
            "Missing YOLO dataset files/folders:\n- " + "\n- ".join(missing)
        )

    data_yaml = Path(args.data).resolve()
    if not data_yaml.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {data_yaml}")

    weights_candidate = Path(args.weights)
    if weights_candidate.exists():
        model_source = str(weights_candidate.resolve())
    else:
        model_source = args.weights

    return {
        "data_yaml": data_yaml,
        "model_source": model_source,
    }


def train(args):
    paths = ensure_paths(args)
    workers = args.workers
    if IS_WINDOWS and not args.force_workers and workers > 0:
        print(
            f"Windows worker override: requested workers={workers}, using workers=0 to avoid WinError 1455 torch DLL load failures. "
            "Use --force-workers to bypass."
        )
        workers = 0

    model_source_lower = paths["model_source"].lower()
    model_cls = RTDETR if "rtdetr" in model_source_lower or "rt-detr" in model_source_lower else YOLO
    model = model_cls(paths["model_source"])
    model.train(
        data=str(paths["data_yaml"]),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=workers,
        device=args.device,
        project=str(Path(args.output_dir).resolve()),
        name=args.run_name,
        pretrained=args.pretrained,
        amp=args.amp,
        cache=args.cache,
        exist_ok=args.exist_ok,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Train an Ultralytics YOLO model on VisDrone YOLO-format dataset.")
    parser.add_argument(
        "--data",
        default=str(DEFAULT_DATA_YAML),
        help="Path to YOLO dataset YAML file.",
    )
    parser.add_argument(
        "--weights",
        default=str(DEFAULT_WEIGHTS),
        help="Model source (local .pt/.yaml path or Ultralytics model name), e.g. yolov8-ghost-p2.yaml.",
    )
    parser.add_argument("--epochs", type=int, default=100, help="Number of training epochs.")
    parser.add_argument("--imgsz", type=int, default=640, help="Training image size.")
    parser.add_argument("--batch", type=int, default=16, help="Batch size.")
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS, help="Dataloader workers.")
    parser.add_argument("--device", default="0", help="Device string, e.g. 0, 0,1, or cpu.")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="Output project directory.")
    parser.add_argument("--run-name", default="yolov8_ghost_p2_visdrone", help="Run name under output directory.")
    parser.add_argument(
        "--force-workers",
        action="store_true",
        help="On Windows, do not auto-override workers to 0 (may trigger WinError 1455 on low pagefile systems).",
    )
    parser.add_argument("--pretrained", action="store_true", default=True, help="Use pretrained weights.")
    parser.add_argument(
        "--no-pretrained",
        action="store_false",
        dest="pretrained",
        help="Disable pretrained initialization.",
    )
    parser.add_argument("--amp", action="store_true", default=True, help="Enable AMP training.")
    parser.add_argument("--no-amp", action="store_false", dest="amp", help="Disable AMP training.")
    parser.add_argument("--cache", action="store_true", help="Cache images for faster training.")
    parser.add_argument("--exist-ok", action="store_true", help="Allow reuse of existing run directory.")
    return parser.parse_args()


if __name__ == "__main__":
    train(parse_args())