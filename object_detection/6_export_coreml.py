from pathlib import Path
import argparse
import shutil

from ultralytics import YOLO


# ===== Adjustable export settings =====
DEFAULT_MODEL_PATHS = [
    "./object_detection/best8np2.pt",
    "./object_detection/best26p2.pt",
]
DEFAULT_OUTPUT_DIR = "./object_detection/coreml"
DEFAULT_IMAGE_SIZE = 800
DEFAULT_HALF_PRECISION = True
DEFAULT_ENABLE_NMS = True
DEFAULT_ENABLE_INT8 = False
DEFAULT_OVERWRITE = False


def expand_path(path_value: str) -> Path:
    return Path(path_value).expanduser().resolve()


def ensure_unique_paths(paths: list[Path]) -> list[Path]:
    ordered = []
    seen = set()
    for path in paths:
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        ordered.append(path)
    return ordered


def move_export_if_needed(exported_path: Path, destination_dir: Path, overwrite: bool) -> Path:
    destination_dir.mkdir(parents=True, exist_ok=True)
    destination_path = destination_dir / exported_path.name

    if destination_path.exists():
        if not overwrite:
            print(f"Skipping move (already exists): {destination_path}")
            return destination_path
        if destination_path.is_dir():
            shutil.rmtree(destination_path)
        else:
            destination_path.unlink()

    shutil.move(str(exported_path), str(destination_path))
    return destination_path


def export_model_to_coreml(
    model_path: Path,
    output_dir: Path,
    imgsz: int,
    half: bool,
    nms: bool,
    int8: bool,
    overwrite: bool,
) -> Path | None:
    if not model_path.exists():
        print(f"Skipping missing model: {model_path}")
        return None

    if model_path.suffix.lower() != ".pt":
        print(f"Skipping non-.pt model: {model_path}")
        return None

    print(f"Exporting to Core ML: {model_path}")
    model = YOLO(str(model_path))
    exported = model.export(
        format="coreml",
        imgsz=imgsz,
        half=half,
        nms=nms,
        int8=int8,
    )

    exported_path = expand_path(str(exported))
    if exported_path.parent != output_dir:
        exported_path = move_export_if_needed(exported_path, output_dir, overwrite)

    print(f"Saved Core ML model: {exported_path}")
    return exported_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export YOLO .pt models to Core ML (.mlpackage) for Apple Silicon runtime."
    )
    parser.add_argument(
        "--models",
        nargs="+",
        default=DEFAULT_MODEL_PATHS,
        help="One or more source YOLO .pt model paths.",
    )
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Directory where exported .mlpackage files will be stored.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=DEFAULT_IMAGE_SIZE,
        help="Export image size.",
    )
    parser.add_argument(
        "--half",
        dest="half",
        action="store_true",
        default=DEFAULT_HALF_PRECISION,
        help="Export FP16 Core ML model (recommended on M1/M2).",
    )
    parser.add_argument(
        "--full-precision",
        dest="half",
        action="store_false",
        help="Export FP32 model instead of FP16.",
    )
    parser.add_argument(
        "--nms",
        dest="nms",
        action="store_true",
        default=DEFAULT_ENABLE_NMS,
        help="Include NMS in exported model.",
    )
    parser.add_argument(
        "--no-nms",
        dest="nms",
        action="store_false",
        help="Disable NMS in exported model.",
    )
    parser.add_argument(
        "--int8",
        action="store_true",
        default=DEFAULT_ENABLE_INT8,
        help="Enable INT8 quantization during export.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        default=DEFAULT_OVERWRITE,
        help="Overwrite existing exported .mlpackage outputs in --out-dir.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = expand_path(args.out_dir)

    model_paths = ensure_unique_paths([expand_path(path) for path in args.models])

    exported_paths = []
    for model_path in model_paths:
        exported_path = export_model_to_coreml(
            model_path=model_path,
            output_dir=output_dir,
            imgsz=args.imgsz,
            half=args.half,
            nms=args.nms,
            int8=args.int8,
            overwrite=args.overwrite,
        )
        if exported_path is not None:
            exported_paths.append(exported_path)

    if not exported_paths:
        raise SystemExit("No models were exported.")

    print("\nExport complete. Core ML outputs:")
    for path in exported_paths:
        print(f"- {path}")


if __name__ == "__main__":
    main()
