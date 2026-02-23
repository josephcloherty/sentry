#!/usr/bin/env python3
"""Convert Ultralytics-trained .pth checkpoints into Ultralytics-compatible .pt files.

This script attempts to load common PyTorch checkpoint layouts (dicts containing
`model`, `state_dict`, `ema`, or a bare state-dict), normalizes key prefixes
from DataParallel, and writes a minimal checkpoint containing `{'model': state_dict}`
which `ultralytics.YOLO()` can load.
"""
from __future__ import annotations

from pathlib import Path
import argparse
import shutil
import sys

try:
	import torch
except Exception:
	torch = None

try:
	from ultralytics import YOLO
except Exception:
	YOLO = None

# ===== Adjustable settings =====
DEFAULT_MODEL_PATHS = [
	'./object_detection/best_stg1.pth',
]
DEFAULT_OVERWRITE = False
DEFAULT_VALIDATE_WITH_ULTRALYTICS = True


def expand_path(path_value: str) -> Path:
	return Path(path_value).expanduser().resolve()


def ensure_unique_paths(paths: list[Path]) -> list[Path]:
	seen = set()
	result = []
	for path in paths:
		key = str(path)
		if key in seen:
			continue
		seen.add(key)
		result.append(path)
	return result


def extract_state_dict(ckpt: object) -> dict | None:
	"""Return a state_dict-like dict if found in checkpoint, else None."""
	if isinstance(ckpt, dict):
		# Preferred containers
		if 'ema' in ckpt and isinstance(ckpt['ema'], dict):
			# Some training pipelines store EMA under 'ema'
			cand = ckpt['ema']
			if 'state_dict' in cand and isinstance(cand['state_dict'], dict):
				return cand['state_dict']
		for key in ('model', 'model_state_dict', 'state_dict', 'state'):
			if key in ckpt and isinstance(ckpt[key], dict):
				return ckpt[key]

		# If the dict itself appears to be a mapping of parameter tensors
		if ckpt and all(hasattr(v, 'dtype') for v in ckpt.values()):
			return ckpt

	# Could be a nn.Module saved directly (rare) — not handled here
	return None


def normalize_state_dict(state_dict: dict) -> dict:
	"""Remove common prefixes like 'module.' from keys."""
	normalized = {}
	for k, v in state_dict.items():
		nk = k
		if k.startswith('module.'):
			nk = k[len('module.'):]
		normalized[nk] = v
	return normalized


def convert_pth_to_pt(source_path: Path, overwrite: bool) -> Path | None:
	if not source_path.exists():
		print(f'Skipping missing file: {source_path}')
		return None

	if source_path.suffix.lower() != '.pth':
		print(f'Skipping non-.pth file: {source_path}')
		return None

	target_path = source_path.with_suffix('.pt')

	if target_path.exists() and not overwrite:
		print(f'Skipping existing .pt (use --overwrite): {target_path}')
		return target_path

	if torch is None:
		print('PyTorch is required for conversion. Install torch and rerun.')
		return None

	try:
		print(f'Loading checkpoint: {source_path} ({source_path.stat().st_size} bytes)')
		sys.stdout.flush()
		ckpt = torch.load(str(source_path), map_location='cpu')
		print('Loaded checkpoint — analyzing contents...')
		sys.stdout.flush()
	except Exception as exc:
		print(f'Failed to load checkpoint {source_path}: {exc}')
		return None

	state_dict = extract_state_dict(ckpt)
	if state_dict is None and isinstance(ckpt, dict):
		# print top-level keys to help debugging
		print('Top-level checkpoint keys:', list(ckpt.keys())[:20])
		sys.stdout.flush()

	if state_dict is None:
		# As a last resort, copy the file and try validation — sometimes the
		# .pth is already a compatible file with a different suffix.
		try:
			shutil.copy2(source_path, target_path)
			print(f'Could not parse checkpoint; renamed: {source_path} -> {target_path}')
			return target_path
		except Exception as exc:
			print(f'Failed copying {source_path}: {exc}')
			return None

	normalized = normalize_state_dict(state_dict)

	# Preserve original checkpoint layout where possible: place the normalized
	# state dict back into the original checkpoint dict so ultralytics loader
	# sees the same top-level structure it expects.
	if isinstance(ckpt, dict):
		out_ckpt = dict(ckpt)
		out_ckpt['model'] = normalized
	else:
		out_ckpt = {'model': normalized}
	try:
		torch.save(out_ckpt, str(target_path))
		print(f'Converted: {source_path} -> {target_path}')
		return target_path
	except Exception as exc:
		print(f'Failed saving converted checkpoint {target_path}: {exc}')
		return None


def validate_pt_with_ultralytics(pt_path: Path) -> tuple[bool, str | None]:
	if YOLO is None:
		return False, 'ultralytics package not installed'
	try:
		_ = YOLO(str(pt_path))
		return True, None
	except Exception as exc:
		import traceback
		tb = ''.join(traceback.format_exception(type(exc), exc, exc.__traceback__))
		return False, tb


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description='Convert .pth model files to .pt (one-time preparation for runtime).'
	)
	parser.add_argument(
		'--models',
		nargs='+',
		default=DEFAULT_MODEL_PATHS,
		help='One or more source .pth model paths.',
	)
	parser.add_argument(
		'--overwrite',
		action='store_true',
		default=DEFAULT_OVERWRITE,
		help='Overwrite existing .pt outputs.',
	)
	parser.add_argument(
		'--no-validate',
		action='store_true',
		default=not DEFAULT_VALIDATE_WITH_ULTRALYTICS,
		help='Skip Ultralytics compatibility validation after conversion.',
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	model_paths = ensure_unique_paths([expand_path(p) for p in args.models])

	converted = []
	should_validate = not args.no_validate

	for source_path in model_paths:
		out = convert_pth_to_pt(source_path, overwrite=args.overwrite)
		if out is not None:
			if should_validate:
				ok, err = validate_pt_with_ultralytics(out)
				if not ok:
					print(f'Validation failed for {out}: {err}')
					print('This checkpoint is not an Ultralytics-exported model and may not be compatible.')
					continue
			converted.append(out)

	if not converted:
		raise SystemExit('No models were converted.')

	print('\nConversion complete:')
	for path in converted:
		print(f'- {path}')


if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		sys.exit(1)
