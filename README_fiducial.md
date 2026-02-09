Fiducial detection and test runner
================================

Files:
- `fiducial_tracker.py` - unified API to detect AprilTag (if installed), QR, or OpenCV quad fallback.
- `fiducial_opencv.py` - OpenCV heuristic that finds quadrilateral contours and prefers nested quads.
- `test_fiducial.py` - small test runner that uses your webcam or a static image and draws a red box/polygon.
- `requirements-fiducial.txt` - minimal Python packages required.

Quick start (macOS or Linux):

1) Create a venv and install dependencies:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r sentry/requirements-fiducial.txt
```

2) Test with your webcam:

```bash
python3 -m sentry.test_fiducial --webcam
```

3) Test with a generated nested tag (SVG -> PNG):

If you have `cairosvg` installed you can convert the SVG produced under `sentry/april_tag_generator/out`:

```bash
cairosvg sentry/april_tag_generator/out/your_file.svg -o /tmp/preview.png
python3 -m sentry.test_fiducial --image /tmp/preview.png
```

Notes:
- The OpenCV fallback is a heuristic and works best with high-contrast, fronto-parallel tags.
- For robust detection on the Pi, install `apriltag` (faster and more accurate). The tracker will prefer `apriltag` when present.
- `server.py` already integrates `process_fiducial_frame` and will draw a red polygon/box for detected fiducials on the cam1 stream.
