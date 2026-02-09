"""Test script to run fiducial detection on a webcam or static image.

Usage:
  python -m sentry.test_fiducial --webcam
  python -m sentry.test_fiducial --image path/to/file.png

Draws a red polygon around the detected tag and prints confidence/area.
"""
import cv2
import argparse
import time
from fiducial_tracker import process_fiducial_frame


def run_webcam(device=0):
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print("Cannot open webcam")
        return
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            res = process_fiducial_frame(frame)
            h, w = frame.shape[:2]
            if getattr(res, 'locked', False):
                if hasattr(res, 'corners') and res.corners is not None:
                    pts = res.corners.reshape((-1, 2)).astype(int)
                    cv2.polylines(frame, [pts], True, (0, 0, 255), 2)
                else:
                    cx = int(w * (0.5 + 0.5 * res.error_x))
                    cy = int(h * (0.5 + 0.5 * res.error_y))
                    size = max(10, int((res.area ** 0.5) / 2))
                    cv2.rectangle(frame, (cx - size, cy - size), (cx + size, cy + size), (0, 0, 255), 2)
                cv2.putText(frame, f"C:{res.confidence:.2f} A:{int(res.area)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            cv2.imshow('fiducial test', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


def run_image(path):
    frame = cv2.imread(path)
    if frame is None:
        print('Failed to load image:', path)
        return
    res = process_fiducial_frame(frame)
    if getattr(res, 'locked', False):
        if hasattr(res, 'corners') and res.corners is not None:
            pts = res.corners.reshape((-1, 2)).astype(int)
            cv2.polylines(frame, [pts], True, (0, 0, 255), 2)
        else:
            h, w = frame.shape[:2]
            cx = int(w * (0.5 + 0.5 * res.error_x))
            cy = int(h * (0.5 + 0.5 * res.error_y))
            size = max(10, int((res.area ** 0.5) / 2))
            cv2.rectangle(frame, (cx - size, cy - size), (cx + size, cy + size), (0, 0, 255), 2)
        fid_id = getattr(res, 'fiducial_id', None)
        print(f"Locked: C={res.confidence:.2f} Area={int(res.area)} ID={fid_id}")
    else:
        print('No fiducial detected')
    cv2.imshow('fiducial image test', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--webcam', action='store_true', help='Use webcam (default if no image)')
    parser.add_argument('--device', type=int, default=0, help='Webcam device index')
    parser.add_argument('--image', type=str, help='Path to test image')
    args = parser.parse_args()

    if args.image:
        run_image(args.image)
    else:
        run_webcam(args.device)


if __name__ == '__main__':
    main()
