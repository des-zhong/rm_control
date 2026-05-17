#!/usr/bin/env python3
"""Optional fallback: train a YOLO detector for the ball class.

This is not used by the HSV/contour grasp-success solution. Use it only when
lighting/background makes classical segmentation unreliable.

Example:
  python3 train_yolo_ball.py \
    --data /path/to/soccervision.v3i.yolov8/data.yaml \
    --model yolov8n.pt --epochs 80 --imgsz 640 --class-id 0
"""

import argparse
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', required=True, help='YOLO data.yaml path')
    parser.add_argument('--model', default='yolov8n.pt', help='Base YOLO model')
    parser.add_argument('--epochs', type=int, default=80)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--class-id', type=int, default=0, help='Ball class id in data.yaml')
    parser.add_argument('--project', default='runs/rm_grip_vision')
    parser.add_argument('--name', default='ball_yolo')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data_path = Path(args.data).expanduser().resolve()
    if not data_path.exists():
        raise FileNotFoundError(f'data.yaml not found: {data_path}')

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            'ultralytics is not installed. Install it first with: python3 -m pip install ultralytics'
        ) from exc

    model = YOLO(args.model)
    model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        classes=[args.class_id],
        project=args.project,
        name=args.name,
    )
    print('Training finished. Best weights are normally under:')
    print(Path(args.project) / args.name / 'weights' / 'best.pt')


if __name__ == '__main__':
    main()
