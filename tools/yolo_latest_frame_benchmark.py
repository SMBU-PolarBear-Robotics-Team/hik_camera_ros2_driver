#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


def read_meta(path):
    meta = {}
    try:
        for line in path.read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                meta[key] = value
    except FileNotFoundError:
        pass
    return meta


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="/home/f/hk_ws/best.pt")
    parser.add_argument("--input-dir", default="/tmp/hk_yolo_live")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--show", action="store_true")
    parser.add_argument(
        "--save-annotated", default="/home/f/hk_ws/yolo_live/latest_annotated.jpg"
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    image_path = input_dir / "latest.jpg"
    meta_path = input_dir / "latest.txt"
    annotated_path = Path(args.save_annotated)
    annotated_path.parent.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.model)

    print(f"model={args.model}")
    print(f"watching={image_path}")
    print(f"annotated={annotated_path}")
    print("columns: frame fps inference_ms latency_ms detections")

    last_mtime_ns = 0
    frame_count = 0
    start_time = time.monotonic()
    last_report = start_time

    while True:
        if args.duration > 0 and time.monotonic() - start_time >= args.duration:
            break
        if not image_path.exists():
            time.sleep(0.02)
            continue

        stat = image_path.stat()
        if stat.st_mtime_ns == last_mtime_ns:
            time.sleep(0.005)
            continue
        last_mtime_ns = stat.st_mtime_ns

        image = cv2.imread(str(image_path))
        if image is None:
            continue

        meta = read_meta(meta_path)
        t0 = time.perf_counter()
        results = model.predict(
            source=image,
            imgsz=args.imgsz,
            conf=args.conf,
            verbose=False,
        )
        infer_ms = (time.perf_counter() - t0) * 1000.0
        done_ns = time.time_ns()

        stamp_ns = int(meta.get("stamp_ns", "0"))
        latency_ms = (done_ns - stamp_ns) / 1e6 if stamp_ns > 0 else -1.0
        detections = len(results[0].boxes) if results else 0
        annotated = results[0].plot()
        cv2.imwrite(str(annotated_path), annotated)

        frame_count += 1
        now = time.monotonic()
        fps = frame_count / max(now - start_time, 1e-6)
        if now - last_report >= 0.5:
            source_frame = meta.get("frame_count", "?")
            print(
                f"frame={source_frame} fps={fps:.2f} "
                f"inference_ms={infer_ms:.1f} latency_ms={latency_ms:.1f} "
                f"detections={detections}",
                flush=True,
            )
            last_report = now

        if args.show:
            cv2.imshow("YOLO live", annotated)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

    if args.show:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
