#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class LatestImageWriter(Node):
    def __init__(self, image_topic, output_dir, frame_stride, jpeg_quality):
        super().__init__("ros_image_latest_writer")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.image_path = self.output_dir / "latest.jpg"
        self.meta_path = self.output_dir / "latest.txt"
        self.frame_stride = max(1, frame_stride)
        self.jpeg_quality = int(jpeg_quality)
        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_log_time = time.monotonic()
        self.last_log_count = 0
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(f"Writing latest frame to: {self.image_path}")

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.frame_stride != 0:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return

        tmp_image_path = self.output_dir / "latest.tmp.jpg"
        tmp_meta_path = self.output_dir / "latest.tmp.txt"
        cv2.imwrite(
            str(tmp_image_path),
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )

        now_ns = time.time_ns()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        tmp_meta_path.write_text(
            f"frame_count={self.frame_count}\n"
            f"stamp_ns={stamp_ns}\n"
            f"write_ns={now_ns}\n"
            f"width={msg.width}\n"
            f"height={msg.height}\n"
            f"encoding={msg.encoding}\n"
        )
        os.replace(tmp_image_path, self.image_path)
        os.replace(tmp_meta_path, self.meta_path)

        now = time.monotonic()
        if now - self.last_log_time >= 2.0:
            fps = (self.frame_count - self.last_log_count) / (now - self.last_log_time)
            self.get_logger().info(f"Bridge input FPS: {fps:.1f}")
            self.last_log_time = now
            self.last_log_count = self.frame_count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/camera/image")
    parser.add_argument("--output-dir", default="/tmp/hk_yolo_live")
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    args = parser.parse_args()

    rclpy.init()
    node = LatestImageWriter(
        args.image_topic,
        args.output_dir,
        args.frame_stride,
        args.jpeg_quality,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
