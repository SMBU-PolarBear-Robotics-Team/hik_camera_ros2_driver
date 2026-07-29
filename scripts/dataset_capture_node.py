#!/usr/bin/env python3

import csv
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


class DatasetCaptureNode(Node):
    def __init__(self):
        super().__init__("dataset_capture")

        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("trigger_topic", "~/trigger")
        self.declare_parameter("camera_node_name", "/hik_camera_ros2_driver")
        self.declare_parameter("output_dir", "~/hk_dataset")
        self.declare_parameter("filename_prefix", "sample")
        self.declare_parameter("image_format", "jpg")
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("exposure_times", [1000, 2500, 5000, 8000])
        self.declare_parameter("gains", [])
        self.declare_parameter("settle_frames", 3)
        self.declare_parameter("settle_time_sec", 0.15)
        self.declare_parameter("save_latest_without_camera_params", False)

        self.image_topic = self.get_parameter("image_topic").value
        self.trigger_topic = self.get_parameter("trigger_topic").value
        self.camera_node_name = self.get_parameter("camera_node_name").value.rstrip("/")
        self.output_dir = Path(os.path.expanduser(self.get_parameter("output_dir").value))
        self.filename_prefix = self.get_parameter("filename_prefix").value
        self.image_format = self.get_parameter("image_format").value.lower().lstrip(".")
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.exposure_times = [int(v) for v in self.get_parameter("exposure_times").value]
        self.gains = [float(v) for v in self.get_array_parameter("gains", [])]
        self.settle_frames = int(self.get_parameter("settle_frames").value)
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.save_without_params = bool(
            self.get_parameter("save_latest_without_camera_params").value
        )

        if self.gains and len(self.gains) != len(self.exposure_times):
            raise ValueError("gains must be empty or have the same length as exposure_times")
        if self.image_format not in ("jpg", "jpeg", "png", "bmp"):
            raise ValueError("image_format must be one of: jpg, jpeg, png, bmp")

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.output_dir / "manifest.csv"
        self.bridge = CvBridge()
        self.latest_msg = None
        self.latest_frame_id = 0
        self.capture_index = 0
        self.lock = threading.Lock()
        self.capture_lock = threading.Lock()
        self.shutdown_requested = threading.Event()

        self.param_client = self.create_client(
            SetParameters, f"{self.camera_node_name}/set_parameters"
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.trigger_sub = self.create_subscription(
            Empty, self.trigger_topic, self.trigger_callback, 10
        )

        self.ensure_manifest()
        self.input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info(f"Listening image topic: {self.image_topic}")
        self.get_logger().info(f"Listening trigger topic: {self.trigger_topic}")
        self.get_logger().info(f"Saving images to: {self.output_dir}")
        self.get_logger().info(
            "Press Enter to capture one exposure bracket, type q then Enter to quit."
        )

    def image_callback(self, msg):
        with self.lock:
            self.latest_msg = msg
            self.latest_frame_id += 1

    def get_array_parameter(self, name, default):
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return default
        if value is None:
            return default
        return value

    def trigger_callback(self, _msg):
        threading.Thread(target=self.capture_bracket, daemon=True).start()

    def keyboard_loop(self):
        while rclpy.ok() and not self.shutdown_requested.is_set():
            try:
                text = input()
            except EOFError:
                return
            if text.strip().lower() in ("q", "quit", "exit"):
                self.shutdown_requested.set()
                rclpy.shutdown()
                return
            self.capture_bracket()

    def capture_bracket(self):
        if not self.capture_lock.acquire(blocking=False):
            self.get_logger().warn("Capture already running, ignoring trigger")
            return

        try:
            self.capture_index += 1
            bracket_id = self.capture_index
            self.get_logger().info(f"Capture {bracket_id:06d} started")

            for setting_index, exposure_time in enumerate(self.exposure_times):
                gain = self.gains[setting_index] if self.gains else None
                if not self.set_camera_params(exposure_time, gain):
                    if not self.save_without_params:
                        self.get_logger().error("Skipping capture because camera parameters failed")
                        continue

                start_frame = self.current_frame_id()
                target_frame = start_frame + max(1, self.settle_frames)
                deadline = time.monotonic() + 3.0
                while self.current_frame_id() < target_frame and time.monotonic() < deadline:
                    time.sleep(0.01)
                time.sleep(self.settle_time_sec)

                msg = self.latest_image()
                if msg is None:
                    self.get_logger().warn("No image received yet")
                    continue

                self.save_image(msg, bracket_id, setting_index, exposure_time, gain)

            self.get_logger().info(f"Capture {bracket_id:06d} finished")
        finally:
            self.capture_lock.release()

    def set_camera_params(self, exposure_time, gain):
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Parameter service not available: {self.camera_node_name}")
            return False

        request = SetParameters.Request()
        request.parameters.append(
            Parameter(
                name="exposure_time",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=int(exposure_time),
                ),
            )
        )
        if gain is not None:
            request.parameters.append(
                Parameter(
                    name="gain",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=float(gain),
                    ),
                )
            )

        done = threading.Event()
        future = self.param_client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=2.0):
            self.get_logger().warn("Timed out while setting camera parameters")
            return False

        response = future.result()
        if response is None:
            self.get_logger().warn("Camera parameter service returned no response")
            return False

        for result in response.results:
            if not result.successful:
                self.get_logger().warn(f"Failed to set camera parameter: {result.reason}")
                return False
        return True

    def save_image(self, msg, bracket_id, setting_index, exposure_time, gain):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        gain_tag = "auto" if gain is None else f"{gain:.1f}".replace(".", "p")
        filename = (
            f"{self.filename_prefix}_{bracket_id:06d}_{setting_index:02d}_"
            f"exp{int(exposure_time)}_gain{gain_tag}_{stamp}.{self.image_format}"
        )
        path = self.output_dir / filename

        params = []
        if self.image_format in ("jpg", "jpeg"):
            params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

        if not cv2.imwrite(str(path), image, params):
            self.get_logger().error(f"Failed to write image: {path}")
            return

        self.append_manifest(path, msg, bracket_id, setting_index, exposure_time, gain)
        self.get_logger().info(f"Saved {path.name}")

    def latest_image(self):
        with self.lock:
            return self.latest_msg

    def current_frame_id(self):
        with self.lock:
            return self.latest_frame_id

    def ensure_manifest(self):
        if self.manifest_path.exists():
            return
        with self.manifest_path.open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "filename",
                    "bracket_id",
                    "setting_index",
                    "exposure_time",
                    "gain",
                    "stamp_sec",
                    "stamp_nanosec",
                    "frame_id",
                    "width",
                    "height",
                    "encoding",
                ]
            )

    def append_manifest(self, path, msg, bracket_id, setting_index, exposure_time, gain):
        with self.manifest_path.open("a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    path.name,
                    bracket_id,
                    setting_index,
                    exposure_time,
                    "" if gain is None else gain,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.header.frame_id,
                    msg.width,
                    msg.height,
                    msg.encoding,
                ]
            )


def main():
    rclpy.init()
    node = DatasetCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
