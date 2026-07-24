import json
import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
import yaml

from dobot_handeye.handeye_common import (
    create_charuco_board,
    make_aruco_dictionary,
    matrix_to_transform_dict,
    node_handeye_config,
    transform_to_matrix,
)


class HandeyeCaptureNode(Node):
    def __init__(self, dataset_root=None, dataset_name=None):
        super().__init__("dobot_handeye_capture")
        self.config = node_handeye_config(self)
        self.bridge = CvBridge()
        self.board = create_charuco_board(self.config["board"])
        self.dictionary = make_aruco_dictionary(self.config["board"]["dictionary"])
        root = Path(dataset_root or self.config["samples_dir"])
        session_name = dataset_name or time.strftime("%Y%m%d_%H%M%S")
        self.dataset_dir = root / session_name
        self.samples_dir = self.dataset_dir / "samples"
        self.samples_dir.mkdir(parents=True, exist_ok=True)
        self.created_at = time.strftime("%Y-%m-%d %H:%M:%S")
        self.min_corners = int(self.config["board"].get("min_charuco_corners", 12))

        self.latest_image = None
        self.latest_stamp = None
        self.camera_matrix = None
        self.distortion = None
        self.sample_count = len(list(self.samples_dir.glob("sample_*.json")))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image,
            self.config["image_topic"],
            self._on_image,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.config["camera_info_topic"],
            self._on_camera_info,
            10,
        )
        self._write_dataset_metadata()
        self.get_logger().info(f"handeye dataset: {self.dataset_dir}")

    def _on_image(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_stamp = msg.header.stamp

    def _on_camera_info(self, msg):
        self.camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=float)
        if msg.header.frame_id:
            self.config["camera_frame"] = msg.header.frame_id

    def capture_sample(self):
        if self.latest_image is None or self.camera_matrix is None:
            return False, "waiting for image and camera_info"

        detection = self._detect_board()
        if detection is None:
            return False, "ChArUco board not detected; adjust pose or lighting"
        board_to_camera, corner_count, marker_count, debug_image = detection

        try:
            base_to_flange_msg = self.tf_buffer.lookup_transform(
                self.config["base_frame"],
                self.config["flange_frame"],
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            return False, f"TF unavailable: {exc}"

        base_to_flange = transform_to_matrix(base_to_flange_msg)
        self.sample_count += 1
        stem = f"sample_{self.sample_count:03d}"
        color_path = self.samples_dir / f"{stem}_color.png"
        debug_path = self.samples_dir / f"{stem}_debug.png"
        cv2.imwrite(str(color_path), self.latest_image)
        cv2.imwrite(str(debug_path), debug_image)
        sample = {
            "sample_id": self.sample_count,
            "stamp_sec": time.time(),
            "base_frame": self.config["base_frame"],
            "flange_frame": self.config["flange_frame"],
            "camera_frame": self.config["camera_frame"],
            "image_topic": self.config["image_topic"],
            "camera_info_topic": self.config["camera_info_topic"],
            "color_image": str(color_path.relative_to(self.dataset_dir)),
            "debug_image": str(debug_path.relative_to(self.dataset_dir)),
            "camera_info": {
                "k": self.camera_matrix.reshape(-1).tolist(),
                "d": self.distortion.reshape(-1).tolist(),
            },
            "base_to_flange": matrix_to_transform_dict(base_to_flange),
            "board_to_camera": matrix_to_transform_dict(board_to_camera),
            "charuco_corners": int(corner_count),
            "aruco_markers": int(marker_count),
            "board": self.config["board"],
        }
        path = self.samples_dir / f"{stem}.json"
        path.write_text(json.dumps(sample, indent=2))
        self._write_dataset_metadata()
        return True, f"saved {path} with {corner_count} ChArUco corners"

    def _write_dataset_metadata(self):
        payload = {
            "created_at": self.created_at,
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "dataset_dir": str(self.dataset_dir),
            "samples_dir": "samples",
            "sample_count": int(self.sample_count),
            "base_frame": self.config["base_frame"],
            "flange_frame": self.config["flange_frame"],
            "camera_frame": self.config["camera_frame"],
            "image_topic": self.config["image_topic"],
            "camera_info_topic": self.config["camera_info_topic"],
            "board": self.config["board"],
        }
        (self.dataset_dir / "dataset.yaml").write_text(
            yaml.safe_dump(payload, sort_keys=False)
        )

    def _detect_board(self):
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self.dictionary)
        debug_image = self.latest_image.copy()
        if ids is None or len(ids) == 0:
            return None
        cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners,
            ids,
            gray,
            self.board,
            self.camera_matrix,
            self.distortion,
        )
        if charuco_ids is None or count < self.min_corners:
            return None
        cv2.aruco.drawDetectedCornersCharuco(debug_image, charuco_corners, charuco_ids)

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            self.board,
            self.camera_matrix,
            self.distortion,
            None,
            None,
        )
        if not ok:
            return None
        if hasattr(cv2, "drawFrameAxes"):
            cv2.drawFrameAxes(
                debug_image,
                self.camera_matrix,
                self.distortion,
                rvec,
                tvec,
                float(self.config["board"]["square_length_m"]) * 2.0,
            )

        rotation, _jacobian = cv2.Rodrigues(rvec)
        board_to_camera = np.eye(4, dtype=float)
        board_to_camera[:3, :3] = rotation
        board_to_camera[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
        return board_to_camera, int(count), int(len(ids)), debug_image


def _spin(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)


def main(args=None):
    parser = argparse.ArgumentParser(description="Capture Dobot hand-eye samples")
    parser.add_argument("--dataset-root", default=None)
    parser.add_argument("--dataset-name", default=None)
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = HandeyeCaptureNode(
        dataset_root=parsed.dataset_root,
        dataset_name=parsed.dataset_name,
    )
    stop_event = threading.Event()
    thread = threading.Thread(target=_spin, args=(node, stop_event), daemon=True)
    thread.start()
    try:
        node.get_logger().info("press Enter to save a sample; type q then Enter to quit")
        while rclpy.ok():
            command = input("handeye> ").strip().lower()
            if command in {"q", "quit", "exit"}:
                break
            success, message = node.capture_sample()
            if success:
                node.get_logger().info(message)
            else:
                node.get_logger().warn(message)
    finally:
        stop_event.set()
        thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
