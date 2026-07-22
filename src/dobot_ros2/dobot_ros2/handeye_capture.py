import json
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

from dobot_ros2.handeye_common import (
    create_charuco_board,
    make_aruco_dictionary,
    matrix_to_transform_dict,
    node_handeye_config,
    transform_to_matrix,
)


class HandeyeCaptureNode(Node):
    def __init__(self):
        super().__init__("dobot_handeye_capture")
        self.config = node_handeye_config(self)
        self.bridge = CvBridge()
        self.board = create_charuco_board(self.config["board"])
        self.dictionary = make_aruco_dictionary(self.config["board"]["dictionary"])
        self.samples_dir = Path(self.config["samples_dir"])
        self.samples_dir.mkdir(parents=True, exist_ok=True)
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
        board_to_camera, corner_count = detection

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
        sample = {
            "sample_id": self.sample_count,
            "stamp_sec": time.time(),
            "base_frame": self.config["base_frame"],
            "flange_frame": self.config["flange_frame"],
            "camera_frame": self.config["camera_frame"],
            "base_to_flange": matrix_to_transform_dict(base_to_flange),
            "board_to_camera": matrix_to_transform_dict(board_to_camera),
            "charuco_corners": int(corner_count),
        }
        path = self.samples_dir / f"sample_{self.sample_count:03d}.json"
        path.write_text(json.dumps(sample, indent=2))
        return True, f"saved {path} with {corner_count} ChArUco corners"

    def _detect_board(self):
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self.dictionary)
        if ids is None or len(ids) == 0:
            return None

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

        rotation, _jacobian = cv2.Rodrigues(rvec)
        board_to_camera = np.eye(4, dtype=float)
        board_to_camera[:3, :3] = rotation
        board_to_camera[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
        return board_to_camera, int(count)


def _spin(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeCaptureNode()
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
