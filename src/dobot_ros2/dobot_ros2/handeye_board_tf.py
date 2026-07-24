import math
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from dobot_ros2.handeye_common import (
    create_charuco_board,
    make_aruco_dictionary,
    matrix_to_xyz_quat,
    node_handeye_config,
    transform_to_matrix,
)


class PoseStabilityWindow:
    def __init__(self, size=30):
        self.transforms = deque(maxlen=int(size))

    def add(self, transform):
        self.transforms.append(np.asarray(transform, dtype=float))

    def report(self):
        if not self.transforms:
            return None

        translations = np.asarray(
            [transform[:3, 3] for transform in self.transforms],
            dtype=float,
        )
        mean_translation = translations.mean(axis=0)
        translation_errors = np.linalg.norm(translations - mean_translation, axis=1)

        quaternions = []
        for transform in self.transforms:
            _xyz, quat = matrix_to_xyz_quat(transform)
            quat = np.asarray(quat, dtype=float)
            if quaternions and np.dot(quat, quaternions[0]) < 0:
                quat = -quat
            quaternions.append(quat)
        mean_quaternion = np.asarray(quaternions, dtype=float).mean(axis=0)
        mean_quaternion = mean_quaternion / np.linalg.norm(mean_quaternion)
        rotation_errors = np.asarray(
            [_quat_angle_deg(quat, mean_quaternion) for quat in quaternions],
            dtype=float,
        )

        return {
            "count": len(self.transforms),
            "translation": mean_translation.tolist(),
            "translation_rms_mm": float(
                np.sqrt(np.mean(translation_errors**2)) * 1000.0
            ),
            "translation_max_mm": float(np.max(translation_errors) * 1000.0),
            "rotation_rms_deg": float(
                math.sqrt(np.mean(rotation_errors**2))
            ),
            "rotation_max_deg": float(np.max(rotation_errors)),
        }


class HandeyeBoardTfNode(Node):
    def __init__(self):
        super().__init__("dobot_handeye_board_tf")
        self.config = node_handeye_config(self)
        self.bridge = CvBridge()
        self.board = create_charuco_board(self.config["board"])
        self.dictionary = make_aruco_dictionary(self.config["board"]["dictionary"])
        self.min_corners = int(self.config["board"].get("min_charuco_corners", 12))

        self.camera_matrix = None
        self.distortion = None
        self.last_report_sec = 0.0
        self.stability = PoseStabilityWindow(size=30)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            CameraInfo,
            self.config["camera_info_topic"],
            self._on_camera_info,
            10,
        )
        self.create_subscription(
            Image,
            self.config["image_topic"],
            self._on_image,
            10,
        )
        self.get_logger().info(
            "publishing detected board TF "
            f"{self.config['camera_frame']} -> {self.config['board_frame']}"
        )

    def _on_camera_info(self, msg):
        self.camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=float)
        if msg.header.frame_id:
            self.config["camera_frame"] = msg.header.frame_id

    def _on_image(self, msg):
        if self.camera_matrix is None:
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detection = self._detect_board(image)
        if detection is None:
            return

        camera_to_board, corner_count, marker_count = detection
        self._publish_board_tf(camera_to_board, msg.header.stamp)
        self._update_base_report(camera_to_board, corner_count, marker_count)

    def _detect_board(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
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
        camera_to_board = np.eye(4, dtype=float)
        camera_to_board[:3, :3] = rotation
        camera_to_board[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
        return camera_to_board, int(count), int(len(ids))

    def _publish_board_tf(self, camera_to_board, stamp):
        msg = _matrix_to_transform_stamped(
            camera_to_board,
            self.config["camera_frame"],
            self.config["board_frame"],
        )
        msg.header.stamp = stamp
        self.tf_broadcaster.sendTransform(msg)

    def _update_base_report(self, camera_to_board, corner_count, marker_count):
        try:
            base_to_camera_msg = self.tf_buffer.lookup_transform(
                self.config["base_frame"],
                self.config["camera_frame"],
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return

        base_to_camera = transform_to_matrix(base_to_camera_msg)
        base_to_board = base_to_camera @ camera_to_board
        self.stability.add(base_to_board)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_report_sec < 1.0:
            return
        self.last_report_sec = now_sec
        report = self.stability.report()
        if report is None:
            return
        xyz = report["translation"]
        self.get_logger().info(
            "board in base "
            f"x={xyz[0]:.4f} y={xyz[1]:.4f} z={xyz[2]:.4f} m; "
            f"window={report['count']}; "
            f"rms={report['translation_rms_mm']:.2f} mm/"
            f"{report['rotation_rms_deg']:.2f} deg; "
            f"max={report['translation_max_mm']:.2f} mm/"
            f"{report['rotation_max_deg']:.2f} deg; "
            f"corners={corner_count}; markers={marker_count}"
        )


def _matrix_to_transform_stamped(matrix, parent_frame, child_frame):
    xyz, quat = matrix_to_xyz_quat(matrix)
    msg = TransformStamped()
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = float(xyz[0])
    msg.transform.translation.y = float(xyz[1])
    msg.transform.translation.z = float(xyz[2])
    msg.transform.rotation.x = float(quat[0])
    msg.transform.rotation.y = float(quat[1])
    msg.transform.rotation.z = float(quat[2])
    msg.transform.rotation.w = float(quat[3])
    return msg


def _quat_angle_deg(quat, reference):
    dot = abs(float(np.dot(quat, reference)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeBoardTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
