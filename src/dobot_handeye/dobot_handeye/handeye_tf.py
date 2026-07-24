import argparse
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
import yaml

from dobot_handeye.handeye_common import (
    invert_matrix,
    matrix_to_xyz_quat,
    transform_to_matrix,
    xyz_quat_to_matrix,
)
from dobot_handeye.handeye_solve import default_result_file


def load_result(path):
    data = yaml.safe_load(Path(path).read_text()) or {}
    required = ("parent_frame", "child_frame", "translation", "rotation_xyzw")
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(f"missing keys in handeye result: {', '.join(missing)}")
    return data


class HandeyeTfNode(Node):
    def __init__(self, result_file, output_child_frame=None):
        super().__init__("dobot_handeye_tf")
        self.result_file = result_file
        self.output_child_frame = output_child_frame
        self.result_data = load_result(self.result_file)
        self.published = False
        self.broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.publish_result)

    def publish_result(self):
        if self.published:
            return

        data = self.result_data
        output_child_frame = self.output_child_frame or data["child_frame"]
        try:
            matrix = handeye_matrix_for_output_child(
                xyz_quat_to_matrix(data["translation"], data["rotation_xyzw"]),
                data["child_frame"],
                output_child_frame,
                self._lookup_transform_matrix,
            )
        except Exception as exc:
            self.get_logger().warn(
                "waiting for camera internal TF "
                f"{output_child_frame} -> {data['child_frame']}: {exc}"
            )
            return

        translation, rotation = matrix_to_xyz_quat(matrix)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = data["parent_frame"]
        msg.child_frame_id = output_child_frame
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(rotation[0])
        msg.transform.rotation.y = float(rotation[1])
        msg.transform.rotation.z = float(rotation[2])
        msg.transform.rotation.w = float(rotation[3])
        self.broadcaster.sendTransform(msg)
        self.published = True
        self.get_logger().info(
            f"published static TF {msg.header.frame_id} -> {msg.child_frame_id}"
        )

    def _lookup_transform_matrix(self, target_frame, source_frame):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=0.1),
        )
        return transform_to_matrix(transform)


def handeye_matrix_for_output_child(
    parent_to_calibrated_child,
    calibrated_child_frame,
    output_child_frame,
    lookup_transform_matrix,
):
    if not output_child_frame or output_child_frame == calibrated_child_frame:
        return np.asarray(parent_to_calibrated_child, dtype=float)

    output_to_calibrated = lookup_transform_matrix(
        output_child_frame,
        calibrated_child_frame,
    )
    return np.asarray(parent_to_calibrated_child, dtype=float) @ invert_matrix(
        output_to_calibrated
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description="Publish Dobot hand-eye static TF")
    parser.add_argument("--dataset", default=None)
    parser.add_argument("--result-file", default=None)
    parser.add_argument("--output-child-frame", default=None)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = HandeyeTfNode(
        default_result_file(args.dataset, args.result_file),
        output_child_frame=args.output_child_frame,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
