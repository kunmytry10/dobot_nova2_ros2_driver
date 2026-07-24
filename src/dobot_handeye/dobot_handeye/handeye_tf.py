import argparse
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
import yaml

from dobot_handeye.handeye_solve import default_result_file


def load_result(path):
    data = yaml.safe_load(Path(path).read_text()) or {}
    required = ("parent_frame", "child_frame", "translation", "rotation_xyzw")
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(f"missing keys in handeye result: {', '.join(missing)}")
    return data


class HandeyeTfNode(Node):
    def __init__(self, result_file):
        super().__init__("dobot_handeye_tf")
        self.result_file = result_file
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_result()

    def publish_result(self):
        data = load_result(self.result_file)
        translation = data["translation"]
        rotation = data["rotation_xyzw"]

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = data["parent_frame"]
        msg.child_frame_id = data["child_frame"]
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(rotation[0])
        msg.transform.rotation.y = float(rotation[1])
        msg.transform.rotation.z = float(rotation[2])
        msg.transform.rotation.w = float(rotation[3])
        self.broadcaster.sendTransform(msg)
        self.get_logger().info(
            f"published static TF {msg.header.frame_id} -> {msg.child_frame_id}"
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description="Publish Dobot hand-eye static TF")
    parser.add_argument("--dataset", default=None)
    parser.add_argument("--result-file", default=None)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = HandeyeTfNode(default_result_file(args.dataset, args.result_file))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
