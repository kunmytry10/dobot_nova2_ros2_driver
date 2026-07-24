import sys
import time

import rclpy
from rclpy.node import Node

from dobot_handeye.handeye_common import node_handeye_config


class HandeyeCheckNode(Node):
    def __init__(self):
        super().__init__("dobot_handeye_check")
        config = node_handeye_config(self)
        self.required_topics = [
            config["image_topic"],
            config["camera_info_topic"],
            "/joint_states",
            "/tcp_pose",
            "/dobot_state",
            "/tf",
            "/tf_static",
        ]

    def check_topics(self):
        topics = {name for name, _types in self.get_topic_names_and_types()}
        missing = [topic for topic in self.required_topics if topic not in topics]
        for topic in self.required_topics:
            status = "OK" if topic in topics else "MISSING"
            self.get_logger().info(f"{status}: {topic}")
        return missing


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeCheckNode()
    try:
        deadline = time.time() + 3.0
        missing = node.check_topics()
        while missing and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            missing = node.check_topics()
        if missing:
            node.get_logger().error("missing required topics: " + ", ".join(missing))
            sys.exit(1)
        node.get_logger().info("handeye inputs are ready")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
