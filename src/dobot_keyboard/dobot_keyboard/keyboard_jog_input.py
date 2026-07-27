import os
import select
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dobot_keyboard.keyboard_common import KeyboardInputEventFilter, parse_key_message


INPUT_EVENT_FORMAT = "llHHI"
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FORMAT)


class KeyboardJogInputNode(Node):
    def __init__(self):
        super().__init__("dobot_keyboard_jog_input")
        self.declare_parameter("input_topic", "/keyboard/input")
        self.declare_parameter("device", "/dev/input/event0")
        self.declare_parameter("poll_timeout_sec", 0.1)
        self.input_topic = self.get_parameter("input_topic").value
        self.device = self.get_parameter("device").value
        self.poll_timeout_sec = float(self.get_parameter("poll_timeout_sec").value)
        self.publisher = self.create_publisher(String, self.input_topic, 10)
        self.get_logger().info(
            f"publishing keyboard jog events from {self.device} to {self.input_topic}"
        )

    def publish_event(self, message: str):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)


def _read_input_event(fd: int):
    data = os.read(fd, INPUT_EVENT_SIZE)
    if len(data) != INPUT_EVENT_SIZE:
        return None
    sec, usec, event_type, code, value = struct.unpack(INPUT_EVENT_FORMAT, data)
    del sec, usec
    return event_type, code, value


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJogInputNode()
    event_filter = KeyboardInputEventFilter()
    fd = os.open(node.device, os.O_RDONLY | os.O_NONBLOCK)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([fd], [], [], node.poll_timeout_sec)
            if not readable:
                continue
            event = _read_input_event(fd)
            if event is None:
                continue
            message = event_filter.handle_event(*event)
            if message is None:
                continue
            node.publish_event(message)
            if parse_key_message(message) == ("down", "esc"):
                break
    finally:
        node.publish_event("stop")
        rclpy.spin_once(node, timeout_sec=0.05)
        os.close(fd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
