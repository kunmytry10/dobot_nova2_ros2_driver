import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dobot_keyboard.keyboard_common import normalize_key


HELP_TEXT = """Keyboard teleop:
  w/s: x +/-    a/d: y +/-    r/f: z +/-
  z/x: rx +/-   t/g: ry +/-   c/v: rz +/-
  space: toggle gripper       q: reset simulation no-op
  ESC: quit
"""


class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__("dobot_keyboard_input")
        self.declare_parameter("input_topic", "/keyboard/input")
        self.declare_parameter("poll_timeout_sec", 0.1)
        topic = self.get_parameter("input_topic").value
        self.poll_timeout_sec = float(self.get_parameter("poll_timeout_sec").value)
        self.publisher = self.create_publisher(String, topic, 10)
        self.get_logger().info(f"publishing keyboard input to {topic}")

    def publish_key(self, key: str):
        msg = String()
        msg.data = normalize_key(key)
        self.publisher.publish(msg)


def _read_key(stream, timeout_sec: float) -> str:
    readable, _, _ = select.select([stream], [], [], timeout_sec)
    if not readable:
        return ""
    char = stream.read(1)
    if char == "\x1b":
        return "esc"
    if char == " ":
        return "space"
    return char


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    input_stream = sys.stdin
    close_stream = False
    try:
        input_stream = open("/dev/tty", "r", encoding="utf-8")
        close_stream = True
    except OSError:
        node.get_logger().warn("failed to open /dev/tty; falling back to stdin")

    old_settings = termios.tcgetattr(input_stream.fileno())
    print(HELP_TEXT)
    try:
        tty.setcbreak(input_stream.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key(input_stream, node.poll_timeout_sec)
            if not key:
                continue
            node.publish_key(key)
            if normalize_key(key) == "esc":
                break
    finally:
        termios.tcsetattr(input_stream.fileno(), termios.TCSADRAIN, old_settings)
        if close_stream:
            input_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
