import time

import rclpy
from dobot_interfaces.msg import DobotState
from dobot_interfaces.srv import JogCommand
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from dobot_joy.joy_common import JoyMapping, axis_to_jog, deadman_pressed


class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__("dobot_joy_teleop")
        self._declare_parameters()
        self.joy_topic = self.get_parameter("joy.topic").value
        self.deadman_button_index = int(
            self.get_parameter("joy.deadman_button_index").value
        )
        self.estop_button_index = int(self.get_parameter("joy.estop_button_index").value)
        self.coord_type = int(self.get_parameter("joy.coord_type").value)
        self.user = int(self.get_parameter("joy.user").value)
        self.tool = int(self.get_parameter("joy.tool").value)
        self.watchdog_timeout_sec = float(
            self.get_parameter("joy.watchdog_timeout_sec").value
        )
        self.mapping = JoyMapping(
            x_axis_index=int(self.get_parameter("joy.x_axis_index").value),
            y_axis_index=int(self.get_parameter("joy.y_axis_index").value),
            z_axis_index=int(self.get_parameter("joy.z_axis_index").value),
            rz_axis_index=int(self.get_parameter("joy.rz_axis_index").value),
            deadzone=float(self.get_parameter("joy.deadzone").value),
        )

        self.current_axis = None
        self.last_joy_time = 0.0
        self.latest_state = None
        self.jog_client = self.create_client(JogCommand, "/move_jog")
        self.estop_client = self.create_client(Trigger, "/emergency_stop")
        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.create_subscription(DobotState, "/dobot_state", self._on_dobot_state, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            "joy teleop ready: "
            f"topic={self.joy_topic}, deadman_button={self.deadman_button_index}, "
            f"coord_type={self.coord_type}"
        )

    def _declare_parameters(self):
        self.declare_parameter("joy.topic", "/joy")
        self.declare_parameter("joy.deadman_button_index", 4)
        self.declare_parameter("joy.estop_button_index", 1)
        self.declare_parameter("joy.x_axis_index", 1)
        self.declare_parameter("joy.y_axis_index", 0)
        self.declare_parameter("joy.z_axis_index", 4)
        self.declare_parameter("joy.rz_axis_index", 3)
        self.declare_parameter("joy.deadzone", 0.25)
        self.declare_parameter("joy.coord_type", 0)
        self.declare_parameter("joy.user", 0)
        self.declare_parameter("joy.tool", 0)
        self.declare_parameter("joy.watchdog_timeout_sec", 0.4)

    def _on_dobot_state(self, msg: DobotState):
        self.latest_state = msg
        if not self._state_allows_jog(log=False):
            self._stop_jog()

    def _on_joy(self, msg: Joy):
        self.last_joy_time = time.monotonic()
        if self._button_pressed(msg.buttons, self.estop_button_index):
            self._emergency_stop()
            return
        if not deadman_pressed(msg.buttons, self.deadman_button_index):
            self._stop_jog()
            return
        if not self._state_allows_jog(log=True):
            self._stop_jog()
            return

        axis = axis_to_jog(msg.axes, self.mapping)
        if axis is None:
            self._stop_jog()
            return
        if axis == self.current_axis:
            return
        self._stop_jog()
        self._start_jog(axis)

    def _state_allows_jog(self, log: bool) -> bool:
        if self.latest_state is None:
            if log:
                self.get_logger().warn("joy jog rejected: no dobot_state received yet")
            return False
        if not self.latest_state.connected or not self.latest_state.feedback_valid:
            if log:
                self.get_logger().warn("joy jog rejected: robot feedback is not ready")
            return False
        if self.latest_state.error_status or self.latest_state.robot_mode == 9:
            if log:
                self.get_logger().warn("joy jog rejected: robot is in error state")
            return False
        if self.latest_state.enable_status != 1:
            if log:
                self.get_logger().warn("joy jog rejected: robot is not enabled")
            return False
        return True

    def _start_jog(self, axis: str):
        if not self.jog_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/move_jog service is not available")
            return
        request = JogCommand.Request()
        request.axis_id = axis
        request.stop = False
        request.coord_type = self.coord_type
        request.user = self.user
        request.tool = self.tool
        self.current_axis = axis
        future = self.jog_client.call_async(request)
        future.add_done_callback(lambda result: self._on_jog_done(result, axis))

    def _stop_jog(self):
        if self.current_axis is None:
            return
        if not self.jog_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/move_jog service is not available")
            self.current_axis = None
            return
        request = JogCommand.Request()
        request.stop = True
        self.current_axis = None
        self.jog_client.call_async(request)

    def _watchdog(self):
        if self.current_axis is None:
            return
        if time.monotonic() - self.last_joy_time > self.watchdog_timeout_sec:
            self.get_logger().warn("joy watchdog timeout; stopping jog")
            self._stop_jog()

    def _emergency_stop(self):
        self._stop_jog()
        if not self.estop_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/emergency_stop service is not available")
            return
        future = self.estop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_estop_done)

    def _on_jog_done(self, future, axis: str):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"joy jog accepted: {axis}")
            else:
                self.get_logger().error(f"joy jog rejected: {response.message}")
                if self.current_axis == axis:
                    self.current_axis = None
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"joy jog service failed: {exc}")
            if self.current_axis == axis:
                self.current_axis = None

    def _on_estop_done(self, future):
        try:
            response = future.result()
            self.get_logger().error(f"emergency stop response: {response.message}")
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"emergency stop service failed: {exc}")

    @staticmethod
    def _button_pressed(buttons, index: int) -> bool:
        return int(index) >= 0 and int(index) < len(buttons) and int(buttons[index]) == 1


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node._stop_jog()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
