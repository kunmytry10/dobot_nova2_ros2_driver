import time
from math import degrees

import rclpy
from dobot_interfaces.msg import DobotState, GripperStatus
from dobot_interfaces.srv import GripperCommand, GripperState, JogCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy, JoyFeedback, JoyFeedbackArray
from std_srvs.srv import Trigger

from dobot_joy.joy_common import (
    JoyMapping,
    axis_to_jog,
    button_pressed,
    clamp,
    deadman_pressed,
    trigger_pressed,
)


class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__("dobot_joy_teleop")
        self._declare_parameters()
        self.joy_topic = self.get_parameter("joy.topic").value
        self.deadman_button_index = int(
            self.get_parameter("joy.deadman_button_index").value
        )
        self.estop_button_index = int(self.get_parameter("joy.estop_button_index").value)
        self.toggle_gripper_button_index = int(
            self.get_parameter("joy.toggle_gripper_button_index").value
        )
        self.clear_error_button_index = int(
            self.get_parameter("joy.clear_error_button_index").value
        )
        self.stop_button_indices = list(
            self.get_parameter("joy.stop_button_indices").value
        )
        self.coord_type = int(self.get_parameter("joy.coord_type").value)
        self.user = int(self.get_parameter("joy.user").value)
        self.tool = int(self.get_parameter("joy.tool").value)
        self.watchdog_timeout_sec = float(
            self.get_parameter("joy.watchdog_timeout_sec").value
        )
        self.gripper_min_opening_mm = float(
            self.get_parameter("joy.gripper_min_opening_mm").value
        )
        self.gripper_max_opening_mm = float(
            self.get_parameter("joy.gripper_max_opening_mm").value
        )
        self.gripper_toggle_threshold_mm = float(
            self.get_parameter("joy.gripper_toggle_threshold_mm").value
        )
        self.gripper_force_percent = int(
            self.get_parameter("joy.gripper_force_percent").value
        )
        self.gripper_wait = bool(self.get_parameter("joy.gripper_wait").value)
        self.gripper_timeout_sec = float(
            self.get_parameter("joy.gripper_timeout_sec").value
        )
        self.gripper_command_period_sec = float(
            self.get_parameter("joy.gripper_command_period_sec").value
        )
        self.enable_rumble = bool(self.get_parameter("joy.enable_rumble").value)
        self.rumble_topic = str(self.get_parameter("joy.rumble_topic").value)
        self.rumble_duration_sec = float(
            self.get_parameter("joy.rumble_duration_sec").value
        )
        self.rumble_intensity = float(self.get_parameter("joy.rumble_intensity").value)
        self.joint_limit_check = bool(self.get_parameter("joy.joint_limit_check").value)
        self.joint_limit_margin_deg = float(
            self.get_parameter("joy.joint_limit_margin_deg").value
        )
        self.joint_lower_limits_deg = list(
            self.get_parameter("joint_lower_limits_deg").value
        )
        self.joint_upper_limits_deg = list(
            self.get_parameter("joint_upper_limits_deg").value
        )
        self.mapping = JoyMapping(
            x_axis_index=int(self.get_parameter("joy.x_axis_index").value),
            x_axis_sign=float(self.get_parameter("joy.x_axis_sign").value),
            y_axis_index=int(self.get_parameter("joy.y_axis_index").value),
            y_axis_sign=float(self.get_parameter("joy.y_axis_sign").value),
            z_axis_index=int(self.get_parameter("joy.z_axis_index").value),
            z_axis_sign=float(self.get_parameter("joy.z_axis_sign").value),
            rz_axis_index=int(self.get_parameter("joy.rz_axis_index").value),
            rz_axis_sign=float(self.get_parameter("joy.rz_axis_sign").value),
            lt_axis_index=int(self.get_parameter("joy.lt_axis_index").value),
            rt_axis_index=int(self.get_parameter("joy.rt_axis_index").value),
            deadzone=float(self.get_parameter("joy.deadzone").value),
            gripper_step_mm=float(self.get_parameter("joy.gripper_step_mm").value),
        )

        self.current_axis = None
        self.last_joy_time = 0.0
        self.latest_state = None
        self.latest_joint_degrees = None
        self.latest_buttons = []
        self.trigger_neutral_axes = None
        self.latest_gripper_opening_mm = None
        self.gripper_busy = False
        self.last_gripper_command_time = 0.0
        self.active_gripper_axis = None
        self.latest_object_detected = False
        self.rumble_until = 0.0
        self.jog_client = self.create_client(JogCommand, "/move_jog")
        self.estop_client = self.create_client(Trigger, "/emergency_stop")
        self.clear_error_client = self.create_client(Trigger, "/clear_error")
        self.gripper_state_client = self.create_client(GripperState, "/get_gripper_state")
        self.gripper_move_client = self.create_client(GripperCommand, "/gripper_move")
        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.create_subscription(DobotState, "/dobot_state", self._on_dobot_state, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(
            GripperStatus, "/gripper_state", self._on_gripper_status, 10
        )
        self.rumble_pub = self.create_publisher(JoyFeedbackArray, self.rumble_topic, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            "joy teleop ready: "
            f"topic={self.joy_topic}, deadman_button={self.deadman_button_index}, "
            f"coord_type={self.coord_type}, deadzone={self.mapping.deadzone}, "
            f"gripper_step={self.mapping.gripper_step_mm}mm"
        )

    def destroy_node(self):
        self._stop_jog(wait=True, force=True)
        self._publish_rumble(0.0)
        super().destroy_node()

    def _declare_parameters(self):
        self.declare_parameter("joy.topic", "/joy")
        self.declare_parameter("joy.deadman_button_index", 4)
        self.declare_parameter("joy.estop_button_index", 1)
        self.declare_parameter("joy.toggle_gripper_button_index", 0)
        self.declare_parameter("joy.clear_error_button_index", 2)
        self.declare_parameter("joy.stop_button_indices", [6, 7])
        self.declare_parameter("joy.x_axis_index", 1)
        self.declare_parameter("joy.x_axis_sign", -1.0)
        self.declare_parameter("joy.y_axis_index", 0)
        self.declare_parameter("joy.y_axis_sign", -1.0)
        self.declare_parameter("joy.z_axis_index", 4)
        self.declare_parameter("joy.z_axis_sign", 1.0)
        self.declare_parameter("joy.rz_axis_index", 3)
        self.declare_parameter("joy.rz_axis_sign", -1.0)
        self.declare_parameter("joy.lt_axis_index", 2)
        self.declare_parameter("joy.rt_axis_index", 5)
        self.declare_parameter("joy.deadzone", 0.25)
        self.declare_parameter("joy.gripper_step_mm", 2.0)
        self.declare_parameter("joy.gripper_min_opening_mm", 0.0)
        self.declare_parameter("joy.gripper_max_opening_mm", 95.0)
        self.declare_parameter("joy.gripper_toggle_threshold_mm", 45.0)
        self.declare_parameter("joy.gripper_force_percent", 50)
        self.declare_parameter("joy.gripper_wait", False)
        self.declare_parameter("joy.gripper_timeout_sec", 2.0)
        self.declare_parameter("joy.gripper_command_period_sec", 0.2)
        self.declare_parameter("joy.enable_rumble", True)
        self.declare_parameter("joy.rumble_topic", "/joy/set_feedback")
        self.declare_parameter("joy.rumble_duration_sec", 0.2)
        self.declare_parameter("joy.rumble_intensity", 0.7)
        self.declare_parameter("joy.joint_limit_check", True)
        self.declare_parameter("joy.joint_limit_margin_deg", 5.0)
        self.declare_parameter(
            "joint_lower_limits_deg",
            [-360.0, -180.0, -156.0, -360.0, -360.0, -360.0],
        )
        self.declare_parameter(
            "joint_upper_limits_deg",
            [360.0, 180.0, 156.0, 360.0, 360.0, 360.0],
        )
        self.declare_parameter("joy.coord_type", 0)
        self.declare_parameter("joy.user", 0)
        self.declare_parameter("joy.tool", 0)
        self.declare_parameter("joy.watchdog_timeout_sec", 0.4)

    def _on_dobot_state(self, msg: DobotState):
        self.latest_state = msg
        if not self._state_allows_jog(log=False):
            self._stop_jog(force=True)

    def _on_joint_state(self, msg: JointState):
        self.latest_joint_degrees = [degrees(position) for position in msg.position[:6]]
        if self.current_axis is not None and not self._joints_allow_jog(log=True):
            self._stop_jog(force=True)

    def _on_gripper_status(self, msg: GripperStatus):
        if msg.success:
            self.latest_gripper_opening_mm = msg.opening_mm
        if msg.object_detected and not self.latest_object_detected:
            self._start_rumble()
        self.latest_object_detected = bool(msg.object_detected)

    def _on_joy(self, msg: Joy):
        self.last_joy_time = time.monotonic()
        if self.trigger_neutral_axes is None:
            self.trigger_neutral_axes = list(msg.axes)
        if button_pressed(msg.buttons, self.estop_button_index):
            self._emergency_stop()
            self.latest_buttons = list(msg.buttons)
            return
        if self._button_edge(msg.buttons, self.clear_error_button_index):
            self._clear_error()
            self.latest_buttons = list(msg.buttons)
            return
        if self._button_edge(msg.buttons, self.toggle_gripper_button_index):
            self._toggle_gripper()
        if any(button_pressed(msg.buttons, index) for index in self.stop_button_indices):
            self._stop_jog(force=True)
            self.latest_buttons = list(msg.buttons)
            return
        self._handle_gripper_axis(msg.axes)
        if not deadman_pressed(msg.buttons, self.deadman_button_index):
            self._stop_jog(force=True)
            self.latest_buttons = list(msg.buttons)
            return
        if not self._state_allows_jog(log=True):
            self._stop_jog(force=True)
            self.latest_buttons = list(msg.buttons)
            return
        if not self._joints_allow_jog(log=True):
            self._stop_jog(force=True)
            self.latest_buttons = list(msg.buttons)
            return

        axis = axis_to_jog(msg.axes, self.mapping)
        if axis is None:
            self._stop_jog(force=True)
            self.latest_buttons = list(msg.buttons)
            return
        if axis == self.current_axis:
            self.latest_buttons = list(msg.buttons)
            return
        if self.current_axis is not None:
            self._stop_jog(force=True)
        self._start_jog(axis)
        self.latest_buttons = list(msg.buttons)

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

    def _joints_allow_jog(self, log: bool) -> bool:
        if not self.joint_limit_check or self.latest_joint_degrees is None:
            return True
        margin = max(0.0, self.joint_limit_margin_deg)
        for index, value in enumerate(self.latest_joint_degrees):
            if index >= len(self.joint_lower_limits_deg) or index >= len(
                self.joint_upper_limits_deg
            ):
                return True
            lower = float(self.joint_lower_limits_deg[index])
            upper = float(self.joint_upper_limits_deg[index])
            if value <= lower + margin or value >= upper - margin:
                if log:
                    self.get_logger().warn(
                        "joy jog rejected: "
                        f"joint{index + 1} near limit "
                        f"({value:.2f} deg, limits {lower:.1f}..{upper:.1f}, "
                        f"margin {margin:.1f})"
                    )
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

    def _stop_jog(self, wait: bool = False, force: bool = False):
        if self.current_axis is None and not force and not wait:
            return
        if not self.jog_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/move_jog service is not available")
            self.current_axis = None
            return
        request = JogCommand.Request()
        request.stop = True
        self.current_axis = None
        future = self.jog_client.call_async(request)
        if wait:
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

    def _watchdog(self):
        self._update_rumble()
        if self.current_axis is None:
            return
        if time.monotonic() - self.last_joy_time > self.watchdog_timeout_sec:
            self.get_logger().warn("joy watchdog timeout; stopping jog")
            self._stop_jog(force=True)

    def _emergency_stop(self):
        self._stop_jog(force=True)
        if not self.estop_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/emergency_stop service is not available")
            return
        future = self.estop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_estop_done)

    def _clear_error(self):
        self._stop_jog(force=True)
        if not self.clear_error_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/clear_error service is not available")
            return
        future = self.clear_error_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_clear_error_done)

    def _handle_gripper_axis(self, axes):
        lt_pressed = trigger_pressed(
            axes,
            self.trigger_neutral_axes,
            self.mapping.lt_axis_index,
            self.mapping.deadzone,
        )
        rt_pressed = trigger_pressed(
            axes,
            self.trigger_neutral_axes,
            self.mapping.rt_axis_index,
            self.mapping.deadzone,
        )
        requested_axis = None
        if lt_pressed and not rt_pressed:
            requested_axis = "close"
        elif rt_pressed and not lt_pressed:
            requested_axis = "open"

        if requested_axis is None:
            if self.active_gripper_axis is not None:
                self._stop_gripper_motion()
            self.active_gripper_axis = None
            return

        if requested_axis == self.active_gripper_axis:
            return

        now = time.monotonic()
        if now - self.last_gripper_command_time < self.gripper_command_period_sec:
            return
        if self.active_gripper_axis is not None:
            self._stop_gripper_motion()
        self.active_gripper_axis = requested_axis
        target = (
            self.gripper_min_opening_mm
            if requested_axis == "close"
            else self.gripper_max_opening_mm
        )
        self._move_gripper(target, f"gripper jog {requested_axis}", allow_busy=True)

    def _toggle_gripper(self):
        opening = self.latest_gripper_opening_mm
        if opening is None:
            self._request_gripper_state(None)
            return
        if opening <= self.gripper_toggle_threshold_mm:
            target = self.gripper_max_opening_mm
        else:
            target = self.gripper_min_opening_mm
        self._move_gripper(target, "gripper toggle")

    def _request_gripper_state(self, pending_delta):
        if self.gripper_busy:
            return
        if not self.gripper_state_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/get_gripper_state service is not available")
            return
        self.gripper_busy = True
        future = self.gripper_state_client.call_async(GripperState.Request())
        future.add_done_callback(
            lambda result: self._on_gripper_state(result, pending_delta)
        )

    def _on_gripper_state(self, future, pending_delta):
        command_sent = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"get gripper state failed: {response.message}")
                return
            self.latest_gripper_opening_mm = response.opening_mm
            self.gripper_busy = False
            if pending_delta is None:
                self._toggle_gripper()
                command_sent = self.gripper_busy
                return
            if pending_delta == "stop":
                self._move_gripper(
                    response.opening_mm,
                    "gripper jog stop",
                    allow_busy=True,
                )
                command_sent = self.gripper_busy
                return
            target = clamp(
                response.opening_mm + pending_delta,
                self.gripper_min_opening_mm,
                self.gripper_max_opening_mm,
            )
            self._move_gripper(target, "gripper fine adjust")
            command_sent = self.gripper_busy
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"get gripper state failed: {exc}")
        finally:
            if not command_sent:
                self.gripper_busy = False

    def _stop_gripper_motion(self):
        if self.latest_gripper_opening_mm is not None:
            self._move_gripper(
                self.latest_gripper_opening_mm,
                "gripper jog stop",
                allow_busy=True,
            )
            return
        self._request_gripper_state("stop")

    def _move_gripper(self, opening_mm: float, label: str, allow_busy: bool = False):
        if self.gripper_busy and not allow_busy:
            return
        if not self.gripper_move_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/gripper_move service is not available")
            return
        request = GripperCommand.Request()
        request.opening_mm = float(opening_mm)
        request.position_permille = 0
        request.force_percent = self.gripper_force_percent
        request.force_n = -1.0
        request.wait = self.gripper_wait
        request.timeout_sec = self.gripper_timeout_sec
        self.latest_gripper_opening_mm = float(opening_mm)
        self.gripper_busy = True
        self.last_gripper_command_time = time.monotonic()
        future = self.gripper_move_client.call_async(request)
        future.add_done_callback(lambda result: self._on_gripper_done(result, label))

    def _on_gripper_done(self, future, label: str):
        try:
            response = future.result()
            if response.success:
                self.latest_gripper_opening_mm = response.opening_mm
                self.get_logger().info(f"{label} accepted: {response.message}")
            else:
                self.get_logger().error(f"{label} rejected: {response.message}")
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"{label} service failed: {exc}")
        finally:
            self.gripper_busy = False

    def _start_rumble(self):
        if not self.enable_rumble:
            return
        self.rumble_until = time.monotonic() + max(0.0, self.rumble_duration_sec)
        self._publish_rumble(self.rumble_intensity)

    def _update_rumble(self):
        if self.rumble_until <= 0.0:
            return
        if time.monotonic() >= self.rumble_until:
            self.rumble_until = 0.0
            self._publish_rumble(0.0)

    def _publish_rumble(self, intensity: float):
        if not self.enable_rumble:
            return
        message = JoyFeedbackArray()
        for rumble_id in (0, 1):
            feedback = JoyFeedback()
            feedback.type = JoyFeedback.TYPE_RUMBLE
            feedback.id = rumble_id
            feedback.intensity = clamp(float(intensity), 0.0, 1.0)
            message.array.append(feedback)
        self.rumble_pub.publish(message)

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

    def _on_clear_error_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"clear error accepted: {response.message}")
            else:
                self.get_logger().error(f"clear error rejected: {response.message}")
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"clear error service failed: {exc}")

    def _button_edge(self, buttons, index: int) -> bool:
        return button_pressed(buttons, index) and not button_pressed(
            self.latest_buttons, index
        )


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
