import json
import time
from math import degrees

import rclpy
from dobot_interfaces.msg import DobotState, GripperStatus, TeleopAction
from dobot_interfaces.srv import (
    GripperCommand,
    GripperState,
    JogCommand,
    LimitRecovery,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy, JoyFeedback
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dobot_joy.joy_common import (
    JoyMapping,
    axis_to_jog,
    button_pressed,
    clamp,
    deadman_pressed,
    gripper_stop_target,
    jog_axis_to_action,
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
        self.toggle_enable_button_index = int(
            self.get_parameter("joy.toggle_enable_button_index").value
        )
        self.toggle_drag_button_index = int(
            self.get_parameter("joy.toggle_drag_button_index").value
        )
        self.clear_error_button_index = int(
            self.get_parameter("joy.clear_error_button_index").value
        )
        self.stop_button_indices = list(
            self.get_parameter("joy.stop_button_indices").value
        )
        self.start_button_index = int(
            self.get_parameter("joy.start_button_index").value
        )
        self.back_button_index = int(
            self.get_parameter("joy.back_button_index").value
        )
        self.data_reject_hold_sec = max(
            0.5,
            float(self.get_parameter("joy.data_reject_hold_sec").value),
        )
        self.limit_recovery_hold_sec = float(
            self.get_parameter("joy.limit_recovery_hold_sec").value
        )
        self.limit_recovery_release_timeout_sec = float(
            self.get_parameter("joy.limit_recovery_release_timeout_sec").value
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
        self.gripper_stop_lead_mm = float(
            self.get_parameter("joy.gripper_stop_lead_mm").value
        )
        self.enable_rumble = bool(self.get_parameter("joy.enable_rumble").value)
        self.rumble_topic = str(self.get_parameter("joy.rumble_topic").value)
        self.diagnostics_topic = str(
            self.get_parameter("joy.diagnostics_topic").value
        )
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
            rx_axis_index=int(self.get_parameter("joy.rx_axis_index").value),
            rx_axis_sign=float(self.get_parameter("joy.rx_axis_sign").value),
            ry_axis_index=int(self.get_parameter("joy.ry_axis_index").value),
            ry_axis_sign=float(self.get_parameter("joy.ry_axis_sign").value),
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
        self.gripper_stop_pending = False
        self.last_gripper_command_time = 0.0
        self.active_gripper_axis = None
        self.latest_gripper_gripped = False
        self.teleop_deadman = False
        self.gripper_action = "hold"
        self.gripper_target_mm = None
        self.drag_active = False
        self.rumble_until = 0.0
        self.start_back_chord_seen = False
        self.start_back_chord_started = 0.0
        self.back_pressed_at = 0.0
        self.limit_recovery_pending = False
        self.limit_recovery_released = False
        self.limit_recovery_released_at = 0.0
        self.jog_client = self.create_client(JogCommand, "/move_jog")
        self.estop_client = self.create_client(Trigger, "/emergency_stop")
        self.clear_error_client = self.create_client(Trigger, "/clear_error")
        self.enable_robot_client = self.create_client(Trigger, "/enable_robot")
        self.disable_robot_client = self.create_client(Trigger, "/disable_robot")
        self.drag_start_client = self.create_client(Trigger, "/drag_start")
        self.drag_stop_client = self.create_client(Trigger, "/drag_stop")
        self.gripper_state_client = self.create_client(GripperState, "/get_gripper_state")
        self.gripper_move_client = self.create_client(GripperCommand, "/gripper_move")
        self.data_start_client = self.create_client(Trigger, "/data_collection/start")
        self.data_stop_client = self.create_client(Trigger, "/data_collection/stop")
        self.data_reject_client = self.create_client(
            Trigger, "/data_collection/reject"
        )
        self.limit_recovery_client = self.create_client(
            LimitRecovery, "/limit_recovery"
        )
        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.create_subscription(DobotState, "/dobot_state", self._on_dobot_state, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(
            GripperStatus, "/gripper_state", self._on_gripper_status, 10
        )
        self.rumble_pub = self.create_publisher(JoyFeedback, self.rumble_topic, 10)
        self.diagnostics_pub = self.create_publisher(String, self.diagnostics_topic, 10)
        self.action_pub = self.create_publisher(
            TeleopAction, "/joy/teleop_action", 10
        )
        self.create_timer(0.1, self._watchdog)
        self.create_timer(0.05, self._publish_action)
        self.get_logger().info(
            "joy teleop ready: "
            f"topic={self.joy_topic}, deadman_button={self.deadman_button_index}, "
            f"coord_type={self.coord_type}, deadzone={self.mapping.deadzone}, "
            f"gripper_step={self.mapping.gripper_step_mm}mm"
        )

    def destroy_node(self):
        if rclpy.ok(context=self.context):
            self._stop_jog(wait=True, force=True)
            if self.limit_recovery_released:
                self._lock_limit_recovery()
            self._publish_rumble(0.0)
        super().destroy_node()

    def _declare_parameters(self):
        self.declare_parameter("joy.topic", "/joy")
        self.declare_parameter("joy.deadman_button_index", 4)
        self.declare_parameter("joy.estop_button_index", 1)
        self.declare_parameter("joy.toggle_gripper_button_index", 0)
        self.declare_parameter("joy.toggle_enable_button_index", 3)
        self.declare_parameter("joy.toggle_drag_button_index", 5)
        self.declare_parameter("joy.clear_error_button_index", 2)
        self.declare_parameter("joy.stop_button_indices", [6, 7])
        self.declare_parameter("joy.start_button_index", 7)
        self.declare_parameter("joy.back_button_index", 6)
        self.declare_parameter("joy.data_reject_hold_sec", 2.0)
        self.declare_parameter("joy.limit_recovery_hold_sec", 3.0)
        self.declare_parameter("joy.limit_recovery_release_timeout_sec", 10.0)
        self.declare_parameter("joy.x_axis_index", 1)
        self.declare_parameter("joy.x_axis_sign", -1.0)
        self.declare_parameter("joy.y_axis_index", 0)
        self.declare_parameter("joy.y_axis_sign", -1.0)
        self.declare_parameter("joy.z_axis_index", 4)
        self.declare_parameter("joy.z_axis_sign", 1.0)
        self.declare_parameter("joy.rz_axis_index", 3)
        self.declare_parameter("joy.rz_axis_sign", -1.0)
        self.declare_parameter("joy.rx_axis_index", 6)
        self.declare_parameter("joy.rx_axis_sign", -1.0)
        self.declare_parameter("joy.ry_axis_index", 7)
        self.declare_parameter("joy.ry_axis_sign", -1.0)
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
        self.declare_parameter("joy.gripper_stop_lead_mm", 3.0)
        self.declare_parameter("joy.enable_rumble", True)
        self.declare_parameter("joy.rumble_topic", "/joy/set_feedback")
        self.declare_parameter("joy.diagnostics_topic", "/joy/teleop_diagnostics")
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
        self.drag_active = msg.robot_mode == 6
        if not self._state_allows_jog(log=False):
            self._stop_jog()

    def _on_joint_state(self, msg: JointState):
        self.latest_joint_degrees = [degrees(position) for position in msg.position[:6]]
        if self.current_axis is not None and not self._joints_allow_jog(log=True):
            self._stop_jog()

    def _on_gripper_status(self, msg: GripperStatus):
        if msg.success:
            self.latest_gripper_opening_mm = msg.opening_mm
            if msg.initialized and not msg.moving:
                self._initialize_gripper_target(msg.opening_mm)
        gripped = bool(msg.object_detected or msg.grip_state == 2)
        if gripped and not self.latest_gripper_gripped:
            self._start_rumble()
        self.latest_gripper_gripped = gripped

    def _on_joy(self, msg: Joy):
        joy_time = time.monotonic()
        self.last_joy_time = joy_time
        self.teleop_deadman = deadman_pressed(msg.buttons, self.deadman_button_index)
        if self.trigger_neutral_axes is None:
            self.trigger_neutral_axes = list(msg.axes)
        if button_pressed(msg.buttons, self.estop_button_index):
            self._emergency_stop()
            if self.limit_recovery_released:
                self._lock_limit_recovery()
            self.latest_buttons = list(msg.buttons)
            return

        start_pressed = button_pressed(msg.buttons, self.start_button_index)
        back_pressed = button_pressed(msg.buttons, self.back_button_index)
        previous_start = button_pressed(self.latest_buttons, self.start_button_index)
        previous_back = button_pressed(self.latest_buttons, self.back_button_index)
        if back_pressed and not previous_back:
            self.back_pressed_at = joy_time
        if start_pressed and back_pressed:
            self._stop_jog()
            self.back_pressed_at = 0.0
            if not self.start_back_chord_seen:
                self.start_back_chord_seen = True
                self.start_back_chord_started = joy_time
            elif (
                not self.limit_recovery_pending
                and not self.limit_recovery_released
                and joy_time - self.start_back_chord_started
                >= self.limit_recovery_hold_sec
            ):
                self._prepare_limit_recovery()
            self.latest_buttons = list(msg.buttons)
            return
        if self.start_back_chord_seen:
            if self.limit_recovery_released:
                self._lock_limit_recovery()
            if not start_pressed and not back_pressed:
                self.start_back_chord_seen = False
                self.start_back_chord_started = 0.0
                self.back_pressed_at = 0.0
            self.latest_buttons = list(msg.buttons)
            return

        if previous_start and not start_pressed:
            self._call_data_collection(self.data_start_client, "start")
        if previous_back and not back_pressed:
            held_sec = (
                joy_time - self.back_pressed_at
                if self.back_pressed_at > 0.0
                else 0.0
            )
            if held_sec >= self.data_reject_hold_sec:
                self._call_data_collection(self.data_reject_client, "reject")
            else:
                self._call_data_collection(self.data_stop_client, "stop")
            self.back_pressed_at = 0.0
        if self._button_edge(msg.buttons, self.clear_error_button_index):
            self._clear_error()
            self.latest_buttons = list(msg.buttons)
            return
        if self._button_edge(msg.buttons, self.toggle_enable_button_index):
            self._toggle_enable()
            self.latest_buttons = list(msg.buttons)
            return
        if self._button_edge(msg.buttons, self.toggle_drag_button_index):
            self._toggle_drag()
            self.latest_buttons = list(msg.buttons)
            return
        if self._button_edge(msg.buttons, self.toggle_gripper_button_index):
            self._toggle_gripper()
        if any(button_pressed(msg.buttons, index) for index in self.stop_button_indices):
            self._stop_jog()
            self.latest_buttons = list(msg.buttons)
            return
        self._handle_gripper_axis(msg.axes)
        if not deadman_pressed(msg.buttons, self.deadman_button_index):
            self._stop_jog()
            self.latest_buttons = list(msg.buttons)
            return
        if not self._state_allows_jog(log=True):
            self._stop_jog()
            self.latest_buttons = list(msg.buttons)
            return
        if not self._joints_allow_jog(log=True):
            self._stop_jog()
            self.latest_buttons = list(msg.buttons)
            return

        axis = axis_to_jog(msg.axes, self.mapping)
        if axis is None:
            self._stop_jog()
            self.latest_buttons = list(msg.buttons)
            return
        if axis == self.current_axis:
            self.latest_buttons = list(msg.buttons)
            return
        if self.current_axis is not None:
            self._stop_jog(force=True)
        self._start_jog(axis, joy_time)
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

    def _start_jog(self, axis: str, joy_time: float = None):
        start_time = time.monotonic()
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
        call_time = time.monotonic()
        self._publish_diagnostic(
            {
                "event": "start_jog_request",
                "axis": axis,
                "joy_to_start_ms": _elapsed_ms(joy_time, start_time),
                "service_wait_ms": _elapsed_ms(start_time, call_time),
            }
        )
        future = self.jog_client.call_async(request)
        future.add_done_callback(
            lambda result: self._on_jog_done(result, axis, call_time)
        )

    def _stop_jog(self, wait: bool = False, force: bool = False):
        start_time = time.monotonic()
        previous_axis = self.current_axis
        if self.current_axis is None and not force and not wait:
            return
        if not self.jog_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/move_jog service is not available")
            self.current_axis = None
            return
        request = JogCommand.Request()
        request.stop = True
        self.current_axis = None
        call_time = time.monotonic()
        self._publish_diagnostic(
            {
                "event": "stop_jog_request",
                "previous_axis": previous_axis,
                "service_wait_ms": _elapsed_ms(start_time, call_time),
                "force": bool(force),
            }
        )
        future = self.jog_client.call_async(request)
        future.add_done_callback(
            lambda result: self._on_jog_stop_done(result, previous_axis, call_time)
        )
        if wait:
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

    def _watchdog(self):
        self._update_rumble()
        if (
            self.limit_recovery_released
            and time.monotonic() - self.limit_recovery_released_at
            >= self.limit_recovery_release_timeout_sec
        ):
            self.get_logger().error(
                "limit recovery release timeout; locking brake"
            )
            self._lock_limit_recovery()
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

    def _call_data_collection(self, client, action: str):
        self._stop_jog(force=True)
        if not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error(
                f"data collection {action} unavailable; recorder is not running"
            )
            self._start_rumble_pattern(0.2, 0.3)
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result: self._on_data_collection_done(result, action)
        )

    def _on_data_collection_done(self, future, action: str):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
                if action == "start":
                    self._start_rumble_pattern(0.35, 0.5)
                elif action == "reject":
                    self._start_rumble_pattern(0.5, 0.35)
                elif "pending review" in response.message:
                    self._start_rumble_pattern(0.3, 0.5)
                else:
                    self._start_rumble_pattern(0.8, 0.8)
            else:
                self.get_logger().error(response.message)
                self._start_rumble_pattern(0.2, 0.3)
            self._publish_diagnostic(
                {
                    "event": "data_collection_result",
                    "action": action,
                    "success": bool(response.success),
                    "message": response.message,
                }
            )
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"data collection {action} failed: {exc}")

    def _prepare_limit_recovery(self):
        self._stop_jog(force=True)
        if not self.limit_recovery_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/limit_recovery service is not available")
            return
        self.limit_recovery_pending = True
        request = LimitRecovery.Request()
        request.action = "prepare"
        future = self.limit_recovery_client.call_async(request)
        future.add_done_callback(self._on_limit_recovery_prepared)

    def _on_limit_recovery_prepared(self, future):
        self.limit_recovery_pending = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f"limit recovery rejected: {response.message}"
                )
                self._start_rumble_pattern(0.2, 0.3)
                return
            if not self._start_back_pressed():
                self.get_logger().warn(
                    "limit recovery cancelled before brake release"
                )
                self._lock_limit_recovery()
                return
            request = LimitRecovery.Request()
            request.action = "release"
            self.limit_recovery_pending = True
            future = self.limit_recovery_client.call_async(request)
            future.add_done_callback(self._on_limit_recovery_released)
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"limit recovery prepare failed: {exc}")

    def _on_limit_recovery_released(self, future):
        self.limit_recovery_pending = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f"limit recovery release failed: {response.message}"
                )
                self._start_rumble_pattern(0.2, 0.3)
                self._lock_limit_recovery()
                return
            self.limit_recovery_released = bool(response.brake_released)
            self.limit_recovery_released_at = time.monotonic()
            self.get_logger().warn(
                f"joint {response.joint} brake released; keep Back+Start held "
                "and move the joint by hand; release either button to lock"
            )
            self._start_rumble_pattern(0.8, 0.8)
            if not self._start_back_pressed():
                self._lock_limit_recovery()
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"limit recovery release failed: {exc}")

    def _lock_limit_recovery(self):
        if self.limit_recovery_pending:
            return
        if not self.limit_recovery_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error(
                "cannot lock recovered joint: /limit_recovery unavailable"
            )
            return
        self.limit_recovery_pending = True
        request = LimitRecovery.Request()
        request.action = "lock"
        future = self.limit_recovery_client.call_async(request)
        future.add_done_callback(self._on_limit_recovery_locked)

    def _on_limit_recovery_locked(self, future):
        self.limit_recovery_pending = False
        try:
            response = future.result()
            if response.success and not response.brake_released:
                self.limit_recovery_released = False
                self.get_logger().warn(response.message)
                self._start_rumble_pattern(0.25, 0.5)
            else:
                self.get_logger().error(
                    f"limit recovery lock failed: {response.message}"
                )
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"limit recovery lock failed: {exc}")

    def _start_back_pressed(self) -> bool:
        return button_pressed(
            self.latest_buttons, self.start_button_index
        ) and button_pressed(self.latest_buttons, self.back_button_index)

    def _clear_error(self):
        self._stop_jog(force=True)
        if not self.clear_error_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/clear_error service is not available")
            return
        future = self.clear_error_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_clear_error_done)

    def _toggle_enable(self):
        self._stop_jog(force=True)
        if self.latest_state is None:
            self.get_logger().warn("enable toggle rejected: no dobot_state received yet")
            return
        if self.latest_state.error_status or self.latest_state.robot_mode == 9:
            self.get_logger().warn("enable toggle rejected: robot is in error state")
            return
        if self.latest_state.enable_status == 1:
            self._call_trigger(
                self.disable_robot_client,
                "/disable_robot",
                "disable robot",
                self._on_disable_done,
            )
            return
        self._call_trigger(
            self.enable_robot_client,
            "/enable_robot",
            "enable robot",
            self._on_enable_done,
        )

    def _toggle_drag(self):
        self._stop_jog(force=True)
        if self.latest_state is None:
            self.get_logger().warn("drag toggle rejected: no dobot_state received yet")
            return
        if self.latest_state.error_status or self.latest_state.robot_mode == 9:
            self.get_logger().warn("drag toggle rejected: robot is in error state")
            return
        if self.drag_active or self.latest_state.robot_mode == 6:
            self._call_trigger(
                self.drag_stop_client,
                "/drag_stop",
                "drag stop",
                self._on_drag_stop_done,
            )
            return
        self._call_trigger(
            self.drag_start_client,
            "/drag_start",
            "drag start",
            self._on_drag_start_done,
        )

    def _call_trigger(self, client, service_name: str, label: str, callback):
        if not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error(f"{service_name} service is not available")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(callback)

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
                released_axis = self.active_gripper_axis
                self.active_gripper_axis = None
                self._stop_gripper_motion(released_axis)
            return

        if requested_axis == self.active_gripper_axis:
            return

        now = time.monotonic()
        if now - self.last_gripper_command_time < self.gripper_command_period_sec:
            return
        if self.active_gripper_axis is not None:
            self._stop_gripper_motion(self.active_gripper_axis)
        self.active_gripper_axis = requested_axis
        self.gripper_action = requested_axis
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
            self.gripper_action = "open"
        else:
            target = self.gripper_min_opening_mm
            self.gripper_action = "close"
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
            if response.initialized and not response.moving:
                self._initialize_gripper_target(response.opening_mm)
            self.gripper_busy = False
            if pending_delta is None:
                self._toggle_gripper()
                command_sent = self.gripper_busy
                return
            if isinstance(pending_delta, str) and pending_delta.startswith("stop:"):
                direction = pending_delta.split(":", 1)[1] or None
                target = gripper_stop_target(
                    response.opening_mm,
                    direction,
                    self.gripper_stop_lead_mm,
                    self.gripper_min_opening_mm,
                    self.gripper_max_opening_mm,
                )
                self._move_gripper(
                    target,
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

    def _stop_gripper_motion(self, direction: str = None):
        self.gripper_action = "hold"
        self.gripper_stop_pending = direction or "hold"
        if self.gripper_busy:
            return
        pending = self.gripper_stop_pending
        self.gripper_stop_pending = False
        self._request_gripper_state(f"stop:{pending}")

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
        self.gripper_target_mm = float(opening_mm)
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
            if self.gripper_stop_pending and self.active_gripper_axis is None:
                pending = self.gripper_stop_pending
                self.gripper_stop_pending = False
                self._request_gripper_state(f"stop:{pending}")

    def _start_rumble(self):
        if not self.enable_rumble:
            return
        self.rumble_until = time.monotonic() + max(0.0, self.rumble_duration_sec)
        self._publish_rumble(self.rumble_intensity)

    def _start_rumble_pattern(self, duration_sec: float, intensity: float):
        if not self.enable_rumble:
            return
        self.rumble_until = time.monotonic() + max(0.0, float(duration_sec))
        self._publish_rumble(intensity)

    def _update_rumble(self):
        if self.rumble_until <= 0.0:
            return
        if time.monotonic() >= self.rumble_until:
            self.rumble_until = 0.0
            self._publish_rumble(0.0)

    def _publish_rumble(self, intensity: float):
        if not self.enable_rumble:
            return
        message = JoyFeedback()
        message.type = JoyFeedback.TYPE_RUMBLE
        message.id = 0
        message.intensity = clamp(float(intensity), 0.0, 1.0)
        self.rumble_pub.publish(message)
        self._publish_diagnostic(
            {
                "event": "rumble",
                "intensity": message.intensity,
                "subscriber_count": self.rumble_pub.get_subscription_count(),
            }
        )
        if message.intensity > 0.0 and self.rumble_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                "rumble requested but /joy/set_feedback has no compatible subscriber"
            )

    def _publish_action(self):
        if self.gripper_target_mm is None:
            return
        message = TeleopAction()
        message.stamp = self.get_clock().now().to_msg()
        message.axis_id = str(self.current_axis or "")
        message.cartesian_jog = jog_axis_to_action(self.current_axis)
        message.motion_active = self.current_axis is not None
        message.deadman = bool(self.teleop_deadman)
        message.coord_type = int(self.coord_type)
        message.user = int(self.user)
        message.tool = int(self.tool)
        message.gripper_action = str(self.gripper_action)
        message.gripper_target_mm = float(self.gripper_target_mm)
        stroke = self.gripper_max_opening_mm - self.gripper_min_opening_mm
        message.gripper_target_normalized = (
            clamp(
                (self.gripper_target_mm - self.gripper_min_opening_mm) / stroke,
                0.0,
                1.0,
            )
            if stroke > 0.0
            else 0.0
        )
        self.action_pub.publish(message)

    def _initialize_gripper_target(self, opening_mm: float):
        if self.gripper_target_mm is not None:
            return
        self.gripper_target_mm = clamp(
            opening_mm,
            self.gripper_min_opening_mm,
            self.gripper_max_opening_mm,
        )
        self.get_logger().info(
            "gripper action initialized from feedback: "
            f"target={self.gripper_target_mm:.3f} mm"
        )

    def _publish_diagnostic(self, payload: dict):
        message = String()
        payload = dict(payload)
        payload["stamp_monotonic_sec"] = time.monotonic()
        message.data = json.dumps(payload, sort_keys=True)
        self.diagnostics_pub.publish(message)

    def _on_jog_done(self, future, axis: str, call_time: float):
        try:
            response = future.result()
            self._publish_diagnostic(
                {
                    "event": "start_jog_response",
                    "axis": axis,
                    "service_roundtrip_ms": _elapsed_ms(call_time, time.monotonic()),
                    "success": bool(response.success),
                    "error_id": int(response.error_id),
                }
            )
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

    def _on_jog_stop_done(self, future, previous_axis: str, call_time: float):
        try:
            response = future.result()
            self._publish_diagnostic(
                {
                    "event": "stop_jog_response",
                    "previous_axis": previous_axis,
                    "service_roundtrip_ms": _elapsed_ms(call_time, time.monotonic()),
                    "success": bool(response.success),
                    "error_id": int(response.error_id),
                }
            )
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"joy jog stop service failed: {exc}")

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

    def _on_enable_done(self, future):
        self._log_trigger_response(future, "enable robot")

    def _on_disable_done(self, future):
        self._log_trigger_response(future, "disable robot")

    def _on_drag_start_done(self, future):
        if self._log_trigger_response(future, "drag start"):
            self.drag_active = True

    def _on_drag_stop_done(self, future):
        if self._log_trigger_response(future, "drag stop"):
            self.drag_active = False

    def _log_trigger_response(self, future, label: str) -> bool:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{label} accepted: {response.message}")
                return True
            self.get_logger().error(f"{label} rejected: {response.message}")
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"{label} service failed: {exc}")
        return False

    def _button_edge(self, buttons, index: int) -> bool:
        return button_pressed(buttons, index) and not button_pressed(
            self.latest_buttons, index
        )


def _elapsed_ms(start: float, end: float) -> float:
    if start is None:
        return -1.0
    return round((float(end) - float(start)) * 1000.0, 3)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
