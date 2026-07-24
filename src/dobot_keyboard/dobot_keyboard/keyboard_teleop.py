import rclpy
from dobot_interfaces.msg import DobotState
from dobot_interfaces.srv import GetTcpPose, GripperCommand, GripperState, MoveCommand
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dobot_keyboard.keyboard_common import (
    ESTOP_KEY,
    KeyboardSafetyConfig,
    RESET_SIM_KEY,
    TOGGLE_GRIPPER_KEY,
    apply_delta,
    decide_gripper_opening,
    key_to_delta,
    normalize_key,
    robot_state_allows_motion,
    target_within_limits,
)


def _service_name(value: str) -> str:
    name = str(value).strip()
    if not name:
        return "/movep"
    if not name.startswith("/"):
        return f"/{name}"
    return name


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("dobot_keyboard_teleop")
        self._declare_parameters()
        self.input_topic = self.get_parameter("keyboard.input_topic").value
        self.translation_step_mm = float(
            self.get_parameter("keyboard.translation_step_mm").value
        )
        self.rotation_step_deg = float(
            self.get_parameter("keyboard.rotation_step_deg").value
        )
        self.motion_service_name = _service_name(
            self.get_parameter("keyboard.motion_service").value
        )
        self.user = int(self.get_parameter("keyboard.user").value)
        self.tool = int(self.get_parameter("keyboard.tool").value)
        self.speed = int(self.get_parameter("keyboard.speed").value)
        self.acceleration = int(self.get_parameter("keyboard.acceleration").value)
        self.wait = bool(self.get_parameter("keyboard.wait").value)
        self.timeout_sec = float(self.get_parameter("keyboard.timeout_sec").value)
        self.gripper_opening_open_mm = float(
            self.get_parameter("keyboard.gripper_opening_open_mm").value
        )
        self.gripper_opening_close_mm = float(
            self.get_parameter("keyboard.gripper_opening_close_mm").value
        )
        self.gripper_toggle_threshold_mm = float(
            self.get_parameter("keyboard.gripper_toggle_threshold_mm").value
        )
        self.gripper_force_percent = int(
            self.get_parameter("keyboard.gripper_force_percent").value
        )
        self.safety = KeyboardSafetyConfig(
            min_pose=list(self.get_parameter("keyboard.workspace_min").value),
            max_pose=list(self.get_parameter("keyboard.workspace_max").value),
            max_xy_radius_mm=float(
                self.get_parameter("keyboard.workspace_max_xy_radius_mm").value
            ),
        )
        if self.safety.max_xy_radius_mm <= 0.0:
            self.safety = KeyboardSafetyConfig(
                min_pose=self.safety.min_pose,
                max_pose=self.safety.max_pose,
                max_xy_radius_mm=None,
            )

        self.busy = False
        self.latest_state = None
        self.subscription = self.create_subscription(
            String, self.input_topic, self._on_key, 10
        )
        self.state_subscription = self.create_subscription(
            DobotState, "/dobot_state", self._on_dobot_state, 10
        )
        self.tcp_client = self.create_client(GetTcpPose, "/get_tcp_pose")
        self.motion_client = self.create_client(MoveCommand, self.motion_service_name)
        self.gripper_state_client = self.create_client(GripperState, "/get_gripper_state")
        self.gripper_move_client = self.create_client(GripperCommand, "/gripper_move")
        self.estop_client = self.create_client(Trigger, "/emergency_stop")
        self.get_logger().info(
            "keyboard teleop ready: "
            f"topic={self.input_topic}, motion_service={self.motion_service_name}, "
            f"step={self.translation_step_mm}mm, rot_step={self.rotation_step_deg}deg"
        )

    def _declare_parameters(self):
        self.declare_parameter("keyboard.input_topic", "/keyboard/input")
        self.declare_parameter("keyboard.translation_step_mm", 5.0)
        self.declare_parameter("keyboard.rotation_step_deg", 2.0)
        self.declare_parameter("keyboard.motion_service", "movep")
        self.declare_parameter("keyboard.user", 0)
        self.declare_parameter("keyboard.tool", 0)
        self.declare_parameter("keyboard.speed", 2)
        self.declare_parameter("keyboard.acceleration", 2)
        self.declare_parameter("keyboard.wait", True)
        self.declare_parameter("keyboard.timeout_sec", 20.0)
        self.declare_parameter(
            "keyboard.workspace_min", [-625.0, -625.0, 20.0, -360.0, -360.0, -360.0]
        )
        self.declare_parameter(
            "keyboard.workspace_max", [625.0, 625.0, 625.0, 360.0, 360.0, 360.0]
        )
        self.declare_parameter("keyboard.workspace_max_xy_radius_mm", 625.0)
        self.declare_parameter("keyboard.gripper_opening_open_mm", 95.0)
        self.declare_parameter("keyboard.gripper_opening_close_mm", 0.0)
        self.declare_parameter("keyboard.gripper_toggle_threshold_mm", 45.0)
        self.declare_parameter("keyboard.gripper_force_percent", 50)

    def _on_key(self, msg: String):
        key = normalize_key(msg.data)
        if key == ESTOP_KEY:
            self._emergency_stop()
            return
        if key == RESET_SIM_KEY:
            self.get_logger().warn("reset simulation is not supported on the real Dobot")
            return
        if key == TOGGLE_GRIPPER_KEY:
            self._toggle_gripper()
            return

        delta = key_to_delta(key, self.translation_step_mm, self.rotation_step_deg)
        if delta is None:
            return
        self._request_motion(delta)

    def _on_dobot_state(self, msg: DobotState):
        self.latest_state = msg

    def _state_allows_motion(self) -> bool:
        if self.latest_state is None:
            self.get_logger().warn("keyboard motion rejected: no dobot_state received yet")
            return False
        ok, reason = robot_state_allows_motion(
            self.latest_state.connected,
            self.latest_state.feedback_valid,
            self.latest_state.robot_mode,
            self.latest_state.enable_status,
            self.latest_state.error_status,
        )
        if not ok:
            self.get_logger().warn(f"keyboard motion rejected: {reason}")
        return ok

    def _emergency_stop(self):
        if not self.estop_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/emergency_stop service is not available")
            return
        future = self.estop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_estop_done)

    def _on_estop_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().error(f"emergency stop accepted: {response.message}")
            else:
                self.get_logger().error(f"emergency stop rejected: {response.message}")
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"emergency stop service failed: {exc}")

    def _try_start_command(self) -> bool:
        if self.busy:
            self.get_logger().warn("previous keyboard command is still running; key ignored")
            return False
        self.busy = True
        return True

    def _finish_command(self):
        self.busy = False

    def _request_motion(self, delta):
        if not self._state_allows_motion():
            return
        if not self._try_start_command():
            return
        if not self.tcp_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/get_tcp_pose service is not available")
            self._finish_command()
            return
        future = self.tcp_client.call_async(GetTcpPose.Request())
        future.add_done_callback(lambda result: self._on_tcp_pose(result, delta))

    def _on_tcp_pose(self, future, delta):
        motion_sent = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"get tcp pose failed: {response.message}")
                return
            target = apply_delta(response.pose, delta)
            ok, reason = target_within_limits(target, self.safety)
            if not ok:
                self.get_logger().warn(f"keyboard target rejected: {reason}")
                return
            if not self.motion_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().error(f"{self.motion_service_name} service is not available")
                return
            request = MoveCommand.Request()
            request.target = target
            request.user = self.user
            request.tool = self.tool
            request.speed = self.speed
            request.acceleration = self.acceleration
            request.wait = self.wait
            request.timeout_sec = self.timeout_sec
            motion_future = self.motion_client.call_async(request)
            motion_future.add_done_callback(self._on_motion_done)
            motion_sent = True
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"keyboard motion failed: {exc}")
        finally:
            if not motion_sent:
                self._finish_command()

    def _on_motion_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"keyboard motion accepted: {response.message}")
            else:
                self.get_logger().error(
                    f"keyboard motion rejected: error_id={response.error_id}; {response.message}"
                )
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"keyboard motion service failed: {exc}")
        finally:
            self._finish_command()

    def _toggle_gripper(self):
        if not self._try_start_command():
            return
        if not self.gripper_state_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("/get_gripper_state service is not available")
            self._finish_command()
            return
        future = self.gripper_state_client.call_async(GripperState.Request())
        future.add_done_callback(self._on_gripper_state)

    def _on_gripper_state(self, future):
        gripper_command_sent = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"get gripper state failed: {response.message}")
                return
            target = decide_gripper_opening(
                response.opening_mm,
                self.gripper_toggle_threshold_mm,
                self.gripper_opening_open_mm,
                self.gripper_opening_close_mm,
            )
            if not self.gripper_move_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().error("/gripper_move service is not available")
                return
            request = GripperCommand.Request()
            request.opening_mm = target
            request.position_permille = 0
            request.force_percent = self.gripper_force_percent
            request.force_n = -1.0
            request.wait = self.wait
            request.timeout_sec = self.timeout_sec
            move_future = self.gripper_move_client.call_async(request)
            move_future.add_done_callback(self._on_gripper_done)
            gripper_command_sent = True
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"gripper toggle failed: {exc}")
        finally:
            if not gripper_command_sent:
                self._finish_command()

    def _on_gripper_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"gripper toggled: {response.message}")
            else:
                self.get_logger().error(f"gripper toggle rejected: {response.message}")
        except Exception as exc:  # pragma: no cover - ROS callback safety
            self.get_logger().error(f"gripper service failed: {exc}")
        finally:
            self._finish_command()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
