import math
from typing import Sequence

import rclpy
from dobot_interfaces.msg import DobotState, GripperStatus
from dobot_interfaces.srv import (
    GetJointState,
    GetRobotState,
    GetTcpPose,
    GripperCommand,
    GripperState,
    MoveCommand,
    TrajectoryCommand,
    TrajectoryList,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from .controller import (
    ROBOT_MODE_TEXT,
    ControllerConfig,
    DashboardResult,
    DobotController,
    FeedbackState,
    TeachResult,
)
from .gripper import DhAgGripper, DobotModbusAgGripper, GripperConfig, GripperResult


class DobotMotionServer(Node):
    """ROS2 service wrapper around Dobot TCP/IP motion commands."""

    def __init__(self):
        super().__init__("dobot_motion_server")
        config = self._load_config()
        self.joint_names = list(
            self.declare_parameter(
                "joint_names",
                ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            ).value
        )
        self.feedback_rate_hz = float(self.declare_parameter("feedback_rate_hz", 20.0).value)
        self._last_feedback_publish = 0.0

        self.controller = DobotController(
            config,
            feedback_callback=self._publish_feedback,
            log_callback=lambda message: self.get_logger().warning(message),
        )
        self.gripper = self._create_gripper(self._load_gripper_config())
        self.gripper_state_rate_hz = float(
            self.declare_parameter("gripper_state_rate_hz", 2.0).value
        )

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tcp_pub = self.create_publisher(Float64MultiArray, "tcp_pose", 10)
        self.dobot_state_pub = self.create_publisher(DobotState, "dobot_state", 10)
        self.gripper_state_pub = self.create_publisher(GripperStatus, "gripper_state", 10)
        self.gripper_state_timer = None
        if self.gripper_state_rate_hz > 0.0:
            self.gripper_state_timer = self.create_timer(
                1.0 / self.gripper_state_rate_hz,
                self._publish_gripper_state,
            )

        self.create_service(MoveCommand, "movej", self._movej)
        self.create_service(MoveCommand, "movel", self._movel)
        self.create_service(MoveCommand, "movep", self._movep)
        self.create_service(MoveCommand, "movejp", self._movejp)
        self.create_service(Trigger, "clear_error", self._clear_error)
        self.create_service(Trigger, "enable_robot", self._enable_robot)
        self.create_service(Trigger, "disable_robot", self._disable_robot)
        self.create_service(Trigger, "emergency_stop", self._emergency_stop)
        self.create_service(Trigger, "get_error_id", self._get_error_id)
        self.create_service(GetRobotState, "get_robot_state", self._get_robot_state)
        self.create_service(GetJointState, "get_joint_state", self._get_joint_state)
        self.create_service(GetTcpPose, "get_tcp_pose", self._get_tcp_pose)
        self.create_service(TrajectoryCommand, "teach_start", self._teach_start)
        self.create_service(TrajectoryCommand, "teach_stop", self._teach_stop)
        self.create_service(TrajectoryCommand, "teach_replay", self._teach_replay)
        self.create_service(TrajectoryCommand, "teach_delete", self._teach_delete)
        self.create_service(TrajectoryList, "teach_list", self._teach_list)
        self.create_service(Trigger, "teach_status", self._teach_status)
        self.create_service(Trigger, "gripper_init", self._gripper_init)
        self.create_service(GripperCommand, "gripper_move", self._gripper_move)
        self.create_service(GripperState, "get_gripper_state", self._get_gripper_state)

        if bool(self.declare_parameter("connect_on_start", True).value):
            try:
                self.controller.connect()
                self.get_logger().info("Connected to Dobot controller")
            except Exception as exc:
                self.get_logger().error(f"Initial Dobot connection failed: {exc}")

    def destroy_node(self):
        self.gripper.disconnect()
        self.controller.disconnect()
        super().destroy_node()

    def _load_config(self) -> ControllerConfig:
        return ControllerConfig(
            robot_ip=str(self.declare_parameter("robot_ip", "192.168.5.1").value),
            dashboard_port=int(self.declare_parameter("dashboard_port", 29999).value),
            move_port=int(self.declare_parameter("move_port", 30003).value),
            feedback_port=int(self.declare_parameter("feedback_port", 30004).value),
            default_user=int(self.declare_parameter("default_user", 0).value),
            default_tool=int(self.declare_parameter("default_tool", 0).value),
            default_speed_j=int(self.declare_parameter("default_speed_j", 0).value),
            default_acc_j=int(self.declare_parameter("default_acc_j", 0).value),
            default_speed_l=int(self.declare_parameter("default_speed_l", 0).value),
            default_acc_l=int(self.declare_parameter("default_acc_l", 0).value),
            robot_model=str(self.declare_parameter("robot_model", "Nova 2").value),
            rated_payload_kg=float(self.declare_parameter("rated_payload_kg", 2.0).value),
            workspace_radius_mm=float(self.declare_parameter("workspace_radius_mm", 625.0).value),
            max_tcp_speed_mps=float(self.declare_parameter("max_tcp_speed_mps", 1.6).value),
            repeatability_mm=float(self.declare_parameter("repeatability_mm", 0.05).value),
            joint_zero_deg=self._float_list_parameter("joint_zero_deg", [0.0] * 6),
            joint_lower_limits_deg=self._float_list_parameter(
                "joint_lower_limits_deg",
                [-360.0, -180.0, -156.0, -360.0, -360.0, -360.0],
            ),
            joint_upper_limits_deg=self._float_list_parameter(
                "joint_upper_limits_deg",
                [360.0, 180.0, 156.0, 360.0, 360.0, 360.0],
            ),
            max_joint_speed_deg_s=self._float_list_parameter(
                "max_joint_speed_deg_s",
                [135.0] * 6,
            ),
            joint_limit_check=bool(self.declare_parameter("joint_limit_check", True).value),
            joint_limit_margin_deg=float(
                self.declare_parameter("joint_limit_margin_deg", 0.0).value
            ),
            command_timeout_sec=float(self.declare_parameter("command_timeout_sec", 3.0).value),
            motion_timeout_sec=float(self.declare_parameter("motion_timeout_sec", 30.0).value),
            wait_for_motion=bool(self.declare_parameter("wait_for_motion", False).value),
            motion_status_check=bool(
                self.declare_parameter("motion_status_check", True).value
            ),
            post_motion_check=bool(
                self.declare_parameter("post_motion_check", True).value
            ),
            post_motion_check_timeout_sec=float(
                self.declare_parameter("post_motion_check_timeout_sec", 2.0).value
            ),
            joint_arrival_tolerance_deg=float(
                self.declare_parameter("joint_arrival_tolerance_deg", 0.5).value
            ),
            tcp_position_tolerance_mm=float(
                self.declare_parameter("tcp_position_tolerance_mm", 1.0).value
            ),
            tcp_rotation_tolerance_deg=float(
                self.declare_parameter("tcp_rotation_tolerance_deg", 1.0).value
            ),
            ik_check=bool(self.declare_parameter("ik_check", True).value),
            ik_use_joint_near=bool(self.declare_parameter("ik_use_joint_near", True).value),
            enable_on_start=bool(self.declare_parameter("enable_on_start", False).value),
            teach_trajectory_dir=str(
                self.declare_parameter("teach_trajectory_dir", "/home/ros/ws/trajectories").value
            ),
            teach_sample_rate_hz=float(
                self.declare_parameter("teach_sample_rate_hz", 5.0).value
            ),
            teach_min_joint_delta_deg=float(
                self.declare_parameter("teach_min_joint_delta_deg", 0.5).value
            ),
            teach_min_tcp_delta_mm=float(
                self.declare_parameter("teach_min_tcp_delta_mm", 1.0).value
            ),
            teach_replay_speed=int(self.declare_parameter("teach_replay_speed", 10).value),
            teach_replay_acc=int(self.declare_parameter("teach_replay_acc", 10).value),
            teach_replay_wait=bool(self.declare_parameter("teach_replay_wait", True).value),
            teach_replay_timeout_sec=float(
                self.declare_parameter("teach_replay_timeout_sec", 20.0).value
            ),
            teach_replay_mode=str(
                self.declare_parameter("teach_replay_mode", "movej").value
            ),
            teach_servoj_rate_hz=float(
                self.declare_parameter("teach_servoj_rate_hz", 33.0).value
            ),
            teach_servoj_t=float(self.declare_parameter("teach_servoj_t", 0.1).value),
            teach_servoj_lookahead_time=float(
                self.declare_parameter("teach_servoj_lookahead_time", 50.0).value
            ),
            teach_servoj_gain=float(
                self.declare_parameter("teach_servoj_gain", 500.0).value
            ),
        )

    def _load_gripper_config(self) -> GripperConfig:
        return GripperConfig(
            enabled=bool(self.declare_parameter("gripper_enabled", False).value),
            transport=str(self.declare_parameter("gripper_transport", "dobot_modbus").value),
            port=str(self.declare_parameter("gripper_port", "/dev/ttyUSB0").value),
            baudrate=int(self.declare_parameter("gripper_baudrate", 115200).value),
            slave_id=int(self.declare_parameter("gripper_slave_id", 1).value),
            modbus_ip=str(self.declare_parameter("gripper_modbus_ip", "127.0.0.1").value),
            modbus_port=int(self.declare_parameter("gripper_modbus_port", 60000).value),
            modbus_index=int(self.declare_parameter("gripper_modbus_index", -1).value),
            timeout_sec=float(self.declare_parameter("gripper_timeout_sec", 0.2).value),
            stroke_mm=float(self.declare_parameter("gripper_stroke_mm", 95.0).value),
            max_force_n=float(self.declare_parameter("gripper_max_force_n", 160.0).value),
            default_force_percent=int(
                self.declare_parameter("gripper_default_force_percent", 50).value
            ),
            min_force_percent=int(self.declare_parameter("gripper_min_force_percent", 20).value),
            max_force_percent=int(self.declare_parameter("gripper_max_force_percent", 100).value),
            auto_connect=bool(self.declare_parameter("gripper_auto_connect", True).value),
        )

    def _create_gripper(self, config: GripperConfig):
        if config.transport == "local_serial":
            return DhAgGripper(config)
        return DobotModbusAgGripper(config, self.controller.dashboard_command)

    def _movej(self, request, response):
        return self._handle_move("movej", request, response)

    def _movel(self, request, response):
        return self._handle_move("movel", request, response)

    def _movep(self, request, response):
        return self._handle_move("movep", request, response)

    def _movejp(self, request, response):
        return self._handle_move("movejp", request, response)

    def _clear_error(self, request, response):
        del request
        return self._handle_dashboard("clear_error", self.controller.clear_error(), response)

    def _enable_robot(self, request, response):
        del request
        return self._handle_dashboard("enable_robot", self.controller.enable_robot(), response)

    def _disable_robot(self, request, response):
        del request
        return self._handle_dashboard("disable_robot", self.controller.disable_robot(), response)

    def _emergency_stop(self, request, response):
        del request
        return self._handle_dashboard("emergency_stop", self.controller.emergency_stop(), response)

    def _get_error_id(self, request, response):
        del request
        return self._handle_dashboard("get_error_id", self.controller.get_error_id(), response)

    def _get_robot_state(
        self,
        request,
        response: GetRobotState.Response,
    ):
        del request
        state = self.controller.latest_state()
        response.connected = self.controller.is_connected()
        response.feedback_valid = state.stamp > 0.0
        response.stamp_sec = float(state.stamp)
        response.speed_scaling = float(state.speed_scaling)
        response.enable_status = int(state.enable_status)
        response.running_status = int(state.running_status)
        response.error_status = int(state.error_status)

        if response.feedback_valid:
            response.success = True
            response.robot_mode = int(state.robot_mode)
            response.robot_mode_text = ROBOT_MODE_TEXT.get(state.robot_mode, "")
            response.message = self._robot_state_message(response.robot_mode)
            return response

        mode_result = self.controller.robot_mode()
        response.success = mode_result.success
        response.robot_mode = int(mode_result.value)
        response.robot_mode_text = ROBOT_MODE_TEXT.get(mode_result.value, "")
        response.enable_status = -1
        response.running_status = -1
        response.error_status = -1
        if mode_result.success:
            response.message = f"feedback not received yet; {mode_result.message}"
        else:
            response.message = f"feedback not received yet; {mode_result.message}"
        return response

    def _get_joint_state(
        self,
        request,
        response: GetJointState.Response,
    ):
        del request
        state = self.controller.latest_state()
        joints_deg = self._six_values(state.joints)
        response.joints_deg = joints_deg
        response.joints_rad = [math.radians(value) for value in joints_deg]
        response.stamp_sec = float(state.stamp)
        response.success = state.stamp > 0.0 and len(state.joints) >= 6
        response.message = "joint state from feedback" if response.success else "feedback not received yet"
        return response

    def _get_tcp_pose(
        self,
        request,
        response: GetTcpPose.Response,
    ):
        del request
        state = self.controller.latest_state()
        response.pose = self._six_values(state.tcp_pose)
        response.stamp_sec = float(state.stamp)
        response.success = state.stamp > 0.0 and len(state.tcp_pose) >= 6
        response.message = (
            "tcp pose from feedback: [x,y,z,rx,ry,rz] in mm/deg"
            if response.success
            else "feedback not received yet"
        )
        return response

    def _teach_start(self, request: TrajectoryCommand.Request, response):
        result = self.controller.teach_start(str(request.name), bool(request.overwrite))
        return self._handle_teach("teach_start", result, response)

    def _teach_stop(self, request: TrajectoryCommand.Request, response):
        result = self.controller.teach_stop(str(request.name))
        return self._handle_teach("teach_stop", result, response)

    def _teach_replay(self, request: TrajectoryCommand.Request, response):
        wait = bool(request.wait) if bool(request.override_wait) else None
        result = self.controller.teach_replay(
            str(request.name),
            speed=int(request.speed),
            acceleration=int(request.acceleration),
            replay_mode=str(request.replay_mode),
            wait=wait,
            timeout_sec=float(request.timeout_sec),
        )
        return self._handle_teach("teach_replay", result, response)

    def _teach_delete(self, request: TrajectoryCommand.Request, response):
        result = self.controller.teach_delete(str(request.name))
        return self._handle_teach("teach_delete", result, response)

    def _teach_list(self, request, response: TrajectoryList.Response):
        del request
        results = self.controller.teach_list()
        response.success = True
        response.message = f"{len(results)} trajectories"
        response.names = [result.trajectory_name for result in results]
        response.paths = [result.path for result in results]
        response.point_counts = [int(result.point_count) for result in results]
        return response

    def _teach_status(self, request, response: Trigger.Response):
        del request
        result = self.controller.teach_status()
        response.success = result.success
        response.message = self._teach_message(result)
        return response

    def _gripper_init(self, request, response: Trigger.Response):
        del request
        result = self.gripper.initialize()
        response.success = result.success
        response.message = result.message
        if result.success:
            self.get_logger().info("gripper_init accepted")
        else:
            self.get_logger().warning(f"gripper_init rejected: {result.message}")
        return response

    def _gripper_move(self, request: GripperCommand.Request, response: GripperCommand.Response):
        result = self.gripper.move(
            float(request.opening_mm),
            int(request.position_permille),
            int(request.force_percent),
            float(request.force_n),
            bool(request.wait),
            float(request.timeout_sec),
        )
        self._fill_gripper_command_response(result, response)
        if result.success:
            self.get_logger().info("gripper_move accepted")
        else:
            self.get_logger().warning(f"gripper_move rejected: {result.message}")
        return response

    def _get_gripper_state(self, request, response: GripperState.Response):
        del request
        result = self.gripper.state()
        response.success = result.success
        response.message = result.message
        response.enabled = bool(self.gripper.config.enabled)
        response.connected = bool(result.connected)
        response.init_state = int(result.init_state)
        response.grip_state = int(result.grip_state)
        response.position_permille = int(result.position_permille)
        response.opening_mm = float(result.opening_mm)
        response.force_percent = int(result.force_percent)
        response.initialized = result.initialized
        response.moving = result.moving
        response.object_detected = result.object_detected
        response.object_dropped = result.object_dropped
        return response

    def _handle_move(self, kind: str, request: MoveCommand.Request, response: MoveCommand.Response):
        """Handle one service call while keeping Dobot units in the API."""

        result = self.controller.move(
            kind,
            list(request.target),
            user=int(request.user),
            tool=int(request.tool),
            speed=int(request.speed),
            acceleration=int(request.acceleration),
            wait=bool(request.wait),
            timeout_sec=float(request.timeout_sec),
        )
        response.success = result.success
        response.error_id = int(result.error_id)
        response.message = result.message
        response.raw_reply = result.raw_reply
        response.ik_reply = result.ik_reply
        response.ik_joints = self._six_values(result.ik_joints)
        if result.success:
            self.get_logger().info(f"{kind} accepted")
        else:
            self.get_logger().warning(f"{kind} rejected: {result.message}")
        return response

    def _fill_gripper_command_response(
        self,
        result: GripperResult,
        response: GripperCommand.Response,
    ) -> None:
        response.success = result.success
        response.message = result.message
        response.init_state = int(result.init_state)
        response.grip_state = int(result.grip_state)
        response.position_permille = int(result.position_permille)
        response.opening_mm = float(result.opening_mm)
        response.force_percent = int(result.force_percent)
        response.object_detected = result.object_detected
        response.object_dropped = result.object_dropped

    def _handle_dashboard(
        self,
        name: str,
        result: DashboardResult,
        response: Trigger.Response,
    ):
        response.success = result.success
        fields = [result.message]
        if result.error_id != 0:
            fields.append(f"error_id={result.error_id}")
        if result.values:
            fields.append("values=" + ",".join(str(value) for value in result.values))
        if result.raw_reply:
            fields.append(f"raw_reply={result.raw_reply}")
        response.message = "; ".join(field for field in fields if field)
        if result.success:
            self.get_logger().info(f"{name} accepted")
        else:
            self.get_logger().warning(f"{name} rejected: {response.message}")
        return response

    def _handle_teach(
        self,
        name: str,
        result: TeachResult,
        response: TrajectoryCommand.Response,
    ):
        response.success = result.success
        response.error_id = int(result.error_id)
        response.message = self._teach_message(result)
        response.trajectory_name = result.trajectory_name
        response.path = result.path
        response.point_count = int(result.point_count)
        response.raw_reply = result.raw_reply
        if result.success:
            self.get_logger().info(f"{name} accepted")
        else:
            self.get_logger().warning(f"{name} rejected: {response.message}")
        return response

    def _teach_message(self, result: TeachResult) -> str:
        fields = [result.message]
        if result.trajectory_name:
            fields.append(f"name={result.trajectory_name}")
        if result.point_count:
            fields.append(f"points={result.point_count}")
        if result.path:
            fields.append(f"path={result.path}")
        if result.raw_reply:
            fields.append(f"raw_reply={result.raw_reply}")
        return "; ".join(field for field in fields if field)

    def _six_values(self, values: Sequence[float]):
        result = list(values[:6])
        while len(result) < 6:
            result.append(0.0)
        return result

    def _float_list_parameter(self, name: str, default: Sequence[float]):
        return [
            float(value)
            for value in self.declare_parameter(name, list(default)).value
        ]

    def _robot_state_message(self, robot_mode: int) -> str:
        mode_text = ROBOT_MODE_TEXT.get(robot_mode, "")
        return f"robot_mode={robot_mode} {mode_text}".strip()

    def _publish_feedback(self, state: FeedbackState) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        min_period = 1.0 / self.feedback_rate_hz if self.feedback_rate_hz > 0.0 else 0.0
        if min_period and now_sec - self._last_feedback_publish < min_period:
            return
        self._last_feedback_publish = now_sec

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        # Dobot feedback reports joints in degrees; robot_state_publisher and
        # RViz expect JointState positions in radians.
        joint_msg.position = [
            math.radians(value) for value in state.joints[: len(self.joint_names)]
        ]
        self.joint_pub.publish(joint_msg)

        tcp_msg = Float64MultiArray()
        tcp_msg.data = state.tcp_pose
        self.tcp_pub.publish(tcp_msg)

        dobot_state = DobotState()
        dobot_state.connected = self.controller.is_connected()
        dobot_state.feedback_valid = state.stamp > 0.0
        dobot_state.stamp_sec = float(state.stamp)
        dobot_state.robot_mode = int(state.robot_mode)
        dobot_state.robot_mode_text = ROBOT_MODE_TEXT.get(state.robot_mode, "")
        dobot_state.speed_scaling = float(state.speed_scaling)
        dobot_state.enable_status = int(state.enable_status)
        dobot_state.running_status = int(state.running_status)
        dobot_state.error_status = int(state.error_status)
        dobot_state.drag_status = int(state.drag_status)
        dobot_state.record_button_signal = int(state.record_button_signal)
        dobot_state.q_target = self._six_values(state.q_target)
        dobot_state.tcp_target = self._six_values(state.tcp_target)
        self.dobot_state_pub.publish(dobot_state)

    def _publish_gripper_state(self) -> None:
        self.gripper_state_pub.publish(self._gripper_status_message(self.gripper.state()))

    def _gripper_status_message(self, result: GripperResult) -> GripperStatus:
        msg = GripperStatus()
        msg.enabled = bool(self.gripper.config.enabled)
        msg.connected = bool(result.connected)
        msg.success = bool(result.success)
        msg.message = result.message
        msg.init_state = int(result.init_state)
        msg.grip_state = int(result.grip_state)
        msg.position_permille = int(result.position_permille)
        msg.opening_mm = float(result.opening_mm)
        msg.force_percent = int(result.force_percent)
        msg.initialized = result.initialized
        msg.moving = result.moving
        msg.object_detected = result.object_detected
        msg.object_dropped = result.object_dropped
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DobotMotionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
