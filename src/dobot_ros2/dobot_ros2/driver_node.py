import json
import math
from typing import Sequence

import rclpy
from dobot_interfaces.srv import GetJointState, GetRobotState, GetTcpPose, MoveCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from .controller import (
    ROBOT_MODE_TEXT,
    ControllerConfig,
    DashboardResult,
    DobotController,
    FeedbackState,
)


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

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tcp_pub = self.create_publisher(Float64MultiArray, "tcp_pose", 10)
        self.status_pub = self.create_publisher(String, "status", 10)

        self.create_service(MoveCommand, "movej", self._movej)
        self.create_service(MoveCommand, "movel", self._movel)
        self.create_service(MoveCommand, "movep", self._movep)
        self.create_service(MoveCommand, "movejp", self._movejp)
        self.create_service(Trigger, "clear_error", self._clear_error)
        self.create_service(Trigger, "enable_robot", self._enable_robot)
        self.create_service(Trigger, "disable_robot", self._disable_robot)
        self.create_service(Trigger, "get_error_id", self._get_error_id)
        self.create_service(GetRobotState, "get_robot_state", self._get_robot_state)
        self.create_service(GetJointState, "get_joint_state", self._get_joint_state)
        self.create_service(GetTcpPose, "get_tcp_pose", self._get_tcp_pose)

        if bool(self.declare_parameter("connect_on_start", True).value):
            try:
                self.controller.connect()
                self.get_logger().info("Connected to Dobot controller")
            except Exception as exc:
                self.get_logger().error(f"Initial Dobot connection failed: {exc}")

    def destroy_node(self):
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
        )

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

    def _six_values(self, values: Sequence[float]):
        result = list(values[:6])
        while len(result) < 6:
            result.append(0.0)
        return result

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

        status_msg = String()
        status_msg.data = json.dumps(
            {
                "robot_mode": state.robot_mode,
                "speed_scaling": state.speed_scaling,
                "enable_status": state.enable_status,
                "running_status": state.running_status,
                "error_status": state.error_status,
                "q_target": state.q_target,
                "tcp_target": state.tcp_target,
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(status_msg)


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
