import json
import math
import threading
import time
from typing import Sequence

import rclpy
from dobot_interfaces.msg import CartesianServoCommand, DobotState, GripperStatus
from dobot_interfaces.srv import (
    GetJointState,
    GetRobotState,
    GetTcpPose,
    GripperCommand,
    GripperState,
    JogCommand,
    LimitRecovery,
    MoveCommand,
    TrajectoryCommand,
    TrajectoryList,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from .controller import (
    ROBOT_MODE_TEXT,
    ControllerConfig,
    DashboardResult,
    DobotController,
    FeedbackState,
    ServoStreamBusy,
    TeachResult,
)
from .cartesian_servo import (
    NON_REVERSING_HOLD_REASONS,
    clamp_normalized_vector,
    heartbeat_expired,
    integrate_pose,
    joints_within_margin,
    pose_within_workspace,
    select_hold_pose,
    slew_vector,
)
from .gripper import DhAgGripper, DobotModbusAgGripper, GripperConfig, GripperResult
from .recover_limit import LimitRecoveryManager


class DobotMotionServer(Node):
    """ROS2 service wrapper around Dobot TCP/IP motion commands."""

    def __init__(self):
        super().__init__("dobot_motion_server")
        config = self._load_config()
        self.config = config
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
            log_callback=lambda message: (
                self.get_logger().warning(message) if self.context.ok() else None
            ),
        )
        self.limit_recovery = LimitRecoveryManager(
            self.controller,
            release_timeout_sec=float(
                self.declare_parameter("limit_recovery_timeout_sec", 12.0).value
            ),
        )
        self.gripper = self._create_gripper(self._load_gripper_config())
        self.gripper_command_callback_group = MutuallyExclusiveCallbackGroup()
        self.gripper_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.gripper_state_rate_hz = float(
            self.declare_parameter("gripper_state_rate_hz", 2.0).value
        )
        self.move_jog_watchdog_sec = float(
            self.declare_parameter("move_jog.watchdog_sec", 0.0).value
        )
        self._move_jog_watchdog_lock = threading.Lock()
        self._move_jog_active = False
        self._move_jog_last_command = 0.0

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tcp_pub = self.create_publisher(Float64MultiArray, "tcp_pose", 10)
        self.dobot_state_pub = self.create_publisher(DobotState, "dobot_state", 10)
        self.gripper_state_pub = self.create_publisher(GripperStatus, "gripper_state", 10)
        self.move_jog_diagnostics_pub = self.create_publisher(
            String, "move_jog/diagnostics", 10
        )
        self._configure_cartesian_servo()
        self.gripper_state_timer = None
        if self.gripper_state_rate_hz > 0.0:
            self.gripper_state_timer = self.create_timer(
                1.0 / self.gripper_state_rate_hz,
                self._publish_gripper_state,
                callback_group=self.gripper_state_callback_group,
            )

        self.create_service(MoveCommand, "movej", self._movej)
        self.create_service(MoveCommand, "movel", self._movel)
        self.create_service(MoveCommand, "movep", self._movep)
        self.create_service(MoveCommand, "movejp", self._movejp)
        self.create_service(Trigger, "clear_error", self._clear_error)
        self.create_service(Trigger, "enable_robot", self._enable_robot)
        self.create_service(Trigger, "disable_robot", self._disable_robot)
        self.create_service(Trigger, "emergency_stop", self._emergency_stop)
        self.create_service(Trigger, "drag_start", self._drag_start)
        self.create_service(Trigger, "drag_stop", self._drag_stop)
        self.create_service(JogCommand, "move_jog", self._move_jog)
        self.create_service(LimitRecovery, "limit_recovery", self._limit_recovery)
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
        self.create_service(
            Trigger,
            "gripper_init",
            self._gripper_init,
            callback_group=self.gripper_command_callback_group,
        )
        self.create_service(
            GripperCommand,
            "gripper_move",
            self._gripper_move,
            callback_group=self.gripper_command_callback_group,
        )
        self.create_service(
            GripperState,
            "get_gripper_state",
            self._get_gripper_state,
            callback_group=self.gripper_state_callback_group,
        )
        self.create_timer(0.1, self._limit_recovery_watchdog)
        if self.move_jog_watchdog_sec > 0.0:
            self.create_timer(0.05, self._move_jog_watchdog)

        if bool(self.declare_parameter("connect_on_start", True).value):
            try:
                self.controller.connect()
                self.get_logger().info("Connected to Dobot controller")
            except Exception as exc:
                self.get_logger().error(f"Initial Dobot connection failed: {exc}")
        self._start_cartesian_servo_loop()

    def destroy_node(self):
        with self._move_jog_watchdog_lock:
            move_jog_active = self._move_jog_active
            self._move_jog_active = False
        if move_jog_active:
            self.controller.move_jog("", stop=True)
        self.servo_stop_event.set()
        if self.servo_thread is not None:
            self.servo_thread.join(timeout=1.0)
            self.servo_thread = None
        self._stop_cartesian_servo("node shutdown", reset_channel=True)
        if self.limit_recovery.brake_released:
            self.limit_recovery.lock()
        self.gripper.disconnect()
        self.controller.disconnect()
        super().destroy_node()

    def _configure_cartesian_servo(self):
        self.servo_rate_hz = float(
            self.declare_parameter("cartesian_servo.rate_hz", 33.0).value
        )
        self.servo_watchdog_sec = float(
            self.declare_parameter("cartesian_servo.watchdog_sec", 0.2).value
        )
        self.servo_feedback_watchdog_sec = float(
            self.declare_parameter(
                "cartesian_servo.feedback_watchdog_sec", 0.25
            ).value
        )
        self.servo_transport_watchdog_sec = float(
            self.declare_parameter(
                "cartesian_servo.transport_watchdog_sec", 0.2
            ).value
        )
        self.servo_applied_rate_hz = float(
            self.declare_parameter(
                "cartesian_servo.applied_rate_hz", 20.0
            ).value
        )
        self.servo_max_translation_mm_s = float(
            self.declare_parameter(
                "cartesian_servo.max_translation_speed_mm_s", 45.0
            ).value
        )
        self.servo_max_rotation_deg_s = float(
            self.declare_parameter(
                "cartesian_servo.max_rotation_speed_deg_s", 15.0
            ).value
        )
        self.servo_accel_normalized_s = float(
            self.declare_parameter(
                "cartesian_servo.acceleration_normalized_s", 16.0
            ).value
        )
        self.servo_joint_margin_deg = float(
            self.declare_parameter("cartesian_servo.joint_limit_margin_deg", 0.0).value
        )
        self.servo_workspace_min = self._float_list_parameter(
            "cartesian_servo.workspace_min",
            [-625.0, -625.0, 20.0, -360.0, -360.0, -360.0],
        )
        self.servo_workspace_max = self._float_list_parameter(
            "cartesian_servo.workspace_max",
            [625.0, 625.0, 625.0, 360.0, 360.0, 360.0],
        )
        self.servo_workspace_radius_mm = float(
            self.declare_parameter(
                "cartesian_servo.workspace_max_xy_radius_mm", 625.0
            ).value
        )
        self.servo_command = [0.0] * 6
        self.servo_command_lock = threading.Lock()
        self.servo_state_lock = threading.RLock()
        self.servo_command_callback_group = MutuallyExclusiveCallbackGroup()
        self.servo_stop_event = threading.Event()
        self.servo_thread = None
        self.servo_command_active = False
        self.servo_command_deadman = False
        self.servo_command_coord_type = 0
        self.servo_command_received = 0.0
        self.servo_active = False
        self.servo_pause_reason = ""
        self.servo_transport_fault_latched = False
        self.servo_target_pose = None
        self.servo_applied_velocity = [0.0] * 6
        self.servo_last_tick = time.monotonic()
        self.servo_stats_started = self.servo_last_tick
        self.servo_stats_ticks = 0
        self.servo_stats_dt_sum = 0.0
        self.servo_stats_dt_max = 0.0
        self.servo_stats_send_max = 0.0
        self.servo_stats_busy_ticks = 0
        self.servo_last_send_success = self.servo_last_tick
        self.servo_last_applied_publish = 0.0
        self.servo_last_applied_status = ""
        servo_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.servo_command_sub = self.create_subscription(
            CartesianServoCommand,
            "/cartesian_servo/command",
            self._on_cartesian_servo_command,
            servo_qos,
            callback_group=self.servo_command_callback_group,
        )
        self.servo_applied_pub = self.create_publisher(
            CartesianServoCommand, "/cartesian_servo/applied", servo_qos
        )

    def _start_cartesian_servo_loop(self):
        if self.servo_rate_hz <= 0.0 or self.servo_thread is not None:
            return
        self.servo_thread = threading.Thread(
            target=self._cartesian_servo_loop,
            name="dobot-cartesian-servo",
            daemon=True,
        )
        self.servo_thread.start()
        self.get_logger().info(
            f"Cartesian ServoP scheduler ready at {self.servo_rate_hz:.1f} Hz"
        )

    def _cartesian_servo_loop(self):
        period = 1.0 / self.servo_rate_hz
        next_tick = time.monotonic()
        while not self.servo_stop_event.is_set():
            delay = next_tick - time.monotonic()
            if delay > 0.0 and self.servo_stop_event.wait(delay):
                return
            if not self.context.ok():
                return
            try:
                self._cartesian_servo_tick()
            except Exception as exc:
                if not self.context.ok() or self.servo_stop_event.is_set():
                    return
                self.get_logger().error(f"Cartesian ServoP scheduler failed: {exc}")
            next_tick += period
            now = time.monotonic()
            if next_tick <= now:
                # Skip missed slots. Sending catch-up bursts creates stale
                # targets and increases controller-side TCP backpressure.
                next_tick = now + period

    def _on_cartesian_servo_command(self, message: CartesianServoCommand):
        with self.servo_command_lock:
            self.servo_command = clamp_normalized_vector(
                message.normalized_velocity
            )
            self.servo_command_active = bool(message.active)
            self.servo_command_deadman = bool(message.deadman)
            self.servo_command_coord_type = int(message.coord_type)
            self.servo_command_received = time.monotonic()
            if not self.servo_command_active or not self.servo_command_deadman:
                self.servo_transport_fault_latched = False

    def _cartesian_servo_tick(self):
        with self.servo_state_lock:
            self._cartesian_servo_tick_locked()

    def _cartesian_servo_tick_locked(self):
        now = time.monotonic()
        tick_dt = max(now - self.servo_last_tick, 0.0)
        dt = min(tick_dt, 2.0 / self.servo_rate_hz)
        self.servo_last_tick = now
        with self.servo_command_lock:
            reason = self._cartesian_servo_rejection(now)
            command = list(self.servo_command)
        if reason:
            if reason in {"command watchdog", "feedback watchdog"}:
                if self._pause_cartesian_servo(reason, now):
                    return
            soft_stop = reason in NON_REVERSING_HOLD_REASONS
            self._stop_cartesian_servo(
                reason,
                send_hold=soft_stop,
                reset_channel=not soft_stop,
            )
            self._publish_cartesian_servo_applied(False, reason)
            return

        if self.servo_pause_reason:
            self.get_logger().info(
                f"Cartesian ServoP stream resumed after {self.servo_pause_reason}"
            )
            self.servo_pause_reason = ""
            self._reset_cartesian_servo_stats(now)

        state = self.controller.latest_state()
        if not self.servo_active:
            try:
                self.controller.prepare_servo_stream()
            except Exception as exc:
                with self.servo_command_lock:
                    self.servo_transport_fault_latched = True
                self.get_logger().error(
                    f"Cartesian ServoP prepare failed: {exc}"
                )
                self._publish_cartesian_servo_applied(
                    False,
                    "transport failure",
                    force=True,
                )
                return
            self.servo_active = True
            self.servo_target_pose = list(state.tcp_pose[:6])
            self.servo_applied_velocity = [0.0] * 6
            self._reset_cartesian_servo_stats(now)
            self.servo_last_send_success = now
            self.get_logger().info(
                f"Cartesian ServoP stream started at {self.servo_rate_hz:.1f} Hz"
            )

        previous_velocity = list(self.servo_applied_velocity)
        self.servo_applied_velocity = slew_vector(
            previous_velocity,
            command,
            self.servo_accel_normalized_s * dt,
        )
        target = integrate_pose(
            self.servo_target_pose,
            self.servo_applied_velocity,
            dt,
            self.servo_max_translation_mm_s,
            self.servo_max_rotation_deg_s,
        )
        if not pose_within_workspace(
            target,
            self.servo_workspace_min,
            self.servo_workspace_max,
            self.servo_workspace_radius_mm,
        ):
            self._stop_cartesian_servo("workspace limit", reset_channel=True)
            self._publish_cartesian_servo_applied(False, "workspace limit")
            return
        try:
            send_started = time.monotonic()
            result = self.controller.servo_p(target, ensure_connected=False)
            send_elapsed = time.monotonic() - send_started
            if not result.success:
                raise RuntimeError(
                    f"error_id={result.error_id}, raw_reply={result.raw_reply}"
                )
        except ServoStreamBusy:
            self.servo_applied_velocity = previous_velocity
            self.servo_stats_busy_ticks += 1
            if now - self.servo_last_send_success <= self.servo_transport_watchdog_sec:
                self._publish_cartesian_servo_applied(
                    True,
                    "transport busy; holding previous target",
                )
                return
            with self.servo_command_lock:
                self.servo_transport_fault_latched = True
            self._stop_cartesian_servo(
                "ServoP transport busy watchdog",
                send_hold=False,
                reset_channel=True,
            )
            self._publish_cartesian_servo_applied(False, "transport failure")
            return
        except Exception as exc:
            with self.servo_command_lock:
                self.servo_transport_fault_latched = True
            self._stop_cartesian_servo(
                f"ServoP transport failed: {exc}",
                send_hold=False,
                reset_channel=True,
            )
            self._publish_cartesian_servo_applied(False, "transport failure")
            return
        self.servo_target_pose = target
        self.servo_last_send_success = now
        self._update_cartesian_servo_stats(now, tick_dt, send_elapsed)
        self._publish_cartesian_servo_applied(True, "applied")

    def _pause_cartesian_servo(self, reason: str, now: float) -> bool:
        if not self.servo_active:
            return False
        self.servo_applied_velocity = [0.0] * 6
        if reason != self.servo_pause_reason:
            if reason == "command watchdog" and self.servo_target_pose is not None:
                try:
                    result = self.controller.servo_p(
                        self.servo_target_pose,
                        ensure_connected=False,
                    )
                    if not result.success:
                        raise RuntimeError(
                            f"error_id={result.error_id}, raw_reply={result.raw_reply}"
                        )
                    self.servo_last_send_success = now
                except Exception as exc:
                    with self.servo_command_lock:
                        self.servo_transport_fault_latched = True
                    self._stop_cartesian_servo(
                        f"ServoP watchdog hold failed: {exc}",
                        send_hold=False,
                        reset_channel=True,
                    )
                    self._publish_cartesian_servo_applied(
                        False,
                        "transport failure",
                        force=True,
                    )
                    return True
            self.servo_pause_reason = reason
            self.get_logger().warning(
                f"Cartesian ServoP stream paused: {reason}"
            )
            self._reset_cartesian_servo_stats(now)
        self._publish_cartesian_servo_applied(False, reason)
        return True

    def _cartesian_servo_rejection(self, now: float) -> str:
        if not self.servo_command_active or not self.servo_command_deadman:
            return "inactive"
        if self.servo_transport_fault_latched:
            return "transport fault latched; release deadman"
        if now - self.servo_command_received > self.servo_watchdog_sec:
            return "command watchdog"
        if self.servo_command_coord_type != 0:
            return "ServoP only supports user coordinate velocity"
        if self.limit_recovery.active:
            return "limit recovery active"
        state = self.controller.latest_state()
        if state.stamp <= 0.0 or len(state.tcp_pose) < 6:
            return "feedback unavailable"
        if time.time() - state.stamp > self.servo_feedback_watchdog_sec:
            return "feedback watchdog"
        if state.enable_status != 1 or state.error_status != 0:
            return "robot is not ready"
        if not joints_within_margin(
            state.joints,
            self.config.joint_lower_limits_deg,
            self.config.joint_upper_limits_deg,
            self.servo_joint_margin_deg,
        ):
            return "joint limit margin"
        return ""

    def _stop_cartesian_servo(
        self,
        reason: str,
        send_hold: bool = True,
        reset_channel: bool = False,
    ):
        with self.servo_state_lock:
            self._stop_cartesian_servo_locked(reason, send_hold, reset_channel)

    def _stop_cartesian_servo_locked(
        self,
        reason: str,
        send_hold: bool,
        reset_channel: bool,
    ):
        if not self.servo_active:
            self.servo_applied_velocity = [0.0] * 6
            return
        state = self.controller.latest_state()
        hold_pose = select_hold_pose(
            reason,
            self.servo_target_pose or [],
            state.tcp_pose,
        )
        if send_hold and len(hold_pose) >= 6:
            try:
                result = self.controller.servo_p(
                    hold_pose,
                    ensure_connected=False,
                )
                if not result.success:
                    self.get_logger().error(
                        "Cartesian ServoP hold rejected: "
                        f"error_id={result.error_id}, raw_reply={result.raw_reply}"
                    )
            except Exception as exc:
                self.get_logger().error(f"Cartesian ServoP hold failed: {exc}")
        try:
            self.controller.end_servo_stream(reset_channel=reset_channel)
        except Exception as exc:
            self.get_logger().error(f"Cartesian ServoP channel reset failed: {exc}")
        self.servo_active = False
        self.servo_target_pose = None
        self.servo_applied_velocity = [0.0] * 6
        self.servo_pause_reason = ""
        self.get_logger().info(f"Cartesian ServoP stream stopped: {reason}")

    def _update_cartesian_servo_stats(
        self,
        now: float,
        dt: float,
        send_elapsed: float,
    ):
        self.servo_stats_ticks += 1
        self.servo_stats_dt_sum += dt
        self.servo_stats_dt_max = max(self.servo_stats_dt_max, dt)
        self.servo_stats_send_max = max(self.servo_stats_send_max, send_elapsed)
        elapsed = now - self.servo_stats_started
        if elapsed < 2.0:
            return
        mean_dt = self.servo_stats_dt_sum / max(1, self.servo_stats_ticks)
        actual_rate = self.servo_stats_ticks / max(elapsed, 1e-6)
        self.get_logger().info(
            "Cartesian ServoP timing: "
            f"rate={actual_rate:.1f} Hz, mean_dt={mean_dt * 1000.0:.1f} ms, "
            f"max_dt={self.servo_stats_dt_max * 1000.0:.1f} ms, "
            f"max_send={self.servo_stats_send_max * 1000.0:.1f} ms, "
            f"busy_ticks={self.servo_stats_busy_ticks}, "
            f"command={[round(value, 3) for value in self.servo_command]}"
        )
        self._reset_cartesian_servo_stats(now)

    def _reset_cartesian_servo_stats(self, now: float):
        self.servo_stats_started = now
        self.servo_stats_ticks = 0
        self.servo_stats_dt_sum = 0.0
        self.servo_stats_dt_max = 0.0
        self.servo_stats_send_max = 0.0
        self.servo_stats_busy_ticks = 0

    def _publish_cartesian_servo_applied(
        self,
        active: bool,
        status: str,
        force: bool = False,
    ):
        if not self.context.ok():
            return
        now = time.monotonic()
        period = (
            1.0 / self.servo_applied_rate_hz
            if self.servo_applied_rate_hz > 0.0
            else 0.0
        )
        status_changed = status != self.servo_last_applied_status
        if (
            not force
            and not status_changed
            and period > 0.0
            and now - self.servo_last_applied_publish < period
        ):
            return
        message = CartesianServoCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.normalized_velocity = list(self.servo_applied_velocity)
        message.active = bool(active)
        message.deadman = bool(self.servo_command_deadman)
        message.coord_type = int(self.servo_command_coord_type)
        message.user = 0
        message.tool = 0
        message.status = str(status)
        self.servo_applied_pub.publish(message)
        self.servo_last_applied_publish = now
        self.servo_last_applied_status = str(status)

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
                self.declare_parameter("teach_trajectory_dir", "/home/ros/ws/data/trajectories").value
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
        if self.limit_recovery.active:
            response.success = False
            response.message = "enable rejected: limit recovery is active"
            return response
        return self._handle_dashboard("enable_robot", self.controller.enable_robot(), response)

    def _disable_robot(self, request, response):
        del request
        self._stop_cartesian_servo("robot disable", reset_channel=True)
        return self._handle_dashboard("disable_robot", self.controller.disable_robot(), response)

    def _emergency_stop(self, request, response):
        del request
        self._stop_cartesian_servo("emergency stop", reset_channel=True)
        return self._handle_dashboard("emergency_stop", self.controller.emergency_stop(), response)

    def _drag_start(self, request, response):
        del request
        self._stop_cartesian_servo("drag start", reset_channel=True)
        if self.limit_recovery.active:
            response.success = False
            response.message = "drag start rejected: limit recovery is active"
            return response
        return self._handle_dashboard("drag_start", self.controller.drag_start(), response)

    def _drag_stop(self, request, response):
        del request
        return self._handle_dashboard("drag_stop", self.controller.drag_stop(), response)

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

    def _move_jog(self, request: JogCommand.Request, response: JogCommand.Response):
        start_time = time.monotonic()
        if self.servo_active and not bool(request.stop):
            response.success = False
            response.error_id = -1
            response.message = "move_jog rejected: Cartesian ServoP stream is active"
            response.raw_reply = ""
            return response
        if self.limit_recovery.active and not bool(request.stop):
            response.success = False
            response.error_id = -1
            response.message = "move_jog rejected: limit recovery is active"
            response.raw_reply = ""
            return response
        result = self.controller.move_jog(
            str(request.axis_id),
            stop=bool(request.stop),
            coord_type=int(request.coord_type),
            user=int(request.user),
            tool=int(request.tool),
        )
        if result.success:
            with self._move_jog_watchdog_lock:
                self._move_jog_active = not bool(request.stop)
                self._move_jog_last_command = time.monotonic()
        response.success = result.success
        response.error_id = int(result.error_id)
        response.message = result.message
        response.raw_reply = result.raw_reply
        elapsed_ms = round((time.monotonic() - start_time) * 1000.0, 3)
        self._publish_move_jog_diagnostics(request, response, elapsed_ms)
        if result.success:
            self.get_logger().info(f"move_jog accepted in {elapsed_ms:.1f} ms")
        else:
            self.get_logger().warning(
                f"move_jog rejected in {elapsed_ms:.1f} ms: {result.message}"
            )
        return response

    def _move_jog_watchdog(self):
        if self.move_jog_watchdog_sec <= 0.0:
            return
        now = time.monotonic()
        with self._move_jog_watchdog_lock:
            expired = heartbeat_expired(
                self._move_jog_active,
                self._move_jog_last_command,
                now,
                self.move_jog_watchdog_sec,
            )
            if expired:
                self._move_jog_active = False
        if not expired:
            return
        result = self.controller.move_jog("", stop=True)
        if result.success:
            self.get_logger().warning(
                "MoveJog heartbeat watchdog stopped motion"
            )
        else:
            self.get_logger().error(
                "MoveJog heartbeat watchdog stop failed: " + result.message
            )

    def _limit_recovery(
        self, request: LimitRecovery.Request, response: LimitRecovery.Response
    ):
        action = str(request.action).strip().lower()
        if action == "prepare":
            self._stop_cartesian_servo(
                "limit recovery prepare",
                reset_channel=True,
            )
            result = self.limit_recovery.prepare()
        elif action == "release":
            result = self.limit_recovery.release()
        elif action == "lock":
            result = self.limit_recovery.lock()
        elif action == "status":
            result = DashboardResult(
                True,
                0,
                "limit recovery status",
            )
        else:
            result = DashboardResult(
                False, message=f"unsupported limit recovery action: {action}"
            )
        response.success = bool(result.success)
        response.error_id = int(result.error_id)
        response.message = str(result.message)
        response.joint = int(self.limit_recovery.joint)
        response.brake_released = bool(self.limit_recovery.brake_released)
        response.raw_reply = str(result.raw_reply)
        return response

    def _limit_recovery_watchdog(self):
        result = self.limit_recovery.watchdog()
        if result is not None:
            self.get_logger().error(
                "limit recovery watchdog timeout; " + result.message
            )

    def _publish_move_jog_diagnostics(
        self,
        request: JogCommand.Request,
        response: JogCommand.Response,
        elapsed_ms: float,
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "event": "move_jog_service",
                "axis": str(request.axis_id),
                "stop": bool(request.stop),
                "server_ms": float(elapsed_ms),
                "success": bool(response.success),
                "error_id": int(response.error_id),
                "stamp_monotonic_sec": time.monotonic(),
            },
            sort_keys=True,
        )
        self.move_jog_diagnostics_pub.publish(message)

    def _teach_start(self, request: TrajectoryCommand.Request, response):
        if self.servo_active:
            return self._reject_teach_for_servo("teach_start", response)
        if self.limit_recovery.active:
            return self._reject_teach_for_limit_recovery("teach_start", response)
        result = self.controller.teach_start(str(request.name), bool(request.overwrite))
        return self._handle_teach("teach_start", result, response)

    def _teach_stop(self, request: TrajectoryCommand.Request, response):
        result = self.controller.teach_stop(str(request.name))
        return self._handle_teach("teach_stop", result, response)

    def _teach_replay(self, request: TrajectoryCommand.Request, response):
        if self.servo_active:
            return self._reject_teach_for_servo("teach_replay", response)
        if self.limit_recovery.active:
            return self._reject_teach_for_limit_recovery("teach_replay", response)
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
        started_at = time.monotonic()
        result = self.gripper.move(
            float(request.opening_mm),
            int(request.position_permille),
            int(request.force_percent),
            float(request.force_n),
            bool(request.wait),
            float(request.timeout_sec),
        )
        elapsed_ms = (time.monotonic() - started_at) * 1000.0
        self._fill_gripper_command_response(result, response)
        if result.success:
            self.get_logger().info(
                f"gripper_move accepted in {elapsed_ms:.1f} ms"
            )
        else:
            self.get_logger().warning(
                f"gripper_move rejected in {elapsed_ms:.1f} ms: {result.message}"
            )
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

        if self.servo_active:
            response.success = False
            response.error_id = -1
            response.message = f"{kind} rejected: Cartesian ServoP stream is active"
            return response
        if self.limit_recovery.active:
            response.success = False
            response.error_id = -1
            response.message = f"{kind} rejected: limit recovery is active"
            return response
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

    def _reject_teach_for_limit_recovery(self, name: str, response):
        response.success = False
        response.error_id = -1
        response.message = f"{name} rejected: limit recovery is active"
        return response

    def _reject_teach_for_servo(self, name: str, response):
        response.success = False
        response.error_id = -1
        response.message = f"{name} rejected: Cartesian ServoP stream is active"
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
        if not self.context.ok():
            return
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
        if not self.context.ok():
            return
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
    # Keep joystick reception and the 33 Hz ServoP loop schedulable while a
    # slower gripper or service callback occupies the default callback group.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        # ros2 launch can invalidate the shared context before spin() returns.
        # That is a normal shutdown race, not a driver failure; preserve any
        # exception raised while the context is still live.
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
