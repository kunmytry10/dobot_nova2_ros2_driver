from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument("start_joy_node", default_value="true"),
            DeclareLaunchArgument("dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("autorepeat_rate", default_value="50.0"),
            DeclareLaunchArgument("start_data_collection", default_value="true"),
            DeclareLaunchArgument("start_operator_panel", default_value="false"),
            DeclareLaunchArgument("dataset_root", default_value="data/collections/move_jog"),
            DeclareLaunchArgument(
                "wrist_image_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "wrist_camera_info_topic", default_value="/camera/color/camera_info"
            ),
            DeclareLaunchArgument(
                "global_image_topic",
                default_value="/global_camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "global_camera_info_topic",
                default_value="/global_camera/color/camera_info",
            ),
            DeclareLaunchArgument("data_sample_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("collection_require_start_pose", default_value="false"),
            DeclareLaunchArgument("collection_start_pose_file", default_value=""),
            DeclareLaunchArgument("collection_start_joint_tolerance_deg", default_value="1.0"),
            DeclareLaunchArgument("collection_start_gripper_tolerance_mm", default_value="3.0"),
            DeclareLaunchArgument("collection_prepare_speed", default_value="10"),
            DeclareLaunchArgument("collection_prepare_acceleration", default_value="10"),
            DeclareLaunchArgument("collection_prepare_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("collection_auto_return_after_stop", default_value="true"),
            DeclareLaunchArgument("collection_auto_return_opening_mm", default_value="95.0"),
            DeclareLaunchArgument("max_image_skew_sec", default_value="0.05"),
            DeclareLaunchArgument("task_instruction", default_value=""),
            DeclareLaunchArgument("task_file", default_value=""),
            DeclareLaunchArgument("lerobot_enabled", default_value="true"),
            DeclareLaunchArgument("lerobot_python", default_value="python3"),
            DeclareLaunchArgument("lerobot_dataset_root", default_value=""),
            DeclareLaunchArgument(
                "lerobot_repo_id", default_value="local/dobot_nova2_pi05"
            ),
            DeclareLaunchArgument("lerobot_export_timeout_sec", default_value="900.0"),
            DeclareLaunchArgument(
                "diagnostics_topic", default_value="/joy/teleop_diagnostics"
            ),
            DeclareLaunchArgument("deadman_button_index", default_value="4"),
            DeclareLaunchArgument("estop_button_index", default_value="1"),
            DeclareLaunchArgument("toggle_gripper_button_index", default_value="0"),
            DeclareLaunchArgument("toggle_enable_button_index", default_value="3"),
            DeclareLaunchArgument("toggle_drag_button_index", default_value="5"),
            DeclareLaunchArgument("collection_prepare_hold_sec", default_value="1.5"),
            DeclareLaunchArgument("clear_error_button_index", default_value="2"),
            DeclareLaunchArgument("deadzone", default_value="0.25"),
            DeclareLaunchArgument("control_mode", default_value="move_jog"),
            DeclareLaunchArgument("response_exponent", default_value="1.2"),
            DeclareLaunchArgument("coord_type", default_value="0"),
            DeclareLaunchArgument("x_axis_index", default_value="1"),
            DeclareLaunchArgument("y_axis_index", default_value="0"),
            DeclareLaunchArgument("x_axis_sign", default_value="-1.0"),
            DeclareLaunchArgument("y_axis_sign", default_value="-1.0"),
            DeclareLaunchArgument("z_axis_sign", default_value="1.0"),
            DeclareLaunchArgument("rz_axis_sign", default_value="-1.0"),
            DeclareLaunchArgument("rx_axis_index", default_value="6"),
            DeclareLaunchArgument("rx_axis_sign", default_value="-1.0"),
            DeclareLaunchArgument("ry_axis_index", default_value="7"),
            DeclareLaunchArgument("ry_axis_sign", default_value="-1.0"),
            DeclareLaunchArgument("gripper_step_mm", default_value="2.0"),
            DeclareLaunchArgument("gripper_stop_lead_mm", default_value="3.0"),
            DeclareLaunchArgument("gripper_force_percent", default_value="50"),
            DeclareLaunchArgument("enable_rumble", default_value="true"),
            DeclareLaunchArgument("joint_limit_margin_deg", default_value="0.0"),
            DeclareLaunchArgument("limit_recovery_hold_sec", default_value="3.0"),
            DeclareLaunchArgument("data_reject_hold_sec", default_value="2.0"),
            DeclareLaunchArgument(
                "limit_recovery_release_timeout_sec", default_value="10.0"
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "dev": LaunchConfiguration("dev"),
                        "autorepeat_rate": ParameterValue(
                            LaunchConfiguration("autorepeat_rate"),
                            value_type=float,
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("start_joy_node")),
            ),
            Node(
                package="dobot_joy",
                executable="dobot_data_collection",
                name="dobot_data_collection",
                output="screen",
                parameters=[
                    {
                        "dataset_root": LaunchConfiguration("dataset_root"),
                        "wrist_image_topic": LaunchConfiguration(
                            "wrist_image_topic"
                        ),
                        "wrist_camera_info_topic": LaunchConfiguration(
                            "wrist_camera_info_topic"
                        ),
                        "global_image_topic": LaunchConfiguration(
                            "global_image_topic"
                        ),
                        "global_camera_info_topic": LaunchConfiguration(
                            "global_camera_info_topic"
                        ),
                        "sample_rate_hz": ParameterValue(
                            LaunchConfiguration("data_sample_rate_hz"),
                            value_type=float,
                        ),
                        "collection.require_start_pose": ParameterValue(
                            LaunchConfiguration("collection_require_start_pose"), value_type=bool
                        ),
                        "collection.start_pose_file": LaunchConfiguration("collection_start_pose_file"),
                        "collection.start_joint_tolerance_deg": ParameterValue(
                            LaunchConfiguration("collection_start_joint_tolerance_deg"), value_type=float
                        ),
                        "collection.start_gripper_tolerance_mm": ParameterValue(
                            LaunchConfiguration("collection_start_gripper_tolerance_mm"), value_type=float
                        ),
                        "collection.prepare_speed": ParameterValue(
                            LaunchConfiguration("collection_prepare_speed"), value_type=int
                        ),
                        "collection.prepare_acceleration": ParameterValue(
                            LaunchConfiguration("collection_prepare_acceleration"), value_type=int
                        ),
                        "collection.prepare_timeout_sec": ParameterValue(
                            LaunchConfiguration("collection_prepare_timeout_sec"), value_type=float
                        ),
                        "collection.auto_return_after_stop": ParameterValue(
                            LaunchConfiguration("collection_auto_return_after_stop"), value_type=bool
                        ),
                        "collection.auto_return_opening_mm": ParameterValue(
                            LaunchConfiguration("collection_auto_return_opening_mm"), value_type=float
                        ),
                        "task_instruction": LaunchConfiguration(
                            "task_instruction"
                        ),
                        "task_file": LaunchConfiguration("task_file"),
                        "lerobot_enabled": ParameterValue(
                            LaunchConfiguration("lerobot_enabled"),
                            value_type=bool,
                        ),
                        "lerobot_python": LaunchConfiguration("lerobot_python"),
                        "lerobot_dataset_root": LaunchConfiguration(
                            "lerobot_dataset_root"
                        ),
                        "lerobot_repo_id": LaunchConfiguration("lerobot_repo_id"),
                        "lerobot_export_timeout_sec": ParameterValue(
                            LaunchConfiguration("lerobot_export_timeout_sec"),
                            value_type=float,
                        ),
                        "max_image_skew_sec": ParameterValue(
                            LaunchConfiguration("max_image_skew_sec"),
                            value_type=float,
                        ),
                        "control_mode": LaunchConfiguration("control_mode"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("start_data_collection")),
            ),
            Node(
                package="dobot_joy",
                executable="dobot_operator_panel",
                name="dobot_operator_panel",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_operator_panel")),
            ),
            Node(
                package="dobot_joy",
                executable="dobot_joy_teleop",
                name="dobot_joy_teleop",
                output="screen",
                parameters=[
                    {
                        "joy.topic": LaunchConfiguration("joy_topic"),
                        "joy.deadman_button_index": ParameterValue(
                            LaunchConfiguration("deadman_button_index"), value_type=int
                        ),
                        "joy.estop_button_index": ParameterValue(
                            LaunchConfiguration("estop_button_index"), value_type=int
                        ),
                        "joy.toggle_gripper_button_index": ParameterValue(
                            LaunchConfiguration("toggle_gripper_button_index"),
                            value_type=int,
                        ),
                        "joy.toggle_enable_button_index": ParameterValue(
                            LaunchConfiguration("toggle_enable_button_index"),
                            value_type=int,
                        ),
                        "joy.toggle_drag_button_index": ParameterValue(
                            LaunchConfiguration("toggle_drag_button_index"),
                            value_type=int,
                        ),
                        "joy.collection_prepare_hold_sec": ParameterValue(
                            LaunchConfiguration("collection_prepare_hold_sec"),
                            value_type=float,
                        ),
                        "joy.clear_error_button_index": ParameterValue(
                            LaunchConfiguration("clear_error_button_index"),
                            value_type=int,
                        ),
                        "joy.deadzone": ParameterValue(
                            LaunchConfiguration("deadzone"), value_type=float
                        ),
                        "joy.control_mode": LaunchConfiguration("control_mode"),
                        "joy.response_exponent": ParameterValue(
                            LaunchConfiguration("response_exponent"), value_type=float
                        ),
                        "joy.x_axis_index": ParameterValue(
                            LaunchConfiguration("x_axis_index"), value_type=int
                        ),
                        "joy.y_axis_index": ParameterValue(
                            LaunchConfiguration("y_axis_index"), value_type=int
                        ),
                        "joy.x_axis_sign": ParameterValue(
                            LaunchConfiguration("x_axis_sign"), value_type=float
                        ),
                        "joy.y_axis_sign": ParameterValue(
                            LaunchConfiguration("y_axis_sign"), value_type=float
                        ),
                        "joy.z_axis_sign": ParameterValue(
                            LaunchConfiguration("z_axis_sign"), value_type=float
                        ),
                        "joy.rz_axis_sign": ParameterValue(
                            LaunchConfiguration("rz_axis_sign"), value_type=float
                        ),
                        "joy.rx_axis_index": ParameterValue(
                            LaunchConfiguration("rx_axis_index"), value_type=int
                        ),
                        "joy.rx_axis_sign": ParameterValue(
                            LaunchConfiguration("rx_axis_sign"), value_type=float
                        ),
                        "joy.ry_axis_index": ParameterValue(
                            LaunchConfiguration("ry_axis_index"), value_type=int
                        ),
                        "joy.ry_axis_sign": ParameterValue(
                            LaunchConfiguration("ry_axis_sign"), value_type=float
                        ),
                        "joy.coord_type": ParameterValue(
                            LaunchConfiguration("coord_type"), value_type=int
                        ),
                        "joy.gripper_step_mm": ParameterValue(
                            LaunchConfiguration("gripper_step_mm"), value_type=float
                        ),
                        "joy.gripper_stop_lead_mm": ParameterValue(
                            LaunchConfiguration("gripper_stop_lead_mm"),
                            value_type=float,
                        ),
                        "joy.gripper_force_percent": ParameterValue(
                            LaunchConfiguration("gripper_force_percent"), value_type=int
                        ),
                        "joy.enable_rumble": ParameterValue(
                            LaunchConfiguration("enable_rumble"), value_type=bool
                        ),
                        "joy.diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
                        "joy.data_reject_hold_sec": ParameterValue(
                            LaunchConfiguration("data_reject_hold_sec"),
                            value_type=float,
                        ),
                        "joy.joint_limit_margin_deg": ParameterValue(
                            LaunchConfiguration("joint_limit_margin_deg"),
                            value_type=float,
                        ),
                        "joy.limit_recovery_hold_sec": ParameterValue(
                            LaunchConfiguration("limit_recovery_hold_sec"),
                            value_type=float,
                        ),
                        "joy.limit_recovery_release_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "limit_recovery_release_timeout_sec"
                            ),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
