from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _robot_launch(context, *args, **kwargs):
    del args, kwargs
    mode = LaunchConfiguration("robot_mode").perform(context).strip().lower()
    if mode == "external":
        return []
    if mode not in {"bringup", "driver"}:
        raise ValueError("robot_mode must be bringup, driver, or external")
    filename = "dobot_bringup.launch.py" if mode == "bringup" else "dobot_driver.launch.py"
    launch_file = PathJoinSubstitution(
        [FindPackageShare("dobot_ros2"), "launch", filename]
    )
    launch_arguments = {
        "params_file": LaunchConfiguration("params_file"),
        "namespace": LaunchConfiguration("namespace"),
    }
    if mode == "bringup":
        launch_arguments.update(
            {
                "rviz": LaunchConfiguration("rviz"),
                "handeye_tf": LaunchConfiguration("start_camera"),
                "handeye_result_file": LaunchConfiguration("handeye_result_file"),
                "handeye_output_child_frame": LaunchConfiguration(
                    "handeye_output_child_frame"
                ),
            }
        )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    camera_launch = PathJoinSubstitution(
        [FindPackageShare("dobot_camera"), "launch", "dual_camera.launch.py"]
    )
    viewer_launch = PathJoinSubstitution(
        [FindPackageShare("dobot_camera"), "launch", "camera_view.launch.py"]
    )
    joy_launch = PathJoinSubstitution(
        [FindPackageShare("dobot_joy"), "launch", "joy_teleop.launch.py"]
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("dobot_ros2"), "config", "dobot_ros2.yaml"]
    )

    declarations = [
        DeclareLaunchArgument("robot_mode", default_value="bringup"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("handeye_result_file", default_value=""),
        DeclareLaunchArgument("handeye_output_child_frame", default_value="camera_link"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_view", default_value="false"),
        DeclareLaunchArgument("start_joy", default_value="true"),
        DeclareLaunchArgument("start_data_collection", default_value="true"),
        DeclareLaunchArgument("start_operator_panel", default_value="false"),
        DeclareLaunchArgument("init_gripper", default_value="true"),
        DeclareLaunchArgument("run_log_dir", default_value="system_logs/current"),
        DeclareLaunchArgument("orbbec_launch_file", default_value="gemini_330_series.launch.py"),
        DeclareLaunchArgument("wrist_camera_name", default_value="camera"),
        DeclareLaunchArgument("wrist_serial_number", default_value=""),
        DeclareLaunchArgument("wrist_usb_port", default_value=""),
        DeclareLaunchArgument("global_camera_name", default_value="global_camera"),
        DeclareLaunchArgument("global_serial_no", default_value="''"),
        DeclareLaunchArgument("global_usb_port_id", default_value="''"),
        DeclareLaunchArgument("global_color_profile", default_value="640,480,30"),
        DeclareLaunchArgument("global_enable_depth", default_value="false"),
        DeclareLaunchArgument("wrist_image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument(
            "wrist_camera_info_topic", default_value="/camera/color/camera_info"
        ),
        DeclareLaunchArgument(
            "global_image_topic", default_value="/global_camera/color/image_raw"
        ),
        DeclareLaunchArgument(
            "global_camera_info_topic",
            default_value="/global_camera/color/camera_info",
        ),
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_autorepeat_rate", default_value="50.0"),
        DeclareLaunchArgument("dataset_root", default_value="data/collections/move_jog"),
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
        DeclareLaunchArgument("task_file", default_value=""),
        DeclareLaunchArgument("lerobot_enabled", default_value="true"),
        DeclareLaunchArgument("lerobot_python", default_value="python3"),
        DeclareLaunchArgument("lerobot_dataset_root", default_value=""),
        DeclareLaunchArgument(
            "lerobot_repo_id", default_value="local/dobot_nova2_pi05"
        ),
        DeclareLaunchArgument("lerobot_export_timeout_sec", default_value="900.0"),
        DeclareLaunchArgument("diagnostics_topic", default_value="/joy/teleop_diagnostics"),
        DeclareLaunchArgument("deadman_button_index", default_value="4"),
        DeclareLaunchArgument("estop_button_index", default_value="1"),
        DeclareLaunchArgument("toggle_enable_button_index", default_value="3"),
        DeclareLaunchArgument("toggle_drag_button_index", default_value="5"),
        DeclareLaunchArgument("collection_prepare_hold_sec", default_value="1.5"),
        DeclareLaunchArgument("deadzone", default_value="0.25"),
        DeclareLaunchArgument("control_mode", default_value="move_jog"),
        DeclareLaunchArgument("response_exponent", default_value="1.2"),
        DeclareLaunchArgument("coord_type", default_value="0"),
        DeclareLaunchArgument("x_axis_index", default_value="1"),
        DeclareLaunchArgument("y_axis_index", default_value="0"),
        DeclareLaunchArgument("rx_axis_index", default_value="6"),
        DeclareLaunchArgument("ry_axis_index", default_value="7"),
        DeclareLaunchArgument("x_axis_sign", default_value="-1.0"),
        DeclareLaunchArgument("y_axis_sign", default_value="-1.0"),
        DeclareLaunchArgument("z_axis_sign", default_value="1.0"),
        DeclareLaunchArgument("rz_axis_sign", default_value="-1.0"),
        DeclareLaunchArgument("rx_axis_sign", default_value="-1.0"),
        DeclareLaunchArgument("ry_axis_sign", default_value="-1.0"),
        DeclareLaunchArgument("gripper_step_mm", default_value="2.0"),
        DeclareLaunchArgument("gripper_stop_lead_mm", default_value="3.0"),
        DeclareLaunchArgument("gripper_force_percent", default_value="50"),
        DeclareLaunchArgument("enable_rumble", default_value="true"),
        DeclareLaunchArgument("joint_limit_margin_deg", default_value="0.0"),
        DeclareLaunchArgument("limit_recovery_hold_sec", default_value="3.0"),
        DeclareLaunchArgument("data_reject_hold_sec", default_value="2.0"),
        DeclareLaunchArgument("limit_recovery_release_timeout_sec", default_value="10.0"),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        launch_arguments={
            "orbbec_launch_file": LaunchConfiguration("orbbec_launch_file"),
            "wrist_camera_name": LaunchConfiguration("wrist_camera_name"),
            "wrist_serial_number": LaunchConfiguration("wrist_serial_number"),
            "wrist_usb_port": LaunchConfiguration("wrist_usb_port"),
            "global_camera_name": LaunchConfiguration("global_camera_name"),
            "global_serial_no": LaunchConfiguration("global_serial_no"),
            "global_usb_port_id": LaunchConfiguration("global_usb_port_id"),
            "global_color_profile": LaunchConfiguration("global_color_profile"),
            "global_enable_depth": LaunchConfiguration("global_enable_depth"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )
    viewer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(viewer_launch),
        launch_arguments={
            "wrist_image_topic": LaunchConfiguration("wrist_image_topic"),
            "global_image_topic": LaunchConfiguration("global_image_topic"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_view")),
    )
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch),
        launch_arguments={
            "joy_topic": LaunchConfiguration("joy_topic"),
            "dev": LaunchConfiguration("joy_dev"),
            "autorepeat_rate": LaunchConfiguration("joy_autorepeat_rate"),
            "start_data_collection": LaunchConfiguration("start_data_collection"),
            "start_operator_panel": LaunchConfiguration("start_operator_panel"),
            "dataset_root": LaunchConfiguration("dataset_root"),
            "wrist_image_topic": LaunchConfiguration("wrist_image_topic"),
            "wrist_camera_info_topic": LaunchConfiguration("wrist_camera_info_topic"),
            "global_image_topic": LaunchConfiguration("global_image_topic"),
            "global_camera_info_topic": LaunchConfiguration("global_camera_info_topic"),
            "data_sample_rate_hz": LaunchConfiguration("data_sample_rate_hz"),
            "collection_require_start_pose": LaunchConfiguration("collection_require_start_pose"),
            "collection_start_pose_file": LaunchConfiguration("collection_start_pose_file"),
            "collection_start_joint_tolerance_deg": LaunchConfiguration("collection_start_joint_tolerance_deg"),
            "collection_start_gripper_tolerance_mm": LaunchConfiguration("collection_start_gripper_tolerance_mm"),
            "collection_prepare_speed": LaunchConfiguration("collection_prepare_speed"),
            "collection_prepare_acceleration": LaunchConfiguration("collection_prepare_acceleration"),
            "collection_prepare_timeout_sec": LaunchConfiguration("collection_prepare_timeout_sec"),
            "collection_auto_return_after_stop": LaunchConfiguration("collection_auto_return_after_stop"),
            "collection_auto_return_opening_mm": LaunchConfiguration("collection_auto_return_opening_mm"),
            "max_image_skew_sec": LaunchConfiguration("max_image_skew_sec"),
            "task_file": LaunchConfiguration("task_file"),
            "lerobot_enabled": LaunchConfiguration("lerobot_enabled"),
            "lerobot_python": LaunchConfiguration("lerobot_python"),
            "lerobot_dataset_root": LaunchConfiguration("lerobot_dataset_root"),
            "lerobot_repo_id": LaunchConfiguration("lerobot_repo_id"),
            "lerobot_export_timeout_sec": LaunchConfiguration(
                "lerobot_export_timeout_sec"
            ),
            "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
            "deadman_button_index": LaunchConfiguration("deadman_button_index"),
            "estop_button_index": LaunchConfiguration("estop_button_index"),
            "toggle_enable_button_index": LaunchConfiguration(
                "toggle_enable_button_index"
            ),
            "toggle_drag_button_index": LaunchConfiguration(
                "toggle_drag_button_index"
            ),
            "collection_prepare_hold_sec": LaunchConfiguration(
                "collection_prepare_hold_sec"
            ),
            "deadzone": LaunchConfiguration("deadzone"),
            "control_mode": LaunchConfiguration("control_mode"),
            "response_exponent": LaunchConfiguration("response_exponent"),
            "coord_type": LaunchConfiguration("coord_type"),
            "x_axis_index": LaunchConfiguration("x_axis_index"),
            "y_axis_index": LaunchConfiguration("y_axis_index"),
            "rx_axis_index": LaunchConfiguration("rx_axis_index"),
            "ry_axis_index": LaunchConfiguration("ry_axis_index"),
            "x_axis_sign": LaunchConfiguration("x_axis_sign"),
            "y_axis_sign": LaunchConfiguration("y_axis_sign"),
            "z_axis_sign": LaunchConfiguration("z_axis_sign"),
            "rz_axis_sign": LaunchConfiguration("rz_axis_sign"),
            "rx_axis_sign": LaunchConfiguration("rx_axis_sign"),
            "ry_axis_sign": LaunchConfiguration("ry_axis_sign"),
            "gripper_step_mm": LaunchConfiguration("gripper_step_mm"),
            "gripper_stop_lead_mm": LaunchConfiguration("gripper_stop_lead_mm"),
            "gripper_force_percent": LaunchConfiguration("gripper_force_percent"),
            "enable_rumble": LaunchConfiguration("enable_rumble"),
            "joint_limit_margin_deg": LaunchConfiguration("joint_limit_margin_deg"),
            "limit_recovery_hold_sec": LaunchConfiguration("limit_recovery_hold_sec"),
            "data_reject_hold_sec": LaunchConfiguration("data_reject_hold_sec"),
            "limit_recovery_release_timeout_sec": LaunchConfiguration(
                "limit_recovery_release_timeout_sec"
            ),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_joy")),
    )
    monitor = Node(
        package="dobot_ros2",
        executable="dobot_system_monitor",
        name="dobot_system_monitor",
        output="screen",
        parameters=[
            {
                "log_dir": LaunchConfiguration("run_log_dir"),
                "robot_mode": LaunchConfiguration("robot_mode"),
                "start_camera": ParameterValue(
                    LaunchConfiguration("start_camera"), value_type=bool
                ),
                "start_joy": ParameterValue(
                    LaunchConfiguration("start_joy"), value_type=bool
                ),
                "start_data_collection": ParameterValue(
                    LaunchConfiguration("start_data_collection"), value_type=bool
                ),
                "init_gripper": ParameterValue(
                    LaunchConfiguration("init_gripper"), value_type=bool
                ),
                "task_file": LaunchConfiguration("task_file"),
                "dataset_root": LaunchConfiguration("dataset_root"),
                "lerobot_dataset_root": LaunchConfiguration(
                    "lerobot_dataset_root"
                ),
                "lerobot_repo_id": LaunchConfiguration("lerobot_repo_id"),
                "control_mode": LaunchConfiguration("control_mode"),
                "data_reject_hold_sec": ParameterValue(
                    LaunchConfiguration("data_reject_hold_sec"), value_type=float
                ),
                "wrist_image_topic": LaunchConfiguration("wrist_image_topic"),
                "global_image_topic": LaunchConfiguration("global_image_topic"),
                "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
            }
        ],
    )

    return LaunchDescription(
        declarations
        + [OpaqueFunction(function=_robot_launch), camera, viewer, joy, monitor]
    )
