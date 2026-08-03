from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    policy_share = FindPackageShare("dobot_policy")
    ros2_share = FindPackageShare("dobot_ros2")
    camera_share = FindPackageShare("dobot_camera")

    declarations = [
        DeclareLaunchArgument("start_robot", default_value="true"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("armed", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("auto_enable_robot", default_value="false"),
        DeclareLaunchArgument("auto_init_gripper", default_value="false"),
        DeclareLaunchArgument("gripper_enabled", default_value="true"),
        DeclareLaunchArgument("motion_test_duration_sec", default_value="3.0"),
        DeclareLaunchArgument("max_episode_sec", default_value="45.0"),
        DeclareLaunchArgument("policy_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("policy_port", default_value="8000"),
        DeclareLaunchArgument("policy_python", default_value="python3"),
        DeclareLaunchArgument(
            "policy_params_file",
            default_value=PathJoinSubstitution(
                [policy_share, "config", "pi05_tape_grasp.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "robot_params_file",
            default_value=PathJoinSubstitution(
                [ros2_share, "config", "dobot_ros2.yaml"]
            ),
        ),
        DeclareLaunchArgument("handeye_result_file", default_value=""),
        DeclareLaunchArgument("orbbec_launch_file", default_value="gemini_330_series.launch.py"),
        DeclareLaunchArgument("wrist_camera_name", default_value="camera"),
        DeclareLaunchArgument("wrist_serial_number", default_value=""),
        DeclareLaunchArgument("wrist_usb_port", default_value=""),
        DeclareLaunchArgument("global_camera_name", default_value="global_camera"),
        DeclareLaunchArgument("global_serial_no", default_value="''"),
        DeclareLaunchArgument("global_usb_port_id", default_value="''"),
        DeclareLaunchArgument("global_color_profile", default_value="640,480,30"),
    ]

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros2_share, "launch", "dobot_bringup.launch.py"])
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("robot_params_file"),
            "rviz": "false",
            "handeye_result_file": LaunchConfiguration("handeye_result_file"),
            "move_jog_watchdog_sec": "0.5",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_robot")),
    )
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([camera_share, "launch", "dual_camera.launch.py"])
        ),
        launch_arguments={
            "orbbec_launch_file": LaunchConfiguration("orbbec_launch_file"),
            "wrist_camera_name": LaunchConfiguration("wrist_camera_name"),
            "wrist_serial_number": LaunchConfiguration("wrist_serial_number"),
            "wrist_usb_port": LaunchConfiguration("wrist_usb_port"),
            "global_camera_name": LaunchConfiguration("global_camera_name"),
            "global_serial_no": LaunchConfiguration("global_serial_no"),
            "global_usb_port_id": LaunchConfiguration("global_usb_port_id"),
            "global_color_profile": LaunchConfiguration("global_color_profile"),
            "global_enable_depth": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )
    policy = Node(
        package="dobot_policy",
        executable="dobot_policy_node",
        name="dobot_policy_node",
        output="screen",
        prefix=[LaunchConfiguration("policy_python")],
        parameters=[
            LaunchConfiguration("policy_params_file"),
            {
                "armed": ParameterValue(
                    LaunchConfiguration("armed"), value_type=bool
                ),
                "auto_start": ParameterValue(
                    LaunchConfiguration("auto_start"), value_type=bool
                ),
                "auto_enable_robot": ParameterValue(
                    LaunchConfiguration("auto_enable_robot"), value_type=bool
                ),
                "auto_init_gripper": ParameterValue(
                    LaunchConfiguration("auto_init_gripper"), value_type=bool
                ),
                "gripper_enabled": ParameterValue(
                    LaunchConfiguration("gripper_enabled"), value_type=bool
                ),
                "motion_test_duration_sec": ParameterValue(
                    LaunchConfiguration("motion_test_duration_sec"), value_type=float
                ),
                "max_episode_sec": ParameterValue(
                    LaunchConfiguration("max_episode_sec"), value_type=float
                ),
                "policy_host": LaunchConfiguration("policy_host"),
                "policy_port": ParameterValue(
                    LaunchConfiguration("policy_port"), value_type=int
                ),
            },
        ],
    )
    return LaunchDescription(declarations + [robot, cameras, policy])
