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
            DeclareLaunchArgument("deadman_button_index", default_value="4"),
            DeclareLaunchArgument("estop_button_index", default_value="1"),
            DeclareLaunchArgument("toggle_gripper_button_index", default_value="0"),
            DeclareLaunchArgument("clear_error_button_index", default_value="2"),
            DeclareLaunchArgument("deadzone", default_value="0.25"),
            DeclareLaunchArgument("coord_type", default_value="0"),
            DeclareLaunchArgument("x_axis_sign", default_value="-1.0"),
            DeclareLaunchArgument("y_axis_sign", default_value="1.0"),
            DeclareLaunchArgument("z_axis_sign", default_value="-1.0"),
            DeclareLaunchArgument("rz_axis_sign", default_value="1.0"),
            DeclareLaunchArgument("gripper_step_mm", default_value="2.0"),
            DeclareLaunchArgument("gripper_force_percent", default_value="50"),
            DeclareLaunchArgument("enable_rumble", default_value="true"),
            DeclareLaunchArgument("joint_limit_margin_deg", default_value="5.0"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"dev": LaunchConfiguration("dev")}],
                condition=IfCondition(LaunchConfiguration("start_joy_node")),
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
                        "joy.clear_error_button_index": ParameterValue(
                            LaunchConfiguration("clear_error_button_index"),
                            value_type=int,
                        ),
                        "joy.deadzone": ParameterValue(
                            LaunchConfiguration("deadzone"), value_type=float
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
                        "joy.coord_type": ParameterValue(
                            LaunchConfiguration("coord_type"), value_type=int
                        ),
                        "joy.gripper_step_mm": ParameterValue(
                            LaunchConfiguration("gripper_step_mm"), value_type=float
                        ),
                        "joy.gripper_force_percent": ParameterValue(
                            LaunchConfiguration("gripper_force_percent"), value_type=int
                        ),
                        "joy.enable_rumble": ParameterValue(
                            LaunchConfiguration("enable_rumble"), value_type=bool
                        ),
                        "joy.joint_limit_margin_deg": ParameterValue(
                            LaunchConfiguration("joint_limit_margin_deg"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
