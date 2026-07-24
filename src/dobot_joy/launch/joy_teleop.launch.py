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
            DeclareLaunchArgument("deadzone", default_value="0.25"),
            DeclareLaunchArgument("coord_type", default_value="0"),
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
                        "joy.deadzone": ParameterValue(
                            LaunchConfiguration("deadzone"), value_type=float
                        ),
                        "joy.coord_type": ParameterValue(
                            LaunchConfiguration("coord_type"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
