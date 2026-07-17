from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dobot_ros2"), "config", "dobot_ros2.yaml"]
                ),
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            Node(
                package="dobot_ros2",
                executable="dobot_motion_server",
                name="dobot_motion_server",
                namespace=namespace,
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
