from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wrist_image_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "global_image_topic",
                default_value="/global_camera/color/image_raw",
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="wrist_camera_view",
                arguments=[LaunchConfiguration("wrist_image_topic")],
                output="screen",
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="global_camera_view",
                arguments=[LaunchConfiguration("global_image_topic")],
                output="screen",
            ),
        ]
    )
