from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    orbbec_launch = PathJoinSubstitution(
        [
            FindPackageShare("orbbec_camera"),
            "launch",
            LaunchConfiguration("orbbec_launch_file"),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "orbbec_launch_file",
                default_value="gemini_330_series.launch.py",
            ),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("serial_number", default_value=""),
            DeclareLaunchArgument("usb_port", default_value=""),
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(orbbec_launch),
                launch_arguments=[
                    ("camera_name", LaunchConfiguration("camera_name")),
                    ("serial_number", LaunchConfiguration("serial_number")),
                    ("usb_port", LaunchConfiguration("usb_port")),
                    ("enable_color", LaunchConfiguration("enable_color")),
                    ("enable_depth", LaunchConfiguration("enable_depth")),
                    ("enable_point_cloud", LaunchConfiguration("enable_point_cloud")),
                ],
            ),
        ]
    )
