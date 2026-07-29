from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsense_launch = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_namespace", default_value=""),
            DeclareLaunchArgument("camera_name", default_value="global_camera"),
            DeclareLaunchArgument("serial_no", default_value="''"),
            DeclareLaunchArgument("usb_port_id", default_value="''"),
            DeclareLaunchArgument(
                "rgb_camera.color_profile", default_value="640,480,30"
            ),
            DeclareLaunchArgument("enable_depth", default_value="false"),
            DeclareLaunchArgument("pointcloud.enable", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch),
                launch_arguments=[
                    ("camera_namespace", LaunchConfiguration("camera_namespace")),
                    ("camera_name", LaunchConfiguration("camera_name")),
                    ("serial_no", LaunchConfiguration("serial_no")),
                    ("usb_port_id", LaunchConfiguration("usb_port_id")),
                    ("enable_color", "true"),
                    (
                        "rgb_camera.color_profile",
                        LaunchConfiguration("rgb_camera.color_profile"),
                    ),
                    ("enable_depth", LaunchConfiguration("enable_depth")),
                    (
                        "pointcloud.enable",
                        LaunchConfiguration("pointcloud.enable"),
                    ),
                ],
            ),
        ]
    )
