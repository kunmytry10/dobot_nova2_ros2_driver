from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_share = FindPackageShare("dobot_camera")
    wrist_launch = PathJoinSubstitution(
        [camera_share, "launch", "gemini305.launch.py"]
    )
    global_launch = PathJoinSubstitution(
        [camera_share, "launch", "realsense_global.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "orbbec_launch_file", default_value="gemini_330_series.launch.py"
            ),
            DeclareLaunchArgument("wrist_camera_name", default_value="camera"),
            DeclareLaunchArgument("wrist_serial_number", default_value=""),
            DeclareLaunchArgument("wrist_usb_port", default_value=""),
            DeclareLaunchArgument("global_camera_name", default_value="global_camera"),
            DeclareLaunchArgument("global_serial_no", default_value="''"),
            DeclareLaunchArgument("global_usb_port_id", default_value="''"),
            DeclareLaunchArgument("global_color_profile", default_value="640,480,30"),
            DeclareLaunchArgument("global_enable_depth", default_value="false"),
            GroupAction(
                scoped=True,
                forwarding=False,
                launch_configurations={
                    "orbbec_launch_file": LaunchConfiguration(
                        "orbbec_launch_file"
                    ),
                    "camera_name": LaunchConfiguration("wrist_camera_name"),
                    "serial_number": LaunchConfiguration("wrist_serial_number"),
                    "usb_port": LaunchConfiguration("wrist_usb_port"),
                },
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(wrist_launch)
                    )
                ],
            ),
            GroupAction(
                scoped=True,
                forwarding=False,
                launch_configurations={
                    "camera_namespace": "",
                    "camera_name": LaunchConfiguration("global_camera_name"),
                    "serial_no": LaunchConfiguration("global_serial_no"),
                    "usb_port_id": LaunchConfiguration("global_usb_port_id"),
                    "rgb_camera.color_profile": LaunchConfiguration(
                        "global_color_profile"
                    ),
                    "enable_depth": LaunchConfiguration("global_enable_depth"),
                    "pointcloud.enable": "false",
                },
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(global_launch)
                    )
                ],
            ),
        ]
    )
