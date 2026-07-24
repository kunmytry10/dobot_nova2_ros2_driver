from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_launch = PathJoinSubstitution(
        [FindPackageShare("dobot_ros2"), "launch", "dobot_bringup.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dobot_ros2"), "config", "dobot_ros2.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dobot_description"), "urdf", "nova2_robot.urdf"]
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("handeye_tf", default_value="true"),
            DeclareLaunchArgument("handeye_result_file", default_value=""),
            DeclareLaunchArgument("handeye_output_child_frame", default_value="camera_link"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dobot_ros2"), "rviz", "nova2.rviz"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments=[
                    ("namespace", LaunchConfiguration("namespace")),
                    ("params_file", LaunchConfiguration("params_file")),
                    ("urdf_file", LaunchConfiguration("urdf_file")),
                    ("rviz", LaunchConfiguration("rviz")),
                    ("rviz_config", LaunchConfiguration("rviz_config")),
                    ("handeye_tf", LaunchConfiguration("handeye_tf")),
                    ("handeye_result_file", LaunchConfiguration("handeye_result_file")),
                    (
                        "handeye_output_child_frame",
                        LaunchConfiguration("handeye_output_child_frame"),
                    ),
                ],
            ),
        ]
    )
