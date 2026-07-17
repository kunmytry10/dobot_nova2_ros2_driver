from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    urdf_file = LaunchConfiguration("urdf_file").perform(context)

    with open(urdf_file, "r", encoding="utf-8") as file:
        robot_description = file.read()

    return [
        Node(
            package="dobot_ros2",
            executable="dobot_motion_server",
            name="dobot_motion_server",
            namespace=namespace,
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("dobot_ros2"), "config", "dobot_ros2.yaml"]
    )
    default_urdf = PathJoinSubstitution(
        [FindPackageShare("dobot_description"), "urdf", "nova2_robot.urdf"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("dobot_ros2"), "rviz", "nova2.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("urdf_file", default_value=default_urdf),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            OpaqueFunction(function=_launch_setup),
        ]
    )
