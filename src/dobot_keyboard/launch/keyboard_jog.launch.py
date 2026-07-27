from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    params_file = LaunchConfiguration("params_file").perform(context)
    teleop_parameters = [
        {
            "keyboard.input_topic": LaunchConfiguration("input_topic"),
            "keyboard.mode": "jog",
            "keyboard.jog_coord_type": ParameterValue(
                LaunchConfiguration("jog_coord_type"), value_type=int
            ),
            "keyboard.user": ParameterValue(LaunchConfiguration("user"), value_type=int),
            "keyboard.tool": ParameterValue(LaunchConfiguration("tool"), value_type=int),
        }
    ]
    if params_file and Path(params_file).is_file():
        teleop_parameters.insert(0, params_file)

    return [
        Node(
            package="dobot_keyboard",
            executable="dobot_keyboard_teleop",
            name="dobot_keyboard_teleop",
            output="screen",
            parameters=teleop_parameters,
        ),
        Node(
            package="dobot_keyboard",
            executable="dobot_keyboard_jog_input",
            name="dobot_keyboard_jog_input",
            output="screen",
            parameters=[
                {
                    "input_topic": LaunchConfiguration("input_topic"),
                    "device": LaunchConfiguration("device"),
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("input_topic", default_value="/keyboard/input"),
            DeclareLaunchArgument("device", default_value="/dev/input/event0"),
            DeclareLaunchArgument("jog_coord_type", default_value="0"),
            DeclareLaunchArgument("user", default_value="0"),
            DeclareLaunchArgument("tool", default_value="0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
