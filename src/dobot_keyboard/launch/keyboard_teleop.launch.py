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
            "keyboard.translation_step_mm": ParameterValue(
                LaunchConfiguration("translation_step_mm"), value_type=float
            ),
            "keyboard.rotation_step_deg": ParameterValue(
                LaunchConfiguration("rotation_step_deg"), value_type=float
            ),
            "keyboard.motion_service": LaunchConfiguration("motion_service"),
            "keyboard.speed": ParameterValue(LaunchConfiguration("speed"), value_type=int),
            "keyboard.acceleration": ParameterValue(
                LaunchConfiguration("acceleration"), value_type=int
            ),
            "keyboard.wait": ParameterValue(LaunchConfiguration("wait"), value_type=bool),
            "keyboard.timeout_sec": ParameterValue(
                LaunchConfiguration("timeout_sec"), value_type=float
            ),
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
            executable="dobot_keyboard_input",
            name="dobot_keyboard_input",
            output="screen",
            emulate_tty=True,
            parameters=[{"input_topic": LaunchConfiguration("input_topic")}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("input_topic", default_value="/keyboard/input"),
            DeclareLaunchArgument("translation_step_mm", default_value="5.0"),
            DeclareLaunchArgument("rotation_step_deg", default_value="2.0"),
            DeclareLaunchArgument("motion_service", default_value="movep"),
            DeclareLaunchArgument("speed", default_value="2"),
            DeclareLaunchArgument("acceleration", default_value="2"),
            DeclareLaunchArgument("wait", default_value="true"),
            DeclareLaunchArgument("timeout_sec", default_value="20.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
