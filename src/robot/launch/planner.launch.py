import sys
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _add_planner_to_path() -> None:
    source_scripts = Path(__file__).resolve().parents[1]
    paths = [source_scripts]
    try:
        paths.append(Path(get_package_prefix("robot")) / "lib" / "robot")
    except PackageNotFoundError:
        pass
    for path in paths:
        if (path / "planner" / "config.py").exists():
            sys.path.insert(0, str(path))
            return


_add_planner_to_path()
from planner.config import PARAMETER_DEFAULTS # pyright: ignore[reportMissingImports]


def _launch_parameter(name: str, default: str | int | float | bool):
    value = LaunchConfiguration(name)
    if isinstance(default, bool):
        return ParameterValue(value, value_type=bool)
    if isinstance(default, int):
        return ParameterValue(value, value_type=int)
    if isinstance(default, float):
        return ParameterValue(value, value_type=float)
    return value


def generate_launch_description():
    inherited_parameters = {
        name: _launch_parameter(name, default)
        for name, default in PARAMETER_DEFAULTS
    }

    prereq = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot"),
                "launch",
                "reactive_prereq.launch.py",
            ])
        ),
        launch_arguments={
            "sim": LaunchConfiguration("sim"),
            "sim_view_enabled": LaunchConfiguration("sim_view_enabled"),
            "start_micro_ros": LaunchConfiguration("start_micro_ros"),
        }.items(),
    )

    driver = Node(
        package="robot",
        executable="wro_planner.py",
        name="wro_planner",
        output="screen",
        parameters=[inherited_parameters],
        condition=IfCondition(LaunchConfiguration("start_driver")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("display", default_value=":0"),
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("sim_view_enabled", default_value="true"),
        DeclareLaunchArgument("start_micro_ros", default_value="true"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        *[
            DeclareLaunchArgument(name, default_value=str(default).lower() if isinstance(default, bool) else str(default))
            for name, default in PARAMETER_DEFAULTS
        ],

        SetEnvironmentVariable("DISPLAY", LaunchConfiguration("display")),
        prereq,
        TimerAction(period=3.0, actions=[driver]),
    ])
