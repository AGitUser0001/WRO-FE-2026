import fcntl
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


_launch_lock = None


def _acquire_launch_lock() -> None:
    global _launch_lock
    if _launch_lock is not None:
        return
    lock = open("/tmp/robot-planner.launch.lock", "w", encoding="ascii")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock.close()
        raise RuntimeError("another robot planner launch is already running") from error
    _launch_lock = lock


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
    _acquire_launch_lock()
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
            "simulator_seed": LaunchConfiguration("simulator_seed"),
            "start_micro_ros": LaunchConfiguration("start_micro_ros"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "odometry_linear_scale": LaunchConfiguration("odometry_linear_scale"),
            "odometry_right_scale": LaunchConfiguration("odometry_right_scale"),
            "odometry_left_scale": LaunchConfiguration("odometry_left_scale"),
            "noise_std_m": LaunchConfiguration("noise_std_m"),
            "random_projection_ratio": LaunchConfiguration("random_projection_ratio"),
            "imu_yaw_scale": LaunchConfiguration("imu_yaw_scale"),
            "steering_effectiveness": LaunchConfiguration("steering_effectiveness"),
            "odometry_linear_drift_amplitude": LaunchConfiguration("odometry_linear_drift_amplitude"),
            "odometry_yaw_drift_amplitude": LaunchConfiguration("odometry_yaw_drift_amplitude"),
            "odometry_drift_period_s": LaunchConfiguration("odometry_drift_period_s"),
            "motor_speed_scale": LaunchConfiguration("motor_speed_scale"),
            "motor_speed_drift_amplitude": LaunchConfiguration("motor_speed_drift_amplitude"),
            "motor_accel_tau_s": LaunchConfiguration("motor_accel_tau_s"),
            "motor_decel_tau_s": LaunchConfiguration("motor_decel_tau_s"),
            "motor_deadband_command": LaunchConfiguration("motor_deadband_command"),
            "planning_motor_reference": LaunchConfiguration("planning_motor_reference"),
            "sim_command_delay_s": LaunchConfiguration("sim_command_delay_s"),
            "lidar_range_scale": LaunchConfiguration("lidar_range_scale"),
            "lidar_range_drift_amplitude": LaunchConfiguration("lidar_range_drift_amplitude"),
            "lidar_ghost_enabled": LaunchConfiguration("lidar_ghost_enabled"),
            "lidar_ghost_band_width_deg": LaunchConfiguration("lidar_ghost_band_width_deg"),
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
        DeclareLaunchArgument("simulator_seed", default_value="-1"),
        DeclareLaunchArgument("start_micro_ros", default_value="true"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("odometry_linear_scale", default_value="0.95"),
        DeclareLaunchArgument("odometry_right_scale", default_value="0.85"),
        DeclareLaunchArgument("odometry_left_scale", default_value="1.10"),
        DeclareLaunchArgument("noise_std_m", default_value="0.03"),
        DeclareLaunchArgument("random_projection_ratio", default_value="0.10"),
        DeclareLaunchArgument("imu_yaw_scale", default_value="1.0"),
        DeclareLaunchArgument("steering_effectiveness", default_value="1.0"),
        DeclareLaunchArgument("odometry_linear_drift_amplitude", default_value="0.08"),
        DeclareLaunchArgument("odometry_yaw_drift_amplitude", default_value="0.04"),
        DeclareLaunchArgument("odometry_drift_period_s", default_value="45.0"),
        DeclareLaunchArgument("motor_speed_scale", default_value="0.912"),
        DeclareLaunchArgument("motor_speed_drift_amplitude", default_value="0.05"),
        DeclareLaunchArgument("motor_accel_tau_s", default_value="0.215"),
        DeclareLaunchArgument("motor_decel_tau_s", default_value="0.354"),
        DeclareLaunchArgument("sim_command_delay_s", default_value="0.15"),
        DeclareLaunchArgument("lidar_range_scale", default_value="1.0"),
        DeclareLaunchArgument("lidar_range_drift_amplitude", default_value="0.015"),
        DeclareLaunchArgument("lidar_ghost_enabled", default_value="true"),
        DeclareLaunchArgument("lidar_ghost_band_width_deg", default_value="8.0"),
        *[
            DeclareLaunchArgument(name, default_value=str(default).lower() if isinstance(default, bool) else str(default))
            for name, default in PARAMETER_DEFAULTS
        ],

        SetEnvironmentVariable("DISPLAY", LaunchConfiguration("display")),
        SetEnvironmentVariable("OPENBLAS_NUM_THREADS", "1"),
        SetEnvironmentVariable("OMP_NUM_THREADS", "1"),
        prereq,
        TimerAction(period=3.0, actions=[driver]),
    ])
