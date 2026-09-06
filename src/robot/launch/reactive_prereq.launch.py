from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    xrce_domain_id = os.environ.get("XRCE_DOMAIN_ID_OVERRIDE", ros_domain_id)

    set_ros_domain = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID",
        value=ros_domain_id,
    )
    set_xrce_domain = SetEnvironmentVariable(
        name="XRCE_DOMAIN_ID_OVERRIDE",
        value=xrce_domain_id,
    )

    urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot"),
                "launch",
                "urdf.launch.py",
            ])
        )
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_sync": "true",
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
        }.items(),
    )

    lidars = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ldlidar_ros2"),
                "launch",
                "lidars.launch.py",
            ])
        )
    )

    laser_merge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("dual_laser_merger"),
                "laser_merger.launch.py",
            ])
        )
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("tm_imu"),
                "launch",
                "imu.launch.py",
            ])
        )
    )

    micro_ros = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=["udp4", "--port", "8888"],
        respawn=True,
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration("sim"),
                "' != 'true' and '",
                LaunchConfiguration("start_micro_ros"),
                "' == 'true'",
            ])
        ),
    )


    odom_udp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hiway_odom"),
                "launch",
                "odom_udp.launch.py",
            ])
        )
    )

    simulator = Node(
        package="robot",
        executable="wro_planner_simulator.py",
        name="wro_planner_io_simulator",
        output="screen",
        parameters=[{
            "view_enabled": LaunchConfiguration("sim_view_enabled"),
            "simulator_seed": LaunchConfiguration("simulator_seed"),
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
            "command_delay_s": LaunchConfiguration("sim_command_delay_s"),
            "lidar_range_scale": LaunchConfiguration("lidar_range_scale"),
            "lidar_range_drift_amplitude": LaunchConfiguration("lidar_range_drift_amplitude"),
            "lidar_ghost_enabled": LaunchConfiguration("lidar_ghost_enabled"),
            "lidar_ghost_band_width_deg": LaunchConfiguration("lidar_ghost_band_width_deg"),
        }],
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("sim_view_enabled", default_value="true"),
        DeclareLaunchArgument("simulator_seed", default_value="-1"),
        DeclareLaunchArgument("start_micro_ros", default_value="false"),
        DeclareLaunchArgument("odom_topic", default_value="/wheel/odometry"),
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
        DeclareLaunchArgument("motor_deadband_command", default_value="90"),
        DeclareLaunchArgument("planning_motor_reference", default_value="150"),
        DeclareLaunchArgument("sim_command_delay_s", default_value="0.15"),
        DeclareLaunchArgument("lidar_range_scale", default_value="1.0"),
        DeclareLaunchArgument("lidar_range_drift_amplitude", default_value="0.015"),
        DeclareLaunchArgument("lidar_ghost_enabled", default_value="true"),
        DeclareLaunchArgument("lidar_ghost_band_width_deg", default_value="8.0"),

        set_ros_domain,
        set_xrce_domain,
        urdf,

        TimerAction(period=1.0, actions=[camera], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[lidars], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=2.0, actions=[laser_merge], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[imu], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[odom_udp], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[simulator], condition=IfCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[micro_ros]),
    ])
