from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
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

    simulator = Node(
        package="robot",
        executable="wro_planner_simulator.py",
        name="wro_planner_io_simulator",
        output="screen",
        parameters=[{"view_enabled": LaunchConfiguration("sim_view_enabled")}],
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("sim_view_enabled", default_value="true"),
        DeclareLaunchArgument("start_micro_ros", default_value="false"),

        set_ros_domain,
        set_xrce_domain,
        urdf,

        TimerAction(period=1.0, actions=[camera], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[lidars], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=2.0, actions=[laser_merge], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[imu], condition=UnlessCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[simulator], condition=IfCondition(LaunchConfiguration("sim"))),
        TimerAction(period=1.0, actions=[micro_ros]),
    ])
