from __future__ import annotations

from dataclasses import dataclass

from rclpy.node import Node


@dataclass(frozen=True)
class DriverConfig:
    scan_topic: str = "/scan"
    odom_topic: str = "/wheel/odometry"
    imu_topic: str = "/imu_data"
    motor_topic: str = "/microROS/motor_control"
    servo_topic: str = "/microROS/servo_control"
    camera_image_topic: str = "/camera/camera/color/image_raw"
    camera_depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw"
    color_obstacle_hold_s: float = 8.0
    drive_motor: int = 0
    motor_deadband_command: int = 90
    planning_motor_reference: int = 150
    max_servo: int = 320
    grid_size_m: float = 6.0
    grid_resolution_m: float = 0.05
    local_grid_size_m: float = 2.4
    lidar_max_range_m: float = 2.2
    wheelbase_m: float = 0.138
    max_steering_angle_rad: float = 0.2531
    robot_length_m: float = 0.22
    robot_width_m: float = 0.15
    trajectory_step_m: float = 0.05
    command_delay_s: float = 0.25
    actuation_delay_s: float = 0.15
    planning_speed_mps: float = 0.319
    planning_accel_tau_s: float = 0.215
    planning_decel_tau_s: float = 0.354
    course_laps: int = 3
    start_yaw_rad: float = 1.5707963267948966
    odometry_correction_max_m: float = 0.30
    fuse_add_alpha: float = 0.08
    fuse_erode_alpha: float = 0.02
    direction_min_wall_points: int = 5
    direction_lock_confirm_frames: int = 3
    direction_lock_min_score_delta: float = 0.18
    tick_rate: float = 20.0
    auto_drive_enabled: bool = False
    debug_view_enabled: bool = False
    pose_debug: bool = False


PARAMETER_DEFAULTS: tuple[tuple[str, str | int | float | bool], ...] = (
    ("scan_topic", DriverConfig.scan_topic),
    ("odom_topic", DriverConfig.odom_topic),
    ("imu_topic", DriverConfig.imu_topic),
    ("motor_topic", DriverConfig.motor_topic),
    ("servo_topic", DriverConfig.servo_topic),
    ("camera_image_topic", DriverConfig.camera_image_topic),
    ("camera_depth_topic", DriverConfig.camera_depth_topic),
    ("color_obstacle_hold_s", DriverConfig.color_obstacle_hold_s),
    ("drive_motor", DriverConfig.drive_motor),
    ("motor_deadband_command", DriverConfig.motor_deadband_command),
    ("planning_motor_reference", DriverConfig.planning_motor_reference),
    ("max_servo", DriverConfig.max_servo),
    ("grid_size_m", DriverConfig.grid_size_m),
    ("grid_resolution_m", DriverConfig.grid_resolution_m),
    ("local_grid_size_m", DriverConfig.local_grid_size_m),
    ("lidar_max_range_m", DriverConfig.lidar_max_range_m),
    ("wheelbase_m", DriverConfig.wheelbase_m),
    ("max_steering_angle_rad", DriverConfig.max_steering_angle_rad),
    ("robot_length_m", DriverConfig.robot_length_m),
    ("robot_width_m", DriverConfig.robot_width_m),
    ("trajectory_step_m", DriverConfig.trajectory_step_m),
    ("command_delay_s", DriverConfig.command_delay_s),
    ("actuation_delay_s", DriverConfig.actuation_delay_s),
    ("planning_speed_mps", DriverConfig.planning_speed_mps),
    ("planning_accel_tau_s", DriverConfig.planning_accel_tau_s),
    ("planning_decel_tau_s", DriverConfig.planning_decel_tau_s),
    ("course_laps", DriverConfig.course_laps),
    ("start_yaw_rad", DriverConfig.start_yaw_rad),
    ("odometry_correction_max_m", DriverConfig.odometry_correction_max_m),
    ("fuse_add_alpha", DriverConfig.fuse_add_alpha),
    ("fuse_erode_alpha", DriverConfig.fuse_erode_alpha),
    ("direction_min_wall_points", DriverConfig.direction_min_wall_points),
    ("direction_lock_confirm_frames", DriverConfig.direction_lock_confirm_frames),
    ("direction_lock_min_score_delta", DriverConfig.direction_lock_min_score_delta),
    ("tick_rate", DriverConfig.tick_rate),
    ("auto_drive_enabled", DriverConfig.auto_drive_enabled),
    ("debug_view_enabled", DriverConfig.debug_view_enabled),
    ("pose_debug", DriverConfig.pose_debug),
)


def _str_param(node: Node, name: str) -> str:
    value = node.get_parameter(name).value
    if not isinstance(value, str):
        raise TypeError(f"parameter {name} must be str")
    return value


def _int_param(node: Node, name: str) -> int:
    value = node.get_parameter(name).value
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"parameter {name} must be int")
    return value


def _float_param(node: Node, name: str) -> float:
    value = node.get_parameter(name).value
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise TypeError(f"parameter {name} must be float")
    return float(value)


def _bool_param(node: Node, name: str) -> bool:
    value = node.get_parameter(name).value
    if not isinstance(value, bool):
        raise TypeError(f"parameter {name} must be bool")
    return value


def declare_and_load_config(node: Node) -> DriverConfig:
    for name, default in PARAMETER_DEFAULTS:
        node.declare_parameter(name, default)

    return DriverConfig(
        scan_topic=_str_param(node, "scan_topic"),
        odom_topic=_str_param(node, "odom_topic"),
        imu_topic=_str_param(node, "imu_topic"),
        motor_topic=_str_param(node, "motor_topic"),
        servo_topic=_str_param(node, "servo_topic"),
        camera_image_topic=_str_param(node, "camera_image_topic"),
        camera_depth_topic=_str_param(node, "camera_depth_topic"),
        color_obstacle_hold_s=_float_param(node, "color_obstacle_hold_s"),
        drive_motor=_int_param(node, "drive_motor"),
        motor_deadband_command=_int_param(node, "motor_deadband_command"),
        planning_motor_reference=_int_param(node, "planning_motor_reference"),
        max_servo=_int_param(node, "max_servo"),
        grid_size_m=_float_param(node, "grid_size_m"),
        grid_resolution_m=_float_param(node, "grid_resolution_m"),
        local_grid_size_m=_float_param(node, "local_grid_size_m"),
        lidar_max_range_m=_float_param(node, "lidar_max_range_m"),
        wheelbase_m=_float_param(node, "wheelbase_m"),
        max_steering_angle_rad=_float_param(node, "max_steering_angle_rad"),
        robot_length_m=_float_param(node, "robot_length_m"),
        robot_width_m=_float_param(node, "robot_width_m"),
        trajectory_step_m=_float_param(node, "trajectory_step_m"),
        command_delay_s=_float_param(node, "command_delay_s"),
        actuation_delay_s=_float_param(node, "actuation_delay_s"),
        planning_speed_mps=_float_param(node, "planning_speed_mps"),
        planning_accel_tau_s=_float_param(node, "planning_accel_tau_s"),
        planning_decel_tau_s=_float_param(node, "planning_decel_tau_s"),
        course_laps=_int_param(node, "course_laps"),
        start_yaw_rad=_float_param(node, "start_yaw_rad"),
        odometry_correction_max_m=_float_param(node, "odometry_correction_max_m"),
        fuse_add_alpha=_float_param(node, "fuse_add_alpha"),
        fuse_erode_alpha=_float_param(node, "fuse_erode_alpha"),
        direction_min_wall_points=_int_param(node, "direction_min_wall_points"),
        direction_lock_confirm_frames=_int_param(node, "direction_lock_confirm_frames"),
        direction_lock_min_score_delta=_float_param(node, "direction_lock_min_score_delta"),
        tick_rate=_float_param(node, "tick_rate"),
        auto_drive_enabled=_bool_param(node, "auto_drive_enabled"),
        debug_view_enabled=_bool_param(node, "debug_view_enabled"),
        pose_debug=_bool_param(node, "pose_debug"),
    )
