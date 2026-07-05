from __future__ import annotations

from dataclasses import dataclass

from rclpy.node import Node


@dataclass(frozen=True)
class DriverConfig:
    scan_topic: str = "/scan"
    imu_topic: str = "/imu_data"
    motor_topic: str = "/microROS/motor_control"
    servo_topic: str = "/microROS/servo_control"
    drive_motor: int = 0
    max_servo: int = 320
    grid_size_m: float = 8.0
    grid_resolution_m: float = 0.05
    local_grid_size_m: float = 1.5
    lidar_max_range_m: float = 2.2
    safety_buffer_m: float = 0.15
    lookahead_m: float = 1.2
    steer_kp: float = 500.0
    wheelbase_m: float = 0.138
    max_steering_angle_rad: float = 0.4188
    robot_length_m: float = 0.25
    robot_width_m: float = 0.15
    trajectory_steps: int = 100
    trajectory_length_m: float = 1.5
    trajectory_step_m: float = 0.05
    command_delay_s: float = 0.5
    planning_speed_mps: float = 0.35
    start_yaw_rad: float = 1.5707963267948966
    correction_alpha: float = 0.04
    fuse_add_alpha: float = 0.08
    fuse_erode_alpha: float = 0.02
    front_wall_motion_alpha: float = 0.85
    front_wall_motion_max_step_m: float = 0.12
    front_wall_motion_min_points: int = 5
    wall_pose_correction_alpha: float = 0.80
    wall_pose_correction_max_step_m: float = 0.50
    wall_pose_max_slope: float = 0.35
    wall_yaw_correction_alpha: float = 0.0
    wall_yaw_correction_max_step_rad: float = 0.05235987755982989
    direction_lock_confirm_frames: int = 4
    direction_lock_min_score_delta: float = 0.10
    front_backup_distance_m: float = 0.26
    reverse_hold_seconds: float = 0.8
    tick_rate: float = 20.0
    auto_drive_enabled: bool = False
    debug_view_enabled: bool = False
    pose_debug: bool = False


PARAMETER_DEFAULTS: tuple[tuple[str, str | int | float | bool], ...] = (
    ("scan_topic", DriverConfig.scan_topic),
    ("imu_topic", DriverConfig.imu_topic),
    ("motor_topic", DriverConfig.motor_topic),
    ("servo_topic", DriverConfig.servo_topic),
    ("drive_motor", DriverConfig.drive_motor),
    ("max_servo", DriverConfig.max_servo),
    ("grid_size_m", DriverConfig.grid_size_m),
    ("grid_resolution_m", DriverConfig.grid_resolution_m),
    ("local_grid_size_m", DriverConfig.local_grid_size_m),
    ("lidar_max_range_m", DriverConfig.lidar_max_range_m),
    ("safety_buffer_m", DriverConfig.safety_buffer_m),
    ("lookahead_m", DriverConfig.lookahead_m),
    ("steer_kp", DriverConfig.steer_kp),
    ("wheelbase_m", DriverConfig.wheelbase_m),
    ("max_steering_angle_rad", DriverConfig.max_steering_angle_rad),
    ("robot_length_m", DriverConfig.robot_length_m),
    ("robot_width_m", DriverConfig.robot_width_m),
    ("trajectory_steps", DriverConfig.trajectory_steps),
    ("trajectory_length_m", DriverConfig.trajectory_length_m),
    ("trajectory_step_m", DriverConfig.trajectory_step_m),
    ("command_delay_s", DriverConfig.command_delay_s),
    ("planning_speed_mps", DriverConfig.planning_speed_mps),
    ("start_yaw_rad", DriverConfig.start_yaw_rad),
    ("correction_alpha", DriverConfig.correction_alpha),
    ("fuse_add_alpha", DriverConfig.fuse_add_alpha),
    ("fuse_erode_alpha", DriverConfig.fuse_erode_alpha),
    ("front_wall_motion_alpha", DriverConfig.front_wall_motion_alpha),
    ("front_wall_motion_max_step_m", DriverConfig.front_wall_motion_max_step_m),
    ("front_wall_motion_min_points", DriverConfig.front_wall_motion_min_points),
    ("wall_pose_correction_alpha", DriverConfig.wall_pose_correction_alpha),
    ("wall_pose_correction_max_step_m", DriverConfig.wall_pose_correction_max_step_m),
    ("wall_pose_max_slope", DriverConfig.wall_pose_max_slope),
    ("wall_yaw_correction_alpha", DriverConfig.wall_yaw_correction_alpha),
    ("wall_yaw_correction_max_step_rad", DriverConfig.wall_yaw_correction_max_step_rad),
    ("direction_lock_confirm_frames", DriverConfig.direction_lock_confirm_frames),
    ("direction_lock_min_score_delta", DriverConfig.direction_lock_min_score_delta),
    ("front_backup_distance_m", DriverConfig.front_backup_distance_m),
    ("reverse_hold_seconds", DriverConfig.reverse_hold_seconds),
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
        imu_topic=_str_param(node, "imu_topic"),
        motor_topic=_str_param(node, "motor_topic"),
        servo_topic=_str_param(node, "servo_topic"),
        drive_motor=_int_param(node, "drive_motor"),
        max_servo=_int_param(node, "max_servo"),
        grid_size_m=_float_param(node, "grid_size_m"),
        grid_resolution_m=_float_param(node, "grid_resolution_m"),
        local_grid_size_m=_float_param(node, "local_grid_size_m"),
        lidar_max_range_m=_float_param(node, "lidar_max_range_m"),
        safety_buffer_m=_float_param(node, "safety_buffer_m"),
        lookahead_m=_float_param(node, "lookahead_m"),
        steer_kp=_float_param(node, "steer_kp"),
        wheelbase_m=_float_param(node, "wheelbase_m"),
        max_steering_angle_rad=_float_param(node, "max_steering_angle_rad"),
        robot_length_m=_float_param(node, "robot_length_m"),
        robot_width_m=_float_param(node, "robot_width_m"),
        trajectory_steps=_int_param(node, "trajectory_steps"),
        trajectory_length_m=_float_param(node, "trajectory_length_m"),
        trajectory_step_m=_float_param(node, "trajectory_step_m"),
        command_delay_s=_float_param(node, "command_delay_s"),
        planning_speed_mps=_float_param(node, "planning_speed_mps"),
        start_yaw_rad=_float_param(node, "start_yaw_rad"),
        correction_alpha=_float_param(node, "correction_alpha"),
        fuse_add_alpha=_float_param(node, "fuse_add_alpha"),
        fuse_erode_alpha=_float_param(node, "fuse_erode_alpha"),
        front_wall_motion_alpha=_float_param(node, "front_wall_motion_alpha"),
        front_wall_motion_max_step_m=_float_param(node, "front_wall_motion_max_step_m"),
        front_wall_motion_min_points=_int_param(node, "front_wall_motion_min_points"),
        wall_pose_correction_alpha=_float_param(node, "wall_pose_correction_alpha"),
        wall_pose_correction_max_step_m=_float_param(node, "wall_pose_correction_max_step_m"),
        wall_pose_max_slope=_float_param(node, "wall_pose_max_slope"),
        wall_yaw_correction_alpha=_float_param(node, "wall_yaw_correction_alpha"),
        wall_yaw_correction_max_step_rad=_float_param(node, "wall_yaw_correction_max_step_rad"),
        direction_lock_confirm_frames=_int_param(node, "direction_lock_confirm_frames"),
        direction_lock_min_score_delta=_float_param(node, "direction_lock_min_score_delta"),
        front_backup_distance_m=_float_param(node, "front_backup_distance_m"),
        reverse_hold_seconds=_float_param(node, "reverse_hold_seconds"),
        tick_rate=_float_param(node, "tick_rate"),
        auto_drive_enabled=_bool_param(node, "auto_drive_enabled"),
        debug_view_enabled=_bool_param(node, "debug_view_enabled"),
        pose_debug=_bool_param(node, "pose_debug"),
    )
