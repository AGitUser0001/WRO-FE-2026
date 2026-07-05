from __future__ import annotations

import rclpy

from .config import DriverConfig, declare_and_load_config
from .debug import DebugFrame, DebugStyle, DebugWindow
from .driver import Driver
from .grid import Cell, Direction, GridMap, LocalGrid
from .localize import Localizer, PoseEstimate
from .planner import GridPlanner, Plan
from .sensors import FrontWallEstimate, ImuReading, ScanPoint, SensorFrame, SideWallEstimate, WallDistances


def create_driver() -> Driver:
    node = Driver()
    config = declare_and_load_config(node)
    grid = GridMap(
        size_m=config.grid_size_m,
        resolution_m=config.grid_resolution_m,
    )
    localizer = Localizer(
        grid,
        start_yaw_rad=config.start_yaw_rad,
        correction_alpha=config.correction_alpha,
        fuse_add_alpha=config.fuse_add_alpha,
        fuse_erode_alpha=config.fuse_erode_alpha,
        front_wall_motion_alpha=config.front_wall_motion_alpha,
        front_wall_motion_max_step_m=config.front_wall_motion_max_step_m,
        front_wall_motion_min_points=config.front_wall_motion_min_points,
        wall_pose_correction_alpha=config.wall_pose_correction_alpha,
        wall_pose_correction_max_step_m=config.wall_pose_correction_max_step_m,
        wall_pose_max_slope=config.wall_pose_max_slope,
        wall_yaw_correction_alpha=config.wall_yaw_correction_alpha,
        wall_yaw_correction_max_step_rad=config.wall_yaw_correction_max_step_rad,
        direction_lock_confirm_frames=config.direction_lock_confirm_frames,
        direction_lock_min_score_delta=config.direction_lock_min_score_delta,
        robot_length_m=config.robot_length_m,
        robot_width_m=config.robot_width_m,
    )
    planner = GridPlanner(
        grid,
        safety_buffer_m=config.safety_buffer_m,
        lookahead_m=config.lookahead_m,
        wheelbase_m=config.wheelbase_m,
        max_steering_angle_rad=config.max_steering_angle_rad,
        robot_length_m=config.robot_length_m,
        robot_width_m=config.robot_width_m,
        trajectory_steps=config.trajectory_steps,
        trajectory_length_m=config.trajectory_length_m,
        trajectory_step_m=config.trajectory_step_m,
        command_delay_s=config.command_delay_s,
        planning_speed_mps=config.planning_speed_mps,
    )
    debug_window = (
        DebugWindow(
            robot_length_m=config.robot_length_m,
            robot_width_m=config.robot_width_m,
        )
        if config.debug_view_enabled
        else None
    )
    node.configure(config, localizer, planner, debug_window)
    return node


def main(args=None) -> None:
    rclpy.init(args=args)
    node = create_driver()
    try:
        rclpy.spin(node)
    finally:
        node.stop_workers()
        node._command(0, 0)
        node.destroy_node()
        rclpy.try_shutdown()

__all__ = [
    "Cell",
    "DriverConfig",
    "Direction",
    "Driver",
    "FrontWallEstimate",
    "GridMap",
    "GridPlanner",
    "DebugFrame",
    "DebugStyle",
    "DebugWindow",
    "ImuReading",
    "LocalGrid",
    "Localizer",
    "Plan",
    "PoseEstimate",
    "ScanPoint",
    "SensorFrame",
    "SideWallEstimate",
    "WallDistances",
    "create_driver",
    "declare_and_load_config",
    "main",
]
