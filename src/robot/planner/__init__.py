from __future__ import annotations

import rclpy

from .config import DriverConfig, declare_and_load_config
from .debug import DebugFrame, DebugStyle, DebugWindow
from .driver import Driver
from .grid import Cell, Direction, GridMap, LocalGrid
from .localize_types import PoseEstimate
from .odometry_localize import OdometryLocalizer
from .planner import GridPlanner, Plan
from .sensors import FrontWallEstimate, ImuReading, ScanPoint, SensorFrame, SideWallEstimate, WallDistances


def create_driver() -> Driver:
    node = Driver()
    config = declare_and_load_config(node)
    grid = GridMap(
        size_m=config.grid_size_m,
        resolution_m=config.grid_resolution_m,
    )
    localizer = OdometryLocalizer(
        grid,
        start_yaw_rad=config.start_yaw_rad,
        correction_max_m=config.odometry_correction_max_m,
        fuse_add_alpha=config.fuse_add_alpha,
        fuse_erode_alpha=config.fuse_erode_alpha,
        direction_min_wall_points=config.direction_min_wall_points,
        direction_lock_confirm_frames=config.direction_lock_confirm_frames,
        direction_lock_min_score_delta=config.direction_lock_min_score_delta,
    )
    planner = GridPlanner(
        grid,
        wheelbase_m=config.wheelbase_m,
        max_steering_angle_rad=config.max_steering_angle_rad,
        robot_length_m=config.robot_length_m,
        robot_width_m=config.robot_width_m,
        trajectory_step_m=config.trajectory_step_m,
        target_laps=config.course_laps,
        actuation_delay_s=config.actuation_delay_s,
        decel_tau_s=config.planning_decel_tau_s,
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
    "OdometryLocalizer",
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
