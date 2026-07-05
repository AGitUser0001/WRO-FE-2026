from __future__ import annotations

import math

from .grid import Cell, Direction, GridMap, LocalGrid
from .sensors import SensorFrame


def scan_world_points(frame: SensorFrame, x_m: float, y_m: float, yaw_rad: float) -> tuple[tuple[float, float], ...]:
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    stride = max(1, len(frame.points) // 140)
    return tuple(
        (x_m + p.x_m * cy - p.y_m * sy, y_m + p.x_m * sy + p.y_m * cy)
        for i, p in enumerate(frame.points)
        if i % stride == 0 and 0.16 <= p.range_m <= 1.8
    )


def scan_motion_score(points: tuple[tuple[float, float], ...], previous: set[tuple[int, int]], dx: float, dy: float, resolution: float) -> float:
    if not previous:
        return 0.0
    matched = sum((round((x + dx) / resolution), round((y + dy) / resolution)) in previous for x, y in points)
    return matched / max(len(points), 1)


def local_obstacle_signature(local: LocalGrid) -> tuple[tuple[int, int], ...]:
    res = max(local.resolution_m, 0.05)
    out: list[tuple[int, int]] = []
    for row, col, value in local.non_free_cells():
        if value in (Cell.MAP_WALL, Cell.WALL) or (row + col) % 2 != 0:
            continue
        x_m, y_m = local.cell_to_local(row, col)
        out.append((round(x_m / res), round(y_m / res)))
    return tuple(out)


def best_local_shift(
    current: tuple[tuple[int, int], ...],
    previous: set[tuple[int, int]],
    pred_f: float,
    pred_l: float,
) -> tuple[float, float] | None:
    res = 0.05
    center_f = round(pred_f / res)
    center_l = round(pred_l / res)
    best = (0, 0, -1.0)
    for df in range(center_f - 2, center_f + 3):
        for dl in range(center_l - 2, center_l + 3):
            score = sum((x + df, y + dl) in previous for x, y in current) / max(len(current), 1)
            if score > best[2]:
                best = (df, dl, score)
    return None if best[2] < 0.35 else (best[0] * res, best[1] * res)


def scan_pose_valid(
    grid: GridMap,
    frame: SensorFrame,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    robot_radius_m: float,
) -> bool:
    r = robot_radius_m + grid.resolution_m
    if not grid._inside_map_bounds(x_m - r, y_m - r) or not grid._inside_map_bounds(x_m + r, y_m + r):
        return False
    cy = math.cos(yaw_rad); sy = math.sin(yaw_rad)
    for row, col, value in frame.local_grid.non_free_cells():
        if value != Cell.WALL:
            continue
        lx, ly = frame.local_grid.cell_to_local(row, col)
        gx = x_m + lx * cy - ly * sy; gy = y_m + lx * sy + ly * cy
        if not grid._inside_map_bounds(gx, gy) or _inside_center_hollow(grid, gx, gy):
            return False
    return True


def _inside_center_hollow(grid: GridMap, x_m: float, y_m: float) -> bool:
    if grid.direction == Direction.UNKNOWN:
        return False
    half_w = grid.corridor_width_m * 0.5
    x_min, x_max = (-2.5, half_w) if grid.direction == Direction.LEFT else (-half_w, 2.5)
    center_x = (x_min + x_max) * 0.5
    margin = grid.resolution_m * 0.75
    return abs(x_m - center_x) < 0.5 - margin and abs(y_m) < 0.5 - margin
