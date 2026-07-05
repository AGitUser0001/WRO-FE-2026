from __future__ import annotations

import math

from .grid import Cell


def local_pose_clear(
    local_grid,
    origin: tuple[float, float, float],
    pose: tuple[float, float, float],
    footprint: tuple[tuple[float, float], ...],
) -> bool:
    ox, oy, oyaw = origin
    x_m, y_m, yaw_rad = pose
    dx = x_m - ox
    dy = y_m - oy
    coy = math.cos(oyaw)
    soy = math.sin(oyaw)
    local_x = dx * coy + dy * soy
    local_y = -dx * soy + dy * coy
    local_yaw = yaw_rad - oyaw
    cy = math.cos(local_yaw)
    sy = math.sin(local_yaw)
    for fx, fy in footprint:
        idx = local_grid.local_to_cell(local_x + fx * cy - fy * sy, local_y + fx * sy + fy * cy)
        if idx is None:
            continue
        if Cell(int(local_grid.cells[idx])) != Cell.FREE:
            return False
    return True
