from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from scipy.ndimage import convolve, maximum_filter

from .dstar_lite import DStarLite
from .grid import Cell
from .local_grid import LocalGrid
from .localize_types import PoseEstimate

if TYPE_CHECKING:
    from .trajectory import TrajectoryMixin

GRAPH_CELL_FACTOR = 4


def graph_waypoints(
    planner: "TrajectoryMixin",
    pose: PoseEstimate,
    guide_points: tuple[tuple[float, float], ...],
    blocked_points: tuple[tuple[float, float], ...] = (),
) -> tuple[tuple[float, float], ...]:
    goal = _goal_point(planner, pose, guide_points)
    start_idx = planner.grid.world_to_cell(pose.x_m, pose.y_m)
    goal_idx = planner.grid.world_to_cell(goal[0], goal[1])
    if start_idx is None or goal_idx is None:
        return guide_points
    cost = _cost_grid(planner, pose)
    _add_route_blockers(planner, cost, blocked_points)
    _add_motion_blocks(planner, pose, cost)
    goal_idx = _nearest_clear_goal(cost, goal_idx)
    cells = _search(planner, cost, start_idx, goal_idx)
    if not cells:
        return guide_points
    points = tuple(planner.grid.cell_to_world(r, c) for r, c in cells)
    return _thin_points(points, planner.trajectory_step_m * 1.5)


def _goal_point(
    planner: "TrajectoryMixin",
    pose: PoseEstimate,
    guide_points: tuple[tuple[float, float], ...],
) -> tuple[float, float]:
    if guide_points:
        return guide_points[-1]
    return (
        pose.x_m + math.cos(pose.yaw_rad) * planner.lookahead_m,
        pose.y_m + math.sin(pose.yaw_rad) * planner.lookahead_m,
    )


def _cost_grid(planner: "TrajectoryMixin", pose: PoseEstimate) -> np.ndarray:
    cells = planner.grid.cells
    live = _cluster_mask(planner.grid.live_mask)
    cost = np.ones(cells.shape, dtype=np.float32)
    cost += np.where(cells == int(Cell.MAP_WALL), 1.0e9, 0.0)
    cost += np.where(cells == int(Cell.WALL), 1.0e6, 0.0)
    cost += np.where(cells == int(Cell.UNKNOWN_OBSTRUCTION), 120.0, 0.0)
    cost += np.where(live, 180.0, 0.0)
    _add_local_cost(planner, pose, cost)
    _add_inflation_cost(planner, cost, live)
    return cost


def _cluster_mask(mask: np.ndarray) -> np.ndarray:
    if not bool(mask.any()):
        return mask
    counts = np.asarray(
        convolve(mask.astype(np.uint8), np.ones((3, 3), dtype=np.uint8), mode="constant", cval=0),
        dtype=np.uint8,
    )
    return mask & (counts >= 3)


def _add_inflation_cost(planner: "TrajectoryMixin", cost: np.ndarray, live: np.ndarray) -> None:
    blocked = (cost >= 1.0e6) | live
    hard_m = math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5 + 0.06
    hard = max(1, int(math.ceil(hard_m / planner.grid.resolution_m)))
    hard_band = maximum_filter(blocked, size=hard * 2 + 1, mode="constant").astype(np.bool_)
    soft_m = hard_m + planner.safety_buffer_m * 0.75
    soft = max(hard + 1, int(math.ceil(soft_m / planner.grid.resolution_m)))
    soft_band = maximum_filter(blocked, size=soft * 2 + 1, mode="constant").astype(np.bool_)
    cost += np.where(hard_band & ~blocked, 1.0e8, 0.0)
    cost += np.where(soft_band & ~hard_band, 420.0, 0.0)


def _add_route_blockers(
    planner: "TrajectoryMixin",
    cost: np.ndarray,
    blocked_points: tuple[tuple[float, float], ...],
) -> None:
    half_cells = max(1, int(math.ceil(0.5 / planner.grid.resolution_m)))
    for x_m, y_m in blocked_points:
        center = planner.grid.world_to_cell(x_m, y_m)
        if center is None:
            continue
        r0 = max(0, center[0] - half_cells)
        r1 = min(cost.shape[0], center[0] + half_cells + 1)
        c0 = max(0, center[1] - half_cells)
        c1 = min(cost.shape[1], center[1] + half_cells + 1)
        cost[r0:r1, c0:c1] = 1.0e9


def _add_motion_blocks(planner: "TrajectoryMixin", pose: PoseEstimate, cost: np.ndarray) -> None:
    points = getattr(planner, "motion_block_points", lambda: ())()
    if not points:
        return
    radius = max(2, int(math.ceil(0.16 / planner.grid.resolution_m)))
    for x_m, y_m in points:
        if math.hypot(x_m - pose.x_m, y_m - pose.y_m) < planner.grid.resolution_m * 1.2:
            continue
        cell = planner.grid.world_to_cell(x_m, y_m)
        if cell is None:
            continue
        _add_hard_patch(cost, cell, radius)


def _nearest_clear_goal(cost: np.ndarray, goal: tuple[int, int]) -> tuple[int, int]:
    if cost[goal] < 1.0e3:
        return goal
    gr, gc = goal
    for radius in range(1, 13):
        r0 = max(0, gr - radius); r1 = min(cost.shape[0], gr + radius + 1)
        c0 = max(0, gc - radius); c1 = min(cost.shape[1], gc + radius + 1)
        rows, cols = np.nonzero(cost[r0:r1, c0:c1] < 1.0e3)
        if len(rows):
            scores = (rows + r0 - gr) ** 2 + (cols + c0 - gc) ** 2
            i = int(np.argmin(scores))
            return int(rows[i] + r0), int(cols[i] + c0)
    return goal


def _add_local_cost(planner: "TrajectoryMixin", pose: PoseEstimate, cost: np.ndarray) -> None:
    if planner._local_collision_grid is None or planner._local_collision_origin is None:
        return
    local = planner._local_collision_grid
    if not isinstance(local, LocalGrid):
        return
    ox, oy, oyaw = planner._local_collision_origin
    cy = math.cos(oyaw)
    sy = math.sin(oyaw)
    for row, col, value in local.non_free_cells():
        lx, ly = local.cell_to_local(row, col)
        idx = planner.grid.world_to_cell(ox + lx * cy - ly * sy, oy + lx * sy + ly * cy)
        if idx is None:
            continue
        clustered = _local_clustered(local, row, col)
        add = 1.0e6 if clustered else 240.0
        cost[idx] += add
        if clustered:
            _add_soft_patch(cost, idx, 3, 360.0)


def _local_clustered(local: LocalGrid, row: int, col: int) -> bool:
    r0 = max(0, row - 1); r1 = min(local.cells.shape[0], row + 2)
    c0 = max(0, col - 1); c1 = min(local.cells.shape[1], col + 2)
    return int(np.count_nonzero(local.cells[r0:r1, c0:c1] != int(Cell.FREE))) >= 3


def _add_soft_patch(cost: np.ndarray, center: tuple[int, int], radius: int, add: float) -> None:
    row, col = center
    r0 = max(0, row - radius); r1 = min(cost.shape[0], row + radius + 1)
    c0 = max(0, col - radius); c1 = min(cost.shape[1], col + radius + 1)
    cost[r0:r1, c0:c1] += add


def _add_hard_patch(cost: np.ndarray, center: tuple[int, int], radius: int) -> None:
    row, col = center
    r0 = max(0, row - radius); r1 = min(cost.shape[0], row + radius + 1)
    c0 = max(0, col - radius); c1 = min(cost.shape[1], col + radius + 1)
    cost[r0:r1, c0:c1] = 1.0e9


def _search(
    planner: "TrajectoryMixin",
    cost: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> tuple[tuple[int, int], ...]:
    factor = GRAPH_CELL_FACTOR
    coarse_cost = _coarsen_cost(cost, factor)
    coarse_start = _coarse_index(start, factor)
    coarse_goal = _coarse_index(goal, factor)
    stable_cost = _stable_cost(coarse_cost, coarse_goal)
    shape = (int(coarse_cost.shape[0]), int(coarse_cost.shape[1]))
    state = getattr(planner, "_dstar_state", None)
    if not isinstance(state, DStarLite) or state.shape != shape or state.goal != coarse_goal:
        state = DStarLite(shape, coarse_goal, stable_cost)
        setattr(planner, "_dstar_state", state)
    else:
        state.update_cost(stable_cost, coarse_start)
    coarse_path = state.path(coarse_start)
    fine_shape = (int(cost.shape[0]), int(cost.shape[1]))
    return _fine_path(coarse_path, factor, fine_shape, start, goal)


def _coarsen_cost(cost: np.ndarray, factor: int) -> np.ndarray:
    if factor <= 1:
        return cost
    h, w = cost.shape
    pad_h = (-h) % factor
    pad_w = (-w) % factor
    padded = np.pad(cost, ((0, pad_h), (0, pad_w)), mode="edge")
    coarse = padded.reshape(padded.shape[0] // factor, factor, padded.shape[1] // factor, factor)
    return coarse.max(axis=(1, 3)).astype(np.float32)


def _coarse_index(cell: tuple[int, int], factor: int) -> tuple[int, int]:
    return cell[0] // factor, cell[1] // factor


def _fine_path(
    coarse_path: tuple[tuple[int, int], ...],
    factor: int,
    fine_shape: tuple[int, int],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> tuple[tuple[int, int], ...]:
    if not coarse_path:
        return ()
    out = [start]
    for row, col in coarse_path[1:-1]:
        rr = min(fine_shape[0] - 1, row * factor + factor // 2)
        cc = min(fine_shape[1] - 1, col * factor + factor // 2)
        if (rr, cc) != out[-1]:
            out.append((rr, cc))
    if goal != out[-1]:
        out.append(goal)
    return tuple(out)


def _stable_cost(cost: np.ndarray, goal: tuple[int, int]) -> np.ndarray:
    rows, cols = np.indices(cost.shape, dtype=np.float32)
    h = np.hypot(rows - float(goal[0]), cols - float(goal[1]))
    h /= max(float(max(cost.shape)), 1.0)
    stable = (np.round(cost.astype(np.float32, copy=False) / 5.0) * 5.0).astype(np.float32)
    stable += h * 1.0e-3
    return stable


def _thin_points(points: tuple[tuple[float, float], ...], min_step: float) -> tuple[tuple[float, float], ...]:
    if len(points) <= 2:
        return points
    out = [points[0]]
    for point in points[1:]:
        if math.hypot(point[0] - out[-1][0], point[1] - out[-1][1]) >= min_step:
            out.append(point)
    if out[-1] != points[-1]:
        out.append(points[-1])
    return tuple(out)
