from __future__ import annotations

import math
import time
from functools import lru_cache
from typing import TYPE_CHECKING, cast

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, find_objects, label, map_coordinates
from scipy.interpolate import splev, splprep
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from .grid import Cell, Direction, inscribed_obstacle_mask
from .localize_types import PoseEstimate
from .obstacle_policy import (
    ACTIVE_OBSTACLE_CLUSTER_RADIUS_M,
    TRACKED_OBSTACLE_ASSOCIATION_M,
)

if TYPE_CHECKING:
    from .planner import GridPlanner

FOOTPRINT_EXTRA_MARGIN_M = 0.03
ROUTE_UNCERTAINTY_MARGIN_M = 0.05
ACTIVE_PASS_TRANSIENT_OBSTACLE_COST = 4.0
ROUTE_ENTRY_CHECK_M = 0.20
ROUTE_ENTRY_RECOVERY_M = 0.12
LATERAL_STOP_LEAD_M = 0.08
LATERAL_SETTLED_M = 0.004
CellIndex = tuple[int, int]


def active_obstacle_component_mask(
    planner: "GridPlanner",
    anchor: tuple[float, float],
) -> np.ndarray:
    dynamic = planner.grid.cells == int(Cell.UNKNOWN_OBSTRUCTION)
    dynamic = dynamic.copy()
    obstacle_cells = (
        planner._confirmed_local_obstacles | planner._previous_local_obstacles
    )
    if obstacle_cells:
        obstacle_rows, obstacle_cols = zip(*obstacle_cells)
        dynamic[np.asarray(obstacle_rows), np.asarray(obstacle_cols)] = True
    components, count = cast(
        tuple[np.ndarray, int],
        label(dynamic, structure=np.ones((3, 3), dtype=np.uint8)),
    )
    if count == 0:
        return np.zeros_like(dynamic)

    rows, cols = np.indices(dynamic.shape, dtype=np.float32)
    world_x = (cols - planner.grid.origin_cell) * planner.grid.resolution_m
    world_y = (planner.grid.origin_cell - rows) * planner.grid.resolution_m
    association_radius_m = (
        math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + planner.grid.resolution_m / math.sqrt(2.0)
    )
    near_anchor = (
        np.square(world_x - anchor[0]) + np.square(world_y - anchor[1])
        <= association_radius_m**2
    )
    component_ids = np.unique(components[near_anchor & dynamic])
    component_ids = component_ids[component_ids != 0]
    if not len(component_ids):
        return np.zeros_like(dynamic)
    return np.isin(components, component_ids)


@lru_cache(maxsize=4)
def _grid_graph_topology(
    height: int,
    width: int,
) -> tuple[csr_matrix, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    ids = np.arange(height * width, dtype=np.int32).reshape(height, width)
    rows: list[np.ndarray] = []
    cols: list[np.ndarray] = []
    for dr, dc in (
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1),
    ):
        source_rows = slice(max(0, -dr), min(height, height - dr))
        source_cols = slice(max(0, -dc), min(width, width - dc))
        target_rows = slice(max(0, dr), min(height, height + dr))
        target_cols = slice(max(0, dc), min(width, width + dc))
        rows.append(ids[target_rows, target_cols].ravel())
        cols.append(ids[source_rows, source_cols].ravel())
    graph = csr_matrix(
        (np.ones(sum(map(len, rows)), dtype=np.float32),
         (np.concatenate(rows), np.concatenate(cols))),
        shape=(height * width, height * width),
    )
    target = np.repeat(np.arange(height * width, dtype=np.int32), np.diff(graph.indptr))
    source = graph.indices.astype(np.int32, copy=False)
    target_row, target_col = np.divmod(target, width)
    source_row, source_col = np.divmod(source, width)
    diagonal = (target_row != source_row) & (target_col != source_col)
    orthogonal_a = np.where(diagonal, target_row * width + source_col, -1).astype(np.int32)
    orthogonal_b = np.where(diagonal, source_row * width + target_col, -1).astype(np.int32)
    weight = np.where(diagonal, math.sqrt(2.0), 1.0).astype(np.float32)
    return graph, target, weight, orthogonal_a, orthogonal_b


def reverse_grid_graph(cost: np.ndarray) -> csr_matrix:
    graph, target, weight, orthogonal_a, orthogonal_b = _grid_graph_topology(*cost.shape)
    flat_cost = np.asarray(cost, dtype=np.float32).ravel()
    edge_cost = flat_cost[target] * weight
    diagonal = orthogonal_a >= 0
    clear = flat_cost < 1.0e6
    blocked_corner = diagonal & ~(clear[orthogonal_a] & clear[orthogonal_b])
    edge_cost[blocked_corner | (edge_cost >= 1.0e6)] = math.inf
    return csr_matrix(
        (edge_cost, graph.indices, graph.indptr),
        shape=graph.shape,
        copy=False,
    )


def scipy_grid_distance_path(
    cost: np.ndarray,
    start: CellIndex,
    goal: CellIndex,
) -> tuple[tuple[CellIndex, ...], np.ndarray]:
    graph = reverse_grid_graph(np.asarray(cost, dtype=np.float32))
    start_flat = int(np.ravel_multi_index(start, cost.shape))
    goal_flat = int(np.ravel_multi_index(goal, cost.shape))
    flat_distances, predecessors = dijkstra(
        graph, directed=True, indices=goal_flat, return_predecessors=True,
    )
    distances = np.asarray(flat_distances, dtype=np.float32).reshape(cost.shape)
    if not math.isfinite(float(distances[start])):
        return (), distances
    flat_route = [start_flat]
    current = start_flat
    for _ in range(cost.size):
        if current == goal_flat:
            return tuple(
                (int(row), int(col))
                for row, col in zip(*np.unravel_index(flat_route, cost.shape))
            ), distances
        current = int(predecessors[current])
        if current < 0:
            return (), distances
        flat_route.append(current)
    return (), distances


def graph_waypoints(
    planner: "GridPlanner",
    pose: PoseEstimate,
    guide_points: tuple[tuple[float, float], ...],
    blocked_points: tuple[tuple[float, float], ...] = (),
) -> tuple[tuple[float, float], ...]:
    continue_color_backup = bool(getattr(planner, "_uncolored_backup_active", False))
    setattr(planner, "_uncolored_backup_active", False)
    setattr(planner, "_uncolored_obstacle_hold", False)
    start_idx = planner.grid.world_to_cell(pose.x_m, pose.y_m)
    guide_indices = tuple(
        planner.grid.world_to_cell(point[0], point[1])
        for point in guide_points
    )
    if start_idx is None or any(index is None for index in guide_indices):
        if planner._uncolored_obstacle_anchor is not None:
            setattr(planner, "_uncolored_obstacle_hold", True)
            setattr(planner, "_uncolored_backup_active", continue_color_backup)
        return guide_points
    started = time.perf_counter()
    cost = _cost_grid(planner)
    cost_done = time.perf_counter()
    _add_route_blockers(planner, cost, blocked_points, start_idx)
    geometry = cost >= 200.0
    passes = planner._route_obstacle_passes(pose)
    obstacle_pass = bool(passes)
    pass_geometry = geometry.copy()
    compact_obstacle_mask: np.ndarray | None = None
    active_obstacle_mask: np.ndarray | None = None
    map_geometry: np.ndarray | None = None
    pass_preference: np.ndarray | None = None
    if passes:
        compact_obstacle_mask = np.zeros_like(geometry)
        active_obstacle_mask = np.zeros_like(geometry)
        map_geometry = planner.grid.cells == int(Cell.MAP_WALL)
        pass_preference = np.zeros_like(cost)
        rows, cols = np.indices(geometry.shape, dtype=np.float32)
        world_x = (cols - planner.grid.origin_cell) * planner.grid.resolution_m
        world_y = (planner.grid.origin_cell - rows) * planner.grid.resolution_m
    for compact_anchor, action, yaw in passes:
        assert compact_obstacle_mask is not None and active_obstacle_mask is not None
        assert map_geometry is not None and pass_preference is not None
        active_mask = active_obstacle_component_mask(
            planner, compact_anchor,
        )
        active_mask |= (
            np.square(world_x - compact_anchor[0])
            + np.square(world_y - compact_anchor[1])
        ) <= ACTIVE_OBSTACLE_CLUSTER_RADIUS_M**2
        compact_obstacle_mask |= active_mask
        physical_mask = np.zeros_like(active_mask)
        current_active = planner._previous_local_obstacles
        if current_active:
            active_rows, active_cols = zip(*current_active)
            physical_mask[
                np.asarray(active_rows), np.asarray(active_cols)
            ] = True
            physical_mask &= active_mask
            physical_mask = inscribed_obstacle_mask(physical_mask)
        pass_geometry[active_mask] = map_geometry[active_mask]
        if not bool(physical_mask.any()):
            anchor_cell = planner.grid.world_to_cell(*compact_anchor)
            if anchor_cell is not None:
                physical_mask[anchor_cell] = True
        active_obstacle_mask |= physical_mask
        forward_x, forward_y = math.cos(yaw), math.sin(yaw)
        left_x, left_y = -forward_y, forward_x
        side_sign = 1.0 if action == "left" else -1.0
        relative_x = world_x - compact_anchor[0]
        relative_y = world_y - compact_anchor[1]
        along = relative_x * forward_x + relative_y * forward_y
        side = side_sign * (relative_x * left_x + relative_y * left_y)
        side_offset = planner._obstacle_pass_side_offset(
            compact_anchor,
            (forward_x, forward_y),
            (left_x, left_y),
            side_sign,
        )
        pass_weight = np.maximum(0.0, 1.0 - np.abs(along) / 0.40)
        side_deficit = np.maximum(0.0, side_offset - side)
        pass_preference += (
            4000.0
            * np.square(pass_weight)
            * np.square(side_deficit / max(side_offset, 0.05))
        ).astype(np.float32)
    if active_obstacle_mask is not None:
        pass_geometry[active_obstacle_mask] = True
    setup_done = time.perf_counter()
    cells: tuple[CellIndex, ...] = ()
    clearance = np.zeros_like(cost, dtype=np.float32)
    footprint_overlap = np.zeros_like(cost, dtype=np.float32)
    segment_cost = cost
    if obstacle_pass:
        segment_cost = np.where(cost < 1.0e6, 1.0, cost).astype(np.float32)
        if compact_obstacle_mask is not None and map_geometry is not None:
            replace_active_obstacle = compact_obstacle_mask & ~map_geometry
            segment_cost[replace_active_obstacle] = 1.0
        if active_obstacle_mask is not None:
            segment_cost[active_obstacle_mask] = 1.0e6
        if pass_preference is not None:
            segment_cost += pass_preference
        for cell in planner._previous_local_obstacles - planner._confirmed_local_obstacles:
            if compact_obstacle_mask is not None and bool(compact_obstacle_mask[cell]):
                continue
            segment_cost[cell] = max(
                float(segment_cost[cell]),
                ACTIVE_PASS_TRANSIENT_OBSTACLE_COST,
            )
    raw_goal = guide_indices[-1]
    assert raw_goal is not None
    goal_idx = _nearest_clear_goal(segment_cost, raw_goal)
    cells, clearance, footprint_overlap = _main_route_search(
        planner,
        segment_cost,
        pass_geometry,
        start_idx,
        goal_idx,
        pose,
        clearance_weight=0.0 if obstacle_pass else 700.0,
        penalize_initial_footprint=not obstacle_pass,
        compact_obstacle_mask=compact_obstacle_mask,
        active_obstacle_mask=active_obstacle_mask,
    )
    search_done = time.perf_counter()
    setattr(
        planner,
        "_profile_graph_ms",
        (
            (cost_done - started) * 1000.0,
            (setup_done - cost_done) * 1000.0,
            (search_done - setup_done) * 1000.0,
        ),
    )
    if not cells:
        if planner._uncolored_obstacle_anchor is not None:
            setattr(planner, "_uncolored_obstacle_hold", True)
            setattr(planner, "_uncolored_backup_active", continue_color_backup)
        return ()
    continuous_started = time.perf_counter()
    grid_path = np.asarray(
        tuple(planner.grid.cell_to_world(*cell) for cell in cells),
        dtype=np.float32,
    )
    grid_path[0] = (pose.x_m, pose.y_m)
    path = _continuous_path(planner, grid_path, clearance, footprint_overlap)
    path, lateral_command = _lateral_maneuver_path(planner, pose, path, clearance)
    path, backing_for_color, hold_for_color = _uncolored_obstacle_path(
        planner, pose, path, continue_color_backup,
    )
    entry_command: tuple[int, float] | None = None
    if lateral_command is None and not backing_for_color and not hold_for_color:
        path, entry_command = _feasible_route_entry(
            planner, pose, path, pass_geometry, clearance,
        )
    continuous_ms = (time.perf_counter() - continuous_started) * 1000.0
    profile = getattr(planner, "_profile_graph_ms", ())
    setattr(planner, "_profile_graph_ms", (*profile, continuous_ms))
    if backing_for_color:
        direction, steering = _route_command(
            planner, pose, path, forced_direction=-1,
        )
    elif lateral_command is not None:
        direction, steering = lateral_command
    elif entry_command is not None:
        direction, steering = entry_command
    else:
        direction, steering = _route_command(
            planner,
            pose,
            path,
        )
    if lateral_command is not None or backing_for_color:
        arc = _kinematic_arc_poses(planner, pose, direction, steering, ROUTE_ENTRY_CHECK_M)
        if not _arc_follows_course(arc, direction, planner._course_tangent(pose)):
            path, entry_command = _feasible_route_entry(
                planner, pose, path, pass_geometry, clearance, (direction, steering),
            )
            if entry_command is not None:
                direction, steering = entry_command
    setattr(planner, "_route_entry_recovery", entry_command is not None)
    setattr(planner, "_uncolored_backup_active", backing_for_color)
    setattr(planner, "_uncolored_obstacle_hold", hold_for_color)
    setattr(planner, "_heading_motion_direction", direction)
    setattr(planner, "_heading_steering_rad", steering)
    setattr(planner, "_path_max_curvature", _path_max_curvature(path))
    setattr(planner, "_path_min_clearance", _path_min_clearance(planner, path, clearance))
    return tuple((float(point[0]), float(point[1])) for point in path[1:])


def _path_max_curvature(path: np.ndarray) -> float:
    if len(path) < 3:
        return 0.0
    first = path[1:-1] - path[:-2]
    second = path[2:] - path[1:-1]
    chord = path[2:] - path[:-2]
    denominator = (
        np.linalg.norm(first, axis=1)
        * np.linalg.norm(second, axis=1)
        * np.linalg.norm(chord, axis=1)
    )
    cross = np.abs(first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0])
    curvature = np.divide(
        2.0 * cross,
        denominator,
        out=np.zeros_like(cross),
        where=denominator > 1.0e-6,
    )
    return float(np.max(curvature, initial=0.0))


def _path_min_clearance(
    planner: "GridPlanner",
    path: np.ndarray,
    clearance: np.ndarray,
) -> float:
    if not len(path):
        return 0.0
    rows = planner.grid.origin_cell - path[:, 1] / planner.grid.resolution_m
    cols = planner.grid.origin_cell + path[:, 0] / planner.grid.resolution_m
    sampled = np.asarray(map_coordinates(
        clearance,
        np.vstack((rows, cols)),
        order=1,
        mode="constant",
        cval=0.0,
    ), dtype=np.float32)
    return float(np.min(sampled))


def _feasible_route_entry(
    planner: "GridPlanner",
    pose: PoseEstimate,
    path: np.ndarray,
    geometry: np.ndarray,
    clearance: np.ndarray,
    command: tuple[int, float] | None = None,
) -> tuple[np.ndarray, tuple[int, float] | None]:
    direction, steering = command or _route_command(planner, pose, path)
    course_forward = planner._course_tangent(pose)
    turning = planner._program_action() == "u_turn"
    commanded = _kinematic_arc_poses(
        planner, pose, direction, steering, ROUTE_ENTRY_CHECK_M,
    )
    course_ok = turning or _arc_follows_course(commanded, direction, course_forward)
    if course_ok and _arc_clears_geometry(planner, commanded, geometry):
        return path, None

    steering_candidates = tuple(dict.fromkeys((
        steering,
        steering * 0.5,
        0.0,
        planner.max_steering_angle_rad,
        -planner.max_steering_angle_rad,
    )))
    path_distance = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))))
    aim = path[min(int(np.searchsorted(path_distance, 0.35)), len(path) - 1)]
    turning_radius = planner.wheelbase_m / math.tan(planner.max_steering_angle_rad)
    candidates: list[tuple[float, int, float]] = []
    for candidate_direction in (direction, -direction):
        for candidate_steering in steering_candidates:
            poses = _kinematic_arc_poses(
                planner, pose, candidate_direction, candidate_steering, ROUTE_ENTRY_CHECK_M,
            )
            if not turning and not _arc_follows_course(poses, candidate_direction, course_forward):
                continue
            if not _arc_clears_geometry(planner, poses, geometry):
                continue
            end = poses[-1]
            dx, dy = aim - end[:2]
            angle = math.atan2(float(dy), float(dx)) - float(end[2])
            heading_error = abs(math.atan2(math.sin(angle), math.cos(angle)))
            endpoint_clearance = _sample_path_field(
                planner, clearance, poses[-1:, :2], order=1,
            )[0]
            score = math.hypot(float(dx), float(dy)) + turning_radius * heading_error - 0.10 * float(endpoint_clearance)
            candidates.append((score, candidate_direction, candidate_steering))
    if not candidates:
        return path, None if course_ok else (0, 0.0)
    _score, recovery_direction, recovery_steering = min(candidates)
    recovery = _kinematic_arc_poses(
        planner, pose, recovery_direction, recovery_steering, ROUTE_ENTRY_RECOVERY_M,
    )
    displayed = np.vstack((recovery[:, :2], path[1:]))
    return displayed, (recovery_direction, recovery_steering)


def _arc_follows_course(
    poses: np.ndarray, direction: int, course_forward: tuple[float, float],
) -> bool:
    alignment = (np.cos(poses[:, 2]) * course_forward[0]
                 + np.sin(poses[:, 2]) * course_forward[1])
    if direction < 0 and np.any(np.diff(poses[:, :2], axis=0) @ course_forward > 1.0e-5):
        return False
    return bool(
        np.min(alignment) >= min(0.0, float(alignment[0])) - 1.0e-4
        and alignment[-1] >= min(0.0, float(alignment[0]) + 0.015) - 1.0e-4
    )


def _arc_clears_geometry(
    planner: "GridPlanner",
    poses: np.ndarray,
    geometry: np.ndarray,
) -> bool:
    depth = tracked_obstacle_overlap(planner, poses)
    if np.any(depth[-1] > 1.0e-6) or np.any(np.diff(depth, axis=0) > 1.0e-6):
        return False
    overlap = _footprint_overlap_counts(planner, poses, geometry)
    if not overlap[0]:
        return not bool(np.any(overlap))
    if overlap[-1]:
        return False
    rows, cols = np.nonzero(geometry)
    dx = (cols - planner.grid.origin_cell) * planner.grid.resolution_m - poses[0, 0]
    dy = (planner.grid.origin_cell - rows) * planner.grid.resolution_m - poses[0, 1]
    cy, sy = math.cos(float(poses[0, 2])), math.sin(float(poses[0, 2]))
    margin = planner.grid.resolution_m / math.sqrt(2.0)
    touching = (
        (np.abs(dx * cy + dy * sy) <= planner.robot_length_m * 0.5 + margin)
        & (np.abs(-dx * sy + dy * cy) <= planner.robot_width_m * 0.5 + margin)
    )
    if not bool(np.any(touching)):
        return False
    remaining = geometry.copy()
    remaining[rows[touching], cols[touching]] = False
    if bool(np.any(_footprint_overlap_counts(planner, poses, remaining))):
        return False
    obstacles = np.column_stack((dx[touching], dy[touching])) + poses[0, :2]
    distance_sq = np.sum(np.square(poses[:, None, :2] - obstacles[None, :, :]), axis=2)
    return bool(np.all(np.diff(distance_sq, axis=0)[overlap[:-1] > 0] >= -1.0e-7))


def tracked_obstacle_overlap(planner: "GridPlanner", poses: np.ndarray) -> np.ndarray:
    anchors = np.asarray([
        obstacle.anchor for obstacle in planner._tracked_obstacles
        if obstacle.observations >= 2
    ], dtype=np.float64).reshape(-1, 2)
    dx = anchors[None, :, 0] - poses[:, 0:1]
    dy = anchors[None, :, 1] - poses[:, 1:2]
    cy, sy = np.cos(poses[:, 2:3]), np.sin(poses[:, 2:3])
    abs_cy, abs_sy = np.abs(cy), np.abs(sy)
    half_l, half_w = planner.robot_length_m * 0.5, planner.robot_width_m * 0.5
    half_obstacle = 0.025
    projected_obstacle = half_obstacle * (abs_cy + abs_sy)
    separation = np.maximum(
        np.maximum(
            np.abs(dx) - half_l * abs_cy - half_w * abs_sy - half_obstacle,
            np.abs(dy) - half_l * abs_sy - half_w * abs_cy - half_obstacle,
        ),
        np.maximum(
            np.abs(dx * cy + dy * sy) - half_l - projected_obstacle,
            np.abs(-dx * sy + dy * cy) - half_w - projected_obstacle,
        ),
    )
    return np.maximum(0.0, -separation)


def _kinematic_arc_poses(
    planner: "GridPlanner",
    pose: PoseEstimate,
    direction: int,
    steering: float,
    distance_m: float,
) -> np.ndarray:
    steps = max(2, int(math.ceil(distance_m / 0.01)))
    step_m = distance_m / steps
    curvature = math.tan(steering) / planner.wheelbase_m
    result = np.empty((steps + 1, 3), dtype=np.float32)
    x_m, y_m, yaw = pose.x_m, pose.y_m, pose.yaw_rad
    result[0] = (x_m, y_m, yaw)
    for index in range(1, steps + 1):
        yaw_delta = direction * step_m * curvature
        travel_yaw = yaw + yaw_delta * 0.5
        x_m += direction * step_m * math.cos(travel_yaw)
        y_m += direction * step_m * math.sin(travel_yaw)
        yaw = math.atan2(math.sin(yaw + yaw_delta), math.cos(yaw + yaw_delta))
        result[index] = (x_m, y_m, yaw)
    return result


def _footprint_overlap_counts(
    planner: "GridPlanner",
    poses: np.ndarray,
    geometry: np.ndarray,
) -> np.ndarray:
    spacing = max(planner.grid.resolution_m * 0.5, 0.02)
    longitudinal = np.linspace(
        -planner.robot_length_m * 0.5,
        planner.robot_length_m * 0.5,
        max(2, int(math.ceil(planner.robot_length_m / spacing)) + 1),
        dtype=np.float32,
    )
    lateral = np.linspace(
        -planner.robot_width_m * 0.5,
        planner.robot_width_m * 0.5,
        max(2, int(math.ceil(planner.robot_width_m / spacing)) + 1),
        dtype=np.float32,
    )
    forward, side = np.meshgrid(longitudinal, lateral, indexing="ij")
    forward = forward.ravel()[None, :]
    side = side.ravel()[None, :]
    yaw = poses[:, 2:3]
    cy, sy = np.cos(yaw), np.sin(yaw)
    x_m = poses[:, 0:1] + forward * cy - side * sy
    y_m = poses[:, 1:2] + forward * sy + side * cy
    cols = np.rint(x_m / planner.grid.resolution_m).astype(np.intp) + planner.grid.origin_cell
    rows = planner.grid.origin_cell - np.rint(y_m / planner.grid.resolution_m).astype(np.intp)
    valid = (
        (rows >= 0) & (rows < geometry.shape[0])
        & (cols >= 0) & (cols < geometry.shape[1])
    )
    occupied = ~valid
    occupied[valid] = geometry[rows[valid], cols[valid]]
    return np.count_nonzero(occupied, axis=1)


def _sample_path_field(
    planner: "GridPlanner",
    field: np.ndarray,
    points: np.ndarray,
    order: int,
) -> np.ndarray:
    rows = planner.grid.origin_cell - points[:, 1] / planner.grid.resolution_m
    cols = planner.grid.origin_cell + points[:, 0] / planner.grid.resolution_m
    return np.asarray(map_coordinates(
        field,
        np.vstack((rows, cols)),
        order=order,
        mode="constant",
        cval=0.0,
    ), dtype=np.float32)


def _lateral_maneuver_path(
    planner: "GridPlanner",
    pose: PoseEstimate,
    path: np.ndarray,
    clearance: np.ndarray,
) -> tuple[np.ndarray, tuple[int, float] | None]:
    if len(path) < 2:
        return path, None
    target = (float(path[-1, 0]), float(path[-1, 1]))
    active_target = planner._lateral_maneuver_target
    if active_target is not None and math.dist(active_target, target) > 0.20:
        _clear_lateral_maneuver(planner)
        active_target = None

    if active_target is None:
        dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
        cy, sy = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        forward = dx * cy + dy * sy
        lateral = -dx * sy + dy * cy
        if math.hypot(dx, dy) > 0.35 or abs(forward) > 0.14 or abs(lateral) <= 0.08:
            _clear_lateral_probe(planner)
            return path, None
        distance = math.hypot(dx, dy)
        if (
            planner._lateral_probe_target is None
            or math.dist(planner._lateral_probe_target, target) > 0.20
        ):
            planner._lateral_probe_target = target
            planner._lateral_probe_best_m = distance
            planner._lateral_probe_odom_m = pose.odometry_travel_m
            return path, None
        if distance < planner._lateral_probe_best_m - 0.025:
            planner._lateral_probe_best_m = distance
            planner._lateral_probe_odom_m = pose.odometry_travel_m
            return path, None
        if pose.odometry_travel_m - planner._lateral_probe_odom_m < 0.12:
            return path, None
        geometry = _lateral_geometry(planner, pose, target, clearance)
        if geometry is None:
            return path, None
        arc_m, sign, direction = geometry
        planner._lateral_maneuver_target = target
        planner._lateral_maneuver_phase = 1
        planner._lateral_maneuver_arc_m = arc_m
        planner._lateral_maneuver_phase_odom_m = pose.odometry_travel_m
        planner._lateral_maneuver_sign = sign
        planner._lateral_maneuver_direction = direction
        _clear_lateral_probe(planner)

    phase = planner._lateral_maneuver_phase
    if phase in (1, 3, 5):
        planner._lateral_maneuver_hold = True
        steering_sign = planner._lateral_maneuver_sign if phase == 1 else (
            -planner._lateral_maneuver_sign if phase == 3 else 0
        )
        if pose.odometry_motion_m > LATERAL_SETTLED_M:
            return path, (
                planner._lateral_maneuver_direction,
                steering_sign * planner.max_steering_angle_rad,
            )
        if phase == 5:
            _clear_lateral_maneuver(planner)
            return path, None
        if phase == 1:
            geometry = _lateral_geometry(planner, pose, target, clearance)
            if geometry is None:
                _clear_lateral_maneuver(planner)
                return path, None
            (
                planner._lateral_maneuver_arc_m,
                planner._lateral_maneuver_sign,
                planner._lateral_maneuver_direction,
            ) = geometry
            planner._lateral_maneuver_phase = 2
        else:
            planner._lateral_maneuver_phase = 4
        planner._lateral_maneuver_phase_odom_m = pose.odometry_travel_m
        planner._lateral_maneuver_hold = False

    phase = planner._lateral_maneuver_phase
    traveled = max(0.0, pose.odometry_travel_m - planner._lateral_maneuver_phase_odom_m)
    stop_at_m = max(0.02, planner._lateral_maneuver_arc_m - LATERAL_STOP_LEAD_M)
    if traveled >= stop_at_m:
        planner._lateral_maneuver_phase = 3 if phase == 2 else 5
        planner._lateral_maneuver_phase_odom_m = pose.odometry_travel_m
        planner._lateral_maneuver_hold = True
        next_sign = -planner._lateral_maneuver_sign if phase == 2 else 0
        return path, (
            planner._lateral_maneuver_direction,
            next_sign * planner.max_steering_angle_rad,
        )

    first_remaining = (
        max(0.0, planner._lateral_maneuver_arc_m - traveled)
        if phase == 2 else 0.0
    )
    second_remaining = (
        planner._lateral_maneuver_arc_m
        if phase == 2
        else max(0.0, planner._lateral_maneuver_arc_m - traveled)
    )
    maneuver = _lateral_arc_points(
        planner,
        pose.x_m,
        pose.y_m,
        pose.yaw_rad,
        planner._lateral_maneuver_direction,
        planner._lateral_maneuver_sign,
        first_remaining,
        second_remaining,
    )
    required_clearance = (
        math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + planner.grid.resolution_m * 0.5
    )
    if _lateral_path_clearance(planner, maneuver, clearance) < required_clearance:
        planner._lateral_maneuver_phase = 5
        planner._lateral_maneuver_phase_odom_m = pose.odometry_travel_m
        planner._lateral_maneuver_hold = True
        return path, (planner._lateral_maneuver_direction, 0.0)
    displayed = np.vstack((maneuver, np.asarray((target,), dtype=np.float32)))
    steering_sign = (
        planner._lateral_maneuver_sign
        if phase == 2
        else -planner._lateral_maneuver_sign
    )
    return displayed, (
        planner._lateral_maneuver_direction,
        steering_sign * planner.max_steering_angle_rad,
    )


def _lateral_geometry(
    planner: "GridPlanner",
    pose: PoseEstimate,
    target: tuple[float, float],
    clearance: np.ndarray,
) -> tuple[float, int, int] | None:
    dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
    lateral = -dx * math.sin(pose.yaw_rad) + dy * math.cos(pose.yaw_rad)
    if abs(lateral) <= 0.05:
        return None
    radius = planner.wheelbase_m / max(math.tan(planner.max_steering_angle_rad), 1.0e-6)
    max_shift = 2.0 * radius * (1.0 - math.cos(math.radians(40.0)))
    shift = min(abs(lateral) - 0.05, max_shift)
    arc_m = radius * math.acos(max(-1.0, min(1.0, 1.0 - shift / (2.0 * radius))))
    sign = 1 if lateral > 0.0 else -1
    candidates = []
    for direction in (-1, 1):
        points = _lateral_arc_points(
            planner, pose.x_m, pose.y_m, pose.yaw_rad,
            direction, sign, arc_m, arc_m,
        )
        candidates.append((_lateral_path_clearance(planner, points, clearance), direction))
    best_clearance, direction = max(candidates)
    required_clearance = (
        math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + planner.grid.resolution_m * 0.5
    )
    return None if best_clearance < required_clearance else (arc_m, sign, direction)


def _lateral_arc_points(
    planner: "GridPlanner",
    x_m: float,
    y_m: float,
    yaw_rad: float,
    direction: int,
    sign: int,
    first_m: float,
    second_m: float,
) -> np.ndarray:
    points = [(x_m, y_m)]
    step_m = max(0.01, planner.trajectory_step_m)
    for distance, steering_sign in ((first_m, sign), (second_m, -sign)):
        steps = max(0, int(math.ceil(distance / step_m)))
        if not steps:
            continue
        ds = distance / steps
        curvature = steering_sign * math.tan(planner.max_steering_angle_rad) / planner.wheelbase_m
        for _ in range(steps):
            yaw_delta = direction * ds * curvature
            travel_yaw = yaw_rad + yaw_delta * 0.5
            x_m += direction * ds * math.cos(travel_yaw)
            y_m += direction * ds * math.sin(travel_yaw)
            yaw_rad = math.atan2(math.sin(yaw_rad + yaw_delta), math.cos(yaw_rad + yaw_delta))
            points.append((x_m, y_m))
    return np.asarray(points, dtype=np.float32)


def _lateral_path_clearance(
    planner: "GridPlanner",
    points: np.ndarray,
    clearance: np.ndarray,
) -> float:
    rows = planner.grid.origin_cell - points[:, 1] / planner.grid.resolution_m
    cols = planner.grid.origin_cell + points[:, 0] / planner.grid.resolution_m
    sampled = np.asarray(map_coordinates(
        clearance,
        np.vstack((rows, cols)),
        order=1,
        mode="constant",
        cval=0.0,
    ), dtype=np.float32)
    return float(np.min(sampled))


def _clear_lateral_maneuver(planner: "GridPlanner") -> None:
    planner._lateral_maneuver_target = None
    planner._lateral_maneuver_phase = 0
    planner._lateral_maneuver_arc_m = 0.0
    planner._lateral_maneuver_sign = 0
    planner._lateral_maneuver_direction = 1
    planner._lateral_maneuver_hold = False


def _clear_lateral_probe(planner: "GridPlanner") -> None:
    planner._lateral_probe_target = None
    planner._lateral_probe_best_m = math.inf
    planner._lateral_probe_odom_m = 0.0

def _uncolored_obstacle_path(
    planner: "GridPlanner",
    pose: PoseEstimate,
    path: np.ndarray,
    continue_backup: bool,
) -> tuple[np.ndarray, bool, bool]:
    now = time.monotonic()
    tracked = planner._tracked_obstacles
    colored_anchors = np.asarray(
        [obstacle.anchor for obstacle in tracked]
        + list(planner._colored_obstacle_anchors),
        dtype=np.float32,
    ).reshape(-1, 2)
    current_cells = planner._color_lidar_obstacles
    current_neighborhood = {
        (row + dr, col + dc)
        for row, col in current_cells
        for dr in (-1, 0, 1)
        for dc in (-1, 0, 1)
    }
    obstacle_cells: set[CellIndex] = set(current_cells)
    obstacle_cells.update(planner._confirmed_local_obstacles & current_neighborhood)
    obstacle_mask = np.zeros(planner.grid.cells.shape, dtype=np.bool_)
    if obstacle_cells:
        rows, cols = zip(*obstacle_cells)
        obstacle_mask[np.asarray(rows), np.asarray(cols)] = True
    components, component_count = cast(
        tuple[np.ndarray, int],
        label(obstacle_mask, structure=np.ones((3, 3), dtype=np.uint8)),
    )
    component_sizes = np.bincount(
        components.ravel(), minlength=int(component_count) + 1,
    )
    uncolored_anchors: list[tuple[float, float]] = []
    for component_id, bounds in enumerate(find_objects(components), start=1):
        if bounds is None:
            continue
        row_slice, col_slice = bounds
        height = int(row_slice.stop - row_slice.start)
        width = int(col_slice.stop - col_slice.start)
        aspect = max(height, width) / max(1, min(height, width))
        if height > 6 or width > 6 or aspect > 3.0 or component_sizes[component_id] > 24:
            continue
        local_rows, local_cols = np.nonzero(
            components[row_slice, col_slice] == component_id,
        )
        rows = local_rows + int(row_slice.start)
        cols = local_cols + int(col_slice.start)
        component_points = np.column_stack((
            (cols - planner.grid.origin_cell) * planner.grid.resolution_m,
            (planner.grid.origin_cell - rows) * planner.grid.resolution_m,
        )).astype(np.float32)
        if len(colored_anchors) and float(np.min(
            np.linalg.norm(
                component_points[:, None, :] - colored_anchors[None, :, :],
                axis=2,
            ),
        )) <= TRACKED_OBSTACLE_ASSOCIATION_M:
            continue
        center = np.mean(component_points, axis=0)
        uncolored_anchors.append((float(center[0]), float(center[1])))
    obstacles = np.asarray(uncolored_anchors, dtype=np.float32).reshape(-1, 2)
    persistent_obstacles = np.asarray(
        [planner.grid.cell_to_world(*cell) for cell in planner._confirmed_local_obstacles],
        dtype=np.float32,
    ).reshape(-1, 2)
    candidate: tuple[float, float, float, float] | None = None
    if len(obstacles):
        cy = math.cos(pose.yaw_rad)
        sy = math.sin(pose.yaw_rad)
        obstacle_relative = obstacles - np.asarray((pose.x_m, pose.y_m), dtype=np.float32)
        camera_forward = obstacle_relative[:, 0] * cy + obstacle_relative[:, 1] * sy
        segments = path[1:] - path[:-1]
        lengths = np.linalg.norm(segments, axis=1)
        valid = lengths > 1.0e-5
        starts = path[:-1][valid]
        vectors = segments[valid]
        lengths = lengths[valid]
        if len(lengths):
            relative = obstacles[:, None, :] - starts[None, :, :]
            projection = np.clip(
                np.sum(relative * vectors[None, :, :], axis=2)
                / np.square(lengths)[None, :],
                0.0,
                1.0,
            )
            closest = starts[None, :, :] + projection[:, :, None] * vectors[None, :, :]
            route_distance = np.linalg.norm(obstacles[:, None, :] - closest, axis=2)
            segment_index = np.argmin(route_distance, axis=1)
            nearest_distance = route_distance[np.arange(len(obstacles)), segment_index]
            cumulative = np.concatenate(([0.0], np.cumsum(lengths[:-1])))
            arclength = cumulative[segment_index] + (
                projection[np.arange(len(obstacles)), segment_index] * lengths[segment_index]
            )
            approaching = (
                (arclength >= 0.0)
                & (arclength <= 0.90)
                & (nearest_distance <= 0.40)
                & (camera_forward >= 0.05)
            )
            for index in np.nonzero(approaching)[0]:
                item = (
                    float(arclength[index]),
                    float(nearest_distance[index]),
                    float(obstacles[index, 0]),
                    float(obstacles[index, 1]),
                )
                if candidate is None or item < candidate:
                    candidate = item

    anchor = getattr(planner, "_uncolored_obstacle_anchor", None)
    if (
        anchor is not None
        and candidate is not None
        and math.hypot(anchor[0] - candidate[2], anchor[1] - candidate[3]) > 0.20
    ):
        candidate = None
    if anchor is not None and candidate is None and not continue_backup:
        anchor_supported = (
            len(persistent_obstacles) > 0
            and float(np.min(
                np.linalg.norm(
                    persistent_obstacles - np.asarray(anchor, dtype=np.float32), axis=1,
                ),
            )) <= 0.22
        )
        if not anchor_supported:
            setattr(planner, "_uncolored_obstacle_anchor", None)
            setattr(planner, "_uncolored_obstacle_debug", "none")
            planner._uncolored_backup_start_pose = None
            setattr(planner, "_uncolored_backup_done", False)
            return path, False, False
    if anchor is not None and not continue_backup:
        dx = anchor[0] - pose.x_m
        dy = anchor[1] - pose.y_m
        anchor_forward = math.cos(pose.yaw_rad) * dx + math.sin(pose.yaw_rad) * dy
        if anchor_forward < 0.05:
            setattr(planner, "_uncolored_obstacle_anchor", None)
            setattr(planner, "_uncolored_obstacle_debug", "none")
            planner._uncolored_backup_start_pose = None
            setattr(planner, "_uncolored_backup_done", False)
            return path, False, False
    if anchor is not None and continue_backup and (
        not len(persistent_obstacles)
        or float(np.min(
            np.linalg.norm(persistent_obstacles - np.asarray(anchor), axis=1),
        )) > 0.22
    ):
        setattr(planner, "_uncolored_obstacle_anchor", None)
        setattr(planner, "_uncolored_obstacle_debug", "none")
        planner._uncolored_backup_start_pose = None
        setattr(planner, "_uncolored_backup_done", False)
        return path, False, False
    if candidate is not None:
        new_anchor = (candidate[2], candidate[3])
        if anchor is None or math.hypot(anchor[0] - new_anchor[0], anchor[1] - new_anchor[1]) > 0.20:
            setattr(planner, "_uncolored_obstacle_started_at", now)
            planner._uncolored_backup_start_pose = None
            setattr(planner, "_uncolored_backup_done", False)
        anchor = new_anchor
        setattr(planner, "_uncolored_obstacle_anchor", anchor)
    elif anchor is not None:
        color_arrived = any(
            math.dist(anchor, colored_anchor) <= TRACKED_OBSTACLE_ASSOCIATION_M
            for colored_anchor in planner._colored_obstacle_anchors
        )
        if color_arrived:
            anchor = None
            setattr(planner, "_uncolored_obstacle_anchor", None)
    if anchor is None:
        setattr(planner, "_uncolored_obstacle_debug", "none")
        planner._uncolored_backup_start_pose = None
        setattr(planner, "_uncolored_backup_done", False)
        return path, False, False

    route_s = candidate[0] if candidate is not None else math.hypot(
        anchor[0] - pose.x_m, anchor[1] - pose.y_m,
    )
    backup_start = planner._uncolored_backup_start_pose
    backup_done = bool(getattr(planner, "_uncolored_backup_done", False))
    if continue_backup and backup_start is not None:
        dx = pose.x_m - backup_start[0]
        dy = pose.y_m - backup_start[1]
        backed_m = -(dx * math.cos(backup_start[2]) + dy * math.sin(backup_start[2]))
        if backed_m >= 0.35:
            continue_backup = False
            backup_done = True
            planner._uncolored_backup_start_pose = None
            setattr(planner, "_uncolored_backup_done", True)
            setattr(planner, "_uncolored_obstacle_started_at", now)

    waiting_s = now - float(getattr(planner, "_uncolored_obstacle_started_at", now))
    if backup_done:
        if waiting_s < 1.0:
            return path, False, True
        backup_done = False
        setattr(planner, "_uncolored_backup_done", False)

    if route_s > 0.60 and not continue_backup:
        setattr(
            planner,
            "_uncolored_obstacle_debug",
            f"{anchor[0]:+.2f}:{anchor[1]:+.2f}/{route_s:.2f}m/approach",
        )
        return path, False, False

    if waiting_s < 0.20 and not continue_backup:
        setattr(
            planner,
            "_uncolored_obstacle_debug",
            f"{anchor[0]:+.2f}:{anchor[1]:+.2f}/wait{waiting_s:.2f}",
        )
        return path, False, True

    if backup_start is None:
        planner._uncolored_backup_start_pose = (pose.x_m, pose.y_m, pose.yaw_rad)

    cy, sy = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    distances = np.asarray((0.10, 0.20, 0.30, 0.40), dtype=np.float32)
    backup = np.column_stack((
        pose.x_m - distances * cy,
        pose.y_m - distances * sy,
    )).astype(np.float32)
    rows = planner.grid.origin_cell - backup[:, 1] / planner.grid.resolution_m
    cols = planner.grid.origin_cell + backup[:, 0] / planner.grid.resolution_m
    physical_geometry = (
        (planner.grid.cells == int(Cell.MAP_WALL))
        | (planner.grid.cells == int(Cell.WALL))
    )
    physical_obstacles = set(planner._confirmed_local_obstacles)
    physical_obstacles.update(planner._previous_local_obstacles)
    if physical_obstacles:
        obstacle_rows, obstacle_cols = zip(*physical_obstacles)
        physical_geometry[np.asarray(obstacle_rows), np.asarray(obstacle_cols)] = True
    cell_margin = planner.grid.resolution_m * 0.5
    half_length = (
        planner.robot_length_m * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + cell_margin
        + 2.0 * ROUTE_UNCERTAINTY_MARGIN_M
    )
    half_width = (
        planner.robot_width_m * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + cell_margin
        + ROUTE_UNCERTAINTY_MARGIN_M
    )
    radius = int(math.ceil(math.hypot(half_length, half_width) / planner.grid.resolution_m))
    axis = np.arange(-radius, radius + 1, dtype=np.float32)
    dc, dr = np.meshgrid(axis, axis)
    dx = dc * planner.grid.resolution_m
    dy = -dr * planner.grid.resolution_m
    footprint = (
        (np.abs(dx * cy + dy * sy) <= half_length)
        & (np.abs(-dx * sy + dy * cy) <= half_width)
    )
    invalid_centers = binary_dilation(
        physical_geometry, structure=footprint, border_value=1,
    )
    blocked_backup = map_coordinates(
        invalid_centers,
        np.vstack((rows, cols)),
        order=0,
        mode="constant",
        cval=1,
    )
    safe_count = 0
    for blocked in blocked_backup:
        if blocked:
            break
        safe_count += 1
    setattr(
        planner,
        "_uncolored_obstacle_debug",
        f"{anchor[0]:+.2f}:{anchor[1]:+.2f}/{route_s:.2f}m/safe{safe_count}",
    )
    if safe_count == 0:
        hold = route_s <= 0.35
        return path, False, hold
    prefix = np.vstack((path[0], backup[:safe_count]))
    return np.vstack((prefix, path[1:])), True, False


def _cost_grid(planner: "GridPlanner") -> np.ndarray:
    cells = planner.grid.cells
    map_walls = cells == int(Cell.MAP_WALL)
    persisted_obstacles = cells == int(Cell.UNKNOWN_OBSTRUCTION)
    cost = np.ones(cells.shape, dtype=np.float32)
    cost += np.where(map_walls, 1.0e9, 0.0)
    cost += np.where(cells == int(Cell.WALL), 1.0e6, 0.0)
    cost += np.where(persisted_obstacles, 250.0, 0.0)
    if (
        planner._route_static_cost is None
        or planner._route_static_cost_direction != planner.grid.direction
    ):
        static_cost = np.zeros(cells.shape, dtype=np.float32)
        _add_route_center_cost(planner, static_cost)
        _add_center_block_corner_cost(planner, static_cost)
        planner._route_static_cost = static_cost
        planner._route_static_cost_direction = planner.grid.direction
    cost += planner._route_static_cost
    _add_local_cost(planner, cost)
    return cost


def _add_route_center_cost(planner: "GridPlanner", cost: np.ndarray) -> None:
    if planner.grid.direction == Direction.UNKNOWN:
        return
    route = planner._course_centerline()
    if len(route) < 2:
        return
    rows, cols = np.indices(cost.shape, dtype=np.float32)
    world_x = (cols - planner.grid.origin_cell) * planner.grid.resolution_m
    world_y = (planner.grid.origin_cell - rows) * planner.grid.resolution_m
    nearest_sq = np.full(cost.shape, np.inf, dtype=np.float32)
    for a, b in zip(route, route[1:]):
        vx, vy = b[0] - a[0], b[1] - a[1]
        length_sq = max(vx * vx + vy * vy, 1.0e-6)
        t = np.clip(
            ((world_x - a[0]) * vx + (world_y - a[1]) * vy) / length_sq,
            0.0,
            1.0,
        )
        dx = world_x - (a[0] + vx * t)
        dy = world_y - (a[1] + vy * t)
        nearest_sq = np.minimum(nearest_sq, dx * dx + dy * dy)
    cost += np.minimum(nearest_sq / (0.25 * 0.25), 8.0).astype(np.float32) * 1.50


def _add_center_block_corner_cost(planner: "GridPlanner", cost: np.ndarray) -> None:
    direction = planner.grid.direction
    if direction == Direction.UNKNOWN:
        return
    center_x = -1.0 if direction == Direction.LEFT else 1.0
    rows, cols = np.indices(cost.shape, dtype=np.float32)
    world_x = (cols - planner.grid.origin_cell) * planner.grid.resolution_m
    world_y = (planner.grid.origin_cell - rows) * planner.grid.resolution_m
    radius_m = 0.25
    nearest_sq = np.full(cost.shape, np.inf, dtype=np.float32)
    for corner_x in (center_x - 0.5, center_x + 0.5):
        for corner_y in (-0.5, 0.5):
            nearest_sq = np.minimum(
                nearest_sq,
                np.square(world_x - corner_x) + np.square(world_y - corner_y),
            )
    proximity = np.maximum(0.0, 1.0 - np.sqrt(nearest_sq) / radius_m)
    traversable = cost < 1.0e6
    cost[traversable] += 10.0 * np.square(proximity[traversable])


def _add_route_blockers(
    planner: "GridPlanner",
    cost: np.ndarray,
    blocked_points: tuple[tuple[float, float], ...],
    start: tuple[int, int],
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
        if r0 <= start[0] < r1 and c0 <= start[1] < c1:
            continue
        cost[r0:r1, c0:c1] = 1.0e9


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


def _add_local_cost(planner: "GridPlanner", cost: np.ndarray) -> None:
    confirmed = set(getattr(planner, "_confirmed_local_obstacles", ()))
    current = set(getattr(planner, "_previous_local_obstacles", ())) - confirmed
    for idx in confirmed:
        cost[idx] = 1.0e6
    for idx in current:
        cost[idx] = max(float(cost[idx]), 80.0)
    if not confirmed:
        return
    obstacle_mask = np.zeros(cost.shape, dtype=np.bool_)
    rows, cols = zip(*confirmed)
    obstacle_mask[np.asarray(rows), np.asarray(cols)] = True
    clearance = _bounded_clearance(
        obstacle_mask,
        planner.grid.resolution_m,
        planner._wall_search_bounds,
    )
    preferred = (
        math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
    )
    deficit = np.maximum(0.0, preferred - clearance)
    traversable = cost < 1.0e6
    cost[traversable] += (
        280.0 * np.square(deficit[traversable] / preferred)
    )


def _bounded_clearance(
    obstacle_mask: np.ndarray,
    resolution_m: float,
    bounds: tuple[int, int, int, int] | None,
) -> np.ndarray:
    if bounds is None:
        clearance = np.asarray(
            distance_transform_edt(~obstacle_mask), dtype=np.float32,
        )
        clearance *= resolution_m
        return clearance
    row0, row1, col0, col1 = bounds
    region = (slice(row0, row1), slice(col0, col1))
    clearance = np.zeros(obstacle_mask.shape, dtype=np.float32)
    local = np.asarray(
        distance_transform_edt(~obstacle_mask[region]), dtype=np.float32,
    )
    local *= resolution_m
    clearance[region] = local
    return clearance


def _main_route_search(
    planner: "GridPlanner",
    cost: np.ndarray,
    geometry: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    pose: PoseEstimate,
    clearance_weight: float = 700.0,
    penalize_initial_footprint: bool = True,
    compact_obstacle_mask: np.ndarray | None = None,
    active_obstacle_mask: np.ndarray | None = None,
) -> tuple[tuple[tuple[int, int], ...], np.ndarray, np.ndarray]:
    started = time.perf_counter()
    route_cost = cost.copy()
    map_walls = planner.grid.cells == int(Cell.MAP_WALL)
    if (
        planner._wall_clearance_m is None
        or planner._wall_clearance_direction != planner.grid.direction
    ):
        wall_clearance = np.asarray(distance_transform_edt(~map_walls), dtype=np.float32)
        wall_clearance *= planner.grid.resolution_m
        wall_rows, wall_cols = np.nonzero(map_walls)
        planner._wall_clearance_m = wall_clearance
        planner._wall_clearance_direction = planner.grid.direction
        planner._wall_search_bounds = (
            (
                int(wall_rows.min()), int(wall_rows.max()) + 1,
                int(wall_cols.min()), int(wall_cols.max()) + 1,
            )
            if len(wall_rows) else None
        )
    wall_clearance = planner._wall_clearance_m
    wall_radius = (
        planner.robot_width_m * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + planner.grid.resolution_m * 0.5
        + ROUTE_UNCERTAINTY_MARGIN_M * 0.5
        if compact_obstacle_mask is not None
        else (
            math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5
            + FOOTPRINT_EXTRA_MARGIN_M
        )
    )
    physically_open = route_cost < 1.0e6
    route_cost[wall_clearance < wall_radius] = 1.0e9
    start_clearance = float(wall_clearance[start])
    if start_clearance < wall_radius:
        rows, cols = np.ogrid[:route_cost.shape[0], :route_cost.shape[1]]
        local_radius_cells = math.ceil(
            (wall_radius + 0.10) / planner.grid.resolution_m
        )
        near_start = (
            np.square(rows - start[0]) + np.square(cols - start[1])
            <= local_radius_cells * local_radius_cells
        )
        recoverable = (
            physically_open
            & near_start
            & (wall_clearance >= start_clearance - planner.grid.resolution_m * 0.5)
            & (wall_clearance < wall_radius)
        )
        deficit = (wall_radius - wall_clearance[recoverable]) / wall_radius
        route_cost[recoverable] = cost[recoverable] + 500.0 + 4500.0 * deficit
    persisted_obstacles = planner.grid.cells == int(Cell.UNKNOWN_OBSTRUCTION)
    dynamic_obstacles = persisted_obstacles.copy()
    for row, col in planner._confirmed_local_obstacles:
        dynamic_obstacles[row, col] = True
    if compact_obstacle_mask is not None:
        persisted_obstacles = persisted_obstacles.copy()
        persisted_obstacles[compact_obstacle_mask] = False
        dynamic_obstacles[compact_obstacle_mask] = False
        if active_obstacle_mask is not None:
            persisted_obstacles[active_obstacle_mask] = True
            dynamic_obstacles[active_obstacle_mask] = True
    obstacle_radius = (
        planner.robot_width_m * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + planner.grid.resolution_m * 0.5
    )
    if bool(dynamic_obstacles.any()):
        obstacle_clearance = _bounded_clearance(
            dynamic_obstacles,
            planner.grid.resolution_m,
            planner._wall_search_bounds,
        )
        pre_obstacle_open = route_cost < 1.0e6
        route_cost[obstacle_clearance < obstacle_radius] = 1.0e6
        start_obstacle_clearance = float(obstacle_clearance[start])
        if (
            start_obstacle_clearance
            < obstacle_radius + planner.grid.resolution_m
        ):
            rows, cols = np.ogrid[:route_cost.shape[0], :route_cost.shape[1]]
            local_radius_cells = math.ceil(
                (obstacle_radius + 0.10) / planner.grid.resolution_m
            )
            near_start = (
                np.square(rows - start[0]) + np.square(cols - start[1])
                <= local_radius_cells * local_radius_cells
            )
            recoverable = (
                pre_obstacle_open
                & near_start
                & (
                    obstacle_clearance
                    >= start_obstacle_clearance - planner.grid.resolution_m * 0.5
                )
                & (obstacle_clearance < obstacle_radius)
            )
            deficit = (
                obstacle_radius - obstacle_clearance[recoverable]
            ) / obstacle_radius
            route_cost[recoverable] = (
                cost[recoverable] + 500.0 + 4500.0 * deficit
            )
    if compact_obstacle_mask is None and bool(persisted_obstacles.any()):
        stable_obstacle_clearance = _bounded_clearance(
            persisted_obstacles,
            planner.grid.resolution_m,
            planner._wall_search_bounds,
        )
        _add_obstacle_wall_balance_cost(
            route_cost,
            wall_clearance,
            stable_obstacle_clearance,
            wall_radius,
            obstacle_radius,
        )
    footprint_overlap = _add_initial_footprint_cost(
        planner,
        route_cost,
        pose,
        goal,
        penalize=penalize_initial_footprint,
    )
    clearance = _bounded_clearance(
        geometry,
        planner.grid.resolution_m,
        planner._wall_search_bounds,
    )

    preferred_clearance = (
        math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + ROUTE_UNCERTAINTY_MARGIN_M
        + planner.grid.resolution_m / math.sqrt(2.0)
    )
    deficit = np.maximum(0.0, preferred_clearance - clearance)
    route_cost[route_cost < 1.0e6] += (
        clearance_weight * deficit[route_cost < 1.0e6] / preferred_clearance
    )
    route_cost[start] = 1.0
    goal = _nearest_clear_goal(route_cost, goal)
    stable_cost = _stable_cost(route_cost, goal)
    wall_bounds = planner._wall_search_bounds
    if wall_bounds is not None:
        wall_row0, wall_row1, wall_col0, wall_col1 = wall_bounds
        row0 = min(wall_row0, start[0], goal[0])
        row1 = max(wall_row1, start[0] + 1, goal[0] + 1)
        col0 = min(wall_col0, start[1], goal[1])
        col1 = max(wall_col1, start[1] + 1, goal[1] + 1)
        local_cost = stable_cost[row0:row1, col0:col1]
        local_start = (start[0] - row0, start[1] - col0)
        local_goal = (goal[0] - row0, goal[1] - col0)
        components, _ = cast(
            tuple[np.ndarray, int],
            label(local_cost < 1.0e6),
        )
        local_cells = ()
        if components[local_start] != 0 and components[local_start] == components[local_goal]:
            local_cells, _distances = scipy_grid_distance_path(
                local_cost, local_start, local_goal,
            )
        cells = tuple((row + row0, col + col0) for row, col in local_cells)
    else:
        cells, _distances = scipy_grid_distance_path(stable_cost, start, goal)
    primary_done = time.perf_counter()
    if not cells:
        cells = _recovery_path(route_cost, start, goal)
    recovery_done = time.perf_counter()
    connected = bool(cells) and cells[-1] == goal
    setattr(planner, "_heading_expansions", len(cells))
    setattr(
        planner,
        "_profile_heading_ms",
        (
            (primary_done - started) * 1000.0,
            (recovery_done - primary_done) * 1000.0,
            0.0,
        ),
    )
    setattr(planner, "_graph_search_connected", connected)
    return cells, clearance, footprint_overlap


def _add_obstacle_wall_balance_cost(
    cost: np.ndarray,
    wall_clearance: np.ndarray,
    obstacle_clearance: np.ndarray,
    wall_radius: float,
    obstacle_radius: float,
) -> None:
    corridor = (
        (cost < 1.0e6)
        & (wall_clearance < 0.70)
        & (obstacle_clearance < 0.70)
    )
    if not bool(corridor.any()):
        return
    wall_space = np.maximum(0.0, wall_clearance[corridor] - wall_radius)
    obstacle_space = np.maximum(
        0.0, obstacle_clearance[corridor] - obstacle_radius,
    )
    total_space = wall_space + obstacle_space
    imbalance = np.abs(wall_space - obstacle_space) / np.maximum(total_space, 0.05)
    cost[corridor] += 8.0 * np.square(imbalance)


def _add_initial_footprint_cost(
    planner: "GridPlanner",
    cost: np.ndarray,
    pose: PoseEstimate,
    goal: tuple[int, int],
    penalize: bool = True,
) -> np.ndarray:
    footprint_overlap = np.zeros_like(cost, dtype=np.float32)
    cell_margin = planner.grid.resolution_m * 0.5
    half_length = planner.robot_length_m * 0.5 + FOOTPRINT_EXTRA_MARGIN_M + cell_margin
    half_width = planner.robot_width_m * 0.5 + FOOTPRINT_EXTRA_MARGIN_M + cell_margin
    radius = int(math.ceil(math.hypot(half_length, half_width) / planner.grid.resolution_m))
    axis = np.arange(-radius, radius + 1, dtype=np.float32)
    dc, dr = np.meshgrid(axis, axis)
    dx = dc * planner.grid.resolution_m
    dy = -dr * planner.grid.resolution_m
    cy, sy = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    structure = (
        (np.abs(dx * cy + dy * sy) <= half_length)
        & (np.abs(-dx * sy + dy * cy) <= half_width)
    )
    blocked = cost >= 1.0e6
    center = planner.grid.world_to_cell(pose.x_m, pose.y_m)
    if center is None:
        return footprint_overlap
    local_cells = int(math.ceil(0.60 / planner.grid.resolution_m))
    row0 = max(0, center[0] - local_cells)
    row1 = min(cost.shape[0], center[0] + local_cells + 1)
    col0 = max(0, center[1] - local_cells)
    col1 = min(cost.shape[1], center[1] + local_cells + 1)
    halo = radius * 2
    source_row0 = max(0, row0 - halo)
    source_row1 = min(cost.shape[0], row1 + halo)
    source_col0 = max(0, col0 - halo)
    source_col1 = min(cost.shape[1], col1 + halo)
    invalid_source = binary_dilation(
        blocked[source_row0:source_row1, source_col0:source_col1],
        structure=structure,
        border_value=1,
    )
    overlap_source = np.asarray(
        distance_transform_edt(invalid_source), dtype=np.float32,
    ) * planner.grid.resolution_m
    local_slice = (
        slice(row0 - source_row0, row1 - source_row0),
        slice(col0 - source_col0, col1 - source_col0),
    )
    invalid_centers = invalid_source[local_slice]
    overlap_depth = overlap_source[local_slice]
    footprint_overlap[row0:row1, col0:col1] = overlap_depth
    rows, cols = np.indices((row1 - row0, col1 - col0), dtype=np.float32)
    rows += row0
    cols += col0
    world_x = (cols - planner.grid.origin_cell) * planner.grid.resolution_m
    world_y = (planner.grid.origin_cell - rows) * planner.grid.resolution_m
    local = np.hypot(world_x - pose.x_m, world_y - pose.y_m) <= 0.60
    local_cost = cost[row0:row1, col0:col1]
    affected = local & invalid_centers & (local_cost < 1.0e6)
    scale = max(min(half_length, half_width), planner.grid.resolution_m)
    if penalize:
        local_cost[affected] += 2000.0 + 4000.0 * np.square(
            overlap_depth[affected] / scale,
        )

    rel_x = world_x - pose.x_m
    rel_y = world_y - pose.y_m
    radius = np.hypot(rel_x, rel_y)
    local = (radius <= 0.45) & (local_cost < 1.0e6)
    goal_x, goal_y = planner.grid.cell_to_world(*goal)
    goal_dx = goal_x - pose.x_m
    goal_dy = goal_y - pose.y_m
    goal_distance = math.hypot(goal_dx, goal_dy)
    if goal_distance > 0.25:
        weight = np.square(np.maximum(0.0, 1.0 - radius / 0.45))
        progress = (rel_x * goal_dx + rel_y * goal_dy) / goal_distance
        away = np.maximum(0.0, -progress)
        local_cost[local] += (
            120.0 * weight[local]
            * np.square(away[local] / 0.15)
        )

    return footprint_overlap


def _continuous_path(
    planner: "GridPlanner",
    grid_path: np.ndarray,
    clearance: np.ndarray,
    footprint_overlap: np.ndarray,
) -> np.ndarray:
    if len(grid_path) < 2:
        return grid_path
    segment_lengths = np.linalg.norm(np.diff(grid_path, axis=0), axis=1)
    keep = np.concatenate(([True], segment_lengths > 1.0e-5))
    source = grid_path[keep]
    if len(source) < 2:
        return source
    source_lengths = np.linalg.norm(np.diff(source, axis=0), axis=1)
    source_distance = np.concatenate(([0.0], np.cumsum(source_lengths)))
    spacing_m = 0.10
    count = max(2, int(math.ceil(float(source_distance[-1]) / spacing_m)) + 1)
    wanted = np.linspace(0.0, float(source_distance[-1]), count, dtype=np.float32)
    initial = np.column_stack((
        np.interp(wanted, source_distance, source[:, 0]),
        np.interp(wanted, source_distance, source[:, 1]),
    )).astype(np.float32)
    points = initial.copy()
    initial_overlap = _sample_footprint_overlap(
        planner, footprint_overlap, initial[1:-1], order=1,
    )
    tangents = initial[2:] - initial[:-2]
    tangents /= np.maximum(np.linalg.norm(tangents, axis=1, keepdims=True), 1.0e-6)
    movable = np.ones(len(wanted) - 2, dtype=np.bool_)
    for _ in range(4):
        inner = points[1:-1]
        if not len(inner):
            break
        update = 0.35 * ((points[:-2] + points[2:]) * 0.5 - inner)
        update[~movable] = 0.0
        update_norm = np.linalg.norm(update, axis=1, keepdims=True)
        update *= np.minimum(1.0, 0.02 / np.maximum(update_norm, 1.0e-6))
        candidate = inner + update
        candidate = _bound_path_adjustment(initial[1:-1], candidate, tangents)
        candidate = _limit_footprint_overlap(
            planner, footprint_overlap, candidate, inner, initial_overlap,
        )
        converged = float(np.max(np.linalg.norm(candidate - inner, axis=1))) < 0.001
        points[1:-1] = candidate
        if converged:
            break
    grad_row, grad_col = np.gradient(clearance, edge_order=1)
    preferred = (
        math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
        + ROUTE_UNCERTAINTY_MARGIN_M
        + planner.grid.resolution_m / math.sqrt(2.0)
    )
    for _ in range(4):
        inner = points[1:-1]
        if not len(inner):
            break
        rows = planner.grid.origin_cell - inner[:, 1] / planner.grid.resolution_m
        cols = planner.grid.origin_cell + inner[:, 0] / planner.grid.resolution_m
        coordinates = np.vstack((rows, cols))
        local_clearance = np.asarray(map_coordinates(
            clearance, coordinates, order=1, mode="constant", cval=0.0,
        ), dtype=np.float32)
        gx = np.asarray(map_coordinates(
            grad_col, coordinates, order=1, mode="constant", cval=0.0,
        ), dtype=np.float32) / planner.grid.resolution_m
        gy = -np.asarray(map_coordinates(
            grad_row, coordinates, order=1, mode="constant", cval=0.0,
        ), dtype=np.float32) / planner.grid.resolution_m
        gradient = np.column_stack((gx, gy)).astype(np.float32)
        gradient_norm = np.linalg.norm(gradient, axis=1, keepdims=True)
        gradient /= np.maximum(gradient_norm, 1.0e-5)
        deficit = np.maximum(0.0, preferred - local_clearance)[:, None] / preferred
        smooth = (points[:-2] + points[2:]) * 0.5 - inner
        update = 0.35 * smooth + 0.018 * deficit * gradient
        update[~movable] = 0.0
        update_norm = np.linalg.norm(update, axis=1, keepdims=True)
        update *= np.minimum(1.0, 0.02 / np.maximum(update_norm, 1.0e-6))
        candidate = inner + update
        candidate = _bound_path_adjustment(initial[1:-1], candidate, tangents)
        candidate = _limit_footprint_overlap(
            planner, footprint_overlap, candidate, inner, initial_overlap,
        )
        converged = float(np.max(np.linalg.norm(candidate - inner, axis=1))) < 0.001
        points[1:-1] = candidate
        if converged:
            break

    hard_clearance = (
        math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5
        + FOOTPRINT_EXTRA_MARGIN_M
    )
    for _ in range(4):
        inner = points[1:-1]
        if not len(inner):
            break
        rows = planner.grid.origin_cell - inner[:, 1] / planner.grid.resolution_m
        cols = planner.grid.origin_cell + inner[:, 0] / planner.grid.resolution_m
        coordinates = np.vstack((rows, cols))
        local_clearance = np.asarray(map_coordinates(
            clearance, coordinates, order=1, mode="constant", cval=0.0,
        ), dtype=np.float32)
        deficit = np.maximum(0.0, hard_clearance - local_clearance)
        if not bool(np.any(deficit > 1.0e-4)):
            break
        gx = np.asarray(map_coordinates(
            grad_col, coordinates, order=1, mode="constant", cval=0.0,
        ), dtype=np.float32) / planner.grid.resolution_m
        gy = -np.asarray(map_coordinates(
            grad_row, coordinates, order=1, mode="constant", cval=0.0,
        ), dtype=np.float32) / planner.grid.resolution_m
        gradient = np.column_stack((gx, gy)).astype(np.float32)
        gradient /= np.maximum(np.linalg.norm(gradient, axis=1, keepdims=True), 1.0e-5)
        update = deficit[:, None] * gradient
        update[~movable] = 0.0
        update_norm = np.linalg.norm(update, axis=1, keepdims=True)
        update *= np.minimum(1.0, 0.02 / np.maximum(update_norm, 1.0e-6))
        candidate = _bound_path_adjustment(
            initial[1:-1], points[1:-1] + update, tangents,
        )
        candidate = _limit_footprint_overlap(
            planner, footprint_overlap, candidate, inner, initial_overlap,
        )
        points[1:-1] = candidate

    _relax_path_curvature(
        planner,
        points,
        initial,
        tangents,
        movable,
        clearance,
        hard_clearance,
        footprint_overlap,
        initial_overlap,
    )

    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    distance = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    count = max(2, int(math.ceil(float(distance[-1]) / planner.trajectory_step_m)) + 1)
    wanted = np.linspace(0.0, float(distance[-1]), count, dtype=np.float32)
    path = np.column_stack((
        np.interp(wanted, distance, points[:, 0]),
        np.interp(wanted, distance, points[:, 1]),
    )).astype(np.float32)
    return _spline_path_candidate(
        planner, points, distance, path, clearance, hard_clearance,
        footprint_overlap,
    )


def _sample_footprint_overlap(
    planner: "GridPlanner",
    field: np.ndarray,
    points: np.ndarray,
    order: int,
) -> np.ndarray:
    if not len(points):
        return np.empty(0, dtype=np.float32)
    rows = planner.grid.origin_cell - points[:, 1] / planner.grid.resolution_m
    cols = planner.grid.origin_cell + points[:, 0] / planner.grid.resolution_m
    return np.asarray(map_coordinates(
        field,
        np.vstack((rows, cols)),
        order=order,
        mode="constant",
        cval=1.0e6,
    ), dtype=np.float32)


def _limit_footprint_overlap(
    planner: "GridPlanner",
    footprint_overlap: np.ndarray,
    candidate: np.ndarray,
    fallback: np.ndarray,
    limit: np.ndarray,
) -> np.ndarray:
    overlap = _sample_footprint_overlap(
        planner, footprint_overlap, candidate, order=1,
    )
    return np.where(
        (overlap <= limit + 1.0e-4)[:, None], candidate, fallback,
    )


def _spline_path_candidate(
    planner: "GridPlanner",
    points: np.ndarray,
    distance: np.ndarray,
    path: np.ndarray,
    clearance: np.ndarray,
    hard_clearance: float,
    footprint_overlap: np.ndarray,
) -> np.ndarray:
    if len(points) < 4 or distance[-1] <= 1.0e-5:
        return path
    parameter = distance / distance[-1]
    sample_parameter = np.linspace(0.0, 1.0, len(path), dtype=np.float32)
    weights = np.ones(len(points), dtype=np.float32)
    weights[[0, -1]] = 100.0
    best = path
    best_curvature = _path_max_curvature(path)
    base_length = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
    base_rows = planner.grid.origin_cell - path[:, 1] / planner.grid.resolution_m
    base_cols = planner.grid.origin_cell + path[:, 0] / planner.grid.resolution_m
    base_clearance = np.asarray(map_coordinates(
        clearance,
        np.vstack((base_rows, base_cols)),
        order=1,
        mode="constant",
        cval=0.0,
    ), dtype=np.float32)
    minimum_clearance = np.minimum(
        hard_clearance,
        np.maximum(0.0, base_clearance - 0.005),
    )
    base_overlap = _sample_footprint_overlap(
        planner, footprint_overlap, path, order=1,
    )
    for smoothing in (0.0025, 0.005, 0.01, 0.02, 0.04):
        try:
            spline, _ = splprep(
                (points[:, 0], points[:, 1]),
                u=parameter,
                w=weights,
                s=smoothing,
                k=min(3, len(points) - 1),
            )
            x_m, y_m = splev(sample_parameter, spline)
        except (TypeError, ValueError):
            continue
        candidate = np.column_stack((x_m, y_m)).astype(np.float32)
        candidate[[0, -1]] = path[[0, -1]]
        candidate_segments = np.linalg.norm(np.diff(candidate, axis=0), axis=1)
        candidate_distance = np.concatenate(([0.0], np.cumsum(candidate_segments)))
        if candidate_distance[-1] <= 1.0e-5:
            continue
        uniform_distance = np.linspace(
            0.0, float(candidate_distance[-1]), len(candidate), dtype=np.float32,
        )
        candidate = np.column_stack((
            np.interp(uniform_distance, candidate_distance, candidate[:, 0]),
            np.interp(uniform_distance, candidate_distance, candidate[:, 1]),
        )).astype(np.float32)
        candidate[[0, -1]] = path[[0, -1]]
        if float(np.max(np.linalg.norm(candidate - path, axis=1))) > 0.15:
            continue
        candidate_length = float(np.sum(np.linalg.norm(np.diff(candidate, axis=0), axis=1)))
        if candidate_length > base_length * 1.05:
            continue
        rows = planner.grid.origin_cell - candidate[:, 1] / planner.grid.resolution_m
        cols = planner.grid.origin_cell + candidate[:, 0] / planner.grid.resolution_m
        candidate_clearance = np.asarray(map_coordinates(
            clearance,
            np.vstack((rows, cols)),
            order=1,
            mode="constant",
            cval=0.0,
        ), dtype=np.float32)
        if bool(np.any(candidate_clearance < minimum_clearance)):
            continue
        candidate_overlap = _sample_footprint_overlap(
            planner, footprint_overlap, candidate, order=1,
        )
        overlap_worse = candidate_overlap > base_overlap + 1.0e-4
        if bool(np.any(overlap_worse & (candidate_clearance < hard_clearance))):
            continue
        curvature = _path_max_curvature(candidate)
        if curvature < best_curvature:
            best = candidate
            best_curvature = curvature
    return best


def _bound_path_adjustment(
    initial: np.ndarray,
    candidate: np.ndarray,
    tangents: np.ndarray,
) -> np.ndarray:
    displacement = candidate - initial
    along = np.sum(displacement * tangents, axis=1, keepdims=True)
    lateral = displacement - along * tangents
    lateral_norm = np.linalg.norm(lateral, axis=1, keepdims=True)
    lateral *= np.minimum(1.0, 0.15 / np.maximum(lateral_norm, 1.0e-6))
    return initial + lateral + np.clip(along, -0.02, 0.02) * tangents


def _relax_path_curvature(
    planner: "GridPlanner",
    points: np.ndarray,
    initial: np.ndarray,
    tangents: np.ndarray,
    movable: np.ndarray,
    clearance: np.ndarray,
    hard_clearance: float,
    footprint_overlap: np.ndarray,
    initial_overlap: np.ndarray,
) -> None:
    if len(points) < 3:
        return
    max_curvature = (
        math.tan(planner.max_steering_angle_rad) / planner.wheelbase_m
    ) * 0.90
    base_rows = planner.grid.origin_cell - points[1:-1, 1] / planner.grid.resolution_m
    base_cols = planner.grid.origin_cell + points[1:-1, 0] / planner.grid.resolution_m
    base_clearance = np.asarray(map_coordinates(
        clearance,
        np.vstack((base_rows, base_cols)),
        order=1,
        mode="constant",
        cval=0.0,
    ), dtype=np.float32)
    minimum_clearance = np.minimum(
        hard_clearance,
        np.maximum(0.0, base_clearance - 0.005),
    )
    for _ in range(4):
        first = points[1:-1] - points[:-2]
        second = points[2:] - points[1:-1]
        chord = points[2:] - points[:-2]
        denominator = (
            np.linalg.norm(first, axis=1)
            * np.linalg.norm(second, axis=1)
            * np.linalg.norm(chord, axis=1)
        )
        cross = np.abs(first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0])
        curvature = np.divide(
            2.0 * cross,
            denominator,
            out=np.zeros_like(cross),
            where=denominator > 1.0e-6,
        )
        violating = movable & (curvature > max_curvature)
        if not bool(np.any(violating)):
            return
        inner = points[1:-1]
        update = 0.45 * ((points[:-2] + points[2:]) * 0.5 - inner)
        update[~violating] = 0.0
        update_norm = np.linalg.norm(update, axis=1, keepdims=True)
        update *= np.minimum(1.0, 0.015 / np.maximum(update_norm, 1.0e-6))
        candidate = inner + update
        candidate = _bound_path_adjustment(initial[1:-1], candidate, tangents)
        rows = planner.grid.origin_cell - candidate[:, 1] / planner.grid.resolution_m
        cols = planner.grid.origin_cell + candidate[:, 0] / planner.grid.resolution_m
        candidate_clearance = np.asarray(map_coordinates(
            clearance,
            np.vstack((rows, cols)),
            order=1,
            mode="constant",
            cval=0.0,
        ), dtype=np.float32)
        candidate_overlap = _sample_footprint_overlap(
            planner, footprint_overlap, candidate, order=1,
        )
        accepted = (
            violating
            & (candidate_clearance >= minimum_clearance)
            & (candidate_overlap <= initial_overlap + 1.0e-4)
        )
        if not bool(np.any(accepted)):
            return
        movement = np.linalg.norm(candidate[accepted] - inner[accepted], axis=1)
        inner[accepted] = candidate[accepted]
        if float(np.max(movement, initial=0.0)) < 0.0005:
            return


def _route_command(
    planner: "GridPlanner",
    pose: PoseEstimate,
    path: np.ndarray,
    forced_direction: int | None = None,
) -> tuple[int, float]:
    if len(path) < 2:
        return planner._last_motion_direction, 0.0
    start = (float(path[0, 0]), float(path[0, 1]))
    if forced_direction is not None:
        first = (float(path[1, 0]), float(path[1, 1]))
        dx, dy = first[0] - start[0], first[1] - start[1]
        if math.hypot(dx, dy) <= 1.0e-6:
            return forced_direction, 0.0
        target_yaw = math.atan2(dy, dx)
        travel_yaw = pose.yaw_rad if forced_direction > 0 else pose.yaw_rad + math.pi
        alpha = math.atan2(
            math.sin(target_yaw - travel_yaw), math.cos(target_yaw - travel_yaw),
        )
        steering = forced_direction * math.atan2(
            2.0 * planner.wheelbase_m * math.sin(alpha),
            max(math.hypot(dx, dy), planner.grid.resolution_m),
        )
        return forced_direction, max(
            -planner.max_steering_angle_rad,
            min(planner.max_steering_angle_rad, steering),
        )
    target = start
    direction_target = start
    distance = 0.0
    target_distance = 0.0
    direction_distance = 0.0
    previous = start
    for raw_point in path[1:]:
        point = (float(raw_point[0]), float(raw_point[1]))
        distance += math.hypot(point[0] - previous[0], point[1] - previous[1])
        if target_distance < 0.20:
            target = point
            target_distance = distance
        if direction_distance < 0.22:
            direction_target = point
            direction_distance = distance
        previous = point
        if distance >= 0.75:
            break
    dx, dy = target[0] - start[0], target[1] - start[1]
    target_yaw = math.atan2(dy, dx)
    lookahead = max(math.hypot(dx, dy), planner.grid.resolution_m)
    direction_dx = direction_target[0] - start[0]
    direction_dy = direction_target[1] - start[1]
    along_heading = (
        direction_dx * math.cos(pose.yaw_rad)
        + direction_dy * math.sin(pose.yaw_rad)
    )
    reverse_deadband = 0.75 if planner.grid.direction == Direction.UNKNOWN else 0.45
    reverse_threshold = -reverse_deadband * max(
        math.hypot(direction_dx, direction_dy), 1.0e-6,
    )
    direction = (
        forced_direction
        if forced_direction is not None
        else (-1 if along_heading < reverse_threshold else 1)
    )
    travel_yaw = pose.yaw_rad if direction > 0 else pose.yaw_rad + math.pi
    alpha = math.atan2(
        math.sin(target_yaw - travel_yaw), math.cos(target_yaw - travel_yaw),
    )
    desired_steering = direction * math.atan2(
        2.0 * planner.wheelbase_m * math.sin(alpha), lookahead,
    )
    desired_steering = max(
        -planner.max_steering_angle_rad,
        min(planner.max_steering_angle_rad, desired_steering),
    )
    return direction, desired_steering


def _recovery_path(
    cost: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> tuple[tuple[int, int], ...]:
    recovery_cost = cost.copy()
    recovery_cost[start] = 1.0
    clear = recovery_cost < 1.0e6
    seed = np.zeros(clear.shape, dtype=np.bool_)
    seed[start] = True
    cross = np.asarray(((0, 1, 0), (1, 1, 1), (0, 1, 0)), dtype=np.bool_)
    reachable = np.asarray(
        binary_dilation(seed, structure=cross, iterations=16, mask=clear),
        dtype=np.bool_,
    )
    rows, cols = np.nonzero(reachable)
    sr, sc = start
    distance_cells = np.hypot(rows - sr, cols - sc)
    goal_distance = np.hypot(rows - goal[0], cols - goal[1])
    clearance_grid = np.asarray(distance_transform_edt(clear), dtype=np.float32)
    clearance = clearance_grid[rows, cols]
    preferred = (distance_cells >= 6.0) & (distance_cells <= 16.0) & (clearance >= 3.0)
    candidates = np.nonzero(preferred)[0]
    if not len(candidates):
        candidates = np.nonzero(distance_cells >= 1.0)[0]
    if not len(candidates):
        return (start,)
    scores = goal_distance[candidates] - np.minimum(clearance[candidates], 8.0) * 0.8
    index = int(candidates[int(np.argmin(scores))])
    target = (int(rows[index]), int(cols[index]))
    row0, row1 = int(rows.min()), int(rows.max()) + 1
    col0, col1 = int(cols.min()), int(cols.max()) + 1
    local_cost = recovery_cost[row0:row1, col0:col1].copy()
    local_cost[~reachable[row0:row1, col0:col1]] = 1.0e8
    local_start = (start[0] - row0, start[1] - col0)
    local_target = (target[0] - row0, target[1] - col0)
    local_path, _ = scipy_grid_distance_path(
        _stable_cost(local_cost, local_target), local_start, local_target,
    )
    return (
        tuple((row + row0, col + col0) for row, col in local_path)
        if local_path else (start,)
    )


def _stable_cost(cost: np.ndarray, goal: tuple[int, int]) -> np.ndarray:
    rows, cols = np.indices(cost.shape, dtype=np.float32)
    h = np.hypot(rows - float(goal[0]), cols - float(goal[1]))
    h /= max(float(max(cost.shape)), 1.0)
    stable = (np.round(cost.astype(np.float32, copy=False) / 4.5) * 4.5).astype(np.float32)
    stable += h * 1.0e-3
    return stable
