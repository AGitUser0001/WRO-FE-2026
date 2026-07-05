from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from .grid import Cell
from .local_grid import LocalGrid
from .localize_types import PoseEstimate

if TYPE_CHECKING:
    from .trajectory import TrajectoryMixin


def optimize_path(
    planner: "TrajectoryMixin",
    pose: PoseEstimate,
    guide_points: tuple[tuple[float, float], ...],
    buffer_m: float,
) -> tuple[tuple[tuple[float, float], ...], float, int]:
    horizon_m = _guide_length(pose, guide_points)
    count = _sample_count(planner, horizon_m)
    target = _resample_guide(pose, guide_points, count, planner.trajectory_step_m)
    path = target.copy()
    obstacles = _obstacle_points(planner, pose, horizon_m)
    influence = max(buffer_m + planner.robot_width_m * 0.5 + 0.22, 0.30)
    clearance = max(buffer_m + math.hypot(planner.robot_length_m, planner.robot_width_m) * 0.5, 0.22)
    half_len = planner.robot_length_m * 0.5 + buffer_m + planner.grid.resolution_m
    half_width = planner.robot_width_m * 0.5 + buffer_m + planner.grid.resolution_m
    for _ in range(36):
        grad = (target - path) * 0.25
        grad += _obstacle_gradient(path, obstacles, influence) * 0.45
        grad += _spacing_gradient(path, planner.trajectory_step_m) * 0.42
        grad += _progress_gradient(path, target, planner.trajectory_step_m) * 1.10
        grad += _smooth_gradient(path) * 0.18
        grad += _curvature_gradient(path) * 0.10
        path[1:] += np.clip(grad[1:], -0.08, 0.08)
        path[0] = (pose.x_m, pose.y_m)
        _enforce_step_lengths(path, planner.trajectory_step_m)
        _clamp_to_guide(path, target, 0.36)
        _enforce_obstacle_clearance(path, obstacles, clearance)
        _enforce_footprint_clearance(path, obstacles, half_len, half_width)
        _enforce_start_feasibility(path, pose, planner)
    points = tuple((float(x), float(y)) for x, y in path)
    steering = _steering_from_path(planner, pose, points)
    direction = planner._motion_direction_for_path(pose.x_m, pose.y_m, pose.yaw_rad, points)
    return points, steering, direction


def _guide_length(pose: PoseEstimate, guide_points: tuple[tuple[float, float], ...]) -> float:
    if not guide_points:
        return 0.0
    total = 0.0
    last = (pose.x_m, pose.y_m)
    for point in guide_points:
        total += math.hypot(point[0] - last[0], point[1] - last[1])
        last = point
    return total


def _sample_count(planner: "TrajectoryMixin", horizon_m: float) -> int:
    count = int(math.ceil(max(horizon_m, planner.trajectory_step_m) / planner.trajectory_step_m)) + 1
    return max(2, min(count, planner.trajectory_steps))


def _resample_guide(
    pose: PoseEstimate,
    guide_points: tuple[tuple[float, float], ...],
    count: int,
    step_m: float,
) -> np.ndarray:
    if not guide_points:
        return np.asarray(
            [(pose.x_m + math.cos(pose.yaw_rad) * step_m * i, pose.y_m + math.sin(pose.yaw_rad) * step_m * i) for i in range(count)],
            dtype=np.float32,
        )
    src = np.asarray(((pose.x_m, pose.y_m), *guide_points), dtype=np.float32)
    seg = np.linalg.norm(np.diff(src, axis=0), axis=1)
    dist = np.concatenate(([0.0], np.cumsum(seg)))
    wanted = np.arange(count, dtype=np.float32) * step_m
    wanted = np.minimum(wanted, dist[-1])
    out = np.empty((count, 2), dtype=np.float32)
    out[:, 0] = np.interp(wanted, dist, src[:, 0])
    out[:, 1] = np.interp(wanted, dist, src[:, 1])
    return out


def _obstacle_points(planner: "TrajectoryMixin", pose: PoseEstimate, horizon_m: float) -> np.ndarray:
    pts: list[tuple[float, float]] = []
    _add_global_obstacles(planner, pose, pts, horizon_m)
    _add_local_obstacles(planner, pts)
    _add_motion_blocks(planner, pose, pts)
    if not pts:
        return np.empty((0, 2), dtype=np.float32)
    return np.asarray(pts, dtype=np.float32)


def _add_global_obstacles(planner: "TrajectoryMixin", pose: PoseEstimate, pts: list[tuple[float, float]], horizon_m: float) -> None:
    radius_cells = int(math.ceil((max(horizon_m, planner.lookahead_m) + 0.8) / planner.grid.resolution_m))
    center = planner.grid.world_to_cell(pose.x_m, pose.y_m)
    if center is None:
        return
    r0 = max(0, center[0] - radius_cells)
    r1 = min(planner.grid.size_cells, center[0] + radius_cells + 1)
    c0 = max(0, center[1] - radius_cells)
    c1 = min(planner.grid.size_cells, center[1] + radius_cells + 1)
    block = (planner.grid.cells[r0:r1, c0:c1] != int(Cell.FREE)) | planner.grid.live_mask[r0:r1, c0:c1]
    rows, cols = np.nonzero(block)
    keep = max(1, int(round(0.10 / max(planner.grid.resolution_m, 1.0e-6))))
    for rr, cc in zip(rows.tolist(), cols.tolist()):
        if (rr + cc) % keep != 0:
            continue
        pts.append(planner.grid.cell_to_world(r0 + rr, c0 + cc))


def _add_local_obstacles(planner: "TrajectoryMixin", pts: list[tuple[float, float]]) -> None:
    if planner._local_collision_grid is None or planner._local_collision_origin is None:
        return
    local = planner._local_collision_grid
    if not isinstance(local, LocalGrid):
        return
    ox, oy, oyaw = planner._local_collision_origin
    cy = math.cos(oyaw)
    sy = math.sin(oyaw)
    for row, col, value in local.non_free_cells():
        if value == Cell.FREE:
            continue
        lx, ly = local.cell_to_local(row, col)
        pts.append((ox + lx * cy - ly * sy, oy + lx * sy + ly * cy))


def _add_motion_blocks(planner: "TrajectoryMixin", pose: PoseEstimate, pts: list[tuple[float, float]]) -> None:
    points = getattr(planner, "motion_block_points", lambda: ())()
    for x_m, y_m in points:
        if math.hypot(x_m - pose.x_m, y_m - pose.y_m) >= planner.grid.resolution_m * 1.2:
            pts.append((x_m, y_m))


def _obstacle_gradient(path: np.ndarray, obstacles: np.ndarray, influence: float) -> np.ndarray:
    if obstacles.size == 0:
        return np.zeros_like(path)
    diff = path[:, None, :] - obstacles[None, :, :]
    dist = np.linalg.norm(diff, axis=2)
    active = (dist > 1e-4) & (dist < influence)
    strength = np.where(active, ((influence - dist) / influence) ** 2 / np.maximum(dist, 1e-4), 0.0)
    return (diff * strength[:, :, None]).sum(axis=1)


def _spacing_gradient(path: np.ndarray, step_m: float) -> np.ndarray:
    grad = np.zeros_like(path)
    seg = path[1:] - path[:-1]
    dist = np.linalg.norm(seg, axis=1)
    err = dist - step_m
    unit = seg / np.maximum(dist[:, None], 1e-4)
    force = unit * err[:, None]
    grad[:-1] += force
    grad[1:] -= force
    return grad


def _progress_gradient(path: np.ndarray, target: np.ndarray, step_m: float) -> np.ndarray:
    grad = np.zeros_like(path)
    desired = target[1:] - target[:-1]
    desired_len = np.linalg.norm(desired, axis=1)
    unit = desired / np.maximum(desired_len[:, None], 1e-4)
    actual = path[1:] - path[:-1]
    progress = (actual * unit).sum(axis=1)
    deficit = np.clip(step_m * 0.70 - progress, 0.0, step_m)
    grad[1:] += unit * deficit[:, None]
    return grad


def _clamp_to_guide(path: np.ndarray, target: np.ndarray, max_offset_m: float) -> None:
    middle = path[1:-1]
    guide = target[1:-1]
    delta = middle - guide
    dist = np.linalg.norm(delta, axis=1)
    mask = dist > max_offset_m
    if bool(mask.any()):
        middle[mask] = guide[mask] + delta[mask] / dist[mask, None] * max_offset_m


def _enforce_obstacle_clearance(path: np.ndarray, obstacles: np.ndarray, clearance_m: float) -> None:
    if obstacles.size == 0 or len(path) <= 2:
        return
    middle = path[1:-1]
    diff = middle[:, None, :] - obstacles[None, :, :]
    dist = np.linalg.norm(diff, axis=2)
    nearest = np.argmin(dist, axis=1)
    rows = np.arange(middle.shape[0])
    nearest_dist = dist[rows, nearest]
    mask = nearest_dist < clearance_m
    if not bool(mask.any()):
        return
    push = diff[rows[mask], nearest[mask]]
    push_len = np.maximum(nearest_dist[mask], 1.0e-4)
    middle[mask] += push / push_len[:, None] * (clearance_m - nearest_dist[mask])[:, None] * 0.65


def _enforce_footprint_clearance(path: np.ndarray, obstacles: np.ndarray, half_len: float, half_width: float) -> None:
    if obstacles.size == 0 or len(path) <= 2:
        return
    radius = math.hypot(half_len, half_width)
    for i in range(1, len(path)):
        prev = path[i - 1]
        nxt = path[i + 1] if i + 1 < len(path) else path[i]
        tangent = nxt - prev
        tlen = float(np.linalg.norm(tangent))
        if tlen < 1.0e-4:
            continue
        cx = float(tangent[0] / tlen); sy = float(tangent[1] / tlen)
        delta = obstacles - path[i]
        near = np.linalg.norm(delta, axis=1) < radius
        if not bool(near.any()):
            continue
        local = delta[near]
        forward = local[:, 0] * cx + local[:, 1] * sy
        lateral = -local[:, 0] * sy + local[:, 1] * cx
        inside = (np.abs(forward) < half_len) & (np.abs(lateral) < half_width)
        if not bool(inside.any()):
            continue
        f = forward[inside]; l = lateral[inside]
        pen_f = half_len - np.abs(f)
        pen_l = half_width - np.abs(l)
        use_lateral = pen_l <= pen_f
        push_f = np.where(use_lateral, 0.0, -np.sign(f) * pen_f)
        push_l = np.where(use_lateral, -np.sign(l) * pen_l, 0.0)
        push_x = push_f * cx - push_l * sy
        push_y = push_f * sy + push_l * cx
        path[i, 0] += float(np.mean(push_x)) * 0.75
        path[i, 1] += float(np.mean(push_y)) * 0.75


def _enforce_start_feasibility(path: np.ndarray, pose: PoseEstimate, planner: "TrajectoryMixin") -> None:
    if len(path) < 3:
        return
    direction = _start_direction(path, pose)
    yaw = pose.yaw_rad if direction >= 0 else pose.yaw_rad + math.pi
    cx = math.cos(yaw); sy = math.sin(yaw)
    max_curve = math.tan(planner.max_steering_angle_rad) / max(planner.wheelbase_m, 0.01)
    last_forward = 0.0
    for i in range(1, min(len(path), 8)):
        dx = float(path[i, 0] - pose.x_m); dy = float(path[i, 1] - pose.y_m)
        forward = max(last_forward + planner.trajectory_step_m * 0.65, dx * cx + dy * sy)
        lateral = -dx * sy + dy * cx
        limit = 0.5 * max_curve * forward * forward + 0.03
        lateral = float(np.clip(lateral, -limit, limit))
        path[i, 0] = pose.x_m + forward * cx - lateral * sy
        path[i, 1] = pose.y_m + forward * sy + lateral * cx
        last_forward = forward


def _start_direction(path: np.ndarray, pose: PoseEstimate) -> int:
    cx = math.cos(pose.yaw_rad); sy = math.sin(pose.yaw_rad)
    prev_x = pose.x_m; prev_y = pose.y_m
    remaining = 0.85
    signed = 0.0
    total = 0.0
    for point in path[1:]:
        dx = float(point[0] - prev_x); dy = float(point[1] - prev_y)
        seg = math.hypot(dx, dy)
        if seg <= 1.0e-6:
            prev_x = float(point[0]); prev_y = float(point[1])
            continue
        used = min(seg, remaining)
        signed += (dx * cx + dy * sy) / seg * used
        total += used
        remaining -= used
        if remaining <= 1.0e-6:
            break
        prev_x = float(point[0]); prev_y = float(point[1])
    return -1 if total > 0.05 and signed < -0.35 * total else 1


def _smooth_gradient(path: np.ndarray) -> np.ndarray:
    grad = np.zeros_like(path)
    grad[1:-1] = (path[:-2] + path[2:] - path[1:-1] * 2.0)
    return grad


def _curvature_gradient(path: np.ndarray) -> np.ndarray:
    grad = np.zeros_like(path)
    if len(path) < 4:
        return grad
    bend = path[:-2] - 2.0 * path[1:-1] + path[2:]
    grad[1:-1] -= bend
    return grad


def _enforce_step_lengths(path: np.ndarray, step_m: float) -> None:
    for i in range(1, len(path)):
        delta = path[i] - path[i - 1]
        dist = float(np.linalg.norm(delta))
        if dist < 1e-4:
            path[i] = path[i - 1] + (step_m, 0.0)
        else:
            path[i] = path[i - 1] + delta / dist * step_m


def _steering_from_path(
    planner: "TrajectoryMixin",
    pose: PoseEstimate,
    points: tuple[tuple[float, float], ...],
) -> float:
    target = _point_at_distance(points, 0.55)
    return planner._steering_to_target(pose.x_m, pose.y_m, pose.yaw_rad, target)


def _point_at_distance(points: tuple[tuple[float, float], ...], distance_m: float) -> tuple[float, float]:
    prev = points[0]
    remaining = distance_m
    for point in points[1:]:
        seg = math.hypot(point[0] - prev[0], point[1] - prev[1])
        if seg >= remaining and seg > 1e-6:
            t = remaining / seg
            return prev[0] + (point[0] - prev[0]) * t, prev[1] + (point[1] - prev[1]) * t
        remaining -= seg
        prev = point
    return points[-1]
