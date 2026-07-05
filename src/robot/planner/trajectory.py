from __future__ import annotations

import math

from .grid import Cell, GridMap
from .localize_types import PoseEstimate
from .path_opt import optimize_path


class TrajectoryMixin:
    grid: GridMap
    safety_buffer_m: float
    lookahead_m: float
    wheelbase_m: float
    max_steering_angle_rad: float
    robot_length_m: float
    robot_width_m: float
    trajectory_steps: int
    trajectory_length_m: float
    trajectory_step_m: float
    command_delay_s: float
    planning_speed_mps: float
    _traversable_cache: dict[tuple[int, int], bool]
    _footprint_local_cache: dict[float, tuple[tuple[float, float], ...]]
    _local_collision_grid: object | None
    _local_collision_origin: tuple[float, float, float] | None
    _last_steering_rad: float
    _last_motion_direction: int
    _last_trajectory: tuple[tuple[float, float], ...]

    def _simulate_trajectory(
        self,
        pose: PoseEstimate,
        guide_points: tuple[tuple[float, float], ...],
        motion_direction: int,
    ) -> tuple[tuple[tuple[float, float], ...], float, int]:
        target = self._target_for_sim_pose(pose.x_m, pose.y_m, pose.yaw_rad, guide_points, motion_direction)
        desired = self._steering_to_target(pose.x_m, pose.y_m, pose.yaw_rad, target)
        return self._planned_path(pose, guide_points, desired, motion_direction, self.safety_buffer_m)

    def _planned_path(
        self,
        pose: PoseEstimate,
        guide_points: tuple[tuple[float, float], ...],
        desired: float,
        preferred_direction: int,
        buffer_m: float,
    ) -> tuple[tuple[tuple[float, float], ...], float, int]:
        for candidate_buffer in self._buffer_candidates(buffer_m):
            result = self._planned_path_with_buffer(pose, guide_points, desired, preferred_direction, candidate_buffer)
            if len(result[0]) >= 2:
                return result
        return result # type: ignore

    def _planned_path_with_buffer(
        self,
        pose: PoseEstimate,
        guide_points: tuple[tuple[float, float], ...],
        desired: float,
        preferred_direction: int,
        buffer_m: float,
    ) -> tuple[tuple[tuple[float, float], ...], float, int]:
        _ = desired, preferred_direction
        return optimize_path(self, pose, guide_points, buffer_m)

    def _straight_best_effort_path(self, pose: PoseEstimate) -> tuple[tuple[float, float], ...]:
        return tuple(
            (
                pose.x_m + math.cos(pose.yaw_rad) * self.trajectory_step_m * i,
                pose.y_m + math.sin(pose.yaw_rad) * self.trajectory_step_m * i,
            )
            for i in range(self.trajectory_steps)
        )

    def _buffer_candidates(self, buffer_m: float) -> tuple[float, ...]:
        raw = (buffer_m, min(buffer_m, 0.10), min(buffer_m, 0.06))
        values: list[float] = []
        for value in raw:
            if value > 1e-4 and all(abs(value - existing) > 1e-4 for existing in values):
                values.append(value)
        return tuple(values)

    @staticmethod
    def _motion_direction_for_path(
        x_m: float,
        y_m: float,
        yaw_rad: float,
        path: tuple[tuple[float, float], ...],
    ) -> int:
        if len(path) < 2:
            return 1
        cx = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        remaining = 0.85
        prev = (x_m, y_m)
        signed = 0.0
        total = 0.0
        for point in path[1:]:
            seg = math.hypot(point[0] - prev[0], point[1] - prev[1])
            if seg <= 1e-6:
                prev = point
                continue
            used = min(seg, remaining)
            signed += ((point[0] - prev[0]) * cx + (point[1] - prev[1]) * sy) / seg * used
            total += used
            remaining -= used
            if remaining <= 1e-6:
                break
            prev = point
        return -1 if total > 0.05 and signed < -0.35 * total else 1

    def _delay_steps(self) -> int:
        delay_m = self.command_delay_s * self.planning_speed_mps
        return max(1, int(math.ceil(delay_m / self.trajectory_step_m)))

    def _steering_candidates(self, desired: float) -> tuple[float, ...]:
        max_angle = self.max_steering_angle_rad
        values = (
            desired, 0.0, desired * 0.5, desired * 1.4,
            desired - max_angle * 0.35, desired + max_angle * 0.35,
            -max_angle, -max_angle * 0.85, -max_angle * 0.65, -max_angle * 0.35,
            max_angle * 0.35, max_angle * 0.65, max_angle * 0.85, max_angle,
        )
        unique: list[float] = []
        for value in values:
            clipped = max(-max_angle, min(max_angle, value))
            if all(abs(clipped - existing) > 1e-4 for existing in unique):
                unique.append(clipped)
        return tuple(unique)

    def _footprint_local_points(self, buffer_m: float) -> tuple[tuple[float, float], ...]:
        key = round(buffer_m, 4)
        cached = self._footprint_local_cache.get(key)
        if cached is not None:
            return cached
        half_l = self.robot_length_m * 0.5 + buffer_m
        half_w = self.robot_width_m * 0.5 + buffer_m
        points = (
            (0.0, 0.0),
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, half_w),
            (-half_l, -half_w),
            (half_l, 0.0),
            (-half_l, 0.0),
            (0.0, half_w),
            (0.0, -half_w),
        )
        self._footprint_local_cache[key] = points
        return points

    def _target_for_sim_pose(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        guide_points: tuple[tuple[float, float], ...],
        motion_direction: int,
    ) -> tuple[float, float]:
        if not guide_points:
            return (
                x_m + math.cos(yaw_rad) * self.lookahead_m,
                y_m + math.sin(yaw_rad) * self.lookahead_m,
            )
        best = guide_points[-1]
        best_score = math.inf
        for point in guide_points:
            dx = point[0] - x_m
            dy = point[1] - y_m
            dist = math.hypot(dx, dy)
            score = abs(dist - self.lookahead_m)
            if score < best_score:
                best = point
                best_score = score
        return best

    def _steering_to_target(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        target: tuple[float, float],
    ) -> float:
        dx = target[0] - x_m
        dy = target[1] - y_m
        local_x = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
        local_y = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
        lookahead = max(math.hypot(local_x, local_y), 0.05)
        curvature = 2.0 * local_y / (lookahead * lookahead)
        steering = math.atan(self.wheelbase_m * curvature)
        return max(-self.max_steering_angle_rad, min(self.max_steering_angle_rad, steering))

    def _is_traversable(self, row: int, col: int) -> bool:
        key = (row, col)
        cached = self._traversable_cache.get(key)
        if cached is not None:
            return cached
        if not (0 <= row < self.grid.size_cells and 0 <= col < self.grid.size_cells):
            self._traversable_cache[key] = False
            return False
        radius_m = max(self.robot_width_m * 0.5, self.robot_length_m * 0.5) + self.safety_buffer_m
        radius = max(0, int(math.ceil(radius_m / self.grid.resolution_m)))
        r0 = row - radius
        r1 = row + radius
        c0 = col - radius
        c1 = col + radius
        if r0 < 0 or c0 < 0 or r1 >= self.grid.size_cells or c1 >= self.grid.size_cells:
            self._traversable_cache[key] = False
            return False
        cells = self.grid.cells[r0 : r1 + 1, c0 : c1 + 1]
        live = self.grid.live_mask[r0 : r1 + 1, c0 : c1 + 1]
        clear = not bool((cells != int(Cell.FREE)).any() or live.any())
        self._traversable_cache[key] = clear
        return clear

    def _cell_clear(self, row: int, col: int) -> bool:
        if not (0 <= row < self.grid.size_cells and 0 <= col < self.grid.size_cells):
            return False
        if bool(self.grid.live_mask[row, col]):
            return False
        return Cell(int(self.grid.cells[row, col])) == Cell.FREE
