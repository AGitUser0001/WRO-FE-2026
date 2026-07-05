from __future__ import annotations

import math
from typing import TYPE_CHECKING

from .grid import Cell, Direction, GridMap, LocalGrid
from .localize_scan import best_local_shift, local_obstacle_signature, scan_motion_score, scan_pose_valid, scan_world_points
from .localize_types import PoseEstimate, angle_diff
from .sensors import SensorFrame


class WallCorrectionMixin:
    grid: GridMap
    pose: PoseEstimate
    front_wall_motion_alpha: float
    front_wall_motion_max_step_m: float
    front_wall_motion_min_points: int
    wall_pose_correction_alpha: float
    wall_pose_correction_max_step_m: float
    wall_pose_max_slope: float
    wall_yaw_correction_alpha: float
    wall_yaw_correction_max_step_rad: float
    last_front_wall_distance_m: float | None
    last_front_wall_yaw_rad: float | None

    if TYPE_CHECKING:
        def _clamp_pose_to_known_bounds(self) -> None: ...

    def _begin_wall_pose_correction(self) -> None:
        self.pose.wall_pose_correction_x_m = 0.0
        self.pose.wall_pose_correction_y_m = 0.0
        self.pose.wall_pose_sources = 0

    def _update_forward_from_front_wall(self, frame: SensorFrame) -> None:
        wall = frame.front_wall
        self.pose.front_wall_motion_m = 0.0
        self.pose.front_wall_distance_m = wall.distance_m
        self.pose.front_wall_points = wall.point_count
        if not wall.valid or wall.point_count < self.front_wall_motion_min_points:
            self.last_front_wall_distance_m = None
            self.last_front_wall_yaw_rad = None
            return
        expected = self.grid.raycast_wall_distance(self.pose.x_m, self.pose.y_m, self.pose.yaw_rad, map_walls_only=True)
        if expected is None or abs(expected - wall.distance_m) > 0.30:
            self.last_front_wall_distance_m = None
            self.last_front_wall_yaw_rad = None
            return
        if self.last_front_wall_distance_m is None:
            self.last_front_wall_distance_m = wall.distance_m
            self.last_front_wall_yaw_rad = self.pose.yaw_rad
            return
        if self.last_front_wall_yaw_rad is None or abs(angle_diff(self.pose.yaw_rad, self.last_front_wall_yaw_rad)) > math.radians(5.0):
            self.last_front_wall_distance_m = wall.distance_m
            self.last_front_wall_yaw_rad = self.pose.yaw_rad
            return
        raw_motion = self.last_front_wall_distance_m - wall.distance_m
        if abs(raw_motion) > self.front_wall_motion_max_step_m:
            self.last_front_wall_distance_m = wall.distance_m
            self.last_front_wall_yaw_rad = self.pose.yaw_rad
            return
        motion = raw_motion * self.front_wall_motion_alpha
        self.pose.x_m += math.cos(self.pose.yaw_rad) * motion
        self.pose.y_m += math.sin(self.pose.yaw_rad) * motion
        self._clamp_pose_to_known_bounds()
        self.pose.front_wall_motion_m = motion
        self.last_front_wall_distance_m = wall.distance_m
        self.last_front_wall_yaw_rad = self.pose.yaw_rad

    def _correct_yaw_from_flat_walls(self, frame: SensorFrame) -> None:
        self.pose.wall_yaw_correction_rad = 0.0
        self.pose.wall_yaw_sources = 0
        if self.wall_yaw_correction_alpha <= 0.0:
            return
        yaw_errors: list[float] = []
        min_points = self.front_wall_motion_min_points
        if (
            frame.front_wall.valid
            and frame.front_wall.point_count >= min_points
            and abs(frame.front_wall.slope) <= self.wall_pose_max_slope
        ):
            yaw_errors.append(math.atan(frame.front_wall.slope))
        for wall in (frame.left_wall, frame.right_wall):
            if wall.valid and wall.point_count >= min_points and abs(wall.slope) <= self.wall_pose_max_slope:
                yaw_errors.append(-math.atan(wall.slope))
        self.pose.wall_yaw_sources = len(yaw_errors)
        if not yaw_errors:
            return
        yaw_error = sum(yaw_errors) / len(yaw_errors)
        correction = -yaw_error * self.wall_yaw_correction_alpha
        if abs(correction) > self.wall_yaw_correction_max_step_rad:
            correction = math.copysign(self.wall_yaw_correction_max_step_rad, correction)
        self.pose.yaw_bias_rad += correction
        self.pose.yaw_rad += correction
        self.pose.wall_yaw_correction_rad = correction

    def _correct_lateral_from_side_ranges(self, frame: SensorFrame) -> None:
        half_w = self.grid.corridor_width_m * 0.5
        estimates: list[float] = []
        if self._side_wall_usable(frame.left_wall):
            estimates.append(half_w - frame.left_wall.distance_m)
        if self._side_wall_usable(frame.right_wall):
            estimates.append(frame.right_wall.distance_m - half_w)
        if not estimates:
            return
        left_corr = sum(estimates) / len(estimates) * min(self.wall_pose_correction_alpha, 0.20)
        max_step = min(self.wall_pose_correction_max_step_m, 0.03)
        left_corr = max(-max_step, min(max_step, left_corr))
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        dx = -left_corr * sy
        dy = left_corr * cy
        self.pose.x_m += dx
        self.pose.y_m += dy
        self._clamp_pose_to_known_bounds()
        self.pose.wall_pose_correction_x_m += dx
        self.pose.wall_pose_correction_y_m += dy
        self.pose.wall_pose_sources += len(estimates)

    def _side_wall_usable(self, wall: object) -> bool:
        return (
            bool(getattr(wall, "valid", False))
            and int(getattr(wall, "point_count", 0)) >= self.front_wall_motion_min_points
            and abs(float(getattr(wall, "slope", math.inf))) <= self.wall_pose_max_slope
        )

    def _correct_pose_from_flat_walls(self, frame: SensorFrame) -> None:
        forward_corrs: list[float] = []
        left_corrs: list[float] = []
        if (
            frame.front_wall.valid
            and frame.front_wall.point_count >= self.front_wall_motion_min_points
            and abs(frame.front_wall.slope) <= self.wall_pose_max_slope
        ):
            expected = self.grid.raycast_wall_distance(self.pose.x_m, self.pose.y_m, self.pose.yaw_rad, map_walls_only=True)
            if expected is not None:
                forward_corrs.append(expected - frame.front_wall.distance_m)
        if (
            frame.left_wall.valid
            and frame.left_wall.point_count >= self.front_wall_motion_min_points
            and abs(frame.left_wall.slope) <= self.wall_pose_max_slope
        ):
            expected = self._expected_side_wall_distance(self.pose.yaw_rad + math.pi / 2.0)
            if expected is not None:
                left_corrs.append(expected - frame.left_wall.distance_m)
        if (
            frame.right_wall.valid
            and frame.right_wall.point_count >= self.front_wall_motion_min_points
            and abs(frame.right_wall.slope) <= self.wall_pose_max_slope
        ):
            expected = self._expected_side_wall_distance(self.pose.yaw_rad - math.pi / 2.0)
            if expected is not None:
                left_corrs.append(frame.right_wall.distance_m - expected)
        self._apply_wall_correction(forward_corrs, left_corrs)

    def _expected_side_wall_distance(self, ray_yaw: float) -> float | None:
        if self.grid.direction != Direction.UNKNOWN:
            return self.grid.raycast_wall_distance(
                self.pose.x_m,
                self.pose.y_m,
                ray_yaw,
                map_walls_only=True,
            )
        ux = math.cos(ray_yaw)
        if abs(ux) < 1e-4:
            return None
        half_w = self.grid.corridor_width_m * 0.5
        wall_x = half_w if ux > 0.0 else -half_w
        distance = (wall_x - self.pose.x_m) / ux
        if distance <= 0.0:
            return None
        y_hit = self.pose.y_m + math.sin(ray_yaw) * distance
        if abs(y_hit) > self.grid.corridor_length_m * 0.5 + 0.1:
            return None
        return distance

    def _apply_wall_correction(self, forward_corrs: list[float], left_corrs: list[float]) -> None:
        source_count = len(forward_corrs) + len(left_corrs)
        if source_count == 0:
            return
        forward_corr = sum(forward_corrs) / len(forward_corrs) if forward_corrs else 0.0
        left_corr = sum(left_corrs) / len(left_corrs) if left_corrs else 0.0
        alpha = min(self.wall_pose_correction_alpha, 0.30)
        forward_corr *= alpha
        left_corr *= alpha
        length = math.hypot(forward_corr, left_corr)
        max_step = min(self.wall_pose_correction_max_step_m, 0.04)
        if length > max_step and length > 1e-6:
            scale = max_step / length
            forward_corr *= scale
            left_corr *= scale
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        dx = forward_corr * cy - left_corr * sy
        dy = forward_corr * sy + left_corr * cy
        self.pose.x_m += dx
        self.pose.y_m += dy
        self._clamp_pose_to_known_bounds()
        self.pose.wall_pose_correction_x_m += dx
        self.pose.wall_pose_correction_y_m += dy
        self.pose.wall_pose_sources += source_count

    def _correct_pose_from_wall_cells(self, local: LocalGrid) -> None:
        wall_points = self._wall_points(local)
        if len(wall_points) < self.front_wall_motion_min_points:
            return
        match = self._best_wall_pattern_offset(wall_points)
        if match is None:
            return
        dx, dy, score = match
        alpha = min(1.0, self.wall_pose_correction_alpha * max(0.25, min(score, 1.0)))
        dx *= alpha
        dy *= alpha
        length = math.hypot(dx, dy)
        if length > self.wall_pose_correction_max_step_m and length > 1e-6:
            scale = self.wall_pose_correction_max_step_m / length
            dx *= scale
            dy *= scale
        self.pose.x_m += dx
        self.pose.y_m += dy
        self._clamp_pose_to_known_bounds()
        self.pose.wall_pose_correction_x_m = dx
        self.pose.wall_pose_correction_y_m = dy
        self.pose.wall_pose_sources = len(wall_points)

    def _correct_pose_from_scan_motion(self, frame: SensorFrame) -> None:
        points = scan_world_points(frame, self.pose.x_m, self.pose.y_m, self.pose.yaw_rad)
        prev = getattr(self, "_prev_scan_world", ())
        pred_x, pred_y = float(getattr(self, "_predicted_dx_m", 0.0)), float(getattr(self, "_predicted_dy_m", 0.0))
        pred_len = math.hypot(pred_x, pred_y)
        local_shift = self._local_obstacle_shift(frame, pred_x, pred_y)
        if local_shift is not None:
            cy = math.cos(self.pose.yaw_rad); sy = math.sin(self.pose.yaw_rad)
            corr_x = local_shift[0] * cy - local_shift[1] * sy - pred_x
            corr_y = local_shift[0] * sy + local_shift[1] * cy - pred_y
            next_x = self.pose.x_m + corr_x * 0.8; next_y = self.pose.y_m + corr_y * 0.8
            if self._scan_pose_valid(frame, next_x, next_y):
                self.pose.x_m = next_x; self.pose.y_m = next_y; self._clamp_pose_to_known_bounds()
            points = scan_world_points(frame, self.pose.x_m, self.pose.y_m, self.pose.yaw_rad)
        if len(points) < 12 or len(prev) < 12:
            self._prev_scan_world = points
            return
        resolution = max(self.grid.resolution_m, 0.03); previous = {(round(x / resolution), round(y / resolution)) for x, y in prev}
        radius = min(0.25, max(0.10, pred_len + 0.08))
        step = max(resolution, 0.04)
        count = max(1, int(round(radius / step)))
        zero_score = scan_motion_score(points, previous, 0.0, 0.0, resolution)
        best = (0.0, 0.0, zero_score)
        for ix in range(-count, count + 1):
            dx = ix * step
            for iy in range(-count, count + 1):
                if not self._scan_pose_valid(frame, self.pose.x_m + dx, self.pose.y_m + iy * step):
                    continue
                dy = iy * step
                score = scan_motion_score(points, previous, dx, dy, resolution)
                score -= math.hypot(dx, dy) * 0.18
                if score > best[2]: best = (dx, dy, score)
        min_gain = 0.025 if pred_len > 0.01 else 0.08
        if best[2] - zero_score > min_gain and math.hypot(best[0], best[1]) > 0.01:
            dx, dy = best[0] * 0.95, best[1] * 0.95
            length = math.hypot(dx, dy)
            max_step = max(0.08, min(0.14, pred_len + 0.04))
            if length > max_step:
                dx *= max_step / length; dy *= max_step / length
            if self._scan_pose_valid(frame, self.pose.x_m + dx, self.pose.y_m + dy):
                self.pose.x_m += dx; self.pose.y_m += dy
                self._clamp_pose_to_known_bounds()
                self.pose.wall_pose_correction_x_m += dx
                self.pose.wall_pose_correction_y_m += dy
                self.pose.wall_pose_sources += len(points)
            points = scan_world_points(frame, self.pose.x_m, self.pose.y_m, self.pose.yaw_rad)
        self._prev_scan_world = points

    def _scan_pose_valid(self, frame: SensorFrame, x_m: float, y_m: float) -> bool:
        return scan_pose_valid(self.grid, frame, x_m, y_m, self.pose.yaw_rad, float(getattr(self, "robot_radius_m", 0.15)))

    def _local_obstacle_shift(self, frame: SensorFrame, pred_x: float, pred_y: float) -> tuple[float, float] | None:
        current = local_obstacle_signature(frame.local_grid)
        previous = getattr(self, "_prev_local_obstacles", ())
        self._prev_local_obstacles = current
        if len(current) < 4 or len(previous) < 4:
            return None
        cy = math.cos(self.pose.yaw_rad); sy = math.sin(self.pose.yaw_rad)
        pred_f = pred_x * cy + pred_y * sy
        pred_l = -pred_x * sy + pred_y * cy
        return best_local_shift(current, set(previous), pred_f, pred_l)

    def _wall_points(self, local: LocalGrid) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        stride = max(1, int(round(0.10 / max(local.resolution_m, 1e-6))))
        for row, col, value in local.observed_cells():
            if value not in (Cell.MAP_WALL, Cell.WALL) or (row + col) % stride != 0:
                continue
            lx, ly = local.cell_to_local(row, col)
            if math.hypot(lx, ly) >= 0.18:
                points.append((self.pose.x_m + lx * cy - ly * sy, self.pose.y_m + lx * sy + ly * cy))
        return points

    def _best_wall_pattern_offset(self, wall_points: list[tuple[float, float]]) -> tuple[float, float, float] | None:
        resolution = self.grid.resolution_m
        max_cells = max(1, int(round(0.25 / resolution)))
        transformed = [idx for point in wall_points if (idx := self.grid.world_to_cell(point[0], point[1])) is not None]
        if len(transformed) > 80:
            transformed = transformed[::max(1, len(transformed) // 80)]
        if len(transformed) < self.front_wall_motion_min_points:
            return None
        best_dr = 0
        best_dc = 0
        best_score = -math.inf
        for dr in range(-max_cells, max_cells + 1):
            for dc in range(-max_cells, max_cells + 1):
                score, matched = self._offset_score(transformed, dr, dc)
                if matched < max(3, len(transformed) // 4):
                    score -= 0.5
                score -= math.hypot(dr, dc) * 0.015
                if score > best_score:
                    best_score = score
                    best_dr = dr
                    best_dc = dc
        if best_score < 0.05:
            return None
        return best_dc * resolution, -best_dr * resolution, best_score

    def _offset_score(self, cells: list[tuple[int, int]], dr: int, dc: int) -> tuple[float, int]:
        score = 0.0
        matched = 0
        for row, col in cells:
            cell_score = self._nearby_map_wall_score(row + dr, col + dc)
            score += cell_score
            if cell_score > 0.0:
                matched += 1
        return score / len(cells), matched

    def _nearby_map_wall_score(self, row: int, col: int) -> float:
        best = -0.35
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                rr = row + dr
                cc = col + dc
                if not (0 <= rr < self.grid.size_cells and 0 <= cc < self.grid.size_cells):
                    continue
                value = Cell(int(self.grid.cells[rr, cc]))
                if value == Cell.MAP_WALL:
                    best = max(best, 1.0 - 0.2 * math.hypot(dr, dc))
        return best
