from __future__ import annotations

import math

import numpy as np

from .grid import Cell, Direction, GridMap
from .localize_direction import DirectionLockMixin
from .localize_types import PoseEstimate, angle_diff
from .localize_walls import WallCorrectionMixin
from .sensors import ImuReading, SensorFrame


class Localizer(WallCorrectionMixin, DirectionLockMixin):
    """Pose owner for the new grid planner.

    IMU absolute yaw is the base orientation. Local->global grid matching
    supplies a small position correction each update.
    """

    def __init__(
        self,
        global_grid: GridMap,
        start_yaw_rad: float = math.pi / 2.0,
        correction_alpha: float = 0.04,
        fuse_add_alpha: float = 0.08,
        fuse_erode_alpha: float = 0.02,
        front_wall_motion_alpha: float = 0.85,
        front_wall_motion_max_step_m: float = 0.12,
        front_wall_motion_min_points: int = 5,
        wall_pose_correction_alpha: float = 0.80,
        wall_pose_correction_max_step_m: float = 0.50,
        wall_pose_max_slope: float = 0.35,
        wall_yaw_correction_alpha: float = 0.12,
        wall_yaw_correction_max_step_rad: float = math.radians(3.0),
        direction_lock_confirm_frames: int = 4,
        direction_lock_min_score_delta: float = 0.10,
        robot_length_m: float = 0.25,
        robot_width_m: float = 0.15,
    ):
        self.grid = global_grid
        self.start_yaw_rad = start_yaw_rad
        self.correction_alpha = correction_alpha
        self.fuse_add_alpha = fuse_add_alpha
        self.fuse_erode_alpha = fuse_erode_alpha
        self.front_wall_motion_alpha = max(0.0, min(front_wall_motion_alpha, 1.0))
        self.front_wall_motion_max_step_m = max(0.0, front_wall_motion_max_step_m)
        self.front_wall_motion_min_points = max(1, front_wall_motion_min_points)
        self.wall_pose_correction_alpha = max(0.0, min(wall_pose_correction_alpha, 1.0))
        self.wall_pose_correction_max_step_m = max(0.0, wall_pose_correction_max_step_m)
        self.wall_pose_max_slope = max(0.0, wall_pose_max_slope)
        self.wall_yaw_correction_alpha = max(0.0, min(wall_yaw_correction_alpha, 1.0))
        self.wall_yaw_correction_max_step_rad = max(0.0, wall_yaw_correction_max_step_rad)
        self.direction_lock_confirm_frames = max(1, direction_lock_confirm_frames)
        self.direction_lock_min_score_delta = max(0.0, direction_lock_min_score_delta)
        self.robot_radius_m = math.hypot(max(robot_length_m, 0.01), max(robot_width_m, 0.01)) * 0.5
        self.pose = PoseEstimate(yaw_rad=start_yaw_rad)
        self.yaw_calibrated = False
        self.pending_direction = Direction.UNKNOWN
        self.pending_direction_frames = 0
        self.pending_unknown_frames = 0
        self.last_direction_scores = (0.0, 0.0)
        self.direction_evidence = {
            Direction.LEFT: 0.0,
            Direction.RIGHT: 0.0,
        }
        self.last_front_wall_distance_m: float | None = None
        self.last_front_wall_yaw_rad: float | None = None
        self.direction_hypotheses = {
            Direction.LEFT: self._make_direction_hypothesis(Direction.LEFT),
            Direction.RIGHT: self._make_direction_hypothesis(Direction.RIGHT),
        }

    def reset(self) -> None:
        self.grid.reset()
        self.pose = PoseEstimate(yaw_rad=self.start_yaw_rad)
        self.yaw_calibrated = False
        self.pending_direction = Direction.UNKNOWN
        self.pending_direction_frames = 0
        self.pending_unknown_frames = 0
        self.last_direction_scores = (0.0, 0.0)
        self.direction_evidence = {
            Direction.LEFT: 0.0,
            Direction.RIGHT: 0.0,
        }
        self.last_front_wall_distance_m = None
        self.last_front_wall_yaw_rad = None
        self.direction_hypotheses = {
            Direction.LEFT: self._make_direction_hypothesis(Direction.LEFT),
            Direction.RIGHT: self._make_direction_hypothesis(Direction.RIGHT),
        }

    def update_imu(self, imu: ImuReading) -> None:
        if imu.yaw_rad is None:
            return
        self.pose.raw_yaw_rad = imu.yaw_rad
        if not self.yaw_calibrated:
            self.pose.yaw_bias_rad = angle_diff(self.start_yaw_rad, imu.yaw_rad)
            self.yaw_calibrated = True
        self.pose.yaw_rad = math.atan2(
            math.sin(imu.yaw_rad + self.pose.yaw_bias_rad),
            math.cos(imu.yaw_rad + self.pose.yaw_bias_rad),
        )

    def predict_motion(self, speed_mps: float, steering_rad: float, wheelbase_m: float, dt_s: float) -> None:
        if dt_s <= 0.0 or abs(speed_mps) < 1e-4:
            return
        start_x, start_y = self.pose.x_m, self.pose.y_m
        dt_s = min(dt_s, 0.2)
        self.pose.yaw_rad += speed_mps / max(wheelbase_m, 0.01) * math.tan(steering_rad) * dt_s
        self.pose.yaw_rad = math.atan2(math.sin(self.pose.yaw_rad), math.cos(self.pose.yaw_rad))
        distance = speed_mps * dt_s
        self.pose.x_m += math.cos(self.pose.yaw_rad) * distance
        self.pose.y_m += math.sin(self.pose.yaw_rad) * distance
        self._clamp_pose_to_known_bounds()
        self._predicted_dx_m = self.pose.x_m - start_x
        self._predicted_dy_m = self.pose.y_m - start_y

    def update_from_sensors(self, frame: SensorFrame) -> PoseEstimate:
        if frame.imu is not None:
            self.update_imu(frame.imu)

        self.pose.front_wall_motion_m = 0.0
        self.pose.front_wall_distance_m = frame.front_wall.distance_m
        self.pose.front_wall_points = frame.front_wall.point_count
        self._begin_wall_pose_correction()
        if self.grid.direction == Direction.UNKNOWN:
            self._correct_lateral_from_side_ranges(frame)

        self._update_forward_from_front_wall(frame)
        self._correct_pose_from_flat_walls(frame)
        self._update_direction_lock(frame)
        if self.grid.direction == Direction.UNKNOWN:
            self.pose.alignment_score = 0.0
            self.pose.confidence = 0.0
            return self.pose
        self._correct_pose_from_wall_cells(frame.local_grid)
        self._correct_pose_from_scan_motion(frame)

        prior_x, prior_y = self.pose.x_m, self.pose.y_m
        dx, dy, score = self._wide_alignment_vector(frame)
        alpha = self._alignment_alpha(score)
        step_x = dx * alpha
        step_y = dy * alpha
        step_len = math.hypot(step_x, step_y)
        max_step = 0.18 if score >= 0.35 else 0.10
        if step_len > max_step and step_len > 1e-6:
            scale = max_step / step_len
            step_x *= scale
            step_y *= scale
        self.pose.x_m += step_x
        self.pose.y_m += step_y
        self._clamp_pose_to_known_bounds()
        if self._has_impossible_wall_points(frame):
            self.pose.x_m, self.pose.y_m = prior_x, prior_y
            self._clamp_pose_to_known_bounds()
            self.pose.alignment_score = 0.0
            self.pose.confidence = 0.0
            return self.pose
        self.pose.alignment_score = score
        self.pose.confidence = max(0.0, min((score + 0.5) / 2.5, 1.0))
        self.grid.fuse_local(frame.local_grid, self.pose.x_m, self.pose.y_m, self.pose.yaw_rad, self.fuse_add_alpha, self.fuse_erode_alpha)
        return self.pose

    def _wide_alignment_vector(self, frame: SensorFrame) -> tuple[float, float, float]:
        points = self._alignment_points(frame)
        if len(points) < self.front_wall_motion_min_points:
            return 0.0, 0.0, 0.0
        bounds = self._known_pose_bounds()
        search_radius = 1.80 if self.pose.confidence < 0.20 else 0.45
        coarse = self._best_alignment_offset(points, bounds, search_radius, 0.15, None)
        fine = self._best_alignment_offset(points, bounds, 0.22, 0.05, coarse)
        if fine is None:
            return 0.0, 0.0, 0.0
        dx, dy, score = fine
        return dx, dy, score

    def _alignment_points(self, frame: SensorFrame) -> tuple[tuple[float, float, Cell], ...]:
        walls = []
        for row, col, value in frame.local_grid.non_free_cells():
            if value != Cell.WALL:
                continue
            lx, ly = frame.local_grid.cell_to_local(row, col)
            walls.append((lx, ly, value))
        return tuple(walls)

    def _has_impossible_wall_points(self, frame: SensorFrame) -> bool:
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        for row, col, value in frame.local_grid.non_free_cells():
            if value != Cell.WALL:
                continue
            lx, ly = frame.local_grid.cell_to_local(row, col)
            gx = self.pose.x_m + lx * cy - ly * sy
            gy = self.pose.y_m + lx * sy + ly * cy
            if not bool(self._inside_known_map(np.asarray(gx), np.asarray(gy), self.grid.resolution_m * 1.5)):
                return True
            if bool(self._inside_center_hollow(np.asarray(gx), np.asarray(gy))):
                return True
        return False

    def _alignment_alpha(self, score: float) -> float:
        base = max(0.0, min(self.correction_alpha, 0.12))
        if score >= 0.75:
            return max(base, 0.75)
        if score >= 0.35:
            return max(base, 0.45)
        if score >= 0.10:
            return max(base, 0.18)
        return min(base, 0.04)

    def _best_alignment_offset(
        self,
        points: tuple[tuple[float, float, Cell], ...],
        bounds: tuple[float, float, float, float],
        radius_m: float,
        step_m: float,
        center: tuple[float, float, float] | None,
    ) -> tuple[float, float, float] | None:
        base_dx, base_dy = (0.0, 0.0) if center is None else (center[0], center[1])
        steps = max(1, int(round(radius_m / step_m)))
        offsets = np.arange(-steps, steps + 1, dtype=np.float32) * step_m
        dx_grid, dy_grid = np.meshgrid(base_dx + offsets, base_dy + offsets)
        dxs = dx_grid.ravel()
        dys = dy_grid.ravel()
        xs = self.pose.x_m + dxs
        ys = self.pose.y_m + dys
        local_xy = np.asarray([(p[0], p[1]) for p in points], dtype=np.float32)
        local_values = np.asarray([int(p[2]) for p in points], dtype=np.uint8)
        local_walls = local_values[None, :] == int(Cell.WALL)
        local_obstacles = (local_values[None, :] == int(Cell.UNKNOWN_OBSTRUCTION)) | (
            local_values[None, :] == int(Cell.LIVE_OBSTACLE)
        )
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        gx_offsets = local_xy[:, 0] * cy - local_xy[:, 1] * sy
        gy_offsets = local_xy[:, 0] * sy + local_xy[:, 1] * cy
        candidates = (bounds[0] <= xs) & (xs <= bounds[1]) & (bounds[2] <= ys) & (ys <= bounds[3])
        candidates &= self._pose_candidates_valid(xs, ys, gx_offsets, gy_offsets, local_walls)
        if not bool(np.any(candidates)):
            return None
        dxs = dxs[candidates]
        dys = dys[candidates]
        xs = xs[candidates]
        ys = ys[candidates]
        scores = self._alignment_scores(xs, ys, gx_offsets, gy_offsets, local_walls, local_obstacles) / len(points)
        totals = scores - np.hypot(dxs, dys) * 0.45
        if not bool(np.isfinite(totals).any()):
            return None
        best_idx = int(np.argmax(totals))
        return float(dxs[best_idx]), float(dys[best_idx]), float(totals[best_idx])

    def _alignment_scores(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        gx_offsets: np.ndarray,
        gy_offsets: np.ndarray,
        local_walls: np.ndarray,
        local_obstacles: np.ndarray,
    ) -> np.ndarray:
        resolution = self.grid.resolution_m
        origin = self.grid.origin_cell
        cols = np.rint((xs[:, None] + gx_offsets[None, :]) / resolution).astype(np.int32) + origin
        rows = origin - np.rint((ys[:, None] + gy_offsets[None, :]) / resolution).astype(np.int32)
        valid = (0 <= rows) & (rows < self.grid.size_cells) & (0 <= cols) & (cols < self.grid.size_cells)
        global_values = np.full(rows.shape, 255, dtype=np.uint8)
        global_values[valid] = self.grid.cells[rows[valid], cols[valid]]
        map_walls = global_values == int(Cell.MAP_WALL)
        map_obstacles = (global_values == int(Cell.UNKNOWN_OBSTRUCTION)) | (
            global_values == int(Cell.LIVE_OBSTACLE)
        )
        scores = np.where(map_walls, np.where(local_walls, 2.0, -0.8), 0.0)
        scores += np.where(map_obstacles, np.where(local_obstacles, 1.0, -0.4), 0.0)
        scores += np.where((global_values == int(Cell.FREE)) & local_walls, -1.2, 0.0)
        scores += np.where(valid, 0.0, -2.0)
        wall_hits = (map_walls & local_walls).sum(axis=1)
        wall_min = max(1.0, float(local_walls.sum()) * 0.10)
        return np.where(wall_hits >= wall_min, scores.sum(axis=1), -np.inf)

    def _inside_known_map(self, xs: np.ndarray, ys: np.ndarray, tolerance_m: float = 0.0) -> np.ndarray:
        if self.grid.direction == Direction.LEFT:
            inside_x = (-2.5 - tolerance_m <= xs) & (xs <= self.grid.corridor_width_m * 0.5 + tolerance_m)
        elif self.grid.direction == Direction.RIGHT:
            inside_x = (-self.grid.corridor_width_m * 0.5 - tolerance_m <= xs) & (xs <= 2.5 + tolerance_m)
        else:
            inside_x = (-2.5 - tolerance_m <= xs) & (xs <= 2.5 + tolerance_m)
        return inside_x & (-1.5 - tolerance_m <= ys) & (ys <= 1.5 + tolerance_m)

    def _inside_center_hollow(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        if self.grid.direction == Direction.UNKNOWN:
            return np.zeros(xs.shape, dtype=np.bool_)
        half_w = self.grid.corridor_width_m * 0.5
        x_min, x_max = (-2.5, half_w) if self.grid.direction == Direction.LEFT else (-half_w, 2.5)
        center_x = (x_min + x_max) * 0.5
        margin = self.grid.resolution_m * 0.75
        return (np.abs(xs - center_x) < 0.5 - margin) & (np.abs(ys) < 0.5 - margin)

    def _pose_candidates_valid(self, xs: np.ndarray, ys: np.ndarray, gx_offsets: np.ndarray, gy_offsets: np.ndarray, walls: np.ndarray) -> np.ndarray:
        r = self.robot_radius_m + self.grid.resolution_m
        valid = self._inside_known_map(xs - r, ys - r) & self._inside_known_map(xs + r, ys + r)
        wall_xs = xs[:, None] + gx_offsets[None, :]
        wall_ys = ys[:, None] + gy_offsets[None, :]
        tol = self.grid.resolution_m * 1.5
        wall_ok = self._inside_known_map(wall_xs, wall_ys, tol) & ~self._inside_center_hollow(wall_xs, wall_ys)
        return valid & ~self._inside_center_hollow_inflated(xs, ys, r) & np.all(np.where(walls, wall_ok, True), axis=1)

    def _inside_center_hollow_inflated(self, xs: np.ndarray, ys: np.ndarray, inflate_m: float) -> np.ndarray:
        if self.grid.direction == Direction.UNKNOWN:
            return np.zeros(xs.shape, dtype=np.bool_)
        half_w = self.grid.corridor_width_m * 0.5
        x_min, x_max = (-2.5, half_w) if self.grid.direction == Direction.LEFT else (-half_w, 2.5)
        center_x = (x_min + x_max) * 0.5
        return (np.abs(xs - center_x) < 0.5 + inflate_m) & (np.abs(ys) < 0.5 + inflate_m)

    def _clamp_pose_to_known_bounds(self) -> None:
        x_min, x_max, y_min, y_max = self._known_pose_bounds()
        r = self.robot_radius_m + self.grid.resolution_m
        x_min += r
        x_max -= r
        y_min += r
        y_max -= r
        self.pose.x_m = max(x_min, min(x_max, self.pose.x_m))
        self.pose.y_m = max(y_min, min(y_max, self.pose.y_m))
        if self.grid.direction == Direction.UNKNOWN:
            return
        half_w = self.grid.corridor_width_m * 0.5
        bx0, bx1 = (-2.5, half_w) if self.grid.direction == Direction.LEFT else (-half_w, 2.5)
        center_x = (bx0 + bx1) * 0.5
        dx = self.pose.x_m - center_x
        dy = self.pose.y_m
        limit = 0.5 + r
        if abs(dx) >= limit or abs(dy) >= limit:
            return
        if limit - abs(dx) < limit - abs(dy):
            self.pose.x_m = center_x + math.copysign(limit, dx if abs(dx) > 1e-6 else 1.0)
        else:
            self.pose.y_m = math.copysign(limit, dy if abs(dy) > 1e-6 else 1.0)

    def _known_pose_bounds(self) -> tuple[float, float, float, float]:
        half_h = 1.5
        if self.grid.direction == Direction.LEFT:
            return -2.5, 0.5, -half_h, half_h
        if self.grid.direction == Direction.RIGHT:
            return -0.5, 2.5, -half_h, half_h
        half_w = self.grid.corridor_width_m * 0.5
        return -half_w, half_w, -half_h, half_h
