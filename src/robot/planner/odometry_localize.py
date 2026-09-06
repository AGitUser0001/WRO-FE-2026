from __future__ import annotations

import math
import time
from collections.abc import Iterable

import numpy as np
from scipy.ndimage import grey_dilation

from .grid import Cell, Direction, GridMap, LocalGrid
from .localize_scan import best_map_shift, scan_world_walls
from .localize_types import PoseEstimate, angle_diff
from .sensors import SensorFrame


class OdometryLocalizer:

    def __init__(
        self,
        grid: GridMap,
        start_yaw_rad: float = math.pi / 2.0,
        correction_max_m: float = 0.30,
        fuse_add_alpha: float = 0.08,
        fuse_erode_alpha: float = 0.02,
        direction_lock_confirm_frames: int = 4,
        direction_lock_min_score_delta: float = 0.10,
        direction_min_wall_points: int = 5,
    ) -> None:
        self.grid = grid
        self.start_yaw_rad = start_yaw_rad
        self.correction_max_m = max(0.0, correction_max_m)
        self.fuse_add_alpha = fuse_add_alpha
        self.fuse_erode_alpha = fuse_erode_alpha
        self.direction_lock_confirm_frames = max(1, direction_lock_confirm_frames)
        self.direction_lock_min_score_delta = max(0.0, direction_lock_min_score_delta)
        self.front_wall_motion_min_points = max(1, direction_min_wall_points)
        self.pose = PoseEstimate(yaw_rad=start_yaw_rad)
        self._last_odometry_speed_mps: float | None = None
        self._base_pose = (0.0, 0.0, start_yaw_rad)
        self._correction = (0.0, 0.0)
        self._total_correction = (0.0, 0.0)
        self._imu_yaw_offset: float | None = None
        self._odometry_travel_m = 0.0
        self._reset_direction_state()

    def _reset_direction_state(self) -> None:
        self.pending_direction = Direction.UNKNOWN
        self.pending_direction_frames = 0
        self.pending_unknown_frames = 0
        self.direction_decision_started = False
        self.last_direction_scores = (0.0, 0.0)
        self.last_direction_direct = Direction.UNKNOWN
        self.last_direction_raw_scores = (-math.inf, -math.inf)
        self.direction_evidence = {Direction.LEFT: 0.0, Direction.RIGHT: 0.0}
        self.direction_hypotheses = {
            Direction.LEFT: self._make_direction_hypothesis(Direction.LEFT),
            Direction.RIGHT: self._make_direction_hypothesis(Direction.RIGHT),
        }
        axis = np.arange(-3, 4, dtype=np.float32)
        dc, dr = np.meshgrid(axis, axis)
        structure = -0.16 * np.hypot(dr, dc)
        self.direction_match_fields = {}
        for direction, hypothesis in self.direction_hypotheses.items():
            walls = (
                (hypothesis.cells == int(Cell.MAP_WALL))
                | (hypothesis.cells == int(Cell.WALL))
            )
            seeds = np.where(walls, 1.0, -np.inf).astype(np.float32)
            field = grey_dilation(
                seeds,
                footprint=np.ones((7, 7), dtype=np.bool_),
                structure=structure,
                mode="constant",
                cval=-np.inf,
            )
            self.direction_match_fields[direction] = np.maximum(field, -0.2)

    def reset(self) -> None:
        self.grid.reset()
        self.pose = PoseEstimate(yaw_rad=self.start_yaw_rad)
        self._last_odometry_speed_mps = None
        self._base_pose = (0.0, 0.0, self.start_yaw_rad)
        self._correction = (0.0, 0.0)
        self._total_correction = (0.0, 0.0)
        self._imu_yaw_offset = None
        self._odometry_travel_m = 0.0
        self._reset_direction_state()

    def lock_direction(self, direction: Direction | int) -> None:
        candidate = Direction(direction)
        if candidate == Direction.UNKNOWN or self.grid.direction == candidate:
            return
        hypothesis = self.direction_hypotheses[candidate]
        self.grid.cells[:, :] = hypothesis.cells
        self.grid.wall_score[:, :] = hypothesis.wall_score
        self.grid.unknown_score[:, :] = hypothesis.unknown_score
        self.grid.direction = candidate
        self.pending_direction = candidate
        self.pending_direction_frames = 0
        half_w = self.grid.corridor_width_m * 0.5
        if abs(self.pose.x_m) <= half_w + 0.25:
            self.pose.x_m = max(-half_w, min(half_w, self.pose.x_m))

    def _update_direction_lock(self, frame: SensorFrame) -> None:
        if self.grid.direction != Direction.UNKNOWN:
            self._maybe_relock_direction(frame)
            return
        at_decision = self._in_direction_decision_section(
            self.pose.x_m,
            self.pose.y_m,
            margin=self.grid.resolution_m,
        )
        if at_decision:
            self.direction_decision_started = True
        candidate = self._corner_direction_candidate(frame)
        if not at_decision:
            self.pending_direction_frames = 0
            return
        if candidate == Direction.UNKNOWN:
            self._note_unknown_direction()
            return
        self.pending_unknown_frames = 0
        left_score, right_score = self.last_direction_scores
        if abs(left_score - right_score) < self.direction_lock_min_score_delta:
            self._note_unknown_direction()
            return
        if candidate != self.pending_direction:
            self.pending_direction = candidate
            self.pending_direction_frames = 1
        else:
            self.pending_direction_frames += 1
        decisive = (
            max(left_score, right_score) >= 0.80
            and abs(left_score - right_score) >= max(0.22, self.direction_lock_min_score_delta)
        )
        required_frames = (
            1
            if decisive
            else self.direction_lock_confirm_frames
        )
        if self.pending_direction_frames >= required_frames:
            self.lock_direction(candidate)

    def _maybe_relock_direction(self, frame: SensorFrame) -> None:
        if not self._in_direction_decision_section(
            self.pose.x_m,
            self.pose.y_m,
            margin=self.grid.resolution_m,
        ):
            self.pending_direction_frames = 0
            return
        current = self.grid.direction
        other = Direction.LEFT if current == Direction.RIGHT else Direction.RIGHT
        wall_x, wall_y = self._direction_wall_points(frame.local_grid)
        current_score = self._score_direction_hypothesis(wall_x, wall_y, current)
        other_score = self._score_direction_hypothesis(wall_x, wall_y, other)
        self.last_direction_scores = (
            other_score if other == Direction.LEFT else current_score,
            other_score if other == Direction.RIGHT else current_score,
        )
        if not math.isfinite(other_score) or other_score < current_score + 0.45 or other_score < 0.20:
            self.pending_direction_frames = 0
            return
        if self.pending_direction != other:
            self.pending_direction = other
            self.pending_direction_frames = 1
            return
        self.pending_direction_frames += 1
        if self.pending_direction_frames >= self.direction_lock_confirm_frames:
            self.lock_direction(other)

    def _note_unknown_direction(self) -> None:
        self.pending_unknown_frames += 1
        if self.pending_unknown_frames >= 3:
            self.pending_direction = Direction.UNKNOWN
            self.pending_direction_frames = 0

    def _corner_direction_candidate(self, frame: SensorFrame) -> Direction:
        wall_x, wall_y = self._direction_wall_points(frame.local_grid)
        direct = self._direct_extension_direction(wall_x, wall_y)
        self.last_direction_direct = direct
        left_raw = self._score_direction_hypothesis(wall_x, wall_y, Direction.LEFT)
        right_raw = self._score_direction_hypothesis(wall_x, wall_y, Direction.RIGHT)
        self.last_direction_raw_scores = (left_raw, right_raw)
        if not math.isfinite(left_raw) and not math.isfinite(right_raw):
            if direct != Direction.UNKNOWN:
                self._update_direction_evidence(direct, 0.4)
                other = Direction.RIGHT if direct == Direction.LEFT else Direction.LEFT
                self._update_direction_evidence(other, -0.1)
        else:
            self._update_direction_evidence(Direction.LEFT, left_raw)
            self._update_direction_evidence(Direction.RIGHT, right_raw)
        left_score = self.direction_evidence[Direction.LEFT]
        right_score = self.direction_evidence[Direction.RIGHT]
        self.last_direction_scores = (left_score, right_score)
        if abs(left_score - right_score) < self.direction_lock_min_score_delta:
            return Direction.UNKNOWN
        if max(left_score, right_score) < 0.25:
            return Direction.UNKNOWN
        return Direction.LEFT if left_score > right_score else Direction.RIGHT

    def _direction_wall_points(self, local: LocalGrid) -> tuple[np.ndarray, np.ndarray]:
        rows, cols = np.nonzero(
            local.observed & (local.cells == int(Cell.WALL)),
        )
        if not len(rows):
            empty = np.empty(0, dtype=np.float32)
            return empty, empty
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        local_x = (local.origin_cell - rows.astype(np.float32)) * local.resolution_m
        local_y = (cols.astype(np.float32) - local.origin_cell) * local.resolution_m
        return (
            self.pose.x_m + local_x * cy - local_y * sy,
            self.pose.y_m + local_x * sy + local_y * cy,
        )

    def _direct_extension_direction(
        self,
        wall_x: np.ndarray,
        wall_y: np.ndarray,
    ) -> Direction:
        corridor_half_w = self.grid.corridor_width_m * 0.5
        extension_min_y = self.grid.corridor_width_m * 0.625
        half = self.grid.corridor_width_m * 0.625 + 0.08
        center_y = self.grid.corridor_width_m
        decision = (
            (np.abs(wall_x) <= half)
            & (np.abs(wall_y - center_y) <= half)
            & (wall_y >= extension_min_y)
        )
        left_count = int(np.count_nonzero(
            decision & (wall_x <= -corridor_half_w + 0.08),
        ))
        right_count = int(np.count_nonzero(
            decision & (wall_x >= corridor_half_w - 0.08),
        ))
        minimum = max(3, self.front_wall_motion_min_points // 2)
        if left_count >= minimum and left_count >= right_count + 2:
            return Direction.RIGHT
        if right_count >= minimum and right_count >= left_count + 2:
            return Direction.LEFT
        return Direction.UNKNOWN

    def _in_direction_decision_section(
        self, x_m: float, y_m: float, margin: float = 0.0,
    ) -> bool:
        half = self.grid.corridor_width_m * 0.625
        center_y = self.grid.corridor_width_m
        return abs(x_m) <= half + margin and abs(y_m - center_y) <= half + margin

    def _update_direction_evidence(self, direction: Direction, raw_score: float) -> None:
        old = self.direction_evidence[direction]
        if not math.isfinite(raw_score):
            self.direction_evidence[direction] = old * 0.85
            return
        clipped = max(-0.4, min(raw_score, 1.0))
        self.direction_evidence[direction] = old * 0.72 + clipped * 0.28

    def _make_direction_hypothesis(self, direction: Direction) -> GridMap:
        hypothesis = GridMap(
            size_m=self.grid.spec.size_m,
            resolution_m=self.grid.resolution_m,
            corridor_width_m=self.grid.corridor_width_m,
            corridor_length_m=self.grid.corridor_length_m,
        )
        hypothesis.lock_direction(direction)
        return hypothesis

    def _score_direction_hypothesis(
        self,
        wall_x: np.ndarray,
        wall_y: np.ndarray,
        direction: Direction,
    ) -> float:
        hypothesis = self.direction_hypotheses[direction]
        half = self.grid.corridor_width_m * 0.625 + 0.08
        center_y = self.grid.corridor_width_m
        selected = (
            (np.abs(wall_x) <= half)
            & (np.abs(wall_y - center_y) <= half)
        )
        selected_x = wall_x[selected]
        selected_y = wall_y[selected]
        count = len(selected_x)
        if count < self.front_wall_motion_min_points:
            return -math.inf
        cols = np.rint(selected_x / hypothesis.resolution_m).astype(np.intp) + hypothesis.origin_cell
        rows = hypothesis.origin_cell - np.rint(selected_y / hypothesis.resolution_m).astype(np.intp)
        valid = (
            (rows >= 0) & (rows < hypothesis.size_cells)
            & (cols >= 0) & (cols < hypothesis.size_cells)
        )
        score = float(np.sum(
            self.direction_match_fields[direction][rows[valid], cols[valid]],
        ))
        return score / count

    def apply_odometry(
        self,
        linear_speed_mps: float,
        dt_s: float,
    ) -> float:
        if not math.isfinite(linear_speed_mps):
            linear_speed_mps = 0.0
        if not math.isfinite(dt_s) or dt_s <= 0.0 or dt_s > 0.5:
            self._last_odometry_speed_mps = linear_speed_mps
            self.pose.odometry_motion_m = 0.0
            self.pose.odometry_speed_mps = linear_speed_mps
            return 0.0

        previous_speed = self._last_odometry_speed_mps
        self._last_odometry_speed_mps = linear_speed_mps
        average_speed = (
            linear_speed_mps
            if previous_speed is None
            else 0.5 * (previous_speed + linear_speed_mps)
        )
        distance_m = average_speed * dt_s
        previous_yaw = self._base_pose[2]
        current_yaw = self.pose.yaw_rad
        travel_yaw = previous_yaw + 0.5 * angle_diff(current_yaw, previous_yaw)
        base_x = self._base_pose[0] + distance_m * math.cos(travel_yaw)
        base_y = self._base_pose[1] + distance_m * math.sin(travel_yaw)
        base_yaw = current_yaw
        self._base_pose = (base_x, base_y, base_yaw)
        self.pose.x_m = base_x + self._correction[0]
        self.pose.y_m = base_y + self._correction[1]
        self.pose.yaw_rad = base_yaw
        self._constrain_pose_to_map()
        motion_m = abs(distance_m)
        self.pose.odometry_motion_m = motion_m
        self.pose.odometry_speed_mps = linear_speed_mps
        self._odometry_travel_m += motion_m
        self.pose.odometry_travel_m = self._odometry_travel_m
        return motion_m

    def _constrain_pose_to_map(self) -> None:
        x_m, y_m = self.pose.x_m, self.pose.y_m
        margin = self.grid.resolution_m * 0.5 + 0.01
        half_w = self.grid.corridor_width_m * 0.5
        if self.grid.direction == Direction.UNKNOWN:
            constrained_x = max(-half_w + margin, min(half_w - margin, x_m))
            constrained_y = y_m
        else:
            x_min, x_max = (
                (-2.5, half_w)
                if self.grid.direction == Direction.LEFT
                else (-half_w, 2.5)
            )
            constrained_x = max(x_min + margin, min(x_max - margin, x_m))
            constrained_y = max(-1.5 + margin, min(1.5 - margin, y_m))
            center_x = (x_min + x_max) * 0.5
            box = (
                center_x - 0.5 - margin,
                -0.5 - margin,
                center_x + 0.5 + margin,
                0.5 + margin,
            )
            if box[0] < constrained_x < box[2] and box[1] < constrained_y < box[3]:
                exits = (
                    (constrained_x - box[0], (box[0], constrained_y)),
                    (box[2] - constrained_x, (box[2], constrained_y)),
                    (constrained_y - box[1], (constrained_x, box[1])),
                    (box[3] - constrained_y, (constrained_x, box[3])),
                )
                _, (constrained_x, constrained_y) = min(exits, key=lambda item: item[0])
        dx = constrained_x - x_m
        dy = constrained_y - y_m
        if abs(dx) < 1.0e-9 and abs(dy) < 1.0e-9:
            return
        self._rebase_translation(dx, dy)

    def apply_imu_yaw(self, yaw_rad: float) -> None:
        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = angle_diff(self.pose.yaw_rad, yaw_rad)
        yaw = yaw_rad + self._imu_yaw_offset
        self.pose.yaw_rad = math.atan2(math.sin(yaw), math.cos(yaw))

    def update_from_sensors(self, frame: SensorFrame) -> PoseEstimate:
        started = time.perf_counter()
        self._clear_correction_debug(frame)
        self._update_direction_lock(frame)
        direction_done = time.perf_counter()
        self._apply_scan_correction(frame)
        correction_done = time.perf_counter()
        self._remove_occluded_obstacles(frame)
        occlusion_done = time.perf_counter()
        self.grid.fuse_local(
            frame.local_grid,
            self.pose.x_m,
            self.pose.y_m,
            self.pose.yaw_rad,
            self.fuse_add_alpha,
            self.fuse_erode_alpha,
        )
        fusion_done = time.perf_counter()
        scan_done = time.perf_counter()
        self._profile_localize_ms = (
            (direction_done - started) * 1000.0,
            (correction_done - direction_done) * 1000.0,
            (occlusion_done - correction_done) * 1000.0,
            (fusion_done - occlusion_done) * 1000.0,
            (scan_done - fusion_done) * 1000.0,
        )
        return self.pose

    def _remove_occluded_obstacles(self, frame: SensorFrame) -> None:
        local = frame.local_grid
        dynamic = (
            (local.cells == int(Cell.UNKNOWN_OBSTRUCTION))
            | (local.cells == int(Cell.LIVE_OBSTACLE))
        )
        rows, cols = np.nonzero(dynamic)
        if not len(rows):
            return
        local_x = (local.origin_cell - rows.astype(np.float32)) * local.resolution_m
        local_y = (cols.astype(np.float32) - local.origin_cell) * local.resolution_m
        endpoint_range = np.hypot(local_x, local_y)
        cy, sy = math.cos(self.pose.yaw_rad), math.sin(self.pose.yaw_rad)
        world_x = self.pose.x_m + local_x * cy - local_y * sy
        world_y = self.pose.y_m + local_x * sy + local_y * cy
        behind_bounds = np.fromiter(
            (
                not self.grid.inside_navigable_bounds(float(x_m), float(y_m))
                for x_m, y_m in zip(world_x, world_y)
            ),
            dtype=np.bool_,
            count=len(rows),
        )

        step_m = max(self.grid.resolution_m * 0.5, 0.01)
        max_steps = int(math.ceil(float(endpoint_range.max(initial=0.0)) / step_m))
        behind_wall = np.zeros(len(rows), dtype=np.bool_)
        if max_steps:
            distance = np.arange(1, max_steps + 1, dtype=np.float32) * step_m
            usable = distance[None, :] < endpoint_range[:, None] - 0.10
            unit_x = np.divide(
                world_x - self.pose.x_m,
                endpoint_range,
                out=np.zeros_like(world_x),
                where=endpoint_range > 1.0e-6,
            )
            unit_y = np.divide(
                world_y - self.pose.y_m,
                endpoint_range,
                out=np.zeros_like(world_y),
                where=endpoint_range > 1.0e-6,
            )
            sample_x = self.pose.x_m + unit_x[:, None] * distance[None, :]
            sample_y = self.pose.y_m + unit_y[:, None] * distance[None, :]
            sample_cols = (
                np.rint(sample_x / self.grid.resolution_m).astype(np.intp)
                + self.grid.origin_cell
            )
            sample_rows = (
                self.grid.origin_cell
                - np.rint(sample_y / self.grid.resolution_m).astype(np.intp)
            )
            on_grid = (
                (sample_rows >= 0) & (sample_rows < self.grid.size_cells)
                & (sample_cols >= 0) & (sample_cols < self.grid.size_cells)
            )
            clipped_rows = np.clip(sample_rows, 0, self.grid.size_cells - 1)
            clipped_cols = np.clip(sample_cols, 0, self.grid.size_cells - 1)
            wall = self.grid.cells[clipped_rows, clipped_cols] == int(Cell.MAP_WALL)
            behind_wall = np.any(usable & on_grid & wall, axis=1)
        remove = (endpoint_range > 0.05) & (behind_bounds | behind_wall)
        local.cells[rows[remove], cols[remove]] = int(Cell.FREE)

    def _clear_correction_debug(self, frame: SensorFrame) -> None:
        self.pose.front_wall_distance_m = frame.front_wall.distance_m
        self.pose.front_wall_points = frame.front_wall.point_count
        self.pose.front_wall_motion_m = 0.0
        self.pose.wall_pose_correction_x_m = 0.0
        self.pose.wall_pose_correction_y_m = 0.0
        self.pose.wall_pose_sources = 0
        self.pose.scan_motion_correction_x_m = 0.0
        self.pose.scan_motion_correction_y_m = 0.0
        self.pose.scan_motion_score = 0.0

    def _apply_scan_correction(self, frame: SensorFrame) -> None:
        started = time.perf_counter()
        startup = self.grid.direction == Direction.UNKNOWN and self._odometry_travel_m <= 0.15
        points, normal_axes = scan_world_walls(
            frame,
            self._base_pose[0],
            self._base_pose[1],
            self.pose.yaw_rad,
        )
        points_done = time.perf_counter()
        if len(points) < 6:
            self.pose.confidence = 0.0
            return
        supports_x = 0 in normal_axes
        supports_y = 1 in normal_axes
        if startup:
            supports_x = supports_y = True
        axes_done = time.perf_counter()
        coarse_step = 0.05
        prior_x, prior_y = self._correction
        coarse_radius = 6 if startup else 2
        coarse_candidates = {
            (prior_x + ix * coarse_step, prior_y + iy * coarse_step)
            for ix in range(-coarse_radius, coarse_radius + 1)
            for iy in range(-coarse_radius, coarse_radius + 1)
        }
        if not startup:
            axis_radius = max(1, int(self.correction_max_m / coarse_step))
            if supports_x:
                coarse_candidates.update(
                    (prior_x + ix * coarse_step, prior_y)
                    for ix in range(-axis_radius, axis_radius + 1)
                )
            if supports_y:
                coarse_candidates.update(
                    (prior_x, prior_y + iy * coarse_step)
                    for iy in range(-axis_radius, axis_radius + 1)
                )
        coarse_offsets = self._valid_correction_offsets(
            sorted(coarse_candidates)
        )
        absolute_penalty = 0.25 if startup else 0.03
        temporal_penalty = 0.25 if startup else 0.06
        coarse = best_map_shift(
            points,
            normal_axes,
            self.grid,
            coarse_offsets,
            offset_penalty=absolute_penalty,
            prior_offset=self._correction,
            prior_penalty=temporal_penalty,
        )
        if coarse is None:
            self.pose.confidence = 0.0
            return
        fine_step = 0.01
        fine_radius = 4
        fine_offsets = {
            (
                coarse[0] + ix * fine_step,
                coarse[1] + iy * fine_step,
            )
            for ix in range(-fine_radius, fine_radius + 1)
            for iy in range(-fine_radius, fine_radius + 1)
        }
        fine_offsets.add(self._correction)
        valid_fine_offsets = self._valid_correction_offsets(sorted(fine_offsets))
        best = best_map_shift(
            points,
            normal_axes,
            self.grid,
            valid_fine_offsets,
            offset_penalty=absolute_penalty,
            prior_offset=self._correction,
            prior_penalty=temporal_penalty,
        )
        match_done = time.perf_counter()
        self._profile_scan_correction_ms = (
            (points_done - started) * 1000.0,
            (axes_done - points_done) * 1000.0,
            (match_done - axes_done) * 1000.0,
        )
        if best is None:
            self.pose.confidence = 0.0
            return
        gain = best[2] - best[3]
        self.pose.alignment_score = gain
        self.pose.confidence = max(0.0, min(gain / 0.20, 1.0))
        if not startup and self.pose.odometry_motion_m < 0.003:
            return
        min_gain = 0.035 if startup else 0.008
        update_len = math.hypot(
            best[0] - self._correction[0],
            best[1] - self._correction[1],
        )
        if update_len > 0.04:
            evidence_scale = 0.70 if startup else (0.35 if supports_x or supports_y else 1.0)
            min_gain = max(
                min_gain,
                (0.08 if startup else 0.11) + (update_len - 0.04) * evidence_scale,
            )
        if gain < min_gain or update_len < 0.005:
            return
        correction_gain = 1.0 if startup else 0.65
        motion_step_limit_m = (
            0.15
            if startup
            else min(0.03, max(0.004, self.pose.odometry_motion_m * 0.35))
        )
        dx = (best[0] - self._correction[0]) * correction_gain
        dy = (best[1] - self._correction[1]) * correction_gain
        strong_axis_match = gain >= 0.20 and update_len >= 0.10
        supported_step_limit_m = min(
            0.10 if strong_axis_match else 0.05,
            max(0.004, self.pose.odometry_motion_m * (1.0 if strong_axis_match else 0.50)),
        )
        x_limit = supported_step_limit_m if supports_x and not startup else motion_step_limit_m
        y_limit = supported_step_limit_m if supports_y and not startup else motion_step_limit_m
        dx = max(-x_limit, min(x_limit, dx))
        dy = max(-y_limit, min(y_limit, dy))
        corr_x = self._correction[0] + dx
        corr_y = self._correction[1] + dy
        corr_len = math.hypot(corr_x, corr_y)
        if corr_len > self.correction_max_m and corr_len > 1.0e-6:
            corr_x *= self.correction_max_m / corr_len
            corr_y *= self.correction_max_m / corr_len
        applied_x = corr_x - self._correction[0]
        applied_y = corr_y - self._correction[1]
        self._correction = (corr_x, corr_y)
        self.pose.x_m = self._base_pose[0] + corr_x
        self.pose.y_m = self._base_pose[1] + corr_y
        self.pose.wall_pose_correction_x_m = applied_x
        self.pose.wall_pose_correction_y_m = applied_y
        self.pose.wall_pose_sources = len(points)
        self.pose.scan_motion_correction_x_m = applied_x
        self.pose.scan_motion_correction_y_m = applied_y
        self.pose.scan_motion_score = gain
        if not startup and math.hypot(*self._correction) >= self.correction_max_m * 0.5:
            self._rebase_translation(0.0, 0.0)

    def _rebase_translation(self, dx: float, dy: float) -> None:
        shift_x = self._correction[0] + dx
        shift_y = self._correction[1] + dy
        if abs(shift_x) < 1.0e-9 and abs(shift_y) < 1.0e-9:
            return
        self._base_pose = (
            self._base_pose[0] + shift_x,
            self._base_pose[1] + shift_y,
            self._base_pose[2],
        )
        self._total_correction = (
            self._total_correction[0] + shift_x,
            self._total_correction[1] + shift_y,
        )
        self._correction = (0.0, 0.0)
        self.pose.x_m = self._base_pose[0]
        self.pose.y_m = self._base_pose[1]

    def _valid_correction_offsets(
        self,
        offsets: Iterable[tuple[float, float]],
    ) -> tuple[tuple[float, float], ...]:
        candidates = tuple(offsets)
        if not candidates:
            return ()
        values = np.asarray(candidates, dtype=np.float64)
        valid = np.hypot(values[:, 0], values[:, 1]) <= self.correction_max_m + 1.0e-9

        half_length = 0.22 * 0.5 + 0.01
        half_width = 0.15 * 0.5 + 0.01
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        forward = np.array((-half_length, -half_length, half_length, half_length))
        lateral = np.array((-half_width, half_width, -half_width, half_width))
        corner_x = (
            self._base_pose[0]
            + values[:, 0, None]
            + forward * cy
            - lateral * sy
        )
        corner_y = (
            self._base_pose[1]
            + values[:, 1, None]
            + forward * sy
            + lateral * cy
        )
        inside = (corner_y >= -1.5) & (corner_y <= 1.5)
        half_w = self.grid.corridor_width_m * 0.5
        if self.grid.direction == Direction.LEFT:
            x_min, x_max = -2.5, half_w
        elif self.grid.direction == Direction.RIGHT:
            x_min, x_max = -half_w, 2.5
        else:
            x_min, x_max = -half_w, half_w
        inside &= (corner_x >= x_min) & (corner_x <= x_max)
        if self.grid.direction != Direction.UNKNOWN:
            center_x = (x_min + x_max) * 0.5
            inside &= ~(
                (corner_x > center_x - 0.5)
                & (corner_x < center_x + 0.5)
                & (corner_y > -0.5)
                & (corner_y < 0.5)
            )
        valid &= np.all(inside, axis=1)
        return tuple(
            (float(offset_x), float(offset_y))
            for (offset_x, offset_y), keep in zip(candidates, valid)
            if keep
        )

    def _footprint_inside_navigable_bounds(self, x_m: float, y_m: float) -> bool:
        half_length = 0.22 * 0.5 + 0.01
        half_width = 0.15 * 0.5 + 0.01
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        for forward in (-half_length, half_length):
            for lateral in (-half_width, half_width):
                corner_x = x_m + forward * cy - lateral * sy
                corner_y = y_m + forward * sy + lateral * cy
                if not self.grid.inside_navigable_bounds(corner_x, corner_y):
                    return False
        return True
